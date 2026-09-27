"""Transport-level behaviour of the ER client: what it retries, and what it doesn't.

Both cases here were paid for in wall-clock. A run with no route to the API hung
on one call for fifteen minutes because `budgets.llm_timeout_s` was defined and
never passed to the client; and when the route came back, a depleted prepaid
balance returned 429 — the same status as rate limiting — so every call spent ten
seconds in backoff before surfacing the one line that said to go top up.
"""
from __future__ import annotations

import time

import pytest

from heron.orchestrator.gemini import _is_permanent


DEPLETED = ("Error code: 429 - {'error': {'message': 'Your prepayment credits are "
            "depleted. Please go to AI Studio at https://ai.studio/projects to manage "
            "your project and billing.', 'code': 'too_many_requests'}}")


@pytest.mark.parametrize("msg", [
    DEPLETED,
    "429 RESOURCE_EXHAUSTED: Quota exceeded for this project",
    "400 API key not valid. Please pass a valid API key.",
    "403 PERMISSION_DENIED",
])
def test_permanent_errors_are_not_retried(msg):
    assert _is_permanent(msg.lower()), f"retrying cannot fix: {msg[:60]}"


@pytest.mark.parametrize("msg", [
    "429 Too Many Requests: rate limit exceeded, retry shortly",
    "503 Service Unavailable",
    "504 deadline exceeded",
    "500 internal error",
    "The model is overloaded. Please try again later.",
])
def test_transient_errors_stay_retryable(msg):
    assert not _is_permanent(msg.lower()), f"should still be retried: {msg[:60]}"


def test_call_gives_up_immediately_on_a_depleted_balance():
    """The point is elapsed time: three retries here would cost ten seconds."""
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator

    pytest.importorskip("google.genai")
    cfg = HeronConfig()
    orch = GeminiOrchestrator.__new__(GeminiOrchestrator)  # skip the network client
    orch.cfg = cfg
    orch.log = EpisodeLogger("/tmp/heron-test-episodes", "transport")
    orch._use_interactions = False
    orch._transport_down = None
    orch._transport_down_at = 0.0
    orch._transport_retry_after_s = 60.0
    orch._transport_patient = False      # sweep default; the rig sets it via env
    orch._ever_connected = False
    orch._unreachable_streak = 0

    def boom(*a, **kw):
        raise RuntimeError(DEPLETED)

    orch._call_generate = boom
    t0 = time.perf_counter()
    with pytest.raises(RuntimeError, match="depleted"):
        orch._call("box", "text", [], "minimal")
    assert time.perf_counter() - t0 < 1.0, "a depleted balance must not be backed off"


def test_timeout_is_taken_from_the_budget_and_sent_in_milliseconds():
    from heron.orchestrator.gemini import _client_with_timeout

    seen = {}

    class FakeClient:
        def __init__(self, http_options=None):
            seen["http_options"] = http_options

    class FakeGenai:
        Client = FakeClient

    _client_with_timeout(FakeGenai, 90.0)
    opts = seen["http_options"]
    # google-genai takes milliseconds; passing seconds would set a 90 ms timeout
    # and fail every call instead of hanging on them — the opposite bug.
    assert getattr(opts, "timeout", None) == 90_000


def test_a_client_that_rejects_http_options_still_starts():
    from heron.orchestrator.gemini import _client_with_timeout

    class Picky:
        def __init__(self, http_options=None):
            if http_options is not None:
                raise TypeError("this SDK version has no http_options")

    class FakeGenai:
        Client = Picky

    assert _client_with_timeout(FakeGenai, 90.0) is not None


UNREACHABLE_ERR = "APIConnectionError: [Errno 101] Network is unreachable"


def _bare_orchestrator(raises: str):
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator

    orch = GeminiOrchestrator.__new__(GeminiOrchestrator)  # skip the network client
    orch.cfg = HeronConfig()
    orch.log = EpisodeLogger("/tmp/heron-test-episodes", "transport")
    orch._use_interactions = False
    orch._transport_down = None
    orch._transport_down_at = 0.0
    orch._transport_retry_after_s = 60.0
    orch._transport_patient = False      # sweep default; the rig sets it via env
    orch._ever_connected = False
    orch._unreachable_streak = 0

    def boom(*a, **kw):
        raise RuntimeError(raises)

    orch._call_generate = boom
    return orch


def test_an_absent_network_costs_one_backoff_not_one_per_call():
    """Measured on the cluster with its proxy down: every call retried four times
    behind a quadratic backoff, and an episode makes a dozen calls — so each of
    18 LIBERO tasks took 727 seconds to arrive at "network is unreachable", and
    the sweep would have spent three and a half hours discovering the same fact
    eighteen times.

    So the quantity to bound is the cost PER SWEEP, not the latency of one
    call. The first call of a process keeps its in-call retries — an instant
    latch condemned whole rig episodes over a one-off Errno 113 that cleared a
    minute later (measured twice, 2026-08-17) — but once the route is latched,
    every later call is refused without touching the network."""
    import time

    orch = _bare_orchestrator(UNREACHABLE_ERR)
    t0 = time.time()
    with pytest.raises(Exception):
        orch._call("plan", "hello", [], "low")
    assert time.time() - t0 < 20.0, "one call's worth of backoff, not a pile-up"

    t0 = time.time()
    with pytest.raises(RuntimeError, match="no route to the model"):
        orch._call("plan", "hello", [], "low")
    assert time.time() - t0 < 1.0, "a latched route is refused without a probe"


def test_once_the_route_is_gone_the_rest_of_the_run_stops_asking():
    """The run is already lost by the time this latches. A half-finished sweep
    that quietly resumed on call two hundred would be worse than one that stops
    and says why — so the flag is one-way and per-process."""
    orch = _bare_orchestrator(UNREACHABLE_ERR)
    with pytest.raises(Exception):
        orch._call("plan", "hello", [], "low")
    calls = []

    def counted(*a, **kw):
        calls.append(1)
        return "{}"

    orch._call_generate = counted
    with pytest.raises(RuntimeError, match="no route to the model"):
        orch._call("locate", "hello", [], "low")
    assert calls == [], "it must not even attempt the request"


def test_a_transient_server_error_is_still_retried():
    """The distinction has to stay a distinction: a 503 is the server having a
    moment and does deserve the backoff."""
    orch = _bare_orchestrator("503 Service Unavailable")
    tries = []

    def flaky(*a, **kw):
        tries.append(1)
        if len(tries) < 2:
            raise RuntimeError("503 Service Unavailable")
        return "{}"

    orch._call_generate = flaky
    assert orch._call("locate", "hello", [], "low") == "{}"
    assert len(tries) == 2
    assert orch._transport_down is None


def test_no_model_call_starts_after_the_deadline():
    """The budget used to be consulted only between steps, and a step is not the
    unit of cost — a model call is. Measured: a 360 s episode ended at 487 s,
    with six calls issued after the deadline."""
    orch = _bare_orchestrator("irrelevant")
    calls = []

    def counted(*a, **kw):
        calls.append(1)
        return "{}"

    orch._call_generate = counted
    orch.deadline = time.time() - 5.0
    with pytest.raises(RuntimeError, match="deadline"):
        orch._call("locate", "hello", [], "low")
    assert calls == [], "it must not even attempt the request"


def test_a_retry_does_not_sleep_past_the_deadline():
    """One locate call spent 120.9 s inside its own retry backoff and sailed
    through the deadline mid-call. A retry that would wake up past the deadline
    is not a retry, it is a slower way to fail."""
    orch = _bare_orchestrator("503 Service Unavailable")
    orch.deadline = time.time() + 1.0   # the first backoff (2 s) would overshoot
    t0 = time.time()
    with pytest.raises(RuntimeError, match="503"):
        orch._call("locate", "hello", [], "low")
    assert time.time() - t0 < 1.5, "it must fail now, not after the backoff"


import time  # noqa: E402


# -- the OpenAI-compatible relay path (models.api_format: openai) --------------
#
# The pp-api relay serves gemini-3.5-flash behind POST /chat/completions with
# Bearer auth. Verified 2026-08-03 that it is NOT Gemini-native: google-genai
# pointed at it 404s with "Invalid URL (POST /v1/v1beta/...)". So it gets its
# own transport, and these tests pin the request shape and check that the
# shared _call discipline — retry, latch, deadline — applies to it unchanged.

import json  # noqa: E402

import numpy as np  # noqa: E402


class _FakeResponse:
    def __init__(self, status_code=200, payload=None, text=""):
        self.status_code = status_code
        self._payload = payload
        self.text = text

    def json(self):
        return self._payload


_OK = {"choices": [{"message": {"content": "[]"}}]}


class _FakeHttp:
    """Feeds canned responses (or raises canned exceptions) and records requests."""

    def __init__(self, *responses):
        self.responses = list(responses)
        self.requests: list = []

    def post(self, url, json=None):
        self.requests.append((url, json))
        r = self.responses.pop(0)
        if isinstance(r, Exception):
            raise r
        return r


def _relay_orchestrator(http):
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator

    orch = GeminiOrchestrator.__new__(GeminiOrchestrator)  # skip the real client
    orch.cfg = HeronConfig()
    orch.log = EpisodeLogger("/tmp/heron-test-episodes", "transport")
    orch.model = "gemini-3.5-flash"
    orch._use_interactions = False
    orch._transport_down = None
    orch._transport_down_at = 0.0
    orch._transport_retry_after_s = 60.0
    orch._transport_patient = False      # sweep default; the rig sets it via env
    orch._ever_connected = False
    orch._unreachable_streak = 0
    orch._lens_cache = {}
    orch._http = http
    return orch


def _frame():
    from heron.types import Frame

    return Frame(camera="cam_high", rgb=np.zeros((8, 8, 3), dtype=np.uint8))


def test_openai_client_is_built_from_named_env_vars(monkeypatch, tmp_path):
    """Neither the relay URL nor the key may land in a committed YAML, so the
    config carries env-var NAMES and the client resolves them at startup."""
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator

    monkeypatch.setenv("FAKE_RELAY_URL", "https://relay.example/v1")
    monkeypatch.setenv("FAKE_RELAY_KEY", "sk-test-not-a-real-key")
    cfg = HeronConfig.model_validate({"models": {
        "api_format": "openai", "er_model": "gemini-3.5-flash",
        "base_url_env": "FAKE_RELAY_URL", "api_key_env": "FAKE_RELAY_KEY"}})
    orch = GeminiOrchestrator(cfg, EpisodeLogger(str(tmp_path), "transport"))
    try:
        assert orch.client is None, "no google-genai client on the relay path"
        assert str(orch._http.base_url).startswith("https://relay.example/v1")
        assert orch._http.headers["authorization"] == "Bearer sk-test-not-a-real-key"
    finally:
        orch._http.close()


def test_openai_client_refuses_to_start_misconfigured(monkeypatch, tmp_path):
    """A missing key must fail at startup, not as a 401 twelve calls into an episode."""
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator

    monkeypatch.delenv("FAKE_RELAY_KEY", raising=False)
    monkeypatch.setenv("FAKE_RELAY_URL", "https://relay.example/v1")
    log = EpisodeLogger(str(tmp_path), "transport")
    with pytest.raises(RuntimeError, match="key"):
        GeminiOrchestrator(HeronConfig.model_validate({"models": {
            "api_format": "openai", "base_url_env": "FAKE_RELAY_URL",
            "api_key_env": "FAKE_RELAY_KEY"}}), log)
    with pytest.raises(RuntimeError, match="base URL"):
        GeminiOrchestrator(HeronConfig.model_validate({"models": {
            "api_format": "openai", "api_key_env": "FAKE_RELAY_URL"}}), log)


def test_explicit_base_url_wins_over_the_env_name(monkeypatch):
    from heron.config import ModelCfg

    monkeypatch.setenv("FAKE_RELAY_URL", "https://from-env.example")
    assert ModelCfg(base_url="https://explicit.example",
                    base_url_env="FAKE_RELAY_URL").resolved_base_url() == "https://explicit.example"
    assert ModelCfg(base_url_env="FAKE_RELAY_URL").resolved_base_url() == "https://from-env.example"
    assert ModelCfg().resolved_base_url() == ""


def test_relay_request_carries_frames_as_data_urls_then_text():
    http = _FakeHttp(_FakeResponse(payload=_OK))
    orch = _relay_orchestrator(http)
    out = orch._call("point", "Point to the block.", [_frame()], "minimal")
    assert out == "[]"
    url, body = http.requests[0]
    assert url.endswith("chat/completions")
    assert body["model"] == "gemini-3.5-flash"
    content = body["messages"][0]["content"]
    assert content[0]["type"] == "image_url"
    assert content[0]["image_url"]["url"].startswith("data:image/png;base64,")
    assert content[-1] == {"type": "text", "text": "Point to the block."}
    assert "response_format" not in body, "grounding answers are JSON LISTS; " \
        "json_object mode would forbid them"


def test_relay_schema_is_stated_in_the_prompt_and_json_mode_pinned():
    """The relay does not enforce response schemas the way Gemini does, so the
    schema rides in the prompt and the lenient parse does the enforcing."""
    http = _FakeHttp(_FakeResponse(payload=_OK))
    orch = _relay_orchestrator(http)
    schema = {"type": "object", "properties": {"steps": {"type": "array"}}}
    orch._call("plan", "Plan the task.", [], "high", schema=schema)
    _, body = http.requests[0]
    assert body["response_format"] == {"type": "json_object"}
    text = body["messages"][0]["content"][-1]["text"]
    assert text.startswith("Plan the task.")
    assert json.dumps(schema) in text


def test_relay_503_is_retried():
    http = _FakeHttp(_FakeResponse(status_code=503, text="Service Unavailable"),
                     _FakeResponse(payload=_OK))
    orch = _relay_orchestrator(http)
    assert orch._call("locate", "hello", [], "minimal") == "[]"
    assert len(http.requests) == 2
    assert orch._transport_down is None


def test_relay_timeout_is_transient_not_permanent():
    """str(ReadTimeout()) is empty — the exception CLASS name has to carry the
    word the retry logic matches on."""
    import httpx

    http = _FakeHttp(httpx.ReadTimeout(""), _FakeResponse(payload=_OK))
    orch = _relay_orchestrator(http)
    assert orch._call("locate", "hello", [], "minimal") == "[]"
    assert len(http.requests) == 2


def test_relay_connection_refused_latches_the_transport():
    """The unreachable latch is transport-agnostic: a dead relay must stop the
    run exactly like a dead route to Google does."""
    import httpx

    # The first call of a process spends its in-call retries before latching
    # (see test_an_absent_network_costs_one_backoff_not_one_per_call), so the
    # fake has to be able to refuse more than once.
    http = _FakeHttp(*[httpx.ConnectError("[Errno 61] Connection refused")] * 8)
    orch = _relay_orchestrator(http)
    with pytest.raises(RuntimeError, match="[Cc]onnection refused"):
        orch._call("plan", "hello", [], "low")
    after = _FakeHttp(_FakeResponse(payload=_OK))
    orch._http = after
    with pytest.raises(RuntimeError, match="no route to the model"):
        orch._call("locate", "hello", [], "low")
    assert after.requests == [], "it must not even attempt the request"


def test_relay_depleted_balance_is_not_backed_off():
    """Same money lesson as the Google path: a relay 429 that means 'top up'
    must surface now, not after ten seconds of backoff."""
    http = _FakeHttp(_FakeResponse(status_code=429,
                                   text="Your prepayment credits are depleted."))
    orch = _relay_orchestrator(http)
    t0 = time.perf_counter()
    with pytest.raises(RuntimeError, match="depleted"):
        orch._call("box", "text", [], "minimal")
    assert time.perf_counter() - t0 < 1.0
    assert len(http.requests) == 1


def test_parse_json_takes_the_first_value_and_ignores_the_commentary():
    """Paid for on the relay's first twin episode: flash (no server-side schema
    enforcement, unlike ER) emitted a valid repair object and kept talking, the
    slice-to-last-bracket fallback swept the commentary's brackets into the
    slice, and 'Extra data' aborted a 92-second episode at the finish line."""
    from heron.orchestrator.gemini import _parse_json

    assert _parse_json('{"type": "retry_step"}\nNote: [1] this retries.') == \
        {"type": "retry_step"}
    assert _parse_json('Here you go:\n[{"point": [5, 5]}]\n(box omitted)') == \
        [{"point": [5, 5]}]
    assert _parse_json('```json\n{"a": 1}\n```') == {"a": 1}
    with pytest.raises(json.JSONDecodeError):
        _parse_json("no json here at all")


def test_relay_no_completion_shape_is_a_plain_error():
    http = _FakeHttp(_FakeResponse(payload={"error": {"message": "wat"}}))
    orch = _relay_orchestrator(http)
    with pytest.raises(RuntimeError, match="no completion"):
        orch._call("vqa", "hello", [], "low")


def test_a_route_that_has_worked_survives_one_refused_connection():
    """The latch was measured against a proxy that was down for a whole sweep,
    and it is right for that. It is wrong for a relay that refuses one TCP
    connection under load: pp-api did exactly that 110 s into a bimanual
    episode and answered normally seconds later, and the episode was declared
    unsalvageable anyway. Whether this process has ever reached the model is
    the evidence that tells an absent network from a busy one."""
    orch = _bare_orchestrator("")
    tries = []

    def flaky(*a, **kw):
        tries.append(1)
        if len(tries) == 2:
            raise RuntimeError("[Errno 61] Connection refused")
        return "{}"

    orch._call_generate = flaky
    assert orch._call("locate", "hello", [], "low") == "{}"   # the route works
    assert orch._call("locate", "hello", [], "low") == "{}"   # refused, then retried
    assert orch._transport_down is None
    assert len(tries) == 3


def test_a_route_that_keeps_refusing_still_latches():
    """Three in a row is not noise. A genuinely dead proxy produces far more."""
    orch = _bare_orchestrator("")
    tries = []

    def once_then_dead(*a, **kw):
        tries.append(1)
        if len(tries) > 1:
            raise RuntimeError("[Errno 61] Connection refused")
        return "{}"

    orch._call_generate = once_then_dead
    assert orch._call("locate", "hello", [], "low") == "{}"
    with pytest.raises(RuntimeError, match="[Cc]onnection refused"):
        orch._call("locate", "hello", [], "low")
    assert orch._transport_down is not None
    with pytest.raises(RuntimeError, match="no route to the model"):
        orch._call("locate", "hello", [], "low")
