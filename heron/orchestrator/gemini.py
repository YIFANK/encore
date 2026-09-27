"""Gemini Robotics-ER 2 as the single brain: grounding, planning, and repair.

Model interaction principles (deliberately different from Pigey):
- STATELESS TYPED CALLS. Each decision is one schema-constrained call carrying
  the current belief/program rendering and fresh frames — no accumulating chat,
  no context-window decay, and every call is individually replayable from the
  journal.
- COST TIERING. Spatial queries (point/box/vqa) run at minimal thinking;
  planning and repair run higher (`thinking_level` per ER 2's API).
- LENIENT PARSING. Coordinates auto-detect 0-1000-normalized vs pixel space
  (Gemini models drift between them; Pigey shipped the same guard).

Primary path is the Interactions API (current robotics docs); falls back to
generate_content for older google-genai installs.
"""
from __future__ import annotations

import io
import json
import os
import re
import time
from typing import Any, Literal, Optional

import numpy as np
from PIL import Image
from pydantic import BaseModel, Field, ValidationError

from ..config import HeronConfig
from ..episode import EpisodeLogger
from ..program import EditType, ProgramEdit, Step, TaskProgram
from ..types import EntityDecl, FailureReport, Frame, PredicateSpec, SpatialRelation, Value
from .. import prompts

# Transient by nature. The connection-level entries were added after three
# episodes of a six-episode run died outright on
# "[SSL: UNEXPECTED_EOF_WHILE_READING]" — a dropped socket, raised as
# APIConnectionError, matched none of the status codes and so was treated as a
# permanent failure of the task rather than of one request.
RETRYABLE = ("429", "500", "502", "503", "504", "timeout", "deadline", "unavailable",
             "overloaded", "connection", "ssl", "eof occurred", "reset by peer",
             "broken pipe", "temporarily unavailable", "handshake")
# How far apart two frames may be stamped and still belong to the same look.
# `_batched_look` takes every camera's shutter before asking anything, so a
# look's frames land within milliseconds of each other; the next look is
# separated by at least one model call, which is seconds.
LOOK_TOGETHER_S = 0.5
PREDICATES = ("holding", "gripper_empty", "visible", "in", "on", "open", "near", "all_in")


# -- structured-output DTOs (closed schemas; Gemini rejects open dicts) -------

class PredicateOut(BaseModel):
    name: Literal["holding", "gripper_empty", "visible", "in", "on", "open", "near", "all_in"]
    args: list[str]
    negated: bool = False

    def to_spec(self) -> PredicateSpec:
        return PredicateSpec(name=self.name, args=self.args, negated=self.negated)


class ArgsOut(BaseModel):
    """Union of all skill parameters; None fields are dropped on conversion.

    A CLOSED schema is the price of typed plans, and the bill comes due when a
    skill is added: the planner reasoned its way to lift_and_peek(cup, block)
    unprompted on the hide-and-seek scene, and every argument was silently
    dropped here because this class had never heard of `occluder`. Any new
    skill parameter must be listed, or the skill is unusable however well the
    model reasons about it. (A test guards this — see test_skills.py.)
    """

    entity: Optional[str] = None
    target: Optional[str] = None
    toward: Optional[str] = None
    cabinet: Optional[str] = None
    distance: Optional[float] = None
    arm: Optional[str] = None
    to_arm: Optional[str] = None
    x: Optional[float] = None
    y: Optional[float] = None
    dx: Optional[float] = None
    dy: Optional[float] = None
    entities: Optional[list[str]] = None
    cameras: Optional[list[str]] = None
    instruction: Optional[str] = None
    max_seconds: Optional[float] = None
    strategy: Optional[str] = None
    occluder: Optional[str] = None
    look_for: Optional[str] = None
    contact: Optional[bool] = None

    def to_dict(self) -> dict[str, Any]:
        return {k: v for k, v in self.model_dump().items() if v is not None}


class StepOut(BaseModel):
    skill: str
    args: ArgsOut = Field(default_factory=ArgsOut)
    pre: list[PredicateOut] = Field(default_factory=list)
    post: list[PredicateOut] = Field(default_factory=list)
    # Optional guard: run this step only when the predicate holds. A guard is
    # not a precondition — false skips the step instead of raising a fault.
    when: Optional[PredicateOut] = None
    rationale: str = ""


class RelationOut(BaseModel):
    """Where the object is, when what it looks like cannot tell it from its twins.

    A closed schema on purpose: `kind` is an enum the executor can actually
    compute, so a relation it cannot honour is rejected by the model's own
    decoder rather than accepted and quietly dropped.
    """

    kind: Literal["between", "near", "far_from", "on", "inside"]
    of: list[str] = Field(default_factory=list)
    negated: bool = False


class EntityOut(BaseModel):
    id: str
    description: str
    role: str = "object"
    relation: Optional[RelationOut] = None


class PlanOut(BaseModel):
    entities: list[EntityOut]
    steps: list[StepOut]
    goal_predicates: list[PredicateOut]
    # Optional program-level while: after the steps complete, this predicate is
    # verified; unsatisfied re-runs the steps. For "every"/"all" tasks.
    repeat_until: Optional[PredicateOut] = None


class RepairOut(BaseModel):
    type: Literal["retry_step", "insert_steps", "replace_step", "skip_step",
                  "reground_entity", "replace_tail", "abort"]
    step_id: Optional[str] = None
    new_args: Optional[ArgsOut] = None
    steps: list[StepOut] = Field(default_factory=list)
    entity_id: Optional[str] = None
    new_description: Optional[str] = None
    rationale: str = ""


def _parse_json(text: str) -> Any:
    text = text.strip()
    fenced = re.search(r"```(?:json)?\s*(.*?)```", text, re.DOTALL)
    if fenced:
        text = fenced.group(1).strip()
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        start = min((i for i in (text.find("["), text.find("{")) if i >= 0), default=-1)
        if start < 0:
            raise
        # The FIRST complete JSON value; whatever follows is commentary. The old
        # slice-to-last-bracket handled prose around the value but died on
        # "Extra data" whenever the tail itself contained a bracket — which is
        # how a relay repair call ended an episode: flash (no server-side schema
        # enforcement, unlike ER) emitted a valid object and kept talking.
        value, _ = json.JSONDecoder().raw_decode(text[start:])
        return value


def _to_px(y: float, x: float, w: int, h: int) -> tuple[int, int]:
    """ER convention: [y, x] normalized 0-1000. Auto-detect raw pixel outputs."""
    if x > 1000 or y > 1000:
        return int(np.clip(x, 0, w - 1)), int(np.clip(y, 0, h - 1))
    return int(np.clip(x / 1000.0 * (w - 1), 0, w - 1)), int(np.clip(y / 1000.0 * (h - 1), 0, h - 1))


# How much of the frame outside the reachable workspace to keep, as a fraction
# of the workspace's own size on screen. Enough that an object sitting on the
# edge is not cut in half, and enough context that the scene still reads as a
# table rather than a swatch.
LENS_MARGIN = 0.12
# Below this the crop is not believable — a bad extrinsic can project the
# workspace to a handful of pixels, and cropping to that would be worse than not
# cropping at all.
LENS_MIN_PX = 64
# Above this there is nothing to win: the workspace already fills the frame.
LENS_MAX_COVERAGE = 0.7
# What the shorter side is upscaled toward. Same reasoning, and the same number,
# as `sensing.IDENTIFY_CROP_MIN_PX`: a vision model reading a 22 px block is
# being asked to identify something smaller than its own patch.
LENS_TARGET_PX = 448
LENS_MAX_UPSCALE = 4
# How far above the table an object can be and still be worth framing. Enough
# for a block on a tray rim or stacked on another; deliberately NOT enough for a
# carried one. The camera is oblique, so every extra centimetre of height throws
# the crop further out — at 0.25 m the box is 36% of the frame, at 0.10 m it is
# 21% — and an object in the gripper is the one thing no look has to find:
# `holding` is proprioceptive and grounding short-circuits a held entity.
LENS_Z_ABOVE_M = 0.10


class _Lens:
    """The part of a frame worth sending to a model, and the way back.

    THE ARM CAN ONLY REACH ELEVEN PERCENT OF THIS PICTURE. Measured on the rig's
    calibrated overhead D405: the safety workspace projects to 138x251 px of a
    640x480 frame, and a 32 mm block inside it is about 22 px across. That is not
    a simulator artefact — the twin's camera IS the real one's pose, intrinsics
    and field of view, so the real cell sends the same picture.

    What it cost is not detector noise but planner-level hallucination. Asked to
    "pick up the red block and place it on the white plate", the planner replied
    that the scene contained "two blue blocks and a gray plate" and aborted the
    task as unreachable — on a frame holding two red blocks, two blue ones, a
    white plate and a grey tray, all plainly visible to a human. The same
    confusion is the likeliest source of the mis-grounded plans that named a grey
    square tray as the target for a white plate.

    So the model is shown the workspace, upscaled, instead of a table lost in a
    black surround. Everything else keeps the full frame: `deproject` needs the
    real intrinsics, and the journal needs the picture a person would recognise.
    Detections come back in this view's coordinates and `to_frame` puts them back
    where the rest of the system expects them — nearest-neighbour and an integer
    factor, so that mapping is exact rather than nearly right.
    """

    def __init__(self, rgb: np.ndarray, u0: int = 0, v0: int = 0, factor: int = 1) -> None:
        self.rgb = rgb
        self.u0 = u0
        self.v0 = v0
        self.factor = factor

    @property
    def wh(self) -> tuple[int, int]:
        h, w = self.rgb.shape[:2]
        return w, h

    @property
    def cropped(self) -> bool:
        return self.u0 != 0 or self.v0 != 0 or self.factor != 1

    def to_frame(self, u: int, v: int) -> tuple[int, int]:
        return int(self.u0 + u / self.factor), int(self.v0 + v / self.factor)


def workspace_box(frame: Frame, cfg: HeronConfig) -> Optional[tuple[int, int, int, int]]:
    """Where the reachable workspace lands in this frame, in pixels.

    None when the answer would not be trustworthy or would not help: an
    uncalibrated frame, a projection that collapses, or a workspace that already
    fills the picture (every wrist frame, which is looking at one object from
    0.15 m and needs no help finding it).
    """
    if frame.intrinsics is None or frame.t_base_cam is None:
        return None
    try:
        inv = np.linalg.inv(np.asarray(frame.t_base_cam, dtype=float))
    except np.linalg.LinAlgError:
        return None
    k = np.asarray(frame.intrinsics, dtype=float)
    h, w = frame.rgb.shape[:2]
    us, vs = [], []
    for x in np.linspace(*cfg.safety.workspace_x, 5):
        for y in np.linspace(*cfg.safety.workspace_y, 5):
            for z in (cfg.table_z, cfg.table_z + LENS_Z_ABOVE_M):
                cam = inv @ np.array([x, y, z, 1.0])
                if cam[2] <= 1e-6:
                    continue          # behind the camera
                q = k @ cam[:3]
                us.append(q[0] / q[2])
                vs.append(q[1] / q[2])
    if not us:
        return None
    pad_u = LENS_MARGIN * (max(us) - min(us))
    pad_v = LENS_MARGIN * (max(vs) - min(vs))
    u0 = int(np.clip(min(us) - pad_u, 0, w - 1))
    u1 = int(np.clip(max(us) + pad_u, 0, w - 1))
    v0 = int(np.clip(min(vs) - pad_v, 0, h - 1))
    v1 = int(np.clip(max(vs) + pad_v, 0, h - 1))
    if u1 - u0 < LENS_MIN_PX or v1 - v0 < LENS_MIN_PX:
        return None
    if (u1 - u0) * (v1 - v0) >= LENS_MAX_COVERAGE * w * h:
        return None
    return u0, v0, u1, v1


def _points_from(raw, lens: "_Lens") -> list[tuple[int, int, float]]:
    """Every candidate the model returned, in the FRAME's pixels.

    A scene with several similar objects is exactly where taking the first
    detection goes wrong, and it goes wrong silently — the caller receives one
    confident point for the wrong can.
    """
    if not isinstance(raw, list):
        return []
    out = []
    for item in raw:
        if not isinstance(item, dict):
            continue
        pt = item.get("point") or [item.get("y"), item.get("x")]
        if not pt or len(pt) < 2 or pt[0] is None or pt[1] is None:
            continue
        try:
            u, v = lens.to_frame(*_to_px(float(pt[0]), float(pt[1]), *lens.wh))
        except (TypeError, ValueError):
            continue
        out.append((u, v, float(item.get("confidence", 0.8))))
    return out


def _box_from(b, lens: "_Lens") -> Optional[tuple[int, int, int, int, float]]:
    # The model returns boxes in several shapes across responses: a flat
    # [ymin, xmin, ymax, xmax] list, the same under "box_2d", or a dict of named
    # edges. Parsing only one of them crashed a whole episode.
    box = b.get("box_2d", b) if isinstance(b, dict) else b
    if isinstance(box, dict):
        y0, x0 = box.get("ymin", box.get("y")), box.get("xmin", box.get("x"))
        y1, x1 = box.get("ymax", box.get("y2")), box.get("xmax", box.get("x2"))
    elif isinstance(box, (list, tuple)) and len(box) >= 4:
        y0, x0, y1, x1 = box[:4]
    else:
        return None
    if None in (y0, x0, y1, x1):
        return None
    try:
        y0, x0, y1, x1 = float(y0), float(x0), float(y1), float(x1)
    except (TypeError, ValueError):
        return None
    u0, v0 = lens.to_frame(*_to_px(y0, x0, *lens.wh))
    u1, v1 = lens.to_frame(*_to_px(y1, x1, *lens.wh))
    conf = float(b.get("confidence", 0.8)) if isinstance(b, dict) else 0.8
    return min(u0, u1), min(v0, v1), max(u0, u1), max(v0, v1), conf


def _client_with_timeout(genai, timeout_s: float, base_url: str = "",
                         api_key_env: str = ""):
    """A client that gives up on a stalled request.

    Reads GEMINI_API_KEY by default. `base_url` and `api_key_env` point the same
    Gemini-format traffic at a relay instead — the reason to want that is cost:
    ER is priced as a robotics preview, and a relay serving a flash-tier model
    answers the minimal-thinking grounding calls (the bulk of every episode) for
    a fraction of it. The relay must speak the Gemini REST shape; an
    OpenAI-compatible one needs a different client, not a different URL —
    that is `models.api_format: openai` and `_openai_compat_client`.

    HttpOptions arrived in google-genai 1.x and takes MILLISECONDS. If this
    version of the SDK does not accept it we still want a working client, so
    fall back to the default — slow to fail beats refusing to start.
    """
    import os  # noqa: PLC0415

    key = os.environ.get(api_key_env) if api_key_env else None
    kwargs: dict = {"api_key": key} if key else {}
    try:
        from google.genai import types  # noqa: PLC0415

        http: dict = {"timeout": int(timeout_s * 1000)}
        if base_url:
            http["base_url"] = base_url
        return genai.Client(http_options=types.HttpOptions(**http), **kwargs)
    except Exception:  # pragma: no cover - depends on the installed SDK
        return genai.Client(**kwargs)


def _is_permanent(msg: str) -> bool:
    """Errors that retrying cannot fix, however transient the status code looks.

    A depleted prepaid balance returns 429, the same code as rate limiting.
    Retrying it burns ten seconds of backoff and buries the one line that tells
    you to go top up the account.
    """
    # Google spells these both ways depending on the surface — "PERMISSION_DENIED"
    # from the gRPC status, "permission denied" in prose — so match on the stem.
    return any(k in msg for k in
               ("credits are depleted", "billing", "quota exceeded", "api key not valid",
                "permission_denied", "permission denied", "consumer_suspended"))


# There is no route to the model at all — a dead proxy, no DNS, an unplugged
# box. Distinguished from the transient errors above because the answer is
# different in kind: a 503 is the server having a moment, and this is the
# network being absent.
UNREACHABLE = ("network is unreachable", "name or service not known",
               "temporary failure in name resolution", "no route to host",
               "failed to establish a new connection", "connection refused",
               # macOS spells DNS failure differently, and httpx (the relay
               # transport) collapses multi-address failures into its own phrase.
               "nodename nor servname", "all connection attempts failed")


# How many calls in a row may fail with "no route" on a route that has already
# worked before it is treated as gone. Three, because one is noise and a
# genuinely dead proxy will produce far more than three.
UNREACHABLE_STREAK_FATAL = 3


def _is_unreachable(msg: str) -> bool:
    return any(k in msg for k in UNREACHABLE)


def _openai_compat_client(cfg: HeronConfig):
    """An httpx client for an OpenAI-compatible relay (`models.api_format: openai`).

    The pp-api relay serves gemini-3.5-flash behind POST /chat/completions with
    Bearer auth; it is NOT Gemini-native (google-genai against it 404s with
    "Invalid URL"), so it needs a different client, not a different base_url.
    Configured entirely from env-var names so neither the URL nor the key lands
    in a committed YAML. Missing configuration fails here, at startup, rather
    than as a 401 twelve calls into an episode.
    """
    import os  # noqa: PLC0415

    import httpx  # noqa: PLC0415

    m = cfg.models
    base = m.resolved_base_url()
    if not base:
        raise RuntimeError("models.api_format is 'openai' but no base URL is configured — "
                           "set models.base_url, or models.base_url_env to the name of an "
                           "env var holding it")
    key = os.environ.get(m.api_key_env) if m.api_key_env else None
    if not key:
        raise RuntimeError(f"the relay needs a key: models.api_key_env={m.api_key_env!r} "
                           "names an env var that is unset or empty")
    return httpx.Client(base_url=base.rstrip("/"),
                        headers={"Authorization": f"Bearer {key}"},
                        timeout=cfg.budgets.llm_timeout_s)


class BudgetExhausted(RuntimeError):
    """A caller's per-scope call allowance ran out. Not an error in the world —
    a decision that this question has been asked enough times."""


class _CallableLog:
    """Adapt a bare callable (headless tools hand in `print`) to the
    EpisodeLogger contract this class actually uses. Without it, the
    error-reporting path itself raises AttributeError and SHADOWS the real
    failure — a judge died on `.event` while trying to say what went wrong."""

    def __init__(self, fn) -> None:
        self._fn = fn

    def _say(self, kind: str, fields: dict) -> None:
        try:
            self._fn(f"[{kind}] " + json.dumps(fields, ensure_ascii=False,
                                               default=str)[:400])
        except Exception:
            pass

    def event(self, kind: str, **fields) -> None:
        self._say(kind, fields)

    def model_call(self, **fields) -> None:
        self._say("model_call", fields)


class GeminiOrchestrator:
    def __init__(self, cfg: HeronConfig, log: EpisodeLogger) -> None:
        self.cfg = cfg
        self.log = log if hasattr(log, "event") else _CallableLog(log)
        # Persistent Live session for the vision-heavy roles (opt-in). Set up
        # FIRST: the relay path returns early below, and the Live session is
        # orthogonal to which REST transport answers everything else — a relay
        # deployment can still ride Live for grounding (it needs only
        # GEMINI_API_KEY, autoloaded from .env by heron.config).
        self._live = None
        if getattr(cfg.models, "live_model", "") and getattr(cfg.models, "live", False):
            try:
                from .live import LiveSession
                self._live = LiveSession(cfg.models.live_model,
                                         system_instruction="Answer exactly in the "
                                         "format each request asks for.",
                                         log=log)
            except Exception as e:
                log.event("live_init_failed", error=str(e)[:120])
                self._live = None
        self.model = cfg.models.er_model
        self._locate_cache: dict = {}
        self._lens_cache: dict = {}
        self._transport_down: Optional[str] = None
        self._transport_down_at: float = 0.0
        # On the rig the only egress is lab WiFi, whose multi-second blips can
        # eat 3 consecutive calls and latch this breaker even though the
        # network recovers moments later. Half-open after a cooldown: one
        # probe call is allowed through; success clears the latch.
        self._transport_retry_after_s: float = 60.0
        # Patient mode (rig): a live episode WAITS OUT the cooldown instead of
        # raising — losing 60 s beats losing the episode. Off (sweeps): latch
        # and raise fast, so a dead proxy costs one call's backoff, not one
        # cooldown per call across an 18-task sweep.
        self._transport_patient = os.environ.get(
            "HERON_TRANSPORT_PATIENT", "").lower() not in ("", "0", "false")
        # Whether this process has ever reached the model, and how many calls
        # in a row have failed to since. A route that has carried traffic is
        # not the same claim as a route that never existed — see _call.
        self._ever_connected = False
        self._unreachable_streak = 0
        self._http = None   # set only for the OpenAI-compatible relay path
        if cfg.models.api_format == "openai":
            # The relay path needs neither google-genai nor GEMINI_API_KEY.
            self._genai = None
            self.client = None
            self._use_interactions = False
            self._http = _openai_compat_client(cfg)
            return
        try:
            from google import genai  # noqa: PLC0415
        except ImportError as e:
            raise RuntimeError("google-genai is not installed (pip install 'google-genai>=2.9'); "
                               "use --orchestrator scripted for offline runs") from e
        self._genai = genai
        # `budgets.llm_timeout_s` existed but was never handed to the client, so a
        # network with no route to the API did not fail — it hung. One measurement
        # run sat on a single call for fifteen minutes before anyone noticed.
        self.client = _client_with_timeout(genai, cfg.budgets.llm_timeout_s,
                                           base_url=cfg.models.resolved_base_url(),
                                           api_key_env=cfg.models.api_key_env)
        self._use_interactions = hasattr(self.client, "interactions")

    # -- what the model is shown ---------------------------------------------
    def lens(self, frame: Frame) -> _Lens:
        """The view of this frame the model gets. See `_Lens`.

        Cached on the frame's identity because the crop is recomputed and the
        upscale re-materialised for every call about the same picture otherwise,
        and a batched look asks about one frame several times.
        """
        key = (id(frame), frame.t)
        hit = self._lens_cache.get(key)
        if hit is not None:
            return hit
        lens = self._make_lens(frame)
        self._lens_cache = {key: lens}
        return lens

    def _make_lens(self, frame: Frame) -> _Lens:
        box = workspace_box(frame, self.cfg)
        if box is None:
            return _Lens(frame.rgb)
        u0, v0, u1, v1 = box
        crop = frame.rgb[v0:v1 + 1, u0:u1 + 1]
        factor = int(min(LENS_MAX_UPSCALE,
                         max(1, np.ceil(LENS_TARGET_PX / max(1, min(crop.shape[:2]))))))
        if factor > 1:
            # Nearest neighbour, deliberately. There is no detail to recover here
            # and an interpolating resize would invent smooth edges — exactly the
            # kind of evidence the model is about to be asked to weigh.
            crop = np.kron(crop, np.ones((factor, factor, 1), np.uint8))
        lens = _Lens(np.ascontiguousarray(crop), u0, v0, factor)
        self.log.event("lens", camera=frame.camera, box=[u0, v0, u1, v1],
                       upscale=factor, sent=list(lens.wh),
                       frame=[frame.rgb.shape[1], frame.rgb.shape[0]])
        return lens

    def _px(self, frame: Frame, y: float, x: float) -> tuple[int, int]:
        """A model's pixel, in the frame's coordinates rather than the view's."""
        lens = self.lens(frame)
        w, h = lens.wh
        return lens.to_frame(*_to_px(y, x, w, h))

    # -- transport -----------------------------------------------------------
    def _png_b64(self, frame: Frame) -> str:
        import base64  # noqa: PLC0415

        buf = io.BytesIO()
        Image.fromarray(self.lens(frame).rgb).save(buf, format="PNG")
        return base64.b64encode(buf.getvalue()).decode()

    def _call(self, role: str, text: str, frames: list[Frame], thinking: str,
              schema: Optional[dict] = None, retries: int = 4) -> str:
        # One counter at the one place every request passes through, and one
        # budget enforced there too. Checking it at the call SITES leaked: the
        # grounding ladder spends locate, point, point, locate, box, box,
        # crop_recheck on a single entity, and the three checks scattered
        # through it all sat after the first four. A ceiling that is not at the
        # door is not a ceiling.
        self.calls = getattr(self, "calls", 0) + 1
        budget = getattr(self, "_budget", None)
        if budget is not None:
            if budget[1] <= 0:
                self.log.event("call_budget_exhausted", scope=budget[0], role=role)
                raise BudgetExhausted(f"{budget[0]}: no calls left for {role}")
            budget[1] -= 1
        # THE NETWORK BEING ABSENT IS NOT A TRANSIENT ERROR, and treating it as
        # one costs whole evaluations. Measured on the cluster with its proxy
        # down: every call retried four times behind a quadratic backoff, about
        # a minute each, and an episode makes a dozen calls — so each of 18
        # LIBERO tasks took 727 seconds to arrive at "network is unreachable",
        # and the sweep would have spent three and a half hours discovering the
        # same fact eighteen times. It is the repair loop's lesson at the
        # transport layer: retrying is a bet that something has changed.
        #
        # Deliberately per-process and one-way. The run is already lost by the
        # time this latches, and a half-finished sweep that quietly resumed on
        # call two hundred would be worse than one that stops and says why.
        if self._transport_down is not None:
            remaining = self._transport_retry_after_s - (time.time() - self._transport_down_at)
            if remaining > 0:
                # The rig's internet is WiFi and its outages are measured in
                # seconds — the latch protects against a dead network, but
                # RAISING here killed live episodes over a blip (2026-08-17:
                # plan() refused mid-episode, "nothing salvageable", route
                # fine a minute later). In patient mode (rig) wait it out —
                # 60 s is strictly cheaper than the episode — unless the
                # episode deadline says otherwise. Sweeps stay fail-fast.
                dl = getattr(self, "deadline", None)
                if dl is not None and time.time() + remaining > dl:
                    raise RuntimeError(
                        f"no route to the model — {self._transport_down}; the "
                        f"{remaining:.0f}s route cooldown would pass the episode "
                        f"deadline, giving up now instead")
                if not self._transport_patient:
                    raise RuntimeError(
                        f"no route to the model — {self._transport_down}. Refusing calls "
                        f"for {self._transport_retry_after_s:.0f}s after the last failure; "
                        f"will re-probe the route after the cooldown.")
                self.log.event("transport_cooldown_wait", down=self._transport_down,
                               wait_s=round(remaining, 1))
                time.sleep(remaining)
            # Half-open: let this one call through as the probe. A success
            # below clears the latch; a fresh unreachable error re-latches
            # with a new timestamp.
            self.log.event("transport_halfopen_probe", down=self._transport_down)
            self._transport_down = None
        # THE DEADLINE IS ENFORCED WHERE THE MONEY IS SPENT. The episode budget
        # used to be consulted only between steps, and a step is not the unit of
        # cost — a model call is. Measured on a LIBERO episode with a 360 s
        # budget: one locate call spent 120.9 s inside its own retry backoff,
        # sailed through the deadline mid-call, and the goal verification then
        # issued five more calls after it, ending at 487 s. Checking here bounds
        # the overrun at one call; refusing to sleep past the deadline (below)
        # bounds that call too.
        deadline = getattr(self, "deadline", None)
        if deadline is not None and time.time() > deadline:
            raise RuntimeError(
                f"episode deadline passed ({role}: {time.time() - deadline:.0f}s over "
                f"the wall-clock budget) — refusing to start another model call")
        t0 = time.time()
        last_err: Exception | None = None
        # Vision-heavy, TEXT-answer roles ride the persistent Live session when
        # one is configured: no per-call connection, the scene context stays
        # server-side, and a grounding answer lands in 1-2 s instead of 2-3 s
        # (measured, tools/live_probe.py). Schema'd planning/repair calls and
        # anything else keep the REST path; a Live failure falls back to REST
        # for THIS call and the session reconnects lazily on the next.
        # box stays on REST: the streaming model merges lookalikes into one
        # box (measured: a box spanning both red blocks, centroid grasped the
        # midpoint of the pair). Points and VQA are its strong suits.
        if (schema is None and getattr(self, "_live", None) is not None
                and role in ("locate", "locate_many", "vqa")):
            tool = {"locate": "answer_points", "locate_many": "answer_points",
                    "box": "answer_boxes", "vqa": "answer_vqa"}.get(role)
            try:
                directive = (f"\n\nRespond ONLY by calling the {tool} tool — "
                             f"no prose, no JSON text." if tool else "")
                out = self._live.ask(text + directive, frames,
                                     timeout_s=float(self.cfg.budgets.llm_timeout_s))
                self.log.model_call(role=role, prompt_chars=len(text), images=len(frames),
                                    latency_s=time.time() - t0, response_preview=out,
                                    thinking="live")
                return out
            except Exception as e:
                self.log.event("live_fallback", role=role, error=str(e)[:120])
        if self._use_interactions:
            transport = self._call_interactions
        elif getattr(self, "_http", None) is not None:
            transport = self._call_openai_compat
        else:
            transport = self._call_generate
        for attempt in range(retries):
            try:
                out = transport(text, frames, thinking, schema)
                self._ever_connected = True
                self._unreachable_streak = 0
                self.log.model_call(role=role, prompt_chars=len(text), images=len(frames),
                                    latency_s=time.time() - t0, response_preview=out, thinking=thinking)
                return out
            except Exception as e:  # retry only transient transport errors
                last_err = e
                msg = str(e).lower()
                if _is_permanent(msg):
                    self.log.event("model_call_permanent_error", role=role,
                                   error=str(e)[:300])
                    raise
                if _is_unreachable(msg):
                    # A ROUTE THAT HAS CARRIED TRAFFIC IS NOT AN ABSENT ROUTE.
                    # The latch below was measured against a proxy that was
                    # down for a whole sweep, and it is right for that. It is
                    # wrong for a relay that refuses one TCP connection under
                    # load: the pp-api relay did exactly that 110 s into a
                    # bimanual episode, answered normally seconds later, and
                    # the episode was declared unsalvageable anyway. Whether
                    # this process has ever reached the model is the evidence
                    # that tells the two apart — so keep the hard latch for a
                    # network that was never there, and let a proven route
                    # fail a few times in a row before giving up on it.
                    self._unreachable_streak += 1
                    # A process that has never reached the model still gets its
                    # in-call retries before latching: on the rig the FIRST
                    # call of an episode hit a one-off Errno 113 at t=0.7s and
                    # the instant latch condemned the whole episode (measured
                    # twice, 2026-08-17 21:03 and 21:08, route healthy a
                    # minute later). A genuinely dead proxy still fails fast —
                    # one call's worth of backoff (~10s), not one per task.
                    fatal = (self._unreachable_streak >= UNREACHABLE_STREAK_FATAL
                             or (not self._ever_connected and attempt >= retries - 1))
                    self.log.event("model_unreachable", role=role, error=str(e)[:300],
                                   streak=self._unreachable_streak,
                                   ever_connected=self._ever_connected,
                                   note=("no route to the model; refusing further calls "
                                         "rather than retrying into a dead network"
                                         if fatal else
                                         "a route that has worked refused one call; "
                                         "retrying rather than latching"))
                    if fatal:
                        self._transport_down = str(e)[:200]
                        self._transport_down_at = time.time()
                        # A route that HAS carried traffic and has attempts
                        # left gets one in-call cooldown before the raise —
                        # the rig's WiFi outages outlast the quadratic
                        # backoff (2s, 8s) but not a cooldown. A network that
                        # was never there still fails fast.
                        cooldown = self._transport_retry_after_s
                        if (self._transport_patient and attempt < retries - 1
                                and (deadline is None
                                     or time.time() + cooldown <= deadline)):
                            self.log.event("transport_cooldown_wait",
                                           down=self._transport_down,
                                           wait_s=cooldown, inline=True)
                            time.sleep(cooldown)
                            self.log.event("transport_halfopen_probe",
                                           down=self._transport_down)
                            self._transport_down = None
                            continue
                        raise
                    if attempt >= retries - 1:
                        raise
                    time.sleep(2.0 * (attempt + 1) ** 2)
                    continue
                if attempt < retries - 1 and any(k in msg for k in RETRYABLE):
                    backoff = 2.0 * (attempt + 1) ** 2
                    if deadline is not None and time.time() + backoff > deadline:
                        # A retry that would wake up past the deadline is not a
                        # retry, it is a slower way to fail.
                        self.log.event("retry_refused_deadline", role=role,
                                       error=str(e)[:200])
                        raise
                    time.sleep(backoff)
                    continue
                raise
        raise last_err  # pragma: no cover

    def _call_interactions(self, text: str, frames: list[Frame], thinking: str, schema: Optional[dict]) -> str:
        parts: list[dict[str, Any]] = [
            {"type": "image", "data": self._png_b64(f), "mime_type": "image/png"} for f in frames
        ]
        parts.append({"type": "text", "text": text})
        kwargs: dict[str, Any] = {
            "model": self.model,
            "input": parts,
            "generation_config": {"thinking_level": thinking},
        }
        if schema is not None:
            kwargs["response_format"] = {"type": "text", "mime_type": "application/json", "schema": schema}
        ix = self.client.interactions.create(**kwargs)
        return ix.output_text or ""

    def _call_openai_compat(self, text: str, frames: list[Frame], thinking: str,
                            schema: Optional[dict]) -> str:
        """The same call, spoken OpenAI: POST {base_url}/chat/completions.

        Differences from the Gemini paths, on purpose:
        - `thinking` is not sent. There is no OpenAI parameter relays agree on
          for it, and an unknown field is a 400 on the strict ones. _call still
          logs the level, so the journal records what the call wanted.
        - A requested schema is stated in the prompt and the response is only
          pinned to `json_object`. The relay does not enforce schemas; the
          lenient _parse_json plus plan()'s retry-with-feedback already handle a
          model that strays, and grounding never passes a schema at all.
        """
        import httpx  # noqa: PLC0415

        if schema is not None:
            text = (text + "\n\nAnswer with JSON only, matching this JSON schema exactly:\n"
                    + json.dumps(schema))
        content: list[dict[str, Any]] = [
            {"type": "image_url",
             "image_url": {"url": f"data:image/png;base64,{self._png_b64(f)}"}}
            for f in frames
        ]
        content.append({"type": "text", "text": text})
        body: dict[str, Any] = {"model": self.model,
                                "messages": [{"role": "user", "content": content}]}
        if schema is not None:
            body["response_format"] = {"type": "json_object"}
        try:
            resp = self._http.post("/chat/completions", json=body)
        except httpx.HTTPError as e:
            # str(ReadTimeout()) is usually empty; the class name is the message,
            # and _call's retry/latch logic matches on the message.
            raise RuntimeError(f"{type(e).__name__}: {e}") from e
        if resp.status_code != 200:
            raise RuntimeError(f"{resp.status_code} {resp.text[:500]}")
        data = resp.json()
        try:
            out = data["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as e:
            raise RuntimeError(f"relay returned no completion: {str(data)[:300]}") from e
        return out or ""

    def _call_generate(self, text: str, frames: list[Frame], thinking: str, schema: Optional[dict]) -> str:
        from google.genai import types  # noqa: PLC0415

        contents: list[Any] = []
        for f in frames:
            buf = io.BytesIO()
            Image.fromarray(self.lens(f).rgb).save(buf, format="PNG")
            contents.append(types.Part.from_bytes(data=buf.getvalue(), mime_type="image/png"))
        contents.append(text)
        config: dict[str, Any] = {}
        if schema is not None:
            config = {"response_mime_type": "application/json", "response_json_schema": schema}
        resp = self.client.models.generate_content(model=self.model, contents=contents,
                                                   config=config or None)
        return resp.text or ""

    # -- Grounder ------------------------------------------------------------
    def point(self, frame: Frame, query: str) -> Optional[tuple[int, int, float]]:
        text = prompts.POINT_PROMPT.format(query=query)
        try:
            raw = _parse_json(self._call("point", text, [frame], self.cfg.models.thinking_ground))
        except Exception as e:
            self.log.event("ground_error", query=query, error=str(e))
            return None
        candidates = self._points_from(raw, frame)
        return candidates[0] if candidates else None

    def points(self, frame: Frame, query: str) -> list[tuple[int, int, float]]:
        got = self.locate(frame, query)
        if got:
            return [(c["point"][0], c["point"][1], c["conf"]) for c in got]
        return self._points_only(frame, query)   # locate saw nothing at all

    def _points_only(self, frame: Frame, query: str) -> list[tuple[int, int, float]]:
        """Every candidate the model saw, not just its favourite.

        A scene with several similar objects is exactly where taking the first
        detection goes wrong, and it goes wrong silently — the caller receives one
        confident point for the wrong can. Returning the whole list lets the
        caller use what it already knows about where the object should be.
        """
        text = prompts.POINT_PROMPT.format(query=query)
        try:
            raw = _parse_json(self._call("point", text, [frame], self.cfg.models.thinking_ground))
        except Exception as e:
            self.log.event("ground_error", query=query, error=str(e))
            return []
        return self._points_from(raw, frame)

    def _points_from(self, raw, frame: Frame) -> list[tuple[int, int, float]]:
        return _points_from(raw, self.lens(frame))

    def _evict_other_frames(self, frame: Frame) -> None:
        """Keep this LOOK's answers, not this frame's.

        It kept only the identical frame, which was right while there was one
        usable camera and silently wrong the moment there were two: priming
        cam_low evicted everything cam_high had just been primed with, so every
        entity then missed on cam_high and paid a real `locate` for an answer
        already in hand. Measured across today's twin runs — 5.9 locate calls
        per episode on one camera, 17.3 on two, for the same work.

        A look's frames are captured together (`sensing._batched_look` takes
        every shutter before asking anything), so they share a timestamp to
        within milliseconds and a time window separates one look from the next.
        That keeps the property the identity check actually wanted: never answer
        about a world that has moved. It was the frame identity that was
        incidental.
        """
        self._locate_cache = {k: v for k, v in self._locate_cache.items()
                              if abs(float(k[1]) - float(frame.t)) <= LOOK_TOGETHER_S}

    def prime_locate(self, frame: Frame, found: dict[str, dict]) -> int:
        """Seed the per-frame cache from one batched request.

        `locate_many` asks about every object in a single reading of the image;
        `locate` is what grounding actually calls, one entity at a time. Without
        this bridge the batched prompt existed and nothing on the agent path
        used it: measured over a 60-episode sweep, 929 one-at-a-time `locate`
        calls and zero `locate_many`.

        Returns how many queries were seeded, so the caller can log what the
        batch actually bought.
        """
        self._evict_other_frames(frame)
        for query, det in found.items():
            if not det or det.get("point") is None:
                continue
            self._locate_cache[(id(frame), frame.t, query)] = [dict(det)]
        return len(found)

    def locate(self, frame: Frame, query: str) -> list[dict]:
        """Points and boxes in ONE call, cached per (frame, query).

        Returns [{"point": (u, v), "box": (u0,v0,u1,v1) or None, "conf": float}],
        best first. The box is genuinely optional: on the lab's overhead frame
        ER-2 points at everything and boxes almost nothing.

        The cache is keyed on the frame's own timestamp, so asking for the point
        and then the box of the same object in the same frame — which is what
        grounding does — costs one round trip rather than two.
        """
        key = (id(frame), frame.t, query)
        hit = self._locate_cache.get(key)
        if hit is not None:
            return hit
        self._evict_other_frames(frame)
        text = prompts.LOCATE_PROMPT.format(query=query)
        try:
            raw = _parse_json(self._call("locate", text, [frame], self.cfg.models.thinking_ground))
        except Exception as e:
            self.log.event("ground_error", query=query, error=str(e))
            return []
        out = []
        if isinstance(raw, list):
            for item in raw:
                if not isinstance(item, dict):
                    continue
                pt = item.get("point") or [item.get("y"), item.get("x")]
                if not pt or len(pt) < 2 or pt[0] is None or pt[1] is None:
                    continue
                try:
                    u, v = self._px(frame, float(pt[0]), float(pt[1]))
                except (TypeError, ValueError):
                    continue
                box = self._box_from(item.get("box"), frame) if item.get("box") else None
                out.append({"point": (u, v), "box": box[:4] if box else None,
                            "conf": float(item.get("confidence", 0.8))})
        self._locate_cache[key] = out
        return out

    def survey(self, frame: Frame, instruction: str) -> dict:
        """Everything on the table, named, plus the goal — in ONE call.

        The entity list and the goal are the same act of understanding: which
        two trays the operator means and what "serve crackers on each" asserts
        about them are decided by the same look. Splitting them, as this code
        did all day, forces every name to be located separately and the goal to
        be translated by a call that cannot see the table.

        Returns {"objects": [{"id", "description", "point", "box", "conf"}],
        "goal": [str], "note": str}. Empty objects means the call failed; the
        caller falls back to the per-entity path it used before.
        """
        text = prompts.SURVEY_PROMPT.format(instruction=instruction)
        try:
            raw = _parse_json(self._call("survey", text, [frame],
                                         self.cfg.models.thinking_plan))
        except Exception as e:
            self.log.event("survey_error", error=str(e)[:200])
            return {"objects": [], "goal": [], "note": ""}
        if not isinstance(raw, dict):
            return {"objects": [], "goal": [], "note": ""}
        objects = []
        seen = set()
        for item in raw.get("objects") or []:
            if not isinstance(item, dict):
                continue
            eid = str(item.get("id") or "").strip()
            pt = item.get("point") or []
            if not eid or eid in seen or len(pt) < 2 or pt[0] is None or pt[1] is None:
                continue
            try:
                u, v = self._px(frame, float(pt[0]), float(pt[1]))
            except (TypeError, ValueError):
                continue
            box = self._box_from(item.get("box"), frame) if item.get("box") else None
            seen.add(eid)
            objects.append({"id": eid,
                            "description": str(item.get("description") or eid).strip(),
                            "point": (u, v), "box": box[:4] if box else None,
                            "conf": float(item.get("confidence", 0.8))})
        goal = [str(g).strip() for g in (raw.get("goal") or []) if str(g).strip()]
        self.log.event("survey", objects=len(objects), goal=goal,
                       note=str(raw.get("note") or "")[:160])
        return {"objects": objects, "goal": goal,
                "note": str(raw.get("note") or "")}

    def locate_many(self, frame: Frame, queries: list[str]) -> dict[str, dict]:
        """Every object in one call, keyed by the query that asked for it.

        The model reads the whole frame to answer about any part of it, so
        asking six times costs six readings of one image. Measured: a
        long-horizon sort spent six calls locating six things that a single
        request describes.

        Falls back to nothing — a caller that gets an empty result should ask
        one at a time, since a partial answer here is more useful than none.
        """
        if not queries:
            return {}
        listing = "\n".join(f"- {q}" for q in queries)
        text = prompts.LOCATE_MANY_PROMPT.format(queries=listing)
        try:
            raw = _parse_json(self._call("locate_many", text, [frame],
                                         self.cfg.models.thinking_ground))
        except Exception as e:
            self.log.event("ground_error", query=f"{len(queries)} queries", error=str(e))
            return {}
        if not isinstance(raw, list):
            return {}
        want = {q.strip().lower(): q for q in queries}
        out: dict[str, dict] = {}
        for item in raw:
            if not isinstance(item, dict):
                continue
            key = want.get(str(item.get("query", "")).strip().lower())
            if key is None or key in out:
                continue
            pt = item.get("point") or [item.get("y"), item.get("x")]
            if not pt or len(pt) < 2 or pt[0] is None or pt[1] is None:
                continue
            try:
                u, v = self._px(frame, float(pt[0]), float(pt[1]))
            except (TypeError, ValueError):
                continue
            box = self._box_from(item.get("box"), frame) if item.get("box") else None
            out[key] = {"point": (u, v), "box": box[:4] if box else None,
                        "conf": float(item.get("confidence", 0.8))}
        self.log.event("locate_many", asked=len(queries), answered=len(out),
                       unmatched=[q for q in queries if q not in out])
        return out

    def boxes(self, frame: Frame, query: str) -> list[tuple[int, int, int, int, float]]:
        """Every box the model returned, from the same round trip as the points.

        Position is derived from the box, so an association that only picks
        among POINTS changes nothing — which is why both are needed, and why
        they should not cost two calls.
        """
        got = self.locate(frame, query)
        # No second request when locate() came back with points but no boxes.
        # That is the NORMAL answer on this rig's wide overhead frame — ER-2
        # points at everything and boxes almost nothing — and the dedicated box
        # prompt, asked about the same image moments later, returns empty too.
        # Measured: three such retries per episode, 17.8 s, for nothing. The
        # extent comes from the mask instead, which is where it was coming from
        # anyway.
        if got:
            return [(*c["box"], c["conf"]) for c in got if c["box"] is not None]
        return self._boxes_only(frame, query)

    def _boxes_only(self, frame: Frame, query: str) -> list[tuple[int, int, int, int, float]]:
        """Fallback: the dedicated box prompt, for when locate() found nothing."""
        text = prompts.BOX_PROMPT.format(query=query)
        try:
            raw = _parse_json(self._call("box", text, [frame], self.cfg.models.thinking_ground))
        except Exception as e:
            self.log.event("ground_error", query=query, error=str(e))
            return []
        if not isinstance(raw, list):
            return []
        out = []
        for item in raw:
            parsed = self._box_from(item, frame)
            if parsed is not None:
                out.append(parsed)
        return out

    def box(self, frame: Frame, query: str) -> Optional[tuple[int, int, int, int, float]]:
        text = prompts.BOX_PROMPT.format(query=query)
        try:
            raw = _parse_json(self._call("box", text, [frame], self.cfg.models.thinking_ground))
        except Exception as e:
            self.log.event("ground_error", query=query, error=str(e))
            return None
        if not isinstance(raw, list) or not raw:
            return None
        return self._box_from(raw[0], frame)

    def _box_from(self, b, frame: Frame) -> Optional[tuple[int, int, int, int, float]]:
        return _box_from(b, self.lens(frame))

    def vqa(self, frames: list[Frame], question: str) -> tuple[Value, float, str]:
        text = prompts.VQA_PROMPT.format(question=question)
        try:
            raw = _parse_json(self._call("vqa", text, frames, self.cfg.models.thinking_verify))
        except Exception as e:
            return Value.UNKNOWN, 0.0, f"vqa error: {e}"
        answer = str(raw.get("answer", "unsure")).lower()
        value = {"yes": Value.TRUE, "no": Value.FALSE}.get(answer, Value.UNKNOWN)
        return value, float(raw.get("confidence", 0.5)), str(raw.get("reason", ""))

    def choose(self, frames: list[Frame], query: str) -> tuple[Optional[int], float, str]:
        """Which of these images shows the thing. (index, confidence, reason).

        None means none of them, which is the usual and correct answer: the
        bottom-up search that calls this is looking for an object the detector
        could not name, and most of the regions it segments are not it.
        """
        if not frames:
            return None, 0.0, "nothing to choose between"
        text = prompts.CHOOSE_PROMPT.format(n=len(frames), query=query)
        try:
            raw = _parse_json(self._call("choose", text, frames,
                                         self.cfg.models.thinking_verify))
        except Exception as e:
            return None, 0.0, f"choose error: {e}"
        if not isinstance(raw, dict):
            return None, 0.0, "malformed answer"
        try:
            pick = int(raw.get("choice", 0))
        except (TypeError, ValueError):
            return None, 0.0, str(raw.get("reason", "unreadable choice"))
        conf = float(raw.get("confidence", 0.5))
        reason = str(raw.get("reason", ""))
        if not (1 <= pick <= len(frames)):
            return None, conf, reason          # 0, or out of range: none of them
        return pick - 1, conf, reason

    # -- Planner -------------------------------------------------------------
    def plan(self, instruction: str, frames: list[Frame], skills_doc: str, hints: str) -> TaskProgram:
        system = prompts.PLANNER_SYSTEM.format(
            embodiment=self.cfg.embodiment,
            wx=list(self.cfg.safety.workspace_x), wy=list(self.cfg.safety.workspace_y),
            skills=skills_doc, hints=hints,
        )
        user = prompts.PLANNER_USER.format(instruction=instruction,
                                           cameras=", ".join(f.camera for f in frames))
        text = system + "\n\n" + user
        # THE CATALOG IS AN ENUM, NOT A SUGGESTION. The relay model invented
        # "pick_block" — a skill name and its argument mashed together — and
        # only the post-hoc registry check caught it. Baking the registry's
        # names into the decoding schema refuses the invention at generation
        # time; built per call because learned skills join the registry live.
        schema = PlanOut.model_json_schema()
        try:
            from ..skills import registry  # noqa: PLC0415
            names = sorted(registry.names())
            schema["$defs"]["StepOut"]["properties"]["skill"]["enum"] = names
        except Exception:
            pass
        feedback = ""
        for _ in range(2):
            out = self._call("plan", text + feedback, frames,
                             self.cfg.models.thinking_plan, schema=schema)
            try:
                plan = PlanOut.model_validate(_parse_json(out))
                return self._to_program(instruction, plan)
            except (ValidationError, ValueError, json.JSONDecodeError) as e:
                feedback = f"\n\nYour previous output failed validation: {e}. Emit corrected JSON only."
        raise RuntimeError("planner produced invalid programs twice")

    def _clean_args(self, skill: str, args: "ArgsOut") -> dict[str, Any]:
        """Drop args the skill's schema does not declare (models copy fields across skills)."""
        from ..skills import registry  # noqa: PLC0415

        raw = args.to_dict()
        try:
            allowed = set(registry.get(skill).params)
        except KeyError:
            return raw
        dropped = {k: v for k, v in raw.items() if k not in allowed}
        if dropped:
            self.log.event("args_filtered", skill=skill, dropped=dropped)
        return {k: v for k, v in raw.items() if k in allowed}

    @staticmethod
    def _clean_preds(preds: list[PredicateOut]) -> list[PredicateSpec]:
        """Arm predicates without an arm argument are unverifiable — drop them."""
        out = []
        for p in preds:
            if p.name in ("holding", "gripper_empty"):
                need = 2 if p.name == "holding" else 1
                if len(p.args) < need or not all(p.args):
                    continue
            out.append(p.to_spec())
        return out

    def _clean_relation(self, e: EntityOut, declared: set[str]):
        """Keep a relation only if the executor can actually resolve it.

        Two ways a well-formed relation is still unusable: the wrong number of
        anchors for its kind, and an anchor that names nothing the plan declared
        — the model does occasionally write `of: ["table"]` for a scene with no
        table entity. Both are dropped here, with the reason logged, because a
        relation that survives to grounding and then quietly does nothing is
        indistinguishable from one that was honoured.
        """
        from ..skills import relations as R  # noqa: PLC0415

        rel = e.relation
        if rel is None:
            return None
        spec = SpatialRelation(kind=rel.kind, of=list(rel.of), negated=rel.negated)
        bad = R.validate(spec)
        if bad is None:
            unknown = [a for a in spec.of if a not in declared and a != e.id]
            if unknown:
                bad = f"anchor(s) {unknown} are not declared entities"
            elif e.id in spec.of:
                bad = "an object cannot be an anchor for itself"
        if bad:
            self.log.event("relation_dropped", entity=e.id, relation=str(spec), reason=bad)
            return None
        return spec

    def _to_program(self, instruction: str, plan: PlanOut) -> TaskProgram:
        declared = {e.id for e in plan.entities}
        entities = [EntityDecl(id=e.id, description=e.description, role=e.role,
                               relation=self._clean_relation(e, declared))
                    for e in plan.entities]
        steps = []
        for i, s in enumerate(plan.steps, start=1):
            for ref in (s.args.entity, s.args.target):
                if ref and ref not in declared:  # lenient: auto-declare with the id as description
                    entities.append(EntityDecl(id=ref, description=ref.replace("_", " ")))
                    declared.add(ref)
            when = None
            if s.when is not None:
                cleaned = self._clean_preds([s.when])
                when = cleaned[0] if cleaned else None
            steps.append(Step(id=f"s{i}", skill=s.skill, args=self._clean_args(s.skill, s.args),
                              pre=self._clean_preds(s.pre), post=self._clean_preds(s.post),
                              when=when, rationale=s.rationale))
        repeat = None
        if plan.repeat_until is not None:
            cleaned = self._clean_preds([plan.repeat_until])
            repeat = cleaned[0] if cleaned else None
        return TaskProgram(goal=instruction, goal_predicates=[p.to_spec() for p in plan.goal_predicates],
                           entities=entities, steps=steps, repeat_until=repeat)

    # -- Repairer ------------------------------------------------------------
    def propose_repair(self, program: TaskProgram, belief_summary: str, failure: FailureReport,
                       skills_doc: str, frames: list[Frame]) -> ProgramEdit:
        system = prompts.REPAIR_SYSTEM.format(skills=skills_doc)
        user = prompts.REPAIR_USER.format(program=program.compact(), belief=belief_summary,
                                          failure=failure.model_dump_json(indent=2))
        # Same second chance plan() gets. A repair that fails to parse used to
        # abort the whole episode — one malformed answer outweighing everything
        # the run had already paid for — and the relay (no server-side schema
        # enforcement) makes malformed answers likelier than ER did.
        feedback = ""
        rep = None
        # The step ids and entity ids a repair may reference are CLOSED SETS,
        # so close them in the decoding schema — the same trick that stopped
        # invented skill names. Before this, a repair targeted step 'r4_2' in
        # a program whose repair steps had different numbers, twice, and the
        # episode died as "orchestrator repair invalid" after doing 12 steps
        # of real work (rig pnp batch EP5/EP7, 2026-08-10).
        schema = RepairOut.model_json_schema()
        def _close(prop: str, values: list[str]) -> None:
            spec = schema.get("properties", {}).get(prop, {})
            for branch in spec.get("anyOf", []):
                if branch.get("type") == "string":
                    branch["enum"] = values
        if program.steps:
            _close("step_id", sorted({st.id for st in program.steps}))
        if program.entities:
            _close("entity_id", sorted({e.id for e in program.entities}))
        for attempt in range(2):
            out = self._call("repair", system + "\n\n" + user + feedback, frames,
                             self.cfg.models.thinking_repair, schema=schema)
            try:
                rep = RepairOut.model_validate(_parse_json(out))
                break
            except (ValidationError, ValueError, json.JSONDecodeError) as e:
                if attempt:
                    raise
                feedback = f"\n\nYour previous output failed validation: {e}. Emit corrected JSON only."
        steps = []
        existing = {st.id for st in program.steps}
        n = len(program.edits) + 1
        for i, s in enumerate(rep.steps):
            new_id = f"r{n}_{i}"
            while new_id in existing:
                n += 1
                new_id = f"r{n}_{i}"
            existing.add(new_id)
            steps.append(Step(id=new_id, skill=s.skill, args=self._clean_args(s.skill, s.args),
                              pre=self._clean_preds(s.pre), post=self._clean_preds(s.post),
                              rationale=s.rationale))
        return ProgramEdit(type=EditType(rep.type), step_id=rep.step_id,
                           new_args=rep.new_args.to_dict() if rep.new_args else None, steps=steps,
                           entity_id=rep.entity_id, new_description=rep.new_description,
                           rationale=rep.rationale, source="orchestrator")
