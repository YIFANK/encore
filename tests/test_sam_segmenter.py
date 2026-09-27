"""The segmenter's job is to be optional. These tests are mostly about failure."""
from __future__ import annotations

import base64
import io
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import pytest
from PIL import Image

from heron.perception import SamSegmenter
from heron.perception.sam import FAILURES_BEFORE_MUTE
from heron.types import Frame


def _serve(reply, status=200):
    """A one-endpoint stub server; returns (url, shutdown, requests_seen)."""
    seen: list[dict] = []

    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def do_POST(self):
            seen.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
            body = json.dumps(reply(seen[-1]) if callable(reply) else reply).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return f"http://127.0.0.1:{srv.server_address[1]}", srv.shutdown, seen


def _mask_b64(mask: np.ndarray) -> str:
    buf = io.BytesIO()
    Image.fromarray(mask.astype(np.uint8) * 255).save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode()


def _rgb(h=40, w=40):
    return np.zeros((h, w, 3), dtype=np.uint8)


def test_no_url_is_disabled_and_returns_none():
    seg = SamSegmenter(None)
    assert not seg.enabled
    assert seg.mask(_rgb(), (0, 0, 10, 10)) is None


def test_unreachable_server_returns_none_then_mutes():
    # Port 1 on loopback refuses immediately, so this stays fast.
    seg = SamSegmenter("http://127.0.0.1:1", timeout_s=1.0)
    for _ in range(FAILURES_BEFORE_MUTE):
        assert seg.mask(_rgb(), (0, 0, 10, 10)) is None
    assert not seg.enabled, "a dead service must stop costing a timeout per call"


def test_good_mask_round_trips():
    m = np.zeros((40, 40), dtype=bool)
    m[10:20, 10:20] = True  # 100 px inside an 11x11=121 px box
    url, stop, seen = _serve({"ok": True, "mask": _mask_b64(m), "score": 0.97})
    try:
        seg = SamSegmenter(url)
        got = seg.mask(_rgb(), (10, 10, 20, 20), point=(15, 15))
    finally:
        stop()
    assert got is not None and got.shape == (40, 40)
    assert got[15, 15] and not got[0, 0]
    assert seen[0]["box"] == [10.0, 10.0, 20.0, 20.0]
    assert seen[0]["point"] == [15.0, 15.0]


@pytest.mark.parametrize("mask_fn,score", [
    (lambda: np.ones((40, 40), dtype=bool), 0.99),          # the whole box
    (lambda: _one_pixel(), 0.99),                            # a speck
    (lambda: _block(), 0.10),                                # low score
])
def test_implausible_masks_are_rejected(mask_fn, score):
    url, stop, _ = _serve({"ok": True, "mask": _mask_b64(mask_fn()), "score": score})
    try:
        assert SamSegmenter(url).mask(_rgb(), (10, 10, 20, 20)) is None
    finally:
        stop()


def _one_pixel():
    m = np.zeros((40, 40), dtype=bool)
    m[15, 15] = True
    return m


def _block():
    m = np.zeros((40, 40), dtype=bool)
    m[10:20, 10:20] = True
    return m


def test_server_error_is_not_an_exception():
    url, stop, _ = _serve({"ok": False, "error": "boom"})
    try:
        assert SamSegmenter(url).mask(_rgb(), (0, 0, 10, 10)) is None
    finally:
        stop()


# -- the Frame side -----------------------------------------------------------

def _frame_with_two_objects():
    """A 1 m plane at z=0 with two 6 cm-tall blocks, seen from straight above.

    The box is drawn around both; the mask covers only the left one. So the
    box estimate lands between them and the mask estimate lands on the left.
    """
    h = w = 60
    depth = np.full((h, w), 1.0, dtype=np.float32)
    depth[20:40, 10:25] = 0.94   # left block
    depth[20:40, 35:50] = 0.94   # right block
    k = np.array([[60.0, 0, 30.0], [0, 60.0, 30.0], [0, 0, 1.0]])
    # Camera 1 m above the origin looking down: x right, y down, z toward table.
    t = np.array([[1.0, 0, 0, 0], [0, -1.0, 0, 0], [0, 0, -1.0, 1.0], [0, 0, 0, 1.0]])
    mask = np.zeros((h, w), dtype=bool)
    mask[20:40, 10:25] = True
    return Frame(camera="c", rgb=np.zeros((h, w, 3), np.uint8), depth=depth,
                 intrinsics=k, t_base_cam=t), mask


def test_mask_separates_two_objects_inside_one_box():
    frame, mask = _frame_with_two_objects()
    box = frame.deproject_region(10, 20, 49, 39, support_z=0.0)
    masked = frame.deproject_region(10, 20, 49, 39, support_z=0.0, mask=mask)
    left_true_x = ((10 - 30) / 60 * 0.94 + (24 - 30) / 60 * 0.94) / 2
    assert abs(masked[0] - left_true_x) < 0.01, "mask should land on the left block"
    assert abs(box[0]) < 0.02, "the box spans both, so its midpoint is between them"
    assert abs(masked[0] - box[0]) > 0.05


def test_wrong_shaped_mask_is_a_programming_error():
    frame, _ = _frame_with_two_objects()
    with pytest.raises(ValueError, match="mask is"):
        frame.deproject_region(0, 0, 10, 10, mask=np.ones((5, 5), dtype=bool))


def test_mask_with_too_little_depth_falls_back_to_the_box():
    frame, _ = _frame_with_two_objects()
    thin = np.zeros(frame.depth.shape, dtype=bool)
    thin[25, 15] = True  # a single pixel: fewer than the 8 needed
    box = frame.deproject_region(10, 20, 49, 39, support_z=0.0)
    got = frame.deproject_region(10, 20, 49, 39, support_z=0.0, mask=thin)
    assert got is not None and np.allclose(got, box), "should degrade, not vanish"


# -- bottom-up identification -------------------------------------------------

def _ctx_with(monkeypatch, proposals, answers):
    """A SkillContext whose segmenter proposes `proposals` and whose grounder
    answers `answers` in order."""
    import types as _t

    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.types import Value

    class Seg:
        enabled = True

        def propose(self, rgb, max_masks=40):
            return proposals

    class Grounder:
        def __init__(self):
            self.asked = []

        def vqa(self, frames, question):
            self.asked.append(question)
            return answers[len(self.asked) - 1]

    cfg = HeronConfig()
    ctx = SkillContext(robot=_t.SimpleNamespace(cameras=[]), belief=None,
                       grounder=Grounder(), cfg=cfg,
                       log=EpisodeLogger("/tmp/heron-test-episodes", "identify"),
                       segmenter=Seg())
    return ctx, Value


def _proposal(box, h=60, w=60, score=0.9):
    m = np.zeros((h, w), dtype=bool)
    m[box[1]:box[3] + 1, box[0]:box[2] + 1] = True
    return {"mask": m, "box": box, "pixels": int(m.sum()), "score": score}


def test_identify_returns_the_region_vqa_confirms(monkeypatch):
    from heron.skills.sensing import identify_by_parts
    from heron.types import Frame, Value

    props = [_proposal((5, 5, 20, 20), score=0.95), _proposal((30, 30, 45, 45), score=0.9)]
    ctx, _ = _ctx_with(monkeypatch, props,
                       [(Value.FALSE, 0.9, "a mug"), (Value.TRUE, 0.88, "butter")])
    frame = Frame(camera="cam_high", rgb=np.zeros((60, 60, 3), np.uint8))
    mask, box, conf = identify_by_parts(ctx, frame, "a stick of butter")
    assert box == (30, 30, 45, 45)
    assert conf == 0.88 and mask[35, 35]


def test_identify_gives_up_rather_than_guessing(monkeypatch):
    from heron.skills.sensing import identify_by_parts
    from heron.types import Frame, Value

    props = [_proposal((5, 5, 20, 20))]
    ctx, _ = _ctx_with(monkeypatch, props, [(Value.FALSE, 0.95, "no")])
    frame = Frame(camera="cam_high", rgb=np.zeros((60, 60, 3), np.uint8))
    assert identify_by_parts(ctx, frame, "a stick of butter") == (None, None, 0.0)


def test_identify_rejects_a_low_confidence_yes(monkeypatch):
    """A hesitant yes on the only candidate is how you grasp the wrong thing."""
    from heron.skills.sensing import IDENTIFY_MIN_CONF, identify_by_parts
    from heron.types import Frame, Value

    props = [_proposal((5, 5, 20, 20))]
    ctx, _ = _ctx_with(monkeypatch, props,
                       [(Value.TRUE, IDENTIFY_MIN_CONF - 0.05, "maybe")])
    frame = Frame(camera="cam_high", rgb=np.zeros((60, 60, 3), np.uint8))
    assert identify_by_parts(ctx, frame, "butter")[0] is None


def test_scenery_is_dropped_before_any_vqa_call(monkeypatch):
    """A region covering half the image is the table. Asking about it wastes a
    call and, worse, invites a yes."""
    from heron.skills.sensing import identify_by_parts
    from heron.types import Frame, Value

    props = [_proposal((0, 0, 55, 55)), _proposal((30, 30, 45, 45))]
    ctx, _ = _ctx_with(monkeypatch, props, [(Value.TRUE, 0.9, "yes")])
    frame = Frame(camera="cam_high", rgb=np.zeros((60, 60, 3), np.uint8))
    mask, box, conf = identify_by_parts(ctx, frame, "butter")
    assert box == (30, 30, 45, 45), "the first question must go to the small region"
    assert len(ctx.grounder.asked) == 1


def test_no_segmenter_means_no_identification():
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.sensing import identify_by_parts
    from heron.types import Frame

    ctx = SkillContext(robot=None, belief=None, grounder=None, cfg=HeronConfig(),
                       log=EpisodeLogger("/tmp/heron-test-episodes", "identify"))
    frame = Frame(camera="c", rgb=np.zeros((10, 10, 3), np.uint8))
    assert identify_by_parts(ctx, frame, "butter") == (None, None, 0.0)


def test_point_only_prompt_is_sent_without_a_box():
    """The real rig's route: ER points, ER will not box, SAM supplies extent."""
    m = np.zeros((40, 40), dtype=bool)
    m[10:20, 10:20] = True
    url, stop, seen = _serve({"ok": True, "mask": _mask_b64(m), "score": 0.95})
    try:
        got = SamSegmenter(url).mask(_rgb(), None, point=(15, 15))
    finally:
        stop()
    assert got is not None and got[15, 15]
    assert seen[0]["box"] is None
    assert seen[0]["point"] == [15.0, 15.0]


def test_point_only_rejects_a_mask_covering_the_whole_scene():
    """A point on a table returns the table. Without a box there is no region
    to measure against, so the frame is the yardstick."""
    from heron.perception.sam import MAX_FRAME_FRACTION

    m = np.zeros((40, 40), dtype=bool)
    m[:, :] = True  # 100% of the frame, far over the limit
    url, stop, _ = _serve({"ok": True, "mask": _mask_b64(m), "score": 0.99})
    try:
        assert SamSegmenter(url).mask(_rgb(), None, point=(15, 15)) is None
    finally:
        stop()
    assert MAX_FRAME_FRACTION < 1.0


def test_point_only_accepts_a_small_mask():
    """The box-prompted lower bound must not apply here: a 2 % floor measured
    against the FRAME would reject every real object."""
    m = np.zeros((40, 40), dtype=bool)
    m[18:22, 18:22] = True  # 16 px of 1600 = 1 %
    url, stop, _ = _serve({"ok": True, "mask": _mask_b64(m), "score": 0.95})
    try:
        got = SamSegmenter(url).mask(_rgb(), None, point=(20, 20))
    finally:
        stop()
    assert got is not None and got[20, 20]


def test_neither_box_nor_point_is_a_programming_error():
    seg = SamSegmenter("http://127.0.0.1:1")
    with pytest.raises(ValueError, match="box, a point, or both"):
        seg.mask(_rgb())


def test_candidates_are_all_returned_and_ordered():
    m1, m2 = np.zeros((40, 40), bool), np.zeros((40, 40), bool)
    m1[18:22, 18:22] = True      # the part
    m2[10:30, 10:30] = True      # the object
    url, stop, _ = _serve({"ok": True, "mask": _mask_b64(m1), "score": 0.95,
                           "candidates": [{"mask": _mask_b64(m1), "score": 0.95},
                                          {"mask": _mask_b64(m2), "score": 0.90}]})
    try:
        got = SamSegmenter(url).candidates(_rgb(), None, point=(20, 20))
    finally:
        stop()
    assert [c["pixels"] for c in got] == [16, 400]
    assert got[0]["score"] > got[1]["score"]


def test_physical_size_overrides_sam_score():
    """SAM ranked a laptop's keyboard above the laptop. Metres settle it."""
    from heron.skills.sensing import _by_physical_size

    frame, _ = _frame_with_two_objects()          # 60x60, 1 m plane, known K
    tiny, right = np.zeros((60, 60), bool), np.zeros((60, 60), bool)
    tiny[30, 30] = True                            # sub-millimetre: implausible
    right[20:40, 10:25] = True                     # ~15 px wide -> ~0.23 m
    cands = [{"mask": tiny, "score": 0.99, "pixels": 1},
             {"mask": right, "score": 0.80, "pixels": int(right.sum())}]

    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext

    ctx = SkillContext(robot=None, belief=None, grounder=None, cfg=HeronConfig(),
                       log=EpisodeLogger("/tmp/heron-test-episodes", "size"))
    chosen = _by_physical_size(ctx, frame, cands)
    assert chosen[30, 15], "should pick the object-sized candidate, not the speck"
    assert not chosen[0, 0]


def test_size_choice_falls_back_when_the_frame_is_uncalibrated():
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.sensing import _by_physical_size
    from heron.types import Frame

    frame = Frame(camera="c", rgb=np.zeros((40, 40, 3), np.uint8))  # no depth, no H
    a, b = np.zeros((40, 40), bool), np.zeros((40, 40), bool)
    a[10:20, 10:20] = True
    b[5:35, 5:35] = True
    ctx = SkillContext(robot=None, belief=None, grounder=None, cfg=HeronConfig(),
                       log=EpisodeLogger("/tmp/heron-test-episodes", "size"))
    chosen = _by_physical_size(ctx, frame,
                               [{"mask": a, "score": 0.9, "pixels": 100},
                                {"mask": b, "score": 0.8, "pixels": 900}])
    assert np.array_equal(chosen, a), "with no calibration, keep SAM's own order"
