"""Parsing every candidate the model returns, in every shape it returns them in.

A single malformed entry once crashed a whole episode, and a parser that silently
keeps only the first entry makes the association logic above it inert.
"""
from __future__ import annotations

import numpy as np
import pytest

from heron.orchestrator.gemini import (GeminiOrchestrator, _Lens, _box_from,
                                       _points_from)
from heron.types import Frame

FRAME = Frame(camera="c", rgb=np.zeros((720, 1280, 3), np.uint8))
# The parsers read the view the model was shown, not the frame. Uncropped
# here; `test_the_lens.py` covers what happens when it is not.
WHOLE = _Lens(FRAME.rgb)


def test_every_point_candidate_is_returned():
    raw = [{"point": [500, 250], "confidence": 0.9},
           {"point": [500, 750], "confidence": 0.6}]
    pts = _points_from(raw, WHOLE)
    assert len(pts) == 2
    assert pts[0][0] < pts[1][0], "order must follow the model's ranking"
    assert pts[0][2] == pytest.approx(0.9)


def test_malformed_entries_are_skipped_not_fatal():
    raw = [{"nonsense": 1}, "a string", {"point": [500, 250]}, {"point": [None, 3]}]
    pts = _points_from(raw, WHOLE)
    assert len(pts) == 1


def test_boxes_parse_in_all_three_shapes():
    for entry in ({"box_2d": [100, 200, 300, 400]},
                  {"y": 100, "x": 200, "y2": 300, "x2": 400},
                  {"box_2d": {"ymin": 100, "xmin": 200, "ymax": 300, "xmax": 400}}):
        box = _box_from(entry, WHOLE)
        assert box is not None, entry
        assert box[0] < box[2] and box[1] < box[3]


def test_a_reversed_box_is_normalised():
    box = _box_from({"box_2d": [300, 400, 100, 200]}, WHOLE)
    assert box[0] < box[2] and box[1] < box[3]


def test_a_box_of_the_wrong_type_is_refused_quietly():
    assert _box_from({"box_2d": "not a box"}, WHOLE) is None
    assert _box_from({"box_2d": [1, 2]}, WHOLE) is None


def test_points_and_boxes_share_one_round_trip():
    """They were two calls on the same image about the same object, moments
    apart: 23 detection round trips for a two-object task, 127 s of a 207 s
    episode."""
    import numpy as np

    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator
    from heron.types import Frame

    calls = []
    orch = GeminiOrchestrator.__new__(GeminiOrchestrator)
    orch.cfg = HeronConfig()
    orch.log = EpisodeLogger("/tmp/heron-test-episodes", "locate")
    orch._locate_cache = {}
    orch._lens_cache = {}

    def fake_call(role, text, frames, thinking, schema=None, retries=3):
        calls.append(role)
        return ('[{"label": "block", "point": [500, 400], '
                '"box": [480, 380, 520, 420], "confidence": 0.9}]')

    orch._call = fake_call
    frame = Frame(camera="cam_high", rgb=np.zeros((100, 100, 3), np.uint8))

    pts = orch.points(frame, "the block")
    boxes = orch.boxes(frame, "the block")
    assert len(calls) == 1, f"one look at the frame, not {len(calls)}: {calls}"
    assert calls == ["locate"]
    assert len(pts) == 1 and len(boxes) == 1
    assert pts[0][:2] == (39, 49)          # 400/1000 and 500/1000 of 99 px
    assert boxes[0][:4] == (37, 47, 41, 51)


def test_a_point_without_a_box_is_normal_not_a_failure():
    """On the lab's overhead frame ER-2 points at everything and boxes almost
    nothing, so grounding has to work from the point alone."""
    import numpy as np

    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator
    from heron.types import Frame

    orch = GeminiOrchestrator.__new__(GeminiOrchestrator)
    orch.cfg = HeronConfig()
    orch.log = EpisodeLogger("/tmp/heron-test-episodes", "locate")
    orch._locate_cache = {}
    orch._lens_cache = {}
    orch._call = lambda *a, **k: '[{"label": "b", "point": [500, 400], "box": null, "confidence": 0.9}]'

    frame = Frame(camera="cam_high", rgb=np.zeros((100, 100, 3), np.uint8))
    assert len(orch.points(frame, "the block")) == 1
    assert orch.boxes(frame, "the block") == []


def test_a_new_frame_is_not_served_from_the_cache():
    import numpy as np

    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator
    from heron.types import Frame

    calls = []
    orch = GeminiOrchestrator.__new__(GeminiOrchestrator)
    orch.cfg = HeronConfig()
    orch.log = EpisodeLogger("/tmp/heron-test-episodes", "locate")
    orch._locate_cache = {}
    orch._lens_cache = {}
    orch._call = lambda *a, **k: (calls.append(1) or
                                  '[{"point": [500, 400], "box": null, "confidence": 0.9}]')

    rgb = np.zeros((100, 100, 3), np.uint8)
    orch.points(Frame(camera="cam_high", rgb=rgb, t=1.0), "the block")
    orch.points(Frame(camera="cam_high", rgb=rgb, t=2.0), "the block")
    assert len(calls) == 2, "a fresh capture must be looked at again"


def test_no_second_request_when_locate_answered_without_boxes():
    """A point with a null box is the normal answer on the overhead frame, and
    asking the dedicated box prompt about the same image returns empty too —
    three such retries per episode, 17.8 s, for nothing."""
    import numpy as np

    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator
    from heron.types import Frame

    roles = []
    orch = GeminiOrchestrator.__new__(GeminiOrchestrator)
    orch.cfg = HeronConfig()
    orch.log = EpisodeLogger("/tmp/heron-test-episodes", "locate")
    orch._locate_cache = {}
    orch._lens_cache = {}
    orch._call = lambda role, *a, **k: (
        roles.append(role) or '[{"point": [500, 400], "box": null, "confidence": 0.9}]')

    frame = Frame(camera="cam_high", rgb=np.zeros((100, 100, 3), np.uint8))
    assert orch.boxes(frame, "the block") == []
    assert roles == ["locate"], f"asked again for boxes it was already told about: {roles}"

    # But when locate finds NOTHING, the dedicated prompt is still worth a try.
    orch._locate_cache = {}
    orch._lens_cache = {}
    roles.clear()
    orch._call = lambda role, *a, **k: roles.append(role) or "[]"
    orch.boxes(Frame(camera="cam_high", rgb=np.zeros((100, 100, 3), np.uint8), t=9.0), "x")
    assert roles == ["locate", "box"]
