"""Perception paid for once per look instead of once per entity.

Grounding is written per entity, and has to be: association, relations and the
re-identification guard are all about one object's history. But it captured its
own frame each time, so six entities meant six captures of a world that had not
moved — six distinct frames, six readings of the same image, and a detector
cache that could never hit because no two entities shared a frame.

Measured over a 60-episode LIBERO-PRO sweep before this: 929 one-at-a-time
`locate` calls, ZERO `locate_many`, and 486 repeat groundings of the same
entity in the same camera inside a single episode.
"""
from __future__ import annotations

import numpy as np

from heron.belief import BeliefStore
from heron.config import HeronConfig
from heron.episode import EpisodeLogger
from heron.skills import SkillContext
from heron.skills.sensing import _batched_look
import pytest
from heron.types import EntityDecl, Frame


class _Grounder:
    """Counts what it is asked, and honours a primed cache like the real one."""

    def __init__(self):
        self.many_calls = 0
        self.single_calls = 0
        self.primed: dict = {}

    def locate_many(self, frame, queries):
        self.many_calls += 1
        return {q: {"point": (10 + 3 * i, 20), "box": None, "conf": 0.9}
                for i, q in enumerate(queries)}

    def prime_locate(self, frame, found):
        for q, det in found.items():
            self.primed[(id(frame), frame.t, q)] = [dict(det)]
        return len(found)

    def locate(self, frame, query):
        hit = self.primed.get((id(frame), frame.t, query))
        if hit is not None:
            return hit
        self.single_calls += 1
        return []


class _Robot:
    arms = ["right"]
    cameras = ["cam_high"]

    def __init__(self):
        self.captures = 0

    def capture(self, cam):
        # A distinct timestamp per capture, as every real backend does
        # (`t=time.time()`). The cache key pairs the frame's identity with its
        # timestamp precisely because CPython reuses id() after a frame is
        # collected; a fake that stamps every frame 1.0 makes two different
        # frames indistinguishable and would test the opposite of the truth.
        self.captures += 1
        return Frame(camera=cam, rgb=np.zeros((60, 60, 3), np.uint8),
                     t=1.0 + self.captures)


def _ctx(n_entities: int):
    cfg = HeronConfig()
    cfg.cameras = {"cam_high": cfg.cameras["cam_high"]}
    belief = BeliefStore()
    belief.declare_entities([
        EntityDecl(id=f"e{i}", description=f"the object number {i}")
        for i in range(n_entities)])
    return SkillContext(robot=_Robot(), belief=belief, grounder=_Grounder(), cfg=cfg,
                        log=EpisodeLogger("/tmp/heron-test-episodes", "batch"))


def test_six_objects_cost_one_request_not_six():
    ctx = _ctx(6)
    frames = _batched_look(ctx, [f"e{i}" for i in range(6)], ["cam_high"])
    assert ctx.grounder.many_calls == 1, "should have asked once for everything"
    assert ctx.robot.captures == 1, "should have captured the scene once"
    assert set(frames) == {"cam_high"}


def test_the_primed_answers_are_what_grounding_then_reads():
    """The batch is only worth making if the per-entity path consumes it. If the
    cache key does not line up, the batch becomes a seventh call rather than a
    replacement for six."""
    ctx = _ctx(6)
    _batched_look(ctx, [f"e{i}" for i in range(6)], ["cam_high"])
    frame = ctx.robot.capture("cam_high")   # a DIFFERENT frame: must not hit
    ctx.grounder.locate(frame, "the object number 0")
    assert ctx.grounder.single_calls == 1, "a new frame must not read stale answers"


def test_a_single_object_is_not_worth_batching():
    """One query through the batched prompt is the same one call, with a longer
    prompt and a matching step that can fail. Take the ordinary path."""
    ctx = _ctx(1)
    assert _batched_look(ctx, ["e0"], ["cam_high"]) == {}
    assert ctx.grounder.many_calls == 0
    assert ctx.robot.captures == 0


def test_a_grounder_without_the_batched_path_is_left_alone():
    """ScriptedOrchestrator and any future backend must keep working."""
    ctx = _ctx(4)

    class _Plain:
        def locate(self, frame, query):
            return []

    ctx.grounder = _Plain()
    assert _batched_look(ctx, ["e0", "e1"], ["cam_high"]) == {}
    assert ctx.robot.captures == 0


def test_a_detector_that_fails_the_batch_does_not_lose_the_frame():
    """A failed batch must degrade to the per-entity path, not to no perception:
    the frames are still handed on so the captures are not repeated either."""
    ctx = _ctx(4)

    def boom(frame, queries):
        raise RuntimeError("model unavailable")

    ctx.grounder.locate_many = boom
    frames = _batched_look(ctx, ["e0", "e1", "e2", "e3"], ["cam_high"])
    assert set(frames) == {"cam_high"}, "the capture should still be reusable"


# -- the bottom-up identifier, which ate 79% of the VQA budget ----------------
#
# Measured over the 60-episode LIBERO-PRO sweep: 129 invocations, 766 VQA calls,
# 12 successes. Seventy-nine per cent of the run's whole VQA budget — 2088 of
# 2636 seconds — in a path that found the object 9% of the time, because a miss
# costs the full six calls and a miss is the usual outcome.

def _crop_ctx(n_regions: int, chooser=None):
    from heron.episode import EpisodeLogger

    cfg = HeronConfig()

    class _Seg:
        enabled = True

        def propose(self, rgb, max_masks=40):
            return [{"box": (10 * i, 10, 10 * i + 8, 18),
                     "mask": np.ones((60, 60), bool), "score": 0.9 - 0.01 * i}
                    for i in range(n_regions)]

    class _Grounder:
        def __init__(self):
            self.vqa_calls = 0
            self.choose_calls = 0

        def vqa(self, frames, question):
            from heron.types import Value
            self.vqa_calls += 1
            return Value.FALSE, 0.9, "no"

    g = _Grounder()
    if chooser is not None:
        def choose(frames, query):
            g.choose_calls += 1
            return chooser(frames, query)
        g.choose = choose

    class _Robot:
        arms = ["right"]
        cameras = ["cam_high"]

    ctx = SkillContext(robot=_Robot(), belief=BeliefStore(), grounder=g, cfg=cfg,
                       log=EpisodeLogger("/tmp/heron-test-episodes", "crops"))
    ctx.segmenter = _Seg()
    return ctx, g


def test_six_candidate_regions_cost_one_question():
    from heron.skills.sensing import identify_by_parts

    ctx, g = _crop_ctx(6, chooser=lambda frames, q: (None, 0.9, "none"))
    frame = Frame(camera="cam_high", rgb=np.zeros((60, 60, 3), np.uint8), t=1.0)
    identify_by_parts(ctx, frame, "the black bowl")
    assert g.choose_calls == 1, "should have asked once about all the crops"
    assert g.vqa_calls == 0, "should not also ask one at a time"


def test_none_of_these_stays_sayable():
    """A bottom-up search that cannot come back empty would bind the entity to
    whichever region looked least unlike it."""
    from heron.skills.sensing import identify_by_parts

    ctx, g = _crop_ctx(4, chooser=lambda frames, q: (None, 0.95, "none match"))
    frame = Frame(camera="cam_high", rgb=np.zeros((60, 60, 3), np.uint8), t=1.0)
    mask, box, conf = identify_by_parts(ctx, frame, "the black bowl")
    assert mask is None and box is None and conf == 0.0


def test_the_chosen_crop_maps_back_to_its_own_region():
    """Off-by-one here silently returns a different object's mask, and every
    measurement downstream would be of the wrong thing."""
    from heron.skills.sensing import identify_by_parts

    ctx, g = _crop_ctx(5, chooser=lambda frames, q: (2, 0.9, "third one"))
    frame = Frame(camera="cam_high", rgb=np.zeros((60, 60, 3), np.uint8), t=1.0)
    mask, box, conf = identify_by_parts(ctx, frame, "the black bowl")
    assert box == (20, 10, 28, 18), f"got {box}, expected the third region's box"
    assert conf == pytest.approx(0.9)


def test_a_grounder_without_choose_keeps_the_old_one_at_a_time_path():
    from heron.skills.sensing import identify_by_parts

    ctx, g = _crop_ctx(3, chooser=None)
    frame = Frame(camera="cam_high", rgb=np.zeros((60, 60, 3), np.uint8), t=1.0)
    identify_by_parts(ctx, frame, "the black bowl")
    assert g.choose_calls == 0
    assert g.vqa_calls == 3, "should have fallen back to asking about each crop"


def test_a_second_camera_does_not_evict_the_first_one_s_answers(tmp_path):
    """It kept only the identical frame, which was right while there was one
    usable camera and silently wrong the moment there were two: priming cam_low
    evicted everything cam_high had just been primed with, so every entity then
    missed on cam_high and paid a real `locate` for an answer already in hand.

    Measured across the twin runs: 5.9 locate calls per episode on one camera,
    17.3 on two, for the same work.
    """
    import numpy as np

    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator
    from heron.types import Frame

    orch = GeminiOrchestrator.__new__(GeminiOrchestrator)
    orch.cfg = HeronConfig()
    orch.log = EpisodeLogger(str(tmp_path), "cache")
    orch._locate_cache = {}
    orch._lens_cache = {}

    # One look: every shutter fires together, so the stamps agree to the
    # millisecond. `_batched_look` is what guarantees that.
    rgb = np.zeros((32, 32, 3), np.uint8)
    high = Frame(camera="cam_high", rgb=rgb, t=100.000)
    low = Frame(camera="cam_low", rgb=rgb, t=100.004)
    det = {"point": (5, 5), "box": None, "conf": 0.9}

    orch.prime_locate(high, {"the red block": det})
    orch.prime_locate(low, {"the red block": det})

    assert orch.locate(high, "the red block"), "cam_high's answer must survive cam_low's"
    assert orch.locate(low, "the red block")


def test_the_next_look_still_clears_the_last_one(tmp_path):
    """The property that mattered was never frame identity — it was refusing to
    answer about a world that has moved. A look is separated from the next by at
    least one model call, which is seconds."""
    import numpy as np

    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import LOOK_TOGETHER_S, GeminiOrchestrator
    from heron.types import Frame

    orch = GeminiOrchestrator.__new__(GeminiOrchestrator)
    orch.cfg = HeronConfig()
    orch.log = EpisodeLogger(str(tmp_path), "cache")
    orch._locate_cache = {}
    orch._lens_cache = {}

    rgb = np.zeros((32, 32, 3), np.uint8)
    det = {"point": (5, 5), "box": None, "conf": 0.9}
    first = Frame(camera="cam_high", rgb=rgb, t=100.0)
    later = Frame(camera="cam_high", rgb=rgb, t=100.0 + 4 * LOOK_TOGETHER_S)

    orch.prime_locate(first, {"the red block": det})
    orch.prime_locate(later, {"the red block": det})
    assert orch._locate_cache and all(abs(k[1] - later.t) < 1e-6 for k in orch._locate_cache), \
        "the stale look must be gone"
