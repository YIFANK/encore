"""A geometric verdict that barely clears its own threshold is not evidence.

`in` and `on` compare two of OUR OWN position estimates. Grounding measures
11 mm from a mask and 22.7 mm from a single pixel on this rig, and the plate's
centre came back 13.6 mm off in the twin. So a verdict decided by a few
millimetres is reporting the noise in its own inputs.

Measured, and it cost a false success: `in(red_block, grey_tray)` returned TRUE
at |dxy| = 35 mm against a 42 mm cavity, with confidence 0.95 — because the
confidence was the DETECTOR's (0.95 on both groundings) and had nothing to do
with how close the call was. The second opinion said "no red block is clearly
visible" and did not veto, because only a confident FALSE vetoed. Ground truth:
100 mm. The episode was recorded as a success and the skill library learned
from it.
"""
from __future__ import annotations

import pytest

from heron.agent import GOAL_NEEDS_BACKUP_CONF
from heron.verify import GEOMETRIC_MARGIN_M, margin_confidence


def test_the_measured_false_positive_now_lands_below_the_backup_threshold():
    """35 mm against a 42 mm cavity: seven millimetres of margin."""
    got = margin_confidence(0.95, 0.042 - 0.035)
    assert got < GOAL_NEEDS_BACKUP_CONF, (
        f"a 7 mm margin still scored {got:.2f} — it would stand unconfirmed again")


def test_a_comfortable_margin_keeps_its_confidence():
    """Otherwise every `in` needs a second opinion, and a container that
    legitimately hides its contents becomes permanently unverifiable."""
    assert margin_confidence(0.95, 0.040) == pytest.approx(0.95)
    assert margin_confidence(0.95, GEOMETRIC_MARGIN_M) == pytest.approx(0.95)


def test_confidence_falls_smoothly_rather_than_off_a_cliff():
    xs = [0.0, 0.003, 0.006, 0.009, 0.012, GEOMETRIC_MARGIN_M]
    got = [margin_confidence(0.95, x) for x in xs]
    assert got == sorted(got), f"not monotone: {got}"
    assert got[0] >= 0.4, "a reading is still a reading; it just must not be trusted"
    assert got[-1] == pytest.approx(0.95)


def test_a_verdict_on_the_wrong_side_is_discounted_the_same_way():
    """The distance to the threshold is what matters, not which side of it the
    answer fell on: a FALSE decided by 2 mm is exactly as uninformative."""
    assert margin_confidence(0.95, 0.002) < GOAL_NEEDS_BACKUP_CONF


def test_the_discount_never_reaches_zero():
    """Zero confidence would make the verdict unusable for diagnosis, which
    still wants to know which side of the line the measurement fell on."""
    assert margin_confidence(0.95, 0.0) >= 0.4
    assert margin_confidence(0.5, 0.0) >= 0.4


def test_the_container_width_is_not_re_asked_for_either():
    """Every `in` and `on` check calls `_half_extent`, and each one re-captured
    the frame and re-asked the detector for a box grounding had just recorded:
    three of the eleven model calls in a 58-second pick-and-place, 6.5 seconds,
    for a number that had not changed."""
    from heron.belief import BeliefStore
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.types import EntityDecl
    from heron.verify import CONTAINER_HALF_EXTENT_MAX, _half_extent

    class _Grounder:
        def box(self, *a, **k):
            raise AssertionError("re-asked for a width already measured")

    class _Robot:
        arms = ["right"]
        cameras = ["cam_high"]

        def capture(self, cam):
            raise AssertionError("re-captured for a width already measured")

    belief = BeliefStore()
    belief.declare_entities([EntityDecl(id="tray", description="the grey tray")])
    belief.update_track("tray", xyz_base=(0.36, 0.28, -0.007), camera="cam_high",
                        span_m=0.140, confidence=0.95)
    ctx = SkillContext(robot=_Robot(), belief=belief, grounder=_Grounder(),
                       cfg=HeronConfig(),
                       log=EpisodeLogger("/tmp/heron-test-episodes", "half"))
    assert _half_extent(ctx, "tray") == pytest.approx(0.070)

    # A bad span must not widen the containment test without bound — the same
    # guard the detector path has always had.
    belief.update_track("tray", span_m=0.90)
    assert _half_extent(ctx, "tray") == pytest.approx(CONTAINER_HALF_EXTENT_MAX)
