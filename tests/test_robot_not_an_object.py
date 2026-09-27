"""A detection that lands on the robot is the robot."""
from __future__ import annotations
import numpy as np
from heron.belief import BeliefStore
from heron.config import HeronConfig
from heron.episode import EpisodeLogger
from heron.skills import SkillContext
from heron.skills.sensing import ROBOT_EXCLUSION_M, _is_the_robot
from heron.types import EntityDecl


def _ctx(tool=(0.42, 0.16, 0.14)):
    class Robot:
        arms = ["right"]
        cameras = ["cam_high"]

        def get_cartesian(self, arm):
            return np.array(tool, dtype=float)

    ctx = SkillContext(robot=Robot(), belief=BeliefStore(), grounder=None,
                       cfg=HeronConfig(),
                       log=EpisodeLogger("/tmp/heron-test-episodes", "robot"))
    ctx.belief.declare_entities([EntityDecl(id="grey_tray", description="the grey tray")])
    return ctx


def test_the_tray_grounded_onto_the_arm_is_rejected():
    """Measured: the tray came back at [0.4209, 0.1598, 0.1431] while the tool
    was there. The wrist was then aimed at that point and the block measured
    against it, so `in(block, tray)` verified TRUE with the block 98 mm away."""
    ctx = _ctx()
    assert _is_the_robot(ctx, np.array([0.4209, 0.1598, 0.1431]), "grey_tray") == "right"


def test_an_object_the_arm_is_hovering_over_is_not_the_arm():
    """The arm spends its life directly above objects. The guard must not
    swallow the object it is reaching for — the hover sits HOVER_CLEARANCE
    (0.10 m) above it, comfortably outside the exclusion radius."""
    from heron.skills.primitives import HOVER_CLEARANCE

    assert ROBOT_EXCLUSION_M < HOVER_CLEARANCE
    ctx = _ctx(tool=(0.30, 0.10, 0.12))
    assert _is_the_robot(ctx, np.array([0.30, 0.10, 0.12 - HOVER_CLEARANCE]), "grey_tray") is None


def test_a_held_object_is_allowed_to_be_where_the_gripper_is():
    ctx = _ctx()
    ctx.belief.update_track("grey_tray", xyz_base=(0.42, 0.16, 0.14), confidence=0.9,
                            held_by="right")
    assert _is_the_robot(ctx, np.array([0.42, 0.16, 0.14]), "grey_tray") is None


def test_a_backend_that_cannot_say_where_its_arm_is_does_not_block_grounding():
    class Robot:
        arms = ["right"]
        cameras = ["cam_high"]

        def get_cartesian(self, arm):
            raise RuntimeError("no encoder")

    ctx = SkillContext(robot=Robot(), belief=BeliefStore(), grounder=None,
                       cfg=HeronConfig(),
                       log=EpisodeLogger("/tmp/heron-test-episodes", "robot"))
    assert _is_the_robot(ctx, np.array([0.4, 0.1, 0.1]), "x") is None
