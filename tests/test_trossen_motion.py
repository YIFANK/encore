"""Motion staging for the real arm — the path, not just the goal, must be safe.

A straight cartesian interpolation from a high pose to a table-level goal
sweeps the wrist diagonally through the table (it drove the arm into the rail
on the abaka rig), and the driver reports it as an IK error at an intermediate
waypoint. These tests pin the two fixes: a table-facing orientation that is not
inherited from the ready pose, and up/over/down staging on descents.
"""
from __future__ import annotations

import sys
import types

import numpy as np
import pytest

from heron.config import HeronConfig


class _FakeDriver:
    """Records commanded JOINT waypoints and reports the pose they imply.

    Heron solves IK itself and sends joint targets, so the fake has to run FK to
    answer get_cartesian_positions — otherwise the tests would be checking a
    stub that agrees with whatever was asked for, which is no check at all.
    """

    def __init__(self, start=(0.0, np.pi / 2, np.pi / 2, 0.0, 0.0, 0.0)):
        from heron.robot.kinematics import Kinematics, rotvec

        self._kin = Kinematics()
        self._rotvec = rotvec
        self.joint_commands: list[np.ndarray] = []
        self.durations: list[float] = []
        self._q = np.asarray(start, dtype=float)

    # every commanded waypoint, as the cartesian pose it puts the tool at
    @property
    def commands(self):
        out = []
        for q in self.joint_commands:
            t = self._kin.fk(q)
            out.append(np.concatenate([t[:3, 3], self._rotvec(t[:3, :3])]))
        return out

    def configure(self, *a, **k): pass
    def set_all_modes(self, *a, **k): pass
    def get_all_positions(self): return np.concatenate([self._q, [0.0]])
    def get_gripper_position(self): return 0.0
    def cleanup(self): pass

    def get_cartesian_positions(self):
        t = self._kin.fk(self._q)
        return np.concatenate([t[:3, 3], self._rotvec(t[:3, :3])])

    def set_arm_positions(self, goal, seconds=1.0, blocking=True):
        self.set_all_positions(list(goal) + [0.0], seconds, blocking)

    def set_all_positions(self, goal, seconds=1.0, blocking=True):
        q = np.asarray(goal, dtype=float)[:6]
        self.joint_commands.append(q.copy())
        self.durations.append(float(seconds))
        self._q = q.copy()

    def set_cartesian_positions(self, *a, **k):
        raise AssertionError("cartesian commands go through the controller's own "
                             "closed-form IK, which is what we stopped using")


@pytest.fixture
def rig(monkeypatch):
    drivers = []

    fake = types.ModuleType("trossen_arm")
    fake.TrossenArmDriver = lambda: drivers[-1]
    fake.Model = types.SimpleNamespace(wxai_v0="wxai_v0")
    fake.StandardEndEffector = types.SimpleNamespace(wxai_v0_follower="ee")
    fake.Mode = types.SimpleNamespace(position="position", external_effort="effort")
    fake.InterpolationSpace = types.SimpleNamespace(cartesian="cartesian", joint="joint")
    monkeypatch.setitem(sys.modules, "trossen_arm", fake)

    fake_rs = types.ModuleType("pyrealsense2")
    monkeypatch.setitem(sys.modules, "pyrealsense2", fake_rs)

    cfg = HeronConfig()
    cfg.arms = {"right": cfg.arms["right"]}
    cfg.arms["right"].ip = "192.168.10.4"
    cfg.cameras = {}
    drivers.append(_FakeDriver())

    from heron.robot.trossen import TrossenStationary

    robot = TrossenStationary(cfg)
    return robot, drivers[-1], cfg


def test_orientation_is_table_facing_not_inherited(rig):
    """The ready pose is horizontal; commands must not adopt it."""
    robot, drv, cfg = rig
    robot.move_cartesian("right", np.array([0.3, 0.0, 0.015]))
    assert drv.commands, "no motion commanded"
    for cmd in drv.commands:
        # 0.01 rad, not exact: the pose now comes back through a numerical IK
        # and FK round trip. The old stub echoed the commanded pose verbatim,
        # so exactness there was measuring nothing.
        assert np.allclose(cmd[3:6], cfg.arms["right"].approach_rvec, atol=0.01), \
            "wrist orientation drifted from the configured approach pose"


def test_descent_is_staged_over_the_target(rig):
    """Down-and-over, never a diagonal sweep across the table."""
    robot, drv, cfg = rig
    target = np.array([0.30, -0.20, cfg.table_z + 0.03])
    robot.move_cartesian("right", target, seconds=2.0)
    path = np.array([c[:3] for c in drv.commands])
    assert len(path) >= 2, "a descent from a high pose must be staged"
    assert np.allclose(path[-1], target, atol=5e-4), "final waypoint must be the goal"
    # Every waypoint before the last sits well above the goal height.
    assert (path[:-1, 2] >= target[2] + 0.10).all(), \
        "an intermediate waypoint dips toward the table — that is the colliding path"
    # The last leg is purely vertical.
    assert np.allclose(path[-2][:2], target[:2], atol=5e-4), \
        "final approach must descend straight down"


def test_clearance_is_measured_from_the_table(rig):
    """Crossing at the CURRENT height pins the wrist into IK-infeasible poses
    near full extension; the crossing height must come from the table."""
    from heron.robot.trossen import TRAVEL_CLEARANCE_M

    robot, drv, cfg = rig
    robot.move_cartesian("right", np.array([0.30, 0.0, cfg.table_z + 0.03]))
    crossing = np.array([c[:3] for c in drv.commands])[0]
    assert crossing[2] == pytest.approx(cfg.table_z + TRAVEL_CLEARANCE_M, abs=5e-4), \
        f"crossed at {crossing[2]:.3f}, not at the table-relative travel height"


def test_move_clear_of_the_table_is_direct(rig):
    """Anything already above the travel height needs no staging."""
    robot, drv, _ = rig
    # 0.15 m, not 0.30: with a straight-down wrist this arm cannot hold a
    # pose much above the travel height at all. Measured with our own IK,
    # which agrees with the controller — (0.35, 0.10, 0.20), (0.30, 0, 0.25)
    # and (0.40, 0, 0.20) all have no solution. safety.workspace_z going up to
    # 0.40 describes where the tool can be, not where it can point downward.
    robot.move_cartesian("right", np.array([0.35, 0.10, 0.15]))
    assert len(drv.commands) == 1, f"expected a direct move, got {len(drv.commands)}"


def test_unstaged_is_a_single_command(rig):
    robot, drv, _ = rig
    robot.move_cartesian("right", np.array([0.3, 0.0, 0.02]), staged=False)
    assert len(drv.commands) == 1


# -- staging a move that is already vertical ----------------------------------
#
# Staging exists to keep a DIAGONAL off the table. A move straight down has no
# diagonal, and staging it lifts the tool out of the descent it was asked to
# make. descend_to_contact issues one such move per 5 mm probing step, so this
# is not a rare case: it is the whole feel-for-the-surface loop.

def test_a_move_straight_down_is_not_staged(rig):
    """Measured in the MuJoCo twin before this fix: a commanded 5 mm descent
    moved the tool 232 mm, up 114 mm and back down. Repeated once per probing
    step, that is the judder the operator saw during a place."""
    robot, drv, cfg = rig
    xy = (0.30, -0.20)
    robot.move_cartesian("right", np.array([xy[0], xy[1], cfg.table_z + 0.03]))
    n_after_approach = len(drv.commands)

    robot.move_cartesian("right", np.array([xy[0], xy[1], cfg.table_z + 0.025]))
    legs = np.array([c[:3] for c in drv.commands[n_after_approach:]])
    assert len(legs) == 1, f"a 5 mm descent was staged into {len(legs)} legs"
    assert legs[0][2] < cfg.table_z + 0.03, "the tool was commanded upward mid-descent"


def test_a_sideways_move_is_still_staged(rig):
    """The guard must not swallow the case staging was written for."""
    robot, drv, cfg = rig
    robot.move_cartesian("right", np.array([0.30, -0.20, cfg.table_z + 0.03]))
    n = len(drv.commands)
    robot.move_cartesian("right", np.array([0.30, 0.10, cfg.table_z + 0.03]))
    assert len(drv.commands) - n >= 2, "a lateral move across the table was not staged"


def test_vertical_is_judged_on_commanded_poses_not_measured_ones():
    """The measured pose carries the servo's tracking error — 40 mm in xy at the
    hover in the twin. Testing against it answers "not vertical" for every real
    descent, which is how the first version of this fix changed nothing."""
    from heron.robot.trossen import VERTICAL_MOVE_TOLERANCE_M, is_vertical_move

    a = np.array([0.376, 0.122, 0.140])
    assert is_vertical_move(a, np.array([0.376, 0.122, 0.019]))
    assert is_vertical_move(a, np.array([0.376, 0.122, 0.400])), "a retreat counts too"
    assert not is_vertical_move(a, np.array([0.376 + 0.041, 0.122, 0.019]))
    assert not is_vertical_move(None, a), "nothing commanded yet: stage it"
    edge = a + np.array([VERTICAL_MOVE_TOLERANCE_M * 0.9, 0.0, -0.1])
    assert is_vertical_move(a, edge)


def test_homing_forgets_the_commanded_pose(rig):
    """home() moves in joint space, so the next cartesian move starts from a
    pose no cartesian command described and has to be staged."""
    from heron.robot.trossen import is_vertical_move

    robot, _drv, cfg = rig
    robot.move_cartesian("right", np.array([0.30, 0.0, cfg.table_z + 0.03]))
    robot.home("right")
    assert not is_vertical_move(robot._last_target.get("right"),
                                np.array([0.30, 0.0, cfg.table_z + 0.02]))


def test_interpolates_in_joint_space(rig):
    """Cartesian interpolation re-solves IK per waypoint and aborts on the first
    unsolvable one — every calibration pose failed that way."""
    robot, drv, _ = rig
    robot.move_cartesian("right", np.array([0.3, 0.0, 0.015]))
    # Nothing is left to choose: Heron solves IK itself and sends joint
    # targets, so the controller never receives a cartesian goal to
    # re-interpolate. _FakeDriver raises if one ever is.
    assert drv.joint_commands, "no joint waypoints were commanded"
    assert all(len(q) == 6 for q in drv.joint_commands)


def test_speed_scale_slows_every_leg(rig):
    robot, drv, cfg = rig
    cfg.safety.speed_scale = 0.5
    robot.move_cartesian("right", np.array([0.3, 0.0, 0.015]), seconds=2.0)
    assert drv.durations, "no motion commanded"
    assert min(drv.durations) >= 2.0, f"legs ran faster than the scale allows: {drv.durations}"


# -- approach-angle fallback --------------------------------------------------

def test_rotvec_inverts_rodrigues():
    import numpy as np

    from heron.robot.trossen import _rodrigues, _rotvec

    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(500):
        v = rng.normal(size=3)
        v = v / np.linalg.norm(v) * rng.uniform(0, np.pi)
        worst = max(worst, float(np.abs(_rodrigues(_rotvec(_rodrigues(v))) - _rodrigues(v)).max()))
    assert worst < 1e-8


def test_tilt_leans_toward_the_base_by_the_requested_angle():
    """The arm gains reach by leaning back, not forward. Getting the sign wrong
    would tilt it further out and make the IK failure worse."""
    import numpy as np

    from heron.robot.trossen import _rodrigues, _tilt_toward_base

    down = np.array([0.0, np.pi / 2, 0.0])
    target = np.array([0.348, 0.178, 0.205])   # the point with no vertical solution
    unit = target[:2] / np.linalg.norm(target[:2])
    for deg in (15, 30, 45, 60):
        axis = _rodrigues(_tilt_toward_base(down, target, np.radians(deg))) @ np.array([1.0, 0, 0])
        off = np.degrees(np.arccos(np.clip(-axis[2], -1, 1)))
        assert abs(off - deg) < 1e-6, f"{deg} deg requested, {off:.2f} applied"
        assert float(np.dot(axis[:2], unit)) < 0, "tilted away from the base"


def test_tilt_is_a_no_op_directly_above_the_base():
    import numpy as np

    from heron.robot.trossen import _tilt_toward_base

    down = np.array([0.0, np.pi / 2, 0.0])
    got = _tilt_toward_base(down, np.array([0.0, 0.0, 0.3]), np.radians(45))
    assert np.allclose(got, down), "no vertical plane exists there; leave it alone"


def test_zero_tilt_returns_the_orientation_untouched():
    import numpy as np

    from heron.robot.trossen import _tilt_toward_base

    down = np.array([0.0, np.pi / 2, 0.0])
    assert np.allclose(_tilt_toward_base(down, np.array([0.3, 0.2, 0.1]), 0.0), down)


def test_approach_height_drops_to_something_the_arm_can_hold(rig):
    """table_z + APPROACH_HEIGHT_M is a constant chosen for a depth-free rig. At
    some reaches this arm cannot point a gripper downward that high at all —
    the first real pick died with no IK solution at 0.205 m while the block
    beneath it was reachable."""
    import numpy as np

    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.primitives import APPROACH_HEIGHT_M, approach_pose

    robot, _drv, cfg = rig
    ctx = SkillContext(robot=robot, belief=None, grounder=None, cfg=cfg,
                       log=EpisodeLogger("/tmp/heron-test-episodes", "approach"))
    xy = (0.347, 0.175)
    want = cfg.table_z + APPROACH_HEIGHT_M
    got = approach_pose(ctx, xy, perceived_z=cfg.table_z, arm="right")

    assert got[2] <= want + 1e-9, "never higher than asked for"
    assert robot.reachable("right", got), "chose a height the arm cannot hold"
    if not robot.reachable("right", np.array([xy[0], xy[1], want])):
        assert got[2] < want, "should have come down when the wanted height was unreachable"


def test_approach_height_is_unchanged_when_the_wanted_height_is_fine(rig):
    """The search must not lower the hover for its own sake: descending early
    means transiting closer to whatever else is on the table."""
    import numpy as np

    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.primitives import APPROACH_HEIGHT_M, approach_pose

    robot, _drv, cfg = rig
    ctx = SkillContext(robot=robot, belief=None, grounder=None, cfg=cfg,
                       log=EpisodeLogger("/tmp/heron-test-episodes", "approach"))
    xy = (0.30, 0.0)
    want = cfg.table_z + APPROACH_HEIGHT_M
    if robot.reachable("right", np.array([xy[0], xy[1], want])):
        got = approach_pose(ctx, xy, perceived_z=cfg.table_z, arm="right")
        assert got[2] == pytest.approx(want)


def test_no_local_rig_is_built_when_the_sidecar_serves_every_camera(monkeypatch):
    """Constructing one anyway makes librealsense try to seize devices the
    sidecar already owns, and every capture fails with "no device connected" —
    for cameras nothing was going to ask it for."""
    import sys
    import types as _t

    from heron.config import HeronConfig

    fake = _t.ModuleType("trossen_arm")

    class _Drv:
        def configure(self, *a, **k): pass
        def set_all_modes(self, *a, **k): pass
        def get_all_positions(self): return np.zeros(7)
        def get_gripper_position(self): return 0.0
        def cleanup(self): pass

    fake.TrossenArmDriver = _Drv
    fake.Model = _t.SimpleNamespace(wxai_v0="wxai_v0")
    fake.StandardEndEffector = _t.SimpleNamespace(wxai_v0_follower="ee")
    fake.Mode = _t.SimpleNamespace(position="position", external_effort="effort")
    fake.InterpolationSpace = _t.SimpleNamespace(cartesian="cartesian", joint="joint")
    monkeypatch.setitem(sys.modules, "trossen_arm", fake)

    def explode(*a, **k):
        raise AssertionError("built a local rig for cameras the sidecar owns")

    monkeypatch.setattr("heron.robot.trossen.RealSenseRig", explode)

    cfg = HeronConfig()
    cfg.arms = {"right": cfg.arms["right"]}
    for cam in cfg.cameras.values():
        cam.depth_server = "http://127.0.0.1:8766"
        cam.index = None

    from heron.robot.trossen import TrossenStationary, _NoRig

    robot = TrossenStationary(cfg)
    assert isinstance(robot._rig, _NoRig)
    with pytest.raises(RuntimeError, match="not served by the depth sidecar"):
        robot._rig.capture("cam_nowhere")


def test_a_short_segment_is_not_charged_as_a_long_one():
    """Rig, 14 episodes: 19 s inside one pick, almost none of it model latency.

    min_move_seconds stretched every segment to the same duration, so the 50 mm
    descent and the 300 mm traverse both cost 2 s of commanded time — 3.3 s of
    blocking wait each after speed_scale. The cap says the thing that was meant.
    """
    import numpy as np

    from heron.config import SafetyCfg
    from heron.robot.safety import SafeRobot

    class Stub:
        arms, cameras = ("right",), ()

        def get_cartesian(self, arm):
            return np.array([0.30, 0.0, 0.15])

    old = SafeRobot(Stub(), SafetyCfg(min_move_seconds=2.0))
    new = SafeRobot(Stub(), SafetyCfg(min_move_seconds=0.4, max_tool_speed_mps=0.12))

    traverse = np.array([0.30, 0.30, 0.15])   # 300 mm
    descent = np.array([0.30, 0.0, 0.10])     # 50 mm

    # The long move is untouched: the cap IS the speed it already ran at.
    assert old._segment_seconds("right", traverse, 2.5) == pytest.approx(2.5)
    assert new._segment_seconds("right", traverse, 2.5) == pytest.approx(2.5)

    # The short one stops paying for distance it does not cover.
    assert old._segment_seconds("right", descent, 1.5) == pytest.approx(2.0)
    assert new._segment_seconds("right", descent, 1.5) == pytest.approx(0.4167, abs=1e-3)

    # No segment moves faster than the cap, whatever the caller asks for.
    assert new._segment_seconds("right", traverse, 0.1) >= 0.30 / 0.12 - 1e-9

    # Unset, the old arithmetic returns exactly.
    off = SafeRobot(Stub(), SafetyCfg(min_move_seconds=2.0))
    assert off._segment_seconds("right", descent, 1.5) == pytest.approx(2.0)
