"""The twin only answers "would this plan work?" if it is the same cell.

Three things it was not, each of which quietly disabled something the twin was
built to check:

  * the wrist camera was attached to link_6 with a transform measured from the
    TOOL, putting it 156 mm back along the approach axis;
  * the arm sagged under its own weight, while the real controller compensates
    gravity, so commanded and measured poses differed by tens of millimetres;
  * the surface reading took the highest thing in the probe window, and the
    highest thing is the gripper.

These run the real model, so they are slower than the rest of the suite. That is
the point: every one of these was invisible to a unit test and visible in one
frame of the twin.
"""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("mujoco", reason="the twin needs MuJoCo")

from heron.config import HeronConfig  # noqa: E402

CONFIG = "configs/trossen_sorting.yaml"


@pytest.fixture(scope="module")
def sim():
    from heron.robot.trossen_sim import TrossenSim

    cfg = HeronConfig.load(CONFIG)
    s = TrossenSim(cfg, scene=cfg.sim_scene, record_camera=None)
    yield s
    s.shutdown()


def test_the_wrist_camera_is_where_the_hand_eye_says_it_is(sim):
    """T_ee_cam is measured from the tool frame; MuJoCo can only attach a camera
    to a body, and the last body's origin is 156 mm back along the approach
    axis. Writing the transform straight into link_6 put the camera 263 mm from
    the tool instead of 115 mm — close enough to look plausible in a render, far
    enough that it saw nothing but the gripper's own mount."""
    import mujoco

    handeye = np.load("calibration/right_wrist_handeye.npz", allow_pickle=True)["T_ee_cam"]
    want = float(np.linalg.norm(handeye[:3, 3]))

    sim.move_cartesian("right", np.array([0.30, 0.10, 0.06]), seconds=1.0)
    cid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, "cam_right_wrist")
    cam = np.array(sim.data.cam_xpos[cid])
    tool = sim._kin.fk(sim.joints())[:3, 3]
    got = float(np.linalg.norm(cam - tool))
    assert abs(got - want) < 0.005, (
        f"wrist camera is {got * 1000:.0f} mm from the tool, hand-eye says "
        f"{want * 1000:.0f} mm")


def test_the_twin_does_not_track_its_own_commands(sim):
    """A KNOWN LIMITATION, pinned so nobody reads commanded == measured.

    The arm is not gravity compensated while the real controller is, so it sags:
    tens of millimetres in xy at a hover. Three things downstream are wrong
    because of it — `carry_offset` reads the sag as the object riding off-centre
    in the gripper, the wrist camera looks up to 41 mm off the point it was
    asked about, and no commanded-vs-measured comparison in the twin means
    anything.

    Setting gravcomp="1" on the arm bodies removes all three and makes the task
    WORSE: 1-5 mm from the aim becomes 64-186 mm, because `pick` has been
    relying on the sag. workspace_z[0] is 0.01 against a table at -0.015, so
    grasp_z clamps to the top 6 mm of a 30 mm block, and 11 mm of downward sag
    was quietly turning that into a grasp at the block's centre.

    This test fails the day the grasp height is fixed and compensation turned
    on. That is when to delete it — not before.
    """
    worst = 0.0
    for p in ([0.376, 0.122, 0.140], [0.30, 0.145, 0.120]):
        target = np.array(p)
        sim.move_cartesian("right", target, seconds=1.5)
        got = sim.get_cartesian("right")
        worst = max(worst, float(np.linalg.norm(got[:2] - target[:2])))
    assert worst > 0.010, (
        f"worst xy tracking error is now {worst * 1000:.1f} mm — if gravity "
        "compensation was turned on, check the grasp height first: see the "
        "comment in tools/build_sim_scene.py")


def test_the_surface_under_the_tool_is_the_surface_not_the_gripper(sim):
    """The gripper is in its own camera's view and it is the highest thing
    there: hovering 0.13 m over a block, 66% of the probe window was gripper
    between 0.158 and 0.185, while the block's top was 0.015. Taking the 90th
    percentile of that returned the mount, and place would have released from
    above where it started."""
    from heron.episode import EpisodeLogger
    from heron.robot.safety import SafeRobot
    from heron.skills import SkillContext
    from heron.skills.primitives import surface_height_below

    cfg = HeronConfig.load(CONFIG)
    ctx = SkillContext(robot=SafeRobot(sim, cfg.safety), belief=None, grounder=None,
                       cfg=cfg, log=EpisodeLogger("/tmp/heron-test-episodes", "twin"))

    checked = 0
    for name, xy, top_offset in (("white_plate", (0.375, 0.135), 0.008),
                                 ("red_block", (0.245, 0.115), 0.015)):
        z = next((float(z) for z in np.arange(0.14, 0.03, -0.01)
                  if sim.reachable("right", np.array([xy[0], xy[1], z]))), None)
        if z is None:
            continue
        sim.move_cartesian("right", np.array([xy[0], xy[1], z]), seconds=1.5)
        got, note = surface_height_below(ctx, "right", xy)
        assert got is not None, f"{name}: {note}"
        true_top = float(sim.body_xyz(name)[2]) + top_offset
        assert abs(got - true_top) < 0.005, (
            f"{name}: read {got:.4f}, true top {true_top:.4f}")
        assert got < z, "read a surface at or above the tool — that is the gripper"
        checked += 1
    assert checked, "no reachable hover to measure from"


def test_a_failed_wrist_read_says_where_the_tool_actually_was(sim):
    """This camera is 55 mm off the approach axis with a 40-degree field, so at
    a 14 cm hover it covers barely 10 cm of table. "saw only 10 points" gives no
    way to tell a tool that ended up somewhere else from a depth failure — and
    the answer, measured, was 41 mm of tracking error."""
    from heron.episode import EpisodeLogger
    from heron.robot.safety import SafeRobot
    from heron.skills import SkillContext
    from heron.skills.primitives import surface_height_below

    cfg = HeronConfig.load(CONFIG)
    ctx = SkillContext(robot=SafeRobot(sim, cfg.safety), belief=None, grounder=None,
                       cfg=cfg, log=EpisodeLogger("/tmp/heron-test-episodes", "twin"))
    sim.move_cartesian("right", np.array([0.245, 0.115, 0.05]), seconds=1.5)
    # Ask about a point far from where the tool is standing.
    got, note = surface_height_below(ctx, "right", (0.245, -0.30))
    assert got is None
    assert "mm from the point asked about" in note, note


def test_the_wrist_reads_the_surface_past_its_own_cargo():
    """During a place the gripper holds the thing being placed, directly under
    the camera and over the aim point — so the probe used to measure the CARGO's
    top, not the support. Measured: hovering at 0.17 with a block in the jaws,
    surface_z came back 0.1605 (the block's own top), was refused as implausible,
    and every place in the twin fell back to feeling its way down. The
    wrist-depth path never once ran for exactly the motion it was built for.

    With the cargo's footprint cut around the tool, the annulus that remains is
    the support: measured 0.0009 against a plate top truly at 0.0009.
    """
    import numpy as np
    import pytest as _pytest

    _pytest.importorskip("mujoco")
    from heron.belief import BeliefStore
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.robot.safety import SafeRobot
    from heron.robot.trossen_sim import TrossenSim
    from heron.skills import SkillContext
    from heron.skills.primitives import pick, surface_height_below
    from heron.types import EntityDecl

    cfg = HeronConfig.load("configs/trossen_sorting.yaml")
    sim = TrossenSim(cfg, scene=cfg.sim_scene)
    belief = BeliefStore()
    belief.declare_entities([EntityDecl(id="red_block", description="the red block",
                                        role="object")])
    ctx = SkillContext(robot=SafeRobot(sim, cfg.safety), belief=belief, grounder=None,
                       cfg=cfg, log=EpisodeLogger("/tmp/heron-test-episodes", "wrist"))
    p = sim.body_xyz("red_block")
    belief.update_track("red_block", xyz_base=tuple(p), confidence=0.9,
                        from_sighting=True, span_m=0.032)
    assert pick(ctx, entity="red_block", arm="right").ok

    plate = sim.body_xyz("white_plate")
    for z in (0.14, 0.13, 0.12, 0.11):
        try:
            sim.move_cartesian("right", np.array([plate[0], plate[1], z]), seconds=1.0)
            break
        except Exception:
            continue
    surface, note = surface_height_below(ctx, "right", plate[:2])
    assert surface is not None, note
    truth = float(plate[2]) + 0.008          # cylinder half-height from the scene
    assert abs(surface - truth) < 0.005, (surface, truth, note)
