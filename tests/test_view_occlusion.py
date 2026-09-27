"""The gripper that did the work must not be what hides it.

`place` finishes by lifting to a hover DIRECTLY ABOVE what it just put down.
Verification then asks the overhead camera to confirm the placement — through
the gripper. Measured across the twin's lifelong runs: 40 groundings failed as
"not found in ['cam_high']" with the tool parked at exactly the xy being looked
for. The object was there every time.

The geometry needs no perception: the camera's pose is calibrated, the target's
is believed, and the tool's is forward kinematics.
"""
from __future__ import annotations

import numpy as np
import pytest

from heron.skills.sensing import (VIEW_BLOCK_RADIUS_M, VIEW_CLEAR_OFFSET_M,
                                  clear_view_offset, occludes_view)

# The twin's overhead camera, from the rig's own calibration.
CAM = np.array([0.488, 0.298, 1.035])


def test_the_hover_directly_above_a_placement_blocks_it():
    """The measured case: a block placed at (0.363, 0.284) and the tool left
    hovering at that same xy, 0.14 m up."""
    target = np.array([0.363, 0.284, 0.016])
    tool = np.array([0.363, 0.284, 0.140])
    assert occludes_view(CAM, target, tool, VIEW_BLOCK_RADIUS_M)


def test_an_arm_parked_well_to_one_side_does_not():
    target = np.array([0.363, 0.284, 0.016])
    tool = np.array([0.363, 0.284 - 0.30, 0.140])
    assert not occludes_view(CAM, target, tool, VIEW_BLOCK_RADIUS_M)


def test_an_arm_behind_the_camera_or_past_the_target_is_not_in_the_way():
    """Only the segment between them matters. Clamping the projection is what
    stops a tool underneath the table from counting as an obstruction."""
    target = np.array([0.363, 0.284, 0.016])
    assert not occludes_view(CAM, target, CAM + np.array([0, 0, 0.5]),
                             VIEW_BLOCK_RADIUS_M)
    below = np.array([0.363, 0.284, -0.30])
    assert not occludes_view(CAM, target, below, VIEW_BLOCK_RADIUS_M)


def test_stepping_aside_actually_clears_the_line():
    """The offset has to be perpendicular to the sightline seen from above.
    Lifting would not help — the tool is already above the target, which is
    exactly where the line is."""
    target = np.array([0.363, 0.284, 0.016])
    tool_h = 0.140
    xy = clear_view_offset(CAM, target, VIEW_CLEAR_OFFSET_M)
    moved = np.array([xy[0], xy[1], tool_h])
    assert not occludes_view(CAM, target, moved, VIEW_BLOCK_RADIUS_M), \
        f"still blocking after stepping aside to {np.round(moved, 3)}"


def test_lifting_straight_up_would_not_have_helped():
    """Pins why the fix is sideways. The old behaviour lifted to the hover and
    the hover is on the line."""
    target = np.array([0.363, 0.284, 0.016])
    for z in (0.10, 0.14, 0.18, 0.25):
        assert occludes_view(CAM, target, np.array([0.363, 0.284, z]),
                             VIEW_BLOCK_RADIUS_M), f"z={z} should still block"


def test_the_step_is_bigger_than_the_thing_it_is_stepping_out_of():
    assert VIEW_CLEAR_OFFSET_M > VIEW_BLOCK_RADIUS_M


@pytest.mark.parametrize("angle", np.linspace(0, 2 * np.pi, 12, endpoint=False))
def test_it_clears_the_line_wherever_the_object_is(angle):
    """The rig's camera is off to one corner, so the sightline direction changes
    across the table. A fixed 'move in +y' would work near the camera's azimuth
    and fail at right angles to it."""
    target = np.array([0.32 + 0.10 * np.cos(angle), 0.10 * np.sin(angle), 0.016])
    tool = np.array([target[0], target[1], 0.14])
    assert occludes_view(CAM, target, tool, VIEW_BLOCK_RADIUS_M)
    xy = clear_view_offset(CAM, target, VIEW_CLEAR_OFFSET_M)
    assert not occludes_view(CAM, target, np.array([xy[0], xy[1], 0.14]),
                             VIEW_BLOCK_RADIUS_M)


# -- one viewpoint was never enough -------------------------------------------

def test_a_second_overhead_would_not_have_helped_at_all():
    """ELEVATION DECIDES THIS, NOT AZIMUTH, and that is the whole reason the
    twin's second camera is low.

    `place` leaves the tool hovering DIRECTLY above what it put down, so every
    camera looking down at that spot is behind the gripper whichever corner it
    sits in. A mirror of cam_high across the table is blocked in every sampled
    placement — exactly as often as cam_high itself. Buying a second camera on
    that reasoning would have bought nothing.
    """
    mirror = np.array([0.488, -0.298, 1.035])
    blocked = 0
    spots = [(x, y) for x in np.linspace(0.18, 0.46, 8)
             for y in np.linspace(-0.30, 0.30, 13)]
    for x, y in spots:
        target = np.array([x, y, 0.0])
        tool = np.array([x, y, 0.14])
        assert occludes_view(CAM, target, tool, VIEW_BLOCK_RADIUS_M)
        blocked += occludes_view(mirror, target, tool, VIEW_BLOCK_RADIUS_M)
    assert blocked == len(spots), \
        f"a mirrored overhead saw past the gripper {len(spots) - blocked} times — rewrite this"


def test_the_low_front_view_sees_what_the_overhead_cannot():
    """The twin's cam_low, at 34 degrees of elevation. Measured on the three
    placements that actually failed for this reason, and across the table."""
    from heron.orchestrator.gemini import workspace_box  # noqa: F401  (same rig, same numbers)

    low = np.array([0.0, -0.50, 0.40])
    both = 0
    spots = [(x, y) for x in np.linspace(0.18, 0.46, 8)
             for y in np.linspace(-0.30, 0.30, 13)]
    for x, y in spots:
        target = np.array([x, y, 0.0])
        tool = np.array([x, y, 0.14])
        both += (occludes_view(CAM, target, tool, VIEW_BLOCK_RADIUS_M)
                 and occludes_view(low, target, tool, VIEW_BLOCK_RADIUS_M))
    assert both <= 2, f"both cameras blocked at {both} of {len(spots)} placements"

    # The three that failed in the eight-episode run, to the millimetre.
    for x, y, z in ((0.424, 0.232, 0.001), (0.338, -0.068, -0.007), (0.423, 0.161, -0.007)):
        target = np.array([x, y, z])
        tool = np.array([x, y, z + 0.14])
        assert occludes_view(CAM, target, tool, VIEW_BLOCK_RADIUS_M)
        assert not occludes_view(low, target, tool, VIEW_BLOCK_RADIUS_M)


def test_the_twin_grounds_from_one_camera_by_choice_not_by_accident():
    """The second viewpoint is OFF as of 2026-08-02 — a decision, not a defect.

    The camera is still in the scene and the reserve logic still stands; the
    config entry is commented, which is the one line that brings it back. This
    pins that the default really is one camera, so nobody has to read a config
    to find out, and that a WRIST camera is never enlisted either way: reaching
    one is a motion and a decision rather than a look, and the right one is
    bolted to the arm that does the occluding.
    """
    import pytest as _pytest

    _pytest.importorskip("mujoco")
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.robot.safety import SafeRobot
    from heron.robot.trossen_sim import TrossenSim
    from heron.skills import SkillContext
    from heron.skills.sensing import _usable_cameras

    cfg = HeronConfig.load("configs/trossen_sorting.yaml")
    sim = TrossenSim(cfg, scene=cfg.sim_scene)
    ctx = SkillContext(robot=SafeRobot(sim, cfg.safety), belief=None, grounder=None,
                       cfg=cfg, log=EpisodeLogger("/tmp/heron-test-episodes", "cams"))
    # cam_low was re-enabled 2026-08-04 as the verification witness: two
    # usable cameras is now the deliberate configuration, and per-look cost
    # stays contained by the second-chance-not-second-habit rule.
    assert _usable_cameras(ctx) == ["cam_high", "cam_low"]
    assert "cam_right_wrist" in sim.cameras, "the wrist is present and still not enlisted"


def test_a_declared_second_fixed_camera_would_be_enlisted():
    """The mechanism outlives the decision to switch the camera off. Written
    against a stub rather than the config, so turning cam_low on or off cannot
    quietly delete the coverage."""
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.sensing import _usable_cameras

    cfg = HeronConfig()

    class Rig:
        cameras = ["cam_high", "cam_low", "cam_right_wrist"]

        @staticmethod
        def camera_calibrated(name):
            return not name.endswith("_wrist")

    ctx = SkillContext(robot=Rig(), belief=None, grounder=None, cfg=cfg,
                       log=EpisodeLogger("/tmp/heron-test-episodes", "cams"))
    assert _usable_cameras(ctx) == ["cam_high", "cam_low"]


def test_the_real_rig_is_not_told_it_has_a_camera_it_has_not_calibrated():
    """The twin's second viewpoint is simulated, and saying so is the price of
    it being useful. `configs/abaka.yaml` is the rig; nothing here may leak."""
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.sensing import _usable_cameras

    cfg = HeronConfig.load("configs/abaka.yaml")

    class Rig:  # the real backend builds its camera list from the config
        cameras = list(cfg.cameras)

    ctx = SkillContext(robot=Rig(), belief=None, grounder=None, cfg=cfg,
                       log=EpisodeLogger("/tmp/heron-test-episodes", "cams"))
    assert _usable_cameras(ctx) == ["cam_high"]


def test_two_views_from_almost_the_same_place_do_not_make_a_depth():
    """The mock's cameras sit 0.05 m apart, both looking straight down from
    ~0.8 m. Triangulating from that put a bowl 59 mm off in z, which is enough
    to turn `on` from true to false.

    REPROJECTION ERROR DOES NOT CATCH THIS, and that is the point of checking the
    geometry instead. The solution fits both images well; it is only badly
    CONSTRAINED along the shared line of sight, and a residual measured in pixels
    cannot see that.
    """
    from heron.robot.mock import MockRobot, standard_scene
    from heron.skills.sensing import TRIANGULATION_MIN_PARALLAX_DEG, _parallax_deg

    mock = MockRobot()
    standard_scene(mock)
    hits = [(0.9, c, (0, 0), mock.capture(c)) for c in ("cam_high", "cam_low")]
    assert _parallax_deg(hits) < TRIANGULATION_MIN_PARALLAX_DEG


def test_the_twins_pair_is_wide_enough_to_triangulate():
    """Read from the scene file rather than by capturing, because the camera is
    in the twin's world while its config entry is commented out. The geometry is
    what this pins, and the geometry does not depend on whether we are currently
    paying for the view."""
    import re as _re
    from pathlib import Path as _Path

    from heron.skills.sensing import TRIANGULATION_MIN_PARALLAX_DEG

    xml = _Path("assets/trossen_sorting.xml").read_text()
    poses = {}
    for name in ("cam_high", "cam_low"):
        m = _re.search(rf'camera name="{name}"[^>]*pos="([^"]+)"', xml)
        assert m, f"{name} is not in the scene"
        poses[name] = np.array([float(v) for v in m.group(1).split()])
    gap = float(np.linalg.norm(poses["cam_high"] - poses["cam_low"]))
    ranges = [float(np.linalg.norm(p - np.array([0.32, 0.0, 0.0]))) for p in poses.values()]
    parallax = np.degrees(2 * np.arcsin(min(1.0, gap / sum(ranges))))
    assert parallax > 3 * TRIANGULATION_MIN_PARALLAX_DEG, f"only {parallax:.0f} degrees apart"


def test_a_camera_with_k_and_a_pose_can_already_project():
    """`proj` arrived as a field for the depth-free rigs, where a DLT fit is the
    only calibration there is, so nothing ever derived it for a camera that had
    intrinsics and an extrinsic instead. Both of the twin's fixed cameras carried
    `proj = None`, and the triangulation path — the whole reason a second camera
    is worth its detection call — never ran once."""
    import numpy as _np

    from heron.types import Frame, triangulate

    k = _np.array([[400.0, 0, 320.0], [0, 400.0, 240.0], [0, 0, 1.0]])
    a, b = _np.eye(4), _np.eye(4)
    a[:3, 3] = [0.0, 0.0, 1.0]
    a[:3, :3] = _np.diag([1.0, -1.0, -1.0])
    b[:3, 3] = [0.8, 0.0, 0.6]
    z = _np.array([-0.8, 0.0, -0.6]) / _np.linalg.norm([-0.8, 0.0, -0.6])
    x = _np.cross(z, [0.0, 0.0, 1.0])
    x /= _np.linalg.norm(x)
    b[:3, :3] = _np.column_stack([x, _np.cross(z, x), z])

    fa = Frame(camera="a", rgb=_np.zeros((480, 640, 3), _np.uint8), intrinsics=k, t_base_cam=a)
    fb = Frame(camera="b", rgb=_np.zeros((480, 640, 3), _np.uint8), intrinsics=k, t_base_cam=b)
    assert fa.proj is not None and fb.proj is not None

    truth = _np.array([0.10, -0.05, 0.03])
    views = []
    for f in (fa, fb):
        q = f.proj @ _np.array([*truth, 1.0])
        views.append((f.proj, (q[0] / q[2], q[1] / q[2])))
    assert _np.linalg.norm(triangulate(views) - truth) < 1e-6


def test_a_supplied_projection_still_wins():
    """A DLT fitted against the rig is a measurement; the derivation is an
    inference from two others."""
    import numpy as _np

    from heron.types import Frame

    mine = _np.arange(12, dtype=float).reshape(3, 4)
    f = Frame(camera="a", rgb=_np.zeros((8, 8, 3), _np.uint8),
              intrinsics=_np.eye(3), t_base_cam=_np.eye(4), proj=mine)
    assert _np.array_equal(f.proj, mine)


def test_triangulation_is_the_fallback_not_the_preference():
    """It took a measurement to find this out, because the reverse is what the
    theory suggests: two views constrain three unknowns, one view plus a plane
    assumes the object is on the plane. Ordered that way for months.

    tools/probe_grounding.py, 48 readings over identical layouts, arm parked:

        single view    median 2.4 mm   p90  8.4 mm   worst 13.5 mm   0/48 over 30 mm
        triangulated   median 2.2 mm   p90 15.9 mm   worst 51.6 mm   2/48 over 30 mm

    The medians tie and every tail statistic is worse, because triangulation
    assumes the two views point at the same physical point and what they report
    is each view's own box centroid. This pins the ORDER, which is the thing
    that measurement changed.
    """
    import inspect

    from heron.skills import sensing

    # ground_entity is the budget wrapper; the order lives in the body.
    src = inspect.getsource(sensing._ground_entity)
    single = src.index("deprojection from the strongest single view")
    tri = src.index("TRIANGULATION IS THE FALLBACK")
    assert single < tri, \
        "single-view deprojection must be tried first; the tails say so"
