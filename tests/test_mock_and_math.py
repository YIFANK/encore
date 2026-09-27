import numpy as np
import pytest

from heron.orchestrator.scripted import project
from heron.robot.mock import MockRobot, standard_scene
from heron.robot.workspace import solve_extrinsics


def test_pick_place_physics():
    m = MockRobot()
    standard_scene(m)
    m.move_cartesian("left", np.array([0.05, 0.10, 0.06]))
    m.set_gripper("left", 0.0)
    assert m._arms["left"].holding == "red_block"
    assert m.get_gripper("left")["effort"] > 1.0
    m.move_cartesian("left", np.array([0.05, -0.15, 0.15]))
    m.set_gripper("left", 0.08)
    assert m._arms["left"].holding is None
    assert m.supporting_container("red_block") == "blue_bowl"


def test_grasp_miss_injection():
    m = MockRobot()
    standard_scene(m)
    m.inject.grasp_misses = 1
    m.move_cartesian("left", np.array([0.05, 0.10, 0.06]))
    m.set_gripper("left", 0.0)
    assert m._arms["left"].holding is None
    assert m.get_gripper("left")["effort"] < 0.5
    m.set_gripper("left", 0.08)
    m.set_gripper("left", 0.0)
    assert m._arms["left"].holding == "red_block"


def test_render_project_deproject_roundtrip():
    m = MockRobot()
    standard_scene(m)
    frame = m.capture("cam_high")
    world = m.gt_xyz("red_block") + np.array([0, 0, 0.02])
    px = project(frame, world)
    assert px is not None
    back = frame.deproject(*px)
    assert back is not None
    assert np.linalg.norm(back[:2] - world[:2]) < 0.01
    assert abs(back[2] - world[2]) < 0.02


def test_solve_extrinsics_recovers_transform():
    rng = np.random.default_rng(0)
    angle = 0.4
    rot = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    t = np.array([0.1, -0.2, 0.5])
    p_cam = rng.uniform(-0.3, 0.3, size=(9, 3))
    p_world = (rot @ p_cam.T).T + t
    T, rms = solve_extrinsics(p_cam, p_world)
    assert rms < 1e-9
    assert np.allclose(T[:3, :3], rot, atol=1e-9)
    assert np.allclose(T[:3, 3], t, atol=1e-9)


def test_homography_fit_and_plane_deproject():
    from heron.robot.workspace import fit_homography
    from heron.types import Frame

    rng = np.random.default_rng(1)
    true_h = np.array([[0.001, 0.0002, -0.3], [0.0001, -0.0011, 0.25], [0.0, 0.0, 1.0]])
    px = rng.uniform(0, 640, size=(12, 2))
    proj = (true_h @ np.column_stack([px, np.ones(12)]).T).T
    xy = proj[:, :2] / proj[:, 2:3]
    h, rms = fit_homography(px, xy)
    assert rms < 1e-9
    frame = Frame(camera="cam_high", rgb=np.zeros((480, 640, 3), np.uint8),
                  h_pixel_world=h, plane_z=0.012)
    p = frame.deproject(int(px[0][0]), int(px[0][1]))
    assert p is not None and abs(p[2] - 0.012) < 1e-9
    expect = (true_h @ np.array([int(px[0][0]), int(px[0][1]), 1.0]))
    expect = expect[:2] / expect[2]
    assert np.allclose(p[:2], expect, atol=1e-6)


def test_projection_fit_and_triangulation():
    """DLT projection calibration + two-view triangulation, the depth-free 3D path."""
    from heron.robot.workspace import fit_projection
    from heron.types import Frame, reprojection_error, triangulate

    m = MockRobot()
    standard_scene(m)
    frames = {c: m.capture(c) for c in ("cam_high", "cam_low")}
    # Simulate calibration: known fingertip poses across three heights, seen in both views.
    grid = [np.array([x, y, z]) for z in (0.04, 0.12, 0.20)
            for x in (-0.12, 0.0, 0.15) for y in (-0.2, 0.0, 0.2)]
    projections = {}
    for cam, frame in frames.items():
        pts, pxs = [], []
        for p in grid:
            q = project(frame, p)
            if q is not None:
                pts.append(p)
                pxs.append(q)
        P, rms = fit_projection(np.array(pts), np.array(pxs))
        assert rms < 1.0, f"{cam} reprojection rms {rms}"
        projections[cam] = P

    # A point 8 cm above the table (off the homography plane) triangulates correctly.
    target = np.array([0.02, -0.05, 0.08])
    views = []
    for cam, frame in frames.items():
        q = project(frame, target)
        assert q is not None
        views.append((projections[cam], (float(q[0]), float(q[1]))))
    est = triangulate(views)
    assert est is not None
    assert np.linalg.norm(est - target) < 0.005
    assert reprojection_error(views[0][0], est, views[0][1]) < 2.0

    # Coplanar calibration points must be rejected, not silently fit.
    flat = [p for p in grid if abs(p[2] - 0.12) < 1e-9]
    frame = frames["cam_high"]
    with pytest.raises(ValueError):
        fit_projection(np.array(flat), np.array([project(frame, p) for p in flat]))


def test_contact_descend_finds_surface():
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.robot.safety import SafeRobot
    from heron.skills import SkillContext
    from heron.skills.primitives import descend_to_contact
    import tempfile

    m = MockRobot()
    standard_scene(m)
    cfg = HeronConfig()
    tmp = tempfile.mkdtemp()
    cfg.episodes_dir = tmp
    robot = SafeRobot(m, cfg.safety)
    ctx = SkillContext(robot=robot, belief=None, grounder=None, cfg=cfg, log=EpisodeLogger(tmp, "t"))
    block = m.gt_xyz("red_block")
    start_z = cfg.table_z + 0.22  # the safe transit height pick() uses
    z, touched = descend_to_contact(ctx, "left", float(block[0]), float(block[1]), start_z,
                                    cfg.safety.workspace_z[0])
    assert touched
    assert abs(z - m.objects["red_block"].top_z) < 0.012  # stops at the block top, not the table
    # Empty spot: nothing to feel there, so the descent bottoms out without contact.
    z2, touched2 = descend_to_contact(ctx, "right", 0.20, -0.30, start_z, cfg.safety.workspace_z[0])
    assert not touched2 or z2 < 0.02


def test_safety_rejects_out_of_workspace():
    from heron.config import SafetyCfg
    from heron.robot.safety import SafeRobot, SafetyViolation

    m = MockRobot()
    standard_scene(m)
    guard = SafeRobot(m, SafetyCfg())
    try:
        guard.move_cartesian("left", np.array([2.0, 0.0, 0.1]))
        raise AssertionError("expected SafetyViolation")
    except SafetyViolation:
        pass
