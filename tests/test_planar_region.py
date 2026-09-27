"""Parallax-corrected planar grounding: the fix for the oblique overhead camera.

The scenario is the rig's, in miniature: a camera 1 m up and well to one side,
a homography exact on the table plane, and depth whose ABSOLUTE values carry a
bias (the D405 past its working range) but whose relative structure is right.
Plain planar grounding puts a block's centre where its top-face pixel meets the
table — displaced by height x tan(incidence). The corrected path must recover
the true footprint centre to a few millimetres despite the depth bias.
"""
from __future__ import annotations

import numpy as np

from heron.types import Frame

W, H = 640, 480
K = np.array([[500.0, 0, W / 2], [0, 500.0, H / 2], [0, 0, 1.0]])
DEPTH_BIAS_M = 0.018  # the measured median error of the rig's overhead D405


def _rig() -> tuple[np.ndarray, np.ndarray]:
    """Camera->world pose looking at the origin from (0.45, 0.35, 1.0)."""
    c = np.array([0.45, 0.35, 1.0])
    look = np.array([0.5, 0.0, 0.0])
    z = look - c
    z /= np.linalg.norm(z)
    x = np.cross(z, np.array([0.0, 0.0, 1.0]))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    t = np.eye(4)
    t[:3, :3] = np.column_stack([x, y, z])
    t[:3, 3] = c
    return t, c


def _project(t_base_cam: np.ndarray, p_world: np.ndarray) -> tuple[float, float, float]:
    p_cam = np.linalg.inv(t_base_cam) @ np.array([*p_world, 1.0])
    u = K[0, 0] * p_cam[0] / p_cam[2] + K[0, 2]
    v = K[1, 1] * p_cam[1] / p_cam[2] + K[1, 2]
    return u, v, p_cam[2]


def _homography(t_base_cam: np.ndarray) -> np.ndarray:
    """Pixel -> plane-XY homography fitted from projected plane points."""
    pts_w = [np.array([x, y, 0.0]) for x in (0.2, 0.4, 0.6, 0.8)
             for y in (-0.3, 0.0, 0.3)]
    a, b = [], []
    for p in pts_w:
        u, v, _ = _project(t_base_cam, p)
        a.append([u, v, 1, 0, 0, 0, -p[0] * u, -p[0] * v])
        a.append([0, 0, 0, u, v, 1, -p[1] * u, -p[1] * v])
        b += [p[0], p[1]]
    h8 = np.linalg.lstsq(np.array(a), np.array(b), rcond=None)[0]
    return np.array([[*h8[:3]], [*h8[3:6]], [h8[6], h8[7], 1.0]])


def _scene(block_xy: np.ndarray, block_h: float = 0.025, half: float = 0.0125):
    """Depth image of a table plane plus one block, with a uniform bias."""
    t_base_cam, _ = _rig()
    t_cam_base = np.linalg.inv(t_base_cam)
    depth = np.full((H, W), np.nan, dtype=np.float32)
    us, vs = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
    rays = np.stack([(us - K[0, 2]) / K[0, 0], (vs - K[1, 2]) / K[1, 1],
                     np.ones_like(us)])
    r_w = np.einsum("ij,jhw->ihw", t_base_cam[:3, :3], rays)
    c = t_base_cam[:3, 3]
    # intersect every pixel ray with the table plane z=0, then with the block top
    with np.errstate(divide="ignore", invalid="ignore"):
        s_table = -c[2] / r_w[2]
        x_t = c[0] + s_table * r_w[0]
        y_t = c[1] + s_table * r_w[1]
        depth = (s_table * rays[2]).astype(np.float32)  # cam-z depth of the plane
        s_top = (block_h - c[2]) / r_w[2]
        x_b = c[0] + s_top * r_w[0]
        y_b = c[1] + s_top * r_w[1]
        on_top = (np.abs(x_b - block_xy[0]) < half) & (np.abs(y_b - block_xy[1]) < half)
        depth[on_top] = (s_top * rays[2])[on_top].astype(np.float32)
    depth += DEPTH_BIAS_M  # the whole sensor reads far, uniformly
    box = np.argwhere(on_top)
    v0, u0 = box.min(axis=0)
    v1, u1 = box.max(axis=0)
    frame = Frame(camera="cam_high", rgb=np.zeros((H, W, 3), np.uint8),
                  depth=depth, intrinsics=K, t_base_cam=t_base_cam,
                  h_pixel_world=_homography(t_base_cam), plane_z=0.0)
    return frame, (u0, v0, u1, v1)


def test_plain_planar_shows_the_parallax_error():
    """Without the correction the displacement is real and worth fixing."""
    frame, (u0, v0, u1, v1) = _scene(np.array([0.55, -0.25]))
    p = frame.h_pixel_world @ np.array([(u0 + u1) / 2, (v0 + v1) / 2, 1.0])
    err = np.linalg.norm(p[:2] / p[2] - [0.55, -0.25])
    assert err > 0.008, f"scene is not oblique enough to test anything ({err:.4f})"


def test_corrected_centre_lands_on_the_footprint():
    for xy in ([0.55, -0.25], [0.35, 0.25], [0.65, 0.0]):
        frame, (u0, v0, u1, v1) = _scene(np.array(xy))
        got = frame.deproject_region(u0, v0, u1, v1)
        err = np.linalg.norm(got[:2] - xy)
        assert err < 0.004, f"block at {xy}: centre off by {1000 * err:.1f} mm"
        assert 0.005 < got[2] < 0.022, f"centre z {got[2]:.3f} not inside the block"


def test_no_depth_still_answers_on_the_plane():
    frame, (u0, v0, u1, v1) = _scene(np.array([0.55, -0.25]))
    frame.depth = None
    got = frame.deproject_region(u0, v0, u1, v1)
    assert got is not None and abs(got[2] - 0.0) < 1e-9


def test_single_pixel_top_is_corrected_too():
    """Every grounding path funnels through deproject(); it must not hand the
    oblique camera's absolute depth back as an answer."""
    frame, (u0, v0, u1, v1) = _scene(np.array([0.55, -0.25]))
    got = frame.deproject((u0 + u1) // 2, (v0 + v1) // 2)
    err = np.linalg.norm(got[:2] - [0.55, -0.25])
    assert err < 0.005, f"top pixel off by {1000 * err:.1f} mm"
    assert 0.015 < got[2] < 0.035, f"top height {got[2]:.3f} not the block's"


def test_single_pixel_on_the_table_stays_planar():
    frame, (u0, v0, u1, v1) = _scene(np.array([0.55, -0.25]))
    u, v, _ = _project(_rig()[0], np.array([0.4, 0.1, 0.0]))
    got = frame.deproject(int(u), int(v))
    err = np.linalg.norm(got[:2] - [0.4, 0.1])
    assert err < 0.004, f"table pixel off by {1000 * err:.1f} mm"
    assert abs(got[2]) < 0.012, f"table pixel floated to z={got[2]:.3f}"
