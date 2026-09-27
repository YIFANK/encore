"""A single pixel reports the surface facing the camera, not the object's centre.

Measured on LIBERO, that bias put a can on the rim of a basket instead of inside
it: 79 mm of error on an 80 mm half-width container. These tests pin the fix.
"""
from __future__ import annotations

import numpy as np

from heron.types import Frame

# Camera 1 m above the origin looking straight down, and tilted scenes below.
K = np.array([[400.0, 0, 160.0], [0, 400.0, 120.0], [0, 0, 1.0]])


def _look_at(eye, target=(0.0, 0.0, 0.0)) -> np.ndarray:
    """Camera-to-world for an OpenCV camera (x right, y down, z forward)."""
    eye = np.asarray(eye, float)
    f = np.asarray(target, float) - eye
    f /= np.linalg.norm(f)
    up = np.array([0.0, 0.0, 1.0])
    if abs(f @ up) > 0.999:
        up = np.array([0.0, 1.0, 0.0])
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    d = np.cross(f, r)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([r, d, f])
    T[:3, 3] = eye
    return T


def _cube_scene(half: float, eye=(0.0, -0.6, 0.85), res=(240, 320)) -> tuple[Frame, np.ndarray]:
    """Render depth of a cube sitting at the origin; return the frame and the cube centre."""
    h, w = res
    t_base_cam = _look_at(eye)
    t_cam_base = np.linalg.inv(t_base_cam)
    centre = np.array([0.0, 0.0, half])       # cube rests on z=0
    depth = np.full((h, w), np.nan, dtype=np.float32)

    # Ray-cast: for every pixel, intersect with the cube's six faces and keep the
    # nearest hit. Simple and exact enough for a unit test.
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    lo = centre - half
    hi = centre + half
    origin = t_base_cam[:3, 3]
    for v in range(h):
        for u in range(w):
            d_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
            d_world = t_base_cam[:3, :3] @ d_cam
            tmin, tmax = 0.0, 1e9
            for i in range(3):
                if abs(d_world[i]) < 1e-9:
                    if origin[i] < lo[i] or origin[i] > hi[i]:
                        tmin = 1e9
                        break
                    continue
                t1 = (lo[i] - origin[i]) / d_world[i]
                t2 = (hi[i] - origin[i]) / d_world[i]
                tmin = max(tmin, min(t1, t2))
                tmax = min(tmax, max(t1, t2))
            if tmin < tmax < 1e9:
                p_world = origin + tmin * d_world
                depth[v, u] = float((t_cam_base @ np.append(p_world, 1.0))[2])
    return Frame(camera="c", rgb=np.zeros((h, w, 3), np.uint8), depth=depth,
                 intrinsics=K, t_base_cam=t_base_cam), centre


def _box_of_visible(frame: Frame) -> tuple[int, int, int, int]:
    vs, us = np.where(np.isfinite(frame.depth))
    return int(us.min()), int(vs.min()), int(us.max()), int(vs.max())


def test_region_centroid_beats_a_single_pixel_on_a_tilted_view():
    """The whole point: averaging the visible patch removes the camera-ward bias."""
    frame, centre = _cube_scene(half=0.08)
    u0, v0, u1, v1 = _box_of_visible(frame)

    single = frame.deproject((u0 + u1) // 2, (v0 + v1) // 2)
    region = frame.deproject_region(u0, v0, u1, v1)
    assert single is not None and region is not None

    err_single = float(np.linalg.norm(single - centre))
    err_region = float(np.linalg.norm(region - centre))
    # The single-pixel estimate is off by something like the object's half-size.
    assert err_single > 0.04, err_single
    # A solid cube only ever shows the faces turned toward the camera, so the
    # centroid of the visible shell still sits in front of the true centre — the
    # residual here is geometry, not a bug. (Open containers, the case that
    # actually broke placement, expose their interior and land within ~7 mm.)
    assert err_region < 0.9 * err_single, (err_region, err_single)


def test_knowing_the_support_plane_recovers_the_centre():
    """Looking down you only ever measure the top surface; the support gives the rest."""
    frame, centre = _cube_scene(half=0.08)
    u0, v0, u1, v1 = _box_of_visible(frame)
    blind = frame.deproject_region(u0, v0, u1, v1)
    informed = frame.deproject_region(u0, v0, u1, v1, support_z=0.0)
    assert abs(informed[2] - centre[2]) < 0.01, informed
    assert np.linalg.norm(informed - centre) < 0.2 * np.linalg.norm(blind - centre)


def test_the_support_surface_is_excluded_from_the_object():
    """A box around an object also contains the table it stands on."""
    frame, centre = _cube_scene(half=0.08)
    u0, v0, u1, v1 = _box_of_visible(frame)
    # Widen the box so it takes in a lot of empty table around the cube.
    wide = frame.deproject_region(max(0, u0 - 60), max(0, v0 - 60), u1 + 60, v1 + 60,
                                  support_z=0.0)
    assert wide is not None
    assert np.linalg.norm(wide[:2] - centre[:2]) < 0.05, wide


def test_the_bias_points_toward_the_camera():
    """Diagnosis, not just magnitude: the error is along the viewing ray."""
    frame, centre = _cube_scene(half=0.08)
    u0, v0, u1, v1 = _box_of_visible(frame)
    single = frame.deproject((u0 + u1) // 2, (v0 + v1) // 2)
    to_camera = frame.t_base_cam[:3, 3] - centre
    to_camera /= np.linalg.norm(to_camera)
    err = single - centre
    assert float(err @ to_camera) > 0.6 * float(np.linalg.norm(err))


def test_depth_free_frames_fall_back_to_the_plane():
    """A homography rig has no depth; the box centre already is the footprint."""
    h_pixel_world = np.array([[0.001, 0, -0.16], [0, 0.001, -0.12], [0, 0, 1.0]])
    frame = Frame(camera="c", rgb=np.zeros((20, 20, 3), np.uint8),
                  h_pixel_world=h_pixel_world, plane_z=-0.015)
    p = frame.deproject_region(100, 100, 200, 200)
    assert p is not None
    assert p[2] == -0.015
    assert np.allclose(p[:2], frame.deproject(150, 150)[:2])


def test_a_region_with_no_depth_returns_nothing():
    frame = Frame(camera="c", rgb=np.zeros((20, 20, 3), np.uint8),
                  depth=np.full((20, 20), np.nan, np.float32),
                  intrinsics=K, t_base_cam=np.eye(4))
    assert frame.deproject_region(0, 0, 19, 19) is None


def test_depth_consistency_rejects_a_second_surface_in_the_region():
    """A few pixels from a different object must not move the answer.

    The silhouette rule reads the EXTREMES of the selected points, so unlike a
    centroid it has no averaging to hide behind. Measured on a real scene: a
    region that caught the shelf under a cream-cheese box landed 92 mm away.
    """
    import numpy as np

    from heron.types import Frame

    h = w = 60
    depth = np.full((h, w), 1.0, dtype=np.float32)
    depth[20:40, 20:40] = 0.94                 # the object, 6 cm tall
    depth[20:40, 40:44] = 1.60                 # something far behind it
    k = np.array([[60.0, 0, 30.0], [0, 60.0, 30.0], [0, 0, 1.0]])
    t = np.array([[1.0, 0, 0, 0], [0, -1.0, 0, 0], [0, 0, -1.0, 1.0], [0, 0, 0, 1.0]])
    frame = Frame(camera="c", rgb=np.zeros((h, w, 3), np.uint8), depth=depth,
                  intrinsics=k, t_base_cam=t)

    got = frame.deproject_region(20, 20, 43, 39, support_z=0.0)
    true_x = ((20 - 30) / 60 * 0.94 + (39 - 30) / 60 * 0.94) / 2
    assert abs(got[0] - true_x) < 0.01, "far-surface pixels must be cut, not averaged"


def test_depth_consistency_keeps_a_flat_object_whole():
    """A plate has almost no depth spread, so its MAD is ~0. Cutting at k*MAD
    would keep a single depth plane and throw the object away."""
    import numpy as np

    from heron.types import Frame

    h = w = 60
    depth = np.full((h, w), 1.0, dtype=np.float32)
    rng = np.random.default_rng(0)
    depth[20:40, 20:40] = 0.98 + rng.normal(0, 0.001, (20, 20))  # flat, slightly noisy
    k = np.array([[60.0, 0, 30.0], [0, 60.0, 30.0], [0, 0, 1.0]])
    t = np.array([[1.0, 0, 0, 0], [0, -1.0, 0, 0], [0, 0, -1.0, 1.0], [0, 0, 0, 1.0]])
    frame = Frame(camera="c", rgb=np.zeros((h, w, 3), np.uint8), depth=depth,
                  intrinsics=k, t_base_cam=t)

    got = frame.deproject_region(20, 20, 39, 39, support_z=0.0)
    assert got is not None
    true_x = ((20 - 30) / 60 * 0.98 + (39 - 30) / 60 * 0.98) / 2
    assert abs(got[0] - true_x) < 0.01
