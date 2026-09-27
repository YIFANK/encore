"""World-frame bookkeeping: camera extrinsics and marker-free calibration.

Calibration idea (no ArUco boards): the robot's own fingertip is the target.
`collect_calibration_pairs` drives an arm through a grid of world-frame poses;
at each pose the orchestrator points at the gripper fingers in the camera
image, and the pixel+depth deprojection in *camera* coordinates is paired with
the known world-frame tool position. `solve_extrinsics` then fits the rigid
camera->world transform (Kabsch/Umeyama without scale).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np


def solve_extrinsics(p_cam: np.ndarray, p_world: np.ndarray) -> tuple[np.ndarray, float]:
    """Least-squares rigid transform T (4x4) with p_world ≈ T @ p_cam.

    Returns (T, rms_error_m). Needs >= 3 non-collinear pairs; 8+ recommended.
    """
    p_cam = np.asarray(p_cam, dtype=float)
    p_world = np.asarray(p_world, dtype=float)
    if p_cam.shape != p_world.shape or p_cam.shape[0] < 3:
        raise ValueError("need matching Nx3 arrays with N >= 3")
    mu_c, mu_w = p_cam.mean(axis=0), p_world.mean(axis=0)
    qc, qw = p_cam - mu_c, p_world - mu_w
    h = qc.T @ qw
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rot = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    t = mu_w - rot @ mu_c
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = t
    residual = (rot @ p_cam.T).T + t - p_world
    rms = float(np.sqrt((residual**2).sum(axis=1).mean()))
    return T, rms


def save_extrinsics(path: str | Path, t_world_cam: np.ndarray, rms: float) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, t_world_cam=t_world_cam, rms=np.array([rms]))


def load_extrinsics(path: str | Path) -> np.ndarray:
    data = np.load(Path(path))
    return data["t_world_cam"]


def pose_from_plane_homography(h_pixel_world: np.ndarray, intrinsics: np.ndarray,
                                plane_z: float = 0.0) -> np.ndarray:
    """Camera pose (camera -> world, 4x4) implied by a table-plane homography.

    A pixel->plane homography and the camera matrix K together determine the
    camera's pose up to the usual planar decomposition: inv(K) @ inv(H) is
    [r1 r2 t] up to scale. This turns the board-stitched homography — the most
    accurate calibration this rig has (1.1 mm rms) — into extrinsics that are
    EXACTLY consistent with it on the table plane, so the depth path and the
    planar path stop disagreeing by the camera's obliquity times the object's
    height. Measured before this existed: a 25 mm block top deprojected through
    the plane landed up to 16 mm away from where the block stood.
    """
    Hinv = np.linalg.inv(np.asarray(h_pixel_world, dtype=float))
    M = np.linalg.inv(np.asarray(intrinsics, dtype=float)) @ Hinv
    M /= np.sqrt(np.linalg.norm(M[:, 0]) * np.linalg.norm(M[:, 1]))
    if M[2, 2] < 0:          # the camera looks at the plane from above
        M = -M
    r1, r2, t = M[:, 0], M[:, 1], M[:, 2]
    R = np.column_stack([r1, r2, np.cross(r1, r2)])
    u, _, vt = np.linalg.svd(R)          # nearest true rotation
    R = u @ vt
    T = np.eye(4)
    T[:3, :3] = R.T
    T[:3, 3] = -R.T @ t + np.array([0.0, 0.0, float(plane_z)])
    return T


def fit_homography(px: np.ndarray, xy: np.ndarray) -> tuple[np.ndarray, float]:
    """DLT fit of a 3x3 homography H with [x, y, 1]^T ~ H @ [u, v, 1]^T.

    Returns (H, rms_error_m in world XY). Needs >= 4 non-collinear pairs.
    Matches the lab's proven depth-free approach: everything sits on one plane,
    so a homography from overhead pixels to table XY is exact.
    """
    px = np.asarray(px, dtype=float)
    xy = np.asarray(xy, dtype=float)
    if px.shape[0] != xy.shape[0] or px.shape[0] < 4:
        raise ValueError("need matching Nx2 arrays with N >= 4")
    rows = []
    for (u, v), (x, y) in zip(px, xy):
        rows.append([u, v, 1, 0, 0, 0, -x * u, -x * v, -x])
        rows.append([0, 0, 0, u, v, 1, -y * u, -y * v, -y])
    _, _, vt = np.linalg.svd(np.asarray(rows))
    h = vt[-1].reshape(3, 3)
    proj = (h @ np.column_stack([px, np.ones(len(px))]).T).T
    proj_xy = proj[:, :2] / proj[:, 2:3]
    rms = float(np.sqrt(((proj_xy - xy) ** 2).sum(axis=1).mean()))
    return h, rms


def save_homography(path: str | Path, h: np.ndarray, plane_z: float, rms: float) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, h_pixel_world=h, plane_z=np.array([plane_z]), rms=np.array([rms]))


def load_homography(path: str | Path) -> tuple[np.ndarray, float]:
    data = np.load(Path(path))
    return data["h_pixel_world"], float(data["plane_z"][0])


def fit_projection(xyz: np.ndarray, px: np.ndarray) -> tuple[np.ndarray, float]:
    """DLT fit of a 3x4 projection matrix P with [u, v, 1]^T ~ P @ [x, y, z, 1]^T.

    Absorbs intrinsics and extrinsics in one matrix, so it needs neither depth
    nor factory intrinsics — the whole point on a rig where librealsense is
    unavailable. Requires >= 6 NON-COPLANAR correspondences (calibration_grid
    with several z values). Returns (P, rms reprojection error in pixels).
    """
    xyz = np.asarray(xyz, dtype=float)
    px = np.asarray(px, dtype=float)
    if xyz.shape[0] != px.shape[0] or xyz.shape[0] < 6:
        raise ValueError("need matching Nx3 / Nx2 arrays with N >= 6")
    if np.linalg.matrix_rank(xyz - xyz.mean(axis=0), tol=1e-6) < 3:
        raise ValueError("calibration points are coplanar — vary the height (z) too")
    # Normalize both spaces for conditioning; undo the transforms afterwards.
    c3, s3 = xyz.mean(axis=0), np.sqrt(3) / (np.linalg.norm(xyz - xyz.mean(axis=0), axis=1).mean() + 1e-12)
    c2, s2 = px.mean(axis=0), np.sqrt(2) / (np.linalg.norm(px - px.mean(axis=0), axis=1).mean() + 1e-12)
    xn, pn = (xyz - c3) * s3, (px - c2) * s2
    rows = []
    for (x, y, z), (u, v) in zip(xn, pn):
        rows.append([-x, -y, -z, -1, 0, 0, 0, 0, u * x, u * y, u * z, u])
        rows.append([0, 0, 0, 0, -x, -y, -z, -1, v * x, v * y, v * z, v])
    _, _, vt = np.linalg.svd(np.asarray(rows))
    pn_mat = vt[-1].reshape(3, 4)
    t3 = np.eye(4) * s3
    t3[:3, 3] = -c3 * s3
    t3[3, 3] = 1.0
    t2 = np.array([[s2, 0, -c2[0] * s2], [0, s2, -c2[1] * s2], [0, 0, 1.0]])
    proj = np.linalg.inv(t2) @ pn_mat @ t3
    proj = proj / (np.linalg.norm(proj[2, :3]) + 1e-12)
    homo = np.column_stack([xyz, np.ones(len(xyz))])
    q = (proj @ homo.T).T
    rms = float(np.sqrt(((q[:, :2] / q[:, 2:3] - px) ** 2).sum(axis=1).mean()))
    return proj, rms


def save_projection(path: str | Path, proj: np.ndarray, rms: float) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, proj=proj, rms=np.array([rms]))


def load_projection(path: str | Path) -> np.ndarray:
    return np.load(Path(path))["proj"]


def calibration_grid(z: float = 0.12, nx: int = 3, ny: int = 3,
                     x_range: tuple[float, float] = (-0.12, 0.15),
                     y_range: tuple[float, float] = (-0.2, 0.2),
                     z_levels: tuple[float, ...] | None = None) -> list[np.ndarray]:
    """World-frame fingertip poses the arm visits during calibration.

    Pass z_levels for projection-matrix calibration — a single-height (coplanar)
    grid cannot determine a 3x4 projection.
    """
    xs = np.linspace(*x_range, nx)
    ys = np.linspace(*y_range, ny)
    zs = z_levels if z_levels is not None else (z,)
    return [np.array([x, y, zz]) for zz in zs for x in xs for y in ys]


def grid_ranges_from_safety(cfg, margin: float = 0.04) -> tuple[tuple[float, float],
                                                                tuple[float, float]]:
    """Grid bounds inset from the configured safety envelope.

    `calibrate` drives the arm directly, without the skill layer's safety check,
    so the grid itself must stay inside the envelope — the packaged defaults
    assume a frame where the workspace straddles the base, which is not true of
    every rig.
    """
    sx, sy = cfg.safety.workspace_x, cfg.safety.workspace_y
    return ((sx[0] + margin, sx[1] - margin), (sy[0] + margin, sy[1] - margin))


def check_grid_reachable(poses: list[np.ndarray], cfg) -> list[str]:
    """Names of safety limits the grid would violate (empty = grid is safe)."""
    sx, sy, sz = cfg.safety.workspace_x, cfg.safety.workspace_y, cfg.safety.workspace_z
    bad = []
    for p in poses:
        if not sx[0] <= p[0] <= sx[1]:
            bad.append(f"x={p[0]:.3f} outside {sx}")
        if not sy[0] <= p[1] <= sy[1]:
            bad.append(f"y={p[1]:.3f} outside {sy}")
        if not sz[0] <= p[2] <= sz[1]:
            bad.append(f"z={p[2]:.3f} outside {sz}")
    return sorted(set(bad))
