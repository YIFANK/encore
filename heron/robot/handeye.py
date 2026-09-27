"""Hand-eye solver AX = XB (Park & Martin, 1994).

OpenCV 5.0 dropped `cv2.calibrateHandEye` from the Python bindings (the
CALIB_HAND_EYE_* constants survive, the function does not), so we solve it
here. Park's method is closed-form: rotations first via a Procrustes fit on
the axis-angle vectors of the relative motions, then translation by linear
least squares — no initial guess, no iteration.

Given per-view gripper poses T_base_gripper and target poses T_cam_target,
the relative motions between view i and j satisfy A_ij X = X B_ij with
    A_ij = T_base_gripper(i)^-1 @ T_base_gripper(j)     (gripper motion)
    B_ij = T_cam_target(i) @ T_cam_target(j)^-1         (camera motion)
and X = T_gripper_cam, the fixed mount we want.

Motions must be rotationally diverse: pure translations leave the rotation
axes collinear and M becomes rank-deficient (the classic degeneracy).
"""
from __future__ import annotations

import numpy as np


def _log_rot(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> axis-angle vector (magnitude = angle)."""
    cos = (np.trace(R) - 1.0) / 2.0
    theta = float(np.arccos(np.clip(cos, -1.0, 1.0)))
    if theta < 1e-9:
        return np.zeros(3)
    if abs(theta - np.pi) < 1e-6:  # near-pi: log is unstable, use the symmetric part
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(A), 0.0, None))
        i = int(np.argmax(axis))
        if axis[i] > 1e-9:
            axis = A[:, i] / axis[i]
        return axis / (np.linalg.norm(axis) + 1e-12) * theta
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w * theta / (2.0 * np.sin(theta))


def _inv(T: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def calibrate_hand_eye(T_base_gripper: list[np.ndarray],
                       T_cam_target: list[np.ndarray]) -> np.ndarray:
    """Solve for X = T_gripper_cam (4x4). Needs >= 3 rotationally diverse views."""
    n = len(T_base_gripper)
    if n < 3 or len(T_cam_target) != n:
        raise ValueError(f"need >= 3 matched views, got {n}/{len(T_cam_target)}")

    A_list, B_list = [], []
    for i in range(n):
        for j in range(i + 1, n):
            A = _inv(T_base_gripper[i]) @ T_base_gripper[j]
            B = T_cam_target[i] @ _inv(T_cam_target[j])
            # Motions with no rotation carry no information about R_X.
            if np.linalg.norm(_log_rot(A[:3, :3])) < 0.05:
                continue
            A_list.append(A)
            B_list.append(B)
    if len(A_list) < 2:
        raise ValueError("motions are not rotationally diverse enough "
                         "(vary the wrist ORIENTATION between views, not just position)")

    M = np.zeros((3, 3))
    for A, B in zip(A_list, B_list):
        M += np.outer(_log_rot(B[:3, :3]), _log_rot(A[:3, :3]))
    # R_X = (M^T M)^(-1/2) M^T, via eigendecomposition of the SPD matrix M^T M.
    w, V = np.linalg.eigh(M.T @ M)
    w = np.clip(w, 1e-12, None)
    R_X = V @ np.diag(1.0 / np.sqrt(w)) @ V.T @ M.T
    U, _, Vt = np.linalg.svd(R_X)  # re-project onto SO(3)
    R_X = U @ Vt
    if np.linalg.det(R_X) < 0:
        U[:, -1] *= -1
        R_X = U @ Vt

    # (R_A - I) t_X = R_X t_B - t_A, stacked over all motion pairs.
    lhs, rhs = [], []
    for A, B in zip(A_list, B_list):
        lhs.append(A[:3, :3] - np.eye(3))
        rhs.append(R_X @ B[:3, 3] - A[:3, 3])
    t_X, *_ = np.linalg.lstsq(np.vstack(lhs), np.concatenate(rhs), rcond=None)

    X = np.eye(4)
    X[:3, :3] = R_X
    X[:3, 3] = t_X
    return X
