"""The hand-eye solver must recover a known mount from synthetic motions."""
from __future__ import annotations

import numpy as np
import pytest

from heron.robot.handeye import calibrate_hand_eye


def _rot(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def _T(R, t):
    out = np.eye(4)
    out[:3, :3] = R
    out[:3, 3] = t
    return out


def _synthetic(n=12, noise=0.0, seed=0):
    """Ground-truth mount X; board fixed in base frame; n diverse gripper poses."""
    rng = np.random.default_rng(seed)
    X = _T(_rot([0.3, -0.8, 0.5], 0.7), [0.04, -0.02, 0.06])   # T_gripper_cam
    T_base_target = _T(_rot([0, 0, 1], 0.4), [0.45, 0.05, 0.0])
    Tbg, Tct = [], []
    for _ in range(n):
        R = _rot(rng.normal(size=3), rng.uniform(0.3, 1.2))
        t = np.array([0.35, 0.0, 0.30]) + rng.normal(scale=0.06, size=3)
        T_b_g = _T(R, t)
        T_c_t = np.linalg.inv(T_b_g @ X) @ T_base_target
        if noise:
            T_c_t[:3, 3] += rng.normal(scale=noise, size=3)
        Tbg.append(T_b_g)
        Tct.append(T_c_t)
    return X, Tbg, Tct


def test_recovers_exact_mount():
    X, Tbg, Tct = _synthetic()
    est = calibrate_hand_eye(Tbg, Tct)
    assert np.allclose(est, X, atol=1e-6)


def test_tolerates_measurement_noise():
    X, Tbg, Tct = _synthetic(n=15, noise=0.001, seed=3)
    est = calibrate_hand_eye(Tbg, Tct)
    assert np.linalg.norm(est[:3, 3] - X[:3, 3]) < 0.01       # < 1 cm
    ang = np.arccos(np.clip((np.trace(est[:3, :3].T @ X[:3, :3]) - 1) / 2, -1, 1))
    assert np.degrees(ang) < 3.0


def test_rejects_pure_translation():
    """Rotationally degenerate input must fail loudly, not return a wrong answer."""
    X = _T(np.eye(3), [0.04, -0.02, 0.06])
    T_base_target = _T(np.eye(3), [0.45, 0.05, 0.0])
    Tbg, Tct = [], []
    for i in range(6):
        T_b_g = _T(np.eye(3), [0.3 + 0.02 * i, 0.01 * i, 0.3])
        Tbg.append(T_b_g)
        Tct.append(np.linalg.inv(T_b_g @ X) @ T_base_target)
    with pytest.raises(ValueError, match="rotationally diverse"):
        calibrate_hand_eye(Tbg, Tct)
