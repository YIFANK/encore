"""Forward and inverse kinematics for the WidowX AI, solved here rather than by
the arm controller.

Why not use the controller's own IK: it is a closed-form solver and it refuses
poses this arm can plainly hold. Measured 2026-07-31 — the controller rejected
[0.347, 0.175, 0.205] at every approach angle from vertical to 60 degrees, while
a numerical solver with 64 seeds found collision-free solutions for all of them.
A 6-DOF analytic solution generally assumes a spherical wrist; where the real
geometry does not oblige, the closed form covers only part of the reachable set
and reports the rest as unreachable.

So Heron solves IK itself and commands JOINT positions. The reachable set then
comes from the joint limits, not from what a closed form happens to cover.

The chain is read from the URDF (assets/wxai_follower.urdf, from
TrossenRobotics/trossen_arm_description), so the model and the hardware cannot
drift apart through a transcription error here.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

DEFAULT_URDF = Path(__file__).resolve().parents[2] / "assets" / "wxai_follower.urdf"
ARM_JOINTS = ("joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5")
# The frame the controller reports and accepts as its cartesian pose. `grasp_frame`
# sits at the same point but rotated; using it would silently reinterpret every
# orientation we send.
TOOL_LINK = "ee_gripper_link"


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rodrigues(rvec) -> np.ndarray:
    """Angle-axis -> rotation matrix. The driver's orientation encoding."""
    v = np.asarray(rvec, dtype=float)
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * kx + (1 - np.cos(th)) * (kx @ kx)


def rotvec(r: np.ndarray) -> np.ndarray:
    """Rotation matrix -> angle-axis."""
    th = float(np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)))
    if th < 1e-9:
        return np.zeros(3)
    if abs(np.pi - th) < 1e-6:
        a = (r + np.eye(3)) / 2.0
        d = np.sqrt(np.maximum(np.diag(a), 0.0))
        k = int(np.argmax(d))
        axis = a[:, k] / d[k] if d[k] > 1e-9 else np.eye(3)[k]
        return axis / max(np.linalg.norm(axis), 1e-12) * th
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return w / (2.0 * np.sin(th)) * th


@dataclass
class _Joint:
    name: str
    axis: np.ndarray
    t_origin: np.ndarray
    lo: float
    hi: float


class Kinematics:
    """FK, a Jacobian, and a damped-least-squares IK for one WidowX AI arm."""

    def __init__(self, urdf: str | Path = DEFAULT_URDF, tool_link: str = TOOL_LINK) -> None:
        root = ET.parse(str(urdf)).getroot()
        by_child = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else "0 0 0".split())]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None else "0 0 0".split())]
            t = np.eye(4)
            t[:3, :3] = _rpy(*rpy)
            t[:3, 3] = xyz
            a = j.find("axis")
            lim = j.find("limit")
            by_child[j.find("child").get("link")] = (
                j.get("name"), j.get("type"), j.find("parent").get("link"), t,
                np.array([float(v) for v in a.get("xyz").split()]) if a is not None else np.array([0, 0, 1.0]),
                float(lim.get("lower")) if lim is not None and lim.get("lower") else 0.0,
                float(lim.get("upper")) if lim is not None and lim.get("upper") else 0.0)

        # Walk up from the tool to the base, so the chain is whatever the URDF
        # actually connects rather than a hand-listed order that can rot.
        chain, link = [], tool_link
        while link in by_child:
            chain.append(by_child[link])
            link = by_child[link][2]
        chain.reverse()
        self.base_link = link

        self.joints: list[_Joint] = []
        self._fixed: list[np.ndarray] = []
        self._pre: list[np.ndarray] = []
        acc = np.eye(4)
        for name, jtype, _parent, t, axis, lo, hi in chain:
            if jtype == "fixed" or name not in ARM_JOINTS:
                acc = acc @ t
                continue
            self._pre.append(acc @ t)
            self.joints.append(_Joint(name, axis / np.linalg.norm(axis), t, lo, hi))
            acc = np.eye(4)
        self._tool = acc  # fixed transform from the last joint to the tool frame
        # Same transform, exposed. Anything that has to place something in the
        # TOOL frame onto the last LINK needs it — MuJoCo attaches cameras to
        # bodies, and a hand-eye transform is measured from the tool.
        self.tool_offset = acc.copy()
        self.lo = np.array([j.lo for j in self.joints])
        self.hi = np.array([j.hi for j in self.joints])
        if len(self.joints) != len(ARM_JOINTS):
            raise ValueError(f"expected {len(ARM_JOINTS)} joints to {tool_link}, "
                             f"found {[j.name for j in self.joints]}")

    @property
    def dof(self) -> int:
        return len(self.joints)

    def fk(self, q) -> np.ndarray:
        """Joint angles -> 4x4 pose of the tool frame in the base frame."""
        q = np.asarray(q, dtype=float)
        t = np.eye(4)
        for i, j in enumerate(self.joints):
            r = np.eye(4)
            r[:3, :3] = rodrigues(j.axis * q[i])
            t = t @ self._pre[i] @ r
        return t @ self._tool

    def link_points(self, q) -> np.ndarray:
        """Every joint origin plus the tool point at `q`, (dof+1, 3), base frame.

        The chain's own skeleton — what the arm physically occupies, as opposed
        to fk()'s single tool point. Consumers that need to know whether a
        detection landed ON the robot test against this polyline.
        """
        q = np.asarray(q, dtype=float)
        t = np.eye(4)
        pts = []
        for i, j in enumerate(self.joints):
            t = t @ self._pre[i]
            pts.append(t[:3, 3].copy())
            r = np.eye(4)
            r[:3, :3] = rodrigues(j.axis * q[i])
            t = t @ r
        pts.append((t @ self._tool)[:3, 3])
        return np.asarray(pts)

    def jacobian(self, q) -> np.ndarray:
        """6xN geometric Jacobian at `q`, rows [vx vy vz wx wy wz]."""
        q = np.asarray(q, dtype=float)
        t = np.eye(4)
        origins, axes = [], []
        for i, j in enumerate(self.joints):
            t = t @ self._pre[i]
            origins.append(t[:3, 3].copy())
            axes.append(t[:3, :3] @ j.axis)
            r = np.eye(4)
            r[:3, :3] = rodrigues(j.axis * q[i])
            t = t @ r
        p_end = (t @ self._tool)[:3, 3]
        jac = np.zeros((6, self.dof))
        for i in range(self.dof):
            jac[:3, i] = np.cross(axes[i], p_end - origins[i])
            jac[3:, i] = axes[i]
        return jac

    def ik(self, xyz, rvec, q_seed=None, *, pos_tol=1e-4, rot_tol=2e-3,
           iters=200, seeds=24, rng=None):
        """Joint angles reaching (xyz, rvec), or None.

        Damped least squares, restarted from several seeds because a single
        descent falls into a local minimum near the workspace edge — exactly
        where the closed-form solver already fails, so a solver that gives up
        there would add nothing.

        Solutions are chosen for closeness to `q_seed` (the arm's present pose)
        so the arm takes the smallest move that reaches the goal, rather than
        flipping through a different elbow configuration on the way.
        """
        target_p = np.asarray(xyz, dtype=float)
        target_r = rodrigues(rvec)
        rng = rng or np.random.default_rng(0)
        mid = (self.lo + self.hi) / 2.0
        start = np.clip(np.asarray(q_seed, float), self.lo, self.hi) if q_seed is not None else mid

        best, best_cost = None, np.inf
        for s in range(seeds):
            q = start.copy() if s == 0 else rng.uniform(self.lo, self.hi)
            for _ in range(iters):
                t = self.fk(q)
                e_p = target_p - t[:3, 3]
                e_r = rotvec(target_r @ t[:3, :3].T)
                if np.linalg.norm(e_p) < pos_tol and np.linalg.norm(e_r) < rot_tol:
                    cost = float(np.linalg.norm(q - start))
                    if cost < best_cost:
                        best, best_cost = q.copy(), cost
                    break
                jac = self.jacobian(q)
                err = np.concatenate([e_p, e_r])
                lam = 0.05 + 0.5 * float(np.linalg.norm(err))   # damp harder when far
                dq = jac.T @ np.linalg.solve(jac @ jac.T + lam ** 2 * np.eye(6), err)
                q = np.clip(q + np.clip(dq, -0.25, 0.25), self.lo, self.hi)
        return best

    def reachable(self, xyz, rvec, **kw) -> bool:
        return self.ik(xyz, rvec, **kw) is not None
