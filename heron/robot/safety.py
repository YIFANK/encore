"""Safety guard: a non-bypassable wrapper between skills and any robot backend.

Every motion is checked against the world-frame workspace box, displacement
limits, and speed floor; a rejected motion surfaces as a SAFETY-class failure
through the normal diagnosis path instead of moving the arm. Also implements
confirm-gate and dry-run modes for first runs on real hardware.
"""
from __future__ import annotations

import sys

import numpy as np

from ..config import SafetyCfg
from ..types import Frame


class SafetyViolation(RuntimeError):
    pass


class EStop(RuntimeError):
    pass


class SafeRobot:
    """Wraps a RobotInterface. Skills only ever see this wrapper."""

    def __init__(self, robot, cfg: SafetyCfg) -> None:
        self._robot = robot
        self.cfg = cfg
        self.arms = robot.arms
        self.cameras = robot.cameras
        self._stopped = False

    # -- guard logic ---------------------------------------------------------
    def _check_target(self, arm: str, xyz: np.ndarray) -> None:
        if self._stopped:
            raise EStop("software e-stop is engaged")
        (x0, x1), (y0, y1), (z0, z1) = self.cfg.workspace_x, self.cfg.workspace_y, self.cfg.workspace_z
        x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        if not (x0 <= x <= x1 and y0 <= y <= y1 and z0 <= z <= z1):
            raise SafetyViolation(
                f"target {np.round(xyz, 3).tolist()} for {arm} arm outside workspace "
                f"x{self.cfg.workspace_x} y{self.cfg.workspace_y} z{self.cfg.workspace_z}"
            )
        # The workspace box is a coarse guard, and it is not the real limit:
        # the reachable set is a curved envelope, and this arm cannot point a
        # gripper downward much above 0.15 m over most of the table while
        # workspace_z claims 0.40. Ask the kinematics too, so an unreachable
        # target is refused HERE with a position in the message, rather than
        # surfacing later as a driver error in the middle of a descent.
        inner = getattr(self._robot, "reachable", None)
        if callable(inner) and not inner(arm, xyz):
            raise SafetyViolation(
                f"target {np.round(xyz, 3).tolist()} is inside the {arm} workspace box "
                f"but the arm cannot hold its approach orientation there — the "
                f"reachable set is an envelope, not a box"
            )
        current = self._robot.get_cartesian(arm)
        step = float(np.linalg.norm(np.asarray(xyz) - current))
        if step > self.cfg.max_step_m:
            raise SafetyViolation(f"displacement {step:.3f} m exceeds max_step_m={self.cfg.max_step_m}")

    def _confirm(self, description: str) -> None:
        if not self.cfg.confirm_actions:
            return
        # A prompt nobody can answer is worse than no prompt: without a tty
        # (launched from a script, a detached Terminal window, a service) input()
        # blocks forever and reads as a crash. Fail fast and say what to change.
        if not sys.stdin.isatty():
            raise SafetyViolation(
                f"confirm_actions is on but stdin is not interactive, so "
                f"{description!r} cannot be confirmed — run from a terminal, or set "
                "safety.confirm_actions: false in the config for scripted motion"
            )
        answer = input(f"[heron confirm] {description} — proceed? [y/N] ").strip().lower()
        if answer not in ("y", "yes"):
            raise SafetyViolation(f"operator declined: {description}")

    # -- RobotInterface passthrough with checks ------------------------------
    def home(self, arm: str) -> None:
        self._confirm(f"home {arm} arm")
        if self.cfg.dry_run:
            return
        self._robot.home(arm)

    def _segment_seconds(self, arm: str, xyz: np.ndarray, requested: float) -> float:
        """How long this segment should take: a SPEED cap, not a TIME floor.

        `min_move_seconds` was doing both jobs and only one of them well. Every
        segment shorter than the floor was stretched to it, so a 50 mm descent
        was commanded over the same 2 s as a 300 mm traverse — and then divided
        by speed_scale, making it 3.3 s of blocking wait for 50 mm of travel.
        Measured over 14 rig episodes: 19 s inside a single `pick`, of which
        almost none was model latency and almost all was waiting out durations
        nobody needed.

        The fix is to say the thing that was actually meant. `max_tool_speed_mps`
        is the cap; the floor is only there so a zero-length move still has a
        duration. The default cap is the speed the LONGEST moves already run at
        today, so nothing on this rig moves faster than it already does — the
        whole saving comes from short segments no longer being charged as if
        they were long ones.

        The caller's `seconds` is then not consulted at all, and that is the
        point: those constants (1.5 here, 2.5 there) are the old conservative
        defaults, not statements about a particular segment. Honouring them
        would keep every short move slow, which is the whole thing being fixed.
        Leave `max_tool_speed_mps` unset and the old behaviour returns exactly.
        """
        cap = float(getattr(self.cfg, "max_tool_speed_mps", 0.0) or 0.0)
        if cap <= 0.0:                      # not configured: the old behaviour
            return max(requested, self.cfg.min_move_seconds)
        try:
            here = np.asarray(self._robot.get_cartesian(arm), dtype=float)[:3]
            dist = float(np.linalg.norm(np.asarray(xyz, dtype=float)[:3] - here))
        except Exception:
            return max(requested, self.cfg.min_move_seconds)
        return max(dist / cap, self.cfg.min_move_seconds)

    def settle(self, seconds: float = 1.0) -> None:
        """Let the world come to rest. No motion is commanded, so no check applies."""
        inner = getattr(self._robot, "settle", None)
        if callable(inner):
            inner(seconds)

    def rest(self, arm: str) -> None:
        """Fold one arm to its sleep pose (post-verdict tidy-up). The backend's
        rest pose is fixed and inside the envelope by construction, so only the
        e-stop gate applies."""
        if self._stopped:
            raise EStop("software e-stop is engaged")
        inner = getattr(self._robot, "rest", None)
        if callable(inner) and not self.cfg.dry_run:
            inner(arm)

    def move_cartesian(self, arm: str, xyz: np.ndarray, seconds: float = 2.0,
                       orientation=None) -> None:
        xyz = np.asarray(xyz, dtype=float)
        self._check_target(arm, xyz)
        seconds = self._segment_seconds(arm, xyz, seconds)
        self._confirm(f"move {arm} arm to {np.round(xyz, 3).tolist()} over {seconds:.1f}s")
        if self.cfg.dry_run:
            return
        if orientation is not None:
            # Only forwarded when a skill explicitly commands a wrist pose (a
            # yawed grasp); backends without the parameter never see it.
            self._robot.move_cartesian(arm, xyz, seconds=seconds,
                                       orientation=np.asarray(orientation, dtype=float))
        else:
            self._robot.move_cartesian(arm, xyz, seconds=seconds)

    def get_cartesian(self, arm: str) -> np.ndarray:
        return self._robot.get_cartesian(arm)

    def move_pose(self, arm: str, xyz: np.ndarray, rotation=None, seconds: float = 3.0) -> None:
        """Position and wrist orientation together, on backends that have it.

        Gated by the same workspace check as move_cartesian — an orientation
        argument is not a reason to skip the box the arm is allowed to be in.
        """
        inner = getattr(self._robot, "move_pose", None)
        if not callable(inner):
            raise NotImplementedError(
                f"{type(self._robot).__name__} cannot command wrist orientation; "
                "skills that need a non-vertical approach are unavailable on it")
        xyz = np.asarray(xyz, dtype=float)
        self._check_target(arm, xyz)
        seconds = self._segment_seconds(arm, xyz, seconds)
        self._confirm(f"move {arm} arm to {np.round(xyz, 3).tolist()} with a commanded wrist")
        if self.cfg.dry_run:
            return
        inner(arm, xyz, rotation=rotation, seconds=seconds)

    def can_orient(self) -> bool:
        return callable(getattr(self._robot, "move_pose", None))

    def reachable(self, arm: str, xyz) -> bool:   # noqa: D401 - see _check_target
        """Whether the backend can hold its approach pose there. Backends without
        a kinematic model say yes: they have no way to know otherwise, and a
        blanket no would disable the height search entirely."""
        (x0, x1), (y0, y1), (z0, z1) = self.cfg.workspace_x, self.cfg.workspace_y, self.cfg.workspace_z
        x, y, z = (float(v) for v in xyz)
        if not (x0 <= x <= x1 and y0 <= y <= y1 and z0 <= z <= z1):
            return False
        inner = getattr(self._robot, "reachable", None)
        return bool(inner(arm, xyz)) if callable(inner) else True

    def set_gripper(self, arm: str, width_m: float, effort_limit: float | None = None) -> None:
        if self._stopped:
            raise EStop("software e-stop is engaged")
        limit = min(effort_limit or self.cfg.gripper_effort_limit, self.cfg.gripper_effort_limit)
        if self.cfg.dry_run:
            return
        self._robot.set_gripper(arm, float(np.clip(width_m, 0.0, 0.09)), effort_limit=limit)

    def get_gripper(self, arm: str) -> dict[str, float]:
        return self._robot.get_gripper(arm)

    def felt_descend(self, arm, xyz_floor, speed_mps, threshold_n):
        inner = getattr(self._robot, "felt_descend", None)
        if inner is None:
            raise AttributeError("backend has no felt_descend")
        self._check_target(arm, np.asarray(xyz_floor, dtype=float))
        return inner(arm, xyz_floor, speed_mps, threshold_n)

    def get_external_effort(self, arm: str) -> float:
        fn = getattr(self._robot, "get_external_effort", None)
        return float(fn(arm)) if fn else 0.0

    def capture(self, camera: str) -> Frame:
        return self._robot.capture(camera)

    def camera_calibrated(self, camera: str) -> bool:
        """Whether the backend can put this camera's pixels in the arm's frame.

        Forwarded rather than answered here. A backend that does not know is not
        the same as a camera that is not calibrated, so the absence of an answer
        has to reach `_usable_cameras` as an absence — otherwise this wrapper
        would quietly veto the twin's second viewpoint by saying "no" on the
        real backend's behalf.
        """
        fn = getattr(self._robot, "camera_calibrated", None)
        if not callable(fn):
            raise AttributeError("backend does not report camera calibration")
        return bool(fn(camera))

    def estop(self) -> None:
        self._stopped = True
        try:
            self._robot.stop()
        except Exception:
            pass

    def stop(self) -> None:
        self._robot.stop()

    def shutdown(self) -> None:
        self._robot.shutdown()
