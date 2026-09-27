"""Real backend for the Trossen AI Stationary kit (2x WidowX AI followers).

Verified against trossen-arm 1.11.0:
- TrossenArmDriver.configure(Model.wxai_v0, StandardEndEffector.wxai_v0_follower, ip, clear_error)
- set_all_modes(Mode.position); set_cartesian_positions([x,y,z,rx,ry,rz], InterpolationSpace.cartesian, goal_time, blocking)
- set_gripper_position(m) / set_gripper_external_effort(N, sign: +open/-close)
- get_all_external_efforts() — gravity/friction-compensated; last entry is the gripper (N)
- errors idle+brake all joints; clear with configure(..., clear_error=True)

Kit IP convention: follower right 192.168.1.4, follower left 192.168.1.5.

Frames: Heron's world frame is defined as the RIGHT follower's base frame.
The left arm's t_world_base and all camera extrinsics come from calibration
(configs/stationary.yaml + `heron calibrate`).

Orientation policy: the end-effector orientation recorded at connect time (arms
manually posed top-down once) is held for every cartesian move — primitives
only translate. This avoids guessing angle-axis targets before the rig is
characterized.
"""
from __future__ import annotations

import sys
import time
from typing import Any

import numpy as np

from ..config import HeronConfig
from ..types import Frame
from .cameras import RealSenseRig

def _rodrigues(rvec: np.ndarray) -> np.ndarray:
    """Angle-axis -> rotation matrix (the driver's cartesian orientation encoding)."""
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-9:
        return np.eye(3)
    k = np.asarray(rvec, dtype=float) / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * kx + (1 - np.cos(theta)) * (kx @ kx)


GRIPPER_DRIVER_MAX = 0.04  # meters of carriage travel per the WXAI spec
GRIPPER_APERTURE_RATIO = 2.0  # commanded aperture -> carriage position divisor
CLOSE_EFFORT_N = -20.0  # gentle default grasp force (max spec is 100 N)
GRIPPER_CLOSE_MODE = "contact"   # "contact" (velocity approach, force hold) | "ramp" (legacy)
GRIPPER_CLOSE_SPEED = 0.012      # carriage m/s for the position-mode approach (~24 mm/s aperture)
# Joint-space sweeps (home, rest) are the only motions the cartesian speed cap
# cannot see: they interpolate the whole swing in one goal time, and at 3.0 s a
# full return read as "sometimes it moves fast" to the operator standing beside
# it. One knob, scaled by speed_scale like everything else; rest's fold gets
# two extra seconds because it ends against the base.
JOINT_SWEEP_SECONDS = 5.0
# The cartesian cap prices TOOL distance; a move from the ready pose can swing
# the elbow through a large angle while the tool travels modestly, and that
# swing is what reads as "too fast" beside the table. Every joint-space goal
# time is floored so no joint exceeds this angular speed.
JOINT_SPEED_CAP_RADPS = 0.5
# SPEED, NOT DURATION (Yifan, 2026-09-02): a program's `seconds=` used to be
# the goal time of every move, so a 3 cm hop and a 30 cm transit both took the
# 3 s the program wrote down, and a staged move (cross + descend) took it
# twice. Goal times are now derived from the tool distance at a fixed speed
# (min of FREE_TOOL_SPEED_MPS and cfg.safety.max_tool_speed_mps; joint cap and
# speed_scale still apply). The caller's `seconds` is ignored for transit
# moves and read as a SPEED request (dist/seconds) for drags, where the
# contact speed is a mechanism choice.
FREE_TOOL_SPEED_MPS = 0.20
MIN_SEGMENT_SECONDS = 0.3
SETTLE_STILL_RAD = 2e-3        # settle(): "still" = no joint moved this much in 0.1 s

HOME_JOINTS = [0.0, np.pi / 2, np.pi / 2, 0.0, 0.0, 0.0]  # official demo ready pose
TRAVEL_CLEARANCE_M = 0.15  # height above a table-level goal to translate at
# Below this horizontal displacement a move is a pure descent or retreat, and
# staging it (cross high, then come down) would lift the tool out of the very
# motion it was asked to make. Larger than the arm's own tracking error, smaller
# than any deliberate reposition.
VERTICAL_MOVE_TOLERANCE_M = 0.005
# Approach angles to try, in order, when the wrist cannot reach a point straight
# down. Vertical first because it is the easiest grasp to reason about; the rest
# lean the gripper back toward the base, which is the direction that buys reach
# on this arm. A cuRobo sweep put an exactly vertical approach at 25.7% of the
# workspace and a 45-degree one at 98.1%, so this is where nearly all of the
# reachable workspace actually lives.
APPROACH_TILT_FALLBACKS_RAD = tuple(np.radians([0, 15, 30, 45, 60]))

# Gravity-droop trim (see TrossenStationary._droop_trim). Trim only low final
# legs (precision work), only real moves (contact probes step ~12 mm and must
# stay pure force-feel), only residuals big enough to matter and small enough
# to still BE droop rather than a collision or a wrong frame.
DROOP_TRIM_MAX_Z_M = 0.10
DROOP_TRIM_MIN_MOVE_M = 0.04
DROOP_TRIM_MIN_M = 0.006
DROOP_TRIM_MAX_M = 0.04


def is_vertical_move(previous, target) -> bool:
    """Is this move a pure descent or retreat from the last commanded pose?

    Deliberately compares COMMANDED poses rather than the measured one. The
    measured pose carries the servo's tracking error, and in the MuJoCo twin
    that error reaches 40 mm in xy at the hover — larger than any tolerance
    worth having, so a test against it answers "no" to every genuine descent
    and the staging never gets skipped.
    """
    if previous is None:
        return False
    a = np.asarray(previous, dtype=float)[:2]
    b = np.asarray(target, dtype=float)[:2]
    return float(np.linalg.norm(a - b)) < VERTICAL_MOVE_TOLERANCE_M


def _tilt_toward_base(rvec: np.ndarray, target_in_arm: np.ndarray,
                      tilt_rad: float) -> np.ndarray:
    """Lean an approach orientation back toward the arm's base by `tilt_rad`.

    The tilt happens in the vertical plane containing the target, so the gripper
    still approaches along the line it would have, just from an angle the arm can
    actually hold. Directly above the base there is no such plane and the
    orientation is returned unchanged — a degenerate case, not an error.
    """
    if abs(tilt_rad) < 1e-9:
        return np.asarray(rvec, dtype=float)
    horiz = np.asarray(target_in_arm, dtype=float)[:2]
    n = float(np.linalg.norm(horiz))
    if n < 1e-6:
        return np.asarray(rvec, dtype=float)
    d = horiz / n
    axis = np.array([-d[1], d[0], 0.0])          # horizontal, perpendicular to d
    r_new = _rodrigues(axis * tilt_rad) @ _rodrigues(np.asarray(rvec, dtype=float))
    return _rotvec(r_new)


def _rotvec(r: np.ndarray) -> np.ndarray:
    """Rotation matrix -> angle-axis. The inverse of _rodrigues."""
    cos = (np.trace(r) - 1.0) / 2.0
    theta = float(np.arccos(np.clip(cos, -1.0, 1.0)))
    if theta < 1e-9:
        return np.zeros(3)
    if abs(np.pi - theta) < 1e-6:
        # Near 180 degrees the skew part vanishes; recover the axis from the
        # symmetric part instead, where it is still well conditioned.
        a = (r + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(a), 0.0))
        k = int(np.argmax(axis))
        if axis[k] > 1e-9:
            axis = a[:, k] / axis[k]
        return axis / max(np.linalg.norm(axis), 1e-12) * theta
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return w / (2.0 * np.sin(theta)) * theta


class _NoRig:
    """Stands in when every camera comes from the depth sidecar.

    It exists so the backend has something to call `close()` on, and so that a
    capture for an unserved camera fails with a sentence explaining why rather
    than with a driver error about hardware nobody meant to touch.
    """

    def capture(self, camera: str, t_base_cam=None):
        raise RuntimeError(
            f"camera {camera!r} is not served by the depth sidecar and has no "
            "local rig — every configured camera is on the sidecar, so this one "
            "was either disabled or misspelled")

    def close(self) -> None:
        pass


class TrossenStationary:
    def __init__(self, cfg: HeronConfig) -> None:
        try:
            import trossen_arm  # noqa: PLC0415
        except ImportError as e:
            raise RuntimeError("trossen-arm is not installed; pip install 'heron[real]' on the robot host") from e
        self._ta = trossen_arm
        self.cfg = cfg
        self.arms = list(cfg.arms)
        self.cameras = list(cfg.cameras)
        self._drivers: dict[str, Any] = {}
        self._t_world_base: dict[str, np.ndarray] = {}
        self._t_base_world: dict[str, np.ndarray] = {}
        self._hold_orientation: dict[str, np.ndarray] = {}
        self._last_tilt: dict[str, float] = {}
        # Last pose COMMANDED, per arm, in that arm's frame. Only used to tell a
        # pure descent from a reposition; see is_vertical_move.
        self._last_target: dict[str, np.ndarray] = {}
        # Last JOINT command per arm, and last commanded gripper aperture — the
        # recorder's "action" stream. The driver interpolates internally and
        # never exposes its setpoint, so the goal we handed it is the honest
        # record of what the controller is driving toward.
        self._last_joint_cmd: dict[str, np.ndarray] = {}
        self._gripper_cmd: dict[str, float] = {}
        from .kinematics import Kinematics  # noqa: PLC0415
        self._kin = Kinematics()
        for side, arm_cfg in cfg.arms.items():
            drv = trossen_arm.TrossenArmDriver()
            # A LATCHED CONTROLLER ERROR OUTLIVES THE PROCESS. Motor 3's
            # temperature sensor read 0.0 C one night, the firmware's
            # fail-safe idled the arm, and every subsequent episode died at
            # startup — an unattended run went from 150 episodes to one.
            # Clear only when the plain configure refuses, and say so loudly:
            # a fault that keeps coming back is a fault, not a formality.
            try:
                drv.configure(trossen_arm.Model.wxai_v0,
                              trossen_arm.StandardEndEffector.wxai_v0_follower,
                              arm_cfg.ip, False)
            except Exception as e:
                print(f"[heron] {side} arm had a latched controller error "
                      f"({type(e).__name__}: {str(e)[:120]}); clearing it",
                      file=sys.stderr, flush=True)
                drv.configure(trossen_arm.Model.wxai_v0,
                              trossen_arm.StandardEndEffector.wxai_v0_follower,
                              arm_cfg.ip, True)
            extra = float(getattr(arm_cfg, "ee_extra_mass_kg", 0.0) or 0.0)
            if extra > 0.0:
                # Tell the gravity feed-forward about the camera on the wrist.
                # The standard wxai_v0_follower palm is 0.6423 kg with no
                # payload; the D405 + mount + cable ride there on this rig,
                # and the unmodelled ~140 g under-compensates gravity — the
                # shortfall grows with reach and reads as droop.
                ee = drv.get_end_effector()
                m0 = float(ee.palm.mass)
                c0 = np.asarray(ee.palm.origin_xyz, dtype=float)
                cx = np.asarray(getattr(arm_cfg, "ee_extra_com_m",
                                        [0.05, 0.0, 0.05]), dtype=float)
                ee.palm.mass = m0 + extra
                ee.palm.origin_xyz = ((m0 * c0 + extra * cx) / (m0 + extra)).tolist()
                drv.set_end_effector(ee)
                print(f"[heron] {side}: gravity model +{extra * 1000:.0f} g wrist "
                      f"payload (palm {m0:.3f} -> {m0 + extra:.3f} kg)", flush=True)
            drv.set_all_modes(trossen_arm.Mode.position)
            self._drivers[side] = drv
            t = np.asarray(arm_cfg.t_world_base, dtype=float)
            self._t_world_base[side] = t
            self._t_base_world[side] = np.linalg.inv(t)
            # Table-facing by default: inheriting the current (often horizontal)
            # wrist orientation is what drives the gripper into the table.
            self._hold_orientation[side] = np.asarray(arm_cfg.approach_rvec, dtype=float)
            # How far off vertical the last motion actually had to lean. Worth
            # recording: a grasp planned for a straight-down gripper but executed
            # at 45 degrees closes on a different part of the object.
            self._last_tilt[side] = 0.0
        # Per-camera source: sidecar depth server > UVC index > RealSense SDK.
        self._depth_clients: dict[str, Any] = {}
        for name, cam in cfg.cameras.items():
            if cam.depth_server:
                from pathlib import Path  # noqa: PLC0415

                from .depthclient import DepthServerClient  # noqa: PLC0415
                from .workspace import load_extrinsics  # noqa: PLC0415

                t = None
                if cam.extrinsics_file and Path(cam.extrinsics_file).exists():
                    t = load_extrinsics(cam.extrinsics_file)
                # Hand the homography over too. A camera with depth but no
                # extrinsics would otherwise have no route to 3D at all, making
                # the RGB-D upgrade a downgrade.
                hpw, pz, wh = None, 0.0, None
                if cam.homography_file and Path(cam.homography_file).exists():
                    hom = np.load(cam.homography_file, allow_pickle=True)
                    hpw = hom["h_pixel_world"]
                    pz = float(np.ravel(hom["plane_z"])[0])
                    if "image_wh" in hom.files:
                        wh = tuple(int(v) for v in np.ravel(hom["image_wh"]))
                    else:
                        # Fitted on the UVC stream, which this rig runs at 720p.
                        wh = (1280, 720)
                self._depth_clients[name] = DepthServerClient(
                    cam.depth_server, t_base_cam=t, h_pixel_world=hpw, plane_z=pz,
                    homography_wh=wh)
        # A rig is only needed for the cameras the sidecar is NOT serving. When
        # it serves them all, constructing one anyway makes librealsense try to
        # seize devices the sidecar already owns, and every capture fails with
        # "no device connected" — for cameras nothing was going to ask it for.
        remaining = {n: c for n, c in cfg.cameras.items() if not c.depth_server}
        if not remaining:
            self._rig = _NoRig()
        elif any(c.index is not None for c in remaining.values()):
            from .uvc import UVCRig  # noqa: PLC0415  (lab Mac: no librealsense over ssh)

            self._rig = UVCRig(cfg)
        else:
            self._rig = RealSenseRig(cfg)

    # -- frame conversion ----------------------------------------------------
    def _world_to_arm(self, side: str, xyz_world: np.ndarray) -> np.ndarray:
        p = self._t_base_world[side] @ np.array([*xyz_world, 1.0])
        return p[:3]

    def _arm_to_world(self, side: str, xyz_arm: np.ndarray) -> np.ndarray:
        p = self._t_world_base[side] @ np.array([*xyz_arm, 1.0])
        return p[:3]

    # -- RobotInterface ------------------------------------------------------
    def home(self, arm: str) -> None:
        """Return to the ready pose.

        NOTE: this interpolates joints straight there, so from a pose near the
        table the tool can drag across it. The rig's own
        remote_model_experiment/reset_all_to_staged_safe.py is the reset to use
        when the arm is down at the table — ./home calls that.
        """
        drv = self._drivers[arm]
        self._last_joint_cmd[arm] = np.asarray(HOME_JOINTS, dtype=float)
        # set_ARM_positions, not set_all_positions: after a force close the
        # gripper joint is in external_effort mode, and commanding its
        # POSITION raises "Requested to set joint 6 position but it is in
        # mode external_effort" — move_cartesian learned this (see above);
        # home had the same bug and it fired on every regrasp-after-hold.
        drv.set_arm_positions(np.asarray(HOME_JOINTS, dtype=float),
                              JOINT_SWEEP_SECONDS / max(float(self.cfg.safety.speed_scale), 1e-3), True)
        # Homing moves in joint space, so there is no commanded cartesian pose
        # any more and the next move must be staged like any other reposition.
        self._last_target.pop(arm, None)
        # Deliberately NOT adopting the ready pose's orientation — it is
        # horizontal, and every subsequent table motion would inherit it.

    def rest(self, arm: str) -> None:
        """Fold the arm to its sleep pose: lift clear, then fold. No ready
        detour — the ready pose reaches toward the table CENTRE, which made
        rest slow and, with two arms, sent both paths through the same air.
        A vertical lift at the current xy clears the table; the fold to zeros
        sweeps toward this arm's OWN base, away from the other one.
        """
        try:
            cur = self.tool_position(arm)
            if cur is not None and cur[2] < 0.18:
                self.move_cartesian(arm, np.array([cur[0], cur[1], 0.20]),
                                    seconds=2.0, staged=False)
        except Exception:
            pass          # folding from wherever we stand beats not folding
        drv = self._drivers[arm]
        drv.set_arm_positions(np.zeros(len(HOME_JOINTS)),
                              (JOINT_SWEEP_SECONDS + 2.0) / max(float(self.cfg.safety.speed_scale), 1e-3), True)
        self._last_target.pop(arm, None)

    def move_cartesian(self, arm: str, xyz: np.ndarray, seconds: float = 2.0,
                       orientation: np.ndarray | None = None, staged: bool = True) -> None:
        """Move the tool to a world point.

        The driver interpolates a STRAIGHT LINE in cartesian space, so a direct
        command from a high pose to a table-level pose sweeps the wrist down
        through the workspace diagonally — that path is what collides with the
        table (and with the rail on this rig), and the failure surfaces as an IK
        error at an intermediate waypoint rather than at the goal.

        So descents are staged: reorient while high, translate above the target,
        then descend vertically. Set staged=False for a deliberate straight line.
        """
        drv = self._drivers[arm]
        rvec = self._hold_orientation[arm] if orientation is None else np.asarray(orientation, float)
        target = self._world_to_arm(arm, np.asarray(xyz, dtype=float))
        previous = self._last_target.get(arm)
        self._last_target[arm] = target.copy()

        scale = max(float(self.cfg.safety.speed_scale), 1e-3)

        def solve(p, tilt_first=True):
            """Joint angles for a point, trying tilts, or None. No motion."""
            q_now = np.asarray(drv.get_all_positions(), dtype=float)[:self._kin.dof]
            angles = APPROACH_TILT_FALLBACKS_RAD if tilt_first else (0.0,)
            for tilt in angles:
                q = self._kin.ik(p, _tilt_toward_base(rvec, p, tilt), q_seed=q_now)
                if q is not None:
                    return q, tilt
            return None, None

        def go(p, secs):
            # We solve IK ourselves and command JOINT positions.
            #
            # The controller's own solver is closed-form, and a closed form for
            # six axes assumes a spherical wrist; where the geometry does not
            # oblige it covers only part of the reachable set and reports the
            # rest as unreachable. Our solver is numerical and multi-seeded, and
            # it was checked against the hardware before being trusted: FK at
            # the arm's measured joint angles reproduces the pose the controller
            # reports to 0.06 mm, and it independently agrees that
            # [0.347, 0.175, 0.205] has no vertical solution while the same
            # point at z <= 0.10 does.
            q, tilt = solve(p)
            if q is None:
                raise RuntimeError(
                    f"no IK solution at {np.round(p, 3).tolist()} for any approach "
                    f"from 0 to {np.degrees(APPROACH_TILT_FALLBACKS_RAD[-1]):.0f} "
                    f"degrees off vertical")
            self._last_tilt[arm] = float(tilt)
            q_now = np.asarray(drv.get_all_positions(), dtype=float)[: len(q)]
            swing = float(np.max(np.abs(np.asarray(q) - q_now)))
            here = np.asarray(drv.get_cartesian_positions(), dtype=float)[:3]
            dist = float(np.linalg.norm(np.asarray(p, float) - here))
            # `secs` (the caller's duration) is deliberately unused: speed, not time.
            secs = max(dist / self._free_speed(), swing / JOINT_SPEED_CAP_RADPS,
                       MIN_SEGMENT_SECONDS)
            self._last_joint_cmd[arm] = np.asarray(q, dtype=float)
            # set_ARM_positions, not set_all_positions: the latter commands the
            # gripper joint too, and after a force-close that joint is in
            # external_effort mode — the controller then refuses the whole
            # motion with "Requested to set joint 6 position but it is in
            # external_effort mode", so the arm could not move while holding
            # anything. The grip must stay a force, not become a position.
            drv.set_arm_positions(q, secs / scale, True)

        if not staged:
            go(target, seconds)
            return
        # A move that does not go anywhere horizontally has no diagonal to avoid,
        # and staging it is worse than useless: the tool is lifted to the travel
        # height and brought back down. Measured in the twin — one commanded 5 mm
        # descent moved the tool 232 mm, up 114 and down again — and
        # descend_to_contact issues that motion once per probing step, which is
        # exactly what the operator saw as the arm juddering during a place.
        if is_vertical_move(previous, target):
            go(self._droop_feedforward(arm, target, previous), seconds)
            return
        # Clearance is measured from the TABLE, not from wherever the tool happens
        # to be. Travelling at the current height sounds safer but pins the wrist
        # into poses the arm cannot hold: near full extension a straight-down
        # gripper has no IK solution, so a high-altitude crossing fails outright
        # while the low goal underneath it is perfectly reachable.
        # ... but only as high as the arm can actually hold the pose there. A
        # fixed clearance is a guess about reach, and at (0.347, 0.175) it was
        # wrong: nothing above z=0.10 has a vertical solution while the object
        # underneath at z=0 does, so the crossing failed for a grasp that was
        # perfectly reachable. Ask the solver instead of assuming.
        want = float(self.cfg.table_z) + TRAVEL_CLEARANCE_M
        travel_z = None
        for z in np.arange(want, target[2] + 0.02, -0.02):
            if solve(np.array([target[0], target[1], z]))[0] is not None:
                travel_z = float(z)
                break
        if travel_z is None or target[2] >= travel_z - 0.005:
            # Nothing higher is reachable: go direct. No droop feed-forward —
            # the pre-move pose is unrelated to the target's posture, so a
            # residual read here would import the WRONG droop.
            go(target, seconds)
            return
        if travel_z < want - 0.005:
            self._log_clearance(arm, want, travel_z)
        go(np.array([target[0], target[1], travel_z]), seconds)   # cross above the goal
        # then straight down, aimed past the droop measured at altitude
        go(self._droop_feedforward(
            arm, target, np.array([target[0], target[1], travel_z])),
            max(seconds * 0.5, 1.0))

    def _droop_feedforward(self, arm: str, target: np.ndarray, previous):
        """Aim a low final leg past the gravity droop measured at altitude.

        At long reach the servos track the commanded joints loosely under
        gravity and the tool lands radially SHORT of the command — measured on
        the right arm 2026-08-19 via carry offsets: 0.2 mm at r=0.29, 2.9 mm at
        r=0.33, 19.6 mm at r=0.37, 15.8 mm at r=0.45, the latter two exactly
        along -r̂. Invisible to IK (the solution exists and is commanded; the
        joints just do not get there).

        The first fix corrected AFTER arrival — a lateral re-command at grasp
        height, where the open fingers straddle the object; a 10-20 mm sweep
        at cube height clipped corners and every grip came up crooked (Yifan,
        same morning). So measure BEFORE descending instead: the arm already
        hovers at the target's xy, its residual there is the droop (same
        extended posture, altitude changes it little), and folding it into
        the descent target means the fingers only ever travel vertically.
        Costs nothing: no extra command, no cameras, no constants.
        """
        t = np.asarray(target, float)
        # OFF by default (2026-08-19 evening): the residual this reads is the
        # CONTROLLER's own FK, and on the -y half of the right arm's envelope
        # the FK is itself wrong under load — link flex carries the physical
        # tool ~20-30 mm PAST the command while the encoders read SHORT, so a
        # mirrored correction moves the fingers the wrong way twice over. The
        # wrist camera (wrist_refine) is the sensor that survives this: it
        # measures object-relative-to-gripper through the same lying chain,
        # so the lie cancels. Keep this available for rigs whose FK is honest.
        if not bool(getattr(self.cfg, "droop_feedforward", False)):
            return t
        if float(t[2]) > float(self.cfg.table_z) + DROOP_TRIM_MAX_Z_M:
            return t  # only precision work near the table is worth the read
        if previous is not None and \
                float(np.linalg.norm(t - np.asarray(previous, float))) \
                < DROOP_TRIM_MIN_MOVE_M:
            return t  # contact-probe micro-steps must stay pure force-feel
        prev_xy = (np.asarray(previous, float)[:2]
                   if previous is not None else t[:2])
        time.sleep(0.1)
        meas = np.asarray(self._drivers[arm].get_cartesian_positions(),
                          dtype=float)[:3]
        err = prev_xy - meas[:2]
        en = float(np.linalg.norm(err))
        if en < DROOP_TRIM_MIN_M:
            return t
        if en > DROOP_TRIM_MAX_M:
            print(f"[heron] {arm}: hovering {en * 1000:.0f} mm from its command "
                  f"— beyond droop, not compensating", flush=True)
            return t
        out = t.copy()
        out[:2] += err
        print(f"[heron] {arm}: droop feed-forward {en * 1000:.1f} mm "
              f"into the descent", flush=True)
        return out

    def _log_clearance(self, arm: str, wanted: float, used: float) -> None:
        """Travelling lower than intended is worth saying out loud: it means the
        gripper passes closer to whatever is on the table than the design
        assumed."""
        print(f"[heron] {arm}: travel clearance {wanted:.3f} m is unreachable here; "
              f"crossing at {used:.3f} m instead", flush=True)

    def reachable(self, arm: str, xyz) -> bool:
        """Can the tool hold the approach orientation at this world point?

        Asked before committing to a height, because on this arm the answer is
        often no for reasons that have nothing to do with the goal: a
        straight-down wrist above roughly 0.15 m has no solution over most of
        the table, while the same (x, y) at object height is fine.
        """
        target = self._world_to_arm(arm, np.asarray(xyz, dtype=float))
        rvec = self._hold_orientation[arm]
        q_now = np.asarray(self._drivers[arm].get_all_positions(), dtype=float)[:self._kin.dof]
        for tilt in APPROACH_TILT_FALLBACKS_RAD:
            if self._kin.ik(target, _tilt_toward_base(rvec, target, tilt),
                            q_seed=q_now, seeds=8) is not None:
                return True
        return False

    def drag_cartesian(self, arm: str, xyz: np.ndarray, seconds: float = 2.0) -> None:
        """One CONTINUOUS straight-line move at the current orientation.

        The staged mover exists to keep transit moves from sweeping the wrist
        through the table — but a deliberate table-level DRAG (brushing,
        pushing, wiping) is exactly a straight line at contact height, and
        chaining sub-5 mm staged steps to fake one is why a 7 cm sweep took
        17 commands (rig_sweep v7). The driver's own cartesian-space
        interpolation does this in one command with a real straightness
        guarantee, which joint-space interpolation cannot give over 30 cm.
        """
        drv = self._drivers[arm]
        target = self._world_to_arm(arm, np.asarray(xyz, dtype=float))
        pose = np.asarray(drv.get_cartesian_positions(), dtype=float)
        dist = float(np.linalg.norm(target - pose[:3]))
        scale = max(float(self.cfg.safety.speed_scale), 1e-3)
        # The caller's `seconds` is read as a SPEED (dist/seconds at the time
        # of the call); the goal time is that speed over the actual distance,
        # floored by MIN_SEGMENT_SECONDS, not by min_move_seconds.
        speed = dist / float(seconds) if float(seconds) > 0 and dist > 1e-4 else self._free_speed()
        cap = float(self.cfg.safety.max_tool_speed_mps)
        if cap > 0:
            speed = min(speed, cap)
        secs = max(dist / max(speed, 1e-3), MIN_SEGMENT_SECONDS)
        goal = list(target) + list(pose[3:6])
        drv.set_cartesian_positions(
            goal, self._ta.InterpolationSpace.cartesian, secs / scale, True)
        self._last_target[arm] = np.asarray(target, dtype=float)

    def _free_speed(self) -> float:
        """Free-space tool speed: FREE_TOOL_SPEED_MPS under the config cap."""
        cap = float(getattr(self.cfg.safety, "max_tool_speed_mps", 0.0) or 0.0)
        return min(FREE_TOOL_SPEED_MPS, cap) if cap > 0 else FREE_TOOL_SPEED_MPS

    def settle(self, seconds: float = 1.0) -> None:
        """Wait until every arm is still (no joint moving), then 0.2 s of grace
        for a released object. `seconds` is only the cap on the wait: a settle
        after a completed move returns in ~0.3 s instead of sleeping it out."""
        cap = max(0.2, float(seconds))
        t0 = time.time()
        prev = {a: np.asarray(d.get_all_positions(), dtype=float) for a, d in self._drivers.items()}
        while time.time() - t0 < cap:
            time.sleep(0.1)
            now = {a: np.asarray(d.get_all_positions(), dtype=float) for a, d in self._drivers.items()}
            if all(float(np.max(np.abs(now[a] - prev[a]))) < SETTLE_STILL_RAD for a in now):
                break
            prev = now
        time.sleep(0.2)

    def get_cartesian(self, arm: str) -> np.ndarray:
        pose = np.asarray(self._drivers[arm].get_cartesian_positions(), dtype=float)
        return self._arm_to_world(arm, pose[:3])

    def arm_polyline(self, arm: str) -> np.ndarray:
        """The arm's skeleton (joint origins + tool), world frame."""
        q = np.asarray(self.joint_state(arm), dtype=float)[:6]
        pts = self._kin.link_points(q)
        t = self._t_world_base[arm]
        return pts @ t[:3, :3].T + t[:3, 3]

    def move_cartesian_path(self, arm: str, points, seconds: float = 4.0,
                            orientation: np.ndarray | None = None) -> None:
        """One SMOOTH motion through several waypoints — no stop in the middle.

        A chain of move_cartesian calls starts and ends every segment at zero
        velocity: the stop-and-go judder a spectator sees on a handover
        transport (Yifan, 2026-08-19). Here every waypoint is IK-solved up
        front (each seeded by its predecessor, keeping the solution on one
        configuration branch — and failing BEFORE anything moves), the knots
        are joined by a clamped per-joint cubic spline (zero velocity only at
        the two ends), and the spline is streamed to the controller at 20 Hz
        with a short lookahead, so the joints chase a moving target instead
        of braking at each knot.

        `orientation` is an angle-axis rvec — one held over the whole path,
        or a LIST with one per waypoint (the wrist reorients through the
        transport; the joint-space spline blends attitude exactly as it
        blends position, so per-knot attitudes cost nothing extra).
        """
        drv = self._drivers[arm]
        pts = [self._world_to_arm(arm, np.asarray(p, dtype=float)) for p in points]
        if len(pts) < 2:
            raise ValueError("move_cartesian_path wants at least 2 waypoints")
        if orientation is None:
            rvecs = [self._hold_orientation[arm]] * len(pts)
        else:
            o = np.asarray(orientation, dtype=float)
            rvecs = [o] * len(pts) if o.ndim == 1 else [np.asarray(r, float) for r in o]
        if len(rvecs) != len(pts):
            raise ValueError("one orientation per waypoint (or a single rvec)")
        scale = max(float(self.cfg.safety.speed_scale), 1e-3)

        # IK every knot before any motion; seed each solve with the previous
        # solution so the arm stays on one branch across the whole path.
        q_seed = np.asarray(drv.get_all_positions(), dtype=float)[: self._kin.dof]
        knots = [q_seed]
        for p, rvec in zip(pts, rvecs):
            q = None
            for tilt in APPROACH_TILT_FALLBACKS_RAD:
                q = self._kin.ik(p, _tilt_toward_base(rvec, p, tilt), q_seed=knots[-1])
                if q is not None:
                    break
            if q is None:
                raise RuntimeError(
                    f"move_cartesian_path: no IK solution at "
                    f"{np.round(p, 3).tolist()} — nothing was moved")
            knots.append(np.asarray(q, dtype=float))
        Q = np.stack(knots)

        # Knot times: proportional to cartesian arc length, floored so no
        # joint ever exceeds the speed cap on any segment.
        arc = np.array([0.0] + [float(np.linalg.norm(pts[i] - (pts[i - 1] if i else
                        np.asarray(self.get_cartesian_arm(arm), dtype=float))))
                        for i in range(len(pts))])
        arc = np.maximum(arc, 1e-4)
        seg = arc[1:] / arc[1:].sum()
        # 1.25: a cubic spline's peak velocity exceeds the linear segment
        # average by up to ~1.5x mid-segment; the margin keeps the peak under
        # the cap without measuring the spline.
        # `seconds` unused on purpose: the path runs at the free-space speed
        # over its arc length, floored by the joint cap.
        total = max(float(arc[1:].sum()) / self._free_speed(),
                    1.25 * float(np.max(np.abs(np.diff(Q, axis=0)), axis=1).sum()
                                 / JOINT_SPEED_CAP_RADPS),
                    MIN_SEGMENT_SECONDS) / scale
        t_knots = np.concatenate([[0.0], np.cumsum(seg) * total])

        from scipy.interpolate import CubicSpline  # noqa: PLC0415
        spline = CubicSpline(t_knots, Q, axis=0, bc_type="clamped")

        dt = 0.05
        self._last_joint_cmd[arm] = Q[-1]
        self._last_target[arm] = pts[-1].copy()
        t0 = time.time()
        while True:
            t = time.time() - t0
            if t >= total:
                break
            drv.set_arm_positions(spline(min(t + 2 * dt, total)), 2 * dt, False)
            time.sleep(dt)
        drv.set_arm_positions(Q[-1], max(0.3, 2 * dt), True)

    def get_cartesian_arm(self, arm: str):
        """Current tool xyz in this ARM's own frame (path-planning helper)."""
        world = np.asarray(self.get_cartesian(arm), dtype=float)
        return self._world_to_arm(arm, world)

    def set_gripper(self, arm: str, width_m: float, effort_limit: float | None = None) -> None:
        drv = self._drivers[arm]
        # For the recorder: a force close is commanded as "shut", which a
        # policy consumes as aperture 0 — the force detail is a controller
        # concern, not a dataset one.
        self._gripper_cmd[arm] = 0.0 if width_m <= 0.01 else float(width_m)
        if width_m <= 0.01:
            effort = -abs(effort_limit if effort_limit is not None else abs(CLOSE_EFFORT_N))
            if GRIPPER_CLOSE_MODE == "contact":
                # Two-stage close (2026-09-01, traced on the right gripper).
                # A force-ramp close from rest sat still for 1.1 s, slammed
                # 66 mm shut in 0.5 s and bounced 25 mm open before settling:
                # an under-damped force loop meeting the object at speed. The
                # external-effort reading is no contact signal either (it
                # sits at -8..-12 N in free motion). What IS clean: a
                # position-mode close with a bounded goal_time is monotone at
                # ~25 mm/s and its loop pushes only ~9 N when blocked. So:
                # approach in position mode, detect the STALL (pads on the
                # object, or jaw shut), then hand over to the force hold.
                width0 = float(drv.get_gripper_position())
                drv.set_gripper_mode(self._ta.Mode.position)
                drv.set_gripper_position(0.0, max(1.0, width0 / GRIPPER_CLOSE_SPEED), False)
                t0, hist = time.time(), []
                while time.time() - t0 < 4.0:
                    time.sleep(0.02)
                    pos = float(drv.get_gripper_position())
                    hist.append(pos)
                    if pos < 0.0008:                      # shut on nothing
                        break
                    if len(hist) >= 10 and (hist[-10] - pos) < 0.0004 \
                            and (width0 - pos) > 0.002:   # moved, then stalled
                        break
                drv.set_gripper_mode(self._ta.Mode.external_effort)
                drv.set_gripper_external_effort(float(effort), 0.3, True)
                time.sleep(0.3)
            else:
                # Legacy force-controlled close: ramp the effort from rest.
                drv.set_gripper_mode(self._ta.Mode.external_effort)
                drv.set_gripper_external_effort(float(effort), 1.5, True)
                time.sleep(0.6)  # let the fingers settle on the object
        else:
            drv.set_gripper_mode(self._ta.Mode.position)
            drv.set_gripper_position(min(width_m / GRIPPER_APERTURE_RATIO, GRIPPER_DRIVER_MAX), 1.0, True)

    def get_gripper(self, arm: str) -> dict[str, float]:
        drv = self._drivers[arm]
        width = float(drv.get_gripper_position()) * GRIPPER_APERTURE_RATIO
        efforts = np.asarray(drv.get_all_external_efforts(), dtype=float)
        out = {"width_m": width, "effort": float(abs(efforts[-1]))}
        out["commanded_m"] = float(self._gripper_cmd.get(arm, -1.0))
        return out

    # -- recording tap --------------------------------------------------------
    # Sampled from the recorder's background thread while a blocking motion
    # call occupies the caller's: the driver getters read the controller's own
    # feedback stream (the teleop stack samples them the same way at rate).
    def joint_state(self, arm: str) -> np.ndarray:
        """Measured joints (6) + gripper aperture (m)."""
        drv = self._drivers[arm]
        q = np.asarray(drv.get_all_positions(), dtype=float)
        width = float(drv.get_gripper_position()) * GRIPPER_APERTURE_RATIO
        return np.append(q[:6], width)

    def joint_command(self, arm: str) -> np.ndarray:
        """Last commanded joint targets (6) + commanded gripper aperture (m).
        Before the first command, the measured state stands in — an action
        stream that starts with "hold where you are" is true."""
        state = None
        q = self._last_joint_cmd.get(arm)
        if q is None:
            state = self.joint_state(arm)
            q = state[:6]
        grip = self._gripper_cmd.get(arm)
        if grip is None:
            grip = float((state if state is not None else self.joint_state(arm))[6])
        return np.append(np.asarray(q, dtype=float), grip)

    def felt_descend(self, arm: str, xyz_floor: np.ndarray, speed_mps: float,
                     threshold_n: float) -> tuple[float, bool]:
        """Constant-velocity descent to xyz_floor (world), halting on contact.

        One non-blocking joint command spans the whole descent; external effort
        is polled at ~12 Hz (a UDP state read, milliseconds) and contact halts
        the motion by re-commanding the CURRENT joint positions. Replaces the
        stepped probe, whose cost was the per-step blocking move — ~3.5 s a
        step on this rig — not the sensor.
        """
        drv = self._drivers[arm]
        rvec = self._hold_orientation[arm]
        target = self._world_to_arm(arm, np.asarray(xyz_floor, dtype=float))
        start_w = self.get_cartesian(arm)
        dist = float(abs(start_w[2] - float(xyz_floor[2])))
        if dist < 1e-4:
            return float(start_w[2]), False
        q_now = np.asarray(drv.get_all_positions(), dtype=float)[:self._kin.dof]
        q = self._kin.ik(target, rvec, q_seed=q_now)
        if q is None:
            raise RuntimeError(f"felt_descend: no IK at {np.round(target, 3).tolist()}")
        goal_time = max(dist / max(speed_mps, 1e-3), 0.4)
        baseline = self.get_external_effort(arm)
        self._last_joint_cmd[arm] = np.asarray(q, dtype=float)
        drv.set_arm_positions(q, goal_time, False)
        t0 = time.time()
        touched = False
        while time.time() - t0 < goal_time + 0.2:
            time.sleep(0.08)
            if self.get_external_effort(arm) - baseline >= threshold_n:
                hold = np.asarray(drv.get_all_positions(), dtype=float)[:self._kin.dof]
                drv.set_arm_positions(hold, 0.15, True)
                self._last_joint_cmd[arm] = hold
                touched = True
                break
        z_stop = float(self.get_cartesian(arm)[2])
        self._last_target[arm] = self._world_to_arm(arm, self.get_cartesian(arm))
        return z_stop, touched

    def get_external_effort(self, arm: str) -> float:
        """Arm-joint external effort magnitude (gripper joint excluded) — the
        driver already compensates gravity and friction, so this is contact."""
        efforts = np.asarray(self._drivers[arm].get_all_external_efforts(), dtype=float)
        return float(np.linalg.norm(efforts[:-1]))

    def capture(self, camera: str) -> Frame:
        cam_cfg = self.cfg.cameras.get(camera)
        if camera in self._depth_clients:
            frame = self._depth_clients[camera].capture(camera)
            # A WRIST camera's extrinsics move with the arm, so a fixed matrix
            # from a file would place its depth wherever the arm happened to be
            # during calibration. Compose the live pose instead. The depth
            # sidecar knows nothing about this — it only serves pixels.
            t = self._wrist_extrinsics(camera, cam_cfg)
            if t is not None:
                frame.t_base_cam = t
            return frame
        if cam_cfg is not None and cam_cfg.index is None:
            t = self._wrist_extrinsics(camera, cam_cfg)
            if t is not None:
                return self._rig.capture(camera, t_base_cam=t)
        return self._rig.capture(camera)

    def _wrist_extrinsics(self, camera: str, cam_cfg) -> np.ndarray | None:
        """T_world_cam for a wrist camera at the arm's CURRENT pose, or None.

        T_world_cam = T_world_base @ T_base_ee @ T_ee_cam, read fresh every
        capture because the middle term is wherever the arm is standing.
        """
        if cam_cfg is None or cam_cfg.kind != "wrist" or cam_cfg.arm not in self._drivers:
            return None
        t_ee_cam = getattr(cam_cfg, "t_ee_cam", None)
        if t_ee_cam is None:
            return None
        side = cam_cfg.arm
        pose = np.asarray(self._drivers[side].get_cartesian_positions(), dtype=float)
        t_base_ee = np.eye(4)
        t_base_ee[:3, :3] = _rodrigues(pose[3:6])
        t_base_ee[:3, 3] = pose[:3]
        return self._t_world_base[side] @ t_base_ee @ np.asarray(t_ee_cam, dtype=float)

    def stop(self) -> None:
        for drv in self._drivers.values():
            try:
                drv.set_all_modes(self._ta.Mode.idle)  # idle = braked, holds position
            except Exception:
                pass

    def shutdown(self) -> None:
        self._rig.close()
        for drv in self._drivers.values():
            try:
                drv.cleanup()
            except Exception:
                pass

    # -- maintenance ---------------------------------------------------------
    def clear_error(self, arm: str) -> None:
        cfg = self.cfg.arms[arm]
        drv = self._drivers[arm]
        drv.configure(self._ta.Model.wxai_v0, self._ta.StandardEndEffector.wxai_v0_follower, cfg.ip, True)
        drv.set_all_modes(self._ta.Mode.position)
