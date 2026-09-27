"""MuJoCo backend for the Trossen cell — the same arm, the same solver.

This exists to answer one question before the hardware is touched: would this
plan work? It can only answer it if the simulation refuses what the hardware
refuses, so the parts that decide that are shared rather than reimplemented:

  * the arm is assets/wxai_follower.urdf, whose kinematics the real controller
    agrees with to 0.06 mm, and MuJoCo's own FK agrees with ours to 0.04 mm;
  * motions are solved by heron.robot.kinematics, the SAME module the real
    backend uses, so an unreachable hover is unreachable in both;
  * table height and camera poses come from the rig's calibration files.

What it does NOT reproduce, and must not be trusted for: depth noise (MuJoCo's
depth is exact, the D405 scatters 5.7 mm on wood and 25 mm on gloss), the
gripper's force behaviour, and anything about the driver — the "joint 6 is in
external_effort mode" failure that stopped a real run has no analogue here.
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np

from ..config import HeronConfig
from ..types import Frame
from .kinematics import Kinematics, rodrigues, rotvec
from .trossen import is_vertical_move

DEFAULT_SCENE = Path(__file__).resolve().parents[2] / "assets" / "trossen_scene.xml"
HOME_JOINTS = np.array([0.0, np.pi / 2, np.pi / 2, 0.0, 0.0, 0.0])
TRAVEL_CLEARANCE_M = 0.15
APPROACH_TILT_FALLBACKS_RAD = tuple(np.radians([0, 15, 30, 45, 60]))
GRIPPER_MAX_M = 0.044          # carriage travel, per the URDF
GRIPPER_APERTURE_RATIO = 2.0   # two carriages -> aperture
SETTLE_STEPS = 400             # how long the world is given to stop moving
CONTACT_FORCE_SCALE = 1.0


def _tilt_toward_base(rvec: np.ndarray, target: np.ndarray, tilt: float) -> np.ndarray:
    """Lean an approach back toward the base — identical to the real backend."""
    if abs(tilt) < 1e-9:
        return np.asarray(rvec, dtype=float)
    horiz = np.asarray(target, dtype=float)[:2]
    n = float(np.linalg.norm(horiz))
    if n < 1e-6:
        return np.asarray(rvec, dtype=float)
    d = horiz / n
    axis = np.array([-d[1], d[0], 0.0])
    return rotvec(rodrigues(axis * tilt) @ rodrigues(np.asarray(rvec, dtype=float)))


def _write_frame_times(film_path, times, n_frames: int) -> None:
    """Save a film's per-frame timestamps next to it, if we have one per frame."""
    import json  # noqa: PLC0415

    if not times or len(times) < n_frames:
        return
    Path(film_path).with_name("frame_times.json").write_text(
        json.dumps([round(float(t), 3) for t in times[:n_frames]]))


class TrossenSim:
    """Drop-in for TrossenStationary, backed by MuJoCo instead of a controller."""

    def __init__(self, cfg: HeronConfig, scene: str | Path = DEFAULT_SCENE,
                 record_camera: str | None = "cam_high", record_every: int = 40) -> None:
        try:
            import mujoco  # noqa: PLC0415
        except ImportError as e:
            raise RuntimeError("mujoco is not installed (uv pip install mujoco)") from e
        self._mj = mujoco
        self.cfg = cfg
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self._kin = Kinematics()
        self.arms = [a for a in cfg.arms] or ["right"]
        self.cameras = [c for c in cfg.cameras
                        if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, c) >= 0]
        self._renderers: dict[tuple[int, int], Any] = {}
        self._hold_orientation = {
            side: np.asarray(cfg.arms[side].approach_rvec, dtype=float)
            for side in self.arms if side in cfg.arms}
        self._last_tilt = {side: 0.0 for side in self.arms}
        # Last pose COMMANDED, per arm. See is_vertical_move for why the
        # commanded pose and not the measured one.
        self._last_target: dict[str, np.ndarray] = {}
        # ONE ARM WAS AN ASSUMPTION, NOT A DESIGN. Joints were addressed by
        # index from zero and the tool was the body called "link_6"; a scene
        # with a second arm has both of those twice. Each arm now carries its
        # own qpos/ctrl indices and its own tool body, looked up by name once.
        # The single-arm scene keeps working because "right" resolves to the
        # unprefixed names it already had.
        self._prefix = {side: ("" if side == "right" else f"{side}_")
                        for side in self.arms}
        self._jnt: dict[str, list[int]] = {}
        self._act: dict[str, list[int]] = {}
        self._tool: dict[str, int] = {}
        self._carriage: dict[str, list[int]] = {}
        for side in self.arms:
            pre = self._prefix[side]
            js, acts = [], []
            for i in range(self._kin.dof):
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{pre}joint_{i}")
                aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{pre}a_joint_{i}")
                if jid < 0 or aid < 0:
                    js, acts = [], []
                    break
                js.append(jid); acts.append(aid)
            if not js:
                continue
            self._jnt[side], self._act[side] = js, acts
            self._tool[side] = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                                 f"{pre}link_6")
            self._carriage[side] = [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                  f"{pre}a_{which}_carriage_joint")
                for which in ("right", "left")]
        if not self._jnt:
            raise SystemExit("no arm joints found in the scene")
        self._link6 = self._tool.get("right", next(iter(self._tool.values())))
        # World -> that arm's own base, from the config. The right arm is the
        # world origin on this rig, so its transform is the identity and the
        # single-arm path is unchanged.
        self._t_world_base = {
            side: np.asarray(cfg.arms[side].t_world_base, dtype=float)
            for side in self._jnt if side in cfg.arms}
        self._grip_cmd = {side: GRIPPER_MAX_M for side in self.arms}
        # Recording. A failure in simulation is only cheaper than one on the
        # hardware if you can see what happened; a journal says the place
        # "succeeded" whether the block landed on the plate or beside it.
        self.frames: list[np.ndarray] = []
        # When each frame was taken, in seconds since the recorder started.
        # The journal timestamps every event the same way, so a film and a
        # sequence of tool calls can be put on one timeline instead of being
        # two things a reader has to correlate by eye.
        self.frame_times: list[float] = []
        self._recording_t0 = time.time()
        self.record_camera = record_camera
        self.record_every = int(record_every)
        self._step_count = 0
        # Called after every physics step. The LeRobot recorder rides this so
        # its samples land on the simulation's own clock (and on the thread
        # that owns MjData) rather than on wall time.
        self.step_listeners: list = []
        self.set_joints(HOME_JOINTS)

    # -- state ---------------------------------------------------------------
    def set_joints(self, q, arm: str | None = None) -> None:
        """Place an arm without simulating the journey. For setup only."""
        q = np.asarray(q, dtype=float)
        side = arm or next(iter(self._jnt))
        for k, jid in enumerate(self._jnt[side]):
            self.data.qpos[self.model.jnt_qposadr[jid]] = q[k]
            self.data.ctrl[self._act[side][k]] = q[k]
        self._mj.mj_forward(self.model, self.data)

    def joints(self, arm: str | None = None) -> np.ndarray:
        side = arm or next(iter(self._jnt))
        return np.array([self.data.qpos[self.model.jnt_qposadr[j]]
                         for j in self._jnt[side]])

    def get_cartesian(self, arm: str) -> np.ndarray:
        # FK is in the arm's OWN frame; the caller works in the world.
        local = self._kin.fk(self.joints(arm))[:3, 3]
        t = self._t_world_base.get(arm)
        if t is None:
            return local
        return (t @ np.append(local, 1.0))[:3]

    def arm_polyline(self, arm: str) -> np.ndarray:
        """The arm's skeleton (joint origins + tool), world frame."""
        pts = self._kin.link_points(self.joints(arm))
        t = self._t_world_base.get(arm)
        if t is None:
            return pts
        return np.array([(t @ np.append(p, 1.0))[:3] for p in pts])

    def reachable(self, arm: str, xyz) -> bool:
        """Whether the tool can hold its approach orientation there.

        The real backend answers this with the same solver, so a target refused
        in simulation is refused on the hardware and the other way round. That
        equivalence is the whole point of the simulation.
        """
        target = np.asarray(xyz, dtype=float)
        # In the ARM's frame, seeded from THAT arm — the world frame and the
        # first arm's joints were fine while there was only one arm, and wrong
        # the moment there were two: the far arm called every point reachable,
        # including points 0.7 m outside its own reach, so choose_arm was
        # picking between two answers to the same question.
        t = self._t_world_base.get(arm)
        if t is not None:
            target = (np.linalg.inv(t) @ np.append(target[:3], 1.0))[:3]
        rvec = self._hold_orientation.get(arm, np.array([0.0, np.pi / 2, 0.0]))
        q_now = self.joints(arm)
        return any(self._kin.ik(target, _tilt_toward_base(rvec, target, t_),
                                q_seed=q_now, seeds=8) is not None
                   for t_ in APPROACH_TILT_FALLBACKS_RAD)

    # -- motion --------------------------------------------------------------
    def _step(self) -> None:
        self._mj.mj_step(self.model, self.data)
        self._step_count += 1
        if self.record_camera and self._step_count % self.record_every == 0:
            self.frames.append(self._render(self.record_camera, 320, 240))
            self.frame_times.append(round(time.time() - self._recording_t0, 3))
        for listener in self.step_listeners:
            listener()

    def _drive_to(self, q_goal: np.ndarray, seconds: float, arm: str) -> None:
        """Interpolate one arm's position targets and let the physics follow.

        Only this arm's actuators are written: the other arm holds whatever it
        was last commanded, which is what "the other agent is not moving right
        now" has to mean in a shared simulation.
        """
        q0 = self.joints(arm)
        acts = self._act[arm]
        steps = max(2, int(seconds / self.model.opt.timestep))
        for i in range(1, steps + 1):
            q = q0 + (q_goal - q0) * (i / steps)
            for k, a in enumerate(acts):
                self.data.ctrl[a] = q[k]
            self._step()
        for _ in range(SETTLE_STEPS // 4):
            self._step()

    def _solve(self, arm: str, p: np.ndarray):
        rvec = self._hold_orientation.get(arm, np.array([0.0, np.pi / 2, 0.0]))
        # IK is solved in the arm's own frame; the target arrives in the world.
        t = self._t_world_base.get(arm)
        if t is not None:
            p = (np.linalg.inv(t) @ np.append(np.asarray(p, dtype=float)[:3], 1.0))[:3]
        q_now = self.joints(arm)
        for tilt in APPROACH_TILT_FALLBACKS_RAD:
            q = self._kin.ik(p, _tilt_toward_base(rvec, p, tilt), q_seed=q_now)
            if q is not None:
                return q, tilt
        return None, None

    def move_cartesian(self, arm: str, xyz: np.ndarray, seconds: float = 2.0,
                       orientation: np.ndarray | None = None, staged: bool = True) -> None:
        target = np.asarray(xyz, dtype=float)
        previous = self._last_target.get(arm)
        self._last_target[arm] = target.copy()
        if orientation is not None:
            self._hold_orientation[arm] = np.asarray(orientation, dtype=float)
        scale = max(float(self.cfg.safety.speed_scale), 1e-3)

        def go(p, secs):
            q, tilt = self._solve(arm, p)
            if q is None:
                raise RuntimeError(
                    f"no IK solution at {np.round(p, 3).tolist()} for any approach "
                    f"from 0 to {np.degrees(APPROACH_TILT_FALLBACKS_RAD[-1]):.0f} "
                    f"degrees off vertical")
            self._last_tilt[arm] = float(tilt)
            self._drive_to(q, float(secs) / scale, arm)

        if not staged:
            go(target, seconds)
            return
        # Nothing to route around when the move is vertical — and staging it
        # lifts the tool out of its own descent. See the real backend for the
        # measurement; this is the same rule so the two agree.
        if is_vertical_move(previous, target):
            go(target, seconds)
            return
        want = float(self.cfg.table_z) + TRAVEL_CLEARANCE_M
        travel_z = None
        for z in np.arange(want, target[2] + 0.02, -0.02):
            if self._solve(arm, np.array([target[0], target[1], z]))[0] is not None:
                travel_z = float(z)
                break
        if travel_z is None or target[2] >= travel_z - 0.005:
            go(target, seconds)
            return
        go(np.array([target[0], target[1], travel_z]), seconds)
        go(target, max(seconds * 0.5, 1.0))

    def home(self, arm: str) -> None:
        self._drive_to(HOME_JOINTS, 3.0 / max(float(self.cfg.safety.speed_scale), 1e-3), arm)
        # Homing moves in joint space, so there is no commanded cartesian pose
        # any more and the next move must be staged like any other reposition.
        self._last_target.pop(arm, None)

    def settle(self, seconds: float = 1.0) -> None:
        for _ in range(int(max(0.0, seconds) / self.model.opt.timestep)):
            self._step()

    # -- gripper -------------------------------------------------------------
    def set_gripper(self, arm: str, width_m: float, effort_limit: float | None = None) -> None:
        carriage = float(np.clip(width_m / GRIPPER_APERTURE_RATIO, 0.0, GRIPPER_MAX_M))
        self._grip_cmd[arm] = carriage
        for _ in range(300):
            for a in self._carriage[arm]:
                self.data.ctrl[a] = carriage
            self._step()

    def get_gripper(self, arm: str) -> dict[str, float]:
        pre = self._prefix[arm]
        jid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_JOINT,
                                  f"{pre}right_carriage_joint")
        pos = float(self.data.qpos[self.model.jnt_qposadr[jid]])
        # Squeeze: how far the jaws are from where they were told to go. On the
        # real arm this is a force reading; here the stall against an object is
        # what stands in for it.
        stall = max(0.0, pos - self._grip_cmd.get(arm, GRIPPER_MAX_M))
        return {"width_m": pos * GRIPPER_APERTURE_RATIO,
                "effort": float(stall * 4000.0 * CONTACT_FORCE_SCALE)}

    # -- recording tap --------------------------------------------------------
    def joint_state(self, arm: str) -> np.ndarray:
        """Measured joints (6) + gripper aperture (m) — what a dataset calls
        observation.state."""
        return np.append(self.joints(arm), self.get_gripper(arm)["width_m"])

    def joint_command(self, arm: str) -> np.ndarray:
        """The joint targets the physics is currently driving toward (6) +
        commanded gripper aperture (m). ctrl IS the interpolated command, so
        this is the same signal the real controller would be tracking."""
        cmd = np.asarray([self.data.ctrl[a] for a in self._act[arm]], dtype=float)
        return np.append(cmd, self._grip_cmd.get(arm, GRIPPER_MAX_M) * GRIPPER_APERTURE_RATIO)

    def apply_joint_command(self, arm: str, q, seconds: float = 1.0 / 30) -> None:
        """Drive one arm to a joint command and let the physics run for a tick.

        The exact inverse of `joint_command`, and deliberately so: a policy
        trained on what that method recorded has to be replayed through the
        same signal, or the rollout is being asked to control a robot it never
        saw. Everything above this (IK, staged travel, contact descent) is the
        agent's way of producing commands; a learned policy produces them
        directly, and this is where they land.
        """
        q = np.asarray(q, dtype=float)
        for k, a in enumerate(self._act[arm][:6]):
            # ctrlrange IS ONLY A RANGE WHEN ctrllimited SAYS SO. On these six
            # actuators it is [0, 0] and the flag is off, so clipping to it
            # commanded every joint to zero — the arm let go of whatever it was
            # holding and swung to the zero pose, on every tick, in every
            # rollout. Both round-0 policies scored 0/20 because of this line
            # and not because of anything they had learned. The joint's own
            # limits are the bound that means something.
            lo, hi = self.model.actuator_ctrlrange[a]
            if not self.model.actuator_ctrllimited[a]:
                j = self._jnt[arm][k]
                lo, hi = ((self.model.jnt_range[j][0], self.model.jnt_range[j][1])
                          if self.model.jnt_limited[j] else (-np.inf, np.inf))
            self.data.ctrl[a] = float(np.clip(q[k], lo, hi))
        if q.shape[0] > 6:
            # The seventh number is an APERTURE, the same units joint_command
            # reports and set_gripper accepts. Written straight to the carriage
            # targets rather than through set_gripper, which runs 300 settling
            # steps of its own — ten times a rollout tick, so a policy driving
            # through it would advance the world ten seconds per second.
            carriage = float(np.clip(q[6] / GRIPPER_APERTURE_RATIO, 0.0, GRIPPER_MAX_M))
            self._grip_cmd[arm] = carriage
            for a in self._carriage[arm]:
                self.data.ctrl[a] = carriage
        for _ in range(max(1, int(round(seconds / self.model.opt.timestep)))):
            self._step()

    def get_external_effort(self, arm: str) -> float:
        """Magnitude of the contact wrench on the arm, standing in for the
        driver's gravity-compensated external effort."""
        force = np.zeros(6)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            b1 = self.model.geom_bodyid[c.geom1]
            b2 = self.model.geom_bodyid[c.geom2]
            if b1 == 0 and b2 == 0:
                continue
            f = np.zeros(6)
            self._mj.mj_contactForce(self.model, self.data, i, f)
            force += np.abs(f)
        return float(np.linalg.norm(force[:3]))

    # -- cameras -------------------------------------------------------------
    def _render(self, camera: str, w: int, h: int) -> np.ndarray:
        key = (w, h)
        if key not in self._renderers:
            self._renderers[key] = self._mj.Renderer(self.model, height=h, width=w)
        r = self._renderers[key]
        r.update_scene(self.data, camera=camera)
        return r.render().copy()

    def render_rgb(self, camera: str, w: int, h: int) -> np.ndarray:
        """One RGB frame at an arbitrary size — the recorder's camera path.
        Unlike capture() this skips the depth pass, which recording never needs
        (recording stays RGB by the standing rule)."""
        return self._render(camera, w, h)

    def save_video(self, path, fps: int = 20, width: int = 320) -> bool:
        """Write what the run looked like. False when nothing was recorded."""
        if not self.frames:
            return False
        import imageio.v2 as imageio  # noqa: PLC0415

        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        # `duration` and `loop` are GIF words. Handed to the video plugin they
        # are not ignored, they are a TypeError — after the run, with the
        # frames still in memory and nowhere to go.
        if out.suffix.lower() == ".gif":
            imageio.mimsave(str(out), self.frames, duration=1.0 / max(fps, 1), loop=0)
        else:
            imageio.mimsave(str(out), self.frames, fps=max(fps, 1))
        # The times go beside the film, not into it. A GIF carries no clock a
        # reader can use, and the journal's events already share this one — so
        # writing them here is what lets a player put the two on one timeline
        # instead of leaving a reader to correlate them by eye.
        _write_frame_times(out, self.frame_times, len(self.frames))
        return True

    # GIF under the name the LIBERO backend uses, so the CLI needs no special case.
    save_gif = save_video

    def camera_calibrated(self, camera: str) -> bool:
        """Every camera in the scene is posed, because the model poses it.

        The rig answers this question with a calibration file; the twin answers
        it with the model, and asking it directly is what lets a viewpoint the
        rig has not calibrated yet be exercised here first.
        """
        return camera in self.cameras

    def capture(self, camera: str) -> Frame:
        if camera not in self.cameras:
            raise RuntimeError(f"camera {camera!r} is not in the scene "
                               f"(have: {self.cameras})")
        w, h = 640, 480
        key = (w, h)
        if key not in self._renderers:
            self._renderers[key] = self._mj.Renderer(self.model, height=h, width=w)
        r = self._renderers[key]
        r.update_scene(self.data, camera=camera)
        rgb = r.render().copy()
        r.enable_depth_rendering()
        r.update_scene(self.data, camera=camera)
        depth = r.render().copy().astype(np.float32)
        r.disable_depth_rendering()
        # MuJoCo returns the distance to the far plane where nothing was hit.
        depth[depth >= depth.max() - 1e-3] = np.nan

        cam_id = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_CAMERA, camera)
        fovy = float(self.model.cam_fovy[cam_id])
        f = 0.5 * h / np.tan(np.radians(fovy) / 2.0)
        intr = np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1.0]])
        # The camera's live pose, so a wrist camera moves with the arm exactly
        # as it does on the rig.
        t = np.eye(4)
        t[:3, :3] = self.data.cam_xmat[cam_id].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        t[:3, 3] = self.data.cam_xpos[cam_id]
        return Frame(camera=camera, rgb=rgb, depth=depth, intrinsics=intr,
                     t_base_cam=t, t=time.time())

    # -- ground truth, for scoring a run -------------------------------------
    def place_body(self, name: str, xyz, settle_steps: int = SETTLE_STEPS) -> None:
        """Teleport a free-standing object and let it settle.

        For setting a scene BETWEEN episodes, not during one — the arm is not
        moved and nothing is verified, so calling this mid-episode would make
        the belief store quietly wrong. Every object in the twin has a free
        joint, so this writes the joint's qpos rather than the body's pose,
        which is the only one of the two the physics reads back.
        """
        bid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise KeyError(f"no body {name!r} in the scene")
        jnt = int(self.model.body_jntadr[bid])
        if jnt < 0 or self.model.jnt_type[jnt] != self._mj.mjtJoint.mjJNT_FREE:
            raise ValueError(f"{name!r} has no free joint; it cannot be moved")
        adr = int(self.model.jnt_qposadr[jnt])
        self.data.qpos[adr:adr + 3] = np.asarray(xyz, dtype=float)
        self.data.qpos[adr + 3:adr + 7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qvel[int(self.model.jnt_dofadr[jnt]):
                       int(self.model.jnt_dofadr[jnt]) + 6] = 0.0
        self._mj.mj_forward(self.model, self.data)
        for _ in range(int(settle_steps)):
            self._mj.mj_step(self.model, self.data)

    def body_xyz(self, name: str) -> np.ndarray:
        bid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise KeyError(f"no body {name!r} in the scene")
        return np.array(self.data.xpos[bid])

    def set_body_xyz(self, name: str, xyz) -> None:
        """Teleport a free body. For resetting between trials, not for acting."""
        bid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise KeyError(f"no body {name!r} in the scene")
        adr = self.model.body_jntadr[bid]
        if adr < 0 or self.model.jnt_type[adr] != self._mj.mjtJoint.mjJNT_FREE:
            raise ValueError(f"{name!r} is bolted down; it has no free joint to move")
        q = self.model.jnt_qposadr[adr]
        self.data.qpos[q:q + 3] = np.asarray(xyz, dtype=float)
        self.data.qpos[q + 3:q + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[self.model.jnt_dofadr[adr]:self.model.jnt_dofadr[adr] + 6] = 0.0
        self._mj.mj_forward(self.model, self.data)

    def stop(self) -> None:
        pass

    def shutdown(self) -> None:
        for r in self._renderers.values():
            try:
                r.close()
            except Exception:
                pass
        self._renderers.clear()
