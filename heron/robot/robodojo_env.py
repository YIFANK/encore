"""RoboDojo (Isaac Sim, ARX X5 bimanual) backend for the fair harness.

The RoboDojo eval client owns the simulator and drives a *policy server* over
WebSocket: per control step it sends one observation (``update_obs``) and asks
for an action chunk (``get_action``).  This module turns that push loop into
the pull-style fair API our policy programs speak (``capture`` / ``move`` /
``grip`` / ...): the program runs in its own sandboxed process and talks to
:class:`RoboDojoRobot` over the fair socket; every ``move`` becomes a queue of
absolute end-effector targets that the policy server hands back to the
simulator one control step (25 Hz, ten physics substeps) at a time.

Frames: RoboDojo reports both end-effector poses and camera extrinsics in the
world frame of the single environment, so the fair API's "base" frame IS the
world frame here (both arms share it).  Poses are ``[x, y, z, qw, qx, qy, qz]``.

Nothing in this file reads the benchmark success bit: the eval client scores
the episode after it ends and ``tools/fair_run_robodojo.py`` reads
``_result.json`` post hoc, exactly as the LIBERO harness reads its predicate
only after ``run()`` returns.
"""
from __future__ import annotations

import queue
import threading
import time
from typing import Optional

import numpy as np

from heron.types import Frame

CAMS = ("cam_head", "cam_left_wrist", "cam_right_wrist")
ARMS = ("left", "right")
CONTROL_HZ = 25.0
# X5 gripper: two mimic fingers, 0.044 m of travel each at full open (URDF
# init joint7/8 = 0.044).  Width reported to programs = both fingers.
GRIPPER_MAX_WIDTH_M = 0.088


def _quat_to_mat(q):
    w, x, y, z = [float(v) for v in q]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def _mat_to_quat(m):
    m = np.asarray(m, float)
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([w, x, y, z], float)
    return q / np.linalg.norm(q)


def _slerp(q0, q1, a):
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + a * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(d)
    return (np.sin((1 - a) * th) * q0 + np.sin(a * th) * q1) / np.sin(th)


class EpisodeAborted(RuntimeError):
    pass


class RoboDojoRobot:
    """One episode's view of the simulator, served to the program over the
    fair socket by ``fair_run.serve_episode``.  Lives inside the policy-server
    process; :class:`RoboDojoBridge` feeds it observations and drains its
    action queue."""

    cameras = dict.fromkeys(CAMS)
    arms = list(ARMS)

    def __init__(self, bridge: "RoboDojoBridge", episode: int, film_every: int = 5,
                 model_call_budget: int = 60):
        self.bridge = bridge
        self.episode = episode
        self.arm = "right"
        self.task_success = False
        self.terminated = False
        self.sim_steps = 0          # control steps consumed by this episode
        self.last_move_residual = float("nan")
        self.program_done = False    # set by the runner when the program process has exited
        self._film: list[np.ndarray] = []
        self._film_every = int(film_every)
        self._model_calls = 0
        self._model_call_budget = int(model_call_budget)
        self._orch = None

    # -- observation access ----------------------------------------------
    @property
    def task_language(self) -> str:
        obs = self.bridge.wait_obs(timeout=120.0)
        return str(obs.get("instruction", "") or "")

    @property
    def obs(self) -> dict:
        o = self.bridge.latest_obs()
        st = o.get("state", {})
        out = {}
        for side in ARMS:
            j = st.get(f"{side}_arm_joint_state")
            if j is not None:
                out[f"{side}_arm_joint_pos"] = [float(v) for v in np.atleast_1d(j)]
            p = st.get(f"{side}_ee_pose")
            if p is not None:
                out[f"{side}_eef_pos"] = [float(v) for v in np.asarray(p)[:3]]
                out[f"{side}_eef_quat_wxyz"] = [float(v) for v in np.asarray(p)[3:7]]
            g = st.get(f"{side}_gripper_joint_state")
            if g is not None:
                out[f"{side}_gripper_qpos"] = [float(v) for v in np.atleast_1d(g)]
        # legacy single-arm keys the proprio op filters on
        d = self.arm
        out["robot0_joint_pos"] = out.get(f"{d}_arm_joint_pos", [])
        out["robot0_eef_pos"] = out.get(f"{d}_eef_pos", [])
        out["robot0_eef_quat"] = out.get(f"{d}_eef_quat_wxyz", [])
        out["robot0_gripper_qpos"] = out.get(f"{d}_gripper_qpos", [])
        return out

    def _side(self, arm) -> str:
        if arm is None or arm == "arm":
            return self.arm
        if arm not in ARMS:
            raise ValueError(f"arm {arm!r} not in {ARMS}")
        return arm

    def _pose(self, side: str) -> np.ndarray:
        st = self.bridge.latest_obs().get("state", {})
        p = st.get(f"{side}_ee_pose")
        if p is None:
            raise RuntimeError(f"no {side}_ee_pose in observation (enable world_ee_state)")
        return np.asarray(p, float).reshape(7)

    def capture(self, camera: str) -> Frame:
        o = self.bridge.latest_obs()
        v = o.get("vision", {}).get(camera)
        if v is None:
            raise ValueError(f"unknown camera {camera!r} (have {list(o.get('vision', {}))})")
        rgb = np.asarray(v["color"])[:, :, :3].astype(np.uint8)
        depth = v.get("depth")
        if depth is None and v.get("approximate_depth") is not None:
            depth = np.asarray(v["approximate_depth"], np.float32) / 1000.0
        depth = None if depth is None else np.asarray(depth, np.float32).reshape(rgb.shape[:2])
        K = v.get("intrinsic_matrix")
        T = v.get("extrinsic_matrix")
        f = Frame(camera=camera, rgb=rgb, depth=depth,
                  intrinsics=None if K is None else np.asarray(K, float).reshape(3, 3),
                  t_base_cam=None if T is None else np.asarray(T, float).reshape(4, 4))
        return f

    # -- proprioception ----------------------------------------------------
    def get_cartesian(self, arm=None) -> np.ndarray:
        return self._pose(self._side(arm))[:3].copy()

    def get_tool_rotation(self, arm=None) -> np.ndarray:
        return _quat_to_mat(self._pose(self._side(arm))[3:7])

    def get_gripper(self, arm=None) -> dict:
        side = self._side(arm)
        st = self.bridge.latest_obs().get("state", {})
        g = st.get(f"{side}_gripper_joint_state")
        cmd = st.get(f"{side}_ee_joint_state")
        cmd_open = None if cmd is None else float(np.atleast_1d(cmd)[0])
        if g is not None:
            j = float(np.atleast_1d(g)[0])
            width = float(np.clip(2.0 * j, 0.0, GRIPPER_MAX_WIDTH_M))
        elif cmd_open is not None:
            width = float(cmd_open * GRIPPER_MAX_WIDTH_M)
        else:
            width = float("nan")
        # Effort proxy: commanded closed but the fingers stopped well short of
        # closed -> something is between them.  Mirrors the LIBERO semantics
        # ("effort 3.0 iff holding") without a force sensor.
        holding = (cmd_open is not None and g is not None and cmd_open < 0.3
                   and width > 0.006)
        return {"width_m": round(width, 4), "effort": 3.0 if holding else 0.05,
                "commanded_open": cmd_open}

    # -- motion --------------------------------------------------------------
    def _action(self, targets: dict[str, np.ndarray], grips: dict[str, float]) -> dict:
        a = {}
        for side in ARMS:
            p = targets.get(side)
            if p is None:
                p = self._pose(side)
            a[f"{side}_ee_pose"] = [float(v) for v in np.asarray(p, float).reshape(7)]
            g = grips.get(side)
            if g is None:
                st = self.bridge.latest_obs().get("state", {})
                cmd = st.get(f"{side}_ee_joint_state")
                g = 1.0 if cmd is None else float(np.atleast_1d(cmd)[0])
            a[f"{side}_ee_joint_state"] = [float(np.clip(g, 0.0, 1.0))]
        return a

    def _run_chunk(self, chunk: list[dict]) -> None:
        if self.terminated:
            raise EpisodeAborted("episode already ended")
        self.bridge.push_chunk(chunk)
        self.bridge.wait_consumed(len(chunk), timeout=60.0 + 2.0 * len(chunk))
        self.sim_steps += len(chunk)

    def _line(self, side: str, xyz: np.ndarray, rot: Optional[np.ndarray], seconds: float,
              grip: Optional[float] = None) -> list[dict]:
        p0 = self._pose(side)
        q0 = p0[3:7]
        q1 = q0 if rot is None else _mat_to_quat(rot)
        x1 = np.asarray(xyz, float).reshape(3)
        dist = float(np.linalg.norm(x1 - p0[:3]))
        n = max(1, min(int(round(seconds * CONTROL_HZ)), int(np.ceil(dist / 0.015)) + 2))
        chunk = []
        for i in range(1, n + 1):
            a = i / n
            x = p0[:3] + a * (x1 - p0[:3])
            q = _slerp(q0, q1, a)
            chunk.append(self._action({side: np.concatenate([x, q])}, {} if grip is None else {side: grip}))
        # two hold steps so the joint interpolation actually lands
        chunk += [chunk[-1]] * 2
        return chunk

    def _residual(self, side: str, xyz) -> float:
        r = float(np.linalg.norm(self._pose(side)[:3] - np.asarray(xyz, float)))
        self.last_move_residual = round(r, 4)
        return r

    def move_cartesian(self, arm, xyz, seconds: float = 2.0) -> None:
        side = self._side(arm)
        self._run_chunk(self._line(side, xyz, None, seconds))
        self._residual(side, xyz)

    def move_pose(self, arm, xyz, rotation, seconds: float = 3.0) -> None:
        side = self._side(arm)
        self._run_chunk(self._line(side, xyz, np.asarray(rotation, float).reshape(3, 3), seconds))
        self._residual(side, xyz)

    def move_path(self, arm, points, rotation=None, seconds: float = 4.0) -> None:
        side = self._side(arm)
        pts = [np.asarray(p, float).reshape(3) for p in points]
        per = max(0.2, seconds / max(1, len(pts)))
        for p in pts:
            self._run_chunk(self._line(side, p, None if rotation is None else np.asarray(rotation, float), per))
        if pts:
            self._residual(side, pts[-1])

    def set_gripper(self, arm, width_m: float) -> None:
        side = self._side(arm)
        openness = float(np.clip(float(width_m) / GRIPPER_MAX_WIDTH_M, 0.0, 1.0))
        hold = self._pose(side)
        chunk = [self._action({side: hold}, {side: openness}) for _ in range(8)]
        self._run_chunk(chunk)

    def settle(self, seconds: float = 1.0) -> None:
        n = max(1, min(int(round(seconds * CONTROL_HZ)), 25))
        self._run_chunk([self._action({}, {}) for _ in range(n)])

    # -- open-vocabulary perception (coordinator side, results only) ----------
    def _count_model_call(self) -> None:
        self._model_calls += 1
        if self._model_calls > self._model_call_budget:
            raise RuntimeError(f"model-call budget exhausted ({self._model_call_budget})")

    def _orchestrator(self):
        if self._orch is None:
            from heron.cli import _build_orchestrator  # noqa: PLC0415
            from heron.config import HeronConfig  # noqa: PLC0415

            class _NullLog:
                # GeminiOrchestrator writes episode events and model-call
                # records to an EpisodeLogger; the bridge keeps no episode log.
                def event(self, *a, **k): return None
                def model_call(self, *a, **k): return None
                def __getattr__(self, name): return lambda *a, **k: None

            self._orch = _build_orchestrator("gemini", HeronConfig.load(self.bridge.heron_config), _NullLog())
        return self._orch

    @staticmethod
    def _opencv_frame(frame: Frame) -> Frame:
        # Isaac reports the camera pose in the USD/OpenGL convention (camera looks
        # along -z, +y up); Frame.deproject assumes OpenCV (+z forward, +y down).
        # The raw pose is what programs receive (documented in the brief); the
        # coordinator-side ground() converts before deprojecting.
        if frame.t_base_cam is None:
            return frame
        flip = np.diag([1.0, -1.0, -1.0, 1.0])
        return Frame(camera=frame.camera, rgb=frame.rgb, depth=frame.depth, intrinsics=frame.intrinsics,
                     t_base_cam=np.asarray(frame.t_base_cam, float) @ flip)

    def ground(self, query: str, camera: str = "cam_head"):
        self._count_model_call()
        frame = self.capture(camera)
        hit = self._orchestrator().point(frame, str(query))
        if hit is None:
            return None
        u, v = int(hit[0]), int(hit[1])
        xyz = self._opencv_frame(frame).deproject(u, v) if frame.depth is not None else None
        if xyz is None:
            return None
        return {"xyz": [round(float(x), 4) for x in np.asarray(xyz, float)], "px": [u, v]}

    def vqa(self, question: str, camera: str = "cam_head") -> dict:
        self._count_model_call()
        frame = self.capture(camera)
        value, conf, note = self._orchestrator().vqa([frame], str(question))
        return {"answer": str(value), "confidence": float(conf), "note": str(note)[:200]}

    # -- bookkeeping -----------------------------------------------------------
    def on_obs(self, obs: dict) -> None:
        if self._film_every and (self.sim_steps % self._film_every == 0):
            v = obs.get("vision", {}).get("cam_head")
            if v is not None:
                self._film.append(np.asarray(v["color"])[:, :, :3].copy())

    def save_gif(self, path) -> None:
        if not self._film:
            return
        from PIL import Image  # noqa: PLC0415
        frames = [Image.fromarray(f) for f in self._film[::2]]
        frames[0].save(str(path), save_all=True, append_images=frames[1:], duration=160, loop=0)

    def start_gt_trace(self, *_a, **_k) -> None:
        pass

    def shutdown(self) -> None:
        self.terminated = True


class RoboDojoBridge:
    """The XPolicyLab ``Model``: receives observations, hands out queued action
    chunks, and tells the current :class:`RoboDojoRobot` when an episode
    starts and ends.  Thread-safe: the policy server calls ``reset`` /
    ``update_obs`` / ``get_action`` from worker threads; the program-serving
    thread blocks in ``wait_obs`` / ``wait_consumed``."""

    HOLD_WAIT_S = 8.0      # get_action waits this long for a live program before holding one step

    def __init__(self, heron_config: str = "configs/libero.yaml"):
        self.heron_config = heron_config
        self._lock = threading.Condition()
        self._obs: Optional[dict] = None
        self._obs_count = 0
        self._consumed = 0
        self._q: "queue.Queue[list[dict]]" = queue.Queue()
        self._pending: list[dict] = []
        self.robot: Optional[RoboDojoRobot] = None
        self.episode_index = -1
        self.on_episode_start = None     # callback(episode_index) -> RoboDojoRobot
        self.on_episode_end = None       # callback(episode_index)
        self.log = print

    # -- XPolicyLab Model interface ----------------------------------------
    def reset(self):
        # RoboDojo calls reset twice per episode (EvalEnv.reset and the policy
        # deploy loop); a second reset with no observation in between is the
        # same episode, not a new one.
        if self.robot is not None and self._obs_count == 0:
            return {"ok": True, "note": "same episode"}
        self._end_episode()
        with self._lock:
            self._obs = None
            self._obs_count = 0
            self._consumed = 0
            self._pending = []
            while not self._q.empty():
                try:
                    self._q.get_nowait()
                except queue.Empty:
                    break
        self.episode_index += 1
        if self.on_episode_start is not None:
            self.robot = self.on_episode_start(self.episode_index)
        return {"ok": True}

    def update_obs(self, obs):
        with self._lock:
            self._obs = obs
            self._obs_count += 1
            self._lock.notify_all()
        if self.robot is not None:
            try:
                self.robot.on_obs(obs)
            except Exception:
                pass

    def update_obs_batch(self, obs_list):
        self.update_obs(obs_list[0])

    def get_action(self):
        if not self._pending:
            # Wait for the program only while it is still alive; once it has
            # returned (or never started) every remaining control step is a hold.
            alive = self.robot is not None and not self.robot.program_done and not self.robot.terminated
            try:
                self._pending = list(self._q.get(timeout=self.HOLD_WAIT_S if alive else 0.01))
            except queue.Empty:
                self._pending = []
        if self._pending:
            act = self._pending.pop(0)
            with self._lock:
                self._consumed += 1
                self._lock.notify_all()
            return [act]
        if self.robot is not None and self.robot.program_done:
            # Program returned: an empty chunk tells our deploy loop to end the
            # episode now (scored exactly as if the step budget had run out).
            return []
        return [self._hold_action()]

    def get_action_batch(self, env_idx_list=None):
        return [self.get_action()]

    def on_trial_end(self, result=None):
        self._end_episode()

    def prepare_case(self, case_meta=None):
        return None

    # -- program-thread side ---------------------------------------------------
    def latest_obs(self) -> dict:
        with self._lock:
            if self._obs is None:
                self._lock.wait_for(lambda: self._obs is not None, timeout=120.0)
            if self._obs is None:
                raise EpisodeAborted("no observation from the simulator")
            return self._obs

    def wait_obs(self, timeout: float = 60.0) -> dict:
        with self._lock:
            self._lock.wait_for(lambda: self._obs is not None, timeout=timeout)
            if self._obs is None:
                raise EpisodeAborted("no observation from the simulator")
            return self._obs

    def push_chunk(self, chunk: list[dict]) -> None:
        self._q.put(list(chunk))

    def wait_consumed(self, n: int, timeout: float) -> None:
        with self._lock:
            target = self._consumed + n
            ok = self._lock.wait_for(lambda: self._consumed >= target, timeout=timeout)
            if not ok:
                raise EpisodeAborted("simulator stopped consuming actions (episode over?)")
            # wait for the observation that follows the last consumed action
            c = self._obs_count
            self._lock.wait_for(lambda: self._obs_count > c, timeout=30.0)

    def _hold_action(self) -> dict:
        o = self._obs or {}
        st = o.get("state", {})
        a = {}
        for side in ARMS:
            p = st.get(f"{side}_ee_pose")
            g = st.get(f"{side}_ee_joint_state")
            if p is None:
                continue
            a[f"{side}_ee_pose"] = [float(v) for v in np.asarray(p, float).reshape(7)]
            a[f"{side}_ee_joint_state"] = [float(np.atleast_1d(g)[0]) if g is not None else 1.0]
        return a

    def _end_episode(self) -> None:
        if self.robot is not None:
            self.robot.terminated = True
            with self._lock:
                self._lock.notify_all()
            if self.on_episode_end is not None:
                try:
                    self.on_episode_end(self.episode_index)
                except Exception as e:  # noqa: BLE001
                    self.log(f"[bridge] on_episode_end failed: {e}")
            self.robot = None
