"""Robosuite (robomimic NutAssemblySquare) adapter mirroring LiberoRobot.

The programs' api surface (capture / move_cartesian / move_pose /
set_gripper / raw _step_env replay / gt traces / films) is inherited from
LiberoRobot unchanged — LIBERO is itself robosuite underneath, so the obs
schema (robot0_eef_pos, gripper qpos, camera images) and the 7-dim OSC
action convention are identical. This subclass replaces only what LIBERO
owns: environment construction (from the robomimic dataset's own
env_args, so the controller matches the demos exactly), episode
initialization (ASPIRE-style environment seeds instead of init-state
files), and the benchmark success check (_check_success).
"""
from __future__ import annotations

import json
import time

import h5py
import numpy as np

from heron.robot.libero import LiberoRobot


class _SuccessShim:
    """Give a raw robosuite env the LIBERO wrapper's face: .env and
    check_success(). Everything else passes through."""

    def __init__(self, env):
        self.env = env

    def step(self, action):
        obs, reward, done, info = self.env.step(np.asarray(action))
        return obs, reward, done, info

    def check_success(self) -> bool:
        return bool(self.env._check_success())

    def __getattr__(self, name):
        return getattr(self.env, name)


class RobosuiteRobot(LiberoRobot):
    """NutAssemblySquare from the robomimic square_ph dataset.

    episode >= 0 selects the ASPIRE-style environment seed: the layout is
    whatever `np.random.seed(seed); env.reset()` produces, which is the
    dataset-independent, benchmark-official randomization.
    """

    def __init__(self, dataset: str, episode: int, horizon: int = 500,
                 camera_hw: int = 256) -> None:
        import robosuite
        import robosuite.utils.camera_utils as CU

        # "path.hdf5::EnvName" borrows the dataset's controller/robot config
        # but instantiates a different env (e.g. the native two-nut NutAssembly)
        env_override = None
        if "::" in dataset:
            dataset, env_override = dataset.split("::", 1)
        with h5py.File(dataset, "r") as f:
            env_args = json.loads(f["data"].attrs["env_args"])
        if env_override:
            env_args["env_name"] = env_override
        kw = dict(env_args["env_kwargs"])
        for k in ("has_renderer", "has_offscreen_renderer", "use_camera_obs",
                  "camera_names", "camera_depths", "camera_heights",
                  "camera_widths", "use_object_obs", "reward_shaping",
                  "ignore_done", "horizon", "render_gpu_device_id"):
            kw.pop(k, None)
        robots = kw.get("robots", ["Panda"])
        self._num_arms = len(robots) if isinstance(robots, (list, tuple)) else 1
        if self._num_arms > 1:
            self._camera_map = {"cam_high": "shouldercamera0",
                                "cam_high2": "shouldercamera1",
                                "cam_arm_wrist": "robot0_eye_in_hand",
                                "cam_arm_wrist2": "robot1_eye_in_hand"}
        else:
            self._camera_map = {"cam_high": "agentview",
                                "cam_arm_wrist": "robot0_eye_in_hand"}
        env = robosuite.make(
            env_args["env_name"],
            has_renderer=False,
            has_offscreen_renderer=True,
            use_camera_obs=True,
            use_object_obs=False,
            camera_names=list(self._camera_map.values()),
            camera_depths=True,
            camera_heights=camera_hw,
            camera_widths=camera_hw,
            reward_shaping=False,
            ignore_done=False,
            horizon=horizon,
            **kw,
        )
        self.env = _SuccessShim(env)
        self._cu = CU
        self._dataset = dataset
        self.task_language = env_args.get("env_name", "")

        np.random.seed(int(episode))
        self.obs = self.env.env.reset()

        # state the inherited machinery expects
        self.cameras = list(self._camera_map)
        self.terminated = False
        self.task_success = False
        self.sim_steps = 0
        self._grip_cmd = -1.0
        self._grip_cmds = [-1.0] * self._num_arms
        self._active_arm = 0
        self._last_close_gap = 0.0
        self.last_move_residual = 0.0
        self._gt_file = None
        self._gt_every = 12
        self._record_every = 0
        # LiberoRobot.__init__ (not called here) also initialises the film hooks
        # that _step_env/set_gripper read; default them off for robosuite.
        self._film_size = 0
        self._film_frames = []
        self._film_cam = None
        self.frames: list = []
        self.frame_times: list = []
        self._recording_t0 = time.time()
        tz = self._detect_table_z()
        self.table_z = float(tz) if tz is not None else 0.8

    # -- two-arm support -----------------------------------------------------
    # TwoArm* envs take a 14-dim action ([dpos,drot,grip] per robot). The
    # inherited single-arm machinery drives ONE arm at a time (the other
    # holds still with its own last grip command); raw two-arm demo actions
    # replay through step_raw.
    def _arm_index(self, arm) -> int:
        if isinstance(arm, int):
            return arm
        return {"left": 0, "right": 1, "arm0": 0, "arm1": 1,
                "robot0": 0, "robot1": 1, "": 0, "arm": 0}.get(str(arm), 0)

    @property
    def _n_arms(self) -> int:
        return getattr(self, "_num_arms", 1)

    def select_arm(self, arm) -> None:
        """Route the inherited single-arm api (move_cartesian, set_gripper,
        ...) to this arm until changed. Their `arm` string params also pass
        through _arm_index, so explicit names work too."""
        self._active_arm = self._arm_index(arm)

    def _eef(self) -> np.ndarray:
        i = getattr(self, "_active_arm", 0)
        return np.asarray(self.obs[f"robot{i}_eef_pos"], dtype=float)

    def _finger_gap(self) -> float:
        i = getattr(self, "_active_arm", 0)
        q = np.asarray(self.obs.get(f"robot{i}_gripper_qpos", []), dtype=float).ravel()
        if q.size < 2:          # tool end-effectors (robosuite Wipe) have no fingers
            return 0.0
        return float(abs(q[0] - q[1]))

    def _step_env(self, delta_pos, delta_rot=None):
        if self._n_arms == 1:
            # Envs whose end-effector is a tool with no gripper (robosuite Wipe)
            # take 6-dim actions; the inherited 7-dim path would assert.
            if int(getattr(self.env.env, "action_dim", 7)) == 6:
                if self.terminated:
                    return
                rot = np.zeros(3) if delta_rot is None else np.clip(delta_rot, -1, 1)
                return self._raw_step(np.concatenate([np.clip(delta_pos, -1, 1), rot]))
            return super()._step_env(delta_pos, delta_rot)
        rot = np.zeros(3) if delta_rot is None else np.clip(delta_rot, -1, 1)
        i = getattr(self, "_active_arm", 0)
        grips = getattr(self, "_grip_cmds", [-1.0, -1.0])
        grips[i] = self._grip_cmd
        parts = []
        for j in range(self._n_arms):
            if j == i:
                parts += [np.clip(delta_pos, -1, 1), rot, [grips[j]]]
            else:
                parts += [np.zeros(3), np.zeros(3), [grips[j]]]
        self._grip_cmds = grips
        return self._raw_step(np.concatenate(parts))

    def step_raw(self, action: np.ndarray):
        """Replay one raw dataset action (7-dim or 14-dim), verbatim."""
        a = np.asarray(action, dtype=float)
        if self._n_arms > 1:
            self._grip_cmds = [float(a[6]), float(a[13])]
            self._grip_cmd = self._grip_cmds[getattr(self, "_active_arm", 0)]
        else:
            self._grip_cmd = float(a[-1])
        return self._raw_step(a)

    def _raw_step(self, action: np.ndarray):
        if self.terminated:
            return
        try:
            self.obs, _, done, info = self.env.step(action.tolist())
        except ValueError as e:
            if "terminated" not in str(e):
                raise
            self.terminated = True
            return
        if done:
            self.terminated = True
        self.sim_steps += 1
        if self._gt_file is not None and self.sim_steps % self._gt_every == 0:
            self._gt_write()
        if self._record_every and self.sim_steps % self._record_every == 0:
            view = self.obs.get("agentview_image")
            if view is not None:
                import time as _t
                self.frames.append(np.asarray(view, dtype=np.uint8)[::-1].copy())
                self.frame_times.append(round(_t.time() - self._recording_t0, 3))
        if self.sim_steps % 5 == 0 or done:
            try:
                if bool(self.env.check_success()):
                    self.task_success = True
            except Exception:
                if done:
                    self.task_success = True

    # -- robomimic-specific hooks -------------------------------------------
    def set_init_state(self, state: np.ndarray):
        """Reset the sim to a flattened mujoco state from the dataset —
        used for same-instrument anchor measurement on the demos' own
        inits (LAWS #6), exactly as robomimic replays do."""
        sim = self._sim()
        sim.set_state_from_flattened(np.asarray(state, dtype=np.float64))
        sim.forward()
        self.obs, _, _, _ = self.env.step(np.zeros(int(self.env.env.action_dim)).tolist())
        self.sim_steps += 1
        self.terminated = False
        self.task_success = False
        return self.obs

    def start_recording(self, every: int = 8) -> None:
        self._record_every = int(every)
        self._recording_t0 = time.time()

    def shutdown(self) -> None:
        try:
            self.env.env.close()
        except Exception:
            pass

    stop = shutdown
