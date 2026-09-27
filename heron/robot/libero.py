"""LIBERO simulation backend: the benchmark Pigey evaluated on, as a RobotInterface.

One Franka arm (named "arm"), OSC position control, agentview + eye-in-hand
cameras with REAL mujoco depth, intrinsics, and live extrinsics — so grounding,
verification, and diagnosis run exactly as on hardware. The decision stack is
untouched; only this adapter knows it is talking to a simulator.

Camera conventions are a minefield (robosuite renders OpenGL, images arrive
bottom-up, extrinsics are z-backward). `tools/libero_selfcheck.py` resolves the
sign choices empirically; the constants below encode its verdict.
"""
from __future__ import annotations

import json
import os
import time
from typing import Any

import numpy as np

from ..config import HeronConfig
from ..types import Frame

# Pigey's LIBERO home, but expressed RELATIVE to the working surface: the same
# absolute height that clears a 0.895 m table is a metre of pointless travel in
# the floor scenes, and vice versa.
HOME_OFFSET = np.array([-0.208, 0.0, 0.278])
P_GAIN_SCALE = 0.05  # their move_ee_to: action = clip(err / 0.05, -1, 1)
POS_TOL = 0.012
STEPS_PER_SECOND = 60
ROT_GAIN = 0.30   # radians of orientation error that saturate the action
ROT_TOL = 0.06    # ~3.4 deg; tighter than this the OSC controller just dithers
GRIPPER_SETTLE_STEPS = 20
HELD_MIN_GAP = 0.005  # finger gap after a close that still counts as holding
# This gripper is binary — robosuite takes -1 (open) or +1 (close) — so a
# commanded WIDTH has to be read as an intent. Below this it means "grip";
# at or above it, "let go".
#
# It was 0.05, which is wider than some things the arm is asked to hold, and
# `place` now opens only as far as the object needs rather than always to the
# full 80 mm. Releasing a 20 mm object asks for 44 mm — under the old threshold
# that is a CLOSE, so the place would have squeezed instead of letting go, on a
# backend where nothing would have reported an error. Every width the codebase
# actually commands is pinned in tests/test_libero_gripper.py.
GRIP_INTENT_WIDTH_M = 0.025
# Bounds on what counts as "part of the scene" when the safety box is widened
# to cover it. Without them the room's floor plane is a scene object and the
# box comes out at +/- 3.12 m.
SCENE_GEOM_MAX_HALF_SIZE = 0.45   # bigger than this is furniture-sized backdrop
SCENE_GEOM_MAX_HEIGHT = 0.60      # above or below the working surface
ARM_REACH_M = 0.95                # Panda's reach; nothing beyond it is a target


def widened_box(mins, maxs, base_xy, current_x, current_y,
                margin: float = 0.12, reach: float = ARM_REACH_M):
    """The safety box that covers this scene, bounded by what the arm can reach.

    Pulled out of the backend because the rule was wrong twice. First it was the
    configured box, which excluded a cabinet the benchmark had placed at
    x = -0.51, so every approach to its drawer was refused as a safety violation
    and the repair loop retried the refused motion until the budget ran out.
    Then it was the extent of every geom in the model, which caught the room's
    floor plane and produced +/- 3.12 m — not a loose check but no check, which
    is the worse of the two failures.

    Two invariants, both worth stating in code rather than in a comment:
    the result never NARROWS what was configured, and it never extends past the
    arm's reach from its own base.
    """
    mins = np.asarray(mins, float)
    maxs = np.asarray(maxs, float)
    base = np.asarray(base_xy, float)
    want = [(float(mins[i] - margin), float(maxs[i] + margin)) for i in (0, 1)]
    want = [(max(lo, float(base[i]) - reach), min(hi, float(base[i]) + reach))
            for i, (lo, hi) in enumerate(want)]
    cur = [tuple(float(v) for v in current_x), tuple(float(v) for v in current_y)]
    out = [(min(cur[i][0], want[i][0]), max(cur[i][1], want[i][1])) for i in (0, 1)]
    return out[0], out[1]


class LiberoRobot:
    def __init__(self, cfg: HeronConfig) -> None:
        os.environ.setdefault("MUJOCO_GL", "egl")
        os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")
        try:
            from libero.libero import benchmark, get_libero_path  # noqa: PLC0415
            from libero.libero.envs import OffScreenRenderEnv  # noqa: PLC0415
        except ImportError as e:
            raise RuntimeError("LIBERO is not installed in this environment "
                               "(pip install -e <LIBERO checkout> robosuite bddl)") from e
        import robosuite.utils.camera_utils as CU  # noqa: PLC0415

        self._cu = CU
        lib = cfg.libero
        # LIBERO-PRO ships perturbed problem files rather than a registered
        # benchmark, so an explicit bddl path bypasses the benchmark lookup.
        if lib.bddl_file:
            bddl = lib.bddl_file
            suite = None
            self.task_language = lib.language or ""
        else:
            suite = benchmark.get_benchmark_dict()[lib.suite]()
            task = suite.get_task(lib.task_id)
            self.task_language = lib.language or task.language
            bddl = os.path.join(get_libero_path("bddl_files"), task.problem_folder, task.bddl_file)
        self._camera_map = {"cam_high": "agentview", "cam_arm_wrist": "robot0_eye_in_hand"}
        env_kwargs = dict(
            bddl_file_name=bddl,
            camera_heights=lib.image_size, camera_widths=lib.image_size,
            camera_depths=True,
            camera_names=list(self._camera_map.values()),
        )
        if lib.horizon:
            env_kwargs["horizon"] = int(lib.horizon)
        self.env = OffScreenRenderEnv(**env_kwargs)
        self.env.seed(lib.seed)
        self.env.reset()
        if lib.plain_robot:
            _m = self.env.env.sim.model
            for _g in range(_m.ngeom):
                _name = _m.geom_id2name(_g) or ""
                if _name.startswith(("robot0", "gripper0", "mount0")):
                    _m.geom_matid[_g] = -1
                    _m.geom_rgba[_g] = np.array([0.80, 0.80, 0.83, 1.0])
                # Living-room/study table textures fail to load under EGL
                # (the table renders flat olive); strip those materials to a
                # neutral warm gray. Kitchen wood loads fine and keeps its
                # material because its geoms are named differently.
                elif getattr(lib, "plain_table", False) and ("table" in _name or _name == "floor"):
                    _m.geom_matid[_g] = -1
                    _m.geom_rgba[_g] = np.array([0.82, 0.79, 0.73, 1.0])
        # LIBERO's benchmark init-state files predate torch 2.6's weights_only
        # default and contain numpy pickles; they ship with the (trusted, local)
        # benchmark, so load them the pre-2.6 way for just this call.
        import functools  # noqa: PLC0415

        import torch  # noqa: PLC0415

        orig_load = torch.load
        try:
            torch.load = functools.partial(orig_load, weights_only=False)
            if lib.init_states_file:
                init_states = torch.load(lib.init_states_file)
            elif suite is not None:
                init_states = suite.get_task_init_states(lib.task_id)
            else:
                init_states = None
        finally:
            torch.load = orig_load
        if init_states is not None and len(init_states):
            self.env.set_init_state(init_states[lib.episode % len(init_states)])

        # LIBERO ships two scene families: floor manipulation (objects at z~0)
        # and tabletop (surface at z~0.895). One configured table height cannot
        # serve both — with the wrong one, every grounding is discarded as
        # implausible even when it is accurate to millimetres. Read it from the
        # scene instead of trusting the config.
        measured = self._detect_table_z()
        if measured is not None and abs(measured - cfg.table_z) > 0.05:
            print(f"[libero] table_z {cfg.table_z:.3f} -> {measured:.3f} (from the scene)")
            cfg.table_z = float(measured)
        self.table_z = cfg.table_z
        self._widen_workspace(cfg)

        self.arms = ["arm"]
        self.cameras = list(self._camera_map)
        self.task_success = False
        self.terminated = False
        self.last_move_residual = 0.0
        self.sim_steps = 0
        self._grip_cmd = -1.0  # robosuite: -1 open, +1 close
        self._last_close_gap = 0.0
        self.obs: dict[str, Any] = {}
        # Episode film strip: the reason a run failed is usually obvious in
        # motion and invisible in a journal, so every episode records one.
        self._record_every = int(lib.record_every or 0)
        self._film_size = int(getattr(lib, "film_size", 0) or 0)
        # When each frame was taken, in seconds since the recorder started, so
        # the film and the journal's events share one timeline.
        self.frame_times: list[float] = []
        self._recording_t0 = time.time()
        self.frames: list[np.ndarray] = []
        # Ground-truth sidecar (see start_gt_trace). Off until a harness asks.
        self._gt_file = None
        self._gt_t0 = 0.0
        self._gt_every = 12   # ~5 records per simulated second at 60 steps/s
        for _ in range(12):  # settle physics with the gripper open
            self._step_env(np.zeros(3))

    def _widen_workspace(self, cfg) -> None:
        """Let the safety box cover the scene the benchmark actually laid out.

        The configured box was a guess, and a guess that excluded objects: the
        perturbed goal scenes put a cabinet at x = -0.51 against a box that
        stopped at -0.45, so the approach to its drawer was refused as a safety
        violation and the repair loop retried the same refused motion. The box
        exists to stop the arm going somewhere silly; anywhere the benchmark
        chose to place an object is not silly.

        Only ever WIDENED, never narrowed — a config that is deliberately tight
        for a real rig must not be loosened by a simulator's opinion, and this
        backend only ever runs in simulation. And widened only as far as the arm
        can actually reach: the first version of this took the extent of every
        geom in the model, caught the room's floor plane, and produced a box of
        +/- 3.12 m. A box that large is not a loose safety check, it is no safety
        check, which is a worse failure than the one it was fixing.
        """
        sim = self.env.env.sim if hasattr(self.env, "env") else self.env.sim
        m, d = sim.model, sim.data
        mins = np.array([np.inf, np.inf])
        maxs = np.array([-np.inf, -np.inf])
        for g in range(m.ngeom):
            body = m.body_id2name(int(m.geom_bodyid[g])) or ""
            if body.startswith("robot0") or body.startswith("gripper0") or body == "world":
                continue
            if int(m.geom_type[g]) == 0:        # a plane is the room, not an object
                continue
            s = np.abs(np.asarray(m.geom_size[g][:2], float))
            if float(s.max()) > SCENE_GEOM_MAX_HALF_SIZE:
                continue                        # walls and backdrops
            c = np.asarray(d.geom_xpos[g][:2], float)
            if abs(float(d.geom_xpos[g][2]) - self.table_z) > SCENE_GEOM_MAX_HEIGHT:
                continue                        # ceiling fixtures, floor markings
            mins = np.minimum(mins, c - s)
            maxs = np.maximum(maxs, c + s)
        if not np.all(np.isfinite(mins)):
            return
        try:
            base = np.asarray(d.body_xpos[m.body_name2id("robot0_base")][:2], float)
        except Exception:
            base = np.zeros(2)
        new_x, new_y = widened_box(mins, maxs, base,
                                   cfg.safety.workspace_x, cfg.safety.workspace_y)
        if new_x != tuple(cfg.safety.workspace_x) or new_y != tuple(cfg.safety.workspace_y):
            print(f"[libero] workspace x{tuple(round(v, 2) for v in cfg.safety.workspace_x)} "
                  f"-> {tuple(round(v, 2) for v in new_x)}, "
                  f"y{tuple(round(v, 2) for v in cfg.safety.workspace_y)} "
                  f"-> {tuple(round(v, 2) for v in new_y)} (to cover the scene)")
        cfg.safety.workspace_x = list(new_x)
        cfg.safety.workspace_y = list(new_y)

    def _detect_table_z(self):
        """Height of the working surface, from the simulator's own model."""
        sim = self.env.env.sim if hasattr(self.env, "env") else self.env.sim
        try:
            names = [sim.model.body_id2name(i) for i in range(sim.model.nbody)]
        except Exception:
            return None
        for key in ("main_table", "table"):
            for name in names:
                if name and key in name:
                    try:
                        bid = sim.model.body_name2id(name)
                    except Exception:
                        continue
                    z = float(sim.data.body_xpos[bid][2])
                    # Body origin sits inside the slab; the top is what matters.
                    try:
                        gids = [g for g in range(sim.model.ngeom)
                                if sim.model.geom_bodyid[g] == bid]
                        if gids:
                            z = max(float(sim.data.geom_xpos[g][2]
                                          + sim.model.geom_size[g][2]) for g in gids)
                    except Exception:
                        pass
                    return z
        return None

    # -- low level -----------------------------------------------------------
    def _step_env(self, delta_pos: np.ndarray, delta_rot: np.ndarray | None = None) -> None:
        # LIBERO terminates at its horizon and then refuses further actions. That
        # is the environment ending, not an error: keep the last observation and
        # let the agent finish its reasoning instead of crashing the episode.
        if self.terminated:
            return
        rot = np.zeros(3) if delta_rot is None else np.clip(delta_rot, -1, 1)
        action = np.concatenate([np.clip(delta_pos, -1, 1), rot, [self._grip_cmd]])
        try:
            self.obs, _, done, info = self.env.step(action.tolist())
        except ValueError as e:
            # robosuite refuses actions once the horizon is reached. The done flag
            # does not always reach us first, so the refusal itself is the signal.
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
            view = None
            if self._film_size:
                # A dedicated render for the film strip only: the policy keeps
                # observing at image_size, and never sees this buffer.
                try:
                    view = self.env.env.sim.render(
                        width=self._film_size, height=self._film_size,
                        camera_name="agentview")
                except Exception:
                    view = None
            if view is None:
                view = self.obs.get("agentview_image")
            if view is not None:
                self.frames.append(np.asarray(view, dtype=np.uint8)[::-1].copy())
                self.frame_times.append(round(time.time() - self._recording_t0, 3))
        # check_success() is LIBERO's canonical benchmark verdict (step's done
        # flag also fires on horizon, and info lacks a success key).
        if self.sim_steps % 5 == 0 or done:
            try:
                if bool(self.env.check_success()):
                    self.task_success = True
            except Exception:
                if done:
                    self.task_success = True

    def _eef(self) -> np.ndarray:
        return np.asarray(self.obs["robot0_eef_pos"], dtype=float)

    # -- RobotInterface ------------------------------------------------------
    def home(self, arm: str) -> None:
        home = HOME_OFFSET + np.array([0.0, 0.0, self.table_z])
        self.move_cartesian(arm, home, seconds=3.0)

    def move_cartesian(self, arm: str, xyz: np.ndarray, seconds: float = 2.0) -> None:
        target = np.asarray(xyz, dtype=float)
        max_steps = max(40, int(STEPS_PER_SECOND * max(seconds, 0.5)) * 2)
        for _ in range(max_steps):
            err = target - self._eef()
            if float(np.linalg.norm(err)) < POS_TOL:
                break
            self._step_env(err / P_GAIN_SCALE)
        # A descent blocked by the object it is reaching for stops short and
        # slides sideways, and returning silently makes that look like arrival:
        # the caller then grasps air and the failure is attributed to belief
        # rather than to motion. Measured: 44.7 mm of unreported residual.
        self.last_move_residual = float(np.linalg.norm(target - self._eef()))

    def get_cartesian(self, arm: str) -> np.ndarray:
        return self._eef().copy()

    # -- orientation ---------------------------------------------------------
    # Everything above commands position with the wrist frozen wherever it
    # started, which is straight down. That is the whole tabletop repertoire and
    # it was fine until a drawer: measured, the pre-grasp point in front of a
    # drawer handle is 314 mm out of reach with a downward wrist and 19 mm away
    # once the wrist may turn. The action always had six slots; three of them
    # were being sent as zeros.

    def get_tool_rotation(self, arm: str) -> np.ndarray:
        import robosuite.utils.transform_utils as T  # noqa: PLC0415

        return T.quat2mat(np.asarray(self.obs["robot0_eef_quat"], dtype=float))

    def move_pose(self, arm: str, xyz: np.ndarray, rotation: np.ndarray | None = None,
                  seconds: float = 3.0, pos_tol: float = POS_TOL) -> None:
        """Drive position and orientation together.

        `rotation` is a 3x3 tool-to-world matrix; None keeps the current one and
        makes this identical to move_cartesian.
        """
        import robosuite.utils.transform_utils as T  # noqa: PLC0415

        target = np.asarray(xyz, dtype=float)
        rot = None if rotation is None else np.asarray(rotation, dtype=float)
        if rot is not None and float(np.linalg.det(rot)) < 0.99:
            # A reflection is not an orientation. Caught here because the first
            # version of the drawer probe built three of them by hand, and the
            # arm chased each for four seconds while reporting a 177-degree
            # error — which reads exactly like an unreachable pose.
            raise ValueError(f"rotation is not a proper rotation (det={np.linalg.det(rot):.3f})")
        for _ in range(max(60, int(STEPS_PER_SECOND * max(seconds, 0.5)))):
            err = target - self._eef()
            rot_cmd = np.zeros(3)
            if rot is not None:
                cur = self.get_tool_rotation(arm)
                rot_cmd = np.clip(T.quat2axisangle(T.mat2quat(rot @ cur.T)) / ROT_GAIN, -1, 1)
            if float(np.linalg.norm(err)) < pos_tol and float(np.linalg.norm(rot_cmd)) < ROT_TOL:
                break
            self._step_env(err / P_GAIN_SCALE, rot_cmd)
        self.last_move_residual = float(np.linalg.norm(target - self._eef()))

    def set_gripper(self, arm: str, width_m: float, effort_limit: float | None = None) -> None:
        self._grip_cmd = 1.0 if width_m < GRIP_INTENT_WIDTH_M else -1.0
        for _ in range(GRIPPER_SETTLE_STEPS):
            self._step_env(np.zeros(3))
        if self._grip_cmd > 0:
            self._last_close_gap = self._finger_gap()

    def _finger_gap(self) -> float:
        q = np.asarray(self.obs["robot0_gripper_qpos"], dtype=float)
        return float(abs(q[0] - q[1]))

    def get_gripper(self, arm: str) -> dict[str, float]:
        gap = self._finger_gap()
        holding = self._grip_cmd > 0 and gap > HELD_MIN_GAP
        return {"width_m": gap, "effort": 3.0 if holding else 0.05}

    def get_external_effort(self, arm: str) -> float:
        return 0.0  # no compensated F/T in obs; contact-descend is disabled in libero configs

    def capture(self, camera: str) -> Frame:
        rs_cam = self._camera_map[camera]
        sim = self.env.env.sim if hasattr(self.env, "env") else self.env.sim
        rgb = np.asarray(self.obs[f"{rs_cam}_image"], dtype=np.uint8)[::-1].copy()
        depth_raw = np.asarray(self.obs[f"{rs_cam}_depth"])[::-1].copy()
        depth = self._cu.get_real_depth_map(sim, depth_raw).squeeze().astype(np.float32)
        h, w = rgb.shape[:2]
        intr = self._cu.get_camera_intrinsic_matrix(sim, rs_cam, h, w)
        # robosuite's extrinsic already carries the OpenGL->CV correction
        # (selfcheck: the GL flip projects behind the camera).
        t_base_cam = self._cu.get_camera_extrinsic_matrix(sim, rs_cam)
        return Frame(camera=camera, rgb=rgb, depth=depth, intrinsics=intr,
                     t_base_cam=t_base_cam, t=time.time())

    def save_gif(self, path, fps: int = 15, max_frames: int = 120, width: int = 256) -> bool:
        """Write the episode film strip. Returns False if nothing was recorded.

        Aggressively subsampled and downscaled: these are for judging WHAT went
        wrong at a glance, and a full-resolution strip runs to tens of megabytes,
        which nobody opens.
        """
        if not self.frames:
            return False
        from pathlib import Path  # noqa: PLC0415

        import imageio.v2 as imageio  # noqa: PLC0415

        frames = self.frames
        if len(frames) > max_frames:      # keep GIFs viewable; drop evenly
            idx = np.linspace(0, len(frames) - 1, max_frames).astype(int)
            frames = [frames[i] for i in idx]
        if self._film_size:
            width = 0        # rendered at this size on purpose; keep it
        if width and frames[0].shape[1] > width:
            step = max(1, frames[0].shape[1] // width)   # nearest-neighbour decimation
            frames = [f[::step, ::step] for f in frames]
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        imageio.mimsave(str(path), frames, duration=1.0 / max(fps, 1), loop=0)
        # The kept frames' own timestamps, so a player can put the film and the
        # journal on one timeline. Subsampled with the frames, or the two would
        # silently disagree.
        from .trossen_sim import _write_frame_times  # noqa: PLC0415

        kept = ([self.frame_times[i] for i in idx]
                if len(self.frames) > max_frames and self.frame_times else self.frame_times)
        _write_frame_times(path, kept, len(frames))
        return True

    # -- ground truth --------------------------------------------------------
    # The simulator knows where everything actually is. On hardware no such
    # oracle exists, so nothing in the decision stack may read any of this: it
    # is written to a sidecar file so a journal's beliefs can be judged against
    # the world OFFLINE — which bowl a grounding actually named, how the cargo
    # really rode in the gripper, where a placement truly landed.

    def _sim(self):
        return self.env.env.sim if hasattr(self.env, "env") else self.env.sim

    def gt_poses(self) -> dict[str, list[float]]:
        """World pose (xyz + wxyz quat) of every free body — the movable objects.

        Free joints select exactly the things the benchmark scatters on the
        table; the arm, the fixtures, and the room are attached and excluded.
        """
        sim = self._sim()
        m, d = sim.model, sim.data
        out: dict[str, list[float]] = {}
        for j in range(m.njnt):
            if int(m.jnt_type[j]) != 0:     # mjJNT_FREE
                continue
            bid = int(m.jnt_bodyid[j])
            name = m.body_id2name(bid) or f"body{bid}"
            out[name] = ([round(float(v), 4) for v in d.body_xpos[bid]]
                         + [round(float(v), 4) for v in d.body_xquat[bid]])
        return out

    def gt_joints(self) -> dict[str, float]:
        """qpos of every scene slide and hinge joint — the drawers and doors.

        Free bodies answer "where did the bowl land"; these answer "did the
        drawer actually move". The arm's own joints are excluded by name so
        the trace stays about the furniture.
        """
        sim = self._sim()
        m, d = sim.model, sim.data
        out: dict[str, float] = {}
        for j in range(m.njnt):
            if int(m.jnt_type[j]) not in (2, 3):    # mjJNT_SLIDE, mjJNT_HINGE
                continue
            name = m.joint_id2name(j) or f"joint{j}"
            if name.startswith(("robot", "gripper")):
                continue
            out[name] = round(float(d.qpos[int(m.jnt_qposadr[j])]), 4)
        return out

    def start_gt_trace(self, path, journal_t0: float | None = None) -> None:
        """Stream ground truth to `path`, stamped on the journal's clock.

        `journal_t0` is the EpisodeLogger's t0, so a trace record and a journal
        event with the same `t` describe the same moment. Sim time would drift
        against it — the simulator does not step while a model call is in
        flight — which is exactly why wall clock is the shared axis.
        """
        from pathlib import Path  # noqa: PLC0415

        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        self._gt_t0 = float(journal_t0) if journal_t0 is not None else time.time()
        self._gt_file = p.open("a")
        env = getattr(self.env, "env", self.env)
        goal = getattr(env, "obj_of_interest", None)
        self._gt_file.write(json.dumps({
            "kind": "meta", "language": self.task_language,
            "obj_of_interest": list(goal) if goal else None,
            "bodies": sorted(self.gt_poses()),
        }) + "\n")
        self._gt_write()

    def _gt_write(self) -> None:
        rec = {"t": round(time.time() - self._gt_t0, 3), "sim_steps": self.sim_steps,
               "eef": [round(float(v), 4) for v in self._eef()],
               "gap": round(self._finger_gap(), 4), "grip": self._grip_cmd,
               "bodies": self.gt_poses(), "joints": self.gt_joints()}
        self._gt_file.write(json.dumps(rec) + "\n")
        self._gt_file.flush()

    def settle(self, seconds: float = 1.0) -> None:
        """Let physics run on with no command.

        The benchmark's success predicate is only evaluated while the simulator
        is stepping, and a released object is still in the air for a few frames.
        Stopping the moment the gripper opens therefore reads the world mid-fall
        — and scores a placement that is about to succeed as a failure. On real
        hardware the same dwell is just as necessary for a different reason: the
        object has to come to rest before it is worth looking at.
        """
        for _ in range(max(1, int(STEPS_PER_SECOND * seconds))):
            self._step_env(np.zeros(3))

    def stop(self) -> None:
        pass

    def shutdown(self) -> None:
        if self._gt_file is not None:
            try:
                self._gt_write()          # the final resting poses
                self._gt_file.close()
            except Exception:
                pass
            self._gt_file = None
        try:
            self.env.close()
        except Exception:
            pass
