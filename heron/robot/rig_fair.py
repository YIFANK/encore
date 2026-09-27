"""Fair-protocol adapter over the physical Trossen rig.

The fair contract on hardware differs from simulation in exactly three ways,
and each difference is a protocol requirement rather than an implementation
detail.

  * An episode's initial state comes from a coordinator-dispatched *pose card*
    (``tools/rig_protocol.py``) realized by the reset executor, not from a
    seed.  The card id is recorded per episode so a trial is reproducible to
    the same tolerance the reset achieves.
  * ``task_success`` is never known during an episode.  The rig has no goal
    predicate; the coordinator's judge (``tools/rig_judge.py``) decides
    afterwards from sealed post-episode captures.  ``terminated`` and
    ``task_success`` therefore stay False for the whole episode, so a program
    cannot read a success signal even by accident — on hardware this is a
    structural guarantee rather than a gate we have to enforce.
  * ``sim_steps`` counts issued motion commands.  The rig has no simulator
    clock; commands are the honest hardware analogue and are what the cost
    tables account.

Everything else — the RPC surface, the sealed splits, the md5 freeze, the AST
gate — is the machinery the simulated campaigns already run under.

Depth: this rig cannot serve depth (macOS will not hand librealsense the
D405s).  Frames therefore carry a table-plane homography instead, and the
program-side ``deproject`` falls back to it.  Anything resting on the table
deprojects exactly; anything above the table needs two cameras and the DLT
projection, which is exposed as ``proj``.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np


class RigSafetyError(RuntimeError):
    """A commanded motion left the declared workspace or exceeded an effort cap."""


class RigRobot:
    """Fair-API-shaped view of the physical rig.

    Constructed by the coordinator, never by a policy program: the program
    lives in a separate process and reaches this object only over the fair
    RPC socket.
    """

    def __init__(self, cfg, *, episode: int, card: dict | None = None,
                 language: str = "", arm: str | None = None,
                 max_commands: int = 400, trace_dir: Path | None = None,
                 backend: str = "real", scene: str | None = None) -> None:
        """`backend="twin"` swaps the hardware for the MuJoCo twin of this same
        cell — same URDF, same `heron.robot.kinematics`, same calibration — so
        one policy program runs unchanged in both. That is the whole point:
        what survives the reality gap becomes a readable diff instead of a
        success rate. The twin does NOT model depth noise, gripper force, or
        the driver (see trossen_sim's own docstring), which is exactly where
        the gap is expected to show up."""
        if backend == "twin":
            from .trossen_sim import TrossenSim  # noqa: PLC0415
            self._rig = (TrossenSim(cfg, scene=scene) if scene
                         else TrossenSim(cfg))
        elif backend == "real":
            from .trossen import TrossenStationary  # noqa: PLC0415
            self._rig = TrossenStationary(cfg)
        else:
            raise ValueError(f"unknown backend {backend!r} (real|twin)")
        self.backend = backend
        self.cfg = cfg
        # Arms the program may command. A bimanual cell (rig_handover) needs
        # BOTH, and every RPC carries the side it means; single-arm cells keep
        # working because `arm=None` resolves to the first active arm.
        self.arms = list(getattr(cfg, "active_arms", None) or cfg.arms)
        self.arm = arm or self.arms[0]
        if isinstance(self.arm, list):
            self.arm = self.arm[0]
        self.cameras = list(cfg.cameras)
        self.task_language = language
        self.episode = int(episode)
        self.card = card or {}

        # Protocol state.  Both stay False for the whole episode by design:
        # the success bit does not exist until the judge runs.
        self.task_success = False
        self.terminated = False

        self.sim_steps = 0            # issued motion commands
        self.last_move_residual = 0.0
        self._max_commands = int(max_commands)
        self._t0 = time.time()

        # Workspace envelope and effort cap, read from config; a command that
        # violates either raises rather than being silently clamped, because a
        # clamped command would make the program's own residual reading a lie.
        ws = getattr(cfg, "workspace", None)
        self._bounds = None
        if ws is not None:
            lo = getattr(ws, "min_xyz", None)
            hi = getattr(ws, "max_xyz", None)
            if lo is not None and hi is not None:
                self._bounds = (np.asarray(lo, float), np.asarray(hi, float))
        self._effort_cap = float(getattr(ws, "max_effort_n", 25.0) if ws else 25.0)

        # Episode trace: the coordinator's record of what happened, written
        # to a private directory.  Never readable by the program.
        self._trace = []
        self._trace_dir = Path(trace_dir) if trace_dir else None
        self._film: list[np.ndarray] = []
        self._film_every = 0
        self._film_n = 0

    # -- fair RPC surface ----------------------------------------------------

    def _side(self, arm: str | None) -> str:
        """Resolve an RPC's arm name against the arms this cell may command.

        An unknown side raises rather than silently falling back: a bimanual
        program that misspells "left" must fail loudly, not drive the right arm
        to the left arm's waypoint.
        """
        if arm is None or arm == "arm":     # legacy single-arm callers
            return self.arm
        side = str(arm)
        if side not in self.arms:
            raise RigSafetyError(f"arm {side!r} is not active (have {self.arms})")
        return side

    def capture(self, camera: str):
        """One camera capture, shaped for the fair wire format.

        Returns the rig's own Frame.  Where the rig has no depth the frame
        carries ``h_pixel_world``/``plane_z`` and ``proj``; ``fair_run`` sends
        those alongside, and the client's ``deproject`` uses them.
        """
        f = self._rig.capture(camera)
        if self._film_every and self._film_n % self._film_every == 0:
            rgb = getattr(f, "rgb", None)
            if rgb is not None:
                self._film.append(np.asarray(rgb).copy())
        self._film_n += 1
        return f

    # -- perception service ---------------------------------------------------
    # Grounding and VQA as part of the robot's SENSOR SUITE (Yifan's call,
    # 2026-08-18): absolute colour thresholds are statements about one
    # evening's light bulbs (rig LAWS L18). The model runs COORDINATOR-side;
    # the program sees only results, and the fair guarantee stands untouched —
    # these answer "where is X" / "what do you see", never the benchmark bit,
    # which continues to exist only in the post-episode judge.

    MODEL_CALL_BUDGET = 60

    def _orchestrator(self):
        if getattr(self, "_orch", None) is None:
            from ..cli import _build_orchestrator  # noqa: PLC0415
            self._orch = _build_orchestrator("gemini", self.cfg, print)
        return self._orch

    def _count_model_call(self) -> None:
        self._model_calls = getattr(self, "_model_calls", 0) + 1
        if self._model_calls > self.MODEL_CALL_BUDGET:
            raise RigSafetyError(
                f"model-call budget exhausted ({self.MODEL_CALL_BUDGET}) — "
                "perceive less often or cache what you saw")

    def ground(self, query: str, camera: str = "cam_high"):
        """World position of one described object, or None. Lighting-robust."""
        self._count_model_call()
        frame = self.capture(camera)
        hit = self._orchestrator().point(frame, str(query))
        if hit is None:
            self._log_step("ground", {"query": str(query)[:80], "hit": None})
            return None
        u, v = int(hit[0]), int(hit[1])
        xyz = None
        if getattr(frame, "depth", None) is not None:
            xyz = frame.deproject(u, v)
        if xyz is None:
            h = getattr(frame, "h_pixel_world", None)
            if h is not None:
                p = np.asarray(h, float) @ np.array([u, v, 1.0])
                if abs(p[2]) > 1e-9:
                    xy = p[:2] / p[2]
                    xyz = np.array([xy[0], xy[1],
                                    float(getattr(frame, "plane_z", 0.0) or 0.0)])
        if xyz is None:
            return None
        out = {"xyz": [round(float(v_), 4) for v_ in np.asarray(xyz, float)],
               "px": [u, v]}
        self._log_step("ground", {"query": str(query)[:80], **out})
        return out

    def vqa(self, question: str, camera: str = "cam_high") -> dict:
        """One visual question about a fresh frame. Answers 'what do you
        see', never the benchmark bit."""
        self._count_model_call()
        frame = self.capture(camera)
        value, conf, note = self._orchestrator().vqa([frame], str(question))
        out = {"answer": str(value), "confidence": float(conf),
               "note": str(note)[:200]}
        self._log_step("vqa", {"question": str(question)[:80],
                               "answer": out["answer"][:40]})
        return out

    def sam3(self, query: str, camera: str = "cam_high") -> list[dict]:
        """SAM3 text-prompted segmentation via the local server (:8772).

        Returns [{mask (H,W bool), box [x1,y1,x2,y2], score}] sorted by
        score. Counts against the model-call budget like ground/vqa —
        it IS open-vocabulary perception, just a cheaper one.
        """
        import base64 as _b64  # noqa: PLC0415
        import io as _io  # noqa: PLC0415
        import urllib.request as _rq  # noqa: PLC0415
        from PIL import Image as _Image  # noqa: PLC0415
        self._count_model_call()
        frame = self.capture(camera)
        buf = _io.BytesIO()
        _Image.fromarray(np.asarray(frame.rgb)).save(buf, format="JPEG", quality=92)
        req = _rq.Request(
            "http://127.0.0.1:8772/segment",
            data=json.dumps({"image_base64": _b64.b64encode(buf.getvalue()).decode(),
                             "text_prompt": str(query)}).encode(),
            headers={"Content-Type": "application/json"})
        with _rq.urlopen(req, timeout=30) as r:
            rep = json.loads(r.read())
        out = []
        for m in rep.get("results", []):
            h, w = m["shape"]
            mask = np.frombuffer(_b64.b64decode(m["mask_base64"]),
                                 dtype=np.uint8).reshape(h, w).astype(bool)
            out.append({"mask": mask, "box": [float(v) for v in m["box"]],
                        "score": float(m["score"])})
        self._log_step("sam3", {"query": str(query)[:60], "n": len(out)})
        return out

    def get_cartesian(self, arm: str) -> np.ndarray:
        return self._rig.get_cartesian(self._side(arm))

    def get_tool_rotation(self, arm: str) -> np.ndarray:
        side = self._side(arm)
        fn = getattr(self._rig, "get_tool_rotation", None)
        if fn is not None:
            return np.asarray(fn(side), float)
        hold = getattr(self._rig, "_hold_orientation", {}).get(side)
        return np.asarray(hold if hold is not None else np.eye(3), float)

    def get_gripper(self, arm: str) -> dict[str, float]:
        return self._rig.get_gripper(self._side(arm))

    @property
    def obs(self) -> dict[str, Any]:
        """Proprioception, in the same key shape the simulated backends use."""
        q = np.asarray(self._rig.joint_state(self.arm), float)
        xyz = np.asarray(self._rig.get_cartesian(self.arm), float)
        grip = self._rig.get_gripper(self.arm)
        return {
            "robot0_joint_pos": q[:6].tolist(),
            "robot0_eef_pos": xyz.tolist(),
            "robot0_gripper_qpos": [float(grip.get("width_m", 0.0))],
            "robot0_external_effort": float(self._rig.get_external_effort(self.arm)),
        }

    def move_cartesian(self, arm: str, xyz, seconds: float = 2.0) -> None:
        side = self._side(arm)
        self._guard(xyz, side)
        target = np.asarray(xyz, float)
        self._rig.move_cartesian(side, target, seconds=seconds)
        self._after_move(side, target)

    def drag(self, arm: str, xyz, seconds: float = 2.0) -> None:
        """Continuous straight-line move at the CURRENT orientation — for
        table-contact drags (brushing/pushing/wiping). One command, driver-
        level cartesian interpolation, real straightness; the staged mover
        would lift-translate-descend and micro-step chains judder."""
        side = self._side(arm)
        self._guard(xyz, side)
        target = np.asarray(xyz, float)
        fn = getattr(self._rig, "drag_cartesian", None)
        if fn is None:
            self._rig.move_cartesian(side, target, seconds=seconds)
        else:
            fn(side, target, seconds=float(seconds))
        self._after_move(side, target)

    def move_pose(self, arm: str, xyz, rotation=None, seconds: float = 3.0) -> None:
        side = self._side(arm)
        self._guard(xyz, side)
        target = np.asarray(xyz, float)
        fn = getattr(self._rig, "move_pose", None)
        if fn is None:
            # The Trossen backend has no move_pose; its move_cartesian takes the
            # orientation as `orientation=`. Dropping it here (the old fallback)
            # silently turned every oriented move into a default-orientation one.
            # It wants ANGLE-AXIS, while the fair wire carries a 3x3 matrix, so
            # convert: handing a matrix straight through would be read as a
            # rotation vector of the first row.
            self._rig.move_cartesian(side, target, seconds=seconds,
                                     orientation=_as_rvec(rotation))
        else:
            fn(side, target, rotation=rotation, seconds=seconds)
        self._after_move(side, target)

    def move_path(self, arm: str, points, rotation=None,
                  seconds: float = 4.0) -> None:
        """Smooth multi-waypoint motion (no interior stops). Every waypoint
        is workspace-guarded BEFORE anything moves; counts as one command."""
        side = self._side(arm)
        pts = [np.asarray(p, float) for p in points]
        for p in pts:
            self._guard(p, side)
        if rotation is None:
            rv = None
        else:
            r = np.asarray(rotation, float)
            rv = [_as_rvec(np.asarray(m, float)) for m in r] if r.ndim == 3 \
                else _as_rvec(r)
        fn = getattr(self._rig, "move_cartesian_path", None)
        if fn is None:
            # Backend without the streaming primitive: degrade to the old
            # stop-and-go chain rather than refusing the program.
            for i, p in enumerate(pts):
                one = rv[i] if isinstance(rv, list) else rv
                self._rig.move_cartesian(side, p, seconds=seconds / len(pts),
                                         orientation=one)
        else:
            fn(side, pts, seconds=seconds, orientation=rv)
        self._after_move(side, pts[-1])

    # -- optional tool surface (rig only): SAM + guarded manipulation skills.
    # LIBERO campaign programs never see these (the sim fair server does not
    # register the ops); on the rig they trade a little latency for a lot of
    # robustness, at the program's choice.

    def sam(self, camera: str, box=None, point=None, mode: str = "propose",
            max_masks: int = 24) -> list[dict]:
        """SAM over a fresh capture. Wire-friendly candidates only — box,
        centroid pixel, pixel count, score, and the centroid's world position
        through the frame's own deprojection (table-plane exact for objects
        resting on it). Masks stay server-side: a program that wants shape
        detail can ask again with a box prompt around one candidate."""
        seg = getattr(self, "_sam", None)
        if seg is None:
            from ..perception.sam import SamSegmenter  # noqa: PLC0415
            seg = self._sam = SamSegmenter(getattr(self.cfg, "sam_url", None))
        frame = self.capture(camera)
        rgb = np.asarray(frame.rgb)
        if mode == "propose" and box is None and point is None:
            raw = seg.propose(rgb, max_masks=int(max_masks))
        else:
            raw = seg.candidates(rgb, box=box, point=point)
        out = []
        for c in raw:
            mask = c.get("mask")
            if mask is None:
                continue
            vs, us = np.nonzero(mask)
            if not len(us):
                continue
            um, vm = int(np.median(us)), int(np.median(vs))
            world = frame.deproject_plane(um, vm) if hasattr(frame, "deproject_plane") else None
            out.append({
                "box": [int(us.min()), int(vs.min()), int(us.max()), int(vs.max())],
                "centroid_px": [um, vm],
                "pixels": int(len(us)),
                "score": round(float(c.get("score", 0.0)), 3),
                "world": (None if world is None
                          else [round(float(v), 4) for v in world]),
            })
        return out

    def pick_at(self, arm: str, xyz, hover_m: float = 0.08) -> dict:
        """Guarded top-down grasp at a world point: open, hover, descend,
        force-close, width check, lift. Returns {'held': bool, 'width_m': ...}.
        The staged descent, force-controlled close and closed-on-air width
        test are the runtime stack's own moves — the program supplies WHERE,
        this supplies HOW."""
        side = self._side(arm)
        p = np.asarray(xyz, float)
        self._guard(p, side)
        self._guard([p[0], p[1], p[2] + hover_m], side)
        self._rig.set_gripper(side, 0.08)
        self._rig.move_cartesian(side, np.array([p[0], p[1], p[2] + hover_m]),
                                 seconds=2.0)
        self._rig.move_cartesian(side, p, seconds=2.0)
        self._rig.set_gripper(side, 0.0)      # <=0.01 -> force-controlled close
        self._rig.settle(0.8)
        g = self._rig.get_gripper(side)
        held = 0.005 < float(g.get("width_m", 0.0)) < 0.075
        self._rig.move_cartesian(side, np.array([p[0], p[1], p[2] + hover_m]),
                                 seconds=2.0)
        self._after_move(side, np.array([p[0], p[1], p[2] + hover_m]))
        return {"held": bool(held), "width_m": round(float(g.get("width_m", 0.0)), 4)}

    def place_at(self, arm: str, xyz, hover_m: float = 0.12) -> dict:
        """Guarded release at a world point: hover, descend, open, retreat."""
        side = self._side(arm)
        p = np.asarray(xyz, float)
        self._guard(p, side)
        self._guard([p[0], p[1], p[2] + hover_m], side)
        self._rig.move_cartesian(side, np.array([p[0], p[1], p[2] + hover_m]),
                                 seconds=2.0)
        self._rig.move_cartesian(side, p, seconds=2.0)
        self._rig.set_gripper(side, 0.08)
        self._rig.settle(0.8)
        self._rig.move_cartesian(side, np.array([p[0], p[1], p[2] + hover_m]),
                                 seconds=2.0)
        self._after_move(side, np.array([p[0], p[1], p[2] + hover_m]))
        return {"released": True}

    def set_gripper(self, arm: str, width_m: float) -> None:
        side = self._side(arm)
        self._count()
        self._rig.set_gripper(side, float(width_m))
        self._log_step("grip", {"arm": side, "width_m": float(width_m)})

    def settle(self, seconds: float = 1.0) -> None:
        self._rig.settle(seconds)

    # -- coordinator-side machinery -----------------------------------------

    def start_recording(self, every: int = 8) -> None:
        self._film_every = max(1, int(every))

    def save_gif(self, path) -> None:
        if not self._film:
            return
        try:
            import imageio.v2 as imageio  # noqa: PLC0415

            imageio.mimsave(str(path), self._film, duration=0.2)
        except Exception:
            pass

    def start_gt_trace(self, path) -> None:
        """Coordinator-private episode trace (mode 0700 by the caller)."""
        self._trace_dir = Path(path)

    def judge_captures(self) -> dict[str, Any]:
        """Post-episode evidence for the judge: one capture per camera.

        Called by the coordinator AFTER ``run()`` returns and the program's
        process is gone, so nothing here can reach the program.
        """
        out = {}
        for cam in self.cameras:
            try:
                f = self._rig.capture(cam)
                out[cam] = {
                    "rgb": np.asarray(f.rgb),
                    "h_pixel_world": _get(f, "h_pixel_world"),
                    "plane_z": _get(f, "plane_z"),
                    "proj": _get(f, "proj"),
                    "t_base_cam": _get(f, "t_base_cam"),
                    "intrinsics": _get(f, "intrinsics"),
                }
            except Exception as e:  # a camera dropping out must not lose the trial
                out[cam] = {"error": f"{type(e).__name__}: {e}"}
        return out

    def flush_trace(self, path) -> None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("w") as fh:
            for row in self._trace:
                fh.write(json.dumps(row) + "\n")

    def shutdown(self) -> None:
        if self._trace_dir is not None:
            try:
                self.flush_trace(self._trace_dir)
            except Exception:
                pass
        try:
            self._rig.stop(self.arm) if _takes_arm(self._rig.stop) else self._rig.stop()
        except Exception:
            pass
        try:
            self._rig.shutdown()
        except Exception:
            pass

    # -- internals -----------------------------------------------------------

    def _guard(self, xyz, arm: str | None = None) -> None:
        side = self._side(arm)
        p = np.asarray(xyz, float)
        if p.shape != (3,) or not np.all(np.isfinite(p)):
            raise RigSafetyError(f"bad target {xyz!r}")
        if self._bounds is not None:
            lo, hi = self._bounds
            if np.any(p < lo) or np.any(p > hi):
                raise RigSafetyError(
                    f"target {p.round(3).tolist()} outside workspace "
                    f"{lo.round(3).tolist()}..{hi.round(3).tolist()}")
        eff = abs(float(self._rig.get_external_effort(side)))
        if eff > self._effort_cap:
            raise RigSafetyError(f"external effort {eff:.1f} N over cap {self._effort_cap:.1f} N")

    def _count(self) -> None:
        self.sim_steps += 1
        if self.sim_steps > self._max_commands:
            self.terminated = True
            raise RigSafetyError(
                f"command budget exhausted ({self._max_commands}) — episode ends")

    def _after_move(self, side: str, target: np.ndarray) -> None:
        self._count()
        reached = np.asarray(self._rig.get_cartesian(side), float)
        self.last_move_residual = float(np.linalg.norm(reached - target))
        self._log_step("move", {"arm": side, "target": target.tolist(),
                                "reached": reached.tolist(),
                                "residual": self.last_move_residual})

    def _log_step(self, kind: str, payload: dict) -> None:
        self._trace.append({"t": time.time() - self._t0, "step": self.sim_steps,
                            "kind": kind, **payload})


def _as_rvec(rotation):
    """Angle-axis for the backend, from either a 3x3 matrix or an rvec.

    The fair wire format ships orientations as rotation matrices; the Trossen
    primitive layer speaks the driver's angle-axis encoding.
    """
    if rotation is None:
        return None
    r = np.asarray(rotation, float)
    if r.shape == (3,):
        return r
    if r.shape != (3, 3):
        raise RigSafetyError(f"rotation must be 3x3 or a 3-vector, got {r.shape}")
    cos = (np.trace(r) - 1.0) / 2.0
    theta = float(np.arccos(np.clip(cos, -1.0, 1.0)))
    if theta < 1e-9:
        return np.zeros(3)
    if abs(np.pi - theta) < 1e-6:
        a = (r + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(a), 0.0))
        k = int(np.argmax(axis))
        if axis[k] > 1e-9:
            axis = a[:, k] / axis[k]
        return axis / max(float(np.linalg.norm(axis)), 1e-12) * theta
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return w / (2.0 * np.sin(theta)) * theta


def _get(obj, name):
    v = getattr(obj, name, None)
    return None if v is None else np.asarray(v).tolist()


def _takes_arm(fn) -> bool:
    try:
        import inspect  # noqa: PLC0415

        return len(inspect.signature(fn).parameters) >= 1
    except Exception:
        return False
