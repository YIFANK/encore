"""Fair-API client: the ONLY thing a c2 policy program may touch.

Runs in its own process (`python -I`, scrubbed env, sandbox cwd), connected to
the fair_run.py server over a unix socket. The simulator, the GT judge, and the
benchmark files live in the SERVER process — there is no object in this
process from which ground truth can be reached, which is the structural
guarantee the c1 ProgramApi lacked (its `api.robot` exposed `_sim()`,
`gt_poses()` and `env`).

What the program receives (the whole allowed surface):
  - RGB + aligned depth + camera intrinsics/extrinsics  (api.capture)
  - end-effector pose + tool rotation + gripper state + joints (proprioception)
  - the task instruction sentence                      (api.instruction)
  - motion/gripper commands and their residuals        (api.move / api.grip)
  - its own log                                        (api.log)
  - the episode-over flag                              (api.done)

Additionally a Python audit hook denies file reads that match benchmark-asset
patterns (.bddl/.xml/.hdf5/init_states/gt_trace/...). That hook is a tripwire
for honest mistakes, not the security boundary — the boundary is that this
process simply has nothing privileged in it.

Protocol: 4-byte big-endian length + JSON; numpy arrays ride behind the JSON
as raw bytes, declared in-order in the header's "arrays" list.
"""
from __future__ import annotations

import json
import re
import socket
import struct
import sys
import time

import numpy as np

# ---------------------------------------------------------------------------
# wire protocol (mirrored in tools/fair_run.py — keep in sync)

def _send(sock: socket.socket, obj: dict, arrays: list[np.ndarray] | None = None) -> None:
    arrays = arrays or []
    obj = dict(obj)
    obj["arrays"] = [{"shape": list(a.shape), "dtype": str(a.dtype)} for a in arrays]
    head = json.dumps(obj).encode()
    sock.sendall(struct.pack(">I", len(head)) + head)
    for a in arrays:
        sock.sendall(np.ascontiguousarray(a).tobytes())


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("fair-api server closed the connection")
        buf += chunk
    return buf


def _recv(sock: socket.socket) -> tuple[dict, list[np.ndarray]]:
    (n,) = struct.unpack(">I", _recv_exact(sock, 4))
    obj = json.loads(_recv_exact(sock, n))
    arrays = []
    for spec in obj.get("arrays", []):
        nbytes = int(np.prod(spec["shape"]) or 1) * np.dtype(spec["dtype"]).itemsize
        arrays.append(np.frombuffer(_recv_exact(sock, nbytes),
                                    dtype=spec["dtype"]).reshape(spec["shape"]).copy())
    return obj, arrays


# ---------------------------------------------------------------------------
# the frame a program sees — deproject math is copied VERBATIM from
# heron.types.Frame.deproject (median 5x5 window over aligned depth) so fair
# runs stay numerically identical to the c1 harness.

class FairFrame:
    def __init__(self, camera: str, rgb: np.ndarray, depth: np.ndarray,
                 intrinsics: np.ndarray, t_base_cam: np.ndarray,
                 h_pixel_world=None, plane_z=None, proj=None) -> None:
        self.camera = camera
        self.rgb = rgb                # HxWx3 uint8
        self.depth = depth            # HxW float32 meters; all-NaN on a
                                      # depth-free rig
        self.intrinsics = intrinsics  # 3x3 K
        self.t_base_cam = t_base_cam  # 4x4 camera->base
        # Depth-free rigs carry a table-plane homography (exact for anything
        # resting on the table) and a DLT projection for off-plane points.
        self.h_pixel_world = (None if h_pixel_world is None
                              else np.asarray(h_pixel_world, float))
        self.plane_z = None if plane_z is None else float(plane_z)
        self.proj = None if proj is None else np.asarray(proj, float)
        self.t = time.time()

    def deproject_plane(self, u: int, v: int):
        """Pixel -> xyz on the table plane, or None without a homography."""
        if self.h_pixel_world is None:
            return None
        p = self.h_pixel_world @ np.array([float(u), float(v), 1.0])
        if abs(p[2]) < 1e-9:
            return None
        xy = p[:2] / p[2]
        return np.array([xy[0], xy[1], float(self.plane_z or 0.0)])

    def deproject(self, u: int, v: int):
        """Pixel -> xyz in robot base frame, or None where depth is invalid."""
        if self.depth is None or self.intrinsics is None or self.t_base_cam is None:
            return self.deproject_plane(u, v)
        h, w = self.depth.shape[:2]
        u = int(np.clip(u, 0, w - 1))
        v = int(np.clip(v, 0, h - 1))
        window = self.depth[max(0, v - 2): v + 3, max(0, u - 2): u + 3]
        valid = window[np.isfinite(window) & (window > 0)]
        if valid.size == 0:
            # No usable depth here: fall back to the table plane, which is
            # exact for objects resting on it and None otherwise.
            return self.deproject_plane(u, v)
        z = float(np.median(valid))
        fx, fy = self.intrinsics[0, 0], self.intrinsics[1, 1]
        cx, cy = self.intrinsics[0, 2], self.intrinsics[1, 2]
        p_cam = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        return (self.t_base_cam @ p_cam)[:3]


def _arm(arm) -> dict:
    """Wire kwarg for an arm-addressed call; empty when the caller said nothing."""
    return {} if arm is None else {"arm": str(arm)}


class FairApi:
    """The program-facing API. No `.robot`, no `.env`, no `.sim` — ever."""

    def __init__(self, sock: socket.socket, hello: dict) -> None:
        self._sock = sock
        self.cameras = list(hello.get("cameras", []))
        # Sides this cell may command. Empty on single-arm backends, which
        # ignore the `arm` argument entirely.
        self.arms = list(hello.get("arms", []))
        self._instruction = str(hello.get("instruction", ""))
        self._done = False

    def _call(self, op: str, **kw) -> tuple[dict, list[np.ndarray]]:
        _send(self._sock, {"op": op, **kw})
        rep, arrays = _recv(self._sock)
        if rep.get("error"):
            raise RuntimeError(f"fair-api {op}: {rep['error']}")
        self._done = bool(rep.get("done", self._done))
        return rep, arrays

    # -- senses --------------------------------------------------------------
    def capture(self, camera: str = "cam_high") -> FairFrame:
        rep, (rgb, depth, intr, t_bc) = self._call("capture", camera=camera)
        return FairFrame(camera, rgb, depth, intr, t_bc,
                         h_pixel_world=rep.get("h_pixel_world"),
                         plane_z=rep.get("plane_z"),
                         proj=rep.get("proj"))

    def eef(self, arm: str | None = None) -> np.ndarray:
        rep, _ = self._call("eef", **_arm(arm))
        return np.asarray(rep["xyz"], float)

    def tool_rotation(self, arm: str | None = None) -> np.ndarray:
        rep, (r,) = self._call("tool_rotation", **_arm(arm))
        return r

    def gripper(self, arm: str | None = None) -> dict:
        rep, _ = self._call("gripper", **_arm(arm))
        return {"width_m": float(rep["width_m"]), "effort": float(rep["effort"])}

    def ground(self, query: str, camera: str = "cam_high"):
        """World position of one described object via the coordinator's
        grounding service, or None: {"xyz": [x, y, z], "px": [u, v]}.
        Lighting-robust (absolute colour thresholds are statements about one
        evening's bulbs); costs one call from a per-episode model budget."""
        rep, _ = self._call("ground", query=str(query), camera=str(camera))
        return rep.get("hit")

    def vqa(self, question: str, camera: str = "cam_high") -> dict:
        """One visual question about a fresh frame:
        {"answer", "confidence", "note"}. Same model budget as ground()."""
        rep, _ = self._call("vqa", question=str(question), camera=str(camera))
        return rep.get("result") or {}

    def proprio(self) -> dict:
        rep, _ = self._call("proprio")
        return rep["proprio"]

    def instruction(self) -> str:
        return self._instruction

    # -- actions -------------------------------------------------------------
    def sam3(self, query: str, camera: str = "cam_high") -> list[dict]:
        """Text-prompted segmentation (SAM3): 'the orange pen' -> masks.
        Returns [{mask (H,W bool), box, score}] best-first."""
        import base64 as _b64  # noqa: PLC0415
        rep, _ = self._call("sam3", query=str(query), camera=camera)
        out = []
        for m in rep.get("results", []):
            h, w = m["shape"]
            bits = np.unpackbits(np.frombuffer(_b64.b64decode(m["mask_b64"]),
                                               dtype=np.uint8))[: h * w]
            out.append({"mask": bits.reshape(h, w).astype(bool),
                        "box": m["box"], "score": m["score"]})
        return out

    def move(self, xyz, rotation=None, seconds: float = 2.0,
             arm: str | None = None) -> float:
        rep, _ = self._call(
            "move", xyz=[float(v) for v in np.asarray(xyz, float)],
            rotation=(None if rotation is None
                      else np.asarray(rotation, float).reshape(3, 3).tolist()),
            seconds=float(seconds), **_arm(arm))
        return float(rep.get("residual", 0.0))

    def drag(self, xyz, seconds: float = 2.0, arm: str | None = None) -> float:
        """One continuous straight-line drag at the current orientation —
        the primitive for table-contact strokes. 30 cm in one command."""
        rep, _ = self._call("drag", xyz=[float(v) for v in np.asarray(xyz, float)],
                            seconds=float(seconds), **_arm(arm))
        return float(rep.get("residual", 0.0))

    def move_path(self, points, rotation=None, seconds: float = 4.0,
                  arm: str | None = None) -> float:
        """Smooth motion through several waypoints without stopping at each
        (use for transports; the discrete move() chain judders). `rotation`:
        one 3x3 matrix held throughout, or a list with one per waypoint."""
        if rotation is None:
            rot = None
        else:
            r = np.asarray(rotation, float)
            rot = (r.reshape(3, 3).tolist() if r.ndim <= 2
                   else [np.asarray(m, float).reshape(3, 3).tolist() for m in r])
        rep, _ = self._call(
            "move_path",
            points=[[float(v) for v in np.asarray(p, float)] for p in points],
            rotation=rot, seconds=float(seconds), **_arm(arm))
        return float(rep.get("residual", 0.0))

    def sam(self, camera: str = "cam_high", box=None, point=None,
            mode: str = "propose", max_masks: int = 24) -> list[dict]:
        """SAM over a fresh capture (rig only). Candidates carry box,
        centroid_px, pixels, score, and the centroid's world position; no
        masks on the wire. Absent on sim fair servers."""
        rep, _ = self._call("sam", camera=str(camera),
                            box=(None if box is None
                                 else [float(v) for v in box]),
                            point=(None if point is None
                                   else [float(v) for v in point]),
                            mode=str(mode), max_masks=int(max_masks))
        return rep.get("candidates", [])

    def pick_at(self, xyz, hover_m: float = 0.08,
                arm: str | None = None) -> dict:
        """Guarded top-down grasp at a world point (rig only): open, hover,
        descend, force-close, width check, lift. -> {'held', 'width_m'}."""
        rep, _ = self._call("pick_at", xyz=[float(v) for v in xyz],
                            hover_m=float(hover_m), **_arm(arm))
        return {"held": bool(rep.get("held")),
                "width_m": float(rep.get("width_m", 0.0))}

    def place_at(self, xyz, hover_m: float = 0.12,
                 arm: str | None = None) -> dict:
        """Guarded release at a world point (rig only)."""
        rep, _ = self._call("place_at", xyz=[float(v) for v in xyz],
                            hover_m=float(hover_m), **_arm(arm))
        return {"released": bool(rep.get("released"))}

    def grip(self, width_m: float, arm: str | None = None) -> None:
        self._call("grip", width_m=float(width_m), **_arm(arm))

    def settle(self, seconds: float = 0.5) -> None:
        self._call("settle", seconds=float(seconds))

    def act(self, delta) -> None:
        """v1.2: execute ONE raw controller action (7-dim single-arm:
        [dx, dy, dz, dax, day, daz, grip]); the environment advances exactly
        one control step per call. Robosuite backends only."""
        self._call("act", delta=[float(v) for v in np.asarray(delta, float)])

    def log(self, msg: str) -> None:
        self._call("log", msg=str(msg)[:2000])

    @property
    def done(self) -> bool:
        return self._done


# ---------------------------------------------------------------------------
# file-access tripwire (PEP 578). Denies READS of benchmark/privileged assets.

_DENY = re.compile(
    r"(\.bddl$|\.hdf5$|\.h5$|\.urdf$|bddl_files|init_files|init_states|"
    r"pruned_init|gt_trace|LIBERO|libero/libero|demo.*\.hdf5|\.xml$)",
    re.IGNORECASE)


def _install_guard(sandbox: str) -> None:
    def hook(event: str, args: tuple) -> None:
        if event != "open":
            return
        path, mode = str(args[0]), str(args[1] or "r")
        if any(m in mode for m in ("w", "a", "x", "+")) and path.startswith(sandbox):
            return                      # writes inside the sandbox are fine
        if _DENY.search(path):
            raise PermissionError(
                f"fair-api guard: reading {path!r} is forbidden inside a policy "
                f"program (benchmark/privileged asset)")
    sys.addaudithook(hook)


# ---------------------------------------------------------------------------

def main() -> int:
    socket_path, program_path, sandbox = sys.argv[1], sys.argv[2], sys.argv[3]

    # Load the program source BEFORE the guard (its own file may sit anywhere),
    # execute it AFTER, so every read it performs at runtime is policed.
    source = open(program_path).read()
    _install_guard(sandbox)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(socket_path)
    hello, _ = _recv(sock)
    api = FairApi(sock, hello)

    note, err = "", None
    try:
        mod: dict = {"__name__": "fair_program", "__file__": program_path}
        exec(compile(source, program_path, "exec"), mod)   # noqa: S102
        note = mod["run"](api)
    except BaseException as e:  # report, then let the server keep the verdict
        err = f"{type(e).__name__}: {e}"
        import traceback
        traceback.print_exc(file=sys.stderr)
    finally:
        try:
            _send(sock, {"op": "end", "note": str(note)[:200], "error": err})
            _recv(sock)          # wait for the ack so the note outruns close()
        except Exception:
            pass
        sock.close()
    return 0 if err is None else 1


if __name__ == "__main__":
    raise SystemExit(main())
