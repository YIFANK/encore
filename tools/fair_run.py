"""Information-fair harness for c2: run a policy program against LIBERO with
the simulator and GT judge ISOLATED in this server process.

Differences from tools/fewshot_run.py (the c1 harness), by design:

  1. TWO PROCESSES. The program runs under `python -I` in a scrubbed-env
     sandbox subprocess speaking RPC over a unix socket (tools/fair_client.py).
     There is no `api.robot`/`env`/`sim` to reach — the c1 audit found that
     surface exposed `_sim()`/`gt_poses()` to any curious program.
  2. SEALED SPLITS (ASPIRE protocol). `--split debug` may only run seeds
     51-65; `--split eval` runs EXACTLY seeds 1-50, blind: all artifacts land
     in a private staging dir and publish to --out only after the last episode,
     so nobody iterates on a half-finished eval.
  3. NO GT SIDE-CARS in the program's namespace. gt traces are written only
     when --gt-dir points at a coordinator-private archive (offline judging).
  4. STATIC REFUSALS. The program source is token-scanned (gt_pose/env.sim/
     h5py/.bddl/...) before it runs; eval mode additionally requires a
     PROVENANCE dict declaring every calibrated constant's source as allowed.

    .venv/bin/python tools/fair_run.py program --seed-episodes \
        --bddl <task.bddl> --program packs/c2_<cell>/program.py \
        --split debug --episode-list 51,53,55 --out results/c2_<cell>_v1
"""
from __future__ import annotations

import argparse
import ast
import json
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import traceback
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

DEBUG_SEEDS = set(range(51, 66))     # ASPIRE debug/learn band
EVAL_SEEDS = list(range(1, 51))      # ASPIRE held-out band, run as a whole

FORBIDDEN_TOKENS = re.compile(
    r"(gt_pose|gt_trace|gt_joints|\b_sim\b|env\.sim|sim\.data|body_xpos|"
    r"site_xpos|geom_xpos|\.bddl|bddl_files|init_states|pruned_init|h5py|"
    r"\.hdf5|api\.robot|self\.robot\b|check_success|OffScreenRenderEnv)")

# v1.1.1: api.done is additionally banned in EVAL programs. Even as
# terminated-only it leaks on LIBERO — the env ends the episode when the
# predicate fires, so early termination is itself a success signal. Official
# programs must be provably oracle-free: fixed sequences + own sensors.
# AST-based: docstrings/comments that merely MENTION api.done (rework
# provenance notes) must not trip the gate — only real attribute reads do.
def _reads_done(src: str) -> bool:
    try:
        tree = ast.parse(src)
    except SyntaxError:
        return True   # unparseable -> refuse conservatively
    return any(isinstance(n, ast.Attribute) and n.attr == "done"
               for n in ast.walk(tree))


# ---------------------------------------------------------------------------
# wire protocol (mirrored in tools/fair_client.py — keep in sync)

def _send(conn: socket.socket, obj: dict, arrays: list[np.ndarray] | None = None) -> None:
    arrays = arrays or []
    obj = dict(obj)
    obj["arrays"] = [{"shape": list(a.shape), "dtype": str(a.dtype)} for a in arrays]
    head = json.dumps(obj).encode()
    conn.sendall(struct.pack(">I", len(head)) + head)
    for a in arrays:
        conn.sendall(np.ascontiguousarray(a).tobytes())


def _recv_exact(conn: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("program process closed the connection")
        buf += chunk
    return buf


def _recv(conn: socket.socket) -> dict:
    (n,) = struct.unpack(">I", _recv_exact(conn, 4))
    obj = json.loads(_recv_exact(conn, n))
    for spec in obj.get("arrays", []):   # drain (client never sends arrays today)
        _recv_exact(conn, int(np.prod(spec["shape"]) or 1) * np.dtype(spec["dtype"]).itemsize)
    return obj


# ---------------------------------------------------------------------------
# program vetting

def scan_program(path: Path, split: str) -> None:
    src = path.read_text()
    hits = sorted({m.group(0) for m in FORBIDDEN_TOKENS.finditer(src)})
    if hits:
        raise SystemExit(f"[fair] REFUSED: {path} contains forbidden tokens {hits}")
    if split == "eval":
        if _reads_done(src):
            raise SystemExit("[fair] REFUSED: eval programs may not read "
                             "api.done (LIBERO terminates on success, so the "
                             "termination flag is an indirect success signal)")
        prov = _provenance(src)
        if prov is None:
            raise SystemExit(
                "[fair] REFUSED: eval requires a top-level PROVENANCE dict "
                "({CONST: {'source': ..., 'allowed': True}}) in the program")
        bad = [k for k, v in prov.items()
               if not (isinstance(v, dict) and v.get("allowed") is True and v.get("source"))]
        if bad:
            raise SystemExit(f"[fair] REFUSED: PROVENANCE entries not allowed/sourced: {bad}")


def _provenance(src: str):
    try:
        tree = ast.parse(src)
    except SyntaxError:
        return None
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == "PROVENANCE":
                    try:
                        return ast.literal_eval(node.value)
                    except Exception:
                        return None
    return None


# ---------------------------------------------------------------------------
# robot construction — LIBERO (same env path as fewshot_run/eval_libero_pro) or
# robosuite (episode = benchmark-official env seed, ASPIRE-style)

def make_robot(args, episode: int):
    if getattr(args, "rig", False) or getattr(args, "twin", False):
        from heron.config import HeronConfig
        from heron.robot.rig_fair import RigRobot

        cfg = HeronConfig.load(args.config)
        card = None
        if args.cards and args.card:
            sys.path.insert(0, str(ROOT))
            from tools.rig_protocol import load_card

            card = load_card(args.cards, args.card)
        robot = RigRobot(cfg, episode=episode, card=card,
                         language=args.language or "",
                         max_commands=int(getattr(args, "max_commands", 400)),
                         backend="twin" if getattr(args, "twin", False) else "real",
                         scene=getattr(args, "scene", "") or None)
        robot.start_recording(1)   # rig: every capture is a saved frame (2026-09-02)
        return robot

    if args.robosuite:
        from heron.robot.robosuite_env import RobosuiteRobot

        robot = RobosuiteRobot(args.robosuite, episode,
                               horizon=int(args.horizon or 500))
        if args.language:
            robot.task_language = args.language
        robot.start_recording(8)   # RobosuiteRobot defaults to no film strip
        return robot

    from heron.config import HeronConfig
    from heron.robot.libero import LiberoRobot

    cfg = HeronConfig.load(args.config)
    cfg.libero.bddl_file = args.bddl
    cfg.libero.language = args.language or None
    cfg.libero.init_states_file = args.init_states or None
    if args.seed_episodes:
        cfg.libero.init_states_file = None
        cfg.libero.seed = episode
    cfg.libero.episode = episode
    if args.horizon:
        cfg.libero.horizon = int(args.horizon)
    return LiberoRobot(cfg)


# ---------------------------------------------------------------------------
# per-episode serve loop

def serve_episode(robot, conn: socket.socket, log_path: Path, deadline: float) -> dict:
    camera_map_keys = list(robot.cameras)
    log = log_path.open("a")
    _send(conn, {"op": "hello", "cameras": camera_map_keys,
                 "arms": list(getattr(robot, "arms", []) or []),
                 "instruction": robot.task_language,
                 "protocol": "fair-v1.1 (api.done = episode termination only; "
                             "no runtime success signal)"})
    end: dict = {}
    while True:
        conn.settimeout(max(1.0, deadline - time.time()))
        try:
            req = _recv(conn)
        except (ConnectionError, socket.timeout) as e:
            end = {"error": f"{type(e).__name__}: {e}"}
            break
        op = req.get("op")
        # Bimanual cells address a side per call; single-arm programs omit it
        # and the backend resolves the default arm.
        arm = req.get("arm", "arm")
        rep: dict = {}
        arrays: list[np.ndarray] = []
        try:
            if op == "end":
                end = {"note": req.get("note", ""), "error": req.get("error")}
                try:
                    _send(conn, {"ok": True})
                except Exception:
                    pass
                break
            elif op == "capture":
                cam = str(req["camera"])
                if cam not in camera_map_keys:
                    raise ValueError(f"unknown camera {cam!r} (have {camera_map_keys})")
                f = robot.capture(cam)
                # A depth-free rig (macOS cannot hand librealsense the D405s)
                # sends NaN depth plus the table-plane homography, and the
                # client deprojects through that instead.
                depth = getattr(f, "depth", None)
                if depth is None:
                    rgb_hw = np.asarray(f.rgb).shape[:2]
                    depth = np.full(rgb_hw, np.nan, dtype=np.float32)
                intr = getattr(f, "intrinsics", None)
                t_bc = getattr(f, "t_base_cam", None)
                arrays = [np.asarray(f.rgb),
                          np.asarray(depth, dtype=np.float32),
                          np.asarray(intr if intr is not None else np.zeros((3, 3)), float),
                          np.asarray(t_bc if t_bc is not None else np.zeros((4, 4)), float)]
                for name in ("h_pixel_world", "proj"):
                    v = getattr(f, name, None)
                    if v is not None:
                        rep[name] = np.asarray(v, float).tolist()
                pz = getattr(f, "plane_z", None)
                if pz is not None:
                    rep["plane_z"] = float(pz)
            elif op == "ground":
                rep["hit"] = robot.ground(str(req.get("query", "")),
                                          str(req.get("camera", "cam_high")))
            elif op == "vqa":
                rep["result"] = robot.vqa(str(req.get("question", "")),
                                          str(req.get("camera", "cam_high")))
            elif op == "eef":
                rep["xyz"] = [float(v) for v in robot.get_cartesian(arm)]
            elif op == "tool_rotation":
                arrays = [np.asarray(robot.get_tool_rotation(arm), float)]
            elif op == "gripper":
                rep.update(robot.get_gripper(arm))
            elif op == "proprio":
                obs = robot.obs
                rep["proprio"] = {
                    k: [round(float(v), 5) for v in np.atleast_1d(obs[k])]
                    for k in ("robot0_joint_pos", "robot0_eef_pos",
                              "robot0_eef_quat", "robot0_gripper_qpos")
                    if k in obs}
            elif op == "move":
                xyz = np.asarray(req["xyz"], float)
                rot = req.get("rotation")
                if rot is None:
                    robot.move_cartesian(arm, xyz, seconds=float(req.get("seconds", 2.0)))
                else:
                    robot.move_pose(arm, xyz, rotation=np.asarray(rot, float),
                                    seconds=float(req.get("seconds", 3.0)))
                rep["residual"] = robot.last_move_residual
            elif op == "drag" and hasattr(robot, "drag"):
                robot.drag(arm, np.asarray(req["xyz"], float),
                           seconds=float(req.get("seconds", 2.0)))
                rep["residual"] = robot.last_move_residual
            elif op == "move_path":
                pts = [np.asarray(p, float) for p in req["points"]]
                rot = req.get("rotation")
                robot.move_path(
                    arm, pts,
                    rotation=(None if rot is None else np.asarray(rot, float)),
                    seconds=float(req.get("seconds", 4.0)))
                rep["residual"] = robot.last_move_residual
            elif op == "sam3" and hasattr(robot, "sam3"):
                import base64 as _b64
                hits = robot.sam3(str(req["query"]), req.get("camera", "cam_high"))
                rep["results"] = [
                    {"mask_b64": _b64.b64encode(np.packbits(h["mask"])).decode(),
                     "shape": list(h["mask"].shape),
                     "box": h["box"], "score": h["score"]} for h in hits]
            elif op == "sam" and hasattr(robot, "sam"):
                rep["candidates"] = robot.sam(
                    str(req.get("camera", "cam_high")),
                    box=req.get("box"), point=req.get("point"),
                    mode=str(req.get("mode", "propose")),
                    max_masks=int(req.get("max_masks", 24)))
            elif op == "pick_at" and hasattr(robot, "pick_at"):
                rep.update(robot.pick_at(arm, req["xyz"],
                                         hover_m=float(req.get("hover_m", 0.08))))
            elif op == "place_at" and hasattr(robot, "place_at"):
                rep.update(robot.place_at(arm, req["xyz"],
                                          hover_m=float(req.get("hover_m", 0.12))))
            elif op == "grip":
                robot.set_gripper(arm, float(req["width_m"]))
            elif op == "settle":
                robot.settle(min(float(req.get("seconds", 0.5)), 10.0))
            elif op == "act":
                # v1.2: one raw controller step, verbatim.  Only backends
                # that implement step_raw support it; others refuse.
                if not hasattr(robot, "step_raw"):
                    rep["error"] = "act unsupported on this backend"
                else:
                    d = np.asarray(req["delta"], float)
                    if d.shape not in ((7,), (14,)):
                        rep["error"] = f"act: bad delta shape {d.shape}"
                    else:
                        robot.step_raw(d)
            elif op == "log":
                log.write(json.dumps({"t": time.time(), "msg": str(req.get("msg", ""))}) + "\n")
                log.flush()
            else:
                raise ValueError(f"unknown op {op!r}")
        except Exception as e:
            rep = {"error": f"{type(e).__name__}: {e}"}
            arrays = []
        # fair-v1.1: done reflects EPISODE TERMINATION ONLY. The benchmark
        # success bit is never visible at runtime — a per-step queryable
        # predicate is an online evaluator oracle no real robot has (and no
        # ASPIRE/CaP-X policy gets). Success is judged post-episode.
        rep["done"] = bool(robot.terminated)
        try:
            _send(conn, rep, arrays)
        except (BrokenPipeError, ConnectionError):
            end = {"error": "program process died mid-reply"}
            break
        if time.time() > deadline:
            end = {"error": "episode wall-clock budget exhausted"}
            break
    log.close()
    return end


def run_episode(args, ep: int, out: Path) -> dict:
    rec = {"episode": ep, "program": str(args.program), "t0": time.time()}
    tmp_root = Path(args.tmp_root)
    tmp_root.mkdir(parents=True, exist_ok=True)
    sandbox = Path(tempfile.mkdtemp(prefix=f"fair_ep{ep}_", dir=tmp_root))
    sock_path = str(sandbox / "api.sock")
    robot, proc, srv = None, None, None
    try:
        robot = make_robot(args, ep)
        if args.gt_dir:   # coordinator-private judging archive, NOT agent-visible
            gt = Path(args.gt_dir)
            gt.mkdir(parents=True, exist_ok=True)
            os.chmod(gt, 0o700)
            robot.start_gt_trace(gt / f"gt_trace_ep{ep}.jsonl")
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(sock_path)
        srv.listen(1)
        srv.settimeout(60.0)
        env = {"PATH": "/usr/bin:/bin", "HOME": str(sandbox), "TMPDIR": str(sandbox)}
        with (out / f"program_ep{ep}.stderr").open("w") as errf:
            proc = subprocess.Popen(
                [sys.executable, "-I", str(ROOT / "tools/fair_client.py"),
                 sock_path, str(args.program), str(sandbox)],
                cwd=str(sandbox), env=env, stdout=errf, stderr=errf)
            conn, _ = srv.accept()
            deadline = time.time() + float(args.ep_timeout)
            end = serve_episode(robot, conn, out / f"program_ep{ep}.log", deadline)
            conn.close()
            proc.wait(timeout=30)
        robot.settle(1.0)
        rec.update({"benchmark_success": bool(robot.task_success),
                    "note": str(end.get("note", ""))[:200],
                    "sim_steps": robot.sim_steps})
        if end.get("error"):
            rec["program_error"] = str(end["error"])[:400]
    except Exception as e:
        rec.update({"benchmark_success": False,
                    "error": f"{type(e).__name__}: {e}",
                    "traceback": traceback.format_exc()[-1200:]})
    finally:
        if proc is not None and proc.poll() is None:
            proc.kill()
        if srv is not None:
            srv.close()
        if robot is not None:
            try:
                # Rig episodes have no runtime success bit; "_fail" in every
                # filename read as a verdict that had not happened yet.
                tag = ("" if getattr(args, "rig", False)
                       else ("_ok" if rec.get("benchmark_success") else "_fail"))
                robot.save_gif(out / f"ep{ep}{tag}.gif")
            except Exception:
                pass
            try:
                robot.shutdown()
            except Exception:
                pass
        shutil.rmtree(sandbox, ignore_errors=True)
    rec["duration_s"] = round(time.time() - rec.pop("t0"), 1)
    return rec


# ---------------------------------------------------------------------------

def cmd_program(args) -> int:
    eps = ([int(x) for x in args.episode_list.split(",") if x.strip()]
           if args.episode_list else [])
    if args.split == "debug":
        illegal = sorted(set(eps) - DEBUG_SEEDS)
        if not eps or illegal:
            raise SystemExit(f"[fair] debug split runs only seeds 51-65 "
                             f"(got {illegal or 'nothing'})")
    elif args.split == "eval":
        if eps and eps != EVAL_SEEDS:
            raise SystemExit("[fair] eval split runs EXACTLY seeds 1-50 "
                             "(omit --episode-list or pass the full band)")
        eps = EVAL_SEEDS

    # The client subprocess runs with cwd inside a private sandbox — a relative
    # --program silently dies there (FileNotFoundError -> accept() timeout).
    args.program = str(Path(args.program).resolve())
    scan_program(Path(args.program), args.split)

    final_out = Path(args.out)
    if final_out.exists():
        raise SystemExit(f"[fair] {final_out} already exists — an eval/debug dir "
                         f"is written once, never appended")
    # Blind eval: stage privately, publish only when the whole band is done.
    out = (final_out.parent / f".{final_out.name}.staging"
           if args.split == "eval" else final_out)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    if args.split == "eval":
        # Best-available blindness hardening on a single-user box: 0700 the
        # staging dir. This is PROCEDURAL, not structural — every process of
        # this user can still read it; the enforceable controls are
        # publish-at-end, a coordinator no-read rule, and transcript audits.
        os.chmod(out, 0o700)
    shutil.copy(args.program, out / "program_archived.py")

    results = out / "results.jsonl"
    n_succ = 0
    for ep in eps:
        rec = run_episode(args, ep, out)
        n_succ += bool(rec.get("benchmark_success"))
        with results.open("a") as f:
            f.write(json.dumps(rec) + "\n")
        if args.split == "eval":
            print(f"[fair-eval] ep{ep} done ({eps.index(ep) + 1}/{len(eps)})", flush=True)
        else:
            # On the rig the success bit does not exist at runtime by design —
            # the judge decides post-episode. Printing "False" here read as a
            # verdict to the operator watching the console.
            shown = ("pending-judge" if getattr(args, "rig", False)
                     else rec.get("benchmark_success"))
            print(f"[fair] ep{ep}: success={shown} "
                  f"{rec.get('program_error', rec.get('error', ''))}", flush=True)

    if args.split == "eval":
        out.rename(final_out)
        print(f"[fair-eval] PUBLISHED {final_out}: {n_succ}/{len(eps)}", flush=True)
    else:
        print(f"[fair] {final_out}: {n_succ}/{len(eps)}", flush=True)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    p = sub.add_parser("program")
    p.add_argument("--config", default=str(ROOT / "configs/libero.yaml"))
    p.add_argument("--bddl", default="")
    p.add_argument("--twin", action="store_true",
                   help="run against the MuJoCo twin of the rig instead of the "
                        "hardware: same URDF, same kinematics module, same "
                        "calibration, same fair API — so one policy program "
                        "runs unchanged in both and the reality gap becomes a "
                        "readable diff")
    p.add_argument("--scene", default="",
                   help="MuJoCo scene xml for --twin (default: the rig scene)")
    p.add_argument("--rig", action="store_true",
                   help="run on the physical Trossen rig (protocol v1.3)")
    p.add_argument("--cards", default="", help="pose-card directory (rig)")
    p.add_argument("--card", default="", help="pose card id for this trial (rig)")
    p.add_argument("--max-commands", type=int, default=400,
                   help="per-episode motion-command budget on the rig")
    p.add_argument("--robosuite", default="",
                   help="robomimic hdf5 path (optionally ::EnvOverride): run "
                        "robosuite instead of LIBERO; episode = env seed")
    p.add_argument("--init-states", default="")
    p.add_argument("--language", default="",
                   help="the intent sentence served to api.instruction() "
                        "(re-authored cells: the coordinator sets this)")
    p.add_argument("--seed-episodes", action="store_true",
                   help="episode number IS the env seed (ASPIRE protocol)")
    p.add_argument("--horizon", type=int, default=0)
    p.add_argument("--program", required=True)
    p.add_argument("--split", required=True, choices=["debug", "eval"],
                   help="debug = seeds 51-65 only; eval = seeds 1-50, blind")
    p.add_argument("--episode-list", default="")
    p.add_argument("--out", required=True)
    p.add_argument("--gt-dir", default="",
                   help="coordinator-PRIVATE dir for gt traces (offline judging); "
                        "never point this inside an agent-visible namespace")
    p.add_argument("--ep-timeout", type=float, default=900.0,
                   help="wall-clock budget per episode (s)")
    p.add_argument("--tmp-root", default="/mnt/data/YifanKang/tmp",
                   help="sandbox root (NOT /tmp: the cluster / is ~99%% full)")
    args = ap.parse_args()
    if not args.bddl and not args.robosuite:
        ap.error("one of --bddl or --robosuite is required")
    return cmd_program(args)


if __name__ == "__main__":
    raise SystemExit(main())
