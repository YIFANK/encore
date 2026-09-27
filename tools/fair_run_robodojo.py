#!/usr/bin/env python3
"""Fair-harness runner for RoboDojo (Isaac Sim, ARX X5 bimanual).

Same contract as tools/fair_run.py: a sandboxed policy program speaks the fair
API over a unix socket; the coordinator runs a fixed band of initial states;
the benchmark's own judge scores each episode after it ends and the program
never sees the bit.  The differences are all on the simulator side: RoboDojo
owns the sim and pulls actions from a WebSocket policy server, so this runner
hosts that server in-process (heron.robot.robodojo_env.RoboDojoBridge) and
launches RoboDojo's own eval client against it.

Bands (fair_run conventions kept so briefs/tooling stay identical):
  debug  episodes 51-65  ->  layouts 0-14 of the official seed dir --dev-pool  (default "2")
  eval   episodes 1-50   ->  layouts 0-49 of the official seed dir --eval-pool (default "0")
The chosen layouts are copied, renumbered in episode order, into
Assets/Eval_Layout/RoboDojo/<env-cfg>/<band>/ so RoboDojo's SeedManager runs
exactly that list (EVAL_NUM = its length) and nothing else.

Run INSIDE the RoboDojo conda env (needs websockets/msgpack + Isaac client):
  /mnt/data/YifanKang/robodojo/home/miniconda3/envs/RoboDojo/bin/python \
      tools/fair_run_robodojo.py --task put_bottles_into_dustbin \
      --program packs/rd_put_bottles_into_dustbin_k3/program.py \
      --split debug --episode-list 51,53,55 --out results/fs_rd_bottles_v1 --gpu 5
"""
from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]          # Heron
sys.path.insert(0, str(ROOT))

RD_ROOT = Path(os.environ.get("ROBODOJO_ROOT", "/mnt/data/YifanKang/robodojo/RoboDojo"))
RD_HOME = Path(os.environ.get("ROBODOJO_HOME", "/mnt/data/YifanKang/robodojo/home"))
sys.path.insert(0, str(RD_ROOT / "XPolicyLab"))
sys.path.insert(0, str(RD_ROOT))

from tools.fair_run import DEBUG_SEEDS, EVAL_SEEDS, scan_program, serve_episode  # noqa: E402
from heron.robot.robodojo_env import RoboDojoBridge, RoboDojoRobot  # noqa: E402

POLICY_NAME = "Encore"
ADDITIONAL_INFO = "ckpt_name=encore,action_type=ee"

# The coordinator-side VLM (Gemini) is unreachable from the cluster directly; the
# runner's own process routes it through the reverse SOCKS tunnel from the Mac
# (ssh -R 7891). The simulator client must NOT inherit this (it only talks to
# 127.0.0.1), so launch_client strips it.
_PROXY = os.environ.get("FAIR_RD_PROXY", "socks5h://127.0.0.1:7891")
if _PROXY:
    os.environ.setdefault("HTTPS_PROXY", _PROXY)
    os.environ.setdefault("HTTP_PROXY", _PROXY)
    os.environ.setdefault("NO_PROXY", "127.0.0.1,localhost,.ppapi.ai")


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def make_band(task: str, env_cfg: str, split: str, eps: list[int], dev_pool: str, eval_pool: str) -> tuple[str, Path]:
    """Copy the requested official layouts into a private band dir, renumbered 0..n-1."""
    layouts = RD_ROOT / "Assets" / "Eval_Layout" / "RoboDojo"
    # A pool is a comma-separated list of official seed dirs, concatenated in
    # order: tasks with fewer than 50 layouts per seed (e.g. arrange_largest_number,
    # 30) draw the sealed band from seed 0 first, then seed 1.
    pools = [layouts / "arx_x5" / p.strip() for p in (dev_pool if split == "debug" else eval_pool).split(",")]
    for pdir in pools:
        if not pdir.is_dir():
            raise SystemExit(f"[fair-rd] no official layout dir {pdir}")
    pool_files: list[Path] = []
    for pdir in pools:
        fs = sorted(pdir.glob(f"{task}_*.json"), key=lambda q: int(q.stem.rsplit("_", 1)[-1]))
        pool_files += fs
    # RoboDojo's client parses --seed as an int and SeedManager uses str(seed) as the
    # layout dir name, so the band is an integer: 1xxxxxxx = debug, 2xxxxxxx = eval.
    # Unique per RUN (episode list + program path + pid): two cells of the same task
    # probing the same episodes concurrently must not share (and delete) a band dir.
    key = f"{','.join(map(str, eps))}|{os.environ.get('FAIR_RD_PROGRAM', '')}|{os.getpid()}|{time.time_ns()}"
    h = int(hashlib.md5(key.encode()).hexdigest()[:6], 16) % 10_000_000
    band = str((10_000_000 if split == "debug" else 20_000_000) + h)
    dst = layouts / env_cfg / band
    if dst.exists():
        shutil.rmtree(dst)
    dst.mkdir(parents=True)
    for k, ep in enumerate(eps):
        lid = ep - 51 if split == "debug" else ep - 1
        if lid >= len(pool_files):
            raise SystemExit(f"[fair-rd] only {len(pool_files)} layouts in pool for {task}; episode {ep} needs #{lid}")
        shutil.copy(pool_files[lid], dst / f"{task}_{k}.json")
        # Some tasks (imitate_sorting_sequence) drive a support arm from a recorded
        # trajectory stored NEXT TO the layout pool, Assets/Traj/RoboDojo/<task>/<seed>/<lid>.pkl,
        # and load it by the band name at reset. Without the mirror every episode dies with
        # "no observation" (rd1: both imitate_sorting workers, and the first sealed eval).
        traj_src = RD_ROOT / "Assets" / "Traj" / "RoboDojo" / task / pool_files[lid].parent.name / f"{pool_files[lid].stem.rsplit('_', 1)[-1]}.pkl"
        if traj_src.exists():
            traj_dst = RD_ROOT / "Assets" / "Traj" / "RoboDojo" / task / band
            traj_dst.mkdir(parents=True, exist_ok=True)
            shutil.copy(traj_src, traj_dst / f"{k}.pkl")
        # play_tic_tac_toe keeps its trajectories flat: Traj/<task>/<seed>_<lid>.pkl
        flat_src = RD_ROOT / "Assets" / "Traj" / "RoboDojo" / task / f"{pool_files[lid].parent.name}_{pool_files[lid].stem.rsplit('_', 1)[-1]}.pkl"
        if flat_src.exists():
            shutil.copy(flat_src, flat_src.with_name(f"{band}_{k}.pkl"))
    (dst / "_source.json").write_text(json.dumps({str(k): str(pool_files[(ep - 51 if split == 'debug' else ep - 1)]) for k, ep in enumerate(eps)}, indent=1))
    return band, dst


def start_policy_server(bridge: RoboDojoBridge, port: int) -> threading.Thread:
    from client_server.ws.model_server import PolicyServer, PolicyServerConfig  # noqa: PLC0415
    server = PolicyServer(bridge, PolicyServerConfig(host="127.0.0.1", port=int(port)))

    def run():
        asyncio.run(server.serve_forever())

    t = threading.Thread(target=run, name="policy-server", daemon=True)
    t.start()
    return t


def launch_client(args, band: str, port: int, run_id: str, n_eps: int, out: Path) -> subprocess.Popen:
    env = dict(os.environ)
    env.update({
        "HOME": str(RD_HOME), "XDG_CACHE_HOME": str(RD_HOME / ".cache"), "TMPDIR": str(RD_HOME / "tmp"),
        "OMNI_KIT_ACCEPT_EULA": "YES", "ACCEPT_EULA": "Y", "PRIVACY_CONSENT": "Y",
        "EVAL_NUM": str(n_eps), "ROBODOJO_RUN_ID": run_id, "ROBODOJO_MAX_BASH_RETRIES": "2",
        "PYTHONUNBUFFERED": "1", "TERM": "xterm-256color",
    })
    for k in ("PYTHONPATH", "HTTPS_PROXY", "HTTP_PROXY", "ALL_PROXY", "https_proxy", "http_proxy", "all_proxy"):
        env.pop(k, None)
    cmd = (f"source {RD_HOME}/miniconda3/bin/activate RoboDojo && cd {RD_ROOT} && "
           f"bash scripts/eval_policy.sh --root_dir {RD_ROOT} --task_name {args.task} "
           f"--env_cfg_type {args.env_cfg} --device_id {args.gpu} --policy_name {POLICY_NAME} "
           f"--port {port} --host 127.0.0.1 --additional_info {ADDITIONAL_INFO} --seed {band}")
    logf = (out / "sim_client.log").open("w")
    return subprocess.Popen(["bash", "-lc", cmd], cwd=str(RD_ROOT), env=env, stdout=logf, stderr=subprocess.STDOUT)


class EpisodeRunner:
    """Per-episode program process + fair socket, driven by bridge callbacks."""

    def __init__(self, args, eps: list[int], out: Path, bridge: RoboDojoBridge):
        self.args, self.eps, self.out, self.bridge = args, eps, out, bridge
        self.records: dict[int, dict] = {}
        self._threads: dict[int, threading.Thread] = {}
        self._robots: dict[int, RoboDojoRobot] = {}
        bridge.on_episode_start = self.start
        bridge.on_episode_end = self.end

    def start(self, idx: int) -> RoboDojoRobot:
        ep = self.eps[idx] if idx < len(self.eps) else -1
        robot = RoboDojoRobot(self.bridge, ep)
        self._robots[idx] = robot
        self.records[idx] = {"episode": ep, "program": str(self.args.program), "t0": time.time()}
        t = threading.Thread(target=self._serve, args=(idx, robot), name=f"ep{ep}", daemon=True)
        self._threads[idx] = t
        t.start()
        return robot

    def _serve(self, idx: int, robot: RoboDojoRobot) -> None:
        ep = robot.episode
        rec = self.records[idx]
        tmp_root = Path(self.args.tmp_root)
        tmp_root.mkdir(parents=True, exist_ok=True)
        sandbox = Path(tempfile.mkdtemp(prefix=f"fair_rd_ep{ep}_", dir=tmp_root))
        sock_path = str(sandbox / "api.sock")
        proc = srv = None
        try:
            srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            srv.bind(sock_path)
            srv.listen(1)
            srv.settimeout(60.0)
            env = {"PATH": "/usr/bin:/bin", "HOME": str(sandbox), "TMPDIR": str(sandbox)}
            with (self.out / f"program_ep{ep}.stderr").open("w") as errf:
                proc = subprocess.Popen(
                    [sys.executable, "-I", str(ROOT / "tools/fair_client.py"),
                     sock_path, str(self.args.program), str(sandbox)],
                    cwd=str(sandbox), env=env, stdout=errf, stderr=errf)
                conn, _ = srv.accept()
                deadline = time.time() + float(self.args.ep_timeout)
                end = serve_episode(robot, conn, self.out / f"program_ep{ep}.log", deadline)
                conn.close()
                try:
                    proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    proc.kill()
            rec.update({"note": str(end.get("note", ""))[:200]})
            if end.get("error"):
                rec["program_error"] = str(end["error"])[:400]
        except Exception as e:  # noqa: BLE001
            rec.update({"error": f"{type(e).__name__}: {e}", "traceback": traceback.format_exc()[-1200:]})
        finally:
            if proc is not None and proc.poll() is None:
                proc.kill()
            if srv is not None:
                srv.close()
            shutil.rmtree(sandbox, ignore_errors=True)
            rec["program_done_t"] = time.time()
            robot.program_done = True

    def end(self, idx: int) -> None:
        robot = self._robots.get(idx)
        if robot is not None:
            robot.terminated = True
        t = self._threads.get(idx)
        if t is not None:
            t.join(timeout=40.0)
        rec = self.records.get(idx)
        if rec is not None and robot is not None:
            rec["sim_steps"] = robot.sim_steps
            rec["duration_s"] = round(time.time() - rec["t0"], 1)

    def finish_all(self) -> None:
        for idx in list(self._threads):
            self.end(idx)


def read_result_json(args, band: str, run_id: str) -> dict:
    d = (RD_ROOT / "eval_result" / "RoboDojo" / args.task / POLICY_NAME / args.env_cfg
         / f"{band}_{ADDITIONAL_INFO}" / run_id)
    f = d / "_result.json"
    if not f.is_file():
        raise RuntimeError(f"no _result.json at {f}")
    return {"dir": str(d), **json.loads(f.read_text())}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", required=True)
    ap.add_argument("--program", required=True)
    ap.add_argument("--split", required=True, choices=["debug", "eval"])
    ap.add_argument("--episode-list", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--gpu", default="0")
    ap.add_argument("--env-cfg", default="arx_x5_encore")
    ap.add_argument("--dev-pool", default="2", help="official seed dir used as the development band")
    ap.add_argument("--eval-pool", default="0,1", help="official seed dirs (comma list) concatenated for the sealed band")
    ap.add_argument("--ep-timeout", type=float, default=900.0)
    ap.add_argument("--eval-n", type=int, default=50, help="sealed band size: episodes 1..N of the eval pool (default 50)")
    ap.add_argument("--tmp-root", default="/mnt/data/YifanKang/tmp")
    ap.add_argument("--heron-config", default=str(ROOT / "configs/robodojo_ppapi.yaml"))  # Gemini direct credits depleted 2026-09-24
    args = ap.parse_args()

    eps = [int(x) for x in args.episode_list.split(",") if x.strip()]
    if args.split == "debug":
        illegal = sorted(set(eps) - DEBUG_SEEDS)
        if not eps or illegal:
            raise SystemExit(f"[fair-rd] debug split runs only episodes 51-65 (got {illegal or 'nothing'})")
    else:
        if eps:
            raise SystemExit("[fair-rd] eval split takes no --episode-list: it runs EXACTLY episodes 1..--eval-n")
        if not 1 <= args.eval_n <= 50:
            raise SystemExit("[fair-rd] --eval-n must be 1..50")
        eps = sorted(EVAL_SEEDS)[: args.eval_n]

    args.program = str(Path(args.program).resolve())
    scan_program(Path(args.program), args.split)

    final_out = Path(args.out).resolve()
    if final_out.exists():
        raise SystemExit(f"[fair-rd] {final_out} already exists — written once, never appended")
    out = final_out.parent / f".{final_out.name}.staging" if args.split == "eval" else final_out
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    if args.split == "eval":
        os.chmod(out, 0o700)
    shutil.copy(args.program, out / "program_archived.py")

    os.environ['FAIR_RD_PROGRAM'] = args.program
    band, band_dir = make_band(args.task, args.env_cfg, args.split, eps, args.dev_pool, args.eval_pool)
    run_id = time.strftime("%Y-%m-%d_%H-%M-%S") + f"_{os.getpid()}"
    port = _free_port()
    bridge = RoboDojoBridge(heron_config=args.heron_config)
    runner = EpisodeRunner(args, eps, out, bridge)
    start_policy_server(bridge, port)
    time.sleep(1.0)
    print(f"[fair-rd] policy server 127.0.0.1:{port}; band {band} ({len(eps)} layouts); run_id {run_id}", flush=True)

    t0 = time.time()
    client = launch_client(args, band, port, run_id, len(eps), out)
    rc = client.wait()
    runner.finish_all()
    print(f"[fair-rd] sim client exited rc={rc} after {time.time() - t0:.0f}s", flush=True)

    try:
        res = read_result_json(args, band, run_id)
    except Exception as e:  # noqa: BLE001
        res = {"error": str(e), "details": {}}
    details = {int(v["layout_id"]): v for k, v in (res.get("details") or {}).items() if isinstance(v, dict)}

    results = out / "results.jsonl"
    n_succ = 0
    with results.open("w") as f:
        for idx, ep in enumerate(eps):
            rec = runner.records.get(idx, {"episode": ep, "program": str(args.program)})
            rec.pop("t0", None)
            rec.pop("program_done_t", None)
            d = details.get(idx)
            if d is None:
                rec.update({"benchmark_success": False, "score": 0.0,
                            "judge": "missing (layout unstable or client died)"})
            else:
                rec.update({"benchmark_success": bool(d.get("success")), "score": float(d.get("score", 0.0))})
            n_succ += bool(rec.get("benchmark_success"))
            f.write(json.dumps(rec) + "\n")
            robot = runner._robots.get(idx)
            if robot is not None:
                try:
                    robot.save_gif(out / f"ep{ep}_{'ok' if rec['benchmark_success'] else 'fail'}.gif")
                except Exception:  # noqa: BLE001
                    pass
    (out / "robodojo_result.json").write_text(json.dumps(res, indent=1))
    shutil.rmtree(band_dir, ignore_errors=True)
    shutil.rmtree(RD_ROOT / "Assets" / "Traj" / "RoboDojo" / args.task / band, ignore_errors=True)
    for _f in (RD_ROOT / "Assets" / "Traj" / "RoboDojo" / args.task).glob(f"{band}_*.pkl"):
        _f.unlink(missing_ok=True)

    n_missing = sum(1 for idx in range(len(eps)) if idx not in details)
    # RoboDojo's own loop drops a layout whose physics went unstable and draws another;
    # a fixed sealed band has nothing to draw, so those episodes legitimately have no
    # verdict (they count as failures above, judge "missing"). Only holes BEYOND the
    # unstable count mean the simulator never reached the layout (truncation, crash).
    n_unstable = 0
    try:
        for line in (out / "sim_client.log").read_text(errors="ignore").splitlines():
            if "Unstable nums:" in line:
                n_unstable = int(line.rsplit("Unstable nums:", 1)[1].strip().split()[0])
    except Exception:  # noqa: BLE001
        pass
    if args.split == "eval":
        print(f"[fair-rd-eval] verdicts {len(eps) - n_missing}/{len(eps)}, unstable layouts {n_unstable}", flush=True)
        if n_missing > n_unstable:
            # A sealed result with holes is not a result: RoboDojo's per-task eval_nums
            # (25 for arrange/fold/pack...) truncated two 50-layout bands on 09-17 and the
            # published 0/50 looked like a verdict. Park it where the scheduler will not
            # count it as done.
            bad = final_out.with_name(final_out.name + f".incomplete_{len(eps) - n_missing}of{len(eps)}")
            out.rename(bad)
            print(f"[fair-rd-eval] INCOMPLETE {bad}: {n_missing} of {len(eps)} episodes have no simulator verdict; NOT published", flush=True)
            return 3
        out.rename(final_out)
        print(f"[fair-rd-eval] PUBLISHED {final_out}: {n_succ}/{len(eps)}", flush=True)
    else:
        print(f"[fair-rd] {final_out}: {n_succ}/{len(eps)}  score {res.get('score', 'n/a')}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
