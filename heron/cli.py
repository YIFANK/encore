"""Heron CLI.

  heron demo                          offline mock episode with injected failures
  heron run "put the red block ..."   run a task (mock or real, Gemini or scripted)
  heron doctor                        environment / hardware / API checks
  heron calibrate --camera cam_high   ER-pointing fingertip calibration
  heron replay episodes/<dir>         pretty-print an episode journal
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table

from .config import HeronConfig

console = Console()


def _build_robot(cfg: HeronConfig, want_scene: bool = True):
    from .robot.safety import SafeRobot

    if cfg.mode == "mock":
        from .robot.mock import MockRobot, standard_scene

        mock = MockRobot()
        if want_scene:
            standard_scene(mock)
        return SafeRobot(mock, cfg.safety), mock
    if cfg.mode == "libero":
        from .robot.libero import LiberoRobot

        sim = LiberoRobot(cfg)
        return SafeRobot(sim, cfg.safety), sim
    if cfg.mode == "sim":
        # The MuJoCo twin of the real cell: same URDF, same IK, same measured
        # table height and camera calibration. A plan that survives here has
        # been checked against the cell's actual geometry.
        from .robot.trossen_sim import TrossenSim, DEFAULT_SCENE

        sim = TrossenSim(cfg, scene=cfg.sim_scene or DEFAULT_SCENE,
                         record_camera=cfg.sim_record_camera,
                         record_every=cfg.sim_record_every)
        return SafeRobot(sim, cfg.safety), sim
    from .robot.trossen import TrossenStationary

    real = TrossenStationary(cfg)
    return SafeRobot(real, cfg.safety), real


def _build_orchestrator(kind: str, cfg: HeronConfig, log, mock=None):
    if kind == "scripted":
        if mock is None:
            raise SystemExit("--orchestrator scripted requires --mode mock")
        from .orchestrator.scripted import ScriptedOrchestrator

        return ScriptedOrchestrator(mock)
    from .orchestrator.gemini import GeminiOrchestrator

    return GeminiOrchestrator(cfg, log)


def cmd_run(args: argparse.Namespace) -> int:
    cfg = HeronConfig.load(args.config) if args.config else HeronConfig()
    if args.mode:
        cfg.mode = args.mode
    if cfg.mode == "real" and not args.config:
        raise SystemExit("real mode requires --config configs/stationary.yaml (with your serials/IPs)")
    robot, backend = _build_robot(cfg)
    if cfg.mode == "libero" and args.instruction == "auto":
        args.instruction = backend.task_language
        console.print(f"[dim]LIBERO task: {args.instruction}[/dim]")
    from .agent import Agent
    from .episode import EpisodeLogger

    log = EpisodeLogger(cfg.episodes_dir, "run")
    orchestrator = _build_orchestrator(args.orchestrator, cfg, log,
                                       mock=backend if cfg.mode == "mock" else None)
    agent = Agent(robot, orchestrator, cfg, log=log)
    try:
        # Operator early verdict: while the episode runs, typing s<Enter> or
        # f<Enter> in this terminal ends it as succeeded/failed immediately.
        import threading

        # The plan briefing owns stdin first (it reads corrections); the
        # verdict thread waits its turn or the two race for the same lines.
        briefing_done = threading.Event()
        if not getattr(cfg, "interactive", False):
            briefing_done.set()

        plan_hook = None
        if getattr(cfg, "interactive", False):
            def _speak(text: str) -> None:
                if not getattr(cfg, "speak", False):
                    return
                import shutil
                import subprocess
                if shutil.which("say"):
                    voice = (["-v", "Tingting"]
                             if any("一" <= ch <= "鿿" for ch in text) else [])
                    try:
                        # Fire and forget: the operator should be able to hit
                        # Enter the moment they have read enough — the voice
                        # is a convenience, not a gate.
                        subprocess.Popen(["say", *voice, text])
                    except Exception:
                        pass

            def plan_hook(agent_, program, frames):
                text = agent_.brief(program, frames)
                console.print(f"[bold cyan][agent][/bold cyan] {text}")
                _speak(text)
                console.print("[dim]回车放行执行;或输入纠正意见(自然语言)后回车重新计划:[/dim]")
                try:
                    reply = input().strip()
                except EOFError:
                    reply = ""
                if not reply:
                    _speak("好,开始执行。")
                    briefing_done.set()
                return reply or None

        def _stdin_verdict():
            try:
                briefing_done.wait()
                for line in sys.stdin:
                    w = line.strip().lower()
                    if w in ("s", "y", "成功"):
                        agent.human_verdict = "succeeded"
                        console.print("[bold green][operator] success verdict received — finishing up[/bold green]")
                        break
                    if w in ("f", "n", "失败"):
                        agent.human_verdict = "failed"
                        console.print("[bold red][operator] failure verdict received — finishing up[/bold red]")
                        break
            except Exception:
                pass
        threading.Thread(target=_stdin_verdict, daemon=True).start()
        console.print("[dim]operator: s+Enter = success, f+Enter = fail (ends the episode early)[/dim]")
        summary = agent.run(args.instruction, plan_hook=plan_hook)
    finally:
        # STAND THE ARM DOWN before releasing the drivers. Success used to
        # leave the tool frozen wherever the last verification parked it; an
        # episode is over when the arm is at rest, not when the verdict is in.
        # SEQUENTIAL ON PURPOSE. Parallel rest sent both arms swinging toward
        # their centre-reaching ready poses at once and they nearly met over
        # the table. Twenty seconds per arm is ceremony; a collision is a
        # repair bill. Order: the arm nearer the table goes first.
        for _side in (cfg.active_arms or list(getattr(backend, "arms", []) or [])):
            try:
                _stand = getattr(backend, "rest", None) or getattr(backend, "home", None)
                if callable(_stand):
                    _stand(_side)
            except Exception as _e:                     # noqa: BLE001
                console.print(f"[dim]rest({_side}) failed: {_e}[/dim]")
        robot.shutdown()
    if cfg.mode == "libero":
        summary["libero_env_success"] = bool(getattr(backend, "task_success", False))
        summary["sim_steps"] = getattr(backend, "sim_steps", None)
    save = getattr(backend, "save_video", None) or getattr(backend, "save_gif", None)
    if callable(save):
        video = agent.log.dir / "run.gif"
        try:
            if save(video):
                console.print(f"[dim]video: {video}[/dim]")
                summary["video"] = str(video)
        except Exception as e:
            console.print(f"[dim]video not written: {e}[/dim]")
    console.print(summary)
    console.print(f"[dim]episode: {agent.log.dir}[/dim]")
    ok = summary.get("status") == "succeeded"
    # Every run leaves one machine-readable line: enough to compute success
    # rates and wall-clock stats later without re-parsing journals (which
    # remain, per-episode, the detailed record).
    try:
        import json as _json
        with open(Path(cfg.episodes_dir).parent / "runs_log.jsonl", "a") as fh:
            fh.write(_json.dumps({
                "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
                "instruction": args.instruction,
                "status": summary.get("status"),
                "reason": str(summary.get("reason", ""))[:200],
                "wall_s": summary.get("wall_s"),
                "steps": summary.get("steps_executed"),
                "edits": summary.get("edits_applied"),
                "human_verdict": getattr(agent, "human_verdict", None),
                "episode_dir": str(agent.log.dir),
            }) + "\n")
    except Exception as e:
        console.print(f"[dim]runs_log append failed: {e}[/dim]")
    # The verdict must be unmissable: a person at the rig watches the ARM, not
    # a JSON blob scrolling by — a banner for the terminal, a voice for the room.
    if ok:
        console.print("[bold green]✔ TASK SUCCEEDED — all goal predicates verified[/bold green]")
    else:
        console.print(f"[bold red]✘ TASK {str(summary.get('status', 'failed')).upper()}"
                      f" — {str(summary.get('reason', ''))[:120]}[/bold red]")
    if getattr(cfg, "mode", "") == "real":
        import subprocess
        try:
            subprocess.Popen(["say", "-v", "Tingting",
                              "任务成功" if ok else "任务失败"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass
    return 0 if ok else 1


def cmd_demo(args: argparse.Namespace) -> int:
    """Mock episode showcasing the failure taxonomy end to end on
    'put the red block in the blue bowl':
      1. the first grasp closes on air     -> SKILL_EFFECT, rule repair (re-perceive + jittered retry)
      2. an external hand snatches the held block before the place
                                           -> STATE_DRIFT, orchestrator repair (insert re-pick)"""
    from .agent import Agent
    from .orchestrator.scripted import ScriptedOrchestrator
    from .robot.mock import MockRobot, standard_scene
    from .robot.safety import SafeRobot

    cfg = HeronConfig()
    cfg.episodes_dir = args.out or cfg.episodes_dir
    mock = MockRobot()
    standard_scene(mock)
    mock.inject.grasp_misses = 1  # first grasp closes on air
    robot = SafeRobot(mock, cfg.safety)
    drift = {"armed": True}

    def perturb(step, program):
        # After the first successful pick, an invisible lab-mate takes the block
        # out of the gripper and drops it elsewhere on the table.
        if drift["armed"] and step.skill == "pick" and step.status.value == "done":
            mock.teleport("red_block", (0.12, 0.08, 0.02))
            drift["armed"] = False
            console.print("[yellow]>>> demo: external hand snatched the red block from the gripper[/yellow]")

    agent = Agent(robot, ScriptedOrchestrator(mock), cfg, name="demo", on_step_end=perturb)
    summary = agent.run("put the red block in the blue bowl")
    console.print(summary)
    _print_story(agent.log.dir)
    console.print(f"[dim]full journal: {agent.log.dir}[/dim]")
    return 0 if summary["status"] == "succeeded" else 1


def _print_story(episode_dir) -> None:
    from .episode import read_journal

    picks = {
        "step_start": lambda r: f"[bold]step {r['step_id']}[/bold] {r['skill']}({r.get('args', {})}) attempt {r.get('attempt')}",
        "failure": lambda r: f"[red]FAILURE[/red] {r['step_id']} ({r['failure_class']}): {r['summary'][:110]}",
        "program_edit": lambda r: f"[cyan]REPAIR[/cyan] {r['edit']['type']} ({r['edit'].get('source')}): {r['edit'].get('rationale', '')[:90]}",
        "perception_override": lambda r: f"[magenta]PERCEPTION OVERRIDE[/magenta] {r.get('note', '')}",
        "info_gathering": lambda r: f"[blue]LOOK CLOSER[/blue] {r['predicate']} via inspect({r['inspect']})",
        "step_done": lambda r: f"[green]done[/green] {r['step_id']} {r['skill']}",
        "goal_check": lambda r: f"goal check — unmet: {r['unmet'] or 'none'}",
        "budget_abort": lambda r: f"[red]BUDGET ABORT[/red] {r['reason']}",
    }
    console.rule("episode story")
    for rec in read_journal(episode_dir):
        fn = picks.get(rec.get("kind"))
        if fn:
            console.print(f"  t={rec['t']:7.2f}  " + fn(rec))
    console.rule()


def cmd_doctor(args: argparse.Namespace) -> int:
    table = Table(title="heron doctor")
    table.add_column("check")
    table.add_column("status")

    def row(name: str, ok: bool, note: str = "") -> None:
        table.add_row(name, ("[green]ok[/green] " if ok else "[red]missing[/red] ") + note)

    for mod in ("numpy", "pydantic", "PIL", "yaml", "rich"):
        try:
            __import__(mod)
            row(f"import {mod}", True)
        except ImportError:
            row(f"import {mod}", False)
    try:
        from google import genai  # noqa: F401

        row("google-genai", True)
    except ImportError:
        row("google-genai", False, "pip install 'google-genai>=2.9'")
    key = os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
    row("GEMINI_API_KEY", bool(key), "" if key else "export GEMINI_API_KEY=... (aistudio.google.com)")
    for mod, hint in (("trossen_arm", "real arms"), ("pyrealsense2", "real cameras"), ("lerobot", "vla skill")):
        try:
            __import__(mod)
            row(f"import {mod}", True, f"({hint})")
        except ImportError:
            row(f"import {mod}", False, f"optional — needed for {hint}")
    if args.config:
        cfg = HeronConfig.load(args.config)
        row("config parses", True, f"mode={cfg.mode}")
        if cfg.models.api_format == "openai":
            # The relay path never reads GEMINI_API_KEY, so the row above saying
            # "missing" would be a red herring — these two are what it runs on.
            m = cfg.models
            url = m.resolved_base_url()
            row(f"relay url ({m.base_url_env or 'models.base_url'})", bool(url),
                "" if url else "set it in .env — the relay path needs it")
            key_ok = bool(m.api_key_env and os.environ.get(m.api_key_env))
            row(f"relay key ({m.api_key_env or 'models.api_key_env unset'})", key_ok,
                "" if key_ok else "set it in .env — the relay path needs it")
        if cfg.mode == "real":
            for name, cam in cfg.cameras.items():
                if cam.depth_server:
                    from .robot.depthclient import DepthServerClient

                    ok, note = DepthServerClient(cam.depth_server).healthy()
                    row(f"camera {name}", ok, f"depth server {cam.depth_server} — {note}")
                elif cam.index is not None:
                    row(f"camera {name}", True, f"UVC index {cam.index}")
                else:
                    row(f"camera {name}", bool(cam.serial),
                        f"serial {cam.serial}" if cam.serial else "no serial/index configured")
            for side, arm in cfg.arms.items():
                row(f"arm {side} ip", bool(arm.ip), arm.ip)
    console.print(table)
    return 0


def cmd_calibrate(args: argparse.Namespace) -> int:
    """Marker-free eye-to-world calibration: the arm presents its fingertip on a
    grid; ER points at the fingertip in each image.

    Modes:
      default      Kabsch fit of full extrinsics (needs aligned depth).
      --planar     pixel -> table-XY homography (depth-free, table plane only).
      --projection 3x4 world -> pixel projection matrix (depth-free, full 3D once
                   two cameras are calibrated; visits several heights)."""
    cfg = HeronConfig.load(args.config)
    from .episode import EpisodeLogger
    from .robot.workspace import (calibration_grid, check_grid_reachable, fit_homography,
                                  fit_projection, grid_ranges_from_safety, save_extrinsics,
                                  save_homography, save_projection, solve_extrinsics)

    default_x, default_y = grid_ranges_from_safety(cfg)
    x_range = tuple(args.x_range) if args.x_range else default_x
    y_range = tuple(args.y_range) if args.y_range else default_y
    grid_z = (cfg.table_z + 0.03) if args.planar else 0.12
    z_levels = (cfg.table_z + 0.04, cfg.table_z + 0.12, cfg.table_z + 0.20) if args.projection else None
    if getattr(args, "z_levels", None):
        z_levels = tuple(float(z) for z in args.z_levels)
    poses = calibration_grid(z=grid_z, nx=args.grid[0], ny=args.grid[1],
                             x_range=x_range, y_range=y_range, z_levels=z_levels)
    violations = check_grid_reachable(poses, cfg)
    console.print(f"grid: {len(poses)} poses, "
                  f"x=[{x_range[0]:.3f}, {x_range[1]:.3f}] "
                  f"y=[{y_range[0]:.3f}, {y_range[1]:.3f}] "
                  f"z={sorted({round(float(p[2]), 3) for p in poses})}")
    if violations:
        console.print("[red]grid violates the safety envelope — refusing to move:[/red]")
        for v in violations:
            console.print(f"  {v}")
        console.print("[dim]pass --x-range/--y-range, or widen safety in the config[/dim]")
        return 1
    if args.dry_run:
        for i, p in enumerate(poses):
            console.print(f"  pose {i}: {np.round(p, 3).tolist()}")
        console.print("[green]grid is inside the safety envelope; nothing moved[/green]")
        return 0
    if args.yes:
        cfg.safety.confirm_actions = False

    robot, _ = _build_robot(cfg, want_scene=False)
    log = EpisodeLogger(cfg.episodes_dir, f"calibrate-{args.camera}")
    gripper_probe = None
    if args.detector == "aruco":
        from .robot.markers import detect_aruco_center

        def find_tip(frame):
            return detect_aruco_center(frame.rgb, dict_name=args.aruco_dict,
                                       marker_id=args.aruco_id)
        console.print("[dim]detector: aruco marker clamped in the gripper (no API calls)[/dim]")
    elif args.detector == "gripper":
        from .robot.fingertip import pinch_point_from_pair

        def gripper_probe(arm_name, camera):
            """Only the fingers move, so the change between the two frames is
            the gripper and its centroid is the pinch point."""
            robot.set_gripper(arm_name, 0.06)
            opened = robot.capture(camera)
            robot.set_gripper(arm_name, 0.015)
            closed = robot.capture(camera)
            hit = pinch_point_from_pair(opened.rgb, closed.rgb)
            return hit, opened

        def find_tip(frame):  # unused on this path
            return None
        console.print("[dim]detector: gripper open/close differencing (no API calls, "
                      "immune to the idle arm)[/dim]")
    else:
        orch = _build_orchestrator("gemini", cfg, log)

        def find_tip(frame):
            return orch.point(frame, "the robot gripper fingertips")
    arm = args.arm
    pairs_cam, pairs_world, pairs_px = [], [], []
    unreachable = 0

    def travel(target: np.ndarray, seconds: float = 2.5) -> None:
        """Move to `target`, hopping if it is farther than max_step_m.

        The step limit exists to catch a hallucinated coordinate, not to forbid
        legitimate long travel — and the ready pose is over half a metre from the
        table. Hops stay on the straight segment to the target, so each one is
        individually bounded and individually safety-checked.
        """
        limit = cfg.safety.max_step_m
        for _ in range(12):
            here = np.asarray(robot.get_cartesian(arm), dtype=float)
            delta = np.asarray(target, dtype=float) - here
            dist = float(np.linalg.norm(delta))
            if dist <= limit:
                robot.move_cartesian(arm, target, seconds=seconds)
                return
            robot.move_cartesian(arm, here + delta / dist * (limit * 0.9), seconds=seconds)
        raise RuntimeError(f"could not reach {np.round(target, 3).tolist()} within 12 hops")

    try:
        console.print("homing before the grid …")
        robot.home(arm)
        for i, pose in enumerate(poses):
            # Envelope-legal is not the same as IK-reachable: corner poses may be
            # outside the arm's actual dexterous range, and one refusal must not
            # abandon the whole grid.
            try:
                travel(pose)
            except Exception as e:
                # A comms failure is not an IK failure: the controller accepts a
                # single owner, so a second client (teleop, another script) makes
                # every pose look unreachable. Say which one it is.
                msg = str(e)
                comms = any(s in msg for s in ("TCP", "UDP", "timed out", "temporarily unavailable"))
                if comms:
                    console.print(f"[red]pose {i}: controller comms failed ({type(e).__name__}). "
                                  "Another process is probably talking to this arm "
                                  "(teleop, dashboard, another Heron run) — stop it and "
                                  "rerun.[/red]")
                    console.print(f"[dim]{msg}[/dim]")
                    break
                unreachable += 1
                console.print(f"[yellow]pose {i} {np.round(pose, 3).tolist()} failed "
                              f"({type(e).__name__}): {msg}[/yellow]")
                try:
                    robot.home(arm)
                except Exception:
                    console.print("[red]could not recover to home — stopping[/red]")
                    break
                continue
            if gripper_probe is not None:
                hit, frame = gripper_probe(arm, args.camera)
            else:
                frame = robot.capture(args.camera)
                hit = find_tip(frame)
            log.frame(frame, tag=f"calib{i:02d}")
            if hit is None:
                console.print(f"[yellow]pose {i}: fingertip not found, skipping[/yellow]")
                continue
            u, v, _ = hit
            if args.planar or args.projection:
                pairs_px.append([u, v])
                pairs_world.append(list(pose) if args.projection else [pose[0], pose[1]])
                console.print(f"pose {i}: px=({u},{v}) -> {np.round(pose, 3).tolist()}")
                continue
            if frame.depth is None:
                console.print(f"[yellow]pose {i}: no depth on this rig — use --planar[/yellow]")
                continue
            # Deproject in CAMERA coordinates (extrinsics are what we're solving).
            z = float(np.median(frame.depth[max(0, v - 2): v + 3, max(0, u - 2): u + 3]))
            fx, fy = frame.intrinsics[0, 0], frame.intrinsics[1, 1]
            cx, cy = frame.intrinsics[0, 2], frame.intrinsics[1, 2]
            pairs_cam.append([(u - cx) * z / fx, (v - cy) * z / fy, z])
            pairs_world.append(list(pose))
            console.print(f"pose {i}: px=({u},{v}) z={z:.3f}")
    finally:
        robot.home(arm)
        robot.shutdown()
    if unreachable:
        console.print(f"[yellow]{unreachable}/{len(poses)} grid poses were unreachable — "
                      "narrow --x-range/--y-range to keep the grid inside the arm's "
                      "dexterous range[/yellow]")
    if args.projection:
        if len(pairs_px) < 6:
            console.print("[red]need >= 6 detections across several heights[/red]")
            return 1
        P, rms = fit_projection(np.array(pairs_world), np.array(pairs_px))
        out = Path(cfg.cameras[args.camera].projection_file or f"calibration/{args.camera}_projection.npz")
        save_projection(out, P, rms)
        console.print(f"[green]saved {out} (reprojection rms {rms:.1f} px over {len(pairs_px)} points)[/green]")
        console.print("[dim]calibrate a second camera the same way to enable triangulation[/dim]")
        return 0
    if args.planar:
        if len(pairs_px) < 4:
            console.print("[red]not enough correspondences; check lighting/camera[/red]")
            return 1
        px = np.array(pairs_px, dtype=float)
        world = np.array(pairs_world, dtype=float)
        H, rms = fit_homography(px, world)
        # Per-point residuals: a large RMS from one bad detection is a different
        # problem (drop the point) than one spread evenly over the grid (the
        # detected feature does not sit where forward kinematics says it does).
        hom = np.hstack([px, np.ones((len(px), 1))]) @ H.T
        pred = hom[:, :2] / hom[:, 2:3]
        resid = np.linalg.norm(pred - world, axis=1)
        console.print("residuals (mm):")
        for (u, v), w, r in zip(pairs_px, world, resid):
            console.print(f"  px=({u:4d},{v:4d}) world=({w[0]:+.3f},{w[1]:+.3f})  {r * 1000:6.1f}")
        console.print(f"  worst {resid.max() * 1000:.1f} mm, median {np.median(resid) * 1000:.1f} mm")
        out = Path(cfg.cameras[args.camera].homography_file or f"calibration/{args.camera}_homography.npz")
        save_homography(out, H, cfg.table_z, rms)
        console.print(f"[green]saved {out} (rms {rms * 1000:.1f} mm over {len(pairs_px)} points)[/green]")
        return 0
    if len(pairs_cam) < 4:
        console.print("[red]not enough correspondences; check lighting/camera[/red]")
        return 1
    T, rms = solve_extrinsics(np.array(pairs_cam), np.array(pairs_world))
    out = Path(cfg.cameras[args.camera].extrinsics_file or f"calibration/{args.camera}.npz")
    save_extrinsics(out, T, rms)
    console.print(f"[green]saved {out} (rms {rms * 1000:.1f} mm over {len(pairs_cam)} points)[/green]")
    return 0


def cmd_replay(args: argparse.Namespace) -> int:
    _print_story(args.episode)
    from .episode import read_journal

    calls = [r for r in read_journal(args.episode) if r.get("kind") == "model_call"]
    if calls:
        t = Table(title="model calls")
        for col in ("role", "thinking", "latency_s", "images", "prompt_chars"):
            t.add_column(col)
        for c in calls:
            t.add_row(c["role"], c.get("thinking", ""), str(c["latency_s"]), str(c["images"]), str(c["prompt_chars"]))
        console.print(t)
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="heron", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    pr = sub.add_parser("run", help="run a task")
    pr.add_argument("instruction")
    pr.add_argument("--config", default=None)
    pr.add_argument("--mode", choices=["mock", "real", "libero"], default=None)
    pr.add_argument("--orchestrator", choices=["gemini", "scripted"], default="gemini")
    pr.set_defaults(fn=cmd_run)

    pd = sub.add_parser("demo", help="offline mock demo with injected failures")
    pd.add_argument("--out", default=None)
    pd.set_defaults(fn=cmd_demo)

    pdoc = sub.add_parser("doctor", help="environment checks")
    pdoc.add_argument("--config", default=None)
    pdoc.set_defaults(fn=cmd_doctor)

    pc = sub.add_parser("calibrate", help="ER-pointing fingertip camera calibration")
    pc.add_argument("--config", required=True)
    pc.add_argument("--camera", default="cam_high")
    pc.add_argument("--arm", default="right")
    pc.add_argument("--detector", choices=["gripper", "er", "aruco"], default="gripper",
                    help="fingertip finder: open/close the gripper and difference the "
                         "frames (default; offline and unambiguous on a two-arm rig), "
                         "ER pointing, or an ArUco marker clamped between the fingers")
    pc.add_argument("--aruco-dict", default="4X4_50")
    pc.add_argument("--aruco-id", type=int, default=None)
    pc.add_argument("--planar", action="store_true",
                    help="fit a depth-free pixel->table-XY homography (for the UVC/RGB-only rig)")
    pc.add_argument("--projection", action="store_true",
                    help="fit a depth-free 3x4 world->pixel projection matrix; two calibrated "
                         "cameras then triangulate full 3D")
    pc.add_argument("--x-range", type=float, nargs=2, metavar=("MIN", "MAX"),
                    help="grid x bounds (m); default: safety envelope inset by 4 cm")
    pc.add_argument("--y-range", type=float, nargs=2, metavar=("MIN", "MAX"))
    pc.add_argument("--grid", type=int, nargs=2, default=(3, 3), metavar=("NX", "NY"))
    pc.add_argument("--z-levels", type=float, nargs="+", metavar="Z",
                    help="fingertip heights to visit (m, world frame). A grid at ONE "
                         "height is coplanar, and the Kabsch fit then reads the "
                         "camera's tilt and distance off a plane that constrains "
                         "them weakly — cam_high was fit this way and recorded "
                         "14.8 mm rms. Two or three heights cost one extra pass each.")
    pc.add_argument("--dry-run", action="store_true",
                    help="print the grid poses and safety check, move nothing")
    pc.add_argument("--yes", action="store_true",
                    help="skip per-motion confirmation; the grid is validated against "
                         "the safety envelope before anything moves")
    pc.set_defaults(fn=cmd_calibrate)

    prep = sub.add_parser("replay", help="pretty-print an episode journal")
    prep.add_argument("episode")
    prep.set_defaults(fn=cmd_replay)

    args = p.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
