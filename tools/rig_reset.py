#!/usr/bin/env python3
"""Reset the physical scene to a pose card, and verify that it worked.

Autonomous reset is what makes a hardware campaign affordable: a sealed
50-trial evaluation is 50 scene resets, and doing those by hand is both the
dominant cost and a source of unrecorded variation.  The loop here is

    perceive -> plan a placement per object -> place -> verify -> report

with three properties the protocol depends on:

  * **Reset is a program, not a policy.**  Its accuracy requirement is the
    card tolerance (~2 cm), an order of magnitude looser than the tasks it
    sets up, so it can be written once and reused across every arm of a
    campaign.  It carries no task-solving knowledge, so sharing it between
    the text and demonstration arms leaks nothing.
  * **Verification is separate from execution.**  ``verify`` re-perceives and
    reports per-object error against the card.  A trial whose reset does not
    verify is not run; it is re-reset or escalated.  This is what makes the
    physical initial state auditable in the way a seed is.
  * **Escalation is counted, never silent.**  Unrecoverable states (an object
    off the table, a tangle, a dropped item out of reach) print a
    ``NEEDS-HUMAN`` line and exit non-zero.  Each escalation is one row in the
    campaign's cost table.

    python3 tools/rig_reset.py apply  --config configs/abaka.yaml \
        --cards cards/bowl_on_plate --card d51
    python3 tools/rig_reset.py verify --config configs/abaka.yaml \
        --cards cards/bowl_on_plate --card d51 --json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tools.rig_protocol import load_card  # noqa: E402


# ---------------------------------------------------------------------------
# perception: pixel -> world on the table plane

def locate(robot, orch, camera: str, query: str):
    """World-frame (x, y, yaw?) of one object, or None if not found.

    Uses the frame's table-plane homography when the rig has no depth, which
    is exact for anything resting on the table — the only case a reset cares
    about.
    """
    frame = robot.capture(camera)
    hit = orch.point(frame, query)
    if hit is None:
        return None
    u, v = int(hit[0]), int(hit[1])
    xyz = None
    if getattr(frame, "depth", None) is not None:
        xyz = frame.deproject(u, v)
    if xyz is None:
        h = getattr(frame, "h_pixel_world", None)
        if h is None:
            return None
        p = np.asarray(h, float) @ np.array([u, v, 1.0])
        if abs(p[2]) < 1e-9:
            return None
        xy = p[:2] / p[2]
        z = float(getattr(frame, "plane_z", 0.0))
        xyz = np.array([xy[0], xy[1], z])
    return np.asarray(xyz, float)


def _errors(robot, orch, camera: str, card: dict) -> dict:
    out = {}
    for name, target in card.items():
        got = locate(robot, orch, camera, name.replace("_", " "))
        if got is None:
            out[name] = {"found": False}
            continue
        err = float(np.linalg.norm(got[:2] - np.asarray(target[:2], float)))
        out[name] = {"found": True, "xy": got[:2].round(4).tolist(),
                     "target": [round(float(v), 4) for v in target[:2]],
                     "error_m": round(err, 4)}
    return out


# ---------------------------------------------------------------------------
# execution

def place_one(robot, arm: str, pick_xy, place_xy, *, plane_z: float,
              lift: float = 0.12, grip_open: float = 0.06) -> None:
    """Pick an object at ``pick_xy`` and set it down at ``place_xy``.

    Deliberately coarse: approach from above, close, lift, translate, lower,
    release.  The card tolerance is what this has to hit, not a grasp
    specification.
    """
    pick = np.array([pick_xy[0], pick_xy[1], plane_z], float)
    place = np.array([place_xy[0], place_xy[1], plane_z], float)
    up = np.array([0.0, 0.0, lift])

    robot.set_gripper(arm, grip_open)
    robot.move_cartesian(arm, pick + up, seconds=2.0)
    robot.move_cartesian(arm, pick, seconds=1.5)
    robot.set_gripper(arm, 0.0)          # force close
    robot.move_cartesian(arm, pick + up, seconds=1.5)
    robot.move_cartesian(arm, place + up, seconds=2.5)
    robot.move_cartesian(arm, place, seconds=1.5)
    robot.set_gripper(arm, grip_open)
    robot.move_cartesian(arm, place + up, seconds=1.5)


def cmd_apply(args: argparse.Namespace) -> int:
    from heron.config import HeronConfig
    from heron.cli import _build_orchestrator
    from heron.robot.trossen import TrossenStationary

    cfg = HeronConfig.load(args.config)
    card = load_card(args.cards, args.card)
    meta = json.loads((Path(args.cards) / "meta.json").read_text())
    tol = float(meta.get("tolerance_m", 0.02))
    plane_z = float(args.plane_z)

    robot = TrossenStationary(cfg)
    orch = _build_orchestrator(args.orchestrator, cfg, print)
    arm = args.arm or list(cfg.arms)[0]
    camera = args.camera

    try:
        robot.home(arm)
        for attempt in range(1, int(args.max_attempts) + 1):
            errs = _errors(robot, orch, camera, card)
            todo = [n for n, e in errs.items()
                    if not e["found"] or e["error_m"] > tol]
            if not todo:
                print(f"[reset] card {args.card} satisfied on attempt {attempt}")
                print(json.dumps(errs, indent=1))
                return 0
            missing = [n for n in todo if not errs[n]["found"]]
            if missing:
                print(f"NEEDS-HUMAN: object(s) not visible: {missing}. "
                      f"Place them roughly on the table and re-run.")
                return 2
            for name in todo:
                e = errs[name]
                print(f"[reset] {name}: {e['error_m']*1000:.0f} mm off, moving")
                place_one(robot, arm, e["xy"], e["target"], plane_z=plane_z)
            robot.home(arm)
        print(f"NEEDS-HUMAN: card {args.card} not satisfied after "
              f"{args.max_attempts} attempts; last state:")
        print(json.dumps(_errors(robot, orch, camera, card), indent=1))
        return 2
    finally:
        try:
            robot.shutdown()
        except Exception:
            pass


def cmd_verify(args: argparse.Namespace) -> int:
    from heron.config import HeronConfig
    from heron.cli import _build_orchestrator
    from heron.robot.trossen import TrossenStationary

    cfg = HeronConfig.load(args.config)
    card = load_card(args.cards, args.card)
    meta = json.loads((Path(args.cards) / "meta.json").read_text())
    tol = float(meta.get("tolerance_m", 0.02))

    robot = TrossenStationary(cfg)
    orch = _build_orchestrator(args.orchestrator, cfg, print)
    try:
        errs = _errors(robot, orch, args.camera, card)
        ok = all(e["found"] and e["error_m"] <= tol for e in errs.values())
        report = {"card": args.card, "tolerance_m": tol, "ok": ok,
                  "objects": errs}
        print(json.dumps(report, indent=1) if args.json
              else f"[verify] card {args.card}: {'OK' if ok else 'OUT OF TOLERANCE'}")
        if not args.json:
            for n, e in errs.items():
                where = f"{e['error_m']*1000:.0f} mm" if e["found"] else "NOT FOUND"
                print(f"  {n:>12}: {where}")
        return 0 if ok else 1
    finally:
        try:
            robot.shutdown()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["apply", "verify"])
    ap.add_argument("--config", required=True)
    ap.add_argument("--cards", required=True)
    ap.add_argument("--card", required=True)
    ap.add_argument("--camera", default="cam_high")
    ap.add_argument("--arm", default=None)
    ap.add_argument("--orchestrator", default="gemini")
    ap.add_argument("--plane-z", type=float, default=0.0,
                    help="table height in world frame (m)")
    ap.add_argument("--max-attempts", type=int, default=3)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    return cmd_apply(args) if args.cmd == "apply" else cmd_verify(args)


if __name__ == "__main__":
    sys.exit(main())
