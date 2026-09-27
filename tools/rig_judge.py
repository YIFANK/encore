#!/usr/bin/env python3
"""Decide a hardware trial's success bit, after the episode, coordinator-side.

Simulation hands us a goal predicate.  Hardware does not, so the predicate has
to be written down and evaluated from perception — and where it is evaluated
matters as much as how.  This tool runs only after the policy program's
process is gone, reads only the coordinator's post-episode captures, and
writes the bit into the trial record.  A program can therefore no more query
it than it can query a sealed evaluation seed.

The predicate is geometric first.  A VLM second opinion is recorded when
available but never overrides geometry: a language model's confident "yes" on
a near miss is exactly the failure mode that would quietly inflate a success
rate, so its verdict is stored alongside for audit and disagreement is
surfaced rather than resolved.

A task's goal file:

    {
      "language": "put the bowl on the plate",
      "predicate": {"type": "on", "object": "bowl", "target": "plate",
                    "xy_tol_m": 0.05},
      "vlm_question": "Is the bowl resting on the plate? Answer yes or no."
    }

Supported predicate types:
  on          object within ``xy_tol_m`` of target's centre in the table plane
  in_region   object inside an axis-aligned world rectangle
  near        object within ``xy_tol_m`` of a fixed world point
  moved       object moved at least ``min_delta_m`` from its card placement
  all         every sub-predicate in ``terms`` holds (an all-or-nothing goal)

    python3 tools/rig_judge.py --config configs/abaka.yaml \
        --goal goals/bowl_on_plate.json --cards cards/bowl_on_plate \
        --card d51 --episode 51 --out results/rig_dbg_bowl_on_plate
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tools.rig_protocol import load_card  # noqa: E402
from tools.rig_reset import locate  # noqa: E402


def evaluate(pred: dict, where: dict, card: dict) -> tuple[bool, str]:
    """Evaluate one predicate against located world positions."""
    kind = pred["type"]

    if kind == "all":
        reasons = []
        ok = True
        for term in pred["terms"]:
            t_ok, t_why = evaluate(term, where, card)
            ok = ok and t_ok
            reasons.append(("+" if t_ok else "-") + t_why)
        return ok, "; ".join(reasons)

    obj = pred["object"]
    p = where.get(obj)
    if p is None:
        return False, f"{obj} not located"
    p = np.asarray(p, float)[:2]

    if kind == "on" or kind == "near":
        if kind == "on":
            q = where.get(pred["target"])
            if q is None:
                return False, f"{pred['target']} not located"
            q = np.asarray(q, float)[:2]
            label = pred["target"]
        else:
            q = np.asarray(pred["point"], float)[:2]
            label = f"({q[0]:.2f},{q[1]:.2f})"
        d = float(np.linalg.norm(p - q))
        tol = float(pred.get("xy_tol_m", 0.05))
        return d <= tol, f"{obj}->{label} {d*1000:.0f}mm (tol {tol*1000:.0f})"

    if kind == "in_region":
        x0, y0, x1, y1 = [float(v) for v in pred["region"]]
        ok = (min(x0, x1) <= p[0] <= max(x0, x1)
              and min(y0, y1) <= p[1] <= max(y0, y1))
        return ok, f"{obj} at ({p[0]:.3f},{p[1]:.3f}) {'in' if ok else 'outside'} region"

    if kind == "moved":
        start = np.asarray(card[obj][:2], float)
        d = float(np.linalg.norm(p - start))
        need = float(pred.get("min_delta_m", 0.05))
        return d >= need, f"{obj} moved {d*1000:.0f}mm (need {need*1000:.0f})"

    raise ValueError(f"unknown predicate type {kind!r}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--goal", required=True)
    ap.add_argument("--cards", required=True)
    ap.add_argument("--card", required=True)
    ap.add_argument("--episode", type=int, required=True)
    ap.add_argument("--out", required=True, help="trial record directory")
    ap.add_argument("--camera", default="cam_high")
    ap.add_argument("--orchestrator", default="gemini")
    ap.add_argument("--no-vlm", action="store_true",
                    help="skip the recorded second opinion")
    args = ap.parse_args()

    from heron.config import HeronConfig
    from heron.cli import _build_orchestrator
    from heron.robot.trossen import TrossenStationary

    cfg = HeronConfig.load(args.config)
    goal = json.loads(Path(args.goal).read_text())
    card = load_card(args.cards, args.card)

    robot = TrossenStationary(cfg)
    orch = _build_orchestrator(args.orchestrator, cfg, print)
    try:
        objects = sorted({*card.keys(), *_named(goal["predicate"])})
        where = {}
        for name in objects:
            p = locate(robot, orch, args.camera, name.replace("_", " "))
            where[name] = None if p is None else p.round(4).tolist()

        ok, why = evaluate(goal["predicate"], where, card)

        vlm = None
        if not args.no_vlm and goal.get("vlm_question"):
            try:
                frame = robot.capture(args.camera)
                value, conf, note = orch.vqa([frame], goal["vlm_question"])
                vlm = {"answer": str(value), "confidence": float(conf),
                       "note": str(note)[:200]}
            except Exception as e:
                vlm = {"error": f"{type(e).__name__}: {e}"}

        rec = {
            "episode": int(args.episode),
            "card": args.card,
            "benchmark_success": bool(ok),
            "judge": {"predicate": goal["predicate"], "reason": why,
                      "located": where, "vlm_second_opinion": vlm},
            "judged_at": time.time(),
        }
        if vlm and "answer" in vlm:
            said_yes = vlm["answer"].strip().lower().startswith(("y", "true", "1"))
            if said_yes != ok:
                rec["judge"]["disagreement"] = (
                    f"geometry={ok} vlm={said_yes} — geometry governs, "
                    f"flagged for coordinator review")

        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        with (out / "results.jsonl").open("a") as fh:
            fh.write(json.dumps(rec) + "\n")
        print(f"[judge] ep{args.episode} card={args.card} "
              f"success={ok} :: {why}")
        if "disagreement" in rec["judge"]:
            print(f"[judge] DISAGREEMENT {rec['judge']['disagreement']}")
        return 0
    finally:
        try:
            robot.shutdown()
        except Exception:
            pass


def _named(pred: dict) -> set[str]:
    """Every object name a predicate mentions."""
    if pred["type"] == "all":
        out: set[str] = set()
        for t in pred["terms"]:
            out |= _named(t)
        return out
    names = {pred["object"]}
    if pred.get("target"):
        names.add(pred["target"])
    return names


if __name__ == "__main__":
    sys.exit(main())
