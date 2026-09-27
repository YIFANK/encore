#!/usr/bin/env python3
"""Pose cards: the hardware equivalent of a simulator seed.

In simulation an episode's initial state is a seed, and sealing the evaluation
band means refusing seeds outside it.  Hardware has no seed, so the protocol
needs an artifact that plays the same role: a *pose card* names where every
task object starts, within a stated tolerance.  A trial is then

    card -> reset executor places the scene -> verifier confirms it -> episode

and the reproducibility of the trial is exactly the tolerance the verifier
enforces, which is recorded per episode rather than assumed.

Bands mirror the simulated protocol.  Debug cards (ids ``d51``..``d65``) are
the only ones an acquiring session may run.  Evaluation cards (``e1``..``e50``)
are written to a separate directory that this tool creates mode 0700 and that
the coordinator alone dispatches, once, after the program is frozen.

    # once per task, by the coordinator
    python3 tools/rig_protocol.py make --task bowl_on_plate \
        --layout layouts/bowl_on_plate.json --seed 20260817 \
        --out cards/bowl_on_plate --sealed-out /private/rig_eval/bowl_on_plate

    # what a worker is allowed to see
    python3 tools/rig_protocol.py show --cards cards/bowl_on_plate --band debug

A layout file declares the placement regions and is the only human-authored
input:

    {
      "objects": {
        "bowl":  {"region": [0.30, -0.10, 0.46, 0.06], "yaw_range": [-3.14, 3.14]},
        "plate": {"region": [0.30,  0.10, 0.46, 0.26], "yaw_range": [0.0, 0.0]}
      },
      "min_separation_m": 0.12,
      "tolerance_m": 0.02,
      "tolerance_rad": 0.20
    }

Regions are axis-aligned world-frame rectangles ``[x0, y0, x1, y1]`` on the
table plane.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

DEBUG_IDS = [f"d{i}" for i in range(51, 66)]
EVAL_IDS = [f"e{i}" for i in range(1, 51)]


def _draw(rng: np.random.Generator, layout: dict) -> dict:
    """One card: a placement per object, respecting the separation constraint."""
    objs = layout["objects"]
    min_sep = float(layout.get("min_separation_m", 0.0))
    for _ in range(2000):
        placed = {}
        ok = True
        for name, spec in objs.items():
            x0, y0, x1, y1 = [float(v) for v in spec["region"]]
            lo, hi = spec.get("yaw_range", [0.0, 0.0])
            p = np.array([rng.uniform(x0, x1), rng.uniform(y0, y1)])
            for q in placed.values():
                if np.linalg.norm(p - np.array(q[:2])) < min_sep:
                    ok = False
                    break
            if not ok:
                break
            placed[name] = [round(float(p[0]), 4), round(float(p[1]), 4),
                            round(float(rng.uniform(float(lo), float(hi))), 4)]
        if ok and len(placed) == len(objs):
            return placed
    raise RuntimeError("could not satisfy min_separation_m after 2000 draws; "
                       "widen the regions or lower the separation")


def cmd_make(args: argparse.Namespace) -> int:
    layout = json.loads(Path(args.layout).read_text())
    rng = np.random.default_rng(int(args.seed))
    meta = {
        "task": args.task,
        "layout": layout,
        "seed": int(args.seed),
        "tolerance_m": float(layout.get("tolerance_m", 0.02)),
        "tolerance_rad": float(layout.get("tolerance_rad", 0.2)),
        "note": "world-frame [x, y, yaw]; regions are table-plane rectangles",
    }
    # Draw the evaluation band FIRST so that regenerating the debug band later
    # (a legitimate operation) cannot shift the sealed one.
    cards = {cid: _draw(rng, layout) for cid in EVAL_IDS}
    cards.update({cid: _draw(rng, layout) for cid in DEBUG_IDS})

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "meta.json").write_text(json.dumps(meta, indent=1))
    (out / "debug_cards.json").write_text(json.dumps(
        {cid: cards[cid] for cid in DEBUG_IDS}, indent=1))

    sealed = Path(args.sealed_out)
    sealed.mkdir(parents=True, exist_ok=True)
    os.chmod(sealed, 0o700)
    (sealed / "eval_cards.json").write_text(json.dumps(
        {cid: cards[cid] for cid in EVAL_IDS}, indent=1))
    os.chmod(sealed / "eval_cards.json", 0o600)
    (sealed / "meta.json").write_text(json.dumps(meta, indent=1))

    print(f"[cards] {args.task}: {len(DEBUG_IDS)} debug cards -> {out}")
    print(f"[cards] {len(EVAL_IDS)} evaluation cards -> {sealed} (mode 0700)")
    print(f"[cards] tolerance {meta['tolerance_m']*1000:.0f} mm / "
          f"{np.degrees(meta['tolerance_rad']):.0f} deg")
    return 0


def load_card(cards_dir: str | Path, card_id: str) -> dict:
    """Resolve one card.  Debug ids come from the open directory; evaluation
    ids come from the sealed directory and are refused unless the caller is
    the coordinator (i.e. is running with the sealed path in hand)."""
    d = Path(cards_dir)
    if card_id in DEBUG_IDS:
        return json.loads((d / "debug_cards.json").read_text())[card_id]
    if card_id in EVAL_IDS:
        f = d / "eval_cards.json"
        if not f.exists():
            raise PermissionError(
                f"{card_id} is an evaluation card; it lives in the sealed "
                f"directory and is dispatched by the coordinator only")
        return json.loads(f.read_text())[card_id]
    raise ValueError(f"unknown card id {card_id!r}")


def cmd_show(args: argparse.Namespace) -> int:
    d = Path(args.cards)
    if args.band == "debug":
        cards = json.loads((d / "debug_cards.json").read_text())
    else:
        cards = json.loads((d / "eval_cards.json").read_text())
    meta = json.loads((d / "meta.json").read_text())
    print(f"task={meta['task']}  band={args.band}  n={len(cards)}  "
          f"tol={meta['tolerance_m']*1000:.0f}mm")
    for cid, placement in cards.items():
        parts = "  ".join(f"{k}=({v[0]:+.3f},{v[1]:+.3f},{np.degrees(v[2]):+.0f}deg)"
                          for k, v in placement.items())
        print(f"  {cid:>4}  {parts}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    m = sub.add_parser("make", help="generate both bands for a task")
    m.add_argument("--task", required=True)
    m.add_argument("--layout", required=True)
    m.add_argument("--seed", required=True)
    m.add_argument("--out", required=True, help="open dir for the debug band")
    m.add_argument("--sealed-out", required=True, help="0700 dir for the eval band")
    m.set_defaults(fn=cmd_make)

    s = sub.add_parser("show", help="print a band")
    s.add_argument("--cards", required=True)
    s.add_argument("--band", choices=["debug", "eval"], default="debug")
    s.set_defaults(fn=cmd_show)

    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
