#!/usr/bin/env python3
"""Validate a planned motion against the fair guard and the real IK solver.

NO HARDWARE. Run this before spending a physical attempt: it answers "is this
geometry even legal" in software, which is cheap, instead of on the rig, which
is not — and on a crushable or single-shot object a wasted attempt also costs a
scene reset.

It checks each planned pose for
  * finiteness and shape, exactly as RigRobot._guard does;
  * containment in the fair workspace box, read from the config rather than
    copied, so this tool cannot drift away from the guard it models;
  * an IK solution at the requested attitude (Kinematics.ik -> joint angles or
    None), across a tilt ladder if you give it one.

It cannot check external effort (that needs the arm) and it does not claim a
solvable pose is graspable — L15: reach is not grasp. Keep the comfort-band
gate separate.

Usage
    python3 tools/rig_dryrun.py --plan plan.json [--config configs/rig5090.yaml]

`plan.json` is {"poses": [{"name": ..., "xyz": [x,y,z],
                           "rvec": [rx,ry,rz]        # optional
                           "rotation": [[..],[..],[..]]  # or a 3x3, converted
                          }, ...],
                "tilts_deg": [0, 15, 30]}   # optional ladder, applied about +y
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))


def load_bounds(cfg_path: Path):
    """The guard box, from the same file RigRobot loads it from."""
    import yaml
    cfg = yaml.safe_load(cfg_path.read_text())
    ws = (cfg or {}).get("workspace") or {}
    lo, hi = ws.get("min_xyz"), ws.get("max_xyz")
    if lo is None or hi is None:
        raise SystemExit(f"{cfg_path} has no workspace.min_xyz/max_xyz — "
                         "the fair guard would be OFF (see L9)")
    return np.asarray(lo, float), np.asarray(hi, float)


def as_rvec(pose):
    import cv2
    if "rvec" in pose:
        return np.asarray(pose["rvec"], float).reshape(3)
    if "rotation" in pose:
        R = np.asarray(pose["rotation"], float).reshape(3, 3)
        return cv2.Rodrigues(R)[0].reshape(3)
    raise SystemExit(f"pose {pose.get('name')!r} has neither rvec nor rotation")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", required=True)
    ap.add_argument("--config", default=str(REPO / "configs" / "rig5090.yaml"))
    args = ap.parse_args()

    from heron.robot.kinematics import Kinematics
    import cv2

    lo, hi = load_bounds(Path(args.config))
    kin = Kinematics()
    plan = json.loads(Path(args.plan).read_text())
    tilts = np.radians(plan.get("tilts_deg") or [0.0])

    print(f"guard box  lo={lo.tolist()}  hi={hi.tolist()}")
    print(f"{'pose':28s} {'in box':>7} {'IK @ tilt (deg)':>30}")
    bad = 0
    for pose in plan.get("poses", []):
        name = str(pose.get("name", "?"))[:28]
        p = np.asarray(pose["xyz"], float).reshape(3)
        ok_shape = p.shape == (3,) and np.all(np.isfinite(p))
        inside = ok_shape and bool(np.all(p >= lo) and np.all(p <= hi))
        base = as_rvec(pose)
        marks = []
        for t in tilts:
            R = cv2.Rodrigues(base)[0]
            R = R @ cv2.Rodrigues(np.array([0.0, float(t), 0.0]))[0]
            q = kin.ik(p, cv2.Rodrigues(R)[0].reshape(3))
            marks.append("." if q is not None else "x")
        solved = any(m == "." for m in marks)
        if not (inside and solved):
            bad += 1
        print(f"{name:28s} {'yes' if inside else 'NO':>7} "
              f"{''.join(marks):>30}   "
              f"{'' if inside and solved else '<-- BLOCKED'}")
    print(f"\n'.' = IK solved, 'x' = no solution, at tilts "
          f"{np.degrees(tilts).round(0).tolist()}")
    if bad:
        print(f"{bad} pose(s) blocked. Fix the geometry before touching the robot.")
    else:
        print("all poses legal and solvable. Reach is not grasp (L15) — the "
              "comfort-band gate is still yours to apply.")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
