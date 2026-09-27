#!/usr/bin/env python3
"""Build a keyframes-rung pack from a full c2 pack (information ladder).

Keeps: language, k, demos[].{demo,length,stride,keyframes[t,image,ee,
gripper_cmd,gripper_state]} and the keyframes/ images.
Strips: actions, ee_path, ee_path6, action_scale (the executable-action
channel). Usage: strip_pack.py SRC_DIR DST_DIR
"""
import json, shutil, sys
from pathlib import Path

src, dst = Path(sys.argv[1]), Path(sys.argv[2])
if dst.exists():
    print(f"exists: {dst}"); sys.exit(0)
shutil.copytree(src, dst)
pj = dst / "pack.json"
p = json.load(open(pj))
removed = []
for d in p.get("demos", []):
    for k in ("actions", "ee_path", "ee_path6", "action_scale"):
        if k in d:
            del d[k]; removed.append(k)
p["schema"] = p.get("schema", "") + "+ladder-keyframes(no-actions)"
json.dump(p, open(pj, "w"), indent=1)
print(f"built {dst}: removed {sorted(set(removed))}")
