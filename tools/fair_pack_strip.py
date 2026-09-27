"""Strip a fair pack down to one input modality (abl_noact campaign).

Reads a `fair-pack-v1` pack.json written by tools/fair_pack.py and rewrites it
IN PLACE with every robot-state / action channel removed, keeping only what a
camera and the task sentence would give you:

  keeps : schema, language, k, demos[].demo, demos[].length,
          demos[].keyframes[].t, demos[].keyframes[].image
  drops : keyframes[].ee / gripper_cmd / gripper_state (and *_left variants),
          demos[].ee_path / ee_path6 / action_scale / actions / stride /
          ee_path_left / ee_path6_left / arms

The keyframe PNGs are untouched. Note for the record: WHICH frames are
keyframes was chosen by fair_pack.py from gripper events in the action
stream, so the frame selection itself still carries a trace of the action
channel (disclosed in the campaign's PREREGISTERED.md; the alternative,
uniform-stride frames, would change the vision channel too).

    .venv/bin/python tools/fair_pack_strip.py --pack packs/noact_<cell> --mode vision
    .venv/bin/python tools/fair_pack_strip.py --pack packs/noact_<cell> --audit

`--audit` exits non-zero if any dropped key survives anywhere in pack.json.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

KEEP_TOP = ("schema", "language", "k", "demos")
KEEP_DEMO = ("demo", "length", "keyframes")
KEEP_KF = ("t", "image")
DROPPED = {"ee", "gripper_cmd", "gripper_state", "ee_left", "gripper_cmd_left",
           "gripper_state_left", "ee_path", "ee_path6", "action_scale", "actions",
           "stride", "ee_path_left", "ee_path6_left", "arms"}


def strip_vision(pack: dict) -> dict:
    out = {k: pack[k] for k in KEEP_TOP if k in pack}
    out["schema"] = "fair-pack-v1-vision"
    out["demos"] = [
        {**{k: d[k] for k in KEEP_DEMO if k in d},
         "keyframes": [{k: kf[k] for k in KEEP_KF if k in kf} for kf in d["keyframes"]]}
        for d in pack["demos"]
    ]
    return out


def leaked_keys(x, found=None) -> set:
    found = set() if found is None else found
    if isinstance(x, dict):
        for k, v in x.items():
            if k in DROPPED:
                found.add(k)
            leaked_keys(v, found)
    elif isinstance(x, list):
        for v in x:
            leaked_keys(v, found)
    return found


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pack", required=True, help="pack directory containing pack.json")
    ap.add_argument("--mode", choices=["vision"], default="vision")
    ap.add_argument("--audit", action="store_true", help="only check; do not rewrite")
    args = ap.parse_args()
    p = Path(args.pack) / "pack.json"
    pack = json.loads(p.read_text())
    if args.audit:
        bad = leaked_keys(pack)
        n_img = sum(1 for d in pack["demos"] for kf in d["keyframes"] if kf.get("image"))
        ok = not bad and pack.get("schema") == "fair-pack-v1-vision"
        print(f"[strip-audit] {args.pack}: schema={pack.get('schema')} leaked={sorted(bad)} "
              f"demos={len(pack['demos'])} keyframes_with_image={n_img} -> {'OK' if ok else 'FAIL'}")
        return 0 if ok else 1
    stripped = strip_vision(pack)
    assert not leaked_keys(stripped)
    p.write_text(json.dumps(stripped, indent=1))
    n_kf = sum(len(d["keyframes"]) for d in stripped["demos"])
    print(f"[strip] {args.pack}: {len(stripped['demos'])} demos, {n_kf} keyframes, "
          f"mode={args.mode}, dropped={sorted(DROPPED & set(leaked_keys(pack)))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
