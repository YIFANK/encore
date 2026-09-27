#!/usr/bin/env python3
"""Build a sanitized bimanual demo pack from RoboDojo HDF5 episodes.

Same contract as tools/fair_pack.py (fair-pack-v1, whitelist-validated, no
paths, no simulator state).  RoboDojo episodes carry three RGB views (head,
left wrist, right wrist), per-arm end-effector poses [x, y, z, qw, qx, qy, qz]
in the world frame, per-arm gripper openness in [0, 1], and 25 Hz actions.

Bimanual convention (rig_pack_bi): the base fields are the RIGHT arm, the
``*_left`` fields the left arm, so single-arm consumers read a bi pack
unchanged.  Keyframes are selected per arm (gripper open/close transitions +
heading breaks of the moving arm) and unioned.  Each keyframe stores the head
view as ``image`` and all three views under ``images``.

    .venv/bin/python tools/fair_pack_robodojo.py \
        --episodes data/put_bottles_into_dustbin/episode_0000000.hdf5 [...] \
        --language "<intent>" --out packs/rd_put_bottles_into_dustbin_k3
"""
from __future__ import annotations

import argparse
import io
import json
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fewshot_pack import keyframe_indices  # noqa: E402

CAMS = ("cam_head", "cam_left_wrist", "cam_right_wrist")
DEMO_KEYS = {"demo", "length", "stride", "keyframes", "ee_path", "ee_path6", "action_scale", "actions",
             "ee_path_left", "ee_path6_left", "arms", "action_layout", "control_hz"}
KEYFRAME_KEYS = {"t", "image", "images", "ee", "gripper_cmd", "gripper_state",
                 "ee_left", "gripper_cmd_left", "gripper_state_left"}
TOP_KEYS = {"schema", "language", "k", "demos", "cameras"}
PATHLIKE = re.compile(r"(/mnt/|/home/|\.hdf5|\.h5\b|\.bddl|init_state|states)", re.IGNORECASE)


def validate(pack: dict) -> None:
    assert set(pack) <= TOP_KEYS, f"foreign top-level keys: {set(pack) - TOP_KEYS}"
    for d in pack["demos"]:
        assert set(d) <= DEMO_KEYS, f"foreign demo keys: {set(d) - DEMO_KEYS}"
        for kf in d["keyframes"]:
            assert set(kf) <= KEYFRAME_KEYS, f"foreign keyframe keys: {set(kf) - KEYFRAME_KEYS}"

    def sweep(x, where="pack"):
        if isinstance(x, str) and PATHLIKE.search(x) and not x.startswith("keyframes/"):
            raise AssertionError(f"path-like string in {where}: {x!r}")
        if isinstance(x, dict):
            for k, v in x.items():
                sweep(k, where), sweep(v, where)
        if isinstance(x, list):
            for v in x:
                sweep(v, where)
    sweep(pack)


def quat_wxyz_to_rpy(q):
    w, x, y, z = [float(v) for v in q]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


def pose7_to_6(p):
    p = np.asarray(p, float).reshape(-1, 7)
    return np.concatenate([p[:, :3], np.array([quat_wxyz_to_rpy(q) for q in p[:, 3:7]])], axis=1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--episodes", nargs="+", required=True, help="RoboDojo episode_*.hdf5 files, in demo order")
    ap.add_argument("--language", default="", help="intent sentence (default: the episode's own instruction)")
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--close-threshold", type=float, default=0.7, help="openness below this = closed")
    ap.add_argument("--out", required=True)
    ap.add_argument("--images-only", action="store_true",
                    help="modality ablation: keep the keyframe images (and their order), drop every pose, gripper and action number")
    args = ap.parse_args()

    import h5py
    from PIL import Image

    out = Path(args.out)
    (out / "keyframes").mkdir(parents=True, exist_ok=True)
    demos, language = [], args.language
    for i, ep in enumerate(args.episodes):
        with h5py.File(ep, "r") as f:
            if not language:
                language = str(np.asarray(f["instruction"]).item().decode() if isinstance(np.asarray(f["instruction"]).item(), bytes) else np.asarray(f["instruction"]).item())
            st, ac = f["state"], f["action"]
            T = int(st["left_ee_poses"].shape[0])
            pose = {s: np.asarray(st[f"{s}_ee_poses"]) for s in ("left", "right")}
            grip_state = {s: np.asarray(st[f"{s}_ee_joint_states"]).reshape(T, -1)[:, 0] for s in ("left", "right")}
            grip_cmd = {s: np.asarray(ac[f"{s}_ee_joint_states"]).reshape(T, -1)[:, 0] for s in ("left", "right")}
            act_pose = {s: np.asarray(ac[f"{s}_ee_poses"]) for s in ("left", "right")}
            colors = {c: f["vision"][c]["colors"] for c in CAMS if c in f["vision"]}
            hz = int(np.asarray(f["additional_info"]["frequency"]).item()) if "additional_info" in f else 25

            ee6 = {s: pose7_to_6(pose[s]) for s in ("left", "right")}
            # per-arm keyframes: synthetic +/-1 grip signal + xyz heading breaks; only arms that move
            keys = {0, T - 1}
            moving = {s: float(np.abs(np.diff(pose[s][:, :3], axis=0)).sum()) > 0.05 for s in ("left", "right")}
            for s in ("left", "right"):
                if not moving[s]:
                    continue
                sig = np.where(grip_cmd[s] > args.close_threshold, 1.0, -1.0).reshape(-1, 1)
                keys |= set(keyframe_indices(sig, pose[s][:, :3]))
            keys = sorted(keys)

            frames = []
            for t in keys:
                paths = {}
                for c, ds in colors.items():
                    fname = f"demo{i}_t{t:04d}_{c}.png"
                    Image.open(io.BytesIO(bytes(ds[t]))).convert("RGB").save(out / "keyframes" / fname)
                    paths[c] = f"keyframes/{fname}"
                frames.append({
                    "t": int(t),
                    "image": paths.get("cam_head"),
                    "images": paths,
                    "ee": [round(float(x), 4) for x in ee6["right"][t]],
                    "gripper_cmd": round(float(grip_cmd["right"][t]), 3),
                    "gripper_state": [round(float(grip_state["right"][t]), 4)],
                    "ee_left": [round(float(x), 4) for x in ee6["left"][t]],
                    "gripper_cmd_left": round(float(grip_cmd["left"][t]), 3),
                    "gripper_state_left": [round(float(grip_state["left"][t]), 4)],
                })
            # actions: [L xyz, L rpy, L grip, R xyz, R rpy, R grip] (absolute targets, world frame)
            a6 = {s: pose7_to_6(act_pose[s]) for s in ("left", "right")}
            actions = np.concatenate([a6["left"], grip_cmd["left"].reshape(-1, 1),
                                      a6["right"], grip_cmd["right"].reshape(-1, 1)], axis=1)
            sl = slice(0, T, args.stride)
            demos.append({
                "demo": f"demo{i}", "length": T, "stride": int(args.stride), "control_hz": hz,
                "arms": [s for s in ("left", "right") if moving[s]] or ["left", "right"],
                "keyframes": frames,
                "ee_path": [[round(float(x), 4) for x in r[:3]] for r in ee6["right"][sl]],
                "ee_path6": [[round(float(x), 4) for x in r] + [round(float(g), 2)]
                             for r, g in zip(ee6["right"][sl], grip_cmd["right"][sl])],
                "ee_path_left": [[round(float(x), 4) for x in r[:3]] for r in ee6["left"][sl]],
                "ee_path6_left": [[round(float(x), 4) for x in r] + [round(float(g), 2)]
                                  for r, g in zip(ee6["left"][sl], grip_cmd["left"][sl])],
                "action_layout": "left_xyz(3) left_rpy(3) left_grip_open(1) right_xyz(3) right_rpy(3) right_grip_open(1); absolute world-frame targets at control_hz",
                "action_scale": [round(float(np.abs(actions[:, j]).mean()), 4) for j in range(actions.shape[1])],
                "actions": [[round(float(x), 3) for x in a] for a in actions],
            })
    if args.images_only:
        for d in demos:
            for k in ("ee_path", "ee_path6", "ee_path_left", "ee_path6_left", "actions", "action_scale", "action_layout"):
                d.pop(k, None)
            d["keyframes"] = [{"t": kf["t"], "image": kf["image"], "images": kf["images"]} for kf in d["keyframes"]]
    pack = {"schema": "fair-pack-v1", "language": language, "k": len(demos), "cameras": list(CAMS), "demos": demos}
    validate(pack)
    (out / "pack.json").write_text(json.dumps(pack, indent=1))
    n_img = len(list((out / "keyframes").glob("*.png")))
    print(f"[fair-pack-rd] {len(demos)} demos, {n_img} keyframe images -> {out} (validated); language={language!r}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
