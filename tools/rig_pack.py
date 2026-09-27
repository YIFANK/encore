#!/usr/bin/env python3
"""Distil rig teleoperation episodes into a fair demonstration pack.

The simulated campaigns build packs from robomimic HDF5 (``tools/fair_pack.py``).
Hardware records LeRobot datasets instead, but a pack must be *identical in
shape* whichever it came from, because the whole point of the information
ablation is that only the channel differs.  This tool produces the same
schema, from the same keyframe distillation, with the same whitelist
validator — so a policy program written against a simulated pack reads a rig
pack without knowing the difference.

What changes on hardware, and why:

  * The end-effector path is not recorded; it is computed from the recorded
    joints with the rig's own forward kinematics.  That is the honest source:
    the controller's setpoints are joints, and any Cartesian trace would be a
    derived quantity either way.
  * ``actions`` are the recorded joint commands (7-dim: 6 joints + aperture),
    not Cartesian deltas.  A program replays them through ``api.act`` exactly
    as the simulated arms replay controller actions, so the step-level channel
    that R42 added carries over unchanged.
  * ``action_scale`` records the control period, since a hardware step is a
    wall-clock interval rather than a simulator tick.

    python3 tools/rig_pack.py --dataset ~/rig_data/bowl_on_plate --k 3 \\
        --language "put the bowl on the plate" --out packs/rig_bowl_on_plate
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fair_pack import validate  # noqa: E402
from fewshot_pack import keyframe_indices  # noqa: E402


def _load_episodes(root: Path, k: int, camera: str):
    """Yield (episode_index, states, actions, frames) for the first k episodes."""
    try:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    except ImportError:  # newer layout
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

    # torchvision >=0.28 removed VideoReader; pyav is the portable decoder
    ds = LeRobotDataset(repo_id=root.name, root=str(root), video_backend="torchcodec")
    key = f"observation.images.{camera}"
    by_ep: dict[int, dict[str, list]] = {}
    for i in range(len(ds)):
        item = ds[i]
        ep = int(item["episode_index"])
        if len(by_ep) >= k and ep not in by_ep:
            continue
        slot = by_ep.setdefault(ep, {"state": [], "action": [], "rgb": []})
        slot["state"].append(np.asarray(item["observation.state"], dtype=float))
        slot["action"].append(np.asarray(item["action"], dtype=float))
        img = item.get(key)
        slot["rgb"].append(None if img is None else _to_uint8(img))
    for ep in sorted(by_ep)[:k]:
        s = np.stack(by_ep[ep]["state"])
        a = np.stack(by_ep[ep]["action"])
        yield ep, s, a, by_ep[ep]["rgb"]


def _to_uint8(img) -> np.ndarray:
    arr = np.asarray(img)
    if arr.ndim == 3 and arr.shape[0] in (1, 3) and arr.shape[0] < arr.shape[-1]:
        arr = np.transpose(arr, (1, 2, 0))          # CHW -> HWC
    if arr.dtype != np.uint8:
        arr = (np.clip(arr, 0.0, 1.0) * 255).astype(np.uint8)
    return arr


def _ee_path(states: np.ndarray, kin) -> tuple[np.ndarray, np.ndarray]:
    """Forward-kinematics the recorded joints into xyz and xyz+rpy paths."""
    xyz, xyz6 = [], []
    for q in states[:, :6]:
        T = kin.fk(np.asarray(q, float)) if hasattr(kin, "fk") else None
        if T is None:
            pts = kin.link_points(np.asarray(q, float))
            p = np.asarray(pts[-1], float)
            R = np.eye(3)
        else:
            T = np.asarray(T, float)
            p, R = T[:3, 3], T[:3, :3]
        xyz.append(p)
        xyz6.append(np.concatenate([p, _rpy(R)]))
    return np.stack(xyz), np.stack(xyz6)


def _rpy(R: np.ndarray) -> np.ndarray:
    sy = float(np.hypot(R[0, 0], R[1, 0]))
    if sy > 1e-6:
        return np.array([np.arctan2(R[2, 1], R[2, 2]),
                         np.arctan2(-R[2, 0], sy),
                         np.arctan2(R[1, 0], R[0, 0])])
    return np.array([np.arctan2(-R[1, 2], R[1, 1]), np.arctan2(-R[2, 0], sy), 0.0])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", required=True, help="LeRobot dataset root")
    ap.add_argument("--k", type=int, default=3)
    ap.add_argument("--language", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--camera", default="cam_high")
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--fps", type=float, default=30.0)
    args = ap.parse_args()

    from heron.robot.kinematics import Kinematics

    kin = Kinematics()
    out = Path(args.out)
    (out / "keyframes").mkdir(parents=True, exist_ok=True)

    demos = []
    for n, (ep, states, actions, rgbs) in enumerate(
            _load_episodes(Path(args.dataset), args.k, args.camera)):
        ee, ee6 = _ee_path(states, kin)
        keys = keyframe_indices(actions, ee)
        kf = []
        for t in keys:
            rec = {"t": int(t),
                   "ee": [round(float(v), 5) for v in ee[t]],
                   "gripper_cmd": float(actions[t, -1]),
                   "gripper_state": float(states[t, -1])}
            rgb = rgbs[t] if t < len(rgbs) else None
            if rgb is not None:
                name = f"keyframes/demo{n}_t{int(t):04d}.png"
                _write_png(out / name, rgb)
                rec["image"] = name
            kf.append(rec)
        demos.append({
            "demo": n,
            "length": int(len(actions)),
            "stride": int(args.stride),
            "keyframes": kf,
            "ee_path": [[round(float(v), 5) for v in p]
                        for p in ee[::args.stride]],
            "ee_path6": [[round(float(v), 5) for v in p]
                         for p in ee6[::args.stride]],
            "action_scale": round(1.0 / float(args.fps), 5),
            "actions": [[round(float(v), 5) for v in a] for a in actions],
        })
        print(f"[rig-pack] episode {ep}: {len(actions)} steps, {len(kf)} keyframes")

    pack = {"schema": "heron-fair-pack/1", "language": args.language,
            "k": len(demos), "demos": demos}
    validate(pack)          # same whitelist the simulated packs pass
    (out / "pack.json").write_text(json.dumps(pack))
    print(f"[rig-pack] {len(demos)} demos -> {out} (validated)")
    return 0


def _write_png(path: Path, rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        import imageio.v2 as imageio

        imageio.imwrite(str(path), rgb)
    except Exception:
        from PIL import Image

        Image.fromarray(rgb).save(str(path))


if __name__ == "__main__":
    sys.exit(main())
