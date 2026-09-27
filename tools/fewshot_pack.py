"""Distill LIBERO human demos into a multimodal few-shot pack.

The pack is what the coding agent actually reads: for each demo, the moments
where the DECISIONS live — gripper transitions, motion-direction breaks, start
and end — each with the agentview image, the end-effector state, and what the
gripper was doing. Everything between keyframes is interpolation the agent can
re-derive; shipping all 300 frames per demo would bury the five that matter.

    .venv/bin/python tools/fewshot_pack.py \
        --hdf5 datasets/libero_goal/turn_on_the_stove_demo.hdf5 \
        --k 3 --out packs/turn_on_the_stove

Output: pack.json (trajectory skeletons + language) and keyframes/*.png.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def keyframe_indices(actions: np.ndarray, ee: np.ndarray, cap: int = 9) -> list[int]:
    """Start, end, gripper transitions, and the sharpest heading breaks.

    The gripper channel is the demonstrator's own annotation of intent — a
    close IS "I am now holding it". Heading breaks catch the approach-descend
    and lift-carry corners that a gripper-only reading misses on tasks with no
    grasp at all (push, knob turn).
    """
    t_end = len(actions) - 1
    keys = {0, t_end}
    grip = actions[:, -1]
    for t in range(1, len(grip)):
        if np.sign(grip[t]) != np.sign(grip[t - 1]):
            keys.add(t)
    # Heading breaks: angle between consecutive smoothed velocity vectors.
    v = np.diff(ee[:, :3], axis=0)
    k = 5
    if len(v) > 2 * k:
        vs = np.stack([v[max(0, i - k) : i + 1].mean(0) for i in range(len(v))])
        n = np.linalg.norm(vs, axis=1, keepdims=True)
        vs = vs / np.maximum(n, 1e-6)
        turn = 1.0 - (vs[1:] * vs[:-1]).sum(1)
        moving = (n[1:, 0] > 1e-4) & (n[:-1, 0] > 1e-4)
        turn = np.where(moving, turn, 0.0)
        for t in np.argsort(turn)[::-1][:cap]:
            if turn[t] > 0.3 and all(abs(int(t) + 1 - kk) > 8 for kk in keys):
                keys.add(int(t) + 1)
    return sorted(keys)[: cap + 4]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hdf5", required=True)
    ap.add_argument("--k", type=int, default=3, help="demos to include")
    ap.add_argument("--out", required=True)
    # NOTE: consumers must not infer this from len(actions)//len(ee_path6)
    # — floor division under-estimates it and shifts every waypoint anchor
    # (a 9-vs-10 error compounded into a 12 cm overshoot on l1). The pack
    # now records it per demo as "stride".
    ap.add_argument("--stride", type=int, default=10,
                    help="downsample stride for the full ee path")
    ap.add_argument("--render-robosuite", action="store_true",
                    help="dataset has no images (robomimic lowdim): render "
                         "keyframes by resetting a robosuite env to states[t]")
    args = ap.parse_args()

    import h5py
    from PIL import Image

    out = Path(args.out)
    (out / "keyframes").mkdir(parents=True, exist_ok=True)
    packs = []
    with h5py.File(args.hdf5, "r") as f:
        data = f["data"]
        language = ""
        if "problem_info" in data.attrs:
            language = json.loads(data.attrs["problem_info"]).get("language_instruction", "")
        names = sorted(data.keys(), key=lambda s: int(s.split("_")[-1]))[: args.k]
        for name in names:
            d = data[name]
            obs = d["obs"]
            actions = np.asarray(d["actions"])
            ee_key = next((k for k in ("ee_states", "ee_pos", "robot0_eef_pos")
                           if k in obs), None)
            ee = np.asarray(obs[ee_key]) if ee_key else np.zeros((len(actions), 3))
            grip_key = next((k for k in ("gripper_states", "robot0_gripper_qpos")
                             if k in obs), None)
            grip = np.asarray(obs[grip_key]) if grip_key else None
            img_key = next((k for k in ("agentview_rgb", "agentview_image")
                            if k in obs), None)
            keys = keyframe_indices(actions, ee)
            renderer = None
            if img_key is None and args.render_robosuite:
                import sys as _sys
                _sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
                from heron.robot.robosuite_env import RobosuiteRobot
                renderer = RobosuiteRobot(args.hdf5, episode=0)
                states = np.asarray(d["states"])
            frames = []
            for t in keys:
                fname = f"{name}_t{t:04d}.png"
                if renderer is not None:
                    renderer.set_init_state(states[t])
                    Image.fromarray(renderer.capture("cam_high").rgb).save(
                        out / "keyframes" / fname)
                elif img_key is not None:
                    # LIBERO stores agentview upside down relative to render.
                    Image.fromarray(np.asarray(obs[img_key][t])[::-1]).save(
                        out / "keyframes" / fname)
                frames.append({
                    "t": int(t),
                    "image": (f"keyframes/{fname}"
                              if (img_key or renderer is not None) else None),
                    "ee": [round(float(x), 4) for x in ee[t][:6]],
                    "gripper_cmd": round(float(actions[t, -1]), 3),
                    "gripper_state": ([round(float(x), 4) for x in grip[t]]
                                      if grip is not None else None),
                })
            if renderer is not None:
                renderer.shutdown()
            packs.append({
                "demo": name,
                "length": int(len(actions)),
                "stride": int(args.stride),
                "keyframes": frames,
                "ee_path": [[round(float(x), 4) for x in ee[t][:3]]
                            for t in range(0, len(ee), args.stride)],
                # Full 6d (pos + axis-angle) at the same stride, with the
                # matching gripper command: enough to TRACK the demo, not just
                # summarize it — the drawer taught that position-only waypoints
                # lose exactly the DOF the task depends on.
                "ee_path6": [[round(float(x), 4) for x in ee[t][:6]]
                             + [round(float(actions[min(t, len(actions) - 1), -1]), 2)]
                             for t in range(0, len(ee), args.stride)],
                "action_scale": [round(float(np.abs(actions[:, i]).mean()), 4)
                                 for i in range(actions.shape[1])],
                # The raw action sequence. Deltas are position-invariant to
                # first order, so a grounded program can align the start and
                # replay a demo PHASE at native speed — the micro-corrections
                # that keep a pushed dish engaged ride along for free, and 90
                # raw steps cost less horizon than 3 P-controlled hops.
                "actions": [[round(float(x), 3) for x in a] for a in actions],
            })
    (out / "pack.json").write_text(json.dumps({
        "source": str(args.hdf5),
        "language": language,
        "demos": packs,
    }, indent=1))
    n_img = len(list((out / "keyframes").glob("*.png")))
    print(f"[pack] {len(packs)} demos, {n_img} keyframe images -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
