"""Build a SANITIZED demo pack for c2 (information-fair campaign).

Same keyframe distillation as tools/fewshot_pack.py, but the output is
whitelist-validated so nothing privileged can ride along:

  MAY contain : K=3 demos' keyframe RGB images, EEF/proprioception paths,
                gripper commands/state, raw action sequences, task language.
  MUST NOT    : simulator states, object poses, BDDL/init references, GT
                traces, or ANY filesystem path (the c1 pack.json carried
                "source": /mnt/.../demo.hdf5 — a curious agent could follow it
                to the HDF5's `states` and replay the whole layout).

The builder itself may read the hdf5 (it runs coordinator-side, not in the
agent's namespace); only its OUTPUT is agent-visible.

    .venv/bin/python tools/fair_pack.py --hdf5 <demo.hdf5> --k 3 \
        --language "<intent sentence>" --out packs/c2_<cell>
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fewshot_pack import keyframe_indices  # noqa: E402  (same distillation)

DEMO_KEYS = {"demo", "length", "stride", "keyframes", "ee_path", "ee_path6",
             "action_scale", "actions",
             # bimanual extension (rig_pack_bi): the base fields stay the
             # RIGHT arm so single-arm consumers read a bi pack unchanged
             "ee_path_left", "ee_path6_left", "arms"}
KEYFRAME_KEYS = {"t", "image", "ee", "gripper_cmd", "gripper_state",
                 "ee_left", "gripper_cmd_left", "gripper_state_left"}
TOP_KEYS = {"schema", "language", "k", "demos"}
PATHLIKE = re.compile(r"(/mnt/|/home/|\.hdf5|\.h5\b|\.bddl|init_state|states)",
                      re.IGNORECASE)


def validate(pack: dict) -> None:
    """Refuse to write a pack that leaks structure or paths."""
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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hdf5", required=True)
    ap.add_argument("--k", type=int, default=3)
    ap.add_argument("--stride", type=int, default=10)
    ap.add_argument("--language", default="",
                    help="override the intent sentence (re-authored cells)")
    ap.add_argument("--render-robosuite", action="store_true",
                    help="dataset has no images (robomimic lowdim): render "
                         "keyframes by resetting a robosuite env to states[t] "
                         "(builder-side only; states never enter the pack)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    import h5py
    from PIL import Image

    out = Path(args.out)
    (out / "keyframes").mkdir(parents=True, exist_ok=True)
    demos = []
    with h5py.File(args.hdf5, "r") as f:
        data = f["data"]
        language = args.language
        if not language and "problem_info" in data.attrs:
            language = json.loads(data.attrs["problem_info"]).get("language_instruction", "")
        names = sorted(data.keys(), key=lambda s: int(s.split("_")[-1]))[: args.k]
        for i, name in enumerate(names):
            d = data[name]
            obs = d["obs"]
            actions = np.asarray(d["actions"])
            ee_key = next((k for k in ("ee_states", "ee_pos", "robot0_eef_pos") if k in obs), None)
            ee = np.asarray(obs[ee_key]) if ee_key else np.zeros((len(actions), 3))
            grip_key = next((k for k in ("gripper_states", "robot0_gripper_qpos") if k in obs), None)
            grip = np.asarray(obs[grip_key]) if grip_key else None
            img_key = next((k for k in ("agentview_rgb", "agentview_image") if k in obs), None)
            renderer = None
            if img_key is None and args.render_robosuite:
                sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
                from heron.robot.robosuite_env import RobosuiteRobot
                renderer = RobosuiteRobot(args.hdf5, episode=0)
                states = np.asarray(d["states"])
            frames = []
            for t in keyframe_indices(actions, ee):
                fname = f"demo{i}_t{t:04d}.png"
                if renderer is not None:
                    renderer.set_init_state(states[t])
                    Image.fromarray(renderer.capture("cam_high").rgb).save(
                        out / "keyframes" / fname)
                elif img_key is not None:
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
            demos.append({
                "demo": f"demo{i}",
                "length": int(len(actions)),
                "stride": int(args.stride),
                "keyframes": frames,
                "ee_path": [[round(float(x), 4) for x in ee[t][:3]]
                            for t in range(0, len(ee), args.stride)],
                "ee_path6": [[round(float(x), 4) for x in ee[t][:6]]
                             + [round(float(actions[min(t, len(actions) - 1), -1]), 2)]
                             for t in range(0, len(ee), args.stride)],
                "action_scale": [round(float(np.abs(actions[:, j]).mean()), 4)
                                 for j in range(actions.shape[1])],
                "actions": [[round(float(x), 3) for x in a] for a in actions],
            })

    pack = {"schema": "fair-pack-v1", "language": language, "k": len(demos),
            "demos": demos}
    validate(pack)
    (out / "pack.json").write_text(json.dumps(pack, indent=1))
    n_img = len(list((out / "keyframes").glob("*.png")))
    print(f"[fair-pack] {len(demos)} demos, {n_img} keyframes -> {out} (validated)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
