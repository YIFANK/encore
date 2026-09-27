"""Drop mid-episode idle from a LeRobotDataset; keep contact semantics.

26% of the r1 frames are still (gripper settles, post-place dwell). Stillness
teaches a policy to stay still — the freeze failure mode — and burns a quarter
of every training epoch. Runs of stillness longer than KEEP*2 lose their
middle; every run keeps its first/last KEEP frames, because the pause AROUND a
contact is the contact's signature and cutting it flush teaches teleports.
"""
from __future__ import annotations
import argparse, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

IDLE = 0.004     # rad (or m) per frame, max over dims
KEEP = 6         # frames kept at each end of a still run (0.2 s at 30 fps)

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    args = ap.parse_args()
    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from heron.record.writer import LeRobotWriter, read_sidecar
    src = Path(args.src)
    ds = LeRobotDataset(src.name, root=src)
    rows = {r["episode_index"]: r for r in read_sidecar(src)} if (src / "heron_meta.jsonl").exists() else {}
    cams = {k.removeprefix("observation.images."): tuple(v["shape"][:2])
            for k, v in ds.meta.features.items() if k.startswith("observation.images.")}
    w = LeRobotWriter(Path(args.dst), f"{src.name}_trim", cameras=cams,
                      fps=int(round(float(ds.meta.fps))), use_videos=True)
    eps = np.array(ds.hf_dataset["episode_index"])
    states = torch.stack([torch.as_tensor(x) for x in ds.hf_dataset["observation.state"]]).numpy()
    kept = dropped = 0
    for e in np.unique(eps):
        idx = np.where(eps == e)[0]
        v = np.abs(np.diff(states[idx], axis=0)).max(axis=1)
        still = np.concatenate([[False], v < IDLE])
        keep = np.ones(len(idx), bool)
        i = 0
        while i < len(idx):
            if still[i]:
                j = i
                while j < len(idx) and still[j]:
                    j += 1
                if j - i > 2 * KEEP:
                    keep[i + KEEP: j - KEEP] = False
                i = j
            else:
                i += 1
        task = ""
        for k in np.where(keep)[0]:
            it = ds[int(idx[k])]
            task = str(it.get("task", ""))
            img = {c: (np.transpose(it[f"observation.images.{c}"].numpy(), (1, 2, 0)) * 255).astype(np.uint8)
                   for c in cams}
            w.add_frame(state=it["observation.state"].numpy().astype(np.float32),
                        action=it["action"].numpy().astype(np.float32),
                        images=img, task=task)
        meta = dict(rows.get(int(e), {})); meta["source_episode"] = int(e)
        w.save_episode(task, meta=meta)
        kept += int(keep.sum()); dropped += int((~keep).sum())
    w.finalize()
    print(f"kept {kept} frames, dropped {dropped} ({100*dropped/(kept+dropped):.0f}%) -> {args.dst}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
