#!/usr/bin/env python3
"""Distil BIMANUAL rig teleoperation episodes into a fair demonstration pack.

Same schema family as tools/rig_pack.py (heron-fair-pack/1) with the bimanual
extension the validator whitelists: base fields carry the RIGHT arm so any
single-arm consumer reads the pack unchanged; the left arm rides in
``ee_left`` / ``*_left`` / ``ee_path_left``. Both arms' end-effector paths are
expressed in the WORLD frame (right-arm base), using the calibrated
``t_world_base`` of the left arm from the rig config — cross-arm geometry
(handover points!) is only meaningful in one frame.

Feature layout (bi_widowxai lerobot datasets):
  [0:6] left joints, [6] left gripper, [7:13] right joints, [13] right gripper.

    python3 tools/rig_pack_bi.py \
        --dataset /mnt/data2/aloha/recordings/stationary_ai_A \
        --dataset /mnt/data2/aloha/recordings/stationary_ai_B \
        --dataset /mnt/data2/aloha/recordings/stationary_ai_C \
        --language "hand the object from the right arm to the left arm" \
        --config configs/rig5090.yaml --out packs/rig_handover
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
from rig_pack import _ee_path, _write_png  # noqa: E402


def _load_bi_episode(root: Path, camera: str):
    """states/actions straight from parquet, frames straight from PyAV —
    no lerobot video stack (its decoders churn with torch/ffmpeg versions)."""
    import pandas as pd

    parts = sorted(root.glob("data/chunk-*/*.parquet"))
    df = pd.concat([pd.read_parquet(f) for f in parts], ignore_index=True)
    df = df[df["episode_index"] == int(df["episode_index"].min())]
    states = np.stack(df["observation.state"].to_numpy())
    actions = np.stack(df["action"].to_numpy())

    def frames_at(indices, cam: str = camera):
        """Decoded frames at `indices` from ONE camera, in a single pass."""
        import av

        vids = sorted(root.glob(f"videos/observation.images.{cam}/chunk-*/*.mp4"))
        want = set(int(i) for i in indices)
        got: dict[int, np.ndarray] = {}
        idx = 0
        for vp in vids:
            with av.open(str(vp)) as container:
                for frame in container.decode(video=0):
                    if idx in want:
                        got[idx] = frame.to_ndarray(format="rgb24")
                    idx += 1
                    if len(got) == len(want):
                        return got
        return got

    def cameras():
        return sorted(p.name.split("observation.images.")[-1]
                      for p in root.glob("videos/observation.images.*"))

    return states, actions, frames_at, cameras

def _prominent_peaks(g, frac: float = 0.20, gap: int = 10, scale: float = 0.0):
    """Indices of local maxima that RISE far, regardless of how high they sit.

    A level threshold is the wrong instrument here and the data says so: a
    paper cup falls out of the jaws after a few millimetres, so an operator
    often opens far less to release than they opened to approach. Detecting
    events by a midpoint crossing missed the release in 2 of 10 demonstrations
    of one task. Prominence -- the height above the higher of the two minima
    bounding the peak -- measures the rise instead of the level, which is the
    property that actually separates a gripper event from noise.
    """
    g = np.asarray(g, float)
    rng = float(g.max() - g.min())
    ref = float(scale) if scale > 0 else rng
    if ref <= 0.0 or rng < 0.10 * ref:
        return []                 # this channel never actuated
    need = frac * ref
    n = len(g)
    hits = []
    for i in range(1, n - 1):
        if not (g[i] >= g[i - 1] and g[i] > g[i + 1]):
            continue
        lo = i
        while lo > 0 and g[lo - 1] <= g[i]:
            lo -= 1
        hi = i
        while hi < n - 1 and g[hi + 1] <= g[i]:
            hi += 1
        base = max(float(g[lo:i + 1].min()), float(g[i:hi + 1].min()))
        if g[i] - base >= need:
            hits.append(i)
    keep = []
    for i in hits:
        if not keep or i - keep[-1] > gap:
            keep.append(i)
    return keep


def _gripper_events(g, hz: float, scale: float = 0.0):
    """The gripper channel's own transitions — a fact about the signal.

    Deliberately labelled "open"/"close" and not "grasp"/"release": which
    opening is a release is a conclusion about the task, and conclusions are
    the consumer's to draw.
    """
    g = np.asarray(g, float)
    opens = _prominent_peaks(g, scale=scale)
    # Boundary case (rig_brush_sweep 160149, 2026-09-01): a jaw opened BEFORE
    # recording started sits at its maximum from frame 0, which the peak
    # finder cannot see (no rising flank, and frame 0 is excluded). The
    # close that follows is real, so anchor a virtual "open" at t=0 when the
    # channel starts high and later drops by a prominent amount.
    rng = float(g.max() - g.min())
    ref = float(scale) if scale > 0 else rng
    if ref > 0 and rng >= 0.10 * ref and len(g) > 2:
        first_open = opens[0] if len(opens) else len(g)
        head = g[:first_open]
        if len(head) > 2 and g[0] - float(head.min()) >= 0.5 * ref and 0 not in opens:
            opens = [0] + list(opens)
    ev = []
    for j, t in enumerate(opens):
        ev.append({"t": int(t), "t_s": round(t / hz, 3), "kind": "open",
                   "gripper": round(float(g[t]), 5)})
        nxt = opens[j + 1] if j + 1 < len(opens) else len(g)
        seg = g[t:nxt]
        if len(seg) > 2:
            c = t + int(np.argmin(seg))
            ev.append({"t": int(c), "t_s": round(c / hz, 3), "kind": "close",
                       "gripper": round(float(g[c]), 5)})
    ev.sort(key=lambda e: e["t"])
    return ev


def _strip(frames: dict, times, hz: float, path: Path, width: int = 320) -> bool:
    """Tile frames around an event into ONE image.

    A still frame cannot show a correction; the correction is the change over
    the approach. A coding agent reads images, not video, so the approach has
    to become an image for any of it to be visible at all.
    """
    from PIL import Image, ImageDraw

    tiles = []
    for t in times:
        a = frames.get(int(t))
        if a is None:
            continue
        im = Image.fromarray(a)
        h = max(1, int(im.height * width / im.width))
        tiles.append((int(t), im.resize((width, h), Image.LANCZOS)))
    if not tiles:
        return False
    h = tiles[0][1].height
    sheet = Image.new("RGB", (width * len(tiles), h + 18), (16, 16, 20))
    dr = ImageDraw.Draw(sheet)
    for k, (t, im) in enumerate(tiles):
        sheet.paste(im, (k * width, 18))
        dr.text((k * width + 4, 3), f"t={t}  {t / hz:.2f}s", fill=(235, 235, 235))
    path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(path)
    return True


L_Q, L_G, R_Q, R_G = slice(0, 6), 6, slice(7, 13), 13


def _left_base_transform(config_path: str | None) -> np.ndarray:
    if not config_path:
        return np.eye(4)
    import yaml

    cfg = yaml.safe_load(Path(config_path).read_text())
    t = (((cfg or {}).get("arms") or {}).get("left") or {}).get("t_world_base")
    if t is None:
        print("[rig-pack-bi] WARNING: no arms.left.t_world_base in config — "
              "left arm reported in its own base frame", flush=True)
        return np.eye(4)
    return np.asarray(t, dtype=float).reshape(4, 4)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", action="append", required=True,
                    help="LeRobot dataset root (repeat; one episode taken from "
                         "each until --k demos)")
    ap.add_argument("--k", type=int, default=3)
    ap.add_argument("--language", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--camera", default="cam_high")
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--config", default=None,
                    help="rig yaml carrying arms.left.t_world_base")
    args = ap.parse_args()

    from heron.robot.kinematics import Kinematics

    kin = Kinematics()
    t_wl = _left_base_transform(args.config)
    out = Path(args.out)
    (out / "keyframes").mkdir(parents=True, exist_ok=True)

    episodes = []
    for root in args.dataset:
        states, actions, frames_at, cameras = _load_bi_episode(
            Path(root), args.camera)
        episodes.append((Path(root).name, states, actions, frames_at, cameras))
        if len(episodes) >= args.k:
            break
    if len(episodes) < args.k:
        print(f"[rig-pack-bi] only {len(episodes)} episodes found for k={args.k}",
              flush=True)

    demos = []
    for n, (src, states, actions, frames_at, cameras) in enumerate(
            episodes[: args.k]):
        if states.shape[1] < 14:
            raise SystemExit(f"{src}: state dim {states.shape[1]} is not bimanual")
        # per-arm views, each shaped like a single-arm episode (6 joints + grip)
        s_r = np.concatenate([states[:, R_Q], states[:, [R_G]]], axis=1)
        a_r = np.concatenate([actions[:, R_Q], actions[:, [R_G]]], axis=1)
        s_l = np.concatenate([states[:, L_Q], states[:, [L_G]]], axis=1)
        a_l = np.concatenate([actions[:, L_Q], actions[:, [L_G]]], axis=1)
        ee_r, ee6_r = _ee_path(s_r, kin)
        ee_l_local, ee6_l_local = _ee_path(s_l, kin)
        # left into the world frame
        ee_l = (t_wl[:3, :3] @ ee_l_local.T).T + t_wl[:3, 3]
        ee6_l = np.concatenate([ee_l, ee6_l_local[:, 3:]], axis=1)

        hz = float(args.fps)

        # the gripper channels' own transitions, per arm — these also seed
        # the keyframes: rig width commands have no sign structure for the
        # LIBERO flip reading, so the events table is the one detector
        ev_by_arm = {}
        events = []
        arm_g = (("right", a_r[:, -1]), ("left", a_l[:, -1]))
        travel = max(float(gg.max() - gg.min()) for _, gg in arm_g)
        for arm, gg in arm_g:
            ev_by_arm[arm] = _gripper_events(gg, hz, scale=travel)
            for e in ev_by_arm[arm]:
                events.append({**e, "arm": arm})
        events.sort(key=lambda e: (e["t"], e["arm"]))

        keys = sorted(
            set(keyframe_indices(a_r, ee_r,
                                 grip_events=[e["t"] for e in ev_by_arm["right"]]))
            | set(keyframe_indices(a_l, ee_l,
                                   grip_events=[e["t"] for e in ev_by_arm["left"]])))
        missed = [e for e in events
                  if all(abs(e["t"] - t) > 8 for t in keys)]
        if missed:
            raise SystemExit(
                f"[rig-pack-bi] demo {n}: {len(missed)} gripper events have no "
                f"keyframe within 8 steps: "
                f"{[(e['arm'], e['kind'], e['t']) for e in missed]}")

        # every frame any output needs, gathered once per camera
        strip_span = [list(range(max(0, e["t"] - 36), min(len(a_r), e["t"] + 10), 5))
                      for e in events]
        need = sorted(set(keys) | {t for span in strip_span for t in span})
        cams = cameras() or [args.camera]
        per_cam = {c: frames_at(need, c) for c in cams}

        kf = []
        for t in keys:
            rec = {"t": int(t), "t_s": round(t / hz, 3),
                   "ee": [round(float(v), 5) for v in ee_r[t]],
                   "ee6": [round(float(v), 5) for v in ee6_r[t]],
                   "q": [round(float(v), 5) for v in s_r[t, :6]],
                   "ee_left": [round(float(v), 5) for v in ee_l[t]],
                   "ee6_left": [round(float(v), 5) for v in ee6_l[t]],
                   "q_left": [round(float(v), 5) for v in s_l[t, :6]],
                   "gripper_cmd": float(a_r[t, -1]),
                   "gripper_state": float(s_r[t, -1]),
                   "gripper_cmd_left": float(a_l[t, -1]),
                   "gripper_state_left": float(s_l[t, -1])}
            images = {}
            for c in cams:
                rgb = per_cam[c].get(t)
                if rgb is None:
                    continue
                name = f"keyframes/demo{n}_t{int(t):04d}_{c}.png"
                _write_png(out / name, rgb)
                images[c] = name
            if images:
                rec["images"] = images
                # the overhead view stays under the old key so any consumer
                # written against the single-view schema keeps working
                rec["image"] = images.get(args.camera, sorted(images.values())[0])
            kf.append(rec)

        for e, span in zip(events, strip_span):
            strips = {}
            for c in cams:
                name = (f"keyframes/strips/demo{n}_{e['arm']}_{e['kind']}"
                        f"{e['t']:04d}_{c}.png")
                if _strip(per_cam[c], span, hz, out / name):
                    strips[c] = name
            if strips:
                e["strips"] = strips
        demos.append({
            "demo": n,
            "length": int(len(actions)),
            "stride": int(args.stride),
            "keyframes": kf,
            "ee_path": [[round(float(v), 5) for v in p] for p in ee_r[::args.stride]],
            "ee_path6": [[round(float(v), 5) for v in p] for p in ee6_r[::args.stride]],
            "ee_path_left": [[round(float(v), 5) for v in p] for p in ee_l[::args.stride]],
            "ee_path6_left": [[round(float(v), 5) for v in p] for p in ee6_l[::args.stride]],
            "arms": "right-base world frame; actions are 14-dim "
                    "[left 6q+grip, right 6q+grip] joint targets",
            "events": events,
            "speed": [round(float(v), 5) for v in
                      (np.linalg.norm(np.diff(ee_r[::args.stride], axis=0), axis=1)
                       * (float(args.fps) / args.stride))],
            "rate_hz": float(args.fps),
            "units": {
                "position": "metres, right-arm base frame",
                "orientation": "ee_path6/ee6 rows are [x, y, z, rx, ry, rz]; "
                               "EULER rpy (ZYX order), NOT an rvec — verified vs fk 2026-08-25",
                "gripper": "carriage position, not jaw aperture; the API's "
                           "aperture is about twice this",
                "speed": "metres per second, one entry per ee_path interval",
                "t": "frame index at rate_hz; t_s is the same instant in seconds",
            },
            "action_scale": round(1.0 / float(args.fps), 5),
            "actions": [[round(float(v), 5) for v in a] for a in actions],
        })
        print(f"[rig-pack-bi] demo {n} ({src}): {len(actions)} steps, "
              f"{len(kf)} keyframes x {len(cams)} views, {len(events)} gripper "
              f"events", flush=True)

    pack = {"schema": "heron-fair-pack/1", "language": args.language,
            "k": len(demos), "demos": demos}
    validate(pack)
    (out / "pack.json").write_text(json.dumps(pack))
    print(f"[rig-pack-bi] {len(demos)} demos -> {out} (validated)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
