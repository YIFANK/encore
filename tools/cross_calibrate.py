"""Calibrate a second camera FROM the first, off one shared ChArUco view.

The fingertip method needs the arm to present poses inside the camera's view
and the ER pointer to hit a 15 mm fingertip — on cam_low's oblique view that
is 3-6 mm of noise per point, and 5/9 grid poses were not even reachable.
A ChArUco board both cameras see at the same instant needs NO arm at all and
gives sub-pixel corners:

    T_base_camL  =  T_base_camH  @  T_camH_board  @  inv(T_camL_board)

cam_high's extrinsic is the anchor (fingertip-calibrated and battle-tested);
each camera's board pose comes from solvePnP on the same corner ids.

The check is NOT circular: the derived extrinsic is scored by deprojecting
the shared corners through cam_low's own MEASURED DEPTH and comparing those
world points against cam_high's. Depth never entered the fit, so agreement is
evidence. Writes the npz only under --write and only if that rms clears 10 mm.

    .venv/bin/python tools/cross_calibrate.py --config configs/rig-first.yaml --write
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from heron.config import HeronConfig  # noqa: E402
from heron.robot.workspace import save_extrinsics  # noqa: E402


def detect_charuco(rgb, k, board, dictionary):
    """(ids, corners_px, T_cam_board) or None. 3x upscale for far boards."""
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    det = cv2.aruco.CharucoDetector(board)
    ch_corners, ch_ids = None, None
    for up in (1, 3):
        big = gray if up == 1 else cv2.resize(
            gray, None, fx=up, fy=up, interpolation=cv2.INTER_CUBIC)
        cc, ci, _, _ = det.detectBoard(big)
        if ci is not None and len(ci) >= 8:
            ch_corners, ch_ids = cc / up, ci
            break
    if ch_ids is None:
        return None
    obj = board.getChessboardCorners()[ch_ids.flatten()]
    good, rvec, tvec = cv2.solvePnP(obj, ch_corners, k, None,
                                    flags=cv2.SOLVEPNP_IPPE)
    if not good:
        return None
    t = np.eye(4)
    t[:3, :3], _ = cv2.Rodrigues(rvec)
    t[:3, 3] = tvec.flatten()
    return ch_ids.flatten(), ch_corners.reshape(-1, 2), t


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--anchor", default="cam_high")
    ap.add_argument("--target", default="cam_low")
    ap.add_argument("--sx", type=int, default=8)
    ap.add_argument("--sy", type=int, default=5)
    ap.add_argument("--square", type=float, default=0.034)
    ap.add_argument("--marker", type=float, default=0.025)
    ap.add_argument("--dict", default="5X5_50")
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()

    cfg = HeronConfig.load(args.config)
    from heron.robot.trossen import TrossenStationary
    robot = TrossenStationary(cfg)
    try:
        fa = robot.capture(args.anchor)
        ft = robot.capture(args.target)
    finally:
        robot.shutdown()
    if getattr(fa, "t_base_cam", None) is None:
        print(f"{args.anchor} has no extrinsic — it is the anchor, it must")
        return 1
    if ft.intrinsics is None:
        print(f"{args.target} frame carries no intrinsics")
        return 1

    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, f"DICT_{args.dict}"))
    board = cv2.aruco.CharucoBoard((args.sx, args.sy), args.square,
                                   args.marker, dictionary)
    ka = np.asarray(fa.intrinsics, dtype=float)
    kt = np.asarray(ft.intrinsics, dtype=float)
    da = detect_charuco(fa.rgb, ka, board, dictionary)
    dt = detect_charuco(ft.rgb, kt, board, dictionary)
    if da is None or dt is None:
        print(f"board not decoded ({args.anchor}: {'ok' if da else 'MISS'}, "
              f"{args.target}: {'ok' if dt else 'MISS'}) — place the board "
              "flat where BOTH cameras see it, check glare")
        return 1
    ids_a, px_a, t_a_board = da
    ids_t, px_t, t_t_board = dt
    shared = np.intersect1d(ids_a, ids_t)
    if len(shared) < 8:
        print(f"only {len(shared)} shared corners — need 8+")
        return 1

    t_base_anchor = np.asarray(fa.t_base_cam, dtype=float)
    t_base_target = t_base_anchor @ t_a_board @ np.linalg.inv(t_t_board)

    # Non-circular check: cam_low's own depth deprojects the shared corners;
    # the new extrinsic carries them to base; cam_high's answer is the truth.
    obj = board.getChessboardCorners()
    world_a = (t_base_anchor @ t_a_board @
               np.hstack([obj[shared], np.ones((len(shared), 1))]).T)[:3].T
    errs = []
    idx_t = {i: n for n, i in enumerate(ids_t)}
    for n, cid in enumerate(shared):
        u, v = px_t[idx_t[cid]]
        z = float(ft.depth[int(round(v)), int(round(u))])
        if not np.isfinite(z) or z <= 0.05:
            continue
        pc = np.array([(u - kt[0, 2]) * z / kt[0, 0],
                       (v - kt[1, 2]) * z / kt[1, 1], z, 1.0])
        errs.append(float(np.linalg.norm((t_base_target @ pc)[:3] - world_a[n])))
    if len(errs) < 6:
        print(f"only {len(errs)} corners have live depth — board too glossy/far?")
        return 1
    rms = float(np.sqrt(np.mean(np.square(errs))))
    print(f"shared corners: {len(shared)}, depth-checked: {len(errs)}")
    print(f"depth-vs-anchor rms: {rms * 1000:.1f} mm  "
          f"(worst {max(errs) * 1000:.1f} mm)")
    print("t_base_cam:")
    print(np.round(t_base_target, 4))
    if rms > 0.010:
        print("rms over 10 mm — NOT writing; reposition the board and retry")
        return 1
    if args.write:
        out = ROOT / "calib" / f"{args.target}_extrinsics.npz"
        save_extrinsics(out, t_base_target, rms)
        print(f"wrote {out} — set cameras.{args.target}.extrinsics_file: "
              f"calib/{args.target}_extrinsics.npz in the rig config")
    else:
        print("dry run (pass --write to save)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
