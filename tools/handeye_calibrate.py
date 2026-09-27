#!/usr/bin/env python3
"""Wrist-camera hand-eye calibration (T_ee_cam) by hand-guided poses.

The ChArUco board lies FLAT on the table and never moves. The RIGHT arm floats
in gravity-comp; you hand-guide the wrist camera to look at the board from a
different position AND tilt each time and press Enter to record (FK pose +
image). 'q' + Enter finishes. Offline the script then:
  1. calibrates the wrist camera INTRINSICS from the board corners,
  2. solves PnP per view (board pose in camera frame),
  3. solves hand-eye AX=XB  ->  T_ee_cam,
and saves to calibration/right_wrist_handeye.npz.

Every recorded view is checkpointed to disk IMMEDIATELY (raw npz + jpg), so a
solver failure never costs you the collection session. Re-solve without the
robot at all:

    .venv/bin/python tools/handeye_calibrate.py --solve-from calibration/handeye_raw

Pose diversity matters twice over: intrinsics from a planar board need varied
TILT (a fronto-parallel-only set is degenerate and yields a nonsense focal
length), and AX=XB needs varied ORIENTATION (pure translation has no solution).
Aim for 12-15 views: tilt the wrist 20-45 deg in different directions, vary
distance 20-40 cm, and keep the board mostly in frame.

MUST RUN FROM A GUI TERMINAL on the robot Mac (ssh has no camera permission).
SAFETY: hands on the arm at all times, estop running, dashboard cameras stopped.

    .venv/bin/python tools/handeye_calibrate.py --square 0.034
"""
from __future__ import annotations

import argparse
import select
import sys
import time
from pathlib import Path

import cv2
import numpy as np

MIN_VIEWS = 6
MIN_CORNERS = 8


def make_board(args):
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, f"DICT_{args.dict}"))
    board = aruco.CharucoBoard((args.sx, args.sy), args.square, args.marker, dictionary)
    params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return board, aruco.CharucoDetector(board, detectorParams=params)


def _sharpness(gray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def collect(args, board, detector, raw_dir: Path):
    import trossen_arm as ta
    import yaml

    cfg = yaml.safe_load(open(args.config))
    ip = cfg["arms"]["right"]["ip"]
    cam_index = cfg["cameras"][args.camera]["index"]

    cap = cv2.VideoCapture(cam_index, cv2.CAP_AVFOUNDATION)
    ok, probe = cap.read()
    if not ok:
        print(f"camera index {cam_index} gave no frame.\n"
              "  - running over ssh? macOS blocks camera there: use a GUI Terminal\n"
              "  - dashboard still holding the camera? stop it first")
        sys.exit(1)
    h, w = probe.shape[:2]
    print(f"camera {args.camera} (index {cam_index}): {w}x{h}")
    print(f"board: {args.sx}x{args.sy}, square {args.square*1000:.1f} mm, "
          f"marker {args.marker*1000:.1f} mm, DICT_{args.dict}")
    print("  (if the printed square does NOT measure that, rerun with --square <meters>)")

    raw_dir.mkdir(parents=True, exist_ok=True)
    print(f"checkpointing every view to {raw_dir}/")

    print(f"\nconnecting to right follower @ {ip} …")
    drv = ta.TrossenArmDriver()
    drv.configure(ta.Model.wxai_v0, ta.StandardEndEffector.wxai_v0_follower, ip, False)
    n_saved = 0
    input("\nHANDS ON THE ARM. Press Enter to enable gravity-comp float … ")
    try:
        drv.set_all_modes(ta.Mode.external_effort)
        drv.set_all_external_efforts(np.zeros(drv.get_num_joints()).tolist(), 0.0, False)
        print("floating. Guide the WRIST CAMERA to view the board; Enter records a view;")
        print("'q'+Enter finishes. VARY THE TILT between views, not just position.\n")
        while True:
            for _ in range(4):  # flush stale frames so the image matches the held pose
                cap.read()
            ok, img = cap.read()
            if not ok:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            _, ch_ids, _, _ = detector.detectBoard(gray)
            n = 0 if ch_ids is None else len(ch_ids)
            print(f"\r  corners: {n:2d}   sharpness: {_sharpness(gray):6.0f}   "
                  f"saved: {n_saved:2d}   (Enter=record, q=done)   ", end="")
            if not select.select([sys.stdin], [], [], 0.2)[0]:
                continue
            if sys.stdin.readline().strip().lower() == "q":
                break
            # HOLD STILL: re-grab now that the hand has settled on the Enter press.
            time.sleep(0.3)
            for _ in range(3):
                cap.read()
            ok, img = cap.read()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ch_corners, ch_ids, _, _ = detector.detectBoard(gray)
            n = 0 if ch_ids is None else len(ch_ids)
            if n < MIN_CORNERS:
                print(f"\n  only {n} corners — adjust and retry (need >= {MIN_CORNERS})")
                continue
            sharp = _sharpness(gray)
            if sharp < args.min_sharpness:
                print(f"\n  too blurry ({sharp:.0f} < {args.min_sharpness}) — hold still and retry")
                continue
            pose = np.asarray(drv.get_cartesian_positions())
            np.savez(raw_dir / f"view{n_saved:03d}.npz", pose=pose,
                     corners=np.asarray(ch_corners), ids=np.asarray(ch_ids))
            cv2.imwrite(str(raw_dir / f"view{n_saved:03d}.jpg"), img)
            n_saved += 1
            print(f"\n  view {n_saved} saved ({n} corners, sharpness {sharp:.0f}) "
                  "— now CHANGE THE TILT")
    finally:
        print("\nswitching back to position hold, then idle …")
        try:
            drv.set_all_modes(ta.Mode.position)
            drv.set_all_positions(np.asarray(drv.get_all_positions()), 0.5, True)
        except Exception:
            pass
        drv.cleanup()
        cap.release()
    print(f"collection done: {n_saved} views in {raw_dir}/")
    return load_raw(raw_dir)


def load_raw(raw_dir: Path):
    views = []
    for f in sorted(raw_dir.glob("view*.npz")):
        d = np.load(f)
        img = cv2.imread(str(f.with_suffix(".jpg")))
        views.append({"pose": d["pose"], "corners": d["corners"], "ids": d["ids"],
                      "img": img, "name": f.stem})
    return views


def _pose_to_T(p) -> np.ndarray:
    """Trossen cartesian [x y z rx ry rz] (angle-axis) -> 4x4 T_base_gripper."""
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(p[3:6], dtype=float))[0]
    T[:3, 3] = np.asarray(p[:3], dtype=float)
    return T


def solve(args, board, views):
    from heron.robot.handeye import calibrate_hand_eye

    size = views[0]["img"].shape[1::-1]
    obj_all, img_all, keep = [], [], []
    for v in views:
        obj, imgp = board.matchImagePoints(v["corners"], v["ids"])
        if obj is None or len(obj) < MIN_CORNERS:
            print(f"  {v['name']}: too few matched points, dropped")
            continue
        obj_all.append(obj); img_all.append(imgp); keep.append(v)
    print(f"\ncalibrating intrinsics from {len(keep)} views ({size[0]}x{size[1]}) …")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_all, img_all, size, None, None)
    print(f"  overall rms = {rms:.3f} px")

    # Per-view reprojection error: one bad view (blur, mismatched pose) poisons
    # the whole fit, so surface it and optionally drop it.
    per_view = []
    for obj, imgp, rv, tv in zip(obj_all, img_all, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rv, tv, K, dist)
        per_view.append(float(np.sqrt(np.mean(np.sum(
            (proj.reshape(-1, 2) - imgp.reshape(-1, 2)) ** 2, axis=1)))))
    for v, e in zip(keep, per_view):
        flag = "  <-- OUTLIER" if e > args.max_view_rms else ""
        print(f"    {v['name']}: {e:6.2f} px{flag}")

    good = [i for i, e in enumerate(per_view) if e <= args.max_view_rms]
    if len(good) < len(keep) and len(good) >= MIN_VIEWS:
        print(f"  refitting without {len(keep) - len(good)} outlier view(s) …")
        obj_all = [obj_all[i] for i in good]
        img_all = [img_all[i] for i in good]
        keep = [keep[i] for i in good]
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_all, img_all, size, None, None)
        print(f"  refit rms = {rms:.3f} px")

    print(f"  fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
    fov = 2 * np.degrees(np.arctan(size[0] / (2 * K[0, 0])))
    print(f"  horizontal FOV = {fov:.1f} deg   (D405 colour ~= 84 deg)")
    if rms > 1.5:
        print("  WARNING: rms > 1.5 px — intrinsics are NOT trustworthy.")
        print("           usual causes: too little TILT variety, motion blur,")
        print("           or --square not matching the printed board.")
    if abs(K[0, 2] - size[0] / 2) > size[0] * 0.15 or abs(K[1, 2] - size[1] / 2) > size[1] * 0.15:
        print("  WARNING: principal point far from image centre — fit likely diverged.")

    T_base_gripper, T_cam_target = [], []
    for v, obj, imgp in zip(keep, obj_all, img_all):
        ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, dist)
        if not ok:
            continue
        T_c_t = np.eye(4)
        T_c_t[:3, :3] = cv2.Rodrigues(rvec)[0]
        T_c_t[:3, 3] = tvec.reshape(3)
        T_cam_target.append(T_c_t)
        T_base_gripper.append(_pose_to_T(v["pose"]))
    print(f"  PnP succeeded on {len(T_cam_target)} views")
    if len(T_cam_target) < MIN_VIEWS:
        print("not enough views for hand-eye"); return 1

    X = calibrate_hand_eye(T_base_gripper, T_cam_target)   # T_gripper_cam

    # The board never moved, so every view must place it at the SAME base-frame
    # pose. The spread of those estimates IS the calibration error.
    origins = np.array([(Tbg @ X @ Tct)[:3, 3]
                        for Tbg, Tct in zip(T_base_gripper, T_cam_target)])
    spread = np.linalg.norm(origins - origins.mean(0), axis=1)
    print(f"\nT_ee_cam:\n{np.round(X, 4)}")
    print(f"  camera sits {np.linalg.norm(X[:3, 3])*100:.1f} cm from the EE origin "
          "(D405 on a WXAI wrist ~= 3-10 cm)")
    print(f"  board-origin consistency: mean {spread.mean()*1000:.1f} mm, "
          f"max {spread.max()*1000:.1f} mm")
    print(f"  board origin in base frame = {np.round(origins.mean(0), 4).tolist()}")
    verdict = ("GOOD" if spread.mean() < 0.01 else
               "USABLE" if spread.mean() < 0.02 else "POOR — recollect with more tilt variety")
    print(f"  VERDICT: {verdict}")

    out = Path(args.out); out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, T_ee_cam=X, K=K, dist=dist, intrinsics_rms=rms,
             board_origin_base=origins.mean(0), consistency_mm=spread * 1000)
    print(f"saved -> {out}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/abaka.yaml")
    ap.add_argument("--camera", default="cam_right_wrist")
    ap.add_argument("--sx", type=int, default=8)
    ap.add_argument("--sy", type=int, default=5)
    ap.add_argument("--square", type=float, default=0.034, help="MEASURED square side (m)")
    ap.add_argument("--marker", type=float, default=0.025)
    ap.add_argument("--dict", default="5X5_50")
    ap.add_argument("--out", default="calibration/right_wrist_handeye.npz")
    ap.add_argument("--raw-dir", default="calibration/handeye_raw")
    ap.add_argument("--solve-from", help="skip collection; solve from this raw dir")
    ap.add_argument("--min-sharpness", type=float, default=40.0)
    ap.add_argument("--max-view-rms", type=float, default=2.0)
    args = ap.parse_args()

    board, detector = make_board(args)
    if args.solve_from:
        views = load_raw(Path(args.solve_from))
        print(f"loaded {len(views)} saved views from {args.solve_from}")
    else:
        views = collect(args, board, detector, Path(args.raw_dir))
    if len(views) < MIN_VIEWS:
        print(f"only {len(views)} views — need >= {MIN_VIEWS}; nothing solved")
        return 1
    return solve(args, board, views)


if __name__ == "__main__":
    sys.exit(main())
