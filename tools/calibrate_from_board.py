#!/usr/bin/env python3
"""Calibrate a fixed camera against the ChArUco board already on the table.

Driving the arm to a grid and finding its fingertip in the image is the
bootstrap method — it needs nothing but the robot, but the fingertip is a small,
self-occluding, shadow-casting target and the correspondences come out noisy
(41 mm RMS on this rig). Once the wrist hand-eye is solved, the board itself is
a far better ruler:

    T_base_board = T_base_gripper . T_ee_cam . T_cam_board

is known from the hand-eye session, so every chessboard corner has a known
base-frame position. Detecting the board in the fixed camera then yields dozens
of sub-pixel correspondences from ONE frame, with no motion at all.

Accuracy is bounded by the hand-eye consistency (~5 mm here), which still beats
fingertip differencing by a wide margin.

    tools/gui_run.sh '.venv/bin/python tools/calibrate_from_board.py --camera cam_high'
"""
from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


def board_pose_in_base(args, board, detector):
    """Average T_base_board over the saved hand-eye views."""
    he = np.load(args.handeye)
    T_ee_cam, K, dist = he["T_ee_cam"], he["K"], he["dist"]
    files = sorted(glob.glob(str(Path(args.raw_dir) / "view*.npz")))
    if not files:
        sys.exit(f"no hand-eye views in {args.raw_dir}")
    Ts, origins = [], []
    for f in files:
        d = np.load(f)
        obj, imgp = board.matchImagePoints(d["corners"], d["ids"])
        if obj is None or len(obj) < 8:
            continue
        ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, dist)
        if not ok:
            continue
        T_cam_board = np.eye(4)
        T_cam_board[:3, :3] = cv2.Rodrigues(rvec)[0]
        T_cam_board[:3, 3] = tvec.reshape(3)
        p = d["pose"]
        T_base_gripper = np.eye(4)
        T_base_gripper[:3, :3] = cv2.Rodrigues(np.asarray(p[3:6], float))[0]
        T_base_gripper[:3, 3] = np.asarray(p[:3], float)
        T = T_base_gripper @ T_ee_cam @ T_cam_board
        Ts.append(T)
        origins.append(T[:3, 3])
    if len(Ts) < 4:
        sys.exit(f"only {len(Ts)} usable hand-eye views")
    origins = np.asarray(origins)
    spread = np.linalg.norm(origins - origins.mean(0), axis=1)
    print(f"board pose from {len(Ts)} hand-eye views; origin spread "
          f"mean {spread.mean()*1000:.1f} mm, max {spread.max()*1000:.1f} mm")
    # Average rotations by projecting the mean matrix back onto SO(3).
    U, _, Vt = np.linalg.svd(np.mean([T[:3, :3] for T in Ts], axis=0))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    T_base_board = np.eye(4)
    T_base_board[:3, :3] = R
    T_base_board[:3, 3] = origins.mean(0)
    return T_base_board


PARK_JOINTS = [-1.4, 0.9, 1.9, 0.0, 0.0, 0.0]   # swung right, clear of the table view


def _connect(cfg, side="right"):
    import trossen_arm as ta  # noqa: PLC0415

    drv = ta.TrossenArmDriver()
    drv.configure(ta.Model.wxai_v0, ta.StandardEndEffector.wxai_v0_follower,
                  cfg["arms"][side]["ip"], False)
    drv.set_all_modes(ta.Mode.position)
    return ta, drv


def park_arm(cfg, side="right") -> bool:
    """Get the arm out of the camera's view of the board."""
    try:
        ta, drv = _connect(cfg, side)
    except Exception as e:
        print(f"could not park the arm ({type(e).__name__}); make sure it is not "
              "occluding the board")
        return False
    try:
        scale = float(cfg.get("safety", {}).get("speed_scale", 1.0)) or 1.0
        drv.set_all_positions(np.array([*PARK_JOINTS, drv.get_gripper_position()]),
                              5.0 / scale, True)
        print("arm parked clear of the board")
        return True
    finally:
        drv.cleanup()


def probe_fingertip(args, cfg, side="right"):
    """One fingertip observation: (pixel, world xy) straight from kinematics."""
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from heron.robot.fingertip import pinch_point_from_pair  # noqa: PLC0415

    ta, drv = _connect(cfg, side)
    idx = cfg["cameras"][args.camera]["index"]
    scale = float(cfg.get("safety", {}).get("speed_scale", 1.0)) or 1.0
    table_z = float(cfg.get("table_z", 0.0))
    target = np.array([0.32, 0.0, table_z + 0.03])
    rvec = np.asarray(cfg["arms"][side].get("approach_rvec", [0.0, 1.5708, 0.0]), float)
    cap = None
    try:
        print(f"probing with the fingertip at {np.round(target, 3).tolist()} …")
        drv.set_cartesian_positions(np.concatenate([target, rvec]),
                                    ta.InterpolationSpace.joint, 6.0 / scale, True)
        cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
        for _ in range(10):
            cap.read()

        def shot():
            for _ in range(4):
                cap.read()
            ok, img = cap.read()
            return cv2.cvtColor(img, cv2.COLOR_BGR2RGB) if ok else None

        drv.set_gripper_position(0.03, 1.5 / scale, True)
        opened = shot()
        drv.set_gripper_position(0.005, 1.5 / scale, True)
        closed = shot()
        if opened is None or closed is None:
            return None
        hit = pinch_point_from_pair(opened, closed)
        if hit is None:
            print("  fingertip not detected in the probe frame")
            return None
        xy = np.asarray(drv.get_cartesian_positions(), float)[:2]
        print(f"  fingertip at px=({hit[0]},{hit[1]}), kinematics says "
              f"({xy[0]:+.3f},{xy[1]:+.3f})")
        return (float(hit[0]), float(hit[1])), xy
    except Exception as e:
        print(f"  probe failed: {type(e).__name__}: {str(e)[:120]}")
        return None
    finally:
        if cap is not None:
            cap.release()
        try:
            drv.set_all_positions(np.array([*PARK_JOINTS, drv.get_gripper_position()]),
                                  5.0 / scale, True)
        except Exception:
            pass
        drv.cleanup()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/abaka.yaml")
    ap.add_argument("--camera", default="cam_high")
    ap.add_argument("--handeye", default="calibration/right_wrist_handeye.npz")
    ap.add_argument("--raw-dir", default="calibration/handeye_raw")
    ap.add_argument("--sx", type=int, default=8)
    ap.add_argument("--sy", type=int, default=5)
    ap.add_argument("--square", type=float, default=0.034)
    ap.add_argument("--marker", type=float, default=0.025)
    ap.add_argument("--dict", default="5X5_50")
    ap.add_argument("--image", help="use this image instead of grabbing a frame")
    ap.add_argument("--out", help="output npz (default: config's homography_file)")
    ap.add_argument("--no-park", action="store_true",
                    help="do not move the arm out of the camera's view first")
    # A probe measured once stays valid as long as neither the camera nor the
    # board moves, so a re-fit needs no robot at all.
    ap.add_argument("--probe-px", type=float, nargs=2, metavar=("U", "V"),
                    help="recorded fingertip pixel, to skip the live probe")
    ap.add_argument("--probe-xy", type=float, nargs=2, metavar=("X", "Y"),
                    help="world XY that the recorded pixel corresponds to")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, f"DICT_{args.dict}"))
    board = aruco.CharucoBoard((args.sx, args.sy), args.square, args.marker, dictionary)
    params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    detector = aruco.CharucoDetector(board, detectorParams=params)

    T_base_board = board_pose_in_base(args, board, detector)
    print("T_base_board:\n", np.round(T_base_board, 4))
    table_z = float(cfg.get("table_z", 0.0))
    print(f"  board origin z = {T_base_board[2,3]:+.4f} m  (table_z = {table_z:+.4f}; "
          f"difference {abs(T_base_board[2,3]-table_z)*1000:.1f} mm)")

    if args.image:
        img = cv2.imread(args.image)
    else:
        if not args.no_park:
            park_arm(cfg)
        idx = cfg["cameras"][args.camera]["index"]
        cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
        for _ in range(10):        # let auto-exposure settle
            ok, img = cap.read()
        cap.release()
        if not ok:
            sys.exit(f"camera {args.camera} (index {idx}) gave no frame — GUI terminal? "
                     "dashboard stopped?")
    print(f"frame: {img.shape[1]}x{img.shape[0]}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    all_obj = board.getChessboardCorners()
    ch_corners, ch_ids, _, _ = detector.detectBoard(gray)
    n = 0 if ch_ids is None else len(ch_ids)
    inner = (args.sx - 1, args.sy - 1)

    if n >= 6:
        print(f"ChArUco corners detected: {n}/{inner[0]*inner[1]}")
        obj = all_obj[np.asarray(ch_ids).reshape(-1)]
        px = np.asarray(ch_corners, dtype=np.float64).reshape(-1, 2)
        candidates = [(obj, px)]
    else:
        # From a metre away the printed markers span ~10 px and their bits are
        # unresolvable, so ArUco decoding fails — but the black/white squares are
        # still crisp, and a plain chessboard detector recovers every corner.
        # Ordering is then ambiguous (the detector may start at either end), so
        # both orderings are fitted and the better residual decides.
        print(f"ChArUco decode failed ({n} corners) — markers are too small at this "
              "distance; falling back to plain chessboard corners")
        ok, corners = cv2.findChessboardCornersSB(
            gray, inner, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if not ok:
            ok, corners = cv2.findChessboardCorners(
                gray, inner, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ok:
                cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
        if not ok:
            sys.exit("no chessboard found — is the board fully visible and unoccluded "
                     "(park the arm), and lit without glare?")
        px = corners.reshape(-1, 2).astype(np.float64)
        print(f"chessboard corners: {len(px)}/{inner[0]*inner[1]}")
        candidates = [(all_obj, px), (all_obj, px[::-1])]

    fits = []
    for obj, px_c in candidates:
        world_c = (T_base_board[:3, :3] @ obj.T).T + T_base_board[:3, 3]
        H_c, mask_c = cv2.findHomography(px_c, world_c[:, :2], cv2.RANSAC, 0.004)
        if H_c is None:
            continue
        proj_c = cv2.perspectiveTransform(px_c.reshape(-1, 1, 2), H_c).reshape(-1, 2)
        err_c = np.linalg.norm(proj_c - world_c[:, :2], axis=1) * 1000
        fits.append((err_c, H_c, mask_c, px_c, world_c))
    if not fits:
        sys.exit("homography fit failed")

    if len(fits) > 1:
        # Residuals CANNOT choose between them: rotating a rectangular corner grid
        # by 180 degrees maps it onto itself, so both orderings fit perfectly while
        # differing by ~40 cm on the table. One measurement from the robot settles
        # it — the arm's own kinematics is the outside information the image lacks.
        if args.probe_px and args.probe_xy:
            probe = ((float(args.probe_px[0]), float(args.probe_px[1])),
                     np.asarray(args.probe_xy, dtype=float))
            print(f"using the recorded probe: px={tuple(args.probe_px)} -> "
                  f"({args.probe_xy[0]:+.3f},{args.probe_xy[1]:+.3f})")
        else:
            probe = probe_fingertip(args, cfg)
        if probe is None:
            sys.exit("could not disambiguate corner ordering: the fingertip probe "
                     "failed. Rerun with the workspace clear, or pass --image plus "
                     "a probe of your own.")
        probe_px, probe_xy = probe
        scored = []
        for f in fits:
            mapped = cv2.perspectiveTransform(
                np.array([[probe_px]], dtype=np.float64), f[1]).reshape(2)
            scored.append((float(np.linalg.norm(mapped - probe_xy)), f, mapped))
        scored.sort(key=lambda s: s[0])
        print(f"corner ordering resolved by the fingertip probe: "
              f"{scored[0][0]*1000:.0f} mm vs {scored[1][0]*1000:.0f} mm for the alternative")
        fits = [scored[0][1]]
    err, H, mask, px, world = fits[0]
    print(f"homography over {len(px)} corners ({int(mask.sum())} inliers)")
    print(f"  reprojection: mean {err.mean():.1f} mm, median {np.median(err):.1f} mm, "
          f"max {err.max():.1f} mm")

    out = Path(args.out or cfg["cameras"][args.camera].get("homography_file")
               or f"calibration/{args.camera}_homography.npz")
    out.parent.mkdir(parents=True, exist_ok=True)
    # Write through Heron's own helper so the field names cannot drift apart.
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from heron.robot.workspace import save_homography  # noqa: PLC0415

    save_homography(out, H, table_z, float(err.mean() / 1000.0))
    print(f"saved -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
