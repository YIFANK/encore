"""Refit a fixed camera's extrinsic from the board the hand-eye session measured.

cam_high's t_world_cam was fit by presenting the FINGERTIP on a grid and finding
it in the image — a 15 mm, self-occluding, shadow-casting target — and the file
it wrote says 14.8 mm rms. cam_low, cross-calibrated off a ChArUco board the
same week, says 5.5 mm. On 2026-08-06 the two disagreed by 23 mm about a block
neither had touched, three grasps closed on air in the right-front corner where
both are extrapolating, and the batch's episode time went up 1.8x.

The board is the better ruler, and one board is already tied to the arm:

    T_world_board = T_base_gripper . T_ee_cam . T_cam_board    (the hand-eye session)
    t_world_cam   = T_world_board  . inv(T_cam_board_seen_now)

One frame, dozens of sub-pixel corners, no arm motion at all.

THE SCORE IS NOT THE FIT'S OWN RESIDUAL. Every corner is deprojected through the
camera's MEASURED DEPTH and compared against where the board says it is. Depth
never enters the fit, so agreement is evidence rather than arithmetic — the same
reason cross_calibrate.py scores that way. The npz is written only under --write
and only if the new extrinsic beats the one on disk by that measure.

Extra boards are welcome and are not decoration. Identical prints share marker
ids, so the frame is split into clusters by marker proximity first; the anchor
is whichever cluster lands where the hand-eye session left it. The rest are
scored too, and a board propped up at an angle is the most informative of all:
its corners sit at many heights, where a fit that is wrong about the camera's
tilt or distance cannot hide.

    .venv/bin/python tools/calibrate_cam_from_board.py --config configs/rig-first.yaml
    .venv/bin/python tools/calibrate_cam_from_board.py --config configs/rig-first.yaml --write
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import sys
import urllib.request
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from calibrate_from_board import board_pose_in_base  # noqa: E402
from calibrate_multiboard import cluster_markers, decode_board_region  # noqa: E402


def sidecar_rgbd(url: str, role: str):
    """RGB, depth in metres, and K — one consistent capture from the sidecar."""
    from PIL import Image  # noqa: PLC0415

    with urllib.request.urlopen(f"{url}/frame/{role}", timeout=20) as r:
        payload = json.loads(r.read())
    rgb = np.asarray(Image.open(io.BytesIO(base64.b64decode(payload["rgb_png_b64"]))),
                     dtype=np.uint8)
    raw = np.asarray(Image.open(io.BytesIO(base64.b64decode(payload["depth_png16_b64"]))),
                     dtype=np.float32)
    with urllib.request.urlopen(f"{url}/info/{role}", timeout=10) as r:
        info = json.loads(r.read())
    K = np.array([[info["fx"], 0, info["ppx"]],
                  [0, info["fy"], info["ppy"]], [0, 0, 1.0]], dtype=np.float64)
    return rgb, raw * float(info["depth_scale"]), K


def deproject(px: np.ndarray, depth: np.ndarray, K: np.ndarray, patch: int = 2):
    """Camera-frame 3D for each pixel, or NaN where depth has no opinion.

    The median of a small patch, not the single pixel: a chessboard corner is
    exactly where the projector's pattern is ambiguous, black square against
    white, and one pixel there is the noisiest sample on the board.
    """
    h, w = depth.shape
    out = np.full((len(px), 3), np.nan)
    for i, (u, v) in enumerate(px):
        u_i, v_i = int(round(u)), int(round(v))
        if not (patch <= u_i < w - patch and patch <= v_i < h - patch):
            continue
        win = depth[v_i - patch:v_i + patch + 1, u_i - patch:u_i + patch + 1]
        good = win[np.isfinite(win) & (win > 0.05)]
        if good.size < 4:
            continue
        z = float(np.median(good))
        out[i] = [(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z]
    return out


def to_world(t_world_cam: np.ndarray, pts_cam: np.ndarray) -> np.ndarray:
    ok = np.isfinite(pts_cam[:, 0])
    out = np.full_like(pts_cam, np.nan)
    if ok.any():
        out[ok] = (t_world_cam[:3, :3] @ pts_cam[ok].T).T + t_world_cam[:3, 3]
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/rig-first.yaml")
    ap.add_argument("--camera", default="cam_high")
    ap.add_argument("--handeye", default="calibration/right_wrist_handeye.npz")
    ap.add_argument("--raw-dir", default="calibration/handeye_raw")
    ap.add_argument("--sx", type=int, default=8)
    ap.add_argument("--sy", type=int, default=5)
    ap.add_argument("--square", type=float, default=0.034)
    ap.add_argument("--marker", type=float, default=0.025)
    ap.add_argument("--dict", default="5X5_50")
    ap.add_argument("--image", help="score this saved frame instead of capturing")
    ap.add_argument("--write", action="store_true", help="write the npz if it scores better")
    ap.add_argument("--max-rms", type=float, default=0.010)
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    cam = cfg["cameras"][args.camera]
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f"DICT_{args.dict}"))
    board = cv2.aruco.CharucoBoard((args.sx, args.sy), args.square, args.marker, dictionary)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.CharucoDetector(board, detectorParams=params)

    # Where the arm says the anchor board is. This is the only tie between the
    # camera and the world, and it is as old as the hand-eye session.
    T_base_board = board_pose_in_base(
        SimpleNamespace(handeye=args.handeye, raw_dir=args.raw_dir), board, detector)
    print(f"anchor board origin (from hand-eye): "
          f"{np.round(T_base_board[:3, 3], 4).tolist()}")

    rgb, depth, K = sidecar_rgbd(cam["depth_server"], args.camera)
    print(f"frame {rgb.shape[1]}x{rgb.shape[0]}, depth coverage "
          f"{100 * np.mean(depth > 0.05):.0f}%")

    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    # At a metre the printed markers span ~20 px and their bits are unresolvable
    # at native scale — 20 of 48 markers on a first pass, and no board decoded.
    # Detect on a 3x upscale, exactly as calibrate_multiboard.py learned to.
    up3 = cv2.resize(gray, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(up3)
    corners = [c / 3.0 for c in (corners or [])]
    if ids is None or len(ids) < 4:
        print("no ArUco markers found — is a board visible and unoccluded?")
        return 1
    # SEPARATION HAS TO BE MEASURED IN MARKERS, NOT PIXELS. The fixed 75 px gap
    # merged a board propped on a box with the one lying below it — and two
    # identical prints in one cluster share every marker id, so the ChArUco
    # decoder sees duplicates and returns nothing. A marker spans ~16 px at this
    # distance; boards that are two marker-widths apart are two boards.
    side = float(np.median([
        np.linalg.norm(c.reshape(-1, 2)[0] - c.reshape(-1, 2)[1]) for c in corners]))
    gap = max(30.0, 2.5 * side)
    clusters = cluster_markers(corners, ids, gap_px=gap)
    print(f"marker side {side:.0f} px -> clustering at {gap:.0f} px")
    print(f"{len(ids)} markers in {len(clusters)} board cluster(s)")

    old = np.load(cam.get("extrinsics_file", f"calibration/{args.camera}.npz"))
    t_old = old["t_world_cam"].astype(np.float64)
    print(f"current extrinsic on disk: rms {float(old['rms']) * 1000:.1f} mm")

    boards = []
    for n, members in enumerate(clusters):
        ch_px = ch_ids = None
        for scale in (3.0, 4.5, 6.0):
            ch_px, ch_ids = decode_board_region(gray, board, detector, members,
                                                corners, scale=scale)
            if ch_ids is not None:
                break
        if ch_ids is None:
            print(f"  cluster {n}: {len(members)} markers, corners would not decode")
            continue
        obj = board.getChessboardCorners()[ch_ids]
        ok, rvec, tvec = cv2.solvePnP(obj, ch_px.reshape(-1, 1, 2), K, None)
        if not ok:
            continue
        T_cam_board = np.eye(4)
        T_cam_board[:3, :3] = cv2.Rodrigues(rvec)[0]
        T_cam_board[:3, 3] = tvec.reshape(3)
        origin_old = (t_old @ np.append(T_cam_board[:3, 3], 1.0))[:3]
        d = float(np.linalg.norm(origin_old[:2] - T_base_board[:2, 3]))
        tilt = float(np.degrees(np.arccos(
            abs(float((t_old[:3, :3] @ T_cam_board[:3, 2])[2])))))
        print(f"  cluster {n}: {len(ch_px)} corners, origin "
              f"{np.round(origin_old, 3).tolist()}, {d * 1000:5.0f} mm from the "
              f"anchor, tilted {90 - tilt:.0f} deg off the table")
        boards.append((n, obj, ch_px.reshape(-1, 2), T_cam_board, d))

    if not boards:
        print("nothing decoded")
        return 1
    anchor = min(boards, key=lambda b: b[4])
    print(f"anchor = cluster {anchor[0]} ({anchor[4] * 1000:.0f} mm from where the "
          f"hand-eye session left it)")
    if anchor[4] > 0.06:
        print("  WARNING: that is far. If the board has been MOVED since the hand-eye "
              "session, this fit inherits the move — put it back, or redo hand-eye.")

    t_new = T_base_board @ np.linalg.inv(anchor[3])
    print("\ncandidate t_world_cam:\n", np.round(t_new, 4))
    print(f"  camera moved {np.linalg.norm(t_new[:3, 3] - t_old[:3, 3]) * 1000:.0f} mm "
          f"from the old fit")

    # The verdict, on depth the fit never saw.
    def score(t_wc: np.ndarray) -> tuple[float, int]:
        pts = deproject(anchor[2], depth, K)
        world = to_world(t_wc, pts)
        truth = (T_base_board[:3, :3] @ anchor[1].T).T + T_base_board[:3, 3]
        ok = np.isfinite(world[:, 0])
        if ok.sum() < 6:
            return float("nan"), int(ok.sum())
        e = np.linalg.norm(world[ok] - truth[ok], axis=1)
        return float(np.sqrt(np.mean(e ** 2))), int(ok.sum())

    rms_old, n_old = score(t_old)
    rms_new, n_new = score(t_new)
    print(f"\ndepth-vs-board rms over {n_new} corners:")
    print(f"  old extrinsic  {rms_old * 1000:6.1f} mm")
    print(f"  new extrinsic  {rms_new * 1000:6.1f} mm")

    # Every other board is a probe of the same fit somewhere else on the table.
    # A tilted one is the sharpest: getting its corners right at several heights
    # is exactly what a wrong camera pose cannot fake.
    for n, obj, px, _T, _d in boards:
        world = to_world(t_new, deproject(px, depth, K))
        ok = np.isfinite(world[:, 0])
        if ok.sum() < 6:
            continue
        # Fit the plane those corners actually lie on and report the scatter
        # about it: the board IS flat, so scatter is the camera's error there.
        P = world[ok]
        c = P.mean(0)
        normal = np.linalg.svd(P - c)[2][-1]
        resid = np.abs((P - c) @ normal)
        print(f"  board {n}: {ok.sum():3d} corners, flatness rms "
              f"{np.sqrt(np.mean(resid ** 2)) * 1000:4.1f} mm, "
              f"z range {P[:, 2].min():+.3f}..{P[:, 2].max():+.3f}")

    if not args.write:
        print("\n(dry run — pass --write to save)")
        return 0
    if not np.isfinite(rms_new) or rms_new > args.max_rms:
        print(f"\nrms over {args.max_rms * 1000:.0f} mm — NOT writing")
        return 1
    if rms_new >= rms_old:
        print("\nno better than what is already on disk — NOT writing")
        return 1
    from heron.robot.workspace import save_extrinsics  # noqa: PLC0415

    out = Path(cam.get("extrinsics_file", f"calibration/{args.camera}.npz"))
    out.parent.mkdir(parents=True, exist_ok=True)
    save_extrinsics(out, t_new, rms_new)
    print(f"\nsaved -> {out}")
    print("cam_low is cross-calibrated FROM this camera — rerun "
          "tools/cross_calibrate.py --write, then tools/calibrate_multiboard.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
