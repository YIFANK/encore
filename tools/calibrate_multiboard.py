"""Refit a camera's table homography from SEVERAL identical ChArUco boards.

One board covers one patch of the table, and everywhere else the homography
is extrapolation — measured on the rig, 2-3 cm of error in the right-front
region, two grasps closed on air there in one episode. Scatter several
copies of the SAME board across the table and each one anchors its own
patch: per board, decode corners (3x upscale — at 1.04 m the printed
markers span ~20 px), solve its pose with the camera intrinsics, map the
corners into the world through the calibrated t_world_cam, and fit ONE
homography over the union of all px<->world pairs.

Identical prints mean identical marker IDs, so the frame is first split
into board clusters by marker proximity and each cluster is decoded alone
with the others masked out.

The write guard compares the new fit against the CURRENT homography on the
union set; it must be strictly better to be written.

    .venv/bin/python tools/calibrate_multiboard.py --config configs/rig-first.yaml
"""
from __future__ import annotations

import argparse
import io
import json
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent


def sidecar_frame(url: str, role: str):
    """RGB image + intrinsics from the depth sidecar (JSON + base64 PNG)."""
    import base64
    from PIL import Image
    with urllib.request.urlopen(f"{url}/frame/{role}", timeout=15) as r:
        payload = json.loads(r.read())
    rgb = np.asarray(Image.open(io.BytesIO(base64.b64decode(payload["rgb_png_b64"]))),
                     dtype=np.uint8)
    with urllib.request.urlopen(f"{url}/info/{role}", timeout=10) as r:
        info = json.loads(r.read())
    K = np.array([[info["fx"], 0, info["ppx"]],
                  [0, info["fy"], info["ppy"]], [0, 0, 1.0]], dtype=np.float64)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), K


def cluster_markers(corners, ids, gap_px: float = 75.0):
    """Group detected ArUco markers into boards by spatial proximity."""
    if ids is None or len(ids) == 0:
        return []
    centres = np.array([c.reshape(-1, 2).mean(axis=0) for c in corners])
    unassigned = list(range(len(centres)))
    clusters = []
    while unassigned:
        seed = unassigned.pop(0)
        members = [seed]
        changed = True
        while changed:
            changed = False
            for i in list(unassigned):
                if min(np.linalg.norm(centres[i] - centres[m]) for m in members) < gap_px:
                    members.append(i)
                    unassigned.remove(i)
                    changed = True
        clusters.append(members)
    return clusters


def decode_board_region(gray, board, detector, members, corners, scale=3.0):
    """ChArUco corners for ONE board, others masked, image upscaled."""
    mask = np.zeros_like(gray)
    pts = np.concatenate([corners[m].reshape(-1, 2) for m in members]).astype(np.int32)
    x0, y0 = np.maximum(pts.min(axis=0) - 40, 0)
    x1, y1 = np.minimum(pts.max(axis=0) + 40, np.array(gray.shape[::-1]) - 1)
    mask[y0:y1, x0:x1] = 255
    isolated = np.where(mask > 0, gray, 255).astype(np.uint8)
    up = cv2.resize(isolated, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    ch_corners, ch_ids, _, _ = detector.detectBoard(up)
    if ch_ids is None or len(ch_ids) < 6:
        return None, None
    return np.asarray(ch_corners, np.float64).reshape(-1, 2) / scale, np.asarray(ch_ids).reshape(-1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/rig-first.yaml")
    ap.add_argument("--camera", default="cam_high")
    ap.add_argument("--sx", type=int, default=8)
    ap.add_argument("--sy", type=int, default=5)
    ap.add_argument("--square", type=float, default=0.034)
    ap.add_argument("--marker", type=float, default=0.025)
    ap.add_argument("--dict", default="5X5_50")
    ap.add_argument("--extrinsics", default="calibration/cam_high.npz")
    ap.add_argument("--out", default=None)
    ap.add_argument("--prior", help="previous trusted homography npz for anchor pairs")
    ap.add_argument("--prior-px-box", type=float, nargs=4, default=(420, 300, 760, 520),
                    metavar=("X0", "Y0", "X1", "Y1"),
                    help="pixel box the prior fit is trusted in (its board region)")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    cam = cfg["cameras"][args.camera]
    img, K = sidecar_frame(cam["depth_server"], args.camera)
    print(f"frame {img.shape[1]}x{img.shape[0]}, fx={K[0,0]:.1f}")
    t_world_cam = np.load(args.extrinsics)["t_world_cam"].astype(np.float64)
    table_z = float(cfg.get("table_z", 0.0))

    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, f"DICT_{args.dict}"))
    board = aruco.CharucoBoard((args.sx, args.sy), args.square, args.marker, dictionary)
    params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    detector = aruco.CharucoDetector(board, detectorParams=params)
    mdetector = aruco.ArucoDetector(dictionary, params)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    up3 = cv2.resize(gray, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
    mcorners, mids, _ = mdetector.detectMarkers(up3)
    mcorners = [c / 3.0 for c in (mcorners or [])]
    clusters = cluster_markers(mcorners, mids)
    print(f"{0 if mids is None else len(mids)} markers in {len(clusters)} board cluster(s)")

    all_obj = board.getChessboardCorners()
    pairs_px, pairs_world = [], []
    for ci, members in enumerate(clusters):
        px = ids = None
        for scale in (3.0, 4.5, 6.0):
            px, ids = decode_board_region(gray, board, detector, members, mcorners,
                                          scale=scale)
            if px is not None:
                break
        if px is None:
            print(f"  board {ci}: decode failed at all scales, skipped")
            continue
        obj = all_obj[ids]
        ok, rvec, tvec = cv2.solvePnP(obj, px, K, None)
        if not ok:
            print(f"  board {ci}: PnP failed, skipped")
            continue
        R, _ = cv2.Rodrigues(rvec)
        T_cam_board = np.eye(4)
        T_cam_board[:3, :3], T_cam_board[:3, 3] = R, tvec.ravel()
        T_world_board = t_world_cam @ T_cam_board
        world = (T_world_board[:3, :3] @ obj.T + T_world_board[:3, 3:4]).T
        zerr = abs(float(world[:, 2].mean()) - table_z) * 1000
        print(f"  board {ci}: {len(ids)} corners, plane z err {zerr:.1f} mm "
              f"@ centre {np.round(world[:, :2].mean(axis=0), 3).tolist()}")
        if zerr > 25:
            print(f"  board {ci}: plane {zerr:.0f} mm off the table — rejected")
            continue
        pairs_px.append(px)
        pairs_world.append(world[:, :2])
    if not pairs_px:
        sys.exit("no boards decoded; nothing to fit")

    if args.prior:
        # Anchor pairs synthesised from a TRUSTED previous fit, over the region
        # it was fitted in (the probe-anchored centre board). The new boards
        # extend coverage; the prior keeps the old sweet spot from drifting.
        d = np.load(args.prior)
        H_prior = d["h_pixel_world"]
        cx0, cy0, cx1, cy1 = args.prior_px_box
        gx, gy = np.meshgrid(np.linspace(cx0, cx1, 6), np.linspace(cy0, cy1, 5))
        grid = np.stack([gx.ravel(), gy.ravel()], axis=1)
        prior_world = cv2.perspectiveTransform(
            grid.reshape(-1, 1, 2), H_prior).reshape(-1, 2)
        pairs_px.append(grid)
        pairs_world.append(prior_world)
        print(f"  prior: {len(grid)} synthetic anchors from {args.prior} over px box "
              f"({cx0:.0f},{cy0:.0f})-({cx1:.0f},{cy1:.0f})")

    px = np.concatenate(pairs_px)
    world = np.concatenate(pairs_world)
    H, inl = cv2.findHomography(px, world, cv2.RANSAC, 0.005)

    def region_medians(Hm):
        meds = []
        for ppx, pw in zip(pairs_px, pairs_world):
            pr = cv2.perspectiveTransform(np.asarray(ppx).reshape(-1, 1, 2), Hm).reshape(-1, 2)
            meds.append(float(np.median(np.linalg.norm(pr - pw, axis=1) * 1000)))
        return meds

    err = np.linalg.norm(
        cv2.perspectiveTransform(px.reshape(-1, 1, 2), H).reshape(-1, 2) - world, axis=1) * 1000
    new_meds = region_medians(H)
    print(f"NEW fit: {len(px)} corners, global median {np.median(err):.2f} mm, "
          f"per-region medians {['%.1f' % m for m in new_meds]}")

    out = Path(args.out or cam.get("homography_file", f"calibration/{args.camera}_homography.npz"))
    if out.exists():
        H_old = np.load(out)["h_pixel_world"]
        old_meds = region_medians(H_old)
        print(f"CURRENT fit per-region medians: {['%.1f' % m for m in old_meds]}")
        # Minimax: a fit that tames the worst REGION wins, even if the global
        # median ticks up — a 1.4 mm median with a 25 mm blind corner loses to
        # a 1.8 mm median that covers the whole table.
        if max(new_meds) >= max(old_meds):
            sys.exit(f"worst region {max(new_meds):.1f} mm >= current {max(old_meds):.1f} mm "
                     "— NOT writing")
    if max(new_meds) > 8.0:
        sys.exit(f"worst region {max(new_meds):.1f} mm > 8 mm guard — NOT writing")
    # Same keys the readers load (depthclient/cameras): h_pixel_world, plane_z, rms.
    np.savez(out, h_pixel_world=H, plane_z=np.float64(table_z),
             rms=np.float64(float(np.sqrt((err / 1000) ** 2).mean())))
    print(f"saved -> {out}  ({len(pairs_px)} boards, {len(px)} corners, "
          f"median {np.median(err):.2f} mm)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
