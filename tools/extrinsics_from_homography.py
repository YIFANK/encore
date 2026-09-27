"""Solve the overhead camera's full extrinsics from depth plus the homography we
already have — without moving the arm.

The usual route to `t_world_cam` is to touch known points with a fingertip and
match them to what the camera sees. That needs the arm, an operator watching it,
and a clear table. But this rig already carries a calibrated pixel -> table-XY
homography (fitted to 0.85 mm), and the sidecar now supplies depth. Together
those give 3D-3D correspondences for free:

    a table pixel (u, v)
      -> world  (x, y) from the homography, z = plane_z by definition
      -> camera (X, Y, Z) from its own depth and the intrinsics

Kabsch on those pairs is the extrinsic. Every point lies on one plane, which is
fine here — coplanarity ruins pose-from-2D, not 3D-3D alignment, as long as the
points are not collinear, and a table's worth of pixels spans two dimensions
comfortably.

The homography lives in the pixel space it was fitted in (1280x720, the UVC
stream). The colour stream must therefore run at the same resolution, or the
correspondences are between two different fields of view: 640x480 on a D405 is
a horizontal crop, not a scaling. This refuses to run otherwise.

    python tools/extrinsics_from_homography.py --write
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import sys
import urllib.request
from pathlib import Path

import numpy as np


def fetch(url: str, path: str):
    with urllib.request.urlopen(f"{url}{path}", timeout=30) as r:
        return json.loads(r.read())


def rgbd(url: str, role: str):
    from PIL import Image  # noqa: PLC0415

    info = fetch(url, f"/info/{role}")
    frame = fetch(url, f"/frame/{role}")
    rgb = np.asarray(Image.open(io.BytesIO(
        base64.b64decode(frame["rgb_png_b64"]))).convert("RGB"))
    raw = np.asarray(Image.open(io.BytesIO(base64.b64decode(frame["depth_png16_b64"]))))
    return rgb, raw.astype(np.float32) * float(info["depth_scale"]), info


def fit_plane(pts: np.ndarray, thresh: float, iters: int, seed: int = 0,
              score_on: int = 20000):
    """RANSAC a plane, then refit on its inliers by SVD.

    The refit matters: three randomly chosen points define a plane badly, and
    the normal is what everything downstream leans on.

    Scoring is done on a fixed random subset. A 1280x720 frame has ~900k points
    and scoring all of them for every hypothesis is 2.7 billion operations —
    which is how the first version of this appeared to hang. A subset of 20k
    ranks hypotheses just as well; the final SVD refit still uses every inlier.
    """
    rng = np.random.default_rng(seed)
    sub = pts[rng.choice(len(pts), min(score_on, len(pts)), replace=False)]
    best_n, best = 0, None
    for _ in range(iters):
        tri = sub[rng.choice(len(sub), 3, replace=False)]
        n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            continue
        n = n / ln
        inl = int((np.abs((sub - tri[0]) @ n) < thresh).sum())
        if inl > best_n:
            best_n, best = inl, (n, tri[0])
    if best is None:
        raise RuntimeError("no plane found")
    n, p0 = best
    keep = np.abs((pts - p0) @ n) < thresh
    c = pts[keep].mean(axis=0)
    # full_matrices=False, or numpy builds an N x N left factor — 900k points
    # asks for a 6 TB array and the process is killed with no message at all.
    n = np.linalg.svd(pts[keep] - c, full_matrices=False)[2][-1]
    keep = np.abs((pts - c) @ n) < thresh
    return n, c, keep


def kabsch(src: np.ndarray, dst: np.ndarray):
    """Rigid transform taking src onto dst. Returns (4x4, per-point residuals)."""
    sc, dc = src.mean(axis=0), dst.mean(axis=0)
    u, _, vt = np.linalg.svd((src - sc).T @ (dst - dc))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    t = dc - r @ sc
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, t
    return m, np.linalg.norm((src @ r.T + t) - dst, axis=1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="http://127.0.0.1:8766")
    ap.add_argument("--role", default="cam_high")
    ap.add_argument("--homography", default="calibration/cam_high_homography.npz")
    ap.add_argument("--out", default="calibration/cam_high.npz")
    ap.add_argument("--write", action="store_true", help="save the result")
    ap.add_argument("--plane-thresh", type=float, default=0.012,
                    help="plane inlier band; the measured noise floor is ~5.7 mm RMS")
    ap.add_argument("--iters", type=int, default=3000)
    ap.add_argument("--max-points", type=int, default=40000)
    ap.add_argument("--frames", type=int, default=5,
                    help="average this many depth frames; the noise is uncorrelated "
                         "in time and averaging is free")
    args = ap.parse_args()

    hom = np.load(args.homography, allow_pickle=True)
    h_pixel_world = hom["h_pixel_world"]
    plane_z = float(np.ravel(hom["plane_z"])[0])
    print(f"homography {args.homography}: plane_z={plane_z:.4f} m, "
          f"fit rms={float(np.ravel(hom['rms'])[0]) * 1000:.2f} mm")

    acc, counts, info = None, None, None
    for _ in range(args.frames):
        _, frame_z, info = rgbd(args.url, args.role)
        good = frame_z > 0
        acc = np.where(good, frame_z, 0.0) if acc is None else acc + np.where(good, frame_z, 0.0)
        counts = good.astype(np.float32) if counts is None else counts + good
    z = acc / np.maximum(counts, 1.0)
    z[counts == 0] = 0.0
    h, w = z.shape
    print(f"colour/depth {w}x{h}, averaged over {args.frames} frames")

    if (w, h) != (1280, 720):
        print(f"\nREFUSING: the homography was fitted on 1280x720 and this stream is "
              f"{w}x{h}.\nOn a D405 those are different fields of view, not a scale "
              f"factor, so the\ncorrespondences would be wrong in a way the residual "
              f"would not reveal.\nRestart the sidecar with --width 1280 --height 720.")
        return 2

    fx, fy = info["fx"], info["fy"]
    cx, cy = info["ppx"], info["ppy"]
    vs, us = np.mgrid[0:h, 0:w]
    ok = (z > 0.4) & (z < 2.5)
    print(f"{ok.sum()} pixels with usable depth ({100 * ok.mean():.0f}%)")
    cam = np.stack([(us[ok] - cx) * z[ok] / fx, (vs[ok] - cy) * z[ok] / fy, z[ok]], axis=1)
    uv = np.stack([us[ok], vs[ok]], axis=1).astype(float)

    n, c, keep = fit_plane(cam, args.plane_thresh, args.iters)
    print(f"table plane: {keep.sum()} inliers of {len(cam)} ({100 * keep.mean():.0f}%), "
          f"camera {abs(float(-c @ n)):.3f} m away, "
          f"tilt {np.degrees(np.arccos(min(1.0, abs(float(n[2]))))):.1f} deg")
    cam, uv = cam[keep], uv[keep]
    if len(cam) > args.max_points:
        idx = np.random.default_rng(0).choice(len(cam), args.max_points, replace=False)
        cam, uv = cam[idx], uv[idx]

    ph = np.hstack([uv, np.ones((len(uv), 1))]) @ h_pixel_world.T
    world = np.stack([ph[:, 0] / ph[:, 2], ph[:, 1] / ph[:, 2],
                      np.full(len(ph), plane_z)], axis=1)

    t_world_cam, resid = kabsch(cam, world)
    print(f"\nKabsch over {len(cam)} table points")
    print(f"   residual  rms {1000 * np.sqrt((resid ** 2).mean()):6.2f} mm   "
          f"median {1000 * np.median(resid):6.2f} mm   "
          f"95% {1000 * np.percentile(resid, 95):6.2f} mm")
    print("   camera position in the arm's base frame: "
          f"{np.round(t_world_cam[:3, 3], 4).tolist()} m")

    # The residual only says the fit is self-consistent. This says the geometry
    # is right: table pixels must land back on the table.
    back = (cam @ t_world_cam[:3, :3].T + t_world_cam[:3, 3])[:, 2]
    print(f"   table points reproject to z = {back.mean():.4f} +/- "
          f"{back.std() * 1000:.2f} mm  (expected {plane_z:.4f})")

    if args.write:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        np.savez(args.out, t_world_cam=t_world_cam,
                 rms=np.array([float(np.sqrt((resid ** 2).mean()))]),
                 source=np.array(["homography+depth, no arm motion"]))
        print(f"\nwrote {args.out}")
    else:
        print("\n(dry run — pass --write to save)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
