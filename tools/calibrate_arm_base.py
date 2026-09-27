"""Calibrate an arm's t_world_base from fingertip sightings in a world camera.

The inverse of `heron calibrate`: there the arm's world pose is trusted and the
camera is fit; here cam_high's extrinsic is the trusted ruler and the ARM BASE
is fit. The arm walks a grid commanded in ITS OWN frame (driver-level, no world
transform needed or used); at each pose the gripper opens and closes, and the
pinch-point detector (heron.robot.fingertip) finds the only thing that moved —
deterministic, and immune to the idle twin arm that makes VLM pointing
degenerate on a bimanual rig. cam_high's aligned depth deprojects each pinch
pixel to a world point; Kabsch over (p_arm, p_world) pairs is T_world_base.

    .venv/bin/python tools/calibrate_arm_base.py --arm left \\
        --config configs/rig-first.yaml --write

SAFETY: the grid lives in the arm's own comfortable envelope at a safe height;
estop within reach, operator present.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from heron.config import HeronConfig  # noqa: E402
from heron.robot.fingertip import pinch_point_from_pair  # noqa: E402
from heron.robot.workspace import solve_extrinsics  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left")
    ap.add_argument("--config", required=True)
    ap.add_argument("--camera", default="cam_high")
    # Grid in the ARM's own frame: forward of the base, straddling its centre
    # line, high enough to be safe under any base-pose uncertainty.
    ap.add_argument("--x", nargs=2, type=float, default=(0.22, 0.34))
    ap.add_argument("--y", nargs=2, type=float, default=(-0.10, 0.10))
    ap.add_argument("--z", nargs=2, type=float, default=(0.08, 0.15))
    ap.add_argument("--grid", nargs=2, type=int, default=(3, 3))
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()

    cfg = HeronConfig.load(args.config)
    if args.arm not in cfg.arms:
        print(f"arm {args.arm!r} not in config ({list(cfg.arms)})")
        return 1
    from heron.robot.trossen import TrossenStationary
    # The constructor honours cfg.arms, which may exclude this arm on a
    # single-arm rig config; force it in for the calibration session.
    robot = TrossenStationary(cfg)
    if args.arm not in robot._drivers:
        print(f"no driver for {args.arm} — is it powered and in the config?")
        robot.shutdown()
        return 1
    drv = robot._drivers[args.arm]
    rvec = np.asarray(cfg.arms[args.arm].approach_rvec, dtype=float)

    xs = np.linspace(*args.x, args.grid[0])
    ys = np.linspace(*args.y, args.grid[1])
    zs = np.linspace(*args.z, 2)
    pairs_arm, pairs_world = [], []
    try:
        for zi, z in enumerate(zs):
            for x in xs:
                for y in (ys if zi % 2 == 0 else ys[::-1]):
                    try:
                        q_now = np.asarray(drv.get_all_positions(),
                                           dtype=float)[:robot._kin.dof]
                        q = robot._kin.ik(np.array([x, y, z]), rvec,
                                          q_seed=q_now)
                        if q is None:
                            print(f"pose ({x:.2f},{y:.2f},{z:.2f}): no IK")
                            continue
                        drv.set_arm_positions(list(q), 2.5, True)
                    except Exception as e:
                        print(f"pose ({x:.2f},{y:.2f},{z:.2f}) refused: "
                              f"{type(e).__name__}: {e}"[:120])
                        continue
                    time.sleep(0.4)
                    # Fingertip = the only thing that moves between the pair.
                    robot.set_gripper(args.arm, 0.06)
                    time.sleep(0.6)
                    f_open = robot.capture(args.camera)
                    robot.set_gripper(args.arm, 0.0)
                    time.sleep(0.6)
                    f_closed = robot.capture(args.camera)
                    hit = pinch_point_from_pair(f_open.rgb, f_closed.rgb)
                    if hit is None:
                        print(f"pose ({x:.2f},{y:.2f},{z:.2f}): no pinch point")
                        continue
                    u, v, conf = hit
                    # Depth AT the pinch pixel is the void between the
                    # fingers — the table 10 cm behind them (rms 282 mm on
                    # the first attempt). The fingers themselves are the
                    # changed pixels: per pixel take the NEARER of the two
                    # frames (the finger is in one of them), then the median.
                    import cv2 as _cv2
                    a = _cv2.GaussianBlur(f_open.rgb, (5, 5), 0).astype(np.int16)
                    b = _cv2.GaussianBlur(f_closed.rgb, (5, 5), 0).astype(np.int16)
                    m = (np.abs(a - b).max(axis=2) > 25)
                    d = np.fmin(np.where(np.isfinite(f_open.depth) & (f_open.depth > 0.1),
                                         f_open.depth, np.inf),
                                np.where(np.isfinite(f_closed.depth) & (f_closed.depth > 0.1),
                                         f_closed.depth, np.inf))
                    # The whole finger BODY moves, and its upper parts are
                    # nearer the overhead camera — the median landed 8 cm
                    # above the tips (world z 0.244 for an arm-frame 0.15).
                    # Only pixels around the pinch centroid are the tips.
                    hh, ww = m.shape
                    vv, uu = np.mgrid[0:hh, 0:ww]
                    near = (uu - u) ** 2 + (vv - v) ** 2 <= 15 ** 2
                    dm = d[m & near & np.isfinite(d)]
                    if dm.size < 30:
                        print(f"pose ({x:.2f},{y:.2f},{z:.2f}): dead depth")
                        continue
                    zc = float(np.median(dm))
                    k = np.asarray(f_closed.intrinsics, dtype=float)
                    pc = np.array([(u - k[0, 2]) * zc / k[0, 0],
                                   (v - k[1, 2]) * zc / k[1, 1], zc, 1.0])
                    # This rig carries no measured extrinsics file since
                    # 2026-08-10: the camera pose derives from the
                    # board-stitched homography (the same ruler everything
                    # else uses), so derive it here too when absent.
                    t_bc = getattr(f_closed, "t_base_cam", None)
                    if t_bc is None:
                        from heron.robot.workspace import (  # noqa: PLC0415
                            pose_from_plane_homography)
                        t_bc = pose_from_plane_homography(
                            np.asarray(f_closed.h_pixel_world, float), k,
                            float(getattr(f_closed, "plane_z", 0.0) or 0.0))
                    pw = (np.asarray(t_bc, dtype=float) @ pc)[:3]
                    fk = np.asarray(drv.get_cartesian_positions(), dtype=float)[:3]
                    pairs_arm.append(fk)
                    pairs_world.append(pw)
                    print(f"pose ({x:.2f},{y:.2f},{z:.2f}) -> world "
                          f"{np.round(pw, 3)} (conf {conf:.2f}, "
                          f"{len(pairs_arm)} pairs)")
    finally:
        try:
            robot.set_gripper(args.arm, 0.06)
        except Exception:
            pass
        robot.shutdown()

    if len(pairs_arm) < 6:
        print(f"only {len(pairs_arm)} pairs — need 6+; check the camera can "
              "see this arm's grid")
        return 1
    if len({round(float(p[2]), 2) for p in pairs_arm}) < 2:
        print("all pairs are COPLANAR (one z layer reached) — the fit is "
              "degenerate; widen --z to two reachable heights")
        return 1
    pa, pw = np.asarray(pairs_arm), np.asarray(pairs_world)
    (ROOT / "calib").mkdir(exist_ok=True)
    np.savez(ROOT / "calib" / f"{args.arm}_base_raw.npz", p_arm=pa, p_world=pw)
    t, rms = solve_extrinsics(pa, pw)
    print(f"all-pairs rms {rms * 1000:.1f} mm over {len(pa)}")
    # A pose where the moving SHADOW outvoted the fingers gives a wildly
    # wrong world point; one such pair poisons a least-squares fit. Trim:
    # fit, drop the worst 30 percent by residual, refit on the inliers.
    res = np.linalg.norm((t[:3, :3] @ pa.T).T + t[:3, 3] - pw, axis=1)
    keep = np.argsort(res)[: max(8, int(len(pa) * 0.7))]
    if len({round(float(z), 2) for z in pa[keep][:, 2]}) < 2:
        print("inliers went coplanar — not trusting the trim")
    else:
        t2, rms2 = solve_extrinsics(pa[keep], pw[keep])
        print(f"trimmed rms {rms2 * 1000:.1f} mm over {len(keep)} inliers "
              f"(dropped: {np.round(np.sort(res)[len(keep):] * 1000, 0)})")
        if rms2 < rms:
            t, rms = t2, rms2
    print(f"t_world_base[{args.arm}] rms {rms * 1000:.1f} mm:")
    print(np.round(t, 4))
    if rms > 0.012:
        print("rms over 12 mm — NOT writing")
        return 1
    if args.write:
        out = ROOT / "calib" / f"{args.arm}_t_world_base.json"
        out.parent.mkdir(exist_ok=True)
        import json
        out.write_text(json.dumps(t.tolist(), indent=1))
        print(f"wrote {out} — paste into the rig config under "
              f"arms.{args.arm}.t_world_base")
    else:
        print("dry run (pass --write to save)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
