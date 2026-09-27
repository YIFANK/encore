"""One-shot cam_high recalibration from the ChArUco board on the table.

    ./calibrate            # park the arm clear, board fit at 3x, extrinsics, done

Needs: the board flat on the table at its hand-eye-era spot, the depth sidecar
serving, nothing else. Total ~30 s. The 3x upscale exists because at 1.04 m the
aruco markers are ~20 px in a 720p frame and the decoder wants ~60; detection
happens at 3x and the corners are divided back, so the homography lives in
native pixels. Refuses to overwrite unless the fit is sub-5 mm — a calibration
worse than the one on disk must never win by default.
"""
from __future__ import annotations

import argparse, base64, json, sys, time, types, urllib.request
from pathlib import Path

import cv2
import numpy as np
from cv2 import aruco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

PARK = [-1.4, 0.9, 1.9, 0.0, 0.0, 0.0]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "configs/rig-first.yaml"))
    ap.add_argument("--depth-url", default="http://127.0.0.1:8766")
    ap.add_argument("--scale", type=int, default=3)
    ap.add_argument("--max-mm", type=float, default=5.0)
    ap.add_argument("--no-park", action="store_true")
    ap.add_argument("--allow-flip", action="store_true",
                    help="accept a homography whose axes reverse the trusted "
                         "one on disk (only correct after physically remounting "
                         "the camera)")
    a = ap.parse_args()

    if not a.no_park:
        import trossen_arm as ta
        d = ta.TrossenArmDriver()
        d.configure(ta.Model.wxai_v0, ta.StandardEndEffector.wxai_v0_follower,
                    "192.168.10.4", False)
        g = d.get_gripper_position()
        d.set_all_positions(ta.VectorDouble(PARK + [g]), 3.0, True)
        time.sleep(0.5)
        print("arm parked clear of the view")

    def grab_mean(k=10):
        # At 1.04 m the markers ride the noise floor of a single 720p frame;
        # a k-frame mean lifts detection substantially (2026-08-17).
        frames = []
        for _ in range(k):
            with urllib.request.urlopen(f"{a.depth_url}/frame/cam_high",
                                        timeout=20) as r:
                img_b = base64.b64decode(json.load(r)["rgb_png_b64"])
            frames.append(cv2.imdecode(np.frombuffer(img_b, np.uint8),
                                       cv2.IMREAD_COLOR).astype(np.float32))
        return np.clip(np.mean(frames, axis=0), 0, 255).astype(np.uint8)

    img = grab_mean()
    print(f"frame {img.shape[1]}x{img.shape[0]} (mean of 10)")

    src = open(ROOT / "tools/calibrate_from_board.py").read()
    ns = {"__name__": "cb"}
    exec(compile(src.split("def main(")[0], "cb", "exec"), ns)
    args = types.SimpleNamespace(sx=8, sy=5, square=0.034, marker=0.025,
                                 handeye=str(ROOT / "calibration/right_wrist_handeye.npz"),
                                 raw_dir=str(ROOT / "calibration/handeye_raw"))
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
    board = aruco.CharucoBoard((8, 5), 0.034, 0.025, dictionary)
    params = aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMax = 45
    params.minMarkerPerimeterRate = 0.01
    params.polygonalApproxAccuracyRate = 0.06
    detector = aruco.CharucoDetector(board, detectorParams=params)
    T = ns["board_pose_in_base"](args, board, detector)

    # Different noise realizations surface different corner subsets; union
    # over several capture rounds instead of hoping one round sees enough.
    acc: dict[int, list] = {}
    for rnd in range(6):
        frame = img if rnd == 0 else grab_mean()
        big = cv2.resize(frame, None, fx=a.scale, fy=a.scale,
                         interpolation=cv2.INTER_CUBIC)
        cc, ci, _, _ = detector.detectBoard(big)
        if cc is None:
            continue
        for corner, cid in zip(cc, ci):
            acc.setdefault(int(cid), []).append(np.asarray(corner).reshape(2))
        print(f"round {rnd}: {0 if cc is None else len(cc)} corners "
              f"(union {len(acc)})")
    n = len(acc)
    print(f"charuco corners (union): {n}")
    if n < 8:
        print("too few corners — board occluded, moved, or glare; nothing written")
        return 1
    ids_sorted = sorted(acc)
    cc = np.stack([np.mean(acc[i], axis=0).reshape(1, 2) for i in ids_sorted])
    ci = np.asarray(ids_sorted, dtype=np.int32).reshape(-1, 1)
    obj, imgp = board.matchImagePoints(cc.astype(np.float32), ci)
    px = (imgp.reshape(-1, 2) / a.scale).astype(np.float64)
    world = (T[:3, :3] @ obj.reshape(-1, 3).T).T + T[:3, 3]
    H, mask = cv2.findHomography(px, world[:, :2], cv2.RANSAC, 0.004)
    proj = cv2.perspectiveTransform(px.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(proj - world[:, :2], axis=1) * 1000
    print(f"fit: {len(px)} corners, {int(mask.sum())} inliers, "
          f"mean {err.mean():.1f} mm, max {err.max():.1f} mm")
    if np.median(err) > a.max_mm:
        print("NOT saved — worse than the guard allows")
        return 1
    from heron.config import HeronConfig
    from heron.robot.workspace import save_homography
    cfg = HeronConfig.load(a.config)
    # ORIENTATION GUARD. The board's world pose is ASSUMED (T above); a board
    # placed 180° from the assumption yields a homography that fits its own
    # corners at 0.2 mm and maps the whole table mirrored — measured
    # 2026-08-17: lens crop swung off the objects, every grounding failed,
    # and two cubes 8 cm apart read 1 cm apart. A flip cannot be told from
    # residuals; it CAN be told by comparing axis directions against the
    # homography already trusted on disk.
    old_path = ROOT / "calibration/cam_high_homography.npz"
    if old_path.exists() and not getattr(a, "allow_flip", False):
        H_old = np.asarray(np.load(old_path)["h_pixel_world"], dtype=float)
        c = np.array([640.0, 400.0])
        for d in (np.array([80.0, 0.0]), np.array([0.0, 80.0])):
            def w(Hm, p):
                v = Hm @ np.array([p[0], p[1], 1.0])
                return v[:2] / v[2]
            dn = w(H, c + d) - w(H, c)
            do = w(H_old, c + d) - w(H_old, c)
            if float(np.dot(dn, do)) < 0:
                print("NOT saved — new homography maps this pixel direction "
                      f"{dn.round(3)} where the trusted one maps it {do.round(3)}: "
                      "axis flipped, board is almost certainly rotated 180° from "
                      "the assumed pose. Rotate the board and rerun, or pass "
                      "--allow-flip if the camera itself was remounted.")
                return 1
    save_homography(ROOT / "calibration/cam_high_homography.npz", H, cfg.table_z,
                    float(err.mean() / 1000))
    print("homography saved; deriving extrinsics...")
    import subprocess
    r = subprocess.run([sys.executable, str(ROOT / "tools/extrinsics_from_homography.py"),
                        "--write"], capture_output=True, text=True)
    print("\n".join(r.stdout.strip().splitlines()[-2:]))
    return r.returncode


if __name__ == "__main__":
    raise SystemExit(main())
