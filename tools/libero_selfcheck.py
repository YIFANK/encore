"""Empirically resolve LIBERO camera conventions: for each flip combination,
project the ground-truth eef position into the image, read depth there,
deproject back, and report the round-trip error. The right convention is the
one with mm-level error. Run on a machine with LIBERO installed."""
import os, sys
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
from PIL import Image
from heron.config import HeronConfig
from heron.robot.libero import LiberoRobot
from heron.types import Frame

cfg = HeronConfig.load(sys.argv[1] if len(sys.argv) > 1 else "configs/libero.yaml")
robot = LiberoRobot(cfg)
print("task:", robot.task_language)
sim = robot.env.env.sim if hasattr(robot.env, "env") else robot.env.sim
CU = robot._cu
gt = robot.get_cartesian("arm")
print("eef GT:", np.round(gt, 4).tolist())

rs_cam = "agentview"
rgb_raw = np.asarray(robot.obs[f"{rs_cam}_image"], np.uint8)
depth_raw = np.asarray(robot.obs[f"{rs_cam}_depth"])
h, w = rgb_raw.shape[:2]
K = CU.get_camera_intrinsic_matrix(sim, rs_cam, h, w)
ext = CU.get_camera_extrinsic_matrix(sim, rs_cam)

def project(T, p):
    pc = np.linalg.inv(T) @ np.array([*p, 1.0])
    if pc[2] <= 0: return None
    return int(K[0,0]*pc[0]/pc[2]+K[0,2]), int(K[1,1]*pc[1]/pc[2]+K[1,2])

TABLE_Z = 0.90
for flip_img in (False, True):
    img = rgb_raw[::-1] if flip_img else rgb_raw
    dep = CU.get_real_depth_map(sim, depth_raw[::-1] if flip_img else depth_raw).squeeze().astype(np.float32)
    f = Frame(camera="c", rgb=img, depth=dep, intrinsics=K, t_base_cam=ext)
    zs = []
    for u in range(w // 3, 2 * w // 3, 24):
        for v in range(2 * h // 3, h - 8, 12):   # lower-middle: table surface pixels
            p3 = f.deproject(u, v)
            if p3 is not None:
                zs.append(p3[2])
    med = float(np.median(zs)) if zs else float("nan")
    print(f"flip_img={flip_img}: median table-region z = {med:.3f} (expect ~{TABLE_Z}) over {len(zs)} px")
    Image.fromarray(img).save(f"/tmp/libero_agentview_flip{int(flip_img)}.png")
    if not flip_img:
        eefpx = project(ext, gt)
        if eefpx:
            back = f.deproject(*eefpx)
            if back is not None:
                print(f"eef roundtrip err {float(np.linalg.norm(back-gt))*1000:.0f} mm "
                      f"(includes gripper-surface offset)")
robot.shutdown()
