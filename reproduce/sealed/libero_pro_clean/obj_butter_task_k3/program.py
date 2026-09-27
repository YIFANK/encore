"""Pick the orange juice and place it in the basket (c2clean obj_butter_task k3)."""
import numpy as np
from collections import deque

PROVENANCE = {
    "GRASP_Z": {
        "source": "mate pack (c2clean_obj_butter_task_mate) keyframe ee z at gripper_cmd=1: "
                  "0.1203/0.0931/0.0981 -> the height the orange juice is grasped at; table top "
                  "measured at z=0.001 from debug-seed cam_high depth",
        "allowed": True},
    "TOP_OFFSET": {
        "source": "mate pack grasp z (~0.100) minus this scene's measured juice top (0.139, debug "
                  "seeds 51/53/55/57 cam_high) -> grasp 0.040 below the carton top",
        "allowed": True},
    "LIFT_Z": {"source": "mate pack ee_path6 carry altitude 0.29-0.31", "allowed": True},
    "RELEASE_Z": {
        "source": "k3 pack (c2clean_obj_butter_task_k3) release keyframes ee z 0.16/0.18/0.23 over "
                  "the basket; mate pack 0.15-0.18", "allowed": True},
    "LABEL_RGB": {
        "source": "debug seeds 51/53/55/57 cam_high RGB: fraction of component pixels with "
                  "r>110,g>60,b<70 is 0.14 for the orange-juice carton and <=0.01 for every other "
                  "component; the carton's printed orange label is visible in both packs' keyframes",
        "allowed": True},
    "Z_FLOOR": {"source": "debug-seed cam_high depth: table plane at z=0.001 in base frame",
                "allowed": True},
    "ARM_Z": {"source": "debug-seed cam_high: robot arm component reaches z=0.49, props <=0.15",
              "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics: tool-to-world matrix for a straight-down "
                         "wrist; matches api.tool_rotation() at reset (diag(1,-1,-1))",
               "allowed": True},
    "BASKET_GREY": {
        "source": "debug seeds 51-65 cam_high: the basket component is >1200 px and >40% of its "
                  "pixels have |r-b|<20 (grey wicker); every prop component is coloured",
        "allowed": True},
    "HOLD_CHECK": {
        "source": "debug-seed api.gripper() after a successful close on the carton: effort 3.0, "
                  "width 0.053", "allowed": True},
    "FUSE_SPAN": {
        "source": "debug-seed cam_high: the juice carton's top band spans 0.016 x 0.049 m, so a "
                  "top-band span above 0.09 m means the blob has fused with a neighbour",
        "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
Z_FLOOR = 0.02
ARM_Z = 0.35
TOP_OFFSET = 0.040
LIFT_Z = 0.29
RELEASE_Z = 0.175
X_LO, X_HI, Y_LO, Y_HI = -0.40, 0.40, -0.42, 0.45


def cloud(fr, step=2):
    rgb = np.asarray(fr.rgb, dtype=np.uint8)[::step, ::step, :].astype(np.float32)
    d = np.asarray(fr.depth, dtype=np.float32)[::step, ::step]
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    f = K[0, 0] / step
    cx, cy = K[0, 2] / step, K[1, 2] / step
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    P = np.stack([(u - cx) / f * d, (v - cy) / f * d, d], -1)
    P = P @ T[:3, :3].T + T[:3, 3]
    return P, rgb


def blobs(P, minpx=60):
    Z = P[..., 2]
    m = ((Z > Z_FLOOR) & (Z < ARM_Z) & (P[..., 0] > X_LO) & (P[..., 0] < X_HI)
         & (P[..., 1] > Y_LO) & (P[..., 1] < Y_HI))
    H, W = m.shape
    lab = np.zeros(m.shape, np.int32)
    out = []
    for r0, c0 in np.argwhere(m):
        if lab[r0, c0]:
            continue
        q = deque([(r0, c0)]); lab[r0, c0] = 1; pix = []
        while q:
            a, b = q.popleft(); pix.append((a, b))
            for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                x, y = a + da, b + db
                if 0 <= x < H and 0 <= y < W and m[x, y] and not lab[x, y]:
                    lab[x, y] = 1; q.append((x, y))
        if len(pix) >= minpx:
            out.append(np.array(pix))
    return out


def describe(P, rgb, pix):
    r, c = pix[:, 0], pix[:, 1]
    z = P[r, c, 2]
    top = float(np.percentile(z, 98))
    band = z > top - 0.02
    col = rgb[r, c]
    return dict(
        n=int(len(pix)), top=top,
        x=float(P[r, c, 0][band].mean()), y=float(P[r, c, 1][band].mean()),
        xspan=float(P[r, c, 0][band].max() - P[r, c, 0][band].min()),
        yspan=float(P[r, c, 1][band].max() - P[r, c, 1][band].min()),
        pix=pix,
        lab=float(np.mean((col[:, 0] > 110) & (col[:, 1] > 60) & (col[:, 2] < 70))),
        grey=float(np.mean(np.abs(col[:, 0] - col[:, 2]) < 20)),
        rgb=[float(q) for q in col.mean(0)])


def label_centroid(P, rgb, pix, top):
    r, c = pix[:, 0], pix[:, 1]
    col = rgb[r, c]
    sel = ((col[:, 0] > 110) & (col[:, 1] > 60) & (col[:, 2] < 70))
    if sel.sum() < 10:
        sel = P[r, c, 2] > top - 0.02
    return float(P[r, c, 0][sel].mean()), float(P[r, c, 1][sel].mean())


def run(api):
    api.log("instruction: %s" % api.instruction())
    P, rgb = cloud(api.capture("cam_high"))
    dets = [describe(P, rgb, p) for p in blobs(P)]
    for d in dets:
        api.log("blob %s" % {k: (round(v, 4) if isinstance(v, float) else v)
                             for k, v in d.items() if k != "pix"})

    # target: the component with the most bright-orange label pixels
    target = max(dets, key=lambda d: d["lab"])
    # basket: the largest washed-out (grey wicker) component that is not the target
    cands = [d for d in dets if d is not target and d["n"] > 1200 and d["grey"] > 0.4]
    if not cands:
        cands = [d for d in dets if d is not target and d["n"] > 1200]
    basket = max(cands, key=lambda d: d["n"]) if cands else None
    api.log("TARGET %s" % {k: v for k, v in target.items() if k != "pix"})
    api.log("BASKET %s" % (None if basket is None else
                           {k: v for k, v in basket.items() if k != "pix"}))

    gx, gy = target["x"], target["y"]
    if max(target["xspan"], target["yspan"]) > 0.09:
        # the carton fused with a neighbour: fall back to its own label pixels
        gx, gy = label_centroid(P, rgb, target["pix"], target["top"])
        api.log("fused blob -> label centroid %.4f %.4f" % (gx, gy))
    gz = target["top"] - min(TOP_OFFSET, 0.45 * target["top"])
    api.log("grasp at %.4f %.4f %.4f" % (gx, gy, gz))

    holding = False
    for attempt in range(2):
        api.grip(0.08)
        api.move([gx, gy, LIFT_Z], rotation=R_DOWN, seconds=3.0)
        api.move([gx, gy, gz + 0.06], rotation=R_DOWN, seconds=2.0)
        res = api.move([gx, gy, gz], rotation=R_DOWN, seconds=2.0)
        api.log("attempt %d descend residual %s eef %s" % (attempt, res, api.eef()))
        api.grip(0.0)
        api.settle(0.4)
        g = api.gripper()
        api.log("attempt %d close grip %s eef %s" % (attempt, g, api.eef()))
        api.move([gx, gy, LIFT_Z], rotation=R_DOWN, seconds=3.0)
        g = api.gripper()
        api.log("attempt %d lift grip %s eef %s" % (attempt, g, api.eef()))
        holding = g["effort"] >= 2.5 and g["width_m"] > 0.012
        if holding:
            break
        # missed: re-perceive from the new vantage and aim again, one rung lower
        P2, rgb2 = cloud(api.capture("cam_high"))
        d2 = [describe(P2, rgb2, p) for p in blobs(P2)]
        if d2:
            t2 = max(d2, key=lambda d: d["lab"])
            api.log("retry target %s" % {k: (round(v, 4) if isinstance(v, float) else v)
                                         for k, v in t2.items() if k != "pix"})
            if t2["lab"] > 0.03:
                gx, gy = t2["x"], t2["y"]
                gz = max(0.02, t2["top"] - min(TOP_OFFSET + 0.015, 0.45 * t2["top"]))
    api.log("holding=%s" % holding)

    bx = basket["x"] if basket else 0.02
    by = basket["y"] if basket else 0.26
    api.move([bx, by, LIFT_Z], rotation=R_DOWN, seconds=3.0)
    rz = RELEASE_Z if basket is None else max(RELEASE_Z, basket["top"] + 0.035)
    api.move([bx, by, rz], rotation=R_DOWN, seconds=2.0)
    api.log("over basket grip %s eef %s" % (api.gripper(), api.eef()))
    api.grip(0.08)
    api.settle(0.6)
    api.move([bx, by, LIFT_Z], rotation=R_DOWN, seconds=2.0)
    api.log("done eef %s" % api.eef())
