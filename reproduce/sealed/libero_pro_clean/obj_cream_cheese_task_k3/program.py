"""Pick the alphabet soup (blue can) and place it in the basket."""
import numpy as np

PROVENANCE = {
    "GRID": {"source": "generic camera mechanics: 5 mm top-down voxel for the 512x512 cam_high cloud", "allowed": True},
    "XLO_XHI_YLO_YHI": {"source": "debug-seed measurement: cam_high cloud spans x[-1.99,0.43] y[-1.16,1.15]; props+basket all inside x[-0.40,0.30] y[-0.42,0.45] on seeds 51-65", "allowed": True},
    "ZMIN": {"source": "debug-seed measurement: table plane deprojects to z=0.000-0.005; 0.012 is the first height that separates props from it", "allowed": True},
    "CAN_TOP_LO": {"source": "debug-seed measurement: the two cylinders top out at z=0.081, the flat boxes at 0.019-0.020; 0.040 splits them", "allowed": True},
    "CAN_TOP_HI": {"source": "debug-seed measurement: basket rim 0.144, robot arm 0.47-0.49; 0.20 excludes both", "allowed": True},
    "CAN_W_LO/CAN_W_HI": {"source": "debug-seed measurement: can footprint 0.070x0.075; basket 0.160x0.175", "allowed": True},
    "BODY_FRAC": {"source": "generic camera mechanics: a point at 0.45*top inside a vertical cylinder projects onto its visible near-side body", "allowed": True},
    "BLUE_CUE": {"source": "c2clean_obj_cream_cheese_task_mate keyframes demo0_t0042/demo2_t0046: the can the demos close on is blue-bodied with a grey lid; debug-seed body colours are [29,43,81] vs [71,17,0] for the two cylinders", "allowed": True},
    "GRASP_DROP": {"source": "c2clean_obj_cream_cheese_task_mate pack keyframes: closing EEF z=0.0447/0.0450/0.0454 against a measured can top of 0.081 -> 0.036 below the top", "allowed": True},
    "HOVER_Z": {"source": "debug-seed measurement: can top 0.081, so 0.16 clears it with margin", "allowed": True},
    "CARRY_Z": {"source": "debug-seed measurement: basket rim top 0.144; carrying at 0.24 puts the held can's base (eef-0.045) at 0.195, above the rim", "allowed": True},
    "RELEASE_Z": {"source": "both packs' release keyframes: z=0.124-0.231 over the basket", "allowed": True},
    "GRIP_CLOSED_MIN": {"source": "debug-seed measurement: open gripper reads width 0.078; a can of 0.070 footprint must leave the jaws wider than 0.03 when held", "allowed": True},
}

GRID = 0.005
XLO, XHI, YLO, YHI = -0.40, 0.30, -0.42, 0.45
ZMIN = 0.012
CAN_TOP_LO, CAN_TOP_HI = 0.040, 0.200
CAN_W_LO, CAN_W_HI = 0.030, 0.120
BODY_FRAC = 0.45
GRASP_DROP = 0.036
HOVER_Z = 0.16
CARRY_Z = 0.24
RELEASE_Z = 0.20
GRIP_CLOSED_MIN = 0.030


def _cloud(f):
    d = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    x = (u - K[0, 2]) * d / K[0, 0]
    y = (v - K[1, 2]) * d / K[1, 1]
    pts = np.stack([x, y, d, np.ones_like(d)], -1)
    return (pts @ T.T)[..., :3]


def _project(f, p):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    c = T[:3, :3].T @ (np.asarray(p, float) - T[:3, 3])
    return (K[0, 0] * c[0] / c[2] + K[0, 2], K[1, 1] * c[1] / c[2] + K[1, 2])


def _patch(f, p, r=3):
    u, v = _project(f, p)
    rgb = np.asarray(f.rgb, np.uint8)
    H, W = rgb.shape[:2]
    u0, v0 = int(round(u)), int(round(v))
    sl = rgb[max(0, v0 - r):v0 + r + 1, max(0, u0 - r):u0 + r + 1].reshape(-1, 3).astype(float)
    return sl.mean(0) if len(sl) else np.zeros(3)


def _label(mask):
    nx, ny = mask.shape
    lab = np.zeros(mask.shape, int)
    cur = 0
    for i in range(nx):
        for j in range(ny):
            if mask[i, j] and lab[i, j] == 0:
                cur += 1
                lab[i, j] = cur
                stack = [(i, j)]
                while stack:
                    a, b = stack.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if 0 <= p < nx and 0 <= q < ny and mask[p, q] and lab[p, q] == 0:
                                lab[p, q] = cur
                                stack.append((p, q))
    return lab, cur


def _clusters(f):
    P = _cloud(f)
    nx = int((XHI - XLO) / GRID)
    ny = int((YHI - YLO) / GRID)
    xs, ys, zs = P[..., 0].ravel(), P[..., 1].ravel(), P[..., 2].ravel()
    ix = ((xs - XLO) / GRID).astype(int)
    iy = ((ys - YLO) / GRID).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & np.isfinite(zs)
    ix, iy, zs = ix[ok], iy[ok], zs[ok]
    order = np.argsort(zs)
    hm = np.full((nx, ny), -1.0)
    hm[ix[order], iy[order]] = zs[order]
    lab, n = _label(hm > ZMIN)
    out = []
    for k in range(1, n + 1):
        sel = lab == k
        if sel.sum() < 4:
            continue
        ii, jj = np.nonzero(sel)
        x = XLO + (ii + 0.5) * GRID
        y = YLO + (jj + 0.5) * GRID
        z = hm[sel]
        top = float(z.max())
        hi = z > top - 0.010
        out.append(dict(n=int(sel.sum()), top=top,
                        w=float(x.max() - x.min() + GRID), h=float(y.max() - y.min() + GRID),
                        cx=float(x[hi].mean()), cy=float(y[hi].mean())))
    return out


def perceive(api):
    f = api.capture("cam_high")
    cs = _clusters(f)
    for c in cs:
        api.log("CL n=%d c=(%.3f,%.3f) top=%.3f wh=(%.3f,%.3f)" % (c["n"], c["cx"], c["cy"], c["top"], c["w"], c["h"]))
    cans = [c for c in cs if CAN_TOP_LO < c["top"] < CAN_TOP_HI
            and CAN_W_LO < c["w"] < CAN_W_HI and CAN_W_LO < c["h"] < CAN_W_HI]
    for c in cans:
        body = _patch(f, (c["cx"], c["cy"], BODY_FRAC * c["top"]))
        c["body"] = body
        c["blue"] = float(body[2] / (body.sum() + 1.0))
        api.log("CAND c=(%.3f,%.3f) top=%.3f body=%s blue=%.3f" % (c["cx"], c["cy"], c["top"], [round(v) for v in body], c["blue"]))
    target = max(cans, key=lambda c: c["blue"]) if cans else None
    baskets = [c for c in cs if c["top"] > 0.08 and c["w"] > 0.12 and c["h"] > 0.12 and c["top"] < 0.30]
    basket = max(baskets, key=lambda c: c["n"]) if baskets else None
    return target, basket


def run(api):
    api.log("instruction %r" % api.instruction())
    target, basket = perceive(api)
    if target is None or basket is None:
        api.log("ABORT target=%s basket=%s" % (target, basket))
        return
    tx, ty, ttop = target["cx"], target["cy"], target["top"]
    bx, by = basket["cx"], basket["cy"]
    api.log("TARGET (%.3f,%.3f) top=%.3f blue=%.3f | BASKET (%.3f,%.3f) top=%.3f"
            % (tx, ty, ttop, target["blue"], bx, by, basket["top"]))

    gz = ttop - GRASP_DROP
    api.grip(0.08)
    api.move([tx, ty, HOVER_Z], seconds=3.0)
    api.log("hover eef %s" % np.round(api.eef(), 4).tolist())
    api.move([tx, ty, gz], seconds=2.0)
    api.log("at grasp eef %s" % np.round(api.eef(), 4).tolist())
    api.grip(0.0)
    api.settle(0.5)
    g = api.gripper()
    api.log("after close %s" % g)
    api.move([tx, ty, CARRY_Z], seconds=2.5)
    g2 = api.gripper()
    api.log("after lift %s eef %s" % (g2, np.round(api.eef(), 4).tolist()))
    if g2["width_m"] < GRIP_CLOSED_MIN:
        api.log("LOST GRIP - retry")
        api.grip(0.08)
        target2, _ = perceive(api)
        if target2 is not None:
            tx, ty, ttop = target2["cx"], target2["cy"], target2["top"]
            api.move([tx, ty, HOVER_Z], seconds=2.5)
            api.move([tx, ty, ttop - GRASP_DROP], seconds=2.0)
            api.grip(0.0)
            api.settle(0.5)
            api.move([tx, ty, CARRY_Z], seconds=2.5)
            api.log("retry grip %s" % api.gripper())
    api.move([bx, by, CARRY_Z], seconds=3.0)
    api.log("over basket eef %s" % np.round(api.eef(), 4).tolist())
    api.move([bx, by, RELEASE_Z], seconds=2.0)
    api.grip(0.08)
    api.settle(1.0)
    api.log("released eef %s grip %s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.move([bx, by, CARRY_Z], seconds=2.0)
    api.settle(0.5)
