"""v7: perceive chocolate-pudding box, top-down grasp, place in basket."""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug-seed cam_high depth histogram mode (seeds 51-65)", "allowed": True},
    "ANCHOR_XY": {"source": "debug-seed cam_high cluster survey; the box at this xy reads CHOCOLATE PUDDING in the seed-51 wrist close-up (v5)", "allowed": True},
    "TIP_OFFSET": {"source": "debug-seed v6 probe: closed fingers stall on empty table at eef z=0.0095 (seeds 51,55)", "allowed": True},
    "GRIP_TIP_Z": {"source": "debug-seed geometry: box spans table_z..0.0295, grasp near mid-height", "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics (tool z down)", "allowed": True},
    "OPEN_W": {"source": "debug-seed api.gripper() at reset: width_m 0.0778", "allowed": True},
}
TABLE_Z = 0.0033
ANCHOR_XY = (-0.145, 0.059)
TIP_OFFSET = 0.0062
GRIP_TIP_Z = 0.010
OPEN_W = 0.08
R_DOWN = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])


def grid(f):
    K = np.asarray(f.intrinsics, float); T = np.asarray(f.t_base_cam, float)
    H, W = f.depth.shape
    vv, uu = np.mgrid[0:H, 0:W]
    z = f.depth.astype(float)
    pc = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z], -1)
    return pc @ T[:3, :3].T + T[:3, 3]


def label(mask, x, y, cell=0.012, lo=-0.9, span=1.8):
    NX = int(span / cell) + 2
    ix = np.floor((x - lo) / cell).astype(int)
    iy = np.floor((y - lo) / cell).astype(int)
    occ = np.zeros((NX, NX), bool); occ[ix[mask], iy[mask]] = True
    lab = np.zeros((NX, NX), int); cur = 0
    for a in range(NX):
        for b in range(NX):
            if occ[a, b] and lab[a, b] == 0:
                cur += 1; st = [(a, b)]; lab[a, b] = cur
                while st:
                    p, q = st.pop()
                    for dp in (-1, 0, 1):
                        for dq in (-1, 0, 1):
                            r, s = p + dp, q + dq
                            if 0 <= r < NX and 0 <= s < NX and occ[r, s] and lab[r, s] == 0:
                                lab[r, s] = cur; st.append((r, s))
    pl = np.zeros(x.shape, int); pl[mask] = lab[ix[mask], iy[mask]]
    return pl, cur


def perceive(api):
    f = api.capture("cam_high")
    P = grid(f)
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    ws = (x > -0.55) & (x < 0.45) & (y > -0.55) & (y < 0.55)
    m = ws & (z > TABLE_Z + 0.010) & (z < TABLE_Z + 0.20)
    pl, n = label(m, x, y)
    tgt, bask = None, None
    for c in range(1, n + 1):
        sel = (pl == c) & m
        k = int(sel.sum())
        if k < 150:
            continue
        zt = float(z[sel].max())
        if 0.015 < zt - TABLE_Z < 0.060:
            top = sel & (z > zt - 0.004)
            cx, cy = float(x[top].mean()), float(y[top].mean())
            d = np.hypot(cx - ANCHOR_XY[0], cy - ANCHOR_XY[1])
            api.log("cand C%d n=%d ztop=%.4f xy=(%.4f,%.4f) yr=(%.3f,%.3f) d=%.3f" % (
                c, k, zt, cx, cy, float(y[top].min()), float(y[top].max()), d))
            if tgt is None or d < tgt[0]:
                tgt = (d, cx, cy, zt, float(y[top].min()), float(y[top].max()))
        if k > 3000:
            rim = sel & (z > zt - 0.012)
            if bask is None or k > bask[0]:
                bask = (k, float(x[rim].mean()), float(y[rim].mean()), zt)
    return tgt, bask


def run(api):
    L = api.log
    tgt, bask = perceive(api)
    _, tx, ty, tzt, ylo, yhi = tgt
    _, bx, by, bzt = bask
    ty = 0.5 * (ylo + yhi)
    L("TARGET xy=(%.4f,%.4f) ztop=%.4f  BASKET xy=(%.4f,%.4f) ztop=%.4f" % (tx, ty, tzt, bx, by, bzt))

    api.grip(OPEN_W)
    api.move([tx, ty, 0.16], rotation=R_DOWN, seconds=2.5)
    gz = GRIP_TIP_Z + TIP_OFFSET
    r = api.move([tx, ty, gz], rotation=R_DOWN, seconds=2.5)
    L("descend eef=%s resid=%.4f" % (np.asarray(api.eef()).round(4).tolist(), r))
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    L("closed %s" % g)
    api.move([tx, ty, 0.26], rotation=R_DOWN, seconds=2.5)
    api.settle(0.3)
    L("lifted eef=%s grip=%s" % (np.asarray(api.eef()).round(4).tolist(), api.gripper()))
    api.move([bx, by, 0.26], rotation=R_DOWN, seconds=3.0)
    api.settle(0.3)
    L("overbasket eef=%s grip=%s" % (np.asarray(api.eef()).round(4).tolist(), api.gripper()))
    api.move([bx, by, 0.21], rotation=R_DOWN, seconds=2.0)
    api.grip(OPEN_W)
    api.settle(0.6)
    L("released eef=%s grip=%s" % (np.asarray(api.eef()).round(4).tolist(), api.gripper()))
    api.move([bx, by, 0.30], rotation=R_DOWN, seconds=2.0)
    api.settle(0.5)
    t2, b2 = perceive(api)
    L("after: target %s basket %s" % (None if t2 is None else np.round(t2, 4).tolist(),
                                      None if b2 is None else np.round(b2, 4).tolist()))
    L("END v7")
