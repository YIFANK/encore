"""v6 -- grasp diagnostic: press-to-table descent, wrist yaw set by YAW_DEG.

Target hypothesis: of the two flat ~75x40x18 mm bricks on the table, the BLUE
one is the cream cheese.  Grasp top-down across its short (y) axis, carry high,
release over the basket interior.
"""
import numpy as np

PROVENANCE = {
    "WS_X": {"source": "debug-seed cam_high cloud extent", "allowed": True},
    "WS_Y": {"source": "debug-seed cam_high cloud extent", "allowed": True},
    "CELL": {"source": "generic: 1 cm footprint grid", "allowed": True},
    "BAND_LO": {"source": "debug seeds 51-65: 12 mm clears my measured table-plane noise", "allowed": True},
    "BAND_HI": {"source": "debug seeds 51-65: below the parked arm (eef z 0.261)", "allowed": True},
    "TIP_OFF": {"source": "v4 debug seeds 51,53,55,57: open-gripper table stall, eef z 0.0095 with table z 0.0020",
                "allowed": True},
    "SMALL_H": {"source": "v4 debug seeds: both flat bricks h=0.017-0.018, cans 0.079, cartons 0.140", "allowed": True},
    "SMALL_EXT": {"source": "v4 debug seeds: brick max extent 0.073-0.080, basket 0.156", "allowed": True},
    "GRASP_BITE": {"source": "v4 measured brick height 0.018 -> fingertips at ~mid height", "allowed": True},
    "YAW_DEG": {"source": "v6 debug-seed A/B test of the gripper closing axis", "allowed": True},
    "CARRY_Z": {"source": "v4 measured tallest obstacle (basket rim 0.143, cartons 0.140)", "allowed": True},
    "REL_Z": {"source": "v4 measured basket rim height 0.141-0.142", "allowed": True},
}

WS_X = (-0.45, 0.45)
WS_Y = (-0.45, 0.53)
CELL = 0.01
BAND_LO = 0.012
BAND_HI = 0.055
TIP_OFF = 0.0075
SMALL_H = 0.07
SMALL_EXT = 0.12
GRASP_BITE = 0.008
CARRY_Z = 0.30
REL_Z = 0.21
YAW_DEG = 0.0
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def yawed(deg):
    a = np.radians(deg)
    rz = np.array([[np.cos(a), -np.sin(a), 0.0], [np.sin(a), np.cos(a), 0.0], [0.0, 0.0, 1.0]])
    return rz @ R_DOWN


def cloud(f):
    d = np.asarray(f.depth, dtype=np.float64)
    H, W = d.shape
    K = np.asarray(f.intrinsics, dtype=np.float64)
    T = np.asarray(f.t_base_cam, dtype=np.float64)
    vv, uu = np.mgrid[0:H, 0:W]
    x = (uu - K[0, 2]) / K[0, 0] * d
    y = (vv - K[1, 2]) / K[1, 1] * d
    pts = np.stack([x, y, d, np.ones_like(d)], axis=-1) @ T.T
    return pts[..., :3]


def label_grid(occ):
    NX, NY = occ.shape
    lab = -np.ones((NX, NY), int)
    n = 0
    for a in range(NX):
        for b in range(NY):
            if occ[a, b] and lab[a, b] < 0:
                st = [(a, b)]
                lab[a, b] = n
                while st:
                    p, q = st.pop()
                    for dp in (-1, 0, 1):
                        for dq in (-1, 0, 1):
                            r, s = p + dp, q + dq
                            if 0 <= r < NX and 0 <= s < NY and occ[r, s] and lab[r, s] < 0:
                                lab[r, s] = n
                                st.append((r, s))
                n += 1
    return lab, n


def column_top(zs, zt):
    top = zt + BAND_LO
    for z in np.sort(zs):
        if z < top:
            continue
        if z - top > 0.015:
            break
        top = z
    return top


def perceive(api):
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb)
    P = cloud(f)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    ws = (X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1])
    hist, edges = np.histogram(Z[ws], bins=200, range=(-0.2, 0.6))
    zt = edges[int(np.argmax(hist))] + 0.002
    NX = int((WS_X[1] - WS_X[0]) / CELL) + 1
    NY = int((WS_Y[1] - WS_Y[0]) / CELL) + 1
    gx = np.clip(np.floor((X - WS_X[0]) / CELL).astype(int), 0, NX - 1)
    gy = np.clip(np.floor((Y - WS_Y[0]) / CELL).astype(int), 0, NY - 1)
    band = ws & (Z > zt + BAND_LO) & (Z < zt + BAND_HI)
    occ = np.zeros((NX, NY), bool)
    occ[gx[band], gy[band]] = True
    lab, n = label_grid(occ)
    cells_lab = lab[gx, gy]
    above = ws & (Z > zt + BAND_LO)

    out = []
    for c in range(n):
        m = band & (cells_lab == c)
        if m.sum() < 40:
            continue
        ztop = column_top(Z[above & (cells_lab == c)], zt)
        tf = (cells_lab == c) & ws & (Z > ztop - 0.010) & (Z < ztop + 0.006)
        if tf.sum() < 10:
            tf = m
        col = rgb[m].astype(float).mean(axis=0)
        d = dict(c=c, n=int(m.sum()), ztop=float(ztop), h=float(ztop - zt),
                 x0=float(X[m].min()), x1=float(X[m].max()),
                 y0=float(Y[m].min()), y1=float(Y[m].max()),
                 tx0=float(X[tf].min()), tx1=float(X[tf].max()),
                 ty0=float(Y[tf].min()), ty1=float(Y[tf].max()),
                 col=col, blue=float(col[2] - 0.5 * (col[0] + col[1])))
        d["ext"] = max(d["x1"] - d["x0"], d["y1"] - d["y0"])
        d["cx"] = 0.5 * (d["tx0"] + d["tx1"])
        d["cy"] = 0.5 * (d["ty0"] + d["ty1"])
        out.append(d)
    return out, zt


def run(api):
    cl, zt = perceive(api)
    api.log("TABLE z=%.4f ncl=%d" % (zt, len(cl)))
    for d in cl:
        api.log("CL %d n=%d h=%.3f ext=%.3f cen=(%.3f,%.3f) tx=[%.3f,%.3f] ty=[%.3f,%.3f] "
                "rgb=(%.0f,%.0f,%.0f) blue=%.1f" % (
                    d["c"], d["n"], d["h"], d["ext"], d["cx"], d["cy"],
                    d["tx0"], d["tx1"], d["ty0"], d["ty1"],
                    d["col"][0], d["col"][1], d["col"][2], d["blue"]))

    small = [d for d in cl if d["h"] < SMALL_H and d["ext"] < SMALL_EXT]
    if not small:
        api.log("NO CANDIDATE")
        return
    t = max(small, key=lambda d: d["blue"])
    basket = max(cl, key=lambda d: d["ext"])
    api.log("TARGET c=%d cen=(%.3f,%.3f) h=%.3f blue=%.1f  BASKET c=%d cen=(%.3f,%.3f) h=%.3f" % (
        t["c"], t["cx"], t["cy"], t["h"], t["blue"], basket["c"], basket["cx"], basket["cy"], basket["h"]))

    gx_, gy_ = t["cx"], t["cy"]
    R = yawed(YAW_DEG)
    api.log("YAW %.0f R=%s" % (YAW_DEG, np.round(R, 3).tolist()))
    api.grip(0.08)
    api.move([gx_, gy_, zt + 0.18], R, seconds=2.0)
    api.log("A hover eef=%s rot=%s" % (np.round(api.eef(), 4).tolist(), np.round(api.tool_rotation(), 3).tolist()))
    api.move([gx_, gy_, zt + 0.06], R, seconds=1.5)
    # press the open fingers down until they stall: for a flat brick the tips
    # should reach the table on either side of it.
    r = api.move([gx_, gy_, zt - 0.030], R, seconds=3.0)
    api.settle(0.3)
    api.log("B pressed eef=%s resid=%s tips_z=%.4f" % (
        np.round(api.eef(), 4).tolist(), np.round(r, 4).tolist(), api.eef()[2] - TIP_OFF))
    api.grip(0.005)
    api.settle(0.5)
    api.log("C closed grip=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))

    api.move([gx_, gy_, zt + CARRY_Z], R, seconds=2.5)
    api.settle(0.3)
    api.log("D lifted eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))

    api.move([basket["cx"], basket["cy"], zt + CARRY_Z], R, seconds=2.5)
    api.log("E over basket eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.move([basket["cx"], basket["cy"], zt + REL_Z], R, seconds=2.0)
    api.settle(0.2)
    api.log("F at release eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(0.08)
    api.settle(0.6)
    api.log("G released grip=%s" % api.gripper())
    api.move([basket["cx"], basket["cy"], zt + CARRY_Z], R, seconds=2.0)
    api.settle(0.5)

    cl2, zt2 = perceive(api)
    for d in cl2:
        api.log("POST CL %d h=%.3f ext=%.3f cen=(%.3f,%.3f) blue=%.1f" % (
            d["c"], d["h"], d["ext"], d["cx"], d["cy"], d["blue"]))
