"""v5 -- v4 + fusion-resilient aim, basket-by-area, one grasp retry: perceive -> grasp the orange juice -> place in basket.

Identification: the arm is stripped by an upper z bound and the basket is the
single widest footprint; among the remaining props the orange juice is the
tall one whose top band is reddest.
Grasp: jaws close along base y (measured in v3), so the y aim comes from the
reddest part of the carton's top band -- the axis the oblique cam measures
cleanly, and the one that survives a cluster fused with a neighbour.
"""
import numpy as np

PROVENANCE = {
    "XR": {"source": "fair_run workspace banner in v1 stdout", "allowed": True},
    "YR": {"source": "fair_run workspace banner in v1 stdout", "allowed": True},
    "ZCUT": {"source": "debug seeds 51-65: table plane z=0.0010 from cloud histogram", "allowed": True},
    "ARM_Z": {"source": "debug seeds 51-65: arm cluster ztop 0.486 vs props <=0.147", "allowed": True},
    "RED_PCT": {"source": "debug seeds 51-65: red-blob y aim within 0.7mm of bbox mid", "allowed": True},
    "TALL_H": {"source": "debug seeds 51-65: tall props h 0.142-0.147, short 0.018-0.112", "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics (tool z down)", "allowed": True},
    "Z_BIAS": {"source": "v2/v3 debug seeds: commanded z lands +0.0106 high, 9/9 moves", "allowed": True},
    "Z_GRIP": {"source": "v3 close-ladder: grip holds at eef 0.1355/0.1206/0.1057, air at 0.1508", "allowed": True},
    "CARRY_Z": {"source": "v3 hang geometry + basket rim z=0.144 measured on debug seeds", "allowed": True},
    "DROP_Z": {"source": "basket rim 0.144 and carton hang 0.119 measured on debug seeds", "allowed": True},
    "OPEN_W": {"source": "debug-seed gripper() at reset: width_m 0.0778", "allowed": True},
}

XR = (-0.45, 0.45)
YR = (-0.45, 0.52)
ZCUT = 0.015
ARM_Z = 0.22
RED_PCT = 60.0
TALL_H = 0.12
OPEN_W = 0.08
Z_BIAS = 0.0106
Z_GRIP = 0.120
CARRY_Z = 0.300
DROP_Z = 0.250
R_DOWN = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])


def cloud(f):
    d = np.nan_to_num(np.asarray(f.depth, float), nan=0.0)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = d.shape
    v, u = np.mgrid[0:H, 0:W]
    x = (u - K[0, 2]) * d / K[0, 0]
    y = (v - K[1, 2]) * d / K[1, 1]
    pts = np.stack([x, y, d, np.ones_like(d)], -1) @ T.T
    return pts[..., :3], d


def grid_cluster(pts, mask, cell=0.02, minpix=60):
    P = pts[mask]
    idx = np.argwhere(mask)
    gx = np.floor((P[:, 0] - XR[0]) / cell).astype(int)
    gy = np.floor((P[:, 1] - YR[0]) / cell).astype(int)
    W = int((XR[1] - XR[0]) / cell) + 2
    H = int((YR[1] - YR[0]) / cell) + 2
    occ = np.zeros((W, H), bool)
    occ[gx, gy] = True
    lab = np.zeros((W, H), int)
    cur = 0
    for i in range(W):
        for j in range(H):
            if occ[i, j] and not lab[i, j]:
                cur += 1
                st = [(i, j)]
                lab[i, j] = cur
                while st:
                    a, b = st.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if 0 <= p < W and 0 <= q < H and occ[p, q] and not lab[p, q]:
                                lab[p, q] = cur
                                st.append((p, q))
    out = []
    pl = lab[gx, gy]
    for c in range(1, cur + 1):
        s = pl == c
        if s.sum() >= minpix:
            out.append({"pts": P[s], "pix": idx[s], "n": int(s.sum())})
    return out


def perceive(api):
    f = api.capture("cam_high")
    pts, d = cloud(f)
    rgb = np.asarray(f.rgb, float)
    fin = np.isfinite(pts).all(-1) & (d > 0.01)
    ws = fin & (pts[..., 0] > XR[0]) & (pts[..., 0] < XR[1]) \
        & (pts[..., 1] > YR[0]) & (pts[..., 1] < YR[1])
    hist, edges = np.histogram(pts[..., 2][ws], bins=200, range=(-0.1, 0.3))
    table = edges[hist.argmax()] + (edges[1] - edges[0]) / 2
    m = ws & (pts[..., 2] > table + ZCUT) & (pts[..., 2] < table + ARM_Z)

    recs = []
    for c in grid_cluster(pts, m):
        P = c["pts"]
        dx = P[:, 0].max() - P[:, 0].min()
        dy = P[:, 1].max() - P[:, 1].min()
        ztop = P[:, 2].max()
        tb = P[:, 2] > ztop - 0.04
        q = rgb[c["pix"][tb, 0], c["pix"][tb, 1]]
        rbp = q[:, 0] - q[:, 2]
        recs.append(dict(P=P, pix=c["pix"], n=c["n"], dx=dx, dy=dy, ztop=ztop,
                         h=ztop - table, area=dx * dy, Ptop=P[tb], rbp=rbp,
                         rb=float(rbp.mean()),
                         xmid=(P[:, 0].min() + P[:, 0].max()) / 2,
                         ymid=(P[:, 1].min() + P[:, 1].max()) / 2))

    # the basket is by far the widest footprint (debug: 0.027 m^2 vs <=0.0038)
    basket = max(recs, key=lambda r: r["area"])
    rim = basket["P"][basket["P"][:, 2] > basket["ztop"] - 0.012]
    basket["xmid"] = (rim[:, 0].min() + rim[:, 0].max()) / 2
    basket["ymid"] = (rim[:, 1].min() + rim[:, 1].max()) / 2
    api.log("BASKET n=%d ctr=(%+.4f,%+.4f) rim_z=%.4f area=%.4f"
            % (basket["n"], basket["xmid"], basket["ymid"], basket["ztop"], basket["area"]))

    props = [r for r in recs if r is not basket]
    for r in props:
        api.log("PROP n=%d ctr=(%+.4f,%+.4f) h=%.3f dx=%.3f dy=%.3f rb=%.1f area=%.4f"
                % (r["n"], r["xmid"], r["ymid"], r["h"], r["dx"], r["dy"], r["rb"], r["area"]))

    tall = [p for p in props if p["h"] >= TALL_H]
    juice = max(tall or props, key=lambda p: p["rb"])

    # aim y (the jaw-closing axis) at the reddest part of the top band, so a
    # cluster fused with a neighbour still aims at the juice itself
    red = juice["Ptop"][juice["rbp"] >= np.percentile(juice["rbp"], RED_PCT)]
    juice["yaim"] = (red[:, 1].min() + red[:, 1].max()) / 2
    juice["xaim"] = juice["xmid"]
    api.log("PICK juice bbox=(%+.4f,%+.4f) yaim=%+.4f ztop=%.4f h=%.3f dy=%.3f rb=%.1f table=%.4f"
            % (juice["xmid"], juice["ymid"], juice["yaim"], juice["ztop"], juice["h"],
               juice["dy"], juice["rb"], table))
    return juice, basket, table


def mv(api, x, y, z, s=2.0, tag=""):
    api.move([x, y, z - Z_BIAS], rotation=R_DOWN, seconds=s)
    e = np.asarray(api.eef(), float)
    api.log("MV %s want=(%+.4f,%+.4f,%+.4f) got=(%+.4f,%+.4f,%+.4f)"
            % (tag, x, y, z, e[0], e[1], e[2]))
    return e


def run(api):
    api.log("INSTR %s" % api.instruction())
    juice, basket, table = perceive(api)

    jx, jy = juice["xaim"], juice["yaim"]
    bx, by = basket["xmid"], basket["ymid"]

    api.grip(OPEN_W)
    mv(api, jx, jy, CARRY_Z, s=2.0, tag="hover")
    mv(api, jx, jy, Z_GRIP, s=2.0, tag="descend")
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("GRASP width=%.4f effort=%.2f" % (g["width_m"], g["effort"]))

    # one re-perceive-and-retry if the jaws closed on air or on nothing solid
    if g["effort"] < 2.0 or g["width_m"] < 0.03:
        api.log("GRASP MISS -- retry")
        api.grip(OPEN_W)
        mv(api, jx, jy, CARRY_Z, s=2.0, tag="reset")
        juice2, _, _ = perceive(api)
        jx, jy = juice2["xaim"], juice2["yaim"]
        mv(api, jx, jy, CARRY_Z, s=2.0, tag="rehover")
        mv(api, jx, jy, Z_GRIP, s=2.0, tag="redescend")
        api.grip(0.0)
        api.settle(0.4)
        g = api.gripper()
        api.log("REGRASP width=%.4f effort=%.2f" % (g["width_m"], g["effort"]))

    e = mv(api, jx, jy, CARRY_Z, s=2.0, tag="lift")
    g = api.gripper()
    api.log("LIFTED width=%.4f effort=%.2f" % (g["width_m"], g["effort"]))
    hang = Z_GRIP - table

    mv(api, bx, by, CARRY_Z, s=2.5, tag="transit")
    g = api.gripper()
    api.log("TRANSIT width=%.4f effort=%.2f hang=%.3f" % (g["width_m"], g["effort"], hang))

    mv(api, bx, by, DROP_Z, s=2.0, tag="lower")
    api.grip(OPEN_W)
    api.settle(0.5)
    g = api.gripper()
    api.log("RELEASED width=%.4f effort=%.2f" % (g["width_m"], g["effort"]))
    mv(api, bx, by, CARRY_Z, s=2.0, tag="retreat")
    api.settle(0.3)
