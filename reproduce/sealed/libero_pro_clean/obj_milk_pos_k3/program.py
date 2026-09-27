"""c2clean obj_milk_pos_k3 -- pick up the milk and place it in the basket.

Strategy
--------
The pack's three demos all close the gripper at (x~-0.12, y~-0.25, z~0.10) and
release over (x~0.0, y~0.26).  On the debug seeds that grasp xy holds a flat
brown box, not the milk: the *_pos* perturbation permutes which prop sits where,
so the demo xy is a decoy.  What transfers from the pack is the target's
*appearance* and the grasp/release *heights*.

Projecting the demo grasp keyframe (demo0_t0047, ee = [-0.133,-0.252,0.100])
into the pack's own agentview keyframe with the pack camera pose lands on pixel
(31,54) -- the tall red/white carton.  So the program re-perceives every episode:

  1. park the arm off to one side (at home the arm occludes the far prop row),
  2. band-mask the cam_high point cloud to the carton-top height band and
     group what is left in xy (band first, so the shorter can and the flat
     boxes cannot fuse into a carton's cluster),
  3. small-footprint clusters are the two cartons, the big one is the basket,
  4. the two cartons share geometry (0.028 x 0.051 x 0.139 m), so pick the one
     whose body is least yellow (B-G): the orange-juice carton's body is
     yellow/orange, the milk's is red on white.

Verification: the closed gripper gap must come out at the carton's own measured
y-width with effort 3.0, and the gap must survive the lift; otherwise the
program re-perceives and retries the grasp.
"""

from collections import deque

import numpy as np

PROVENANCE = {
    "R_DOWN": {"source": "debug-seed api.tool_rotation() at reset (straight-down wrist, fingers closing along world y); pack demo grasp keyframes are near-straight-down (ee_path6 roll ~3.14)", "allowed": True},
    "XLO/XHI/YLO/YHI": {"source": "generic workspace crop; walls/floor deproject far outside it on the debug seeds", "allowed": True},
    "BAND_LO/BAND_HI": {"source": "debug seeds 51-65 cam_high: carton tops 0.137-0.142, tomato can top 0.081, flat boxes 0.019-0.029, basket rim 0.143; table plane z=0.000", "allowed": True},
    "GRID_RES": {"source": "generic: xy occupancy cell for connected-component grouping, ~2x the 4.8 mm/px ground sampling of cam_high at stride 2", "allowed": True},
    "MIN_PTS": {"source": "debug-seed measurement: real carton-top clusters are 440-470 pts; 40 rejects depth speckle", "allowed": True},
    "BIG_FOOT": {"source": "debug seeds 51-65: basket footprint 0.15-0.20 m, every prop <= 0.054 m in the band", "allowed": True},
    "GRASP_DROP": {"source": "pack keyframes: demo grasp eef z 0.098/0.105/0.098 vs debug-seed carton top 0.139 -> grasp ~0.040 below the top", "allowed": True},
    "CARRY_Z": {"source": "pack ee_path6 transit z 0.30-0.33", "allowed": True},
    "RELEASE_Z": {"source": "pack keyframes: demo release eef z 0.142/0.152/0.171", "allowed": True},
    "RETREAT_POSES": {"source": "debug-seed probe: two poses that clear the arm out of cam_high's view of the prop rows (residuals 0.010/0.024)", "allowed": True},
    "OPEN_W": {"source": "debug-seed api.gripper() at reset: width_m 0.0778", "allowed": True},
    "HOLD_W_MIN": {"source": "debug-seed v4 run: a real hold reads gap 0.053 (= the carton's measured y-width); a gripper closed on nothing reads ~0.003", "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

XLO, XHI, YLO, YHI = -0.45, 0.45, -0.45, 0.52
BAND_LO, BAND_HI = 0.090, 0.200
GRID_RES = 0.01
MIN_PTS = 40
BIG_FOOT = 0.10
GRASP_DROP = 0.040
CARRY_Z = 0.32
RELEASE_Z = 0.17
OPEN_W = 0.08
HOLD_W_MIN = 0.020

RETREAT_POSES = [(-0.10, 0.40, 0.38), (0.10, 0.42, 0.30)]


def _cloud(fr, st=2):
    dep = np.asarray(fr.depth, dtype=np.float32)[::st, ::st]
    rgb = np.asarray(fr.rgb)[::st, ::st].astype(np.float32)
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    H, W = dep.shape
    fx, fy, cx, cy = K[0, 0] / st, K[1, 1] / st, K[0, 2] / st, K[1, 2] / st
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    P = np.stack([(u - cx) / fx * dep, (v - cy) / fy * dep, dep, np.ones_like(dep)], -1)
    B = (P @ T.T)[..., :3]
    return B[..., 0], B[..., 1], B[..., 2], rgb


def _clusters(fr, lo, hi):
    x, y, z, rgb = _cloud(fr)
    m = (x > XLO) & (x < XHI) & (y > YLO) & (y < YHI) & (z > lo) & (z < hi)
    X, Y, Z, C = x[m], y[m], z[m], rgb[m]
    if X.size == 0:
        return []
    gx = np.round((X - XLO) / GRID_RES).astype(int)
    gy = np.round((Y - YLO) / GRID_RES).astype(int)
    NX = int((XHI - XLO) / GRID_RES) + 2
    NY = int((YHI - YLO) / GRID_RES) + 2
    occ = np.zeros((NX, NY), bool)
    occ[gx, gy] = True
    lab = np.zeros((NX, NY), int)
    cur = 0
    for i in range(NX):
        for j in range(NY):
            if occ[i, j] and not lab[i, j]:
                cur += 1
                lab[i, j] = cur
                q = deque([(i, j)])
                while q:
                    a, b = q.popleft()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, r = a + da, b + db
                            if 0 <= p < NX and 0 <= r < NY and occ[p, r] and not lab[p, r]:
                                lab[p, r] = cur
                                q.append((p, r))
    pl = lab[gx, gy]
    out = []
    for c in range(1, cur + 1):
        s = pl == c
        n = int(s.sum())
        if n < MIN_PTS:
            continue
        cx_, cy_, cz, cc = X[s], Y[s], Z[s], C[s]
        out.append(dict(n=n, ztop=float(np.percentile(cz, 98)),
                        x0=float(cx_.min()), x1=float(cx_.max()),
                        y0=float(cy_.min()), y1=float(cy_.max()),
                        rgb=cc.mean(0)))
    return out


def _mid(c):
    return (c["x0"] + c["x1"]) / 2, (c["y0"] + c["y1"]) / 2


def _foot(c):
    return c["x1"] - c["x0"], c["y1"] - c["y0"]


def _describe(c):
    r, g, b = c["rgb"]
    mx, my = _mid(c)
    dx, dy = _foot(c)
    return ("n=%4d ztop=%.3f mid=(%.3f,%.3f) foot=(%.3f,%.3f) rgb=(%.0f,%.0f,%.0f) bg=%.1f"
            % (c["n"], c["ztop"], mx, my, dx, dy, r, g, b, b - g))


def perceive(api, tag):
    cs = _clusters(api.capture("cam_high"), BAND_LO, BAND_HI)
    for c in cs:
        api.log("CL%s %s" % (tag, _describe(c)))
    small = [c for c in cs if max(_foot(c)) < BIG_FOOT]
    big = [c for c in cs if min(_foot(c)) >= BIG_FOOT]
    milk = max(small, key=lambda c: c["rgb"][2] - c["rgb"][1]) if small else None
    basket = max(big, key=lambda c: c["n"]) if big else None
    if milk is not None:
        api.log("MILK%s %s" % (tag, _describe(milk)))
    if basket is not None:
        api.log("BASKET%s %s" % (tag, _describe(basket)))
    return milk, basket


def park(api):
    for p in RETREAT_POSES:
        r = api.move(np.array(p, dtype=float), rotation=R_DOWN, seconds=2.0)
        api.log("RETREAT %s res=%.4f" % (p, r))
    api.settle(0.3)


def try_grasp(api, milk):
    mx, my = _mid(milk)
    gz = milk["ztop"] - GRASP_DROP
    api.log("PLAN grasp=(%.3f,%.3f,%.3f)" % (mx, my, gz))
    api.grip(OPEN_W)
    api.move(np.array([mx, my, CARRY_Z]), rotation=R_DOWN, seconds=2.0)
    r = api.move(np.array([mx, my, gz]), rotation=R_DOWN, seconds=2.0)
    api.log("DESCEND res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(0.0)
    api.settle(0.4)
    api.log("CLOSED %s" % api.gripper())
    r = api.move(np.array([mx, my, CARRY_Z]), rotation=R_DOWN, seconds=2.0)
    api.settle(0.3)
    g = api.gripper()
    api.log("LIFTED res=%.4f grip=%s eef=%s" % (r, g, np.round(api.eef(), 4).tolist()))
    return g["width_m"] > HOLD_W_MIN and g["effort"] >= 3.0


def run(api):
    api.log("INSTR %s" % api.instruction())
    api.grip(OPEN_W)
    park(api)
    milk, basket = perceive(api, "0")
    if milk is None or basket is None:
        api.log("ABORT milk=%s basket=%s" % (milk is not None, basket is not None))
        return

    held = try_grasp(api, milk)
    if not held:
        api.log("RETRY grasp not verified")
        api.grip(OPEN_W)
        park(api)
        milk2, basket2 = perceive(api, "1")
        if milk2 is not None:
            milk = milk2
            held = try_grasp(api, milk)
        if basket2 is not None:
            basket = basket2
    if not held:
        api.log("GIVEUP no verified hold")
        return

    bx, by = _mid(basket)
    api.move(np.array([bx, by, CARRY_Z]), rotation=R_DOWN, seconds=2.5)
    r = api.move(np.array([bx, by, RELEASE_Z]), rotation=R_DOWN, seconds=2.0)
    api.log("OVER_BASKET res=%.4f grip=%s eef=%s" % (r, api.gripper(), np.round(api.eef(), 4).tolist()))
    api.grip(OPEN_W)
    api.settle(0.5)
    api.move(np.array([bx, by, CARRY_Z]), rotation=R_DOWN, seconds=2.0)
    api.settle(0.5)
    for c in _clusters(api.capture("cam_high"), BAND_LO, BAND_HI):
        api.log("POST %s" % _describe(c))
