import numpy as np

PROVENANCE = {
    "R_DOWN": {"source": "generic controller mechanics; equals api.tool_rotation() at reset on debug seeds 51-65", "allowed": True},
    "Z_MIN_OBJ": {"source": "debug-seed cam_high deprojected grid: table plane at z=0.001", "allowed": True},
    "WS": {"source": "debug-seed observation: every scene object lies inside this xy box", "allowed": True},
    "N_MAX": {"source": "debug seeds 51-65: bottle cluster n=54-59; fused cartons n=303; basket n=677-721", "allowed": True},
    "RED_RANK": {"source": "debug seeds 51-65 cam_high cluster colour: bottle red-ratio 0.626 vs cartons 0.461 / box 0.400 / can 0.359 / basket 0.338", "allowed": True},
    "TIP_DZ": {"source": "v6 debug-seed receipt: closed jaws pushed onto empty table floor at eef z=0.0270-0.0276 across five x positions; table is z=0.001", "allowed": True},
    "GRASP_H": {"source": "v2/v5 debug-seed measurement: bottle top z=0.112-0.113, so 0.055 is mid-body", "allowed": True},
    "CLEAR_Z": {"source": "v2 debug-seed measurement: basket rim top z=0.140-0.142", "allowed": True},
    "CATCH_LO/CATCH_HI": {"source": "v5 wrist-cam measurement: bottle y-width 0.042, jaws open 0.078, empty close reads 0.001", "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
Z_MIN_OBJ = 0.02
WS = (-0.45, 0.45, -0.45, 0.55)
N_MAX = 200
TIP_DZ = 0.027
GRASP_H = 0.055
CLEAR_Z = 0.142
CATCH_LO, CATCH_HI = 0.015, 0.070
STEP = 4


def grid(api, cam="cam_high"):
    f = api.capture(cam)
    H, W = f.depth.shape
    us = np.arange(0, W, STEP); vs = np.arange(0, H, STEP)
    g = np.zeros((len(vs), len(us), 3), np.float32)
    for a, v in enumerate(vs):
        for b, u in enumerate(us):
            g[a, b] = f.deproject(int(u), int(v))
    return g, f.rgb[::STEP, ::STEP].astype(np.float32)


def components(mask, min_n=12):
    lab = -np.ones(mask.shape, int); out = []; cur = 0
    for s in map(tuple, np.argwhere(mask)):
        if lab[s] >= 0:
            continue
        stack = [s]; lab[s] = cur; n = 0
        while stack:
            a, b = stack.pop(); n += 1
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    p, q = a + da, b + db
                    if 0 <= p < mask.shape[0] and 0 <= q < mask.shape[1] and mask[p, q] and lab[p, q] < 0:
                        lab[p, q] = cur; stack.append((p, q))
        if n >= min_n:
            out.append(lab == cur)
        cur += 1
    return out


def perceive(api):
    g, rgb = grid(api)
    X, Y, Z = g[..., 0], g[..., 1], g[..., 2]
    m = (Z > Z_MIN_OBJ) & (X > WS[0]) & (X < WS[1]) & (Y > WS[2]) & (Y < WS[3])
    bottle = basket = None
    for sel in components(m):
        n = int(sel.sum()); ztop = float(np.percentile(Z[sel], 97))
        c = rgb[sel].mean(0); red = float(c[0] / (c.sum() + 1.0))
        xc, yc = float(X[sel].mean()), float(Y[sel].mean())
        d = dict(n=n, ztop=ztop, red=red,
                 x=float((X[sel].min() + X[sel].max()) / 2),
                 y=float((Y[sel].min() + Y[sel].max()) / 2),
                 yw=float(Y[sel].max() - Y[sel].min()))
        api.log("CL n=%d ztop=%.3f xc=%.3f yc=%.3f red=%.3f" % (n, ztop, xc, yc, red))
        if 0.04 < ztop < 0.25 and n < N_MAX and (bottle is None or red > bottle["red"]):
            bottle = d
        if yc > 0.15 and ztop < 0.25 and (basket is None or n > basket["n"]):
            basket = d
    return bottle, basket


def mv(api, xyz, sec=2.0, tag="", tries=1, tol=0.008):
    """Committed move: re-issue while it still makes progress (a starved move
    stops inside the controller's own tolerance, not at a physical limit)."""
    tgt = np.array([float(v) for v in xyz])
    last = None
    for t in range(tries):
        r = api.move([float(v) for v in tgt], rotation=R_DOWN, seconds=sec)
        e = np.array(api.eef())
        api.log("MV %s.%d cmd=[%.4f,%.4f,%.4f] eef=[%.4f,%.4f,%.4f] res=%.4f" % (
            tag, t, tgt[0], tgt[1], tgt[2], e[0], e[1], e[2], r))
        if np.linalg.norm(e - tgt) < tol:
            break
        if last is not None and np.linalg.norm(e - last) < 0.002:
            api.log("MV %s stalled" % tag)
            break
        last = e
    return np.array(api.eef())


def run(api):
    api.log("INSTR %s" % api.instruction())
    api.grip(0.08)
    api.settle(0.2)
    b, k = perceive(api)
    api.log("BOTTLE %s" % b)
    api.log("BASKET %s" % k)
    if b is None or k is None:
        api.log("PERCEPTION_FAIL")
        return

    bx, by = b["x"], b["y"]
    gz = GRASP_H + TIP_DZ

    mv(api, [bx, by, 0.26], 2.5, "hover", tries=2)
    mv(api, [bx, by, gz], 3.0, "descend", tries=3, tol=0.010)
    api.log("PRE_CLOSE eef=%s" % [round(v, 4) for v in api.eef()])

    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("AFTER_CLOSE %s" % g)
    held = CATCH_LO < g["width_m"] < CATCH_HI
    api.log("HELD=%s" % held)

    mv(api, [bx, by, 0.30], 3.0, "lift", tries=3, tol=0.010)
    api.log("AFTER_LIFT %s eef=%s" % (api.gripper(), [round(v, 4) for v in api.eef()]))

    mv(api, [k["x"], k["y"], 0.30], 3.0, "over", tries=3, tol=0.012)
    api.log("OVER_BASKET %s" % api.gripper())

    rel_z = CLEAR_Z + GRASP_H + TIP_DZ + 0.02
    mv(api, [k["x"], k["y"], rel_z], 2.0, "drop", tries=2, tol=0.012)
    api.log("BEFORE_REL %s eef=%s" % (api.gripper(), [round(v, 4) for v in api.eef()]))
    api.grip(0.08)
    api.settle(0.6)
    api.log("AFTER_REL %s" % api.gripper())
    mv(api, [k["x"], k["y"], 0.32], 2.0, "retreat", tries=2)
    api.log("END eef=%s" % [round(v, 4) for v in api.eef()])
