"""v3 -- perceive the milk carton, grasp it, drop it in the basket.

Identification (all re-derived from debug seeds 51..65 of this cell):
the scene holds six props plus a basket. Clustering the cam_high point cloud
above the table separates them. The milk is a gable-top carton: unlike the
three sauce bottles it keeps its full cross-section right up to the top, and
its upper band is red-dominant. The basket is by far the largest cluster.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-65: modal cam_high depth plane inside the "
                          "workspace crop sits at z=0.001 in base frame",
                "allowed": True},
    "Z_CLUSTER_MIN": {"source": "debug-seed measurement: table plane +11 mm separates "
                                "props from the table without fusing them",
                      "allowed": True},
    "WS_X": {"source": "debug-seed measurement: all props and the basket fall inside "
                       "x in (-0.42, 0.42) of the cam_high cloud",
             "allowed": True},
    "WS_Y": {"source": "debug-seed measurement: ditto for y in (-0.46, 0.46)",
             "allowed": True},
    "PROP_PX": {"source": "debug-seed measurement: prop clusters span 970..2230 px, "
                          "the basket 11.4k-11.8k, the arm 9903",
                "allowed": True},
    "PROP_ZTOP": {"source": "debug-seed measurement: prop tops at 0.020..0.148 m, "
                            "arm cluster top at 0.447 m",
                  "allowed": True},
    "TOPBAND_FULL": {"source": "debug-seed measurement: the carton's cross-section "
                               "35..5 mm below its top still spans ~0.050x0.052 m, "
                               "while ketchup/ranch/bbq necks taper to 0.016..0.036",
                     "allowed": True},
    "RED_DOMINANCE": {"source": "debug-seed measurement: upper-band mean RGB of the "
                                "carton is [122,78,67] (R-G = 44); ketchup 12, "
                                "ranch -23, can -5",
                      "allowed": True},
    "BASKET_PX_MIN": {"source": "debug-seed measurement: basket cluster >= 11370 px, "
                                "next largest non-arm prop 2225 px",
                      "allowed": True},
    "Z_GRASP": {"source": "packs/c2k1clean_obj_ketchup_task_mate pack.json keyframe "
                          "t=47: the demo closes on the carton at ee z = 0.1001",
                "allowed": True},
    "Z_CARRY": {"source": "packs/c2k1clean_obj_ketchup_task_k1 ee_path t=190..210 "
                          "transports at z = 0.29",
                "allowed": True},
    "HOLD_EFFORT": {"source": "FairApi docstring: gripper effort is 3.0 iff holding",
                    "allowed": True},
    "ZTOP_MIN/FULLNESS_MIN/REDNESS_MIN": {
        "source": "debug-seed measurement: carton ztop 0.139-0.140, full-to-top "
                  "cross-section 0.050x0.052, upper-band R-G = +44; the only other "
                  "box-shaped prop (the can) tops out at 0.081 with R-G = -5, and "
                  "the three bottles taper to 0.016..0.036",
        "allowed": True},
    "Z_RELEASE_OVER_RIM": {
        "source": "debug-seed measurement: the carton is held 0.100 m above its own "
                  "base, so clearing a rim measured at ztop 0.142 needs the tool at "
                  "least 0.10 above it; 0.13 adds margin",
        "allowed": True},
    "HOME": {"source": "debug-seed measurement: episode-start eef is "
                       "[-0.1485, 0.0, 0.2613]", "allowed": True},
    "MAX_PASSES": {"source": "this cell's own retry budget", "allowed": True},
    "GRIP_LADDER": {"source": "debug-seed probe of this cell (v2): grasp heights "
                              "retried around the pack-derived Z_GRASP",
                    "allowed": True},
}

TABLE_Z = 0.001
Z_CLUSTER_MIN = 0.012
WS_X = (-0.42, 0.42)
WS_Y = (-0.46, 0.46)
PROP_PX = (600, 4000)
BASKET_PX_MIN = 6000
ARM_ZTOP = 0.30
ZTOP_MIN = 0.09
FULLNESS_MIN = 0.035
REDNESS_MIN = 20.0
Z_GRASP = 0.100
Z_CARRY = 0.29
Z_HOVER = 0.26
GRIP_LADDER = (0.100, 0.080, 0.120)
Z_RELEASE_OVER_RIM = 0.13
HOME = (-0.15, 0.0)
MAX_PASSES = 2

# --------------------------------------------------------------- perception

def _world(frame):
    dep = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    H, W = dep.shape
    v, u = np.mgrid[0:H, 0:W]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    with np.errstate(all="ignore"):
        xc = (u - cx) * dep / fx
        yc = (v - cy) * dep / fy
        P = np.stack([xc, yc, dep, np.ones_like(dep)], -1)
        B = P @ T.T
    return B[..., 0], B[..., 1], B[..., 2]


def _label(mask):
    """4-connected components without scipy."""
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    seen = np.zeros((H, W), bool)
    cur = 0
    for sy, sx in np.argwhere(mask):
        if seen[sy, sx]:
            continue
        cur += 1
        stack = [(int(sy), int(sx))]
        seen[sy, sx] = True
        while stack:
            y, x = stack.pop()
            lab[y, x] = cur
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < H and 0 <= nx < W and mask[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    stack.append((ny, nx))
    return lab, cur


def perceive(api):
    f = api.capture("cam_high")
    X, Y, Z = _world(f)
    rgb = np.asarray(f.rgb, float)
    ok = np.isfinite(X) & np.isfinite(Y) & np.isfinite(Z)
    m = (ok & (X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1])
         & (Z > Z_CLUSTER_MIN) & (Z < 0.45))
    lab, n = _label(m)
    cl = []
    for i in range(1, n + 1):
        s = lab == i
        px = int(s.sum())
        if px < 300:
            continue
        xs, ys, zs = X[s], Y[s], Z[s]
        ztop = float(np.percentile(zs, 98))
        band = (zs > ztop - 0.035) & (zs < ztop - 0.005)
        if band.sum() >= 20:
            tx = float(xs[band].max() - xs[band].min())
            ty = float(ys[band].max() - ys[band].min())
        else:
            tx = ty = 0.0
        up = zs > ztop - 0.05
        col = [float(rgb[..., c][s][up].mean()) for c in range(3)] if up.sum() else [0, 0, 0]
        cl.append(dict(px=px, x=float(np.median(xs)), y=float(np.median(ys)),
                       ztop=ztop, topx=tx, topy=ty, col=col))
    return f, cl


def pick_milk(cl):
    """The carton is the only prop that is BOTH box-shaped (keeps its full
    cross-section to the top, unlike the three tapering sauce bottles) AND
    red-dominant in its upper band (unlike the blue-grey can, which is the
    other box-shaped prop but is short and not red). Both gates are applied
    independently so neither has to carry the decision alone."""
    props = [c for c in cl
             if PROP_PX[0] <= c["px"] <= PROP_PX[1] and c["ztop"] <= ARM_ZTOP]
    for c in props:
        c["fullness"] = min(c["topx"], c["topy"])
        c["redness"] = c["col"][0] - c["col"][1]
        c["score"] = 100.0 * c["fullness"] + c["redness"]
    strict = [c for c in props
              if c["fullness"] >= FULLNESS_MIN and c["redness"] >= REDNESS_MIN
              and c["ztop"] >= ZTOP_MIN]
    pool = strict if strict else [c for c in props if c["ztop"] >= ZTOP_MIN]
    if not pool:
        pool = props
    if not pool:
        return None, -1e9, "none"
    best = max(pool, key=lambda c: c["score"])
    return best, best["score"], ("strict" if strict else "relaxed")


def pick_basket(cl):
    best = None
    for c in cl:
        if c["px"] >= BASKET_PX_MIN and c["ztop"] < ARM_ZTOP:
            if best is None or c["px"] > best["px"]:
                best = c
    return best


# ------------------------------------------------------------------ motion

def goto(api, xyz, seconds=2.0, tries=2):
    r = api.move(xyz, seconds=seconds)
    for _ in range(tries - 1):
        if r < 0.01:
            break
        r = api.move(xyz, seconds=seconds)
    return r


def held(api):
    g = api.gripper()
    return float(g.get("effort", 0.0)) > 1.0, float(g.get("width_m", 0.0))


def attempt(api, pas):
    """One full perceive -> grasp-ladder -> transport -> release cycle.
    Returns a short verdict string."""
    f, cl = perceive(api)
    for c in sorted(cl, key=lambda c: -c["px"]):
        api.log("p%d cl px%5d x%+.3f y%+.3f ztop%.3f top(%.3f,%.3f) col[%3.0f %3.0f %3.0f]"
                % (pas, c["px"], c["x"], c["y"], c["ztop"], c["topx"], c["topy"], *c["col"]))

    milk, score, mode = pick_milk(cl)
    basket = pick_basket(cl)
    if milk is None or basket is None:
        api.log("p%d PERCEPTION FAIL milk=%s basket=%s" % (pas, milk is None, basket is None))
        return "perception fail"
    api.log("p%d MILK x%+.3f y%+.3f ztop%.3f full%.3f red%.0f score%.1f (%s) | "
            "BASKET x%+.3f y%+.3f ztop%.3f"
            % (pas, milk["x"], milk["y"], milk["ztop"], milk["fullness"],
               milk["redness"], score, mode,
               basket["x"], basket["y"], basket["ztop"]))

    mx, my = milk["x"], milk["y"]
    ok = False
    for zi, zg in enumerate(GRIP_LADDER):
        api.grip(0.08)
        r1 = goto(api, [mx, my, Z_HOVER], seconds=2.5)
        r2 = goto(api, [mx, my, zg], seconds=2.0)
        api.grip(0.0)
        api.settle(0.4)
        h0, w0 = held(api)
        r3 = goto(api, [mx, my, Z_CARRY], seconds=2.0)
        h1, w1 = held(api)
        api.log("p%d try%d zg%.3f res(%.3f,%.3f,%.3f) close(h%d w%.4f) lift(h%d w%.4f) eef%s"
                % (pas, zi, zg, r1, r2, r3, h0, w0, h1, w1,
                   np.round(api.eef(), 3).tolist()))
        if h1 and w1 > 0.010:
            ok = True
            break
        api.grip(0.08)
        api.settle(0.2)
        goto(api, [mx, my, Z_CARRY], seconds=1.5)

    if not ok:
        api.log("p%d GRASP FAIL after ladder" % pas)
        return "grasp fail"

    bx, by = basket["x"], basket["y"]
    z_rel = basket["ztop"] + Z_RELEASE_OVER_RIM
    goto(api, [(mx + bx) * 0.5, (my + by) * 0.5, Z_CARRY], seconds=2.5)
    goto(api, [bx, by, Z_CARRY], seconds=2.5)
    h2, w2 = held(api)
    api.log("p%d over basket h%d w%.4f eef%s"
            % (pas, h2, w2, np.round(api.eef(), 3).tolist()))
    if not h2:
        api.log("p%d DROPPED in transit" % pas)
        api.grip(0.08)
        return "dropped"
    goto(api, [bx, by, z_rel], seconds=2.0)
    api.grip(0.08)
    api.settle(0.6)
    api.log("p%d released at %s" % (pas, np.round(api.eef(), 3).tolist()))
    goto(api, [bx, by, Z_CARRY], seconds=1.5)
    api.settle(0.4)
    return "placed"


def run(api):
    api.log("v3 start instr=%s" % api.instruction())
    api.grip(0.08)
    api.settle(0.2)
    verdict = "none"
    for pas in range(MAX_PASSES):
        verdict = attempt(api, pas)
        api.log("pass%d verdict=%s" % (pas, verdict))
        if verdict == "placed":
            break
        # Back off to a clear pose so the next pass re-perceives an unoccluded
        # scene, then try the whole cycle again.
        api.grip(0.08)
        goto(api, [HOME[0], HOME[1], Z_CARRY], seconds=2.5)
        api.settle(0.4)
    return "v3 %s" % verdict
