"""c2k1clean / obj_cream_cheese_task_k1 -- "Pick the alphabet soup and place
it in the basket".

Perception: cam_high RGB-D -> base-frame cloud -> above-table clusters.
The target is the can-sized cluster whose label band reads BLUE (the mate
pack's demo object shows a dark lid over a blue band over an orange band);
the destination is the wide, tall-rimmed cluster (the basket).
"""
import numpy as np

PROVENANCE = {
    "TABLE_EPS_M": {
        "source": "debug seeds 51-55 cam_high depth: table plane sits at "
                  "z~0.001 with <1 cm noise (own capture dump)",
        "allowed": True},
    "CAN_TOP_MIN_M": {
        "source": "debug seeds 51-55: the two can-shaped clusters top out at "
                  "z=0.081; flat props top out at z=0.020",
        "allowed": True},
    "CAN_TOP_MAX_M": {
        "source": "debug seeds 51-55: cartons/boxes behind the cans top out "
                  "at z=0.125 (own deprojection)",
        "allowed": True},
    "CAN_EXT_MIN_M": {
        "source": "debug seeds 51-55: can rim bbox measures 0.064 x 0.070 m",
        "allowed": True},
    "CAN_EXT_MAX_M": {
        "source": "debug seeds 51-55: can rim bbox measures 0.064 x 0.070 m",
        "allowed": True},
    "BAND_LO_M": {
        "source": "packs/c2k1clean_obj_cream_cheese_task_mate keyframes: the demo "
                  "object's blue label band sits just below its lid; measured "
                  "on debug seeds as z in [top-0.050, top-0.020]",
        "allowed": True},
    "BAND_HI_M": {
        "source": "same as BAND_LO_M", "allowed": True},
    "BLUE_MARGIN": {
        "source": "debug seeds 51-55: blue can band B-R=+26, other can -18",
        "allowed": True},
    "RIM_BAND_M": {
        "source": "generic: top-disc points within 1 cm of the cluster apex",
        "allowed": True},
    "GRASP_DEPTH_M": {
        "source": "packs/c2k1clean_obj_cream_cheese_task_mate pack.json keyframe "
                  "t=42: gripper closes at ee z=0.0447 on an object whose top "
                  "measures 0.081 on debug seeds -> 0.035 below the apex",
        "allowed": True},
    "HOVER_M": {
        "source": "pack ee_path: the demo approaches from ~0.17 above the "
                  "object and lifts to ~0.26 (own debug clearance check)",
        "allowed": True},
    "CARRY_Z_M": {
        "source": "pack ee_path6 transport altitude 0.25-0.31; debug seeds: "
                  "tallest intervening prop tops at 0.127",
        "allowed": True},
    "BASKET_TOP_MIN_M": {
        "source": "debug seeds 51-55: basket rim at z=0.144", "allowed": True},
    "BASKET_EXT_MIN_M": {
        "source": "debug seeds 51-55: basket rim bbox 0.16 x 0.17 m",
        "allowed": True},
    "RELEASE_OVER_RIM_M": {
        "source": "pack keyframe t=128 releases at ee z=0.1525 with the basket "
                  "rim measured at 0.144 on debug seeds -> ~0.01 over the rim",
        "allowed": True},
    "OPEN_W_M": {
        "source": "api.gripper() at reset reports width 0.0778 m (fully open)",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi contract: effort 3.0 iff holding", "allowed": True},
    "WS_X": {"source": "debug seeds: props lie in x -0.20..0.20, basket to "
                       "y=+0.35 (own capture dump)", "allowed": True},
    "WS_Y": {"source": "same as WS_X", "allowed": True},
}

TABLE_EPS_M = 0.012
CAN_TOP_MIN_M = 0.055
CAN_TOP_MAX_M = 0.115
CAN_EXT_MIN_M = 0.035
CAN_EXT_MAX_M = 0.105
BAND_LO_M = 0.050
BAND_HI_M = 0.020
BLUE_MARGIN = 5.0
RIM_BAND_M = 0.010
GRASP_DEPTH_M = 0.035
HOVER_M = 0.12
CARRY_Z_M = 0.26
BASKET_TOP_MIN_M = 0.10
BASKET_EXT_MIN_M = 0.12
RELEASE_OVER_RIM_M = 0.015
OPEN_W_M = 0.078
HOLD_EFFORT = 3.0
WS_X = (-0.30, 0.42)
WS_Y = (-0.45, 0.45)


# ---------------------------------------------------------------- perception
def _cloud(f):
    d = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    good = np.isfinite(d) & (d > 0)
    z = np.where(good, d, 1.0)
    P = np.stack([(uu - K[0, 2]) * z / K[0, 0],
                  (vv - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], -1)
    return (P @ T.T)[..., :3], good


def _clusters(pts, cols):
    g = np.floor(pts[:, :2] / 0.01).astype(int)
    cell = {}
    for i, k in enumerate(map(tuple, g)):
        cell.setdefault(k, []).append(i)
    seen, out = set(), []
    for k in cell:
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            c = stack.pop()
            comp.extend(cell[c])
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    n = (c[0] + da, c[1] + db)
                    if n in cell and n not in seen:
                        seen.add(n)
                        stack.append(n)
        idx = np.asarray(comp)
        if idx.size < 60:
            continue
        p, c = pts[idx], cols[idx]
        top = float(p[:, 2].max())
        rim = p[p[:, 2] > top - RIM_BAND_M]
        band = c[(p[:, 2] < top - BAND_HI_M) & (p[:, 2] > top - BAND_LO_M)]
        out.append(dict(
            n=int(idx.size), top=top,
            cx=float((rim[:, 0].min() + rim[:, 0].max()) / 2.0),
            cy=float((rim[:, 1].min() + rim[:, 1].max()) / 2.0),
            ex=float(np.ptp(rim[:, 0])), ey=float(np.ptp(rim[:, 1])),
            blue=float(band[:, 2].mean() - band[:, 0].mean()) if band.shape[0] > 20
            else -99.0,
            rgb=c.mean(0)))
    out.sort(key=lambda d: -d["n"])
    return out


def perceive(api):
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb).astype(float)
    xyz, good = _cloud(f)
    X, Y, Z = xyz[..., 0], xyz[..., 1], xyz[..., 2]
    ws = good & (X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1])
    zt = float(np.median(Z[ws]))
    m = ws & (Z > zt + TABLE_EPS_M)
    cl = _clusters(xyz[m], rgb[m])
    api.log("table_z=%.4f clusters=%d" % (zt, len(cl)))
    for c in cl[:10]:
        api.log("  n=%5d xy=(%.3f,%.3f) top=%.3f ext=(%.3f,%.3f) blue=%.1f "
                "rgb=(%d,%d,%d)"
                % (c["n"], c["cx"], c["cy"], c["top"], c["ex"], c["ey"],
                   c["blue"], c["rgb"][0], c["rgb"][1], c["rgb"][2]))
    return cl


def pick_target(api, cl):
    cans = [c for c in cl
            if CAN_TOP_MIN_M < c["top"] < CAN_TOP_MAX_M
            and CAN_EXT_MIN_M < c["ex"] < CAN_EXT_MAX_M
            and CAN_EXT_MIN_M < c["ey"] < CAN_EXT_MAX_M]
    api.log("can candidates: %s" % [(round(c["cx"], 3), round(c["cy"], 3),
                                     round(c["blue"], 1)) for c in cans])
    if not cans:
        return None
    cans.sort(key=lambda c: -c["blue"])
    best = cans[0]
    if len(cans) > 1 and best["blue"] - cans[1]["blue"] < BLUE_MARGIN:
        api.log("WARN blue margin thin (%.1f vs %.1f)"
                % (best["blue"], cans[1]["blue"]))
    return best


def pick_basket(api, cl):
    b = [c for c in cl
         if c["top"] > BASKET_TOP_MIN_M
         and c["ex"] > BASKET_EXT_MIN_M and c["ey"] > BASKET_EXT_MIN_M
         and c["cy"] > 0.05]
    if not b:
        return None
    b.sort(key=lambda c: -c["n"])
    return b[0]


# ------------------------------------------------------------------- motion
def goto(api, xyz, seconds=2.0, tol=0.006, tries=3):
    r = 9.9
    for _ in range(tries):
        r = api.move(list(map(float, xyz)), seconds=seconds)
        if r <= tol:
            break
    e = api.eef()
    api.log("  -> (%.3f,%.3f,%.3f) res=%.4f at (%.3f,%.3f,%.3f)"
            % (xyz[0], xyz[1], xyz[2], r, e[0], e[1], e[2]))
    return r


def holding(api):
    g = api.gripper()
    return float(g.get("effort", 0.0)) >= HOLD_EFFORT - 0.1, g


def run(api):
    api.log("instruction: %s" % api.instruction())
    cl = perceive(api)
    tgt = pick_target(api, cl)
    bsk = pick_basket(api, cl)
    if tgt is None:
        api.log("no can candidate -- abort")
        return
    api.log("TARGET xy=(%.3f,%.3f) top=%.3f blue=%.1f"
            % (tgt["cx"], tgt["cy"], tgt["top"], tgt["blue"]))
    if bsk is None:
        api.log("no basket cluster -- abort")
        return
    api.log("BASKET xy=(%.3f,%.3f) top=%.3f" % (bsk["cx"], bsk["cy"], bsk["top"]))

    gx, gy = tgt["cx"], tgt["cy"]
    gz = tgt["top"] - GRASP_DEPTH_M
    bx, by = bsk["cx"], bsk["cy"]
    rz = bsk["top"] + RELEASE_OVER_RIM_M

    api.grip(OPEN_W_M)
    api.settle(0.2)

    got = False
    for attempt in range(2):
        goto(api, [gx, gy, tgt["top"] + HOVER_M], seconds=2.5)
        goto(api, [gx, gy, gz], seconds=2.0)
        api.grip(0.0)
        api.settle(0.5)
        ok, g = holding(api)
        api.log("grasp attempt %d: width=%.4f effort=%.2f ok=%s"
                % (attempt, g.get("width_m", -1), g.get("effort", -1), ok))
        goto(api, [gx, gy, CARRY_Z_M], seconds=2.5)
        ok, g = holding(api)
        api.log("after lift: width=%.4f effort=%.2f ok=%s"
                % (g.get("width_m", -1), g.get("effort", -1), ok))
        if ok:
            got = True
            break
        api.grip(OPEN_W_M)
        api.settle(0.3)
        cl = perceive(api)
        t2 = pick_target(api, cl)
        if t2 is None:
            break
        gx, gy, gz = t2["cx"], t2["cy"], t2["top"] - GRASP_DEPTH_M
        api.log("re-aim to (%.3f,%.3f,%.3f)" % (gx, gy, gz))

    if not got:
        api.log("grasp failed -- abort before transport")
        return

    goto(api, [bx, by, CARRY_Z_M], seconds=3.0)
    goto(api, [bx, by, rz], seconds=2.0)
    ok, g = holding(api)
    api.log("over basket: width=%.4f effort=%.2f" % (g.get("width_m", -1),
                                                     g.get("effort", -1)))
    api.grip(OPEN_W_M)
    api.settle(0.6)
    goto(api, [bx, by, CARRY_Z_M], seconds=2.0)
    api.settle(0.4)
    f = api.capture("cam_high")
    api.log("done; final eef=%s" % np.round(api.eef(), 3).tolist())
