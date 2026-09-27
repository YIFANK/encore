"""c2k1clean obj_milk_pos_k1 -- v1.

Intent: "pick up the milk and place it in the basket".

Perception-driven: cam_high RGB-D -> table plane -> above-table connected
components -> pick the tall, compact, red-topped carton (the milk) and the
large-footprint hollow fixture (the basket).  Then a top-down grasp at the
carton's top-slab centroid and a release over the basket's rim centroid.
"""
import numpy as np

PROVENANCE = {
    "TABLE_BAND": {
        "source": "debug seeds 51-65 v0 probe: z-mode of the cam_high cloud is "
                  "0.0062 m; objects rise 0.013-0.138 m above it",
        "allowed": True},
    "OBJ_MIN_H": {
        "source": "v0 probe on debug seeds 51-65: flat props measure h=0.013-0.029, "
                  "so 0.012 separates table noise from props",
        "allowed": True},
    "TALL_MIN_H": {
        "source": "v0 probe debug seeds: the two carton clusters measure h=0.135-0.137 "
                  "while every other prop is <=0.081; 0.10 splits them",
        "allowed": True},
    "COMPACT_MAX": {
        "source": "v0 probe debug seeds: carton footprints are dx=0.027 dy=0.048; the "
                  "basket is dx=0.145 dy=0.153; 0.09 splits them",
        "allowed": True},
    "MILK_GB_MAX": {
        "source": "pack keyframes/demo0_t0000.png: the grasped carton reads (81,45,37) "
                  "(G-B=8) vs the other tall carton (70,49,25) (G-B=24); v0 probe on "
                  "debug seeds reads the same two clusters at G-B=6 and G-B=36",
        "allowed": True},
    "BASKET_MIN_FOOT": {
        "source": "v0 probe debug seeds: basket cluster dx=0.145 dy=0.153, every prop "
                  "<=0.077",
        "allowed": True},
    "GRASP_BELOW_TOP": {
        "source": "pack demo0: gripper closes (gripper_cmd -1->1 between t=40 and t=50) "
                  "at ee z=0.1016 while the carton top sits at table+0.136=0.142, i.e. "
                  "0.040 below the top",
        "allowed": True},
    "HOVER": {
        "source": "pack demo0 ee_path6: approach waypoint t=40 is z=0.1491, ~0.05 above "
                  "the grasp; 0.10 is a conservative clear-of-props hover",
        "allowed": True},
    "CARRY_Z": {
        "source": "pack demo0 ee_path6 transport plateau t=80..110 is z=0.297-0.313",
        "allowed": True},
    "RELEASE_ABOVE_RIM": {
        "source": "pack demo0: gripper opens at t=142 with ee z=0.1423 while the basket "
                  "rim measures 0.144 in the v0 probe, i.e. release ~at rim height",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi docstring: gripper() effort is 3.0 iff holding",
        "allowed": True},
    "OPEN_WIDTH": {
        "source": "pack demo0 gripper_state at t=0/t=47 sums to 0.072/0.079 m open",
        "allowed": True},
}

TABLE_BAND = (-0.10, 0.40)
OBJ_MIN_H = 0.012
OBJ_MAX_H = 0.30
TALL_MIN_H = 0.10
COMPACT_MAX = 0.09
MILK_GB_MAX = 25.0
BASKET_MIN_FOOT = 0.10
GRASP_BELOW_TOP = 0.040
HOVER = 0.10
CARRY_Z = 0.31
RELEASE_ABOVE_RIM = 0.00
HOLD_EFFORT = 2.5
OPEN_WIDTH = 0.08
WS = 0.45


# ---------------------------------------------------------------------------
# perception

def cloud(f):
    d = np.asarray(f.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:h, 0:w]
    P = np.stack([(uu - cx) * d / fx, (vv - cy) * d / fy, d, np.ones_like(d)], -1)
    return (P @ T.T)[..., :3], np.isfinite(d) & (d > 0)


def components(mask, min_px=40):
    lab = np.zeros(mask.shape, np.int32)
    H, W = mask.shape
    n = 0
    out = []
    for (r, c) in np.argwhere(mask):
        if lab[r, c]:
            continue
        n += 1
        stack = [(r, c)]
        lab[r, c] = n
        cnt = 0
        while stack:
            a, b = stack.pop()
            cnt += 1
            for p, q in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                if 0 <= p < H and 0 <= q < W and mask[p, q] and not lab[p, q]:
                    lab[p, q] = n
                    stack.append((p, q))
        if cnt >= min_px:
            out.append(n)
    return lab, out


def scene(api, note=""):
    """-> (table_z, [cluster dicts])"""
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb)
    XYZ, ok = cloud(f)
    X, Y, Z = XYZ[..., 0], XYZ[..., 1], XYZ[..., 2]
    ws = ok & (np.abs(X) < WS) & (np.abs(Y) < WS) & (Z > TABLE_BAND[0]) & (Z < TABLE_BAND[1])
    hist, edges = np.histogram(Z[ws], bins=40, range=TABLE_BAND)
    mb = int(np.argmax(hist))
    tz = float(0.5 * (edges[mb] + edges[mb + 1]))

    obj = ws & (Z > tz + OBJ_MIN_H) & (Z < tz + OBJ_MAX_H)
    lab, keys = components(obj)
    cl = []
    for k in keys:
        m = lab == k
        zs, xs, ys = Z[m], X[m], Y[m]
        ztop = float(zs.max())
        top = zs > ztop - 0.012
        cols = rgb[m].astype(float)
        ctop = cols[top].mean(0) if top.sum() > 5 else cols.mean(0)
        vs, us = np.nonzero(m)
        cl.append(dict(
            n=int(m.sum()), u=float(us.mean()), v=float(vs.mean()),
            ztop=ztop, h=ztop - tz,
            x=float(np.median(xs)), y=float(np.median(ys)),
            xt=float(xs[top].mean()), yt=float(ys[top].mean()),
            dx=float(np.percentile(xs, 95) - np.percentile(xs, 5)),
            dy=float(np.percentile(ys, 95) - np.percentile(ys, 5)),
            rgb=cols.mean(0), rgbtop=ctop))
    for c in cl:
        api.log("%sC n=%d px=(%d,%d) top=(%.3f,%.3f) med=(%.3f,%.3f) ztop=%.3f "
                "h=%.3f d=(%.3f,%.3f) rgb=(%d,%d,%d) rt=(%d,%d,%d)"
                % (note, c["n"], c["u"], c["v"], c["xt"], c["yt"], c["x"], c["y"],
                   c["ztop"], c["h"], c["dx"], c["dy"],
                   c["rgb"][0], c["rgb"][1], c["rgb"][2],
                   c["rgbtop"][0], c["rgbtop"][1], c["rgbtop"][2]))
    return tz, cl


def pick_milk(cl, eef):
    """Tall + compact + low green-minus-blue on the top band."""
    cand = [c for c in cl
            if TALL_MIN_H <= c["h"] <= 0.25
            and c["dx"] < COMPACT_MAX and c["dy"] < COMPACT_MAX
            and np.hypot(c["xt"] - eef[0], c["yt"] - eef[1]) > 0.09]
    if not cand:
        return None
    def gb(c):
        return float(c["rgbtop"][1] - c["rgbtop"][2])
    cand.sort(key=gb)
    return cand[0] if gb(cand[0]) < MILK_GB_MAX or len(cand) == 1 else cand[0]


def pick_basket(cl):
    cand = [c for c in cl if c["dx"] > BASKET_MIN_FOOT and c["dy"] > BASKET_MIN_FOOT
            and c["h"] < 0.25]
    if not cand:
        return None
    cand.sort(key=lambda c: -c["n"])
    return cand[0]


# ---------------------------------------------------------------------------

def run(api):
    api.log("v1 start; instruction=%r" % api.instruction())
    e0 = np.asarray(api.eef(), float)
    api.log("eef0=%s R=%s" % (np.round(e0, 4).tolist(),
                              np.round(np.asarray(api.tool_rotation()), 3).tolist()))

    tz, cl = scene(api, "S0:")
    api.log("table_z=%.4f ncl=%d" % (tz, len(cl)))
    milk = pick_milk(cl, e0)
    basket = pick_basket(cl)
    if milk is None or basket is None:
        api.log("ABORT milk=%s basket=%s" % (milk is not None, basket is not None))
        return
    api.log("MILK top=(%.3f,%.3f) ztop=%.3f h=%.3f rt=(%d,%d,%d)"
            % (milk["xt"], milk["yt"], milk["ztop"], milk["h"],
               milk["rgbtop"][0], milk["rgbtop"][1], milk["rgbtop"][2]))
    api.log("BASKET top=(%.3f,%.3f) ztop=%.3f n=%d"
            % (basket["xt"], basket["yt"], basket["ztop"], basket["n"]))

    gx, gy = milk["xt"], milk["yt"]
    gz = milk["ztop"] - GRASP_BELOW_TOP

    api.grip(OPEN_WIDTH)
    r = api.move([gx, gy, milk["ztop"] + HOVER], seconds=2.5)
    api.log("hover res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    r = api.move([gx, gy, gz], seconds=2.0)
    api.log("descend res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("grip w=%.4f eff=%.2f" % (g["width_m"], g["effort"]))

    api.move([gx, gy, CARRY_Z], seconds=2.5)
    api.settle(0.3)
    g = api.gripper()
    api.log("lift w=%.4f eff=%.2f eef=%s"
            % (g["width_m"], g["effort"], np.round(api.eef(), 4).tolist()))

    bx, by = basket["xt"], basket["yt"]
    rz = basket["ztop"] + RELEASE_ABOVE_RIM
    r = api.move([bx, by, CARRY_Z], seconds=3.0)
    api.log("over-basket res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    r = api.move([bx, by, rz], seconds=2.0)
    api.log("lower res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(OPEN_WIDTH)
    api.settle(0.5)
    api.move([bx, by, CARRY_Z], seconds=2.0)
    api.settle(0.5)

    tz2, cl2 = scene(api, "S1:")
    api.log("v1 done")
