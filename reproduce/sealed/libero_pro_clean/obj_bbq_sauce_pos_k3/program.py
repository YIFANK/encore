"""c2clean / obj_bbq_sauce_pos_k3 -- v3

Pick the bbq sauce (the amber bottle with the dark-red cap) and drop it in
the basket.

Perception: cam_high RGB-D -> base-frame point cloud -> height mask ->
connected components.  The target is the standing prop whose TOP band is the
most red-dominant (the dark-red cap); the basket is the largest low cluster.
Grasp xy comes from the mid-point of the cap disc (fully visible from above,
so unbiased), grasp height from the pack's demo offset below the cap.
"""

import numpy as np

PROVENANCE = {
    "Z_TABLE": {
        "source": "debug seeds 51/53/57/61 cam_high depth: table plane "
                  "deprojects to z~0.001 m; 0.015 m clears it",
        "allowed": True},
    "WORKSPACE": {
        "source": "fair_run stdout banner + debug-seed point cloud extent",
        "allowed": True},
    "CAP_BAND": {
        "source": "debug-seed measurement: the bbq cap disc spans ~12 mm of "
                  "height above the neck",
        "allowed": True},
    "RED_MIN": {
        "source": "debug seeds 51/53/57/61: cap redness R-(G+B)/2 is 0.21 for "
                  "the bbq cap and <=0.01 for every other prop cap",
        "allowed": True},
    "TARGET_H": {
        "source": "debug seeds 51/53/57/61: bbq top height 0.113 m "
                  "(fallback ranking only)",
        "allowed": True},
    "GRASP_DROP": {
        "source": "pack.json demo0/demo1 close keyframes (ee z 0.0732/0.0739) "
                  "minus the measured cap height 0.113 -> 0.040 m below top",
        "allowed": True},
    "GRIP_OPEN/GRIP_CLOSE": {
        "source": "debug-seed api.gripper(): open width 0.0778 m; pack demo "
                  "gripper_state while holding 0.0187*2 = 0.037 m",
        "allowed": True},
    "CARRY_Z": {
        "source": "pack.json ee_path max z (0.27-0.32) and demo lift "
                  "keyframes; generic clearance above the 0.144 m basket rim",
        "allowed": True},
    "RELEASE_Z": {
        "source": "pack.json release keyframes ee z 0.174/0.176/0.207",
        "allowed": True},
    "R_DOWN": {
        "source": "debug-seed api.tool_rotation() at reset (straight-down "
                  "tool frame) / generic controller mechanics",
        "allowed": True},
}

Z_TABLE = 0.015
WS = (-0.40, 0.35, -0.40, 0.45)          # xmin, xmax, ymin, ymax
CAP_BAND = 0.012
RED_MIN = 0.10
TARGET_H = 0.113
GRASP_DROP = 0.040
GRIP_OPEN = 0.080
GRIP_CLOSE = 0.000
CARRY_Z = 0.25
RELEASE_Z = 0.185
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


# ---------------------------------------------------------------- perception

def cloud(frame):
    d = np.asarray(frame.depth, float)
    h, w = d.shape
    K = np.asarray(frame.intrinsics, float)
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1) @ np.asarray(
        frame.t_base_cam, float).T
    return P[..., :3]


def components(mask):
    """4-connected labelling, pure numpy/python."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    nxt = 0
    idx = np.argwhere(mask)
    for v0, u0 in idx:
        if lab[v0, u0]:
            continue
        nxt += 1
        stack = [(int(v0), int(u0))]
        lab[v0, u0] = nxt
        while stack:
            v, u = stack.pop()
            for dv, du in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = v + dv, u + du
                if 0 <= a < h and 0 <= b < w and mask[a, b] and not lab[a, b]:
                    lab[a, b] = nxt
                    stack.append((a, b))
    return lab, nxt


def perceive(api):
    f = api.capture("cam_high")
    P = cloud(f)
    rgb = np.asarray(f.rgb, float) / 255.0
    red = rgb[..., 0] - 0.5 * (rgb[..., 1] + rgb[..., 2])
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = ((Z > Z_TABLE) & (X > WS[0]) & (X < WS[1])
         & (Y > WS[2]) & (Y < WS[3]) & np.isfinite(Z))
    lab, n = components(m)
    props = []
    for i in range(1, n + 1):
        s = lab == i
        npx = int(s.sum())
        if npx < 80:
            continue
        ztop = float(Z[s].max())
        if ztop > 0.30:                        # the robot arm
            continue
        cap = s & (Z > ztop - CAP_BAND)
        props.append(dict(
            i=i, n=npx, ztop=ztop,
            cx=float((X[s].min() + X[s].max()) / 2),
            cy=float((Y[s].min() + Y[s].max()) / 2),
            capx=float((X[cap].min() + X[cap].max()) / 2),
            capy=float((Y[cap].min() + Y[cap].max()) / 2),
            capw=float(X[cap].max() - X[cap].min()),
            red=float(red[cap].mean()),
            ywid=float(Y[s].max() - Y[s].min())))
    for p in props:
        api.log("PROP n=%d ztop=%.3f c=(%.3f,%.3f) cap=(%.3f,%.3f) capw=%.3f "
                "red=%.3f ywid=%.3f"
                % (p["n"], p["ztop"], p["cx"], p["cy"], p["capx"], p["capy"],
                   p["capw"], p["red"], p["ywid"]))
    return props


def pick_target(api, props):
    big = max(p["n"] for p in props)
    cand = [p for p in props
            if 0.05 < p["ztop"] < 0.28 and p["n"] < 0.5 * big]
    if not cand:
        cand = list(props)
    red = max(cand, key=lambda p: p["red"])
    if red["red"] >= RED_MIN:
        api.log("TARGET by cap redness %.3f" % red["red"])
        return red
    hit = min(cand, key=lambda p: abs(p["ztop"] - TARGET_H))
    api.log("TARGET by height fallback (best red %.3f)" % red["red"])
    return hit


def pick_basket(api, props):
    low = [p for p in props if p["ztop"] < 0.30]
    b = max(low, key=lambda p: p["n"])
    api.log("BASKET n=%d ztop=%.3f c=(%.3f,%.3f)" % (b["n"], b["ztop"],
                                                     b["cx"], b["cy"]))
    return b


# ------------------------------------------------------------------- control

def holding(api):
    g = api.gripper()
    api.log("grip width=%.4f effort=%.2f" % (g["width_m"], g["effort"]))
    return g["width_m"] > 0.012


def run(api):
    api.log("instruction=%r" % api.instruction())
    props = perceive(api)
    if not props:
        api.log("no props seen")
        return
    tgt = pick_target(api, props)
    bsk = pick_basket(api, props)
    gx, gy = tgt["capx"], tgt["capy"]
    gz = tgt["ztop"] - GRASP_DROP
    api.log("GRASP at (%.3f,%.3f,%.3f) ztop=%.3f" % (gx, gy, gz, tgt["ztop"]))

    api.grip(GRIP_OPEN)
    api.move([gx, gy, CARRY_Z], rotation=R_DOWN, seconds=3.0)
    r = api.move([gx, gy, gz], rotation=R_DOWN, seconds=3.0)
    api.log("descend residual=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(GRIP_CLOSE)
    api.settle(0.4)
    ok = holding(api)

    if not ok:                                  # retry a little lower
        api.log("RETRY grasp")
        api.grip(GRIP_OPEN)
        api.move([gx, gy, gz + 0.06], rotation=R_DOWN, seconds=2.0)
        api.move([gx, gy, gz - 0.015], rotation=R_DOWN, seconds=2.5)
        api.grip(GRIP_CLOSE)
        api.settle(0.4)
        ok = holding(api)

    api.move([gx, gy, CARRY_Z], rotation=R_DOWN, seconds=3.0)
    api.log("after lift eef=%s" % np.round(api.eef(), 4).tolist())
    holding(api)

    bx, by = bsk["cx"], bsk["cy"]
    api.move([bx, by, CARRY_Z], rotation=R_DOWN, seconds=4.0)
    api.move([bx, by, RELEASE_Z], rotation=R_DOWN, seconds=2.5)
    api.log("at release eef=%s" % np.round(api.eef(), 4).tolist())
    api.grip(GRIP_OPEN)
    api.settle(0.6)
    api.move([bx, by, CARRY_Z + 0.03], rotation=R_DOWN, seconds=2.5)
    api.settle(0.5)
    api.log("END eef=%s" % np.round(api.eef(), 4).tolist())
