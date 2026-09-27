"""v1 -- perceive the blue cream-cheese box from cam_high, straddle-grasp it
top-down, carry it over the basket and release.

Identity chain (all from the two named packs + my own debug-seed captures):
  * The mate pack's language is "pick up the cream cheese and place it in the
    basket"; its demo0 keyframes show one small blue box present at t=0000 and
    ABSENT at t=0120 while the arm carries something to the basket.  So the
    cream cheese is the small blue box.
  * The k3 pack's t=0000 frames show the scene I am actually run in; it holds
    the same small blue box asset (crop-compared pixel patches).
  * In my own debug-seed captures that box is the only low (top ~0.017 m) blue
    cluster on the table; the other low cluster is red-brown.
"""

import numpy as np

PROVENANCE = {
    "GRASP_Z": {
        "source": "c2clean_obj_alphabet_soup_task_mate pack.json: the gripper-close "
                  "keyframes of all three demos on the cream cheese sit at eef "
                  "z = 0.0091 / 0.0101 / 0.0102 -> 0.010",
        "allowed": True},
    "R_DOWN": {
        "source": "debug-seed observation: api.tool_rotation() at episode start is "
                  "[[0.998,0,-0.057],[0,-1,0],[-0.057,0,-0.998]] i.e. the "
                  "straight-down home pose; rounded to exact",
        "allowed": True},
    "Z_BAND": {
        "source": "debug-seed cam_high height maps (seeds 51,56,62): table plane "
                  "deprojects to z ~ -0.001, props top out between 0.017 and "
                  "0.146, the robot body above 0.24",
        "allowed": True},
    "WORKSPACE": {
        "source": "debug-seed cam_high height maps: props all fall inside "
                  "x(-0.35,0.35) y(-0.45,0.55)",
        "allowed": True},
    "TOP_PLANE_TOL": {
        "source": "debug-seed observation: the blue box top face deprojects to a "
                  "constant z within 0.002 m across its pixels",
        "allowed": True},
    "LOW_TOP_MAX": {
        "source": "debug-seed observation: the two short props top out at 0.017-0.018 "
                  "while the next shortest is 0.079",
        "allowed": True},
    "CARRY_Z": {
        "source": "mate pack ee_path6 transit segments run at eef z 0.21-0.27; "
                  "0.25 is inside that band and clears the tallest prop (0.146) "
                  "measured on my debug seeds",
        "allowed": True},
    "RELEASE_Z": {
        "source": "mate pack: the three gripper-open keyframes over the basket sit "
                  "at eef z = 0.179 / 0.219 / 0.152 -> 0.185",
        "allowed": True},
    "HOVER_Z": {
        "source": "mate pack ee_path6: the descent onto the cream cheese passes "
                  "through z ~ 0.11-0.12 before the final drop",
        "allowed": True},
    "OPEN_W": {
        "source": "debug-seed observation: api.gripper() reports width 0.0778 m at "
                  "episode start (fully open)",
        "allowed": True},
    "CLOSE_W": {
        "source": "generic gripper mechanics: FairApi closes for any width < 0.025",
        "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
R_YAW90 = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])

DS = 2
Z_BAND = (0.008, 0.20)
WORKSPACE = ((-0.35, 0.35), (-0.45, 0.55))
TOP_PLANE_TOL = 0.006
LOW_TOP_MAX = 0.045
GRASP_Z = 0.010
HOVER_Z = 0.115
CARRY_Z = 0.25
RELEASE_Z = 0.185
OPEN_W = 0.08
CLOSE_W = 0.0


def cloud(frame, ds=DS):
    d = np.asarray(frame.depth, float)[::ds, ::ds]
    d = np.where(np.isfinite(d) & (d > 0), d, 0.0)
    h, w = d.shape
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    vs, us = np.mgrid[0:h, 0:w]
    U = us * ds + (ds - 1) / 2.0
    V = vs * ds + (ds - 1) / 2.0
    x = (U - K[0, 2]) * d / K[0, 0]
    y = (V - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1)
    B = P @ T.T
    rgb = np.asarray(frame.rgb)[::ds, ::ds, :].astype(float)
    return B[..., :3], rgb, d


def components(mask):
    """4-connected labelling of a small boolean mask, pure numpy/python."""
    h, w = mask.shape
    lab = np.zeros((h, w), int)
    cur = 0
    idx = np.argwhere(mask)
    seen = set()
    pos = {(int(a), int(b)) for a, b in idx}
    for a, b in idx:
        key = (int(a), int(b))
        if key in seen:
            continue
        cur += 1
        stack = [key]
        seen.add(key)
        while stack:
            i, j = stack.pop()
            lab[i, j] = cur
            for ni, nj in ((i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1),
                           (i - 1, j - 1), (i - 1, j + 1), (i + 1, j - 1), (i + 1, j + 1)):
                nk = (ni, nj)
                if nk in pos and nk not in seen:
                    seen.add(nk)
                    stack.append(nk)
    return lab, cur


def perceive(api):
    f = api.capture("cam_high")
    B, rgb, d = cloud(f)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    (x0, x1), (y0, y1) = WORKSPACE
    m = ((d > 0) & (Z > Z_BAND[0]) & (Z < Z_BAND[1]) &
         (X > x0) & (X < x1) & (Y > y0) & (Y < y1))
    lab, n = components(m)
    props = []
    for i in range(1, n + 1):
        s = lab == i
        if s.sum() < 12:
            continue
        ztop = float(Z[s].max())
        top = s & (Z >= ztop - TOP_PLANE_TOL)
        col = rgb[top].mean(0)
        blue = float(col[2] - max(col[0], col[1]))
        props.append({
            "px": int(s.sum()), "ztop": ztop,
            "cx": float(X[top].mean()), "cy": float(Y[top].mean()),
            "xspan": float(X[top].max() - X[top].min()),
            "yspan": float(Y[top].max() - Y[top].min()),
            "ylo": float(Y[s].min()), "yhi": float(Y[s].max()),
            "xlo": float(X[s].min()), "xhi": float(X[s].max()),
            "rgb": [round(v, 1) for v in col], "blue": round(blue, 1)})
    return props


def run(api):
    api.log("instruction=%r" % api.instruction())
    props = perceive(api)
    for p in props:
        api.log("prop %s" % {k: (round(v, 4) if isinstance(v, float) else v)
                             for k, v in p.items()})

    low = [p for p in props if p["ztop"] < LOW_TOP_MAX]
    if not low:
        return "no low prop found"
    tgt = max(low, key=lambda p: p["blue"])
    api.log("TARGET %s" % {k: (round(v, 4) if isinstance(v, float) else v)
                           for k, v in tgt.items()})

    # basket: the big tall cluster on the +y side
    tall = [p for p in props if p["ztop"] > 0.10 and p["cy"] > 0.10]
    if tall:
        bk = max(tall, key=lambda p: p["px"])
    else:
        bk = max(props, key=lambda p: p["px"])
    bx = 0.5 * (bk["xlo"] + bk["xhi"])
    by = 0.5 * (bk["ylo"] + bk["yhi"])
    api.log("BASKET px=%d x=%.3f y=%.3f ztop=%.3f" % (bk["px"], bx, by, bk["ztop"]))

    # jaws close along the tool y axis; straddle the SHORT horizontal axis
    R = R_DOWN if tgt["yspan"] <= tgt["xspan"] else R_YAW90
    api.log("R=%s spans x=%.3f y=%.3f" % ("DOWN" if R is R_DOWN else "YAW90",
                                          tgt["xspan"], tgt["yspan"]))

    gx, gy = tgt["cx"], tgt["cy"]
    api.grip(OPEN_W)
    api.log("r_hi=%.4f" % api.move([gx, gy, CARRY_Z], rotation=R, seconds=2.5))
    api.log("r_hov=%.4f" % api.move([gx, gy, HOVER_Z], rotation=R, seconds=2.0))
    api.log("eef_hov=%s" % np.round(api.eef(), 4).tolist())
    api.log("r_dn=%.4f" % api.move([gx, gy, GRASP_Z], rotation=R, seconds=2.0))
    api.log("eef_dn=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(CLOSE_W)
    api.settle(0.4)
    api.log("after_close grip=%s" % api.gripper())

    api.log("r_lift=%.4f" % api.move([gx, gy, CARRY_Z], rotation=R, seconds=2.5))
    api.log("after_lift grip=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))

    api.log("r_over=%.4f" % api.move([bx, by, CARRY_Z], rotation=R, seconds=3.0))
    api.log("r_drop=%.4f" % api.move([bx, by, RELEASE_Z], rotation=R, seconds=2.0))
    api.log("at_drop grip=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))
    api.grip(OPEN_W)
    api.settle(0.6)
    api.log("released grip=%s" % api.gripper())
    api.move([bx, by, CARRY_Z], rotation=R, seconds=2.0)
    api.settle(0.5)
    return "v1 pick-and-place"
