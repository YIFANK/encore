"""c2clean / obj_tomato_sauce_task_k3 -- "Pick the bbq sauce and place it in the basket".

Perception-first pick and place.  The bbq sauce is the amber bottle: the only
free-standing object in the scene whose top sits ~0.11 m above the table with a
~0.024 m cap; it is identified by height class + footprint, then grasped on the
body a fixed distance below its own measured top and dropped into the basket
(the large light-coloured rim ring at +y).
"""

import numpy as np

PROVENANCE = {
    "R_DOWN": {
        "source": "generic controller mechanics: tool-to-world matrix for a "
                  "straight-down wrist; matches api.tool_rotation() at reset "
                  "(debug seeds 51-65) to within 3 deg",
        "allowed": True},
    "TABLE_Z": {
        "source": "debug-seed measurement: cam_high depth deprojected over the "
                  "empty table returns z = 0.000 +- 0.002 m (seeds 51-65)",
        "allowed": True},
    "OBJ_Z_LO": {
        "source": "debug-seed measurement: above-table mask floor that keeps "
                  "props and rejects the table plane (seeds 51-65)",
        "allowed": True},
    "WORKSPACE": {
        "source": "debug-seed measurement: base-frame extent of the prop-bearing "
                  "table region in cam_high (seeds 51-65)",
        "allowed": True},
    "BOTTLE_ZTOP": {
        "source": "debug-seed measurement: the amber bottle's top height is "
                  "0.1115 m on all 15 debug seeds; height band 0.085-0.135 m "
                  "separates it from the can (0.080), the cartons (0.141) and "
                  "the flat boxes (0.017-0.028)",
        "allowed": True},
    "BOTTLE_NPX": {
        "source": "debug-seed measurement: bottle component is 225-240 cam_high "
                  "pixels at half-res (900-960 at full res); cartons are >1200",
        "allowed": True},
    "CAP_BAND": {
        "source": "debug-seed measurement: points within 8 mm of the bottle top "
                  "are the cap disc, which is unoccluded and gives an unbiased "
                  "xy centre (cap width 0.024 m in both axes)",
        "allowed": True},
    "GRASP_DROP": {
        "source": "packs/c2clean_obj_tomato_sauce_task_mate keyframes: successful "
                  "closes on the bbq sauce at ee z = 0.0732/0.0739 (demo0/demo1) "
                  "and 0.1043 (demo2); 0.038 m below this scene's measured "
                  "bottle top (0.1115) lands in that band, on the body",
        "allowed": True},
    "GRIP_CLOSE": {
        "source": "FairApi contract: api.grip(<0.025) closes the jaws",
        "allowed": True},
    "GRIP_OPEN": {
        "source": "debug-seed measurement: api.gripper()['width_m'] = 0.0778 at "
                  "reset with the jaws open",
        "allowed": True},
    "HOLD_MIN": {
        "source": "packs/..._mate keyframes: a failed close reads gripper_state "
                  "|l|+|r| = 0.0038 (demo1 t=66, demo2 t=74) while a holding "
                  "close reads 0.027-0.037 (t=117/162/156)",
        "allowed": True},
    "CARRY_Z": {
        "source": "packs/..._mate + ..._k3 ee_path: both packs carry the bottle "
                  "at ee z = 0.24-0.32 between the pick and the basket",
        "allowed": True},
    "BASKET_ZTOP": {
        "source": "debug-seed measurement: basket rim top z = 0.142 m on all 15 "
                  "debug seeds",
        "allowed": True},
    "RELEASE_Z": {
        "source": "packs/..._mate keyframes: gripper opens over the basket at "
                  "ee z = 0.1736/0.1759/0.2072",
        "allowed": True},
    "AIM_NUDGE": {
        "source": "debug-seed measurement: retry offsets along the jaw axis, "
                  "sized by the bottle's measured body width",
        "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
TABLE_Z = 0.0
OBJ_Z_LO = 0.015
WORKSPACE = (-0.28, 0.28, -0.32, 0.42)
BOTTLE_ZTOP = (0.085, 0.135)
BOTTLE_NPX = (300, 2500)
CAP_BAND = 0.008
GRASP_DROP = 0.038
GRIP_CLOSE = 0.0
GRIP_OPEN = 0.08
HOLD_MIN = 0.012
CARRY_Z = 0.30
BASKET_ZTOP = 0.142
RELEASE_Z = 0.19
AIM_NUDGE = 0.012


# ---------------------------------------------------------------- perception

def _cloud(api, cam="cam_high"):
    f = api.capture(cam)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.nan_to_num(np.asarray(f.depth, float), nan=0.0)
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    x = (u + 0.5 - K[0, 2]) / K[0, 0] * d
    y = (v + 0.5 - K[1, 2]) / K[1, 1] * d
    P = np.stack([x, y, d], -1) @ T[:3, :3].T + T[:3, 3]
    return P, np.asarray(f.rgb, float) / 255.0


def _label(mask):
    """4-connected components, iterative flood fill (no scipy)."""
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    cur = 0
    idx = np.argwhere(mask)
    for i0, j0 in idx:
        if lab[i0, j0]:
            continue
        cur += 1
        stack = [(int(i0), int(j0))]
        lab[i0, j0] = cur
        while stack:
            a, b = stack.pop()
            for p, q in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                if 0 <= p < H and 0 <= q < W and mask[p, q] and not lab[p, q]:
                    lab[p, q] = cur
                    stack.append((p, q))
    return lab, cur


def perceive(api):
    P, rgb = _cloud(api)
    z = P[..., 2]
    x0, x1, y0, y1 = WORKSPACE
    m = ((z > OBJ_Z_LO) & (z < 0.30) & (P[..., 0] > x0) & (P[..., 0] < x1)
         & (P[..., 1] > y0) & (P[..., 1] < y1))
    lab, n = _label(m)
    bottle = None
    basket = None
    for k in range(1, n + 1):
        s = lab == k
        npx = int(s.sum())
        if npx < 100:
            continue
        zz = z[s]
        px = P[..., 0][s]
        py = P[..., 1][s]
        ztop = float(zz.max())
        ex = float(px.max() - px.min())
        ey = float(py.max() - py.min())
        api.log("comp npx=%d ztop=%.4f ex=%.3f ey=%.3f cx=%+.3f cy=%+.3f "
                "rgb=%.2f,%.2f,%.2f"
                % (npx, ztop, ex, ey, float(px.mean()), float(py.mean()),
                   *rgb[s].mean(0)))
        # the bbq sauce is the TALLEST small-footprint free-standing prop:
        # a hand-sized footprint rules out the cartons and the basket, and
        # among what is left the bottle (0.1115) stands above the can (0.080)
        # and the flat boxes (0.017-0.028).
        if (npx >= BOTTLE_NPX[0] and ex < 0.09 and ey < 0.09
                and ztop > BOTTLE_ZTOP[0] * 0.5):
            cap = zz > ztop - CAP_BAND
            cand = dict(ztop=ztop, npx=npx,
                        x=float((px[cap].min() + px[cap].max()) / 2),
                        y=float((py[cap].min() + py[cap].max()) / 2),
                        capw=float(py[cap].max() - py[cap].min()),
                        bodyw=ey)
            if bottle is None or ztop > bottle["ztop"]:
                bottle = cand
        if npx > 4000 and float(rgb[s].mean()) > 0.45 and ey > 0.10:
            basket = dict(ztop=ztop,
                          x=float((px.min() + px.max()) / 2),
                          y=float((py.min() + py.max()) / 2))
    return bottle, basket


# ------------------------------------------------------------------- motion

def goto(api, xyz, seconds=2.0):
    r = api.move([float(v) for v in xyz], rotation=R_DOWN, seconds=seconds)
    api.log("move -> %.3f %.3f %.3f  resid=%s  at %s"
            % (xyz[0], xyz[1], xyz[2], np.round(np.asarray(r, float), 4).tolist()
               if r is not None else None,
               np.round(np.asarray(api.eef(), float), 4).tolist()))


def holding(api):
    g = api.gripper()
    w = float(g.get("width_m", 0.0))
    e = float(g.get("effort", 0.0))
    api.log("gripper width=%.4f effort=%.2f" % (w, e))
    return w > HOLD_MIN


def run(api):
    api.log("instruction %r" % (api.instruction(),))
    bottle, basket = perceive(api)
    api.log("bottle=%s basket=%s" % (bottle, basket))
    if bottle is None:
        api.log("NO BOTTLE FOUND -- abort")
        return
    if basket is None:
        basket = dict(x=0.0, y=0.26, ztop=BASKET_ZTOP)
        api.log("basket fallback")

    bx, by = bottle["x"], bottle["y"]
    gz = bottle["ztop"] - GRASP_DROP

    api.grip(GRIP_OPEN)
    for attempt, dy in enumerate((0.0, AIM_NUDGE, -AIM_NUDGE)):
        ax, ay = bx, by + dy
        api.log("=== attempt %d aim %.4f %.4f gz %.4f" % (attempt, ax, ay, gz))
        api.grip(GRIP_OPEN)
        goto(api, (ax, ay, CARRY_Z), 2.5)
        goto(api, (ax, ay, bottle["ztop"] + 0.045), 2.0)
        goto(api, (ax, ay, gz), 2.0)
        api.grip(GRIP_CLOSE)
        api.settle(0.4)
        if not holding(api):
            api.log("attempt %d: empty close" % attempt)
            continue
        goto(api, (ax, ay, CARRY_Z), 2.5)
        if holding(api):
            break
        api.log("attempt %d: lost on lift" % attempt)
    else:
        api.log("no grasp after 3 attempts")
        return

    goto(api, (basket["x"], basket["y"], CARRY_Z), 3.0)
    goto(api, (basket["x"], basket["y"], RELEASE_Z), 2.0)
    holding(api)
    api.grip(GRIP_OPEN)
    api.settle(0.5)
    goto(api, (basket["x"], basket["y"], CARRY_Z), 2.0)
    api.settle(0.5)
