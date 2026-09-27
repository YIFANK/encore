"""v4 -- pick the chocolate pudding, place it in the basket.

Same mechanism as v2 (15/15 on the debug split) with the perception hardened
for layouts the debug split never shows: the target search runs on a low height
BAND first, so a box that abuts a bottle is still its own component, and a
candidate is rejected if anything tall stands over its footprint.

Identity chain (all re-derived in-cell, see PROVENANCE):
  * the two boxes on this table are the only props whose top is under 0.05 m;
    the four bottles/cartons top out at 0.113-0.148 and the basket at 0.142.
  * the mate pack, whose language names the chocolate pudding, grasps a box
    whose keyframe mean RGB is [0.245,0.206,0.194] (chroma r = 0.380) -- the
    LESS RED of the two; the other box is chroma r = 0.485.
  * the mate pack's held gripper gap, 0.0465 m, matches that box's y footprint
    (0.0475 m); the redder box is 0.0388 m across.  Debug seeds confirm: the
    grasp closes to 0.0458 m with effort 3.0.
"""
import numpy as np
from scipy import ndimage

PROVENANCE = {
    "R_DOWN": {"source": "debug seeds 51-65: api.tool_rotation() at reset is "
                         "[[.998,0,-.057],[0,-1,0],[-.057,0,-.998]]; both packs' "
                         "grasp keyframes have rpy ~ (pi, 0, ~0), straight down",
               "allowed": True},
    "Z_FLOOR": {"source": "debug seeds 51-65 cam_high deprojection: the table "
                          "surface sits at base z = 0.0011; 0.012 clears it",
                "allowed": True},
    "Z_LOW_BAND": {"source": "debug seeds 51-65: box tops measure 0.0191 and "
                             "0.0294; every other prop top is >= 0.113. 0.05 is "
                             "inside the empty gap", "allowed": True},
    "Z_TALL": {"source": "same measurement -- anything over 0.06 belongs to a "
                         "bottle, the basket or the arm", "allowed": True},
    "GRASP_BELOW_TOP": {"source": "mate pack keyframes: grasp EEF z = 0.0094, "
                                  "0.0092, 0.0114; the matching box top measures "
                                  "0.0294 on the debug seeds -> 0.019 below top",
                        "allowed": True},
    "MIN_PIX": {"source": "debug seeds 51-65: the smaller box is 487 px at 512x512; "
                          "120 keeps a half-occluded one", "allowed": True},
    "BASKET_MIN_PIX": {"source": "debug seeds 51-65: basket component is 11793 px, "
                                 "the largest prop 2839", "allowed": True},
    "CARRY_Z": {"source": "debug seeds 51-65: tallest prop top 0.148, basket rim "
                          "0.142; 0.26 clears both with the box in hand",
                "allowed": True},
    "RELEASE_Z": {"source": "both packs' release keyframes: EEF z 0.147-0.234 "
                            "over the basket", "allowed": True},
    "OPEN_W": {"source": "debug seed 51: api.gripper() at reset reads 0.0778 m",
               "allowed": True},
    "WS": {"source": "debug seeds 51-65: all props deproject inside "
                     "x(-0.22,0.18) y(-0.27,0.35); the crop is the runner's own "
                     "printed workspace, widened to keep walls out",
           "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
WS = (-0.42, 0.42, -0.44, 0.50)
Z_FLOOR = 0.012
Z_LOW_BAND = 0.050
Z_TALL = 0.060
Z_SCENE_HI = 0.32
MIN_PIX = 120
BASKET_MIN_PIX = 4000
GRASP_BELOW_TOP = 0.019
GRASP_Z_MIN, GRASP_Z_MAX = 0.008, 0.024
HOVER_Z = 0.16
CARRY_Z = 0.26
RELEASE_Z = 0.19
OPEN_W, CLOSE_W = 0.080, 0.0
PAD = 0.012          # footprint pad when testing for something tall above


def _cloud(f):
    d = np.asarray(f.depth, np.float64)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = d.shape
    vs, us = np.mgrid[0:h, 0:w]
    x = (us - K[0, 2]) * d / K[0, 0]
    y = (vs - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1)
    return (P @ T.T)[..., :3]


def _components(X, Y, Z, rgb, mask, api, tag):
    lab, n = ndimage.label(mask)
    out = []
    for i in range(1, n + 1):
        s = lab == i
        if int(s.sum()) < MIN_PIX:
            continue
        ztop = float(np.quantile(Z[s], 0.97))
        top = s & (Z > ztop - 0.006)
        if int(top.sum()) < 10:
            top = s
        c = rgb[s].mean(0)
        tot = float(c.sum()) + 1e-9
        out.append(dict(npix=int(s.sum()), ztop=ztop, mask=s,
                        x=float((X[top].min() + X[top].max()) / 2),
                        y=float((Y[top].min() + Y[top].max()) / 2),
                        xr=(float(X[s].min()), float(X[s].max())),
                        yr=(float(Y[s].min()), float(Y[s].max())),
                        xwidth=float(X[top].max() - X[top].min()),
                        ywidth=float(Y[top].max() - Y[top].min()),
                        rgb=[round(float(v), 3) for v in c],
                        chr_r=float(c[0] / tot)))
    out.sort(key=lambda d: -d["npix"])
    for d in out:
        api.log("%s%s n=%d ztop=%.4f x=%+.4f y=%+.4f w=(%.3f,%.3f) rgb=%s "
                "chr_r=%.3f" % (tag, "C", d["npix"], d["ztop"], d["x"], d["y"],
                                d["xwidth"], d["ywidth"], d["rgb"], d["chr_r"]))
    return out


def perceive(api, tag=""):
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb, np.float64) / 255.0
    B = _cloud(f)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    ws = (X > WS[0]) & (X < WS[1]) & (Y > WS[2]) & (Y < WS[3])

    full = ws & (Z > Z_FLOOR) & (Z < Z_SCENE_HI)
    comps = _components(X, Y, Z, rgb, full, api, tag)

    # primary rule: a box is a whole component whose top stays under Z_TALL.
    shorts = [c for c in comps if c["ztop"] < Z_TALL]
    if not shorts:
        # fallback for a box that fused with a taller neighbour: re-segment the
        # low band alone, and keep only candidates that are not the base of a
        # tall body (majority of their own footprint covered by tall pixels).
        api.log("%sFALLBACK low-band segmentation" % tag)
        tall = ws & (Z > Z_TALL) & (Z < Z_SCENE_HI)
        low = ws & (Z > Z_FLOOR) & (Z < Z_LOW_BAND)
        for c in _components(X, Y, Z, rgb, low, api, tag + "L"):
            over = tall & (X > c["xr"][0]) & (X < c["xr"][1]) \
                        & (Y > c["yr"][0]) & (Y < c["yr"][1])
            if int(over.sum()) < 0.5 * c["npix"]:
                shorts.append(c)

    tgt = min(shorts, key=lambda c: c["chr_r"]) if shorts else None
    if tgt is not None:
        api.log("%sTARGET x=%+.4f y=%+.4f ztop=%.4f chr_r=%.3f ywidth=%.4f"
                % (tag, tgt["x"], tgt["y"], tgt["ztop"], tgt["chr_r"],
                   tgt["ywidth"]))

    bsk = None
    best = 0
    for c in comps:
        if c["npix"] < BASKET_MIN_PIX or c["ztop"] < 0.08 or c["npix"] <= best:
            continue
        rim = c["mask"] & (Z > c["ztop"] - 0.012)
        if int(rim.sum()) < 20:
            continue
        best = c["npix"]
        bsk = dict(npix=c["npix"], ztop=c["ztop"],
                   x=float((X[rim].min() + X[rim].max()) / 2),
                   y=float((Y[rim].min() + Y[rim].max()) / 2))
    if bsk is not None:
        api.log("%sBASKET n=%d x=%+.4f y=%+.4f ztop=%.4f"
                % (tag, bsk["npix"], bsk["x"], bsk["y"], bsk["ztop"]))
    return tgt, bsk


def go(api, xyz, seconds=2.0, tag=""):
    r = api.move(list(xyz), rotation=R_DOWN, seconds=seconds)
    e = np.asarray(api.eef())
    api.log("MOVE %s -> %s resid=%.4f eef=%s"
            % (tag, [round(float(v), 4) for v in xyz], r, e.round(4).tolist()))
    return r


def run(api):
    api.log("INSTR %r" % api.instruction())
    tgt, bsk = perceive(api)
    if tgt is None or bsk is None:
        api.log("ABORT target=%s basket=%s" % (tgt is not None, bsk is not None))
        return

    gz = float(np.clip(tgt["ztop"] - GRASP_BELOW_TOP, GRASP_Z_MIN, GRASP_Z_MAX))
    api.log("PLAN grasp=(%.4f,%.4f,%.4f) basket=(%.4f,%.4f)"
            % (tgt["x"], tgt["y"], gz, bsk["x"], bsk["y"]))

    api.grip(OPEN_W)
    go(api, (tgt["x"], tgt["y"], HOVER_Z), 2.0, "hover")
    go(api, (tgt["x"], tgt["y"], gz), 2.0, "descend")
    api.grip(CLOSE_W)
    api.settle(0.4)
    api.log("GRIP_CLOSE %s" % api.gripper())

    go(api, (tgt["x"], tgt["y"], CARRY_Z), 2.0, "lift")
    g = api.gripper()
    api.log("GRIP_LIFT %s" % g)
    if g["effort"] < 1.0 or g["width_m"] < 0.010:
        # nothing in hand -- one re-perceive and one retry, then give up.
        api.log("REGRASP: empty hand")
        api.grip(OPEN_W)
        t2, _ = perceive(api, tag="R2_")
        if t2 is not None:
            gz2 = float(np.clip(t2["ztop"] - GRASP_BELOW_TOP,
                                GRASP_Z_MIN, GRASP_Z_MAX))
            go(api, (t2["x"], t2["y"], HOVER_Z), 2.0, "hover2")
            go(api, (t2["x"], t2["y"], gz2), 2.0, "descend2")
            api.grip(CLOSE_W)
            api.settle(0.4)
            go(api, (t2["x"], t2["y"], CARRY_Z), 2.0, "lift2")
            api.log("GRIP_LIFT2 %s" % api.gripper())

    go(api, (bsk["x"], bsk["y"], CARRY_Z), 2.5, "over_basket")
    go(api, (bsk["x"], bsk["y"], RELEASE_Z), 1.5, "lower")
    api.log("GRIP_PRE_RELEASE %s" % api.gripper())
    api.grip(OPEN_W)
    api.settle(0.6)
    go(api, (bsk["x"], bsk["y"], CARRY_Z), 1.5, "retreat")
    api.settle(0.4)
    perceive(api, tag="POST_")
    api.log("END")
