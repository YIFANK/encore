"""v1 -- pick the bbq sauce (dark amber bottle) and place it in the basket.

Identity chain
  * the mate pack ("pick up the bbq sauce and place it in the basket") shows the
    object being lifted into the basket: a short dark-amber bottle;
  * the k1 pack ("... tomato sauce ...", my scene) shows a DIFFERENT prop (a
    squat can at (0.054,-0.107), which matches the debug-seed can cluster at
    (0.070,-0.101)) going into the basket -- so the can is NOT my object;
  * on debug seeds 51/53/55/57 exactly one tabletop cluster is a narrow, tall,
    dark-amber prop: h~0.106 m, ext ~0.026x0.048 m, top colour ~(66,21,4).
"""
import numpy as np

PROVENANCE = {
    "WS_X": {"source": "debug-seed 51/53/55/57 observation: crop that contains every prop cluster", "allowed": True},
    "WS_Y": {"source": "debug-seed 51/53/55/57 observation: crop that contains every prop cluster", "allowed": True},
    "OBJ_CUT": {"source": "debug-seed observation: table plane mode at z=0.0069, props start ~0.015 above it", "allowed": True},
    "CEIL_Z": {"source": "debug-seed observation: tallest tabletop cluster top=0.144; arm hardware sits above 0.20", "allowed": True},
    "BASKET_MIN_EXT": {"source": "debug-seed observation: basket ext ~0.16x0.17 m vs every prop <=0.09 m", "allowed": True},
    "CAND_MAX_EXT": {"source": "debug-seed observation: bbq-sauce cluster ext 0.024-0.028 x 0.047-0.049 m", "allowed": True},
    "CAND_H": {"source": "debug-seed observation: bbq-sauce h=0.106 m; can 0.074, boxes 0.136, flat prop 0.022", "allowed": True},
    "GRASP_DROP": {"source": "mate pack keyframe t=54: close at ee z=0.0797 with the same bottle asset whose measured top is 0.113 -> 0.033 below the top", "allowed": True},
    "RELEASE_RISE": {"source": "k1 pack keyframe t=112: release at ee z=0.1808 over a basket whose measured top is 0.144 -> 0.037 above the rim", "allowed": True},
    "DROP_CLEAR": {"source": "v1 debug receipt: payload hang 0.079 m, descent to rim+0.037 blocked 0.027 m short on 8/8 seeds -> release with the base just clear of the rim instead", "allowed": True},
    "CARRY_Z": {"source": "k1/mate pack ee_path6 transport altitude (0.23-0.32 m)", "allowed": True},
    "HOVER_Z": {"source": "debug-seed observation: tallest prop top 0.144; hover must clear it", "allowed": True},
    "GRIP_HELD_EFFORT": {"source": "FairApi contract: gripper() effort is 3.0 iff holding", "allowed": True},
}

WS_X = (-0.45, 0.45)
WS_Y = (-0.50, 0.50)
OBJ_CUT = 0.015
CEIL_Z = 0.20
BASKET_MIN_EXT = 0.12
CAND_MAX_EXT = 0.09
CAND_H = (0.07, 0.13)
GRASP_DROP = 0.033
RELEASE_RISE = 0.040
DROP_CLEAR = 0.012
CARRY_Z = 0.27
HOVER_Z = 0.21


# ---------------------------------------------------------------- perception
def _cloud(fr):
    d = np.asarray(fr.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(fr.intrinsics, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    v, u = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 0)
    z = np.where(ok, d, 1.0)
    pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z, np.ones_like(z)], -1)
    P = pc @ np.asarray(fr.t_base_cam, float).T
    return P[..., :3], ok


def _components(mask):
    """4-connected labelling of a boolean image, without scipy."""
    lab = np.zeros(mask.shape, np.int32)
    nxt = 0
    for (r, c) in np.argwhere(mask):
        if lab[r, c]:
            continue
        nxt += 1
        lab[r, c] = nxt
        stack = [(r, c)]
        while stack:
            a, b = stack.pop()
            for aa, bb in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                if 0 <= aa < mask.shape[0] and 0 <= bb < mask.shape[1] \
                        and mask[aa, bb] and not lab[aa, bb]:
                    lab[aa, bb] = nxt
                    stack.append((aa, bb))
    return lab, nxt


def scene(api):
    fr = api.capture("cam_high")
    P, ok = _cloud(fr)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    inbox = ok & (X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1])
    hist, edges = np.histogram(Z[inbox], bins=40)
    table_z = float(edges[int(np.argmax(hist))] + 0.5 * (edges[1] - edges[0]))
    # The ceiling drops the arm: nothing resting on this table reaches 0.20 m.
    mask = inbox & (Z > table_z + OBJ_CUT) & (Z < CEIL_Z)
    lab, n = _components(mask)
    out = []
    for i in range(1, n + 1):
        sel = lab == i
        cnt = int(sel.sum())
        if cnt < 60:
            continue
        x, y, z = X[sel], Y[sel], Z[sel]
        top = float(z.max())
        trgb = fr.rgb[sel & (Z > top - 0.02)].mean(0)
        out.append(dict(n=cnt, cx=float(np.median(x)), cy=float(np.median(y)),
                        top=top, h=top - table_z,
                        ex=float(x.max() - x.min()), ey=float(y.max() - y.min()),
                        trgb=[float(v) for v in trgb]))
    out.sort(key=lambda r: -r["n"])
    return table_z, out


def pick_targets(api, table_z, cl):
    baskets = [c for c in cl if max(c["ex"], c["ey"]) > BASKET_MIN_EXT]
    basket = max(baskets, key=lambda c: c["n"]) if baskets else None
    cands = [c for c in cl if c is not basket
             and max(c["ex"], c["ey"]) <= CAND_MAX_EXT
             and CAND_H[0] <= c["h"] <= CAND_H[1]]
    # The bbq sauce is the dark-amber one: every other prop's top is far
    # brighter (>=200 summed) and far less red-dominant.
    obj = min(cands, key=lambda c: sum(c["trgb"])) if cands else None
    return basket, obj


# -------------------------------------------------------------------- motion
def held(api):
    g = api.gripper()
    return g["effort"] > 1.0 and g["width_m"] > 0.005, g


def run(api):
    api.log("instruction=%r" % api.instruction())
    table_z, cl = scene(api)
    api.log("table_z=%.4f nclusters=%d" % (table_z, len(cl)))
    for c in cl:
        api.log("CL n=%d c=(%.3f,%.3f) top=%.3f h=%.3f ext=(%.3f,%.3f) trgb=%s"
                % (c["n"], c["cx"], c["cy"], c["top"], c["h"], c["ex"], c["ey"],
                   [round(v) for v in c["trgb"]]))
    basket, obj = pick_targets(api, table_z, cl)
    if basket is None or obj is None:
        api.log("ABORT basket=%s obj=%s" % (basket is not None, obj is not None))
        return
    api.log("BASKET c=(%.3f,%.3f) top=%.3f | OBJ c=(%.3f,%.3f) top=%.3f h=%.3f trgb=%s"
            % (basket["cx"], basket["cy"], basket["top"],
               obj["cx"], obj["cy"], obj["top"], obj["h"],
               [round(v) for v in obj["trgb"]]))

    ox, oy = obj["cx"], obj["cy"]
    grasp_z = obj["top"] - GRASP_DROP

    api.grip(0.08)
    r = api.move([ox, oy, HOVER_Z], seconds=2.0)
    api.log("hover residual=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    r = api.move([ox, oy, grasp_z], seconds=2.0)
    api.log("descend to %.4f residual=%.4f eef=%s"
            % (grasp_z, r, np.round(api.eef(), 4).tolist()))
    api.grip(0.0)
    api.settle(0.3)
    ok, g = held(api)
    api.log("close1 held=%s g=%s" % (ok, g))

    if not ok:
        # Second attempt, one finger-width lower: a bottle that slipped the
        # shoulder is still catchable on the body.
        api.grip(0.08)
        z2 = obj["top"] - GRASP_DROP - 0.020
        r = api.move([ox, oy, z2], seconds=1.5)
        api.grip(0.0)
        api.settle(0.3)
        ok, g = held(api)
        api.log("close2 z=%.4f residual=%.4f held=%s g=%s" % (z2, r, ok, g))

    # How far the payload hangs below the tool: its base was on the table at
    # the moment the jaws closed (v1 receipt: 0.079 m, and that length is what
    # caught the basket rim on the way down).
    hang = float(api.eef()[2]) - table_z
    r = api.move([ox, oy, CARRY_Z], seconds=2.0)
    api.log("lift residual=%.4f eef=%s held=%s hang=%.4f"
            % (r, np.round(api.eef(), 4).tolist(), held(api)[0], hang))
    api.settle(0.4)

    bx, by = basket["cx"], basket["cy"]
    r = api.move([bx, by, CARRY_Z], seconds=3.0)
    api.settle(0.6)                      # let the payload stop swinging
    api.log("transport residual=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    # Release from just clear of the rim instead of reaching inside: a tilted
    # payload that is lowered past the rim catches it (v1: descent blocked
    # 27 mm short on 8/8 seeds, payload toppled out).
    rel_z = min(CARRY_Z, basket["top"] + hang + DROP_CLEAR)
    r = api.move([bx, by, rel_z], seconds=2.5)
    api.log("over-basket z=%.4f residual=%.4f eef=%s held=%s"
            % (rel_z, r, np.round(api.eef(), 4).tolist(), held(api)[0]))
    api.settle(0.3)
    api.grip(0.08)
    api.settle(1.0)
    api.move([bx, by, CARRY_Z], seconds=2.0)
    api.settle(0.5)

    # post-hoc receipt: where did the dark prop end up?
    tz2, cl2 = scene(api)
    b2, o2 = pick_targets(api, tz2, cl2)
    if o2 is not None:
        api.log("AFTER obj c=(%.3f,%.3f) top=%.3f h=%.3f" % (o2["cx"], o2["cy"], o2["top"], o2["h"]))
    else:
        api.log("AFTER obj not found among %d clusters" % len(cl2))
    for c in cl2:
        api.log("CL2 n=%d c=(%.3f,%.3f) top=%.3f ext=(%.3f,%.3f) trgb=%s"
                % (c["n"], c["cx"], c["cy"], c["top"], c["ex"], c["ey"],
                   [round(v) for v in c["trgb"]]))
