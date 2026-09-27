"""c2clean obj_bbq_sauce_task_k0 -- v6: perceive -> grasp ketchup -> place in basket.

Intent: "Pick the ketchup and place it in the basket".

The table carries two reddish bottles (debug seeds 51-57): a shorter BBQ sauce
bottle and a taller one whose label reads "Tomato Ketchup" in the debug RGB.
v4 placed the BBQ bottle in the basket cleanly (grasp effort 3.0, object visibly
inside at the end of every GIF) and scored 0/4, so the graded object is the one
the instruction actually names: the ketchup.  Selector = among clusters scoring
mean R - mean B > 12 (only the two bottles do), take the taller.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-57 (v1/v2): cam_high deprojection of bare table -> z = 0.0015", "allowed": True},
    "Z_BAND": {"source": "debug-seed measurement: props segment cleanly at table_z + 0.012", "allowed": True},
    "MIN_PIX/MAX_PIX": {"source": "debug seeds 51/53/55/57 cluster sizes: target 1343-1367 px, basket ~11700 px, robot ~800 px", "allowed": True},
    "RED_MIN": {"source": "debug seeds 51/53/55/57 cluster mean RGB: the two bottles score R-B = 51 and 27, every other cluster < 6", "allowed": True},
    "TARGET_RULE": {"source": "debug seeds 51-57 RGB: the taller reddish bottle (ztop 0.148 vs 0.113) carries the legible 'Tomato Ketchup' label named by the instruction; v4 receipt: placing the shorter reddish bottle (BBQ) in the basket scored 0/4", "allowed": True},
    "GRASP_Z": {"source": "debug seed 51 height profile of the ketchup cluster: y-width 0.063 at the base tapering to 0.034 at the cap; at z=0.100 the width is 0.042 and the body widens below, blocking downward slip", "allowed": True},
    "HOVER_Z": {"source": "debug seeds 51-57: tallest prop ztop 0.148, basket rim 0.142; transiting at 0.29 keeps the carried base 0.19 clear of the rim", "allowed": True},
    "DROP_Z": {"source": "debug seeds 51-57: basket rim ztop = 0.142; gripping 0.100 up the bottle, an eef at 0.22 puts the held base at 0.12, inside the rim", "allowed": True},
    "TIP_OFFSET": {"source": "debug seed 51 (v3): open gripper stalled on the bare table at eef z = 0.0089", "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics; matches the home tool_rotation logged in v1", "allowed": True},
    "RETRY_DROP": {"source": "debug seed 51 height profile: 0.014 steps the jaws down one band of the ketchup taper (width 0.042 -> 0.054)", "allowed": True},
    "HOLD_EFFORT/EMPTY_W": {"source": "debug seed 51 (v3): free-air closed width 0.0010 at effort 0.05; v5 holding the ketchup reads width 0.0334 at effort 3.0", "allowed": True},
    "POS_TOL": {"source": "debug seed 51 (v3): api.move residual floors near 0.010 at convergence; eef repeats to ~5 mm", "allowed": True},
}

TABLE_Z = 0.0015
Z_BAND = 0.012
MIN_PIX, MAX_PIX = 500, 4000
RED_MIN = 12.0
GRASP_Z = 0.100
HOVER_Z = 0.29
DROP_Z = 0.22
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
POS_TOL = 0.004
RETRY_DROP = 0.014
HOLD_EFFORT = 1.0
EMPTY_W = 0.006


def _cloud(fr):
    d = np.asarray(fr.depth, dtype=np.float64)
    if d.ndim == 3:
        d = d[..., 0]
    H, W = d.shape
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    vv, uu = np.mgrid[0:H, 0:W]
    cam = np.stack([(uu - K[0, 2]) / K[0, 0] * d, (vv - K[1, 2]) / K[1, 1] * d, d], axis=-1)
    return ((T[:3, :3] @ cam.reshape(-1, 3).T).T + T[:3, 3]).reshape(H, W, 3)


def _label(mask):
    """4-connected labelling (scipy is not relied on inside the sandbox)."""
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    cur = 0
    for v0, u0 in np.argwhere(mask):
        if lab[v0, u0]:
            continue
        cur += 1
        stack = [(v0, u0)]
        lab[v0, u0] = cur
        while stack:
            v, u = stack.pop()
            for a, b in ((v + 1, u), (v - 1, u), (v, u + 1), (v, u - 1)):
                if 0 <= a < H and 0 <= b < W and mask[a, b] and not lab[a, b]:
                    lab[a, b] = cur
                    stack.append((a, b))
    return lab, cur


def _scene(api):
    fr = api.capture("cam_high")
    rgb = np.asarray(fr.rgb).astype(np.float64)
    P = _cloud(fr)
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    m = (z > TABLE_Z + Z_BAND) & (z < 0.30) & (x > -0.44) & (x < 0.44) & (y > -0.44) & (y < 0.54)
    lab, n = _label(m)
    props, basket = [], None
    for c in range(1, n + 1):
        sel = lab == c
        k = int(sel.sum())
        if k < 200:
            continue
        col = rgb[sel].mean(axis=0)
        d = dict(n=k, col=col, zt=float(np.percentile(z[sel], 99)),
                 ymid=float((y[sel].min() + y[sel].max()) / 2.0),
                 ywid=float(y[sel].max() - y[sel].min()),
                 xmax=float(x[sel].max()), xmin=float(x[sel].min()),
                 xmid=float((x[sel].min() + x[sel].max()) / 2.0))
        api.log("CL n=%d rgb=%s zt=%.4f y=[%.4f w %.4f] x=[%.4f,%.4f]" % (
            k, np.round(col, 1).tolist(), d["zt"], d["ymid"], d["ywid"], d["xmin"], d["xmax"]))
        if k > 6000 and d["zt"] > 0.10:
            if basket is None or k > basket["n"]:
                basket = d
        if MIN_PIX <= k <= MAX_PIX and d["zt"] < 0.22:
            props.append(d)
    return props, basket


def _pick_target(api, props):
    """Two reddish bottles stand on the table (debug seeds 51-57): the BBQ sauce
    (mean R-B = 51, ztop 0.113) and the Tomato Ketchup bottle whose label is
    legible in the debug RGB (mean R-B = 27, ztop 0.148).  Everything else scores
    R-B < 6.  The instruction names the ketchup, so among the reddish clusters
    take the taller one."""
    red = []
    for d in props:
        sc = d["col"][0] - d["col"][2]
        api.log("cand n=%d score=%.1f zt=%.4f ymid=%.4f" % (d["n"], sc, d["zt"], d["ymid"]))
        if sc > RED_MIN:
            red.append((sc, d))
    if not red:
        return None
    red.sort(key=lambda t: -t[1]["zt"])
    return red[0]


def _goto(api, xyz, seconds=2.5, tries=3):
    xyz = np.asarray(xyz, float)
    cmd = xyz.copy()
    for i in range(tries):
        r = api.move(cmd.tolist(), rotation=R_DOWN, seconds=seconds)
        e = np.asarray(api.eef(), float)
        err = xyz - e
        api.log("goto want=%s got=%s err=%s r=%.4f" % (
            np.round(xyz, 4).tolist(), np.round(e, 4).tolist(), np.round(err, 4).tolist(), r))
        if np.max(np.abs(err)) < POS_TOL:
            break
        cmd = cmd + err
    return np.asarray(api.eef(), float)


def run(api):
    api.log("INSTR %r" % (api.instruction(),))
    props, basket = _scene(api)
    if basket is None:
        api.log("NO BASKET"); return
    sel = _pick_target(api, props)
    if sel is None:
        api.log("NO TARGET"); return
    sc, tgt = sel
    r = tgt["ywid"] / 2.0
    tx = tgt["xmax"] - r
    ty = tgt["ymid"]
    api.log("TARGET score=%.1f xy=(%.4f,%.4f) r=%.4f ztop=%.4f" % (sc, tx, ty, r, tgt["zt"]))
    bx, by = basket["xmid"], basket["ymid"]
    api.log("BASKET xy=(%.4f,%.4f) rim=%.4f" % (bx, by, basket["zt"]))

    api.grip(0.08)
    api.settle(0.2)
    _goto(api, [tx, ty, HOVER_Z])

    # top-down refinement from the wrist camera (no grazing-view bias)
    fr = api.capture("cam_arm_wrist")
    P = _cloud(fr)
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    near = (np.abs(x - tx) < 0.06) & (np.abs(y - ty) < 0.06) & (z > TABLE_Z + 0.03) & (z < 0.25)
    if near.sum() > 150:
        rx = float((x[near].min() + x[near].max()) / 2.0)
        ry = float((y[near].min() + y[near].max()) / 2.0)
        zt = float(np.percentile(z[near], 99))
        api.log("WRIST refine n=%d xy=(%.4f,%.4f) dx=%.4f dy=%.4f zt=%.4f" % (
            near.sum(), rx, ry, rx - tx, ry - ty, zt))
        tx, ty = rx, ry
    else:
        api.log("WRIST refine skipped n=%d" % (near.sum(),))

    held = False
    for attempt in range(3):
        gz = GRASP_Z - attempt * RETRY_DROP
        api.grip(0.08)
        api.settle(0.2)
        _goto(api, [tx, ty, gz + 0.06])
        _goto(api, [tx, ty, gz], seconds=2.0)
        api.grip(0.0)
        api.settle(0.6)
        g = api.gripper()
        api.log("CLOSED try=%d gz=%.3f %r" % (attempt, gz, g))
        if g["effort"] < HOLD_EFFORT or g["width_m"] < EMPTY_W:
            api.log("no grip -> retry")
            continue
        _goto(api, [tx, ty, HOVER_Z], seconds=3.0)
        g2 = api.gripper()
        api.log("LIFTED try=%d %r eef=%s" % (attempt, g2, np.round(np.asarray(api.eef(), float), 4).tolist()))
        if g2["effort"] >= HOLD_EFFORT and g2["width_m"] > EMPTY_W:
            held = True
            break
        # the object was dropped on the way up; re-perceive before trying again
        api.log("lost on lift -> re-perceive")
        props2, basket2 = _scene(api)
        sel2 = _pick_target(api, props2)
        if sel2 is not None:
            t2 = sel2[1]
            tx, ty = t2["xmax"] - t2["ywid"] / 2.0, t2["ymid"]
            api.log("re-aim xy=(%.4f,%.4f)" % (tx, ty))
    if not held:
        api.log("GRASP FAILED after retries")
        return

    _goto(api, [bx, by, HOVER_Z], seconds=3.0)
    api.log("OVER BASKET %r" % (api.gripper(),))
    _goto(api, [bx, by, DROP_Z], seconds=2.0)
    for _ in range(3):
        api.grip(0.08)
        api.settle(0.6)
        g = api.gripper()
        api.log("RELEASE %r" % (g,))
        if g["effort"] < HOLD_EFFORT:
            break
    _goto(api, [bx, by, HOVER_Z], seconds=2.0)
    api.settle(0.5)
    api.log("DONE v6")
