"""c2 obj_bbq_sauce_stock -- v1.1: perceive the bbq-sauce bottle, top-down grasp,
carry to the basket, release.

v1.1 de-oracle pass (mechanical, no tuning): v6 body verbatim except that the
single read of the episode-terminated flag -- which appeared only inside the
`goto()` log format string, with no control flow attached to it -- is removed
(this program now never reads that flag at all). Nothing in this
program ever branched on runtime success: the one retry gate (post-lift regrasp)
is conditioned on `api.gripper()["effort"]`, an own-sensor reading, and the
remaining early returns are perception failures inside the estimator helpers.
No constant, threshold or motion changed.

Mechanism (all of it re-derived from packs/c2_obj_bbq_sauce_stock + debug seeds
51/53/55, see PROVENANCE):
  * The intent object is the only dark red-brown item in the scene. A simple
    chromatic mask (R clearly above G and B, and dark) isolates it; the largest
    connected component of that mask is the bottle.
  * Deprojecting the component gives its silhouette in the base frame. The
    horizontal cap face (the pixels within ~1 cm of the component's highest
    deprojected z) is a horizontal surface, so its (min+max)/2 in x and y is
    the bottle axis; the body band gives an independent y centre.
  * The wrist camera, parked directly above that estimate, sees the cap almost
    straight down and refines the axis.
  * Grasp height comes from the demos: they close the gripper at eef
    z ~ 0.078 (bottle top measured at z ~ 0.113, table at z ~ 0.001).
"""
import numpy as np

PROVENANCE = {
    "RED_DB": {"source": "debug seeds 51/53/55 cam_high RGB: bbq bottle "
                         "mean (55,30,12) vs distractors (green bottle 81/87/75, "
                         "amber-label bottle 71/59/45, blue can 32/44/72, "
                         "table 173/164/156)", "allowed": True},
    "RED_DG": {"source": "same debug-seed colour survey", "allowed": True},
    "RED_RMAX": {"source": "same debug-seed colour survey", "allowed": True},
    "RED_GMAX": {"source": "same debug-seed colour survey", "allowed": True},
    "MIN_BLOB": {"source": "debug seeds 51/53/55: bbq component 75-81 px at "
                           "1/4 resolution (~1200 px at 512), distractors <=25",
                 "allowed": True},
    "Z_TABLE": {"source": "debug seed 51 cam_high: deprojected table plane "
                          "z median 0.0011 m", "allowed": True},
    "Z_GRASP": {"source": "pack.json ee_path6 gripper-close transitions "
                          "(demo0 z=0.0797, demo1 z=0.0784); bottle top measured "
                          "at z=0.113 on debug seeds", "allowed": True},
    "Z_HOVER": {"source": "clear of the 0.113 m bottle top measured on debug "
                          "seeds; generic approach margin", "allowed": True},
    "Z_LIFT": {"source": "pack.json ee_path6 carry apex (demo0 z=0.316, "
                         "demo2 z=0.298), reduced to a clearing height",
               "allowed": True},
    "Z_RELEASE": {"source": "pack.json ee_path6 gripper-open over basket "
                            "(demo0 0.1742, demo1 0.1990, demo2 0.2119)",
                  "allowed": True},
    "BASKET_FALLBACK": {"source": "pack.json ee_path6 release points "
                                  "(-0.007,0.249), (-0.007,0.258), (0.027,0.237)",
                        "allowed": True},
    "BASKET_GREY": {"source": "debug seeds 51/53/55: basket pixels are neutral "
                              "(|R-G|<14,|G-B|<18) at 70-165 vs warm table "
                              "173/164/156", "allowed": True},
    "RIM_Z_LO": {"source": "debug seeds 51-65 v2 log: basket rim height "
                           "measured 0.136-0.138 m", "allowed": True},
    "RIM_Z_HI": {"source": "debug seeds 51-65 v2 log: basket rim height "
                           "measured 0.136-0.138 m", "allowed": True},
    "BASKET_CLAMP": {"source": "spread of the basket centre over debug seeds "
                               "51/53/55 (<0.02 m) plus the demo release spread "
                               "in pack.json (0.03 m)", "allowed": True},
    "RIM_TRIM_R": {"source": "debug seeds 51-65: basket mouth measured ~0.15 x "
                             "0.17 m, so its rim lies within ~0.11 m of the rim "
                             "point median, while the contaminating white label "
                             "sits ~0.17 m away", "allowed": True},
    "X_THICK_HALF": {"source": "debug seeds 51/53/55: bottle front face "
                               "deprojects to x~0.065 while the cap face centre "
                               "is ~0.012 m behind it", "allowed": True},
    "OBJ_TOP_LO": {"source": "debug seeds 51-65: bbq bottle top face "
                             "deprojects to z=0.1117-0.1128 m", "allowed": True},
    "OBJ_TOP_HI": {"source": "debug seeds 51-65: bbq bottle top face "
                             "deprojects to z=0.1117-0.1128 m", "allowed": True},
    "PRIOR_GRASP_XY": {"source": "mean perceived bottle axis over debug seeds "
                                 "51,53,55,57,59,61,63,65 (x 0.043-0.053, "
                                 "y -0.088..-0.103); agrees with the pack demo "
                                 "close points (0.033,-0.121),(0.034,-0.111),"
                                 "(0.056,-0.115)", "allowed": True},
    "GRIP_OPEN_M": {"source": "api.gripper() at reset on debug seed 51: "
                              "width 0.0778 m", "allowed": True},
    "MAX_REFINE_JUMP": {"source": "debug-seed spread of the cam_high estimate "
                                  "(<0.01 m across seeds 51/53/55); anything "
                                  "larger is a mis-detection", "allowed": True},
}

RED_DB, RED_DG, RED_RMAX, RED_GMAX = 25, 15, 150, 100
MIN_BLOB = 200
Z_TABLE = 0.001
Z_GRASP = 0.078
Z_HOVER = 0.250
Z_LIFT = 0.260
Z_RELEASE = 0.185
BASKET_FALLBACK = (0.004, 0.248)
X_THICK_HALF = 0.012
GRIP_OPEN_M = 0.078
MAX_REFINE_JUMP = 0.045
RIM_Z_LO, RIM_Z_HI = 0.100, 0.165
BASKET_CLAMP = 0.040
RIM_TRIM_R = 0.110
OBJ_TOP_LO, OBJ_TOP_HI = 0.050, 0.200
PRIOR_GRASP_XY = (0.048, -0.097)

NEIGH = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def red_mask(rgb):
    a = rgb.astype(np.int16)
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    return (r > b + RED_DB) & (r > g + RED_DG) & (r < RED_RMAX) & (g < RED_GMAX)


def largest_component(mask):
    h, w = mask.shape
    seen = np.zeros(mask.shape, bool)
    best = []
    vs, us = np.nonzero(mask)
    for v0, u0 in zip(vs, us):
        if seen[v0, u0]:
            continue
        stack = [(int(v0), int(u0))]
        seen[v0, u0] = True
        comp = []
        while stack:
            a, b = stack.pop()
            comp.append((a, b))
            for da, db in NEIGH:
                na, nb = a + da, b + db
                if 0 <= na < h and 0 <= nb < w and mask[na, nb] and not seen[na, nb]:
                    seen[na, nb] = True
                    stack.append((na, nb))
        if len(comp) > len(best):
            best = comp
    return best


def cloud(frame, comp):
    pts = []
    for v, u in comp:
        p = frame.deproject(u, v)
        if p is not None:
            pts.append(p)
    return np.asarray(pts, float) if pts else np.zeros((0, 3))


def axis_from_cloud(pts, tag, api):
    """Return (x, y, z_top, n_top) for a vertical object from its visible cloud."""
    if len(pts) < 10:
        return None
    z_top = float(np.percentile(pts[:, 2], 97))
    top = pts[pts[:, 2] > z_top - 0.012]
    body = pts[(pts[:, 2] > Z_TABLE + 0.015) & (pts[:, 2] < z_top - 0.025)]
    x_cap = y_cap = None
    if len(top) >= 6:
        x_cap = float(top[:, 0].min() + top[:, 0].max()) / 2.0
        y_cap = float(top[:, 1].min() + top[:, 1].max()) / 2.0
    y_body = x_near = None
    if len(body) >= 10:
        y_body = float(body[:, 1].min() + body[:, 1].max()) / 2.0
        x_near = float(np.percentile(body[:, 0], 98))
    api.log("%s cloud n=%d z_top=%.4f n_top=%d n_body=%d x_cap=%s y_cap=%s "
            "y_body=%s x_near=%s y[%.4f,%.4f]"
            % (tag, len(pts), z_top, len(top), len(body),
               None if x_cap is None else round(x_cap, 4),
               None if y_cap is None else round(y_cap, 4),
               None if y_body is None else round(y_body, 4),
               None if x_near is None else round(x_near, 4),
               pts[:, 1].min(), pts[:, 1].max()))
    x = x_cap if x_cap is not None else (
        (x_near - X_THICK_HALF) if x_near is not None else None)
    y = y_body if y_body is not None else y_cap
    if x is None or y is None:
        return None
    return float(x), float(y), z_top, len(top)


def find_basket(frame, api):
    """Basket centre from the rim band, spatially trimmed.

    v2 used the bbox of every neutral-grey pixel above z=0.05 and v3 restricted
    that to the rim height band. Both stayed contaminated: the debug logs show
    grey points out at x=-0.162, y=0.050, which is the WHITE LABEL of the other
    (amber) bottle, not the basket. Its bbox corner dragged the drop point up to
    0.08 m off the basket mouth. The basket rim is by far the densest cluster of
    rim-height grey points, so trimming to the points near their median rejects
    the label and leaves a bbox that is the mouth itself.
    """
    a = frame.rgb.astype(np.int16)
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    m = (np.abs(r - g) < 14) & (np.abs(g - b) < 18) & (r > 70) & (r < 165)
    m[:180, :] = False
    vs, us = np.nonzero(m)
    pts = []
    for v, u in zip(vs[::2], us[::2]):
        p = frame.deproject(int(u), int(v))
        if p is None:
            continue
        if (RIM_Z_LO < p[2] < RIM_Z_HI and 0.03 < p[1] < 0.45
                and -0.25 < p[0] < 0.25):
            pts.append(p)
    if len(pts) < 40:
        api.log("basket: only %d rim pts -> fallback %s" % (len(pts), BASKET_FALLBACK))
        return BASKET_FALLBACK
    pts = np.asarray(pts)
    mx, my = float(np.median(pts[:, 0])), float(np.median(pts[:, 1]))
    keep = pts[np.hypot(pts[:, 0] - mx, pts[:, 1] - my) < RIM_TRIM_R]
    api.log("basket rim n=%d med=(%.4f,%.4f) kept=%d raw_x[%.3f,%.3f] raw_y[%.3f,%.3f]"
            % (len(pts), mx, my, len(keep), pts[:, 0].min(), pts[:, 0].max(),
               pts[:, 1].min(), pts[:, 1].max()))
    if len(keep) < 40:
        api.log("basket: trimmed to %d pts -> fallback" % len(keep))
        return BASKET_FALLBACK
    cx = float(keep[:, 0].min() + keep[:, 0].max()) / 2.0
    cy = float(keep[:, 1].min() + keep[:, 1].max()) / 2.0
    api.log("basket trimmed centre=(%.4f,%.4f) x[%.3f,%.3f] y[%.3f,%.3f]"
            % (cx, cy, keep[:, 0].min(), keep[:, 0].max(),
               keep[:, 1].min(), keep[:, 1].max()))
    cx = float(np.clip(cx, BASKET_FALLBACK[0] - BASKET_CLAMP,
                       BASKET_FALLBACK[0] + BASKET_CLAMP))
    cy = float(np.clip(cy, BASKET_FALLBACK[1] - BASKET_CLAMP,
                       BASKET_FALLBACK[1] + BASKET_CLAMP))
    api.log("basket target=(%.4f,%.4f)" % (cx, cy))
    return (cx, cy)


def perceive(api, camera, tag):
    """Locate the bbq bottle: the largest dark-red connected component.

    v5 tried ranking several components and rejecting any whose horizontal span
    exceeded 0.12 m. That span is meaningless: a component's border pixels
    deproject to the TABLE behind the object, so the true bottle measured a
    0.19 m span and was rejected, and the amber-label bottle was grasped
    instead -- 0/8 on debug seeds 51-65, which also proves the benchmark bit is
    specific to the bbq sauce and not to "some bottle in the basket". Size is
    the discriminator that actually separates them (1249 px vs 694 px on debug
    seed 51), so this returns to the largest component and only checks that its
    top face is at a plausible height.
    """
    f = api.capture(camera)
    m = red_mask(f.rgb)
    comp = largest_component(m)
    api.log("%s mask_px=%d comp_px=%d" % (tag, int(m.sum()), len(comp)))
    if len(comp) < MIN_BLOB:
        return None, f
    est = axis_from_cloud(cloud(f, comp), tag, api)
    if est is None:
        return None, f
    if not (OBJ_TOP_LO < est[2] < OBJ_TOP_HI):
        api.log("%s rejected: z_top=%.4f outside [%.2f,%.2f]"
                % (tag, est[2], OBJ_TOP_LO, OBJ_TOP_HI))
        return None, f
    return est, f


def goto(api, xyz, seconds=1.2, tag=""):
    res = api.move(xyz, seconds=seconds)
    e = api.eef()
    api.log("move%s -> %s got %s res=%.4f"
            % (tag, np.round(xyz, 4).tolist(), np.round(e, 4).tolist(), res))
    return e


def run(api):
    api.log("instruction=%r" % api.instruction())
    api.grip(GRIP_OPEN_M)

    est, f_high = perceive(api, "cam_high", "high")
    basket = find_basket(f_high, api)
    if est is None:
        api.log("cam_high found nothing plausible -> prior grasp point %s"
                % (PRIOR_GRASP_XY,))
        gx, gy, z_top = PRIOR_GRASP_XY[0], PRIOR_GRASP_XY[1], 0.113
    else:
        gx, gy, z_top, _ = est
    api.log("high estimate x=%.4f y=%.4f z_top=%.4f" % (gx, gy, z_top))

    goto(api, [gx, gy, Z_HOVER], seconds=1.5, tag="/hover")

    west, _ = perceive(api, "cam_arm_wrist", "wrist")
    if west is not None:
        wx, wy, wz, ntop = west
        jump = float(np.hypot(wx - gx, wy - gy))
        api.log("wrist estimate x=%.4f y=%.4f z_top=%.4f jump=%.4f" % (wx, wy, wz, jump))
        if jump < MAX_REFINE_JUMP:
            gx, gy = wx, wy
        else:
            api.log("wrist refinement rejected (jump %.4f)" % jump)

    api.log("GRASP TARGET x=%.4f y=%.4f z=%.4f" % (gx, gy, Z_GRASP))
    goto(api, [gx, gy, Z_HOVER], seconds=0.8, tag="/align")
    goto(api, [gx, gy, 0.150], seconds=1.0, tag="/pre")
    goto(api, [gx, gy, Z_GRASP], seconds=1.0, tag="/down")
    api.grip(0.0)
    api.settle(0.3)
    g = api.gripper()
    api.log("after close: %s" % g)

    goto(api, [gx, gy, Z_LIFT], seconds=1.2, tag="/lift")
    g = api.gripper()
    api.log("after lift: %s eef=%s" % (g, np.round(api.eef(), 4).tolist()))

    if g["effort"] < 1.0:
        api.log("REGRASP: nothing held after lift")
        api.grip(GRIP_OPEN_M)
        est2, _ = perceive(api, "cam_high", "high2")
        if est2 is not None:
            gx, gy = est2[0], est2[1]
            api.log("regrasp target x=%.4f y=%.4f" % (gx, gy))
        goto(api, [gx, gy, 0.150], seconds=1.0, tag="/pre2")
        goto(api, [gx, gy, Z_GRASP - 0.006], seconds=1.0, tag="/down2")
        api.grip(0.0)
        api.settle(0.3)
        api.log("after close2: %s" % api.gripper())
        goto(api, [gx, gy, Z_LIFT], seconds=1.2, tag="/lift2")
        api.log("after lift2: %s" % api.gripper())

    goto(api, [basket[0], basket[1], Z_LIFT], seconds=1.5, tag="/over")
    api.log("over basket gripper=%s" % api.gripper())
    goto(api, [basket[0], basket[1], Z_RELEASE], seconds=1.0, tag="/drop")
    api.grip(GRIP_OPEN_M)
    api.settle(0.5)
    api.log("released, eef=%s gripper=%s"
            % (np.round(api.eef(), 4).tolist(), api.gripper()))
    goto(api, [basket[0], basket[1], 0.28], seconds=1.0, tag="/retreat")
    api.settle(0.5)
    return "done"
