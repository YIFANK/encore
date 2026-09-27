"""abl_c2 cell ablC_goal_put_bowl_top_cabinet -- variant C (no verification).

Intent: "put the bowl on top of the cabinet".

ONE version, written from packs/ablC_goal_put_bowl_top_cabinet/ alone (pack.json
keyframes + the 128x128 keyframe renders).  Zero episodes were executed while
authoring it, so every number below is either (a) read straight out of the pack
or (b) generic pinhole-camera / OSC-controller mechanics.

Mechanism
---------
1.  One cam_high RGB-D capture from the untouched home pose (the arm is high and
    behind the table there, exactly as in every demo*_t0000 keyframe, so nothing
    is occluded).
2.  A metric point cloud is built from .intrinsics / .t_base_cam and *validated*
    against a handful of api.deproject() calls, so the camera-axis convention is
    established at run time rather than assumed.
3.  The table plane is the dominant height mode.  Blobs standing above it are
    scored on height / radius / concavity / achromatic colour; the bowl is the
    concave mid-grey blob of bowl-ish size sitting directly on the table.
4.  The pack says the hold width is 6-12 mm (gripper_state at the carry
    keyframes) -- far too thin for a body grasp of a bowl -- so the grasp is a
    *rim pinch*: the gripper centre goes onto the rim crest, offset from the bowl
    centre by the measured rim radius, along a horizontal tool axis so the jaws
    close radially across the rim wall.  Which tool column is the closing axis is
    not assumed: attempt 1 uses the tool-Y horizontal, attempt 2 the tool-X one,
    and the two are separated by the gripper's own effort/width sensor (never by
    any task-success signal).
5.  The cabinet top is the large, dark, flat surface standing >=0.15 m above the
    table; the drop point is set inboard of its visible front/near edges, gated
    and clamped against the demo release prior.
6.  Release height is Z_GRASP + (cabinet_top - table) + a few mm.  That identity
    comes from the pack: the demos raised the EEF by 0.2423 m between closing on
    the rim and opening over the cabinet, so the bowl's underside ends level with
    the cabinet top and is set down rather than dropped.

No run-time success/termination signal is read anywhere.
"""

import numpy as np

# --------------------------------------------------------------------------
# Calibrated constants.  Every one of them is declared in PROVENANCE below.
# --------------------------------------------------------------------------

# pack.json, keyframe with gripper_cmd flipping to +1 (close): demo0 t=31
# z=0.9147, demo1 t=33 z=0.9139, demo2 t=32 z=0.9162  ->  mean 0.9149, spread 2mm
Z_GRASP = 0.9149
# same keyframes, xy: (-0.0990,0.0504) (-0.1047,0.0338) (-0.0848,0.0179)
GRASP_PRIOR_XY = (-0.0962, 0.0340)

# pack.json, keyframe where gripper_cmd returns to -1 over the cabinet:
# demo0 t=81 z=1.1489, demo1 t=98 z=1.1748, demo2 t=92 z=1.1473 -> mean 1.1570
Z_RELEASE_PRIOR = 1.1570
# same keyframes, xy: (0.0057,-0.1767) (0.0488,-0.1553) (-0.0379,-0.2079)
PLACE_PRIOR_XY = (0.0055, -0.1800)
# Z_RELEASE_PRIOR - Z_GRASP: how far the demos lifted the held bowl.
CABINET_LIFT = 0.2421

# max z reached along ee_path6 while carrying: 1.2219 / 1.2638 / 1.1983
CARRY_Z = 1.2550

# gripper_state sums.  open 0.0787 (t0 / after release), holding 0.0125 / 0.0059
OPEN_WIDTH = 0.0800
HOLD_WIDTH_MAX = 0.0340
HOLD_WIDTH_MIN = 0.0015

# --- scene appearance, measured off the pack's own keyframe renders ----------
# demo0_t0000 patch means: bowl grey 117 sat 0.071, burner grey 91 sat 0.006,
# plate grey 160 sat 0.070, table grey 172 sat 0.178, cabinet grey 29,
# wine bottle sat 0.483, blue box sat 0.229.
BOWL_SAT_MAX = 0.135
BOWL_GREY_MIN = 55.0
BOWL_GREY_MAX = 200.0
DARK_MAX_CHANNEL = 78.0

# --- geometry gates.  Bowl-scale ranges kept deliberately wide; they only have
# to separate the bowl from the plate / stove slab / burner / bottle / box.
BOWL_H_MIN = 0.022
BOWL_H_MAX = 0.115
BOWL_R_MIN = 0.025
BOWL_R_MAX = 0.105
OBJ_BAND_HI = 0.150

CAB_MIN_RISE = 0.150
CAB_MAX_RISE = 0.420
CAB_INSET_X = 0.045
CAB_INSET_Y = 0.055
CAB_TRUST_RADIUS = 0.150

# --- generic controller / camera mechanics ---------------------------------
APPROACH_CLEAR = 0.090
PLACE_CLEAR = 0.008
MOVE_SPEED = 0.150
MOVE_S_MIN = 0.70
MOVE_S_MAX = 2.20
MOVE_TOL = 0.014
TIME_BUDGET = 26.0
RETRY_BUDGET = 15.0

PROVENANCE = {
    "Z_GRASP": {"source": "pack.json demos[*].keyframes: ee[2] at the keyframe "
                          "where gripper_cmd flips to +1 (0.9147/0.9139/0.9162)",
                "allowed": True},
    "GRASP_PRIOR_XY": {"source": "pack.json demos[*].keyframes: mean ee[0:2] at "
                                 "the gripper-close keyframe", "allowed": True},
    "Z_RELEASE_PRIOR": {"source": "pack.json demos[*].keyframes: ee[2] at the "
                                  "keyframe where gripper_cmd returns to -1 "
                                  "(1.1489/1.1748/1.1473)", "allowed": True},
    "PLACE_PRIOR_XY": {"source": "pack.json demos[*].keyframes: mean ee[0:2] at "
                                 "the gripper-open (release) keyframe",
                       "allowed": True},
    "CABINET_LIFT": {"source": "pack.json: Z_RELEASE_PRIOR - Z_GRASP, the height "
                               "the demos raised the held bowl by", "allowed": True},
    "CARRY_Z": {"source": "pack.json demos[*].ee_path6 max z while carrying "
                          "(1.2219/1.2638/1.1983)", "allowed": True},
    "OPEN_WIDTH": {"source": "pack.json demos[*].keyframes gripper_state sum with "
                             "gripper_cmd=-1 (0.0787)", "allowed": True},
    "HOLD_WIDTH_MAX": {"source": "pack.json gripper_state sum while carrying the "
                                 "bowl (0.0125/0.0059) plus margin", "allowed": True},
    "HOLD_WIDTH_MIN": {"source": "pack.json gripper_state sum while carrying the "
                                 "bowl (min 0.0059) minus margin", "allowed": True},
    "BOWL_SAT_MAX": {"source": "pack keyframes/demo*_t0000.png patch statistics: "
                               "bowl saturation 0.071, table 0.178, bottle 0.483",
                     "allowed": True},
    "BOWL_GREY_MIN": {"source": "pack keyframes/demo*_t0000.png: bowl grey mean "
                                "117, std 35", "allowed": True},
    "BOWL_GREY_MAX": {"source": "pack keyframes/demo*_t0000.png: bowl grey mean "
                                "117, std 35", "allowed": True},
    "DARK_MAX_CHANNEL": {"source": "pack keyframes/demo*_t0000.png: cabinet patch "
                                   "mean rgb (33,28,25)", "allowed": True},
    "BOWL_H_MIN": {"source": "pack keyframes: bowl stands clearly proud of the "
                             "table while the plate is nearly flat; generic "
                             "depth-segmentation band", "allowed": True},
    "BOWL_H_MAX": {"source": "pack keyframes: bowl is far shorter than the wine "
                             "bottle / stove fitting; generic band", "allowed": True},
    "BOWL_R_MIN": {"source": "pack keyframes/demo*_t0000.png: bowl subtends ~20 "
                             "of 128 px against a ~25 px plate and a much wider "
                             "stove slab; wide generic bracket", "allowed": True},
    "BOWL_R_MAX": {"source": "pack keyframes/demo*_t0000.png: same width "
                             "comparison, upper bracket", "allowed": True},
    "OBJ_BAND_HI": {"source": "generic depth segmentation: table-relative band "
                              "that keeps tabletop crockery and cuts tall items",
                    "allowed": True},
    "CAB_MIN_RISE": {"source": "pack.json: the demos lifted the bowl 0.2421 m "
                               "onto the cabinet top, so its top is far above the "
                               "table; loose lower bracket", "allowed": True},
    "CAB_MAX_RISE": {"source": "pack.json CABINET_LIFT=0.2421; loose upper "
                               "bracket", "allowed": True},
    "CAB_INSET_X": {"source": "pack keyframes: the three demo release points span "
                              "0.087 m in x on the same cabinet top, so its top "
                              "face is large; inset keeps the bowl off the edge",
                    "allowed": True},
    "CAB_INSET_Y": {"source": "pack keyframes: demo release points span 0.053 m "
                              "in y on the same top face", "allowed": True},
    "CAB_TRUST_RADIUS": {"source": "pack keyframes/demo*_t0000.png: the cabinet "
                                   "silhouette moves <=2 px of 128 across the "
                                   "three demo layouts, so the release prior is "
                                   "tight; clamp radius around it", "allowed": True},
    "APPROACH_CLEAR": {"source": "generic controller mechanics: vertical standoff "
                                 "before a top-down descent", "allowed": True},
    "PLACE_CLEAR": {"source": "generic controller mechanics: few-mm standoff so "
                              "the bowl is set down, not dropped", "allowed": True},
    "MOVE_SPEED": {"source": "generic controller mechanics: seconds-per-metre used "
                             "to size api.move durations", "allowed": True},
    "MOVE_S_MIN": {"source": "generic controller mechanics: minimum settle time "
                             "for a commanded OSC waypoint", "allowed": True},
    "MOVE_S_MAX": {"source": "generic controller mechanics: cap on a single "
                             "waypoint duration", "allowed": True},
    "MOVE_TOL": {"source": "generic controller mechanics: residual under which a "
                           "waypoint counts as reached", "allowed": True},
    "TIME_BUDGET": {"source": "pack.json demos[*].length 91/110/99 control steps; "
                              "budget sized as a multiple of a demo",
                    "allowed": True},
    "RETRY_BUDGET": {"source": "pack.json demo lengths; point past which a second "
                               "grasp attempt no longer fits", "allowed": True},
}


# ==========================================================================
# small numpy helpers (no scipy dependency)
# ==========================================================================

def _label(mask):
    """Connected components (4-neighbour) on a small boolean array."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    idx = np.argwhere(mask)
    seen = mask.copy()
    for r0, c0 in idx:
        if not seen[r0, c0]:
            continue
        cur += 1
        stack = [(int(r0), int(c0))]
        seen[r0, c0] = False
        while stack:
            r, c = stack.pop()
            lab[r, c] = cur
            if r > 0 and seen[r - 1, c]:
                seen[r - 1, c] = False
                stack.append((r - 1, c))
            if r + 1 < h and seen[r + 1, c]:
                seen[r + 1, c] = False
                stack.append((r + 1, c))
            if c > 0 and seen[r, c - 1]:
                seen[r, c - 1] = False
                stack.append((r, c - 1))
            if c + 1 < w and seen[r, c + 1]:
                seen[r, c + 1] = False
                stack.append((r, c + 1))
    return lab, cur


def _unit(v):
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return None
    return np.asarray(v, float) / n


# ==========================================================================
# perception
# ==========================================================================

def _build_cloud(frame, log):
    """(H,W,3) base-frame points + validity mask.

    The camera-axis convention is *established*, not assumed: both the
    z-forward and the y-up/-z-forward conventions are evaluated and the one
    that reproduces frame.deproject() is kept.
    """
    depth = np.asarray(frame.depth, dtype=float)
    if depth.ndim == 3:
        depth = depth[..., 0]
    K = np.asarray(frame.intrinsics, dtype=float)
    T = np.asarray(frame.t_base_cam, dtype=float)
    h, w = depth.shape
    valid = np.isfinite(depth) & (depth > 0.05) & (depth < 6.0)

    vv, uu = np.mgrid[0:h, 0:w]
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    xn = (uu - cx) / fx
    yn = (vv - cy) / fy

    # sample pixels to score the conventions against the api's own deprojection
    probes = []
    step = max(h // 9, 1)
    for r in range(step, h - step, step):
        for c in range(step, w - step, step):
            if valid[r, c]:
                probes.append((r, c))
    probes = probes[:12]
    truth = []
    for r, c in probes:
        try:
            truth.append(np.asarray(frame.deproject(int(c), int(r)), float)[:3])
        except Exception:
            truth.append(None)

    best_pts, best_err = None, None
    for sgn in ((1.0, 1.0, 1.0), (1.0, -1.0, -1.0)):
        cam = np.stack([xn * depth * sgn[0],
                        yn * depth * sgn[1],
                        depth * sgn[2]], axis=-1)
        pts = cam @ T[:3, :3].T + T[:3, 3]
        errs = [float(np.linalg.norm(pts[r, c] - t))
                for (r, c), t in zip(probes, truth) if t is not None]
        err = float(np.median(errs)) if errs else 9.9
        if best_err is None or err < best_err:
            best_pts, best_err = pts, err
    log("cloud convention residual %.4f m" % best_err)

    if best_err > 0.05:
        # analytic model rejected -> coarse cloud straight from the api
        log("falling back to api.deproject grid")
        pts = np.full((h, w, 3), np.nan)
        st = 8
        for r in range(0, h, st):
            for c in range(0, w, st):
                if valid[r, c]:
                    try:
                        pts[r:r + st, c:c + st] = np.asarray(
                            frame.deproject(int(c), int(r)), float)[:3]
                    except Exception:
                        pass
        valid = valid & np.isfinite(pts[..., 2])
        return pts, valid

    valid = valid & np.isfinite(best_pts[..., 2])
    return best_pts, valid


def _colour_stats(rgb):
    a = np.asarray(rgb, dtype=float)
    mx = a.max(axis=-1)
    mn = a.min(axis=-1)
    sat = (mx - mn) / np.maximum(mx, 1.0)
    grey = a.mean(axis=-1)
    return mx, sat, grey


def _table_height(pts, valid):
    z = pts[..., 2][valid]
    x = pts[..., 0][valid]
    y = pts[..., 1][valid]
    keep = (np.abs(x) < 0.60) & (np.abs(y) < 0.75) & (z > 0.50) & (z < 1.10)
    z = z[keep]
    if z.size < 500:
        return None
    hist, edges = np.histogram(z, bins=np.arange(0.50, 1.101, 0.005))
    k = int(np.argmax(hist))
    lo, hi = edges[k], edges[k + 1]
    band = z[(z >= lo - 0.010) & (z <= hi + 0.010)]
    return float(np.median(band))


def _blob_features(pts, valid, sub, lab, comp, z_table, sat, grey):
    sel = (lab == comp)
    rows, cols = np.nonzero(sel)
    if rows.size < 6:
        return None
    rr = rows * sub
    cc = cols * sub
    P = pts[rr, cc]
    ok = np.isfinite(P[..., 2])
    P = P[ok]
    if P.shape[0] < 6:
        return None
    z_top = float(np.percentile(P[:, 2], 97))
    ring = P[P[:, 2] > z_top - 0.014]
    if ring.shape[0] < 4:
        ring = P
    cen = ring[:, :2].mean(axis=0)
    rad = float(np.median(np.linalg.norm(ring[:, :2] - cen, axis=1)))
    if not np.isfinite(rad) or rad < 1e-4:
        return None

    # concavity / support are read from the *raw* cloud, so a convex object
    # (bottle, knob) scores negative instead of being helped by masking.
    P_all = pts[valid]
    d_all = np.linalg.norm(P_all[:, :2] - cen, axis=1)
    inner = P_all[d_all < max(0.45 * rad, 0.012)]
    annulus = P_all[(d_all > 1.35 * rad) & (d_all < 1.35 * rad + 0.05)]
    z_in = float(np.median(inner[:, 2])) if inner.shape[0] >= 4 else z_top
    z_sup = float(np.median(annulus[:, 2])) if annulus.shape[0] >= 10 else z_table

    return {
        "centre": cen,
        "radius": rad,
        "z_top": z_top,
        "height": z_top - z_table,
        "concavity": z_top - z_in,
        "support": z_sup,
        "sat": float(np.mean(sat[rr, cc])),
        "grey": float(np.mean(grey[rr, cc])),
        "npx": int(rows.size),
    }


def _find_bowl(pts, valid, rgb, z_table, log):
    mx, sat, grey = _colour_stats(rgb)
    z = pts[..., 2]
    band = (valid & np.isfinite(z) & (z > z_table + BOWL_H_MIN * 0.7)
            & (z < z_table + OBJ_BAND_HI))
    tint = (sat < BOWL_SAT_MAX) & (grey > BOWL_GREY_MIN) & (grey < BOWL_GREY_MAX)
    mask = band & tint
    sub = 4
    small = mask[::sub, ::sub]
    lab, n = _label(small)
    log("bowl candidates: %d" % n)

    best, best_score = None, -1e9
    for comp in range(1, n + 1):
        if int((lab == comp).sum()) < 12:
            continue
        f = _blob_features(pts, valid, sub, lab, comp, z_table, sat, grey)
        if f is None:
            continue
        if not (BOWL_H_MIN <= f["height"] <= BOWL_H_MAX):
            continue
        if not (BOWL_R_MIN <= f["radius"] <= BOWL_R_MAX):
            continue
        score = 0.0
        score += 3.0 * min(f["concavity"], 0.06) / 0.06      # bowls are hollow
        score -= 6.0 * max(0.0, -f["concavity"])             # convex -> reject
        score -= 4.0 * max(0.0, abs(f["support"] - z_table) - 0.020)
        score += 0.8 * min(f["height"], 0.070) / 0.070       # proud of the table
        score += 0.6 * (1.0 - abs(f["radius"] - 0.05) / 0.06)
        score += 0.4 * (1.0 - abs(f["grey"] - 120.0) / 90.0)
        score += 0.3 * min(f["npx"], 400) / 400.0
        log("  cand r=%.3f h=%.3f conc=%.3f sup-dz=%.3f grey=%.0f npx=%d s=%.2f"
            % (f["radius"], f["height"], f["concavity"],
               f["support"] - z_table, f["grey"], f["npx"], score))
        if score > best_score:
            best, best_score = f, score
    return best


def _find_cabinet_top(pts, valid, rgb, z_table, log):
    mx, sat, grey = _colour_stats(rgb)
    z = pts[..., 2]
    high = (valid & np.isfinite(z) & (z > z_table + CAB_MIN_RISE)
            & (z < z_table + CAB_MAX_RISE) & (mx < DARK_MAX_CHANNEL))
    if int(high.sum()) < 200:
        log("no dark elevated surface")
        return None
    zt = float(np.percentile(z[high], 97))
    face = high & (z > zt - 0.018)
    sub = 4
    small = face[::sub, ::sub]
    lab, n = _label(small)
    best, best_px = None, 0
    for comp in range(1, n + 1):
        px = int((lab == comp).sum())
        if px < 12 or px <= best_px:
            continue
        rows, cols = np.nonzero(lab == comp)
        P = pts[rows * sub, cols * sub]
        P = P[np.isfinite(P[:, 2])]
        if P.shape[0] < 8:
            continue
        if (P[:, 0].max() - P[:, 0].min()) < 0.06:
            continue
        best, best_px = P, px
    if best is None:
        log("dark elevated surface too fragmented")
        return None
    log("cabinet top: %d px, z=%.3f, x[%.3f,%.3f] y[%.3f,%.3f]"
        % (best_px, zt, best[:, 0].min(), best[:, 0].max(),
           best[:, 1].min(), best[:, 1].max()))
    return {
        "z_top": zt,
        "x_front": float(np.percentile(best[:, 0], 92)),
        "y_near": float(np.percentile(best[:, 1], 92)),
        "x_med": float(np.median(best[:, 0])),
        "y_med": float(np.median(best[:, 1])),
        "npx": best_px,
    }


# ==========================================================================
# motion
# ==========================================================================

class Mover(object):
    def __init__(self, api):
        self.api = api
        self.spent = 0.0

    def goto(self, target, seconds=None, tol=MOVE_TOL, fix=True):
        target = [float(t) for t in target]
        here = np.asarray(self.api.eef(), float)
        dist = float(np.linalg.norm(np.asarray(target) - here))
        if seconds is None:
            seconds = float(np.clip(dist / MOVE_SPEED, MOVE_S_MIN, MOVE_S_MAX))
        self.spent += seconds
        res = self.api.move(target, seconds=seconds)
        try:
            bad = float(np.max(np.abs(np.asarray(res, float))))
        except Exception:
            bad = float(np.linalg.norm(np.asarray(self.api.eef(), float)
                                       - np.asarray(target)))
        if fix and bad > tol and self.spent < TIME_BUDGET:
            self.spent += 0.8
            self.api.move(target, seconds=0.8)
        return bad


def _horizontal_axes(api):
    """Two orthogonal horizontal directions taken from the tool frame.

    Which tool column actually carries the jaw-closing axis is a convention we
    refuse to assume, so both are offered and the gripper sensor picks.
    """
    axes = []
    try:
        R = np.asarray(api.tool_rotation(), float).reshape(3, 3)
        for col in (1, 0):
            a = _unit([R[0, col], R[1, col], 0.0])
            if a is not None:
                axes.append(a)
    except Exception:
        pass
    for fallback in ([0.0, 1.0, 0.0], [1.0, 0.0, 0.0]):
        f = np.asarray(fallback)
        if not any(abs(float(np.dot(f, a))) > 0.95 for a in axes):
            axes.append(f)
    out = []
    for a in axes[:3]:
        if abs(a[1]) >= abs(a[0]):
            if a[1] < 0:
                a = -a
        elif a[0] < 0:
            a = -a
        out.append(a)
    return out


def _holding(api):
    try:
        g = api.gripper()
    except Exception:
        return True, -1.0, -1.0
    width = float(g.get("width_m", -1.0))
    effort = float(g.get("effort", -1.0))
    ok = (effort >= 1.5) and (HOLD_WIDTH_MIN <= width <= HOLD_WIDTH_MAX
                              or width < 0.0)
    return ok, width, effort


# ==========================================================================
# entry point
# ==========================================================================

def run(api):
    log = getattr(api, "log", lambda m: None)
    try:
        log("intent: %s" % api.instruction())
    except Exception:                                         # noqa: BLE001
        pass
    mv = Mover(api)

    # ---------------- perception (home pose, nothing occluded) -------------
    bowl = None
    cab = None
    z_table = None
    try:
        frame = api.capture("cam_high")
        pts, valid = _build_cloud(frame, log)
        z_table = _table_height(pts, valid)
        log("table z = %s" % (None if z_table is None else round(z_table, 4)))
        if z_table is not None:
            bowl = _find_bowl(pts, valid, frame.rgb, z_table, log)
            cab = _find_cabinet_top(pts, valid, frame.rgb, z_table, log)
    except Exception as exc:                                  # noqa: BLE001
        log("perception failed (%s); falling back to pack priors" % exc)

    # ---------------- grasp target ----------------------------------------
    if bowl is not None:
        bx, by = float(bowl["centre"][0]), float(bowl["centre"][1])
        r_rim = float(np.clip(bowl["radius"], BOWL_R_MIN, BOWL_R_MAX))
        lift_dz = float(np.clip(bowl["support"] - z_table, -0.010, 0.080))
        if lift_dz < 0.015:
            lift_dz = 0.0
        log("bowl centre (%.3f,%.3f) r=%.3f support_dz=%.3f"
            % (bx, by, r_rim, lift_dz))
    else:
        bx, by = GRASP_PRIOR_XY
        r_rim = 0.040
        lift_dz = 0.0
        log("bowl not perceived -> pack prior grasp")
    z_grasp = Z_GRASP + lift_dz

    # ---------------- place target ----------------------------------------
    px, py = PLACE_PRIOR_XY
    z_release = Z_RELEASE_PRIOR + lift_dz
    if cab is not None and z_table is not None and cab["npx"] >= 25:
        cx = cab["x_front"] - CAB_INSET_X
        cy = cab["y_near"] - CAB_INSET_Y
        if abs(cx - PLACE_PRIOR_XY[0]) > CAB_TRUST_RADIUS:
            cx = PLACE_PRIOR_XY[0] + np.sign(cx - PLACE_PRIOR_XY[0]) * CAB_TRUST_RADIUS
        if abs(cy - PLACE_PRIOR_XY[1]) > CAB_TRUST_RADIUS:
            cy = PLACE_PRIOR_XY[1] + np.sign(cy - PLACE_PRIOR_XY[1]) * CAB_TRUST_RADIUS
        px, py = float(cx), float(cy)
        rise = cab["z_top"] - z_table
        z_release = float(np.clip(Z_GRASP + rise + PLACE_CLEAR,
                                  Z_GRASP + CAB_MIN_RISE,
                                  Z_GRASP + CAB_MAX_RISE))
    log("place target (%.3f,%.3f) release z %.3f" % (px, py, z_release))

    axes = _horizontal_axes(api)
    log("grasp axes: %s" % [list(np.round(a, 3)) for a in axes])

    # ---------------- pick ------------------------------------------------
    api.grip(OPEN_WIDTH)
    ok = False
    used_axis = axes[0] if axes else np.array([0.0, 1.0, 0.0])
    for attempt, axis in enumerate(axes[:2]):
        gx = bx + r_rim * float(axis[0])
        gy = by + r_rim * float(axis[1])
        log("grasp attempt %d at (%.3f,%.3f,%.3f)" % (attempt, gx, gy, z_grasp))
        mv.goto([gx, gy, z_grasp + APPROACH_CLEAR])
        mv.goto([gx, gy, z_grasp], seconds=1.1, fix=False)
        api.grip(0.0)
        api.settle(0.35)
        ok, width, effort = _holding(api)
        log("  gripper width %.4f effort %.2f -> hold=%s" % (width, effort, ok))
        used_axis = axis
        if ok:
            break
        if attempt + 1 >= 2 or mv.spent > RETRY_BUDGET or len(axes) < 2:
            break
        api.grip(OPEN_WIDTH)
        api.settle(0.15)
        mv.goto([gx, gy, z_grasp + APPROACH_CLEAR], seconds=1.0)

    if not ok:
        log("no hold reported; carrying out the transport regardless")

    # ---------------- carry -----------------------------------------------
    gx = bx + r_rim * float(used_axis[0])
    gy = by + r_rim * float(used_axis[1])
    mv.goto([gx, gy, z_grasp + 0.10], seconds=1.0)
    mv.goto([gx, gy, CARRY_Z])

    # the bowl hangs roughly under the jaws but keeps part of the rim offset;
    # split the difference so it lands near the middle of the cabinet top
    ex = px + 0.5 * r_rim * float(used_axis[0])
    ey = py + 0.5 * r_rim * float(used_axis[1])
    mv.goto([ex, ey, CARRY_Z])
    mv.goto([ex, ey, z_release], seconds=1.3)

    # ---------------- place -----------------------------------------------
    api.grip(OPEN_WIDTH)
    api.settle(0.4)
    mv.goto([ex, ey, min(CARRY_Z, z_release + 0.10)], seconds=1.0)
    api.settle(0.3)
    log("sequence complete, %.1f s of commanded motion" % mv.spent)
