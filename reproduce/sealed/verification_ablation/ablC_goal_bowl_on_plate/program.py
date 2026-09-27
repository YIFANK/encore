"""abl_c2 / cell ablC_goal_bowl_on_plate  (variant C -- no verification)

Intent: "put the bowl on the plate".

ONE version, written from the pack alone (packs/ablC_goal_bowl_on_plate/pack.json
+ keyframes/*.png).  Zero episodes were run before freezing this file.

Mechanism
---------
1. Look once with cam_high.  Segment the scene by colour statistics measured on
   the pack keyframes:
     * the bowl is the only compact, *neutral-grey, strongly mottled* blob that
       stands proud of the table (keyframe stats: mean value ~110, per-pixel
       value std ~35, |R-B| ~ 0);
     * the plate is identified by its unique salmon rim ring (R-G ~ 35 while
       G-B ~ 6, versus the wooden table's R-G ~ 16 / G-B ~ 17) together with the
       very uniform bright interior disc it encircles (value ~168, std ~5).
   Blob pixels are lifted to the base frame with api.deproject (the API's own
   deprojection, so no camera-convention guessing), giving metric centres, radii
   and heights above the table.
2. The bowl is ~72 mm across, so a top-down grasp cannot straddle it: the three
   demos all pinch the *rim wall* (gripper closes to an 11 mm gap) at a point
   offset ~35 mm from the bowl centre along the gripper's closing axis, which in
   the demos is the base +y axis.  The offset direction is re-derived at runtime
   from the tool rotation so that a different canonical wrist yaw is handled.
3. Grasp, lift, carry, and place with the *same* rim offset re-applied about the
   plate centre, so the bowl centre lands on the plate centre.
4. Every stage is checked with the program's own sensors only (gripper
   width/effort, move residuals, re-perception).  No runtime success signal and
   no episode-termination flag is consulted anywhere.
"""

import math
import numpy as np

# --------------------------------------------------------------------------
# Calibrated constants.  Sources are the pack for everything task-specific and
# the documented FairApi / camera-geometry mechanics for the rest.
# --------------------------------------------------------------------------
GRASP_Z = 0.9212            # ee z at the gripper-close keyframe of the 3 demos
PLACE_DZ = 0.012            # (place ee z - pick ee z) over the 3 demos
TRANSIT_Z = 1.030           # carry-waypoint ee z in ee_path6 of the 3 demos
HOVER_Z = 1.000             # just above the carry height, below the start pose
RETREAT_Z = 1.030
START_XYZ = (-0.207, 0.000, 1.168)   # mean t=0 keyframe ee of the 3 demos

RIM_OFFSET = 0.036          # demo grasp offset from bowl centre / bowl radius
RIM_OFFSET_MIN = 0.028
RIM_OFFSET_MAX = 0.045

BOWL_R_MIN, BOWL_R_MAX = 0.024, 0.050
PLATE_R_MIN, PLATE_R_MAX = 0.018, 0.062
BOWL_H_MIN, BOWL_H_MAX = 0.012, 0.100    # bowl rim height above the table top
PLATE_H_MAX = 0.032                      # plate is a ~10 mm thick flat disc

BOWL_V_MIN, BOWL_V_MAX = 70.0, 162.0     # keyframe bowl value range
BOWL_VSTD_MIN, BOWL_VSTD_MAX = 14.0, 62.0
PLATE_V_MIN, PLATE_V_MAX = 148.0, 198.0  # keyframe plate-interior value range
PLATE_VSTD_MAX = 14.0
NEUTRAL_TOL = 16.0                       # |R-G|,|G-B|,|R-B| for a grey surface
PINK_RG_MIN = 24.0                       # plate rim: R-G large, G-B near zero
PINK_GB_MAX = 13.0
PINK_GB_MIN = -6.0
PINK_V_MIN, PINK_V_MAX = 105.0, 205.0
WOOD_RB_MIN = 24.0                       # table top: warm wood, R-B ~ 33
WOOD_V_MIN = 150.0

WS_X_MIN, WS_X_MAX = -0.300, 0.280       # workspace box around the demo paths
WS_Y_MIN, WS_Y_MAX = -0.300, 0.300
FALLBACK_BOWL_XY = (-0.102, 0.006)       # demo grasp xy minus the rim offset
FALLBACK_PLATE_XY = (0.048, -0.010)      # demo release xy minus the rim offset
FALLBACK_TABLE_Z = 0.880

GRASP_ROTVEC = (3.0965, -0.0677, -0.0459)  # ee orientation at the demo grasps
GRIP_CLOSE = 0.000
GRIP_OPEN = 0.080
HOLD_EFFORT_MIN = 2.5       # FairApi: effort is 3.0 iff the gripper holds
HOLD_W_MIN, HOLD_W_MAX = 0.0035, 0.0320
MAX_GRASP_TRIES = 3

PROVENANCE = {
    "GRASP_Z": {"source": "pack.json demos[*].keyframes: ee z at the gripper-close "
                          "keyframe (0.9237 / 0.9202 / 0.9197 -> 0.9212)",
                "allowed": True},
    "PLACE_DZ": {"source": "pack.json demos[*].keyframes: release ee z minus grasp "
                           "ee z (0.0122 / 0.0078 / 0.0112 -> ~0.010, +2 mm margin)",
                 "allowed": True},
    "TRANSIT_Z": {"source": "pack.json demos[*].ee_path6 carry waypoints "
                            "(z = 1.0179 / 1.0322 / 1.0277)", "allowed": True},
    "HOVER_Z": {"source": "pack.json demos[*].ee_path6: approach waypoint band "
                          "between the start height and the carry height",
                "allowed": True},
    "RETREAT_Z": {"source": "pack.json demos[*].ee_path6 carry waypoints",
                  "allowed": True},
    "START_XYZ": {"source": "pack.json demos[*].keyframes t=0 ee "
                            "(-0.2144/-0.1981/-0.2090, 0.0079/-0.0078/-0.0067, "
                            "1.156/1.1766/1.1724)", "allowed": True},
    "RIM_OFFSET": {"source": "pack.json + keyframes: demo grasp ee xy minus the "
                             "bowl silhouette centroid, expressed with the "
                             "keyframe image scale (0.0335 / 0.0368 / 0.0357); "
                             "equals the bowl silhouette radius (~17 px)",
                   "allowed": True},
    "RIM_OFFSET_MIN": {"source": "pack keyframes: bowl silhouette radius spread "
                                 "across the 3 demos, lower clamp",
                       "allowed": True},
    "RIM_OFFSET_MAX": {"source": "pack keyframes: bowl silhouette radius spread "
                                 "across the 3 demos, upper clamp",
                       "allowed": True},
    "BOWL_R_MIN": {"source": "pack keyframes: bowl silhouette 16-17 px wide at the "
                             "demo image scale, widened for layout variation",
                   "allowed": True},
    "BOWL_R_MAX": {"source": "pack keyframes: bowl silhouette radius ~8.5 px at "
                             "the demo image scale (~0.036 m), widened; also "
                             "rejects the wider neutral appliance blob (30 px)",
                   "allowed": True},
    "PLATE_R_MIN": {"source": "pack keyframes: plate interior disc 16 px wide at "
                              "the demo image scale, widened", "allowed": True},
    "PLATE_R_MAX": {"source": "pack keyframes: plate outer rim 21 px wide at the "
                              "demo image scale, widened", "allowed": True},
    "BOWL_H_MIN": {"source": "pack keyframes: the bowl stands proud of the table "
                             "whereas the plate is flush; generic depth mechanics",
                   "allowed": True},
    "BOWL_H_MAX": {"source": "generic scene mechanics: cap that rejects tall "
                             "background objects and the elevated appliance",
                   "allowed": True},
    "PLATE_H_MAX": {"source": "pack.json: place ee z exceeds grasp ee z by ~10 mm, "
                              "i.e. the plate is a ~10 mm thick flat disc",
                    "allowed": True},
    "BOWL_V_MIN": {"source": "pack keyframes demo{0,1,2}_t0000 bowl pixels: value "
                             "mean 108-112, 10th pct ~52", "allowed": True},
    "BOWL_V_MAX": {"source": "pack keyframes bowl pixels: value 90th pct ~149",
                   "allowed": True},
    "BOWL_VSTD_MIN": {"source": "pack keyframes bowl blob per-pixel value std "
                                "33-38 (mottled) vs plate std 5", "allowed": True},
    "BOWL_VSTD_MAX": {"source": "pack keyframes bowl blob value std, widened",
                      "allowed": True},
    "PLATE_V_MIN": {"source": "pack keyframes plate interior pixels: value "
                              "168-169", "allowed": True},
    "PLATE_V_MAX": {"source": "pack keyframes plate interior pixels: value "
                              "168-169, widened", "allowed": True},
    "PLATE_VSTD_MAX": {"source": "pack keyframes plate interior blob value std "
                                 "~5 (uniform), widened", "allowed": True},
    "NEUTRAL_TOL": {"source": "pack keyframes: bowl |R-B| ~ 0 and plate |R-B| ~ 5 "
                              "versus table |R-B| ~ 33 and plate rim |R-B| ~ 40",
                    "allowed": True},
    "PINK_RG_MIN": {"source": "pack keyframes plate rim pixels: R-G ~ 30-40 versus "
                              "table R-G ~ 16", "allowed": True},
    "PINK_GB_MAX": {"source": "pack keyframes plate rim pixels: G-B ~ 5-8 versus "
                              "table G-B ~ 17", "allowed": True},
    "PINK_GB_MIN": {"source": "pack keyframes plate rim pixels: G-B ~ 5-8, lower "
                              "clamp", "allowed": True},
    "PINK_V_MIN": {"source": "pack keyframes plate rim pixels: value 128-174",
                   "allowed": True},
    "PINK_V_MAX": {"source": "pack keyframes plate rim pixels: value 128-174, "
                             "widened", "allowed": True},
    "WOOD_RB_MIN": {"source": "pack keyframes table pixels: R-B ~ 33",
                    "allowed": True},
    "WOOD_V_MIN": {"source": "pack keyframes table pixels: value ~192",
                   "allowed": True},
    "WS_X_MIN": {"source": "pack.json ee_path extents (x in -0.214..0.057), "
                           "widened for unseen layouts", "allowed": True},
    "WS_X_MAX": {"source": "pack.json ee_path extents, widened", "allowed": True},
    "WS_Y_MIN": {"source": "pack.json ee_path extents (y in -0.008..0.061), "
                           "widened for unseen layouts", "allowed": True},
    "WS_Y_MAX": {"source": "pack.json ee_path extents, widened", "allowed": True},
    "FALLBACK_BOWL_XY": {"source": "pack.json demo grasp ee xy minus RIM_OFFSET*+y "
                                   "(mean of the 3 demos)", "allowed": True},
    "FALLBACK_PLATE_XY": {"source": "pack.json demo release ee xy minus "
                                    "RIM_OFFSET*+y (mean of the 3 demos)",
                          "allowed": True},
    "FALLBACK_TABLE_Z": {"source": "pack.json: grasp ee z 0.9212 sits at the bowl "
                                   "rim, a few cm above the table top",
                         "allowed": True},
    "GRASP_ROTVEC": {"source": "pack.json demos[*].keyframes: ee orientation at "
                               "the grasp keyframes (mean rotation vector)",
                     "allowed": True},
    "GRIP_CLOSE": {"source": "FairApi surface: api.grip(w) with w < 0.025 closes",
                   "allowed": True},
    "GRIP_OPEN": {"source": "FairApi surface: api.grip(w) with w >= 0.025 opens",
                  "allowed": True},
    "HOLD_EFFORT_MIN": {"source": "FairApi surface: gripper effort is 3.0 iff the "
                                  "gripper is holding", "allowed": True},
    "HOLD_W_MIN": {"source": "pack.json demos[*].keyframes gripper_state while "
                             "holding: finger gap 0.011-0.022 m (versus ~0 on a "
                             "closed-on-air grasp)", "allowed": True},
    "HOLD_W_MAX": {"source": "pack.json gripper_state while holding, widened",
                   "allowed": True},
    "MAX_GRASP_TRIES": {"source": "generic control mechanics: bounded retry budget "
                                  "so the fixed sequence always terminates",
                        "allowed": True},
}


# ---------------------------------------------------------------- math utils
def _rodrigues(rv):
    rv = np.asarray(rv, dtype=float)
    th = float(np.linalg.norm(rv))
    if th < 1e-9:
        return np.eye(3)
    k = rv / th
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)


def _unit2(v):
    n = float(math.hypot(v[0], v[1]))
    if n < 1e-9:
        return np.array([0.0, 1.0])
    return np.array([v[0] / n, v[1] / n])


# ------------------------------------------------------------- image helpers
def _down(img, target=128):
    """Block-mean an HxWx3 image down to roughly `target` on a side."""
    a = np.asarray(img, dtype=float)
    h, w = a.shape[0], a.shape[1]
    f = max(1, int(round(min(h, w) / float(target))))
    h2, w2 = (h // f) * f, (w // f) * f
    a = a[:h2, :w2]
    a = a.reshape(h2 // f, f, w2 // f, f, 3).mean(axis=(1, 3))
    return a, f


def _erode(mask):
    m = mask
    out = m.copy()
    out[1:, :] &= m[:-1, :]
    out[:-1, :] &= m[1:, :]
    out[:, 1:] &= m[:, :-1]
    out[:, :-1] &= m[:, 1:]
    return out


def _components(mask, min_px=12, max_px=4000):
    """8-connected components; each one is flooded completely, then filtered."""
    h, w = mask.shape
    seen = np.zeros((h, w), dtype=bool)
    comps = []
    nbr = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    for v0 in range(h):
        row = mask[v0]
        for u0 in range(w):
            if not row[u0] or seen[v0, u0]:
                continue
            stack = [(v0, u0)]
            seen[v0, u0] = True
            px = []
            while stack:
                a, b = stack.pop()
                px.append((a, b))
                for da, db in nbr:
                    na, nb = a + da, b + db
                    if 0 <= na < h and 0 <= nb < w and mask[na, nb] and not seen[na, nb]:
                        seen[na, nb] = True
                        stack.append((na, nb))
            if min_px <= len(px) <= max_px:
                comps.append(px)
    return comps


def _dedupe(comps):
    """Drop near-duplicate blobs (a blob and its eroded twin)."""
    keys = []
    out = []
    for px in comps:
        cv = sum(p[0] for p in px) / float(len(px))
        cu = sum(p[1] for p in px) / float(len(px))
        dup = False
        for ov, ou, on in keys:
            if abs(cv - ov) < 2.0 and abs(cu - ou) < 2.0 and \
                    abs(len(px) - on) < 0.35 * max(on, len(px)):
                dup = True
                break
        if not dup:
            keys.append((cv, cu, len(px)))
            out.append(px)
    return out


def _masks(small):
    R = small[:, :, 0]
    G = small[:, :, 1]
    B = small[:, :, 2]
    V = small.max(axis=2)
    neutral = ((np.abs(R - B) <= NEUTRAL_TOL) & (np.abs(R - G) <= NEUTRAL_TOL) &
               (np.abs(G - B) <= NEUTRAL_TOL) & (V > 45.0) & (V < 215.0))
    pink = (((R - G) >= PINK_RG_MIN) & ((G - B) <= PINK_GB_MAX) &
            ((G - B) >= PINK_GB_MIN) & (V >= PINK_V_MIN) & (V <= PINK_V_MAX))
    wood = (((R - B) >= WOOD_RB_MIN) & (R > G) & (G > B) & (V >= WOOD_V_MIN))
    return neutral, pink, wood, V


# ------------------------------------------------------------- 3-D from blobs
def _lift(api, px, factor, limit=70):
    """api.deproject a subsample of blob pixels -> Nx3 base-frame points."""
    n = len(px)
    if n == 0:
        return np.zeros((0, 3))
    step = max(1, int(math.ceil(n / float(limit))))
    pts = []
    for i in range(0, n, step):
        v, u = px[i]
        uu = int(u * factor + factor // 2)
        vv = int(v * factor + factor // 2)
        try:
            p = api.deproject(uu, vv)
        except Exception:
            continue
        p = np.asarray(p, dtype=float).reshape(-1)
        if p.shape[0] >= 3 and np.all(np.isfinite(p[:3])):
            pts.append(p[:3])
    if not pts:
        return np.zeros((0, 3))
    return np.asarray(pts, dtype=float)


def _ring_centre(pts, band=0.012):
    """Centre / radius of a blob, preferring its topmost band (a bowl rim)."""
    if pts.shape[0] == 0:
        return None
    zmax = float(np.max(pts[:, 2]))
    top = pts[pts[:, 2] >= zmax - band]
    use = top if top.shape[0] >= 10 else pts
    c = np.array([float(np.median(use[:, 0])), float(np.median(use[:, 1]))])
    d = np.hypot(use[:, 0] - c[0], use[:, 1] - c[1])
    r = float(np.percentile(d, 85.0)) if d.size else 0.0
    return c, r, zmax


def _in_ws(c):
    return (WS_X_MIN <= c[0] <= WS_X_MAX) and (WS_Y_MIN <= c[1] <= WS_Y_MAX)


def _annulus_wood(wood, px, r_px):
    """Fraction of table-coloured pixels in a ring around a blob."""
    h, w = wood.shape
    vs = np.array([p[0] for p in px], dtype=float)
    us = np.array([p[1] for p in px], dtype=float)
    cv, cu = float(vs.mean()), float(us.mean())
    lo, hi = 1.5 * r_px, 2.8 * r_px
    hits = tot = 0
    v0, v1 = int(max(0, cv - hi)), int(min(h - 1, cv + hi))
    u0, u1 = int(max(0, cu - hi)), int(min(w - 1, cu + hi))
    for v in range(v0, v1 + 1):
        for u in range(u0, u1 + 1):
            d = math.hypot(v - cv, u - cu)
            if lo <= d <= hi:
                tot += 1
                if wood[v, u]:
                    hits += 1
    return (hits / float(tot)) if tot else 0.0


def perceive(api, want_bowl=True, want_plate=True):
    """Return (bowl_centre_xy, bowl_radius, plate_centre_xy, table_z) or Nones."""
    try:
        frame = api.capture("cam_high")
    except Exception:
        return None, None, None, None
    small, factor = _down(np.asarray(frame.rgb))
    neutral, pink, wood, V = _masks(small)

    # ---- table height: median z of table-coloured pixels
    table_z = FALLBACK_TABLE_Z
    wv, wu = np.nonzero(wood)
    if wv.size >= 40:
        idx = np.linspace(0, wv.size - 1, min(60, wv.size)).astype(int)
        wpx = [(int(wv[i]), int(wu[i])) for i in idx]
        wpts = _lift(api, wpx, factor, limit=60)
        if wpts.shape[0] >= 10:
            table_z = float(np.median(wpts[:, 2]))

    # tableware blobs are ~150-200 px at 128x128 in the pack keyframes; the cap
    # discards the wall/robot region without ever lifting it to 3-D
    cand = _dedupe(_components(neutral, 20, 1200) +
                   _components(_erode(neutral), 15, 1200))

    bowl_c = bowl_r = None
    plate_c = None

    if want_bowl:
        best = None
        for px in cand:
            vals = np.array([V[p] for p in px], dtype=float)
            vm, vs = float(vals.mean()), float(vals.std())
            if not (BOWL_V_MIN <= vm <= BOWL_V_MAX):
                continue
            if not (BOWL_VSTD_MIN <= vs <= BOWL_VSTD_MAX):
                continue
            uu = np.array([p[1] for p in px], dtype=float)
            vv = np.array([p[0] for p in px], dtype=float)
            du = uu.max() - uu.min() + 1.0
            dv = vv.max() - vv.min() + 1.0
            if du > 3.0 * dv or dv > 3.0 * du:
                continue
            pts = _lift(api, px, factor)
            got = _ring_centre(pts)
            if got is None:
                continue
            c, r, zmax = got
            if not _in_ws(c):
                continue
            if not (BOWL_R_MIN <= r <= BOWL_R_MAX):
                continue
            hgt = zmax - table_z
            if not (BOWL_H_MIN <= hgt <= BOWL_H_MAX):
                continue
            wf = _annulus_wood(wood, px, 0.5 * max(du, dv))
            if wf < 0.20:          # a free-standing object sits on bare table
                continue
            score = (2.0 * wf - 12.0 * abs(r - RIM_OFFSET) -
                     abs(vm - 110.0) / 120.0 - abs(hgt - 0.040) * 3.0)
            if best is None or score > best[0]:
                best = (score, c, r, hgt, vm, vs)
        if best is not None:
            bowl_c, bowl_r = best[1], best[2]
            api.log("bowl xy=(%.3f,%.3f) r=%.3f h=%.3f v=%.0f/%.0f" %
                    (bowl_c[0], bowl_c[1], bowl_r, best[3], best[4], best[5]))

    if want_plate:
        # the salmon rim ring is unique in the scene -> anchors the plate
        rim_c = None
        rims = _components(pink, 8, 3000)
        if rims:
            rims.sort(key=len, reverse=True)
            rpts = _lift(api, rims[0], factor)
            if rpts.shape[0] >= 8:
                rim_c = np.array([float(np.median(rpts[:, 0])),
                                  float(np.median(rpts[:, 1]))])
                if not _in_ws(rim_c):
                    rim_c = None
        best = None
        for px in cand:
            vals = np.array([V[p] for p in px], dtype=float)
            vm, vs = float(vals.mean()), float(vals.std())
            if not (PLATE_V_MIN <= vm <= PLATE_V_MAX) or vs > PLATE_VSTD_MAX:
                continue
            pts = _lift(api, px, factor)
            got = _ring_centre(pts)
            if got is None:
                continue
            c, r, zmax = got
            if not _in_ws(c):
                continue
            if not (PLATE_R_MIN <= r <= PLATE_R_MAX):
                continue
            if (zmax - table_z) > PLATE_H_MAX:
                continue
            near = 0.0 if rim_c is None else float(np.hypot(*(c - rim_c)))
            if rim_c is not None and near > 0.060:
                continue
            score = -3.0 * near - abs(zmax - table_z - 0.010) * 4.0
            if best is None or score > best[0]:
                best = (score, c, r)
        if best is not None:
            plate_c = best[1]
            api.log("plate xy=(%.3f,%.3f) r=%.3f" % (plate_c[0], plate_c[1], best[2]))
        elif rim_c is not None:
            plate_c = rim_c
            api.log("plate (rim only) xy=(%.3f,%.3f)" % (plate_c[0], plate_c[1]))

    return bowl_c, bowl_r, plate_c, table_z


# ------------------------------------------------------------------- motions
def _scalar(res):
    """Coerce a move residual (scalar or vector) to a non-negative magnitude."""
    try:
        a = np.asarray(res, dtype=float).reshape(-1)
    except Exception:
        return 0.0
    if a.size == 0 or not np.all(np.isfinite(a)):
        return 0.0
    if a.size == 1:
        return abs(float(a[0]))
    return float(np.linalg.norm(a[:3]))


def goto(api, xyz, seconds=2.0, tries=2, tol=0.010):
    res = None
    for _ in range(max(1, tries)):
        try:
            res = api.move([float(xyz[0]), float(xyz[1]), float(xyz[2])],
                           seconds=seconds)
        except Exception:
            return res
        if _scalar(res) <= tol:
            return res
    return res


def _grip_state(api):
    try:
        g = api.gripper()
    except Exception:
        return None, None
    w = e = None
    try:
        w = float(g["width_m"])
        e = float(g["effort"])
    except Exception:
        w = float(getattr(g, "width_m", "nan"))
        e = float(getattr(g, "effort", "nan"))
    if w is not None and not np.isfinite(w):
        w = None
    if e is not None and not np.isfinite(e):
        e = None
    return w, e


def holding(api):
    """True iff the program's own sensors say the gripper carries something."""
    w, e = _grip_state(api)
    if e is not None:
        return (e >= HOLD_EFFORT_MIN) and ((w is None) or (w <= HOLD_W_MAX))
    if w is None:
        return False
    return HOLD_W_MIN <= w <= HOLD_W_MAX


def holding_stable(api):
    """Two reads; a single dropped sample must not cost a good grasp."""
    if holding(api):
        return True
    api.settle(0.25)
    return holding(api)


def grasp_dir(api):
    """Unit xy direction from the bowl centre to the rim-pinch point."""
    d = np.array([0.0, 1.0])
    try:
        R_now = np.asarray(api.tool_rotation(), dtype=float).reshape(3, 3)
        off_tool = _rodrigues(GRASP_ROTVEC).T @ np.array([0.0, 1.0, 0.0])
        w = R_now @ off_tool
        if np.all(np.isfinite(w)) and abs(w[2]) < 0.55 and math.hypot(w[0], w[1]) > 1e-6:
            u = _unit2(w)
            # snap to the nearest principal axis; trust only y-like / x-like
            axes = (np.array([0.0, 1.0]), np.array([0.0, -1.0]),
                    np.array([1.0, 0.0]), np.array([-1.0, 0.0]))
            best = max(axes, key=lambda a: float(a @ u))
            if float(best @ u) >= math.cos(math.radians(35.0)):
                d = best
    except Exception:
        pass
    return d


def run(api):
    try:
        api.log("intent: %s" % str(api.instruction()))
    except Exception:
        pass

    api.grip(GRIP_OPEN)
    api.settle(0.2)

    bowl_c, bowl_r, plate_c, _tz = perceive(api)
    if bowl_c is None or plate_c is None:
        # one retry from the demo start pose, in case the arm shadowed the table
        goto(api, START_XYZ, seconds=1.6, tries=1)
        api.settle(0.2)
        b2, r2, p2, _tz = perceive(api, want_bowl=bowl_c is None,
                                   want_plate=plate_c is None)
        if bowl_c is None and b2 is not None:
            bowl_c, bowl_r = b2, r2
        if plate_c is None and p2 is not None:
            plate_c = p2
    if bowl_c is None:
        bowl_c = np.array(FALLBACK_BOWL_XY, dtype=float)
        api.log("bowl not segmented -> pack fallback")
    if plate_c is None:
        plate_c = np.array(FALLBACK_PLATE_XY, dtype=float)
        api.log("plate not segmented -> pack fallback")
    if bowl_r is None:
        bowl_r = RIM_OFFSET
    r_off = min(RIM_OFFSET_MAX, max(RIM_OFFSET_MIN, float(bowl_r)))

    d0 = grasp_dir(api)
    perp = np.array([-d0[1], d0[0]])
    ladder = ((d0, 1.00), (-d0, 1.00), (perp, 1.00))

    held = False
    used = None
    for attempt in range(MAX_GRASP_TRIES):
        d, k = ladder[attempt % len(ladder)]
        off = d * (r_off * k)
        gx = float(bowl_c[0] + off[0])
        gy = float(bowl_c[1] + off[1])
        api.log("grasp try %d at (%.3f,%.3f) dir=(%.2f,%.2f) r=%.3f" %
                (attempt + 1, gx, gy, d[0], d[1], r_off * k))

        api.grip(GRIP_OPEN)
        goto(api, (gx, gy, TRANSIT_Z), seconds=2.0, tries=2)
        goto(api, (gx, gy, GRASP_Z + 0.045), seconds=1.4, tries=1)
        goto(api, (gx, gy, GRASP_Z), seconds=1.4, tries=1, tol=0.004)
        api.settle(0.2)
        api.grip(GRIP_CLOSE)
        api.settle(0.5)
        goto(api, (gx, gy, GRASP_Z + 0.055), seconds=1.4, tries=1)
        api.settle(0.3)
        if holding_stable(api):
            held = True
            used = off
            api.log("holding after try %d" % (attempt + 1))
            break
        api.log("no hold after try %d" % (attempt + 1))
        api.grip(GRIP_OPEN)
        api.settle(0.2)
        goto(api, (gx, gy, TRANSIT_Z), seconds=1.4, tries=1)
        if attempt + 1 < MAX_GRASP_TRIES:
            # step the arm back to the demo start pose so it does not shadow the
            # table while the scene is re-segmented
            goto(api, START_XYZ, seconds=1.6, tries=1)
            api.settle(0.2)
            b2, r2, p2, _tz = perceive(api)
            if b2 is not None:
                bowl_c = b2
                if r2 is not None:
                    r_off = min(RIM_OFFSET_MAX, max(RIM_OFFSET_MIN, float(r2)))
            if p2 is not None:
                plate_c = p2

    if used is None:
        used = d0 * r_off

    # ---- carry: re-apply the same rim offset about the plate centre so that the
    # ---- bowl centre (which hangs at eef - offset) lands on the plate centre.
    px = float(plate_c[0] + used[0])
    py = float(plate_c[1] + used[1])
    try:
        here = np.asarray(api.eef(), dtype=float).reshape(-1)
        lift_from = (float(here[0]), float(here[1]), TRANSIT_Z)
    except Exception:
        lift_from = (px, py, TRANSIT_Z)
    goto(api, lift_from, seconds=1.2, tries=1)
    goto(api, (px, py, TRANSIT_Z), seconds=2.0, tries=2)
    api.settle(0.2)

    if held and not holding_stable(api):
        api.log("lost the bowl in transit")
        held = False

    goto(api, (px, py, GRASP_Z + PLACE_DZ + 0.045), seconds=1.4, tries=1)
    goto(api, (px, py, GRASP_Z + PLACE_DZ), seconds=1.6, tries=1, tol=0.004)
    api.settle(0.3)
    api.grip(GRIP_OPEN)
    api.settle(0.5)
    goto(api, (px, py, RETREAT_Z), seconds=1.6, tries=1)
    api.settle(0.3)
    api.log("done: released above plate xy=(%.3f,%.3f)" % (plate_c[0], plate_c[1]))
