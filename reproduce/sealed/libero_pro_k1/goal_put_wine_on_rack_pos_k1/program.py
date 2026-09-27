"""put the wine bottle on the rack  (c2k1clean, K=1, _pos cell)

Mechanism, read off pack demo0:
  keyframe t=81 closes the jaws 0.029 m below the bottle's top with a 0.0177 m
  finger gap -- a pinch on the NECK, not the body -- lifts, and releases the
  bottle over the rack (t=150).  The pack's absolute xy is from another scene
  (its frames show a cabinet this scene does not have), so only that relative
  grasp height and the pick-then-place-on-the-rack mechanism transfer; every
  coordinate below is perceived from this episode's own cam_high RGB-D.

Scene facts re-derived on debug seeds 51-65:
  table top z = 0.901
  bottle: dark green, 0.1485 m long, body 0.041 m across, neck 0.0179 m across.
          14/15 debug seeds spawn it standing, seed 64 spawns it lying down.
  rack:   two planar slat beds, both tilted dz/dy = -0.575, 0.103 m apart in z.
          The upper bed is the target.
Controller facts: move(rotation=...) chases a 1-degree tolerance it cannot hold
  and then tracks ~50 mm wide (probe2), so the wrist is set at most once, high
  and empty-handed, and every other move passes rotation=None.  Episode horizon
  is 1000 controller steps (probe3b); one pick-and-place costs ~150.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-65: mode of the cam_high depth over the "
                          "central crop, z=0.901", "allowed": True},
    "BOTTLE_LEN": {"source": "debug seeds: standing dark-green component top z "
                             "1.0494+-0.0003 minus TABLE_Z", "allowed": True},
    "BOTTLE_DIA": {"source": "debug seeds: the component is 0.041 m wide below the "
                             "shoulder (seed 64, lying: 0.042 m tall)", "allowed": True},
    "GRASP_DROP": {"source": "pack demo0 keyframe t=81 EEF z 1.0207 sits 0.029 below "
                             "the bottle top; that drop lands in the neck band measured "
                             "on the debug seeds", "allowed": True},
    "NECK_BAND": {"source": "debug seeds: the component is 0.0179 m wide over "
                            "z_top-0.049..z_top-0.019 (~200 px on all 15); the wider "
                            "fallback bands only fire when that band is occluded",
                  "allowed": True},
    "LIE_GRASP_Z": {"source": "debug seed 64 + the table-press calibration in probe2 "
                              "(open fingertips stop 0.008 below the EEF), so EEF "
                              "TABLE_Z+0.028 puts the tips just under a lying bottle's "
                              "mid-line", "allowed": True},
    "STAND_MIN": {"source": "debug seeds: standing tops sit 0.148 above the table, the "
                            "lying one 0.042; 0.10 separates them", "allowed": True},
    "DARK_MAX": {"source": "debug-seed cam_high RGB: the bottle reads (1,6,0)..(28,33,28) "
                           "and is the only green-dominant dark object", "allowed": True},
    "WOOD_R_MIN": {"source": "debug-seed cam_high RGB: the rack slats read R>110 with "
                             "R-B>10", "allowed": True},
    "BED_SLOPE": {"source": "debug seeds: least-squares plane through the rack slat "
                            "points gives dz/dy = -0.575 (rms 2.2 mm) on both beds",
                  "allowed": True},
    "BED_HALF": {"source": "debug seeds: the two beds' plane intercepts differ by 0.103, "
                           "so a 0.012 window round the upper peak isolates it",
                 "allowed": True},
    "TRANSIT_CLEAR": {"source": "debug seeds: the transit height is the bed's own high "
                                "edge plus the measured hang plus 0.035; the 1.33..1.40 "
                                "clip is the band the arm tracked cleanly in probe5",
                      "allowed": True},
    "PLACE_CLEAR": {"source": "debug seeds: 0.004 above the bed plane; the move lands "
                              "~0.010 high, which drops the bottle onto the bed",
                    "allowed": True},
    "XBIAS": {"source": "debug seeds: the descent lands +0.009 in x of its command "
                        "(want 0.0471 -> got 0.0556 and the same on 57/63); the "
                        "0.020 clip and 0.045 gate are the envelope runs g_gx_*",
              "allowed": True},
    "HELD_W": {"source": "debug seeds: a neck pinch reports width 0.0148, a body pinch "
                         "~0.040, an empty close 0.0018", "allowed": True},
    "WS": {"source": "debug seeds: x -0.45..0.32, y -0.55..0.35 covers the table and "
                     "excludes the walls, which deproject to x=-1.99", "allowed": True},
}

TABLE_Z = 0.901
BOTTLE_LEN = 0.1485
GRASP_DROP = 0.029
STAND_MIN = 0.10
LIE_GRASP_Z = TABLE_Z + 0.028
DARK_MAX = 45
WOOD_R_MIN = 110
WOOD_RB = 10
BED_SLOPE = 0.575
BED_HALF = 0.012
TRANSIT_CLEAR = 0.035
PLACE_CLEAR = 0.004
HELD_MIN_W = 0.006
POS_TOL = 0.012          # the controller's own arrival test (probe4 residuals)
HORIZON = 1000           # controller steps per episode (probe3b)
BUDGET = {"used": 0}


def _spend(n):
    BUDGET["used"] += int(n)


def _left():
    return HORIZON - BUDGET["used"]


# --------------------------------------------------------------------------- vision
def _cloud(f):
    d = np.asarray(f.depth, dtype=np.float32)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = d.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    z = np.where(np.isfinite(d) & (d > 0), d, np.nan)
    p = np.stack([(u - K[0, 2]) * z / K[0, 0],
                  (v - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], -1)
    return (p @ T.T)[..., :3]


def _label(mask):
    """4-connected labelling (scipy is not assumed present in the sandbox)."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    parent = [0]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    nxt = 1
    for i in range(h):
        row = mask[i]
        if not row.any():
            continue
        for j in np.nonzero(row)[0]:
            up = lab[i - 1, j] if i else 0
            lf = lab[i, j - 1] if j else 0
            if up and lf:
                lab[i, j] = min(up, lf)
                ra, rb = find(up), find(lf)
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)
            elif up or lf:
                lab[i, j] = up or lf
            else:
                parent.append(nxt)
                lab[i, j] = nxt
                nxt += 1
    for i in range(1, nxt):
        parent[i] = find(i)
    return np.array(parent, np.int32)[lab], nxt


def _ws(X, Y, Z):
    return np.isfinite(Z) & (X > -0.45) & (X < 0.32) & (Y > -0.55) & (Y < 0.35)


def _bottle_mask(f, B, zmax):
    C = np.asarray(f.rgb).astype(int)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    dark = ((C.max(2) < DARK_MAX) & (C[..., 1] > C[..., 0]) & (C[..., 1] > C[..., 2]))
    m = dark & _ws(X, Y, Z) & (Z > TABLE_Z + 0.014) & (Z < zmax)
    lab, n = _label(m)
    best, bn = None, 0
    for i in range(1, n):
        c = lab == i
        k = int(c.sum())
        if k > bn:
            bn, best = k, c
    if best is None or bn < 250:
        return None, 0
    return best, bn


def find_bottle(api, f, B):
    """-> dict(kind, xy grasp point, z_top, closing direction) or None."""
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    c, bn = _bottle_mask(f, B, TABLE_Z + 0.25)
    if c is None:
        return None
    zt = float(Z[c].max())
    if zt - TABLE_Z > STAND_MIN:
        # The silhouette's y-extent is the local diameter (y is the camera's
        # horizontal axis, so it is not foreshortened); 2r is then the radius and
        # x_max at the centre column is the near surface, so x_c = x_max - r.
        neck = c & (Z >= zt - 0.049) & (Z < zt - 0.019)
        if int(neck.sum()) < 40:                       # neck occluded: widen
            neck = c & (Z >= zt - 0.065) & (Z < zt - 0.010)
        if int(neck.sum()) < 25:                       # still thin: whole silhouette
            neck = c
        ys = Y[neck]
        yc = 0.5 * (float(ys.min()) + float(ys.max()))
        r = 0.25 * (float(ys.max()) - float(ys.min()))
        col = neck & (np.abs(Y - yc) < 0.004)
        if int(col.sum()) < 3:
            col = neck
        xc = float(X[col].max()) - 2.0 * r
        if not (np.isfinite(xc) and np.isfinite(yc)):
            return None
        api.log("bottle STANDING px=%d ztop=%.4f xc=%.4f yc=%.4f rneck=%.4f"
                % (bn, zt, xc, yc, r))
        return {"kind": "stand", "x": xc, "y": yc, "ztop": zt,
                "close": np.array([0.0, 1.0])}
    # lying: principal axis of the footprint, grasp across the wide (body) end
    px, py = X[c], Y[c]
    mu = np.array([px.mean(), py.mean()])
    q = np.stack([px - mu[0], py - mu[1]], 1)
    evals, evecs = np.linalg.eigh(np.cov(q.T))
    ax = evecs[:, 1]
    perp = np.array([-ax[1], ax[0]])
    t = q @ ax
    s = q @ perp
    best_t, best_w = 0.0, -1.0
    for t0 in np.arange(t.min(), t.max(), 0.01):
        b = (t >= t0) & (t < t0 + 0.02)
        if int(b.sum()) < 20:
            continue
        w = float(s[b].max() - s[b].min())
        if w > best_w:
            best_w, best_t = w, float(t0) + 0.01
    g = mu + ax * best_t
    api.log("bottle LYING px=%d ztop=%.4f axis=%s grasp=%s width=%.4f"
            % (bn, zt, ax.round(3).tolist(), g.round(4).tolist(), best_w))
    return {"kind": "lie", "x": float(g[0]), "y": float(g[1]), "ztop": zt,
            "close": perp / float(np.linalg.norm(perp))}


def find_upper_bed(api, f, B):
    """Highest of the rack's two slat beds -> (plane coefs, x_lo, x_hi, y_lo, y_hi)."""
    C = np.asarray(f.rgb).astype(int)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    wood = (C[..., 0] > WOOD_R_MIN) & (C[..., 0] - C[..., 2] > WOOD_RB)
    m = (wood & _ws(X, Y, Z) & (Y < -0.05) & (Z > TABLE_Z + 0.06)
         & (Z < TABLE_Z + 0.45))
    if int(m.sum()) < 800:
        return None
    w = (Z + BED_SLOPE * Y)[m]
    hist, edges = np.histogram(w, bins=120)
    top = max(i for i in range(len(hist)) if hist[i] > 0.15 * hist.max())
    w0 = 0.5 * (edges[top] + edges[top + 1])
    sel = m & (np.abs(Z + BED_SLOPE * Y - w0) < BED_HALF)
    if int(sel.sum()) < 500:
        return None
    px, py, pz = X[sel], Y[sel], Z[sel]
    coef = np.linalg.lstsq(np.c_[px, py, np.ones(px.size)], pz, rcond=None)[0]
    xlo, xhi = np.percentile(px, [2, 98])
    ylo, yhi = np.percentile(py, [2, 98])
    api.log("bed n=%d plane=%.4f,%.4f,%.4f x=%.3f..%.3f y=%.3f..%.3f"
            % (int(sel.sum()), coef[0], coef[1], coef[2], xlo, xhi, ylo, yhi))
    return coef, float(xlo), float(xhi), float(ylo), float(yhi)


def on_bed(api, f, B, bed):
    """Is the bottle resting on the upper bed now?"""
    coef, xlo, xhi, ylo, yhi = bed
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    c, bn = _bottle_mask(f, B, TABLE_Z + 0.50)
    if c is None:
        return False
    zb = float(np.median(Z[c]))
    xb, yb = float(np.median(X[c])), float(np.median(Y[c]))
    plane = float(coef[0] * xb + coef[1] * yb + coef[2])
    ok = (zb > plane - 0.03) and (xlo - 0.05 < xb < xhi + 0.05) and (ylo - 0.05 < yb < yhi + 0.05)
    api.log("check bottle med=(%.3f,%.3f,%.3f) plane=%.3f -> on_bed=%s"
            % (xb, yb, zb, plane, ok))
    return ok


# --------------------------------------------------------------------------- motion
def goto(api, tag, xyz, seconds=1.0, refine=2, tol=0.014):
    """`tol` must sit above the controller's POS_TOL: below it every move is
    'unconverged' and each refine burns its full 60-step cap for nothing."""
    _spend(max(40, 120 * seconds))
    r = api.move(xyz, seconds=seconds)
    for _ in range(refine):
        if r < tol or _left() < 120:
            break
        _spend(60)
        r = api.move(xyz, seconds=0.5)
    api.log("%s want=%s got=%s resid=%.4f"
            % (tag, [round(float(v), 4) for v in xyz],
               np.asarray(api.eef()).round(4).tolist(), r))
    return r


def set_yaw(api, close, xyz):
    """Point the jaw-closing axis along `close` (horizontal unit vector)."""
    d = np.asarray(close, float)
    R = np.array([[-d[1], d[0], 0.0],
                  [d[0], d[1], 0.0],
                  [0.0, 0.0, -1.0]])
    _spend(180)
    api.move(xyz, rotation=R, seconds=2.0)
    _spend(60)
    api.move(xyz, rotation=R, seconds=1.0)
    api.log("yaw set want_close=%s tool=%s eef=%s"
            % (d.round(3).tolist(), np.asarray(api.tool_rotation()).round(3).tolist(),
               np.asarray(api.eef()).round(4).tolist()))


def pick(api, bot):
    """Grasp the bottle; returns the EEF z at the close, or None."""
    if bot["kind"] == "stand":
        _spend(20)
        api.grip(0.08)
        goto(api, "hover", [bot["x"], bot["y"], bot["ztop"] + 0.06],
             seconds=1.2, tol=0.022)
        goto(api, "down", [bot["x"], bot["y"], bot["ztop"] - GRASP_DROP],
             seconds=0.6, refine=2)
    else:
        if abs(float(bot["close"][1])) < 0.90:
            set_yaw(api, bot["close"], [bot["x"], bot["y"], TABLE_Z + 0.18])
        _spend(20)
        api.grip(0.08)
        goto(api, "hover", [bot["x"], bot["y"], TABLE_Z + 0.08],
             seconds=1.2, tol=0.022)
        goto(api, "down", [bot["x"], bot["y"], LIE_GRASP_Z], seconds=0.6, refine=2)
    # Cancel the descent's tracking bias: the controller stops as soon as it is
    # within its own 12 mm POS_TOL, and measured on the debug seeds it lands a
    # consistent +9 mm in x.  The finger plates are only ~20 mm wide, so that
    # offset eats most of the margin (a further +15 mm misses 4/4).  One bounded
    # re-command, over-shot by the measured residual, re-centres the pinch.
    tgt = np.array([bot["x"], bot["y"]], float)
    e = np.asarray(api.eef(), float)
    err = tgt - e[:2]
    if 0.004 < float(np.linalg.norm(err)) < 0.045:
        corr = np.clip(err, -0.020, 0.020)
        _spend(60)
        api.move([tgt[0] + corr[0], tgt[1] + corr[1], float(e[2])], seconds=0.5)
        api.log("recentre err=%s -> eef=%s"
                % (err.round(4).tolist(), np.asarray(api.eef()).round(4).tolist()))
    e_close = float(np.asarray(api.eef())[2])
    _spend(20)
    api.grip(0.0)
    g = api.gripper()
    api.log("closed grip=%s e_close=%.4f" % (g, e_close))
    if g["effort"] < 1.0 or g["width_m"] < HELD_MIN_W:
        return None
    return e_close


def transit_z(bed, hang):
    """Clear the bed's own high edge with the hanging bottle."""
    coef, xlo, xhi, ylo, yhi = bed
    top = max(float(coef[0] * x + coef[1] * ylo + coef[2]) for x in (xlo, xhi))
    return float(np.clip(top + hang + TRANSIT_CLEAR, 1.33, 1.40))


def place(api, bed, hang, attempt=0):
    coef, xlo, xhi, ylo, yhi = bed
    tz = transit_z(bed, hang)
    xt = float(np.clip(0.5 * (xlo + xhi), -0.06, 0.06))
    yt = 0.5 * (ylo + yhi) - 0.025 * attempt
    zbed = float(coef[0] * xt + coef[1] * yt + coef[2])
    api.log("target x=%.4f y=%.4f zbed=%.4f hang=%.4f" % (xt, yt, zbed, hang))
    goto(api, "over", [xt, yt, tz], seconds=1.0, tol=0.020)
    goto(api, "place", [xt, yt, zbed + hang + PLACE_CLEAR], seconds=0.8, refine=1)
    api.log("at-place eef=%s grip=%s"
            % (np.asarray(api.eef()).round(4).tolist(), api.gripper()))
    _spend(20)
    api.grip(0.08)
    _spend(60)
    api.settle(1.0)
    goto(api, "retreat", [xt, yt, tz], seconds=0.6, refine=0)
    _spend(30)
    api.settle(0.5)


def run(api):
    f = api.capture("cam_high")
    B = _cloud(f)
    bed = find_upper_bed(api, f, B)
    if bed is None:
        api.log("NO BED")
        return
    holding, hang = False, 0.0
    for attempt in range(3):
        if _left() < 260:
            api.log("budget spent (%d used); stopping" % BUDGET["used"])
            return
        if not holding:
            if attempt:
                f = api.capture("cam_high")
                B = _cloud(f)
            bot = find_bottle(api, f, B)
            if bot is None:
                api.log("attempt %d: NO BOTTLE" % attempt)
                return
            api.log("--- attempt %d kind=%s used=%d" % (attempt, bot["kind"], BUDGET["used"]))
            e_close = pick(api, bot)
            if e_close is None:
                api.log("attempt %d: grasp missed" % attempt)
                _spend(20)
                api.grip(0.08)
                goto(api, "recover", [bot["x"], bot["y"], TABLE_Z + 0.22],
                     seconds=1.0, tol=0.030)
                continue
            hang = e_close - TABLE_Z
            tz = transit_z(bed, hang)
            goto(api, "lift",
                 [float(np.clip(bot["x"], -0.10, 0.07)), bot["y"], tz],
                 seconds=1.2, tol=0.060)
            if api.gripper()["effort"] < 1.0:
                api.log("attempt %d: dropped on the lift" % attempt)
                continue
            holding = True
        place(api, bed, hang, attempt)
        holding = False
        f = api.capture("cam_high")
        B = _cloud(f)
        if on_bed(api, f, B, bed):
            api.log("attempt %d: bottle is on the bed (used=%d)" % (attempt, BUDGET["used"]))
            return
        api.log("attempt %d: not on the bed, retrying (used=%d)" % (attempt, BUDGET["used"]))
