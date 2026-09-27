"""c2 cell goal_bowl_on_plate_stock -- v1.1 (de-oracled v4; mechanism unchanged)

De-oracle pass (fair-v1.1: api.done = episode termination ONLY, never success):
every read of `api.done` is excised. Nothing else is tuned or restructured; all
calibrated constants keep their v4 values.
  * removed success-conditioned early exits: the `api.done` breaks in the grasp
    candidate loop, in `glide`, in the grasp-failed branch, and -- the
    load-bearing one -- the `if api.done: break` that skipped verification
    immediately after a placement.
  * removed the `not api.done` guard on the residual re-command in `precise`.
  * removed `done=%s` from four log lines.
  * the retry gate `if off <= 0.012 or api.done` keeps only its sensor half:
    the program's OWN re-perception of bowl-centre vs plate-centre offset
    (VERIFY_OFF_TOL, named from the existing 0.012 literal). This is the sole
    stop condition for the (unchanged, at-most-2) place cycle.


Intent: "put the bowl on the plate".

Mechanism (re-derived from packs/c2_goal_bowl_on_plate_stock + debug seeds 51-65):
  * cam_high depth -> base-frame cloud -> modal table plane (0.9009 measured).
  * bowl and plate are coaxial discs. Each is localised by a prior-windowed
    ABSOLUTE height band and an algebraic circle fit of its outer edge; the
    bowl's rim fits with ~2 mm rms, and the fit centre is ~27 mm away from the
    naive pixel median (the visible rim is not symmetric in the camera's
    view direction), which is what made v1/v2 place off-centre.
  * the pack closes the gripper to a ~11 mm gap: a RIM pinch. The tool centre
    is driven onto the rim circle along the gripper's finger-opening axis, on
    the +y quadrant (the quadrant all three demos use).
  * a rim pinch carries the BOWL centre at (eef - grasp_offset); the release
    point re-adds that offset over the plate centre. Accuracy matters: a bowl
    landed ~20 mm off the plate centre still scores 0 (seed 63), ~5 mm scores 1.
  * motions glide in <=45 mm waypoints: the OSC saturates on a large position
    error and the resulting lurch shakes the bowl out of the pinch (seeds
    51/53/55/59/61 of v2 all lost the bowl mid-carry).
  * if the pinch is lost, the whole pick-and-place is re-perceived and retried.
"""

import numpy as np

PROVENANCE = {
    "Z_GRASP": {
        "source": "pack.json demos: EEF z at the three close-gripper keyframes "
                  "(0.9252 / 0.9179 / 0.9199) -> 0.921; debug seeds 51-65 confirm a "
                  "catch (finger gap 0.009-0.017, effort 3.0) at that height",
        "allowed": True},
    "Z_LIFT": {
        "source": "pack.json ee_path6 carry apex t=60 (1.0179 / 1.0322 / 1.0277) -> 1.026",
        "allowed": True},
    "Z_RELEASE_OVER_RIM": {
        "source": "pack.json release keyframes (0.9359 / 0.9280 / 0.9309) sit 0.011 above "
                  "the plate rim measured on the debug seeds (0.9197) -> plate_top + 0.011",
        "allowed": True},
    "Z_OVER": {
        "source": "pack.json ee_path6 pre-descent waypoints t=20..30 (z 1.10..0.99) -> 1.05",
        "allowed": True},
    "RIM_SIDE_PLUS_Y": {
        "source": "pack.json: all three demos grasp at y +0.030..+0.044 and release at "
                  "y +0.012..+0.040 -- the +y quadrant of the rim circle",
        "allowed": True},
    "PLACE_RIM_COMPENSATION": {
        "source": "debug seeds 51-65: measured grasp offset (eef_grasp - fitted bowl "
                  "centre) ~0.053 m; v1 released without it (0/4), v2/v3 with it (3/8, 4/8)",
        "allowed": True},
    "BOWL_PRIOR": {
        "source": "pack.json close-gripper keyframes (-0.103..-0.109, y +0.030..+0.044), "
                  "recentred on the bowl centres fitted on debug seeds 51-65 "
                  "(x -0.078..-0.094, y -0.009..+0.014); search seed only",
        "allowed": True},
    "PLATE_PRIOR": {
        "source": "pack.json release keyframes (0.045..0.050, y +0.012..+0.040), recentred "
                  "on the plate centres measured on debug seeds 51-65 (x 0.030..0.053, "
                  "y -0.023..+0.008); search seed only",
        "allowed": True},
    "BOWL_RIM_H": {
        "source": "debug seeds 51-65: the bowl rim stands 0.0507 m above the measured "
                  "table plane in every seed",
        "allowed": True},
    "PLATE_RIM_H": {
        "source": "debug seeds 51-65: the plate rim stands 0.0187 m above the measured "
                  "table plane in every seed",
        "allowed": True},
    "OPEN_W": {
        "source": "generic gripper mechanics: width >= 0.025 m is an open command "
                  "(pack gripper_state at the open keyframes is a ~0.078 m span)",
        "allowed": True},
    "POS_TOL_REFINE": {
        "source": "generic controller mechanics: api.move returns inside its own position "
                  "tolerance, so the residual is re-commanded once, to under 6 mm",
        "allowed": True},
    "GLIDE_STEP": {
        "source": "debug seeds (v2): single long moves saturate the OSC and shake the bowl "
                  "out of the pinch; 0.045 m waypoints keep the commanded error small",
        "allowed": True},
    "VERIFY_OFF_TOL": {
        "source": "unchanged v4 literal (0.012), named here because the de-oracle pass "
                  "makes it the sole stop condition: debug seeds 51-65 (v3) bracket the "
                  "predicate -- seed 55 landed the bowl 0.005 m from the fitted plate "
                  "centre and scored 1, seed 63 landed ~0.020 m off and scored 0",
        "allowed": True},
}

CAM = "cam_high"
Z_GRASP = 0.921
Z_LIFT = 1.026
Z_OVER = 1.05
BOWL_PRIOR = np.array([-0.086, 0.004])
PLATE_PRIOR = np.array([0.044, -0.006])
BOWL_RIM_H = 0.0507
PLATE_RIM_H = 0.0187
OPEN_W = 0.08
CLOSE_W = 0.0
POS_TOL_REFINE = 0.006
GLIDE_STEP = 0.045
VERIFY_OFF_TOL = 0.012


# --------------------------------------------------------------------- vision
def _pc(frame):
    d = np.asarray(frame.depth, float)
    if d.ndim == 3:
        d = d[..., 0]
    H, W = d.shape
    K = np.asarray(frame.intrinsics, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    valid = np.isfinite(d) & (d > 1e-4) & (d < 6.0)
    z = np.where(valid, d, 0.0)
    pts = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z, np.ones_like(z)], axis=-1)
    B = pts @ np.asarray(frame.t_base_cam, float).T
    return B[..., 0], B[..., 1], B[..., 2], valid


def _table_z(X, Y, Z, valid):
    m = (valid & (X > -0.35) & (X < 0.35) & (Y > -0.35) & (Y < 0.35)
         & (Z > 0.60) & (Z < 1.30))
    zz = Z[m]
    if zz.size < 500:
        return None
    hist, edges = np.histogram(zz, bins=np.arange(0.60, 1.30, 0.002))
    k = int(np.argmax(hist))
    band = zz[(zz > edges[k] - 0.004) & (zz < edges[k + 2] + 0.004)]
    return float(np.median(band))


def _circle_fit(x, y):
    A = np.stack([x, y, np.ones_like(x)], axis=1)
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = 0.5 * sol[0], 0.5 * sol[1]
    r = float(np.sqrt(max(sol[2] + cx * cx + cy * cy, 1e-9)))
    rms = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - r) ** 2)))
    return float(cx), float(cy), r, rms


def _mid(v):
    return float(0.5 * (np.percentile(v, 2) + np.percentile(v, 98)))


def locate(api, X, Y, Z, valid, prior, zlo, zhi, win, tag):
    """Absolute height band inside a prior window -> outer-edge circle fit."""
    ws = valid & (X > -0.30) & (X < 0.30) & (Y > -0.30) & (Y < 0.30) & (Z > zlo) & (Z < zhi)
    c = np.asarray(prior, float).copy()
    out = None
    for _ in range(4):
        sel = ws & (np.hypot(X - c[0], Y - c[1]) < win)
        n = int(sel.sum())
        if n < 120:
            api.log("locate %s: only %d px at c=(%.3f,%.3f)" % (tag, n, c[0], c[1]))
            break
        xs, ys = X[sel], Y[sel]
        c0 = np.array([_mid(xs), _mid(ys)])
        rr = np.hypot(xs - c0[0], ys - c0[1])
        edge = rr > np.percentile(rr, 80)
        cx, cy, r, rms = _circle_fit(xs[edge], ys[edge])
        use = np.array([cx, cy]) if (rms < 0.006 and 0.03 < r < 0.10) else c0
        out = {"c": use, "cfit": np.array([cx, cy]), "cmid": c0, "r": r,
               "rms": rms, "n": n, "ztop": float(np.percentile(Z[sel], 97))}
        if float(np.linalg.norm(use - c)) < 0.002:
            c = use
            break
        c = use
    if out is not None:
        api.log("locate %s n=%d c=(%.4f,%.4f) cfit=(%.4f,%.4f) cmid=(%.4f,%.4f) r=%.4f "
                "rms=%.4f ztop=%.4f"
                % (tag, out["n"], out["c"][0], out["c"][1], out["cfit"][0], out["cfit"][1],
                   out["cmid"][0], out["cmid"][1], out["r"], out["rms"], out["ztop"]))
    return out


def see(api, tz, bowl_prior):
    """Locate the bowl (rim height found adaptively -- it stands higher once it
    is on the plate) and the plate (fixed rim band)."""
    frame = api.capture(CAM)
    X, Y, Z, valid = _pc(frame)
    ws = valid & (X > -0.30) & (X < 0.30) & (Y > -0.30) & (Y < 0.30)
    seed = ws & (np.hypot(X - bowl_prior[0], Y - bowl_prior[1]) < 0.085) \
        & (Z > tz + 0.030) & (Z < tz + 0.16)
    bowl = None
    if int(seed.sum()) >= 150:
        ztop = float(np.clip(np.percentile(Z[seed], 97), tz + 0.035, tz + 0.13))
        bowl = locate(api, X, Y, Z, valid, bowl_prior,
                      ztop - 0.011, ztop + 0.006, 0.075, "bowl")
    else:
        api.log("see: bowl seed only %d px" % int(seed.sum()))
    plate = locate(api, X, Y, Z, valid, PLATE_PRIOR,
                   tz + 0.006, tz + PLATE_RIM_H + 0.008, 0.090, "plate")
    return bowl, plate


# -------------------------------------------------------------------- motion
def precise(api, target, seconds=0.8, tol=POS_TOL_REFINE):
    target = np.asarray(target, float)
    api.move(target, seconds=seconds)
    e = target - api.eef()
    res = float(np.linalg.norm(e))
    if res > tol:
        api.move(target + e, seconds=0.4)
        res = float(np.linalg.norm(target - api.eef()))
    return res


def glide(api, target, step=GLIDE_STEP):
    """Walk to target in short hops: a saturated OSC lurch drops the bowl."""
    target = np.asarray(target, float)
    start = api.eef()
    d = float(np.linalg.norm(target - start))
    k = int(np.ceil(d / step))
    for i in range(1, k):
        api.move(start + (target - start) * (i / float(k)), seconds=0.4)
    return precise(api, target, seconds=0.5)


def attempt(api, tz, bowl_prior):
    """One full perceive -> rim pinch -> carry -> release cycle."""
    bowl, plate = see(api, tz, bowl_prior)
    if bowl is None:
        api.log("no bowl seen")
        return None, None
    bc, br = np.asarray(bowl["c"], float), float(np.clip(bowl["r"], 0.045, 0.060))
    pc = np.asarray(plate["c"], float) if plate is not None else PLATE_PRIOR.copy()
    p_top = plate["ztop"] if plate is not None else tz + PLATE_RIM_H
    z_rel = float(np.clip(p_top + 0.011, 0.925, 0.950))

    R = np.asarray(api.tool_rotation(), float)
    ty = R[:2, 1] / max(float(np.linalg.norm(R[:2, 1])), 1e-9)
    tx = R[:2, 0] / max(float(np.linalg.norm(R[:2, 0])), 1e-9)
    cand = [("+y", ty if ty[1] > 0 else -ty),
            ("-y", -ty if ty[1] > 0 else ty),
            ("+x", tx if tx[0] > 0 else -tx)]

    api.grip(OPEN_W)
    held = None
    for nm, a in cand:
        g = bc + br * a
        precise(api, [g[0], g[1], Z_OVER], seconds=1.0)
        precise(api, [g[0], g[1], Z_GRASP], seconds=0.6)
        api.grip(CLOSE_W)
        api.settle(0.2)
        gr = api.gripper()
        e = api.eef()
        api.log("attempt %s tgt=(%.4f,%.4f) eef=(%.4f,%.4f,%.4f) grip=%.4f/%.1f"
                % (nm, g[0], g[1], e[0], e[1], e[2], gr["width_m"], gr["effort"]))
        if gr["effort"] >= 1.0:
            held = gr
            break
        api.grip(OPEN_W)
        precise(api, [g[0], g[1], Z_OVER], seconds=0.6)
    if held is None:
        return None, bc

    eg = api.eef()
    delta = np.array([eg[0] - bc[0], eg[1] - bc[1]])
    tgt = pc + delta
    api.log("HELD gap=%.4f delta=(%.4f,%.4f) plate=(%.4f,%.4f) tgt=(%.4f,%.4f) z_rel=%.4f"
            % (held["width_m"], delta[0], delta[1], pc[0], pc[1], tgt[0], tgt[1], z_rel))
    glide(api, [eg[0], eg[1], Z_LIFT])
    api.log("lifted grip=%s" % (api.gripper(),))
    glide(api, [tgt[0], tgt[1], Z_LIFT])
    gr = api.gripper()
    api.log("carried grip=%s eef=%s"
            % (gr, [round(v, 4) for v in api.eef()]))
    glide(api, [tgt[0], tgt[1], z_rel])
    api.log("at place eef=%s grip=%s"
            % ([round(v, 4) for v in api.eef()], api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.3)
    precise(api, [tgt[0], tgt[1], Z_OVER], seconds=0.8)
    api.settle(0.3)
    return pc, tgt - delta


def run(api):
    api.log("instruction=%r" % api.instruction())
    frame = api.capture(CAM)
    X, Y, Z, valid = _pc(frame)
    tz = _table_z(X, Y, Z, valid)
    api.log("table_z=%s" % (None if tz is None else round(tz, 4)))
    if tz is None:
        tz = 0.9009

    prior = BOWL_PRIOR.copy()
    note = "none"
    for cycle in range(2):
        pc, expect = attempt(api, tz, prior)
        if pc is None:
            note = "grasp-failed"
            if expect is not None:
                prior = expect
            continue
        note = "placed"
        # verify: where did the bowl actually end up?
        b2, p2 = see(api, tz, np.asarray(pc, float))
        if b2 is None:
            api.log("verify: bowl not found above the plate")
            prior = np.asarray(expect, float)
            continue
        fc = np.asarray(b2["c"], float)
        pcc = np.asarray(p2["c"], float) if p2 is not None else pc
        off = float(np.hypot(fc[0] - pcc[0], fc[1] - pcc[1]))
        api.log("VERIFY bowl=(%.4f,%.4f) plate=(%.4f,%.4f) off=%.4f rim_z=%.4f"
                % (fc[0], fc[1], pcc[0], pcc[1], off, b2["ztop"]))
        if off <= VERIFY_OFF_TOL:
            break
        api.log("re-centring: off=%.4f" % off)
        prior = fc
    return note
