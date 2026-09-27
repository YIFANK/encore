"""c2k1clean / spa_bowl_on_ramekin_pos_k1 -- v5.

Task: pick up the black bowl on the ramekin and place it on the plate.

Scene (cam_high RGB-D, measured on debug seeds 51-65):
  table plane 0.9020; the plate is the light disc (lum ~153) 0.036 m proud;
  the two bowls are the dark hollow discs (lum 104-120) 0.110 m wide; the one
  on the ramekin stands 0.046 m higher than the one on the table, which is the
  only cue that names the target.

Grasp: the jaws span 0.072 m, much wider than the bowl wall, so the bowl is
taken by a wall straddle -- the inner finger drops into the bowl, the outer
finger passes outside it, and the close pinches the near wall.  v4 measured the
fingertips 0.0069 m below api.eef() (open jaws pressed on bare table) and found
a shallow straddle (0.015 m below the rim) grips but loses the bowl on the
lift, while a deep one (0.032 m) holds.  v5 therefore tries the deep straddle
first and keeps the shallower/wider ones as fallbacks, re-perceiving the bowl
between rungs because a dropped bowl moves.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z_FALLBACK": {
        "source": "debug-seed measurement (v1/v2, seeds 51-65): the cam_high "
                  "depth mode is 0.9020 on every seed", "allowed": True},
    "OBJ_BAND": {
        "source": "debug-seed measurement (v2): plate top +0.036 m, bowl tops "
                  "+0.050/+0.097 m above the table, so a +5 mm cut separates "
                  "props from the table", "allowed": True},
    "BOWL_LUM": {
        "source": "debug-seed measurement (v2): bowl clusters read lum "
                  "104-120, the plate 151-153, the stove burner 26-27",
        "allowed": True},
    "BOWL_EXT": {
        "source": "debug-seed measurement (v2): both bowls are 0.106-0.111 m "
                  "wide, the plate 0.136 m", "allowed": True},
    "TIP_OFF": {
        "source": "debug-seed measurement (v4, seeds 51/53/55/57): pressing "
                  "the open jaws onto bare table stops with api.eef() 0.0069 m "
                  "above the table plane", "allowed": True},
    "STRADDLE_DEPTH": {
        "source": "debug-seed measurement (v4): a fingertip depth of 0.032 m "
                  "below the rim survives the lift; 0.015 m grips but drops",
        "allowed": True},
    "STRADDLE_DELTA": {
        "source": "debug-seed measurement (v4): a radial ee offset of 0.045 m "
                  "puts one finger inside the 0.054 m bowl and one outside",
        "allowed": True},
    "TOL_BIAS": {
        "source": "generic controller mechanics: heron's LIBERO move_cartesian "
                  "stops inside a 12 mm ball, so descents are commanded 12 mm "
                  "low", "allowed": True},
    "CARRY_Z": {"source": "pack.json demos[0].ee_path6[8][2] = 1.1345, the "
                          "demo's transport altitude", "allowed": True},
    "RELEASE_OFFSET": {
        "source": "pack.json: the open keyframe's ee z 0.9554 is 0.017 m above "
                  "the measured plate top 0.9384", "allowed": True},
    "OPEN_WIDTH": {"source": "pack.json demos[0].keyframes[0].gripper_state "
                             "-> 0.072 m open jaw span", "allowed": True},
    "HELD_GAP": {"source": "generic gripper mechanics: a closed jaw still "
                           "reporting a gap has something in it; the pack's "
                           "holding gap is 0.0072 m", "allowed": True},
    "WORKSPACE_BOX": {"source": "envelope of pack.json demos[0].ee_path6 xy, "
                                "padded; crops the depth cloud", "allowed": True},
}

TABLE_Z_FALLBACK = 0.9020
OBJ_BAND = 0.005
TIP_OFF = 0.0069
TOL_BIAS = 0.012
CARRY_Z = 1.1345
RELEASE_OFFSET = 0.017
OPEN_WIDTH = 0.072
HELD_GAP = 0.005
LADDER = [(0.045, 0.032, +1.0), (0.045, 0.032, -1.0),
          (0.045, 0.015, -1.0), (0.045, 0.045, +1.0),
          (0.060, 0.032, -1.0), (0.060, 0.032, +1.0)]


# -- perception -------------------------------------------------------------

def cloud(frame):
    z = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    H, W = z.shape[:2]
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    valid = np.isfinite(z) & (z > 0)
    zz = np.where(valid, z, 1.0)
    x = (us - K[0, 2]) * zz / K[0, 0]
    y = (vs - K[1, 2]) * zz / K[1, 1]
    pts = np.stack([x, y, zz, np.ones_like(zz)], axis=-1) @ T.T
    return pts[..., :3], valid


def components(mask, min_cells=3):
    lab = -np.ones(mask.shape, int)
    out = []
    for seed in np.argwhere(mask):
        if lab[seed[0], seed[1]] >= 0:
            continue
        stack = [tuple(seed)]
        lab[seed[0], seed[1]] = len(out)
        cells = []
        while stack:
            r, c = stack.pop()
            cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < mask.shape[0] and 0 <= cc < mask.shape[1] \
                            and mask[rr, cc] and lab[rr, cc] < 0:
                        lab[rr, cc] = len(out)
                        stack.append((rr, cc))
        out.append(cells)
    return [c for c in out if len(c) >= min_cells]


def segment(api, cell=0.008):
    f = api.capture("cam_high")
    pts, valid = cloud(f)
    rgb = np.asarray(f.rgb, float)
    X, Y, Z = pts[..., 0], pts[..., 1], pts[..., 2]
    box = valid & (X > -0.45) & (X < 0.40) & (Y > -0.45) & (Y < 0.45) \
        & (Z > 0.5) & (Z < 1.6)
    hist, edges = np.histogram(Z[box], bins=np.arange(0.5, 1.6, 0.004))
    table_z = float(edges[int(np.argmax(hist))] + 0.002)
    if not (TABLE_Z_FALLBACK - 0.05 < table_z < TABLE_Z_FALLBACK + 0.05):
        table_z = TABLE_Z_FALLBACK
    above = box & (Z > table_z + OBJ_BAND)
    gx = np.floor((X + 0.5) / cell).astype(int)
    gy = np.floor((Y + 0.5) / cell).astype(int)
    n = int(1.0 / cell) + 2
    occ = np.zeros((n, n), bool)
    gi, gj = gx[above], gy[above]
    keep = (gi >= 0) & (gi < n) & (gj >= 0) & (gj < n)
    occ[gi[keep], gj[keep]] = True
    key = gx * (n + 5) + gy
    clusters = []
    for cells in components(occ):
        want = np.array([c[0] * (n + 5) + c[1] for c in cells])
        m = above & np.isin(key, want)
        if m.sum() < 60:
            continue
        top = float(np.percentile(Z[m], 99))
        cx = 0.5 * (float(X[m].min()) + float(X[m].max()))
        cy = 0.5 * (float(Y[m].min()) + float(Y[m].max()))
        r_body = 0.25 * ((float(X[m].max()) - float(X[m].min()))
                         + (float(Y[m].max()) - float(Y[m].min())))
        inner = m & (np.hypot(X - cx, Y - cy) < 0.5 * max(r_body, 1e-3))
        z_in = float(np.median(Z[inner])) if inner.sum() > 20 else top
        col = rgb[m].mean(axis=0)
        clusters.append(dict(
            cx=cx, cy=cy, top=top, h=top - table_z, n=int(m.sum()),
            r_body=r_body, ext=max(float(X[m].max() - X[m].min()),
                                   float(Y[m].max() - Y[m].min())),
            hollow=top - z_in, lum=float(col.mean())))
    clusters.sort(key=lambda c: -c["n"])
    return table_z, clusters


def is_bowl(c):
    return (0.085 < c["ext"] < 0.140 and 0.030 < c["h"] < 0.130
            and 80 < c["lum"] < 140 and c["hollow"] > 0.020)


def is_plate(c):
    return (0.100 < c["ext"] < 0.180 and 0.010 < c["h"] < 0.060
            and c["lum"] > 140)


def dump(api, clusters, tag=""):
    for i, c in enumerate(clusters):
        api.log("%sC%d xy=(%.3f,%.3f) top=%.4f h=%.3f ext=%.3f n=%d r=%.3f "
                "hollow=%.3f lum=%.0f" % (tag, i, c["cx"], c["cy"], c["top"],
                                          c["h"], c["ext"], c["n"],
                                          c["r_body"], c["hollow"], c["lum"]))


def held(api):
    g = api.gripper()
    return (g["effort"] > 1.0 and g["width_m"] > HELD_GAP), g


# -- policy -----------------------------------------------------------------

def run(api):
    api.log("instruction: " + api.instruction())
    R0 = np.asarray(api.tool_rotation(), float)
    ax = R0[:2, 1]
    ax = ax / max(float(np.linalg.norm(ax)), 1e-6)

    table_z, clusters = segment(api)
    dump(api, clusters)
    bowls = [c for c in clusters if is_bowl(c)]
    plates = [c for c in clusters if is_plate(c)]
    api.log("TABLE_Z %.4f bowls=%d plates=%d AX %s"
            % (table_z, len(bowls), len(plates), np.round(ax, 3).tolist()))
    if not bowls or not plates:
        api.log("ABORT missing candidate")
        return "no candidate"

    target = max(bowls, key=lambda c: c["top"])
    plate = max(plates, key=lambda c: c["n"])
    home = (target["cx"], target["cy"])
    api.log("TARGET (%.3f,%.3f) top %.4f r %.3f | PLATE (%.3f,%.3f) top %.4f"
            % (target["cx"], target["cy"], target["top"], target["r_body"],
               plate["cx"], plate["cy"], plate["top"]))

    api.grip(OPEN_WIDTH)
    grabbed = False
    hold_sgn, hold_delta = 1.0, 0.045

    for rung, (delta, depth, sgn) in enumerate(LADDER):
        props = [c for c in clusters if c["h"] < 0.30 and c["n"] > 500]
        gx = target["cx"] + sgn * ax[0] * delta
        gy = target["cy"] + sgn * ax[1] * delta
        outer = (target["cx"] + sgn * ax[0] * (delta + 0.039),
                 target["cy"] + sgn * ax[1] * (delta + 0.039))
        clr = min([np.hypot(c["cx"] - outer[0], c["cy"] - outer[1])
                   for c in props
                   if np.hypot(c["cx"] - target["cx"],
                               c["cy"] - target["cy"]) > 0.02] or [1.0])
        rim = target["top"]
        z_cmd = rim - depth + TIP_OFF - TOL_BIAS
        api.log("TRY%d d=%.3f dep=%.3f sgn%+.0f ee=(%.3f,%.3f) z=%.4f clr=%.3f"
                % (rung, delta, depth, sgn, gx, gy, z_cmd, clr))
        if clr < 0.070:
            api.log("  skip: neighbour under the outer finger")
            continue
        api.move(np.array([gx, gy, rim + TIP_OFF + 0.06]), seconds=1.2)
        res = api.move(np.array([gx, gy, z_cmd]), seconds=0.8)
        api.log("  descend res=%.4f eef=%s" % (res, np.round(api.eef(), 4).tolist()))
        api.grip(0.0)
        api.settle(0.2)
        ok, g = held(api)
        api.log("  close held=%s gap=%.4f" % (ok, g["width_m"]))
        if ok:
            api.move(np.array([gx, gy, CARRY_Z]), seconds=1.5)
            ok, g = held(api)
            api.log("  lift held=%s gap=%.4f" % (ok, g["width_m"]))
            if ok:
                grabbed, hold_sgn, hold_delta = True, sgn, delta
                break
        api.grip(OPEN_WIDTH)
        api.move(np.array([gx, gy, rim + TIP_OFF + 0.09]), seconds=1.0)
        # a dropped bowl moves: re-find it near where it was
        table_z, clusters = segment(api)
        cands = [c for c in clusters if is_bowl(c)
                 and np.hypot(c["cx"] - home[0], c["cy"] - home[1]) < 0.12]
        if cands:
            target = min(cands, key=lambda c: np.hypot(c["cx"] - home[0],
                                                       c["cy"] - home[1]))
            api.log("  re-target (%.3f,%.3f) top %.4f"
                    % (target["cx"], target["cy"], target["top"]))

    if not grabbed:
        api.log("FAIL no grasp")
        return "no grasp"

    px = plate["cx"] + hold_sgn * ax[0] * hold_delta
    py = plate["cy"] + hold_sgn * ax[1] * hold_delta
    api.move(np.array([px, py, CARRY_Z]), seconds=2.0)
    rel_z = plate["top"] + RELEASE_OFFSET + TIP_OFF
    res = api.move(np.array([px, py, rel_z]), seconds=1.2)
    api.log("PLACE rel_z %.4f res %.4f eef %s"
            % (rel_z, res, np.round(api.eef(), 4).tolist()))
    api.grip(OPEN_WIDTH)
    api.settle(0.5)
    api.move(np.array([px, py, CARRY_Z]), seconds=1.2)
    api.settle(0.3)

    tz2, cl2 = segment(api)
    dump(api, cl2, tag="END ")
    on_plate = [c for c in cl2 if is_bowl(c)
                and np.hypot(c["cx"] - plate["cx"], c["cy"] - plate["cy"]) < 0.05]
    api.log("VERIFY bowl-on-plate=%d" % len(on_plate))
    return "placed"
