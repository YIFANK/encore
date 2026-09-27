"""c2clean spa_bowl_on_cookie_box_pos_k3 -- v7

Mechanism (from the pack's K=3 demos): straight-down wrist; descend onto the
target bowl's rim, close, lift, translate to the plate, descend, open.

Everything positional is perceived from cam_high in this episode; the pack
supplies only the mechanism plus two offsets (grasp depth below the rim, and
the lateral rim offset recovered by projecting the demo EEF into the demo
keyframe images).
"""
import numpy as np

PROVENANCE = {
    "GRASP_RIM_OFFSET_Y": {
        "source": "pack demo0: EEF at gripper-close (0.0687, 0.0642) minus the "
                  "demo bowl centre (0.0808, 0.0237) recovered by ray-casting the "
                  "bowl-rim centre pixel of keyframes/demo0_t0000.png onto the "
                  "measured rim-top plane -> +0.0405 m along +y (the gripper's "
                  "closing axis)",
        "allowed": True},
    "GRASP_DEPTH_BELOW_RIM": {
        "source": "pack demo0 keyframe t=55 (gripper_state closed): EEF z 0.9530 "
                  "vs the rim-top z 0.971 measured on debug seeds 51-65 -> 0.018 m. "
                  "v3-v6 debug runs: api.move stops within POS_TOL and landed ~7 mm "
                  "high of that, biting the rim's thin top edge (seed 61 then slid "
                  "0.026 m through the jaws mid-lift), so the command is deepened "
                  "to 0.028 to realise the demo's own bite",
        "allowed": True},
    "TABLE_BAND_LO": {
        "source": "debug seeds 51-65 cam_high depth: table plane z = 0.901, cookie "
                  "box top 0.920, bowl rims 0.952/0.971 -> a 0.030 m band floor "
                  "separates rims from the box",
        "allowed": True},
    "BOWL_FOOTPRINT_M": {
        "source": "debug seeds 51-65: both large bowls measure 0.110 m across in "
                  "the top-down height map",
        "allowed": True},
    "RED_BOX_THRESH": {
        "source": "debug seeds 51-65 cam_high RGB: the cookie box's checker red is "
                  "the only r>115, g<95, b<95, r-g>45 material in the table band "
                  "(9-23 px every seed, centroid 0.045 m from the target bowl)",
        "allowed": True},
    "PLATE_RISE": {
        "source": "debug seeds 51-65: cabinet top plane z = 1.1268, the plate on it "
                  "rises to 1.148 -> a 0.006 m rise threshold isolates the plate",
        "allowed": True},
    "PLATE_BRIGHT": {
        "source": "debug seeds 51-65 cam_high RGB: plate mean 134 vs cabinet top 82",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug-seed measurement: cabinet top 1.127, plate top 1.148, and "
                  "the bowl hangs 0.033 m below the EEF (rim-top 0.971 minus bowl "
                  "base 0.920 minus the 0.018 grasp depth) -> carry at 1.215",
        "allowed": True},
    "RELEASE_ABOVE_PLATE": {
        "source": "pack demos release with the EEF 0.016-0.038 m above the plate's "
                  "top surface; 0.030 m puts the measured bowl base (EEF-0.033) "
                  "on the plate",
        "allowed": True},
    "HELD_BOWL_BAND": {
        "source": "debug-seed measurement: bowl rim-top 0.971 minus base 0.920 = "
                  "0.051 m tall, grasped 0.018 below its rim -> in flight it "
                  "occupies EEF-0.015 down to EEF-0.055",
        "allowed": True},
    "TRAIL_GUARD": {
        "source": "v4 debug run: seed 61 slipped in the jaws during the lift "
                  "(gap 0.0080 -> 0.0041) and missed the plate; the other 7 seeds "
                  "kept the nominal offset, so a re-measured trail is trusted only "
                  "within 0.030 m of it",
        "allowed": True},
    "BOWL_HANG": {
        "source": "debug seeds 51-65: bowl rim-top 0.971, base (= cookie box top) "
                  "0.920, grasped 0.018 below the rim -> the base hangs 0.033 m "
                  "below the EEF; re-measured in flight each episode",
        "allowed": True},
    "HANG_GUARD": {
        "source": "v5 debug run: seed 61's bowl sat ~0.02 m higher in the jaws "
                  "(gap 0.0041 vs 0.008, in-flight silhouette 0.108 wide vs 0.085) "
                  "and was dropped from too high; an in-flight hang re-measurement "
                  "is trusted within 0.030 m of the nominal",
        "allowed": True},
    "PLACE_TRAIL_CORRECTION": {
        "source": "v3 debug run (seed 51 gif + log): released at EEF y -0.276 with "
                  "the plate centre at -0.274, and the bowl landed on the plate's "
                  "-y edge -> the rim-pinched bowl centre trails the EEF by the "
                  "grasp offset, so the place aim carries the same +0.040 m",
        "allowed": True},
}

GRASP_RIM_OFFSET_Y = 0.040
GRASP_DEPTH_BELOW_RIM = 0.028
TABLE_BAND_LO = 0.030
BOWL_FOOTPRINT_M = (0.075, 0.155)
PLATE_RISE = 0.006
PLATE_BRIGHT = 110.0
CARRY_Z = 1.215
RELEASE_ABOVE_PLATE = 0.030
BOWL_HANG = 0.033

RES = 0.005
X0, Y0 = -0.80, -0.80
NX, NY = 230, 320


def cloud(f):
    rgb = np.asarray(f.rgb).astype(np.float32)
    depth = np.asarray(f.depth).astype(np.float64)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    good = np.isfinite(depth) & (depth > 0.05) & (depth < 5.0)
    dd = np.where(good, depth, 1.0)
    x = (u - K[0, 2]) / K[0, 0] * dd
    y = (v - K[1, 2]) / K[1, 1] * dd
    P = np.stack([x, y, dd, np.ones_like(dd)], -1)
    B = P @ T.T
    B[~good] = np.nan
    return B[..., :3], rgb


def height_map(B, rgb, zlo, zhi):
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    ok = (np.isfinite(Z) & (Z > zlo) & (Z < zhi) & (X > X0) & (X < X0 + NX * RES)
          & (Y > Y0) & (Y < Y0 + NY * RES))
    gi = np.clip(((X - X0) / RES), 0, NX - 1)
    gj = np.clip(((Y - Y0) / RES), 0, NY - 1)
    gi = np.where(ok, gi, 0).astype(np.int32)
    gj = np.where(ok, gj, 0).astype(np.int32)
    hm = np.full((NX, NY), -1.0)
    col = np.zeros((NX, NY, 3))
    order = np.argsort(Z[ok])
    I, J = gi[ok][order], gj[ok][order]
    hm[I, J] = Z[ok][order]
    col[I, J] = rgb[ok][order]
    return hm, col


def components(mask):
    lab = np.zeros(mask.shape, np.int32)
    n = 0
    stack = []
    for i in range(mask.shape[0]):
        for j in range(mask.shape[1]):
            if mask[i, j] and lab[i, j] == 0:
                n += 1
                lab[i, j] = n
                stack.append((i, j))
                while stack:
                    a, b = stack.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if (0 <= p < mask.shape[0] and 0 <= q < mask.shape[1]
                                    and mask[p, q] and lab[p, q] == 0):
                                lab[p, q] = n
                                stack.append((p, q))
    return lab, n


def mode_z(z, lo, hi, bins=120):
    z = z[(z > lo) & (z < hi)]
    if z.size < 50:
        return None
    h, e = np.histogram(z, bins=bins)
    k = int(h.argmax())
    return float(0.5 * (e[k] + e[k + 1]))


def perceive(api):
    f = api.capture("cam_high")
    B, rgb = cloud(f)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    table = mode_z(Z[np.isfinite(Z)], 0.80, 1.00)
    api.log(f"PERC table_z={table:.4f}")

    # -- cookie box: the only strongly red material near the table ----------
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    red = (np.isfinite(Z) & (Z > table - 0.02) & (Z < table + 0.15)
           & (r > 115) & (g < 95) & (b < 95) & (r - g > 45))
    box_xy = None
    if red.sum() >= 5:
        box_xy = (float(np.median(X[red])), float(np.median(Y[red])))
        api.log(f"PERC cookie_box n={int(red.sum())} xy={box_xy} "
                f"ztop={float(Z[red].max()):.4f}")
    else:
        api.log("PERC cookie_box NOT FOUND")

    # -- bowl candidates ---------------------------------------------------
    hm, col = height_map(B, rgb, table + TABLE_BAND_LO, table + 0.14)
    lab, n = components(hm > table + TABLE_BAND_LO)
    cands = []
    for k in range(1, n + 1):
        ii, jj = np.nonzero(lab == k)
        if len(ii) < 60:
            continue
        x = X0 + ii * RES
        y = Y0 + jj * RES
        ex, ey = float(x.max() - x.min()), float(y.max() - y.min())
        if not (BOWL_FOOTPRINT_M[0] <= ex <= BOWL_FOOTPRINT_M[1]
                and BOWL_FOOTPRINT_M[0] <= ey <= BOWL_FOOTPRINT_M[1]):
            continue
        z = hm[ii, jj]
        c = (float(0.5 * (x.min() + x.max())), float(0.5 * (y.min() + y.max())))
        cands.append({"c": c, "top": float(np.percentile(z, 97)),
                      "ext": (ex, ey), "n": len(ii)})
        api.log(f"PERC cand c={c} top={cands[-1]['top']:.4f} ext=({ex:.3f},{ey:.3f}) n={len(ii)}")
    if not cands:
        return None
    if box_xy is not None:
        cands.sort(key=lambda d: (d["c"][0] - box_xy[0]) ** 2 + (d["c"][1] - box_xy[1]) ** 2)
    else:
        cands.sort(key=lambda d: -d["top"])
    bowl = cands[0]
    api.log(f"PERC TARGET bowl c={bowl['c']} top={bowl['top']:.4f}")

    # -- plate: a bright disc raised above whatever surface carries it ------
    plate = None
    hi = np.isfinite(Z) & (Z > table + 0.15) & (Z < table + 0.40) & (Y < -0.10)
    if hi.sum() > 500:
        sup = mode_z(Z[hi], table + 0.15, table + 0.40)
        # the support surface's own footprint bounds the search, so the arm
        # (which is also tall and also bright) cannot masquerade as the plate
        sm = hi & (Z > sup - 0.006) & (Z < sup + 0.006)
        sx, sy = X[sm], Y[sm]
        pm = (hi & (Z > sup + PLATE_RISE) & (Z < sup + 0.06)
              & (rgb.mean(-1) > PLATE_BRIGHT)
              & (X >= sx.min() - 0.02) & (X <= sx.max() + 0.02)
              & (Y >= sy.min() - 0.02) & (Y <= sy.max() + 0.02))
        api.log(f"PERC support_top={sup:.4f} plate_px={int(pm.sum())}")
        if pm.sum() > 300:
            px, py, pz = X[pm], Y[pm], Z[pm]
            diam = float(px.max() - px.min())
            cx = float(0.5 * (px.min() + px.max()))
            # the -y side of the plate runs off the left of the frame, so its
            # y midpoint is biased; the x extent is the true diameter
            ey = float(py.max() - py.min())
            cy = (float(0.5 * (py.min() + py.max())) if ey > 0.92 * diam
                  else float(py.max() - 0.5 * diam))
            plate = {"c": (cx, cy), "top": float(np.percentile(pz, 97)), "diam": diam,
                     "cy_mid": float(0.5 * (py.min() + py.max()))}
            api.log(f"PERC PLATE c={plate['c']} top={plate['top']:.4f} "
                    f"diam={diam:.3f} cy_mid={plate['cy_mid']:.3f}")
    if plate is None:
        # fallback: a bright raised disc sitting on the table itself
        pm = (np.isfinite(Z) & (Z > table + 0.008) & (Z < table + 0.035)
              & (rgb.mean(-1) > PLATE_BRIGHT))
        if pm.sum() > 300:
            px, py, pz = X[pm], Y[pm], Z[pm]
            plate = {"c": (float(px.mean()), float(py.mean())),
                     "top": float(np.percentile(pz, 97)),
                     "diam": float(px.max() - px.min()), "cy_mid": float(py.mean())}
            api.log(f"PERC PLATE(table fallback) {plate}")
    return {"table": table, "bowl": bowl, "plate": plate}


def measure_held(api, e):
    """The bowl in flight: its centre, and how far its base hangs below the EEF.

    Both matter because the bowl can sit differently in the jaws than the demo
    did: a slip moves the centre sideways AND changes the hang, and the hang is
    what decides the release height.
    """
    f = api.capture("cam_high")
    B, rgb = cloud(f)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    m = (np.isfinite(Z) & (Z > e[2] - 0.055) & (Z < e[2] - 0.015)
         & (np.abs(X - e[0]) < 0.14) & (np.abs(Y - e[1]) < 0.14))
    n = int(m.sum())
    if n < 150:
        api.log(f"HELD too few px ({n})")
        return None
    ex = float(X[m].max() - X[m].min())
    ey = float(Y[m].max() - Y[m].min())
    cx = float(0.5 * (X[m].min() + X[m].max()))
    cy = float(0.5 * (Y[m].min() + Y[m].max()))
    # the whole silhouette, for the base: a wider band, but only the bowl's own
    # footprint so the hand above it cannot join in
    w = (np.isfinite(Z) & (Z > e[2] - 0.100) & (Z < e[2] + 0.005)
         & (np.abs(X - cx) < 0.055) & (np.abs(Y - cy) < 0.055))
    zlo = float(np.percentile(Z[w], 2)) if int(w.sum()) > 100 else float("nan")
    ztop = float(np.percentile(Z[m], 98))
    api.log(f"HELD n={n} ext=({ex:.3f},{ey:.3f}) c=({cx:.4f},{cy:.4f}) "
            f"zlo={zlo:.4f} ztop={ztop:.4f} eefz={e[2]:.4f} "
            f"hang={e[2] - zlo:.4f} nw={int(w.sum())}")
    if not (0.07 < ex < 0.16 and 0.07 < ey < 0.16):
        return None
    return (cx, cy, zlo)


def run(api):
    api.log(f"INSTR {api.instruction()}")
    s = perceive(api)
    if s is None:
        api.log("ABORT no bowl candidate")
        return
    bowl, plate = s["bowl"], s["plate"]
    bx, by = bowl["c"]
    top = bowl["top"]
    gx, gy = bx, by + GRASP_RIM_OFFSET_Y

    api.grip(0.08)
    r = api.move([gx, gy, top + 0.09], seconds=3.0)
    api.log(f"MOVE hover res={r:.4f} eef={api.eef()}")
    r = api.move([gx, gy, top - GRASP_DEPTH_BELOW_RIM], seconds=2.5)
    api.log(f"MOVE descend res={r:.4f} eef={api.eef()}")
    api.grip(0.0)
    api.settle(0.3)
    g = api.gripper()
    api.log(f"GRASP gripper={g} eef={api.eef()}")

    r = api.move([gx, gy, CARRY_Z], seconds=3.0)
    api.log(f"MOVE lift res={r:.4f} eef={api.eef()} grip={api.gripper()}")

    # Where is the bowl actually sitting in the jaws? A slip during the close or
    # the lift moves it, and the place aim has to carry the REAL offset.
    trail = (bx - gx, by - gy)
    e = api.eef()
    m = measure_held(api, e)
    hang = BOWL_HANG
    if m is not None:
        h = e[2] - m[2]
        if abs(h - BOWL_HANG) < 0.030:
            hang = h
        cand = (m[0] - e[0], m[1] - e[1])
        d = ((cand[0] - trail[0]) ** 2 + (cand[1] - trail[1]) ** 2) ** 0.5
        api.log(f"HELD measured={m} trail_nom={trail} trail_meas={cand} d={d:.4f}")
        if d < 0.030:
            trail = cand
    api.log(f"TRAIL {trail} HANG {hang:.4f}")

    if plate is None:
        api.log("ABORT no plate")
        return
    # the bowl is held by a rim pinch, so its centre trails the EEF by the
    # same lateral offset the grasp used: aim the EEF that far past the plate.
    px, py = plate["c"][0] - trail[0], plate["c"][1] - trail[1]
    api.log(f"PLACE aim=({px:.4f},{py:.4f}) plate_c={plate['c']}")
    r = api.move([px, py, CARRY_Z], seconds=4.0)
    api.log(f"MOVE overplate res={r:.4f} eef={api.eef()} grip={api.gripper()}")
    # release so the bowl's own measured base lands on the plate; api.move
    # stops within POS_TOL (0.012 m), so aim at contact and let it stop high
    zrel = plate["top"] + hang - 0.004
    api.log(f"ZREL {zrel:.4f} plate_top={plate['top']:.4f} hang={hang:.4f}")
    r = api.move([px, py, zrel], seconds=2.5)
    api.log(f"MOVE descend2 res={r:.4f} eef={api.eef()} grip={api.gripper()}")
    api.grip(0.08)
    api.settle(0.4)
    api.log(f"RELEASE eef={api.eef()} grip={api.gripper()}")
    r = api.move([px, py, CARRY_Z], seconds=2.5)
    api.log(f"MOVE retreat res={r:.4f} eef={api.eef()}")
    r = api.move([px + 0.10, py + 0.12, CARRY_Z], seconds=2.5)
    api.log(f"MOVE clear res={r:.4f} eef={api.eef()}")
    post = perceive(api)
    if post is not None and post["plate"] is not None:
        api.log(f"POST plate={post['plate']['c']} bowl={post['bowl']['c']} "
                f"bowl_top={post['bowl']['top']:.4f}")
