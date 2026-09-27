"""c2k1clean / spa_bowl_cookie_box_pos_k1 -- v10.

v3 receipt 2/8 (51,55) -- the first end-to-end successes; the release over the
plate converged to 0.008 m, so the carry geometry is right.  All six failures
were the grasp.  Cause, from the v3 logs: a single descent from the park height
lands ~0.010 m high, and the black bowl's wall radius grows 0.6 mm per mm of
height, so a pinch radius computed for the intended depth falls inside the
cavity and the jaws close on air (width 0.003-0.005, effort 0.05).  The two
closes that gripped had the pinch radius 1-7 mm outside the measured surface.

v10 (v9 = 5/8) generalises the yaw.  In v9 the -90 deg yaw succeeded whenever
the bowl was caught on the first attempt (dR 0.013, release error 0.0075) and
failed with dR exactly 1.000 -- i.e. the wrist did not move at all -- on the two
seeds that needed a grasp retry, whose extra motion leaves the last wrist joint
against its limit.  v10 therefore tries a ladder of yaws (-90, 180, 0) and uses
the release pose that matches whichever one the wrist actually reached; the
ladder is ordered by how near the base the resulting release pose is.  It also
checks the grip at the first lift stage, so a slipping bowl is set back down
instead of being dropped from 0.11 m.

v9 fixes the last two v8 failure modes (v8 = 4/8).
  * The 180 deg yaw is unreliable -- it sits on the wrist's +-180 ambiguity, so
    from an identical park pose seed 51 rotated cleanly (dR 0.028) while seed 61
    did not (dR 1.63) and seed 57 rotated but then swung to y = -0.37 while
    trying to hold that orientation across the table.  v9 yaws -90 deg instead:
    half the wrist excursion, and it swings the bowl to -x of the tool, so the
    release pose becomes plate + pinch_radius in x ~ (-0.155, 0.20) -- nearer
    the base than either the 180 deg target (0.16) or the unyawed one (0.25).
    The yaw is verified and retried, and the release target follows whichever
    orientation was actually achieved.
  * Seeds 61/65 gripped at 0.0054-0.0072 and lost the bowl during the single
    1.8 s lift.  v9 lifts in two slower stages to cut the inertial load on a
    pinch that only holds a sloping 6 mm wall.

v8 = v7's grasp (which held 7/8 on the probe, widths 0.0054-0.0075 at effort
3.0) with the carry re-routed.  v7 scored only 1/8 because the transit stalled:
a straight-line midpoint between the bowl and the plate leaves the arm unable to
pull back in x (it froze around x=+0.05 and was pushed up to z=1.24).  The reach
survey had shown that the plate region *is* reachable when approached from the
central park pose (0.02, 0.00, table+0.30), so v8 routes the transit through it
and yaws the wrist there, in a comfortable configuration, rather than out over
the bowl.

v7 fixes the grasp properly.  Survey s11 showed the descent does not converge:
api.move leaves a steady ~0.010 m proportional droop in z, so re-commanding the
same target changes nothing and the fingertip lands 0.010 higher than asked.
Because the bowl's wall radius grows 0.6 mm per mm of height, a pinch radius
computed for the *intended* depth then falls inside the cavity and the jaws shut
on air -- exactly the 0.0010-0.0048 m closes seen in v4/v6.  s11 also showed
api.grip is binary: grip(0.035) opens the jaws fully, so a partial opening is
not available.

v7 therefore (a) cancels the droop by re-commanding at ztgt - gap, and (b) after
landing, measures the fingertip height that was actually achieved, looks the
wall radius up at *that* height in the cloud it already has, and slides the tool
radially to mid-wall (measured radius + 0.004) before closing.  The radial slide
is free: with the jaws open the inner finger sits ~0.010 from the bowl axis and
the outer ~0.088, both clear of the wall.

v6 keeps v4's grasp (6/8 holds) and rebuilds the carry.  v4 and v5 both lost
seeds at the plate: the release pose is plate_y + pinch_radius ~ 0.26, which is
at the edge of the arm's envelope (converged on plate_y 0.187-0.193, stalled
with 0.19-0.21 m of error on plate_y 0.206-0.211).  Since the bowl hangs one
pinch radius to -y of the tool, v6 yaws the wrist 180 deg about world z once the
bowl is up, which swings the bowl to the other side of the tool and moves the
release pose to plate_y - pinch_radius ~ 0.16, well inside the envelope.

v4: (a) iterate the descent to a 0.004 m tolerance so the fingertip lands where
the radius was measured; (b) fix the side-ranking, which compared object
identity against a rebuilt dict and so always measured the target against its
own footprint (every gap came out negative and both sides were rejected);
(c) pinch 0.007 m outside the measured wall and re-squeeze after the lift;
(d) carry at table+0.20 instead of +0.30 to halve the time under load, because
a marginal grip creeps open (seed 57 held at +0.12 and had lost it by +0.30).
"""
import numpy as np

PROVENANCE = {
    "XLO_XHI_YLO_YHI": {
        "source": "debug-seed measurement (cam_high depth extent of the table)",
        "allowed": True},
    "TABLE_MODE_BINS": {
        "source": "generic perception mechanics: depth histogram mode = table plane",
        "allowed": True},
    "FINGER_DZ": {
        "source": "debug-seed 51 contact ladder: descent stalls at eef z = table_z "
                  "+ 0.0231 with the jaws open",
        "allowed": True},
    "GRASP_DZ": {
        "source": "pack.json demos[0].keyframes[1].ee[2] 0.9263 - debug-seed table_z "
                  "0.9010 = 0.025; debug seed 53 held the wall at eef rel 0.0353",
        "allowed": True},
    "HOLD_WIDTH_BAND": {
        "source": "pack.json demos[0].keyframes[2].gripper_state = 8.6 mm held wall; "
                  "debug seed 53 measured 8.8 mm -> band 0.004-0.030",
        "allowed": True},
    "JAW_HALF_OPEN": {
        "source": "debug-seed api.gripper() width 0.0778 with the jaws open",
        "allowed": True},
    "PARK_DZ": {
        "source": "debug-seed reach survey: every probed xy is reachable at "
                  "table_z+0.30 and descends cleanly from there",
        "allowed": True},
    "BOWL_SHAPE_GATE": {
        "source": "debug-seed 51/53/55 clusters: bowls h 0.042-0.051, Kasa rim "
                  "radius 0.041-0.053, rstd < 0.010",
        "allowed": True},
    "PLATE_SHAPE_GATE": {
        "source": "debug-seed 51/53/55 clusters: plate h 0.019, r 0.059, span 0.136",
        "allowed": True},
    "BOX_COLOUR_GATE": {
        "source": "debug-seed 51/55 cookie-box top colour (95,59,32): R-B 63, R-G 36",
        "allowed": True},
    "STALL_TOL": {
        "source": "debug-seed reach survey: a clean 0.25 m descent lands within "
                  "0.012 of the command; a blocked one is >= 0.10 short",
        "allowed": True},
}

XLO, XHI, YLO, YHI = -0.33, 0.32, -0.42, 0.42
FINGER_DZ = 0.023
GRASP_DZ = 0.030
HOLD_LO, HOLD_HI = 0.004, 0.030
JAW_HALF = 0.039
PARK_DZ = 0.30
STALL_TOL = 0.030
OPEN_W = 0.08


# --------------------------------------------------------------- perception
def cloud(frame):
    d = np.asarray(frame.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:h, 0:w]
    pc = np.stack([(uu - cx) * d / fx, (vv - cy) * d / fy, d], -1)
    return pc @ T[:3, :3].T + T[:3, 3]


def table_z(W):
    X, Y, Z = W[..., 0], W[..., 1], W[..., 2]
    m = np.isfinite(Z) & (X > XLO) & (X < XHI) & (Y > YLO) & (Y < YHI)
    hist, edges = np.histogram(Z[m], bins=400, range=(0.5, 1.3))
    i = int(np.argmax(hist))
    return 0.5 * (edges[i] + edges[i + 1])


def _label(occ):
    NX, NY = occ.shape
    lab = np.zeros((NX, NY), int)
    cur = 0
    for a in range(NX):
        for b in range(NY):
            if occ[a, b] and not lab[a, b]:
                cur += 1
                st = [(a, b)]
                lab[a, b] = cur
                while st:
                    p, q = st.pop()
                    for dp in (-1, 0, 1):
                        for dq in (-1, 0, 1):
                            r, s = p + dp, q + dq
                            if 0 <= r < NX and 0 <= s < NY and occ[r, s] and not lab[r, s]:
                                lab[r, s] = cur
                                st.append((r, s))
    return lab, cur


def segment(W, mask, res=0.01, minn=60):
    X, Y = W[..., 0], W[..., 1]
    NX = int((XHI - XLO) / res) + 2
    NY = int((YHI - YLO) / res) + 2
    xi = np.clip((X - XLO) / res, -1, NX - 1).astype(int)
    yi = np.clip((Y - YLO) / res, -1, NY - 1).astype(int)
    m = mask & (xi >= 0) & (yi >= 0)
    occ = np.zeros((NX, NY), bool)
    occ[xi[m], yi[m]] = True
    lab, cur = _label(occ)
    ids = np.zeros(X.shape, int)
    ids[m] = lab[xi[m], yi[m]]
    return [m & (ids == c) for c in range(1, cur + 1) if (m & (ids == c)).sum() >= minn]


def kasa(x, y):
    A = np.stack([x, y, np.ones_like(x)], 1)
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    return float(cx), float(cy), float(np.sqrt(max(sol[2] + cx * cx + cy * cy, 1e-9)))


def stats(W, rgb, sel, zt):
    X, Y, Z = W[..., 0], W[..., 1], W[..., 2]
    x, y, z = X[sel], Y[sel], Z[sel]
    col = rgb[sel].astype(float)
    ztop = float(np.percentile(z, 99.0))
    band = z > ztop - 0.010
    if band.sum() < 12:
        band = np.ones_like(z, bool)
    cx, cy, r = kasa(x[band], y[band])
    rr = np.hypot(x[band] - cx, y[band] - cy)
    ct = col[band].mean(0)
    return dict(n=int(sel.sum()), cx=cx, cy=cy, r=r, rstd=float(rr.std()),
                h=float(ztop - zt), x0=float(x.min()), x1=float(x.max()),
                y0=float(y.min()), y1=float(y.max()),
                sx=float(x.max() - x.min()), sy=float(y.max() - y.min()),
                rt=(float(ct[0]), float(ct[1]), float(ct[2])))


def look(api):
    f = api.capture("cam_high")
    W = cloud(f)
    rgb = np.asarray(f.rgb)
    zt = table_z(W)
    X, Y, Z = W[..., 0], W[..., 1], W[..., 2]
    R, G, B = (rgb[..., 0].astype(float), rgb[..., 1].astype(float),
               rgb[..., 2].astype(float))
    base = (np.isfinite(Z) & (X > XLO) & (X < XHI) & (Y > YLO) & (Y < YHI)
            & (Z > zt + 0.008) & (Z < zt + 0.10))
    warm = base & (R - B > 25) & (R - G > 12)
    boxes = [stats(W, rgb, s, zt) for s in segment(W, warm, minn=120)]
    others = [stats(W, rgb, s, zt) for s in segment(W, base & ~warm, minn=150)]
    return zt, boxes, others


def is_bowl(s):
    return (0.030 < s["h"] < 0.075 and 0.030 < s["r"] < 0.070
            and s["rstd"] < 0.012 and s["sx"] < 0.15 and s["sy"] < 0.15)


def is_plate(s):
    return (0.010 < s["h"] < 0.030 and 0.045 < s["r"] < 0.085
            and s["rstd"] < 0.016 and 0.09 < s["sx"] < 0.18 and 0.09 < s["sy"] < 0.18)


def aabb_gap(p, s):
    """Signed distance from point p to cluster s's axis-aligned footprint."""
    dx = max(s["x0"] - p[0], p[0] - s["x1"])
    dy = max(s["y0"] - p[1], p[1] - s["y1"])
    if dx < 0 and dy < 0:
        return max(dx, dy)
    return float(np.hypot(max(dx, 0.0), max(dy, 0.0)))




# ------------------------------------------------------------------ program
FINGER_HEIGHT = 0.030      # fingertip height above the table at the pinch
MIDWALL = 0.004            # pinch radius = wall radius at the achieved height + this
RPIN_MARGIN = 0.004
TAPER_MIN = 0.010          # black bowl (0.020) vs ramekin (0.003)
CARRY_DZ = 0.30
HOME_X, HOME_Y = 0.02, 0.00   # central park, verified reachable in the reach survey
LAND_TOL = 0.004


def radius_at(W, zt, s, hgt, half=0.004):
    X, Y, Z = W[..., 0], W[..., 1], W[..., 2]
    d = np.hypot(X - s["cx"], Y - s["cy"])
    b = (np.isfinite(Z) & (d < s["r"] + 0.03) & (Z >= zt + hgt - half)
         & (Z < zt + hgt + half))
    if b.sum() < 15:
        return None
    return float(np.median(d[b]))


def look2(api):
    f = api.capture("cam_high")
    W = cloud(f)
    rgb = np.asarray(f.rgb)
    zt = table_z(W)
    X, Y, Z = W[..., 0], W[..., 1], W[..., 2]
    R, G, B = rgb[..., 0].astype(float), rgb[..., 1].astype(float), rgb[..., 2].astype(float)
    base = (np.isfinite(Z) & (X > XLO) & (X < XHI) & (Y > YLO) & (Y < YHI)
            & (Z > zt + 0.008) & (Z < zt + 0.10))
    warm = base & (R - B > 25) & (R - G > 12)
    boxes = [stats(W, rgb, s, zt) for s in segment(W, warm, minn=120)]
    others = [stats(W, rgb, s, zt) for s in segment(W, base & ~warm, minn=150)]
    return W, zt, boxes, others


def vessels(W, zt, others):
    out = []
    for i, s in enumerate(others):
        if not (0.030 < s["h"] < 0.075 and 0.028 < s["r"] < 0.070):
            continue
        lo = radius_at(W, zt, s, 0.012)
        hi = radius_at(W, zt, s, min(s["h"] - 0.006, 0.044))
        if lo is None or hi is None:
            continue
        q = dict(s)
        q["taper"] = hi - lo
        q["idx"] = i
        out.append(q)
    return out


def land(api, px, py, ztgt, tag):
    """Descend from the park height, cancelling the controller's z droop."""
    api.move([px, py, ztgt + 0.085], seconds=1.6)
    api.move([px, py, ztgt], seconds=1.5)
    e = api.eef()
    for _ in range(2):
        gap = e[2] - ztgt
        if abs(gap) <= LAND_TOL or gap > 0.030:
            break
        api.move([px, py, ztgt - min(max(gap, -0.015), 0.015)], seconds=1.1)
        e = api.eef()
    api.log("%s land eef=(%.4f,%.4f,%.4f) want=%.4f gap=%+.4f"
            % (tag, e[0], e[1], e[2], ztgt, e[2] - ztgt))
    return np.asarray(e), (e[2] - ztgt) > 0.030


def run(api):
    W, zt, boxes, others = look2(api)
    park = zt + PARK_DZ
    carry = zt + CARRY_DZ
    api.log("table_z=%.4f" % zt)
    ves = vessels(W, zt, others)
    for s in ves:
        api.log("VES c=(%.3f,%.3f) r=%.3f h=%.3f taper=%.4f bowl=%d"
                % (s["cx"], s["cy"], s["r"], s["h"], s["taper"], s["taper"] > TAPER_MIN))
    plates = [s for s in others if is_plate(s)]
    bowls = [s for s in ves if s["taper"] > TAPER_MIN]
    if not bowls and ves:
        bowls = sorted(ves, key=lambda s: -s["taper"])[:1]
        api.log("no tapered vessel -- using the most tapered")
    if not bowls or not plates:
        api.log("ABORT bowls=%d plates=%d" % (len(bowls), len(plates)))
        return "no-target"
    plate = max(plates, key=lambda s: s["n"])
    if boxes:
        box = max(boxes, key=lambda s: s["n"])
        anchor = (box["cx"], box["cy"])
    else:
        anchor = (plate["cx"], plate["cy"])
    api.log("anchor=(%.3f,%.3f) plate=(%.3f,%.3f) h=%.3f"
            % (anchor[0], anchor[1], plate["cx"], plate["cy"], plate["h"]))
    bowls.sort(key=lambda s: np.hypot(s["cx"] - anchor[0], s["cy"] - anchor[1]))
    tgt = bowls[0]
    cx, cy = tgt["cx"], tgt["cy"]
    rs = radius_at(W, zt, tgt, FINGER_HEIGHT)
    if rs is None:
        rs = tgt["r"] - 0.010
    rpin = rs + RPIN_MARGIN
    api.log("target=(%.3f,%.3f) r=%.3f h=%.3f taper=%.4f rsurf=%.4f rpin=%.4f"
            % (cx, cy, tgt["r"], tgt["h"], tgt["taper"], rs, rpin))

    # ---- side ranking (exclude the target by cluster index, not identity) --
    occ = [s for i, s in enumerate(others) if i != tgt["idx"]] + boxes
    sides = []
    for sgn in (1.0, -1.0):
        pin = (cx, cy + sgn * rpin)
        outer = (cx, cy + sgn * (rpin + JAW_HALF))
        gap = min([min(aabb_gap(pin, s), aabb_gap(outer, s)) for s in occ] or [9.9])
        sides.append([gap, sgn, pin])
        api.log("side %+.0f pin=(%.3f,%.3f) gap=%+.4f" % (sgn, pin[0], pin[1], gap))
    sides.sort(key=lambda q: -q[0])
    sgn = sides[0][1]
    tx, ty = plate["cx"], plate["cy"] + sgn * rpin
    api.log("SIDE %+.0f release=(%.3f,%.3f)" % (sgn, tx, ty))

    # ---- grasp -------------------------------------------------------------
    held, zg = False, zt + FINGER_HEIGHT + FINGER_DZ
    for attempt, extra in enumerate((0.0, 0.005)):
        if attempt:
            W, zt, b2, o2 = look2(api)
            cs = [s for s in vessels(W, zt, o2)
                  if np.hypot(s["cx"] - cx, s["cy"] - cy) < 0.10]
            if not cs:
                api.log("retry: target lost")
                break
            t2 = min(cs, key=lambda q: np.hypot(q["cx"] - cx, q["cy"] - cy))
            tgt = t2
            cx, cy = t2["cx"], t2["cy"]
            r2 = radius_at(W, zt, t2, FINGER_HEIGHT)
            rs = r2 if r2 is not None else rs
            rpin = rs + RPIN_MARGIN
            api.log("retry c=(%.3f,%.3f) rsurf=%.4f rpin=%.4f" % (cx, cy, rs, rpin + extra))
        px, py = cx, cy + sgn * (rpin + extra)
        ztgt = zt + FINGER_HEIGHT + FINGER_DZ
        api.grip(OPEN_W)
        api.move([px, py, park], seconds=2.0)
        e, stalled = land(api, px, py, ztgt, "G%d" % attempt)
        if stalled:
            api.log("G%d STALLED -- abandon" % attempt)
            api.move([px, py, park], seconds=1.8)
            continue
        # radial correction at the height actually achieved
        h_act = float(e[2] - zt - FINGER_DZ)
        r_act = radius_at(W, zt, tgt, h_act)
        if r_act is not None:
            rfix = r_act + MIDWALL + extra
            py_new = cy + sgn * rfix
            if abs(py_new - py) > 0.0015:
                api.move([px, py_new, float(e[2])], seconds=1.1)
                e = api.eef()
            api.log("G%d h_act=%.4f r_act=%.4f rpin->%.4f eef=(%.4f,%.4f,%.4f)"
                    % (attempt, h_act, r_act, rfix, e[0], e[1], e[2]))
            py = py_new
        zg = float(e[2])
        api.grip(0.0)
        api.settle(0.4)
        g0 = api.gripper()
        api.move([px, py, zt + 0.11], seconds=1.6)
        api.settle(0.2)
        api.grip(0.0)
        gmid = api.gripper()
        if gmid["effort"] < 2.5 or gmid["width_m"] <= HOLD_LO:
            api.log("G%d slipped at the first lift stage (%.4f/%.1f) -- set down"
                    % (attempt, gmid["width_m"], gmid["effort"]))
            api.move([px, py, float(e[2]) + 0.004], seconds=1.4)
            api.grip(OPEN_W)
            api.settle(0.2)
            api.move([px, py, park], seconds=1.6)
            continue
        api.move([px, py, carry], seconds=2.2)
        api.grip(0.0)
        api.settle(0.3)
        g1 = api.gripper()
        api.log("G%d closed=%.4f/%.1f carried=%.4f/%.1f"
                % (attempt, g0["width_m"], g0["effort"], g1["width_m"], g1["effort"]))
        if g1["effort"] >= 2.5 and HOLD_LO < g1["width_m"] < HOLD_HI:
            held = True
            break
        api.grip(OPEN_W)
        api.settle(0.2)
        api.move([px, py, park], seconds=1.6)
    api.log("held=%d zg=%.4f rel=%.4f" % (held, zg, zg - zt))
    if not held:
        return "no-grasp"

    # ---- retreat to the central park, then yaw there ----------------------
    R0 = np.asarray(api.tool_rotation(), float)
    api.move([HOME_X, HOME_Y, carry], seconds=2.2)
    e = api.eef()
    perr = float(np.hypot(e[0] - HOME_X, e[1] - HOME_Y))
    if perr > 0.015:
        api.move([HOME_X, HOME_Y, carry], seconds=1.4)
        e = api.eef()
        perr = float(np.hypot(e[0] - HOME_X, e[1] - HOME_Y))
    api.log("at park got=(%.4f,%.4f,%.4f) err=%.4f grip=%s"
            % (e[0], e[1], e[2], perr, api.gripper()))

    # yaw ladder, ordered by how near the base the resulting release pose is.
    # the bowl hangs at -sgn*rpin*yhat of the tool; a yaw of theta about world z
    # rotates that offset, so the release pose is plate - Rz(theta)*offset.
    def rz(th):
        c, s_ = float(np.cos(th)), float(np.sin(th))
        return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])

    rot, tx, ty = None, plate["cx"], plate["cy"] + sgn * rpin
    for th in (-0.5 * np.pi, np.pi, 0.0):
        if abs(th) < 1e-6:
            api.log("yaw ladder exhausted -- releasing unyawed")
            break
        RY = rz(th) @ R0
        api.move([HOME_X, HOME_Y, carry], rotation=RY, seconds=2.2)
        api.settle(0.3)
        R1 = np.asarray(api.tool_rotation(), float)
        gy = api.gripper()
        dR = float(np.abs(R1 - RY).max())
        ok = dR < 0.20 and gy["effort"] >= 2.5
        api.log("yaw %+.0fdeg ok=%d dR=%.3f grip=%s"
                % (np.degrees(th), ok, dR, gy))
        if ok:
            off = rz(th) @ np.array([0.0, -sgn * rpin, 0.0])
            rot = RY
            tx, ty = plate["cx"] - float(off[0]), plate["cy"] - float(off[1])
            break
    if rot is None:
        api.move([HOME_X, HOME_Y, carry], rotation=R0, seconds=1.8)
        api.settle(0.2)
        api.log("no yaw achieved; restored grip=%s" % (api.gripper(),))
    yaw_ok = rot is not None

    api.log("release target=(%.3f,%.3f) yawed=%d" % (tx, ty, yaw_ok))
    api.move([tx, ty, carry], rotation=rot, seconds=2.6)
    e = api.eef()
    if float(np.hypot(e[0] - tx, e[1] - ty)) > 0.030:
        api.move([tx, ty, carry], rotation=rot, seconds=1.6)
    e = api.eef()
    api.log("over plate cmd=(%.3f,%.3f) got=(%.4f,%.4f,%.4f) err=%.4f grip=%s"
            % (tx, ty, e[0], e[1], e[2], float(np.hypot(e[0] - tx, e[1] - ty)),
               api.gripper()))
    rel_z = zg + plate["h"] + 0.005
    api.move([tx, ty, rel_z + 0.05], rotation=rot, seconds=1.3)
    api.move([tx, ty, rel_z], rotation=rot, seconds=1.3)
    e = api.eef()
    api.log("release want_z=%.4f got=(%.4f,%.4f,%.4f) grip=%s"
            % (rel_z, e[0], e[1], e[2], api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.5)
    api.move([tx, ty, carry], rotation=rot, seconds=1.8)
    api.settle(0.3)

    W3, zt3, b3, o3 = look2(api)
    for s in vessels(W3, zt3, o3):
        api.log("FINAL c=(%.3f,%.3f) h=%.3f taper=%.4f d_plate=%.3f"
                % (s["cx"], s["cy"], s["h"], s["taper"],
                   float(np.hypot(s["cx"] - plate["cx"], s["cy"] - plate["cy"]))))
    return "done"
