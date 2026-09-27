"""v5 -- v4 with GRIP_SIDE and the workspace crop declared. No behaviour change.

v4 scored 15/15 on the formal debug-seed run (results/sel_..._v4); v5 is byte-
identical in behaviour and was re-run formally so the frozen md5 matches its
own receipt.

What v2 taught (receipts in results/fs_..._v2):
  * api.move stops ~10 mm short: commanded z 0.998 -> eef 1.009.  Every aim
    must be closed-loop (re-issue the command shifted by the observed error).
  * The bowl that stands on the ramekin is TILTED ~16 deg toward the camera:
    its rim crest reads 0.100 above table on the far (-x) arc but only 0.070
    on the near (+x) arc, and 0.085 on both +/-y meridians -- a clean
    azimuthal sinusoid, identical in all 15 debug seeds.  Aiming the grip at
    the global crest therefore aims ~15 mm above the rim where the jaws
    actually close.
So: pinch at a +/-y meridian, where a tilt about y leaves the wall vertical in
the jaw plane, and take the rim height AND the wall radius from that meridian.

What v3 taught (results/fs_..._v3): the pinch itself now works -- 8/8 closed on
the wall at width 0.0061-0.0069 with effort 3.00 -- but 0/8 scored, and the
ep51 gif shows the bowl parked on the FAR RIM of the plate, about 46 mm off
centre.  A rim pinch holds the bowl by its edge, so the bowl's centre sits a
full grip radius away from the eef; driving the eef to the plate centre puts
the BOWL a grip radius off it.  v4 carries that offset explicitly.
"""

import numpy as np

PROVENANCE = {
    "TIP_BELOW_EEF": {
        "source": "debug-seed measurement: cam_arm_wrist depth at episode start "
                  "resolves both finger pads; lowest pad point z=1.1640 vs "
                  "api.eef() z=1.17328 -> 0.0093 m",
        "allowed": True},
    "JAW_MAX": {
        "source": "debug-seed measurement: api.gripper()['width_m']=0.0778 open at "
                  "episode start; wrist depth puts the pad inner faces at +/-0.0389 "
                  "in tool y, so width_m is the inner-face gap",
        "allowed": True},
    "BOWL_H": {
        "source": "debug-seed measurement: the table-standing bowl's rim crest sits "
                  "0.0514 above the table plane (cam_high cloud, all 15 seeds)",
        "allowed": True},
    "BOWL_DIA": {
        "source": "debug-seed measurement: both bowls' cam_high footprint bbox is "
                  "0.105-0.115 across in all 15 debug seeds; 0.110 > JAW_MAX so a "
                  "body straddle is impossible and the rim is the widest feature",
        "allowed": True},
    "GRIP_DEPTH": {
        "source": "debug-seed measurement of the wall profile at the +/-y meridian: "
                  "the wall is a ~5 mm thick lip whose radius tapers ~0.45 mm per mm "
                  "of depth; 0.008 below the local rim top is a full-thickness bite "
                  "that is still above the taper",
        "allowed": True},
    "WALL_HALF": {
        "source": "debug-seed measurement: at the -y meridian the visible (exterior) "
                  "wall radius at the rim is 0.0548 and the interior surface reaches "
                  "0.050, i.e. a ~5 mm lip -> mid-wall is 0.0025 inside the exterior",
        "allowed": True},
    "MOVE_TOL": {
        "source": "debug-seed measurement: v2 move residuals were 0.0076-0.0108 on "
                  "every leg, so the controller's own stop band is ~0.010; the "
                  "closed-loop retry tolerance is set tighter than that",
        "allowed": True},
    "PLATE_RGB_MIN": {
        "source": "debug-seed measurement: plate cluster mean RGB ~154 vs the stove "
                  "slab ~69 in all 15 debug seeds",
        "allowed": True},
    "GRIP_SIDE": {
        "source": "debug-seed measurement: at the -y meridian cam_high sees the "
                  "bowl's EXTERIOR wall (that side faces the camera at y=0), so the "
                  "measured radius there is the outer surface and mid-wall is "
                  "WALL_HALF inside it; at +y only the interior sheet is visible and "
                  "the sign would invert.  -y is also the side toward the robot "
                  "centreline.",
        "allowed": True},
    "WORKSPACE_CROP": {
        "source": "debug-seed measurement: XLIM/YLIM/RES bound the table region that "
                  "carries the four props in all 15 debug captures; the walls "
                  "deproject outside it and the modal-z table fit is taken inside it",
        "allowed": True},
    "R_DOWN": {
        "source": "generic controller mechanics: tool z straight down, jaw axis along "
                  "base y (matches the measured start tool_rotation)",
        "allowed": True},
}

TIP_BELOW_EEF = 0.0093
JAW_MAX = 0.0778
BOWL_H = 0.0514
GRIP_DEPTH = 0.008
WALL_HALF = 0.0025
MOVE_TOL = 0.004
PLATE_RGB_MIN = 120.0
GRIP_SIDE = -1.0          # pinch the -y arc (the side toward the robot centreline)

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

XLIM = (-0.30, 0.35)
YLIM = (-0.45, 0.45)
RES = 0.005


# ---------------------------------------------------------------- perception

def _cloud(frame):
    K = np.asarray(frame.intrinsics, dtype=np.float64)
    T = np.asarray(frame.t_base_cam, dtype=np.float64)
    d = np.asarray(frame.depth, dtype=np.float64)
    H, W = d.shape
    v, u = np.mgrid[0:H, 0:W]
    pc = np.stack([(u - K[0, 2]) / K[0, 0] * d, (v - K[1, 2]) / K[1, 1] * d, d], -1)
    return pc @ T[:3, :3].T + T[:3, 3]


def _heightmap(P):
    nx = int(round((XLIM[1] - XLIM[0]) / RES))
    ny = int(round((YLIM[1] - YLIM[0]) / RES))
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    ix = np.floor((X - XLIM[0]) / RES).astype(int)
    iy = np.floor((Y - YLIM[0]) / RES).astype(int)
    ok = np.isfinite(Z) & (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    z = Z[ok]
    order = np.argsort(z)
    flat = (ix[ok] * ny + iy[ok])[order]
    vv, uu = np.nonzero(ok)
    lin = (vv * P.shape[1] + uu)[order]
    hm = np.full(nx * ny, -np.inf)
    src = np.full(nx * ny, -1, dtype=np.int64)
    hm[flat] = z[order]
    src[flat] = lin
    return hm.reshape(nx, ny), src.reshape(nx, ny)


def _label(mask):
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    H, W = mask.shape
    for i in range(H):
        for j in range(W):
            if mask[i, j] and lab[i, j] == 0:
                cur += 1
                stack = [(i, j)]
                lab[i, j] = cur
                while stack:
                    a, b = stack.pop()
                    for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        p, q = a + da, b + db
                        if 0 <= p < H and 0 <= q < W and mask[p, q] and lab[p, q] == 0:
                            lab[p, q] = cur
                            stack.append((p, q))
    return lab, cur


def _components(hm, src, rgb, zlo, zhi, minarea=40):
    m = np.isfinite(hm) & (hm > zlo) & (hm < zhi)
    lab, n = _label(m)
    out = []
    for k in range(1, n + 1):
        ii, jj = np.nonzero(lab == k)
        if len(ii) < minarea:
            continue
        xs = XLIM[0] + (ii + 0.5) * RES
        ys = YLIM[0] + (jj + 0.5) * RES
        zs = hm[ii, jj]
        col = rgb.reshape(-1, 3)[src[ii, jj]].mean(0)
        out.append(dict(n=int(len(ii)), zmax=float(zs.max()),
                        cx=float(0.5 * (xs.min() + xs.max())),
                        cy=float(0.5 * (ys.min() + ys.max())),
                        w=float(xs.max() - xs.min()), d=float(ys.max() - ys.min()),
                        rgb=float(col.mean())))
    return sorted(out, key=lambda c: -c["n"])


def _table_z(P):
    Z, X, Y = P[..., 2], P[..., 0], P[..., 1]
    m = np.isfinite(Z) & (X > XLIM[0]) & (X < XLIM[1]) & (Y > YLIM[0]) & (Y < YLIM[1])
    h, e = np.histogram(Z[m], bins=200, range=(0.85, 1.05))
    return float(e[int(np.argmax(h))] + 0.5 * (e[1] - e[0]))


def perceive(api):
    f = api.capture("cam_high")
    P = _cloud(f)
    rgb = np.asarray(f.rgb, dtype=np.float64)
    tz = _table_z(P)
    hm, src = _heightmap(P)
    comps = _components(hm, src, rgb, tz + 0.012, tz + 0.16)
    api.log("TABLE_Z %.4f" % tz)
    for c in comps:
        api.log("COMP n=%d c=(%+.3f,%+.3f) top=+%.3f wd=%.3f/%.3f rgb=%.0f"
                % (c["n"], c["cx"], c["cy"], c["zmax"] - tz, c["w"], c["d"], c["rgb"]))
    return P, tz, comps


def classify(api, tz, comps):
    bowls, plates = [], []
    for c in comps:
        square = 0.085 < c["w"] < 0.135 and 0.085 < c["d"] < 0.135
        if square and (c["zmax"] - tz) > 0.035 and c["rgb"] < PLATE_RGB_MIN:
            bowls.append(c)
        elif (c["zmax"] - tz) < 0.055 and c["rgb"] >= PLATE_RGB_MIN and c["n"] > 200:
            plates.append(c)
    bowls.sort(key=lambda c: -c["zmax"])
    plates.sort(key=lambda c: -c["n"])
    api.log("CLASSIFY nbowl=%d nplate=%d" % (len(bowls), len(plates)))
    return (bowls[0] if bowls else None), (plates[0] if plates else None)


def meridian(api, P, bowl, tz, sgn):
    """Rim top and exterior wall radius on the +/-y meridian of the bowl."""
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    cx, cy = bowl["cx"], bowl["cy"]
    dy = sgn * (Y - cy)
    band = (np.isfinite(Z) & (np.abs(X - cx) < 0.010) & (dy > 0.028) & (dy < 0.080)
            & (Z > tz + 0.02) & (Z < bowl["zmax"] + 0.004))
    if band.sum() < 8:
        return None
    zz, rr = Z[band], dy[band]
    top = np.argsort(zz)[-8:]
    rim_z = float(np.median(zz[top]))
    r_top = float(np.median(rr[top]))
    z_tips = rim_z - GRIP_DEPTH
    at = band & (np.abs(Z - z_tips) < 0.002)
    if at.sum() >= 4:
        r_ext = float(np.median(sgn * (Y[at] - cy)))
        how = "measured"
    else:
        r_ext = r_top - 0.45 * GRIP_DEPTH
        how = "tapered"
    api.log("MERIDIAN sgn=%+d rim_z=%.4f (+%.3f) r_top=%.4f r_ext@grip=%.4f (%s, n=%d)"
            % (sgn, rim_z, rim_z - tz, r_top, r_ext, how, int(at.sum())))
    return dict(rim_z=rim_z, r_ext=r_ext, r_top=r_top)


def plate_floor(P, plate):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    r = np.hypot(X - plate["cx"], Y - plate["cy"])
    m = np.isfinite(Z) & (r < 0.025)
    return float(np.median(Z[m])) if m.sum() > 20 else plate["zmax"] - 0.012


# ---------------------------------------------------------------- motion

def go(api, xyz, seconds=2.0, tag="", tries=1, tol=MOVE_TOL):
    """Closed-loop move: re-issue the command shifted by the observed error."""
    tgt = np.asarray(xyz, dtype=np.float64)
    cmd = tgt.copy()
    e = None
    for k in range(tries):
        api.move(cmd, rotation=R_DOWN, seconds=seconds)
        e = np.asarray(api.eef(), dtype=np.float64)
        err = tgt - e
        api.log("MOVE %-9s#%d cmd=(%+.3f,%+.3f,%+.3f) got=(%+.3f,%+.3f,%+.3f) err=%.4f"
                % (tag, k, cmd[0], cmd[1], cmd[2], e[0], e[1], e[2], float(np.linalg.norm(err))))
        if float(np.linalg.norm(err)) < tol:
            break
        cmd = np.clip(cmd + err, tgt - 0.06, tgt + 0.06)
    return e


def grip_state(api, tag):
    g = api.gripper()
    api.log("GRIP %-9s width=%.4f effort=%.2f" % (tag, g["width_m"], g["effort"]))
    return g


def run(api):
    api.log("INTENT %r" % (api.instruction(),))
    P, tz, comps = perceive(api)
    bowl, plate = classify(api, tz, comps)
    if bowl is None or plate is None:
        api.log("ABORT missing bowl=%s plate=%s" % (bowl is not None, plate is not None))
        return
    api.log("TARGET bowl c=(%+.3f,%+.3f) top=+%.3f | PLATE c=(%+.3f,%+.3f)"
            % (bowl["cx"], bowl["cy"], bowl["zmax"] - tz, plate["cx"], plate["cy"]))
    pf = plate_floor(P, plate)
    api.log("PLATE_FLOOR %.4f (+%.3f)" % (pf, pf - tz))

    md = meridian(api, P, bowl, tz, GRIP_SIDE)
    if md is None:
        api.log("ABORT no meridian")
        return

    r_grip = md["r_ext"] - WALL_HALF
    gx = bowl["cx"]
    gy = bowl["cy"] + GRIP_SIDE * r_grip
    z_grip = md["rim_z"] - GRIP_DEPTH + TIP_BELOW_EEF
    z_high = bowl["zmax"] + 0.10
    api.log("PLAN grip=(%+.3f,%+.3f,%.4f) r_grip=%.4f tips_at=%.4f"
            % (gx, gy, z_grip, r_grip, z_grip - TIP_BELOW_EEF))

    api.grip(JAW_MAX)
    go(api, [gx, gy, z_high], 2.0, "hover", tries=2, tol=0.006)
    go(api, [gx, gy, z_grip], 2.0, "descend", tries=3, tol=0.003)
    grip_state(api, "pre")
    api.grip(0.0)
    api.settle(0.4)
    g = grip_state(api, "closed")

    go(api, [gx, gy, z_high], 2.0, "lift", tries=1)
    g2 = grip_state(api, "lifted")
    api.log("HOLD closed=%.4f lifted=%.4f effort=%.2f" % (g["width_m"], g2["width_m"], g2["effort"]))

    # how far below the eef does the bowl hang now?
    hang = TIP_BELOW_EEF + (BOWL_H - GRIP_DEPTH)
    try:
        P2 = _cloud(api.capture("cam_high"))
        e = np.asarray(api.eef(), dtype=np.float64)
        X, Y, Z = P2[..., 0], P2[..., 1], P2[..., 2]
        m = (np.isfinite(Z) & (np.abs(X - e[0]) < 0.10) & (np.abs(Y - e[1]) < 0.12)
             & (Z > tz + 0.10) & (Z < e[2] - 0.006))
        if m.sum() > 60:
            base = float(np.percentile(Z[m], 1))
            hang_meas = e[2] - base
            bx = 0.5 * (X[m].min() + X[m].max())
            by = 0.5 * (Y[m].min() + Y[m].max())
            api.log("HANG base=%.4f hang=%.4f (default %.4f) n=%d bowl_off=(%+.4f,%+.4f)"
                    % (base, hang_meas, hang, int(m.sum()), bx - e[0], by - e[1]))
            if 0.02 < hang_meas < 0.12:
                hang = hang_meas
    except Exception as exc:  # noqa: BLE001
        api.log("HANG_ERR %r" % (exc,))

    # the pinch holds the bowl by its rim, so the bowl centre sits a full grip
    # radius from the eef, on the far side of the pinched arc.
    off_y = -GRIP_SIDE * r_grip
    px = plate["cx"]
    py = plate["cy"] - off_y
    z_rel = pf + 0.012 + hang
    api.log("PLACE base_target=%.4f eef_z=%.4f hang=%.4f carry_off_y=%+.4f eef_xy=(%+.3f,%+.3f)"
            % (pf + 0.012, z_rel, hang, off_y, px, py))
    go(api, [px, py, z_high], 3.0, "travel", tries=2, tol=0.008)
    grip_state(api, "travel")
    go(api, [px, py, z_rel], 2.0, "lower", tries=2, tol=0.004)
    grip_state(api, "atplate")
    api.grip(JAW_MAX)
    api.settle(0.6)
    grip_state(api, "released")
    go(api, [px, py, z_high], 2.0, "retreat", tries=1)
    go(api, [px - 0.15, py, z_high], 2.0, "away", tries=1)
    api.settle(0.5)
    api.log("END")
