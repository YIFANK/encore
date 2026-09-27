"""c2clean spa_bowl_between_pos_k0 -- v5: ring-vote perception + jam-aware motion.

v3 scored 11/15 on the debug band.  All four failures were perception or
configuration, not grasping:

  * ep56, ep64 -- the ramekin's footprint touches bowl B's, so height-gating at
    table+0.008 fused them into one 0.205 m blob; with no ramekin left there was
    no "between" test and the program grasped the blob.
  * ep61, ep65 -- the cookie box fused with the target bowl, dragging its
    centroid 50 mm toward the box; the arm then jammed reaching the bad aim
    point (eef frozen at x = 0.078 while the bias-cancel loop ran the command
    away) and burned the whole 1000-step horizon.

Fixes:
  1. Find the bowls by VOTING FOR THEIR RIM CIRCLE in a band above the ramekin
     top (0.9465 m) instead of clustering footprints.  A fused blob cannot fool
     a circle of fixed radius, and a broken annulus still votes for its centre.
     Validated on all 15 debug frames: exactly two peaks, 37-59 votes, third
     peak 9-16 -- a 2.3x margin.
  2. Find the ramekin as what is left of the 0.930 m vessel band once both rim
     circles are masked out.  Found on 15/15.
  3. Drop the per-episode table-touch calibration: it measured 0.0082-0.0091 m
     on all 15 seeds, and the excursion to a far-flung bare patch is what put
     the arm in the configuration that jammed.  TIP is now that measurement.
  4. Bounded, jam-aware servoing: if the eef stops responding, stop correcting
     and recover straight up rather than chasing the error.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "TABLE_Z": {
        "source": "debug-seed measurement: mode of the cam_high point-cloud z over "
                  "the tabletop crop = 0.900 on every probed seed (v1)",
        "allowed": True},
    "CELL, X0/X1/Y0/Y1, OBJ_DZ": {
        "source": "debug-seed measurement: 5 mm height-map grid over the base-frame "
                  "box that holds the props; table noise is +/-0.002 m so +0.008 m "
                  "separates a prop from the table (v1)",
        "allowed": True},
    "RIM_R": {
        "source": "debug-seed measurement: rim-circle vote peaks at radius 0.054 m "
                  "(votes 55-61) versus 48 at 0.0555 and 38 at 0.050, swept on seeds "
                  "51/56/61 (v3 t0 frames)",
        "allowed": True},
    "BOWL_BAND": {
        "source": "debug-seed measurement: bowl rims top out at 0.952-0.953 m, the "
                  "ramekin at 0.944, plate and cookie box at 0.920-0.921; 0.9465 m "
                  "keeps only the two bowl rims (v1/v3)",
        "allowed": True},
    "VESSEL_BAND, ARM_Z": {
        "source": "debug-seed measurement: 0.930 m clears the plate/box tops; the arm "
                  "and the stove-cabinet read above 0.99 m and are excluded (v1/v3)",
        "allowed": True},
    "VOTE_MIN, PEAK_SEP, RAM_CLEAR, RAM_ZLO/ZHI, RAM_MAX_W": {
        "source": "debug-seed measurement: on all 15 debug frames the two bowl peaks "
                  "poll 37-59 and the next peak 9-16; bowl centres are >= 0.13 m "
                  "apart; the ramekin reads 0.944 m top and 0.09 m across (v3 t0)",
        "allowed": True},
    "PLATE_ZTOP_HI": {
        "source": "debug-seed measurement: the plate is the largest component below "
                  "0.928 m on all 15 debug frames (n = 594-606, 0.135-0.140 m across)",
        "allowed": True},
    "TIP": {
        "source": "debug-seed measurement: pressing the jaws onto bare table read "
                  "eef_z - TABLE_Z = 0.0082-0.0091 m across all 15 debug seeds "
                  "(v3 PRESS/TIP-OFFSET logs); 0.0086 is the mid-point",
        "allowed": True},
    "AIM_R": {
        "source": "debug-seed measurement: offset from the rim centre that put the "
                  "jaws across the shell -- closed gap 0.0075-0.0083 m with effort "
                  "3.0 on seeds 51/53/55/57 (v3 AFTER-CLOSE logs)",
        "allowed": True},
    "BITE_Z": {
        "source": "debug-seed measurement: fingertip height above the table; the bowl "
                  "interior floor reads 0.908 m so the inner finger still clears it, "
                  "and the rim top is 0.952 m so the jaws are 28 mm down the shell (v1)",
        "allowed": True},
    "PLACE_CLEAR": {
        "source": "debug-seed measurement: plate top is 0.020 m above TABLE_Z (v1); "
                  "5 mm more sets the bowl down rather than dropping it",
        "allowed": True},
    "R_DOWN": {
        "source": "generic: tool-straight-down rotation, matching the episode's reset "
                  "tool_rotation to 0.06 rad (v1 ROT0 log)",
        "allowed": True},
    "OPEN_W, CLOSE_W": {
        "source": "generic: api.gripper() reads 0.0778 m at reset, 0.0799 m after "
                  "grip(0.078); <0.025 commands a close (v1/v2)",
        "allowed": True},
    "SERVO_TOL, SERVO_PASSES, JAM_DZ, MAX_CORR": {
        "source": "generic: open-loop moves land 6-12 mm off (v2/v3 MOVE logs), so "
                  "cancel the measured error in a bounded retry and stop when the eef "
                  "stops responding (v3 ep61 ran the command away)",
        "allowed": True},
    "GRASP_W_LO/HI": {
        "source": "debug-seed measurement: a good bite closes to 0.0075-0.0083 m with "
                  "effort 3.0; closing on nothing reads 0.001 and a total miss leaves "
                  "the jaws at 0.0800 (v2/v3/v4 AFTER-CLOSE logs)",
        "allowed": True},
    "TRANSIT_Z, HOVER_Z": {
        "source": "generic: transit clearance above the 0.953 m rim tops, kept near "
                  "the reset eef height 1.173 where the arm is well conditioned",
        "allowed": True},
    "DUMP_SIDE, CHUNK": {"source": "generic: api.log truncates at 2000 chars", "allowed": True},
}

TABLE_Z = 0.900
CELL = 0.005
X0, X1 = -0.40, 0.30
Y0, Y1 = -0.15, 0.50
OBJ_DZ = 0.008

RIM_R = 0.054
BOWL_BAND = 0.9465
VESSEL_BAND = 0.930
ARM_Z = 0.990
VOTE_MIN = 25.0
PEAK_SEP = 0.070
RAM_CLEAR = 0.070
RAM_ZLO, RAM_ZHI = 0.935, 0.9465
RAM_MAX_W = 0.130
PLATE_ZTOP_HI = 0.928

TIP = 0.0086
AIM_R = 0.047
BITE_Z = 0.024
PLACE_CLEAR = 0.005

OPEN_W = 0.078
CLOSE_W = 0.0
GRASP_W_LO, GRASP_W_HI = 0.003, 0.030

SERVO_TOL = 0.004
SERVO_PASSES = 3
JAM_DZ = 0.001
MAX_CORR = 0.030
TRANSIT_Z = 1.10
HOVER_Z = 1.06

DUMP_SIDE = 256
CHUNK = 1800

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


# ---------------------------------------------------------------------------
# telemetry

def _emit(api, tag, payload):
    b64 = base64.b64encode(zlib.compress(payload, 6)).decode()
    n = (len(b64) + CHUNK - 1) // CHUNK
    api.log("DUMP %s nchunks=%d" % (tag, n))
    for i in range(n):
        api.log("D %s %d %s" % (tag, i, b64[i * CHUNK:(i + 1) * CHUNK]))


def dump_frame(api, f, tag):
    rgb, depth = f.rgb, f.depth
    s = max(1, depth.shape[0] // DUMP_SIDE)
    d = np.asarray(depth, np.float64)[::s, ::s]
    d = np.where(np.isfinite(d) & (d > 0), d, 0.0)
    api.log("FRAME %s step=%d K=%s T=%s" % (
        tag, s, np.asarray(f.intrinsics).round(5).tolist(),
        np.asarray(f.t_base_cam).round(6).tolist()))
    _emit(api, tag + ".rgb", np.ascontiguousarray(rgb[::s, ::s, :]).tobytes())
    _emit(api, tag + ".d16", np.ascontiguousarray(
        np.clip(d * 1000.0, 0, 65535).astype(np.uint16)).tobytes())


# ---------------------------------------------------------------------------
# perception

def height_map(f):
    d = np.asarray(f.depth, np.float64)
    K, T = np.asarray(f.intrinsics, float), np.asarray(f.t_base_cam, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    B = (np.stack([x, y, d, np.ones_like(d)], -1) @ T.T)[..., :3]
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    m = ((d > 0) & (X > X0) & (X < X1) & (Y > Y0) & (Y < Y1)
         & (Z > TABLE_Z + OBJ_DZ))
    nx, ny = int((X1 - X0) / CELL), int((Y1 - Y0) / CELL)
    H = np.full((nx, ny), -1.0)
    ix = np.clip(((X - X0) / CELL).astype(int), 0, nx - 1)
    iy = np.clip(((Y - Y0) / CELL).astype(int), 0, ny - 1)
    for v, u in zip(*np.nonzero(m)):
        a, b, z = ix[v, u], iy[v, u], Z[v, u]
        if z > H[a, b]:
            H[a, b] = z
    return H


def grid_xy(H):
    nx, ny = H.shape
    ii, jj = np.mgrid[0:nx, 0:ny]
    return X0 + (ii + 0.5) * CELL, Y0 + (jj + 0.5) * CELL


def components(mask, min_cells):
    lab = np.zeros(mask.shape, int)
    cur, (nx, ny) = 0, mask.shape
    for i in range(nx):
        for j in range(ny):
            if mask[i, j] and lab[i, j] == 0:
                cur += 1
                st = [(i, j)]
                lab[i, j] = cur
                while st:
                    a, b = st.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if (0 <= p < nx and 0 <= q < ny and mask[p, q]
                                    and lab[p, q] == 0):
                                lab[p, q] = cur
                                st.append((p, q))
    return [np.argwhere(lab == k) for k in range(1, cur + 1)
            if (lab == k).sum() >= min_cells]


def stats(idx, H):
    xs = X0 + (idx[:, 0] + 0.5) * CELL
    ys = Y0 + (idx[:, 1] + 0.5) * CELL
    zs = H[idx[:, 0], idx[:, 1]]
    return dict(n=len(idx), cx=float(xs.mean()), cy=float(ys.mean()),
                ztop=float(zs.max()),
                w=float(xs.max() - xs.min() + CELL),
                h=float(ys.max() - ys.min() + CELL))


def ring_peaks(H, radius, band_lo, band_hi, ntheta=72):
    """Vote for circle centres of a known radius -- immune to footprint fusion."""
    nx, ny = H.shape
    acc = np.zeros((nx, ny))
    th = np.arange(ntheta) * 2 * np.pi / ntheta
    dxs = np.round(radius * np.cos(th) / CELL).astype(int)
    dys = np.round(radius * np.sin(th) / CELL).astype(int)
    for i, j in np.argwhere((H > band_lo) & (H < band_hi)):
        p, q = i - dxs, j - dys
        ok = (p >= 0) & (p < nx) & (q >= 0) & (q < ny)
        np.add.at(acc, (p[ok], q[ok]), 1.0)
    out = []
    nsep = int(PEAK_SEP / CELL)
    a = acc.copy()
    for _ in range(4):
        i, j = np.unravel_index(a.argmax(), a.shape)
        if a[i, j] < VOTE_MIN:
            break
        out.append((float(a[i, j]), X0 + (i + 0.5) * CELL, Y0 + (j + 0.5) * CELL))
        a[max(0, i - nsep):i + nsep + 1, max(0, j - nsep):j + nsep + 1] = 0.0
    return out


def _seg_dist(p, a, b):
    ab = b - a
    t = min(1.0, max(0.0, float(np.dot(p - a, ab) / max(1e-9, np.dot(ab, ab)))))
    return float(np.hypot(*(p - (a + t * ab))))


def perceive(api, tag):
    f = api.capture("cam_high")
    dump_frame(api, f, tag)
    H = height_map(f)

    bowls = ring_peaks(H, RIM_R, BOWL_BAND, ARM_Z)
    for v, x, y in bowls:
        api.log("BOWL[%s] votes=%d c=(%.3f,%.3f)" % (tag, v, x, y))

    full = [stats(i, H) for i in components(H > 0, 60)]
    flats = [p for p in full if p["ztop"] < PLATE_ZTOP_HI]
    plate = max(flats, key=lambda p: p["n"]) if flats else None

    gx, gy = grid_xy(H)
    M = (H > VESSEL_BAND) & (H < ARM_Z)
    for _, x, y in bowls:
        M = M & (np.hypot(gx - x, gy - y) > RAM_CLEAR)
    cand = [stats(i, H) for i in components(M, 20)]
    cand = [c for c in cand if RAM_ZLO < c["ztop"] < RAM_ZHI
            and c["w"] < RAM_MAX_W and c["h"] < RAM_MAX_W]
    ramekin = max(cand, key=lambda p: p["n"]) if cand else None

    api.log("SCENE[%s] bowls=%d plate=%s ramekin=%s" % (
        tag, len(bowls),
        None if plate is None else (round(plate["cx"], 3), round(plate["cy"], 3),
                                    round(plate["ztop"], 3), plate["n"]),
        None if ramekin is None else (round(ramekin["cx"], 3), round(ramekin["cy"], 3),
                                      round(ramekin["ztop"], 3), ramekin["n"])))
    return bowls, plate, ramekin


def pick_target(api, bowls, plate, ramekin):
    """The bowl BETWEEN the plate and the ramekin."""
    if not bowls:
        return None
    pts = [np.array([x, y]) for _, x, y in bowls]
    if len(pts) == 1 or plate is None:
        return pts[0]
    a = np.array([plate["cx"], plate["cy"]])
    if ramekin is None:
        # fall back on the weaker reading: the bowl nearest the plate.  On all
        # 15 debug frames this agrees with the segment test.
        scored = [(float(np.hypot(*(p - a))), p) for p in pts]
        api.log("BETWEEN fallback=plate-distance %s"
                % [round(s, 3) for s, _ in scored])
    else:
        b = np.array([ramekin["cx"], ramekin["cy"]])
        scored = [(_seg_dist(p, a, b), p) for p in pts]
        for s, p in scored:
            api.log("BETWEEN bowl=(%.3f,%.3f) segdist=%.3f" % (p[0], p[1], s))
    scored.sort(key=lambda t: t[0])
    return scored[0][1]


# ---------------------------------------------------------------------------
# motion

def servo(api, xyz, seconds=2.0, passes=SERVO_PASSES, tol=SERVO_TOL, tag=""):
    goal = np.asarray(xyz, float)
    cmd = goal.copy()
    prev = api.eef()
    e = prev
    jammed = False
    for k in range(passes):
        api.move(cmd, R_DOWN, seconds=seconds if k == 0 else 1.2)
        e = api.eef()
        err = goal - e
        api.log("SERVO%s p%d cmd=%s eef=%s err=%s" % (
            tag, k, np.round(cmd, 4).tolist(), np.round(e, 4).tolist(),
            np.round(err, 4).tolist()))
        if np.linalg.norm(err) < tol:
            break
        if k > 0 and np.linalg.norm(e - prev) < JAM_DZ:
            api.log("SERVO%s JAMMED (eef frozen at %s, err %.4f)"
                    % (tag, np.round(e, 4).tolist(), float(np.linalg.norm(err))))
            jammed = True
            break
        prev = e
        cmd = cmd + np.clip(err, -MAX_CORR, MAX_CORR)
    return np.asarray(e, float), jammed


def go_xy(api, x, y, z, tag=""):
    """Reposition at transit height, then settle onto (x, y, z)."""
    here = api.eef()
    if here[2] < TRANSIT_Z - 0.02:
        servo(api, [here[0], here[1], TRANSIT_Z], 1.5, 2, tag=tag + "-up")
    servo(api, [x, y, TRANSIT_Z], 2.5, 2, tag=tag + "-over")
    return servo(api, [x, y, z], 2.0, 3, tag=tag + "-down")


def attempt_grasp(api, bowl, bite_z, tag):
    gx, gy = float(bowl[0]), float(bowl[1]) - AIM_R
    z_grasp = TABLE_Z + bite_z + TIP
    api.log("GRASP%s aim=(%.4f,%.4f) eef_z=%.4f (fingertips %.3f above table)"
            % (tag, gx, gy, z_grasp, bite_z))
    api.grip(OPEN_W)
    _, jam = go_xy(api, gx, gy, z_grasp, tag=" grasp" + tag)
    api.grip(CLOSE_W)
    api.settle(0.5)
    g = api.gripper()
    ok = (GRASP_W_LO < g["width_m"] < GRASP_W_HI) and not jam
    api.log("AFTER-CLOSE%s grip=%s jam=%s ok=%s eef=%s"
            % (tag, g, jam, ok, np.round(api.eef(), 4).tolist()))
    return gx, gy, z_grasp, g, ok


def run(api):
    api.log("INSTRUCTION: %s" % api.instruction())
    api.grip(OPEN_W)
    api.settle(0.3)
    bowls, plate, ramekin = perceive(api, "t0")
    target = pick_target(api, bowls, plate, ramekin)
    if target is None or plate is None:
        return "perception failed (bowls=%d plate=%s)" % (len(bowls), plate is not None)
    api.log("TARGET=(%.3f,%.3f) PLATE=(%.3f,%.3f,z%.3f)"
            % (target[0], target[1], plate["cx"], plate["cy"], plate["ztop"]))

    gx, gy, z_grasp, g, ok = attempt_grasp(api, target, BITE_Z, "1")
    if not ok:
        # re-open, back off, look again and bite 5 mm deeper
        api.grip(OPEN_W)
        servo(api, [gx, gy, TRANSIT_Z], 2.0, 2, tag=" recover")
        bowls2, plate2, ram2 = perceive(api, "retry")
        t2 = pick_target(api, bowls2, plate2 or plate, ram2 or ramekin)
        if t2 is not None:
            target = t2
        gx, gy, z_grasp, g, ok = attempt_grasp(api, target, BITE_Z - 0.005, "2")

    servo(api, [gx, gy, TRANSIT_Z], 2.5, 2, tag=" lift")
    g2 = api.gripper()
    api.log("AFTER-LIFT grip=%s" % g2)

    z_place = z_grasp + (plate["ztop"] - TABLE_Z) + PLACE_CLEAR
    px, py = plate["cx"], plate["cy"] - AIM_R
    go_xy(api, px, py, z_place, tag=" place")
    api.log("PRE-RELEASE grip=%s z_place=%.4f" % (api.gripper(), z_place))
    api.grip(OPEN_W)
    api.settle(0.6)
    servo(api, [px, py, TRANSIT_Z], 2.0, 2, tag=" retreat")
    api.settle(0.4)
    perceive(api, "final")
    return "v5 ok=%s close_w=%.4f lift_w=%.4f" % (ok, g["width_m"], g2["width_m"])
