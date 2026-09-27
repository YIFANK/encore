"""c2k1clean spa_bowl_between_task_k1 -- v22b (frozen candidate).

Perception is now a fixed-radius Hough over the bowl-rim height band, so
bowls that abut the ramekin (seeds 57/61/63/65 fused them into one connected
component) are still separated.  Motion is budgeted: LIBERO's move_cartesian
stops at POS_TOL, so each waypoint gets at most two commands.
"""
import numpy as np

# ---- calibrated constants (see PROVENANCE) ------------------------------
RIM_Z = 0.0445       # height band floor that isolates bowl rims
RIM_R = 0.0515       # bowl rim ring radius used by the Hough vote
RAM_Z = 0.030        # height band floor for the ramekin
Z_CLOSE = 0.029      # grasp height above the table
OFF = 0.030          # radial offset of the TCP from the rim centre
Z_LOCK = 0.106       # hover height the good descent regime starts from
W_EMPTY = 0.0060     # a close this tight held nothing at all
N_TRY = 3            # grasp attempts before giving up
PAYLOAD_MIN = 400    # profile points that mean something really is hanging
Z_REL = 0.072        # release height above the table
X_BIAS = 0.004       # the bowl lands this far -x of its profiled centroid
Z_APPROACH = 0.095   # first hover, just above the props
Z_HOVER = 0.16
BAND_LO, BAND_HI = 0.012, 0.14
# -------------------------------------------------------------------------

PROVENANCE = {
    "GX0/GX1/GY0/GY1/GS": {
        "source": "debug seeds 51-65 cam_high depth: every table prop sits "
                  "inside x in [-0.35, 0.25], y in [0.00, 0.38]; the 0.004 m "
                  "vote grid is a fifth of the 0.021 m gap between the bowl "
                  "and ramekin rim radii",
        "allowed": True},
    "RIM_Z": {
        "source": "debug seeds 51/53/55/57/59/61/63/65 cam_high depth: bowl "
                  "tops are table+0.050 and the ramekin top is table+0.042, so "
                  "a floor at table+0.0445 keeps only bowl rims",
        "allowed": True},
    "RIM_R": {
        "source": "debug seed 51 v4p probe: Kasa fit of the topmost 6 mm of a "
                  "bowl cluster gives r=0.053 with radial sd 0.0013; the "
                  "cluster bounding box is 0.109x0.111 (outer r=0.0545)",
        "allowed": True},
    "RAM_Z": {
        "source": "debug seeds 51/53: ramekin top table+0.042, plate and "
                  "cookie box tops table+0.019",
        "allowed": True},
    "Z_CLOSE": {
        "source": "pack.json demo0 ee_path lowest point z=0.9301 while the "
                  "gripper is closing, minus the table plane z=0.9012 "
                  "measured from cam_high depth on debug seeds",
        "allowed": True},
    "OFF": {
        "source": "debug-seed v2/v5 sweep, measured from the Kasa rim centre: "
                  "radial offsets 0.019/0.032/0.037 pinched the rim, "
                  "0.044/0.045/0.052 closed on air; LIBERO's move_cartesian "
                  "POS_TOL leaves ~0.012 of aim error, so 0.030 centres the "
                  "usable window",
        "allowed": True},
    "Z_LOCK": {
        "source": "debug seeds 51-65, v10b: the four episodes whose hover "
                  "settled at table+0.106 descended to table+0.033 and closed "
                  "to 0.0150-0.0166; the four that settled at table+0.1035 "
                  "stalled at table+0.042 and closed to 0.0010-0.0101",
        "allowed": True},
    "PAYLOAD_MIN": {
        "source": "debug seeds 51-65, v10b/v14b: a cradled bowl profiles "
                  "1900-2400 points in the 10-50 mm slices under the eef, "
                  "while an empty gripper profiles 66-69 -- and on seed 53 a "
                  "bowl merely shoved 0.048 m aside left its cell empty, so "
                  "the cell test alone reports a phantom grasp",
        "allowed": True},
    "Z_REL/X_BIAS": {
        "source": "debug seeds 51-65, v15 selection: across the ten firm "
                  "carries the bowl landed at the profiled offset to within "
                  "0.005 m, with a systematic -0.004 m in x; the loose carries "
                  "slipped a further 0.030 m in -x during the drop from the "
                  "table+0.082 release height",
        "allowed": True},
    "N_TRY": {
        "source": "debug seeds 51-65: three attempts is what the episode "
                  "horizon affords -- the v5 four-step ladder ran the arm past "
                  "the end of the episode (frozen eef, gripper refusing to "
                  "close) while three attempts always completed",
        "allowed": True},
    "W_EMPTY": {
        "source": "debug seed 51, v1: a closed gripper in free air reports "
                  "width 0.0010; every debug-seed close that held nothing "
                  "reported exactly that, while every close with the rim "
                  "between the jaws reported 0.0088 or more",
        "allowed": True},
    "DESCENT_SHAPE": {
        "source": "debug seeds 51/53, v9d: from a hover at table+0.103 a single "
                  "rotation-locked move commanded to table+0.020 converges to "
                  "table+0.031 with the commanded xy reached to 0.002 m; the "
                  "same descent issued with rotation=None stalls at "
                  "table+0.047 and needs six pushes to reach table+0.030",
        "allowed": True},
    "Z_APPROACH": {
        "source": "debug seeds: prop tops are table+0.050, so table+0.095 "
                  "clears them; a shorter descent leaves less of the +x drift "
                  "measured in v6 (0.016 m over a 0.13 m descent)",
        "allowed": True},
    "Z_HOVER": {
        "source": "debug seeds: tallest table prop top is table+0.050",
        "allowed": True},
    "BAND_LO/BAND_HI": {
        "source": "debug seeds: prop tops table+0.019..0.050; the cabinet/stove "
                  "mass is table+0.226",
        "allowed": True},
    "PLATE_DISCRIMINATOR": {
        "source": "debug seeds: plate h=0.019 ext=0.136 rgb~(153,142,139); "
                  "cookie box h=0.019 ext=0.082x0.060 rgb~(92,65,45)",
        "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def cloud(frame):
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    ok = np.isfinite(d) & (d > 0)
    z = np.where(ok, d, 1.0)
    pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], axis=-1)
    return pc @ T[:3, :3].T + T[:3, 3], ok


def label(mask, min_px=200):
    lab = np.zeros(mask.shape, np.int32)
    H, W = mask.shape
    cur = 0
    for p in map(tuple, np.argwhere(mask)):
        if lab[p]:
            continue
        cur += 1
        lab[p] = cur
        st = [p]
        while st:
            y, x = st.pop()
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                q = (y + dy, x + dx)
                if 0 <= q[0] < H and 0 <= q[1] < W and mask[q] and not lab[q]:
                    lab[q] = cur
                    st.append(q)
    return [np.argwhere(lab == i) for i in range(1, cur + 1)
            if (lab == i).sum() >= min_px]


def kasa(x, y):
    A = np.stack([x, y, np.ones_like(x)], 1)
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    return float(cx), float(cy), float(np.sqrt(max(sol[2] + cx * cx + cy * cy, 1e-9)))


GX0, GX1, GY0, GY1, GS = -0.40, 0.30, -0.06, 0.42, 0.004


def hough(pts, r, n_ang=48):
    """Fixed-radius circle vote -> (accumulator, nx, ny)."""
    nx = int((GX1 - GX0) / GS) + 1
    ny = int((GY1 - GY0) / GS) + 1
    acc = np.zeros((nx, ny), np.int32)
    ang = np.linspace(0, 2 * np.pi, n_ang, endpoint=False)
    for a in ang:
        cx = pts[:, 0] + r * np.cos(a)
        cy = pts[:, 1] + r * np.sin(a)
        ix = np.round((cx - GX0) / GS).astype(int)
        iy = np.round((cy - GY0) / GS).astype(int)
        good = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        np.add.at(acc, (ix[good], iy[good]), 1)
    # 3x3 box blur
    sm = acc.astype(float).copy()
    for ax in (0, 1):
        sm = sm + np.roll(sm, 1, ax) + np.roll(sm, -1, ax)
    return sm, nx, ny


def find_bowls(api, pts, want=2):
    sm, nx, ny = hough(pts, RIM_R)
    out = []
    work = sm.copy()
    for _ in range(want + 2):
        i = int(np.argmax(work))
        ix, iy = i // ny, i % ny
        val = work[ix, iy]
        if val <= 0:
            break
        cx, cy = GX0 + ix * GS, GY0 + iy * GS
        d = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
        ring = (d > 0.035) & (d < 0.068)
        if ring.sum() >= 200:
            kx, ky, kr = kasa(pts[ring, 0], pts[ring, 1])
            res = np.hypot(pts[ring, 0] - kx, pts[ring, 1] - ky)
            if abs(kx - cx) < 0.02 and abs(ky - cy) < 0.02 and res.std() < 0.006:
                cx, cy = kx, ky
            out.append({"x": float(cx), "y": float(cy), "vote": float(val),
                        "nring": int(ring.sum()), "r": float(np.median(res))})
        # suppress this peak
        gx = np.arange(nx)[:, None] * GS + GX0
        gy = np.arange(ny)[None, :] * GS + GY0
        work[np.hypot(gx - cx, gy - cy) < 0.075] = 0
    out.sort(key=lambda b: -b["vote"])
    for b in out:
        api.log("HB x=%.3f y=%.3f vote=%.0f nring=%d r=%.4f" % (
            b["x"], b["y"], b["vote"], b["nring"], b["r"]))
    keep = [b for b in out if b["vote"] > 0.45 * out[0]["vote"]][:want] if out else []
    return keep


def perceive(api):
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb)
    base, ok = cloud(f)
    X, Y, Z = base[..., 0], base[..., 1], base[..., 2]
    ws = ok & (X > GX0) & (X < GX1) & (Y > GY0) & (Y < GY1) \
        & (Z > 0.60) & (Z < 1.40)
    table = float(np.median(Z[ok & (abs(X) < 0.4) & (abs(Y) < 0.4)
                              & (Z > 0.6) & (Z < 1.4)]))
    rim = ws & (Z > table + RIM_Z) & (Z < table + BAND_HI)
    pts = np.stack([X[rim], Y[rim]], 1)
    api.log("table=%.4f rim_pts=%d" % (table, len(pts)))
    bowls = find_bowls(api, pts) if len(pts) > 300 else []

    # ramekin: raised material that no bowl explains
    ram_m = ws & (Z > table + RAM_Z) & (Z < table + RIM_Z)
    rp = np.stack([X[ram_m], Y[ram_m]], 1)
    for b in bowls:
        rp = rp[np.hypot(rp[:, 0] - b["x"], rp[:, 1] - b["y"]) > 0.064]
    ramekin = None
    if len(rp) > 150:
        ramekin = {"x": float((rp[:, 0].min() + rp[:, 0].max()) / 2),
                   "y": float((rp[:, 1].min() + rp[:, 1].max()) / 2),
                   "n": len(rp)}
    api.log("ramekin=" + ("None" if ramekin is None else
                          "%.3f %.3f n=%d" % (ramekin["x"], ramekin["y"],
                                              ramekin["n"])))

    # plate: the flat wide light disc
    flat = ws & (Z > table + BAND_LO) & (Z < table + 0.030)
    plate = None
    for c in label(flat, 300):
        ys, xs = c[:, 0], c[:, 1]
        px, py, pz = X[ys, xs], Y[ys, xs], Z[ys, xs]
        col = rgb[ys, xs].mean(0)
        xe, ye = float(px.max() - px.min()), float(py.max() - py.min())
        api.log("F n=%d ctr=(%.3f,%.3f) ext=%.3f/%.3f rgb=(%d,%d,%d)" % (
            len(c), (px.min() + px.max()) / 2, (py.min() + py.max()) / 2,
            xe, ye, col[0], col[1], col[2]))
        if min(col) > 110 and 0.11 < max(xe, ye) < 0.18:
            cand = {"x": float((px.min() + px.max()) / 2),
                    "y": float((py.min() + py.max()) / 2),
                    "top": float(np.percentile(pz, 97)), "ext": max(xe, ye)}
            if plate is None or cand["ext"] > plate["ext"]:
                plate = cand
    api.log("plate=" + ("None" if plate is None else
                        "%.3f %.3f" % (plate["x"], plate["y"])))
    return table, bowls, ramekin, plate, (X, Y, Z, ok)


def seg_dist(p, a, b):
    p, a, b = map(np.asarray, (p, a, b))
    ab = b - a
    t = 0.0 if ab @ ab < 1e-9 else float(np.clip((p - a) @ ab / (ab @ ab), 0, 1))
    return float(np.linalg.norm(p - (a + t * ab)))


def choose(api, bowls, ramekin, plate):
    if len(bowls) < 2:
        return bowls[0] if bowls else None
    if ramekin is not None and plate is not None:
        d = [seg_dist((b["x"], b["y"]), (plate["x"], plate["y"]),
                      (ramekin["x"], ramekin["y"])) for b in bowls]
    elif plate is not None:
        d = [np.hypot(b["x"] - plate["x"], b["y"] - plate["y"]) for b in bowls]
    else:
        d = [b["y"] for b in bowls]
    api.log("bowl scores %s" % np.round(d, 4).tolist())
    return bowls[int(np.argmax(d))]



def profile(api, table, XYZ, e):
    """Where material sits under the gripper, in 10 mm slices."""
    X, Y, Z, ok = XYZ
    near = ok & (np.hypot(X - e[0], Y - e[1]) < 0.15) & (Z > table + 0.085)
    out = []
    for k in range(9):
        lo, hi = e[2] - 0.010 * (k + 1), e[2] - 0.010 * k
        m = near & (Z > lo) & (Z <= hi)
        if m.sum() < 40:
            continue
        cx = float((X[m].min() + X[m].max()) / 2)
        cy = float((Y[m].min() + Y[m].max()) / 2)
        api.log("SL %d n=%d d=(%+.3f,%+.3f) ext=%.3f/%.3f" % (
            k, m.sum(), cx - e[0], cy - e[1],
            float(X[m].max() - X[m].min()), float(Y[m].max() - Y[m].min())))
        out.append((k, int(m.sum()), cx, cy))
    return out


def mv(api, x, y, z, seconds=1.2):
    api.move([x, y, z], rotation=R_DOWN, seconds=seconds)
    return api.eef()


def mv2(api, x, y, z, seconds=2.0, csec=1.2):
    """Command, then re-command with the measured error mirrored."""
    e = mv(api, x, y, z, seconds)
    return mv(api, x + (x - e[0]), y + (y - e[1]), z + (z - e[2]), csec)


def grab(api, table, tx, ty, off):
    """One rim pinch on the +x arc; returns the closed gripper width."""
    api.grip(0.08)
    e = mv(api, tx + off, ty, table + 0.14, 2.0)
    e = mv(api, tx + off + (tx + off - e[0]), ty + (ty - e[1]),
           table + Z_APPROACH, 1.4)
    zt = table + Z_LOCK
    for _ in range(2):
        if abs(e[2] - zt) < 0.002:
            break
        e = mv(api, tx + off + (tx + off - e[0]), ty + (ty - e[1]),
               zt + (zt - e[2]), 0.9)
    api.log("  approach %s d=(%+.3f,%+.3f) dz=%.4f" % (
        np.round(e, 4).tolist(), e[0] - tx, e[1] - ty, e[2] - table))
    for k in range(3):
        e = mv(api, tx + off, ty, table + Z_CLOSE - 0.009, 1.0)
        api.log("  push%d %s d=(%+.3f,%+.3f) dz=%.4f" % (
            k, np.round(e, 4).tolist(), e[0] - tx, e[1] - ty, e[2] - table))
        if e[2] - table < 0.040:
            break
    api.grip(0.0)
    api.settle(0.6)
    return api.gripper()["width_m"], e


def run(api):
    api.log("instr=%s" % api.instruction())
    table, bowls, ramekin, plate, XYZ = perceive(api)
    tgt = choose(api, bowls, ramekin, plate)
    if tgt is None or plate is None:
        api.log("ABORT bowls=%d plate=%s" % (len(bowls), plate is not None))
        return
    api.log("TARGET %.3f %.3f" % (tgt["x"], tgt["y"]))

    held = False
    for k in range(N_TRY):
        off = OFF
        tx, ty = tgt["x"], tgt["y"]
        w, e = grab(api, table, tx, ty, off)
        if w <= W_EMPTY and k < N_TRY - 1:
            # nothing at all in the jaws: nothing was disturbed either, so
            # re-fly from the hover without paying for a lift and a scan
            api.log("try%d closew=%.4f -> empty close, re-fly" % (k, w))
            api.grip(0.08)
            mv(api, e[0], e[1], table + 0.14, 1.0)
            table2, bw2, ra2, pl2, XYZ2 = perceive(api)
            cand = [b for b in bw2 if np.hypot(b["x"] - tx, b["y"] - ty) < 0.10]
            if cand:
                tgt = cand[0]
                api.log("re-locate -> %.3f %.3f" % (tgt["x"], tgt["y"]))
            continue
        mv(api, e[0], e[1], table + 0.075, 0.9)
        mv(api, e[0], e[1], table + Z_HOVER, 1.2)
        api.settle(0.5)
        e = api.eef()
        table2, bw2, ra2, pl2, XYZ2 = perceive(api)
        still = [b for b in bw2 if np.hypot(b["x"] - tx, b["y"] - ty) < 0.045]
        sl = profile(api, table, XYZ2, e)
        body = [s for s in sl if 1 <= s[0] <= 5]
        pn = sum(s[1] for s in body)
        api.log("try%d closew=%.4f still=%d payload_n=%d" % (
            k, w, len(still), pn))
        # an empty cell is not a grasp: a bowl shoved aside empties it too
        if pn >= PAYLOAD_MIN and not still:
            held = True
            break
        api.grip(0.08)
        cand = [b for b in bw2
                if np.hypot(b["x"] - tx, b["y"] - ty) < 0.14]
        if not cand:
            break
        tgt = min(cand, key=lambda b: np.hypot(b["x"] - tx, b["y"] - ty))
        api.log("re-locate -> %.3f %.3f" % (tgt["x"], tgt["y"]))
    if not held:
        api.log("NO PAYLOAD")
        return

    e = api.eef()
    if body:
        wsum = sum(s[1] for s in body)
        bx = sum(s[1] * s[2] for s in body) / wsum
        by = sum(s[1] * s[3] for s in body) / wsum
    else:
        bx, by = e[0], e[1]
    ox = float(np.clip(bx - e[0], -0.06, 0.06))
    oy = float(np.clip(by - e[1], -0.06, 0.06))
    api.log("carry_off=%+.4f %+.4f" % (ox, oy))

    ex, ey = plate["x"] - ox + X_BIAS, plate["y"] - oy
    mv2(api, ex, ey, table + Z_HOVER, 2.0, 1.2)
    api.log("over_plate=%s" % np.round(api.eef(), 4).tolist())
    mv(api, ex, ey, table + Z_REL, 1.1)
    api.log("at_release=%s" % np.round(api.eef(), 4).tolist())
    api.grip(0.08)
    api.settle(1.2)
    e3 = api.eef()
    mv(api, e3[0] - 0.10, e3[1] - 0.10, table + 0.24, 1.3)
    api.settle(0.8)
    table3, bw3, ra3, pl3, _ = perceive(api)
    for b in bw3:
        api.log("FINAL bowl %.3f %.3f  d_plate=%.3f" % (
            b["x"], b["y"],
            float(np.hypot(b["x"] - plate["x"], b["y"] - plate["y"]))))
    api.log("v22b done")
