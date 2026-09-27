"""c2k1clean / spa_bowl_top_drawer_cabinet_pos_k1

"pick up the black bowl in the top drawer of the wooden cabinet and place it
on the plate".

Mechanism
---------
Two identical bowls sit on the cabinet: one on the cabinet top, one inside the
open top drawer.  Both are found as ring-plus-hole rims in a top-down max-z
map; the drawer one is the LOWER of the two (the drawer floor sits below the
cabinet top), which is what "in the top drawer" means geometrically.

The bowl is 11 cm across, far wider than the 78 mm jaw span, so the only grasp
is a straddle of its rim.  Measured on the debug seeds, the free space around
the rim inside the drawer is ~5.5 cm along world x and only ~2 cm along world
y (the drawer's side walls stand as high as the bowl rim).  The jaws separate
along the tool's y axis, which is world y at the home wrist, so the wrist is
yawed 90 deg about world z and the rim is straddled at its +x extreme.
"""
import numpy as np

PROVENANCE = {
    "GRID_CELL_M": {
        "source": "probe resolution chosen for the 1 cm top-down map; not a "
                  "scene measurement",
        "allowed": True},
    "RIM_RADIUS_RANGE": {
        "source": "debug seeds 51-65, cam_high depth: the drawer bowl's rim "
                  "circle fits at r = 0.053 +/- 0.001 m, so the search runs "
                  "0.040-0.066 m",
        "allowed": True},
    "CABINET_MIN_HEIGHT": {
        "source": "debug seeds 51-65: both cabinet bowls have rims 0.215 m "
                  "and 0.278 m above the modal table plane, while every table "
                  "prop tops out at 0.043 m; 0.12 m separates the two groups",
        "allowed": True},
    "DEPTH_LADDER": {
        "source": "pack demo0 t=48: eef z 1.0917 with the jaws closing on the "
                  "rim, i.e. 0.024 m below the measured rim height of this "
                  "task's bowl (debug seeds: rim = table+0.215); the +-0.008 m "
                  "rungs span the band between the rim lip and the drawer "
                  "floor measured on the debug seeds",
        "allowed": True},
    "HOLD_MIN_W": {
        "source": "v4 on debug seeds 51-65: after the lift the finger gap is "
                  "0.0033-0.0073 m whenever the bowl came along and 0.0010 m "
                  "when it did not, so 0.0025 m separates hold from drop",
        "allowed": True},
    "DRAWER_FLOOR_OFFSET": {
        "source": "debug seeds 51-65, cam_high depth cross-sections: the "
                  "drawer floor the bowl rests on is 0.165 m above the modal "
                  "table plane",
        "allowed": True},
    "EEF_IS_FINGERTIP": {
        "source": "pack demo0: the grasp eef z (1.0917) is only 0.026 m above "
                  "the drawer floor measured on the debug seeds (table+0.165), "
                  "so the reported eef is the point between the fingertips, "
                  "not the wrist",
        "allowed": True},
    "JAW_HALF_SPAN_M": {
        "source": "debug-seed observation: api.gripper() reports 0.0778 m with "
                  "the jaws open, so each finger sits 0.039 m off the tool axis",
        "allowed": True},
    "PLATE_MAX_HEIGHT": {
        "source": "debug seeds 51-65: the plate's top is 0.020 m above the "
                  "table and spans 0.14-0.15 m; the only rivals are the "
                  "ramekin (0.043 m) and the cookie box (0.020 m, half the "
                  "footprint)",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug seeds: the cabinet top, the drawer walls and the "
                  "second bowl top out at table+0.278, so the carry runs at "
                  "table+0.37",
        "allowed": True},
    "RELEASE_CLEARANCE_M": {
        "source": "pack demo0 t=139 releases 0.045 m above its table with the "
                  "bowl hanging 0.026 m below the fingertips (bowl height "
                  "0.050 m measured on the debug seeds minus the 0.024 m "
                  "grasp depth)",
        "allowed": True},
}

CELL = 0.01
X0, X1 = -0.40, 0.45
Y0, Y1 = -0.50, 0.50
R_LO, R_HI = 0.040, 0.066
CABINET_MIN_H = 0.12
DEPTH_LADDER = (0.024, 0.032, 0.017)
HOLD_MIN_W = 0.0025
JAW_HALF = 0.039
PLATE_MAX_H = 0.026
CARRY_H = 0.37
REL_CLEAR = 0.005


# --------------------------------------------------------------- perception
def _cloud(f):
    dep = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = dep.shape[:2]
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    x = (uu - K[0, 2]) * dep / K[0, 0]
    y = (vv - K[1, 2]) * dep / K[1, 1]
    return (np.stack([x, y, dep, np.ones_like(dep)], -1) @ T.T)[..., :3]


def _build(api):
    f = api.capture("cam_high")
    P = _cloud(f)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (X > -0.1) & (X < 0.4) & (np.abs(Y) < 0.45) & np.isfinite(Z)
    hh, e = np.histogram(Z[m], bins=1100, range=(0.5, 1.6))
    zt = float(0.5 * (e[np.argmax(hh)] + e[np.argmax(hh) + 1]))
    nx = int(round((X1 - X0) / CELL))
    ny = int(round((Y1 - Y0) / CELL))
    gi = np.floor((X - X0) / CELL).astype(int)
    gj = np.floor((Y - Y0) / CELL).astype(int)
    ok = (gi >= 0) & (gi < nx) & (gj >= 0) & (gj < ny) & np.isfinite(Z)
    H = np.full(nx * ny, -np.inf)
    np.maximum.at(H, (gi * ny + gj)[ok], Z[ok])
    H[~np.isfinite(H)] = np.nan
    return zt, H.reshape(nx, ny)


def _ring(H, cx, cy, r, n=36):
    a = np.arange(n) * 2 * np.pi / n
    ii = np.round((cx + r * np.cos(a) - X0) / CELL - 0.5).astype(int)
    jj = np.round((cy + r * np.sin(a) - Y0) / CELL - 0.5).astype(int)
    nx, ny = H.shape
    m = (ii >= 0) & (ii < nx) & (jj >= 0) & (jj < ny)
    v = np.full(n, np.nan)
    v[m] = H[ii[m], jj[m]]
    return v


def _disc(H, cx, cy, r):
    nx, ny = H.shape
    k = int(np.ceil(r / CELL))
    i0 = int((cx - X0) / CELL)
    j0 = int((cy - Y0) / CELL)
    out = []
    for a in range(-k, k + 1):
        for b in range(-k, k + 1):
            if np.hypot(a, b) * CELL <= r and 0 <= i0 + a < nx and 0 <= j0 + b < ny:
                out.append(H[i0 + a, j0 + b])
    v = np.asarray(out, float)
    return v[np.isfinite(v)]


def _bowl_candidates(H, zmin, zmax):
    """ring-plus-hole rims -> (score, cx, cy, r, z_rim, z_in, out_frac, on)."""
    nx, ny = H.shape
    cand = []
    radii = np.arange(R_LO, R_HI, 0.002)
    for i in range(nx):
        cx = X0 + (i + 0.5) * CELL
        for j in range(ny):
            cy = Y0 + (j + 0.5) * CELL
            hin = _disc(H, cx, cy, 0.025)
            if hin.size < 12:
                continue
            zin = float(np.median(hin))
            if not (zmin - 0.10 < zin < zmax):
                continue
            best = None
            for r in radii:
                v = _ring(H, cx, cy, r)
                f = np.isfinite(v)
                if f.sum() < 30:
                    continue
                zr = float(np.median(v[f]))
                if not (zmin <= zr <= zmax) or zr - zin < 0.030:
                    continue
                on = int((np.abs(v[f] - zr) < 0.008).sum())
                if on < 22:
                    continue
                vo = _ring(H, cx, cy, r + 0.030)
                fo = np.isfinite(vo)
                if fo.sum() < 20:
                    continue
                olow = float((vo[fo] < zr - 0.020).mean())
                if olow < 0.42:
                    continue
                sc = on / 36.0 + min(zr - zin, 0.08) * 4 + olow
                if best is None or sc > best[0]:
                    best = (sc, r, zr, zin, olow, on)
            if best:
                cand.append((best[0], cx, cy, best[1], best[2], best[3],
                             best[4], best[5]))
    cand.sort(key=lambda t: -t[0])
    keep = []
    for c in cand:
        if all(np.hypot(c[1] - k[1], c[2] - k[2]) > 0.05 for k in keep):
            keep.append(c)
    return keep


def _kasa(px, py):
    A = np.stack([px, py, np.ones_like(px)], 1)
    b = px ** 2 + py ** 2
    sol = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = sol[0] / 2, sol[1] / 2
    return float(cx), float(cy), float(np.sqrt(max(sol[2] + cx * cx + cy * cy, 1e-9)))


def _refine(H, cx0, cy0, zr0):
    """Fit the rim circle; returns (cx, cy, r, z_rim)."""
    nx, ny = H.shape
    ii, jj = np.mgrid[0:nx, 0:ny]
    Xg = X0 + (ii + 0.5) * CELL
    Yg = Y0 + (jj + 0.5) * CELL
    cx, cy, r, zr = cx0, cy0, 0.053, zr0
    for _ in range(3):
        prof = []
        for rr in np.arange(R_LO, R_HI, 0.0025):
            v = _ring(H, cx, cy, rr, 48)
            v = v[np.isfinite(v)]
            if v.size > 24:
                prof.append((float(np.percentile(v, 70)), rr))
        if not prof:
            break
        zr2 = max(prof)[0]
        m = np.isfinite(H) & (np.hypot(Xg - cx, Yg - cy) < 0.075) & (np.abs(H - zr2) < 0.006)
        if m.sum() < 12:
            break
        a, b, c = _kasa(Xg[m], Yg[m])
        if np.hypot(a - cx0, b - cy0) > 0.030 or not (R_LO <= c <= R_HI):
            break
        keep = np.abs(np.hypot(Xg[m] - a, Yg[m] - b) - c) < 0.010
        if keep.sum() < 12:
            break
        cx, cy, r = _kasa(Xg[m][keep], Yg[m][keep])
        zr = zr2
    return cx, cy, r, zr


def _clearance(H, cx, cy, r, zr, dx, dy):
    """Free radial run outside the rim along (dx,dy), in metres."""
    nx, ny = H.shape
    for k in range(26):
        dd = 0.014 + 0.005 * k
        hits = 0
        for t in (-0.02, 0.0, 0.02):
            px = cx + dx * (r + dd) - dy * t
            py = cy + dy * (r + dd) + dx * t
            i = int((px - X0) / CELL)
            j = int((py - Y0) / CELL)
            if not (0 <= i < nx and 0 <= j < ny):
                hits += 1
                continue
            h = H[i, j]
            if np.isfinite(h) and h > zr - 0.020:
                hits += 1
        if hits >= 2:
            return dd
    return 0.135


def _components(mask):
    nx, ny = mask.shape
    lab = np.zeros(mask.shape, int)
    n = 0
    for i in range(nx):
        for j in range(ny):
            if mask[i, j] and not lab[i, j]:
                n += 1
                st = [(i, j)]
                lab[i, j] = n
                while st:
                    a, b = st.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if 0 <= p < nx and 0 <= q < ny and mask[p, q] and not lab[p, q]:
                                lab[p, q] = n
                                st.append((p, q))
    return lab, n


def _plate(H, zt):
    m = np.isfinite(H) & (H > zt + 0.006) & (H < zt + PLATE_MAX_H)
    lab, n = _components(m)
    best = None
    for k in range(1, n + 1):
        ii, jj = np.where(lab == k)
        if ii.size < 110:
            continue
        ex = (ii.max() - ii.min() + 1) * CELL
        ey = (jj.max() - jj.min() + 1) * CELL
        if not (0.10 <= ex <= 0.22 and 0.10 <= ey <= 0.22):
            continue
        if abs(ex - ey) > 0.05:
            continue
        x = X0 + (ii.mean() + 0.5) * CELL
        y = Y0 + (jj.mean() + 0.5) * CELL
        top = float(np.percentile(H[ii, jj], 95))
        if best is None or ii.size > best[0]:
            best = (ii.size, x, y, top)
    return best


# ------------------------------------------------------------------ control
def _rz(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a), 0.0],
                     [np.sin(a), np.cos(a), 0.0],
                     [0.0, 0.0, 1.0]])


def _ang_err(A, B):
    c = (np.trace(A @ B.T) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


def _go(api, tag, xyz, seconds, rotation=None):
    """One move that always leaves a receipt: residual + where it stopped."""
    res = api.move(xyz, rotation=rotation, seconds=seconds)
    api.log("mv %-8s tgt=(%.3f,%.3f,%.3f) s=%.1f res=%.4f eef=%s"
            % (tag, xyz[0], xyz[1], xyz[2], seconds, res,
               np.round(api.eef(), 4).tolist()))
    return res


def run(api):
    api.log("instruction: %s" % api.instruction())
    zt, H = _build(api)
    api.log("table_z %.4f" % zt)

    cands = _bowl_candidates(H, zt + CABINET_MIN_H, zt + 0.35)
    for c in cands[:5]:
        api.log("cand sc=%.2f c=(%.3f,%.3f) r=%.3f rim=+%.3f in=+%.3f out=%.2f on=%d"
                % (c[0], c[1], c[2], c[3], c[4] - zt, c[5] - zt, c[6], c[7]))
    if not cands:
        api.log("ABORT no bowl candidate")
        return
    solid = [c for c in cands if c[7] >= 28]
    tgt = min(solid, key=lambda c: c[4]) if solid else max(cands, key=lambda c: c[7])
    cx, cy, r, zr = _refine(H, tgt[1], tgt[2], tgt[4])
    api.log("BOWL c=(%.4f,%.4f) r=%.4f rim=%.4f (+%.4f)" % (cx, cy, r, zr, zr - zt))

    cl_p = _clearance(H, cx, cy, r, zr, 1.0, 0.0)
    cl_m = _clearance(H, cx, cy, r, zr, -1.0, 0.0)
    api.log("clearance +x=%.3f -x=%.3f" % (cl_p, cl_m))
    side = 1.0 if cl_p >= cl_m else -1.0

    pl = _plate(H, zt)
    if pl is None:
        api.log("ABORT no plate")
        return
    _, px, py, ptop = pl
    api.log("PLATE c=(%.3f,%.3f) top=%.4f (+%.4f)" % (px, py, ptop, ptop - zt))

    rot0 = np.asarray(api.tool_rotation(), float)
    api.log("rot0 %s" % np.round(rot0, 3).tolist())
    R = _rz(90.0) @ rot0

    gx, gy = cx + side * r, cy
    zin_floor = zt + 0.165
    api.log("grasp side=%+.0f at (%.4f,%.4f) rim=%.4f" % (side, gx, gy, zr))

    api.grip(0.08)
    # One rotation-carrying move: move_pose exits only when BOTH position and
    # orientation converge, and the orientation criterion is never met here
    # (v4/v5 on the debug seeds: rot_err settles at 0.01-0.06 rad and the move
    # always spends its whole step budget), so the yaw is staged once and
    # every later move runs rotation=None, which holds the yaw to <0.10 rad.
    res = _go(api, "stage", [gx, gy, zr + 0.09], 2.5, rotation=R)
    cur = np.asarray(api.tool_rotation(), float)
    err = _ang_err(cur, R)
    api.log("stage rot_err=%.3f" % err)
    if err > 0.35:
        R = _rz(-90.0) @ rot0
        _go(api, "stage2", [gx, gy, zr + 0.09], 2.5, rotation=R)
        api.log("stage2 rot_err=%.3f"
                % _ang_err(np.asarray(api.tool_rotation(), float), R))

    held = False
    used_depth = DEPTH_LADDER[0]
    for att, depth in enumerate(DEPTH_LADDER):
        used_depth = depth
        tx = cx + side * r
        _go(api, "descend", [tx, cy, zr - depth], 1.6)
        api.grip(0.0)
        g = api.gripper()
        api.log("close#%d depth=%.3f width=%.4f effort=%.2f"
                % (att, depth, g["width_m"], g["effort"]))
        _go(api, "lift", [tx, cy, zr + 0.05], 1.0)
        g = api.gripper()
        api.log("lift#%d width=%.4f effort=%.2f" % (att, g["width_m"], g["effort"]))
        if g["width_m"] >= HOLD_MIN_W:
            held = True
            break
        if att + 1 < len(DEPTH_LADDER):
            api.grip(0.08)
            _go(api, "reopen", [tx, cy, zr + 0.09], 0.8)
            zt2, H2 = _build(api)
            c2 = _bowl_candidates(H2, zt2 + CABINET_MIN_H, zt2 + 0.35)
            if c2:
                s2 = [c for c in c2 if c[7] >= 28]
                t2 = min(s2, key=lambda c: c[4]) if s2 else max(c2, key=lambda c: c[7])
                cx, cy, r, zr = _refine(H2, t2[1], t2[2], t2[4])
                api.log("re-perceive c=(%.4f,%.4f) r=%.4f rim=+%.4f"
                        % (cx, cy, r, zr - zt2))
    if not held:
        api.log("GRASP FAILED after %d attempts" % len(DEPTH_LADDER))
        return

    # Step budget: a move that converges exits early, so a generous `seconds`
    # on a converging move is free; only a move that never converges spends
    # its whole budget.  v6 measured which is which on the debug seeds --
    # stage (rot_err never clears its tolerance) and retreat (blocked, res
    # 0.115) always run to the cap, so they are the only tight ones, while the
    # transport gets a wide budget because starving it is what lost ep52.
    # the bowl hangs (rim - drawer floor) - grasp_depth below the fingertips
    hang = (zr - zin_floor) - used_depth
    rel_z = ptop + hang + REL_CLEAR
    tx, ty = px + side * r, py
    _go(api, "carry", [cx + side * r, cy, zt + CARRY_H], 1.2)
    g = api.gripper()
    api.log("carry width=%.4f effort=%.2f rot_err=%.3f"
            % (g["width_m"], g["effort"],
               _ang_err(np.asarray(api.tool_rotation(), float), R)))
    _go(api, "over", [tx, ty, zt + CARRY_H], 3.0)
    _go(api, "lower", [tx, ty, rel_z], 1.8)
    g = api.gripper()
    api.log("place rel_z=%.4f hang=%.4f width=%.4f rot_err=%.3f"
            % (rel_z, hang, g["width_m"],
               _ang_err(np.asarray(api.tool_rotation(), float), R)))
    api.grip(0.08)
    api.settle(0.3)
    _go(api, "retreat", [tx, ty, rel_z + 0.12], 0.4)
    api.log("DONE")
