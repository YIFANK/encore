"""c2k1clean / goal_put_wine_top_cabinet_pos_k1 -- v1.

Intent: "put the wine bottle on top of the cabinet".

Mechanism (all of it re-derived from this cell's pack + debug seeds 51-65):
  * the pack's single demo grips the bottle NECK (closed finger gap 0.0165 m)
    at 0.123 m above its support, lifts, and releases 0.1235 m above the
    support it places on -- i.e. the bottle hangs from the eef by ~0.123 m and
    the release height is (target surface) + (hang).
  * the demo's xy anchors are decoys: this suite swaps the fixtures (in the
    pack images the cabinet is at -y, on debug seeds it is at +y) and _pos
    moves the props, so both the bottle and the cabinet top are perceived
    from cam_high RGB-D every episode.
  * cabinet top = the largest FLAT slab above the table; the wine rack, the
    other tall fixture, is a set of tilted slats and has no large flat set.
  * bottle = the tallest small-footprint object standing on the table; its
    neck centre is recovered from the cross-view (y) chord, which is unbiased,
    plus the near-arc apex in x.
"""
import json

import numpy as np

PROVENANCE = {
    "GRASP_BELOW_TOP": {
        "source": "pack demo0: close commanded between ee_path6 t=30 (z=1.0404) "
                  "and t=40 (z=1.0223), keyframe t=32 z=1.0291; debug seeds "
                  "51/57/65 measure the bottle top at z=1.059 -> grip sits "
                  "0.030-0.037 m below the bottle top; 0.033 taken.",
        "allowed": True},
    "HANG_M": {
        "source": "pack demo0: grasp z 1.023-1.029 with the bottle standing on "
                  "its support; debug-seed cam_high depth puts that support "
                  "(table) at z=0.901 -> the bottle base hangs 0.123-0.128 m "
                  "below the eef. Cross-checked against the demo release "
                  "z=1.2515 over a 1.128 cabinet top (0.1235).",
        "allowed": True},
    "NECK_GAP_M": {
        "source": "pack demo0 keyframe t=85 gripper_state [0.0049,-0.0116] -> "
                  "closed finger gap 0.0165 m; debug seeds measure the bottle "
                  "neck chord at 0.014-0.016 m.",
        "allowed": True},
    "TABLE_BAND": {
        "source": "debug-seed cam_high depth: modal z over the workspace is "
                  "0.901 (133k of 214k points).",
        "allowed": True},
    "SLAB_MIN_H": {
        "source": "debug seeds 51-65: bottle top is table+0.159, bowl top "
                  "table+0.052; the cabinet top slab sits at table+0.227. "
                  "0.18 separates the fixtures from every tabletop prop.",
        "allowed": True},
    "SMALL_FOOTPRINT_M2": {
        "source": "debug seeds: the bottle occupies ~48 cells of 25 mm^2 "
                  "(0.0012 m^2); the rack 1700 and the cabinet 2000.",
        "allowed": True},
    "GRID_RES": {"source": "chosen resolution for the top-down height map "
                           "(generic perception mechanics)", "allowed": True},
    "CARRY_CLEAR_M": {
        "source": "pack demo0 ee_path peaks at z=1.2945, i.e. 0.043 above its "
                  "release height 1.2515; 0.045 used.",
        "allowed": True},
    "HOVER_M": {"source": "generic: pre-grasp standoff above the bottle top, "
                          "clear of the 0.159 m bottle", "allowed": True},
}

GRASP_BELOW_TOP = 0.033
HANG_M = 0.1255
NECK_GAP_M = 0.0165
SLAB_MIN_H = 0.18
SMALL_FOOTPRINT_M2 = 0.006
GRID_RES = 0.005
CARRY_CLEAR_M = 0.045
HOVER_M = 0.085

XLO, XHI, YLO, YHI = -0.55, 0.30, -0.50, 0.50


# ---------------------------------------------------------------- perception
def _cloud(f):
    d = np.asarray(f.depth, dtype=float)
    K = np.asarray(f.intrinsics, dtype=float)
    T = np.asarray(f.t_base_cam, dtype=float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 0)
    z = np.where(ok, d, 1.0)
    x = (uu - K[0, 2]) * z / K[0, 0]
    y = (vv - K[1, 2]) * z / K[1, 1]
    P = np.stack([x, y, z, np.ones_like(z)], -1) @ T.T
    return P[..., :3], ok


def _components(mask):
    """4-connected components of a 2-D boolean grid -> list of (rows, cols)."""
    lab = np.zeros(mask.shape, np.int32)
    out = []
    cur = 0
    stack = []
    for s0, s1 in zip(*np.nonzero(mask)):
        if lab[s0, s1]:
            continue
        cur += 1
        lab[s0, s1] = cur
        stack.append((s0, s1))
        pts = []
        while stack:
            a, b = stack.pop()
            pts.append((a, b))
            for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                na, nb = a + da, b + db
                if (0 <= na < mask.shape[0] and 0 <= nb < mask.shape[1]
                        and mask[na, nb] and not lab[na, nb]):
                    lab[na, nb] = cur
                    stack.append((na, nb))
        p = np.array(pts)
        out.append((p[:, 0], p[:, 1]))
    return out


def perceive(api):
    f = api.capture("cam_high")
    P, ok = _cloud(f)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    box = ok & (X > XLO) & (X < XHI) & (Y > YLO) & (Y < YHI)

    # table height: modal z of the workspace
    zt = Z[box & (Z > 0.5) & (Z < 1.05)]
    hist, edges = np.histogram(zt, bins=np.arange(0.5, 1.05, 0.002))
    table_z = float(edges[int(np.argmax(hist))] + 0.001)

    # drop the arm: it is the only thing well above every fixture, and it sits
    # over the eef.
    e0 = np.asarray(api.eef(), float)
    arm = ((Z > table_z + 0.25)
           & (np.hypot(X - e0[0], Y - e0[1]) < 0.20))
    m = box & ~arm & (Z > table_z + 0.02)

    nx = int((XHI - XLO) / GRID_RES) + 1
    ny = int((YHI - YLO) / GRID_RES) + 1
    ix = ((X[m] - XLO) / GRID_RES).astype(int)
    iy = ((Y[m] - YLO) / GRID_RES).astype(int)
    zs = Z[m]
    order = np.argsort(zs)
    H = np.full((nx, ny), np.nan)
    H[ix[order], iy[order]] = zs[order]

    # -- cabinet top: the largest FLAT set among the tall fixtures -----------
    best = None
    for rr, cc in _components(np.nan_to_num(H, nan=-9) > table_z + SLAB_MIN_H):
        if len(rr) < 40:
            continue
        zc = H[rr, cc]
        hb, he = np.histogram(zc, bins=np.arange(table_z + SLAB_MIN_H,
                                                 zc.max() + 0.01, 0.004))
        zmode = float(he[int(np.argmax(hb))] + 0.002)
        flat = np.abs(zc - zmode) < 0.007
        if best is None or int(flat.sum()) > best[0]:
            best = (int(flat.sum()), float(np.median(zc[flat])),
                    XLO + rr[flat] * GRID_RES, YLO + cc[flat] * GRID_RES)
    if best is None:
        raise RuntimeError("no cabinet slab found")
    n_flat, cab_z, cab_x, cab_y = best
    cab = (float(np.median(cab_x)), float(np.median(cab_y)), cab_z)
    api.log("CAB n=%d z=%.4f x[%.3f,%.3f] y[%.3f,%.3f] cen=(%.3f,%.3f)"
            % (n_flat, cab_z, cab_x.min(), cab_x.max(), cab_y.min(), cab_y.max(),
               cab[0], cab[1]))

    # -- bottle: tallest small-footprint prop standing on the table ----------
    cand = None
    for rr, cc in _components(np.nan_to_num(H, nan=-9) > table_z + 0.05):
        area = len(rr) * GRID_RES * GRID_RES
        ztop = float(H[rr, cc].max())
        if area > SMALL_FOOTPRINT_M2 or ztop < table_z + 0.10:
            continue
        if cand is None or ztop > cand[0]:
            cand = (ztop, XLO + rr * GRID_RES, YLO + cc * GRID_RES, area)
    if cand is None:
        raise RuntimeError("no bottle found")
    ztop, bxs, bys, area = cand
    cx0, cy0 = float(np.median(bxs)), float(np.median(bys))
    api.log("BOTTLE coarse cen=(%.3f,%.3f) ztop=%.3f area=%.5f"
            % (cx0, cy0, ztop, area))

    # neck refinement: the y chord is cross-view (unbiased); x comes from the
    # near-arc apex minus the chord radius.
    z_grip = ztop - GRASP_BELOW_TOP
    sel = (ok & (np.abs(X - cx0) < 0.06) & (np.abs(Y - cy0) < 0.06)
           & (Z > z_grip - 0.012) & (Z < z_grip + 0.012))
    if int(sel.sum()) >= 12:
        xs, ys = X[sel], Y[sel]
        ylo, yhi = float(np.percentile(ys, 2)), float(np.percentile(ys, 98))
        r = max(0.004, min(0.022, (yhi - ylo) / 2.0))
        bx = 0.5 * (float(np.percentile(xs, 2)) + float(np.percentile(xs, 98)) - r)
        by = 0.5 * (ylo + yhi)
        api.log("NECK n=%d r=%.4f cen=(%.4f,%.4f) z=%.4f"
                % (int(sel.sum()), r, bx, by, z_grip))
    else:
        bx, by = cx0, cy0
        api.log("NECK fallback n=%d" % int(sel.sum()))
    return table_z, (bx, by, ztop), cab


# -------------------------------------------------------------------- policy
def run(api):
    table_z, bottle, cab = perceive(api)
    bx, by, ztop = bottle
    cx, cy, cab_z = cab
    api.log("table_z=%.4f bottle=(%.4f,%.4f,%.4f) cab=(%.4f,%.4f,%.4f)"
            % (table_z, bx, by, ztop, cx, cy, cab_z))

    z_grip = ztop - GRASP_BELOW_TOP
    hang = z_grip - table_z
    carry_z = cab_z + hang + CARRY_CLEAR_M
    place_z = cab_z + hang + 0.002

    api.grip(0.08)
    r = api.move([bx, by, ztop + HOVER_M], seconds=2.0)
    api.log("hover res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    r = api.move([bx, by, z_grip], seconds=2.0)
    api.log("descend res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))

    api.grip(0.0)
    g = api.gripper()
    api.log("closed %s (neck expect %.4f)" % (json.dumps(g), NECK_GAP_M))

    r = api.move([bx, by, carry_z], seconds=2.0)
    api.log("lift res=%.4f eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), json.dumps(api.gripper())))

    r = api.move([cx, cy, carry_z], seconds=3.0)
    api.log("transit res=%.4f eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), json.dumps(api.gripper())))

    r = api.move([cx, cy, place_z], seconds=2.0)
    api.log("place res=%.4f eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), json.dumps(api.gripper())))

    api.grip(0.08)
    api.settle(0.5)
    api.log("released eef=%s grip=%s"
            % (np.round(api.eef(), 4).tolist(), json.dumps(api.gripper())))

    e = np.asarray(api.eef(), float)
    api.move([e[0], e[1], carry_z + 0.02], seconds=1.5)
    api.settle(0.5)
    api.log("retreat eef=%s" % np.round(api.eef(), 4).tolist())
    return "v1 done"
