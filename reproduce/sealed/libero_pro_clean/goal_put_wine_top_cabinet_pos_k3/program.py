"""c2clean / goal_put_wine_top_cabinet_pos_k3 -- v1

Mechanism taken from the K=3 pack: top-down close on the wine bottle's neck,
lift, carry above the cabinet, lower until the bottle's base is just over the
cabinet's top face, open.

All positions are perceived from cam_high on this episode.  The pack supplies
only *relative* quantities (how far below the bottle's top to close, how high
to carry, how far above a support to release) -- its absolute xy is a decoy,
because this is a _pos cell and the debug layout is mirrored w.r.t. the pack.
"""

import numpy as np

PROVENANCE = {
    "GRASP_BELOW_TOP": {
        "source": "pack demos: grasp eef z 1.029/0.992/1.024 vs the bottle top "
                  "measured at 1.056-1.059 on debug seeds 51-65 -> 0.029/0.066/"
                  "0.034 below the cork; 0.042 is the middle of that range",
        "allowed": True},
    "CARRY_Z": {
        "source": "pack ee_path6 peak z over the carry: 1.2945 / 1.2828 / 1.2966",
        "allowed": True},
    "RELEASE_CLEAR": {
        "source": "pack demos: release eef z minus the grasp height above the "
                  "bottle's base puts the base at 1.124/1.118/1.132 against a "
                  "support whose top this cell measures at 1.120 -> a 0.004-0.012 "
                  "drop clearance",
        "allowed": True},
    "HOVER": {
        "source": "pack demo0 ee_path6 descends from 1.1543 to the close at "
                  "1.0223, i.e. it approaches from ~0.10 m above the grasp",
        "allowed": True},
    "TABLE_BAND": {
        "source": "debug-seed cam_high depth: the table plane z=0.901 is the "
                  "modal height of the workspace (>128k px of 512x512)",
        "allowed": True},
    "BOTTLE_MIN_HEIGHT": {
        "source": "debug seeds 51-65: the bottle top is table+0.156..0.159; the "
                  "next tallest compact dark object on the table tops out at "
                  "table+0.059.  The +0.09 cut sits between them.",
        "allowed": True},
    "BOTTLE_SPAN": {
        "source": "debug seeds 51-65: the bottle's dark footprint measures "
                  "0.015-0.025 m in x by 0.025-0.030 m in y in every seed.  The "
                  "other tall dark returns (wooden rack edges, robot arm) are "
                  "slivers <=0.010 m in one axis or masses >0.10 m wide.",
        "allowed": True},
    "DARK_LUM": {
        "source": "debug-seed cam_high rgb: wine-bottle body/neck pixels have "
                  "mean channel luminance 1-25; the bare table is ~145",
        "allowed": True},
    "MED_LUM_MAX": {
        "source": "debug seeds 51-65: the bottle blob's median pixel luminance "
                  "is 2.0-2.3 in every seed.  Every other tall candidate is >=12 "
                  "(robot arm) or >=67 (wooden-rack edges).  8.0 sits in a gap "
                  "with 3x margin below and 8x above.",
        "allowed": True},
    "H_TOP_BAND": {
        "source": "debug seeds 51-65: the bottle's top sits table+0.153..0.157 "
                  "(the asset's own height; _pos moves it but cannot resize it). "
                  "The 0.10-0.22 band is a loose sanity bracket around that.",
        "allowed": True},
    "CAB_BAND": {
        "source": "debug-seed cam_high depth: a ~6.5k-px horizontal plane at "
                  "z=1.120 = table+0.219, the only large flat structure above "
                  "table+0.12; it matches the support height the pack implies "
                  "(release z minus grasp height = 1.124/1.118/1.132)",
        "allowed": True},
    "GRID_RES": {"source": "chosen resolution for footprint clustering", "allowed": True},
    "GRIP_OPEN": {"source": "FairApi: api.grip(>=0.025) opens", "allowed": True},
    "GRIP_CLOSE": {"source": "FairApi: api.grip(<0.025) closes", "allowed": True},
}

GRASP_BELOW_TOP = 0.042
CARRY_Z = 1.292
RELEASE_CLEAR = 0.008
HOVER = 0.100
BOTTLE_MIN_HEIGHT = 0.090
SPAN_MIN = 0.015
SPAN_MAX = 0.060
DARK_LUM = 90.0
MED_LUM_MAX = 8.0
H_TOP_MIN = 0.10
H_TOP_MAX = 0.22
CAB_MIN_ABOVE = 0.12
CAB_MAX_ABOVE = 0.34
RES = 0.005
GX0, GY0 = -0.60, -0.45
NX, NY = 190, 180


# ---------------------------------------------------------------- perception

def _cloud(f):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.nan_to_num(np.asarray(f.depth, float), nan=0.0, posinf=0.0, neginf=0.0)
    if d.ndim == 3:
        d = d[..., 0]
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    fx, cx, cy = K[0, 0], K[0, 2], K[1, 2]
    P = np.stack([(uu - cx) * d / fx, (vv - cy) * d / fx, d], -1)
    B = P @ T[:3, :3].T + T[:3, 3]
    return B[..., 0], B[..., 1], B[..., 2], d > 1e-6


def _peak(z, lo, hi, bw=0.004):
    sel = z[(z >= lo) & (z < hi)]
    if sel.size == 0:
        return None, 0
    n = max(1, int(round((hi - lo) / bw)))
    cnt, edge = np.histogram(sel, bins=n, range=(lo, hi))
    k = int(np.argmax(cnt))
    return float(0.5 * (edge[k] + edge[k + 1])), int(cnt[k])


def _components(mask):
    """4-connected labelling of a boolean grid (iterative flood fill)."""
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    H, W = mask.shape
    for p in map(tuple, np.argwhere(mask)):
        if lab[p]:
            continue
        cur += 1
        stack = [p]
        lab[p] = cur
        while stack:
            i, j = stack.pop()
            for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if 0 <= a < H and 0 <= b < W and mask[a, b] and lab[a, b] == 0:
                    lab[a, b] = cur
                    stack.append((a, b))
    return lab, cur


def _cellid(X, Y):
    ix = np.clip(((X - GX0) / RES).astype(int), 0, NX - 1)
    iy = np.clip(((Y - GY0) / RES).astype(int), 0, NY - 1)
    return ix * NY + iy


def _cluster(mask, X, Y):
    """Return [(cells, pixel_mask, sx, sy)] for each xy-connected blob of mask."""
    G = np.zeros((NX, NY), bool)
    cid = _cellid(X, Y)
    G.ravel()[cid[mask]] = True
    lab, n = _components(G)
    flat = lab.ravel()
    out = []
    for c in range(1, n + 1):
        cells = np.flatnonzero(flat == c)
        ix, iy = cells // NY, cells % NY
        sx = (ix.max() - ix.min() + 1) * RES
        sy = (iy.max() - iy.min() + 1) * RES
        pm = mask & np.isin(cid, cells)
        out.append((cells, pm, sx, sy))
    return out


def perceive(api):
    f = api.capture("cam_high")
    X, Y, Z, ok = _cloud(f)
    rgb = np.asarray(f.rgb, float)
    lum = rgb.mean(-1) if rgb.ndim == 3 else rgb

    work = ok & (X > -0.58) & (X < 0.34) & (np.abs(Y) < 0.44)

    table_z, ntab = _peak(Z[work & (Z > 0.80) & (Z < 1.00)], 0.80, 1.00)
    api.log("PERC table_z=%.4f n=%d" % (table_z, ntab))

    # --- cabinet top: the dominant horizontal plane well above the table.
    # Take the largest xy-connected blob of that plane so the robot arm, which
    # also crosses this height band, cannot drag the centroid.
    cab_pk, ncab = _peak(Z[work], table_z + CAB_MIN_ABOVE, table_z + CAB_MAX_ABOVE)
    plane = work & (np.abs(Z - cab_pk) < 0.012)
    blobs = _cluster(plane, X, Y)
    blobs.sort(key=lambda b: -len(b[0]))
    for cells, pm, sx, sy in blobs[:4]:
        api.log("PERC cabblob cells=%d span=(%.3f,%.3f) x=%.3f y=%.3f"
                % (len(cells), sx, sy, float(X[pm].mean()), float(Y[pm].mean())))
    pm = blobs[0][1]
    cab_z = float(np.median(Z[pm]))
    tx = float(0.5 * (np.percentile(X[pm], 2) + np.percentile(X[pm], 98)))
    ty = float(0.5 * (np.percentile(Y[pm], 2) + np.percentile(Y[pm], 98)))
    api.log("PERC cab_z=%.4f npx=%d x[%.3f,%.3f] y[%.3f,%.3f] -> tgt(%.3f,%.3f)"
            % (cab_z, int(pm.sum()), X[pm].min(), X[pm].max(),
               Y[pm].min(), Y[pm].max(), tx, ty))

    # --- bottle: a compact dark column standing well above everything else on
    # the table.  Slivers (rack edges) and the arm fail the two-sided footprint
    # test; among what is left the bottle is the tallest.
    dark = (work & (lum < DARK_LUM) & (Z > table_z + BOTTLE_MIN_HEIGHT)
            & (Z < cab_z - 0.02))
    best, fallback = None, None
    for cells, pmk, sx, sy in _cluster(dark, X, Y):
        if len(cells) < 6:
            continue
        ztop = float(Z[pmk].max())
        ml = float(np.median(lum[pmk]))
        span_ok = SPAN_MIN <= sx <= SPAN_MAX and SPAN_MIN <= sy <= SPAN_MAX
        keep = (span_ok and ml < MED_LUM_MAX
                and H_TOP_MIN <= ztop - table_z <= H_TOP_MAX)
        api.log("PERC cand cells=%d span=(%.3f,%.3f) ztop=%.4f h=%.3f medlum=%.1f "
                "x=%.3f y=%.3f keep=%d"
                % (len(cells), sx, sy, ztop, ztop - table_z, ml,
                   float(X[pmk].mean()), float(Y[pmk].mean()), keep))
        if keep and (best is None or len(cells) > best[0]):
            best = (len(cells), pmk)
        if span_ok and (fallback is None or ztop > fallback[0]):
            fallback = (ztop, pmk)
    if best is None:
        # nothing passed every gate; fall back to the tallest compact blob
        # rather than abandoning the episode
        api.log("PERC FALLBACK to tallest compact blob")
        best = fallback
    if best is None:
        raise RuntimeError("no bottle candidate")
    pmk = best[1]
    ztop = float(Z[pmk].max())

    # Neck band: a short slice below the cork where the silhouette is a clean
    # vertical cylinder.  y is unbiased (cam_high's u axis is exactly base +y);
    # x sees only the near surface, so back it off by the measured radius.
    band = pmk & (Z > ztop - 0.060) & (Z < ztop - 0.025)
    if band.sum() < 20:
        band = pmk & (Z > ztop - 0.080)
    by, bx = Y[band], X[band]
    r = float(0.5 * (by.max() - by.min()))
    y_c = float(0.5 * (by.min() + by.max()))
    x_c = float(bx.max()) - r
    api.log("PERC bottle ztop=%.4f npx=%d r=%.4f x=%.4f y=%.4f"
            % (ztop, int(band.sum()), r, x_c, y_c))
    return dict(table_z=table_z, cab_z=cab_z, tx=tx, ty=ty,
                bx=x_c, by=y_c, btop=ztop, r=r)


# ------------------------------------------------------------------- motion

def _go(api, name, xyz, seconds=2.0):
    res = api.move([float(v) for v in xyz], seconds=seconds)
    e = api.eef()
    api.log("MOVE %-8s tgt=(%.4f,%.4f,%.4f) got=(%.4f,%.4f,%.4f) res=%s"
            % (name, xyz[0], xyz[1], xyz[2], e[0], e[1], e[2], res))
    return e


def _grip_state(api, tag):
    g = api.gripper()
    api.log("GRIP %-9s width=%.4f effort=%.2f" % (tag, g["width_m"], g["effort"]))
    return g


def run(api):
    api.log("INSTR %r" % (api.instruction(),))
    s = perceive(api)

    grasp_z = s["btop"] - GRASP_BELOW_TOP
    hold_h = grasp_z - s["table_z"]          # grasp height above the bottle's base
    release_z = s["cab_z"] + hold_h + RELEASE_CLEAR
    api.log("PLAN grasp=(%.4f,%.4f,%.4f) hold_h=%.4f place=(%.4f,%.4f,%.4f)"
            % (s["bx"], s["by"], grasp_z, hold_h, s["tx"], s["ty"], release_z))

    api.grip(0.08)
    _go(api, "hover", (s["bx"], s["by"], grasp_z + HOVER), 2.0)
    _go(api, "descend", (s["bx"], s["by"], grasp_z), 2.0)
    api.grip(0.0)
    api.settle(0.6)
    _grip_state(api, "afterclose")

    _go(api, "lift", (s["bx"], s["by"], CARRY_Z), 2.5)
    _grip_state(api, "afterlift")

    _go(api, "over", (s["tx"], s["ty"], CARRY_Z), 3.0)
    _grip_state(api, "overcab")

    _go(api, "lower", (s["tx"], s["ty"], release_z), 2.5)
    _grip_state(api, "atplace")

    api.grip(0.08)
    api.settle(0.6)
    _go(api, "retreat", (s["tx"], s["ty"], CARRY_Z), 2.0)
    api.settle(0.4)
    _grip_state(api, "end")
    api.log("EEF_END %s" % ([float(v) for v in api.eef()],))
