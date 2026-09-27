"""pick up the black bowl on the wooden cabinet and place it on the plate.

Everything metric is measured from cam_high RGB-D at the start of the episode;
the pack supplies only the *kind* of grasp (a rim pinch with the jaws closing
along base y, and how deep below the rim the wrist sits) and the fact that the
bowl travels to a plate.  The demo's own xy coordinates are NOT used: the pack
was recorded in a layout whose plate sits where a table bowl sits here.
"""
import numpy as np

PROVENANCE = {
    # -- grid ------------------------------------------------------------
    "CELL": {"source": "generic perception mechanics (1 cm top-down grid)",
             "allowed": True},
    "GRID_WINDOW": {
        "source": "debug seeds 51-65: every prop and the fixture fall inside "
                  "x[-0.50,0.40] y[-0.55,0.45] of the cam_high cloud",
        "allowed": True},
    # -- finding the bowl on the fixture ---------------------------------
    "FIXTURE_BAND_M": {
        "source": "debug seeds 51-65: the cabinet plateau measures 226 mm and "
                  "the bowl on it 279 mm above the table, while the arm's own "
                  "links read 370-470 mm",
        "allowed": True},
    "BOWL_STANDS_OFF_PLATEAU_M": {
        "source": "debug seeds 51-65: bowl rim 279 mm vs cabinet plateau "
                  "226 mm above the table plane", "allowed": True},
    # -- finding the plate ------------------------------------------------
    "PROP_BAND_M": {
        "source": "debug seeds 51-65: the table props top out at 19-51 mm "
                  "above the table; nothing else lies in that band",
        "allowed": True},
    "SHELL_M": {
        "source": "debug seeds 51-65: the plate's rim-to-floor drop is 13 mm "
                  "and every bowl's is 41-45 mm, so a 20 mm shell keeps a "
                  "plate whole and reduces a bowl to its rim ring",
        "allowed": True},
    "PROP_MIN_CELLS": {"source": "debug seeds 51-65: the plate covers 112-162 "
                                 "cells, noise blobs under 25", "allowed": True},
    "PROP_SPAN_M": {"source": "debug seeds 51-65: plate 0.11-0.12 m across, "
                              "bowls 0.09-0.12 m", "allowed": True},
    "PROP_TOP_MIN_M": {"source": "debug seeds 51-65: plate rim 49 mm, the flat "
                                 "cookie box only 19 mm", "allowed": True},
    "LUM_MIN": {"source": "debug seeds 51-65: plate/bowl tops read 115-141 mean "
                          "grey, the robot mount 22-25", "allowed": True},
    # -- grasp ------------------------------------------------------------
    "RIM_OFFSET_M": {
        "source": "debug seeds 51/53/55 grasp sweep (probe p7): the jaws close "
                  "along base y, and a wrist placed 50 mm from the measured "
                  "bowl centre on the +y side pinched the rim 3/3",
        "allowed": True},
    "RUNG_STEP_M": {
        "source": "debug seeds 51-65 (v1): the close gap varies 4.5-10.7 mm at "
                  "a fixed 50 mm offset, so +-6 mm spans the aim band",
        "allowed": True},
    "GRASP_DROP_M": {
        "source": "pack keyframe t=50 (ee z 1.159 while the rim top measures "
                  "1.180) plus the debug-seed press that self-limits at "
                  "rim - 4 mm", "allowed": True},
    "HOVER_M": {"source": "debug-seed measurement: 60 mm clears the 53 mm bowl "
                          "before the descent", "allowed": True},
    "LIFT_M": {"source": "debug-seed measurement: 90 mm lifts the bowl's 53 mm "
                         "body clear of the plateau", "allowed": True},
    "RETRY_LIFT_M": {"source": "debug-seed measurement: 120 mm clears the bowl "
                               "before the next rung", "allowed": True},
    "GRIP_GOOD_M": {
        "source": "debug seeds 51-65 (v1): closes of 4.5-6.6 mm crept loose "
                  "during the carry (seeds 52, 60); every close of 7.3 mm or "
                  "more survived", "allowed": True},
    # -- carry and release -------------------------------------------------
    "BOWL_HANG_M": {
        "source": "debug-seed measurement: bowl height 53 mm (rim 279 minus "
                  "plateau 226) and the wrist sits 4 mm below its rim, so the "
                  "carried bowl's base is 49 mm below the wrist",
        "allowed": True},
    "CARRY_Z_M": {"source": "debug-seed measurement: cabinet plateau top is "
                            "1.127, so a carry at 1.27 clears it with the bowl "
                            "hanging 49 mm below the wrist", "allowed": True},
    "HOP_M": {"source": "generic controller mechanics: move() sends "
                        "clip(err/0.05,-1,1), so legs under 50 mm stay out of "
                        "saturation", "allowed": True},
    "RELEASE_CLEAR_M": {
        "source": "debug-seed measurement: the plate floor sits 37-39 mm above "
                  "the table, so releasing the bowl 6 mm above it is a settle, "
                  "not a drop", "allowed": True},
    "POST_LIFT_M": {"source": "debug-seed measurement: 120 mm withdraws the "
                              "jaws clear of the placed bowl", "allowed": True},
}

CELL = 0.01
X0, Y0 = -0.50, -0.55
NX, NY = 90, 100
GRID_WINDOW = (X0, Y0, NX, NY)

FIXTURE_BAND_M = (0.15, 0.34)
BOWL_STANDS_OFF_PLATEAU_M = 0.025

PROP_BAND_M = (0.025, 0.12)
SHELL_M = 0.020
PROP_MIN_CELLS = 25
PROP_SPAN_M = (0.08, 0.22)
PROP_TOP_MIN_M = 0.035
LUM_MIN = 100.0

RIM_OFFSET_M = 0.050
RUNG_STEP_M = 0.006
GRASP_DROP_M = 0.010
HOVER_M = 0.06
LIFT_M = 0.09
RETRY_LIFT_M = 0.12
# A pinch that closes to less than this creeps loose during the carry
# (debug seed 60: 5.2 mm at the lift, 4.8 mm and empty by the plate; seed 52:
# 6.4 mm -> 3.4 mm).  Every pinch of 7.3 mm or more survived, 10/10.
GRIP_GOOD_M = 0.0070

BOWL_HANG_M = 0.049
CARRY_Z_M = 1.27
HOP_M = 0.045
RELEASE_CLEAR_M = 0.006
POST_LIFT_M = 0.12


# ---------------------------------------------------------------- perception
def _cloud(f):
    z = np.asarray(f.depth, float)
    H, W = z.shape[:2]
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vs, us = np.mgrid[0:H, 0:W]
    ok = np.isfinite(z) & (z > 0)
    zz = np.where(ok, z, 1.0)
    P = np.stack([(us - cx) * zz / fx, (vs - cy) * zz / fy, zz,
                  np.ones_like(zz)], axis=-1)
    return (P @ T.T)[..., :3], ok


def _label(mask):
    lab = np.zeros(mask.shape, int)
    cur = 0
    for i in range(mask.shape[0]):
        for j in range(mask.shape[1]):
            if mask[i, j] and not lab[i, j]:
                cur += 1
                st = [(i, j)]
                lab[i, j] = cur
                while st:
                    a, b = st.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if 0 <= p < mask.shape[0] and 0 <= q < mask.shape[1] \
                               and mask[p, q] and not lab[p, q]:
                                lab[p, q] = cur
                                st.append((p, q))
    return lab, cur


def _mid(ii, jj):
    return (X0 + (ii.min() + ii.max() + 1) * 0.5 * CELL,
            Y0 + (jj.min() + jj.max() + 1) * 0.5 * CELL)


def scene(api):
    """Top-down 1 cm grid of height-above-table and mean brightness."""
    f = api.capture("cam_high")
    P, ok = _cloud(f)
    rgb = np.asarray(f.rgb).astype(float)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    zs = Z[ok]
    hh, e = np.histogram(zs[(zs > 0.7) & (zs < 1.5)], bins=320, range=(0.7, 1.5))
    zt = float(e[int(np.argmax(hh))]) + 0.00125
    sel = ok & (X > X0) & (X < X0 + NX * CELL) & (Y > Y0) & (Y < Y0 + NY * CELL)
    gx = np.clip(((X - X0) / CELL).astype(int), 0, NX - 1)
    gy = np.clip(((Y - Y0) / CELL).astype(int), 0, NY - 1)
    idx = (gx * NY + gy)[sel]
    top = np.full(NX * NY, -9.0)
    np.maximum.at(top, idx, Z[sel])
    cnt = np.bincount(idx, minlength=NX * NY)
    lum = np.bincount(idx, weights=rgb.mean(-1)[sel], minlength=NX * NY) \
        / np.maximum(cnt, 1)
    return (top.reshape(NX, NY) - zt, cnt.reshape(NX, NY) > 0,
            lum.reshape(NX, NY), zt)


def find_bowl(Hh, valid):
    """The vessel standing on the tallest fixture plateau."""
    lab, n = _label(valid & (Hh > FIXTURE_BAND_M[0]) & (Hh < FIXTURE_BAND_M[1]))
    best, bm = -1, None
    for k in range(1, n + 1):
        m = lab == k
        if m.sum() > best:
            best, bm = m.sum(), m
    if bm is None:
        return None
    hist = np.bincount(np.clip((Hh[bm] * 200).astype(int), 0, 120))
    plateau = float(np.argmax(hist)) / 200.0
    bw = bm & (Hh > plateau + BOWL_STANDS_OFF_PLATEAU_M)
    lb, nb = _label(bw)
    if nb == 0:
        return None
    sizes = [(lb == k).sum() for k in range(1, nb + 1)]
    bw = lb == (int(np.argmax(sizes)) + 1)
    ii, jj = np.nonzero(bw)
    bx, by = _mid(ii, jj)
    rr = 0.25 * ((ii.max() - ii.min()) + (jj.max() - jj.min())) * CELL
    return bx, by, float(Hh[bw].max()), plateau, rr, int(bw.sum())


def find_plate(Hh, valid, lum):
    """The flat bright disc: a plate's top shell fills its bounding box, a
    bowl's is only a ring."""
    lab, n = _label(valid & (Hh > PROP_BAND_M[0]) & (Hh < PROP_BAND_M[1])
                   & (lum > LUM_MIN))
    best = None
    for k in range(1, n + 1):
        m = lab == k
        if m.sum() < PROP_MIN_CELLS:
            continue
        hmax = float(Hh[m].max())
        sh = m & (Hh > hmax - SHELL_M)
        ii, jj = np.nonzero(sh)
        w = (ii.max() - ii.min() + 1) * CELL
        d = (jj.max() - jj.min() + 1) * CELL
        if not (PROP_SPAN_M[0] <= w <= PROP_SPAN_M[1]
                and PROP_SPAN_M[0] <= d <= PROP_SPAN_M[1]
                and hmax > PROP_TOP_MIN_M):
            continue
        fill = sh.sum() / float((ii.max() - ii.min() + 1) * (jj.max() - jj.min() + 1))
        interior = float(np.median(Hh[sh]))
        px, py = _mid(ii, jj)
        if best is None or fill > best[0]:
            best = (fill, px, py, hmax, interior)
    return best


# ------------------------------------------------------------------- program
def held(api):
    g = api.gripper()
    return (float(g.get("effort", 0.0)) >= 2.5
            and float(g.get("width_m", 0.0)) > 0.005), g


def hop(api, target, step=HOP_M, seconds=0.5):
    """Walk to `target` in short legs.

    move() sends clip(err/0.05, -1, 1), so anything further than 50 mm away is
    commanded at full saturation and the carried bowl is jerked out of a thin
    rim pinch.  Legs shorter than that stay in the proportional regime.
    """
    target = np.asarray(target, float)
    res = 0.0
    while True:
        cur = np.asarray(api.eef(), float)
        d = target - cur
        n = float(np.linalg.norm(d))
        if n <= step:
            return api.move(target, seconds=seconds)
        res = api.move(cur + d * (step / n), seconds=seconds)
    return res


def try_rung(api, bx, by, zr, dx, dy):
    """One grasp attempt; returns (held, width)."""
    api.grip(0.08)
    api.move([bx + dx, by + dy, zr + HOVER_M], seconds=1.6)
    api.move([bx + dx, by + dy, zr - GRASP_DROP_M], seconds=1.1)
    api.grip(0.0)
    api.settle(0.2)
    api.move([bx + dx, by + dy, zr + LIFT_M], seconds=1.0)
    api.settle(0.2)
    ok, g = held(api)
    return ok, float(g.get("width_m", 0.0))


def run(api):
    Hh, valid, lum, zt = scene(api)
    p = find_plate(Hh, valid, lum)
    api.log("zt=%.4f plate=%s" % (zt, None if p is None else
                                  [round(float(v), 4) for v in p]))

    for cycle in range(2):
        Hh, valid, lum, zt = scene(api)
        b = find_bowl(Hh, valid)
        api.log("cycle%d bowl=%s" % (cycle, None if b is None else
                                     [round(float(v), 4) for v in b]))
        if b is None:
            api.log("no bowl on the plateau; stop")
            return
        bx, by, rim, plateau, rr, _n = b
        zr = zt + rim

        # ---- grasp: rim pinch, jaws along y.  Rungs span the aim band,
        #      and a pinch thinner than GRIP_GOOD_M is not carried. --------
        rungs = [(0.0, RIM_OFFSET_M), (0.0, RIM_OFFSET_M + RUNG_STEP_M),
                 (0.0, -RIM_OFFSET_M), (0.0, RIM_OFFSET_M - RUNG_STEP_M)]
        bestw, bestd, dx, dy = 0.0, None, None, None
        okgrip = False
        for k, (ddx, ddy) in enumerate(rungs):
            if k:   # the bowl may have been nudged by the previous rung
                Hh2, v2b, l2b, zt2b = scene(api)
                b2 = find_bowl(Hh2, v2b)
                if b2 is not None:
                    bx, by, rim = b2[0], b2[1], b2[2]
                    zr = zt2b + rim
            ok, w = try_rung(api, bx, by, zr, ddx, ddy)
            api.log("cycle%d rung%d d=(%.3f,%.3f) held=%s w=%.4f"
                    % (cycle, k, ddx, ddy, ok, w))
            if ok and w > bestw:
                bestw, bestd = w, (ddx, ddy)
            if ok and w >= GRIP_GOOD_M:
                dx, dy, okgrip = ddx, ddy, True
                break
            api.grip(0.08)
            api.move([bx + ddx, by + ddy, zr + RETRY_LIFT_M], seconds=1.0)
        if not okgrip:
            if bestd is None:
                api.log("no rung held; stop")
                return
            # nothing reached the safe band: take the firmest one seen
            Hh2, v2b, l2b, zt2b = scene(api)
            b2 = find_bowl(Hh2, v2b)
            if b2 is not None:
                bx, by, zr = b2[0], b2[1], zt2b + b2[2]
            ok, w = try_rung(api, bx, by, zr, bestd[0], bestd[1])
            api.log("cycle%d redo best d=%s held=%s w=%.4f" % (cycle, bestd, ok, w))
            if not ok:
                continue
            dx, dy = bestd

        if p is None:
            api.log("no plate found; holding")
            return
        fill, px, py, prim, pfloor = p
        # the bowl hangs RIM_OFFSET_M in -y from the wrist, so shift the
        # release the same way to land it centred on the plate
        rx, ry = px, py + RIM_OFFSET_M
        gx, gy = bx + dx, by + dy
        hop(api, [gx, gy, CARRY_Z_M])
        hop(api, [rx, ry, CARRY_Z_M])
        okh, g = held(api)
        api.log("over-plate eef=%s grip=%s"
                % (np.round(api.eef(), 4).tolist(),
                   {kk: round(float(v), 4) for kk, v in g.items()}))
        if not okh:
            api.log("lost the bowl in transit; re-perceiving")
            continue
        zdrop = zt + pfloor + BOWL_HANG_M + RELEASE_CLEAR_M
        hop(api, [rx, ry, zdrop])
        api.settle(0.2)
        okh, g = held(api)
        api.log("at release z=%.3f eef=%s grip=%s"
                % (zdrop, np.round(api.eef(), 4).tolist(),
                   {kk: round(float(v), 4) for kk, v in g.items()}))
        api.grip(0.08)
        api.settle(0.3)
        api.move([rx, ry, zdrop + POST_LIFT_M], seconds=1.2)
        api.settle(0.3)
        H2, v2, l2, zt2 = scene(api)
        lab, n = _label(v2 & (H2 > 0.05) & (H2 < 0.20))
        for k in range(1, n + 1):
            m = lab == k
            if m.sum() < 15:
                continue
            ii, jj = np.nonzero(m)
            cx, cy = _mid(ii, jj)
            api.log("post k=%d n=%d c=(%.3f,%.3f) hmax=%.3f"
                    % (k, int(m.sum()), cx, cy, float(H2[m].max())))
        api.log("done")
        return
