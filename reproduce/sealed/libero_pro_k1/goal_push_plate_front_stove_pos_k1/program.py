"""push the plate to the front of the stove  --  c2k1clean, K=1.

Mechanism (read off the pack demo): the gripper stays OPEN the whole episode
(gripper_cmd == -1.0 at both keyframes), descends onto the plate and drags it
sideways with a constant downward press (the demo's raw actions hold dz around
-0.3..-0.5 while the eef z sits flat at 0.917).  So this is press-and-drag, not
a grasp.

Perception is a top-down height map built from cam_high depth:
  * table plane  -> modal z of the workspace point cloud
  * plate        -> largest connected component in the thin band just above
                    the table (the plate is a 2 cm disc ~0.14 m across)
  * stove        -> largest connected component in the wider above-table band
                    (a ~0.19 x 0.21 m slab standing ~0.032 m proud)
Goal = a point GOAL_DX in front of the stove's front edge, GOAL_DY past the
stove's own y centre-line -- the demo's final keyframe leaves the plate on the
centre-line, and a debug-seed bias sweep put the tight edge of the goal region
on the -y side, so the aim is nudged past it.  Overshoot is free: LIBERO ends
the episode the moment the predicate fires, so a drag that sweeps through the
region cannot leave it again.
"""

import numpy as np

PROVENANCE = {
    "RES": {"source": "generic: 5 mm height-map cell, my own choice",
            "allowed": True},
    "XR": {"source": "generic: workspace crop for the height map",
           "allowed": True},
    "YR": {"source": "generic: workspace crop for the height map",
           "allowed": True},
    "PLATE_BAND": {
        "source": "debug seeds 51-65 cam_high depth: table plane z=0.900, "
                  "plate top z=0.920 -> band table+0.010..table+0.024 isolates "
                  "the plate from the bowl (top table+0.052) and the stove "
                  "(top table+0.032)",
        "allowed": True},
    "PLATE_SPAN": {
        "source": "debug seeds 51-65: plate bbox measured 0.135 x 0.135 m",
        "allowed": True},
    "PLATE_BAND_WIDE": {
        "source": "debug seeds 51-65: slack around the measured plate band "
                  "(table+0.010..0.024), used only as a fallback when the "
                  "strict band finds nothing",
        "allowed": True},
    "PLATE_SPAN_WIDE": {
        "source": "debug seeds 51-65: slack around the measured 0.135 m plate "
                  "bbox, fallback only",
        "allowed": True},
    "OBJ_BAND": {
        "source": "debug seeds 51-65: above-table band that contains every "
                  "tabletop prop; the stove is the largest component in it "
                  "(~1470 cells vs ~560 for the plate)",
        "allowed": True},
    "GOAL_DX": {
        "source": "pack keyframes/demo0_t0154.png: the plate ends 0.08-0.11 m "
                  "in front of the stove slab's front edge (pixel "
                  "deprojection onto the table plane); debug-seed bias sweep "
                  "on seeds 51,55,59,63 scored front-edge+0.14 -> 4/4, "
                  "+0.08 -> 8/8, +0.05 -> 3/4, +0.02 -> 3/4, so 0.11 sits "
                  "inside the measured band",
        "allowed": True},
    "GOAL_DY": {
        "source": "debug-seed bias sweep on seeds 51,55,59,63 around the "
                  "stove's y centre-line: -0.045 -> 0/4, -0.03 -> 1/4, "
                  "0 -> 15/15, +0.06 -> 4/4. The tight edge is on the -y "
                  "side, and the plate always approaches from -y, so aim "
                  "0.04 past the centre-line to carry margin",
        "allowed": True},
    "CATCH_IN": {
        "source": "pack demo0: eef moved 0.28 m in y while the plate moved "
                  "0.22 m -> ~0.06 m of lost motion crossing the plate "
                  "interior; start the finger that far toward the far rim",
        "allowed": True},
    "PRESS_Z": {
        "source": "pack demo0 ee_path6: the push is run with a standing "
                  "downward command and the eef z stalls flat (0.9157-0.9191) "
                  "while the table measures 0.900 -- command below the table "
                  "and let contact set the height",
        "allowed": True},
    "HOVER": {
        "source": "debug seeds: tallest tabletop prop near the plate is the "
                  "bowl at table+0.052; 0.12 m clears it",
        "allowed": True},
    "TRAVEL_Z": {
        "source": "debug seed 51: eef starts at z=1.173; travelling in y at "
                  "the start height clears every prop",
        "allowed": True},
    "RETREAT_X": {
        "source": "debug seed 51: the arm starts at x=-0.208 and cam_high has "
                  "a clean view of the table from there; backing off in -x "
                  "after a push un-occludes the plate for re-perception",
        "allowed": True},
    "TOL": {
        "source": "generic: controller POS_TOL-scale placement tolerance",
        "allowed": True},
    "N_PUSH": {"source": "generic: episode step budget", "allowed": True},
}

RES = 0.005
XR = (-0.45, 0.45)
YR = (-0.45, 0.45)
PLATE_BAND = (0.010, 0.024)
PLATE_SPAN = (0.09, 0.20)
PLATE_BAND_WIDE = (0.006, 0.030)
PLATE_SPAN_WIDE = (0.07, 0.24)
OBJ_BAND = (0.008, 0.090)
GOAL_DX = 0.11
GOAL_DY = 0.04
CATCH_IN = 0.035
PRESS_Z = 0.005
HOVER = 0.12
TRAVEL_Z = 1.17
RETREAT_X = -0.21
TOL = 0.018
N_PUSH = 3


# ---------------------------------------------------------------- perception

def cloud(f):
    d = np.asarray(f.depth, dtype=float)
    K = np.asarray(f.intrinsics, dtype=float)
    T = np.asarray(f.t_base_cam, dtype=float)
    h, w = d.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    good = np.isfinite(d) & (d > 0.05) & (d < 5.0)
    z = np.where(good, d, np.nan)
    x = (u - K[0, 2]) / K[0, 0] * z
    y = (v - K[1, 2]) / K[1, 1] * z
    P = np.stack([x, y, z], -1) @ T[:3, :3].T + T[:3, 3]
    return P, good


def table_z(P, good):
    z = P[..., 2][good]
    z = z[(z > 0.6) & (z < 1.2)]
    hist, edges = np.histogram(z, bins=300, range=(0.6, 1.2))
    i = int(np.argmax(hist))
    return float((edges[i] + edges[i + 1]) / 2.0)


def hmap(P, good):
    x, y, z = P[..., 0][good], P[..., 1][good], P[..., 2][good]
    nx = int(round((XR[1] - XR[0]) / RES))
    ny = int(round((YR[1] - YR[0]) / RES))
    ix = ((x - XR[0]) / RES).astype(int)
    iy = ((y - YR[0]) / RES).astype(int)
    m = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    H = np.full((nx, ny), -1.0)
    np.maximum.at(H, (ix[m], iy[m]), z[m])
    return H


def comps(mask):
    seen = np.zeros(mask.shape, bool)
    out = []
    for i, j in zip(*np.nonzero(mask)):
        if seen[i, j]:
            continue
        seen[i, j] = True
        stack = [(i, j)]
        pix = []
        while stack:
            a, b = stack.pop()
            pix.append((a, b))
            for p, q in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                if 0 <= p < mask.shape[0] and 0 <= q < mask.shape[1] \
                        and mask[p, q] and not seen[p, q]:
                    seen[p, q] = True
                    stack.append((p, q))
        out.append(np.array(pix))
    return out


def bbox(c):
    xs = XR[0] + (c[:, 0] + 0.5) * RES
    ys = YR[0] + (c[:, 1] + 0.5) * RES
    return xs.min(), xs.max(), ys.min(), ys.max()


def _plate_in(H, tz, band, span, min_cells):
    m = (H > tz + band[0]) & (H < tz + band[1])
    best = None
    for c in comps(m):
        if len(c) < min_cells:
            continue
        x0, x1, y0, y1 = bbox(c)
        sx, sy = x1 - x0, y1 - y0
        if not (span[0] <= sx <= span[1] and span[0] <= sy <= span[1]):
            continue
        if best is None or len(c) > len(best[0]):
            best = (c, (x0 + x1) / 2.0, (y0 + y1) / 2.0, sx, sy)
    return best


def find_plate(H, tz):
    """Strict band first; a looser one only if the strict one sees nothing.

    The strict band picked the plate cleanly on all 15 debug seeds, so the
    fallback never runs there -- it exists so that a seed whose plate reads a
    few millimetres off still yields a target instead of a silent no-op.
    """
    best = _plate_in(H, tz, PLATE_BAND, PLATE_SPAN, 60)
    if best is None:
        best = _plate_in(H, tz, PLATE_BAND_WIDE, PLATE_SPAN_WIDE, 30)
    return best


def find_stove(H, tz):
    m = (H > tz + OBJ_BAND[0]) & (H < tz + OBJ_BAND[1])
    best = None
    for c in comps(m):
        if best is None or len(c) > len(best[0]):
            best = (c, ) + bbox(c)
    return best


# ---------------------------------------------------------------------- run

def run(api):
    R = np.asarray(api.tool_rotation(), dtype=float)
    api.log("start eef=%s" % np.asarray(api.eef()).round(4).tolist())

    f = api.capture("cam_high")
    P, good = cloud(f)
    tz = table_z(P, good)
    H = hmap(P, good)
    st = find_stove(H, tz)
    pl = find_plate(H, tz)
    if st is None or pl is None:
        api.log("PERCEPTION FAILED stove=%s plate=%s" % (st is None, pl is None))
        return
    _, sx0, sx1, sy0, sy1 = st
    _, px, py, spanx, spany = pl
    gx = sx1 + GOAL_DX
    gy = (sy0 + sy1) / 2.0 + GOAL_DY
    api.log("table_z=%.4f" % tz)
    api.log("stove x[%.3f,%.3f] y[%.3f,%.3f]" % (sx0, sx1, sy0, sy1))
    api.log("plate0=(%.3f,%.3f) span=(%.3f,%.3f)" % (px, py, spanx, spany))
    api.log("goal=(%.3f,%.3f)" % (gx, gy))

    z_hi = tz + HOVER
    z_press = tz + PRESS_Z

    for k in range(N_PUSH):
        d = np.array([gx - px, gy - py])
        dist = float(np.linalg.norm(d))
        api.log("push %d: plate=(%.3f,%.3f) err=%.4f" % (k, px, py, dist))
        if dist < TOL:
            break
        u = d / dist
        catch = min(CATCH_IN, 0.4 * min(spanx, spany))
        a = np.array([px, py]) + u * catch
        b = np.array([gx, gy]) + u * catch

        # travel: sideways in y at the start height (the bottle at y=-0.05 is
        # the only tall prop and this keeps clear of it), then over the catch
        # point, then straight down onto the plate.
        e = np.asarray(api.eef(), dtype=float)
        api.move([e[0], a[1], max(e[2], TRAVEL_Z)], rotation=R, seconds=0.6)
        api.move([a[0], a[1], z_hi], rotation=R, seconds=0.6)
        api.move([a[0], a[1], z_press], rotation=R, seconds=0.7)
        api.log("  pressed eef=%s" % np.asarray(api.eef()).round(4).tolist())
        api.move([b[0], b[1], z_press], rotation=R, seconds=1.0)
        api.log("  dragged eef=%s" % np.asarray(api.eef()).round(4).tolist())
        e = np.asarray(api.eef(), dtype=float)
        api.move([e[0], e[1], TRAVEL_Z], rotation=R, seconds=0.6)
        api.move([RETREAT_X, e[1], TRAVEL_Z], rotation=R, seconds=0.6)

        f = api.capture("cam_high")
        P, good = cloud(f)
        H = hmap(P, good)
        pl = find_plate(H, tz)
        if pl is None:
            api.log("  lost the plate after push %d" % k)
            break
        _, px, py, spanx, spany = pl
        api.log("  plate now=(%.3f,%.3f) span=(%.3f,%.3f)" % (px, py, spanx, spany))

    api.log("final plate=(%.3f,%.3f) goal=(%.3f,%.3f)" % (px, py, gx, gy))
