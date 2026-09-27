"""v5 -- v4's rim pinch plus closed-loop verification.

Two sensors close the loop: the finger gap after a close (a bite on the bowl
wall settles near 0.0045 m, an empty close collapses below 0.002 m) and a
re-perception of the scene after the release (the bowl's rim ring has to show
up over the plate). Either check failing restarts the cycle from a fresh
perception rather than ending the episode on a silent miss.
"""
import numpy as np

PROVENANCE = {
    "X0/X1/Y0/Y1": {"source": "debug seeds 51-65 cam_high clouds: room walls "
                              "deproject to x<-1.9; this box keeps the table",
                    "allowed": True},
    "RES": {"source": "generic: 5 mm top-down raster, well under the 0.11 m bowl",
            "allowed": True},
    "RIM_BAND": {"source": "debug seeds 51-65: target bowl rim top sits 0.0775 m "
                           "above the table plane mode, its support 0.024 m above it",
                 "allowed": True},
    "BOWL_SPAN": {"source": "debug seeds 51-65: target bowl footprint 0.105-0.11 m; "
                            "the other table bowl is 0.087 m", "allowed": True},
    "PLATE_BAND": {"source": "debug seeds 51-65: plate top is 0.0175 m above the "
                             "table plane", "allowed": True},
    "PLATE_SPAN": {"source": "debug seeds 51-65: plate footprint 0.135 m across",
                   "allowed": True},
    "FINGER_DZ": {"source": "v3 debug probe: an open-jaw descent onto the bare "
                            "table stalled with api.eef() z 0.0062 m above the "
                            "table plane", "allowed": True},
    "GRIP_DEPTH": {"source": "geometry from debug clouds: the wall is 0.053 m "
                             "tall, so 0.015 m below the rim is clear of both the "
                             "rim edge and the bowl floor", "allowed": True},
    "BOWL_H": {"source": "debug seeds 51-65: rim top 0.9796 minus the support "
                         "surface 0.9262 measured in the annulus around the bowl",
               "allowed": True},
    "HOLD_GAP": {"source": "v4 debug runs: after the lift the finger gap read "
                           "0.0042-0.0048 on the three episodes that carried the "
                           "bowl and 0.0010-0.0017 on the one that dropped it",
                 "allowed": True},
    "PLACE_CLEAR": {"source": "generic: release with the payload just above the "
                              "receptacle", "allowed": True},
    "Z_HOVER/Z_CARRY": {"source": "generic: clear of every measured prop top",
                        "allowed": True},
}

X0, X1, Y0, Y1, RES = -0.55, 0.35, -0.60, 0.60, 0.005
FINGER_DZ = 0.006
GRIP_DEPTH = 0.015
BOWL_H = 0.053
HOLD_GAP = 0.003
PLACE_CLEAR = 0.004
Z_HOVER = 1.06
Z_CARRY = 1.05


# --------------------------------------------------------------- perception
def cloud(f):
    d = np.asarray(f.depth, dtype=float)
    K = np.asarray(f.intrinsics, dtype=float)
    T = np.asarray(f.t_base_cam, dtype=float)
    h, w = d.shape
    v, u = np.mgrid[0:h, 0:w]
    pc = np.stack([(u - K[0, 2]) * d / K[0, 0],
                   (v - K[1, 2]) * d / K[1, 1], d], -1)
    return pc @ T[:3, :3].T + T[:3, 3]


def height_map(P, zmax):
    nx = int(round((X1 - X0) / RES))
    ny = int(round((Y1 - Y0) / RES))
    H = np.full((nx, ny), -9.0)
    ix = ((P[..., 0] - X0) / RES).astype(int)
    iy = ((P[..., 1] - Y0) / RES).astype(int)
    ok = ((ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
          & np.isfinite(P[..., 2]) & (P[..., 2] < zmax))
    np.maximum.at(H, (ix[ok], iy[ok]), P[..., 2][ok])
    return H


def dilate(m, k):
    o = m.copy()
    for di in range(-k, k + 1):
        for dj in range(-k, k + 1):
            o |= np.roll(np.roll(m, di, 0), dj, 1)
    return o


def close_box(m, k):
    return ~dilate(~dilate(m, k), k)


def label4(mask):
    lab = np.zeros(mask.shape, dtype=np.int32)
    cur = 0
    for a, b in np.argwhere(mask):
        if lab[a, b]:
            continue
        cur += 1
        lab[a, b] = cur
        stack = [(a, b)]
        while stack:
            i, j = stack.pop()
            for p, q in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if 0 <= p < mask.shape[0] and 0 <= q < mask.shape[1] \
                        and mask[p, q] and not lab[p, q]:
                    lab[p, q] = cur
                    stack.append((p, q))
    return lab, cur


def kasa(x, y):
    A = np.c_[x, y, np.ones(len(x))]
    c = np.linalg.lstsq(A, x * x + y * y, rcond=None)[0]
    cx, cy = c[0] / 2.0, c[1] / 2.0
    return float(cx), float(cy), float(np.sqrt(max(c[2] + cx * cx + cy * cy, 1e-9)))


def rings(H, lo, hi, k, span_lo, span_hi, minn):
    raw = (H > lo) & (H < hi)
    lab, n = label4(close_box(raw, k))
    out = []
    for i in range(1, n + 1):
        sel = lab == i
        if sel.sum() < minn:
            continue
        a, b = np.nonzero(sel)
        sx = (a.max() - a.min()) * RES
        sy = (b.max() - b.min()) * RES
        if not (span_lo <= sx <= span_hi and span_lo <= sy <= span_hi):
            continue
        ring = sel & raw
        ra, rb = np.nonzero(ring)
        cx, cy, r = kasa(X0 + ra * RES, Y0 + rb * RES)
        out.append((int(sel.sum()), cx, cy, r, float(H[ring].max())))
    out.sort(key=lambda t: -t[0])
    return out


def slab(H, lo, hi, k, span_lo, span_hi, minn):
    raw = (H > lo) & (H < hi)
    lab, n = label4(close_box(raw, k))
    best = None
    for i in range(1, n + 1):
        sel = lab == i
        if sel.sum() < minn:
            continue
        a, b = np.nonzero(sel)
        sx = (a.max() - a.min()) * RES
        sy = (b.max() - b.min()) * RES
        if not (span_lo <= sx <= span_hi and span_lo <= sy <= span_hi):
            continue
        if best is None or sel.sum() > best[0]:
            best = (int(sel.sum()), float(X0 + 0.5 * (a.min() + a.max()) * RES),
                    float(Y0 + 0.5 * (b.min() + b.max()) * RES),
                    float(H[sel & raw].max()))
    return best


def look(api):
    f = api.capture("cam_high")
    P = cloud(f)
    z = P[..., 2]
    ws = (np.isfinite(z) & (P[..., 0] > X0) & (P[..., 0] < X1)
          & (np.abs(P[..., 1]) < 0.6))
    hist, edges = np.histogram(z[ws], bins=np.arange(0.80, 1.00, 0.005))
    tz = float(edges[hist.argmax()] + 0.0025)
    return tz, height_map(P, tz + 0.15), P


def find_bowls(api, tz, H, tag):
    a = rings(H, tz + 0.05, tz + 0.11, 4, 0.092, 0.14, 40)
    b = rings(H, tz + 0.042, tz + 0.10, 4, 0.092, 0.14, 40)
    seen = []
    for cand in a + b:
        if all(np.hypot(cand[1] - s[1], cand[2] - s[2]) > 0.04 for s in seen):
            seen.append(cand)
    for c in seen:
        api.log("%s ring n=%d c=(%.4f,%.4f) r=%.4f top=%.4f"
                % (tag, c[0], c[1], c[2], c[3], c[4]))
    return seen


# ------------------------------------------------------------------ motion
def goto(api, target, seconds=1.0, tol=0.006, tries=3, tag=""):
    target = np.asarray(target, dtype=float)
    cmd = target.copy()
    e = np.asarray(api.eef(), dtype=float)
    for i in range(tries):
        api.move(cmd.tolist(), seconds=seconds)
        e = np.asarray(api.eef(), dtype=float)
        err = target - e
        api.log("GOTO%s try%d cmd=%s eef=%s err=%.4f"
                % (tag, i, cmd.round(4).tolist(), e.round(4).tolist(),
                   float(np.linalg.norm(err))))
        if float(np.linalg.norm(err)) < tol:
            break
        cmd = cmd + err
        seconds = 0.5
    return e


def attempt(api, tz, bowl, plate, side):
    cx, cy, r, rim = bowl[1], bowl[2], bowl[3], bowl[4]
    px, py, ptop = plate[1], plate[2], plate[3]
    gx, gy = cx, cy + side * r
    z_grip = rim - GRIP_DEPTH + FINGER_DZ

    api.grip(0.08)
    goto(api, [gx, gy, Z_HOVER], seconds=1.2, tol=0.005, tries=3, tag=":hover")
    goto(api, [gx, gy, z_grip], seconds=1.0, tol=0.004, tries=3, tag=":down")
    api.grip(0.02)
    api.log("CLOSED %s" % (api.gripper(),))

    goto(api, [gx, gy, Z_CARRY], seconds=1.0, tol=0.010, tries=2, tag=":lift")
    g = api.gripper()
    e = np.asarray(api.eef(), dtype=float)
    api.log("LIFTED %s eef=%s" % (g, e.round(4).tolist()))
    if float(g["width_m"]) < HOLD_GAP:
        api.log("HOLD CHECK FAILED (gap %.4f)" % g["width_m"])
        api.grip(0.08)
        return False

    hang = (rim - GRIP_DEPTH) - (rim - BOWL_H)          # payload under the tips
    hang += FINGER_DZ
    f2 = api.capture("cam_high")
    P2 = cloud(f2)
    near = (np.isfinite(P2[..., 2])
            & (np.hypot(P2[..., 0] - e[0], P2[..., 1] - e[1]) < 0.075)
            & (P2[..., 2] > e[2] - 0.13) & (P2[..., 2] < e[2] - 0.004))
    if near.sum() > 40:
        rim_h = float(np.percentile(P2[..., 2][near], 99))
        h2 = e[2] - (rim_h - BOWL_H)
        api.log("PAYLOAD rimtop=%.4f hang=%.4f (geom %.4f) n=%d"
                % (rim_h, h2, hang, int(near.sum())))
        if 0.02 < h2 < 0.09:
            hang = h2

    goto(api, [px, py + side * r, Z_CARRY], seconds=1.4, tol=0.010, tries=2,
         tag=":over")
    z_rel = ptop + PLACE_CLEAR + hang
    goto(api, [px, py + side * r, z_rel], seconds=1.0, tol=0.008, tries=2,
         tag=":place")
    api.log("PRE-RELEASE %s" % (api.gripper(),))
    api.grip(0.08)
    api.settle(0.3)
    goto(api, [px, py + side * r, Z_CARRY], seconds=0.8, tol=0.02, tries=1,
         tag=":up")
    return True


def run(api):
    tz, H, P = look(api)
    api.log("TABLE z=%.4f" % tz)
    plate = slab(H, tz + 0.008, tz + 0.026, 3, 0.115, 0.155, 60)
    if plate is None:
        api.log("ABORT: no plate")
        return
    api.log("PLATE n=%d c=(%.4f,%.4f) top=%.4f" % plate)

    side = 1.0
    for cycle in range(3):
        tz, H, P = look(api)
        found = find_bowls(api, tz, H, "CYCLE%d" % cycle)
        on_plate = [c for c in found
                    if np.hypot(c[1] - plate[1], c[2] - plate[2]) < 0.045]
        if on_plate:
            api.log("VERIFIED bowl over plate at cycle %d" % cycle)
            return
        rest = [c for c in found
                if np.hypot(c[1] - plate[1], c[2] - plate[2]) >= 0.045]
        if not rest:
            api.log("ABORT: no bowl candidate at cycle %d" % cycle)
            return
        bowl = rest[0]
        api.log("CYCLE%d target c=(%.4f,%.4f) r=%.4f top=%.4f side=%+.0f"
                % (cycle, bowl[1], bowl[2], bowl[3], bowl[4], side))
        if not attempt(api, tz, bowl, plate, side):
            side = -side
    tz, H, P = look(api)
    find_bowls(api, tz, H, "FINAL")
    api.log("DONE v5")
