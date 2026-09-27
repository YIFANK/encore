"""v6 -- perceive, yaw the wrist into the drawer's free slot, rim-pinch the
bowl, carry it to the plate and set it down.

The drawer is barely wider than the bowl, so an open Panda hand (78 mm across)
only fits beside the rim over two narrow arcs. The arc is found at run time
from the height map and the wrist is yawed so the jaws close along it.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "GRID_CELL": {
        "source": "generic: 5 mm top-down height map built from api.capture "
                  "depth deprojection",
        "allowed": True},
    "RIM_BAND": {
        "source": "debug seeds 51-65 cam_high height maps: the drawer bowl's "
                  "wall occupies slab-0.040..slab-0.006 (slab = modal height "
                  "of the cabinet top) on every seed; the only other component "
                  "there is the drawer handle, separated by aspect ratio",
        "allowed": True},
    "BAND_AR_MAX": {
        "source": "debug seeds 51-65: bowl component aspect ratio 1.00, "
                  "drawer handle 1.33-1.42",
        "allowed": True},
    "PLATE_MIN_CELLS": {
        "source": "debug seeds 51-65: plate footprint 552-626 cells vs <=235 "
                  "for the cookie box and the table bowl",
        "allowed": True},
    "TIP_OFFSET": {
        "source": "v3 debug-seed measurement: gripper parked over bare table, "
                  "the fingertips read 0.008 m below the reported EEF z in the "
                  "depth cloud",
        "allowed": True},
    "JAW_HALF_OPEN": {
        "source": "debug-seed api.gripper(): open width 0.0778 m, so each "
                  "finger sits 0.039 m from the tool axis",
        "allowed": True},
    "FINGER_PATCH": {
        "source": "v3 optical scan: the open finger occupies ~0.015 m across; "
                  "clearance is tested over a 0.008 m radius patch",
        "allowed": True},
    "GRASP_DEPTH": {
        "source": "pack.json demo grasp EEF z 1.092/1.097/1.098 against the "
                  "1.116 rim measured on the debug seeds -> 0.022 below the rim",
        "allowed": True},
    "ARC_WINDOW": {
        "source": "debug seeds 51-65: the drawer's open corner is a free arc "
                  "starting at 108-114 deg and 32-48 deg wide in every seed; "
                  "the other free arc (near 0 deg) hangs off the drawer front",
        "allowed": True},
    "PROBE_RADIUS": {
        "source": "JAW_HALF_OPEN + half a finger width: the outer finger "
                  "centre rides 0.094 m from the bowl centre for a rim pinch",
        "allowed": True},
}

XMIN, XMAX, YMIN, YMAX, CELL = -0.62, 0.46, -0.60, 0.60, 0.005
NX = int(round((XMAX - XMIN) / CELL))
NY = int(round((YMAX - YMIN) / CELL))
TIP = 0.008          # fingertip sits this far below the reported EEF z
HALF_OPEN = 0.039    # finger offset from the tool axis, jaws open
DUMP_DEPTH = False


# ------------------------------------------------------------------ plumbing
def dump(api, tag, arr):
    if not DUMP_DEPTH:
        return
    b = base64.b64encode(zlib.compress(np.ascontiguousarray(arr).tobytes(), 6)).decode()
    api.log("DUMP %s shape=%s dtype=%s nchunk=%d" % (
        tag, list(arr.shape), arr.dtype.str, (len(b) + 1799) // 1800))
    for i in range(0, len(b), 1800):
        api.log("D %s %d %s" % (tag, i // 1800, b[i:i + 1800]))


def cloud(frame):
    d = np.asarray(frame.depth, dtype=np.float64)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    K = np.asarray(frame.intrinsics, float)
    x = (uu - K[0, 2]) / K[0, 0] * d
    y = (vv - K[1, 2]) / K[1, 1] * d
    T = np.asarray(frame.t_base_cam, float)
    return np.stack([x, y, d], -1) @ T[:3, :3].T + T[:3, 3]


def heightmap(P):
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    m = (x >= XMIN) & (x < XMAX) & (y >= YMIN) & (y < YMAX) & (z > 0.80) & (z < 1.30)
    ix = ((x[m] - XMIN) / CELL).astype(int)
    iy = ((y[m] - YMIN) / CELL).astype(int)
    H = np.zeros((NX, NY))
    np.maximum.at(H, (ix, iy), z[m])
    return H


def components(mask):
    lab = np.zeros(mask.shape, int)
    n = 0
    todo = set(map(tuple, np.argwhere(mask)))
    while todo:
        s = todo.pop()
        n += 1
        st = [s]
        while st:
            c = st.pop()
            lab[c] = n
            for d in ((1, 0), (-1, 0), (0, 1), (0, -1),
                      (1, 1), (1, -1), (-1, 1), (-1, -1)):
                q = (c[0] + d[0], c[1] + d[1])
                if q in todo:
                    todo.discard(q)
                    st.append(q)
    return lab, n


def bbox_mid(idx):
    xs = XMIN + (idx[:, 0] + 0.5) * CELL
    ys = YMIN + (idx[:, 1] + 0.5) * CELL
    return ((xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2,
            xs.max() - xs.min(), ys.max() - ys.min())


def mode_z(vals, lo, hi, step=0.002):
    c, e = np.histogram(vals, bins=max(1, int((hi - lo) / step)), range=(lo, hi))
    return float(e[int(c.argmax())] + step / 2)


def patch(H, x, y, rad):
    i = int((x - XMIN) / CELL)
    j = int((y - YMIN) / CELL)
    k = max(1, int(rad / CELL))
    sub = H[max(0, i - k):i + k + 1, max(0, j - k):j + k + 1]
    if sub.size == 0:
        return 9.0, 9.0
    if (sub == 0).mean() > 0.5:          # mostly unseen -> treat as blocking
        return 9.0, 9.0
    s = sub[sub > 0]
    return float(s.max()), float(s.min())


# ---------------------------------------------------------------- perception
def look(api, tag):
    f = api.capture("cam_high")
    dump(api, tag, np.clip(np.asarray(f.depth) * 10000.0, 0, 65535).astype(np.uint16))
    return heightmap(cloud(f))


def scene(api, H):
    table = mode_z(H[(H > 0.85) & (H < 0.95)], 0.85, 0.95)
    slab = mode_z(H[H > table + 0.15], table + 0.15, table + 0.35)

    band = (H > slab - 0.040) & (H < slab - 0.006)
    lab, n = components(band)
    best, bestn = None, 0
    for k in range(1, n + 1):
        idx = np.argwhere(lab == k)
        if len(idx) < 60:
            continue
        mx, my, wx, wy = bbox_mid(idx)
        ar = max(wx, wy) / max(1e-6, min(wx, wy))
        api.log("BAND k=%d n=%d mid=%.3f,%.3f w=%.3f,%.3f ar=%.2f"
                % (k, len(idx), mx, my, wx, wy, ar))
        if ar < 1.30 and len(idx) > bestn:
            best, bestn = k, len(idx)
    if best is None:
        return None
    idx = np.argwhere(lab == best)
    bx, by, wx, wy = bbox_mid(idx)
    rim = float(H[lab == best].max())
    rout = (wx + wy) / 4.0 + 0.0025

    low = (H > table + 0.006) & (H < table + 0.08)
    xs = XMIN + (np.arange(NX) + 0.5) * CELL
    low[xs < -0.25, :] = False
    lab2, n2 = components(low)
    plate, pn = None, 0
    for k in range(1, n2 + 1):
        idx2 = np.argwhere(lab2 == k)
        if len(idx2) < 120:
            continue
        mx, my, ux, uy = bbox_mid(idx2)
        ar = max(ux, uy) / max(1e-6, min(ux, uy))
        api.log("LOW k=%d n=%d mid=%.3f,%.3f w=%.3f,%.3f ar=%.2f"
                % (k, len(idx2), mx, my, ux, uy, ar))
        if ar < 1.45 and len(idx2) > pn:
            plate, pn = k, len(idx2)
    if plate is None:
        return None
    idx2 = np.argwhere(lab2 == plate)
    px, py, ux, uy = bbox_mid(idx2)
    i0 = int((px - XMIN) / CELL)
    j0 = int((py - YMIN) / CELL)
    inner = H[max(0, i0 - 4):i0 + 5, max(0, j0 - 4):j0 + 5]
    inner = inner[inner > table - 0.01]
    pmid = float(np.median(inner)) if inner.size else table
    api.log("SCENE table=%.3f slab=%.3f BOWL=%.3f,%.3f rim=%.3f rout=%.3f "
            "PLATE=%.3f,%.3f mid=%.3f n=%d"
            % (table, slab, bx, by, rim, rout, px, py, pmid, pn))
    return dict(table=table, slab=slab, bx=bx, by=by, rim=rim, rout=rout,
                px=px, py=py, pmid=pmid)


def free_arc(api, H, s):
    """Widest arc where the outer finger of an open hand clears the drawer."""
    rp = s["rout"] - 0.006                    # pinch centre on the wall midline
    ro = rp + HALF_OPEN + 0.004               # outer finger centre
    ri = abs(rp - HALF_OPEN)                  # inner finger centre
    step = 2
    degs = list(range(0, 360, step))
    ceil_ = []
    floor_ = []
    for d in degs:
        a = np.radians(d)
        mo, mn = patch(H, s["bx"] + ro * np.cos(a), s["by"] + ro * np.sin(a), 0.008)
        mi, _ = patch(H, s["bx"] + ri * np.cos(a), s["by"] + ri * np.sin(a), 0.008)
        ceil_.append(max(mo, mi))
        floor_.append(mn)
    ok = [c < s["rim"] - 0.020 for c in ceil_]
    runs = []
    N = len(degs)
    i = 0
    while i < N:
        if ok[i] and (i == 0 or not ok[i - 1]):
            j = 0
            while j < N and ok[(i + j) % N]:
                j += 1
            runs.append((i, j))
            i += max(1, j)
        else:
            i += 1
    if not runs:
        return None
    # prefer the drawer's open corner (arc centre in 90..180 deg); otherwise
    # take the widest arc
    scored = []
    for st, ln in runs:
        mid = (degs[st] + ln * step / 2.0) % 360
        pref = 1 if 90 <= mid <= 180 else 0
        scored.append((pref, ln, mid, st))
    scored.sort(reverse=True)
    pref, ln, mid, st = scored[0]
    # the surface the bowl stands on: the drawer floor just outside its wall,
    # sampled over the free arc (ignore the table seen through gaps)
    sup = []
    for k in range(ln):
        a = np.radians(degs[(st + k) % N])
        for rr in (s["rout"] + 0.010, s["rout"] + 0.020, s["rout"] + 0.030):
            i = int((s["bx"] + rr * np.cos(a) - XMIN) / CELL)
            j = int((s["by"] + rr * np.sin(a) - YMIN) / CELL)
            v = H[i, j]
            if s["rim"] - 0.100 < v < s["rim"] - 0.015:
                sup.append(v)
    support = float(np.median(sup)) if len(sup) >= 5 else s["rim"] - 0.052
    api.log("ARC runs=%s -> mid=%.0f width=%.0f support=%.3f rp=%.3f"
            % ([(degs[a], b * step) for a, b in runs], mid, ln * step, support, rp))
    return mid, rp, support, ceil_[(st + ln // 2) % N]


def yaw_to(R0, deg):
    """Rotate the straight-down wrist about base z so the jaws close along deg."""
    a = np.radians(((deg - 90.0) + 90.0) % 180.0 - 90.0)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ R0


def run(api):
    api.log("INSTR %s" % api.instruction())
    R0 = np.asarray(api.tool_rotation(), float)
    H = look(api, "t0_dep")
    s = scene(api, H)
    if s is None:
        return "perception failed"
    arc = free_arc(api, H, s)
    if arc is None:
        return "no free arc"
    th, rp, support, ceil_ = arc
    a = np.radians(th)
    ux, uy = float(np.cos(a)), float(np.sin(a))
    R = yaw_to(R0, th)

    zdeep = ceil_ + TIP + 0.006            # deepest bite the interior allows
    api.log("PLAN th=%.0f rp=%.3f zdeep=%.3f support=%.3f" % (th, rp, zdeep, support))

    def mv(xyz, sec, tag):
        r = api.move(xyz, rotation=R, seconds=sec)
        e = np.asarray(api.eef(), float)
        api.log("MV %s tgt=%.3f,%.3f,%.3f res=%.4f eef=%.4f,%.4f,%.4f"
                % (tag, xyz[0], xyz[1], xyz[2], r, e[0], e[1], e[2]))
        return r, e

    # --- grasp, verified by a short lift; retry deeper / rotated if it slips
    tries = [(0.0, 0.0), (0.006, 0.0), (0.0, 10.0), (0.006, -10.0)]
    held = False
    gx = gy = 0.0
    zg_used = zdeep
    R_use = R
    for k, (dz, dth) in enumerate(tries):
        thk = th + dth
        ak = np.radians(thk)
        uxk, uyk = float(np.cos(ak)), float(np.sin(ak))
        R_use = yaw_to(R0, thk)
        gx, gy = s["bx"] + rp * uxk, s["by"] + rp * uyk
        zg = zdeep + dz
        zg_used = zg
        R = R_use
        api.grip(0.08)
        mv([gx, gy, s["rim"] + 0.075], 2.5 if k == 0 else 1.2, "above%d" % k)
        r, e = mv([gx, gy, zg], 1.2, "descend%d" % k)
        if r > 0.012:
            r, e = mv([gx, gy, zg], 0.8, "descend%da" % k)
        api.grip(0.0)
        g = api.gripper()
        api.log("GRASP%d th=%.0f zg=%.3f z=%.4f gap=%.4f eff=%.1f"
                % (k, thk, zg, e[2], g["width_m"], g["effort"]))
        api.settle(0.2)
        r, e = mv([gx, gy, s["rim"] + 0.040], 1.0, "check%d" % k)
        g = api.gripper()
        api.log("CHECK%d gap=%.4f eff=%.1f z=%.4f" % (k, g["width_m"], g["effort"], e[2]))
        if g["effort"] >= 1.0:
            ux, uy = uxk, uyk
            held = True
            break
    if not held:
        api.log("NOHOLD after %d tries" % len(tries))

    zcarry = s["rim"] + 0.095
    r, e = mv([gx, gy, zcarry], 1.2, "lift")
    g = api.gripper()
    api.log("LIFTED gap=%.4f eff=%.1f" % (g["width_m"], g["effort"]))

    # the bowl centre trails the EEF by the full pinch radius
    tx, ty = s["px"] + rp * ux, s["py"] + rp * uy
    mv([tx, ty, zcarry], 2.2, "carry")
    g = api.gripper()
    api.log("CARRIED gap=%.4f eff=%.1f" % (g["width_m"], g["effort"]))
    zp = s["pmid"] + (zg_used - support) - 0.004
    r, e = mv([tx, ty, zp], 1.6, "place")
    g = api.gripper()
    api.log("PLACE z=%.4f res=%.4f gap=%.4f eff=%.1f" % (e[2], r, g["width_m"], g["effort"]))
    api.grip(0.08)
    api.settle(0.4)
    mv([tx, ty, zcarry], 1.0, "retreat")
    return "v6 th=%.0f held=%s" % (th, held)
