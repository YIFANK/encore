"""c2clean spa_bowl_on_wooden_cabinet_task_k3 -- v2 (park the arm before perceiving).

Intent: "Pick the akita black bowl on the stove and place it on the plate".

Scene reading (all from my own debug-seed cam_high RGB-D, seeds 51/53/55):
three bowls sit at three distinct SUPPORT heights -- one on the table
(support ~0.901), one on a slab whose top is ~0.927 (the stove), one on a tall
box whose top is ~1.128 (the wooden cabinet).  The named bowl is the one whose
support is the middle height.  The plate is the flat near-circular dish whose
rim tops out ~0.921 with a sunken interior ~0.908.

Grasp: the stove bowl measures 0.108 m across at the rim (top z 0.980) with an
interior that falls to ~0.935; at the grasp depth the wall spans radius
0.036..0.054, so the jaw midline wants to sit at radius ~0.045 from the bowl
centre, along the jaw axis (world +y for the straight-down wrist).  Grasp
height comes from the mate pack's three grasp keyframes (eef z 0.944/0.946/
0.948, i.e. 0.034 below the measured rim top).
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51/53/55 cam_high height histogram: "
                          "124k of 184k workspace points at z 0.900-0.905",
                "allowed": True},
    "BOWL_BAND": {"source": "debug-seed height maps: bowl rims reach 0.944 "
                            "(table bowl) / 0.980 (stove bowl); table and "
                            "stove slab tops are 0.901 / 0.927",
                  "allowed": True},
    "SUPPORT_CUTS": {"source": "debug-seed height maps: the three bowl "
                               "supports measure 0.901 (table), 0.927 (stove "
                               "slab), 1.128 (cabinet top)",
                     "allowed": True},
    "PLATE_BAND": {"source": "debug-seed height map of the plate: rim 0.921, "
                             "interior floor 0.908; extent 0.144 x 0.152 m",
                   "allowed": True},
    "WALL_HALF": {"source": "debug-seed height map cross-section of the stove "
                            "bowl: outer rim radius 0.054, inner surface at "
                            "grasp depth 0.036 -> midline is outer - 0.009",
                  "allowed": True},
    "GRASP_DROP": {"source": "mate pack ee_path6 grasp keyframes (eef z "
                             "0.9482/0.9464/0.9482) vs my measured rim top "
                             "0.980 -> 0.034 below the rim",
                   "allowed": True},
    "PLACE_Z": {"source": "mate + k3 pack ee_path6 release keyframes: eef z "
                          "0.9306/0.9387/0.9290 and 0.9879/0.9541/0.9802",
                "allowed": True},
    "CARRY_Z": {"source": "pack ee_path6 carry segments peak at z 1.08-1.14; "
                          "my debug-seed map puts the cabinet top at 1.128 and "
                          "every other obstacle below 0.99",
                "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics: api.tool_rotation() at "
                         "reset is diag(1,-1,-1) to 3 deg; the fair harness "
                         "keeps the wrist straight down for move(rotation=None)",
               "allowed": True},
    "PARK": {"source": "debug seed 51 v1 log: at reset the arm sits over the "
                       "stove and fuses with the bowl in the height map; park "
                       "it at +y where cam_high sees the left half clear",
             "allowed": True},
    "BOWL_CAP": {"source": "debug-seed height maps: every bowl rim is below "
                           "1.05; the cabinet top (1.128) and the arm (1.37) "
                           "are above it",
                 "allowed": True},
    "GRIP_OPEN": {"source": "debug-seed api.gripper() at reset: width_m 0.0778",
                  "allowed": True},
}

TABLE_Z = 0.901
CELL = 0.006
XLO, XHI, YLO, YHI = -0.42, 0.30, -0.46, 0.46
BOWL_BAND = 0.930          # cells above this are bowl walls, not supports
BOWL_CAP = 1.050           # ... and below this: above it is cabinet/arm
PARK = (0.06, 0.40, 1.20)  # arm out of cam_high's view of the left half
PLATE_LO, PLATE_HI = 0.912, 0.929
SUPPORT_TABLE_MAX = 0.915  # supports at/below this are the bare table
SUPPORT_HIGH_MIN = 1.050   # supports at/above this are the wooden cabinet
WALL_HALF = 0.009
GRASP_DROP = 0.034
PLACE_Z = 0.936
CARRY_Z = 1.060
GRIP_OPEN = 0.078
HOLD_GAP_MIN = 0.008       # closed gap below this means the jaws met air


def _cloud(f):
    d = np.asarray(f.depth, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    fx, fy = f.intrinsics[0, 0], f.intrinsics[1, 1]
    cx, cy = f.intrinsics[0, 2], f.intrinsics[1, 2]
    good = np.isfinite(d) & (d > 0)
    d = np.where(good, d, 0.0)
    p = np.stack([(uu - cx) * d / fx, (vv - cy) * d / fy, d, np.ones_like(d)], -1)
    b = p @ np.asarray(f.t_base_cam, float).T
    return b[..., 0], b[..., 1], b[..., 2], good


def _grid(f):
    X, Y, Z, good = _cloud(f)
    m = (good & (X > XLO) & (X < XHI) & (Y > YLO) & (Y < YHI)
         & np.isfinite(Z) & (Z > 0.5) & (Z < 1.6))
    ni = int((XHI - XLO) / CELL) + 1
    nj = int((YHI - YLO) / CELL) + 1
    g = np.full((ni, nj), np.nan)
    gi = ((X[m] - XLO) / CELL).astype(int)
    gj = ((Y[m] - YLO) / CELL).astype(int)
    for a, b, z in zip(gi, gj, Z[m]):
        if np.isnan(g[a, b]) or z > g[a, b]:
            g[a, b] = z
    return g


def _comps(mask):
    ni, nj = mask.shape
    seen = np.zeros_like(mask)
    out = []
    for i in range(ni):
        for j in range(nj):
            if mask[i, j] and not seen[i, j]:
                stack = [(i, j)]
                seen[i, j] = True
                cells = []
                while stack:
                    a, b = stack.pop()
                    cells.append((a, b))
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if (0 <= p < ni and 0 <= q < nj and mask[p, q]
                                    and not seen[p, q]):
                                seen[p, q] = True
                                stack.append((p, q))
                out.append(np.array(cells))
    return out


def _xy(i, j):
    return XLO + i * CELL, YLO + j * CELL


def _support(g, cells):
    """median height of the ring of cells just outside a component's bbox"""
    ii, jj = cells[:, 0], cells[:, 1]
    i0, i1, j0, j1 = ii.min(), ii.max(), jj.min(), jj.max()
    ni, nj = g.shape
    vals = []
    for r in (2, 3, 4):
        for i in range(i0 - r, i1 + r + 1):
            for j in (j0 - r, j1 + r):
                if 0 <= i < ni and 0 <= j < nj and np.isfinite(g[i, j]):
                    vals.append(g[i, j])
        for j in range(j0 - r, j1 + r + 1):
            for i in (i0 - r, i1 + r):
                if 0 <= i < ni and 0 <= j < nj and np.isfinite(g[i, j]):
                    vals.append(g[i, j])
    return float(np.median(vals)) if vals else TABLE_Z


def _bbox(cells):
    ii, jj = cells[:, 0], cells[:, 1]
    x0, y0 = _xy(ii.min(), jj.min())
    x1, y1 = _xy(ii.max(), jj.max())
    return x0, x1, y0, y1


def perceive(api):
    api.move(list(PARK), seconds=3.0)
    api.settle(0.4)
    api.log("parked eef=%s" % np.round(api.eef(), 4).tolist())
    f = api.capture("cam_high")
    g = _grid(f)
    hi = np.isfinite(g) & (g > BOWL_BAND) & (g < BOWL_CAP)
    bowls = []
    for cells in _comps(hi):
        if len(cells) < 30:
            continue
        x0, x1, y0, y1 = _bbox(cells)
        dx, dy = x1 - x0, y1 - y0
        top = float(np.nanmax([g[a, b] for a, b in cells]))
        sup = _support(g, cells)
        bowls.append(dict(cx=(x0 + x1) / 2, cy=(y0 + y1) / 2, dx=dx, dy=dy,
                          top=top, sup=sup, n=len(cells)))
        api.log("CAND bowl c=(%.3f,%.3f) d=(%.3f,%.3f) top=%.3f sup=%.3f n=%d"
                % (bowls[-1]["cx"], bowls[-1]["cy"], dx, dy, top, sup, len(cells)))
    target = None
    for b in bowls:
        if (SUPPORT_TABLE_MAX < b["sup"] < SUPPORT_HIGH_MIN
                and 0.07 < b["dx"] < 0.17 and 0.07 < b["dy"] < 0.17
                and b["top"] < SUPPORT_HIGH_MIN):
            if target is None or b["n"] > target["n"]:
                target = b
    if target is None:                       # fallback: highest support < cap
        cands = [b for b in bowls
                 if 0.07 < b["dx"] < 0.17 and 0.07 < b["dy"] < 0.17
                 and b["top"] < SUPPORT_HIGH_MIN]
        if cands:
            target = max(cands, key=lambda b: b["sup"])

    flat = np.isfinite(g) & (g > PLATE_LO) & (g < PLATE_HI)
    plate = None
    for cells in _comps(flat):
        if len(cells) < 40:
            continue
        x0, x1, y0, y1 = _bbox(cells)
        dx, dy = x1 - x0, y1 - y0
        c = dict(cx=(x0 + x1) / 2, cy=(y0 + y1) / 2, dx=dx, dy=dy, n=len(cells))
        api.log("CAND flat c=(%.3f,%.3f) d=(%.3f,%.3f) n=%d"
                % (c["cx"], c["cy"], dx, dy, len(cells)))
        if not (0.11 < dx < 0.19 and 0.11 < dy < 0.19 and abs(dx - dy) < 0.035):
            continue
        if target is not None and abs(c["cx"] - target["cx"]) < 0.09 \
                and abs(c["cy"] - target["cy"]) < 0.09:
            continue                      # that's the target's own support
        if plate is None or c["n"] > plate["n"]:
            plate = c
    return target, plate


def run(api):
    api.log("instruction=%r" % api.instruction())
    target, plate = perceive(api)
    api.log("TARGET=%s" % target)
    api.log("PLATE=%s" % plate)
    if target is None or plate is None:
        return "perception failed target=%s plate=%s" % (bool(target), bool(plate))

    r_out = max(target["dx"], target["dy"]) / 2.0
    off = r_out - WALL_HALF
    gz = target["top"] - GRASP_DROP
    api.log("r_out=%.3f off=%.3f grasp_z=%.3f" % (r_out, off, gz))

    api.grip(GRIP_OPEN)
    best = None
    for sgn in (+1.0, -1.0):
        gx, gy = target["cx"], target["cy"] + sgn * off
        api.log("TRY sgn=%+.0f grasp=(%.3f,%.3f,%.3f)" % (sgn, gx, gy, gz))
        api.move([gx, gy, target["top"] + 0.075], seconds=3.0)
        api.move([gx, gy, gz], seconds=2.5)
        api.grip(0.0)
        api.settle(0.6)
        gp = api.gripper()
        api.log("closed sgn=%+.0f gripper=%s eef=%s"
                % (sgn, gp, np.round(api.eef(), 4).tolist()))
        if gp["width_m"] > HOLD_GAP_MIN:
            best = (gx, gy, sgn)
            break
        api.grip(GRIP_OPEN)
        api.settle(0.3)
        api.move([gx, gy, target["top"] + 0.075], seconds=2.5)
    if best is None:
        return "no grasp: both jaw offsets closed on air"
    gx, gy, sgn = best

    api.move([gx, gy, CARRY_Z], seconds=3.0)
    gp = api.gripper()
    api.log("lifted gripper=%s eef=%s" % (gp, np.round(api.eef(), 4).tolist()))

    px, py = plate["cx"], plate["cy"] + sgn * off
    api.move([px, py, CARRY_Z], seconds=3.5)
    api.move([px, py, PLACE_Z], seconds=3.0)
    gp = api.gripper()
    api.log("at place gripper=%s eef=%s" % (gp, np.round(api.eef(), 4).tolist()))
    api.grip(GRIP_OPEN)
    api.settle(0.8)
    api.move([px, py, CARRY_Z], seconds=3.0)
    api.settle(0.5)

    f = api.capture("cam_high")
    g = _grid(f)
    hi = np.isfinite(g) & (g > BOWL_BAND) & (g < BOWL_CAP)
    for cells in _comps(hi):
        if len(cells) < 30:
            continue
        x0, x1, y0, y1 = _bbox(cells)
        api.log("POST comp c=(%.3f,%.3f) d=(%.3f,%.3f) top=%.3f n=%d"
                % ((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0,
                   float(np.nanmax([g[a, b] for a, b in cells])), len(cells)))
    return "placed at (%.3f,%.3f)" % (px, py)
