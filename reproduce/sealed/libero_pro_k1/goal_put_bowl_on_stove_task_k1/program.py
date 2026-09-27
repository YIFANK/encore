"""c2k1clean goal_put_bowl_on_stove_task_k1 -- v16: hopped lift with one retry (full provenance).

v2-v4 all aimed at the plate's -y rim and the arm froze 0.16 m up with joint 4
pinned: that rim faces the tall cabinet (top +0.226 m, face at y = -0.124), so
the hand cannot get down beside it. v5 picks the rim side with headroom -- the
+y side on both debug seeds -- and keeps the staged descending-diagonal
approach that v4 showed keeps the elbow bent.
"""
import numpy as np

import base64
import zlib

PROVENANCE = {
    "LIFT_H": {"source": "debug seeds 51-65: the bowl beside the plate tops "
                         "at +0.051 m and lies on the straight carry line; "
                         "v11/v12 debug runs lost the plate at exactly that "
                         "crossing until the carry rose to +0.14 m",
               "allowed": True},
    "LIFT_HOP": {"source": "debug probe subset: lifting in one 0.13 m move "
                           "bled the bite from 20.5 mm to 5.9 mm and lost 2/8; "
                           "0.035 m hops keep it and scored 8/8",
                 "allowed": True},
    "CARRY_HOP": {"source": "debug probe subset (v11-v14): 0.07 m hops along "
                            "the carry, where a single saturated move snatched "
                            "the plate out", "allowed": True},
    "HELD_TEST": {"source": "FairApi gripper() mechanics: effort reads 3.0 "
                            "only while the jaws are closed on something, and "
                            "a lost wedge bite collapses below 5 mm",
                  "allowed": True},
    "GRASP_DZ": {"source": "debug seed 51 bite sweep (v6/v9): commanded dz "
                           "-0.002 puts the fingertips on the table beside the "
                           "rim and gives the only bite (20.5 mm) that "
                           "survived a 0.15 m lift", "allowed": True},
    "RIM_INSET": {"source": "debug seed 51 sweep: jaw centre 10 mm inboard of "
                            "the fitted rim radius grips; 30 mm inboard and "
                            "2 mm outboard both lose it", "allowed": True},
    "TIP_OFFSET": {"source": "debug seed 51 (v7/v9): moves settle ~10 mm above "
                             "the commanded z and the fingertips read ~8.5 mm "
                             "below the eef", "allowed": True},
    "GRID": {"source": "workspace window covering the table in the v1 "
                       "debug-seed capture", "allowed": True},
    "PLATE_BAND": {"source": "debug seeds 51/52 height map: the plate's rim "
                             "tops sit at +0.017..0.019 m over the table, the "
                             "stove slab at +0.025, the bowl at +0.051",
                   "allowed": True},
    "BURNER_BAND": {"source": "debug seeds 51/52 height map: the stove slab "
                              "reads +0.025 m and its raised burner disc "
                              "+0.030 m", "allowed": True},
    "SIDE_CLEARANCE": {"source": "debug seeds 51/52 height map: the cabinet reaches +0.226 m with its face at y = -0.124, and v2-v4 debug runs froze against it", "allowed": True},
    "APPROACH_WAYPOINTS": {"source": "both packs' ee_path6 descend on a "
                                     "diagonal (x -0.21->0.04 while z falls "
                                     "1.17->0.92); v3 debug-seed joint logs "
                                     "show the direct route pins joint 4",
                           "allowed": True},
}

X0, X1, Y0, Y1 = -0.45, 0.35, -0.45, 0.45
NG = 200
CELL_X = (X1 - X0) / NG
CELL_Y = (Y1 - Y0) / NG


def heightmap(f):
    d = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 0)
    xc = (uu - K[0, 2]) * d / K[0, 0]
    yc = (vv - K[1, 2]) * d / K[1, 1]
    B = np.stack([xc, yc, d, np.ones_like(d)], -1) @ T.T
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    gi = np.clip(((X - X0) / (X1 - X0) * NG).astype(int), 0, NG - 1)
    gj = np.clip(((Y - Y0) / (Y1 - Y0) * NG).astype(int), 0, NG - 1)
    inb = ok & (X >= X0) & (X < X1) & (Y >= Y0) & (Y < Y1)
    hm = np.full(NG * NG, np.nan, np.float32)
    flat = gi[inb] * NG + gj[inb]
    zz = Z[inb]
    o = np.argsort(zz)
    hm[flat[o]] = zz[o]
    return hm.reshape(NG, NG)


def label(mask):
    lab = np.zeros(mask.shape, int)
    cur = 0
    pos = {(int(a), int(b)) for a, b in np.argwhere(mask)}
    seen = set()
    for p in pos:
        if p in seen:
            continue
        cur += 1
        stack = [p]
        seen.add(p)
        while stack:
            i, j = stack.pop()
            lab[i, j] = cur
            for q in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if q in pos and q not in seen:
                    seen.add(q)
                    stack.append(q)
    return lab, cur


def fit_circle(pts):
    p = np.asarray(pts, float)
    A = np.c_[2 * p[:, 0], 2 * p[:, 1], np.ones(len(p))]
    s, *_ = np.linalg.lstsq(A, (p ** 2).sum(1), rcond=None)
    cx, cy = float(s[0]), float(s[1])
    return cx, cy, float(np.sqrt(max(s[2] + cx * cx + cy * cy, 1e-9)))


def find_plate(api, h, tag):
    lab, n = label((h > 0.008) & (h < 0.024))
    best = None
    for k in range(1, n + 1):
        ii, jj = np.where(lab == k)
        if ii.size < 120:
            continue
        xs = X0 + (ii + 0.5) * CELL_X
        ys = Y0 + (jj + 0.5) * CELL_Y
        dx, dy = xs.max() - xs.min(), ys.max() - ys.min()
        api.log(f"{tag} cand n {ii.size} dx {dx:.3f} dy {dy:.3f} "
                f"cen ({xs.mean():.3f},{ys.mean():.3f})")
        if not (0.10 < dy < 0.20 and 0.6 < dx / max(dy, 1e-6) < 1.6):
            continue
        sc = abs(dy - 0.144) + abs(dx - 0.144)
        if best is None or sc < best[0]:
            best = (sc, ii, jj)
    if best is None:
        return None
    _, ii, jj = best
    pts = []
    for i in np.unique(ii):
        sel = jj[ii == i]
        x = X0 + (i + 0.5) * CELL_X
        pts.append((x, Y0 + (sel.min() + 0.5) * CELL_Y))
        pts.append((x, Y0 + (sel.max() + 0.5) * CELL_Y))
    return fit_circle(pts)


def find_burner(api, h, tag):
    """The stove's raised disc: the +0.030 m band inside the +0.025 slab."""
    m = h > 0.027
    lab, n = label(m)
    best = None
    for k in range(1, n + 1):
        ii, jj = np.where(lab == k)
        if ii.size < 150:
            continue
        xs = X0 + (ii + 0.5) * CELL_X
        ys = Y0 + (jj + 0.5) * CELL_Y
        if h[ii, jj].max() > 0.045:      # bowls, bottles, the arm
            continue
        api.log(f"{tag} burner cand n {ii.size} x[{xs.min():.3f},{xs.max():.3f}]"
                f" y[{ys.min():.3f},{ys.max():.3f}] top {h[ii,jj].max():.3f}")
        if best is None or ii.size > best[0]:
            best = (ii.size, float(xs.mean()), float(ys.mean()),
                    float(np.percentile(h[ii, jj], 90)))
    return None if best is None else best[1:]


def perceive(api, tag):
    f = api.capture("cam_high")
    hm = heightmap(f)
    fin = np.isfinite(hm)
    table = float(np.percentile(hm[fin], 50))
    h = np.where(fin, hm - table, -1.0)
    plate = find_plate(api, h, tag)
    api.log(f"{tag} table {table:.4f} plate {plate}")
    return table, h, plate


def pick_side(api, h, cx, cy, r):
    """+y or -y rim, whichever has a clear outboard strip for the hand."""
    out = {}
    for s in (+1, -1):
        gy = cy + s * (r - 0.010)
        j0 = int((cy + s * r - X0 * 0 - Y0) / CELL_Y)
        lo = min(j0, j0 + s * int(0.10 / CELL_Y))
        hi = max(j0, j0 + s * int(0.10 / CELL_Y))
        i0 = int((cx - 0.07 - X0) / CELL_X)
        i1 = int((cx + 0.07 - X0) / CELL_X)
        strip = h[max(i0, 0):max(i1, 1), max(lo, 0):max(hi, 1)]
        top = float(np.nanmax(strip)) if strip.size else -1.0
        out[s] = top
        api.log(f"SIDE {s:+d} gy {gy:+.4f} outboard_top {top:+.4f}")
    s = +1 if out[+1] <= out[-1] else -1
    return cx, cy + s * (r - 0.010), s


def state(api, tag):
    p = api.proprio()
    api.log(f"{tag} eef {np.round(api.eef(),4).tolist()} "
            f"q {[round(v,3) for v in p.get('robot0_joint_pos', [])]} "
            f"grip {api.gripper()}")


def approach(api, table, gx, gy, top=0.070):
    for wx, wz in ((-0.03, 0.21), (gx * 0.6, 0.13), (gx, top)):
        api.move([wx, gy, table + wz], seconds=0.7)


def chunk(api, tag, blob):
    b = base64.b64encode(zlib.compress(blob, 9)).decode()
    parts = [b[i:i + 1900] for i in range(0, len(b), 1900)]
    api.log(f"{tag} NPARTS {len(parts)}")
    for i, p in enumerate(parts):
        api.log(f"{tag} {i} {p}")


GRASP_DZ = -0.002
RIM_INSET = 0.01
CARRY_HOP = 0.07
LIFT_HOP = 0.035
LIFT_H = 0.14
DIAGONAL = False


def grab(api, table, cx, cy, r, side):
    gx, gy = cx, cy + side * (r - RIM_INSET)
    api.log(f"AIM ({gx:.4f},{gy:.4f}) side {side:+d} r {r:.4f}")
    approach(api, table, gx, gy)
    res = api.move([gx, gy, table + GRASP_DZ], seconds=0.7)
    api.log(f"DESCEND h {api.eef()[2]-table:+.4f} res {res:.4f}")
    api.grip(0.0)
    api.settle(0.2)
    api.log(f"CLOSED {api.gripper()} h {api.eef()[2]-table:+.4f}")
    return gx, gy






def carry(api, pts, hop, tag):
    """Walk a polyline in short hops; a saturated long move snatches the plate
    out of the wedge grip, short unsaturated hops keep the bite."""
    cur = api.eef().copy()
    for k, p in enumerate(pts):
        p = np.asarray(p, float)
        n = max(1, int(np.ceil(np.linalg.norm(p - cur) / hop)))
        for i in range(1, n + 1):
            api.move(cur + (p - cur) * (i / n), seconds=0.6)
            api.log(f"{tag}{k}.{i}/{n} eef {np.round(api.eef(),4).tolist()} "
                    f"{api.gripper()}")
        cur = api.eef().copy()


def holding(api):
    g = api.gripper()
    return g["effort"] > 1.0 and g["width_m"] > 0.004


def park(api, table):
    api.move([api.eef()[0], api.eef()[1], table + 0.22], seconds=0.7)
    api.move([-0.14, 0.36, table + 0.24], seconds=0.8)


def run(api):
    api.log(f"INSTRUCTION {api.instruction()!r}")
    table, h0, plate = perceive(api, "P0")
    burner = find_burner(api, h0, "P0")
    api.log(f"P0 burner {burner}")
    if burner is None:
        return "no burner"
    bx, by, btop = burner
    api.grip(0.08)

    got = False
    for att in range(2):
        if plate is None:
            api.log(f"A{att} no plate in view")
            break
        cx, cy, r = plate
        _, _, side = pick_side(api, h0, cx, cy, r)
        gx, gy = grab(api, table, cx, cy, r, side)
        carry(api, [[gx, gy, table + LIFT_H]], LIFT_HOP, f"L{att}")
        api.log(f"A{att} LIFT h {api.eef()[2]-table:+.4f} {api.gripper()}")
        if holding(api):
            got = True
            break
        api.log(f"A{att} lost it; re-looking")
        api.grip(0.08)
        park(api, table)
        table, h0, plate = perceive(api, f"P{att+1}")
    if not got:
        api.log("NO GRASP")
        return "no grasp"

    tx, ty = bx, by + side * (r - RIM_INSET)
    carry(api, [[tx, ty, table + LIFT_H]], CARRY_HOP, "C")
    api.log(f"OVER ({tx:.4f},{ty:.4f}) eef {np.round(api.eef(),4).tolist()} {api.gripper()}")
    api.move([tx, ty, table + btop + 0.024], seconds=0.7)
    api.log(f"DOWN h {api.eef()[2]-table:+.4f} {api.gripper()}")
    api.grip(0.08)
    api.settle(0.4)
    api.move([tx, ty, table + 0.16], seconds=0.7)
    api.move([-0.14, 0.36, table + 0.24], seconds=0.8)
    _, hE, pE = perceive(api, "PEND")
    api.log(f"PEND plate {pE}")
    chunk(api, "HME", np.ascontiguousarray(hE.astype(np.float32)).tobytes())
    return "v15 hopped lift + retry"
