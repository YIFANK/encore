"""v4: perceive bottle + cabinet top, grasp, carry, place down on the top.

Everything is derived from debug-seed (51/53/55/57) RGB-D measurements taken with
the v1 perception probe and the v2 motion probe, plus generic camera/controller
mechanics.  See PROVENANCE.
"""
import zlib
import base64
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug 51/53/55/57 cam_high: table plane is the dominant z bin (0.9025); found at runtime by histogram, not hard-coded", "allowed": True},
    "OBJ_MIN_H/OBJ_MAX_H": {"source": "debug 51-57 cam_high: bottle top = table+0.154; plate/box tops table+0.015, bowl rim table+0.045, robot arm blob table+0.44", "allowed": True},
    "OBJ_MAX_FOOT": {"source": "debug 51-57: bottle footprint <= 0.03 x 0.05 m; rack/stove cluster spans > 0.24 m", "allowed": True},
    "DARK_MAX": {"source": "debug 51-57 cam_high RGB: bottle body mean ~[8,12,7]; wooden rack ~[110,100,85]; cabinet ~[63,61,59]", "allowed": True},
    "GROUND_TOL": {"source": "debug 51-57: bottle component's lowest top-cell is table+0.06; the floating arm blob's lowest cell is table+0.43", "allowed": True},
    "BODY_HI/BODY_LO": {"source": "debug 51-57 vertical profile: straight cylindrical body (diam 0.036-0.042) at ztop-0.115..ztop-0.075; neck (0.016) above it", "allowed": True},
    "GRASP_DH": {"source": "v2 probe on debug 51/53/55/57: commanding xy=bottle centre, z=ztop-0.09 closed on the bottle at width 0.015 with effort 3.0 in 4/4 seeds", "allowed": True},
    "HOLD_W_MIN/HOLD_W_MAX": {"source": "v2 probe debug 51-57: closed width on the bottle 0.0150-0.0162; free-air close would report the empty-jaw width", "allowed": True},
    "CAB_FLAT_TOL/CAB_MIN_PTS/CAB_MIN_SPAN": {"source": "debug 51-57: cabinet top is a flat slab (median z == max z) at table+0.223 spanning 0.25 x 0.19 m, 6500 points", "allowed": True},
    "EDGE_MARGIN": {"source": "debug 51-57: bottle radius 0.021; 0.07 keeps the base well inside the measured top rectangle", "allowed": True},
    "PLACE_GAP/CARRY_GAP": {"source": "generic clearance: hang length measured at runtime from the eef z at closure minus the table plane", "allowed": True},
    "CELL": {"source": "generic: 0.01 m top-down grid, ~ the depth-noise scale", "allowed": True},
    "APPROACH_H/DESCEND_STEP/DESCEND_SEC/DESCEND_STEPS/DESCEND_MIN_PROGRESS": {"source": "v3 probe on debug 51-65: the descent onto the bottle always stalls at ztop-0.042 (the gripper lands on the cap and the jaws straddle the 0.016 neck); a second push at that stall toppled the bottle on seed 52, so the descent is stepped and stopped on the first stalled step", "allowed": True},
    "MOVE_TRIES/MOVE_TOL": {"source": "generic controller mechanics: v2 showed a single api.move of 0.36 m leaves a 0.11 m residual, so moves are repeated until the residual converges", "allowed": True},
}

CELL = 0.01
OBJ_MIN_H = 0.10
OBJ_MAX_H = 0.30
OBJ_MAX_FOOT = 0.09
DARK_MAX = 60.0
GROUND_TOL = 0.09
BODY_HI = 0.115
BODY_LO = 0.075
GRASP_DH = 0.090
HOLD_W_MIN = 0.008
HOLD_W_MAX = 0.030
CAB_FLAT_TOL = 0.006
CAB_MIN_PTS = 150
CAB_MIN_SPAN = 0.10
EDGE_MARGIN = 0.07
PLACE_GAP = 0.006
CARRY_GAP = 0.055
MOVE_TOL = 0.015
APPROACH_H = 0.075
DESCEND_STEP = 0.018
DESCEND_SEC = 0.8
DESCEND_STEPS = 9
DESCEND_MIN_PROGRESS = 0.006
MOVE_TRIES = 4


# ---------------------------------------------------------------- perception
def cloud(fr):
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    d = np.asarray(fr.depth, dtype=np.float64)
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    x = (u - K[0, 2]) / K[0, 0] * d
    y = (v - K[1, 2]) / K[1, 1] * d
    pc = np.stack([x, y, d, np.ones_like(d)], -1)
    base = pc @ T.T
    ok = np.isfinite(d) & (d > 0.05) & (d < 5.0)
    return base[..., :3], ok


def components(mask):
    """4-connected labelling of a small 2-D bool grid."""
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    for p in map(tuple, np.argwhere(mask)):
        if lab[p]:
            continue
        cur += 1
        stack = [p]
        lab[p] = cur
        while stack:
            a, b = stack.pop()
            for q in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                if 0 <= q[0] < mask.shape[0] and 0 <= q[1] < mask.shape[1] \
                        and mask[q] and lab[q] == 0:
                    lab[q] = cur
                    stack.append(q)
    return lab, cur


def scene(api, tag="S"):
    fr = api.capture("cam_high")
    pts, ok = cloud(fr)
    rgb = np.asarray(fr.rgb, dtype=np.float64)
    x, y, z = pts[..., 0], pts[..., 1], pts[..., 2]
    ws = ok & (x > -0.60) & (x < 0.45) & (y > -0.55) & (y < 0.55) & (z > 0.5) & (z < 1.8)
    hist, edges = np.histogram(z[ws], bins=np.arange(0.5, 1.8, 0.005))
    table = float(edges[int(np.argmax(hist))]) + 0.0025
    api.log("%s table_z %.4f" % (tag, table))

    sel = ws & (z > table + 0.008)
    X, Y, Z = x[sel], y[sel], z[sel]
    C = rgb[sel]
    gx = np.floor((X + 0.60) / CELL).astype(int)
    gy = np.floor((Y + 0.55) / CELL).astype(int)
    nx, ny = int(1.05 / CELL) + 2, int(1.10 / CELL) + 2
    top = np.full((nx, ny), -1.0)
    np.maximum.at(top, (gx, gy), Z)
    lab, n = components(top > 0)
    key = gx * 10000 + gy
    objs = []
    for c in range(1, n + 1):
        cells = np.argwhere(lab == c)
        if len(cells) < 3:
            continue
        ck = cells[:, 0] * 10000 + cells[:, 1]
        m = np.isin(key, ck)
        objs.append(dict(
            id=c, ncell=len(cells), zbot=float(top[lab == c].min()),
            xlo=float(X[m].min()), xhi=float(X[m].max()),
            ylo=float(Y[m].min()), yhi=float(Y[m].max()),
            ztop=float(Z[m].max()), col=float(C[m].mean()), mask=m))
    objs.sort(key=lambda r: -r["ncell"])
    for r in objs[:10]:
        api.log("%s OBJ id=%d n=%d x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f zbot=%.3f col=%.0f"
                % (tag, r["id"], r["ncell"], r["xlo"], r["xhi"], r["ylo"], r["yhi"],
                   r["ztop"], r["zbot"], r["col"]))
    return dict(table=table, objs=objs, X=X, Y=Y, Z=Z, C=C)


def find_bottle(api, sc):
    table = sc["table"]
    cands = []
    for r in sc["objs"]:
        h = r["ztop"] - table
        if h < OBJ_MIN_H or h > OBJ_MAX_H:
            continue
        if (r["xhi"] - r["xlo"]) > OBJ_MAX_FOOT or (r["yhi"] - r["ylo"]) > OBJ_MAX_FOOT:
            continue
        if r["col"] > DARK_MAX or r["zbot"] > table + GROUND_TOL:
            continue
        cands.append(r)
    api.log("BOTTLE cands %d" % len(cands))
    if not cands:
        return None
    r = max(cands, key=lambda q: q["ztop"])
    m = r["mask"]
    X, Y, Z = sc["X"][m], sc["Y"][m], sc["Z"][m]
    ztop = r["ztop"]
    b = (Z > ztop - BODY_HI) & (Z < ztop - BODY_LO)
    if b.sum() < 20:
        b = (Z > ztop - 0.13) & (Z < ztop - 0.05)
    ylo, yhi = float(Y[b].min()), float(Y[b].max())
    rad = (yhi - ylo) / 2.0
    cy = (ylo + yhi) / 2.0
    cx = float(X[b].max()) - rad      # the camera sees only the near (+x) face
    api.log("BOTTLE ztop=%.3f r=%.3f centre=(%.3f,%.3f)" % (ztop, rad, cx, cy))
    return dict(cx=cx, cy=cy, rad=rad, ztop=ztop)


def find_cabinet(api, sc):
    table = sc["table"]
    best = None
    for r in sc["objs"]:
        if r["ncell"] < CAB_MIN_PTS or r["ztop"] < table + 0.15:
            continue
        m = r["mask"]
        X, Y, Z = sc["X"][m], sc["Y"][m], sc["Z"][m]
        t = np.abs(Z - r["ztop"]) < CAB_FLAT_TOL
        if t.sum() < CAB_MIN_PTS:
            continue
        sx, sy = float(X[t].max() - X[t].min()), float(Y[t].max() - Y[t].min())
        api.log("CAB cand id=%d ztop=%.3f flat=%d span=%.2fx%.2f" % (r["id"], r["ztop"], int(t.sum()), sx, sy))
        if sx < CAB_MIN_SPAN or sy < CAB_MIN_SPAN:
            continue
        cand = dict(ztop=r["ztop"], cx=float(X[t].mean()), cy=float(Y[t].mean()),
                    xlo=float(X[t].min()), xhi=float(X[t].max()),
                    ylo=float(Y[t].min()), yhi=float(Y[t].max()), n=int(t.sum()))
        if best is None or cand["n"] > best["n"]:
            best = cand
    if best:
        api.log("CABINET ztop=%.3f c=(%.3f,%.3f) x[%.3f,%.3f] y[%.3f,%.3f]"
                % (best["ztop"], best["cx"], best["cy"], best["xlo"], best["xhi"],
                   best["ylo"], best["yhi"]))
    return best


# ---------------------------------------------------------------- motion
def goto(api, tag, xyz, seconds=2.0, tries=MOVE_TRIES, tol=MOVE_TOL):
    res = None
    for i in range(tries):
        res = api.move([float(v) for v in xyz], seconds=seconds)
        e = api.eef()
        api.log("%s try%d res=%.4f eef=[%.4f,%.4f,%.4f]" % (tag, i, float(res), e[0], e[1], e[2]))
        if float(res) < tol:
            break
    return float(res)


def descend(api, tag, bot, zfloor):
    """Lower onto the bottle in small steps; stop as soon as the eef stops
    making progress (the gripper has landed on the bottle's cap/shoulder).
    A single blind push is what toppled the bottle on debug seed 52."""
    ztop = bot["ztop"]
    z = ztop + APPROACH_H
    goto(api, tag + ".pre", [bot["cx"], bot["cy"], z], seconds=2.0)
    last = float(api.eef()[2])
    for i in range(DESCEND_STEPS):
        z = max(zfloor, z - DESCEND_STEP)
        api.move([bot["cx"], bot["cy"], z], seconds=DESCEND_SEC)
        now = float(api.eef()[2])
        api.log("%s.step%d cmd=%.4f eefz=%.4f d=%.4f" % (tag, i, z, now, last - now))
        if (last - now) < DESCEND_MIN_PROGRESS:
            api.log("%s contact at z=%.4f (ztop-%.3f)" % (tag, now, ztop - now))
            return now
        last = now
        if z <= zfloor + 1e-6:
            break
    return float(api.eef()[2])


def holding(api):
    g = api.gripper()
    return (g["effort"] > 1.0) and (HOLD_W_MIN < g["width_m"] < HOLD_W_MAX), g


def dump(api, tag, fr):
    d = np.clip(np.asarray(fr.depth, np.float32) * 10000.0, 0, 65535).astype(np.uint16)
    for name, arr in (("D", d), ("C", np.asarray(fr.rgb, np.uint8))):
        b = base64.b64encode(zlib.compress(arr.tobytes(), 6)).decode()
        chunks = [b[i:i + 1700] for i in range(0, len(b), 1700)]
        api.log("%s %s NCHUNK %d" % (tag, name, len(chunks)))
        for i, c in enumerate(chunks):
            api.log("%s %s %d %s" % (tag, name, i, c))


# ---------------------------------------------------------------- behaviour
def run(api):
    api.log("v4 %r" % api.instruction())
    sc = scene(api, "S0")
    table = sc["table"]
    bot = find_bottle(api, sc)
    cab = find_cabinet(api, sc)
    if bot is None or cab is None:
        api.log("PERCEPTION FAILED")
        return

    zg = bot["ztop"] - GRASP_DH
    api.grip(0.08)
    held = False
    for attempt in range(2):
        goto(api, "a%d.above" % attempt, [bot["cx"], bot["cy"], table + 0.28], seconds=2.5)
        descend(api, "a%d" % attempt, bot, zg)
        api.grip(0.0)
        api.settle(0.4)
        held, g = holding(api)
        e = api.eef()
        api.log("a%d closed held=%s w=%.4f e=%.2f eefz=%.4f" % (attempt, held, g["width_m"], g["effort"], e[2]))
        if held:
            break
        api.grip(0.08)
        api.settle(0.3)
        goto(api, "a%d.retreat" % attempt, [bot["cx"], bot["cy"], table + 0.28], seconds=2.0)
        sc2 = scene(api, "S%d" % (attempt + 1))
        b2 = find_bottle(api, sc2)
        if b2 is not None:
            bot, table = b2, sc2["table"]
            zg = bot["ztop"] - GRASP_DH

    hang = float(api.eef()[2]) - table          # bottle base below the eef while held
    api.log("hang=%.4f held=%s" % (hang, held))

    zlift = table + 0.34
    goto(api, "lift", [bot["cx"], bot["cy"], zlift], seconds=2.5)

    tx = min(cab["cx"], cab["xhi"] - EDGE_MARGIN)
    tx = max(tx, cab["xlo"] + EDGE_MARGIN)
    ty = float(np.clip(cab["cy"], cab["ylo"] + EDGE_MARGIN, cab["yhi"] - EDGE_MARGIN))
    zplace = cab["ztop"] + hang + PLACE_GAP
    zcarry = zplace + CARRY_GAP
    api.log("PLACE target=(%.3f,%.3f) zplace=%.3f zcarry=%.3f" % (tx, ty, zplace, zcarry))

    goto(api, "carry", [tx, ty, max(zcarry, zlift)], seconds=3.0, tries=5)
    goto(api, "over", [tx, ty, zcarry], seconds=2.0, tries=3)
    goto(api, "down", [tx, ty, zplace], seconds=2.0, tries=3)
    api.grip(0.08)
    api.settle(0.6)
    goto(api, "up", [tx, ty, zplace + 0.09], seconds=2.0, tries=2)
    goto(api, "away", [tx + 0.10, ty - 0.14, table + 0.34], seconds=2.5, tries=2)
    api.settle(0.8)

    sc3 = scene(api, "SEND")
    b3 = find_bottle(api, sc3)
    if b3 is not None:
        api.log("END bottle at (%.3f,%.3f) ztop=%.3f" % (b3["cx"], b3["cy"], b3["ztop"]))
    fr = api.capture("cam_high")
    dump(api, "END", fr)
    api.log("v4 done")
