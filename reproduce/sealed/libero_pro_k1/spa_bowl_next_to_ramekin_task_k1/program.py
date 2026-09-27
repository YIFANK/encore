"""c2k1clean / spa_bowl_next_to_ramekin_task_k1 -- v3.

Intent: "Pick the akita black bowl next to the cookie box and place it on the
plate."  Two look-alike black bowls sit on the table; the brown cookie box is
what tells them apart, so the box is found first and the target bowl is the rim
nearest it.

Perception (one cam_high RGB-D frame, taken before the arm enters the scene):
  * deproject to the base frame, take the z-histogram mode as the table;
  * key every 1 cm cell on the MAX height above the table, so the cabinet, the
    arm and the stove drop out through a height window rather than a hand-drawn
    box, and a bowl can no longer fuse with the flat box beside it;
  * 3.0-7.5 cm cells = bowl/ramekin rims, 1.0-3.0 cm cells = flat props;
  * the brown flat blob (R-B) is the cookie box, the widest other one the plate.

Grasp is a rim pinch, not a centre grasp: the jaws span 78 mm and the bowl is
120 mm wide, so at the centre they simply rest on the rim (measured v1: the
descent stalls at ee z 0.949, 29 mm short, and closes on a 1 mm gap).  Offset
38 mm along the jaw axis instead and one finger drops inside the bowl while the
other drops outside it; closing pinches the wall.  Both packs grasp at that
same offset from the bowl and hold an 11.5 mm gap, which is the wall.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {
        "source": "debug-seed measurement: cam_high depth z-histogram mode over the "
                  "workspace = 0.9008 m on seeds 51/53/55/61; re-measured every "
                  "episode, the literal is only a fallback",
        "allowed": True},
    "WS": {
        "source": "debug-seed measurement: every table prop lies inside "
                  "x -0.33..0.33, y -0.40..0.45 (seed 51/53 cluster dump)",
        "allowed": True},
    "GRID": {"source": "generic camera mechanics: 1 cm cells on the deprojected cloud",
             "allowed": True},
    "RIM_BAND": {
        "source": "debug-seed measurement: bowl rims top out 0.051-0.055 m above the "
                  "table and the ramekin 0.043 m, while the cabinet is 0.227 m and the "
                  "arm 0.470 m (seeds 51/53/55/61)",
        "allowed": True},
    "FLAT_BAND": {
        "source": "debug-seed measurement: plate top 0.019 m, cookie box 0.020 m above "
                  "the table (seeds 51/53)",
        "allowed": True},
    "RIM_EXT": {
        "source": "debug-seed measurement: bowl/ramekin blob extents are 0.09-0.13 m "
                  "(seeds 51/53/55/61)",
        "allowed": True},
    "BROWN_MIN": {
        "source": "debug-seed measurement: cookie-box blob mean RGB [93,65,47] (R-B=46) "
                  "vs plate [155,145,143] (R-B=12) and bowls (R-B ~ 0..3), seed 51",
        "allowed": True},
    "HOLLOW_R": {
        "source": "debug-seed measurement: a bowl's hollow interior also produces a "
                  "flat-band blob at the bowl centre (seed 51: 0.135,-0.075), so flat "
                  "blobs within one bowl radius of a rim blob are dropped",
        "allowed": True},
    "RETRY_OFF": {
        "source": "debug-seed measurement: the hold gap is the bowl wall thickness at "
                  "the pinch height (0.007-0.012 m over seeds 51-65), so a failed rung "
                  "is retried with the bracket moved 6 mm in (0.038, which held seeds "
                  "51/53/59/61 in v4) and then 6 mm out (0.050), and finally across "
                  "the rim to a chord 22 mm either side of the bowl centre",
        "allowed": True},
    "PINCH_OFF": {
        "source": "both packs' grasp frames sit ~36 mm off the bowl centre "
                  "(k1: bowl (-0.180,0.320) vs grasp (-0.163,0.351)); debug-seed "
                  "diagnostic v2 confirmed a +y offset grips on seeds 51/55/61 "
                  "(effort 3.0, gap 7-12 mm). Set to 0.044 because in v4 the 0.038 rung "
                  "lost the bowl on the lift on seeds 55/57/63/65 (gap fell to "
                  "0.002-0.0045) while the 0.044 rung held every one of them "
                  "(gap 0.008-0.0092)",
        "allowed": True},
    "JAW_AXIS": {
        "source": "debug-seed measurement: api.tool_rotation() at home is "
                  "[[.998,0,-.057],[0,-1,0],[-.057,0,-.998]], i.e. the tool y axis is "
                  "base -y; v2 confirmed the pinch works along base y",
        "allowed": True},
    "GRASP_DZ": {
        "source": "both packs' ee z at the closing frame (0.922 / 0.919) is table+0.021 "
                  "/ +0.018; commanded 5 mm BELOW that (table+0.000) because the move "
                  "loop stops anywhere within its 12 mm tolerance and v3 seed 57 "
                  "stopped at table+0.028, pinching the thin upper lip and slipping "
                  "(gap 0.0085 -> 0.0047 on the lift) while seed 51 reached "
                  "table+0.023 and held (gap 0.0122). The inner finger bottoms on the "
                  "bowl's interior floor (measured table+0.027), so the press is "
                  "self-limiting.",
        "allowed": True},
    "PLACE_DZ": {
        "source": "both packs' ee z at the release frame (0.945 / 0.933) minus the "
                  "measured table z",
        "allowed": True},
    "CARRY_DZ": {
        "source": "packs' transport apex (k1 z=1.092, mate z=1.062) minus table z",
        "allowed": True},
    "OPEN_W": {
        "source": "debug-seed measurement: api.gripper() reports 0.0778 m open at "
                  "episode start (seed 51); FairApi opens for any width >= 0.025",
        "allowed": True},
    "HOLD_EFFORT": {"source": "FairApi contract: effort is 3.0 iff holding", "allowed": True},
}

TABLE_Z = 0.9008
WS = (-0.33, 0.33, -0.40, 0.45)
GRID = 0.01
RIM_BAND = (0.030, 0.075)
FLAT_BAND = (0.010, 0.030)
RIM_EXT = (0.07, 0.18)
BROWN_MIN = 25.0
HOLLOW_R = 0.07
PINCH_OFF = 0.044
# (extra offset along the jaw axis, offset across it).  The first three rungs
# slide the bracket along the jaw axis; the last two move it to a different
# CHORD of the rim, which is the only lever left when the bowl's interior floor
# sits high and the inner finger bottoms out before the wall gets thick
# (measured on seed 54: interior top 0.030 m vs 0.027 m elsewhere, close gap
# 0.0082-0.0096 and a slip on every axial rung).  The chord rungs need slightly
# less axial offset because the wall is nearer the centre line there:
# sqrt(0.058^2 - 0.022^2) = 0.0537 against a 0.058 m rim radius.
RETRY_OFF = ((0.0, 0.0), (-0.006, 0.0), (0.006, 0.0),
             (-0.004, 0.022), (-0.004, -0.022))
GRASP_DZ = 0.000
PLACE_DZ = 0.045
CARRY_DZ = 0.160
OPEN_W = 0.078
HOLD_EFFORT = 2.5


# ---------------------------------------------------------------- perception
def _cloud(frame):
    """Vectorised pixel -> base deprojection (same math as FairFrame.deproject)."""
    d = np.asarray(frame.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 0)
    z = np.where(ok, d, 1.0)
    pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z, np.ones_like(z)], -1)
    return (pc @ T.T)[..., :3], ok


def _label(occ):
    """4-connected components over the occupancy grid."""
    lab = np.zeros(occ.shape, int)
    n = 0
    H, W = occ.shape
    for i in range(H):
        for j in range(W):
            if not occ[i, j] or lab[i, j]:
                continue
            n += 1
            lab[i, j] = n
            stack = [(i, j)]
            while stack:
                a, b = stack.pop()
                for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    p, q = a + da, b + db
                    if 0 <= p < H and 0 <= q < W and occ[p, q] and not lab[p, q]:
                        lab[p, q] = n
                        stack.append((p, q))
    return lab, n


def perceive(api):
    f = api.capture("cam_high")
    P, ok = _cloud(f)
    rgb = f.rgb.astype(float)
    inws = (ok & (P[..., 0] > WS[0]) & (P[..., 0] < WS[1])
            & (P[..., 1] > WS[2]) & (P[..., 1] < WS[3]))
    zs = P[..., 2][inws]
    if zs.size > 1000:
        hist, edges = np.histogram(zs, bins=60,
                                   range=(np.percentile(zs, 1), np.percentile(zs, 99)))
        table = float(edges[int(np.argmax(hist))])
    else:
        table = TABLE_Z

    nx = int((WS[1] - WS[0]) / GRID) + 1
    ny = int((WS[3] - WS[2]) / GRID) + 1
    hmap = np.full((nx, ny), -1e9)
    cnt = np.zeros((nx, ny), int)
    col = np.zeros((nx, ny, 3))
    sel = inws & (P[..., 2] > table + 0.004)
    for a, b, z, c in zip(((P[..., 0][sel] - WS[0]) / GRID).astype(int),
                          ((P[..., 1][sel] - WS[2]) / GRID).astype(int),
                          P[..., 2][sel] - table, rgb[sel]):
        if z > hmap[a, b]:
            hmap[a, b] = z
        cnt[a, b] += 1
        col[a, b] += c

    def blobs(band, minc):
        occ = (cnt >= 3) & (hmap >= band[0]) & (hmap <= band[1])
        lab, n = _label(occ)
        out = []
        for k in range(1, n + 1):
            m = lab == k
            if m.sum() < minc:
                continue
            aa, bb = np.nonzero(m)
            xs = WS[0] + (aa + 0.5) * GRID
            ys = WS[2] + (bb + 0.5) * GRID
            c = col[m].sum(0) / max(1, cnt[m].sum())
            out.append({"cells": int(m.sum()),
                        "cx": float((xs.min() + xs.max()) / 2),
                        "cy": float((ys.min() + ys.max()) / 2),
                        "top": float(hmap[m].max()),
                        "ex": float(xs.max() - xs.min() + GRID),
                        "ey": float(ys.max() - ys.min() + GRID),
                        "rgb": [float(v) for v in c]})
        return out

    rims = [b for b in blobs(RIM_BAND, 25)
            if RIM_EXT[0] <= max(b["ex"], b["ey"]) <= RIM_EXT[1]]
    flats = blobs(FLAT_BAND, 15)
    # A bowl's hollow interior lands in the flat band too; drop it.
    free = [b for b in flats
            if all((b["cx"] - r["cx"]) ** 2 + (b["cy"] - r["cy"]) ** 2 > HOLLOW_R ** 2
                   for r in rims)]
    brown = [b for b in free if b["rgb"][0] - b["rgb"][2] >= BROWN_MIN]
    cookie = max(brown, key=lambda b: b["rgb"][0] - b["rgb"][2]) if brown else None
    rest = [b for b in free if b is not cookie]
    plate = max(rest, key=lambda b: max(b["ex"], b["ey"])) if rest else None

    bowl = None
    if rims:
        if cookie is not None:
            bowl = min(rims, key=lambda b: (b["cx"] - cookie["cx"]) ** 2
                       + (b["cy"] - cookie["cy"]) ** 2)
        elif len(rims) >= 2:
            # Fallback: the ramekin is the smallest rim and it stands beside the
            # OTHER bowl, so my bowl is the rim farthest from it.
            ram = min(rims, key=lambda b: b["cells"])
            bowl = max([b for b in rims if b is not ram],
                       key=lambda b: (b["cx"] - ram["cx"]) ** 2 + (b["cy"] - ram["cy"]) ** 2)
    for b in rims:
        api.log("RIM c=%d (%.3f,%.3f) top=%.3f ext=(%.2f,%.2f) rgb=%s"
                % (b["cells"], b["cx"], b["cy"], b["top"], b["ex"], b["ey"],
                   [int(v) for v in b["rgb"]]))
    for b in flats:
        api.log("FLAT c=%d (%.3f,%.3f) top=%.3f ext=(%.2f,%.2f) rb=%.0f"
                % (b["cells"], b["cx"], b["cy"], b["top"], b["ex"], b["ey"],
                   b["rgb"][0] - b["rgb"][2]))
    return table, bowl, plate, cookie


# -------------------------------------------------------------------- motion
def run(api):
    api.log("instr=%s" % api.instruction())
    table, bowl, plate, cookie = perceive(api)
    api.log("table=%.4f cookie=%s plate=%s bowl=%s"
            % (table,
               None if cookie is None else (round(cookie["cx"], 3), round(cookie["cy"], 3)),
               None if plate is None else (round(plate["cx"], 3), round(plate["cy"], 3)),
               None if bowl is None else (round(bowl["cx"], 3), round(bowl["cy"], 3))))
    if bowl is None or plate is None:
        api.log("ABORT: perception incomplete")
        return

    z_hover = table + 0.130
    z_grasp = table + GRASP_DZ
    z_carry = table + CARRY_DZ
    z_place = table + PLACE_DZ
    # The aim sits one pinch-offset off the bowl centre along the jaw axis, so
    # the bowl hangs one pinch-offset behind the tool: the release aims the
    # same offset past the plate centre to land the bowl on it.
    ax, ay = bowl["cx"], bowl["cy"] + PINCH_OFF
    px, py = plate["cx"], plate["cy"] + PINCH_OFF

    held = False
    for attempt, (extra, across) in enumerate(RETRY_OFF):
        aim = [ax + across, ay + extra]
        api.move([aim[0], aim[1], z_hover], seconds=1.1)
        r = api.move([aim[0], aim[1], z_grasp], seconds=0.8)
        api.grip(0.0)
        api.settle(0.3)
        e = api.eef()
        g = api.gripper()
        api.log("try%d aim=(%.3f,%.3f) z=%.4f res=%.4f gap=%.4f eff=%.2f"
                % (attempt, aim[0], aim[1], e[2], r, g["width_m"], g["effort"]))
        # Two-stage lift: a single 16 cm climb saturates the position command
        # and shook the bowl out of the pinch on half the v4/v5 rungs (gap fell
        # from ~0.012 to 0.002-0.0045); breaking it in two keeps the command
        # unsaturated for the part that matters, just off the table.
        api.move([aim[0], aim[1], table + 0.055], seconds=0.9)
        g0 = api.gripper()
        api.move([aim[0], aim[1], z_carry], seconds=0.9)
        g = api.gripper()
        api.log("lift%d low_gap=%.4f low_eff=%.2f" % (attempt, g0["width_m"], g0["effort"]))
        held = g["effort"] >= HOLD_EFFORT
        api.log("lift%d gap=%.4f eff=%.2f held=%s" % (attempt, g["width_m"], g["effort"], held))
        if held:
            ax, ay = aim
            break
        api.grip(OPEN_W)
    if not held:
        api.log("no hold; placing anyway")

    api.move([px, py, z_carry], seconds=1.3)
    g = api.gripper()
    api.log("overplate gap=%.4f eff=%.2f" % (g["width_m"], g["effort"]))
    api.move([px, py, z_place], seconds=1.0)
    api.grip(OPEN_W)
    api.settle(0.3)
    api.move([px, py, z_carry], seconds=0.9)
    api.log("end eef=%s" % np.round(api.eef(), 4).tolist())
