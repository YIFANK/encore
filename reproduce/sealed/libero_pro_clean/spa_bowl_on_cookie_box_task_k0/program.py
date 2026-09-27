"""c2clean spa_bowl_on_cookie_box_task_k0 -- v9.

Intent: "Pick the akita black bowl on the top of the cabinet and place it on
the plate".  No demonstrations.

Receipt chain (own debug runs):
  v5  open jaws descending on the bare table (cam_high plane 0.900 m) stop at
      eef_z 0.9090 -> the lowest part of the hand is 0.009 m below the EEF.
  v6  a straddle of the rim wall on the +y arc closes with width 0.0067-0.0081 m
      and effort 3.00 (an empty close reads width 0.0010 m, effort 0.05); ep51
      carried it to the plate and scored, ep53 lost it during a single 2 s lift.
  v6  the descent always halts at eef_z ~1.186 = rim + 0.009, i.e. the hand
      bottoms out on the rim: the bite is only as deep as the rim lip, so the
      lift has to be gentle and the hold has to be re-checked as it goes.
v9 = v7's carry (which scored 15/15 on the full debug split) with a deeper
bite.  v8 tried a stepped 0.03 m descent onto the plate instead and did worse
(7/8 on the probe subset: the repeated re-commands swing the eef laterally and
shake the rim out of the jaws), so the carry is left alone.  Across v7's 15
seeds the closed gap tracks the grasp height -- eef 1.1825-1.1830 bit
0.0080-0.0098 m of wall, eef 1.1877-1.1887 only 0.0065-0.0071 m -- so v9
presses the hand down onto the rim before closing instead of accepting
wherever the first descent converged.

v7: effort (not width) is the hold test, the grasp command is bias-corrected
from the observed tracking error, the lift is stepped with a hold check per
step, and a lost grasp is retried on the next arc of the rim.
"""

import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-65 cam_high: dominant height-map plane at 0.897-0.900 m", "allowed": True},
    "CAB_TOP_Z": {"source": "debug seeds 51-65 cam_high: flat plateau at 1.124 m over the cabinet footprint", "allowed": True},
    "BOWL_BAND_LO": {"source": "debug seeds: cabinet-top bowl spans 1.140-1.180 m above that plateau", "allowed": True},
    "BOWL_BAND_HI": {"source": "debug seeds: robot arm re-enters the height map above 1.25 m", "allowed": True},
    "BOWL_H": {"source": "debug seeds: cabinet-top bowl ztop 1.180 minus cabinet top 1.124", "allowed": True},
    "RIM_R": {"source": "debug seeds 51-65: cabinet-top bowl footprint bbox 0.108-0.112 m -> radius 0.056", "allowed": True},
    "FINGER_DZ": {"source": "v5 debug: open-jaw descent onto the 0.900 m table stalls at eef_z 0.9090", "allowed": True},
    "PRESS_DZ": {"source": "v7 15-seed debug: seeds seating at eef 1.1825 bit 0.0080-0.0098 m of wall, seeds stopping at 1.1887 only 0.0065 m; 12 mm of extra command seats the hand on the lip", "allowed": True},
    "PINCH_DEPTH": {"source": "v6 debug: the hand bottoms out on the rim at eef_z rim+0.009, so command 10 mm lower and let it settle there", "allowed": True},
    "Z_CEIL": {"source": "v2 debug: commanded eef_z 1.3198 over the cabinet only reached 1.2964", "allowed": True},
    "HOLD_EFFORT": {"source": "v6 debug: a bite on the rim reads effort 3.00, an empty close reads 0.05", "allowed": True},
    "PLATE_BAND": {"source": "debug seeds 51-65: plate is the flat 0.136 m disc with ztop 0.920 on the +y table half", "allowed": True},
    "PLACE_CLEAR": {"source": "v6 ep51 scored releasing with the bowl base 5 mm over the 0.920 m plate top", "allowed": True},
    "R_DOWN": {"source": "generic: tool z anti-parallel to world z; api.tool_rotation() at reset is within 3.2 deg", "allowed": True},
    "GRID_RES": {"source": "generic perception choice (4 mm height-map cell)", "allowed": True},
}

TABLE_Z = 0.900
CAB_TOP_Z = 1.124
BOWL_BAND_LO = 1.140
BOWL_BAND_HI = 1.250
BOWL_H = 0.056
RIM_R = 0.056
FINGER_DZ = 0.009
PINCH_DEPTH = 0.010
Z_CEIL = 1.285
HOLD_EFFORT = 1.0
PLACE_CLEAR = 0.005
GRID_RES = 0.004
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
XR = (-0.60, 0.50)
YR = (-0.90, 0.60)


def height_map(api, cam="cam_high"):
    fr = api.capture(cam)
    dep = np.asarray(fr.depth, dtype=np.float64)
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    H, W = dep.shape
    v, u = np.mgrid[0:H, 0:W]
    x = (u - K[0, 2]) * dep / K[0, 0]
    y = (v - K[1, 2]) * dep / K[1, 1]
    P = np.stack([x, y, dep], -1).reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]
    P = P.reshape(H, W, 3)
    ok = np.isfinite(P).all(-1)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    w = ok & (X > XR[0]) & (X < XR[1]) & (Y > YR[0]) & (Y < YR[1]) & (Z > 0.5) & (Z < 1.8)
    nx = int((XR[1] - XR[0]) / GRID_RES)
    ny = int((YR[1] - YR[0]) / GRID_RES)
    G = np.full((nx, ny), -1.0)
    ix = ((X[w] - XR[0]) / GRID_RES).astype(int).clip(0, nx - 1)
    iy = ((Y[w] - YR[0]) / GRID_RES).astype(int).clip(0, ny - 1)
    np.maximum.at(G, (ix, iy), Z[w])
    return G


def cell_xy(i, j):
    return XR[0] + (i + 0.5) * GRID_RES, YR[0] + (j + 0.5) * GRID_RES


def components(mask):
    seen = np.zeros(mask.shape, bool)
    out = []
    for s in np.argwhere(mask):
        s = tuple(s)
        if seen[s]:
            continue
        seen[s] = True
        stack = [s]
        comp = []
        while stack:
            a, b = stack.pop()
            comp.append((a, b))
            for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                p, q = a + da, b + db
                if 0 <= p < mask.shape[0] and 0 <= q < mask.shape[1] and mask[p, q] and not seen[p, q]:
                    seen[p, q] = True
                    stack.append((p, q))
        out.append(np.array(comp))
    return out


def blob_stats(G, comp):
    pts = np.array([cell_xy(a, b) for a, b in comp])
    z = G[comp[:, 0], comp[:, 1]]
    return {"n": int(len(comp)),
            "cx": round(0.5 * (pts[:, 0].min() + pts[:, 0].max()), 4),
            "cy": round(0.5 * (pts[:, 1].min() + pts[:, 1].max()), 4),
            "w": round(float(np.ptp(pts[:, 0])), 4),
            "h": round(float(np.ptp(pts[:, 1])), 4),
            "ztop": round(float(z.max()), 4)}


def find_cab_bowl(G):
    cand = [blob_stats(G, c) for c in components((G > BOWL_BAND_LO) & (G < BOWL_BAND_HI)) if len(c) >= 40]
    cand = [s for s in cand if s["cy"] < -0.15]
    return max(cand, key=lambda s: s["n"]) if cand else None


def find_plate(G):
    cand = [blob_stats(G, c) for c in components((G > 0.908) & (G < 0.935)) if len(c) >= 60]
    cand = [s for s in cand if s["cy"] > 0.10 and s["cx"] > -0.15]
    return max(cand, key=lambda s: s["n"]) if cand else None


def holding(api):
    return api.gripper()["effort"] >= HOLD_EFFORT


def goto(api, xyz, seconds=2.0, tol=0.005, tries=2, tag=None):
    xyz = np.asarray(xyz, dtype=float)
    e = np.asarray(api.eef())
    for k in range(tries):
        api.move(xyz.tolist(), rotation=R_DOWN, seconds=seconds if k == 0 else 1.0)
        e = np.asarray(api.eef())
        if float(np.linalg.norm(e - xyz)) < tol:
            break
    if tag:
        api.log("%s|tgt %s eef %s" % (tag, xyz.round(4).tolist(), e.round(4).tolist()))
    return e


def attempt(api, cx, cy, rim, dx, dy, tag):
    """Straddle the rim wall on the (dx,dy) arc, bias-correcting the aim once."""
    gx, gy = cx + dx * RIM_R, cy + dy * RIM_R
    z_gr = rim + FINGER_DZ - PINCH_DEPTH
    api.grip(0.08)
    goto(api, [gx, gy, min(Z_CEIL, rim + 0.06)], seconds=2.0, tries=2, tag=tag + "_hi")
    e = goto(api, [gx, gy, z_gr], seconds=1.5, tol=0.004, tries=2, tag=tag + "_dn")
    ex, ey = e[0] - gx, e[1] - gy
    if abs(ex) > 0.003 or abs(ey) > 0.003:
        e = goto(api, [gx - ex, gy - ey, z_gr], seconds=1.0, tol=0.003, tries=2, tag=tag + "_fix")
    # press onto the rim: the hand bottoms out on the lip, and a deeper seat
    # puts a thicker slice of wall between the jaws
    for _ in range(2):
        api.move([e[0], e[1], z_gr - 0.012], rotation=R_DOWN, seconds=0.8)
    e = np.asarray(api.eef())
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("%s|closed w %.4f e %.2f eef %s" % (tag, g["width_m"], g["effort"], e.round(4).tolist()))
    return e


def stepped_lift(api, x, y, z0, z1, step=0.018):
    z = z0
    while z < z1 - 1e-6:
        z = min(z1, z + step)
        api.move([x, y, z], rotation=R_DOWN, seconds=0.8)
        if not holding(api):
            api.log("lift|LOST at %.4f" % np.asarray(api.eef())[2])
            return False
    api.log("lift|ok eef %s grip %s" % (np.asarray(api.eef()).round(4).tolist(), api.gripper()))
    return True


def run(api):
    api.log("instr|%s" % api.instruction())
    G = height_map(api)
    bowl = find_cab_bowl(G)
    plate = find_plate(G)
    api.log("bowl|%s" % bowl)
    api.log("plate|%s" % plate)
    if bowl is None or plate is None:
        api.log("abort|perception")
        return
    cx, cy, rim = bowl["cx"], bowl["cy"], bowl["ztop"]

    grasp = None
    for dx, dy, tag in ((0.0, 1.0, "a1"), (0.0, 1.0, "a2"), (1.0, 0.0, "a3"), (0.0, -1.0, "a4")):
        e = attempt(api, cx, cy, rim, dx, dy, tag)
        if holding(api):
            z_top = min(Z_CEIL, rim + 0.09)
            if stepped_lift(api, e[0], e[1], e[2], z_top):
                grasp = (dx, dy, e)
                break
        api.grip(0.08)
        api.settle(0.2)
        p = np.asarray(api.eef())
        goto(api, [p[0], p[1], min(Z_CEIL, rim + 0.06)], seconds=1.2, tol=0.008, tries=1, tag=tag + "_up")

    api.log("grasped|%s" % (grasp is not None))
    if grasp is None:
        return
    dx, dy, ge = grasp
    hang = ge[2] - (rim - BOWL_H)
    api.log("hang|%.4f" % hang)

    px = plate["cx"] + dx * RIM_R
    py = plate["cy"] + dy * RIM_R
    z_car = min(Z_CEIL, rim + 0.09)
    goto(api, [px, py, z_car], seconds=3.0, tol=0.010, tries=2, tag="over")
    if not holding(api):
        api.log("carry|LOST")
        return
    z_rel = plate["ztop"] + PLACE_CLEAR + hang
    goto(api, [px, py, z_rel + 0.08], seconds=2.0, tol=0.008, tries=2, tag="pre")
    goto(api, [px, py, z_rel], seconds=1.5, tol=0.004, tries=2, tag="place")
    api.log("at_place|grip %s" % api.gripper())
    api.grip(0.08)
    api.settle(0.5)
    for dz in (0.02, 0.05, 0.10):
        api.move([px, py, z_rel + dz], rotation=R_DOWN, seconds=0.8)
    api.settle(0.5)
    api.log("retreat|eef %s" % np.asarray(api.eef()).round(4).tolist())
    G3 = height_map(api)
    api.log("final|%s" % [blob_stats(G3, c) for c in components((G3 > 0.928) & (G3 < 1.10)) if len(c) > 30])
