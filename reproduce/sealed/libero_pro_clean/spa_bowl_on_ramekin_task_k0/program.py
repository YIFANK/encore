"""c2clean spa_bowl_on_ramekin_task_k0 -- v7: rim pinch + place on the plate.

Identical to v5 (15/15 on the debug band) except that read_scene falls back to
looser cluster bands instead of aborting, for layouts outside the debug draw.

Mechanism.  The target bowl's rim is 0.111 m across but the jaws open only
0.0794 m, so the bowl cannot be straddled; and the wall flares outward going up,
so a centred descent would have to pass a finger through the wall.  The only
top-down grip is a rim pinch: park the tool centre over the wall itself, on the
+y side of the rim (the jaws close along world y when rotation=R_DOWN), drop the
fingertips D_PINCH below the rim so one finger is inside the bowl and one
outside, and close.

Heights.  Fingertips sit TIP_DZ below the EEF frame (measured by pressing the
open gripper onto bare table).  Every pose is closed-loop (goto) because the
open-loop tracking bias is pose dependent (+7 to +11 mm in z).
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug-seed 51 cam_high deprojection of bare-table pixels (0.9014, 1e5 pts)", "allowed": True},
    "TIP_DZ": {"source": "debug seeds 51/53/57/61 v4: open gripper pressed onto bare table stalls at eef z 0.9089 => 0.0075 above TABLE_Z", "allowed": True},
    "TOUCH_XY": {"source": "debug seeds 51-65 v2 height maps: (-0.07,0.00) is bare table in every seed", "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics: tool-down orientation, matches home tool_rotation to 3 deg", "allowed": True},
    "BOWL_ZTOP_BAND": {"source": "debug seeds 51-65 v2: target bowl ztop = TABLE_Z+0.070; distractor bowl TABLE_Z+0.100", "allowed": True},
    "BOWL_ROUND_TOL": {"source": "debug seeds 51-65 v2: target rim |rx-ry|<0.0006; distractor 0.030, robot base 0.027", "allowed": True},
    "PLATE_ZTOP_BAND": {"source": "debug seeds 51-65 v2: plate ztop = TABLE_Z+0.019, w=h=0.135-0.140; stove slab TABLE_Z+0.031, w=0.19", "allowed": True},
    "BOWL_H": {"source": "debug seed 57 v2: cookie-box top visible at z=0.9172 under the bowl, rim top 0.9712 => bowl height 0.054", "allowed": True},
    "D_PINCH": {"source": "debug seeds 51/53/57/61 v3: fingertips 0.020 below the rim closed on the wall with effort 3.0 in 4/4", "allowed": True},
    "WALL_HALF_T": {"source": "debug seeds 51/53/57/61 v3: closed gap on the pinched wall 0.0050-0.0058 => half-thickness 0.0025", "allowed": True},
    "READ_SCENE_FALLBACK_BANDS": {"source": "debug seeds 51-65 v2: strict bands are the measured spread (bowl dz 0.0693-0.0700, |rx-ry| <0.0006, n 1227-1312; plate dz 0.0192-0.0194, w/h 0.135-0.140); the two looser tiers widen those bands while staying clear of the distractor bowl (dz 0.0993) and, via x>-0.12, of the stove slab and robot base", "allowed": True},
    "AIM_TOLERANCE": {"source": "debug seeds 51/55/59/63 v6a-v6i: radial aim displaced -0.040..+0.040 -> pass until the inner finger clears the wall (r_tool-0.0397 > r_wall); +0.040 scored 1/4, -0.040 scored 4/4", "allowed": True},
}

TABLE_Z = 0.9014
TIP_DZ = 0.0075
CELL = 0.005
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
BOWL_H = 0.054
D_PINCH = 0.020
WALL_HALF_T = 0.0025


# ---------------------------------------------------------------- perception
def to_cloud(frame):
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    d = np.nan_to_num(np.asarray(frame.depth, float), nan=0.0)
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    x = (u - K[0, 2]) / K[0, 0] * d
    y = (v - K[1, 2]) / K[1, 1] * d
    return np.stack([x, y, d], -1) @ T[:3, :3].T + T[:3, 3]


def _components(mask):
    lab = np.zeros(mask.shape, np.int32)
    out = []
    cur = 0
    for s in zip(*np.nonzero(mask)):
        if lab[s]:
            continue
        cur += 1
        lab[s] = cur
        stack = [s]
        px = []
        while stack:
            i, j = stack.pop()
            px.append((i, j))
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = i + di, j + dj
                if 0 <= a < mask.shape[0] and 0 <= b < mask.shape[1] and mask[a, b] and not lab[a, b]:
                    lab[a, b] = cur
                    stack.append((a, b))
        out.append(np.array(px))
    return out


def clusters(P, zlo, zhi, xlo=-0.50, xhi=0.40, ylo=-0.50, yhi=0.50, minpx=25):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (X > xlo) & (X < xhi) & (Y > ylo) & (Y < yhi) & (Z > zlo) & (Z < zhi)
    nx = int((xhi - xlo) / CELL) + 1
    ny = int((yhi - ylo) / CELL) + 1
    H = np.zeros((nx, ny))
    np.maximum.at(H, (((X[m] - xlo) / CELL).astype(np.int32),
                      ((Y[m] - ylo) / CELL).astype(np.int32)), Z[m])
    out = []
    for px in _components(H > 0):
        if len(px) < minpx:
            continue
        xs = xlo + px[:, 0] * CELL
        ys = ylo + px[:, 1] * CELL
        zs = H[px[:, 0], px[:, 1]]
        out.append(dict(n=len(px), ztop=float(zs.max()),
                        mx=float((xs.min() + xs.max()) / 2.0),
                        my=float((ys.min() + ys.max()) / 2.0),
                        w=float(xs.max() - xs.min()), h=float(ys.max() - ys.min())))
    return out


def refine_ring(P, cx, cy, ztop, rmax=0.09):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    band = None
    for _ in range(4):
        band = (np.abs(X - cx) < rmax) & (np.abs(Y - cy) < rmax) & (Z > ztop - 0.012) & (Z < ztop + 0.006)
        if band.sum() < 20:
            return cx, cy, 0.0, 0.0, 0
        cx = float((X[band].min() + X[band].max()) / 2.0)
        cy = float((Y[band].min() + Y[band].max()) / 2.0)
    return cx, cy, float((X[band].max() - X[band].min()) / 2.0), \
        float((Y[band].max() - Y[band].min()) / 2.0), int(band.sum())


def wall_radius(P, cx, cy, z, half=0.004):
    """Inner-wall radius of the flared bowl at height z."""
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (np.abs(X - cx) < 0.09) & (np.abs(Y - cy) < 0.09) & (np.abs(Z - z) < half)
    if m.sum() < 10:
        return None
    r = np.sqrt((X[m] - cx) ** 2 + (Y[m] - cy) ** 2)
    return float(np.percentile(r, 90))


def _pick_bowl(P, cs, round_tol, nmin, dzlo, dzhi, wlo, whi):
    best = None
    for c in cs:
        if c["mx"] < -0.12:
            continue
        dz = c["ztop"] - TABLE_Z
        if not (dzlo < dz < dzhi and wlo < c["w"] < whi and wlo < c["h"] < whi):
            continue
        cx, cy, rx, ry, n = refine_ring(P, c["mx"], c["my"], c["ztop"])
        if abs(rx - ry) < round_tol and n > nmin and (best is None or n > best["n"]):
            best = dict(cx=cx, cy=cy, r=(rx + ry) / 2.0, ztop=c["ztop"], n=n)
    return best


def _pick_plate(P, cs, dzlo, dzhi, wlo, whi):
    best = None
    for c in cs:
        if c["mx"] < -0.12:
            continue
        dz = c["ztop"] - TABLE_Z
        if dzlo < dz < dzhi and wlo < c["w"] < whi and wlo < c["h"] < whi:
            if best is None or c["n"] > best["n"]:
                best = dict(cx=c["mx"], cy=c["my"], ztop=c["ztop"],
                            r=(c["w"] + c["h"]) / 4.0, n=c["n"])
    return best


def read_scene(api):
    """Target bowl = the only round-rimmed prop 70mm above the table; plate = the
    19mm-high disc.  The stove slab (31mm, 190mm wide) and the robot base (rim
    |rx-ry|=0.027) are excluded by those two tests plus x>-0.12.

    The tight bands below are what all 15 debug seeds satisfy with large margin.
    If an unseen layout falls outside them the strict pass returns nothing, and
    rather than abort (a certain zero) we retry with progressively looser bands;
    the ztop test is relaxed last because it is the one that separates the target
    bowl from the distractor bowl 30mm above it."""
    P = to_cloud(api.capture("cam_high"))
    cs = clusters(P, TABLE_Z + 0.004, 1.10)
    bowl = _pick_bowl(P, cs, 0.006, 400, 0.055, 0.085, 0.085, 0.135)
    if bowl is None:
        bowl = _pick_bowl(P, cs, 0.012, 200, 0.050, 0.090, 0.075, 0.145)
    if bowl is None:
        bowl = _pick_bowl(P, cs, 0.020, 100, 0.045, 0.092, 0.065, 0.155)
    plate = _pick_plate(P, cs, 0.012, 0.027, 0.115, 0.16)
    if plate is None:
        plate = _pick_plate(P, cs, 0.009, 0.029, 0.100, 0.175)
    if plate is None:
        plate = _pick_plate(P, cs, 0.006, 0.032, 0.090, 0.190)
    return P, bowl, plate


# ---------------------------------------------------------------- motion
def goto(api, target, seconds=2.0, tol=0.004, tries=3, tag=""):
    """Closed-loop EEF move: the open-loop tracking bias is pose dependent, so
    re-issue with the observed error folded into the command (bounded)."""
    target = np.asarray(target, float)
    cmd = target.copy()
    e = np.asarray(api.eef(), float)
    for i in range(tries):
        api.move(cmd.tolist(), R_DOWN, seconds)
        e = np.asarray(api.eef(), float)
        err = target - e
        if tag:
            api.log("  %s try%d cmd=(%.4f,%.4f,%.4f) eef=(%.4f,%.4f,%.4f) err=%.4f"
                    % (tag, i, cmd[0], cmd[1], cmd[2], e[0], e[1], e[2], float(np.linalg.norm(err))))
        if np.linalg.norm(err) <= tol:
            break
        cmd = cmd + np.clip(err, -0.030, 0.030)
    return e


def tip_to_eef(z):
    return z + TIP_DZ


def run(api):
    P, bowl, plate = read_scene(api)
    api.log("BOWL %s" % bowl)
    api.log("PLATE %s" % plate)
    if bowl is None or plate is None:
        api.log("ABORT perception")
        return

    z_f = bowl["ztop"] - D_PINCH
    rw = wall_radius(P, bowl["cx"], bowl["cy"], z_f)
    if rw is None:
        rw = bowl["r"] * 0.865
    r_tool = rw + WALL_HALF_T
    gx, gy = bowl["cx"], bowl["cy"] + r_tool
    api.log("PLAN z_f=%.4f rw=%.4f r_tool=%.4f grasp=(%.4f,%.4f)" % (z_f, rw, r_tool, gx, gy))

    api.grip(0.08)
    api.settle(0.2)

    # ---- pick
    goto(api, [gx, gy, tip_to_eef(bowl["ztop"] + 0.045)], 2.0, tag="APPROACH")
    goto(api, [gx, gy, tip_to_eef(z_f)], 2.0, tol=0.003, tag="DESCEND")
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("CLOSED grip=%s eef=%s" % (g, np.asarray(api.eef()).round(4).tolist()))

    # ---- lift, then check the grip survived it
    lift_z = tip_to_eef(bowl["ztop"] + 0.070)
    e = goto(api, [gx, gy, lift_z], 2.5, tol=0.006, tag="LIFT")
    api.settle(0.3)
    g = api.gripper()
    api.log("LIFTED grip=%s eef=%s" % (g, np.asarray(api.eef()).round(4).tolist()))
    holding = g["width_m"] > 0.0015 and g["effort"] > 1.0
    api.log("HOLDING %s" % holding)

    # ---- carry to the plate and set the bowl down on it
    # held by the wall on its +y side => bowl centre sits r_tool in -y from the tool
    px, py = plate["cx"], plate["cy"] + r_tool
    goto(api, [px, py, lift_z], 2.5, tol=0.006, tag="CARRY")
    place_tip = plate["ztop"] + (BOWL_H - D_PINCH) + 0.002
    goto(api, [px, py, tip_to_eef(place_tip)], 2.0, tol=0.004, tag="PLACE")
    api.log("AT_PLACE grip=%s eef=%s" % (api.gripper(), np.asarray(api.eef()).round(4).tolist()))
    api.grip(0.08)
    api.settle(0.5)
    api.log("RELEASED grip=%s" % api.gripper())

    # ---- retreat clear of the scene, then report where things ended up
    goto(api, [px, py, tip_to_eef(plate["ztop"] + 0.12)], 2.0, tol=0.010, tag="RETREAT")
    api.settle(0.3)
    P2, bowl2, plate2 = read_scene(api)
    api.log("AFTER_BOWL %s" % bowl2)
    api.log("AFTER_PLATE %s" % plate2)
