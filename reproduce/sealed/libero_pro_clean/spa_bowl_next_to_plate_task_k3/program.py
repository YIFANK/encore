"""c2clean / spa_bowl_next_to_plate_task_k3  -- v2

Intent: "Pick the akita black bowl next to the ramekin and place it on the plate".

v1 scored 4/5 on debug seeds 51,53,57,61,65.  The single loss (ep53) was a
PERCEPTION fault, not a manipulation fault: the plate was found by masking a
thin height band, and a neighbouring bowl's sloping wall passes through that
same band, so the bowl fused into the plate component and dragged the plate
centre 37 mm in +y.  The bowl was then released off the plate.

v2 replaces band-masking with a top-down MAX-Z HEIGHT MAP.  Each XY cell is
labelled by the highest thing standing in it, so a bowl occupies its cells at
bowl height and can never also appear at plate height.  Bowl and ramekin
neighbourhoods are additionally subtracted before the plate is read, which
removes the bowl-interior cells (the inner wall crosses the plate band).

v2 also closes the loop on the two XY moves that matter: api.move lands with a
direction-specific residual (up to 11 mm in +x on debug seeds), so the hover
pose is re-commanded with the measured error subtracted before descending.

v3 (= v2 apart from one constant): v2 scored 8/8 on debug seeds 51-58, but the
logs showed that api.gripper()'s `effort` field is a GAP THRESHOLD, not a
force reading -- four of those eight holds carried the bowl to the plate while
reporting effort 0.1 at a jaw width of 0.0044-0.0048 m.  v2's regrasp guard
fired below 0.004 m, i.e. 0.4 mm under a real hold, so a slightly thinner bite
would have made the program open its jaws in mid-air and drop a bowl it was
actually carrying.  Across 13 successful grasps the closed width never fell
below 0.0044 m, and an empty close reaches ~0, so the guard moves to 0.0025 m.
"""
import json

import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51/53/57/61/65: modal z of the cam_high cloud over the workspace = 0.9006 m (identical on all five)", "allowed": True},
    "TOP_BOWL": {"source": "debug-seed segmentation: bowl tops 0.0509-0.0512 m above table vs ramekin top 0.0429-0.0431 m", "allowed": True},
    "TOP_RAMEKIN": {"source": "debug-seed segmentation: ramekin top 0.0429-0.0431 m above table", "allowed": True},
    "TOP_PLATE": {"source": "debug-seed segmentation: plate top 0.0193-0.0194 m above table", "allowed": True},
    "R_BOWL": {"source": "debug-seed segmentation: bowl footprint 0.109-0.111 m -> radius 0.055 m", "allowed": True},
    "R_RAMEKIN": {"source": "debug-seed segmentation: ramekin footprint 0.085-0.088 m -> radius 0.044 m", "allowed": True},
    "MIN_PLATE_SPAN": {"source": "debug-seed segmentation: plate footprint 0.135-0.137 m, cookie box 0.082 m -> 0.10 m separates them", "allowed": True},
    "GRASP_Z": {"source": "both packs' keyframes: all 6 demos close the gripper at eef z 0.919-0.930 m = table+0.019..0.029", "allowed": True},
    "RIM_OFF": {"source": "mate pack keyframes: closing eef y (0.347/0.351/0.360) sits 0.04-0.05 m to +y of the bowl footprint centre; verified by v1 grasping 5/5 on debug seeds at +0.045", "allowed": True},
    "PLACE_Z": {"source": "both packs: release eef z 0.933-0.956 m = table+0.032..0.055", "allowed": True},
    "CARRY_Z": {"source": "both packs: lift between grasp and place peaks at eef z 1.05-1.10 m = table+0.15..0.20", "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics: tool-to-world matrix for a straight-down wrist, matching api.tool_rotation() at reset", "allowed": True},
    "JAW_OPEN": {"source": "debug-seed api.gripper() at reset: width_m 0.0778", "allowed": True},
    "HOLD_WIDTH": {"source": "debug seeds 51-58 + 51/53/57/61/65: 13 successful bowl-wall grasps closed to width 0.0044-0.0093 m; an empty close reaches ~0, so 0.0025 m separates hold from miss with margin on both sides", "allowed": True},
    "AIM_TOL": {"source": "debug-seed v1 runs: api.move XY residual up to 0.011 m, so a 0.004 m re-command tolerance is meaningful and reachable", "allowed": True},
}

TABLE_Z = 0.9006
TOP_BOWL = 0.046
TOP_RAMEKIN = (0.030, 0.0455)
TOP_PLATE = (0.013, 0.027)
R_BOWL = 0.055
R_RAMEKIN = 0.044
MIN_PLATE_SPAN = 0.10

GRASP_Z = TABLE_Z + 0.024
PLACE_Z = TABLE_Z + 0.048
CARRY_Z = TABLE_Z + 0.16
RIM_OFF = 0.045
JAW_OPEN = 0.078
HOLD_WIDTH = 0.0025
AIM_TOL = 0.004

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

XLIM = (-0.32, 0.30)
YLIM = (0.05, 0.45)
CELL = 0.006


# ---------------------------------------------------------------- perception
def cloud(frame):
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    d = np.asarray(frame.depth, float)
    d = np.where(np.isfinite(d) & (d > 0), d, 0.0)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1) @ T.T
    return P[..., 0], P[..., 1], P[..., 2], d > 0


def height_map(frame):
    """XY cell -> highest z standing in it, over the right-hand table region."""
    X, Y, Z, ok = cloud(frame)
    m = (ok & (Z > TABLE_Z + 0.004) & (Z < TABLE_Z + 0.14)
         & (X > XLIM[0]) & (X < XLIM[1]) & (Y > YLIM[0]) & (Y < YLIM[1]))
    gi = np.round(X[m] / CELL).astype(int)
    gj = np.round(Y[m] / CELL).astype(int)
    zz = Z[m] - TABLE_Z
    top = {}
    for k in range(gi.size):
        c = (int(gi[k]), int(gj[k]))
        v = float(zz[k])
        if v > top.get(c, -1.0):
            top[c] = v
    return top


def blobs(cells, link=0.014, minc=8):
    """Connected components over a set of XY cells."""
    r = int(np.ceil(link / CELL))
    seen, out = set(), []
    for c in cells:
        if c in seen:
            continue
        stack, comp = [c], []
        seen.add(c)
        while stack:
            a, b = stack.pop()
            comp.append((a, b))
            for da in range(-r, r + 1):
                for db in range(-r, r + 1):
                    n = (a + da, b + db)
                    if n in cells and n not in seen:
                        seen.add(n)
                        stack.append(n)
        if len(comp) < minc:
            continue
        xs = np.array([a for a, _ in comp]) * CELL
        ys = np.array([b for _, b in comp]) * CELL
        out.append(dict(n=len(comp),
                        xc=float((xs.min() + xs.max()) / 2),
                        yc=float((ys.min() + ys.max()) / 2),
                        dx=float(xs.max() - xs.min()),
                        dy=float(ys.max() - ys.min())))
    return out


def survey(api):
    top = height_map(api.capture("cam_high"))
    bowls = blobs({c for c, z in top.items() if z >= TOP_BOWL}, minc=40)

    def near(c, cx, cy, r):
        return np.hypot(c[0] * CELL - cx, c[1] * CELL - cy) < r

    def not_near_bowl(c):
        return not any(near(c, b["xc"], b["yc"], R_BOWL + 0.010) for b in bowls)

    ram_cells = {c for c, z in top.items()
                 if TOP_RAMEKIN[0] <= z < TOP_RAMEKIN[1] and not_near_bowl(c)}
    rams = blobs(ram_cells, minc=40)
    ramekin = max(rams, key=lambda c: c["n"]) if rams else None

    def free(c):
        if not not_near_bowl(c):
            return False
        if ramekin is not None and near(c, ramekin["xc"], ramekin["yc"],
                                        R_RAMEKIN + 0.010):
            return False
        return True

    plate_cells = {c for c, z in top.items()
                   if TOP_PLATE[0] <= z < TOP_PLATE[1] and free(c)}
    cands = [b for b in blobs(plate_cells, minc=40)
             if b["dx"] > MIN_PLATE_SPAN and b["dy"] > MIN_PLATE_SPAN]
    plate = max(cands, key=lambda c: c["n"]) if cands else None
    return bowls, ramekin, plate


def log(api, tag, obj):
    api.log(tag + " " + json.dumps(obj, default=lambda o: round(float(o), 4)))


def aim(api, x, y, z, seconds=3.0, tries=2):
    """Command an XY pose, then re-command with the measured error removed."""
    api.move([x, y, z], rotation=R_DOWN, seconds=seconds)
    for _ in range(tries):
        e = api.eef()
        ex, ey = float(e[0]) - x, float(e[1]) - y
        if np.hypot(ex, ey) <= AIM_TOL:
            break
        api.move([x - ex, y - ey, z], rotation=R_DOWN, seconds=2.0)
    e = api.eef()
    return [round(float(v), 4) for v in e]


# ------------------------------------------------------------------ the plan
def run(api):
    api.log("INSTRUCTION " + api.instruction())
    bowls, ramekin, plate = survey(api)
    log(api, "BOWLS", bowls)
    log(api, "RAMEKIN", ramekin)
    log(api, "PLATE", plate)

    if not bowls or plate is None:
        api.log("ABORT missing bowl or plate")
        return "no target"

    if ramekin is not None and len(bowls) > 1:
        target = min(bowls, key=lambda b: np.hypot(b["xc"] - ramekin["xc"],
                                                   b["yc"] - ramekin["yc"]))
    else:
        target = max(bowls, key=lambda b: b["n"])
    log(api, "TARGET", target)

    gx, gy = target["xc"], target["yc"] + RIM_OFF
    px, py = plate["xc"], plate["yc"] + RIM_OFF

    api.grip(JAW_OPEN)
    api.log("HOVER " + json.dumps(aim(api, gx, gy, CARRY_Z)))
    api.move([gx, gy, TABLE_Z + 0.075], rotation=R_DOWN, seconds=2.0)
    api.move([gx, gy, GRASP_Z], rotation=R_DOWN, seconds=2.0)
    api.log("AT_GRASP " + json.dumps([round(float(v), 4) for v in api.eef()]))

    api.grip(0.0)
    api.settle(0.4)
    log(api, "CLOSED", api.gripper())

    api.move([gx, gy, CARRY_Z], rotation=R_DOWN, seconds=3.0)
    g = api.gripper()
    log(api, "LIFTED", g)
    if g["width_m"] < HOLD_WIDTH:
        api.log("REGRASP: jaws empty after lift")
        api.grip(JAW_OPEN)
        bowls, ramekin, plate2 = survey(api)
        log(api, "RE_BOWLS", bowls)
        if bowls and plate2 is not None:
            plate = plate2
            px, py = plate["xc"] + 0.0, plate["yc"] + RIM_OFF
            if ramekin is not None and len(bowls) > 1:
                target = min(bowls, key=lambda b: np.hypot(b["xc"] - ramekin["xc"],
                                                           b["yc"] - ramekin["yc"]))
            else:
                target = max(bowls, key=lambda b: b["n"])
            gx, gy = target["xc"], target["yc"] + RIM_OFF
            aim(api, gx, gy, CARRY_Z)
            api.move([gx, gy, GRASP_Z], rotation=R_DOWN, seconds=2.5)
            api.grip(0.0)
            api.settle(0.4)
            api.move([gx, gy, CARRY_Z], rotation=R_DOWN, seconds=3.0)
            log(api, "RELIFTED", api.gripper())

    api.log("OVER_PLATE " + json.dumps(aim(api, px, py, CARRY_Z)))
    api.move([px, py, PLACE_Z], rotation=R_DOWN, seconds=2.5)
    api.log("AT_PLACE " + json.dumps([round(float(v), 4) for v in api.eef()]))
    api.grip(JAW_OPEN)
    api.settle(0.5)
    api.move([px, py, CARRY_Z], rotation=R_DOWN, seconds=2.0)
    api.settle(0.5)

    b2, r2, p2 = survey(api)
    log(api, "AFTER_BOWLS", b2)
    log(api, "AFTER_PLATE", p2)
    return "v2 done"
