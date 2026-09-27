"""c2clean / spa_bowl_cookie_box_pos_k3  --  v7

Intent: "pick up the black bowl next to the cookie box and place it on the plate"

Mechanism (all geometry re-derived from this pack + debug-seed RGB-D):
  1. cam_high RGB-D -> base-frame height map on a 4 mm grid; the modal height
     is the table plane.
  2. Connected components above the table.  Each component is classified by
     two measurements only: its TOP height above the table, and the mean
     height of its own interior disc (hollow vs solid).
        bowl  : top >= 0.030 and interior < 0.6*top
        plate : 0.012 <= top < 0.030, interior < 0.6*top, rim radius > 0.045
        box   : interior >= 0.6*top  (filled to its own top => solid slab)
     The cookie box is the reddest solid.
  3. Target bowl = the bowl whose rim centre is nearest the cookie box
     ("next to the cookie box" is the instruction's own selector).
  4. Grasp = rim straddle.  The default down-wrist closes its jaws along the
     base y axis, so the grasp point is the rim point offset from the bowl
     centre along +-y; the side is chosen by clearance to everything else
     taller than 30 mm (the cabinet forbids the -y side in this scene).
  5. The bowl hangs from the pinch, so the eef leads the bowl centre by the
     same rim offset: the release eef goes to plate_centre + side*rim_offset,
     at plate_floor + the pack's own grasp depth above the table.
  6. Every long motion is a stack of short legs, and the carry is an L:
     climb to table+0.30, pull straight IN along -x at the grasp y, and only
     then swing +y to the plate.
     Why: over the bowl the arm is at FULL elbow extension -- probe2 measured
     joint 4 at -0.097 against its -0.07 stop.  A diagonal carry asks it to
     fold and rotate at once and it freezes at full stretch, which was the
     whole of v1/v2/v3's failure (the eef reaches the commanded y and z and
     sticks ~0.2 m short in x for the rest of the horizon).  The pull-in is a
     pure radial move: it folds the elbow monotonically (joint 4 -0.30 ->
     -2.33) and the swing afterwards is then free.  probe3 placed 5/5 on
     exactly the seeds the diagonal wedged on.
  8. The wedge is DETECTABLE: a leg that ends >30 mm from its own waypoint is
     the arm pinned, not a slow servo.  So the carry is a ladder of routes
     (altitude x order) and each is abandoned at the first leg that fails to
     track.  v5's route is tried first and still carries 12/15 debug seeds;
     the remaining three (52,58,62) pin joint 4 at its -0.070 stop during the
     lift, which is a null-space branch the OSC picks and cannot leave, and a
     different altitude or a swing-first order puts it in another branch.
  7. The approach must NOT climb first.  This arm is redundant, so the elbow
     posture the OSC settles into on the way to the grasp is still there
     during the carry: approaching over the top (v4) leaves a posture whose
     pull-in wedges at x~+0.03 on 5/8 seeds, while reaching the hover
     directly from home leaves one that pulls in cleanly.  Same carry code,
     opposite outcome -- so the approach is two plain moves, hover then down.
"""
import numpy as np

PROVENANCE = {
    "GRID_RES": {"source": "generic: 4 mm voxel, ~2x the cam_high ground sample distance",
                 "allowed": True},
    "WORKSPACE": {"source": "debug seeds 51-65 cam_high: the table surface visible to the arm",
                  "allowed": True},
    "PROP_MIN_H": {"source": "debug seeds 51-65: table plane depth noise is <2 mm, props stand >=19 mm",
                   "allowed": True},
    "BOWL_MIN_TOP": {"source": "debug seeds 51-65 cam_high: bowls measure top 0.042/0.051 m, "
                               "flat props 0.019 m", "allowed": True},
    "HOLLOW_FRAC": {"source": "debug seeds 51-65: bowl/plate interior 0.007 m vs own top 0.019-0.051; "
                              "the cookie box interior equals its top (0.019)", "allowed": True},
    "PLATE_MIN_R": {"source": "debug seeds 51-65 cam_high: plate rim radius 0.0645 m, "
                              "cookie box 0.030 m", "allowed": True},
    "WALL_GAP": {"source": "pack.json demos: gripper_state at the place keyframe, "
                           "|q0-q1| = 0.0086/0.0104/0.0076 m = the pinched rim wall",
                 "allowed": True},
    "GRASP_DEPTH": {"source": "pack.json ee_path6: close happens at z 0.9263/0.9228/0.9309, "
                              "table 0.9012 -> 0.0255 m above the table", "allowed": True},
    "PLACE_LIFT": {"source": "pack.json ee_path6: release at z 0.9332/0.9300/0.9372 = the same "
                             "0.0255 m above the surface the bowl lands on", "allowed": True},
    "HOVER_H": {"source": "pack.json ee_path6: pre-grasp pass height ~0.97-1.02, i.e. 0.07-0.12 m "
                          "over the rim", "allowed": True},
    "CARRY_H": {"source": "pack.json ee_path6: carry apex z 1.02-1.077 = table + 0.12..0.18",
                "allowed": True},
    "HIGH_H": {"source": "probe2/probe3 on debug seeds 51/53/57/59/61/63: at table+0.17 over the "
                         "bowl joint 4 reads -0.097 against its -0.07 stop and the carry freezes; "
                         "at table+0.30 joint 4 is -0.30 and the L route tracks every leg",
               "allowed": True},
    "OBSTACLE_H": {"source": "debug seeds 51-65: cabinet stands 0.226 m, bowls 0.042-0.051 m; "
                             "0.030 m separates a tall neighbour from the table", "allowed": True},
    "TIE_M": {"source": "debug seeds 51-65: the two candidate bowls sit 0.109-0.144 m and "
                        "0.146-0.195 m from the cookie box", "allowed": True},
    "LEG_M": {"source": "probe1 on debug seeds 51/53/57/61/65: the carry walked in 0.048 m legs "
                        "tracked within 12 mm every leg, where one long move froze 0.21 m short",
              "allowed": True},
    "WEDGE_TOL": {"source": "probe4/probe6 debug seeds: a tracking leg lands within 12 mm, a pinned "
                            "one lands 20-120 mm away and the eef moves the wrong way",
                  "allowed": True},
    "ROUTES": {"source": "probe2/probe3/probe5/probe6 and v6 on debug seeds: which altitude, which "
                         "order and whether the posture is reset first decides whether joint 4 "
                         "leaves its stop, and it differs per seed, so try them in turn",
               "allowed": True},
    "YAW_DEG": {"source": "generic wrist mechanics: a rotation about the tool's own approach axis "
                          "keeps a top-held bowl level; 90 deg is the largest such change",
                "allowed": True},
    "CORR_CAP": {"source": "generic controller mechanics: api.move's own tolerance is 12 mm, so a "
                           "bias-cancel overshoot beyond 20 mm is a runaway, not a correction",
                 "allowed": True},
}

GRID_RES = 0.004
WORKSPACE = (-0.35, 0.45, -0.45, 0.45)
PROP_MIN_H = 0.008
BOWL_MIN_TOP = 0.030
HOLLOW_FRAC = 0.6
PLATE_MIN_R = 0.045
WALL_GAP = 0.0086
GRASP_DEPTH = 0.0255
PLACE_LIFT = 0.0255
HOVER_H = 0.090
CARRY_H = 0.170
OBSTACLE_H = 0.030
TIE_M = 0.020
LEG_M = 0.055
CORR_CAP = 0.020
HIGH_H = 0.300
WEDGE_TOL = 0.030
# (name, height above table, swing-before-pull-in, posture action first).
# v5's rung is first; it carries 12/15 debug seeds on its own.  Once joint 4
# is on its stop no change of altitude or order frees it (v6, seed 52), so the
# later rungs first RESET the posture -- descend back over the now-empty pick
# spot, which refolds the elbow to the value it had at the grasp -- and then
# YAW the wrist about its own approach axis, which leaves the held bowl level
# but moves the arm into a different IK branch.
ROUTES = (("hiL", 0.300, False, None), ("hiS", 0.300, True, None),
          ("rstL", 0.300, False, "reset"), ("yawL", 0.300, False, "yaw"),
          ("yawS", 0.300, True, None), ("yaw2L", 0.300, False, "yaw"))
YAW_DEG = 90.0

GX = np.arange(WORKSPACE[0], WORKSPACE[1] + 1e-9, GRID_RES)
GY = np.arange(WORKSPACE[2], WORKSPACE[3] + 1e-9, GRID_RES)


# --------------------------------------------------------------------------- perception

def height_map(frame):
    d = frame.depth.astype(float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    z = np.where(np.isfinite(d) & (d > 0), d, np.nan)
    K, T = np.asarray(frame.intrinsics, float), np.asarray(frame.t_base_cam, float)
    P = np.stack([(uu - K[0, 2]) * z / K[0, 0],
                  (vv - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], -1)
    B = (P @ T.T)[..., :3]
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    m = (np.isfinite(Z) & (X > GX[0]) & (X < GX[-1]) & (Y > GY[0]) & (Y < GY[-1]))
    table = float(np.median(Z[m]))
    H = np.full((len(GX), len(GY)), np.nan)
    ix = ((X[m] - GX[0]) / GRID_RES).astype(int)
    iy = ((Y[m] - GY[0]) / GRID_RES).astype(int)
    o = np.argsort(Z[m])                       # tallest sample wins the cell
    H[ix[o], iy[o]] = Z[m][o]
    return H - table, table


def components(mask):
    lab = np.zeros(mask.shape, np.int32)
    cells = set(map(tuple, np.argwhere(mask)))
    n = 0
    for p in list(cells):
        if lab[p]:
            continue
        n += 1
        stack = [p]
        lab[p] = n
        while stack:
            a, b = stack.pop()
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    q = (a + da, b + db)
                    if q in cells and not lab[q]:
                        lab[q] = n
                        stack.append(q)
    return lab, n


def _kasa(xs, ys):
    A = np.c_[xs, ys, np.ones(len(xs))]
    s = np.linalg.lstsq(A, xs ** 2 + ys ** 2, rcond=None)[0]
    cx, cy = s[0] / 2.0, s[1] / 2.0
    return float(cx), float(cy), float(np.sqrt(max(s[2] + cx ** 2 + cy ** 2, 1e-9)))


def parse_scene(api):
    f = api.capture("cam_high")
    Hn, table = height_map(f)
    filled = np.nan_to_num(Hn, nan=-1.0)
    lab, n = components(filled > PROP_MIN_H)
    X, Y = np.meshgrid(GX, GY, indexing="ij")
    props = []
    for i in range(1, n + 1):
        m = lab == i
        if m.sum() < 30:
            continue
        top = float(np.nanmax(Hn[m]))
        if top > 0.15:
            continue                            # cabinet / arm / backdrop
        rim = m & (filled > top - 0.006)
        cx, cy, r = _kasa(X[rim], Y[rim])
        dd = np.sqrt((X - cx) ** 2 + (Y - cy) ** 2)
        inner = (dd < 0.5 * r) & np.isfinite(Hn)
        interior = float(np.nanmean(Hn[inner])) if inner.sum() else top
        props.append(dict(cx=cx, cy=cy, r=r, top=top, interior=interior,
                          n=int(m.sum()), mask=m))
    return table, props, Hn, filled


# --------------------------------------------------------------------------- motion

def precise_move(api, api_log, xyz, seconds=2.0, tries=2, tol=0.004):
    """api.move stops at its own 12 mm tolerance; cancel the standing error by
    re-issuing the target shifted by the observed miss.

    BOUNDED (v1 lesson): against a blocked axis an unbounded correction turns a
    stall into a runaway command — cap the overshoot at CORR_CAP and stop as
    soon as a correction fails to improve the miss.
    """
    tgt = np.asarray(xyz, float)
    res = api.move(tgt, seconds=seconds)
    best = float(np.linalg.norm(tgt - api.eef()))
    for _ in range(tries):
        err = tgt - api.eef()
        miss = float(np.linalg.norm(err))
        if miss < tol:
            break
        cmd = tgt + np.clip(err, -CORR_CAP, CORR_CAP)
        res = api.move(cmd, seconds=1.0)
        new = float(np.linalg.norm(tgt - api.eef()))
        if new >= best - 0.001:
            break
        best = new
    cur = api.eef()
    api_log("move -> %s got %s miss=%.4f residual=%.4f"
            % (np.round(tgt, 4).tolist(), np.round(cur, 4).tolist(),
               float(np.linalg.norm(tgt - cur)), res))
    return cur


def walk(api, api_log, tgt, leg, tag, seconds=0.5, tol=WEDGE_TOL):
    """Legs to tgt, abandoned at the first leg that fails to track.

    A leg that ends more than WEDGE_TOL from its own waypoint is the arm
    pinned against a joint stop (measured: joint 4 sitting at -0.070 while the
    eef drifts the wrong way), not a servo that needs more time.  Reporting it
    lets the caller try another route instead of burning the horizon.
    """
    start = api.eef()
    tgt = np.asarray(tgt, float)
    n = max(1, int(np.ceil(float(np.linalg.norm(tgt - start)) / leg)))
    for i in range(1, n + 1):
        w = start + (tgt - start) * (i / n)
        api.move(w, seconds=seconds)
        miss = float(np.linalg.norm(w - api.eef()))
        if miss > tol:
            api_log("%s WEDGED leg %d/%d want=%s got=%s q=%s"
                    % (tag, i, n, np.round(w, 3).tolist(), np.round(api.eef(), 3).tolist(),
                       np.round(api.proprio()["robot0_joint_pos"], 3).tolist()))
            return False
    api_log("%s ok -> %s" % (tag, np.round(api.eef(), 3).tolist()))
    return True


def yaw_tool(api, deg):
    """Rotate the tool about its OWN approach axis: the bowl stays level, the
    arm changes IK branch."""
    R = np.asarray(api.tool_rotation(), float)
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    api.move(api.eef(), rotation=R @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]),
             seconds=1.5)


def carry(api, api_log, place_xy, table, reset_xyz):
    """Ladder of carry routes; the first that tracks all the way wins."""
    for name, h, swing_first, pre in ROUTES:
        if pre == "reset":
            api.move(reset_xyz, seconds=2.0)
            api_log("%s: posture reset -> %s q=%s" % (
                name, np.round(api.eef(), 3).tolist(),
                np.round(api.proprio()["robot0_joint_pos"], 3).tolist()))
        elif pre == "yaw":
            yaw_tool(api, YAW_DEG)
            api_log("%s: yawed -> q=%s" % (
                name, np.round(api.proprio()["robot0_joint_pos"], 3).tolist()))
        z = table + h
        cur = api.eef()
        if not walk(api, api_log, [cur[0], cur[1], z], 0.060, name + ":climb"):
            continue
        cur = api.eef()
        legs = ([[cur[0], place_xy[1], z], [place_xy[0], place_xy[1], z]] if swing_first
                else [[place_xy[0], cur[1], z], [place_xy[0], place_xy[1], z]])
        ok = True
        for j, w in enumerate(legs):
            if not walk(api, api_log, w, LEG_M, "%s:leg%d" % (name, j)):
                ok = False
                break
        cur = api.eef()
        if ok and float(np.hypot(cur[0] - place_xy[0], cur[1] - place_xy[1])) < 0.030:
            api_log("carry route %s succeeded" % name)
            return z
        # perturb the posture before the next rung: back out and drop a little
        api.move([cur[0] + 0.05, cur[1] - 0.04, cur[2] - 0.05], seconds=0.8)
    api_log("carry: every route wedged")
    return None


def traverse(api, api_log, xy, z, leg=LEG_M):
    """Walk to (xy, z) in short legs.

    A single long diagonal move saturates every axis at once and wedges: on
    debug seeds 51/57/61/65 the eef reached the commanded y and z and then
    froze 0.21 m short in x for the rest of the 1000-step horizon. The same
    path walked in <=LEG_M legs tracked to within 12 mm on every leg
    (probe1, seeds 51/53/57/61/65).
    """
    start = api.eef()
    tgt = np.array([xy[0], xy[1], z], float)
    n = max(1, int(np.ceil(float(np.linalg.norm(tgt - start)) / leg)))
    for i in range(1, n + 1):
        w = start + (tgt - start) * (i / n)
        api.move(w, seconds=0.8)
    cur = api.eef()
    api_log("traverse %d legs -> %s got %s miss=%.4f"
            % (n, np.round(tgt, 4).tolist(), np.round(cur, 4).tolist(),
               float(np.linalg.norm(tgt - cur))))
    return cur


# --------------------------------------------------------------------------- policy

def run(api):
    L = api.log
    table, props, Hn, filled = parse_scene(api)
    L("table=%.4f props=%d" % (table, len(props)))
    for p in props:
        L("  prop c=(%+.3f,%+.3f) r=%.4f top=%.4f interior=%.4f n=%d"
          % (p["cx"], p["cy"], p["r"], p["top"], p["interior"], p["n"]))

    bowls = [p for p in props if p["top"] >= BOWL_MIN_TOP
             and p["interior"] < HOLLOW_FRAC * p["top"]]
    plates = [p for p in props if 0.012 <= p["top"] < BOWL_MIN_TOP
              and p["interior"] < HOLLOW_FRAC * p["top"] and p["r"] > PLATE_MIN_R]
    solids = [p for p in props if p["interior"] >= HOLLOW_FRAC * p["top"]]
    if not bowls or not plates:
        L("ABORT: bowls=%d plates=%d" % (len(bowls), len(plates)))
        return "no target"
    plate = max(plates, key=lambda p: p["r"])

    if solids:
        box = min(solids, key=lambda p: p["r"])
        ranked = sorted(bowls, key=lambda b: np.hypot(b["cx"] - box["cx"], b["cy"] - box["cy"]))
        d0 = np.hypot(ranked[0]["cx"] - box["cx"], ranked[0]["cy"] - box["cy"])
        d1 = (np.hypot(ranked[1]["cx"] - box["cx"], ranked[1]["cy"] - box["cy"])
              if len(ranked) > 1 else 9.0)
        L("cookiebox=(%+.3f,%+.3f) bowl dists %.3f / %.3f" % (box["cx"], box["cy"], d0, d1))
        bowl = ranked[0] if (d1 - d0) >= TIE_M else max(ranked[:2], key=lambda b: b["r"])
    else:
        bowl = max(bowls, key=lambda b: b["r"])
        L("no solid found; falling back to the widest bowl")

    rim_off = bowl["r"] - 0.5 * WALL_GAP
    X, Y = np.meshgrid(GX, GY, indexing="ij")
    tall = (filled > OBSTACLE_H) & (~bowl["mask"])
    best = None
    for s in (+1.0, -1.0):
        gx, gy = bowl["cx"], bowl["cy"] + s * rim_off
        d = np.sqrt((X - gx) ** 2 + (Y - gy) ** 2)
        near = tall & (d < 0.12)
        clear = float(np.min(d[near])) if near.any() else 0.12
        L("side %+d grasp=(%+.3f,%+.3f) clearance=%.3f" % (s, gx, gy, clear))
        if best is None or clear > best[0]:
            best = (clear, s, gx, gy)
    _, side, gx, gy = best

    rim_top = table + bowl["top"]
    grasp_z = rim_top - GRASP_DEPTH
    hover_z = rim_top + HOVER_H
    carry_z = table + CARRY_H
    place_xy = (plate["cx"], plate["cy"] + side * rim_off)
    place_z = table + plate["interior"] + PLACE_LIFT
    L("target bowl c=(%+.3f,%+.3f) r=%.4f top=%.4f | side=%+d grasp=(%+.3f,%+.3f,%.4f)"
      % (bowl["cx"], bowl["cy"], bowl["r"], bowl["top"], side, gx, gy, grasp_z))
    L("plate c=(%+.3f,%+.3f) r=%.4f floor=%.4f | place=(%+.3f,%+.3f,%.4f)"
      % (plate["cx"], plate["cy"], plate["r"], plate["interior"],
         place_xy[0], place_xy[1], place_z))

    high_z = table + HIGH_H
    api.grip(0.08)
    # straight in from home: hover, then down. No climb -- see note 7.
    r = api.move([gx, gy, hover_z], seconds=2.0)
    L("hover -> %s got %s res=%.4f" % ([round(gx, 4), round(gy, 4), round(hover_z, 4)],
                                       np.round(api.eef(), 4).tolist(), r))
    r = api.move([gx, gy, grasp_z], seconds=1.5)
    L("descend res=%.4f eef=%s q=%s" % (r, np.round(api.eef(), 4).tolist(),
                                        np.round(api.proprio()["robot0_joint_pos"], 3).tolist()))
    api.grip(0.0)
    api.settle(0.2)
    g = api.gripper()
    L("after close: width=%.4f effort=%.2f eef=%s"
      % (g["width_m"], g["effort"], np.round(api.eef(), 4).tolist()))

    api.move([gx, gy, high_z], seconds=2.0)
    g = api.gripper()
    L("after lift: width=%.4f effort=%.2f eef=%s q=%s"
      % (g["width_m"], g["effort"], np.round(api.eef(), 4).tolist(),
         np.round(api.proprio()["robot0_joint_pos"], 3).tolist()))

    reached_z = carry(api, L, place_xy, table, [gx, gy, grasp_z + 0.040])
    if reached_z is None:
        return "v7 carry wedged on every route"
    g = api.gripper()
    L("over plate: width=%.4f effort=%.2f q=%s"
      % (g["width_m"], g["effort"],
         np.round(api.proprio()["robot0_joint_pos"], 3).tolist()))
    traverse(api, L, place_xy, carry_z, leg=0.060)
    r = api.move([place_xy[0], place_xy[1], place_z], seconds=1.5)
    L("place descend res=%.4f" % r)
    g = api.gripper()
    L("at place: width=%.4f effort=%.2f eef=%s"
      % (g["width_m"], g["effort"], np.round(api.eef(), 4).tolist()))
    api.grip(0.08)
    api.settle(0.3)
    api.move([place_xy[0], place_xy[1], carry_z], seconds=1.2)
    L("done eef=%s" % np.round(api.eef(), 4).tolist())
    return "v7 placed"
