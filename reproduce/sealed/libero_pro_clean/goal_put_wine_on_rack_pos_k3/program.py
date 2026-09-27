"""v12 — v9 with a coarser lying descent so the release fits the step horizon.

Receipt chain on the 15 debug seeds:
  v2 open loop                                 3/8 probe
  v3 closed-loop aim + hold check              8/8 probe, 13/15 formal
  v4 v3 + park + height test + finer descent + up-slope bias   9/15
  v5 v4 with the up-slope bias reverted + lying branch          2/15

  v6 v3's motion + only the perception fixes        14/15
  v7 v6 + budget trim + stepped lying reorientation  14/15
  v8 v7 + starvation-proof lying transit             14/15
  v9  v8 + lying lay-down by yaw only                14/15
  v10 v9 + lying carry lowered to 1.278              14/15
  v11 v10 + 3 rot sub-steps                          14/15
  v12 v9 + coarser lying descent                     (this file)

v9's lying path worked mechanically: the yaw held the eef to within 1 mm, the
descent reached 1.2419 against a 1.2283 target, and the hand was still holding
at effort 3.0 over the rack centre. It lost only on budget -- 1000 sim steps, so
the release and settle were never simulated. v10 tried to buy that budget by
carrying lower (1.278 instead of 1.34) and that is worse, not better: the eef
froze bit-identical at z=1.2837 from the transit onward through rot1-3 and six
descent moves, which is the hand jammed against the rack (its observed top is
1.213 and the posts run higher), not starvation. v11 confirmed it -- restoring
3 rot sub-steps changed nothing while the carry stayed low.

So v12 keeps v9's high carry and buys the budget where it is actually free: the
lying descent covers 0.11 m, and taking it in 0.055 steps instead of 0.030
removes three moves worth about 180 steps.

v8 fixed the lying transit (ep64 reached the carry pose at z=1.343) but the
lay-down rotation then dragged the eef UP 0.12 m across its three sub-steps and
wedged the arm at z=1.474, so the bottle was let go 0.27 m above the shelf.
Rotating a horizontal bottle onto the slope needs ~70 deg of yaw AND 32 deg of
pitch, and the wrist cannot hold position through it. v9 keeps only the yaw --
it lines the bottle up along the rack's slope line while leaving it horizontal,
which is a rotation about the tool's own approach axis and so barely moves the
eef. Gravity does the rest on a 30 deg ramp.

v7's trims turned out to be free but inert: `seconds` only sets a move's step
CAP (max(60, 20*seconds)), so a move that converges costs what it costs and the
14 standing seeds reported byte-identical sim_steps under v6 and v7. What burns
budget is a move that STARVES -- it spends its whole cap and still misses. That
is also why v7's ep64 got worse: dropping the intermediate carry waypoint left
one 0.30 m transit that starved at 70 steps, stranding the arm at z=1.086 so
every later move inherited the miss. v8 keeps the standing path bit-identical
and, for the lying path only, lifts clear before translating and re-commands the
transit until the eef actually arrives.

The v4/v5 collapse was NOT geometry. `sim_steps` in results.jsonl names the
cause: the episode horizon is 1000 steps, and v5 hit exactly 1000 on 13 of 15
seeds, so its release and settle were never simulated at all. On ep51 v5 and v6
put the eef within 0.3 mm of the same release pose; v5 spent 1000 steps and
scored 0, v6 spent 783 and scored 1. The extra descent moves, not their
end pose, were what cost v4/v5. The same budget explains v3's two losses: its
ep60 and ep64 both read 1000 steps.

So the step budget is a first-class resource here, and v7 trims it (aim capped
at 3 iterations, shorter park, shorter post-release retreat, one carry move for
the lying path) to widen every seed's margin -- v6 left only 30 steps of slack
on ep60 and none on ep64.

v6/v7 keep v3's descent character exactly and import only what perception
receipts justified:

  - park the hand at (-0.05, 0.33, 1.30) before each capture and drop v3's
    0.26 m eef box: on ep64 that box swallowed the bottle and perception fell
    through to the stove knob (top = table+0.06), so the run grasped the bowl.
  - refuse candidates shorter than table+0.11 (standing bottle tops out at
    table+0.151; tallest decoy 0.059).
  - a lying-bottle branch: on ep64 the bottle settles ONTO ITS SIDE during the
    env reset, so there is no standing candidate at all. It is grasped across
    its own axis by yawing the wrist and laid down by rotating that axis onto
    the rack slope. v6 proved the grasp (width 0.044, effort 3.0) but issued
    that large reorientation as one move_pose, which starved and left the eef
    0.05 m high and 0.17 m above the release target; v7 walks the rotation in
    equal sub-steps instead.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-65 cam_high depth: the flat table plane deprojects to z=0.901", "allowed": True},
    "DARK_GREY": {"source": "debug seeds: wine-bottle body pixels have mean RGB 6-15; table/rack/stove are >80", "allowed": True},
    "BOTTLE_FOOT_MAX": {"source": "debug seeds: the bottle's dark cluster footprint is 0.04 m; dark rack/cabinet structures span >0.25 m", "allowed": True},
    "NECK_DROP": {"source": "debug seeds: bottle top (cork) at z=1.053, neck band z=1.01-1.05 (visible width 0.014); grip 0.025 below the top sits under the collar", "allowed": True},
    "TILT_DEG": {"source": "pack keyframes: R_place @ R_grasp^T on the vertical bottle axis gives 33-58 deg from vertical leaning -y (demos 0/1/2); 58 deg = demo1", "allowed": True},
    "RACK_WOOD": {"source": "debug seeds: rack slats are wood-coloured (r>g+8>b+12, r in 80..240) and lie above table+0.20", "allowed": True},
    "CARRY_Z": {"source": "debug seeds: highest rack point deprojects to z=1.213; carry so the hanging bottle base clears it", "allowed": True},
    "LAY_CLEAR": {"source": "pack keyframes: demo release eef sits 0.025-0.04 above the fitted rack plane; 0.035 chosen for the bottle centre", "allowed": True},
    "HOLD_WIDTH": {"source": "v2 debug receipts: an empty close reads width 0.001, a neck hold reads 0.008", "allowed": True},
    "AIM_TOL": {"source": "v2 debug receipts: the residual x-bias was 0.025 m; iterate until under 0.004 m", "allowed": True},
    "PARK": {"source": "debug seeds: a pose at z=1.30 puts the whole hand above the table+0.21 perception band and off the -y rack side", "allowed": True},
    "MIN_TOP": {"source": "debug seeds: the standing bottle's cork tops out at table+0.151; the next-tallest dark object (stove knob) at table+0.059", "allowed": True},
    "LYING_LEN": {"source": "debug seed 64 cam_high dump: the bottle settles on its side; its dark cluster is 0.147 long, 0.040 wide, and tops out at table+0.042", "allowed": True},
    "LYING_GRIP_DROP": {"source": "debug seed 64: the lying bottle's top surface is table+0.042 and its axis sits one radius (0.020) below that; close just under the top", "allowed": True},
    "LYING_REST": {"source": "debug seeds: the bottle body is 0.040 across, so a body grasp puts its underside 0.020 below the eef", "allowed": True},
    "TRANSIT_TOL": {"source": "v7 debug receipt ep64: a starved transit left the eef 0.30 m short, so re-command until within 0.030 of the carry pose", "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics + debug seeds: api.tool_rotation() at reset is within 3 deg of diag(1,-1,-1), the straight-down tool frame", "allowed": True},
    "LYING_LIFT": {"source": "v7 debug receipt ep64: a 0.30 m diagonal transit starves; lift clear of the rack first so the transit is a short horizontal one", "allowed": True},
    "LYING_YAW_ONLY": {"source": "v8 debug receipt ep64: the full lying-to-slope rotation lifted the eef 0.12 m and jammed the wrist at z=1.474; a yaw about the approach axis does not", "allowed": True},
    "ROT_STEPS": {"source": "v6/v9 debug receipts ep64: one move_pose through the whole rotation starves; 23 deg sub-steps converge to within 1 mm", "allowed": True},
    "LYING_DOWN_STEP": {"source": "v9 debug receipt ep64: a 0.11 m descent in 0.030 steps cost six moves and exhausted the 1000-step horizon", "allowed": True},
    "LYING_HOLD": {"source": "debug seeds: the bottle body is 0.040 across, so a body close reads width 0.035-0.045 while an empty close reads 0.001", "allowed": True},
}

TABLE_Z = 0.901
DARK_GREY = 60.0
BOTTLE_FOOT_MAX = 0.09
NECK_DROP = 0.025
TILT_DEG = 58.0
CARRY_Z = 1.34
LAY_CLEAR = 0.035
HOLD_WIDTH = 0.003
AIM_TOL = 0.004
PARK = (-0.05, 0.33, 1.30)
MIN_TOP = 0.11
LYING_LEN = (0.085, 0.230)
LYING_HOLD = 0.010
LYING_GRIP_DROP = 0.012
ROT_STEPS = 3
LYING_LIFT = 0.32
LYING_REST = 0.020
LYING_DOWN_STEP = 0.055
TRANSIT_TOL = 0.030

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def _rx(deg):
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _cloud(f):
    d = np.asarray(f.depth, float)
    K, T = np.asarray(f.intrinsics, float), np.asarray(f.t_base_cam, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1) @ T.T
    return P[..., :3], np.asarray(f.rgb), d


def _cluster(xs, ys, cell):
    cells = {}
    for i in range(len(xs)):
        cells.setdefault((int(round(xs[i] / cell)), int(round(ys[i] / cell))), []).append(i)
    seen, groups = set(), []
    for c in list(cells):
        if c in seen:
            continue
        stack, comp = [c], []
        seen.add(c)
        while stack:
            k = stack.pop()
            comp += cells[k]
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    n = (k[0] + da, k[1] + db)
                    if n in cells and n not in seen:
                        seen.add(n)
                        stack.append(n)
        groups.append(comp)
    return groups


def find_bottle(P, rgb, dv, eef):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    grey = rgb.astype(np.float64).mean(-1)
    # the hand is parked above this band, so no eef mask is needed -- v3 masked
    # a 0.26 m box here and on ep64 that box swallowed the bottle.
    m = ((dv > 0) & (grey < DARK_GREY) & (Z > TABLE_Z + 0.035) & (Z < TABLE_Z + 0.21)
         & (X > -0.42) & (X < 0.28) & (Y > -0.45) & (Y < 0.45))
    xs, ys, zs = X[m], Y[m], Z[m]
    if len(xs) < 20:
        return None
    best, bestz = None, -1e9
    for gp in _cluster(xs, ys, 0.035):
        if len(gp) < 25:
            continue
        gz, gx, gy = zs[gp], xs[gp], ys[gp]
        if (gx.max() - gx.min()) > BOTTLE_FOOT_MAX or (gy.max() - gy.min()) > BOTTLE_FOOT_MAX:
            continue
        if gz.min() > TABLE_Z + 0.07:
            continue
        if gz.max() < TABLE_Z + MIN_TOP:
            continue                      # too short to be the standing bottle
        if gz.max() > bestz:
            bestz, best = gz.max(), gp
    if best is None:
        return None
    gx, gy, gz = xs[best], ys[best], zs[best]
    top = float(gz.max())
    # aim at the NECK: use the band just under the collar, where the silhouette
    # is the neck itself, so the centre estimate is not dragged by the body.
    band = (gz > top - 0.045) & (gz < top - 0.010)
    if band.sum() < 8:
        band = gz > top - 0.06
    ny = 0.5 * (gy[band].min() + gy[band].max())
    nr = 0.5 * (gy[band].max() - gy[band].min())
    nx = gx[band].max() - nr            # cam_high sees only the +x face
    bband = (gz > TABLE_Z + 0.05) & (gz < TABLE_Z + 0.09)
    brad = 0.5 * (gy[bband].max() - gy[bband].min()) if bband.sum() > 8 else nr
    return {"x": float(nx), "y": float(ny), "top": top, "neck_r": float(nr),
            "body_r": float(brad), "n": int(len(best))}


def find_lying_bottle(P, rgb, dv):
    """Seed 64 settles the bottle onto its side: a long, low, thin dark blob."""
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    grey = rgb.astype(np.float64).mean(-1)
    m = ((dv > 0) & (grey < DARK_GREY) & (Z > TABLE_Z + 0.012) & (Z < TABLE_Z + 0.085)
         & (X > -0.42) & (X < 0.28) & (Y > -0.45) & (Y < 0.45))
    xs, ys, zs = X[m], Y[m], Z[m]
    if len(xs) < 60:
        return None
    best = None
    for gp in _cluster(xs, ys, 0.030):
        if len(gp) < 60:
            continue
        gx, gy, gz = xs[gp], ys[gp], zs[gp]
        pts = np.stack([gx - gx.mean(), gy - gy.mean()], 1)
        w, V = np.linalg.eigh(pts.T @ pts / len(pts))
        u = V[:, 1]
        proj = pts @ u
        perp = pts @ V[:, 0]
        major = float(proj.max() - proj.min())
        minor = float(perp.max() - perp.min())
        if not (LYING_LEN[0] < major < LYING_LEN[1]):
            continue
        if not (0.018 < minor < 0.075):
            continue                      # the body is 0.040 across; thinner
            # streaks are rack shadow lines, wider blobs are other props
        if gz.max() > TABLE_Z + 0.075:
            continue
        if major / max(minor, 1e-6) < 2.0:
            continue                      # a bottle on its side is elongated
        cand = {"x": float(gx.mean()), "y": float(gy.mean()), "top": float(gz.max()),
                "ux": float(u[0]), "uy": float(u[1]), "major": major, "minor": minor,
                "ratio": major / minor, "n": int(len(gp))}
        if best is None or cand["ratio"] > best["ratio"]:
            best = cand
    return best


def _rot_between(a, b):
    """Rotation matrix taking unit vector a onto unit vector b."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    b = np.asarray(b, float) / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else -np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K / (1.0 + c)


def find_rack(P, rgb, dv):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    r = rgb[..., 0].astype(np.float64)
    g = rgb[..., 1].astype(np.float64)
    b = rgb[..., 2].astype(np.float64)
    wood = (r > g + 8) & (g > b + 4) & (r > 80) & (r < 240)
    m = ((dv > 0) & wood & (Y < -0.10) & (Y > -0.45) & (X > -0.30) & (X < 0.30)
         & (Z > TABLE_Z + 0.20))
    xs, ys, zs = X[m], Y[m], Z[m]
    if len(xs) < 200:
        return None
    coef = None
    for _ in range(2):
        A = np.stack([np.ones_like(xs), xs, ys], 1)
        coef = np.linalg.lstsq(A, zs, rcond=None)[0]
        keep = np.abs(zs - A @ coef) < 0.012
        if keep.sum() < 100:
            break
        xs, ys, zs = xs[keep], ys[keep], zs[keep]
    return {"c": [float(v) for v in coef], "zmax": float(zs.max()),
            "xlo": float(np.percentile(xs, 2)), "xhi": float(np.percentile(xs, 98)),
            "ylo": float(np.percentile(ys, 2)), "yhi": float(np.percentile(ys, 98)),
            "n": int(len(xs))}


def aim(api, tgt, rot, tol=AIM_TOL, tries=3, seconds=2.0):
    """Command a pose, then re-command from the measured error (the arm has a
    repeatable tracking bias near its reach boundary)."""
    cmd = np.array(tgt, float)
    e = None
    for i in range(tries):
        api.move(cmd, rotation=rot, seconds=seconds)
        e = np.asarray(api.eef(), float)
        err = np.array(tgt, float) - e
        api.log("  aim%d cmd=%s eef=%s err=%s" % (i, np.round(cmd, 4).tolist(),
                                                  np.round(e, 4).tolist(),
                                                  np.round(err, 4).tolist()))
        if np.linalg.norm(err) < tol:
            break
        cmd = cmd + err
    return e


def run(api):
    api.log("v12 start eef=%s" % np.round(api.eef(), 4).tolist())
    api.move(list(PARK), rotation=R_DOWN, seconds=2.2)
    api.log("parked eef=%s" % np.round(api.eef(), 4).tolist())
    f = api.capture("cam_high")
    P, rgb, dv = _cloud(f)
    rk = find_rack(P, rgb, dv)
    api.log("rack=%s" % rk)
    if rk is None:
        return "no rack"

    held = False
    grip_z = None
    top = None
    lying = None
    R_grasp = R_DOWN
    for attempt in range(3):
        if attempt:
            api.move(list(PARK), rotation=R_DOWN, seconds=2.2)
            f = api.capture("cam_high")
            P, rgb, dv = _cloud(f)
        bt = find_bottle(P, rgb, dv, api.eef())
        api.log("attempt%d bottle=%s" % (attempt, bt))
        if bt is None:
            ly = find_lying_bottle(P, rgb, dv)
            api.log("attempt%d lying=%s" % (attempt, ly))
            if ly is None:
                break
            # close ACROSS the cylinder: with R_DOWN the jaws travel along base
            # y, so yaw the wrist until that travel is perpendicular to the
            # bottle's own axis.
            psi = np.arctan2(-ly["uy"], -ly["ux"])
            cs, sn = float(np.cos(psi)), float(np.sin(psi))
            R_grasp = np.array([[cs, -sn, 0.0], [sn, cs, 0.0], [0.0, 0.0, 1.0]]) @ R_DOWN
            gz = ly["top"] - (LYING_GRIP_DROP + 0.010 * attempt)
            api.grip(0.08)
            api.move([ly["x"], ly["y"], gz + 0.16], rotation=R_grasp, seconds=3.0)
            aim(api, [ly["x"], ly["y"], gz], R_grasp, tol=0.012, tries=2, seconds=2.0)
            api.grip(0.0)
            api.settle(0.4)
            g = api.gripper()
            e = np.asarray(api.eef(), float)
            api.log("attempt%d lying-closed grip=%s eef=%s" % (attempt, g, np.round(e, 4).tolist()))
            if g["width_m"] > LYING_HOLD:
                grip_z = float(e[2])
                lying = ly
                held = True
                break
            api.grip(0.08)
            api.move([ly["x"], ly["y"], gz + 0.16], rotation=R_grasp, seconds=2.5)
            continue
        top = bt["top"]
        gz = top - NECK_DROP
        api.grip(0.08)
        api.move([bt["x"], bt["y"], gz + 0.14], rotation=R_DOWN, seconds=3.0)
        aim(api, [bt["x"], bt["y"], gz], R_DOWN, seconds=2.0)
        api.grip(0.0)
        api.settle(0.4)
        g = api.gripper()
        e = np.asarray(api.eef(), float)
        api.log("attempt%d closed grip=%s eef=%s" % (attempt, g, np.round(e, 4).tolist()))
        if g["width_m"] > HOLD_WIDTH:
            grip_z = float(e[2])
            held = True
            break
        api.grip(0.08)
        api.move([bt["x"], bt["y"], gz + 0.16], rotation=R_DOWN, seconds=2.5)

    if not held:
        api.log("no hold after retries")
        return "grasp failed"

    lift_h = 0.12 if lying is None else LYING_LIFT
    api.move([api.eef()[0], api.eef()[1], grip_z + lift_h], rotation=R_grasp, seconds=2.5)
    api.log("lifted grip=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))

    th = np.radians(TILT_DEG)
    sth, cth = float(np.sin(th)), float(np.cos(th))
    rxc = 0.5 * (rk["xlo"] + rk["xhi"])
    ryc = 0.5 * (rk["ylo"] + rk["yhi"])
    c = rk["c"]
    cz = c[0] + c[1] * rxc + c[2] * ryc + LAY_CLEAR
    axis = np.array([0.0, -sth, cth])      # the rack's own slope direction
    if lying is None:
        hg = grip_z - TABLE_Z              # grip height above the bottle's base
        L = top - TABLE_Z                  # bottle length
        dmid = hg - 0.5 * L                # grip point -> bottle centre, along the axis
        R_PL = _rx(TILT_DEG) @ R_DOWN
    else:
        dmid = 0.0                         # gripped at the lying bottle's centroid
        # YAW ONLY: line the horizontal bottle up with the rack's slope line.
        # The full lying->slope rotation needs a 32 deg pitch as well and the
        # wrist cannot hold position through it (v8 receipt).
        u2 = np.array([lying["ux"], lying["uy"]], float)
        u2 /= max(np.linalg.norm(u2), 1e-9)
        tgt = np.array([0.0, -1.0])
        if float(np.dot(u2, tgt)) < 0.0:
            tgt = -tgt                     # take the shorter way round
        dpsi = float(np.arctan2(u2[0] * tgt[1] - u2[1] * tgt[0], float(np.dot(u2, tgt))))
        cs, sn = float(np.cos(-dpsi)), float(np.sin(-dpsi))
        R_PL = np.array([[cs, -sn, 0.0], [sn, cs, 0.0], [0.0, 0.0, 1.0]]) @ R_grasp
        api.log("lying yaw=%.1f deg" % np.degrees(dpsi))
    ex, ey, ez = rxc, ryc - dmid * sth, cz + dmid * cth
    if lying is not None:
        ez = cz + LYING_REST          # the body hangs at the eef, not below it
    api.log("geom lying=%s dmid=%.4f centre=(%.4f,%.4f,%.4f) release=(%.4f,%.4f,%.4f)"
            % (lying is not None, dmid, rxc, ryc, cz, ex, ey, ez))

    if lying is None:
        api.move([ex, api.eef()[1], CARRY_Z], rotation=R_grasp, seconds=3.0)
        api.move([ex, ey, CARRY_Z], rotation=R_grasp, seconds=3.5)
    else:
        for _ in range(3):
            api.move([ex, ey, CARRY_Z], rotation=R_grasp, seconds=3.5)
            e = np.asarray(api.eef(), float)
            api.log("  transit eef=%s" % np.round(e, 4).tolist())
            if np.linalg.norm(e - np.array([ex, ey, CARRY_Z])) < TRANSIT_TOL:
                break
    api.log("carried grip=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))
    if lying is None:
        api.move([ex, ey, CARRY_Z], rotation=R_PL, seconds=3.5)
    else:
        # walk the rotation: one move_pose through the whole lying-to-slope
        # rotation starves and abandons the position target with it (ep64).
        Rrel = R_PL @ R_grasp.T
        ang = float(np.arccos(np.clip((np.trace(Rrel) - 1.0) / 2.0, -1.0, 1.0)))
        if ang > 1e-6:
            w = np.array([Rrel[2, 1] - Rrel[1, 2], Rrel[0, 2] - Rrel[2, 0],
                          Rrel[1, 0] - Rrel[0, 1]]) / (2.0 * np.sin(ang))
        else:
            w = np.array([0.0, 0.0, 1.0])
        for k in range(1, ROT_STEPS + 1):
            a = ang * k / ROT_STEPS
            K = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])
            Rk = (np.eye(3) + np.sin(a) * K + (1.0 - np.cos(a)) * K @ K) @ R_grasp
            api.move([ex, ey, CARRY_Z], rotation=Rk, seconds=2.2)
            api.log("  rot%d eef=%s" % (k, np.round(api.eef(), 4).tolist()))
    api.log("tilted grip=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))

    # stepwise descent: move_pose starves on a long combined pos+rot command
    prev = np.asarray(api.eef(), float)
    for k in range(6):
        zt = max(ez, prev[2] - (0.030 if lying is None else LYING_DOWN_STEP))
        api.move([ex, ey, zt], rotation=R_PL, seconds=2.0)
        now = np.asarray(api.eef(), float)
        api.log("  down%d want=%.4f eef=%s grip=%s" % (k, zt, np.round(now, 4).tolist(), api.gripper()))
        if now[2] <= ez + 0.006:
            prev = now
            break
        if abs(now[2] - prev[2]) < 0.004 and now[2] < ez + 0.035:
            prev = now
            break            # genuine contact/limit; every scoring seed
            # stopped between ez+0.012 and ez+0.028
        prev = now

    api.grip(0.08)
    api.settle(0.6)
    api.log("released eef=%s" % np.round(api.eef(), 4).tolist())
    api.move([ex, ey, CARRY_Z], rotation=R_PL, seconds=1.2)
    api.settle(1.0)
    f2 = api.capture("cam_high")
    P2, rgb2, dv2 = _cloud(f2)
    api.log("post bottle=%s" % find_bottle(P2, rgb2, dv2, api.eef()))
    return "v12 done"
