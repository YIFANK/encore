"""c2k1clean / goal_put_cream_cheese_in_bowl_pos_k1

Intent: "put the cream cheese in the bowl".

Mechanism (from the K=1 pack): open -> descend onto the cream cheese at
eef z 0.9104 -> close -> lift -> traverse -> descend to eef z 0.9669 over the
bowl -> open. The pack's *xy* is a decoy (this is a _pos cell: in every debug
seed the props sit elsewhere than in the demo), so both xy's are perceived
from cam_high RGB-D each episode; only the pack's *heights* transfer.

Perception (all from a single cam_high capture, deprojected to base frame and
rasterised into a 5 mm top-down max-z map):
  target = the low prop (top 10-35 mm above the table) whose mean colour is
           blue-dominant.  Measured on debug seeds 51-65: the cream cheese
           carton scores B-R = +22..+25, every other prop <= +6.
  goal   = the union of the bright cells whose top lies 38-100 mm above the
           table.  That band holds only the bowl rim (top 52 mm) and two dark
           props (the pot on the stove, the bottle neck), which the brightness
           test rejects.  The rim can be split by the arm's occlusion, so the
           pieces are merged before the centre is taken.

Descent envelope (measured on debug seeds 55 and 57 with a full retreat to the
home pose between probes): with the wrist straight down the arm cannot reach
grasp height at negative eef y - it stalls dead at z 1.065, 1.095 or 1.106
depending how far out it is, and the boundary sits near y = -0.033 (seed 55)
to y = -0.045 (seed 57).  The carton lands inside that strip on ~2 seeds in 15.
Yawing the wrist +20 deg about base z clears it on every seed tried, and still
closes across the carton's 44 mm side (the jaw span needed is
80*sin20 + 44*cos20 = 69 mm, inside the 78 mm opening; the measured closed
width is 0.042 either way).  So +20 deg is the primary wrist and the program
falls back through a ladder of (yaw, x-offset) rungs, using its own eef reading
to tell a stalled descent from a landed one.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {
        "source": "debug seeds 51-65: mode of the deprojected cam_high depth "
                  "histogram sits at 0.900 m (117k of 262k pixels in one 10 mm bin)",
        "allowed": True},
    "GRASP_Z": {
        "source": "pack.json demos[0].keyframes t=40 (the close keyframe) ee[2] "
                  "= 0.9104; cross-checked against my own debug-seed measurement "
                  "of the carton top (0.9196) minus half its 20 mm height",
        "allowed": True},
    "RELEASE_Z_OVER_RIM": {
        "source": "pack.json demos[0].ee_path6[8] (the open command) z = 0.9669 "
                  "minus the bowl-rim top I measure on debug seeds (0.952) = 0.015",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug seeds 51-65: the tallest obstacle between the carton and "
                  "the bowl is the bottle at z 1.059; 1.12 clears it by 60 mm and "
                  "is below the home eef height 1.173",
        "allowed": True},
    "HOVER_Z": {
        "source": "debug-seed measurement: 1.02 is above every prop top in the "
                  "carton's neighbourhood (plate 0.920, carton 0.920)",
        "allowed": True},
    "TARGET_BAND": {
        "source": "debug seeds 51-65: the carton top is 0.9196, i.e. 19.6 mm above "
                  "the table; the band (10, 35) mm separates it from the table and "
                  "from the bowl rim (52 mm)",
        "allowed": True},
    "BLUE_MARGIN": {
        "source": "debug seeds 51-65: carton mean B-R = +21.7..+24.6; the next "
                  "bluest low prop (the plate) is -12; 6.0 is the midpoint-ish cut",
        "allowed": True},
    "GOAL_BAND": {
        "source": "debug seeds 51-65: bowl rim top 0.9515-0.9523 (52 mm above the "
                  "table); the plate/carton top out at 20 mm and the bottle/cabinet "
                  "start above 100 mm",
        "allowed": True},
    "GOAL_BRIGHT": {
        "source": "debug seeds 51-65: bowl-rim cells mean RGB 106-114; the two other "
                  "props in the same height band (stove pot 25-31, bottle neck 11-30) "
                  "are far darker",
        "allowed": True},
    "GOAL_MERGE_R": {
        "source": "debug seed 58: the arm splits the rim into two bright pieces "
                  "11 cm apart; 0.14 m merges them and is smaller than the 0.19 m "
                  "gap to any other bright band cell",
        "allowed": True},
    "GRID": {
        "source": "generic camera mechanics: 5 mm cells over the table region that "
                  "cam_high sees (x -0.40..0.35, y -0.45..0.45 in base frame)",
        "allowed": True},
    "R_DOWN": {
        "source": "generic controller mechanics: tool z along -base z (straight "
                  "down), matching the home tool_rotation read on debug seed 51 "
                  "([[0.998,0,-0.057],[0,-1,0],[-0.057,0,-0.998]]); tool y then "
                  "lies along base y, so the jaws close across the carton's 40 mm "
                  "side rather than its 80 mm side",
        "allowed": True},
    "OPEN_W": {
        "source": "debug seed 51 api.gripper() at reset reports width_m 0.0778 "
                  "with the jaws open; 0.08 commands full open",
        "allowed": True},
    "GOTO_TOL": {
        "source": "v1 debug run: converged moves land 6-12 mm from the command "
                  "and grasp fine; the two failures ended 46-58 mm out, so 0.012 "
                  "separates a healthy move from a stalled one",
        "allowed": True},
    "GOTO_TRIES/STEP_CAP": {
        "source": "generic controller mechanics: bias-cancelling retries must be "
                  "bounded or the command runs away; 40 mm is the largest single "
                  "correction v1 ever needed",
        "allowed": True},
    "FREE_LIFT": {
        "source": "v1 debug seeds 55/63: a stalled move left the eef 50 mm past "
                  "the command with the arm extended; 0.10 m of straight-up "
                  "retreat is within the 1.173 m home height",
        "allowed": True},
    "HELD_W": {
        "source": "v1 debug run: a successful close reports width_m 0.042 (the "
                  "carton's 40 mm side) with effort 3.0; a missed close reports "
                  "0.001 with effort 0.05",
        "allowed": True},
    "RUNGS": {
        "source": "probe_v8 on debug seeds 55/56/57/63 (a clean retreat to the "
                  "home pose before each rung): yaw +20 deg reached grasp height "
                  "and lifted on 4/4; yaw +20 with a +30 mm x offset on 3/4; yaw "
                  "0 on 2/4 (it is the one that stalls on 55/63); yaw -20 and yaw "
                  "180 on 2/4",
        "allowed": True},
    "STALL_MARGIN": {
        "source": "probe_v7/v8 on debug seeds 55/57: a landed descent reads eef z "
                  "0.918-0.922 against a 0.9104 command; a stalled one reads "
                  "1.064+, so 0.020 separates them with a 140 mm gap to spare",
        "allowed": True},
    "RELEASE_YAWS": {
        "source": "probe_v9 on debug seeds 51/57/62/63/65 (empty gripper, clean "
                  "retreat between rungs): descending to release height over the "
                  "bowl stalls at 0.994 for yaw 0/+-20/180 whenever the bowl "
                  "centre is below y -0.055; yaw +90 landed on 5/5 and yaw +45 "
                  "on 4/5, so +90 leads",
        "allowed": True},
    "HOME": {
        "source": "debug seeds 51-65: api.eef() at reset is (-0.2085, 0, 1.1733) "
                  "in every one; probe_v8 used it as the between-rung retreat and "
                  "each rung then behaved as if run first",
        "allowed": True},
}

TABLE_Z = 0.900
CELL = 0.005
XLO, XHI = -0.40, 0.35
YLO, YHI = -0.45, 0.45
ZMAX = 1.10

TARGET_BAND = (0.010, 0.035)
BLUE_MARGIN = 6.0
GOAL_BAND = (0.038, 0.100)
GOAL_BRIGHT = 55.0
GOAL_MERGE_R = 0.14

GRASP_Z = 0.9104
RELEASE_Z_OVER_RIM = 0.015
CARRY_Z = 1.12
HOVER_Z = 0.96
OPEN_W = 0.08
GOTO_TOL = 0.012
GOTO_TRIES = 4
STEP_CAP = 0.04
FREE_LIFT = 0.10
HELD_W = (0.025, 0.060)
STALL_MARGIN = 0.020
HOME = np.array([-0.2085, 0.0, 1.1733])

# (wrist yaw about base z in degrees, grasp offset along base x)
RUNGS = [(20.0, 0.000), (20.0, 0.030), (0.0, 0.000), (-20.0, 0.000)]
# the same envelope bites over the bowl; these yaws are tried for the release
RELEASE_YAWS = [90.0, 45.0, 20.0, 0.0]


def R_of(deg):
    """Straight-down wrist yawed `deg` about base z.  Tool z is (0,0,-1); the
    jaw axis (tool y) is (sin, -cos, 0), so yaw 0 and 180 close exactly across
    base y and +-20 deg closes almost across it."""
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])


R_DOWN = R_of(0.0)


# ---------------------------------------------------------------- perception

def _grid(frame):
    """cam_high RGB-D -> (max-z map, per-cell colour) on a 5 mm base-frame raster."""
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    rgb = np.asarray(frame.rgb)
    d = np.asarray(frame.depth, float)
    H, W = d.shape[:2]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    z = np.where(np.isfinite(d) & (d > 0), d, 0.0)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z, np.ones_like(z)], -1)
    pw = pc @ T.T
    X, Y, Z = pw[..., 0], pw[..., 1], pw[..., 2]

    ok = ((z > 0) & np.isfinite(Z) & (X > XLO) & (X < XHI)
          & (Y > YLO) & (Y < YHI) & (Z > TABLE_Z - 0.02) & (Z < ZMAX))
    nx = int(round((XHI - XLO) / CELL))
    ny = int(round((YHI - YLO) / CELL))
    hm = np.full((nx, ny), TABLE_Z)
    col = np.zeros((nx, ny, 3))
    ix = np.clip(((X - XLO) / CELL).astype(int), 0, nx - 1)
    iy = np.clip(((Y - YLO) / CELL).astype(int), 0, ny - 1)
    xs, ys, zs, cs = ix[ok], iy[ok], Z[ok], rgb[ok].astype(float)
    o = np.argsort(zs)                      # highest point per cell wins
    hm[xs[o], ys[o]] = zs[o]
    col[xs[o], ys[o]] = cs[o]
    return hm, col


def _cc(mask):
    """8-connected labelling (no scipy in the sandbox)."""
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    nx, ny = mask.shape
    for i in range(nx):
        for j in range(ny):
            if mask[i, j] and lab[i, j] == 0:
                cur += 1
                lab[i, j] = cur
                st = [(i, j)]
                while st:
                    a, b = st.pop()
                    for p in (a - 1, a, a + 1):
                        for q in (b - 1, b, b + 1):
                            if 0 <= p < nx and 0 <= q < ny and mask[p, q] and lab[p, q] == 0:
                                lab[p, q] = cur
                                st.append((p, q))
    return lab, cur


def _centres(shape):
    nx, ny = shape
    ii, jj = np.meshgrid(np.arange(nx), np.arange(ny), indexing="ij")
    return XLO + (ii + 0.5) * CELL, YLO + (jj + 0.5) * CELL


def find_target(hm, col, Xc, Yc, log):
    """Lowest-lying prop whose colour is blue-dominant = the cream cheese."""
    mask = (hm > TABLE_Z + TARGET_BAND[0]) & (hm < TABLE_Z + TARGET_BAND[1])
    lab, n = _cc(mask)
    best, score = None, -1e9
    for k in range(1, n + 1):
        sel = lab == k
        npx = int(sel.sum())
        if npx < 20 or npx > 600:
            continue
        c = col[sel]
        c = c[c.sum(1) > 0]
        if len(c) == 0:
            continue
        blue = float(c[:, 2].mean() - c[:, 0].mean())
        log("  tgt cand n=%d blue=%.1f x=%.3f y=%.3f ztop=%.3f"
            % (npx, blue, float(Xc[sel].mean()), float(Yc[sel].mean()), float(hm[sel].max())))
        if blue > score:
            score, best = blue, sel
    if best is None or score < BLUE_MARGIN:
        return None
    ztop = float(hm[best].max())
    top = best & (hm > ztop - 0.006)          # the carton's upper face only
    xs, ys = Xc[top], Yc[top]
    return dict(x=float((xs.min() + xs.max()) / 2), y=float((ys.min() + ys.max()) / 2),
                dx=float(xs.max() - xs.min()), dy=float(ys.max() - ys.min()),
                ztop=ztop, blue=score, n=int(best.sum()))


def find_goal(hm, col, Xc, Yc, log):
    """Bright cells in the rim height band, merged across the arm's occlusion."""
    mask = (hm > TABLE_Z + GOAL_BAND[0]) & (hm < TABLE_Z + GOAL_BAND[1])
    lab, n = _cc(mask)
    pieces = []
    for k in range(1, n + 1):
        sel = lab == k
        npx = int(sel.sum())
        if npx < 8:
            continue
        c = col[sel]
        c = c[c.sum(1) > 0]
        if len(c) == 0:
            continue
        bright = float(c.mean())
        cx, cy = float(Xc[sel].mean()), float(Yc[sel].mean())
        log("  goal cand n=%d bright=%.0f x=%.3f y=%.3f ztop=%.3f"
            % (npx, bright, cx, cy, float(hm[sel].max())))
        if bright > GOAL_BRIGHT:
            pieces.append(dict(sel=sel, n=npx, cx=cx, cy=cy))
    if not pieces:
        return None
    seed = max(pieces, key=lambda p: p["n"])
    keep = [p for p in pieces
            if np.hypot(p["cx"] - seed["cx"], p["cy"] - seed["cy"]) < GOAL_MERGE_R]
    sel = np.zeros(hm.shape, bool)
    for p in keep:
        sel |= p["sel"]
    xs, ys = Xc[sel], Yc[sel]
    return dict(x=float((xs.min() + xs.max()) / 2), y=float((ys.min() + ys.max()) / 2),
                dx=float(xs.max() - xs.min()), dy=float(ys.max() - ys.min()),
                ztop=float(hm[sel].max()), n=int(sel.sum()), pieces=len(keep))


# ------------------------------------------------------------------- motion

def goto(api, xyz, log, seconds=3.0, tol=GOTO_TOL, tries=GOTO_TRIES, tag="",
         R=None):
    """One commanded pose, then bias-cancelling retries until the eef is there.

    v1's two debug failures were single moves that ended 50 mm past the command
    with the arm extended; re-issuing the same command does not help, so each
    retry shifts the command by the observed tracking error (bounded), and a
    retry that fails to shrink the error first retreats straight up.
    """
    R = R_DOWN if R is None else R
    goal = np.asarray(xyz, float)
    cmd = goal.copy()
    prev = None
    err = 1e9
    for i in range(tries):
        api.move(cmd, R, seconds)
        e = np.asarray(api.eef(), float)
        d = goal - e
        err = float(np.linalg.norm(d))
        log("  goto%s try%d cmd=%s eef=%s err=%.4f"
            % (tag, i, np.round(cmd, 4).tolist(), np.round(e, 4).tolist(), err))
        if err <= tol:
            return err
        if prev is not None and err > prev - 0.003:
            # not converging: unload the arm before trying again
            api.move([e[0], e[1], min(e[2] + FREE_LIFT, 1.16)], R, 2.0)
            log("  goto%s freed to %s" % (tag, np.round(api.eef(), 4).tolist()))
            cmd = goal.copy()
            prev = None
            continue
        prev = err
        cmd = cmd + np.clip(d, -STEP_CAP, STEP_CAP)
    return err


def held(api):
    g = api.gripper()
    return (g["effort"] >= 2.0 and HELD_W[0] <= g["width_m"] <= HELD_W[1]), g


def retreat(api, log, R):
    """Back to the reset pose, which probe_v8 showed clears a stalled arm."""
    e = np.asarray(api.eef(), float)
    api.move([e[0], e[1], 1.16], R, 2.0)
    api.move(HOME, R, 3.0)
    api.settle(0.3)
    log("  retreat eef=%s" % np.round(api.eef(), 4).tolist())


# ------------------------------------------------------------------- program

def run(api):
    log = api.log
    log("instruction=%r" % api.instruction())

    f = api.capture("cam_high")
    hm, col = _grid(f)
    Xc, Yc = _centres(hm.shape)
    tgt = find_target(hm, col, Xc, Yc, log)
    goal = find_goal(hm, col, Xc, Yc, log)
    log("TARGET %s" % tgt)
    log("GOAL   %s" % goal)
    if tgt is None or goal is None:
        return "perception failed tgt=%s goal=%s" % (tgt is None, goal is None)

    grasp_z = GRASP_Z
    release_z = goal["ztop"] + RELEASE_Z_OVER_RIM
    log("grasp_z=%.4f release_z=%.4f" % (grasp_z, release_z))

    # ---- pick: walk down the rung ladder until the carton is actually held
    ok, R, gx, gy = False, R_of(RUNGS[0][0]), tgt["x"], tgt["y"]
    for ri, (yaw, dx) in enumerate(RUNGS):
        R = R_of(yaw)
        gx, gy = tgt["x"] + dx, tgt["y"]
        if ri:
            retreat(api, log, R)
        api.grip(OPEN_W)
        goto(api, [gx, gy, CARRY_Z], log, 3.0, tag="/over%d" % ri, R=R)
        goto(api, [gx, gy, HOVER_Z], log, 2.5, tag="/hover%d" % ri, R=R)
        goto(api, [gx, gy, grasp_z], log, 2.0, tag="/down%d" % ri, R=R)
        ez = float(api.eef()[2])
        if ez > grasp_z + STALL_MARGIN:
            log("rung%d yaw=%+.0f dx=%+.3f STALLED at z=%.4f" % (ri, yaw, dx, ez))
            continue
        api.grip(0.0)
        api.settle(0.3)
        ok, g = held(api)
        log("rung%d yaw=%+.0f dx=%+.3f landed z=%.4f held=%s gripper=%s"
            % (ri, yaw, dx, ez, ok, g))
        if ok:
            break
        api.grip(OPEN_W)

    goto(api, [gx, gy, CARRY_Z], log, 2.5, tag="/lift", R=R)
    lifted, g = held(api)
    log("lifted held=%s gripper=%s" % (lifted, g))

    goto(api, [goal["x"], goal["y"], CARRY_Z], log, 3.0, tag="/cross", R=R)
    log("crossed held=%s gripper=%s" % (held(api)[0], api.gripper()))

    # ---- place: the same descent envelope stalls over the bowl whenever its
    # centre sits below y ~ -0.055, so try wrists until one lands.  Plain
    # moves, not goto: the bias-cancelling retry turns a 27 mm stall into a
    # 115 mm one by chasing a command the arm cannot follow.
    Rr = R
    for ri, ryaw in enumerate(RELEASE_YAWS):
        Rr = R_of(ryaw)
        api.move([goal["x"], goal["y"], CARRY_Z], Rr, 2.5)   # reorient up high
        api.move([goal["x"], goal["y"], release_z], Rr, 2.5)
        ez = float(api.eef()[2])
        h, g = held(api)
        log("place%d yaw=%+.0f z=%.4f err=%.4f held=%s gripper=%s"
            % (ri, ryaw, ez, ez - release_z, h, g))
        if ez <= release_z + STALL_MARGIN:
            break
    log("lowered eef=%s" % np.round(api.eef(), 4).tolist())

    api.grip(OPEN_W)
    api.settle(0.6)
    log("released gripper=%s" % api.gripper())

    api.move([goal["x"], goal["y"], CARRY_Z], Rr, 2.0)
    api.settle(0.5)
    return "done held=%s" % lifted
