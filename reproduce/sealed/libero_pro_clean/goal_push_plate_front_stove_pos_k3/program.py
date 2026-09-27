"""c2clean / goal_push_plate_front_stove_pos_k3  (v2: drag budget 2.0 s, 3 passes)

Mechanism read from the pack: the gripper never closes (gripper_cmd == -1.0 in
every keyframe of all three demos). The arm descends onto the plate at table
height (ee_path6 z settles at 0.917-0.920 and stays there) and DRAGS it
sideways, then lifts. So this is a press-and-drag, not a grasp.

This version perceives the plate and the stove from cam_high, computes the goal
as the stove centre plus the offset the demos' own final plate position shows,
drags, then re-perceives and corrects.
"""
import numpy as np

PROVENANCE = {
    "TABLE_BAND": {
        "source": "debug seeds 51-65 cam_high depth: modal z of the workspace "
                  "point cloud is 0.9010 m on every seed; used as a measured "
                  "per-episode value, not a literal",
        "allowed": True},
    "PLATE_ZTOP": {
        "source": "debug seeds 51-65: the disc component's 97th-pct height is "
                  "table+0.018 m on all 15; band (0.010,0.034) brackets it",
        "allowed": True},
    "PLATE_EXTENT": {
        "source": "debug seeds 51-65: disc extent 0.131-0.136 m in x and y "
                  "(aspect 0.97-1.02); band (0.09,0.19) + aspect (0.7,1.4)",
        "allowed": True},
    "STOVE_ZTOP": {
        "source": "debug seeds 51-65: slab component 97th-pct height "
                  "table+0.038..0.044 m; band (0.030,0.115)",
        "allowed": True},
    "STOVE_EXTENT": {
        "source": "debug seeds 51-65: slab extent 0.244-0.262 x 0.189-0.190 m; "
                  "thresholds ex>=0.15, ey>=0.13",
        "allowed": True},
    "GOAL_OFFSET": {
        "source": "pack keyframes demo0_t0154/demo1_t0127/demo2_t0124: the "
                  "plate's bright interior back-projected onto the plate-top "
                  "plane lands at base (-0.046,0.203) mean (gripper-occluded, "
                  "so biased a little -y); the same camera puts the measured "
                  "stove centre at (-0.260,0.206) on the debug seeds, giving "
                  "the goal as stove + (0.210,0.008)",
        "allowed": True},
    "PRESS_Z": {
        "source": "pack ee_path6: all three demos hold z = 0.917-0.920 while "
                  "dragging, i.e. table+0.016..0.019; PRESS_Z = table+0.016",
        "allowed": True},
    "DRAG_SLACK": {
        "source": "pack demo0: eef travels dy=+0.284 while the plate (keyframe "
                  "back-projection) travels dy=+0.224 -- 0.06 m of free travel "
                  "before the fingers reach the dish wall; overshoot by 0.05",
        "allowed": True},
    "HOVER_Z": {"source": "generic: clear of the tallest measured prop tops "
                          "(table+0.16 m) before translating", "allowed": True},
    "RETREAT_XYZ": {
        "source": "debug seeds: the arm's start pose (-0.208,0.0,1.173) leaves "
                  "cam_high's view of the table clear (v0 probe frames)",
        "allowed": True},
}

WS_X = (-0.42, 0.38)
WS_Y = (-0.52, 0.52)
GOAL_OFFSET = np.array([0.210, 0.008])
PRESS_DZ = 0.016
HOVER_DZ = 0.16
DRAG_SLACK = 0.05
TOL = 0.015
RETREAT = np.array([-0.208, 0.0, 1.173])


# ---------------------------------------------------------------- perception
def _deproject_grid(K, T, dep):
    H, W = dep.shape
    v, u = np.mgrid[0:H, 0:W]
    x = (u - K[0, 2]) / K[0, 0] * dep
    y = (v - K[1, 2]) / K[1, 1] * dep
    pts = np.stack([x, y, dep, np.ones_like(dep)], -1)
    return (pts @ T.T)[..., :3]


def _label(mask):
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    cur = 0
    for sy, sx in np.argwhere(mask):
        if lab[sy, sx]:
            continue
        cur += 1
        stack = [(int(sy), int(sx))]
        lab[sy, sx] = cur
        while stack:
            y, x = stack.pop()
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < H and 0 <= nx < W and mask[ny, nx] and not lab[ny, nx]:
                    lab[ny, nx] = cur
                    stack.append((ny, nx))
    return lab, cur


def _table_z(P):
    z = P[..., 2]
    m = ((P[..., 0] > WS_X[0]) & (P[..., 0] < WS_X[1]) &
         (P[..., 1] > WS_Y[0]) & (P[..., 1] < WS_Y[1]) & (z > 0.5) & (z < 1.4))
    zz = z[m]
    hist, edges = np.histogram(zz, bins=200, range=(0.5, 1.4))
    k = int(np.argmax(hist))
    sel = zz[(zz >= edges[k] - 0.01) & (zz <= edges[k + 1] + 0.01)]
    return float(np.median(sel))


def _components(P, tz):
    z = P[..., 2]
    m = ((P[..., 0] > WS_X[0]) & (P[..., 0] < WS_X[1]) &
         (P[..., 1] > WS_Y[0]) & (P[..., 1] < WS_Y[1]) &
         (z > tz + 0.008) & (z < tz + 0.35))
    lab, n = _label(m)
    out = []
    for i in range(1, n + 1):
        mm = lab == i
        cnt = int(mm.sum())
        if cnt < 120:
            continue
        X, Y, Z = P[..., 0][mm], P[..., 1][mm], z[mm]
        out.append(dict(n=cnt, cx=float(X.mean()), cy=float(Y.mean()),
                        ex=float(X.max() - X.min()), ey=float(Y.max() - Y.min()),
                        ztop=float(np.percentile(Z, 97)) - tz))
    return out


def _find_plate(comps):
    best, score = None, -1e9
    for c in comps:
        if not 0.010 <= c["ztop"] <= 0.034:
            continue
        if not (0.09 <= c["ex"] <= 0.19 and 0.09 <= c["ey"] <= 0.19):
            continue
        ar = c["ex"] / max(c["ey"], 1e-6)
        if not 0.7 <= ar <= 1.4:
            continue
        s = c["n"] - 400 * abs(np.log(ar))
        if s > score:
            best, score = c, s
    return best


def _find_stove(comps):
    best, score = None, -1e9
    for c in comps:
        if not 0.030 <= c["ztop"] <= 0.115:
            continue
        if c["ex"] < 0.15 or c["ey"] < 0.13:
            continue
        if c["n"] > score:
            best, score = c, c["n"]
    return best


def _look(api):
    f = api.capture("cam_high")
    P = _deproject_grid(np.asarray(f.intrinsics, float),
                        np.asarray(f.t_base_cam, float),
                        np.asarray(f.depth, float))
    tz = _table_z(P)
    comps = _components(P, tz)
    return tz, comps


# --------------------------------------------------------------------- policy
def run(api):
    api.log("instruction: %s" % api.instruction())
    tz, comps = _look(api)
    api.log("table_z=%.4f ncomp=%d" % (tz, len(comps)))
    for c in comps:
        api.log("  comp n=%d c=(%.3f,%.3f) ex=%.3f ey=%.3f ztop=%.3f"
                % (c["n"], c["cx"], c["cy"], c["ex"], c["ey"], c["ztop"]))
    plate = _find_plate(comps)
    stove = _find_stove(comps)
    if plate is None:
        api.log("ABORT: no plate component")
        return
    if stove is None:
        api.log("ABORT: no stove component")
        return
    goal = np.array([stove["cx"], stove["cy"]]) + GOAL_OFFSET
    api.log("plate=(%.3f,%.3f) stove=(%.3f,%.3f) goal=(%.3f,%.3f)"
            % (plate["cx"], plate["cy"], stove["cx"], stove["cy"], goal[0], goal[1]))

    press_z = tz + PRESS_DZ
    hover_z = tz + HOVER_DZ
    here = np.array([plate["cx"], plate["cy"]])

    for attempt in range(3):
        err = goal - here
        api.log("attempt %d: plate=(%.3f,%.3f) err=(%.3f,%.3f) |err|=%.3f"
                % (attempt, here[0], here[1], err[0], err[1], float(np.linalg.norm(err))))
        if float(np.linalg.norm(err)) < TOL:
            api.log("within tolerance; stop")
            break
        u = err / max(float(np.linalg.norm(err)), 1e-6)
        drag_to = here + err + u * DRAG_SLACK

        r = api.move([here[0], here[1], hover_z], seconds=1.0)
        api.log("hover residual %.4f eef %s" % (r, np.round(api.eef(), 4).tolist()))
        r = api.move([here[0], here[1], press_z], seconds=0.6)
        api.log("press residual %.4f eef %s grip %s"
                % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
        r = api.move([drag_to[0], drag_to[1], press_z], seconds=2.0)
        api.log("drag residual %.4f eef %s" % (r, np.round(api.eef(), 4).tolist()))
        r = api.move([drag_to[0], drag_to[1], hover_z], seconds=0.8)
        api.log("lift residual %.4f" % r)
        r = api.move(RETREAT, seconds=1.2)
        api.log("retreat residual %.4f eef %s" % (r, np.round(api.eef(), 4).tolist()))
        api.settle(0.4)

        tz2, comps2 = _look(api)
        p2 = _find_plate(comps2)
        if p2 is None:
            api.log("re-perception lost the plate; stop")
            break
        here = np.array([p2["cx"], p2["cy"]])
        api.log("after drag: plate=(%.3f,%.3f) resid=(%.3f,%.3f)"
                % (here[0], here[1], goal[0] - here[0], goal[1] - here[1]))
    api.log("final plate=(%.3f,%.3f) goal=(%.3f,%.3f)" % (here[0], here[1], goal[0], goal[1]))
