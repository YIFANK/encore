"""v12 -- v11, trimmed to fit the retry inside the 1000-step episode.

The decisive evidence is the k3 pack's raw actions. Through the whole pull
phase (demo0 t=34..55, where the eef y runs -0.045 -> +0.065 at a constant
z=1.105) the commanded action is

    dy ~ +0.70  (near saturation)      dz ~ -0.60 .. -0.25   (hard DOWN)

The eef z does not move, so that -dz is pure force: the demo is pressing the
open jaw onto the handle bar and dragging the drawer by friction. Every version
of mine so far commanded a *position* at the contact height, which converges
with ~zero contact force -- v2 scraped 0.090 m out of it by luck and v6/v7
reproduced the same eef trace for 0.000 m on four seeds.

So: seat the far jaw on the bar, then command each +y waypoint 0.06 BELOW the
seated height. The bar blocks the descent, the controller keeps pushing, and
the residual downward command becomes the normal force that makes the drag bite.

v9 receipt (debug seeds 51,53,55,57): drawer TRAVEL 0.167 / 0.241 / 0.160 /
0.250 m, and the eef tracked its own +y ladder to within 5 mm, i.e. the drawer
now follows the hand 1:1. But a move that cannot converge burns seconds x ~100
sim steps, and seven pressing hops ate the entire 1000-step episode before the
pick ever ran. v10 does the same travel in four longer hops (~0.041 each) and
trims the tail, which leaves ~200 steps of margin for the pick and release.

v10 receipt (debug seeds 51,53,55,57): 3/4, travel 0.159 on all four. The one
loss was seed 55, where the cream cheese sits at y=0.113 -- only ~0.08 in front
of the now-open drawer's front wall -- and the straight-down descent stalled at
z=0.989 instead of 0.912, so the jaws closed on air (width 0.001, effort 0.05)
and the program carried nothing to the drawer. v11 therefore CHECKS the grasp
(effort is 3.0 only when something is held) and, if it failed, re-approaches
from +y, further from the drawer, and descends again. It also drops v10's final
retreat move, which stalled on every seed and cost ~120 steps for nothing.

Approaches from below are not available: v4 (descend to z=1.052 in front of the
bar) and v8 (slide -y at z=1.045) both stalled with the eef refusing to follow,
so the arm cannot work under the 1.13 cabinet top. Everything happens from above.
"""
import numpy as np

PROVENANCE = {
    "PRESS_DEPTH": {
        "source": "k3 pack raw actions: through the pull the commanded dz is "
                  "-0.60..-0.25 (scale 0.448/0.509/0.475) while the eef z holds "
                  "constant -- a sustained downward press, not a position hold",
        "allowed": True},
    "DRAG_EEF_Y_OFFSET": {
        "source": "k3 pack ee_path6: the pull starts at eef y ~ -0.090 against "
                  "a bar whose outer surface debug seed 51 (v5/v8) measures at "
                  "y=-0.126, i.e. eef y = bar_y + 0.037",
        "allowed": True},
    "SEAT_Z": {
        "source": "k3 pack ee_path6 pull z 1.1045/1.1084/1.1047; on debug seeds "
                  "51-57 the eef seats on the bar at z=1.112",
        "allowed": True},
    "BAR_Z_BAND": {
        "source": "debug seed 51 (v8) (z,y) occupancy at the bar's x centre: "
                  "the top bar fills z in [1.08,1.095], y in [-0.152,-0.125]; "
                  "z in [1.03,1.075] there is empty",
        "allowed": True},
    "PULL_STEP": {
        "source": "debug seeds 51-57 (v9): under a standing press the drawer "
                  "follows the eef 1:1 (travel 0.167/0.241/0.160/0.250 against "
                  "0.164 of eef travel), so the hop size only has to stay small "
                  "enough to keep the press loaded",
        "allowed": True},
    "PULL_END_Y": {
        "source": "k3 pack ee_path6: the pull ends at y ~ +0.07",
        "allowed": True},
    "JAW_HALF_SPAN": {
        "source": "api.gripper() width 0.0796 when opened to 0.08; jaws straddle "
                  "the eef along base y (closing on the cream cheese, whose blue "
                  "cluster spans 0.040 in y, gives width 0.042)",
        "allowed": True},
    "INTERIOR_Z_BAND": {
        "source": "debug seed 51 (v2) POST heightmap: the opened drawer's "
                  "interior floor reads 1.06, its rim 1.12; the top bar starts "
                  "at 1.08 (v8)",
        "allowed": True},
    "DROP_Z": {
        "source": "k3 pack release z 1.135/1.130/1.166, above the 1.12 drawer "
                  "rim measured in v2",
        "allowed": True},
    "GRASP_Z": {
        "source": "mate pack keyframes eef z at gripper_cmd=+1 "
                  "(0.9104/0.9205/0.9106); closes to width 0.042 / effort 3.0 "
                  "on debug seeds 51-57",
        "allowed": True},
    "TABLE_Z": {
        "source": "debug seed 51 (v1) cam_high heightmap plateau at 0.900",
        "allowed": True},
    "BLUE_MASK": {
        "source": "k3/mate pack keyframe RGB: the cream cheese is the only "
                  "b>r+10, b>g+5 pixel group",
        "allowed": True},
    "RETRY_Y_BACKOFF": {
        "source": "debug seed 55 (v10): the cream cheese at y=0.113 sits ~0.08 "
                  "in front of the opened drawer's front wall and the descent "
                  "stalled 0.077 high; the retry stages 0.075 further out in +y",
        "allowed": True},
    "HOLDING_EFFORT": {
        "source": "api.gripper() documented/observed: effort 3.0 when holding "
                  "(debug seeds 51/53/57) vs 0.05 when the jaws closed on air "
                  "(debug seed 55)",
        "allowed": True},
    "CARRY_Z": {
        "source": "k3 pack carry apex 1.219/1.234/1.258, above the 1.13 cabinet "
                  "top measured in v1",
        "allowed": True},
}

TABLE_Z = 0.900
GRASP_Z = 0.912
CARRY_Z = 1.250
JAW_HALF_SPAN = 0.039
DRAG_EEF_Y_OFFSET = 0.037
SEAT_Z = 1.100
PRESS_DEPTH = 0.060
PULL_STEP = 0.042
PULL_STEPS = 4
PULL_END_Y = 0.080
DROP_Z = 1.155
PARK = (0.06, 0.26, 1.27)
HOLDING_EFFORT = 1.0
RETRY_Y_BACKOFF = 0.075


def base_cloud(f):
    d = np.asarray(f.depth, dtype=np.float64)
    H, W = d.shape
    K = np.asarray(f.intrinsics, dtype=np.float64)
    T = np.asarray(f.t_base_cam, dtype=np.float64)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u = np.arange(W)[None, :].repeat(H, 0).astype(np.float64)
    v = np.arange(H)[:, None].repeat(W, 1).astype(np.float64)
    pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
    return pc @ T[:3, :3].T + T[:3, 3][None, None, :]


def find_bar(api, P, tag):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    sel = (Z > 1.078) & (Z < 1.100) & (X > -0.06) & (X < 0.14) & (Y > -0.34) & (Y < 0.32)
    if sel.sum() < 60:
        api.log("%s bar: %d px" % (tag, int(sel.sum())))
        return None
    out = float(np.percentile(Y[sel], 99.5))
    hx = float(np.median(X[sel & (Y > out - 0.030)]))
    api.log("%s bar n=%d outer_y=%.3f x=%.3f" % (tag, int(sel.sum()), out, hx))
    return hx, out


def find_interior(api, P, tag, face_y):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    sel = (Z > 1.040) & (Z < 1.078) & (X > -0.10) & (X < 0.16) \
        & (Y > face_y + 0.008) & (Y < 0.22)
    n = int(sel.sum())
    if n < 120:
        api.log("%s interior %d px (not open)" % (tag, n))
        return None
    api.log("%s interior n=%d ctr=(%.3f,%.3f) y[%.3f,%.3f] x[%.3f,%.3f]"
            % (tag, n, np.median(X[sel]), np.median(Y[sel]),
               np.percentile(Y[sel], 2), np.percentile(Y[sel], 98),
               np.percentile(X[sel], 2), np.percentile(X[sel], 98)))
    return float(np.median(X[sel])), float(np.median(Y[sel])), n


def find_cheese(api, P, rgb):
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (b > r + 10.0) & (b > g + 5.0) & (b > 25.0) \
        & (Z > TABLE_Z + 0.005) & (Z < TABLE_Z + 0.14) \
        & (X > -0.34) & (X < 0.28) & (Y > -0.08) & (Y < 0.38)
    if m.sum() < 20:
        api.log("cheese %d px" % int(m.sum()))
        return None
    pts = np.stack([X[m], Y[m], Z[m]], -1)
    xy = pts[:, :2]
    used = np.zeros(len(pts), bool)
    clusters = []
    for i in range(len(pts)):
        if used[i]:
            continue
        used[i] = True
        grp, front = [i], [i]
        while front:
            j = front.pop()
            nb = np.nonzero((np.linalg.norm(xy - xy[j], axis=1) < 0.03) & (~used))[0]
            for k in nb:
                used[k] = True
                grp.append(int(k))
                front.append(int(k))
        clusters.append(grp)
    clusters.sort(key=len, reverse=True)
    q = pts[clusters[0]]
    api.log("cheese n=%d ctr=(%.3f,%.3f) ztop=%.3f" % (
        len(q), np.median(q[:, 0]), np.median(q[:, 1]), q[:, 2].max()))
    return float(np.median(q[:, 0])), float(np.median(q[:, 1])), float(q[:, 2].max())


def mv(api, xyz, seconds=1.2):
    api.move(list(xyz), seconds=seconds)
    e = np.asarray(api.eef(), dtype=np.float64)
    api.log("  mv (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)"
            % (xyz[0], xyz[1], xyz[2], e[0], e[1], e[2]))
    return e


def run(api):
    api.log("instruction: %r" % (api.instruction(),))
    f = api.capture("cam_high")
    P = base_cloud(f)
    bar = find_bar(api, P, "PRE")
    cheese0 = find_cheese(api, P, np.asarray(f.rgb, dtype=np.float64))
    if bar is None:
        api.log("no bar; abort")
        return
    hx, bar_y = bar
    face_y = bar_y - 0.030
    api.log("bar x=%.3f outer_y=%.3f face_y=%.3f" % (hx, bar_y, face_y))

    api.grip(0.08)
    api.settle(0.3)

    # ---- seat the far jaw on the bar, then drag with a standing press ----
    ey = bar_y + DRAG_EEF_Y_OFFSET
    api.log("drag: eef y %.3f puts the far jaw at %.3f, on a bar whose outer "
            "surface is %.3f" % (ey, ey - JAW_HALF_SPAN, bar_y))
    mv(api, (hx, ey + 0.035, SEAT_Z + 0.060), 1.4)
    seated = mv(api, (hx, ey, SEAT_Z - 0.020), 0.7)
    api.log("seated z=%.4f (bar top ~1.09)" % seated[2])
    press_z = seated[2] - PRESS_DEPTH

    # fixed ladder of waypoints -- never loop on achieved state, a stalled arm
    # would never reach the target and the loop would not terminate.
    y = seated[1]
    for _ in range(PULL_STEPS):
        y = min(y + PULL_STEP, PULL_END_Y)
        mv(api, (hx, y, press_z), 0.8)
    mv(api, (hx, max(y, 0.02), CARRY_Z), 1.2)
    mv(api, PARK, 1.4)

    f2 = api.capture("cam_high")
    P2 = base_cloud(f2)
    b2 = find_bar(api, P2, "POST")
    if b2:
        api.log("TRAVEL = %.3f" % (b2[1] - bar_y))
    interior = find_interior(api, P2, "POST", face_y)

    # ---- pick the cream cheese ------------------------------------------
    f3 = api.capture("cam_high")
    ch = find_cheese(api, base_cloud(f3), np.asarray(f3.rgb, dtype=np.float64)) or cheese0
    if ch is None:
        api.log("cheese lost; abort")
        return
    cx, cy, _ = ch
    mv(api, (cx, cy, 1.02), 1.6)
    mv(api, (cx, cy, GRASP_Z), 1.1)
    api.grip(0.0)
    api.settle(0.35)
    g = api.gripper()
    api.log("closed: %s" % (g,))
    if float(g.get("effort", 0.0)) < HOLDING_EFFORT:
        # nothing in the jaws: the descent was blocked. Back off, come in from
        # +y (away from the opened drawer) and try once more.
        api.log("grasp failed; re-approaching from +y")
        api.grip(0.08)
        mv(api, (cx, cy + RETRY_Y_BACKOFF, 1.010), 1.1)
        mv(api, (cx, cy, 0.965), 1.0)
        mv(api, (cx, cy, GRASP_Z), 0.9)
        api.grip(0.0)
        api.settle(0.35)
        g = api.gripper()
        api.log("closed(retry): %s" % (g,))
    if float(g.get("effort", 0.0)) < HOLDING_EFFORT:
        api.log("still empty-handed; stop")
        return

    if interior is None:
        api.log("drawer not open; stop rather than dump the box on the table")
        return
    dx, dy = interior[0], interior[1]
    api.log("drop (%.3f,%.3f,%.3f)" % (dx, dy, DROP_Z))
    mv(api, (cx, cy, CARRY_Z), 1.6)
    api.log("lifted: %s" % (api.gripper(),))
    mv(api, (dx, dy, CARRY_Z), 1.8)
    mv(api, (dx, dy, DROP_Z), 1.2)
    api.log("at drop: %s eef=%s" % (api.gripper(), np.round(api.eef(), 3).tolist()))
    api.grip(0.08)
    api.settle(0.3)
    api.log("released: %s" % (api.gripper(),))
