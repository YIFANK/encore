"""c2 spa_bowl_next_to_plate_stock -- v8.

Mechanism.  The pack's three demos all close the gripper to a 7-17 mm finger
gap, far narrower than a bowl: they pinch the bowl's near RIM WALL, not the
whole bowl.  Their grasp points (x=0.008/0.012/0.015, y=0.254/0.262/0.270) lie
exactly on the -y outer edge of a 0.105 m-wide bowl centred near (0.01, 0.32),
and their release points sit the same ~5 cm to -y of the plate centre: the end
effector tracks object_centre + (0, -R) throughout.

So: rebuild the tabletop from cam_high depth, fit a circle to each object's
top rim, classify by (rim radius, height), pinch the target bowl's near rim,
and release above the plate with the same -R offset.  The rim-circle fit
(rather than a bounding box) is what survives seeds where the bowl is clipped
by the image edge.

v4 adds the off-view search.  cam_high only reaches base y~0.43; on debug seed
62 the target bowl had been displaced to y=0.527, completely outside the fixed
camera, leaving only the distractor bowl in view.  When fewer than two wide
bowls are visible the program sweeps the wrist camera over the strip beyond
cam_high and merges what it finds.  Selection weights the x-gap to the plate
above the y-gap: in the demos the target bowl shares the plate's row (x~0.01
vs plate x~0.06) while the distractor sits ~0.18 m behind it, an ordering that
survives a large y displacement.
"""
import base64
import sys
import zlib

import numpy as np

PROVENANCE = {
    "TABLE_WIN": {"source": "debug 51-65 cam_high depth: clear-table window whose "
                            "median gives base table z=0.9010 on every seed",
                  "allowed": True},
    "OBJ_Z_MIN": {"source": "debug 51-65: table depth noise band <8 mm", "allowed": True},
    "MIN_PX": {"source": "debug 51-65: smallest real object cluster ~1500 px, "
                         "clutter below 300 px", "allowed": True},
    "BOWL_R": {"source": "debug 51-65 rim-circle fits: wide bowls r=0.0526-0.0541 m, "
                         "the small ramekin r=0.0414 m", "allowed": True},
    "BOWL_H": {"source": "debug 51-65: wide bowls 0.051 m tall, ramekin 0.043 m",
               "allowed": True},
    "PLATE_MIN_DIAM": {"source": "debug 51-65: plate 0.131 m wide, 0.019 m tall; the "
                                 "cookie box is 0.081x0.060", "allowed": True},
    "PLATE_MAX_H": {"source": "debug 51-65: plate height 0.019 m", "allowed": True},
    "GRASP_IN": {"source": "pack demo grasp keyframes lie 0-13 mm inside the bowl's "
                           "measured near edge (d0 -0.013, d1 -0.005, d2 +0.003)",
                 "allowed": True},
    "GRASP_Z": {"source": "pack demo grasp ee z 0.9226/0.9395/0.9426 minus measured "
                          "table 0.9010 -> +0.034 m", "allowed": True},
    "PLACE_Z": {"source": "pack demo release ee z 0.9355/0.9394/0.9373 (+0.036); "
                          "raised to clear the measured 0.019 m plate rim",
                "allowed": True},
    "SAFE_Z": {"source": "pack demo ee_path6 transit heights 1.02-1.18", "allowed": True},
    "RETRY_SHIFTS": {"source": "debug-seed search over the pinch offset", "allowed": True},
    "SEARCH_POSES": {"source": "debug seed 62 wrist probe: cam_high stops at base "
                               "y~0.43 while the displaced bowl sat at y=0.527. "
                               "Measured wrist footprint is 1.83h x 2.0h centred at "
                               "eef+(0.32h,-0.14h); at z=1.28 one look spans y up to "
                               "0.58 and costs less travel than the y=0.45 pose",
                     "allowed": True},
    "DRAG_DX": {"source": "debug 62 v7: a straight -y pull left the bowl at x=0.118 "
                          "where the descent stalled; pulling -x as well lands it in "
                          "the row the demos grasp from", "allowed": True},
    "BLOCK_XY": {"source": "debug 51-65: good descents land within 0.012 m in xy "
                           "while pressing 0.02 m low in z, so the jam test must "
                           "ignore z", "allowed": True},
    "X_WEIGHT": {"source": "pack demos: the grasped bowl at x~0.012 shares the plate's "
                           "row (plate x~0.06) while the debug 51-65 distractor bowl "
                           "sits at x~-0.175", "allowed": True},
    "DUP_R": {"source": "debug 51-65: bowl centres are never closer than 0.15 m, so "
                        "0.06 m matches a re-detection of the same bowl", "allowed": True},
    "REACH_Y": {"source": "debug seed 62 v4 receipt: a descent to y=0.478 at grasp "
                          "height stalled at y=0.4596 with 0.041 m residual, and the "
                          "arm then jammed; 0.450 is inside that boundary",
                "allowed": True},
    "DRAG_DY": {"source": "debug seed 62 v6 receipt: a 0.16 m pull moved the bowl "
                          "0.144 m (16 mm of slip) to y=0.382, still clipped; 0.22 m "
                          "lands it near the nominal bowl row the demos grasp from",
                "allowed": True},
    "REDETECT_R": {"source": "debug 62 v6: the partly-clipped dragged bowl fitted "
                             "r=0.0461 from 425 px, below the clean-view 0.0527",
                   "allowed": True},
    "MOVE_S": {"source": "generic controller mechanics: move_cartesian's step cap is "
                         "2*60*seconds and a blocked move always burns it; debug 62 "
                         "v4-v6 exhausted the 1000-step horizon that way",
               "allowed": True},
    "PARK": {"source": "debug 51-65: the arm's rest pose (eef -0.208,0.0,1.173) "
                       "leaves every tabletop object unoccluded in cam_high, unlike "
                       "hovering over the bowl (debug 62 v5 receipt)", "allowed": True},
    "RES_STUCK": {"source": "debug 51-65: a normal move lands within 0.012 m "
                            "(controller tolerance); >0.030 means blocked",
                  "allowed": True},
}

TABLE_WIN = (280, 500, 200, 512)
OBJ_Z_MIN = 0.008
MIN_PX = 300
BOWL_R = (0.047, 0.060)
BOWL_H = (0.044, 0.070)
PLATE_MIN_DIAM = 0.115
PLATE_MAX_H = 0.030
GRASP_IN = 0.004
GRASP_Z = 0.034
PLACE_Z = 0.055
SAFE_Z = 1.045
RETRY_SHIFTS = [(0.0, 0.0), (0.0, 0.010), (0.0, -0.010)]
R_CLAMP = (0.050, 0.056)
SEARCH_POSES = [[0.00, 0.25, 1.28], [-0.26, 0.25, 1.28]]
X_WEIGHT = 4.0
DUP_R = 0.06
REACH_Y = 0.450
DRAG_DY = 0.20
DRAG_DX = -0.06
RES_STUCK = 0.030
PARK = [-0.21, 0.0, 1.17]
MOVE_S = 1.0
REDETECT_R = (0.035, 0.065)
BLOCK_XY = 0.022


def _dump(tag, arr):
    b = zlib.compress(np.ascontiguousarray(arr).tobytes(), 6)
    sys.stderr.write("DUMP %s %s %s %d\n" % (tag, arr.dtype, list(arr.shape), len(b)))
    sys.stderr.write(base64.b64encode(b).decode() + "\n")
    sys.stderr.flush()


def cloud(f):
    K, T = np.asarray(f.intrinsics, float), np.asarray(f.t_base_cam, float)
    dep = np.asarray(f.depth, np.float32)
    h, w = dep.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * dep / K[0, 0]
    y = (vv - K[1, 2]) * dep / K[1, 1]
    P = np.stack([x, y, dep, np.ones_like(dep)], -1)
    return (P @ T.T)[..., :3]


def components(mask):
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    for r, c in np.argwhere(mask):
        if lab[r, c]:
            continue
        cur += 1
        lab[r, c] = cur
        st = [(r, c)]
        while st:
            a, b = st.pop()
            for p, q in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                if 0 <= p < h and 0 <= q < w and mask[p, q] and not lab[p, q]:
                    lab[p, q] = cur
                    st.append((p, q))
    return lab, cur


def fit_circle(x, y):
    A = np.stack([x, y, np.ones_like(x)], 1)
    b = x ** 2 + y ** 2
    c = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = c[0] / 2.0, c[1] / 2.0
    return float(cx), float(cy), float(np.sqrt(max(c[2] + cx ** 2 + cy ** 2, 1e-6)))


def clusters(B, mask, table, min_px):
    lab, n = components(mask)
    objs = []
    for i in range(1, n + 1):
        s = lab == i
        npx = int(s.sum())
        if npx < min_px:
            continue
        pts = B[s]
        xs, ys = np.sort(pts[:, 0]), np.sort(pts[:, 1])
        lo = max(1, int(0.01 * npx))
        x0, x1 = float(xs[lo]), float(xs[-lo])
        y0, y1 = float(ys[lo]), float(ys[-lo])
        top = float(np.percentile(pts[:, 2], 99.5))
        rim = pts[pts[:, 2] > top - 0.010]
        if len(rim) >= 40:
            rx, ry, rr = fit_circle(rim[:, 0], rim[:, 1])
        else:
            rx, ry, rr = 0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.25 * (x1 - x0 + y1 - y0)
        objs.append({"px": npx, "dx": x1 - x0, "dy": y1 - y0,
                     "bx": 0.5 * (x0 + x1), "by": 0.5 * (y0 + y1),
                     "cx": rx, "cy": ry, "r": rr, "h": top - table})
    return objs


def perceive(api):
    f = api.capture("cam_high")
    B = cloud(f)
    Z = B[..., 2]
    v0, v1, u0, u1 = TABLE_WIN
    win = Z[v0:v1, u0:u1]
    table = float(np.median(win[(win > 0.80) & (win < 0.98)]))
    m = ((Z > table + OBJ_Z_MIN) & (Z < table + 0.25)
         & (B[..., 0] > -0.32) & (B[..., 0] < 0.45)
         & (B[..., 1] > -0.42) & (B[..., 1] < 0.45)
         & np.isfinite(Z))
    return f, table, clusters(B, m, table, MIN_PX)


def is_bowl(o):
    return BOWL_H[0] <= o["h"] <= BOWL_H[1] and BOWL_R[0] <= o["r"] <= BOWL_R[1]


def search_offview(api, table, known):
    """Wrist sweep for a bowl displaced outside cam_high's strip."""
    found = []
    for i, pose in enumerate(SEARCH_POSES):
        r = api.move(pose, seconds=MOVE_S)
        api.log("search%d pose=%s eef=%s res=%.4f"
                % (i, pose, np.round(api.eef(), 4).tolist(), r))
        w = api.capture("cam_arm_wrist")
        B = cloud(w)
        Z = B[..., 2]
        m = (Z > table + 0.020) & (Z < table + 0.095) & np.isfinite(Z)
        for o in clusters(B, m, table, 600):
            if not is_bowl(o):
                continue
            dup = any((o["cx"] - kx) ** 2 + (o["cy"] - ky) ** 2 < DUP_R ** 2
                      for kx, ky in known)
            api.log("search%d cand=(%.3f,%.3f) r=%.4f h=%.3f px=%d dup=%s"
                    % (i, o["cx"], o["cy"], o["r"], o["h"], o["px"], dup))
            if not dup:
                found.append(o)
                known.append((o["cx"], o["cy"]))
        if found:
            break
    return found


def run(api):
    api.log("instruction=%r" % api.instruction())
    f, table, objs = perceive(api)
    api.log("table_z=%.4f nobj=%d" % (table, len(objs)))
    for o in objs:
        api.log("obj px=%5d rim=(%.3f,%.3f) r=%.4f h=%.3f d=(%.3f,%.3f)"
                % (o["px"], o["cx"], o["cy"], o["r"], o["h"], o["dx"], o["dy"]))

    plates = [o for o in objs if o["h"] <= PLATE_MAX_H
              and max(o["dx"], o["dy"]) >= PLATE_MIN_DIAM]
    bowls = [o for o in objs if is_bowl(o)]
    if not plates:
        api.log("PERCEPTION FAIL: no plate")
        _dump("rgb_percfail", f.rgb)
        return "perception fail"
    plate = max(plates, key=lambda o: o["px"])
    api.log("plate rim=(%.3f,%.3f) bbox=(%.3f,%.3f) d=(%.3f,%.3f)"
            % (plate["cx"], plate["cy"], plate["bx"], plate["by"], plate["dx"], plate["dy"]))

    if len(bowls) < 2:
        api.log("only %d wide bowl(s) in cam_high -- sweeping wrist" % len(bowls))
        known = [(o["cx"], o["cy"]) for o in objs]
        bowls = bowls + search_offview(api, table, known)
    if not bowls:
        api.log("PERCEPTION FAIL: no bowl")
        _dump("rgb_percfail", f.rgb)
        return "perception fail"

    def score(o):
        return X_WEIGHT * (o["cx"] - plate["bx"]) ** 2 + (o["cy"] - plate["by"]) ** 2

    bowl = min(bowls, key=score)
    R = min(max(bowl["r"], R_CLAMP[0]), R_CLAMP[1])
    api.log("bowl rim=(%.3f,%.3f) r=%.4f -> R=%.4f score=%.4f (of %d bowls)"
            % (bowl["cx"], bowl["cy"], bowl["r"], R, score(bowl), len(bowls)))

    gz = table + GRASP_Z
    # A bowl displaced past the arm's reach envelope cannot be lifted where it
    # stands (debug 62: the descent stalls and the arm jams).  Pinch its rim at
    # the edge of the envelope, drag it back into the workspace, and re-perceive.
    if bowl["cy"] - R + GRASP_IN > REACH_Y:
        api.log("target rim y=%.3f beyond reach %.3f -- hook-drag"
                % (bowl["cy"] - R + GRASP_IN, REACH_Y))
        hx = bowl["cx"]
        api.grip(0.08)
        api.move([hx, REACH_Y, SAFE_Z], seconds=MOVE_S)
        r = api.move([hx, REACH_Y, gz], seconds=MOVE_S)
        hook_start = api.eef()
        api.log("hook down eef=%s res=%.4f" % (np.round(hook_start, 4).tolist(), r))
        api.grip(0.0)
        g = api.gripper()
        api.log("hook closed gap=%.4f effort=%.2f" % (g["width_m"], g["effort"]))
        if g["effort"] >= 1.0:
            r = api.move([hx + DRAG_DX, REACH_Y - DRAG_DY, gz], seconds=1.5)
            api.log("dragged eef=%s res=%.4f" % (np.round(api.eef(), 4).tolist(), r))
        drag_end = api.eef()
        api.grip(0.08)
        api.settle(0.3)
        api.move([hx + DRAG_DX, REACH_Y - DRAG_DY, SAFE_Z], seconds=MOVE_S)
        # The arm must leave the frame or it merges with the bowl in cam_high.
        api.move(PARK, seconds=2.0)
        f, table, objs = perceive(api)
        plates2 = [o for o in objs if o["h"] <= PLATE_MAX_H
                   and max(o["dx"], o["dy"]) >= PLATE_MIN_DIAM]
        bowls2 = [o for o in objs if is_bowl(o)]
        for o in objs:
            api.log("re-obj px=%5d rim=(%.3f,%.3f) r=%.4f h=%.3f"
                    % (o["px"], o["cx"], o["cy"], o["r"], o["h"]))
        if plates2:
            plate = max(plates2, key=lambda o: o["px"])
        moved = float(hook_start[1] - drag_end[1])
        guess = (bowl["cx"] + DRAG_DX, bowl["cy"] - moved)
        near = [o for o in objs
                if BOWL_H[0] <= o["h"] <= BOWL_H[1]
                and REDETECT_R[0] <= o["r"] <= REDETECT_R[1]
                and (o["cx"] - guess[0]) ** 2 + (o["cy"] - guess[1]) ** 2 < 0.09 ** 2]
        api.log("dead-reckon guess=(%.3f,%.3f) moved=%.3f near=%d"
                % (guess[0], guess[1], moved, len(near)))
        if near:
            bowl = min(near, key=lambda o: (o["cx"] - guess[0]) ** 2 + (o["cy"] - guess[1]) ** 2)
            R = R_CLAMP[0] + 0.002
        elif bowls2:
            bowl = min(bowls2, key=score)
            R = min(max(bowl["r"], R_CLAMP[0]), R_CLAMP[1])
        else:
            # Dead-reckon the bowl from how far the pinch actually carried it.
            bowl = dict(bowl)
            bowl["cy"] = guess[1]
            api.log("no bowl re-detected; using dead-reckoned centre")
        api.log("after drag: bowl rim=(%.3f,%.3f) R=%.4f" % (bowl["cx"], bowl["cy"], R))
    api.grip(0.08)
    held = False
    offx = offy = 0.0
    for k, (sx, sy) in enumerate(RETRY_SHIFTS):
        gx = bowl["cx"] + sx
        gy = bowl["cy"] - R + GRASP_IN + sy
        api.log("try%d target=(%.3f,%.3f,%.3f)" % (k, gx, gy, gz))
        api.move([gx, gy, SAFE_Z], seconds=MOVE_S)
        r = api.move([gx, gy, gz], seconds=MOVE_S)
        e = api.eef()
        exy = float(np.hypot(gx - e[0], gy - e[1]))
        api.log("try%d down eef=%s res=%.4f exy=%.4f"
                % (k, np.round(e, 4).tolist(), r, exy))
        if exy > BLOCK_XY:
            # The wrist stalled short in xy: re-aim once by the observed error.
            gx2, gy2 = gx + (gx - e[0]), gy + (gy - e[1])
            r = api.move([gx2, gy2, gz], seconds=MOVE_S)
            e = api.eef()
            exy = float(np.hypot(gx - e[0], gy - e[1]))
            api.log("try%d re-aim -> eef=%s exy=%.4f"
                    % (k, np.round(e, 4).tolist(), exy))
            if exy > BLOCK_XY:
                api.log("try%d blocked -- abandoning ladder" % k)
                break
        api.grip(0.0)
        g = api.gripper()
        api.log("try%d closed gap=%.4f effort=%.2f" % (k, g["width_m"], g["effort"]))
        if g["effort"] < 1.0:
            api.grip(0.08)
            continue
        api.move([gx, gy, SAFE_Z], seconds=MOVE_S)
        g = api.gripper()
        api.log("try%d lifted eef=%s gap=%.4f effort=%.2f"
                % (k, np.round(api.eef(), 4).tolist(), g["width_m"], g["effort"]))
        if g["effort"] >= 1.0:
            held = True
            offx, offy = bowl["cx"] - gx, bowl["cy"] - gy
            break
        api.grip(0.08)
    if not held:
        api.log("GRASP FAIL")
        _dump("rgb_graspfail", api.capture("cam_high").rgb)
        return "grasp fail"

    api.log("carry offset=(%.3f,%.3f)" % (offx, offy))
    px, py = plate["bx"] - offx, plate["by"] - offy
    api.move([px, py, SAFE_Z], seconds=MOVE_S)
    g = api.gripper()
    api.log("over plate eef=%s gap=%.4f effort=%.2f"
            % (np.round(api.eef(), 4).tolist(), g["width_m"], g["effort"]))
    r = api.move([px, py, table + PLACE_Z], seconds=MOVE_S)
    api.log("place down eef=%s res=%.4f" % (np.round(api.eef(), 4).tolist(), r))
    api.grip(0.08)
    api.settle(0.5)
    api.move([px, py, SAFE_Z], seconds=MOVE_S)
    api.settle(0.5)

    f2, t2, objs2 = perceive(api)
    for o in objs2:
        api.log("post px=%5d rim=(%.3f,%.3f) r=%.4f h=%.3f"
                % (o["px"], o["cx"], o["cy"], o["r"], o["h"]))
    return "placed"
