"""v6 -- rim-pinch pick of a black bowl, place on the plate.

Target selection flag TARGET: "center" (bowl nearest the table-centre bowl's
debug-seed home) or "plate" (bowl nearest the plate).  v2 probes "center".
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "RIM_R": {
        "source": "debug seeds 51-65 cam_high depth: the two bowl clusters "
                  "measure 0.108 m across (height band table+0.046..0.075), "
                  "so the rim sits 0.054 m from the cluster centre",
        "allowed": True},
    "GRASP_DZ": {
        "source": "packs/c2clean_spa_bowl_table_center_task_k3 pack.json "
                  "ee_path6 grasp waypoints z = 0.9178/0.9194 minus the table "
                  "height 0.901 measured from my own debug-seed depth",
        "allowed": True},
    "PLACE_DZ": {
        "source": "same pack, release keyframes z = 0.9342/0.9453/0.9307 "
                  "minus the same measured table height",
        "allowed": True},
    "RIM_SIGN": {
        "source": "debug-seed depth: the +y bowl centres at y ~ 0.32 would put "
                  "a +y rim grasp at y ~ 0.37, past the largest eef y either "
                  "pack ever demonstrates (0.284); the near arc is used instead",
        "allowed": True},
    "RIM_DIR_Y": {
        "source": "same pack: the three grasp keyframes sit at y = 0.040/"
                  "0.059/0.056 while the table-centre bowl cluster measured on "
                  "debug seeds sits at y ~ -0.003, i.e. the demos pinch the "
                  "+y rim; the release keyframes sit the same +0.055 in y from "
                  "the plate cluster centre",
        "allowed": True},
    "CARRY_DZ": {
        "source": "debug-seed depth: tallest tabletop obstacle between the "
                  "bowls and the plate is 0.120 m above the table",
        "allowed": True},
    "BOWL_BAND": {
        "source": "debug-seed depth height map: bowls top out 0.051 above the "
                  "table, the next-tallest tabletop props 0.043 (small vessel) "
                  "and 0.031 (stove slab)",
        "allowed": True},
    "BOWL_SPAN_LO": {
        "source": "debug-seed depth: both bowls measure 0.108 m across on "
                  "every seed; the only other prop that could reach the bowl "
                  "height band is the 0.085 m small vessel",
        "allowed": True},
    "AIR_CLOSE_GAP": {
        "source": "debug-seed v4 run: a bite on the bowl wall leaves a finger "
                  "gap of 0.0079-0.0169 m; a close on nothing leaves ~0",
        "allowed": True},
    "PLATE_BAND": {
        "source": "debug-seed depth height map: the plate tops out 0.019 and "
                  "spans 0.132-0.138 m in both axes; the cookie box also tops "
                  "at 0.019 but spans only 0.078 x 0.060, and the stove slab "
                  "tops at 0.031 and spans 0.162-0.186",
        "allowed": True},
    "CENTER_HOME": {
        "source": "debug seeds 51-65 cam_high depth: one of the two bowls sits "
                  "at x = -0.080, y in [-0.018, +0.012] on every seed while "
                  "the other one is what the seed moves; this literal is that "
                  "fixed bowl's measured home, used only to tell the two apart",
        "allowed": True},
    "PLATE_SPAN": {
        "source": "debug-seed depth: the plate spans 0.132-0.138 m in both "
                  "axes, the stove slab 0.162-0.186, the cookie box 0.078",
        "allowed": True},
    "RETRY_STEP": {
        "source": "debug-seed v4 logs: the successful bites sit within 0.008 m "
                  "of the nominal rim aim, so one step of that size is the "
                  "smallest useful correction after a close on nothing",
        "allowed": True},
    "GRID_RES": {
        "source": "chosen so the 0.108 m bowl footprint spans ~18 cells in my "
                  "own debug-seed height maps; generic perception mechanics",
        "allowed": True},
    "TABLE_CROP": {
        "source": "debug-seed depth: tabletop props all fall inside "
                  "x in (-0.35, 0.35), y in (-0.45, 0.45) in base frame",
        "allowed": True},
}

TARGET = "far"

RIM_R = 0.054
GRASP_DZ = 0.017
PLACE_DZ = 0.034
CARRY_DZ = 0.145
AIR_CLOSE_GAP = 0.004
RETRY_STEP = 0.007
BOWL_LO, BOWL_HI = 0.046, 0.075
BOWL_SPAN_LO = 0.095
PLATE_LO, PLATE_HI = 0.013, 0.023
PLATE_SPAN_LO, PLATE_SPAN_HI = 0.10, 0.17
X0, X1, Y0, Y1 = -0.35, 0.35, -0.45, 0.45
RES = 0.006
CHUNK = 1800

CENTER_HOME = (-0.085, 0.000)


# --------------------------------------------------------------------- utils
def blob(api, tag, arr):
    b = base64.b64encode(zlib.compress(
        np.ascontiguousarray(arr).tobytes(), 6)).decode()
    n = (len(b) + CHUNK - 1) // CHUNK
    api.log("BLOB %s n=%d dtype=%s shape=%s" % (tag, n, arr.dtype.str, arr.shape))
    for i in range(n):
        api.log("B %s %d %s" % (tag, i, b[i * CHUNK:(i + 1) * CHUNK]))


def cloud(f):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.asarray(f.depth, float)
    if d.ndim == 3:
        d = d[..., 0]
    H, W = d.shape
    vs, us = np.mgrid[0:H, 0:W]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    pc = np.stack([(us - cx) * d / fx, (vs - cy) * d / fy, d, np.ones_like(d)], -1)
    return pc @ T.T


def height_map(P):
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    m = (x > X0) & (x < X1) & (y > Y0) & (y < Y1) & (z > 0.7) & (z < 1.05)
    h, e = np.histogram(z[m], bins=200)
    tz = 0.5 * (e[h.argmax()] + e[h.argmax() + 1])
    nx = int((X1 - X0) / RES)
    ny = int((Y1 - Y0) / RES)
    gi = ((x - X0) / RES).astype(int)
    gj = ((y - Y0) / RES).astype(int)
    ok = m & (gi >= 0) & (gi < nx) & (gj >= 0) & (gj < ny)
    hm = np.full((nx, ny), -1.0)
    np.maximum.at(hm, (gi[ok], gj[ok]), z[ok] - tz)
    return tz, hm


def components(hm, lo, hi, min_cells=15):
    """4-connected labelling of cells whose height lies in [lo, hi]."""
    m = (hm > lo) & (hm < hi)
    lab = np.zeros(m.shape, int)
    cur = 0
    idx = np.argwhere(m)
    seen = set(map(tuple, idx))
    out = []
    for start in map(tuple, idx):
        if lab[start]:
            continue
        cur += 1
        stack = [start]
        lab[start] = cur
        cells = []
        while stack:
            i, j = stack.pop()
            cells.append((i, j))
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                p = (i + di, j + dj)
                if p in seen and not lab[p]:
                    lab[p] = cur
                    stack.append(p)
        if len(cells) < min_cells:
            continue
        ii = np.array([c[0] for c in cells])
        jj = np.array([c[1] for c in cells])
        out.append(dict(
            n=len(cells),
            x0=X0 + ii.min() * RES, x1=X0 + ii.max() * RES,
            y0=Y0 + jj.min() * RES, y1=Y0 + jj.max() * RES,
            cx=X0 + 0.5 * (ii.min() + ii.max()) * RES,
            cy=Y0 + 0.5 * (jj.min() + jj.max()) * RES,
            top=float(hm[ii, jj].max())))
    return out


def merge_close(cs, gap=0.03):
    """Occlusion by the parked arm splits one bowl into two strips."""
    out = list(cs)
    changed = True
    while changed:
        changed = False
        for a in range(len(out)):
            for b in range(a + 1, len(out)):
                A, B = out[a], out[b]
                dx = max(A["x0"] - B["x1"], B["x0"] - A["x1"], 0.0)
                dy = max(A["y0"] - B["y1"], B["y0"] - A["y1"], 0.0)
                if dx <= gap and dy <= gap:
                    M = dict(
                        n=A["n"] + B["n"],
                        x0=min(A["x0"], B["x0"]), x1=max(A["x1"], B["x1"]),
                        y0=min(A["y0"], B["y0"]), y1=max(A["y1"], B["y1"]),
                        top=max(A["top"], B["top"]))
                    M["cx"] = 0.5 * (M["x0"] + M["x1"])
                    M["cy"] = 0.5 * (M["y0"] + M["y1"])
                    out = [c for k, c in enumerate(out) if k not in (a, b)] + [M]
                    changed = True
                    break
            if changed:
                break
    return out


# ------------------------------------------------------------------ the task
def perceive(api):
    f = api.capture("cam_high")
    P = cloud(f)
    tz, hm = height_map(P)
    bowls = [c for c in merge_close(components(hm, BOWL_LO, BOWL_HI))
             if max(c["x1"] - c["x0"], c["y1"] - c["y0"]) > BOWL_SPAN_LO]
    flats = components(hm, PLATE_LO, PLATE_HI)
    api.log("TZ %.4f" % tz)
    for c in bowls:
        api.log("BOWL n=%d mid(%.3f,%.3f) span(%.3f,%.3f) top %.3f"
                % (c["n"], c["cx"], c["cy"], c["x1"] - c["x0"],
                   c["y1"] - c["y0"], c["top"]))
    for c in flats:
        api.log("FLAT n=%d mid(%.3f,%.3f) span(%.3f,%.3f) top %.3f"
                % (c["n"], c["cx"], c["cy"], c["x1"] - c["x0"],
                   c["y1"] - c["y0"], c["top"]))
    # plate: the only flat prop wider than 0.10 m in both axes
    pl = [c for c in flats
          if c["top"] < PLATE_HI
          and min(c["x1"] - c["x0"], c["y1"] - c["y0"]) > PLATE_SPAN_LO
          and max(c["x1"] - c["x0"], c["y1"] - c["y0"]) < PLATE_SPAN_HI]
    plate = max(pl, key=lambda c: c["n"]) if pl else None
    return tz, bowls, plate, f


def choose(bowls, plate):
    if not bowls:
        return None
    key = lambda c: (c["cx"] - CENTER_HOME[0]) ** 2 + (c["cy"] - CENTER_HOME[1]) ** 2
    if TARGET == "far":
        return max(bowls, key=key)
    return min(bowls, key=key)


def rim_sign(cy):
    """Pinch the rim arc that faces the robot base: the far arc of the +y bowl
    sits past every eef y the packs ever demonstrate."""
    return -1.0 if cy > 0.15 else 1.0


def go(api, xyz, seconds=2.0, tag=""):
    r = api.move([float(v) for v in xyz], seconds=seconds)
    e = api.eef()
    api.log("MOVE %s -> want(%.4f,%.4f,%.4f) got(%.4f,%.4f,%.4f) res %.4f"
            % (tag, xyz[0], xyz[1], xyz[2], e[0], e[1], e[2], r))
    return np.asarray(e, float)


def aim(api, xyz, tries=2, seconds=1.2, tag=""):
    """Close the loop on the tracking bias: re-issue with the measured error."""
    want = np.asarray(xyz, float)
    got = go(api, want, seconds=2.0, tag=tag)
    for k in range(tries):
        err = want - got
        if float(np.linalg.norm(err)) < 0.004:
            break
        got = go(api, want + err, seconds=seconds, tag=tag + "/fix%d" % k)
    return got


def run(api):
    api.log("INSTR %s" % api.instruction())
    api.log("TARGET_MODE %s" % TARGET)
    tz, bowls, plate, f0 = perceive(api)
    if plate is None or not bowls:
        api.log("ABORT no plate or no bowl")
        return
    b = choose(bowls, plate)
    api.log("PICK bowl mid(%.3f,%.3f) top %.3f | PLATE mid(%.3f,%.3f)"
            % (b["cx"], b["cy"], b["top"], plate["cx"], plate["cy"]))

    sgn = rim_sign(b["cy"])
    api.log("RIM_SIGN %+.0f" % sgn)
    gx, gy = b["cx"], b["cy"] + sgn * RIM_R
    px, py = plate["cx"], plate["cy"] + sgn * RIM_R

    api.grip(0.08)
    aim(api, [gx, gy, tz + CARRY_DZ], tag="hover")
    aim(api, [gx, gy, tz + GRASP_DZ], tries=2, tag="descend")
    api.grip(0.0)
    api.settle(0.3)
    g = api.gripper()
    api.log("AFTER_CLOSE gap %.4f effort %.2f" % (g["width_m"], g["effort"]))
    if g["width_m"] < AIR_CLOSE_GAP:
        # The jaws met each other, not the bowl wall: the bite missed the rim.
        # Step the aim one finger-width further out along the rim normal and
        # take the one retry the step budget affords.
        api.log("RETRY air close")
        api.grip(0.08)
        go(api, [gx, gy, tz + CARRY_DZ], seconds=1.5, tag="retry_up")
        gy = gy + sgn * RETRY_STEP
        aim(api, [gx, gy, tz + CARRY_DZ], tries=1, tag="retry_hover")
        aim(api, [gx, gy, tz + GRASP_DZ], tries=2, tag="retry_descend")
        api.grip(0.0)
        api.settle(0.3)
        g = api.gripper()
        api.log("RETRY_CLOSE gap %.4f effort %.2f" % (g["width_m"], g["effort"]))

    go(api, [gx, gy, tz + CARRY_DZ], seconds=2.0, tag="lift")
    g = api.gripper()
    api.log("AFTER_LIFT gap %.4f effort %.2f" % (g["width_m"], g["effort"]))

    aim(api, [px, py, tz + CARRY_DZ], tag="over_plate")
    g = api.gripper()
    api.log("OVER_PLATE gap %.4f effort %.2f" % (g["width_m"], g["effort"]))
    aim(api, [px, py, tz + PLACE_DZ], tries=1, tag="lower")
    g = api.gripper()
    api.log("BEFORE_RELEASE gap %.4f effort %.2f" % (g["width_m"], g["effort"]))

    api.grip(0.08)
    api.settle(1.0)
    go(api, [px, py, tz + CARRY_DZ], seconds=1.5, tag="retreat")
    api.settle(0.5)

    tz2, bowls2, plate2, f1 = perceive(api)
    blob(api, "final_rgb", np.asarray(f1.rgb)[::2, ::2])
    api.log("END")
