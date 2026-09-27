"""c2clean spa_bowl_cookie_box_task_k3 -- v2.

Intent: "Pick the akita black bowl on the stove and place it on the plate".

Perception-driven: cam_high at the home pose gives a base-frame height map.
The bowl standing on the stove is the only structure above 0.955 m in the
stove neighbourhood; the plate is the largest 0.910-0.938 m slab on the +y
half of the table. The grasp straddles the bowl wall on the +y arc, which is
the offset the mate pack's three demos used.
"""
import numpy as np

PROVENANCE = {
    "BOWL_WIN": {
        "source": "debug seeds 51-65 cam_high height maps: the stove bowl's rim "
                  "ring lies inside x[-0.37,-0.16] y[-0.34,0.06]; the cabinet "
                  "(top ~1.13) starts at x>-0.14 and the nearest other bowl is "
                  "at y>+0.15",
        "allowed": True},
    "BOWL_ZLO/BOWL_ZHI": {
        "source": "debug seeds 51-65: table 0.900, stove slab top ~0.93, stove "
                  "bowl rim top 0.979-0.980 on all 15 seeds; 0.955-1.05 "
                  "isolates the rim and excludes the gripper (z>1.05 at home)",
        "allowed": True},
    "GRASP_DY": {
        "source": "mate pack ee_path6 close points vs the ring centre measured "
                  "on debug seeds: the three demo grasps sit ~0.042 m on the "
                  "+y side of the bowl centre (ring outer radius 0.055)",
        "allowed": True},
    "GRASP_DZ": {
        "source": "mate pack close z 0.948/0.942/0.959 vs measured rim top "
                  "0.979-0.980 -> 0.030 m below the rim top",
        "allowed": True},
    "PLATE_WIN/PLATE_ZLO/PLATE_ZHI": {
        "source": "debug seeds 51-65: plate top 0.920 over a 0.13x0.14 m disc "
                  "centred near (0.06,0.20); window x[-0.08,0.30] y[0.08,0.40] "
                  "holds it and excludes the table bowl at (-0.19,0.20)",
        "allowed": True},
    "PLACE_DY": {
        "source": "mate pack release y 0.248/0.229/0.239 vs measured plate "
                  "centre y 0.185-0.215 -> release ~0.040 m on the +y side, "
                  "which is the same offset the rim grasp carries",
        "allowed": True},
    "PLACE_DZ": {
        "source": "mate pack release z 0.931/0.939/0.929 vs measured plate top "
                  "0.920 -> 0.015 m above the plate",
        "allowed": True},
    "HELD_GAP": {
        "source": "v1 debug run: finger gap after the lift 0.0046-0.0047 m on "
                  "the three successful grasps, 0.0018 m on the empty one",
        "allowed": True},
    "FALLBACK_GRASP/FALLBACK_PLATE": {
        "source": "mate pack ee_path6 mean close and release points",
        "allowed": True},
    "LIFT_Z": {
        "source": "mate pack carry apex z 1.077-1.138",
        "allowed": True},
}

BOWL_WIN = (-0.37, -0.16, -0.34, 0.06)
BOWL_ZLO, BOWL_ZHI = 0.955, 1.05
PLATE_WIN = (-0.08, 0.30, 0.08, 0.40)
PLATE_ZLO, PLATE_ZHI = 0.910, 0.938

GRASP_DY = 0.042
GRASP_DZ = -0.030
PLACE_DY = 0.040
PLACE_DZ = 0.015
LIFT_Z = 1.11
HELD_GAP = 0.0035

FALLBACK_GRASP = (-0.276, -0.139, 0.979)   # mate-pack mean close, as a centre
FALLBACK_PLATE = (0.060, 0.200, 0.920)

OPEN_W, CLOSE_W = 0.08, 0.0
RES = 0.01


def _points(f):
    """Every pixel of a frame as base-frame xyz (N,3) plus rgb (N,3)."""
    d = np.asarray(f.depth, dtype=np.float64)
    K = np.asarray(f.intrinsics, dtype=np.float64)
    T = np.asarray(f.t_base_cam, dtype=np.float64)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    good = np.isfinite(d) & (d > 0)
    d = np.where(good, d, 1.0)
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], axis=-1).reshape(-1, 4)
    B = (P @ T.T)[:, :3]
    g = good.reshape(-1)
    return B[g], np.asarray(f.rgb).reshape(-1, 3)[g]


def _cell_set(P, win, zlo, zhi):
    """Unique 1 cm (ix,iy) cells occupied inside a window, plus the points."""
    x0, x1, y0, y1 = win
    m = ((P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1)
         & (P[:, 2] > zlo) & (P[:, 2] < zhi))
    Q = P[m]
    if len(Q) == 0:
        return None, None
    ix = ((Q[:, 0] - x0) / RES).astype(int)
    iy = ((Q[:, 1] - y0) / RES).astype(int)
    cells = np.unique(np.stack([ix, iy], 1), axis=0)
    return cells, Q


def _xy(cells, win):
    x0, _, y0, _ = win
    return (x0 + cells[:, 0] * RES + RES / 2, y0 + cells[:, 1] * RES + RES / 2)


def _largest_blob(cells):
    """8-neighbour connected components over integer cell indices."""
    if cells is None or len(cells) == 0:
        return None
    key = {(int(a), int(b)) for a, b in cells}
    seen = set()
    best = []
    for start in key:
        if start in seen:
            continue
        comp, stack = [], [start]
        seen.add(start)
        while stack:
            a, b = stack.pop()
            comp.append((a, b))
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    n = (a + da, b + db)
                    if n in key and n not in seen:
                        seen.add(n)
                        stack.append(n)
        if len(comp) > len(best):
            best = comp
    return np.array(best, dtype=int)


def _bowl(api, f):
    P, _ = _points(f)
    cells, Q = _cell_set(P, BOWL_WIN, BOWL_ZLO, BOWL_ZHI)
    if cells is None or len(cells) < 8:
        api.log("bowl: no cells, using fallback")
        return FALLBACK_GRASP, False
    X, Y = _xy(cells, BOWL_WIN)
    cx = (X.min() + X.max()) / 2
    cy = (Y.min() + Y.max()) / 2
    sx = X.max() - X.min()
    sy = Y.max() - Y.min()
    top = float(Q[:, 2].max())
    api.log("bowl cells=%d cen=(%.3f,%.3f) size=(%.3f,%.3f) top=%.3f"
            % (len(cells), cx, cy, sx, sy, top))
    if not (0.06 <= sx <= 0.15 and 0.06 <= sy <= 0.15):
        api.log("bowl: implausible size, using fallback")
        return FALLBACK_GRASP, False
    return (float(cx), float(cy), top), True


def _plate(api, f):
    P, _ = _points(f)
    cells, Q = _cell_set(P, PLATE_WIN, PLATE_ZLO, PLATE_ZHI)
    blob = _largest_blob(cells)
    if blob is None or len(blob) < 20:
        api.log("plate: no blob, using fallback")
        return FALLBACK_PLATE
    X, Y = _xy(blob, PLATE_WIN)
    px = (X.min() + X.max()) / 2
    py = (Y.min() + Y.max()) / 2
    sx = X.max() - X.min()
    sy = Y.max() - Y.min()
    api.log("plate cells=%d cen=(%.3f,%.3f) size=(%.3f,%.3f)"
            % (len(blob), px, py, sx, sy))
    if not (0.08 <= sx <= 0.20 and 0.08 <= sy <= 0.20):
        api.log("plate: implausible size, using fallback")
        return FALLBACK_PLATE
    return (float(px), float(py), 0.920)


def _try_grasp(api, bowl):
    cx, cy, top = bowl
    gx, gy, gz = cx, cy + GRASP_DY, top + GRASP_DZ
    api.grip(OPEN_W)
    r = api.move([gx, gy, top + 0.07], seconds=2.0)
    api.log("hover res %.4f" % r)
    r = api.move([gx, gy, gz], seconds=1.5)
    api.log("descend res %.4f eef %s"
            % (r, np.round(api.eef(), 4).tolist()))
    api.grip(CLOSE_W)
    r = api.move([gx, gy, LIFT_Z], seconds=1.5)
    g = api.gripper()
    api.log("lift res %.4f gap %.4f" % (r, g["width_m"]))
    return g["width_m"]


def run(api):
    api.log("instruction: %s" % api.instruction())
    f = api.capture("cam_high")
    bowl, _ = _bowl(api, f)
    plate = _plate(api, f)

    gap = _try_grasp(api, bowl)
    if gap < HELD_GAP:
        api.log("grasp empty (gap %.4f) -- re-perceiving" % gap)
        api.grip(OPEN_W)
        api.move([bowl[0], bowl[1] + 0.25, 1.15], seconds=2.0)
        f2 = api.capture("cam_high")
        bowl2, ok = _bowl(api, f2)
        if ok:
            bowl = bowl2
        gap = _try_grasp(api, bowl)
        api.log("retry gap %.4f" % gap)

    px, py, ptop = plate
    rx, ry = px, py + PLACE_DY
    r = api.move([rx, ry, LIFT_Z], seconds=2.0)
    api.log("carry res %.4f gap %.4f" % (r, api.gripper()["width_m"]))
    r = api.move([rx, ry, ptop + PLACE_DZ + 0.04], seconds=1.5)
    api.log("predrop res %.4f" % r)
    r = api.move([rx, ry, ptop + PLACE_DZ], seconds=1.0)
    api.log("drop res %.4f eef %s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(OPEN_W)
    api.settle(0.3)
    api.move([rx, ry, 1.05], seconds=1.0)
    api.log("released, eef %s" % np.round(api.eef(), 4).tolist())
