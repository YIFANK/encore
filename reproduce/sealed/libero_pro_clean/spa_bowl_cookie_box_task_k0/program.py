"""c2clean spa_bowl_cookie_box_task_k0 -- v7.

Pick the akita black bowl standing on the stove and place it on the plate.

Perception is a top-down height map built from cam_high.  The target is the
only object whose top surface lies in the 0.055-0.115 m band above the table
(it is raised by the stove slab); the two table bowls top out at 0.051 and the
cabinet/robot are far above the band.  The plate is the only pale 0.135 m disc
sitting 0.007-0.021 m above the table.  The bowl is 0.11 m across, wider than
the 0.078 m jaw opening, so it is taken by a rim pinch on the +y arc, where the
straight-down wrist's closing axis (world y) crosses the wall.
"""

import base64
import zlib

import numpy as np

PROVENANCE = {
    "R_DOWN": {"source": "generic controller mechanics; matches home "
                         "api.tool_rotation() ~ diag(1,-1,-1) on debug seeds 51-57",
               "allowed": True},
    "GRID_XL/GRID_YL/GRID_RES": {
        "source": "debug seeds 51-57 cam_high cloud extent; 5 mm cells",
        "allowed": True},
    "BOWL_BAND": {"source": "debug seeds 51,53,55,57: stove bowl rim 0.0770-0.0775 above "
                            "the table; table bowls 0.0495/0.0415; band 0.055-0.115 "
                            "isolates the stove bowl (cabinet top 0.226, robot 0.297)",
                  "allowed": True},
    "BOWL_MIN_CELLS/BOWL_FOOT": {
        "source": "debug seeds 51,53,55,57: merged rim ring is 120-144 cells and "
                  "0.100-0.110 m across; stove knobs are <=45 cells",
        "allowed": True},
    "PLATE_BAND/PLATE_FOOT/PLATE_PALE": {
        "source": "debug seeds 51,53,55,57: plate top 0.0176 above the table, "
                  "0.130-0.140 m across, mean rgb 148; next-palest flat object 71",
        "allowed": True},
    "Z_BIAS": {"source": "debug seeds 51,57 v3 calibration sweep: free-space eef z "
                         "settles 0.0087-0.0113 m above the commanded z",
               "allowed": True},
    "FREE_SPOT": {"source": "debug seeds 51,53,55,57: nothing above the table within "
                            "0.07 m of (x=-0.08, y=+0.06)", "allowed": True},
    "GRASP_DEPTH": {"source": "debug-seed bowl geometry: rim top 0.0775 above the "
                              "table, so a 0.025 m bite stays inside the wall",
                    "allowed": True},
    "BOWL_R": {"source": "debug seeds 51,53,55,57: rim-ring bbox 0.105-0.110 m across",
               "allowed": True},
    "OFF_Y": {"source": "measured two independent ways on debug seeds: (a) v4 placed "
                        "with an assumed -0.055 offset and the bowl landed +0.014 in "
                        "y of the plate centre on 4/4 seeds -> true offset -0.041; "
                        "(b) v5 lift re-perception far-edge y_min + BOWL_R on 7 seeds "
                        "-> -0.041 to -0.046. The closing jaws drag the bowl ~0.013 m "
                        "in from the rim radius.", "allowed": True},
    "HANG_NOM/HANG_MIN": {
        "source": "debug seeds 51-65 odd: v4/v5 lift re-perception gives 0.0429-0.0475 "
                  "for a thin bite; for a firm bite the gripper body occludes the "
                  "base and it reads 0.023, but the v6 release stall height minus "
                  "the plate top gives the TRUE hang 0.045-0.049 on those same seeds",
        "allowed": True},
    "DROP": {"source": "debug seeds 55,57,59 (v6): releasing at plate+0.015+hang "
                       "stalled the arm against the plate and saturating xy dragged "
                       "the placement up to 0.019 m off centre; 0.020 clears it",
             "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
GRID_XL = (-0.50, 0.28)
GRID_YL = (-0.50, 0.50)
GRID_RES = 0.005
BOWL_BAND = (0.055, 0.115)
BOWL_MIN_CELLS = 60
BOWL_FOOT = (0.06, 0.16)
PLATE_BAND = (0.007, 0.021)
PLATE_FOOT = (0.10, 0.18)
PLATE_PALE = 110.0
Z_BIAS = 0.0103
FREE_SPOT = (-0.08, 0.06)
GRASP_DEPTH = 0.025
BOWL_R = 0.0545
OFF_Y = -0.041
HANG_NOM = 0.046
HANG_MIN = 0.045
DROP = 0.020
CHUNK = 1500


def _dump(api, tag, arr):
    raw = np.ascontiguousarray(arr).tobytes()
    b = base64.b64encode(zlib.compress(raw, 6)).decode("ascii")
    api.log("DUMP %s dtype=%s shape=%s nchunks=%d"
            % (tag, arr.dtype.str, tuple(arr.shape), (len(b) + CHUNK - 1) // CHUNK))
    for i in range(0, len(b), CHUNK):
        api.log("D %s %d %s" % (tag, i // CHUNK, b[i:i + CHUNK]))


# ------------------------------------------------------------------ geometry

def _cloud(api, cam="cam_high", step=2):
    f = api.capture(cam)
    rgb = np.asarray(f.rgb)[::step, ::step].astype(np.float32).reshape(-1, 3)
    dep = np.asarray(f.depth, dtype=np.float32)[::step, ::step]
    K = np.asarray(f.intrinsics)
    T = np.asarray(f.t_base_cam)
    h, w = dep.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu * step - K[0, 2]) / K[0, 0] * dep
    y = (vv * step - K[1, 2]) / K[1, 1] * dep
    pb = np.stack([x, y, dep], -1).reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]
    return pb, rgb


def _grid(pb, rgb):
    g = np.isfinite(pb).all(1)
    p, c = pb[g], rgb[g]
    nx = int((GRID_XL[1] - GRID_XL[0]) / GRID_RES)
    ny = int((GRID_YL[1] - GRID_YL[0]) / GRID_RES)
    ix = ((p[:, 0] - GRID_XL[0]) / GRID_RES).astype(int)
    iy = ((p[:, 1] - GRID_YL[0]) / GRID_RES).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    ix, iy, p, c = ix[ok], iy[ok], p[ok], c[ok]
    o = np.argsort(p[:, 2])
    ix, iy, p, c = ix[o], iy[o], p[o], c[o]
    H = np.full((nx, ny), -9.0)
    C = np.zeros((nx, ny, 3))
    H[ix, iy] = p[:, 2]
    C[ix, iy] = c
    return H, C


def _dilate(m, r):
    d = m.copy()
    for _ in range(r):
        e = d.copy()
        e[1:, :] |= d[:-1, :]
        e[:-1, :] |= d[1:, :]
        e[:, 1:] |= d[:, :-1]
        e[:, :-1] |= d[:, 1:]
        d = e
    return d


def _label8(mask):
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    H, W = mask.shape
    for sy, sx in np.argwhere(mask):
        if lab[sy, sx]:
            continue
        cur += 1
        stack = [(sy, sx)]
        lab[sy, sx] = cur
        while stack:
            y, x = stack.pop()
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < H and 0 <= nx < W and mask[ny, nx] and not lab[ny, nx]:
                        lab[ny, nx] = cur
                        stack.append((ny, nx))
    return lab, cur


def _bbox(k):
    ii, jj = np.where(k)
    return (GRID_XL[0] + ii.min() * GRID_RES, GRID_XL[0] + ii.max() * GRID_RES,
            GRID_YL[0] + jj.min() * GRID_RES, GRID_YL[0] + jj.max() * GRID_RES)


def perceive(api):
    pb, rgb = _cloud(api)
    H, C = _grid(pb, rgb)
    z = pb[np.isfinite(pb).all(1), 2]
    hist, edges = np.histogram(z[(z > 0.5) & (z < 2.0)], bins=np.arange(0.60, 1.60, 0.005))
    table = float(edges[int(np.argmax(hist))] + 0.0025)
    api.log("TABLE z=%.4f n=%d" % (table, hist.max()))

    # --- target bowl: only object topping out inside the elevated band
    m = (H > table + BOWL_BAND[0]) & (H < table + BOWL_BAND[1])
    lab, n = _label8(_dilate(m, 3))
    best = None
    for i in range(1, n + 1):
        k = m & (lab == i)
        if k.sum() < BOWL_MIN_CELLS:
            continue
        x0, x1, y0, y1 = _bbox(k)
        dx, dy = x1 - x0, y1 - y0
        api.log("CAND n=%d c=(%.4f,%.4f) d=%.3fx%.3f ztop=%.4f"
                % (k.sum(), (x0 + x1) / 2, (y0 + y1) / 2, dx, dy, H[k].max()))
        if not (BOWL_FOOT[0] <= dx <= BOWL_FOOT[1] and BOWL_FOOT[0] <= dy <= BOWL_FOOT[1]):
            continue
        if best is None or H[k].max() > best[0]:
            best = (float(H[k].max()), k, x0, x1, y0, y1)
    if best is None:
        api.log("NO_BOWL")
        return None
    ztop, k, x0, x1, y0, y1 = best
    bowl = dict(cx=(x0 + x1) / 2.0, cy=(y0 + y1) / 2.0, ztop=ztop,
                rx=(x1 - x0) / 2.0, ry=(y1 - y0) / 2.0)
    api.log("BOWL c=(%.4f,%.4f) ztop=%.4f (=table+%.4f) r=(%.4f,%.4f)"
            % (bowl["cx"], bowl["cy"], ztop, ztop - table, bowl["rx"], bowl["ry"]))

    # --- plate: the pale flat disc
    mp = (H > table + PLATE_BAND[0]) & (H < table + PLATE_BAND[1])
    lp, npq = _label8(mp)
    plate = None
    for i in range(1, npq + 1):
        k = lp == i
        if k.sum() < 100:
            continue
        x0, x1, y0, y1 = _bbox(k)
        dx, dy = x1 - x0, y1 - y0
        pale = float(C[k].mean())
        api.log("FLAT n=%d c=(%.4f,%.4f) d=%.3fx%.3f ztop=%.4f pale=%.0f"
                % (k.sum(), (x0 + x1) / 2, (y0 + y1) / 2, dx, dy, H[k].max(), pale))
        if (PLATE_FOOT[0] <= dx <= PLATE_FOOT[1] and PLATE_FOOT[0] <= dy <= PLATE_FOOT[1]
                and pale > PLATE_PALE):
            if plate is None or pale > plate["pale"]:
                plate = dict(cx=(x0 + x1) / 2.0, cy=(y0 + y1) / 2.0,
                             ztop=float(H[k].max()), rx=dx / 2.0, ry=dy / 2.0, pale=pale)
    if plate is None:
        api.log("NO_PLATE")
    else:
        api.log("PLATE c=(%.4f,%.4f) ztop=%.4f r=(%.4f,%.4f)"
                % (plate["cx"], plate["cy"], plate["ztop"], plate["rx"], plate["ry"]))
    return dict(table=table, bowl=bowl, plate=plate, H=H)


# -------------------------------------------------------------------- motion

def measure_held(api, e, table):
    """Re-perceive what hangs under the eef.  Returns (hang, off_y_est) or None.

    The z window reaches above the eef so a *firm* bite -- which holds the bowl
    high, close to the fingers -- is not mistaken for an empty gripper (that is
    what a narrower window did on debug seed 55).  The fingers are therefore
    inside the mask, but they are only ~0.03 m wide in x while the bowl is
    0.109 m, so the x extent separates "bowl" from "fingers only", and the
    fingertips sit above every part of the payload so zmin is still the bowl's
    own base.  The +y side of the bbox IS corrupted by the fingers, so the
    centre is estimated from the far (-y) edge plus the known outer radius.
    """
    pb, rgb = _cloud(api)
    _dump(api, "lift_rgb", rgb.reshape(256, 256, 3).astype(np.uint8))
    Z, X, Y = pb[:, 2], pb[:, 0], pb[:, 1]
    nb = np.isfinite(Z) & (np.abs(X - e[0]) < 0.10) & (np.abs(Y - e[1]) < 0.13) \
        & (Z > table + 0.06) & (Z < e[2] + 0.02)
    n = int(nb.sum())
    if n < 40:
        api.log("HELD_NONE n=%d" % n)
        return None
    x0, x1 = float(X[nb].min()), float(X[nb].max())
    y0, y1 = float(Y[nb].min()), float(Y[nb].max())
    hang = float(e[2] - Z[nb].min())
    off_y = y0 + BOWL_R - e[1]
    api.log("HELD n=%d zmin=%.4f x[%.3f,%.3f] y[%.3f,%.3f] d=%.3fx%.3f hang=%.4f "
            "off_y_est=%.4f" % (n, float(Z[nb].min()), x0, x1, y0, y1,
                                x1 - x0, y1 - y0, hang, off_y))
    if (x1 - x0) < 0.07:
        api.log("HELD_TOO_NARROW")
        return None
    return hang, off_y


class Mover(object):
    def __init__(self, api):
        self.api = api
        self.n = 0

    def go(self, xyz, seconds=2.0, tag="", bias=True):
        self.n += 1
        cmd = [xyz[0], xyz[1], xyz[2] - (Z_BIAS if bias else 0.0)]
        self.api.move(cmd, R_DOWN, seconds)
        e = np.asarray(self.api.eef(), dtype=float)
        g = self.api.gripper()
        self.api.log("MV%02d %-10s want=[%.4f,%.4f,%.4f] eef=[%.4f,%.4f,%.4f] "
                     "err=[%+.4f,%+.4f,%+.4f] w=%.4f ef=%.2f"
                     % (self.n, tag, xyz[0], xyz[1], xyz[2], e[0], e[1], e[2],
                        e[0] - xyz[0], e[1] - xyz[1], e[2] - xyz[2],
                        g["width_m"], g["effort"]))
        return e

    def go_exact(self, xyz, seconds=2.0, tag="", tol=0.004):
        e = self.go(xyz, seconds, tag)
        err = e - np.asarray(xyz, dtype=float)
        if np.abs(err).max() > tol:
            e = self.go(np.asarray(xyz, dtype=float) - err, max(1.2, seconds * 0.6),
                        tag + "'", bias=False)
        return e


def run(api):
    api.log("INSTRUCTION %r" % (api.instruction(),))
    s = perceive(api)
    if s is None:
        return
    table, bowl, plate = s["table"], s["bowl"], s["plate"]
    mv = Mover(api)
    api.grip(0.08)
    api.settle(0.2)

    # ---- fingertip offset: press the open jaws onto bare table
    fx, fy = FREE_SPOT
    mv.go((fx, fy, table + 0.14), 2.0, "cal_up")
    prev = None
    tip = None
    for zc in (0.03, 0.01, 0.00, -0.01, -0.02, -0.04):
        e = mv.go((fx, fy, table + zc), 1.5, "cal%+.2f" % zc)
        if prev is not None and (prev - e[2]) < 0.003:
            tip = float(e[2]) - table
            break
        prev = e[2]
    if tip is None:
        tip = float(api.eef()[2]) - table
    api.log("TIP_OFFSET %.4f" % tip)
    mv.go((fx, fy, table + 0.16), 2.0, "cal_out")

    # ---- rim pinch on the +y arc
    r = bowl["ry"]
    gx, gy = bowl["cx"], bowl["cy"] + r
    hover_z = bowl["ztop"] + 0.09 + tip
    lift_z = bowl["ztop"] + 0.12 + tip
    held = None
    for attempt in (1, 2):
        mv.go((gx, gy, hover_z), 2.5, "hover%d" % attempt)
        mv.go((gx, gy, hover_z - 0.05), 1.5, "predesc")
        grasp_z = bowl["ztop"] - GRASP_DEPTH + tip
        mv.go_exact((gx, gy, grasp_z), 1.5, "descend")
        api.grip(0.0)
        api.settle(0.6)
        g = api.gripper()
        api.log("CLOSED%d w=%.4f ef=%.2f" % (attempt, g["width_m"], g["effort"]))

        mv.go((gx, gy, lift_z), 2.0, "lift%d" % attempt)
        g = api.gripper()
        e = np.asarray(api.eef(), dtype=float)
        api.log("LIFTED%d w=%.4f ef=%.2f eef=%s"
                % (attempt, g["width_m"], g["effort"], e.round(4).tolist()))
        held = measure_held(api, e, table)
        # Two independent hold receipts.  The effort flag is a jaw-gap
        # threshold, so a thin rim bite reads "not holding" (w=0.005, ef=0.05)
        # while the bowl is plainly there; conversely a firm bite holds the
        # bowl high where re-perception is weakest.  Either one is enough.
        if held is not None or g["effort"] >= 3.0:
            break
        api.log("RETRY_PINCH")
        api.grip(0.08)
        api.settle(0.3)
        s2 = perceive(api)
        if s2 is not None and s2["bowl"] is not None:
            bowl = s2["bowl"]
            r = bowl["ry"]
            gx, gy = bowl["cx"], bowl["cy"] + r
            hover_z = bowl["ztop"] + 0.09 + tip
            lift_z = bowl["ztop"] + 0.12 + tip
    if held is None and g["effort"] < 3.0:
        api.log("NO_HOLD_GIVE_UP")
        return
    # The gripper body hides the bowl's base from cam_high whenever the bite
    # is firm enough to hold the bowl high, so the measured hang can be 0.022
    # short.  Floored: on every debug seed the arm stalled with the bowl on the
    # plate at a true hang of 0.045-0.049, in BOTH grip regimes.
    hang = max(held[0], HANG_MIN) if held is not None else HANG_NOM
    off_y_est = held[1] if held is not None else None

    # ---- carry so the BOWL, not the eef, lands on the plate centre
    if plate is None:
        api.log("ABORT_NO_PLATE")
        return
    # The lift estimate reads the bowl's FAR edge, which this camera sees
    # through the bowl's own near wall and so under-measures; OFF_Y comes from
    # the near edge and from v4's landing error, so it is the one used and the
    # estimate is logged only as a cross-check.
    off_y = OFF_Y
    api.log("OFFSET use=%.4f nominal=%.4f est=%s hang=%.4f"
            % (off_y, OFF_Y, "None" if off_y_est is None else round(off_y_est, 4), hang))
    px, py = plate["cx"], plate["cy"] - off_y
    mv.go((px, py, lift_z), 3.0, "carry")
    rel_z = plate["ztop"] + DROP + hang
    mv.go_exact((px, py, rel_z), 2.0, "lower")
    g = api.gripper()
    api.log("PRE_RELEASE w=%.4f ef=%.2f" % (g["width_m"], g["effort"]))
    api.grip(0.08)
    api.settle(0.6)
    mv.go((px, py, rel_z + 0.12), 2.0, "retreat")

    pb, rgb = _cloud(api)
    _dump(api, "final_rgb", rgb.reshape(256, 256, 3).astype(np.uint8))
    api.log("MOVES %d" % mv.n)
    api.log("DONE_V7")
