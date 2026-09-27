"""Pick the akita black bowl standing on the stove and place it on the plate.

MECHANISM (every number below is measured, see PROVENANCE)

The akita bowl is ~0.11 m across at every height and the jaws open to 0.0778 m,
so the gripper cannot straddle it.  Both demo packs close to a finger gap of
~0.0086 m, which is a wall thickness, so the grasp is a WALL PINCH: one jaw
inside the cavity, one outside, the tool centred on the bowl wall about a
rim-radius off the bowl centre along the jaw axis.  The jaw axis is base-y (the
two finger blobs sit at y = +-0.049 with the tool straight down), so the pinch
offset is a signed y offset.  v4 measured this directly on debug seeds:
+-0.050 both give 4/4, a centred descent is blocked by the bowl (residual
0.026-0.032) and closes on nothing (gap 0.002, effort 0.05) for 0/4.

Grasp height: the k1 demo closes on a table-standing bowl at h=0.019 above the
table, the mate demo closes on the stove-standing bowl at h=0.048-0.051; the
0.030 difference is exactly the stove slab height measured in the v3 probe.  So
grasp_h = bowl_base_h + 0.019, with bowl_base_h = h_top - 0.051.

The stove bowl is found by a height band, not by colour: its rim top sits at
h=0.079-0.082 because the stove lifts it 0.030 above the table, while every
table-standing bowl tops out at h=0.051 and the plate and cookie box at h=0.020.
The band 0.058..0.105 over the stove region therefore isolates it uniquely.

Verification is by the program's own sensors only: the descend residual says
whether the approach column was blocked, and the finger gap plus effort sampled
immediately after the close says whether a wall is actually pinched.  A rung
that fails either test is retried on the other side of the bowl.
"""
import numpy as np

PROVENANCE = {
    "WS_X": {"source": "debug-seed measurement (v1/v2 probes ep51/53/57): the table surface spans x -0.36..0.32 in base frame", "allowed": True},
    "WS_Y": {"source": "debug-seed measurement (v1/v2 probes ep51/53/57): the table surface spans y -0.36..0.36 in base frame", "allowed": True},
    "STOVE_X": {"source": "debug-seed measurement (v3 probe ep51/57): the stove slab occupies x -0.37..-0.17 and the cabinet only starts at x>-0.14", "allowed": True},
    "STOVE_Y": {"source": "debug-seed measurement (v3 probe ep51/57): the stove slab occupies y -0.23..-0.05", "allowed": True},
    "STOVE_BAND": {"source": "debug-seed measurement (v3 probe ep51/57): the bowl on the stove has rim top h=0.079-0.082 and cavity floor h=0.040; every table-standing bowl tops out at h=0.051", "allowed": True},
    "PLATE_BAND": {"source": "debug-seed measurement (v2 probe ep51/53/57): the plate tops out at h=0.019-0.020", "allowed": True},
    "PLATE_X": {"source": "debug-seed measurement (v2 probe ep51/53/57): the plate sits at x 0.00..0.14", "allowed": True},
    "PLATE_Y": {"source": "debug-seed measurement (v2 probe ep51/53/57): the plate sits at y 0.13..0.28", "allowed": True},
    "PLATE_DIA": {"source": "debug-seed measurement (v2/v4 probes): the plate blob bbox is 0.134 x 0.136", "allowed": True},
    "BOWL_HEIGHT": {"source": "debug-seed measurement (v2 probe ep51/53/57): a table-standing bowl's top is h=0.051 above its base", "allowed": True},
    "GRASP_H_ABOVE_BASE": {"source": "pack keyframes: the k1 demo closes at ee z=0.919 (h=0.019 above the table) on a table bowl and the mate demo at ee z=0.951 (h=0.051) on the stove bowl, whose base is the 0.030 stove slab measured in the v3 probe", "allowed": True},
    "PINCH_RUNGS": {"source": "debug-seed measurement (v4a/v4b/v4c on ep51,53,57,61): pinch offset +0.050 gives 4/4, -0.050 gives 4/4, 0.000 gives 0/4", "allowed": True},
    "APPROACH_CLEAR": {"source": "pack mate ee_path6: the demo stands 0.023 above the rim before the final descent; 0.075 is a conservative clearance above the measured rim top", "allowed": True},
    "CARRY_H": {"source": "pack mate ee_path6: the demo transports the bowl at h=0.237..0.247", "allowed": True},
    "RELEASE_H": {"source": "pack mate ee_path6: the demo opens the gripper at h=0.030 over the plate", "allowed": True},
    "BLOCKED_RES": {"source": "debug-seed measurement (v4a vs v4c): a clear descent lands with residual 0.008-0.012, a descent blocked by the bowl body stalls at 0.026-0.032", "allowed": True},
    "HOLD_GAP_MIN": {"source": "debug-seed measurement (v4a vs v4c): a wall pinch reads finger gap 0.0122-0.0153 with effort 3.0 right after the close, a failed close reads 0.0019-0.0020 with effort 0.05", "allowed": True},
}

WS_X = (-0.36, 0.32)
WS_Y = (-0.36, 0.36)
STOVE_X = (-0.42, -0.15)
STOVE_Y = (-0.34, 0.02)
STOVE_BAND = (0.058, 0.105)
PLATE_BAND = (0.010, 0.030)
PLATE_X = (-0.10, 0.32)
PLATE_Y = (0.05, 0.36)
PLATE_DIA = (0.09, 0.19)
BOWL_HEIGHT = 0.051
GRASP_H_ABOVE_BASE = 0.019
PINCH_RUNGS = (0.050, -0.050, 0.044)
APPROACH_CLEAR = 0.075
CARRY_H = 0.245
RELEASE_H = 0.032
BLOCKED_RES = 0.018
HOLD_GAP_MIN = 0.005

# ---------------------------------------------------------------- perception


def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.asarray(f.depth, float)
    H, W = d.shape[:2]
    vv, uu = np.mgrid[0:H, 0:W]
    good = np.isfinite(d) & (d > 0)
    z = np.where(good, d, 1.0)
    xc = (uu - K[0, 2]) * z / K[0, 0]
    yc = (vv - K[1, 2]) * z / K[1, 1]
    pb = np.stack([xc, yc, z, np.ones_like(z)], -1) @ T.T
    return pb[..., 0], pb[..., 1], pb[..., 2], good


def label(mask):
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    seen = np.zeros((H, W), bool)
    cur = 0
    for sy, sx in np.argwhere(mask):
        if seen[sy, sx]:
            continue
        cur += 1
        stack = [(sy, sx)]
        seen[sy, sx] = True
        while stack:
            y, x = stack.pop()
            lab[y, x] = cur
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < H and 0 <= nx < W and mask[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    stack.append((ny, nx))
    return lab, cur


def kasa(px, py):
    """Least-squares circle through the points; None if ill-conditioned."""
    if px.size < 12:
        return None
    A = np.stack([2 * px, 2 * py, np.ones_like(px)], 1)
    b = px * px + py * py
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    a, c, d = sol
    rr = d + a * a + c * c
    if not np.isfinite(rr) or rr <= 0:
        return None
    return float(a), float(c), float(np.sqrt(rr))


def blobs(X, Y, h, good, xr, yr, band, min_area=25):
    m = (good & (X > xr[0]) & (X < xr[1]) & (Y > yr[0]) & (Y < yr[1])
         & (h > band[0]) & (h < band[1]))
    lab, n = label(m[::2, ::2])
    out = []
    for c in range(1, n + 1):
        sel = lab == c
        area = int(sel.sum())
        if area < min_area:
            continue
        ys, xs = np.nonzero(sel)
        fy, fx = ys * 2, xs * 2
        bx, by, bh = X[fy, fx], Y[fy, fx], h[fy, fx]
        x0, x1 = float(bx.min()), float(bx.max())
        y0, y1 = float(by.min()), float(by.max())
        fit = kasa(bx, by)
        out.append({"n": area, "cx": 0.5 * (x0 + x1), "cy": 0.5 * (y0 + y1),
                    "top": float(np.percentile(bh, 98)),
                    "x0": x0, "x1": x1, "y0": y0, "y1": y1,
                    "dx": x1 - x0, "dy": y1 - y0,
                    "kx": None if fit is None else fit[0],
                    "ky": None if fit is None else fit[1],
                    "kr": None if fit is None else fit[2]})
    out.sort(key=lambda b: -b["n"])
    return out


def perceive(api):
    X, Y, Z, good = cloud(api)
    inws = good & (X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1])
    hist, edges = np.histogram(Z[inws], bins=400, range=(0.7, 1.1))
    tz = float(edges[int(np.argmax(hist))] + 0.0005)
    h = Z - tz
    return {"tz": tz,
            "stove": blobs(X, Y, h, good, STOVE_X, STOVE_Y, STOVE_BAND, 30),
            "plate": blobs(X, Y, h, good, PLATE_X, PLATE_Y, PLATE_BAND, 60)}


def show(api, tag, bs):
    for b in bs[:3]:
        api.log("%s n=%d c=(%.3f,%.3f) top=%.3f d=(%.3f,%.3f) kasa=%s"
                % (tag, b["n"], b["cx"], b["cy"], b["top"], b["dx"], b["dy"],
                   "none" if b["kr"] is None else "(%.3f,%.3f,r%.3f)" % (b["kx"], b["ky"], b["kr"])))


def pick_plate(bs):
    """Largest blob whose bbox is plate-sized and roughly circular."""
    for b in bs:
        if (PLATE_DIA[0] < b["dx"] < PLATE_DIA[1]
                and PLATE_DIA[0] < b["dy"] < PLATE_DIA[1]
                and abs(b["dx"] - b["dy"]) < 0.05):
            return b
    return bs[0] if bs else None


# ------------------------------------------------------------------- program


def run(api):
    api.log("instruction=%r" % api.instruction())
    sc = perceive(api)
    tz = sc["tz"]
    api.log("table_z=%.4f" % tz)
    show(api, "stove", sc["stove"])
    show(api, "plate", sc["plate"])

    plate = pick_plate(sc["plate"])
    if not sc["stove"] or plate is None:
        api.log("ABORT: stove bowl=%d plate=%s" % (len(sc["stove"]), plate is not None))
        return
    bowl = sc["stove"][0]
    base_h = bowl["top"] - BOWL_HEIGHT
    gz = tz + base_h + GRASP_H_ABOVE_BASE
    hover_z = tz + bowl["top"] + APPROACH_CLEAR
    api.log("bowl c=(%.3f,%.3f) top=%.3f base=%.3f gz=%.4f | plate c=(%.3f,%.3f) d=(%.3f,%.3f)"
            % (bowl["cx"], bowl["cy"], bowl["top"], base_h, gz,
               plate["cx"], plate["cy"], plate["dx"], plate["dy"]))

    held_dy = None
    for dy in PINCH_RUNGS:
        gx, gy = bowl["cx"], bowl["cy"] + dy
        api.grip(0.08)
        api.move([gx, gy, hover_z], seconds=1.2)
        res = api.move([gx, gy, gz], seconds=1.2)
        if res > BLOCKED_RES:
            api.log("rung dy=%+.3f BLOCKED res=%.4f eef=%s"
                    % (dy, res, np.round(api.eef(), 4).tolist()))
            continue
        api.grip(0.0)
        g = api.gripper()
        api.log("rung dy=%+.3f res=%.4f close gap=%.4f effort=%.2f"
                % (dy, res, g["width_m"], g["effort"]))
        if g["width_m"] > HOLD_GAP_MIN and g["effort"] > 2.5:
            held_dy = dy
            break
    if held_dy is None:
        api.log("NO HOLD after %d rungs" % len(PINCH_RUNGS))
        return

    gx, gy = bowl["cx"], bowl["cy"] + held_dy
    api.move([gx, gy, tz + CARRY_H], seconds=1.5)
    api.log("lifted eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))

    # the pinch holds the bowl at a fixed offset from the tool, so the tool has
    # to stand that same offset off the plate centre for the bowl to land on it
    px, py = plate["cx"], plate["cy"] + held_dy
    api.move([px, py, tz + CARRY_H], seconds=1.8)
    api.move([px, py, tz + RELEASE_H + 0.06], seconds=1.2)
    res = api.move([px, py, tz + RELEASE_H], seconds=1.2)
    api.log("over plate res=%.4f eef=%s grip=%s"
            % (res, np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(0.08)
    api.settle(0.3)
    api.move([px, py, tz + 0.18], seconds=1.0)

    post = perceive(api)
    api.log("POST table_z=%.4f" % post["tz"])
    show(api, "POSTstove", post["stove"])
    show(api, "POSTplate", post["plate"])
    api.log("DONE held_dy=%+.3f" % held_dy)
