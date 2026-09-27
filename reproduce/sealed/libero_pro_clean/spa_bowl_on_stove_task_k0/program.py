"""c2clean spa_bowl_on_stove_task_k0 -- v7: the same motion, inside the step budget.

The one seed that kept failing (55) never had a grasp problem: v5/v6 both bit the
shell there (closed width 0.0051, effort 3.0) and carried the bowl to the plate.
It ran out of episode.  `move_cartesian` returns as soon as the residual drops
below POS_TOL, and otherwise runs `2 * 60 * seconds` steps; seed 55's residual
settles at 0.016-0.026, just *above* that tolerance, so each of its moves burned
its full budget -- three 2.5 s legs at 300 steps each and the 1000-step horizon was
gone before the release ever executed.  Seed 51, whose residual lands at 0.010,
finished the entire task in 205 steps.

v7 therefore treats `seconds` as the step budget it is and keeps every leg short
(1.0-1.2 s, corrections 0.6 s), and widens the correction-loop exit tolerance to
0.006 so a 5 mm miss does not buy a second full-budget move.  Nothing about the
grasp geometry changes.  That took the probe seeds to 8/8, seed 55 at 828 of its
1000 steps.

v8 buys back that margin.  Seed 55's first bite measured 0.0047 and triggered a
re-grasp it did not need -- the same 0.0050 bite had carried the bowl to the plate
in v6.  Across every version, a bite that holds reads 0.0047-0.0081 and jaws that
met only air read 0.0010, so the verify threshold moves to 0.003, between the two
populations instead of inside the holding one.

The grasp, established in v3-v5
------------------------------
* the eef reference sits 0.010 above the fingertips (a descent blocked on the
  cabinet top at 1.1355 with the top at 1.125, and one blocked on the bowl's
  cavity floor at 1.135).
* the bowl is a thin cone: outer radius 0.0546 at the rim (z 1.180) and 0.0436 at
  z 1.150, with the inner face ~0.009 inside, so the shell is a 7-10 mm plate
  tilted 24 deg off vertical and far too wide (0.110) to straddle with a 0.078
  jaw span.  The grasp is a rim pinch on the +y arc, fingertips 0.020 below the
  rim, aimed 0.007 inside the circle-fitted rim radius.
* v2 descended to a stall instead, which drove the inner fingertip into the
  cavity floor, let the eef drift 0.025 in +x, and closed at ~33 deg off the wall
  normal; the bowl escaped during every lift.
* the bite is legible in the gripper width: 0.0051-0.0081 with effort 3.0 when the
  jaws hold the shell, 0.0010 when they met only air.  v4 added the re-grasp that
  checks it; v5 replaced a bbox centre (which a clipped far edge had displaced
  0.008 on seed 55) with a circle fit to the rim ring, rms 0.0028.
"""
import numpy as np

PROVENANCE = {
    "WS_X": {"source": "debug 51-57 cam_high cloud extent", "allowed": True},
    "WS_Y": {"source": "debug 51-57 cam_high cloud extent", "allowed": True},
    "Z_BIN": {"source": "generic: 5 mm histogram bin for plane-mode finding", "allowed": True},
    "CAB_MIN_RISE": {"source": "debug 51-57: cabinet-top plane 1.125, table 0.903",
                     "allowed": True},
    "BOWL_CLEAR": {"source": "debug 51-57: bowl cells start >0.015 above the cabinet top",
                   "allowed": True},
    "PLATE_BAND": {"source": "debug 51-57: plate top = table+0.017; table bowl "
                             "top = table+0.041; stove slab = table+0.029", "allowed": True},
    "PLATE_MIN_BRIGHT": {"source": "debug 51-57 blob mean RGB: plate 145-147, cookie "
                                   "box 69, stove slab 63-86, dark bowl 19", "allowed": True},
    "GRID_RES": {"source": "generic: 4 mm top-down height-map cell", "allowed": True},
    "TIP_OFF": {"source": "debug 51 v1 (blocked on the cabinet top at eef 1.1355, top "
                          "1.125) and debug 51-57 v2 (blocked on the cavity floor)",
                "allowed": True},
    "PINCH_DEPTH/DEPTH_MIN/DEPTH_MAX/DEPTH_TARGET": {
        "source": "debug 51-65 v8/v9: bites that held had the fingertips 0.0199-0.0207 "
                  "below the rim; every air close was 0.0135-0.0183 (58,64) and the one "
                  "bite that rotated in the jaws mid-carry was 0.0264 (57 v9, width grew "
                  "0.0044->0.0170).  So the window is 0.019-0.025 and the descent is "
                  "measured and corrected into it from either side",
        "allowed": True},
    "PINCH_IN": {"source": "debug 51 cross-section: shell midline sits 0.011 inside the "
                           "outer rim edge at that depth", "allowed": True},
    "BIAS_TOL": {"source": "debug 51-57 v2: api.move left 0.010-0.025 of xy tracking "
                           "error at the bottom of the descent; 0.006 is inside the "
                           "controller's own 0.012 arrival tolerance", "allowed": True},
    "BOWL_H": {"source": "debug 51-57: rim 1.180 over a base on the cabinet top 1.127",
               "allowed": True},
    "PLACE_CLEAR": {"source": "generic: release clearance above the goal surface",
                    "allowed": True},
    "GRIP_BIT": {"source": "debug 51-65 v3-v7: closed width 0.0047-0.0081 with effort "
                           "3.0 on every bite that carried the bowl, 0.0010 on the one "
                           "that closed on air; 0.003 splits them", "allowed": True},
    "RETRY_DY/RETRY_DZ": {"source": "debug 64 v8: the retry that turned a 0.0013 air "
                                    "close into a 0.0051 bite moved the fingertips from "
                                    "0.0135 to 0.0199 deep", "allowed": True},
    "RING_BAND": {"source": "debug 51 cross-section: the rim annulus spans the top "
                            "0.010 of the bowl", "allowed": True},
    "RING_RMS": {"source": "debug 51/53 offline fit: circle residual 4 mm grid quantum",
                 "allowed": True},
    "RIM_R_RANGE": {"source": "debug 51/53 offline circle fit on the rim ring: "
                              "R = 0.0504/0.0505", "allowed": True},
    "PINCH_IN_FIT": {"source": "debug 51-65 v8: bites held at radial offsets from "
                               "R_fit-0.006 (seed 51) to R_fit-0.015 (seed 64 retry), so "
                               "the lateral window is wide; 0.011 centres it",
                     "allowed": True},
    "PLACE_STAGE": {"source": "generic: stage the descent so the xy correction is "
                              "applied before the last 0.14 of travel", "allowed": True},
    "DESC_BIAS": {"source": "debug 51/55/57 v4-v5: the hover->grasp descent lands at a "
                            "repeatable offset, err = (-0.0074..-0.0082, +0.0026.."
                            "+0.0037, -0.0063..-0.0077); pre-compensating it lets the "
                            "correction loop exit in one move", "allowed": True},
    "FIX_SECONDS/S_*": {"source": "generic controller mechanics: move_cartesian runs "
                                  "2*60*seconds steps unless the residual falls under "
                                  "POS_TOL, so `seconds` is a step budget; debug 55 "
                                  "v6 spent 300 steps per leg and died at the horizon",
                        "allowed": True},
}

WS_X = (-0.45, 0.30)
WS_Y = (-0.50, 0.50)
Z_BIN = 0.005
CAB_MIN_RISE = 0.15
BOWL_CLEAR = 0.015
PLATE_BAND = (0.010, 0.032)
PLATE_MIN_BRIGHT = 115.0
GRID_RES = 0.004
TIP_OFF = 0.010
PINCH_DEPTH = 0.022
PINCH_IN = 0.011
BIAS_TOL = 0.006
PLACE_CLEAR = 0.000
GRIP_BIT = 0.003
RETRY_DY = 0.006
RETRY_DZ = 0.000
DEPTH_MIN = 0.019
DEPTH_MAX = 0.025
DEPTH_TARGET = 0.022
RING_BAND = 0.010
RING_RMS = 0.004
RIM_R_RANGE = (0.040, 0.065)
PINCH_IN_FIT = 0.011
PLACE_STAGE = 0.14
DESC_BIAS = (-0.0077, 0.0032, -0.0050)
FIX_SECONDS = 0.6
S_HOVER = 1.2
S_DESC = 1.0
S_LIFT = 1.0
S_STAGE = 1.2
S_LOWER = 1.0


# ---------------------------------------------------------------- perception #
def _cloud(frame):
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    d = np.asarray(frame.depth, float)
    h, w = d.shape
    v, u = np.mgrid[0:h, 0:w]
    x = (u - K[0, 2]) * d / K[0, 0]
    y = (v - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1) @ T.T
    return P[..., :3], np.asarray(frame.rgb, float), np.isfinite(d) & (d > 0)


def _mode_z(z, lo, hi):
    if z.size == 0:
        return None
    h, e = np.histogram(z, bins=max(1, int(round((hi - lo) / Z_BIN))), range=(lo, hi))
    if h.max() == 0:
        return None
    i = int(np.argmax(h))
    return float(0.5 * (e[i] + e[i + 1]))


def _grid(pts, rgb, res=GRID_RES):
    nx = int((WS_X[1] - WS_X[0]) / res)
    ny = int((WS_Y[1] - WS_Y[0]) / res)
    H = np.full((nx, ny), -np.inf)
    C = np.zeros((nx, ny, 3))
    ix = ((pts[:, 0] - WS_X[0]) / res).astype(int)
    iy = ((pts[:, 1] - WS_Y[0]) / res).astype(int)
    m = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    ix, iy, pz, cc = ix[m], iy[m], pts[m, 2], rgb[m]
    o = np.argsort(pz)
    H[ix[o], iy[o]] = pz[o]
    C[ix[o], iy[o]] = cc[o]
    return H, C


def _label(mask):
    lab = np.zeros(mask.shape, int)
    cur = 0
    nx, ny = mask.shape
    for si in range(nx):
        for sj in range(ny):
            if not mask[si, sj] or lab[si, sj]:
                continue
            cur += 1
            stack = [(si, sj)]
            lab[si, sj] = cur
            while stack:
                i, j = stack.pop()
                for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    a, b = i + di, j + dj
                    if 0 <= a < nx and 0 <= b < ny and mask[a, b] and not lab[a, b]:
                        lab[a, b] = cur
                        stack.append((a, b))
    return lab, cur


def _blobs(H, C, mask, min_cells=60):
    lab, n = _label(mask)
    out = []
    for k in range(1, n + 1):
        m = lab == k
        if int(m.sum()) < min_cells:
            continue
        xi, yi = np.nonzero(m)
        x0, x1 = WS_X[0] + xi.min() * GRID_RES, WS_X[0] + xi.max() * GRID_RES
        y0, y1 = WS_Y[0] + yi.min() * GRID_RES, WS_Y[0] + yi.max() * GRID_RES
        out.append(dict(cells=int(m.sum()), x0=x0, x1=x1, y0=y0, y1=y1,
                        xc=0.5 * (x0 + x1), yc=0.5 * (y0 + y1),
                        ex=x1 - x0, ey=y1 - y0,
                        area=(x1 - x0 + GRID_RES) * (y1 - y0 + GRID_RES),
                        ztop=float(H[m].max()), bright=float(C[m].mean()),
                        _xy=(WS_X[0] + xi * GRID_RES, WS_Y[0] + yi * GRID_RES),
                        _z=H[m].copy()))
    return out


def _kasa(x, y):
    """Algebraic circle fit; returns (cx, cy, R, rms) or None."""
    if x.size < 30:
        return None
    A = np.c_[x, y, np.ones(x.size)]
    b = x ** 2 + y ** 2
    try:
        c, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy = 0.5 * c[0], 0.5 * c[1]
    v = c[2] + cx ** 2 + cy ** 2
    if v <= 0:
        return None
    R = float(np.sqrt(v))
    rms = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - R) ** 2)))
    return float(cx), float(cy), R, rms


def _r(d):
    return None if d is None else {k: (round(v, 4) if isinstance(v, float) else v)
                                   for k, v in d.items() if not k.startswith("_")}


def perceive(api, tag=""):
    f = api.capture("cam_high")
    P, RGB, ok = _cloud(f)
    pts, rgb = P[ok], RGB[ok]
    m = ((pts[:, 0] > WS_X[0]) & (pts[:, 0] < WS_X[1]) &
         (pts[:, 1] > WS_Y[0]) & (pts[:, 1] < WS_Y[1]))
    pts, rgb = pts[m], rgb[m]
    H, C = _grid(pts, rgb)

    z_table = _mode_z(pts[:, 2], 0.70, 1.05)
    z_cab = _mode_z(pts[:, 2][pts[:, 2] > z_table + CAB_MIN_RISE],
                    z_table + CAB_MIN_RISE, z_table + 0.45)

    cbl = sorted(_blobs(H, C, np.isfinite(H) & (np.abs(H - z_cab) < 0.012),
                        min_cells=200), key=lambda b: -b["cells"])
    cab = cbl[0] if cbl else None

    up = np.isfinite(H) & (H > z_cab + BOWL_CLEAR) & (H < z_cab + 0.18)
    if cab is not None:
        gx = WS_X[0] + np.arange(H.shape[0]) * GRID_RES
        gy = WS_Y[0] + np.arange(H.shape[1]) * GRID_RES
        up &= ((gx >= cab["x0"] - 0.01) & (gx <= cab["x1"] + 0.01))[:, None]
        up &= ((gy >= cab["y0"] - 0.01) & (gy <= cab["y1"] + 0.01))[None, :]
    bb = sorted(_blobs(H, C, up), key=lambda b: -b["cells"])
    bowl = bb[0] if bb else None
    if bowl is not None:
        # the rim is the highest ring of the blob; fit a circle to it.  This is
        # immune to a bbox edge being clipped, which is what cost seed 55.
        bxs, bys = bowl["_xy"]
        ring = bowl["_z"] > bowl["ztop"] - RING_BAND
        fit = _kasa(bxs[ring], bys[ring])
        api.log(f"PERC{tag} ring n={int(ring.sum())} kasa={None if fit is None else [round(v,4) for v in fit]}")
        if fit is not None and RIM_R_RANGE[0] < fit[2] < RIM_R_RANGE[1] and fit[3] < RING_RMS:
            bowl["fx"], bowl["fy"], bowl["fR"] = fit[0], fit[1], fit[2]

    low = np.isfinite(H) & (H > z_table + PLATE_BAND[0]) & (H < z_table + PLATE_BAND[1])
    cand = [b for b in _blobs(H, C, low) if b["bright"] > PLATE_MIN_BRIGHT]
    plate = max(cand, key=lambda b: b["area"]) if cand else None

    api.log(f"PERC{tag} z_table={z_table:.3f} z_cab={z_cab:.3f} cab={_r(cab)}")
    api.log(f"PERC{tag} bowl={_r(bowl)}")
    api.log(f"PERC{tag} plate={_r(plate)}")
    return dict(z_table=z_table, z_cab=z_cab, cab=cab, bowl=bowl, plate=plate)


# -------------------------------------------------------------------- motion #
def _st(api, tag):
    e = np.asarray(api.eef(), float)
    g = api.gripper()
    api.log(f"ST {tag} eef={e.round(4).tolist()} w={round(g['width_m'],4)} "
            f"eff={round(g['effort'],2)}")
    return e, g


def goto(api, target, seconds=3.0, fix=1, tag="", pre=None, fix_seconds=None):
    """Move, then re-issue with the tracking error folded into the command.

    `pre` is a known, repeatable tracking bias for this leg, added to the first
    command so the correction loop usually exits after a single move.

    `seconds` is a step BUDGET, not a duration: a move that converges returns at
    once, and one that does not runs the whole budget, so every leg here is
    deliberately short.
    """
    t = np.asarray(target, float)
    cmd = t.copy() if pre is None else t + np.asarray(pre, float)
    e = None
    for k in range(fix + 1):
        r = api.move(cmd, seconds=(seconds if k == 0 else (fix_seconds or FIX_SECONDS)))
        e = np.asarray(api.eef(), float)
        err = t - e
        api.log(f"GO {tag}.{k} cmd={cmd.round(4).tolist()} eef={e.round(4).tolist()} "
                f"err={err.round(4).tolist()} res={r:.4f}")
        if np.linalg.norm(err) < BIAS_TOL or k == fix:
            break
        cmd = cmd + err
    return e


def run(api):
    api.log(f"INSTR {api.instruction()}")
    s = perceive(api)
    bowl, plate = s["bowl"], s["plate"]
    if bowl is None or plate is None:
        api.log("ABORT perception")
        return
    rim = bowl["ztop"]
    # the camera sits at +x,+y of the bowl, so x1/y1 are the unoccluded bbox
    # edges and either extent can be truncated on its far side; take the radius
    # from the larger extent and the centre from the near edges.
    if "fR" in bowl:
        bx, by, R, inset = bowl["fx"], bowl["fy"], bowl["fR"], PINCH_IN_FIT
    else:
        R = 0.5 * max(bowl["ex"], bowl["ey"])
        bx, by, inset = bowl["x1"] - R, bowl["y1"] - R, PINCH_IN
    # pinch the +y arc: the shell midline at the chosen depth sits `inset` inside
    # the outer rim edge, and the rim is tangent to x there, so the grasp is
    # insensitive to the x centre.
    gx = bx
    gy = by + R - inset
    gz = rim - PINCH_DEPTH + TIP_OFF
    base_below_eef = gz - s["z_cab"]          # bowl base relative to the eef
    api.log(f"GEO bowl=({bx:.3f},{by:.3f}) R={R:.3f} rim={rim:.3f} grasp=({gx:.3f},"
            f"{gy:.3f},{gz:.3f}) base_off={base_below_eef:.3f} fitted={'fR' in bowl}")
    _st(api, "home")

    api.grip(0.08)
    goto(api, [gx, gy, rim + 0.07], seconds=S_HOVER, fix=0, tag="hover")
    # close, and check the bite by its width before committing to the carry: a
    # bite on the shell reads 0.0047-0.0081, jaws that met only air read 0.0010.
    for attempt in range(2):
        ay = gy - attempt * RETRY_DY
        if attempt:
            api.grip(0.08)
            goto(api, [gx, ay, rim + 0.05], seconds=S_DESC, fix=0, tag=f"reopen{attempt}")
        az = gz - attempt * RETRY_DZ
        e = goto(api, [gx, ay, az], seconds=S_DESC, fix=1, tag=f"desc{attempt}",
                 pre=DESC_BIAS)
        # depth, not lateral aim, decides whether the jaws straddle the shell or
        # skate over the lip -- and the descent undershoots by a seed-dependent
        # 0.004-0.010.  Measure what we actually got and top it up.
        depth = rim - (e[2] - TIP_OFF)
        api.log(f"DEPTH attempt={attempt} got={depth:.4f}")
        if not (DEPTH_MIN <= depth <= DEPTH_MAX):
            e = goto(api, [gx, ay, e[2] - (DEPTH_TARGET - depth)], seconds=FIX_SECONDS,
                     fix=0, tag=f"redepth{attempt}")
            api.log(f"DEPTH attempt={attempt} corrected={rim - (e[2] - TIP_OFF):.4f}")
        api.grip(0.0)
        api.settle(0.4)
        _, g = _st(api, f"closed{attempt}")
        if g["width_m"] >= GRIP_BIT:
            gy = ay
            break
        api.log(f"REGRASP attempt={attempt} w={g['width_m']:.4f} below {GRIP_BIT}")

    goto(api, [gx, gy, rim + 0.10], seconds=S_LIFT, fix=0, tag="lift")
    e, g = _st(api, "lifted")

    # --- carry and place ---------------------------------------------------- #
    # the bowl centre trails the eef by the pinch offset actually used, which the
    # re-grasp may have changed.
    carry_y_off = gy - by
    px, py = plate["xc"], plate["yc"]
    tx, ty = px, py + carry_y_off
    zc = plate["ztop"] + base_below_eef + PLACE_CLEAR
    api.log(f"PLACE target=({tx:.3f},{ty:.3f}) carry_off={carry_y_off:.3f} "
            f"drop_z={zc:.3f} plate_top={plate['ztop']:.3f}")
    # one diagonal leg to a staging point above the plate: settling xy there,
    # BEFORE the last stretch of descent, is what keeps the bowl off the plate's
    # raised rim -- a long vertical move drags xy by ~0.010.
    goto(api, [tx, ty, plate["ztop"] + PLACE_STAGE], seconds=S_STAGE, fix=1, tag="stage")
    _st(api, "staged")
    goto(api, [tx, ty, zc], seconds=S_LOWER, fix=1, tag="lower")
    e, g = _st(api, "lowered")
    api.grip(0.08)
    api.settle(0.4)
    _st(api, "released")
    api.move([tx, ty, zc + 0.10], seconds=0.6)
    _st(api, "retreat")
    api.log("DONE v10")
