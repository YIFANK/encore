"""c2clean spa_bowl_on_ramekin_task_k3 -- v4

Intent: "Pick the akita black bowl on the cookie box and place it on the plate".

Scene (debug seeds 51-65, own cam_high RGB-D): two identical-looking bowls.
One rests on a red/white checkered box (rim top 0.970), one on a grey ramekin
(rim top 1.000).  The instruction names the one on the COOKIE BOX -> the
checkered support.  Support colour (redness) is the identity cue; rim-top
height is the tie-break.

Grasp: bowl outer diameter ~0.116 m exceeds the max jaw opening (0.078 m), so
the rim wall is pinched: eef offset one rim radius along -y (the jaws separate
along base y under the straight-down wrist), at a depth just under the rim top.
Both packs close the gripper that way.
"""
import json
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-65 cam_high depth: modal workspace height 0.901 m (recomputed per episode at runtime)", "allowed": True},
    "GRID": {"source": "generic: 5 mm top-down occupancy grid", "allowed": True},
    "BOWL_BAND": {"source": "debug seeds 51-65: bowl rim tops measured at 0.970 (cookie box) and 1.000 (ramekin); band [0.945,1.020] brackets both", "allowed": True},
    "PLATE_SHAPE": {"source": "debug seeds 51-65: plate ring r=0.062, footprint/circle fill 0.74, radial residual std 0.0042; the rectangular stove base scores fill 1.64 / residual 0.025", "allowed": True},
    "PLATE_BAND": {"source": "debug seeds 51-65: plate rim top 0.920, table 0.901; band [0.910,0.938]", "allowed": True},
    "FURNITURE_Z": {"source": "debug seeds 51-65: cabinet top 1.128, robot column >1.25; 1.020 ceiling removes both", "allowed": True},
    "RED_BAND_DZ": {"source": "debug seeds 51-65: support pixels 0.045 m below each rim top; checkered box redness 0.45 vs ramekin 0.33", "allowed": True},
    "PINCH_R": {"source": "debug seeds 51-65: Kasa fit of the rim ring = 0.053 m; matches k3 pack eef-to-bowl-centre offset ~0.05 m at gripper close", "allowed": True},
    "GRASP_DZ": {"source": "mate pack demo0 t0055 (gripper closed, z=0.953) vs bowl-on-cookie-box rim top 0.970 -> -0.017 m", "allowed": True},
    "HANG": {"source": "debug seeds 51-65: bowl interior floor 0.927, outer base ~0.920; with eef at rim_top-0.017 the base hangs ~0.035 m below the eef", "allowed": True},
    "CARRY_Z": {"source": "debug seeds 51-65: tallest obstacle between bowl and plate is the ramekin bowl rim at 1.001; carry at 1.10", "allowed": True},
    "PLACE_CLEAR": {"source": "debug seeds 51-65: plate rim top 0.920; release with the bowl base ~0.008 m above it", "allowed": True},
}

TABLE_FALLBACK = 0.901
GS = 0.005
XLO, XHI, YLO, YHI = -0.45, 0.40, -0.40, 0.40
FURNITURE_Z = 1.020
BOWL_LO, BOWL_HI = 0.945, 1.020
PLATE_LO, PLATE_HI = 0.910, 0.938
RED_BAND_DZ = 0.045
PINCH_R = 0.053
GRASP_DZ = -0.017
HANG = 0.035
CARRY_Z = 1.10
PLACE_CLEAR = 0.008
PLATE_R_LO, PLATE_R_HI = 0.045, 0.085
FILL_MAX = 1.10
RESID_MAX = 0.008
PLATE_TOP_MAX = 0.930


# ---------------------------------------------------------------- perception
def _cloud(fr):
    d = np.asarray(fr.depth, dtype=np.float64)
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    x = (u - K[0, 2]) / K[0, 0] * d
    y = (v - K[1, 2]) / K[1, 1] * d
    P = np.stack([x, y, d], -1)
    return P @ T[:3, :3].T + T[:3, 3]


def _grid(B):
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    m = (X > XLO) & (X < XHI) & (Y > YLO) & (Y < YHI) & np.isfinite(Z)
    nx = int((XHI - XLO) / GS) + 1
    ny = int((YHI - YLO) / GS) + 1
    ix = ((X - XLO) / GS).astype(np.int64)
    iy = ((Y - YLO) / GS).astype(np.int64)
    hm = np.zeros((nx, ny))
    np.maximum.at(hm, (ix[m], iy[m]), Z[m])
    return hm, nx, ny


def _components(mask):
    """4/8-connected labelling of a boolean grid, pure numpy/python."""
    lab = np.zeros(mask.shape, np.int64)
    n = 0
    idx = np.argwhere(mask)
    for i0, j0 in idx:
        if lab[i0, j0]:
            continue
        n += 1
        stack = [(i0, j0)]
        lab[i0, j0] = n
        while stack:
            a, b = stack.pop()
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    p, q = a + da, b + db
                    if 0 <= p < mask.shape[0] and 0 <= q < mask.shape[1] \
                            and mask[p, q] and not lab[p, q]:
                        lab[p, q] = n
                        stack.append((p, q))
    return lab, n


def _kasa(x, y):
    A = np.c_[x, y, np.ones(len(x))]
    b = x ** 2 + y ** 2
    c = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = c[0] / 2.0, c[1] / 2.0
    return cx, cy, float(np.sqrt(max(c[2] + cx ** 2 + cy ** 2, 1e-9)))


def _ring_fit(B, cellmask, zlo):
    """Centre/radius/top of one component, using EXACT cell membership.

    v2 used a rectangular 3D box around the component and leaked neighbouring
    props (the robot column) into the fit.
    """
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    inb = (X > XLO) & (X < XHI) & (Y > YLO) & (Y < YHI) & np.isfinite(Z)
    ix = np.clip(((X - XLO) / GS).astype(np.int64), 0, cellmask.shape[0] - 1)
    iy = np.clip(((Y - YLO) / GS).astype(np.int64), 0, cellmask.shape[1] - 1)
    sel = inb & cellmask[ix, iy] & (Z > zlo)
    if sel.sum() < 30:
        return None
    ztop = float(np.percentile(Z[sel], 99.0))
    ring = sel & (Z > ztop - 0.008)
    if ring.sum() < 20:
        return None
    cx, cy, r = _kasa(X[ring], Y[ring])
    return cx, cy, r, ztop


def perceive(api):
    fr = api.capture("cam_high")
    B = _cloud(fr)
    rgb = np.asarray(fr.rgb, dtype=np.float64)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    hm, nx, ny = _grid(B)

    inb = (X > XLO) & (X < XHI) & (Y > YLO) & (Y < YHI)
    zz = Z[inb]
    hist, edges = np.histogram(zz[(zz > 0.80) & (zz < 1.05)], bins=100)
    table = float(edges[int(np.argmax(hist))]) if hist.sum() else TABLE_FALLBACK
    api.log("PERC table=%.4f" % table)

    tall = hm > FURNITURE_Z
    # a tall column poisons its whole footprint; keep it out of every band
    bowl_mask = (hm > BOWL_LO) & (hm < BOWL_HI) & (~tall)
    lab, n = _components(bowl_mask)
    bowls = []
    for k in range(1, n + 1):
        cm = lab == k
        if cm.sum() < 60:
            continue
        fit = _ring_fit(B, cm, BOWL_LO)
        if fit is None:
            continue
        cx, cy, r, ztop = fit
        if not (0.035 < r < 0.075) or not (BOWL_LO < ztop < BOWL_HI):
            api.log("PERC reject bowl c=(%.3f,%.3f) r=%.3f ztop=%.3f" % (cx, cy, r, ztop))
            continue
        d = np.hypot(X - cx, Y - cy)
        sup = (d < r + 0.020) & (Z > table + 0.004) & (Z < ztop - RED_BAND_DZ)
        if sup.sum() < 40:
            red = 0.0
        else:
            c = rgb[sup]
            red = float(np.mean(c[:, 0] / (c.sum(1) + 1e-6)))
        bowls.append(dict(cx=cx, cy=cy, r=r, ztop=ztop, red=red, n=int(cm.sum())))
        api.log("PERC bowl c=(%.3f,%.3f) r=%.3f ztop=%.3f red=%.3f cells=%d"
                % (cx, cy, r, ztop, red, cm.sum()))

    plate_mask = (hm > PLATE_LO) & (hm < PLATE_HI) & (~tall)
    lab, n = _components(plate_mask)
    plate = None
    best = 0
    for k in range(1, n + 1):
        cm = lab == k
        if cm.sum() < 120:
            continue
        fit = _ring_fit(B, cm, PLATE_LO)
        if fit is None:
            continue
        cx, cy, r, ztop = fit
        area = cm.sum() * GS * GS
        fill = area / (np.pi * r * r)
        ix = np.clip(((X - XLO) / GS).astype(np.int64), 0, cm.shape[0] - 1)
        iy = np.clip(((Y - YLO) / GS).astype(np.int64), 0, cm.shape[1] - 1)
        ring = cm[ix, iy] & (Z > ztop - 0.008) & np.isfinite(Z)
        resid = float(np.hypot(X[ring] - cx, Y[ring] - cy).std()) if ring.sum() > 20 else 9.9
        api.log("PERC cand plate c=(%.3f,%.3f) r=%.3f ztop=%.3f n=%d fill=%.2f res=%.4f"
                % (cx, cy, r, ztop, cm.sum(), fill, resid))
        # a disc cannot cover more area than its own circle, and its rim points
        # must sit at a constant radius; the rectangular stove base fails both.
        if not (PLATE_R_LO < r < PLATE_R_HI):
            continue
        if fill > FILL_MAX or resid > RESID_MAX:
            continue
        if ztop > PLATE_TOP_MAX:
            continue
        if cm.sum() > best:
            best = int(cm.sum())
            plate = dict(cx=cx, cy=cy, r=r, ztop=ztop, n=best)
    if plate:
        api.log("PERC plate c=(%.3f,%.3f) r=%.3f ztop=%.3f cells=%d"
                % (plate["cx"], plate["cy"], plate["r"], plate["ztop"], plate["n"]))
    return table, bowls, plate


# ---------------------------------------------------------------------- main
def run(api):
    api.log("INSTR %s" % api.instruction())
    table, bowls, plate = perceive(api)
    if not bowls:
        api.log("ABORT no bowl found")
        return
    # the cookie box is the warm/checkered support; the ramekin is grey.
    target = max(bowls, key=lambda b: b["red"])
    if len(bowls) > 1:
        other = min(bowls, key=lambda b: b["red"])
        api.log("SELECT target red=%.3f ztop=%.3f | other red=%.3f ztop=%.3f"
                % (target["red"], target["ztop"], other["red"], other["ztop"]))
    cx, cy, ztop = target["cx"], target["cy"], target["ztop"]

    if plate is None:
        api.log("ABORT no plate found")
        return
    px, py, ptop = plate["cx"], plate["cy"], plate["ztop"]

    gx, gy = cx, cy - PINCH_R
    gz = ztop + GRASP_DZ
    api.log("PLAN grasp=(%.3f,%.3f,%.3f) place=(%.3f,%.3f)" % (gx, gy, gz, px, py))

    api.grip(0.08)
    api.log("R hover %.4f" % api.move([gx, gy, ztop + 0.10], seconds=2.0))
    api.log("R down %.4f" % api.move([gx, gy, gz], seconds=2.0))
    api.log("EEF@grasp %s" % json.dumps([round(float(v), 4) for v in api.eef()]))
    api.grip(0.0)
    api.settle(0.6)
    g = api.gripper()
    api.log("GRIP after close %s" % json.dumps(g))

    api.log("R lift %.4f" % api.move([gx, gy, CARRY_Z], seconds=2.0))
    api.log("GRIP after lift %s" % json.dumps(api.gripper()))
    api.log("EEF@lift %s" % json.dumps([round(float(v), 4) for v in api.eef()]))

    api.log("R over %.4f" % api.move([px, py - PINCH_R, CARRY_Z], seconds=2.0))
    pz = ptop + HANG + PLACE_CLEAR
    api.log("R place %.4f" % api.move([px, py - PINCH_R, pz], seconds=2.0))
    api.log("GRIP at place %s" % json.dumps(api.gripper()))
    api.grip(0.08)
    api.settle(0.6)
    api.log("R up %.4f" % api.move([px, py - PINCH_R, ptop + 0.16], seconds=2.0))
    api.settle(0.5)
