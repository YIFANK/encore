"""c2clean / obj_alphabet_soup_pos_k3 -- v2

Same skeleton as v1 (perceive from cam_high, rank can-shaped props by body
blueness, grasp at the pack's own closing height, release over the perceived
basket) with two hardenings for layouts not seen on the debug split:

  * a second component pass on a LOW HEIGHT SLAB, so a can standing against a
    taller prop (the pack's own layout has exactly that) is not fused with it;
  * a retry ladder that steps along the CLOSING axis, which the debug-seed
    envelope probe showed is the only axis with a tight tolerance.
"""
import numpy as np

PROVENANCE = {
    # --- pack-derived ---
    "GRASP_Z": {
        "source": "pack.json: the gripper_cmd=+1 keyframes close at ee z=0.045 "
                  "(demo0 t42, demo2 t46); demo1's first close at z=0.076 came "
                  "back empty (gripper_state 0.0012/-0.0013 at t65) and its "
                  "re-close at z=0.045 (t80) held",
        "allowed": True},
    "RELEASE_Z": {
        "source": "pack.json: the gripper re-opens over the basket at ee z=0.152 "
                  "(demo0 t128), 0.145 (demo1 t159), 0.124 (demo2 t131)",
        "allowed": True},
    "R_DOWN": {
        "source": "pack.json ee_path6 attitude is roll~3.11 rad, pitch~0 in every "
                  "demo, i.e. the straight-down tool frame; matches "
                  "api.tool_rotation() at reset on debug seeds 51-65",
        "allowed": True},
    "TARGET_IS_BLUE_BODIED": {
        "source": "pack.json: each demo's closing EEF, projected into that demo's "
                  "own keyframe with the debug-seed cam_high intrinsics/extrinsics, "
                  "lands at RGB~(29,43,79) -- a blue-bodied can -- in all three "
                  "demos; that prop is present in demo0_t0000 and absent from "
                  "demo0_t0128 after the place",
        "allowed": True},
    # --- debug-seed (51-65) measurements ---
    "CELL": {"source": "generic: 1 cm top-down occupancy grid", "allowed": True},
    "XL_YL": {
        "source": "debug seeds 51-65: every prop deprojects inside "
                  "x[-0.21,+0.19], y[-0.24,+0.35]; the grid is padded past that",
        "allowed": True},
    "Z_LO": {
        "source": "debug seeds 51-65: the table plane deprojects to z=0.000-0.002, "
                  "so 0.020 clears it",
        "allowed": True},
    "Z_HI": {
        "source": "debug seeds 51-65: the tallest prop tops out at z=0.148 while "
                  "the arm at reset occupies z 0.25-0.49 over x[-0.235,-0.095]; "
                  "capping the map at 0.200 keeps the arm out of it",
        "allowed": True},
    "SLAB_HI": {
        "source": "debug seeds 51-65: both cans top out at z=0.081 while every "
                  "taller prop (bottle 0.148, carton 0.140, basket rim 0.143) is "
                  "above 0.13; 0.110 slices between them",
        "allowed": True},
    "CAN_GATE": {
        "source": "debug seeds 51-65: the cans measure 0.064-0.070 across the top "
                  "rim (0.058 by a Kasa fit) with top z=0.081 and 45-49 occupied "
                  "cells; the basket footprint is 0.16x0.17 with 130-160 cells",
        "allowed": True},
    "BASKET_GATE": {
        "source": "debug seeds 51-65: the basket is the only component with a "
                  "footprint wider than 0.12 m and a top above 0.10 m",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug seeds 51-65: tallest prop 0.148, basket rim 0.143; at eef "
                  "z=0.250 the can (grasped with its base ~0.045 below the eef) "
                  "clears both",
        "allowed": True},
    "HELD_MIN_W": {
        "source": "pack demo1 t65 reads width 0.0025 for an empty close; debug "
                  "seeds 51-65 read 0.0625 for a centred hold and 0.0499 for an "
                  "off-centre hold, so 0.020 separates a hold from a miss",
        "allowed": True},
    "OPEN_W": {
        "source": "debug seeds 51-65: api.gripper() reports width_m 0.0778 at reset",
        "allowed": True},
    "LADDER_DY": {
        "source": "debug-seed envelope probe (seeds 52,58, results dir "
                  "fs_c2clean_obj_alphabet_soup_pos_k3_env): displacing the aim "
                  "along +y still holds at 0.008 and misses at 0.014/0.020, while "
                  "+x holds out to 0.020; so y is the tight axis and a miss means "
                  "the error already exceeds ~0.010",
        "allowed": True},
}

CELL = 0.010
XL = (-0.40, 0.35)
YL = (-0.45, 0.45)
Z_LO = 0.020
Z_HI = 0.200
SLAB_HI = 0.110
CARRY_Z = 0.250
GRASP_Z = 0.045
RELEASE_Z = 0.145
HELD_MIN_W = 0.020
OPEN_W = 0.080
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


# ---------------------------------------------------------------- perception
def _cloud(fr):
    dep = np.asarray(fr.depth, dtype=np.float64)
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    H, W = dep.shape
    vv, uu = np.mgrid[0:H, 0:W]
    x = (uu - K[0, 2]) * dep / K[0, 0]
    y = (vv - K[1, 2]) * dep / K[1, 1]
    pts = np.stack([x, y, dep], -1).reshape(-1, 3)
    pb = (T[:3, :3] @ pts.T).T + T[:3, 3]
    return pb.reshape(H, W, 3), np.isfinite(pb).all(-1).reshape(H, W) & (dep > 0.05)


def _label(mask):
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    for i in range(mask.shape[0]):
        for j in range(mask.shape[1]):
            if mask[i, j] and lab[i, j] == 0:
                cur += 1
                st = [(i, j)]
                lab[i, j] = cur
                while st:
                    a, b = st.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if (0 <= p < mask.shape[0] and 0 <= q < mask.shape[1]
                                    and mask[p, q] and lab[p, q] == 0):
                                lab[p, q] = cur
                                st.append((p, q))
    return lab, cur


def _describe(Hm, lab, k, xs, ys, zs, rgb, good):
    ii, jj = np.where(lab == k)
    if len(ii) < 4:
        return None
    x = XL[0] + (ii + 0.5) * CELL
    y = YL[0] + (jj + 0.5) * CELL
    xr = (float(x.min()), float(x.max()))
    yr = (float(y.min()), float(y.max()))
    top = float(Hm[ii, jj].max())
    box = (good & (xs >= xr[0] - 0.005) & (xs <= xr[1] + 0.005)
           & (ys >= yr[0] - 0.005) & (ys <= yr[1] + 0.005)
           & (zs > Z_LO - 0.005) & (zs < top + 0.010))
    rim = box & (zs > top - 0.008)
    if rim.sum() < 20:
        return None
    rx, ry = xs[rim], ys[rim]
    cen = (float((rx.min() + rx.max()) / 2.0), float((ry.min() + ry.max()) / 2.0))
    dia = float(max(np.ptp(rx), np.ptp(ry)))
    body = box & (zs > 0.015) & (zs < top - 0.015)
    col = rgb[body].mean(0) if body.sum() > 20 else rgb[rim].mean(0)
    return dict(n=int(len(ii)), cen=cen, top=top, dia=dia,
                w=xr[1] - xr[0], l=yr[1] - yr[0],
                blue=float(col[2] - col[0]), col=col)


def perceive(api):
    fr = api.capture("cam_high")
    rgb = np.asarray(fr.rgb).astype(np.float64)
    pb, good = _cloud(fr)
    xs, ys, zs = pb[..., 0], pb[..., 1], pb[..., 2]
    keep = (good & (xs > XL[0]) & (xs < XL[1]) & (ys > YL[0]) & (ys < YL[1])
            & (zs > -0.05) & (zs < Z_HI))
    nx = int(round((XL[1] - XL[0]) / CELL))
    ny = int(round((YL[1] - YL[0]) / CELL))
    ix = ((xs[keep] - XL[0]) / CELL).astype(int)
    iy = ((ys[keep] - YL[0]) / CELL).astype(int)
    Hm = np.full((nx, ny), -1.0)
    np.maximum.at(Hm, (ix, iy), zs[keep])

    full = []
    lab, n = _label(Hm > Z_LO)
    for k in range(1, n + 1):
        d = _describe(Hm, lab, k, xs, ys, zs, rgb, good)
        if d:
            full.append(d)
    # second pass on a low slab: a short prop touching a tall one is its own
    # component here, because the tall prop's cells are simply not in the mask
    slab = []
    lab2, n2 = _label((Hm > Z_LO) & (Hm < SLAB_HI))
    for k in range(1, n2 + 1):
        d = _describe(Hm, lab2, k, xs, ys, zs, rgb, good)
        if d:
            slab.append(d)
    return full, slab


def _cans(objs, basket):
    out = []
    for o in objs:
        if basket is not None and abs(o['cen'][0] - basket['cen'][0]) < 0.02 \
           and abs(o['cen'][1] - basket['cen'][1]) < 0.02:
            continue
        if o['n'] < 12 or o['dia'] < 0.040 or o['dia'] > 0.100:
            continue
        if o['top'] < 0.040 or o['top'] > 0.130:
            continue
        if max(o['w'], o['l']) > 0.11:
            continue
        out.append(o)
    return out


def choose(api, full, slab):
    for tag, lst in (("FULL", full), ("SLAB", slab)):
        for o in lst:
            api.log("%s cen=(%+.4f,%+.4f) top=%.3f dia=%.4f box=%.2fx%.2f n=%d "
                    "rgb=(%.0f,%.0f,%.0f) B-R=%+.1f"
                    % (tag, o['cen'][0], o['cen'][1], o['top'], o['dia'],
                       o['w'], o['l'], o['n'], o['col'][0], o['col'][1],
                       o['col'][2], o['blue']))
    tall = [o for o in full if o['top'] > 0.10 and max(o['w'], o['l']) > 0.12]
    basket = max(tall, key=lambda o: o['n']) if tall else max(full, key=lambda o: o['n'])

    pool = _cans(full, basket) + _cans(slab, basket)
    # dedupe: same prop found by both passes
    keep = []
    for o in sorted(pool, key=lambda o: -o['n']):
        if not any(abs(o['cen'][0] - k['cen'][0]) < 0.030
                   and abs(o['cen'][1] - k['cen'][1]) < 0.030 for k in keep):
            keep.append(o)
    if not keep:
        keep = [o for o in full if o is not basket] or full
    keep.sort(key=lambda o: -o['blue'])
    api.log("CANDIDATES %s" % ([(round(o['cen'][0], 3), round(o['cen'][1], 3),
                                 round(o['blue'], 1)) for o in keep],))
    return keep[0], basket


# ------------------------------------------------------------------- motion
def goto(api, xyz, seconds=2.0, tag=""):
    r = api.move([float(v) for v in xyz], rotation=R_DOWN, seconds=seconds)
    e = api.eef()
    try:
        rs = round(float(r), 4)
    except (TypeError, ValueError):
        rs = np.round(np.asarray(r), 4).tolist()
    api.log("MOVE%s -> (%.4f,%.4f,%.4f) res=%s eef=(%.4f,%.4f,%.4f)"
            % (tag, xyz[0], xyz[1], xyz[2], rs, e[0], e[1], e[2]))
    return r


def width(api):
    g = api.gripper()
    return float(g['width_m']), float(g.get('effort', 0.0))


# y is the closing axis, so the ladder walks y; x is forgiving to >=0.020.
LADDER = [(0.0, 0.0, 0.0), (0.0, 0.0, -0.010), (0.0, -0.014, 0.0),
          (0.0, +0.014, 0.0), (0.0, -0.024, 0.0)]


def run(api):
    api.log("instruction=%r" % (api.instruction(),))
    full, slab = perceive(api)
    target, basket = choose(api, full, slab)
    bx, by = basket['cen']
    api.log("TARGET (%.4f,%.4f) dia=%.4f top=%.3f | BASKET (%.4f,%.4f) top=%.3f"
            % (target['cen'][0], target['cen'][1], target['dia'], target['top'],
               bx, by, basket['top']))

    api.grip(OPEN_W)
    held = False
    for i, (dx, dy, dz) in enumerate(LADDER):
        if i > 0:
            # the previous attempt may have nudged the prop; look again
            f2, s2 = perceive(api)
            t2, b2 = choose(api, f2, s2)
            target = t2
            bx, by = b2['cen']
        tx, ty = target['cen']
        ax, ay, az = tx + dx, ty + dy, GRASP_Z + dz
        api.log("ATTEMPT%d centre=(%.4f,%.4f) aim=(%.4f,%.4f,%.4f)"
                % (i, tx, ty, ax, ay, az))
        goto(api, (ax, ay, CARRY_Z), 2.0, " pre%d" % i)
        goto(api, (ax, ay, az), 2.0, " down%d" % i)
        api.grip(0.0)
        api.settle(0.3)
        w, ef = width(api)
        api.log("CLOSE%d w=%.4f effort=%.2f" % (i, w, ef))
        goto(api, (ax, ay, CARRY_Z), 2.0, " lift%d" % i)
        w2, ef2 = width(api)
        api.log("LIFT%d w=%.4f effort=%.2f" % (i, w2, ef2))
        if w2 > HELD_MIN_W:
            held = True
            break
        api.grip(OPEN_W)
        api.settle(0.2)
    api.log("HELD=%s" % held)

    goto(api, (bx, by, CARRY_Z), 2.5, " over")
    goto(api, (bx, by, RELEASE_Z), 2.0, " in")
    w3, ef3 = width(api)
    api.log("BEFORE_RELEASE w=%.4f effort=%.2f" % (w3, ef3))
    api.grip(OPEN_W)
    api.settle(0.5)
    goto(api, (bx, by, CARRY_Z), 2.0, " out")
    api.settle(0.5)
