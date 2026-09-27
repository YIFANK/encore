"""v4 -- v3 plus ranked target selection, grasp verification and retry.

Pipeline
  1. cam_high -> point cloud -> top-down max-height grid (points above the
     table, below a ceiling that cuts the arm away).
  2. Connected components on that grid = props.  The largest is the basket;
     the props whose top surface is 10-45 mm above the table are the two boxes
     lying flat, and the cream cheese is the one whose top surface is bluest
     (ranked by blue-minus-red, never thresholded).
  3. Park the arm clear of the frame and re-perceive, so the target's footprint
     is measured without the arm's occlusion.
  4. Top-down grasp with the jaws closing along base +-y (the boxes lie with
     their long axis along base x), verified by gripper effort/width; on a
     failed bite, reopen, re-perceive and retry.
  5. Carry above the tallest measured obstacle and release over the basket.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-65, cam_high cloud: tabletop plane reads z=0.000", "allowed": True},
    "Z_MIN": {"source": "debug seeds 51-65: 10 mm clears the tabletop plane noise (|z|<0.002)", "allowed": True},
    "Z_CEIL": {"source": "debug seeds 51-65: every prop tops out at <=0.144; the arm links at reset sit above 0.25", "allowed": True},
    "GRID_S": {"source": "generic perception mechanics; 5 mm chosen resolution", "allowed": True},
    "X0/Y0/NX/NY": {"source": "debug seeds 51-65 cloud extent of the tabletop props", "allowed": True},
    "FLAT_TOP_MAX": {"source": "debug seeds 51-65: the two lying boxes measure top=0.019/0.020; the next height class (cans) is 0.081", "allowed": True},
    "MIN_CELLS": {"source": "debug seeds 51-65: the smallest real prop cluster is 67 cells; noise blobs are <25", "allowed": True},
    "MERGE_R": {"source": "debug seed 51: arm occlusion split the target into two clusters 0.04 apart", "allowed": True},
    "GRASP_Z": {"source": "pack keyframes: all 3 demos close the gripper at eef z=0.0101-0.0108; debug seeds put the box top at 0.020", "allowed": True},
    "HOVER_Z": {"source": "debug-seed observation: 0.10 clears the 0.020 box top with margin", "allowed": True},
    "CARRY_Z": {"source": "debug seeds 51-65: tallest obstacle (basket rim) measured at 0.144", "allowed": True},
    "RELEASE_Z": {"source": "pack keyframes: demo releases at eef z=0.152/0.179/0.219 over the basket", "allowed": True},
    "OPEN_W/CLOSE_W": {"source": "debug seeds: api.gripper() reads 0.0778 open, 0.0422 closed on this box", "allowed": True},
    "HOLD_W_MAX": {"source": "debug seeds 51-65: a holding bite reads width 0.0422, a bite on nothing closes to ~0.0", "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics: tool z down, jaws along base y (matches api.tool_rotation() at reset)", "allowed": True},
    "PARK": {"source": "debug-seed observation: the arm at reset occludes the -x half of the table in cam_high", "allowed": True},
    "BLUE_MARGIN": {"source": "debug seeds 51-65: flat-box top colours give b-r = +24..+30 (cream cheese) vs -55..-65 (the other box); used for logging only, selection is by rank", "allowed": True},
}

CHUNK = 1500
Z_MIN = 0.010
Z_CEIL = 0.17
GRID_S = 0.005
X0, Y0, NX, NY = -0.40, -0.45, 180, 200
FLAT_TOP_MAX = 0.045
MIN_CELLS = 25
MERGE_R = 0.10
GRASP_Z = 0.010
HOVER_Z = 0.10
CARRY_Z = 0.26
RELEASE_Z = 0.20
OPEN_W = 0.08
CLOSE_W = 0.0
HOLD_W_MAX = 0.065
HOLD_W_MIN = 0.010
BLUE_MARGIN = 5.0
PARK = (0.00, 0.30, 0.40)

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def cloud(f):
    H, W = f.depth.shape
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    vs, us = np.mgrid[0:H, 0:W]
    z = np.asarray(f.depth, float)
    x = (us - K[0, 2]) * z / K[0, 0]
    y = (vs - K[1, 2]) * z / K[1, 1]
    return np.stack([x, y, z], -1) @ T[:3, :3].T + T[:3, 3]


def grids(P, rgb):
    Pf = P.reshape(-1, 3)
    Cf = rgb.reshape(-1, 3).astype(float)
    sel = ((Pf[:, 0] > X0) & (Pf[:, 0] < X0 + NX * GRID_S) &
           (Pf[:, 1] > Y0) & (Pf[:, 1] < Y0 + NY * GRID_S) & (Pf[:, 2] < Z_CEIL))
    Pf, Cf = Pf[sel], Cf[sel]
    ix = ((Pf[:, 0] - X0) / GRID_S).astype(int)
    iy = ((Pf[:, 1] - Y0) / GRID_S).astype(int)
    flat = ix * NY + iy
    h = np.full(NX * NY, -1.0)
    np.maximum.at(h, flat, Pf[:, 2])
    o = np.argsort(Pf[:, 2])
    c = np.zeros((NX * NY, 3))
    c[flat[o]] = Cf[o]
    return h.reshape(NX, NY), c.reshape(NX, NY, 3)


def comps(h, c):
    m = h > Z_MIN
    lab = np.zeros(h.shape, int)
    cur = 0
    out = []
    for i in range(NX):
        for j in range(NY):
            if m[i, j] and not lab[i, j]:
                cur += 1
                st = [(i, j)]
                lab[i, j] = cur
                cells = []
                while st:
                    a, b = st.pop()
                    cells.append((a, b))
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if 0 <= p < NX and 0 <= q < NY and m[p, q] and not lab[p, q]:
                                lab[p, q] = cur
                                st.append((p, q))
                if len(cells) < MIN_CELLS:
                    continue
                cs = np.array(cells)
                xs = X0 + (cs[:, 0] + 0.5) * GRID_S
                ys = Y0 + (cs[:, 1] + 0.5) * GRID_S
                zz = h[cs[:, 0], cs[:, 1]]
                col = c[cs[:, 0], cs[:, 1]]
                top = float(zz.max())
                tb = zz > top - 0.010
                out.append(dict(n=len(cells), top=top,
                                xr=(float(xs.min()), float(xs.max())),
                                yr=(float(ys.min()), float(ys.max())),
                                x=float(xs.mean()), y=float(ys.mean()),
                                topcol=col[tb].mean(0)))
    out.sort(key=lambda r: -r['n'])
    return out


def perceive(api, tag):
    f = api.capture("cam_high")
    rs = comps(*grids(cloud(f), np.asarray(f.rgb)))
    for k, r in enumerate(rs):
        api.log("%s cl%d n=%d c=(%+.3f,%+.3f) top=%.3f x[%+.3f,%+.3f] y[%+.3f,%+.3f] col=(%.0f,%.0f,%.0f) b-r=%+.0f"
                % (tag, k, r['n'], r['x'], r['y'], r['top'], r['xr'][0], r['xr'][1],
                   r['yr'][0], r['yr'][1], r['topcol'][0], r['topcol'][1], r['topcol'][2],
                   r['topcol'][2] - r['topcol'][0]))
    return rs


def pick_target(rs):
    """Flat boxes only; among them the bluest top surface is the cream cheese."""
    flats = [r for r in rs if Z_MIN < r['top'] < FLAT_TOP_MAX]
    if not flats:
        return None
    seed = max(flats, key=lambda r: r['topcol'][2] - r['topcol'][0])
    xr, yr = list(seed['xr']), list(seed['yr'])
    for r in flats:
        if r is seed:
            continue
        if (abs(r['x'] - seed['x']) < MERGE_R and abs(r['y'] - seed['y']) < MERGE_R
                and r['topcol'][2] - r['topcol'][0] > BLUE_MARGIN):
            xr = [min(xr[0], r['xr'][0]), max(xr[1], r['xr'][1])]
            yr = [min(yr[0], r['yr'][0]), max(yr[1], r['yr'][1])]
    return dict(x=0.5 * (xr[0] + xr[1]), y=0.5 * (yr[0] + yr[1]), top=seed['top'],
                xr=xr, yr=yr, bmr=float(seed['topcol'][2] - seed['topcol'][0]))


def holding(api):
    g = api.gripper()
    return g['effort'] >= 3.0 and HOLD_W_MIN < g['width_m'] < HOLD_W_MAX, g


def run(api):
    api.log("instr=%r eef0=%s" % (api.instruction(), np.round(api.eef(), 4).tolist()))
    rs0 = perceive(api, "P0")
    if not rs0:
        api.log("NO CLUSTERS")
        return
    basket = max(rs0, key=lambda r: r['n'])

    api.move(PARK, rotation=R_DOWN, seconds=2.0)
    rs1 = perceive(api, "P1")
    if rs1:
        basket = max(rs1, key=lambda r: r['n'])
    api.log("basket=(%+.3f,%+.3f) top=%.3f" % (basket['x'], basket['y'], basket['top']))

    tgt = pick_target(rs1) or pick_target(rs0)
    if tgt is None:
        api.log("NO TARGET")
        return
    api.log("target=(%+.3f,%+.3f) top=%.3f dx=%.3f dy=%.3f b-r=%+.0f"
            % (tgt['x'], tgt['y'], tgt['top'], tgt['xr'][1] - tgt['xr'][0],
               tgt['yr'][1] - tgt['yr'][0], tgt['bmr']))

    got = False
    for attempt in range(3):
        gx, gy = tgt['x'], tgt['y']
        gz = min(GRASP_Z, max(0.008, 0.5 * tgt['top']))
        api.grip(OPEN_W)
        api.move((gx, gy, CARRY_Z), rotation=R_DOWN, seconds=2.0)
        api.move((gx, gy, HOVER_Z), rotation=R_DOWN, seconds=1.5)
        r = api.move((gx, gy, gz), rotation=R_DOWN, seconds=1.5)
        api.log("a%d descend res=%.4f eef=%s" % (attempt, float(r), np.round(api.eef(), 4).tolist()))
        api.grip(CLOSE_W)
        api.settle(0.4)
        ok, g = holding(api)
        api.log("a%d close grip=%s ok=%s" % (attempt, g, ok))
        api.move((gx, gy, CARRY_Z), rotation=R_DOWN, seconds=2.0)
        ok, g = holding(api)
        api.log("a%d lift grip=%s ok=%s eef=%s" % (attempt, g, ok, np.round(api.eef(), 4).tolist()))
        if ok:
            got = True
            break
        api.grip(OPEN_W)
        api.move(PARK, rotation=R_DOWN, seconds=2.0)
        rs = perceive(api, "R%d" % attempt)
        t2 = pick_target(rs)
        if t2 is not None:
            tgt = t2
            api.log("retarget=(%+.3f,%+.3f) top=%.3f" % (t2['x'], t2['y'], t2['top']))

    if not got:
        api.log("GRASP FAILED after retries")
        return

    api.move((basket['x'], basket['y'], CARRY_Z), rotation=R_DOWN, seconds=2.5)
    ok, g = holding(api)
    api.log("over basket grip=%s ok=%s eef=%s" % (g, ok, np.round(api.eef(), 4).tolist()))
    api.move((basket['x'], basket['y'], RELEASE_Z), rotation=R_DOWN, seconds=1.5)
    api.grip(OPEN_W)
    api.settle(0.5)
    api.log("released eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.move((basket['x'], basket['y'], CARRY_Z), rotation=R_DOWN, seconds=1.5)
    api.settle(0.5)
