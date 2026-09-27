"""c2k1clean / obj_orange_juice_pos_k1 -- pick the orange juice, place in basket.

All constants derived from packs/c2k1clean_obj_orange_juice_pos_k1/pack.json and
from own observations on debug seeds 51..65 (cam_high RGB-D dumps).
"""
import numpy as np
from collections import defaultdict

PROVENANCE = {
    "WS_X": {"source": "debug-seed cam_high point cloud: all table props fall in base x -0.22..0.18; bound widened to -0.32..0.34", "allowed": True},
    "WS_Y": {"source": "debug-seed cam_high point cloud: props+basket span base y -0.27..0.35; bound widened to -0.45..0.45", "allowed": True},
    "TABLE_Z": {"source": "debug-seed cam_high depth histogram: dominant plane mode at base z in [0.000,0.010)", "allowed": True},
    "Z_MIN": {"source": "debug-seed: 0.015 = TABLE_Z + noise margin, keeps table out of the object mask", "allowed": True},
    "Z_MAX": {"source": "debug-seed: tallest prop top 0.148, arm reaches 0.484; 0.50 keeps everything then filtered by height gate", "allowed": True},
    "ROBOT_G": {"source": "debug-seed cam_high RGB: arm links render saturated green/blue (G>R+20 & G>B+20) unlike every prop", "allowed": True},
    "ROBOT_B": {"source": "debug-seed cam_high RGB: arm links render saturated blue (B>R+30 & B>G+30)", "allowed": True},
    "VOXEL": {"source": "debug-seed: 0.015 m voxel with 26-connectivity separates all props (nearest prop gap > 0.03 m)", "allowed": True},
    "MIN_PTS": {"source": "debug-seed: smallest prop cluster 421 px, noise blobs < 150 px", "allowed": True},
    "MAX_PTS": {"source": "debug-seed: basket cluster 11k-11.6k px, every prop < 2.8k px; 6000 splits them", "allowed": True},
    "ORANGE_RULE": {"source": "debug-seed cam_high RGB: orange-pixel fraction of the juice carton 0.144 vs 0.022 for the next prop", "allowed": True},
    "TGT_ZTOP": {"source": "debug-seed: juice carton top at base z 0.141; gate 0.06..0.26 excludes flat boxes (0.019,0.029) and the arm", "allowed": True},
    "TOP_BAND": {"source": "debug-seed: 0.025 m band below ztop isolates the carton top face for its xy footprint", "allowed": True},
    "GRASP_DROP": {"source": "pack ee_path idx5 grasp z 0.1145 vs debug-seed carton top 0.141 -> grasp 0.027 below the top", "allowed": True},
    "HOVER_Z": {"source": "pack ee_path idx4 pre-grasp 0.1294 and idx0 0.2729; 0.26 clears the tallest prop top 0.148", "allowed": True},
    "CARRY_Z": {"source": "pack ee_path idx8-10 transfer z 0.29-0.296", "allowed": True},
    "RELEASE_DZ": {"source": "pack ee_path idx12 release z 0.175 vs debug-seed basket rim 0.142 -> 0.033 above the rim", "allowed": True},
    "BASKET_MIN_PTS": {"source": "debug-seed: basket cluster 11.1k-11.7k px, far above any prop", "allowed": True},
    "HOLD_EFFORT": {"source": "FairApi doc + debug-seed reading: effort 0.05 when open/empty, 3.0 iff holding", "allowed": True},
    "MERGE_SPAN": {"source": "debug-seed: gripper open width 0.0778 m, so a top face spanning >0.075 m cannot be a single graspable prop and signals a fused cluster", "allowed": True},
    "MIN_ORANGE_PTS": {"source": "debug-seed: the juice carton contributes ~267 orange px (0.144 of 1853); 60 is a floor for a usable orange footprint", "allowed": True},
    "CLEAR_POSE": {"source": "debug-seed: reset eef (-0.1485,0,0.2613) sits over the far prop row; (0.05,-0.25,0.30) is inside the demo's travelled envelope (pack ee_path x -0.14..0.08, y -0.11..0.23) and moves the links off the far row", "allowed": True},
}

WS_X = (-0.32, 0.34)
WS_Y = (-0.45, 0.45)
Z_MIN, Z_MAX = 0.015, 0.50
VOXEL = 0.015
MIN_PTS, MAX_PTS = 150, 6000
BASKET_MIN_PTS = 6000
TGT_ZTOP = (0.06, 0.26)
TOP_BAND = 0.025
GRASP_DROP = 0.027
HOVER_Z = 0.26
CARRY_Z = 0.29
RELEASE_DZ = 0.033
HOLD_EFFORT = 1.0
MERGE_SPAN = 0.075
MIN_ORANGE_PTS = 60
CLEAR_POSE = (0.05, -0.25, 0.30)

NEIGH = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)]


def _cloud(f):
    rgb = np.asarray(f.rgb).astype(np.float32)
    d = np.asarray(f.depth, dtype=np.float64)
    K = np.asarray(f.intrinsics)
    T = np.asarray(f.t_base_cam)
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    pc = np.stack([x, y, d, np.ones_like(d)], axis=-1)
    return rgb, (pc @ T.T)[..., :3], d


def _vox(X, Y, Z):
    gx = np.round(X / VOXEL).astype(int)
    gy = np.round(Y / VOXEL).astype(int)
    gz = np.round(Z / VOXEL).astype(int)
    cells = defaultdict(list)
    for i in range(len(gx)):
        cells[(gx[i], gy[i], gz[i])].append(i)
    seen = set()
    out = []
    for c0 in cells:
        if c0 in seen:
            continue
        st = [c0]
        seen.add(c0)
        comp = []
        while st:
            cc = st.pop()
            comp.extend(cells[cc])
            for dn in NEIGH:
                n = (cc[0] + dn[0], cc[1] + dn[1], cc[2] + dn[2])
                if n in cells and n not in seen:
                    seen.add(n)
                    st.append(n)
        out.append(np.array(comp))
    out.sort(key=len, reverse=True)
    return out


def perceive(api):
    f = api.capture("cam_high")
    rgb, P, d = _cloud(f)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    R, G, B = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    robot = ((G > R + 20) & (G > B + 20)) | ((B > R + 30) & (B > G + 30))
    m = (np.isfinite(Z) & (d > 0.3) & (d < 2.5) & (~robot)
         & (X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1])
         & (Z > Z_MIN) & (Z < Z_MAX))
    xs, ys, zs = X[m], Y[m], Z[m]
    cols = rgb[m] / 255.0
    objs = []
    for ci in _vox(xs, ys, zs):
        if len(ci) < MIN_PTS:
            continue
        c = cols[ci]
        om = ((c[:, 0] > 0.45) & (c[:, 0] > c[:, 1] + 0.13)
              & (c[:, 1] > c[:, 2] + 0.05))
        orange = float(om.mean())
        ztop = float(np.percentile(zs[ci], 99))
        top = ci[zs[ci] > ztop - TOP_BAND]
        oi = ci[om]
        if len(oi):
            oxr = (float(np.percentile(xs[oi], 5)), float(np.percentile(xs[oi], 95)))
            oyr = (float(np.percentile(ys[oi], 5)), float(np.percentile(ys[oi], 95)))
        else:
            oxr = oyr = (0.0, 0.0)
        objs.append(dict(
            n=len(ci), ztop=ztop, orange=orange, on=int(om.sum()),
            x=float(np.median(xs[ci])), y=float(np.median(ys[ci])),
            txr=(float(xs[top].min()), float(xs[top].max())),
            tyr=(float(ys[top].min()), float(ys[top].max())),
            oxr=oxr, oyr=oyr,
            xr=(float(xs[ci].min()), float(xs[ci].max())),
            yr=(float(ys[ci].min()), float(ys[ci].max()))))
    return objs


def select(objs):
    """Return (target, basket) or (None, None)."""
    cand = [o for o in objs if MIN_PTS <= o['n'] <= MAX_PTS
            and TGT_ZTOP[0] <= o['ztop'] <= TGT_ZTOP[1]]
    bas = [o for o in objs if o['n'] > BASKET_MIN_PTS and o['ztop'] < 0.30]
    if not cand or not bas:
        return None, None
    return max(cand, key=lambda o: o['orange']), max(bas, key=lambda o: o['n'])


def grasp_xy(t, api):
    """Top-face centre, with a merge guard: if the top face is wider than the jaws
    can span the cluster has fused with a neighbour, so fall back to the orange
    pixels' own footprint."""
    sx = t['txr'][1] - t['txr'][0]
    sy = t['tyr'][1] - t['tyr'][0]
    if (sy > MERGE_SPAN or sx > MERGE_SPAN) and t['on'] >= MIN_ORANGE_PTS:
        api.log("merge guard span x%.3f y%.3f -> orange fp" % (sx, sy))
        return 0.5 * (t['oxr'][0] + t['oxr'][1]), 0.5 * (t['oyr'][0] + t['oyr'][1])
    return 0.5 * (t['txr'][0] + t['txr'][1]), 0.5 * (t['tyr'][0] + t['tyr'][1])


def holding(api):
    g = api.gripper()
    return g['effort'] >= HOLD_EFFORT, g


def run(api):
    api.log("v4 start eef %s" % np.round(api.eef(), 4).tolist())
    api.grip(0.08)
    api.settle(0.2)

    objs = perceive(api)
    t, b = select(objs)
    if t is None:
        # the arm may be occluding the carton: step the wrist aside and re-look
        api.log("no cand (%d objs) -> clearing view" % len(objs))
        try:
            api.move([CLEAR_POSE[0], CLEAR_POSE[1], CLEAR_POSE[2]], seconds=2.0)
            api.settle(0.3)
            objs = perceive(api)
            t, b = select(objs)
        except Exception as e:
            api.log("clear-view failed %r" % (e,))
    if t is None or b is None:
        api.log("FAIL perception")
        return
    for o in objs[:9]:
        api.log("o n%d z%.3f or%.3f c(%.3f,%.3f)" % (o['n'], o['ztop'], o['orange'], o['x'], o['y']))

    bx = 0.5 * (b['xr'][0] + b['xr'][1])
    by = 0.5 * (b['yr'][0] + b['yr'][1])
    rz = b['ztop'] + RELEASE_DZ
    api.log("BSK (%.3f,%.3f) ztop%.3f rz%.3f" % (bx, by, b['ztop'], rz))

    got = False
    for att in range(3):
        if att > 0:
            objs = perceive(api)
            t2, _ = select(objs)
            if t2 is not None:
                t = t2
        gx, gy = grasp_xy(t, api)
        if att == 2:
            # a genuinely different estimate: the full cluster centre, which the
            # camera-facing front face pulls toward +x
            gx = 0.5 * (t['xr'][0] + t['xr'][1])
            gy = 0.5 * (t['yr'][0] + t['yr'][1])
        gz = t['ztop'] - GRASP_DROP
        api.log("a%d TGT (%.3f,%.3f) ztop%.3f gz%.3f or%.3f span x%.3f y%.3f"
                % (att, gx, gy, t['ztop'], gz, t['orange'],
                   t['txr'][1] - t['txr'][0], t['tyr'][1] - t['tyr'][0]))
        api.grip(0.08)
        api.settle(0.2)
        api.move([gx, gy, HOVER_Z], seconds=2.0)
        r = api.move([gx, gy, gz], seconds=2.0)
        api.log("a%d descend res %.4f eef %s" % (att, float(np.max(np.abs(np.asarray(r)))),
                                                 np.round(api.eef(), 4).tolist()))
        api.grip(0.0)
        api.settle(0.5)
        ok, g = holding(api)
        api.log("a%d close w%.4f e%.2f ok%d" % (att, g['width_m'], g['effort'], ok))
        api.move([gx, gy, CARRY_Z], seconds=2.0)
        ok, g = holding(api)
        api.log("a%d lift w%.4f e%.2f ok%d" % (att, g['width_m'], g['effort'], ok))
        if ok:
            got = True
            break
        api.grip(0.08)
        api.settle(0.3)
    if not got:
        api.log("FAIL no grasp")
        return

    r = api.move([bx, by, CARRY_Z], seconds=2.5)
    api.log("over res %.4f eef %s" % (float(np.max(np.abs(np.asarray(r)))),
                                      np.round(api.eef(), 4).tolist()))
    ok, g = holding(api)
    api.log("over w%.4f e%.2f ok%d" % (g['width_m'], g['effort'], ok))
    r = api.move([bx, by, rz], seconds=2.0)
    api.log("lower res %.4f eef %s" % (float(np.max(np.abs(np.asarray(r)))),
                                       np.round(api.eef(), 4).tolist()))
    api.grip(0.08)
    api.settle(0.5)
    g = api.gripper()
    api.log("release w%.4f e%.2f" % (g['width_m'], g['effort']))
    api.move([bx, by, CARRY_Z], seconds=1.5)
    api.settle(0.5)
    api.log("v4 done")
