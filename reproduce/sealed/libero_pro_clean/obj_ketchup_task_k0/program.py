"""v4: perceive the milk carton from cam_high, grasp its upper body, drop it in
the basket. Tip offset and the move tracking bias were measured in v2."""

from collections import defaultdict, deque

import numpy as np

PROVENANCE = {
    "Z_BAND": {"source": "debug seeds 51-65 cam_high deprojection: table plane at "
                         "base z~0.000, props occupy z 0.012..0.17, arm z>0.24",
               "allowed": True},
    "WORKSPACE_XY": {"source": "debug seeds 51-65: all props in x[-0.30,0.20], |y|<0.35",
                     "allowed": True},
    "CLUSTER_CELL": {"source": "debug seeds: 0.02 m grid separates all 7 props "
                               "(inter-prop gaps >0.03 m)", "allowed": True},
    "MILK_ZTOP_MIN": {"source": "debug seeds: carton top z=0.142; next tallest "
                                "box-shaped prop tops at 0.081", "allowed": True},
    "MILK_MINEXT": {"source": "debug seeds: carton footprint 0.050x0.053 m; bottles "
                              "measure 0.028..0.034 across the minor axis",
                    "allowed": True},
    "MILK_EXT_MAX": {"source": "debug seeds: carton bbox never exceeds 0.054 m on "
                               "either axis; the basket measures 0.157x0.171",
                     "allowed": True},
    "MILK_ZTOP_MAX": {"source": "debug seeds: the two tall bottles top out at 0.148, "
                                "the carton at 0.142", "allowed": True},
    "MILK_EXT_REF": {"source": "debug seeds 51-65: carton min bbox extent 0.050-0.052",
                     "allowed": True},
    "TIP_OFF": {"source": "v2 debug-seed table press with closed jaws: eef_z stopped "
                          "tracking at 0.009 over the bare table", "allowed": True},
    "MOVE_BIAS": {"source": "v2 debug-seed probe: eef_z settled 0.011 m above every "
                            "commanded z under a 1.2 s move", "allowed": True},
    "GRASP_DEPTH": {"source": "debug seeds: carton spans z 0.012..0.142, so fingertips "
                              "0.045 below the top sit on the straight body",
                    "allowed": True},
    "R_DOWN": {"source": "api.tool_rotation() at episode start (tool z points down)",
               "allowed": True},
    "HOLD_WIDTH_MIN": {"source": "debug-seed carton width 0.050-0.053 m; a close on air "
                                 "reads 0.001 (v2)", "allowed": True},
}

ZMIN, ZMAX = 0.012, 0.22
CELL = 0.02
MINPTS = 150
MILK_ZTOP_MIN = 0.12
MILK_MINEXT = 0.042
MILK_EXT_MAX = 0.090
MILK_ZTOP_MAX = 0.19
MILK_EXT_REF = 0.051
TIP_OFF = 0.009
GRASP_DEPTH = 0.045
HOLD_WIDTH_MIN = 0.02
CARRY_Z = 0.30


def _deproject_all(fr):
    depth = np.asarray(fr.depth, dtype=np.float64)
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    P = np.stack([(u - K[0, 2]) / K[0, 0] * depth,
                  (v - K[1, 2]) / K[1, 1] * depth, depth], -1)
    return P @ T[:3, :3].T + T[:3, 3]


def _clusters(Pb, rgb):
    m = ((Pb[..., 2] > ZMIN) & (Pb[..., 2] < ZMAX) &
         (Pb[..., 0] > -0.40) & (Pb[..., 0] < 0.40) & (np.abs(Pb[..., 1]) < 0.45))
    pts = Pb[m]
    cols = rgb[m].astype(float)
    g = np.floor(pts[:, :2] / CELL).astype(int)
    grid = defaultdict(list)
    for i, ab in enumerate(g):
        grid[(int(ab[0]), int(ab[1]))].append(i)
    seen = set()
    out = []
    for key in list(grid):
        if key in seen:
            continue
        q = deque([key])
        seen.add(key)
        comp = []
        while q:
            k = q.popleft()
            comp += grid[k]
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    nb = (k[0] + da, k[1] + db)
                    if nb in grid and nb not in seen:
                        seen.add(nb)
                        q.append(nb)
        if len(comp) < MINPTS:
            continue
        p = pts[comp]
        out.append(dict(n=len(comp), col=cols[comp].mean(0),
                        xmid=float((p[:, 0].min() + p[:, 0].max()) / 2.0),
                        ymid=float((p[:, 1].min() + p[:, 1].max()) / 2.0),
                        xext=float(p[:, 0].max() - p[:, 0].min()),
                        yext=float(p[:, 1].max() - p[:, 1].min()),
                        ztop=float(p[:, 2].max())))
    return out


def perceive(api, tag):
    fr = api.capture("cam_high")
    cl = _clusters(_deproject_all(fr), np.asarray(fr.rgb))
    for c in cl:
        api.log("%s CL n=%d xmid=%.4f ymid=%.4f ztop=%.3f xext=%.3f yext=%.3f col=%s"
                % (tag, c['n'], c['xmid'], c['ymid'], c['ztop'], c['xext'],
                   c['yext'], np.round(c['col']).astype(int).tolist()))
    if not cl:
        return None, None
    basket = max(cl, key=lambda c: c['n'])
    cand = [c for c in cl if c is not basket
            and MILK_ZTOP_MIN < c['ztop'] < MILK_ZTOP_MAX
            and min(c['xext'], c['yext']) > MILK_MINEXT
            and max(c['xext'], c['yext']) < MILK_EXT_MAX]
    milk = (min(cand, key=lambda c: abs(min(c['xext'], c['yext']) - MILK_EXT_REF))
            if cand else None)
    return milk, basket


def goto(api, xyz, R, seconds=2.0, tries=2, tol=0.004):
    """Command a pose, then cancel the steady tracking bias once."""
    tgt = np.asarray(xyz, dtype=np.float64)
    cmd = tgt.copy()
    for _ in range(tries):
        api.move(cmd.tolist(), R, seconds)
        e = np.asarray(api.eef(), dtype=np.float64)
        err = tgt - e
        if np.max(np.abs(err)) <= tol:
            break
        cmd = cmd + err
        seconds = 1.2
    api.log("GOTO tgt=%s eef=%s" % (np.round(tgt, 4).tolist(),
                                    np.round(api.eef(), 4).tolist()))


def try_grasp(api, milk, R):
    gz = milk['ztop'] - GRASP_DEPTH + TIP_OFF
    api.grip(0.08)
    api.settle(0.3)
    goto(api, [milk['xmid'], milk['ymid'], 0.25], R, 2.0, tries=1)
    goto(api, [milk['xmid'], milk['ymid'], gz], R, 2.0)
    api.grip(0.0)
    api.settle(0.5)
    g = api.gripper()
    api.log("CLOSED %s" % (g,))
    goto(api, [milk['xmid'], milk['ymid'], CARRY_Z], R, 2.0, tries=1)
    api.settle(0.3)
    g = api.gripper()
    api.log("LIFTED %s eef=%s" % (g, np.round(api.eef(), 4).tolist()))
    return float(g['width_m']) > HOLD_WIDTH_MIN


def run(api):
    R = np.asarray(api.tool_rotation(), dtype=np.float64)
    milk, basket = perceive(api, "P0")
    if milk is None or basket is None:
        api.log("NO TARGET")
        return
    api.log("MILK %.4f %.4f ztop %.3f | BASKET %.4f %.4f ztop %.3f"
            % (milk['xmid'], milk['ymid'], milk['ztop'],
               basket['xmid'], basket['ymid'], basket['ztop']))

    held = try_grasp(api, milk, R)
    if not held:
        api.log("RETRY")
        goto(api, [milk['xmid'], milk['ymid'], 0.30], R, 2.0, tries=1)
        m2, b2 = perceive(api, "P1")
        if m2 is not None:
            milk = m2
        held = try_grasp(api, milk, R)
    api.log("HELD %s" % held)

    # carry over the basket, lower the carton inside, release
    goto(api, [basket['xmid'], basket['ymid'], CARRY_Z], R, 3.0, tries=1)
    api.settle(0.3)
    api.log("OVER %s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))
    goto(api, [basket['xmid'], basket['ymid'], 0.215], R, 2.0, tries=1)
    api.grip(0.08)
    api.settle(0.8)
    api.log("RELEASED %s" % (api.gripper(),))
    goto(api, [basket['xmid'], basket['ymid'], 0.30], R, 2.0, tries=1)
    api.settle(0.5)
