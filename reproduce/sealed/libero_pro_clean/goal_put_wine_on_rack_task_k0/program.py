import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug-seed 51 cam_high deprojected table points -> 0.901 m", "allowed": True},
    "BLUE_RULE": {"source": "debug seeds 51/53/57/61 cam_high RGB-D: the only blue-dominant patch "
                            "in the 0.905-0.98 m table band is the cream-cheese box", "allowed": True},
    "GRASP_Z_CMD": {"source": "debug seeds 51/61 v3 descend-and-close ladder: closes empty at cmd "
                              "z>=0.915, holds (w 0.0422, effort 3.0) at cmd z 0.905", "allowed": True},
    "EEF_TO_BOX_BOTTOM": {"source": "v3: eef z 0.915 when box bottom rests on table 0.901 -> 0.014 m",
                          "allowed": True},
    "MOVE_BIAS": {"source": "v3: commanded z lands ~0.010 m high (res ~0.010) on every rung",
                  "allowed": True},
    "RACK_WINDOW": {"source": "debug seeds 51/53/57/61 cam_high: the only structure above 1.10 m in "
                              "x[-0.45,-0.11] y[-0.45,-0.10] is the slatted rack", "allowed": True},
    "CARRY_Z": {"source": "debug seed 51 height map: cabinet top 1.13 m, rack ridge 1.245 m", "allowed": True},
}

TABLE_Z = 0.901
BIAS = 0.010
EEF_TO_BOTTOM = 0.014
CARRY_Z = 1.31


def _cloud(api, cam="cam_high"):
    fr = api.capture(cam)
    d = np.nan_to_num(np.asarray(fr.depth, dtype=np.float64), nan=0.0)
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    H, W = d.shape
    vv, uu = np.meshgrid(np.arange(H, dtype=np.float64), np.arange(W, dtype=np.float64), indexing="ij")
    X = (uu - K[0, 2]) / K[0, 0] * d
    Y = (vv - K[1, 2]) / K[1, 1] * d
    P = np.stack([X, Y, d], -1) @ T[:3, :3].T + T[:3, 3]
    return np.asarray(fr.rgb, dtype=np.int32), P, d


def blue_mask(rgb, P, d, zlo, zhi, xw, yw):
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    h = P[..., 2]
    return ((b > r + 12) & (b > g + 8) & (d > 0) & (h > zlo) & (h < zhi)
            & (P[..., 0] > xw[0]) & (P[..., 0] < xw[1]) & (P[..., 1] > yw[0]) & (P[..., 1] < yw[1]))


def find_box(api):
    rgb, P, d = _cloud(api)
    m = blue_mask(rgb, P, d, TABLE_Z + 0.004, TABLE_Z + 0.08, (-0.45, 0.25), (-0.10, 0.45))
    p = P[m]
    api.log("BOXPIX %d" % int(m.sum()))
    if p.shape[0] < 30:
        return None
    lo, hi, ly, hy = p[:, 0].min(), p[:, 0].max(), p[:, 1].min(), p[:, 1].max()
    api.log("BOX x[%.3f,%.3f] y[%.3f,%.3f] ztop %.3f" % (lo, hi, ly, hy, np.percentile(p[:, 2], 95)))
    return float((lo + hi) / 2.0), float((ly + hy) / 2.0)


def find_rack(api):
    rgb, P, d = _cloud(api)
    pts = P.reshape(-1, 3)
    m = ((pts[:, 0] > -0.45) & (pts[:, 0] < -0.11) & (pts[:, 1] > -0.45) & (pts[:, 1] < -0.10)
         & (pts[:, 2] > 1.10) & (pts[:, 2] < 1.40))
    p = pts[m]
    api.log("RACKPIX %d" % len(p))
    if len(p) < 300:
        return None
    A = np.c_[p[:, 0], p[:, 1], np.ones(len(p))]
    co = np.linalg.lstsq(A, p[:, 2], rcond=None)[0]
    xw = (float(p[:, 0].min()), float(p[:, 0].max()))
    yw = (float(p[:, 1].min()), float(p[:, 1].max()))
    api.log("RACK plane %.4f %.4f %.4f x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f"
            % (co[0], co[1], co[2], xw[0], xw[1], yw[0], yw[1], p[:, 2].max()))
    return co, xw, yw


def run(api):
    api.log("INSTR %s" % api.instruction())
    api.grip(0.08)
    api.settle(0.3)
    b = find_box(api)
    rk = find_rack(api)
    if b is None or rk is None:
        api.log("PERCEPT_FAIL")
        return
    bx, by = b
    co, xw, yw = rk

    api.move([bx, by, 1.05])
    held = False
    for zc in [0.905, 0.895, 0.915]:
        api.move([bx, by, zc], seconds=1.5)
        api.grip(0.0)
        api.settle(0.4)
        gs = api.gripper()
        api.log("CLOSE zc %.3f w %.4f eff %.2f" % (zc, gs["width_m"], gs["effort"]))
        if gs["width_m"] > 0.020:
            held = True
            break
        api.grip(0.08)
        api.settle(0.3)
    if not held:
        api.log("NO_GRASP")
        return
    api.move([bx, by, 1.12], seconds=1.5)
    gs = api.gripper()
    api.log("LIFT w %.4f eff %.2f" % (gs["width_m"], gs["effort"]))

    px = float(np.clip((xw[0] + xw[1]) / 2.0, -0.36, -0.16))
    py = float((yw[0] + yw[1]) / 2.0)
    surf = co[0] * px + co[1] * py + co[2]
    api.log("PLACE px %.3f py %.3f surf %.3f" % (px, py, surf))

    api.move([bx, by, CARRY_Z], seconds=2.0)
    api.move([px, py, CARRY_Z], seconds=2.5)
    gs = api.gripper()
    api.log("CARRY w %.4f eff %.2f eef %s" % (gs["width_m"], gs["effort"], list(np.round(api.eef(), 3))))
    zrel = surf + EEF_TO_BOTTOM + 0.008
    api.move([px, py, zrel - BIAS], seconds=2.5)
    api.log("AT_REL eef %s" % list(np.round(api.eef(), 3)))
    api.grip(0.08)
    api.settle(0.8)
    api.move([px, py, CARRY_Z], seconds=2.0)
    api.settle(1.0)

    rgb, P, d = _cloud(api)
    m = blue_mask(rgb, P, d, 0.90, 1.40, (-0.55, 0.35), (-0.55, 0.45))
    p = P[m]
    if len(p) > 20:
        api.log("AFTER blue n %d ctr %s zmax %.3f" % (len(p), list(np.round(p.mean(0), 3)), p[:, 2].max()))
    else:
        api.log("AFTER blue n %d (lost)" % len(p))
