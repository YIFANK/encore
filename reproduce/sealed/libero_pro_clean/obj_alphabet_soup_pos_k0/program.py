"""v2: perceive blue can + basket, top-down grasp near the can's upper body, carry, release."""
import numpy as np

PROVENANCE = {
    "Z_TABLE": {"source": "debug seed 51-65 cam_high cloud: table plane z=0.001 base frame", "allowed": True},
    "WS_BOUNDS": {"source": "debug seed 51-65: all props inside |x|<0.45,|y|<0.5, z<0.35", "allowed": True},
    "CAN_SHAPE_GATE": {"source": "debug seed 51-65 clusters: the two cans have ztop=0.081 and top spans 0.063/0.070 m; all other props differ (ztop 0.019/0.142/0.144/0.148)", "allowed": True},
    "BLUE_CAN_CUE": {"source": "debug seed 51-65 cluster mean rgb: candidate cans have B-R=+7.4 (blue label) and -13.3 (warm label); target picked as max B-R", "allowed": True},
    "TIP_OFFSET": {"source": "v1 debug probe: open-finger press onto bare table stalled at eef z=0.0095 at two separate spots (seeds 51,53)", "allowed": True},
    "GRASP_DEPTH": {"source": "v1 can ztop=0.081 + TIP_OFFSET; grip 0.020 m below the can top", "allowed": True},
    "BASKET_GATE": {"source": "debug seed 51-65: basket is the only cluster with n>8000 and ztop=0.144", "allowed": True},
    "CARRY_Z": {"source": "v1 can height 0.081 with a 0.020 grip depth puts the can bottom 0.061 below the tips; basket rim 0.144 -> carry eef z 0.25", "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def cloud(fr):
    d = np.nan_to_num(np.asarray(fr.depth, dtype=np.float64), nan=0.0)
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    H, W = d.shape
    v, u = np.mgrid[0:H, 0:W]
    pc = np.stack([(u - K[0, 2]) / K[0, 0] * d, (v - K[1, 2]) / K[1, 1] * d, d], -1)
    return pc @ T[:3, :3].T + T[:3, 3]


def label(mask):
    try:
        from scipy import ndimage
        return ndimage.label(mask)
    except Exception:
        pass
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    cur = 0
    seen = mask.copy()
    for sy, sx in np.transpose(np.nonzero(mask)):
        if not seen[sy, sx]:
            continue
        cur += 1
        stack = [(sy, sx)]
        seen[sy, sx] = False
        while stack:
            y, x = stack.pop()
            lab[y, x] = cur
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < H and 0 <= nx < W and seen[ny, nx]:
                    seen[ny, nx] = False
                    stack.append((ny, nx))
    return lab, cur


def clusters(api, zlo=0.012, zhi=0.35, tag=""):
    fr = api.capture("cam_high")
    rgb = np.asarray(fr.rgb, dtype=np.float64)
    P = cloud(fr)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (X > -0.45) & (X < 0.45) & (Y > -0.5) & (Y < 0.5) & (Z > zlo) & (Z < zhi)
    lab, n = label(m)
    out = []
    for i in range(1, n + 1):
        s = lab == i
        cnt = int(s.sum())
        if cnt < 200:
            continue
        zt = float(Z[s].max())
        top = s & (Z > zt - 0.008)
        c = rgb[s].mean(0)
        out.append(dict(n=cnt, ztop=zt,
                        tx=float(0.5 * (X[top].min() + X[top].max())),
                        ty=float(0.5 * (Y[top].min() + Y[top].max())),
                        xspan=float(X[top].max() - X[top].min()),
                        yspan=float(Y[top].max() - Y[top].min()),
                        bmr=float(c[2] - c[0])))
    for c in out:
        api.log("cluster%s n=%d ztop=%.3f tx=%.3f ty=%.3f sp=%.3f/%.3f bmr=%.1f"
                % (tag, c["n"], c["ztop"], c["tx"], c["ty"], c["xspan"], c["yspan"], c["bmr"]))
    return out


def can_gate(cl):
    return [c for c in cl if 0.06 < c["ztop"] < 0.11
            and 0.045 < c["xspan"] < 0.09 and 0.045 < c["yspan"] < 0.09]


def find_target(api):
    """Blue-label can. Primary: whole-prop clustering. Fallback: can-height band
    mask, which survives a footprint fusion with a taller neighbour."""
    cl = clusters(api)
    cans = can_gate(cl)
    if cans:
        return max(cans, key=lambda c: c["bmr"]), cl
    api.log("primary gate empty -> band fallback")
    band = clusters(api, zlo=0.055, zhi=0.095, tag="_band")
    cans = [c for c in band if c["ztop"] > 0.06 and 0.045 < c["xspan"] < 0.09 and 0.045 < c["yspan"] < 0.09]
    if cans:
        return max(cans, key=lambda c: c["bmr"]), cl
    return None, cl


def goto(api, xyz, tol=0.006, tries=4, seconds=2.0):
    r = None
    for _ in range(tries):
        r = api.move(list(xyz), rotation=R_DOWN, seconds=seconds)
        e = np.asarray(api.eef(), dtype=np.float64)
        if float(np.linalg.norm(e - np.asarray(xyz))) < tol:
            break
    api.log("goto %s -> eef %s resid %s" % (np.round(xyz, 4).tolist(),
                                            np.round(np.asarray(api.eef()), 4).tolist(), r))
    return np.asarray(api.eef(), dtype=np.float64)


TIP = 0.0095          # eef z when the open fingertips stall on the bare table (v1)
GRIP_DEPTH = 0.020    # jaws bite this far below the can top
CARRY_Z = 0.25


def attempt_grasp(api, tgt, depth):
    api.grip(0.08)
    api.settle(0.2)
    goto(api, [tgt["tx"], tgt["ty"], 0.20], tol=0.006, tries=4)
    goto(api, [tgt["tx"], tgt["ty"], 0.12], tol=0.005, tries=3)
    goto(api, [tgt["tx"], tgt["ty"], tgt["ztop"] - depth + TIP], tol=0.004, tries=4, seconds=1.5)
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("closed %s" % g)
    goto(api, [tgt["tx"], tgt["ty"], CARRY_Z], tol=0.008, tries=4)
    g = api.gripper()
    api.log("lifted %s" % g)
    # a real can in the jaws holds them apart: effort flag plus a plausible width
    held = (float(g["effort"]) > 1.0) and (0.03 < float(g["width_m"]) < 0.078)
    return held


def run(api):
    tgt, cl = find_target(api)
    baskets = [c for c in cl if c["n"] > 8000 and 0.11 < c["ztop"] < 0.20]
    if tgt is None:
        api.log("NO CAN")
        return
    api.log("target tx=%.3f ty=%.3f ztop=%.3f bmr=%.1f | nbask=%d"
            % (tgt["tx"], tgt["ty"], tgt["ztop"], tgt["bmr"], len(baskets)))
    if not baskets:
        api.log("NO BASKET")
        return
    bk = max(baskets, key=lambda c: c["n"])

    held = attempt_grasp(api, tgt, GRIP_DEPTH)
    if not held:
        api.log("retry: re-perceive after miss")
        goto(api, [tgt["tx"], tgt["ty"], 0.25], tol=0.02, tries=1)
        t2, _ = find_target(api)
        if t2 is not None:
            tgt = t2
        held = attempt_grasp(api, tgt, GRIP_DEPTH + 0.012)
    api.log("held=%s" % held)

    goto(api, [bk["tx"], bk["ty"], CARRY_Z], tol=0.010, tries=5, seconds=2.5)
    api.log("over_basket %s" % api.gripper())
    api.grip(0.08)
    api.settle(0.5)
    api.log("released %s" % api.gripper())
    goto(api, [bk["tx"], bk["ty"], 0.28], tol=0.02, tries=2)
    clusters(api, tag="_final")
    api.log("end")
