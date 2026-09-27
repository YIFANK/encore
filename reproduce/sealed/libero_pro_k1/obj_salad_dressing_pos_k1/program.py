"""v2 -- perceive the salad-dressing bottle by colour+shape, pick it by the
neck, drop it in the basket.  Also logs the full candidate table and the
target's vertical width profile so the next version can be calibrated.

Scene facts established on debug seeds 51/53/55/57 with v0 (perception-only):
table plane at base z ~ 0.001; the parked arm forms one tall cluster
(h ~ 0.40) at x ~ -0.124; the basket is a large bright cluster
(n ~ 1.2e4, extent ~0.16 x 0.17 m, top 0.141 m); the props are four to six
small clusters of height 0.02 - 0.15 m.  The demo pack's grasp xy does NOT
name the target in these scenes (the demo scene holds a different prop set),
so the target is chosen by appearance, not by the demo anchor.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-57: modal z of the cam_high cloud (v0 log 'table_z~0.0009')", "allowed": True},
    "ABOVE_M": {"source": "debug 51-57: 0.012 m clears table depth noise (v0 histogram)", "allowed": True},
    "CELL_M": {"source": "generic: 0.02 m grid for connected-component clustering", "allowed": True},
    "ARM_H_MAX": {"source": "debug 51-57: parked arm cluster h=0.396 vs props <=0.147 (v0 log)", "allowed": True},
    "PROP_EXT_MAX": {"source": "debug 51-57: props ext<=0.081 m, basket ext 0.16x0.17 (v0 log)", "allowed": True},
    "NECK_BELOW_TOP": {"source": "pack demo0: grasp ee z 0.1238 vs a 0.146 m tall bottle top -> ~0.023 under the cap", "allowed": True},
    "GRASP_WIDTH": {"source": "pack demo0 t=128 gripper_state [0.0091,-0.0089] -> ~0.018 m held width", "allowed": True},
    "HOVER_Z": {"source": "debug 51-57: 0.28 m clears every prop top (max 0.147) and the basket rim 0.141", "allowed": True},
    "RELEASE_Z": {"source": "pack demo0 t=128 release ee z 0.1709 over the basket", "allowed": True},
    "BASKET_RIM_BAND": {"source": "debug 51-57: basket top 0.141 m; 0.020 m band isolates the rim ring", "allowed": True},
    "CARRY_Z": {"source": "pack demo0 t=136 retreat ee z 0.2092; 0.30 adds clearance over the basket rim 0.141", "allowed": True},
}

TABLE_Z = 0.001
ABOVE_M = 0.012
CELL_M = 0.02
ARM_H_MAX = 0.25
PROP_EXT_MAX = 0.11
NECK_BELOW_TOP = 0.025
HOVER_Z = 0.28
CARRY_Z = 0.30
RELEASE_Z = 0.19


def _cloud(frame):
    d = frame.depth
    h, w = d.shape[:2]
    K = frame.intrinsics
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:h, 0:w]
    z = d.astype(np.float64)
    ok = np.isfinite(z) & (z > 0)
    zz = np.where(ok, z, 1.0)
    x = (uu - cx) * zz / fx
    y = (vv - cy) * zz / fy
    pc = np.stack([x, y, zz, np.ones_like(zz)], axis=-1)
    P = (pc.reshape(-1, 4) @ frame.t_base_cam.T)[:, :3].reshape(h, w, 3)
    return P, ok


def _clusters(frame):
    P, ok = _cloud(frame)
    X, Y, Z = P[:, :, 0], P[:, :, 1], P[:, :, 2]
    ws = ok & (X > -0.35) & (X < 0.42) & (Y > -0.45) & (Y < 0.45)
    above = ws & (Z > TABLE_Z + ABOVE_M) & (Z < TABLE_Z + 0.45)
    idx = np.argwhere(above)
    if idx.size == 0:
        return []
    pts = P[above]
    cols = frame.rgb[above].astype(np.float64)
    gx = np.floor(pts[:, 0] / CELL_M).astype(int)
    gy = np.floor(pts[:, 1] / CELL_M).astype(int)
    keys = {}
    for i in range(len(pts)):
        keys.setdefault((int(gx[i]), int(gy[i])), []).append(i)
    parent = {k: k for k in keys}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for k in list(keys):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                n = (k[0] + dx, k[1] + dy)
                if n in keys:
                    ra, rb = find(k), find(n)
                    if ra != rb:
                        parent[ra] = rb
    groups = {}
    for k in keys:
        groups.setdefault(find(k), []).extend(keys[k])
    out = []
    for ids in groups.values():
        if len(ids) < 120:
            continue
        p = pts[ids]
        c = cols[ids]
        uv = idx[ids]
        ztop = float(np.percentile(p[:, 2], 98))
        out.append(dict(
            n=len(ids), p=p, c=c,
            ztop=ztop, h=ztop - TABLE_Z,
            xmin=float(p[:, 0].min()), xmax=float(p[:, 0].max()),
            ymin=float(p[:, 1].min()), ymax=float(p[:, 1].max()),
            ex=float(p[:, 0].max() - p[:, 0].min()),
            ey=float(p[:, 1].max() - p[:, 1].min()),
            r=float(c[:, 0].mean()), g=float(c[:, 1].mean()), b=float(c[:, 2].mean()),
            u=float(uv[:, 1].mean()), v=float(uv[:, 0].mean()),
        ))
    return out


def _centre(cl, zlo, zhi):
    """Circular-cross-section centre of the band [zlo, zhi] of a cluster.
    The camera sees only the near arc, so x is recovered from the near
    surface minus the half-width measured across y (fully visible)."""
    p = cl["p"]
    m = (p[:, 2] >= zlo) & (p[:, 2] <= zhi)
    if m.sum() < 20:
        return None
    q = p[m]
    ylo, yhi = float(np.percentile(q[:, 1], 2)), float(np.percentile(q[:, 1], 98))
    cy = 0.5 * (ylo + yhi)
    rad = max(0.006, 0.5 * (yhi - ylo))
    xnear = float(np.percentile(q[:, 0], 98))
    return np.array([xnear - rad, cy, 0.0]), rad, int(m.sum())


def run(api):
    api.log("=== v2 ===")
    frame = api.capture("cam_high")
    cls = _clusters(frame)
    cls.sort(key=lambda d: -d["n"])
    props = []
    for d in cls:
        tag = "?"
        if d["h"] > ARM_H_MAX:
            tag = "ARM"
        elif d["ex"] > PROP_EXT_MAX or d["ey"] > PROP_EXT_MAX:
            tag = "BASKET"
        else:
            tag = "prop"
            props.append(d)
        d["tag"] = tag
        api.log("%-6s n=%4d xy=(%.3f,%.3f) h=%.3f ext=(%.3f,%.3f) rgb=(%3.0f,%3.0f,%3.0f) grn=%+5.1f px=(%.0f,%.0f)"
                % (tag, d["n"], np.median(d["p"][:, 0]), np.median(d["p"][:, 1]),
                   d["h"], d["ex"], d["ey"], d["r"], d["g"], d["b"],
                   d["g"] - 0.5 * (d["r"] + d["b"]), d["u"], d["v"]))

    baskets = [d for d in cls if d["tag"] == "BASKET"]
    if not baskets:
        api.log("NO BASKET -> abort")
        return
    basket = max(baskets, key=lambda d: d["n"])
    # Centre on the RIM band only: the rim is a closed rectangle seen from
    # above, so its x/y midpoints are the true centre.  The whole-cluster
    # midpoint is biased by the near outer wall and the interior floor.
    bp = basket["p"]
    rim = bp[bp[:, 2] > basket["ztop"] - 0.020]
    if rim.shape[0] < 200:
        rim = bp
    bx = 0.5 * (float(np.percentile(rim[:, 0], 2)) + float(np.percentile(rim[:, 0], 98)))
    by = 0.5 * (float(np.percentile(rim[:, 1], 2)) + float(np.percentile(rim[:, 1], 98)))
    api.log("basket rim n=%d centre (%.3f,%.3f) top %.3f allmid (%.3f,%.3f)"
            % (rim.shape[0], bx, by, basket["ztop"],
               0.5 * (basket["xmin"] + basket["xmax"]),
               0.5 * (basket["ymin"] + basket["ymax"])))

    # --- target: the green bottle.  Tall (a bottle, not a flat box) and the
    # greenest of the tall props.  greenness = g - (r+b)/2 on the cluster mean.
    tall = [d for d in props if d["h"] > 0.06]
    if not tall:
        api.log("NO TALL PROP -> abort")
        return
    for d in tall:
        d["grn"] = d["g"] - 0.5 * (d["r"] + d["b"])
    tgt = max(tall, key=lambda d: d["grn"])
    api.log("TARGET grn=%+.1f h=%.3f ext=(%.3f,%.3f) n=%d"
            % (tgt["grn"], tgt["h"], tgt["ex"], tgt["ey"], tgt["n"]))

    # vertical width profile (diagnostic: where is the neck?)
    p = tgt["p"]
    prof = []
    z0 = TABLE_Z + ABOVE_M
    zt = tgt["ztop"]
    zb = z0
    while zb < zt:
        m = (p[:, 2] >= zb) & (p[:, 2] < zb + 0.01)
        if m.sum() >= 10:
            q = p[m]
            prof.append("%.2f:%.3f/%d" % (zb, float(q[:, 1].max() - q[:, 1].min()), int(m.sum())))
        zb += 0.01
    api.log("profile(z:ywidth/n) " + " ".join(prof))

    band = _centre(tgt, zt - 0.045, zt - 0.008)
    if band is None:
        band = _centre(tgt, zt - 0.06, zt)
    if band is None:
        api.log("no band -> abort")
        return
    c, rad, nb = band
    tx, ty = float(c[0]), float(c[1])
    gz = zt - NECK_BELOW_TOP
    api.log("grasp (%.3f,%.3f,%.3f) rad=%.3f nb=%d" % (tx, ty, gz, rad, nb))

    api.grip(0.08)
    api.move([tx, ty, HOVER_Z], seconds=2.0)
    r = api.move([tx, ty, gz], seconds=2.0)
    api.log("descend residual %.4f eef %s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("after close %s" % g)
    api.move([tx, ty, CARRY_Z], seconds=2.0)
    g = api.gripper()
    api.log("after lift %s eef %s" % (g, np.round(api.eef(), 4).tolist()))
    api.move([bx, by, CARRY_Z], seconds=3.0)
    api.move([bx, by, RELEASE_Z], seconds=2.0)
    api.log("over basket eef %s grip %s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(0.08)
    api.settle(0.4)
    api.move([bx, by, CARRY_Z], seconds=2.0)
    api.log("=== end v2 ===")
