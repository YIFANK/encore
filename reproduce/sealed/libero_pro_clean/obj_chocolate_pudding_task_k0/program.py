"""v5 (= v4 plus a closed-loop cancellation of the descent tracking bias): perceive the green-capped bottle + the basket from cam_high, grasp the
bottle by its cap/neck column, carry it over the basket and release.

Everything is re-derived at runtime from the cam_high point cloud; the only
baked constants are the workspace crop, the table plane, the fingertip offset
(measured in v2b) and the grasp depth below the cap top.
"""
import numpy as np

PROVENANCE = {
    "XLO": {"source": "debug-seed cam_high point cloud (v1b): crop that excludes the robot column (x<-0.06)", "allowed": True},
    "XHI": {"source": "debug-seed cam_high point cloud (v1b): far table edge", "allowed": True},
    "YLO": {"source": "debug-seed cam_high point cloud (v1b): table extent", "allowed": True},
    "YHI": {"source": "debug-seed cam_high point cloud (v1b): table extent", "allowed": True},
    "ZLO": {"source": "debug-seed cam_high point cloud (v1b): table plane z=0.0012, cropped 20 mm above it", "allowed": True},
    "ZHI": {"source": "debug-seed cam_high point cloud (v1b): above every prop top (0.148) and below the parked arm (0.40)", "allowed": True},
    "CELL": {"source": "generic: 15 mm xy binning for footprint clustering", "allowed": True},
    "GREEN_T": {"source": "debug-seed cam_high RGB (v1b): cap band G-max(R,B) separates 0.073 (target) from -0.13..-0.006 (others)", "allowed": True},
    "TIP_OFFSET": {"source": "debug-seed measurement (v2b): closed fingertips stalled on the table at eef z=0.0095, table z=0.0012", "allowed": True},
    "GRASP_DEPTH": {"source": "debug-seed cam_high point cloud (v1b): target neck column is 35 mm wide from z=0.09 to the 0.1475 top; 25 mm below the top sits mid-cap", "allowed": True},
    "CARRY_Z": {"source": "debug-seed measurement: prop base hangs GRASP_DEPTH+ below the fingertips; basket rim measured at 0.144", "allowed": True},
    "AIM_TOL": {"source": "generic controller mechanics: closed-loop lateral correction threshold", "allowed": True},
    "RELEASE_Z": {"source": "debug-seed measurement: basket rim 0.144 + hang length, plus clearance", "allowed": True},
}

XLO, XHI = -0.06, 0.45
YLO, YHI = -0.50, 0.50
ZLO, ZHI = 0.020, 0.30
CELL = 0.015
GREEN_T = 0.04
TIP_OFFSET = 0.0083
GRASP_DEPTH = 0.025
CARRY_Z = 0.34
RELEASE_Z = 0.30
AIM_TOL = 0.004


# ---------------------------------------------------------------- perception
def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    d = np.asarray(f.depth, dtype=np.float64)
    K = np.asarray(f.intrinsics, dtype=np.float64)
    T = np.asarray(f.t_base_cam, dtype=np.float64)
    H, W = d.shape
    v, u = np.mgrid[0:H, 0:W]
    X = (u - K[0, 2]) / K[0, 0] * d
    Y = (v - K[1, 2]) / K[1, 1] * d
    P = np.stack([X, Y, d], -1)
    Pb = P @ T[:3, :3].T + T[:3, 3]
    return Pb, np.asarray(f.rgb, dtype=np.float64) / 255.0


def clusters(Pb, rgb):
    x, y, z = Pb[..., 0], Pb[..., 1], Pb[..., 2]
    m = (x > XLO) & (x < XHI) & (y > YLO) & (y < YHI) & (z > ZLO) & (z < ZHI)
    xs, ys, zs, cs = x[m], y[m], z[m], rgb[m]
    nx = int((XHI - XLO) / CELL) + 1
    ny = int((YHI - YLO) / CELL) + 1
    ix = ((xs - XLO) / CELL).astype(int)
    iy = ((ys - YLO) / CELL).astype(int)
    flat = ix * ny + iy
    hmap = np.full(nx * ny, -1.0)
    np.maximum.at(hmap, flat, zs)
    occ = (hmap > 0).reshape(nx, ny)
    lab = -np.ones((nx, ny), int)
    ncomp = 0
    for a in range(nx):
        for b in range(ny):
            if not occ[a, b] or lab[a, b] >= 0:
                continue
            stack = [(a, b)]
            lab[a, b] = ncomp
            while stack:
                p, q = stack.pop()
                for dp in (-1, 0, 1):
                    for dq in (-1, 0, 1):
                        r, s = p + dp, q + dq
                        if 0 <= r < nx and 0 <= s < ny and occ[r, s] and lab[r, s] < 0:
                            lab[r, s] = ncomp
                            stack.append((r, s))
            ncomp += 1
    labflat = lab.reshape(-1)[flat]
    out = []
    for cid in range(ncomp):
        sel = labflat == cid
        n = int(sel.sum())
        if n < 40:
            continue
        cx, cy, cz, col = xs[sel], ys[sel], zs[sel], cs[sel]
        top = float(cz.max())
        g = col[:, 1] - np.maximum(col[:, 0], col[:, 2])
        capband = cz > top - 0.030
        xyband = cz > top - 0.012
        out.append(dict(
            n=n, top=top,
            dx=float(cx.max() - cx.min()), dy=float(cy.max() - cy.min()),
            gfrac=float((g[capband] > GREEN_T).mean()),
            bx=float((cx[xyband].min() + cx[xyband].max()) / 2),
            by=float((cy[xyband].min() + cy[xyband].max()) / 2),
            cx=float((cx.min() + cx.max()) / 2), cy=float((cy.min() + cy.max()) / 2),
        ))
    return out


def select(api, out):
    for c in sorted(out, key=lambda c: -c["n"]):
        api.log("CL n=%5d top=%.3f dx=%.3f dy=%.3f gfrac=%.2f b=(%.3f,%.3f)"
                % (c["n"], c["top"], c["dx"], c["dy"], c["gfrac"], c["bx"], c["by"]))
    low = [c for c in out if c["top"] < 0.25]
    small = [c for c in low if max(c["dx"], c["dy"]) < 0.10]
    big = [c for c in low if max(c["dx"], c["dy"]) >= 0.10]
    tgt = max(small, key=lambda c: c["gfrac"]) if small else None
    bas = max(big, key=lambda c: c["n"]) if big else None
    return tgt, bas


# ------------------------------------------------------------------- control
def goto(api, xyz, R, tag, tol=0.006, tries=4, seconds=2.0):
    res = 9.0
    for i in range(tries):
        res = api.move(list(xyz), rotation=R, seconds=seconds)
        if res < tol:
            break
    e = np.asarray(api.eef())
    api.log("GOTO %s cmd=%s res=%.4f eef=%s iters=%d"
            % (tag, [round(v, 4) for v in xyz], res, e.round(4).tolist(), i + 1))
    return res, e


def run(api):
    api.log("INSTRUCTION %s" % api.instruction())
    R = np.asarray(api.tool_rotation(), dtype=float)
    api.grip(0.08)
    api.settle(0.2)

    Pb, rgb = cloud(api)
    tgt, bas = select(api, clusters(Pb, rgb))
    if tgt is None or bas is None:
        api.log("PERCEPTION FAILED")
        return
    api.log("TARGET b=(%.4f,%.4f) top=%.4f gfrac=%.2f" % (tgt["bx"], tgt["by"], tgt["top"], tgt["gfrac"]))
    api.log("BASKET c=(%.4f,%.4f) top=%.4f" % (bas["cx"], bas["cy"], bas["top"]))

    gz = tgt["top"] - GRASP_DEPTH + TIP_OFFSET
    goto(api, [tgt["bx"], tgt["by"], 0.26], R, "hover", tol=0.012, tries=3)
    goto(api, [tgt["bx"], tgt["by"], gz], R, "descend", tol=0.006, tries=3)
    # cancel the steady-state lateral tracking bias of the descent: re-issue the
    # command shifted by the measured error (the jaws are still 22 mm clear of
    # the neck at this height, so the correction cannot disturb the prop).
    cx, cy = tgt["bx"], tgt["by"]
    for _ in range(2):
        e = np.asarray(api.eef())
        ex, ey = e[0] - tgt["bx"], e[1] - tgt["by"]
        api.log("AIM_ERR ex=%+.4f ey=%+.4f" % (ex, ey))
        if max(abs(ex), abs(ey)) < AIM_TOL:
            break
        cx, cy = cx - ex, cy - ey
        api.move([cx, cy, gz], rotation=R, seconds=1.5)
    e = np.asarray(api.eef())
    api.log("GRASP_POSE eef=%s aim=(%.4f,%.4f)" % (e.round(4).tolist(), tgt["bx"], tgt["by"]))
    api.grip(0.0)
    api.settle(0.4)
    api.log("CLOSED %s" % api.gripper())

    goto(api, [cx, cy, CARRY_Z], R, "lift", tol=0.015, tries=3)
    api.log("AFTER_LIFT %s" % api.gripper())

    goto(api, [bas["cx"], bas["cy"], CARRY_Z], R, "over_basket", tol=0.012, tries=4)
    goto(api, [bas["cx"], bas["cy"], RELEASE_Z], R, "release_pose", tol=0.015, tries=2)
    api.log("BEFORE_RELEASE %s" % api.gripper())
    api.grip(0.08)
    api.settle(0.6)
    api.log("RELEASED %s" % api.gripper())
