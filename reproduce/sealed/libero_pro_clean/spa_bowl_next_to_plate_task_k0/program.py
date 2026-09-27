"""v5 -- v4 with the component-labelling fix: the tall-structure cut is
applied to a component TOP, not inside the mask (masking on z first slices
the cabinet into prop-sized blobs)."""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-57 cam_high depth: modal z of the "
                          "workspace point cloud = 0.900", "allowed": True},
    "TIP_DZ": {"source": "debug seed 51 blocked-descent probe (v3): open fingers "
                         "stall the eef at z=0.9086 on the 0.900 table -> tips "
                         "sit 0.0086 below the eef origin", "allowed": True},
    "H_MIN": {"source": "debug-seed height histogram: table noise < 0.012 m",
              "allowed": True},
    "WS": {"source": "debug-seed observation: props lie in x(-0.30,0.30) "
                     "y(-0.50,0.50)", "allowed": True},
    "MIN_PX": {"source": "debug seeds 51-57 component sizes: props >= 1400 px, "
                         "speckle < 600", "allowed": True},
    "Z_ARM": {"source": "debug seeds 51-57: arm/cabinet components top out at "
                        "1.13-1.37, props at <= 0.952", "allowed": True},
    "VESSEL_H": {"source": "debug seeds 51-57 component tops: bowls 0.052, "
                           "ramekin 0.044, plate 0.020, box 0.021 above table",
                 "allowed": True},
    "GRASP_DEPTH": {"source": "chosen from the measured bowl height (0.052) -- "
                              "tips 0.018 below the rim top", "allowed": True},
    "CARRY_Z": {"source": "measured prop tops (max 0.952) + held-bowl hang",
                "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics: straight-down tool "
                         "frame; matches the measured start tool_rotation",
               "allowed": True},
}

TABLE_Z = 0.900
TIP_DZ = 0.0086
H_MIN = 0.012
MIN_PX = 600
Z_ARM = 1.00
VESSEL_H = 0.030
GRASP_DEPTH = 0.018
CARRY_Z = 1.03
R_DOWN = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])


def _label(mask):
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    seen = set(map(tuple, np.argwhere(mask)))
    order = sorted(seen)
    for p in order:
        if p not in seen:
            continue
        cur += 1
        stack = [p]
        seen.discard(p)
        while stack:
            r, c = stack.pop()
            lab[r, c] = cur
            for q in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                if q in seen:
                    seen.discard(q)
                    stack.append(q)
    return lab, cur


def perceive(api):
    f = api.capture("cam_high")
    d = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]
    P = np.stack([(uu - K[0, 2]) * d / K[0, 0],
                  (vv - K[1, 2]) * d / K[1, 1], d], -1)
    X = P @ T[:3, :3].T + T[:3, 3]
    rgb = np.asarray(f.rgb, float)
    m = ((X[..., 2] - TABLE_Z > H_MIN)
         & (X[..., 0] > -0.30) & (X[..., 0] < 0.30) & (np.abs(X[..., 1]) < 0.50))
    lab, n = _label(m)
    props = []
    for i in range(1, n + 1):
        sel = lab == i
        if sel.sum() < MIN_PX:
            continue
        xs, ys, zs = X[..., 0][sel], X[..., 1][sel], X[..., 2][sel]
        if zs.max() > Z_ARM:
            continue
        p = {"px": int(sel.sum()), "x0": xs.min(), "x1": xs.max(),
             "y0": ys.min(), "y1": ys.max(), "ztop": zs.max(),
             "cx": (xs.min() + xs.max()) / 2, "cy": (ys.min() + ys.max()) / 2,
             "rgb": rgb[sel].mean(0)}
        p["span"] = max(p["x1"] - p["x0"], p["y1"] - p["y0"])
        p["ry"] = (p["y1"] - p["y0"]) / 2
        props.append(p)
        api.log("PROP px=%d x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f span=%.3f rgb=%s"
                % (p["px"], p["x0"], p["x1"], p["y0"], p["y1"], p["ztop"],
                   p["span"], np.round(p["rgb"], 0).tolist()))
    return props


def goto(api, xyz, seconds=2.0, iters=2, tol=0.004):
    """closed-loop move: cancel the controller's standing tracking bias."""
    tgt = np.asarray(xyz, float)
    cmd = tgt.copy()
    for k in range(iters):
        api.move(cmd, rotation=R_DOWN, seconds=seconds)
        e = np.asarray(api.eef(), float)
        err = tgt - e
        api.log("GOTO k=%d tgt=%s eef=%s err=%.4f" % (
            k, np.round(tgt, 4).tolist(), np.round(e, 4).tolist(),
            float(np.linalg.norm(err))))
        if np.linalg.norm(err) < tol:
            break
        cmd = cmd + err
    return np.asarray(api.eef(), float)


def run(api):
    props = perceive(api)
    vessels = sorted([p for p in props if p["ztop"] - TABLE_Z > VESSEL_H],
                     key=lambda p: p["span"])
    flats = sorted([p for p in props if p["ztop"] - TABLE_Z <= VESSEL_H],
                   key=lambda p: -p["span"])
    if len(vessels) < 2 or not flats:
        api.log("ABORT vessels=%d flats=%d" % (len(vessels), len(flats)))
        return
    ram, bowls = vessels[0], vessels[1:]
    tgt = min(bowls, key=lambda p: np.hypot(p["cx"] - ram["cx"], p["cy"] - ram["cy"]))
    plate = flats[0]
    api.log("RAMEKIN cx=%.3f cy=%.3f span=%.3f | TARGET cx=%.3f cy=%.3f ry=%.3f "
            "ztop=%.3f | PLATE cx=%.3f cy=%.3f span=%.3f ztop=%.3f | nbowls=%d"
            % (ram["cx"], ram["cy"], ram["span"], tgt["cx"], tgt["cy"], tgt["ry"],
               tgt["ztop"], plate["cx"], plate["cy"], plate["span"],
               plate["ztop"], len(bowls)))

    gx, gy = tgt["cx"], tgt["y1"]                 # +y rim arc
    z_grasp = tgt["ztop"] - GRASP_DEPTH + TIP_DZ

    api.grip(0.08)
    goto(api, [gx, gy, CARRY_Z], seconds=2.5)
    goto(api, [gx, gy, z_grasp], seconds=2.0)
    api.log("PREGRIP eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(0.0)
    api.settle(0.5)
    g = api.gripper()
    api.log("CLOSED grip=%s eef=%s" % (g, np.round(api.eef(), 4).tolist()))

    goto(api, [gx, gy, CARRY_Z], seconds=2.5)
    api.log("LIFTED grip=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))

    # bowl centre trails the eef by the rim offset used at the grasp
    off_y = gy - tgt["cy"]
    px, py = plate["cx"], plate["cy"] + off_y
    goto(api, [px, py, CARRY_Z], seconds=3.0)
    api.log("OVERPLATE grip=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))

    hang = tgt["ztop"] - TABLE_Z - GRASP_DEPTH      # tips to bowl base
    z_rel = plate["ztop"] + 0.006 + hang + TIP_DZ
    goto(api, [px, py, z_rel], seconds=2.0)
    api.log("ATPLACE grip=%s eef=%s z_rel=%.4f" % (
        api.gripper(), np.round(api.eef(), 4).tolist(), z_rel))
    api.grip(0.08)
    api.settle(0.5)
    goto(api, [px, py, CARRY_Z], seconds=3.0, iters=2)
    api.settle(1.0)
    perceive(api)
    api.log("DONE")
