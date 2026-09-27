"""c2k1clean / goal_put_bowl_top_cabinet_pos_k1 -- v1.

Put the bowl on top of the cabinet.

Mechanism (from the K=1 pack): descend beside the bowl wall, close (rim /
wall straddle), lift, carry, descend over the cabinet top, open.  Every
scene number is perceived from cam_high RGB-D at run time; the pack supplies
only the mechanism and the grasp/release HEIGHT relations.
"""
import json

import numpy as np

PROVENANCE = {
    "R_DOWN": {"source": "api.tool_rotation() at episode start on debug seeds 51-65 "
                         "(~diag(1,-1,-1)); the pack's keyframe euler rpy are all "
                         "near (pi,0,0) i.e. straight down",
               "allowed": True},
    "XLO_XHI_YLO_YHI": {"source": "debug-seed 51-65 cam_high deprojection: table points "
                                  "span this box; wider crops pull in wall/background",
                        "allowed": True},
    "RES": {"source": "generic: 5 mm top-down height-map cell, ~ depth resolution at 1 m",
            "allowed": True},
    "TABLE_Z": {"source": "debug seeds 51-65: modal deprojected z of the workspace = 0.902 m "
                          "(re-measured every episode, this is only the sanity band)",
                "allowed": True},
    "CAB_BAND": {"source": "debug seeds 51-65: the only large flat plateau above the table "
                           "sits at z=1.128 m (cabinet top); band chosen to bracket it",
                 "allowed": True},
    "BOWL_BAND": {"source": "debug seeds 51-65: the bowl's rim top deprojects to z=0.953 m, "
                            "51 mm above the table; band brackets it",
                  "allowed": True},
    "BOWL_DIA": {"source": "debug seeds 51-65: bowl footprint bbox is 0.110 x 0.110-0.116 m "
                           "in both axes; the black pot (the only other object in the band) "
                           "is 0.090 x 0.028 m, so a both-axes size window separates them",
                 "allowed": True},
    "GRASP_DEPTH": {"source": "pack demo0: gripper_cmd flips to close at ee z=0.9147 with the "
                              "table at 0.902, i.e. the jaws straddle the bowl wall deep, ~26 mm "
                              "below the rim top on this bowl",
                    "allowed": True},
    "PLACE_CLEAR": {"source": "pack demo0: release ee z=1.152 with the demo cabinet top; "
                              "expressed here as (bowl bottom offset below the eef) + 6 mm",
                    "allowed": True},
    "CARRY_Z": {"source": "debug seeds 51-65: tallest obstacle between bowl and cabinet is the "
                          "cabinet top itself at 1.128 m; 1.26 m clears it with the payload",
                "allowed": True},
    "JAW_OPEN": {"source": "api.gripper() at episode start on debug seeds: width_m=0.0778",
                 "allowed": True},
}

XLO, XHI, YLO, YHI = -0.70, 0.40, -0.60, 0.65
RES = 0.005
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
CARRY_Z = 1.26
JAW_OPEN = 0.08
GRASP_DEPTH = 0.026          # below the perceived rim top
PLACE_CLEAR = 0.006


# --------------------------------------------------------------- perception
def _cloud(f):
    d = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    x = (u - K[0, 2]) / K[0, 0] * d
    y = (v - K[1, 2]) / K[1, 1] * d
    pc = np.stack([x, y, d], -1)
    return pc @ T[:3, :3].T + T[:3, 3]


def _grid(pts, depth):
    nx = int((XHI - XLO) / RES)
    ny = int((YHI - YLO) / RES)
    m = ((pts[..., 0] > XLO) & (pts[..., 0] < XHI) & (pts[..., 1] > YLO)
         & (pts[..., 1] < YHI) & np.isfinite(depth) & (depth > 0.1))
    P = pts[m]
    ix = np.clip(((P[:, 0] - XLO) / RES).astype(int), 0, nx - 1)
    iy = np.clip(((P[:, 1] - YLO) / RES).astype(int), 0, ny - 1)
    Z = np.full((nx, ny), np.nan)
    for k in np.argsort(P[:, 2]):
        Z[ix[k], iy[k]] = P[k, 2]
    return Z, P


def _components(mask):
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    nx, ny = mask.shape
    for i in range(nx):
        for j in range(ny):
            if mask[i, j] and lab[i, j] == 0:
                cur += 1
                stack = [(i, j)]
                lab[i, j] = cur
                while stack:
                    a, b = stack.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if 0 <= p < nx and 0 <= q < ny and mask[p, q] and lab[p, q] == 0:
                                lab[p, q] = cur
                                stack.append((p, q))
    return lab, cur


def _blobs(Z, lo, hi, minn):
    m = np.isfinite(Z) & (Z > lo) & (Z < hi)
    lab, n = _components(m)
    out = []
    for c in range(1, n + 1):
        ix, iy = np.where(lab == c)
        if len(ix) < minn:
            continue
        x = XLO + (ix + 0.5) * RES
        y = YLO + (iy + 0.5) * RES
        z = Z[ix, iy]
        out.append(dict(n=int(len(ix)), x=x, y=y, z=z,
                        xr=(float(x.min()), float(x.max())),
                        yr=(float(y.min()), float(y.max())),
                        ztop=float(z.max()), zmed=float(np.median(z))))
    out.sort(key=lambda r: -r["n"])
    return out


def perceive(api):
    f = api.capture("cam_high")
    pts = _cloud(f)
    Z, P = _grid(pts, np.asarray(f.depth, float))
    zf = Z[np.isfinite(Z)]
    h, e = np.histogram(zf, bins=np.arange(0.80, 1.45, 0.004))
    table_z = float(e[int(np.argmax(h))] + 0.002)
    api.log("PERC table_z=%.4f" % table_z)

    # --- cabinet top: the large flat plateau well above the table
    cab = None
    for b in _blobs(Z, table_z + 0.12, table_z + 0.34, 300):
        flat = float(np.percentile(b["z"], 95) - np.percentile(b["z"], 5))
        api.log("PERC cabcand n=%d c=(%.3f,%.3f) x[%.3f,%.3f] y[%.3f,%.3f] zmed=%.3f flat=%.4f"
                % (b["n"], b["x"].mean(), b["y"].mean(), b["xr"][0], b["xr"][1],
                   b["yr"][0], b["yr"][1], b["zmed"], flat))
        if flat < 0.012 and cab is None:
            cab = b
    if cab is None:
        raise RuntimeError("no cabinet top found")

    # --- bowl: round blob just above the table
    bowl = None
    for b in _blobs(Z, table_z + 0.022, table_z + 0.11, 80):
        wx = b["xr"][1] - b["xr"][0]
        wy = b["yr"][1] - b["yr"][0]
        api.log("PERC bowlcand n=%d c=(%.3f,%.3f) w=(%.3f,%.3f) ztop=%.3f"
                % (b["n"], b["x"].mean(), b["y"].mean(), wx, wy, b["ztop"]))
        if 0.08 < wx < 0.15 and 0.08 < wy < 0.15 and min(wx, wy) / max(wx, wy) > 0.7:
            if bowl is None:
                bowl = b
    if bowl is None:
        raise RuntimeError("no bowl found")

    ring = bowl["z"] > bowl["ztop"] - 0.008
    cx = float((bowl["x"][ring].min() + bowl["x"][ring].max()) / 2.0)
    cy = float((bowl["y"][ring].min() + bowl["y"][ring].max()) / 2.0)
    rim_z = bowl["ztop"]

    # outer radius of the wall at the intended closing height
    zc = rim_z - GRASP_DEPTH
    sel = P[(np.hypot(P[:, 0] - cx, P[:, 1] - cy) < 0.10)
            & (P[:, 2] > zc - 0.005) & (P[:, 2] < zc + 0.005)]
    r_wall = float(np.percentile(np.hypot(sel[:, 0] - cx, sel[:, 1] - cy), 92)) if len(sel) > 30 else 0.048

    cab_x = float((cab["xr"][0] + cab["xr"][1]) / 2.0)
    cab_y = float((cab["yr"][0] + cab["yr"][1]) / 2.0)
    api.log("PERC bowl c=(%.4f,%.4f) rim_z=%.4f r_wall=%.4f | cab c=(%.4f,%.4f) z=%.4f "
            "x[%.3f,%.3f] y[%.3f,%.3f]"
            % (cx, cy, rim_z, r_wall, cab_x, cab_y, cab["zmed"],
               cab["xr"][0], cab["xr"][1], cab["yr"][0], cab["yr"][1]))
    return dict(table_z=table_z, cx=cx, cy=cy, rim_z=rim_z, r_wall=r_wall,
                cab_x=cab_x, cab_y=cab_y, cab_z=float(cab["zmed"]),
                cab_xr=cab["xr"], cab_yr=cab["yr"])


# ------------------------------------------------------------------- motion
def goto(api, tag, xyz, seconds=2.5, tries=2):
    xyz = np.asarray(xyz, float)
    for i in range(tries):
        res = api.move(xyz, rotation=R_DOWN, seconds=seconds)
        e = api.eef()
        err = float(np.linalg.norm(np.asarray(e) - xyz))
        api.log("MOVE %s try%d tgt=%s eef=%s res=%s err=%.4f"
                % (tag, i, np.round(xyz, 4).tolist(), np.round(e, 4).tolist(), res, err))
        if err < 0.008:
            break
    return api.eef()


def run(api):
    api.log("INSTR %s" % api.instruction())
    s = perceive(api)

    api.grip(JAW_OPEN)
    api.settle(0.3)

    zc = s["rim_z"] - GRASP_DEPTH
    gx, gy = s["cx"], s["cy"] + s["r_wall"]

    goto(api, "hover", [gx, gy, s["rim_z"] + 0.09], seconds=3.0)
    goto(api, "predesc", [gx, gy, s["rim_z"] + 0.02], seconds=2.0)
    goto(api, "descend", [gx, gy, zc], seconds=2.0)

    api.grip(0.0)
    api.settle(0.6)
    g = api.gripper()
    api.log("GRASP gripper=%s eef=%s" % (json.dumps(g), np.round(api.eef(), 4).tolist()))

    goto(api, "lift", [gx, gy, CARRY_Z], seconds=3.0)
    api.log("AFTERLIFT gripper=%s" % json.dumps(api.gripper()))

    tx = float(np.clip(s["cab_x"], s["cab_xr"][0] + 0.055, s["cab_xr"][1] - 0.055))
    ty = float(np.clip(s["cab_y"], s["cab_yr"][0] + 0.055, s["cab_yr"][1] - 0.055))
    goto(api, "carry", [tx, ty, CARRY_Z], seconds=4.0, tries=3)

    drop = s["cab_z"] + (zc - s["table_z"]) + PLACE_CLEAR
    goto(api, "place", [tx, ty, drop], seconds=3.0, tries=2)
    api.log("BEFORE_RELEASE gripper=%s eef=%s"
            % (json.dumps(api.gripper()), np.round(api.eef(), 4).tolist()))

    api.grip(JAW_OPEN)
    api.settle(0.8)
    goto(api, "retreat", [tx, ty, CARRY_Z], seconds=2.5)
    api.settle(0.5)
    api.log("END gripper=%s eef=%s" % (json.dumps(api.gripper()), np.round(api.eef(), 4).tolist()))
