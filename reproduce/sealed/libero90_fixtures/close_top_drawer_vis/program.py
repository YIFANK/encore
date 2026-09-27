"""v8 -- push with the wrist turned horizontal.

v7 measured the stall exactly: the drawer front went 0.067 -> 0.165 and then
froze, with gripper points sitting at y 0.18..0.22 / z 1.125..1.216 against the
cabinet top slab (z=1.127, y>=0.21).  With the wrist straight down the hand
body leads the fingertips into that slab, so the last ~0.045 m of travel is
geometrically unreachable.  Turning the tool to point along +y puts the
fingertips at the front and trails the whole hand behind them in -y.
"""
import numpy as np

PROVENANCE = {
    "PUSH_AXIS_PLUS_Y": {
        "source": "pack keyframes demo0/1/2 t0 vs tN (the protruding slab "
                  "retracts into the cabinet) + debug seed 51 cam_high cloud, "
                  "which places the cabinet body at larger +y than the drawer",
        "allowed": True},
    "Z_BAND": {"source": "debug seed 51 cam_high cloud: cabinet+drawer tops lie "
                         "in table+0.10..+0.23; the arm is above table+0.35",
               "allowed": True},
    "PANEL_Z_CUT": {"source": "debug seed 51 cam_high cloud: the drawer handle "
                              "tops out at z=1.098 while the front panel reaches "
                              "1.124, so table+0.20 isolates panel from handle",
                    "allowed": True},
    "TOOL_FWD": {"source": "generic controller mechanics: a proper rotation whose "
                           "third column (the tool approach axis) is world +y; "
                           "needed because v7 showed the hand body, not the "
                           "fingertip, is what contacts first with a down wrist",
                 "allowed": True},
    "PUSH_DX": {"source": "debug seed 51: handle spans x[-0.051,0.038], panel "
                          "spans x[-0.112,0.110]; +0.07 from centre is on the "
                          "panel and clear of the handle", "allowed": True},
    "PUSH_H": {"source": "v5 fingertip calibration (eef frame sits 0.008-0.015 m "
                         "above the contact point) + debug seed 51 panel face "
                         "span z=table+0.153..+0.223", "allowed": True},
    "CLOSE_TRAVEL": {"source": "v7 on debug seed 51: panel reached y=0.165 and "
                               "the cabinet front sits at y=0.20-0.22, so 0.22 "
                               "of commanded travel overshoots the joint stop",
                     "allowed": True},
    "TURN_BACK": {"source": "debug seed 51 cam_high cloud: the corridor 0.18 m "
                            "in -y of the drawer front is empty above the table",
                  "allowed": True},
}

Z_BAND = (0.10, 0.23)
PANEL_Z_CUT = 0.20
PUSH_DX = 0.070
PUSH_H = 0.19
CLOSE_TRAVEL = 0.22
TURN_BACK = 0.18
# tool approach axis (3rd column) = world +y
TOOL_FWD = np.array([[1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0],
                     [0.0, -1.0, 0.0]])


def cloud(f):
    d = np.asarray(f.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(f.intrinsics, float)
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    pc = np.stack([(uu - K[0, 2]) * d / K[0, 0],
                   (vv - K[1, 2]) * d / K[1, 1], d, np.ones_like(d)], axis=-1)
    P = pc @ np.asarray(f.t_base_cam, float).T[:, :3]
    return P[np.isfinite(d) & (d > 0)]


def survey(api, tag):
    v = cloud(api.capture("cam_high"))
    tbl = float(np.median(v[(np.abs(v[:, 0]) < 0.30) & (np.abs(v[:, 1]) < 0.35)
                            & (v[:, 2] < 0.95), 2]))
    b = v[(v[:, 2] > tbl + Z_BAND[0]) & (v[:, 2] < tbl + Z_BAND[1])
          & (v[:, 1] > -0.02) & (np.abs(v[:, 0]) < 0.25)]
    if len(b) < 200:
        api.log(f"[{tag}] table_z={tbl:.4f} EMPTY BAND n={len(b)}")
        return tbl, None, None
    hi = b[b[:, 2] > tbl + PANEL_Z_CUT]
    y_front = float(np.percentile(hi[:, 1], 0.5))
    near = hi[hi[:, 1] < y_front + 0.06]
    xc = float(0.5 * (np.percentile(near[:, 0], 1) + np.percentile(near[:, 0], 99)))
    api.log(f"[{tag}] table_z={tbl:.4f} y_front={y_front:.4f} xc={xc:.4f} "
            f"near_x=[{near[:,0].min():.3f},{near[:,0].max():.3f}] n={len(b)}")
    return tbl, y_front, xc


def run(api):
    tbl, y_front, xc = survey(api, "pre")
    if y_front is None:
        return
    xp = xc + PUSH_DX
    z_eef = tbl + PUSH_H
    y_turn = y_front - TURN_BACK
    y_goal = y_front + CLOSE_TRAVEL
    api.log(f"plan xp={xp:.4f} z={z_eef:.4f} y_turn={y_turn:.4f} y_goal={y_goal:.4f}")

    api.grip(0.0)
    r = api.move([xp, y_turn, tbl + 0.24], seconds=1.2)
    api.log(f"lift resid={r:.4f} eef={np.round(api.eef(),4).tolist()}")
    r = api.move([xp, y_turn, z_eef], rotation=TOOL_FWD, seconds=2.0)
    api.log(f"turn resid={r:.4f} eef={np.round(api.eef(),4).tolist()} "
            f"rot={np.round(api.tool_rotation(),2).tolist()}")

    for i in range(7):
        r = api.move([xp, y_goal, z_eef], rotation=TOOL_FWD, seconds=0.6)
        api.log(f"push{i} resid={r:.4f} eef={np.round(api.eef(),4).tolist()}")

    api.move([xp, api.eef()[1] - 0.12, z_eef], rotation=TOOL_FWD, seconds=0.8)
    api.log(f"retracted eef={np.round(api.eef(),4).tolist()}")
    survey(api, "post")
