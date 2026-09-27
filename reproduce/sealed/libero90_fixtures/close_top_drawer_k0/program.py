"""v5: as v4 but the push runs to the cabinet front plane, not 33 mm short.

Measured on debug seeds 51/52/53/55 (cam_high point clouds, v1/v2) and on the
v3 run:
  * cabinet front plane  y = 0.2163 (dead flat over z 0.96..1.04)
  * open top-drawer front panel face  y = 0.0671 ; its handle reaches y = 0.0353
  * handle bar sits at z 1.083..1.098 ; drawer floor z 1.064 ; cab top z 1.127
  => closing travel = 0.2163 - 0.0671 = 0.149 m in +y.
  * v3: the closed gripper stalled on the cabinet top with eef z = 1.1382 over a
    1.1272 surface -> fingertips are 0.011 below the eef point.
  * v3: the hand casing spans 0.20 m along the gripper's open-width axis and only
    0.03 m across it, so the wrist is yawed 90 deg about world z to keep the
    casing from fouling the cabinet top while the fingertips push home.
  * v3 also showed the 1000-step episode budget is the binding constraint, so this
    version keeps the move count low.
  * v4 (3/4 on debug 51/53/55/57): the one failure left the drawer front at
    y=0.1845 with the eef stopped at 0.1719, i.e. the gripper block pushes the
    front face 0.0126 ahead of the eef, so the eef must run to
    y_closed-0.013 rather than stopping 0.033 short.
"""

import numpy as np

PROVENANCE = {
    "TIP_OFFSET": {
        "source": "v3 debug run: closed gripper pressed onto the cabinet top "
                  "(z=1.1272 measured in the same episode) stalled at eef z=1.1382",
        "allowed": True},
    "PUSH_TIP_Z": {
        "source": "debug seeds 51/52/53/55 cam_high: handle bar points span "
                  "z=1.083..1.098; pushing tip placed at 1.090",
        "allowed": True},
    "YAW90": {
        "source": "v3 debug run cam_high: hand casing measured 0.20 m along the "
                  "gripper width axis vs 0.03 m across it; generic wrist mechanics",
        "allowed": True},
    "FRONT_BAND": {
        "source": "debug seeds: drawer front face / handle are the only structure "
                  "with z in 1.05..1.13 and y < 0.19",
        "allowed": True},
    "CLOSED_BAND": {
        "source": "debug seeds: the cabinet front plane is the front-most surface "
                  "in z 0.96..1.04 (y=0.2163 on all four seeds)",
        "allowed": True},
    "FLOOR_BAND": {
        "source": "debug seeds: open drawer interior floor z=1.057..1.090, used "
                  "only for the drawer centre x",
        "allowed": True},
    "ROI": {
        "source": "debug seeds: cabinet occupies x=[-0.14,0.14], y=[0.02,0.40]",
        "allowed": True},
    "Y_CAP": {
        "source": "v4 debug run: drawer front ended 0.0126 ahead of the eef, so "
                  "the eef must reach y_closed-0.013 for the front to sit flush",
        "allowed": True},
    "APPROACH_GAP": {
        "source": "debug-seed handle stand-off; 0.02 m clear of the handle face",
        "allowed": True},
}

TIP_OFFSET = 0.011
PUSH_TIP_Z = 1.090
APPROACH_GAP = 0.020
SAFE_Z = 1.30
ROI_X = (-0.20, 0.22)
YAW90 = np.array([[0.0, 1.0, 0.0],
                  [1.0, 0.0, 0.0],
                  [0.0, 0.0, -1.0]])


def cloud(fr):
    d = np.asarray(fr.depth, dtype=np.float64)
    H, W = d.shape
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    vv, uu = np.mgrid[0:H, 0:W]
    x = (uu - K[0, 2]) / K[0, 0] * d
    y = (vv - K[1, 2]) / K[1, 1] * d
    return (np.stack([x, y, d], -1) @ T[:3, :3].T + T[:3, 3]).reshape(-1, 3)


def perceive(api):
    P = cloud(api.capture("cam_high"))
    P = P[(P[:, 0] > ROI_X[0]) & (P[:, 0] < ROI_X[1]) &
          (P[:, 1] > 0.01) & (P[:, 1] < 0.42) &
          (P[:, 2] > 0.91) & (P[:, 2] < 1.14)]
    out = {"n": len(P)}

    body = P[(P[:, 2] > 0.96) & (P[:, 2] < 1.045)]
    if len(body) > 200:
        out["y_closed"] = float(np.percentile(body[:, 1], 2))

    front = P[(P[:, 2] > 1.050) & (P[:, 2] < 1.132) & (P[:, 1] < 0.19)]
    out["front_n"] = len(front)
    if len(front) > 100:
        out["y_face"] = float(np.percentile(front[:, 1], 2))
        out["face_xc"] = float(0.5 * (np.percentile(front[:, 0], 2) +
                                      np.percentile(front[:, 0], 98)))

    floor = P[(P[:, 2] > 1.050) & (P[:, 2] < 1.100) &
              (P[:, 1] > 0.08) & (P[:, 1] < 0.20)]
    if len(floor) > 200:
        out["floor_xc"] = float(0.5 * (np.percentile(floor[:, 0], 2) +
                                       np.percentile(floor[:, 0], 98)))
    return out


def show(d):
    return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in d.items()}


def goto(api, xyz, rot, seconds, tag):
    r = api.move(list(xyz), rotation=rot, seconds=seconds)
    e = np.asarray(api.eef(), dtype=float)
    api.log("MOVE %-8s -> %s eef=%s resid=%s" % (
        tag, np.round(xyz, 4).tolist(), np.round(e, 4).tolist(), np.round(r, 4)))
    return e


def run(api):
    api.log("EEF0=%s ROT0=%s" % (np.round(api.eef(), 4).tolist(),
                                 np.round(api.tool_rotation(), 3).tolist()))
    g0 = perceive(api)
    api.log("PERCEIVE0 %s" % show(g0))

    api.grip(0.0)
    api.settle(0.2)

    xc = g0.get("floor_xc", g0.get("face_xc", 0.0))
    y_face = g0.get("y_face", 0.0353)
    y_closed = g0.get("y_closed", 0.2163)
    travel = y_closed - y_face
    z_eef = PUSH_TIP_Z + TIP_OFFSET
    y0 = y_face - APPROACH_GAP
    y_cap = y_closed - 0.008
    api.log("GEOM xc=%.4f y_face=%.4f y_closed=%.4f travel=%.4f y0=%.4f cap=%.4f"
            % (xc, y_face, y_closed, travel, y0, y_cap))

    goto(api, [xc, y0, z_eef], YAW90, 3.0, "approach")

    y = y0
    for k in range(6):
        y = min(y + 0.05, y_cap)
        e = goto(api, [xc, y, z_eef], YAW90, 2.2, "push%d" % k)
        if y >= y_cap - 1e-6:
            break
    for k in range(2):
        e = goto(api, [xc, y_cap, z_eef], YAW90, 1.8, "seat%d" % k)
        api.log("seat%d eef_y=%.4f gap=%.4f" % (k, e[1], y_cap - e[1]))

    goto(api, [xc, y0, z_eef], YAW90, 1.6, "back")
    goto(api, [xc, y0, SAFE_Z], YAW90, 1.6, "lift")
    api.log("PERCEIVE1 %s" % show(perceive(api)))
