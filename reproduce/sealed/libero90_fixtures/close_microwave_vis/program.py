"""close the microwave -- v2: perceive the open door from cam_high depth, then
sweep it shut with the closed gripper along the door's own hinge arc.

Geometry (all re-derived on debug seeds 51/52/53 from cam_high RGB-D):
  * the table top is the modal height of the workspace cloud (0.902 m there);
  * the microwave body reads as a filled slab in the height band
    [table+0.14, table+0.22] at y >= ~0.24; the open door reads as a thin
    line of cells in the same band sticking out toward -y;
  * the door panel spans z ~ table+0.03 .. table+0.20, is ~0.233 m long, and
    is hinged at the body's front-left corner (min-x, min-y of the body slab);
  * closed, the door lies along +x from that corner (the body front face
    plane, y = body_ymin), so closing = rotating the door from its measured
    angle (-92 deg .. -102 deg on seeds 51-53) up to 0.
"""
import numpy as np

PROVENANCE = {
    "TABLE_BIN": {"source": "debug seeds 51-53 cam_high depth: modal workspace "
                            "height 0.902 m (histogram peak)", "allowed": True},
    "BAND_LO": {"source": "debug 51-53: mug tops reach ~table+0.10, microwave "
                          "body/door tops ~table+0.20; 0.14 separates them",
                "allowed": True},
    "BAND_HI": {"source": "debug 51-53: door top 1.108 = table+0.206; 0.22 caps "
                          "the band just above it and below the resting arm",
                "allowed": True},
    "CELL": {"source": "2 cm occupancy grid, chosen vs the 0.233 m door length "
                       "measured on debug 51-53", "allowed": True},
    "BODY_MIN_CELLS": {"source": "debug 51-53: microwave-body y-rows occupy "
                                 ">=16 x-cells, door rows 1-3", "allowed": True},
    "DOOR_L": {"source": "debug 51-53 measurement: 0.232/0.235/0.234 m from the "
                         "hinge corner to the door's far edge", "allowed": True},
    "CONTACT_R": {"source": "0.17 m = ~0.73 of the measured 0.233 m door length "
                            "(inboard of the free edge so the blade cannot slip "
                            "off)", "allowed": True},
    "PUSH_Z": {"source": "debug 51-53: door panel spans table+0.03..table+0.20; "
                         "push at table+0.05 keeps the hand top (measured "
                         "~0.13 above the eef on the resting arm) below the "
                         "1.108 m microwave top", "allowed": True},
    "TIP_OFFSET": {"source": "probe v1 on seed 51: closed fingers bottom out on "
                             "the table with eef z = 0.908 vs table 0.902",
                   "allowed": True},
    "STEP_DEG": {"source": "arc discretisation; 15 deg at r=0.17 m is a 45 mm "
                           "tool hop, ~4x the 12 mm move tolerance, and keeps "
                           "the whole sweep inside the 1000-step episode "
                           "(probe v2 measurement)", "allowed": True},
    "STAGE_BACK": {"source": "v1 receipt: descending on the door line jammed on "
                             "the door's top edge (eef stalled at 1.075 m, "
                             "0.12 m residual) on 7/8 debug seeds; 0.055 m "
                             "behind the panel is clear table", "allowed": True},
}

TABLE_BIN = (0.70, 1.20)
BAND_LO, BAND_HI = 0.14, 0.22
CELL = 0.02
BODY_MIN_CELLS = 6
CONTACT_R = 0.17
PUSH_Z = 0.05
TIP_OFFSET = 0.006
STEP_DEG = 15.0
STAGE_BACK = 0.055


def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    dep = np.asarray(f.depth, float)
    K, T = np.asarray(f.intrinsics, float), np.asarray(f.t_base_cam, float)
    h, w = dep.shape
    v, u = np.mgrid[0:h, 0:w]
    ok = np.isfinite(dep) & (dep > 0) & (dep < 2.5)
    z = np.where(ok, dep, 0.0)
    X = (u - K[0, 2]) * z / K[0, 0]
    Y = (v - K[1, 2]) * z / K[1, 1]
    P = np.stack([X, Y, z, np.ones_like(z)], -1) @ T.T
    return P[..., :3], ok


def scene(api):
    """Table height, hinge corner, door angle and length, from one cam_high frame."""
    P, ok = cloud(api)
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    ws = ok & (x > -0.55) & (x < 0.40) & (y > -0.55) & (y < 0.65)
    hist, edges = np.histogram(z[ws], bins=200, range=TABLE_BIN)
    tbl = float(edges[int(hist.argmax())]) + 0.5 * (edges[1] - edges[0])
    hi = ws & (z > tbl + BAND_LO) & (z < tbl + BAND_HI) & (y > -0.35)
    pts = P[hi]
    if len(pts) < 200:
        return None
    gx = np.round(pts[:, 0] / CELL).astype(int)
    gy = np.round(pts[:, 1] / CELL).astype(int)
    cnt = {}
    for a, b in zip(gx.tolist(), gy.tolist()):
        cnt[(a, b)] = cnt.get((a, b), 0) + 1
    cells = [k for k, c in cnt.items() if c >= 6]
    if not cells:
        return None
    per_row = {}
    for a, b in cells:
        per_row[b] = per_row.get(b, 0) + 1
    body_rows = [b for b, c in per_row.items() if c >= BODY_MIN_CELLS]
    if not body_rows:
        return None
    body_ymin_cell = min(body_rows)
    body = [c for c in cells if c[1] >= body_ymin_cell]
    # precise front-left corner: min x / min y of the body points themselves
    bmask = (pts[:, 1] >= (body_ymin_cell - 0.5) * CELL)
    bp = pts[bmask]
    hy = float(np.percentile(bp[:, 1], 2))
    hx = float(np.percentile(bp[:, 0], 2))
    hinge = np.array([hx, hy])
    dp = pts[pts[:, 1] < hy - 0.015][:, :2]
    if len(dp) < 100:
        return None
    rel = dp - hinge
    r = np.hypot(rel[:, 0], rel[:, 1])
    keep = r > 0.05
    rel, r = rel[keep], r[keep]
    if len(rel) < 50:
        return None
    ang = np.arctan2(rel[:, 1], rel[:, 0])
    theta = float(np.median(ang))
    L = float(np.percentile(r, 95))
    return {"tbl": tbl, "hinge": hinge, "theta": theta, "L": L,
            "body_x": (float(min(a for a, b in body) * CELL),
                       float(max(a for a, b in body) * CELL)),
            "n_door": int(len(rel))}


def contact_xy(hinge, theta, radius):
    return hinge + radius * np.array([np.cos(theta), np.sin(theta)])


def run(api):
    api.log("instruction=%r" % api.instruction())
    s = scene(api)
    if s is None:
        api.log("PERCEPTION FAILED")
        return "no scene"
    api.log("scene tbl=%.3f hinge=%s theta=%.1fdeg L=%.3f n=%d body_x=%s"
            % (s["tbl"], np.round(s["hinge"], 3).tolist(), np.degrees(s["theta"]),
               s["L"], s["n_door"], np.round(s["body_x"], 3).tolist()))

    tbl, hinge, th0 = s["tbl"], s["hinge"], s["theta"]
    zp = tbl + PUSH_Z + TIP_OFFSET
    api.grip(0.0)

    # stage BEHIND the door plane (v1 jammed on the door's top edge), then
    # descend in free table space to the push height
    u0 = np.array([np.cos(th0), np.sin(th0)])
    v0 = np.array([-u0[1], u0[0]])          # the direction the door swings shut
    start = contact_xy(hinge, th0, CONTACT_R) - STAGE_BACK * v0
    api.move([start[0], start[1], tbl + 0.24], seconds=1.0)
    r = api.move([start[0], start[1], zp], seconds=1.0)
    api.log("staged at %s res=%.4f eef=%s"
            % (np.round(start, 3).tolist(), r, np.round(api.eef(), 3).tolist()))

    th = th0
    k = 0
    while th < -np.radians(4.0) and k < 10:
        th = min(th + np.radians(STEP_DEG), 0.0)
        tgt = contact_xy(hinge, th, CONTACT_R)
        r = api.move([tgt[0], tgt[1], zp], seconds=0.7)
        e = api.eef()
        api.log("push %d theta=%.1f tgt=%s res=%.4f eef=%s"
                % (k, np.degrees(th), np.round(tgt, 3).tolist(), r, np.round(e, 3).tolist()))
        k += 1

    # retract clear of the camera line, then re-perceive to record the result
    api.move([hinge[0] - 0.05, hinge[1] - 0.30, tbl + 0.26], seconds=0.5)
    api.move([-0.40, -0.35, tbl + 0.28], seconds=0.5)
    s2 = scene(api)
    if s2 is None:
        api.log("post: no door found below the body front (consistent with closed)")
    else:
        api.log("post: theta=%.1fdeg L=%.3f n=%d"
                % (np.degrees(s2["theta"]), s2["L"], s2["n_door"]))
    return "v2 arc push"
