"""c2clean obj_tomato_sauce_pos_k0 -- v5: v4 plus a re-centre above the rim and
a sensed grasp retry.

Identity: the tomato sauce is the only prop whose top lies in the 0.055-0.100 m
band above the table (debug seeds 51-65: can 0.080; bbq bottle 0.112; milk /
orange juice / basket 0.141; flat boxes 0.018 / 0.028).

v4 grasped 8/8 on the probe subset but landed 5.8 mm off the can centre in y,
against 7.6 mm of body clearance and 4.4 mm of rim clearance. v5 cancels the
bias a second time just above the rim, where the fingertips are still clear,
and verifies the grasp by finger gap before carrying.
"""
import json

import numpy as np

PROVENANCE = {
    "XLO/XHI/YLO/YHI": {
        "source": "debug seeds 51-65 cam_high deprojection: props span "
                  "x[-0.21,+0.19] y[-0.28,+0.35]; the back wall deprojects to "
                  "x=-1.99 and is cropped out",
        "allowed": True},
    "CELL": {"source": "chosen top-down grid resolution (generic)", "allowed": True},
    "ZARM": {"source": "debug seeds 51-65: the parked arm cluster tops at "
                       "0.484 m; every prop tops at or below 0.142 m",
             "allowed": True},
    "CAN_BAND": {"source": "debug seeds 51-65 cluster tops: the can at 0.080 is "
                           "alone between the flat boxes (0.028) and the bbq "
                           "bottle (0.112)", "allowed": True},
    "BODY_LO/BODY_HI": {"source": "debug seeds 51/55/61 height slices of the can "
                                  "cluster: body width 0.062-0.063 m over "
                                  "z=0.010-0.058, flaring to 0.069 m at the rim "
                                  "(z>=0.074)", "allowed": True},
    "FINGER_OFF": {"source": "v3 debug seeds 51/53/55: the open gripper driven "
                             "into the bare table stalls at eef z=0.00915 with "
                             "the table at z=0.0015 -> tips sit 0.0077 m below "
                             "the eef", "allowed": True},
    "GRASP_TIP_Z": {"source": "v2 debug seeds: the can body spans z=0.001-0.074; "
                              "0.026 m is inside the parallel-sided section",
                    "allowed": True},
    "RIM_CLEAR_Z": {"source": "v2 debug seeds: the can rim tops at 0.080, so "
                              "fingertips at 0.102 (eef 0.11) are clear of it",
                    "allowed": True},
    "SAFE_Z": {"source": "debug seeds 51-65: the tallest props (basket, "
                         "cartons) top at 0.142 m", "allowed": True},
    "HELD_GAP_MIN": {"source": "v4 debug seeds 51-65: a grasp on the can body "
                               "closes to 0.0622 m (body measured 0.0625 m); a "
                               "close on air reads ~0", "allowed": True},
    "DROP_CLEAR": {"source": "v2 debug seeds: the basket rim tops at 0.142 m and "
                             "its interior is 0.14 x 0.155 m, far wider than the "
                             "0.063 m can", "allowed": True},
}

XLO, XHI, YLO, YHI = -0.45, 0.45, -0.60, 0.60
CELL = 0.006
ZARM = 0.20
CAN_BAND = (0.055, 0.100)
BODY_LO, BODY_HI = 0.015, 0.060
FINGER_OFF = 0.0077
GRASP_TIP_Z = 0.026
RIM_CLEAR_Z = 0.110
SAFE_Z = 0.20
HELD_GAP_MIN = 0.035
DROP_CLEAR = 0.026


# ---------------------------------------------------------------- perception
def _cc(occ):
    nx, ny = occ.shape
    lab = np.zeros(occ.shape, int)
    cur = 0
    out = []
    for i in range(nx):
        for j in range(ny):
            if not occ[i, j] or lab[i, j]:
                continue
            cur += 1
            st = [(i, j)]
            lab[i, j] = cur
            n = 0
            while st:
                p, q = st.pop()
                n += 1
                for dp in (-1, 0, 1):
                    for dq in (-1, 0, 1):
                        r, s = p + dp, q + dq
                        if 0 <= r < nx and 0 <= s < ny and occ[r, s] and not lab[r, s]:
                            lab[r, s] = cur
                            st.append((r, s))
            out.append((cur, n))
    return lab, out


def perceive(api):
    f = api.capture("cam_high")
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    z = np.asarray(f.depth, float)
    H, W = z.shape
    v, u = np.mgrid[0:H, 0:W]
    P = np.stack([(u - K[0, 2]) * z / K[0, 0],
                  (v - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], -1) @ T.T
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = np.isfinite(Z) & (X > XLO) & (X < XHI) & (Y > YLO) & (Y < YHI)
    h, e = np.histogram(Z[m], bins=400, range=(0.0, 0.4))
    tz = float(e[h.argmax()] + (e[1] - e[0]) / 2)
    obj = m & (Z > tz + 0.010) & (Z < tz + ZARM)
    nx = int((XHI - XLO) / CELL)
    ny = int((YHI - YLO) / CELL)
    ix = np.clip(((X - XLO) / CELL).astype(int), 0, nx - 1)
    iy = np.clip(((Y - YLO) / CELL).astype(int), 0, ny - 1)
    cnt = np.zeros((nx, ny), int)
    np.add.at(cnt, (ix[obj], iy[obj]), 1)
    lab, comps = _cc(cnt >= 2)
    rows = []
    for cid, ncell in comps:
        if ncell < 8:
            continue
        pm = obj & (lab[ix, iy] == cid)
        if int(pm.sum()) < 40:
            continue
        pts = P[pm][:, :3]
        rows.append(dict(n=int(pm.sum()), top=float(pts[:, 2].max() - tz), pts=pts))
    rows.sort(key=lambda r: -r["n"])
    return tz, rows


def find_can(rows):
    c = [r for r in rows if CAN_BAND[0] <= r["top"] <= CAN_BAND[1]]
    return max(c, key=lambda r: r["n"]) if c else None


def can_pose(can, tz):
    """Cylinder centre from the parallel-sided body band.

    cam_high looks down the -x axis, so y is the cross-view axis and its extent
    is the true diameter; x shows only the near tangent, so the centre is that
    tangent pulled back by one radius.
    """
    p = can["pts"]
    b = p[(p[:, 2] - tz >= BODY_LO) & (p[:, 2] - tz <= BODY_HI)]
    if b.shape[0] < 30:
        b = p
    d = float(b[:, 1].max() - b[:, 1].min())
    yc = float((b[:, 1].max() + b[:, 1].min()) / 2)
    xc = float(b[:, 0].max()) - d / 2.0
    return xc, yc, d


def basket_pose(rows, tz):
    b = max(rows, key=lambda r: r["n"])
    p = b["pts"]
    top = float(p[:, 2].max() - tz)
    rim = p[p[:, 2] - tz > top - 0.010]
    return (float((rim[:, 0].min() + rim[:, 0].max()) / 2),
            float((rim[:, 1].min() + rim[:, 1].max()) / 2), top)


# ------------------------------------------------------------------- motion
def approach_xy(api, tag, tx, ty, z, iters=3, tol=0.003, seconds=0.8):
    """Drive to (tx,ty) at height z, cancelling the controller's standing bias.

    Bounded: a pose the arm simply cannot hold would otherwise run the command
    away from the target.
    """
    cx, cy = tx, ty
    for k in range(iters):
        api.move([cx, cy, z], seconds=seconds)
        e = api.eef()
        ex, ey = tx - float(e[0]), ty - float(e[1])
        api.log("A|" + json.dumps({"tag": tag, "it": k,
                                   "eef": [round(float(t), 4) for t in e],
                                   "err": [round(ex, 4), round(ey, 4)]}))
        if (ex * ex + ey * ey) ** 0.5 < tol:
            break
        cx += max(-0.03, min(0.03, ex))
        cy += max(-0.03, min(0.03, ey))
        seconds = 0.5
    return cx, cy


def try_grasp(api, tz, cx, cy, attempt):
    api.grip(0.08)
    api.settle(0.3)
    api.move([cx, cy, tz + SAFE_Z], seconds=1.2)
    gx, gy = approach_xy(api, "hi%d" % attempt, cx, cy, tz + SAFE_Z)
    # re-centre just above the rim, where the fingertips are still clear of it
    gx, gy = approach_xy(api, "rim%d" % attempt, cx, cy, tz + RIM_CLEAR_Z,
                         iters=2, tol=0.003, seconds=0.6)
    r = api.move([gx, gy, tz + GRASP_TIP_Z + FINGER_OFF], seconds=0.8)
    e = api.eef()
    api.log("G|" + json.dumps({"att": attempt, "resid": round(float(r), 4),
                               "eef": [round(float(t), 4) for t in e],
                               "aim_err": [round(cx - float(e[0]), 4),
                                           round(cy - float(e[1]), 4)]}))
    api.grip(0.0)
    api.settle(0.3)
    g = api.gripper()
    api.log("H|" + json.dumps({"att": attempt, "closed": g}))
    api.move([gx, gy, tz + 0.28], seconds=1.2)
    g2 = api.gripper()
    api.log("L|" + json.dumps({"att": attempt, "lifted": g2,
                               "eef": [round(float(t), 4) for t in api.eef()]}))
    return g2["width_m"] > HELD_GAP_MIN and g2["effort"] > 1.0


def run(api):
    tz, rows = perceive(api)
    api.log("P|" + json.dumps({"tz": tz,
                               "tops": [round(r["top"], 4) for r in rows],
                               "ns": [r["n"] for r in rows]}))
    can = find_can(rows)
    if can is None:
        api.log("E|no can in band")
        return
    cx, cy, diam = can_pose(can, tz)
    bx, by, btop = basket_pose(rows, tz)
    api.log("C|" + json.dumps({"can": [round(cx, 4), round(cy, 4)],
                               "diam": round(diam, 4),
                               "can_top": round(can["top"], 4),
                               "basket": [round(bx, 4), round(by, 4)],
                               "btop": round(btop, 4)}))

    held = try_grasp(api, tz, cx, cy, 0)
    if not held:
        # re-perceive: a failed close usually means the can moved or toppled
        api.grip(0.08)
        api.settle(0.3)
        api.move([cx, cy, tz + 0.28], seconds=1.0)
        tz2, rows2 = perceive(api)
        can2 = find_can(rows2)
        if can2 is not None:
            cx, cy, diam = can_pose(can2, tz2)
            bx, by, btop = basket_pose(rows2, tz2)
            api.log("C2|" + json.dumps({"can": [round(cx, 4), round(cy, 4)],
                                        "diam": round(diam, 4),
                                        "top": round(can2["top"], 4)}))
        held = try_grasp(api, tz, cx, cy, 1)

    api.log("V|" + json.dumps({"held": bool(held)}))

    # carry over the basket and release with the can bottom above the rim
    api.move([bx, by, tz + 0.28], seconds=1.5)
    api.move([bx, by, tz + btop + GRASP_TIP_Z + FINGER_OFF + DROP_CLEAR],
             seconds=0.8)
    api.log("B|" + json.dumps({"pre_release": api.gripper(),
                               "eef": [round(float(t), 4) for t in api.eef()]}))
    api.grip(0.08)
    api.settle(0.5)
    api.move([bx, by, tz + 0.30], seconds=0.8)

    # post-hoc receipt: is a can-height cluster now sitting inside the basket?
    tz3, rows3 = perceive(api)
    c3 = find_can(rows3)
    inb = None
    if c3 is not None:
        p3 = c3["pts"]
        mx = float((p3[:, 0].min() + p3[:, 0].max()) / 2)
        my = float((p3[:, 1].min() + p3[:, 1].max()) / 2)
        inb = [round(mx, 4), round(my, 4), round(c3["top"], 4)]
    api.log("Z|" + json.dumps({"end_grip": api.gripper(), "can_after": inb,
                               "basket": [round(bx, 4), round(by, 4)]}))
