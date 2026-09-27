"""v8 -- v5 (15/15 on the debug split) plus two hardenings for the blind eval
split, whose object positions I have never seen:

  * the reach envelope measured in v6probe saturates at x ~ 0.216 with the
    wrist straight down, so targets are clamped to x <= 0.21 rather than being
    allowed to saturate;
  * v7probe showed a saturated arm is freed by a straight-up retreat, so a
    frozen `goto` now retreats to z=0.25 and re-approaches once.

Identification rule (my own debug-seed wrist-camera survey, v3): the flat boxes
are CHOCOLATE PUDDING (top 0.029, R-B 0.082) and FARM FRESH BUTTER (top 0.019,
R-B 0.234). Butter = the flat cluster with the largest R-B.

Identification rule (my own debug-seed wrist-camera survey, v3): the flat boxes
are CHOCOLATE PUDDING (top 0.029, R-B 0.082) and FARM FRESH BUTTER (top 0.019,
R-B 0.234). Butter = the flat cluster with the largest R-B.
"""

import numpy as np

PROVENANCE = {
    "XLO/XHI/YLO/YHI": {"source": "debug-seed cam_high cloud: workspace crop that excludes the walls (uncropped x reached -1.99)", "allowed": True},
    "Z_PROP_MIN/Z_PROP_MAX": {"source": "debug-seed cam_high height histogram: table plane in z=[0,0.010), prop band 0.012-0.15, arm band from 0.25", "allowed": True},
    "CELL/MIN_PTS": {"source": "generic occupancy-grid mechanics", "allowed": True},
    "FLAT_TOP_MAX": {"source": "debug-seed measurement: the two flat boxes top at 0.019/0.029, the next tallest prop at 0.081", "allowed": True},
    "BASKET_MIN_W": {"source": "debug-seed measurement: basket footprint 0.156-0.159 x 0.170-0.172; every other prop is <0.081 wide", "allowed": True},
    "FINGERTIP_OFFSET": {"source": "debug-seed v4 measurement: closed fingers descending on bare table stalled at eef z=0.0092-0.0094 with the table plane at z=0.005 (4 seeds)", "allowed": True},
    "HOVER_Z/LIFT_Z/OVER_Z": {"source": "debug-seed observation: props top out at 0.148 and the eef starts at z=0.261", "allowed": True},
    "DROP_CLEAR": {"source": "debug-seed measurement: basket rim top 0.141-0.142; clearance to carry the held box over it", "allowed": True},
    "GRASP_FRAC": {"source": "generic grasp mechanics: aim the fingertips at mid-height of the measured box", "allowed": True},
    "OPEN_W/CLOSE_W": {"source": "FairApi doc (<0.025 closes) + debug-seed gripper() width_m=0.0778 when open", "allowed": True},
    "HOLD_W_LO/HOLD_W_HI": {"source": "debug-seed v4 measurement: a good butter grasp closed to width 0.0389 (box short side 0.038-0.039); air-close reads 0.001 and a failed close 0.080", "allowed": True},
    "MAX_CORR": {"source": "generic servo mechanics: bound the feed-forward correction so a jammed arm cannot diverge the command", "allowed": True},
    "X_MAX": {"source": "debug-seed v6probe reach sweep: straight-down eef saturates at x=0.212-0.216 for every y", "allowed": True},
    "FREE_Z": {"source": "debug-seed v7probe: a straight-up move to z=0.25 frees a saturated arm", "allowed": True},
}

XLO, XHI, YLO, YHI = -0.40, 0.36, -0.46, 0.46
Z_PROP_MIN, Z_PROP_MAX = 0.012, 0.22
CELL = 0.006
MIN_PTS = 60
FLAT_TOP_MAX = 0.060
BASKET_MIN_W = 0.12
FINGERTIP_OFFSET = 0.0043
HOVER_Z, LIFT_Z, OVER_Z = 0.16, 0.22, 0.28
DROP_CLEAR = 0.02
GRASP_FRAC = 0.5
OPEN_W, CLOSE_W = 0.08, 0.0
HOLD_W_LO, HOLD_W_HI = 0.025, 0.055
MAX_CORR = 0.03
X_MAX = 0.21
FREE_Z = 0.25

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def cloud(fr):
    d = np.nan_to_num(np.asarray(fr.depth, float), nan=0.0)
    K = np.asarray(fr.intrinsics, float)
    T = np.asarray(fr.t_base_cam, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    pc = np.stack([x, y, d, np.ones_like(d)], -1).reshape(-1, 4)
    return (T @ pc.T).T[:, :3].reshape(h, w, 3), d


def components(occ):
    nx, ny = occ.shape
    lab = np.zeros((nx, ny), int)
    cur = 0
    for a in range(nx):
        for b in range(ny):
            if occ[a, b] and lab[a, b] == 0:
                cur += 1
                st = [(a, b)]
                lab[a, b] = cur
                while st:
                    q, r = st.pop()
                    for dq in (-1, 0, 1):
                        for dr in (-1, 0, 1):
                            s, t = q + dq, r + dr
                            if 0 <= s < nx and 0 <= t < ny and occ[s, t] and lab[s, t] == 0:
                                lab[s, t] = cur
                                st.append((s, t))
    return lab, cur


def perceive(api, tag=""):
    fr = api.capture("cam_high")
    pb, d = cloud(fr)
    ok = (d > 0.01) & np.isfinite(pb).all(-1)
    W = ok & (pb[..., 0] > XLO) & (pb[..., 0] < XHI) & (pb[..., 1] > YLO) & (pb[..., 1] < YHI) \
        & (pb[..., 2] > Z_PROP_MIN) & (pb[..., 2] < Z_PROP_MAX)
    nx = int((XHI - XLO) / CELL)
    ny = int((YHI - YLO) / CELL)
    ix = np.clip(((pb[..., 0] - XLO) / CELL).astype(int), 0, nx - 1)
    iy = np.clip(((pb[..., 1] - YLO) / CELL).astype(int), 0, ny - 1)
    occ = np.zeros((nx, ny), bool)
    vs, us = np.nonzero(W)
    occ[ix[vs, us], iy[vs, us]] = True
    lab, n = components(occ)
    props = []
    for c in range(1, n + 1):
        sel = W & (lab[ix, iy] == c)
        cnt = int(sel.sum())
        if cnt < MIN_PTS:
            continue
        P = pb[sel]
        col = fr.rgb[sel].astype(float).mean(0) / 255.0
        props.append(dict(n=cnt, cx=float(P[:, 0].mean()), cy=float(P[:, 1].mean()),
                          top=float(np.percentile(P[:, 2], 97)),
                          w=float(P[:, 0].max() - P[:, 0].min()),
                          h=float(P[:, 1].max() - P[:, 1].min()),
                          red=float(col[0] - col[2])))
    props.sort(key=lambda r: -r['n'])
    for i, r in enumerate(props):
        api.log("PROP%s%d n=%d ctr=(%.3f,%.3f) top=%.3f wh=(%.3f,%.3f) red=%.3f"
                % (tag, i, r['n'], r['cx'], r['cy'], r['top'], r['w'], r['h'], r['red']))
    return props


def pick_butter(props):
    flats = [p for p in props if p['top'] < FLAT_TOP_MAX and p['w'] < BASKET_MIN_W
             and p['h'] < BASKET_MIN_W]
    return max(flats, key=lambda p: p['red']) if flats else None


def pick_basket(props):
    big = [p for p in props if p['w'] > BASKET_MIN_W and p['h'] > BASKET_MIN_W]
    return max(big, key=lambda p: p['n']) if big else None


def _servo(api, tgt, seconds, tries, tol, tag):
    cmd = tgt.copy()
    prev = np.asarray(api.eef(), float)
    for k in range(tries):
        api.move(cmd.tolist(), rotation=R_DOWN, seconds=seconds)
        e = np.asarray(api.eef(), float)
        err = tgt - e
        moved = float(np.linalg.norm(e - prev))
        api.log("GOTO%s k=%d cmd=(%.3f,%.3f,%.3f) eef=(%.3f,%.3f,%.3f) err=%.4f moved=%.4f"
                % (tag, k, cmd[0], cmd[1], cmd[2], e[0], e[1], e[2],
                   float(np.linalg.norm(err)), moved))
        if np.linalg.norm(err) < tol:
            return e, True, False
        if k > 0 and moved < 0.001:
            api.log("GOTO%s FROZEN" % tag)
            return e, False, True
        prev = e
        cmd = tgt + np.clip(tgt - e, -MAX_CORR, MAX_CORR)
    e = np.asarray(api.eef(), float)
    return e, False, False


def goto(api, xyz, seconds=2.0, tries=3, tol=0.004, tag="", recover=True):
    """Closed-loop move with a bounded feed-forward correction.

    api.move under-shoots by ~10mm and the residual it returns is not the
    standing error, so the error is re-measured from api.eef(). The correction
    is clipped (MAX_CORR) and the loop bails out if the eef stopped moving, so
    a saturated arm cannot run the command away from the workspace. Targets are
    clamped into the reach envelope measured in v6probe; a frozen servo is
    freed by a straight-up retreat and then re-tried once (v7probe).
    """
    tgt = np.array(xyz, float)
    if tgt[0] > X_MAX:
        api.log("GOTO%s CLAMP x %.3f -> %.3f" % (tag, tgt[0], X_MAX))
        tgt[0] = X_MAX
    e, ok, frozen = _servo(api, tgt, seconds, tries, tol, tag)
    # Only a genuine freeze warrants the retreat. A servo that is merely short
    # of tolerance is already close enough, and retreating would throw away a
    # good descent (or a grasp).
    if ok or not recover or not frozen or float(np.linalg.norm(tgt - e)) < 0.015:
        return e, ok
    api.log("GOTO%s RECOVER" % tag)
    api.move([e[0], e[1], FREE_Z], rotation=R_DOWN, seconds=2.0)
    api.log("GOTO%s freed eef=%s" % (tag, np.round(api.eef(), 4).tolist()))
    e, ok, _ = _servo(api, tgt, seconds, tries, tol, tag + "/r")
    return e, ok


def try_grasp(api, butter, tag):
    bx, by = butter['cx'], butter['cy']
    api.grip(OPEN_W)
    api.settle(0.2)
    goto(api, [bx, by, HOVER_Z], seconds=2.5, tag=tag + "/hover")
    z_grasp = butter['top'] * GRASP_FRAC + FINGERTIP_OFFSET
    goto(api, [bx, by, z_grasp], seconds=2.5, tol=0.003, tag=tag + "/down")
    api.log("PREGRIP%s eef=%s grip=%s" % (tag, np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(CLOSE_W)
    api.settle(0.5)
    api.log("GRIPPED%s grip=%s" % (tag, api.gripper()))
    goto(api, [bx, by, LIFT_Z], seconds=2.5, tag=tag + "/lift")
    g = api.gripper()
    w = float(g['width_m'])
    api.log("LIFTED%s grip=%s eef=%s" % (tag, g, np.round(api.eef(), 4).tolist()))
    return HOLD_W_LO < w < HOLD_W_HI


def run(api):
    api.log("INSTRUCTION %s" % api.instruction())
    props = perceive(api)
    butter = pick_butter(props)
    basket = pick_basket(props)
    if butter is None or basket is None:
        api.log("ABORT butter=%s basket=%s" % (butter, basket))
        return
    api.log("BUTTER ctr=(%.3f,%.3f) top=%.3f wh=(%.3f,%.3f) red=%.3f"
            % (butter['cx'], butter['cy'], butter['top'], butter['w'], butter['h'], butter['red']))
    api.log("BASKET ctr=(%.3f,%.3f) top=%.3f" % (basket['cx'], basket['cy'], basket['top']))

    held = try_grasp(api, butter, "")
    if not held:
        api.log("RETRY -- first grasp did not hold")
        api.grip(OPEN_W)
        goto(api, [butter['cx'], butter['cy'], OVER_Z], seconds=2.5, tag="/reset")
        props = perceive(api, tag="R")
        b2 = pick_butter(props)
        if b2 is not None:
            butter = b2
            api.log("BUTTER2 ctr=(%.3f,%.3f) top=%.3f" % (b2['cx'], b2['cy'], b2['top']))
            held = try_grasp(api, butter, "2")
    api.log("HELD %s" % held)

    goto(api, [basket['cx'], basket['cy'], OVER_Z], seconds=3.0, tag="/over")
    api.log("OVERBASKET grip=%s" % api.gripper())
    z_drop = basket['top'] + FINGERTIP_OFFSET + DROP_CLEAR
    goto(api, [basket['cx'], basket['cy'], z_drop], seconds=2.5, tag="/drop")
    api.log("ATDROP eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.8)
    api.log("RELEASED grip=%s" % api.gripper())
    api.move([basket['cx'], basket['cy'], OVER_Z], rotation=R_DOWN, seconds=2.0)
    api.log("END eef=%s" % np.round(api.eef(), 4).tolist())
