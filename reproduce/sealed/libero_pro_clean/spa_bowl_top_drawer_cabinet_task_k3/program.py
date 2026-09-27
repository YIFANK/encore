"""c2clean / spa_bowl_top_drawer_cabinet_task_k3 -- v2.

Intent: "Pick the akita black bowl on the top of the wooden cabinet and place
it on the plate."

Mechanism (all of it re-derived from the two packs + debug seeds 51/53):
  * the cabinet top is a flat slab; the target bowl is the only thing standing
    on it.  Perceive the slab plane, then the cells above it -> Kasa circle fit
    on the rim ring -> bowl centre + rim radius + rim-top height.
  * the mate pack ("black bowl on the wooden cabinet" -> plate) closes the
    gripper to a ~5 mm gap: that is a pinch across the thin rim WALL, not a
    straddle of the whole bowl.  Its close pose sits 0.028 m below the rim top
    (demo mean z 1.153 vs the rim top 1.180 measured on seeds 51/53).
  * the tool frame at reset is straight down with the jaws along base y, so
    the pinch point is the rim arc at (centre_x, centre_y + r).
  * carried rigidly, the bowl centre trails the eef by r in -y.  Cross-check:
    BOTH packs release over the plate at y = plate_y + r (k3 0.251/0.250/0.272,
    mate 0.218/0.238/0.214 vs the plate centre 0.205 measured here).
  * release height: eef = plate_top + 0.026, i.e. the bowl base (rim top minus
    the 0.053 m bowl height) just kisses the plate.  Demo releases: 0.945.

v4 (v1 6/8, v2 5/8, v3 6/8).  Receipts so far:
  * ONE api.move costs 40-75 sim steps whatever `seconds` is, and the episode
    horizon is 1000 -- past it the eef silently freezes and every later command
    is a no-op (v2/v3 failures all show a constant eef and sim_steps=1000).
    So the episode affords ~14 moves and they must be budgeted.
  * `effort` is only a gap flag (3.0 iff the gap exceeds ~5 mm), so the hold
    test is gripper WIDTH alone: an empty close reads 0.0010, a real rim
    pinch 0.0045..0.0107.
  * Long moves track fine (0.47 m across the table lands within 0.012), but
    the descent onto the cabinet-top bowl swings up to 3 cm in +x and stalls
    1-2 cm high; that swing, not perception, is what misses the rim.  A
    bias-cancelled re-command from the stalled pose removes most of it.
  * Staging the descent (high hover -> a tightly converged hover 0.05 above
    the grasp -> the last 0.05) removes the swing almost entirely: v4 landed
    the descent within 0.0007..0.0013 on 7 of 8 debug seeds and scored 7/8 in
    224 sim steps.
  * v4's remaining loss (ep63) came from its lateral correction: sliding
    sideways AT the grasp height jammed the arm -- the eef then stopped
    responding to every later command, including a straight lift.
  * v5's safe re-descend did not help either: on ep63 the descent is
    CONTACT/REACH LIMITED -- it stalls ~0.01 m above the commanded grasp z at
    a rim-relative depth of 0.022 -- and pressing into that stall is what
    costs sim steps: ep63 burned the whole 1000-step horizon in seven moves
    while the eight-move seeds finish in 224.  After the horizon the eef
    freezes and even the gripper stops responding (width stays 0.0800).
  * v6 closed at the stall and the pinch DID catch (ep63 width 0.0054, held),
    but the episode was already out of steps by then.  Measured cost law: a
    move runs until it is within the controller's own 0.012 m tolerance or for
    60*seconds sim steps, whichever comes first -- so a move that cannot
    converge costs 180 steps at seconds=3.0 and only ~10 when it converges.
    Six starving moves are the whole 1000-step horizon.
v7 keeps v6's motion plan and prices each hop: seconds is sized to the hop
length (1.0 for the 0.05-0.13 m moves around the bowl, 2.0 for the 0.5 m
transit to the plate, 1.5 for the place descent), so even an episode whose
every move starves stays inside the horizon.
"""
PROVENANCE = {
    "CELL": {"source": "generic: 5 mm top-down height-map cell", "allowed": True},
    "WORKSPACE": {"source": "debug seeds 51/53 cam_high point cloud extent", "allowed": True},
    "SLAB_BAND": {"source": "debug seeds 51/53: cabinet top plane measured at z=1.127, table at 0.900", "allowed": True},
    "BOWL_BAND": {"source": "debug seeds 51/53: bowl rim top 1.180 = slab + 0.053", "allowed": True},
    "GRASP_BELOW_RIM": {"source": "mate pack keyframe close-pose z (1.159/1.147/1.152, mean 1.153) minus the rim top 1.180 measured on debug seeds 51/53", "allowed": True},
    "BOWL_HEIGHT": {"source": "debug seeds 51/53: rim top 1.180 minus slab 1.127", "allowed": True},
    "PLACE_ABOVE_PLATE": {"source": "k3+mate pack release z 0.945/0.956/0.973 vs plate top 0.920 measured on debug seeds 51/53; = bowl height - grasp offset", "allowed": True},
    "PINCH_ARC": {"source": "both packs release at plate_y + rim radius (k3 0.251/0.250/0.272 vs plate centre y 0.205, r 0.054), so the demos pinch the +y rim arc and carry the bowl centre at eef_y - r", "allowed": True},
    "CARRY_Z": {"source": "debug seeds 51/53: cabinet slab 1.127 is the tallest obstacle on the path; carry 0.10 above it", "allowed": True},
    "HOLD_MIN": {"source": "both packs: gripper_state at the closed keyframes is 0.0026..0.0057 per finger (~5 mm gap) = the rim wall thickness; v1 debug seeds: an empty close reads 0.0010 with effort 0.05, a real hold 0.0057..0.0078 with effort 3.0", "allowed": True},
    "PLATE_BAND": {"source": "debug seeds 51/53: plate cluster top 0.920, table 0.900", "allowed": True},
    "MOVE_TOL": {"source": "v1 debug seeds: converged moves land within 0.010 of the command and succeed; the two failures stopped 0.019-0.030 short", "allowed": True},
    "MOVE_BUDGET": {"source": "v1/v2 receipts: 9 moves = 537 sim steps and the episode freezes at 1000, so ~60 steps per move and ~16 moves per episode", "allowed": True},
    "MOVE_SECONDS": {"source": "generic controller mechanics measured on debug seeds: a move stops at the controller's own tolerance or after 60*seconds steps, so `seconds` is sized to each hop length (>=0.05 m/60 steps of travel)", "allowed": True},
    "BIAS_MAX": {"source": "generic OSC mechanics + v1 debug seeds: the residual is a standing tracking offset, so re-command the target plus the observed error, bounded", "allowed": True},
}

import numpy as np

CELL = 0.005
LO, HI = -0.45, 0.45
NG = int((HI - LO) / CELL) + 1

GRASP_BELOW_RIM = 0.028
BOWL_HEIGHT = 0.053
PLACE_ABOVE_PLATE = 0.026
CARRY_Z = 1.25
HOLD_MIN = 0.0035
MOVE_TOL = 0.008
BIAS_MAX = 0.030
MOVE_SECONDS = 3.0
MOVE_BUDGET = 14
RESERVE = 4


# ---------------------------------------------------------------- perception
def height_map(f):
    d = np.asarray(f.depth, float)
    H, W = d.shape
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    v, u = np.mgrid[0:H, 0:W]
    x = (u - K[0, 2]) * d / K[0, 0]
    y = (v - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d], -1) @ T[:3, :3].T + T[:3, 3]
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (np.isfinite(d) & (d > 0) & (X > LO) & (X < HI) & (Y > LO) & (Y < HI)
         & (Z > 0.85) & (Z < 1.45))
    gx = np.clip(((X - LO) / CELL).astype(int), 0, NG - 1)
    gy = np.clip(((Y - LO) / CELL).astype(int), 0, NG - 1)
    hm = np.full((NG, NG), np.nan)
    idx = np.argsort(Z[m])
    hm[gx[m][idx], gy[m][idx]] = Z[m][idx]
    return hm


def components(mask):
    lab = np.zeros(mask.shape, int)
    cur = 0
    for i, j in zip(*np.nonzero(mask)):
        if lab[i, j]:
            continue
        cur += 1
        lab[i, j] = cur
        stack = [(i, j)]
        while stack:
            a, b = stack.pop()
            for p in (a - 1, a, a + 1):
                for q in (b - 1, b, b + 1):
                    if 0 <= p < mask.shape[0] and 0 <= q < mask.shape[1] \
                            and mask[p, q] and not lab[p, q]:
                        lab[p, q] = cur
                        stack.append((p, q))
    out = []
    for c in range(1, cur + 1):
        ii, jj = np.nonzero(lab == c)
        out.append((ii, jj))
    out.sort(key=lambda t: -len(t[0]))
    return out


def xy(ii, jj):
    return ii * CELL + LO, jj * CELL + LO


def kasa(xs, ys):
    A = np.stack([xs, ys, np.ones_like(xs)], 1)
    b = xs ** 2 + ys ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    r = float(np.sqrt(max(sol[2] + cx * cx + cy * cy, 1e-9)))
    return float(cx), float(cy), r


def find_slab(api, hm):
    mask = (~np.isnan(hm)) & (hm > 1.09) & (hm < 1.145)
    cs = components(mask)
    if not cs:
        return None
    ii, jj = cs[0]
    if len(ii) < 200:
        return None
    xs, ys = xy(ii, jj)
    slab = dict(z=float(np.median(hm[ii, jj])), x0=float(xs.min()), x1=float(xs.max()),
                y0=float(ys.min()), y1=float(ys.max()), n=len(ii))
    api.log("SLAB %s" % {k: (round(v, 3) if isinstance(v, float) else v)
                         for k, v in slab.items()})
    return slab


def find_bowl(api, hm, slab, tag=""):
    pad = 0.04
    ii0, jj0 = np.mgrid[0:NG, 0:NG]
    xs0, ys0 = xy(ii0, jj0)
    inside = ((xs0 > slab["x0"] - pad) & (xs0 < slab["x1"] + pad)
              & (ys0 > slab["y0"] - pad) & (ys0 < slab["y1"] + pad))
    mask = (~np.isnan(hm)) & inside & (hm > slab["z"] + 0.020) & (hm < slab["z"] + 0.090)
    cands = []
    for ii, jj in components(mask):
        if len(ii) < 40:
            continue
        xs, ys = xy(ii, jj)
        ztop = float(np.nanmax(hm[ii, jj]))
        ring = hm[ii, jj] >= ztop - 0.005
        if ring.sum() < 15:
            continue
        cx, cy, r = kasa(xs[ring], ys[ring])
        res = float(np.std(np.hypot(xs[ring] - cx, ys[ring] - cy) - r))
        cands.append(dict(n=len(ii), cx=cx, cy=cy, r=r, ztop=ztop, res=res,
                          nring=int(ring.sum())))
    for c in cands:
        api.log("BOWLCAND%s n=%d ring=%d cx=%.4f cy=%.4f r=%.4f ztop=%.4f res=%.4f"
                % (tag, c["n"], c["nring"], c["cx"], c["cy"], c["r"], c["ztop"], c["res"]))
    good = [c for c in cands if 0.035 < c["r"] < 0.075 and c["res"] < 0.010]
    if not good:
        return None
    return max(good, key=lambda c: c["n"])


def find_plate(api, hm):
    ii0, jj0 = np.mgrid[0:NG, 0:NG]
    xs0, ys0 = xy(ii0, jj0)
    mask = (~np.isnan(hm)) & (hm > 0.910) & (hm < 0.935) & (ys0 > 0.05) & (xs0 > -0.15)
    best = None
    for ii, jj in components(mask):
        if len(ii) < 60:
            continue
        xs, ys = xy(ii, jj)
        w, h = float(xs.max() - xs.min()), float(ys.max() - ys.min())
        c = dict(n=len(ii), cx=float((xs.min() + xs.max()) / 2),
                 cy=float((ys.min() + ys.max()) / 2), w=w, h=h,
                 ztop=float(np.nanmax(hm[ii, jj])))
        api.log("PLATECAND n=%d cx=%.4f cy=%.4f w=%.3f h=%.3f ztop=%.4f"
                % (c["n"], c["cx"], c["cy"], w, h, c["ztop"]))
        if 0.09 < w < 0.20 and 0.09 < h < 0.20 and (best is None or c["n"] > best["n"]):
            best = c
    return best


# ------------------------------------------------------------------- program
def run(api):
    api.log("instruction: %r" % api.instruction())
    R = np.asarray(api.tool_rotation(), float)
    api.log("R0=%s eef0=%s" % (np.round(R, 3).tolist(), np.round(api.eef(), 4).tolist()))
    spent = [0]

    def goto(p, tag="", tol=MOVE_TOL, tries=2, reserve=0, seconds=MOVE_SECONDS):
        p = np.asarray(p, float)
        cmd = p.copy()
        e = np.asarray(api.eef(), float)
        prev = e.copy()
        for k in range(tries):
            if spent[0] >= MOVE_BUDGET - reserve:
                api.log("BUDGET stop (spent=%d reserve=%d) at%s" % (spent[0], reserve, tag))
                break
            api.move([float(v) for v in cmd], rotation=R, seconds=seconds)
            spent[0] += 1
            e = np.asarray(api.eef(), float)
            n = float(np.linalg.norm(p - e))
            api.log("GOTO%s try%d n=%d cmd=%s eef=%s err=%.4f"
                    % (tag, k, spent[0], np.round(cmd, 4).tolist(),
                       np.round(e, 4).tolist(), n))
            if n <= tol:
                break
            if k and float(np.linalg.norm(e - prev)) < 0.0015:
                api.log("JAM%s: eef moved %.4f while %.4f from target"
                        % (tag, float(np.linalg.norm(e - prev)), n))
                break
            prev = e
            cmd = p + np.clip(cmd - e, -BIAS_MAX, BIAS_MAX)
        return e

    hm = height_map(api.capture("cam_high"))
    slab = find_slab(api, hm)
    if slab is None:
        api.log("NO SLAB -- abort")
        return "no slab"
    bowl = find_bowl(api, hm, slab)
    plate = find_plate(api, hm)
    api.log("BOWL=%s" % bowl)
    api.log("PLATE=%s" % plate)
    if bowl is None or plate is None:
        return "perception failed"

    # ------------------------------------------------------------- grasp
    held = False
    for attempt in range(2):
        gx, gy = bowl["cx"], bowl["cy"] + bowl["r"]
        gz = bowl["ztop"] - GRASP_BELOW_RIM
        api.log("GRASP#%d target x=%.4f y=%.4f z=%.4f (rim=%.4f r=%.4f)"
                % (attempt, gx, gy, gz, bowl["ztop"], bowl["r"]))
        api.grip(0.08)
        goto([gx, gy, CARRY_Z], " hover", tol=0.012, tries=2, reserve=RESERVE,
             seconds=2.0)
        goto([gx, gy, gz + 0.05], " approach", tol=0.006, tries=2, reserve=RESERVE,
             seconds=1.0)
        e = goto([gx, gy, gz], " down", tol=0.005, tries=2, reserve=RESERVE,
                 seconds=1.0)
        api.log("AT-GRASP eef=%s aim=(%.4f,%.4f,%.4f) dxy=%.4f dz=%+.4f"
                % (np.round(e, 4).tolist(), gx, gy, gz,
                   float(np.hypot(e[0] - gx, e[1] - gy)), e[2] - gz))
        api.grip(0.0)
        api.settle(0.5)
        g = api.gripper()
        api.log("CLOSED#%d width=%.4f eef=%s" % (attempt, g["width_m"],
                                                 np.round(api.eef(), 4).tolist()))
        goto([gx, gy, CARRY_Z], " lift", tol=0.012, tries=1, reserve=RESERVE - 1,
             seconds=1.0)
        g2 = api.gripper()
        api.log("LIFTED#%d width=%.4f effort=%.2f spent=%d"
                % (attempt, g2["width_m"], g2["effort"], spent[0]))
        if g2["width_m"] >= HOLD_MIN:
            held = True
            break
        if spent[0] > MOVE_BUDGET - RESERVE - 4:
            api.log("GRASP#%d empty, no budget to retry" % attempt)
            break
        api.log("GRASP#%d EMPTY -- re-perceiving" % attempt)
        api.grip(0.08)
        hm = height_map(api.capture("cam_high"))
        s2 = find_slab(api, hm)
        b2 = find_bowl(api, hm, s2, tag="R%d" % attempt) if s2 else None
        if b2 is not None:
            bowl = b2

    api.log("HELD=%s gripper=%s spent=%d" % (held, api.gripper(), spent[0]))

    # ------------------------------------------------------------- place
    px, py = plate["cx"], plate["cy"] + bowl["r"]
    pz = plate["ztop"] + PLACE_ABOVE_PLATE
    api.log("PLACE target x=%.4f y=%.4f z=%.4f" % (px, py, pz))
    goto([px, py, CARRY_Z], " overplate", tol=0.010, tries=2, seconds=2.0)
    goto([px, py, pz], " place", tol=0.008, tries=2, seconds=1.5)
    api.log("AT-PLACE eef=%s gripper=%s"
            % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(0.08)
    api.settle(0.5)
    api.log("RELEASED gripper=%s eef=%s"
            % (api.gripper(), np.round(api.eef(), 4).tolist()))
    goto([px, py, pz + 0.14], " retreat", tol=0.030, tries=1, seconds=1.0)

    # ------------------------------------------------------- verification
    hm3 = height_map(api.capture("cam_high"))
    ii0, jj0 = np.mgrid[0:NG, 0:NG]
    xs0, ys0 = xy(ii0, jj0)
    m = (~np.isnan(hm3)) & (hm3 > 0.945) & (hm3 < 1.02) & (ys0 > 0.05) & (xs0 > -0.15)
    for ii, jj in components(m)[:3]:
        if len(ii) < 25:
            continue
        xs, ys = xy(ii, jj)
        cx, cy = float((xs.min() + xs.max()) / 2), float((ys.min() + ys.max()) / 2)
        api.log("FINAL bowl? n=%d cx=%.3f cy=%.3f ztop=%.3f plate=(%.3f,%.3f) d=%.3f"
                % (len(ii), cx, cy, float(np.nanmax(hm3[ii, jj])), plate["cx"],
                   plate["cy"], float(np.hypot(cx - plate["cx"], cy - plate["cy"]))))
    return "v7 done held=%s moves=%d" % (held, spent[0])
