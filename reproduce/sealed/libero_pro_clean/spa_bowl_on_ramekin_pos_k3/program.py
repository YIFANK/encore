"""v7 -- straddle the bowl wall using the rim height measured AT the straddle
point, not the bowl's global top.

Measured on the debug seeds (probes v0-v6):
  * table 0.9012; three props -- a bright plate (mean RGB ~158) and two dark
    bowls (~120/~136). One bowl stands on a ramekin (global top ~1.000); the
    other rests on the table (top ~0.952), so the bowl asset is ~51 mm tall.
  * the bowl on the ramekin is TILTED: its rim reads 1.0006 at the far-x edge
    and 0.9726 at the near-x edge, ~26 mm across the bowl. Aiming the jaws at
    the global top therefore closes them in free air above the near rim (v5:
    0/8, finger gap 0.0011 every time). Aiming at the rim height measured in
    the straddle column is the fix.
  * closed fingertips pressed on the table stop with api.eef() 9 mm above it.
  * the jaws separate along base y, so the wall is straddled by putting the eef
    on the wall's midline, found per-column as the mean of the rim's outer y
    edge and the y where the inner surface has dropped 10 mm. That wall
    measures 7.5 mm thick -- the pack's demo holds at a 7 mm finger gap.
"""
import numpy as np

PROVENANCE = {
    "GRID_BOX": {"source": "debug-seed observation (v0 probe): cam_high point-cloud extent",
                 "allowed": True},
    "ZCAP": {"source": "debug-seed observation (v0/v1): arm and cabinet tops lie above 1.06 m",
             "allowed": True},
    "N_MIN/N_MAX": {"source": "debug-seed observation (v1/v2): prop footprints span 57-110 grid cells",
                    "allowed": True},
    "FINE_C/FINE_M": {"source": "debug-seed choice (v6 probe): 2.5 mm cells over a 0.12 m window "
                                "resolve the rim wall", "allowed": True},
    "BAND": {"source": "debug-seed measurement (v6): the top 30 mm band of the target is its bowl "
                       "silhouette, giving centre and radius (r ~ 0.053)", "allowed": True},
    "GRASP_TIP_DEPTH": {"source": "debug-seed measurement (v6): 10 mm below the local rim the wall is "
                                  "7.5 mm thick, matching the pack demo's 7 mm closed finger gap",
                        "allowed": True},
    "TIP_BELOW_EEF": {"source": "debug-seed measurement (v4b): closed fingers pressed on the table "
                                "stop with eef 9 mm above it", "allowed": True},
    "BOWL_H": {"source": "debug-seed measurement: twin bowl top minus table z (~0.051)", "allowed": True},
    "DROP_CLEAR": {"source": "debug-seed choice: release the bowl 12 mm above the plate top",
                   "allowed": True},
    "REFINE_TOL": {"source": "generic controller mechanics: the harness move stops inside a 12 mm "
                             "ball, so the command is re-issued against the measured offset",
                   "allowed": True},
}

X0, X1, Y0, Y1, N = -0.40, 0.35, -0.40, 0.40, 60
CW, CH = (X1 - X0) / N, (Y1 - Y0) / N
ZCAP = 1.06
N_MIN, N_MAX = 25, 145
FINE_C, FINE_M = 0.0025, 48
BAND = 0.030
GRASP_TIP_DEPTH = 0.010
TIP_BELOW_EEF = 0.009
DROP_CLEAR = 0.012


def cloud(f):
    h, w = f.depth.shape[:2]
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    z = f.depth.astype(float)
    K = f.intrinsics
    x = (us - K[0, 2]) * z / K[0, 0]
    y = (vs - K[1, 2]) * z / K[1, 1]
    P = np.stack([x, y, z, np.ones_like(z)], -1) @ f.t_base_cam.T
    return P[..., :3], (np.isfinite(z) & (z > 0))


def _comps(mask):
    lab = np.zeros((N, N), int); cur = 0
    for i in range(N):
        for j in range(N):
            if mask[i, j] and lab[i, j] == 0:
                cur += 1; st = [(i, j)]; lab[i, j] = cur
                while st:
                    a, b = st.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if 0 <= p < N and 0 <= q < N and mask[p, q] and lab[p, q] == 0:
                                lab[p, q] = cur; st.append((p, q))
    return lab, cur


def look(api):
    f = api.capture("cam_high")
    P, ok = cloud(f)
    xs, ys, zs = P[..., 0], P[..., 1], P[..., 2]
    m = ok & (xs >= X0) & (xs < X1) & (ys >= Y0) & (ys < Y1)
    ix = np.clip(((xs - X0) / CW).astype(int), 0, N - 1)
    iy = np.clip(((ys - Y0) / CH).astype(int), 0, N - 1)
    hm = np.full((N, N), -1.0)
    cm = np.zeros((N, N, 3)); cnt = np.zeros((N, N))
    np.maximum.at(hm, (ix[m], iy[m]), zs[m])
    np.add.at(cm, (ix[m], iy[m]), f.rgb[m].astype(float))
    np.add.at(cnt, (ix[m], iy[m]), 1)
    cm /= np.maximum(cnt, 1)[..., None]
    table = float(np.median(hm[hm > 0]))
    lab, K = _comps((hm > table + 0.012) & (hm < ZCAP))
    props = []
    for k in range(1, K + 1):
        ii, jj = np.where(lab == k)
        if not (N_MIN <= len(ii) <= N_MAX):
            continue
        xlo, xhi = X0 + (ii.min() + .5) * CW, X0 + (ii.max() + .5) * CW
        ylo, yhi = Y0 + (jj.min() + .5) * CH, Y0 + (jj.max() + .5) * CH
        if 0.5 * (xlo + xhi) < -0.30:
            continue
        props.append({"n": len(ii), "x": 0.5 * (xlo + xhi), "y": 0.5 * (ylo + yhi),
                      "top": float(hm[ii, jj].max()), "lum": float(cm[ii, jj].mean())})
    return P, ok, table, props, hm


def fine(P, ok, cx0, cy0):
    xs, ys, zs = P[..., 0], P[..., 1], P[..., 2]
    x0, y0 = cx0 - FINE_M * FINE_C / 2, cy0 - FINE_M * FINE_C / 2
    sub = ok & (xs >= x0) & (xs < x0 + FINE_M * FINE_C) & \
          (ys >= y0) & (ys < y0 + FINE_M * FINE_C) & (zs < ZCAP)
    jx = np.clip(((xs - x0) / FINE_C).astype(int), 0, FINE_M - 1)
    jy = np.clip(((ys - y0) / FINE_C).astype(int), 0, FINE_M - 1)
    A = np.full((FINE_M, FINE_M), -9.0)
    np.maximum.at(A, (jx[sub], jy[sub]), zs[sub])
    return A, x0, y0


def circle(A, x0, y0):
    top = float(A.max())
    ii, jj = np.where(A > top - BAND)
    xs = x0 + np.array([ii.min(), ii.max()]) * FINE_C
    ys = y0 + np.array([jj.min(), jj.max()]) * FINE_C
    cx, cy = float(xs.mean()), float(ys.mean())
    r = 0.25 * float((xs[1] - xs[0]) + (ys[1] - ys[0]))
    return cx, cy, r, top


def wall(A, x0, y0, cx, cy, r, side):
    """Rim height and wall midline in the straddle column, on the given y side."""
    ic = int(round((cx - x0) / FINE_C))
    rows = [i for i in range(ic - 2, ic + 3) if 0 <= i < FINE_M]
    ytar = cy + side * r
    vals = []
    for i in rows:
        for j in range(FINE_M):
            yj = y0 + j * FINE_C
            if A[i, j] > -1 and abs(yj - ytar) < 0.013:
                vals.append(A[i, j])
    if not vals:
        return None
    ltop = float(max(vals))
    mids = []
    for i in [ic - 1, ic, ic + 1]:
        if not (0 <= i < FINE_M):
            continue
        order = range(FINE_M - 1, -1, -1) if side > 0 else range(FINE_M)
        yr = jr = None
        for j in order:
            if A[i, j] > ltop - 0.006:
                yr, jr = y0 + j * FINE_C, j
                break
        if yr is None:
            continue
        seq = range(jr, -1, -1) if side > 0 else range(jr, FINE_M)
        for j in seq:
            if -1 < A[i, j] < ltop - GRASP_TIP_DEPTH:
                mids.append(0.5 * (yr + y0 + j * FINE_C))
                break
    if not mids:
        return None
    return ltop, float(np.median(mids))


def site_top(hm, table, x, y, cap, rad=0.055):
    best = table
    for i in range(N):
        gx = X0 + (i + 0.5) * CW
        if abs(gx - x) > rad:
            continue
        for j in range(N):
            gy = Y0 + (j + 0.5) * CH
            if abs(gy - y) > rad or (gx - x) ** 2 + (gy - y) ** 2 > rad * rad:
                continue
            if best < hm[i, j] < cap:
                best = float(hm[i, j])
    return best


def go(api, tgt, R, log, tol=0.003, iters=3, axes=3, tag=""):
    """Re-issue the command against the measured offset; the harness move only
    guarantees a 12 mm ball."""
    tgt = np.array(tgt, float)
    cmd = tgt.copy()
    for _ in range(iters):
        api.move(cmd, rotation=R, seconds=1.4)
        e = api.eef()
        err = tgt - e
        if max(abs(err[:axes])) < tol:
            break
        cmd = cmd + np.concatenate([err[:axes], np.zeros(3 - axes)])
        cmd = tgt + np.clip(cmd - tgt, -0.035, 0.035)
    log("%s eef=%s want=%s" % (tag, np.round(api.eef(), 4).tolist(), np.round(tgt, 4).tolist()))
    return api.eef()


def run(api):
    R = api.tool_rotation()
    P, ok, table, props, hm = look(api)
    api.log("table=%.4f props=%s" % (table, [(round(p["x"], 3), round(p["y"], 3),
                                              round(p["top"], 3), round(p["lum"])) for p in props]))
    if len(props) < 3:
        return "perception: %d props" % len(props)
    plate = max(props, key=lambda p: p["lum"])
    bowls = [p for p in props if p is not plate]
    target = max(bowls, key=lambda p: p["top"])
    twin = min(bowls, key=lambda p: p["top"])
    bowl_h = twin["top"] - table
    Ap, px0, py0 = fine(P, ok, plate["x"], plate["y"])
    pcx, pcy, _pr, ptop = circle(Ap, px0, py0)
    api.log("plate=(%.4f,%.4f,%.4f) twin_top=%.4f bowl_h=%.4f" % (pcx, pcy, ptop, twin["top"], bowl_h))

    held = False
    side = 1.0
    grasp_off = 0.0
    for attempt in range(3):
        if attempt:
            P, ok, table, props2, hm = look(api)
            cand = [p for p in props2 if abs(p["x"] - target["x"]) < 0.10 and abs(p["y"] - target["y"]) < 0.10]
            if not cand:
                api.log("attempt%d: target not found" % attempt)
                break
            target = max(cand, key=lambda p: p["top"])
        A, ax0, ay0 = fine(P, ok, target["x"], target["y"])
        cx, cy, r, gtop = circle(A, ax0, ay0)
        w = wall(A, ax0, ay0, cx, cy, r, side)
        if w is None:
            ltop, gy = gtop - 0.015, cy + side * (r - 0.004)
        else:
            ltop, gy = w
        gz = ltop - GRASP_TIP_DEPTH + TIP_BELOW_EEF
        grasp_off = gy - cy
        api.log("attempt%d side=%+.0f centre=(%.4f,%.4f) r=%.4f gtop=%.4f ltop=%.4f -> (%.4f,%.4f,%.4f)"
                % (attempt, side, cx, cy, r, gtop, ltop, cx, gy, gz))
        api.grip(0.08)
        go(api, [cx, gy, ltop + 0.075], R, api.log, axes=2, tag="hover")
        go(api, [cx, gy, gz], R, api.log, axes=3, tag="descend")
        api.grip(0.0)
        api.settle(0.2)
        api.log("closed %s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))
        lift_z = gtop + 0.14
        api.move([cx, gy, lift_z], rotation=R, seconds=1.5)
        api.move([0.5 * (cx + pcx), 0.5 * (cy + pcy), lift_z], rotation=R, seconds=2.0)
        api.log("staged eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
        _P, _o, t2, _p, hm2 = look(api)
        still = site_top(hm2, t2, cx, cy, cap=gtop + 0.03)
        api.log("site_top after lift=%.4f (was %.4f)" % (still, gtop))
        if still < gtop - 0.025:
            held = True
            break
        api.log("attempt%d: bowl still on its stand" % attempt)
        side = -side
        api.grip(0.08)

    if not held:
        return "grasp not confirmed"

    hang = bowl_h - GRASP_TIP_DEPTH + TIP_BELOW_EEF
    place_z = ptop + hang + DROP_CLEAR
    cruise = max(place_z + 0.10, ptop + 0.16)
    gy_place = pcy + grasp_off
    api.move([pcx, gy_place, cruise], rotation=R, seconds=2.0)
    go(api, [pcx, gy_place, cruise], R, api.log, axes=2, tag="over plate")
    go(api, [pcx, gy_place, place_z], R, api.log, axes=3, tag="place")
    api.log("hang=%.4f place_z=%.4f grip=%s" % (hang, place_z, api.gripper()))
    api.grip(0.08)
    api.settle(0.4)
    api.move([pcx, gy_place, cruise], rotation=R, seconds=1.2)
    api.settle(0.4)
    return "v7 done"
