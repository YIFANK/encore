"""c2 cell spa_bowl_between_stock -- v1.1 (de-oracled v4).

Intent: "pick up the black bowl between the plate and the ramekin and place it
on the plate".

v1.1 de-oracle pass (mechanical, protocol compliance only): under fair-v1.0
`api.done` doubled as a runtime task-success signal. Under fair-v1.1 it means
episode termination only, so every read of it has been removed. The single
read was the `and not api.done` conjunct guarding the empty-grasp retry; the
retry's other conjunct (`gripper width below EMPTY_GAP` -- the fingers met with
nothing between them) is already a pure own-sensor test, and it can never
co-occur with a completed placement, so the guard is excised outright with no
replacement. No other logic, constant, or ordering is changed from v4.

Mechanism (every constant measured on debug seeds 51-65; see PROVENANCE):

  * The four task objects separate by height above the table plane: plate
    +0.018..0.019, ramekin +0.043 (identical on every debug seed), black bowls
    +0.051..0.056.  A rim band at table+44 mm therefore holds ONLY the two
    black bowls.
  * The two bowls, however, sometimes TOUCH (debug seeds 56 and 62: one blob of
    extent 0.19x0.23 that no z threshold splits, because the rims are 0.109 m
    apart and each rim is 0.055 m in radius).  v3 rejected that blob on extent
    and failed perception (13/15).  v4 instead fits k fixed-radius circles to
    the rim points, k = 1 or 2 by blob extent, which recovers both centres.
  * The intended bowl is the one nearer the plate; the sibling bowl sits
    0.26-0.30 m from the plate, off the plate/ramekin line.
  * A centre grasp closes on air (mouth 0.11 m > 0.078 m gripper stroke), so
    the gripper straddles the RIM: offset from the bowl centre along the
    finger-separation axis by (rim radius - 12 mm); the outer finger then stops
    against the outer wall and the rim is pinched (finger gap 0.008-0.015).
  * Carrying with that offset means the bowl centre trails the tool by the same
    vector, so the release point is the plate centre PLUS the grasp offset.
"""
import numpy as np

PROVENANCE = {
    "RIM_BAND": {
        "source": "debug seeds 51-65 (v1/v2/v3 probes + diag run "
                  "fs_c2_spa_bowl_between_stock_diag): height above the modal table "
                  "plane is plate 0.018-0.019, ramekin 0.043 on every seed, black bowl "
                  "0.051-0.056; 0.044 separates the bowls from the ramekin",
        "allowed": True},
    "MID_BAND": {"source": "same debug measurement: 0.030-0.044 holds the ramekin",
                 "allowed": True},
    "FLAT_LO/FLAT_HI": {"source": "same debug measurement: the plate occupies the "
                                  "0.008-0.028 band above the table",
                        "allowed": True},
    "ROI_X/ROI_Y": {"source": "debug seeds 51-65 layout: the four task objects sit at "
                              "y 0.17-0.36 and x -0.26..0.14, while the stove is at "
                              "y=-0.13, the cabinet at y=-0.24, the cookie box at "
                              "y=0.02-0.06",
                    "allowed": True},
    "RIM_R": {"source": "debug seeds 51-65: an isolated bowl rim measures ext "
                        "0.109-0.111 in both x and y on every seed -> radius 0.055",
              "allowed": True},
    "SPLIT_EXT": {"source": "debug seeds: an isolated bowl rim blob is <=0.112 wide, a "
                            "two-bowl blob is 0.176-0.231 wide (seeds 56, 62)",
                  "allowed": True},
    "RIM_INSET": {"source": "v2/v3 probes on debug seeds 51-65: a grasp offset of 0.043 "
                            "against the 0.055 rim radius gave a stable rim pinch "
                            "(finger gap 0.008-0.015, 21/23 episodes grasped)",
                  "allowed": True},
    "Z_GRASP_BELOW_RIM": {
        "source": "pack demos ee z at the close keyframe (0.9301/0.9244/0.9185) vs the "
                  "debug-measured bowl rim top 0.951-0.956; commanded 10 mm lower "
                  "because move_cartesian returns once inside POS_TOL",
        "allowed": True},
    "Z_PLACE": {"source": "pack demos ee z at the release keyframe "
                          "(0.9367/0.9360/0.9269), commanded 10 mm low for the same "
                          "tolerance reason",
                "allowed": True},
    "Z_CARRY": {"source": "pack demos ee_path6 apex between grasp and release "
                          "(1.0373/1.0533/1.0193), rounded up for clearance",
                "allowed": True},
    "OPEN_W": {"source": "pack demos gripper_state at rest, |q0-q1| = 0.0724",
               "allowed": True},
    "CLUSTER_CELL": {"source": "generic point-cloud mechanics: 8 mm xy grid, 4-connected",
                     "allowed": True},
    "EMPTY_GAP": {"source": "v2/v3 probes: a loaded rim pinch reports a finger gap of "
                            "0.0042-0.0153 after the lift, so below 0.0035 means the "
                            "fingers met with nothing between them",
                  "allowed": True},
}

RIM_BAND = 0.044
MID_BAND = 0.030
FLAT_LO, FLAT_HI = 0.008, 0.028
ROI_X = (-0.38, 0.34)
ROI_Y = (0.05, 0.46)
CLUSTER_CELL = 0.008
RIM_R = 0.055
SPLIT_EXT = 0.135
RIM_INSET = 0.012
Z_GRASP_BELOW_RIM = 0.035
Z_PLACE = 0.925
Z_CARRY = 1.060
OPEN_W = 0.08
EMPTY_GAP = 0.0035


def _cloud(f):
    d = np.asarray(f.depth, dtype=np.float64)
    if d.ndim == 3:
        d = d[..., 0]
    H, W = d.shape
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    P = (np.stack([(uu - cx) * d / fx, (vv - cy) * d / fy, d], axis=-1).reshape(-1, 3)
         @ T[:3, :3].T + T[:3, 3])
    ok = (np.isfinite(d) & (d > 0.02) & (d < 6.0)).reshape(-1)
    return P.reshape(H, W, 3), ok.reshape(H, W)


def _table_z(P, ok):
    m = ok & (np.abs(P[..., 0]) < 0.5) & (np.abs(P[..., 1]) < 0.5)
    zz = P[..., 2][m]
    if zz.size < 100:
        return None
    hist, edges = np.histogram(zz, bins=np.arange(zz.min(), zz.max() + 0.005, 0.005))
    i = int(np.argmax(hist))
    return float(np.median(zz[(zz >= edges[i]) & (zz < edges[i + 1])]))


def _components(pts, cell):
    ix = np.floor(pts[:, 0] / cell).astype(int)
    iy = np.floor(pts[:, 1] / cell).astype(int)
    cells = {}
    for i in range(pts.shape[0]):
        cells.setdefault((int(ix[i]), int(iy[i])), []).append(i)
    todo = set(cells)
    out = []
    while todo:
        seed = todo.pop()
        comp, stack = [seed], [seed]
        while stack:
            a, b = stack.pop()
            for n in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                if n in todo:
                    todo.discard(n)
                    comp.append(n)
                    stack.append(n)
        idx = []
        for c in comp:
            idx.extend(cells[c])
        out.append(np.asarray(idx, dtype=int))
    return out


def _fit_circles(xy, R, k):
    """k fixed-radius circles fitted to ring points; returns centres + labels."""
    if k <= 1:
        c = np.array([[0.5 * (xy[:, 0].min() + xy[:, 0].max()),
                       0.5 * (xy[:, 1].min() + xy[:, 1].max())]])
        return c, np.zeros(xy.shape[0], int)
    m = xy.mean(0)
    d = xy - m
    w, V = np.linalg.eigh(d.T @ d)
    ax = V[:, -1]
    t = d @ ax
    cs = np.vstack([xy[int(np.argmin(t))] + R * ax, xy[int(np.argmax(t))] - R * ax])
    lab = np.zeros(xy.shape[0], int)
    for _ in range(40):
        dist = np.linalg.norm(xy[:, None, :] - cs[None, :, :], axis=2)
        lab = np.argmin(np.abs(dist - R), axis=1)
        new = cs.copy()
        for j in range(k):
            sel = xy[lab == j]
            if sel.shape[0] < 10:
                continue
            v = sel - cs[j]
            n = np.linalg.norm(v, axis=1, keepdims=True)
            n[n < 1e-6] = 1e-6
            new[j] = (sel - R * v / n).mean(0)
        if float(np.max(np.abs(new - cs))) < 1e-5:
            cs = new
            break
        cs = new
    return cs, lab


def _mk(p, c, tz, cx=None, cy=None, rr=None):
    if cx is None:
        cx = 0.5 * (float(p[:, 0].min()) + float(p[:, 0].max()))
        cy = 0.5 * (float(p[:, 1].min()) + float(p[:, 1].max()))
    return dict(n=int(p.shape[0]), x=float(cx), y=float(cy),
                ztop=float(np.percentile(p[:, 2], 97)), h=float(np.percentile(p[:, 2], 97)) - tz,
                ex=float(p[:, 0].max() - p[:, 0].min()),
                ey=float(p[:, 1].max() - p[:, 1].min()),
                r=(RIM_R if rr is None else float(rr)),
                br=float(c.mean()), pts=p)


def _rim_radius(s):
    p = s["pts"]
    band = p[np.abs(p[:, 0] - s["x"]) < 0.020]
    if band.shape[0] < 20:
        return RIM_R
    hi = float(np.percentile(band[:, 1], 96)) - s["y"]
    lo = s["y"] - float(np.percentile(band[:, 1], 4))
    rr = 0.5 * (hi + lo)
    return rr if 0.040 < rr < 0.070 else RIM_R


def perceive(api, tag=""):
    f = api.capture("cam_high")
    P, ok = _cloud(f)
    tz = _table_z(P, ok)
    if tz is None:
        api.log(tag + "no table plane")
        return None, [], [], []
    rgb = np.asarray(f.rgb, dtype=np.float64)
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    roi = ok & (x > ROI_X[0]) & (x < ROI_X[1]) & (y > ROI_Y[0]) & (y < ROI_Y[1])

    # ---- bowls: rim band sits above the ramekin's top; split touching rims
    mr = roi & (z > tz + RIM_BAND) & (z < tz + 0.14)
    pr, cr = P[mr], rgb[mr]
    bowls = []
    for idx in _components(pr, CLUSTER_CELL):
        if idx.size < 80:
            continue
        p, c = pr[idx], cr[idx]
        ext = max(float(p[:, 0].max() - p[:, 0].min()), float(p[:, 1].max() - p[:, 1].min()))
        k = 1 if ext <= SPLIT_EXT else 2
        cs, lab = _fit_circles(p[:, :2], RIM_R, k)
        if k == 2 and float(np.linalg.norm(cs[0] - cs[1])) < 0.060:
            k = 1
            cs, lab = _fit_circles(p[:, :2], RIM_R, 1)
        for j in range(cs.shape[0]):
            sel = lab == j
            if int(sel.sum()) < 60:
                continue
            s = _mk(p[sel], c[sel], tz, cs[j, 0], cs[j, 1], RIM_R)
            s["split"] = (k > 1)
            if not s["split"]:
                s["r"] = _rim_radius(s)
            bowls.append(s)
    bowls.sort(key=lambda s: -s["n"])

    # ---- ramekin / other mid-height shells: clearance only
    mt = roi & (z > tz + MID_BAND) & (z < tz + RIM_BAND)
    pt, ct = P[mt], rgb[mt]
    if pt.shape[0]:
        keep = np.ones(pt.shape[0], bool)
        for s in bowls:
            keep &= np.hypot(pt[:, 0] - s["x"], pt[:, 1] - s["y"]) > s["r"] + 0.010
        pt, ct = pt[keep], ct[keep]
    mids = []
    for idx in _components(pt, CLUSTER_CELL):
        if idx.size < 150:
            continue
        s = _mk(pt[idx], ct[idx], tz)
        s["r"] = 0.5 * max(s["ex"], s["ey"])
        mids.append(s)
    mids.sort(key=lambda s: -s["n"])

    # ---- plate: flat band with every shell footprint removed
    mf = roi & (z > tz + FLAT_LO) & (z < tz + FLAT_HI)
    pf, cf = P[mf], rgb[mf]
    if pf.shape[0]:
        keep = np.ones(pf.shape[0], bool)
        for s in bowls + mids:
            keep &= np.hypot(pf[:, 0] - s["x"], pf[:, 1] - s["y"]) > s["r"] + 0.018
        pf, cf = pf[keep], cf[keep]
    flats = []
    for idx in _components(pf, CLUSTER_CELL):
        if idx.size < 150:
            continue
        s = _mk(pf[idx], cf[idx], tz)
        s["r"] = 0.5 * max(s["ex"], s["ey"])
        flats.append(s)
    flats.sort(key=lambda s: -max(s["ex"], s["ey"]))

    api.log("%stz=%.4f nbowl=%d nmid=%d nflat=%d" % (tag, tz, len(bowls), len(mids), len(flats)))
    for nm, lst in (("bowl", bowls), ("mid", mids), ("flat", flats)):
        for i, s in enumerate(lst[:4]):
            api.log("%s%s%d n=%d xy=(%.3f,%.3f) h=%.3f ext=(%.3f,%.3f) r=%.3f br=%.0f%s"
                    % (tag, nm, i, s["n"], s["x"], s["y"], s["h"], s["ex"], s["ey"],
                       s["r"], s["br"], " SPLIT" if s.get("split") else ""))
    return tz, bowls, mids, flats


def _plan(bowls, flats):
    # the plate is the bright wide disk (br 145-157 on every debug seed; the
    # cookie box, the only other flat blob, reads br 64-69)
    bright = [s for s in flats if s["br"] > 100 and max(s["ex"], s["ey"]) > 0.09]
    plate = max(bright or flats, key=lambda s: max(s["ex"], s["ey"]))
    for s in bowls:
        s["dp"] = float(np.hypot(s["x"] - plate["x"], s["y"] - plate["y"]))
    bowls.sort(key=lambda s: s["dp"])
    return plate, bowls[0]


def run(api):
    api.log("instruction=%r" % api.instruction())
    R = api.tool_rotation()
    u = np.asarray(R[:, 1][:2], float)
    u = u / (np.linalg.norm(u) + 1e-9)
    api.log("eef0=%s finger_axis=%s" % (np.round(api.eef(), 4).tolist(),
                                        np.round(u, 3).tolist()))

    tz, bowls, mids, flats = perceive(api, "P0 ")
    if tz is None or not bowls or not flats:
        return "perception failed"
    plate, tgt = _plan(bowls, flats)
    api.log("plate xy=(%.3f,%.3f) ext=%.3f br=%.0f | target xy=(%.3f,%.3f) r=%.3f "
            "d_plate=%.3f others=%s"
            % (plate["x"], plate["y"], max(plate["ex"], plate["ey"]), plate["br"],
               tgt["x"], tgt["y"], tgt["r"], tgt["dp"], [round(s["dp"], 3) for s in bowls[1:]]))

    mag = max(tgt["r"] - RIM_INSET, 0.020)
    others = list(bowls[1:]) + mids + [plate]
    best, bs = 1.0, -9.0
    for sgn in (1.0, -1.0):
        q = np.array([tgt["x"], tgt["y"]]) + sgn * mag * u
        cl = min([float(np.hypot(q[0] - o["x"], q[1] - o["y"]) - o["r"]) for o in others] or [1.0])
        api.log("side %+0.0f pt=(%.3f,%.3f) clearance=%.3f" % (sgn, q[0], q[1], cl))
        if cl > bs:
            bs, best = cl, sgn
    off = best * mag * u
    gx, gy = tgt["x"] + off[0], tgt["y"] + off[1]
    zg = float(np.clip(tgt["ztop"] - Z_GRASP_BELOW_RIM, tz + 0.008, tz + 0.045))
    api.log("grasp xy=(%.3f,%.3f) zcmd=%.3f rim_ztop=%.3f mag=%.3f" % (gx, gy, zg, tgt["ztop"], mag))

    api.grip(OPEN_W)
    api.move([gx, gy, Z_CARRY], seconds=2.5)
    r = api.move([gx, gy, zg], seconds=2.0)
    api.log("descend res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(0.0)
    api.settle(0.3)
    api.log("closed grip=%s" % api.gripper())
    api.move([gx, gy, Z_CARRY], seconds=2.0)
    g = api.gripper()
    api.log("lifted grip=%s eef=%s" % (g, np.round(api.eef(), 4).tolist()))

    if g["width_m"] < EMPTY_GAP:
        api.log("retry: fingers met empty")
        api.grip(OPEN_W)
        tz2, b2, m2, f2 = perceive(api, "R0 ")
        if tz2 is not None and b2 and f2:
            plate, t2 = _plan(b2, f2)
            off = best * max(t2["r"] - RIM_INSET, 0.020) * u
            gx, gy = t2["x"] + off[0], t2["y"] + off[1]
            zg = float(np.clip(t2["ztop"] - Z_GRASP_BELOW_RIM, tz2 + 0.008, tz2 + 0.045))
            api.log("retry grasp xy=(%.3f,%.3f) z=%.3f" % (gx, gy, zg))
            api.move([gx, gy, Z_CARRY], seconds=2.0)
            api.move([gx, gy, zg], seconds=2.0)
            api.grip(0.0)
            api.settle(0.3)
            api.move([gx, gy, Z_CARRY], seconds=2.0)
            api.log("retry lifted grip=%s" % api.gripper())

    px, py = plate["x"] + off[0], plate["y"] + off[1]
    api.move([px, py, Z_CARRY], seconds=2.5)
    r = api.move([px, py, Z_PLACE], seconds=2.0)
    api.log("place res=%.4f eef=%s grip=%s" % (r, np.round(api.eef(), 4).tolist(),
                                               api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.4)
    api.move([px, py, Z_CARRY], seconds=1.5)
    api.settle(0.5)
    perceive(api, "P1 ")
    return "v4 done"
