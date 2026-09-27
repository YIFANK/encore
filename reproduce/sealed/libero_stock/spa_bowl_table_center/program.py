"""c2 cell spa_bowl_table_center_stock -- v1.1 (== v4 body, de-oracled).

v1.1 de-oracle pass (mechanical, no tuning): the single read of `api.done`
(a trailing diagnostic log field) is excised.  It was never in control flow --
run() has no success-conditioned early exit and no success-gated retry/stop:
the one retry branch is already sensor-based (`api.gripper()["effort"]`).
No constant, threshold, waypoint or branch is otherwise changed from v4.

Intent: pick up the black bowl from table center and place it on the plate.

Mechanism (measured, see PROVENANCE):
  * The bowl is a ~111 mm wide open bowl; the jaws (which close along world y)
    cannot span it, so the demos pinch the RIM: the demo EEF at the closing
    frame sits one rim radius on the +y side of the bowl centre.
  * Depth from cam_high sees the rim ring plus the far inner wall, so the
    point CENTROID is biased away from the camera in x; the bbox MIDPOINT of
    an above-table cluster is the unbiased bowl centre (verified on seed 51:
    bbox mid (-0.0756,-0.0007), half-extents 0.0554/0.0553).
  * v4 grounds both the bowl and the plate per-episode instead of replaying
    the demo-mean waypoints, and falls back to the demo-mean if a ground
    fails.
"""
import numpy as np

PROVENANCE = {
    "NOM_GRASP_XY": {
        "source": "pack.json demos[*].keyframes: EEF xy at the gripper-close "
                  "frame (d0 -0.0758,0.0402; d1 -0.0732,0.0587; "
                  "d2 -0.0671,0.0562) -> mean; used as fallback + as the "
                  "'table center' anchor for target selection",
        "allowed": True},
    "NOM_PLACE_XY": {
        "source": "pack.json demos[*].keyframes: EEF xy at the release frame "
                  "(0.0634,0.2268 / 0.0565,0.2166 / 0.0619,0.2375) -> mean",
        "allowed": True},
    "RIM_OFF_Y": {
        "source": "debug seed 51-57 (results/fs_c2_spa_bowl_table_center_"
                  "stock_v1): successful rim pinches closed at EEF y "
                  "0.0503/0.0508/0.0487/0.0487 while the bowl bbox-mid y was "
                  "-0.0007/0.0040/-0.0120/-0.0122 -> +0.051 mean",
        "allowed": True},
    "GRASP_CLEAR": {
        "source": "debug seed 51: EEF z 0.922 at the holding pinch, table "
                  "plane 0.9025 (modal depth) -> +0.0195; matches pack demo "
                  "close heights 0.917-0.945",
        "allowed": True},
    "PLACE_CLEAR": {
        "source": "debug seed 51: plate top 0.9199, v1 release settled at "
                  "0.9518 with the bowl resting -> aim +0.030 above plate top",
        "allowed": True},
    "TRAVEL_Z": {
        "source": "pack.json demos[*].ee_path6 transfer-arc apex "
                  "(1.0298-1.0796)",
        "allowed": True},
    "BOWL_BAND": {
        "source": "debug seed 51-57 cluster survey: the demo-grasped bowl "
                  "measures 0.049 tall and 0.111 wide above the table plane",
        "allowed": True},
    "PLATE_BAND": {
        "source": "debug seed 51 cluster survey: the flat light-grey disc at "
                  "the demo release point measures 0.017 tall, 0.127x0.131 "
                  "wide, mean rgb (144,135,131)",
        "allowed": True},
    "CELL_M": {"source": "clustering grid resolution (algorithm parameter, "
                         "not a scene measurement)", "allowed": True},
    "OPEN_W": {"source": "FairApi.grip semantics (>=0.025 opens) -- generic "
                         "gripper mechanics", "allowed": True},
    "CLOSE_W": {"source": "FairApi.grip semantics (<0.025 closes) -- generic "
                          "gripper mechanics", "allowed": True},
    "HOLD_EFFORT": {"source": "FairApi.gripper() contract: effort 3.0 iff "
                              "holding", "allowed": True},
    "MAX_BOWL_D": {
        "source": "debug seeds 51-65: the target bowl's bbox mid stays within "
                  "0.013 of the pack anchor (x -0.0756 fixed, y -0.012..0.007)"
                  "; 0.10 admits far more drift than observed while still "
                  "excluding the neighbouring bowls (0.20-0.30 away)",
        "allowed": True},
    "MAX_PLATE_D": {
        "source": "debug seeds 51-65: the plate disc mid spans x 0.050-0.096, "
                  "y 0.184-0.212, i.e. <=0.045 from the pack release anchor",
        "allowed": True},
}

NOM_GRASP_XY = (-0.0720, 0.0517)
NOM_PLACE_XY = (0.0606, 0.2270)
RIM_OFF_Y = 0.051
GRASP_CLEAR = 0.0195
PLACE_CLEAR = 0.030
TRAVEL_Z = 1.060
BOWL_BAND = {"h": (0.030, 0.068), "w": (0.085, 0.145)}
PLATE_BAND = {"h": (0.008, 0.030), "w": (0.100, 0.175)}
CELL_M = 0.012
OPEN_W = 0.080
CLOSE_W = 0.000
HOLD_EFFORT = 1.0
MAX_BOWL_D = 0.10
MAX_PLATE_D = 0.13


# ---------------------------------------------------------------- perception
def cloud(frame):
    d = np.asarray(frame.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    pts = np.stack([x, y, d, np.ones_like(d)], axis=-1) @ T.T
    ok = np.isfinite(d) & (d > 0.02)
    return pts[..., :3], ok


def table_z(P, ok):
    m = ok & (np.abs(P[..., 0]) < 0.36) & (np.abs(P[..., 1]) < 0.36)
    z = P[..., 2][m]
    z = z[(z > 0.6) & (z < 1.25)]
    if z.size == 0:
        return 0.9025
    hist, edges = np.histogram(z, bins=130, range=(0.6, 1.25))
    i = int(np.argmax(hist))
    return float(0.5 * (edges[i] + edges[i + 1]))


def clusters(P, ok, rgb, tz, zmin=0.010, zmax=0.30):
    m = (ok & (P[..., 2] > tz + zmin) & (P[..., 2] < tz + zmax)
         & (np.abs(P[..., 0]) < 0.36) & (np.abs(P[..., 1]) < 0.36))
    if not m.any():
        return []
    xs, ys, zs = P[..., 0][m], P[..., 1][m], P[..., 2][m]
    cols = np.asarray(rgb, float)[m]
    gx = np.floor(xs / CELL_M).astype(int)
    gy = np.floor(ys / CELL_M).astype(int)
    cells = {}
    for k in range(gx.size):
        cells.setdefault((int(gx[k]), int(gy[k])), []).append(k)
    seen, out = set(), []
    for c0 in cells:
        if c0 in seen:
            continue
        stack, comp = [c0], []
        seen.add(c0)
        while stack:
            c = stack.pop()
            comp.append(c)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in cells and n not in seen:
                        seen.add(n)
                        stack.append(n)
        mem = np.array([k for c in comp for k in cells[c]], dtype=int)
        if mem.size < 60:
            continue
        x0, x1 = float(xs[mem].min()), float(xs[mem].max())
        y0, y1 = float(ys[mem].min()), float(ys[mem].max())
        out.append({
            "n": int(mem.size),
            "mid": [round(0.5 * (x0 + x1), 4), round(0.5 * (y0 + y1), 4)],
            "cen": [round(float(xs[mem].mean()), 4),
                    round(float(ys[mem].mean()), 4)],
            "w": [round(x1 - x0, 4), round(y1 - y0, 4)],
            "h": round(float(np.percentile(zs[mem], 98)) - tz, 4),
            "ztop": round(float(np.percentile(zs[mem], 98)), 4),
            "rgb": [int(v) for v in cols[mem].mean(axis=0).round()],
        })
    out.sort(key=lambda c: -c["n"])
    return out


def flat_discs(P, ok, rgb, tz):
    """Clusters of cells whose LOCAL max height is inside the plate band.

    Plain point clustering merges the plate with a bowl standing against it
    (measured: debug seeds 53/55/57/59/63 gave one 0.169x0.231 blob).  A bowl
    is 49 mm tall and its rim overhangs its base, so every cell it covers has
    a max height far above the plate band -- filtering cells by their own max
    height therefore deletes the bowl and leaves the disc intact.
    """
    m = (ok & (P[..., 2] > tz + 0.006) & (P[..., 2] < tz + 0.30)
         & (np.abs(P[..., 0]) < 0.36) & (np.abs(P[..., 1]) < 0.36))
    if not m.any():
        return []
    xs, ys, zs = P[..., 0][m], P[..., 1][m], P[..., 2][m]
    cols = np.asarray(rgb, float)[m]
    gx = np.floor(xs / CELL_M).astype(int)
    gy = np.floor(ys / CELL_M).astype(int)
    cells = {}
    for k in range(gx.size):
        cells.setdefault((int(gx[k]), int(gy[k])), []).append(k)
    flat = {}
    for c, ks in cells.items():
        mem = np.array(ks, dtype=int)
        if mem.size < 8:
            continue
        hmax = float(zs[mem].max()) - tz
        if PLATE_BAND["h"][0] <= hmax <= PLATE_BAND["h"][1]:
            flat[c] = mem
    seen, out = set(), []
    for c0 in flat:
        if c0 in seen:
            continue
        stack, comp = [c0], []
        seen.add(c0)
        while stack:
            c = stack.pop()
            comp.append(c)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in flat and n not in seen:
                        seen.add(n)
                        stack.append(n)
        mem = np.concatenate([flat[c] for c in comp])
        if mem.size < 400:
            continue
        x0, x1 = float(xs[mem].min()), float(xs[mem].max())
        y0, y1 = float(ys[mem].min()), float(ys[mem].max())
        out.append({
            "n": int(mem.size),
            "mid": [round(0.5 * (x0 + x1), 4), round(0.5 * (y0 + y1), 4)],
            "w": [round(x1 - x0, 4), round(y1 - y0, 4)],
            "h": round(float(np.percentile(zs[mem], 98)) - tz, 4),
            "ztop": round(float(zs[mem].max()), 4),
            "rgb": [int(v) for v in cols[mem].mean(axis=0).round()],
        })
    out.sort(key=lambda c: -c["n"])
    return out


def pick_near(cands, band, anchor, max_d):
    """Band-filter clusters, take the one nearest the demo anchor, and refuse
    it if it sits further from that anchor than the layout ever drifts --
    a mis-ground must degrade to the demo-mean waypoint, never to a different
    object."""
    hit = [c for c in cands
           if band["h"][0] <= c["h"] <= band["h"][1]
           and band["w"][0] <= c["w"][0] <= band["w"][1]
           and band["w"][0] <= c["w"][1] <= band["w"][1]]
    if not hit:
        return None
    ax, ay = anchor
    hit.sort(key=lambda c: (c["mid"][0] - ax) ** 2 + (c["mid"][1] - ay) ** 2)
    best = hit[0]
    d = float(np.hypot(best["mid"][0] - ax, best["mid"][1] - ay))
    return None if d > max_d else best


# ------------------------------------------------------------------- motion
def goto(api, xyz, seconds=2.0, refine=1, tag=""):
    tgt = np.asarray(xyz, float)
    api.move(tgt, seconds=seconds)
    for _ in range(refine):
        err = tgt - np.asarray(api.eef(), float)
        if float(np.linalg.norm(err)) < 0.004:
            break
        api.move(tgt + err, seconds=1.0)
    cur = np.asarray(api.eef(), float)
    api.log("goto %s tgt=%s eef=%s err=%.4f" %
            (tag, np.round(tgt, 4).tolist(), np.round(cur, 4).tolist(),
             float(np.linalg.norm(tgt - cur))))
    return cur


def survey(api, tag):
    f = api.capture("cam_high")
    P, ok = cloud(f)
    tz = table_z(P, ok)
    cl = clusters(P, ok, f.rgb, tz)
    fd = flat_discs(P, ok, f.rgb, tz)
    api.log("%s table_z=%.4f nclust=%d nflat=%d" % (tag, tz, len(cl), len(fd)))
    for i, c in enumerate(cl[:8]):
        api.log("%s c%d %s" % (tag, i, c))
    for i, c in enumerate(fd[:6]):
        api.log("%s flat%d %s" % (tag, i, c))
    return tz, cl, fd


def run(api):
    api.log("instruction: %r" % api.instruction())
    api.grip(OPEN_W)

    tz, cl, fd = survey(api, "pre")
    anchor_b = (NOM_GRASP_XY[0], NOM_GRASP_XY[1] - RIM_OFF_Y)
    bowl = pick_near(cl, BOWL_BAND, anchor_b, MAX_BOWL_D)
    plate = pick_near(fd, PLATE_BAND, NOM_PLACE_XY, MAX_PLATE_D)
    if plate is None:                       # last resort: un-split clusters
        plate = pick_near(cl, PLATE_BAND, NOM_PLACE_XY, MAX_PLATE_D)
    api.log("bowl=%s" % bowl)
    api.log("plate=%s" % plate)

    if bowl is None:
        gx, gy = NOM_GRASP_XY
        gz = tz + GRASP_CLEAR
        api.log("FALLBACK grasp -> demo nominal")
    else:
        gx = bowl["mid"][0]
        gy = bowl["mid"][1] + RIM_OFF_Y
        gz = tz + GRASP_CLEAR

    if plate is None:
        px, py = NOM_PLACE_XY
        pz = tz + 0.0174 + PLACE_CLEAR
        api.log("FALLBACK place -> demo nominal")
    else:
        # the held bowl hangs somewhere between "rigidly offset by RIM_OFF_Y"
        # and "directly under the jaws"; aiming half a rim offset past the
        # plate centre keeps it inside the disc under either model.
        px = plate["mid"][0]
        py = plate["mid"][1] + 0.5 * RIM_OFF_Y
        pz = plate["ztop"] + PLACE_CLEAR
    api.log("targets grasp=(%.4f,%.4f,%.4f) place=(%.4f,%.4f,%.4f)"
            % (gx, gy, gz, px, py, pz))

    goto(api, [gx, gy, TRAVEL_Z], seconds=2.5, tag="pre")
    goto(api, [gx, gy, gz], seconds=2.0, tag="descend")
    api.grip(CLOSE_W)
    api.settle(0.3)
    g = api.gripper()
    api.log("after close: grip=%s eef=%s" % (g, np.round(api.eef(), 4).tolist()))

    if g["effort"] < HOLD_EFFORT:
        api.log("RETRY: no hold at the first pinch")
        api.grip(OPEN_W)
        goto(api, [gx, gy, TRAVEL_Z], seconds=1.5, tag="retry_up")
        tz2, cl2, _fd2 = survey(api, "retry")
        b2 = pick_near(cl2, BOWL_BAND, anchor_b, MAX_BOWL_D)
        if b2 is not None:
            gx = b2["mid"][0]
            gy = b2["mid"][1] + RIM_OFF_Y
            gz = tz2 + GRASP_CLEAR
        goto(api, [gx, gy, TRAVEL_Z], seconds=1.5, tag="retry_pre")
        goto(api, [gx, gy, gz], seconds=1.5, tag="retry_descend")
        api.grip(CLOSE_W)
        api.settle(0.3)
        api.log("after retry close: grip=%s" % api.gripper())

    goto(api, [gx, gy, TRAVEL_Z], seconds=2.0, tag="lift")
    goto(api, [px, py, TRAVEL_Z], seconds=2.5, tag="over_plate")
    goto(api, [px, py, pz], seconds=2.0, tag="place")
    api.log("at place: grip=%s" % api.gripper())
    api.grip(OPEN_W)
    api.settle(0.4)
    goto(api, [px, py, TRAVEL_Z], seconds=2.0, tag="retreat")
    api.log("final eef=%s" % (np.round(api.eef(), 4).tolist(),))
    survey(api, "post")
    return "v1.1 grounded"
