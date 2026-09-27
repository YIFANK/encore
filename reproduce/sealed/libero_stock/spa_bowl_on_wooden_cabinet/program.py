"""c2 cell spa_bowl_on_wooden_cabinet_stock -- v1.1.1

v1.1.1 = v4 with every `api.done` read EXCISED (fair-v1.1.1: LIBERO ends the
episode when the success predicate fires, so even a "terminated only" flag is an
indirect success oracle).  v4 read `.done` at exactly three sites, and the v4
selection logs (results/sel_c2_..._v4, all 15 seeds) show none of the three ever
decided anything: every seed held on the FIRST "+y" pinch, so "pinch -y",
"pinch +x", "ATTEMPT 2", "held=False" and "NO HOLD" appear zero times across the
15 logs.

  1. grasp-search early return   `if api.done: return "terminated in grasp"`
     -- reached only after a pinch that failed the hold test; DEAD on every
     debug seed.  DELETED with no replacement: the search is already bounded by
     3 rim candidates x 2 attempts, so removing the exit cannot loop.
  2. attempt-loop break          `if held or api.done: break`
     -- `held` was True on all 15 seeds, so the `.done` disjunct never chose the
     branch.  REPLACED by `if held: break`, i.e. the sensor half alone (finger
     gap + the cabinet-band re-perception second opinion, unchanged).  The
     enclosing `for attempt in range(2)` still bounds the loop.
  3. final log line              `L("done=%s" % api.done)`
     -- pure logging, after all actuation.  DELETED.

No other change: every constant, threshold, offset and motion is byte-identical
to v4, and PROVENANCE is unchanged (no new constants were introduced).

Original v4 header follows.
--------------------------------------------------------------------------
v4 = v3 (8/8 on debug seeds 51..65 odd, results/fs_c2_..._v3, byte-identical
sim_steps to v2 -> the recovery paths never fired) with the hold test fixed.
v3's threshold was base_gap+0.0020 = 0.0038 while the measured carry-time gap
of a genuine rim pinch was 0.0040-0.0046: one millimetre of margin, and a false
"lost" verdict would have opened the fingers 31 cm above the plate.  Measured
separation: an EMPTY close reads 0.0018 on every seed, a real pinch 0.0040 at
its lowest, so v4 splits them at 0.0029 and, more importantly, cross-checks the
gap against a free second opinion -- after a 10 cm lift the bowl is out of the
cabinet-rim band, so if the band is still occupied the pick failed no matter
what the fingers report.

Original v3/v2 headers follow.
--------------------------------------------------------------------------
v3 = v2 (8/8 on debug seeds 51..65 odd, results/fs_c2_..._v2) plus three
robustness changes that cannot fire on the seeds v2 already solved:
  * the pick is retried from scratch (re-perceive + re-pinch) if the finger gap
    says the bowl was lost during the lift or the carry;
  * the bowl search disc grows 0.17 -> 0.19 m (observed bowl centres sit
    0.063-0.089 m from the pack anchor, so the disc must contain centre+rim);
  * the descent is logged before the close so a blocked descent can be told
    apart from a close that drags the arm (seeds 61/63 ended 25 mm high).

Original v2 header follows.
--------------------------------------------------------------------------
c2 cell spa_bowl_on_wooden_cabinet_stock -- v2

"pick up the black bowl on the wooden cabinet and place it on the plate"

v1 receipt (results/fs_c2_..._v1, 8 debug seeds, 0/8): the wrist-camera cloud is
unusable (it reports a blob at z=1.33 while the EEF is at 1.28), so every grasp
closed on air 15 cm too high.  But cam_high resolved the scene cleanly:

  * the disc around the pack pick-anchor contains three z-modes: the table
    (0.900), the cabinet top (1.120) and the bowl rim (1.170-1.180);
  * the bowl-rim top z measured 1.1798/1.1799/1.1798/1.1799 on debug seeds
    51/53/57/61 -- deterministic; only the bowl's xy jitters;
  * on the two seeds where the cabinet top happened to win the histogram mode
    the isolated bowl blob measured ext (0.120,0.108) -> rim radius ~0.056;
  * the plate is a flat blob, top z 0.9201 on every seed, ext ~(0.144,0.156).

v2 therefore drops the wrist camera, isolates the bowl by an absolute-height
BAND under the rim instead of a histogram mode, and pinches the rim on the +y
side (see GRASP_OFFSET below).
"""
import numpy as np

PROVENANCE = {
    "PICK_ANCHOR_XY": {
        "source": "pack.json: mean over the 3 demos of the ee xy at the keyframe "
                  "where gripper_cmd flips to +1 (close): "
                  "(-0.0097,-0.2572)/(-0.0150,-0.2175)/(-0.0167,-0.2153)",
        "allowed": True},
    "PLACE_ANCHOR_XY": {
        "source": "pack.json: mean over the 3 demos of the ee xy at the release "
                  "keyframe (gripper_cmd back to -1, fingers still closed): "
                  "(0.0527,0.2182)/(0.0717,0.2383)/(0.0676,0.2142)",
        "allowed": True},
    "DEMO_GRASP_Z": {
        "source": "pack.json: mean ee z at those close keyframes "
                  "(1.1590/1.1467/1.1522)",
        "allowed": True},
    "RIM_TOP_Z": {
        "source": "debug-seed measurement (results/fs_c2_..._v1 program logs, "
                  "seeds 51/53/57/61): top of the cam_high depth cloud in the "
                  "pick-anchor disc = 1.1798/1.1799/1.1798/1.1799",
        "allowed": True},
    "RIM_TO_EEF_DZ": {
        "source": "DEMO_GRASP_Z minus RIM_TOP_Z = 1.1526-1.1798 = -0.0272 m: how "
                  "far below the perceived rim the demos put the EEF to close",
        "allowed": True},
    "CABINET_TOP_Z": {
        "source": "debug-seed measurement (v1 logs): second z-mode of the "
                  "pick-anchor disc, 1.12 on seeds 51/53/57/61",
        "allowed": True},
    "PLATE_TOP_Z": {
        "source": "debug-seed measurement (v1 logs): plate blob ztop 0.9201 on "
                  "seeds 51/53/57/61",
        "allowed": True},
    "RELEASE_DZ": {
        "source": "pack.json mean release ee z (0.9670) minus the debug-measured "
                  "plate top (0.9201) = 0.0469 m",
        "allowed": True},
    "PLACE_OFFSET_XY": {
        "source": "pack.json mean release ee xy (0.0640,0.2236) minus the mean "
                  "debug-measured plate centre (0.0660,0.1965 over seeds "
                  "51/53/57/61) = (-0.002,+0.027)",
        "allowed": True},
    "RIM_R_FALLBACK": {
        "source": "debug-seed measurement (v1 logs, seeds 57/61): isolated bowl "
                  "blob ext (0.120,0.108) -> radius ~0.056",
        "allowed": True},
    "TRANSIT_Z": {
        "source": "pack.json: max ee z on the demos' carry segment of ee_path6 "
                  "(1.3150/1.2783/1.3085), rounded down",
        "allowed": True},
    "HOLD_MARGIN": {
        "source": "debug-seed measurement: the empty-close gap is 0.0018 on every "
                  "seed (v2/v3 logs) and the lowest gap of a genuine rim pinch was "
                  "0.0040 (seed 61), so the discriminator is put midway at "
                  "base+0.0011; the demos' own closed gaps were 0.0045-0.0054",
        "allowed": True},
    "VERIFY_LIFT_DZ": {
        "source": "debug-seed measurement: rim top 1.1798 with the bowl's body "
                  "0.060 m deep (rim top minus the 1.120 cabinet top), so a lift "
                  "of 0.10 m puts a held bowl's base 0.028 m above the top of the "
                  "cabinet-rim detection band",
        "allowed": True},
    "GRID_M": {"source": "generic depth-clustering mechanics: footprint cell size",
               "allowed": True},
    "SEARCH_R": {"source": "generic: neighbourhood radius around the pack anchors",
                 "allowed": True},
    "BOWL_SEARCH_R": {
        "source": "debug-seed measurement (v2 logs, seeds 51..65 odd): perceived "
                  "bowl centres lie 0.063-0.089 m from PICK_ANCHOR, so the disc "
                  "is sized to contain centre + rim radius with margin",
        "allowed": True},
    "APPROACH_DZ": {"source": "generic controller mechanics: clearance kept above "
                              "a grasp/release point before descending",
                    "allowed": True},
}

PICK_ANCHOR = np.array([-0.0138, -0.2300])
PLACE_ANCHOR = np.array([0.0640, 0.2236])
DEMO_GRASP_Z = 1.1526
RIM_TOP_Z = 1.1798
RIM_TO_EEF_DZ = -0.0272
CABINET_TOP_Z = 1.120
PLATE_TOP_Z = 0.9201
RELEASE_DZ = 0.0469
PLACE_OFFSET_XY = np.array([-0.002, 0.027])
RIM_R_FALLBACK = 0.056
TRANSIT_Z = 1.28
HOLD_MARGIN = 0.0011
VERIFY_LIFT_DZ = 0.10
GRID_M = 0.012
SEARCH_R = 0.17
BOWL_SEARCH_R = 0.19
APPROACH_DZ = 0.10


# ---------------------------------------------------------------- perception

def _cloud(api, cam="cam_high"):
    f = api.capture(cam)
    d = np.asarray(f.depth, float)
    if d.ndim == 3:
        d = d[..., 0]
    H, W = d.shape
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    X = (uu - cx) * d / fx
    Y = (vv - cy) * d / fy
    P = np.stack([X, Y, d, np.ones_like(d)], axis=-1)
    B = P.reshape(-1, 4) @ T.T
    xyz = B[:, :3].reshape(H, W, 3)
    ok = np.isfinite(d) & (d > 0.02) & (d < 6.0)
    return {"xyz": xyz, "ok": ok, "uu": uu, "vv": vv}


def _cells(pts, cell=GRID_M):
    ij = np.floor(pts[:, :2] / cell).astype(np.int64)
    key = (ij[:, 0] + 5000) * 100000 + (ij[:, 1] + 5000)
    uk, inv = np.unique(key, return_inverse=True)
    zmax = np.full(uk.shape, -1e9)
    np.maximum.at(zmax, inv, pts[:, 2])
    cnt = np.bincount(inv, minlength=len(uk))
    ci = (uk // 100000) - 5000
    cj = (uk % 100000) - 5000
    return np.stack([ci, cj], 1), zmax, cnt


def _components(cij):
    have = {(int(a), int(b)): k for k, (a, b) in enumerate(cij)}
    seen, comps = set(), []
    for c in have:
        if c in seen:
            continue
        stack, comp = [c], []
        seen.add(c)
        while stack:
            p = stack.pop()
            comp.append(have[p])
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    q = (p[0] + da, p[1] + db)
                    if q in have and q not in seen:
                        seen.add(q)
                        stack.append(q)
        comps.append(np.array(comp, int))
    return comps


def _stats(cij, idx, cell=GRID_M):
    xs = (cij[idx, 0] + 0.5) * cell
    ys = (cij[idx, 1] + 0.5) * cell
    lo = np.array([xs.min() - cell / 2, ys.min() - cell / 2])
    hi = np.array([xs.max() + cell / 2, ys.max() + cell / 2])
    return (lo + hi) / 2, hi - lo, len(idx)


def _circle(xy):
    """Kasa algebraic circle fit -> (cx, cy, r); robust to a clipped arc."""
    A = np.column_stack([xy[:, 0], xy[:, 1], np.ones(len(xy))])
    b = (xy ** 2).sum(1)
    try:
        s, *_ = np.linalg.lstsq(A, b, rcond=None)
    except Exception:
        return None
    cx, cy = s[0] / 2, s[1] / 2
    rr = s[2] + cx * cx + cy * cy
    if rr <= 0:
        return None
    return np.array([cx, cy, np.sqrt(rr)])


def _band_blob(api, C, centre, radius, z_lo, z_hi, tag, min_cell=4):
    """Cluster the points inside an absolute z-band, return the blob nearest
    `centre`."""
    xyz, ok = C["xyz"], C["ok"]
    m = ok & (xyz[..., 2] > z_lo) & (xyz[..., 2] < z_hi)
    d = xyz[..., :2] - centre[None, None, :]
    m &= (d[..., 0] ** 2 + d[..., 1] ** 2) < radius ** 2
    pts = xyz[m]
    if len(pts) < 30:
        api.log("%s: only %d pts in band [%.3f,%.3f]" % (tag, len(pts), z_lo, z_hi))
        return None
    cij, zmax, cnt = _cells(pts)
    best = None
    for comp in _components(cij):
        mid, ext, n = _stats(cij, comp)
        if n < min_cell:
            continue
        dd = float(np.linalg.norm(mid - centre))
        npts = int(cnt[comp].sum())
        rec = {"mid": mid, "ext": ext, "ncell": n, "npts": npts, "d": dd,
               "ztop": float(zmax[comp].max())}
        if best is None or dd < best["d"]:
            best = rec
    if best is None:
        api.log("%s: no component >= %d cells" % (tag, min_cell))
        return None
    # sub-cell refinement: bbox midpoint + circle fit over the raw band points
    sel = pts[np.linalg.norm(pts[:, :2] - best["mid"], axis=1) < 0.09]
    lo, hi = sel[:, :2].min(0), sel[:, :2].max(0)
    best["mid_pt"] = (lo + hi) / 2
    best["ext_pt"] = hi - lo
    fit = _circle(sel[:, :2])
    best["fit"] = fit
    uu = C["uu"][m]
    api.log("%s: cellmid=(%.4f,%.4f) ext=(%.3f,%.3f) n=%d/%d ztop=%.4f d=%.4f | "
            "ptmid=(%.4f,%.4f) ptext=(%.3f,%.3f) | fit=%s | u=%d..%d" %
            (tag, best["mid"][0], best["mid"][1], best["ext"][0], best["ext"][1],
             best["ncell"], best["npts"], best["ztop"], best["d"],
             best["mid_pt"][0], best["mid_pt"][1], best["ext_pt"][0],
             best["ext_pt"][1],
             None if fit is None else np.round(fit, 4).tolist(),
             int(uu.min()), int(uu.max())))
    return best


# ---------------------------------------------------------------------- main

def _find_bowl(api, C, tag="BOWL"):
    """Bowl on the cabinet = the only structure in an absolute z-band under the
    (deterministic) rim top; the table bowl and the plate sit 20 cm lower."""
    bowl = _band_blob(api, C, PICK_ANCHOR, BOWL_SEARCH_R,
                      RIM_TOP_Z - 0.030, RIM_TOP_Z + 0.012, tag)
    if bowl is not None and bowl["d"] < 0.14:
        c = bowl["mid_pt"]
        rr = float(np.mean(bowl["ext_pt"]) / 2.0)
        if bowl["fit"] is not None and 0.040 < bowl["fit"][2] < 0.075 and \
                np.linalg.norm(bowl["fit"][:2] - c) < 0.02:
            c = 0.5 * (c + bowl["fit"][:2])
            rr = 0.5 * (rr + float(bowl["fit"][2]))
        rim_top = bowl["ztop"]
        src = "cam_high"
    else:
        c = PICK_ANCHOR - np.array([0.0, RIM_R_FALLBACK])
        rr = RIM_R_FALLBACK
        rim_top = RIM_TOP_Z
        src = "anchor-fallback"
    rr = float(np.clip(rr, 0.045, 0.070))
    rim_top = float(np.clip(rim_top, RIM_TOP_Z - 0.03, RIM_TOP_Z + 0.03))
    grasp_z = rim_top + RIM_TO_EEF_DZ
    api.log("%s(%s) centre=(%.4f,%.4f) r=%.4f rim_top=%.4f grasp_z=%.4f" %
            (tag, src, c[0], c[1], rr, rim_top, grasp_z))
    return c, rr, grasp_z


def _bowl_still_there(api, C, tag):
    """Free second opinion on the pick: is the cabinet-rim band still occupied?
    A held bowl has been lifted clear of the band, so occupancy means failure."""
    b = _band_blob(api, C, PICK_ANCHOR, BOWL_SEARCH_R, RIM_TOP_Z - 0.030,
                   RIM_TOP_Z + 0.012, tag, min_cell=8)
    there = bool(b is not None and b["d"] < 0.14)
    api.log("%s: bowl still on the cabinet = %s" % (tag, there))
    return there


def run(api):
    L = api.log
    L("v111 | %r" % api.instruction())

    api.grip(0.0)
    base_gap = float(api.gripper()["width_m"])
    api.grip(0.08)
    hold_thr = base_gap + HOLD_MARGIN
    L("empty-close gap=%.4f (hold test: gap > %.4f)" % (base_gap, hold_thr))
    L("tool_rotation home = %s" % np.round(api.tool_rotation(), 4).tolist())

    C = _cloud(api)

    # -- plate ---------------------------------------------------------------
    plate = _band_blob(api, C, PLACE_ANCHOR, SEARCH_R,
                       PLATE_TOP_Z - 0.012, PLATE_TOP_Z + 0.012, "PLATE",
                       min_cell=20)
    if plate is not None and plate["d"] < 0.12 and max(plate["ext"]) > 0.08:
        p = plate["mid_pt"]
        plate_top = plate["ztop"]
        psrc = "cam_high"
    else:
        p = PLACE_ANCHOR - PLACE_OFFSET_XY
        plate_top = PLATE_TOP_Z
        psrc = "anchor-fallback"
    plate_top = float(np.clip(plate_top, PLATE_TOP_Z - 0.02, PLATE_TOP_Z + 0.02))
    L("PLATE(%s) centre=(%.4f,%.4f) top=%.4f" % (psrc, p[0], p[1], plate_top))

    # -- rim pinch -----------------------------------------------------------
    # The demos' close point sits at the bowl centre + rim radius along +y
    # (demo grasp y -0.230 vs measured bowl centre y ~ -0.288 on seeds 57/61,
    # demo grasp x -0.014 vs centre x ~ +0.006): the jaw closing axis is world
    # y and the demos take the rim edge nearest the robot centre line.
    cands = [("+y", np.array([0.0, 1.0])),
             ("-y", np.array([0.0, -1.0])),
             ("+x", np.array([1.0, 0.0]))]
    rel = p + PLACE_OFFSET_XY
    rel_z = plate_top + RELEASE_DZ
    held, gap = False, base_gap

    for attempt in range(2):
        if attempt:
            L("=== ATTEMPT %d: the bowl was not in the fingers, re-perceiving"
              % (attempt + 1))
            api.grip(0.08)
            api.move([rel[0], rel[1], TRANSIT_Z], seconds=1.2)
            C = _cloud(api)
        c, rr, grasp_z = _find_bowl(api, C, "BOWL%d" % (attempt + 1))
        for name, u in cands:
            g = c + rr * u
            r1 = api.move([g[0], g[1], grasp_z + APPROACH_DZ], seconds=1.5)
            r2 = api.move([g[0], g[1], grasp_z], seconds=1.0)
            e_pre = api.eef()
            api.grip(0.0)
            gap = float(api.gripper()["width_m"])
            held = gap > hold_thr
            L("pinch %s at (%.4f,%.4f,%.4f) res=%.4f/%.4f pre=%s post=%s "
              "gap=%.4f held=%s" %
              (name, g[0], g[1], grasp_z, r1, r2, np.round(e_pre, 4).tolist(),
               np.round(api.eef(), 4).tolist(), gap, held))
            if held:
                api.move([g[0], g[1], grasp_z + VERIFY_LIFT_DZ], seconds=1.0)
                gap = float(api.gripper()["width_m"])
                still = _bowl_still_there(api, _cloud(api), "VERIFY-%s" % name)
                held = (gap > hold_thr) and not still
                L("after %.0fcm lift: gap=%.4f still_there=%s held=%s" %
                  (VERIFY_LIFT_DZ * 100, gap, still, held))
                if held:
                    break
            api.grip(0.08)
            # fair-v1.1.1: the `if api.done: return` exit that stood here is
            # excised.  It was dead on every debug seed, and the search is
            # bounded by len(cands) x range(2) without it.
        if not held:
            continue
        # -- carry ------------------------------------------------------------
        e = api.eef()
        api.move([e[0], e[1], TRANSIT_Z], seconds=1.2)
        L("carry to (%.4f,%.4f) rel_z=%.4f gap=%.4f" %
          (rel[0], rel[1], rel_z, api.gripper()["width_m"]))
        api.move([rel[0], rel[1], TRANSIT_Z], seconds=2.0)
        gap = float(api.gripper()["width_m"])
        # Only a BOTH-signals-agree verdict may re-open the fingers here: an
        # unnecessary re-attempt would drop the bowl from transit height.
        held = gap > hold_thr
        if not held:
            held = not _bowl_still_there(api, _cloud(api), "CARRY-CHECK")
        L("above plate eef=%s gap=%.4f held=%s" %
          (np.round(api.eef(), 4).tolist(), gap, held))
        # fair-v1.1.1: was `if held or api.done`; the sensor half alone decides.
        if held:
            break

    if not held:
        L("NO HOLD after %d attempts -- releasing over the plate anyway" %
          (attempt + 1))
        api.move([rel[0], rel[1], TRANSIT_Z], seconds=1.5)

    r = api.move([rel[0], rel[1], rel_z], seconds=1.5)
    L("descend res=%.4f eef=%s gap=%.4f" %
      (r, np.round(api.eef(), 4).tolist(), api.gripper()["width_m"]))
    api.grip(0.08)
    api.move([rel[0], rel[1], rel_z + 0.12], seconds=1.2)
    api.move([-0.18, -0.02, 1.24], seconds=1.6)   # get the arm out of the view

    # -- post-hoc: where did the bowl land? ----------------------------------
    C2 = _cloud(api)
    res = _band_blob(api, C2, p, 0.16, plate_top + 0.020, plate_top + 0.090,
                     "RESULT", min_cell=4)
    if res is not None:
        L("RESULT bowl centre offset from plate centre = (%.4f,%.4f) |d|=%.4f" %
          (res["mid_pt"][0] - p[0], res["mid_pt"][1] - p[1],
           float(np.linalg.norm(res["mid_pt"] - p))))
    _band_blob(api, C2, PICK_ANCHOR, BOWL_SEARCH_R, RIM_TOP_Z - 0.030,
               RIM_TOP_Z + 0.012, "POST-CABINET")
    # fair-v1.1.1: the trailing `L("done=%s" % api.done)` line is excised.
    return "v111 held=%s gap=%.4f" % (held, gap)
