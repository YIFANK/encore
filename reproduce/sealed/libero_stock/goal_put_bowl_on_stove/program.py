"""c2 / goal_put_bowl_on_stove_stock -- v1.1.1 (v6 with every termination-flag read excised)

v1.1.1 is a MECHANICAL done-excision pass over the frozen v6; no constant, no
threshold and no nominal motion changed.  Under fair-v1.1.1 any read of the episode-termination flag
is an indirect success signal (LIBERO terminates the episode when the success
predicate fires), so all 9 reads are gone:
  * 4 were cosmetic (three log strings + the return string) -> the `done=%s`
    field is dropped; the release logs now print the gripper state instead.
  * 3 were redundant termination guards inside already-bounded loops (the
    cycle-top guard, and the two `break`s taken after a failed grasp).  Both
    loops are finite on their own -- the ladder is <=4 attempts, the cycle loop
    is <=3 -- so deleting the guards changes only how many no-op steps a dead
    episode burns.
  * 2 were the load-bearing success-conditioned exits taken right after the
    release (the nominal one, and its twin on the still-holding retry path).
    Both are replaced by the RE-PERCEPTION check v6 already carried in-line
    below the nominal exit and never reached: park, re-image, and require a
    bowl-signature rim ring within LAND_TOL of the burner centre.  That check
    is pure RGB-D, needs no runtime verdict, and its constants are unchanged.

Receipts
  v1  0/8    perception absorbed the robot arm.
  v2  8/8 probe, 14/15 selection.  Seed 54 failed because the bowl's footprint
             cluster FUSED with the abutting wine bottle and the rim band was
             read off the bottle top (h=0.133 m, radius 3 mm).
  v3  9/9 probe (51,53,54,55,57,59,61,63,65) -- the single change was to
             cluster only cells whose MAX height lies in the bowl-rim band, so
             the 0.13 m bottle drops out.  Detection margin on seed 54:
             0.044 m to the bowl vs 0.346 m to the next component.
  v4  8/9 probe.  v4 added a bounded self-verified retry and regressed seed 55:
             the mid-carry "did I drop it" test used the api's `effort` flag,
             which is a 5 mm finger-gap threshold, and this bowl is carried at
             a 4.8-8.5 mm gap.  A bowl that had merely settled 2 mm in the
             fingers was declared dropped and the gripper was opened in mid-air.

v5 = v4 with the hold test replaced by the measured gap itself.  Over 39
logged closes the two states are cleanly bimodal -- 0.0066-0.0085 m holding
vs 0.0010-0.0012 m closed on air -- so a 0.0030 m threshold sits ~2.5x clear
of both, where the built-in flag sits inside the holding distribution.
  v5  9/9 probe -- the retry never fired; it is a dormant safety net.

v6 = v5 with three margin changes, none of which alters the nominal path:
 (a) the bowl search radius grows 0.13 -> 0.20 m.  The rim signature (radius
     0.040-0.068 m at 0.038-0.070 m over the table) is matched by nothing else
     in this scene -- the nearest non-bowl component that clears it is 0.35 m
     away -- so the wider radius buys tolerance to layout jitter beyond the
     debug band without opening the door to a wrong target.
 (b) the descent depth tracks the measured rim instead of being pinned to the
     demo value, clamped to +/-8 mm of it, so a table-plane misread cannot
     drive the fingers into or over the rim.
 (c) the stove is MEASURED and logged but NOT yet acted on -- v6 exists partly
     to find out whether the fixture moves enough to be worth tracking.
"""
import json

import numpy as np

PROVENANCE = {
    "GRASP_ANCHOR_XY": {
        "source": "pack.json demos[0..2].keyframes[1].ee[:2] (the keyframe where "
                  "gripper_cmd flips to +1 = close); mean = (-0.1126, 0.0471)",
        "allowed": True},
    "GRASP_Z": {
        "source": "pack.json demos[0..2].keyframes[1].ee[2]; mean 0.92907 m",
        "allowed": True},
    "PLACE_ANCHOR_XY": {
        "source": "pack.json demos[0..2].keyframes[2].ee[:2] (the keyframe where "
                  "gripper_cmd flips to -1 = release); mean = (-0.2566, 0.2556)",
        "allowed": True},
    "PLACE_Z": {
        "source": "pack.json demos[0..2].keyframes[2].ee[2]; mean 0.95523 m",
        "allowed": True},
    "RIM_R": {
        "source": "debug-seed measurement: rim half-extent of the bowl on seeds "
                  "51,53,54,55,57,59,61,63,65 (v2/v3 logs) = 0.0503-0.0538 m",
        "allowed": True},
    "RIM_H_LO/RIM_H_HI": {
        "source": "debug-seed measurement: the bowl rim stands 0.0508 m over the "
                  "measured table plane on every probe seed; band widened to "
                  "0.035-0.075 m",
        "allowed": True},
    "TALL_H": {
        "source": "debug-seed measurement: the abutting bottle tops out at 0.133 m "
                  "and the arm at 0.27 m over the table",
        "allowed": True},
    "CELL_M": {"source": "generic perception hyper-parameter: 1 cm ground-footprint grid",
               "allowed": True},
    "RIM_BAND_M": {"source": "generic perception hyper-parameter: depth of the top band "
                             "taken as the rim ring of a concave vessel", "allowed": True},
    "OPEN_W": {"source": "generic gripper mechanics: api.grip(>0.025) opens", "allowed": True},
    "CLOSE_W": {"source": "generic gripper mechanics: api.grip(<0.025) closes", "allowed": True},
    "SAFE_Z": {"source": "pack.json demos[*].keyframes[0].ee[2] ~1.17 m start height; "
                         "carry height chosen below it", "allowed": True},
    "PARK_XY": {"source": "debug-seed observation: the episode's own start EEF xy "
                          "(-0.2085, 0.0), the viewpoint under which detection was "
                          "validated", "allowed": True},
    "SEARCH_R": {"source": "generic robustness hedge: a bowl candidate is trusted only "
                           "within this radius of the pack-demo grasp anchor; sized "
                           "against the debug-seed margin (bowl 0.038-0.062 m from the "
                           "anchor, nearest signature-passing competitor 0.35 m)",
                 "allowed": True},
    "RIM_TO_GRASP": {"source": "debug-seed measurement: on every probe seed the rim top "
                               "sits 0.0508 m over the table and the pack's close "
                               "keyframe is 0.0228 m below that rim top",
                     "allowed": True},
    "GRASP_Z_SLACK": {"source": "generic robustness hedge: the measured-rim descent depth "
                                "is clamped to this much either side of the pack value",
                      "allowed": True},
    "STOVE_H_LO/STOVE_H_HI": {"source": "debug-seed measurement (v2 ASCII height maps): "
                                        "the stove platform stands 0.02 m over the table "
                                        "with a 0.03 m burner disc on it",
                              "allowed": True},
    "LAND_TOL": {"source": "generic robustness hedge: post-release the rim ring must be "
                           "found within this radius of the implied burner centre",
                 "allowed": True},
    "HOLD_W": {"source": "debug-seed measurement: 39 logged closes on seeds 51-65 "
                         "(v2/v3/v4 logs) separate into 0.0066-0.0085 m holding vs "
                         "0.0010-0.0012 m closed on air; 0.0030 m splits them",
               "allowed": True},
}

GRASP_ANCHOR_XY = np.array([-0.11260, 0.04710])
GRASP_Z = 0.92907
PLACE_ANCHOR_XY = np.array([-0.25660, 0.25560])
PLACE_Z = 0.95523
RIM_R = 0.053
RIM_H_LO, RIM_H_HI = 0.035, 0.075
TALL_H = 0.085
CELL_M = 0.010
RIM_BAND_M = 0.012
OPEN_W = 0.08
CLOSE_W = 0.0
SAFE_Z = 1.08
PARK_XY = np.array([-0.2085, 0.0])
SEARCH_R = 0.20
LAND_TOL = 0.08
HOLD_W = 0.0030
RIM_TO_GRASP = 0.0228
GRASP_Z_SLACK = 0.008
STOVE_H_LO, STOVE_H_HI = 0.012, 0.048
XLIM = (-0.46, 0.20)
YLIM = (-0.34, 0.44)
# Burner centre implied by the demos: the release EEF holds the rim, so the
# bowl centre lands one rim radius on the -y side of the release anchor.
BURNER_XY = PLACE_ANCHOR_XY - np.array([0.0, RIM_R])


# --------------------------------------------------------------------------
def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    d = np.asarray(f.depth, dtype=float)
    K = np.asarray(f.intrinsics, dtype=float)
    T = np.asarray(f.t_base_cam, dtype=float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    pc = np.stack([x, y, d, np.ones_like(d)], axis=-1) @ T.T
    ok = np.isfinite(d) & (d > 0.05) & (d < 5.0)
    return f, pc[..., :3], ok


def table_height(P, ok):
    m = ok & (P[..., 2] > 0.6) & (P[..., 2] < 1.2) \
        & (np.abs(P[..., 0]) < 0.7) & (np.abs(P[..., 1]) < 0.7)
    zs = P[..., 2][m]
    if zs.size < 500:
        return 0.90
    hist, edges = np.histogram(zs, bins=np.arange(0.6, 1.2, 0.003))
    i = int(np.argmax(hist))
    sel = zs[(zs > edges[i] - 0.004) & (zs < edges[i + 1] + 0.004)]
    return float(np.median(sel))


def cellmap(P, ok, tz):
    """Per-cell MAX height: a fused blob is one component in the image but not
    one component in height, so this is what keeps a tall neighbour out."""
    m = (ok & (P[..., 2] > tz + 0.004) & (P[..., 2] < tz + 0.60)
         & (P[..., 0] > XLIM[0]) & (P[..., 0] < XLIM[1])
         & (P[..., 1] > YLIM[0]) & (P[..., 1] < YLIM[1]))
    pts = P[m]
    hs = pts[:, 2] - tz
    ij = np.floor(pts[:, :2] / CELL_M).astype(int)
    cells = {}
    for n in range(pts.shape[0]):
        k = (int(ij[n, 0]), int(ij[n, 1]))
        c = cells.get(k)
        if c is None:
            c = cells[k] = [0, -9.0, []]
        c[0] += 1
        if hs[n] > c[1]:
            c[1] = float(hs[n])
        c[2].append(n)
    return pts, cells


def components(keys):
    ks = set(keys)
    parent = {k: k for k in ks}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for k in ks:
        for du in (-1, 0, 1):
            for dv in (-1, 0, 1):
                nk = (k[0] + du, k[1] + dv)
                if nk in ks:
                    ra, rb = find(k), find(nk)
                    if ra != rb:
                        parent[ra] = rb
    groups = {}
    for k in ks:
        groups.setdefault(find(k), []).append(k)
    return list(groups.values())


def rim_components(pts, cells, hlo=RIM_H_LO, hhi=RIM_H_HI, min_cells=12):
    """Bowl-rim-height components with their fitted ring centre and radius.

    hhi is deliberately tight (0.075 m) while grasping: the abutting bottle
    throws a halo of partial-depth border cells at intermediate heights, and a
    wider band lets that halo bridge bottle and bowl back into one component --
    exactly the v2 seed-54 failure.  Verification, which must see the bowl
    standing on the 0.03 m stove platform, passes a wider hhi instead.
    """
    band = [k for k, v in cells.items()
            if v[0] >= 2 and hlo <= v[1] <= hhi]
    out = []
    for g in components(band):
        if len(g) < min_cells:
            continue
        idx = []
        for k in g:
            idx.extend(cells[k][2])
        q = pts[np.asarray(idx, dtype=int)]
        top = float(np.percentile(q[:, 2], 99))
        ring = q[q[:, 2] > top - RIM_BAND_M]
        if ring.shape[0] < 20:
            ring = q
        cx = float((np.percentile(ring[:, 0], 2) + np.percentile(ring[:, 0], 98)) / 2)
        cy = float((np.percentile(ring[:, 1], 2) + np.percentile(ring[:, 1], 98)) / 2)
        rx = float(np.percentile(ring[:, 0], 98) - np.percentile(ring[:, 0], 2)) / 2
        ry = float(np.percentile(ring[:, 1], 98) - np.percentile(ring[:, 1], 2)) / 2
        out.append({"c": np.array([cx, cy]), "rx": rx, "ry": ry,
                    "ncell": len(g), "nring": int(ring.shape[0]), "top": top})
    return out


def is_bowl(c, tz, hhi=0.070):
    return (0.040 <= c["ry"] <= 0.068 and 0.038 <= (c["top"] - tz) <= hhi)


def perceive(api, log, tag, hhi=RIM_H_HI):
    f, P, ok = cloud(api, "cam_high")
    tz = table_height(P, ok)
    pts, cells = cellmap(P, ok, tz)
    comps = rim_components(pts, cells, hhi=hhi)
    log("%s table_z=%.4f comps=%s" % (tag, tz, json.dumps(
        [{"c": np.round(c["c"], 4).tolist(), "rx": round(c["rx"], 4),
          "ry": round(c["ry"], 4), "h": round(c["top"] - tz, 4),
          "ncell": c["ncell"], "bowl": bool(is_bowl(c, tz))} for c in comps])))
    if tag.startswith("PERCEIVE[0]"):
        measure_stove(pts, cells, log)
    return tz, comps


def pick_bowl(comps, tz, anchor, radius, log):
    best, bd = None, 9.0
    for c in comps:
        if not is_bowl(c, tz):
            continue
        d = float(np.linalg.norm(c["c"] - anchor))
        if d < bd:
            best, bd = c, d
    if best is None or bd > radius:
        log("  no bowl-signature component within %.3f (best d=%.3f)" % (radius, bd))
        return None, bd
    return best, bd


def measure_stove(pts, cells, log):
    """Where is the fixture, by its own footprint? Logged for calibration."""
    sel = [k for k, v in cells.items()
           if v[0] >= 2 and STOVE_H_LO <= v[1] <= STOVE_H_HI
           and abs((k[0] + 0.5) * CELL_M - BURNER_XY[0]) < 0.20
           and abs((k[1] + 0.5) * CELL_M - BURNER_XY[1]) < 0.20]
    out = []
    for g in components(sel):
        if len(g) < 30:
            continue
        cc = np.array([[(k[0] + 0.5) * CELL_M, (k[1] + 0.5) * CELL_M] for k in g])
        hi = np.array([cells[k][1] for k in g])
        mid = [(cc[:, 0].min() + cc[:, 0].max()) / 2, (cc[:, 1].min() + cc[:, 1].max()) / 2]
        ext = [cc[:, 0].max() - cc[:, 0].min() + CELL_M,
               cc[:, 1].max() - cc[:, 1].min() + CELL_M]
        disc = cc[hi >= 0.025]
        dmid = ([(disc[:, 0].min() + disc[:, 0].max()) / 2,
                 (disc[:, 1].min() + disc[:, 1].max()) / 2] if disc.shape[0] >= 20 else None)
        out.append({"ncell": len(g), "mid": [round(v, 4) for v in mid],
                    "ext": [round(v, 3) for v in ext],
                    "disc_mid": None if dmid is None else [round(v, 4) for v in dmid],
                    "ndisc": int(disc.shape[0])})
    out.sort(key=lambda c: -c["ncell"])
    log("STOVE measured (burner prior=%s): %s"
        % (np.round(BURNER_XY, 4).tolist(), json.dumps(out[:3])))
    return out


def holding(api):
    """Is something between the fingers?

    Measured, not inferred: the api's own effort flag trips at a 5 mm finger
    gap, and this bowl is held at 4.8-8.5 mm, so the flag flickers on a 2 mm
    settle.  The gap itself is bimodal with a wide empty band.
    """
    g = api.gripper()
    return bool(g["width_m"] > HOLD_W), g


def park(api):
    e = api.eef()
    api.move([e[0], e[1], SAFE_Z], seconds=1.0)
    api.move([PARK_XY[0], PARK_XY[1], SAFE_Z + 0.06], seconds=1.4)


def landed_on_burner(api, log, tag):
    """Did the release leave a bowl on the burner?  Answered by RE-PERCEPTION.

    This is v6's own post-release verification, unchanged, lifted out of run()
    so both release sites can call it -- in v6 both sites exited on the termination flag
    first, so this block was written but never reached.  On the stove the bowl
    rim stands one platform height higher than it does on the table, so the
    verification band has to reach past the grasp-time ceiling.
    """
    tz2, comps2 = perceive(api, log, tag, hhi=0.115)
    near = [c for c in comps2
            if is_bowl(c, tz2, hhi=0.115)
            and float(np.linalg.norm(c["c"] - BURNER_XY)) <= LAND_TOL]
    log("  verify: %d bowl-signature comps within %.2f of burner" % (len(near), LAND_TOL))
    return bool(near)


# --------------------------------------------------------------------------
def run(api):
    log = api.log
    log("instruction=%r" % api.instruction())
    log("eef0=%s tool_rot0=%s" % (np.round(api.eef(), 4).tolist(),
                                  np.round(np.asarray(api.tool_rotation()), 3).tolist()))
    placed = False
    sgn = +1.0
    last_r = RIM_R
    for cycle in range(3):
        carried, g0 = holding(api)
        if carried and cycle > 0:
            log("cycle %d: still holding (%s) -- skip re-grasp, go place" % (cycle, g0))
            place = BURNER_XY + np.array([0.0, sgn * last_r])
            e = api.eef()
            api.move([e[0], e[1], SAFE_Z], seconds=1.0)
            api.move([place[0], place[1], SAFE_Z], seconds=1.6)
            rr = api.move([place[0], place[1], PLACE_Z], seconds=1.0)
            log("  set down res=%.4f eef=%s" % (rr, np.round(api.eef(), 4).tolist()))
            api.grip(OPEN_W)
            api.settle(1.0)
            api.move([place[0], place[1], SAFE_Z], seconds=1.0)
            api.settle(0.5)
            log("  released, gripper=%s" % api.gripper())
            park(api)
            if landed_on_burner(api, log, "VERIFY[%d]" % cycle):
                placed = True
                break
            continue
        tz, comps = perceive(api, log, "PERCEIVE[%d]" % cycle)
        # after the first cycle the bowl may be anywhere it was dropped
        radius = SEARCH_R if cycle == 0 else 0.26
        cand, bd = pick_bowl(comps, tz, GRASP_ANCHOR_XY, radius, log)
        if cand is None:
            centre, r, gz = GRASP_ANCHOR_XY - np.array([0.0, RIM_R]), RIM_R, GRASP_Z
            log("cycle %d: pack-anchor fallback centre=%s" % (cycle, np.round(centre, 4).tolist()))
        else:
            centre = cand["c"]
            r = float(np.clip(cand["ry"], 0.045, 0.062))
            gz = float(np.clip(cand["top"] - RIM_TO_GRASP,
                               GRASP_Z - GRASP_Z_SLACK, GRASP_Z + GRASP_Z_SLACK))
            log("cycle %d: BOWL d=%.4f centre=%s r=%.4f ncell=%d rim_top=%.4f gz=%.4f"
                % (cycle, bd, np.round(centre, 4).tolist(), r, cand["ncell"],
                   cand["top"], gz))
        last_r = r

        api.grip(OPEN_W)
        held = False
        ladder = (+r, -r, +r * 0.88, -r * 0.88) if cycle == 0 else (+r, -r)
        for dy in ladder:
            tgt = centre + np.array([0.0, dy])
            api.move([tgt[0], tgt[1], gz + 0.10], seconds=1.2)
            rr = api.move([tgt[0], tgt[1], gz], seconds=1.0)
            api.grip(CLOSE_W)
            api.move([tgt[0], tgt[1], gz + 0.13], seconds=1.0)
            hold, g = holding(api)
            log("  GRASP dy=%+.4f tgt=%s descend_res=%.4f -> hold=%s %s"
                % (dy, np.round(tgt, 4).tolist(), rr, hold, g))
            if hold:
                held, sgn = True, (1.0 if dy > 0 else -1.0)
                break
            api.grip(OPEN_W)
        log("cycle %d: HELD=%s sgn=%+0.0f" % (cycle, held, sgn))
        if not held:
            park(api)
            continue

        place = BURNER_XY + np.array([0.0, sgn * r])
        e = api.eef()
        api.move([e[0], e[1], SAFE_Z], seconds=1.0)
        api.move([place[0], place[1], SAFE_Z], seconds=1.6)
        hold, g = holding(api)
        log("  over stove eef=%s hold=%s %s" % (np.round(api.eef(), 4).tolist(), hold, g))
        if not hold:
            log("  DROPPED in transit -- re-perceive")
            park(api)
            continue
        rr = api.move([place[0], place[1], PLACE_Z], seconds=1.0)
        log("  set down res=%.4f eef=%s %s"
            % (rr, np.round(api.eef(), 4).tolist(), api.gripper()))
        api.grip(OPEN_W)
        api.settle(1.0)
        api.move([place[0], place[1], SAFE_Z], seconds=1.0)
        api.settle(0.5)
        log("  released, gripper=%s" % api.gripper())

        park(api)
        if landed_on_burner(api, log, "VERIFY[%d]" % cycle):
            placed = True
            break
    return "v111 placed=%s" % placed
