"""c2 cell obj_salad_dressing_stock -- v1.1.1 (DONE-EXCISION of v5).

Change vs v5 (v5 = 15/15 formal selection): every read of the episode
termination flag is removed. Under fair-v1.1.1 that flag is an indirect
success signal (the benchmark ends the episode when its predicate fires), so
this program must be provably oracle-free: a fixed sequence driven only by its
own sensors.

Five reads existed in v5 (AST-located). Four were pure instrumentation and are
simply deleted:
  * the "after lift" log line in grasp_cycle,
  * the "released" log line,
  * the "end" log line,
  * the returned note string (now derived from the gripper effort instead).
One was LOAD-BEARING and gets the minimal sensor replacement:
  * v5 exited with "ok" as soon as the flag went true after the release, and
    otherwise ran a second-chance recovery cycle. The flag is the only thing
    that could tell a satisfied placement from an unsatisfied one, so the
    post-release half of that test is unobservable and is excised. The half
    that IS observable with the program's own sensors is kept unchanged: the
    mid-carry hold check (api.gripper()["effort"] >= HOLD_EFFORT, which reads
    3.0 while holding and ~0.05 on an empty close -- measured on the
    ctrl_dx30/dy30 arms). Recovery now runs iff the carry lost the bottle
    before the release; if the release was executed while still holding, the
    program lifts away and ends. On the debug seeds effort is 3.0 at the
    mid-carry check on 15/15 episodes and the release always executed, so the
    nominal motion sequence is bit-for-bit what v5 did (v5's recovery never
    fired either -- it was gated behind the same flag).
No constant changed; no constant was added.

v5 change vs v4 (v4 = 8/8 probe + 15/15 formal selection): DOCUMENTATION ONLY --
every gate that was an inline literal is hoisted into a named, PROVENANCE-
declared constant. No numeric value changes; re-selected to keep the frozen
md5 attached to its own 15-seed receipt.

v4 change vs v3: the basket is
accepted by a SIZE SIGNATURE (pixel count + XY extent + height above table)
measured on the debug seeds, and its bbox midpoint is then trusted outright.
v2/v3 additionally clamped the release to within 6 cm of the 3-demo mean; on a
seed that placed the basket further away that clamp would drag the release
toward the rim, while the signature (n>=5000 and ext 0.10-0.30 m vs n<=2783 and
ext<=0.081 m for every prop) cannot be matched by a prop. Nominal debug
behaviour is unchanged (the clamp was never active: perceived-vs-anchor
distance 0.024-0.030 m < 0.06 m).

Inherited:
  * v3 -- second-chance recovery: mid-carry hold check, and after a release
    that leaves the episode not-done, re-perceive and re-place ONLY if a
    prop-sized cluster sits OUTSIDE the basket footprint (never re-grab a
    bottle that is already in the basket). Never fires on debug seeds.
  * v2 -- perceived release over the basket BBOX MIDPOINT (the concave
    interior biases the point centroid +0.068 m in x); grasp height from the
    measured cap top (TOP_DZ below it, clamped to the demo z +-CLAMP_Z);
    effort-gated grasp retry ladder; anchor-displacement margin logging.
  * v1 -- demo-anchored identification: the K=3 pack demos close the gripper
    within 4 cm of each other; the nearest above-table footprint cluster to
    their mean is the salad dressing (0.014 m away vs 0.181 m for the nearest
    competitor). Footprint clustering on a 1.5 cm XY lattice, not image
    connectivity.
"""

import numpy as np

# --------------------------------------------------------------------------
PROVENANCE = {
    "GRASP_ANCHOR": {
        "source": "pack.json demos[0..2] keyframe with gripper_cmd +1 "
                  "(t=51/44/48): ee xy/z (0.0787,-0.1025,0.1238), "
                  "(0.0492,-0.1032,0.1356), (0.0375,-0.1186,0.1169); mean",
        "allowed": True},
    "GRASP_Z": {
        "source": "same three pack close-keyframes, mean of ee z "
                  "(0.1238,0.1356,0.1169)",
        "allowed": True},
    "RELEASE_ANCHOR": {
        "source": "pack.json demos[0..2] keyframe with gripper_cmd -1 after the "
                  "carry (t=128/107/115): ee (0.0183,0.2201,0.1709), "
                  "(-0.0451,0.2464,0.2045), (0.0047,0.2450,0.1884); mean",
        "allowed": True},
    "TRAVEL_Z": {
        "source": "pack.json ee_path max z over the three demos (0.3115, "
                  "0.3109, 0.3147) -> carry height",
        "allowed": True},
    "HOME_Z": {
        "source": "pack.json demos t=0 ee z (0.2565,0.2597,0.2591)",
        "allowed": True},
    "GRID_M": {
        "source": "generic clustering mechanic: XY footprint cell size chosen "
                  "coarser than depth noise, finer than prop spacing",
        "allowed": True},
    "TABLE_MARGIN": {
        "source": "generic depth mechanic: height band above the fitted table "
                  "plane that rejects plane noise",
        "allowed": True},
    "OPEN_W": {"source": "FairApi doc: >=0.025 opens", "allowed": True},
    "CLOSE_W": {"source": "FairApi doc: <0.025 closes", "allowed": True},
    "HOLD_EFFORT": {"source": "FairApi doc: effort 3.0 iff holding",
                    "allowed": True},
    "TOP_DZ": {
        "source": "debug seeds 51..65 odd (results fs_..._v1): target cluster "
                  "top measured 0.147 m while the pack grasp z is 0.1254 -> "
                  "grasp sits 0.0216 m below the perceived cap top",
        "allowed": True},
    "CLAMP_Z": {
        "source": "debug-seed v1 logs: perceived top identical (0.147) on all "
                  "8 probes; clamp keeps a depth dropout from moving the "
                  "grasp more than the demo z spread (0.1169..0.1356)",
        "allowed": True},
    "BASKET_SIGNATURE": {
        "source": "debug seeds 51..65 odd (results fs_..._v1/v2/v3 CL logs): "
                  "basket cluster n=11344..11581, ext 0.157..0.159 x 0.171, "
                  "height above table 0.140..0.141; every prop cluster has "
                  "n<=2783 and ext<=0.081",
        "allowed": True},
    "RELEASE_CLEAR": {
        "source": "debug-seed measurement: basket top 0.142 m, pack release z "
                  "0.1879 -> 0.045 m of rim clearance for the gripper",
        "allowed": True},
    "BASKET_MIN_N": {
        "source": "debug-seed v1 logs: basket cluster n=11557 vs <=2746 for "
                  "every prop cluster",
        "allowed": True},
    "RELEASE_Z": {
        "source": "pack.json release keyframes' ee z (0.1709,0.2045,0.1884), "
                  "mean; floor for the drop height",
        "allowed": True},
    "PROP_MIN_N": {
        "source": "debug-seed CL logs: prop clusters carry n=826..2783 while "
                  "depth speckle components stay far below 60 points",
        "allowed": True},
    "PROP_MAX_EXT": {
        "source": "debug-seed CL logs: every prop cluster has ext<=0.081 m; "
                  "0.20 m admits a prop but excludes basket/arm (>=0.135 m)",
        "allowed": True},
    "MIN_H": {
        "source": "debug-seed CL logs: props stand 0.138..0.146 m above the "
                  "table, the flat card lies at 0.018 m -> 0.02 m separates "
                  "standing props from table clutter",
        "allowed": True},
    "APPROACH_DZ": {
        "source": "generic controller mechanic: pre-grasp standoff above the "
                  "final descent, smaller than the object height (0.146 m)",
        "allowed": True},
    "BASKET_EXT_RANGE": {
        "source": "debug-seed CL logs: basket ext 0.157..0.159 x 0.171 m",
        "allowed": True},
    "BASKET_H_RANGE": {
        "source": "debug-seed CL logs: basket top 0.140..0.141 m above table",
        "allowed": True},
    "BASKET_SEARCH_R": {
        "source": "debug-seed CL logs: perceived basket bbox midpoint sits "
                  "0.024..0.030 m from the pack release anchor; 0.25 m admits "
                  "several times that jitter",
        "allowed": True},
    "WORKSPACE": {
        "source": "debug-seed cloud extent + pack ee_path range; crops the "
                  "cloud to the table area in front of the robot",
        "allowed": True},
    "CAP_BAND": {
        "source": "debug-seed CL logs: target cluster top 0.147 m; a 0.02 m "
                  "band under the top isolates the bottle's upper body "
                  "(logged as cap= in the CL lines)",
        "allowed": True},
    "RECOVER_MAX_R": {
        "source": "debug-seed geometry: basket half-extent 0.086 m; a dropped "
                  "bottle beyond 0.30 m of the release point is not the one we "
                  "released, so fall back to the demo anchor",
        "allowed": True},
    "MARGIN_PROBE": {
        "source": "instrumentation only (logs which cluster a displaced "
                  "anchor would select); no effect on motion",
        "allowed": True},
    "RETRY_DZ": {
        "source": "generic grasp mechanic: re-try offsets smaller than the "
                  "measured jaw closure width (0.0368 m on debug seeds)",
        "allowed": True},
}

GRASP_ANCHOR = np.array([0.05513, -0.10810])
GRASP_Z = 0.1254
RELEASE_ANCHOR = np.array([-0.00737, 0.23717])
RELEASE_Z = 0.1879
TRAVEL_Z = 0.3120
HOME_Z = 0.2584
GRID_M = 0.015
TABLE_MARGIN = 0.015
OPEN_W = 0.08
CLOSE_W = 0.0
HOLD_EFFORT = 2.5
TOP_DZ = 0.0216
CLAMP_Z = 0.012
BASKET_MIN_N = 5000
RETRY_DZ = (0.0, -0.012, 0.012)
PROP_MIN_N = 60
PROP_MAX_EXT = 0.20
MIN_H = 0.02
APPROACH_DZ = 0.05
BASKET_EXT_RANGE = (0.10, 0.30)
BASKET_H_RANGE = (0.08, 0.25)
BASKET_SEARCH_R = 0.25
RELEASE_CLEAR = 0.045
WORKSPACE = (-0.30, 0.40, -0.45, 0.45, 0.40)   # xmin, xmax, ymin, ymax, hmax
MARGIN_PROBE = 0.05
CAP_BAND = 0.02
RECOVER_MAX_R = 0.30


# --------------------------------------------------------------------------
def cloud(frame):
    """Vectorised deprojection of a FairFrame into base-frame xyz + rgb."""
    d = np.asarray(frame.depth, float)
    if d.ndim == 3:
        d = d[..., 0]
    h, w = d.shape
    K = np.asarray(frame.intrinsics, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 0.01)
    z = np.where(ok, d, 0.0)
    pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z,
                   np.ones_like(z)], axis=-1)
    pb = pc.reshape(-1, 4) @ np.asarray(frame.t_base_cam, float).T
    xyz = pb[:, :3].reshape(h, w, 3)
    return xyz, ok, np.asarray(frame.rgb)


def table_z(xyz, ok):
    z = xyz[..., 2][ok]
    z = z[(z > -0.5) & (z < 1.0)]
    hist, edges = np.histogram(z, bins=200, range=(float(z.min()),
                                                   float(z.max())))
    i = int(np.argmax(hist))
    lo, hi = edges[i], edges[i + 1]
    sel = z[(z >= lo - 0.01) & (z <= hi + 0.01)]
    return float(np.median(sel))


def footprint_clusters(xyz, ok, rgb, tz, api):
    """Group above-table points by ground footprint on a GRID_M XY lattice."""
    x, y, z = xyz[..., 0], xyz[..., 1], xyz[..., 2]
    x0, x1, y0, y1, hmax = WORKSPACE
    m = (ok & (z > tz + TABLE_MARGIN) & (z < tz + hmax)
         & (x > x0) & (x < x1) & (y > y0) & (y < y1))
    idx = np.argwhere(m)
    if idx.size == 0:
        return []
    px = x[m]
    py = y[m]
    pz = z[m]
    col = rgb[m].astype(float)
    cx_ = np.floor(px / GRID_M).astype(int)
    cy_ = np.floor(py / GRID_M).astype(int)
    cells = {}
    for i in range(len(px)):
        cells.setdefault((cx_[i], cy_[i]), []).append(i)
    # 8-connected flood fill over occupied cells
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
        ii = np.array([i for c in comp for i in cells[c]])
        if len(ii) < 25:
            continue
        cxs, cys, czs = px[ii], py[ii], pz[ii]
        top = float(np.percentile(czs, 98))
        capm = czs > top - CAP_BAND
        out.append(dict(
            n=int(len(ii)),
            cx=float(np.median(cxs)), cy=float(np.median(cys)),
            bx=float((cxs.min() + cxs.max()) / 2.0),
            by=float((cys.min() + cys.max()) / 2.0),
            capx=float(np.median(cxs[capm])), capy=float(np.median(cys[capm])),
            top=top, ext_x=float(cxs.max() - cxs.min()),
            ext_y=float(cys.max() - cys.min()),
            rgb=[int(v) for v in col[ii].mean(axis=0)],
            ncell=len(comp)))
    return out


# --------------------------------------------------------------------------
def perceive(api, tag=""):
    f = api.capture("cam_high")
    xyz, ok, rgb = cloud(f)
    tz = table_z(xyz, ok)
    cl = footprint_clusters(xyz, ok, rgb, tz, api)
    for c in sorted(cl, key=lambda c: -c["n"]):
        d = float(np.hypot(c["cx"] - GRASP_ANCHOR[0], c["cy"] - GRASP_ANCHOR[1]))
        api.log("%sCL n=%5d cell=%3d med=(%.3f,%.3f) bbox=(%.3f,%.3f) "
                "cap=(%.3f,%.3f) top=%.3f h=%.3f ext=(%.3f,%.3f) rgb=%s "
                "d_anchor=%.3f" %
                (tag, c["n"], c["ncell"], c["cx"], c["cy"], c["bx"], c["by"],
                 c["capx"], c["capy"], c["top"], c["top"] - tz,
                 c["ext_x"], c["ext_y"], c["rgb"], d))
    return tz, cl


def pick_target(cl, tz, anchor):
    cand = [c for c in cl if c["top"] - tz > MIN_H and c["n"] >= PROP_MIN_N
            and c["n"] < BASKET_MIN_N
            and c["ext_x"] < PROP_MAX_EXT and c["ext_y"] < PROP_MAX_EXT]
    if not cand:
        return None, []
    d = [float(np.hypot(c["cx"] - anchor[0], c["cy"] - anchor[1]))
         for c in cand]
    i = int(np.argmin(d))
    return cand[i], sorted(d)


def grasp_cycle(api, tx, ty, gz, tag=""):
    """Descend/close/lift with the effort-gated retry ladder. -> held bool."""
    for att, ddz in enumerate(RETRY_DZ):
        api.move([tx, ty, TRAVEL_Z], seconds=2.5 if att == 0 else 1.5)
        api.move([tx, ty, gz + ddz + APPROACH_DZ], seconds=1.5)
        r = api.move([tx, ty, gz + ddz], seconds=1.5)
        api.grip(CLOSE_W)
        api.settle(0.4)
        g = api.gripper()
        api.log("%satt%d z=%.4f eef=%s resid=%.4f grip=%s" %
                (tag, att, gz + ddz, np.round(api.eef(), 4).tolist(), r, g))
        api.move([tx, ty, TRAVEL_Z], seconds=2.0)
        g = api.gripper()
        api.log("%satt%d after lift: grip=%s eef=%s" %
                (tag, att, g, np.round(api.eef(), 4).tolist()))
        if g["effort"] >= HOLD_EFFORT:
            return True
        api.log("%satt%d NO HOLD -- retrying" % (tag, att))
        api.grip(OPEN_W)
        tz2, cl2 = perceive(api, tag="re:")
        b2, _ = pick_target(cl2, tz2, GRASP_ANCHOR)
        if b2 is not None:
            tx, ty = b2["cx"], b2["cy"]
    return False


def run(api):
    api.log("instruction=%r cameras=%s" % (api.instruction(), api.cameras))
    api.log("eef0=%s grip0=%s" % (np.round(api.eef(), 4).tolist(),
                                  api.gripper()))
    api.grip(OPEN_W)

    tz, cl = perceive(api)
    api.log("table_z=%.4f grasp_z_demo=%.4f (dz=%.4f)" %
            (tz, GRASP_Z, GRASP_Z - tz))

    best, dists = pick_target(cl, tz, GRASP_ANCHOR)
    if best is None:
        api.log("NO CANDIDATE -- falling back to raw demo anchor")
        tx, ty = float(GRASP_ANCHOR[0]), float(GRASP_ANCHOR[1])
        gz = GRASP_Z
    else:
        tx, ty = best["cx"], best["cy"]
        gz = float(np.clip(best["top"] - TOP_DZ,
                           GRASP_Z - CLAMP_Z, GRASP_Z + CLAMP_Z))
        api.log("PICK med=(%.3f,%.3f) top=%.3f gz=%.4f rgb=%s n=%d | "
                "dists=%s" % (tx, ty, best["top"], gz, best["rgb"],
                              best["n"], [round(v, 3) for v in dists[:4]]))

    # margin instrumentation -- logging only, no behaviour change
    _mp = MARGIN_PROBE
    for dx, dy in ((_mp, 0.0), (-_mp, 0.0), (0.0, _mp), (0.0, -_mp)):
        alt, _ = pick_target(cl, tz, GRASP_ANCHOR + np.array([dx, dy]))
        api.log("MARGIN anchor+(%+0.2f,%+0.2f) -> (%.3f,%.3f) same=%s" %
                (dx, dy, -9 if alt is None else alt["cx"],
                 -9 if alt is None else alt["cy"],
                 alt is not None and best is not None
                 and abs(alt["cx"] - tx) < 1e-9 and abs(alt["cy"] - ty) < 1e-9))

    # ---- perceive the basket ONCE, while it is still unoccluded ----------
    bx, by, bz = RELEASE_ANCHOR[0], RELEASE_ANCHOR[1], RELEASE_Z
    bbox = None
    # Basket SIZE SIGNATURE measured on the debug seeds (n 11344-11581, ext
    # 0.157-0.159 x 0.171, height 0.140-0.141 above table); no prop comes
    # close (n<=2783, ext<=0.081), so a matching cluster IS the basket and its
    # bbox midpoint can be trusted outright.
    bask = [c for c in cl if c["n"] >= BASKET_MIN_N
            and BASKET_EXT_RANGE[0] <= c["ext_x"] <= BASKET_EXT_RANGE[1]
            and BASKET_EXT_RANGE[0] <= c["ext_y"] <= BASKET_EXT_RANGE[1]
            and BASKET_H_RANGE[0] <= c["top"] - tz <= BASKET_H_RANGE[1]
            and np.hypot(c["bx"] - RELEASE_ANCHOR[0],
                         c["by"] - RELEASE_ANCHOR[1]) < BASKET_SEARCH_R]
    if bask:
        b = max(bask, key=lambda c: c["n"])
        bx, by = float(b["bx"]), float(b["by"])
        bz = float(max(RELEASE_Z, b["top"] + RELEASE_CLEAR))
        bbox = (b["bx"], b["by"], max(b["ext_x"], b["ext_y"]) / 2.0)
        api.log("BASKET bbox=(%.3f,%.3f) med=(%.3f,%.3f) top=%.3f n=%d "
                "ext=(%.3f,%.3f) -> release=(%.3f,%.3f,%.3f)" %
                (b["bx"], b["by"], b["cx"], b["cy"], b["top"], b["n"],
                 b["ext_x"], b["ext_y"], bx, by, bz))
    else:
        api.log("BASKET not perceived -- using pack release anchor")

    # ---- pick / carry / place, with one recovery cycle -------------------
    for cyc in range(2):
        tag = "" if cyc == 0 else "sc%d:" % cyc
        if not grasp_cycle(api, tx, ty, gz, tag):
            api.log("%sGRASP FAILED after %d attempts" % (tag, len(RETRY_DZ)))
            return "no-hold"

        api.move([bx, by, TRAVEL_Z], seconds=3.0)
        g = api.gripper()
        api.log("%sover basket: eef=%s grip=%s" %
                (tag, np.round(api.eef(), 4).tolist(), g))
        if g["effort"] < HOLD_EFFORT:
            api.log("%sHOLD LOST MID-CARRY" % tag)
        else:
            api.move([bx, by, bz], seconds=1.5)
            api.log("%sat release: eef=%s grip=%s" %
                    (tag, np.round(api.eef(), 4).tolist(), api.gripper()))
            api.grip(OPEN_W)
            api.settle(0.6)
            api.log("%sreleased: eef=%s" %
                    (tag, np.round(api.eef(), 4).tolist()))
            api.move([bx, by, TRAVEL_Z], seconds=1.5)
            api.settle(0.5)
            # v1.1.1: the sequence is complete. v5 asked the termination flag
            # whether the placement had satisfied the predicate and only then
            # exited; that question is unanswerable with the program's own
            # sensors, so the exit is now unconditional on a release that was
            # executed while the effort sensor still read a hold.
            api.log("%sreleased over the basket -- sequence complete" % tag)
            return "released"
        if cyc:
            break

        # -- recovery (unchanged body): the carry lost the bottle before the
        # -- release. Re-place ONLY a prop-sized cluster OUTSIDE the basket.
        api.log("HOLD LOST -- recovery probe")
        tz3, cl3 = perceive(api, tag="sc:")
        loose = [c for c in cl3
                 if PROP_MIN_N <= c["n"] < BASKET_MIN_N
                 and c["top"] - tz3 > MIN_H
                 and c["ext_x"] < PROP_MAX_EXT and c["ext_y"] < PROP_MAX_EXT
                 and (bbox is None
                      or np.hypot(c["cx"] - bbox[0], c["cy"] - bbox[1])
                      > bbox[2] + MIN_H)]
        if not loose:
            api.log("no loose prop outside the basket -- settling only")
            api.settle(1.0)
            break
        # the bottle is the loose prop closest to where we released it
        t2 = min(loose, key=lambda c: np.hypot(c["cx"] - bx, c["cy"] - by))
        if np.hypot(t2["cx"] - bx, t2["cy"] - by) > RECOVER_MAX_R:
            t2, _ = pick_target(cl3, tz3, GRASP_ANCHOR)
            if t2 is None:
                break
        tx, ty = t2["cx"], t2["cy"]
        gz = float(np.clip(t2["top"] - TOP_DZ,
                           GRASP_Z - CLAMP_Z, GRASP_Z + CLAMP_Z))
        api.log("RECOVER target=(%.3f,%.3f) top=%.3f gz=%.4f n=%d" %
                (tx, ty, t2["top"], gz, t2["n"]))

    g = api.gripper()
    api.log("end: no release executed, final grip=%s eef=%s" %
            (g, np.round(api.eef(), 4).tolist()))
    return "no-release"
