"""c2 cell spa_bowl_cookie_box_stock -- v1.1.1 (v7 with the episode-termination
attribute read excised; no other change).

v1.1.1 pass: v7 contained exactly one read of the episode-termination flag,
a guard inside the grasp-retry loop that returned early when the episode had
already terminated while the bowl was still on the table. It was dead on
every episode ever run by this cell (fs_v5/v6/v7, sel_v5, sel_v7: 56/56
episodes reached the loop only via the still_on_table=None break or via the
normal retry path; no log ever emitted its return note), so it is excised
outright with no sensor replacement. All constants, PROVENANCE entries and
control flow are otherwise byte-identical to v7.

--- v7 header ---

v6 receipt: 9/9 on debug seeds 51,53,54,55,57,59,61,63,65 (fs_..._v6), 147-460
sim steps of 1000. Two of the nine needed the recovery loop, and both first
attempts failed the same way: the descent to PINCH_Z stopped ~8 mm high AND
slid 10-13 mm in -x (ep61: commanded (0.1425,-0.0274,0.940), arrived
(0.1296,-0.0273,0.9480)), then shoved the bowl. That is the INNER finger
landing on the bowl's inner floor -- measured at EEF z 0.932-0.937 in the v3
logs, i.e. fingertips 0.920-0.925, only 3-8 mm below v6's fingertip height of
0.928. v7 raises the pinch by 5 mm (fingertips 0.933) and lets the per-episode
wall profile supply the correspondingly larger radius. Single-variable change.

--- v6 header ---

v5 receipt: 8/8 probe (odd debug seeds) and 14/15 on the full debug band
(sel_..._v5). The single failure, ep54, was NOT a perception or grasp failure:
  g0 missed and shoved the bowl 3 cm; the re-perception check found it at its
  new footprint and re-grounded; g1 pinched it (gap 0.0058) and the check
  confirmed the table was clear -- and then the episode hit its 1000-step
  horizon holding the bowl 12 cm above the plate, with the release descent
  never executed.
Step accounting from that log: a `move` that cannot reach POS_TOL burns its
whole cap (2*60*seconds), and v5 spent ~380 of its 1000 steps on refine moves
that improved the residual by under a millimetre, plus a redundant transit to
a pose the arm was already standing in.
v6 changes only the motion budget: refines are short (0.4 s), judged on the
axis that matters (the jaw axis y) and abandoned when they stop improving;
transit moves do not refine; the place descends from the verification pose
instead of re-flying to it; at most two grasp attempts.

--- v5 header ---

v4 receipt: 8/8 on debug seeds 51..65 odd (fs_..._v4) -- but ep57 succeeded by
accident and the logs expose two defects:
  * the finger-gap hold test is not decidable. Measured post-lift gaps that
    were CARRYING the bowl: 0.0038-0.0060; gaps that closed on air: 0.0010-
    0.0037. ep57 held the bowl at 0.0038, the 0.0040 gate called it a miss,
    and the "regrasp" opened the jaws 12 cm over the plate -- which dropped
    the bowl onto the plate and scored. v5 stops gating on the gap at all.
  * the regrasp re-grounded with the demo anchor while the ARM was in frame
    and locked onto the cabinet (r_rim 0.088, ztop 1.13). v5 re-grounds only
    among bowl-shaped clusters within 12 cm of the original footprint.
The surviving hold test is re-perception from over the plate, where the arm no
longer covers the bowl's old footprint: in all 8 v4 episodes it correctly
reported the footprint empty, including ep57's 0.0038 hold.

--- v4 header ---

v3 receipt: 2/4 on debug seeds 51,53,55,57 (fs_..._v3), ~160-300 sim steps of
the 1000-step horizon. Diagnosis from the v3 logs:
  * the descent to the pack's grasp z (0.9267) was BLOCKED every time, stopping
    at EEF z 0.932-0.937: the inner finger bottoms out inside the bowl, so the
    contact height -- and with it the wall radius the jaws meet -- was set by
    an uncontrolled collision and varied 5 mm seed to seed.
  * post-lift finger gap: 0.0049 (ep51 success), 0.0043/0.0045 (ep55/57).
    A close on air measures 0.0010-0.0037. The hold/air boundary is only
    ~0.5 mm wide, and v3's threshold (0.0045) aborted two episodes that may
    well have been holding the bowl.
v4: (a) descend to an UNBLOCKED height so the contact height is commanded, not
collided into; (b) aim the EEF at the wall's mid-thickness, r_out(tip)-0.0025,
measured per episode; (c) never abort on the gap alone -- verify by
re-perceiving the bowl's old footprint from a viewpoint the arm has left, and
retry the grasp with a radial correction if the bowl is still on the table.

--- v3 header ---

Measured on debug seeds 51/53/55/57 (fs_..._v1 and fs_..._v2 logs):
  * table plane z = 0.9010 on every seed; target bowl cluster top z = 0.9519.
  * rim circle fit (cells within 10 mm of the cluster top): c=(0.1287,-0.0703),
    r = 0.0529, rms 0.0056.
  * wall radius profile r95(z): 0.911->0.038 0.916->0.041 0.921->0.044
    0.926->0.045 0.931->0.047 0.936->0.049 0.941->0.052 0.946->0.056.
    The bowl flares: the radius at the pinch height is NOT the rim radius.
  * fingertip calibration (open gripper driven into the table on a clear
    patch): the descent stops with EEF z = 0.9131 = table + 0.0120, so the
    fingertips are 12 mm below the reported EEF point.
  => the pack's grasp EEF z (0.9267) puts the fingertips 13.7 mm above the
     table, where the wall radius is ~0.041, not 0.053.
  * v2's closed-loop align diverged (12 cm overshoot) because it issued a long
    move at seconds=1.0 straight out of a blocked table push; v1's slower
    move-then-correct pattern landed within 3-4 mm. v3 keeps that pattern and
    drops the (now-known) fingertip calibration.
"""
import numpy as np

PROVENANCE = {
    "DEMO_GRASP_XYZ": {"source": "pack.json demos[0..2].keyframes: mean EEF at "
                                 "the gripper_cmd=+1 (close) keyframe",
                       "allowed": True},
    "DEMO_RELEASE_XYZ": {"source": "pack.json demos[0..2].keyframes: mean EEF "
                                   "at the first gripper_cmd=-1 (release) "
                                   "keyframe", "allowed": True},
    "TIP_OFF": {"source": "debug-seed measurement (fs_..._v2 ep51/53/55/57 "
                          "TIPCAL): open gripper driven into the table stops "
                          "with EEF z = table + 0.0120", "allowed": True},
    "CELL_M": {"source": "footprint-clustering grid (1.2 cm), generic",
               "allowed": True},
    "OBJ_MIN_H": {"source": "debug-seed depth: 0.020 m above the measured table "
                            "plane separates props from surface noise",
                  "allowed": True},
    "RIM_BAND_M": {"source": "debug-seed depth: cells within 10 mm of a "
                             "cluster's top form its rim ring", "allowed": True},
    "GRASP_Z": {"source": "pack.json demo close-keyframe mean EEF z; the table "
                          "plane is identical (0.9010) on every debug seed",
                "allowed": True},
    "RELEASE_Z": {"source": "pack.json demo release-keyframe mean EEF z",
                  "allowed": True},
    "PINCH_SIDE": {"source": "pack.json: the three demo close-keyframe EEF y "
                             "(-0.033,-0.055,-0.029) span only 26 mm, i.e. one "
                             "side of the rim; debug-seed bowl centres sit at "
                             "y=-0.070..-0.084, so the demo EEF is on the +y "
                             "side of the centre by ~one wall radius",
                   "allowed": True},
    "HOLD_W": {"source": "measured finger gaps on debug seeds: 0.0043-0.0049 "
                         "after a lift that carried the bowl (fs_..._v3 "
                         "ep51/53 succeeded at 0.0049) vs 0.0010-0.0037 for a "
                         "close on air (fs_..._v1/v2)", "allowed": True},
    "PINCH_Z": {"source": "debug seeds: descents to the pack's 0.9267 were "
                          "blocked at EEF z 0.932-0.937 (fs_..._v3) = the bowl's "
                          "inner floor under the inner finger; 0.940 still "
                          "clipped it on 2/9 seeds (fs_..._v6), so 0.945",
                "allowed": True},
    "WALL_HALF_T": {"source": "debug-seed gripper caliper: the equilibrium "
                              "finger gap on the pinched wall is ~0.005 m, so "
                              "the wall's mid-thickness is 0.0025 m inside the "
                              "silhouette radius", "allowed": True},
    "LIFT_M": {"source": "generic manipulation clearance", "allowed": True},
    "CARRY_Z": {"source": "RELEASE_Z + 0.09: high enough that the carried bowl "
                          "(its base sits ~0.026 m below the EEF, measured on "
                          "debug seeds) clears the plate top measured at "
                          "table+0.019, low enough that the release descent "
                          "finishes inside its step cap", "allowed": True},
}

DEMO_GRASP_XYZ = np.array([0.13950, -0.03903, 0.92667])
DEMO_RELEASE_XYZ = np.array([0.07513, 0.23920, 0.93347])

CELL_M = 0.012
OBJ_MIN_H = 0.020
RIM_BAND_M = 0.010
TIP_OFF = 0.0120
GRASP_Z = 0.92667
RELEASE_Z = 0.93347
HOLD_W = (0.0040, 0.030)
LIFT_M = 0.12
PINCH_SIDE = 1.0          # +y side of the bowl centre
PINCH_Z = 0.945           # commanded EEF z at the pinch (arrives unblocked)
CARRY_Z = 1.0235          # RELEASE_Z + 0.09: clears the plate, short descent
WALL_HALF_T = 0.0025


def _cloud(frame):
    d = np.asarray(frame.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    x = (us - K[0, 2]) * d / K[0, 0]
    y = (vs - K[1, 2]) * d / K[1, 1]
    pc = np.stack([x, y, d, np.ones_like(d)], axis=-1).reshape(-1, 4)
    pts = (T @ pc.T).T[:, :3]
    good = np.isfinite(d).reshape(-1) & (d.reshape(-1) > 0.05) & (d.reshape(-1) < 6.0)
    return pts, good


def _table_z(pts, good):
    z = pts[good, 2]
    z = z[(z > 0.70) & (z < 1.05)]
    if z.size == 0:
        return None
    hist, edges = np.histogram(z, bins=np.arange(0.70, 1.05, 0.004))
    i = int(np.argmax(hist))
    return float(np.median(z[(z >= edges[i]) & (z < edges[i + 1])]))


def _components(cells):
    seen, out = set(), []
    for c in cells:
        if c in seen:
            continue
        stack, comp = [c], []
        seen.add(c)
        while stack:
            k = stack.pop()
            comp.append(k)
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    n = (k[0] + di, k[1] + dj)
                    if n in cells and n not in seen:
                        seen.add(n)
                        stack.append(n)
        out.append(comp)
    return out


def scene(api, tag=""):
    f = api.capture("cam_high")
    pts, good = _cloud(f)
    tz = _table_z(pts, good)
    if tz is None:
        return None, []
    m = good.copy()
    m &= (pts[:, 0] > -0.30) & (pts[:, 0] < 0.45)
    m &= (pts[:, 1] > -0.45) & (pts[:, 1] < 0.45)
    m &= (pts[:, 2] > tz + 0.004) & (pts[:, 2] < tz + 0.45)
    P = pts[m]
    api.log("%s: table_z=%.4f above-table pts=%d" % (tag, tz, P.shape[0]))
    if P.shape[0] < 50:
        return tz, []
    ij = np.floor(P[:, :2] / CELL_M).astype(int)
    key = ij[:, 0] * 100000 + ij[:, 1]
    order = np.argsort(key)
    P, ij, key = P[order], ij[order], key[order]
    bnd = np.flatnonzero(np.diff(key)) + 1
    starts = np.concatenate([[0], bnd])
    ends = np.concatenate([bnd, [key.size]])
    cells = {}
    for s, e in zip(starts, ends):
        cells[(int(ij[s, 0]), int(ij[s, 1]))] = {
            "zmax": float(P[s:e, 2].max()), "s": int(s), "e": int(e)}
    tall = {c: v for c, v in cells.items() if v["zmax"] > tz + OBJ_MIN_H}
    flat = {c: v for c, v in cells.items() if tz + 0.004 < v["zmax"] <= tz + OBJ_MIN_H}
    out = []
    for name, cs in (("tall", tall), ("flat", flat)):
        for comp in _components(cs):
            idx = np.concatenate([np.arange(cs[c]["s"], cs[c]["e"]) for c in comp])
            if idx.size < 30:
                continue
            q = P[idx]
            cx = np.array([c[0] for c in comp]) * CELL_M + CELL_M / 2
            cy = np.array([c[1] for c in comp]) * CELL_M + CELL_M / 2
            zc = np.array([cs[c]["zmax"] for c in comp])
            out.append({"kind": name, "cells": len(comp),
                        "mid": np.array([0.5 * (cx.min() + cx.max()),
                                         0.5 * (cy.min() + cy.max())]),
                        "ext": np.array([cx.max() - cx.min() + CELL_M,
                                         cy.max() - cy.min() + CELL_M]),
                        "ztop": float(zc.max()), "pts": q,
                        "cellxy": np.stack([cx, cy], axis=1), "zc": zc})
    out.sort(key=lambda c: -c["cells"])
    for i, c in enumerate(out):
        api.log("%s cl%d %s cells=%d mid=(%.3f,%.3f) ext=(%.3f,%.3f) ztop=%.3f "
                "d_grasp=%.3f d_rel=%.3f"
                % (tag, i, c["kind"], c["cells"], c["mid"][0], c["mid"][1],
                   c["ext"][0], c["ext"][1], c["ztop"],
                   float(np.linalg.norm(c["mid"] - DEMO_GRASP_XYZ[:2])),
                   float(np.linalg.norm(c["mid"] - DEMO_RELEASE_XYZ[:2]))))
    return tz, out


def fit_circle(xy):
    x, y = xy[:, 0], xy[:, 1]
    A = np.stack([x, y, np.ones_like(x)], axis=1)
    sol, *_ = np.linalg.lstsq(A, x * x + y * y, rcond=None)
    c = np.array([sol[0] / 2, sol[1] / 2])
    r = float(np.sqrt(max(sol[2] + c @ c, 1e-9)))
    rms = float(np.sqrt(np.mean((np.linalg.norm(xy - c, axis=1) - r) ** 2)))
    return c, r, rms


def wall_radius(api, cl, centre, tz, z_query, tag):
    """95th-percentile silhouette radius per 5 mm height bin, sampled at z."""
    q = cl["pts"]
    r = np.linalg.norm(q[:, :2] - centre, axis=1)
    edges = np.arange(tz, cl["ztop"] + 0.006, 0.005)
    zs, rs, rows = [], [], []
    for i in range(len(edges) - 1):
        sel = (q[:, 2] >= edges[i]) & (q[:, 2] < edges[i + 1])
        if int(sel.sum()) < 15:
            continue
        zs.append(edges[i] + 0.0025)
        rs.append(float(np.percentile(r[sel], 95)))
        rows.append("%.3f:%.3f" % (zs[-1], rs[-1]))
    api.log("%s wall profile %s" % (tag, " ".join(rows)))
    if len(zs) < 2:
        return None
    return float(np.interp(z_query, np.array(zs), np.array(rs)))


def move_to(api, xyz, seconds=1.5, tol=0.004, refine=0, tag=""):
    """Move, then optionally correct along the jaw axis (y) only.

    A move that cannot reach the controller's own POS_TOL burns its entire
    step cap, so refines are short and are abandoned the moment they stop
    buying accuracy (measured on debug seed 54: four refines, 380 steps, no
    measurable gain, and the episode ran out of horizon before the release).
    """
    xyz = np.asarray(xyz, float)
    api.move(xyz, seconds=seconds)
    e = api.eef()
    api.log("%s -> eef=(%.4f,%.4f,%.4f) err=%.4f dy=%+.4f"
            % (tag, e[0], e[1], e[2], float(np.linalg.norm(xyz - e)), xyz[1] - e[1]))
    prev = abs(float(xyz[1] - e[1]))
    for k in range(refine):
        if prev < tol:
            break
        api.move(xyz + (xyz - e), seconds=0.4)
        e = api.eef()
        dy = abs(float(xyz[1] - e[1]))
        api.log("%s refine%d -> eef=(%.4f,%.4f,%.4f) dy=%+.4f"
                % (tag, k, e[0], e[1], e[2], xyz[1] - e[1]))
        if dy > prev - 0.001:
            break
        prev = dy
    return api.eef()


def ground_target(api, cl, tz, tag, tgt=None):
    """Rim circle + wall radius at the pinch height for the demo-anchored bowl."""
    if tgt is None:
        cand = [c for c in cl if c["kind"] == "tall" and c["cells"] >= 4]
        tgt = min(cand or cl,
                  key=lambda c: float(np.linalg.norm(c["mid"] - DEMO_GRASP_XYZ[:2])))
    rim = tgt["cellxy"][tgt["zc"] > tgt["ztop"] - RIM_BAND_M]
    if rim.shape[0] >= 6:
        c_rim, r_rim, rms = fit_circle(rim)
    else:
        c_rim, r_rim, rms = tgt["mid"], 0.053, -1.0
    if float(np.linalg.norm(c_rim - tgt["mid"])) > 0.02 or not (0.03 < r_rim < 0.08):
        api.log("%s rim fit implausible (r=%.3f); using bbox mid" % (tag, r_rim))
        c_rim, r_rim = tgt["mid"], 0.053
    tip_z = PINCH_Z - TIP_OFF
    r_out = wall_radius(api, tgt, c_rim, tz, tip_z, tag)
    if r_out is None or not (0.025 < r_out < 0.070):
        r_out = r_rim - 0.008
    api.log("%s c=(%.4f,%.4f) r_rim=%.4f rms=%.4f ztop=%.4f tip_z=%.4f "
            "r_out=%.4f" % (tag, c_rim[0], c_rim[1], r_rim, rms, tgt["ztop"],
                            tip_z, r_out))
    return tgt, c_rim, r_out


def attempt_grasp(api, c_rim, r_out, bias, tag):
    """One rim pinch; returns (eef_at_pinch, finger gap after the lift)."""
    d = r_out - WALL_HALF_T + bias
    gxy = c_rim + np.array([0.0, PINCH_SIDE * d])
    api.log("%s pinch radius d=%.4f (r_out=%.4f bias=%+.4f) at (%.4f,%.4f)"
            % (tag, d, r_out, bias, gxy[0], gxy[1]))
    api.grip(0.08)
    move_to(api, [gxy[0], gxy[1], PINCH_Z + 0.045], seconds=1.6, tol=0.003,
            refine=2, tag=tag + "/approach")
    e = api.eef()
    api.move([e[0], e[1], PINCH_Z], seconds=0.7)
    e = api.eef()
    api.log("%s at pinch: eef=(%.4f,%.4f,%.4f) dy=%+.4f"
            % (tag, e[0], e[1], e[2], gxy[1] - e[1]))
    api.grip(0.0)
    api.settle(0.3)
    g = api.gripper()
    api.log("%s closed: width=%.4f" % (tag, g["width_m"]))
    move_to(api, [e[0], e[1], PINCH_Z + LIFT_M], seconds=1.0, tag=tag + "/lift")
    g = api.gripper()
    api.log("%s post-lift width=%.4f effort=%.2f" % (tag, g["width_m"], g["effort"]))
    return e, float(g["width_m"])


def bowl_like(c, tz):
    """Cluster shaped like this scene's bowl (measured: 56-58 cells at a 1.2 cm
    grid, footprint ~0.12 m, top 0.051 m above the table)."""
    return (c["kind"] == "tall" and 15 <= c["cells"] <= 150
            and max(c["ext"]) < 0.20
            and tz + 0.030 < c["ztop"] < tz + 0.075)


def bowl_on_table(cl, c_rim, tz, radius=0.12):
    """The target bowl still sitting on the table near its original footprint."""
    hits = [c for c in cl
            if bowl_like(c, tz) and float(np.linalg.norm(c["mid"] - c_rim)) < radius]
    if not hits:
        return None
    return min(hits, key=lambda c: float(np.linalg.norm(c["mid"] - c_rim)))


def run(api):
    api.log("instruction: %s" % api.instruction())
    tz, cl = scene(api, "s0")
    if not cl:
        return "no clusters"
    tgt, c_rim, r_out = ground_target(api, cl, tz, "TARGET")
    c_rim0 = c_rim.copy()

    plates = [c for c in cl if c["kind"] == "flat" and c["cells"] >= 6]
    if plates:
        pl = min(plates, key=lambda c: float(np.linalg.norm(c["mid"] - DEMO_RELEASE_XYZ[:2])))
        plate_xy = pl["mid"].copy()
        api.log("PLATE mid=(%.4f,%.4f) ext=(%.3f,%.3f) ztop=%.4f"
                % (plate_xy[0], plate_xy[1], pl["ext"][0], pl["ext"][1], pl["ztop"]))
    else:
        plate_xy = DEMO_RELEASE_XYZ[:2].copy()
        api.log("PLATE: none found; using demo release anchor")

    d_used = r_out - WALL_HALF_T
    rxy = plate_xy + np.array([0.0, PINCH_SIDE * d_used])
    for att, bias in enumerate([0.0, 0.003]):
        e, gap = attempt_grasp(api, c_rim, r_out, bias, "g%d" % att)
        d_used = r_out - WALL_HALF_T + bias
        # Step aside over the plate before judging: over the bowl's own
        # footprint the arm is what the camera sees. This is also the release
        # pose, so a held bowl needs no further transit.
        rxy = plate_xy + np.array([0.0, PINCH_SIDE * d_used])
        move_to(api, [rxy[0], rxy[1], CARRY_Z], seconds=1.6,
                tag="g%d/over_plate" % att)
        g = api.gripper()
        tz2, cl2 = scene(api, "chk%d" % att)
        left = bowl_on_table(cl2, c_rim0, tz2 or tz)
        api.log("g%d verdict: gap=%.4f width_now=%.4f still_on_table=%s"
                % (att, gap, g["width_m"],
                   None if left is None else "(%.3f,%.3f)cells=%d ztop=%.3f"
                   % (left["mid"][0], left["mid"][1], left["cells"], left["ztop"])))
        if left is None:
            break            # the bowl left the table with the gripper
        api.grip(0.08)       # jaws are empty: re-ground on the bowl we can see
        try:
            tgt, c_rim, r_out = ground_target(api, cl2, tz2 or tz,
                                              "RETARGET%d" % att, tgt=left)
        except Exception as ex:
            api.log("reground failed: %s" % ex)
    else:
        api.log("all grasp attempts exhausted")

    g = api.gripper()
    e = api.eef()
    api.log("carrying: width=%.4f effort=%.2f eef=(%.4f,%.4f,%.4f)"
            % (g["width_m"], g["effort"], e[0], e[1], e[2]))
    api.move([e[0], e[1], RELEASE_Z + 0.006], seconds=1.0)
    e = api.eef()
    api.log("at release eef=(%.4f,%.4f,%.4f) bowl centre ~(%.4f,%.4f) "
            "plate (%.4f,%.4f)" % (e[0], e[1], e[2], e[0],
                                   e[1] - PINCH_SIDE * d_used,
                                   plate_xy[0], plate_xy[1]))
    api.grip(0.08)
    api.settle(0.3)
    api.move([e[0], e[1], CARRY_Z], seconds=0.8)
    scene(api, "s1")
    return "placed"
