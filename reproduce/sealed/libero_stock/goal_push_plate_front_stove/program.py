"""c2 cell goal_push_plate_front_stove_stock -- v1.1.1 (v10 with the
termination-flag reads excised; NO other change)

fair-v1.1.1 bans every read of the runtime termination flag: LIBERO ends the
episode when the success predicate fires, so even a terminated-only flag is an
indirect success signal.  v10 read it in five places.  Three were pure
diagnostics (a field in the PUSH log line, the post-lift VERIFY log, the END
log) and are simply dropped from those format strings.  The remaining two were
control gates, and neither is load-bearing:

  * the pass-loop head skipped further passes once the episode was over.  The
    move cap in the same condition is kept; the flag term is removed.
  * the push-step loop broke out once the episode was over.  Removed; the loop
    still exits on the move cap, on lost contact, on a stall, and on reaching
    the planned end point.

Both are safe to drop because termination is absorbing on the server side: once
the episode ends the robot ignores every further command, so the residual moves
consume no simulator steps, and the benchmark verdict is latched -- extra
commands after the predicate has fired cannot undo it.  In the regime that
actually decides the outcome (episode still live) the flag was False anyway:
the v10 selection run peaked at 175 steps of a 1000-step horizon, so the flag
never gated anything on a live episode.  Placement is unchanged, all 23
constants are unchanged, and no new constant was introduced.

--- v10 rationale (unchanged) ---

v8 (single diagonal push at contact height 0.919 with proprioceptive plate
tracking and re-acquisition) scored 8/8 on the odd probe seeds and 13/15 on the
full debug split.  Both selection failures -- seeds 56 and 64, neither ever
probed -- fail the same way, and the logs blame v8's tracker, not the push:

v8 re-acquired only after TWO consecutive out-of-contact steps, so a one-step
dropout (seed 56 step 7, seed 64 step 8; eef z 0.9196/0.9199 = bare table) was
absorbed.  The plate centre is recomputed absolutely as eef - R_INNER*dhat, so
the following step snapped the estimate 0.065 m forward as if the plate had kept
up when the fingertip had actually slipped past it.  Both runs then declared
"already at goal" (estimate y 0.223/0.208) and stopped, while the parked
end-of-run capture puts the real plate at y 0.165/0.160 -- ~0.05 m short.

v9 tried to fix this by re-observing from a parked pose at the start of every
later pass, and regressed to 6/10: the adopted observation put the re-descent on
the rim (seed 51 pass 2, eef z 0.9437), and because the contact test only had a
LOWER bound a fingertip perched on the rim still read as "contact", so the whole
pass dead-reckoned while pushing nothing.

v10 therefore keeps v8's structure exactly -- no re-observation, no park detour
-- and changes only the tracker's honesty:
  * ONE out-of-contact step ends the pass; the estimate is stale the moment the
    fingertip stops touching the plate, so the next pass re-descends at the last
    value that was actually measured;
  * loss is only counted once contact has been established within that pass, so
    a pass that must first close a gap over bare table is not aborted at once;
  * the contact test is two-sided (0.9215..0.9310), which is the floor-contact
    band -- a fingertip on the 0.028 m rim reads 0.932-0.944 and is no longer
    mistaken for a fingertip driving the plate.
"""

import numpy as np

PROVENANCE = {
    "GRIPPER_ALWAYS_OPEN": {
        "source": "pack.json demos[0..2].actions[:,6]: np.unique == [-1.0] for every "
                  "step of all three demos -> push, never a grasp.",
        "allowed": True},
    "OPEN_WIDTH_M": {
        "source": "pack.json keyframe gripper_state = [0.0362,-0.0362] (jaws open).",
        "allowed": True},
    "PUSH_Z": {
        "source": "debug-seed measurement: commanding the pack's sweep height 0.919 "
                  "rests the eef at 0.9255-0.9259 with the fingertip on the 0.010 m "
                  "dish floor and moves the plate; commanding 0.932 (v6) left the "
                  "fingertip 0.023 m up, skimming the 0.028 m rim, and moved the "
                  "plate on 0/8 seeds.  Pack cross-check: the demos hold z "
                  "0.9183-0.9205 throughout the table-level phase.",
        "allowed": True},
    "CONTACT_Z_BAND": {
        "source": "debug-seed measurement of the three things the fingertip can be "
                  "resting on: bare table 0.9192-0.9205 (v7 seeds 51/61 steps 8-10, "
                  "v8 seeds 56/64 at the dropout); the 0.010 m dish floor with the "
                  "plate loaded 0.9218-0.9295 (v7 seeds 53/55/59/63 throughout, v8 "
                  "seed 51 pass-2 descent 0.9295); the 0.028 m rim 0.932-0.944 (v5 "
                  "seeds 53/55/59/63 descents, v9 seed 51 pass 2).  Band "
                  "[0.9215,0.9310] selects the middle case.",
        "allowed": True},
    "RIM_Z": {
        "source": "debug-seed measurement: 0.9256 fingertip-on-floor vs 0.932-0.938 "
                  "fingertip-on-rim (v5); the descent also under-shoots its command "
                  "by ~0.0067 m (v6), so 0.940 is used as the rim test.",
        "allowed": True},
    "APPROACH_Z": {
        "source": "pack.json demos[*].ee_path pre-descent z at the descend xy "
                  "(1.048/1.011/1.034) -> 1.02 m.",
        "allowed": True},
    "CLEAR_Z": {
        "source": "debug-seed measurement: the eef rests at 0.943 with the fingertip "
                  "on the 0.027 m rim, so 0.965 clears the plate for repositioning.",
        "allowed": True},
    "PLATE_PRIOR": {
        "source": "pack.json demos[*].ee_path first z<0.94 point (0.0364,-0.0150), "
                  "(0.0371,-0.0221), (0.0419,-0.0278) -> median (0.038,-0.022); the "
                  "3 demos agree to 1.3 cm, which anchors the plate (law C2-L1).",
        "allowed": True},
    "STOVE_PRIOR": {
        "source": "debug-seed measurement: the flat 0.015-0.05 m slab sits at "
                  "x[-0.31,-0.17], y[0.11,0.31] on seeds 51..65; its near edge reads "
                  "x1 = -0.165/-0.155 at 0.01 m cell resolution on every probed seed.",
        "allowed": True},
    "GOAL_DX_FROM_STOVE_EDGE": {
        "source": "pack keyframes demo0_t0154/demo1_t0127/demo2_t0124 converted to "
                  "base coordinates with the cam_high K,T logged by v4 (the "
                  "calibration reproduces four objects independently measured from "
                  "debug-seed depth to ~1 cm): demo final plate centres "
                  "(-0.065,0.217), (-0.043,0.223), (-0.056,0.219), i.e. 0.105-0.127 m "
                  "in front of the stove near edge -> 0.095 m.  The same conversion "
                  "applied to my own rollout gifs put successes at plate "
                  "(-0.03..-0.02, 0.18..0.26) and failures at (+0.02..+0.03, 0.22) "
                  "and (-0.04..-0.03, 0.10).",
        "allowed": True},
    "GOAL_DY_FROM_STOVE_MID": {
        "source": "same conversion: demo final plate y 0.217/0.223/0.219 (mean 0.220) "
                  "against a stove y-mid of 0.20-0.21 -> +0.015.",
        "allowed": True},
    "R_INNER": {
        "source": "debug-seed measurement: v1 seed 51, end of the +y leg -- gripper y "
                  "0.2567 vs plate blob y 0.2012 -> the pushed plate trails the "
                  "fingertip by 0.0555 m.  Pack cross-check: the demos' final eef "
                  "(-0.061,0.264)/(-0.134,0.258)/(-0.077,0.229) lie 0.05-0.06 m "
                  "beyond their final plate centres along the push direction.",
        "allowed": True},
    "PLATE_BAND": {
        "source": "debug-seed measurement: v2 map heights -- plate interior "
                  "0.005-0.015 m, rim 0.015-0.030 m, akita bowl 0.05-0.08 m; band "
                  "[0.006,0.032] isolates the plate by per-cell max height (C2-L5).",
        "allowed": True},
    "PLATE_ERODE_H": {
        "source": "debug-seed measurement: the bowl's skirt leaves in-band cells "
                  "beside its 0.05-0.08 m body and pulled v3's estimate 0.035 m in "
                  "-x on seed 51; cells within 2 of a >0.040 m cell are dropped.",
        "allowed": True},
    "STEP_LEN_SECONDS": {
        "source": "Controller mechanics: a blocked api.move burns 2*60*seconds sim "
                  "steps against a 1000-step horizon (v4 seeds 55/59 and v5 seeds "
                  "53/55/59/63 exhausted it), so the push is issued as 0.035 m / "
                  "0.35 s increments under a hard move cap.",
        "allowed": True},
    "STALL_EPS": {
        "source": "debug-seed measurement: a healthy 0.035 m increment advances the "
                  "eef 0.030-0.036 m along the push direction (v6/v7), a jammed one "
                  "advances <0.010 m (v5 seeds 53/63).",
        "allowed": True},
    "LIFT_Z": {
        "source": "debug-seed observation: 1.00 m lifts the fingertip ~0.09 m clear "
                  "of the table (v4/v7/v8 VERIFY steps), which withdraws the arm "
                  "off the plate at the end of the run.",
        "allowed": True},
    "STOVE_EDGE_PRIOR": {
        "source": "debug-seed measurement: the stove slab's near (+x) edge reads "
                  "x1 = -0.165 on seeds 51..65 at 0.01 m cell resolution (-0.155 on "
                  "one seed); used as the fallback and as the +-0.06 sanity gate on "
                  "the per-episode detection.",
        "allowed": True},
    "STOVE_BAND": {
        "source": "debug-seed measurement: v2 map stove cells carry height codes 2-3 "
                  "= 0.015-0.050 m; band [0.012,0.080] with the search confined to "
                  "x < -0.14 so the arm cannot enter it.",
        "allowed": True},
    "CELL": {
        "source": "Generic raster choice: 0.01 m XY cells, fine enough that the "
                  "plate's ~0.13 m footprint spans ~13 cells (n = 144-173 in-band "
                  "cells measured on seeds 51..65) while keeping the per-cell max "
                  "height statistic well populated.",
        "allowed": True},
    "PARK": {
        "source": "debug-seed observation: v3/v4 re-perception failed (plate=None or "
                  "arm-contaminated bboxes) whenever the arm sat between cam_high "
                  "and the plate; (0.20,-0.25,1.05) is clear of that sight line and "
                  "of the tall prop at y < -0.25.  Used only for the closing "
                  "diagnostic capture, after the plate is already placed.",
        "allowed": True},
    "MAX_MOVES_PASSES": {
        "source": "Controller mechanics + debug-seed measurement: the horizon is "
                  "1000 sim steps and a blocked api.move burns 2*60*seconds, so the "
                  "episode is capped at 36 moves / 3 passes; successful runs on "
                  "seeds 51..65 use 12-19 moves and 112-175 steps.",
        "allowed": True},
    "TABLE_Z_FALLBACK": {
        "source": "debug-seed measurement: modal deprojected table height 0.9012 m "
                  "on seeds 51..65.",
        "allowed": True},
}

OPEN_WIDTH_M = 0.08
PUSH_Z = 0.919
CONTACT_Z_MIN = 0.9215
CONTACT_Z_MAX = 0.9310
RIM_Z = 0.940
APPROACH_Z = 1.02
CLEAR_Z = 0.965
LIFT_Z = 1.00
R_INNER = 0.056
PLATE_PRIOR = (0.038, -0.022)
STOVE_PRIOR = (-0.240, 0.205)
STOVE_EDGE_PRIOR = -0.165
GOAL_DX_FROM_STOVE_EDGE = 0.095
GOAL_DY_FROM_STOVE_MID = 0.015
PLATE_BAND = (0.006, 0.032)
PLATE_ERODE_H = 0.040
STOVE_BAND = (0.012, 0.080)
CELL = 0.01
TABLE_Z_FALLBACK = 0.9012
STEP_LEN = 0.035
STALL_EPS = 0.010
MAX_MOVES = 36
MAX_PASSES = 3
PARK = (0.20, -0.25, 1.05)

_moves = [0]


def mv(api, xyz, seconds):
    if _moves[0] >= MAX_MOVES:
        return False
    _moves[0] += 1
    api.move(list(xyz), seconds=seconds)
    return True


def deproject_frame(fr):
    d = np.asarray(fr.depth, dtype=np.float64)
    if d.ndim == 3:
        d = d[..., 0]
    H, W = d.shape
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:H, 0:W].astype(np.float64)
    pc = np.stack([(uu - cx) / fx * d, (vv - cy) / fy * d, d], axis=-1)
    return pc @ T[:3, :3].T + T[:3, 3]


def cell_heights(pts, ztab, xr=(-0.36, 0.34), yr=(-0.44, 0.44)):
    X, Y, Z = pts[..., 0], pts[..., 1], pts[..., 2]
    sel = ((X >= xr[0]) & (X < xr[1]) & (Y >= yr[0]) & (Y < yr[1]) &
           np.isfinite(Z) & (Z > 0.5) & (Z < 1.8))
    nx = int(round((xr[1] - xr[0]) / CELL))
    ny = int(round((yr[1] - yr[0]) / CELL))
    h = np.full(nx * ny, -9.0)
    ix = np.floor((X[sel] - xr[0]) / CELL).astype(np.int64)
    iy = np.floor((Y[sel] - yr[0]) / CELL).astype(np.int64)
    np.maximum.at(h, ix * ny + iy, Z[sel] - ztab)
    h = h.reshape(nx, ny)
    return h, xr[0] + (np.arange(nx) + 0.5) * CELL, yr[0] + (np.arange(ny) + 0.5) * CELL


def _erode(mask, h, thresh):
    tall = h > thresh
    grow = np.zeros_like(tall)
    for dx in (-2, -1, 0, 1, 2):
        for dy in (-2, -1, 0, 1, 2):
            grow |= np.roll(np.roll(tall, dx, axis=0), dy, axis=1)
    return mask & (~grow)


def band_centre(h, cxs, cys, band, prior, radius, erode=None, iters=6):
    mask = (h >= band[0]) & (h <= band[1])
    if erode is not None:
        mask = _erode(mask, h, erode)
    ii, jj = np.nonzero(mask)
    if ii.size == 0:
        return None
    px, py = np.asarray(cxs)[ii], np.asarray(cys)[jj]
    c = np.array(prior, dtype=np.float64)
    keep = None
    for _ in range(iters):
        k = np.hypot(px - c[0], py - c[1]) <= radius
        if k.sum() < 8:
            return None
        nc = np.array([0.5 * (px[k].min() + px[k].max()),
                       0.5 * (py[k].min() + py[k].max())])
        keep = k
        if np.linalg.norm(nc - c) < 1e-4:
            c = nc
            break
        c = nc
    return dict(cx=round(float(c[0]), 4), cy=round(float(c[1]), 4), n=int(keep.sum()),
                x0=round(float(px[keep].min()), 3), x1=round(float(px[keep].max()), 3),
                y0=round(float(py[keep].min()), 3), y1=round(float(py[keep].max()), 3))


def observe(api, tag, plate_prior, want_stove=False):
    try:
        fr = api.capture("cam_high")
        pts = deproject_frame(fr)
        Z = pts[..., 2]
        m = np.isfinite(Z) & (Z > 0.7) & (Z < 1.3)
        hist, edges = np.histogram(Z[m], bins=240, range=(0.7, 1.3))
        ztab = float(edges[int(np.argmax(hist))] + 0.00125)
        if not (0.87 < ztab < 0.93):
            ztab = TABLE_Z_FALLBACK
        h, cxs, cys = cell_heights(pts, ztab)
        plate = band_centre(h, cxs, cys, PLATE_BAND, plate_prior, 0.09,
                            erode=PLATE_ERODE_H)
        stove = None
        if want_stove:
            hs = h.copy()
            hs[np.asarray(cxs) > -0.14, :] = -9.0
            stove = band_centre(hs, cxs, cys, STOVE_BAND, STOVE_PRIOR, 0.14)
        api.log("OBS %s ztab=%.4f plate=%s stove=%s" % (tag, ztab, plate, stove))
        return plate, stove
    except Exception as e:
        api.log("OBS %s FAILED %r" % (tag, e))
        return None, None


def run(api):
    api.log("instruction: %s" % api.instruction())
    e0 = api.eef()
    api.log("MARK start eef=(%.4f,%.4f,%.4f)" % (e0[0], e0[1], e0[2]))
    plate, stove = observe(api, "t0", PLATE_PRIOR, want_stove=True)

    px, py = PLATE_PRIOR
    if plate is not None:
        px, py = plate["cx"], plate["cy"]
    edge, smid = STOVE_EDGE_PRIOR, STOVE_PRIOR[1]
    if stove is not None and abs(stove["cy"] - STOVE_PRIOR[1]) < 0.10 \
            and abs(stove["x1"] - STOVE_EDGE_PRIOR) < 0.06:
        edge, smid = stove["x1"], 0.5 * (stove["y0"] + stove["y1"])
    goal = np.array([edge + GOAL_DX_FROM_STOVE_EDGE, smid + GOAL_DY_FROM_STOVE_MID])
    api.log("PLAN plate=(%.4f,%.4f) stove_edge=%.3f smid=%.3f goal=(%.4f,%.4f)"
            % (px, py, edge, smid, goal[0], goal[1]))

    api.grip(OPEN_WIDTH_M)
    plate_est = np.array([px, py], dtype=np.float64)

    for p in range(1, MAX_PASSES + 1):
        if _moves[0] >= MAX_MOVES - 3:
            break
        vec = goal - plate_est
        dist = float(np.linalg.norm(vec))
        api.log("PASS %d plate_est=(%.4f,%.4f) dist=%.4f" %
                (p, plate_est[0], plate_est[1], dist))
        if dist < 0.030:
            api.log("PASS %d already at goal" % p)
            break
        dhat = vec / dist

        # settle the fingertip onto the dish floor at the tracked plate centre
        if p > 1:
            e = api.eef()
            mv(api, [e[0], e[1], CLEAR_Z], 0.4)
            mv(api, [plate_est[0], plate_est[1], CLEAR_Z], 0.5)
        else:
            mv(api, [plate_est[0], plate_est[1], APPROACH_Z], 0.7)
        mv(api, [plate_est[0], plate_est[1], PUSH_Z], 0.6)
        z = api.eef()[2]
        api.log("DESC p%d at=(%.3f,%.3f) eef_z=%.4f rim=%s contact=%s"
                % (p, plate_est[0], plate_est[1], z, z > RIM_Z,
                   CONTACT_Z_MIN <= z <= CONTACT_Z_MAX))

        e = np.array(api.eef())
        start = e[:2].copy()
        end = goal + R_INNER * dhat
        total = float(np.linalg.norm(end - start))
        nstep = max(1, int(np.ceil(total / STEP_LEN)))
        prev_s = 0.0
        lost = 0
        stalls = 0
        z0 = api.eef()[2]
        established = CONTACT_Z_MIN <= z0 <= CONTACT_Z_MAX
        for i in range(1, nstep + 1):
            t = start + (end - start) * (i / float(nstep))
            if not mv(api, [t[0], t[1], PUSH_Z], 0.35):
                api.log("PUSH move cap")
                break
            e = np.array(api.eef())
            s = float(np.dot(e[:2] - start, dhat))
            contact = CONTACT_Z_MIN <= e[2] <= CONTACT_Z_MAX
            if contact:
                plate_est = e[:2] - R_INNER * dhat
                established = True
                lost = 0
            elif established:
                # the dead-reckoned centre is only valid while the fingertip is
                # actually driving the plate
                lost += 1
            api.log("PUSH p%d %d/%d eef=(%.4f,%.4f,%.4f) prog=%.4f contact=%s "
                    "plate_est=(%.4f,%.4f)"
                    % (p, i, nstep, e[0], e[1], e[2], s - prev_s, contact,
                       plate_est[0], plate_est[1]))
            stalls = stalls + 1 if (s - prev_s) < STALL_EPS else 0
            prev_s = s
            if lost >= 1:
                api.log("PUSH p%d contact lost -> re-acquire" % p)
                break
            if stalls >= 2:
                api.log("PUSH p%d stalled -> re-acquire" % p)
                break

    e = api.eef()
    mv(api, [e[0], e[1], LIFT_Z], 0.5)
    api.settle(0.2)
    api.log("VERIFY lifted")
    mv(api, PARK, 0.9)
    observe(api, "t2", (goal[0], goal[1]))
    api.log("END moves=%d" % _moves[0])
