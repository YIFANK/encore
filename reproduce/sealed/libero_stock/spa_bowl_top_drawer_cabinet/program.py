"""c2 cell spa_bowl_top_drawer_cabinet_stock -- v5

Receipts
  v1 (4 seeds) 0/4 : release offset applied with the wrong sign.
  v3 (8 seeds) 0/8 : wrist "close look" locked onto the gripper fingers.
  v4 (8 seeds) 5/8 : fixed demo-anchor pinch grasps on the first try on 8/8
      (effort 3.0, jaw gap 12-17 mm). Two residual failure modes:
        (A) ep53 the rim slipped out of the jaws during the lift (gap fell to
            1.7 mm = empty) -> bowl dropped back into the drawer;
        (B) ep57/ep59 the bowl landed off-centre on the plate (gif: the plate
            crescent is exposed on the -x/+y side) -> predicate not satisfied.

v5 changes, one mechanism each:
  (A) two-stage gentle lift (3 cm, then carry height) so the P-controller does
      not jerk the rim out of the pinch;
  (B) closed-loop place: at the carry pose over the plate, look at the HELD
      bowl (annulus around the eef in a height band below the wrist, so the
      fingers are excluded) and shift the drop by (plate - bowl) before
      descending.
  v5 (8 seeds) 7/8. Held-look measured the bowl at eef+(0.019,-0.042) on every
      seed (the rigid-pinch prediction is +(0.026,-0.048)), and the corrected
      place succeeded on 7. The single failure (ep63) was mode (A) again: the
      rim escaped the jaws during the second lift stage (gap 1.98 mm = empty),
      after which the program placed nothing.

v9 = v6 with a COMPLETE PROVENANCE dict (LADDER_DZ, OPEN_W, MAX_PICKS,
RETREAT_XY were used but undeclared). No behavioural change: same constants,
same control flow, re-run on all 15 debug seeds as a reproducibility receipt.

v6: the slip is stochastic (ep53 in v4, ep63 in v5, never the same seed), so
instead of chasing it, DETECT and RECOVER -- the whole pick is retried (up to
3 attempts) whenever the jaw gap says the bowl is gone, with the rim height
re-perceived each time. A settle after the close lets the jaws finish
squeezing before the lift starts.

v111 (fair-v1.1.1 termination-flag excision) = v9 with the three episode-
termination-flag reads removed and NOTHING else changed. All three were
non-load-bearing:
  1. inner grasp-ladder early exit (after the z_g decrement) -- a guard that
     only stopped no-op round-trips once the episode had already ended;
  2. pick-loop early exit after a slip retry -- same guard, same role;
  3. one log field in the post-retreat line -- pure telemetry.
Excision is provably score-neutral. The flag is only ever True after the
episode has terminated, and the harness's _step_env returns immediately when
terminated, so every command the deleted guards used to skip is a server-side
no-op that steps no simulation and changes no verdict (the benchmark bit is
latched server-side). Whenever the flag would have been False the guards never
fired, so the control flow is byte-identical. No sensor substitute was needed
and no constant changed: the hold sensor stays the raw jaw gap vs EMPTY_GAP
(2.5 mm; empty reads 1.0 mm, a held rim 3.7-17 mm on debug seeds 51-65), the
carried offset stays the per-episode held-look at the carry pose,
and the attempt counts (2 ladder rungs, MAX_PICKS=3) were already fixed.
PROVENANCE is unchanged from v9.
"""
import numpy as np

PROVENANCE = {
    "PINCH_XY": {
        "source": "pack.json demos 0-2 ee_path at gripper close -> mean "
                  "(0.044,-0.115); lies on the bowl rim ring in every "
                  "debug-seed 51-65 height map", "allowed": True},
    "GRASP_YAW_DEG": {
        "source": "pack.json ee_path6 rotvec at gripper close -> tool-y "
                  "azimuth median -61.5 deg", "allowed": True},
    "Z_RIM_PRIOR": {
        "source": "debug seeds 51-65 cam_high height maps: drawer-bowl rim at "
                  "z=1.10-1.12", "allowed": True},
    "GRASP_DZ": {
        "source": "debug seed 53 (v1) + seeds 51-65 (v4): pinch closes with "
                  "effort 3.0 at eef = rim_top - 0.012", "allowed": True},
    "LIFT_STEP": {
        "source": "debug seed 53 (v4) slip: generic P-controller mechanics "
                  "(action = err/0.05 saturates above 5 cm)", "allowed": True},
    "CARRY_Z": {
        "source": "pack.json ee_path max z while holding (1.241-1.248)",
        "allowed": True},
    "RELEASE_OFFSET": {
        "source": "pack.json demo release xy mean (0.069,0.248) minus the "
                  "debug-seed plate centre (0.069,0.207) -> (0.000,+0.041); "
                  "used as the pre-look drop prior", "allowed": True},
    "RELEASE_DZ": {
        "source": "pack.json demo release z median 0.9418 minus the "
                  "debug-seed plate top 0.920 -> +0.022", "allowed": True},
    "EMPTY_GAP_M": {
        "source": "debug seeds: an empty closed gripper reads gap 0.0010 m "
                  "(v4 ep53, v5 ep63) while a held rim reads 0.0037-0.017 m "
                  "-> lost-bowl threshold 0.0025", "allowed": True},
    "HELD_ANNULUS": {
        "source": "debug-seed geometry: bowl rim radius ~0.055 m, jaw "
                  "half-width ~0.03 m -> measure the held bowl at radius "
                  "0.032-0.14 m from the eef", "allowed": True},
    "PLATE_ANCHOR": {
        "source": "pack.json demo release xy mean (0.069,0.248)", "allowed": True},
    "TABLE_Z": {"source": "debug-seed depth histogram modal plane",
                "allowed": True},
    "LADDER_DZ": {
        "source": "harness position tolerance POS_TOL=0.012 m (generic "
                  "controller mechanics): one tolerance-width step per retry",
        "allowed": True},
    "OPEN_W": {
        "source": "pack.json gripper_state at t=0 (0.0362,-0.0362) -> 72 mm "
                  "jaw gap; any value above the harness's 0.025 m grip-intent "
                  "threshold means 'open'", "allowed": True},
    "MAX_PICKS": {
        "source": "debug-seed step budget: one pick attempt costs ~200 of the "
                  "1000 sim steps, so at most 3 fit alongside the place",
        "allowed": True},
    "RETREAT_XY": {
        "source": "debug-seed camera extrinsics (t_base_cam): park the arm off "
                  "the camera-to-plate ray so the post-check can see the "
                  "landing", "allowed": True},
}

PINCH = np.array([0.044, -0.115])
GRASP_YAW_DEG = -61.5
Z_RIM_PRIOR = 1.110
GRASP_DZ = -0.012
LADDER_DZ = 0.012
LIFT_STEP = 0.030
EMPTY_GAP = 0.0025
MAX_PICKS = 3
CARRY_Z = 1.250
RELEASE_ANCHOR = np.array([0.069, 0.248])
RELEASE_OFFSET = np.array([0.000, 0.041])
RELEASE_DZ = 0.022
OPEN_W = 0.08


def rot_from_yaw(yaw_deg):
    a = np.radians(yaw_deg)
    ty = np.array([np.cos(a), np.sin(a), 0.0])
    tz = np.array([0.0, 0.0, -1.0])
    return np.stack([np.cross(ty, tz), ty, tz], axis=1)


def make_cloud(frame, step=1):
    d = np.asarray(frame.depth, float)
    h, w = d.shape[:2]
    vs, us = np.mgrid[0:h:step, 0:w:step]
    z = d[vs, us]
    K = np.asarray(frame.intrinsics, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    p = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z, np.ones_like(z)], axis=-1)
    base = (p @ np.asarray(frame.t_base_cam, float).T)[..., :3]
    rgb = np.asarray(frame.rgb, float)[vs, us]
    ok = np.isfinite(z) & (z > 1e-4)
    return base.reshape(-1, 3), rgb.reshape(-1, 3), ok.reshape(-1)


def blobs(pts, cols, ok, ref, cell=0.015, min_pts=12):
    p, c = pts[ok], cols[ok].mean(axis=1)
    if not len(p):
        return []
    keys = np.floor(p[:, :2] / cell).astype(int)
    cells = {}
    for i in range(len(p)):
        cells.setdefault((int(keys[i, 0]), int(keys[i, 1])), []).append(i)
    seen, rows = set(), []
    for k in cells:
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            q = stack.pop()
            comp.append(q)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nb = (q[0] + dx, q[1] + dy)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        idx = np.concatenate([np.asarray(cells[q], int) for q in comp])
        if len(idx) < min_pts:
            continue
        q = p[idx]
        mid = [(q[:, 0].min() + q[:, 0].max()) / 2, (q[:, 1].min() + q[:, 1].max()) / 2]
        rows.append(dict(n=int(len(idx)), mid=[round(v, 4) for v in mid],
                         ext=[round(float(np.ptp(q[:, 0])), 3),
                              round(float(np.ptp(q[:, 1])), 3)],
                         zmax=round(float(q[:, 2].max()), 3),
                         br=round(float(c[idx].mean()), 0),
                         d=round(float(np.hypot(mid[0] - ref[0], mid[1] - ref[1])), 3)))
    rows.sort(key=lambda r: r["d"])
    return rows


def held_bowl_xy(api, eef):
    """Where is the carried bowl? Annulus around the eef, below the wrist."""
    f = api.capture("cam_high")
    p, c, ok = make_cloud(f, 1)
    r = np.hypot(p[:, 0] - eef[0], p[:, 1] - eef[1])
    m = ok & (r > 0.032) & (r < 0.14) & (p[:, 2] > eef[2] - 0.10) & (p[:, 2] < eef[2] + 0.015)
    n = int(m.sum())
    if n < 150:
        api.log("held-look: only %d pts" % n)
        return None, n
    q = p[m]
    mid = np.array([(q[:, 0].min() + q[:, 0].max()) / 2,
                    (q[:, 1].min() + q[:, 1].max()) / 2])
    api.log("held-look n=%d mid=%s ext=%s zspan=%s"
            % (n, np.round(mid, 4).tolist(),
               [round(float(np.ptp(q[:, 0])), 3), round(float(np.ptp(q[:, 1])), 3)],
               [round(float(q[:, 2].min()), 3), round(float(q[:, 2].max()), 3)]))
    return mid, n


def run(api):
    f = api.capture("cam_high")
    pts, cols, ok = make_cloud(f, 1)
    hist, edges = np.histogram(pts[ok][:, 2], bins=120)
    z_table = float(edges[int(np.argmax(hist))] + (edges[1] - edges[0]) / 2)

    near = ok & (np.hypot(pts[:, 0] - PINCH[0], pts[:, 1] - PINCH[1]) < 0.030) \
        & (pts[:, 2] > 1.05) & (pts[:, 2] < 1.16)
    z_rim = float(np.percentile(pts[near][:, 2], 97)) if near.sum() > 20 else Z_RIM_PRIOR
    if not (1.06 <= z_rim <= 1.15):
        z_rim = Z_RIM_PRIOR
    api.log("z_table=%.4f z_rim=%.4f" % (z_table, z_rim))

    flat = ok & (pts[:, 2] > z_table + 0.006) & (pts[:, 2] < z_table + 0.05) \
        & (np.abs(pts[:, 0]) < 0.45) & (np.abs(pts[:, 1]) < 0.45)
    prows = blobs(pts, cols, flat, RELEASE_ANCHOR, min_pts=40)
    for r in prows[:3]:
        api.log("PLATE %s" % r)
    plate = np.array(prows[0]["mid"], float) if prows else RELEASE_ANCHOR.copy()
    plate_z = prows[0]["zmax"] if prows else z_table + 0.02
    if np.hypot(*(plate - RELEASE_ANCHOR)) > 0.10:
        plate, plate_z = RELEASE_ANCHOR.copy(), z_table + 0.02
    api.log("plate=%s z=%.3f" % (np.round(plate, 4).tolist(), plate_z))

    # ---- grasp, with recovery from a slipped rim ---------------------------
    R = rot_from_yaw(GRASP_YAW_DEG)
    api.grip(OPEN_W)
    api.move([PINCH[0], PINCH[1], CARRY_Z], rotation=R, seconds=2.0)

    held = False
    for pick in range(MAX_PICKS):
        if pick:                       # re-perceive the rim after a drop
            fr = api.capture("cam_high")
            pr, cr, okr = make_cloud(fr, 1)
            nr = okr & (np.hypot(pr[:, 0] - PINCH[0], pr[:, 1] - PINCH[1]) < 0.030) \
                & (pr[:, 2] > 1.05) & (pr[:, 2] < 1.16)
            if nr.sum() > 20:
                zr = float(np.percentile(pr[nr][:, 2], 97))
                if 1.06 <= zr <= 1.15:
                    z_rim = zr
            api.log("pick%d re-perceived z_rim=%.4f" % (pick, z_rim))
        z_g = z_rim + GRASP_DZ
        closed = False
        for k in range(2):
            res = api.move([PINCH[0], PINCH[1], z_g], seconds=1.0)
            api.grip(0.0)
            api.settle(0.25)
            g = api.gripper()
            api.log("pick%d try%d z=%.4f res=%.4f eef=%s grip=%s"
                    % (pick, k, z_g, res, np.round(api.eef(), 4).tolist(), g))
            if g["width_m"] > EMPTY_GAP:
                closed = True
                break
            api.grip(OPEN_W)
            z_g -= LADDER_DZ
        if not closed:
            api.log("pick%d: closed on air" % pick)
            continue
        # gentle staged lift, checking the jaw gap after each stage
        lost = False
        for dz in (0.015, LIFT_STEP + 0.02, None):
            tz = CARRY_Z if dz is None else z_g + dz
            api.move([PINCH[0], PINCH[1], tz], seconds=0.7 if dz is not None else 1.0)
            g = api.gripper()
            api.log("pick%d lift z=%.4f eef=%s grip=%s"
                    % (pick, tz, np.round(api.eef(), 4).tolist(), g))
            if g["width_m"] < EMPTY_GAP:
                lost = True
                break
        if not lost:
            held = True
            break
        api.log("pick%d: rim slipped, retrying" % pick)
        api.grip(OPEN_W)
        api.move([PINCH[0], PINCH[1], CARRY_Z], seconds=0.8)
    api.log("held=%s" % held)

    # ---- carry, look at the held bowl, correct, place ----------------------
    drop = plate + RELEASE_OFFSET
    api.move([drop[0], drop[1], CARRY_Z], seconds=1.5)
    eef = api.eef()
    api.log("over eef=%s grip=%s" % (np.round(eef, 4).tolist(), api.gripper()))
    bowl, n = held_bowl_xy(api, eef)
    if bowl is not None:
        shift = plate - bowl
        if np.hypot(*shift) > 0.07:
            api.log("shift %s too big -> clipped" % np.round(shift, 4).tolist())
            shift = shift / np.hypot(*shift) * 0.07
        drop = np.array([eef[0], eef[1]]) + shift
        api.log("bowl=%s shift=%s -> drop=%s"
                % (np.round(bowl, 4).tolist(), np.round(shift, 4).tolist(),
                   np.round(drop, 4).tolist()))
        api.move([drop[0], drop[1], CARRY_Z], seconds=0.8)

    z_rel = plate_z + RELEASE_DZ
    res = api.move([drop[0], drop[1], z_rel], seconds=1.2)
    api.log("place drop=%s res=%.4f eef=%s grip=%s"
            % (np.round(drop, 4).tolist(), res,
               np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.5)
    api.move([drop[0], drop[1], CARRY_Z], seconds=1.0)
    api.move([0.02, -0.16, 1.30], seconds=1.2)
    api.settle(0.4)
    api.log("retreated eef=%s" % (np.round(api.eef(), 4).tolist(),))

    # ---- clean landing measurement (band well above the plate) -------------
    try:
        f2 = api.capture("cam_high")
        p2, c2, ok2 = make_cloud(f2, 1)
        m = ok2 & (p2[:, 2] > plate_z + 0.030) & (p2[:, 2] < plate_z + 0.12) \
            & (np.abs(p2[:, 0] - plate[0]) < 0.22) & (np.abs(p2[:, 1] - plate[1]) < 0.22)
        rows = blobs(p2, c2, m, plate, min_pts=20)
        for r in rows[:3]:
            api.log("POST %s" % r)
        if rows:
            api.log("LANDED err=%s d=%.3f"
                    % (np.round(np.array(rows[0]["mid"]) - plate, 4).tolist(),
                       rows[0]["d"]))
    except Exception as e:
        api.log("post failed: %r" % (e,))
    return "v111 held=%s" % held
