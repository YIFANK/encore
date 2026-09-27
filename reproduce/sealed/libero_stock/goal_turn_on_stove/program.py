"""c2 cell goal_turn_on_stove_stock -- v1.1 (v3 body, de-oracled).

v1.1 DE-ORACLE PASS (protocol compliance, no re-tuning).  v3 was selected under
fair-v1.0, where api.done doubled as a runtime success signal.  Under fair-v1.1
api.done means "episode terminated" only, so every read of it has been excised:

  * twist() no longer stops the increment ladder when done fires -- it always
    runs the fixed TWIST_DEG = 90 deg (6 x 15 deg) cap, and returns nothing;
  * run()'s retry loop no longer breaks on done, and no longer decides between
    "return" and "retreat + retry" on the twist's return value: a twist on a
    lever the gripper is measurably holding is run to completion and the
    program stops there.  The RETRY_LADDER is now purely grasp-failure
    recovery, gated on the finger gap (HOLD_MIN_W), which is own-sensor data;
  * the two api.done fields in log lines are gone.

No constant changed and no new constant was introduced.


Mechanism (read off packs/c2_goal_turn_on_stove_stock/pack.json, K=3 demos, and
confirmed on debug seeds 51/53/55/57 by v1):

  The stove is a flat pad with a black control knob at its back edge: a wide
  dark disk with an upright dark lever rising out of it.  Every demo drives the
  OPEN gripper down onto that lever (final EEF z = 0.9294/0.9294/0.9292 in all
  three demos, i.e. a fixed height), closes the fingers on it, and then rotates
  the wrist about WORLD +z by +57.7 / +50.7 / +72.5 deg while holding position.
  Nothing is carried and nothing is placed -- the whole task is one twist.

  v1 executed that with the demo-median EEF pose held fixed and scored 4/4 on
  debug seeds 51,53,55,57 (results/fs_c2_goal_turn_on_stove_stock_v1).  The
  debug captures show the stove is re-placed by ~1 cm per seed, so v2 measures
  the lever instead of assuming it: the fingers only straddle a ~2.5 cm lever,
  so a fixed pose is one bad draw away from closing beside it.  v2 scored 8/8
  on debug seeds 51..65 odd (results/fs_c2_goal_turn_on_stove_stock_v2).

  v3 fixes the one path v2 never exercised on the debug band: its retries
  re-measured the knob while the gripper was parked 8 cm directly above it, so
  cam_high saw the arm and not the lever.  v3 retreats to a clear viewing pose
  with the wrist restored to its reset orientation before every re-measurement,
  and widens the retry ladder (lower bite on the lever, then +/-x, then +/-y).
"""
import os
import numpy as np

PROVENANCE = {
    "GRASP_Z": {
        "source": "pack.json demos[*].keyframes[-1].ee[2] = 0.9308/0.9294/0.9292 "
                  "(all three demos twist at the same height); debug seeds "
                  "51/53/55/57 grasped at 0.9289 with gripper effort 3.0",
        "allowed": True},
    "PRIOR_XY": {
        "source": "pack.json demos[*].keyframes[-1].ee[:2], per-axis median of "
                  "(-0.4197,0.2020), (-0.4249,0.2051), (-0.3834,0.2105)",
        "allowed": True},
    "CLAMP_R": {
        "source": "debug seeds 51/53/55/57 cam_high captures: measured lever "
                  "bbox mid ranged over x[-0.4225,-0.4095] y[0.1985,0.2095], "
                  "i.e. ~1 cm of layout jitter; 0.06 m admits ~6x that and "
                  "still rejects a mis-detection",
        "allowed": True},
    "DX": {
        "source": "debug seeds 51/53/55/57: PRIOR_XY[0] minus the mean measured "
                  "lever bbox x-mid (-0.4160)",
        "allowed": True},
    "DY": {
        "source": "debug seeds 51/53/55/57: PRIOR_XY[1] minus the mean measured "
                  "lever bbox y-mid (0.20325)",
        "allowed": True},
    "DARK_MAX": {
        "source": "debug-seed cam_high RGB: the knob renders below 85 in every "
                  "channel while the table/wall behind it stay above 120",
        "allowed": True},
    "WIN_U": {
        "source": "debug-seed cam_high 512x512 captures: the resting arm's dark "
                  "pixels stop at u=333, the knob lever occupies u 352..369",
        "allowed": True},
    "WIN_V": {
        "source": "debug-seed cam_high captures: the lever occupies v 160..202; "
                  "the stove pad's own dark top starts below v=205",
        "allowed": True},
    "LEVER_Z": {
        "source": "debug-seed captures: deprojected lever points above the disk "
                  "lie in z 0.935..0.985 (disk top itself sits at ~0.928)",
        "allowed": True},
    "SEARCH_BOX": {
        "source": "pack.json demo EEF targets + debug-seed captures: the knob "
                  "always lies in x[-0.55,-0.25] y[0.05,0.36]",
        "allowed": True},
    "APPROACH_DZ": {
        "source": "pack.json demos[*].ee_path6: the wrist passes ~0.08 m above "
                  "the knob at t~30 before the final descent",
        "allowed": True},
    "TWIST_DEG": {
        "source": "pack.json demos[*].keyframes: relative world rotation t=0 -> "
                  "final is +57.7/+50.7/+72.5 deg about world +z; 90 deg runs "
                  "past the largest demo value.  v1.1: executed OPEN-LOOP as a "
                  "fixed 6 x 15 deg ladder (no stop condition) -- the debug "
                  "band's per-increment audit (sel run, seeds 51-65) showed the "
                  "worst seed needed 60 deg, and ~150 sim steps covers the whole "
                  "task against a 1000-step horizon",
        "allowed": True},
    "TWIST_STEP_DEG": {
        "source": "generic controller mechanics (move_pose is a P controller, "
                  "ROT_GAIN 0.30 rad saturates the command); 15 deg per call "
                  "keeps every command inside the linear range",
        "allowed": True},
    "MIN_LEVER_PTS": {
        "source": "debug seeds 51/53/55/57: the lever detector returned "
                  "418..432 points; anything under 60 is not the lever",
        "allowed": True},
    "HOLD_MIN_W": {
        "source": "debug seeds 51/53/55/57: closing on the lever leaves a finger "
                  "gap of 0.0254..0.0255 m with effort 3.0; closing on air "
                  "collapses the gap toward 0",
        "allowed": True},
    "OPEN_W": {
        "source": "debug-seed api.gripper() at reset: the open gap is 0.0778 m",
        "allowed": True},
    "VIEW_POSE": {
        "source": "debug-seed api.eef() at reset = (-0.2085, 0.0, 1.1733); the "
                  "knob detector was calibrated from cam_high captures taken "
                  "with the arm parked there, so re-measurements return to it",
        "allowed": True},
    "RETRY_LADDER": {
        "source": "debug-seed grasp geometry: the lever is ~0.025 m wide in y "
                  "(measured bbox) and the fingers close along y, so 0.012 m "
                  "steps stay inside the lever while changing the bite",
        "allowed": True},
    "DUMP_DIR": {
        "source": "debug instrumentation only (saves this program's own "
                  "api.capture output); no motion depends on it",
        "allowed": True},
}

GRASP_Z = 0.9294
PRIOR_XY = np.array([-0.4197, 0.2051])
CLAMP_R = 0.060
DX = -0.0037
DY = 0.0019
DARK_MAX = 85.0
WIN_U = (330, 470)
WIN_V = (140, 205)
LEVER_Z = (0.935, 0.985)
SEARCH_BOX = ((-0.55, -0.25), (0.05, 0.36))
APPROACH_DZ = 0.080
TWIST_DEG = 90.0
TWIST_STEP_DEG = 15.0
MIN_LEVER_PTS = 60
HOLD_MIN_W = 0.008
OPEN_W = 0.060
VIEW_POSE = np.array([-0.2085, 0.0, 1.1733])
# (dx, dy, dz) applied to the measured knob pose, one per attempt
RETRY_LADDER = [(0.000, 0.000, 0.000),
                (0.000, 0.000, -0.008),
                (-0.012, 0.000, 0.000),
                (0.012, 0.000, 0.000),
                (0.000, -0.010, 0.000),
                (0.000, 0.010, 0.000)]
DUMP_DIR = ""   # set to a writable dir to archive captures while debugging


def _rz(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _dump(api, frame, tag):
    if not DUMP_DIR:
        return
    try:
        os.makedirs(DUMP_DIR, exist_ok=True)
        i = len([p for p in os.listdir(DUMP_DIR) if p.endswith("_%s.npz" % tag)])
        np.savez_compressed(
            os.path.join(DUMP_DIR, "cap_%02d_%s_%s.npz" % (i, frame.camera, tag)),
            rgb=frame.rgb, depth=frame.depth.astype(np.float32),
            K=np.asarray(frame.intrinsics, float),
            T=np.asarray(frame.t_base_cam, float),
            eef=np.asarray(api.eef(), float))
    except Exception as e:
        api.log("dump failed: %r" % (e,))


def _cloud(frame):
    """Full-frame base-frame point cloud + validity mask."""
    dep = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = dep.shape[:2]
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    ok = np.isfinite(dep) & (dep > 0.05) & (dep < 5.0)
    z = np.where(ok, dep, 1.0)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    pts = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z, np.ones_like(z)], -1)
    return (pts @ T.T)[..., :3], ok, us, vs


def find_knob(api, frame):
    """Return (xy, n_points) for the stove knob's upright lever, or (None, n)."""
    P, ok, us, vs = _cloud(frame)
    rgb = np.asarray(frame.rgb, float)
    dark = rgb.max(-1) < DARK_MAX
    (xlo, xhi), (ylo, yhi) = SEARCH_BOX
    m = (dark & ok
         & (us >= WIN_U[0]) & (us <= WIN_U[1])
         & (vs >= WIN_V[0]) & (vs <= WIN_V[1])
         & (P[..., 2] > LEVER_Z[0]) & (P[..., 2] < LEVER_Z[1])
         & (P[..., 0] > xlo) & (P[..., 0] < xhi)
         & (P[..., 1] > ylo) & (P[..., 1] < yhi))
    n = int(m.sum())
    if n < MIN_LEVER_PTS:
        return None, n
    Q = P[m]
    # The camera sees the lever's front face and its top edge, so the silhouette
    # spans the whole body: its bounding-box midpoint is the stable anchor.
    xy = np.array([0.5 * (Q[:, 0].min() + Q[:, 0].max()) + DX,
                   0.5 * (Q[:, 1].min() + Q[:, 1].max()) + DY])
    api.log("knob: n=%d bbox x[%.4f,%.4f] y[%.4f,%.4f] -> xy=%s"
            % (n, Q[:, 0].min(), Q[:, 0].max(), Q[:, 1].min(), Q[:, 1].max(),
               np.round(xy, 4).tolist()))
    return xy, n


def knob_target(api):
    """Measured knob xy, clamped to the demo prior; prior on detection failure."""
    try:
        f = api.capture("cam_high")
        _dump(api, f, "t0")
        xy, n = find_knob(api, f)
    except Exception as e:
        api.log("perception failed: %r -- using demo prior" % (e,))
        xy, n = None, -1
    if xy is None:
        api.log("knob not detected (n=%s) -- using demo prior %s"
                % (n, PRIOR_XY.tolist()))
        return PRIOR_XY.copy()
    d = xy - PRIOR_XY
    if float(np.linalg.norm(d)) > CLAMP_R:
        api.log("knob %s is %.3f m off the demo prior -- clamping"
                % (np.round(xy, 4).tolist(), float(np.linalg.norm(d))))
        xy = PRIOR_XY + d / float(np.linalg.norm(d)) * CLAMP_R
    return xy


def grasp_knob(api, xy, nudge=(0.0, 0.0, 0.0)):
    """Open, descend onto the lever, close. Returns (target, gripper state)."""
    tgt = np.array([xy[0] + nudge[0], xy[1] + nudge[1], GRASP_Z + nudge[2]])
    api.grip(OPEN_W)
    r1 = api.move(tgt + np.array([0.0, 0.0, APPROACH_DZ]), seconds=3.0)
    r2 = api.move(tgt, seconds=2.5)
    api.grip(0.0)
    api.settle(0.3)
    g = api.gripper()
    api.log("grasp at %s: res %.4f/%.4f eef=%s grip=%s"
            % (np.round(tgt, 4).tolist(), r1, r2,
               np.round(api.eef(), 4).tolist(), g))
    return tgt, g


def retreat(api, R_home):
    """Release, lift clear of the stove and restore the reset wrist, so the
    next cam_high measurement sees the lever instead of the gripper."""
    api.grip(OPEN_W)
    here = api.eef().copy()
    api.move(np.array([here[0], here[1], GRASP_Z + APPROACH_DZ]), seconds=1.5)
    api.move(VIEW_POSE, rotation=R_home, seconds=3.0)


def twist(api):
    """Rotate the held knob about world +z by the fixed TWIST_DEG cap.

    v1.1: open-loop.  The v1.0 version stopped the increment ladder the moment
    api.done went True, which under fair-v1.1 is no longer a success signal, so
    the ladder now always runs to TWIST_DEG (6 x 15 deg = 90 deg) -- the cap the
    constant already declares, which covers the worst debug seed (65, which
    needed 60 deg) with margin and stays inside the step budget.
    """
    Rg = api.tool_rotation()
    hold = api.eef().copy()
    n = int(round(TWIST_DEG / TWIST_STEP_DEG))
    for i in range(1, n + 1):
        ang = TWIST_STEP_DEG * i
        r = api.move(hold, rotation=_rz(ang) @ Rg, seconds=1.5)
        api.log("twist %5.1f: res=%.4f eef=%s grip=%s"
                % (ang, r, np.round(api.eef(), 4).tolist(), api.gripper()))


def run(api):
    api.log("instruction: %r" % api.instruction())
    api.log("start eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    R_home = api.tool_rotation()

    for k, nudge in enumerate(RETRY_LADDER):
        xy = knob_target(api)
        _, g = grasp_knob(api, xy, nudge)
        if g["width_m"] < HOLD_MIN_W:
            api.log("attempt %d: fingers closed on air (w=%.4f) -- retry"
                    % (k, g["width_m"]))
            retreat(api, R_home)
            continue
        # Held the lever (sensor: finger gap >= HOLD_MIN_W).  Run the fixed
        # twist to completion and stop: v1.1 has no runtime success feedback,
        # so the ladder is a grasp-failure recovery only.
        twist(api)
        api.log("attempt %d: fixed %.0f deg twist completed on a held lever"
                % (k, TWIST_DEG))
        api.settle(0.5)
        api.log("final eef=%s" % (np.round(api.eef(), 4).tolist(),))
        return "knob twisted (attempt %d)" % k

    api.settle(0.5)
    api.log("final eef=%s" % (np.round(api.eef(), 4).tolist(),))
    return "knob twist exhausted retries"
