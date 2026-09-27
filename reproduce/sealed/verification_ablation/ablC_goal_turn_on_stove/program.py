"""abl_c2 / ablC_goal_turn_on_stove  --  variant C (no-verify): ONE version, zero episodes.

Intent: "turn on the stove".

Mechanism, entirely derived from packs/ablC_goal_turn_on_stove/pack.json (K=3 demos)
plus generic controller/camera mechanics:

  * All three demos do the same thing: from the home pose the arm translates to a
    single point (eef xy ~ (-0.417, 0.212), eef z ~ 0.930) with the wrist pointing
    down, CLOSES the gripper there (gripper_cmd flips -1 -> +1 at t = 41 / 48 / 40),
    and then rotates the tool about the WORLD +z axis while holding that xy and
    pushing gently down (raw action column 5 is pinned at +0.375 for the whole tail
    of every demo, column 2 stays strongly negative while the achieved z does not
    change -> the controller is pressing down against a stop).
  * Fitting R_final = Rz(theta) @ R_at_close over the three demos gives
    theta = +53 deg / +50 deg / +72 deg about world +z.  So: grasp a small
    vertical knob, then yaw it counter-clockwise (seen from above) by >= ~50 deg.
  * The closed gripper width at the end of every demo is 0.029 - 0.035 m, i.e. the
    fingers are pinching a ~3 cm post, not closing on air.  That gives a cheap
    self-check that does not touch any success signal.

The only quantity that plausibly varies across unseen layouts is the knob's xy
(the three demos span 0.048 m in x and 0.013 m in y).  z is constant to +-2 mm
across all three demos, so z is taken from the pack and xy is re-perceived every
episode with a top-down wrist-camera look: from directly above, the nearest dark
surface under the tool is the knob's cap, and its deprojected base-frame xy is the
knob axis, which for a straight-down tool is exactly where the eef must be.  Every
perception result is gated (pixel count, height below the eef, distance from the
pack nominal) and clamped; if any gate fails the pack nominal is used unchanged.
No runtime success feedback is read anywhere.
"""

import numpy as np

PROVENANCE = {
    "NOM_XY": {
        "source": "pack.json demos[*].ee_path6 -- mean of the eef xy of the 8 "
                  "post-grasp (gripper_cmd=+1) waypoints across the 3 demos: "
                  "(-0.4342,0.2167),(-0.4324,0.2119),(-0.4306,0.2200),"
                  "(-0.4300,0.2137),(-0.4276,0.2084),(-0.3967,0.2120),"
                  "(-0.3946,0.2107),(-0.3857,0.2064) -> (-0.4165, 0.2125)",
        "allowed": True},
    "Z_GRASP": {
        "source": "pack.json demos[*].ee_path6 -- eef z during the twist phase of "
                  "all 3 demos lies in [0.9279, 0.9316]; midpoint 0.930",
        "allowed": True},
    "Z_PRESS": {
        "source": "pack.json demos[*].actions column 2 stays at -0.25..-0.55 for the "
                  "whole twist while the achieved z is unchanged (demos press down "
                  "against a stop); 0.010 m below Z_GRASP reproduces that press",
        "allowed": True},
    "Z_PRE": {
        "source": "pack.json demos[*].ee_path6 -- the descent passes through "
                  "z ~ 1.00-1.02 just above the grasp; used as the clear pre-grasp "
                  "height",
        "allowed": True},
    "Z_LOOK": {
        "source": "pack.json demos[*].ee_path6[0] -- home eef z is 1.156-1.173; the "
                  "look pose is placed between home and Z_PRE so the wrist camera "
                  "sees the knob from above with clearance",
        "allowed": True},
    "TWIST_DEG": {
        "source": "pack.json demos[*].keyframes -- Rz-fit of R_final vs R_at_close "
                  "gives +53/+50/+72 deg about world +z; ramp past the largest demo "
                  "value and hold, because the demos' raw yaw command (+0.375, "
                  "saturated) is visibly resisted by the knob",
        "allowed": True},
    "GRIP_OPEN": {
        "source": "FairApi contract (api.grip: <0.025 closes, else opens) plus "
                  "pack.json gripper_state at the open keyframes (~0.072 m span)",
        "allowed": True},
    "GRIP_CLOSE": {
        "source": "FairApi contract (api.grip: <0.025 closes)",
        "allowed": True},
    "HOLD_W_MIN": {
        "source": "pack.json demos[*].keyframes[2].gripper_state -- final closed "
                  "finger spans 0.0306 / 0.0285 / 0.0346 m; lower bound set well "
                  "below the smallest so an empty (fully closed) hand is rejected",
        "allowed": True},
    "HOLD_W_MAX": {
        "source": "same pack field -- upper bound set above the largest closed span "
                  "(0.0346 m) with margin",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi contract (api.gripper effort is 3.0 iff holding)",
        "allowed": True},
    "RETRY_OFFSETS": {
        "source": "pack.json demos[*].ee_path6 -- the post-grasp eef x spans "
                  "[-0.4342, -0.3857] about the mean, i.e. +-0.024 m; the retry "
                  "offsets are the two demo extremes relative to NOM_XY",
        "allowed": True},
    "CLAMP_XY": {
        "source": "pack.json demos[*].ee_path6 -- observed cross-demo spread of the "
                  "grasp xy is 0.048 m in x / 0.013 m in y; corrections are clamped "
                  "to +-0.06 m of the pack nominal, i.e. just outside that spread",
        "allowed": True},
    "DARK_MAX": {
        "source": "pack.json keyframes/demo*_t0000.png -- the knob renders at "
                  "max(R,G,B) <= 29 while the burner disk is 60-129 and the stove "
                  "plate 140-190; 45 separates the knob from both",
        "allowed": True},
    "NEAR_BAND": {
        "source": "generic depth-camera mechanics -- a top-down look separates a "
                  "raised knob cap from the plate beneath it by its depth; 0.035 m "
                  "is the thickness of the near-surface slab kept",
        "allowed": True},
    "BORDER_FRAC": {
        "source": "generic eye-in-hand camera mechanics -- the gripper fingers "
                  "occupy the image periphery, so the outer 22% of the wrist frame "
                  "is discarded before searching for the knob",
        "allowed": True},
    "GATE_DZ": {
        "source": "generic camera mechanics -- a valid top-down knob detection must "
                  "lie below the tool; band chosen to span the Z_LOOK-to-table gap",
        "allowed": True},
    "R_DOWN": {
        "source": "pack.json demos[*].ee_path6[0][3:6] axis-angle -> the home tool "
                  "frame is diag(1,-1,-1) to within 8 deg in all 3 demos; used only "
                  "as a fallback basis if api.tool_rotation() is unavailable",
        "allowed": True},
    "GATE_XY": {
        "source": "pack.json demos[*].ee_path6 grasp-xy spread (0.048 m) -- a "
                  "detection further than 0.10 m from the pack nominal is rejected",
        "allowed": True},
}

NOM_XY = (-0.4165, 0.2125)
Z_GRASP = 0.930
Z_PRESS = 0.920
Z_PRE = 1.005
Z_LOOK = 1.075
TWIST_DEG = (25.0, 50.0, 75.0, 95.0, 110.0, 110.0, 110.0)
GRIP_OPEN = 0.080
GRIP_CLOSE = 0.000
HOLD_W_MIN = 0.012
HOLD_W_MAX = 0.055
HOLD_EFFORT = 2.5
RETRY_OFFSETS = ((-0.024, 0.000), (0.024, 0.000))
CLAMP_XY = 0.060
DARK_MAX = 45
NEAR_BAND = 0.035
BORDER_FRAC = 0.22
GATE_DZ = (0.02, 0.40)
GATE_XY = 0.10
R_DOWN = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))


def _rz(theta):
    """Rotation by theta about the world +z axis."""
    c, s = float(np.cos(theta)), float(np.sin(theta))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def _wrist_knob_xy(api):
    """Top-down wrist look -> base-frame xy of the knob axis, or None.

    Nearest dark surface inside the central crop of the wrist frame.  Everything
    is gated; a failed gate returns None and the caller keeps its prior estimate.
    """
    try:
        frame = api.capture("cam_arm_wrist")
        rgb = np.asarray(frame.rgb)
        dep = np.asarray(frame.depth, dtype=float)
    except Exception:
        return None
    if dep.ndim != 2 or rgb.ndim != 3 or dep.shape[:2] != rgb.shape[:2]:
        return None
    h, w = dep.shape
    r0, c0 = int(BORDER_FRAC * h), int(BORDER_FRAC * w)
    r1, c1 = h - r0, w - c0
    if r1 - r0 < 16 or c1 - c0 < 16:
        return None
    sd = dep[r0:r1, c0:c1]
    sc = rgb[r0:r1, c0:c1, :3].astype(np.int32)
    valid = np.isfinite(sd) & (sd > 0.02) & (sd < 3.0)
    if int(valid.sum()) < 60:
        return None
    cand = valid & (sc.max(axis=2) < DARK_MAX)
    if int(cand.sum()) < 15:
        cand = valid
    dmin = float(np.percentile(sd[cand], 5.0))
    near = cand & (sd <= dmin + NEAR_BAND)
    if int(near.sum()) < 12:
        return None
    rows, cols = np.nonzero(near)
    rmed, cmed = float(np.median(rows)), float(np.median(cols))
    keep = (np.abs(rows - rmed) < 0.12 * h) & (np.abs(cols - cmed) < 0.12 * w)
    if int(keep.sum()) < 8:
        return None
    rr = float(rows[keep].mean()) + r0
    cc = float(cols[keep].mean()) + c0
    try:
        pt = np.asarray(frame.deproject(int(round(cc)), int(round(rr))), dtype=float)
    except Exception:
        return None
    if pt.shape != (3,) or not bool(np.all(np.isfinite(pt))):
        return None
    try:
        eef_z = float(np.asarray(api.eef(), dtype=float)[2])
    except Exception:
        return None
    drop = eef_z - float(pt[2])
    if not (GATE_DZ[0] <= drop <= GATE_DZ[1]):
        return None
    if abs(float(pt[0]) - NOM_XY[0]) > GATE_XY or abs(float(pt[1]) - NOM_XY[1]) > GATE_XY:
        return None
    return float(pt[0]), float(pt[1])


def _holding(api):
    try:
        g = api.gripper()
    except Exception:
        return False, -1.0, -1.0
    try:
        width = float(g["width_m"])
    except Exception:
        width = -1.0
    try:
        effort = float(g["effort"])
    except Exception:
        effort = -1.0
    ok = (effort >= HOLD_EFFORT) or (HOLD_W_MIN <= width <= HOLD_W_MAX)
    return bool(ok), width, effort


def run(api):
    api.log("ablC turn-on-stove v1: pack-only policy, no runtime success feedback")

    lo = np.array([NOM_XY[0] - CLAMP_XY, NOM_XY[1] - CLAMP_XY], dtype=float)
    hi = np.array([NOM_XY[0] + CLAMP_XY, NOM_XY[1] + CLAMP_XY], dtype=float)
    target = np.array(NOM_XY, dtype=float)

    # --- 1. get above the pack-nominal knob position, wrist straight down --------
    api.grip(GRIP_OPEN)
    api.move([float(target[0]), float(target[1]), Z_LOOK], seconds=1.8)
    api.settle(0.2)

    # --- 2. up to two top-down wrist looks to recover the true knob axis ---------
    for look in range(2):
        found = _wrist_knob_xy(api)
        if found is None:
            api.log("look %d: no valid knob detection, keeping current estimate" % look)
            break
        new = np.clip(np.array(found, dtype=float), lo, hi)
        shift = float(np.linalg.norm(new - target))
        target = new
        api.log("look %d: knob xy = (%.4f, %.4f), shift %.4f m"
                % (look, target[0], target[1], shift))
        if shift < 0.006:
            break
        api.move([float(target[0]), float(target[1]), Z_LOOK], seconds=0.9)
        api.settle(0.15)

    # --- 3. descend and pinch the knob; self-check with the finger span ----------
    grasp_xy = np.array(target, dtype=float)
    held = False
    attempts = [(0.0, 0.0)] + [tuple(o) for o in RETRY_OFFSETS]
    for idx, (dx, dy) in enumerate(attempts):
        gx = float(target[0] + dx)
        gy = float(target[1] + dy)
        api.grip(GRIP_OPEN)
        api.move([gx, gy, Z_PRE], seconds=(1.0 if idx == 0 else 0.8))
        api.move([gx, gy, Z_GRASP], seconds=1.1)
        api.grip(GRIP_CLOSE)
        api.settle(0.35)
        ok, width, effort = _holding(api)
        api.log("grasp attempt %d at (%.4f, %.4f): width=%.4f effort=%.2f -> %s"
                % (idx, gx, gy, width, effort, "hold" if ok else "empty"))
        if ok:
            grasp_xy = np.array([gx, gy], dtype=float)
            held = True
            break

    if not held:
        # No attempt reported a hold: go back to the best estimate and close there.
        gx, gy = float(target[0]), float(target[1])
        api.grip(GRIP_OPEN)
        api.move([gx, gy, Z_GRASP], seconds=1.0)
        api.grip(GRIP_CLOSE)
        api.settle(0.3)
        grasp_xy = np.array([gx, gy], dtype=float)
        api.log("no verified hold; twisting from the primary estimate anyway")

    # --- 4. twist about world +z, pressing down, exactly as the demos do ---------
    try:
        r_base = np.asarray(api.tool_rotation(), dtype=float).reshape(3, 3)
        if not bool(np.all(np.isfinite(r_base))):
            raise ValueError("non-finite tool rotation")
    except Exception:
        r_base = np.array(R_DOWN, dtype=float)
        api.log("tool_rotation unavailable; twisting from the nominal down frame")
    px, py = float(grasp_xy[0]), float(grasp_xy[1])
    for step, deg in enumerate(TWIST_DEG):
        rot = _rz(np.radians(float(deg))).dot(r_base)
        api.move([px, py, Z_PRESS], rotation=rot, seconds=0.8)
        if step == 2:
            api.grip(GRIP_CLOSE)
    api.settle(0.3)
    api.log("twist ramp complete (%.0f deg about world +z)" % TWIST_DEG[-1])
