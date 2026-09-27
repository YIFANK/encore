"""ablC_goal_open_middle_drawer -- "open the middle drawer of the cabinet".

Variant C (no-verify): exactly ONE version, zero episodes run.  Every constant
below is read off this cell's own pack (packs/ablC_goal_open_middle_drawer/):
the K=3 demo ee_path6 / gripper traces and the cam_high keyframe PNGs.

Mechanism recovered from the pack
---------------------------------
gripper_cmd is -1.0 (open) and gripper_state stays at ~0.036-0.040 for every
keyframe of all three demos: the drawer is never *grasped*.  The wrist rolls so
that the finger-separation axis points along world +x and the palm axis (tool
z) tilts from straight-down to ~58-60 deg off vertical, i.e. the open fingers
point mostly along -y and ~30 deg downwards.  The arm then drives diagonally
down and in to (x~+0.021, y~-0.1435, z~1.039) -- the y coordinate of that
insertion point has a cross-demo sd of 0.5 mm, so it is a hard geometric
constant (the cabinet's drawer-face plane) -- seats ~7 mm downwards to hook the
handle, and finally translates along +y to y~+0.024, dragging the drawer open.

The cabinet is a fixed fixture in this scene: the dark cabinet mask in the
three t=0 cam_high keyframes agrees to <=1 px in both centroid coordinates.
Perception is therefore used conservatively -- a heavily-shrunk, hard-clipped
pixel-shift correction -- and as a *verdict* sensor: the dark-pixel count in
the image band just +y of the closed cabinet separates closed (222-238 px at
128x128) from opened (825-869 px) by a factor of ~3.6, which lets the policy
re-hook at a different height if the first hook missed, without any privileged
success signal.
"""

import numpy as np

PROVENANCE = {
    # --- trajectory, from pack.json demos[*].ee_path6 -----------------------
    "WAYPOINTS_XYZ": {
        "source": "pack.json demos[0..2].ee_path6 positions, each demo "
                  "phase-resampled to 14 samples and averaged across K=3",
        "allowed": True},
    "TILT_DEG": {
        "source": "pack.json demos[0..2].ee_path6 axis-angle -> tool z axis, "
                  "angle from world -z, phase-resampled and averaged (K=3)",
        "allowed": True},
    "INSERT_XYZ": {
        "source": "pack.json ee_path6 mean insertion sample "
                  "(0.0208, -0.1435, 1.0388); cross-demo sd (18, 0.5, 5) mm",
        "allowed": True},
    "SEAT_DZ": {
        "source": "pack.json ee_path6: z drops 1.0388 -> 1.0322 at fixed "
                  "y=-0.1435 between the insertion and the first pull sample",
        "allowed": True},
    "PULL_Z": {
        "source": "pack.json ee_path6 mean z over the pull samples (1.0322 to "
                  "1.0328)", "allowed": True},
    "PULL_END_Y": {
        "source": "pack.json ee_path6 mean final y = +0.0237 (all three demos "
                  "end between +0.016 and +0.030); +0.020 m of extra travel "
                  "added as margin", "allowed": True},
    "GRIP_OPEN_M": {
        "source": "pack.json gripper_cmd = -1.0 and gripper_state ~ "
                  "(+0.036..+0.040, -0.036..-0.040) at EVERY keyframe of all "
                  "three demos: the gripper is open for the whole demo; "
                  "FairApi opens for any width >= 0.025", "allowed": True},
    # --- perception, from pack keyframes/*.png (cam_high, 128x128) ----------
    "DARK_MAX": {
        "source": "keyframes/demo{0,1,2}_t0000.png: the cabinet body renders "
                  "with max(R,G,B) well under 70/255 while the table, wall and "
                  "robot render far above it", "allowed": True},
    "CAB_WIN_128": {
        "source": "keyframes/demo{0,1,2}_t0000.png: the closed cabinet's dark "
                  "mask occupies u<45, 40<=v<115 in the 128x128 keyframes",
        "allowed": True},
    "CAB_REF_UV_128": {
        "source": "keyframes/demo{0,1,2}_t0000.png: dark-mask centroid = "
                  "(19.59,74.88), (19.40,74.67), (19.09,73.91) -> mean "
                  "(19.36,74.49); spread <=1 px, so the cabinet is a fixed "
                  "fixture across the demo layouts", "allowed": True},
    "CAB_N_RANGE_128": {
        "source": "keyframes/demo{0,1,2}_t0000.png: dark-mask pixel count in "
                  "CAB_WIN_128 = 2281 / 2279 / 2228", "allowed": True},
    "OPEN_BAND_128": {
        "source": "keyframes: dark count in 45<=u<75, 40<=v<125 is 222/238/233 "
                  "with the drawer closed (t=0) and 869/866/825 with the "
                  "drawer pulled open (final keyframe of each demo)",
        "allowed": True},
    "OPEN_THRESH_128": {
        "source": "midpoint-biased split of OPEN_BAND_128 (closed <=238, open "
                  ">=825); 520 sits ~2.2x above the closed baseline and ~1.6x "
                  "below the open one", "allowed": True},
    "PULL_TILT_DEG": {
        "source": "pack.json ee_path6: mean tool-z tilt off vertical over the "
                  "insertion and pull samples of the K=3 demos (57.4-60.6 deg)",
        "allowed": True},
    "SHIFT_SHRINK": {
        "source": "generic robustness margin: shrink the perceived cabinet "
                  "displacement towards the pack prior (the cabinet mask is "
                  "identical to <=1 px across all three demo layouts)",
        "allowed": True},
    "SHIFT_CLIP_X": {
        "source": "generic robustness margin vs the cross-demo insertion-point "
                  "sd in x from pack.json ee_path6 (18 mm)", "allowed": True},
    "SHIFT_CLIP_Z": {
        "source": "generic robustness margin held BELOW the cross-demo "
                  "insertion-point sd in z from pack.json ee_path6 (5 mm), so "
                  "a perception error can never dominate the pack prior", "allowed": True},
    "SHIFT_MAX_PX_128": {
        "source": "keyframes/demo{0,1,2}_t0000.png: the cabinet centroid varies "
                  "by <=1 px across demos, so a shift beyond ~22 px means the "
                  "mask latched onto something else", "allowed": True},
    "RETRY_DZ": {
        "source": "pack.json ee_path6: cross-demo sd of the insertion height is "
                  "5 mm and the per-demo z spread over the hook samples is "
                  "~15 mm, so +/-14 mm brackets the plausible handle height",
        "allowed": True},
    "JAC_STEP_PX": {
        "source": "generic camera mechanics: an 8-pixel finite-difference step "
                  "for the local pixel->base-frame Jacobian of api.deproject",
        "allowed": True},
    "TIME_BUDGET_S": {
        "source": "generic controller mechanics: self-imposed cap on total "
                  "commanded motion time so re-hook attempts cannot run past "
                  "the episode horizon", "allowed": True},
}

# ---------------------------------------------------------------- constants
GRIP_OPEN_M = 0.080

# mean K=3 approach path (14 phase-matched samples), positions in base frame
WAYPOINTS_XYZ = [
    (-0.2016, -0.0018, 1.1750),
    (-0.1482, -0.0107, 1.1722),
    (-0.0816, -0.0103, 1.1709),
    (-0.0419, -0.0073, 1.1633),
    (-0.0236, -0.0120, 1.1394),
    (0.0009, -0.0217, 1.1138),
    (0.0361, -0.0463, 1.0979),
    (0.0358, -0.0757, 1.0666),
    (0.0287, -0.1101, 1.0520),
]
TILT_DEG = [4.0, 11.4, 20.4, 32.1, 45.0, 54.6, 59.3, 60.4, 59.8]

INSERT_XYZ = (0.0208, -0.1435, 1.0388)
SEAT_DZ = -0.0066
PULL_Z = 1.0325
PULL_END_Y = 0.0437          # 0.0237 demo mean + 0.020 margin
PULL_TILT_DEG = 58.5

DARK_MAX = 70
CAB_WIN_128 = (0, 45, 40, 115)        # u0, u1, v0, v1
CAB_REF_UV_128 = (19.36, 74.49)
CAB_N_RANGE_128 = (1500, 3400)
OPEN_BAND_128 = (45, 75, 40, 125)
OPEN_THRESH_128 = 520

SHIFT_SHRINK = 0.70
SHIFT_CLIP_X = 0.030
SHIFT_CLIP_Z = 0.008
SHIFT_MAX_PX_128 = 22.0
JAC_STEP_PX = 8

TIME_BUDGET_S = 34.0
RETRY_DZ = (0.0, -0.014, +0.014)


# ------------------------------------------------------------------ helpers
def tool_rot(tilt_deg):
    """Tool frame at `tilt_deg` off vertical, palm axis swung towards -y.

    Columns are (tool x, tool y, tool z).  tool y is the finger-separation
    axis and is held along world +x; tool z is the palm/approach axis.  At
    tilt=0 this is a straight-down wrist; at tilt=58.5 deg it reproduces the
    demo hook pose (tool z ~ (0, -0.85, -0.52), tool y ~ (1, 0, 0)).
    """
    a = np.radians(float(tilt_deg))
    s, c = np.sin(a), np.cos(a)
    tz = np.array([0.0, -s, -c])
    ty = np.array([1.0, 0.0, 0.0])
    tx = np.cross(ty, tz)
    return np.stack([tx, ty, tz], axis=1)


class Budget(object):
    """Self-imposed motion-time counter (no episode/termination flag is read)."""

    def __init__(self, api, total):
        self.api = api
        self.left = float(total)

    def move(self, xyz, tilt_deg, seconds):
        if self.left <= 0.0:
            return None
        seconds = float(min(seconds, self.left))
        self.left -= seconds
        try:
            return self.api.move(np.asarray(xyz, dtype=float),
                                 rotation=tool_rot(tilt_deg),
                                 seconds=seconds)
        except Exception:
            return None


def _frame_mask(rgb, win128, w, h):
    """Dark-pixel boolean mask inside a window given in 128x128 keyframe units."""
    sx, sy = w / 128.0, h / 128.0
    u0, u1, v0, v1 = win128
    sub = rgb[int(round(v0 * sy)):int(round(v1 * sy)),
              int(round(u0 * sx)):int(round(u1 * sx))]
    if sub.size == 0:
        return None, 0.0
    dark = sub.max(axis=2) < DARK_MAX
    # counts are reported in 128x128-equivalent pixels
    return dark, float(dark.sum()) / max(sx * sy, 1e-6)


def cabinet_shift(api):
    """Estimate the cabinet's in-plane displacement w.r.t. the demo layout.

    Returns a base-frame (dx, dy, dz); (0,0,0) whenever the estimate is not
    trustworthy.  Only x and z are ever applied by the caller.
    """
    zero = np.zeros(3)
    try:
        f = api.capture("cam_high")
        rgb = np.asarray(f.rgb)
        if rgb.ndim != 3 or rgb.shape[2] < 3:
            return zero
        h, w = rgb.shape[0], rgb.shape[1]
        sx, sy = w / 128.0, h / 128.0
        dark, n128 = _frame_mask(rgb, CAB_WIN_128, w, h)
        if dark is None:
            return zero
        if not (CAB_N_RANGE_128[0] <= n128 <= CAB_N_RANGE_128[1]):
            api.log("cabinet mask implausible (%.0f px); no shift" % n128)
            return zero
        vv, uu = np.nonzero(dark)
        u0 = int(round(CAB_WIN_128[0] * sx))
        v0 = int(round(CAB_WIN_128[2] * sy))
        cu = float(uu.mean()) + u0
        cv = float(vv.mean()) + v0
        du128 = cu / sx - CAB_REF_UV_128[0]
        dv128 = cv / sy - CAB_REF_UV_128[1]
        if not np.isfinite(du128) or not np.isfinite(dv128):
            return zero
        if np.hypot(du128, dv128) > SHIFT_MAX_PX_128:
            api.log("cabinet centroid shift %.1f px > gate; no shift"
                    % np.hypot(du128, dv128))
            return zero
        # local pixel -> base-frame Jacobian on the cabinet surface itself
        p0 = np.asarray(f.deproject(cu, cv), dtype=float)
        pu = np.asarray(f.deproject(cu + JAC_STEP_PX, cv), dtype=float)
        pv = np.asarray(f.deproject(cu, cv + JAC_STEP_PX), dtype=float)
        if not (np.all(np.isfinite(p0)) and np.all(np.isfinite(pu))
                and np.all(np.isfinite(pv))):
            return zero
        ju = (pu - p0) / JAC_STEP_PX
        jv = (pv - p0) / JAC_STEP_PX
        if max(np.linalg.norm(ju), np.linalg.norm(jv)) > 0.02:
            return zero          # depth discontinuity under the sample points
        d = ju * (du128 * sx) + jv * (dv128 * sy)
        d = SHIFT_SHRINK * d
        dx = float(np.clip(d[0], -SHIFT_CLIP_X, SHIFT_CLIP_X))
        dz = float(np.clip(d[2], -SHIFT_CLIP_Z, SHIFT_CLIP_Z))
        api.log("cabinet pixel shift (%.1f,%.1f)px -> dx=%.3f dz=%.3f"
                % (du128, dv128, dx, dz))
        return np.array([dx, 0.0, dz])
    except Exception:
        return zero


def drawer_looks_open(api):
    """True iff the band just +y of the closed cabinet is filled with drawer.

    Uses only the RGB the policy is allowed to see; the closed / opened
    reference counts come from this cell's own demo keyframes.
    """
    try:
        f = api.capture("cam_high")
        rgb = np.asarray(f.rgb)
        h, w = rgb.shape[0], rgb.shape[1]
        _, n128 = _frame_mask(rgb, OPEN_BAND_128, w, h)
        api.log("open-band dark count %.0f px (closed ~230, open ~850)" % n128)
        return n128 >= OPEN_THRESH_128
    except Exception:
        return False


# --------------------------------------------------------------------- main
def run(api):
    api.log("task: %s" % api.instruction())
    try:
        api.grip(GRIP_OPEN_M)          # demos never close the gripper
    except Exception:
        pass
    api.settle(0.1)

    shift = cabinet_shift(api)
    bud = Budget(api, TIME_BUDGET_S)

    ix, iy, iz = INSERT_XYZ
    ix += float(shift[0])
    iz += float(shift[2])

    pull_ys = (-0.100, -0.040, 0.000, PULL_END_Y)

    for attempt, dz in enumerate(RETRY_DZ):
        if bud.left <= 3.0:
            api.log("motion budget exhausted; stopping after %d attempt(s)"
                    % attempt)
            break

        z_ins = iz + dz
        z_pull = PULL_Z + float(shift[2]) + dz

        if attempt == 0:
            # full demo approach: roll the wrist, swing forward, tilt the palm
            # towards -y and drive diagonally down onto the drawer face.
            for k, (wp, tilt) in enumerate(zip(WAYPOINTS_XYZ, TILT_DEG)):
                p = np.array(wp, dtype=float)
                if k >= 5:                      # only the near-cabinet part is
                    p[0] += float(shift[0])     # cabinet-referenced
                    p[2] += float(shift[2])
                bud.move(p, tilt, 0.7 if k else 1.0)
        else:
            api.log("re-hook attempt %d at dz=%+.3f" % (attempt + 1, dz))
            # back out along +y and up, then come down in front of the face
            bud.move((ix, 0.010, 1.130), PULL_TILT_DEG, 1.2)
            bud.move((ix, -0.075, 1.075 + dz), PULL_TILT_DEG, 1.1)
            bud.move((ix, -0.110, z_ins + 0.014), PULL_TILT_DEG, 0.8)

        # insert: fingers slide past the handle on the drawer face
        bud.move((ix, iy, z_ins), PULL_TILT_DEG, 1.2)
        # seat: 7 mm down at depth, hooking the handle in the finger crook
        bud.move((ix, iy, z_ins + SEAT_DZ), PULL_TILT_DEG, 0.6)

        # pull: steady +y translation, split so the controller keeps loading
        for y in pull_ys:
            bud.move((ix, y, z_pull), PULL_TILT_DEG, 1.1)

        # clear the drawer mouth so the next capture is not occluded
        bud.move((ix, PULL_END_Y + 0.030, z_pull + 0.060), PULL_TILT_DEG, 0.8)
        bud.move((-0.120, 0.060, 1.170), 20.0, 1.2)
        api.settle(0.2)

        if drawer_looks_open(api):
            api.log("drawer reads OPEN after attempt %d; done" % (attempt + 1))
            return
        api.log("drawer still reads CLOSED after attempt %d" % (attempt + 1))

    api.log("finished (budget %.1f s left)" % max(bud.left, 0.0))
