"""abl_c2 / cell ablC_goal_put_wine_on_rack  --  variant C (no-verification ablation).

Intent: "put the wine bottle on the rack".

Written from packs/ablC_goal_put_wine_on_rack/ ONLY (pack.json + keyframes/*.png).
Zero episodes were run while authoring this file; there is exactly one version.

Mechanism (all of it read off the pack):
  * The three demos share one geometry.  The gripper yaws so that its finger-
    opening axis lies along world +x, tips its tool axis ~56 deg from vertical
    toward -y, approaches the standing bottle along that tilted axis, closes,
    lifts, carries toward -y while UN-tilting back to near-vertical (which
    lays the held bottle over by ~47 deg), and opens above the rack cradle.
  * Layouts are randomised, so the bottle and the rack are re-located at
    episode start from cam_high.  Both are located DIFFERENTIALLY: the same
    pixel-domain estimator is run on the pack keyframe (whose pixel statistic
    is baked in below) and on the live frame, both pixels are cast onto one
    common horizontal plane through the live camera model, and the difference
    is added to the pack's own end-effector pose.  Any systematic bias of the
    estimator (silhouette centroid vs. true object axis, unknown offset
    between api.eef() and the fingertips) cancels in that difference, so no
    hand-guessed tool offset is ever needed.
  * The only runtime feedback used is the gripper's own effort/width, which
    licenses one re-attempt of the grasp (demo2 in the pack needed exactly
    such a re-attempt: it closed on empty air at x=-0.146 and succeeded at
    x=-0.198).
"""

import math

import numpy as np

PROVENANCE = {
    # ---- grasp reference pose (paired, per demo, with BOTTLE_PX) ----------
    "GRASP_REF": {
        "source": "pack.json demos[0].keyframes t=81 ee=[-0.2098,-0.0815,1.0176] and "
                  "demos[1].keyframes t=76 ee=[-0.2139,-0.0685,0.9831]: the two "
                  "first-attempt gripper-close transitions (actions[:,6] -1 -> +1) "
                  "that were never followed by a re-open before the lift",
        "allowed": True},
    "GRASP_Z": {
        "source": "pack.json demos[1].keyframes t=76 ee[2]=0.9831, the cleanest "
                  "single-attempt demo; bracketed by demos[0] 1.0176 and demos[2] "
                  "successful re-grasp 0.9469",
        "allowed": True},
    "BOTTLE_PX": {
        "source": "pack keyframes/demo0_t0000.png and demo1_t0000.png: centroid of the "
                  "connected dark blob (max channel < 60) of the standing bottle, "
                  "(55.6,58.4) and (54.3,59.5) in the native 128x128 render",
        "allowed": True},
    # ---- release reference pose (paired, per demo, with RACK_PX) ---------
    "RELEASE_REF": {
        "source": "pack.json demos[1].keyframes t=157 ee=[-0.1924,-0.2627,1.1910] and "
                  "demos[2].keyframes t=169 ee=[-0.1888,-0.2508,1.1926]: the two "
                  "gripper-open transitions that placed the bottle and needed no "
                  "follow-up adjustment (demo0's t=150 release was immediately "
                  "re-grasped at t=182, so it is excluded)",
        "allowed": True},
    "RELEASE_Z": {
        "source": "pack.json demos[1] t=157 ee[2]=1.1910 and demos[2] t=169 ee[2]=1.1926",
        "allowed": True},
    "RACK_PX": {
        "source": "pack keyframes/demo1_t0000.png and demo2_t0000.png: centroid of the "
                  "wood-coloured mask inside the fixed above-horizon window, "
                  "(23.882,30.042) and (25.725,30.064) in the native 128x128 render",
        "allowed": True},
    # ---- orientations ----------------------------------------------------
    "TILT_GRASP_DEG": {
        "source": "pack.json ee_path6 axis-angle at the close transitions: tool z axis "
                  "(0.10,-0.844,-0.526)/(0.069,-0.832,-0.551) = 58.3/56.6 deg from "
                  "vertical toward -y; 55 deg is their working centre",
        "allowed": True},
    "TILT_RELEASE_DEG": {
        "source": "pack.json ee_path6 axis-angle at the open transitions: tool z axis "
                  "tilts of 11.5/1.4/13.4 deg from vertical toward -y",
        "allowed": True},
    "TOOL_Y_IS_WORLD_X": {
        "source": "pack.json ee_path6 axis-angle: the tool y (finger-opening) axis is "
                  "(0.99,0.13,-0.01)/(1.00,0.07,0.01)/(1.00,0.03,0.08) at every close "
                  "transition, i.e. the fingers straddle the bottle along world x",
        "allowed": True},
    # ---- path shape ------------------------------------------------------
    "APPROACH_L": {
        "source": "pack.json demos[1].ee_path6 t=50 -> t=80 displacement "
                  "(0.043,0.091,0.082), ~0.13 m travelled along the tool axis into "
                  "the grasp; 0.10 m is used as the stand-off",
        "allowed": True},
    "LIFT_DZ": {
        "source": "pack.json demos[1].ee_path6 t=80 (z=0.972) -> t=100 (z=1.091), a "
                  "straight +0.119 m lift at unchanged tilt",
        "allowed": True},
    "CARRY_WP": {
        "source": "pack.json demos[1].ee_path6 carry waypoints t=110 "
                  "(-0.192,-0.126,1.185, tilt 46 deg) and t=130 "
                  "(-0.130,-0.229,1.251, tilt 16 deg), expressed as offsets from "
                  "that demo's own grasp and release points",
        "allowed": True},
    "RELEASE_CLEAR_DZ": {
        "source": "pack.json demos[1].ee_path6 t=130 -> t=150 descends 0.061 m onto "
                  "the rack; 0.005 m of extra clearance is kept at the open so the "
                  "bottle is never pressed into the slats",
        "allowed": True},
    # ---- perception thresholds ------------------------------------------
    "DARK_MAX": {
        "source": "pack keyframe pixel values: the bottle body reads 3-8 on every "
                  "channel while the table behind it reads 139-164, so any cut in "
                  "between isolates it; 60 is used",
        "allowed": True},
    "WOOD_MASK": {
        "source": "pack keyframe pixel values: rack slats (124,102,79)/(98,81,64) vs "
                  "table (188,172,154) vs back wall (113,111,107); the rack is the "
                  "only R>G>B, R-B>24, 80<R<175 surface above the table horizon",
        "allowed": True},
    "BOTTLE_WIN": {
        "source": "pack keyframes: the bottle blob occupies u[52..59] v[47..66] in all "
                  "three 128x128 keyframes; the window u[47..69] v[42..80] brackets it "
                  "while excluding the dark cabinet (u<=47) and the dark bowl",
        "allowed": True},
    "RACK_WIN": {
        "source": "pack keyframes: the rack occupies u[6..44] v[13..>43] and the table "
                  "horizon is at v~46, so u[0..48] v[8..44] contains the rack and "
                  "nothing else that passes WOOD_MASK",
        "allowed": True},
    "PX_SCALE": {
        "source": "generic camera mechanics: FairApi cam_high is 512x512 and the pack "
                  "keyframes are the same camera at 128x128, so u512 = 4*u128 + 1.5 "
                  "and a 4x4 box average of the live frame reproduces the keyframe "
                  "sampling",
        "allowed": True},
    "SHIFT_CLAMP": {
        "source": "pack cross-demo spread: the bottle blob centroid moves 1.3 px and "
                  "the rack centroid 1.8 px across the three layouts (~1-2 cm), so a "
                  "correction beyond 0.10 m (bottle) / 0.05 m (rack) can only be a "
                  "mis-detection and is clamped",
        "allowed": True},
    # ---- grasp verification / retry -------------------------------------
    "HOLD_EFFORT_MIN": {
        "source": "FairApi surface contract: api.gripper()['effort'] is 3.0 iff holding",
        "allowed": True},
    "HOLD_WIDTH_MIN": {
        "source": "pack.json gripper_state at the holding keyframes sums to "
                  "0.018/0.031/0.045 m, versus 0.002 m when demos[2] closed on empty "
                  "air at t=102",
        "allowed": True},
    "RETRY_X_NUDGE": {
        "source": "pack.json demos[2]: the close at t=79 with ee x=-0.1456 caught "
                  "nothing (gripper_state 0.0011/-0.0012 at t=102) while every "
                  "successful close sits at x=-0.198..-0.214, i.e. the pack's only "
                  "recorded miss was on the +x side of the bottle",
        "allowed": True},
}

# ---------------------------------------------------------------- constants
# (demo pixel, demo end-effector xy) pairs -- see PROVENANCE.
GRASP_PAIRS = (((55.6, 58.4), (-0.2098, -0.0815)),
               ((54.3, 59.5), (-0.2139, -0.0685)))
RELEASE_PAIRS = (((23.882, 30.042), (-0.1924, -0.2627)),
                 ((25.725, 30.064), (-0.1888, -0.2508)))

GRASP_Z = 0.985
RELEASE_Z = 1.1918
RELEASE_CLEAR_DZ = 0.005

TILT_GRASP_DEG = 55.0
TILT_CARRY1_DEG = 45.0
TILT_CARRY2_DEG = 18.0
TILT_RELEASE_DEG = 8.0

APPROACH_L = 0.10
LIFT_DZ = 0.115
STAGE_Z = 1.10

DARK_MAX = 60
BOTTLE_WIN = (42, 80, 47, 70)          # v0, v1, u0, u1  (128-space)
BOTTLE_EXPECT = (55.0, 59.0)
BOTTLE_MIN_PX = 25
BOTTLE_MAX_PX = 500
RACK_WIN = (8, 44, 0, 48)
RACK_MIN_PX = 250

BOTTLE_SHIFT_CLAMP = 0.10
RACK_SHIFT_CLAMP = 0.05

HOLD_EFFORT_MIN = 2.0
HOLD_WIDTH_MIN = 0.006
RETRY_X_NUDGE = -0.020
RETRY_Z_NUDGE = -0.015

OPEN_W = 0.040
CLOSE_W = 0.000


# ------------------------------------------------------------------ helpers
def tool_rot(tilt_deg):
    """Tool frame with the finger-opening axis along world +x and the tool axis
    tipped `tilt_deg` from straight-down toward -y (the pack's grasp family)."""
    t = math.radians(tilt_deg)
    z_ax = np.array([0.0, -math.sin(t), -math.cos(t)])
    y_ax = np.array([1.0, 0.0, 0.0])
    x_ax = np.cross(y_ax, z_ax)
    return np.column_stack([x_ax, y_ax, z_ax])


def small_rgb(rgb):
    """4x4 box average of the 512x512 frame -> the 128x128 sampling the pack
    keyframes were rendered at."""
    a = np.asarray(rgb, dtype=np.float64)
    h, w = a.shape[0], a.shape[1]
    fh, fw = h // 128, w // 128
    if fh < 1 or fw < 1:
        return a
    a = a[:fh * 128, :fw * 128]
    return a.reshape(128, fh, 128, fw, a.shape[2]).mean(axis=(1, 3))


def px_to_full(u_small, v_small, shape):
    """128-space pixel -> full-resolution pixel of the same camera."""
    fh = shape[0] / 128.0
    fw = shape[1] / 128.0
    return (u_small + 0.5) * fw - 0.5, (v_small + 0.5) * fh - 0.5


def components(mask):
    """Connected components (4-neighbourhood) of a small boolean mask."""
    h, w = mask.shape
    seen = np.zeros((h, w), dtype=bool)
    out = []
    for sy in range(h):
        for sx in range(w):
            if not mask[sy, sx] or seen[sy, sx]:
                continue
            stack = [(sy, sx)]
            seen[sy, sx] = True
            pts = []
            while stack:
                y, x = stack.pop()
                pts.append((y, x))
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        stack.append((ny, nx))
            out.append(np.array(pts))
    return out


def find_bottle_px(small):
    """Centroid (u,v) in 128-space of the standing dark bottle, or None."""
    v0, v1, u0, u1 = BOTTLE_WIN
    dark = small.max(axis=2) < DARK_MAX
    win = np.zeros_like(dark)
    win[v0:v1, u0:u1] = dark[v0:v1, u0:u1]
    best, best_d = None, 1e9
    for pts in components(win):
        n = len(pts)
        if n < BOTTLE_MIN_PX or n > BOTTLE_MAX_PX:
            continue
        ys, xs = pts[:, 0], pts[:, 1]
        hgt = ys.max() - ys.min() + 1
        wid = xs.max() - xs.min() + 1
        if hgt < 8 or hgt < 1.6 * wid:
            continue
        cu, cv = float(xs.mean()), float(ys.mean())
        d = math.hypot(cu - BOTTLE_EXPECT[0], cv - BOTTLE_EXPECT[1])
        if d < best_d and d < 14.0:
            best, best_d = (cu, cv, pts), d
    return best


def find_rack_px(small):
    """Centroid (u,v) in 128-space of the wood rack above the table horizon."""
    v0, v1, u0, u1 = RACK_WIN
    r = small[:, :, 0]
    g = small[:, :, 1]
    b = small[:, :, 2]
    wood = (r > 80) & (r < 175) & ((r - b) > 24) & (r >= g) & (g >= b)
    win = np.zeros_like(wood)
    win[v0:v1, u0:u1] = wood[v0:v1, u0:u1]
    if int(win.sum()) < RACK_MIN_PX:
        return None
    ys, xs = np.nonzero(win)
    return float(xs.mean()), float(ys.mean()), np.stack([ys, xs], axis=1)


def ray_to_plane(frame, u_full, v_full, z_plane):
    """Base-frame point where the camera ray through (u,v) meets z = z_plane.
    Uses only the live intrinsics/extrinsics via deproject, so no image-frame
    convention has to be assumed."""
    try:
        p = np.asarray(frame.deproject(float(u_full), float(v_full)), dtype=np.float64)
    except Exception:
        return None
    if p.shape[0] < 3 or not np.all(np.isfinite(p)):
        return None
    c = np.asarray(frame.t_base_cam, dtype=np.float64)[:3, 3]
    d = p - c
    if abs(d[2]) < 1e-6:
        return None
    t = (z_plane - c[2]) / d[2]
    if t <= 0.0:
        return None
    return c + t * d


def sample_plane_z(frame, pts_small, shape, limit=40):
    """Median base-frame height of a set of 128-space mask pixels."""
    if pts_small is None or len(pts_small) == 0:
        return None
    idx = np.linspace(0, len(pts_small) - 1, min(limit, len(pts_small)))
    zs = []
    for i in idx:
        vy, ux = pts_small[int(round(i))]
        uf, vf = px_to_full(float(ux), float(vy), shape)
        try:
            p = np.asarray(frame.deproject(uf, vf), dtype=np.float64)
        except Exception:
            continue
        if p.shape[0] >= 3 and np.all(np.isfinite(p)):
            zs.append(float(p[2]))
    if len(zs) < 5:
        return None
    return float(np.median(np.array(zs)))


def differential_target(frame, pairs, live_uv, z_plane, clamp, shape):
    """mean_i( ee_i - world(demo_px_i) ) + world(live_px), all cast onto the
    same plane so estimator bias cancels."""
    lu, lv = px_to_full(live_uv[0], live_uv[1], shape)
    live_p = ray_to_plane(frame, lu, lv, z_plane)
    if live_p is None:
        return None
    offs = []
    for (du, dv), ee in pairs:
        fu, fv = px_to_full(du, dv, shape)
        dp = ray_to_plane(frame, fu, fv, z_plane)
        if dp is None:
            continue
        offs.append(np.array(ee, dtype=np.float64) - dp[:2])
    if not offs:
        return None
    target = np.mean(np.array(offs), axis=0) + live_p[:2]
    nominal = np.mean(np.array([np.array(ee) for _, ee in pairs]), axis=0)
    delta = target - nominal
    n = float(np.linalg.norm(delta))
    if n > clamp:
        delta = delta * (clamp / n)
    return nominal + delta


def nominal_xy(pairs):
    return np.mean(np.array([np.array(ee, dtype=np.float64) for _, ee in pairs]), axis=0)


def holding(api):
    try:
        g = api.gripper()
    except Exception:
        return True
    try:
        if float(g.get("effort", 0.0)) >= HOLD_EFFORT_MIN:
            return True
        return float(g.get("width_m", 0.0)) >= HOLD_WIDTH_MIN
    except Exception:
        return True


# --------------------------------------------------------------------- main
def run(api):
    r_grasp = tool_rot(TILT_GRASP_DEG)
    r_c1 = tool_rot(TILT_CARRY1_DEG)
    r_c2 = tool_rot(TILT_CARRY2_DEG)
    r_rel = tool_rot(TILT_RELEASE_DEG)
    sin_t = math.sin(math.radians(TILT_GRASP_DEG))
    cos_t = math.cos(math.radians(TILT_GRASP_DEG))

    grasp_xy = nominal_xy(GRASP_PAIRS)
    rel_xy = nominal_xy(RELEASE_PAIRS)

    # ---- one look, before anything moves (the demos' t=0 view is clear) ----
    try:
        frame = api.capture("cam_high")
        rgb = np.asarray(frame.rgb)
        small = small_rgb(rgb)
        shape = rgb.shape

        b = find_bottle_px(small)
        if b is not None:
            zb = sample_plane_z(frame, b[2], shape)
            if zb is not None:
                t = differential_target(frame, GRASP_PAIRS, (b[0], b[1]), zb,
                                        BOTTLE_SHIFT_CLAMP, shape)
                if t is not None:
                    grasp_xy = t
                    api.log("bottle px=(%.2f,%.2f) -> grasp xy=(%.3f,%.3f)"
                            % (b[0], b[1], t[0], t[1]))

        k = find_rack_px(small)
        if k is not None:
            zk = sample_plane_z(frame, k[2], shape)
            if zk is not None:
                t = differential_target(frame, RELEASE_PAIRS, (k[0], k[1]), zk,
                                        RACK_SHIFT_CLAMP, shape)
                if t is not None:
                    rel_xy = t
                    api.log("rack px=(%.2f,%.2f) -> release xy=(%.3f,%.3f)"
                            % (k[0], k[1], t[0], t[1]))
    except Exception as exc:            # perception is optional; the pack pose is not
        api.log("perception fell back to pack pose: %r" % (exc,))

    gx, gy = float(grasp_xy[0]), float(grasp_xy[1])
    rx, ry = float(rel_xy[0]), float(rel_xy[1])

    # ---- grasp (at most two attempts, gated on the gripper's own sensors) --
    api.grip(OPEN_W)
    gz = GRASP_Z
    got = False
    for attempt in range(2):
        if attempt == 1:
            gx += RETRY_X_NUDGE
            gz += RETRY_Z_NUDGE
            api.grip(OPEN_W)
            api.move([gx, gy + APPROACH_L * sin_t + 0.04, STAGE_Z + 0.04],
                     rotation=r_grasp, seconds=1.6)
        api.move([gx, gy + APPROACH_L * sin_t + 0.03, STAGE_Z],
                 rotation=r_grasp, seconds=2.2 if attempt == 0 else 1.2)
        api.move([gx, gy + APPROACH_L * sin_t, gz + APPROACH_L * cos_t],
                 rotation=r_grasp, seconds=1.3)
        api.move([gx, gy, gz], rotation=r_grasp, seconds=1.2)
        api.grip(CLOSE_W)
        api.settle(0.4)
        if holding(api):
            got = True
            api.log("grasp attempt %d holding" % attempt)
            break
        api.log("grasp attempt %d empty" % attempt)

    if not got:
        # Nothing detected in the fingers; keep the fingers shut and still run
        # the placement leg -- an unfelt-but-real grip is the likelier of the
        # two remaining possibilities, and an empty gripper loses nothing.
        api.grip(CLOSE_W)

    # ---- lift, carry, un-tilt (this is what lays the bottle over) ---------
    api.move([gx, gy, gz + LIFT_DZ], rotation=r_grasp, seconds=1.2)
    api.move([gx + 0.010, gy - 0.060, 1.185], rotation=r_c1, seconds=1.5)
    api.move([rx + 0.045, ry + 0.020, 1.250], rotation=r_c2, seconds=1.5)

    rz = RELEASE_Z + RELEASE_CLEAR_DZ + (gz - GRASP_Z)
    api.move([rx, ry, rz], rotation=r_rel, seconds=1.4)
    api.settle(0.3)
    api.grip(OPEN_W)
    api.settle(0.3)

    # ---- retreat straight up, clear of the slats -------------------------
    api.move([rx, ry, rz + 0.075], rotation=r_rel, seconds=1.0)
    api.settle(0.2)
