"""abl_c2 / cell ablC_goal_put_cream_cheese_in_bowl  (variant C -- no verification)

Intent: "put the cream cheese in the bowl".

Everything below is derived from packs/ablC_goal_put_cream_cheese_in_bowl/
(pack.json keyframes + keyframes/*.png pixel statistics) and from generic
pinhole-camera / position-controller mechanics.  ZERO episodes were run while
authoring this file, so every constant is either a pack measurement or a
generic-mechanics quantity, and the policy is written closed-loop wherever the
pack lets me argue for it.

Scene facts read off the three demo keyframe sets:
  * the cream cheese is the only blue-ish object in the whole image: a mask
    (B-R>=12, B>=45, max<=185) fires on exactly one ~50px blob in every one of
    the 12 keyframes and nowhere else.
  * the bowl is the mottled achromatic grey bowl (mean value ~105, per-pixel
    value std ~34).  The nearby plate is achromatic too but uniform and bright
    (mean ~165, std ~5), so value-mean/value-std separate them cleanly.
  * in the last keyframe of every demo the blue blob sits on top of the bowl
    blob -- that is the direct evidence for which grey object is "the bowl".
  * demo gripper-close happens at ee z ~0.911 and demo gripper-open (release)
    at ee z ~0.970, with the carry apex at ~1.04-1.08.
"""

import numpy as np

# ----------------------------------------------------------------- constants
# demo0/1/2 keyframe with gripper_cmd flipping to +1 (close):
#   ee z = 0.9104 / 0.9205 / 0.9106   -> grasp height
# demo0/1/2 keyframe with gripper_cmd flipping back to -1 (open):
#   ee z = 0.9592 / 0.9731 / 0.9785   -> release height
#   ee xy = (-0.0961,-0.0246) / (-0.0453,-0.0099) / (-0.0835,-0.0016)
# demo0/1/2 grasp ee xy = (-0.0213,0.1119) / (-0.0134,0.1367) / (-0.0479,0.1521)
# demo0/1/2 t=0 ee     = (-0.2054,0.0076,1.185)/(-0.2052,0.0184,1.1605)/
#                        (-0.2045,0.0028,1.1736)
Z_GRASP = 0.911
Z_RELEASE = 0.970
Z_CARRY = 1.070
Z_HOVER = 1.030
Z_RETREAT = 1.060
HOME_EE = (-0.2050, 0.0096, 1.1730)

CHEESE_PRIOR_XY = (-0.0275, 0.1336)
BOWL_PRIOR_XY = (-0.0750, -0.0110)
CHEESE_WIN = (0.20, 0.20)
BOWL_WIN = (0.13, 0.115)

GRIP_OPEN_M = 0.080
GRIP_CLOSE_M = 0.000
HOLD_EFFORT = 2.5
HOLD_WIDTH_MIN = 0.008
HOLD_WIDTH_MAX = 0.062

BLUE_DR = 12
BLUE_BMIN = 45
BLUE_VMAX = 185
BLUE_MIN_AREA = 40

GREY_CHROMA_MAX = 16
GREY_V_MIN = 35
GREY_V_MAX = 158
GREY_VSTD_MIN = 10.0
GREY_MIN_AREA = 150

TABLE_CLEARANCE = 0.012
OBJ_MAX_HEIGHT = 0.30
IN_BOWL_TOL = 0.090
MOVE_TOL = 0.008
TABLE_WARM_DR = 22
LABEL_MAX_PIXELS = 60000

PROVENANCE = {
    "Z_GRASP": {
        "source": "pack.json demos[*].keyframes: ee z at the gripper_cmd->+1 "
                  "(close) keyframe = 0.9104/0.9205/0.9106 m",
        "allowed": True},
    "Z_RELEASE": {
        "source": "pack.json demos[*].keyframes: ee z at the gripper_cmd->-1 "
                  "(open/release) keyframe = 0.9592/0.9731/0.9785 m",
        "allowed": True},
    "Z_CARRY": {
        "source": "pack.json demos[*].ee_path6: carry apex z between grasp and "
                  "release = 1.0415/1.0790/1.0510 m",
        "allowed": True},
    "Z_HOVER": {
        "source": "pack.json demos[*].ee_path6: last pre-descent waypoint z "
                  "above the cream cheese = 1.0243/1.0816/0.9926 m",
        "allowed": True},
    "Z_RETREAT": {
        "source": "pack.json demos[*].keyframes: final keyframe ee z after "
                  "release = 1.0118/1.0441/1.0404 m",
        "allowed": True},
    "HOME_EE": {
        "source": "pack.json demos[*].keyframes[0].ee = "
                  "(-0.2054,0.0076,1.1850)/(-0.2052,0.0184,1.1605)/"
                  "(-0.2045,0.0028,1.1736); used only as an out-of-frame park "
                  "pose for re-perception",
        "allowed": True},
    "CHEESE_PRIOR_XY": {
        "source": "pack.json demos[*] grasp-keyframe ee xy mean of "
                  "(-0.0213,0.1119)/(-0.0134,0.1367)/(-0.0479,0.1521)",
        "allowed": True},
    "BOWL_PRIOR_XY": {
        "source": "pack.json demos[*] release-keyframe ee xy mean of "
                  "(-0.0961,-0.0246)/(-0.0453,-0.0099)/(-0.0835,-0.0016)",
        "allowed": True},
    "CHEESE_WIN": {
        "source": "pack keyframe images: cream-cheese blob centroid moves "
                  "<=3.3 px (~3 cm) across the three demos; window is a ~6x "
                  "sanity gate around CHEESE_PRIOR_XY",
        "allowed": True},
    "BOWL_WIN": {
        "source": "pack keyframe images: bowl blob centroid moves <=2.5 px "
                  "(~2 cm) across the three demos; window is a ~5x sanity gate "
                  "around BOWL_PRIOR_XY, sized to sit inside the ~0.10 m "
                  "bowl-to-plate separation implied by the keyframes",
        "allowed": True},
    "GRIP_OPEN_M": {
        "source": "FairApi spec (api.grip: >=0.025 opens) + pack.json "
                  "gripper_state at the open keyframes (0.0362..0.0405 per "
                  "finger, i.e. ~0.079 m span)",
        "allowed": True},
    "GRIP_CLOSE_M": {
        "source": "FairApi spec (api.grip: <0.025 closes)",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi spec: api.gripper()['effort'] == 3.0 iff holding",
        "allowed": True},
    "HOLD_WIDTH_MIN": {
        "source": "pack.json gripper_state at the release keyframe while the "
                  "cream cheese is held: 0.0216/0.0211/0.0237 per finger "
                  "(~0.043 m grasped width); lower bracket, generous",
        "allowed": True},
    "HOLD_WIDTH_MAX": {
        "source": "pack.json gripper_state at the release keyframe while the "
                  "cream cheese is held: 0.0216/0.0211/0.0237 per finger "
                  "(~0.043 m grasped width); upper bracket, generous",
        "allowed": True},
    "BLUE_DR": {
        "source": "pack keyframes/*.png pixel statistics: the cream-cheese box "
                  "is the only pixel population with B-R>=12 in any of the 12 "
                  "keyframes",
        "allowed": True},
    "BLUE_BMIN": {
        "source": "pack keyframes/*.png pixel statistics: cream-cheese blue "
                  "channel is 45..100 on the box body",
        "allowed": True},
    "BLUE_VMAX": {
        "source": "pack keyframes/*.png pixel statistics: cream-cheese box is "
                  "dark (max channel <=185); the warm table is brighter",
        "allowed": True},
    "BLUE_MIN_AREA": {
        "source": "pack keyframes/*.png: cream-cheese blob is ~50 px at "
                  "128x128, i.e. ~800 px at the 512x512 FairApi capture; 40 px "
                  "is a conservative floor",
        "allowed": True},
    "GREY_CHROMA_MAX": {
        "source": "pack keyframes/*.png: bowl pixels are achromatic "
                  "(max-min<=14); the wood table has max-min ~33",
        "allowed": True},
    "GREY_V_MIN": {
        "source": "pack keyframes/*.png: bowl blob value mean 104-107 with std "
                  "33-36; 35 excludes the near-black wine bottle beside it",
        "allowed": True},
    "GREY_V_MAX": {
        "source": "pack keyframes/*.png: bowl blob value mean 104-107; 158 "
                  "excludes the bright plate (value mean 164-166)",
        "allowed": True},
    "GREY_VSTD_MIN": {
        "source": "pack keyframes/*.png: bowl blob value std 33-36 (mottled) "
                  "versus ~5 for the uniform plate",
        "allowed": True},
    "GREY_MIN_AREA": {
        "source": "pack keyframes/*.png: bowl blob is ~165 px at 128x128, i.e. "
                  "~2600 px at 512x512; 150 px is a conservative floor",
        "allowed": True},
    "TABLE_CLEARANCE": {
        "source": "generic camera/geometry mechanics: margin separating "
                  "table-plane depth returns from objects standing on it, "
                  "relative to the runtime median table height",
        "allowed": True},
    "OBJ_MAX_HEIGHT": {
        "source": "generic camera/geometry mechanics: upper height gate that "
                  "rejects the wall and the robot body from the object cloud",
        "allowed": True},
    "IN_BOWL_TOL": {
        "source": "pack keyframes: in the final keyframe of every demo the "
                  "cream-cheese blob centroid sits within ~3 px (~3 cm) of the "
                  "bowl blob centroid; 0.09 m is a conservative 3x gate used "
                  "only to decide whether a retry is warranted",
        "allowed": True},
    "LABEL_MAX_PIXELS": {
        "source": "generic mechanics: wallclock guard on the pure-numpy "
                  "connected-components pass; a mask larger than this means "
                  "the colour/geometry gate failed, so we report no detection",
        "allowed": True},
    "MOVE_TOL": {
        "source": "generic position-controller mechanics: residual threshold "
                  "for deciding a commanded api.move has converged",
        "allowed": True},
    "TABLE_WARM_DR": {
        "source": "pack keyframes/*.png: table wood pixels have R-B ~ 33 "
                  "(e.g. [191,174,157], [157,141,124]) versus <=16 for every "
                  "achromatic object",
        "allowed": True},
}

TABLE_WARM_DR = 22


# ------------------------------------------------------------------ geometry
def _deproject_all(frame):
    """Vectorised base-frame point cloud for a FairFrame.

    Builds the cloud under both the OpenCV (+z forward, +y down) and the
    OpenGL/MuJoCo (-z forward, +y up) camera-axis conventions and returns the
    one that agrees with the authoritative api-side frame.deproject() on a
    handful of sample pixels.  Generic pinhole mechanics only.
    """
    depth = np.asarray(frame.depth, dtype=np.float64)
    if depth.ndim == 3:
        depth = depth[..., 0]
    H, W = depth.shape
    K = np.asarray(frame.intrinsics, dtype=np.float64)
    T = np.asarray(frame.t_base_cam, dtype=np.float64)
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float64),
                         np.arange(H, dtype=np.float64))
    xn = (uu - K[0, 2]) / K[0, 0]
    yn = (vv - K[1, 2]) / K[1, 1]
    R = T[:3, :3]
    t = T[:3, 3]

    def build(sy, sz):
        cam = np.stack([xn * depth, sy * yn * depth, sz * depth], axis=-1)
        return cam @ R.T + t

    cands = [build(1.0, 1.0), build(-1.0, -1.0)]

    # probe pixels spread over the frame, on flat-ish areas
    probes = []
    for pv in (0.35, 0.55, 0.75):
        for pu in (0.3, 0.5, 0.7):
            probes.append((int(pu * W), int(pv * H)))
    errs = [0.0, 0.0]
    used = 0
    for (pu, pv) in probes:
        try:
            ref = np.asarray(frame.deproject(pu, pv), dtype=np.float64)
        except Exception:
            continue
        if not np.all(np.isfinite(ref)):
            continue
        used += 1
        for i, c in enumerate(cands):
            errs[i] += float(np.linalg.norm(c[pv, pu] - ref))
    if used == 0:
        return cands[1], False
    best = 0 if errs[0] <= errs[1] else 1
    ok = (errs[best] / used) < 0.02
    return cands[best], ok


def _label(mask):
    """4-connected components of a boolean mask -> list of (ys, xs)."""
    H, W = mask.shape
    if int(mask.sum()) > LABEL_MAX_PIXELS:
        # a mask this large means the colour/geometry gate did not bite; treat
        # it as "no confident detection" rather than burning wallclock on it
        return []
    lab = np.zeros((H, W), dtype=np.int32)
    ys, xs = np.nonzero(mask)
    comps = []
    cur = 0
    for sy, sx in zip(ys.tolist(), xs.tolist()):
        if lab[sy, sx]:
            continue
        cur += 1
        stack = [(sy, sx)]
        lab[sy, sx] = cur
        py = []
        px = []
        while stack:
            y, x = stack.pop()
            py.append(y)
            px.append(x)
            if y > 0 and mask[y - 1, x] and not lab[y - 1, x]:
                lab[y - 1, x] = cur
                stack.append((y - 1, x))
            if y + 1 < H and mask[y + 1, x] and not lab[y + 1, x]:
                lab[y + 1, x] = cur
                stack.append((y + 1, x))
            if x > 0 and mask[y, x - 1] and not lab[y, x - 1]:
                lab[y, x - 1] = cur
                stack.append((y, x - 1))
            if x + 1 < W and mask[y, x + 1] and not lab[y, x + 1]:
                lab[y, x + 1] = cur
                stack.append((y, x + 1))
        comps.append((np.asarray(py), np.asarray(px)))
    return comps


def _channels(rgb):
    a = np.asarray(rgb).astype(np.int16)
    R = a[..., 0]
    G = a[..., 1]
    B = a[..., 2]
    mx = a.max(axis=2)
    mn = a.min(axis=2)
    V = a.mean(axis=2)
    return R, G, B, mx, mn, V


def _table_height(cloud, valid, R, B):
    warm = valid & ((R - B) >= TABLE_WARM_DR)
    if warm.sum() < 200:
        warm = valid
    z = cloud[..., 2][warm]
    if z.size == 0:
        return None
    return float(np.median(z))


# ---------------------------------------------------------------- perception
class Scene(object):
    def __init__(self):
        self.cheese = None      # (x, y)
        self.cheese_top = None
        self.bowl = None        # (x, y)
        self.table_z = None
        self.cheese_seen = False
        self.bowl_seen = False


def _blob_xy(cloud, ys, xs):
    pts = cloud[ys, xs]
    good = np.all(np.isfinite(pts), axis=1)
    pts = pts[good]
    if pts.shape[0] < 5:
        return None, None
    return (float(np.median(pts[:, 0])), float(np.median(pts[:, 1]))), \
        float(np.percentile(pts[:, 2], 85))


def perceive(api, want_bowl=True):
    """cam_high perception of the cream cheese and the bowl, in base frame."""
    sc = Scene()
    frame = api.capture("cam_high")
    cloud, conv_ok = _deproject_all(frame)
    R, G, B, mx, mn, V = _channels(frame.rgb)
    depth = np.asarray(frame.depth, dtype=np.float64)
    if depth.ndim == 3:
        depth = depth[..., 0]
    valid = np.isfinite(depth) & (depth > 1e-4) & np.isfinite(cloud[..., 2])

    tz = _table_height(cloud, valid, R, B)
    if tz is None:
        return sc
    sc.table_z = tz
    if not conv_ok:
        api.log("deproject convention check weak; using best-fit cloud")

    Z = cloud[..., 2]
    on_table = valid & (Z > tz + TABLE_CLEARANCE) & (Z < tz + OBJ_MAX_HEIGHT)

    def near(prior, win):
        return (np.abs(cloud[..., 0] - prior[0]) <= win[0]) & \
               (np.abs(cloud[..., 1] - prior[1]) <= win[1])

    # -- cream cheese: the unique blue-ish population -----------------------
    blue = ((B - R) >= BLUE_DR) & (B >= BLUE_BMIN) & (mx <= BLUE_VMAX)
    cand = blue & valid & near(CHEESE_PRIOR_XY, CHEESE_WIN)
    best = None
    for ys, xs in _label(cand):
        if ys.size < BLUE_MIN_AREA:
            continue
        xy, top = _blob_xy(cloud, ys, xs)
        if xy is None:
            continue
        if best is None or ys.size > best[0]:
            best = (ys.size, xy, top)
    if best is not None:
        sc.cheese = best[1]
        sc.cheese_top = best[2]
        sc.cheese_seen = True

    if not want_bowl:
        return sc

    # -- bowl: mottled achromatic mid-value blob near the release prior ------
    grey = ((mx - mn) <= GREY_CHROMA_MAX) & (V >= GREY_V_MIN) & \
        (V <= GREY_V_MAX)
    cand = grey & on_table & near(BOWL_PRIOR_XY, BOWL_WIN)
    bb = None
    for ys, xs in _label(cand):
        if ys.size < GREY_MIN_AREA:
            continue
        if float(np.std(V[ys, xs])) < GREY_VSTD_MIN:
            continue          # uniform bright plate, not the mottled bowl
        xy, _top = _blob_xy(cloud, ys, xs)
        if xy is None:
            continue
        d = np.hypot(xy[0] - BOWL_PRIOR_XY[0], xy[1] - BOWL_PRIOR_XY[1])
        if bb is None or d < bb[0]:
            bb = (d, xy)
    if bb is not None:
        sc.bowl = bb[1]
        sc.bowl_seen = True
    return sc


def _wrist_refine(api, kind, coarse_xy, max_corr):
    """Refine an xy estimate from the wrist camera while hovering over it."""
    try:
        frame = api.capture("cam_arm_wrist")
        cloud, _ok = _deproject_all(frame)
        R, G, B, mx, mn, V = _channels(frame.rgb)
        depth = np.asarray(frame.depth, dtype=np.float64)
        if depth.ndim == 3:
            depth = depth[..., 0]
        valid = np.isfinite(depth) & (depth > 1e-4) & np.isfinite(cloud[..., 2])
        if kind == "cheese":
            m = ((B - R) >= BLUE_DR) & (B >= BLUE_BMIN) & (mx <= BLUE_VMAX)
            min_area = BLUE_MIN_AREA
        else:
            m = ((mx - mn) <= GREY_CHROMA_MAX) & (V >= GREY_V_MIN) & \
                (V <= GREY_V_MAX) & ((B - R) < BLUE_DR)
            min_area = GREY_MIN_AREA
        m = m & valid
        # only look near the image centre: that is what we are hovering over
        H, W = m.shape
        keep = np.zeros_like(m)
        keep[int(0.20 * H):int(0.80 * H), int(0.20 * W):int(0.80 * W)] = True
        m = m & keep
        best = None
        for ys, xs in _label(m):
            if ys.size < min_area:
                continue
            if kind == "bowl" and float(np.std(V[ys, xs])) < GREY_VSTD_MIN:
                continue
            xy, _t = _blob_xy(cloud, ys, xs)
            if xy is None:
                continue
            if best is None or ys.size > best[0]:
                best = (ys.size, xy)
        if best is None:
            return coarse_xy, False
        xy = best[1]
        corr = np.hypot(xy[0] - coarse_xy[0], xy[1] - coarse_xy[1])
        if corr > max_corr:
            api.log("wrist %s correction %.3f m rejected" % (kind, corr))
            return coarse_xy, False
        api.log("wrist %s correction %.3f m accepted" % (kind, corr))
        return xy, True
    except Exception as exc:                       # perception must never kill
        api.log("wrist refine (%s) failed: %r" % (kind, exc))
        return coarse_xy, False


# ------------------------------------------------------------------- motion
def goto(api, xyz, seconds=2.0, tries=2, tol=MOVE_TOL):
    target = np.asarray(xyz, dtype=np.float64)
    api.move([float(target[0]), float(target[1]), float(target[2])],
             seconds=seconds)
    d = float(np.linalg.norm(np.asarray(api.eef(), dtype=np.float64) - target))
    n = 0
    while d > tol and n < tries:
        api.move([float(target[0]), float(target[1]), float(target[2])],
                 seconds=0.8)
        d = float(np.linalg.norm(np.asarray(api.eef(), dtype=np.float64)
                                 - target))
        n += 1
    return d


def holding(api):
    try:
        g = api.gripper()
        try:
            eff = float(g["effort"])
            w = float(g["width_m"])
        except Exception:
            eff = float(getattr(g, "effort", 0.0))
            w = float(getattr(g, "width_m", 0.0))
    except Exception:
        return True    # unreadable sensor: do not trigger a needless regrasp
    return (eff >= HOLD_EFFORT) or (HOLD_WIDTH_MIN <= w <= HOLD_WIDTH_MAX)


def pick(api, xy, table_z):
    """Hover, wrist-refine, descend, close.  Returns (held, xy_used)."""
    z_grasp = Z_GRASP
    if table_z is not None:
        z_grasp = max(Z_GRASP, table_z + 0.005)
    api.grip(GRIP_OPEN_M)
    goto(api, [xy[0], xy[1], Z_HOVER], seconds=2.0)
    xy, _used = _wrist_refine(api, "cheese", xy, 0.05)
    goto(api, [xy[0], xy[1], Z_HOVER], seconds=0.8)
    goto(api, [xy[0], xy[1], z_grasp], seconds=1.5, tol=0.010)
    api.grip(GRIP_CLOSE_M)
    api.settle(0.4)
    if holding(api):
        return True, xy
    # one sensor-driven regrasp: re-look from a low hover, then close again
    api.grip(GRIP_OPEN_M)
    goto(api, [xy[0], xy[1], 1.000], seconds=1.0)
    xy2, ok = _wrist_refine(api, "cheese", xy, 0.06)
    goto(api, [xy2[0], xy2[1], z_grasp], seconds=1.4, tol=0.010)
    api.grip(GRIP_CLOSE_M)
    api.settle(0.4)
    return holding(api), xy2


def place(api, xy):
    goto(api, [xy[0], xy[1], Z_CARRY], seconds=1.6)
    xy, _ok = _wrist_refine(api, "bowl", xy, 0.06)
    goto(api, [xy[0], xy[1], Z_CARRY], seconds=0.8)
    goto(api, [xy[0], xy[1], Z_RELEASE], seconds=1.2, tol=0.010)
    api.grip(GRIP_OPEN_M)
    api.settle(0.5)
    goto(api, [xy[0], xy[1], Z_RETREAT], seconds=1.0)
    return xy


def park(api):
    goto(api, [HOME_EE[0], HOME_EE[1], HOME_EE[2]], seconds=2.0, tol=0.02)


# ---------------------------------------------------------------------- main
def run(api):
    api.log("instruction: %s" % (api.instruction(),))
    api.grip(GRIP_OPEN_M)

    cheese_xy = CHEESE_PRIOR_XY
    bowl_xy = BOWL_PRIOR_XY
    table_z = None
    try:
        sc = perceive(api)
        table_z = sc.table_z
        if sc.cheese_seen:
            cheese_xy = sc.cheese
        if sc.bowl_seen:
            bowl_xy = sc.bowl
        api.log("cam_high: cheese=%s (seen=%s) bowl=%s (seen=%s) table_z=%s"
                % (np.round(cheese_xy, 4).tolist(), sc.cheese_seen,
                   np.round(bowl_xy, 4).tolist(), sc.bowl_seen, table_z))
    except Exception as exc:
        api.log("cam_high perception failed (%r); using pack priors" % (exc,))

    held, cheese_xy = pick(api, cheese_xy, table_z)
    api.log("attempt 1 grasp held=%s at %s"
            % (held, np.round(cheese_xy, 4).tolist()))
    bowl_xy = place(api, bowl_xy)

    # -------- one conservative, sensor-only retry --------------------------
    # No success feedback is available or used.  We simply re-perceive: if the
    # cream cheese is unambiguously NOT at the bowl (>IN_BOWL_TOL away in xy,
    # versus the ~3 cm the demos show for an in-bowl box), the first attempt
    # left it on the table and a second pick-and-place can only help.
    try:
        park(api)
        sc2 = perceive(api, want_bowl=False)
        if sc2.cheese_seen:
            d = float(np.hypot(sc2.cheese[0] - bowl_xy[0],
                               sc2.cheese[1] - bowl_xy[1]))
            api.log("post-place: cheese at %s, %.3f m from bowl"
                    % (np.round(sc2.cheese, 4).tolist(), d))
            if d > IN_BOWL_TOL:
                api.log("retrying pick-and-place")
                held2, xy2 = pick(api, sc2.cheese, sc2.table_z)
                api.log("attempt 2 grasp held=%s" % (held2,))
                place(api, bowl_xy)
        else:
            api.log("post-place: cream cheese not visible; no retry")
    except Exception as exc:
        api.log("post-place check failed (%r); no retry" % (exc,))

    api.log("done")
