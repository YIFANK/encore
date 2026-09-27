"""c2clean / goal_turn_on_stove_pos_k3 -- v3

Mechanism (read off the K=3 pack, direction fixed by a debug-seed A/B):
  the stove's control knob is a small dark disc on the table with a raised
  bar-shaped handle on top.  Approach it straight down with the jaws across
  the bar's narrow axis, close, then TWIST about the tool's own approach axis
  while pressing gently down.

  v2 ran both twist senses back to back on debug seeds 51/53/55/57.  The
  sense that leaves the jaws closed (0.0258 -> 0.0297 m) and rotates the
  handle bar (yaw 0.006 -> 0.893 rad on re-perception) is R_app @ Rz(-theta);
  the opposite sense pries the jaws open (0.0257 -> 0.0541 m) and leaves the
  bar's yaw unchanged.  So TWIST_SIGN = -1.

  The knob's table position moves between scenes (the three demos grasp it at
  x = -0.433 / -0.435 / -0.386, the debug seeds at x = -0.037), so its xy and
  its bar yaw are perceived from cam_high every episode; only the vertical
  offsets and the twist magnitude come from the pack.

  Self-check: the jaws being forced open during the twist is the signature of
  a failed turn, so if the post-twist width exceeds PRY_W the program
  re-seats the grasp and turns again.
"""
import numpy as np

PROVENANCE = {
    "GRASP_DZ": {
        "source": "pack.json demos[*].keyframes[1].ee[2] / ee_path6 -- the eef sits "
                  "at z = 0.9279/0.9306/0.9292 while the jaws are closed on the "
                  "handle; minus the table height 0.9009 measured from cam_high "
                  "depth on debug seeds 51-65",
        "allowed": True},
    "HOVER_DZ": {
        "source": "pack.json demos[*].ee_path6: the approach passes ~0.10-0.13 m "
                  "above the grasp height before the final descent",
        "allowed": True},
    "PRESS_DZ": {
        "source": "pack.json demos[*].actions: dz stays at -0.37..-0.52 (scale "
                  "0.308-0.387) through the entire post-grasp twist phase, i.e. the "
                  "demos keep pressing down while turning",
        "allowed": True},
    "TWIST_RAD": {
        "source": "pack.json ee_path6: grasp-keyframe -> final-keyframe relative "
                  "rotation is 0.93 / 1.08 / 1.50 rad about the tool approach axis",
        "allowed": True},
    "TWIST_SIGN": {
        "source": "debug-seed A/B (program_v2 on seeds 51,53,55,57): the negative "
                  "sense keeps the jaws closed and rotates the re-perceived handle "
                  "bar by 0.89 rad; the positive sense pries the jaws open and "
                  "leaves the bar where it was",
        "allowed": True},
    "DARK_LUM": {
        "source": "debug-seed cam_high RGB: knob pixels have mean luminance 12-25, "
                  "the stove slab 64, the table 160",
        "allowed": True},
    "KNOB_BAND": {
        "source": "debug-seed cam_high depth: the knob's disc top sits at "
                  "table+0.019 m and its handle bar top at table+0.059 m",
        "allowed": True},
    "KNOB_TALL": {
        "source": "debug-seed cam_high depth: the other dark objects in the scene "
                  "(bottle, cabinet) carry dark pixels above table+0.10 m, the knob "
                  "does not",
        "allowed": True},
    "KNOB_MAX_SPAN": {
        "source": "debug-seed cam_high depth: the knob footprint measures "
                  "0.084 x 0.025 m in the handle band; the cabinet component spans "
                  "0.356 m",
        "allowed": True},
    "BAR_DZ": {
        "source": "debug-seed cam_high depth: points above table+0.045 m belong to "
                  "the raised handle bar rather than the disc below it",
        "allowed": True},
    "PRY_W": {
        "source": "debug-seed v2 receipt: a turn that takes hold ends at width "
                  "0.0297 m, a turn that fails ends at 0.0541 m (open width is "
                  "0.0778 m)",
        "allowed": True},
}

GRASP_DZ = 0.028
HOVER_DZ = 0.130
PRESS_DZ = 0.006
TWIST_RAD = 1.55
TWIST_SIGN = -1.0
TWIST_STEPS = 11
DARK_LUM = 48.0
KNOB_BAND = (0.028, 0.078)
KNOB_TALL = 0.095
KNOB_MAX_SPAN = 0.16
BAR_DZ = 0.045
PRY_W = 0.045

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _cloud(frame):
    d = np.asarray(frame.depth, float)
    d = np.where(np.isfinite(d) & (d > 0), d, 0.0)
    K, T = np.asarray(frame.intrinsics, float), np.asarray(frame.t_base_cam, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1) @ T.T
    return P[..., :3], d > 0


def _label(mask):
    """4-connected components (no scipy in the sandbox)."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    seen = np.zeros((h, w), bool)
    for sy, sx in np.argwhere(mask):
        if seen[sy, sx]:
            continue
        cur += 1
        stack = [(int(sy), int(sx))]
        seen[sy, sx] = True
        while stack:
            y, x = stack.pop()
            lab[y, x] = cur
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    stack.append((ny, nx))
    return lab, cur


def find_knob(api, tag="P"):
    """Locate the stove knob: the only compact dark thing whose top lives in the
    handle band and that carries nothing dark above KNOB_TALL."""
    f = api.capture("cam_high")
    P, ok = _cloud(f)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    lum = np.asarray(f.rgb, float).mean(-1)
    ws = ok & (X > -0.85) & (X < 0.45) & (np.abs(Y) < 0.7) & (Z > 0.6) & (Z < 1.5)
    zz = Z[ws & (lum > 90)]
    table = float(np.median(zz[zz < np.percentile(zz, 60)]))

    dark = ws & (lum < DARK_LUM)
    band = dark & (Z > table + KNOB_BAND[0]) & (Z < table + KNOB_BAND[1])
    lab, n = _label(band)
    best = None
    for i in range(1, n + 1):
        s = lab == i
        if s.sum() < 25:
            continue
        xmin, xmax = float(X[s].min()), float(X[s].max())
        ymin, ymax = float(Y[s].min()), float(Y[s].max())
        if max(xmax - xmin, ymax - ymin) > KNOB_MAX_SPAN:
            continue
        pad = 0.02
        near = dark & (X > xmin - pad) & (X < xmax + pad) \
            & (Y > ymin - pad) & (Y < ymax + pad)
        if int((near & (Z > table + KNOB_TALL)).sum()) > 6:
            continue
        if best is None or s.sum() > best[0]:
            best = (int(s.sum()), (xmin, xmax, ymin, ymax))
    if best is None:
        api.log("%s table=%.4f NO KNOB (comps=%d)" % (tag, table, n))
        return None, table, 0.0

    xmin, xmax, ymin, ymax = best[1]
    cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
    body = dark & (Z > table + 0.012) & (Z < table + KNOB_TALL) \
        & (np.abs(X - cx) < 0.09) & (np.abs(Y - cy) < 0.09)
    if int(body.sum()) >= 20:
        cx = 0.5 * (float(X[body].min()) + float(X[body].max()))
        cy = 0.5 * (float(Y[body].min()) + float(Y[body].max()))

    bar = body & (Z > table + BAR_DZ)
    yaw, nbar = 0.0, int(bar.sum())
    if nbar >= 12:
        pts = np.stack([X[bar], Y[bar]], -1)
        pts = pts - pts.mean(0)
        _, sv, vt = np.linalg.svd(pts, full_matrices=False)
        if sv[0] > 1.3 * sv[1]:                    # an actual elongated bar
            yaw = float(np.arctan2(vt[0, 1], vt[0, 0]))
            if yaw > np.pi / 2:
                yaw -= np.pi
            if yaw < -np.pi / 2:
                yaw += np.pi
    api.log("%s table=%.4f knob=(%.4f,%.4f) nbody=%d nbar=%d yaw=%.3f"
            % (tag, table, cx, cy, int(body.sum()), nbar, yaw))
    return (cx, cy), table, yaw


def turn(api, cx, cy, table, yaw, tag):
    """Seat the jaws on the handle bar and twist. Returns the final jaw width."""
    R_app = rz(yaw) @ R_DOWN
    zg = table + GRASP_DZ
    api.grip(0.08)
    api.move([cx, cy, table + HOVER_DZ], R_app, seconds=2.0)
    r = api.move([cx, cy, zg], R_app, seconds=1.8)
    api.grip(0.0)
    api.settle(0.5)
    g = api.gripper()
    api.log("%s closed w=%.4f e=%.2f res=%.4f eef=%s"
            % (tag, g["width_m"], g["effort"], r, np.round(api.eef(), 4).tolist()))
    for k in range(1, TWIST_STEPS + 1):
        th = TWIST_SIGN * TWIST_RAD * k / TWIST_STEPS
        api.move([cx, cy, zg - PRESS_DZ], R_app @ rz(th), seconds=0.8)
    api.settle(0.3)
    g = api.gripper()
    api.log("%s twisted w=%.4f e=%.2f eef=%s"
            % (tag, g["width_m"], g["effort"], np.round(api.eef(), 4).tolist()))
    return g["width_m"]


def run(api):
    api.log("instruction=%r eef0=%s" % (api.instruction(), np.round(api.eef(), 4).tolist()))
    xy, table, yaw = find_knob(api, "P0")
    if xy is None:
        return "no knob"
    w = turn(api, xy[0], xy[1], table, yaw, "T1")
    if w > PRY_W:
        # the jaws were forced open -> the handle did not follow. Re-seat on the
        # knob where it now stands and turn again.
        api.grip(0.08)
        api.move([xy[0], xy[1], table + HOVER_DZ + 0.05], R_DOWN, seconds=1.2)
        xy2, table2, yaw2 = find_knob(api, "P1")
        if xy2 is not None:
            w = turn(api, xy2[0], xy2[1], table2, yaw2, "T2")
    return "turned w=%.4f" % w
