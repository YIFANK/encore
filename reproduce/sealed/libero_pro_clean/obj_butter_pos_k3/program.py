"""c2clean / obj_butter_pos_k3 -- v4.

Pick the butter (the small flat warm-orange slab) and drop it into the basket.

Identity chain (all of it re-derived in this cell):
  * the pack's three demos all close the gripper at eef z ~= 0.010 with the
    table top at z = 0.001, so the demonstrated target is a very low slab,
    not one of the bottles/cartons;
  * the pack keyframes show the grasped prop as a small ORANGE box with a dark
    top band, and show a second, larger BROWN box that is never touched;
  * on the debug seeds exactly two props sit below 45 mm: a brown slab
    (warmth ~43) and an orange slab (warmth ~126). The orange one is the one
    the demos picked.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {
        "source": "debug seeds 51-65: modal z of the cam_high depth cloud "
                  "deprojected into base frame = 0.001 m",
        "allowed": True},
    "WIN_X": {
        "source": "fair_run stdout on the debug seeds prints the reachable "
                  "workspace x(-0.45, 0.45); used only to crop the cloud",
        "allowed": True},
    "WIN_Y": {
        "source": "same fair_run workspace line, y(-0.45, 0.52)",
        "allowed": True},
    "Z_FLOOR": {
        "source": "debug-seed height maps: 8 mm above TABLE_Z removes the "
                  "table plane's depth noise while keeping the 19 mm slab",
        "allowed": True},
    "ROBOT_CHROMA": {
        "source": "debug seeds: the arm renders green/blue (G or B exceeding R "
                  "by >12); every table prop and the wood table are warm, so "
                  "this separates the arm from the props",
        "allowed": True},
    "ROBOT_Z": {
        "source": "debug seeds: the tallest table prop tops out at 0.148, the "
                  "arm hangs from 0.26 up; 0.16 is the gap, and gating the "
                  "colour mask by it keeps the butter's blue top band",
        "allowed": True},
    "CELL": {
        "source": "debug seeds: 12 mm XY grid keeps the six props apart while "
                  "keeping each prop in one component",
        "allowed": True},
    "MIN_PTS": {
        "source": "debug seeds: the smallest real prop (the butter) gives "
                  ">130 subsampled points; 40 rejects depth speckle",
        "allowed": True},
    "FLAT_TOP": {
        "source": "debug seeds: the two slab props top out at 0.019 and 0.029, "
                  "the next prop at 0.081; 0.045 is the gap",
        "allowed": True},
    "BASKET_MIN_SPAN": {
        "source": "debug seeds: the basket spans 0.16 x 0.17 in XY, every prop "
                  "spans <0.08",
        "allowed": True},
    "TOPFACE_BAND": {
        "source": "debug seeds: 5 mm below the slab's max z isolates its top "
                  "face, whose centroid is stable to +-1 mm across seeds",
        "allowed": True},
    "GRASP_Z": {
        "source": "pack.json keyframes: the three demos close the gripper at "
                  "eef z = 0.0095 / 0.0100 / 0.0111",
        "allowed": True},
    "HOVER_Z": {
        "source": "pack.json ee_path6: the demos approach the grasp from "
                  "z ~ 0.14-0.19 above the prop",
        "allowed": True},
    "CARRY_Z": {
        "source": "pack.json ee_path6: the demos transport at z = 0.25-0.29",
        "allowed": True},
    "PLACE_Z": {
        "source": "pack.json keyframes: the demos open the gripper over the "
                  "basket at eef z = 0.191 / 0.160 / 0.156; debug-seed basket "
                  "rim measures 0.144, so 0.20 clears it",
        "allowed": True},
    "OPEN_W": {
        "source": "api.gripper() reports width_m 0.0778 at episode start with "
                  "the gripper open; >=0.025 commands an open",
        "allowed": True},
    "CLOSE_W": {
        "source": "FairApi.grip docstring: <0.025 closes",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi docstring: effort 3.0 iff holding",
        "allowed": True},
    "R_DOWN": {
        "source": "api.tool_rotation() at episode start is diag-like "
                  "[[1,0,0],[0,-1,0],[0,0,-1]] up to a 3 deg pitch; the demos "
                  "grasp with yaw ~ 0",
        "allowed": True},
}

TABLE_Z = 0.001
WIN_X = (-0.45, 0.45)
WIN_Y = (-0.45, 0.52)
Z_FLOOR = 0.008
ROBOT_CHROMA = 12.0
ROBOT_Z = 0.16
CELL = 0.012
MIN_PTS = 40
FLAT_TOP = 0.045
BASKET_MIN_SPAN = 0.12
TOPFACE_BAND = 0.005
GRASP_Z = 0.010
HOVER_Z = 0.14
CARRY_Z = 0.26
PLACE_Z = 0.20
OPEN_W = 0.08
CLOSE_W = 0.0
HOLD_EFFORT = 2.0
SUB = 2

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


# ---------------------------------------------------------------- perception

def _cloud(frame, sub=SUB):
    """Whole-frame pixel -> base-frame xyz (the same pinhole math api.deproject
    does per pixel), subsampled for speed."""
    d = np.asarray(frame.depth, dtype=np.float64)[::sub, ::sub]
    rgb = np.asarray(frame.rgb)[::sub, ::sub].astype(np.float64)
    K = np.asarray(frame.intrinsics, dtype=np.float64)
    T = np.asarray(frame.t_base_cam, dtype=np.float64)
    h, w = d.shape
    uu, vv = np.meshgrid(np.arange(w) * sub, np.arange(h) * sub)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    pts = np.stack([(uu - cx) * d / fx, (vv - cy) * d / fy, d,
                    np.ones_like(d)], -1)
    return rgb, (pts @ T.T)[..., :3]


def _components(keep_idx, gi, gj):
    occ = {}
    for k in keep_idx:
        occ.setdefault((gi[k], gj[k]), []).append(k)
    seen, comps = set(), []
    for c in occ:
        if c in seen:
            continue
        stack, cells = [c], []
        seen.add(c)
        while stack:
            a = stack.pop()
            cells.append(a)
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb = (a[0] + di, a[1] + dj)
                    if nb in occ and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        comps.append(np.concatenate([occ[c] for c in cells]))
    return comps


def survey(api, cam="cam_high"):
    frame = api.capture(cam)
    rgb, B = _cloud(frame)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    # The arm is the only green/blue thing in the scene AND the only thing
    # above ROBOT_Z; the butter carries a blue band, so colour alone would
    # eat half of it.
    robot = ((g > r + ROBOT_CHROMA) | (b > r + ROBOT_CHROMA)) & (Z > ROBOT_Z)
    grown = robot.copy()
    for dv in (-2, -1, 1, 2):
        grown |= np.roll(robot, dv, 0)
        grown |= np.roll(robot, dv, 1)
    for dv in (-2, -1, 1, 2):
        for du in (-2, -1, 1, 2):
            grown |= np.roll(np.roll(robot, dv, 0), du, 1)
    keep = ((X > WIN_X[0]) & (X < WIN_X[1]) &
            (Y > WIN_Y[0]) & (Y < WIN_Y[1]) &
            (Z > TABLE_Z + Z_FLOOR) & (Z < 0.40) & ~grown)
    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], -1)
    cols = rgb.reshape(-1, 3)
    gi = np.floor(pts[:, 0] / CELL).astype(int)
    gj = np.floor(pts[:, 1] / CELL).astype(int)
    props = []
    for idx in _components(np.nonzero(keep.ravel())[0], gi, gj):
        if len(idx) < MIN_PTS:
            continue
        p, c = pts[idx], cols[idx]
        top = float(p[:, 2].max())
        face = p[p[:, 2] > top - TOPFACE_BAND]
        props.append(dict(
            n=int(len(idx)), top=top,
            cx=float(p[:, 0].mean()), cy=float(p[:, 1].mean()),
            fx=float(face[:, 0].mean()), fy=float(face[:, 1].mean()),
            xlo=float(p[:, 0].min()), xhi=float(p[:, 0].max()),
            ylo=float(p[:, 1].min()), yhi=float(p[:, 1].max()),
            warm=float(np.percentile(c[:, 0] - c[:, 2], 85))))
    return props


def choose(props):
    basket = None
    wide = [p for p in props
            if (p['xhi'] - p['xlo']) > BASKET_MIN_SPAN
            and (p['yhi'] - p['ylo']) > BASKET_MIN_SPAN]
    if wide:
        basket = max(wide, key=lambda p: p['n'])
    flats = [p for p in props if p is not basket and p['top'] < FLAT_TOP]
    butter = max(flats, key=lambda p: p['warm']) if flats else None
    return basket, butter


# ---------------------------------------------------------------- behaviour

def _holding(api):
    gs = api.gripper()
    return float(gs.get("effort", 0.0)) >= HOLD_EFFORT, gs


def _attempt(api, bx, by):
    """Open, drop straight down onto the slab, close. -> (held, gripper)."""
    api.grip(OPEN_W)
    api.move([bx, by, CARRY_Z], rotation=R_DOWN, seconds=2.0)
    api.move([bx, by, HOVER_Z], rotation=R_DOWN, seconds=2.0)
    api.log("hover eef=%s" % np.asarray(api.eef()).round(4).tolist())
    api.move([bx, by, GRASP_Z], rotation=R_DOWN, seconds=2.0)
    api.log("descend eef=%s" % np.asarray(api.eef()).round(4).tolist())
    api.grip(CLOSE_W)
    api.settle(0.6)
    return _holding(api)


def run(api):
    api.log("instruction: %r" % api.instruction())
    props = survey(api)
    for p in props:
        api.log("prop n=%d c=(%+.3f,%+.3f) face=(%+.3f,%+.3f) top=%.3f "
                "span=(%.3f,%.3f) warm=%.1f"
                % (p['n'], p['cx'], p['cy'], p['fx'], p['fy'], p['top'],
                   p['xhi'] - p['xlo'], p['yhi'] - p['ylo'], p['warm']))
    basket, butter = choose(props)
    if butter is None or basket is None:
        api.log("PERCEPTION FAILED butter=%r basket=%r" % (butter, basket))
        return
    bx, by = butter['fx'], butter['fy']
    kx = 0.5 * (basket['xlo'] + basket['xhi'])
    ky = 0.5 * (basket['ylo'] + basket['yhi'])
    api.log("BUTTER (%+.4f,%+.4f) top=%.4f | BASKET (%+.4f,%+.4f) top=%.4f"
            % (bx, by, butter['top'], kx, ky, basket['top']))

    held, gs = _attempt(api, bx, by)
    api.log("after close held=%s grip=%r" % (held, gs))
    api.move([bx, by, CARRY_Z], rotation=R_DOWN, seconds=2.5)
    held, gs = _holding(api)
    api.log("after lift held=%s grip=%r eef=%s"
            % (held, gs, np.asarray(api.eef()).round(4).tolist()))

    if not held:
        # Nothing in the jaws: re-perceive from the lifted pose and try once
        # more. The slab may have been nudged by the failed close.
        api.log("RETRY: re-surveying")
        props = survey(api)
        _, butter = choose(props)
        if butter is not None:
            bx, by = butter['fx'], butter['fy']
            api.log("RETRY butter (%+.4f,%+.4f)" % (bx, by))
            held, gs = _attempt(api, bx, by)
            api.log("retry close held=%s grip=%r" % (held, gs))
            api.move([bx, by, CARRY_Z], rotation=R_DOWN, seconds=2.5)
            held, gs = _holding(api)
            api.log("retry lift held=%s grip=%r" % (held, gs))

    r1 = api.move([0.5 * (bx + kx), 0.5 * (by + ky), CARRY_Z],
                  rotation=R_DOWN, seconds=2.5)
    r2 = api.move([kx, ky, CARRY_Z], rotation=R_DOWN, seconds=2.5)
    r3 = api.move([kx, ky, PLACE_Z], rotation=R_DOWN, seconds=2.0)
    api.log("transport residuals %.4f %.4f %.4f" % (r1, r2, r3))
    held, gs = _holding(api)
    api.log("over basket held=%s grip=%r eef=%s"
            % (held, gs, np.asarray(api.eef()).round(4).tolist()))
    api.grip(OPEN_W)
    api.settle(1.0)
    api.move([kx, ky, CARRY_Z + 0.02], rotation=R_DOWN, seconds=2.0)
    api.settle(0.5)
    api.log("released grip=%r" % (api.gripper(),))
