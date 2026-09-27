"""v6 -- v5 plus an outside-first lane order and a lateral re-push.

v3 (tilt 70 deg, hand above the drawer rim, push in 0.03 m steps) scored 3/8 on
debug seeds 51..65 and split its failures into two mechanisms:

  A. staging deadlock (53, 55, 63).  The tilt was commanded at
     y0 = front_y - 0.13, which for those seeds is y0 <= -0.100.  The wrist
     could not reach that tilted pose: the hand drifted UP to z ~ 1.178 and
     then froze -- every following move reported prog = 0.0000 with a growing
     residual, and the drawer never moved at all.  The five seeds that worked
     all had y0 >= -0.095.  So the tilted staging pose has to stay forward of
     about y = -0.09.

  B. the push ran out of FORCE, not of reach (51/57/59/61/65).  Every one of
     those pushes stalled with the hand at y = 0.196..0.218, i.e. jammed
     against the drawer it was pushing, and the drawer front sitting 0.00-0.05
     short of the cabinet face.  The controller is action = clip(err/0.05,-1,1),
     so a target only 0.03 m ahead commands 60% of the available effort and the
     drawer stops as soon as friction matches it.  Seeds that ended at
     y >= 0.205 closed the drawer (51/59/65 = success); the two that stopped at
     0.196/0.202 did not.
     Fix: command the push target 0.30 m past the drawer front and hold it, so
     the error never falls below 0.05 and the action stays saturated for the
     whole push.

v4 then commanded the deep target from the staging pose and scored 0/8: with
the whole error saturated the hand travelled DIAGONALLY -- it was still at
z ~ 1.06 as it crossed the panel plane and only reached push height 0.035 m
PAST the panel, so it flew over the drawer front and merely brushed the handle
(front moved 0.11-0.14 m and stopped, hand ahead of the panel, no contact).
The descent itself is blocked: at the staging pose the tilted fingers hang
forward onto the panel/handle, so the DOWN move always reports a 0.12 m
residual.  v3 got away with that because its 0.03 m steps let the hand walk
down the panel as the drawer retreated.

v5 (incremental walk-in + saturated finish) scored 7/8 on debug seeds 51..65.
Its one failure, seed 61, stalled with the hand at y = 0.1962 while every
success stalled at y = 0.200..0.218 -- and seed 61 is also the seed whose
cabinet sits ~0.009 m further out, so it needed MORE stroke and got less.  The
difference is the lane: seed 61 pushed at x = -0.043 (the only clean lane the
old ordering tried before returning), the successes at x = -0.075..-0.092 or
+0.114.  Across all eight seeds the hand's furthest +y at push height grows
with |x - drawer centre|, so v6 tries the outside lanes first and, if the
stroke still ends short of the measured cabinet face, slides sideways while
pressed and leans again.

v5 keeps v3's incremental engagement (which reliably puts the hand at push
height, in contact, in front of the panel) and only then holds a target 0.30 m
past the drawer so the last part of the stroke runs at full effort.  It also
clamps the staging y to the reachable band found above.
"""
import numpy as np

PROVENANCE = {
    "PUSH_AXIS_PLUS_Y": {
        "source": "pack ee_path: all three demos end 0.14-0.20 m in +y of their "
                  "start with gripper_cmd = -1 (open) throughout -- a push",
        "allowed": True},
    "TILT_DEG 70": {
        "source": "pack ee_path6 final keyframes read as robosuite axis-angle: "
                  "tool-z = (-0.40,0.61,-0.68)/(-0.29,0.93,-0.24)/(-0.66,0.58,-0.48) "
                  "= 45-75 deg forward; v2 debug seeds showed the untilted wrist "
                  "jams at y~0.117",
        "allowed": True},
    "Y0_BACK 0.13 / Y0_LIMIT -0.085 (staging y)": {
        "source": "v3 debug seeds: tilted staging at y0 <= -0.100 (seeds 53/55/63) "
                  "froze the wrist at z~1.178 with prog=0.0000; y0 >= -0.095 "
                  "(seeds 51/57/59/61/65) always reached the drawer",
        "allowed": True},
    "SAT 0.30 (push target beyond the front)": {
        "source": "v3 debug seeds: pushes commanded 0.03 m ahead stalled at "
                  "y=0.196-0.218 with the drawer 0-0.05 m short; the controller "
                  "saturates only for errors above 0.05 m",
        "allowed": True},
    "STEP 0.03 / N_STEP 6 (incremental engagement)": {
        "source": "v3 vs v4 debug seeds: 0.03 m steps walked the hand down to "
                  "push height in front of the panel (drawer moved 0.13-0.15 m); "
                  "a single saturated command crossed the panel plane 0.06 m too "
                  "high and moved it only 0.11-0.14 m",
        "allowed": True},
    "Z_BAND (table+0.02 .. table+0.09)": {
        "source": "v1 debug-seed cam_high cloud: drawer side walls span z "
                  "0.918-0.983, table top measured at 0.900",
        "allowed": True},
    "EEF_Z = panel_top + 0.012": {
        "source": "v1/v3 debug-seed measurement: drawer rim z~0.985, bowl rim "
                  "z~0.955; at this height the hand clears both and the tilted "
                  "fingers hang into the panel's z band",
        "allowed": True},
    "MIN_STRUCT_COUNT 60/150": {
        "source": "v1 debug-seed point counts per 1 cm y-slice (walls ~260, front "
                  "panel ~440-1100, bowl ~130)",
        "allowed": True},
    "CAND_DX outside-first order": {
        "source": "v5 debug seeds: stall y = 0.2000-0.2182 at lanes "
                  "x = -0.092..-0.075 and +0.114, but only 0.1962 at x = -0.043",
        "allowed": True},
    "RETRY_MARGIN 0.010 / LATERAL_SLIDE 0.045": {
        "source": "v5 debug seeds: every success stalled at least 0.009 m past "
                  "the pre-push cabinet face, the failure 0.009 m short of it",
        "allowed": True},
    "LANE_HALF_WIDTH 0.035": {
        "source": "api.gripper() on debug seeds: 0.078 m open finger span",
        "allowed": True},
    "CLOSED_GAP_TOL 0.02": {
        "source": "v3 debug seeds: 1 cm y-binning of the perceived front panel; "
                  "successful closes read front_y within 0.001-0.005 of the "
                  "measured cabinet face",
        "allowed": True},
}

Z_LO, Z_HI = 0.020, 0.090
XW = (-0.30, 0.30)
YW = (-0.12, 0.60)
LANE_HW = 0.035
TILT_DEG = 70.0
Y0_BACK = 0.13
Y0_LIMIT = -0.085
STEP = 0.03
N_STEP = 6
SAT = 0.30
CAND_DX = (-0.095, 0.095, -0.075, 0.075, -0.045, 0.045, 0.0)


def cloud(frame):
    d = np.asarray(frame.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    vs, us = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 1e-4)
    z = np.where(ok, d, 1.0)
    x = (us - K[0, 2]) * z / K[0, 0]
    y = (vs - K[1, 2]) * z / K[1, 1]
    pts = np.stack([x, y, z, np.ones_like(z)], axis=-1) @ T.T
    return pts[..., :3][ok]


def table_height(P):
    m = (np.abs(P[:, 0]) < 0.35) & (np.abs(P[:, 1]) < 0.35) & (P[:, 2] > 0.5)
    h, e = np.histogram(P[m, 2], bins=np.arange(0.6, 1.3, 0.005))
    return float(e[int(np.argmax(h))]) + 0.0025


def tilt_matrix(deg):
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, -c, s],
                     [0.0, -s, -c]])


def perceive(api, tag=""):
    f = api.capture("cam_high")
    P = cloud(f)
    tz = table_height(P)
    band = P[(P[:, 2] > tz + Z_LO) & (P[:, 2] < tz + Z_HI)
             & (P[:, 0] > XW[0]) & (P[:, 0] < XW[1])
             & (P[:, 1] > YW[0]) & (P[:, 1] < YW[1])]
    deep = band[(band[:, 1] > 0.12) & (band[:, 1] < 0.30)]
    if deep.shape[0] < 200:
        raise RuntimeError("no drawer interior (n=%d)" % deep.shape[0])
    x_lo = float(np.percentile(deep[:, 0], 2))
    x_hi = float(np.percentile(deep[:, 0], 98))
    xc = 0.5 * (x_lo + x_hi)
    width = x_hi - x_lo
    inx = band[(band[:, 0] > x_lo + 0.005) & (band[:, 0] < x_hi - 0.005)]
    panel_y, prof = None, []
    for y in np.arange(-0.10, 0.32, 0.01):
        s = inx[(inx[:, 1] >= y) & (inx[:, 1] < y + 0.01)]
        if s.shape[0] < 60:
            continue
        ext = float(np.percentile(s[:, 0], 98) - np.percentile(s[:, 0], 2))
        prof.append((round(float(y), 3), int(s.shape[0]), round(ext, 3)))
        if panel_y is None and ext > 0.75 * width and s.shape[0] > 150:
            panel_y = float(y)
    if panel_y is None:
        raise RuntimeError("front panel not found; profile=%s" % prof)
    handle = inx[(inx[:, 1] > panel_y - 0.06) & (inx[:, 1] < panel_y - 0.005)
                 & (np.abs(inx[:, 0] - xc) < 0.06)]
    handle_y = float(np.percentile(handle[:, 1], 3)) if handle.shape[0] > 60 else panel_y
    face = band[(band[:, 1] > panel_y - 0.01) & (band[:, 1] < panel_y + 0.03)
                & (np.abs(band[:, 0] - xc) < 0.09)]
    panel_top = float(np.percentile(face[:, 2], 98)) if face.shape[0] > 30 else tz + 0.085
    up = P[(P[:, 2] > tz + 0.10) & (P[:, 2] < tz + 0.145)
           & (np.abs(P[:, 0] - xc) < 0.16) & (P[:, 1] > panel_y + 0.02) & (P[:, 1] < 0.6)]
    closed_y = float(np.percentile(up[:, 1], 2)) if up.shape[0] > 120 else panel_y + 0.16
    out = dict(table_z=tz, x_lo=x_lo, x_hi=x_hi, xc=xc, width=width,
               panel_y=panel_y, handle_y=handle_y, panel_top=panel_top,
               front_y=min(handle_y, panel_y), closed_y=closed_y)
    api.log("PERCEIVE%s %s" % (tag, {k: round(v, 4) for k, v in out.items()}))
    api.log("PERCEIVE%s prof %s" % (tag, prof[:26]))
    return out, P


def pick_lane(api, P, g, y0, avoid=None):
    """Lane with the clearest column at the staging y."""
    tz = g["table_z"]
    best, best_n = None, None
    for dx in CAND_DX:
        x = g["xc"] + dx
        if not (g["x_lo"] + 0.015 < x < g["x_hi"] - 0.015):
            continue
        if avoid is not None and abs(x - avoid) < 0.03:
            continue
        blk = P[(np.abs(P[:, 0] - x) < LANE_HW) & (P[:, 2] > tz + 0.030)
                & (P[:, 1] > y0 - 0.04) & (P[:, 1] < y0 + 0.06)]
        n = int(blk.shape[0])
        api.log("LANE x=%.3f blocked_n=%d" % (x, n))
        if n < 40:
            return x, True
        if best_n is None or n < best_n:
            best, best_n = x, n
    return (best if best is not None else g["xc"]), False


def stage(api, g, lane_x, y0, R):
    """Get the tilted hand to (lane_x, y0, panel_top+0.012)."""
    eef_z = g["panel_top"] + 0.012
    api.move([lane_x, y0, g["table_z"] + 0.21], seconds=1.2)
    r = api.move([lane_x, y0, g["table_z"] + 0.21], rotation=R, seconds=1.8)
    api.log("TILT eef=%s res=%.4f toolz=%s"
            % (np.round(api.eef(), 4).tolist(), r,
               np.round(api.tool_rotation()[:, 2], 3).tolist()))
    r = api.move([lane_x, y0, eef_z], rotation=R, seconds=1.2)
    api.log("DOWN eef=%s res=%.4f target=%.4f"
            % (np.round(api.eef(), 4).tolist(), r, eef_z))
    return eef_z


def walk_in(api, lane_x, eef_z, R, y_start, y_goal):
    """v3's incremental advance: small steps so the hand can settle down onto
    push height in front of the panel instead of vaulting over it."""
    y = y_start
    last = api.eef()
    stalls = 0
    for i in range(N_STEP):
        y = min(y + STEP, y_goal)
        r = api.move([lane_x, y, eef_z], rotation=R, seconds=0.5)
        cur = api.eef()
        prog = float(cur[1] - last[1])
        api.log("WALK%d ->%.3f eef=%s res=%.4f prog=%.4f"
                % (i, y, np.round(cur, 4).tolist(), r, prog))
        last = cur
        if prog < 0.004:
            stalls += 1
            if stalls >= 2:
                api.log("WALK stall at y=%.4f" % float(cur[1]))
                break
        else:
            stalls = 0
        if y >= y_goal - 1e-6:
            break
    return float(api.eef()[1])


def power_push(api, lane_x, eef_z, R, y_far, chunks=3, seconds=1.2):
    """Hold a target far past the drawer so the OSC action stays saturated
    (action = clip(err/0.05, -1, 1)); this is the force the last centimetres of
    the stroke need."""
    for i in range(chunks):
        r = api.move([lane_x, y_far, eef_z], rotation=R, seconds=seconds)
        api.log("PUSH%d eef=%s res=%.4f grip=%s"
                % (i, np.round(api.eef(), 4).tolist(), r, api.gripper()))
    return float(api.eef()[1])


def run(api):
    api.log("instruction %r" % api.instruction())
    g, P = perceive(api, "0")
    R = tilt_matrix(TILT_DEG)
    y0 = max(g["front_y"] - Y0_BACK, Y0_LIMIT)
    lane_x, clean = pick_lane(api, P, g, y0)
    api.log("lane %.4f clean=%s y0=%.4f" % (lane_x, clean, y0))
    api.grip(0.0)
    eef_z = stage(api, g, lane_x, y0, R)
    walk_in(api, lane_x, eef_z, R, y0, g["front_y"] + 0.10)
    stall = power_push(api, lane_x, eef_z, R, g["front_y"] + SAT)

    # The stroke has to carry the drawer front past the cabinet face I measured
    # before touching anything; if the hand stopped short of it, the lane was
    # the limit, so slide sideways (still pressed) and lean again.
    if stall < g["closed_y"] + 0.010:
        alt = lane_x - 0.045
        if not (g["x_lo"] + 0.015 < alt < g["x_hi"] - 0.015):
            alt = lane_x + 0.045
        alt = float(min(max(alt, g["x_lo"] + 0.015), g["x_hi"] - 0.015))
        api.log("SHORT stall=%.4f closed_y=%.4f -> slide lane %.4f"
                % (stall, g["closed_y"], alt))
        api.move([alt, float(api.eef()[1]), eef_z], rotation=R, seconds=0.8)
        power_push(api, alt, eef_z, R, g["front_y"] + SAT, chunks=2)

    # verify with my own sensors (logging only -- no runtime success signal)
    api.move([lane_x, float(api.eef()[1]) - 0.07, g["table_z"] + 0.24],
             rotation=R, seconds=1.0)
    api.move([-0.16, -0.12, g["table_z"] + 0.26], seconds=1.0)
    try:
        g2, _ = perceive(api, "1")
        api.log("RESULT front_y %.4f -> %.4f moved %.4f (closed_y %.4f gap %.4f)"
                % (g["front_y"], g2["front_y"], g2["front_y"] - g["front_y"],
                   g2["closed_y"], g2["closed_y"] - g2["front_y"]))
    except Exception as ex:
        api.log("verify failed: %s" % ex)
    return "walk-in + power push"
