"""open the top drawer of the cabinet  (l90abl / open_top_drawer_s2_vis)

Strategy, derived entirely from this cell's vision pack + debug-seed sensing:

  The pack's three keyframe triplets show the arm going to the cabinet at the
  image-left edge of the table and a drawer box ending up protruding toward
  image-right.  cam_high's extrinsics say image-right is base +y, so the
  cabinet's drawer panel faces +y and the drawer must be pulled in +y.

  Perceived per episode from one cam_high frame: the panel is the dominant
  vertical plane on the -y side (y_face); three cylindrical bar handles stand
  ~0.031 m proud of it, each on two end standoffs.  The TOP bar is the target.

  A top-down approach does not work: v1 measured the slot behind a bar at only
  ~0.015 m and every descent in front of the panel stalled with the fingertips
  resting on the cabinet top.  api.grip is binary, so there is no narrow-jaw
  option either.  Instead the wrist is turned so the tool approach axis points
  -y and the finger axis is vertical: the open jaws (0.078 m) straddle the bar
  vertically -- 0.029 m of free air above it up to the cabinet top, 0.058 m
  below it down to the middle bar -- close onto it, and pull +y.

  Verification is entirely proprioceptive: a close that leaves a finger gap
  (api.gripper reports effort 3.0 only then) means the bar is between the jaws,
  and eef travel in +y while that grip holds means the drawer came with it.
  Up to three engage attempts, with a small y/z search, fit in the horizon.
"""
import json

import numpy as np

PROVENANCE = {
    "R_HORIZ": {
        "source": "generic tool-frame construction: approach axis = base -y "
                  "(the pull direction read off the pack keyframes + cam_high "
                  "extrinsics, reversed), finger axis = base +z; proper "
                  "rotation, det = 1",
        "allowed": True},
    "BAR_R": {
        "source": "debug seeds 51-57, cam_high point cloud: the top handle's "
                  "protruding band spans z 1.083-1.098, i.e. bar radius ~0.008 m",
        "allowed": True},
    "HANDLE_MIN_PROUD": {
        "source": "debug seeds 51-57: handle bars deproject 0.031 m in front of "
                  "the drawer panel plane, so 0.015 m separates bar from panel",
        "allowed": True},
    "HANDLE_MAX_PROUD": {
        "source": "same measurement, upper bound on the handle protrusion",
        "allowed": True},
    "BAND_GAP_Z": {
        "source": "debug seeds 51-57: the three bars are 0.053-0.058 m apart in "
                  "z and each band is ~0.015 m tall, so 0.015 m splits them",
        "allowed": True},
    "STANDOFF": {
        "source": "chosen approach clearance in front of the handle; the debug "
                  "cam_high cloud shows the tabletop props top out ~0.06 m "
                  "above the table, well below the bar-height traverse",
        "allowed": True},
    "ENGAGE_DY": {
        "source": "debug seed 51-57 measurement: a closed gripper pressed on "
                  "bare table stalls with eef 0.0085 m above it, so the eef "
                  "reference sits just behind the fingertips; 0.004 m past the "
                  "bar axis seats the bar between the pads",
        "allowed": True},
    "PULL_STEPS": {
        "source": "chosen pull schedule; achieved travel is read back from the "
                  "eef and the drawer's own stop ends it (measured ~0.14 m on "
                  "debug seeds)",
        "allowed": True},
    "HOLD_MIN_GAP": {
        "source": "api.gripper(): a closed gripper reports effort 3.0 only "
                  "while a finger gap remains, i.e. only while it holds "
                  "something",
        "allowed": True},
    "HOLD_MAX_GAP": {
        "source": "debug seeds 51-57: closing on the bar leaves a 0.0174 m gap; "
                  "a much larger gap would mean the jaws caught something else",
        "allowed": True},
    "TRAVEL_OK": {
        "source": "debug seeds 51-57: the drawer runs ~0.14 m before its stop, "
                  "so 0.08 m of held eef travel proves the drawer moved",
        "allowed": True},
    "SEARCH": {
        "source": "chosen re-engage offsets around the perceived bar centre, "
                  "each smaller than the 0.029 m of free air above the bar",
        "allowed": True},
}

R_HORIZ = np.array([[1.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0],
                    [0.0, 1.0, 0.0]])
BAR_R = 0.008
HANDLE_MIN_PROUD = 0.015
HANDLE_MAX_PROUD = 0.060
BAND_GAP_Z = 0.015
STANDOFF = 0.12
ENGAGE_DY = 0.004
PULL_STEPS = (0.10, 0.20, 0.30)
HOLD_MIN_GAP = 0.005
HOLD_MAX_GAP = 0.040
TRAVEL_OK = 0.08
SEARCH = ((0.0, 0.0), (0.0, -0.006), (0.006, 0.0))


def _cloud(f):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.asarray(f.depth, float)
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    with np.errstate(all="ignore"):
        X = (u - K[0, 2]) * d / K[0, 0]
        Y = (v - K[1, 2]) * d / K[1, 1]
        P = np.stack([X, Y, d, np.ones_like(d)], -1) @ T.T
    return P[..., :3]


def perceive(api):
    """Drawer panel plane + the handle bars standing proud of it."""
    f = api.capture("cam_high")
    pts = _cloud(f)
    pts = pts[np.isfinite(pts).all(-1)]
    tbl = float(np.median(pts[:, 2][pts[:, 2] > 0.5]))
    up = pts[(pts[:, 2] > tbl + 0.03) & (pts[:, 1] < -0.05)]
    if len(up) < 200:
        return None
    hist, edges = np.histogram(up[:, 1], bins=np.arange(-0.60, 0.0, 0.002))
    y_face = float(edges[int(np.argmax(hist))] + 0.001)
    face = up[np.abs(up[:, 1] - y_face) < 0.006]
    if len(face) < 50:
        return None
    y_face = float(np.median(face[:, 1]))
    x_lo = float(np.percentile(face[:, 0], 1))
    x_hi = float(np.percentile(face[:, 0], 99))
    hd = up[(up[:, 1] > y_face + HANDLE_MIN_PROUD)
            & (up[:, 1] < y_face + HANDLE_MAX_PROUD)
            & (up[:, 0] > x_lo - 0.02) & (up[:, 0] < x_hi + 0.02)]
    if len(hd) < 60:
        return None
    bands, grp, prev = [], [], None
    for z in np.sort(hd[:, 2]):
        if prev is not None and z - prev > BAND_GAP_Z:
            bands.append(grp)
            grp = []
        grp.append(z)
        prev = z
    if grp:
        bands.append(grp)
    handles = []
    for g in bands:
        if len(g) < 30:
            continue
        g = np.asarray(g)
        sel = hd[(hd[:, 2] >= g.min()) & (hd[:, 2] <= g.max())]
        handles.append({"ztop": float(np.percentile(g, 99)),
                        "xc": float(0.5 * (np.percentile(sel[:, 0], 2)
                                           + np.percentile(sel[:, 0], 98))),
                        "y_front": float(np.percentile(sel[:, 1], 98)),
                        "n": int(len(sel))})
    if not handles:
        return None
    return {"table_z": tbl, "y_face": y_face, "handles": handles}


def _target(scene):
    top = max(scene["handles"], key=lambda h: h["ztop"])
    return {"x": top["xc"], "y": top["y_front"] - BAR_R,
            "z": top["ztop"] - BAR_R, "raw": top}


def run(api):
    scene = perceive(api)
    if scene is None:
        api.log("perception found no cabinet handles")
        return "no handle perceived"
    api.log("SCENE=%s" % json.dumps(scene))
    bar = _target(scene)
    api.log("BAR=%s" % json.dumps(bar))

    api.grip(0.08)
    # turn the wrist in free air, clear of the cabinet top and of the props
    r = api.move([bar["x"], bar["y"] + STANDOFF, bar["z"] + 0.06],
                 rotation=R_HORIZ, seconds=3.0)
    api.log("turned residual=%.4f rot=%s"
            % (r, np.round(api.tool_rotation(), 3).tolist()))

    for attempt, (dz, dy) in enumerate(SEARCH):
        if attempt:                       # re-perceive before every retry
            api.grip(0.08)
            api.move([bar["x"], bar["y"] + STANDOFF, bar["z"] + 0.02],
                     rotation=R_HORIZ, seconds=2.0)
            again = perceive(api)
            if again is not None:
                bar = _target(again)
                api.log("RE-BAR=%s" % json.dumps(bar))
        xt, yt, zt = bar["x"], bar["y"] + dy, bar["z"] + dz
        api.move([xt, yt + STANDOFF, zt], rotation=R_HORIZ, seconds=1.5)
        r = api.move([xt, yt - ENGAGE_DY, zt], rotation=R_HORIZ, seconds=2.0)
        e0 = api.eef()
        api.log("attempt %d engage residual=%.4f eef=%s"
                % (attempt, r, np.round(e0, 4).tolist()))
        api.grip(0.0)
        api.settle(0.3)
        g = api.gripper()
        api.log("attempt %d closed grip=%s" % (attempt, g))
        if not (HOLD_MIN_GAP < g["width_m"] < HOLD_MAX_GAP and g["effort"] >= 3.0):
            api.log("attempt %d: jaws did not capture the bar" % attempt)
            continue
        for step in PULL_STEPS:
            r = api.move([xt, yt - ENGAGE_DY + step, zt],
                         rotation=R_HORIZ, seconds=2.0)
            e = api.eef()
            g = api.gripper()
            api.log("attempt %d pull %.2f residual=%.4f eef=%s grip=%s"
                    % (attempt, step, r, np.round(e, 4).tolist(), g))
        e = api.eef()
        travel = float(e[1] - e0[1])
        held = api.gripper()
        api.log("attempt %d travel=%.4f held=%s" % (attempt, travel, held))
        if travel >= TRAVEL_OK:
            return "pulled %.3f m" % travel
        api.log("attempt %d: drawer did not travel far enough" % attempt)
    return "no verified pull"
