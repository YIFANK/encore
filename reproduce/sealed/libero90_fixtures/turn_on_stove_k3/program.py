"""l90abl / turn_on_stove_k3 -- v2: perceive the knob, verify the grasp, retry.

Mechanism (established in v1, 15/15 on the debug band):
  the manipulandum is a small dark knob standing on the stove slab; grasp it
  top-down and spin the wrist counter-clockwise about world +z. The pack's
  turn segment is contact-limited (actions[:,5] pins at +0.375 for ~40 steps
  while the realised yaw only reaches 53-67 deg), so the turn is commanded as
  staged orientation targets with a standing downward position error rather
  than as a converging pose move.

What v2 adds over the fixed pack-mean aim point of v1:
  1. the aim xy AND the grasp/press heights come from the knob actually seen
     in cam_high, so the program follows the stove wherever it is placed
     instead of trusting a constant;
  2. the detection is gated on its own shape receipts and falls back to the
     pack-mean prior if any gate fails;
  3. the grasp is verified from gripper width/effort, and a turn that does not
     move the wrist is retried once from a fresh perception.
No success signal is read at any point -- every branch is driven by the
program's own sensors.
"""

import numpy as np

PROVENANCE = {
    "PRIOR_XY": {"source": "pack.json: mean of the three demos' EEF xy at the "
                           "keyframe where gripper_cmd flips -1 -> +1 "
                           "((-0.1954,0.2096),(-0.1810,0.2092),(-0.2114,0.2027))",
                 "allowed": True},
    "SEARCH_HALF": {"source": "debug seeds 51-65 (v1 cam_high dumps): the knob "
                              "centroid sits 8-31 mm from PRIOR_XY, so a 0.13 m "
                              "half-window always contains it with room for a "
                              "displaced prior",
                    "allowed": True},
    "SUPPORT_BAND": {"source": "debug seeds 51-65: the z histogram under the "
                               "prior has the table at 0.900 and the stove slab "
                               "top at 0.9315; [0.905,0.950) isolates the slab "
                               "from the knob standing on it",
                     "allowed": True},
    "STAND_LO": {"source": "debug seeds 51-65: knob points sit 8-45 mm above the "
                           "slab top (slab 0.9315, knob top 0.9601)",
                 "allowed": True},
    "STAND_HI": {"source": "same measurement, upper edge with margin",
                 "allowed": True},
    "DARK_LUM": {"source": "debug seeds 51-65: mean RGB luminance of the knob "
                           "pixels is 47-49 against a slab mean of ~110",
                 "allowed": True},
    "MIN_PX": {"source": "debug seeds 51-65: the gated knob blob is 488-512 px "
                         "in every one of the 15 seeds",
               "allowed": True},
    "MAX_PX": {"source": "same measurement, upper bound with margin",
               "allowed": True},
    "KNOB_H_LO": {"source": "debug seeds 51-65: knob top minus slab top is "
                            "0.0286 m in every seed",
                  "allowed": True},
    "KNOB_H_HI": {"source": "same measurement, bracketed with margin",
                  "allowed": True},
    "Z_CLOSE_BELOW_TOP": {"source": "pack.json close-keyframe z relative to the "
                                    "knob top measured on the debug seeds "
                                    "(0.9601): 0.9493 mean pack z = top-0.011; "
                                    "v1 logged the descent stalling on contact "
                                    "at top-0.001 with that target",
                          "allowed": True},
    "Z_PRESS_BELOW_TOP": {"source": "pack.json final-keyframe z (0.9285) minus "
                                    "the standing press v1 used (0.015), against "
                                    "the measured knob top 0.9601 -> top-0.047",
                          "allowed": True},
    "Z_HOVER_ABOVE_TOP": {"source": "pack.json ee_path z one waypoint before the "
                                    "close (1.045/1.015/1.033) vs the measured "
                                    "knob top 0.9601 -> ~top+0.09",
                          "allowed": True},
    "YAW_TOTAL_DEG": {"source": "pack.json: realised yaw change of the tool "
                                "x-axis between first and last keyframe "
                                "(+67/+67/+53 deg); target set past the max",
                      "allowed": True},
    "YAW_STEP_DEG": {"source": "pack.json: actions[:,5] holds +0.375 during the "
                               "turn and the harness maps a rotation command to "
                               "clip(err/0.30,-1,1), so 0.375 == 6.4 deg of "
                               "standing orientation error; staged targets 10 "
                               "deg ahead reproduce that command magnitude",
                     "allowed": True},
    "HELD_W_LO": {"source": "debug seeds 51-65 (v1 logs): the finger gap after "
                            "closing on the knob is 0.0253-0.0280 m",
                  "allowed": True},
    "HELD_W_HI": {"source": "same measurement, bracketed with margin",
                  "allowed": True},
    "YAW_OK_DEG": {"source": "debug seeds 51-65 (v1 logs): a captured knob "
                             "always yielded >=37.8 deg of realised yaw; a "
                             "wrist that has not turned this far has not "
                             "engaged the knob",
                   "allowed": True},
    "FALLBACK_ZTOP": {"source": "debug seeds 51-65: the detected knob top is "
                                "0.9601 in all 15 seeds (sd 0.00000); used only "
                                "when the detection gate rejects a frame",
                      "allowed": True},
    "R_DOWN": {"source": "pack.json: every demo's start rotation vector has "
                         "norm ~pi about +x (tool z straight down); matches "
                         "api.tool_rotation() at t=0 on the debug seeds to 0.06",
               "allowed": True},
}

PRIOR_XY = (-0.1959, 0.2072)
SEARCH_HALF = 0.13
SUPPORT_BAND = (0.905, 0.950)
STAND_LO, STAND_HI = 0.008, 0.060
DARK_LUM = 80.0
MIN_PX, MAX_PX = 200, 1400
KNOB_H_LO, KNOB_H_HI = 0.015, 0.055

Z_CLOSE_BELOW_TOP = 0.011
Z_PRESS_BELOW_TOP = 0.047
Z_HOVER_ABOVE_TOP = 0.090

YAW_TOTAL_DEG = 100.0
YAW_STEP_DEG = 10.0
HELD_W_LO, HELD_W_HI = 0.010, 0.045
YAW_OK_DEG = 20.0

R_DOWN = np.array([[1.0, 0.0, 0.0],
                   [0.0, -1.0, 0.0],
                   [0.0, 0.0, -1.0]])

FALLBACK_ZTOP = 0.9601   # debug seeds 51-65: identical in all 15


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _yaw(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))


def _cloud(f):
    d = np.asarray(f.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(f.intrinsics, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    ok = np.isfinite(d) & (d > 0) & (d < 3.0)
    z = np.where(ok, d, 1.0)
    pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z,
                   np.ones_like(z)], axis=-1)
    return (pc @ np.asarray(f.t_base_cam, float).T)[..., :3], ok


def find_knob(api, f):
    """Dark blob standing on the flat support under the prior. Gated; returns
    (xy, ztop, why) with xy None when any receipt fails."""
    xyz, ok = _cloud(f)
    X, Y, Z = xyz[..., 0], xyz[..., 1], xyz[..., 2]
    lum = np.asarray(f.rgb, float).mean(axis=-1)
    px, py = PRIOR_XY
    near = (ok & (np.abs(X - px) < SEARCH_HALF) & (np.abs(Y - py) < SEARCH_HALF)
            & (Z > 0.85) & (Z < 1.10))
    supp = near & (Z >= SUPPORT_BAND[0]) & (Z < SUPPORT_BAND[1])
    if int(supp.sum()) < 200:
        return None, None, "support px %d" % int(supp.sum())
    slab = float(np.percentile(Z[supp], 90))
    m = (near & (Z > slab + STAND_LO) & (Z < slab + STAND_HI)
         & (lum < DARK_LUM))
    n = int(m.sum())
    if not (MIN_PX <= n <= MAX_PX):
        return None, None, "blob px %d (slab %.4f)" % (n, slab)
    cx_, cy_ = float(X[m].mean()), float(Y[m].mean())
    ztop = float(Z[m].max())
    h = ztop - slab
    if not (KNOB_H_LO <= h <= KNOB_H_HI):
        return None, None, "knob height %.4f out of band" % h
    d = float(np.hypot(cx_ - px, cy_ - py))
    api.log("knob: xy=(%+.4f,%+.4f) ztop=%.4f slab=%.4f h=%.4f px=%d "
            "dprior=%.4f lum=%.0f" % (cx_, cy_, ztop, slab, h, n, d,
                                      float(lum[m].mean())))
    return (cx_, cy_), ztop, "ok"


def grasp_and_turn(api, xy, ztop, attempt):
    kx, ky = xy
    hov = [kx, ky, ztop + Z_HOVER_ABOVE_TOP]
    api.grip(0.06)
    r = api.move(hov, rotation=R_DOWN, seconds=2.0)
    api.log("a%d hover res %.4f eef %s" % (attempt, r,
                                           np.round(api.eef(), 4).tolist()))
    r = api.move([kx, ky, ztop - Z_CLOSE_BELOW_TOP], rotation=R_DOWN,
                 seconds=1.5)
    api.log("a%d descend res %.4f eef %s" % (attempt, r,
                                             np.round(api.eef(), 4).tolist()))
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    held = (g["effort"] >= 3.0) and (HELD_W_LO <= g["width_m"] <= HELD_W_HI)
    api.log("a%d after close %s held=%s eef %s"
            % (attempt, g, held, np.round(api.eef(), 4).tolist()))

    yaw0 = _yaw(api.tool_rotation())
    press = [kx, ky, ztop - Z_PRESS_BELOW_TOP]
    for i in range(int(round(YAW_TOTAL_DEG / YAW_STEP_DEG))):
        tgt = yaw0 + np.radians(YAW_STEP_DEG * (i + 1))
        r = api.move(press, rotation=_rz(tgt) @ R_DOWN, seconds=0.7)
        if i % 3 == 2:
            api.log("a%d stage %2d want %+.1f got %+.1f posres %.4f grip %s"
                    % (attempt, i + 1, np.degrees(tgt),
                       np.degrees(_yaw(api.tool_rotation())), r, api.gripper()))
    turned = np.degrees(_yaw(api.tool_rotation()) - yaw0)
    api.log("a%d turned %+.1f deg (held=%s) eef %s"
            % (attempt, turned, held, np.round(api.eef(), 4).tolist()))
    return float(turned), held


def run(api):
    api.log("instruction: %r" % api.instruction())
    api.log("start eef %s rot %s"
            % (np.round(api.eef(), 4).tolist(),
               np.round(api.tool_rotation(), 4).tolist()))

    f = api.capture("cam_high")
    xy, ztop, why = find_knob(api, f)
    if xy is None:
        api.log("knob gate FAILED (%s) -> pack-mean fallback" % why)
        xy, ztop = PRIOR_XY, FALLBACK_ZTOP

    turned, held = grasp_and_turn(api, xy, ztop, 1)

    # A wrist that never moved did not engage the knob. Re-perceive from the
    # retreated pose and try once more. On every debug seed the first attempt
    # already turns the knob, so this branch is dead there by construction.
    if turned < YAW_OK_DEG:
        api.log("attempt 1 insufficient turn (%.1f deg) -> retry" % turned)
        api.grip(0.06)
        api.move([xy[0], xy[1], ztop + Z_HOVER_ABOVE_TOP], rotation=R_DOWN,
                 seconds=1.5)
        f2 = api.capture("cam_high")
        xy2, ztop2, why2 = find_knob(api, f2)
        if xy2 is None:
            api.log("retry gate FAILED (%s) -> reuse attempt-1 target" % why2)
            xy2, ztop2 = xy, ztop
        t2, h2 = grasp_and_turn(api, xy2, ztop2, 2)
        turned, held = max(turned, t2), (held or h2)

    return "v2: turned %+.1f deg held=%s" % (turned, held)
