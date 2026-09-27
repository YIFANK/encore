"""v9 — open the top drawer, then drop the cream cheese into it.

Mechanism, in the order the debug seeds forced it:

* The handle cannot be taken from above. The slot between the bar's back face
  (y = face + 0.013) and the drawer front is roofed by the cabinet top, which
  overhangs to y = face + 0.003; v1's top-down descent stalled with the
  fingertip at 1.1276 against the measured 1.1274 cabinet top.
* So the wrist turns 90 deg: approach axis -y, jaws separating along z, and
  they straddle the bar top-to-bottom in open air. The closed gap then reads
  0.0174, which is the bar diameter measured from depth — the receipt that the
  bar and not air is in the jaws.
* Every commanded pose lands ~0.011 m short, because the mover stops at its
  own 0.012 position tolerance. Anything that has to be accurate (the grasp,
  the release) goes through a bounded bias-cancelling loop.
* Every transit between the drawer side and the table side now goes over
  z 1.22. The drawer rim measures 1.1234 and the home pose sits at 1.165, so
  the direct move was scraping the drawer's front wall.
* v4/v5's last loss was the same kind of jam one stage later. Joint logs on
  seeds 57 and 63 show identical arm configurations at the hover and then,
  under one saturated descent command, joint 4 running from -1.87 straight to
  its -0.0695 limit on 63 alone. Descents are now walked down in 0.035 m
  rungs, which command 0.7 of full scale and never saturate.
* v3 lost two seeds to an arm that jammed on the way from the drawer to the
  cream cheese: the lift after a good grasp (width 0.0422, effort 3.0) burned
  its whole step budget and rose 44 mm. Both phases now start from the home
  pose, which is also how both packs start their demos.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-65 cam_high depth: median z over the "
                          "open table = 0.9014", "allowed": True},
    "TIP_OFFSET": {"source": "debug seed 51 cam_arm_wrist depth: the robot's "
                             "own geometry bottoms out at base z 1.164 while "
                             "api.eef() reads 1.1733", "allowed": True},
    "FACE_BAND": {"source": "debug-seed observation: the cabinet front face is "
                            "the dominant y-plane over z in (0.95,1.105), "
                            "x in (-0.10,0.20)", "allowed": True},
    "BAR_R": {"source": "debug seed 51 cross-section at the bar mid-x: the top "
                        "bar spans y -0.1405..-0.1264 and z 1.0928..1.0985",
              "allowed": True},
    "CAB_TOP_Z": {"source": "debug seeds 51-65: cabinet top surface z = 1.1274",
                  "allowed": True},
    "ROOF_RECEIPT": {"source": "v1 debug run PROBE A_gap: a top-down descent "
                               "stalled with the fingertip at 1.1276",
                     "allowed": True},
    "PULL_TO": {"source": "v7/v8 debug runs: seed 57 (drawer front left at "
                          "y -0.003) reaches the cream cheese, seed 63 (front "
                          "at +0.013) deadlocks; 0.022 puts the eef stop where "
                          "57's was. Travel is then 0.14-0.16 m, the same "
                          "order as the k1 pack's own 0.158 m drag.",
                "allowed": True},
    "BAR_GRASP_DY": {"source": "v2 debug run: after the side-on close the eef "
                               "settled 0.0073 in +y of the perceived bar "
                               "centre", "allowed": True},
    "TIP_TO_FACE": {"source": "debug seeds 51-65: bar tip y - face y = 0.0343",
                    "allowed": True},
    "MOUTH_BACK": {"source": "v2 debug run post_open depth: the drawer cavity "
                             "spans y -0.145..-0.010 with its front panel at "
                             "y 0.004, so the cavity centre is 0.081 behind "
                             "the panel", "allowed": True},
    "DROP_Z": {"source": "v2 debug run post_open depth: cavity floor z 1.0565, "
                         "wall rim 1.1234", "allowed": True},
    "CC_BLUE": {"source": "debug seeds 51-65 cam_high RGB: exactly one table "
                          "cluster with B/(R+G+B) > 0.40, footprint "
                          "0.077 x 0.041, top z 0.9203", "allowed": True},
    "CC_GRASP_Z": {"source": "mate pack keyframe t=40: the cream cheese is "
                             "closed on at eef z 0.9104", "allowed": True},
    "CC_HOLD_MIN": {"source": "debug-seed measurement: the cream cheese is "
                              "0.038 wide in y, so a gap above 0.020 after the "
                              "lift means it is held", "allowed": True},
    "DITHER": {"source": "debug-seed measurement: the cream cheese footprint "
                         "is 0.065 long in x, so a 0.016 shift of the approach "
                         "stays on the slab", "allowed": True},
    "OVER_Z": {"source": "v2 debug run post_open depth: the open drawer's wall "
                         "rim is at z 1.1234, so 1.22 clears it with margin",
               "allowed": True},
    "HOME": {"source": "debug seeds 51-65: api.eef() at episode start = "
                       "(-0.2085, 0.0, 1.1733); both packs' ee_path also "
                       "starts there", "allowed": True},
    "PRECISE_LOOP": {"source": "v2/v3 debug runs: commanded poses land ~0.011 "
                               "short (the mover's own position tolerance)",
                     "allowed": True},
    "DESCENT_RUNG": {"source": "v4/v5/diag debug runs on seed 63: a saturated "
                               "descent command drove joint 4 to its -0.0695 "
                               "limit; 0.035 m rungs stay unsaturated against "
                               "the mover's own 0.05 m gain scale",
                     "allowed": True},
    "AIM_GUARD": {"source": "v4 debug run: on the seven clean seeds the "
                            "lateral error at the grasp never exceeded 0.005",
                  "allowed": True},
}

TIP_OFFSET = 0.009
BAR_R = 0.007
PULL_TO = 0.0220
BAR_GRASP_DY = 0.0073
TIP_TO_FACE = 0.0343
MOUTH_BACK = 0.081
DROP_Z = 1.1000
CC_GRASP_Z = 0.9104
CC_HOLD_MIN = 0.020
HOME = [-0.2085, 0.0, 1.1733]
OVER_Z = 1.2200
DITHER = ((0.0, 0.0), (0.016, 0.0))
CHUNK = 1800

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
R_SIDE = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])


# --------------------------------------------------------------------- utils

def _dump(api, tag, arr):
    raw = zlib.compress(np.ascontiguousarray(arr).tobytes(), 6)
    b64 = base64.b64encode(raw).decode()
    n = (len(b64) + CHUNK - 1) // CHUNK
    api.log("BLOB %s shape=%s dtype=%s nchunk=%d" %
            (tag, list(arr.shape), str(arr.dtype), n))
    for i in range(n):
        api.log("B %s %d %s" % (tag, i, b64[i * CHUNK:(i + 1) * CHUNK]))


def dump_scene(api, tag, stride=2):
    f = api.capture("cam_high")
    api.log("CAM cam_high K=%s T=%s" % (np.asarray(f.intrinsics).round(5).tolist(),
                                        np.asarray(f.t_base_cam).round(5).tolist()))
    rgb = np.asarray(f.rgb)[::stride, ::stride]
    d = np.asarray(f.depth, float)[::stride, ::stride]
    d = np.where(np.isfinite(d), d, 0.0)
    _dump(api, tag + "_rgb", rgb.astype(np.uint8))
    _dump(api, tag + "_d", np.clip(d * 1000.0, 0, 65000).astype(np.uint16))


def point_cloud(frame):
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    H, W = d.shape
    vs, us = np.mgrid[0:H, 0:W]
    xc = (us - K[0, 2]) * d / K[0, 0]
    yc = (vs - K[1, 2]) * d / K[1, 1]
    P = np.stack([xc, yc, d, np.ones_like(d)], -1) @ T.T
    ok = np.isfinite(P[..., :3]).all(-1) & (d > 0.05)
    return P[..., :3], ok, np.asarray(frame.rgb, float) / 255.0


def step(api, tag, xyz, rot, seconds=2.0, tries=2):
    r = 9.0
    for _ in range(tries):
        r = api.move(xyz, rotation=rot, seconds=seconds)
    e = np.asarray(api.eef(), float)
    api.log("MV %s tgt=%s eef=%s resid=%.4f"
            % (tag, [round(float(v), 4) for v in xyz], e.round(4).tolist(), r))
    return e


def ladder(api, tag, x, y, z_from, z_to, rot, rung=0.035, seconds=0.7):
    """Descend in rungs small enough that err/0.05 never saturates.

    A single 0.08 m descent command clips to full scale on z and carries
    whatever lateral error is left along with it; on debug seed 63 that drove
    joint 4 from -1.87 to its -0.0695 limit and wedged the arm 0.11 m off
    target. Rungs of 0.035 m command 0.7 of scale and keep the elbow bent."""
    z = z_from
    while z - z_to > 1e-4:
        z = max(z_to, z - rung)
        api.move([x, y, z], rotation=rot, seconds=seconds)
    e = np.asarray(api.eef(), float)
    api.log("LD %s to=%.4f eef=%s" % (tag, z_to, e.round(4).tolist()))
    return e


def ladder_up(api, tag, x, y, z_from, z_to, rot, rung=0.035, seconds=0.7):
    """The same unsaturated rungs, upward.

    v7 receipt (seed 55): a good grasp (width 0.0422, effort 3.0) was thrown
    away because one saturated 0.18 m lift command stalled at 0.956."""
    z = z_from
    while z_to - z > 1e-4:
        z = min(z_to, z + rung)
        api.move([x, y, z], rotation=rot, seconds=seconds)
    e = np.asarray(api.eef(), float)
    api.log("LU %s to=%.4f eef=%s" % (tag, z_to, e.round(4).tolist()))
    return e


def precise(api, tag, tgt, rot, seconds=1.5, iters=4, tol=0.003, bound=0.05):
    tgt = np.asarray(tgt, float)
    cmd = tgt.copy()
    e = np.asarray(api.eef(), float)
    for i in range(iters):
        api.move(cmd.tolist(), rotation=rot, seconds=seconds)
        e = np.asarray(api.eef(), float)
        err = tgt - e
        api.log("PR %s i=%d cmd=%s eef=%s err=%.4f"
                % (tag, i, cmd.round(4).tolist(), e.round(4).tolist(),
                   float(np.linalg.norm(err))))
        if float(np.linalg.norm(err)) < tol:
            break
        cmd = np.clip(cmd + err, tgt - bound, tgt + bound)
    return e


# ---------------------------------------------------------------- perception

def perceive(api, tag):
    f = api.capture("cam_high")
    P, ok, rgb = point_cloud(f)
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    band = ok & (z > 0.95) & (z < 1.105) & (y > -0.35) & (y < -0.05) \
        & (x > -0.10) & (x < 0.20)
    edges = np.arange(-0.35, -0.05, 0.004)
    hist, _ = np.histogram(y[band], bins=edges)
    face_y = float(edges[int(hist.argmax())] + 0.002)
    hand = band & (y > face_y + 0.012)
    zt = float(z[hand].max())
    top = hand & (z > zt - 0.022)
    out = {"face_y": face_y, "bar_top_z": zt,
           "bar_tip_y": float(np.percentile(y[top], 99)),
           "bar_x_lo": float(np.percentile(x[top], 1)),
           "bar_x_hi": float(np.percentile(x[top], 99))}
    out["bar_x_mid"] = 0.5 * (out["bar_x_lo"] + out["bar_x_hi"])
    api.log("%s %s" % (tag, {k: round(v, 4) for k, v in out.items()}))
    return out, (P, ok, rgb)


def find_cream_cheese(api, scene, tag):
    P, ok, rgb = scene
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    s = rgb.sum(-1) + 1e-6
    sel = ok & (z > 0.905) & (z < 0.95) & (y > -0.05) & (y < 0.45) \
        & (x > -0.35) & (x < 0.35) & (rgb[..., 2] / s > 0.40)
    n = int(sel.sum())
    if n < 40:
        api.log("CC %s none n=%d" % (tag, n))
        return None
    out = {"n": n,
           "cx": 0.5 * (float(x[sel].min()) + float(x[sel].max())),
           "cy": 0.5 * (float(y[sel].min()) + float(y[sel].max())),
           "xw": float(x[sel].max() - x[sel].min()),
           "yw": float(y[sel].max() - y[sel].min()),
           "ztop": float(z[sel].max())}
    api.log("CC %s %s" % (tag, {k: (round(v, 4) if isinstance(v, float) else v)
                                for k, v in out.items()}))
    # the robot's own body is blue too: a real cream cheese is a flat slab
    if not (0.03 < out["xw"] < 0.12 and 0.02 < out["yw"] < 0.075):
        api.log("CC %s rejected (footprint)" % tag)
        return None
    return out


# ---------------------------------------------------------------------- task

def run(api):
    api.log("instruction=%r" % api.instruction())
    api.grip(0.08)
    api.settle(0.3)
    api.log("grip_open=%s" % api.gripper())

    cab, scene0 = perceive(api, "CAB0")
    cc = find_cream_cheese(api, scene0, "t0")

    xb = cab["bar_x_mid"]
    bar_y = cab["bar_tip_y"] - BAR_R
    bar_z = cab["bar_top_z"] - BAR_R
    api.log("PLAN xb=%.4f bar_y=%.4f bar_z=%.4f face=%.4f"
            % (xb, bar_y, bar_z, cab["face_y"]))

    # ---- open the top drawer: side-on bar grasp, then drag +y ----
    step(api, "stage", [xb, bar_y + 0.18, 1.15], R_SIDE, seconds=2.5, tries=2)
    step(api, "front", [xb, bar_y + 0.09, bar_z], R_SIDE, seconds=2.0, tries=1)
    step(api, "at_bar", [xb, bar_y + 0.006, bar_z], R_SIDE, seconds=2.0, tries=2)
    api.grip(0.0)
    api.settle(0.4)
    gh = api.gripper()
    api.log("CLOSED grip=%s eef=%s" % (gh, np.asarray(api.eef()).round(4).tolist()))

    # Pull to an ABSOLUTE stop, not a fixed distance. The cabinet's own y
    # varies by 2 cm across seeds, so a fixed 0.175 m drag left the drawer
    # 16 mm further out on seed 63 than on seed 57 -- far enough forward that
    # the forearm grazed its front wall on the way to the cream cheese and the
    # descent deadlocked at z 0.988 over bare table.
    y0 = float(api.eef()[1])
    y = y0
    while PULL_TO - y > 1e-4:
        y = min(PULL_TO, y + 0.030)
        api.move([xb, y, bar_z], rotation=R_SIDE, seconds=1.2)
    y_end = float(api.eef()[1])
    api.log("PULLED y0=%.4f y_end=%.4f grip=%s" % (y0, y_end, api.gripper()))

    api.grip(0.08)
    api.settle(0.3)
    step(api, "unhook", [xb, y_end + 0.10, 1.20], R_SIDE, seconds=1.5, tries=1)
    step(api, "home1", HOME, R_DOWN, seconds=2.5, tries=2)

    bar_end_y = y_end - BAR_GRASP_DY
    front_y = bar_end_y - TIP_TO_FACE + BAR_R
    drop_x = xb
    drop_y = front_y - MOUTH_BACK
    api.log("MOUTH bar_end=%.4f front=%.4f drop=(%.4f, %.4f)"
            % (bar_end_y, front_y, drop_x, drop_y))

    # ---- pick the cream cheese ----
    held = False
    for attempt in range(2):
        cc2 = find_cream_cheese(api, perceive(api, "CABp%d" % attempt)[1],
                                "p%d" % attempt) or cc
        if cc2 is None:
            break
        api.grip(0.08)
        api.settle(0.2)
        # Cross the open drawer ABOVE its rim. The home pose sits at z 1.165
        # and the drawer rim measures 1.1234, so a direct home -> hover move
        # drags the hand through the drawer's front wall on the way out.
        step(api, "over%d" % attempt, [cc2["cx"], cc2["cy"], OVER_Z],
             R_DOWN, seconds=2.5, tries=2)
        zg = CC_GRASP_Z - 0.004 * attempt
        e = None
        for dx, dy in DITHER:
            gx, gy = cc2["cx"] + dx, cc2["cy"] + dy
            h = step(api, "hov%d%+.0f" % (attempt, dx * 1000),
                     [gx, gy, 1.05], R_DOWN, seconds=1.8, tries=2)
            e = ladder(api, "cc%d%+.0f" % (attempt, dx * 1000), gx, gy,
                       float(h[2]), zg, R_DOWN)
            off = float(np.hypot(e[0] - gx, e[1] - gy))
            api.log("TRY a=%d dx=%.3f z=%.4f (want %.4f) off=%.4f"
                    % (attempt, dx, e[2], zg, off))
            if e[2] < zg + 0.018 and off < 0.020:
                break
            # A stalled descent with the arm still on aim is a configuration
            # deadlock, not contact: the scene depth under the jaws is bare
            # table on the seed where this bites. Shifting the approach a
            # centimetre re-solves the arm without leaving the 0.065 x 0.038
            # footprint.
            step(api, "unstick%d" % attempt, [gx, gy, OVER_Z], R_DOWN,
                 seconds=2.0, tries=1)
        if e is None or e[2] > zg + 0.030:
            step(api, "home_retry", HOME, R_DOWN, seconds=2.0, tries=1)
            continue
        precise(api, "cc%dz" % attempt, [float(e[0]), float(e[1]), zg],
                R_DOWN, seconds=1.0, iters=2, bound=0.02)
        api.grip(0.0)
        api.settle(0.4)
        g = api.gripper()
        api.log("CC_CLOSE%d grip=%s" % (attempt, g))
        e = np.asarray(api.eef(), float)
        e = ladder_up(api, "lift%d" % attempt, float(e[0]), float(e[1]),
                      float(e[2]), 1.10, R_DOWN)
        g = api.gripper()
        api.log("CC_LIFT%d grip=%s" % (attempt, g))
        if g["width_m"] > CC_HOLD_MIN:
            held = True
            break
        step(api, "up_retry2", [float(e[0]), float(e[1]), OVER_Z], R_DOWN,
             seconds=2.0, tries=1)
        step(api, "home_retry", HOME, R_DOWN, seconds=2.0, tries=1)
    api.log("HELD=%s" % held)

    # ---- carry it over the open drawer and release inside ----
    ee = np.asarray(api.eef(), float)
    ladder_up(api, "rise", float(ee[0]), float(ee[1]), float(ee[2]), OVER_Z,
              R_DOWN)
    e = step(api, "transit", [drop_x, drop_y, OVER_Z], R_DOWN, seconds=3.0, tries=2)
    ladder(api, "drop", drop_x, drop_y, float(e[2]), DROP_Z, R_DOWN)
    precise(api, "drop", [drop_x, drop_y, DROP_Z], R_DOWN, seconds=1.0, iters=2,
            bound=0.02)
    api.log("AT_DROP grip=%s" % api.gripper())
    api.grip(0.08)
    api.settle(0.8)
    step(api, "clear", [drop_x, drop_y, OVER_Z], R_DOWN, seconds=2.0, tries=1)
    api.settle(0.5)
    dump_scene(api, "post_drop")
    return "v9 held=%s" % held
