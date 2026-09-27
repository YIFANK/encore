"""c2clean goal_put_bowl_on_plate_task_k3 -- "Put the wine bottle on the plate".

Mechanism
---------
The intent names an OBJECT (the wine bottle) and a TARGET (the plate).
* The mate pack ("put the wine bottle on top of the cabinet") is shot in THIS
  scene and shows how the bottle is taken: the closing keyframe sits 0.032 m
  below the bottle's top (two of three demos) and the held finger gap there is
  0.016 m -- a NECK grasp.  The third demo closes lower on the shoulder at a
  0.025 m gap.  Both are the same top-down straight-wrist approach.
* The k3 pack ("put the bowl on the plate") acts on THIS target: its release
  keyframes land at x ~ +0.048, which is where the plate is.
Neither pack is the task, so the object and the target are both perceived from
cam_high depth on the seed itself; the packs supply only the grasp offset and
the confirmation that the plate is the front disc.

Perception (cam_high RGB-D, arm at home, nothing occluded)
  bottle  -- the only dark, tall thing in the mid-table crop; its top slice is
             the neck, whose axis is (max_x - neck_radius, y-midline).
  plate   -- the low flat disc left over after the bowl's own rim disc is cut
             out of the 0.9035..0.9215 m band.
Heights are differences of two depth measurements (plate floor - table), so a
common depth bias cancels.
"""
import json

import numpy as np

PROVENANCE = {
    "GRASP_BELOW_TOP": {
        "source": "mate pack keyframes: closing EEF z 1.0291 (demo0) and 1.0238 "
                  "(demo2) against a bottle top measured at 1.0553-1.0586 m on "
                  "debug seeds 51/53/57/61/65 -> 0.030-0.035 m below the top; "
                  "held finger gap 0.0165/0.0157 m matches the 0.0134 m neck "
                  "diameter measured from debug-seed depth",
        "allowed": True},
    "NECK_GAP_MAX": {
        "source": "mate pack keyframes gripper_state at the hold: gaps 0.0165, "
                  "0.0246, 0.0157 m -- an upper bound on a real bottle hold",
        "allowed": True},
    "WS_X": {"source": "debug-seed cam_high depth: table props deproject to "
                       "x in [-0.21, +0.12]; the far black prop sits at "
                       "x=-0.389 and the walls beyond -0.4", "allowed": True},
    "WS_Y": {"source": "debug-seed cam_high depth: the cabinet block occupies "
                       "y < -0.12, the stove y > +0.10", "allowed": True},
    "BOTTLE_BAND": {"source": "debug-seed depth profile of the bottle: body top "
                              "0.975, neck 1.005-1.059 m; nothing else in the "
                              "crop rises above 0.97 m (arm at home clears 1.14)",
                    "allowed": True},
    "BOWL_BAND": {"source": "debug-seed depth: the bowl rim tops out at 0.952 m, "
                            "the plate rim at 0.920 m", "allowed": True},
    "PLATE_BAND": {"source": "debug-seed depth: table plane 0.9010 m, plate "
                             "floor 0.9080 m, plate rim 0.920 m", "allowed": True},
    "PLATE_FLOOR_OVER_TABLE": {
        "source": "debug-seed depth, 5 seeds: median plate-centre z minus median "
                  "bare-table z = 0.0070-0.0074 m", "allowed": True},
    "CARRY_Z": {"source": "debug-seed depth: tallest obstacle on the bottle->plate "
                          "path is the bowl rim at 0.952 m; the bottle base hangs "
                          "0.126 m under the neck grip", "allowed": True},
    "HOVER_Z": {"source": "debug-seed observation: arm home EEF z = 1.173 m, "
                          "bottle top 1.059 m", "allowed": True},
    "OPEN_W": {"source": "generic gripper mechanics: fair_run grip(width) is an "
                         "intent flag; >=0.025 opens", "allowed": True},
    "CLOSE_W": {"source": "generic gripper mechanics: <0.025 closes", "allowed": True},
}

WS_X = (-0.28, 0.20)
WS_Y = (-0.30, 0.30)
BOTTLE_X = (-0.28, -0.05)
BOTTLE_Y = (-0.12, 0.06)
BOTTLE_BAND = (0.97, 1.10)
BOTTLE_DARK = 80.0
BOWL_BAND = (0.932, 0.958)
PLATE_BAND = (0.9035, 0.9215)
PLATE_Y = (-0.12, 0.10)
PLATE_BRIGHT = 90.0
GRASP_BELOW_TOP = 0.032
NECK_GAP_MAX = 0.030
PLATE_FLOOR_OVER_TABLE = 0.0070
CARRY_Z = 1.13
HOVER_Z = 1.15
OPEN_W = 0.08
CLOSE_W = 0.0


def _cloud(f):
    dep = np.nan_to_num(np.asarray(f.depth, float), nan=0.0)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = dep.shape
    v, u = np.mgrid[0:H, 0:W]
    x = (u - K[0, 2]) / K[0, 0] * dep
    y = (v - K[1, 2]) / K[1, 1] * dep
    P = np.stack([x, y, dep, np.ones_like(dep)], -1)
    B = P @ T.T
    return B[..., :3]


def perceive(api):
    f = api.capture("cam_high")
    B = _cloud(f)
    L = np.asarray(f.rgb, float).mean(-1)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    ok = np.isfinite(Z) & (Z > 0.3)
    ws = ok & (X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1])
    out = {}

    # table plane: the modal height of the cropped workspace
    zs = Z[ws]
    hist, edges = np.histogram(zs, bins=np.arange(0.80, 1.30, 0.002))
    table = float(edges[int(hist.argmax())] + 0.001)
    out["table"] = table

    # --- bottle: dark + tall -------------------------------------------------
    mb = (ok & (X > BOTTLE_X[0]) & (X < BOTTLE_X[1])
          & (Y > BOTTLE_Y[0]) & (Y < BOTTLE_Y[1])
          & (Z > BOTTLE_BAND[0]) & (Z < BOTTLE_BAND[1]) & (L < BOTTLE_DARK))
    P = B[mb]
    out["n_bottle"] = int(mb.sum())
    if len(P) >= 40:
        top = float(P[:, 2].max())
        sl = P[P[:, 2] > top - 0.025]
        ymid = float((sl[:, 1].max() + sl[:, 1].min()) / 2.0)
        rad = float((sl[:, 1].max() - sl[:, 1].min()) / 2.0)
        rad = min(max(rad, 0.004), 0.012)
        out["bottle"] = (float(sl[:, 0].max()) - rad, ymid, top)
        out["neck_r"] = rad
        out["n_neck"] = int(len(sl))

    # --- bowl rim disc, to cut it out of the plate band ----------------------
    mw = (ok & (X > WS_X[0]) & (X < WS_X[1])
          & (Y > PLATE_Y[0]) & (Y < PLATE_Y[1])
          & (Z > BOWL_BAND[0]) & (Z < BOWL_BAND[1]) & (L > 60.0))
    Q = B[mw]
    if len(Q) >= 100:
        bx, by = float(Q[:, 0].mean()), float(Q[:, 1].mean())
        br = float(np.sqrt((Q[:, 0] - bx) ** 2 + (Q[:, 1] - by) ** 2).max())
        out["bowl"] = (bx, by, min(br, 0.13))
    else:
        out["bowl"] = (-0.11, 0.0, 0.10)

    # --- plate: the low flat disc that is not the bowl -----------------------
    mp = (ok & (X > WS_X[0]) & (X < WS_X[1])
          & (Y > PLATE_Y[0]) & (Y < PLATE_Y[1])
          & (Z > PLATE_BAND[0]) & (Z < PLATE_BAND[1]) & (L > PLATE_BRIGHT))
    R = B[mp]
    bx, by, br = out["bowl"]
    if len(R):
        d = np.sqrt((R[:, 0] - bx) ** 2 + (R[:, 1] - by) ** 2)
        R = R[d > br + 0.012]
    out["n_plate"] = int(len(R))
    if len(R) >= 500:
        cx, cy = float(R[:, 0].mean()), float(R[:, 1].mean())
        for _ in range(2):
            d = np.sqrt((R[:, 0] - cx) ** 2 + (R[:, 1] - cy) ** 2)
            S = R[d < 0.09]
            if len(S) < 200:
                break
            cx, cy = float(S[:, 0].mean()), float(S[:, 1].mean())
        d = np.sqrt((R[:, 0] - cx) ** 2 + (R[:, 1] - cy) ** 2)
        out["plate"] = (cx, cy, float(np.percentile(d, 95)))
        near = R[d < 0.03]
        out["plate_floor"] = (float(np.median(near[:, 2])) if len(near) > 50
                              else table + PLATE_FLOOR_OVER_TABLE)
    return out


def _held(api):
    g = api.gripper()
    return (float(g["effort"]) > 1.0), float(g["width_m"])


def run(api):
    api.log("INSTR %r" % (api.instruction(),))
    S = perceive(api)
    api.log("PERCEIVE " + json.dumps({k: (list(v) if isinstance(v, tuple) else v)
                                      for k, v in S.items()}))
    if "bottle" not in S or "plate" not in S:
        api.log("ABORT: perception incomplete")
        return

    bx, by, btop = S["bottle"]
    px, py, prad = S["plate"]
    table = S["table"]
    pfloor = S.get("plate_floor", table + PLATE_FLOOR_OVER_TABLE)

    grasp_z = btop - GRASP_BELOW_TOP
    # the grip sits this far above the bottle's own base (it stands on the table)
    lift_above_base = grasp_z - table
    release_z = pfloor + lift_above_base
    api.log("PLAN grasp=(%.4f,%.4f,%.4f) plate=(%.4f,%.4f) r=%.3f "
            "pfloor=%.4f table=%.4f release_z=%.4f"
            % (bx, by, grasp_z, px, py, prad, pfloor, table, release_z))

    api.grip(OPEN_W)
    api.move([bx, by, HOVER_Z], seconds=1.5)

    held = False
    for attempt in range(2):
        r = api.move([bx, by, grasp_z], seconds=1.5)
        api.log("DESCEND%d residual=%.4f eef=%s" % (attempt, r, [round(float(v), 4) for v in api.eef()]))
        api.grip(CLOSE_W)
        held, w = _held(api)
        api.log("CLOSE%d held=%s gap=%.4f" % (attempt, held, w))
        if held and w <= NECK_GAP_MAX:
            break
        held = False
        api.grip(OPEN_W)
        api.move([bx, by, HOVER_Z], seconds=1.2)
        S2 = perceive(api)
        api.log("REPERCEIVE " + json.dumps({k: (list(v) if isinstance(v, tuple) else v)
                                            for k, v in S2.items()}))
        if "bottle" in S2:
            bx, by, btop = S2["bottle"]
            grasp_z = btop - GRASP_BELOW_TOP
            release_z = pfloor + (grasp_z - table)

    if not held:
        api.log("ABORT: no hold after 2 attempts")
        return

    api.move([bx, by, CARRY_Z], seconds=1.5)
    h, w = _held(api)
    api.log("LIFT held=%s gap=%.4f eef=%s" % (h, w, [round(float(v), 4) for v in api.eef()]))
    api.move([px, py, CARRY_Z], seconds=2.0)

    # closed-loop descent onto the plate: re-issue with the tracking bias
    # folded into the command until the tool is within 4 mm of the seat.
    target = release_z
    cmd = target
    prev = None
    for i in range(3):
        r = api.move([px, py, cmd], seconds=1.2)
        z = float(api.eef()[2])
        api.log("SEAT%d cmd=%.4f z=%.4f residual=%.4f" % (i, cmd, z, r))
        if z <= target + 0.004:
            break
        if prev is not None and abs(prev - z) < 0.002:
            api.log("SEAT stalled -- treating as contact")
            break
        prev = z
        cmd = cmd - (z - target)
        if cmd < target - 0.030:
            break

    h, w = _held(api)
    api.log("PRE_RELEASE held=%s gap=%.4f eef=%s" % (h, w, [round(float(v), 4) for v in api.eef()]))
    api.grip(OPEN_W)
    api.settle(0.5)
    api.move([px, py, CARRY_Z], seconds=1.2)
    api.settle(0.3)

    V = perceive(api)
    api.log("VERIFY " + json.dumps({k: (list(v) if isinstance(v, tuple) else v)
                                    for k, v in V.items()}))
