"""c2clean obj_salad_dressing_pos_k3 -- v4.

v1 commanded the grasp point in one saturated move and stopped 20 mm short with
a +16 mm lateral bias (measured, both attempts, both seeds).  v2's ladder showed
that 1 cm increments walk the same arm 23 mm lower.  v3 therefore descends in
increments and cancels the lateral bias from the eef it actually reaches, and
succeeded on both probe seeds.  v4 adds an over-the-target approach waypoint, a
stall-tolerant descent that starts from the height actually reached, and one
re-perceive-and-retry if the close comes up empty.

v7 fixes two failures the v5 aim-envelope probe exposed at the reach envelope:
the lateral refinement used to command the height it had reached, and the arm
bought that lateral correction by RISING 26 mm before the close (measured
0.1382 -> 0.1641); and `held` was believed at a finger gap of 0.0800, i.e. a
fully open hand that never closed because the horizon was already spent.  So the
refinement now commands the descent target height, a bounded press keeps
driving down while the height still improves, and a hold must show a gap
narrower than the open hand.
"""
import numpy as np

PROVENANCE = {
    "WORKSPACE_X": {"source": "debug seeds 51-65 cam_high point cloud: table "
                              "props all fall in x[-0.25,0.40]", "allowed": True},
    "WORKSPACE_Y": {"source": "debug seeds 51-65 cam_high point cloud: table "
                              "props all fall in y[-0.42,0.48]", "allowed": True},
    "Z_TABLE_CLEAR": {"source": "debug-seed point cloud: floor plane sits at "
                                "z~0.005, props start above 0.02", "allowed": True},
    "Z_PROP_MAX": {"source": "debug-seed point cloud: every table prop tops out "
                             "below 0.15; only the arm is above 0.22", "allowed": True},
    "GREEN_MIN": {"source": "debug seeds 51-65: top-band (g-max(r,b)) is +19.5 "
                            "for the green bottle and <= -0.2 for every other "
                            "prop and the basket", "allowed": True},
    "TOP_BAND_M": {"source": "debug-seed measurement: the cap top face spans "
                             "~6 mm of depth noise", "allowed": True},
    "GRASP_BELOW_TOP": {"source": "pack.json K=3 demos: mean close EEF z 0.1254 "
                                  "vs bottle top 0.1475 measured on debug seeds",
                        "allowed": True},
    "HOVER_Z": {"source": "debug-seed v2 ladder: 0.20 is reachable everywhere "
                          "in the workspace", "allowed": True},
    "APPROACH_Z": {"source": "debug-seed point cloud: the tallest thing on the "
                             "floor is 0.148, so 0.24 clears every prop",
                   "allowed": True},
    "DESCEND_STEP": {"source": "debug-seed v2 ladder: 1 cm increments track, one "
                               "big move stalls 23 mm higher", "allowed": True},
    "BIAS_GAIN": {"source": "debug-seed v1/v2: the descent lags +8..16 mm in x; "
                            "gain<1 with a clip keeps the correction bounded",
                  "allowed": True},
    "BIAS_CLIP": {"source": "debug-seed v1: observed lateral lag never exceeded "
                            "16 mm; 50 mm is a hard guard", "allowed": True},
    "STALL_DZ": {"source": "generic controller mechanics: POS_TOL is 12 mm, so a "
                           "step that buys under 2 mm of z is a stall",
                 "allowed": True},
    "CARRY_Z": {"source": "pack.json demos ee_path6: transit apex z ~0.31",
                "allowed": True},
    "RELEASE_ABOVE_RIM": {"source": "pack.json demos: release EEF z 0.176-0.199 "
                                    "vs basket rim 0.143 on debug seeds",
                          "allowed": True},
    "BASKET_MIN_Z": {"source": "debug-seed point cloud: the basket rim is the "
                               "only structure above 0.09 beside the props",
                     "allowed": True},
    "GRID_CELL": {"source": "generic occupancy-grid mechanics", "allowed": True},
    "HELD_GAP_MIN": {"source": "FairApi mechanics: effort is 3.0 only while the "
                               "finger gap stays open on something", "allowed": True},
    "HELD_GAP_MAX": {"source": "debug-seed measurement: the open hand reports "
                               "0.0778-0.0800 and a real hold on this bottle "
                               "reports 0.0368; 0.060 separates them",
                     "allowed": True},
    "PRESS_OVER": {"source": "debug-seed v2 ladder: commanding below the goal "
                             "keeps the proportional term saturated and buys "
                             "further descent", "allowed": True},
    "PRESS_TRIES": {"source": "debug-seed step accounting: 3 presses fit the "
                              "500-step horizon alongside a retry",
                    "allowed": True},
}

XLO, XHI = -0.25, 0.40
YLO, YHI = -0.42, 0.48
Z_TABLE_CLEAR = 0.02
Z_PROP_MAX = 0.22
GREEN_MIN = 10.0
TOP_BAND_M = 0.006
GRASP_BELOW_TOP = 0.022
HOVER_Z = 0.20
APPROACH_Z = 0.24
DESCEND_STEP = 0.010
BIAS_GAIN = 0.8
BIAS_CLIP = 0.05
STALL_DZ = 0.002
CARRY_Z = 0.30
RELEASE_ABOVE_RIM = 0.049
BASKET_MIN_Z = 0.09
GRID_CELL = 0.01
HELD_GAP_MIN = 0.005
HELD_GAP_MAX = 0.060
PRESS_OVER = 0.020
PRESS_TRIES = 3


# ---------------------------------------------------------------- perception
def _cloud(f):
    depth = np.asarray(f.depth, dtype=np.float64)
    h, w = depth.shape
    K = np.asarray(f.intrinsics, dtype=np.float64)
    T = np.asarray(f.t_base_cam, dtype=np.float64)
    vv, uu = np.mgrid[0:h, 0:w]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    z = np.where(np.isfinite(depth), depth, 0.0)
    P = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z, np.ones_like(z)], -1)
    B = P @ T.T
    return B[..., 0], B[..., 1], B[..., 2]


def _largest_component(grid):
    seen = np.zeros(grid.shape, bool)
    best, bestn = None, 0
    for s in np.argwhere(grid):
        if seen[s[0], s[1]]:
            continue
        comp = np.zeros(grid.shape, bool)
        comp[s[0], s[1]] = True
        while True:
            g = comp.copy()
            g[1:, :] |= comp[:-1, :]
            g[:-1, :] |= comp[1:, :]
            g[:, 1:] |= comp[:, :-1]
            g[:, :-1] |= comp[:, 1:]
            g &= grid
            if int(g.sum()) == int(comp.sum()):
                break
            comp = g
        seen |= comp
        n = int(comp.sum())
        if n > bestn:
            bestn, best = n, comp
    return best, bestn


def perceive(api):
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb)
    x, y, z = _cloud(f)
    ok = (z > Z_TABLE_CLEAR) & (z < Z_PROP_MAX) \
        & (x > XLO) & (x < XHI) & (y > YLO) & (y < YHI)
    r = rgb[..., 0].astype(float)
    g = rgb[..., 1].astype(float)
    b = rgb[..., 2].astype(float)
    green = ok & ((g - np.maximum(r, b)) > GREEN_MIN)

    out = {}
    if int(green.sum()) >= 20:
        zt = float(np.percentile(z[green], 98))
        top = green & (z > zt - TOP_BAND_M)
        if int(top.sum()) < 8:
            top = green & (z > zt - 2 * TOP_BAND_M)
        out["target"] = (float(x[top].mean()), float(y[top].mean()), zt,
                         float(y[top].max() - y[top].min()), int(green.sum()))

    rim = ok & (z > BASKET_MIN_Z) & (~green)
    nx = int((XHI - XLO) / GRID_CELL) + 1
    ny = int((YHI - YLO) / GRID_CELL) + 1
    cnt = np.zeros((nx, ny), int)
    hi = np.zeros((nx, ny))
    ix = ((x[rim] - XLO) / GRID_CELL).astype(int)
    iy = ((y[rim] - YLO) / GRID_CELL).astype(int)
    np.add.at(cnt, (ix, iy), 1)
    np.maximum.at(hi, (ix, iy), z[rim])
    comp, n = _largest_component(cnt >= 3)
    if comp is not None:
        cx_, cy_ = np.nonzero(comp)
        out["basket"] = (float(XLO + (cx_.mean() + 0.5) * GRID_CELL),
                         float(YLO + (cy_.mean() + 0.5) * GRID_CELL),
                         float(np.median(hi[comp])), n)
    return out


# -------------------------------------------------------------------- motion
def walk_down(api, R, tx, ty, z_to):
    """Descend in DESCEND_STEP increments, cancelling the lateral lag."""
    bx = by = 0.0
    z = min(HOVER_Z, float(api.eef()[2]))
    prev = None
    stalls = 0
    while z > z_to + 1e-9:
        z = max(z_to, z - DESCEND_STEP)
        api.move([tx + bx, ty + by, z], rotation=R, seconds=0.6)
        e = api.eef()
        bx = float(np.clip(bx - BIAS_GAIN * (e[0] - tx), -BIAS_CLIP, BIAS_CLIP))
        by = float(np.clip(by - BIAS_GAIN * (e[1] - ty), -BIAS_CLIP, BIAS_CLIP))
        api.log("  step z=%.3f eef=%.4f,%.4f,%.4f bias=%.4f,%.4f"
                % (z, e[0], e[1], e[2], bx, by))
        stalls = stalls + 1 if (prev is not None and (prev - e[2]) < STALL_DZ) else 0
        prev = e[2]
        if stalls >= 2:
            api.log("  stalled at %.4f" % e[2])
            break
    # press: keep commanding below the goal while the height still improves
    for _ in range(PRESS_TRIES):
        e = api.eef()
        if e[2] <= z_to + STALL_DZ:
            break
        api.move([tx + bx, ty + by, z_to - PRESS_OVER], rotation=R, seconds=0.8)
        e2 = api.eef()
        bx = float(np.clip(bx - BIAS_GAIN * (e2[0] - tx), -BIAS_CLIP, BIAS_CLIP))
        by = float(np.clip(by - BIAS_GAIN * (e2[1] - ty), -BIAS_CLIP, BIAS_CLIP))
        api.log("  press eef=%.4f,%.4f,%.4f bias=%.4f,%.4f"
                % (e2[0], e2[1], e2[2], bx, by))
        if (e[2] - e2[2]) < STALL_DZ:
            break
    # lateral refinement, always commanding the GOAL height: at the reach
    # envelope the arm buys lateral correction by rising, and commanding the
    # height it has reached let it climb 26 mm before the close (v5 probe).
    for _ in range(2):
        e = api.eef()
        if abs(e[0] - tx) < 0.003 and abs(e[1] - ty) < 0.003:
            break
        bx = float(np.clip(bx - BIAS_GAIN * (e[0] - tx), -BIAS_CLIP, BIAS_CLIP))
        by = float(np.clip(by - BIAS_GAIN * (e[1] - ty), -BIAS_CLIP, BIAS_CLIP))
        api.move([tx + bx, ty + by, min(z_to, e[2])], rotation=R, seconds=0.6)
        e = api.eef()
        api.log("  refine eef=%.4f,%.4f,%.4f bias=%.4f,%.4f"
                % (e[0], e[1], e[2], bx, by))
    return api.eef()


def run(api):
    R = np.asarray(api.tool_rotation(), dtype=float)
    api.grip(0.08)
    p = perceive(api)
    api.log("perceive %s" % {k: np.round(v, 4).tolist() for k, v in p.items()})
    if "target" not in p or "basket" not in p:
        return "perception failed: %s" % list(p)

    tx, ty, zt = p["target"][0], p["target"][1], p["target"][2]
    bx, by, rim = p["basket"][0], p["basket"][1], p["basket"][2]
    z_grasp = zt - GRASP_BELOW_TOP
    api.log("plan target=(%.4f,%.4f,%.4f) z_grasp=%.4f basket=(%.4f,%.4f) rim=%.4f"
            % (tx, ty, zt, z_grasp, bx, by, rim))

    held = False
    for attempt in (0, 1):
        api.move([tx, ty, APPROACH_Z], rotation=R, seconds=2.0)
        api.move([tx, ty, HOVER_Z], rotation=R, seconds=1.0)
        api.log("a%d hover eef=%s" % (attempt, np.round(api.eef(), 4).tolist()))
        e = walk_down(api, R, tx, ty, z_grasp)
        api.grip(0.01)
        gs = api.gripper()
        api.log("a%d closed gap=%.4f effort=%.2f at z=%.4f"
                % (attempt, gs["width_m"], gs["effort"], e[2]))
        api.move([tx, ty, CARRY_Z], rotation=R, seconds=1.5)
        gs = api.gripper()
        held = (HELD_GAP_MIN < gs["width_m"] < HELD_GAP_MAX
                and gs["effort"] > 1.0)
        api.log("a%d lifted gap=%.4f effort=%.2f held=%s eef=%s"
                % (attempt, gs["width_m"], gs["effort"], held,
                   np.round(api.eef(), 4).tolist()))
        if held:
            break
        api.grip(0.08)
        p2 = perceive(api)
        api.log("a%d re-perceive %s"
                % (attempt, {k: np.round(v, 4).tolist() for k, v in p2.items()}))
        if "target" in p2:
            tx, ty, zt = p2["target"][0], p2["target"][1], p2["target"][2]
            z_grasp = zt - GRASP_BELOW_TOP

    api.move([bx, by, CARRY_Z], rotation=R, seconds=2.0)
    api.move([bx, by, rim + RELEASE_ABOVE_RIM], rotation=R, seconds=1.0)
    api.log("over basket eef=%s gap=%.4f" % (np.round(api.eef(), 4).tolist(),
                                             api.gripper()["width_m"]))
    api.grip(0.08)
    api.settle(0.3)
    api.move([bx, by, CARRY_Z], rotation=R, seconds=1.0)
    api.log("done eef=%s" % np.round(api.eef(), 4).tolist())
    return "held=%s" % held
