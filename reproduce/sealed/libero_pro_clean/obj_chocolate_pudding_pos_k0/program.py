"""v4 - perceive the target box and the basket from cam_high, top-down straddle
grasp across the box's short axis, carry, release over the basket opening."""
import numpy as np

PROVENANCE = {
    "R_DOWN": {"source": "debug-seed measurement: api.tool_rotation() at reset reads "
                         "[[.998,.0005,-.057],[.0005,-1,0],[-.057,0,-.998]]; R_DOWN is "
                         "its exact straight-down form", "allowed": True},
    "WS_X/WS_Y": {"source": "debug seeds 51-65 cam_high cloud: every table prop lies in "
                            "x[-0.19,0.18], y[-0.27,0.36]; bounds widened around that",
                  "allowed": True},
    "Z_TABLE_EPS": {"source": "debug seeds 51-65: the table surface deprojects to "
                              "z=0.001+-0.003, so 0.012 clears it", "allowed": True},
    "Z_ARM_CUT": {"source": "debug seeds 51-65: the robot arm cluster has ztop=0.297 "
                            "while every prop has ztop<=0.148", "allowed": True},
    "MIN_PX": {"source": "debug seeds 51-65: the smallest real prop cluster is 175 px at "
                         "256x256 (699 px at 512x512); speckle is <40 px", "allowed": True},
    "TOP_BAND": {"source": "debug seed 51: the target's top face is flat within 3 mm "
                           "(z 0.027-0.029), so a 6 mm band isolates it", "allowed": True},
    "RIM_BAND": {"source": "debug seeds 51-65: the basket rim top is z=0.1435+-0.001 and "
                           "its walls fall away below; 15 mm keeps the rim ring only",
                 "allowed": True},
    "TIP_Z0": {"source": "debug-seed measurement: with the gripper open, a commanded "
                         "descent to z=-0.06 on clear table stalls at eef z=0.009 at both "
                         "(-0.05,0.12) [v2 ep51] and (0.20,0.06) [v3 ep51]", "allowed": True},
    "GRASP_TIP_Z": {"source": "debug seed 51: the target box is 0.030 m tall, so tips at "
                              "0.012 bite near mid-height", "allowed": True},
    "GRIP_CLOSE/GRIP_OPEN": {"source": "generic gripper mechanics (api.grip(<0.025) "
                                       "closes) plus the measured open width 0.0778",
                             "allowed": True},
    "CARRY_TIP_Z": {"source": "debug seeds 51-65: the tallest obstacle on the carry path "
                              "is the basket rim at z=0.144; tips at 0.22 clear it",
                    "allowed": True},
    "DROP_TIP_Z": {"source": "debug seeds 51-65: basket rim z=0.1435; tips at 0.180 put "
                             "the held box bottom ~0.02 above the rim", "allowed": True},
    "HOLD_MIN_W": {"source": "debug seed 51 (v2): a closed-on-nothing gripper decays "
                             "below 0.023 while the 0.0475-wide box should hold near "
                             "0.045", "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
WS_X = (-0.30, 0.35)
WS_Y = (-0.45, 0.45)
Z_TABLE_EPS = 0.012
Z_ARM_CUT = 0.20
MIN_PX = 40
TOP_BAND = 0.006
RIM_BAND = 0.015
TIP_Z0 = 0.009
GRASP_TIP_Z = 0.012
GRIP_CLOSE = 0.020
GRIP_OPEN = 0.080
CARRY_TIP_Z = 0.22
DROP_TIP_Z = 0.180
HOLD_MIN_W = 0.030


def _label(mask):
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    cur = 0
    seen = mask.copy()
    for sy, sx in np.argwhere(mask):
        if not seen[sy, sx]:
            continue
        cur += 1
        stack = [(int(sy), int(sx))]
        seen[sy, sx] = False
        while stack:
            y, x = stack.pop()
            lab[y, x] = cur
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < H and 0 <= nx < W and seen[ny, nx]:
                    seen[ny, nx] = False
                    stack.append((ny, nx))
    return lab, cur


def perceive(api):
    f = api.capture("cam_high")
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    z = np.asarray(f.depth, float)
    z = np.where(np.isfinite(z), z, 0.0)
    H, W = z.shape
    vv, uu = np.mgrid[0:H, 0:W]
    P = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1],
                  z, np.ones_like(z)], -1) @ T.T
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = ((z > 0) & (Z > Z_TABLE_EPS) & (Z < Z_ARM_CUT) &
         (X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1]))
    lab, n = _label(m)
    comps = []
    for i in range(1, n + 1):
        k = lab == i
        if int(k.sum()) < MIN_PX:
            continue
        comps.append({"n": int(k.sum()), "ztop": float(Z[k].max()), "k": k})
    if len(comps) < 2:
        return None
    # the basket is by far the largest component (2.6x the next on every debug
    # seed); the target is the lowest-topped of the remaining ones (0.030 vs
    # 0.141+ on every debug seed).
    basket = max(comps, key=lambda c: c["n"])
    tgt = min((c for c in comps if c is not basket), key=lambda c: c["ztop"])
    kt = tgt["k"] & (Z > tgt["ztop"] - TOP_BAND)
    kb = basket["k"] & (Z > basket["ztop"] - RIM_BAND)
    return {
        "t": (float((X[kt].min() + X[kt].max()) / 2),
              float((Y[kt].min() + Y[kt].max()) / 2), float(tgt["ztop"])),
        "tspan": (float(X[kt].max() - X[kt].min()), float(Y[kt].max() - Y[kt].min())),
        "b": (float((X[kb].min() + X[kb].max()) / 2),
              float((Y[kb].min() + Y[kb].max()) / 2), float(basket["ztop"])),
        "bspan": (float(X[kb].max() - X[kb].min()), float(Y[kb].max() - Y[kb].min())),
        "comps": [(c["n"], round(c["ztop"], 3)) for c in comps],
    }


def goto(api, x, y, z, seconds=2.0, tag=""):
    r = api.move([float(x), float(y), float(z)], rotation=R_DOWN, seconds=float(seconds))
    e = api.eef()
    api.log("MOVE%s cmd=(%.3f,%.3f,%.3f) got=(%.3f,%.3f,%.3f) res=%.4f"
            % (tag, x, y, z, e[0], e[1], e[2], r))
    return e


def run(api):
    s = perceive(api)
    api.log("SCENE %s" % s)
    if s is None:
        return "no scene"
    tx, ty, _ = s["t"]
    bx, by, _ = s["b"]

    api.grip(GRIP_OPEN)
    goto(api, tx, ty, TIP_Z0 + 0.11, seconds=2.0, tag=" hover")
    goto(api, tx, ty, TIP_Z0 + GRASP_TIP_Z, seconds=2.5, tag=" down")
    api.grip(GRIP_CLOSE)
    api.settle(0.3)
    g = api.gripper()
    api.log("CLOSED %s" % g)

    goto(api, tx, ty, TIP_Z0 + CARRY_TIP_Z, seconds=2.5, tag=" lift")
    api.log("LIFTED %s" % api.gripper())
    goto(api, bx, by, TIP_Z0 + CARRY_TIP_Z, seconds=2.5, tag=" over")
    goto(api, bx, by, TIP_Z0 + DROP_TIP_Z, seconds=1.5, tag=" drop")
    api.log("PREDROP %s" % api.gripper())
    api.grip(GRIP_OPEN)
    api.settle(0.5)
    goto(api, bx, by, TIP_Z0 + CARRY_TIP_Z, seconds=1.5, tag=" retreat")
    s2 = perceive(api)
    api.log("POST %s" % (s2 if s2 else None))
    return "v4"
