"""v15: drop to bar height in the clear, then creep in along -y in small steps.

Every earlier attempt drove down the 45 deg approach diagonal in one move and
jammed 0.027 m short of the bar (v11/v12/v13/v14 alike, and v14 showed closing
the gripper first does not help). v10 proved the arm CAN hold eef z = 0.941 at
45 deg once it is at y = -0.145, so this separates the two motions: get low
where the arm is known to fit, then translate in pure -y at that height in
steps small enough that the OSC keeps pushing instead of wedging.

At the end pose the bar sits 0.006 m off the tool axis and 0.002 m inside the
fingertip plane, so a close should take it.
"""
import numpy as np

PROVENANCE = {
    "CAB_Z_LO": {"source": "debug seeds 51/55/60 cam_high depth: table plane z=0.900, "
                           "cabinet body starts z=0.920", "allowed": True},
    "CAB_Z_HI": {"source": "debug seeds 51/55/60: cabinet top slab at z=1.128", "allowed": True},
    "TOP_SLAB_Z": {"source": "debug seeds 51/55/60: cabinet top slab lower edge, above the "
                             "topmost bar at z=1.090", "allowed": True},
    "PROTRUDE_M": {"source": "debug seeds 51/55/60: bars stand 0.0315 m proud of the drawer "
                             "face; a 0.022 m band keeps the bars and drops the face",
                   "allowed": True},
    "Z_GAP": {"source": "debug seeds 51/55/60: bars at z 0.948/1.017/1.090, spacing "
                        "0.069-0.073", "allowed": True},
    "TILT_DEG": {"source": "debug seeds 51/55 v10 probe: lowest reachable eef z by wrist tilt "
                           "is 1.137/1.026/0.941/0.965/1.000 at 0/30/45/60/90 deg, so 45 deg "
                           "is the only tilt that clears the 0.948 bar", "allowed": True},
    "STANDOFF_M": {"source": "debug seeds 51/55 v10: eef z 0.9411 was reached at 45 deg with "
                             "y = bar_front + 0.055", "allowed": True},
    "GRASP_Z_DROP": {"source": "debug seeds 51/55 v10: 0.006 m below the bar centre keeps the "
                               "eef above the measured 0.9411 floor and puts the bar just "
                               "inside the fingertip plane", "allowed": True},
    "BAR_HALF_D": {"source": "debug seeds 51/55 v7/v8: bar front y_f, bar back y_f-0.018",
                   "allowed": True},
    "CREEP_N": {"source": "debug seed 51 v13: one long -y command wedges (48 steps moved "
                          "0.0002 m); short repeated commands re-issue the push",
                "allowed": True},
    "PULL_M": {"source": "pack keyframes demo0/demo2: the drawer face moves from y=-0.200 to "
                         "y=-0.054 when deprojected with the measured cam_high intrinsics "
                         "and extrinsics, i.e. a 0.146 m pull", "allowed": True},
    "HOLD_MIN_W": {"source": "debug seeds 51/55 v11/v12/v14: an empty close reads width 0.0018",
                   "allowed": True},
}

CAB_Z_LO, CAB_Z_HI = 0.915, 1.140
TOP_SLAB_Z = 1.112
PROTRUDE_M = 0.022
Z_GAP = 0.018
TILT_DEG = 45.0
STANDOFF_M = 0.055
GRASP_Z_DROP = 0.006
BAR_HALF_D = 0.009
CREEP_N = 5
PULL_M = 0.155
HOLD_MIN_W = 0.006

_T = np.radians(TILT_DEG)
_C, _S = np.cos(_T), np.sin(_T)
RT = np.array([[1.0, 0.0, 0.0], [0.0, -_C, -_S], [0.0, _S, -_C]])


def cloud(f):
    d = np.nan_to_num(f.depth.astype(float), nan=0.0, posinf=0.0)
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]
    K = f.intrinsics
    P = np.stack([(uu - K[0, 2]) * d / K[0, 0],
                  (vv - K[1, 2]) * d / K[1, 1], d, np.ones_like(d)], -1) @ f.t_base_cam.T
    return P[..., 0], P[..., 1], P[..., 2], d > 0


def perceive(api, f):
    X, Y, Z, ok = cloud(f)
    box = (ok & (Z > CAB_Z_LO) & (Z < CAB_Z_HI) & (Y > -0.50) & (Y < -0.10)
           & (X > -0.30) & (X < 0.30))
    if int(box.sum()) < 200:
        return []
    ye = float(np.percentile(Y[box], 99.5))
    prot = box & (Y > ye - PROTRUDE_M) & (Z < TOP_SLAB_Z)
    api.log(f"face_edge={ye:.4f} box_n={int(box.sum())} prot_n={int(prot.sum())}")
    zs, xs, ys = Z[prot], X[prot], Y[prot]
    o = np.argsort(zs)
    zs, xs, ys = zs[o], xs[o], ys[o]
    bars = []
    for g in np.split(np.arange(len(zs)), np.where(np.diff(zs) > Z_GAP)[0] + 1):
        if len(g) >= 20:
            bars.append(dict(n=len(g), zc=float((zs[g].min() + zs[g].max()) / 2.0),
                             x=float(np.median(xs[g])),
                             yf=float(np.percentile(ys[g], 95))))
    return bars


def run(api):
    f = api.capture("cam_high")
    bars = perceive(api, f)
    for b in bars:
        api.log("BAR " + " ".join(f"{k}={v:.4f}" if isinstance(v, float) else f"{k}={v}"
                                  for k, v in b.items()))
    if not bars:
        return "no bars"
    bot = bars[0]
    gx, gz, yf = bot["x"], bot["zc"], bot["yf"]
    zh = gz - GRASP_Z_DROP
    y_stand = yf + STANDOFF_M
    y_grasp = yf - BAR_HALF_D
    api.log(f"PLAN x={gx:.4f} bar_z={gz:.4f} bar_front={yf:.4f} hold_z={zh:.4f} "
            f"y_stand={y_stand:.4f} y_grasp={y_grasp:.4f} pull_to={yf + PULL_M:.4f}")

    api.grip(0.08)
    api.move([gx, y_stand, zh + 0.115], rotation=RT, seconds=1.2)
    r = api.move([gx, y_stand, zh], rotation=RT, seconds=1.4)
    api.log(f"LOW eef {np.round(api.eef(),4).tolist()} res {r:.4f}")

    for i in range(1, CREEP_N + 1):
        ty = y_stand + (y_grasp - y_stand) * i / CREEP_N
        r = api.move([gx, ty, zh], rotation=RT, seconds=1.0)
        e = api.eef()
        api.log(f"CREEP {i} tgt_y={ty:.4f} eef {np.round(e,4).tolist()} res {r:.4f} "
                f"short={e[1]-ty:+.4f}")

    api.grip(0.0)
    g = api.gripper()
    w = float(g["width_m"])
    api.log(f"CLOSED grip={g} width={w:.4f} held={w > HOLD_MIN_W} "
            f"eef {np.round(api.eef(),4).tolist()}")

    e0 = api.eef()
    y0, zhold = float(e0[1]), float(e0[2])
    pull_to = yf + PULL_M
    n = 4
    for i in range(1, n + 1):
        ty = y0 + (pull_to - y0) * i / n
        r = api.move([gx, ty, zhold], rotation=RT, seconds=1.0)
        e = api.eef()
        api.log(f"PULL {i} tgt_y={ty:.4f} eef {np.round(e,4).tolist()} res {r:.4f} "
                f"grip={api.gripper()} travel={e[1]-y0:+.4f}")
    return "v15"
