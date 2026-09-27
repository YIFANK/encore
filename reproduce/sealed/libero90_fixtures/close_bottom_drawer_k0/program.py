"""l90abl close_bottom_drawer_k0 -- v9: corridor limited to the actual approach lane, left-preferred column."""
import numpy as np

PROVENANCE = {
    "CAB_TALL_DZ": {
        "source": "debug seeds 51-65 cam_high depth: the cabinet's top face is the dominant horizontal "
                  "plane more than 0.12 m above the table on the far (y>0.15) side of the scene",
        "allowed": True},
    "WALL_BAND": {
        "source": "debug seeds 51-65 depth: the open drawer's side-wall tops sit 0.081 m above the table "
                  "while the bowl rim and drawer handle sit at 0.051 m, so a table+0.06..+0.11 band "
                  "isolates the drawer",
        "allowed": True},
    "PUSH_DZ": {
        "source": "debug seeds 51-65: bowl rim = table+0.050, drawer side-wall top = table+0.081, drawer "
                  "front panel top edge ~ table+0.10; table+0.072 clears the bowl and still lands on the "
                  "panel",
        "allowed": True},
    "EEF_TIP_DZ": {
        "source": "v3 debug seed 51 contact probe: closed-gripper descent on bare table stalled with "
                  "eef z only 0.008 above the measured table plane",
        "allowed": True},
    "OBST_DZ": {
        "source": "derived from PUSH_DZ: only geometry reaching above table+0.055 can foul a push at "
                  "table+0.072",
        "allowed": True},
    "ARM_MASK_R": {
        "source": "debug seeds: the robot's own arm at reset is a tall structure at x~-0.21, y~0; v4 "
                  "showed it impersonating the cabinet (y_front 0.038 in all 8 seeds) unless masked",
        "allowed": True},
    "CORRIDOR_DY": {
        "source": "v8 debug run: a 0.24 m corridor swept in the wooden rack (y < -0.16) and zeroed the "
                  "clearance profile across the whole drawer; the gripper only ever occupies the lane "
                  "from drawer_front-0.13 to drawer_front",
        "allowed": True},
    "CLR_MIN": {
        "source": "v6 debug run: a centre-weighted column with clearance 0.000 drove the gripper into the "
                  "bowl and stalled at y~0.085 (0/8); clearance must be a hard constraint. v7 raised the "
                  "push to table+0.072 to fly over the bowl and the drawer stopped 0.07 m short (0/8), so "
                  "the push height stays at v5's table+0.045 and the column must dodge the bowl instead.",
        "allowed": True},
    "PREFER_LEFT": {
        "source": "v5 debug run: pushes at x~-0.09 stalled at eef y = 0.116-0.129 and scored 6/6; pushes "
                  "at x~+0.10 stalled at y = 0.110-0.115 and scored 0/2 -- the -x column buys reach",
        "allowed": True},
    "EDGE_KEEPOUT": {"source": "debug seeds: stay inside the measured drawer wall span", "allowed": True},
    "APPROACH_DY": {"source": "debug seeds 51-65: 0.13 m in front of the drawer front is clear table", "allowed": True},
    "OVERTRAVEL": {
        "source": "v4/v5: a converging push leaves the drawer short; command far past the closed pose so "
                  "the move stays unconverged and keeps pressing",
        "allowed": True},
    "STEP_BUDGET": {"source": "v3 results.jsonl sim_steps=1000; move(seconds=s) costs ~100*s steps", "allowed": True},
}

WALL_LO, WALL_HI = 0.060, 0.110
PUSH_DZ = 0.045
OBST_DZ = 0.030
ARM_MASK_R = 0.05
CLR_MIN = 0.032
EDGE_KEEPOUT = 0.015
APPROACH_DY = 0.13
CORRIDOR_DY = 0.15
OVERTRAVEL = 0.32
Y_CAP = 0.34

FB = dict(table=0.902, x_lo=-0.100, x_hi=0.120, y_drawer=0.070, push_x=-0.075)


def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    d = np.asarray(f.depth, dtype=float)
    K = np.asarray(f.intrinsics)
    T = np.asarray(f.t_base_cam)
    H, W = d.shape
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(uu - K[0, 2]) / K[0, 0] * d, (vv - K[1, 2]) / K[1, 1] * d, d], -1)
    P = pc @ T[:3, :3].T + T[:3, 3]
    return P[..., 0], P[..., 1], P[..., 2]


def perceive(api):
    g = dict(FB)
    X, Y, Z = cloud(api)
    ex, ey = np.asarray(api.eef(), dtype=float)[:2]
    near = (X > -0.35) & (X < 0.35) & (Y > -0.45) & (Y < 0.45) & (Z > 0.6) & (Z < 1.5)
    h, e = np.histogram(Z[near], bins=200)
    table = float(e[int(h.argmax())] + 0.5 * (e[1] - e[0]))
    g["table"] = table
    noarm = near & (np.hypot(X - ex, Y - ey) > ARM_MASK_R) & (X > ex + 0.05)

    tall = noarm & (Y > 0.15) & (Z > table + 0.12)
    if tall.sum() > 500:
        h2, e2 = np.histogram(Z[tall], bins=60)
        ctop = float(e2[int(h2.argmax())] + 0.5 * (e2[1] - e2[0]))
        cab = noarm & (np.abs(Z - ctop) < 0.015) & (Y > 0.0)
        if cab.sum() > 500:
            g["cab_x"] = (float(np.percentile(X[cab], 1)), float(np.percentile(X[cab], 99)))
            g["y_cab"] = float(np.percentile(Y[cab], 1))
            api.log("cab top=%.3f x=%.3f..%.3f y_front=%.3f" % (ctop, g["cab_x"][0], g["cab_x"][1], g["y_cab"]))

    xlo, xhi = g.get("cab_x", (-0.15, 0.15))
    ycab = g.get("y_cab", 0.20)
    w = noarm & (Z > table + WALL_LO) & (Z < table + WALL_HI) & (X > xlo + 0.005) & (X < xhi - 0.005) \
        & (Y < ycab) & (Y > -0.15)
    if w.sum() > 300:
        g["x_lo"] = float(np.percentile(X[w], 1))
        g["x_hi"] = float(np.percentile(X[w], 99))
        g["y_drawer"] = float(np.percentile(Y[w], 1))
    api.log("table=%.4f drawer x=%.3f..%.3f y_front=%.3f n=%d" % (table, g["x_lo"], g["x_hi"], g["y_drawer"], int(w.sum())))

    # obstacles: only geometry that reaches the push height can foul the approach
    y0 = g["y_drawer"]
    cor = noarm & (Z > table + OBST_DZ) & (Y < y0 - 0.012) & (Y > y0 - CORRIDOR_DY)
    obs = X[cor]
    cands = np.arange(g["x_lo"] + EDGE_KEEPOUT, g["x_hi"] - EDGE_KEEPOUT + 1e-6, 0.005)
    if len(cands) == 0:
        cands = np.array([0.5 * (g["x_lo"] + g["x_hi"])])
    clr = np.array([np.min(np.abs(obs - c)) for c in cands]) if obs.size else np.full(len(cands), 1.0)
    ok = np.flatnonzero(clr >= CLR_MIN)
    if ok.size:
        k = int(ok[0])            # smallest x: the -x column buys +y reach (v5 receipt)
    else:
        half = int(np.searchsorted(cands, 0.5 * (g["x_lo"] + g["x_hi"])))
        left = clr[:max(half, 1)]
        k = int(np.argmax(left)) if left.max() >= 0.030 else int(np.argmax(clr))
    g["push_x"] = float(cands[k])
    api.log("push_x=%.3f clr=%.3f nfeas=%d obs=%d prof=%s" % (
        g["push_x"], clr[k], int(ok.size), obs.size,
        [(round(float(c), 3), round(float(v), 3)) for c, v in zip(cands[::4], clr[::4])]))
    return g


def run(api):
    try:
        g = perceive(api)
    except Exception as e:
        api.log("perceive failed %s" % e)
        g = dict(FB)
    api.grip(0.0)
    z = g["table"] + PUSH_DZ
    x = g["push_x"]
    ys = g["y_drawer"] - APPROACH_DY
    ye = min(g["y_drawer"] + OVERTRAVEL, Y_CAP)
    api.log("plan x=%.3f y %.3f->%.3f z=%.4f" % (x, ys, ye, z))

    r = api.move([x, ys, z + 0.06], seconds=1.8)
    api.log("over res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    r = api.move([x, ys, z], seconds=0.8)
    api.log("down res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    r = api.move([x, ye, z], seconds=6.0)
    e = np.asarray(api.eef(), dtype=float)
    api.log("push res=%.4f eef=%s" % (r, np.round(e, 4).tolist()))

    try:
        X, Y, Z = cloud(api)
        t = g["table"]
        w = (Z > t + WALL_LO) & (Z < t + WALL_HI) & (X > g["x_lo"] - 0.01) & (X < g["x_hi"] + 0.01) \
            & (Y > -0.15) & (Y < 0.5) & (np.hypot(X - e[0], Y - e[1]) > 0.13)
        if w.sum() > 100:
            api.log("POST drawer y_front=%.3f n=%d (cab_front=%.3f)" % (
                np.percentile(Y[w], 1), w.sum(), g.get("y_cab", -9)))
        else:
            api.log("POST drawer band empty n=%d" % w.sum())
    except Exception as ex:
        api.log("post fail %s" % ex)
