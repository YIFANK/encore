"""c2clean obj_milk_task_k3 (v2 = v1 + occlusion-retry perception) -- "Pick the butter and place it in the basket".

Mechanism (both packs agree): approach over the object, descend, close, lift to
~0.28, traverse to the basket (y ~ +0.26), descend to ~0.19, open.

Identity: the mate pack ("pick up the butter and place it in the basket")
grasps a SHORT box (eef z 0.009-0.011 at close) whose package is warm
orange/red, and leaves a dark-brown box of the same size class untouched --
that dark box also appears in this scene, so it is the anti-target.
"""
import numpy as np

PROVENANCE = {
    "R_DOWN": {"source": "generic controller mechanics: straight-down tool frame; "
                         "matches demo ee_path6 rx~3.14 in both packs",
               "allowed": True},
    "GRASP_Z": {"source": "mate pack keyframes: eef z at gripper_cmd=+1 is "
                          "0.0095/0.0100/0.0111 across its 3 demos",
                "allowed": True},
    "CARRY_Z": {"source": "mate+k3 pack ee_path: transfer altitude 0.26-0.33",
                "allowed": True},
    "RELEASE_Z": {"source": "mate pack ee_path: release at z 0.19-0.23 over the basket",
                  "allowed": True},
    "TABLE_Z": {"source": "debug seeds 51-57 cam_high depth: table plane z ~ 0.000-0.005",
                "allowed": True},
    "WS_BOUNDS": {"source": "debug seeds 51-57: walls deproject to x<-0.4; props lie in "
                            "x[-0.35,0.35] y[-0.45,0.45]",
                  "allowed": True},
    "FLAT_TOP_MAX": {"source": "debug seeds 51-57 cluster tops: flat boxes 0.019-0.020, "
                               "tall props 0.140-0.143",
                     "allowed": True},
    "WARM_MIN": {"source": "debug seeds 51-57 top-band colour: warm box r-b ~ +56, "
                           "cool box r-b ~ -25",
                 "allowed": True},
    "BASKET_MIN_SPAN": {"source": "debug seeds 51-57: basket footprint span 0.15-0.17 m, "
                                  "every prop <= 0.08 m",
                        "allowed": True},
    "RETREAT": {"source": "debug seeds 51-65: arm start pose occludes/fuses props near "
                          "(-0.13, 0.0); this pose is inside the demonstrated carry "
                          "envelope (mate/k3 ee_path reach y=+0.28, z=0.33)",
                "allowed": True},
    "GRID_M": {"source": "debug-seed clustering: 0.012 m grid separates all props",
               "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
GRASP_Z = 0.010
CARRY_Z = 0.285
RELEASE_Z = 0.195
RETREAT = [-0.10, 0.30, 0.33]
WS = dict(xlo=-0.35, xhi=0.35, ylo=-0.45, yhi=0.45, zlo=0.008, zhi=0.30)
FLAT_TOP_MAX = 0.060
WARM_MIN = 20.0
BASKET_MIN_SPAN = 0.12
GRID_M = 0.012


def cloud(frame):
    """Full-frame deprojection -> (H,W,3) base-frame xyz."""
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1) @ T.T
    return P[..., :3]


def clusters(api, frame):
    P = cloud(frame)
    rgb = np.asarray(frame.rgb, float)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = ((X > WS["xlo"]) & (X < WS["xhi"]) & (Y > WS["ylo"]) & (Y < WS["yhi"])
         & (Z > WS["zlo"]) & (Z < WS["zhi"]) & np.isfinite(Z))
    pts = np.stack([X[m], Y[m], Z[m]], -1)
    cols = rgb[m]
    gi = np.round(pts[:, 0] / GRID_M).astype(int)
    gj = np.round(pts[:, 1] / GRID_M).astype(int)
    cells = {}
    for k in range(len(gi)):
        cells.setdefault((gi[k], gj[k]), []).append(k)
    seen, out = set(), []
    for c in cells:
        if c in seen:
            continue
        stack, comp = [c], []
        seen.add(c)
        while stack:
            cc = stack.pop()
            comp += cells[cc]
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    n = (cc[0] + da, cc[1] + db)
                    if n in cells and n not in seen:
                        seen.add(n)
                        stack.append(n)
        if len(comp) < 120:
            continue
        p = pts[comp]
        c_ = cols[comp]
        top = float(p[:, 2].max())
        band = p[:, 2] > top - 0.006
        face = p[band]
        col = c_[band].mean(0)
        out.append(dict(n=len(comp), top=top,
                        cx=float(face[:, 0].mean()), cy=float(face[:, 1].mean()),
                        dx=float(np.ptp(p[:, 0])), dy=float(np.ptp(p[:, 1])),
                        fdx=float(np.ptp(face[:, 0])), fdy=float(np.ptp(face[:, 1])),
                        warm=float(col[0] - col[2]), rgb=[round(float(v)) for v in col]))
    out.sort(key=lambda d: -d["n"])
    for d in out:
        api.log("CLUSTER n=%d top=%.3f c=(%+.3f,%+.3f) d=(%.3f,%.3f) face=(%.3f,%.3f) "
                "warm=%+.0f rgb=%s" % (d["n"], d["top"], d["cx"], d["cy"], d["dx"], d["dy"],
                                       d["fdx"], d["fdy"], d["warm"], d["rgb"]))
    return out


def run(api):
    api.log("INSTRUCTION %r" % api.instruction())
    api.grip(0.08)
    baskets, warm = [], []
    for attempt in range(2):
        if attempt:
            # the parked arm is a tall structure: it can fuse with, or hide, a
            # prop near (-0.13, 0.0). Retreat it and look again.
            api.log("RETRY perception after arm retreat")
            api.move(RETREAT, rotation=R_DOWN, seconds=3.0)
            api.settle(0.4)
        f = api.capture("cam_high")
        cl = clusters(api, f)
        baskets = [d for d in cl if max(d["dx"], d["dy"]) > BASKET_MIN_SPAN and d["top"] < 0.25]
        flats = [d for d in cl if d["top"] < FLAT_TOP_MAX
                 and max(d["dx"], d["dy"]) < BASKET_MIN_SPAN]
        warm = [d for d in flats if d["warm"] > WARM_MIN]
        if baskets and warm:
            break
        api.log("PERCEPTION THIN baskets=%d flats=%d warm=%d" % (len(baskets), len(flats), len(warm)))
    if not baskets or not warm:
        api.log("PERCEPTION FAIL")
        return "perception fail"
    basket = max(baskets, key=lambda d: d["n"])
    tgt = max(warm, key=lambda d: d["warm"])
    api.log("TARGET (%.3f,%.3f) top=%.3f warm=%.0f | BASKET (%.3f,%.3f) top=%.3f"
            % (tgt["cx"], tgt["cy"], tgt["top"], tgt["warm"],
               basket["cx"], basket["cy"], basket["top"]))

    tx, ty = tgt["cx"], tgt["cy"]
    # approach
    api.move([tx, ty, 0.18], rotation=R_DOWN, seconds=3.0)
    r = api.move([tx, ty, 0.10], rotation=R_DOWN, seconds=2.0)
    api.log("HOVER eef=%s res=%.4f" % (np.round(api.eef(), 4).tolist(), r))
    r = api.move([tx, ty, GRASP_Z], rotation=R_DOWN, seconds=2.5)
    api.log("DESCEND eef=%s res=%.4f" % (np.round(api.eef(), 4).tolist(), r))
    api.grip(0.0)
    api.settle(0.6)
    g = api.gripper()
    api.log("CLOSED %s eef=%s" % (g, np.round(api.eef(), 4).tolist()))

    # lift
    api.move([tx, ty, 0.16], rotation=R_DOWN, seconds=2.0)
    api.move([tx, ty, CARRY_Z], rotation=R_DOWN, seconds=2.0)
    g = api.gripper()
    api.log("LIFTED %s eef=%s" % (g, np.round(api.eef(), 4).tolist()))

    # traverse
    bx, by = basket["cx"], basket["cy"]
    api.move([(tx + bx) / 2, (ty + by) / 2, CARRY_Z], rotation=R_DOWN, seconds=2.5)
    api.move([bx, by, CARRY_Z], rotation=R_DOWN, seconds=2.5)
    api.log("OVER_BASKET eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.move([bx, by, RELEASE_Z], rotation=R_DOWN, seconds=2.0)
    api.grip(0.08)
    api.settle(1.0)
    api.log("RELEASED eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.move([bx, by, CARRY_Z], rotation=R_DOWN, seconds=2.0)
    api.settle(0.5)

    f2 = api.capture("cam_high")
    clusters(api, f2)
    return "v1 done"
