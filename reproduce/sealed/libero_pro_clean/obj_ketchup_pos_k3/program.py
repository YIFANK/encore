"""c2clean / obj_ketchup_pos_k3 -- v2.

Task: "Pick up the ketchup and place it in the basket."

Mechanism, end to end:
  1. One cam_high RGB-D frame -> base-frame point cloud -> table plane at z~0.
  2. Height-gated (z > Z_OBJ) mask, 4-connected labelling -> scene components.
  3. Identity: among prop-sized components, rank by (body warmth - 3 * cap
     greyness). The pack's target is a bottle with a GREY cap over a RED-ORANGE
     body; every distractor either has a coloured cap or a cold body.
  4. Grasp the cap: xy from the top-band bbox midpoint, z = top - CAP_GRASP_DROP
     (the pack's three demos all close CAP_GRASP_DROP below the asset's top).
  5. Lift, traverse at CARRY_Z, descend over the basket rim centre, open.

v2 adds sensor-verified retries: a move that ends far from its target is
re-issued (the controller stops at its own 12 mm tolerance and a blocked
descent stops short silently), and a close that does not report a held cap
re-perceives and picks again. No runtime success signal is consulted -- the
only evidence used is the finger gap, the reported residual, and re-perception.

Everything is re-perceived per episode; no scene coordinate is hardcoded.
"""
import numpy as np

PROVENANCE = {
    "Z_OBJ": {
        "source": "debug seeds 51-65 cam_high depth: table plane sits at "
                  "z=0.0005 +/- 0.0002 in base frame; 0.02 clears it",
        "allowed": True},
    "WS_X": {"source": "debug-seed cam_high cloud: all props/basket lie in "
                       "x in (-0.35,0.35) in base frame", "allowed": True},
    "WS_Y": {"source": "debug-seed cam_high cloud: all props/basket lie in "
                       "y in (-0.40,0.40) in base frame", "allowed": True},
    "ARM_TOP_Z": {"source": "debug seeds: the robot component tops out at "
                            "z=0.482, every prop at z<=0.147", "allowed": True},
    "MIN_PIX": {"source": "debug seeds: smallest prop component ~1200 px at "
                          "512x512; noise specks are <100", "allowed": True},
    "MAX_PIX": {"source": "debug seeds: props 1200-2100 px, basket ~11000 px "
                          "at 512x512", "allowed": True},
    "MAX_CAP_W": {"source": "debug seeds: prop top bands span 0.023-0.069 m in "
                            "y, the basket rim spans 0.170 m", "allowed": True},
    "CAP_BAND": {"source": "debug-seed z-profile of the target: its grey cap "
                           "occupies the top ~0.042 m", "allowed": True},
    "BODY_BAND": {"source": "debug-seed z-profile: the coloured body sits "
                            "0.035-0.060 m below the target's top", "allowed": True},
    "CAP_GRASP_DROP": {
        "source": "pack.json ee_path6: demo1 closes at z=0.113, demo2 at "
                  "z=0.112; debug-seed target top z=0.147 -> 0.035 below top",
        "allowed": True},
    "HOVER_Z": {"source": "pack.json ee_path6 pre-grasp altitude (0.16-0.25) "
                          "and debug-seed tallest prop top 0.147", "allowed": True},
    "CARRY_Z": {"source": "pack.json ee_path6 demo1/demo2 traverse at "
                          "z=0.31-0.38; debug-seed basket rim top 0.142",
                "allowed": True},
    "DROP_Z": {"source": "pack.json ee_path6 release heights 0.176/0.176/0.212",
               "allowed": True},
    "OPEN_W": {"source": "debug-seed api.gripper() width at episode start "
                         "(0.0778 m, fully open)", "allowed": True},
    "HELD_GAP": {"source": "debug-seed target cap width 0.030 m; a closed-on-air "
                           "gap is ~0", "allowed": True},
    "MOVE_TOL": {"source": "debug-seed v1 residuals: every converged move "
                           "reported 0.006-0.012; the controller's own stop "
                           "tolerance", "allowed": True},
    "MAX_PICKS": {"source": "own budget choice; v1 used 142 of the episode's "
                            "sim steps on debug seeds", "allowed": True},
}

Z_OBJ = 0.02
WS_X = (-0.35, 0.35)
WS_Y = (-0.40, 0.40)
ARM_TOP_Z = 0.30
MIN_PIX = 400
MAX_PIX = 5000
MAX_CAP_W = 0.09
CAP_BAND = 0.020
BODY_BAND = (0.060, 0.035)
CAP_GRASP_DROP = 0.035
HOVER_Z = 0.25
CARRY_Z = 0.32
DROP_Z = 0.21
OPEN_W = 0.08
HELD_GAP = 0.010
MOVE_TOL = 0.016
MAX_PICKS = 3


# --------------------------------------------------------------------------
def cloud(frame):
    """Full-resolution base-frame xyz for every pixel."""
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    p = np.stack([x, y, d, np.ones_like(d)], -1)
    return (p @ T.T)[..., :3]


def label4(mask):
    """4-connected component labels (pure numpy/python)."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    for p in map(tuple, np.argwhere(mask)):
        if lab[p]:
            continue
        cur += 1
        lab[p] = cur
        stack = [p]
        while stack:
            v, u = stack.pop()
            for q in ((v + 1, u), (v - 1, u), (v, u + 1), (v, u - 1)):
                if 0 <= q[0] < h and 0 <= q[1] < w and mask[q] and not lab[q]:
                    lab[q] = cur
                    stack.append(q)
    return lab, cur


def describe(rgb, P, sel):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    top = float(np.percentile(Z[sel], 99))
    capb = sel & (Z > top - CAP_BAND)
    topb = sel & (Z > top - 0.012)
    if topb.sum() < 4:
        topb = capb
    body = sel & (Z > top - BODY_BAND[0]) & (Z < top - BODY_BAND[1])
    c = rgb[capb].mean(0)
    b = rgb[body].mean(0) if body.sum() > 4 else np.array([0.0, 0.0, 0.0])
    return dict(
        n=int(sel.sum()), top=top,
        x=float((X[topb].min() + X[topb].max()) / 2),
        y=float((Y[topb].min() + Y[topb].max()) / 2),
        capw=float(Y[topb].max() - Y[topb].min()),
        grey=float(c.max() - c.min()), warm=float(b[0] - b[2]),
        cap=[round(float(v), 1) for v in c], bod=[round(float(v), 1) for v in b])


def perceive(api):
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb, float)
    P = cloud(f)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    ws = ((X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1])
          & (Z > -0.1) & (Z < 0.8))
    m = ws & (Z > Z_OBJ)
    # label at half resolution for speed, then expand membership back
    lab_h, n = label4(m[::2, ::2])
    lab = np.repeat(np.repeat(lab_h, 2, 0), 2, 1)[:m.shape[0], :m.shape[1]]
    comps = []
    for i in range(1, n + 1):
        sel = m & (lab == i)
        if sel.sum() < 150:
            continue
        d = describe(rgb, P, sel)
        if d["top"] > ARM_TOP_Z:          # the robot itself
            continue
        comps.append(d)
    comps.sort(key=lambda d: -d["n"])
    for d in comps:
        api.log("COMP n=%d top=%.3f xy=(%.3f,%.3f) capw=%.3f grey=%.1f warm=%.1f "
                "cap=%s bod=%s" % (d["n"], d["top"], d["x"], d["y"], d["capw"],
                                   d["grey"], d["warm"], d["cap"], d["bod"]))
    props = [d for d in comps
             if MIN_PIX <= d["n"] <= MAX_PIX and d["capw"] <= MAX_CAP_W
             and d["top"] > 0.06]
    baskets = [d for d in comps if d["capw"] > MAX_CAP_W and d["n"] > MAX_PIX]
    if not props:
        props = comps
    target = max(props, key=lambda d: d["warm"] - 3.0 * d["grey"])
    basket = max(baskets, key=lambda d: d["n"]) if baskets else None
    return target, basket


# --------------------------------------------------------------------------
def goto(api, xyz, tag, seconds=1.2, tries=2):
    """Move, and re-issue while the controller reports it stopped short."""
    r = api.move(xyz, seconds=seconds)
    for _ in range(tries - 1):
        if r <= MOVE_TOL:
            break
        r = api.move(xyz, seconds=seconds)
    e = api.eef()
    api.log("%s residual=%.4f eef=(%.4f,%.4f,%.4f)" % (tag, r, e[0], e[1], e[2]))
    return r


def holding(api):
    g = api.gripper()
    return g["effort"] > 1.0 and g["width_m"] > HELD_GAP, g


def attempt_pick(api, target):
    """Open, descend onto the cap, close. Returns (held, gripper)."""
    tx, ty = target["x"], target["y"]
    gz = target["top"] - CAP_GRASP_DROP
    api.grip(OPEN_W)
    goto(api, [tx, ty, HOVER_Z], "hover", seconds=1.5)
    goto(api, [tx, ty, gz], "descend", seconds=1.2)
    api.grip(0.0)
    api.settle(0.3)
    held, g = holding(api)
    api.log("after close held=%s gripper=%s" % (held, g))
    return held, g


def run(api):
    home = [float(v) for v in api.eef()]   # the pose perception was validated at
    target, basket = perceive(api)
    if basket is None:
        return "no basket found"
    api.log("TARGET xy=(%.3f,%.3f) top=%.3f score=%.1f" %
            (target["x"], target["y"], target["top"],
             target["warm"] - 3.0 * target["grey"]))
    api.log("BASKET xy=(%.3f,%.3f) top=%.3f" %
            (basket["x"], basket["y"], basket["top"]))

    held, g = False, {"width_m": 0.0}
    for k in range(MAX_PICKS):
        if k:                      # the last try moved the prop: look again
            api.grip(OPEN_W)
            goto(api, [target["x"], target["y"], CARRY_Z], "clear", seconds=1.0)
            goto(api, home, "park", seconds=1.2)   # unocclude the scene
            target, b2 = perceive(api)
            basket = b2 or basket
            api.log("RETRY %d target xy=(%.3f,%.3f) top=%.3f"
                    % (k, target["x"], target["y"], target["top"]))
        held, g = attempt_pick(api, target)
        if not held:
            continue
        goto(api, [target["x"], target["y"], CARRY_Z], "lift", seconds=1.2)
        held, g = holding(api)
        api.log("after lift held=%s gripper=%s" % (held, g))
        if held:
            break

    if not held:
        api.log("GIVE UP: no held cap after %d picks" % MAX_PICKS)
        return "not held"

    bx, by = basket["x"], basket["y"]
    goto(api, [bx, by, CARRY_Z], "traverse", seconds=1.8)
    h2, _ = holding(api)
    if not h2:
        api.log("dropped in transit")
    goto(api, [bx, by, DROP_Z], "lower", seconds=1.0, tries=1)
    api.grip(OPEN_W)
    api.settle(0.8)
    api.move([bx, by, CARRY_Z], seconds=1.0)
    api.settle(0.5)
    return "held=%.4f" % g["width_m"]
