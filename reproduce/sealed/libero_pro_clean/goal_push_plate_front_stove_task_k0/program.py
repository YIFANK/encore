"""v5: v4 (which scored 4/4 on the probe seeds, success firing during the +y push)
plus three robustness fixes, all of which only matter on episodes that do NOT
succeed on the first push:
  * perception rejects points within EEF_MASK_R of the gripper xy (the closed
    jaws are dark and blue-dominant, so they join the cream-cheese mask; v4's
    PERC2 read the box as 8 cm deep in y for exactly that reason),
  * the arm parks out of the way before every re-look,
  * a corrective +y push if the box is still short of the stove's y centre.

Mechanism: the cream cheese is a 0.078 x 0.040 x 0.020 m slab on the table
(top z 0.920, table 0.900). A closed gripper with the fingertips at z~0.905
(EEF z 0.915, tips 0.010 lower) slides it. The goal region lies in front of the
stove, whose slab footprint is measured each episode from the same frame.
"""
import numpy as np

PROVENANCE = {
    "Z_CONTACT": {"source": "debug-seed v2 descent probe: closed gripper floors at EEF z=0.9102 on the table", "allowed": True},
    "TIP": {"source": "v2 contact floor 0.9102 vs table plane z=0.900 in the cam_high cloud", "allowed": True},
    "Z_PUSH": {"source": "v2 floor 0.9102 + v3 receipt (EEF z 0.943 rode over the 0.920 box top); v4 receipt 4/4 at 0.915", "allowed": True},
    "Z_HI": {"source": "v2/v3 debug seeds: 1.02 commanded clears the 0.920 box top and the 0.95 stove slab", "allowed": True},
    "BOX_BAND": {"source": "debug-seed cam_high: cream cheese top z=0.920, table 0.900", "allowed": True},
    "BOX_COLOUR": {"source": "debug-seed cam_high: box rgb ~(65,71,88), plate (146,135,133), table ~(150,120,90)", "allowed": True},
    "STOVE_BAND": {"source": "debug-seed cam_high: stove slab z 0.913-0.95 over x[-0.45,-0.16], y[0.11,0.30]", "allowed": True},
    "FRONT_OFFSET": {"source": "intent 'front of the stove' + measured stove front edge (max x) each episode", "allowed": True},
    "FINGER_HALF": {"source": "generic gripper mechanics: closed jaw half-thickness ~0.008 m", "allowed": True},
    "EEF_MASK_R": {"source": "v4 PERC2 receipt: gripper pixels contaminate the colour mask within ~0.06 m of the EEF", "allowed": True},
    "PARK": {"source": "debug-seed cam_high workspace: table corner clear of every prop", "allowed": True},
}

Z_HI = 1.02
Z_PUSH = 0.915
FINGER_HALF = 0.008
APPROACH_GAP = 0.070
FRONT_OFFSET = 0.060
NEAR_R = 0.12
EEF_MASK_R = 0.075
PARK = [0.20, -0.28, 1.06]


def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    K = np.asarray(f.intrinsics)
    T = np.asarray(f.t_base_cam)
    d = np.asarray(f.depth, dtype=np.float32)
    rgb = np.asarray(f.rgb).astype(np.float32)
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]
    fx = K[0, 0]
    x = (uu - K[0, 2]) / fx * d
    y = (vv - K[1, 2]) / fx * d
    P = np.stack([x, y, d], -1) @ T[:3, :3].T + T[:3, 3]
    return P, rgb


def find_box(api, P, rgb, near=None):
    m = (P[..., 0] > -0.45) & (P[..., 0] < 0.30) & (np.abs(P[..., 1]) < 0.50) \
        & (P[..., 2] > 0.911) & (P[..., 2] < 0.945) \
        & (rgb[..., 2] > rgb[..., 0] + 8) & (rgb.mean(-1) < 120)
    e = api.eef()
    m = m & ((P[..., 0] - e[0]) ** 2 + (P[..., 1] - e[1]) ** 2 > EEF_MASK_R ** 2)
    if near is not None:
        m = m & ((P[..., 0] - near[0]) ** 2 + (P[..., 1] - near[1]) ** 2 < NEAR_R ** 2)
    if int(m.sum()) < 60:
        return None
    p = P[m]
    x0, x1 = float(p[:, 0].min()), float(p[:, 0].max())
    y0, y1 = float(p[:, 1].min()), float(p[:, 1].max())
    return dict(n=int(m.sum()), x0=x0, x1=x1, y0=y0, y1=y1,
                cx=0.5 * (x0 + x1), cy=0.5 * (y0 + y1))


def find_stove(P):
    m = (P[..., 0] > -0.46) & (P[..., 0] < -0.14) & (P[..., 1] > 0.05) \
        & (P[..., 1] < 0.45) & (P[..., 2] > 0.913) & (P[..., 2] < 0.945)
    p = P[m]
    return dict(n=int(m.sum()), x1=float(p[:, 0].max()),
                y0=float(p[:, 1].min()), y1=float(p[:, 1].max()))


def push_y(api, b, y_goal, tag):
    """Slide the box in +y until its centre reaches y_goal."""
    hy = 0.5 * (b["y1"] - b["y0"])
    sy = b["y0"] - APPROACH_GAP
    api.move([b["cx"], sy, Z_HI], seconds=1.2)
    api.move([b["cx"], sy, Z_PUSH], seconds=1.4)
    ey = y_goal - hy - FINGER_HALF
    api.move([b["cx"], ey, Z_PUSH], seconds=1.8)
    e = api.eef()
    api.log("%s y_goal=%.3f ey=%.3f eef=(%.3f,%.3f,%.3f)" % (tag, y_goal, ey, e[0], e[1], e[2]))


def push_x(api, b, x_goal, tag):
    """Slide the box in -x until its centre reaches x_goal."""
    hx = 0.5 * (b["x1"] - b["x0"])
    sx = b["x1"] + APPROACH_GAP
    api.move([sx, b["cy"], Z_HI], seconds=1.2)
    api.move([sx, b["cy"], Z_PUSH], seconds=1.4)
    ex = x_goal + hx + FINGER_HALF
    api.move([ex, b["cy"], Z_PUSH], seconds=1.8)
    e = api.eef()
    api.log("%s x_goal=%.3f ex=%.3f eef=(%.3f,%.3f,%.3f)" % (tag, x_goal, ex, e[0], e[1], e[2]))


def relook(api, near, tag):
    api.move(PARK, seconds=1.5)
    P, rgb = cloud(api)
    b = find_box(api, P, rgb, near=near)
    api.log("%s box=%s" % (tag, b))
    return b


def run(api):
    P, rgb = cloud(api)
    box = find_box(api, P, rgb)
    st = find_stove(P)
    api.log("PERC box=%s stove=%s" % (box, st))
    if box is None:
        api.log("ABORT no box")
        return
    XT = st["x1"] + FRONT_OFFSET
    YT = 0.5 * (st["y0"] + st["y1"])
    api.log("TARGET (%.3f,%.3f)" % (XT, YT))

    api.grip(0.0)
    api.settle(0.2)

    push_y(api, box, YT, "PUSH1")

    b2 = relook(api, (box["cx"], YT), "PERC2")
    if b2 is None:
        b2 = dict(x0=box["x0"], x1=box["x1"], y0=YT - 0.020, y1=YT + 0.020,
                  cx=box["cx"], cy=YT)
    if b2["cx"] > XT + 0.02:
        push_x(api, b2, XT, "PUSH2")

    b3 = relook(api, (XT, YT), "PERC3")
    if b3 is not None and b3["cy"] < YT - 0.03:
        push_y(api, b3, YT + 0.01, "PUSH3")
        b3 = relook(api, (XT, YT), "PERC4")
    api.log("FINAL box=%s target=(%.3f,%.3f)" % (b3, XT, YT))
