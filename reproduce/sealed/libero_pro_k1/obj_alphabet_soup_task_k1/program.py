"""c2k1clean / obj_alphabet_soup_task_k1

Intent: "Pick the cream cheese and place it in the basket".

Mechanism
---------
1. cam_high RGB-D -> base-frame point cloud -> table plane.
2. The OBJECT the intent names is identified by the two cues the mate pack
   supplies: it is the blue-dominant item (the mate demo, whose language is
   "pick up the cream cheese ...", drives the gripper onto the small blue box)
   and it is FLAT (the mate demo closes its fingers at eef z = 0.011, i.e. a
   couple of centimetres above the table, whereas the k1 demo, acting on a can,
   closes at z = 0.062).  So: bluest connected cluster whose top sits within
   3.5 cm of the table.
3. The TARGET is the basket: the largest low structure on the table; both demos
   end by opening the fingers a few centimetres above its rim.
4. Grasp across the footprint's SHORT axis with a straight-down wrist, lift,
   carry above every other object, release over the rim-bbox centre.

Verification is by the program's own sensors only: gripper width + effort after
the close, and eef residuals -- never by any success signal.
"""
import numpy as np

PROVENANCE = {
    "BLUE_MIN": {
        "source": "debug seeds 51-65 cam_high: the cream-cheese box measures "
                  "b-(r+g)/2 = +20.6 while every other table item is <= 0; "
                  "identity of the box as 'cream cheese' comes from the mate "
                  "pack (language 'pick up the cream cheese and place it in "
                  "the basket'; its t=0054 keyframe shows the gripper closing "
                  "on the small blue box)",
        "allowed": True},
    "FLAT_TOP_MAX": {
        "source": "debug seeds 51-65: cream-cheese top is 0.0200 m above the "
                  "table plane; the next-lowest blue fragment (milk carton) "
                  "tops out at 0.047 m -- cut placed between them",
        "allowed": True},
    "GRASP_Z_ABOVE_TABLE": {
        "source": "mate pack ee_path6: fingers close at eef z = 0.0108 with "
                  "the table at z ~ 0.001, i.e. ~0.010 m above the table, "
                  "which is mid-height of the 0.019 m box measured on debug "
                  "seeds 51-65",
        "allowed": True},
    "HOLD_WIDTH_RANGE": {
        "source": "mate pack keyframe t=0120 gripper_state [0.0199,-0.0239] "
                  "-> held width 0.0438 m, consistent with the 0.041 m short "
                  "axis of the box measured on debug seeds 51-65",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug seeds 51-65: tallest table item tops at 0.147 m "
                  "(bottle); carry above it",
        "allowed": True},
    "RELEASE_ABOVE_RIM": {
        "source": "mate pack ee_path6 releases at eef z = 0.1789 with the "
                  "basket rim measured at 0.142 m on debug seeds 51-65 "
                  "(~0.037 m of clearance)",
        "allowed": True},
    "WORKSPACE_BOX": {
        "source": "debug seeds 51-65 cam_high deprojection: every table item "
                  "lies inside x in [-0.35,0.35], y in [-0.40,0.45]",
        "allowed": True},
    "HOME_XYZ": {
        "source": "debug seeds 51-65: api.eef() at episode start is "
                  "[-0.1485, 0.0, 0.2613]",
        "allowed": True},
}

BLUE_MIN = 10.0
FLAT_TOP_MAX = 0.035
GRASP_Z_ABOVE_TABLE = 0.010
HOLD_WIDTH_RANGE = (0.030, 0.060)
CARRY_Z = 0.26
RELEASE_ABOVE_RIM = 0.040
WS = (-0.35, 0.35, -0.40, 0.45)
HOME_XYZ = (-0.1485, 0.0, 0.2613)


# ---------------------------------------------------------------- perception
def _cloud(frame):
    d = np.asarray(frame.depth, dtype=float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    z = np.where(np.isfinite(d) & (d > 0), d, np.nan)
    x = (uu - K[0, 2]) * z / K[0, 0]
    y = (vv - K[1, 2]) * z / K[1, 1]
    P = np.stack([x, y, z, np.ones_like(z)], -1)
    return (P @ T.T)[..., :3]


def _label(mask):
    """4-connected labelling of a boolean image."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    for sy, sx in np.argwhere(mask):
        if lab[sy, sx]:
            continue
        cur += 1
        stack = [(sy, sx)]
        lab[sy, sx] = cur
        while stack:
            y, x = stack.pop()
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not lab[ny, nx]:
                    lab[ny, nx] = cur
                    stack.append((ny, nx))
    return lab, cur


def perceive(api):
    f = api.capture("cam_high")
    B = _cloud(f)
    rgb = np.asarray(f.rgb).astype(float)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    ws = (X > WS[0]) & (X < WS[1]) & (Y > WS[2]) & (Y < WS[3])
    flat = ws & np.isfinite(Z) & (Z < 0.02)
    table = float(np.nanmedian(Z[flat])) if flat.any() else 0.0

    blue = rgb[..., 2] - 0.5 * (rgb[..., 0] + rgb[..., 1])
    bm = (ws & np.isfinite(Z) & (Z > table + 0.008)
          & (Z < table + FLAT_TOP_MAX) & (blue > BLUE_MIN))
    lab, n = _label(bm)
    seeds = []
    for i in range(1, n + 1):
        m = lab == i
        k = int(m.sum())
        if k < 120:
            continue
        seeds.append((k, B[m][:, :2].mean(0)))
    obj = None
    if seeds:
        seeds.sort(key=lambda s: -s[0])
        k, c = seeds[0]
        near = (np.abs(X - c[0]) < 0.07) & (np.abs(Y - c[1]) < 0.07)
        om = ws & near & np.isfinite(Z) & (Z > table + 0.006) & (Z < table + FLAT_TOP_MAX + 0.005)
        pts = B[om]
        top = float(np.percentile(pts[:, 2], 96))
        face = pts[pts[:, 2] > top - 0.003]
        if len(face) < 40:
            face = pts
        cx = float((face[:, 0].min() + face[:, 0].max()) / 2.0)
        cy = float((face[:, 1].min() + face[:, 1].max()) / 2.0)
        sx = float(np.ptp(face[:, 0]))
        sy = float(np.ptp(face[:, 1]))
        d = face[:, :2] - np.array([cx, cy])
        evals, evecs = np.linalg.eigh(d.T @ d / max(len(d), 1))
        maj = evecs[:, -1]
        obj = dict(seed_px=k, n=int(om.sum()), c=(cx, cy), top=top,
                   span=(sx, sy), yaw=float(np.arctan2(maj[1], maj[0])))

    # basket: largest structure that stays below 0.25 m
    sm = ws & np.isfinite(Z) & (Z > table + 0.015) & (Z < 0.25)
    lab2, n2 = _label(sm)
    best = None
    for i in range(1, n2 + 1):
        m = lab2 == i
        if m.sum() < 500:
            continue
        p = B[m]
        if np.percentile(p[:, 2], 99.5) > 0.25:
            continue
        if best is None or m.sum() > best[0]:
            best = (int(m.sum()), p)
    basket = None
    if best is not None:
        k, p = best
        top = float(np.percentile(p[:, 2], 98))
        rim = p[p[:, 2] > top - 0.012]
        basket = dict(n=k, top=top,
                      c=(float((rim[:, 0].min() + rim[:, 0].max()) / 2.0),
                         float((rim[:, 1].min() + rim[:, 1].max()) / 2.0)),
                      span=(float(np.ptp(rim[:, 0])), float(np.ptp(rim[:, 1]))))
    return table, obj, basket


# ---------------------------------------------------------------- behaviour
def _holding(api):
    g = api.gripper()
    w = float(g.get("width_m", 0.0))
    e = float(g.get("effort", 0.0))
    return (HOLD_WIDTH_RANGE[0] < w < HOLD_WIDTH_RANGE[1] and e > 1.0), w, e


def run(api):
    api.log("instruction=%r" % api.instruction())
    table, obj, basket = perceive(api)
    api.log("table_z=%.4f" % table)
    api.log("obj=%s" % (obj,))
    api.log("basket=%s" % (basket,))
    if obj is None or basket is None:
        return "perception failed obj=%s basket=%s" % (obj is not None, basket is not None)

    ox, oy = obj["c"]
    z_grasp = table + GRASP_Z_ABOVE_TABLE
    bx, by = basket["c"]
    z_release = basket["top"] + RELEASE_ABOVE_RIM

    held = False
    # Retry ladder: spans the aim band around the measured footprint centre.
    ladder = [(0.000, 0.000, 0.000), (0.000, 0.000, -0.004), (0.006, 0.000, 0.002)]
    for att, (dx, dy, dz) in enumerate(ladder):
        api.grip(0.08)
        api.settle(0.2)
        r1 = api.move([ox + dx, oy + dy, 0.14], seconds=2.5)
        r2 = api.move([ox + dx, oy + dy, z_grasp + dz], seconds=2.0)
        e = api.eef()
        api.log("att%d hover_res=%.4f down_res=%.4f eef=%s"
                % (att, r1, r2, np.round(e, 4).tolist()))
        api.grip(0.0)
        api.settle(0.5)
        held, w, ef = _holding(api)
        api.log("att%d after_close width=%.4f effort=%.2f held=%s" % (att, w, ef, held))
        r3 = api.move([ox + dx, oy + dy, CARRY_Z], seconds=2.5)
        api.settle(0.3)
        held, w, ef = _holding(api)
        api.log("att%d after_lift res=%.4f width=%.4f effort=%.2f held=%s"
                % (att, r3, w, ef, held))
        if held:
            break
        api.grip(0.08)
        api.move(list(HOME_XYZ), seconds=2.0)
        api.settle(0.4)
        table2, obj2, basket2 = perceive(api)
        api.log("re-perceive obj=%s" % (obj2,))
        if obj2 is not None:
            ox, oy = obj2["c"]
            z_grasp = table2 + GRASP_Z_ABOVE_TABLE
        if basket2 is not None:
            bx, by = basket2["c"]
            z_release = basket2["top"] + RELEASE_ABOVE_RIM

    if not held:
        api.log("no grasp after %d attempts" % len(ladder))
        return "grasp failed"

    api.move([bx, by, CARRY_Z], seconds=3.0)
    api.settle(0.2)
    held, w, ef = _holding(api)
    api.log("over_basket width=%.4f effort=%.2f held=%s" % (w, ef, held))
    r = api.move([bx, by, z_release], seconds=2.0)
    api.log("descend_res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(0.08)
    api.settle(0.8)
    api.move([bx, by, CARRY_Z], seconds=2.0)
    api.settle(0.3)
    _, w, ef = _holding(api)
    api.log("after_release width=%.4f effort=%.2f" % (w, ef))
    api.move(list(HOME_XYZ), seconds=2.5)
    return "placed"
