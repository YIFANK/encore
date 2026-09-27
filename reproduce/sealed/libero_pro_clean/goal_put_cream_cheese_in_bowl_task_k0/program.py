import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug-seed 51-57 cam_high depth: median table plane z = 0.901 m", "allowed": True},
    "GRASP_DEPTH": {"source": "debug-seed 51/57 v2 close-height sweep: closes at eef z = neck_top-0.013/-0.033 read width 0.0148-0.0150 with effort 3.0, higher closes read 0.001", "allowed": True},
    "CARRY_Z": {"source": "debug-seed 51-57 geometry: bowl rim 0.952 m, bottle 0.158 m tall, grasp offset ~0.133 m -> base clears the rim at eef 1.15", "allowed": True},
    "BOWL_FLOOR_Z": {"source": "debug-seed 51-57 cam_high depth: bowl inner surface bottoms out at z ~0.918-0.924", "allowed": True},
    "SEG_THRESHOLDS": {"source": "debug-seed 51-57 cam_high depth component survey (table 0.901, bottle top 1.059, bowl rim 0.952, arm top 1.37)", "allowed": True},
}
TABLE_Z = 0.901
GRASP_DEPTH = 0.030
CARRY_Z = 1.15
BOWL_FLOOR_Z = 0.928


def cloud(f):
    d = np.asarray(f.depth, dtype=np.float32)
    K = np.asarray(f.intrinsics, dtype=np.float64)
    T = np.asarray(f.t_base_cam, dtype=np.float64)
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    x = (u - K[0, 2]) / K[0, 0] * d
    y = (v - K[1, 2]) / K[1, 1] * d
    pc = np.stack([x, y, d], -1)
    return pc @ T[:3, :3].T + T[:3, 3]


def label(mask):
    """4-connected components, pure numpy union-find."""
    H, W = mask.shape
    idx = -np.ones((H, W), int)
    ys, xs = np.where(mask)
    for k in range(len(ys)):
        idx[ys[k], xs[k]] = k
    parent = list(range(len(ys)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for k in range(len(ys)):
        y, x = ys[k], xs[k]
        if y + 1 < H and idx[y + 1, x] >= 0:
            union(k, idx[y + 1, x])
        if x + 1 < W and idx[y, x + 1] >= 0:
            union(k, idx[y, x + 1])
    roots = {}
    out = np.zeros((H, W), int)
    for k in range(len(ys)):
        r = find(k)
        if r not in roots:
            roots[r] = len(roots) + 1
        out[ys[k], xs[k]] = roots[r]
    return out, len(roots)


def perceive(api):
    f = api.capture("cam_high")
    P = cloud(f)
    rgb = np.asarray(f.rgb, dtype=np.float64)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    ws = (X > -0.40) & (X < 0.16) & (Y > -0.115) & (Y < 0.10) & (Z > 0.80) & (Z < 1.40)
    m = ws & (Z > TABLE_Z + 0.025)
    lab, n = label(m)
    bottle = None
    bowl = None
    for i in range(1, n + 1):
        mm = lab == i
        npx = int(mm.sum())
        if npx < 200:
            continue
        ztop = float(Z[mm].max())
        bright = float(rgb[mm].mean())
        yspan = float(Y[mm].max() - Y[mm].min())
        api.log("COMP %d px%d ztop%.3f rgb%.0f x[%.3f,%.3f] y[%.3f,%.3f]"
                % (i, npx, ztop, bright, X[mm].min(), X[mm].max(), Y[mm].min(), Y[mm].max()))
        if TABLE_Z + 0.10 < ztop < TABLE_Z + 0.20 and npx < 4000:
            if bottle is None or npx > int(bottle.sum()):
                bottle = mm
        elif 0.06 < yspan < 0.16:
            if bowl is None or npx > int(bowl.sum()):
                bowl = mm
    out = {}
    if bottle is not None:
        neck = bottle & (Z > 1.015)
        body = bottle & (Z < TABLE_Z + 0.055)
        out["neck_y"] = float((Y[neck].min() + Y[neck].max()) / 2)
        out["neck_x"] = float(X[neck].max()) - 0.0075
        out["neck_top"] = float(Z[bottle].max())
        out["body_y"] = float((Y[body].min() + Y[body].max()) / 2)
        out["body_xmax"] = float(X[body].max())
    if bowl is not None:
        out["bowl_y"] = float((Y[bowl].min() + Y[bowl].max()) / 2)
        out["bowl_x"] = float((X[bowl].min() + X[bowl].max()) / 2)
        out["bowl_rim_z"] = float(Z[bowl].max())
        out["bowl_yspan"] = float(Y[bowl].max() - Y[bowl].min())
    api.log("PERC %s" % (out,))
    # validate our deprojection against the api's own
    if bottle is not None:
        vv, uu = np.where(bottle)
        k = len(vv) // 2
        api.log("DEPROJ api=%s ours=%s" % (np.asarray(f.deproject(int(uu[k]), int(vv[k]))).round(4).tolist(),
                                           P[vv[k], uu[k]].round(4).tolist()))
    return out


def run(api):
    o = perceive(api)
    if "neck_x" not in o or "bowl_x" not in o:
        api.log("PERCEPTION FAIL")
        return
    nx, ny, ntop = o["neck_x"], o["neck_y"], o["neck_top"]
    bx, by = o["bowl_x"], o["bowl_y"]
    api.grip(0.08)
    api.move([nx, ny, ntop + 0.08], seconds=2.0)
    api.move([nx, ny, ntop - GRASP_DEPTH], seconds=1.5)
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    ez = float(api.eef()[2])
    api.log("GRASP eef%s w%.4f e%.2f" % (np.asarray(api.eef()).round(4).tolist(), g["width_m"], g["effort"]))
    base_off = ez - TABLE_Z
    api.move([nx, ny, CARRY_Z], seconds=2.0)
    g = api.gripper()
    api.log("LIFT eef%s w%.4f e%.2f base_off%.4f" % (np.asarray(api.eef()).round(4).tolist(), g["width_m"], g["effort"], base_off))
    api.move([bx, by, CARRY_Z], seconds=2.5)
    api.log("OVER eef%s" % (np.asarray(api.eef()).round(4).tolist(),))
    place_z = BOWL_FLOOR_Z + base_off
    r = api.move([bx, by, place_z], seconds=2.0)
    g = api.gripper()
    api.log("PLACE cmd%.4f eef%s res%s w%.4f e%.2f"
            % (place_z, np.asarray(api.eef()).round(4).tolist(), np.asarray(r).round(4).tolist(), g["width_m"], g["effort"]))
    api.settle(0.6)
    api.grip(0.08)
    api.settle(0.8)
    api.log("RELEASED eef%s" % (np.asarray(api.eef()).round(4).tolist(),))
    api.move([bx, by, CARRY_Z], seconds=2.0)
    api.settle(0.5)
    perceive(api)
