"""c2k1clean / obj_tomato_sauce_pos_k1 -- pick up the tomato sauce, place it in
the basket.

Perception: cam_high RGB-D -> base-frame cloud -> table plane -> above-table
clusters on a 1 cm xy grid.  The arm/pedestal column is removed first (it is
the only structure reaching above ARM_Z), so the remaining clusters are the
free-standing props.

Identification: the demo's held gripper width (0.0635 m, from the pack's
gripper_state at the carry keyframe) plus the demo grasp height say the target
is a squat wide cylinder -- a lidded can.  Every distractor in view is either
a tall narrow box/bottle or a flat slab, so ranking clusters by
|width-0.0635| + |height-DEMO_OBJ_H| separates them cleanly.

Placement: the basket is the widest-footprint cluster; release over its
centroid at the demo's rim clearance.
"""
import numpy as np

PROVENANCE = {
    "DEMO_HOLD_W": {
        "source": "pack.json demos[0].keyframes[t=112].gripper_state "
                  "= [0.0265, -0.037] -> |sum| = 0.0635 m jaw span while "
                  "carrying the target",
        "allowed": True},
    "DEMO_GRASP_DZ": {
        "source": "pack.json demos[0].ee_path[t=60] z = 0.0489 minus the table "
                  "plane z = 0.0021 measured as the modal cloud height on "
                  "debug seeds 51/53/55 -> 0.0468 m grasp height above table",
        "allowed": True},
    "DEMO_OBJ_H": {
        "source": "debug seeds 51/53/55 cam_high cloud: the only cluster whose "
                  "footprint matches DEMO_HOLD_W is 0.079 m tall",
        "allowed": True},
    "DEMO_RELEASE_DZ": {
        "source": "pack.json demos[0].ee_path[t=110] z = 0.1802 minus the "
                  "basket rim top 0.144 measured on debug seeds 51/53/55 "
                  "-> 0.036 m release clearance above the rim",
        "allowed": True},
    "ARM_Z": {
        "source": "debug seeds 51/53/55: the robot column reaches z = 0.402 "
                  "while every prop tops out at 0.144; 0.22 separates them",
        "allowed": True},
    "CARRY_Z": {
        "source": "pack.json demos[0].ee_path transport height "
                  "(t=80..110, z = 0.18-0.23); 0.22 clears the 0.144 rim",
        "allowed": True},
    "CELL": {"source": "generic: 1 cm occupancy grid for clustering", "allowed": True},
    "OPEN_W": {
        "source": "api.gripper() at episode start reports width_m = 0.0778 "
                  "(generic controller mechanics: open command)",
        "allowed": True},
    "WORKSPACE": {
        "source": "debug seeds 51/53/55: all props lie in x[-0.30,0.30] "
                  "y[-0.40,0.45]; the runner prints the same workspace",
        "allowed": True},
    "BAD_SCORE": {
        "source": "debug seeds 51-65 v1 logs: the can scores 0.0016 and the "
                  "nearest distractor 0.0664; 0.030 sits between them",
        "allowed": True},
    "PARK": {
        "source": "api.eef() at episode start = (-0.1485, 0, 0.2613); parking "
                  "is the same xy lifted clear of the 0.144 prop band",
        "allowed": True},
}

DEMO_HOLD_W = 0.0635
DEMO_GRASP_DZ = 0.0468
DEMO_OBJ_H = 0.079
DEMO_RELEASE_DZ = 0.036
ARM_Z = 0.22
CARRY_Z = 0.22
CELL = 0.01
OPEN_W = 0.078
WORKSPACE = (-0.30, 0.30, -0.40, 0.45)
BAD_SCORE = 0.030
PARK = (-0.1485, 0.0, 0.42)


# ---------------------------------------------------------------------------
# perception

def _cloud(frame):
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x = (uu - cx) * d / fx
    y = (vv - cy) * d / fy
    P = np.stack([x, y, d, np.ones_like(d)], axis=-1)
    B = P @ T.T
    return B[..., :3], np.isfinite(d) & (d > 0)


_NB26 = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)
         if (a, b, c) != (0, 0, 0)]


def _label3(vox):
    """26-connected components over a set of (i, j, k) voxels -> {vox: label}."""
    lab, n = {}, 0
    for s in vox:
        if s in lab:
            continue
        lab[s] = n
        stack = [s]
        while stack:
            i, j, k = stack.pop()
            for a, b, c in _NB26:
                q = (i + a, j + b, k + c)
                if q in vox and q not in lab:
                    lab[q] = n
                    stack.append(q)
        n += 1
    return lab, n


def perceive(api, camera="cam_high"):
    """-> (z0, [cluster dicts])."""
    f = api.capture(camera)
    rgb = np.asarray(f.rgb).astype(float)
    B, ok = _cloud(f)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    zv = Z[ok]
    hist, edges = np.histogram(zv, bins=300)
    z0 = float(0.5 * (edges[np.argmax(hist)] + edges[np.argmax(hist) + 1]))

    x0, x1, y0, y1 = WORKSPACE
    sel = ok & (Z > z0 + 0.012) & (X > x0) & (X < x1) & (Y > y0) & (Y < y1)
    xs, ys, zs = X[sel], Y[sel], Z[sel]
    cols = rgb[sel]

    # 3-D voxel connected components: the arm's links float above the props,
    # so a prop whose xy footprint hides under an arm link is still its own
    # component (a 2-D column grid would fuse them).
    gi = np.floor((xs - x0) / CELL).astype(int)
    gj = np.floor((ys - y0) / CELL).astype(int)
    gk = np.floor((zs - z0) / CELL).astype(int)
    trip = np.stack([gi, gj, gk], axis=1)
    uniq, inv, cnt = np.unique(trip, axis=0, return_inverse=True,
                               return_counts=True)
    keep = cnt >= 3
    vox = {tuple(int(v) for v in uniq[i]) for i in np.nonzero(keep)[0]}
    lab3, nlab = _label3(vox)
    vlab = np.array([lab3.get(tuple(int(v) for v in u), -1) for u in uniq], int)
    vlab[~keep] = -1
    plab = vlab[inv]

    out = []
    for c in range(nlab):
        m = plab == c
        if m.sum() < 40:
            continue
        top = float(zs[m].max())
        if top - z0 > ARM_Z:            # the robot column, not a prop
            continue
        h = top - z0
        ex = float(xs[m].max() - xs[m].min())
        ey = float(ys[m].max() - ys[m].min())
        # centre from the top band only: for a lidded can the whole lid disc
        # is visible, so its area centroid is the cylinder axis, while the
        # full-cluster centroid is biased toward the camera-facing wall.
        band = zs[m] > top - 0.015
        bx, by = (xs[m][band], ys[m][band]) if band.sum() >= 15 else (xs[m], ys[m])
        out.append({
            "n": int(m.sum()),
            "xy": (float(bx.mean()), float(by.mean())),
            "xy_all": (float(xs[m].mean()), float(ys[m].mean())),
            "top": top, "h": h, "ex": ex, "ey": ey,
            "w": min(ex, ey), "wmax": max(ex, ey),
            "rgb": cols[m].mean(axis=0),
        })
    return z0, out


# ---------------------------------------------------------------------------

def _score(c):
    """Distance from the demo's target signature: a can as wide as the demo's
    carried jaw span and as tall as the matching debug-seed cluster."""
    return abs(c["w"] - DEMO_HOLD_W) + abs(c["h"] - DEMO_OBJ_H)


def _pick(cl):
    """-> (basket, target, props) or (None, None, None)."""
    if len(cl) < 2:
        return None, None, None
    basket = max(cl, key=lambda c: c["wmax"])     # basket = widest footprint
    props = sorted([c for c in cl if c is not basket], key=_score)
    if not props:
        return basket, None, None
    return basket, props[0], props


def run(api):
    api.log("v2 start eef=%s" % np.round(api.eef(), 4).tolist())
    z0, cl = perceive(api)
    basket, tgt, props = _pick(cl)
    api.log("z0=%.4f nclusters=%d best=%s"
            % (z0, len(cl), None if tgt is None else round(_score(tgt), 4)))
    for i, c in enumerate(cl):
        api.log("C%d n=%d xy=(%.3f,%.3f) all=(%.3f,%.3f) h=%.3f ext=(%.3f,%.3f) rgb=%s"
                % (i, c["n"], c["xy"][0], c["xy"][1], c["xy_all"][0], c["xy_all"][1],
                   c["h"], c["ex"], c["ey"], np.round(c["rgb"], 0).tolist()))

    # If nothing looks like the can, the arm is probably standing in front of
    # it: park high out of the prop band and look again.
    if tgt is None or _score(tgt) > BAD_SCORE:
        api.log("poor first read -> parking the arm and re-capturing")
        api.move([PARK[0], PARK[1], PARK[2]], seconds=2.5)
        api.settle(0.3)
        z0b, clb = perceive(api)
        bb, tb, pb = _pick(clb)
        api.log("retry nclusters=%d best=%s"
                % (len(clb), None if tb is None else round(_score(tb), 4)))
        for i, c in enumerate(clb):
            api.log("R%d n=%d xy=(%.3f,%.3f) h=%.3f ext=(%.3f,%.3f)"
                    % (i, c["n"], c["xy"][0], c["xy"][1], c["h"], c["ex"], c["ey"]))
        if tb is not None and (tgt is None or _score(tb) < _score(tgt)):
            z0, cl, basket, tgt, props = z0b, clb, bb, tb, pb

    if tgt is None or basket is None:
        api.log("no target found")
        return "no-target"
    api.log("basket xy=(%.3f,%.3f) top=%.3f | target xy=(%.3f,%.3f) h=%.3f "
            "w=%.3f score=%.4f | runner-up score=%.4f"
            % (basket["xy_all"][0], basket["xy_all"][1], basket["top"],
               tgt["xy"][0], tgt["xy"][1], tgt["h"], tgt["w"], _score(tgt),
               _score(props[1]) if len(props) > 1 else -1))

    tx, ty = tgt["xy"]
    z_grasp = max(z0 + DEMO_GRASP_DZ, tgt["top"] - 0.035)
    z_grasp = min(z_grasp, tgt["top"] - 0.012)

    # ---- grasp ladder: nominal centre, then small radial nudges -----------
    offsets = [(0.0, 0.0), (0.008, 0.0), (-0.008, 0.0), (0.0, 0.008), (0.0, -0.008)]
    held = False
    for k, (dx, dy) in enumerate(offsets):
        api.grip(OPEN_W)
        api.move([tx + dx, ty + dy, max(z0 + 0.14, tgt["top"] + 0.05)], seconds=2.0)
        r = api.move([tx + dx, ty + dy, z_grasp], seconds=2.0)
        api.grip(0.0)
        api.settle(0.4)
        g = api.gripper()
        api.log("try%d off=(%.3f,%.3f) zg=%.3f resid=%.4f grip=%s"
                % (k, dx, dy, z_grasp, r, {kk: round(vv, 4) for kk, vv in g.items()}))
        # a closed-on-nothing jaw runs to ~0; holding the can leaves a gap
        if g["effort"] >= 2.0 and g["width_m"] > 0.03:
            held = True
            break
        api.grip(OPEN_W)
        api.move([tx + dx, ty + dy, max(z0 + 0.14, tgt["top"] + 0.05)], seconds=1.5)

    api.log("held=%s after ladder" % held)

    # ---- lift, transport, release -----------------------------------------
    api.move([tx, ty, CARRY_Z], seconds=2.0)
    g = api.gripper()
    api.log("post-lift grip=%s eef=%s"
            % ({kk: round(vv, 4) for kk, vv in g.items()},
               np.round(api.eef(), 4).tolist()))

    bx, by = basket["xy_all"]
    zr = basket["top"] + DEMO_RELEASE_DZ
    api.move([bx, by, CARRY_Z], seconds=3.0)
    api.move([bx, by, max(zr, basket["top"] + 0.02)], seconds=2.0)
    api.settle(0.3)
    api.grip(OPEN_W)
    api.settle(0.6)
    api.log("released at (%.3f,%.3f,%.3f) grip=%s"
            % (bx, by, zr, {kk: round(vv, 4) for kk, vv in api.gripper().items()}))
    api.move([bx, by, CARRY_Z + 0.04], seconds=2.0)
    api.settle(0.5)
    return "v1 held=%s" % held
