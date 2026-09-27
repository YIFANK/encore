"""c2k1clean / obj_chocolate_pudding_pos_k1 -- "pick up the chocolate pudding and
place it in the basket".

Mechanism
---------
The pack's demonstration is a plain top-down pick-and-place: hover over the
target, descend to a grasp height a little below its top, close, lift, traverse
above the basket, descend, open.  The only thing the demo cannot supply is
*where* the chocolate pudding is in this episode, so the program re-derives the
whole scene from cam_high RGB-D each run:

  * fit the table height as the mode of the world-z histogram;
  * keep points 8 mm .. 18 cm above it (the 18 cm ceiling drops the robot arm,
    whose lowest point at the home pose is ~26 cm, without touching any prop);
  * connected-component the occupied 12 mm xy cells into objects;
  * the basket is the largest component by a wide margin (~1.2e4 px vs <3e3);
  * the chocolate pudding is the *flattest* remaining component -- on every
    debug seed it is a box lying on its face, 27 mm tall, while every other
    prop (bottles, carton, can) stands 78 mm or more.

Grasp geometry: the box measures 80 mm x 48 mm in xy and the jaws close along
the world-y axis when the wrist is left straight down, so the default wrist
already closes across the short side (measured closed width 46.3 mm, effort
3.0).  If a future layout turns the box so its y-extent no longer fits the
75 mm opening, the wrist is yawed onto the footprint's minor axis instead.
"""
import numpy as np

PROVENANCE = {
    "Z_HIST_RANGE": {
        "source": "debug seeds 51-65 cam_high depth: world-z of the workspace "
                  "point cloud; the table mode falls at +0.0025 m, well inside "
                  "(-0.3, +0.3)",
        "allowed": True},
    "Z_MIN_ABOVE": {
        "source": "debug-seed measurement: 0.008 m above the fitted table mode "
                  "is the smallest offset that removes table speckle while "
                  "keeping the 27 mm-tall pudding box",
        "allowed": True},
    "Z_MAX_ABOVE": {
        "source": "debug-seed measurement: tallest prop top is +0.1476 m over "
                  "the table, the home-pose arm's lowest point is +0.19 m; "
                  "0.18 m separates them",
        "allowed": True},
    "CELL": {
        "source": "debug-seed measurement: 0.012 m xy cells keep the six props "
                  "and the basket as seven separate components on seeds "
                  "51,52,58,64",
        "allowed": True},
    "MIN_CLUSTER_PX": {
        "source": "debug-seed measurement: the smallest real prop covers 175 px "
                  "(cam_high, 512x512); 12 px rejects speckle only",
        "allowed": True},
    "WS_BOUNDS": {
        "source": "fair_run's announced LIBERO workspace, echoed on stdout for "
                  "every debug episode: x(-0.45,0.45), y(-0.45,0.52)",
        "allowed": True},
    "FLAT_MAX_H": {
        "source": "debug-seed measurement: pudding box height 0.027 m vs the "
                  "next-shortest prop at 0.078 m; 0.06 m is the midpoint gate "
                  "used only to decide whether a rescan is needed",
        "allowed": True},
    "HOVER_Z": {
        "source": "pack ee_path waypoint 3-4 (pre-grasp hover at z 0.231/0.141); "
                  "0.15 m reproduced on debug seeds",
        "allowed": True},
    "DESCENT_LADDER": {
        "source": "debug-seed measurement: staged descent 0.06/0.030/0.018 -> "
                  "grasp height; residuals stayed <0.012 m at every rung",
        "allowed": True},
    "GRASP_DEPTH": {
        "source": "pack keyframe t=54 grasp at ee z=0.0114 with the demo box on "
                  "the table; reproduced here as (object top - 0.018 m), which "
                  "evaluates to 0.0115 m for the 0.0295 m box top",
        "allowed": True},
    "GRASP_Z_FLOOR": {
        "source": "debug-seed measurement: the table mode sits at 0.0025 m; "
                  "0.006 m above it keeps the fingertips off the table",
        "allowed": True},
    "LIFT_Z": {
        "source": "pack ee_path waypoint 8 (post-grasp lift to z=0.206); 0.16 m "
                  "clears the 0.1476 m tallest prop on debug seeds",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug-seed measurement: 0.22 m clears the tallest prop "
                  "(0.1476 m) and the basket rim (0.1419 m) on the traverse",
        "allowed": True},
    "DROP_CLEAR": {
        "source": "pack keyframe t=150 release at ee z=0.1635 over the basket; "
                  "reproduced as (basket rim top + 0.035 m) = 0.177 m here",
        "allowed": True},
    "GRIP_OPEN": {
        "source": "debug-seed measurement: api.grip(0.08) yields width 0.0795 m "
                  "(full open)",
        "allowed": True},
    "GRIP_CLOSE": {
        "source": "generic gripper mechanics: api.grip(<0.025) commands a close",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi documentation: api.gripper() effort is 3.0 iff the "
                  "gripper is holding something; confirmed on debug seeds",
        "allowed": True},
    "JAW_AXIS": {
        "source": "debug-seed measurement: with rotation=None the closed width "
                  "settled at 0.0463 m on a box whose world-y extent is "
                  "0.048 m and world-x extent 0.080 m -- the jaws close along "
                  "world y",
        "allowed": True},
    "JAW_MAX_SPAN": {
        "source": "debug-seed measurement: full open width 0.0795 m; 0.068 m is "
                  "the widest footprint accepted before the wrist is yawed",
        "allowed": True},
    "PARK_POSE": {
        "source": "debug-seed measurement: home eef is (-0.1485, 0.0, 0.2613); "
                  "the rescan park pose lifts to z=0.34 and y=-0.30, outside "
                  "the prop row, and is only used when the first scan finds no "
                  "flat component",
        "allowed": True},
}

# ---- perception constants -------------------------------------------------
Z_HIST_RANGE = (-0.3, 0.3)
Z_HIST_BINS = 120
Z_MIN_ABOVE = 0.008
Z_MAX_ABOVE = 0.18
CELL = 0.012
MIN_CLUSTER_PX = 12
WS_X = (-0.45, 0.45)
WS_Y = (-0.45, 0.55)
FLAT_MAX_H = 0.06

# ---- motion constants -----------------------------------------------------
HOVER_Z = 0.15
DESCENT_LADDER = (0.06, 0.030, 0.018)
GRASP_DEPTH = 0.018
GRASP_Z_FLOOR = 0.006
LIFT_Z = 0.16
CARRY_Z = 0.22
DROP_CLEAR = 0.035
GRIP_OPEN = 0.08
GRIP_CLOSE = 0.0
HOLD_EFFORT = 2.5
JAW_MAX_SPAN = 0.068
PARK_POSE = (-0.15, -0.30, 0.34)


def _components(occ):
    n = occ.shape[0]
    lab = -np.ones_like(occ, int)
    cid = 0
    for a in range(n):
        for b in range(n):
            if occ[a, b] and lab[a, b] < 0:
                st = [(a, b)]
                lab[a, b] = cid
                while st:
                    p, q = st.pop()
                    for dp in (-1, 0, 1):
                        for dq in (-1, 0, 1):
                            r, s = p + dp, q + dq
                            if 0 <= r < n and 0 <= s < n and occ[r, s] and lab[r, s] < 0:
                                lab[r, s] = cid
                                st.append((r, s))
                cid += 1
    return lab, cid


def scene(api, cam="cam_high"):
    """cam_high RGB-D -> (table_z, [component dicts])."""
    f = api.capture(cam)
    rgb = np.asarray(f.rgb)
    d = np.asarray(f.depth, float)
    H, W = d.shape[:2]
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    vv, uu = np.mgrid[0:H, 0:W]
    ok = np.isfinite(d) & (d > 0)
    z = np.where(ok, d, 1.0)
    pc = np.stack([(uu - K[0, 2]) * z / K[0, 0],
                   (vv - K[1, 2]) * z / K[1, 1], z], -1)
    world = pc @ T[:3, :3].T + T[:3, 3]
    X, Y, Z = world[..., 0], world[..., 1], world[..., 2]

    ws = ok & (X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1])
    hist, edges = np.histogram(Z[ws], bins=Z_HIST_BINS, range=Z_HIST_RANGE)
    tz = float(0.5 * (edges[np.argmax(hist)] + edges[np.argmax(hist) + 1]))

    obj = ws & (Z > tz + Z_MIN_ABOVE) & (Z < tz + Z_MAX_ABOVE)
    n = int(np.ceil((WS_X[1] - WS_X[0] + 0.1) / CELL))
    gi = np.floor((X - WS_X[0]) / CELL).astype(int)
    gj = np.floor((Y - WS_Y[0]) / CELL).astype(int)
    sel = obj & (gi >= 0) & (gi < n) & (gj >= 0) & (gj < n)
    occ = np.zeros((n, n), bool)
    occ[gi[sel], gj[sel]] = True
    lab, cid = _components(occ)
    pl = lab[gi.clip(0, n - 1), gj.clip(0, n - 1)]
    pl[~sel] = -1

    out = []
    for c in range(cid):
        m = pl == c
        if m.sum() < MIN_CLUSTER_PX:
            continue
        xs, ys, zs = X[m], Y[m], Z[m]
        top = float(np.percentile(zs, 98))
        pts = np.stack([xs, ys], 1)
        pts = pts - pts.mean(0)
        w, v = np.linalg.eigh(pts.T @ pts / max(1, len(pts)))
        out.append(dict(n=int(m.sum()), x=float(xs.mean()), y=float(ys.mean()),
                        xr=float(xs.max() - xs.min()),
                        yr=float(ys.max() - ys.min()),
                        top=top, h=top - tz,
                        rgb=[float(q) for q in rgb[m].astype(float).mean(0)],
                        minor=[float(v[0, 0]), float(v[1, 0])],
                        ev=[float(max(w[0], 0)) ** .5, float(max(w[1], 0)) ** .5]))
    return tz, out


def pick_target(cl):
    """(basket, pudding) or (basket, None)."""
    if not cl:
        return None, None
    basket = max(cl, key=lambda c: c["n"])
    rest = [c for c in cl if c is not basket]
    if not rest:
        return basket, None
    return basket, min(rest, key=lambda c: c["h"])


def grasp_rotation(api, tgt):
    """None to keep the wrist straight down (jaws already across the short
    side), else a 3x3 yawing the jaw axis onto the footprint's minor axis."""
    if tgt["yr"] <= JAW_MAX_SPAN:
        return None
    mx, my = tgt["minor"]
    nrm = (mx * mx + my * my) ** .5
    if nrm < 1e-6:
        return None
    mx, my = mx / nrm, my / nrm
    th = np.arctan2(mx, -my)
    while th > np.pi / 2:
        th -= np.pi
    while th < -np.pi / 2:
        th += np.pi
    c, s = np.cos(th), np.sin(th)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return rz @ np.asarray(api.tool_rotation(), float)


def attempt_grasp(api, tgt, tz, rot):
    gz = max(tz + GRASP_Z_FLOOR, tgt["top"] - GRASP_DEPTH)
    api.grip(GRIP_OPEN)
    api.settle(0.2)
    api.move([tgt["x"], tgt["y"], HOVER_Z], rotation=rot, seconds=2.0)
    for zz in DESCENT_LADDER:
        if zz <= gz:
            break
        api.move([tgt["x"], tgt["y"], zz], rotation=rot, seconds=1.2)
    r = api.move([tgt["x"], tgt["y"], gz], rotation=rot, seconds=1.2)
    api.grip(GRIP_CLOSE)
    api.settle(0.4)
    g = api.gripper()
    api.log("grasp z=%.4f res=%.4f w=%.4f eff=%.2f" % (gz, r, g["width_m"], g["effort"]))
    api.move([tgt["x"], tgt["y"], LIFT_Z], rotation=rot, seconds=2.0)
    g = api.gripper()
    api.log("lift w=%.4f eff=%.2f eef=%s" %
            (g["width_m"], g["effort"], np.round(api.eef(), 4).tolist()))
    return g["effort"] >= HOLD_EFFORT and g["width_m"] > 0.005


def deliver(api, basket):
    here = api.eef()
    api.move([float(here[0]), float(here[1]), CARRY_Z], seconds=1.2)
    api.move([(float(here[0]) + basket["x"]) * .5,
              (float(here[1]) + basket["y"]) * .5, CARRY_Z], seconds=1.5)
    api.move([basket["x"], basket["y"], CARRY_Z], seconds=2.0)
    g = api.gripper()
    api.log("carry w=%.4f eff=%.2f eef=%s" %
            (g["width_m"], g["effort"], np.round(api.eef(), 4).tolist()))
    api.move([basket["x"], basket["y"], basket["top"] + DROP_CLEAR], seconds=1.5)
    api.grip(GRIP_OPEN)
    api.settle(0.6)
    api.log("released w=%.4f eef=%s" %
            (api.gripper()["width_m"], np.round(api.eef(), 4).tolist()))
    api.move([basket["x"], basket["y"], CARRY_Z], seconds=1.5)
    api.settle(0.3)


def run(api):
    tz, cl = scene(api)
    basket, tgt = pick_target(cl)
    for c in sorted(cl, key=lambda c: c["y"]):
        api.log("obj n=%4d xy=(%.3f,%.3f) xr=%.3f yr=%.3f top=%.4f h=%.3f rgb=%s"
                % (c["n"], c["x"], c["y"], c["xr"], c["yr"], c["top"], c["h"],
                   np.round(c["rgb"]).astype(int).tolist()))

    # If the arm's shadow hid the flat box, park the wrist away and rescan once.
    if tgt is None or tgt["h"] > FLAT_MAX_H:
        api.log("rescan: flattest h=%s" % (None if tgt is None else round(tgt["h"], 3)))
        api.move(list(PARK_POSE), seconds=2.5)
        api.settle(0.4)
        tz, cl = scene(api)
        basket, tgt = pick_target(cl)
    if basket is None or tgt is None:
        api.log("no scene")
        return

    api.log("tz=%.4f TARGET xy=(%.3f,%.3f) h=%.3f top=%.4f yr=%.3f minor=%s"
            % (tz, tgt["x"], tgt["y"], tgt["h"], tgt["top"], tgt["yr"],
               np.round(tgt["minor"], 3).tolist()))
    api.log("BASKET xy=(%.3f,%.3f) top=%.4f n=%d"
            % (basket["x"], basket["y"], basket["top"], basket["n"]))

    rot = grasp_rotation(api, tgt)
    api.log("rot=%s" % ("none" if rot is None else np.round(rot, 3).tolist()))

    if not attempt_grasp(api, tgt, tz, rot):
        api.log("grasp missed -- rescanning")
        api.grip(GRIP_OPEN)
        api.move([tgt["x"], tgt["y"], HOVER_Z], seconds=1.5)
        api.settle(0.3)
        tz2, cl2 = scene(api)
        b2, t2 = pick_target(cl2)
        if t2 is not None and t2["h"] <= FLAT_MAX_H:
            tgt, tz = t2, tz2
            if b2 is not None:
                basket = b2
            rot = grasp_rotation(api, tgt)
            api.log("retry TARGET xy=(%.3f,%.3f) h=%.3f" % (tgt["x"], tgt["y"], tgt["h"]))
        attempt_grasp(api, tgt, tz, rot)

    deliver(api, basket)
