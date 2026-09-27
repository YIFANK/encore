"""c2k1clean / spa_bowl_on_cookie_box_task_k1 -- v4.

Intent: pick the akita black bowl on TOP OF THE CABINET, place it on the plate.

Pipeline
  1. two cam_high captures with the arm displaced between them; pixels whose
     depth is unchanged are static scene (removes the robot from perception).
  2. the target is the HIGHEST compact bowl-interior cluster in the static
     scene (the cabinet is the tallest support; the demo packs put the
     cabinet-top bowl grasp at z=1.159 vs 0.953 for the cookie-box bowl).
  3. rim-pinch ladder: jaw axis x radial sign, close, verify with
     gripper effort/width, retry on failure.
  4. plate = the wide, table-height bright disc; the jaws hold the RIM, so
     the drop point is offset by the grasp radius; release just clear.
"""
import numpy as np

PROVENANCE = {
    "GRASP_DEPTH": {
        "source": "pack grasp keyframes vs my own debug-seed rim-top "
                  "measurement: task_mate t=123 grasp ee z=1.159 with the "
                  "cabinet-bowl rim top measured at 1.179 on seed 51 "
                  "(v1 log, cluster c7); task_k1 t=55 grasp ee z=0.953 with "
                  "the cookie-box bowl rim top measured at 0.970 (v1 log, "
                  "bowl6). Both give ~0.018 m below the rim top.",
        "allowed": True},
    "CLOSED_RIM_W": {
        "source": "pack keyframes gripper_state at the grasp: task_k1 t=55 "
                  "[0.004,-0.0045], task_mate t=123 [0.0026,-0.0027] -> the "
                  "jaws hold ~5-9 mm of bowl wall, i.e. a rim pinch",
        "allowed": True},
    "OPEN_W": {"source": "pack keyframes gripper_state open ~0.0393 per "
                         "finger (total ~0.079)", "allowed": True},
    "RELEASE_ABOVE_PLATE": {
        "source": "task_k1 keyframe t=86 release ee z=0.9476 and task_mate "
                  "t=123 z=0.9728, with the plate rim measured at z=0.9196 "
                  "on debug seeds; and v2 debug seeds 53/57 where the "
                  "descent to plate_z+0.040 stalled at eef z=0.955 (bowl "
                  "bottom hangs ~0.036 below the jaws) -> release just clear "
                  "at plate_z+0.046", "allowed": True},
    "PLACE_XY_FALLBACK": {
        "source": "task_k1 t=86 ee (0.0687,0.2235) and task_mate t=123 ee "
                  "(0.0527,0.2182) -- used only if the plate is not detected",
        "allowed": True},
    "PROBE_SHIFT": {"source": "generic: any safe arm displacement between the "
                              "two captures makes the robot's own pixels "
                              "differ", "allowed": True},
    "TABLE_BIN": {"source": "debug-seed measurement: the dominant depth plane "
                            "is the table at z=0.9025 (v1 log)",
                  "allowed": True},
}

GRASP_DEPTH = 0.018
OPEN_W = 0.08
RELEASE_ABOVE_PLATE = 0.046
PLACE_XY_FALLBACK = np.array([0.060, 0.221])
PROBE_SHIFT = np.array([0.10, 0.12, 0.08])


# ---------------------------------------------------------------- perception
def _xyz(frame):
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape[:2]
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    ok = np.isfinite(d) & (d > 1e-4)
    z = np.where(ok, d, 1.0)
    x = (uu - K[0, 2]) * z / K[0, 0]
    y = (vv - K[1, 2]) * z / K[1, 1]
    B = np.stack([x, y, z, np.ones_like(z)], axis=-1) @ T.T
    return B[..., :3], ok, d


def _label(mask):
    from scipy import ndimage
    return ndimage.label(mask)


class Scene(object):
    """Static (robot-free) geometry of the table from cam_high."""

    def __init__(self, api):
        fa = api.capture("cam_high")
        e0 = np.asarray(api.eef(), float)
        api.move(e0 + PROBE_SHIFT, seconds=1.6)
        fb = api.capture("cam_high")
        XA, okA, dA = _xyz(fa)
        XB, okB, dB = _xyz(fb)
        static = okA & okB & (np.abs(dA - dB) < 0.008)
        self.XYZ, self.ok, self.rgb = XB, static, np.asarray(fb.rgb, int)
        X, Y, Z = self.XYZ[..., 0], self.XYZ[..., 1], self.XYZ[..., 2]
        self.ws = self.ok & (X > -0.42) & (X < 0.45) & (np.abs(Y) < 0.55) \
            & (Z > 0.60) & (Z < 1.70)
        hist, edges = np.histogram(Z[self.ws], bins=np.arange(0.60, 1.70,
                                                             0.005))
        self.table_z = float(edges[int(np.argmax(hist))] + 0.0025)
        self.home = e0

    def clusters(self, api, tag):
        X, Y, Z = self.XYZ[..., 0], self.XYZ[..., 1], self.XYZ[..., 2]
        mn, mx = self.rgb.min(axis=2), self.rgb.max(axis=2)
        bright = (mn > 105) & ((mx - mn) < 60)
        lab, n = _label(self.ws & bright & (Z > self.table_z + 0.008))
        out = []
        for i in range(1, n + 1):
            m = lab == i
            a = int(m.sum())
            if a < 60:
                continue
            out.append(dict(area=a, ztop=float(np.percentile(Z[m], 97)),
                            cx=float(X[m].mean()), cy=float(Y[m].mean()),
                            ex=float(X[m].max() - X[m].min()),
                            ey=float(Y[m].max() - Y[m].min())))
        out.sort(key=lambda d: -d["ztop"])
        for k, d in enumerate(out[:14]):
            api.log("[%s] k%d a=%d ztop=%.3f c=(%.3f,%.3f) e=(%.3f,%.3f)"
                    % (tag, k, d["area"], d["ztop"], d["cx"], d["cy"],
                       d["ex"], d["ey"]))
        return out

    def ring(self, cx, cy, ztop, rad=0.10, band=0.016):
        X, Y, Z = self.XYZ[..., 0], self.XYZ[..., 1], self.XYZ[..., 2]
        m = self.ok & (np.abs(X - cx) < rad) & (np.abs(Y - cy) < rad) \
            & (Z > ztop - band) & (Z < ztop + 0.04)
        if int(m.sum()) < 30:
            return cx, cy, 0.055, ztop, 0
        px, py, pz = X[m], Y[m], Z[m]
        c = np.array([0.5 * (px.min() + px.max()), 0.5 * (py.min() + py.max())])
        r = float(np.percentile(np.hypot(px - c[0], py - c[1]), 85))
        return float(c[0]), float(c[1]), r, float(np.percentile(pz, 96)), \
            int(m.sum())

    def support_z(self, cx, cy, ztop):
        """Height of whatever the object at (cx,cy) is standing on."""
        X, Y, Z = self.XYZ[..., 0], self.XYZ[..., 1], self.XYZ[..., 2]
        m = self.ok & (np.abs(X - cx) < 0.18) & (np.abs(Y - cy) < 0.18) \
            & (Z < ztop - 0.03) & (Z > self.table_z + 0.01)
        if int(m.sum()) < 50:
            return self.table_z
        return float(np.median(Z[m]))


# ------------------------------------------------------------------ helpers
def holding(api):
    g = api.gripper()
    return (g["effort"] >= 2.5 and 0.003 <= g["width_m"] <= 0.025), g


def run(api):
    api.log("instr=%s" % api.instruction())
    sc = Scene(api)
    api.log("table_z=%.4f home=%s" % (sc.table_z, np.round(sc.home, 3).tolist()))
    cl = sc.clusters(api, "t0")

    bowls = [d for d in cl if 150 <= d["area"] <= 4000
             and d["ex"] < 0.17 and d["ey"] < 0.17]
    if not bowls:
        api.log("NO BOWL")
        return
    tgt = max(bowls, key=lambda d: d["ztop"])
    cx, cy, r, zt, npx = sc.ring(tgt["cx"], tgt["cy"], tgt["ztop"])
    sup = sc.support_z(cx, cy, zt)
    api.log("TGT c=(%.4f,%.4f) r=%.4f zt=%.4f npx=%d support=%.4f"
            % (cx, cy, r, zt, npx, sup))

    # plate: wide, low, bright
    pl = [d for d in cl if d["ztop"] < sc.table_z + 0.045
          and max(d["ex"], d["ey"]) > 0.11]
    if pl:
        p = max(pl, key=lambda d: d["area"])
        plate = np.array([p["cx"], p["cy"]])
        plate_z = p["ztop"]
        api.log("PLATE c=(%.4f,%.4f) z=%.4f a=%d e=(%.3f,%.3f)"
                % (p["cx"], p["cy"], plate_z, p["area"], p["ex"], p["ey"]))
    else:
        plate = PLACE_XY_FALLBACK.copy()
        plate_z = sc.table_z + 0.018
        api.log("PLATE fallback")

    R = np.asarray(api.tool_rotation(), float)
    ax = np.array([R[0, 0], R[1, 0], 0.0])
    ay = np.array([R[0, 1], R[1, 1], 0.0])
    ax = ax / max(np.linalg.norm(ax), 1e-6)
    ay = ay / max(np.linalg.norm(ay), 1e-6)
    api.log("jaw candidates ax=%s ay=%s" % (np.round(ax, 3).tolist(),
                                            np.round(ay, 3).tolist()))

    ladder = [(ay, +1.0), (ay, -1.0), (ax, +1.0), (ax, -1.0)]
    held = False
    grip_off = np.zeros(2)     # gripper xy minus bowl-centre xy at the grasp
    for i, (a, s) in enumerate(ladder):
        if i:
            # a failed attempt nudges the bowl, so the ring model is stale:
            # park the arm and re-measure before aiming again.
            api.move(sc.home, seconds=2.0)
            sc = Scene(api)
            cl2 = sc.clusters(api, "r%d" % i)
            cand = [d for d in cl2 if 150 <= d["area"] <= 4000
                    and d["ex"] < 0.17 and d["ey"] < 0.17
                    and abs(d["cx"] - cx) < 0.16 and abs(d["cy"] - cy) < 0.16]
            if cand:
                t2 = max(cand, key=lambda d: d["ztop"])
                cx, cy, r, zt, npx = sc.ring(t2["cx"], t2["cy"], t2["ztop"])
                api.log("RETGT c=(%.4f,%.4f) r=%.4f zt=%.4f npx=%d"
                        % (cx, cy, r, zt, npx))
        safe_z = zt + 0.13
        gp = np.array([cx + s * r * a[0], cy + s * r * a[1], zt - GRASP_DEPTH])
        api.grip(OPEN_W)
        api.move([gp[0], gp[1], safe_z], seconds=1.8)
        res = api.move(gp, seconds=1.6)
        api.grip(0.0)
        api.settle(0.35)
        ok_, g = holding(api)
        api.log("try%d gp=%s res=%.4f w=%.4f eff=%.2f ok=%s"
                % (i, np.round(gp, 4).tolist(), res, g["width_m"],
                   g["effort"], ok_))
        if ok_:
            api.move([gp[0], gp[1], zt + 0.06], seconds=1.4)
            ok2, g2 = holding(api)
            api.log("lift%d w=%.4f eff=%.2f ok=%s"
                    % (i, g2["width_m"], g2["effort"], ok2))
            if ok2:
                held = True
                grip_off = np.array([s * r * a[0], s * r * a[1]])
                break
        api.grip(OPEN_W)
        api.move([gp[0], gp[1], safe_z], seconds=1.4)
    api.log("HELD=%s" % held)
    if not held:
        return

    # the jaws hold the RIM, so the bowl centre trails the gripper by
    # grip_off; aim the gripper so the bowl centre lands on the plate centre.
    drop = plate + grip_off
    api.log("drop=%s grip_off=%s" % (np.round(drop, 4).tolist(),
                                     np.round(grip_off, 4).tolist()))
    e = np.asarray(api.eef(), float)
    api.move([e[0], e[1], zt + 0.16], seconds=1.8)
    api.move([drop[0], drop[1], zt + 0.16], seconds=3.0)
    rel = plate_z + RELEASE_ABOVE_PLATE
    api.move([drop[0], drop[1], rel + 0.08], seconds=1.8)
    res = api.move([drop[0], drop[1], rel], seconds=1.8)
    ok3, g3 = holding(api)
    api.log("at plate res=%.4f w=%.4f eff=%.2f eef=%s"
            % (res, g3["width_m"], g3["effort"],
               np.round(api.eef(), 4).tolist()))
    api.grip(OPEN_W)
    api.settle(0.6)
    api.move([drop[0], drop[1], rel + 0.14], seconds=1.8)
    api.settle(0.4)
