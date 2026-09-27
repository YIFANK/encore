"""c2k1clean obj_butter_task_k1 -- v5: clear the basket rim in transit.

v4 centred the release on the rim bbox and still left the carton standing
against the basket's near wall.  Cause: the carton hangs 0.126 m below the eef
(grasp eef z 0.127 with the carton bottom on the table at 0.001), so at the v4
transport height of 0.26 its base was at 0.134 -- BELOW the 0.144 rim.  The
carton was driven into the near wall instead of over it.  v5 carries at 0.30
(base 0.174, clear of the rim) and only then descends inside.

Intent: "Pick the orange juice and place it in the basket".

Perception (v1/v2 probes on debug seeds 51,53,55,57):
  table top z = 0.0013; seven props above it.  The orange-juice carton is the
  only tall (top ~0.143) prop with a large saturated-orange pixel fraction
  (0.44 vs <=0.14 for every other tall prop) -- matching the carton visible in
  BOTH pack keyframe sets (identified in the mate pack by differencing its
  t0000 / t0116 frames: the removed object is an orange-labelled carton).
  The basket is the largest component, rim top 0.144, ~0.16 m across.

Grasp height: the mate pack (which handles the object this intent names)
closes its gripper at eef z = 0.120 while that carton's top is at 0.143 in
this scene -> grasp ~0.023 below the carton top.  Cross-check: the k1 pack
(same scene, flat butter box, top 0.019) closes at z = 0.0095, i.e. the eef z
sits at the mid-height of what it grips, so the eef frame is at fingertip
mid-span.

Release: both packs open the gripper over the basket at z ~ 0.17-0.19 with the
eef near the basket centre.
"""

import numpy as np

PROVENANCE = {
    "TABLE_Z_FALLBACK": {
        "source": "debug seeds 51/53/55/57 probe v2: median workspace point z = 0.0013",
        "allowed": True},
    "BAND_LO": {"source": "debug probe v2: 0.015 above table separates props from table noise",
                "allowed": True},
    "BAND_HI": {"source": "debug probe v2: tallest prop top = 0.148; robot arm at home "
                          "starts above 0.25", "allowed": True},
    "CELL": {"source": "debug probe v2: 0.012 m xy grid kept the 7 props separate",
             "allowed": True},
    "ORANGE_HSV": {"source": "mate-pack keyframe t0000 crop of the carton removed by t0116 "
                             "(hue 0.05-0.13, sat>0.55) confirmed on debug seeds: the tall "
                             "orange-fraction gap is 0.44 vs <=0.14", "allowed": True},
    "MIN_TALL": {"source": "debug probe v2: carton top 0.143, flat boxes top 0.019-0.029",
                 "allowed": True},
    "GRASP_DROP": {"source": "mate pack keyframe t=42 eef z=0.120 vs carton top 0.143 "
                             "measured on debug seeds", "allowed": True},
    "HOVER": {"source": "k1 pack ee_path6 idx3->5: approach descends from ~0.24 to the "
                        "grasp height straight down", "allowed": True},
    "CARRY_Z": {"source": "mate pack ee_path6 transport altitude 0.29; raised to 0.30 so "
                          "the measured 0.126 m carton hang clears the measured 0.144 m "
                          "basket rim (v4 debug run collided)", "allowed": True},
    "HANG": {"source": "debug seeds 51-65: grasp eef z 0.127 with the carton resting on the "
                       "0.0013 table -> the payload hangs 0.126 m below the eef",
             "allowed": True},
    "RELEASE_Z": {"source": "k1 pack keyframe t=150 release z=0.191; mate pack t=116 z=0.168; "
                            "v3 debug run dropped the carton beside the basket, so this "
                            "version uses the lower end of the two pack releases",
                  "allowed": True},
    "RIM_BAND": {"source": "debug probe v2/v3: basket rim top z=0.144; the point-cloud "
                           "centroid of the basket is pulled onto the camera-facing walls "
                           "(v3 released outside), so the centre is taken as the midpoint "
                           "of the rim-band bbox", "allowed": True},
    "OPEN_W": {"source": "debug-seed observation: api.gripper() reports width 0.0778 at reset",
               "allowed": True},
    "HOLD_EFFORT": {"source": "FairApi docstring: effort 3.0 iff holding", "allowed": True},
}

BAND_LO, BAND_HI = 0.015, 0.25
CELL = 0.012
MIN_TALL = 0.07
GRASP_DROP = 0.023
HOVER = 0.10
CARRY_Z = 0.30
RELEASE_Z = 0.20
OPEN_W = 0.08


def _hsv(arr):
    a = arr.astype(np.float32) / 255.0
    mx = a.max(2)
    d = mx - a.min(2)
    h = np.zeros_like(mx)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    m = d > 1e-6
    i = (mx == r) & m
    h[i] = ((g[i] - b[i]) / d[i]) % 6
    i = (mx == g) & m
    h[i] = (b[i] - r[i]) / d[i] + 2
    i = (mx == b) & m
    h[i] = (r[i] - g[i]) / d[i] + 4
    return h / 6.0, np.where(mx > 1e-6, d / np.maximum(mx, 1e-6), 0.0), mx


def _cloud(f):
    dep = np.asarray(f.depth, float)
    H, W = dep.shape[:2]
    vs, us = np.mgrid[0:H, 0:W]
    us, vs = us.ravel(), vs.ravel()
    z = dep[vs, us]
    ok = np.isfinite(z) & (z > 0)
    us, vs, z = us[ok], vs[ok], z[ok]
    K = np.asarray(f.intrinsics, float)
    pc = np.stack([(us - K[0, 2]) * z / K[0, 0],
                   (vs - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], 1)
    return (np.asarray(f.t_base_cam, float) @ pc.T).T[:, :3], us, vs


def _components(P, cell=CELL, gap=1):
    ix = np.floor(P[:, 0] / cell).astype(int)
    iy = np.floor(P[:, 1] / cell).astype(int)
    ix -= ix.min()
    iy -= iy.min()
    nx, ny = ix.max() + 1, iy.max() + 1
    occ = np.zeros((nx, ny), bool)
    occ[ix, iy] = True
    lab = np.zeros((nx, ny), int)
    cur = 0
    for sx in range(nx):
        for sy in range(ny):
            if not occ[sx, sy] or lab[sx, sy]:
                continue
            cur += 1
            lab[sx, sy] = cur
            stack = [(sx, sy)]
            while stack:
                x, y = stack.pop()
                for dx in range(-gap, gap + 1):
                    for dy in range(-gap, gap + 1):
                        a, b = x + dx, y + dy
                        if 0 <= a < nx and 0 <= b < ny and occ[a, b] and not lab[a, b]:
                            lab[a, b] = cur
                            stack.append((a, b))
    m = lab[ix, iy]
    return [np.nonzero(m == c)[0] for c in range(1, cur + 1)]


def perceive(api):
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb)
    hs, ss, vv = _hsv(rgb)
    P, us, vs = _cloud(f)
    ws = ((P[:, 0] > -0.28) & (P[:, 0] < 0.32) &
          (P[:, 1] > -0.40) & (P[:, 1] < 0.45))
    zt = float(np.median(P[ws, 2]))
    band = ws & (P[:, 2] > zt + BAND_LO) & (P[:, 2] < zt + BAND_HI)
    Pb, ub, vb = P[band], us[band], vs[band]
    comps = [c for c in _components(Pb) if len(c) >= 150]
    comps.sort(key=lambda c: -len(c))
    out = []
    for c in comps:
        p = Pb[c]
        uu, vx = ub[c], vb[c]
        h, s, v = hs[vx, uu], ss[vx, uu], vv[vx, uu]
        orange = float((((h > 0.02) & (h < 0.13) & (s > 0.45) & (v > 0.30))).mean())
        ztop = float(p[:, 2].max())
        cap = p[p[:, 2] > ztop - 0.02]
        rim = p[p[:, 2] > ztop - 0.012]
        out.append({"n": len(c), "xy": (float(p[:, 0].mean()), float(p[:, 1].mean())),
                    "top_xy": (float(np.median(cap[:, 0])), float(np.median(cap[:, 1]))),
                    "rim_mid": (float(0.5 * (rim[:, 0].min() + rim[:, 0].max())),
                                float(0.5 * (rim[:, 1].min() + rim[:, 1].max()))),
                    "box_mid": (float(0.5 * (p[:, 0].min() + p[:, 0].max())),
                                float(0.5 * (p[:, 1].min() + p[:, 1].max()))),
                    "ztop": ztop, "orange": orange,
                    "span": (float(np.ptp(p[:, 0])), float(np.ptp(p[:, 1]))),
                    "px": (int(uu.mean()), int(vx.mean()))})
    return zt, out


def run(api):
    api.log("v5 start instr=%r" % api.instruction())
    zt, objs = perceive(api)
    api.log("table z=%.4f objs=%d" % (zt, len(objs)))
    for i, o in enumerate(objs):
        api.log("o%d n=%d xy=(%+.3f,%+.3f) top_xy=(%+.3f,%+.3f) ztop=%.3f "
                "span=(%.3f,%.3f) orng=%.2f px=%s"
                % (i, o["n"], o["xy"][0], o["xy"][1], o["top_xy"][0], o["top_xy"][1],
                   o["ztop"], o["span"][0], o["span"][1], o["orange"], o["px"]))

    basket = max(objs, key=lambda o: o["n"])
    tall = [o for o in objs if o is not basket and o["ztop"] - zt > MIN_TALL]
    if not tall:
        api.log("no tall prop found; abort")
        return
    target = max(tall, key=lambda o: o["orange"])
    api.log("target px=%s ztop=%.3f orng=%.2f" % (target["px"], target["ztop"], target["orange"]))
    api.log("basket cen=(%+.3f,%+.3f) rim=(%+.3f,%+.3f) box=(%+.3f,%+.3f) ztop=%.3f"
            % (basket["xy"][0], basket["xy"][1], basket["rim_mid"][0], basket["rim_mid"][1],
               basket["box_mid"][0], basket["box_mid"][1], basket["ztop"]))

    tx, ty = target["top_xy"]
    gz = target["ztop"] - GRASP_DROP

    api.grip(OPEN_W)
    api.move([tx, ty, target["ztop"] + HOVER], seconds=2.5)
    api.log("hover residual eef=%s" % np.round(api.eef(), 4).tolist())
    r = api.move([tx, ty, gz], seconds=2.0)
    api.log("descend r=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("closed %s" % g)

    r = api.move([tx, ty, CARRY_Z], seconds=2.5)
    g = api.gripper()
    api.log("lifted %s eef=%s" % (g, np.round(api.eef(), 4).tolist()))

    bx, by = basket["rim_mid"]
    r = api.move([bx, by, CARRY_Z], seconds=3.0)
    api.log("over basket r=%.4f eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    r = api.move([bx, by, CARRY_Z], seconds=1.5)
    api.log("settle over basket r=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    r = api.move([bx, by, RELEASE_Z], seconds=1.5)
    api.log("lowered r=%.4f eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.5)
    api.move([bx, by, CARRY_Z], seconds=1.5)
    zt2, objs2 = perceive(api)
    for o in objs2:
        api.log("post n=%d xy=(%+.3f,%+.3f) ztop=%.3f orng=%.2f px=%s"
                % (o["n"], o["xy"][0], o["xy"][1], o["ztop"], o["orange"], o["px"]))
    api.log("v5 done eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
