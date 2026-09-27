"""c2clean obj_orange_juice_pos_k3 -- v1.

Perceive the orange-juice carton by colour on a height-gated top-down map,
perceive the basket as the largest flat-topped footprint, grasp the carton
top-down and lower it inside the basket.
"""
import json

import numpy as np

PROVENANCE = {
    "WS_X": {"source": "debug seeds 51/53/57/61 cam_high deprojection: all props "
                       "lie in base X[-0.22,0.18]; wall/floor returns fall outside",
             "allowed": True},
    "WS_Y": {"source": "debug seeds 51/53/57/61: props lie in base Y[-0.27,0.35]",
             "allowed": True},
    "Z_TABLE": {"source": "debug seed 51 depth histogram: dominant plane at "
                          "base Z=0.000-0.006, so the table top is Z=0",
                "allowed": True},
    "Z_PROP": {"source": "debug seeds: 0.018 separates the table plane from prop "
                         "pixels (thinnest prop top seen at Z=0.019)",
               "allowed": True},
    "Z_ARM": {"source": "debug seeds: the robot component reaches Z=0.420 while "
                        "every table prop tops out at Z<=0.148",
              "allowed": True},
    "ORANGE": {"source": "debug seeds 51/53/57/61 cam_high RGB: the juice carton "
                         "label is the only large blob with r/(r+g+b)>0.44 and "
                         "b/(r+g+b)<0.22 at sum>200",
               "allowed": True},
    "GRASP_DROP": {"source": "pack.json keyframes: close-frame EEF z = 0.120/0.093/"
                             "0.098 against a carton top measured at Z=0.143 on the "
                             "debug seeds -> grasp ~45mm below the carton top",
                   "allowed": True},
    "CARRY_Z": {"source": "pack.json ee_path z-max = 0.296/0.317/0.300",
                "allowed": True},
    "RELEASE_DZ": {"source": "pack.json release-keyframe EEF z = 0.168/0.147/0.146 "
                             "against a basket rim measured at Z=0.143 -> release "
                             "just above rim height",
                   "allowed": True},
    "GRIP_OPEN": {"source": "api.gripper() at episode start reports width_m=0.0778",
                  "allowed": True},
    "LOCALISE_R": {"source": "debug seeds 51-65: the carton footprint measured "
                             "27x51mm, so every carton pixel is within 45mm of the "
                             "orange-label centroid; a no-op on all 15 debug seeds "
                             "(same aim as v1) and a guard if a prop abuts it",
                   "allowed": True},
    "RETRY_DZ": {"source": "debug seeds 51-65 v1 run: commanded grasp z=0.098 landed "
                           "at 0.1076, a ~10mm undershoot, so a retry bites 15mm "
                           "lower to clear it",
                 "allowed": True},
    "HOLD_EFFORT": {"source": "debug seeds 51-65 v1 run: api.gripper() reports "
                              "effort 3.0 and width 0.0531 while the carton is held",
                    "allowed": True},
}

WS_X = (-0.32, 0.32)
WS_Y = (-0.45, 0.45)
Z_PROP = 0.018
Z_ARM = 0.35
GRASP_DROP = 0.045
CARRY_Z = 0.30
RELEASE_DZ = 0.027
GRIP_OPEN = 0.078
LOCALISE_R = 0.045
RETRY_DZ = 0.015


def _cc(mask):
    """4/8-connected labelling without scipy."""
    lab = np.zeros(mask.shape, np.int32)
    n = 0
    H, W = mask.shape
    ys, xs = np.nonzero(mask)
    for sv, su in zip(ys, xs):
        if lab[sv, su]:
            continue
        n += 1
        stack = [(sv, su)]
        lab[sv, su] = n
        while stack:
            y, x = stack.pop()
            y0, y1 = max(0, y - 1), min(H, y + 2)
            x0, x1 = max(0, x - 1), min(W, x + 2)
            sub = mask[y0:y1, x0:x1] & (lab[y0:y1, x0:x1] == 0)
            for dy, dx in zip(*np.nonzero(sub)):
                lab[y0 + dy, x0 + dx] = n
                stack.append((y0 + dy, x0 + dx))
    return lab, n


def perceive(api):
    f = api.capture("cam_high")
    rgb = f.rgb.astype(float)
    d = np.asarray(f.depth, float)
    K, T = np.asarray(f.intrinsics, float), np.asarray(f.t_base_cam, float)
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]
    z = np.where(np.isfinite(d) & (d > 0), d, 0.0)
    cam = np.stack([(uu - K[0, 2]) * z / K[0, 0],
                    (vv - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], -1)
    P = cam @ T.T
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    s = rgb.sum(-1) + 1e-6
    r, b = rgb[..., 0] / s, rgb[..., 2] / s
    ws = ((z > 0) & (X > WS_X[0]) & (X < WS_X[1])
          & (Y > WS_Y[0]) & (Y < WS_Y[1]))
    prop = ws & (Z > Z_PROP) & (Z < 0.45)
    orange = prop & (r > 0.44) & (b < 0.22) & (s > 200)
    lab, n = _cc(prop)
    comps = []
    for i in range(1, n + 1):
        m = lab == i
        if m.sum() < 40:
            continue
        ztop = float(Z[m].max())
        if ztop > Z_ARM:
            continue          # robot body
        om = orange & m
        if om.sum() >= 40:
            # Anti-fusion: a prop touching the carton would be welded into this
            # component and drag the bbox mid off the carton.  Keep only the
            # part of the component that sits near the orange label itself.
            ox, oy = float(X[om].mean()), float(Y[om].mean())
            near = m & (np.hypot(X - ox, Y - oy) < LOCALISE_R)
            if near.sum() >= 40:
                m = near
                ztop = float(Z[m].max())
        comps.append({
            "i": i, "n": int(m.sum()), "orange": int(om.sum()),
            "x0": float(X[m].min()), "x1": float(X[m].max()),
            "y0": float(Y[m].min()), "y1": float(Y[m].max()), "ztop": ztop})
    return f, comps


def _holding(api):
    g = api.gripper()
    return g["effort"] >= 3.0 and g["width_m"] > 0.010, g


def run(api):
    f, comps = perceive(api)
    for c in comps:
        api.log("comp " + json.dumps({k: (round(v, 4) if isinstance(v, float) else v)
                                      for k, v in c.items()}))
    if not comps:
        return "no components"

    tgt = max(comps, key=lambda c: c["orange"])
    rest = [c for c in comps if c is not tgt]
    basket = max(rest, key=lambda c: (c["x1"] - c["x0"]) * (c["y1"] - c["y0"]))
    gx = 0.5 * (tgt["x0"] + tgt["x1"])
    gy = 0.5 * (tgt["y0"] + tgt["y1"])
    gz = tgt["ztop"] - GRASP_DROP
    bx = 0.5 * (basket["x0"] + basket["x1"])
    by = 0.5 * (basket["y0"] + basket["y1"])
    bz = basket["ztop"] + RELEASE_DZ
    api.log("target=%s" % json.dumps([round(gx, 4), round(gy, 4), round(gz, 4)]))
    api.log("basket=%s" % json.dumps([round(bx, 4), round(by, 4), round(bz, 4)]))

    api.grip(GRIP_OPEN)
    api.move([gx, gy, CARRY_Z], seconds=2.0)
    api.log("above: eef=%s rot=%s" % (
        np.round(api.eef(), 4).tolist(),
        np.round(api.tool_rotation(), 3).tolist()))

    attempt, g2 = 0, {"width_m": 0.0, "effort": 0.0}
    while attempt < 2:
        api.move([gx, gy, gz], seconds=2.0)
        api.log("at grasp %d: eef=%s" % (attempt, np.round(api.eef(), 4).tolist()))
        api.grip(0.0)
        api.settle(0.4)
        ok, g = _holding(api)
        api.log("closed %d: %s" % (attempt, json.dumps(g)))
        api.move([gx, gy, CARRY_Z], seconds=2.0)
        ok, g2 = _holding(api)
        api.log("lifted %d: eef=%s grip=%s" % (
            attempt, np.round(api.eef(), 4).tolist(), json.dumps(g2)))
        if ok:
            break
        # empty jaws or the object slipped out: re-perceive and bite lower.
        attempt += 1
        api.grip(GRIP_OPEN)
        _, comps2 = perceive(api)
        cand = [c for c in comps2 if c["orange"] >= 40]
        if cand:
            t2 = max(cand, key=lambda c: c["orange"])
            gx = 0.5 * (t2["x0"] + t2["x1"])
            gy = 0.5 * (t2["y0"] + t2["y1"])
            gz = t2["ztop"] - GRASP_DROP - RETRY_DZ
            api.log("retry target=%s" % json.dumps(
                [round(gx, 4), round(gy, 4), round(gz, 4)]))
        else:
            gz -= RETRY_DZ

    api.move([bx, by, CARRY_Z], seconds=2.5)
    api.log("over basket: eef=%s" % np.round(api.eef(), 4).tolist())
    api.move([bx, by, bz], seconds=2.0)
    api.log("lowered: eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(),
                                         json.dumps(api.gripper())))
    api.grip(GRIP_OPEN)
    api.settle(0.6)
    api.move([bx, by, CARRY_Z], seconds=2.0)
    api.settle(0.4)
    return "v2 gz=%.3f hold=%.4f tries=%d" % (gz, g2["width_m"], attempt + 1)
