"""v5 -- v4 plus a sensor check on the lift and one re-perceive/retry.

v4 scored 4/4 on the probe subset; the retry is insurance for the blind band,
not a fix for an observed failure.

v4 -- v3 with the identity cue fixed.

v3 picked the wrong can: it scored colour over the depth-cluster pixels, and
the cluster had lost roughly half of each can to the arm-cell mask (a cell that
holds any point above 0.20 m is dropped, and the robot's own links hang over
both cans in the home pose).  What survived was mostly the grey lid, so the two
cans scored rb = -1.0 and -0.4 -- a coin flip.  Receipt: after the arm left,
the same can re-measured at n=1350 (vs 615) and rb = +17.6 (v3 ep51 Q5).

Fix: score colour over a HEIGHT WINDOW around the candidate (table+10mm .. top),
not over the arm-masked cluster, and take the whole can body, lid included.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "WS_X/WS_Y": {"source": "debug-seed v1: bounds enclosing every deprojected "
                            "table point", "allowed": True},
    "TABLE_BAND": {"source": "generic depth mechanics", "allowed": True},
    "ARM_Z": {"source": "debug-seed v1/v2: only the robot column exceeded "
                        "0.15 m (top 0.477); every prop topped out at 0.148",
              "allowed": True},
    "CAN_TOP": {"source": "mate pack closing EEF z 0.045-0.058 + debug-seed v2: "
                          "the matching short-can class tops at 0.081 m",
                "allowed": True},
    "COL_R": {"source": "debug-seed v2: can footprints measure 0.037-0.070 m "
                        "across, so a 0.045 m half-window covers one can",
              "allowed": True},
    "GRASP_DROP": {"source": "mate pack: closing EEF sits 0.023-0.036 m below "
                             "the can top (0.081)", "allowed": True},
    "CARRY_Z": {"source": "both packs: transfer leg runs at EEF z 0.21-0.31",
                "allowed": True},
    "RELEASE_Z": {"source": "both packs: gripper opens over the basket at EEF "
                            "z 0.16-0.22", "allowed": True},
    "HOLD_W": {"source": "debug-seed v3/v4: a held can reads width 0.0616-0.0625 with effort 3.0, empty jaws read ~0", "allowed": True},
    "OPEN_W/CLOSE_W": {"source": "generic gripper mechanics; open width reads "
                                 "0.0778 m on debug seeds", "allowed": True},
}

WS_X = (-0.40, 0.45)
WS_Y = (-0.45, 0.50)
TABLE_BAND = 0.012
ARM_Z = 0.20
CELL = 0.012
CAN_TOP = (0.05, 0.11)
COL_R = 0.045
GRASP_DROP = 0.030
CARRY_Z = 0.25
RELEASE_Z = 0.20
OPEN_W = 0.08
CLOSE_W = 0.0
HOLD_W = 0.035

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def scene(api, cam="cam_high"):
    f = api.capture(cam)
    H, W = f.depth.shape
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    vs, us = np.mgrid[0:H, 0:W]
    z = f.depth.astype(float)
    ok = np.isfinite(z) & (z > 0)
    xc = (us - K[0, 2]) * z / K[0, 0]
    yc = (vs - K[1, 2]) * z / K[1, 1]
    P = np.stack([xc, yc, z, np.ones_like(z)], axis=-1) @ T.T
    return f, P[..., :3], ok, us, vs


def survey(api):
    f, pts, ok, us, vs = scene(api)
    x, y, z = pts[..., 0], pts[..., 1], pts[..., 2]
    inws = ok & (x > WS_X[0]) & (x < WS_X[1]) & (y > WS_Y[0]) & (y < WS_Y[1])
    hist, edges = np.histogram(z[inws], bins=200)
    table_z = float(edges[int(np.argmax(hist))])
    m = inws & (z > table_z + TABLE_BAND)
    gx = np.round(x / CELL).astype(int)
    gy = np.round(y / CELL).astype(int)
    tall = {}
    for a, b, zz in zip(gx[m], gy[m], z[m]):
        k = (int(a), int(b))
        if zz > tall.get(k, -9.0):
            tall[k] = float(zz)
    keys = {}
    for i, (a, b) in enumerate(zip(gx[m], gy[m])):
        k = (int(a), int(b))
        if tall[k] > ARM_Z:
            continue
        keys.setdefault(k, []).append(i)
    U, V, X, Y, Z = us[m], vs[m], x[m], y[m], z[m]
    seen, out = set(), []
    for k in list(keys):
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            a, b = stack.pop()
            comp.extend(keys[(a, b)])
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    nk = (a + da, b + db)
                    if nk in keys and nk not in seen:
                        seen.add(nk)
                        stack.append(nk)
        idx = np.asarray(comp)
        if idx.size < 40:
            continue
        cz = Z[idx]
        top = float(np.percentile(cz, 98))
        lid = idx[cz > top - 0.010]
        out.append({
            "n": int(idx.size),
            "xy": [float(X[idx].mean()), float(Y[idx].mean())],
            "lid_xy": [float((X[lid].min() + X[lid].max()) / 2),
                       float((Y[lid].min() + Y[lid].max()) / 2)],
            "top": top,
            "ext": [float(X[idx].max() - X[idx].min()),
                    float(Y[idx].max() - Y[idx].min())],
        })
    out.sort(key=lambda c: -c["n"])
    return out, table_z, (f, pts, ok, us, vs)


def colour(raw, table_z, cx, cy, top):
    """Colour statistics over a height window around (cx, cy) -- immune to the
    arm-cell mask, and bounded above by the object's own top so the robot
    hanging over it contributes nothing."""
    f, pts, ok, us, vs = raw
    x, y, z = pts[..., 0], pts[..., 1], pts[..., 2]
    m = (ok & (np.abs(x - cx) < COL_R) & (np.abs(y - cy) < COL_R)
         & (z > table_z + 0.010) & (z < top + 0.010))
    col = f.rgb[m].astype(float)
    if col.shape[0] < 20:
        return None
    rb = col[:, 0] - col[:, 2]
    return {"n": int(col.shape[0]), "mean": [round(float(v), 1) for v in col.mean(0)],
            "rb": round(float(rb.mean()), 2),
            "rb75": round(float(np.percentile(rb, 75)), 2),
            "warm_frac": round(float((rb > 15).mean()), 3)}


def crop_log(api, raw, tag, cx, cy, top, table_z):
    f, pts, ok, us, vs = raw
    x, y, z = pts[..., 0], pts[..., 1], pts[..., 2]
    m = (ok & (np.abs(x - cx) < COL_R) & (np.abs(y - cy) < COL_R)
         & (z > table_z + 0.010) & (z < top + 0.010))
    if not m.any():
        return
    u0, u1 = int(us[m].min()), int(us[m].max())
    v0, v1 = int(vs[m].min()), int(vs[m].max())
    sub = np.ascontiguousarray(f.rgb[v0:v1 + 1, u0:u1 + 1])
    b = base64.b64encode(zlib.compress(sub.tobytes(), 9)).decode()
    api.log(f"CROP {tag} shape={sub.shape} chunks={(len(b) + 1799) // 1800}")
    for i in range(0, len(b), 1800):
        api.log(f"CROPDATA {tag} {i // 1800} {b[i:i + 1800]}")


def run(api):
    cl, table_z, raw = survey(api)
    api.log(f"TABLE_Z {table_z:.4f} NPROP {len(cl)}")
    for i, c in enumerate(cl):
        api.log(f"P{i} n={c['n']} xy={[round(v, 4) for v in c['xy']]} "
                f"lid={[round(v, 4) for v in c['lid_xy']]} top={c['top']:.4f} "
                f"ext={[round(v, 4) for v in c['ext']]}")

    basket = max(cl, key=lambda c: c["n"])
    cans = [c for c in cl if CAN_TOP[0] < c["top"] < CAN_TOP[1]]
    for i, c in enumerate(cans):
        c["col"] = colour(raw, table_z, c["lid_xy"][0], c["lid_xy"][1], c["top"])
        api.log(f"CAN{i} lid={[round(v, 4) for v in c['lid_xy']]} col={c['col']}")
        crop_log(api, raw, f"can{i}", c["lid_xy"][0], c["lid_xy"][1], c["top"], table_z)
    if not cans:
        return "no can-class prop found"

    tgt = max(cans, key=lambda c: (c["col"] or {"rb": -99})["rb"])
    gx, gy = tgt["lid_xy"]
    api.log(f"TARGET lid_xy=({gx:.4f},{gy:.4f}) top={tgt['top']:.4f} col={tgt['col']}")
    bx, by = basket["xy"]
    api.log(f"BASKET xy=({bx:.4f},{by:.4f}) top={basket['top']:.4f}")

    top = tgt["top"]
    for attempt in range(2):
        api.grip(OPEN_W)
        api.move([gx, gy, CARRY_Z], rotation=R_DOWN, seconds=3.0)
        api.log(f"HOVER{attempt} eef={api.eef().round(4).tolist()}")
        z_goal = top - GRASP_DROP
        api.move([gx, gy, 0.13], rotation=R_DOWN, seconds=2.0)
        r = api.move([gx, gy, float(z_goal)], rotation=R_DOWN, seconds=2.0)
        api.log(f"AT_GRASP{attempt} z*={z_goal:.3f} "
                f"eef={api.eef().round(4).tolist()} res={r:.4f}")

        api.grip(CLOSE_W)
        api.settle(0.4)
        api.log(f"CLOSED{attempt} {api.gripper()}")
        api.move([gx, gy, CARRY_Z], rotation=R_DOWN, seconds=3.0)
        g = api.gripper()
        api.log(f"LIFTED{attempt} eef={api.eef().round(4).tolist()} grip={g}")
        # a held can holds the jaws apart; empty jaws close to ~0
        if g["width_m"] > HOLD_W and g["effort"] > 1.0:
            break
        if attempt == 0:
            api.log("RETRY: jaws came up empty, re-perceiving from the lifted pose")
            api.grip(OPEN_W)
            cl3, tz3, raw3 = survey(api)
            again = [c for c in cl3 if CAN_TOP[0] < c["top"] < CAN_TOP[1]]
            for c in again:
                c["col"] = colour(raw3, tz3, c["lid_xy"][0], c["lid_xy"][1], c["top"])
            again = [c for c in again if c["col"]]
            if again:
                t2 = max(again, key=lambda c: c["col"]["rb"])
                gx, gy = t2["lid_xy"]
                top = t2["top"]
                api.log(f"RETARGET lid_xy=({gx:.4f},{gy:.4f}) top={top:.4f} "
                        f"col={t2['col']}")

    api.move([bx, by, CARRY_Z], rotation=R_DOWN, seconds=3.0)
    api.move([bx, by, RELEASE_Z], rotation=R_DOWN, seconds=2.0)
    api.grip(OPEN_W)
    api.settle(0.6)
    api.log(f"RELEASED eef={api.eef().round(4).tolist()} grip={api.gripper()}")
    api.move([bx, by, CARRY_Z + 0.05], rotation=R_DOWN, seconds=2.0)

    cl2, tz2, _ = survey(api)
    for i, c in enumerate(cl2):
        api.log(f"Q{i} n={c['n']} xy={[round(v, 4) for v in c['xy']]} "
                f"top={c['top']:.4f}")
    return "v5 pick-place + grasp verification"
