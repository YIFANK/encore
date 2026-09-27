"""v7: v5 plus a straight-down wrist re-centring before each grasp and
up to three guarded attempts, so the pick survives a biased cam_high
estimate (v5 loses the can to a 10 mm aim error). Identical to v6 but for
the removal of one unused leftover constant.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "R_DOWN": {"source": "generic controller mechanics: straight-down tool "
                         "frame (api.tool_rotation() at reset is this, up to "
                         "a 3 deg tilt)", "allowed": True},
    "TABLE_Z": {"source": "debug seeds 51/53/57/61 cam_high depth: the flat "
                          "support plane deprojects to z = 0.001 m",
                "allowed": True},
    "WORKSPACE": {"source": "debug-seed cam_high cloud extent; crop that keeps "
                            "only the table top", "allowed": True},
    "Z_OBJ_FLOOR": {"source": "debug-seed height map: 0.055 m is above the "
                              "table noise and below every prop top (0.081 m "
                              "for the short cans)", "allowed": True},
    "SHORT_BAND": {"source": "debug seeds 51/53/57/61: the two can-shaped "
                             "props top out at 0.081 m; every other prop tops "
                             "at 0.140-0.148 m or is flat (0.020 m)",
                   "allowed": True},
    "RED_CUE": {"source": "debug-seed cam_high RGB body band (z 0.02-0.07): "
                          "red can mean(r-b) = +0.190, blue can = -0.122",
                "allowed": True},
    "FINGER_OFFSET": {"source": "v4 closing staircase over the perceived "
                                "can (top 0.081 m): closes at eef z 0.091 and "
                                "above read air (width 0.001), eef z 0.079 read "
                                "0.062 m at effort 3.0, so the fingertips sit "
                                "~0.006 m below the eef", "allowed": True},
    "HOVER_Z": {"source": "debug seeds: a straight-down wrist view from "
                          "0.30 m clears every prop (tallest 0.148 m)",
                "allowed": True},
    "REFINE_R": {"source": "debug seeds: nearest other prop is 0.10 m from "
                           "the can, so a 0.07 m gate cannot pick up a "
                           "neighbour", "allowed": True},
    "MAX_TRIES": {"source": "own episode budget", "allowed": True},
    "GRASP_FINGERTIP_Z": {"source": "mid-body of the perceived can, whose "
                                    "top is 0.081 m over a table plane at 0",
                          "allowed": True},
    "HOLD_W": {"source": "v2: closing on air reads width 0.001 m; the "
                         "perceived can is 0.061 m across", "allowed": True},
    "CARRY_Z": {"source": "debug-seed height map: basket rim tops at 0.143 m; "
                          "carry the can bottom above it", "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
TABLE_Z = 0.0
WORKSPACE = (-0.45, 0.45, -0.45, 0.51)
Z_OBJ_FLOOR = 0.055
SHORT_BAND = (0.055, 0.105)
FINGER_OFFSET = 0.006
HOLD_W = 0.020
CARRY_Z = 0.30


def _dump(api, tag, arr):
    b64 = base64.b64encode(zlib.compress(np.ascontiguousarray(arr).tobytes(), 6)).decode()
    api.log(f"DUMP {tag} shape={list(arr.shape)} dtype={arr.dtype} "
            f"nchunk={(len(b64) + 1799) // 1800}")
    for i in range(0, len(b64), 1800):
        api.log(f"D {tag} {i // 1800} {b64[i:i + 1800]}")


def _cloud(frame):
    d = np.asarray(frame.depth, float)
    d = np.where(np.isfinite(d), d, 0.0)
    K, T = np.asarray(frame.intrinsics, float), np.asarray(frame.t_base_cam, float)
    h, w = d.shape
    v, u = np.mgrid[0:h, 0:w]
    p = np.stack([(u - K[0, 2]) * d / K[0, 0], (v - K[1, 2]) * d / K[1, 1], d], -1)
    return p @ T[:3, :3].T + T[:3, 3], d


def _components(Pw, valid, zlo, zhi=0.25, min_px=150):
    """Label the height-gated mask with a 4-neighbour flood fill (no scipy
    dependency assumptions)."""
    X, Y, Z = Pw[..., 0], Pw[..., 1], Pw[..., 2]
    x0, x1, y0, y1 = WORKSPACE
    m = (valid & (X > x0) & (X < x1) & (Y > y0) & (Y < y1) & (Z > zlo) & (Z < zhi))
    lab = np.zeros(m.shape, np.int32)
    cur = 0
    idx = np.argwhere(m)
    seen = np.zeros(m.shape, bool)
    h, w = m.shape
    for r0, c0 in idx:
        if seen[r0, c0]:
            continue
        cur += 1
        stack = [(r0, c0)]
        seen[r0, c0] = True
        while stack:
            r, c = stack.pop()
            lab[r, c] = cur
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and m[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    stack.append((rr, cc))
    out = []
    for i in range(1, cur + 1):
        mi = lab == i
        n = int(mi.sum())
        if n < min_px:
            continue
        out.append({"mask": mi, "n": n, "top": float(Z[mi].max()),
                    "x": float(X[mi].mean()), "y": float(Y[mi].mean())})
    return out


def _slab_box(Pw, mask, zlo, zhi):
    X, Y, Z = Pw[..., 0], Pw[..., 1], Pw[..., 2]
    m = mask & (Z > zlo) & (Z < zhi)
    if m.sum() < 10:
        return None
    return {"cx": float((X[m].min() + X[m].max()) / 2),
            "cy": float((Y[m].min() + Y[m].max()) / 2),
            "dx": float(X[m].max() - X[m].min()),
            "dy": float(Y[m].max() - Y[m].min()), "n": int(m.sum())}


def perceive(api):
    f = api.capture("cam_high")
    Pw, d = _cloud(f)
    rgb = np.asarray(f.rgb, float) / 255.0
    Z = Pw[..., 2]
    comps = _components(Pw, d > 0, Z_OBJ_FLOOR)
    for c in comps:
        body = c["mask"] & (Z > 0.02) & (Z < 0.07)
        use = body if body.sum() > 40 else c["mask"]
        col = rgb[use]
        c["rb"] = float(np.mean(col[:, 0] - col[:, 2]))
        c["box_top"] = _slab_box(Pw, c["mask"], c["top"] - 0.015, c["top"] + 0.01)
        api.log(f"COMP xy=({c['x']:+.3f},{c['y']:+.3f}) top={c['top']:.3f} "
                f"n={c['n']} rb={c['rb']:+.3f} box={c['box_top']}")
    return f, Pw, rgb, comps



GRASP_FINGERTIP_Z = 0.044
HOVER_Z = 0.30
REFINE_R = 0.07
MAX_TRIES = 3


def select_target(comps):
    """The tomato sauce: the short (can-height) prop with the reddest body."""
    shorts = [c for c in comps if SHORT_BAND[0] < c["top"] < SHORT_BAND[1]]
    return max(shorts, key=lambda c: c["rb"]) if shorts else None


def wrist_refine(api, x, y, expect_top):
    """Re-centre the target from a straight-down wrist view at HOVER_Z.

    cam_high sees the table at a slant, so its bbox centre carries a bias that
    grows with the prop's height; the wrist view from directly above does not.
    Returns (x, y, top) or the input unchanged when nothing plausible is found.
    """
    api.move([x, y, HOVER_Z], rotation=R_DOWN, seconds=2.0)
    api.settle(0.3)
    f = api.capture("cam_arm_wrist")
    Pw, d = _cloud(f)
    best, bestd = None, REFINE_R
    for c in _components(Pw, d > 0, Z_OBJ_FLOOR, zhi=0.25, min_px=80):
        dist = float(np.hypot(c["x"] - x, c["y"] - y))
        if dist < bestd:
            best, bestd = c, dist
    if best is None:
        api.log(f"REFINE none within {REFINE_R:.3f} of ({x:+.3f},{y:+.3f})")
        return x, y, expect_top
    box = _slab_box(Pw, best["mask"], best["top"] - 0.015, best["top"] + 0.01)
    if box is None or not (0.03 < box["dy"] < 0.10 and 0.03 < box["dx"] < 0.10):
        api.log(f"REFINE rejected box={box}")
        return x, y, expect_top
    api.log(f"REFINE ({x:+.3f},{y:+.3f}) -> ({box['cx']:+.3f},{box['cy']:+.3f}) "
            f"d={bestd:.4f} top={best['top']:.3f} dx={box['dx']:.3f} "
            f"dy={box['dy']:.3f}")
    return box["cx"], box["cy"], best["top"]


def try_pick(api, x, y, top, tag):
    """One guarded top-down grasp. -> (held, width)."""
    gz = GRASP_FINGERTIP_Z + FINGER_OFFSET
    api.grip(0.08)
    api.settle(0.3)
    api.move([x, y, gz + 0.08], rotation=R_DOWN, seconds=2.0)
    r = api.move([x, y, gz], rotation=R_DOWN, seconds=2.0)
    api.log(f"AT_GRASP[{tag}] xy=({x:+.3f},{y:+.3f}) gz={gz:.3f} res={r:.4f} "
            f"eef={np.round(api.eef(), 4).tolist()}")
    api.grip(0.0)
    api.settle(0.5)
    g = api.gripper()
    api.log(f"CLOSED[{tag}] w={g['width_m']:.4f} eff={g['effort']:.2f}")
    api.move([x, y, HOVER_Z], rotation=R_DOWN, seconds=2.0)
    api.settle(0.3)
    g = api.gripper()
    held = g["width_m"] > HOLD_W and g["effort"] > 1.0
    api.log(f"LIFTED[{tag}] w={g['width_m']:.4f} eff={g['effort']:.2f} "
            f"held={held}")
    return held, g["width_m"]


def deliver(api, bt, tag):
    api.move([bt["cx"], bt["cy"], CARRY_Z], rotation=R_DOWN, seconds=3.0)
    api.settle(0.3)
    g = api.gripper()
    api.log(f"OVER_BASKET[{tag}] w={g['width_m']:.4f} eff={g['effort']:.2f} "
            f"eef={np.round(api.eef(), 4).tolist()}")
    api.grip(0.08)
    api.settle(1.0)
    api.log(f"RELEASED[{tag}] w={api.gripper()['width_m']:.4f}")
    api.move([bt["cx"], bt["cy"] - 0.10, CARRY_Z], rotation=R_DOWN, seconds=2.0)


def run(api):
    api.log(f"INSTR {api.instruction()!r}")
    f, Pw, rgb, comps = perceive(api)
    basket = max(comps, key=lambda c: c["n"])
    bt = basket["box_top"]
    api.log(f"BASKET xy=({bt['cx']:+.3f},{bt['cy']:+.3f}) top={basket['top']:.3f}")

    tgt = select_target(comps)
    if tgt is None:
        api.log("NO_TARGET")
        return
    b = tgt["box_top"]
    x, y, top = b["cx"], b["cy"], tgt["top"]
    api.log(f"TARGET xy=({x:+.3f},{y:+.3f}) top={top:.3f} dx={b['dx']:.3f} "
            f"dy={b['dy']:.3f} rb={tgt['rb']:+.3f}")

    for attempt in range(MAX_TRIES):
        x, y, top = wrist_refine(api, x, y, top)
        held, w = try_pick(api, x, y, top, f"t{attempt}")
        if held:
            deliver(api, bt, f"t{attempt}")
            # did it actually leave the gripper over the basket?
            api.settle(0.5)
            break
        api.grip(0.08)
        api.settle(0.3)
        api.move([x, y, HOVER_Z], rotation=R_DOWN, seconds=2.0)
        # a failed close may have nudged the can: re-perceive from cam_high
        api.move([x, y - 0.12, HOVER_Z], rotation=R_DOWN, seconds=2.0)
        api.settle(0.5)
        f2 = api.capture("cam_high")
        Pw2, d2 = _cloud(f2)
        rgb2 = np.asarray(f2.rgb, float) / 255.0
        Z2 = Pw2[..., 2]
        comps2 = _components(Pw2, d2 > 0, Z_OBJ_FLOOR)
        for c in comps2:
            body = c["mask"] & (Z2 > 0.02) & (Z2 < 0.07)
            use = body if body.sum() > 40 else c["mask"]
            col = rgb2[use]
            c["rb"] = float(np.mean(col[:, 0] - col[:, 2]))
            c["box_top"] = _slab_box(Pw2, c["mask"], c["top"] - 0.015,
                                     c["top"] + 0.01)
        t2 = select_target(comps2)
        if t2 is None or t2["box_top"] is None:
            api.log("RETRY lost the target")
            break
        x, y, top = t2["box_top"]["cx"], t2["box_top"]["cy"], t2["top"]
        api.log(f"RETRY next aim ({x:+.3f},{y:+.3f}) top={top:.3f}")

    api.move([-0.05, -0.35, CARRY_Z], rotation=R_DOWN, seconds=2.0)
    api.settle(0.5)
    f3 = api.capture("cam_high")
    Pw3, d3 = _cloud(f3)
    for c in _components(Pw3, d3 > 0, Z_OBJ_FLOOR):
        api.log(f"POST xy=({c['x']:+.3f},{c['y']:+.3f}) top={c['top']:.3f} "
                f"n={c['n']}")
    _dump(api, "final_rgb", f3.rgb[::2, ::2])
    api.log("DONE v7")
