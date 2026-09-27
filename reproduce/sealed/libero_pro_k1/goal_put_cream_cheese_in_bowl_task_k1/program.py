"""c2k1clean / goal_put_cream_cheese_in_bowl_task_k1 -- "put the wine bottle in the bowl".

Mechanism
  The mate pack ("put the wine bottle on top of the cabinet") demonstrates the
  OBJECT half: it closes the gripper at eef z = 1.0223 on a straight-ish wrist,
  which on my own debug seeds is 36 mm below the bottle's top -- the neck.
  The k1 pack ("put the cream cheese in the bowl") demonstrates the TARGET half:
  it releases over the bowl from ~60 mm above the table.
  So: find the bottle by its neck (the only small component standing 90-210 mm
  above the table), pinch the neck at the pack's height, carry it over the bowl
  (the only round ring 25-90 mm above the table) and lower it until the bottle's
  base is just inside the dish, then let go.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "GRASP_Z_ABS": {
        "source": "mate pack c2k1clean_goal_put_cream_cheese_in_bowl_task_mate "
                  "ee_path6/keyframe t=32: the eef z at which gripper_cmd flips "
                  "to +1 (close) on the wine bottle = 1.0223 m",
        "allowed": True},
    "NECK_DROP_LO": {
        "source": "debug seeds 51-65 cam_high: bottle top z = 1.059 every seed, "
                  "neck (dy 0.014-0.017 m) spans z 1.005-1.059; clamp keeps the "
                  "pack height inside that measured neck band",
        "allowed": True},
    "NECK_DROP_HI": {
        "source": "same measurement as NECK_DROP_LO (neck band upper edge)",
        "allowed": True},
    "BOTTLE_BAND_LO": {
        "source": "debug seeds 51-65: bottle occupies z 0.90-1.059; the band "
                  "table+0.09 excludes the bowl (rim table+0.051) and the stove",
        "allowed": True},
    "BOTTLE_BAND_HI": {
        "source": "debug seeds 51-65: parked arm/gripper sits above z 1.11; "
                  "table+0.21 cuts it off while keeping the whole bottle neck",
        "allowed": True},
    "BOTTLE_SPAN_MAX": {
        "source": "debug seeds 51-65: bottle component span 0.017-0.026 m, while "
                  "the fixture edge in the same band spans 0.48 m",
        "allowed": True},
    "BOWL_BAND_LO": {
        "source": "debug seeds 51-65: bowl rim z = 0.951-0.952 = table+0.051, "
                  "interior floor 0.907; band table+0.025 keeps the ring, drops "
                  "the plate and the cream-cheese box",
        "allowed": True},
    "BOWL_BAND_HI": {
        "source": "debug seeds 51-65: table+0.09 sits above the bowl rim and "
                  "below the bottle shoulder",
        "allowed": True},
    "BOWL_D_LO": {
        "source": "debug seeds 51-65: bowl outer diameter 0.109-0.112 m",
        "allowed": True},
    "BOWL_D_HI": {"source": "same measurement as BOWL_D_LO", "allowed": True},
    "BOWL_ASPECT": {
        "source": "debug seeds 51-65: bowl x/y span ratio 0.97-1.01, while the "
                  "stove slab in the same band is 0.17 m and the cabinet strip "
                  "is 0.158 x 0.04",
        "allowed": True},
    "SEAT_CLEAR": {
        "source": "debug seeds 51-65 radial profile of the bowl: the dish rises "
                  "from 0.907 at r=0 to 0.913 at r=0.021 (the bottle base radius), "
                  "so 10 mm above the floor leaves the base just clear",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug seeds 51-65: bottle top 1.059 and bowl rim 0.952; "
                  "carrying the neck at 1.20 keeps the base (0.123 below the "
                  "grip) above every measured obstacle",
        "allowed": True},
    "HOVER_Z": {
        "source": "debug seeds 51-65: bottle top z = 1.059; 1.15 clears it",
        "allowed": True},
    "R_DOWN": {
        "source": "generic controller mechanics: the straight-down tool frame "
                  "diag(1,-1,-1); the mate pack's grasp keyframe is a 3x3 "
                  "rotation about the vertical, immaterial for a round neck",
        "allowed": True},
    "WS_CROP": {
        "source": "debug seeds 51-65: the bottle sits at x -0.18..-0.21, the bowl "
                  "at x -0.09..-0.12; the crop excludes the cabinet (y<-0.18) and "
                  "the far wall",
        "allowed": True},
    "PARK": {
        "source": "debug seeds 51-65: v2's final cam_high dump showed the arm "
                  "parked at its start pose occluding the seated bottle; the "
                  "bowl sits at y ~= 0.00 and the tallest thing on the +y side "
                  "is the stove at z 0.96, so parking at y +0.15 / z 1.20 clears "
                  "both the view and the scene",
        "allowed": True},
    "HOLD_GAP_MIN": {
        "source": "debug seeds 51-65: bottle neck diameter 0.014-0.017 m, so a "
                  "closed gap above 0.008 m is neck, not air",
        "allowed": True},
}

GRASP_Z_ABS = 1.0223
NECK_DROP_LO, NECK_DROP_HI = 0.022, 0.050
BOTTLE_BAND_LO, BOTTLE_BAND_HI = 0.09, 0.21
BOTTLE_SPAN_MAX = 0.06
BOWL_BAND_LO, BOWL_BAND_HI = 0.025, 0.09
BOWL_D_LO, BOWL_D_HI = 0.07, 0.15
BOWL_ASPECT = 1.35
SEAT_CLEAR = 0.010
CARRY_Z = 1.20
HOVER_Z = 1.15
WS_CROP = (-0.34, 0.20, -0.18, 0.30)
HOLD_GAP_MIN = 0.008
PARK = [-0.2085, 0.15, 1.20]
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


# --------------------------------------------------------------------------- perception
def _cloud(f):
    z = np.asarray(f.depth, float)
    K, T = np.asarray(f.intrinsics, float), np.asarray(f.t_base_cam, float)
    h, w = z.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    with np.errstate(all="ignore"):
        pts = np.stack([(u - K[0, 2]) * z / K[0, 0],
                        (v - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], -1) @ T.T
    return pts[..., :3]


def _components(mask, min_px):
    """4-connected components of a boolean image, as (v, u) index arrays."""
    seen = np.zeros(mask.shape, bool)
    h, w = mask.shape
    out = []
    for v0, u0 in zip(*np.nonzero(mask)):
        if seen[v0, u0]:
            continue
        stack = [(v0, u0)]
        seen[v0, u0] = True
        px = []
        while stack:
            v, u = stack.pop()
            px.append((v, u))
            for a, b in ((v + 1, u), (v - 1, u), (v, u + 1), (v, u - 1)):
                if 0 <= a < h and 0 <= b < w and mask[a, b] and not seen[a, b]:
                    seen[a, b] = True
                    stack.append((a, b))
        if len(px) >= min_px:
            out.append(np.array(px))
    return out


def _table_z(Z, ws):
    """Modal height in the workspace crop: the table is most of it."""
    vals = Z[ws]
    vals = vals[np.isfinite(vals)]
    hist, edges = np.histogram(vals, bins=np.arange(0.80, 1.40, 0.002))
    i = int(np.argmax(hist))
    return float((edges[i] + edges[i + 1]) / 2)


def perceive(api):
    f = api.capture("cam_high")
    p = _cloud(f)
    X, Y, Z = p[..., 0], p[..., 1], p[..., 2]
    x0, x1, y0, y1 = WS_CROP
    ws = (X > x0) & (X < x1) & (Y > y0) & (Y < y1) & np.isfinite(Z)
    table = _table_z(Z, ws)
    api.log(f"PERC table_z={table:.4f}")

    # -- the bottle: the one narrow thing standing well above the table
    band = ws & (Z > table + BOTTLE_BAND_LO) & (Z < table + BOTTLE_BAND_HI)
    bottle = None
    for c in _components(band, 60):
        v, u = c[:, 0], c[:, 1]
        sx = float(X[v, u].max() - X[v, u].min())
        sy = float(Y[v, u].max() - Y[v, u].min())
        api.log(f"PERC tallcomp n={len(c)} span=({sx:.3f},{sy:.3f}) "
                f"ztop={float(Z[v, u].max()):.3f}")
        if sx > BOTTLE_SPAN_MAX or sy > BOTTLE_SPAN_MAX:
            continue
        if bottle is None or len(c) > len(bottle):
            bottle = c
    if bottle is None:
        raise RuntimeError("no bottle component")
    v, u = bottle[:, 0], bottle[:, 1]
    ztop = float(Z[v, u].max())
    cap = Z[v, u] > ztop - 0.012           # the flat top disc, fully visible
    bx, by = float(X[v, u][cap].mean()), float(Y[v, u][cap].mean())
    api.log(f"PERC bottle xy=({bx:+.4f},{by:+.4f}) ztop={ztop:.4f} "
            f"npx={len(bottle)} ncap={int(cap.sum())}")

    # -- the bowl: the round ring just above the table
    lo = ws & (Z > table + BOWL_BAND_LO) & (Z < table + BOWL_BAND_HI)
    bowl = None
    for c in _components(lo, 300):
        v, u = c[:, 0], c[:, 1]
        sx = float(X[v, u].max() - X[v, u].min())
        sy = float(Y[v, u].max() - Y[v, u].min())
        ok = (BOWL_D_LO < sx < BOWL_D_HI and BOWL_D_LO < sy < BOWL_D_HI
              and 1 / BOWL_ASPECT < sx / sy < BOWL_ASPECT)
        api.log(f"PERC lowcomp n={len(c)} span=({sx:.3f},{sy:.3f}) ok={ok}")
        if not ok:
            continue
        if bowl is None or len(c) > len(bowl):
            bowl = c
    if bowl is None:
        raise RuntimeError("no bowl component")
    v, u = bowl[:, 0], bowl[:, 1]
    wx, wy = float(X[v, u].mean()), float(Y[v, u].mean())
    rim = float(np.percentile(Z[v, u], 95))
    r = min(float(X[v, u].max() - X[v, u].min()),
            float(Y[v, u].max() - Y[v, u].min())) / 2
    near = ws & (np.hypot(X - wx, Y - wy) < 0.5 * r)
    floor = float(np.percentile(Z[near], 5)) if near.sum() > 20 else table
    api.log(f"PERC bowl xy=({wx:+.4f},{wy:+.4f}) rim={rim:.4f} floor={floor:.4f} "
            f"r={r:.4f} npx={len(bowl)}")
    return dict(table=table, bx=bx, by=by, ztop=ztop,
                wx=wx, wy=wy, rim=rim, floor=floor)


def _dump(api, tag, f):
    rgb = np.ascontiguousarray(np.asarray(f.rgb, dtype=np.uint8))
    b = base64.b64encode(zlib.compress(rgb.tobytes(), 9)).decode()
    api.log(f"FRAME {tag} shape={list(rgb.shape)}")
    api.log(f"RGBLEN {tag} {len(b)}")
    for i in range(0, len(b), 1900):
        api.log(f"RGB {tag} {b[i:i + 1900]}")
    api.log(f"ENDFRAME {tag}")


# --------------------------------------------------------------------------- task
def run(api):
    api.log(f"INTENT {api.instruction()!r}")
    s = perceive(api)

    grasp_z = float(np.clip(GRASP_Z_ABS,
                            s["ztop"] - NECK_DROP_HI, s["ztop"] - NECK_DROP_LO))
    base_off = grasp_z - s["table"]          # bottle base hangs this far below the grip
    place_z = s["floor"] + SEAT_CLEAR + base_off
    api.log(f"PLAN grasp_z={grasp_z:.4f} base_off={base_off:.4f} place_z={place_z:.4f}")

    api.grip(0.08)
    r = api.move([s["bx"], s["by"], HOVER_Z], rotation=R_DOWN, seconds=3.0)
    api.log(f"ACT hover res={r:.4f} eef={np.round(api.eef(), 4).tolist()}")
    r = api.move([s["bx"], s["by"], grasp_z], rotation=R_DOWN, seconds=3.0)
    api.log(f"ACT descend res={r:.4f} eef={np.round(api.eef(), 4).tolist()}")

    api.grip(0.0)
    g = api.gripper()
    api.log(f"ACT closed gap={g['width_m']:.4f} effort={g['effort']:.2f}")

    r = api.move([s["bx"], s["by"], CARRY_Z], rotation=R_DOWN, seconds=3.0)
    g = api.gripper()
    api.log(f"ACT lift res={r:.4f} gap={g['width_m']:.4f} effort={g['effort']:.2f} "
            f"eef={np.round(api.eef(), 4).tolist()}")

    r = api.move([s["wx"], s["wy"], CARRY_Z], rotation=R_DOWN, seconds=3.0)
    g = api.gripper()
    api.log(f"ACT over_bowl res={r:.4f} gap={g['width_m']:.4f} "
            f"eef={np.round(api.eef(), 4).tolist()}")

    r = api.move([s["wx"], s["wy"], place_z], rotation=R_DOWN, seconds=3.0)
    g = api.gripper()
    api.log(f"ACT seat res={r:.4f} gap={g['width_m']:.4f} effort={g['effort']:.2f} "
            f"eef={np.round(api.eef(), 4).tolist()}")

    api.grip(0.08)
    api.settle(0.5)
    api.log(f"ACT released eef={np.round(api.eef(), 4).tolist()}")

    api.move([s["wx"], s["wy"], CARRY_Z], rotation=R_DOWN, seconds=3.0)
    api.move(PARK, rotation=R_DOWN, seconds=3.0)
    api.settle(1.0)

    try:
        after = perceive(api)
        dxy = float(np.hypot(after["bx"] - after["wx"], after["by"] - after["wy"]))
        api.log(f"VERIFY bottle_now=({after['bx']:+.4f},{after['by']:+.4f}) "
                f"ztop={after['ztop']:.4f} bowl=({after['wx']:+.4f},{after['wy']:+.4f}) "
                f"dxy={dxy:.4f} seated={dxy < 0.055}")
    except Exception as e:
        api.log(f"VERIFY failed: {type(e).__name__}: {e}")
    _dump(api, "final", api.capture("cam_high"))
