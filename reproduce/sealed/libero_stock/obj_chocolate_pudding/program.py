"""c2 obj_chocolate_pudding_stock -- v1.1 (de-oracle pass over v2).

Intent: "pick up the chocolate pudding and place it in the basket".

v1.1 de-oracle pass: under fair-v1.1 the api's terminated flag means
episode-terminated only, not runtime task success.  Audit of v2 found exactly
ONE read of that flag, in the final api.log format string; it gated nothing.
It is removed here, and this program now reads no oracle signal at all.  The
program has no success-conditioned early exit and no success-gated retry: the
single conditional in run() is the post-lift re-grasp, already gated on the
sensor `api.gripper()["effort"]`.  No other change -- constants, perception,
sequencing and PROVENANCE are byte-identical to v2.

v1 finding (results/fs_c2_obj_chocolate_pudding_stock_v1, 0/8): image-space
connected components MERGE the target with the tall bottle standing in front of
it (one cluster with dx = 0.281 m), so the "top face" centroid landed on the
bottle and the nearest-anchor rule picked a cup 0.19 m away.  v2 clusters the
depth cloud by its XY FOOTPRINT instead, which separates objects that merely
overlap in the image, and it grasps the footprint nearest the pack's grasp
anchor.
"""
from __future__ import annotations

import numpy as np

PROVENANCE = {
    "GRASP_ANCHOR_XY": {
        "source": "pack.json demos[*].keyframes[1].ee[:2] at the gripper-close "
                  "keyframe: (-0.1103,-0.2504), (-0.1234,-0.2634), "
                  "(-0.1107,-0.2489) -> mean (-0.1148,-0.2542)",
        "allowed": True},
    "ANCHOR_GATE_M": {
        "source": "spread of the three pack grasp keyframes about their mean "
                  "(max 0.011 m) widened for the per-seed layout jitter seen in "
                  "the v1 debug census (basket xy moved <=0.016 m over seeds "
                  "51-65) -> 0.06 m acceptance gate",
        "allowed": True},
    "GRASP_Z": {
        "source": "pack.json demos[*].keyframes[1].ee[2] at the gripper-close "
                  "keyframe: 0.0114, 0.0092, 0.0094 -> 0.010",
        "allowed": True},
    "PREGRASP_Z": {
        "source": "pack.json demos[*].ee_path6: the approach passes z ~0.14-0.23 "
                  "above the target before the vertical descent -> 0.16",
        "allowed": True},
    "PLACE_ANCHOR_XY": {
        "source": "pack.json demos[*].keyframes[2].ee[:2] at the gripper-open "
                  "keyframe: (-0.0027,0.2460), (-0.0026,0.2452), (-0.0185,0.2628) "
                  "-> mean (-0.0079,0.2513)",
        "allowed": True},
    "PLACE_OFFSET_XY": {
        "source": "PLACE_ANCHOR_XY minus the basket rim centroid measured by the "
                  "v1 debug census on seeds 51-65 (mean (0.0075,0.2582)) -> "
                  "(-0.0154,-0.0069)",
        "allowed": True},
    "PLACE_Z": {
        "source": "pack.json demos[*].keyframes[2].ee[2] at the gripper-open "
                  "keyframe: 0.1635, 0.1751, 0.1656 -> 0.168",
        "allowed": True},
    "TRANSIT_Z": {
        "source": "pack.json demos[*].ee_path6 carry-segment z (0.21-0.27) -> 0.24",
        "allowed": True},
    "OBJ_Z_BAND": {
        "source": "v1 debug census on seeds 51-65: every tabletop prop had "
                  "z_top in (0.08,0.148) while the parked arm cluster reached "
                  "z_top 0.319 -> keep 0.020 < z < 0.30, drop clusters above 0.20",
        "allowed": True},
    "FOOTPRINT_CELL_M": {
        "source": "v1 debug census: a single prop spans <=0.061 m in x and the "
                  "target sits ~0.2 m behind the bottle it merged with; 0.015 m "
                  "grid cells separate them",
        "allowed": True},
    "GRIP_WIDTHS": {
        "source": "FairApi contract (api.grip: <0.025 closes, else opens)",
        "allowed": True},
    "BASKET_Y_MIN": {
        "source": "pack.json release keyframes are at y ~ +0.25 and the v1 debug "
                  "census found exactly one prop with y > 0.15 (the basket)",
        "allowed": True},
}

GRASP_ANCHOR_XY = (-0.1148, -0.2542)
ANCHOR_GATE_M = 0.06
GRASP_Z = 0.010
PREGRASP_Z = 0.16
PLACE_ANCHOR_XY = (-0.0079, 0.2513)
PLACE_OFFSET_XY = (-0.0154, -0.0069)
PLACE_Z = 0.168
TRANSIT_Z = 0.24
Z_MIN, Z_MAX = 0.020, 0.30
Z_TOP_MAX = 0.20
CELL = 0.015
BASKET_Y_MIN = 0.15
OPEN_W = 0.08
CLOSE_W = 0.02


# ---------------------------------------------------------------- perception

def _cloud(frame, step=2):
    d = np.asarray(frame.depth, float)[::step, ::step]
    rgb = np.asarray(frame.rgb, int)[::step, ::step]
    h, w = d.shape[:2]
    K = np.asarray(frame.intrinsics, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    uu, vv = np.meshgrid(np.arange(w) * step, np.arange(h) * step)
    good = np.isfinite(d) & (d > 0)
    z = np.where(good, d, 0.0)
    pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z, np.ones_like(z)], -1)
    base = pc @ np.asarray(frame.t_base_cam, float).T
    return base[..., :3], rgb, good


def _footprint_clusters(P, C, min_pts=40):
    """Cluster points by XY footprint on a CELL grid (8-connected)."""
    keys = np.floor(P[:, :2] / CELL).astype(int)
    cells = {}
    for i, (a, b) in enumerate(map(tuple, keys)):
        cells.setdefault((a, b), []).append(i)
    seen = set()
    out = []
    for c0 in list(cells):
        if c0 in seen:
            continue
        stack, comp = [c0], []
        seen.add(c0)
        while stack:
            c = stack.pop()
            comp.extend(cells[c])
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    n = (c[0] + da, c[1] + db)
                    if n in cells and n not in seen:
                        seen.add(n)
                        stack.append(n)
        if len(comp) < min_pts:
            continue
        idx = np.array(comp)
        p, col = P[idx], C[idx]
        ztop = float(np.percentile(p[:, 2], 97))
        top = p[:, 2] > (ztop - 0.012)
        out.append({
            "n": int(len(idx)),
            "xy": (round(float(np.mean(p[top, 0])), 4),
                   round(float(np.mean(p[top, 1])), 4)),
            "med_xy": (round(float(np.median(p[:, 0])), 4),
                       round(float(np.median(p[:, 1])), 4)),
            "z_top": round(ztop, 4),
            "dx": round(float(p[:, 0].max() - p[:, 0].min()), 3),
            "dy": round(float(p[:, 1].max() - p[:, 1].min()), 3),
            "rgb": [int(v) for v in col.mean(0).round()],
            "rgb_top": [int(v) for v in col[top].mean(0).round()],
        })
    out.sort(key=lambda c: c["xy"][1])
    return out


def census(api, tag=""):
    f = api.capture("cam_high")
    P, C, good = _cloud(f)
    m = (good & (P[..., 2] > Z_MIN) & (P[..., 2] < Z_MAX)
         & (P[..., 0] > -0.40) & (P[..., 0] < 0.40)
         & (P[..., 1] > -0.42) & (P[..., 1] < 0.42))
    cl = _footprint_clusters(P[m], C[m])
    cl = [c for c in cl if c["z_top"] < Z_TOP_MAX]
    for c in cl:
        api.log("census%s %s" % (tag, c))
    return cl


# ---------------------------------------------------------------- behaviour

def run(api):
    api.log("instruction=%r" % api.instruction())
    api.log("eef0=%s grip0=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    cl = census(api)

    ax, ay = GRASP_ANCHOR_XY
    tgt, best = None, 1e9
    for c in cl:
        d = float(np.hypot(c["xy"][0] - ax, c["xy"][1] - ay))
        if d < best:
            tgt, best = c, d
    if tgt is not None and best <= ANCHOR_GATE_M:
        gx, gy = tgt["xy"]
        api.log("TARGET(seen) %s d=%.3f" % (tgt, best))
    else:
        gx, gy = ax, ay
        api.log("TARGET(anchor fallback) nearest=%s d=%.3f" % (tgt, best))

    cand = [c for c in cl if c["xy"][1] > BASKET_Y_MIN]
    if cand:
        b = max(cand, key=lambda c: c["n"])
        px = b["xy"][0] + PLACE_OFFSET_XY[0]
        py = b["xy"][1] + PLACE_OFFSET_XY[1]
        api.log("BASKET %s -> place (%.4f,%.4f)" % (b, px, py))
    else:
        px, py = PLACE_ANCHOR_XY
        api.log("BASKET not seen -> demo anchor")

    api.grip(OPEN_W)
    api.move([gx, gy, PREGRASP_Z], seconds=2.0)
    r = api.move([gx, gy, GRASP_Z], seconds=2.0)
    api.log("descend res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(CLOSE_W)
    api.settle(0.3)
    api.log("closed grip=%s" % (api.gripper(),))

    api.move([gx, gy, TRANSIT_Z], seconds=2.0)
    g = api.gripper()
    api.log("lifted grip=%s eef=%s" % (g, np.round(api.eef(), 4).tolist()))

    if g["effort"] < 1.0:                      # nothing in hand -> one retry
        api.log("RETRY grasp")
        api.grip(OPEN_W)
        api.move([gx, gy, PREGRASP_Z], seconds=1.5)
        r = api.move([gx, gy, GRASP_Z - 0.004], seconds=2.0)
        api.log("retry descend res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
        api.grip(CLOSE_W)
        api.settle(0.3)
        api.move([gx, gy, TRANSIT_Z], seconds=2.0)
        api.log("retry lifted grip=%s" % (api.gripper(),))

    api.move([px, py, TRANSIT_Z], seconds=3.0)
    api.log("over basket grip=%s eef=%s" % (api.gripper(),
                                            np.round(api.eef(), 4).tolist()))
    r = api.move([px, py, PLACE_Z], seconds=2.0)
    api.log("place res=%.4f eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.5)
    api.move([px, py, TRANSIT_Z], seconds=1.5)
    api.log("END eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
