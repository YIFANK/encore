"""c2 / obj_cream_cheese_stock — v4

Mechanism (established by v1-v3 on debug seeds 51/53):
  * api.move exits as soon as the residual is under ~0.012 m, and its approach
    is asymptotic, so commanding the demo grasp z (0.0098) directly stalls the
    descent at eef z = 0.042 +- 0.0002 (v1, 8/8 seeds) with the jaws 3 cm above
    the box -> close on air (width 0.001, effort 0.05).
  * Commanding a z BELOW the physical floor keeps the position error large, so
    the controller drives down until contact: eef z = 0.0091 over the target,
    which is exactly the demos' closing-keyframe z (0.0091/0.0101/0.0102).
    Close there -> width 0.0422, effort 3.0 (demos hold 0.0428-0.0438). v3: 2/2.

Pipeline: footprint-cluster survey (C2-L2) -> nearest cluster to the demo
closing anchor (C2-L1) -> bbox midpoint (C2-L5) -> press-to-contact grasp ->
carry -> release at the demo opening anchor over the basket.
"""
import numpy as np

PROVENANCE = {
    "DEMO_ANCHOR_XY": {
        "source": "pack.json demos[*].keyframes with gripper_cmd 1.0: EEF xy "
                  "(0.0627,-0.1162),(0.0230,-0.1089),(0.0228,-0.1194); mean",
        "allowed": True},
    "RELEASE_XY": {
        "source": "pack.json opening-keyframe (gripper_cmd -1 while holding) EEF xy "
                  "(0.0298,0.2661),(0.0654,0.2396),(-0.0354,0.2555); mean",
        "allowed": True},
    "RELEASE_Z": {
        "source": "pack.json opening-keyframe EEF z 0.1789/0.2193/0.1517; mean 0.183",
        "allowed": True},
    "HOVER_Z": {
        "source": "pack.json ee_path6 pre-grasp hover z (0.11-0.20)", "allowed": True},
    "CARRY_Z": {
        "source": "pack.json ee_path6 mid-carry z 0.22-0.27", "allowed": True},
    "PUSH_Z": {
        "source": "generic controller mechanics: a command below the physical floor "
                  "keeps the residual above api.move's ~0.012 m exit tolerance "
                  "(tolerance measured on debug seeds 51/53, v2; floor reached "
                  "eef z 0.0091 on seeds 51/53, v3)",
        "allowed": True},
    "PUSH_TRIES / PUSH_SECONDS": {
        "source": "debug seeds 51/53 (v3): push0 already reaches the floor, push1 "
                  "repeats it to within 0.0001 m", "allowed": True},
    "CELL_M": {
        "source": "generic footprint-clustering grid resolution (campaign law C2-L2)",
        "allowed": True},
    "OBJ_BAND": {
        "source": "generic above-table segmentation band over the depth-median table "
                  "plane (measured 0.0016 on debug seeds 51-65)", "allowed": True},
    "MIN_CELLS": {"source": "generic small-blob rejection", "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi contract (effort 3.0 iff holding); observed 3.0 after a "
                  "successful close on debug seeds 51/53", "allowed": True},
    "TIE_M": {
        "source": "debug seeds 51-65 (v1/v4): target d_anchor 0.019-0.023, runner-up "
                  "0.108 — a 0.05 m band is far below that gap, so the colour "
                  "tiebreak is inert unless the anchor becomes ambiguous",
        "allowed": True},
    "BLUE_MARGIN": {
        "source": "pack keyframes (the prop that leaves the table and appears in the "
                  "jaws is the blue-faced flat box) + debug-seed cluster mean RGB: "
                  "target (73,80,98) has B-R = +25, the other flat box (105,64,44) "
                  "has B-R = -61", "allowed": True},
    "OPEN_W / CLOSE_W": {
        "source": "FairApi contract (grip <0.025 closes, else opens)", "allowed": True},
}

DEMO_ANCHOR_XY = np.array([0.0362, -0.1148])
RELEASE_XY = np.array([0.0199, 0.2537])
RELEASE_Z = 0.183
HOVER_Z = 0.15
CARRY_Z = 0.25
PUSH_Z = -0.03
PUSH_TRIES = 2
PUSH_SECONDS = 1.5
CELL_M = 0.015
OBJ_BAND = (0.012, 0.16)
MIN_CELLS = 3
HOLD_EFFORT = 2.0
TIE_M = 0.05
OPEN_W = 0.08
CLOSE_W = 0.0


# ---------------------------------------------------------------- perception
def _cloud(api, cam="cam_high"):
    f = api.capture(cam)
    d = np.asarray(f.depth, float)
    if d.ndim == 3:
        d = d[..., 0]
    H, W = d.shape
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    vs, us = np.mgrid[0:H, 0:W]
    valid = np.isfinite(d) & (d > 1e-4)
    z = np.where(valid, d, 1.0)
    xc = (us - K[0, 2]) * z / K[0, 0]
    yc = (vs - K[1, 2]) * z / K[1, 1]
    pts = np.stack([xc, yc, z], -1).reshape(-1, 3)
    P = (pts @ T[:3, :3].T + T[:3, 3]).reshape(H, W, 3)
    return f, P, valid


def _clusters(P, mask, cell=CELL_M):
    """Group above-table points by GROUND FOOTPRINT cell connectivity (C2-L2)."""
    idx = np.nonzero(mask)
    if idx[0].size == 0:
        return []
    xy = P[..., :2][mask]
    zz = P[..., 2][mask]
    cells = np.floor(xy / cell).astype(int)
    occ = {}
    for i, c in enumerate(map(tuple, cells)):
        occ.setdefault(c, []).append(i)
    seen, out = set(), []
    for c0 in occ:
        if c0 in seen:
            continue
        stack, comp = [c0], []
        seen.add(c0)
        while stack:
            c = stack.pop()
            comp.append(c)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in occ and n not in seen:
                        seen.add(n)
                        stack.append(n)
        m = np.concatenate([np.asarray(occ[c], int) for c in comp])
        pxy, pz = xy[m], zz[m]
        lo, hi = pxy.min(0), pxy.max(0)
        out.append({"n": int(m.size), "cells": len(comp),
                    "mid": (lo + hi) / 2.0, "centroid": pxy.mean(0),
                    "extent": hi - lo, "ztop": float(np.percentile(pz, 95)),
                    "rows": idx[0][m], "cols": idx[1][m]})
    return out


def _target(api):
    f, P, valid = _cloud(api)
    zb = P[..., 2]
    table = float(np.median(zb[valid]))
    band = valid & (zb > table + OBJ_BAND[0]) & (zb < table + OBJ_BAND[1])
    cl = [c for c in _clusters(P, band) if c["cells"] >= MIN_CELLS]
    rgb = np.asarray(f.rgb, int)
    for c in cl:
        c["d_anchor"] = float(np.linalg.norm(c["mid"] - DEMO_ANCHOR_XY))
        col = rgb[c["rows"], c["cols"]].mean(0)
        c["rgb"] = col
        c["blue"] = float(col[2] - col[0])
    cl.sort(key=lambda c: c["d_anchor"])
    api.log("survey table_z=%.4f clusters=%d" % (table, len(cl)))
    for c in cl[:3]:
        api.log("  cl n=%4d mid=(%.4f,%.4f) ext=(%.3f,%.3f) ztop=%.4f rgb=(%3.0f,%3.0f,%3.0f) "
                "d_anchor=%.4f" % (c["n"], c["mid"][0], c["mid"][1], c["extent"][0],
                                   c["extent"][1], c["ztop"] - table,
                                   c["rgb"][0], c["rgb"][1], c["rgb"][2], c["d_anchor"]))
    if not cl:
        return None, table
    # Inert unless the anchor is ambiguous: among clusters essentially tied on
    # the demo anchor, prefer the blue-faced one (the demos' carried prop).
    tied = [c for c in cl if c["d_anchor"] - cl[0]["d_anchor"] < TIE_M]
    best = cl[0]
    if len(tied) > 1:
        alt = max(tied, key=lambda c: c["blue"])
        api.log("anchor AMBIGUOUS (%d tied within %.3f) -> colour tiebreak picks "
                "mid=(%.4f,%.4f) blue=%.1f" % (len(tied), TIE_M, alt["mid"][0],
                                               alt["mid"][1], alt["blue"]))
        best = alt
    return best, table


# ------------------------------------------------------------------- motions
def _press_down(api, x, y, tag):
    """Drive to contact: command below the floor so the residual never falls
    under api.move's exit tolerance."""
    zs = []
    for k in range(PUSH_TRIES):
        r = api.move([x, y, PUSH_Z], seconds=PUSH_SECONDS)
        e = api.eef()
        zs.append(float(e[2]))
        api.log("%s press%d -> eef=(%.4f,%.4f,%.4f) res=%.4f" % (tag, k, e[0], e[1], e[2], r))
        if k and abs(zs[-1] - zs[-2]) < 0.002:
            break
        if api.done:
            break
    return zs[-1]


def run(api):
    api.log("instruction=%r eef0=%s" % (api.instruction(), np.round(api.eef(), 4).tolist()))
    api.grip(OPEN_W)
    tgt, table = _target(api)
    if tgt is None:
        return "no clusters"
    gx, gy = float(tgt["mid"][0]), float(tgt["mid"][1])
    api.log("TARGET mid=(%.4f,%.4f) ztop=%.4f d_anchor=%.4f"
            % (gx, gy, tgt["ztop"] - table, tgt["d_anchor"]))

    held = False
    for attempt in range(2):
        api.move([gx, gy, HOVER_Z], seconds=2.0)
        api.grip(OPEN_W)
        z = _press_down(api, gx, gy, "grasp%d" % attempt)
        api.grip(CLOSE_W)
        api.settle(0.4)
        g = api.gripper()
        api.log("attempt%d close at z=%.4f -> width=%.4f effort=%.2f"
                % (attempt, z, g["width_m"], g["effort"]))
        api.move([gx, gy, CARRY_Z], seconds=1.5)
        g = api.gripper()
        api.log("attempt%d after lift width=%.4f effort=%.2f eef=%s"
                % (attempt, g["width_m"], g["effort"], np.round(api.eef(), 4).tolist()))
        if g["effort"] >= HOLD_EFFORT and g["width_m"] > 0.005:
            held = True
            break
        api.log("attempt%d lost it — re-surveying" % attempt)
        api.grip(OPEN_W)
        # the box may have been nudged; re-ground it, and fall back to the point
        # centroid as an independent centre estimate
        try:
            tgt2, table = _target(api)
            if tgt2 is not None:
                gx, gy = float(tgt2["centroid"][0]), float(tgt2["centroid"][1])
                api.log("retry target centroid=(%.4f,%.4f) d_anchor=%.4f"
                        % (gx, gy, tgt2["d_anchor"]))
        except Exception as e:
            api.log("re-survey failed: %r" % (e,))

    if not held:
        api.log("no hold after retries; carrying anyway")

    api.move([RELEASE_XY[0], RELEASE_XY[1], CARRY_Z], seconds=2.0)
    g = api.gripper()
    api.log("over basket width=%.4f effort=%.2f eef=%s"
            % (g["width_m"], g["effort"], np.round(api.eef(), 4).tolist()))
    api.move([RELEASE_XY[0], RELEASE_XY[1], RELEASE_Z], seconds=1.0)
    api.grip(OPEN_W)
    api.settle(0.6)
    api.log("released; done=%s" % api.done)
    return "v4 held=%s" % held
