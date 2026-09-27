"""c2 / obj_ketchup_stock — v1_1 (v2 body, de-oracled for fair-v1.1).

v1_1 = v2 with every read of the runner's episode-done flag excised (this file
now contains zero references to it).  Under protocol v1.0 that flag also
signalled runtime task success; under fair-v1.1 it means episode termination
only.  In v2 it appeared ONLY inside two api.log() format strings ([release]
and [end]) and never in any branch, loop, or return value,
so no control flow depended on success feedback and nothing had to be replaced
by a sensor surrogate.  All decisions were already sensor-based: grasp
verification, drop detection and the release gate all read
api.gripper()["effort"] (>= 1.0 == holding), the descent ladder is a fixed
count, and perception is the program's own cam_high RGB-D.  Constants,
perception, motion and control flow are bit-identical to v2.

v2 = v1 + failure-triggered hardening ONLY (the nominal path is unchanged, so
the v1 receipt of 8/8 carries over): every grasp attempt re-perceives, the
descent height walks a ladder, a drop detected after the lift or over the
basket restarts a pick cycle, and the gripper is never opened over the basket
while nothing is held.  v1 used 172 of the 1000-step horizon, so two extra
pick cycles fit comfortably.

Intent: "pick up the ketchup and place it in the basket".

Strategy (fair, pack + debug-seed derived):
  1. cam_high RGB-D -> base-frame cloud (vectorised copy of FairFrame.deproject).
  2. table height = modal z of the cloud; above-table mask, eroded 1 px so that
     depth-discontinuity border pixels (which deproject onto the background
     plane) cannot join a prop.
  3. GROUND-FOOTPRINT clustering on a 1.5 cm XY grid (campaign law C2-L2).
  4. Identity by DEMO ANCHOR (campaign law C2-L1): the K demos' closing
     keyframe EEF xy agree to ~1.4 cm; the cluster nearest that mean is the
     ketchup. No colour / semantic model (law C2-L6: placing the wrong bottle
     scores zero, and this scene holds several look-alike bottles).
  5. Grasp at the cap-face bbox midpoint, descend to the demo grasp height,
     close, verify effort, carry, release over the basket.
"""

import numpy as np

PROVENANCE = {
    "ANCHOR_XY": {
        "source": "pack.json demos 1,2 closing keyframes (gripper_cmd -1 -> +1): "
                  "ee xy (-0.1123,-0.2516) and (-0.1261,-0.2498); mean",
        "allowed": True},
    "GRASP_Z": {
        "source": "pack.json demos 1,2 minimum ee z at the close keyframe "
                  "(0.1132, 0.1124) — the demo's own grasp height",
        "allowed": True},
    "BASKET_XY": {
        "source": "pack.json demo release keyframes (gripper_cmd +1 -> -1): "
                  "(0.0199,0.2410), (-0.0270,0.2455), (0.0923,0.2234); mean",
        "allowed": True},
    "RELEASE_Z": {
        "source": "pack.json demo release keyframe ee z (0.173, 0.212, 0.176); mean",
        "allowed": True},
    "CARRY_Z": {
        "source": "pack.json demo transport ee z (demo1 max 0.381, demo2 0.310); "
                  "a height the demos are known to traverse",
        "allowed": True},
    "APPROACH_Z": {
        "source": "pack.json demo pre-descent ee z above the target "
                  "(demo1 t=30 0.2376, demo2 t=30 0.1859)",
        "allowed": True},
    "HELD_WIDTH_M": {
        "source": "pack.json gripper_state while carrying (0.0170,-0.0169) -> "
                  "0.034 m object width; used only as a grasp sanity check",
        "allowed": True},
    "GRID_M": {
        "source": "campaign LAWS.md C2-L2 (sanctioned cross-cell channel): "
                  "1.5 cm ground-footprint grid; generic clustering resolution",
        "allowed": True},
    "TABLE_BAND": {
        "source": "debug-seed depth: modal cloud z is the table plane; props are "
                  "the 1.5-25 cm band above it (measured per episode at runtime)",
        "allowed": True},
    "CLAMP_M": {
        "source": "generic: perception is clamped to within 8 cm of the demo "
                  "prior (pack anchor) so a bad blob cannot send the arm away",
        "allowed": True},
}

CAM = "cam_high"
ANCHOR_XY = np.array([-0.1192, -0.2507])
BASKET_XY = np.array([0.0284, 0.2366])
GRASP_Z = 0.113
RELEASE_Z = 0.190
CARRY_Z = 0.320
APPROACH_Z = 0.230
HELD_WIDTH_M = 0.034
GRID_M = 0.015
CLAMP_M = 0.08
MIN_H = 0.015
MAX_H = 0.25


# ---------------------------------------------------------------- perception
def cloud(frame):
    """Base-frame xyz for every pixel + validity mask (same math as deproject)."""
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    p = np.stack([(uu - cx) * d / fx, (vv - cy) * d / fy, d, np.ones_like(d)], -1)
    return (p @ T.T)[..., :3], np.isfinite(d) & (d > 1e-6)


def erode(m):
    out = m.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            out &= np.roll(np.roll(m, dy, 0), dx, 1)
    return out


def table_height(P, valid):
    z = P[..., 2][valid]
    xy = P[..., :2][valid]
    near = (np.abs(xy[:, 0]) < 0.45) & (np.abs(xy[:, 1]) < 0.45)
    z = z[near]
    hist, edges = np.histogram(z, bins=np.arange(-0.30, 0.60, 0.005))
    k = int(np.argmax(hist))
    sel = z[(z >= edges[k] - 0.005) & (z <= edges[k + 1] + 0.005)]
    return float(np.median(sel)), int(hist[k]), int(z.size)


def clusters(P, mask):
    """Ground-footprint clusters (law C2-L2) of the masked points."""
    pts = P[mask]
    if pts.shape[0] < 10:
        return []
    ij = np.floor(pts[:, :2] / GRID_M).astype(np.int64)
    cells, inv = np.unique(ij, axis=0, return_inverse=True)
    inv = inv.ravel()
    counts = np.bincount(inv, minlength=cells.shape[0])
    live = counts >= 3
    index = {(int(c[0]), int(c[1])): n for n, c in enumerate(cells) if live[n]}
    seen, groups = set(), []
    for cell in index:
        if cell in seen:
            continue
        stack, comp = [cell], []
        seen.add(cell)
        while stack:
            c = stack.pop()
            comp.append(index[c])
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nb = (c[0] + dx, c[1] + dy)
                    if nb in index and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        groups.append(comp)
    out = []
    for comp in groups:
        sel = np.isin(inv, np.asarray(comp))
        q = pts[sel]
        if q.shape[0] < 12:
            continue
        top = float(q[:, 2].max())
        cap = q[q[:, 2] >= top - 0.012]
        mid = lambda a: (a.min(0) + a.max(0)) / 2.0          # noqa: E731
        out.append({
            "n": int(q.shape[0]), "cells": len(comp),
            "bbox_mid": mid(q[:, :2]), "cap_mid": mid(cap[:, :2]),
            "centroid": q[:, :2].mean(0), "top": top,
            "base": float(np.percentile(q[:, 2], 2)),
            "ext": (q[:, :2].max(0) - q[:, :2].min(0)),
            "cap_n": int(cap.shape[0]),
        })
    return out


def perceive(api, note=""):
    f = api.capture(CAM)
    P, valid = cloud(f)
    tz, tn, ttot = table_height(P, valid)
    above = valid & (P[..., 2] > tz + MIN_H) & (P[..., 2] < tz + MAX_H)
    above &= (np.abs(P[..., 0]) < 0.45) & (np.abs(P[..., 1]) < 0.45)
    cs = clusters(P, erode(above))
    api.log(f"[perceive{note}] table_z={tz:.4f} (mode n={tn}/{ttot}) "
            f"props={len(cs)}")
    for c in sorted(cs, key=lambda c: np.linalg.norm(c["cap_mid"] - ANCHOR_XY)):
        api.log("  prop n=%4d cells=%3d cap_mid=(%.4f,%.4f) bbox_mid=(%.4f,%.4f) "
                "cent=(%.4f,%.4f) top=%.4f h=%.4f ext=(%.3f,%.3f) "
                "d_anchor=%.4f d_basket=%.4f"
                % (c["n"], c["cells"], c["cap_mid"][0], c["cap_mid"][1],
                   c["bbox_mid"][0], c["bbox_mid"][1], c["centroid"][0],
                   c["centroid"][1], c["top"], c["top"] - tz, c["ext"][0],
                   c["ext"][1], float(np.linalg.norm(c["cap_mid"] - ANCHOR_XY)),
                   float(np.linalg.norm(c["cap_mid"] - BASKET_XY))))
    return tz, cs


def pick_target(api, cs):
    cand = [c for c in cs if 0.03 <= c["top"] - TABLE_Z[0] <= 0.22]
    if not cand:
        cand = list(cs)
    if not cand:
        api.log("[target] NO cluster -> demo anchor prior")
        return ANCHOR_XY.copy(), None
    best = min(cand, key=lambda c: np.linalg.norm(c["cap_mid"] - ANCHOR_XY))
    d = float(np.linalg.norm(best["cap_mid"] - ANCHOR_XY))
    xy = best["cap_mid"].copy()
    if d > CLAMP_M:
        api.log(f"[target] nearest blob {d:.3f} m from anchor > clamp -> prior")
        return ANCHOR_XY.copy(), best
    api.log(f"[target] xy=({xy[0]:.4f},{xy[1]:.4f}) d_anchor={d:.4f} "
            f"top={best['top']:.4f} n={best['n']}")
    return xy, best


def pick_basket(api, cs):
    cand = [c for c in cs if c["cells"] >= 15]
    if cand:
        b = min(cand, key=lambda c: np.linalg.norm(c["bbox_mid"] - BASKET_XY))
        d = float(np.linalg.norm(b["bbox_mid"] - BASKET_XY))
        if d <= 0.10:
            api.log(f"[basket] xy=({b['bbox_mid'][0]:.4f},{b['bbox_mid'][1]:.4f}) "
                    f"d_prior={d:.4f} cells={b['cells']} top={b['top']:.4f}")
            return b["bbox_mid"].copy()
        api.log(f"[basket] best big blob {d:.3f} m from prior -> prior")
    else:
        api.log("[basket] no big blob -> prior")
    return BASKET_XY.copy()


TABLE_Z = [0.0]


# --------------------------------------------------------------------- motor
def holding(api):
    g = api.gripper()
    return g["effort"] >= 1.0, g


DZ_LADDER = (0.0, -0.012, +0.012, -0.024)


def pick_cycle(api, cs0, cycle):
    """One approach->descend->close->verify attempt ladder. Returns (ok, txy)."""
    cs = cs0
    txy, _ = pick_target(api, cs)
    for attempt, dz in enumerate(DZ_LADDER):
        if attempt or cycle:                      # re-perceive before any retry
            tz, cs = perceive(api, f"-retry{cycle}.{attempt}")
            TABLE_Z[0] = tz
            txy, _ = pick_target(api, cs)
        api.grip(0.08)
        r0 = api.move([txy[0], txy[1], APPROACH_Z], seconds=2.5)
        gz = GRASP_Z + dz
        r = api.move([txy[0], txy[1], gz], seconds=2.0)
        api.log(f"[grasp{cycle}.{attempt}] z={gz:.4f} res_above={r0:.4f} "
                f"res={r:.4f} eef={np.round(api.eef(), 4).tolist()}")
        api.grip(0.0)
        api.settle(0.3)
        ok, g = holding(api)
        api.log(f"[grasp{cycle}.{attempt}] gripper={g} holding={ok} "
                f"(demo width {HELD_WIDTH_M})")
        if ok:
            return True, txy
        api.grip(0.08)
        api.move([txy[0], txy[1], APPROACH_Z], seconds=1.5)
    return False, txy


def run(api):
    api.log(f"instruction: {api.instruction()!r} cams={api.cameras}")
    api.log(f"eef0={np.round(api.eef(), 4).tolist()} grip0={api.gripper()}")

    tz, cs = perceive(api)
    TABLE_Z[0] = tz
    bxy = pick_basket(api, cs)

    grasped, txy = False, ANCHOR_XY.copy()
    for cycle in range(2):
        grasped, txy = pick_cycle(api, cs, cycle)
        if not grasped:
            api.log(f"[cycle{cycle}] no grasp after ladder")
            continue
        api.move([txy[0], txy[1], CARRY_Z], seconds=2.0)
        ok, g = holding(api)
        api.log(f"[lift] holding={ok} gripper={g} "
                f"eef={np.round(api.eef(), 4).tolist()}")
        if not ok:                                # dropped during the lift
            api.log("[lift] DROPPED -> new pick cycle")
            grasped = False
            continue
        api.move([bxy[0], bxy[1], CARRY_Z], seconds=3.0)
        ok, g = holding(api)
        api.log(f"[over-basket] eef={np.round(api.eef(), 4).tolist()} holding={ok}")
        if not ok:                                # dropped in transit
            api.log("[carry] DROPPED -> new pick cycle")
            grasped = False
            continue
        api.move([bxy[0], bxy[1], RELEASE_Z], seconds=2.0)
        api.grip(0.08)
        api.settle(1.0)
        api.log(f"[release] eef={np.round(api.eef(), 4).tolist()}")
        api.move([bxy[0], bxy[1], CARRY_Z], seconds=1.5)
        api.settle(0.5)
        break

    api.log(f"[end] grasped={grasped}")
    return f"grasped={grasped} target={np.round(txy, 4).tolist()}"
