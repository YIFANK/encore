"""c2 cell obj_tomato_sauce_stock -- v2

Intent: "pick up the tomato sauce and place it in the basket".

Strategy (all constants re-derived from this cell's pack + debug seeds):
  * The K=3 demos' closing keyframes agree to ~1.3 cm in EEF xy -> their mean is
    a target anchor.  Perceive above-table props from cam_high depth, group them
    by GROUND FOOTPRINT on a 1.5 cm XY grid (image connectivity fuses a short
    prop standing behind a tall one), and pick the footprint whose XY bbox
    midpoint is nearest the anchor.
  * Grasp top-down at the demo grasp height, lift, carry to the basket
    (perceived, clamped to the demos' release anchor), release.
v2 = v1 (8/8 on odd debug seeds) plus three fallback hedges: a height gate on
the target footprint, a shape-gated basket with a wider positional clamp, and a
second regrasp attempt.  The program is also a perception dump: every component is logged with midpoint / height /
pixel count so the next version can be diagnosed from logs alone.
"""
import numpy as np

# ---- calibrated constants (see PROVENANCE) ---------------------------------
PICK_ANCHOR = (0.0512, -0.1116)      # mean EEF xy at the demos' close keyframe
GRASP_Z = 0.0492                     # mean EEF z at the demos' close keyframe
RELEASE_ANCHOR = (-0.0232, 0.2600)   # mean EEF xy at the demos' open keyframe
RELEASE_Z = 0.175                    # demo release z 0.133..0.181 -> top of band
SAFE_Z = 0.235                       # demo transit z (ee_path max ~0.246)
BASKET_CLAMP = 0.12                  # accept a shape-gated basket this far out
TARGET_H = (0.045, 0.115)            # target prop height band (measured 0.080)
BASKET_H = (0.100, 0.190)            # basket rim height band (measured 0.141)
CELL = 0.015                         # footprint grid pitch
ABOVE_TABLE = 0.012                  # a point is "prop" this far above the table
WS = (-0.40, 0.40, -0.45, 0.45)      # x_lo,x_hi,y_lo,y_hi search box

PROVENANCE = {
    "PICK_ANCHOR": {"source": "pack.json demos[*].keyframes gripper_cmd=+1 EEF xy mean (0.0543,-0.1108)/(0.0560,-0.1156)/(0.0434,-0.1083)", "allowed": True},
    "GRASP_Z": {"source": "pack.json demos[*].keyframes gripper_cmd=+1 EEF z mean (0.0460/0.0442/0.0575)", "allowed": True},
    "RELEASE_ANCHOR": {"source": "pack.json demos[*].keyframes gripper_cmd=-1-after-grasp EEF xy mean (-0.0402,0.2591)/(-0.0265,0.2596)/(-0.0029,0.2614)", "allowed": True},
    "RELEASE_Z": {"source": "pack.json demo release-keyframe EEF z band 0.1333..0.1808", "allowed": True},
    "SAFE_Z": {"source": "pack.json demos[*].ee_path transit z max ~0.246", "allowed": True},
    "BASKET_CLAMP": {"source": "generic hedge: accept a shape-gated basket within 12 cm of the pack release anchor, else fall back to the anchor", "allowed": True},
    "TARGET_H": {"source": "debug seeds 51-65 (results/fs_c2_obj_tomato_sauce_stock_v1 logs): anchored prop ztop-table = 0.0797 m on 8/8 seeds; band padded +-45%", "allowed": True},
    "BASKET_H": {"source": "debug seeds 51-65 (results/fs_c2_obj_tomato_sauce_stock_v1 logs): basket component ztop-table = 0.141 m on 8/8 seeds; band padded", "allowed": True},
    "CELL": {"source": "generic perception mechanics: XY grid pitch below prop pitch", "allowed": True},
    "ABOVE_TABLE": {"source": "generic perception mechanics: depth-noise margin above the fitted table plane", "allowed": True},
    "WS": {"source": "pack.json ee_path xy extent padded to a table-sized search box", "allowed": True},
}


# ---------------------------------------------------------------------------
def cloud(frame):
    """Full base-frame point cloud (mirrors FairFrame.deproject, vectorised)."""
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape[:2]
    vs, us = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 0)
    z = d[ok].astype(float)
    u = us[ok].astype(float)
    v = vs[ok].astype(float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)
    P = pc @ T[:3, :3].T + T[:3, 3]
    return P, np.stack([u, v], 1)


def components(P, table_z):
    """Group above-table points by ground footprint on a CELL grid."""
    m = ((P[:, 0] > WS[0]) & (P[:, 0] < WS[1]) &
         (P[:, 1] > WS[2]) & (P[:, 1] < WS[3]) &
         (P[:, 2] > table_z + ABOVE_TABLE) & (P[:, 2] < table_z + 0.60))
    Q = P[m]
    if Q.shape[0] == 0:
        return [], Q
    ij = np.floor(Q[:, :2] / CELL).astype(int)
    cells = {}
    for k in range(Q.shape[0]):
        cells.setdefault((int(ij[k, 0]), int(ij[k, 1])), []).append(k)
    seen, comps = set(), []
    for c in cells:
        if c in seen:
            continue
        stack, group = [c], []
        seen.add(c)
        while stack:
            cur = stack.pop()
            group.append(cur)
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb = (cur[0] + di, cur[1] + dj)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        idx = np.concatenate([np.asarray(cells[g], int) for g in group])
        pts = Q[idx]
        comps.append({
            "n": int(pts.shape[0]),
            "cells": len(group),
            "mid": ((float(pts[:, 0].min()) + float(pts[:, 0].max())) / 2.0,
                    (float(pts[:, 1].min()) + float(pts[:, 1].max())) / 2.0),
            "cen": (float(pts[:, 0].mean()), float(pts[:, 1].mean())),
            "ext": (float(pts[:, 0].max() - pts[:, 0].min()),
                    float(pts[:, 1].max() - pts[:, 1].min())),
            "ztop": float(np.percentile(pts[:, 2], 99)),
        })
    comps.sort(key=lambda c: -c["n"])
    return comps, Q


def perceive(api):
    f = api.capture("cam_high")
    P, _ = cloud(f)
    cam = np.asarray(f.t_base_cam, float)[:3, 3]
    box = ((P[:, 0] > WS[0]) & (P[:, 0] < WS[1]) &
           (P[:, 1] > WS[2]) & (P[:, 1] < WS[3]))
    table_z = float(np.median(P[box][:, 2])) if box.sum() else 0.0
    comps, _ = components(P, table_z)
    api.log("cam@%s table_z=%.4f pts=%d comps=%d"
            % (np.round(cam, 3).tolist(), table_z, int(box.sum()), len(comps)))
    for c in comps[:12]:
        d = float(np.hypot(c["mid"][0] - PICK_ANCHOR[0], c["mid"][1] - PICK_ANCHOR[1]))
        api.log("  comp n=%d cells=%d mid=(%.4f,%.4f) cen=(%.4f,%.4f) "
                "ext=(%.3f,%.3f) ztop=%.4f d_anchor=%.4f"
                % (c["n"], c["cells"], c["mid"][0], c["mid"][1], c["cen"][0],
                   c["cen"][1], c["ext"][0], c["ext"][1], c["ztop"], d))
    return comps, table_z


# ---------------------------------------------------------------------------
def run(api):
    api.log("instruction: %s" % api.instruction())
    comps, table_z = perceive(api)
    if not comps:
        return "no components"

    def d_anc(c):
        return float(np.hypot(c["mid"][0] - PICK_ANCHOR[0],
                              c["mid"][1] - PICK_ANCHOR[1]))

    small = [c for c in comps if c["n"] >= 40]
    gated = [c for c in small
             if TARGET_H[0] <= c["ztop"] - table_z <= TARGET_H[1]]
    if not gated:
        api.log("height gate empty -> falling back to ungated nearest")
    tgt = min(gated or small or comps, key=d_anc)
    tx, ty = tgt["mid"]
    d_anchor = d_anc(tgt)
    api.log("TARGET mid=(%.4f,%.4f) ztop=%.4f n=%d d_anchor=%.4f"
            % (tx, ty, tgt["ztop"], tgt["n"], d_anchor))

    # basket: biggest footprint on the far (+y) side, clamped to the demo anchor
    bx, by = RELEASE_ANCHOR
    cand = [c for c in comps
            if c["mid"][1] > 0.12 and c["cells"] >= 40
            and BASKET_H[0] <= c["ztop"] - table_z <= BASKET_H[1]
            and np.hypot(c["mid"][0] - bx, c["mid"][1] - by) < BASKET_CLAMP]
    if cand:
        b = max(cand, key=lambda c: c["cells"])
        api.log("basket cand mid=(%.4f,%.4f) cells=%d ztop=%.4f (anchor %.4f,%.4f)"
                % (b["mid"][0], b["mid"][1], b["cells"], b["ztop"], bx, by))
        bx, by = b["mid"]
    else:
        api.log("no gated basket component -> demo release anchor")

    # ---- pick ---------------------------------------------------------------
    api.grip(0.08)
    api.move([tx, ty, SAFE_Z], seconds=2.0)
    api.move([tx, ty, GRASP_Z], seconds=2.0)
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("after close: eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), g))
    for dz in (-0.010, +0.010):
        if g["effort"] >= 2.5:
            break
        api.log("regrasp at dz=%+.3f" % dz)
        api.grip(0.08)
        api.move([tx, ty, GRASP_Z + dz], seconds=1.5)
        api.grip(0.0)
        api.settle(0.4)
        g = api.gripper()
        api.log("after regrasp %+.3f: %s" % (dz, g))

    # ---- lift + carry -------------------------------------------------------
    api.move([tx, ty, SAFE_Z], seconds=2.0)
    api.log("lifted: eef=%s grip=%s"
            % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.move([bx, by, SAFE_Z], seconds=2.5)
    api.move([bx, by, RELEASE_Z], seconds=1.5)
    api.log("over basket: eef=%s grip=%s"
            % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(0.08)
    api.settle(0.6)
    api.move([bx, by, SAFE_Z], seconds=1.2)
    api.settle(0.5)
    api.log("end: eef=%s done=%s" % (np.round(api.eef(), 4).tolist(), api.done))
    return "v2 target=(%.3f,%.3f) basket=(%.3f,%.3f)" % (tx, ty, bx, by)
