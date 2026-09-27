"""c2 cell obj_orange_juice_stock -- v2
Intent: pick up the orange juice and place it in the basket.

v1 (8/8 on debug 51..65 odd) established the mechanism:
  demo-anchor identification of the target footprint + perceived basket.
v2 adds only guards that cannot change the v1-good path:
  size gating of candidate footprints, a 6 cm clamp of the perceived target to
  the demo anchor (bounds wrong-object risk), and a re-perceive/regrasp loop
  keyed on gripper effort.

All constants come from packs/c2_obj_orange_juice_stock/pack.json (K=3 demos),
from debug-seed 51..65 measurements logged by v1, or from generic camera /
controller mechanics. See PROVENANCE.
"""
import numpy as np

PROVENANCE = {
    "ANCHOR_GRASP_XY": {
        "source": "pack.json demos[*].ee_path6[5] = first sampled pose carrying gripper cmd +1 "
                  "(0.060,-0.104),(0.038,-0.109),(0.045,-0.109) -> mean (0.0477,-0.1073)",
        "allowed": True},
    "ANCHOR_GRASP_Z": {
        "source": "pack.json demos[*].ee_path6[5][2] = 0.115,0.094,0.099 -> mean 0.1027",
        "allowed": True},
    "ANCHOR_RELEASE_XY": {
        "source": "pack.json demos[*].keyframes[2].ee (frame where gripper_cmd returns to -1) "
                  "(0.0018,0.2243),(-0.0215,0.2575),(0.0546,0.3026) -> mean (0.0116,0.2615)",
        "allowed": True},
    "ANCHOR_RELEASE_Z": {
        "source": "pack.json demos[*].keyframes[2].ee[2] = 0.1683,0.1467,0.1464 -> mean 0.1538",
        "allowed": True},
    "HELD_WIDTH": {
        "source": "pack.json demos[*].keyframes[2].gripper_state |a|+|b| = 0.0546,0.0533,0.0533; "
                  "reproduced on debug seeds 51..65 as 0.05341 with effort 3.0 (v1 logs)",
        "allowed": True},
    "CARRY_Z": {
        "source": "pack.json demos[*].ee_path max z during transfer = 0.296,0.317,0.300 -> 0.30",
        "allowed": True},
    "TABLE_CLEAR": {
        "source": "generic above-table depth cut; debug-seed 51 measured table_z=0.0020 and the "
                  "shortest prop top at 0.019, so 0.015 keeps every prop",
        "allowed": True},
    "CELL": {
        "source": "generic ground-footprint clustering grid, 0.015 m (debug-seed verified: each "
                  "prop stayed a separate component, extents <= 0.06 m)",
        "allowed": True},
    "TGT_CLAMP": {
        "source": "debug seeds 51..65 (v1 logs): target footprint sits 0.007 m from the demo "
                  "anchor while the nearest competitor sits 0.114 m away -> 0.06 m clamp lies "
                  "strictly between the two",
        "allowed": True},
    "BSK_CLAMP": {
        "source": "debug seeds 51..65 (v1 logs): perceived basket 0.002-0.026 m from the release "
                  "anchor, nearest competitor 0.272 m -> 0.10 m clamp",
        "allowed": True},
    "TGT_MAX_EXT": {
        "source": "debug seeds 51..65 (v1 logs): target footprint extent 0.053x0.053 m; basket "
                  "0.153x0.170 m and arm 0.121x0.237 m -> 0.12 m separates them",
        "allowed": True},
    "BSK_MIN_EXT": {
        "source": "same v1 debug-seed measurement (basket extent >= 0.15 m)",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi contract: gripper effort 3.0 iff holding; observed 3.0 on 8/8 debug "
                  "episodes after closing",
        "allowed": True},
}

ANCHOR_GRASP_XY = np.array([0.0477, -0.1073])
ANCHOR_GRASP_Z = 0.1027
ANCHOR_RELEASE_XY = np.array([0.0116, 0.2615])
ANCHOR_RELEASE_Z = 0.1538
HELD_WIDTH = 0.0537
CARRY_Z = 0.30
TABLE_CLEAR = 0.015
CELL = 0.015
TGT_CLAMP = 0.06
BSK_CLAMP = 0.10
TGT_MAX_EXT = 0.12
BSK_MIN_EXT = 0.12
HOLD_EFFORT = 2.5


# ---------------------------------------------------------------- perception
def get_cloud(api, cam="cam_high", step=2):
    """Base-frame point cloud from one capture (HxWx3, NaN where depth invalid)."""
    f = api.capture(cam)
    d = np.asarray(f.depth, dtype=float)
    K = np.asarray(f.intrinsics, dtype=float)
    T = np.asarray(f.t_base_cam, dtype=float)
    H, W = d.shape
    vs, us = np.mgrid[0:H:step, 0:W:step]
    z = d[vs, us]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    cands = {"cv": np.stack([x, y, z], -1), "gl": np.stack([x, -y, -z], -1)}
    good = np.isfinite(z) & (z > 1e-4)

    # Pick the camera-axis convention that reproduces api.deproject (generic
    # camera mechanics -- no task prior).  Falls back to "cv", which is the one
    # debug seeds 51..65 confirmed (grasp closed on the demo anchor, effort 3.0).
    ref = []
    ii = np.argwhere(good)
    if len(ii):
        for (a, b) in ii[:: max(1, len(ii) // 9)][:8]:
            try:
                ref.append((a, b, np.asarray(f.deproject(int(us[a, b]), int(vs[a, b])), float)))
            except Exception as e:
                if not ref:
                    api.log("deproject unavailable: %r" % (e,))
                break
    best, berr = "cv", float("inf")
    built = {}
    for name, C in cands.items():
        Pb = C @ T[:3, :3].T + T[:3, 3]
        built[name] = Pb
        if ref:
            e = float(np.mean([np.linalg.norm(Pb[a, b] - r) for (a, b, r) in ref]))
            if e < berr:
                berr, best = e, name
    api.log("cloud: convention=%s err=%s nref=%d" % (best, berr, len(ref)))
    P = built[best]
    P[~good] = np.nan
    return P


def table_z(P):
    z = P[..., 2]
    z = z[np.isfinite(z)]
    if len(z) == 0:
        return 0.0
    lo, hi = np.percentile(z, 1), np.percentile(z, 99)
    if not np.isfinite(lo) or hi <= lo:
        return float(np.median(z))
    hist, edges = np.histogram(z, bins=200, range=(lo, hi))
    k = int(np.argmax(hist))
    return float(0.5 * (edges[k] + edges[k + 1]))


def footprint_clusters(P, zmin, workspace=(-0.40, 0.45, -0.55, 0.55), minpts=25):
    """Group above-table points by GROUND FOOTPRINT cell (8-connected)."""
    x0, x1, y0, y1 = workspace
    m = (np.isfinite(P).all(-1) & (P[..., 2] > zmin) &
         (P[..., 0] > x0) & (P[..., 0] < x1) &
         (P[..., 1] > y0) & (P[..., 1] < y1))
    pts = P[m]
    if len(pts) == 0:
        return []
    gi = np.floor(pts[:, 0] / CELL).astype(int)
    gj = np.floor(pts[:, 1] / CELL).astype(int)
    i0, j0 = gi.min(), gj.min()
    ni, nj = gi.max() - i0 + 1, gj.max() - j0 + 1
    occ = np.zeros((ni, nj), dtype=bool)
    occ[gi - i0, gj - j0] = True
    lab = np.zeros((ni, nj), dtype=int)
    cur = 0
    for a in range(ni):
        for b in range(nj):
            if occ[a, b] and lab[a, b] == 0:
                cur += 1
                lab[a, b] = cur
                stack = [(a, b)]
                while stack:
                    ca, cb = stack.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            na, nb = ca + da, cb + db
                            if 0 <= na < ni and 0 <= nb < nj and occ[na, nb] and lab[na, nb] == 0:
                                lab[na, nb] = cur
                                stack.append((na, nb))
    plab = lab[gi - i0, gj - j0]
    out = []
    for c in range(1, cur + 1):
        sel = pts[plab == c]
        if len(sel) < minpts:
            continue
        ex = float(sel[:, 0].max() - sel[:, 0].min())
        ey = float(sel[:, 1].max() - sel[:, 1].min())
        out.append(dict(
            n=int(len(sel)),
            box=np.array([0.5 * (sel[:, 0].min() + sel[:, 0].max()),
                          0.5 * (sel[:, 1].min() + sel[:, 1].max())]),
            top=float(np.percentile(sel[:, 2], 98)),
            ext=(ex, ey),
        ))
    out.sort(key=lambda d: -d["n"])
    return out


def perceive(api, tag=""):
    P = get_cloud(api)
    tz = table_z(P)
    cl = footprint_clusters(P, tz + TABLE_CLEAR)
    api.log("%stable_z=%.4f nclusters=%d" % (tag, tz, len(cl)))
    for i, c in enumerate(cl):
        api.log("%scl%d n=%d box=(%.3f,%.3f) top=%.3f ext=(%.3f,%.3f) dG=%.3f dR=%.3f" %
                (tag, i, c["n"], c["box"][0], c["box"][1], c["top"], c["ext"][0], c["ext"][1],
                 float(np.linalg.norm(c["box"] - ANCHOR_GRASP_XY)),
                 float(np.linalg.norm(c["box"] - ANCHOR_RELEASE_XY))))
    return cl


def pick_target(api, cl):
    """Nearest SIZE-GATED footprint to the demo grasp anchor, clamped to it."""
    cand = [c for c in cl if max(c["ext"]) <= TGT_MAX_EXT]
    if not cand:
        cand = cl
    if not cand:
        api.log("TARGET fallback: anchor (no clusters)")
        return ANCHOR_GRASP_XY.copy(), None
    t = min(cand, key=lambda c: float(np.linalg.norm(c["box"] - ANCHOR_GRASP_XY)))
    d = float(np.linalg.norm(t["box"] - ANCHOR_GRASP_XY))
    comp = sorted(float(np.linalg.norm(c["box"] - ANCHOR_GRASP_XY)) for c in cand)
    api.log("TARGET box=(%.4f,%.4f) top=%.4f n=%d d=%.4f nextd=%.4f" %
            (t["box"][0], t["box"][1], t["top"], t["n"], d,
             comp[1] if len(comp) > 1 else float("nan")))
    if d > TGT_CLAMP:
        api.log("TARGET clamped to anchor (d=%.3f > %.3f)" % (d, TGT_CLAMP))
        return ANCHOR_GRASP_XY.copy(), t
    return t["box"].copy(), t


def pick_basket(api, cl):
    cand = [c for c in cl if max(c["ext"]) >= BSK_MIN_EXT] or cl
    if not cand:
        api.log("BASKET fallback: anchor (no clusters)")
        return ANCHOR_RELEASE_XY.copy(), None
    b = min(cand, key=lambda c: float(np.linalg.norm(c["box"] - ANCHOR_RELEASE_XY)))
    d = float(np.linalg.norm(b["box"] - ANCHOR_RELEASE_XY))
    api.log("BASKET box=(%.4f,%.4f) top=%.4f n=%d d=%.4f" %
            (b["box"][0], b["box"][1], b["top"], b["n"], d))
    if d > BSK_CLAMP:
        api.log("BASKET clamped to anchor (d=%.3f > %.3f)" % (d, BSK_CLAMP))
        return ANCHOR_RELEASE_XY.copy(), b
    return b["box"].copy(), b


def holding(api):
    try:
        g = api.gripper()
        return float(g["effort"]) >= HOLD_EFFORT
    except Exception:
        return False


# ---------------------------------------------------------------- program
def run(api):
    api.log("instruction: %s" % api.instruction())
    api.log("eef0=%s grip0=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))

    api.grip(0.08)
    api.settle(0.2)

    cl = perceive(api)
    gxy, _ = pick_target(api, cl)
    bxy, bcl = pick_basket(api, cl)

    # ------------------------------------------------ pick (with regrasp loop)
    for attempt in range(3):
        if api.done:
            return
        gz = ANCHOR_GRASP_Z - 0.008 * attempt
        api.move([gxy[0], gxy[1], CARRY_Z], seconds=2.0)
        api.grip(0.08)
        api.move([gxy[0], gxy[1], gz + 0.05], seconds=1.5)
        api.move([gxy[0], gxy[1], gz], seconds=1.5)
        api.log("a%d at grasp eef=%s" % (attempt, np.round(api.eef(), 4).tolist()))
        api.grip(0.0)
        api.settle(0.4)
        api.log("a%d after close grip=%s" % (attempt, api.gripper()))
        api.move([gxy[0], gxy[1], CARRY_Z], seconds=2.0)
        api.settle(0.2)
        api.log("a%d after lift grip=%s eef=%s" %
                (attempt, api.gripper(), np.round(api.eef(), 4).tolist()))
        if holding(api):
            break
        api.log("a%d NOT holding -- re-perceiving" % attempt)
        api.grip(0.08)
        api.settle(0.2)
        cl = perceive(api, tag="r%d " % attempt)
        gxy, _ = pick_target(api, cl)
        bxy2, _ = pick_basket(api, cl)
        bxy = bxy2

    # ------------------------------------------------------------- place
    api.move([bxy[0], bxy[1], CARRY_Z], seconds=2.5)
    api.log("over basket eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.move([bxy[0], bxy[1], ANCHOR_RELEASE_Z], seconds=2.0)
    api.log("at release eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(0.08)
    api.settle(0.5)
    api.move([bxy[0], bxy[1], CARRY_Z], seconds=1.5)
    api.settle(0.3)
    api.log("done eef=%s done=%s" % (np.round(api.eef(), 4).tolist(), api.done))
