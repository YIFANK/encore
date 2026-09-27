"""c2 cell obj_butter_stock -- v1.1 (de-oracled v3).

Intent: "pick up the butter and place it in the basket".

Nominal trajectory is bit-identical to v2/v3 (15/15 on debug seeds 51-65).
Guards, all no-ops when the nominal path works:
  (A) colour tie-break when two small blobs are both near the pack anchor,
  (B) close-and-verify retry if the gripper reports no load after closing,
  (C) slip detection after the lift -> one full re-pick,
  (D) basket-shape sanity check with a pack-anchor fallback.

fair-v1.1 de-oracle pass: v3 gated guards (B) and (C) on `and not api.done`,
which under protocol v1.0 leaked runtime task success.  Both `api.done` reads
are EXCISED; each guard now fires on its own gripper-effort sensor condition
alone (`effort < HOLD_EFFORT`), which was already the primary term.  No other
change: no constant altered, no new constant introduced, no restructuring, and
the nominal (guard-free) path is byte-identical to v3.  The program now reads
no success signal of any kind.

Everything is derived from packs/c2_obj_butter_stock/pack.json (K=3 demos),
from debug-seed measurements logged by v2, and from generic camera/controller
mechanics.  No benchmark asset is opened.
"""

import numpy as np

PROVENANCE = {
    "TABLE_Z": {
        "source": "generic: estimated at runtime as the median deprojected z of the "
                  "in-bounds cam_high point cloud (no constant baked in); measured "
                  "0.0013-0.0014 on debug seeds 51-57",
        "allowed": True},
    "OBJ_Z_MIN": {
        "source": "generic segmentation margin above the estimated table plane",
        "allowed": True},
    "OBJ_Z_MAX": {
        "source": "pack.json demos[*].ee_path6: manipulated object tops lie below the "
                  "carry height 0.28; 0.22 excludes the arm from the point cloud",
        "allowed": True},
    "WS_X": {"source": "pack.json demos[*].ee_path6 x range (-0.166..0.071), padded",
             "allowed": True},
    "WS_Y": {"source": "pack.json demos[*].ee_path6 y range (-0.266..0.287), padded",
             "allowed": True},
    "BUTTER_ANCHOR_XY": {
        "source": "pack.json demos[*].keyframes closing frame ee xy = "
                  "(-0.1208,-0.2543), (-0.1408,-0.2663), (-0.1143,-0.2641); mean",
        "allowed": True},
    "BUTTER_MAX_H": {
        "source": "debug seeds 51-57 (fs_c2_obj_butter_stock_v2 logs): the grasped "
                  "blob measured h=0.018, ext=[0.075,0.039]; 0.12/0.14 are loose "
                  "upper gates that only exclude tall/large scene furniture",
        "allowed": True},
    "ORANGE_MIN": {
        "source": "debug seeds 51-57 logs: butter blob mean RGB ~[88,54,38] (R-B~50) "
                  "vs the nearest box-shaped distractor ~[73,58,52] (R-B~21); "
                  "used only as a tie-break inside AMBIG_R",
        "allowed": True},
    "AMBIG_R": {
        "source": "debug seeds 51-65: butter sat 0.022-0.026 m from the pack anchor "
                  "while the nearest other blob sat ~0.24 m away; 0.07 is the radius "
                  "inside which two candidates are treated as ambiguous",
        "allowed": True},
    "GRASP_Z": {
        "source": "pack.json demos[*].keyframes closing frame ee z = "
                  "0.0095, 0.0100, 0.0111; mean 0.0102",
        "allowed": True},
    "APPROACH_Z": {
        "source": "pack.json demos[*].ee_path6: pre-grasp hover height ~0.145-0.19",
        "allowed": True},
    "CARRY_Z": {
        "source": "pack.json demos[*].ee_path6 transport plateau z ~0.27-0.29",
        "allowed": True},
    "RELEASE_Z": {
        "source": "pack.json demos[*].keyframes release frame ee z = "
                  "0.1909, 0.1595, 0.1561; mean 0.169",
        "allowed": True},
    "BASKET_ANCHOR_XY": {
        "source": "pack.json demos[*].keyframes release frame ee xy = "
                  "(0.0234,0.2500), (-0.0267,0.2608), (0.0040,0.2697); mean",
        "allowed": True},
    "BASKET_MIN_EXT": {
        "source": "debug seeds 51-57 logs: the basket blob measured ext ~[0.16,0.17], "
                  "h~0.142; 0.10 is a loose lower gate",
        "allowed": True},
    "GRIP_CLOSE_M": {
        "source": "FairApi doc: api.grip(w) with w<0.025 closes the gripper",
        "allowed": True},
    "GRIP_OPEN_M": {
        "source": "FairApi doc: api.grip(w) with w>=0.025 opens; demo release "
                  "gripper_state ~0.039 per finger",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi doc: api.gripper()['effort'] is 3.0 iff holding; measured "
                  "3.0 while carrying and 0.05 when empty on debug seeds 51-57",
        "allowed": True},
    "REGRASP_DZ": {
        "source": "debug seeds 51-57: commanded grasp z 0.0102 settled at eef z "
                  "0.0174, i.e. the controller stops ~7 mm high; a retry biases "
                  "5 mm lower",
        "allowed": True},
    "CLUSTER_CELL_M": {"source": "generic occupancy-grid clustering resolution",
                       "allowed": True},
}

WS_X = (-0.32, 0.28)
WS_Y = (-0.42, 0.44)
OBJ_Z_MIN = 0.013
OBJ_Z_MAX = 0.22
CLUSTER_CELL_M = 0.014
MIN_CLUSTER_PTS = 25

BUTTER_ANCHOR_XY = (-0.1253, -0.2616)
BASKET_ANCHOR_XY = (0.0002, 0.2602)
BUTTER_MAX_H = 0.12
BUTTER_MAX_EXT = 0.14
ORANGE_MIN = 35.0
AMBIG_R = 0.07
BASKET_MIN_EXT = 0.10
GRASP_Z = 0.0102
APPROACH_Z = 0.16
CARRY_Z = 0.28
RELEASE_Z = 0.175
GRIP_CLOSE_M = 0.0
GRIP_OPEN_M = 0.08
HOLD_EFFORT = 2.0
REGRASP_DZ = 0.005


# --------------------------------------------------------------------------
# generic camera mechanics: vectorised pinhole deprojection into the base frame

def cloud(frame):
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    z = d
    px = (uu - cx) * z / fx
    py = (vv - cy) * z / fy
    P = np.stack([px, py, z, np.ones_like(z)], axis=-1)
    B = P.reshape(-1, 4) @ T.T
    ok = np.isfinite(d).ravel() & (d.ravel() > 1e-4)
    return B[:, :3], ok, (h, w)


def grid_clusters(xy, cell, min_pts):
    """Occupancy-grid connected components in the table plane (pure numpy)."""
    keys = np.floor(xy / cell).astype(np.int64)
    uniq, inv, cnt = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    index = {(int(a), int(b)): i for i, (a, b) in enumerate(uniq)}
    seen = np.zeros(len(uniq), bool)
    out = []
    for start in range(len(uniq)):
        if seen[start]:
            continue
        seen[start] = True
        stack, comp = [start], [start]
        while stack:
            c = stack.pop()
            a, b = uniq[c]
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    j = index.get((int(a) + da, int(b) + db))
                    if j is not None and not seen[j]:
                        seen[j] = True
                        stack.append(j)
                        comp.append(j)
        m = np.isin(inv, np.asarray(comp))
        if int(m.sum()) >= min_pts:
            out.append(m)
    return out


def scene(api, cam="cam_high"):
    f = api.capture(cam)
    rgb = np.asarray(f.rgb).reshape(-1, 3).astype(float)
    P, ok, _ = cloud(f)
    inb = (ok
           & (P[:, 0] > WS_X[0]) & (P[:, 0] < WS_X[1])
           & (P[:, 1] > WS_Y[0]) & (P[:, 1] < WS_Y[1]))
    if inb.sum() < 500:
        return []
    table_z = float(np.median(P[inb, 2]))
    m = inb & (P[:, 2] > table_z + OBJ_Z_MIN) & (P[:, 2] < table_z + OBJ_Z_MAX)
    idx = np.nonzero(m)[0]
    if idx.size < MIN_CLUSTER_PTS:
        return []
    pts = P[idx]
    cols = rgb[idx]
    blobs = []
    for sel in grid_clusters(pts[:, :2], CLUSTER_CELL_M, MIN_CLUSTER_PTS):
        q = pts[sel]
        c = cols[sel]
        top = q[:, 2].max()
        face = q[q[:, 2] > top - 0.012]
        mean_rgb = c.mean(axis=0)
        blobs.append({
            "n": int(sel.sum()),
            "xy": [float(face[:, 0].mean()), float(face[:, 1].mean())],
            "cxy": [float(q[:, 0].mean()), float(q[:, 1].mean())],
            "top": float(top),
            "h": float(top - table_z),
            "ext": [float(np.ptp(q[:, 0])), float(np.ptp(q[:, 1]))],
            "rgb": [round(float(v), 1) for v in mean_rgb],
            "orange": float(mean_rgb[0] - mean_rgb[2]),
            "table_z": round(table_z, 4),
        })
    blobs.sort(key=lambda b: -b["n"])
    return blobs


def dist(b, anchor):
    return float(np.hypot(b["xy"][0] - anchor[0], b["xy"][1] - anchor[1]))


def find_butter(api, blobs):
    """Nearest small blob to the pack anchor; colour breaks near-ties (guard A)."""
    cand = [b for b in blobs
            if b["h"] < BUTTER_MAX_H and max(b["ext"]) < BUTTER_MAX_EXT]
    if not cand:
        return None
    cand.sort(key=lambda b: dist(b, BUTTER_ANCHOR_XY))
    best = cand[0]
    near = [b for b in cand if dist(b, BUTTER_ANCHOR_XY) <= dist(best, BUTTER_ANCHOR_XY) + AMBIG_R]
    if len(near) > 1:
        orange = [b for b in near if b["orange"] >= ORANGE_MIN]
        api.log("ambiguous: %d near candidates, %d orange"
                % (len(near), len(orange)))
        if len(orange) == 1:
            best = orange[0]
        elif orange:
            orange.sort(key=lambda b: dist(b, BUTTER_ANCHOR_XY))
            best = orange[0]
    return best


def find_basket(api, blobs):
    """Big blob on the +y side; falls back to the pack anchor (guard D)."""
    cand = [b for b in blobs
            if b["xy"][1] > 0.10 and max(b["ext"]) > BASKET_MIN_EXT and b["h"] > 0.08]
    if not cand:
        return None
    cand.sort(key=lambda b: dist(b, BASKET_ANCHOR_XY))
    return cand[0]


def close_and_check(api, bx, by, z):
    api.move([bx, by, z], seconds=2.0)
    api.log("down eef=%s" % np.round(api.eef(), 4).tolist())
    api.grip(GRIP_CLOSE_M)
    api.settle(0.5)
    g = api.gripper()
    api.log("closed %s" % g)
    return g["effort"] >= HOLD_EFFORT


def attempt_pick(api, blobs):
    """Returns (holding, xy_used) -- nominal path identical to v2."""
    butter = find_butter(api, blobs)
    if butter is None:
        return False, None
    bx, by = butter["xy"]
    api.log("butter=%s d=%.3f h=%.3f ext=%s rgb=%s"
            % ([round(v, 4) for v in butter["xy"]], dist(butter, BUTTER_ANCHOR_XY),
               butter["h"], [round(v, 3) for v in butter["ext"]], butter["rgb"]))

    api.move([bx, by, APPROACH_Z], seconds=2.5)
    api.log("hover eef=%s" % np.round(api.eef(), 4).tolist())
    ok = close_and_check(api, bx, by, GRASP_Z)
    if not ok:                                       # guard B (sensor-only)
        api.log("regrasp: no load after close")
        api.grip(GRIP_OPEN_M)
        api.move([bx, by, APPROACH_Z], seconds=1.2)
        ok = close_and_check(api, bx, by, GRASP_Z - REGRASP_DZ)
    return ok, (bx, by)


def run(api):
    api.log("instruction=%r cams=%r" % (api.instruction(), api.cameras))
    api.log("eef0=%s grip0=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.log("R0=%s" % np.round(api.tool_rotation(), 3).tolist())

    api.grip(GRIP_OPEN_M)
    api.settle(0.3)

    blobs = scene(api)
    for i, b in enumerate(blobs):
        api.log("blob%d n=%d xy=%s top=%.4f h=%.3f ext=%s rgb=%s or=%.1f tz=%s"
                % (i, b["n"], [round(v, 4) for v in b["xy"]], b["top"], b["h"],
                   [round(v, 3) for v in b["ext"]], b["rgb"], b["orange"],
                   b["table_z"]))
    if not blobs:
        return "no blobs"

    basket = find_basket(api, blobs)
    if basket is None:
        px, py = BASKET_ANCHOR_XY
        api.log("basket: fallback to pack anchor")
    else:
        px, py = basket["cxy"]
        api.log("basket=%s d=%.3f top=%.3f ext=%s"
                % ([round(v, 4) for v in basket["cxy"]], dist(basket, BASKET_ANCHOR_XY),
                   basket["top"], [round(v, 3) for v in basket["ext"]]))

    holding, xy = attempt_pick(api, blobs)
    if xy is None:
        return "no butter candidate"
    bx, by = xy

    api.move([bx, by, CARRY_Z], seconds=2.0)
    api.settle(0.3)
    g = api.gripper()
    api.log("lifted eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), g))

    if g["effort"] < HOLD_EFFORT:                    # guard C: dropped / missed
        api.log("slip after lift -> re-perceive and re-pick")
        api.grip(GRIP_OPEN_M)
        api.settle(0.3)
        blobs2 = scene(api)
        for i, b in enumerate(blobs2[:8]):
            api.log("re-blob%d n=%d xy=%s h=%.3f or=%.1f"
                    % (i, b["n"], [round(v, 4) for v in b["xy"]], b["h"], b["orange"]))
        holding, xy2 = attempt_pick(api, blobs2)
        if xy2 is not None:
            bx, by = xy2
        api.move([bx, by, CARRY_Z], seconds=2.0)
        api.settle(0.3)
        g = api.gripper()
        api.log("re-lifted eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), g))

    api.move([px, py, CARRY_Z], seconds=3.0)
    api.log("over-basket eef=%s grip=%s"
            % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.move([px, py, RELEASE_Z], seconds=2.0)
    api.grip(GRIP_OPEN_M)
    api.settle(0.8)
    api.log("released eef=%s grip=%s"
            % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.move([px, py, CARRY_Z], seconds=1.5)
    api.settle(0.5)

    post = scene(api)
    for i, b in enumerate(post[:6]):
        api.log("post%d n=%d xy=%s top=%.4f rgb=%s"
                % (i, b["n"], [round(v, 4) for v in b["xy"]], b["top"], b["rgb"]))
    return "v1.1 finished holding=%s" % holding
