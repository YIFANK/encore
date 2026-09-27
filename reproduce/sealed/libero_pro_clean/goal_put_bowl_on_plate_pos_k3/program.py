"""c2clean / goal_put_bowl_on_plate_pos_k3 -- v6.

put the bowl on the plate.

Perception (cam_high RGB-D -> base-frame cloud):
  * both vessels are found as a RING of known radius: an annulus vote on a 4 mm
    grid whose interior must be empty inside the searched height window.
  * the window is SWEPT, because the plate stands 0.920-0.935 depending on the
    seed, and candidates are ranked by angular completeness -- on the seeds
    where the cabinet occludes a third of the plate the true rim still covers
    more bins than any phantom arc.
  * a plate candidate must also have a FLOOR: points inside the rim, below it,
    above the table. Phantom arcs (stove edges, cabinet corners) have none.

Policy: rim-pinch the bowl one rim radius along the jaw axis (pack demos),
carry that offset to the plate centre, release just above the plate rim.
"""

import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-65 cam_high depth: dominant z mode of the "
                          "workspace cloud (0.900); refitted live every episode",
                "allowed": True},
    "BOWL_R0": {"source": "debug seeds 51-65 (11 dumped): circle fit of the bowl rim ring "
                          "gives r=0.0528 m on every seed", "allowed": True},
    "PLATE_R0": {"source": "debug seeds 51-65 (11 dumped): circle fit of the plate rim ring "
                           "gives r=0.063-0.066 m on every seed", "allowed": True},
    "BOWL_BAND": {"source": "debug seeds: the bowl rim tops out at z=0.951-0.952 "
                            "(table+0.051); window table+0.040..0.075", "allowed": True},
    "PLATE_BAND": {"source": "debug seeds: the plate rim tops out at 0.920 on most seeds but "
                             "0.927-0.935 on 52/54/56/64, so a 17 mm window is swept over "
                             "table+0.008..table+0.032", "allowed": True},
    "RING_TOL": {"source": "debug-seed depth noise on the two rim rings (~5 mm)",
                 "allowed": True},
    "FIT_RMS_MAX": {"source": "debug seeds: radial RMS of the true rim inliers is 0.002-0.0045",
                    "allowed": True},
    "FLOOR_MIN": {"source": "debug seeds 51/54/64: the plate's inner floor holds 700-1000 "
                            "points in the window (table+0.003, rim-0.005); every phantom "
                            "arc holds exactly 0", "allowed": True},
    "GRASP_DZ": {"source": "pack demos: the close (gripper_cmd -> +1) happens at eef z "
                           "0.9252/0.9179/0.9199 against a rim top measured at 0.951 -> the "
                           "eef reference sits 0.032 below the rim top at the grasp",
                 "allowed": True},
    "RIM_OFFSET": {"source": "pack demo0: grasp eef xy (-0.1027,0.0301) vs the demo bowl "
                             "centre (-0.0755,-0.0096), recovered by ray-casting the demo "
                             "keyframe's bowl pixel onto the rim plane -> 0.048 m radial, "
                             "dominant along +y, which is the jaw-closing axis of the home "
                             "wrist (tool y = -base y in api.tool_rotation())",
                   "allowed": True},
    "RELEASE_DZ": {"source": "pack demos: release (gripper_cmd -> -1) at eef z "
                             "0.9359/0.928/0.9309 above a plate rim of 0.920, i.e. "
                             "rim+0.012; +0.024 here keeps the same clearance after the "
                             "0.032/0.020 grasp geometry", "allowed": True},
    "CARRY_Z": {"source": "pack demo ee_path6 transport z 1.018/1.032/1.028", "allowed": True},
    "HOVER_DZ": {"source": "generic approach clearance above the perceived rim", "allowed": True},
    "OPEN_W": {"source": "debug seed: api.gripper() reads width_m 0.0778 at the open home pose",
               "allowed": True},
}

BOWL_R0 = 0.0528
PLATE_R0 = 0.0648
RING_TOL = 0.008
FIT_RMS_MAX = 0.006
FLOOR_MIN = 200
GRASP_DZ = 0.032
RIM_OFFSET = 0.048
RELEASE_DZ = 0.024
CARRY_Z = 1.025
HOVER_DZ = 0.10
OPEN_W = 0.078
CELL = 0.004
XLIM = (-0.35, 0.45)
YLIM = (-0.55, 0.55)


# --------------------------------------------------------------------------- perception

def _cloud(frame):
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 0)
    z = np.where(ok, d, 1.0)
    P = np.stack([(uu - K[0, 2]) * z / K[0, 0],
                  (vv - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], -1)
    X = (P @ T.T)[..., :3]
    X[~ok] = np.nan
    return X


def _corr(H, kern, k):
    nx, ny = H.shape
    acc = np.zeros_like(H)
    for a, b in np.argwhere(kern > 0):
        oi, oj = int(a) - k, int(b) - k
        acc[max(0, -oi):nx - max(0, oi), max(0, -oj):ny - max(0, oj)] += \
            H[max(0, oi):nx + min(0, oi), max(0, oj):ny + min(0, oj)]
    return acc


def find_rings(xs, ys, r0, min_bins, min_n=200, topk=30):
    """All hollow rings of radius ~r0 in this height slice, best peaks first."""
    hits = []
    if xs.size < min_n:
        return hits
    nx = int((XLIM[1] - XLIM[0]) / CELL) + 1
    ny = int((YLIM[1] - YLIM[0]) / CELL) + 1
    gx = np.clip(((xs - XLIM[0]) / CELL).astype(int), 0, nx - 1)
    gy = np.clip(((ys - YLIM[0]) / CELL).astype(int), 0, ny - 1)
    H = np.zeros((nx, ny), np.float32)
    np.add.at(H, (gx, gy), 1.0)
    k = int(np.ceil((r0 + RING_TOL) / CELL))
    di, dj = np.mgrid[-k:k + 1, -k:k + 1]
    d = np.hypot(di, dj) * CELL
    ring = _corr(H, (np.abs(d - r0) <= RING_TOL).astype(np.float32), k)
    inner = _corr(H, (d <= r0 - 0.018).astype(np.float32), k)
    score = np.where(inner <= 0.03 * np.maximum(ring, 1.0), ring, 0.0)
    for _ in range(topk):
        bi, bj = np.unravel_index(int(np.argmax(score)), score.shape)
        if score[bi, bj] <= 0:
            break
        cx, cy = XLIM[0] + bi * CELL, YLIM[0] + bj * CELL
        score[max(0, bi - 8):bi + 9, max(0, bj - 8):bj + 9] = 0.0
        on = np.abs(np.hypot(xs - cx, ys - cy) - r0) <= RING_TOL
        if on.sum() < min_n:
            continue
        th = np.arctan2(ys[on] - cy, xs[on] - cx)
        bins = len(np.unique(((th + np.pi) / (np.pi / 18)).astype(int)))
        if bins < min_bins:
            continue
        A = np.stack([xs[on], ys[on], np.ones(int(on.sum()))], 1)
        b2 = xs[on] ** 2 + ys[on] ** 2
        sol, *_ = np.linalg.lstsq(A, b2, rcond=None)
        cx2, cy2 = float(sol[0]) / 2.0, float(sol[1]) / 2.0
        r2 = float(np.sqrt(max(sol[2] + cx2 ** 2 + cy2 ** 2, 1e-9)))
        rms = float(np.sqrt(np.mean((np.hypot(xs[on] - cx2, ys[on] - cy2) - r2) ** 2)))
        if abs(r2 - r0) > 0.006 or rms > FIT_RMS_MAX:
            continue
        hits.append({"cx": cx2, "cy": cy2, "r": r2, "n": int(on.sum()),
                     "bins": bins, "rms": rms})
    return hits


def _sweep(X, Y, Z, ws, table, r0, lo, hi, min_bins, step=0.004, thick=0.017,
           avoid=None, need_floor=False):
    out = []
    z0 = lo
    while z0 <= hi + 1e-9:
        m = ws & (Z > table + z0) & (Z < table + z0 + thick)
        xs, ys, zs = X[m], Y[m], Z[m]
        for h in find_rings(xs, ys, r0, min_bins):
            if avoid is not None and np.hypot(h["cx"] - avoid["cx"],
                                              h["cy"] - avoid["cy"]) < 0.04:
                continue
            on = np.abs(np.hypot(xs - h["cx"], ys - h["cy"]) - r0) <= RING_TOL
            h["ztop"] = float(np.percentile(zs[on], 90.0))
            h["band"] = round(table + z0, 4)
            if need_floor:
                rad = np.hypot(X - h["cx"], Y - h["cy"])
                fl = (np.isfinite(Z) & (rad < h["r"] - 0.024)
                      & (Z > table + 0.003) & (Z < h["ztop"] - 0.005))
                h["floor"] = int(fl.sum())
                if h["floor"] < FLOOR_MIN:
                    continue
            out.append(h)
        z0 += step
    if not out:
        return None
    out.sort(key=lambda h: (h["bins"], h["n"]), reverse=True)
    return out[0]


def perceive(api, tag=""):
    f = api.capture("cam_high")
    P = _cloud(f)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    ws = (np.isfinite(Z) & (X > XLIM[0]) & (X < XLIM[1])
          & (Y > YLIM[0]) & (Y < YLIM[1]) & (Z > 0.80) & (Z < 1.30))
    hist, edges = np.histogram(Z[ws], bins=100, range=(0.80, 1.30))
    table = float(edges[int(np.argmax(hist))] + 0.0025)

    bowl = _sweep(X, Y, Z, ws, table, BOWL_R0, 0.040, 0.040, 30, thick=0.035)
    if bowl is None:
        bowl = _sweep(X, Y, Z, ws, table, BOWL_R0, 0.030, 0.056, 24)
    plate = _sweep(X, Y, Z, ws, table, PLATE_R0, 0.008, 0.032, 16,
                   avoid=bowl, need_floor=True)
    api.log("%sPERC table=%.4f bowl=%s" % (tag, table, bowl))
    api.log("%sPERC plate=%s" % (tag, plate))
    return {"table": table, "bowl": bowl, "plate": plate}


# --------------------------------------------------------------------------- policy

def run(api):
    api.log("instruction=%r" % api.instruction())
    s = perceive(api)
    bowl, plate = s["bowl"], s["plate"]
    if bowl is None or plate is None:
        api.log("ABORT bowl=%s plate=%s" % (bowl, plate))
        return "perception failed"

    off = np.array([0.0, RIM_OFFSET])          # jaws close along base y
    gx, gy = bowl["cx"] + off[0], bowl["cy"] + off[1]
    grasp_z = bowl["ztop"] - GRASP_DZ
    api.log("GRASP (%.4f,%.4f,%.4f)" % (gx, gy, grasp_z))

    api.grip(OPEN_W)
    api.move([gx, gy, bowl["ztop"] + HOVER_DZ], seconds=2.0)
    api.move([gx, gy, grasp_z], seconds=2.0)
    api.log("at_grasp eef=%s g=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(0.0)
    api.settle(0.4)
    api.log("closed g=%s" % api.gripper())

    api.move([gx, gy, CARRY_Z], seconds=2.0)
    api.settle(0.3)
    api.log("lifted eef=%s g=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))

    px, py = plate["cx"] + off[0], plate["cy"] + off[1]
    api.move([px, py, CARRY_Z], seconds=2.5)
    api.log("over_plate eef=%s" % np.round(api.eef(), 4).tolist())
    rel_z = plate["ztop"] + RELEASE_DZ
    api.move([px, py, rel_z], seconds=2.0)
    api.log("at_release eef=%s g=%s rel_z=%.4f"
            % (np.round(api.eef(), 4).tolist(), api.gripper(), rel_z))
    api.grip(OPEN_W)
    api.settle(0.5)
    api.move([px, py, CARRY_Z], seconds=1.5)
    api.settle(0.5)

    perceive(api, tag="POST ")
    return "v6 done"
