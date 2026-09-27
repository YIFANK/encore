"""ENCORE abl_c2 / cell ablC_goal_put_bowl_on_stove  (variant C: no verification).

Intent: "put the bowl on the stove".

One frozen version, written from packs/ablC_goal_put_bowl_on_stove/ alone (K=3
demos: keyframe EEF/gripper states, ee_path6, action_scale, 128x128 keyframe
renders).  Zero episodes were executed while writing it.

Mechanism
---------
1.  One cam_high RGB-D capture at the start (arm is parked high, scene clear).
    A base-frame point cloud is built vectorised from .intrinsics/.t_base_cam
    and *validated* against api.deproject on sample pixels; if no camera
    convention reproduces api.deproject the cloud falls back to a coarse grid
    of api.deproject calls.
2.  Table height = modal z of the cloud inside the plausible band implied by
    the pack's grasp height.  Objects = points above it.
3.  Bowl = the elevated cluster in the pack-implied bowl region that best
    matches a bowl signature (diameter, height above table, annulus/ring
    occupancy, low colour saturation) and is closest to the pack prior.
    Its centre is the bounding-box centre of the silhouette, its rim radius
    half the silhouette diameter.
4.  Stove = the large bright low-saturation elevated slab on the +y side;
    the burner target is the dark disc on it (bounding-box centre of the slab
    is used when the disc is not separable).
5.  The pack's hold widths (6.3 mm and 12.3 mm between fingers, against a
    78.6 mm open width) show the demos did NOT cage the ~0.12 m bowl - they
    pinched the RIM WALL, one finger inside the bowl, one outside.  So the
    grasp point is bowl_centre + r * d with d the gripper's finger-closing
    axis.  d is measured in-episode, fairly, from proprio+perception: the
    wrist camera is diffed open-vs-closed and the two finger blobs are
    deprojected, giving the closing axis in the base frame (PCA sign
    ambiguity is harmless - a symmetric rim admits either sign).  If that
    measurement is not confident the axis defaults to base +y and the
    alternative axis is simply the next retry candidate.
6.  Grasp attempts are gated on the gripper's OWN sensors (width inside the
    pack's hold band, effort, and survival of a lift) - never on task
    feedback.  Up to three attempts, varying axis then depth, re-perceiving
    between attempts.
7.  The held offset (grasp_xy - bowl_centre_xy) is carried through to the
    release, so the bowl - not the wrist - is centred over the burner.
    Release height is the pack's grasp height shifted by the measured
    (stove_top - table) rise, i.e. the bowl is set down, not dropped.
"""

import numpy as np

# --------------------------------------------------------------------------
# Every calibrated constant, with its source.
# --------------------------------------------------------------------------
PROVENANCE = {
    "Z_GRASP": {
        "source": "pack.json keyframes at the gripper-close command: demo0 t39 "
                  "ee.z=0.9325, demo1 t37 ee.z=0.9312, demo2 t43 ee.z=0.9235; "
                  "ee_path minima 0.9269/0.9204/0.9214 -> 0.925",
        "allowed": True},
    "Z_GRASP_DEEP": {
        "source": "pack.json ee_path descent minima (0.9204 lowest of 3 demos); "
                  "retry depth 0.916 = just under the deepest demo contact",
        "allowed": True},
    "Z_APPROACH": {
        "source": "pack.json ee_path pre-descent waypoint z: 1.0237 / 1.0047 / "
                  "1.0430 -> 1.02",
        "allowed": True},
    "Z_TRANSPORT": {
        "source": "pack.json ee_path carry-phase maxima: 1.0526 / 1.0838 / "
                  "1.0403 -> 1.06",
        "allowed": True},
    "Z_RELEASE_PRIOR": {
        "source": "pack.json keyframes at the gripper-open command: demo0 t87 "
                  "ee.z=0.9451, demo1 t91 0.9721, demo2 t95 0.9485 -> 0.955",
        "allowed": True},
    "BOWL_PRIOR_XY": {
        "source": "pack.json mean ee (x,y) at the gripper-close keyframes: "
                  "(-0.0973,0.0563), (-0.1086,0.0460), (-0.1319,0.0390) "
                  "-> (-0.1126, 0.0471)",
        "allowed": True},
    "STOVE_PRIOR_XY": {
        "source": "pack.json mean ee (x,y) at the gripper-open keyframes: "
                  "(-0.2729,0.2605), (-0.2516,0.2599), (-0.2453,0.2464) "
                  "-> (-0.2566, 0.2556)",
        "allowed": True},
    "RIM_RADIUS_PRIOR": {
        "source": "pack keyframes/*_t0000.png: bowl silhouette is 16 px wide at "
                  "128 px render; image scale 128 px/m recovered from the pack "
                  "itself (u(stove blob)-u(bowl blob) vs ee dy between the "
                  "close and open keyframes, 125.8/131.5/126.5 px/m across the "
                  "3 demos) -> 0.125 m across, radius 0.0625",
        "allowed": True},
    "RIM_RADIUS_CLAMP": {
        "source": "same measurement, +/-20% tolerance for per-layout "
                  "perception noise -> [0.048, 0.075]",
        "allowed": True},
    "HOLD_WIDTH_BAND": {
        "source": "pack.json gripper_state while carrying: demo0 "
                  "0.0092-(-0.0031)=0.0123, demo1 0.0023-(-0.0040)=0.0063, "
                  "demo2 0.0091-(-0.0032)=0.0123 -> accept [0.003, 0.030]",
        "allowed": True},
    "OPEN_WIDTH": {
        "source": "pack.json gripper_state fully open 0.0393-(-0.0393)=0.0786",
        "allowed": True},
    "TABLE_Z_SEARCH_BAND": {
        "source": "pack.json: the bowl rests on the table and is grasped at "
                  "ee z ~0.925, and released 0.03 higher onto the stove, so "
                  "the table top lies in [0.75, 0.95]",
        "allowed": True},
    "BOWL_REGION": {
        "source": "pack.json ee (x,y) at the close keyframes spans x in "
                  "[-0.132,-0.097], y in [0.039,0.056]; widened generously for "
                  "unseen layouts to x in [-0.30,0.10], y in [-0.10,0.17]",
        "allowed": True},
    "STOVE_REGION": {
        "source": "pack.json ee (x,y) at the open keyframes spans x in "
                  "[-0.273,-0.245], y in [0.246,0.261]; the stove slab is the "
                  "bright elevated body at y > 0.10",
        "allowed": True},
    "BOWL_SIGNATURE": {
        "source": "pack keyframes/*_t0000.png + pack hold widths: a ~0.125 m "
                  "wide, ~0.05 m tall open ring; scored with sigma 0.05 m on "
                  "diameter and 0.035 m on height",
        "allowed": True},
    "WORKSPACE_BOX": {
        "source": "pack.json ee_path6 across all 3 demos spans x in "
                  "[-0.273,-0.052], y in [-0.015,0.263], z in [0.920,1.176]; "
                  "padded for perception -> generic workspace bound",
        "allowed": True},
    "MOVE_SECONDS": {
        "source": "generic controller mechanics (position moves sized to the "
                  "travel distance); pack demos are 94-110 control steps long, "
                  "so per-move durations are kept short",
        "allowed": True},
    "CAMERA_MODEL": {
        "source": "generic pinhole camera mechanics; the chosen axis "
                  "convention is validated in-episode against api.deproject "
                  "and discarded if it disagrees by more than 0.02 m",
        "allowed": True},
    "MAX_ATTEMPTS": {
        "source": "budget bound from generic controller mechanics: pack demos "
                  "complete in 94-110 control steps, so the retry search is "
                  "capped at 3 grasp attempts to stay inside a comparable "
                  "episode horizon",
        "allowed": True},
    "FINGER_AXIS_DEFAULT": {
        "source": "pack keyframes at the grasp: the open hand's silhouette is "
                  "elongated along image u, and image u tracks base +y (the "
                  "125-132 px/m fit above), so the closing axis defaults to "
                  "base +y; measured in-episode from the wrist camera when "
                  "that measurement is confident",
        "allowed": True},
}

# ---- constants (see PROVENANCE) -------------------------------------------
Z_GRASP = 0.925
Z_GRASP_DEEP = 0.916
Z_APPROACH = 1.02
Z_TRANSPORT = 1.06
Z_RELEASE_PRIOR = 0.955
BOWL_PRIOR_XY = (-0.1126, 0.0471)
STOVE_PRIOR_XY = (-0.2566, 0.2556)
RIM_RADIUS_PRIOR = 0.0625
RIM_RADIUS_CLAMP = (0.048, 0.075)
HOLD_WIDTH_BAND = (0.003, 0.030)
OPEN_WIDTH = 0.0786
TABLE_Z_SEARCH_BAND = (0.75, 0.95)
BOWL_REGION = (-0.30, 0.10, -0.10, 0.17)      # xmin, xmax, ymin, ymax
STOVE_REGION = (-0.45, 0.05, 0.10, 0.45)
WORKSPACE_BOX = (-0.55, 0.30, -0.45, 0.55)
MAX_ATTEMPTS = 3


# ==========================================================================
# small helpers
# ==========================================================================
def _log(api, msg):
    try:
        api.log(str(msg))
    except Exception:
        pass


def _arr(x):
    return np.asarray(x, dtype=float)


def _label_grid(occ):
    """8-connected components of a 2-D boolean grid (no scipy dependency)."""
    h, w = occ.shape
    lab = np.zeros((h, w), dtype=np.int32)
    cur = 0
    for i0 in range(h):
        for j0 in range(w):
            if not occ[i0, j0] or lab[i0, j0]:
                continue
            cur += 1
            stack = [(i0, j0)]
            lab[i0, j0] = cur
            while stack:
                i, j = stack.pop()
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        a, b = i + di, j + dj
                        if 0 <= a < h and 0 <= b < w and occ[a, b] and not lab[a, b]:
                            lab[a, b] = cur
                            stack.append((a, b))
    return lab, cur


# ==========================================================================
# point cloud
# ==========================================================================
def _cloud(api, frame, stride=3):
    """(pts_base Nx3, rgb Nx3) from a FairFrame, validated against deproject."""
    depth = _arr(frame.depth)
    rgb = np.asarray(frame.rgb)
    h, w = depth.shape[:2]
    K = _arr(frame.intrinsics)
    T = _arr(frame.t_base_cam)
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])

    vv, uu = np.mgrid[0:h:stride, 0:w:stride]
    d = depth[::stride, ::stride].astype(float)
    col = rgb[::stride, ::stride, :3].astype(float)
    good = np.isfinite(d) & (d > 1e-3) & (d < 12.0)

    xc = (uu - cx) / fx * d
    yc = (vv - cy) / fy * d
    zc = d
    R, t = T[:3, :3], T[:3, 3]

    # validation pixels
    ys, xs = np.nonzero(good)
    ref_uv, ref_xyz = [], []
    if len(ys) > 40:
        idx = np.linspace(0, len(ys) - 1, 24).astype(int)
        for k in idx:
            iu = int(uu[ys[k], xs[k]])
            iv = int(vv[ys[k], xs[k]])
            try:
                p = _arr(api.deproject(iu, iv))
            except Exception:
                continue
            if p.shape == (3,) and np.all(np.isfinite(p)):
                ref_uv.append((ys[k], xs[k]))
                ref_xyz.append(p)

    best, best_err = None, 1e9
    for sx, sy, sz in ((1, 1, 1), (1, -1, -1), (-1, -1, 1), (-1, 1, -1),
                       (1, -1, 1), (-1, 1, 1)):
        P = np.stack([xc * sx, yc * sy, zc * sz], axis=-1)
        W = P @ R.T + t
        if not ref_xyz:
            best, best_err = W, 0.0
            break
        errs = [float(np.linalg.norm(W[a, b] - q))
                for (a, b), q in zip(ref_uv, ref_xyz)]
        err = float(np.median(errs))
        if err < best_err:
            best, best_err = W, err

    if best is not None and best_err < 0.02:
        pts = best[good]
        cols = col[good]
        return pts.reshape(-1, 3), cols.reshape(-1, 3)

    # fallback: ask the API directly on a coarse grid
    _log(api, "cloud: vectorised deprojection rejected (err=%.3f), using API grid"
         % best_err)
    st = max(stride * 2, 6)
    pts, cols = [], []
    for iv in range(0, h, st):
        for iu in range(0, w, st):
            dv = depth[iv, iu]
            if not np.isfinite(dv) or dv <= 1e-3 or dv > 12.0:
                continue
            try:
                p = _arr(api.deproject(iu, iv))
            except Exception:
                continue
            if p.shape == (3,) and np.all(np.isfinite(p)):
                pts.append(p)
                cols.append(rgb[iv, iu, :3].astype(float))
    if not pts:
        return np.zeros((0, 3)), np.zeros((0, 3))
    return np.asarray(pts), np.asarray(cols)


def _table_z(pts):
    z = pts[:, 2]
    m = (z > TABLE_Z_SEARCH_BAND[0]) & (z < TABLE_Z_SEARCH_BAND[1])
    m &= (pts[:, 0] > WORKSPACE_BOX[0]) & (pts[:, 0] < WORKSPACE_BOX[1])
    m &= (pts[:, 1] > WORKSPACE_BOX[2]) & (pts[:, 1] < WORKSPACE_BOX[3])
    if m.sum() < 50:
        return None
    zz = z[m]
    hist, edges = np.histogram(zz, bins=40,
                               range=(TABLE_Z_SEARCH_BAND[0], TABLE_Z_SEARCH_BAND[1]))
    k = int(np.argmax(hist))
    c = 0.5 * (edges[k] + edges[k + 1])
    near = zz[np.abs(zz - c) < 0.012]
    return float(np.median(near)) if near.size else float(c)


def _clusters(pts, cols, region, zlo, zhi, cell=0.012, min_cells=8):
    """Grid-cluster elevated points inside a region; returns cluster dicts."""
    x0, x1, y0, y1 = region
    m = ((pts[:, 0] > x0) & (pts[:, 0] < x1) &
         (pts[:, 1] > y0) & (pts[:, 1] < y1) &
         (pts[:, 2] > zlo) & (pts[:, 2] < zhi))
    if m.sum() < 12:
        return []
    P, C = pts[m], cols[m]
    gi = ((P[:, 0] - x0) / cell).astype(int)
    gj = ((P[:, 1] - y0) / cell).astype(int)
    H = int(np.ceil((x1 - x0) / cell)) + 1
    W = int(np.ceil((y1 - y0) / cell)) + 1
    gi = np.clip(gi, 0, H - 1)
    gj = np.clip(gj, 0, W - 1)
    occ = np.zeros((H, W), dtype=bool)
    occ[gi, gj] = True
    lab, n = _label_grid(occ)
    plab = lab[gi, gj]
    out = []
    for c in range(1, n + 1):
        sel = plab == c
        if int(occ[lab == c].size) < min_cells or sel.sum() < 12:
            continue
        Q, QC = P[sel], C[sel]
        xmin, xmax = float(Q[:, 0].min()), float(Q[:, 0].max())
        ymin, ymax = float(Q[:, 1].min()), float(Q[:, 1].max())
        centre = np.array([0.5 * (xmin + xmax), 0.5 * (ymin + ymax)])
        diam = 0.5 * ((xmax - xmin) + (ymax - ymin))
        top = float(np.percentile(Q[:, 2], 95))
        rad = np.linalg.norm(Q[:, :2] - centre, axis=1)
        R = max(1e-3, 0.5 * diam)
        ring = float(np.mean((rad > 0.55 * R) & (rad < 1.15 * R)))
        mx = QC.max(axis=1)
        mn = QC.min(axis=1)
        out.append(dict(centre=centre, diam=diam, top=top, ring=ring,
                        n=int(sel.sum()),
                        value=float(QC.mean()),
                        sat=float(np.mean(mx - mn)),
                        minc=float(np.mean(mn)),
                        xr=(xmin, xmax), yr=(ymin, ymax)))
    return out


def _find_bowl(api, pts, cols, ztab, avoid=None):
    cl = _clusters(pts, cols, BOWL_REGION, ztab + 0.015, ztab + 0.14)
    best, bs = None, -1e9
    for c in cl:
        if avoid is not None and np.linalg.norm(c["centre"] - avoid) < 0.12:
            continue
        hgt = c["top"] - ztab
        s = 0.0
        s += 2.0 * np.exp(-((c["diam"] - 0.125) ** 2) / (2 * 0.05 ** 2))
        s += 1.5 * np.exp(-((hgt - 0.055) ** 2) / (2 * 0.035 ** 2))
        s += 0.8 * c["ring"]
        if c["sat"] < 45.0:
            s += 0.4
        if c["value"] < 175.0:
            s += 0.3
        s -= 3.0 * float(np.linalg.norm(c["centre"] - np.array(BOWL_PRIOR_XY)))
        if s > bs:
            best, bs = c, s
    if best is None:
        _log(api, "bowl: no cluster, falling back to pack prior")
        return np.array(BOWL_PRIOR_XY), RIM_RADIUS_PRIOR, None
    r = float(np.clip(0.5 * best["diam"], RIM_RADIUS_CLAMP[0], RIM_RADIUS_CLAMP[1]))
    _log(api, "bowl: centre=(%.3f,%.3f) diam=%.3f top=%.3f ring=%.2f score=%.2f"
         % (best["centre"][0], best["centre"][1], best["diam"], best["top"],
            best["ring"], bs))
    return best["centre"].copy(), r, best["top"]


def _find_stove(api, pts, cols, ztab):
    x0, x1, y0, y1 = STOVE_REGION
    bright = (cols.min(axis=1) > 135.0) & ((cols.max(axis=1) - cols.min(axis=1)) < 48.0)
    m = ((pts[:, 0] > x0) & (pts[:, 0] < x1) &
         (pts[:, 1] > y0) & (pts[:, 1] < y1) &
         (pts[:, 2] > ztab + 0.006) & (pts[:, 2] < ztab + 0.13) & bright)
    if m.sum() < 30:
        _log(api, "stove: no bright slab, using pack prior")
        return None, None
    cl = _clusters(pts[m], cols[m], STOVE_REGION, ztab + 0.006, ztab + 0.13,
                   cell=0.014, min_cells=10)
    if not cl:
        _log(api, "stove: slab did not cluster, using pack prior")
        return None, None
    cl.sort(key=lambda c: -c["n"])
    slab = cl[0]
    centre = slab["centre"].copy()
    top = slab["top"]
    # burner disc = dark elevated points inside the slab footprint
    xr, yr = slab["xr"], slab["yr"]
    dark = (cols.mean(axis=1) < 130.0)
    dm = ((pts[:, 0] > xr[0] - 0.01) & (pts[:, 0] < xr[1] + 0.01) &
          (pts[:, 1] > yr[0] - 0.01) & (pts[:, 1] < yr[1] + 0.01) &
          (pts[:, 2] > ztab + 0.006) & (pts[:, 2] < top + 0.02) & dark)
    if dm.sum() >= 25:
        D = pts[dm]
        dc = np.array([0.5 * (D[:, 0].min() + D[:, 0].max()),
                       0.5 * (D[:, 1].min() + D[:, 1].max())])
        dd = 0.5 * ((D[:, 0].max() - D[:, 0].min()) + (D[:, 1].max() - D[:, 1].min()))
        if 0.06 < dd < 0.22 and np.linalg.norm(dc - centre) < 0.08:
            centre = 0.5 * (centre + dc)
            _log(api, "stove: burner disc d=%.3f blended" % dd)
    if np.linalg.norm(centre - np.array(STOVE_PRIOR_XY)) > 0.18:
        _log(api, "stove: perceived centre (%.3f,%.3f) far from prior, keeping prior"
             % (centre[0], centre[1]))
        return None, None
    _log(api, "stove: centre=(%.3f,%.3f) top=%.3f n=%d"
         % (centre[0], centre[1], top, slab["n"]))
    return centre, float(top)


# ==========================================================================
# finger-closing axis, measured in-episode from the wrist camera
# ==========================================================================
def _finger_axis(api):
    default = np.array([0.0, 1.0])
    try:
        api.grip(OPEN_WIDTH)
        api.settle(0.15)
        f_open = api.capture("cam_arm_wrist")
        img_open = np.asarray(f_open.rgb).astype(float)
        api.grip(0.0)
        api.settle(0.15)
        f_cl = api.capture("cam_arm_wrist")
        img_cl = np.asarray(f_cl.rgb).astype(float)
        api.grip(OPEN_WIDTH)
        api.settle(0.15)
    except Exception as e:
        _log(api, "axis: wrist probe unavailable (%s), default +y" % e)
        return default, "default"

    try:
        diff = np.abs(img_open - img_cl).sum(axis=2)
        mask = diff > 45.0
        if int(mask.sum()) < 150:
            _log(api, "axis: wrist diff too small, default +y")
            return default, "default"
        ys, xs = np.nonzero(mask)
        pu = np.stack([xs.astype(float), ys.astype(float)], axis=1)
        mu = pu.mean(axis=0)
        cov = np.cov((pu - mu).T)
        w, V = np.linalg.eigh(cov)
        if w[1] <= 1e-6 or (w[1] / max(w[0], 1e-6)) < 2.0:
            _log(api, "axis: wrist diff not elongated, default +y")
            return default, "default"
        d2 = V[:, 1]
        proj = (pu - mu) @ d2
        aidx = proj > 0.35 * proj.max()
        bidx = proj < 0.35 * proj.min()
        if aidx.sum() < 25 or bidx.sum() < 25:
            _log(api, "axis: wrist diff lobes too small, default +y")
            return default, "default"
        ca = pu[aidx].mean(axis=0)
        cb = pu[bidx].mean(axis=0)
        depth = _arr(f_open.depth)
        H, W = depth.shape[:2]
        pa = pb = None
        for target, store in ((ca, "a"), (cb, "b")):
            u0, v0 = int(round(target[0])), int(round(target[1]))
            u0 = int(np.clip(u0, 2, W - 3))
            v0 = int(np.clip(v0, 2, H - 3))
            got = None
            for rr in (0, 2, 4, 6):
                for du in (-rr, 0, rr):
                    for dv in (-rr, 0, rr):
                        uu = int(np.clip(u0 + du, 2, W - 3))
                        vv = int(np.clip(v0 + dv, 2, H - 3))
                        dz = depth[vv, uu]
                        if not np.isfinite(dz) or dz <= 1e-3:
                            continue
                        try:
                            q = _arr(api.deproject(uu, vv))
                        except Exception:
                            continue
                        if q.shape == (3,) and np.all(np.isfinite(q)):
                            got = q
                            break
                    if got is not None:
                        break
                if got is not None:
                    break
            if store == "a":
                pa = got
            else:
                pb = got
        if pa is None or pb is None:
            _log(api, "axis: lobe depth invalid, default +y")
            return default, "default"
        v3 = pa - pb
        horiz = v3[:2]
        n = float(np.linalg.norm(horiz))
        if n < 1e-4 or n < 0.55 * float(np.linalg.norm(v3)):
            _log(api, "axis: lobe separation not horizontal, default +y")
            return default, "default"
        ax = horiz / n
        _log(api, "axis: measured closing axis (%.3f,%.3f) sep=%.3f m"
             % (ax[0], ax[1], n))
        return ax, "measured"
    except Exception as e:
        _log(api, "axis: measurement failed (%s), default +y" % e)
        return default, "default"


# ==========================================================================
# gripper state test  (own sensors only)
# ==========================================================================
def _holding(api):
    try:
        g = api.gripper()
    except Exception:
        return False, -1.0, -1.0
    try:
        wdt = float(g["width_m"])
    except Exception:
        wdt = -1.0
    try:
        eff = float(g["effort"])
    except Exception:
        eff = -1.0
    ok = (HOLD_WIDTH_BAND[0] <= wdt <= HOLD_WIDTH_BAND[1])
    if eff >= 0.0:
        ok = ok and (eff >= 1.0)
    return bool(ok), wdt, eff


def _move(api, xyz, seconds=1.1):
    try:
        return api.move([float(xyz[0]), float(xyz[1]), float(xyz[2])],
                        seconds=seconds)
    except Exception as e:
        _log(api, "move failed: %s" % e)
        return None


# ==========================================================================
# main
# ==========================================================================
def _main(api):
    _log(api, "instruction: %s" % api.instruction())

    # ---- perceive -------------------------------------------------------
    ztab = None
    pts = np.zeros((0, 3))
    cols = np.zeros((0, 3))
    for attempt in range(2):
        try:
            fr = api.capture("cam_high")
            pts, cols = _cloud(api, fr, stride=3)
        except Exception as e:
            _log(api, "capture failed: %s" % e)
            pts, cols = np.zeros((0, 3)), np.zeros((0, 3))
        if pts.shape[0] > 500:
            ztab = _table_z(pts)
        if ztab is not None:
            break
        if attempt == 0:
            _log(api, "perception thin, retreating the arm and retrying")
            _move(api, [-0.21, -0.02, 1.19], seconds=1.0)
            api.settle(0.2)
    if ztab is None:
        ztab = Z_GRASP - 0.05
        _log(api, "table height not measured, assuming %.3f" % ztab)
    else:
        _log(api, "table z = %.4f (%d pts)" % (ztab, pts.shape[0]))

    stove_xy, stove_top = _find_stove(api, pts, cols, ztab)
    bowl_xy, rim_r, _bowl_top = _find_bowl(
        api, pts, cols, ztab, avoid=(stove_xy if stove_xy is not None else None))

    # ---- go above the bowl and measure the finger-closing axis -----------
    api.grip(OPEN_WIDTH)
    _move(api, [bowl_xy[0], bowl_xy[1], Z_APPROACH], seconds=1.4)
    axis, axis_src = _finger_axis(api)
    perp = np.array([-axis[1], axis[0]])

    plan = [(axis, Z_GRASP), (perp, Z_GRASP), (axis, Z_GRASP_DEEP)]
    plan = plan[:MAX_ATTEMPTS]

    held = False
    hold_off = np.array([0.0, 0.0])
    for k, (d, zg) in enumerate(plan):
        gx = bowl_xy[0] + rim_r * d[0]
        gy = bowl_xy[1] + rim_r * d[1]
        _log(api, "attempt %d: axis=(%.2f,%.2f) src=%s rim=(%.3f,%.3f) z=%.3f"
             % (k + 1, d[0], d[1], axis_src, gx, gy, zg))
        api.grip(OPEN_WIDTH)
        _move(api, [gx, gy, Z_APPROACH], seconds=0.9)
        _move(api, [gx, gy, zg], seconds=1.0)
        api.settle(0.15)
        api.grip(0.0)
        api.settle(0.30)
        ok, wdt, eff = _holding(api)
        _log(api, "  after close: width=%.4f effort=%.2f ok=%s" % (wdt, eff, ok))
        if ok:
            _move(api, [gx, gy, Z_TRANSPORT], seconds=1.0)
            api.settle(0.20)
            ok2, w2, e2 = _holding(api)
            _log(api, "  after lift: width=%.4f effort=%.2f ok=%s" % (w2, e2, ok2))
            if ok2:
                held = True
                hold_off = np.array([gx, gy]) - bowl_xy
                break
        # failed -> release, back off, re-perceive
        api.grip(OPEN_WIDTH)
        api.settle(0.10)
        _move(api, [gx, gy, Z_APPROACH], seconds=0.9)
        if k + 1 < len(plan):
            try:
                fr = api.capture("cam_high")
                p2, c2 = _cloud(api, fr, stride=3)
                if p2.shape[0] > 500:
                    zt2 = _table_z(p2)
                    if zt2 is not None:
                        ztab = zt2
                    nb, nr, _ = _find_bowl(api, p2, c2, ztab,
                                           avoid=(stove_xy if stove_xy is not None
                                                  else None))
                    if np.linalg.norm(nb - bowl_xy) < 0.15:
                        bowl_xy, rim_r = nb, nr
            except Exception as e:
                _log(api, "  re-perception failed: %s" % e)

    if not held:
        # last resort: carry whatever is in the hand toward the stove anyway
        _log(api, "no confirmed hold; proceeding with the best-effort transport")
        hold_off = np.array([rim_r * plan[0][0][0], rim_r * plan[0][0][1]])
        _move(api, [bowl_xy[0] + hold_off[0], bowl_xy[1] + hold_off[1],
                    Z_TRANSPORT], seconds=1.0)

    # ---- transport & place ----------------------------------------------
    if stove_xy is not None:
        tgt = np.asarray(stove_xy, dtype=float)
        place_xy = tgt + hold_off
    else:
        # fall back on the pack's own release pose (already includes an offset
        # of the same kind, taken along the demos' closing axis)
        place_xy = np.array(STOVE_PRIOR_XY, dtype=float)

    if stove_top is not None:
        place_z = float(np.clip(Z_GRASP + (stove_top - ztab) + 0.008, 0.945, 0.995))
    else:
        place_z = Z_RELEASE_PRIOR
    _log(api, "place ee=(%.3f,%.3f,%.3f) hold_off=(%.3f,%.3f)"
         % (place_xy[0], place_xy[1], place_z, hold_off[0], hold_off[1]))

    _move(api, [place_xy[0], place_xy[1], Z_TRANSPORT], seconds=1.6)
    api.settle(0.15)
    _move(api, [place_xy[0], place_xy[1], place_z], seconds=1.1)
    api.settle(0.20)
    api.grip(OPEN_WIDTH)
    api.settle(0.35)
    _move(api, [place_xy[0], place_xy[1], Z_TRANSPORT], seconds=0.9)
    _move(api, [-0.20, 0.10, 1.12], seconds=1.0)
    api.settle(0.30)
    _log(api, "sequence complete")


def run(api):
    try:
        _main(api)
    except Exception as e:
        _log(api, "run aborted: %r" % (e,))
        try:
            api.grip(OPEN_WIDTH)
            api.settle(0.2)
        except Exception:
            pass
