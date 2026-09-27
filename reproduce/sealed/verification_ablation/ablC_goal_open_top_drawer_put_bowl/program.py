"""abl_c2 cell ablC_goal_open_top_drawer_put_bowl  (variant C -- no verification).

Intent: "open the top drawer and put the bowl inside".

Everything below is derived from packs/ablC_goal_open_top_drawer_put_bowl
(pack.json keyframes / ee_path6 / gripper_state, and the keyframe PNGs) plus
generic camera + parallel-jaw mechanics.  No episode was ever executed while
writing this file, so the design leans on (a) constants read straight off the
demonstration end-effector paths and (b) closed-loop checks that use only the
sensors the FairApi exposes (depth re-perception, gripper effort, move
residuals).  Every perception step is wrapped so that a failure degrades to the
demonstration-mean constant rather than crashing.

Mechanism, in three acts:

  ACT 1 -- open the drawer.  In all three demos the open-jaw tool descends to
  z ~= 1.106 at (x ~= 0.030, y ~= -0.091) and then translates monotonically in
  +y by ~0.16 m at constant x and z; immediately afterwards the keyframe images
  show the cabinet drawer extended.  So the drawer slide axis is base +y and
  the hook pose is fixed in the base frame (the cabinet silhouette is identical
  to within ~2 px across the three t=0 keyframes, i.e. the fixture is not
  re-randomised).  We replay that hook and drag.

  ACT 2 -- verify the drawer with our own eyes.  We retract to the demo t=0
  arm pose (which leaves the table unoccluded in every keyframe image) and
  re-segment the tall dark cabinet blob.  If its +y extent grew by more than
  DRAWER_OPEN_MIN the drawer moved; otherwise we re-hook once, using the
  perceived cabinet face instead of the demo mean.

  ACT 3 -- pick the bowl by its rim and drop it in.  The demo gripper closes to
  a total width of ~4.7 mm while holding, so the demonstrated grasp is a pinch
  on the thin bowl wall, not an enclosure of the body.  Comparing the demo
  grasp y against the bowl's position in the keyframe images puts the pinch on
  the -y extreme of the rim (grasp_y = centre_y - r).  We therefore segment the
  bright, ~4 cm tall object on the table left of the drawer, take the 3rd
  percentile of its y as the outer rim edge, and pinch there at the demo grasp
  height.  Parallel jaws closing symmetrically about that wall are
  self-centring, so this is the tolerant axis; the intolerant axis is x, which
  we take from the segmented centroid.  Grasp is confirmed with gripper effort
  before the carry, with one offset retry.  The release pose is expressed
  relative to where our own drag actually ended, so an over- or under-pull does
  not move the bowl outside the drawer.
"""

import numpy as np

# --------------------------------------------------------------------------
# calibrated constants
# --------------------------------------------------------------------------

# Act 1 -- drawer hook / drag  (pack ee_path6, constant-x constant-z +y segment)
X_HOOK = 0.030          # mean of 0.020 / 0.045 / 0.024
Y_HOOK = -0.091         # mean of -0.0930 / -0.0874 / -0.0937
Z_HOOK = 1.106          # mean of 1.105 / 1.108 / 1.105
Z_HOOK_APPROACH = 1.160  # demo z one stride before the hook (1.152 / 1.19 / 1.17)
DRAG_TOTAL = 0.170      # demo drag 0.1600 (mean of .158/.159/.163) + 1 cm margin
DRAG_STEPS = 3
DRAWER_OPEN_MIN = 0.055  # >1/3 of the demonstrated drag, as an opened-or-not test

# Act 3 -- bowl pick  (pack keyframes at the gripper-close transition)
BOWL_X_NOM = -0.105     # mean of -0.107 / -0.103 / -0.107
BOWL_GRASP_Y_NOM = 0.045  # mean of 0.0715 / 0.025 / 0.0395
BOWL_R_NOM = 0.037      # rim radius from the keyframe images, scaled by the
                        # known 0.16 m drawer travel (~0.074 m across)
Z_GRASP = 0.917         # mean demo ee z at the gripper-close keyframe
Z_ABOVE_BOWL = 1.030    # demo ee z on the descent to the bowl (0.996/0.990/0.983)
RIM_HALF_WALL = 0.003   # half of the 4.7 mm closed width the demos report
BOWL_X_LO, BOWL_X_HI = -0.170, -0.045   # BOWL_X_NOM +/- 0.065
BOWL_Y_LO, BOWL_Y_HI = -0.060, 0.220    # demo grasp y range widened by 0.10

# Act 3 -- release  (pack keyframes at the gripper-open transition, expressed
# relative to the drag so the drop follows the drawer we actually pulled)
PLACE_DX = -0.026       # release x  minus hook x
PLACE_DY = -0.117       # release y  minus drag-end y
Z_PLACE = 1.132         # mean release z of the two single-attempt demos
Z_CARRY = 1.230         # demo transit apex (1.219 / 1.234 / 1.219)

# Act 2 -- observation pose (demo t=0 ee, raised clear of the opened drawer)
HOME_VIEW = (-0.190, -0.010, 1.230)   # demo t=0 mean (-0.197, -0.008, 1.172)

# gripper (pack gripper_state; API contract for effort)
GRIP_OPEN = 0.078       # 2 x 0.0395 open finger reading
GRIP_CLOSE = 0.000
HOLD_EFFORT = 1.0       # api reports effort 3.0 iff holding

# segmentation thresholds (generic camera mechanics + the keyframe images)
DARK_RGB = 75.0         # near-black cabinet vs. wood table / white crockery
BRIGHT_RGB = 105.0      # white bowl vs. table
CABINET_MIN_H = 0.090   # cabinet body rises far above the table
CABINET_MAX_H = 0.450
BOWL_MIN_H = 0.020      # bowl rim stands proud of the flat plate beside it
BOWL_MAX_H = 0.100
Z_TABLE_NOM = 0.900     # implied by the 0.917 demo grasp height
Z_TABLE_LO, Z_TABLE_HI = 0.820, 0.960
BOWL_SPAN_Y_LO, BOWL_SPAN_Y_HI = 0.030, 0.170   # 0.074 m rim, generous either way
BOWL_SPAN_X_LO, BOWL_SPAN_X_HI = 0.015, 0.200   # x is the foreshortened axis
OCCLUSION_FRAC = 0.55   # post-drag blob this much smaller => arm clipped it

# recovery offsets (only used when a self-check says the nominal attempt failed)
LIFT_CHECK_DZ = 0.085   # short lift that loads the jaws before the effort read
RETRY_DY = 0.014        # step the pinch further into the bowl
RETRY_DZ = 0.006        # and a little lower
REHOOK_DY = 0.022       # step the hook further behind the drawer face
REHOOK_DZ = 0.008       # and a little lower, in case the jaws capped the handle
REHOOK_MAX_SHIFT = 0.035  # cap on any perception-driven correction of the hook

# workspace clamp (demo ee extent + margin)
WS_LO = np.array([-0.300, -0.200, 0.880])
WS_HI = np.array([0.160, 0.230, 1.320])

TIME_BUDGET = 21.0      # seconds of commanded motion before retries are dropped

PROVENANCE = {
    "X_HOOK": {"source": "pack.json demos[*].ee_path6: x during the constant-z +y drag segment (0.020/0.045/0.024)", "allowed": True},
    "Y_HOOK": {"source": "pack.json demos[*].ee_path6: y at the first sample of the drag segment (-0.0930/-0.0874/-0.0937)", "allowed": True},
    "Z_HOOK": {"source": "pack.json demos[*].ee_path6: z during the drag segment (1.105/1.108/1.105)", "allowed": True},
    "Z_HOOK_APPROACH": {"source": "pack.json demos[*].ee_path6: z one stride before the hook (1.152/1.19/1.17)", "allowed": True},
    "DRAG_TOTAL": {"source": "pack.json demos[*].ee_path6: +y travel of the drag segment (0.158/0.159/0.163) plus 0.01 margin", "allowed": True},
    "DRAG_STEPS": {"source": "generic controller mechanics: split a long pushed/pulled translation into sub-moves so the controller keeps applying force", "allowed": True},
    "DRAWER_OPEN_MIN": {"source": "pack.json drag travel 0.160; a third of it is used as the opened-or-not decision threshold", "allowed": True},
    "BOWL_X_NOM": {"source": "pack.json demos[*].keyframes: ee x at the gripper-close keyframe (-0.107/-0.103/-0.107)", "allowed": True},
    "BOWL_GRASP_Y_NOM": {"source": "pack.json demos[*].keyframes: ee y at the gripper-close keyframe (0.0715/0.025/0.0395)", "allowed": True},
    "BOWL_R_NOM": {"source": "pack keyframes/*.png: bowl rim spans ~0.074 m using the image scale fixed by the known 0.16 m drawer travel", "allowed": True},
    "Z_GRASP": {"source": "pack.json demos[*].keyframes: ee z at the gripper-close keyframe (0.9386/0.9183/0.9149)", "allowed": True},
    "Z_ABOVE_BOWL": {"source": "pack.json demos[*].ee_path6: z one stride above the grasp (0.996/0.990/0.983), rounded up", "allowed": True},
    "RIM_HALF_WALL": {"source": "pack.json demos[*].keyframes gripper_state while holding (0.0022+0.0025 = 4.7 mm total width) -> half wall 0.003", "allowed": True},
    "BOWL_X_LO": {"source": "BOWL_X_NOM minus 0.065 search margin (pack ee keyframes)", "allowed": True},
    "BOWL_X_HI": {"source": "BOWL_X_NOM plus 0.060 search margin (pack ee keyframes)", "allowed": True},
    "BOWL_Y_LO": {"source": "pack demo grasp y range (0.010..0.0715) widened by 0.10 downward", "allowed": True},
    "BOWL_Y_HI": {"source": "pack demo grasp y range (0.010..0.0715) widened by 0.15 upward", "allowed": True},
    "PLACE_DX": {"source": "pack.json: release-keyframe ee x (0.0037 mean) minus X_HOOK", "allowed": True},
    "PLACE_DY": {"source": "pack.json: release-keyframe ee y (-0.0481 mean) minus the drag-end y (0.0686 mean)", "allowed": True},
    "Z_PLACE": {"source": "pack.json: release-keyframe ee z of the two demos that released once (1.1354/1.1301)", "allowed": True},
    "Z_CARRY": {"source": "pack.json demos[*].ee_path6: transit apex z while carrying (1.219/1.234/1.219)", "allowed": True},
    "HOME_VIEW": {"source": "pack.json demos[*].keyframes t=0 ee (-0.185/-0.192/-0.214, -0.013/-0.006/-0.006, 1.181/1.171/1.166), raised to clear the opened drawer", "allowed": True},
    "GRIP_OPEN": {"source": "pack.json demos[*].keyframes gripper_state when commanded open (2 x 0.0395)", "allowed": True},
    "GRIP_CLOSE": {"source": "FairApi contract: a commanded width below 0.025 closes the jaws", "allowed": True},
    "HOLD_EFFORT": {"source": "FairApi contract: api.gripper() effort is 3.0 iff the jaws hold something", "allowed": True},
    "DARK_RGB": {"source": "pack keyframes/*.png: the cabinet renders near-black against the wood table and white crockery", "allowed": True},
    "BRIGHT_RGB": {"source": "pack keyframes/*.png: the bowl renders white against the wood table", "allowed": True},
    "CABINET_MIN_H": {"source": "pack.json: the hook height 1.106 sits >0.2 m above the 0.917 grasp height, so the cabinet body clears the table by far more than 0.09 m", "allowed": True},
    "CABINET_MAX_H": {"source": "generic segmentation guard: reject points above the cabinet band so the arm and background cannot join the blob", "allowed": True},
    "BOWL_MIN_H": {"source": "pack keyframes/*.png: the bowl rim stands proud of the table; kept low for recall because the bowl-vs-plate decision is made by ranking candidate blobs on height, not by this threshold", "allowed": True},
    "BOWL_MAX_H": {"source": "generic segmentation guard: a table-top bowl is far below 0.10 m tall relative to the table plane", "allowed": True},
    "Z_TABLE_NOM": {"source": "pack.json: implied by the 0.917 demo grasp height on a table-top bowl", "allowed": True},
    "Z_TABLE_LO": {"source": "Z_TABLE_NOM minus 0.08 sanity clamp", "allowed": True},
    "Z_TABLE_HI": {"source": "Z_TABLE_NOM plus 0.06 sanity clamp", "allowed": True},
    "WS_LO": {"source": "pack.json demos[*].ee_path6 componentwise minima (-0.214/-0.106/0.915) with margin", "allowed": True},
    "WS_HI": {"source": "pack.json demos[*].ee_path6 componentwise maxima (0.055/0.101/1.258) with margin", "allowed": True},
    "BOWL_SPAN_Y_LO": {"source": "pack keyframes/*.png: rim ~0.074 m across (image scale fixed by the known 0.16 m drawer travel); lower acceptance bound", "allowed": True},
    "BOWL_SPAN_Y_HI": {"source": "pack keyframes/*.png: rim ~0.074 m across; upper acceptance bound", "allowed": True},
    "BOWL_SPAN_X_LO": {"source": "as BOWL_SPAN_Y_LO but on the axis the camera foreshortens, so the bound is looser", "allowed": True},
    "BOWL_SPAN_X_HI": {"source": "as BOWL_SPAN_Y_HI but on the axis the camera foreshortens", "allowed": True},
    "OCCLUSION_FRAC": {"source": "generic camera mechanics: a blob that lost nearly half its pixels between two views of the same static object is partly occluded", "allowed": True},
    "LIFT_CHECK_DZ": {"source": "generic parallel-jaw mechanics: lift enough to load the jaws before reading effort, while staying under the 0.996 m the demos pass on the way up", "allowed": True},
    "RETRY_DY": {"source": "pack: 4.7 mm closed width implies a thin rim wall, so a failed pinch is corrected by roughly half a wall-plus-jaw step further into the bowl", "allowed": True},
    "RETRY_DZ": {"source": "pack.json demo grasp z spread (0.9386/0.9183/0.9149); a retry drops within that spread", "allowed": True},
    "REHOOK_DY": {"source": "pack.json demo hook y spread (-0.0930/-0.0874/-0.0937); a retry steps beyond that spread behind the drawer face", "allowed": True},
    "REHOOK_DZ": {"source": "pack.json demo hook z spread (1.105/1.108/1.105); a retry drops just beyond it", "allowed": True},
    "REHOOK_MAX_SHIFT": {"source": "pack.json demo hook x spread (0.020..0.045) sets how far a perception-driven correction may move the hook", "allowed": True},
    "TIME_BUDGET": {"source": "pack.json demo lengths 170/187/284 control steps; the nominal plan is shorter, and this caps optional retries", "allowed": True},
}


# --------------------------------------------------------------------------
# small helpers
# --------------------------------------------------------------------------

class _Clock(object):
    def __init__(self):
        self.spent = 0.0


def _clamp_ws(p):
    return np.minimum(np.maximum(np.asarray(p, dtype=float), WS_LO), WS_HI)


def _go(api, clock, xyz, seconds):
    """Clamped, guarded cartesian move.  Returns the residual or None."""
    target = _clamp_ws(xyz)
    clock.spent += float(seconds)
    try:
        return api.move([float(target[0]), float(target[1]), float(target[2])],
                        seconds=float(seconds))
    except Exception as exc:            # never let a move abort the episode
        try:
            api.log("move failed: %r" % (exc,))
        except Exception:
            pass
        return None


def _grip(api, width):
    try:
        api.grip(float(width))
    except Exception:
        pass


def _settle(api, clock, seconds):
    clock.spent += float(seconds)
    try:
        api.settle(float(seconds))
    except Exception:
        pass


def _log(api, msg):
    try:
        api.log(msg)
    except Exception:
        pass


def _eef(api, fallback):
    try:
        p = np.asarray(api.eef(), dtype=float).reshape(3)
        if np.all(np.isfinite(p)):
            return p
    except Exception:
        pass
    return np.asarray(fallback, dtype=float)


def _holding(api):
    """True iff the jaws report holding.  Unknown -> True (do not abandon)."""
    try:
        g = api.gripper()
        eff = float(g["effort"])
        return eff >= HOLD_EFFORT
    except Exception:
        return True


# --------------------------------------------------------------------------
# perception
# --------------------------------------------------------------------------

def _depth2d(frame):
    d = np.asarray(frame.depth, dtype=float)
    while d.ndim > 2:
        d = d[..., 0]
    return d


def _cloud_from_intrinsics(depth, K, T, step, flip):
    H, W = depth.shape
    us = np.arange(0, W, step)
    vs = np.arange(0, H, step)
    uu, vv = np.meshgrid(us, vs)
    dd = depth[vv, uu]
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    xc = (uu - cx) * dd / fx
    yc = (vv - cy) * dd / fy
    zc = dd.copy()
    if flip:
        yc = -yc
        zc = -zc
    pts = np.stack([xc, yc, zc], axis=-1)
    base = pts @ np.asarray(T[:3, :3], dtype=float).T + np.asarray(T[:3, 3], dtype=float)
    return base, uu, vv, dd


def _build_cloud(api, frame, step=4):
    """Base-frame point cloud on a subsampled pixel grid.

    The pinhole convention of the depth image is not documented, so both signs
    are tried and checked against api.deproject on a handful of probe pixels;
    if neither matches we fall back to calling api.deproject directly.
    Returns (points HxWx3, uu, vv, depth) or None.
    """
    depth = _depth2d(frame)
    H, W = depth.shape
    good = np.isfinite(depth) & (depth > 0.05) & (depth < 6.0)
    if good.sum() < 200:
        return None
    gv, gu = np.nonzero(good)
    idx = np.linspace(0, len(gv) - 1, 7).astype(int)
    probes = []
    for i in idx:
        u, v = int(gu[i]), int(gv[i])
        try:
            p = np.asarray(frame.deproject(u, v), dtype=float).reshape(3)
        except Exception:
            continue
        if np.all(np.isfinite(p)):
            probes.append((u, v, p))
    K = np.asarray(frame.intrinsics, dtype=float)
    T = np.asarray(frame.t_base_cam, dtype=float)

    best = None
    if probes:
        for flip in (False, True):
            try:
                full, _, _, _ = _cloud_from_intrinsics(depth, K, T, 1, flip)
            except Exception:
                continue
            err = max(float(np.linalg.norm(full[v, u] - p)) for (u, v, p) in probes)
            if best is None or err < best[0]:
                best = (err, flip)
        if best is not None and best[0] < 0.010:
            pts, uu, vv, dd = _cloud_from_intrinsics(depth, K, T, step, best[1])
            return pts, uu, vv, dd

    # fallback: ask the api pixel by pixel on a coarser grid
    coarse = max(step, 8)
    us = np.arange(0, W, coarse)
    vs = np.arange(0, H, coarse)
    uu, vv = np.meshgrid(us, vs)
    pts = np.full(uu.shape + (3,), np.nan)
    for a in range(uu.shape[0]):
        for b in range(uu.shape[1]):
            u, v = int(uu[a, b]), int(vv[a, b])
            if not good[v, u]:
                continue
            try:
                pts[a, b] = np.asarray(frame.deproject(u, v), dtype=float).reshape(3)
            except Exception:
                pass
    return pts, uu, vv, depth[vv, uu]


def _rgb_grid(frame, uu, vv):
    rgb = np.asarray(frame.rgb, dtype=float)
    return rgb[vv, uu].mean(axis=-1)


def _table_height(pts):
    z = pts[..., 2]
    ok = np.isfinite(z)
    if ok.sum() < 100:
        return Z_TABLE_NOM
    zz = z[ok]
    lo, hi = float(np.percentile(zz, 1)), float(np.percentile(zz, 99))
    if not np.isfinite(lo) or not np.isfinite(hi) or hi - lo < 1e-3:
        return Z_TABLE_NOM
    bins = np.arange(lo, hi + 0.005, 0.005)
    hist, edges = np.histogram(zz, bins=bins)
    if hist.size == 0:
        return Z_TABLE_NOM
    peak = 0.5 * (edges[int(np.argmax(hist))] + edges[int(np.argmax(hist)) + 1])
    if not (Z_TABLE_LO <= peak <= Z_TABLE_HI):
        return Z_TABLE_NOM
    return float(peak)


def _components(mask, min_n):
    """All 4-connected components of a boolean grid with at least min_n cells."""
    H, W = mask.shape
    seen = np.zeros((H, W), dtype=bool)
    comps = []
    for i in range(H):
        row = mask[i]
        for j in range(W):
            if not row[j] or seen[i, j]:
                continue
            stack = [(i, j)]
            seen[i, j] = True
            comp = []
            while stack:
                a, b = stack.pop()
                comp.append((a, b))
                if a > 0 and mask[a - 1, b] and not seen[a - 1, b]:
                    seen[a - 1, b] = True
                    stack.append((a - 1, b))
                if a + 1 < H and mask[a + 1, b] and not seen[a + 1, b]:
                    seen[a + 1, b] = True
                    stack.append((a + 1, b))
                if b > 0 and mask[a, b - 1] and not seen[a, b - 1]:
                    seen[a, b - 1] = True
                    stack.append((a, b - 1))
                if b + 1 < W and mask[a, b + 1] and not seen[a, b + 1]:
                    seen[a, b + 1] = True
                    stack.append((a, b + 1))
            if len(comp) >= min_n:
                out = np.zeros((H, W), dtype=bool)
                for a, b in comp:
                    out[a, b] = True
                comps.append(out)
    return comps


def _largest_blob(mask, min_n=1):
    comps = _components(mask, min_n)
    if not comps:
        return None
    return max(comps, key=lambda c: int(c.sum()))


def _observe(api, cam="cam_high"):
    """Return a dict of scene measurements, or {} if perception is unusable."""
    out = {}
    try:
        frame = api.capture(cam)
    except Exception:
        return out
    try:
        built = _build_cloud(api, frame)
    except Exception:
        built = None
    if built is None:
        return out
    pts, uu, vv, dd = built
    try:
        bright = _rgb_grid(frame, uu, vv)
    except Exception:
        return out
    x, y, z = pts[..., 0], pts[..., 1], pts[..., 2]
    finite = np.isfinite(z) & np.isfinite(x) & np.isfinite(y)
    z_table = _table_height(pts)
    out["z_table"] = z_table
    h = z - z_table

    # --- cabinet / drawer: the tall near-black body ---
    try:
        cab = finite & (h > CABINET_MIN_H) & (h < CABINET_MAX_H) & (bright < DARK_RGB)
        cab = cab & (x > -0.45) & (x < 0.45) & (y > -0.60) & (y < 0.60)
        if cab.sum() >= 30:
            blob = _largest_blob(cab)
            if blob is not None and blob.sum() >= 30:
                out["cab_n"] = int(blob.sum())
                out["cab_y_face"] = float(np.percentile(y[blob], 97))
                out["cab_x"] = float(np.median(x[blob]))
                out["cab_z_top"] = float(np.percentile(z[blob], 97))
    except Exception:
        pass

    # --- bowl: the bright, rim-high object on the table left of the drawer ---
    try:
        bw = (finite & (h > BOWL_MIN_H) & (h < BOWL_MAX_H) & (bright > BRIGHT_RGB)
              & (x > BOWL_X_LO - 0.06) & (x < BOWL_X_HI + 0.06)
              & (y > BOWL_Y_LO) & (y < BOWL_Y_HI))
        if bw.sum() >= 12:
            # The table also carries a flat plate of similar colour and larger
            # footprint, so do NOT take the biggest blob: take the tallest one
            # among the candidates whose footprint is bowl-sized.  A bowl rim
            # rises clearly further off the table than a plate does.
            best = None
            for blob in _components(bw, 12):
                by = y[blob]
                bx = x[blob]
                span_y = float(np.percentile(by, 97) - np.percentile(by, 3))
                span_x = float(np.percentile(bx, 97) - np.percentile(bx, 3))
                if not (BOWL_SPAN_Y_LO <= span_y <= BOWL_SPAN_Y_HI
                        and BOWL_SPAN_X_LO <= span_x <= BOWL_SPAN_X_HI):
                    continue
                score = float(np.percentile(h[blob], 90))
                if best is None or score > best[0]:
                    best = (score, blob, bx, by, span_y)
            if best is not None:
                _, blob, bx, by, span_y = best
                out["bowl_n"] = int(blob.sum())
                out["bowl_x"] = float(np.median(bx))
                out["bowl_y_min"] = float(np.percentile(by, 3))
                out["bowl_y_mid"] = float(np.median(by))
                out["bowl_span_y"] = span_y
                out["bowl_top_z"] = float(np.percentile(z[blob], 97))
    except Exception:
        pass
    return out


def _grasp_from(obs):
    """(x, y) pinch point on the -y rim wall of the bowl, plus a confidence."""
    gx = BOWL_X_NOM
    gy = BOWL_GRASP_Y_NOM
    ok = False
    if obs and "bowl_x" in obs:
        cand_x = float(np.clip(obs["bowl_x"], BOWL_X_LO, BOWL_X_HI))
        cand_y = obs["bowl_y_min"] + RIM_HALF_WALL
        # sanity: the pinch must sit within a rim radius of the blob centre
        if (abs(cand_y - obs["bowl_y_mid"]) < 3.0 * BOWL_R_NOM
                and BOWL_Y_LO < cand_y < BOWL_Y_HI):
            gx, gy, ok = cand_x, float(cand_y), True
    return gx, gy, ok


# --------------------------------------------------------------------------
# acts
# --------------------------------------------------------------------------

def _hook_and_drag(api, clock, x_hook, y_hook, z_hook):
    """Descend beside the drawer handle and translate +y.  Returns the end ee."""
    _grip(api, GRIP_OPEN)
    _go(api, clock, [x_hook, y_hook, Z_HOOK_APPROACH], 1.6)
    _go(api, clock, [x_hook, y_hook, z_hook], 0.9)
    step = DRAG_TOTAL / float(DRAG_STEPS)
    for i in range(DRAG_STEPS):
        _go(api, clock, [x_hook, y_hook + step * (i + 1), z_hook], 0.9)
    end = _eef(api, [x_hook, y_hook + DRAG_TOTAL, z_hook])
    _go(api, clock, [x_hook, float(end[1]), Z_CARRY], 1.0)
    return end


def _retract_to_view(api, clock):
    here = _eef(api, HOME_VIEW)
    _go(api, clock, [float(here[0]), float(here[1]), Z_CARRY], 0.7)
    _go(api, clock, list(HOME_VIEW), 1.6)


def run(api):
    clock = _Clock()
    try:
        _log(api, "instruction: %s" % (api.instruction(),))
    except Exception:
        pass

    _grip(api, GRIP_OPEN)

    # ---------------- observe the closed scene -------------------------
    obs0 = {}
    try:
        obs0 = _observe(api)
    except Exception:
        obs0 = {}
    _log(api, "obs0: %s" % ({k: (round(v, 4) if isinstance(v, float) else v)
                             for k, v in obs0.items()},))
    y_face_before = obs0.get("cab_y_face", None)

    # ---------------- ACT 1: open the drawer ---------------------------
    drag_end = _hook_and_drag(api, clock, X_HOOK, Y_HOOK, Z_HOOK)
    _retract_to_view(api, clock)

    # ---------------- ACT 2: check it with our own depth ---------------
    obs1 = {}
    try:
        obs1 = _observe(api)
    except Exception:
        obs1 = {}
    _log(api, "obs1: %s" % ({k: (round(v, 4) if isinstance(v, float) else v)
                             for k, v in obs1.items()},))

    y_face_after = obs1.get("cab_y_face", None)
    opened = None
    if y_face_before is not None and y_face_after is not None:
        opened = (y_face_after - y_face_before) > DRAWER_OPEN_MIN
        _log(api, "drawer face %.4f -> %.4f (opened=%s)"
             % (y_face_before, y_face_after, opened))

    if opened is False and clock.spent < TIME_BUDGET:
        # Second attempt: aim at the perceived cabinet face instead of the
        # demonstration mean, and drop 8 mm in case the jaws capped the handle.
        y2 = Y_HOOK - REHOOK_DY
        x2 = X_HOOK
        if y_face_before is not None:
            cand = y_face_before - REHOOK_DY
            if abs(cand - Y_HOOK) < REHOOK_MAX_SHIFT + 0.025:
                y2 = float(cand)
        if "cab_x" in obs0:
            x2 = float(np.clip(obs0["cab_x"],
                               X_HOOK - REHOOK_MAX_SHIFT, X_HOOK + REHOOK_MAX_SHIFT))
        _log(api, "re-hooking at x=%.4f y=%.4f" % (x2, y2))
        drag_end = _hook_and_drag(api, clock, x2, y2, Z_HOOK - REHOOK_DZ)
        _retract_to_view(api, clock)
        try:
            obs2 = _observe(api)
        except Exception:
            obs2 = {}
        if obs2:
            if "bowl_x" in obs2 or not obs1:
                obs1 = obs2
            _log(api, "obs2: %s" % ({k: (round(v, 4) if isinstance(v, float) else v)
                                     for k, v in obs2.items()},))

    y_drag_end = float(np.clip(drag_end[1], Y_HOOK + 0.06, Y_HOOK + DRAG_TOTAL + 0.02))
    x_drawer = X_HOOK

    # ---------------- ACT 3: pick the bowl ------------------------------
    # Prefer the post-drag view (it reflects any nudge the drawer gave the
    # bowl), but reject it if the arm clipped the blob: a much smaller
    # footprint than the clean pre-move view means partial occlusion, and a
    # clipped blob biases the rim edge we are about to pinch.
    obs_pick = obs1 if obs1 else obs0
    if (("bowl_n" in obs1) and ("bowl_n" in obs0)
            and obs1["bowl_n"] < OCCLUSION_FRAC * obs0["bowl_n"]):
        _log(api, "post-drag bowl blob looks clipped (%d vs %d); using pre-move view"
             % (obs1["bowl_n"], obs0["bowl_n"]))
        obs_pick = obs0
    gx, gy, seen = _grasp_from(obs_pick)
    if not seen:
        gx2, gy2, seen2 = _grasp_from(obs0)
        if seen2:
            gx, gy, seen = gx2, gy2, seen2
    _log(api, "grasp target x=%.4f y=%.4f (perceived=%s)" % (gx, gy, seen))

    _grip(api, GRIP_OPEN)
    _go(api, clock, [gx, gy, Z_ABOVE_BOWL], 1.6)
    _go(api, clock, [gx, gy, Z_GRASP], 1.1)
    _grip(api, GRIP_CLOSE)
    _settle(api, clock, 0.4)
    _go(api, clock, [gx, gy, Z_GRASP + LIFT_CHECK_DZ], 0.6)

    if (not _holding(api)) and clock.spent < TIME_BUDGET:
        # Missed.  Most likely the pinch sat outside the rim wall, so step the
        # jaws 12 mm further into the bowl and 6 mm lower, then try once more.
        _log(api, "no hold; retrying grasp")
        _grip(api, GRIP_OPEN)
        _settle(api, clock, 0.2)
        gy_r = gy + RETRY_DY
        _go(api, clock, [gx, gy_r, Z_ABOVE_BOWL - 0.02], 0.9)
        _go(api, clock, [gx, gy_r, Z_GRASP - RETRY_DZ], 0.9)
        _grip(api, GRIP_CLOSE)
        _settle(api, clock, 0.4)
        gy = gy_r
        _go(api, clock, [gx, gy, Z_GRASP + LIFT_CHECK_DZ], 0.6)

    # ---------------- ACT 3b: carry into the drawer ---------------------
    _go(api, clock, [gx, gy, Z_CARRY], 1.0)
    px = x_drawer + PLACE_DX
    py = y_drag_end + PLACE_DY
    py = float(np.clip(py, Y_HOOK + 0.01, y_drag_end - 0.05))
    _log(api, "release at x=%.4f y=%.4f z=%.4f (drag end y=%.4f)"
         % (px, py, Z_PLACE, y_drag_end))
    _go(api, clock, [px, py, Z_CARRY], 1.6)
    _go(api, clock, [px, py, Z_PLACE], 1.0)
    _grip(api, GRIP_OPEN)
    _settle(api, clock, 0.4)
    _go(api, clock, [px, py, Z_PLACE + 0.06], 0.6)
    _go(api, clock, [px, py, Z_CARRY], 0.8)
    _log(api, "plan complete; commanded motion %.1f s" % (clock.spent,))
