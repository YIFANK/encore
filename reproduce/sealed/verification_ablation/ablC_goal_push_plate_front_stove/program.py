"""abl_c2 / cell ablC_goal_push_plate_front_stove  (variant C -- no verification).

Intent: "push the plate to the front of the stove".

Everything below is derived from packs/ablC_goal_push_plate_front_stove/ (pack.json +
keyframes/*.png) and from generic controller / pinhole-camera mechanics.  ZERO
episodes were run while authoring this file, and no runtime termination or
outcome signal is consulted anywhere; the only feedback used is the program's
own re-perception of the scene.

Mechanism
---------
1.  Perceive from `cam_high` with the arm parked at its start pose.  The RGB
    frame is block-averaged 4x down to 128x128 so that every colour threshold
    below is applied to *exactly* the same quantity that was measured on the
    pack keyframes (which are 128x128 renders of the same camera).
2.  The plate is the large, round, low-saturation blob whose brightness sits in
    the narrow band measured on the pack keyframes and which deprojects to
    table height.  The stove is located through its burner: the dark, round,
    well-filled disc that is *ringed by bright neutral pixels* (the white stove
    top).  That halo test is what separates the burner from the cabinet, the
    bottle and the table shadows -- on the pack keyframes the burner scores
    0.62-0.64 and every other dark blob scores at most 0.32.
3.  A homography from image pixels onto the table plane is fitted at run time
    from a grid of deprojected bare-table points.  The goal pose of the plate
    is expressed as a fixed pixel offset from the burner (measured on the three
    demos: plate centroid in the final keyframe minus burner centroid in the
    first) and mapped through that homography, so the offset is applied in the
    same projective frame in which it was measured, and the parallax of the
    raised stove top cancels between measurement and use.
4.  The open gripper is placed just behind the plate on the line towards the
    goal, lowered to the push height that all three demos hold constant, and
    translated by exactly (goal - plate).  Then the arm parks, re-perceives and
    repeats while the plate is short of the goal, inside a fixed motion budget.
5.  If perception fails outright, the program replays the pack's own push
    trajectory open loop.
"""

import math

import numpy as np

# --------------------------------------------------------------------------
# Calibrated constants.  Every one of these is declared in PROVENANCE below.
# --------------------------------------------------------------------------

Z_PUSH = 0.918                       # base-frame z held constant through the push
Z_TRAVEL = 1.140                     # base-frame z used for free-space traverse
PARK_XYZ = (-0.206, 0.003, 1.167)    # arm start pose (clear of the table view)
GRIP_OPEN_M = 0.0724                 # demo gripper aperture (stays open throughout)

NEUTRAL_SAT_MAX = 14                 # max(RGB)-min(RGB) of the neutral props
PLATE_V_LO, PLATE_V_HI = 150.0, 182.0   # plate interior mean-RGB band
BRIGHT_V_MIN = 175.0                 # stove top mean-RGB floor
DARK_V_LO, DARK_V_HI = 40.0, 105.0   # burner disc mean-RGB band

PLATE_WIDE_LO, PLATE_WIDE_HI = 120.0, 210.0   # relaxed band used with the rim test
RED_DR_MIN, RED_DB_MIN = 22.0, 30.0  # plate rim is R-G ~46, R-B ~55; wood table is 16 / 34
RED_V_MAX = 185.0                    # the rim is darker than the bare table
RED_HALO_MIN = 0.08                  # fraction of the plate's halo that is rim-red

PLATE_MIN_PX128 = 70                 # plate blob is ~137-141 px at 128x128
PLATE_WIDE_MIN_PX128 = 55
BURNER_MIN_PX128 = 60                # burner blob is ~146-149 px at 128x128
BLOB_MAX_PX128 = 900                 # nothing this program looks for is bigger
BURNER_HALO_MIN = 0.45               # fraction of the burner's halo that is stove-white
BLOB_MAX_ASPECT = 2.6                # foreshortened circles stay well under this
BLOB_MIN_FILL = 0.55                 # area / bbox-area of a filled disc

TARGET_OFFSET_PX128 = (4.13, 21.53)  # goal plate centroid minus burner centroid
SLAB_OFFSET_PX128 = (3.99, 19.60)    # same, measured against the stove-top blob
NOMINAL_BURNER_PX128 = (91.83, 57.43)
NOMINAL_PLATE_PX128 = (59.77, 90.27)

RIM_RADIUS_FACTOR = 1.27             # full plate width / neutral-interior width
CONTACT_CLEARANCE_M = 0.015          # finger stand-off outside the plate rim
PLATE_R_FALLBACK_M = 0.075

WS_X_LO, WS_X_HI = -0.356, 0.248     # demo end-effector travel, padded by 0.15
WS_Y_LO, WS_Y_HI = -0.186, 0.442
TABLE_BAND_M = 0.045                 # a table-height object sits within this of the plane
ELEVATED_MIN_M = 0.012               # the stove top rises at least this far
TABLE_Z_WIN = (Z_PUSH - 0.15, Z_PUSH + 0.10)   # where the table plane must lie

PUSH_SEG_M = 0.05                    # straight-line push chopped into 5 cm steps
PUSH_SEG_S = 0.70
TOL_M = 0.025                        # stop correcting once the plate is this close
MAX_ATTEMPTS = 3
MOTION_BUDGET_S = 20.0               # soft cap on total commanded motion time
ATTEMPT_RESERVE_S = 5.5              # do not start a sweep without this much left

DEMO_PUSH_PATH = (                   # pack demo1 ee_path6, push phase, for open-loop replay
    (0.0371, -0.0221), (0.0440, 0.0042), (0.0612, 0.0580), (0.0814, 0.0977),
    (0.0918, 0.1409), (0.0811, 0.2057), (0.0624, 0.2580), (-0.0161, 0.2760),
    (-0.0957, 0.2577), (-0.1339, 0.2583),
)

PROVENANCE = {
    "Z_PUSH": {"source": "pack.json ee_path6: z is held at 0.9157-0.9236 through the "
                         "whole push phase of all three demos (mean 0.918)",
               "allowed": True},
    "Z_TRAVEL": {"source": "pack.json ee_path6: free-space z between t=0 and t=20 in all "
                           "three demos (1.09-1.17); 1.14 clears every prop the demos fly over",
                 "allowed": True},
    "PARK_XYZ": {"source": "pack.json demo0 keyframe t=0 ee = [-0.2061, 0.0033, 1.167]",
                 "allowed": True},
    "GRIP_OPEN_M": {"source": "pack.json keyframe gripper_state [0.0362, -0.0362] with "
                              "gripper_cmd -1.0 held for every frame of every demo",
                    "allowed": True},
    "NEUTRAL_SAT_MAX": {"source": "pack keyframes: plate/stove/burner pixels have "
                                  "max(RGB)-min(RGB) <= 5 while the wood table has 34",
                        "allowed": True},
    "PLATE_V_LO": {"source": "pack keyframes demo0/1/2 t=0: plate interior mean-RGB 168-171",
                   "allowed": True},
    "PLATE_V_HI": {"source": "pack keyframes demo0/1/2 t=0: plate interior mean-RGB 168-171, "
                             "stove top starts at 175", "allowed": True},
    "BRIGHT_V_MIN": {"source": "pack keyframes: stove top surface mean-RGB 175-227",
                     "allowed": True},
    "DARK_V_LO": {"source": "pack keyframes: burner disc mean-RGB 46-95", "allowed": True},
    "DARK_V_HI": {"source": "pack keyframes: burner disc mean-RGB 46-95", "allowed": True},
    "PLATE_WIDE_LO": {"source": "pack keyframes: relaxed brightness band that still brackets "
                                "the plate interior (165) once the rim test is doing the work",
                      "allowed": True},
    "PLATE_WIDE_HI": {"source": "pack keyframes: relaxed brightness band that still brackets "
                                "the plate interior (165) once the rim test is doing the work",
                      "allowed": True},
    "RED_DR_MIN": {"source": "pack keyframe demo0 t=0 pixel scan: plate rim pixels are "
                             "168/122/113 (R-G = 46) while the wood table is 193/177/159 "
                             "(R-G = 16)", "allowed": True},
    "RED_DB_MIN": {"source": "pack keyframe demo0 t=0 pixel scan: plate rim R-B = 55, wood "
                             "table R-B = 34", "allowed": True},
    "RED_V_MAX": {"source": "pack keyframe demo0 t=0 pixel scan: the rim (mean-RGB ~134) is "
                            "darker than the bare table (~176)", "allowed": True},
    "RED_HALO_MIN": {"source": "pack keyframes: the 3 px halo around the plate blob is "
                               "0.33-0.36 rim-red in the three t=0 frames and 0.10-0.37 in the "
                               "three final frames; every other round neutral blob in all six "
                               "frames scores 0.00", "allowed": True},
    "PLATE_MIN_PX128": {"source": "pack keyframes: plate blob area 137/137/141 px at 128x128",
                        "allowed": True},
    "PLATE_WIDE_MIN_PX128": {"source": "pack keyframes: the plate blob under the relaxed band "
                                       "is 142-146 px at 128x128", "allowed": True},
    "BURNER_MIN_PX128": {"source": "pack keyframes: burner blob area 148/149/146 px at 128x128",
                         "allowed": True},
    "BLOB_MAX_PX128": {"source": "pack keyframes: plate and burner blobs are <=150 px at "
                                 "128x128; the cabinet face is ~400 px and is rejected here",
                       "allowed": True},
    "BURNER_HALO_MIN": {"source": "pack keyframes: bright-neutral fraction of the 3 px halo "
                                  "around the burner is 0.62-0.64; every other dark blob in "
                                  "the three t=0 frames scores <= 0.32", "allowed": True},
    "BLOB_MAX_ASPECT": {"source": "pack keyframes: plate bbox 15x11/16x12/16x11, burner bbox "
                                  "17x11; the stove's front shadow strip is 28x3 and is "
                                  "rejected here", "allowed": True},
    "BLOB_MIN_FILL": {"source": "pack keyframes: area/bbox-area is 0.71-0.84 for the plate "
                                "and 0.74-0.80 for the burner", "allowed": True},
    "TARGET_OFFSET_PX128": {"source": "pack keyframes: plate blob centroid in the final "
                                      "keyframe minus burner blob centroid in the t=0 "
                                      "keyframe, per demo (3.0,21.7)/(4.1,22.2)/(5.3,20.7); "
                                      "mean used", "allowed": True},
    "SLAB_OFFSET_PX128": {"source": "pack keyframes: same measurement taken against the "
                                    "bright stove-top blob centroid (3.0,19.8)/(4.1,20.3)/"
                                    "(4.9,18.7); mean used", "allowed": True},
    "NOMINAL_BURNER_PX128": {"source": "pack keyframes t=0: burner centroid "
                                       "(91.0,57.0)/(93.0,58.0)/(91.5,57.3); mean used",
                             "allowed": True},
    "NOMINAL_PLATE_PX128": {"source": "pack keyframes t=0: plate centroid "
                                      "(61.9,90.0)/(59.7,89.8)/(57.7,91.0); mean used",
                            "allowed": True},
    "RIM_RADIUS_FACTOR": {"source": "pack keyframe demo0 t=0 pixel row scan: full plate spans "
                                    "19 px including the reddish rim, the neutral interior "
                                    "spans 15 px", "allowed": True},
    "CONTACT_CLEARANCE_M": {"source": "generic gripper mechanics: stand-off so the descending "
                                      "fingers land beside the rim instead of on it",
                            "allowed": True},
    "PLATE_R_FALLBACK_M": {"source": "pack.json: over the push the plate travels with the "
                                     "end-effector (~0.28 m) across ~37 px at 128x128, giving "
                                     "~7.6 px per 0.05 m and a 19 px plate radius of ~0.075 m",
                           "allowed": True},
    "WS_X_LO": {"source": "pack.json ee_path6 x range [-0.206, 0.098] padded by 0.15",
                "allowed": True},
    "WS_X_HI": {"source": "pack.json ee_path6 x range [-0.206, 0.098] padded by 0.15",
                "allowed": True},
    "WS_Y_LO": {"source": "pack.json ee_path6 y range [-0.036, 0.292] padded by 0.15",
                "allowed": True},
    "WS_Y_HI": {"source": "pack.json ee_path6 y range [-0.036, 0.292] padded by 0.15",
                "allowed": True},
    "TABLE_BAND_M": {"source": "generic depth mechanics: a plate lying on the table deprojects "
                               "within a few cm of the fitted table plane", "allowed": True},
    "ELEVATED_MIN_M": {"source": "generic depth mechanics: the stove top is a raised slab, so "
                                 "its burner deprojects above the fitted table plane",
                       "allowed": True},
    "TABLE_Z_WIN": {"source": "pack.json ee_path6 push height 0.918 is by construction just "
                              "above the table, so the table plane lies in a window around it",
                    "allowed": True},
    "PUSH_SEG_M": {"source": "generic OSC mechanics: short straight-line increments keep the "
                             "controller on the commanded chord", "allowed": True},
    "PUSH_SEG_S": {"source": "pack.json: the demo push covers ~0.28 m in ~75 control steps, "
                             "i.e. a slow steady sweep; 0.05 m per 0.70 s matches it",
                   "allowed": True},
    "TOL_M": {"source": "generic closed-loop tolerance, set just under the pack's own "
                        "demo-to-demo spread of the final plate pose (~0.02 m)",
              "allowed": True},
    "MAX_ATTEMPTS": {"source": "generic budgeting: the demo push costs ~130 control steps, so "
                               "a small number of sweeps fits a LIBERO-length episode",
                     "allowed": True},
    "MOTION_BUDGET_S": {"source": "generic budgeting: the pack's demos are 125-155 control "
                                  "steps, so the commanded motion time is capped at a few "
                                  "times one demo", "allowed": True},
    "ATTEMPT_RESERVE_S": {"source": "generic budgeting: a corrective sweep costs about this "
                                    "much commanded motion time", "allowed": True},
    "DEMO_PUSH_PATH": {"source": "pack.json demo1 ee_path6, push phase t=40..127, used only as "
                                 "an open-loop fallback if perception fails", "allowed": True},
}


# --------------------------------------------------------------------------
# small image helpers
# --------------------------------------------------------------------------

def _to128(rgb):
    """Block-average an HxWx3 frame down to a 128x128 float RGB image."""
    a = np.asarray(rgb).astype(np.float64)
    h, w = a.shape[0], a.shape[1]
    f = max(1, int(round(min(h, w) / 128.0)))
    if f > 1 and h % f == 0 and w % f == 0:
        a = a.reshape(h // f, f, w // f, f, 3).mean(axis=(1, 3))
    return a


def _label(mask):
    """4-connected connected components on a small boolean image."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    for y0 in range(h):
        for x0 in range(w):
            if not mask[y0, x0] or lab[y0, x0]:
                continue
            cur += 1
            lab[y0, x0] = cur
            stack = [(y0, x0)]
            while stack:
                y, x = stack.pop()
                if y > 0 and mask[y - 1, x] and not lab[y - 1, x]:
                    lab[y - 1, x] = cur
                    stack.append((y - 1, x))
                if y + 1 < h and mask[y + 1, x] and not lab[y + 1, x]:
                    lab[y + 1, x] = cur
                    stack.append((y + 1, x))
                if x > 0 and mask[y, x - 1] and not lab[y, x - 1]:
                    lab[y, x - 1] = cur
                    stack.append((y, x - 1))
                if x + 1 < w and mask[y, x + 1] and not lab[y, x + 1]:
                    lab[y, x + 1] = cur
                    stack.append((y, x + 1))
    return lab, cur


def _blobs(mask, min_size):
    lab, n = _label(mask)
    out = []
    for i in range(1, n + 1):
        m = lab == i
        size = int(m.sum())
        if size < min_size:
            continue
        yy, xx = np.nonzero(m)
        u0, u1 = int(xx.min()), int(xx.max())
        v0, v1 = int(yy.min()), int(yy.max())
        bw, bh = u1 - u0 + 1, v1 - v0 + 1
        out.append({
            "size": size, "u": float(xx.mean()), "v": float(yy.mean()),
            "u0": u0, "u1": u1, "v0": v0, "v1": v1,
            "aspect": max(bw / float(bh), bh / float(bw)),
            "fill": size / float(bw * bh), "mask": m,
        })
    out.sort(key=lambda b: -b["size"])
    return out


def _halo_fraction(blob_mask, bright):
    d = blob_mask.copy()
    for _ in range(3):
        e = np.zeros_like(d)
        e[1:, :] |= d[:-1, :]
        e[:-1, :] |= d[1:, :]
        e[:, 1:] |= d[:, :-1]
        e[:, :-1] |= d[:, 1:]
        d = d | e
    ring = d & ~blob_mask
    tot = int(ring.sum())
    if tot == 0:
        return 0.0
    return float((ring & bright).sum()) / float(tot)


def _round_blob(b):
    return (b["aspect"] <= BLOB_MAX_ASPECT and b["fill"] >= BLOB_MIN_FILL
            and b["size"] <= BLOB_MAX_PX128)


# --------------------------------------------------------------------------
# table-plane homography  (pixel -> base xy)
# --------------------------------------------------------------------------

def _fit_homography(pix, xy):
    pix = np.asarray(pix, float)
    xy = np.asarray(xy, float)
    n = pix.shape[0]
    if n < 8:
        return None

    def norm(p):
        c = p.mean(axis=0)
        d = np.sqrt(((p - c) ** 2).sum(axis=1)).mean()
        if d < 1e-9:
            return None, None
        s = math.sqrt(2.0) / d
        t = np.array([[s, 0.0, -s * c[0]], [0.0, s, -s * c[1]], [0.0, 0.0, 1.0]])
        return t, np.column_stack([(p - c) * s, np.ones(len(p))])

    t1, p1 = norm(pix)
    t2, p2 = norm(xy)
    if t1 is None or t2 is None:
        return None
    rows = []
    for i in range(n):
        u, v = p1[i, 0], p1[i, 1]
        x, y = p2[i, 0], p2[i, 1]
        rows.append([-u, -v, -1.0, 0.0, 0.0, 0.0, x * u, x * v, x])
        rows.append([0.0, 0.0, 0.0, -u, -v, -1.0, y * u, y * v, y])
    try:
        _, _, vt = np.linalg.svd(np.asarray(rows, float))
        hn = vt[-1].reshape(3, 3)
        h = np.linalg.inv(t2) @ hn @ t1
    except Exception:
        return None
    if not np.all(np.isfinite(h)) or abs(h[2, 2]) < 1e-12:
        return None
    return h / h[2, 2]


def _apply_h(h, u, v):
    q = h @ np.array([float(u), float(v), 1.0])
    if abs(q[2]) < 1e-9:
        return None
    out = np.array([q[0] / q[2], q[1] / q[2]])
    return out if np.all(np.isfinite(out)) else None


# --------------------------------------------------------------------------
# perception
# --------------------------------------------------------------------------

class Scene(object):
    def __init__(self):
        self.ok = False
        self.table_z = None
        self.plate_xy = None
        self.plate_r = PLATE_R_FALLBACK_M
        self.goal_xy = None
        self.plate_seen = False
        self.goal_seen = False


def _safe_deproject(api, u, v):
    try:
        p = np.asarray(api.deproject(float(u), float(v)), float).reshape(3)
        return p if np.all(np.isfinite(p)) else None
    except Exception:
        return None


def _perceive(api):
    sc = Scene()
    try:
        rgb = np.asarray(api.capture("cam_high").rgb)
    except Exception:
        return sc
    if rgb.ndim != 3 or rgb.shape[2] < 3:
        return sc
    img_h, img_w = int(rgb.shape[0]), int(rgb.shape[1])
    small = _to128(rgb[:, :, :3])
    vmap = small.mean(axis=2)
    smap = small.max(axis=2) - small.min(axis=2)
    redmap = ((small[:, :, 0] - small[:, :, 1] >= RED_DR_MIN)
              & (small[:, :, 0] - small[:, :, 2] >= RED_DB_MIN)
              & (vmap <= RED_V_MAX))
    scale = img_w / 128.0

    def to_px(u_small, v_small):
        return ((u_small + 0.5) * scale - 0.5, (v_small + 0.5) * scale - 0.5)

    # --- table plane -------------------------------------------------------
    grid_pix, grid_xyz = [], []
    step = max(8, int(round(img_w / 32.0)))
    for vv in range(int(img_h * 0.30), img_h - 2, step):
        for uu in range(2, img_w - 2, step):
            p = _safe_deproject(api, uu, vv)
            if p is None:
                continue
            grid_pix.append((uu, vv))
            grid_xyz.append(p)
    if len(grid_xyz) < 20:
        return sc
    grid_pix = np.asarray(grid_pix, float)
    grid_xyz = np.asarray(grid_xyz, float)

    zs = grid_xyz[:, 2]
    cand = zs[(zs >= TABLE_Z_WIN[0]) & (zs <= TABLE_Z_WIN[1])]
    if cand.size < 12:
        cand = zs
    lo, hi = float(cand.min()), float(cand.max())
    if hi - lo < 1e-6:
        table_z = float(np.median(cand))
    else:
        counts, edges = np.histogram(cand, bins=40, range=(lo, hi))
        k = int(np.argmax(counts))
        sel = cand[(cand >= edges[k]) & (cand <= edges[k + 1])]
        table_z = float(np.median(sel)) if sel.size else float(np.median(cand))
    sc.table_z = table_z

    flat = np.abs(zs - table_z) < 0.008
    homo = None
    if int(flat.sum()) >= 12:
        homo = _fit_homography(grid_pix[flat], grid_xyz[flat][:, :2])
    if homo is None:
        homo = _fit_homography(grid_pix, grid_xyz[:, :2])

    def ground(u, v):
        if homo is not None:
            q = _apply_h(homo, u, v)
            if q is not None:
                return q
        p = _safe_deproject(api, u, v)
        return None if p is None else p[:2]

    def height_small(u_small, v_small):
        p = _safe_deproject(api, *to_px(u_small, v_small))
        return None if p is None else float(p[2]) - table_z

    def in_ws(q, pad):
        return (q is not None and WS_X_LO - pad <= q[0] <= WS_X_HI + pad
                and WS_Y_LO - pad <= q[1] <= WS_Y_HI + pad)

    neutral = smap <= NEUTRAL_SAT_MAX
    bright = neutral & (vmap >= BRIGHT_V_MIN)

    # --- burner (stove reference) -----------------------------------------
    burner = None
    for lo_v, hi_v, halo_min in ((DARK_V_LO, DARK_V_HI, BURNER_HALO_MIN),
                                 (DARK_V_LO - 12.0, DARK_V_HI + 25.0, 0.35)):
        best = None
        for b in _blobs(neutral & (vmap >= lo_v) & (vmap <= hi_v), BURNER_MIN_PX128):
            if not _round_blob(b):
                continue
            halo = _halo_fraction(b["mask"], bright)
            if halo < halo_min:
                continue
            dz = height_small(b["u"], b["v"])
            if dz is not None and dz < ELEVATED_MIN_M - 0.02:
                continue
            if best is None or halo > best[0]:
                best = (halo, b)
        if best is not None:
            burner = best[1]
            break

    if burner is not None:
        goal_px = to_px(burner["u"] + TARGET_OFFSET_PX128[0],
                        burner["v"] + TARGET_OFFSET_PX128[1])
        sc.goal_seen = True
    else:
        slab = None
        for b in _blobs(bright, 90):
            if b["size"] > 1400:
                continue
            dz = height_small(b["u"], b["v"])
            if dz is not None and dz < ELEVATED_MIN_M - 0.02:
                continue
            slab = b
            break
        if slab is not None:
            goal_px = to_px(slab["u"] + SLAB_OFFSET_PX128[0],
                            slab["v"] + SLAB_OFFSET_PX128[1])
            sc.goal_seen = True
        else:
            goal_px = to_px(NOMINAL_BURNER_PX128[0] + TARGET_OFFSET_PX128[0],
                            NOMINAL_BURNER_PX128[1] + TARGET_OFFSET_PX128[1])
    goal_xy = ground(goal_px[0], goal_px[1])
    if not in_ws(goal_xy, 0.30):
        goal_px = to_px(NOMINAL_BURNER_PX128[0] + TARGET_OFFSET_PX128[0],
                        NOMINAL_BURNER_PX128[1] + TARGET_OFFSET_PX128[1])
        goal_xy = ground(goal_px[0], goal_px[1])
        sc.goal_seen = False
    sc.goal_xy = None if goal_xy is None else np.asarray(goal_xy, float)

    # --- plate -------------------------------------------------------------
    def plate_ok(b):
        dz = height_small(b["u"], b["v"])
        if dz is not None and (dz > TABLE_BAND_M or dz < -0.03):
            return False
        return in_ws(ground(*to_px(b["u"], b["v"])), 0.30)

    plate = None
    # tier A: the only round neutral disc whose halo carries the plate's red rim
    best = None
    for b in _blobs(neutral & (vmap >= PLATE_WIDE_LO) & (vmap <= PLATE_WIDE_HI),
                    PLATE_WIDE_MIN_PX128):
        if not _round_blob(b):
            continue
        rh = _halo_fraction(b["mask"], redmap)
        if rh < RED_HALO_MIN:
            continue
        if not plate_ok(b):
            continue
        if best is None or rh > best[0]:
            best = (rh, b)
    if best is not None:
        plate = best[1]
    # tier B/C: fall back on the plate's brightness band alone
    if plate is None:
        for lo_v, hi_v, min_sz in ((PLATE_V_LO, PLATE_V_HI, PLATE_MIN_PX128),
                                   (PLATE_WIDE_LO, PLATE_WIDE_HI, PLATE_WIDE_MIN_PX128)):
            cand = None
            for b in _blobs(neutral & (vmap >= lo_v) & (vmap <= hi_v), min_sz):
                if not _round_blob(b) or not plate_ok(b):
                    continue
                if cand is None or b["size"] > cand["size"]:
                    cand = b
            if cand is not None:
                plate = cand
                break

    if plate is not None:
        pu, pv = to_px(plate["u"], plate["v"])
        direct = _safe_deproject(api, pu, pv)
        via_h = ground(pu, pv)
        plate_xy = None
        if direct is not None and abs(float(direct[2]) - table_z) < TABLE_BAND_M:
            plate_xy = np.asarray(direct[:2], float)
            if via_h is not None and np.linalg.norm(plate_xy - np.asarray(via_h)) > 0.06:
                plate_xy = np.asarray(via_h, float)
        elif via_h is not None:
            plate_xy = np.asarray(via_h, float)
        if in_ws(plate_xy, 0.30):
            sc.plate_xy = plate_xy
            sc.plate_seen = True
            left = ground(*to_px(plate["u0"] - 0.5, plate["v"]))
            right = ground(*to_px(plate["u1"] + 0.5, plate["v"]))
            if left is not None and right is not None:
                r = 0.5 * float(np.linalg.norm(np.asarray(left) - np.asarray(right)))
                r *= RIM_RADIUS_FACTOR
                if 0.035 <= r <= 0.14:
                    sc.plate_r = r
    if sc.plate_xy is None:
        q = ground(*to_px(NOMINAL_PLATE_PX128[0], NOMINAL_PLATE_PX128[1]))
        if q is not None:
            sc.plate_xy = np.asarray(q, float)

    sc.ok = sc.plate_xy is not None and sc.goal_xy is not None
    return sc


# --------------------------------------------------------------------------
# motion
# --------------------------------------------------------------------------

class Budget(object):
    def __init__(self, total):
        self.left = float(total)

    def spend(self, s):
        self.left -= float(s)


def _clamp_xy(xy):
    return np.array([min(max(float(xy[0]), WS_X_LO), WS_X_HI),
                     min(max(float(xy[1]), WS_Y_LO), WS_Y_HI)])


def _move(api, xyz, seconds, budget):
    budget.spend(seconds)
    try:
        api.move([float(xyz[0]), float(xyz[1]), float(xyz[2])], seconds=float(seconds))
    except Exception:
        pass


def _lift(api, budget, seconds=1.0):
    try:
        e = np.asarray(api.eef(), float).reshape(3)
        _move(api, (e[0], e[1], Z_TRAVEL), seconds, budget)
    except Exception:
        _move(api, (PARK_XYZ[0], PARK_XYZ[1], Z_TRAVEL), seconds, budget)


def _park(api, budget):
    _move(api, PARK_XYZ, 1.5, budget)
    try:
        api.settle(0.2)
    except Exception:
        pass


def _sweep(api, start_xy, delta, budget, approach_s):
    """Descend behind the plate at start_xy and translate by delta at push height."""
    s = _clamp_xy(start_xy)
    _move(api, (s[0], s[1], Z_TRAVEL), approach_s, budget)
    _move(api, (s[0], s[1], Z_PUSH), 1.4, budget)
    try:
        api.settle(0.15)
    except Exception:
        pass
    dist = float(np.linalg.norm(delta))
    nseg = max(1, int(math.ceil(dist / PUSH_SEG_M)))
    for i in range(1, nseg + 1):
        w = _clamp_xy(s + np.asarray(delta, float) * (float(i) / nseg))
        _move(api, (w[0], w[1], Z_PUSH), PUSH_SEG_S, budget)
    _lift(api, budget, 1.0)


def _open_loop_demo_push(api, budget):
    """Last-resort replay of the pack's own push trajectory."""
    p0 = DEMO_PUSH_PATH[0]
    _move(api, (p0[0], p0[1], Z_TRAVEL), 2.0, budget)
    _move(api, (p0[0], p0[1], Z_PUSH), 1.4, budget)
    for x, y in DEMO_PUSH_PATH[1:]:
        _move(api, (x, y, Z_PUSH), 0.8, budget)
    _lift(api, budget, 1.0)


# --------------------------------------------------------------------------
# entry point
# --------------------------------------------------------------------------

def run(api):
    budget = Budget(MOTION_BUDGET_S)
    try:
        api.log("push plate to front of stove :: %s" % (api.instruction(),))
    except Exception:
        pass
    try:
        api.grip(GRIP_OPEN_M)
        api.settle(0.2)
    except Exception:
        pass

    swept = 0
    for attempt in range(MAX_ATTEMPTS):
        if budget.left < ATTEMPT_RESERVE_S:
            break
        sc = _perceive(api)
        if not sc.ok:
            if swept == 0:
                _open_loop_demo_push(api, budget)
                swept += 1
            break

        delta = sc.goal_xy - sc.plate_xy
        dist = float(np.linalg.norm(delta))
        try:
            api.log("attempt %d plate=(%.3f,%.3f) goal=(%.3f,%.3f) r=%.3f d=%.3f "
                    "table_z=%.3f seen=%s/%s budget=%.1f"
                    % (attempt, sc.plate_xy[0], sc.plate_xy[1], sc.goal_xy[0], sc.goal_xy[1],
                       sc.plate_r, dist,
                       sc.table_z if sc.table_z is not None else float("nan"),
                       sc.plate_seen, sc.goal_seen, budget.left))
        except Exception:
            pass

        if dist <= TOL_M:
            break
        if dist > 0.60:
            # implausible for this scene -- fall back rather than fling the arm
            if swept == 0:
                _open_loop_demo_push(api, budget)
                swept += 1
            break

        unit = delta / dist
        start_xy = sc.plate_xy - unit * (sc.plate_r + CONTACT_CLEARANCE_M)
        _sweep(api, start_xy, delta, budget, 2.0 if attempt == 0 else 1.6)
        swept += 1

        if attempt < MAX_ATTEMPTS - 1 and budget.left >= ATTEMPT_RESERVE_S:
            _park(api, budget)

    _lift(api, budget, 1.0)
