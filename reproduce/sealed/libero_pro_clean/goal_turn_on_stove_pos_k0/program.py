"""v2 -- calibrate the fingertip offset, then grasp the knob bar and try to
twist it about the disc axis.  Heavy logging + a before/after RGB-D dump."""

import base64
import zlib

import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51/53/55/57: cam_high cloud, clear table "
                          "patch x[0.00,0.12] y[0.28,0.42] reads z=0.902",
                "allowed": True},
    "CLEAR_XY": {"source": "debug seeds 51-57: that same patch holds no points "
                           "above the table plane",
                 "allowed": True},
    "DARK_LUM": {"source": "debug seed 51/53 cam_high: the knob disc+bar are the "
                           "only sub-50 luminance pixels above the table in "
                           "x[-0.14,0.30], y[-0.12,0.45]",
                 "allowed": True},
    "SEARCH_BOX": {"source": "debug seeds 51-57: knob dark blob sits in "
                             "x[-0.11,0.01] y[0.07,0.18]; the arm sits at "
                             "x<-0.15 so the box excludes it",
                   "allowed": True},
    "BAR_BAND": {"source": "debug seed 51 cross-section: bar top 0.961, disc top "
                           "~0.920, so the raised bar is the top ~0.02 band",
                 "allowed": True},
    "CHUNK": {"source": "generic: api.log truncates at 2000 chars", "allowed": True},
}

TABLE_Z = 0.902
CLEAR_XY = (0.06, 0.34)
DARK_LUM = 50.0
SEARCH_BOX = (-0.14, 0.30, -0.12, 0.45)
BAR_BAND = 0.020
CHUNK = 1800


# --------------------------------------------------------------- log datapipe
def _emit(api, tag, arr):
    b64 = base64.b64encode(zlib.compress(np.ascontiguousarray(arr).tobytes(), 6)).decode()
    api.log(f"DUMP {tag} dtype={arr.dtype} shape={list(arr.shape)} "
            f"nchunks={(len(b64) + CHUNK - 1) // CHUNK}")
    for i in range(0, len(b64), CHUNK):
        api.log(f"D {tag} {i // CHUNK} {b64[i:i + CHUNK]}")


def _dump(api, f, tag):
    rgb = np.asarray(f.rgb)
    depth = np.asarray(f.depth, float)
    s = max(1, depth.shape[0] // 256)
    d_mm = np.clip(np.where(np.isfinite(depth) & (depth > 0), depth * 1000.0, 0.0),
                   0, 65535).astype(np.uint16)
    api.log(f"K {tag} " + " ".join(f"{v:.6f}" for v in np.asarray(f.intrinsics).ravel()))
    api.log(f"T {tag} " + " ".join(f"{v:.6f}" for v in np.asarray(f.t_base_cam).ravel()))
    _emit(api, f"{tag}_rgb", rgb[::s, ::s])
    _emit(api, f"{tag}_d", d_mm[::s, ::s])


# ------------------------------------------------------------------ perception
def _cloud(f):
    depth = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = depth.shape
    vs, us = np.mgrid[0:h, 0:w]
    z = np.where(np.isfinite(depth) & (depth > 0), depth, 0.0)
    x = (us - K[0, 2]) * z / K[0, 0]
    y = (vs - K[1, 2]) * z / K[1, 1]
    pts = np.stack([x, y, z, np.ones_like(z)], -1) @ T.T
    return pts[..., :3], np.asarray(f.rgb), z > 0


def _label(mask):
    """4-connected labelling (no scipy in the sandbox)."""
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    h, w = mask.shape
    for v0 in range(h):
        for u0 in range(w):
            if not mask[v0, u0] or lab[v0, u0]:
                continue
            cur += 1
            stack = [(v0, u0)]
            lab[v0, u0] = cur
            while stack:
                v, u = stack.pop()
                for dv, du in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    a, b = v + dv, u + du
                    if 0 <= a < h and 0 <= b < w and mask[a, b] and not lab[a, b]:
                        lab[a, b] = cur
                        stack.append((a, b))
    return lab, cur


def find_knob(api):
    f = api.capture("cam_high")
    P, rgb, ok = _cloud(f)
    lum = rgb.astype(float).mean(-1)
    x0, x1, y0, y1 = SEARCH_BOX
    m = (ok & (lum < DARK_LUM) & (P[..., 2] > TABLE_Z + 0.004)
         & (P[..., 0] > x0) & (P[..., 0] < x1)
         & (P[..., 1] > y0) & (P[..., 1] < y1))
    lab, n = _label(m)
    best, best_tall = None, 0
    for i in range(1, n + 1):
        sel = lab == i
        if sel.sum() < 80:
            continue
        p = P[sel]
        span = max(np.ptp(p[:, 0]), np.ptp(p[:, 1]))
        if span > 0.20:
            continue
        tall = int((p[:, 2] > TABLE_Z + 0.030).sum())
        api.log(f"CAND {i}: n={int(sel.sum())} tall={tall} "
                f"x[{p[:, 0].min():.3f},{p[:, 0].max():.3f}] "
                f"y[{p[:, 1].min():.3f},{p[:, 1].max():.3f}] ztop={p[:, 2].max():.3f}")
        if tall > best_tall:
            best, best_tall = p, tall
    if best is None:
        return None, f
    disc_c = np.array([(best[:, 0].min() + best[:, 0].max()) / 2.0,
                       (best[:, 1].min() + best[:, 1].max()) / 2.0])
    ztop = float(best[:, 2].max())
    bar = best[best[:, 2] > ztop - BAR_BAND]
    bc = bar[:, :2].mean(0)
    d = bar[:, :2] - bc
    cov = d.T @ d / max(1, len(d))
    w_, V = np.linalg.eigh(cov)
    axis = V[:, int(np.argmax(w_))]
    ang = float(np.degrees(np.arctan2(axis[1], axis[0])) % 180.0)
    info = dict(disc_c=disc_c, ztop=ztop, bar_c=bc, bar_ang=ang,
                bar_len=float(np.sqrt(max(w_)) * 2 * np.sqrt(3)),
                bar_wid=float(np.sqrt(min(w_)) * 2 * np.sqrt(3)),
                n=len(best), n_bar=len(bar))
    api.log("KNOB " + " ".join(f"{k}={np.round(v, 4).tolist() if hasattr(v, 'tolist') else round(v, 4)}"
                               for k, v in info.items()))
    return info, f


# ---------------------------------------------------------------------- motion
def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _say(api, tag, res=None):
    e = api.eef()
    g = api.gripper()
    api.log(f"{tag} eef=[{e[0]:.4f},{e[1]:.4f},{e[2]:.4f}] "
            f"gap={g['width_m']:.4f} eff={g['effort']:.2f}"
            + ("" if res is None else f" res={res:.4f}"))


def run(api):
    api.log(f"INSTRUCTION {api.instruction()!r}")
    R0 = np.asarray(api.tool_rotation(), float)
    api.log("R0 " + " ".join(f"{v:.4f}" for v in R0.ravel()))

    knob, f0 = find_knob(api)
    _dump(api, f0, "before")
    if knob is None:
        api.log("NO KNOB FOUND")
        return

    # --- A: fingertip offset from a blocked descent on clear table -----------
    api.grip(0.0)
    cx, cy = CLEAR_XY
    r = api.move([cx, cy, 1.05], seconds=2.0)
    _say(api, "CAL_HOVER", r)
    r = api.move([cx, cy, 0.86], seconds=2.5)
    _say(api, "CAL_DOWN", r)
    z_block = float(api.eef()[2])
    tip_off = z_block - TABLE_Z
    api.log(f"TIP_OFFSET {tip_off:.4f}  (blocked eef z {z_block:.4f} - table {TABLE_Z})")
    r = api.move([cx, cy, 1.06], seconds=2.0)
    _say(api, "CAL_UP", r)

    # --- B: grasp the bar at the disc axis -----------------------------------
    ax = np.asarray(knob["disc_c"], float)
    z_bar_mid = knob["ztop"] - 0.012          # a little below the bar top
    z_eef_grasp = z_bar_mid + tip_off
    api.log(f"GRASP_TARGET xy={ax.round(4).tolist()} z_bar_mid={z_bar_mid:.4f} "
            f"z_eef={z_eef_grasp:.4f}")
    api.grip(0.08)
    r = api.move([ax[0], ax[1], z_eef_grasp + 0.09], seconds=2.0)
    _say(api, "HOVER", r)
    r = api.move([ax[0], ax[1], z_eef_grasp], seconds=2.0)
    _say(api, "DESCEND", r)
    api.grip(0.0)
    api.settle(0.3)
    _say(api, "CLOSED")

    # --- C: twist about the disc axis ----------------------------------------
    for deg in (30, 60, 90):
        R = _rz(np.radians(deg)) @ R0
        r = api.move([ax[0], ax[1], z_eef_grasp], rotation=R, seconds=2.0)
        _say(api, f"TWIST{deg}", r)

    api.grip(0.08)
    api.settle(0.3)
    r = api.move([ax[0], ax[1], z_eef_grasp + 0.12], rotation=R0, seconds=2.0)
    _say(api, "RETREAT", r)
    r = api.move([-0.18, -0.05, 1.15], rotation=R0, seconds=2.0)
    _say(api, "PARK", r)

    k2, f2 = find_knob(api)
    _dump(api, f2, "after")
    api.log("V2 DONE")
