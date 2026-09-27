"""v2 -- first end-to-end attempt at "Put the cream cheese on the rack".

Mechanism, split across the two packs exactly as the brief describes them:
  * mate pack ("put the cream cheese IN THE BOWL") supplies the OBJECT half:
    the cream cheese is taken with a straight-down wrist, closing on it at
    eef z ~= 0.911 (demo0 t40 z=0.9104, demo2 t42 z=0.9106).
  * k3 pack ("put the WINE BOTTLE on the rack") supplies the TARGET half:
    every demo releases over the rack's upper shelf at y ~= -0.26..-0.28,
    z ~= 1.19..1.22 -- i.e. onto the upper plank, a little down-slope from
    its high back edge.

Everything metric is re-derived here from the live cam_high RGB-D.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "GRASP_CLEAR_M": {
        "source": "mate pack keyframes: eef z at the closing keyframe "
                  "(demo0 t40 0.9104, demo1 t45 0.9205, demo2 t42 0.9106) "
                  "minus the table height I measure in the same frame "
                  "(0.9009 from debug seeds 51/53/55/57) -> ~0.010 m",
        "allowed": True},
    "BLUE_MASK": {
        "source": "debug seeds 51/53/55/57 cam_high RGB: the cream cheese is "
                  "the only blue-dominant object on the table "
                  "(B/sum>0.42, B>R+18, B>G+12); 253-260 top-face pixels/seed",
        "allowed": True},
    "WOOD_MASK": {
        "source": "debug seeds 51/53/55/57 cam_high RGB: rack slats are warm "
                  "(R-B>12, 70<R<200) vs the neutral cabinet (R-B~4); the "
                  "cropped cloud z in [1.00,1.35], y<-0.02 isolates the rack",
        "allowed": True},
    "SHELF_Z_BAND": {
        "source": "debug-seed measurement: the rack's two planks are parallel "
                  "tilted planes; the UPPER one spans z 1.13..1.24 and the "
                  "lower one 1.00..1.08, so z>1.115 selects the upper shelf",
        "allowed": True},
    "DROP_DOWNSLOPE_M": {
        "source": "k3 pack release keyframes (demo0 t182 y=-0.281, demo1 t173 "
                  "y=-0.273, demo2 t183 y=-0.234) sit 0.03-0.08 m down-slope "
                  "of the shelf's high back edge I measure at y~=-0.31",
        "allowed": True},
    "DROP_CLEAR_M": {
        "source": "debug-seed measurement: the cream cheese's underside sits "
                  "0.010 m below the eef when held (GRASP_CLEAR_M), so a "
                  "0.015 m eef offset above the plank leaves ~5 mm of drop",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug-seed measurement: tallest things on the carry line "
                  "are the cabinet top (1.134) and the rack top (1.243); k3 "
                  "demos themselves transit at z 1.24-1.29",
        "allowed": True},
    "TABLE_PATCH": {
        "source": "debug-seed measurement: x in [0.05,0.25], y in [-0.15,0.15] "
                  "is bare table in every seed observed; its median z is the "
                  "table height (0.9009)",
        "allowed": True},
}

GRASP_CLEAR_M = 0.010
SHELF_Z_BAND = 1.115
DROP_DOWNSLOPE_M = 0.05
DROP_CLEAR_M = 0.015
CARRY_Z = 1.29
TABLE_PATCH = ((0.05, 0.25), (-0.15, 0.15))
CHUNK = 1900


def _dump(api, tag, arr):
    b = base64.b64encode(zlib.compress(np.ascontiguousarray(arr).tobytes(), 9)).decode()
    api.log("ARR %s shape=%s dtype=%s nchunk=%d"
            % (tag, list(arr.shape), str(arr.dtype), (len(b) + CHUNK - 1) // CHUNK))
    for i in range(0, len(b), CHUNK):
        api.log("D %s %d %s" % (tag, i // CHUNK, b[i:i + CHUNK]))


def _cloud(f):
    d = np.asarray(f.depth, np.float64)
    K, T = np.asarray(f.intrinsics, float), np.asarray(f.t_base_cam, float)
    h, w = d.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    bad = ~np.isfinite(d) | (d <= 0)
    d = np.where(bad, 1.0, d)
    P = np.stack([(u - K[0, 2]) * d / K[0, 0],
                  (v - K[1, 2]) * d / K[1, 1], d, np.ones_like(d)], -1)
    B = (P @ T.T)[..., :3]
    B[bad] = np.nan
    return B


def _table_z(B):
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    (x0, x1), (y0, y1) = TABLE_PATCH
    m = (X > x0) & (X < x1) & (Y > y0) & (Y < y1) & np.isfinite(Z)
    return float(np.median(Z[m]))


def find_cheese(rgb, B, table_z):
    """The cream cheese: the only blue-dominant thing standing on the table."""
    f = rgb.astype(float)
    R, G, Bl = f[..., 0], f[..., 1], f[..., 2]
    s = f.sum(-1) + 1e-6
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    m = ((Bl / s > 0.42) & (Bl > R + 18) & (Bl > G + 12) & (s > 60)
         & np.isfinite(Z) & (Z > table_z + 0.004) & (Z < table_z + 0.15)
         & (X > -0.45) & (X < 0.30) & (Y > -0.45) & (Y < 0.45))
    if m.sum() < 30:
        return None
    top = m & (Z > Z[m].max() - 0.006)      # top face only, no oblique side
    if top.sum() < 20:
        top = m
    return {"n": int(m.sum()), "ntop": int(top.sum()),
            "x": float(np.median(X[top])), "y": float(np.median(Y[top])),
            "ztop": float(np.nanmax(Z[m])),
            "dx": float(X[top].max() - X[top].min()),
            "dy": float(Y[top].max() - Y[top].min())}


def find_shelf(rgb, B):
    """The rack's upper plank: warm-coloured points in the upper z band."""
    f = rgb.astype(float)
    R, Bl = f[..., 0], f[..., 2]
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    wood = ((R - Bl > 12) & (R > 70) & (R < 200) & np.isfinite(Z)
            & (Z > 1.00) & (Z < 1.35)
            & (X > -0.55) & (X < 0.05) & (Y > -0.45) & (Y < -0.02))
    up = wood & (Z > SHELF_Z_BAND)
    if up.sum() < 200:
        return None
    x, y, z = X[up], Y[up], Z[up]
    y_back = float(np.percentile(y, 3))     # high (back) edge of the plank
    return {"n": int(up.sum()), "y_back": y_back,
            "x_med": float(np.median(x)),
            "x_lo": float(np.percentile(x, 3)), "x_hi": float(np.percentile(x, 97)),
            "y_front": float(np.percentile(y, 97)),
            "z_back": float(np.percentile(z, 97)), "z_front": float(np.percentile(z, 3)),
            "pts": (x, y, z)}


def surface_z(shelf, xq, yq, r=0.025):
    """Measured plank height at (xq,yq): median z of nearby plank points."""
    x, y, z = shelf["pts"]
    d = (x - xq) ** 2 + (y - yq) ** 2
    sel = d < r * r
    if sel.sum() < 10:
        sel = d < (2 * r) ** 2
    return float(np.median(z[sel]))


def run(api):
    api.log("INSTRUCTION %r" % api.instruction())
    f = api.capture("cam_high")
    B = _cloud(f)
    tz = _table_z(B)
    api.log("TABLE_Z %.4f" % tz)

    ch = find_cheese(f.rgb, B, tz)
    api.log("CHEESE %s" % ch)
    sh = find_shelf(f.rgb, B)
    api.log("SHELF %s" % {k: v for k, v in (sh or {}).items() if k != "pts"})
    if ch is None or sh is None:
        return "perception failed"

    # ---- pick -------------------------------------------------------------
    gx, gy = ch["x"], ch["y"]
    gz = tz + GRASP_CLEAR_M
    api.grip(0.08)
    api.log("R1 %.4f" % api.move([gx, gy, max(gz + 0.08, 1.00)], seconds=3.0))
    api.log("R2 %.4f" % api.move([gx, gy, gz], seconds=2.5))
    api.log("PRE_EEF %s" % np.round(api.eef(), 4).tolist())
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("GRIP_AFTER_CLOSE %s" % g)

    # ---- carry ------------------------------------------------------------
    api.log("R3 %.4f" % api.move([gx, gy, CARRY_Z], seconds=3.0))
    g2 = api.gripper()
    api.log("GRIP_AFTER_LIFT %s eef=%s" % (g2, np.round(api.eef(), 4).tolist()))

    dx = float(np.clip(sh["x_med"], max(sh["x_lo"] + 0.04, -0.26), min(sh["x_hi"] - 0.04, -0.14)))
    dy = sh["y_back"] + DROP_DOWNSLOPE_M
    sz = surface_z(sh, dx, dy)
    api.log("DROP target x=%.4f y=%.4f surface_z=%.4f" % (dx, dy, sz))

    api.log("R4 %.4f" % api.move([dx, dy, CARRY_Z], seconds=3.5))
    api.log("MID_EEF %s" % np.round(api.eef(), 4).tolist())
    api.log("R5 %.4f" % api.move([dx, dy, sz + GRASP_CLEAR_M + DROP_CLEAR_M], seconds=2.5))
    api.log("PRE_REL_EEF %s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(0.08)
    api.settle(0.6)
    api.log("POST_REL_EEF %s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))

    # ---- retreat and look at what happened --------------------------------
    api.move([dx, dy + 0.06, CARRY_Z], seconds=3.0)
    api.move([-0.21, 0.10, CARRY_Z], seconds=3.0)
    api.settle(0.8)
    f2 = api.capture("cam_high")
    B2 = _cloud(f2)
    ch2 = find_cheese(f2.rgb, B2, tz)
    api.log("CHEESE_AFTER %s" % ch2)
    _dump(api, "after_rgb", f2.rgb)
    _dump(api, "after_depth", np.asarray(f2.depth, np.float32).astype(np.float16))
    return "v2 done"
