"""v4 -- v3 (15/15 on debug) plus fallbacks that only fire when the strict
path finds nothing: a splitter for two fused bowl blobs, a relaxed plate
filter, and a three-rung grasp ladder gated on the measured gripper width.

Model derived from the pack + debug seeds 51-65:
  * the scene holds two identical bowls (top z 0.952, 0.111 dia), a ramekin
    (0.944, 0.088), a plate (0.920, 0.136, bright grey) and a cookie box
    (0.920, 0.085x0.060, brown) on a table plane at z ~0.902.
  * "table center" = the bowl nearest the centre of the visible table surface.
  * the demo grasps the target bowl at bowl_centre + (-0.0128, +0.0409), i.e.
    it straddles the +y wall, at z 0.9194, and releases at z 0.9342 with the
    bowl centre over the plate centre.
"""
import numpy as np

PROVENANCE = {
    "WS": {"source": "debug seeds 51-65 (v1/v2): every prop lies inside x[-0.45,0.45], y[-0.45,0.45]", "allowed": True},
    "GRID": {"source": "generic: 5 mm top-down occupancy grid", "allowed": True},
    "BOWL_LO/BOWL_HI": {"source": "debug seeds 51-65 (v2): bowl tops 0.952; ramekin 0.944; plate/box 0.920 -- band (0.9455,0.9650) selects bowls only", "allowed": True},
    "BOWL_EXT": {"source": "debug seeds 51-65 (v2): both bowls measure 0.110x0.110 top-down", "allowed": True},
    "PLATE_LO/PLATE_HI/PLATE_EXT/PLATE_MIN_CELLS/PLATE_MIN_RGB": {"source": "debug seeds 51-65 (v2): plate blob = 460-560 cells, ext 0.135-0.140, ztop 0.920, mean rgb ~(158,151,149); the brown cookie box at the same height has min channel ~46", "allowed": True},
    "GRASP_OFF": {"source": "pack demo0: grasp eef (-0.0908,0.0409) minus the bowl centre (-0.078,0.000) measured at that site on debug seeds", "allowed": True},
    "GRASP_Z": {"source": "pack demo0 ee_path6[4] z=0.9194, the lowest point of the descent where gripper_cmd flips to close", "allowed": True},
    "RELEASE_Z": {"source": "pack demo0 keyframe t=99 z=0.9342, where gripper_cmd flips to open", "allowed": True},
    "CARRY_Z": {"source": "generic clearance: the bowl bottom hangs 0.018 below the eef (GRASP_Z - table), so 1.02 clears the 0.952 bowl tops", "allowed": True},
    "HOVER_Z": {"source": "generic pre-grasp hover above the 0.952 bowl rim", "allowed": True},
    "OPEN_W": {"source": "pack demo0 gripper_state at t=0: 2*0.0362 = 0.0724 open", "allowed": True},
    "HELD_W": {"source": "debug seeds 51-65 (v3): a successful close on the bowl wall reads width 0.0094-0.0095; the same close on empty air reads 0.0046", "allowed": True},
    "grasp ladder": {"source": "pack demo0 offset, its mirror through the bowl centre (same straddle, opposite wall), and an 8 mm deeper descent", "allowed": True},
    "plate fallback / bowl split": {"source": "debug seeds 51-65 (v2/v3) blob statistics; fallbacks only fire when the strict filters return nothing", "allowed": True},
}

WS = (-0.45, 0.45, -0.45, 0.45)
GRID = 0.005
BOWL_LO, BOWL_HI = 0.9455, 0.9650
BOWL_EXT = (0.085, 0.135)
PLATE_LO, PLATE_HI = 0.9080, 0.9280
PLATE_EXT = (0.110, 0.175)
PLATE_MIN_CELLS = 250
PLATE_MIN_RGB = 120.0
GRASP_OFF = np.array([-0.0128, 0.0409])
GRASP_Z = 0.9194
RELEASE_Z = 0.9342
CARRY_Z = 1.02
HOVER_Z = 1.00
OPEN_W = 0.0724
HELD_W = 0.007


# ---------------------------------------------------------------- perception
def _cloud(f):
    d = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    p = np.stack([x, y, d, np.ones_like(d)], -1) @ T.T
    return p[..., :3], np.isfinite(d) & (d > 0), np.asarray(f.rgb)


def _grid(xyz, ok, rgb):
    nx = int(round((WS[1] - WS[0]) / GRID))
    ny = int(round((WS[3] - WS[2]) / GRID))
    sel = ok & (xyz[..., 0] >= WS[0]) & (xyz[..., 0] < WS[1]) \
        & (xyz[..., 1] >= WS[2]) & (xyz[..., 1] < WS[3])
    p = xyz[sel]
    c = rgb[sel].astype(np.float64)
    ix = ((p[:, 0] - WS[0]) / GRID).astype(int)
    iy = ((p[:, 1] - WS[2]) / GRID).astype(int)
    flat = ix * ny + iy
    order = np.argsort(p[:, 2])
    zm = np.full(nx * ny, -np.inf)
    cm = np.zeros((nx * ny, 3))
    zm[flat[order]] = p[order, 2]
    cm[flat[order]] = c[order]
    return zm.reshape(nx, ny), cm.reshape(nx, ny, 3)


def _blobs(mask, min_cells=8):
    h, w = mask.shape
    seen = np.zeros_like(mask)
    out = []
    for sy, sx in np.argwhere(mask):
        if seen[sy, sx]:
            continue
        st = [(sy, sx)]
        seen[sy, sx] = True
        comp = []
        while st:
            y, x = st.pop()
            comp.append((y, x))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    a, b = y + dy, x + dx
                    if 0 <= a < h and 0 <= b < w and mask[a, b] and not seen[a, b]:
                        seen[a, b] = True
                        st.append((a, b))
        if len(comp) >= min_cells:
            out.append(np.array(comp))
    return out


def _describe(comp, zm, cm):
    xs = WS[0] + (comp[:, 0] + 0.5) * GRID
    ys = WS[2] + (comp[:, 1] + 0.5) * GRID
    return {"n": len(comp),
            "c": np.array([(xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2]),
            "ex": xs.max() - xs.min(), "ey": ys.max() - ys.min(),
            "ztop": float(zm[comp[:, 0], comp[:, 1]].max()),
            "rgb": cm[comp[:, 0], comp[:, 1]].mean(0)}


def perceive(api):
    f = api.capture("cam_high")
    xyz, ok, rgb = _cloud(f)
    zm, cm = _grid(xyz, ok, rgb)
    fin = zm[np.isfinite(zm)]
    hist, edges = np.histogram(fin, bins=300)
    table_z = float(edges[int(np.argmax(hist))] + (edges[1] - edges[0]) / 2)

    tm = np.isfinite(zm) & (np.abs(zm - table_z) < 0.006)
    ti, tj = np.nonzero(tm)
    txs = WS[0] + (ti + 0.5) * GRID
    tys = WS[2] + (tj + 0.5) * GRID
    tcen = np.array([(txs.min() + txs.max()) / 2, (tys.min() + tys.max()) / 2])

    bowls, plates = [], []
    for comp in _blobs(np.isfinite(zm) & (zm > BOWL_LO) & (zm < BOWL_HI)):
        d = _describe(comp, zm, cm)
        big = max(d["ex"], d["ey"])
        if BOWL_EXT[0] <= d["ex"] <= BOWL_EXT[1] and BOWL_EXT[0] <= d["ey"] <= BOWL_EXT[1]:
            bowls.append(d)
        elif BOWL_EXT[1] < big <= 2 * BOWL_EXT[1] and min(d["ex"], d["ey"]) >= BOWL_EXT[0]:
            # two bowls touching: split along the long axis at the midpoint
            ax = 0 if d["ex"] >= d["ey"] else 1
            xs = WS[0] + (comp[:, 0] + 0.5) * GRID
            ys = WS[2] + (comp[:, 1] + 0.5) * GRID
            v = xs if ax == 0 else ys
            mid = (v.min() + v.max()) / 2
            for side in (v < mid, v >= mid):
                if side.sum() >= 40:
                    bowls.append(_describe(comp[side], zm, cm))

    pm = np.isfinite(zm) & (zm > PLATE_LO) & (zm < PLATE_HI)
    cands = [_describe(c, zm, cm) for c in _blobs(pm, min_cells=120)]
    for d in cands:
        if (PLATE_EXT[0] <= d["ex"] <= PLATE_EXT[1]
                and PLATE_EXT[0] <= d["ey"] <= PLATE_EXT[1]
                and d["n"] >= PLATE_MIN_CELLS
                and d["rgb"].min() >= PLATE_MIN_RGB):
            plates.append(d)
    if not plates:
        # relaxed fallback: the brightest sizeable flat blob that is not the
        # brown cookie box and not a bowl footprint
        loose = [d for d in cands
                 if d["n"] >= 150 and 0.09 <= d["ex"] <= 0.22 and 0.09 <= d["ey"] <= 0.22
                 and d["rgb"].min() >= 90.0
                 and d["rgb"][0] - d["rgb"][2] < 25.0]
        if loose:
            plates.append(max(loose, key=lambda d: d["n"]))
    return table_z, tcen, bowls, plates


# --------------------------------------------------------------------- motion
def run(api):
    table_z, tcen, bowls, plates = perceive(api)
    api.log("table_z=%.4f tcen=(%.3f,%.3f) nbowl=%d nplate=%d"
            % (table_z, tcen[0], tcen[1], len(bowls), len(plates)))
    for b in bowls:
        api.log("bowl n=%d c=(%.3f,%.3f) ext=(%.3f,%.3f) ztop=%.3f rgb=%s d_tcen=%.3f"
                % (b["n"], b["c"][0], b["c"][1], b["ex"], b["ey"], b["ztop"],
                   [round(float(v)) for v in b["rgb"]],
                   float(np.linalg.norm(b["c"] - tcen))))
    for p in plates:
        api.log("plate n=%d c=(%.3f,%.3f) ext=(%.3f,%.3f) ztop=%.3f rgb=%s"
                % (p["n"], p["c"][0], p["c"][1], p["ex"], p["ey"], p["ztop"],
                   [round(float(v)) for v in p["rgb"]]))
    if not bowls or not plates:
        api.log("ABORT: bowls=%d plates=%d" % (len(bowls), len(plates)))
        return
    target = min(bowls, key=lambda b: np.linalg.norm(b["c"] - tcen))
    plate = max(plates, key=lambda p: p["n"])
    api.log("TARGET bowl=(%.3f,%.3f) PLATE=(%.3f,%.3f)"
            % (target["c"][0], target["c"][1], plate["c"][0], plate["c"][1]))

    api.grip(OPEN_W)
    g = target["c"] + GRASP_OFF
    api.move([g[0], g[1], HOVER_Z], seconds=2.5)
    api.log("hover eef=%s rot=%s" % (np.round(api.eef(), 4).tolist(),
                                     np.round(api.tool_rotation(), 3).tolist()))
    # grasp ladder: the demo offset first, then the same straddle rotated to
    # the opposite wall, then a slightly deeper descent on the demo offset.
    ladder = [(GRASP_OFF, GRASP_Z),
              (np.array([-GRASP_OFF[0], -GRASP_OFF[1]]), GRASP_Z),
              (GRASP_OFF, GRASP_Z - 0.008)]
    held = False
    used_off = GRASP_OFF
    for k, (off, gz) in enumerate(ladder):
        g = target["c"] + off
        api.move([g[0], g[1], HOVER_Z], seconds=2.0)
        api.move([g[0], g[1], gz], seconds=2.0)
        api.grip(0.0)
        api.settle(0.4)
        gr = api.gripper()
        api.log("try%d off=(%.4f,%.4f) z=%.4f eef=%s width=%.4f effort=%.2f"
                % (k, off[0], off[1], gz, np.round(api.eef(), 4).tolist(),
                   gr["width_m"], gr["effort"]))
        if gr["effort"] >= 1.0 and gr["width_m"] >= HELD_W:
            held = True
            used_off = off
            break
        api.grip(OPEN_W)
        api.settle(0.2)
    if not held:
        api.log("GRASP LADDER EXHAUSTED -- continuing with the last attempt")

    api.move([g[0], g[1], CARRY_Z], seconds=2.0)
    api.settle(0.3)
    gr = api.gripper()
    api.log("lifted eef=%s width=%.4f effort=%.2f"
            % (np.round(api.eef(), 4).tolist(), gr["width_m"], gr["effort"]))

    r = plate["c"] + used_off
    api.move([r[0], r[1], CARRY_Z], seconds=3.0)
    api.move([r[0], r[1], RELEASE_Z], seconds=2.0)
    api.log("at release eef=%s" % np.round(api.eef(), 4).tolist())
    api.grip(OPEN_W)
    api.settle(0.4)
    api.move([r[0], r[1], CARRY_Z], seconds=2.0)
    api.move([r[0] - 0.12, r[1], CARRY_Z], seconds=2.0)
    api.settle(0.5)

    # self-verification: where did the bowl end up relative to the plate?
    tz2, tc2, b2, p2 = perceive(api)
    for b in b2:
        api.log("POST bowl c=(%.3f,%.3f) ztop=%.3f ext=(%.3f,%.3f) d_plate=%.3f"
                % (b["c"][0], b["c"][1], b["ztop"], b["ex"], b["ey"],
                   float(np.linalg.norm(b["c"] - plate["c"]))))
    for p in p2:
        api.log("POST plate c=(%.3f,%.3f) ztop=%.3f n=%d"
                % (p["c"][0], p["c"][1], p["ztop"], p["n"]))
