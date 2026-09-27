"""c2 / goal_put_cream_cheese_in_bowl_stock — v3 (= v2 + off-nominal guards)

v2 receipt: probe 8/8 (fs_..._v2), formal selection 15/15
(sel_c2_goal_put_cream_cheese_in_bowl_stock_v2).  v3 adds ONLY guards that
cannot fire on the nominal path measured on seeds 51-65 (shape gates around
the measured cluster statistics, an anchor fallback if no gated candidate is
within D_MAX, and a re-grasp if the close does not reach holding effort), so
nominal behaviour is bit-identical to v2 while blind-seed anomalies degrade
gracefully instead of mis-targeting.

--- v2 header ---

v1 diagnosis (results/fs_..._v1, 0/8): the pick worked on every episode
(gripper closed to 0.042 m at effort 3.0 and held through transit) but the
bowl was never isolated — plain above-table clustering chained bowl + bottle +
robot arm + wall into one 0.45 m-tall blob, so the release happened next to
the pick point.  v2 replaces image/point connectivity with PER-CELL MAX HEIGHT
gating (C2-L5) computed over the *unbanded* cloud, so tall props (robot 0.45,
cabinet 0.31, bottle) are removed cell-wise before clustering, and logs an
ASCII height map of the workspace for diagnosis.
"""
import numpy as np

PROVENANCE = {
    "CHEESE_ANCHOR": {
        "source": "pack.json demos[*].keyframes: mean xy of the 3 gripper-close "
                  "keyframes (t=40/45/42) = (-0.02753, 0.13357)",
        "allowed": True},
    "BOWL_ANCHOR": {
        "source": "pack.json demos[*].keyframes: mean xy of the 3 gripper-open "
                  "(release) keyframes (t=75/95/85) = (-0.07497, -0.01203)",
        "allowed": True},
    "GRASP_Z": {
        "source": "pack.json demos[*].ee_path minimum z around the close "
                  "keyframe (0.9104/0.9097/0.9102) -> 0.9105; confirmed on "
                  "debug seeds 51-65 v1 (close -> effort 3.0 on 8/8)",
        "allowed": True},
    "RELEASE_Z": {
        "source": "pack.json demos[*].keyframes ee z at the release keyframe "
                  "(0.9592/0.9731/0.9785) -> 0.970",
        "allowed": True},
    "APPROACH_Z": {
        "source": "pack.json demos[*].ee_path z just before the descent "
                  "(1.0243/1.0816/0.9926) -> 1.02",
        "allowed": True},
    "TRAVEL_Z": {
        "source": "pack.json demos[*].ee_path max z between grasp and release "
                  "(1.0415/1.0790/1.0510) -> 1.06",
        "allowed": True},
    "OPEN_W": {
        "source": "generic gripper command; api doc says <0.025 closes, else "
                  "opens. Measured open width 0.0778 m on debug seed 51",
        "allowed": True},
    "CELL": {
        "source": "generic footprint-clustering grid size 0.015 m (C2-L2)",
        "allowed": True},
    "ROI": {
        "source": "debug-seed 51-65 v1 cloud statistics: workspace window that "
                  "contains both demo anchors and the props around them",
        "allowed": True},
    "TABLE_BAND": {
        "source": "debug-seed measurement: modal cloud z inside ROI = 0.900 on "
                  "seed 51 (v1 log); band brackets it",
        "allowed": True},
    "H_CHEESE": {
        "source": "debug-seed 51 v1 measurement: the cluster 0.011 m from the "
                  "cheese anchor has max height 0.020 m above the table",
        "allowed": True},
    "H_BOWL": {
        "source": "debug-seed measurement (v2 logs, seeds 51-65): the vessel "
                  "cluster at the release anchor has max height 0.052 m; the "
                  "abutting flat disc is ~0.01 m, so the 0.028 floor separates "
                  "them (C2-L5)",
        "allowed": True},
    "N_CHEESE/EXT_CHEESE": {
        "source": "debug-seed measurement (v2 logs, seeds 51-65): the cheese "
                  "cluster is n=1069-1145 points, extent 0.080x0.042 m on "
                  "15/15 seeds; gate set generously around that",
        "allowed": True},
    "N_BOWL_MIN/EXT_BOWL": {
        "source": "debug-seed measurement (v2 logs, seeds 51-65): the bowl "
                  "cluster is n=2204-2307 points, extent 0.110x0.112 m on "
                  "15/15 seeds; gate set generously around that",
        "allowed": True},
    "D_MAX": {
        "source": "debug-seed measurement (v2 logs, seeds 51-65): chosen "
                  "cluster sits 0.003-0.043 m from its demo anchor while the "
                  "nearest competitor sits 0.13-0.26 m away; 0.09 splits them",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "debug-seed measurement: api.gripper() effort is 3.0 while "
                  "holding and 0.05 when empty (v1/v2 logs); 2.0 splits them",
        "allowed": True},
}

CHEESE_ANCHOR = np.array([-0.02753, 0.13357])
BOWL_ANCHOR = np.array([-0.07497, -0.01203])
GRASP_Z = 0.9105
RELEASE_Z = 0.970
APPROACH_Z = 1.02
TRAVEL_Z = 1.06
OPEN_W = 0.08
CELL = 0.015
ROI = dict(xlo=-0.22, xhi=0.32, ylo=-0.32, yhi=0.36)
TABLE_BAND = (0.75, 1.05)
H_CHEESE = (0.010, 0.075)
H_BOWL = (0.028, 0.100)
N_CHEESE = (200, 3500)
EXT_CHEESE = 0.13
N_BOWL_MIN = 800
EXT_BOWL = (0.06, 0.17)
D_MAX = 0.09
HOLD_EFFORT = 2.0


# --------------------------------------------------------------------------
def _cloud(frame):
    d = np.asarray(frame.depth, float)
    if d.ndim == 3:
        d = d[..., 0]
    h, w = d.shape[:2]
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    good = np.isfinite(d) & (d > 1e-4)
    z = np.where(good, d, 1.0)
    xc = (uu - K[0, 2]) * z / K[0, 0]
    yc = (vv - K[1, 2]) * z / K[1, 1]
    P = np.stack([xc, yc, z], -1).reshape(-1, 3)
    Pb = P @ T[:3, :3].T + T[:3, 3]
    return Pb, good.reshape(-1)


def _table_z(p):
    zz = p[:, 2]
    zz = zz[(zz > TABLE_BAND[0]) & (zz < TABLE_BAND[1])]
    if zz.size < 100:
        return None
    hist, edges = np.histogram(zz, bins=np.arange(TABLE_BAND[0], TABLE_BAND[1], 0.004))
    k = int(np.argmax(hist))
    return float(0.5 * (edges[k] + edges[k + 1]))


def _cellmap(p, col, tz):
    """Per-cell max height over the UNBANDED above-table cloud (C2-L5)."""
    sel = ((p[:, 0] > ROI["xlo"]) & (p[:, 0] < ROI["xhi"]) &
           (p[:, 1] > ROI["ylo"]) & (p[:, 1] < ROI["yhi"]) &
           (p[:, 2] > tz + 0.006) & (p[:, 2] < tz + 0.80))
    q = p[sel]
    c = col[sel]
    ij = np.floor(q[:, :2] / CELL).astype(np.int64)
    keys, inv = np.unique(ij, axis=0, return_inverse=True)
    hmax = np.zeros(len(keys))
    np.maximum.at(hmax, inv, q[:, 2] - tz)
    return keys, inv, hmax, q, c


def _components(keys, hmax, band):
    ok = np.nonzero((hmax > band[0]) & (hmax < band[1]))[0]
    kmap = {(int(keys[i][0]), int(keys[i][1])): i for i in ok}
    seen, lab_of, lab = set(), {}, 0
    for start in ok:
        k0 = (int(keys[start][0]), int(keys[start][1]))
        if k0 in seen:
            continue
        stack = [k0]
        seen.add(k0)
        while stack:
            a, b = stack.pop()
            lab_of[(a, b)] = lab
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    nb = (a + da, b + db)
                    if nb in kmap and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        lab += 1
    return lab_of, lab


def _stats(keys, inv, q, c, lab_of, nlab, band, tz):
    cell_lab = np.full(len(keys), -1, int)
    kidx = {(int(keys[i][0]), int(keys[i][1])): i for i in range(len(keys))}
    for k, L in lab_of.items():
        cell_lab[kidx[k]] = L
    plab = cell_lab[inv]
    hh = q[:, 2] - tz
    out = []
    for L in range(nlab):
        m = (plab == L) & (hh < band[1] + 0.02)
        if m.sum() < 20:
            continue
        r = q[m]
        out.append(dict(
            n=int(m.sum()),
            cen=r[:, :2].mean(0),
            mid=np.array([0.5 * (r[:, 0].min() + r[:, 0].max()),
                          0.5 * (r[:, 1].min() + r[:, 1].max())]),
            ext=np.array([r[:, 0].max() - r[:, 0].min(),
                          r[:, 1].max() - r[:, 1].min()]),
            top=float((r[:, 2] - tz).max()),
            rgb=c[m].mean(0)))
    out.sort(key=lambda d: -d["n"])
    return out


def _fmt(c):
    return ("n=%4d cen=(%.3f,%.3f) mid=(%.3f,%.3f) ext=(%.3f,%.3f) h=%.3f "
            "rgb=(%3d,%3d,%3d)" % (c["n"], c["cen"][0], c["cen"][1], c["mid"][0],
                                   c["mid"][1], c["ext"][0], c["ext"][1],
                                   c["top"], c["rgb"][0], c["rgb"][1], c["rgb"][2]))


def _map(api, keys, hmax):
    """ASCII height map: rows = x (top row = xlo), cols = y (left col = ylo).
    Matches the camera view (image right = +y, image down = +x)."""
    i0 = int(np.floor(ROI["xlo"] / CELL))
    j0 = int(np.floor(ROI["ylo"] / CELL))
    ni = int(np.ceil((ROI["xhi"] - ROI["xlo"]) / CELL))
    nj = int(np.ceil((ROI["yhi"] - ROI["ylo"]) / CELL))
    g = np.full((ni, nj), -1.0)
    for n in range(len(keys)):
        i, j = int(keys[n][0]) - i0, int(keys[n][1]) - j0
        if 0 <= i < ni and 0 <= j < nj:
            g[i, j] = hmax[n]
    api.log("map x=%.2f..%.2f (rows) y=%.2f..%.2f (cols) cell=%.3f"
            % (ROI["xlo"], ROI["xhi"], ROI["ylo"], ROI["yhi"], CELL))
    for i in range(ni):
        row = "".join("." if g[i, j] < 0 else
                      ("%d" % min(9, int(g[i, j] / 0.02))) for j in range(nj))
        api.log("  x=%+.3f |%s|" % ((i + i0) * CELL, row))


# --------------------------------------------------------------------------
def run(api):
    log = api.log
    f = api.capture("cam_high")
    p, good = _cloud(f)
    col = np.asarray(f.rgb, float).reshape(-1, 3)
    p, col = p[good], col[good]
    tz = _table_z(p)
    log("table_z=%.4f npts=%d" % (tz, p.shape[0]))

    keys, inv, hmax, q, c = _cellmap(p, col, tz)
    _map(api, keys, hmax)

    lo, nl = _components(keys, hmax, H_CHEESE)
    cl_c = _stats(keys, inv, q, c, lo, nl, H_CHEESE, tz)
    log("cheese-band clusters (%d):" % len(cl_c))
    for cc in cl_c[:10]:
        log("  %s d=%.3f" % (_fmt(cc), np.linalg.norm(cc["cen"] - CHEESE_ANCHOR)))

    lo, nl = _components(keys, hmax, H_BOWL)
    cl_b = _stats(keys, inv, q, c, lo, nl, H_BOWL, tz)
    log("bowl-band clusters (%d):" % len(cl_b))
    for cc in cl_b[:10]:
        log("  %s d=%.3f" % (_fmt(cc), np.linalg.norm(cc["cen"] - BOWL_ANCHOR)))

    # ---- shape-gated, anchor-nearest selection with anchor fallback --------
    gc = [d for d in cl_c if N_CHEESE[0] <= d["n"] <= N_CHEESE[1]
          and max(d["ext"]) <= EXT_CHEESE and d["top"] <= 0.05]
    cheese = (min(gc, key=lambda d: np.linalg.norm(d["cen"] - CHEESE_ANCHOR))
              if gc else None)
    if cheese is None or np.linalg.norm(cheese["cen"] - CHEESE_ANCHOR) > D_MAX:
        log("GUARD cheese fallback to anchor (cands=%d)" % len(gc))
        cxy = CHEESE_ANCHOR.copy()
    else:
        cxy = cheese["mid"]
        log("PICK  %s" % _fmt(cheese))

    gb = [d for d in cl_b if d["n"] >= N_BOWL_MIN
          and EXT_BOWL[0] <= max(d["ext"]) <= EXT_BOWL[1]]
    bowl = (min(gb, key=lambda d: np.linalg.norm(d["cen"] - BOWL_ANCHOR))
            if gb else None)
    if bowl is None or np.linalg.norm(bowl["cen"] - BOWL_ANCHOR) > D_MAX:
        log("GUARD bowl fallback to anchor (cands=%d)" % len(gb))
        bxy = BOWL_ANCHOR.copy()
    else:
        bxy = bowl["mid"]
        log("PLACE %s" % _fmt(bowl))

    api.grip(OPEN_W)
    api.move([cxy[0], cxy[1], APPROACH_Z], seconds=2.0)
    api.move([cxy[0], cxy[1], GRASP_Z], seconds=1.5)
    log("at-grasp eef=%s" % np.round(api.eef(), 4).tolist())
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    log("closed %s" % g)
    # ---- grasp-verify retry (fires only when the close did not seat) ------
    for k in range(2):
        if g["effort"] >= HOLD_EFFORT:
            break
        log("GUARD regrasp %d (effort=%.2f w=%.4f)" % (k, g["effort"], g["width_m"]))
        api.grip(OPEN_W)
        api.move([cxy[0], cxy[1], APPROACH_Z], seconds=1.2)
        api.move([cxy[0], cxy[1], GRASP_Z - 0.006 * (k + 1)], seconds=1.2)
        api.grip(0.0)
        api.settle(0.4)
        g = api.gripper()
        log("regrasp%d -> %s" % (k, g))

    api.move([cxy[0], cxy[1], TRAVEL_Z], seconds=1.5)
    log("lift %s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))
    api.move([bxy[0], bxy[1], TRAVEL_Z], seconds=2.0)
    api.move([bxy[0], bxy[1], RELEASE_Z], seconds=1.5)
    log("over-bowl %s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))
    api.grip(OPEN_W)
    api.settle(0.6)
    log("released done=%s" % api.done)
    api.move([bxy[0], bxy[1], TRAVEL_Z], seconds=1.5)
    api.settle(0.5)
    log("end done=%s" % api.done)
    return "v2 pick=%s place=%s" % (np.round(cxy, 3).tolist(), np.round(bxy, 3).tolist())
