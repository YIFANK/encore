"""c2 cell spa_bowl_next_to_ramekin_stock -- v1.1 (de-oracled v2)

v2 -> v1.1, MECHANICAL de-oracle only (no tuning, no new behaviour): under the
fair-v1.1 server `api.done` means "episode terminated", not "runtime success",
so every read of it has been removed.  Two sites, neither load-bearing:
  1. the `if api.done: return` early exit inside the effort-gated grasp-retry
     loop (the loop's real gate is already sensor-based: effort >= 2.5 and
     0.004 <= width <= 0.022 -- kept unchanged);
  2. `done=%s` in the final end-of-episode log line (cosmetic).
Nothing else differs from program_v2.py.


Intent: "pick up the black bowl next to the ramekin and place it on the plate"

v1 -> v2, ONE change: the target bowl is no longer localised by the bbox
midpoint of its depth component.  v1's components fused the target bowl with
its neighbour on 5/8 debug seeds (measured y-extent 0.210-0.240 m vs 0.120 m
when clean), which dragged the pinch point ~4 cm inboard and cost every one of
those episodes.  v2 localises the pinch point directly as the +y rim edge of
the above-table cells around the demo anchor -- a statistic the neighbour
cannot corrupt, because the neighbour always sits at LOWER y than the target.

Everything here is derived from packs/c2_spa_bowl_next_to_ramekin_stock/pack.json
plus debug-seed (51-65) measurements.  See PROVENANCE.
"""

import numpy as np

PROVENANCE = {
    "ANCHOR_GRASP_XY": {
        "source": "pack.json: mean of the 3 demos' closing keyframes "
                  "(demo0 t=55, demo1 t=54, demo2 t=50 -- the frames whose "
                  "gripper_cmd flips to +1) ee[:2] = "
                  "(-0.1575,0.3512),(-0.1456,0.3576),(-0.1678,0.3473)",
        "allowed": True},
    "ANCHOR_PLACE_XY": {
        "source": "pack.json: mean of the 3 demos' release keyframes "
                  "(demo0 t=131, demo1 t=117, demo2 t=127 -- gripper_cmd back "
                  "to -1 while the jaws are still narrow) ee[:2] = "
                  "(0.0895,0.2280),(0.0692,0.2315),(0.0750,0.2469)",
        "allowed": True},
    "ANCHOR_TO_CENTRE": {
        "source": "debug-seed measurement (v1 run, seeds 51/59/63 -- the three "
                  "episodes whose target component was clean): "
                  "ANCHOR_GRASP - component bbox midpoint = "
                  "(+0.023,+0.0295),(+0.023,+0.022),(+0.023,+0.037); mean "
                  "(+0.023,+0.030) is used only to seed the search window",
        "allowed": True},
    "GRASP_Z": {
        "source": "pack.json: mean ee[2] of the 3 closing keyframes "
                  "(0.9318,0.9169,0.9215)",
        "allowed": True},
    "PLACE_Z": {
        "source": "pack.json: mean ee[2] of the 3 release keyframes "
                  "(0.9450,0.9322,0.9322)",
        "allowed": True},
    "TRAVEL_Z": {
        "source": "pack.json: transport-leg ee_path z (demo0 1.071/1.092, "
                  "demo1 1.055/1.104, demo2 1.079), rounded down",
        "allowed": True},
    "PINCH_HYPOTHESIS": {
        "source": "pack.json: the demo gripper_state while loaded collapses to "
                  "a total width ~0.011-0.013 m against an open width ~0.078 m "
                  "-> the demo closes on a thin wall, i.e. a RIM pinch, not an "
                  "enveloping grasp (campaign LAWS.md C2-L4). Confirmed on "
                  "debug seeds 51/59/63: closing at the rim edge gave "
                  "width 0.007-0.011 with effort 3.00 and a successful lift.",
        "allowed": True},
    "RIM_EDGE_RULE": {
        "source": "debug-seed measurement (v1 run): on all three seeds that "
                  "lifted the bowl, the commanded pinch y coincided with the "
                  "largest occupied cell-centre y of the near-anchor blob "
                  "(0.3670/0.3820/0.3820); the neighbour object sits ~0.13 m "
                  "at LOWER y (v1 comp lists), so that maximum is uncorrupted "
                  "even when the two objects' footprints fuse",
        "allowed": True},
    "HOLD_OFFSET": {
        "source": "debug-seed measurement (v1 run, clean seeds): pinch y minus "
                  "component bbox-mid y = 0.052/0.0595/0.052 m",
        "allowed": True},
    "JAW_AXIS": {
        "source": "runtime api.tool_rotation() column 1 (tool y = jaw closing "
                  "axis); measured as world +/-y on every debug episode",
        "allowed": True},
    "TABLE_Z": {
        "source": "debug-seed measurement: modal depth-cloud height in the "
                  "workspace ROI of api.capture('cam_high') (0.9013 on 8/8)",
        "allowed": True},
    "GRID_M / HEIGHT_TOL / ABOVE_TABLE": {
        "source": "generic perception mechanics: 1.5 cm ground-footprint cells "
                  "with per-cell max height (campaign LAWS.md C2-L2/C2-L5)",
        "allowed": True},
    "HOLD_TEST": {
        "source": "debug-seed measurement: a genuine rim pinch reads "
                  "effort 3.00 with width 0.007-0.011 m after the lift; jaws "
                  "closed on air read width <0.005 and effort 0.05",
        "allowed": True},
}

ANCHOR_GRASP_XY = np.array([-0.1570, 0.3520])
ANCHOR_PLACE_XY = np.array([0.0779, 0.2355])
ANCHOR_TO_CENTRE = np.array([0.023, 0.030])
GRASP_Z = 0.9234
PLACE_Z = 0.9365
TRAVEL_Z = 1.030
HOLD_OFFSET = 0.0545          # pinch -> bowl-centre distance along the jaw

GRID_M = 0.015
HEIGHT_TOL = 0.030
ABOVE_TABLE = 0.006
WIN = 0.090                   # half-width of the anchor-local search window
BAND = 0.030                  # depth of the rim band used for the x centre
ROI = dict(xmin=-0.40, xmax=0.40, ymin=-0.48, ymax=0.48)


# ---------------------------------------------------------------- perception
def cloud(frame):
    d = np.asarray(frame.depth, float)
    if d.ndim == 3:
        d = d[..., 0]
    h, w = d.shape
    K = np.asarray(frame.intrinsics, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 0)
    z = d[ok]
    pc = np.stack([(uu[ok] - cx) * z / fx, (vv[ok] - cy) * z / fy, z], 1)
    T = np.asarray(frame.t_base_cam, float)
    return pc @ T[:3, :3].T + T[:3, 3]


def table_height(pb):
    m = ((pb[:, 0] > ROI["xmin"]) & (pb[:, 0] < ROI["xmax"]) &
         (pb[:, 1] > ROI["ymin"]) & (pb[:, 1] < ROI["ymax"]) &
         (pb[:, 2] > 0.70) & (pb[:, 2] < 1.10))
    z = pb[m, 2]
    if z.size == 0:
        return 0.90
    hist, edges = np.histogram(z, bins=200)
    i = int(np.argmax(hist))
    return float(0.5 * (edges[i] + edges[i + 1]))


def cellmap(pb, tz, zmax):
    """(ci,cj) -> [max_h, npx] for every above-table footprint cell."""
    m = ((pb[:, 0] > ROI["xmin"]) & (pb[:, 0] < ROI["xmax"]) &
         (pb[:, 1] > ROI["ymin"]) & (pb[:, 1] < ROI["ymax"]) &
         (pb[:, 2] > tz + ABOVE_TABLE) & (pb[:, 2] < zmax))
    p = pb[m]
    cells = {}
    if p.shape[0] == 0:
        return cells
    ci = np.floor(p[:, 0] / GRID_M).astype(int)
    cj = np.floor(p[:, 1] / GRID_M).astype(int)
    for a, b, z in zip(ci, cj, p[:, 2]):
        k = (int(a), int(b))
        c = cells.get(k)
        if c is None:
            cells[k] = [float(z), 1]
        else:
            if z > c[0]:
                c[0] = float(z)
            c[1] += 1
    return cells


def components(cells):
    keys = list(cells)
    idx = {k: n for n, k in enumerate(keys)}
    parent = list(range(len(keys)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for k in keys:
        for da, db in ((1, 0), (0, 1), (1, 1), (1, -1)):
            n = (k[0] + da, k[1] + db)
            if n in idx and abs(cells[k][0] - cells[n][0]) < HEIGHT_TOL:
                ra, rb = find(idx[k]), find(idx[n])
                if ra != rb:
                    parent[rb] = ra
    groups = {}
    for k in keys:
        groups.setdefault(find(idx[k]), []).append(k)
    out = []
    for g in groups.values():
        xs = np.array([a for a, _ in g]) * GRID_M + 0.5 * GRID_M
        ys = np.array([b for _, b in g]) * GRID_M + 0.5 * GRID_M
        hz = np.array([cells[k][0] for k in g])
        out.append(dict(ncell=len(g), npx=int(sum(cells[k][1] for k in g)),
                        cx=float(0.5 * (xs.min() + xs.max())),
                        cy=float(0.5 * (ys.min() + ys.max())),
                        ex=float(xs.max() - xs.min() + GRID_M),
                        ey=float(ys.max() - ys.min() + GRID_M),
                        top=float(hz.max())))
    out.sort(key=lambda c: -c["ncell"])
    return out


def rim_edge(cells, tz, seed, log):
    """Pinch point = the +y rim edge of the above-table cells near `seed`.

    The distractor this scene fuses the target with always sits at LOWER y,
    so the maximum-y occupied cell of the local window belongs to the target
    even when the two footprints touch.
    """
    sel = []
    for (a, b), (h, npx) in cells.items():
        x = a * GRID_M + 0.5 * GRID_M
        y = b * GRID_M + 0.5 * GRID_M
        if abs(x - seed[0]) > WIN or abs(y - seed[1]) > WIN:
            continue
        if npx < 4:
            continue
        if not (tz + 0.020 < h < tz + 0.120):
            continue
        nb = sum(1 for d in ((1, 0), (-1, 0), (0, 1), (0, -1),
                             (1, 1), (1, -1), (-1, 1), (-1, -1))
                 if (a + d[0], b + d[1]) in cells)
        if nb < 3:
            continue
        sel.append((x, y))
    if not sel:
        return None
    sel = np.array(sel, float)
    y_edge = float(sel[:, 1].max())
    band = sel[sel[:, 1] >= y_edge - BAND]
    x_c = float(0.5 * (band[:, 0].min() + band[:, 0].max()))
    body = sel[sel[:, 1] >= y_edge - 0.13]
    log("rim: n=%d y_edge=%.4f x_c=%.4f nband=%d xspan=%.3f yspan=%.3f"
        % (len(sel), y_edge, x_c, len(band),
           float(body[:, 0].max() - body[:, 0].min()),
           float(body[:, 1].max() - body[:, 1].min())))
    return np.array([x_c, y_edge])


def pick_near(comps, anchor, rmax, maxcell=200):
    best, bd = None, 1e9
    for c in comps:
        if c["ncell"] > maxcell:
            continue
        d = float(np.hypot(c["cx"] - anchor[0], c["cy"] - anchor[1]))
        if d < bd:
            best, bd = c, d
    if best is None or bd > rmax:
        return None, bd
    return best, bd


# -------------------------------------------------------------------- policy
def run(api):
    log = api.log
    log("instruction: %s" % api.instruction())

    R0 = np.asarray(api.tool_rotation(), float)
    jaw = R0[:2, 1].astype(float)
    n = float(np.hypot(jaw[0], jaw[1]))
    jaw = jaw / n if n > 1e-6 else np.array([0.0, 1.0])
    if jaw[1] < 0:
        jaw = -jaw
    log("jaw=%.3f,%.3f eef0=%s" % (jaw[0], jaw[1],
                                   np.round(api.eef(), 4).tolist()))

    f = api.capture("cam_high")
    pb = cloud(f)
    tz = table_height(pb)
    cells = cellmap(pb, tz, tz + 0.30)
    comps = components(cells)
    log("table_z=%.4f ncell=%d ncomp=%d" % (tz, len(cells), len(comps)))
    for c in comps[:8]:
        log("  comp n=%3d c=(%.3f,%.3f) ext=(%.3f,%.3f) top=+%.3f "
            "dG=%.3f dP=%.3f"
            % (c["ncell"], c["cx"], c["cy"], c["ex"], c["ey"], c["top"] - tz,
               np.hypot(c["cx"] - ANCHOR_GRASP_XY[0],
                        c["cy"] - ANCHOR_GRASP_XY[1]),
               np.hypot(c["cx"] - ANCHOR_PLACE_XY[0],
                        c["cy"] - ANCHOR_PLACE_XY[1])))

    seed = ANCHOR_GRASP_XY - ANCHOR_TO_CENTRE
    pinch = rim_edge(cells, tz, seed, log)
    if pinch is None:
        log("RIM NOT FOUND -> demo anchor")
        pinch = ANCHOR_GRASP_XY.copy()
    log("pinch=(%.4f,%.4f) seed=(%.4f,%.4f)"
        % (pinch[0], pinch[1], seed[0], seed[1]))

    plate, dp = pick_near(comps, ANCHOR_PLACE_XY, 0.10)
    if plate is None:
        log("PLATE NOT FOUND (d=%.3f) -> demo anchor" % dp)
        pc = ANCHOR_PLACE_XY.copy()
    else:
        pc = np.array([plate["cx"], plate["cy"]])
        log("plate=(%.4f,%.4f) ext=(%.3f,%.3f) top=+%.3f d=%.3f"
            % (pc[0], pc[1], plate["ex"], plate["ey"], plate["top"] - tz, dp))

    api.grip(0.08)
    held = None
    for k, inset in enumerate((0.0, 0.010)):
        g = pinch - inset * jaw
        log("try%d pinch=(%.4f,%.4f) inset=%.3f" % (k, g[0], g[1], inset))
        api.move([g[0], g[1], TRAVEL_Z], seconds=1.6)
        api.move([g[0], g[1], GRASP_Z], seconds=1.2)
        api.grip(0.0)
        api.settle(0.3)
        gr = api.gripper()
        log("  close: w=%.4f e=%.2f eef=%s"
            % (gr["width_m"], gr["effort"], np.round(api.eef(), 4).tolist()))
        api.move([g[0], g[1], TRAVEL_Z], seconds=1.2)
        gr = api.gripper()
        log("  lift : w=%.4f e=%.2f eef=%s"
            % (gr["width_m"], gr["effort"], np.round(api.eef(), 4).tolist()))
        if gr["effort"] >= 2.5 and 0.004 <= gr["width_m"] <= 0.022:
            held = g
            break
        api.grip(0.08)
        api.settle(0.2)
    if held is None:
        held = pinch
        log("no confirmed hold; continuing")

    tgt = pc + 0.5 * HOLD_OFFSET * jaw
    log("carry to (%.4f,%.4f)" % (tgt[0], tgt[1]))
    api.move([tgt[0], tgt[1], TRAVEL_Z], seconds=1.6)
    api.move([tgt[0], tgt[1], PLACE_Z + 0.015], seconds=1.0)
    gr = api.gripper()
    log("over plate: w=%.4f e=%.2f eef=%s"
        % (gr["width_m"], gr["effort"], np.round(api.eef(), 4).tolist()))
    api.grip(0.08)
    api.settle(0.4)
    api.move([tgt[0], tgt[1], TRAVEL_Z], seconds=1.0)
    api.settle(0.3)
    log("end eef=%s" % (np.round(api.eef(), 4).tolist(),))
    return "v1_1"
