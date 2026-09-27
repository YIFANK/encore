"""c2 cell goal_put_wine_top_cabinet_stock -- v111 (v4, termination-flag-free)

v111 is a MECHANICAL pass over the frozen v4 removing every read of the
episode-termination flag (fair-v1.1.1: such a read is an indirect success
signal, because LIBERO terminates the episode when the success predicate
fires).  Two reads existed, both in run(); neither was load-bearing:
  1. the retry gate combined the flag with `held`; it is now `if not held:`.
     The retry is already gated by the gripper-effort sensor, and the flag
     term only suppressed the retry once the episode had ended.  Dropping it
     can only add retries to an already-failed grasp, and on the debug split
     the grasp closes at effort 3.0 on the first attempt, so it is unreached.
  2. the final api.log() line interpolated the flag -- a log string only.
No other change: every constant, threshold and motion is byte-identical to v4.

Receipt chain:
  v1  0/8  (51..65 odd): the home-pose arm hovers over the bottle, so
           footprint clustering of ALL above-table points fuses arm+bottle.
  v2  8/8  (51..65 odd): cluster a LOW z-slice (table+0.02..+0.13) only.
  v3  8/8  probe, 14/15 selection: + cylinder-centre correction, slice-span
           discriminator, re-perceive retry.  Sole failure = seed 54, where
           the bottle's footprint TOUCHED a short abutting prop, fusing them
           into a 0.129 x 0.136 m cluster that the slim filter rejected ->
           blind demo-anchor grasp closed on air.
  v4  this: keep only grid cells whose MAX height reaches table+0.095 before
           clustering.  The bottle column reaches ~table+0.13; the abutting
           prop tops out at table+0.05, so its cells are dropped and the two
           can no longer fuse.  Plus a graceful "tall cells near the anchor"
           fallback instead of a blind demo-anchor grasp.
"""

import numpy as np

PROVENANCE = {
    "DEMO_GRASP_ANCHOR": {
        "source": "pack.json demos[*] grasp keyframes (gripper_cmd -1->+1 at "
                  "t=32/35/30) EEF xy, mean of K=3", "allowed": True},
    "DEMO_GRASP_Z": {
        "source": "pack.json same keyframes, median EEF z "
                  "(1.0291/0.9917/1.0238); confirmed on debug seeds "
                  "(v2/v3: effort 3.0, width 0.0149 m, first attempt, 22/23 eps)",
        "allowed": True},
    "DEMO_RELEASE_ANCHOR": {
        "source": "pack.json demos[*] release keyframes (gripper_cmd +1->-1 at "
                  "t=85/122/80) EEF xy, mean of K=3", "allowed": True},
    "DEMO_RELEASE_Z": {
        "source": "pack.json same release keyframes, mean EEF z (sanity bound "
                  "/ z_cab fallback)", "allowed": True},
    "SLICE_LO/SLICE_HI": {
        "source": "debug-seed v1/v2 measurement: no arm point near the anchor "
                  "lies below z=1.05, table plane measured at 0.9012",
        "allowed": True},
    "TALL_CELL_H": {
        "source": "debug-seed measurement: bottle column reaches table+0.13, "
                  "the abutting prop that fused with it on seed 54 tops out "
                  "at table+0.05 (seed 51 cluster z[0.921,0.952])",
        "allowed": True},
    "MIN_SLICE_SPAN": {
        "source": "debug-seed v2/v3: bottle cluster spans the whole 0.11 m "
                  "slice, flat props span <=0.035 m", "allowed": True},
    "SEARCH_R_BOTTLE": {
        "source": "covers the K=3 demo grasp-anchor spread (0.07 m) plus the "
                  "observed per-seed bottle jitter (~0.02 m)", "allowed": True},
    "CELL": {"source": "generic footprint-clustering grid resolution",
             "allowed": True},
    "PLACE_DROP": {
        "source": "pack.json demo release EEF z is ~1.2 cm below "
                  "z_cab+(grasp_z-z_table) with the debug-measured z_cab "
                  "1.1272 / z_table 0.9012", "allowed": True},
    "TRAVERSE_Z": {"source": "pack.json ee_path maxima 1.2828-1.2966",
                   "allowed": True},
    "HOLD_EFFORT": {"source": "FairApi doc: effort 3.0 iff holding",
                    "allowed": True},
}

DEMO_GRASP_ANCHOR = (-0.2205, -0.0578)
DEMO_GRASP_Z = 1.0238
DEMO_RELEASE_ANCHOR = (-0.0158, -0.2127)
DEMO_RELEASE_Z = 1.2383

SLICE_LO, SLICE_HI = 0.020, 0.130
TALL_CELL_H = 0.095
MIN_SLICE_SPAN = 0.090
SEARCH_R_BOTTLE = 0.16
CELL = 0.015
PLACE_DROP = 0.010
TRAVERSE_Z = 1.280
HOLD_EFFORT = 2.5


def build_cloud(api, cam="cam_high", step=2):
    f = api.capture(cam)
    d = np.asarray(f.depth, dtype=float)
    if d.ndim == 3:
        d = d[..., 0]
    K = np.asarray(f.intrinsics, dtype=float)
    T = np.asarray(f.t_base_cam, dtype=float)
    H, W = d.shape
    vv, uu = np.mgrid[0:H:step, 0:W:step]
    z = d[::step, ::step]
    ok = np.isfinite(z) & (z > 0.05) & (z < 10.0)
    x = (uu[ok] - K[0, 2]) * z[ok] / K[0, 0]
    y = (vv[ok] - K[1, 2]) * z[ok] / K[1, 1]
    pts = np.stack([x, y, z[ok]], -1) @ T[:3, :3].T + T[:3, 3]
    return pts, T


def mode_z(zs, bin_m=0.004):
    zs = np.asarray(zs)
    if zs.size == 0:
        return None, 0
    h, e = np.histogram(zs, bins=np.arange(zs.min() - 1e-6,
                                           zs.max() + bin_m, bin_m))
    i = int(np.argmax(h))
    sel = zs[(zs >= e[i]) & (zs < e[i + 1] + 1e-9)]
    return float(np.median(sel)), int(h[i])


def tall_cell_components(pts, z_hi_gate, cell=CELL):
    """Occupied XY cells whose per-cell MAX height reaches z_hi_gate,
    8-connected.  Returns list of point-index arrays."""
    if pts.shape[0] == 0:
        return []
    ij = np.floor(pts[:, :2] / cell).astype(int)
    keys = {}
    for n in range(ij.shape[0]):
        keys.setdefault((ij[n, 0], ij[n, 1]), []).append(n)
    tall = {}
    for k, idxs in keys.items():
        if pts[idxs, 2].max() >= z_hi_gate:
            tall[k] = idxs
    seen, out = set(), []
    for k in tall:
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            c = stack.pop()
            comp.append(c)
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb = (c[0] + di, c[1] + dj)
                    if nb in tall and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        out.append(np.concatenate([np.asarray(tall[c], int) for c in comp]))
    return out


def cyl_centre(p, cam_xy):
    """Centre of a vertical cylinder seen from one side: only the
    camera-facing half deprojects, so the bbox midpoint is biased ~r/2
    toward the camera."""
    cx = 0.5 * (p[:, 0].min() + p[:, 0].max())
    cy = 0.5 * (p[:, 1].min() + p[:, 1].max())
    u = np.array([cx, cy]) - np.asarray(cam_xy, float)
    u = u / (np.linalg.norm(u) + 1e-9)
    w = np.array([-u[1], u[0]])
    pu, pw = p[:, :2] @ u, p[:, :2] @ w
    rad = float(np.clip(0.5 * (pw.max() - pw.min()), 0.010, 0.035))
    cen = u * (float(pu.min()) + rad) + w * (0.5 * float(pw.min() + pw.max()))
    return float(cen[0]), float(cen[1]), rad, float(cx), float(cy)


def find_bottle(api, pts, cam_xy, z_table, tag=""):
    ax, ay = DEMO_GRASP_ANCHOR
    dg = np.hypot(pts[:, 0] - ax, pts[:, 1] - ay)
    lo, hi = z_table + SLICE_LO, z_table + SLICE_HI
    gate = z_table + TALL_CELL_H
    sub = pts[(dg < SEARCH_R_BOTTLE) & (pts[:, 2] > lo) & (pts[:, 2] < hi)]
    api.log("%sslice[%.3f,%.3f]: %d pts, tall-cell gate %.3f"
            % (tag, lo, hi, sub.shape[0], gate))
    cands = []
    for idx in tall_cell_components(sub, gate):
        p = sub[idx]
        s = dict(n=int(p.shape[0]), p=p,
                 cx=float(0.5 * (p[:, 0].min() + p[:, 0].max())),
                 cy=float(0.5 * (p[:, 1].min() + p[:, 1].max())),
                 ex=float(p[:, 0].max() - p[:, 0].min()),
                 ey=float(p[:, 1].max() - p[:, 1].min()),
                 span=float(p[:, 2].max() - p[:, 2].min()))
        s["d"] = float(np.hypot(s["cx"] - ax, s["cy"] - ay))
        cands.append(s)
        api.log("%s  cl n=%3d c=(%.3f,%.3f) ext=(%.3f,%.3f) span=%.3f d=%.3f"
                % (tag, s["n"], s["cx"], s["cy"], s["ex"], s["ey"], s["span"],
                   s["d"]))
    good = [s for s in cands if s["ex"] < 0.09 and s["ey"] < 0.09
            and s["n"] >= 8 and s["span"] >= MIN_SLICE_SPAN]
    slim = [s for s in cands if s["ex"] < 0.09 and s["ey"] < 0.09 and s["n"] >= 8]
    pool = good or slim
    if pool:
        b = min(pool, key=lambda s: s["d"])
        cx, cy, rad, bx0, by0 = cyl_centre(b["p"], cam_xy)
        api.log("%sBOTTLE bboxmid=(%.4f,%.4f) r=%.4f -> centre=(%.4f,%.4f) "
                "(%d good/%d slim/%d)" % (tag, bx0, by0, rad, cx, cy,
                                          len(good), len(slim), len(cands)))
        return cx, cy
    # graceful fallback: tall points near the anchor, whatever they belong to
    near = sub[(np.hypot(sub[:, 0] - ax, sub[:, 1] - ay) < 0.075)]
    if near.shape[0] >= 15:
        cx, cy, rad, bx0, by0 = cyl_centre(near, cam_xy)
        api.log("%sFALLBACK tall-points-near-anchor n=%d bboxmid=(%.4f,%.4f) "
                "r=%.4f -> centre=(%.4f,%.4f)"
                % (tag, near.shape[0], bx0, by0, rad, cx, cy))
        return cx, cy
    api.log("%sNO BOTTLE CANDIDATE" % tag)
    return None


def run(api):
    api.log("=== v111 :: %s" % api.instruction())
    api.log("eef0=%s grip=%s" % (np.round(api.eef(), 4), api.gripper()))
    pts, T = build_cloud(api)
    cam_xy = T[:3, 3][:2]
    ax, ay = DEMO_GRASP_ANCHOR
    rx, ry = DEMO_RELEASE_ANCHOR

    dg = np.hypot(pts[:, 0] - ax, pts[:, 1] - ay)
    z_table, nt = mode_z(pts[dg < 0.30][:, 2])
    api.log("cam=%s z_table=%.4f (n=%d)" % (np.round(cam_xy, 3), z_table, nt))

    dr = np.hypot(pts[:, 0] - rx, pts[:, 1] - ry)
    cabreg = pts[(dr < 0.14) & (pts[:, 2] > z_table + 0.05)]
    z_cab = None
    if cabreg.shape[0] > 40:
        hc, ec = np.histogram(cabreg[:, 2],
                              bins=np.arange(cabreg[:, 2].min(),
                                             cabreg[:, 2].max() + 0.01, 0.01))
        thr = max(10, int(0.10 * cabreg.shape[0]))
        cand = [i for i in range(len(hc)) if hc[i] >= thr]
        if cand:
            sel = cabreg[(cabreg[:, 2] >= ec[cand[-1]])
                         & (cabreg[:, 2] < ec[cand[-1] + 1])]
            z_cab = float(np.median(sel[:, 2]))
            api.log("z_cab=%.4f n=%d x[%.3f,%.3f] y[%.3f,%.3f]"
                    % (z_cab, sel.shape[0], sel[:, 0].min(), sel[:, 0].max(),
                       sel[:, 1].min(), sel[:, 1].max()))
    if z_cab is None:
        z_cab = DEMO_RELEASE_Z - (DEMO_GRASP_Z - z_table)
        api.log("z_cab FALLBACK (demo geometry) %.4f" % z_cab)

    tgt = find_bottle(api, pts, cam_xy, z_table)
    bx, by = tgt if tgt is not None else (ax, ay)
    if tgt is None:
        api.log("fallback to demo anchor")

    # ---------------------------------------------------------------- grasp
    api.grip(0.08)
    api.move([bx, by, DEMO_GRASP_Z + 0.13], seconds=1.6)
    held, gz = False, DEMO_GRASP_Z
    for att, dz in enumerate((0.0, -0.020, +0.020)):
        gz = DEMO_GRASP_Z + dz
        api.move([bx, by, gz], seconds=1.3)
        api.grip(0.0)
        api.settle(0.2)
        g = api.gripper()
        api.log("grasp att%d xy=(%.4f,%.4f) z=%.4f -> %s" % (att, bx, by, gz, g))
        if float(g.get("effort", 0.0)) >= HOLD_EFFORT:
            held = True
            break
        api.grip(0.08)
    if not held:
        api.log("grasp failed -> retreat + re-perceive")
        api.move([bx, by, DEMO_GRASP_Z + 0.16], seconds=1.4)
        p2, T2 = build_cloud(api)
        t2 = find_bottle(api, p2, T2[:3, 3][:2], z_table, tag="re:")
        if t2 is not None:
            bx, by = t2
        for att, dz in enumerate((0.0, -0.020)):
            gz = DEMO_GRASP_Z + dz
            api.move([bx, by, gz], seconds=1.3)
            api.grip(0.0)
            api.settle(0.2)
            g = api.gripper()
            api.log("retry att%d xy=(%.4f,%.4f) z=%.4f -> %s"
                    % (att, bx, by, gz, g))
            if float(g.get("effort", 0.0)) >= HOLD_EFFORT:
                held = True
                break
            api.grip(0.08)
    api.log("held=%s gz=%.4f" % (held, gz))

    # ------------------------------------------------------------ transport
    place_z = z_cab + (gz - z_table) - PLACE_DROP
    trav = min(TRAVERSE_Z, max(place_z + 0.04, 1.26))
    api.log("place_z=%.4f trav=%.4f (z_cab %.4f, reach %.4f, demo %.4f)"
            % (place_z, trav, z_cab, gz - z_table, DEMO_RELEASE_Z))
    api.move([bx, by, trav], seconds=1.4)
    api.log("lift: eef=%s grip=%s" % (np.round(api.eef(), 4), api.gripper()))
    api.move([rx, ry, trav], seconds=1.8)
    api.log("traverse: eef=%s grip=%s" % (np.round(api.eef(), 4), api.gripper()))
    api.move([rx, ry, place_z], seconds=1.4)
    api.log("descend: eef=%s grip=%s" % (np.round(api.eef(), 4), api.gripper()))
    api.grip(0.08)
    api.settle(0.4)
    api.log("released eef=%s grip=%s" % (np.round(api.eef(), 4), api.gripper()))
    api.move([rx, ry, trav], seconds=1.2)
    api.settle(0.3)

    try:
        p3, _ = build_cloud(api)
        d3 = np.hypot(p3[:, 0] - rx, p3[:, 1] - ry)
        up = p3[(d3 < 0.18) & (p3[:, 2] > z_cab + 0.02)
                & (p3[:, 2] < z_cab + 0.30)]
        api.log("VERIFY %d pts above cabinet top" % up.shape[0])
        if up.shape[0] > 5:
            api.log("VERIFY bbox x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f"
                    % (up[:, 0].min(), up[:, 0].max(), up[:, 1].min(),
                       up[:, 1].max(), up[:, 2].max()))
    except Exception as e:
        api.log("verify failed %r" % (e,))
    api.log("=== run() complete")
