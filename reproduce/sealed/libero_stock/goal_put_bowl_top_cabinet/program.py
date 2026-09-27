"""c2 cell goal_put_bowl_top_cabinet_stock -- v111 (v5 + fair-v1.1.1 excision).

Identical control flow to v5. The only edit is mechanical: the six reads of
the episode-termination flag -- all of which were log-line fields, none a
branch, gate or exit -- are deleted. Each affected log line keeps its legal
sensors (move residual, eef, gripper state). No constant changed.

Rim-pinch the bowl and set it down on the cabinet's top face.

Perception (cam_high depth -> base-frame cloud):
  * table height = dominant z mode
  * bowl  = footprint cluster in the band [table+0.030, table+0.085] nearest the
            demo pick anchor; centre = bbox midpoint, rim radius = extent/2
  * cabinet top = z-mode plateau in [table+0.15, table+0.35]; its NEAR (+x) and
            +y edges are fully visible, the -y side is clipped by the image
            border, so the drop point is set by inset from the two real edges.
A per-cell max-height gate (a 2 cm cell holding anything taller than the bowl
rim is not bowl) stops the wine bottle bridging into the bowl's footprint.
Grasp: gripper centre at bowl_centre + rim_r along +y (jaw axis = tool y).
Hold is verified by gripper WIDTH, never by effort (effort is an instantaneous
contact read that drops to 0.05 mid-motion while the bowl is still pinched);
the recovery re-grasp reuses the original detection and never re-perceives
while something is in the hand.
Release: eef xy = target bowl centre + the same rim offset (the bowl hangs at
eef - offset), eef z = cabinet_top + hold_offset + clearance.
"""
import numpy as np

PROVENANCE = {
    "ANCHOR_PICK": {
        "source": "pack.json demos[*] closing keyframe ee[:2] (t=31/33/32): "
                  "(-0.0990,0.0504),(-0.1047,0.0338),(-0.0848,0.0179) -> "
                  "mean (-0.0962,0.0340)", "allowed": True},
    "GRASP_DZ_TABLE": {
        "source": "pack.json demos[*] closing keyframe ee[2] median 0.9160 "
                  "minus table z measured on debug seeds 51/53/57 (0.9010) "
                  "= 0.0150", "allowed": True},
    "RIM_SIGN_PLUS_Y": {
        "source": "debug seeds 51/53/57: measured bowl centres y = -0.005/"
                  "+0.013/+0.008 vs the three demo grasp anchors y = +0.050/"
                  "+0.034/+0.018 -> the demo pinch point sits on the +y side "
                  "of the bowl centre", "allowed": True},
    "RIM_R_FROM_EXTENT": {
        "source": "per-episode measurement: half the bowl cluster's xy extent "
                  "(0.105 x 0.101 m on debug seeds 51/53/57; radius profile "
                  "0.045 m at z=0.923 -> 0.056 m at the rim 0.952)",
        "allowed": True},
    "BAND_LO / BAND_HI": {
        "source": "debug seeds 51/53/57: bowl rim top 0.9521 = table+0.051; "
                  "band table+0.030..+0.085 isolates the bowl wall from the "
                  "plate (table+0.007) and the blue box (table+0.019)",
        "allowed": True},
    "CAB_BAND": {
        "source": "debug seeds 51/53/57: cabinet top plateau z = 1.1272 "
                  "= table+0.226; search band table+0.15..+0.35",
        "allowed": True},
    "PLACE_INSET_X": {
        "source": "debug seeds 51/53/57: top-face near edge x_max = 0.159/"
                  "0.155/0.164, far edge ~-0.09 -> 0.125 inset puts the bowl "
                  "mid-face", "allowed": True},
    "PLACE_INSET_Y": {
        "source": "debug seeds 51/53/57: top-face +y edge = -0.157/-0.160/"
                  "-0.169; inset = bowl radius 0.053 + 0.017 margin",
        "allowed": True},
    "RELEASE_CLEARANCE": {
        "source": "pack.json release eef z 1.157 minus measured cabinet top "
                  "1.1272 minus hold offset 0.015 = 0.015; 0.010 used",
        "allowed": True},
    "CELL_HEIGHT_GATE": {
        "source": "debug seeds 51/53/57: bowl rim top = table+0.051, wine "
                  "bottle top = table+0.085; a 0.070 gate drops the bottle's "
                  "cells (they bridged into the bowl on debug seed 65)",
        "allowed": True},
    "HOLD_WIDTH_MIN": {
        "source": "debug-seed measurement: gripper width while the bowl rim "
                  "is pinched = 0.0039..0.0122 m (seeds 51..65); width with "
                  "the jaws closed on air = 0.0010..0.0014 m -> 0.0025 m "
                  "separates them", "allowed": True},
    "SENSOR_ONLY_CONTROL": {
        "source": "no constant; branches use gripper width and perceived "
                  "geometry only. The episode-termination flag is not read "
                  "at all (fair-v1.1.1 excision); it was never branched on.",
        "allowed": True},
    "CARRY_Z": {
        "source": "pack.json demos[*].ee_path6 max z while gripper closed "
                  "(1.2219/1.2638/1.1983) -> 1.24", "allowed": True},
    "GRIP_CLOSE_M / GRIP_OPEN_M": {
        "source": "FairApi doc: <0.025 closes, else opens", "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi doc: gripper effort 3.0 iff holding -- REFUTED on "
                  "debug seeds: effort reads 0.05 mid-motion while the bowl is "
                  "still pinched, so it is logged only, never branched on",
        "allowed": True},
}

CAM = "cam_high"
ANCHOR_PICK = np.array([-0.0962, 0.0340])
GRASP_DZ_TABLE = 0.0150
BAND_LO, BAND_HI = 0.030, 0.085
CELL_HEIGHT_GATE = 0.070
HOLD_WIDTH_MIN = 0.0025
CAB_LO, CAB_HI = 0.15, 0.35
PLACE_INSET_X = 0.125
PLACE_INSET_Y = 0.070
RELEASE_CLEARANCE = 0.010
CARRY_Z = 1.24
GRIP_CLOSE_M = 0.0
GRIP_OPEN_M = 0.08
HOLD_EFFORT = 2.5


def base_cloud(f):
    d = np.asarray(f.depth, float)
    if d.ndim == 3:
        d = d[..., 0]
    h, w = d.shape
    K = np.asarray(f.intrinsics, float)
    vv, uu = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 1e-4)
    z = np.where(ok, d, 1.0)
    x = (uu - K[0, 2]) * z / K[0, 0]
    y = (vv - K[1, 2]) * z / K[1, 1]
    B = np.stack([x, y, z, np.ones_like(z)], axis=-1) @ np.asarray(
        f.t_base_cam, float).T
    return B[..., :3].reshape(-1, 3), ok.reshape(-1)


def cluster(pts, cell=0.02, min_cell=4, veto=None):
    """Footprint clustering; cells flagged by `veto` are excluded entirely."""
    ix = np.floor(pts[:, 0] / cell).astype(np.int64)
    iy = np.floor(pts[:, 1] / cell).astype(np.int64)
    uk, inv = np.unique(ix * 1000003 + iy, return_inverse=True)
    cix = np.zeros(uk.size, np.int64)
    ciy = np.zeros(uk.size, np.int64)
    cix[inv] = ix
    ciy[inv] = iy
    cnt = np.bincount(inv, minlength=uk.size).astype(float)
    if veto is not None:
        bad = np.bincount(inv, weights=veto.astype(float), minlength=uk.size)
        cnt = np.where(bad > 0, 0.0, cnt)
    idx = {(int(a), int(b)): i for i, (a, b) in enumerate(zip(cix, ciy))}
    lab = -np.ones(uk.size, np.int64)
    n = 0
    for s in range(uk.size):
        if lab[s] >= 0 or cnt[s] < min_cell:
            continue
        lab[s] = n
        st = [s]
        while st:
            c = st.pop()
            a, b = int(cix[c]), int(ciy[c])
            for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                j = idx.get((a + da, b + db))
                if j is not None and lab[j] < 0 and cnt[j] >= min_cell:
                    lab[j] = n
                    st.append(j)
        n += 1
    return lab[inv], n


def mode_z(z, lo, hi, step=0.002, half=0.010):
    sub = z[(z > lo) & (z < hi)]
    if sub.size < 50:
        return None
    hist, edges = np.histogram(sub, bins=np.arange(lo, hi, step))
    k = int(np.argmax(hist))
    m = (edges[k] + edges[k + 1]) / 2.0
    return float(np.median(sub[np.abs(sub - m) < half]))


def perceive(api, log):
    f = api.capture(CAM)
    P, ok = base_cloud(f)
    m = ok & (P[:, 0] > -0.45) & (P[:, 0] < 0.35) & (np.abs(P[:, 1]) < 0.45)
    P = P[m]
    z_table = mode_z(P[:, 2], 0.80, 1.45)
    log("z_table=%.4f n=%d" % (z_table, P.shape[0]))

    band_m = (P[:, 2] > z_table + BAND_LO) & (P[:, 2] < z_table + BAND_HI)
    band = P[band_m]
    cs = 0.02
    key = (np.floor(P[:, 0] / cs).astype(np.int64) * 1000003
           + np.floor(P[:, 1] / cs).astype(np.int64))
    tallset = np.unique(key[P[:, 2] > z_table + CELL_HEIGHT_GATE])
    veto = np.isin(key[band_m], tallset)
    log("cell gate: %d tall cells, %d/%d band pts vetoed"
        % (tallset.size, int(veto.sum()), veto.size))
    lab, nc = cluster(band, cs, 4, veto)
    best, cands = None, []
    for c in range(nc):
        s = lab == c
        if int(s.sum()) < 150:
            continue
        p = band[s]
        lo = np.percentile(p[:, :2], 3, axis=0)
        hi = np.percentile(p[:, :2], 97, axis=0)
        mid = (lo + hi) / 2.0
        ext = hi - lo
        d = float(np.linalg.norm(mid - ANCHOR_PICK))
        cands.append((d, mid, ext, int(s.sum()), float(np.percentile(p[:, 2], 99))))
        log("band comp n=%5d mid=(%+.4f,%+.4f) ext=(%.3f,%.3f) zmax=%.4f d=%.3f"
            % (s.sum(), mid[0], mid[1], ext[0], ext[1], p[:, 2].max(), d))
    ok_c = [c for c in cands if 0.06 <= max(c[2]) <= 0.17]
    if not ok_c:
        ok_c = cands
    if not ok_c:
        return None
    best = min(ok_c, key=lambda c: c[0])
    bowl_c = best[1]
    rim_r = float(np.mean(best[2])) / 2.0
    log("BOWL mid=(%+.4f,%+.4f) rim_r=%.4f rim_top=%.4f d_anchor=%.3f"
        % (bowl_c[0], bowl_c[1], rim_r, best[4], best[0]))

    z_top = mode_z(P[:, 2], z_table + CAB_LO, z_table + CAB_HI)
    face = P[np.abs(P[:, 2] - z_top) < 0.012]
    labf, nf = cluster(face, 0.02, 4)
    sizes = [int((labf == c).sum()) for c in range(nf)]
    big = int(np.argmax(sizes))
    pf = face[labf == big]
    x_max = float(np.percentile(pf[:, 0], 99))
    y_max = float(np.percentile(pf[:, 1], 99))
    log("CAB z_top=%.4f nface=%d x %.3f..%.3f y %.3f..%.3f (x99=%.4f y99=%.4f)"
        % (z_top, pf.shape[0], pf[:, 0].min(), pf[:, 0].max(),
           pf[:, 1].min(), pf[:, 1].max(), x_max, y_max))
    return dict(z_table=z_table, bowl=bowl_c, rim_r=rim_r, rim_top=best[4],
                z_top=z_top, x_max=x_max, y_max=y_max)


def run(api):
    def log(m):
        api.log(m)

    log("instruction=%r" % api.instruction())
    R0 = api.tool_rotation()
    log("tool_rot col1=%s eef0=%s grip0=%s"
        % (np.round(R0[:, 1], 3).tolist(), np.round(api.eef(), 4).tolist(),
           api.gripper()))

    S = perceive(api, log)
    if S is None:
        log("PERCEPTION FAILED")
        return "no bowl"

    jaw = np.array([0.0, 1.0])          # +y side of the bowl (RIM_SIGN_PLUS_Y)
    ry = R0[:, 1][:2]
    if np.linalg.norm(ry) > 1e-6:
        ry = ry / np.linalg.norm(ry)
        if ry[1] < 0:
            ry = -ry
        jaw = ry
    log("jaw(+y)=%s" % np.round(jaw, 3).tolist())

    off = S["rim_r"] * jaw
    gxy = S["bowl"] + off
    gz = S["z_table"] + GRASP_DZ_TABLE
    log("GRASP xy=(%+.4f,%+.4f) z=%.4f off=%s"
        % (gxy[0], gxy[1], gz, np.round(off, 4).tolist()))

    target_c = np.array([S["x_max"] - PLACE_INSET_X, S["y_max"] - PLACE_INSET_Y])
    rxy = target_c + off
    rz = S["z_top"] + GRASP_DZ_TABLE + RELEASE_CLEARANCE
    log("PLACE bowl_target=(%+.4f,%+.4f) eef=(%+.4f,%+.4f) z=%.4f"
        % (target_c[0], target_c[1], rxy[0], rxy[1], rz))

    api.grip(GRIP_OPEN_M)
    r = api.move([gxy[0], gxy[1], gz + 0.11], seconds=1.8)
    log("pregrasp res=%.4f eef=%s"
        % (r, np.round(api.eef(), 4).tolist()))
    r = api.move([gxy[0], gxy[1], gz], seconds=1.1)
    log("descend res=%.4f eef=%s"
        % (r, np.round(api.eef(), 4).tolist()))
    api.grip(GRIP_CLOSE_M)
    api.settle(0.3)
    g = api.gripper()
    log("closed grip=%s eef=%s" % (g, np.round(api.eef(), 4).tolist()))

    r = api.move([gxy[0], gxy[1], gz + 0.13], seconds=1.1)
    api.settle(0.2)
    g = api.gripper()
    log("lift res=%.4f eef=%s grip=%s"
        % (r, np.round(api.eef(), 4).tolist(), g))

    # HOLD TEST: width only. effort reads 0.05 mid-motion on a pinched bowl
    # (debug seeds 51/65), so branching on effort throws away good grasps.
    if g["width_m"] < HOLD_WIDTH_MIN:
        # Jaws are on air. Re-grasp the SAME detected rim point 5 mm lower
        # with a slightly tighter radius. Deliberately no re-perception: with
        # anything in the hand the bowl cluster is gone from the table and the
        # nearest-anchor rule would lock onto a different prop.
        log("RECOVER: width %.4f < %.4f -> regrasp same rim point"
            % (g["width_m"], HOLD_WIDTH_MIN))
        api.grip(GRIP_OPEN_M)
        gxy = S["bowl"] + 0.92 * S["rim_r"] * jaw
        api.move([gxy[0], gxy[1], gz + 0.06], seconds=1.0)
        api.move([gxy[0], gxy[1], gz - 0.005], seconds=0.9)
        api.grip(GRIP_CLOSE_M)
        api.settle(0.3)
        api.move([gxy[0], gxy[1], gz + 0.13], seconds=1.0)
        api.settle(0.2)
        log("recover lift eef=%s grip=%s"
            % (np.round(api.eef(), 4).tolist(), api.gripper()))

    r = api.move([rxy[0], rxy[1], CARRY_Z], seconds=2.0)
    log("carry res=%.4f eef=%s grip=%s"
        % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    r = api.move([rxy[0], rxy[1], rz], seconds=1.2)
    log("descend2 res=%.4f eef=%s grip=%s"
        % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(GRIP_OPEN_M)
    api.settle(0.4)
    log("released grip=%s eef=%s"
        % (api.gripper(), np.round(api.eef(), 4).tolist()))
    api.move([rxy[0], rxy[1], rz + 0.09], seconds=0.7)
    api.settle(0.3)

    try:
        f2 = api.capture(CAM)
        P2, ok2 = base_cloud(f2)
        P2 = P2[ok2]
        n = P2[(np.abs(P2[:, 0] - target_c[0]) < 0.09)
               & (np.abs(P2[:, 1] - target_c[1]) < 0.09)
               & (P2[:, 2] > S["z_top"] + 0.01)]
        log("post: pts above cab top near target=%d zmax=%.4f"
            % (n.shape[0],
               float(np.percentile(n[:, 2], 99)) if n.shape[0] else -1))
    except Exception as e:
        log("post fail %s" % e)
    return "done"
