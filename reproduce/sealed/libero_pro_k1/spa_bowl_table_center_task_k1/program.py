"""c2k1clean / spa_bowl_table_center_task_k1 -- v1.

Intent: "Pick the akita black bowl next to the plate and place it on the plate"

Scene model (all of it derived from the two packs + debug seeds 51-58):
  * Two identical wide bowls (footprint 0.111 m) sit on a table at z=0.902.
    One of them is pinned at x=-0.088, y~0.000 in every debug seed -- that is
    the "table center" bowl the k1 pack's language names.  The other one is
    the bowl this intent names ("next to the plate"); it moves seed to seed.
  * A bright flat disc (footprint 0.136 m, top 0.919) is the plate.
  * A smaller round object (footprint 0.087) and a dark flat box are distractors.

Motion model (read off both packs, which agree):
  * Both demos grasp a bowl with the tool offset 0.041 m from the bowl centre
    along the world y axis -- a rim straddle, since the bowl (0.111) is wider
    than the open jaws (0.079).  The offset points toward the plate.
  * k1 closes while passing z=0.9194 (table+0.017); mate at 0.9226 (+0.021).
  * Both release with the same +-0.041 y offset over the plate centre, at
    z=0.9337 / 0.9369 (table+0.032 / +0.035), i.e. the plate top plus the
    0.017 the tool sat above the table when it took the bowl off it.
"""
import numpy as np

PROVENANCE = {
    "ZTAB_FALLBACK": {
        "source": "debug seeds 51-58: depth histogram mode of the cam_high "
                  "cloud = 0.902 m in every seed",
        "allowed": True},
    "BOWL_BAND": {
        "source": "debug seeds 51-58: bowl clusters have top z=0.951 "
                  "(table+0.049); plate top 0.919 (table+0.017)",
        "allowed": True},
    "BOWL_FOOTPRINT": {
        "source": "debug seeds 51-58: the two akita bowls measure "
                  "0.111x0.111 m; the small round distractor 0.087",
        "allowed": True},
    "PLATE_FOOTPRINT": {
        "source": "debug seeds 51/54/56: unfused plate cluster ext "
                  "0.136x0.137, mean rgb (160,153,150)",
        "allowed": True},
    "TABLE_CENTRE_SITE": {
        "source": "debug seeds 51-58: one bowl is pinned at x=-0.088, "
                  "y in [-0.015,0.012]; k1 pack grasps it at x=-0.0908",
        "allowed": True},
    "RIM_OFFSET_Y": {
        "source": "k1 pack: grasp ee y=0.0409 with that bowl centred at "
                  "y~0.000; release ee y=0.2268 with the plate at y~0.186. "
                  "mate pack: release ee y=0.1529, plate y~0.194 -> same "
                  "0.041 magnitude, opposite sign",
        "allowed": True},
    "Z_GRASP_OVER_TABLE": {
        "source": "k1 pack ee_path6 gripper closes at z=0.9194 = table+0.017; "
                  "mate pack keyframe t=53 closes at z=0.9226 = table+0.021",
        "allowed": True},
    "Z_RELEASE_OVER_PLATE": {
        "source": "k1 pack release z=0.9337, mate pack 0.9369; plate top "
                  "measured 0.919 on debug seeds -> plate_top + ~0.016",
        "allowed": True},
    "CARRY_Z": {
        "source": "k1 pack ee_path transport apex z=1.069; mate pack 1.1795",
        "allowed": True},
    "JAW_AXIS": {
        "source": "both packs' grasp/release offsets are purely along world "
                  "y, so the straight-down tool closes along y",
        "allowed": True},
    "GRIP_CLOSED_WIDTH": {
        "source": "k1 pack gripper_state at release 0.0081/-0.0076; open "
                  "state 0.0362/-0.0362 -> open span ~0.079",
        "allowed": True},
}

ZTAB_FALLBACK = 0.902
RIM_OFFSET_Y = 0.041
Z_GRASP_OVER_TABLE = 0.018
Z_RELEASE_OVER_PLATE = 0.016
CARRY_Z = 1.03
TC_SITE = np.array([-0.088, 0.000])


# ---------------------------------------------------------------- perception
def _cloud(api, cam="cam_high"):
    f = api.capture(cam)
    d = f.depth.astype(float)
    H, W = d.shape[:2]
    K, T = f.intrinsics, f.t_base_cam
    vv, uu = np.mgrid[0:H, 0:W]
    ok = np.isfinite(d) & (d > 0)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x = (uu - cx) * d / fx
    y = (vv - cy) * d / fy
    P = np.stack([x, y, d, np.ones_like(d)], axis=-1) @ T.T
    return P[..., :3][ok], f.rgb.astype(float)[ok]


def _clusters(pts, cols, cell=0.015, min_pts=40):
    if len(pts) == 0:
        return []
    key = np.floor(pts[:, :2] / cell).astype(int)
    occ = {}
    for i, k in enumerate(map(tuple, key)):
        occ.setdefault(k, []).append(i)
    seen, out = set(), []
    for k in occ:
        if k in seen:
            continue
        stack, mem = [k], []
        seen.add(k)
        while stack:
            c = stack.pop()
            mem.extend(occ[c])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in occ and n not in seen:
                        seen.add(n)
                        stack.append(n)
        if len(mem) < min_pts:
            continue
        m = np.array(mem)
        p, c2 = pts[m], cols[m]
        out.append({"n": len(m),
                    "xy": p[:, :2].mean(0),
                    "mid": 0.5 * (p[:, :2].max(0) + p[:, :2].min(0)),
                    "ztop": float(np.percentile(p[:, 2], 97)),
                    "ext": p[:, :2].max(0) - p[:, :2].min(0),
                    "rgb": c2.mean(0)})
    return out


def perceive(api):
    pts, cols = _cloud(api)
    w = ((pts[:, 0] > -0.30) & (pts[:, 0] < 0.42) &
         (pts[:, 1] > -0.12) & (pts[:, 1] < 0.52) & (pts[:, 2] > 0.85))
    pts, cols = pts[w], cols[w]
    hist, edges = np.histogram(pts[:, 2], bins=200)
    zt = float(edges[int(np.argmax(hist))] + 0.5 * (edges[1] - edges[0]))
    if not (0.87 < zt < 0.94):
        zt = ZTAB_FALLBACK

    # --- bowls: tall band, wide footprint
    a = (pts[:, 2] > zt + 0.028) & (pts[:, 2] < zt + 0.12)
    ca = _clusters(pts[a], cols[a], min_pts=300)
    bowls = [c for c in ca
             if max(c["ext"]) > 0.095 and c["n"] > 700 and c["xy"][0] > -0.18]
    bowls.sort(key=lambda c: -c["n"])

    # --- plate: flat bright band, with every tall object's disc removed
    b = (pts[:, 2] > zt + 0.006) & (pts[:, 2] < zt + 0.026)
    fp, fc = pts[b], cols[b]
    if len(fp):
        keep = np.ones(len(fp), bool)
        for c in ca:
            keep &= (np.hypot(fp[:, 0] - c["xy"][0],
                              fp[:, 1] - c["xy"][1]) > 0.066)
        keep &= fc.mean(1) > 135.0
        fp, fc = fp[keep], fc[keep]
    cb = _clusters(fp, fc, min_pts=400) if len(fp) else []
    plates = [c for c in cb if 0.10 < max(c["ext"]) < 0.19]
    plates.sort(key=lambda c: -c["n"])
    return zt, bowls, plates, ca


# ---------------------------------------------------------------------- run
def run(api):
    api.log("INSTR=%s" % api.instruction()[:90])
    zt, bowls, plates, allc = perceive(api)
    api.log("ztab=%.3f nbowl=%d nplate=%d" % (zt, len(bowls), len(plates)))
    for i, c in enumerate(bowls[:4]):
        api.log("bowl%d xy=%.3f,%.3f n=%d ext=%.3f,%.3f top=%.3f"
                % (i, c["mid"][0], c["mid"][1], c["n"],
                   c["ext"][0], c["ext"][1], c["ztop"]))
    for i, c in enumerate(plates[:3]):
        api.log("plate%d xy=%.3f,%.3f n=%d ext=%.3f,%.3f top=%.3f rgb=%d"
                % (i, c["mid"][0], c["mid"][1], c["n"], c["ext"][0],
                   c["ext"][1], c["ztop"], c["rgb"].mean()))

    if not bowls or not plates:
        api.log("ABORT no detection")
        return
    plate = plates[0]
    px, py = plate["mid"]
    ptop = plate["ztop"]

    # target bowl = the one that is NOT parked on the table-centre site
    if len(bowls) >= 2:
        d = [float(np.hypot(*(c["mid"] - TC_SITE))) for c in bowls]
        tgt = bowls[int(np.argmax(d))]
        api.log("dist_to_tc=%s pick=%d" % (["%.3f" % v for v in d],
                                           int(np.argmax(d))))
    else:
        tgt = bowls[0]
        api.log("only one bowl seen")
    bx, by = tgt["mid"]

    s = 1.0 if py > by else -1.0          # rim side faces the plate
    gx, gy = float(bx), float(by + s * RIM_OFFSET_Y)
    zg = zt + Z_GRASP_OVER_TABLE
    zr = ptop + Z_RELEASE_OVER_PLATE
    rx, ry = float(px), float(py + s * RIM_OFFSET_Y)
    api.log("grasp=%.3f,%.3f,%.3f side=%+.0f release=%.3f,%.3f,%.3f"
            % (gx, gy, zg, s, rx, ry, zr))

    api.grip(0.08)
    r = api.move([gx, gy, CARRY_Z], seconds=2.5)
    api.log("r_hover=%.4f eef=%s" % (r, np.round(api.eef(), 3).tolist()))
    r = api.move([gx, gy, zg], seconds=2.0)
    api.log("r_down=%.4f eef=%s" % (r, np.round(api.eef(), 3).tolist()))
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("grip w=%.4f eff=%.2f" % (g["width_m"], g["effort"]))

    r = api.move([gx, gy, CARRY_Z], seconds=2.0)
    g = api.gripper()
    api.log("r_lift=%.4f w=%.4f eff=%.2f eef=%s"
            % (r, g["width_m"], g["effort"], np.round(api.eef(), 3).tolist()))

    r = api.move([rx, ry, CARRY_Z], seconds=2.5)
    api.log("r_over=%.4f eef=%s" % (r, np.round(api.eef(), 3).tolist()))
    r = api.move([rx, ry, zr], seconds=2.0)
    api.log("r_place=%.4f eef=%s" % (r, np.round(api.eef(), 3).tolist()))
    api.grip(0.08)
    api.settle(0.5)
    api.move([rx, ry, CARRY_Z], seconds=2.0)
    api.settle(0.5)

    zt2, bowls2, plates2, _ = perceive(api)
    for i, c in enumerate(bowls2[:3]):
        api.log("post bowl%d xy=%.3f,%.3f top=%.3f"
                % (i, c["mid"][0], c["mid"][1], c["ztop"]))
    api.log("END")
