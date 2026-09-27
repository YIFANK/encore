"""c2 cell obj_alphabet_soup_stock -- v1.1 (de-oracled v3).

Intent: "pick up the alphabet soup and place it in the basket".

v1 (4/4 on debug 51,53,55,57) picked the cluster nearest the pack's demonstrated
grasp XY.  v2 (8/8 on debug 51..65 odd) identified the target by an intrinsic
signature measured on the debug seeds -- top height, footprint, and the fact
that it is the only bluish object -- so a differently-arranged scene cannot
silently retarget it, and tied the release height to the measured basket rim.
v3 added an `api.done`-driven outer retry loop.

v1.1 is v3 with ALL `api.done` reads excised (under fair protocol v1.1
`api.done` means episode-terminated only and carries no success information):
the outer "repeat the whole pick-place while not done" loop is replaced by ONE
fixed nominal pick-and-place, and the two `api.done` early-exits inside the
grasp loop are removed.  The retry that survives is the pre-existing
sensor-gated one: up to 3 grasp attempts, each gated on the gripper's own
effort reading (`effort > 1.0` == holding), re-perceiving between attempts.
Nothing else changed -- same constants, same perception, same PROVENANCE.
"""
import numpy as np

PROVENANCE = {
    "PRIOR_OBJ_XY": {
        "source": "pack.json demos: gripper-close keyframes ee[:2] -- demo0 "
                  "(-0.1234,-0.2507), demo1 retry (-0.1101,-0.2443), demo2 "
                  "(-0.1195,-0.2416); mean (-0.118,-0.245). Confirmed by debug "
                  "seeds 51/53/55/57 cam_high: target cluster top at "
                  "(-0.118,-0.242).",
        "allowed": True},
    "PRIOR_BASKET_XY": {
        "source": "pack.json demos: release keyframes ee[:2] -- (-0.0291,0.2280), "
                  "(-0.0260,0.2678), (-0.0536,0.2752); mean (-0.036,0.257)",
        "allowed": True},
    "GRASP_Z": {
        "source": "pack.json demos: eef z at the close that held -- demo1 0.0450, "
                  "demo2 0.0470 (demo1's close at 0.0664 came up empty, "
                  "gripper_state 0.0012). Verified on debug 51-57: close at "
                  "commanded 0.047 gives effort 3.0 and a 0.0627 finger gap.",
        "allowed": True},
    "APPROACH_Z / LIFT_Z": {
        "source": "pack.json ee_path6: hover ~0.20 before the descent, transfer "
                  "at 0.21-0.30",
        "allowed": True},
    "RELEASE_CLEARANCE / RELEASE_Z_RANGE": {
        "source": "pack.json demos: release eef z 0.1220/0.1447/0.1615 sits just "
                  "above the basket rim; debug 51-57 measured the basket top at "
                  "z=0.144, and a release at 0.155 placed the object 4/4",
        "allowed": True},
    "TGT_H_RANGE / TGT_FOOT_RANGE": {
        "source": "debug seeds 51,53,55,57 cam_high depth: the demonstrated "
                  "grasp cluster measures top height 0.079 m above the table "
                  "and a 0.065x0.070 m footprint",
        "allowed": True},
    "TGT_BLUE_MIN": {
        "source": "debug seeds 51,53,55,57 cam_high RGB: the demonstrated grasp "
                  "cluster is the only scene object with mean B-R > 0 "
                  "(+12 to +14); the same-size neighbour is -11.7, the tall "
                  "bottle -1.2, the other object -36",
        "allowed": True},
    "HELD_GAP_RANGE": {
        "source": "pack.json demos: finger gap while carrying the object -- "
                  "demo0 0.0631, demo2 0.0640; debug 51-57 measured 0.0627",
        "allowed": True},
    "OPEN_W / CLOSE_W": {
        "source": "FairApi contract: api.grip(<0.025) closes, else opens",
        "allowed": True},
    "TABLE_MARGIN / CELL / WS_X / WS_Y / ARM_H_MAX": {
        "source": "generic RGB-D table-plane segmentation mechanics; the table "
                  "plane is measured per-episode from cam_high depth, and the "
                  "workspace box is the extent of the demonstrated motion in "
                  "pack.json ee_path6, padded",
        "allowed": True},
}

PRIOR_OBJ_XY = np.array([-0.118, -0.245])
PRIOR_BASKET_XY = np.array([-0.036, 0.257])
GRASP_Z = 0.047
APPROACH_Z = 0.20
LIFT_Z = 0.26
RELEASE_CLEARANCE = 0.013
RELEASE_Z_RANGE = (0.140, 0.185)
OPEN_W = 0.08
CLOSE_W = 0.0
TABLE_MARGIN = 0.014
CELL = 0.012
WS_X = (-0.32, 0.22)
WS_Y = (-0.45, 0.45)
ARM_H_MAX = 0.30
TGT_H_RANGE = (0.050, 0.120)
TGT_FOOT_RANGE = (0.040, 0.100)
TGT_BLUE_MIN = 4.0
HELD_GAP_RANGE = (0.030, 0.075)


# ---------------------------------------------------------------- perception
def _deproject_all(frame):
    rgb = np.asarray(frame.rgb)
    d = np.asarray(frame.depth, dtype=float)
    if d.ndim == 3:
        d = d[..., 0]
    H, W = d.shape[:2]
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vs, us = np.mgrid[0:H, 0:W]
    valid = np.isfinite(d) & (d > 0.05) & (d < 12.0)
    z = np.where(valid, d, 1.0)
    P = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z, np.ones_like(z)], axis=-1)
    B = P @ T.T
    return rgb, valid, B[..., 0], B[..., 1], B[..., 2]


def _clusters(api, tag="scene"):
    frame = api.capture("cam_high")
    rgb, valid, bx, by, bz = _deproject_all(frame)
    inbox = valid & (bx > WS_X[0]) & (bx < WS_X[1]) & (by > WS_Y[0]) & (by < WS_Y[1])
    if not inbox.any():
        api.log("[%s] no in-box depth" % tag)
        return None, []
    edges = np.arange(-0.60, 1.60, 0.005)
    hist, _ = np.histogram(bz[inbox], bins=edges)
    table_z = float(edges[int(np.argmax(hist))] + 0.0025)
    obj = inbox & (bz > table_z + TABLE_MARGIN) & (bz < table_z + 0.45)
    n_obj = int(obj.sum())
    api.log("[%s] table_z=%.4f inbox_px=%d obj_px=%d" % (tag, table_z, int(inbox.sum()), n_obj))
    if n_obj < 20:
        return table_z, []
    pts = np.stack([bx[obj], by[obj], bz[obj]], axis=-1)
    cols = rgb[obj].astype(float)
    ij = np.floor(pts[:, :2] / CELL).astype(int)
    cells = {}
    for n in range(ij.shape[0]):
        cells.setdefault((int(ij[n, 0]), int(ij[n, 1])), []).append(n)
    seen = set()
    out = []
    for key in list(cells):
        if key in seen:
            continue
        stack = [key]
        seen.add(key)
        members = []
        while stack:
            c = stack.pop()
            members.extend(cells[c])
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb = (c[0] + di, c[1] + dj)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        m = np.asarray(members, dtype=int)
        if m.size < 25:
            continue
        p = pts[m]
        c = cols[m]
        top = float(p[:, 2].max())
        cap = p[p[:, 2] > top - 0.015]
        mean_rgb = c.mean(axis=0)
        out.append({
            "n": int(m.size),
            "cx": float(p[:, 0].mean()), "cy": float(p[:, 1].mean()),
            "top": top, "h": top - table_z,
            "dx": float(p[:, 0].max() - p[:, 0].min()),
            "dy": float(p[:, 1].max() - p[:, 1].min()),
            "tx": float(np.median(cap[:, 0])), "ty": float(np.median(cap[:, 1])),
            "blue": float(mean_rgb[2] - mean_rgb[0]),
            "rgb": [round(float(v), 1) for v in mean_rgb],
        })
    out.sort(key=lambda c: -c["n"])
    for c in out:
        api.log("[%s] cl n=%5d c=(%.3f,%.3f) top=(%.3f,%.3f) z=%.3f h=%.3f "
                "d=(%.3f,%.3f) blue=%.1f rgb=%s" %
                (tag, c["n"], c["cx"], c["cy"], c["tx"], c["ty"], c["top"], c["h"],
                 c["dx"], c["dy"], c["blue"], c["rgb"]))
    return table_z, out


def _geom_ok(c):
    foot = max(c["dx"], c["dy"])
    return (c["n"] >= 40 and c["h"] <= ARM_H_MAX
            and TGT_H_RANGE[0] <= c["h"] <= TGT_H_RANGE[1]
            and TGT_FOOT_RANGE[0] <= foot <= TGT_FOOT_RANGE[1])


def _dist_prior(c):
    return float(np.hypot(c["tx"] - PRIOR_OBJ_XY[0], c["ty"] - PRIOR_OBJ_XY[1]))


def _pick_target(api, cands, tag=""):
    geom = [c for c in cands if _geom_ok(c)]
    sig = [c for c in geom if c["blue"] >= TGT_BLUE_MIN]
    if sig:
        sig.sort(key=_dist_prior)
        api.log("target%s: signature match (%d geom, %d bluish)" % (tag, len(geom), len(sig)))
        return sig[0]
    if geom:
        geom.sort(key=_dist_prior)
        api.log("target%s: NO bluish match among %d geom candidates -> prior" % (tag, len(geom)))
        return geom[0]
    loose = [c for c in cands if c["n"] >= 40 and c["h"] <= ARM_H_MAX]
    if loose:
        loose.sort(key=_dist_prior)
        api.log("target%s: NO geom candidate -> loose nearest-prior" % tag)
        return loose[0]
    return None


def _pick_basket(cands):
    ok = [c for c in cands if c["cy"] > 0.10 and max(c["dx"], c["dy"]) > 0.10
          and c["h"] <= ARM_H_MAX]
    if not ok:
        return None
    ok.sort(key=lambda c: -(c["dx"] * c["dy"]))
    return ok[0]


def _holding(api):
    g = api.gripper()
    return g["effort"] > 1.0, g


# --------------------------------------------------------------------- policy
def _pick_and_place(api, cycle):
    table_z, cands = _clusters(api, "c%d" % cycle)
    tgt = _pick_target(api, cands, tag=str(cycle))
    bsk = _pick_basket(cands)

    if tgt is None:
        gx, gy = float(PRIOR_OBJ_XY[0]), float(PRIOR_OBJ_XY[1])
        api.log("c%d target: FALLBACK prior (%.3f,%.3f)" % (cycle, gx, gy))
    else:
        gx, gy = tgt["tx"], tgt["ty"]
        api.log("c%d target: (%.3f,%.3f) z=%.3f h=%.3f blue=%.1f d=(%.3f,%.3f)" %
                (cycle, gx, gy, tgt["top"], tgt["h"], tgt["blue"], tgt["dx"], tgt["dy"]))
    if bsk is None:
        bxy = PRIOR_BASKET_XY
        rel_z = float(np.clip(0.144 + RELEASE_CLEARANCE, *RELEASE_Z_RANGE))
        api.log("c%d basket: FALLBACK prior %s" % (cycle, np.round(bxy, 3).tolist()))
    else:
        bxy = np.array([bsk["cx"], bsk["cy"]])
        rel_z = float(np.clip(bsk["top"] + RELEASE_CLEARANCE, *RELEASE_Z_RANGE))
        api.log("c%d basket: (%.3f,%.3f) top=%.3f rel_z=%.3f d=(%.3f,%.3f)" %
                (cycle, bsk["cx"], bsk["cy"], bsk["top"], rel_z, bsk["dx"], bsk["dy"]))

    api.grip(OPEN_W)
    held = False
    for attempt in range(3):
        gz = GRASP_Z - 0.006 * attempt
        r1 = api.move([gx, gy, APPROACH_Z], seconds=2.0)
        r2 = api.move([gx, gy, gz], seconds=2.0)
        api.log("c%d a%d descend (%.3f,%.3f,%.3f) res=%.4f/%.4f eef=%s" %
                (cycle, attempt, gx, gy, gz, r1, r2, np.round(api.eef(), 4).tolist()))
        api.grip(CLOSE_W)
        api.settle(0.3)
        held, g = _holding(api)
        api.log("c%d a%d close: width=%.4f effort=%.2f held=%s" %
                (cycle, attempt, g["width_m"], g["effort"], held))
        api.move([gx, gy, LIFT_Z], seconds=2.0)
        held, g = _holding(api)
        api.log("c%d a%d lift: width=%.4f effort=%.2f held=%s eef=%s" %
                (cycle, attempt, g["width_m"], g["effort"], held,
                 np.round(api.eef(), 4).tolist()))
        if held:
            break
        api.grip(OPEN_W)
        _, c2 = _clusters(api, "c%dr%d" % (cycle, attempt))
        t2 = _pick_target(api, c2, tag="%d.r%d" % (cycle, attempt))
        if t2 is not None:
            gx, gy = t2["tx"], t2["ty"]
            api.log("c%d a%d re-perceived target (%.3f,%.3f)" % (cycle, attempt, gx, gy))

    if not held:
        api.log("c%d grasp failed after retries" % cycle)
        return False, bxy

    api.move([float(bxy[0]), float(bxy[1]), LIFT_Z], seconds=3.0)
    held, g = _holding(api)
    api.log("c%d over basket: eef=%s held=%s width=%.4f" %
            (cycle, np.round(api.eef(), 4).tolist(), held, g["width_m"]))
    api.move([float(bxy[0]), float(bxy[1]), rel_z], seconds=2.0)
    api.grip(OPEN_W)
    api.settle(0.5)
    api.log("c%d released at eef=%s" % (cycle, np.round(api.eef(), 4).tolist()))
    api.move([float(bxy[0]), float(bxy[1]), LIFT_Z], seconds=2.0)
    return True, bxy


def run(api):
    api.log("instruction: %r" % api.instruction())
    api.log("eef0=%s" % np.round(api.eef(), 4).tolist())
    # Fixed nominal sequence: one pick-and-place.  The only retry left is the
    # sensor-gated grasp retry inside _pick_and_place (gripper effort).
    placed, _bxy = _pick_and_place(api, 0)
    note = "placed" if placed else "no-grasp"
    api.settle(0.5)
    return note
