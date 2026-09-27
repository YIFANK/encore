"""c2k1clean / obj_ketchup_pos_k1 -- v5.

Find the ketchup by appearance, grasp it with a sensor-verified ladder, and
drop it through the centre of the basket's rim ring.

What the probes taught me:
  * table plane sits at z = 0.0011 (depth-fitted, identical on 51/53/55);
  * the camera is at (0.897, 0, 0.65) looking 32 deg below horizontal, so a
    blob's y-extent is its true width but its x-extent is only the front
    sliver -> the median x of a blob is biased toward the camera;
  * descending onto a 0.113-tall bottle stalls the eef at z = 0.042 on all
    three seeds, i.e. the gripper's palm sits 0.0709 m above the eef site;
  * the ketchup is the blob with an achromatic (grey) cap over a red body --
    exactly what the pack keyframe shows for the object the demo removed;
  * the basket's median x sits on its near wall, so the drop point must come
    from the midpoint of the rim ring's bounding box instead (v2 dropped the
    bottle onto the rim; v3/v4 dropped it inside).
"""
import numpy as np

PROVENANCE = {
    "DROP_CLEARANCE": {
        "source": "pack.json demo0: release ee z 0.1768 minus the basket top z "
                  "0.142 measured from debug-seed depth = 0.035",
        "allowed": True},
    "CARRY_Z": {
        "source": "pack.json demo0 ee_path6 transit apex z ~0.29",
        "allowed": True},
    "PALM_OFFSET": {
        "source": "debug seeds 51/53/55: a descent commanded to z=0.0133 over a "
                  "blob of top_z=0.113 stalled at z=0.0422/0.0422/0.0419 -> the "
                  "colliding part of the gripper is 0.0709 m above the eef site",
        "allowed": True},
    "GRASP_BELOW_TOP": {
        "source": "PALM_OFFSET minus a clearance margin: the fingers may only "
                  "straddle the top 0.071 m of a standing object",
        "allowed": True},
    "WORKSPACE_BOX": {
        "source": "pack.json demo0 ee_path6 xy span, padded; crops the cloud",
        "allowed": True},
    "TABLE_MIN_H": {
        "source": "debug-seed measurement: height above the fitted table plane "
                  "that separates depth noise from objects",
        "allowed": True},
    "ARM_MAX_H": {
        "source": "debug seeds 51/53/55: the robot blob reaches h=0.475 while "
                  "every tabletop object measured h<=0.148",
        "allowed": True},
    "CAP_BAND": {
        "source": "pack keyframe demo0_t0000: the removed object's top ~6 px are "
                  "its cap; expressed as a fraction of measured blob height",
        "allowed": True},
    "GRASP_LADDER": {
        "source": "PALM_OFFSET (0.0709) bounds how far below an object's top "
                  "the eef can go; the ladder spans that band, ordered by the "
                  "depth that worked on debug seeds 51-65",
        "allowed": True},
    "HELD_MIN_W": {
        "source": "debug seeds 51/53/55: an empty close settles to width "
                  "0.0010 with effort 0.05; a held bottle reads 0.0335 / 3.00",
        "allowed": True},
    "RIM_BAND": {
        "source": "debug-seed measurement: depth band below the basket's top z "
                  "that still belongs to its rim rather than its interior",
        "allowed": True},
    "TARGET_H_RANGE": {
        "source": "debug seeds 51/53/55: bottle blobs measured h in 0.11-0.15, "
                  "the basket 0.140 with a 0.15 m footprint, the can 0.079",
        "allowed": True},
}

CARRY_Z = 0.29
DROP_CLEARANCE = 0.035
PALM_OFFSET = 0.0709
GRASP_BELOW_TOP = 0.050
WS_X = (-0.45, 0.45)
WS_Y = (-0.50, 0.52)
TABLE_MIN_H = 0.020
ARM_MAX_H = 0.32
CAP_BAND = 0.25
RIM_BAND = 0.025
GRASP_LADDER = (0.050, 0.035, 0.062, 0.025)
HELD_MIN_W = 0.004
TARGET_H_RANGE = (0.09, 0.20)
TARGET_W_MAX = 0.10


# ---------------------------------------------------------------- perception
def cloud(frame):
    d = np.asarray(frame.depth, dtype=float)
    K = np.asarray(frame.intrinsics, dtype=float)
    T = np.asarray(frame.t_base_cam, dtype=float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    z = np.where(np.isfinite(d) & (d > 0), d, np.nan)
    x = (uu - K[0, 2]) * z / K[0, 0]
    y = (vv - K[1, 2]) * z / K[1, 1]
    pts = np.stack([x, y, z, np.ones_like(z)], axis=-1)
    base = pts @ T.T
    return base[..., :3], np.isfinite(base[..., 2])


def fit_table(P, ok):
    m = ok & (P[..., 0] > WS_X[0]) & (P[..., 0] < WS_X[1]) \
           & (P[..., 1] > WS_Y[0]) & (P[..., 1] < WS_Y[1])
    zs = P[..., 2][m]
    zs = zs[np.isfinite(zs)]
    if zs.size == 0:
        return 0.0
    hist, edges = np.histogram(zs, bins=200, range=(-0.4, 0.6))
    i = int(np.argmax(hist))
    sel = zs[(zs >= edges[i] - 0.01) & (zs <= edges[i + 1] + 0.01)]
    return float(np.median(sel))


def components(mask, min_px=40, limit=24):
    out = []
    todo = mask.copy()
    while todo.any() and len(out) < limit:
        seed = np.zeros_like(todo)
        seed.flat[int(np.argmax(todo))] = True
        prev = 0
        while True:
            g = seed.copy()
            g[1:, :] |= seed[:-1, :]
            g[:-1, :] |= seed[1:, :]
            g[:, 1:] |= seed[:, :-1]
            g[:, :-1] |= seed[:, 1:]
            seed = g & todo
            n = int(seed.sum())
            if n == prev:
                break
            prev = n
        todo &= ~seed
        if prev >= min_px:
            out.append(seed)
    return out


def describe(seed, P, rgb, tz):
    ys, xs = np.where(seed)
    good = np.isfinite(P[..., 2][seed])
    px = P[..., 0][seed][good]
    py = P[..., 1][seed][good]
    pz = P[..., 2][seed][good]
    cols = rgb[seed].astype(float)[good]
    if px.size < 20:
        return None
    top = float(np.percentile(pz, 98))
    h = top - tz
    wy = float(np.percentile(py, 97) - np.percentile(py, 3))
    capm = pz > top - max(CAP_BAND * h, 0.012)
    bodym = (pz > tz + 0.15 * h) & (pz < tz + 0.70 * h)
    cap = cols[capm].mean(0) if capm.any() else np.zeros(3)
    body = cols[bodym].mean(0) if bodym.any() else np.zeros(3)
    topm = pz > top - 0.008
    rimm = pz > top - RIM_BAND
    return {
        "rim_x": [float(np.percentile(px[rimm], 1)),
                  float(np.percentile(px[rimm], 99))] if rimm.sum() > 20 else None,
        "rim_y": [float(np.percentile(py[rimm], 1)),
                  float(np.percentile(py[rimm], 99))] if rimm.sum() > 20 else None,
        "x01": float(np.percentile(px, 1)), "x99": float(np.percentile(px, 99)),
        "y01": float(np.percentile(py, 1)), "y99": float(np.percentile(py, 99)),
        "n": int(px.size),
        "bbox": [int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())],
        "top_z": top, "h": float(h), "wy": wy,
        "wx": float(np.percentile(px, 97) - np.percentile(px, 3)),
        "x50": float(np.median(px)), "x98": float(np.percentile(px, 98)),
        "y50": float(np.median(py)),
        "xtop": float(np.mean(px[topm])) if topm.any() else float(np.median(px)),
        "ytop": float(np.mean(py[topm])) if topm.any() else float(np.median(py)),
        "cap": cap, "body": body,
        "touches_top": bool(ys.min() <= 2),
    }


def look(api):
    f = api.capture("cam_high")
    P, ok = cloud(f)
    tz = fit_table(P, ok)
    inbox = ok & (P[..., 0] > WS_X[0]) & (P[..., 0] < WS_X[1]) \
               & (P[..., 1] > WS_Y[0]) & (P[..., 1] < WS_Y[1])
    blobs = []
    for s in components(inbox & (P[..., 2] > tz + TABLE_MIN_H)):
        d = describe(s, P, f.rgb, tz)
        if d is not None:
            blobs.append(d)
    return tz, blobs


def cap_achroma(b):
    return float(b["cap"].max() - b["cap"].min())


def body_red(b):
    r, g, bl = b["body"]
    return float(r - max(g, bl))


def pick_target(blobs, basket=None):
    """The ketchup: a bottle-sized blob with a grey cap over a red body.
    Ranked, not thresholded."""
    onscene = [b for b in blobs
               if not b["touches_top"] and b["h"] < ARM_MAX_H and b is not basket]
    cand = [b for b in onscene
            if TARGET_H_RANGE[0] < b["h"] < TARGET_H_RANGE[1]
            and b["wy"] < TARGET_W_MAX]
    if not cand:
        # The size gate is a convenience, not the evidence.  If nothing passes
        # it -- e.g. the ketchup ends up touching a neighbour and the two
        # merge into one blob -- rank the whole scene rather than give up.
        cand = [b for b in onscene if b["h"] > TARGET_H_RANGE[0]]
    if not cand:
        return None, cand
    return max(cand, key=lambda b: body_red(b) - cap_achroma(b)), cand


def pick_basket(blobs):
    cand = [b for b in blobs if not b["touches_top"] and b["h"] < ARM_MAX_H]
    if not cand:
        return None
    return max(cand, key=lambda b: b["wy"] * max(b["wx"], 0.001))


# -------------------------------------------------------------------- policy
def run(api):
    tz, blobs = look(api)
    api.log("table_z=%.4f nblobs=%d eef=%s"
            % (tz, len(blobs), np.round(api.eef(), 4).tolist()))
    for i, b in enumerate(blobs):
        api.log("b%d n=%d box=%s h=%.3f top=%.3f wx=%.3f wy=%.3f x50=%.3f "
                "x98=%.3f y50=%.3f xtop=%.3f ytop=%.3f cap=%s body=%s "
                "capA=%.1f red=%.1f arm=%s"
                % (i, b["n"], b["bbox"], b["h"], b["top_z"], b["wx"], b["wy"],
                   b["x50"], b["x98"], b["y50"], b["xtop"], b["ytop"],
                   b["cap"].round(1).tolist(), b["body"].round(1).tolist(),
                   cap_achroma(b), body_red(b), b["touches_top"]))

    basket = pick_basket(blobs)
    tgt, cand = pick_target(blobs, basket)
    if basket is None or tgt is None:
        api.log("ABORT: basket=%s target=%s" % (basket is not None, tgt is not None))
        return
    api.log("ranked: %s" % [(round(b["x50"], 3), round(b["y50"], 3),
                             round(body_red(b) - cap_achroma(b), 1))
                            for b in sorted(cand, key=lambda b: -(body_red(b) - cap_achroma(b)))])

    # v2 evidence: x98 - radius overshoots by ~13 mm (the descent clipped the
    # cap and stalled); the top-band mean lands on the axis.
    tx = tgt["xtop"]
    ty = tgt["ytop"]
    gz = tgt["top_z"] - GRASP_BELOW_TOP
    api.log("TARGET x98=%.3f -> tx=%.3f ty=%.3f top=%.3f gz=%.3f xtop=%.3f"
            % (tgt["x98"], tx, ty, tgt["top_z"], gz, tgt["xtop"]))

    # The whole rim ring of an open container is visible from a 32 deg
    # elevation view (we look INTO it), so the midpoint of the rim's bounding
    # box is the opening centre.  The blob's median x is not: most of its
    # pixels sit on the near wall, which drags the drop point toward the rim.
    if basket["rim_x"] is None:
        bx, by = basket["x50"], basket["y50"]
    else:
        bx = 0.5 * (basket["rim_x"][0] + basket["rim_x"][1])
        by = 0.5 * (basket["rim_y"][0] + basket["rim_y"][1])
    bz = basket["top_z"] + DROP_CLEARANCE
    api.log("BASKET x50=%.3f y50=%.3f rim_x=%s rim_y=%s x01/99=%.3f/%.3f "
            "-> (%.3f,%.3f) top=%.3f drop_z=%.3f"
            % (basket["x50"], basket["y50"],
               np.round(basket["rim_x"], 3).tolist(),
               np.round(basket["rim_y"], 3).tolist(),
               basket["x01"], basket["x99"], bx, by, basket["top_z"], bz))

    # Grasp with my own receipt: effort reads 3.0 only while the jaws are
    # closed on something, so a ladder of grasp heights can be retried until
    # the sensor agrees.  No success feedback is involved.
    held = False
    for k, dz in enumerate(GRASP_LADDER):
        z = tgt["top_z"] - dz
        api.grip(0.08)
        api.move([tx, ty, max(0.20, z + 0.10)], seconds=2.0)
        r = api.move([tx, ty, z], seconds=2.0)
        api.grip(0.0)
        api.settle(0.3)
        g = api.gripper()
        api.log("try%d z=%.3f r=%.4f eef=%s w=%.4f e=%.2f"
                % (k, z, r, np.round(api.eef(), 4).tolist(),
                   g["width_m"], g["effort"]))
        api.move([tx, ty, CARRY_Z], seconds=2.5)
        g = api.gripper()
        api.log("lift%d w=%.4f e=%.2f eef=%s"
                % (k, g["width_m"], g["effort"], np.round(api.eef(), 4).tolist()))
        if g["effort"] >= 3.0 and g["width_m"] > HELD_MIN_W:
            held = True
            break
        # nothing in the jaws: re-perceive, the object may have been nudged
        tz, blobs = look(api)
        t2, _ = pick_target(blobs, pick_basket(blobs))
        if t2 is not None:
            tgt, tx, ty = t2, t2["xtop"], t2["ytop"]
            api.log("re-aim -> (%.3f,%.3f) top=%.3f" % (tx, ty, t2["top_z"]))
    api.log("held=%s" % held)

    api.move([0.5 * (tx + bx), 0.5 * (ty + by), CARRY_Z], seconds=2.0)
    api.move([bx, by, CARRY_Z], seconds=2.0)
    r = api.move([bx, by, bz], seconds=2.0)
    g = api.gripper()
    api.log("over basket r=%.4f eef=%s w=%.4f e=%.2f"
            % (r, np.round(api.eef(), 4).tolist(), g["width_m"], g["effort"]))
    api.grip(0.08)
    api.settle(0.5)
    api.move([bx, by, CARRY_Z], seconds=2.0)

    tz2, blobs2 = look(api)
    api.log("post table_z=%.4f n=%d" % (tz2, len(blobs2)))
    for i, b in enumerate(blobs2):
        api.log("post b%d box=%s h=%.3f x50=%.3f y50=%.3f cap=%s body=%s"
                % (i, b["bbox"], b["h"], b["x50"], b["y50"],
                   b["cap"].round(1).tolist(), b["body"].round(1).tolist()))
