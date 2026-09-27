"""v4 -- pick the akita black bowl NOT between the plate and the ramekin and
place it on the plate.

Changes from v2, both driven by debug-seed receipts:

  1. FUSION-ROBUST DETECTION. v2 found vessels as connected components of a
     height band. On debug seeds 54/56/62 the two bowls and the ramekin abut,
     fuse into one component, and the circle fit returns one bowl of inflated
     radius (seed 56: r=0.074, grasp missed, grip lost on the lift). v3
     replaces the component fit with a constrained-radius Hough vote: the bowl
     rim radius is 0.0538 m on every clean detection, so voting for centres at
     that fixed radius separates two touching rims. Validated offline on 11
     debug seeds (51-58,60,62,64): two bowls + ramekin + plate on every one.
  2. CLOSED-LOOP PLACE. v2's transport stopped up to POS_TOL (12 mm) short and
     the following descent re-aimed and overshot (seed 58: bowl centre landed
     9 mm off the plate centre, which is most of the 10 mm of plate-over-bowl
     clearance, and it was released from 8.7 mm up -> the one failure).
     v3 cancels the measured lateral bias at transport height BEFORE
     descending, and releases with the bowl base 3 mm above the plate.
  3. (v4) BIAS-CANCEL IS GUARDED, AND MOVES ARE STEP-CAPPED. On debug seed 56
     the plate sits further out (+x 0.129) than on any other seed and the arm
     saturates against its reach envelope there. A repeatable stop at the
     envelope is NOT tracking slop, so doubling the command only pushes the
     target further out of reach: v3's bias move made the error worse (0.017 ->
     0.026) and the episode spent 456 of its 500 sim steps. v4 applies the
     cancel only for residuals in [0.005, 0.015] -- the tracking-slop band --
     and shortens every `seconds` so a stalled move burns a smaller cap.
"""
import json
from collections import deque

import numpy as np

PROVENANCE = {
    "TABLE_Z_FALLBACK": {
        "source": "debug seeds 51-58,60,62,64 cam_high depth: modal z of the "
                  "tabletop = 0.9008-0.9010 m (per-frame mode is used; this is "
                  "only the sanity bound)",
        "allowed": True},
    "TIP_OFFSET": {
        "source": "debug seed 51, v1 calibration: a closed gripper descending "
                  "on the 0.9008 m table stalls at eef z=0.9092 -> the "
                  "fingertips sit 0.0084 m below the eef reference",
        "allowed": True},
    "UNDERSHOOT": {
        "source": "debug seed 51, v1 descents: achieved eef z ran 0.008-0.011 m "
                  "above the command (POS_TOL=0.012, generic controller "
                  "mechanics) -> command 0.010 m low",
        "allowed": True},
    "R_BOWL": {
        "source": "debug seeds 51-58,60,62,64: Kasa fit of each bowl's rim ring "
                  "gives 0.0537-0.0539 m on every clean (unfused) detection",
        "allowed": True},
    "R_RAM": {
        "source": "same seeds: the ramekin's rim ring fits 0.0416-0.0421 m",
        "allowed": True},
    "BOWL_BAND": {
        "source": "same seeds: bowl tops sit 0.051-0.053 m above the table and "
                  "the ramekin top 0.043 m, so a 0.046 m cut holds bowls only "
                  "even when the two props touch in XY",
        "allowed": True},
    "RAM_BAND": {
        "source": "same seeds: the ramekin rim lies 0.034-0.043 m above the "
                  "table; bowl interior points in that band are excluded by "
                  "distance from the two bowl centres",
        "allowed": True},
    "PLATE_BAND": {
        "source": "same seeds: the plate top sits 0.019 m above the table",
        "allowed": True},
    "PLATE_MIN_BRIGHT": {
        "source": "same seeds cam_high rgb: the plate's mean rgb is ~(153,142,"
                  "139) and the cookie box, which shares its height (0.020 m), "
                  "is ~(93,66,46); a mean-channel cut at 110 separates them",
        "allowed": True},
    "GRASP_DEPTH": {
        "source": "debug-seed choice validated by v2 (14/15): fingertips 0.025 "
                  "m below the measured rim top, inside a 0.051 m tall bowl",
        "allowed": True},
    "PLACE_CLEAR": {
        "source": "debug seed 58 receipt: v2 released from 0.0087 m up and the "
                  "bowl left the plate; 0.003 m is a seat rather than a drop",
        "allowed": True},
    "WORKSPACE": {
        "source": "debug seeds 51-58,60,62,64: every vessel lies in "
                  "x[-0.55,0.35], y[0.00,0.55]; the cabinet/stove fixture is "
                  "entirely at y<-0.03",
        "allowed": True},
    "V_SKY": {
        "source": "same seeds: image rows above v=180 hold only the robot body "
                  "and the back wall",
        "allowed": True},
    "BIAS_BAND": {
        "source": "debug seed 56 receipt: at the +x reach envelope a move's "
                  "residual was 0.017-0.026 and re-commanding past it made it "
                  "worse, while ordinary tracking slop is bounded by "
                  "POS_TOL=0.012 -> cancel only residuals in [0.005,0.015]",
        "allowed": True},
    "LIFT_Z": {
        "source": "debug-seed choice: 1.035 m clears every measured prop top "
                  "(max 0.953 m) by >0.08 m",
        "allowed": True},
}

TABLE_Z_FALLBACK = 0.9008
TIP_OFFSET = 0.0084
UNDERSHOOT = 0.010
R_BOWL = 0.0538
R_RAM = 0.0418
BOWL_BAND = 0.046
RAM_BAND = (0.032, 0.047)
PLATE_BAND = (0.013, 0.026)
PLATE_MIN_BRIGHT = 110.0
GRASP_DEPTH = 0.025
PLACE_CLEAR = 0.003
WORKSPACE = ((-0.55, 0.35), (0.00, 0.55))
V_SKY = 180
LIFT_Z = 1.035


def _log(api, **kw):
    api.log(json.dumps(kw, default=float))


def _cloud(f):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.asarray(f.depth, float)
    d = np.where(np.isfinite(d) & (d > 0), d, np.nan)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    return (np.stack([x, y, d, np.ones_like(d)], -1) @ T.T)[..., :3]


def _kasa(x, y):
    A = np.stack([x, y, np.ones_like(x)], 1)
    sol, *_ = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)
    return float(sol[0] / 2.0), float(sol[1] / 2.0)


def _hough(px, py, R, tol=0.008, step=0.004, nmax=2, sep=0.075, frac=0.35):
    """Centres of up to nmax circles of KNOWN radius R among 2-D points.

    A fixed radius is what separates two rims that touch: a point on rim A
    votes only for centres one bowl-radius away from it, and the two rims'
    vote peaks stay distinct however close the bowls sit.
    """
    if px.size < 40:
        return []
    gx = np.arange(px.min() - R, px.max() + R + step, step)
    gy = np.arange(py.min() - R, py.max() + R + step, step)
    GX, GY = np.meshgrid(gx, gy, indexing="ij")
    d = np.sqrt((px[None, None, :] - GX[..., None]) ** 2
                + (py[None, None, :] - GY[..., None]) ** 2)
    score = (np.abs(d - R) < tol).sum(-1).astype(float)
    out, best = [], None
    for _ in range(nmax):
        i = int(np.argmax(score))
        s = float(score.flat[i])
        if best is None:
            best = s
        elif s < frac * best:
            break
        cx, cy = float(GX.flat[i]), float(GY.flat[i])
        m = np.abs(np.sqrt((px - cx) ** 2 + (py - cy) ** 2) - R) < tol * 1.6
        if int(m.sum()) >= 20:
            rx, ry = _kasa(px[m], py[m])
            if np.hypot(rx - cx, ry - cy) < 0.02:
                cx, cy = rx, ry
        out.append(dict(cx=cx, cy=cy, n=int(s)))
        score[np.sqrt((GX - cx) ** 2 + (GY - cy) ** 2) < sep] = -1
    return out


def _components(mask, min_px):
    seen = np.zeros(mask.shape, bool)
    out = []
    for v0, u0 in np.argwhere(mask):
        if seen[v0, u0]:
            continue
        q = deque([(v0, u0)])
        seen[v0, u0] = True
        pix = []
        while q:
            v, u = q.popleft()
            pix.append((v, u))
            for dv in (-1, 0, 1):
                for du in (-1, 0, 1):
                    a, b = v + dv, u + du
                    if (0 <= a < mask.shape[0] and 0 <= b < mask.shape[1]
                            and mask[a, b] and not seen[a, b]):
                        seen[a, b] = True
                        q.append((a, b))
        if len(pix) >= min_px:
            out.append(np.array(pix))
    out.sort(key=len, reverse=True)
    return out


def perceive(api):
    f = api.capture("cam_high")
    P = _cloud(f)
    rgb = np.asarray(f.rgb, float)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    (x0, x1), (y0, y1) = WORKSPACE
    ws = np.isfinite(Z) & (X > x0) & (X < x1) & (Y > y0) & (Y < y1)
    ws[:V_SKY, :] = False

    flat = Z[ws]
    hist, edges = np.histogram(flat, bins=400)
    tz = float(edges[int(np.argmax(hist))]) if flat.size else TABLE_Z_FALLBACK
    if not (TABLE_Z_FALLBACK - 0.05 < tz < TABLE_Z_FALLBACK + 0.05):
        tz = TABLE_Z_FALLBACK
    hgt = Z - tz

    bm = ws & (hgt > BOWL_BAND) & (hgt < 0.15)
    bowls = _hough(X[bm], Y[bm], R_BOWL, nmax=2)
    for b in bowls:
        ring = bm & (np.abs(np.sqrt((X - b["cx"]) ** 2 + (Y - b["cy"]) ** 2)
                            - R_BOWL) < 0.012)
        b["ztop"] = float(np.nanmax(Z[ring])) if ring.any() else tz + 0.051

    rm = ws & (hgt > RAM_BAND[0]) & (hgt < RAM_BAND[1])
    for b in bowls:
        rm &= ((X - b["cx"]) ** 2 + (Y - b["cy"]) ** 2) > (R_BOWL + 0.014) ** 2
    rr = _hough(X[rm], Y[rm], R_RAM, nmax=1)
    ram = rr[0] if rr else None

    pm = (ws & (hgt > PLATE_BAND[0]) & (hgt < PLATE_BAND[1])
          & (rgb.mean(-1) > PLATE_MIN_BRIGHT))
    plate = None
    pc = _components(pm, 200)
    if pc:
        pix = pc[0]
        xs, ys = X[pix[:, 0], pix[:, 1]], Y[pix[:, 0], pix[:, 1]]
        cx, cy = _kasa(xs, ys)
        plate = dict(cx=cx, cy=cy, n=len(pix),
                     ztop=float(np.nanmax(Z[pix[:, 0], pix[:, 1]])))

    _log(api, stage="perceive", table_z=round(tz, 4),
         bowls=[{k: (round(v, 4) if isinstance(v, float) else v)
                 for k, v in b.items()} for b in bowls],
         ramekin=(None if ram is None else
                  {k: (round(v, 4) if isinstance(v, float) else v)
                   for k, v in ram.items()}),
         plate=(None if plate is None else
                {k: (round(v, 4) if isinstance(v, float) else v)
                 for k, v in plate.items()}))
    return tz, bowls, ram, plate


def pick_target(api, bowls, ram, plate):
    """The bowl NOT between the plate and the ramekin.

    Betweenness = |P-B| + |B-R| - |P-R|, which is 0 on the segment and grows
    off it. The target is the argmax.
    """
    if len(bowls) < 2:
        return (bowls[0] if bowls else None)
    p = np.array([plate["cx"], plate["cy"]])
    if ram is None:      # no ramekin: fall back to the bowl further from the plate
        ex = [float(np.hypot(b["cx"] - p[0], b["cy"] - p[1])) for b in bowls]
    else:
        r = np.array([ram["cx"], ram["cy"]])
        base = float(np.linalg.norm(p - r))
        ex = [float(np.linalg.norm(p - np.array([b["cx"], b["cy"]]))
                    + np.linalg.norm(np.array([b["cx"], b["cy"]]) - r) - base)
              for b in bowls]
    _log(api, stage="between", excess=[round(e, 4) for e in ex],
         has_ram=ram is not None)
    return bowls[int(np.argmax(ex))]


def goto(api, xyz, seconds=2.0, tag=""):
    r = api.move([float(v) for v in xyz], seconds=seconds)
    e = api.eef()
    _log(api, tag=tag, cmd=[round(float(v), 4) for v in xyz],
         eef=[round(float(v), 4) for v in e], res=round(float(r), 4))
    return e


def goto_xy(api, x, y, z, seconds=1.0, tag="", lo=0.005, hi=0.015):
    """Move, then cancel the measured lateral bias once -- but only inside the
    tracking-slop band.

    A move legitimately stops up to POS_TOL (0.012 m) short, and on the
    following descent that leftover error is re-aimed at and overshot;
    cancelling it here keeps the descent's lateral command near zero. A
    residual LARGER than that band is not slop, it is the arm sitting on its
    reach envelope (debug seed 56, where the plate is at +x 0.129), and
    re-commanding past it only wastes the step budget.
    """
    e = goto(api, [x, y, z], seconds=seconds, tag=tag)
    dx, dy = x - e[0], y - e[1]
    err = float(np.hypot(dx, dy))
    if lo < err < hi:
        e = goto(api, [x + dx, y + dy, z], seconds=0.6, tag=tag + "_bias")
    else:
        _log(api, tag=tag + "_nobias", err=round(err, 4))
    return e


def run(api):
    _log(api, instr=api.instruction())
    tz, bowls, ram, plate = perceive(api)
    if not bowls or plate is None:
        api.log("perception incomplete: bowls=%d plate=%s" % (len(bowls), plate))
        return
    tgt = pick_target(api, bowls, ram, plate)

    # Rim pinch: the fingers separate along world y with the home wrist, so an
    # offset of one rim radius along y puts one finger inside the bowl and one
    # outside. Take the arc that points AWAY from the other vessels.
    others = [np.array([b["cx"], b["cy"]]) for b in bowls if b is not tgt]
    if ram is not None:
        others.append(np.array([ram["cx"], ram["cy"]]))
    side = 1.0
    if others and np.mean([o[1] for o in others]) > tgt["cy"]:
        side = -1.0

    rim_top = tgt["ztop"]
    gx = tgt["cx"]
    gy = tgt["cy"] + side * R_BOWL
    tip_z = rim_top - GRASP_DEPTH
    eef_z = tip_z + TIP_OFFSET
    _log(api, stage="plan", grasp=[round(gx, 4), round(gy, 4)], side=side,
         rim_top=round(rim_top, 4), eef_z=round(eef_z, 4))

    api.grip(0.08)
    goto_xy(api, gx, gy, LIFT_Z, seconds=1.0, tag="hover")
    goto(api, [gx, gy, eef_z - UNDERSHOOT], seconds=0.8, tag="descend")
    e = api.eef()
    if e[2] > eef_z + 0.004:
        goto(api, [gx, gy, 2 * (eef_z - UNDERSHOOT) - e[2]], seconds=0.6,
             tag="descend2")
    api.grip(0.0)
    api.settle(0.2)
    _log(api, stage="closed", grip=api.gripper(),
         eef=[round(float(v), 4) for v in api.eef()])

    goto(api, [gx, gy, LIFT_Z], seconds=1.0, tag="lift")
    _log(api, stage="lifted", grip=api.gripper())

    # The bowl hangs from the rim: its base is (rim_top - tip_z) below the
    # fingertips and its centre trails the eef by the same rim offset.
    hang = rim_top - tip_z
    place_eef_z = plate["ztop"] + hang + PLACE_CLEAR + TIP_OFFSET
    px = plate["cx"]
    py = plate["cy"] + side * R_BOWL
    goto_xy(api, px, py, LIFT_Z, seconds=1.0, tag="over_plate")
    goto(api, [px, py, place_eef_z - UNDERSHOOT], seconds=0.8, tag="lower")
    api.grip(0.08)
    api.settle(0.3)
    _log(api, stage="released", grip=api.gripper(),
         eef=[round(float(v), 4) for v in api.eef()])
    goto(api, [px, py, LIFT_Z], seconds=0.6, tag="retreat")
    api.log("v4 done")
