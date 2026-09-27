"""c2clean / goal_put_cream_cheese_in_bowl_task_k3 -- "put the wine bottle in the bowl".

Mechanism
---------
The k3 pack demos "put the cream cheese in the bowl" (my TARGET, wrong object);
the mate pack demos "put the wine bottle on top of the cabinet" (my OBJECT,
wrong target).  The intent joins them: grasp the wine bottle the way the mate
pack does, release it over the bowl the way the k3 pack does.

Perception (cam_high RGB-D only).  Height-band mask above the measured table,
4-connected clustered:

  * the WINE BOTTLE is the only cluster standing 0.10-0.30 m proud of the table
    (arm 0.45, cabinet 0.34, stove 0.058, bowl 0.051, plate/box 0.019).  Its
    axis is the bbox centre of the top 5 mm of the cluster -- the cap's
    horizontal face, which the oblique camera sees whole, unlike the body,
    whose far side is hidden and whose bbox is therefore biased +x.
  * On some seeds the bottle and the bowl fuse into one pixel component (the
    bottle occludes part of the rim).  v1 rejected those on footprint and
    scored 0/3 there.  v2 therefore finds the bottle FIRST and then re-masks
    the band with a 0.032 m cylinder around the bottle axis removed, which
    restores the bowl as its own component on every debug seed (n ~ 2340,
    identical to the unfused seeds).
  * the BOWL is found by CIRCULARITY, not by size.  Every remaining cluster
    standing 0.03-0.09 m proud of the table gets a Kasa circle fit to the top
    6 mm of its silhouette; the bowl is the one whose fitted radius lands in
    0.040-0.070 m with a small residual.  v2 ranked these by height with a
    footprint window instead, and that is fragile: the stove slab stands
    0.058 m -- taller than the 0.051 m bowl and inside the same height window
    -- and was excluded only because the workspace crop happened to clip its
    footprint to 0.238 m.  Measured on all 15 debug seeds the rim fit gives
    r = 0.0536-0.0537 m at residual 0.0013 m, while the stove gives r = 0.022
    at residual 0.0093 and the far table edge r = 0.080 at residual 0.0208:
    the radius window alone separates them, and the residual separates them
    again by 7x.

Grasp: the mate pack closes at z = 0.992 / 1.024 / 1.029 while the bottle top
measures 1.0588 -- i.e. on the narrow neck, 3-7 cm below the cap.  We take
top-0.035.  The neck measures 0.0135 m across, so a real bite leaves the
fingers open by about that much.

Place: holding the neck leaves the bottle base a fixed (grasp_z - table)
below the hand, so lowering the hand to base_target + that offset stands the
bottle on the bowl's interior floor (measured 0.0065 m above the table) with
the fingers still ~8 cm clear of the 0.051 m rim.
"""
import json

import numpy as np

PROVENANCE = {
    "TABLE_MARGIN": {
        "source": "debug seeds 51-65 cam_high depth: workspace floor median "
                  "z = 0.901 on all 15; a 12 mm band margin separates the flat "
                  "props (plate/box tops at 0.920) from the floor",
        "allowed": True},
    "BOTTLE_H_LO/BOTTLE_H_HI": {
        "source": "debug seeds 51-65: the bottle cluster measures 0.1577-0.1579 m "
                  "tall on every seed; the other band clusters measure 0.019 "
                  "(plate, cream-cheese box), 0.051 (bowl), 0.058 (stove slab), "
                  "0.344 (cabinet) and 0.450 (arm)",
        "allowed": True},
    "CAP_BAND": {
        "source": "debug-seed bottle profile: the neck holds a constant 0.0135 m "
                  "diameter from z=1.007 to the 1.0588 cap, so the top 5 mm is "
                  "all cap and its bbox centre is the bottle axis (verified "
                  "against the body's near-edge + radius on seed 51: -0.2027 "
                  "vs -0.2028)",
        "allowed": True},
    "BOTTLE_CLEAR_R": {
        "source": "debug-seed bottle profile: widest body diameter 0.043 m, so "
                  "radius 0.0215; 0.032 m clears it with margin and still "
                  "leaves 2286-2377 bowl pixels on all 15 debug seeds",
        "allowed": True},
    "BOWL_H_LO/BOWL_H_HI": {
        "source": "debug seeds 51-65: bowl rim stands 0.051 m above the table; "
                  "plate and cream cheese 0.019 m, stove slab 0.058 m",
        "allowed": True},
    "BOWL_FOOT_HI": {
        "source": "debug seeds 51-65: bowl footprint 0.110-0.112 m; a loose "
                  "0.30 m cap only drops the cabinet (0.55 m) and keeps every "
                  "tabletop prop in the running for the circularity test",
        "allowed": True},
    "RIM_RES_MAX": {
        "source": "debug seeds 51-65 rim-band Kasa residual: bowl 0.0012-"
                  "0.0013 m, stove slab 0.0090-0.0093 m, far table edge "
                  "0.0208 m; 0.005 sits in the 7x gap",
        "allowed": True},
    "RIM_BAND": {
        "source": "debug seeds 51-65: the top 6 mm of the bowl cluster yields "
                  "516-574 rim points and a Kasa circle fit of r = 0.0536-"
                  "0.0537 m on every seed",
        "allowed": True},
    "RIM_R_LO/RIM_R_HI": {
        "source": "debug seeds 51-65: the fitted rim radius is 0.0536-0.0537 m "
                  "on every seed, while the two other props inside the bowl "
                  "height window fit r = 0.022 (stove slab) and r = 0.080 "
                  "(far table edge); 0.040-0.070 brackets the bowl alone",
        "allowed": True},
    "WS_X/WS_Y": {
        "source": "debug-seed depth: the tabletop and its props deproject "
                  "inside x(-0.55,0.45) y(-0.55,0.55); the crop is set wide "
                  "enough that no prop footprint is clipped by it, since a "
                  "clipped footprint is what made v2's size test fragile",
        "allowed": True},
    "NECK_DROP": {
        "source": "mate pack keyframes (gripper_cmd -1 -> +1): grasp z = "
                  "1.029 / 0.992 / 1.024; debug-seed bottle top = 1.0588, so "
                  "the demonstrated close sits 0.030-0.067 below the cap",
        "allowed": True},
    "BOWL_FLOOR_RISE": {
        "source": "debug seeds 51/53/55 cam_high depth within 0.02 m of the rim "
                  "centre: z = 0.9075, i.e. 0.0065 above the 0.901 table",
        "allowed": True},
    "BASE_CLEARANCE": {
        "source": "debug-seed choice: release the bottle base 6 mm above the "
                  "measured bowl floor so the descent is never a jam",
        "allowed": True},
    "HOVER_RISE": {
        "source": "debug-seed choice; the open jaw (0.0778 m, read from "
                  "api.gripper at reset) must clear the 0.015 m cap before it "
                  "descends",
        "allowed": True},
    "CARRY_RISE": {
        "source": "debug-seed geometry: the carried base must clear the 0.051 m "
                  "bowl rim and the 0.058 m stove slab; 0.10 m of lift puts it "
                  "0.05 m above both",
        "allowed": True},
    "GRIP_CLOSED/GRIP_OPEN": {
        "source": "api.grip contract (< 0.025 m closes, else opens) and the "
                  "0.0778 m open width read from api.gripper at reset",
        "allowed": True},
    "HOLD_GAP_LO/HOLD_GAP_HI": {
        "source": "mate pack held gripper_state sums (0.0157/0.0246/0.0165) and "
                  "the debug-seed neck diameter 0.0135 m: a real bite leaves "
                  "the fingers 0.006-0.045 m apart, and 0.045 is the widest "
                  "part of the bottle",
        "allowed": True},
    "REGRASP_DROP": {
        "source": "debug-seed bottle profile: the neck runs from z=1.007 to the "
                  "cap, so a retry 0.012 m lower is still on the neck",
        "allowed": True},
}

TABLE_MARGIN = 0.012
BAND_TOP = 0.45
MIN_PX = 200
BOTTLE_H_LO, BOTTLE_H_HI = 0.10, 0.30
CAP_BAND = 0.005
BOTTLE_CLEAR_R = 0.032
BOWL_H_LO, BOWL_H_HI = 0.030, 0.090
BOWL_FOOT_HI = 0.30
RIM_BAND = 0.006
RIM_MIN_PX = 80
RIM_R_LO, RIM_R_HI = 0.040, 0.070
RIM_RES_MAX = 0.005
NECK_DROP = 0.035
REGRASP_DROP = 0.012
BOWL_FLOOR_RISE = 0.0065
BASE_CLEARANCE = 0.006
HOVER_RISE = 0.055
CARRY_RISE = 0.10
GRIP_CLOSED = 0.0
GRIP_OPEN = 0.08
HOLD_GAP_LO, HOLD_GAP_HI = 0.006, 0.045

WS_X = (-0.55, 0.45)
WS_Y = (-0.55, 0.55)


# --------------------------------------------------------------------------
# perception

def _cloud(frame):
    z = np.asarray(frame.depth, float)
    h, w = z.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    k = np.asarray(frame.intrinsics, float)
    x = (u - k[0, 2]) * z / k[0, 0]
    y = (v - k[1, 2]) * z / k[1, 1]
    p = np.stack([x, y, z, np.ones_like(z)], -1)
    return p @ np.asarray(frame.t_base_cam, float).T


def _label(mask):
    """4-connected component labelling (scipy is not in the sandbox)."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    parent = [0]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    nxt = 1
    for i in range(h):
        for j in np.nonzero(mask[i])[0]:
            up = lab[i - 1, j] if i else 0
            left = lab[i, j - 1] if j else 0
            if up and left:
                lab[i, j] = min(up, left)
                ra, rb = find(up), find(left)
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)
            elif up or left:
                lab[i, j] = up or left
            else:
                lab[i, j] = nxt
                parent.append(nxt)
                nxt += 1
    for i in range(h):
        for j in np.nonzero(lab[i])[0]:
            lab[i, j] = find(lab[i, j])
    return lab


def _clusters(mask, x, y, z, table):
    lab = _label(mask)
    out = []
    for i in np.unique(lab):
        if i == 0:
            continue
        m = lab == i
        if int(m.sum()) < MIN_PX:
            continue
        xs, ys, zs = x[m], y[m], z[m]
        ztop = float(zs.max())
        out.append({
            "m": m, "n": int(m.sum()), "h": ztop - table, "ztop": ztop,
            "foot": float(max(xs.max() - xs.min(), ys.max() - ys.min())),
            "xc": float((xs.min() + xs.max()) / 2),
            "yc": float((ys.min() + ys.max()) / 2)})
    return out


def _kasa(x, y):
    """Least-squares circle through a partial ring -> (cx, cy, r, residual)."""
    a = np.stack([x, y, np.ones_like(x)], 1)
    c, *_ = np.linalg.lstsq(a, x ** 2 + y ** 2, rcond=None)
    cx, cy = c[0] / 2.0, c[1] / 2.0
    r = float(np.sqrt(max(c[2] + cx ** 2 + cy ** 2, 0.0)))
    res = float(np.std(np.hypot(x - cx, y - cy) - r))
    return float(cx), float(cy), r, res


def _brief(c):
    return {k: round(v, 4) for k, v in c.items() if k != "m"}


def _perceive(api):
    f = api.capture("cam_high")
    p = _cloud(f)
    x, y, z = p[..., 0], p[..., 1], p[..., 2]
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    ws = ok & (x > WS_X[0]) & (x < WS_X[1]) & (y > WS_Y[0]) & (y < WS_Y[1])
    table = float(np.median(z[ws & (z > 0.6) & (z < 0.95)]))
    band = ws & (z > table + TABLE_MARGIN) & (z < table + BAND_TOP)

    props = _clusters(band, x, y, z, table)
    for c in props:
        api.log("prop " + json.dumps(_brief(c)))
    tall = [c for c in props if BOTTLE_H_LO < c["h"] < BOTTLE_H_HI]
    if not tall:
        return table, None, None
    b = max(tall, key=lambda c: c["h"])
    cap = b["m"] & (z > b["ztop"] - CAP_BAND)
    bottle = {"x": float((x[cap].min() + x[cap].max()) / 2),
              "y": float((y[cap].min() + y[cap].max()) / 2),
              "ztop": b["ztop"], "h": b["h"], "n": b["n"],
              "cap_span": float(max(x[cap].max() - x[cap].min(),
                                    y[cap].max() - y[cap].min()))}

    # Re-mask with the bottle removed: on the fused seeds this is what gives
    # the bowl back as its own component.
    r = np.hypot(x - bottle["x"], y - bottle["y"])
    rest = _clusters(band & (r > BOTTLE_CLEAR_R), x, y, z, table)
    scored = []
    for c in rest:
        if not (BOWL_H_LO < c["h"] < BOWL_H_HI and c["foot"] < BOWL_FOOT_HI):
            continue
        ring = c["m"] & (z > c["ztop"] - RIM_BAND)
        if int(ring.sum()) < RIM_MIN_PX:
            continue
        cx, cy, rr, res = _kasa(x[ring], y[ring])
        cand = {"x": cx, "y": cy, "r": rr, "res": res, "rim": c["ztop"],
                "h": c["h"], "foot": c["foot"], "n": c["n"],
                "ring": int(ring.sum())}
        api.log("rimfit " + json.dumps({k: round(v, 4) for k, v in cand.items()}))
        scored.append(cand)
    round_ = [c for c in scored
              if RIM_R_LO < c["r"] < RIM_R_HI and c["res"] < RIM_RES_MAX]
    if round_:
        return table, bottle, min(round_, key=lambda c: c["res"])
    return table, bottle, None


# --------------------------------------------------------------------------

def run(api):
    api.log("intent=%r" % api.instruction())
    table, bottle, bowl = _perceive(api)
    api.log("table=%.4f" % table)
    if bottle is None:
        return "no bottle candidate"
    if bowl is None:
        return "no bowl candidate"
    api.log("BOTTLE " + json.dumps({k: round(v, 4) for k, v in bottle.items()}))
    api.log("BOWL " + json.dumps({k: round(v, 4) for k, v in bowl.items()}))

    # -- grasp the neck ----------------------------------------------------
    bx, by = bottle["x"], bottle["y"]
    grasp_z = bottle["ztop"] - NECK_DROP
    api.grip(GRIP_OPEN)
    r = api.move([bx, by, grasp_z + HOVER_RISE], seconds=2.0)
    api.log("hover res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))

    held = False
    for attempt in range(2):
        z_try = grasp_z - attempt * REGRASP_DROP
        r = api.move([bx, by, z_try], seconds=1.5)
        api.grip(GRIP_CLOSED)
        g = api.gripper()
        api.log("close attempt=%d z=%.4f res=%.4f eef=%s grip=%s"
                % (attempt, z_try, r, np.round(api.eef(), 4).tolist(), g))
        if HOLD_GAP_LO < g["width_m"] < HOLD_GAP_HI and g["effort"] > 1.0:
            held = True
            break
        api.grip(GRIP_OPEN)
        api.move([bx, by, grasp_z + HOVER_RISE], seconds=1.5)
    if not held:
        api.log("grasp not verified -- proceeding on the last close anyway")

    base_offset = float(api.eef()[2]) - table   # bottle stood on the table

    # -- carry -------------------------------------------------------------
    carry_z = table + base_offset + bowl["h"] + CARRY_RISE
    r = api.move([bx, by, carry_z], seconds=2.0)
    api.log("lift res=%.4f eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    r = api.move([bowl["x"], bowl["y"], carry_z], seconds=2.5)
    api.log("traverse res=%.4f eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), api.gripper()))

    # -- release -----------------------------------------------------------
    drop_z = table + BOWL_FLOOR_RISE + BASE_CLEARANCE + base_offset
    r = api.move([bowl["x"], bowl["y"], drop_z], seconds=2.0)
    api.log("lower res=%.4f eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(GRIP_OPEN)
    api.settle(1.2)
    api.log("released eef=%s grip=%s"
            % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.move([bowl["x"], bowl["y"], carry_z + 0.05], seconds=2.0)
    api.settle(1.0)
    api.log("retreat eef=%s" % np.round(api.eef(), 4).tolist())
    return "v3 held=%s base_offset=%.4f drop_z=%.4f" % (held, base_offset, drop_z)
