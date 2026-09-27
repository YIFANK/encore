"""v4 -- v3 with the grasp offset centred in its measured envelope.

Mechanism (all re-derived from the pack + debug-seed depth; see PROVENANCE):

* Fingertips ride 8 mm below api.eef() -- measured by pressing the open gripper
  onto a bare patch of table on debug seeds 51/53 (stall at table + 0.0080).
  The pack's three demos all close at eef z = table + 0.014, so the fingertips
  sit ~6 mm above the table: a DEEP straddle, one finger inside the bowl and
  one outside, pinching the wall near its base.  The pack's closed finger gaps
  (0.0125 / 0.0059 / 0.0125 m) are wall thicknesses, which is what that
  produces -- a rim-top pinch could not close that far.
* Back-projecting the bowl's pixel in each demo's t=0 keyframe onto the rim
  plane puts the demo close point ~0.05 m in +y of the bowl centre -- one rim
  radius along the finger-separation axis (tool y = base -y for the default
  straight-down wrist).  So offset the grasp by the measured rim radius in y,
  and flip to -y when that side is occupied.
* The cabinet is the one large flat plateau above table + 0.15; the bowl is the
  one circular vessel whose rim clears table + 0.035.
* Displacing the grasp offset on debug seeds 51/54/58/62 gives 4/4 at
  rim_r-0.030, rim_r-0.015, rim_r and rim_r+0.015, and 3/4 at rim_r+0.030: the
  straddle fails when the inner finger lands outside the bowl. The offset is
  therefore trimmed 8 mm toward the bowl centre, which puts the aim >=22 mm
  from either edge of the measured band instead of >=15 mm on the tight side.
* Every step is verified with my own sensors: the finger gap after the close,
  and a re-perception of the bowl's old table footprint after the lift.  A
  failed check costs one retry, which the ~1000-step horizon affords (a full
  attempt measured 160 steps).
"""
import json
from collections import deque

import numpy as np

PROVENANCE = {
    "GRASP_CLEAR": {
        "source": "pack.json demos[*].keyframes[1].ee[2] (0.9147/0.9139/0.9162) "
                  "minus the table plane measured on debug seeds 51-65 (0.9010)",
        "allowed": True},
    "PLACE_CLEAR": {
        "source": "pack.json demos[*] release-keyframe ee[2] (1.1489/1.1748/"
                  "1.1473) minus the cabinet-top plane measured on debug seeds "
                  "51-65 (1.1276): 0.020..0.047",
        "allowed": True},
    "RIM_BAND": {
        "source": "debug-seed cam_high depth: bowl rim tops out 0.049 m above "
                  "the table; plate/blue box/stove slab all sit below "
                  "table+0.035",
        "allowed": True},
    "CAB_BAND": {
        "source": "debug-seed cam_high depth: the only flat plateau above "
                  "table+0.15 is the cabinet top (1500 px at 1.1276)",
        "allowed": True},
    "ASPECT_MAX": {
        "source": "debug-seed footprints: bowl 0.109x0.109 (1.00), kettle "
                  "0.081x0.026 (3.1), bottle 0.027x0.043 (1.6)",
        "allowed": True},
    "FINGER_OFFSET": {
        "source": "debug-seed 51/53: open gripper pressed onto bare table "
                  "stalls at eef z = table + 0.0080",
        "allowed": True},
    "OPEN_W": {
        "source": "debug-seed 51/53 api.gripper() after api.grip(0.08): 0.0797",
        "allowed": True},
    "HOLD_MIN_GAP": {
        "source": "debug-seed v2 runs: a grasp that carried the bowl reported a "
                  "finger gap of 0.0108 (ep51); free-air full close reports "
                  "0.0010",
        "allowed": True},
    "OFFSET_TRIM": {
        "source": "debug-seed 51/54/58/62 envelope probes of the grasp offset: "
                  "4/4 at rim_r-0.030/-0.015/+0.000/+0.015, 3/4 at rim_r+0.030 "
                  "-> trim 0.008 inward to centre the aim",
        "allowed": True},
    "CARRY_CLEAR": {
        "source": "debug-seed depth: the cabinet top is the tallest obstacle on "
                  "the carry; 0.16 m above it clears the hanging bowl",
        "allowed": True},
    "STEP_BUDGET": {
        "source": "results.jsonl sim_steps for the unverified v2 program: 157-165 "
                  "steps per attempt; LIBERO's own horizon is the cap",
        "allowed": True},
}

GRASP_CLEAR = 0.0137
OFFSET_TRIM = 0.008
PLACE_CLEAR = 0.025
RIM_BAND = 0.035
RIM_BAND_HI = 0.12
CAB_BAND = (0.15, 0.35)
ASPECT_MAX = 1.40
CARRY_CLEAR = 0.16
OPEN_W = 0.079
HOLD_MIN_GAP = 0.006
STEP_BUDGET = 820


def L(api, **kw):
    api.log(json.dumps(kw, default=float))


def cloud(f, step=2):
    intr = np.asarray(f.intrinsics, float)
    tbc = np.asarray(f.t_base_cam, float)
    dep = np.asarray(f.depth, float)[::step, ::step]
    H, W = dep.shape
    vv, uu = np.mgrid[0:H, 0:W].astype(float) * step
    fx, fy, cx, cy = intr[0, 0], intr[1, 1], intr[0, 2], intr[1, 2]
    pc = np.stack([(uu - cx) / fx * dep, (vv - cy) / fy * dep, dep], -1)
    return pc @ tbc[:3, :3].T + tbc[:3, 3]


def components(mask, min_px):
    seen = np.zeros(mask.shape, bool)
    out = []
    for sv, su in np.argwhere(mask):
        if seen[sv, su]:
            continue
        q = deque([(sv, su)])
        seen[sv, su] = True
        pts = []
        while q:
            v, u = q.popleft()
            pts.append((v, u))
            for a, b in ((v + 1, u), (v - 1, u), (v, u + 1), (v, u - 1)):
                if (0 <= a < mask.shape[0] and 0 <= b < mask.shape[1]
                        and mask[a, b] and not seen[a, b]):
                    seen[a, b] = True
                    q.append((a, b))
        if len(pts) >= min_px:
            out.append(np.array(pts))
    return out


def box(P, pts):
    xyz = P[pts[:, 0], pts[:, 1]]
    return {"n": int(len(pts)),
            "x0": float(xyz[:, 0].min()), "x1": float(xyz[:, 0].max()),
            "y0": float(xyz[:, 1].min()), "y1": float(xyz[:, 1].max()),
            "ztop": float(np.percentile(xyz[:, 2], 98))}


def table_mask(P):
    return ((P[..., 0] > -0.25) & (P[..., 0] < 0.45) &
            (P[..., 1] > -0.50) & (P[..., 1] < 0.50))


def find_bowl(api, P, table):
    z = P[..., 2]
    m = table_mask(P) & (z > table + RIM_BAND) & (z < table + RIM_BAND_HI)
    cands, fallback = [], None
    for pts in components(m, 80):
        b = box(P, pts)
        w, d = b["x1"] - b["x0"], b["y1"] - b["y0"]
        b.update(w=w, d=d, asp=max(w / max(d, 1e-6), d / max(w, 1e-6)))
        L(api, k="bowl_cand", **b)
        if fallback is None or b["n"] > fallback["n"]:
            fallback = b
        if 0.06 < w < 0.17 and 0.06 < d < 0.17 and b["asp"] <= ASPECT_MAX:
            cands.append(b)
    best = max(cands, key=lambda b: b["n"]) if cands else fallback
    return best


def find_cabinet(api, P, table):
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    ws = (x > -0.90) & (x < 0.45) & (y > -0.80) & (y < 0.80)
    band = ws & (z > table + CAB_BAND[0]) & (z < table + CAB_BAND[1])
    if band.sum() < 100:
        return None
    edges = np.arange(table + CAB_BAND[0], table + CAB_BAND[1], 0.01)
    h, _ = np.histogram(z[band], bins=edges)
    best = None
    for j in np.argsort(h)[::-1][:3]:
        ztop = float(edges[j] + 0.005)
        m = ws & (np.abs(z - ztop) < 0.012)
        for pts in components(m, 200):
            b = box(P, pts)
            b.update(w=b["x1"] - b["x0"], d=b["y1"] - b["y0"])
            L(api, k="cab_cand", **b)
            if b["w"] < 0.10 or b["d"] < 0.10:
                continue
            if best is None or b["n"] > best["n"]:
                best = b
        if best is not None:
            break
    return best


def footprint_count(P, table, cx, cy, rad):
    z = P[..., 2]
    m = (table_mask(P) & (z > table + RIM_BAND) & (z < table + RIM_BAND_HI) &
         (np.abs(P[..., 0] - cx) < rad) & (np.abs(P[..., 1] - cy) < rad))
    return int(m.sum())


def side_blocked(P, table, bx, by, rim_r, sgn):
    """Is the outer-finger landing strip on side `sgn` occupied by something
    other than the bowl?  The outer finger lands one half-opening beyond the
    grasp point."""
    z = P[..., 2]
    yo = by + sgn * (rim_r + OPEN_W / 2.0)
    m = (table_mask(P) & (z > table + 0.012) &
         (np.abs(P[..., 0] - bx) < 0.035) & (np.abs(P[..., 1] - yo) < 0.030))
    return int(m.sum())


def state(api, tag, extra=None):
    e = [float(v) for v in api.eef()]
    g = api.gripper()
    L(api, k="state", tag=tag, eef=e, w=g["width_m"], eff=g["effort"], extra=extra)
    return np.array(e), g


class Budget:
    def __init__(self):
        self.n = 0

    def move(self, api, xyz, seconds):
        self.n += int(120 * max(seconds, 0.5))
        return api.move([float(v) for v in xyz], seconds=seconds)

    def grip(self, api, w):
        self.n += 20
        api.grip(w)


def run(api):
    B = Budget()
    f = api.capture("cam_high")
    P0 = cloud(f)
    z = P0[..., 2]
    table = float(np.median(z[table_mask(P0) & (z > 0.85) & (z < 0.95)]))
    cab = find_cabinet(api, P0, table)
    L(api, k="table", v=table)
    L(api, k="cab", v=cab)
    if cab is None:
        return "no cabinet"
    cx = 0.5 * (cab["x0"] + cab["x1"])
    cy = 0.5 * (cab["y0"] + cab["y1"])
    ctop = cab["ztop"]

    P = P0
    for attempt in range(3):
        if B.n > STEP_BUDGET:
            L(api, k="budget_stop", n=B.n, attempt=attempt)
            break
        bowl = find_bowl(api, P, table)
        L(api, k="bowl", attempt=attempt, v=bowl)
        if bowl is None:
            break
        bx = 0.5 * (bowl["x0"] + bowl["x1"])
        by = 0.5 * (bowl["y0"] + bowl["y1"])
        rim_r = 0.25 * ((bowl["x1"] - bowl["x0"]) + (bowl["y1"] - bowl["y0"]))
        rim_r = float(np.clip(rim_r, 0.035, 0.070))
        n_before = footprint_count(P, table, bx, by, rim_r + 0.02)

        blk_p = side_blocked(P, table, bx, by, rim_r, +1)
        blk_m = side_blocked(P, table, bx, by, rim_r, -1)
        sgn = 1 if blk_p <= blk_m else -1          # +y is the demos' side
        gx, gy = bx, by + sgn * (rim_r - OFFSET_TRIM)
        L(api, k="plan", attempt=attempt, bowl_xy=[bx, by], rim_r=rim_r,
          sgn=sgn, blocked=[blk_p, blk_m], grasp=[gx, gy], n_before=n_before,
          cab_xy=[cx, cy], ctop=ctop, steps=B.n)

        B.grip(api, 0.08)
        B.move(api, [gx, gy, table + 0.16], 1.6)
        r = B.move(api, [gx, gy, table + GRASP_CLEAR], 1.2)
        state(api, "down", extra={"residual": float(r), "attempt": attempt})
        B.grip(api, 0.0)
        _, g = state(api, "closed", extra={"attempt": attempt})

        B.move(api, [gx, gy, table + 0.30], 1.2)
        _, g2 = state(api, "lifted", extra={"attempt": attempt})
        f2 = api.capture("cam_high")
        P = cloud(f2)
        n_after = footprint_count(P, table, bx, by, rim_r + 0.02)
        held = (g["width_m"] > HOLD_MIN_GAP and g["effort"] >= 3.0
                and n_after < 0.5 * max(n_before, 1))
        L(api, k="grasp_check", attempt=attempt, gap=g["width_m"],
          gap_lift=g2["width_m"], n_before=n_before, n_after=n_after, held=held)
        if not held:
            B.grip(api, 0.08)
            B.move(api, [gx, gy, table + 0.30], 0.8)
            f3 = api.capture("cam_high")
            P = cloud(f3)
            continue

        px, py = cx, cy + sgn * rim_r
        r = B.move(api, [px, py, ctop + CARRY_CLEAR], 1.8)
        state(api, "over_cab", extra={"residual": float(r)})
        r = B.move(api, [px, py, ctop + PLACE_CLEAR], 1.2)
        state(api, "at_place", extra={"residual": float(r)})
        B.grip(api, 0.08)
        state(api, "released")
        B.move(api, [px, py, ctop + CARRY_CLEAR], 1.0)
        L(api, k="done", attempt=attempt, steps=B.n)
        return "v4 placed on attempt %d" % attempt
    return "v4 exhausted"
