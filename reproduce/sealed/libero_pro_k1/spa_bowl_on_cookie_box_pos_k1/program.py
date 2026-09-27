"""v3 -- rim-pinch the elevated bowl, carry it, seat it on the goal disc.

Scene reading (all re-derived from this cell's debug-seed RGB-D):
  * The working surface is the modal height of the deprojected cloud.
  * Objects are height-continuity components above it. Two deep ~0.11 m discs
    (the bowls), one shallow bright disc (the plate), one small dark box, one
    wide dark slab, the cabinet, the arm.
  * "the black bowl ON the cookie box" = of the two deep discs, the one whose
    BASE sits higher (measured: 0.0235 m up, versus the other resting on the
    surface).
  * The goal disc's height is read per episode -- in this band it happens to
    stand on the cabinet, but nothing here assumes that.

Grasp: outer diameter 0.110 m exceeds the 0.0778 m jaw span, so the only hold
is a pinch across the rim wall. v2 showed the pinch depth is what decides
whether it survives the carry: a pinch 11 mm below the rim slipped in transit
(3/8 episodes), one 16-26 mm below did not. The controller stops within its
own 12 mm position tolerance, measured 6-11 mm short on a descent, so the
commanded depth carries that bias as an explicit term.

Hold geometry is computed, not re-perceived: v2's attempt to re-find the bowl
in the air locked onto the arm (which surrounds it in the depth image) and
released 5 cm off-centre. After the jaws self-centre on the wall, the wall sits
at the tool's xy, so the bowl centre is one wall radius away along the pinch
direction -- and that radius is measured off the bowl's own profile at the
height the jaws actually reached.
"""
from collections import deque

import numpy as np

PROVENANCE = {
    "RIM_BAND_M": {
        "source": "debug seeds 51-65: the target disc's top 6 mm of points form "
                  "a ring whose bbox midpoint is stable to <1 mm across seeds",
        "allowed": True},
    "SEG_DZ_M": {
        "source": "debug seeds 51-65: a 0.008 m height-continuity link splits "
                  "the goal disc from the surface under it in 15/15 scenes",
        "allowed": True},
    "DEEP_DISC_RANGE_M": {
        "source": "measured on debug seeds: bowl footprint 0.110 x 0.111 m and "
                  "0.045 m deep; the ramekin is 0.087 m; the goal disc is "
                  "0.12-0.14 m across with a 0.013 m rise",
        "allowed": True},
    "FLAT_DISC_RANGE_M": {
        "source": "measured on debug seeds: goal disc 0.120-0.137 m across, "
                  "0.013 m rise, mean grey 131; the dark slab is 0.19-0.26 m "
                  "across at grey 62-68",
        "allowed": True},
    "GRASP_DEPTH_LADDER_M": {
        "source": "v2/v3 receipts on debug seeds: a pinch reaching 11 mm below "
                  "the rim slipped in transit, 27 mm fell out of the jaws on "
                  "the lift, 16-20 mm held; the pack demo closes 18 mm below "
                  "the same bowl's measured rim. Commanded depths 0.021-0.031 "
                  "land in that window once the tolerance bias is added",
        "allowed": True},
    "MOVE_BIAS_M": {
        "source": "v2 and v3 receipts: every descent stopped 6-11 mm above the "
                  "commanded z (the runner's own 12 mm position tolerance)",
        "allowed": True},
    "GENTLE_LIFT_M": {
        "source": "v2 receipts: bowls that survived the close were lost on the "
                  "saturated lift; 0.02 m is the largest first step that keeps "
                  "the controller action below saturation",
        "allowed": True},
    "PINCH_INSET_LADDER_M": {
        "source": "v2/v3 receipts: aiming 2-6 mm inside the measured outer "
                  "wall radius at the grasp height put the wall between the "
                  "jaws on every attempt (closed gap 0.007-0.009 m)",
        "allowed": True},
    "HELD_GAP_MIN_M": {
        "source": "debug-seed api.gripper(): a close onto the wall leaves "
                  "0.007-0.009 m; a slip collapses it to <=0.005 m and effort "
                  "falls from 3.0 to 0.05",
        "allowed": True},
    "CARRY_CLEAR_M": {
        "source": "debug seeds: goal-disc rim measures 1.148 m; carrying 0.09 m "
                  "above the higher of rim/goal clears it with the bowl hanging",
        "allowed": True},
    "PLACE_CLEAR_M": {
        "source": "v2 receipts: releases leaving the bowl's measured base "
                  "8-13 mm above the measured goal floor seated it",
        "allowed": True},
    "REACQUIRE_R_M": {
        "source": "debug seeds: the two deep discs are never closer than "
                  "0.15 m, so 0.15 m re-identifies the target after a slip",
        "allowed": True},
}

RIM_BAND_M = 0.006
SEG_DZ_M = 0.008
MOVE_BIAS_M = 0.007
HELD_GAP_MIN_M = 0.0055
CARRY_CLEAR_M = 0.09
PLACE_CLEAR_M = 0.005
REACQUIRE_R_M = 0.15
OPEN_M = 0.08
SHUT_M = 0.0


# ---------------------------------------------------------------------------
# perception

def _xyz(frame):
    dep = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = dep.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    with np.errstate(all="ignore"):
        X = (uu - K[0, 2]) * dep / K[0, 0]
        Y = (vv - K[1, 2]) * dep / K[1, 1]
        P = np.stack([X, Y, dep, np.ones_like(dep)], -1)
        W = P @ T.T
    return W[..., :3]


def _segment(mask, Z, dz=SEG_DZ_M, min_px=150):
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    keep = []
    for sy, sx in np.argwhere(mask):
        if lab[sy, sx]:
            continue
        cur += 1
        q = deque([(sy, sx)])
        lab[sy, sx] = cur
        npx = 0
        while q:
            y, x = q.popleft()
            npx += 1
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if (0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not lab[ny, nx]
                        and abs(Z[ny, nx] - Z[y, x]) < dz):
                    lab[ny, nx] = cur
                    q.append((ny, nx))
        if npx >= min_px:
            keep.append(cur)
    return lab, keep


def _scene(api):
    f = api.capture("cam_high")
    W = _xyz(f)
    rgb = np.asarray(f.rgb).astype(float)
    Z = W[..., 2]
    ok = np.isfinite(Z) & np.isfinite(W[..., 0]) & np.isfinite(W[..., 1])
    ws = (ok & (W[..., 0] > -0.45) & (W[..., 0] < 0.50)
          & (W[..., 1] > -0.50) & (W[..., 1] < 0.50) & (Z < 1.32))
    hist, edges = np.histogram(Z[ws], bins=np.arange(0.60, 1.40, 0.005))
    surface = float(edges[int(np.argmax(hist))] + 0.0025)
    lab, ids = _segment(ws & (Z > surface + 0.004), Z)
    out = []
    for i in ids:
        m = lab == i
        p = W[m]
        ring = p[p[:, 2] > p[:, 2].max() - RIM_BAND_M]
        out.append({
            "n": int(m.sum()),
            "zmin": float(p[:, 2].min()), "zmax": float(p[:, 2].max()),
            "dx": float(np.ptp(p[:, 0])), "dy": float(np.ptp(p[:, 1])),
            "cx": float((ring[:, 0].min() + ring[:, 0].max()) / 2.0),
            "cy": float((ring[:, 1].min() + ring[:, 1].max()) / 2.0),
            "rx": float(np.ptp(ring[:, 0]) / 2.0),
            "ry": float(np.ptp(ring[:, 1]) / 2.0),
            "grey": float(rgb[m].mean()),
            "pts": p,
        })
    return surface, out


def _deep_discs(blobs):
    return [b for b in blobs
            if 0.098 <= min(b["dx"], b["dy"]) and max(b["dx"], b["dy"]) <= 0.128
            and (b["zmax"] - b["zmin"]) >= 0.035]


def _pick_goal(blobs):
    flat = [b for b in blobs
            if 0.095 <= min(b["dx"], b["dy"]) and max(b["dx"], b["dy"]) <= 0.175
            and (b["zmax"] - b["zmin"]) <= 0.030 and b["grey"] > 100.0
            and b["n"] > 1200]
    return max(flat, key=lambda b: b["n"]) if flat else None


def _goal_floor(goal):
    p = goal["pts"]
    r = np.hypot(p[:, 0] - goal["cx"], p[:, 1] - goal["cy"])
    inner = p[r < 0.35 * max(goal["rx"], goal["ry"])]
    if inner.shape[0] < 30:
        return float(goal["zmax"] - 0.010)
    return float(np.median(inner[:, 2]))


def _wall_radius(bowl, z):
    """Outer radius of the bowl wall at height z, off the bowl's own profile."""
    p = bowl["pts"]
    r = np.hypot(p[:, 0] - bowl["cx"], p[:, 1] - bowl["cy"])
    for half in (0.004, 0.007, 0.012):
        sel = np.abs(p[:, 2] - z) < half
        if int(sel.sum()) >= 40:
            return float(np.percentile(r[sel], 90))
    return float(max(bowl["rx"], bowl["ry"]) * 0.88)


# ---------------------------------------------------------------------------

def _holding(api):
    g = api.gripper()
    return (g["width_m"] > HELD_GAP_MIN_M and g["effort"] > 1.0), g


def _blob_line(tag, b):
    return (f"{tag} n={b['n']} c=({b['cx']:+.4f},{b['cy']:+.4f}) "
            f"z[{b['zmin']:.4f}..{b['zmax']:.4f}] d=({b['dx']:.3f},{b['dy']:.3f}) "
            f"grey={b['grey']:.0f}")


def run(api):
    api.log(f"instr={api.instruction()!r}")
    surface, blobs = _scene(api)
    api.log(f"surface={surface:.4f} nblobs={len(blobs)}")
    for b in blobs:
        api.log(_blob_line("  blob", b))

    deep = _deep_discs(blobs)
    goal = _pick_goal(blobs)
    if not deep or goal is None:
        api.log(f"PERCEPTION FAILED ndeep={len(deep)} goal={goal is not None}")
        return "no target/goal"
    tgt = max(deep, key=lambda b: b["zmin"])
    anchor = (tgt["cx"], tgt["cy"])
    floor = _goal_floor(goal)
    api.log(_blob_line("TARGET", tgt))
    api.log(_blob_line("GOAL", goal) + f" floor={floor:.4f}")

    carry_z = max(goal["zmax"], tgt["zmax"]) + CARRY_CLEAR_M
    note = "no attempt run"
    held = False
    z_grasp = gap = 0.0
    gx = gy = 0.0

    # The environment allows 500 control steps per episode and a grip command
    # costs 20 of them, so the ladder is three attempts, not nine.
    for attempt, (cmd_depth, inset) in enumerate(
            ((0.026, 0.004), (0.021, 0.002), (0.031, 0.006))):
        if attempt:
            surface, blobs = _scene(api)
            near = [b for b in _deep_discs(blobs)
                    if np.hypot(b["cx"] - anchor[0], b["cy"] - anchor[1]) < REACQUIRE_R_M]
            if not near:
                api.log("REACQUIRE lost the bowl")
                break
            tgt = min(near, key=lambda b:
                      np.hypot(b["cx"] - anchor[0], b["cy"] - anchor[1]))
            anchor = (tgt["cx"], tgt["cy"])
            api.log(_blob_line(f"a{attempt} REACQUIRED", tgt))

        rim = tgt["zmax"]
        cmd_z = rim - cmd_depth
        r_wall = _wall_radius(tgt, cmd_z + MOVE_BIAS_M)
        pr = r_wall - inset
        gx, gy = tgt["cx"], tgt["cy"] + pr
        api.grip(OPEN_M)
        api.move([gx, gy, rim + 0.09], seconds=3.0)
        r2 = api.move([gx, gy, cmd_z], seconds=2.5)
        e2 = api.eef()
        api.log(f"a{attempt} pr={pr:.4f} r_wall={r_wall:.4f} cmd_z={cmd_z:.4f} "
                f"res={r2:.4f} eef={np.round(e2, 4).tolist()}")
        api.grip(SHUT_M)
        api.settle(0.2)
        _, g0 = _holding(api)
        # Break the bowl free gently before the full lift: a saturated jerk
        # straight off the box is what shook it out of the jaws in v2.
        api.move([gx, gy, float(e2[2]) + 0.02], seconds=1.0)
        api.settle(0.2)
        api.move([gx, gy, rim + 0.14], seconds=2.0)
        api.settle(0.2)
        held, g1 = _holding(api)
        api.log(f"a{attempt} close gap={g0['width_m']:.4f} -> lift "
                f"gap={g1['width_m']:.4f} eff={g1['effort']:.2f} held={held}")
        if held:
            z_grasp = float(e2[2])
            gap = float(g1["width_m"])
            break
        note = "grasp failed"
        api.grip(OPEN_M)
        api.move([gx, gy, rim + 0.14], seconds=2.0)

    if held:
        # Hold geometry: the jaws self-centred on the wall, so the wall is at
        # the tool xy and the bowl centre is one wall radius back along +y.
        r_hold = _wall_radius(tgt, z_grasp) - gap / 2.0
        drop = z_grasp - tgt["zmin"]
        rel_x = goal["cx"]
        rel_y = goal["cy"] + r_hold
        rel_z = floor + drop + PLACE_CLEAR_M
        api.log(f"z_grasp={z_grasp:.4f} gap={gap:.4f} r_hold={r_hold:.4f} "
                f"drop={drop:.4f} release=({rel_x:+.4f},{rel_y:+.4f},{rel_z:.4f})")

        api.move([gx, gy, carry_z], seconds=2.5)
        ok, g = _holding(api)
        api.log(f"at carry gap={g['width_m']:.4f} eff={g['effort']:.2f}")
        api.move([rel_x, rel_y, carry_z], seconds=3.5)
        ok, g = _holding(api)
        api.log(f"over goal gap={g['width_m']:.4f} eff={g['effort']:.2f} "
                f"eef={np.round(api.eef(), 4).tolist()}")
        r = api.move([rel_x, rel_y, rel_z], seconds=2.5)
        api.log(f"descend res={r:.4f} eef={np.round(api.eef(), 4).tolist()}")
        api.grip(OPEN_M)
        api.settle(0.5)
        api.move([rel_x, rel_y, carry_z], seconds=2.5)
        api.settle(0.4)
        note = "placed"

    _, final = _scene(api)
    for b in _deep_discs(final):
        api.log(_blob_line("FINAL", b))
    api.log(f"note={note}")
    return note
