"""v12 -- pick the plate off the table and set it on the stove slab.

Mechanism, all measured on debug seeds 51-57:
  * a commanded move shorter than the arm's 12mm position tolerance returns
    without stepping, so every precise placement overshoots (goto);
  * the OPEN gripper's fingers hang 37mm below the eef and bottom out softly on
    the table near eef z 0.9377; a saturated command ploughs past that to
    ~0.912, which puts the pads alongside a 19mm-tall dish;
  * aiming the tool at the plate centre offset by (R + 1mm) on the near side and
    closing there bites 63-68mm of the dish at effort 3.0 and holds through a
    lift (v10, seeds 51/53);
  * the plate's pose in the jaws is read off the close itself: its centre sits
    (cx-eef_x, cy-eef_y) from the tool and its base sits (eef_z - TABLE_Z)
    below the tool, because at the close it was still resting on the table.
"""
import numpy as np

PROVENANCE = {
    "OPEN_FLOOR": {"source": "v7 debug seed 51: the OPEN gripper stalls on bare "
                             "table at eef z=0.9377, twice at the same height",
                   "allowed": True},
    "TABLE_Z": {"source": "debug seeds 51/53/57/61 cam_high depth: dominant plane "
                          "of the deprojected cloud, z=0.901", "allowed": True},
    "BAND_LO": {"source": "TABLE_Z + 5mm; excludes the table plane from the "
                          "object clusters", "allowed": True},
    "FLAT_TOP_MAX": {"source": "debug-seed cluster tops: plate 0.920, bowl 0.952, "
                               "stove 0.960 -> 0.930 isolates the flat dish",
                     "allowed": True},
    "OFF_RIM": {"source": "v8/v10 debug seeds 51,53: tool aimed at plate centre + "
                          "(R+0.001) on the near side closed on a 0.063-0.068 "
                          "bite with effort 3.0", "allowed": True},
    "GRIP_MIN": {"source": "v3/v4/v5 debug seeds: an unobstructed close reports "
                           "gap 0.0018, so 0.005 separates bite from air",
                 "allowed": True},
    "MIN_CMD": {"source": "v9 debug seeds: 7mm commands did not step the arm at "
                          "all; 17mm is safely past the tolerance", "allowed": True},
    "CARRY_Z": {"source": "v10/v11 debug seeds: the plate clears the table at eef "
                          "z 0.99 and the tallest thing on the carry line is the "
                          "stove knob at 0.960", "allowed": True},
    "DROP_CLEAR": {"source": "v11 debug seeds: 8mm of clearance released cleanly; "
                             "15mm keeps the dish off the slab until it is let go",
                   "allowed": True},
    "ON_STOVE_TOL": {"source": "debug-seed slab footprint 0.29 x 0.19, so a plate "
                               "centre within 0.10 of the slab centre is on it",
                     "allowed": True},
    "PLUNGE": {"source": "v12 debug seeds: a close at eef z 0.912-0.918 bit "
                         "63-68mm of dish, one at 0.921-0.922 bit 29mm or "
                         "nothing; commanding 45mm below the soft floor forces "
                         "the deeper of the two", "allowed": True},
    "ATTEMPTS": {"source": "v14 selection run on debug seeds 51-65: 14/15 bit on "
                           "the first near-side approach; seed 52 missed three "
                           "times from that side (each miss shoving the dish +x) "
                           "and only the +y side bit. v15 measured the +y bite at "
                           "0.025 against 0.060 near-side, so it is the third "
                           "choice, not the second", "allowed": True},
}

OPEN_FLOOR = 0.9377
TABLE_Z = 0.901
BAND_LO = 0.906
FLAT_TOP_MAX = 0.930
OFF_RIM = 0.001
GRIP_MIN = 0.005
MIN_CMD = 0.017
CARRY_Z = 0.995
DROP_CLEAR = 0.015
ON_STOVE_TOL = 0.10
PLUNGE = 0.045
ATTEMPTS = (((1.0, 0.0), 0.001),      # near (-x) side: the v13/v14 grip
            ((1.0, 0.0), 0.010),      # same side, further out
            ((0.0, -1.0), 0.001),     # +y side: different rim phase, weaker bite
            ((1.0, 0.0), -0.008))

STEP = 2
XLIM = (-0.55, 0.45)
YLIM = 0.60


def _cloud(f, step=STEP):
    d = np.asarray(f.depth, float)[::step, ::step]
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = d.shape
    ii, jj = np.mgrid[0:H, 0:W]
    x = (jj * step - K[0, 2]) * d / K[0, 0]
    y = (ii * step - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1)
    return (P @ T.T)[..., :3], np.asarray(f.rgb)[::step, ::step, :]


def _label(mask, min_px=60):
    H, W = mask.shape
    lab = -np.ones(mask.shape, np.int32)
    cur = 0
    for s in np.argwhere(mask):
        a0, b0 = int(s[0]), int(s[1])
        if lab[a0, b0] >= 0:
            continue
        stack = [(a0, b0)]
        lab[a0, b0] = cur
        comp = []
        while stack:
            a, b = stack.pop()
            comp.append((a, b))
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    na, nb = a + da, b + db
                    if 0 <= na < H and 0 <= nb < W and mask[na, nb] and lab[na, nb] < 0:
                        lab[na, nb] = cur
                        stack.append((na, nb))
        if len(comp) < min_px:
            for a, b in comp:
                lab[a, b] = -1
        else:
            cur += 1
    return lab, cur


def scene(api, tag=""):
    f = api.capture("cam_high")
    B, rgb = _cloud(f)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    m = (np.isfinite(Z) & (Z > BAND_LO) & (X > XLIM[0]) & (X < XLIM[1])
         & (np.abs(Y) < YLIM))
    lab, n = _label(m)
    out = []
    for c in range(n):
        s = lab == c
        pts = B[s]
        info = {"n": int(s.sum()), "ztop": float(pts[:, 2].max()), "pts": pts,
                "xr": (float(pts[:, 0].min()), float(pts[:, 0].max())),
                "yr": (float(pts[:, 1].min()), float(pts[:, 1].max()))}
        info["dx"] = info["xr"][1] - info["xr"][0]
        info["dy"] = info["yr"][1] - info["yr"][0]
        info["cx"] = 0.5 * (info["xr"][0] + info["xr"][1])
        info["cy"] = 0.5 * (info["yr"][0] + info["yr"][1])
        out.append(info)
        api.log("CLU%s n=%d ztop=%.3f x[%.3f,%.3f] y[%.3f,%.3f]"
                % (tag, info["n"], info["ztop"], info["xr"][0], info["xr"][1],
                   info["yr"][0], info["yr"][1]))
    return out


def pick_plate(comps):
    cand = [c for c in comps if c["ztop"] < FLAT_TOP_MAX
            and 0.08 < c["dx"] < 0.22 and 0.08 < c["dy"] < 0.22]
    return max(cand, key=lambda c: c["dx"] + c["dy"]) if cand else None


def pick_stove(comps, plate):
    cand = [c for c in comps if c is not plate and 0.93 < c["ztop"] < 1.00
            and c["dx"] > 0.15 and c["dy"] > 0.10]
    return max(cand, key=lambda c: c["n"]) if cand else None


def slab_top(api, stove):
    p = stove["pts"]
    lo = np.percentile(p[:, 2], 55)
    s = (p[:, 2] >= lo) & (p[:, 2] <= lo + 0.008)
    tx, ty, tz = float(p[s, 0].mean()), float(p[s, 1].mean()), float(np.median(p[s, 2]))
    api.log("SLAB z=%.4f cen=(%.4f,%.4f) n=%d" % (tz, tx, ty, int(s.sum())))
    return tx, ty, tz


def goto(api, target, tol=0.004, tries=3, seconds=0.8):
    """Drive to `target` despite the 12mm move tolerance, by overshooting."""
    t = np.asarray(target, float)
    for _ in range(tries):
        e = api.eef()
        err = t - e
        n = float(np.linalg.norm(err))
        if n < tol:
            break
        k = max(2.0, MIN_CMD / max(n, 1e-6))
        api.move(t + err * (k - 1.0), seconds=seconds)
    return float(np.linalg.norm(t - api.eef()))


def cycle(api, plate, slab, tag, approach=(1.0, 0.0), extra=OFF_RIM):
    """One pick-and-place. Returns True only if the dish was released over the
    slab still in the jaws.

    `approach` is the unit vector from the tool towards the dish centre, so the
    jaws are parked `off` away on the opposite side.
    """
    sx, sy, sz = slab
    cx, cy = plate["cx"], plate["cy"]
    R = 0.25 * (plate["dx"] + plate["dy"])
    off = R + extra
    ux, uy = approach
    ax, ay = cx - ux * off, cy - uy * off

    # Plough straight down. The open jaws meet the table softly near eef
    # z 0.9377 and a saturated command drives them past it; the bite depends on
    # reaching ~0.912 or lower (v12: 0.912-0.918 bit 63-68mm, 0.921-0.922 bit
    # 29mm or nothing), so command well below and press twice.
    api.grip(0.08)
    api.move([ax, ay, OPEN_FLOOR + 0.072], seconds=1.0)
    api.move([ax, ay, OPEN_FLOOR - PLUNGE], seconds=1.0)
    api.move([ax, ay, OPEN_FLOOR - PLUNGE], seconds=0.6)
    api.log("%s AT u=(%.0f,%.0f) eef=%s"
            % (tag, ux, uy, np.round(api.eef(), 4).tolist()))
    api.grip(0.0)
    g = api.gripper()
    ec = api.eef()
    api.log("%s CLOSE gap=%.4f effort=%.2f eef=%s off_cmd=%.4f off_got=%.4f"
            % (tag, g["width_m"], g["effort"], np.round(ec, 4).tolist(), off,
               float(np.hypot(cx - float(ec[0]), cy - float(ec[1])))))
    if g["width_m"] < GRIP_MIN:
        api.grip(0.08)
        api.move([float(ec[0]), float(ec[1]), CARRY_Z], seconds=1.0)
        return False

    # the dish is still on the table at this instant, so its pose in the jaws
    # is exactly the offset between the tool and what cam_high measured.
    dx, dy = cx - float(ec[0]), cy - float(ec[1])
    ddown = float(ec[2]) - TABLE_Z
    api.log("%s HOLD dx=%.4f dy=%.4f ddown=%.4f" % (tag, dx, dy, ddown))

    # Every metre of motion bleeds the bite (the dish ratchets out of a wedge
    # pinch), so the carry is as few moves as it can be: two lifts, one diagonal.
    z = float(ec[2])
    for i in range(2):
        z += 0.043
        e = api.eef()
        api.move([float(e[0]), float(e[1]), z], seconds=0.6)
        g = api.gripper()
        api.log("%s RAISE%d eef=%s gap=%.4f effort=%.2f"
                % (tag, i, np.round(api.eef(), 4).tolist(), g["width_m"], g["effort"]))
        if g["width_m"] < GRIP_MIN:
            api.grip(0.08)
            e = api.eef()
            api.move([float(e[0]), float(e[1]), CARRY_Z], seconds=0.8)
            return False

    tx, ty = sx - dx, sy - dy
    api.move([tx, ty, CARRY_Z], seconds=1.5)
    g = api.gripper()
    api.log("%s OVER eef=%s gap=%.4f" % (tag, np.round(api.eef(), 4).tolist(),
                                         g["width_m"]))
    if g["width_m"] < GRIP_MIN:
        api.log("%s lost the dish on the carry" % tag)
        api.grip(0.08)
        return False
    goto(api, [tx, ty, sz + ddown + DROP_CLEAR], tries=2, seconds=0.8)
    g = api.gripper()
    api.log("%s DOWN eef=%s gap=%.4f effort=%.2f"
            % (tag, np.round(api.eef(), 4).tolist(), g["width_m"], g["effort"]))
    still_held = g["width_m"] >= GRIP_MIN
    api.grip(0.08)
    api.settle(0.4)
    e = api.eef()
    api.move([float(e[0]), float(e[1]), CARRY_Z + 0.04], seconds=1.0)
    api.settle(0.5)
    return still_held


def released_over_slab(api, slab, comps):
    """The dish is on the slab if no table-height dish is left anywhere off it."""
    p = pick_plate(comps)
    if p is None:
        return True, None
    d = float(np.hypot(p["cx"] - slab[0], p["cy"] - slab[1]))
    return (d < ON_STOVE_TOL and p["ztop"] > slab[2]), p


def run(api):
    api.log("INSTRUCTION %r" % api.instruction())
    comps = scene(api)
    plate = pick_plate(comps)
    stove = pick_stove(comps, plate)
    if plate is None or stove is None:
        return "perception failed"
    api.log("PLATE cen=(%.4f,%.4f) d=(%.4f,%.4f)"
            % (plate["cx"], plate["cy"], plate["dx"], plate["dy"]))
    slab = slab_top(api, stove)

    for k, (approach, extra) in enumerate(ATTEMPTS):
        carried = cycle(api, plate, slab, "C%d" % k, approach, extra)
        comps = scene(api, "-v%d" % k)
        ok, p = released_over_slab(api, slab, comps)
        api.log("C%d VERDICT carried=%s on_slab=%s plate=%s"
                % (k, carried, ok,
                   None if p is None else (round(p["cx"], 4), round(p["cy"], 4),
                                           round(p["ztop"], 4))))
        if carried and ok:
            return "placed"
        # A failed close leaves the arm parked clear of the dish, so a miss here
        # means the dish is hidden or fused, not gone: keep the last estimate.
        if p is not None:
            plate = p
        else:
            api.log("C%d re-perception lost the dish; reusing the last estimate" % k)
    return "not placed"
