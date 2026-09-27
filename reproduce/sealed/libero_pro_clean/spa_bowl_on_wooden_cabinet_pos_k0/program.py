"""v13 -- rim pinch; the jaws are CENTRED on the bowl before they close.

v7 scored 4/8 and taught two things:

  * the carry route is right but its clearance was wrong. Dropping to
    mid-height at y=-0.045 leaves the HELD BOWL (outer radius 0.056, centre
    trailing the eef by ~0.042) overlapping the cabinet's +y face, and its
    widest part is then below the cabinet top -- so the bowl is driven into
    the cabinet's side and x freezes. That is exactly the wedge the ep61 gif
    shows. v8 computes the clearance from the perceived cabinet instead:
    the eef stays at cab_ymax + OFF_Y + BOWL_R + 0.020, and the traverse
    holds that y until x is past the cabinet before turning onto the plate.
v8-v12 all scored 6/8, always losing the same two seeds, and two diagnostic
probes finally located the cause -- it is the GRASP, not the carry.

  probe_reach: with an EMPTY gripper, seeds 61/63 retract -x from the pregrasp
  pose exactly as far as seed 55 does (0.234 m, every hop tracked). So there is
  no kinematic barrier and no reach limit.

  probe_bowl: holding the bowl, seed 61 retracts only 0.100 m before the eef
  stalls in x and starts being dragged -y and +z, and its gripper gap is
  0.0050 and falling. Seed 55, same manoeuvre, retracts 0.290 m with a gap of
  0.0085 and holding.

The gap is the tell, and it tracks one thing -- how well the jaws were centred
on the bowl when they closed:

    seed   |eef_x - bcx| at close   closed gap
    55/57/51        0.003            0.0085-0.0091
    61/63           0.013            0.0052-0.0055

The jaws close along y, so they only cut a diameter when eef_x equals the
bowl's centre x; 13 mm off makes the bite a CHORD that catches a thin sliver of
rim. descend_to was converging z only -- the bite depth -- and let x drift by
up to 14 mm during the descent (pregrasp 0.0395 -> at_grasp 0.0538 on seed 61).
That thin bite is what fails later: the bowl hangs badly, drags, and stalls the
carry. v13 converges the descent in x and y as well as z.

Superseded reasoning (v9/v10/v11 all scored 6/8 and between them pin the blocker on seeds 61/63 down
to arm POSTURE, not reach, not budget, not saturation:
  * v9 walked the retract at z=1.086: hard stop at x=0.0615.
  * v10 walked it at z=1.27: reached x=0.0024 while the eef CLIMBED to 1.2889.
  * v11 retraced the inbound diagonal: same stop, x=0.0027, z climbed to 1.2889.
Every attempt stalls with the eef RISING -- the controller is pushing -x and
the arm converts it into +z, which is what a kinematic boundary looks like: at
that height the arm simply cannot retract further. The episode's own start pose
is the counter-example, x=-0.2085 at y=0 but at z=1.1733, well below.
So v12 gets LOW first: descend at a y clear of the cabinet down to a carry
height just above the plate, and retract from there, where the arm sits in the
same unfolded posture it started the episode in.

Background (v8 scored 6/8). The two survivors (61,63) are the seeds whose bowl sits at the
far +x corner of the cabinet, leaving the arm extended: the carry then asks for
a single 0.33 m retract in -x, and the gif shows the arm folded with x frozen
while the command saturates. A saturated far target buys force on every axis at
once and drives the arm into a configuration it cannot leave; the same retract
taken in 0.05 m hops keeps the controller in its well-behaved regime. v9 walks
the traverse and logs each hop, so a genuine reach limit would still be visible
as the hop where progress stops.

Inherited from v8:
  * seeds 51/59 failed on BUDGET, not mechanics -- three grasp attempts plus
    the carry exceeded the 1000-step horizon with nothing left for the place.
    Waypoints do not need the 4 mm accuracy the grasp needs, and every extra
    bias-cancel pass costs up to a full move cap, so waypoint moves now run at
    a 0.02 tolerance and only grasp/place poses are converged tightly.

The carry offset is no longer measured at runtime (v6/v7's ring fit on the held
bowl was unreliable: the hand body outranks the rim in a top-down max-z map).
It is the constant v6 measured on the seeds where that fit did succeed.
"""
import numpy as np

PROVENANCE = {
    "TIP_OFFSET": {"source": "debug seeds 51,53,55,57 (v2): closed jaws driven "
                             "into the bare table stall 0.0091 above it",
                   "allowed": True},
    "PINCH_R": {"source": "debug-seed radial profile: bowl outer radius ~0.056, "
                          "Kasa rim ring r=0.0527; the eef sits 0.055 out along "
                          "y so the 0.0792 jaws straddle the wall",
                "allowed": True},
    "OFF_Y": {"source": "debug seeds 51/55/59 (v6 held-bowl ring fit): the held "
                        "bowl's centre trails the eef by 0.0383-0.0438, mean "
                        "0.042; off_x measured 0.000 as expected for jaws that "
                        "open along y",
              "allowed": True},
    "BOWL_R": {"source": "debug-seed radial profile: max-z falls back to the "
                         "cabinet top beyond r=0.056",
               "allowed": True},
    "PINCH_DEPTHS": {"source": "debug-seed radial profile (rim top 1.1799 over "
                               "an interior floor at 1.136) and the v5-v7 "
                               "ladder: 0.022 held first-try on 5/8 seeds",
                     "allowed": True},
    "OPEN_W": {"source": "debug-seed v2: api.grip(0.08) settles at 0.0792",
               "allowed": True},
    "GAP_BAND": {"source": "debug seeds v3-v7: grips that survived the lift "
                           "closed to 0.0052-0.0091; the one that slipped fell "
                           "to 0.0048",
                 "allowed": True},
    "BOWL_H": {"source": "debug seeds 51-65: rim top 1.1799 - cabinet top "
                         "1.1273 = 0.0526",
               "allowed": True},
    "WALK_STEP": {"source": "generic controller mechanics: the action is "
                            "clip(err/0.05), so hops of 0.05 keep the command "
                            "off its saturation limit",
                  "allowed": True},
    "CARRY_LOW_Z": {"source": "debug seeds 51-65: plate top 0.9508 and slab "
                              "0.951, so a bowl hanging 0.040 below the "
                              "fingertips clears them at eef z 1.03",
                    "allowed": True},
    "MID_Z": {"source": "debug seeds 51-65: below the cabinet top (1.1273) so "
                        "the arm folds back comfortably, above the slab (0.951) "
                        "with the bowl hanging",
              "allowed": True},
    "BIAS_MAX": {"source": "generic controller mechanics: POS_TOL is 0.012, so "
                           "a residual past ~0.030 is a starved move, not bias",
                 "allowed": True},
    "SEC_*": {"source": "generic controller mechanics: move_cartesian caps at "
                        "max(40,120*seconds) steps against a 1000-step horizon",
              "allowed": True},
}

TIP_OFFSET = 0.0091
PINCH_R = 0.055
OFF_Y = 0.042
BOWL_R = 0.056
PINCH_DEPTHS = (0.022, 0.016, 0.028)
OPEN_W = 0.0792
GAP_BAND = (0.0045, 0.020)
BOWL_H = 0.0526
MID_Z = 1.06
CARRY_LOW_Z = 1.03
DESC_STEP = 0.06
BIAS_MAX = 0.030
SEC_FAR = 1.2      # cap 144 steps
SEC_NEAR = 0.5     # cap 60 steps
TOL_WP = 0.020
TOL_TIGHT = 0.005
WALK_STEP = 0.05


def _log(api, **kw):
    api.log(" ".join(f"{k}={v}" for k, v in kw.items()))


# --------------------------------------------------------------------------

def _cloud(api, cam="cam_high"):
    f = api.capture(cam)
    K, T = np.asarray(f.intrinsics), np.asarray(f.t_base_cam)
    d = np.asarray(f.depth, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    z = np.where(np.isfinite(d) & (d > 0), d, np.nan)
    c = np.stack([(uu - K[0, 2]) * z / K[0, 0],
                  (vv - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], -1)
    P = c @ T.T
    return P[..., 0], P[..., 1], P[..., 2]


def _comps(mask, minc):
    lab = np.zeros(mask.shape, bool)
    out = []
    for a in range(mask.shape[0]):
        for b in range(mask.shape[1]):
            if mask[a, b] and not lab[a, b]:
                stack, cells = [(a, b)], []
                lab[a, b] = True
                while stack:
                    p, q = stack.pop()
                    cells.append((p, q))
                    for dp, dq in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        r, s = p + dp, q + dq
                        if (0 <= r < mask.shape[0] and 0 <= s < mask.shape[1]
                                and mask[r, s] and not lab[r, s]):
                            lab[r, s] = True
                            stack.append((r, s))
                if len(cells) >= minc:
                    out.append(cells)
    return out


def _kasa(x, y):
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    sol, *_ = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)
    cx, cy = float(sol[0]), float(sol[1])
    r = float(np.sqrt(max(sol[2] + cx ** 2 + cy ** 2, 1e-9)))
    return cx, cy, r, float(np.abs(np.hypot(x - cx, y - cy) - r).mean())


def perceive(api):
    X, Y, Z = _cloud(api)
    xlo, xhi, ylo, yhi, cell = -0.55, 0.35, -0.60, 0.60, 0.006
    m = (np.isfinite(Z) & (X > xlo) & (X < xhi) & (Y > ylo) & (Y < yhi)
         & (Z > 0.84) & (Z < 1.50))
    nx, ny = int((xhi - xlo) / cell), int((yhi - ylo) / cell)
    H = np.full((nx, ny), np.nan)
    ix = np.clip(((X[m] - xlo) / cell).astype(int), 0, nx - 1)
    iy = np.clip(((Y[m] - ylo) / cell).astype(int), 0, ny - 1)
    zz = Z[m]
    for i in np.argsort(zz):
        H[ix[i], iy[i]] = zz[i]
    fin = np.isfinite(H)
    hist, edges = np.histogram(H[fin], bins=200)
    table_z = float(edges[hist.argmax()])

    def desc(cells):
        xs = np.array([xlo + (a + 0.5) * cell for a, b in cells])
        ys = np.array([ylo + (b + 0.5) * cell for a, b in cells])
        zs = np.array([H[a, b] for a, b in cells])
        return dict(n=len(cells), xs=xs, ys=ys, zs=zs, xmin=xs.min(),
                    xmax=xs.max(), ymin=ys.min(), ymax=ys.max(),
                    x=xs.mean(), y=ys.mean(), ztop=zs.max(),
                    zmed=float(np.median(zs)))

    # the cabinet: the tallest fixture. The arm shares this band and hangs over
    # the midline, so drop any component straddling y=0.
    tall = [desc(c) for c in _comps(fin & (H > table_z + 0.15), 40)]
    tall = [c for c in tall if not (c["ymin"] < 0.02 and c["ymax"] > -0.02)]
    tall.sort(key=lambda c: -c["n"])
    cab = tall[0]
    cab_top = cab["zmed"]

    # the bowl on it: above the cabinet top, inside the cabinet's footprint
    above = [desc(c) for c in _comps(
        fin & (H > cab_top + 0.018) & (H < cab_top + 0.12), 15)]
    above = [c for c in above
             if c["ymin"] >= cab["ymin"] - 0.012 and c["ymax"] <= cab["ymax"] + 0.012
             and c["xmin"] >= cab["xmin"] - 0.012 and c["xmax"] <= cab["xmax"] + 0.012]
    above.sort(key=lambda c: -c["n"])
    bowl = above[0]
    # the camera clips this bowl at the left FOV edge, so its bbox centroid is
    # biased; a Kasa fit on the top ring recovers the true centre.
    sel = bowl["zs"] > bowl["ztop"] - 0.010
    bcx, bcy, rim_r, rim_res = _kasa(bowl["xs"][sel], bowl["ys"][sel])

    # the plate: the disc standing proud of the slab that carries it
    band = [desc(c) for c in _comps(
        fin & (H > table_z + 0.012) & (H < table_z + 0.16), 12)]
    slabs = [c for c in band if c["xmin"] > -0.45 and c["n"] > 250]
    slabs.sort(key=lambda c: -c["n"])
    slab = slabs[0]
    sh, se = np.histogram(slab["zs"], bins=30)
    slab_z = float(se[sh.argmax()])
    psel = slab["zs"] > slab_z + 0.005
    plate = dict(x=float(slab["xs"][psel].mean()),
                 y=float(slab["ys"][psel].mean()),
                 ztop=float(slab["zs"][psel].max()))

    return dict(table_z=table_z, cab_top=cab_top, cab_ymax=float(cab["ymax"]),
                cab_xmin=float(cab["xmin"]), bowl_top=float(bowl["ztop"]),
                bcx=bcx, bcy=bcy, rim_r=rim_r, rim_res=rim_res, plate=plate)


# --------------------------------------------------------------------------

def goto(api, target, tries=2, tol=TOL_WP, seconds=SEC_NEAR):
    """Position-only. A SMALL residual is controller bias and is folded back
    into the command; a LARGE one means the move ran out of steps or is being
    blocked, so the same target is re-issued rather than extrapolated out of
    the workspace."""
    target = np.asarray(target, float)
    cmd = target.copy()
    eef = api.eef()
    e = float(np.linalg.norm(target - eef))
    if e < tol:
        return eef, e
    for _ in range(tries):
        api.move(cmd, rotation=None, seconds=seconds)
        eef = api.eef()
        err = target - eef
        e = float(np.linalg.norm(err))
        if e < tol:
            break
        cmd = target.copy() if e > BIAS_MAX else cmd + err
    return eef, e


def descend_to(api, x, y, z, tries=4, tol_z=0.0025, tol_xy=0.0035):
    """Converge the grasp pose in all three axes.

    z sets the bite depth; x sets whether the jaws cut a DIAMETER of the rim or
    a thin chord of it. POS_TOL is 0.012, so both need the bias-cancel loop --
    converging z alone (as v8-v12 did) let x drift 14 mm during the descent and
    halved the captured wall thickness."""
    target = np.array([x, y, z], float)
    cmd = target.copy()
    eef = api.eef()
    for _ in range(tries):
        api.move(cmd, rotation=None, seconds=SEC_NEAR)
        eef = api.eef()
        err = target - eef
        if abs(err[2]) < tol_z and float(np.linalg.norm(err[:2])) < tol_xy:
            break
        cmd = target.copy() if float(np.linalg.norm(err)) > BIAS_MAX else cmd + err
    return eef


def walk_to(api, target, z, y, tries=1):
    """Walk a long retract in short hops. A single far command saturates every
    axis at once and can fold the arm into a pose it cannot leave; short hops
    stay inside the controller's linear range. Logs each hop so a real reach
    limit shows up as the hop where progress stops."""
    x0 = float(api.eef()[0])
    n = max(1, int(np.ceil(abs(target - x0) / WALK_STEP)))
    eef = api.eef()
    for i in range(1, n + 1):
        xi = x0 + (target - x0) * i / n
        eef, e = goto(api, [xi, y, z], tries=tries, tol=TOL_WP, seconds=SEC_NEAR)
        api.log(f"hop {i}/{n} want_x={round(xi, 4)} got={eef.round(4).tolist()}")
        if abs(float(eef[0]) - xi) > 0.060:
            api.log(f"hop stalled at x={round(float(eef[0]), 4)}")
            break
    return eef


def walk_z(api, x, y, target_z, step=DESC_STEP):
    """Walk a long descent in short hops, same reasoning as walk_to."""
    z0 = float(api.eef()[2])
    n = max(1, int(np.ceil(abs(target_z - z0) / step)))
    eef = api.eef()
    for i in range(1, n + 1):
        zi = z0 + (target_z - z0) * i / n
        eef, e = goto(api, [x, y, zi], tries=1, tol=TOL_WP, seconds=SEC_NEAR)
        api.log(f"zhop {i}/{n} want_z={round(zi, 4)} got={eef.round(4).tolist()}")
        if abs(float(eef[2]) - zi) > 0.060:
            api.log(f"zhop stalled at z={round(float(eef[2]), 4)}")
            break
    return eef


def try_grasp(api, s, depth):
    gx, gy = s["bcx"], s["bcy"] + PINCH_R
    rim_top = s["bowl_top"]
    api.grip(OPEN_W)
    api.settle(0.2)
    eef, e = goto(api, [gx, gy, rim_top + 0.060], tries=2, tol=0.006,
                  seconds=SEC_FAR)
    _log(api, phase="pregrasp", eef=eef.round(4).tolist(), err=round(e, 4))
    eef = descend_to(api, gx, gy, rim_top - depth + TIP_OFFSET)
    tip_z = float(eef[2]) - TIP_OFFSET
    _log(api, phase="at_grasp", depth=depth, eef=eef.round(4).tolist(),
         tip_z=round(tip_z, 4), bite=round(rim_top - tip_z, 4),
         dx=round(float(eef[0]) - gx, 4), dy=round(float(eef[1]) - gy, 4))

    api.grip(0.0)
    api.settle(0.4)
    g1 = api.gripper()
    z0 = float(api.eef()[2])
    eef, e = goto(api, [gx, gy, rim_top + 0.090], tries=2, tol=TOL_WP)
    g2 = api.gripper()
    rose = float(eef[2]) - z0
    gap = g2["width_m"]
    held = (GAP_BAND[0] < gap < GAP_BAND[1] and gap > 0.80 * g1["width_m"]
            and rose > 0.040)
    _log(api, phase="lift", depth=depth, close_gap=round(g1["width_m"], 5),
         lift_gap=round(gap, 5), rose=round(rose, 4), held=held,
         eef=eef.round(4).tolist())
    return held, eef, rose


def run(api):
    s = perceive(api)
    _log(api, table_z=round(s["table_z"], 4), cab_top=round(s["cab_top"], 4),
         cab_ymax=round(s["cab_ymax"], 4), bowl_top=round(s["bowl_top"], 4),
         bcx=round(s["bcx"], 4), bcy=round(s["bcy"], 4),
         rim_r=round(s["rim_r"], 4), rim_res=round(s["rim_res"], 4),
         plate=s["plate"])

    held, depth, eef = False, PINCH_DEPTHS[0], api.eef()
    for k, d in enumerate(PINCH_DEPTHS):
        if k:
            api.grip(OPEN_W)
            api.settle(0.2)
            goto(api, [s["bcx"], s["bcy"] + PINCH_R, s["bowl_top"] + 0.090],
                 tries=1, tol=TOL_WP)
            try:
                s2 = perceive(api)
                s.update(s2)
                _log(api, phase="reperceive", attempt=k, bcx=round(s["bcx"], 4),
                     bcy=round(s["bcy"], 4), rim_r=round(s["rim_r"], 4))
            except Exception as ex:
                _log(api, phase="reperceive_failed", err=str(ex)[:120])
                break
        held, eef, rose = try_grasp(api, s, d)
        depth = d
        if held:
            break
        if rose < 0.005:
            _log(api, phase="starved", rose=round(rose, 4))
            break
    _log(api, phase="grasp_done", held=held, depth=depth, attempts=k + 1)

    p = s["plate"]
    px, py = p["x"], p["y"] + OFF_Y
    carry_z = float(eef[2])
    # the held bowl's -y edge must clear the cabinet's +y face, and its widest
    # part hangs BELOW the cabinet top once the arm drops to mid-height.
    clear_y = s["cab_ymax"] + OFF_Y + BOWL_R + 0.020
    _log(api, phase="route", clear_y=round(clear_y, 4), px=round(px, 4),
         py=round(py, 4))

    # y out of the cabinet's footprint while still above it ...
    eefw, e = goto(api, [float(eef[0]), clear_y, carry_z], tries=2, seconds=SEC_FAR)
    _log(api, phase="wp_clear", eef=eefw.round(4).tolist(), err=round(e, 4))
    # ... all the way down to a low carry, walked, so the arm unfolds ...
    eefw = walk_z(api, float(eefw[0]), clear_y, CARRY_LOW_Z)
    _log(api, phase="wp_drop", eef=eefw.round(4).tolist())
    # ... across in x down there, still clear of the cabinet in y ...
    eefw = walk_to(api, px, CARRY_LOW_Z, clear_y)
    _log(api, phase="wp_across", eef=eefw.round(4).tolist(),
         err=round(abs(float(eefw[0]) - px), 4))
    # ... and only now onto the plate.
    eef2, e = goto(api, [px, py, CARRY_LOW_Z], tries=3, tol=TOL_TIGHT,
                   seconds=SEC_NEAR)
    _log(api, phase="over_plate", eef=eef2.round(4).tolist(), err=round(e, 4),
         grip=api.gripper())

    # set the bowl down ON the plate: aim 10 mm through the contact height and
    # let the plate stop the arm, so a tilted bowl levels itself instead of
    # being dropped and rolling off (only ~10 mm of margin on the plate).
    contact_z = p["ztop"] + TIP_OFFSET + (BOWL_H - depth)
    eef3, e = goto(api, [px, py, contact_z - 0.010], tries=2, seconds=SEC_NEAR)
    _log(api, phase="at_place", contact_z=round(contact_z, 4),
         eef=eef3.round(4).tolist(), err=round(e, 4), grip=api.gripper(),
         stall=round(float(eef3[2]) - (contact_z - 0.010), 4))

    api.grip(OPEN_W)
    api.settle(0.8)
    _log(api, phase="released", grip=api.gripper())
    return f"v13 held={held} depth={depth} att={k + 1}"
