"""c2clean goal_put_bowl_on_plate_pos_k0 -- v7.

Intent: "put the bowl on the plate".

Mechanism: the gripper opens to 0.0778 and the bowl is 0.107 across, so it
cannot be grasped across. Its interior is open from 0.008 above the table
upward and its wall is ~6 mm thick, so the hold is a WALL STRADDLE -- one
finger inside the bowl, one outside, closing on the wall.

Three changes from v2 (5/8 on the probe seeds), each aimed at one of its
three distinct failures:

  * the in-flight `hang` measurement was reading the mask floor, not the
    bowl, so the bowl was released 0.11 m above the plate and bounced (ep57).
    The band is now anchored to the fingertips and the bowl is SET DOWN, its
    base 4 mm above the plate floor;
  * ep65 ran out of the episode's 1000 simulator steps mid-descent, so the
    release happened at travel height.  The long trip out to a parking pose
    is gone (the start pose does not occlude the bowl, and the plate is read
    later, from over the bowl, where nothing occludes it either), the travel
    height is down from 0.20 to 0.10, and every move is shorter;
  * ep53's straddle slipped during the lift.  The grasp is now checked from
    the fingertip band after the lift and retried once if the hand came up
    empty.

v4 flips the straddle to the bowl's +y side.  v3 scored 8/8 but its receipts
show the set-down never happened: LOWER stalled at z = 1.003 with a 0.05
residual on every seed, so the bowl was still being dropped 55 mm.  Grasping
the -y side puts the eef 47 mm to -y of the plate centre at release, and the
hand body (+/-0.06 in y, measured off the start pose) then reaches y = -0.155,
inside the cabinet, whose top stands 0.26-0.35 above the table for all
y < -0.122.  Straddling the +y side instead puts the release pose at y ~ 0.00,
clear of it, and the bowl can actually be set down.  v4 scored 8/8 and every
move converged (residuals 0.002-0.013, 93-154 of the 1000 simulator steps
against v3's 310-441).

v5 fixes the one thing v4's receipts still got wrong: the carried bowl's
`hang` was measured over a disc of radius 0.13 around the eef, which reaches
the stove slab (its near edge is 0.115 away at the grasp pose), so on two
seeds it read 0.087 -- the mask floor -- and forced a pointless re-grasp.
The minimum is now taken only over points inside the fitted bowl circle.
v5 scored 8/8 on the probe seeds.

v6 attacks the only failure left in v4's full-15 receipt (ep54).  Two
measurements, both cheap, decide this task and nothing else does:

  * how much aim error the task tolerates.  Probed directly by displacing the
    release aim: +0.020 in y is 8/8, +0.030 in y is 0/8, +0.030 in x is 3/8.
    The cliff is where geometry says it should be -- the plate's floor disc
    is 0.052 and the bowl's foot ring 0.025, so about 0.025 of slack;
  * where the plate is.  ep54 released 0.03 out because the plate came back
    from a floor-cell centroid over a blob the arm had eaten: PLATE0 was None
    and PLATE2 had 58 floor cells.

So v6 (a) reads the scene from a parked pose at (0.05, 0.32) -- bare table on
every probed seed and off every cam_high sight line to bowl and plate -- and
(b) takes the plate centre from a CIRCLE FIT to its rim ring rather than a
centroid.  A centroid of a clipped disc is biased by the clipping; a circle
fit needs only an arc.  Measured on the 15 start-pose dumps, where the arm
does cover the plate, the rim fit lands 0.012-0.020 to +x of the centroid --
the direction the arm's shadow predicts.

v7 adds the two things ep54 turned out to need.  Its plate is not flat: it
rests against the cabinet's foot, its floor plane tilts 7.9 degrees and its
centre sits 0.0133 above the table instead of 0.0075 (ep64 tilts 3.8).  That
broke v6 twice over -- absolute height bands read the tilted floor as the rim
ring and fitted a circle of the wrong radius (0.0545 against 0.0655), and the
bowl, set down on a slope, slid 0.025 downhill, which is exactly the aim
tolerance measured above.

  * every plate band is now taken RELATIVE to the plate's own fitted floor
    plane, so tilt and standoff drop out of the segmentation, and the release
    height is read off that plane at the release point;
  * the placement is verified and corrected.  After the bowl is set down the
    arm parks, the bowl is re-fitted, and if its centre is more than 0.012
    from the plate's it is picked up and set down again displaced by minus the
    observed error -- the slide is a property of the slope, so repeating it
    from a corrected start lands the bowl on the centre.  LIBERO latches task
    success, so a correction can only ever add.

The grasp height is now expressed relative to the bowl's own measured rim
(0.0211 below it) rather than to the table: the same thing while the bowl
stands on the table, and the right thing once it stands on a plate.
"""

import numpy as np

PROVENANCE = {
    "TABLE_Z_FALLBACK": {
        "source": "debug-seed measurement: modal z of the flat surface in the "
                  "cam_high cloud, 0.9010 on all 8 probed seeds; only a "
                  "fallback -- the value used is re-measured each episode",
        "allowed": True},
    "TIP": {
        "source": "debug-seed measurement, two independent ways: (a) v0 dump, "
                  "seed 51 -- lowest gripper point in the cam_high cloud is "
                  "z=1.1652 with the eef reported at z=1.17328, difference "
                  "0.0081; (b) v1 run -- closed gripper pressed onto the empty "
                  "table stalled with the eef at 0.9089 over a 0.9009 table, "
                  "difference 0.0080",
        "allowed": True},
    "BOWL_BAND": {
        "source": "debug-seed measurement: the bowl rim tops out 0.0511 above "
                  "the table on all 8 probed seeds; the 0.038-0.075 band "
                  "separates it from the stove slab (0.032) and the box (0.020)",
        "allowed": True},
    "PLATE_BAND": {
        "source": "debug-seed measurement: plate floor 0.008-0.009 and rim "
                  "0.019-0.024 above the table, seeds 51-65",
        "allowed": True},
    "WALL_INSET0/WALL_SLOPE": {
        "source": "debug-seed measurement: radial height profile of the bowl "
                  "(v0 dump, seed 51) -- outer wall radius 0.0417/0.0451/0.0498 "
                  "and inner wall radius 0.0352/0.0392/0.0440 at heights "
                  "0.020/0.030/0.040, so the wall midline is 0.01525 inside "
                  "the fitted rim radius at h=0.020 and climbs 0.4225 per "
                  "metre of height",
        "allowed": True},
    "H_CLOSE": {
        "source": "debug-seed measurement: the bowl's interior is clear from "
                  "0.0074 upward, so fingers straddling at 0.030 sit inside "
                  "the cavity and 0.021 below the rim",
        "allowed": True},
    "SIDE": {
        "source": "debug-seed measurement: the cabinet occupies every y < -0.122 "
                  "with a top 0.26-0.35 above the table, and the hand body "
                  "spans +/-0.06 in y (measured off the start pose in the v0 "
                  "cloud), so releasing at y = -0.095 (what a -y straddle "
                  "forces) collides -- v3 receipts show LOWER stalling at "
                  "z=1.003 on all 8 probed seeds",
        "allowed": True},
    "PARK": {
        "source": "debug-seed measurement: (0.05, 0.32) is bare table on all "
                  "15 debug seeds, and the cam_high rays to the bowl and to "
                  "the plate pass at y = 0.12 and y = -0.04 there",
        "allowed": True},
    "PLATE_RIM_BAND": {
        "source": "debug-seed measurement: measured off its own floor plane "
                  "the plate rises 0.0055-0.0145 in the ring that fits a "
                  "circle of radius 0.0655-0.0668 with 1-2 mm scatter on 14 "
                  "of the 15 debug seeds; taken relative because the plate is "
                  "not always flat (ep54 tilts 7.9 deg, ep64 3.8 deg)",
        "allowed": True},
    "RIM_TO_GRASP": {
        "source": "debug-seed measurement: the bowl's rim tops out 0.0511 "
                  "above the table and the straddle closes at 0.030, i.e. "
                  "0.0211 below the bowl's own rim -- the form that still "
                  "holds once the bowl stands on a plate",
        "allowed": True},
    "FIX_TOL": {
        "source": "debug-seed measurement: displacing the release aim is 8/8 "
                  "at +0.020 in y and 0/8 at +0.030, so a settled miss past "
                  "0.012 is worth correcting and anything under it is not",
        "allowed": True},
    "CARRY_H": {
        "source": "debug-seed measurement: the straight line from the bowl to "
                  "the plate crosses nothing taller than the plate rim (0.024) "
                  "and the box (0.020); the carried bowl hangs 0.030 below the "
                  "fingertips, so 0.10 clears both with 0.045 to spare",
        "allowed": True},
    "PLACE_CLEAR": {
        "source": "debug-seed measurement: the plate floor is flat (0.0080 to "
                  "0.0096) out to radius 0.045, wider than the bowl's foot, so "
                  "the bowl only has to be let go 4 mm above it",
        "allowed": True},
    "GRIP_OPEN/GRIP_SHUT": {
        "source": "generic controller mechanics: grip(<0.025) closes, else "
                  "opens; measured open gap 0.0778 at episode start",
        "allowed": True},
    "MOVE_SECONDS/TRIES": {
        "source": "generic controller mechanics: move stops inside a 12 mm "
                  "tolerance and costs at most max(40, 120*seconds) steps of a "
                  "1000-step episode (sim_steps read from the v1 receipt), so "
                  "waypoints are re-commanded with the standing error added "
                  "back, at most 3 times, at 0.6 s each",
        "allowed": True},
}

TABLE_Z_FALLBACK = 0.9010
TIP = 0.0081                 # eef reference above the fingertip plane
CELL = 0.005
XLO, XHI = -0.40, 0.30
YLO, YHI = -0.42, 0.42

H_CLOSE = 0.030              # straddle height, above the bowl's own base
RIM_TO_GRASP = 0.0211        # ...i.e. this far below the bowl's own rim
FIX_TOL = 0.012              # settled miss worth a second attempt
WALL_INSET0 = 0.01525        # rim radius -> wall midline, at h = 0.020
WALL_SLOPE = 0.4225          # d(wall midline)/dh
CARRY_H = 0.10               # fingertip travel height above the table
PARK = (0.05, 0.32)          # bare table, off every cam_high sight line
PARK_H = 0.15
GRIP_OPEN = 0.08
GRIP_SHUT = 0.0
PLACE_CLEAR = 0.004          # bowl base above the plate floor at release
SIDE = +1.0                  # straddle the bowl's +y side (away from the cabinet)
AIM_DX = 0.0                 # deliberate place-aim displacement (envelope probes)
AIM_DY = 0.0


def wall_mid(r_rim, h):
    """Radius of the bowl wall's midline at height h above the table."""
    return r_rim - (WALL_INSET0 - WALL_SLOPE * (h - 0.020))


# ---------------------------------------------------------------------------
# perception

def cloud(frame):
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    d = np.asarray(frame.depth, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    good = np.isfinite(d) & (d > 0)
    z = np.where(good, d, 1.0)
    x = (uu - K[0, 2]) * z / K[0, 0]
    y = (vv - K[1, 2]) * z / K[1, 1]
    P = np.stack([x, y, z, np.ones_like(z)], -1) @ T.T
    return P[..., :3], good


def table_z(P, good):
    Z = P[..., 2][good & (np.abs(P[..., 0]) < 0.30) & (np.abs(P[..., 1]) < 0.40)]
    if Z.size < 1000:
        return TABLE_Z_FALLBACK
    hist, edges = np.histogram(Z, bins=400, range=(0.70, 1.10))
    k = int(np.argmax(hist))
    sel = Z[(Z > edges[k] - 0.004) & (Z < edges[k + 1] + 0.004)]
    return float(np.median(sel))


def grid(P, good, tz):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = good & (X > XLO) & (X < XHI) & (Y > YLO) & (Y < YHI) & (Z > tz - 0.03)
    nx = int(round((XHI - XLO) / CELL))
    ny = int(round((YHI - YLO) / CELL))
    ix = np.clip(((X[m] - XLO) / CELL).astype(int), 0, nx - 1)
    iy = np.clip(((Y[m] - YLO) / CELL).astype(int), 0, ny - 1)
    H = np.zeros((nx, ny))
    np.maximum.at(H, (ix, iy), Z[m] - tz)
    return H


def components(mask):
    lab = np.where(mask, np.arange(mask.size).reshape(mask.shape) + 1, 0)
    for _ in range(400):
        prev = lab
        big = np.where(lab > 0, lab, lab.size + 1)
        m = big.copy()
        m[1:, :] = np.minimum(m[1:, :], big[:-1, :])
        m[:-1, :] = np.minimum(m[:-1, :], big[1:, :])
        m[:, 1:] = np.minimum(m[:, 1:], big[:, :-1])
        m[:, :-1] = np.minimum(m[:, :-1], big[:, 1:])
        m[1:, 1:] = np.minimum(m[1:, 1:], big[:-1, :-1])
        m[:-1, :-1] = np.minimum(m[:-1, :-1], big[1:, 1:])
        m[1:, :-1] = np.minimum(m[1:, :-1], big[:-1, 1:])
        m[:-1, 1:] = np.minimum(m[:-1, 1:], big[1:, :-1])
        lab = np.where(mask, m, 0)
        if np.array_equal(lab, prev):
            break
    return lab


def blobs(H, lo, hi, nmin):
    lab = components((H > lo) & (H < hi))
    out = []
    for k in np.unique(lab):
        if k == 0:
            continue
        idx = np.where(lab == k)
        if len(idx[0]) < nmin:
            continue
        xs = XLO + (idx[0] + 0.5) * CELL
        ys = YLO + (idx[1] + 0.5) * CELL
        out.append(dict(n=len(xs), x=float(xs.mean()), y=float(ys.mean()),
                        xs=xs, ys=ys, hs=H[idx], htop=float(H[idx].max()),
                        rx=float((xs.max() - xs.min()) / 2),
                        ry=float((ys.max() - ys.min()) / 2)))
    return out


def kasa(x, y):
    A = np.c_[x, y, np.ones(len(x))]
    c = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)[0]
    cx, cy = c[0] / 2.0, c[1] / 2.0
    return float(cx), float(cy), float(np.sqrt(max(c[2] + cx ** 2 + cy ** 2, 1e-9)))


def find_bowl(P, good, H, tz):
    cands = [b for b in blobs(H, 0.038, 0.075, 30)
             if 0.030 < max(b["rx"], b["ry"]) < 0.080]
    if not cands:
        return None
    b = max(cands, key=lambda b: b["n"])
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    h = Z - tz
    ring = (good & (np.hypot(X - b["x"], Y - b["y"]) < 0.10)
            & (h > b["htop"] - 0.006) & (h < b["htop"] + 0.002))
    if int(ring.sum()) < 60:
        return None
    cx, cy, r = kasa(X[ring], Y[ring])
    scat = float(np.hypot(X[ring] - cx, Y[ring] - cy).std())
    return dict(cx=cx, cy=cy, r=r, htop=b["htop"], scat=scat, n=int(ring.sum()))


def plate_fit(P, good, tz, seed_xy):
    """Plate pose: floor plane first, then a circle fit to the rim ring taken
    RELATIVE to that plane.  A clipped disc's centroid is biased by the
    clipping and an absolute height band is biased by tilt; this is neither."""
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    h = Z - tz
    near = (good & (np.hypot(X - seed_xy[0], Y - seed_xy[1]) < 0.095)
            & (h > 0.003) & (h < 0.038))
    cx, cy = float(seed_xy[0]), float(seed_xy[1])
    r, plane, keep, x, y = 0.066, None, None, None, None
    for _ in range(3):
        inner = near & (np.hypot(X - cx, Y - cy) < 0.035)
        if int(inner.sum()) < 150:
            return None
        A = np.c_[X[inner] - cx, Y[inner] - cy, np.ones(int(inner.sum()))]
        plane = np.linalg.lstsq(A, h[inner], rcond=None)[0]
        dh = h - (plane[0] * (X - cx) + plane[1] * (Y - cy) + plane[2])
        m = near & (dh > 0.0055) & (dh < 0.0145)
        if int(m.sum()) < 120:
            return None
        x, y = X[m], Y[m]
        for _ in range(5):
            keep = np.abs(np.hypot(x - cx, y - cy) - r) < 0.010
            if int(keep.sum()) < 80:
                return None
            cx, cy, r = kasa(x[keep], y[keep])
    d = np.abs(np.hypot(x - cx, y - cy) - r)
    keep = d < 0.010
    ang = np.arctan2(y[keep] - cy, x[keep] - cx)
    cover = float(np.unique((ang / np.pi * 12).astype(int)).size) / 24.0
    return dict(cx=float(cx), cy=float(cy), r=float(r), n=int(keep.sum()),
                cover=cover, scat=float(d[keep].std()),
                gx=float(plane[0]), gy=float(plane[1]), h0=float(plane[2]),
                tilt=float(np.degrees(np.arctan(np.hypot(plane[0], plane[1])))))


def plate_floor_at(pl, x, y):
    """Height of the plate's floor plane above the table at (x, y)."""
    return pl["h0"] + pl["gx"] * (x - pl["cx"]) + pl["gy"] * (y - pl["cy"])


def find_plate(P, good, H, bowl):
    best = None
    for b in blobs(H, 0.005, 0.024, 60):
        if bowl is not None and np.hypot(b["x"] - bowl["cx"], b["y"] - bowl["cy"]) < 0.08:
            continue
        if not (0.050 < max(b["rx"], b["ry"]) < 0.12):
            continue
        floor = b["hs"] < 0.011
        if int(floor.sum()) < 40:
            continue
        if best is None or floor.sum() > best[0]:
            best = (int(floor.sum()), b, floor)
    if best is None:
        return None
    _, b, floor = best
    return dict(cx=float(b["xs"][floor].mean()), cy=float(b["ys"][floor].mean()),
                htop=b["htop"], floor=float(np.median(b["hs"][floor])),
                rx=b["rx"], ry=b["ry"], n=b["n"], nfloor=int(floor.sum()))


# ---------------------------------------------------------------------------
# motion

def goto(api, target, seconds=0.6, tol=0.005, tries=3):
    """Drive to `target`, cancelling the controller's 12 mm stop tolerance by
    adding the standing error back into the next command."""
    t = np.asarray(target, float)
    cmd = t.copy()
    for _ in range(tries):
        api.move(cmd, seconds=seconds)
        err = t - api.eef()
        if float(np.linalg.norm(err)) < tol:
            break
        cmd = cmd + err
    cur = api.eef()
    return cur, float(np.linalg.norm(t - cur))


def look(api):
    f = api.capture("cam_high")
    return cloud(f)


def held_bowl(api, P, good, tz):
    """Fit the carried bowl from the band just under the fingertips.  Nothing
    on the arm reaches below its own fingertips, so anything there is cargo."""
    eef = api.eef()
    tip = eef[2] - TIP
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (good & (Z < tip - 0.004) & (Z > tip - 0.080) & (Z > tz + 0.020)
         & (np.hypot(X - eef[0], Y - eef[1]) < 0.13))
    if int(m.sum()) < 120:
        return None
    top = float(Z[m].max())
    ring = m & (Z > top - 0.008)
    if int(ring.sum()) < 60:
        return None
    cx, cy, r = kasa(X[ring], Y[ring])
    scat = float(np.hypot(X[ring] - cx, Y[ring] - cy).std())
    # the disc above still reaches the stove slab, whose near edge is 0.115
    # from the grasp pose; only what falls inside the fitted circle is bowl.
    own = m & (np.hypot(X - cx, Y - cy) < r + 0.012)
    if int(own.sum()) < 100:
        return None
    return dict(cx=cx, cy=cy, r=r, scat=scat, n=int(own.sum()), nring=int(ring.sum()),
                dx=float(cx - eef[0]), dy=float(cy - eef[1]),
                hang=float(eef[2] - float(Z[own].min())))


def held_ok(h):
    return (h is not None and 0.035 < h["r"] < 0.070 and h["scat"] < 0.006
            and h["nring"] > 60 and abs(h["dx"]) < 0.05
            and 0.020 < -SIDE * h["dy"] < 0.085 and 0.020 < h["hang"] < 0.060)


def read_scene(api, tz, seed_xy=None):
    """Bowl and plate from one cam_high frame."""
    P, good = look(api)
    H = grid(P, good, tz)
    bowl = find_bowl(P, good, H, tz)
    cen = find_plate(P, good, H, bowl)
    if seed_xy is None:
        seed_xy = (cen["cx"], cen["cy"]) if cen is not None else (-0.21, -0.045)
    pl = plate_fit(P, good, tz, seed_xy)
    return P, good, bowl, cen, pl


def grasp(api, bowl, tz, log, tag):
    """Straddle the bowl's wall on the +y side and close.  The grasp height is
    referred to the bowl's own rim, so it is the same whether the bowl stands
    on the table or on the plate."""
    rho = wall_mid(bowl["r"], H_CLOSE)
    gx = bowl["cx"]
    gy = bowl["cy"] + SIDE * rho
    close_z = tz + bowl["htop"] - RIM_TO_GRASP + TIP
    cur, res = goto(api, [gx, gy, tz + CARRY_H + TIP], tries=2)
    log(f"{tag}_OVER rho={rho:.4f} aim=({gx:.4f},{gy:.4f}) eef={np.round(cur, 4).tolist()} res={res:.4f}")
    cur, res = goto(api, [gx, gy, close_z], tol=0.004)
    log(f"{tag}_DESCEND eef={np.round(cur, 4).tolist()} res={res:.4f} "
        f"tip_h={cur[2] - tz - TIP:.4f}")
    api.grip(GRIP_SHUT)
    log(f"{tag}_CLOSED {api.gripper()}")
    cur, res = goto(api, [gx, gy, tz + CARRY_H + TIP], tries=2)
    log(f"{tag}_LIFT eef={np.round(cur, 4).tolist()} res={res:.4f} grip={api.gripper()}")
    return gx, gy, rho


def put_down(api, tz, plate, aim_xy, dx, dy, hang, log, tag):
    """Set the carried bowl down with its centre on `aim_xy`."""
    px, py = aim_xy[0] - dx, aim_xy[1] - dy
    floor = plate_floor_at(plate, aim_xy[0], aim_xy[1])
    place_z = tz + floor + PLACE_CLEAR + hang
    log(f"{tag}_PLAN aim=({aim_xy[0]:.4f},{aim_xy[1]:.4f}) eef=({px:.4f},{py:.4f}) "
        f"z={place_z:.4f} floor={floor:.4f} dx={dx:.4f} dy={dy:.4f} hang={hang:.4f}")
    cur, res = goto(api, [px, py, tz + CARRY_H + TIP], tries=2)
    log(f"{tag}_OVER eef={np.round(cur, 4).tolist()} res={res:.4f} grip={api.gripper()}")
    cur, res = goto(api, [px, py, place_z], tol=0.004)
    log(f"{tag}_LOWER eef={np.round(cur, 4).tolist()} res={res:.4f} grip={api.gripper()}")
    api.grip(GRIP_OPEN)
    api.settle(0.4)
    goto(api, [px, py, tz + CARRY_H + TIP], tries=1)
    api.settle(0.6)


def run(api):
    log = api.log
    log(f"INSTRUCTION {api.instruction()!r}")

    # -- 1. park clear of the scene, then read bowl and plate --------------
    cur, res = goto(api, [PARK[0], PARK[1], TABLE_Z_FALLBACK + PARK_H], tries=2)
    log(f"PARK eef={np.round(cur, 4).tolist()} res={res:.4f}")
    P, good = look(api)
    tz = table_z(P, good)
    P, good, bowl, cen, plate = read_scene(api, tz)
    log(f"TABLE_Z {tz:.4f}")
    log(f"BOWL {bowl}")
    log(f"PLATE_CENTROID {cen}")
    log(f"PLATE_FIT {plate}")
    if bowl is None:
        return "bowl not found"
    if plate is None or not (0.055 < plate["r"] < 0.080) or plate["cover"] < 0.55:
        if cen is None:
            return "plate not found"
        plate = dict(cx=cen["cx"], cy=cen["cy"], r=0.066, cover=0.0, scat=0.0,
                     n=cen["n"], gx=0.0, gy=0.0, h0=cen["floor"], tilt=0.0)
        log("PLATE_FIT rejected, falling back to the floor-cell centroid")

    # -- 2. pick the bowl up -----------------------------------------------
    gx, gy, rho = grasp(api, bowl, tz, log, "G1")
    P2, good2 = look(api)
    held = held_bowl(api, P2, good2, tz)
    log(f"HELD {held}")

    if not held_ok(held):
        api.grip(GRIP_OPEN)
        goto(api, [gx, gy, tz + CARRY_H + TIP], tries=1)
        _, _, b2, _, _ = read_scene(api, tz, seed_xy=(plate["cx"], plate["cy"]))
        log(f"RETRY_BOWL {b2}")
        use = b2 if (b2 is not None and abs(b2["r"] - bowl["r"]) < 0.010) else bowl
        grasp(api, use, tz, log, "G2")
        P2, good2 = look(api)
        held = held_bowl(api, P2, good2, tz)
        log(f"RETRY_HELD {held}")

    if held_ok(held):
        dx, dy, hang, src = held["dx"], held["dy"], held["hang"], "measured"
    else:
        dx, dy, hang, src = 0.0, -SIDE * rho, TIP + H_CLOSE, "nominal"
    log(f"CARRY[{src}] dx={dx:.4f} dy={dy:.4f} hang={hang:.4f}")

    # -- 3. set it down on the plate ---------------------------------------
    aim = (plate["cx"] + AIM_DX, plate["cy"] + AIM_DY)
    put_down(api, tz, plate, aim, dx, dy, hang, log, "P1")

    # -- 4. verify, and correct the aim by the miss it actually made -------
    goto(api, [PARK[0], PARK[1], tz + PARK_H], tries=2)
    P3, good3 = look(api)
    H3 = grid(P3, good3, tz)
    now = find_bowl(P3, good3, H3, tz)
    if now is None:
        log("VERIFY bowl not found; leaving it where it is")
        return "v7 placed, unverified"
    miss = (now["cx"] - plate["cx"], now["cy"] - plate["cy"])
    log(f"VERIFY bowl=({now['cx']:.4f},{now['cy']:.4f}) htop={now['htop']:.4f} "
        f"r={now['r']:.4f} miss=({miss[0]:+.4f},{miss[1]:+.4f}) "
        f"tilt={plate['tilt']:.2f}")
    if float(np.hypot(*miss)) <= FIX_TOL:
        return "v7 placed and verified"

    # the bowl slid; the slide belongs to the slope, so start that far up it
    gx, gy, rho = grasp(api, now, tz, log, "G3")
    P4, good4 = look(api)
    held2 = held_bowl(api, P4, good4, tz)
    log(f"FIX_HELD {held2}")
    if held_ok(held2):
        dx, dy, hang = held2["dx"], held2["dy"], held2["hang"]
    aim = (plate["cx"] - miss[0], plate["cy"] - miss[1])
    put_down(api, tz, plate, aim, dx, dy, hang, log, "P2")

    goto(api, [PARK[0], PARK[1], tz + PARK_H], tries=2)
    P5, good5 = look(api)
    H5 = grid(P5, good5, tz)
    fin = find_bowl(P5, good5, H5, tz)
    if fin:
        log(f"FINAL bowl=({fin['cx']:.4f},{fin['cy']:.4f}) htop={fin['htop']:.4f} "
            f"miss=({fin['cx'] - plate['cx']:+.4f},{fin['cy'] - plate['cy']:+.4f})")
    else:
        log("FINAL bowl not found")
    return "v7 placed, verified and corrected"
