"""c2k1clean / spa_bowl_on_ramekin_task_k1

Intent: "Pick the akita black bowl on the cookie box and place it on the plate".

The scene carries two akita black bowls. One stands on the ramekin, one on the
cookie box; they are told apart by the height of their rim above the table,
which is a property of the support and not of where the seed put them
(debug seeds 51-65: ramekin-borne rim top 1.0005-1.0007 m, cookie-box-borne
rim top 0.9707 m in every one of the 15). The cookie-box bowl is the one this
intent names, so the target is the LOW rim.

The bowl is 0.110 m across and the jaws span 0.080 m, so it cannot be grasped
across the body: the demonstrations pinch the bowl WALL, one finger inside the
bowl and one outside. The jaws open along base y with the default wrist
(measured: closing the gripper in place moves wrist-camera pixels along an axis
that maps to base [0,-1,0]), so the pinch belongs on the +/-y arc of the rim.
"""
import numpy as np

PROVENANCE = {
    "RIM_TOP_LO": {
        "source": "debug seeds 51-65 (probe1): the cookie-box-borne bowl's rim "
                  "top measured 0.9707 m on every seed; the ramekin-borne one "
                  "1.0005-1.0007 m. Window chosen to separate them.",
        "allowed": True},
    "RIM_TOP_HI": {
        "source": "same debug-seed measurement as RIM_TOP_LO", "allowed": True},
    "GRASP_OFF": {
        "source": "packs/c2k1clean_spa_bowl_on_ramekin_task_mate pack.json: its demo "
                  "grasps the cookie-box bowl at ee y 0.0638->0.0642 while the "
                  "bowl centre in its keyframes/demo0_t0000.png reprojects to "
                  "y~0.023, i.e. a rim offset of ~0.042 m; confirmed on debug "
                  "seeds 51/53/55 (grip effort 3.0 after closing).",
        "allowed": True},
    "GRASP_DZ": {
        "source": "mate pack: grasp ee z 0.953 against a 0.9707 m rim top = "
                  "0.018 m below the rim; debug seeds 51/53/55 confirm the "
                  "descent stalls on contact at ~0.959 and still grips.",
        "allowed": True},
    "RELEASE_DZ": {
        "source": "mate pack: it grasps at ee z 0.953 and releases at 0.9476, "
                  "5 mm lower, i.e. the bowl steps down from the cookie box to "
                  "the plate; against the debug-seed table plane 0.9011 that "
                  "release sits 0.046 m above the table. 0.040 is commanded so "
                  "the bowl is set down by contact rather than dropped.",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug-seed observation: 1.06 m clears both bowls (rim tops "
                  "<= 1.0007) and the plate on the transport between them.",
        "allowed": True},
    "PLATE_MIN_PX": {
        "source": "debug seeds 51-65 (probe1/probe2): the plate is the bright "
                  "flat region 4-22 mm proud of the table, 4000+ px at 512x512.",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi contract + debug seeds: api.gripper()['effort'] is "
                  "3.0 exactly while an object is held, 0.05 otherwise.",
        "allowed": True},
    "RETREAT": {
        "source": "debug-seed observation: parking the arm here before the "
                  "verification capture stops it occluding the plate.",
        "allowed": True},
    "WIN": {
        "source": "debug seeds 51-65: the workspace crop that keeps both bowls "
                  "and the plate (x in [-0.32,0.32], |y| < 0.42) while dropping "
                  "the wall and the far table edge.",
        "allowed": True},
    "TABLE_LO": {"source": "debug seeds 51-65: table plane at z=0.9011.",
                 "allowed": True},
    "TABLE_HI": {"source": "debug seeds 51-65: table plane at z=0.9011.",
                 "allowed": True},
}

TABLE_LO, TABLE_HI = 0.86, 0.94
RIM_TOP_LO, RIM_TOP_HI = 0.955, 0.985
GRASP_OFF = 0.042
GRASP_DZ = -0.018
RELEASE_DZ = 0.040
CARRY_Z = 1.06
PLATE_MIN_PX = 900
HOLD_EFFORT = 2.0          # effort is 3.0 when holding, 0.05 when not
WIN = (-0.32, 0.32, 0.42)  # x lo, x hi, |y| max
RETREAT = (-0.20, -0.18, 1.15)  # park clear of the camera ray to the plate


# ---------------------------------------------------------------- perception

def cloud(f):
    d = np.asarray(f.depth, float)
    h, w = d.shape
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    v, u = np.mgrid[0:h, 0:w].astype(float)
    x = (u - K[0, 2]) * d / K[0, 0]
    y = (v - K[1, 2]) * d / K[1, 1]
    return np.stack([x, y, d, np.ones_like(d)], -1) @ T.T[:, :3]


def label(mask):
    """4-connected components, iterative flood fill (no scipy in the sandbox)."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    seen = np.zeros((h, w), bool)
    cur = 0
    for y0, x0 in np.argwhere(mask):
        if seen[y0, x0]:
            continue
        cur += 1
        stack = [(y0, x0)]
        seen[y0, x0] = True
        while stack:
            y, x = stack.pop()
            lab[y, x] = cur
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    stack.append((ny, nx))
    return lab, cur


def perceive(api):
    """-> (target xyz-of-rim-centre, plate xy, table z). Either may be None."""
    f = api.capture("cam_high")
    P = cloud(f)
    rgb = np.asarray(f.rgb).astype(float)
    Z = P[..., 2]
    tbl = float(np.median(Z[(Z > TABLE_LO) & (Z < TABLE_HI)]))
    win = ((P[..., 0] > WIN[0]) & (P[..., 0] < WIN[1]) & (np.abs(P[..., 1]) < WIN[2]))

    target, tbest = None, 0
    lab, n = label(win & (Z > tbl + 0.020) & (Z < 1.15))
    for i in range(1, n + 1):
        s = lab == i
        if s.sum() < 250:
            continue
        zt = float(P[s][:, 2].max())
        if not (RIM_TOP_LO < zt < RIM_TOP_HI):
            continue
        rim = P[s & (Z > zt - 0.012)]
        # a bowl rim is a full circle ~0.11 across in both axes; anything much
        # smaller in one axis is not a rim seen whole and is not trusted
        dx, dy = np.ptp(rim[:, 0]), np.ptp(rim[:, 1])
        api.log("cand n=%d top=%.4f dx=%.3f dy=%.3f c=(%.3f,%.3f)"
                % (s.sum(), zt, dx, dy,
                   0.5 * (rim[:, 0].min() + rim[:, 0].max()),
                   0.5 * (rim[:, 1].min() + rim[:, 1].max())))
        if dx < 0.07 or dy < 0.07 or s.sum() <= tbest:
            continue
        tbest = int(s.sum())
        target = np.array([0.5 * (rim[:, 0].min() + rim[:, 0].max()),
                           0.5 * (rim[:, 1].min() + rim[:, 1].max()), zt])

    plate, best = None, 0
    lab2, n2 = label(win & (Z > tbl + 0.004) & (Z < tbl + 0.022) & (rgb.mean(-1) > 140))
    for i in range(1, n2 + 1):
        s = lab2 == i
        if s.sum() < PLATE_MIN_PX:
            continue
        q = P[s]
        if s.sum() > best:
            best = s.sum()
            plate = np.array([0.5 * (q[:, 0].min() + q[:, 0].max()),
                              0.5 * (q[:, 1].min() + q[:, 1].max())])
    api.log("perceive table=%.4f target=%s plate=%s"
            % (tbl, None if target is None else target.round(4).tolist(),
               None if plate is None else plate.round(4).tolist()))
    return target, plate, tbl


def holding(api):
    g = api.gripper()
    return g["effort"] >= HOLD_EFFORT, g


# ------------------------------------------------------------------- program

def run(api):
    api.log("instruction=%r" % api.instruction())
    target, plate, tbl = perceive(api)
    if target is None or plate is None:
        return "perception failed"

    # --- pick: rim pinch on a y arc, with a retry ladder ---------------------
    # +y first (the mate demonstration's arc), then -y, then the same arcs a
    # little further out in case the fingers landed on the rim lip.
    ladder = [(0.0, +GRASP_OFF), (0.0, -GRASP_OFF),
              (0.0, +GRASP_OFF + 0.005), (0.0, -GRASP_OFF - 0.005),
              (0.0, +GRASP_OFF - 0.006)]
    held = False
    for k, (ox, oy) in enumerate(ladder):
        gx, gy, zt = target[0] + ox, target[1] + oy, target[2]
        api.grip(0.08)
        api.move([gx, gy, zt + 0.09], seconds=2.0)
        api.move([gx, gy, zt + GRASP_DZ], seconds=2.0)
        r = api.move([gx, gy, zt + GRASP_DZ], seconds=1.2)   # converge: one
        e = api.eef()                                        # pass leaves ~9 mm
        api.log("try%d arc=(%.3f,%.3f) descend res=%.4f eef=%s err=(%.4f,%.4f)"
                % (k, ox, oy, r, e.round(4).tolist(), e[0] - gx, e[1] - gy))
        api.grip(0.0)
        api.settle(0.8)
        ok, g = holding(api)
        api.log("try%d closed grip=%s" % (k, g))
        if not ok:
            api.grip(0.08)
            api.settle(0.4)
            target, plate2, tbl = perceive(api)
            if plate2 is not None:
                plate = plate2
            if target is None:
                return "target lost"
            continue
        # gentle two-stage lift, checking the hold after each
        api.move([gx, gy, zt + 0.03], seconds=1.5)
        api.settle(0.3)
        api.move([gx, gy, CARRY_Z], seconds=2.0)
        api.settle(0.4)
        ok, g = holding(api)
        api.log("try%d lifted grip=%s" % (k, g))
        if ok:
            held = True
            grasp_off = np.array([ox, oy])
            break
        api.grip(0.08)
        api.settle(0.4)
        target, plate2, tbl = perceive(api)
        if plate2 is not None:
            plate = plate2
        if target is None:
            return "target lost after drop"
    if not held:
        return "no grasp"

    # --- place -------------------------------------------------------------
    # The jaws clamp the bowl WALL, so the carried bowl's centre sits one wall
    # radius from the eef, on the side the pinch came from: bowl = eef -
    # grasp_off. To land the bowl on the plate the eef therefore goes to
    # plate + grasp_off.
    px, py = plate[0] + grasp_off[0], plate[1] + grasp_off[1]
    api.move([px, py, CARRY_Z], seconds=2.5)
    api.settle(0.3)
    api.move([px, py, tbl + RELEASE_DZ], seconds=2.5)
    r = api.move([px, py, tbl + RELEASE_DZ], seconds=1.2)
    api.log("over plate res=%.4f eef=%s grip=%s"
            % (r, api.eef().round(4).tolist(), api.gripper()))
    api.grip(0.08)
    api.settle(0.8)
    api.move([px, py, CARRY_Z], seconds=2.0)
    # retreat clear of the camera's line to the plate before looking, or the
    # arm occludes the very rim we are trying to measure
    api.move(list(RETREAT), seconds=2.5)
    api.settle(0.6)

    # --- verify with our own eyes -------------------------------------------
    bowl, plate3, _ = perceive(api)
    if bowl is not None:
        ref = plate3 if plate3 is not None else plate
        api.log("verify bowl=%s plate=%s miss=(%.4f,%.4f) eef_release=(%.4f,%.4f) "
                "carry_off=(%.4f,%.4f)"
                % (bowl.round(4).tolist(), np.asarray(ref).round(4).tolist(),
                   bowl[0] - ref[0], bowl[1] - ref[1], px, py,
                   px - bowl[0], py - bowl[1]))
    else:
        api.log("verify: no full rim seen")
    return "placed"
