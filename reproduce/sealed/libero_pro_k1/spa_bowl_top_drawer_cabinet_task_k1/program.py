"""c2k1clean / spa_bowl_top_drawer_cabinet_task_k1 — v3.

Intent: "Pick the akita black bowl on the top of the wooden cabinet and place
it on the plate".

Shape of the solution
---------------------
* cam_high RGB-D -> base-frame point cloud.  The cabinet-top bowl is the only
  structure in the y < -0.10 half of the scene that rises above the cabinet's
  top plane; its rim is a circle, fitted with Kasa least squares, and its top
  height is the 99th percentile of the cluster.
* The grasp is a rim straddle along the jaw axis: the tool centre goes to
  (rim_cx, rim_cy + rim_r) so that the near finger descends inside the bowl and
  the far finger outside it, then the close pinches the wall.
* The controller stops inside POS_TOL, which on this descent is a systematic
  ~10 mm shortfall in +z; the commanded point is therefore re-issued with the
  measured error mirrored until the achieved height is inside the grasp band.
* Transport happens at a height above every prop in the scene, the bowl is
  centred over the plate (tool centre offset by the same rim radius, since the
  bowl hangs from the pinch point) and released just above the plate rim.
"""
import numpy as np

PROVENANCE = {
    "GREY_SAT": {
        "source": "debug-seed RGB (probe p1, seeds 51-65): robot links render "
                  "strongly coloured (green/blue/yellow), every table prop is "
                  "near-neutral; max(RGB)-min(RGB) separates them",
        "allowed": True},
    "BOWL_Z_LO": {
        "source": "probe p1 depth, seeds 51-65: cabinet top plane at z=1.125-1.130, "
                  "cabinet-top bowl rim at z=1.179; 1.14 is the cut between them",
        "allowed": True},
    "BOWL_Z_HI": {
        "source": "probe p1 depth: nothing on the cabinet exceeds z=1.18; 1.26 "
                  "only excludes the robot body (z up to 1.371)",
        "allowed": True},
    "BOWL_Y_MAX": {
        "source": "probe p1: the wooden cabinet occupies y < -0.12 in all 15 "
                  "debug seeds, the parked arm sits at y = +0.04",
        "allowed": True},
    "RIM_BAND": {
        "source": "probe p1: rim points lie within 5 mm of the cluster's 99th "
                  "percentile height; Kasa fit on them gives r=0.0537-0.0539 m "
                  "on every debug seed",
        "allowed": True},
    "GRASP_DZ": {
        "source": "probe p2 + v1 debug runs: every close whose achieved eef "
                  "height was rim_top-0.017 .. rim_top-0.025 carried the bowl; "
                  "rim_top-0.008 gripped nothing and rim_top-0.016 slipped "
                  "during the lift, so the command aims for rim_top-0.026",
        "allowed": True},
    "VERIFY_GAP": {
        "source": "probe p2 + v1/v2 debug runs: a carried bowl reads 0.0033-0.0065 m "
                  "of finger gap after the lift, a close on air 0.004 m and a "
                  "slipped bowl 0.0021 m, so the gap alone cannot decide; it is "
                  "used only as a floor next to the re-perception check",
        "allowed": True},
    "VERIFY_PARK": {
        "source": "probe p1: the cabinet is the whole y < -0.10 half-space, so "
                  "parking the tool at y = +0.12 puts both the arm and a carried "
                  "bowl outside the region find_bowl searches",
        "allowed": True},
    "SAFE_Z": {
        "source": "probe p1: the tallest prop in the scene is the cabinet-top "
                  "bowl rim at z=1.179, so 1.30 clears everything",
        "allowed": True},
    "PLATE_BAND": {
        "source": "probe p1, seeds 51-65: the plate is the flat 0.136 m disc on "
                  "the table with its rim at z=0.920; the ramekin (x<-0.12, top "
                  "0.944) and the cookie box (non-neutral colour, y<0.08) are "
                  "excluded by the band",
        "allowed": True},
    "RELEASE_Z": {
        "source": "probe p1 plate rim height 0.920 plus the bowl's measured "
                  "hang: grasped at rim_top-0.021 the bowl bottom sits 0.033 m "
                  "below the tool centre (rim 1.179, bowl base on the cabinet "
                  "plane 1.127)",
        "allowed": True},
    "BIAS_PASSES": {
        "source": "generic controller mechanics: move() exits inside its own "
                  "position tolerance, so the command is re-issued with the "
                  "measured error mirrored",
        "allowed": True},
    "STEP_BUDGET": {
        "source": "v1 debug runs: the episode ends after 1000 simulator steps "
                  "(seeds 61 and 63 exhausted it mid-placement), and move()'s "
                  "step cap is 2x60x`seconds`, so every command carries the "
                  "smallest cap that still converges",
        "allowed": True},
}

GREY_SAT = 40.0
BOWL_Z_LO, BOWL_Z_HI = 1.14, 1.26
BOWL_Y_MAX = -0.10
RIM_BAND = 0.005
GRASP_DZ = -0.026
VERIFY_GAP = 0.0025
VERIFY_PARK_Y = 0.12
SAFE_Z = 1.28
RELEASE_Z = 0.958
BIAS_PASSES = 2


# ---------------------------------------------------------------- perception
def _world(f):
    depth = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    ok = np.isfinite(depth) & (depth > 0)
    z = np.where(ok, depth, 1.0)
    X = (u - K[0, 2]) * z / K[0, 0]
    Y = (v - K[1, 2]) * z / K[1, 1]
    P = np.stack([X, Y, z, np.ones_like(z)], -1)
    W = (P @ T.T)[..., :3]
    rgb = np.asarray(f.rgb, float)
    neutral = (rgb.max(-1) - rgb.min(-1)) < GREY_SAT
    return W, ok & neutral & np.isfinite(W).all(-1)


def _kasa(P):
    x, y = P[:, 0], P[:, 1]
    A = np.stack([x, y, np.ones_like(x)], 1)
    c = np.linalg.lstsq(A, x * x + y * y, rcond=None)[0]
    cx, cy = c[0] / 2.0, c[1] / 2.0
    return float(cx), float(cy), float(np.sqrt(max(c[2] + cx * cx + cy * cy, 1e-9)))


def find_bowl(W, m):
    z = W[..., 2]
    sel = (m & (z > BOWL_Z_LO) & (z < BOWL_Z_HI) & (W[..., 1] < BOWL_Y_MAX)
           & (W[..., 0] > -0.45) & (W[..., 0] < 0.35))
    P = W[sel]
    if len(P) < 200:
        return None
    ztop = float(np.percentile(P[:, 2], 99.0))
    rim = P[P[:, 2] > ztop - RIM_BAND]
    if len(rim) < 80:
        return None
    cx, cy, r = _kasa(rim)
    if not (0.040 < r < 0.070):
        return None
    return {"cx": cx, "cy": cy, "r": r, "ztop": ztop, "n": int(len(P))}


def find_plate(W, m):
    z = W[..., 2]
    sel = (m & (z > 0.906) & (z < 0.935) & (W[..., 1] > 0.08)
           & (W[..., 0] > -0.12) & (W[..., 0] < 0.30))
    P = W[sel]
    if len(P) < 500:
        return None
    return {"cx": (float(P[:, 0].min()) + float(P[:, 0].max())) / 2.0,
            "cy": (float(P[:, 1].min()) + float(P[:, 1].max())) / 2.0,
            "ztop": float(np.percentile(P[:, 2], 99.0)), "n": int(len(P))}


# ------------------------------------------------------------------- motion
def goto(api, xyz, seconds=1.0, passes=1, tol=0.004, corr_s=0.5):
    """move(), then re-issue the command with the measured error mirrored.

    move() returns as soon as it is inside its own 12 mm position tolerance,
    which is coarse next to the 8 mm-wide band this grasp has to land in.  The
    correction commands carry a small step cap: when the descent is blocked by
    the rim rather than short of it, a long cap just burns the episode.
    """
    target = np.asarray(xyz, float)
    api.move(target, seconds=seconds)
    for _ in range(max(0, passes)):
        err = target - api.eef()
        if float(np.linalg.norm(err)) < tol:
            break
        api.move(target + np.clip(err, -0.04, 0.04), seconds=corr_s)
    return float(np.linalg.norm(target - api.eef()))


def pick(api, bowl, attempt):
    """One rim-straddle attempt: descend, close, lift.  -> gap after the lift."""
    ztop = bowl["ztop"]
    gx, gy = bowl["cx"], bowl["cy"] + bowl["r"]
    ztarget = ztop + GRASP_DZ - 0.006 * attempt
    api.move([gx, gy, ztop + 0.05], seconds=1.5)
    res = goto(api, [gx, gy, ztarget], seconds=1.0, passes=BIAS_PASSES)
    e = api.eef()
    api.log("attempt%d goal=(%.3f,%.3f,%.3f) eef=%s res=%.4f dz_ach=%+.4f"
            % (attempt, gx, gy, ztarget, np.round(e, 4).tolist(), res,
               float(e[2]) - ztop))
    api.grip(0.0)
    g = api.gripper()
    api.move([float(e[0]), float(e[1]), SAFE_Z], seconds=1.5)
    g2 = api.gripper()
    api.log("attempt%d gap_close=%.4f eff=%.1f gap_lift=%.4f eff2=%.1f"
            % (attempt, g["width_m"], g["effort"], g2["width_m"], g2["effort"]))
    return g["width_m"], g2["width_m"]


def run(api):
    api.log("instruction: %s" % api.instruction())
    W, m = _world(api.capture("cam_high"))
    bowl, plate = find_bowl(W, m), find_plate(W, m)
    api.log("bowl=%s plate=%s" % (bowl, plate))
    if bowl is None or plate is None:
        return "perception failed"

    held, cx0 = False, bowl["cx"]
    for attempt in range(3):
        _, g2 = pick(api, bowl, attempt)
        # The finger gap alone cannot separate "pinched the wall" from "closed
        # on the rim and let go on the way up" — both leave a few millimetres.
        # The decisive evidence is that the cabinet top is now empty, read from
        # a viewpoint the arm has stepped out of (and which is on the way to
        # the plate anyway).
        api.move([cx0, VERIFY_PARK_Y, SAFE_Z], seconds=1.5)
        W, m = _world(api.capture("cam_high"))
        b = find_bowl(W, m)
        api.log("attempt%d verify: cabinet bowl=%s" % (attempt, b))
        if b is None and g2 > VERIFY_GAP:
            held = True
            break
        api.grip(0.08)
        if b is None:
            api.log("bowl left the cabinet top but is not in the hand")
            return "bowl lost"
        bowl = b

    if not held:
        api.log("no grasp")
        return "grasp failed"

    # transport: the bowl hangs from the pinch point, so its centre trails the
    # tool centre by one rim radius in -y.
    px, py = plate["cx"], plate["cy"] + bowl["r"]
    api.move([px, py, SAFE_Z], seconds=1.5)
    res = goto(api, [px, py, RELEASE_Z], seconds=1.5, passes=1, tol=0.008)
    e = api.eef()
    g3 = api.gripper()
    api.log("release at %s res=%.4f gap=%.4f eff=%.1f"
            % (np.round(e, 4).tolist(), res, g3["width_m"], g3["effort"]))
    api.grip(0.08)
    api.settle(0.3)
    api.move([px, py, 1.10], seconds=1.0)
    return "placed"
