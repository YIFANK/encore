"""c2k1clean spa_bowl_on_wooden_cabinet_task_k1 -- v3.

Intent: "Pick the akita black bowl on the stove and place it on the plate".

Scene, measured on debug seeds 51/53/55/57 with v1/v2 (cam_high RGB-D):
three identical bowls at three rim heights above the measured table plane --
cabinet top ~+0.28, stove ~+0.077, bare table ~+0.042.  The height band alone
names the bowl the intent asks for.

Bowl: outer radius 0.044 m (measured on the isolated table bowl and on the
placed bowl, span 0.085-0.088 m).  The jaws open to ~0.080 m, so the bowl
cannot be straddled; both demo packs hold it at a finger gap of ~0.005 m,
i.e. pinched across the rim wall.  With the wrist straight down the jaws open
along base +/-y, so the pinch point is the rim's +y extreme.

v2 -> v3: the stove-bowl cluster is contaminated by the stove's own raised
ring (span 0.110 vs the bowl's 0.088), and in ep57 a dark background blob won
the largest-cluster vote.  v3 locates the bowl by fitting its known rim radius
with a centre grid search, and gates candidates on fit quality + workspace.

v3 -> v4: aim is not the problem -- the close lands on the rim wall at a gap
of 0.0117-0.0132 m for every rung of a 13 mm offset ladder.  Retention is the
problem: on seeds 53/55 the bowl was lifted clear of the stove and then fell
out, and on 53 it fell out again mid-transport.  The controller is a
proportional servo whose action saturates at 0.05 m of error, so every long
command (a 0.15 m lift, a 0.35 m transport) starts at full speed and jerks the
bowl out of a pinch that only holds ~5 mm of rim wall.  v4 GLIDES every
loaded motion in 0.025 m increments, which keeps the command below saturation
throughout, and verifies the lift by re-perception instead of by the finger
gap alone (0.0045 vs 0.0053 m is not a reliable held/dropped boundary).
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {
        "source": "debug seeds: dominant z mode of the cam_high depth cloud; "
                  "re-measured every episode, never hard-coded",
        "allowed": True},
    "STOVE_BAND": {
        "source": "debug seeds 51,53,55,57 cam_high: the three bowl rims sit at "
                  "+0.28 / +0.077 / +0.042 m above the measured table plane, so "
                  "(0.055,0.105) isolates the stove one",
        "allowed": True},
    "BOWL_R_M": {
        "source": "debug seeds 51/57 cam_high: the isolated table bowl and the "
                  "placed bowl measure span 0.085-0.088 m -> radius 0.044",
        "allowed": True},
    "GRASP_DEPTH_M": {
        "source": "demo packs' grasp keyframe EEF z below the rim top measured on "
                  "debug seeds (k1 t=50: 1.159 vs 1.179; mate t=43: 0.949 vs "
                  "0.979), confirmed by the v2 grasps on seeds 51/53/55",
        "allowed": True},
    "RIM_OFF_LADDER_M": {
        "source": "v2/v3 debug-seed grasps: the close lands on the rim wall at a "
                  "gap of 0.0117-0.0132 m for every rung of 0.026-0.039, and the "
                  "demo packs' grasp EEF minus bowl centre is ~+0.030; the ladder "
                  "brackets the 0.044 rim radius from inside",
        "allowed": True},
    "GLIDE_STEP_M": {
        "source": "generic controller mechanics: heron's LIBERO cartesian servo "
                  "sends clip(err/0.05) so any command longer than 0.05 m runs "
                  "saturated; 0.025 m increments stay at half command",
        "allowed": True},
    "HOVER_M": {"source": "debug seeds: rim top + 0.09 clears the stove fixture",
                "allowed": True},
    "CARRY_Z_M": {"source": "debug seeds: cabinet top 1.128, every other prop "
                            "below 0.98; 1.10 clears the transport corridor",
                  "allowed": True},
    "PLACE_CLEAR_M": {"source": "debug-seed measured hang (eef z minus the held "
                                "bowl's lowest point, 0.025 on seed 51) + margin",
                      "allowed": True},
    "PLATE_BAND": {"source": "debug seeds 51-57: the plate is the largest bright "
                             "cluster within 0.032 m of the table plane",
                   "allowed": True},
    "WORKSPACE": {"source": "debug-seed corner deprojections of cam_high: the "
                            "table surface spans x -0.45..0.30, y -0.40..0.40",
                  "allowed": True},
}

STOVE_BAND = (0.055, 0.105)
BOWL_R_M = 0.044
GRASP_DEPTH_M = 0.030
RIM_OFF_LADDER_M = (0.036, 0.030, 0.042)
GLIDE_STEP_M = 0.025
HOVER_M = 0.09
CARRY_Z_M = 1.10
PLACE_CLEAR_M = 0.015
PLATE_BAND = (0.006, 0.032)
WORKSPACE = (-0.45, 0.30, -0.40, 0.40)


# ---------------------------------------------------------------- perception
def xyz_image(frame):
    d = np.asarray(frame.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u = np.arange(w)[None, :].repeat(h, 0).astype(float)
    v = np.arange(h)[:, None].repeat(w, 1).astype(float)
    bad = ~np.isfinite(d) | (d <= 0)
    z = np.where(bad, np.nan, d)
    pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], -1)
    out = pc @ T[:3, :3].T + T[:3, 3]
    out[bad] = np.nan
    return out


def components(mask, min_px):
    lab = np.zeros(mask.shape, bool)
    out = []
    ys, xs = np.nonzero(mask)
    H, W = mask.shape
    for y0, x0 in zip(ys, xs):
        if lab[y0, x0]:
            continue
        lab[y0, x0] = True
        stack = [(y0, x0)]
        pix = []
        while stack:
            y, x = stack.pop()
            pix.append((y, x))
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < H and 0 <= nx < W and mask[ny, nx] and not lab[ny, nx]:
                    lab[ny, nx] = True
                    stack.append((ny, nx))
        if len(pix) >= min_px:
            out.append(np.array(pix))
    return out


def table_plane(P):
    z = P[..., 2]
    z = z[np.isfinite(z)]
    z = z[(z > 0.80) & (z < 1.05)]
    hist, edges = np.histogram(z, bins=np.arange(0.80, 1.05, 0.004))
    i = int(np.argmax(hist))
    sel = z[(z >= edges[i] - 0.006) & (z < edges[i + 1] + 0.006)]
    return float(np.median(sel))


def ring_fit(xy, r_nom=BOWL_R_M, tol=0.010):
    """Best circle centre of radius r_nom for a set of rim points.

    Grid search rather than least squares: the stove's own raised ring shares
    the band with the bowl rim and drags an algebraic fit outward, while a
    fixed-radius vote locks onto the bowl's own circle.  Points well inside
    the circle count against a centre, which is what separates the bowl rim
    from the (larger, concentric) stove ring.
    """
    c0 = np.array([(xy[:, 0].min() + xy[:, 0].max()) / 2.0,
                   (xy[:, 1].min() + xy[:, 1].max()) / 2.0])
    best, bestc = -1e9, c0
    grid = np.arange(-0.045, 0.0451, 0.003)
    for dx in grid:
        for dy in grid:
            c = c0 + (dx, dy)
            d = np.hypot(xy[:, 0] - c[0], xy[:, 1] - c[1])
            on = float(np.count_nonzero(np.abs(d - r_nom) < tol))
            inside = float(np.count_nonzero(d < r_nom - tol))
            s = on - 0.7 * inside
            if s > best:
                best, bestc = s, c
    d = np.hypot(xy[:, 0] - bestc[0], xy[:, 1] - bestc[1])
    on = float(np.count_nonzero(np.abs(d - r_nom) < tol))
    return bestc, on / max(1.0, float(len(xy))), int(on)


def cluster_stats(pix, P, rgb):
    ys, xs = pix[:, 0], pix[:, 1]
    p = P[ys, xs]
    ok = np.isfinite(p[:, 0])
    if ok.sum() < 20:
        return None
    p = p[ok]
    c = rgb[ys, xs].astype(float)[ok]
    top = float(np.percentile(p[:, 2], 95))
    ring = p[p[:, 2] > top - 0.008]
    if len(ring) < 25:
        ring = p[p[:, 2] > top - 0.015]
    if len(ring) < 15:
        return None
    ctr, frac, non = ring_fit(ring[:, :2])
    return dict(n=int(len(pix)), upx=float(xs.mean()), vpx=float(ys.mean()),
                top=top, mx=float(p[:, 0].mean()), my=float(p[:, 1].mean()),
                ex=float((ring[:, 0].min() + ring[:, 0].max()) / 2.0),
                ey=float((ring[:, 1].min() + ring[:, 1].max()) / 2.0),
                cx=float(ctr[0]), cy=float(ctr[1]), frac=frac, non=non,
                spanx=float(p[:, 0].max() - p[:, 0].min()),
                spany=float(p[:, 1].max() - p[:, 1].min()),
                lum=float(c.mean()))


def fmt(st):
    return ("n=%d px=(%.0f,%.0f) top=%.3f mean=(%.3f,%.3f) ext=(%.3f,%.3f) "
            "fit=(%.3f,%.3f) frac=%.2f on=%d span=(%.3f,%.3f) lum=%.0f"
            % (st["n"], st["upx"], st["vpx"], st["top"], st["mx"], st["my"],
               st["ex"], st["ey"], st["cx"], st["cy"], st["frac"], st["non"],
               st["spanx"], st["spany"], st["lum"]))


def in_workspace(x, y):
    return (WORKSPACE[0] <= x <= WORKSPACE[1]) and (WORKSPACE[2] <= y <= WORKSPACE[3])


def find_bowl(api, P, rgb, tz, tag="bowl"):
    h = P[..., 2] - tz
    band = np.isfinite(h) & (h > STOVE_BAND[0]) & (h < STOVE_BAND[1])
    best = None
    for pix in components(band, 150):
        st = cluster_stats(pix, P, rgb)
        if st is None:
            continue
        api.log("%s-cand %s" % (tag, fmt(st)))
        if not in_workspace(st["cx"], st["cy"]):
            continue
        if st["spanx"] > 0.20 or st["spany"] > 0.20 or st["n"] > 4500:
            continue
        if st["non"] < 40 or st["frac"] < 0.25:
            continue
        if best is None or st["non"] > best["non"]:
            best = st
    return best


def find_plate(api, P, rgb, tz):
    h = P[..., 2] - tz
    lum = np.asarray(rgb, float).mean(-1)
    pb = (np.isfinite(h) & (h > PLATE_BAND[0]) & (h < PLATE_BAND[1]) & (lum > 140))
    best = None
    for pix in components(pb, 150):
        st = cluster_stats(pix, P, rgb)
        if st is None:
            continue
        api.log("plate-cand %s" % fmt(st))
        if not in_workspace(st["ex"], st["ey"]):
            continue
        if best is None or st["n"] > best["n"]:
            best = st
    return best


# ------------------------------------------------------------------- motions
def gstate(api, tag):
    g = api.gripper()
    e = api.eef()
    api.log("%s eef=(%.4f,%.4f,%.4f) gap=%.4f eff=%.2f"
            % (tag, e[0], e[1], e[2], g["width_m"], g["effort"]))
    return g, e


def held(g):
    return g["effort"] >= 3.0 and g["width_m"] > 0.005


def glide(api, tgt, step=GLIDE_STEP_M, seconds=0.7, cap=40):
    """Walk to `tgt` in sub-saturation increments.

    The servo sends clip(err / 0.05), so a single long command runs at full
    commanded velocity from the first step and jerks a rim pinch open.  Each
    increment here is half of the saturation distance.
    """
    tgt = np.asarray(tgt, float)
    for _ in range(cap):
        cur = api.eef()
        d = tgt - cur
        n = float(np.linalg.norm(d))
        if n < 0.012:
            break
        if n <= step:
            api.move(tgt, seconds=seconds)
            break
        api.move(cur + d / n * step, seconds=seconds)
    return float(np.linalg.norm(tgt - api.eef()))


def payload_under(api, tz, e):
    """Depth evidence that something is hanging in the jaws: (n, bottom_z)."""
    f = api.capture("cam_high")
    P = xyz_image(f)
    d = np.hypot(P[..., 0] - e[0], P[..., 1] - e[1])
    near = (np.isfinite(P[..., 2]) & (d < 0.085)
            & (P[..., 2] < e[2] - 0.004) & (P[..., 2] > tz + 0.06))
    n = int(near.sum())
    if n < 40:
        return n, None, None, None
    q = P[near]
    return (n, float(np.percentile(q[:, 2], 2)),
            float(np.median(q[:, 0])) - e[0], float(np.median(q[:, 1])) - e[1])


def run(api):
    api.log("INSTRUCTION %r" % api.instruction())
    f = api.capture("cam_high")
    P = xyz_image(f)
    rgb = np.asarray(f.rgb)
    tz = table_plane(P)
    api.log("TABLE_Z=%.4f" % tz)

    bowl = find_bowl(api, P, rgb, tz)
    plate = find_plate(api, P, rgb, tz)
    if bowl is None or plate is None:
        api.log("ABORT bowl=%s plate=%s" % (bowl is not None, plate is not None))
        return
    api.log("BOWL %s" % fmt(bowl))
    api.log("PLATE %s" % fmt(plate))

    got, off_used, hang, payload_dy = False, None, 0.025, None
    for att, off in enumerate(RIM_OFF_LADDER_M):
        if att:                       # the bowl may have been nudged: re-look
            f = api.capture("cam_high")
            P = xyz_image(f)
            rgb = np.asarray(f.rgb)
            b2 = find_bowl(api, P, rgb, tz, tag="re")
            if b2 is not None:
                bowl = b2
                api.log("REBOWL %s" % fmt(b2))
        bx, by, btop = bowl["cx"], bowl["cy"], bowl["top"]
        grasp_z = btop - GRASP_DEPTH_M
        api.log("--- attempt %d off=%.3f xy=(%.3f,%.3f) top=%.3f gz=%.3f"
                % (att, off, bx, by, btop, grasp_z))
        api.grip(0.08)
        r = api.move([bx, by + off, btop + HOVER_M], seconds=2.0)
        api.log("hover residual=%.4f" % r)
        r = glide(api, [bx, by + off, grasp_z], step=0.03)
        api.log("descend residual=%.4f" % r)
        gstate(api, "at-grasp")
        api.grip(0.0)
        g, _ = gstate(api, "closed")
        if g["width_m"] <= 0.005:
            api.log("closed on air, next rung")
            continue
        r = glide(api, [bx, by + off, btop + 0.11])
        api.log("lift residual=%.4f" % r)
        g, e = gstate(api, "lifted")
        n, bot, dx, dy = payload_under(api, tz, e)
        api.log("lift-payload n=%d bottom=%s dx=%s dy=%s"
                % (n, None if bot is None else round(bot, 3),
                   None if dx is None else round(dx, 3),
                   None if dy is None else round(dy, 3)))
        carrying = held(g) or (bot is not None and bot > tz + 0.045)
        if carrying:
            got, off_used = True, off
            hang = 0.025 if bot is None else float(e[2] - bot)
            payload_dy = dy
            break
        api.grip(0.08)
        api.move([bx, by + off, btop + 0.15], seconds=1.5)

    api.log("GRASPED=%s off=%s" % (got, off_used))
    if not got:
        return
    if not (0.0 < hang < 0.10):
        api.log("hang fallback (was %.3f)" % hang)
        hang = 0.025
    api.log("hang=%.3f payload_dy=%s" % (hang, payload_dy))

    # --- transport and release ---------------------------------------------
    px, py, ptop = plate["ex"], plate["ey"], plate["top"]
    rel_z = ptop + hang + PLACE_CLEAR_M
    # put the BOWL over the plate centre, not the wrist: the bowl hangs off to
    # -y because it is pinched on its +y rim.
    aim_y = py + off_used
    api.log("plate xy=(%.3f,%.3f) top=%.3f rel_z=%.3f aim_y=%.3f"
            % (px, py, ptop, rel_z, aim_y))

    e = api.eef()
    r = glide(api, [e[0], e[1], CARRY_Z_M])
    api.log("raise residual=%.4f" % r)
    r = glide(api, [px, aim_y, CARRY_Z_M])
    api.log("over-plate residual=%.4f" % r)
    g, _ = gstate(api, "over-plate")
    r = glide(api, [px, aim_y, rel_z])
    api.log("lower residual=%.4f" % r)
    gstate(api, "lowered")
    api.grip(0.08)
    api.settle(0.6)
    gstate(api, "released")
    glide(api, [px, aim_y, ptop + 0.16])
    api.settle(0.8)

    # --- post-check ---------------------------------------------------------
    f3 = api.capture("cam_high")
    P3 = xyz_image(f3)
    rgb3 = np.asarray(f3.rgb)
    h3 = P3[..., 2] - tz
    band = np.isfinite(h3) & (h3 > 0.020) & (h3 < 0.12)
    for pix in components(band, 150):
        st = cluster_stats(pix, P3, rgb3)
        if st is not None and st["spanx"] < 0.25:
            api.log("post %s" % fmt(st))
    api.log("DONE")
