"""put the wine bottle in the bowl  --  v1

Perception (cam_high RGB-D only):
  * table plane  = mode of the height histogram over the central table patch
  * wine bottle  = the dark green-tinted pixel cluster.  It is a vertical
                   cylinder, so in every horizontal band its centre is
                   (x_max - r, y_mid); the body / shoulder / neck bands agree
                   to under a millimetre, which is the check that the cluster
                   really is the bottle.
  * bowl         = the single connected component in the [table+43, table+62] mm
                   height band whose footprint is 90-140 mm square.

Motion: grasp the bottle by its neck (the closed finger gap reports which part
of the bottle was caught, so a mis-aimed close can be retried), lift, re-perceive
to measure how far the bottle base hangs below the end effector, carry it over
the bowl above the rim, lower until the base is inside the bowl, release.
"""
import numpy as np

PROVENANCE = {
    "GREEN_MAX": {"source": "debug-seed RGB: bottle pixels read [1,6,0]; every "
                            "other dark prop in the scene is neutral grey",
                  "allowed": True},
    "BAND_BODY": {"source": "debug-seed depth: bottle radius 20.2 mm over "
                            "table+30..65 mm (15/15 seeds)", "allowed": True},
    "BAND_SHOULDER": {"source": "debug-seed depth: radius 15.5 mm over "
                                "table+80..95 mm (15/15 seeds)", "allowed": True},
    "BAND_NECK": {"source": "debug-seed depth: radius 6.9 mm over "
                            "table+115..142 mm (15/15 seeds)", "allowed": True},
    "BOTTLE_TOP": {"source": "debug-seed depth: bottle apex table+147..148 mm",
                   "allowed": True},
    "NECK_GRASP_EEF": {"source": "debug-seed depth profile: puts the fingers in "
                                 "BAND_NECK for any fingertip offset in "
                                 "-30..0 mm", "allowed": True},
    "GAP_NECK": {"source": "debug-seed depth: neck diameter 13.8 mm, shoulder "
                           "31 mm, body 40 mm -- the closed gap says which was "
                           "caught", "allowed": True},
    "BOWL_BAND": {"source": "debug-seed depth: bowl rim table+51 mm, footprint "
                            "108 mm across; unique component in the band on all "
                            "15 seeds", "allowed": True},
    "BOWL_RIM": {"source": "debug-seed depth: rim table+51 mm", "allowed": True},
    "BASE_DROP": {"source": "debug-seed depth: bowl interior floor table+6 mm",
                  "allowed": True},
    "POS_TOL": {"source": "generic controller mechanics: move returns once its "
                          "residual is under tolerance, so free-space waypoints "
                          "are re-commanded with the measured bias",
                "allowed": True},
}

GREEN_MAX = 40
BAND_BODY = (0.030, 0.065)
BAND_SHOULDER = (0.080, 0.095)
BAND_NECK = (0.115, 0.142)
BOTTLE_TOP = 0.147
NECK_GRASP_EEF = 0.141
GAP_NECK = (0.008, 0.024)
BOWL_BAND = (0.043, 0.062)
BOWL_RIM = 0.051
BASE_DROP = 0.018
POS_TOL = 0.012


# ---------------------------------------------------------------- perception
def _cloud(f):
    d = np.asarray(f.depth, float)
    d = np.where(np.isfinite(d) & (d > 0.05), d, np.nan)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1)
    return P @ T.T


def _table(B):
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    m = np.isfinite(X) & (np.abs(X) < 0.20) & (np.abs(Y) < 0.30) & (Z > 0.5) & (Z < 1.6)
    z = Z[m]
    h, e = np.histogram(z, bins=550, range=(0.5, 1.6))
    k = int(np.argmax(h))
    return float(np.median(z[(z >= e[k] - 0.004) & (z <= e[k + 1] + 0.004)]))


def _green(B, rgb, tab, zmax=0.20):
    r, g, b = rgb[..., 0].astype(int), rgb[..., 1].astype(int), rgb[..., 2].astype(int)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    m = ((g > r) & (g > b) & (g < GREEN_MAX) & np.isfinite(X)
         & (Z > tab + 0.005) & (Z < tab + zmax)
         & (X > -0.45) & (X < 0.25) & (np.abs(Y) < 0.40))
    return B[m]


def _cyl_centre(P, tab, band):
    q = P[(P[:, 2] >= tab + band[0]) & (P[:, 2] < tab + band[1])]
    if len(q) < 20:
        return None
    r = 0.5 * (q[:, 1].max() - q[:, 1].min())
    return np.array([q[:, 0].max() - r, 0.5 * (q[:, 1].max() + q[:, 1].min()), r])


def find_bottle(B, rgb, tab):
    P = _green(B, rgb, tab)
    if len(P) < 100:
        return None
    top = P[P[:, 2] > tab + BAND_NECK[0]]
    if len(top) < 20:
        return None
    ax = top[:, 0].max() - 0.007
    ay = 0.5 * (top[:, 1].max() + top[:, 1].min())
    P = P[(np.abs(P[:, 0] - ax) < 0.05) & (np.abs(P[:, 1] - ay) < 0.05)]
    est = [e for e in (_cyl_centre(P, tab, bd)
                       for bd in (BAND_BODY, BAND_SHOULDER, BAND_NECK)) if e is not None]
    if not est:
        return None
    c = np.mean([e[:2] for e in est], axis=0)
    return dict(xy=c, top=float(P[:, 2].max()), base=float(P[:, 2].min()),
                spread=float(max(np.linalg.norm(e[:2] - c) for e in est)),
                radii=[float(e[2]) for e in est], n=int(len(P)))


def held_bottle(B, rgb, tab, near_xy):
    """The bottle once it is in the gripper: the green cluster within 8 cm of
    the tool, with the arm's own green links cut off by a height ceiling."""
    P = _green(B, rgb, tab, zmax=float(near_xy[2] - tab) - 0.02)
    if len(P) < 40:
        return None
    P = P[(np.abs(P[:, 0] - near_xy[0]) < 0.07) & (np.abs(P[:, 1] - near_xy[1]) < 0.07)]
    if len(P) < 40:
        return None
    return dict(base=float(np.percentile(P[:, 2], 1)), top=float(P[:, 2].max()),
                n=int(len(P)), xy=[float(np.median(P[:, 0])), float(np.median(P[:, 1]))])


def find_bowl(B, tab, res=0.006):
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    x0, x1, y0, y1 = -0.32, 0.20, -0.35, 0.35
    ok = (np.isfinite(X) & (X >= x0) & (X < x1) & (Y >= y0) & (Y < y1)
          & (Z > tab + BOWL_BAND[0]) & (Z < tab + BOWL_BAND[1]))
    nx, ny = int((x1 - x0) / res), int((y1 - y0) / res)
    grid = np.zeros((nx, ny), bool)
    ix = np.clip(((X - x0) / res).astype(int), 0, nx - 1)
    iy = np.clip(((Y - y0) / res).astype(int), 0, ny - 1)
    grid[ix[ok], iy[ok]] = True
    lab = np.zeros((nx, ny), int)
    cur = 0
    for i in range(nx):
        for j in range(ny):
            if grid[i, j] and lab[i, j] == 0:
                cur += 1
                st = [(i, j)]
                lab[i, j] = cur
                while st:
                    a, bq = st.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, bq + db
                            if 0 <= p < nx and 0 <= q < ny and grid[p, q] and lab[p, q] == 0:
                                lab[p, q] = cur
                                st.append((p, q))
    cands = []
    for c in range(1, cur + 1):
        ii, jj = np.nonzero(lab == c)
        if len(ii) < 30:
            continue
        xs = x0 + (ii + 0.5) * res
        ys = y0 + (jj + 0.5) * res
        dx, dy = xs.max() - xs.min(), ys.max() - ys.min()
        if 0.09 < dx < 0.14 and 0.09 < dy < 0.14:
            cands.append(dict(xy=np.array([0.5 * (xs.min() + xs.max()),
                                           0.5 * (ys.min() + ys.max())]),
                              dx=float(dx), dy=float(dy), n=int(len(ii))))
    if not cands:
        return None
    cands.sort(key=lambda d: -d["n"])
    return cands[0]


# ------------------------------------------------------------------- motion
def precise(api, target, seconds=1.5, iters=2, tol=0.004):
    """move() stops as soon as its residual is inside POS_TOL, which leaves up
    to a centimetre on the table. Re-command with the measured bias."""
    target = np.asarray(target, float)
    bias = np.zeros(3)
    for _ in range(iters):
        api.move(target + bias, seconds=seconds)
        err = target - api.eef()
        if float(np.linalg.norm(err)) < tol:
            break
        bias = bias + err
        seconds = 1.0
    return float(np.linalg.norm(target - api.eef()))


def run(api):
    api.log("instruction: %s" % api.instruction())
    f = api.capture("cam_high")
    B = _cloud(f)
    rgb = np.asarray(f.rgb)
    tab = _table(B)
    bot = find_bottle(B, rgb, tab)
    bowl = find_bowl(B, tab)
    api.log("table=%.4f" % tab)
    api.log("bottle=%s" % (None if bot is None else
                           dict(xy=np.round(bot["xy"], 4).tolist(),
                                dtop=round(bot["top"] - tab, 4),
                                spread=round(bot["spread"], 4),
                                radii=[round(r, 4) for r in bot["radii"]],
                                n=bot["n"])))
    api.log("bowl=%s" % (None if bowl is None else
                         dict(xy=np.round(bowl["xy"], 4).tolist(),
                              dx=round(bowl["dx"], 3), dy=round(bowl["dy"], 3),
                              n=bowl["n"])))
    if bot is None or bowl is None:
        return "perception failed"

    bx, by = float(bot["xy"][0]), float(bot["xy"][1])
    wx, wy = float(bowl["xy"][0]), float(bowl["xy"][1])

    # --- grasp the neck -----------------------------------------------------
    api.grip(0.08)
    r = precise(api, [bx, by, tab + BOTTLE_TOP + 0.065], seconds=2.0)
    api.log("above bottle eef=%s res=%.4f" % (np.round(api.eef(), 4).tolist(), r))

    z_try = tab + NECK_GRASP_EEF
    gap = 0.0
    for attempt in range(3):
        precise(api, [bx, by, z_try], seconds=1.5, tol=0.003)
        api.log("grasp try %d eef=%s" % (attempt, np.round(api.eef(), 4).tolist()))
        api.grip(0.0)
        api.settle(0.2)
        g = api.gripper()
        gap = g["width_m"]
        api.log("try %d gap=%.4f effort=%.2f" % (attempt, gap, g["effort"]))
        if GAP_NECK[0] <= gap <= GAP_NECK[1]:
            break
        api.grip(0.08)
        z_try += -0.018 if gap < GAP_NECK[0] else 0.022
        z_try = min(max(z_try, tab + 0.10), tab + 0.16)

    # --- lift and measure how far the bottle hangs below the tool -----------
    lift_z = tab + 0.26
    precise(api, [bx, by, lift_z], seconds=2.0, tol=0.006)
    eef = api.eef()
    g = api.gripper()
    api.log("lifted eef=%s gap=%.4f effort=%.2f"
            % (np.round(eef, 4).tolist(), g["width_m"], g["effort"]))
    f2 = api.capture("cam_high")
    B2 = _cloud(f2)
    hb = held_bottle(B2, np.asarray(f2.rgb), tab, eef)
    hang = None
    if hb is not None:
        hang = float(eef[2] - hb["base"])
        api.log("held bottle=%s hang=%.4f"
                % (dict(base=round(hb["base"], 4), top=round(hb["top"], 4),
                        n=hb["n"], xy=[round(v, 4) for v in hb["xy"]]), hang))
    if hang is None or not (0.10 < hang < 0.20):
        hang = float(eef[2] - tab) - 0.115   # fallback: neck grasp geometry
        api.log("hang fallback -> %.4f" % hang)

    # --- carry over the bowl and lower the base into it ---------------------
    carry_z = tab + BOWL_RIM + 0.045 + hang
    r = precise(api, [wx, wy, max(carry_z, lift_z)], seconds=2.5, tol=0.005)
    api.log("over bowl eef=%s res=%.4f" % (np.round(api.eef(), 4).tolist(), r))
    drop_z = tab + BASE_DROP + hang
    api.move([wx, wy, drop_z], seconds=2.0)
    g = api.gripper()
    api.log("at drop eef=%s target_z=%.4f gap=%.4f effort=%.2f"
            % (np.round(api.eef(), 4).tolist(), drop_z, g["width_m"], g["effort"]))
    api.grip(0.08)
    api.settle(0.4)
    api.move([wx, wy, tab + 0.30], seconds=2.0)
    api.settle(0.6)

    f3 = api.capture("cam_high")
    B3 = _cloud(f3)
    b3 = find_bottle(B3, np.asarray(f3.rgb), tab)
    api.log("after: bottle=%s" % (None if b3 is None else
                                  dict(xy=np.round(b3["xy"], 4).tolist(),
                                       dtop=round(b3["top"] - tab, 4),
                                       dbase=round(b3["base"] - tab, 4))))
    api.log("bowl xy was %s" % np.round(bowl["xy"], 4).tolist())
    return "done"
