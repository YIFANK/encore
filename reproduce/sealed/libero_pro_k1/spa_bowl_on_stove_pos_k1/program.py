"""c2k1clean / spa_bowl_on_stove_pos_k1

"pick up the black bowl on the stove and place it on the plate"

Method
------
Everything metric is re-derived per episode from one cam_high RGB-D frame:

  * the table plane is the mode of the height histogram;
  * the target bowl is the compact cluster whose top lies in the band a bowl
    standing on the RAISED stove occupies (table+0.055 .. table+0.125) -- a
    bowl standing on the table tops out at table+0.043 and the bowl on the
    cabinet at table+0.32, so the support height is what names the target,
    which is exactly what the instruction says ("on the stove");
  * the plate is the large round low disc.

The bowl (90 mm across) is wider than the jaws open (80 mm), so it is taken
by a WALL PINCH: the tool point is put one bowl radius off centre along the
jaw axis (world y with the wrist straight down), so one jaw drops inside the
bowl and one outside, and closing traps the wall. The demonstration's own
grasp height -- 30.5 mm below the rim -- is reused.

Because the tool point is at the wall and not at the bowl's centre, the held
bowl's centre stays offset from the tool point by that same vector; the
release aims plate_centre MINUS that offset. The final descent is commanded
below the plate so it is stopped by the bowl touching down rather than by a
position target, which absorbs the unknown hang length.
"""
import numpy as np

PROVENANCE = {
    "GRASP_BELOW_RIM": {
        "source": "pack demo0 keyframe t=43 (grasp) ee z = 0.9488 minus the "
                  "bowl rim top measured on debug seeds 51-65 (table+0.0783) "
                  "= 0.0305 m",
        "allowed": True},
    "RELEASE_ABOVE_PLATE": {
        "source": "pack demo0 keyframe t=154 (release) ee z = 0.9306 minus the "
                  "plate rim top measured on debug seeds (table+0.0187) "
                  "= 0.0119 m, rounded to 0.0125; the descent is contact-"
                  "limited so this is only an upper bound",
        "allowed": True},
    "BOWL_TOP_BAND": {
        "source": "debug-seed measurement (v3 probe, seeds 51-65): bowl on the "
                  "stove tops out at table+0.078, free bowl on the table at "
                  "table+0.0427, cabinet top at table+0.278",
        "allowed": True},
    "RIM_BAND": {
        "source": "debug-seed measurement: bowl wall height 0.0427 m, so cells "
                  "within 0.016 m of the cluster top are rim, not interior",
        "allowed": True},
    "PLATE_SIZE_BAND": {
        "source": "debug-seed measurement: plate footprint 0.140 x 0.140 m, "
                  "top table+0.0187; cookie box 0.088 x 0.064, stove-top "
                  "fragments 0.14 x 0.055",
        "allowed": True},
    "HOVER_H / LIFT_H / CARRY_H": {
        "source": "clearances chosen above the measured stove top "
                  "(table+0.025) and below the measured cabinet top "
                  "(table+0.278)",
        "allowed": True},
    "HELD_MIN_WIDTH": {
        "source": "debug-seed measurement: jaws closed on air report "
                  "width 0.0018 m, closed on the bowl wall 0.012-0.016 m",
        "allowed": True},
    "CELL": {"source": "chosen grid resolution, not calibrated", "allowed": True},
}

CELL = 0.004
X0, X1 = -0.55, 0.42
Y0, Y1 = -0.58, 0.58

GRASP_BELOW_RIM = 0.0305
RELEASE_ABOVE_PLATE = 0.0125
RIM_BAND = 0.016
HOVER_H = 0.09
LIFT_H = 0.11
CARRY_H = 0.15
HELD_MIN_WIDTH = 0.0035
RETRY_DR = (0.0, -0.010, 0.010)


# ---------------------------------------------------------------- perception
def _cloud(f):
    d = np.asarray(f.depth, float)
    h, w = d.shape
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    K = np.asarray(f.intrinsics, float)
    x = (us - K[0, 2]) * d / K[0, 0]
    y = (vs - K[1, 2]) * d / K[1, 1]
    return np.stack([x, y, d, np.ones_like(d)], -1) @ np.asarray(f.t_base_cam, float).T


def _label(mask):
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    for i0, j0 in np.argwhere(mask):
        if lab[i0, j0]:
            continue
        cur += 1
        st = [(i0, j0)]
        lab[i0, j0] = cur
        while st:
            a, b = st.pop()
            for p, q in ((a - 1, b), (a + 1, b), (a, b - 1), (a, b + 1),
                         (a - 1, b - 1), (a - 1, b + 1),
                         (a + 1, b - 1), (a + 1, b + 1)):
                if 0 <= p < h and 0 <= q < w and mask[p, q] and not lab[p, q]:
                    lab[p, q] = cur
                    st.append((p, q))
    return lab, cur


def scene(api):
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb).astype(float)
    P = _cloud(f)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = np.isfinite(Z) & (Z > 0.5) & (Z < 1.6)
    hist, edges = np.histogram(Z[m], bins=550, range=(0.5, 1.6))
    k = int(np.argmax(hist))
    tz = float(0.5 * (edges[k] + edges[k + 1]))
    nx = int((X1 - X0) / CELL)
    ny = int((Y1 - Y0) / CELL)
    H = np.full((nx, ny), -9.0)
    C = np.zeros((nx, ny, 3))
    ix = ((X - X0) / CELL).astype(int)
    iy = ((Y - Y0) / CELL).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & np.isfinite(Z)
    a, b, z, c = ix[ok], iy[ok], Z[ok], rgb[ok]
    o = np.argsort(z)
    H[a[o], b[o]] = z[o]
    C[a[o], b[o]] = c[o]
    return tz, H, C


def clusters(H, C, zlo, zhi, minc=40):
    lab, n = _label((H > zlo) & (H < zhi))
    out = []
    for j in range(1, n + 1):
        s = lab == j
        if int(s.sum()) < minc:
            continue
        aa, bb = np.where(s)
        out.append(dict(n=int(s.sum()), s=s,
                        x=float(aa.mean() * CELL + X0),
                        y=float(bb.mean() * CELL + Y0),
                        wx=float((aa.max() - aa.min() + 1) * CELL),
                        wy=float((bb.max() - bb.min() + 1) * CELL),
                        top=float(np.percentile(H[s], 97)),
                        col=C[s].mean(0)))
    return out


def _rim(H, c):
    ring = c["s"] & (H > c["top"] - RIM_BAND)
    aa, bb = np.where(ring)
    return dict(x=float(aa.mean() * CELL + X0),
                y=float(bb.mean() * CELL + Y0),
                r=float((bb.max() - bb.min() + 1) * CELL / 2.0),
                top=c["top"], n=int(ring.sum()))


def find_bowl(api, tz, H, C, tag=""):
    """The bowl standing on the stove."""
    pool = [c for c in clusters(H, C, tz + 0.030, tz + 0.20)
            if -0.40 <= c["x"] <= 0.35
            and 0.06 <= c["wx"] <= 0.17 and 0.06 <= c["wy"] <= 0.17]
    cand = [c for c in pool if tz + 0.055 <= c["top"] <= tz + 0.125]
    if not cand:                      # fall back: highest compact vessel
        cand = [c for c in pool if c["top"] <= tz + 0.16]
    api.log("%sbowl cand=%d/%d" % (tag, len(cand), len(pool)))
    if not cand:
        return None
    c = max(cand, key=lambda d: d["top"])
    b = _rim(H, c)
    api.log("%sbowl xy=(%.4f,%.4f) r=%.4f top=+%.4f npx=%d"
            % (tag, b["x"], b["y"], b["r"], b["top"] - tz, b["n"]))
    return b


def find_plate(api, tz, H, C, tag=""):
    pool = [c for c in clusters(H, C, tz + 0.008, tz + 0.030)
            if 0.10 <= c["wx"] <= 0.22 and 0.10 <= c["wy"] <= 0.22
            and abs(c["wx"] - c["wy"]) <= 0.05]
    bright = [c for c in pool if c["col"].mean() >= 110
              and (c["col"].max() - c["col"].min()) <= 30]
    cand = bright or pool
    api.log("%splate cand=%d/%d" % (tag, len(cand), len(pool)))
    if not cand:
        return None
    c = max(cand, key=lambda d: d["n"])
    api.log("%splate xy=(%.4f,%.4f) w=(%.3f,%.3f) top=+%.4f"
            % (tag, c["x"], c["y"], c["wx"], c["wy"], c["top"] - tz))
    return dict(x=c["x"], y=c["y"], top=c["top"])


# ------------------------------------------------------------------- motion
def mv(api, tag, xyz, seconds):
    xyz = [float(v) for v in xyz]
    r = api.move(xyz, seconds=seconds)
    e = api.eef()
    api.log("%s cmd=%s got=%s res=%.4f"
            % (tag, [round(v, 4) for v in xyz], np.round(e, 4).tolist(), r))
    return e


def run(api):
    tz, H, C = scene(api)
    api.log("tz=%.4f instr=%r" % (tz, api.instruction()))
    bowl = find_bowl(api, tz, H, C)
    plate = find_plate(api, tz, H, C)
    if bowl is None or plate is None:
        api.log("ABORT bowl=%s plate=%s" % (bowl is not None, plate is not None))
        return

    api.grip(0.08)
    held, egrasp, bx, by = False, None, bowl["x"], bowl["y"]
    for att, dr in enumerate(RETRY_DR):
        gx = bx
        gy = by + bowl["r"] + dr
        gz = bowl["top"] - GRASP_BELOW_RIM
        api.log("attempt %d grasp=(%.4f,%.4f,%.4f)" % (att, gx, gy, gz))
        mv(api, "hover", [gx, gy, tz + HOVER_H], 1.2 if att == 0 else 0.8)
        egrasp = mv(api, "descend", [gx, gy, gz], 0.7)
        api.grip(0.0)
        w = api.gripper()["width_m"]
        api.log("close width=%.5f" % w)
        if w >= HELD_MIN_WIDTH:
            held = True
            break
        api.log("attempt %d: jaws closed on air" % att)
        api.grip(0.08)
        mv(api, "back", [gx, gy, tz + HOVER_H], 0.6)
        tz, H, C = scene(api)
        b2 = find_bowl(api, tz, H, C, "retry ")
        if b2 is not None:
            bowl, bx, by = b2, b2["x"], b2["y"]

    if not held:
        api.log("GRASP_FAILED after %d attempts" % len(RETRY_DR))
        return

    # the held bowl's centre relative to the tool point
    offx = bowl["x"] - egrasp[0]
    offy = bowl["y"] - egrasp[1]
    api.log("carry offset=(%.4f,%.4f)" % (offx, offy))

    mv(api, "lift", [egrasp[0], egrasp[1], tz + LIFT_H], 0.8)

    tx, ty = plate["x"] - offx, plate["y"] - offy
    # keep the carried bowl clear of the cabinet if the plate sits behind it
    if ty < -0.05:
        mv(api, "skirt", [egrasp[0], ty, tz + CARRY_H], 1.0)
    e = mv(api, "transport", [tx, ty, tz + CARRY_H], 1.5)
    # cancel the tracking bias that the transport just showed
    e = mv(api, "place", [tx + (tx - e[0]), ty + (ty - e[1]),
                          plate["top"] + RELEASE_ABOVE_PLATE], 0.9)
    api.log("pre-open grip=%s" % api.gripper())
    api.grip(0.08)
    api.log("open grip=%s" % api.gripper())
    mv(api, "up", [e[0], e[1], tz + CARRY_H], 0.6)
    api.log("RUN_DONE")
