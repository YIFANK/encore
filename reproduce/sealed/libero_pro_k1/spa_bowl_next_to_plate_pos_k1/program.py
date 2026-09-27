"""c2k1clean / spa_bowl_next_to_plate_pos_k1 -- v3 (TARGET_SEL=near).

Derivation (pack + debug seeds 51,53,55,57):
  * table top z = 0.9010 (modal world z of the dominant surface, cam_high).
  * Scene: dark cabinet; three vessels -- two large (rim-fit r=0.053,
    top 0.050 above the table) and one small (r=0.043, top 0.042); a bright
    flat disc (rim-fit r=0.057, top 0.019); a flat brown box (top 0.019).
  * Fitting each vessel's top band with a least-squares circle (Kasa) gives an
    unbiased rim centre; the cluster MEDIAN is biased ~0.03 m by the oblique
    view and must not be used for aiming.
  * Pack grasp keyframe t=53 ee=(0.0084,0.254,0.9226).  Against the rim fit of
    the large vessel nearest the disc, (0.008,0.309,r=0.053), that is
    centre + (0.000,-0.055,-) : a rim pinch at the -y extreme, jaws along y,
    wrist straight down, 0.0284 below the rim top.
  * Pack release keyframe t=109 ee=(0.0584,0.1525,0.9355).  Add back the same
    +y rim offset and the vessel lands at (0.058,0.208) -- the disc centre in
    the pack's own scene, (0.070,0.189), recovered from keyframes/demo0_t0000
    by deprojecting its bright blob onto the table plane.
"""
import numpy as np

TARGET_SEL = "near"   # near | far | small

PROVENANCE = {
    "RIM_DROP": {"source": "pack.json demo0 t=53 ee z 0.9226 vs the target "
                           "vessel's rim top (table 0.9010 + 0.050 measured "
                           "from cam_high depth on debug seed 51)",
                 "allowed": True},
    "RIM_EXCESS": {"source": "pack.json demo0 t=53 ee xy vs the same vessel's "
                             "Kasa rim fit on debug seed 51: |offset| 0.055 "
                             "against fitted r 0.053", "allowed": True},
    "PINCH_DIR": {"source": "pack.json demo0 t=53 ee xy minus the fitted rim "
                            "centre = (0.000,-0.055): the -y rim extreme",
                  "allowed": True},
    "PLACE_RISE": {"source": "pack.json demo0 t=109 ee z 0.9355 vs the disc "
                             "top (table 0.9010 + 0.019 measured from depth)",
                   "allowed": True},
    "CARRY_Z": {"source": "pack.json demo0 ee_path6 t=80 apex z=1.0501",
                "allowed": True},
    "HELD_GAP": {"source": "pack.json demo0 t=109 gripper_state gap "
                           "0.0037+0.0031=0.0068 m while carrying",
                 "allowed": True},
    "LARGE_R_CUT": {"source": "debug-seed cam_high depth: two vessels fit "
                              "r=0.053 and one fits r=0.043", "allowed": True},
    "VESSEL_H_BAND": {"source": "debug-seed cam_high depth: vessel tops 0.042 "
                                "and 0.050 above the table; the flat disc and "
                                "the brown box top out at 0.019",
                      "allowed": True},
    "DISC_LUM_MIN": {"source": "debug-seed cam_high rgb: the flat disc means "
                               "lum 141-154, the brown box 68",
                     "allowed": True},
    "TABLE_Z": {"source": "debug-seed cam_high depth, modal surface z",
                "allowed": True},
    "WORKSPACE_BOX": {"source": "debug-seed cam_high depth extent of the table",
                      "allowed": True},
}

RIM_DROP = 0.0284
RIM_EXCESS = 0.002
PLACE_RISE = 0.0155
CARRY_Z = 1.0501
HELD_LO, HELD_HI = 0.0040, 0.0260
LARGE_R_CUT = 0.048
DISC_LUM_MIN = 120.0


# ---------------------------------------------------------------- perception
def world_maps(f):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.asarray(f.depth, float)
    h, w = d.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    pc = np.stack([(uu - cx) * d / fx, (vv - cy) * d / fy, d,
                   np.ones_like(d)], axis=-1)
    wd = pc @ T.T
    return wd[..., 0], wd[..., 1], wd[..., 2]


def table_height(Z, lo=0.5, hi=1.3):
    zs = Z[np.isfinite(Z) & (Z > lo) & (Z < hi)]
    hist, edges = np.histogram(zs, bins=400, range=(lo, hi))
    i = int(np.argmax(hist))
    c = 0.5 * (edges[i] + edges[i + 1])
    return float(np.median(zs[np.abs(zs - c) < 0.01]))


def label_mask(mask):
    try:
        from scipy import ndimage
        return ndimage.label(mask)
    except Exception:
        pass
    h, w = mask.shape
    lab = np.where(mask, np.arange(h * w).reshape(h, w) + 1, 0)
    for _ in range(600):
        prev = lab
        m = lab.copy()
        m[1:, :] = np.maximum(m[1:, :], lab[:-1, :] * mask[1:, :])
        m[:-1, :] = np.maximum(m[:-1, :], lab[1:, :] * mask[:-1, :])
        m[:, 1:] = np.maximum(m[:, 1:], lab[:, :-1] * mask[:, 1:])
        m[:, :-1] = np.maximum(m[:, :-1], lab[:, 1:] * mask[:, :-1])
        lab = m * mask
        if np.array_equal(lab, prev):
            break
    ids = [i for i in np.unique(lab) if i]
    out = np.zeros_like(lab)
    for k, i in enumerate(ids, 1):
        out[lab == i] = k
    return out, len(ids)


def kasa(xs, ys):
    A = np.stack([2 * xs, 2 * ys, np.ones_like(xs)], axis=1)
    sol, *_ = np.linalg.lstsq(A, xs ** 2 + ys ** 2, rcond=None)
    cx, cy, c = sol
    return float(cx), float(cy), float(np.sqrt(max(c + cx * cx + cy * cy, 1e-9)))


def cluster_list(X, Y, Z, rgb, mask, min_px=60):
    lab, n = label_mask(mask)
    out = []
    for i in range(1, n + 1):
        m = lab == i
        npx = int(m.sum())
        if npx < min_px:
            continue
        xs, ys, zs = X[m], Y[m], Z[m]
        zmax = float(np.percentile(zs, 98))
        band = m & (Z > zmax - 0.010)
        if int(band.sum()) >= 40:
            fx_, fy_, fr = kasa(X[band], Y[band])
        else:
            fx_, fy_ = float(np.median(xs)), float(np.median(ys))
            fr = float(max(xs.max() - xs.min(), ys.max() - ys.min()) / 2)
        out.append({"n": npx, "mx": float(np.median(xs)), "my": float(np.median(ys)),
                    "zmax": zmax, "fx": fx_, "fy": fy_, "fr": fr,
                    "rx": float(0.5 * (xs.max() - xs.min())),
                    "ry": float(0.5 * (ys.max() - ys.min())),
                    "lum": float(rgb[m].mean())})
    return out


def perceive(api, tz=None):
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb, float)
    X, Y, Z = world_maps(f)
    if tz is None:
        tz = table_height(Z)
    box = np.isfinite(Z) & (X > -0.32) & (X < 0.42) & (np.abs(Y) < 0.42)
    h = Z - tz
    cl = cluster_list(X, Y, Z, rgb, box & (h > 0.010) & (h < 0.22))
    fl = cluster_list(X, Y, Z, rgb, box & (h > 0.003) & (h < 0.020), min_px=200)
    return tz, cl, fl


def log_clusters(api, tag, cl, tz):
    buf = ""
    for i, c in enumerate(sorted(cl, key=lambda c: -c["n"])[:10]):
        s = ("#%d n=%d m=(%.3f,%.3f) fit=(%.3f,%.3f,%.3f) h=%.3f lum=%.0f | "
             % (i, c["n"], c["mx"], c["my"], c["fx"], c["fy"], c["fr"],
                c["zmax"] - tz, c["lum"]))
        if len(buf) + len(s) > 1500:
            api.log(tag + " " + buf)
            buf = ""
        buf += s
    if buf:
        api.log(tag + " " + buf)


# ---------------------------------------------------------------------------
def pick_target(vessels, disc, sel):
    large = [c for c in vessels if c["fr"] >= LARGE_R_CUT]
    small = [c for c in vessels if c["fr"] < LARGE_R_CUT]

    def d(c):
        return float(np.hypot(c["fx"] - disc["fx"], c["fy"] - disc["fy"]))
    if sel == "small" and small:
        return min(small, key=lambda c: -c["n"])
    if not large:
        return None
    large.sort(key=d)
    if sel == "far":
        return large[-1]
    return large[0]


def run(api):
    api.log("v3(%s) %r" % (TARGET_SEL, api.instruction()))
    tz, cl, fl = perceive(api)
    api.log("table_z=%.4f eef0=%s" % (tz, np.round(api.eef(), 4).tolist()))
    log_clusters(api, "CL", cl, tz)
    log_clusters(api, "FLAT", fl, tz)

    discs = [c for c in fl if c["lum"] > DISC_LUM_MIN
             and 0.045 < c["fr"] < 0.12]
    vessels = [c for c in cl if 0.030 < c["fr"] < 0.075
               and 0.030 < c["zmax"] - tz < 0.090]
    api.log("cands discs=%d vessels=%d" % (len(discs), len(vessels)))
    if not discs or not vessels:
        api.log("ABORT: missing candidate")
        return
    disc = max(discs, key=lambda c: c["n"])
    api.log("disc fit=(%.3f,%.3f,r=%.3f) top=%.3f n=%d"
            % (disc["fx"], disc["fy"], disc["fr"], disc["zmax"] - tz, disc["n"]))
    for c in vessels:
        api.log("vessel fit=(%.3f,%.3f,r=%.3f) h=%.3f d_disc=%.3f n=%d"
                % (c["fx"], c["fy"], c["fr"], c["zmax"] - tz,
                   float(np.hypot(c["fx"] - disc["fx"], c["fy"] - disc["fy"])),
                   c["n"]))
    tgt = pick_target(vessels, disc, TARGET_SEL)
    if tgt is None:
        api.log("ABORT: no target for sel=%s" % TARGET_SEL)
        return
    api.log("TARGET fit=(%.3f,%.3f,r=%.3f) top=%.3f"
            % (tgt["fx"], tgt["fy"], tgt["fr"], tgt["zmax"] - tz))

    r = tgt["fr"] + RIM_EXCESS
    grasp_z = tgt["zmax"] - RIM_DROP
    ladder = [(0.0, -r), (0.0, -(r + 0.008)), (0.0, -(r - 0.008))]

    held_off = None
    for k, off in enumerate(ladder):
        gx, gy = tgt["fx"] + off[0], tgt["fy"] + off[1]
        api.grip(0.08)
        api.move([gx, gy, CARRY_Z], seconds=2.5)
        res = api.move([gx, gy, grasp_z], seconds=2.5)
        api.grip(0.0)
        g1 = api.gripper()
        api.move([gx, gy, CARRY_Z], seconds=2.5)
        g2 = api.gripper()
        api.log("try%d aim=(%.3f,%.3f,%.3f) res=%.4f gap=%.4f/%.4f eff=%.2f"
                % (k, gx, gy, grasp_z, res, g1["width_m"], g2["width_m"],
                   g2["effort"]))
        if HELD_LO < g2["width_m"] < HELD_HI:
            held_off = off
            break
        api.grip(0.08)
    if held_off is None:
        api.log("ABORT: nothing held")
        return

    rx = disc["fx"] + held_off[0]
    ry = disc["fy"] + held_off[1]
    rz = disc["zmax"] + PLACE_RISE
    api.move([rx, ry, CARRY_Z], seconds=3.0)
    res = api.move([rx, ry, rz], seconds=2.5)
    g = api.gripper()
    api.log("release aim=(%.3f,%.3f,%.3f) res=%.4f gap=%.4f"
            % (rx, ry, rz, res, g["width_m"]))
    api.grip(0.08)
    api.settle(0.5)
    api.move([rx, ry, CARRY_Z], seconds=2.0)
    api.move([rx, ry - 0.10, CARRY_Z], seconds=2.0)

    _, cl2, _ = perceive(api, tz)
    log_clusters(api, "POST", cl2, tz)
