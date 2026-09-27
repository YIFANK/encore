"""c2k1clean obj_butter_pos_k1 -- v5.

v4 (8/8 on the probe subset) plus two guards that are no-ops on the debug
layouts and only bite if an unseen layout differs:
  * jaw yaw aligned to the butter's measured minor footprint axis, applied
    only when that axis is more than YAW_DEADBAND off the default jaw axis;
  * a sensed grasp check (gripper effort/width) with a bounded retry ladder.
"""
import os

import numpy as np

PROVENANCE = {
    # --- perception thresholds -------------------------------------------
    "ABOVE_TABLE_M": {
        "source": "debug 51-58 depth: table plane z=0.0015 (histogram mode); "
                  "0.008 m clears the plane noise and keeps every object blob",
        "allowed": True},
    "MIN_BLOB_PX": {
        "source": "debug 51-58 blob sizes: smallest real object 865 px, "
                  "largest spurious fragment well under 120 px",
        "allowed": True},
    "ARM_H_MIN": {
        "source": "debug 51-58: robot-arm blob h=0.445; tallest table object "
                  "h=0.148. v3 failed because the arm's bbox outranked the basket",
        "allowed": True},
    "SHORT_H_MAX": {
        "source": "debug 51-58 blob heights: the two boxes are 0.017 and 0.028, "
                  "every bottle/can/carton is 0.079-0.148",
        "allowed": True},
    "BUTTER_IS_REDDEST_SHORT": {
        "source": "pack keyframes demo0_t0000 vs demo0_t0168: the only non-arm "
                  "pixels that change are u29-36,v61-68, mean RGB (88,63,51), "
                  "R-B=37, vs the other short box (83,70,63), R-B=19",
        "allowed": True},
    # --- grasp ------------------------------------------------------------
    "GRASP_Z_BELOW_TOP": {
        "source": "pack.json keyframe t=63 ee z=0.0095; debug table z=0.0015 and "
                  "butter ztop=0.019 -> grasp 0.010 m below the object top",
        "allowed": True},
    "OPEN_W": {
        "source": "debug 51 api.gripper() at reset: width_m=0.0778",
        "allowed": True},
    "CLOSE_W": {
        "source": "FairApi mechanics: api.grip(<0.025) commands a close",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi mechanics: gripper effort is 3.0 iff holding; "
                  "debug 51-58 measured 3.0 on every successful close",
        "allowed": True},
    "HOLD_W_MAX": {
        "source": "debug 51-58: a loaded gripper reads width 0.0389 (the butter's "
                  "0.039 m minor extent); empty/open reads 0.078-0.080",
        "allowed": True},
    "YAW_DEADBAND_DEG": {
        "source": "debug 51-58: the butter's minor footprint axis is within a "
                  "couple of degrees of world +y in every seed, so a 12 deg "
                  "deadband keeps the proven straight-down wrist on these layouts",
        "allowed": True},
    # --- transport / release ---------------------------------------------
    "CARRY_Z": {
        "source": "pack.json ee_path6 transport segment rides z=0.226-0.278",
        "allowed": True},
    "RELEASE_ABOVE_RIM": {
        "source": "pack.json keyframe t=150 ee z=0.1909 with the debug basket rim "
                  "measured at ztop=0.141 -> 0.050 m above the rim",
        "allowed": True},
}

ABOVE_TABLE_M = 0.008
MIN_BLOB_PX = 120
ARM_H_MIN = 0.30
SHORT_H_MAX = 0.055
GRASP_Z_BELOW_TOP = 0.010
OPEN_W = 0.078
CLOSE_W = 0.010
HOLD_EFFORT = 2.0
HOLD_W_MAX = 0.070
HOLD_W_MIN = 0.004
YAW_DEADBAND_DEG = 12.0
CARRY_Z = 0.27
RELEASE_ABOVE_RIM = 0.050

DUMP_DIR = "/mnt/data/YifanKang/Heron/results/probe_dump_c2k1clean_obj_butter_pos_k1"


# --------------------------------------------------------------- perception
def _cloud(frame):
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    with np.errstate(all="ignore"):
        pts = np.stack([(uu - K[0, 2]) * d / K[0, 0],
                        (vv - K[1, 2]) * d / K[1, 1], d, np.ones_like(d)], -1)
        world = pts @ T.T
    return world[..., :3], np.isfinite(d) & (d > 0)


def _label(mask):
    h, w = mask.shape
    ys, xs = np.nonzero(mask)
    idx = np.zeros((h, w), int)
    idx[ys, xs] = np.arange(1, ys.size + 1)
    parent = np.arange(ys.size + 1)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for m, ia, ib in ((mask[:, :-1] & mask[:, 1:], idx[:, :-1], idx[:, 1:]),
                      (mask[:-1, :] & mask[1:, :], idx[:-1, :], idx[1:, :])):
        for p, q in zip(ia[m], ib[m]):
            ra, rb = find(p), find(q)
            if ra != rb:
                parent[ra] = rb
    roots = np.array([find(i) for i in range(ys.size + 1)])
    lab = np.zeros((h, w), int)
    lab[ys, xs] = roots[idx[ys, xs]]
    return lab


def perceive(api):
    """-> (frame, rgb, z_table, blobs sorted by pixel count)."""
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb)
    P, ok = _cloud(f)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    ws = ok & (X > -0.40) & (X < 0.40) & (Y > -0.50) & (Y < 0.60)
    hist, edges = np.histogram(Z[ws], bins=150, range=(-0.05, 0.10))
    ztab = 0.5 * (edges[hist.argmax()] + edges[hist.argmax() + 1])
    above = ws & (Z > ztab + ABOVE_TABLE_M) & (Z < ztab + 0.45)
    lab = _label(above)
    ids, counts = np.unique(lab[lab > 0], return_counts=True)

    blobs = []
    for i, n in sorted(zip(ids, counts), key=lambda t: -t[1]):
        if n < MIN_BLOB_PX:
            break
        m = lab == i
        ys, xs = np.nonzero(m)
        xx, yy, zz = X[m], Y[m], Z[m]
        ztop = float(np.percentile(zz, 97))
        col = rgb[m].astype(float)
        top = zz > ztop - 0.012
        blobs.append(dict(
            n=int(n), ztop=ztop, h=ztop - ztab,
            x=float(np.median(xx)), y=float(np.median(yy)),
            cx=0.5 * (float(xx.min()) + float(xx.max())),
            cy=0.5 * (float(yy.min()) + float(yy.max())),
            ex=float(xx.max() - xx.min()), ey=float(yy.max() - yy.min()),
            u=float(xs.mean()), v=float(ys.mean()),
            red=float((col[:, 0] - col[:, 2]).mean()),
            rgb=col.mean(0).round(0).tolist(),
            tx=xx[top], ty=yy[top]))
    return f, rgb, ztab, blobs


def pick_targets(api, blobs):
    """-> (basket, butter). Both re-derived per episode, never from the pack xy."""
    table = [b for b in blobs if b["h"] < ARM_H_MIN] or blobs
    basket = max(table, key=lambda b: b["ex"] * b["ey"])
    shorts = [b for b in table if b["h"] < SHORT_H_MAX and b is not basket]
    if not shorts:
        api.log("WARN no short blob; falling back to all non-basket blobs")
        shorts = [b for b in table if b is not basket] or table
    butter = max(shorts, key=lambda b: b["red"])
    return basket, butter


def footprint_axes(b):
    """Principal axes of the butter's top face -> (minor_dir, minor_ext, major_ext)."""
    pts = np.stack([b["tx"], b["ty"]], 1)
    if pts.shape[0] < 12:
        return np.array([0.0, 1.0]), b["ey"], b["ex"]
    c = pts - pts.mean(0)
    w, V = np.linalg.eigh(np.cov(c.T))
    minor, major = V[:, 0], V[:, 1]
    return minor, float(np.ptp(c @ minor)), float(np.ptp(c @ major))


def yaw_rotation(R0, minor):
    """R0 spun about world z so the jaw axis lies along `minor`."""
    theta = np.arctan2(minor[0], -minor[1])
    theta = (theta + np.pi / 2) % np.pi - np.pi / 2
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ R0, theta


def holding(g):
    return (g["effort"] >= HOLD_EFFORT
            and HOLD_W_MIN < g["width_m"] < HOLD_W_MAX)


def _dump(api, rgb, f):
    try:
        os.makedirs(DUMP_DIR, exist_ok=True)
        k = len([p for p in os.listdir(DUMP_DIR) if p.startswith("v5_")])
        np.savez_compressed(os.path.join(DUMP_DIR, "v5_%02d.npz" % k), rgb=rgb,
                            depth=np.asarray(f.depth, np.float32),
                            K=np.asarray(f.intrinsics), T=np.asarray(f.t_base_cam))
        api.log("dump v5_%02d.npz" % k)
    except Exception as exc:  # noqa: BLE001
        api.log("dump failed %r" % (exc,))


# --------------------------------------------------------------------- task
def run(api):
    api.log("=== v5 === instr=%r" % api.instruction())
    R0 = np.asarray(api.tool_rotation(), float)

    f, rgb, ztab, blobs = perceive(api)
    _dump(api, rgb, f)
    api.log("ztab=%.4f nblobs=%d" % (ztab, len(blobs)))
    for i, b in enumerate(blobs):
        api.log("B%02d n=%4d xy=(%.3f,%.3f) h=%.3f ext=(%.3f,%.3f) px=(%.0f,%.0f) "
                "rgb=%s red=%.0f" % (i, b["n"], b["x"], b["y"], b["h"], b["ex"],
                                     b["ey"], b["u"], b["v"], b["rgb"], b["red"]))

    basket, butter = pick_targets(api, blobs)
    api.log("BASKET c=(%.3f,%.3f) rim=%.3f ext=(%.3f,%.3f)"
            % (basket["cx"], basket["cy"], basket["ztop"], basket["ex"], basket["ey"]))

    tx, ty = basket["cx"], basket["cy"]
    rz = basket["ztop"] + RELEASE_ABOVE_RIM

    held = False
    for attempt in range(3):
        if attempt:
            f2, rgb2, ztab, blobs = perceive(api)
            _, butter = pick_targets(api, blobs)
        minor, wmin, wmaj = footprint_axes(butter)
        R, theta = yaw_rotation(R0, minor)
        use_rot = abs(np.degrees(theta)) > YAW_DEADBAND_DEG
        bx, by = butter["cx"], butter["cy"]
        gz = max(ztab + 0.006, butter["ztop"] - GRASP_Z_BELOW_TOP)
        api.log("try%d BUTTER c=(%.3f,%.3f) h=%.3f ztop=%.3f ext=(%.3f,%.3f) "
                "red=%.0f minor=(%.2f,%.2f) w=%.3f/%.3f yaw=%.1fdeg use_rot=%s "
                "grasp_z=%.3f" % (attempt, bx, by, butter["h"], butter["ztop"],
                                  butter["ex"], butter["ey"], butter["red"],
                                  minor[0], minor[1], wmin, wmaj,
                                  np.degrees(theta), use_rot, gz))
        rot = R if use_rot else None

        api.grip(OPEN_W)
        api.settle(0.2)
        api.move([bx, by, CARRY_Z], rotation=rot, seconds=2.0)
        api.move([bx, by, butter["ztop"] + 0.06], rotation=rot, seconds=1.2)
        r = api.move([bx, by, gz], rotation=rot, seconds=1.4)
        api.grip(CLOSE_W)
        api.settle(0.4)
        g = api.gripper()
        api.log("try%d descend_res=%.4f eef=%s close_grip=%s"
                % (attempt, r, np.round(api.eef(), 4).tolist(), g))
        api.move([bx, by, CARRY_Z], rotation=rot, seconds=2.0)
        g = api.gripper()
        api.log("try%d after_lift grip=%s eef=%s"
                % (attempt, g, np.round(api.eef(), 4).tolist()))
        if holding(g):
            held = True
            break
        api.log("try%d NOT HOLDING -- retrying" % attempt)
        api.grip(OPEN_W)
        api.settle(0.2)

    if not held:
        api.log("give up on grasp; still driving to the basket")

    api.move([tx, ty, CARRY_Z], seconds=3.0)
    r = api.move([tx, ty, rz], seconds=1.5)
    api.log("over_basket res=%.4f eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.5)
    api.log("released grip=%s" % api.gripper())
    api.move([tx, ty, CARRY_Z], seconds=1.5)
    api.settle(0.4)
    api.log("=== v5 done held=%s ===" % held)
