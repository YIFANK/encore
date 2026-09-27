"""c2clean spa_bowl_top_drawer_cabinet_task_k0 -- v8.

Intent: "Pick the akita black bowl on the top of the wooden cabinet and place
it on the plate."

Scene, as measured on debug seeds 51/53/57/61 (v1 perception probe):
  table top z = 0.901, cabinet top slab z = 1.127, a bowl standing ON the slab
  (rim top 1.180, interior floor 1.136, rim ring radius 0.052), a second bowl
  down in the open drawer (rim top 1.116) which the intent does NOT name, and
  a light plate on the table (top 0.908, radius 0.057).

Mechanism: the bowl's inner rim diameter (~0.104) exceeds the gripper's full
opening (0.078), so a centred grasp puts both fingers inside the bowl and
closes on nothing. v2 straddles ONE wall instead: the eef is offset from the
bowl centre by the rim radius along the gripper's closing axis (base y at the
resting wrist), so the near finger drops inside the bowl and the far finger
outside it, and the close clamps the wall between them.

v2 also calibrates the fingertip-to-eef vertical offset by pressing the open
gripper onto the bare cabinet slab, so every later height is expressed in
fingertip space.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "CELL/XLO/XHI/YLO/YHI": {
        "source": "generic top-down height-map discretisation of the cam_high "
                  "point cloud; bounds chosen to exclude the room walls, which "
                  "deproject to x~-2.0 (debug-seed 51 RGB-D)",
        "allowed": True},
    "BAND_BOWL (slab+0.038 .. slab+0.068)": {
        "source": "debug seeds 51/53/57/61 RGB-D: bowl rim top sits 0.053 above "
                  "the cabinet slab; band brackets the rim ring",
        "allowed": True},
    "BAND_PLATE (table+0.008 .. table+0.038)": {
        "source": "debug seeds 51/53/57/61 RGB-D: plate top 0.908 vs table 0.901",
        "allowed": True},
    "GRIP_DEPTH = 0.004": {
        "source": "debug-seed RGB-D: bowl interior floor 1.1356-1.1361. v3 "
                  "stopped 10 mm above it and the closed gap decayed from "
                  "0.008 to 0.003 on 5 of 8 seeds (the bite creeping up to the "
                  "thin rim lip); 4 mm clamps a thicker cross-section",
        "allowed": True},
    "RIM_BIAS = 0.002": {
        "source": "debug-seed RGB-D: Kasa ring fit (r=0.052) lies just inside "
                  "the rim-top annulus measured at r=0.0525-0.057",
        "allowed": True},
    "TIP_OFFSET = 0.0103": {
        "source": "debug seeds 57/61 (v2 run): the open gripper was driven into "
                  "the bare cabinet top (z 1.1272) and stalled with the eef at "
                  "1.1374/1.1374, so the eef rides 0.0103 above the fingertips",
        "allowed": True},
    "CARRY_Z = 1.27": {
        "source": "debug-seed RGB-D: tallest scene structure on the transfer "
                  "path is the cabinet slab at 1.127; 1.27 clears it",
        "allowed": True},
    "THIN_GAP = 0.0040": {
        "source": "debug seeds 51-65 (v2/v3): a close that caught the bowl wall "
                  "reported gap 0.005-0.010; below 0.004 the jaws had met each "
                  "other",
        "allowed": True},
    "PLACE_LEGS = (0.20, 0.11, 0.055, 0.030)": {
        "source": "debug seeds 51-65 (v4): lateral drift during a descent is "
                  "~7% of the leg length and is uncorrectable below z~1.05, so "
                  "the 0.33 m drop to the plate is split into ~0.06 m legs",
        "allowed": True},
    "BIAS_CLIP = 0.035": {
        "source": "debug seeds 51-65 (v4/v6): the standing place-aim offset was "
                  "0.006-0.028 m; clipping just above that keeps a genuine "
                  "reach limit from running the command away",
        "allowed": True},
    "PLACE_CLEAR = 0.006": {
        "source": "debug-seed RGB-D: plate inner top 0.908; release the bowl "
                  "base just above it",
        "allowed": True},
}

CELL = 0.005
XLO, XHI, YLO, YHI = -0.50, 0.38, -0.48, 0.48
NX = int((XHI - XLO) / CELL) + 1
NY = int((YHI - YLO) / CELL) + 1
OPEN_W = 0.06
SHUT_W = 0.0
CHUNK = 1700
TIP_OFFSET = 0.0103      # eef sits this far above the fingertips
GRIP_DEPTH = 0.004       # fingertips above the bowl interior floor
THIN_GAP = 0.0040        # a close this tight bit nothing worth carrying
CARRY_Z = 1.27
PLACE_CLEAR = 0.006
PLACE_LEGS = (0.20, 0.11, 0.055, 0.030)
BIAS_CLIP = 0.035


# --------------------------------------------------------------- logging ---
def dump(api, tag, arr):
    arr = np.ascontiguousarray(arr)
    blob = base64.b64encode(zlib.compress(arr.tobytes(), 6)).decode()
    parts = [blob[i:i + CHUNK] for i in range(0, len(blob), CHUNK)]
    api.log("DUMP %s dtype=%s shape=%s n=%d" % (tag, arr.dtype, list(arr.shape), len(parts)))
    for i, p in enumerate(parts):
        api.log("D %s %d %s" % (tag, i, p))


# ------------------------------------------------------------ perception ---
def cloud(rgb, depth, K, T):
    h, w = depth.shape
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    x = (uu - K[0, 2]) * depth / K[0, 0]
    y = (vv - K[1, 2]) * depth / K[1, 1]
    p = np.stack([x, y, depth], -1) @ T[:3, :3].T + T[:3, 3]
    return p.reshape(-1, 3), np.asarray(rgb, float).reshape(-1, 3)


def gridmap(P, C):
    ok = (np.isfinite(P[:, 2]) & (P[:, 0] > XLO) & (P[:, 0] < XHI)
          & (P[:, 1] > YLO) & (P[:, 1] < YHI) & (P[:, 2] > 0.80) & (P[:, 2] < 1.45))
    P, C = P[ok], C[ok]
    h = np.full((NX, NY), np.nan)
    c = np.zeros((NX, NY, 3))
    ix = ((P[:, 0] - XLO) / CELL).astype(int)
    iy = ((P[:, 1] - YLO) / CELL).astype(int)
    o = np.argsort(P[:, 2])
    h[ix[o], iy[o]] = P[o, 2]
    c[ix[o], iy[o]] = C[o]
    return h, c


def ccs(mask, min_cells=12):
    lab = np.zeros(mask.shape, np.int32)
    out = []
    for i0, j0 in np.argwhere(mask):
        if lab[i0, j0]:
            continue
        lab[i0, j0] = 1
        stack = [(i0, j0)]
        cells = []
        while stack:
            a, b = stack.pop()
            cells.append((a, b))
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    p, q = a + da, b + db
                    if (0 <= p < mask.shape[0] and 0 <= q < mask.shape[1]
                            and mask[p, q] and not lab[p, q]):
                        lab[p, q] = 1
                        stack.append((p, q))
        if len(cells) >= min_cells:
            out.append(np.array(cells))
    return out


def kasa(x, y):
    A = np.c_[x, y, np.ones(len(x))]
    s = np.linalg.lstsq(A, x * x + y * y, rcond=None)[0]
    cx, cy = s[0] / 2.0, s[1] / 2.0
    r = float(np.sqrt(max(s[2] + cx * cx + cy * cy, 1e-9)))
    return float(cx), float(cy), r, float(np.std(np.hypot(x - cx, y - cy) - r))


def xy_of(cells):
    return XLO + cells[:, 0] * CELL, YLO + cells[:, 1] * CELL


def modal_z(h, lo, hi, step=0.005):
    v = h[np.isfinite(h) & (h > lo) & (h < hi)]
    if v.size == 0:
        return None
    n, e = np.histogram(v, np.arange(lo, hi + step, step))
    k = int(np.argmax(n))
    return float(np.median(v[(v >= e[k]) & (v < e[k + 1])]))


def find_scene(api, h, c):
    out = {}
    table = modal_z(h, 0.86, 0.96)
    out["table_z"] = table
    slab = modal_z(h, table + 0.17, table + 0.30)
    out["slab_z"] = slab
    if table is None or slab is None:
        return out

    m = np.isfinite(h) & (h > slab + 0.038) & (h < slab + 0.068)
    m &= (c[..., 2] < c[..., 0] + 25)
    best = None
    for cells in ccs(m, 25):
        X, Y = xy_of(cells)
        cx, cy, r, res = kasa(X, Y)
        Z = h[cells[:, 0], cells[:, 1]]
        api.log("  bowlcand n=%d ctr=(%+.3f,%+.3f) r=%.4f res=%.4f ztop=%.3f"
                % (len(cells), cx, cy, r, res, Z.max()))
        if not (0.038 < r < 0.075 and res < 0.007):
            continue
        if best is None or len(cells) > best[0]:
            best = (len(cells), cx, cy, r, float(Z.max()))
    if best is None:
        return out
    _, cx, cy, r, ztop = best
    out.update(bowl_xy=(cx, cy), bowl_r=r, bowl_top=ztop)

    ii, jj = np.nonzero(np.isfinite(h))
    X, Y, Z = XLO + ii * CELL, YLO + jj * CELL, h[ii, jj]
    d = np.hypot(X - cx, Y - cy)
    inner = Z[d < r * 0.45]
    out["bowl_floor"] = float(np.median(inner)) if inner.size else slab
    ring = Z[(d > r + 0.022) & (d < r + 0.055)]
    out["support_z"] = float(np.median(ring)) if ring.size else slab

    mp = np.isfinite(h) & (h > table + 0.008) & (h < table + 0.038)
    mp &= (c[..., 0] > 110) & (np.abs(c[..., 0] - c[..., 2]) < 30)
    pbest = None
    for cells in ccs(mp, 60):
        X2, Y2 = xy_of(cells)
        px, py, pr, pres = kasa(X2, Y2)
        api.log("  platecand n=%d ctr=(%+.3f,%+.3f) r=%.4f res=%.4f"
                % (len(cells), px, py, pr, pres))
        if not (0.045 < pr < 0.095):
            continue
        if pbest is None or len(cells) > pbest[0]:
            pbest = (len(cells), px, py, pr)
    if pbest is not None:
        out.update(plate_xy=(pbest[1], pbest[2]), plate_r=pbest[3])
        d2 = np.hypot(X - pbest[1], Y - pbest[2])
        mid = Z[d2 < pbest[3] * 0.5]
        out["plate_top"] = float(np.median(mid)) if mid.size else table + 0.007
    return out


# ----------------------------------------------------------------- motion ---
def snap(api):
    f = api.capture("cam_high")
    P, C = cloud(np.asarray(f.rgb, np.uint8), np.asarray(f.depth, np.float64),
                 np.asarray(f.intrinsics, float), np.asarray(f.t_base_cam, float))
    return f, gridmap(P, C)


def say(api, tag):
    e = api.eef()
    g = api.gripper()
    api.log("STATE %s eef=%s gap=%.4f eff=%.2f"
            % (tag, np.round(e, 4).tolist(), g["width_m"], g["effort"]))
    return e, g


def aim(api, xyz, seconds, reps, tag):
    """Issue the same position command `reps` times. A long saturated leg
    lands off-target laterally; the repeat starts from close range and the
    proportional controller converges it."""
    res = 0.0
    for _ in range(max(1, reps)):
        res = api.move(xyz, seconds=seconds)
        if res < 0.006:
            break
    say(api, tag)
    return res


def run(api):
    api.log("INSTRUCTION %r" % api.instruction())
    f, (h, c) = snap(api)
    dump(api, "s0_rgb", np.asarray(f.rgb, np.uint8)[::2, ::2])
    sc = find_scene(api, h, c)
    api.log("SCENE %s" % {k: (np.round(v, 4).tolist() if isinstance(v, tuple)
                              else round(v, 4) if isinstance(v, float) else v)
                          for k, v in sc.items()})
    if "bowl_xy" not in sc or "plate_xy" not in sc:
        return "perception failed"

    cx, cy = sc["bowl_xy"]
    r = sc["bowl_r"]
    slab = sc["support_z"]
    floor = sc["bowl_floor"]
    px, py = sc["plate_xy"]
    ptop = sc["plate_top"]

    api.grip(OPEN_W)
    tip = TIP_OFFSET

    # -- 2. rim straddle -----------------------------------------------------
    # Every descent is split so no leg is long enough to saturate the position
    # command: measured on v3, a 0.27 m saturated descent lands ~0.018 m off
    # laterally, while the same move re-issued from close range converges.
    off = r + 0.002
    gx, gy = cx, cy - off
    grip_tip_z = floor + GRIP_DEPTH
    gz = grip_tip_z + tip
    api.log("GRASP target xy=(%.4f,%.4f) off=%.4f tipz=%.4f eefz=%.4f"
            % (gx, gy, off, grip_tip_z, gz))
    aim(api, [gx, gy, slab + 0.13], 1.5, 2, "above_grasp")
    aim(api, [gx, gy, sc["bowl_top"] + 0.020], 1.0, 1, "over_rim")
    res = aim(api, [gx, gy, gz], 1.0, 1, "at_grasp")
    api.log("DESCENT residual=%.4f" % res)
    api.grip(SHUT_W)
    e, g = say(api, "closed")
    api.log("CLOSED_GAP %.4f" % g["width_m"])

    # one sensor-driven retry: a gap this tight means the jaws met each other,
    # not the bowl wall, so re-seat 6 mm deeper and close again.
    if g["width_m"] < THIN_GAP:
        api.grip(OPEN_W)
        aim(api, [gx, gy, sc["bowl_top"] + 0.020], 1.0, 2, "retry_over_rim")
        aim(api, [gx, gy, gz - 0.006], 1.0, 1, "retry_at_grasp")
        api.grip(SHUT_W)
        e, g = say(api, "retry_closed")
        api.log("RETRY_GAP %.4f" % g["width_m"])

    # -- 3. lift -------------------------------------------------------------
    api.move([gx, gy, slab + 0.10], seconds=1.2)
    say(api, "lifted_low")
    api.move([gx, gy, CARRY_Z], seconds=1.0)
    say(api, "lifted")

    # -- 4. carry and place --------------------------------------------------
    # At the grasp the bowl base still rested on the slab, so the base rides
    # (grasp_eef_z - slab) below the eef for the whole carry, and the xy offset
    # is the straddle offset by construction.
    base_below_eef = gz - slab
    place_z = ptop + PLACE_CLEAR + base_below_eef
    tx, ty = px, py - off
    api.log("PLACE target xy=(%.4f,%.4f) eefz=%.4f base_below=%.4f"
            % (tx, ty, place_z, base_below_eef))
    # The descent is split into short legs, then the residual aim error is
    # cancelled in the command. v6 established that splitting alone buys almost
    # nothing (|dx| 0.0164 -> 0.0153 mean, max unchanged at 0.028): the offset
    # is a standing tracking bias, not accumulated drift. It matters because
    # the +8 mm envelope probes failed exactly there -- the bowl caught the
    # plate rim and the release sprang the arm 0.03 m.
    aim(api, [tx, ty, CARRY_Z], 1.5, 2, "over_plate")
    for k, dz in enumerate(PLACE_LEGS):
        aim(api, [tx, ty, place_z + dz], 0.8, 1, "leg%d" % k)
    # Bias cancel. Measured on v4 and v6: at the low hover the eef sits a
    # repeatable 0.006-0.028 m off in x, identical whether the descent is one
    # leg or six, and re-issuing the same command does not move it -- so it is
    # a standing tracking offset, not drift. Shift the COMMAND by the observed
    # offset once (clipped, because a genuine reach limit would otherwise run
    # the command away) and finish the descent on the corrected aim.
    e, _ = say(api, "pre_bias")
    bx = float(np.clip(e[0] - tx, -BIAS_CLIP, BIAS_CLIP))
    by = float(np.clip(e[1] - ty, -BIAS_CLIP, BIAS_CLIP))
    ax, ay = tx - bx, ty - by
    api.log("BIAS dx=%.4f dy=%.4f -> aim (%.4f,%.4f)" % (bx, by, ax, ay))
    aim(api, [ax, ay, place_z + PLACE_LEGS[-2]], 0.8, 2, "biased_hover")
    res = aim(api, [ax, ay, place_z], 0.8, 1, "at_place")
    tx, ty = ax, ay
    api.log("PLACE residual=%.4f" % res)
    api.grip(OPEN_W)
    api.settle(0.4)
    say(api, "released")
    api.move([tx, ty, place_z + 0.12], seconds=1.2)
    api.settle(0.5)
    f3, (h3, c3) = snap(api)
    dump(api, "s2_rgb", np.asarray(f3.rgb, np.uint8)[::2, ::2])
    return "v4 staged straddle place"
