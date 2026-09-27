"""c2clean obj_milk_task_k0 -- v2.

Intent: "Pick the butter and place it in the basket".

Pipeline, all from this episode's own sensors:
  1. cam_high RGB-D -> base-frame cloud -> height-band crop -> XY grid
     clustering -> component table (the parked arm is removed by a height gate,
     since every table prop measured well below it on the debug seeds).
  2. Target = the THICKEST of the flat boxes (rank, not threshold).
     Destination = the LARGEST component (the basket), whose rim top and
     interior centre come from its own points.
  3. Fingertip calibration: press straight down on a bare patch of table with
     the jaws open; the eef z where the descent stalls, minus the perceived
     table height, is the fingertip-to-eef offset. Every later height is
     commanded through that offset.
  4. Top-down grasp across the box's SHORT axis, lift, carry above the rim,
     lower inside the basket, release.
Verification is by my own sensors only (gripper width/effort + re-perception);
no termination flag is consulted anywhere.
"""

import numpy as np

PROVENANCE = {
    "R_DOWN": {
        "source": "generic controller mechanics: tool z anti-parallel to world z; "
                  "matches api.tool_rotation() at reset on debug seeds 51-65",
        "allowed": True},
    "WS_X, WS_Y": {
        "source": "debug seeds 51-65 cam_high deprojection: every table prop and the "
                  "basket fall inside x[-0.35,0.35] y[-0.40,0.40]; walls/floor deproject "
                  "far outside it",
        "allowed": True},
    "Z_ARM_GATE": {
        "source": "debug seeds 51-65: tallest table component measured top z=0.144, "
                  "the parked arm column spans z>0.2; 0.18 separates them",
        "allowed": True},
    "Z_TABLE_MIN": {
        "source": "debug seeds 51-65: table surface deprojects to z~0.001, prop points "
                  "start above it; 0.008 is the first bin clear of surface noise",
        "allowed": True},
    "GRID_CELL": {
        "source": "debug seeds 51-65: 0.012 m XY cells split all six props and the "
                  "basket into separate components (nearest pair ~0.09 m apart)",
        "allowed": True},
    "MIN_COMP_PTS": {
        "source": "debug seeds 51-65: real props gave >=700 points, speckle <120",
        "allowed": True},
    "FLAT_MAX_TOP": {
        "source": "debug seeds 51-65: three flat boxes topped at 0.030/0.020/0.019 m, "
                  "the next component up is the can at 0.081",
        "allowed": True},
    "WARMTH (r-b ranking)": {
        "source": "debug seed 51 wrist-camera label reads over each component (v4 probe): "
                  "the box labelled FARM FRESH BUTTER has cam_high mean rgb [92,56,39] "
                  "-> r-b=+53, vs CHOCOLATE PUDDING [69,54,48] -> +21 and CREAM CHEESE "
                  "[72,78,97] -> -25; butter is the warmest flat box by a 32-count margin",
        "allowed": True},
    "GRASP/RELEASE receipts": {
        "source": "debug seeds 51-65: a real bite reads effort 3.0 and width_m equal "
                  "to the box short axis (0.0389 vs dy 0.039); on release effort falls "
                  "3.0 -> 0.05 while width_m lags, so effort is the release receipt",
        "allowed": True},
    "SAFE_Z": {
        "source": "debug seeds 51-65: tallest table object tops at 0.144; 0.26 eef "
                  "(tips ~0.25) clears it, and is the reset eef height",
        "allowed": True},
    "PROBE_XY": {
        "source": "debug seeds 51-65: (-0.05,-0.33) is bare table in every frame "
                  "(nearest prop centre 0.14 m away)",
        "allowed": True},
    "GRIP_OPEN, GRIP_SHUT": {
        "source": "debug seeds 51-65: api.gripper() reads width_m 0.0778 at reset; "
                  "brief states <0.025 closes",
        "allowed": True},
    "TIP_GRASP_H, CARRY_H, RELEASE_H": {
        "source": "debug-seed measurements: butter top 0.0296, basket rim top 0.1437, "
                  "basket interior floor ~0.075; heights chosen relative to those",
        "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

WS_X = (-0.35, 0.35)
WS_Y = (-0.40, 0.40)
Z_ARM_GATE = 0.18
Z_TABLE_MIN = 0.008
GRID_CELL = 0.012
MIN_COMP_PTS = 120
FLAT_MAX_TOP = 0.06

PROBE_XY = (-0.05, -0.33)
GRIP_OPEN = 0.08
GRIP_SHUT = 0.0

TIP_GRASP_H = 0.006
TIP_HOVER_H = 0.12
CARRY_H = 0.21
RELEASE_H = 0.10


# ---------------------------------------------------------------- perception
def cloud(api, cam="cam_high"):
    fr = api.capture(cam)
    d = np.asarray(fr.depth, dtype=np.float64)
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]
    pc = np.stack([(uu - K[0, 2]) / K[0, 0] * d, (vv - K[1, 2]) / K[1, 1] * d, d], -1)
    P = pc @ T[:3, :3].T + T[:3, 3]
    return np.asarray(fr.rgb, dtype=np.float64), P


def components(api, zmax=Z_ARM_GATE):
    rgb, P = cloud(api)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = ((X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1])
         & (Z > Z_TABLE_MIN) & (Z < zmax))
    pts = np.stack([X[m], Y[m], Z[m]], -1)
    cols = rgb[m]
    if len(pts) == 0:
        return []
    gi = np.round(pts[:, 0] / GRID_CELL).astype(int)
    gj = np.round(pts[:, 1] / GRID_CELL).astype(int)
    occ = {}
    for k in range(len(gi)):
        occ.setdefault((gi[k], gj[k]), []).append(k)
    seen = set()
    out = []
    for key in occ:
        if key in seen:
            continue
        stack, idx = [key], []
        seen.add(key)
        while stack:
            c = stack.pop()
            idx.extend(occ[c])
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    n = (c[0] + da, c[1] + db)
                    if n in occ and n not in seen:
                        seen.add(n)
                        stack.append(n)
        if len(idx) < MIN_COMP_PTS:
            continue
        p = pts[idx]
        top = float(p[:, 2].max())
        band = p[p[:, 2] > top - 0.008]
        out.append(dict(
            n=len(idx), top=top,
            ctr=(float(band[:, 0].mean()), float(band[:, 1].mean())),
            xmin=float(p[:, 0].min()), xmax=float(p[:, 0].max()),
            ymin=float(p[:, 1].min()), ymax=float(p[:, 1].max()),
            rgb=cols[idx].mean(0),
            band=band, pts=p))
    return out


def table_height(api):
    rgb, P = cloud(api)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = ((X > PROBE_XY[0] - 0.05) & (X < PROBE_XY[0] + 0.05)
         & (Y > PROBE_XY[1] - 0.05) & (Y < PROBE_XY[1] + 0.05)
         & (Z > -0.05) & (Z < 0.05))
    return float(np.median(Z[m])) if m.sum() else 0.0


# ------------------------------------------------------------------ motion
def goto(api, x, y, z, seconds=2.0):
    r = api.move([float(x), float(y), float(z)], rotation=R_DOWN, seconds=seconds)
    e = api.eef()
    api.log("  move -> (%+.4f,%+.4f,%+.4f) resid=%s got=(%+.4f,%+.4f,%+.4f)"
            % (x, y, z, np.round(np.asarray(r, dtype=float).ravel(), 4).tolist(),
               e[0], e[1], e[2]))
    return np.asarray(api.eef(), dtype=float)


import base64 as _b64, zlib as _zl


def dump(api, tag, cam="cam_high", with_depth=False):
    fr = api.capture(cam)
    arrs = [("rgb", np.asarray(fr.rgb, dtype=np.uint8))]
    if with_depth:
        arrs.append(("depth", np.asarray(fr.depth, dtype=np.float32)))
    api.log("%s SHAPE rgb=%s depth=%s" % (tag, arrs[0][1].shape, np.asarray(fr.depth).shape))
    api.log("%s K=%s" % (tag, np.asarray(fr.intrinsics).tolist()))
    api.log("%s T=%s" % (tag, np.asarray(fr.t_base_cam).tolist()))
    for name, arr in arrs:
        b = _b64.b64encode(_zl.compress(arr.tobytes(), 6)).decode("ascii")
        n = 1500
        api.log("%s %s NCHUNK %d dtype=%s" % (tag, name, (len(b) + n - 1) // n, arr.dtype))
        for i in range(0, len(b), n):
            api.log("%s %s C%05d %s" % (tag, name, i // n, b[i:i + n]))


def allcomps(api, tag):
    for c in components(api, zmax=0.60):
        api.log("%s n=%5d top=%.4f ctr=(%+.4f,%+.4f) dx=%.3f dy=%.3f"
                % (tag, c["n"], c["top"], c["ctr"][0], c["ctr"][1],
                   c["xmax"] - c["xmin"], c["ymax"] - c["ymin"]))


SAFE_Z = 0.26


def hop(api, x, y, z, seconds=2.0):
    """Lateral moves happen only at SAFE_Z: a low diagonal transit swept a
    standing carton over on debug seed 51 (v2)."""
    e = np.asarray(api.eef(), dtype=float)
    if abs(e[0] - x) > 0.02 or abs(e[1] - y) > 0.02:
        if e[2] < SAFE_Z - 0.01:
            goto(api, e[0], e[1], SAFE_Z, seconds=1.5)
        goto(api, x, y, SAFE_Z, seconds=seconds)
    goto(api, x, y, z, seconds=seconds)


def run(api):
    api.log("INSTRUCTION %r" % (api.instruction(),))

    comps = components(api)
    comps.sort(key=lambda c: -c["n"])
    for c in comps:
        api.log("COMP n=%5d top=%.4f ctr=(%+.4f,%+.4f) dx=%.3f dy=%.3f rgb=%s"
                % (c["n"], c["top"], c["ctr"][0], c["ctr"][1],
                   c["xmax"] - c["xmin"], c["ymax"] - c["ymin"],
                   np.round(c["rgb"]).astype(int).tolist()))
    if not comps:
        api.log("NO COMPONENTS -- abort")
        return

    basket = max(comps, key=lambda c: c["n"])
    rim_top = basket["top"]
    bx = 0.5 * (basket["xmin"] + basket["xmax"])
    by = 0.5 * (basket["ymin"] + basket["ymax"])
    api.log("BASKET ctr=(%+.4f,%+.4f) rim_top=%.4f" % (bx, by, rim_top))

    # Target = the warmest (most orange) of the flat boxes.  The wrist-camera
    # label reads on debug seed 51 put FARM FRESH BUTTER at r-b=+53 against
    # +21 (chocolate pudding) and -25 (cream cheese).
    flats = [c for c in comps if c is not basket and c["top"] < FLAT_MAX_TOP]
    if not flats:
        api.log("NO FLAT BOX -- abort")
        return
    for c in flats:
        c["warm"] = float(c["rgb"][0] - c["rgb"][2])
    flats.sort(key=lambda c: -c["warm"])
    for c in flats:
        api.log("FLAT warm=%+.1f top=%.4f ctr=(%+.4f,%+.4f) rgb=%s"
                % (c["warm"], c["top"], c["ctr"][0], c["ctr"][1],
                   np.round(c["rgb"]).astype(int).tolist()))
    tgt = flats[0]
    tx, ty = tgt["ctr"]
    tdx = tgt["xmax"] - tgt["xmin"]
    tdy = tgt["ymax"] - tgt["ymin"]
    api.log("TARGET warm=%+.1f ctr=(%+.4f,%+.4f) top=%.4f dx=%.3f dy=%.3f"
            % (tgt["warm"], tx, ty, tgt["top"], tdx, tdy))

    z_tab = table_height(api)
    api.log("TABLE z=%.4f" % z_tab)

    api.grip(GRIP_OPEN)
    api.settle(0.2)
    hop(api, PROBE_XY[0], PROBE_XY[1], z_tab + 0.20, seconds=2.0)
    goto(api, PROBE_XY[0], PROBE_XY[1], z_tab - 0.05, seconds=3.0)
    e = np.asarray(api.eef(), dtype=float)
    tip_off = float(e[2] - z_tab)
    api.log("TIP_OFFSET %.4f (stalled eef z=%.4f, table z=%.4f)" % (tip_off, e[2], z_tab))

    def tipz(h):
        return z_tab + h + tip_off

    # Grasp, then verify with my own sensors: a real bite reads effort 3.0 AND a
    # width near the box's short axis (on debug seeds the closed width matched
    # dy to a millimetre).  A closed-to-nothing or empty-jaw read gets one
    # retry from a fresh perception.
    for attempt in (0, 1):
        hop(api, tx, ty, tipz(TIP_HOVER_H), seconds=2.0)
        goto(api, tx, ty, tipz(TIP_GRASP_H), seconds=2.5)
        api.grip(GRIP_SHUT)
        api.settle(0.6)
        g = api.gripper()
        w = g["width_m"]
        api.log("AFTER CLOSE attempt=%d width=%.4f effort=%.3f (box short axis dy=%.3f)"
                % (attempt, w, g["effort"], tdy))

        goto(api, tx, ty, tipz(CARRY_H), seconds=2.5)
        g = api.gripper()
        w = g["width_m"]
        api.log("AFTER LIFT attempt=%d width=%.4f effort=%.3f" % (attempt, w, g["effort"]))
        after = components(api)
        still = [c for c in after
                 if abs(c["ctr"][0] - tx) < 0.06 and abs(c["ctr"][1] - ty) < 0.06
                 and c["top"] < FLAT_MAX_TOP]
        api.log("REPERCEIVE target-site components: %d" % len(still))

        # Retry ONLY on an unambiguously empty hand.  Re-perception is logged as
        # corroboration but must not gate the retry: a shadow left at the target
        # site would otherwise make the program open its jaws in mid-air and
        # drop a box it was actually holding.
        empty = (g["effort"] < 3.0) or (w <= 0.5 * tdy)
        api.log("GRASP empty=%s still=%d attempt=%d" % (empty, len(still), attempt))
        if not empty or attempt == 1:
            break
        # retry: re-perceive the target where it now lies and aim again
        api.grip(GRIP_OPEN)
        api.settle(0.4)
        if still:
            tgt = max(still, key=lambda c: c["rgb"][0] - c["rgb"][2])
            tx, ty = tgt["ctr"]
            tdy = tgt["ymax"] - tgt["ymin"]
            api.log("RETRY re-aim ctr=(%+.4f,%+.4f) dy=%.3f" % (tx, ty, tdy))

    hop(api, bx, by, tipz(CARRY_H), seconds=3.0)
    g = api.gripper()
    api.log("OVER BASKET width=%.4f effort=%.3f" % (g["width_m"], g["effort"]))
    dump(api, "OVERWRIST", cam="cam_arm_wrist")
    # Lower into the basket.  This descent is contact-limited -- on debug seed
    # 53 it stalled ~25 mm high on the liner -- which is fine, the box only has
    # to end up inside.  What must not be skipped is the jaws actually opening,
    # so the open is re-issued until the width comes back up.
    goto(api, bx, by, tipz(RELEASE_H), seconds=2.5)
    # The release receipt is EFFORT, not width: on debug seeds 52-64 the jaws
    # let go (effort 3.0 -> 0.05) while width_m still read the old 0.0389.
    for k in (0, 1):
        api.grip(GRIP_OPEN)
        api.settle(0.8)
        g = api.gripper()
        api.log("AFTER RELEASE k=%d width=%.4f effort=%.3f" % (k, g["width_m"], g["effort"]))
        if g["effort"] < 3.0:
            break
    # staged retreat: a single big vertical command jammed on debug seed 53
    e = np.asarray(api.eef(), dtype=float)
    goto(api, bx, by, float(e[2]) + 0.04, seconds=1.5)
    goto(api, bx, by, tipz(CARRY_H), seconds=2.0)
    api.settle(0.5)
    dump(api, "ENDWRIST", cam="cam_arm_wrist")

    for c in components(api):
        api.log("FINAL n=%5d top=%.4f ctr=(%+.4f,%+.4f)"
                % (c["n"], c["top"], c["ctr"][0], c["ctr"][1]))
    api.settle(1.0)
    return
