"""c2clean / spa_bowl_next_to_ramekin_task_k3 -- v4.

v3 scored 8/8 on the probe seeds, but its instrumentation showed the carry move
stalling against the arm's reach envelope on 5 of them: the commanded place x
(plate centre minus the carry bias) is unreachable when the plate sits far in
+y, so the move burned its whole budget, the jaws kept squeezing, and the bowl
dropped onto the plate instead of being set down on it.  v4 bounds every move by
PROGRESS rather than by a step cap -- re-issue only while the residual is still
shrinking -- so a stalled move costs ~50 steps instead of ~300 and the bite
survives the carry.  The post-release reach probe is removed; it has served.
"""
import base64
import json
import zlib
from collections import defaultdict

import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-65 cam_high depth: modal bare-tabletop z = 0.900 m",
                "allowed": True},
    "RIM_BAND_Z": {"source": "debug seeds 51-65: a z>0.946 m band over cam_high depth isolates "
                             "exactly the two bowl rim annuli (radii 0.045-0.055 m)",
                   "allowed": True},
    "LOW_BAND_Z": {"source": "debug seeds 51-65: cookie box and plate both top at 0.920 m, so the "
                             "0.908-0.940 m band carries both", "allowed": True},
    "WARM_MIN": {"source": "debug seeds 51-65 cam_high RGB: cookie-box cluster mean R-B = 0.18, "
                           "plate 0.05", "allowed": True},
    "BOWL_SPAN": {"source": "debug seeds 51-65: bowl rim annulus bbox span 0.107-0.111 m",
                  "allowed": True},
    "PLATE_SPAN_MIN": {"source": "debug seeds 51-65: plate footprint span 0.133-0.137 m vs cookie "
                                 "box 0.079-0.082 m", "allowed": True},
    "PLATE_TOP_MAX": {"source": "debug seeds 51-65: plate footprint max height 0.920 m; every "
                                "other low-band prop tops at 0.944 m or above", "allowed": True},
    "RIM_R": {"source": "debug seeds 51-65: rim-annulus radial histogram about the bbox mid peaks "
                        "at 0.045-0.055 m, wall midline 0.050 m", "allowed": True},
    "GRASP_DZ": {"source": "pack c2clean_spa_bowl_next_to_ramekin_task_k3 keyframes: close EEF z "
                           "0.9318/0.9169/0.9215 (mean 0.9234) = 0.029 m below the 0.952 m rim top "
                           "measured on debug seeds", "allowed": True},
    "RUNGS": {"source": "v1/v2 debug runs: a straddle whose post-lift gap stays above 0.006 m held; "
                        "gaps that decayed were re-seated by descending 0.008 m further or by "
                        "moving the aim 0.005 m inboard of the wall midline", "allowed": True},
    "MIN_BITE": {"source": "v1/v2 debug runs: post-lift gaps 0.007-0.015 m survived, 0.0013-0.0052 m "
                           "did not (harness counts gap>0.005 m as holding)", "allowed": True},
    "BOWL_HANG": {"source": "v1 debug run: the carried bowl bottomed out on the plate with the EEF "
                            "at z 0.958-0.962 while the plate top measured 0.920 m", "allowed": True},
    "POS_TOL_BIAS": {"source": "v2 debug run: every descent stopped 0.010-0.013 m above its "
                               "commanded z (the controller breaks inside its own position "
                               "tolerance), so the command is lowered by that much", "allowed": True},
    "CARRY_BIAS": {"source": "v1 debug run, 7 seeds grasped on the +y rim: the released bowl centre "
                             "landed at plate centre + (+0.0149, -0.0282) m, re-perceived from the "
                             "post-place cam_high frame", "allowed": True},
    "CARRY_Z": {"source": "debug seeds: plate and cookie box top at 0.920 m and the carried bowl "
                          "base hangs 0.040 m below the EEF, so an EEF at 0.985 m clears them by "
                          "0.025 m", "allowed": True},
    "HOVER_Z": {"source": "debug-seed heights: tallest prop tops at 0.952 m; 1.02 m clears it",
                "allowed": True},
    "REACH_GAIN": {"source": "v3 debug run: a move that cannot converge repeats the same residual "
                             "to 4 decimal places, so 0.002 m of improvement separates progress "
                             "from a stall", "allowed": True},
}

TABLE_Z = 0.900
RIM_BAND_Z = 0.946
LOW_BAND_Z = (0.908, 0.940)
WARM_MIN = 0.12
BOWL_SPAN = (0.085, 0.130)
PLATE_SPAN_MIN = 0.110
PLATE_TOP_MAX = 0.928
RIM_R = 0.050
GRASP_DZ = 0.029
RUNGS = ((0.050, 0.000), (0.050, 0.008), (0.045, 0.004))
MIN_BITE = 0.006
BOWL_HANG = 0.040
POS_TOL_BIAS = 0.011
CARRY_BIAS = (0.0149, -0.0282)
CARRY_Z = 0.985
HOVER_Z = 1.02
REACH_GAIN = 0.002

CHUNK = 1800


def _emit(api, name, arr):
    b64 = base64.b64encode(zlib.compress(np.ascontiguousarray(arr).tobytes(), 6)).decode()
    api.log("ARR %s %s %s %d" % (name, arr.dtype.str, list(arr.shape), len(b64)))
    for i in range(0, len(b64), CHUNK):
        api.log("D %s %d %s" % (name, i // CHUNK, b64[i:i + CHUNK]))


def _cloud(frame):
    d = np.asarray(frame.depth, float)
    d = np.where(np.isfinite(d) & (d > 0), d, 0.0)
    h, w = d.shape
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    vv, uu = np.mgrid[0:h, 0:w].astype(float)
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1) @ T.T
    return P[..., 0], P[..., 1], P[..., 2], d > 0


def _clusters(X, Y, Z, mask, tol=0.015, minn=60):
    px, py, pz = X[mask], Y[mask], Z[mask]
    if px.size == 0:
        return []
    g = np.round(np.stack([px, py], -1) / tol).astype(int)
    cell = defaultdict(list)
    for i, (a, b) in enumerate(g):
        cell[(a, b)].append(i)
    seen, out = set(), []
    for c in list(cell):
        if c in seen:
            continue
        stack, comp = [c], []
        seen.add(c)
        while stack:
            cc = stack.pop()
            comp += cell[cc]
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    nb = (cc[0] + da, cc[1] + db)
                    if nb in cell and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        if len(comp) < minn:
            continue
        a, b, c2 = px[comp], py[comp], pz[comp]
        out.append(dict(n=len(comp),
                        mid=(float((a.min() + a.max()) / 2), float((b.min() + b.max()) / 2)),
                        span=(float(a.max() - a.min()), float(b.max() - b.min())),
                        ztop=float(c2.max())))
    return out


def perceive(api, tag="scene"):
    f = api.capture("cam_high")
    X, Y, Z, ok = _cloud(f)
    rgb = np.asarray(f.rgb, float) / 255.0
    warm = rgb[..., 0] - rgb[..., 2]
    work = ok & (X > -0.32) & (X < 0.32) & (Y > -0.14) & (Y < 0.45)

    bowls = [c for c in _clusters(X, Y, Z, work & (Z > RIM_BAND_Z) & (Z < 1.00))
             if BOWL_SPAN[0] < max(c["span"]) < BOWL_SPAN[1]]
    boxes = _clusters(X, Y, Z, work & (Z > LOW_BAND_Z[0]) & (Z < LOW_BAND_Z[1])
                      & (warm > WARM_MIN))
    plates = []
    for c in _clusters(X, Y, Z, work & (Z > 0.910) & (Z < PLATE_TOP_MAX)):
        mx, my = c["mid"]
        fp = ok & (np.abs(X - mx) < 0.075) & (np.abs(Y - my) < 0.075)
        if fp.sum() and float(Z[fp].max()) <= PLATE_TOP_MAX and max(c["span"]) >= PLATE_SPAN_MIN:
            plates.append(c)

    box = max(boxes, key=lambda c: c["n"]) if boxes else None
    if box is not None and len(bowls) > 1:
        bowl = min(bowls, key=lambda c: (c["mid"][0] - box["mid"][0]) ** 2
                                        + (c["mid"][1] - box["mid"][1]) ** 2)
    else:
        bowl = max(bowls, key=lambda c: c["mid"][0]) if bowls else None
    plate = max(plates, key=lambda c: c["n"]) if plates else None
    api.log("%s bowl=%s plate=%s box=%s" %
            (tag, json.dumps(bowl), json.dumps(plate), json.dumps(box)))
    return bowl, plate, box, f


def _mv(api, tag, xyz, seconds):
    r = api.move(xyz, seconds=seconds)
    e = api.eef()
    api.log("mv %s tgt=%s got=%s res=%.4f" %
            (tag, json.dumps([round(float(v), 4) for v in xyz]),
             json.dumps([round(float(v), 4) for v in e]), r))
    return r


def _reach(api, tag, xyz, seconds=0.8, tries=4, gain=REACH_GAIN):
    """Drive to xyz, re-issuing while the residual keeps shrinking.

    A single long move is the wrong tool at the reach envelope: it cannot
    converge and it spends every step it is given, and each of those steps is
    another increment of gripper closure. Short moves plus a progress test stop
    within ~50 steps of the arm giving up.
    """
    prev = None
    r = 0.0
    for k in range(tries):
        r = _mv(api, "%s.%d" % (tag, k), xyz, seconds)
        if r < 0.012:
            break
        if prev is not None and (prev - r) < gain:
            api.log("%s stalled at res=%.4f" % (tag, r))
            break
        prev = r
    return r


def _g(api, tag):
    g = api.gripper()
    api.log("g %s w=%.4f eff=%.2f" % (tag, g["width_m"], g["effort"]))
    return g


def straddle(api, bowl, off_r, extra_dz, tag):
    cx, cy = bowl["mid"]
    tx, ty = cx, cy + off_r
    gz = bowl["ztop"] - GRASP_DZ - extra_dz
    api.grip(0.08)
    _mv(api, tag + ".hover", [tx, ty, HOVER_Z], 2.0)
    _mv(api, tag + ".down", [tx, ty, gz], 1.2)
    api.grip(0.0)
    _g(api, tag + ".closed")
    _mv(api, tag + ".lift", [tx, ty, CARRY_Z], 1.2)
    return _g(api, tag + ".lifted")


def run(api):
    api.log("instr=%r" % api.instruction())
    bowl, plate, box, f0 = perceive(api)
    if bowl is None or plate is None:
        return "v4 perception failed"

    g = {"width_m": 0.0, "effort": 0.0}
    for i, (off_r, dz) in enumerate(RUNGS):
        g = straddle(api, bowl, off_r, dz, "R%d" % i)
        if g["width_m"] >= MIN_BITE:
            break
    api.log("bite=%.4f eff=%.2f" % (g["width_m"], g["effort"]))

    px = plate["mid"][0] - CARRY_BIAS[0]
    py = plate["mid"][1] - CARRY_BIAS[1]
    _reach(api, "carry", [px, py, CARRY_Z])
    _g(api, "over-plate")

    rz = plate["ztop"] + BOWL_HANG - POS_TOL_BIAS
    _reach(api, "place", [px, py, rz], seconds=0.8, tries=3)
    _g(api, "pre-release")
    api.grip(0.08)
    api.settle(0.5)
    _mv(api, "clear", [px, py, HOVER_Z], 1.2)

    _, _, _, f2 = perceive(api, "post")
    _emit(api, "post_rgb", np.asarray(f2.rgb)[::2, ::2].astype(np.uint8))
    d = np.nan_to_num(np.asarray(f2.depth, float))[::2, ::2]
    _emit(api, "post_dep", np.clip(d * 10000.0, 0, 65535).astype(np.uint16))
    return "v4 bite=%.4f" % g["width_m"]
