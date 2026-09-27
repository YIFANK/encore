"""spa_bowl_table_center_pos -- perceive both bowls + the plate from cam_high,
rim-pinch the table-centre bowl, carry it over the plate, release.

Mechanism (all of it read off the K=3 pack and re-measured on debug seeds):
  * the two black bowls are the ONLY props whose top rises >44 mm above the
    table plane; each deprojects to a 0.111 x 0.111 m footprint.
  * "table center" = the bowl nearest the pack's demo grasp anchor; on every
    debug seed the runner-up is >3.3x further away.
  * the plate is the unique bright, >0.10 m wide component whose top sits
    12-26 mm above the table.
  * the gripper cannot span a 0.111 m bowl (max opening 0.079 m), so the demos
    pinch the rim wall: the demo EEF closes 0.048 m from the bowl centre along
    +y and 34 mm below the rim top.  The bowl therefore hangs at
    eef - GRASP_OFF, so the release pose is plate_centre + GRASP_OFF.
"""
import os
import numpy as np

try:
    from scipy import ndimage as _nd
except Exception:                                    # pragma: no cover
    _nd = None


def _label(mask):
    """4-connected connected components; pure-numpy fallback for scipy.label."""
    if _nd is not None:
        return _nd.label(mask)
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    parent = [0]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    nxt = 1
    for i in range(H):
        row = mask[i]
        if not row.any():
            continue
        for j in np.flatnonzero(row):
            up = lab[i - 1, j] if i else 0
            left = lab[i, j - 1] if j else 0
            if up and left:
                lab[i, j] = min(up, left)
                union(up, left)
            elif up or left:
                lab[i, j] = up or left
            else:
                lab[i, j] = nxt
                parent.append(nxt)
                nxt += 1
    if nxt == 1:
        return lab, 0
    root = np.array([find(k) for k in range(nxt)], np.int32)
    uniq, comp = np.unique(root[1:], return_inverse=True)
    remap = np.zeros(nxt, np.int32)
    remap[1:] = comp + 1
    lab = remap[lab]
    return lab, int(len(uniq))

PROVENANCE = {
    "WS": {"source": "debug-seed 51-65 cam_high deprojection: table-top extent that "
                     "excludes the back wall and the camera's far clutter",
           "allowed": True},
    "DEMO_ANCHOR": {"source": "pack.json demos 0-2 ee_path descent minima, mean of "
                              "(-0.091,0.041)/(-0.089,0.044)/(-0.072,0.056)",
                    "allowed": True},
    "BOWL_BAND": {"source": "debug seeds 51-65: bowl tops deproject to table+0.0512, "
                            "every other prop top is <= table+0.044",
                  "allowed": True},
    "BOWL_SIZE": {"source": "debug seeds 51-65: both bowls deproject to a "
                            "0.109-0.111 m square footprint",
                  "allowed": True},
    "PLATE_BAND": {"source": "debug seeds 51-65: plate top deprojects to table+0.0192; "
                             "band table+0.012..+0.028 with brightness>125 and "
                             "width>0.10 isolates it (cookie box brightness 68)",
                   "allowed": True},
    "GRASP_OFF": {"source": "pack.json demo grasp EEF minus the bowl centre measured "
                            "on debug seeds (demo mean (-0.084,0.047); bowl centre "
                            "(-0.076,0.000)) -> (-0.008,+0.047)",
                  "allowed": True},
    "GRASP_DZ": {"source": "debug seeds 51-65 v1: the pack-derived -0.034 bite "
                           "(close gap 0.0122) always slipped on the lift; the "
                           "-0.040 retry (close gap 0.0091) survived 3/8",
                 "allowed": True},
    "PLACE_DZ": {"source": "pack.json demos release EEF z 0.931-0.945 vs the plate "
                           "top 0.920 measured on debug seeds -> +0.022",
                 "allowed": True},
    "CARRY_Z": {"source": "pack.json demos 0-2 ee_path apex between grasp and release "
                          "(1.019-1.069)",
                "allowed": True},
    "OPEN_W": {"source": "pack.json keyframe gripper_state open finger 0.0393 -> "
                         "0.0786 m jaw opening",
               "allowed": True},
    "HOLD_GAP_MIN": {"source": "pack.json demo hold gripper_state widths 0.0105-0.0157; "
                               "debug-seed v0 run: a failed grasp read 0.0048 after the "
                               "lift while a held one stays above 0.008",
                     "allowed": True},
}

WS = (-0.32, 0.32, -0.30, 0.42)
DEMO_ANCHOR = np.array([-0.084, 0.047])
BOWL_BAND = (0.044, 0.15)
BOWL_SIZE = (0.08, 0.15)
PLATE_BAND = (0.012, 0.028)
PLATE_BRIGHT = 125.0
PLATE_MIN_W = 0.10
GRASP_OFF = np.array([-0.008, 0.047])
GRASP_DZ = -0.046
PLACE_DZ = 0.022
CARRY_Z = 1.05
OPEN_W = 0.08
HOLD_GAP_MIN = 0.006

DUMP = "/mnt/data/YifanKang/Heron/results/probe_c2clean_spa_bowl_table_center_pos_k3"


def _cloud(f):
    depth = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = depth.shape
    v, u = np.mgrid[0:H, 0:W]
    d = np.where(np.isfinite(depth) & (depth > 0), depth, np.nan)
    x = (u - K[0, 2]) * d / K[0, 0]
    y = (v - K[1, 2]) * d / K[1, 1]
    B = np.stack([x, y, d, np.ones_like(d)], -1) @ T.T
    return B[..., 0], B[..., 1], B[..., 2]


def perceive(api):
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb, float)
    X, Y, Z = _cloud(f)
    ws = ((X > WS[0]) & (X < WS[1]) & (Y > WS[2]) & (Y < WS[3]) & np.isfinite(Z))
    zs = Z[ws]
    hist, edges = np.histogram(zs[(zs > 0.6) & (zs < 1.6)], bins=400)
    table = float(edges[int(hist.argmax())])

    def comps(lo, hi, minpx):
        lab, n = _label(ws & (Z > lo) & (Z < hi))
        out = []
        for i in range(1, n + 1):
            m = lab == i
            k = int(m.sum())
            if k < minpx:
                continue
            out.append(dict(n=k,
                            mx=float((X[m].min() + X[m].max()) / 2),
                            my=float((Y[m].min() + Y[m].max()) / 2),
                            ztop=float(Z[m].max()),
                            dx=float(np.ptp(X[m])), dy=float(np.ptp(Y[m])),
                            bright=float(rgb[m].mean())))
        return out

    bowls = [c for c in comps(table + BOWL_BAND[0], table + BOWL_BAND[1], 200)
             if c["n"] < 5000
             and BOWL_SIZE[0] < c["dx"] < BOWL_SIZE[1]
             and BOWL_SIZE[0] < c["dy"] < BOWL_SIZE[1]]
    plates = [c for c in comps(table + PLATE_BAND[0], table + PLATE_BAND[1], 600)
              if c["ztop"] < table + PLATE_BAND[1] - 0.002
              and c["dx"] > PLATE_MIN_W and c["dy"] > PLATE_MIN_W
              and c["bright"] > PLATE_BRIGHT]
    return table, bowls, plates



STAGE_DZ = 0.015
STAGE_N = 3
MAX_TRIES = 3


def _grasp_once(api, R, tgt, dz):
    gx = tgt["mx"] + GRASP_OFF[0]
    gy = tgt["my"] + GRASP_OFF[1]
    gz = tgt["ztop"] + dz
    api.grip(OPEN_W)
    api.move([gx, gy, CARRY_Z], rotation=R, seconds=3.0)
    rd = api.move([gx, gy, gz], rotation=R, seconds=3.0)
    e = api.eef()
    api.grip(0.0)
    api.settle(0.3)
    g0 = api.gripper()
    api.grip(0.0)
    api.settle(0.3)
    g1 = api.gripper()
    api.log("descend to (%.3f,%.3f,%.3f) res=%.4f eef=%s close=%.4f/%.1f resqueeze=%.4f/%.1f"
            % (gx, gy, gz, rd, np.round(e, 4).tolist(),
               g0["width_m"], g0["effort"], g1["width_m"], g1["effort"]))
    z = gz
    for k in range(STAGE_N):
        z += STAGE_DZ
        api.move([gx, gy, z], rotation=R, seconds=1.5)
        api.grip(0.0)
        g = api.gripper()
        api.log("  stage %d z=%.3f gap=%.4f effort=%.1f" % (k, z, g["width_m"], g["effort"]))
        if g["effort"] < 2.5 and g["width_m"] < HOLD_GAP_MIN:
            return False, (gx, gy)
    api.move([gx, gy, CARRY_Z], rotation=R, seconds=2.5)
    g = api.gripper()
    api.log("  after carry-lift gap=%.4f effort=%.1f" % (g["width_m"], g["effort"]))
    return bool(g["effort"] >= 2.5 or g["width_m"] > HOLD_GAP_MIN), (gx, gy)


def run(api):
    api.log("instruction=%r GRASP_DZ=%.3f" % (api.instruction(), GRASP_DZ))
    R = np.asarray(api.tool_rotation(), float)

    held = False
    plate = None
    for attempt in range(MAX_TRIES):
        table, bowls, plates = perceive(api)
        api.log("try %d table=%.4f bowls=%s plates=%s"
                % (attempt, table,
                   [(round(b["mx"], 3), round(b["my"], 3), round(b["ztop"], 3), b["n"])
                    for b in bowls],
                   [(round(p["mx"], 3), round(p["my"], 3), p["n"]) for p in plates]))
        if plates:
            plate = max(plates, key=lambda p: p["n"])
        if not bowls:
            break
        bowls.sort(key=lambda c: float(np.hypot(c["mx"] - DEMO_ANCHOR[0],
                                                c["my"] - DEMO_ANCHOR[1])))
        tgt = bowls[0]
        api.log("try %d target=(%.3f,%.3f,%.3f)" % (attempt, tgt["mx"], tgt["my"], tgt["ztop"]))
        held, _ = _grasp_once(api, R, tgt, GRASP_DZ)
        if held:
            break
        api.grip(OPEN_W)
        api.move([tgt["mx"] + GRASP_OFF[0], tgt["my"] + GRASP_OFF[1], CARRY_Z],
                 rotation=R, seconds=2.0)
    api.log("held=%s after %d tries" % (held, attempt + 1))
    if plate is None:
        return "no plate perceived"

    px = plate["mx"] + GRASP_OFF[0]
    py = plate["my"] + GRASP_OFF[1]
    pz = plate["ztop"] + PLACE_DZ
    api.move([px, py, CARRY_Z], rotation=R, seconds=3.0)
    r2 = api.move([px, py, pz], rotation=R, seconds=3.0)
    api.log("place (%.3f,%.3f,%.3f) res=%.4f eef=%s gap=%.4f"
            % (px, py, pz, r2, np.round(api.eef(), 4).tolist(), api.gripper()["width_m"]))
    api.grip(OPEN_W)
    api.settle(0.6)
    api.move([px, py, CARRY_Z], rotation=R, seconds=2.0)
    api.settle(0.5)
    return "held=%s" % held
