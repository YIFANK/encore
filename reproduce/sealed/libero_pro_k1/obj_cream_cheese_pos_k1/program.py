"""c2k1clean / obj_cream_cheese_pos_k1 -- v2

Perceive -> identify the cream cheese (the one FLAT, BLUE box) and the basket
(the largest above-table cluster) from cam_high RGB-D, pinch the box top-down,
carry it over the basket rim and release.
"""
import os
import numpy as np

PROVENANCE = {
    "WS_X": {"source": "debug seeds 51-65 cam_high cloud: all table objects lie in x[-0.20,+0.20]; crop widened to the visible table", "allowed": True},
    "WS_Y": {"source": "debug seeds 51-65 cam_high cloud: all objects lie in y[-0.28,+0.36]; crop widened to the visible table", "allowed": True},
    "Z_LO": {"source": "debug seeds 51-65: table plane mode at z=0.005; 12 mm clears plane noise (measured min object ztop 0.0188)", "allowed": True},
    "Z_HI": {"source": "debug seeds 51-65: tallest scene object ztop=0.1438 (basket rim); arm at home starts above 0.19, so 0.165 excludes the arm (7 clusters, no arm, in all 15 seeds)", "allowed": True},
    "CELL": {"source": "debug seeds 51-65: 15 mm xy grid separates all 7 clusters (closest pair 0.09 m apart)", "allowed": True},
    "MIN_PTS": {"source": "debug seeds 51-65: smallest real cluster is the blue box at 508-514 px; 80 rejects speckle", "allowed": True},
    "TOP_BAND": {"source": "debug seeds 51-65: 5 mm band below ztop isolates the top face of every object", "allowed": True},
    "FLAT_ZTOP_MAX": {"source": "debug seeds 51-65: the two flat boxes measure ztop 0.0188/0.0201; next tallest object is 0.081 -> 0.035 is a clean cut", "allowed": True},
    "BLUE_MARGIN": {"source": "debug seeds 51-65 top-face mean RGB: blue box B-R = +23..+24, orange box B-R = -60..-61", "allowed": True},
    "FALLBACK_BOX_XY": {"source": "debug seeds 51-65 mean measured blue-box top-face centroid (-0.1443, 0.0580)", "allowed": True},
    "FALLBACK_BASKET_XY": {"source": "debug seeds 51-65 mean measured basket rim centroid (0.0061, 0.2648)", "allowed": True},
    "GRASP_DZ": {"source": "pack demo0 keyframe t=54 closes the gripper at ee z=0.0091; the box top measures ztop=0.0201 on debug seeds -> grip point sits 0.011 below the top face", "allowed": True},
    "GRASP_Z_PUSH": {"source": "heron move_cartesian stops at POS_TOL=0.012; commanding 0.012 lower makes the tolerance sphere land at or below the demo grip height (debug-seed descent logs)", "allowed": True},
    "HOVER_Z": {"source": "debug seeds 51-65: tallest object 0.1438; 0.19 clears every object and the loaded gripper", "allowed": True},
    "CARRY_Z": {"source": "pack demo0 ee_path6 carries at z=0.24-0.25 between grasp and release", "allowed": True},
    "RELEASE_DZ": {"source": "pack demo0 keyframe t=120 opens at ee z=0.1789; basket rim measures ztop=0.1437 on debug seeds -> release 0.035 above the rim", "allowed": True},
    "OPEN_W": {"source": "heron set_gripper: width >= 0.025 opens; measured home finger gap 0.0778", "allowed": True},
    "CLOSE_W": {"source": "heron set_gripper: width < 0.025 closes", "allowed": True},
    "HELD_GAP_MIN": {"source": "debug measurement: the blue box is 0.042 wide across the jaw (base y) axis; a gap above 0.015 after a close means fingers are on the box, not on each other", "allowed": True},
    "XY_TOL": {"source": "controller mechanics: move_cartesian's POS_TOL is 0.012, so an xy refine loop is needed to get inside the 0.042 m jaw window", "allowed": True},
    "REFINE_N": {"source": "debug-seed residual logs: xy converges in <=3 re-commands", "allowed": True},
    "DBG_DIR": {"source": "coordinator-declared results dir for this cell", "allowed": True},
}

WS_X = (-0.40, 0.45)
WS_Y = (-0.55, 0.55)
Z_LO, Z_HI = 0.012, 0.165
CELL = 0.015
MIN_PTS = 80
TOP_BAND = 0.005
FLAT_ZTOP_MAX = 0.035
BLUE_MARGIN = 8.0
FALLBACK_BOX_XY = (-0.1443, 0.0580)
FALLBACK_BASKET_XY = (0.0061, 0.2648)
GRASP_DZ = -0.011
GRASP_Z_PUSH = 0.012
HOVER_Z = 0.19
CARRY_Z = 0.25
RELEASE_DZ = 0.035
OPEN_W = 0.08
CLOSE_W = 0.0
HELD_GAP_MIN = 0.015
XY_TOL = 0.004
REFINE_N = 3

DBG_DIR = "/mnt/data/YifanKang/Heron/results/dbg_c2k1clean_obj_cream_cheese_pos_k1"


# ---------------------------------------------------------------- perception
def cloud(frame):
    d = np.asarray(frame.depth, float)
    H, W = d.shape[:2]
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    vv, uu = np.mgrid[0:H, 0:W]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    P = (np.stack([x, y, d, np.ones_like(d)], -1) @ T.T)[..., :3]
    return P, np.isfinite(d) & (d > 0)


def segment(P, ok, rgb):
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    m = (ok & (x > WS_X[0]) & (x < WS_X[1]) & (y > WS_Y[0]) & (y < WS_Y[1])
         & (z > Z_LO) & (z < Z_HI))
    pts = np.stack([x[m], y[m], z[m]], 1)
    cols = rgb[m].astype(float)
    if len(pts) == 0:
        return []
    key = np.floor(pts[:, :2] / CELL).astype(int)
    occ = {}
    for i, k in enumerate(map(tuple, key)):
        occ.setdefault(k, []).append(i)
    seen, out = set(), []
    for k in occ:
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            c = stack.pop()
            comp.append(c)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in occ and n not in seen:
                        seen.add(n)
                        stack.append(n)
        ii = np.concatenate([occ[c] for c in comp])
        if len(ii) < MIN_PTS:
            continue
        p, c = pts[ii], cols[ii]
        ztop = float(p[:, 2].max())
        sel = p[:, 2] > ztop - TOP_BAND
        top = p[sel][:, :2]
        ctr = top.mean(0)
        ext = top.max(0) - top.min(0)
        out.append({"n": int(len(ii)), "ztop": ztop, "ctr": ctr, "ext": ext,
                    "rgb": c[sel].mean(0)})
    return out


# -------------------------------------------------------------------- motion
def goto(api, xyz, seconds=2.0):
    return float(api.move(np.asarray(xyz, float), seconds=seconds))


def refine_xy(api, tx, ty, z, api_log):
    """move_cartesian stops anywhere inside a 12 mm ball; re-command until the
    achieved xy is inside XY_TOL of the target."""
    for i in range(REFINE_N):
        e = api.eef()
        dx, dy = tx - e[0], ty - e[1]
        if abs(dx) < XY_TOL and abs(dy) < XY_TOL:
            break
        # push the command past the target so the remaining error exceeds the
        # controller's own stop tolerance and it keeps driving.
        goto(api, [tx + dx, ty + dy, z], seconds=1.2)
        e = api.eef()
        api_log(f"refine{i} -> eef={np.round(e,4).tolist()}")


def run(api):
    log = api.log
    try:
        os.makedirs(DBG_DIR, exist_ok=True)
    except Exception:
        pass
    tag = os.path.basename(os.environ.get("HOME", "ep"))

    f = api.capture("cam_high")
    P, ok = cloud(f)
    cl = segment(P, ok, f.rgb)
    log(f"nclusters={len(cl)}")
    for c in cl:
        log("cl n=%d ctr=(%+.4f,%+.4f) ztop=%.4f ext=(%.3f,%.3f) rgb=(%.0f,%.0f,%.0f)"
            % (c["n"], c["ctr"][0], c["ctr"][1], c["ztop"], c["ext"][0], c["ext"][1],
               c["rgb"][0], c["rgb"][1], c["rgb"][2]))

    flat = [c for c in cl if c["ztop"] < FLAT_ZTOP_MAX]
    blue = [c for c in flat if c["rgb"][2] - c["rgb"][0] > BLUE_MARGIN]
    if len(blue) == 1:
        box = blue[0]
    elif flat:
        box = max(flat, key=lambda c: c["rgb"][2] - c["rgb"][0])
        log("WARN: blue count=%d, fell back to bluest flat" % len(blue))
    else:
        box = {"ctr": np.array(FALLBACK_BOX_XY), "ztop": 0.0201}
        log("WARN: no flat cluster, using fallback xy")
    bx, by = float(box["ctr"][0]), float(box["ctr"][1])
    bz = float(box["ztop"])

    if cl:
        basket = max(cl, key=lambda c: c["n"])
        kx, ky, kz = float(basket["ctr"][0]), float(basket["ctr"][1]), float(basket["ztop"])
    else:
        kx, ky = FALLBACK_BASKET_XY
        kz = 0.1437
        log("WARN: no basket cluster, using fallback xy")
    log(f"TARGET box=({bx:.4f},{by:.4f},{bz:.4f}) basket=({kx:.4f},{ky:.4f},{kz:.4f})")

    try:
        np.savez_compressed(os.path.join(DBG_DIR, f"{tag}_v2.npz"),
                            rgb=f.rgb, P=P.astype(np.float32), ok=ok)
    except Exception:
        pass

    # --- approach ---------------------------------------------------------
    api.grip(OPEN_W)
    r = goto(api, [bx, by, HOVER_Z], seconds=2.5)
    log(f"hover residual={r:.4f} eef={np.round(api.eef(),4).tolist()}")
    refine_xy(api, bx, by, HOVER_Z, log)

    # --- descend ----------------------------------------------------------
    gz = bz + GRASP_DZ
    r = goto(api, [bx, by, gz], seconds=2.0)
    log(f"descend1 residual={r:.4f} eef={np.round(api.eef(),4).tolist()}")
    r = goto(api, [bx, by, gz - GRASP_Z_PUSH], seconds=1.5)
    e = api.eef()
    log(f"descend2 residual={r:.4f} eef={np.round(e,4).tolist()} target_gz={gz:.4f}")

    # --- grasp ------------------------------------------------------------
    api.grip(CLOSE_W)
    api.settle(0.3)
    g = api.gripper()
    log(f"after close gap={g['width_m']:.4f} effort={g['effort']}")

    # --- lift -------------------------------------------------------------
    goto(api, [bx, by, CARRY_Z], seconds=2.5)
    g = api.gripper()
    log(f"after lift gap={g['width_m']:.4f} effort={g['effort']} eef={np.round(api.eef(),4).tolist()}")

    # --- carry & release --------------------------------------------------
    goto(api, [kx, ky, CARRY_Z], seconds=3.0)
    log(f"over basket eef={np.round(api.eef(),4).tolist()}")
    refine_xy(api, kx, ky, CARRY_Z, log)
    rz = kz + RELEASE_DZ
    goto(api, [kx, ky, rz], seconds=1.5)
    log(f"at release eef={np.round(api.eef(),4).tolist()}")
    api.grip(OPEN_W)
    api.settle(0.5)
    goto(api, [kx, ky, CARRY_Z], seconds=1.5)
    api.settle(0.3)

    try:
        f2 = api.capture("cam_high")
        np.savez_compressed(os.path.join(DBG_DIR, f"{tag}_v2_end.npz"), rgb=f2.rgb)
    except Exception:
        pass
    return "v2"
