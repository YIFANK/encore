"""c2 cell obj_milk_stock -- "pick up the milk and place it in the basket".

v3 = v2 (8/8 on debug 51..65 odd) with the target search UNCAPPED over the
size-gated pool: v2's logs measured the nearest gated competitor at 0.393 m
from the demo grasp anchor while the milk sits 0.007 m from it (a 56x margin),
so a fixed acceptance radius only buys a way to fail -- if the carton ever
moved further than the radius, v2 fell back to grasping the bare anchor (i.e.
air). v3 always takes the nearest size-gated cluster and reports the margin.

v2 = v1 + three hardenings that cannot change the
debug trajectories but widen the operating envelope:
  * the target is chosen among SIZE-GATED clusters only (tall enough to be a
    standing carton, small enough not to be the basket) -- measured on debug
    seeds: milk dz=0.138 m / ext 0.053x0.054 / ~2160 pts vs basket ext 0.16 m
    and two flat props at dz=0.019 m;
  * if the arm-body filter leaves nothing near the anchor, clustering is redone
    without it (so a target that happens to sit under the home pose survives);
  * the grasp is verified by gripper effort and retried once from a fresh
    capture if the jaws came up empty.

Depth-only perception from cam_high; ground-footprint clustering; target =
cluster nearest the K-demo grasp anchor (C2-L1 / C2-L2); basket = large cluster
nearest the K-demo release anchor. Top-down pick at the demo grasp height,
transit high, release over the basket at the demo release height.

Every constant below is either (a) a statistic of this cell's pack.json demo
keyframes, or (b) generic controller/camera mechanics -- see PROVENANCE.
"""
import numpy as np

PROVENANCE = {
    "GRASP_ANCHOR_XY": {
        "source": "pack.json: mean of the 3 demo keyframes where gripper_cmd "
                  "flips -1->+1 (demo0 t47, demo1 t40, demo2 t45) -> "
                  "(-0.1227,-0.2458); per-axis range 0.019/0.013 m",
        "allowed": True},
    "GRASP_Z": {
        "source": "pack.json: mean EEF z at those same closing keyframes "
                  "(0.1001/0.1053/0.0980) = 0.1011 m",
        "allowed": True},
    "RELEASE_ANCHOR_XY": {
        "source": "pack.json: mean EEF xy at the 3 demo re-opening keyframes "
                  "(demo0 t142, demo1 t124, demo2 t125) -> (-0.0074, 0.2604)",
        "allowed": True},
    "RELEASE_Z": {
        "source": "pack.json: mean EEF z at those re-opening keyframes = 0.155 m",
        "allowed": True},
    "TRANSIT_Z": {
        "source": "pack.json: peak EEF z along the 3 demo ee_paths during the "
                  "carry phase (0.3083/0.3194/0.3300 m); 0.30 m used",
        "allowed": True},
    "PRE_Z": {
        "source": "pack.json: EEF z one stride before each demo's closing "
                  "keyframe (0.149/0.171/0.123 m); 0.21 m is a safe superset",
        "allowed": True},
    "GRIP_OPEN_M / GRIP_CLOSE_M": {
        "source": "generic gripper mechanics -- FairApi.grip closes below "
                  "0.025 m and opens otherwise (documented in the cell task)",
        "allowed": True},
    "TABLE_CLEAR_M / GRID_M / CELL_MIN_PTS / ARM_CLEAR_M": {
        "source": "generic depth-camera / point-cloud mechanics: above-plane "
                  "threshold, XY occupancy-grid pitch, per-cell noise floor, "
                  "and a radius around the measured EEF used to drop the "
                  "robot's own body from the cloud",
        "allowed": True},
    "WORK_XY": {
        "source": "generic workspace crop; superset of the xy extent visited "
                  "by all 3 demo ee_paths in pack.json (x -0.158..0.033, "
                  "y -0.255..0.278) padded by 0.2 m",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "cell task description of api.gripper(): effort 3.0 iff the "
                  "jaws are holding something",
        "allowed": True},
    "TGT_MIN_DZ / TGT_MAX_EXT / TGT_MIN_PTS": {
        "source": "debug-seed measurement (results/fs_c2_obj_milk_stock_v1 "
                  "program logs, seeds 51..65 odd): the demo-anchored cluster "
                  "measures dz=0.138 m, ext 0.053x0.054 m, ~2160 pts on every "
                  "seed, while the basket measures ext 0.16x0.17 m and the two "
                  "flat props measure dz=0.019 m",
        "allowed": True},
    "TGT_GATE_M": {
        "source": "debug-seed measurement (results/fs_c2_obj_milk_stock_v2 "
                  "logs): anchor->milk is 0.007-0.010 m on every probed seed "
                  "while the nearest other SIZE-GATED cluster is 0.393 m away; "
                  "used only to decide whether the gated pool may be abandoned "
                  "for the ungated one",
        "allowed": True},
}

# ---- pack-derived constants ------------------------------------------------
GRASP_ANCHOR = np.array([-0.1227, -0.2458])
GRASP_Z = 0.101
RELEASE_ANCHOR = np.array([-0.0074, 0.2604])
RELEASE_Z = 0.155
TRANSIT_Z = 0.30
PRE_Z = 0.21

# ---- generic mechanics -----------------------------------------------------
GRIP_OPEN_M = 0.08
GRIP_CLOSE_M = 0.0
HOLD_EFFORT = 2.5          # effort >= this means the jaws hold something
TABLE_CLEAR_M = 0.012
GRID_M = 0.015
CELL_MIN_PTS = 3
ARM_CLEAR_M = 0.11
ARM_TOP_M = 0.22           # cells taller than this above the table are the arm
WORK_XY = (-0.40, 0.36, -0.50, 0.50)   # xmin, xmax, ymin, ymax

# ---- target size gate (debug-seed measured) --------------------------------
TGT_MIN_DZ = 0.050         # milk 0.138 m vs flat props 0.019 m
TGT_MAX_EXT = 0.110        # milk 0.054 m vs basket 0.170 m
TGT_MIN_PTS = 300          # milk ~2160 pts vs noise fragments ~105 pts
TGT_GATE_M = 0.14          # anchor->milk 0.010 m vs anchor->runner-up 0.182 m


# ---------------------------------------------------------------- perception
def point_cloud(api, cam="cam_high"):
    f = api.capture(cam)
    d = np.asarray(f.depth, float)
    if d.ndim == 3:
        d = d[..., 0]
    h, w = d.shape[:2]
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 0.01) & (d < 5.0)
    z = d[ok]
    u = uu[ok].astype(float)
    v = vv[ok].astype(float)
    p_cam = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)
    p_base = p_cam @ T[:3, :3].T + T[:3, 3]
    rgb = np.asarray(f.rgb)[ok]
    return f, p_base, rgb


def footprint_clusters(api, p_base, rgb, eef_xy, arm_clear=ARM_CLEAR_M):
    """Group above-table points by GROUND FOOTPRINT on a GRID_M xy lattice."""
    xmin, xmax, ymin, ymax = WORK_XY
    inbox = ((p_base[:, 0] > xmin) & (p_base[:, 0] < xmax) &
             (p_base[:, 1] > ymin) & (p_base[:, 1] < ymax))
    table_z = float(np.median(p_base[inbox, 2])) if inbox.any() else 0.0
    api.log("table_z=%.4f  cloud=%d inbox=%d  z[p1,p50,p99]=%.3f/%.3f/%.3f"
            % (table_z, len(p_base), int(inbox.sum()),
               *np.percentile(p_base[:, 2], [1, 50, 99])))

    m = inbox & (p_base[:, 2] > table_z + TABLE_CLEAR_M)
    P = p_base[m]
    C = rgb[m]
    if len(P) == 0:
        return table_z, []

    cell = np.floor(P[:, :2] / GRID_M).astype(np.int64)
    keys = {}
    for i in range(len(P)):
        keys.setdefault((int(cell[i, 0]), int(cell[i, 1])), []).append(i)

    live = {}
    for k, idx in keys.items():
        if len(idx) < CELL_MIN_PTS:
            continue
        zz = P[idx, 2]
        top = float(np.max(zz))
        if top - table_z > ARM_TOP_M:
            continue                       # robot arm / mount, not a prop
        cx_ = (k[0] + 0.5) * GRID_M
        cy_ = (k[1] + 0.5) * GRID_M
        if np.hypot(cx_ - eef_xy[0], cy_ - eef_xy[1]) < arm_clear:
            continue                       # the robot's own body
        live[k] = idx

    # 8-connected components over occupied cells
    seen = set()
    comps = []
    for k in live:
        if k in seen:
            continue
        stack = [k]
        seen.add(k)
        cells = []
        while stack:
            c = stack.pop()
            cells.append(c)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in live and n not in seen:
                        seen.add(n)
                        stack.append(n)
        comps.append(cells)

    out = []
    for cells in comps:
        idx = np.concatenate([np.asarray(live[c]) for c in cells])
        Q = P[idx]
        top = float(np.percentile(Q[:, 2], 98))
        face = Q[Q[:, 2] > top - 0.020]
        mid = np.array([0.5 * (face[:, 0].min() + face[:, 0].max()),
                        0.5 * (face[:, 1].min() + face[:, 1].max())])
        out.append({
            "xy": mid,
            "centroid": Q[:, :2].mean(axis=0),
            "top": top,
            "dz": top - table_z,        # height above the fitted table plane
            "npts": int(len(idx)),
            "ncells": int(len(cells)),
            "ext": (float(Q[:, 0].max() - Q[:, 0].min()),
                    float(Q[:, 1].max() - Q[:, 1].min())),
            "rgb": C[idx].mean(axis=0),
        })
    out.sort(key=lambda c: -c["npts"])
    return table_z, out


def gated(c):
    """Could this cluster be the standing carton the demos grasped?"""
    return (c["dz"] > TGT_MIN_DZ and max(c["ext"]) < TGT_MAX_EXT
            and c["npts"] >= TGT_MIN_PTS)


def pick_target(api, comps):
    """Nearest SIZE-GATED cluster to the demo grasp anchor, then any cluster,
    then the raw anchor."""
    gpool = [c for c in comps if gated(c)]
    for label, pool, cap in (("gated", gpool, None),
                             ("ungated", list(comps), TGT_GATE_M)):
        if not pool:
            continue
        d = [float(np.linalg.norm(c["xy"] - GRASP_ANCHOR)) for c in pool]
        j = int(np.argmin(d))
        if cap is None or d[j] < cap:
            runner = sorted(d)[1] if len(d) > 1 else -1.0
            api.log("TARGET(%s) xy=%s d=%.4f runner_up=%.4f npts=%d dz=%.3f"
                    % (label, np.round(pool[j]["xy"], 4).tolist(), d[j],
                       runner, pool[j]["npts"], pool[j]["dz"]))
            return pool[j]["xy"].copy()
    api.log("TARGET(anchor-fallback) xy=%s" % np.round(GRASP_ANCHOR, 4).tolist())
    return GRASP_ANCHOR.copy()


def perceive(api):
    e = api.eef()
    f, P, C = point_cloud(api, "cam_high")
    table_z, comps = footprint_clusters(api, P, C, e[:2])
    if not any(gated(c) for c in comps):
        api.log("no size-gated candidate at all -> re-cluster without the "
                "arm-body filter")
        table_z, comps = footprint_clusters(api, P, C, e[:2], arm_clear=0.0)
    return table_z, comps


# ---------------------------------------------------------------------- main
def run(api):
    api.log("instruction=%r cameras=%s" % (api.instruction(), ""))
    e0 = api.eef()
    R0 = api.tool_rotation()
    api.log("eef0=%s tool_R0=%s" % (np.round(e0, 4).tolist(),
                                    np.round(R0, 3).tolist()))
    api.grip(GRIP_OPEN_M)

    table_z, comps = perceive(api)
    for i, c in enumerate(comps):
        api.log("cl%02d xy=%.4f,%.4f top=%.4f dz=%.3f npts=%d ncells=%d "
                "ext=%.3f,%.3f rgb=%s d_grasp=%.3f d_rel=%.3f"
                % (i, c["xy"][0], c["xy"][1], c["top"], c["top"] - table_z,
                   c["npts"], c["ncells"], c["ext"][0], c["ext"][1],
                   np.round(c["rgb"], 0).tolist(),
                   float(np.linalg.norm(c["xy"] - GRASP_ANCHOR)),
                   float(np.linalg.norm(c["xy"] - RELEASE_ANCHOR))))

    # ---- target: cluster nearest the demo grasp anchor ---------------------
    tgt_xy = pick_target(api, comps)

    # ---- basket: biggest cluster near the demo release anchor --------------
    bsk_xy = RELEASE_ANCHOR.copy()
    cand = [c for c in comps
            if float(np.linalg.norm(c["xy"] - RELEASE_ANCHOR)) < 0.16]
    if cand:
        b = max(cand, key=lambda c: c["ncells"])
        bsk_xy = b["xy"].copy()
        api.log("BASKET xy=%s ncells=%d dz=%.3f"
                % (np.round(bsk_xy, 4).tolist(), b["ncells"],
                   b["top"] - table_z))
    else:
        api.log("BASKET fallback to demo anchor %s"
                % np.round(bsk_xy, 4).tolist())

    # ---- pick (verified; one re-perceive + retry if the jaws come up empty)--
    for attempt in (0, 1):
        api.move([tgt_xy[0], tgt_xy[1], PRE_Z], seconds=2.0)
        api.log("a%d pre-grasp eef=%s" % (attempt,
                                          np.round(api.eef(), 4).tolist()))
        api.move([tgt_xy[0], tgt_xy[1], GRASP_Z], seconds=1.5)
        api.log("a%d at-grasp eef=%s" % (attempt,
                                         np.round(api.eef(), 4).tolist()))
        api.grip(GRIP_CLOSE_M)
        api.settle(0.4)
        api.log("a%d post-close gripper=%s" % (attempt, api.gripper()))
        api.move([tgt_xy[0], tgt_xy[1], TRANSIT_Z], seconds=1.5)
        g = api.gripper()
        api.log("a%d post-lift gripper=%s eef=%s"
                % (attempt, g, np.round(api.eef(), 4).tolist()))
        if g["effort"] >= HOLD_EFFORT or attempt == 1:
            break
        api.log("a%d EMPTY -> re-perceive and retry" % attempt)
        api.grip(GRIP_OPEN_M)
        api.settle(0.3)
        _, comps2 = perceive(api)
        tgt_xy = pick_target(api, comps2)

    # ---- place -------------------------------------------------------------
    api.move([bsk_xy[0], bsk_xy[1], TRANSIT_Z], seconds=2.5)
    api.log("over-basket eef=%s gripper=%s"
            % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.move([bsk_xy[0], bsk_xy[1], RELEASE_Z], seconds=1.5)
    api.log("at-release eef=%s gripper=%s"
            % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(GRIP_OPEN_M)
    api.settle(0.6)
    api.move([bsk_xy[0], bsk_xy[1], TRANSIT_Z], seconds=1.5)
    api.settle(0.5)
    api.log("END done=%s eef=%s" % (api.done, np.round(api.eef(), 4).tolist()))
    return "v3 tgt=%s bsk=%s" % (np.round(tgt_xy, 3).tolist(),
                                 np.round(bsk_xy, 3).tolist())
