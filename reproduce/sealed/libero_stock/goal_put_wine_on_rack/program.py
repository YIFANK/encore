"""c2 goal_put_wine_on_rack_stock -- v3.

v1/v2 (identical motion: pure demo1 replay with a FIXED release pose) scored
7/8 and 14/15 on the debug band; the only failure is seed 61, where the bottle
was released a hair off the rack's cradle and rolled off.  The only thing that
moves between seeds is the RACK: measured top-ridge centroid spans
x -0.2457..-0.2651, y -0.2992..-0.3184 over the 15 debug seeds (the bottle's
start pose is seed-invariant -- the same commanded grasp closed with effort 3.0
and width 0.0248-0.0254 m on all 15).

v3 therefore makes the carry+release RACK-RELATIVE: everything downstream of
the grasp is translated by (perceived ridge centroid - debug-band median ridge
centroid), capped at 3 cm.  Nothing else changes.
"""
import numpy as np

PROVENANCE = {
    "D1_GRASP_EE": {"source": "pack.json demo1 keyframe t=76 'ee' xyz (gripper_cmd flips to close)", "allowed": True},
    "D1_GRASP_AA": {"source": "pack.json demo1 keyframe t=76 'ee' axis-angle [3:6]", "allowed": True},
    "D1_RELEASE_EE": {"source": "pack.json demo1 keyframe t=157 'ee' xyz (gripper_cmd flips to open while holding)", "allowed": True},
    "D1_RELEASE_AA": {"source": "pack.json demo1 keyframe t=157 'ee' axis-angle [3:6]", "allowed": True},
    "APPROACH_WP": {"source": "pack.json demo1 ee_path samples 5-7", "allowed": True},
    "CARRY_WP": {"source": "pack.json demo1 ee_path samples 10-13", "allowed": True},
    "RETREAT_DZ": {"source": "pack.json demo1 ee_path sample 17 minus 15", "allowed": True},
    "RACK_ROI": {"source": "debug seeds 51-65 (fs_..._v2) occupancy maps: bounds of the back-left structure, cabinet excluded at x>-0.10", "allowed": True},
    "REF_TOPCEN": {"source": "median over debug seeds 51-65 (fs_..._v2 PRE topcen) of the rack ridge centroid", "allowed": True},
    "TOP_BAND": {"source": "generic: ridge = points within 2.5 cm of the ROI's max height", "allowed": True},
    "MAX_SHIFT": {"source": "debug seeds 51-65: observed ridge-centroid spread is <2.1 cm, so 3 cm caps a perception blunder", "allowed": True},
    "OPEN_W": {"source": "pack.json gripper_state open width 0.079", "allowed": True},
}

D1_GRASP_EE = np.array([-0.2139, -0.0685, 0.9831])
D1_GRASP_AA = np.array([1.5507, 1.6737, -0.8284])
D1_RELEASE_EE = np.array([-0.1924, -0.2627, 1.1910])
D1_RELEASE_AA = np.array([2.0566, 2.3968, 0.0265])
APPROACH_WP = np.array([-0.190, -0.010, 1.070])
CARRY_WP = np.array([-0.180, -0.150, 1.215])
CARRY_WP2 = np.array([-0.150, -0.235, 1.250])
RETREAT_DZ = 0.085
RACK_ROI = (-0.45, -0.105, -0.45, -0.12)     # xlo, xhi, ylo, yhi
REF_TOPCEN = np.array([-0.2567, -0.3059])
TOP_BAND = 0.025
MAX_SHIFT = 0.03
OPEN_W = 0.079


def rod(aa):
    aa = np.asarray(aa, float)
    th = float(np.linalg.norm(aa))
    if th < 1e-9:
        return np.eye(3)
    k = aa / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * K @ K


def cloud(frame):
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape[:2]
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    ok = np.isfinite(d) & (d > 0)
    z = np.where(ok, d, 0.0)
    x = (uu - K[0, 2]) * z / K[0, 0]
    y = (vv - K[1, 2]) * z / K[1, 1]
    P = np.stack([x, y, z], -1).reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]
    return P.reshape(h, w, 3), ok


def run(api):
    log = api.log
    log("v3 start eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    f = api.capture("cam_high")
    P, ok = cloud(f)
    zs = P[..., 2][ok]
    band = zs[(zs > 0.5) & (zs < 1.4)]
    hist, edges = np.histogram(band, bins=90, range=(0.5, 1.4))
    table = float(edges[int(np.argmax(hist))] + 0.005)
    xlo, xhi, ylo, yhi = RACK_ROI
    roi = (ok & (P[..., 0] > xlo) & (P[..., 0] < xhi) & (P[..., 1] > ylo)
           & (P[..., 1] < yhi) & (P[..., 2] > table + 0.04) & (P[..., 2] < table + 0.60))
    R = P[roi]
    log("table_z=%.4f rackROI n=%d" % (table, R.shape[0]))
    shift = np.zeros(2)
    if R.shape[0] > 200:
        zmax = float(R[:, 2].max())
        top = R[R[:, 2] > zmax - TOP_BAND]
        topcen = top[:, :2].mean(0)
        log("rack ztop=%.4f h=%.3f topcen=%s ntop=%d bbox x[%.3f,%.3f] y[%.3f,%.3f]"
            % (zmax, zmax - table, np.round(topcen, 4).tolist(), top.shape[0],
               R[:, 0].min(), R[:, 0].max(), R[:, 1].min(), R[:, 1].max()))
        shift = np.clip(topcen - REF_TOPCEN, -MAX_SHIFT, MAX_SHIFT)
        # 1.5 cm height map of the rack, cm above table (diagnostic)
        gx = np.arange(-0.45, -0.10, 0.015)
        gy = np.arange(-0.45, -0.11, 0.015)
        log("RACKMAP rows x=%.3f..%.3f cols y=%.3f..%.3f step 0.015 (cm above table, '--'=empty)"
            % (gx[0], gx[-1], gy[0], gy[-1]))
        for xa in gx:
            cells = []
            for ya in gy:
                m = ((R[:, 0] >= xa) & (R[:, 0] < xa + 0.015)
                     & (R[:, 1] >= ya) & (R[:, 1] < ya + 0.015))
                cells.append("--" if m.sum() < 3 else
                             "%02d" % min(99, int(round((R[m, 2].max() - table) * 100))))
            log("RACKMAP x=%+.3f %s" % (xa, " ".join(cells)))
    log("rack shift=%s" % np.round(shift, 4).tolist())
    d3 = np.array([shift[0], shift[1], 0.0])

    Rg = rod(D1_GRASP_AA)
    Rr = rod(D1_RELEASE_AA)
    api.grip(OPEN_W)
    r = api.move(APPROACH_WP, rotation=Rg, seconds=2.0)
    log("approach res=%.4f" % r)
    r = api.move(D1_GRASP_EE, rotation=Rg, seconds=1.5)
    log("at grasp res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(0.0)
    api.settle(0.4)
    log("post-close gripper=%s" % api.gripper())
    r = api.move(D1_GRASP_EE + np.array([0.0, 0.0, 0.11]), rotation=Rg, seconds=1.2)
    log("lift res=%.4f grip=%s" % (r, api.gripper()))
    r = api.move(CARRY_WP + d3, rotation=Rr, seconds=1.5)
    log("carry1 res=%.4f" % r)
    r = api.move(CARRY_WP2 + d3, rotation=Rr, seconds=1.2)
    log("carry2 res=%.4f" % r)
    rel = D1_RELEASE_EE + d3
    r = api.move(rel, rotation=Rr, seconds=1.5)
    log("at release res=%.4f target=%s eef=%s grip=%s"
        % (r, np.round(rel, 4).tolist(), np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.5)
    log("post-release gripper=%s" % api.gripper())
    r = api.move(rel + np.array([0.0, 0.02, RETREAT_DZ]), rotation=Rr, seconds=1.2)
    log("retreat res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.settle(1.0)
