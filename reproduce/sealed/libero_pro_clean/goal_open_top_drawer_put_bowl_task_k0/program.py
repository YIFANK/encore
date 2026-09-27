"""v6: open the top drawer, then put the cream-cheese box inside.

v6 vs v5: the drawer-aperture search is confined to the cabinet's own x-span
and to the y-window between the pulled-out handle and the cabinet body, so the
drop point is the aperture centre instead of its rear lip."""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "HANDLE_Z_BAND": {"source": "debug seeds 51/53/55 cam_high depth: three handle bars at z 0.930-0.956, 1.009-1.025, 1.082-1.098; top bar isolated by z in (1.03,1.115)", "allowed": True},
    "HANDLE_Y_BAND": {"source": "debug seeds 51/53/55 cam_high depth: drawer face plane y=-0.161, handle fronts y=-0.125", "allowed": True},
    "BAR_RADIUS": {"source": "debug seeds 51/53/55 cam_high depth: top handle bar spans z 1.082-1.098 -> 8 mm radius; v4 closed gap 0.0174 m confirms the bar diameter", "allowed": True},
    "TIP": {"source": "debug seed 51 v3 descent: eef stalled at z=0.9096 over a table plane measured at z=0.902 -> fingertips 8 mm beyond the grip site", "allowed": True},
    "JAW_AXIS": {"source": "debug seed 51 v3 pre-grasp cam_high: open fingers resolved at y=-0.175 and y=-0.085 about eef y=-0.128 (jaws separate along tool y)", "allowed": True},
    "R_SIDE": {"source": "generic controller mechanics: tool-to-world matrix putting the approach axis along -y and the jaw axis along +z", "allowed": True},
    "R_DOWN": {"source": "debug seed 51 v1 start pose tool_rotation (straight-down wrist, jaws along base y)", "allowed": True},
    "PULL_M": {"source": "debug seed 51 v4: handle held to +0.15 m of pull (gap 0.0173, effort 3.0); at +0.20 m the gap collapsed to 0.0044 -> drawer travel limit", "allowed": True},
    "TABLE_Z": {"source": "debug seeds 51/53/55 cam_high depth: dominant table plane z=0.902", "allowed": True},
    "BOX_COLOUR": {"source": "debug seeds 51/53/55 cam_high RGB: the only blue-dominant (b>r+25) prop on the table, top face z=0.919, footprint 0.072 x 0.035 m", "allowed": True},
    "CAVITY_Z_BAND": {"source": "debug seed 51 v4 post-open cam_high depth: open-drawer floor plane at z=1.064, rim z=1.124, cabinet top z=1.127", "allowed": True},
    "CAVITY_WINDOW": {"source": "debug seed 51 v4 post-open cam_high depth: cabinet x-span is the handle bar centre +-0.14 and the aperture lies between the pulled handle and 0.22 m behind it", "allowed": True},
}

R_SIDE = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
TIP = 0.008
PULL_M = 0.15


def _dump(api, tag, arr):
    arr = np.ascontiguousarray(arr)
    b = base64.b64encode(zlib.compress(arr.tobytes(), 9)).decode()
    n = (len(b) + 1799) // 1800
    api.log("%s shape=%s dtype=%s nchunks=%d" % (tag, list(arr.shape), arr.dtype, n))
    for i in range(n):
        api.log("%s#%d:%s" % (tag, i, b[i * 1800:(i + 1) * 1800]))


def _snap(api, tag, f):
    api.log("%s T=%s" % (tag, np.asarray(f.t_base_cam).reshape(-1).tolist()))
    api.log("%s K=%s" % (tag, np.asarray(f.intrinsics).reshape(-1).tolist()))
    _dump(api, tag + "_rgb", np.asarray(f.rgb)[::2, ::2, :])
    d = np.asarray(f.depth, np.float32)
    _dump(api, tag + "_d", np.where(np.isfinite(d), d, 0.0)[::2, ::2].astype(np.float16))


def cloud(f):
    d = np.asarray(f.depth, np.float32)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = d.shape
    v, u = np.mgrid[0:H, 0:W]
    z = np.where(np.isfinite(d), d, 0.0)
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    P = np.stack([x, y, z, np.ones_like(z)], -1) @ T.T
    return P[..., :3], (z > 0)


def top_handle(api, f):
    P, ok = cloud(f)
    m = ok & (P[..., 2] > 1.03) & (P[..., 2] < 1.115) \
        & (P[..., 1] > -0.20) & (P[..., 1] < -0.08) \
        & (P[..., 0] > -0.35) & (P[..., 0] < 0.35)
    Q = P[m]
    ymax = float(np.percentile(Q[:, 1], 99.5))
    H = Q[Q[:, 1] > ymax - 0.015]
    bx, bz = float(np.median(H[:, 0])), float(np.median(H[:, 2]))
    api.log("handle n=%d ymax=%.4f bx=%.4f bz=%.4f" % (len(H), ymax, bx, bz))
    return bx, ymax - 0.008, bz


def find_box(api, f):
    P, ok = cloud(f)
    rgb = np.asarray(f.rgb, int)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    m = ok & (b > r + 25) & (b > 50) & (P[..., 2] > 0.910) & (P[..., 2] < 0.97) \
        & (P[..., 0] > -0.45) & (P[..., 0] < 0.25) & (P[..., 1] > -0.10) & (P[..., 1] < 0.55)
    Q = P[m]
    top = Q[Q[:, 2] > float(np.percentile(Q[:, 2], 60)) - 0.004]
    cx = 0.5 * (float(top[:, 0].min()) + float(top[:, 0].max()))
    cy = 0.5 * (float(top[:, 1].min()) + float(top[:, 1].max()))
    tz = float(np.median(top[:, 2]))
    api.log("box n=%d c=(%.4f,%.4f) top_z=%.4f xr=%.3f..%.3f yr=%.3f..%.3f"
            % (len(top), cx, cy, tz, top[:, 0].min(), top[:, 0].max(), top[:, 1].min(), top[:, 1].max()))
    return cx, cy, tz


def find_cavity(api, f, hx, hy_end):
    """Aperture of the opened top drawer: the modal plane between the drawer
    floor and the rim, searched only over the cabinet's own x-span and between
    the pulled-out handle and the cabinet body."""
    P, ok = cloud(f)
    m = ok & (P[..., 2] > 1.040) & (P[..., 2] < 1.105) \
        & (P[..., 0] > hx - 0.14) & (P[..., 0] < hx + 0.14) \
        & (P[..., 1] > hy_end - 0.22) & (P[..., 1] < hy_end - 0.01)
    Q = P[m]
    if len(Q) < 200:
        api.log("cavity NOT FOUND n=%d" % len(Q))
        return None
    hist, edges = np.histogram(Q[:, 2], bins=26)
    k = int(np.argmax(hist))
    zf = 0.5 * (edges[k] + edges[k + 1])
    F = Q[np.abs(Q[:, 2] - zf) < 0.008]
    if len(F) < 150:
        api.log("cavity THIN n=%d" % len(F))
        return None
    x0, x1 = float(np.percentile(F[:, 0], 5)), float(np.percentile(F[:, 0], 95))
    y0, y1 = float(np.percentile(F[:, 1], 5)), float(np.percentile(F[:, 1], 95))
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    api.log("cavity n=%d floor_z=%.4f c=(%.4f,%.4f) x %.3f..%.3f y %.3f..%.3f"
            % (len(F), zf, cx, cy, x0, x1, y0, y1))
    return cx, cy, zf


def mp(api, xyz, rot, tag, seconds=0.6, tries=2):
    res = 9.0
    for i in range(tries):
        res = api.move(xyz, rotation=rot, seconds=seconds)
        e = np.asarray(api.eef())
        api.log("mp %s try%d cmd=%s res=%.4f eef=%s"
                % (tag, i, [round(v, 3) for v in xyz], res, np.round(e, 4).tolist()))
        if res < 0.012:
            break
    return res


def run(api):
    f0 = api.capture("cam_high")
    bx, by, bz = top_handle(api, f0)
    px, py, ptz = find_box(api, f0)

    # ---- open the top drawer: side-on grasp of the bar, pull +y
    api.grip(0.08)
    mp(api, [bx, by + 0.14, bz], R_SIDE, "approach", seconds=1.6, tries=2)
    mp(api, [bx, by + TIP, bz], R_SIDE, "engage", seconds=0.5, tries=2)
    api.grip(0.0)
    g = api.gripper()
    api.log("handle closed grip=%s" % g)
    for dy in (0.06, 0.11, PULL_M):
        mp(api, [bx, by + TIP + dy, bz], R_SIDE, "pull%.2f" % dy, seconds=0.5, tries=1)
    api.log("after pull grip=%s eef=%s" % (api.gripper(), np.round(np.asarray(api.eef()), 4).tolist()))
    api.grip(0.08)
    mp(api, [bx, by + TIP + PULL_M + 0.10, bz + 0.10], R_SIDE, "release_back", seconds=0.6, tries=1)

    # ---- pick the cream-cheese box
    mp(api, [px, py, ptz + 0.16], R_DOWN, "over_box", seconds=1.2, tries=2)
    mp(api, [px, py, 0.912], R_DOWN, "descend_box", seconds=0.4, tries=2)
    api.grip(0.0)
    gb = api.gripper()
    api.log("box closed grip=%s eef=%s" % (gb, np.round(np.asarray(api.eef()), 4).tolist()))
    mp(api, [px, py, 1.22], R_DOWN, "lift_box", seconds=0.8, tries=2)

    # ---- place it in the open drawer
    f1 = api.capture("cam_high")
    cav = find_cavity(api, f1, bx, by + TIP + PULL_M)
    if cav is None:
        cx, cy, zf = bx - 0.01, by + TIP + PULL_M - 0.115, 1.064
        api.log("cavity fallback c=(%.3f,%.3f) zf=%.3f" % (cx, cy, zf))
    else:
        cx, cy, zf = cav
    mp(api, [cx, cy, 1.22], R_DOWN, "over_cavity", seconds=0.8, tries=2)
    mp(api, [cx, cy, zf + 0.045], R_DOWN, "into_cavity", seconds=0.5, tries=2)
    api.grip(0.08)
    api.settle(0.5)
    api.log("released eef=%s grip=%s" % (np.round(np.asarray(api.eef()), 4).tolist(), api.gripper()))
    mp(api, [cx, cy, 1.25], R_DOWN, "retreat", seconds=0.6, tries=1)
    api.settle(0.4)
    f2 = api.capture("cam_high")
    _snap(api, "final", f2)
    return "v6 drawer+box"
