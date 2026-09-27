"""v23 -- v21 with the footprint height test taken over the inner disc, not the
bbox: on seeds 56 and 62 the neighbouring bowl fell inside the plate's bounding
box and vetoed the true plate (PLATE NOT FOUND, 13/15). The jaws keep squeezing after they close, so the wedge-shaped rim
is ratcheted out at a fixed rate per sim step: v18 lost the plate on move 13
after closing, v19 on move 15, whatever the step size. So do the whole job in
nine moves: coarse arc, one hoist, one carry, three to lay it down.
"""
"""v17 notes:

A rim pinch bites 19 mm of brim (v10/v14) but a straight-up lift ratchets the
plate out of the jaws.  The plate cannot rise vertically: while its far edge is
still on the table it can only pivot about that edge, so the gripped rim point
must travel on a circle of radius D about the far edge.  Follow that circle:
lift h, and come in by D - sqrt(D^2 - h^2), until the plate hangs vertical --
at which point the jaws clamp the shell slab instead of the brim wedge.
"""
import zlib, base64
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-65 cam_high depth: flat table plane z=0.901 m", "allowed": True},
    "PLATE_TOP": {"source": "debug seeds 51-65: plate component top z=0.920 m", "allowed": True},
    "PLATE_DISC_TEST": {"source": "sel run v21 seeds 56/62: the bowl sits inside the plate bbox; measured over the inner 0.75 r disc the plate reads 0.9115 and every distractor stays out", "allowed": True},
    "PLATE_DIAM": {"source": "debug seeds 51-65: plate component bbox 0.136 x 0.136 m", "allowed": True},
    "GRASP_INSET": {"source": "v10/v14 debug runs: jaw centre 0.060 inboard of the plate centre bites 0.019 m of brim (0.063->0.011, 0.0687->0.001)", "allowed": True},
    "GRASP_TIPS": {"source": "v10/v14 debug runs: tips at 0.9095 give the widest bite", "allowed": True},
    "TIP_OFFSET": {"source": "v7 debug seed 51: closed jaws stopped on the bare table at eef_z=0.9105, table 0.901", "allowed": True},
    "ARC_D": {"source": "geometry: jaw centre 0.060 from the plate centre, far edge 0.0687 the other way -> pivot radius 0.1287", "allowed": True},
    "CAB_TOP_Z": {"source": "debug seeds 51/54/60/65 cam_high: dominant flat plane over the left block, z=1.127 m", "allowed": True},
    "DROP_XY": {"source": "debug seeds 51/54/60/65: that plane spans x[-0.12,0.16], y[-0.36,-0.15]", "allowed": True},
    "PLATE_SIT_OFF": {"source": "v10/v14: with the plate flat on the table (0.901) the grasp eef sits at 0.919, i.e. +0.018 over the support", "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics: home tool rotation is straight down", "allowed": True},
}

TABLE_Z = 0.901
PLATE_TOP = 0.920
TIP_OFFSET = 0.0095
GRASP_INSET = 0.060
GRASP_TIPS = 0.9095
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def cloud(f):
    K = np.asarray(f.intrinsics, dtype=np.float64)
    T = np.asarray(f.t_base_cam, dtype=np.float64)
    d = np.asarray(f.depth, dtype=np.float64)
    H, W = d.shape
    v, u = np.mgrid[0:H, 0:W]
    x = (u - K[0, 2]) / K[0, 0] * d
    y = (v - K[1, 2]) / K[1, 1] * d
    pts = np.stack([x, y, d, np.ones_like(d)], -1)
    return (pts @ T.T)[..., :3]


def components(mask, minpix=300):
    from collections import deque
    seen = np.zeros(mask.shape, bool)
    out = []
    H, W = mask.shape
    for v0, u0 in np.argwhere(mask):
        if seen[v0, u0]:
            continue
        q = deque([(v0, u0)])
        seen[v0, u0] = True
        pix = []
        while q:
            v, u = q.popleft()
            pix.append((v, u))
            for a, b in ((v + 1, u), (v - 1, u), (v, u + 1), (v, u - 1)):
                if 0 <= a < H and 0 <= b < W and mask[a, b] and not seen[a, b]:
                    seen[a, b] = True
                    q.append((a, b))
        if len(pix) >= minpix:
            out.append(np.array(pix))
    return out


def find_plate(api, tag=""):
    f = api.capture("cam_high")
    P = cloud(f)
    rgb = np.asarray(f.rgb, dtype=np.float64)
    Z = P[..., 2]
    good = np.isfinite(Z)
    inbox = good & (P[..., 0] > -0.20) & (P[..., 0] < 0.40) & (P[..., 1] > -0.15) & (P[..., 1] < 0.35)
    band = inbox & (Z > TABLE_Z + 0.005) & (Z < PLATE_TOP + 0.008) & (rgb.mean(-1) > 110)
    best = None
    for pix in components(band):
        pts = P[pix[:, 0], pix[:, 1]]
        xr = pts[:, 0].max() - pts[:, 0].min()
        yr = pts[:, 1].max() - pts[:, 1].min()
        cx = 0.5 * (pts[:, 0].max() + pts[:, 0].min())
        cy = 0.5 * (pts[:, 1].max() + pts[:, 1].min())
        rr = 0.25 * (xr + yr)
        disc = good & (np.hypot(P[..., 0] - cx, P[..., 1] - cy) < 0.75 * rr)
        tall = float(np.percentile(Z[disc], 99.0)) if int(disc.sum()) > 50 else 0.0
        api.log("cand%s n=%d c=(%.3f,%.3f) dx=%.3f dy=%.3f tall=%.3f" % (tag, len(pix), cx, cy, xr, yr, tall))
        if not (0.10 < xr < 0.18 and 0.10 < yr < 0.18):
            continue
        if tall > PLATE_TOP + 0.010:
            continue
        if best is None or len(pix) > best[0]:
            best = (len(pix), cx, cy, 0.5 * (xr + yr))
    return best


def goto(api, xyz, R, tag, tries=3, tol=0.004, seconds=2.0):
    cmd = [float(v) for v in xyz]
    e = np.asarray(api.eef(), dtype=float)
    for i in range(tries):
        api.move(cmd, rotation=R, seconds=seconds)
        e = np.asarray(api.eef(), dtype=float)
        err = np.asarray(xyz, dtype=float) - e
        n = float(np.linalg.norm(err))
        api.log("goto %s t%d eef=(%.4f,%.4f,%.4f) err=%.4f" % (tag, i, e[0], e[1], e[2], n))
        if n < tol:
            break
        cmd = [cmd[k] + float(err[k]) for k in range(3)]
    return e


def dump(api, tag, f, step=2):
    arr = np.ascontiguousarray(np.asarray(f.rgb, dtype=np.uint8)[::step, ::step])
    b = base64.b64encode(zlib.compress(arr.tobytes(), 6)).decode()
    api.log("%s shape=%s dtype=uint8 nchunks=%d" % (tag, arr.shape, (len(b) + 1899) // 1900))
    for i in range(0, len(b), 1900):
        api.log("%s#%d %s" % (tag, i // 1900, b[i:i + 1900]))


CAB_TOP_Z = 1.127
DROP_X, DROP_Y = 0.030, -0.240
SIT_OFF = 0.018
ARC_H = (0.035, 0.065, 0.092, 0.114)
HANG_H = 0.114


def arc_y(D, h):
    return D - float(np.sqrt(max(D * D - h * h, 1e-6)))


def run(api):
    api.log("INSTR %r" % (api.instruction(),))
    found = find_plate(api)
    if found is None:
        api.log("PLATE NOT FOUND")
        return
    n, cx, cy, diam = found
    r = 0.5 * diam
    api.log("PLATE n=%d c=(%.3f,%.3f) diam=%.3f" % (n, cx, cy, diam))

    gy = cy + GRASP_INSET
    z0 = GRASP_TIPS + TIP_OFFSET
    D = GRASP_INSET + r

    api.grip(0.08)
    goto(api, [cx, gy, 1.00], R_DOWN, "above", tries=2, tol=0.010)
    goto(api, [cx, gy, z0], R_DOWN, "grasp", tries=3)
    api.grip(0.0)
    api.log("closed w=%.4f" % api.gripper()["width_m"])

    for h in ARC_H:
        api.move([cx, gy - arc_y(D, h), z0 + h], rotation=R_DOWN, seconds=1.2)
        e = api.eef()
        api.log("arc h=%.3f eef=(%.4f,%.4f,%.4f) w=%.4f" % (h, e[0], e[1], e[2], api.gripper()["width_m"]))
    ey = float(api.eef()[1])

    dy_h = arc_y(D, HANG_H)
    zt = CAB_TOP_Z + SIT_OFF + HANG_H + 0.020
    api.move([cx, ey, zt], rotation=R_DOWN, seconds=1.2)
    api.log("hoist eef=%s w=%.4f" % (np.round(api.eef(), 4).tolist(), api.gripper()["width_m"]))
    api.move([DROP_X, DROP_Y - dy_h, zt], rotation=R_DOWN, seconds=1.2)
    api.log("carry eef=%s w=%.4f" % (np.round(api.eef(), 4).tolist(), api.gripper()["width_m"]))
    for h in (HANG_H, 0.060, 0.005):
        api.move([DROP_X, DROP_Y - arc_y(D, h), CAB_TOP_Z + SIT_OFF + h], rotation=R_DOWN, seconds=1.2)
        e = api.eef()
        api.log("lay h=%.3f eef=(%.4f,%.4f,%.4f) w=%.4f" % (h, e[0], e[1], e[2], api.gripper()["width_m"]))
    api.grip(0.08)
    api.settle(0.4)
    goto(api, [DROP_X, DROP_Y + 0.03, 1.28], R_DOWN, "retreat", tries=2, tol=0.020)
    dump(api, "FINAL", api.capture("cam_high"))
    api.log("DONE v23")
