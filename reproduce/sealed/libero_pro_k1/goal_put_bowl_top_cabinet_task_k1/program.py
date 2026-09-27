"""c2k1clean goal_put_bowl_top_cabinet_task_k1 -- v12 (final).

Intent: "Put the plate on the top of the drawer."

The flat disc the intent names is 0.141 across against a 0.080 jaw span, so no
diametral grasp exists.  The only pinch available is the radial wedge: the
outer jaw outside the disc's vertical outer wall, the inner jaw down inside
the dish, force-close.  That bite lifts the disc but has a short life -- the
jaws keep closing and ratchet out along the tapering rim (measured:
0.0208 -> 0.0058 over one lift, gone after a second command).

The thing that made it work is what the disc does in flight: pinched at
w ~ 0.005 it is held on a thin section and swings edge-down, hanging most of
a diameter -- about 0.15 m -- below the eef, not the 0.020 m it occupied at
the grasp.  Carry heights chosen for 0.020 dragged it straight through the
raised top.  Carrying at top + 0.180 clears it, and the disc arrives over the
top and lands on it.

Sequence: perceive -> wedge grasp -> one vertical lift to 1.300 -> one
traverse at top + 0.180 -> release.  Re-perceives afterwards and repeats once
if the disc is still on the table.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "GRID": {"source": "generic camera/workspace mechanics: raster bounds/cell of my own top-down height map", "allowed": True},
    "TABLE_Z": {"source": "debug seeds 51-57 cam_high depth: dominant z mode of the deprojected cloud, 0.900", "allowed": True},
    "TOP_BAND": {"source": "debug seeds 51-57: the raised flat top plane measured at z 1.1272 (x[-0.100,0.155], y[-0.350,-0.160]); band brackets it", "allowed": True},
    "DISC_BAND": {"source": "debug seeds 51-57: the disc's cluster tops at 0.9201, the flat slab at -x tops at 0.9263, so the band stops between them", "allowed": True},
    "DISC_XLIM": {"source": "debug seeds 51-57 height map: x window that excludes the -x flat slab", "allowed": True},
    "DISC_YLIM": {"source": "debug seeds 51-57 height map: y window that excludes the raised top at -y", "allowed": True},
    "DISC_RMIN": {"source": "debug seeds 51-57: the disc's equivalent radius is 0.0706-0.0708; the small box in the same height band is r_eq 0.035", "allowed": True},
    "TIP_OFF": {"source": "debug seeds 51-55: shut jaws pressed at bare table stalled with eef z 0.9098, i.e. tip = eef - 0.0098", "allowed": True},
    "JAW_HALF": {"source": "debug seeds 51-57 api.gripper() at reset: open width 0.0797-0.0800, half-span 0.0399", "allowed": True},
    "FLOOR_CLR": {"source": "debug seeds 51-57 radial profile: the dish floor reads 0.9078; clearance the inner jaw needs on descent", "allowed": True},
    "OUT_CLR": {"source": "debug seeds 51-57 radial profile: clearance the outer jaw needs beyond the 0.0707 outer edge on descent", "allowed": True},
    "BITE_H": {"source": "debug seeds 51-57 radial profile (8 mm floor rising to 20 mm at the rim); v5 (9.5 mm tip) outlived v7 (8.5 mm tip), so the inner contact belongs on the steeper flange", "allowed": True},
    "APEX_Z": {"source": "debug seeds 51-57: the vertical lift the wedge survives; the arm reached 1.2888 against this command (v6 reached 1.2508, k1 demo0 apex was 1.2219)", "allowed": True},
    "CARRY_RISE": {"source": "debug seeds 51-57 v11: the disc hangs edge-down ~0.150 (its own 0.141 diameter) below the eef once pinched at w~0.005; 0.180 = that plus clearance over the 1.1272 top, 4/4 on 51/53/55/57", "allowed": True},
    "HANG_D": {"source": "debug seeds 51-57: the disc's measured diameter 0.141, i.e. how far it hangs when held edge-on", "allowed": True},
    "SET_Z": {"source": "debug seeds 51-57: gap left above the 1.1272 top when the load is lowered before release", "allowed": True},
    "PLACE_INSET": {"source": "debug seeds 51-57 top-plane extent (y[-0.350,-0.160]); k1 pack demo0 released 0.017 inside that +y edge", "allowed": True},
    "R_DOWN": {"source": "debug seed 51 api.tool_rotation() at reset: [[0.998,0,-0.057],[0,-1,0],[-0.057,0,-0.998]], i.e. straight down", "allowed": True},
    "N_ATTEMPTS": {"source": "generic policy mechanics: the 1000-step horizon fits two ~165-step attempts", "allowed": True},
}

GX = (-0.40, 0.40)
GY = (-0.45, 0.45)
CELL = 0.005
TABLE_Z = 0.900
TOP_BAND = (1.10, 1.16)
DISC_BAND = (0.9035, 0.9240)
DISC_XLIM = (-0.15, 0.30)
DISC_YLIM = (-0.13, 0.30)
DISC_RMIN = 0.055
TIP_OFF = 0.0098
JAW_HALF = 0.0399
FLOOR_CLR = 0.0007
OUT_CLR = 0.004
BITE_H = 0.0095
APEX_Z = 1.300
CARRY_RISE = 0.180
HANG_D = 0.150
SET_Z = 0.010
PLACE_INSET = 0.040
N_ATTEMPTS = 2
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
OPEN_W = 0.08
SHUT_W = 0.0


# ---------------------------------------------------------------- perception

def ship(api, tag, arr):
    b64 = base64.b64encode(zlib.compress(np.ascontiguousarray(arr).tobytes(), 9)).decode()
    api.log("%s META shape=%s dtype=%s" % (tag, list(arr.shape), str(arr.dtype)))
    for i in range(0, len(b64), 1800):
        api.log("%s CHUNK %d %s" % (tag, i // 1800, b64[i:i + 1800]))


def scene(api, cam="cam_high"):
    """cam_high RGB-D -> (top-down max-z height map, base-frame cloud, valid)."""
    f = api.capture(cam)
    h, w = f.depth.shape[:2]
    K = np.asarray(f.intrinsics, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:h, 0:w]
    z = np.asarray(f.depth, float)
    ok = np.isfinite(z) & (z > 0)
    pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z, np.ones_like(z)], -1)
    P = (pc.reshape(-1, 4) @ np.asarray(f.t_base_cam, float).T)[:, :3].reshape(h, w, 3)
    nx = int((GX[1] - GX[0]) / CELL)
    ny = int((GY[1] - GY[0]) / CELL)
    hz = np.full((nx, ny), np.nan)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = ok & (X >= GX[0]) & (X < GX[1]) & (Y >= GY[0]) & (Y < GY[1])
    ix = ((X - GX[0]) / CELL).astype(int)
    iy = ((Y - GY[0]) / CELL).astype(int)
    order = np.argsort(Z[m])                 # low first, so max z wins each cell
    hz[ix[m][order], iy[m][order]] = Z[m][order]
    return hz, P, ok


def components(mask):
    lab = np.zeros(mask.shape, int)
    cur = 0
    for s in np.argwhere(mask):
        s = (int(s[0]), int(s[1]))
        if lab[s]:
            continue
        cur += 1
        lab[s] = cur
        stack = [s]
        while stack:
            a, b = stack.pop()
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    p, q = a + da, b + db
                    if (0 <= p < mask.shape[0] and 0 <= q < mask.shape[1]
                            and mask[p, q] and not lab[p, q]):
                        lab[p, q] = cur
                        stack.append((p, q))
    return lab, cur


def clusters(hz, lo, hi, xlim=None, ylim=None, minn=20):
    m = np.isfinite(hz) & (hz > lo) & (hz < hi)
    nx, ny = hz.shape
    ix, iy = np.mgrid[0:nx, 0:ny]
    X = GX[0] + ix * CELL
    Y = GY[0] + iy * CELL
    if xlim:
        m &= (X >= xlim[0]) & (X <= xlim[1])
    if ylim:
        m &= (Y >= ylim[0]) & (Y <= ylim[1])
    lab, n = components(m)
    out = []
    for c in range(1, n + 1):
        idx = np.argwhere(lab == c)
        if len(idx) < minn:
            continue
        xs = GX[0] + idx[:, 0] * CELL
        ys = GY[0] + idx[:, 1] * CELL
        zs = hz[idx[:, 0], idx[:, 1]]
        out.append({"n": int(len(idx)),
                    "cx": float(0.5 * (xs.min() + xs.max())),
                    "cy": float(0.5 * (ys.min() + ys.max())),
                    "y1": float(ys.max()),
                    "r": float(np.sqrt(len(idx) * CELL * CELL / np.pi)),
                    "zmax": float(zs.max()), "zmed": float(np.median(zs))})
    out.sort(key=lambda d: -d["n"])
    return out


def find_disc(hz):
    for c in clusters(hz, DISC_BAND[0], DISC_BAND[1], DISC_XLIM, DISC_YLIM, minn=200):
        if c["r"] > DISC_RMIN:
            return c
    return None


def radial(api, P, ok, cx, cy, tag="PROF"):
    """Height vs radius about the disc centre, 2.5 mm bins."""
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    r = np.sqrt((X - cx) ** 2 + (Y - cy) ** 2)
    m = ok & (r < 0.095) & (Z > 0.895) & (Z < 0.96)
    rr, zz = r[m], Z[m]
    prof = []
    for i in range(38):
        s = (rr >= i * 0.0025) & (rr < (i + 1) * 0.0025)
        prof.append(float(np.percentile(zz[s], 90)) if s.sum() >= 3 else np.nan)
    api.log("%s %s" % (tag, [int(round((v - TABLE_Z) * 1000)) if np.isfinite(v) else -1
                             for v in prof]))
    return prof


# ------------------------------------------------------------------- motion

def state(api, tag):
    g = api.gripper()
    e = api.eef()
    api.log("%s eef=%s w=%.4f eff=%.2f" % (tag, np.round(e, 4).tolist(),
                                           g["width_m"], g["effort"]))
    return e, g


def goto(api, xyz, tag, seconds=2.5, tol=0.006, tries=2):
    """Command, read back, cancel the standing tracking bias once."""
    tgt = np.asarray(xyz, float)
    cmd = tgt.copy()
    for k in range(tries):
        api.move(cmd.tolist(), R_DOWN, seconds=seconds)
        e = api.eef()
        err = tgt - e
        api.log("%s#%d eef=%s err=%s" % (tag, k, np.round(e, 4).tolist(),
                                         np.round(err, 4).tolist()))
        if np.linalg.norm(err) < tol:
            break
        cmd = cmd + err
    return api.eef()


def plan_bite(api, disc, prof):
    """Inner jaw tip just above the dish floor on the flange; outer jaw beyond
    the outer wall.  d is the jaw-pair centre, offset along +y from the disc."""
    R = disc["r"]
    vals = [v for v in prof[:6] if np.isfinite(v)]
    floor = float(np.min(vals)) if vals else TABLE_Z + 0.008
    z_tip = max(floor + FLOOR_CLR, TABLE_Z + BITE_H)
    r_in = 0.010
    for i, v in enumerate(prof):
        if np.isfinite(v) and v >= z_tip:
            r_in = (i + 0.5) * 0.0025
            break
    r_in = float(np.clip(r_in, 0.010, R - 0.020))
    d = max(0.5 * (R + r_in), R + OUT_CLR - JAW_HALF)
    api.log("BITE floor=%.4f z_tip=%.4f r_in=%.4f R=%.4f d=%.4f inner=%.4f outer=%.4f"
            % (floor, z_tip, r_in, R, d, d - JAW_HALF, d + JAW_HALF))
    return d, z_tip


def attempt(api, disc, prof, top, tag):
    d, z_tip = plan_bite(api, disc, prof)
    px, py = disc["cx"], disc["cy"]
    gy = py + d
    z_eef = z_tip + TIP_OFF
    tx = top["cx"]
    ty = top["y1"] - PLACE_INSET
    z_top = top["zmed"]

    api.grip(OPEN_W)
    goto(api, [px, gy, z_eef + 0.065], tag + "_hov", seconds=2.5)
    goto(api, [px, gy, z_eef], tag + "_dn", seconds=2.0, tol=0.004)
    state(api, tag + "_PRE")
    api.grip(SHUT_W)
    api.settle(0.2)
    state(api, tag + "_SHUT")

    # The wedge's life is a couple of commands, so exactly two are spent:
    # one vertical lift (the motion it survives) and one traverse flown high
    # enough that the edge-down disc hanging ~0.150 below the eef clears the
    # raised top instead of being dragged through it.
    api.move([px, gy, APEX_Z], R_DOWN, seconds=1.5)
    state(api, tag + "_APEX")
    api.move([tx, ty, z_top + CARRY_RISE], R_DOWN, seconds=2.0)
    e, g = state(api, tag + "_OVER_TOP")
    if g["effort"] >= 1.0:
        api.move([tx, ty, z_top + HANG_D + SET_Z], R_DOWN, seconds=1.5)
        state(api, tag + "_SET")
    api.grip(OPEN_W)
    api.settle(0.5)
    state(api, tag + "_RELEASED")
    goto(api, [tx, ty + 0.09, APEX_Z], tag + "_retreat", seconds=2.5, tries=1)
    goto(api, [0.10, 0.26, 1.18], tag + "_park", seconds=3.0, tries=1)


def run(api):
    api.log("INSTRUCTION: %s" % api.instruction())
    hz, P, ok = scene(api)
    ship(api, "HZ0", np.nan_to_num(hz, nan=0.0).astype(np.float16))
    tops = clusters(hz, TOP_BAND[0], TOP_BAND[1], minn=300)
    api.log("TOP %s" % (tops[0] if tops else None))
    if not tops:
        return "no raised top"
    top = tops[0]

    for k in range(N_ATTEMPTS):
        disc = find_disc(hz)
        api.log("ATTEMPT %d DISC %s" % (k, disc))
        if disc is None:
            api.log("no disc on the table -- stopping")
            break
        prof = radial(api, P, ok, disc["cx"], disc["cy"], "PROF%d" % k)
        attempt(api, disc, prof, top, "A%d" % k)
        hz, P, ok = scene(api)
        ship(api, "HZ%d" % (k + 1), np.nan_to_num(hz, nan=0.0).astype(np.float16))
        api.log("AFTER%d table_disc=%s" % (k, find_disc(hz)))
        api.log("AFTER%d on_top=%s" % (k, clusters(hz, top["zmed"] + 0.006,
                                                   top["zmed"] + 0.12,
                                                   (-0.11, 0.17), (-0.37, -0.145),
                                                   minn=30)[:2]))
    api.settle(0.5)
    return "v12 done"
