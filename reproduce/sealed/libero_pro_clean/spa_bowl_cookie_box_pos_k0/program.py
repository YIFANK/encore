"""v11 - full task: rim-pinch the bowl nearest the cookie box, carry, place on the plate.

Everything below is derived from debug seeds 51-58 (cam_high RGB-D dumps, v2/v3/v4 probes):
  * table top z ~ 0.901 (modal plane of the cam_high cloud)
  * cookie box  = warm-coloured flat slab, top 0.920
  * plate       = grey flat disc, top 0.920
  * bowls       = components above z 0.930; target = smallest bbox gap to the box
  * jaw closing axis = base y  (v3 wrist-cam open/closed pixel diff moves along image u)
  * fingertip sits 0.0077 m below the eef reference (v4 closed-gripper table-press stall)
  * a rim-pinched bowl trails the eef by the full rim radius and hangs ~0.042 m below it
  * horizon is 1000 sim steps; api.move(seconds=s) costs 20*s steps (v5a)
"""
import numpy as np

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
R_YAW = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])

TABLE_Z = 0.901
TIP_DZ = 0.0077
PINCH_DEPTH = 0.024
HANG = 0.042
CARRY_Z = 1.10

PROVENANCE = {
    "R_DOWN": {"source": "generic tool-down rotation; equals api.tool_rotation() at reset (v3 debug log)", "allowed": True},
    "TABLE_Z": {"source": "debug seeds 51-58 cam_high cloud: modal plane z=0.901", "allowed": True},
    "TIP_DZ": {"source": "v4 debug probe seeds 51/53/57: closed gripper pressed on the table stalls at eef z=0.9087, table 0.901", "allowed": True},
    "PINCH_DEPTH": {"source": "v4/v6 debug: bite width grows with depth (seed 57 landed 2mm lower, bit 0.0093 and was the only carry that held); v7 sweep confirmed the bite decays with every post-close move", "allowed": True},
    "HANG": {"source": "v4 debug seed 51 post-lift cam_high cloud: held bowl's lowest point 0.042 below the eef; re-measured per episode from the post-lift cloud, this is the fallback", "allowed": True},
    "CARRY_Z": {"source": "v8 debug: bowl bottom sits ~0.026 below the eef, so 1.10 clears the tallest carry-path obstacle (far bowl rim 0.944)", "allowed": True},
    "NEUTRAL": {"source": "v9p2/v9p3/v10p2 debug probes: with the straight-down wrist (jaw axis along base y) the arm cannot retract past x~0.05-0.10 after reaching the bowl, at any height; with the wrist yawed 90 deg (v9p3) it moved freely between (0.07,-0.07,0.94) and (-0.25,0.20,1.00)", "allowed": True},
    "R_YAW": {"source": "v9p3 debug probe: jaw axis along base x; both the -x rim pose and the -x plate pose are reachable (res 0.0024 / 0.0047)", "allowed": True},
    "WARM_CUT": {"source": "debug seeds 51-58: cookie box mean rgb (0.37,0.26,0.18), plate (0.52,0.49,0.48)", "allowed": True},
    "BOWL_THR": {"source": "debug seeds 51-58: bowl rim tops 0.944/0.952 vs flat objects 0.920", "allowed": True},
    "MOVE_STEP_COST": {"source": "v5a debug probe: 5 moves @2.0s + 2 grips + settle(1.0) = 214 sim_steps", "allowed": True},
}

XR = (-0.45, 0.35); YR = (-0.55, 0.55); RES = 0.005


def cloud(f):
    dep = np.asarray(f.depth, dtype=np.float64)
    K = np.asarray(f.intrinsics); T = np.asarray(f.t_base_cam)
    H, W = dep.shape
    v, u = np.mgrid[0:H, 0:W]
    P = np.stack([(u - K[0, 2]) / K[0, 0] * dep, (v - K[1, 2]) / K[1, 1] * dep, dep], -1)
    return P @ T[:3, :3].T + T[:3, 3]


def _grid(B, rgb, thr):
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    m = (X > XR[0]) & (X < XR[1]) & (Y > YR[0]) & (Y < YR[1]) & (Z > thr)
    nx = int((XR[1] - XR[0]) / RES); ny = int((YR[1] - YR[0]) / RES)
    ix = ((X[m] - XR[0]) / RES).astype(int).clip(0, nx - 1)
    iy = ((Y[m] - YR[0]) / RES).astype(int).clip(0, ny - 1)
    H = np.zeros((nx, ny)); C = np.zeros((nx, ny, 3))
    z = Z[m]; c = rgb[m]; o = np.argsort(z)
    H[ix[o], iy[o]] = z[o]; C[ix[o], iy[o]] = c[o]
    return H, C


def _comps(H):
    M = H > 0; lab = np.zeros(M.shape, int); cur = 0
    for i, j in np.argwhere(M):
        if lab[i, j]:
            continue
        cur += 1; st = [(i, j)]; lab[i, j] = cur
        while st:
            a, b = st.pop()
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    p, q = a + da, b + db
                    if 0 <= p < M.shape[0] and 0 <= q < M.shape[1] and M[p, q] and not lab[p, q]:
                        lab[p, q] = cur; st.append((p, q))
    return lab, cur


def objects(B, rgb, thr, minn=25):
    H, C = _grid(B, rgb, thr)
    lab, n = _comps(H)
    out = []
    for k in range(1, n + 1):
        ii, jj = np.where(lab == k)
        if len(ii) < minn:
            continue
        x = XR[0] + ii * RES + RES / 2; y = YR[0] + jj * RES + RES / 2; z = H[ii, jj]
        out.append(dict(n=len(ii), x0=x.min(), x1=x.max(), y0=y.min(), y1=y.max(),
                        cx=x.mean(), cy=y.mean(), ztop=z.max(), col=C[ii, jj].mean(0)))
    return out


def gap(a, b):
    dx = max(a['x0'] - b['x1'], b['x0'] - a['x1'], 0.0)
    dy = max(a['y0'] - b['y1'], b['y0'] - a['y1'], 0.0)
    return (dx * dx + dy * dy) ** 0.5


def kasa(x, y):
    A = np.c_[x, y, np.ones(len(x))]; b = x * x + y * y
    c = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = c[0] / 2, c[1] / 2
    return float(cx), float(cy), float(np.sqrt(max(c[2] + cx * cx + cy * cy, 1e-9)))


def perceive(api):
    f = api.capture("cam_high")
    B = cloud(f); rgb = np.asarray(f.rgb, dtype=np.float64) / 255.0
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    ws = (X > XR[0]) & (X < XR[1]) & (Y > YR[0]) & (Y < YR[1])
    r, b = rgb[..., 0], rgb[..., 2]
    warm = (r - b > 0.15) & (r > 0.25) & (Z > 0.910) & (Z < 0.935) & ws
    bx, by = X[warm], Y[warm]
    box = dict(x0=float(np.percentile(bx, 2)), x1=float(np.percentile(bx, 98)),
               y0=float(np.percentile(by, 2)), y1=float(np.percentile(by, 98)))
    api.log("BOX %s n=%d" % ({k: round(v, 4) for k, v in box.items()}, int(warm.sum())))

    bowls = [o for o in objects(B, rgb, 0.930) if o['ztop'] < 1.05 and o['cx'] > -0.30]
    for o in bowls:
        api.log("BOWL n=%d c=(%.4f,%.4f) ztop=%.4f gap=%.4f" % (o['n'], o['cx'], o['cy'], o['ztop'], gap(o, box)))
    tgt = min(bowls, key=lambda o: gap(o, box))

    flats = [o for o in objects(B, rgb, 0.912)
             if o['ztop'] < 0.935 and o['cx'] > -0.35 and (o['col'][0] - o['col'][2]) < 0.12]
    plate = max(flats, key=lambda o: o['n'])
    api.log("PLATE n=%d c=(%.4f,%.4f) ztop=%.4f" % (plate['n'], plate['cx'], plate['cy'], plate['ztop']))

    # rim fit: stay inside the component's own bbox AND under its own top, so a
    # neighbouring tall fixture (the cabinet sits ~4 mm from the bowl bbox on some
    # seeds) can never leak into the fit.
    ztop = float(tgt['ztop'])
    band = (ws & (X >= tgt['x0'] - 0.004) & (X <= tgt['x1'] + 0.004)
            & (Y >= tgt['y0'] - 0.004) & (Y <= tgt['y1'] + 0.004)
            & (Z > ztop - 0.007) & (Z < ztop + 0.004))
    cx, cy, rr = kasa(X[band], Y[band])
    ok = (0.040 < rr < 0.070) and int(band.sum()) > 200
    if not ok:
        cx, cy = float(tgt['cx']), float(tgt['cy'])
        rr = float((tgt['x1'] - tgt['x0'] + tgt['y1'] - tgt['y0']) / 4.0)
    api.log("RIM centre=(%.4f,%.4f) r=%.4f ztop=%.4f n=%d fit_ok=%s" % (cx, cy, rr, ztop, int(band.sum()), ok))
    return dict(cx=cx, cy=cy, r=rr, ztop=ztop, plate=plate)


class Arm(object):
    """Closed-loop positioner: api.move has a ~1 cm steady-state bias, so feed the
    residual back into the command instead of re-issuing the same target."""

    def __init__(self, api, budget=1000):
        self.api = api; self.left = budget

    def spend(self, s):
        n = int(round(20 * s))
        if self.left < n:
            return False
        self.left -= n
        return True

    def go(self, xyz, tol=0.004, first=2.0, fix=1.0, nfix=2, rot=None):
        rot = R_DOWN if rot is None else rot
        t = np.asarray(xyz, dtype=float)
        cmd = t.copy()
        if not self.spend(first):
            return 9.0
        self.api.move([float(v) for v in cmd], rotation=rot, seconds=first)
        for _ in range(nfix):
            e = np.asarray(self.api.eef(), dtype=float)
            err = t - e
            if np.linalg.norm(err) < tol:
                return float(np.linalg.norm(err))
            cmd = cmd + err
            if not self.spend(fix):
                break
            self.api.move([float(v) for v in cmd], rotation=rot, seconds=fix)
        e = np.asarray(self.api.eef(), dtype=float)
        return float(np.linalg.norm(t - e))


def holding(api):
    g = api.gripper()
    return g['effort'] > 1.0 and g['width_m'] > 0.004


def measure_hang(api, bowl_xy, eef_z):
    """Lowest point of the held bowl, from the post-lift cloud. Costs no sim steps."""
    try:
        f = api.capture("cam_high")
        B = cloud(f); rgb = np.asarray(f.rgb, dtype=np.float64) / 255.0
        X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
        robot = ((rgb[..., 1] - rgb[..., 0] > 0.05) | (rgb[..., 2] - rgb[..., 0] > 0.05))
        m = ((np.hypot(X - bowl_xy[0], Y - bowl_xy[1]) < 0.075)
             & (Z > eef_z - 0.12) & (Z < eef_z + 0.01) & ~robot)
        if int(m.sum()) < 150:
            return None, int(m.sum())
        return float(eef_z - np.percentile(Z[m], 2)), int(m.sum())
    except Exception:
        return None, -1


NEUTRAL = [0.00, 0.06, 1.10]
PLACE_Z = 1.00


def run(api):
    arm = Arm(api)
    api.log("START eef=%s grip=%s" % (np.asarray(api.eef()).round(4).tolist(), api.gripper()))
    S = perceive(api)
    plate = S['plate']

    got = False
    px = py = z_eef = 0.0
    for attempt in range(3):
        depth = PINCH_DEPTH + 0.006 * attempt
        # jaw axis is base x (R_YAW); grip the -x side of the rim so the bowl trails
        # at +x and the plate pose stays inside the reachable envelope.
        px, py = S['cx'] - S['r'], S['cy']
        z_eef = S['ztop'] - depth + TIP_DZ
        api.grip(0.08); api.settle(0.3); arm.spend(0.3)
        arm.go([px, py, S['ztop'] + 0.075], tol=0.005, first=1.8, fix=0.9, nfix=2, rot=R_YAW)
        r2 = arm.go([px, py, z_eef], tol=0.003, first=1.2, fix=0.8, nfix=2, rot=R_YAW)
        api.log("A%d AT r=%.4f eef=%s aim=(%.4f,%.4f,%.4f) left=%d"
                % (attempt, r2, np.asarray(api.eef()).round(4).tolist(), px, py, z_eef, arm.left))
        api.grip(0.0); api.settle(0.8); arm.spend(0.8)
        api.log("A%d CLOSED grip=%s" % (attempt, api.gripper()))
        arm.go([px, py, CARRY_Z], tol=0.012, first=1.8, fix=0.0, nfix=0, rot=R_YAW)
        api.log("A%d UP grip=%s eef=%s left=%d" % (attempt, api.gripper(), np.asarray(api.eef()).round(4).tolist(), arm.left))
        if holding(api):
            got = True
            break
        if arm.left < 470:
            break
        api.grip(0.08); api.settle(0.3); arm.spend(0.3)
        S = perceive(api)

    hang, nh = measure_hang(api, (px + S['r'], py), float(api.eef()[2]))
    if hang is None or not (0.015 < hang < 0.075):
        hang = HANG
    api.log("GRASP_DONE got=%s hang=%.4f n=%d left=%d" % (got, hang, nh, arm.left))

    rN = arm.go(NEUTRAL, tol=0.012, first=1.7, fix=0.9, nfix=1, rot=R_YAW)
    api.log("NEUTRAL r=%.4f eef=%s grip=%s left=%d" % (rN, np.asarray(api.eef()).round(4).tolist(), api.gripper(), arm.left))
    tx, ty = plate['cx'] - S['r'], plate['cy']
    r3 = arm.go([tx, ty, PLACE_Z], tol=0.006, first=1.8, fix=0.9, nfix=1, rot=R_YAW)
    api.log("CARRY r=%.4f eef=%s grip=%s left=%d" % (r3, np.asarray(api.eef()).round(4).tolist(), api.gripper(), arm.left))

    z_place = plate['ztop'] + hang + 0.006
    r4 = arm.go([tx, ty, z_place], tol=0.005, first=1.2, fix=0.8, nfix=1, rot=R_YAW)
    api.log("PLACE r=%.4f eef=%s grip=%s z_place=%.4f" % (r4, np.asarray(api.eef()).round(4).tolist(), api.gripper(), z_place))
    api.grip(0.08); api.settle(0.6); arm.spend(0.6)
    arm.go([tx - 0.02, ty, z_place + 0.10], tol=0.02, first=1.2, fix=0.0, nfix=0, rot=R_YAW)
    api.log("END eef=%s grip=%s left=%d" % (np.asarray(api.eef()).round(4).tolist(), api.gripper(), arm.left))
