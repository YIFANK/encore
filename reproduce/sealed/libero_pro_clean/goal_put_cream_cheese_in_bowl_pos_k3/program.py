"""v5: v4 + a cabinet-ledge aware grasp column.

The v2/v4 stalls (descent frozen at eef z 1.0635 on seeds 55 and 63) were the hand landing
on the cabinet ledge that runs along y=-0.125 over the carton's x range with its top at
z=1.098.  Descents clear it only when the eef stays ~0.095 m in +y of that edge, so the
grasp column is pushed out to a measured clearance; the jaws still straddle the carton
because the offset stays under half the open jaw minus the carton half-width.
"""
import numpy as np

PROVENANCE = {
    "WS_X": {"source": "debug seeds 51-65 cam_high cloud: props lie in x[-0.32,0.25]", "allowed": True},
    "WS_Y": {"source": "debug seeds 51-65 cam_high cloud: table props lie in y[-0.32,0.32]; cabinet occupies y<-0.12", "allowed": True},
    "BLUE_DR": {"source": "debug seeds 51-65: the carton is the only workspace surface with B>R+20 and B>G+10 below z=1.0", "allowed": True},
    "BLUE_DG": {"source": "same as BLUE_DR", "allowed": True},
    "TOP_SLAB": {"source": "debug seeds: 4 mm slab below the carton's max z isolates its top face (75x39 mm, flat)", "allowed": True},
    "RIM_LO": {"source": "debug seeds 51-65: bowl rim top z=0.9522 +-0.0003; plate/carton tops are 0.920, stove 0.932", "allowed": True},
    "RIM_HI": {"source": "same as RIM_LO", "allowed": True},
    "RIM_R": {"source": "debug seeds 51-65 Kasa fit of the rim band: radius 0.0526 +-0.0001 on all 15 seeds", "allowed": True},
    "RIM_TOL": {"source": "debug seeds: rim band points sit within ~6 mm of the fitted circle", "allowed": True},
    "GRASP_DZ": {"source": "pack keyframe grasp-close ee z (0.9104/0.9205/0.9106) vs my measured carton top 0.9196; v1/v2 logs show the descent bottoming at eef z 0.923 with a 42 mm jaw reading", "allowed": True},
    "STAGE_Z": {"source": "debug seeds: tallest corridor prop (bottle) tops out at 1.059; v3 probe: fingertips sit 0.030 below the eef, so 1.12 clears it by 3 cm", "allowed": True},
    "APPROACH_DY": {"source": "v3 probe seeds 55/57/63: descents succeed when the column is entered from a +y offset at altitude", "allowed": True},
    "RELEASE_DZ": {"source": "pack keyframe release ee z (0.959/0.973/0.979) minus my measured rim top 0.9522", "allowed": True},
    "MOVE_TOL": {"source": "v1/v2 logs: free-space moves settle ~8 mm short in +x and do not improve on re-issue, so 12 mm is the useful convergence test", "allowed": True},
    "HOLD_W_MIN": {"source": "v1/v2 logs: a real carton grasp reads jaw width 0.042; a miss reads 0.001 and a corner bite 0.016", "allowed": True},
    "OPEN_W": {"source": "api.gripper() reports width 0.0778 at reset = fully open", "allowed": True},
    "Y_CLEAR": {"source": "debug descents: blocked at eef_y-ywall=0.0868 (seeds 55/63), clear at 0.0956 (v3 probe) and above; 0.100 adds margin", "allowed": True},
    "MAX_OFF": {"source": "carton half-width 0.0195 vs half the open jaw 0.039 -> up to 0.0195 of lateral offset still straddles it; 0.015 keeps a margin", "allowed": True},
    "LEDGE_Z": {"source": "debug seeds: the cabinet ledge over the carton x-range tops at z=1.098, set-back parts at 1.019/1.127", "allowed": True},
}

WS_X = (-0.32, 0.25)
WS_Y = (-0.32, 0.32)
BLUE_DR = 20
BLUE_DG = 10
TOP_SLAB = 0.004
RIM_LO, RIM_HI = 0.942, 0.958
RIM_R = 0.0526
RIM_TOL = 0.006
GRASP_DZ = -0.004
STAGE_Z = 1.12
APPROACH_DY = 0.09
RELEASE_DZ = 0.018
MOVE_TOL = 0.012
HOLD_W_MIN = 0.030
OPEN_W = 0.078
Y_CLEAR = 0.100
MAX_OFF = 0.015


def _cloud(f):
    K = np.asarray(f.intrinsics, dtype=float)
    T = np.asarray(f.t_base_cam, dtype=float)
    d = np.nan_to_num(np.asarray(f.depth, dtype=float), nan=0.0)
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]
    cam = np.stack([(uu - K[0, 2]) * d / K[0, 0],
                    (vv - K[1, 2]) * d / K[1, 1], d], -1)
    return cam @ T[:3, :3].T + T[:3, 3]


def _kasa(px, py):
    A = np.stack([px, py, np.ones_like(px)], 1)
    b = px * px + py * py
    c = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = c[0] / 2.0, c[1] / 2.0
    return cx, cy, float(np.sqrt(max(c[2] + cx * cx + cy * cy, 1e-9)))


def find_carton(P, rgb, api):
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    ws = ((x > WS_X[0]) & (x < WS_X[1]) & (y > WS_Y[0]) & (y < WS_Y[1]) &
          (z > 0.85) & (z < 1.00))
    r = rgb[..., 0].astype(int)
    g = rgb[..., 1].astype(int)
    b = rgb[..., 2].astype(int)
    m = ws & (b > r + BLUE_DR) & (b > g + BLUE_DG)
    n = int(m.sum())
    if n < 100:
        api.log("carton NOT FOUND (%d blue px)" % n)
        return None
    ztop = float(z[m].max())
    top = m & (z > ztop - TOP_SLAB)
    if int(top.sum()) < 40:
        api.log("carton top face too small (%d)" % int(top.sum()))
        return None
    pts = np.stack([x[top], y[top]], 1)
    c0 = pts.mean(0)
    q = pts - c0
    vt = np.linalg.svd(q, full_matrices=False)[2]
    e = q @ vt.T
    mid = np.array([(e[:, 0].min() + e[:, 0].max()) / 2.0,
                    (e[:, 1].min() + e[:, 1].max()) / 2.0])
    c = c0 + mid @ vt
    api.log("carton n %d ztop %.4f c %.4f %.4f len %.3f wid %.3f"
            % (n, ztop, c[0], c[1], float(np.ptp(e[:, 0])), float(np.ptp(e[:, 1]))))
    return float(c[0]), float(c[1]), ztop


def find_bowl(P, api):
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    band = ((x > WS_X[0]) & (x < WS_X[1]) & (y > -0.12) & (y < WS_Y[1]) &
            (z > RIM_LO) & (z < RIM_HI))
    n = int(band.sum())
    if n < 150:
        api.log("bowl NOT FOUND (%d band px)" % n)
        return None
    px, py = x[band], y[band]
    gx = np.arange(WS_X[0], WS_X[1], 0.002)
    gy = np.arange(-0.12, WS_Y[1], 0.002)
    best, bestc = -1, (0.0, 0.0)
    for cx in gx:
        rr = np.sqrt((px - cx) ** 2 + (py[None, :] - gy[:, None]) ** 2)
        votes = (np.abs(rr - RIM_R) < RIM_TOL).sum(1)
        k = int(votes.argmax())
        if votes[k] > best:
            best, bestc = int(votes[k]), (float(cx), float(gy[k]))
    cx, cy = bestc
    inl = np.abs(np.hypot(px - cx, py - cy) - RIM_R) < RIM_TOL
    fx, fy, fr = _kasa(px[inl], py[inl])
    ztop = float(z[band][inl].max())
    api.log("bowl vote %d kasa %.4f %.4f r %.4f rim %.4f" % (best, fx, fy, fr, ztop))
    if not (0.045 < fr < 0.062):
        fx, fy = cx, cy
    return float(fx), float(fy), ztop


def wall_y(P, cx, cy, api):
    """Nearest +y edge of the cabinet structure in the column's x window."""
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    m = ((x > cx - 0.10) & (x < cx + 0.10) & (y < cy - 0.03) & (y > -0.32) &
         (z > 0.96) & (z < 1.15))
    if int(m.sum()) < 30:
        api.log("wall: none in window")
        return -0.32
    yw = float(y[m].max())
    api.log("wall y %.4f (clearance %.4f)" % (yw, cy - yw))
    return yw


def step(api, tgt, seconds=1.5, tries=2, tol=MOVE_TOL, tag=""):
    tgt = [float(v) for v in tgt]
    for i in range(tries):
        api.move(tgt, seconds=seconds)
        p = np.asarray(api.eef(), dtype=float)
        d = float(np.linalg.norm(p - np.asarray(tgt)))
        api.log("step %s.%d -> %.4f %.4f %.4f err %.4f" % (tag, i, p[0], p[1], p[2], d))
        if d < tol:
            break
    return np.asarray(api.eef(), dtype=float)


def walk_to(api, x, y, z, tag):
    """Enter a column from a +y offset at altitude, in two legs."""
    step(api, [x, y + APPROACH_DY, z], seconds=2.0, tries=2, tag=tag + ".far")
    return step(api, [x, y, z], seconds=1.5, tries=2, tag=tag + ".col")


def run(api):
    api.log("instr %s" % api.instruction())
    f = api.capture("cam_high")
    P = _cloud(f)
    carton = find_carton(P, np.asarray(f.rgb), api)
    bowl = find_bowl(P, api)
    if carton is None or bowl is None:
        api.log("PERCEPTION FAILED")
        return
    bx, by, btop = carton
    wx, wy, rim = bowl
    gy = min(by + MAX_OFF, max(by, wall_y(P, bx, by, api) + Y_CLEAR))
    if abs(gy - by) > 1e-4:
        api.log("grasp column offset +y %.4f (carton y %.4f -> %.4f)" % (gy - by, by, gy))
    grasp_z = btop + GRASP_DZ
    release_z = rim + RELEASE_DZ
    api.log("plan grasp %.4f %.4f %.4f release %.4f %.4f %.4f"
            % (bx, by, grasp_z, wx, wy, release_z))

    api.grip(OPEN_W)
    held = False
    for attempt in range(2):
        walk_to(api, bx, gy, STAGE_Z, "app%d" % attempt)
        step(api, [bx, gy, grasp_z], seconds=1.5, tries=2, tol=0.004, tag="down%d" % attempt)
        api.grip(0.0)
        api.settle(0.4)
        api.log("close%d %s" % (attempt, str(api.gripper())))
        step(api, [bx, gy, STAGE_Z], seconds=2.0, tries=2, tag="lift%d" % attempt)
        g = api.gripper()
        api.log("lift%d %s" % (attempt, str(g)))
        if float(g["width_m"]) > HOLD_W_MIN:
            held = True
            break
        api.grip(OPEN_W)
        api.settle(0.3)
        f2 = api.capture("cam_high")
        c2 = find_carton(_cloud(f2), np.asarray(f2.rgb), api)
        if c2 is None:
            break
        bx, by, btop = c2
        P2 = _cloud(f2)
        gy = min(by + MAX_OFF, max(by, wall_y(P2, bx, by, api) + Y_CLEAR))
        grasp_z = btop + GRASP_DZ
        api.log("regrasp at %.4f %.4f (gy %.4f)" % (bx, by, gy))

    api.log("held=%s" % held)
    walk_to(api, wx, wy, STAGE_Z, "over")
    step(api, [wx, wy, release_z], seconds=2.0, tries=2, tol=0.006, tag="drop")
    api.log("at drop %s" % str(api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.6)
    step(api, [wx, wy, STAGE_Z], seconds=2.0, tries=1, tag="retreat")
    api.settle(0.5)

    f3 = api.capture("cam_high")
    c3 = find_carton(_cloud(f3), np.asarray(f3.rgb), api)
    if c3 is not None:
        api.log("final carton %.4f %.4f top %.4f dxy_to_bowl %.4f"
                % (c3[0], c3[1], c3[2], float(np.hypot(c3[0] - wx, c3[1] - wy))))
