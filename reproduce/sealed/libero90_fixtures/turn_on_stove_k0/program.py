"""l90abl_turn_on_stove_k0 v8 -- grasp the knob bar and twist the wrist.

Findings (debug seeds 51/53/55/57; cam_high, eef, gripper):
  * The stove slab (top ~table+25mm) has, just behind it, a rotary knob: a disc
    (top ~table+22mm) carrying an upright lever bar whose flat top face is an
    80x25mm rectangle at ~table+59mm.  The bar's centroid is the pivot.
  * v4a: a straight +y blade push on the bar is rigid -- 133mm of overshoot
    yields 0mm of eef travel and 0deg of bar rotation.  That sense is blocked.
  * v4b: the same push in -y turns the bar +12.9deg, then stalls -- i.e. the
    free sense is increasing atan2 angle, but a sliding blade loses the bar.
  * v7 arc-swept the blade about the pivot and scored 2/4; the two failures
    show the bar springing back to upright once the blade slips past it.
A blade is a slipping contact, so v8 instead straddles the bar with the open
jaws (they separate along base y, the bar runs along base x, so no pre-rotation
is needed), closes on it, and rotates the wrist about world +z with the eef
parked on the pivot -- a positive, non-slipping drive of the knob angle.
"""
import numpy as np

PROVENANCE = {
    "KNOB_WIN": {"source": "debug seeds 51/53/55/57 cam_high height maps: knob "
                           "at x[-0.258,-0.168] y[+0.183,+0.219]; window widened "
                           "to +-0.05 for seed jitter", "allowed": True},
    "BAR_ZLO/BAR_ZHI": {"source": "debug-seed height map: bar top table+0.059, "
                                  "disc top table+0.022; band table+0.048..+0.070 "
                                  "isolates the bar's top face", "allowed": True},
    "TIP_OFF": {"source": "debug seed 51/55 v3: closed gripper pressed onto the "
                          "table stalled with eef at table+0.0075; corroborated in "
                          "v4/v7, where an eef at table+0.042 engaged the bar "
                          "(top table+0.059) and cleared the disc (table+0.022)",
                "allowed": True},
    "GRASP_Z": {"source": "debug-seed height map: fingertip at table+0.033 lies "
                          "on the bar's shank, 11mm above the disc top",
                "allowed": True},
    "TURN_DIR/TURN_STEP/NTURN": {
        "source": "debug seeds 51/53/55 v4a vs v4b: the +y push is rigid (0deg) "
                  "while -y turns the bar, so the free sense is increasing atan2 "
                  "angle, i.e. positive rotation about world +z", "allowed": True},
    "APPROACH_R": {"source": "debug-seed height map: knob disc half-width ~0.045, "
                             "so a 0.085 radius descends onto bare table",
                   "allowed": True},
    "ZTOL": {"source": "generic controller mechanics: descents starve, so the "
                       "height command is repeated until the eef converges",
             "allowed": True},
    "TABLE_Z": {"source": "in-episode: modal deprojected z over the workspace",
                "allowed": True},
}

KNOB_WIN = (-0.32, -0.11, 0.10, 0.30)
BAR_ZLO, BAR_ZHI = 0.048, 0.070
TIP_OFF = 0.0075
GRASP_Z = 0.033
TURN_STEP = np.radians(18.0)
NTURN = 6
SAFE_Z = 0.16
ZTOL = 0.004


def cloud(f):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.asarray(f.depth, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 1e-4)
    z = d[ok]
    x = (uu[ok] - K[0, 2]) * z / K[0, 0]
    y = (vv[ok] - K[1, 2]) * z / K[1, 1]
    return (np.stack([x, y, z, np.ones_like(z)], 1) @ T.T)[:, :3]


def table_z(P):
    m = (P[:, 0] > -0.40) & (P[:, 0] < 0.20) & (np.abs(P[:, 1]) < 0.45)
    h, e = np.histogram(P[m, 2], bins=np.arange(0.80, 1.10, 0.002))
    return float(e[int(np.argmax(h))] + 0.001)


def components(Q, cell=0.015):
    gx = np.floor(Q[:, 0] / cell).astype(int)
    gy = np.floor(Q[:, 1] / cell).astype(int)
    cells = {}
    for i, (a, b) in enumerate(zip(gx, gy)):
        cells.setdefault((a, b), []).append(i)
    seen, out = set(), []
    for c in cells:
        if c in seen:
            continue
        stack, comp = [c], []
        seen.add(c)
        while stack:
            a, b = stack.pop()
            comp += cells[(a, b)]
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    n = (a + da, b + db)
                    if n in cells and n not in seen:
                        seen.add(n)
                        stack.append(n)
        out.append(np.array(comp))
    return out


def find_bar(api, P, tz, tag):
    xlo, xhi, ylo, yhi = KNOB_WIN
    m = ((P[:, 0] > xlo) & (P[:, 0] < xhi) & (P[:, 1] > ylo) & (P[:, 1] < yhi)
         & (P[:, 2] > tz + BAR_ZLO) & (P[:, 2] < tz + BAR_ZHI))
    Q = P[m]
    if len(Q) < 30:
        api.log(f"[{tag}] no bar pts={len(Q)}")
        return None
    R = Q[sorted(components(Q), key=len, reverse=True)[0]]
    c = R.mean(0)
    d = R[:, :2] - c[:2]
    w, V = np.linalg.eigh(d.T @ d / len(d))
    axis = V[:, int(np.argmax(w))]
    if axis[0] > 0:
        axis = -axis
    th = float(np.arctan2(axis[1], axis[0]))
    api.log(f"[{tag}] n={len(R)} c={np.round(c,4).tolist()} th={np.degrees(th):+.1f} "
            f"x[{R[:,0].min():+.3f},{R[:,0].max():+.3f}] "
            f"y[{R[:,1].min():+.3f},{R[:,1].max():+.3f}]")
    return c, th


def rz(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]])


def settle_z(api, tgt, tries=3):
    for _ in range(tries):
        e = api.eef()
        if abs(e[2] - tgt[2]) <= ZTOL:
            break
        api.move(tgt, seconds=0.6)
    return api.eef()


def run(api):
    P = cloud(api.capture("cam_high"))
    tz = table_z(P)
    got = find_bar(api, P, tz, "bar0")
    if got is None:
        return "no bar"
    c, th0 = got

    api.grip(0.08)                       # jaws open, separating along base y
    zg = tz + GRASP_Z + TIP_OFF
    api.log(f"[cfg] tz={tz:.3f} zg={zg:.4f} c={np.round(c[:2],4).tolist()} "
            f"th0={np.degrees(th0):+.1f} grip={api.gripper()}")

    api.move([c[0], c[1], tz + SAFE_Z], seconds=2.5)
    api.move([c[0], c[1], zg], seconds=2.0)
    e = settle_z(api, [c[0], c[1], zg])
    api.log(f"[grasp] eef={np.round(e,4).tolist()} zerr={e[2]-zg:+.4f}")
    api.grip(0.0)
    api.settle(0.3)
    g = api.gripper()
    api.log(f"[grasp] gripper={g}")

    R0 = np.asarray(api.tool_rotation(), float)
    for k in range(1, NTURN + 1):
        Rk = rz(TURN_STEP * k) @ R0
        r = api.move([c[0], c[1], zg], rotation=Rk, seconds=0.9)
        e = api.eef()
        api.log(f"[turn{k:02d}] want={np.degrees(TURN_STEP*k):+.0f} "
                f"eef={np.round(e,4).tolist()} res={float(r):.4f} grip={api.gripper()}")

    api.grip(0.08)
    api.settle(0.2)
    api.move([c[0], c[1], tz + SAFE_Z], seconds=1.2)
    api.move([0.05, -0.32, tz + 0.22], seconds=2.0)
    api.settle(0.3)
    got2 = find_bar(api, cloud(api.capture("cam_high")), tz, "bar1")
    if got2:
        d = np.degrees(got2[1] - th0)
        api.log(f"[verdict] dth={((d + 90) % 180) - 90:+.1f}deg")
    return "v8 grasp+twist"
