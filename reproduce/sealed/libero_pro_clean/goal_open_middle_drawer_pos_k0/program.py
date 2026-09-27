"""c2clean goal_open_middle_drawer_pos_k0 -- v1.

Mechanism.  The cabinet carries three horizontal bar handles whose axis runs
along base x, standing ~32 mm proud of the drawer face.  The bar is ~16 mm
thick and sits ~15 mm from the face, so nothing can be hooked behind it: the
only grip is a side-on straddle, wrist rolled so the fingers separate
vertically and the approach axis points +y into the face.

The hard part is vertical clearance.  The free channel between the top of the
bar below and the bottom of the bar above is ~126 mm and the fully-open Panda
hand is ~123 mm across its finger backs, so the eef height has to be right to a
few mm.  cam_high cannot measure that (2.1 mm/px at a grazing angle), so the
program stages in front of the cabinet first and then re-measures the channel
with the wrist camera (0.7 mm/px, and placed between the bars so it sees the
top bar's underside and the bottom bar's top surface directly).  If the advance
still jams on a neighbouring bar it backs off and re-tries at a shifted height.
"""
import json

import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51/53/55/57 cam_high cloud: modal support plane 0.901 m", "allowed": True},
    "SAT_MAX": {"source": "debug seed 51 cam_high RGB: robot links are saturated, cabinet+handles neutral", "allowed": True},
    "R_SIDE": {"source": "debug seeds 51/53/57: bar axis along base x, face normal -y; start wrist rolled +90 deg about world x (p4/p5 receipts: the 180-deg-rolled variant jams the arm)", "allowed": True},
    "ARC_FACTOR": {"source": "generic cylinder-under-oblique-view geometry with the debug-seed cam_high extrinsics", "allowed": True},
    "TIP_OFFSET": {"source": "debug seed 51 p5 preclose frame: front-most gripper point 0.1057 vs api.eef y 0.0942", "allowed": True},
    "STAND_OFF": {"source": "debug seed 51 p5: staging 0.10 m in front of the measured bar front", "allowed": True},
    "CHUNK_S": {"source": "generic controller mechanics (60 steps per 0.5 s chunk, 1000-step episode)", "allowed": True},
    "Z_RETRY": {"source": "debug seed 51 p5: a jam on a neighbouring bar stops the eef ~30 mm short of the face", "allowed": True},
    "TILT_DEG": {"source": "debug seeds 51-65 geometry: bar centre 19 mm from the face, bar pitch 73 mm, open-hand finger backs 123 mm apart -- 30 deg is the largest tilt whose far finger still clears the face", "allowed": True},
    "GRASP_D": {"source": "debug seed 51/59 measured bar centre, face plane and gripper finger extent", "allowed": True},
    "ADV": {"source": "debug seed 51 p5: fingertip sits 0.0115 m ahead of api.eef", "allowed": True},
    "ROT_DRIFT": {"source": "debug seeds 51/53/57 p5+v1: rolling the wrist in place carries the eef +0.050 m in y and +0.047 m in z", "allowed": True},
    "GRIP_REPEAT": {"source": "debug seeds 51/55/59 v4: one api.grip call closes only 40 mm of the 60 mm needed", "allowed": True},
    "PULL_FAR": {"source": "generic controller mechanics: a far target keeps the position command saturated for the whole chunk", "allowed": True},
    "R_BAR": {"source": "debug seeds 53/55/59/61 v1: gripper gap after closing on the bar = 0.0174 m", "allowed": True},
}

TABLE_Z = 0.905
SAT_MAX = 0.18
ARC_FACTOR = 1.196
R_BAR = 0.0087
ROT_DY = 0.033
ROT_DZ = 0.031
PULL_FAR = 0.35
TIP_OFFSET = 0.0115
STAND_OFF = 0.10
CHUNK_S = 0.5
Z_RETRY = (0.0, -0.004, 0.004, -0.008)

# Grasp frame: the approach axis is tilted TILT_DEG below horizontal so the
# finger pair separates in y as well as z.  That does two things a purely
# side-on straddle cannot: the far finger clears the bars above/below in y
# instead of having to thread a 3 mm vertical window, and the wrist ends only
# 60 deg from the start pose instead of 90, which is what the arm needs to keep
# retracting in -y while it drags the drawer (p7/p8 receipts).
TILT_DEG = 30.0
_c = float(np.cos(np.radians(TILT_DEG)))
_s = float(np.sin(np.radians(TILT_DEG)))
U = np.array([0.0, _s, _c])        # finger-separation axis (up and back)
AX = np.array([0.0, _c, -_s])      # tool approach axis (forward and down)
R_TILT = np.array([[1.0, 0.0, 0.0],
                   [0.0, -_s, _c],
                   [0.0, -_c, -_s]])
# Grasp frame: wrist rolled a full 90 deg from the start pose, so the approach
# axis is +y and the fingers separate vertically.  That seats the bar deep
# against the palm, which is what survives the pull (a tilted grasp holds the
# bar near the fingertips and it squirts out along the approach axis -- v5).
R_SIDE = np.array([[1.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0],
                   [0.0, -1.0, 0.0]])
GRASP_D = 0.025    # slide the grip centre back along U so the far finger
                   # stays in front of the drawer face
ADV = 0.020        # push past the bar along AX; the face stops the fingers


# ------------------------------------------------------------------ geometry
def cloud(f):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.asarray(f.depth, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    z = np.where(np.isfinite(d) & (d > 1e-3), d, np.nan)
    x = (uu - K[0, 2]) / K[0, 0] * z
    y = (vv - K[1, 2]) / K[1, 1] * z
    P = np.stack([x, y, z, np.ones_like(z)])
    return (T @ P.reshape(4, -1)).reshape(4, h, w)[:3]


def components(pts, cell=0.03, minpts=150):
    g = np.floor(pts[:, :2] / cell).astype(int)
    cells = {}
    for i, k in enumerate(map(tuple, g)):
        cells.setdefault(k, []).append(i)
    seen, out = set(), []
    for c in cells:
        if c in seen:
            continue
        st, comp = [c], []
        seen.add(c)
        while st:
            k = st.pop()
            comp.append(k)
            for d in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                n = (k[0] + d[0], k[1] + d[1])
                if n in cells and n not in seen:
                    seen.add(n)
                    st.append(n)
        idx = [i for k in comp for i in cells[k]]
        if len(idx) >= minpts:
            out.append(pts[idx])
    return out


def find_cabinet(rgb, P):
    """Locate the drawer cabinet and its three bar handles in cam_high."""
    X, Y, Z = P
    f = np.asarray(rgb, np.float64)
    mx = f.max(2)
    sat = (mx - f.min(2)) / np.maximum(mx, 1.0)
    ok = (np.isfinite(Z) & (Z > TABLE_Z + 0.015) & (Z < TABLE_Z + 0.40)
          & (X > -0.85) & (X < 0.30) & (np.abs(Y) < 0.75) & (sat < SAT_MAX))
    pts = np.stack([X[ok], Y[ok], Z[ok]], 1)
    if len(pts) < 300:
        return None
    tall = pts[pts[:, 2] > TABLE_Z + 0.15]
    cands = components(tall)
    if not cands:
        return None
    cab = max(cands, key=len)
    xlo, xhi = np.percentile(cab[:, 0], [1, 99])
    ylo, yhi = np.percentile(cab[:, 1], [1, 99])
    ztop = float(np.percentile(cab[:, 2], 99))
    sel = pts[(pts[:, 0] > xlo - 0.06) & (pts[:, 0] < xhi + 0.06) &
              (pts[:, 1] > ylo - 0.15) & (pts[:, 1] < yhi + 0.10) &
              (pts[:, 2] < ztop - 0.008)]
    if len(sel) < 200:
        return None
    hist, edges = np.histogram(sel[:, 1],
                              bins=np.arange(sel[:, 1].min(), sel[:, 1].max() + 0.005, 0.005))
    yface = float(edges[int(np.argmax(hist))] + 0.0025)
    hp = sel[(sel[:, 1] < yface - 0.012) & (sel[:, 1] > yface - 0.10)]
    bars = []
    if len(hp) >= 30:
        hp = hp[np.argsort(hp[:, 2])]
        for k in np.split(np.arange(len(hp)), np.where(np.diff(hp[:, 2]) > 0.015)[0] + 1):
            if len(k) < 25:
                continue
            c = hp[k]
            if c[:, 2].max() - c[:, 2].min() > 0.040:
                continue
            x2, x98 = np.percentile(c[:, 0], [2, 98])
            if x98 - x2 < 0.03:
                continue
            zlo_, zhi_ = float(c[:, 2].min()), float(c[:, 2].max())
            r = max(0.005, (zhi_ - zlo_) / ARC_FACTOR)
            bars.append(dict(zc=zhi_ - r, r=r, zlo=zlo_, zhi=zhi_,
                             xmid=float((x2 + x98) / 2), xlo=float(x2), xhi=float(x98),
                             yfront=float(np.percentile(c[:, 1], 2)), n=int(len(c))))
    bars.sort(key=lambda b: b["zc"])
    return dict(yface=yface, ztop=ztop, xlo=float(xlo), xhi=float(xhi), bars=bars)


def wrist_channel(f, xg, yface, ez):
    """Re-measure the free vertical channel around the middle bar up close.

    The wrist camera sits between the bars, so it sees the underside of the bar
    above and the top surface of the bar below as direct silhouette edges -- no
    cylinder model needed.
    """
    X, Y, Z = cloud(f)
    m = (np.isfinite(Z) & (Y > 0.04) & (Y < yface - 0.012)
         & (X > xg - 0.05) & (X < xg + 0.05)
         & (Z > ez - 0.14) & (Z < ez + 0.14))
    pts = np.stack([X[m], Y[m], Z[m]], 1)
    if len(pts) < 60:
        return None
    pts = pts[np.argsort(pts[:, 2])]
    bands = []
    for k in np.split(np.arange(len(pts)), np.where(np.diff(pts[:, 2]) > 0.018)[0] + 1):
        if len(k) < 20:
            continue
        c = pts[k]
        if c[:, 2].max() - c[:, 2].min() > 0.035:
            continue
        bands.append((float(c[:, 2].min()), float(c[:, 2].max()), int(len(c)),
                      float(np.percentile(c[:, 1], 2))))
    return bands


# ------------------------------------------------------------------ actuation
class Budget:
    def __init__(self, api, total=1000):
        self.api = api
        self.left = total

    def pose(self, xyz, R, s=CHUNK_S):
        if self.left < 60:
            return None
        r = self.api.move([float(v) for v in xyz], rotation=R, seconds=s)
        self.left -= max(60, int(60 * max(s, 0.5)))
        return r

    def lin(self, xyz, s=CHUNK_S):
        if self.left < 60:
            return None
        r = self.api.move([float(v) for v in xyz], seconds=s)
        self.left -= max(40, int(120 * max(s, 0.5)))
        return r

    def grip(self, w):
        self.api.grip(w)
        self.left -= 20


def panel_y(rgb, P, xlo, xhi, zbar, yface):
    """Modal y of the middle drawer's front panel, sampled clear of the handles."""
    X, Y, Z = P
    f = np.asarray(rgb, np.float64)
    mx = f.max(2)
    sat = (mx - f.min(2)) / np.maximum(mx, 1.0)
    m = (np.isfinite(Z) & (sat < SAT_MAX) & (X > xlo) & (X < xhi)
         & (np.abs(Z - zbar) < 0.028) & (Y < yface + 0.02) & (Y > yface - 0.28))
    ys = Y[m]
    if ys.size < 60:
        return None
    h, edges = np.histogram(ys, bins=np.arange(yface - 0.28, yface + 0.025, 0.005))
    return float(edges[int(np.argmax(h))] + 0.0025)


def rot_err(api, R):
    C = np.asarray(api.tool_rotation(), float)
    c = (np.trace(R @ C.T) - 1.0) / 2.0
    return float(np.arccos(max(-1.0, min(1.0, c))))


def log_state(api, tag):
    api.log("ST %s %s" % (tag, json.dumps({
        "eef": [round(float(v), 4) for v in api.eef()],
        "grip": {k: round(float(v), 4) for k, v in api.gripper().items()}})))


def run(api):
    B = Budget(api)
    f = api.capture("cam_high")
    cab = find_cabinet(np.asarray(f.rgb), cloud(f))
    api.log("CAB " + json.dumps(cab))
    if cab is None or len(cab["bars"]) != 3:
        api.log("ABORT bars")
        return
    bot, mid, top = cab["bars"]
    xg = float(mid["xmid"])
    yface = float(cab["yface"])
    yfront = float(mid["yfront"])
    # centre of the free vertical channel between the bar below and the bar
    # above; zhi is the silhouette top, which cam_high measures directly, and
    # the bar radius is the measured closed-gripper gap.
    zstar = (bot["zhi"] + top["zhi"]) / 2.0 - R_BAR
    ystage = yfront - STAND_OFF
    ytarget = yface - 0.010
    api.log("PLAN " + json.dumps({"xg": xg, "zstar": zstar, "yfront": yfront, "yface": yface}))

    for i in range(2):
        r = B.lin([xg, ystage - ROT_DY, zstar - ROT_DZ])
        api.log("S1.%d res=%s left=%d" % (i, r, B.left))
        log_state(api, "S1.%d" % i)
        if r is None or r < 0.012:
            break
    for i in range(2):
        B.pose(api.eef(), R_SIDE)
        re = rot_err(api, R_SIDE)
        api.log("S2.%d roterr=%.3f left=%d" % (i, re, B.left))
        log_state(api, "S2.%d" % i)
        if re < 0.06:
            break

    through = False
    for attempt, dz in enumerate((0.0, -0.005)):
        if B.left < 260:
            break
        z = zstar + dz
        B.pose([xg, ystage, z], R_SIDE)          # trim height at the stand-off
        api.log("S3.%d got=%s left=%d" % (attempt, np.round(np.asarray(api.eef(), float), 4).tolist(), B.left))
        for i in range(2):
            B.pose([xg, ytarget, z], R_SIDE)
            e = np.asarray(api.eef(), float)
            api.log("S4.%d.%d got=%s left=%d" % (attempt, i, np.round(e, 4).tolist(), B.left))
            if float(e[1]) > ytarget - 0.015:
                break
        e = np.asarray(api.eef(), float)
        through = float(e[1]) > yface - 0.030
        api.log("S4.%d through=%s y=%.4f z=%.4f" % (attempt, through, e[1], e[2]))
        if through:
            break

    for i in range(3):
        B.grip(0.0)
        api.log("G%d grip=%s left=%d" % (i, json.dumps(api.gripper()), B.left))
    log_state(api, "closed")

    # Pull.  Two stages: first with the grasp wrist, then with the wrist rolled
    # 30 deg back about the bar axis.  A cylinder grip is free to roll about its
    # own axis, and the shallower wrist is what lets the arm keep retracting in
    # -y (under the 90 deg wrist the arm walls out at y ~ 0.06 -- p7/p8).
    # Every target is re-derived from the current eef so the command is pure -y;
    # a standing lateral error binds the drawer in its slide (v4 receipt).
    i = 0
    while B.left >= 60:
        R = R_SIDE if i < 2 else R_TILT
        p = np.asarray(api.eef(), float)
        B.pose([p[0], p[1] - PULL_FAR, p[2]], R)
        q = np.asarray(api.eef(), float)
        api.log("D%d got=%s grip=%s left=%d"
                % (i, np.round(q, 4).tolist(), json.dumps(api.gripper()), B.left))
        i += 1
    log_state(api, "end")
    f3 = api.capture("cam_high")
    xa = max(float(mid["xhi"]) + 0.045, float(cab["xlo"]))
    xb = float(cab["xhi"]) - 0.010
    api.log("ENDPANEL y=%s (face was %.4f) window=%.3f..%.3f"
            % (panel_y(np.asarray(f3.rgb), cloud(f3), xa, xb, float(mid["zhi"]) - R_BAR, yface),
               yface, xa, xb))
    api.log("DONE v12 through=%s left=%d" % (through, B.left))
