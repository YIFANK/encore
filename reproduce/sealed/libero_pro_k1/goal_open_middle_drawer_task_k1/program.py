"""c2k1clean / goal_open_middle_drawer_task_k1  --  FINAL CANDIDATE (v16)

Intent: "open the bottom drawer of the cabinet".

The K=1 pack demonstrates the MIDDLE drawer -- pack.json's own `language`
field says "open the middle drawer of the cabinet" -- so the pack is read for
MECHANISM only and the instruction names the TARGET.

Mechanism, read off the pack:
  * the 7th slot of every one of the 138 raw actions is -1.0 and every
    keyframe's gripper_cmd is -1.0, so the demo NEVER closes its jaws: this is
    an open-jaw hook, not a grasp;
  * `ee_path6`'s rotation triples are rotation VECTORS (see PROVENANCE), and
    they tilt the wrist from straight down to an approach axis of
    (-0.059,-0.785,-0.617) -- 38 deg below horizontal, pointing into the
    cabinet, which swings the hand body up and away from the drawer face;
  * the hook then drags +y by 0.170 m at constant height.

Target, measured by me: the cabinet's three pull-bars are the three z bands
that protrude in +y from the cabinet top slab's front edge in my own cam_high
depth.  The BOTTOM one is bars[0].  Its own x/y readout is polluted by a prop
standing on the table beside the cabinet, so x and y come from the two upper
bars (which the drawer above shields) and only z comes from the low band.

The demo's engage pose is then re-anchored onto that bar by the offsets the
demo holds relative to its own bar, and the whole approach arc is translated
with it.  A post-hoc re-perception (arm retracted first) measures how far the
drawer actually travelled, and the hook is repeated once if it did not.
"""
import numpy as np

PROVENANCE = {
    "DEMO (5 waypoints) / D_ENGAGE / PULL_RVEC": {
        "source": "pack.json demos[0].ee_path6 entries t=50,60,70,80,90,130. "
                  "The rotation triples are rotation VECTORS, not euler rpy: "
                  "rv2R([3.1477,-0.0273,-0.0727]) (the t=0 triple) is "
                  "symmetric and matches the symmetric matrix "
                  "api.tool_rotation() returns at the start of debug seed 51, "
                  "[[0.9984,0.0005,-0.0568],[0.0005,-1,0],[-0.0568,0,-0.9984]], "
                  "while both euler orders give a non-symmetric matrix with "
                  "the off-diagonal terms in the wrong slots",
        "allowed": True},
    "PULL_M": {
        "source": "pack.json demos[0].ee_path: the drag runs y=-0.1453 (the "
                  "t=89 keyframe, where y stops decreasing) to y=+0.0247 at "
                  "t=130, i.e. 0.170 m of +y; rounded up to 0.20",
        "allowed": True},
    "DY_ENGAGE": {
        "source": "debug-seed measurement: the demo's engage y (-0.1453, "
                  "pack keyframes[1].ee) minus the middle pull-bar's front y "
                  "as I measure it on debug seeds 51/53/57/61 "
                  "(-0.126/-0.129/-0.139/-0.133, mean -0.133)",
        "allowed": True},
    "DZ_ENGAGE": {
        "source": "debug-seed measurement: the demo's engage z (1.0322) minus "
                  "the middle pull-bar's top z, which measures 1.0245 on every "
                  "one of debug seeds 51/53/57/61",
        "allowed": True},
    "bars / yface / ztab / ztop (find_bars, bottom_bar)": {
        "source": "own cam_high RGB-D on debug seeds 51/53/57/61, deprojected "
                  "with the FairFrame's own intrinsics and t_base_cam. Table "
                  "plane 0.9025 = densest z bin; cabinet top slab 1.1275 = "
                  "densest z band above it at y<0; its x extent = the longest "
                  "run of populated 2 cm x-bins; yface = its 99th-percentile "
                  "y. The three pull-bars are the z bands of points that "
                  "protrude past yface, tops 0.9557 / 1.0245 / 1.0979 on every "
                  "seed",
        "allowed": True},
    "RETRACT / YWIN / MIN_TRAVEL": {
        "source": "own debug-seed observations: the arm must leave the "
                  "cam_high view before the re-perception or it is itself "
                  "detected as a band (v1 log); a successful drag moves the "
                  "bar 0.17 in +y (v11 measurement), so anything under 0.10 "
                  "means the hook slipped",
        "allowed": True},
}

DEMO = [
    (-0.0454, 0.0045, 1.1011, 1.9334, 1.6249, -0.9184),
    (0.0088, -0.0317, 1.1011, 1.7539, 1.7867, -1.0813),
    (0.0272, -0.0780, 1.0849, 1.7476, 1.7367, -1.0470),
    (0.0075, -0.1283, 1.0500, 1.7657, 1.6519, -0.9118),
    (-0.0036, -0.1453, 1.0322, 1.7341, 1.6522, -0.8649),
]
D_ENGAGE = np.array([-0.0036, -0.1453, 1.0322])
PULL_RVEC = (1.4084, 1.4306, -1.1595)
TILT_RVEC = (1.7341, 1.6522, -0.8649)
PULL_M = 0.20
DY_ENGAGE = -0.012
DZ_ENGAGE = 0.0075
YWIN = 0.40
MIN_TRAVEL = 0.10


def rv2R(v):
    v = np.asarray(v, float)
    th = float(np.linalg.norm(v))
    if th < 1e-9:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


# --- perception -------------------------------------------------------------
def base_cloud(fr, step=2):
    d = np.asarray(fr.depth, float)
    K = np.asarray(fr.intrinsics, float)
    T = np.asarray(fr.t_base_cam, float)
    dd = d[::step, ::step]
    H, W = dd.shape
    s = d.shape[1] / W
    fx, fy = K[0, 0] / s, K[1, 1] / s
    cx, cy = K[0, 2] / s, K[1, 2] / s
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    P = np.stack([(u - cx) / fx * dd, (v - cy) / fy * dd, dd], -1)
    return P @ T[:3, :3].T + T[:3, 3]


def _longest_run(counts, thr):
    best = (0, -1)
    cur = None
    for i, c in enumerate(counts):
        if c > thr:
            cur = i if cur is None else cur
            if i - cur > best[1] - best[0]:
                best = (cur, i)
        else:
            cur = None
    return best


def find_bars(B, ywin=0.12):
    P = B.reshape(-1, 3)
    P = P[(P[:, 0] > -0.6) & (P[:, 0] < 0.6) & (P[:, 1] > -0.6) &
          (P[:, 1] < 0.6) & (P[:, 2] > 0.5) & (P[:, 2] < 1.5)]
    hz, ez = np.histogram(P[:, 2], bins=np.arange(0.5, 1.5, 0.005))
    ztab = float(ez[int(np.argmax(hz))] + 0.0025)
    C = P[(P[:, 1] < 0.0) & (P[:, 2] > ztab + 0.10)]
    hz, ez = np.histogram(C[:, 2], bins=np.arange(ztab + 0.10, 1.5, 0.01))
    ztop = float(ez[int(np.argmax(hz))] + 0.005)
    S = C[np.abs(C[:, 2] - ztop) < 0.015]
    hx, ex = np.histogram(S[:, 0], bins=np.arange(-0.6, 0.62, 0.02))
    i0, i1 = _longest_run(hx, 0.20 * hx.max())
    x0, x1 = float(ex[i0]), float(ex[i1 + 1])
    S = S[(S[:, 0] >= x0) & (S[:, 0] <= x1)]
    yface = float(np.percentile(S[:, 1], 99.0))

    Hp = P[(P[:, 0] > x0 - 0.01) & (P[:, 0] < x1 + 0.01) &
           (P[:, 1] > yface + 0.006) & (P[:, 1] < yface + ywin) &
           (P[:, 2] > ztab + 0.025) & (P[:, 2] < ztop - 0.01)]
    edges = np.arange(ztab + 0.025, ztop, 0.005)
    hz, ez = np.histogram(Hp[:, 2], bins=edges)
    thr = max(8.0, 0.12 * hz.max())
    groups, cur = [], []
    for i, c in enumerate(hz):
        if c > thr:
            cur.append(i)
        elif cur:
            groups.append(cur); cur = []
    if cur:
        groups.append(cur)
    bars = []
    for g in groups:
        lo, hi = float(ez[g[0]]), float(ez[g[-1] + 1])
        Q = Hp[(Hp[:, 2] >= lo) & (Hp[:, 2] <= hi)]
        if len(Q) < 30 or (hi - lo) > 0.035:
            continue
        bars.append({"n": int(len(Q)), "lo": lo, "hi": hi,
                     "ztop": float(np.percentile(Q[:, 2], 98)),
                     "xlo": float(np.percentile(Q[:, 0], 1)),
                     "xhi": float(np.percentile(Q[:, 0], 99)),
                     "yfront": float(np.percentile(Q[:, 1], 98))})
    bars.sort(key=lambda r: r["ztop"])
    return {"ztab": ztab, "ztop": ztop, "x0": x0, "x1": x1,
            "yface": yface, "bars": bars, "cloud": Hp}


def bottom_bar(res):
    """The lowest bar's own xlo/yfront can be polluted by a prop standing on
    the table beside the cabinet; the upper bars are shielded by the drawer
    above them, so take x and y from their consensus and z from the low band."""
    bars = res["bars"]
    if not bars:
        return None
    upper = bars[1:] if len(bars) > 1 else bars
    xc = float(np.mean([(b["xlo"] + b["xhi"]) / 2 for b in upper]))
    yf = float(np.mean([b["yfront"] for b in upper]))
    return {"xc": xc, "yfront": yf, "ztop": bars[0]["ztop"],
            "nbars": len(bars)}




def run(api):
    api.log("instr=%r" % api.instruction())
    api.grip(0.08)                      # the demo never closes; keep it open
    RP = rv2R(PULL_RVEC)
    RT = rv2R(TILT_RVEC)

    res = find_bars(base_cloud(api.capture("cam_high")), ywin=0.12)
    api.log("perc ztab=%.4f ztop=%.4f yface=%.4f nbars=%d"
            % (res["ztab"], res["ztop"], res["yface"], len(res["bars"])))
    for b in res["bars"]:
        api.log("  bar ztop=%.4f x[%.4f,%.4f] yfront=%.4f n=%d"
                % (b["ztop"], b["xlo"], b["xhi"], b["yfront"], b["n"]))
    t = bottom_bar(res)
    if t is None:
        api.log("NO BARS -- abort")
        return
    xc, ybar, zbar = t["xc"], t["yfront"], t["ztop"]
    api.log("BOTTOM bar xc=%.4f ybar=%.4f zbar=%.4f" % (xc, ybar, zbar))

    for c in range(2):
        E = np.array([xc, ybar + DY_ENGAGE, zbar + DZ_ENGAGE])
        api.log("cycle%d E=%s" % (c, E.round(4).tolist()))
        if c == 0:
            delta = E - D_ENGAGE
            for k, w in enumerate(DEMO):
                p = np.array(w[:3]) + delta
                r = api.move(p, rotation=rv2R(w[3:]), seconds=1.0)
                api.log(" apr%d res=%.4f eef=%s"
                        % (k, r, np.asarray(api.eef()).round(4).tolist()))
        else:
            api.move([xc, E[1] + 0.13, E[2] + 0.06], rotation=RT, seconds=1.2)
            r = api.move(E, rotation=RT, seconds=0.9)
            api.log(" re-engage res=%.4f eef=%s"
                    % (r, np.asarray(api.eef()).round(4).tolist()))
        for k in (0.35, 0.70, 1.00):
            p = E + np.array([0.0, PULL_M * k, 0.0])
            r = api.move(p, rotation=RP, seconds=1.0)
            api.log(" pull%.2f res=%.4f eef=%s"
                    % (k, r, np.asarray(api.eef()).round(4).tolist()))

        api.move([xc, E[1] + PULL_M + 0.12, E[2] + 0.17], rotation=RT,
                 seconds=1.5)
        api.settle(0.4)
        res2 = find_bars(base_cloud(api.capture("cam_high")), ywin=YWIN)
        low = [b for b in res2["bars"] if abs(b["ztop"] - zbar) < 0.02]
        if not low:
            api.log("cycle%d: bottom band not re-found" % c)
            return
        travel = low[0]["yfront"] - ybar
        api.log("cycle%d TRAVEL %.4f (yfront %.4f -> %.4f)"
                % (c, travel, ybar, low[0]["yfront"]))
        if travel >= MIN_TRAVEL:
            return
        ybar = low[0]["yfront"]
