"""l90abl / open_top_drawer_s1_k3 -- v10  (press on the bar and drag)

The slot behind the top bar is not enterable.  v9's descent held the jaw to
within 3 mm of its commanded y all the way down and still stalled with the
fingertip at 1.0997 -- the bar's top -- at jaw y=-0.2229 and -0.2200, while a
jaw at -0.2251 stalled at 1.1305, the cabinet top.  So the bar body runs back
to about y=-0.215, the counter lip reaches y=-0.225, and what is left between
them is thinner than the finger.

But that stall pose IS the pack's contact pose: v9 stalls at eef
(0.003,-0.180,1.1059), fingertip 1.0995; the pack's three contact keyframes sit
at eef y -0.163/-0.170/-0.177, z 1.1045/1.1046/1.1068, fingertip 1.098..1.100.
The pack is not hooking anything -- it rests the open fingertip on top of the
bar and drags the drawer out by friction.

v7 reproduced that pose and still failed because it then commanded the pull at
the height the descent had reached, so the wrist floated 3 mm up and let go.
v10 keeps commanding a z BELOW the contact throughout the pull, so the
controller holds the finger down on the bar while it translates +y, and it
ladders over how hard to press.
"""

import numpy as np

PROVENANCE = {
    "CAB_TOP": {"source": "v2 height maps, all 15 debug seeds: cabinet top z=1.131",
                "allowed": True},
    "CAB_Y/MID_X": {
        "source": "v2 debug-seed height maps: cabinet y in (-0.40,-0.23); MID_X "
                  "keeps the face sample off the cabinet's +x front edge at x~0.11",
        "allowed": True},
    "PROTRUDE_MIN": {
        "source": "v3 fine max-y map (seeds 51/53/55/57): face -0.2317 vs bars "
                  "-0.1995, a 32 mm step",
        "allowed": True},
    "JAW_HALF": {
        "source": "debug-seed api.gripper(): open width 0.080 m; tool rotation "
                  "column 1 = (0,-1,0), so each jaw is 0.040 m from the eef in y",
        "allowed": True},
    "TIP_DROP": {
        "source": "v6 debug seeds: open jaws pressed onto the cabinet top "
                  "(z=1.131) stalled the eef at z=1.1374",
        "allowed": True},
    "JAW_DEPTHS": {
        "source": "pack.json contact keyframes put the far jaw at y=-0.203/-0.210/"
                  "-0.217, i.e. 0.014-0.028 m in front of the perceived face; v9 "
                  "showed -0.220 and -0.223 both land on the bar top and -0.225 "
                  "on the counter lip",
        "allowed": True},
    "PRESS": {
        "source": "v7 debug seeds: pulling at the height the descent reached let "
                  "the wrist rise 3 mm off the bar (res 0.008, no load); the "
                  "press must exceed api.move's 12 mm stop band to stay in contact",
        "allowed": True},
    "PULL_LEN/PULL_STEP": {
        "source": "pack.json: +y travel of 0.177/0.177/0.225 m from contact, run "
                  "in the demos at roughly 0.04-0.06 m per keyframe interval",
        "allowed": True},
    "PLACE_TOL/STEP": {
        "source": "generic controller mechanics: api.move's stop band is 12 mm, so "
                  "placement must be re-commanded and a descent slice must exceed it",
        "allowed": True},
}

CAB_TOP = 1.131
CAB_Y = (-0.45, -0.10)
MID_X = (-0.12, 0.07)
PROTRUDE_MIN = 0.012
JAW_HALF = 0.040
TIP_DROP = 0.0064
JAW_DEPTHS = (0.014, 0.021, 0.009)
PRESS = (0.022, 0.045)
PULL_LEN = 0.22
PULL_STEP = 0.040
PLACE_TOL = 0.003
STEP = 0.020
OPEN_W = 0.080


def cloud(f):
    d = np.asarray(f.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    vv, uu = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 0)
    z = np.where(ok, d, 1.0)
    xc = (uu - K[0, 2]) * z / K[0, 0]
    yc = (vv - K[1, 2]) * z / K[1, 1]
    P = (T[:3, :3] @ np.stack([xc, yc, z]).reshape(3, -1)).reshape(3, h, w) \
        + T[:3, 3][:, None, None]
    return P[0].ravel(), P[1].ravel(), P[2].ravel(), ok.ravel()


def perceive(api):
    f = api.capture("cam_high")
    X, Y, Z, ok = cloud(f)
    m = ok & (X > MID_X[0]) & (X < MID_X[1]) & (Y > CAB_Y[0]) & (Y < CAB_Y[1]) \
        & (Z > 0.92) & (Z < CAB_TOP + 0.005)
    x, y, z = X[m], Y[m], Z[m]
    if x.size < 500:
        return None
    bmax, bpts = {}, {}
    for zc in np.arange(0.93, CAB_TOP - 0.015, 0.005):
        s = (z >= zc) & (z < zc + 0.005)
        if s.sum() < 5:
            continue
        k = round(float(zc), 3)
        bmax[k] = float(np.max(y[s]))
        bpts[k] = (x[s], y[s])
    if len(bmax) < 10:
        return None
    face_y = float(np.median(list(bmax.values())))
    bands = [k for k in sorted(bmax) if bmax[k] > face_y + PROTRUDE_MIN]
    api.log(f"perceive face_y={face_y:.4f} bands={[(k, round(bmax[k], 4)) for k in bands]}")
    if not bands:
        return None
    groups, cur = [], [bands[0]]
    for k in bands[1:]:
        if k - cur[-1] <= 0.0101:
            cur.append(k)
        else:
            groups.append(cur)
            cur = [k]
    groups.append(cur)
    top = groups[-1]
    bar_z0, bar_z1 = top[0], top[-1] + 0.005
    bar_y = float(np.median([bmax[k] for k in top]))
    xs = np.concatenate([bpts[k][0][bpts[k][1] > face_y + PROTRUDE_MIN] for k in top])
    bar_xc = 0.5 * (float(np.percentile(xs, 5)) + float(np.percentile(xs, 95)))
    api.log(f"TOPBAR z=[{bar_z0:.3f},{bar_z1:.3f}] front_y={bar_y:.4f} "
            f"xc={bar_xc:.4f} ngroups={len(groups)}")
    return dict(face_y=face_y, bar_z0=bar_z0, bar_z1=bar_z1, bar_y=bar_y,
                bar_xc=bar_xc)


def place(api, target, seconds=0.9, tries=3, tol=PLACE_TOL):
    target = np.asarray(target, float)
    cmd = target.copy()
    e = api.eef()
    for _ in range(tries):
        api.move(cmd, seconds=seconds)
        e = api.eef()
        err = target - e
        if float(np.max(np.abs(err))) < tol:
            break
        cmd = cmd + err
    return e, cmd


def guarded_descend(api, want_xy, cmd, z_to):
    """Descend in slices larger than api.move's 12 mm stop band, holding x/y."""
    cmd = np.asarray(cmd, float).copy()
    e = api.eef()
    z = float(e[2])
    while z > z_to + 1e-6:
        z = max(z - STEP, z_to)
        cmd[0] += want_xy[0] - e[0]
        cmd[1] += want_xy[1] - e[1]
        api.move((cmd[0], cmd[1], z), seconds=0.5)
        e2 = api.eef()
        drop = e[2] - e2[2]
        api.log(f"   step z={z:.4f} eef={np.round(e2, 4).tolist()} drop={drop:.4f}")
        if drop < 0.40 * STEP:
            return e2, cmd, "stall"
        e = e2
    return e, cmd, "ok"


def run(api):
    api.log(f"start eef={np.round(api.eef(), 4).tolist()} grip={api.gripper()}")
    p = perceive(api)
    if p is None:
        return "v10 perception failed"
    xc = float(np.clip(p["bar_xc"], -0.02, 0.02))
    z_high = p["bar_z1"] + 0.022 + TIP_DROP
    z_low = p["bar_z0"] - 0.020 + TIP_DROP
    api.log(f"xc={xc:.4f} z_high={z_high:.4f} z_low={z_low:.4f}")
    api.grip(OPEN_W)

    for press in PRESS:
        for dy in JAW_DEPTHS:
            y_eef = p["face_y"] + dy + JAW_HALF
            e, cmd = place(api, (xc, y_eef, z_high), seconds=0.9, tries=3)
            api.log(f"RUNG press={press:.3f} dy={dy:.3f} want_y={y_eef:.4f} "
                    f"above={np.round(e, 4).tolist()}")
            if abs(e[1] - y_eef) > 0.006:
                api.log("  placement off; next rung")
                continue

            e, cmd, how = guarded_descend(api, (xc, y_eef), cmd, z_low)
            tip = float(e[2]) - TIP_DROP
            api.log(f"  descend {how} eef={np.round(e, 4).tolist()} tip={tip:.4f}")
            if how != "stall" or tip > p["bar_z1"] + 0.010:
                api.log("  no bar contact; next rung")
                api.move((cmd[0], cmd[1], z_high + 0.02), seconds=0.8)
                continue

            # drag: keep commanding BELOW the contact so the finger stays loaded
            z_press = float(e[2]) - press
            y = float(e[1])
            y_goal = p["face_y"] + PULL_LEN
            while y < y_goal - 1e-6:
                y = min(y + PULL_STEP, y_goal)
                r = api.move((cmd[0], y, z_press), seconds=0.6)
                ee = api.eef()
                api.log(f"  drag y={y:.3f} eef={np.round(ee, 4).tolist()} res={r:.4f} "
                        f"w={api.gripper()['width_m']:.4f}")

            api.move((cmd[0], float(api.eef()[1]), z_high + 0.03), seconds=0.8)

    return "v10 press-and-drag ladder"
