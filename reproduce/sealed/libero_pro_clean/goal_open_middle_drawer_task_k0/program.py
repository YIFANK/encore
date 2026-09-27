"""v33: the bottom drawer's bar cannot be engaged (see NOTES: the hand is taller
than the 48 mm gap under the bar, and the 11 mm slot behind the bar is narrower
than the 15 mm finger pair).  This version does everything the arm CAN do: it
side-pinches and pulls open every handle bar it can reach, middle first, then
top, verifying each grip with the closed width."""
import numpy as np

PROVENANCE = {
    "FACE_Y": {"source": "debug 51/53/57 cam_high: modal y of elevated cabinet points (-0.157/-0.162/-0.172)", "allowed": True},
    "HANDLE_BAND": {"source": "debug 51/53/57: points protruding face_y+0.006..+0.06 cluster in z at 0.953/1.021/1.095", "allowed": True},
    "BAR_X": {"source": "debug 51/53/57: band x spans -0.006..0.081 (bar axis along base x)", "allowed": True},
    "BAR_THICK": {"source": "debug 51 v5: closed width 0.0174 at effort 3.0 on the bar", "allowed": True},
    "GRASP_DY": {"source": "debug 51 v4/v5 depth ladder: grip at eef y = ytip-0.005", "allowed": True},
    "R_SIDE": {"source": "generic controller mechanics: tool z = -y approach, tool y = -z so the jaws close vertically", "allowed": True},
    "SIDE_FLOOR": {"source": "debug 51 v12/v13: horizontal wrist floors at eef z~1.00, so only the two upper bars are graspable", "allowed": True},
    "PULL_STEP": {"source": "debug 51 v5: 0.025 m steps keep width 0.0173 and effort 3.0 through 0.16 m of travel", "allowed": True},
}

R_SIDE = np.array([[-1.0, 0.0, 0.0],
                   [0.0, 0.0, -1.0],
                   [0.0, -1.0, 0.0]])
REACHABLE_BANDS = (1, 2)      # bottom band (0) is out of the arm's envelope
Z_FLOOR = 1.000               # horizontal wrist cannot go below this (v12/v13)


def _cloud(f):
    d = np.asarray(f.depth, dtype=np.float64)
    K = np.asarray(f.intrinsics, dtype=np.float64)
    T = np.asarray(f.t_base_cam, dtype=np.float64)
    h, w = d.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    pc = np.stack([(u - K[0, 2]) / K[0, 0] * d, (v - K[1, 2]) / K[1, 1] * d, d], axis=-1)
    return pc @ T[:3, :3].T + T[:3, 3]


def perceive(api, tag=""):
    f = api.capture("cam_high")
    P = _cloud(f).reshape(-1, 3)
    e = P[(P[:, 2] > 0.93) & (P[:, 2] < 1.15) & (P[:, 1] < 0.30) & (P[:, 1] > -0.5)
          & (P[:, 0] > -0.35) & (P[:, 0] < 0.30)]
    hist, edges = np.histogram(e[:, 1], bins=np.arange(-0.5, 0.30, 0.005))
    fy = edges[int(hist.argmax())] + 0.0025
    pr = e[(e[:, 1] > fy + 0.006) & (e[:, 1] < fy + 0.06)
           & (e[:, 0] > -0.12) & (e[:, 0] < 0.25)]
    hz, bz = np.histogram(pr[:, 2], bins=np.arange(0.90, 1.16, 0.01))
    bands = []
    i = 0
    while i < len(hz):
        if hz[i] > 20:
            j = i
            while j < len(hz) and hz[j] > 20:
                j += 1
            s = pr[(pr[:, 2] >= bz[i]) & (pr[:, 2] < bz[j])]
            bands.append(dict(z=float(s[:, 2].mean()), n=int(len(s)),
                              x0=float(np.percentile(s[:, 0], 3)),
                              x1=float(np.percentile(s[:, 0], 97)),
                              ytip=float(np.percentile(s[:, 1], 98))))
            i = j
        else:
            i += 1
    api.log("PERCEIVE%s face_y=%.4f nbands=%d" % (tag, fy, len(bands)))
    for b in bands:
        api.log("  BAND%s z=%.4f n=%d x=%.3f..%.3f ytip=%.4f" % (tag, b["z"], b["n"], b["x0"], b["x1"], b["ytip"]))
    return fy, bands


def goto(api, p, tries=2, seconds=2.0, tol=0.008):
    p = np.asarray(p, dtype=float)
    cmd = p.copy()
    for _ in range(tries):
        api.move(cmd.tolist(), rotation=R_SIDE, seconds=seconds)
        e = np.asarray(api.eef())
        err = p - e
        if np.linalg.norm(err) < tol:
            break
        cmd = cmd + np.clip(err, -0.06, 0.06)
    return np.asarray(api.eef())


def held(api):
    g = api.gripper()
    return 0.008 < g["width_m"] < 0.035 and g["effort"] > 1.0, g


def open_bar(api, b):
    bx = 0.5 * (b["x0"] + b["x1"])
    bz = b["z"]
    ytip = b["ytip"]
    api.log("BAR x=%.4f z=%.4f ytip=%.4f" % (bx, bz, ytip))
    api.grip(0.08)
    api.move([bx, ytip + 0.22, bz + 0.08], seconds=2.0)
    goto(api, [bx, ytip + 0.14, bz], tries=2, seconds=2.0)

    gy = None
    for dy in (-0.005, 0.010):
        goto(api, [bx, ytip + dy, bz], tries=2, seconds=1.2)
        api.grip(0.0)
        api.settle(0.2)
        ok, g = held(api)
        api.log("  TRY dy=%.3f width=%.4f effort=%.2f eef=%s ok=%s"
                % (dy, g["width_m"], g["effort"], np.round(api.eef(), 4).tolist(), ok))
        if ok:
            gy = dy
            break
        api.grip(0.08)
    if gy is None:
        api.log("  NOGRIP")
        return False

    e = np.asarray(api.eef())
    x0, z0 = e[0], e[2]
    y = float(e[1])
    for k in range(8):
        y += 0.025
        goto(api, [x0, y, z0], tries=1, seconds=1.0, tol=0.01)
        ok, g = held(api)
        ee = np.asarray(api.eef())
        api.log("  PULL%d width=%.4f eef=%s" % (k, g["width_m"], np.round(ee, 4).tolist()))
        if not ok:
            api.log("  LOST at pull%d" % k)
            break
        y = float(ee[1])
    api.grip(0.08)
    api.settle(0.2)
    e = np.asarray(api.eef())
    api.move([e[0], e[1] + 0.10, e[2] + 0.08], rotation=R_SIDE, seconds=1.2)
    return True


def run(api):
    fy, bands = perceive(api)
    if len(bands) < 3:
        api.log("ABORT bands=%d" % len(bands))
        return
    for idx in REACHABLE_BANDS:
        b = bands[idx]
        if b["z"] < Z_FLOOR:
            api.log("band %d at z=%.3f is below the wrist floor; skipped" % (idx, b["z"]))
            continue
        ok = open_bar(api, b)
        api.log("band %d opened=%s" % (idx, ok))
    perceive(api, "_after")
