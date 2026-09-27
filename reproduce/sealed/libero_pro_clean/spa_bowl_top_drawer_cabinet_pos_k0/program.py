"""Pick the bowl out of the open top drawer and set it on the plate.

Zero demonstrations: every number below was measured from this cell's own
debug-seed (51..65) cam_high RGB-D and from its own probe episodes.

Shape of the solution
  - The three bowls in the scene are the only bright (min-channel > 100)
    objects above the table.  They sit at three well-separated heights --
    table+0.279 on the cabinet top, table+0.215 in the drawer, table+0.043
    on the table -- so the drawer one is the middle of the three.
  - The bowl is 0.110 m across and the gripper opens 0.078 m, so the only
    grasp available is a pinch on the rim wall.  The drawer walls leave
    0.015 m of clearance on the near (+x) side and 0.030 m in y, but 0.075 m
    on the far (-x) side, so the bite goes on the far arc with the wrist
    yawed 90 deg to put the jaws along x.
  - The wall tapers inward with depth (0.45 m/m), so the pinch radius is
    read off the measured interior profile at the planned bite depth rather
    than taken from the rim.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "TABLE_BAND": {"source": "debug 51-65 cam_high depth: modal z of the workspace cloud is the table plane (0.9010 m, std 0.1 mm)", "allowed": True},
    "BRIGHT_CUT": {"source": "debug 51-65 cam_high RGB: the three bowls are the only min-channel>100 objects above the table", "allowed": True},
    "BOWL_SIZE": {"source": "debug 51-65: bowl bright clusters are 0.09-0.14 m across, n>900; the drawer handle is 0.032 m radius / n<700", "allowed": True},
    "TARGET_BAND": {"source": "debug 51-65: three bowls sit at table+0.279 (cabinet top), table+0.215 (drawer), table+0.043 (table); the drawer one is the middle", "allowed": True},
    "PLATE_SHAPE": {"source": "debug 51-65: the plate is the only 0.134-0.137 m round flat cluster, ztop=table+0.019", "allowed": True},
    "TIP_OFF": {"source": "debug 51 v4: cam_high depth-difference between two hover heights puts the lowest gripper point 0.0078/0.0069 m below the eef; the wrist camera agrees (0.0093)", "allowed": True},
    "BITE": {"source": "debug 51/57 v5: three bites at rim-0.013..0.017 all closed on the wall (gap 0.0063-0.0084, effort 3.0)", "allowed": True},
    "CARRY_Z": {"source": "debug 51-65: drawer walls and cabinet top read 1.12, the cabinet-top bowl 1.18; 1.26 clears both with the bowl hanging ~0.04 below the eef", "allowed": True},
    "PARK": {"source": "debug 51-65: (-0.34,0.30) reads a clear table plane and is 0.14 m clear of the plate rim", "allowed": True},
    "HOLD_GAP": {"source": "debug 51/57/62 v5-v6: a bite on the wall leaves a finger gap 0.0063-0.0084 m; a miss or a slip leaves 0.0010-0.0018", "allowed": True},
}

TIP_OFF = 0.008
BITE = 0.024
CARRY_Z = 1.26
HOLD_GAP = 0.004
PARK = (-0.34, 0.30, 1.28)
R_YAW = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
XB = (-0.55, 0.32)
YB = (-0.55, 0.55)


def _log(api, *a):
    api.log(" ".join(str(x) for x in a))


def dump(api, tag, cam="cam_high"):
    f = api.capture(cam)
    rgb = np.asarray(f.rgb, dtype=np.uint8)
    d = (np.asarray(f.depth, dtype=np.float32) * 10000.0).astype(np.uint16)
    api.log("K %s %s" % (tag, np.asarray(f.intrinsics).ravel().tolist()))
    api.log("T %s %s" % (tag, np.asarray(f.t_base_cam).ravel().tolist()))
    for name, arr in (("rgb", rgb), ("dep", d)):
        blob = base64.b64encode(zlib.compress(arr.tobytes(), 6)).decode()
        ch = [blob[i:i + 1900] for i in range(0, len(blob), 1900)]
        api.log("BLOB %s %s nchunks=%d dtype=%s shape=%s" % (tag, name, len(ch), arr.dtype, arr.shape))
        for i, c in enumerate(ch):
            api.log("B %s %s %d %s" % (tag, name, i, c))


def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.asarray(f.depth, float)
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    P = np.stack([(u - K[0, 2]) / K[0, 0] * d, (v - K[1, 2]) / K[1, 1] * d, d], -1)
    return P @ T[:3, :3].T + T[:3, 3], np.asarray(f.rgb)


def clusters(P, res=0.010, minpts=40):
    if len(P) == 0:
        return []
    ix = np.floor((P[:, 0] - XB[0]) / res).astype(int)
    iy = np.floor((P[:, 1] - YB[0]) / res).astype(int)
    nx, ny = int(ix.max()) + 2, int(iy.max()) + 2
    cnt = np.zeros((nx, ny), int)
    np.add.at(cnt, (ix, iy), 1)
    occ = cnt >= 2
    lab = np.zeros((nx, ny), int)
    cur = 0
    for i in range(nx):
        for j in range(ny):
            if occ[i, j] and lab[i, j] == 0:
                cur += 1
                st = [(i, j)]
                lab[i, j] = cur
                while st:
                    a, b = st.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if 0 <= p < nx and 0 <= q < ny and occ[p, q] and lab[p, q] == 0:
                                lab[p, q] = cur
                                st.append((p, q))
    pl = lab[ix, iy]
    return [P[pl == k] for k in range(1, cur + 1) if (pl == k).sum() >= minpts]


def kasa(x, y):
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    c = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)[0]
    return float(c[0]), float(c[1]), float(np.sqrt(max(c[2] + c[0] ** 2 + c[1] ** 2, 1e-9)))


def perceive(api):
    B, rgb = cloud(api)
    eef = np.asarray(api.eef(), float)
    ws = (B[..., 0] > XB[0]) & (B[..., 0] < XB[1]) & (B[..., 1] > YB[0]) & (B[..., 1] < YB[1])
    h, e = np.histogram(B[..., 2][ws], bins=np.arange(0.7, 1.5, 0.002))
    tz = float(e[int(h.argmax())] + 0.001)

    bowls = []
    bright = ws & (rgb.min(-1) > 100) & (B[..., 2] > tz + 0.02)
    for P in clusters(B[bright]):
        ex = P[:, 0].max() - P[:, 0].min()
        ey = P[:, 1].max() - P[:, 1].min()
        cx, cy = P[:, 0].mean(), P[:, 1].mean()
        if len(P) < 900 or not (0.07 < ex < 0.20 and 0.07 < ey < 0.20):
            continue
        if max(ex, ey) / min(ex, ey) > 1.7:
            continue
        if abs(cx - eef[0]) < 0.09 and abs(cy - eef[1]) < 0.09:
            continue
        ztop = float(P[:, 2].max())
        rim = P[P[:, 2] > ztop - 0.008]
        rx, ry, rr = kasa(rim[:, 0], rim[:, 1])
        bowls.append(dict(ztop=ztop, h=ztop - tz, n=len(P), cx=rx, cy=ry, r=rr, P=P,
                          x0=float(rim[:, 0].min()), x1=float(rim[:, 0].max()),
                          y0=float(rim[:, 1].min()), y1=float(rim[:, 1].max())))
    bowls.sort(key=lambda b: b["h"])
    for b in bowls:
        _log(api, "BOWL h=%.3f ztop=%.3f n=%d c=(%.3f,%.3f) r=%.3f" % (b["h"], b["ztop"], b["n"], b["cx"], b["cy"], b["r"]))

    tgt = None
    mid = [b for b in bowls if 0.12 < b["h"] < 0.25]
    if len(mid) == 1:
        tgt = mid[0]
    elif len(bowls) >= 3:
        tgt = bowls[len(bowls) // 2]
    elif bowls:
        tgt = bowls[-1]

    plate = None
    flat = ws & (B[..., 2] > tz + 0.008) & (B[..., 2] < tz + 0.04)
    for P in clusters(B[flat], res=0.012, minpts=200):
        ex = P[:, 0].max() - P[:, 0].min()
        ey = P[:, 1].max() - P[:, 1].min()
        if not (0.11 < ex < 0.17 and 0.11 < ey < 0.17):
            continue
        if max(ex, ey) / min(ex, ey) > 1.2:
            continue
        c = dict(cx=float((P[:, 0].min() + P[:, 0].max()) / 2),
                 cy=float((P[:, 1].min() + P[:, 1].max()) / 2),
                 ztop=float(P[:, 2].max()), n=len(P), ex=ex, ey=ey)
        _log(api, "PLATE c=(%.3f,%.3f) ztop=%.3f ex=%.3f ey=%.3f n=%d" % (c["cx"], c["cy"], c["ztop"], ex, ey, c["n"]))
        if plate is None or c["n"] > plate["n"]:
            plate = c

    # support the target rests on: the ring of surface just outside its rim
    sup = tz
    if tgt is not None:
        rad = np.hypot(B[..., 0] - tgt["cx"], B[..., 1] - tgt["cy"])
        ring = ws & (rad > tgt["r"] + 0.008) & (rad < tgt["r"] + 0.030) & (B[..., 2] < tgt["ztop"] - 0.010)
        if ring.sum() > 50:
            sup = float(np.median(B[ring][:, 2]))
    return tz, tgt, plate, sup


def r_in(P, cx, cy, ztop, d):
    """Interior radius of the bowl wall at depth d below its rim."""
    r = np.hypot(P[:, 0] - cx, P[:, 1] - cy)
    m = np.abs(P[:, 2] - (ztop - d)) < 0.0025
    return float(np.median(r[m])) if m.sum() > 10 else float("nan")


def pick(api, tgt, tag):
    """One rim bite on the far (-x) arc. -> (holding, pinch radius)."""
    P = tgt["P"]
    cx, cy, ztop = tgt["cx"], tgt["cy"], tgt["ztop"]
    rp = r_in(P, cx, cy, ztop, 0.012)
    if not (0.030 < rp < 0.070):
        rp = 0.90 * tgt["r"]
    gx, gy = cx - rp, cy
    r = api.move([gx, gy, ztop + 0.06], rotation=R_YAW, seconds=1.3)
    _log(api, tag, "HOVER rp=%.4f res=%.4f eef=%s" % (rp, r, np.round(api.eef(), 4).tolist()))
    r = api.move([gx, gy, ztop + TIP_OFF - BITE], rotation=R_YAW, seconds=0.6)
    e = np.asarray(api.eef(), float)
    _log(api, tag, "DESC res=%.4f eef=%s depth=%.4f" % (r, np.round(e, 4).tolist(), ztop + TIP_OFF - e[2]))
    api.grip(0.0)
    _log(api, tag, "CLOSED", api.gripper())
    # Lift from where the arm actually IS. Re-commanding the planned xy makes
    # the lift also correct the descent's lateral residual, and that shear
    # pulled the wall out of a good bite on debug seed 62 (v6).
    e = np.asarray(api.eef(), float)
    r = api.move([e[0], e[1], CARRY_Z], rotation=R_YAW, seconds=0.8)
    g = api.gripper()
    _log(api, tag, "LIFT res=%.4f eef=%s grip=%s" % (r, np.round(api.eef(), 4).tolist(), g))
    return float(g["width_m"]) > HOLD_GAP, rp


def run(api):
    tz, tgt, plate, sup = perceive(api)
    _log(api, "TABLE", round(tz, 4))
    if tgt is None or plate is None:
        _log(api, "ABORT no target/plate")
        return
    _log(api, "TARGET c=(%.3f,%.3f) ztop=%.3f plate=(%.3f,%.3f,%.3f)"
         % (tgt["cx"], tgt["cy"], tgt["ztop"], plate["cx"], plate["cy"], plate["ztop"]))

    ok, rp = pick(api, tgt, "P0")
    if not ok:
        # Step the arm out of the scene first: perceive() drops any cluster
        # sitting at the eef xy (that is how it rejects the robot), and after
        # a failed bite the eef is 0.048 m from the bowl centre -- inside that
        # rejection radius.
        api.grip(0.08)
        api.move([PARK[0], PARK[1], PARK[2]], rotation=R_YAW, seconds=1.2)
        tz2, tgt2, plate2, sup2 = perceive(api)
        if tgt2 is not None:
            _log(api, "RETRY c=(%.3f,%.3f) ztop=%.3f" % (tgt2["cx"], tgt2["cy"], tgt2["ztop"]))
            ok, rp = pick(api, tgt2, "P1")

    # the bowl centre trails the eef by the pinch radius, in +x
    px, py = plate["cx"] - rp, plate["cy"]
    r = api.move([px, py, CARRY_Z], rotation=R_YAW, seconds=1.3)
    _log(api, "OVER res=%.4f eef=%s grip=%s" % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    pz = plate["ztop"] + 0.055
    r = api.move([px, py, pz], rotation=R_YAW, seconds=0.9)
    _log(api, "LOWER res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(0.08)
    _log(api, "RELEASED", api.gripper())
    r = api.move([px, py, pz + 0.12], rotation=R_YAW, seconds=0.7)
    _log(api, "RETREAT res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    dump(api, "end")
