"""c2clean cell goal_open_top_drawer_put_bowl_pos_k0 - frozen program.

Open the top drawer of the three-drawer cabinet, then put the bowl in it.
No demonstration pack: every constant below was measured on debug seeds 51-65
through the FairApi (cam_high / cam_arm_wrist RGB-D, api.eef, api.gripper).

Mechanism, in one line each:
  * the drawer face is a plane of constant y, so the drawers pull along +y;
  * the handle is a SOLID ledge, so it cannot be straddled top-down with the
    binary 0.079 m jaw - the wrist is turned so the jaws open along z and the
    ledge is clamped from above and below;
  * the bowl is taken by a rim pinch whose radius is the measured mid-wall
    radius, deep enough that the commanded pose stays inside the arm's low-z
    reach.
"""

import base64
import zlib

import numpy as np

PROVENANCE = {
    "TIP": {"source": "debug-seed contact: gripper closed, pressed onto the "
                      "cabinet top at two different xy on seed 51 (v3); both "
                      "stopped at eef_z - cab_ztop = 0.0111 and 0.0124, so the "
                      "fingertip sits 0.0117 m below the eef along the tool "
                      "approach axis. Agreement across two spots is what makes "
                      "it contact rather than a reach limit.",
            "allowed": True},
    "ENGAGE": {"source": "debug-seed gripper receipts: with fingertips only "
                         "0.010 m past the ledge face the jaws closed on air "
                         "(width 0.0015, effort 0.05, seed 51 v5); at 0.016 m "
                         "they close on the ledge (width 0.0174, effort 3.0) "
                         "on every debug seed. The arm drifts ~+0.010 m in y "
                         "while the gripper closes, which is what eats the "
                         "shallower engagement.",
               "allowed": True},
    "PULL": {"source": "debug-seed: commanding +0.20 m of +y travel realises "
                       "0.1598-0.1599 m of drawer travel (re-perceived handle "
                       "yout, all debug seeds); the drawer reaches its stop.",
             "allowed": True},
    "PINCH_DROP": {"source": "debug-seed cam_high radial profile of the bowl: "
                             "at 0.010 m below the rim the mid-wall radius is "
                             "~0.0499 and the wall 0.0020-0.0033 m thick; at "
                             "0.016 m it is ~0.0474 and 0.0028-0.0043 m thick. "
                             "The deeper cut both thickens the target and "
                             "pulls the commanded jaw x back inside the arm's "
                             "low-z reach (v8 receipts: every jaw_x >~ 0.010 "
                             "stops 1.3-1.5 mm short and misses the wall).",
                   "allowed": True},
    "JAW_HALF_OPEN": {"source": "debug-seed api.gripper(): api.grip is binary, "
                                "open width 0.0778-0.0799 m, so each finger "
                                "reaches at most the jaw centre. That is why "
                                "the jaw centre must land inside the bowl wall "
                                "and why the handle cannot be straddled.",
                      "allowed": True},
    "GRIP_OK_W": {"source": "debug-seed gripper receipts: a real rim pinch "
                            "reads width 0.0064-0.0068 with effort 3.0, a miss "
                            "reads 0.0010-0.0033 with effort 0.05.",
                  "allowed": True},
    "BEARING_+x": {"source": "debug-seed collision receipts: the +y bearing "
                             "puts the wrist down on the stove at "
                             "x<-0.16, y in [0.11,0.30] (v6, descent blocked "
                             "52 mm high on both seeds); the -y bearing is "
                             "blocked at the hover height (v11, 0/8); a -60 "
                             "deg wrist is unreachable (v9, 0/8). The +x "
                             "bearing with the jaws along x is the one that "
                             "descends.",
                   "allowed": True},
    "PROP_HEIGHT_GATE": {"source": "debug-seed cam_high: the bowl measures "
                                   "h=0.051, dx=dy=0.110; the plate and the "
                                   "box both measure h~0.019, and the OPENED "
                                   "drawer h=0.224 dx=0.258. The 0.030-0.120 "
                                   "height window plus the 0.070-0.160 "
                                   "footprint window names the bowl alone.",
                         "allowed": True},
    "TABLE_AND_CABINET_BANDS": {"source": "debug-seed cam_high z-histogram: "
                                          "table plane 0.9009-0.9010, cabinet "
                                          "top slab 1.1270, three handle bars "
                                          "at 1.090 / 1.017 / 0.949.",
                                "allowed": True},
    "DEPROJECTION": {"source": "generic pinhole camera mechanics using the "
                               "frame's own .intrinsics and .t_base_cam; "
                               "verified against api.deproject at five probe "
                               "pixels to <0.5 mm (v1b).",
                     "allowed": True},
}

# ---------------------------------------------------------------- plumbing
def _emit(api, tag, arr):
    raw = np.ascontiguousarray(arr).tobytes()
    blob = base64.b64encode(zlib.compress(raw, 6)).decode("ascii")
    api.log("BLOB %s dtype=%s shape=%s nchunk=%d"
            % (tag, arr.dtype.str, tuple(arr.shape), (len(blob) + 1399) // 1400))
    for i in range(0, len(blob), 1400):
        api.log("B %s %d %s" % (tag, i // 1400, blob[i:i + 1400]))


def cloud(frame):
    """Deproject a whole FairFrame to base-frame xyz (verified vs api.deproject
    to <0.5 mm on debug seeds)."""
    d = np.asarray(frame.depth, dtype=np.float64)
    K = np.asarray(frame.intrinsics, dtype=np.float64)
    T = np.asarray(frame.t_base_cam, dtype=np.float64)
    h, w = d.shape
    U, V = np.meshgrid(np.arange(w, dtype=np.float64),
                       np.arange(h, dtype=np.float64))
    xc = (U - K[0, 2]) / K[0, 0] * d
    yc = (V - K[1, 2]) / K[1, 1] * d
    P = np.stack([xc, yc, d], -1)
    return P @ T[:3, :3].T + T[:3, 3]


# ------------------------------------------------------------- perception
def table_z(P):
    m = ((P[:, 0] > -0.30) & (P[:, 0] < 0.35) & (np.abs(P[:, 1]) < 0.45)
         & (P[:, 2] > 0.70) & (P[:, 2] < 1.10))
    z = P[m, 2]
    h, e = np.histogram(z, bins=np.arange(0.70, 1.10, 0.005))
    zc = e[h.argmax()] + 0.0025
    return float(np.median(z[np.abs(z - zc) < 0.006]))


def cabinet(P, zt):
    m = ((P[:, 0] > -0.15) & (P[:, 0] < 0.32) & (P[:, 1] > -0.50)
         & (P[:, 1] < -0.05) & (P[:, 2] > zt + 0.14))
    Q = P[m]
    ztop = float(np.median(Q[:, 2]))
    slab = Q[np.abs(Q[:, 2] - ztop) < 0.012]
    return dict(ztop=ztop, yface=float(slab[:, 1].max()),
                xlo=float(np.percentile(slab[:, 0], 1)),
                xhi=float(np.percentile(slab[:, 0], 99)), n=int(len(slab)))


def handles(P, cab, zt):
    m = ((P[:, 0] > cab["xlo"] - 0.03) & (P[:, 0] < cab["xhi"] + 0.03)
         & (P[:, 1] > cab["yface"] + 0.006) & (P[:, 1] < cab["yface"] + 0.10)
         & (P[:, 2] > zt + 0.02) & (P[:, 2] < cab["ztop"] - 0.015))
    Q = P[m]
    if len(Q) < 30:
        return []
    zs = np.sort(Q[:, 2])
    idx = np.concatenate([[0], np.where(np.diff(zs) > 0.010)[0] + 1, [len(zs)]])
    out = []
    for a, b in zip(idx[:-1], idx[1:]):
        if b - a < 25:
            continue
        s = Q[(Q[:, 2] >= zs[a]) & (Q[:, 2] <= zs[b - 1])]
        out.append(dict(n=int(len(s)), zlo=float(s[:, 2].min()),
                        zhi=float(s[:, 2].max()),
                        zc=float(0.5 * (s[:, 2].min() + s[:, 2].max())),
                        yout=float(s[:, 1].max()),
                        xlo=float(s[:, 0].min()), xhi=float(s[:, 0].max()),
                        xc=float(0.5 * (s[:, 0].min() + s[:, 0].max()))))
    out.sort(key=lambda h: -h["zc"])
    return out


def _components(mask):
    """4/8-connected labelling on a small bool grid, no scipy."""
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    H, W = mask.shape
    for i in range(H):
        for j in range(W):
            if not mask[i, j] or lab[i, j]:
                continue
            cur += 1
            stack = [(i, j)]
            lab[i, j] = cur
            while stack:
                a, b = stack.pop()
                for da in (-1, 0, 1):
                    for db in (-1, 0, 1):
                        p, q = a + da, b + db
                        if 0 <= p < H and 0 <= q < W and mask[p, q] and not lab[p, q]:
                            lab[p, q] = cur
                            stack.append((p, q))
    return lab, cur


def props(P, zt, cab):
    """Free-standing props on the table, in front of the cabinet face."""
    m = ((P[:, 1] > cab["yface"] + 0.06) & (P[:, 2] > zt + 0.012)
         & (P[:, 2] < zt + 0.30) & (P[:, 0] > -0.16) & (P[:, 0] < 0.35)
         & (P[:, 1] < 0.45))
    Q = P[m]
    if len(Q) < 50:
        return []
    gx = ((Q[:, 0] + 0.16) / 0.005).astype(int)
    gy = ((Q[:, 1] + 0.50) / 0.005).astype(int)
    G = np.zeros((int(gx.max()) + 2, int(gy.max()) + 2), bool)
    G[gx, gy] = True
    lab, n = _components(G)
    ids = lab[gx, gy]
    out = []
    for k in range(1, n + 1):
        s = Q[ids == k]
        if len(s) < 150:
            continue
        out.append(dict(n=int(len(s)),
                        dx=float(s[:, 0].max() - s[:, 0].min()),
                        dy=float(s[:, 1].max() - s[:, 1].min()),
                        h=float(s[:, 2].max() - zt), ztop=float(s[:, 2].max()),
                        xc=float(0.5 * (s[:, 0].min() + s[:, 0].max())),
                        yc=float(0.5 * (s[:, 1].min() + s[:, 1].max()))))
    return out


# ------------------------------------------------------------------- main
TIP = 0.0117          # eef -> fingertip along the tool approach axis (v3)
ENGAGE = 0.016        # ledge overlap that clamps (v5 s57; v6 2/2; v7 15/15)
PULL = 0.20           # +y drawer travel commanded (0.1598 m realised)
PINCH_DROP = 0.016    # fingertip depth below the bowl rim.  6 mm deeper than
                      # v7/v8: the mid-wall radius there is ~0.047 instead of
                      # ~0.0499, which pulls the commanded jaw x back inside
                      # the arm's low-z reach (it stops 1.3-1.5 mm short of
                      # any jaw_x >~ 0.010, which is what lost seeds 56/59),
                      # and the wall is slightly thicker (2.7 vs 2.0 mm).
GRIP_OK_W = 0.0035    # closed width that means material is between the pads
RETRY_OUT = 0.0       # the retry changes bearing, not aim: the wall
                      # midline is already the right radius (v8 receipts)
# Grasp bearing, measured out from the bowl centre in the xy plane.
#   +x (phi=0, v7/v8) needs eef_x = bowl_x + 0.050; that failed on every debug
#   seed whose bowl sat at x >= -0.039, where the arm stops 1.3-1.5 mm short
#   at pinch height and the jaw lands inboard of the 2 mm wall.
#   +y (v6) puts the forearm down on the stove at x<-0.16, y in [0.11,0.30].
#   -60 deg keeps eef_x <= -0.013 and eef_y <= 0.10 on all fifteen debug seeds.
# Attempt 0 keeps v7's proven +x bearing (13/15 on the full debug band).
# Attempt 1 falls back to the -y bearing with the RESET wrist: it puts the eef
# at (bowl_x, bowl_y-0.05), so |eef_x| stays small (no reach cap) and eef_y
# ~0.08 stays out of the stove's y band [0.11,0.30] that wrecked the +y
# bearing in v6.

R_SIDE = np.array([[1.0, 0.0, 0.0],
                   [0.0, 0.0, -1.0],
                   [0.0, 1.0, 0.0]])


def rot_jaw(phi):
    """Wrist pointing straight down with the jaws opening along
    u = (cos phi, sin phi, 0)."""
    c, s = float(np.cos(phi)), float(np.sin(phi))
    return np.array([[-s, c, 0.0],
                     [c, s, 0.0],
                     [0.0, 0.0, -1.0]])


R_XJAW = rot_jaw(0.0)                      # jaws along +x
U_XJAW = np.array([1.0, 0.0, 0.0])
R_YJAW = rot_jaw(-np.pi / 2.0)             # jaws along y == the reset wrist
U_YJAW = np.array([0.0, -1.0, 0.0])
BEARINGS = ((U_XJAW, R_XJAW, "+x"), (U_YJAW, R_YJAW, "-y"))


def _mv(api, n, xyz, secs, rot=None, tag="", ncorr=0, csecs=0.4):
    xyz = np.asarray(xyz, float)
    api.move(xyz.tolist(), rotation=rot, seconds=secs)
    e = np.asarray(api.eef())
    api.log("MV%02d %s tgt=%s eef=%s err=%s g=%s"
            % (n, tag, xyz.round(4).tolist(), e.round(4).tolist(),
               (xyz - e).round(4).tolist(), api.gripper()))
    for k in range(ncorr):
        api.move((xyz + (xyz - e)).tolist(), rotation=rot, seconds=csecs)
        e = np.asarray(api.eef())
        api.log("MV%02dc%d %s eef=%s err=%s"
                % (n, k, tag, e.round(4).tolist(), (xyz - e).round(4).tolist()))
    return e


def pick_bowl(props_list, api):
    """The bowl is the only deep round vessel: h ~ 0.051 where every other free
    prop measured <0.02, and dx ~ dy ~ 0.11.  The shape gate also keeps the
    OPENED drawer (h 0.22, dx 0.26) out of the re-perception path."""
    c = [p for p in props_list
         if 0.030 < p["h"] < 0.120 and 0.070 < p["dx"] < 0.160
         and 0.070 < p["dy"] < 0.160]
    if not c:
        api.log("BOWL none of %d props passed the shape gate" % len(props_list))
        return None
    return max(c, key=lambda p: p["h"])


def wall_radius(P, bowl, z_tip, api):
    """Mid-wall radius at the pinch height.  From cam_high (+x of the scene)
    the far half of the bowl shows its INNER surface and the near half its
    OUTER surface; the jaw centre must land between the two, because each
    finger can only travel as far as the jaw centre."""
    xc, yc = bowl["xc"], bowl["yc"]
    m = ((np.abs(P[:, 0] - xc) < 0.09) & (np.abs(P[:, 1] - yc) < 0.09)
         & (np.abs(P[:, 2] - z_tip) < 0.0018))
    Q = P[m]
    if len(Q) < 12:
        api.log("WALL fallback (n=%d)" % len(Q))
        return 0.25 * (bowl["dx"] + bowl["dy"]) - 0.005
    r = np.hypot(Q[:, 0] - xc, Q[:, 1] - yc)
    near, far = Q[:, 0] > xc + 0.015, Q[:, 0] < xc - 0.015
    if near.sum() < 3 or far.sum() < 5:
        api.log("WALL one-sided (near=%d far=%d)" % (near.sum(), far.sum()))
        return float(np.median(r)) + 0.001
    ri, ro = float(np.median(r[far])), float(np.median(r[near]))
    api.log("WALL r_in=%.4f r_out=%.4f mid=%.4f thk=%.4f"
            % (ri, ro, 0.5 * (ri + ro), ro - ri))
    return 0.5 * (ri + ro)


def cavity(P, cab, h, api):
    m = ((P[:, 0] > cab["xlo"] - 0.04) & (P[:, 0] < cab["xhi"] + 0.04)
         & (P[:, 1] > cab["yface"] - 0.02) & (P[:, 1] < cab["yface"] + 0.30)
         & (P[:, 2] > h["zlo"] - 0.060) & (P[:, 2] < h["zlo"] - 0.008))
    Q = P[m]
    if len(Q) < 120:
        api.log("CAVITY none (n=%d)" % len(Q))
        return None
    hz, e = np.histogram(Q[:, 2], bins=np.arange(h["zlo"] - 0.060,
                                                 h["zlo"] - 0.006, 0.004))
    s = Q[np.abs(Q[:, 2] - (e[hz.argmax()] + 0.002)) < 0.006]
    out = dict(z=float(np.median(s[:, 2])), n=int(len(s)))
    api.log("CAVITY %s" % out)
    return out


def run(api):
    api.log("== v12: side clamp opens drawer; deep +x wall-midline rim pinch ==")
    f = api.capture("cam_high")
    P = cloud(f).reshape(-1, 3)
    zt = table_z(P)
    cab = cabinet(P, zt)
    hs = handles(P, cab, zt)
    pl = props(P, zt, cab)
    api.log("TABLE z=%.4f CAB %s" % (zt, cab))
    for i, h in enumerate(hs):
        api.log("HANDLE%d %s" % (i, h))
    for p in pl:
        api.log("PROP %s" % p)
    if not hs:
        api.log("NO HANDLE - abort")
        return
    bowl = pick_bowl(pl, api)
    api.log("BOWL %s" % bowl)
    h = hs[0]
    hx, yout, zc = h["xc"], h["yout"], h["zc"]

    # ---------------- stage 1: open the top drawer --------------------
    _mv(api, 1, [hx, yout + 0.21, cab["ztop"] + 0.08], 2.0, R_SIDE, "reorient")
    _mv(api, 2, [hx, yout + 0.21, zc], 1.0, R_SIDE, "at-height")
    _mv(api, 3, [hx, yout - ENGAGE, zc], 1.4, R_SIDE, "engage", 1)
    api.grip(0.0)
    api.settle(0.25)
    api.log("CLAMP g=%s ledge_thk=%.4f" % (api.gripper(), h["zhi"] - h["zlo"]))
    e = _mv(api, 4, [hx, yout + PULL, zc], 2.2, R_SIDE, "pull")
    api.grip(0.08)
    api.settle(0.15)
    api.log("OPEN_DONE eef=%s" % e.round(4).tolist())
    if bowl is None:
        return

    # ---------------- stage 2: rim-pinch the bowl ---------------------
    _mv(api, 5, [hx, yout + 0.26, cab["ztop"] + 0.12], 1.6, R_XJAW, "reorient2")
    held = False
    n = 6
    pr = 0.050
    U, R_P, bname = BEARINGS[0]
    for attempt in (0,):
        U, R_P, bname = BEARINGS[attempt]
        z_tip = bowl["ztop"] - PINCH_DROP
        pr = wall_radius(P, bowl, z_tip, api) + attempt * RETRY_OUT
        ctr = np.array([bowl["xc"], bowl["yc"], 0.0])
        jaw = ctr + pr * U
        api.log("PINCH try=%d bearing=%s pr=%.4f jaw=(%.4f,%.4f) z_tip=%.4f"
                % (attempt, bname, pr, jaw[0], jaw[1], z_tip))
        _mv(api, n, [jaw[0], jaw[1], zt + 0.19], 2.0, R_P, "over-bowl")
        n += 1
        _mv(api, n, [jaw[0], jaw[1], z_tip + TIP], 1.4, R_P, "to-rim", 1)
        n += 1
        api.grip(0.0)
        api.settle(0.35)
        g = api.gripper()
        held = (g["effort"] > 1.0) and (g["width_m"] > GRIP_OK_W)
        api.log("PINCH_CLOSED try=%d g=%s held=%s eef=%s"
                % (attempt, g, held, np.round(api.eef(), 4).tolist()))
        break
        api.grip(0.08)
        api.settle(0.15)
        _mv(api, n, [jaw[0], jaw[1], zt + 0.26], 1.2, R_P, "abort-lift")
        n += 1
        _mv(api, n, [bowl["xc"], bowl["yc"] - 0.22, zt + 0.30], 1.8,
            BEARINGS[1][1], "park")
        n += 1
        P = cloud(api.capture("cam_high")).reshape(-1, 3)
        nb = pick_bowl(props(P, zt, cab), api)
        if nb is not None:
            api.log("REPERCEIVE bowl %s -> %s"
                    % (np.round([bowl["xc"], bowl["yc"]], 4).tolist(),
                       np.round([nb["xc"], nb["yc"]], 4).tolist()))
            bowl = nb

    lift = np.array([bowl["xc"], bowl["yc"], 0.0]) + pr * U
    _mv(api, n, [lift[0], lift[1], zt + 0.27], 1.4, R_P, "lift")
    n += 1
    api.log("LIFT g=%s held=%s" % (api.gripper(), held))

    # ---------------- stage 3: place it in the drawer -----------------
    P2 = cloud(api.capture("cam_high")).reshape(-1, 3)
    cav = cavity(P2, cab, h, api)
    hs2 = handles(P2, cabinet(P2, zt), zt)
    yout2 = hs2[0]["yout"] if hs2 else yout + 0.15
    api.log("POST_OPEN yout %.4f -> %.4f (travel %.4f)" % (yout, yout2, yout2 - yout))

    drop_y = float(min(max(0.5 * (cab["yface"] + yout2), cab["yface"] + 0.070),
                       yout2 - 0.065))
    floor = cav["z"] if cav else (h["zlo"] - 0.020)
    z_rel = floor + bowl["h"] + 0.018 + TIP
    tgt = np.array([hx, drop_y, 0.0]) + pr * U
    api.log("DROP bowl->(%.4f,%.4f) floor=%.4f z_rel=%.4f eef=(%.4f,%.4f)"
            % (hx, drop_y, floor, z_rel, tgt[0], tgt[1]))

    _mv(api, n, [tgt[0], tgt[1], cab["ztop"] + 0.13], 2.2, R_P, "over-drawer")
    n += 1
    _mv(api, n, [tgt[0], tgt[1], z_rel], 1.2, R_P, "lower")
    n += 1
    api.grip(0.08)
    api.settle(0.3)
    api.log("RELEASED g=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))
    _mv(api, n, [tgt[0], tgt[1], cab["ztop"] + 0.19], 1.4, R_P, "away")

    for p in props(cloud(api.capture("cam_high")).reshape(-1, 3), zt, cab):
        api.log("FINAL_PROP %s" % p)
