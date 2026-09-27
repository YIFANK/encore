"""v7 -- v6 with a tolerance-limited bite depth and a re-perceived hold check.

v6 receipt (4/8, debug 51..65): every hold closed 11.6-20.3 mm below the rim
crest (51, 59, 61, 63) and every loss closed 17.7-33.0 mm below it (53, 55,
57, 65 and all the retry arcs). On a tilted bowl the contact-limited descent
runs 31-33 mm past the crest because the outer finger falls all the way to
the table beside the bowl, so the jaws close on the shell's flat under-belly
and it slides out. Commanding rim-20 mm instead stops the descent by
POS_TOL (~8 mm short, i.e. ~12 mm under the crest) whatever the tilt.

The gap alone is not a hold test either: a shell bite reads 0.0045 m, under
the harness's 5 mm "effort" flag, yet it is a real grip. v7 verifies the hold
by re-perception instead -- once the arm has carried the bowl over the stove,
the bowl's old footprint must be bare.

Receipt chain that produced this:
  v2 (3/4): a 12 mm inset pinch works on a tilted bowl, and ep51 showed the
       -y arc fouls the 0.34 m cabinet.
  v3 (2/8): clearance-picked arc fixed the collision but the 12 mm inset
       pinch ratchets out: gap 0.0083 -> 0.0048 over the lift.
  v4probe: the descent is CONTACT-limited, not tolerance-limited (commanding
       rim-35 mm lands at rim-13 mm; the fingertips sit ~38 mm under the EEF,
       i.e. on the table), and the flat bowl never leaves the table.
  v5probe (radial sweep, seeds 51/61/53): the bite is stable only when the
       EEF is aimed AT the rim line or just outside it --
         inset -8 mm: 0.00819->0.00813, 0.00839->0.00832, 0.00825->0.00680
         inset  0 mm: 0.00909->0.00902, 0.00727->0.00691, 0.00889->0.00860
         inset +12 mm: 0.00815->0.00807, 0.00838->0.00386, 0.00640->0.00631
       over a 45 mm lift. Inset 0 it is.

Because the grasp closes with the bowl still standing on the table, the bowl's
base sits exactly (z_close - table) below the EEF: that is the hang, and it
needs no measurement of the held object.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "WORKSPACE_CROP": {"source": "debug 51/53/55 cam_high depth: table and props "
                                 "lie inside x[-0.45,0.45] y[-0.50,0.50]",
                       "allowed": True},
    "TABLE_MODAL_Z": {"source": "debug 51/61/53 v5probe: the modal 5 mm depth bin "
                                "reads 0.9010 with or without the arm in view, "
                                "where the median jumps to 0.926", "allowed": True},
    "STOVE_BAND": {"source": "debug 51/53/55: stove slab top plane 24.4 mm above "
                             "the table (zmed 0.9258, sigma 2.3 mm), 0.19x0.19 m",
                   "allowed": True},
    "BOWL_HOLLOW_TEST": {"source": "debug 51/53/55: bowl interior fill 0/96 cells "
                                   "vs 13/13 (bottle), 34/39 (kettle)",
                         "allowed": True},
    "BOWL_SIZE_RANGE": {"source": "debug 51/53/55: bowl footprint 0.110 x 0.111 m",
                        "allowed": True},
    "GRASP_INSET": {"source": "v5probe radial sweep on debug 51/61/53: gap after a "
                              "45 mm lift keeps 99% of the closed gap at inset 0, "
                              "and drops to 46% at inset +12 mm (ep61)",
                    "allowed": True},
    "DESCEND_TARGET": {"source": "v6 receipt: holds closed 11.6-20.3 mm below the "
                                 "rim crest, losses 17.7-33.0 mm below it; "
                                 "commanding rim-20 mm lands ~12 mm below it "
                                 "(POS_TOL stops the move ~8 mm high)",
                       "allowed": True},
    "FOOTPRINT_CLEAR": {"source": "v1/v6 debug scans: the bowl's own footprint "
                                  "holds 800-1500 points in the table+[20,90] mm "
                                  "band; once carried away it must fall below a "
                                  "third of that", "allowed": True},
    "CARRY_Z": {"source": "pack.json demo0 ee_path6 carry apex z 1.0526",
                "allowed": True},
    "RELEASE_CLEARANCE": {"source": "debug 51/53/55: bowl base released 8 mm over "
                                    "the stove's own top (burner ring included)",
                          "allowed": True},
    "CLEARANCE_RADIUS": {"source": "debug ep51 v2: the hand fouled the cabinet "
                                   "whose face was 42 mm from the arc point; "
                                   "90 mm keeps it clear", "allowed": True},
    "HOLD_GAP": {"source": "debug v2/v3: a closed-on-nothing grasp reads 0.001 m, "
                           "a held bowl 0.0064-0.0091 m", "allowed": True},
    "PARK_POSE": {"source": "the episode's own start pose (api.eef() at t0), used "
                            "to clear the camera before re-perceiving",
                  "allowed": True},
}

XLO, XHI, YLO, YHI = -0.45, 0.45, -0.50, 0.50
STOVE_BAND = (0.018, 0.040)
BOWL_THR = 0.040
GRASP_INSET = 0.000
DESCEND_EXTRA = 0.020
CARRY_Z = 1.05
RELEASE_CLEAR = 0.008
CLEAR_R = 0.090
CLEAR_H = 0.030
MIN_GAP = 0.0025        # below this the jaws closed on air
FP_R = 0.045            # footprint radius for the hold check
FP_BAND = (0.020, 0.090)
FP_FRAC = 0.33

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
R_YAW = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


# --------------------------------------------------------------------- utils
def _label(mask):
    h, w = mask.shape
    lab = np.where(mask, np.arange(h * w).reshape(h, w) + 1, 0)
    for _ in range(4000):
        m = lab.copy()
        m[:-1, :] = np.maximum(m[:-1, :], lab[1:, :] * (mask[:-1, :] & mask[1:, :]))
        m[1:, :] = np.maximum(m[1:, :], lab[:-1, :] * (mask[1:, :] & mask[:-1, :]))
        m[:, :-1] = np.maximum(m[:, :-1], lab[:, 1:] * (mask[:, :-1] & mask[:, 1:]))
        m[:, 1:] = np.maximum(m[:, 1:], lab[:, :-1] * (mask[:, 1:] & mask[:, :-1]))
        m = m * mask
        if np.array_equal(m, lab):
            break
        lab = m
    return lab


def _label_full(mask):
    small = mask[::2, ::2]
    lab = _label(small)
    big = np.repeat(np.repeat(lab, 2, 0), 2, 1)[:mask.shape[0], :mask.shape[1]]
    return big * mask


def _cloud(frame):
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    z = np.where(np.isfinite(d) & (d > 0), d, np.nan)
    pts = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z, np.ones_like(z)], -1)
    return (pts @ T.T)[..., :3]


def _blob(api, tag, arr):
    b = base64.b64encode(zlib.compress(np.ascontiguousarray(arr).tobytes(), 6)).decode()
    api.log(f"BLOB {tag} dtype={arr.dtype} shape={list(arr.shape)} n={len(b)}")
    for i in range(0, len(b), 1800):
        api.log(f"B {tag} {i // 1800} {b[i:i + 1800]}")


def _mode_z(v):
    v = v[np.isfinite(v)]
    h, e = np.histogram(v, bins=np.arange(0.80, 1.45, 0.005))
    i = int(np.argmax(h))
    sel = v[(v >= e[i]) & (v < e[i + 1])]
    return float(np.median(sel))


def _hollow(px, py, pz, res=0.005):
    dx, dy = px.max() - px.min(), py.max() - py.min()
    nx, ny = int(dx / res) + 1, int(dy / res) + 1
    hm = np.zeros((nx, ny))
    np.maximum.at(hm, (((px - px.min()) / res).astype(int),
                       ((py - py.min()) / res).astype(int)), pz)
    occ = hm > 0
    ii, jj = np.nonzero(occ)
    ci, cj = ii.mean(), jj.mean()
    rmax = np.hypot(ii - ci, jj - cj).max() * res
    r = np.hypot(np.arange(nx)[:, None] - ci, np.arange(ny)[None, :] - cj) * res
    inner = r < 0.45 * rmax
    return 1.0 - (occ & inner).sum() / max(inner.sum(), 1), rmax, dx, dy


# ---------------------------------------------------------------- perception
def scene(api, tag, want_stove=True):
    f = api.capture("cam_high")
    P = _cloud(f)
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    ok = np.isfinite(z) & (x > XLO) & (x < XHI) & (y > YLO) & (y < YHI)
    tab = _mode_z(z[ok])

    stove = None
    if want_stove:
        band = ok & (z > tab + STOVE_BAND[0]) & (z < tab + STOVE_BAND[1])
        L = _label_full(band)
        ids, cnt = np.unique(L[L > 0], return_counts=True)
        for i in np.argsort(-cnt):
            if cnt[i] < 2000:
                break
            s = L == ids[i]
            bx, by, bz = x[s], y[s], z[s]
            dx, dy = bx.max() - bx.min(), by.max() - by.min()
            if 0.13 < dx < 0.30 and 0.13 < dy < 0.30:
                stove = dict(cx=float((bx.min() + bx.max()) / 2),
                             cy=float((by.min() + by.max()) / 2),
                             top=float(np.percentile(bz, 90)),
                             dx=float(dx), dy=float(dy), n=int(cnt[i]))
                break

    hi = ok & (z > tab + BOWL_THR)
    L = _label_full(hi)
    ids, cnt = np.unique(L[L > 0], return_counts=True)
    bowl, cands = None, []
    for i in np.argsort(-cnt):
        if cnt[i] < 250:
            break
        s = L == ids[i]
        bx, by, bz = x[s], y[s], z[s]
        if bz.max() > tab + 0.13:
            continue
        hol, rmax, dx, dy = _hollow(bx, by, bz)
        cands.append((round(float(hol), 3), round(float(dx), 3), round(float(dy), 3),
                      round(float(bz.max()), 3), round(float(bx.mean()), 3),
                      round(float(by.mean()), 3), int(cnt[i])))
        if hol > 0.6 and 0.07 < max(dx, dy) < 0.17 and bowl is None:
            bowl = dict(sel=s, cx=float((bx.min() + bx.max()) / 2),
                        cy=float((by.min() + by.max()) / 2),
                        top=float(bz.max()), dx=float(dx), dy=float(dy),
                        rmax=float(rmax), n=int(cnt[i]))
    api.log(f"[{tag}] table={tab:.4f} stove={stove}")
    api.log(f"[{tag}] bowl_cands={cands}")
    return dict(P=P, tab=tab, stove=stove, bowl=bowl, ok=ok)


def arc_plan(api, sc):
    x, y, z = sc["P"][..., 0], sc["P"][..., 1], sc["P"][..., 2]
    b = sc["bowl"]
    s = b["sel"]
    eef = api.eef()
    arm = (np.abs(x - eef[0]) < 0.12) & (np.abs(y - eef[1]) < 0.14)
    plans = []
    for name, d, yaw in (("+y", (0.0, 1.0), False), ("-y", (0.0, -1.0), False),
                         ("+x", (1.0, 0.0), True), ("-x", (-1.0, 0.0), True)):
        if d[1]:
            strip = s & (np.abs(x - b["cx"]) < 0.014) & ((y - b["cy"]) * d[1] > 0)
        else:
            strip = s & (np.abs(y - b["cy"]) < 0.014) & ((x - b["cx"]) * d[0] > 0)
        if strip.sum() < 5:
            continue
        px, py, pz = x[strip], y[strip], z[strip]
        k = int(np.argmax(pz))
        rim = (float(px[k]), float(py[k]), float(pz[k]))
        gz = rim[2] - 0.020
        ax = b["cx"] + d[0] * (b["rmax"] + 0.020)
        ay = b["cy"] + d[1] * (b["rmax"] + 0.020)
        obst = sc["ok"] & (~s) & (~arm) & (z > gz + CLEAR_H) & \
            (np.hypot(x - ax, y - ay) < CLEAR_R)
        plans.append(dict(name=name, d=d, yaw=yaw, rim=rim, n_obst=int(obst.sum()),
                          htall=float(z[obst].max()) if obst.any() else 0.0))
    plans.sort(key=lambda p: (p["n_obst"], 0 if p["name"] == "+y" else
                              (1 if not p["yaw"] else 2)))
    for p in plans:
        api.log(f"arc {p['name']} rim={np.round(p['rim'], 4).tolist()} "
                f"n_obst={p['n_obst']} htall={p['htall']:.3f}")
    return plans


# --------------------------------------------------------------------- grasp
def attempt(api, sc, plan):
    b, d, rim = sc["bowl"], plan["d"], plan["rim"]
    gx = rim[0] - d[0] * GRASP_INSET
    gy = rim[1] - d[1] * GRASP_INSET
    gz = rim[2] - DESCEND_EXTRA
    api.log(f"try arc={plan['name']} grasp=({gx:.4f},{gy:.4f},{gz:.4f}) "
            f"rim_r={np.hypot(rim[0] - b['cx'], rim[1] - b['cy']):.4f}")

    api.grip(0.08)
    hov = max(rim[2] + 0.075, CARRY_Z - 0.02)
    r = api.move([gx, gy, hov], rotation=(R_YAW if plan["yaw"] else R_DOWN), seconds=3.0)
    api.log(f"hover res={r:.4f} eef={np.round(api.eef(), 4).tolist()}")
    if r > 0.03:
        api.log("hover stalled -> abandon this arc")
        return None
    api.move([gx, gy, gz], seconds=1.5)
    api.grip(0.0)
    api.settle(0.2)
    e1 = api.eef()
    g0 = api.gripper()["width_m"]
    api.log(f"closed eef={np.round(e1, 4).tolist()} gap={g0:.5f} "
            f"below_rim={rim[2] - e1[2]:.4f}")
    if g0 <= MIN_GAP:
        api.log("closed on air")
        return None
    r = api.move([e1[0], e1[1], CARRY_Z], seconds=1.5)
    g1 = api.gripper()["width_m"]
    api.log(f"lift res={r:.4f} eef={np.round(api.eef(), 4).tolist()} gap={g1:.5f}")
    if g1 <= MIN_GAP:
        api.log(f"grip emptied during the lift ({g0:.5f} -> {g1:.5f})")
        return None
    return dict(e_close=e1, gap0=g0, gap=g1, bowl_cx=b["cx"], bowl_cy=b["cy"],
                tab=sc["tab"])


def footprint_pts(sc, cx, cy):
    x, y, z = sc["P"][..., 0], sc["P"][..., 1], sc["P"][..., 2]
    m = sc["ok"] & (np.hypot(x - cx, y - cy) < FP_R) & \
        (z > sc["tab"] + FP_BAND[0]) & (z < sc["tab"] + FP_BAND[1])
    return int(m.sum())


# ---------------------------------------------------------------------- main
def run(api):
    api.log(f"instruction={api.instruction()!r}")
    park = api.eef().copy()
    sc = scene(api, "pre")
    _blob(api, "z0", np.where(sc["ok"], sc["P"][..., 2], np.nan).astype(np.float16))
    if sc["bowl"] is None or sc["stove"] is None:
        api.log("ABORT: bowl or stove not found")
        return
    st = sc["stove"]
    api.log(f"bowl c=({sc['bowl']['cx']:.4f},{sc['bowl']['cy']:.4f}) "
            f"top={sc['bowl']['top']:.4f} rmax={sc['bowl']['rmax']:.4f}")

    fp0 = footprint_pts(sc, sc["bowl"]["cx"], sc["bowl"]["cy"])
    api.log(f"footprint baseline={fp0}")

    grip, tx, ty = None, None, None
    plans = arc_plan(api, sc)
    for i in range(3):
        if i >= len(plans):
            break
        grip = attempt(api, sc, plans[i])
        if grip is not None:
            # carry over the stove first: from there the old footprint is in
            # clear view, and an empty footprint is the hold receipt
            e1 = grip["e_close"]
            tx = st["cx"] + (e1[0] - grip["bowl_cx"])
            ty = st["cy"] + (e1[1] - grip["bowl_cy"])
            r = api.move([tx, ty, CARRY_Z], seconds=2.0)
            api.log(f"over res={r:.4f} eef={np.round(api.eef(), 4).tolist()} "
                    f"gap={api.gripper()['width_m']:.5f}")
            scc = scene(api, f"check{i}", want_stove=False)
            fp = footprint_pts(scc, grip["bowl_cx"], grip["bowl_cy"])
            api.log(f"footprint now={fp} (baseline {fp0}) -> "
                    f"{'HELD' if fp < FP_FRAC * fp0 else 'STILL ON THE TABLE'}")
            if fp < FP_FRAC * fp0:
                break
            grip = None
        api.grip(0.08)
        e = api.eef()
        api.move([e[0], e[1], CARRY_Z], seconds=1.0)
        api.move([park[0], park[1], CARRY_Z + 0.05], seconds=2.0)
        sc2 = scene(api, f"retry{i}", want_stove=False)
        if sc2["bowl"] is not None:
            sc = dict(sc2, stove=st)
            plans = arc_plan(api, sc)
            fp0 = footprint_pts(sc, sc["bowl"]["cx"], sc["bowl"]["cy"])
        else:
            api.log("re-perception lost the bowl; reusing the previous model")
    if grip is None:
        api.log("ABORT: never held the bowl")
        return

    e1 = grip["e_close"]
    hang = float(e1[2] - grip["tab"])          # bowl base was on the table
    tz = st["top"] + hang + RELEASE_CLEAR
    api.log(f"place target=({tx:.4f},{ty:.4f},{tz:.4f}) hang={hang:.4f} "
            f"stove=({st['cx']:.3f},{st['cy']:.3f},top={st['top']:.4f})")
    r = api.move([tx, ty, tz], seconds=1.5)
    api.log(f"lower res={r:.4f} eef={np.round(api.eef(), 4).tolist()} gap={api.gripper()['width_m']:.5f}")
    api.grip(0.08)
    api.settle(0.4)
    api.log(f"released eef={np.round(api.eef(), 4).tolist()}")
    r = api.move([tx, ty, CARRY_Z], seconds=1.5)
    api.move([park[0], park[1], CARRY_Z + 0.05], seconds=2.0)

    sc3 = scene(api, "post")
    if sc3["bowl"] is not None:
        b3 = sc3["bowl"]
        api.log(f"post bowl c=({b3['cx']:.4f},{b3['cy']:.4f}) top={b3['top']:.4f} "
                f"stove=({st['cx']:.3f},{st['cy']:.3f})")
    _blob(api, "z2", np.where(sc3["ok"], sc3["P"][..., 2], np.nan).astype(np.float16))
    api.log("done")
