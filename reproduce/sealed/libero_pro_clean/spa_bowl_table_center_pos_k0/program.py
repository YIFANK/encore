"""v8 -- frozen candidate: v7 with the dead seating branch removed.

Mechanism: the target bowl is 0.114 m across (debug 51-65), wider than the
0.079 m the jaws open, so it cannot be straddled. The jaws separate along world
y (v2 wrist-depth receipt), so the gripper is parked offset +y from the bowl
centre and closed across the rim wall.

v4 receipt (debug 51..65, 7 completed): aiming from the lift-time hang offset
put the bowl on the plate every time but consistently off-centre by
(-0.033, +0.023) m, std (0.004, 0.009) -- the cam_high view of a held bowl is
occluded on the gripper side, so the measured hang offset is ~0.02 m too long.
v5 keeps the lift measurement, adds that measured landing bias back as a fixed
correction, and seats the bowl on the plate by descending until the eef stops
tracking instead of dropping it from a computed height.

v4 also lost seed 59 to plate detection: the plate merged with the bowl beside
it into one blob. v5 finds the plate by height band first, then clusters, which
separates them on all 15 debug seeds.

v7 receipt: 15/15 on the full debug split (sel_..._v7). v8 is v7 with the
unused contact-seating branch deleted and PROVENANCE completed; the executed
path is byte-for-byte the same sequence.

v6b receipt: 14/15 on the full debug split (sel_..._v6b). The one loss, seed 62,
came from a single bad held-bowl reading: its lift blob was n=83/top=0.176
against n=53-67/top=0.186-0.189 everywhere else, which threw the hang offset to
(-0.0085,-0.0574) against a median (0.0101,-0.0606) and moved the aim 0.019 in
x. Displacing the aim 0.015 by hand costs 3-5 of 8 seeds, so the placement basin
is only about +-0.012 wide and a 0.019 slip is fatal. v7 keeps the measurement
but rejects one further than HANG_GATE from the debug median and falls back to
the median, which is what 14 of 15 seeds measured anyway.

v5 receipt: 5/8 with the seating descent and 5/8 without it (same three seeds
53/55/63 lost either way, so seating is neutral and the loss is lateral). Those
three landed (-0.010,+0.018) from the plate centre. Cause found in the logs:
goto read the eef while the arm was still travelling, so its "converged" exit
left an over-corrected command standing and the following settle carried the
arm up to 0.012 past the aim. v6 settles before reading, and pins the descent
to the command goto converged on rather than to the drifted eef.
"""
import zlib, base64
import numpy as np

PROVENANCE = {
    "WS_BOX": {"source": "debug 51-65 cam_high clouds: table plane spans x[-0.64,0.27] "
                         "y[-0.56,0.55]; box padded to cover it", "allowed": True},
    "CELL": {"source": "generic: 5 mm grid, ~2x the 2.1 mm/px ground sampling measured "
                       "from the v1 deprojection probes", "allowed": True},
    "ABOVE_TABLE": {"source": "debug 51 z-histogram: table plane occupies 0.895-0.906",
                    "allowed": True},
    "ARM_CUT": {"source": "debug 51-65: arm tops 0.47 m and cabinet 0.227 m above table",
                "allowed": True},
    "TIP_DZ": {"source": "v2 debug 51/53/55/57: closed gripper stalls at eef_z 0.9087 "
                         "while the table is at 0.9013 -> tips sit 0.0074 below the eef",
               "allowed": True},
    "BOWL_R": {"source": "v2/v1b debug 51-65 cross-sections of the target bowl: outer rim "
                         "radius 0.057, interior surface reaches 25 mm at radius 0.036",
               "allowed": True},
    "PINCH_R": {"source": "midpoint of the 0.036/0.050 wall at the 0.025 grasp height "
                          "measured in the debug 51-65 cross-sections", "allowed": True},
    "GRASP_H": {"source": "debug 51-65: rim top 0.051, interior floor 0.007; 0.025 is "
                          "below the rim and clear of the floor", "allowed": True},
    "SAFE_H": {"source": "debug 51-65: tallest thing on the approach path is the target "
                         "bowl rim at 0.051; 0.135 clears it and the 0.107 cabinet edge "
                         "is not on the path", "allowed": True},
    "LIFT_H": {"source": "debug 51-65: the carried bowl hangs ~0.039 below the tips and "
                         "the tallest obstacle on the carry is bowl E at 0.051, so 0.170 "
                         "keeps 0.08 of clearance", "allowed": True},
    "CHUNK": {"source": "v1 receipt: api.log truncates a message at 2000 chars",
              "allowed": True},
    "HANG_RADIUS": {"source": "debug 51-65: bowl is 0.114 across and hangs <0.07 from the "
                              "gripper, so 0.10 encloses it", "allowed": True},
    "BOWL_CLASS": {"source": "debug 51-65: the three bowls read top 0.042-0.051, span "
                             "0.085-0.115, mean rgb 107-148; the black stove knob reads "
                             "top 0.059 span 0.09 rgb 20", "allowed": True},
    "BOWL_H": {"source": "debug 51-65 cross-sections: rim top 0.051 above the bowl base",
               "allowed": True},
    "HANG_BAND": {"source": "v3 debug 51/53 lift snapshots: the held bowl occupies the "
                            "0.14-0.20 band with the tips at 0.170, i.e. tips-0.04..tips+0.03",
                  "allowed": True},
    "PLACE_GAP": {"source": "generic: release with the bowl base a few mm over the plate "
                            "so it settles rather than drops", "allowed": True},
    "HANG_MED": {"source": "sel_..._v6b full debug split 51-65: measured hang offset "
                           "median (0.0101,-0.0606), 14/15 within (0.0073..0.0161, "
                           "-0.0629..-0.0587)", "allowed": True},
    "HANG_GATE": {"source": "same run: the one bad reading (seed 62) sat 0.019 from the "
                            "median while every good one sat within 0.006",
                  "allowed": True},
    "PLACE_BIAS": {"source": "v4 debug 51/53/55/57/61/63/65: landed bowl sat "
                             "(-0.033,+0.023) from the plate centre, std (0.004,0.009)",
                   "allowed": True},
    "PLATE_BAND": {"source": "debug 51-65: the plate is the only 0.012-0.030 band blob "
                             "wider than 0.11 with mean rgb>120", "allowed": True},
    "GOTO_SETTLE": {"source": "v5 debug 51-65 logs: eef still moving 0.012 after a "
                              "goto returned; 0.25 s of settle reaches steady state",
                    "allowed": True},
    "PLATE_CLASS": {"source": "debug 51-65: the plate reads top 0.019, span 0.135, "
                              "rgb ~150; the cookie box reads top 0.019, span 0.080x0.060, "
                              "rgb [95,66,46]", "allowed": True},
}

CHUNK = 1800
X0, X1, Y0, Y1 = -0.72, 0.30, -0.62, 0.62
CELL = 0.005
ABOVE_TABLE = 0.012
ARM_CUT = 0.12
TIP_DZ = 0.0074
PINCH_R = 0.043
GRASP_H = 0.025
SAFE_H = 0.135
LIFT_H = 0.170
BOWL_H = 0.051
HANG_LO, HANG_HI = 0.045, 0.030
PLACE_GAP = 0.004
PLACE_BIAS = (0.0334, -0.0230)
HANG_MED = (0.0101, -0.0606)
HANG_GATE = 0.012
GOTO_SETTLE = 0.25
NX = int(round((X1 - X0) / CELL))
NY = int(round((Y1 - Y0) / CELL))
GX, GY = np.meshgrid(np.arange(NX), np.arange(NY), indexing="ij")
WX = X0 + (GX + 0.5) * CELL
WY = Y0 + (GY + 0.5) * CELL


def _dump(api, tag, arr):
    raw = np.ascontiguousarray(arr).tobytes()
    b = base64.b64encode(zlib.compress(raw, 6)).decode("ascii")
    api.log("DUMP %s dtype=%s shape=%s nchunk=%d" % (
        tag, arr.dtype.str, "x".join(str(s) for s in arr.shape),
        (len(b) + CHUNK - 1) // CHUNK))
    for i in range(0, len(b), CHUNK):
        api.log("D %s %d %s" % (tag, i // CHUNK, b[i:i + CHUNK]))


def snap(api, tag):
    f = api.capture("cam_high")
    _dump(api, tag + "_rgb", np.asarray(f.rgb)[::2, ::2, :].astype(np.uint8))
    _dump(api, tag + "_dep", np.asarray(f.depth, dtype=np.float32)[::2, ::2])
    api.log("SNAP %s eef=%s grip=%s" % (tag, np.round(api.eef(), 4).tolist(), api.gripper()))


def heightmap(frame):
    K = np.asarray(frame.intrinsics)
    T = np.asarray(frame.t_base_cam)
    dep = np.asarray(frame.depth, dtype=np.float64)
    rgb = np.asarray(frame.rgb, dtype=np.float32)
    Hpx, Wpx = dep.shape
    vv, uu = np.mgrid[0:Hpx, 0:Wpx]
    pts = np.stack([(uu - K[0, 2]) / K[0, 0] * dep,
                    (vv - K[1, 2]) / K[1, 1] * dep, dep], -1).reshape(-1, 3)
    pb = pts @ T[:3, :3].T + T[:3, 3]
    x, y, z = pb[:, 0], pb[:, 1], pb[:, 2]
    col = rgb.reshape(-1, 3)
    ok = (np.isfinite(z) & (x > X0) & (x < X1) & (y > Y0) & (y < Y1)
          & (z > 0.5) & (z < 1.6))
    flat = ((x[ok] - X0) / CELL).astype(int) * NY + ((y[ok] - Y0) / CELL).astype(int)
    zz, cc = z[ok], col[ok]
    Hf = np.full(NX * NY, -np.inf)
    Nf = np.zeros(NX * NY)
    Cf = np.zeros((NX * NY, 3), np.float32)
    np.maximum.at(Hf, flat, zz)
    np.add.at(Nf, flat, 1)
    order = np.argsort(zz)
    Cf[flat[order]] = cc[order]
    H = Hf.reshape(NX, NY)
    H[Nf.reshape(NX, NY) == 0] = np.nan
    return H, Cf.reshape(NX, NY, 3)


def table_z(H):
    v = H[np.isfinite(H)]
    hist, edges = np.histogram(v, bins=200, range=(0.7, 1.3))
    k = int(np.argmax(hist))
    return float(np.median(v[(v >= edges[k]) & (v <= edges[k + 1])]))


def components(mask):
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    for st in map(tuple, np.argwhere(mask)):
        if lab[st]:
            continue
        cur += 1
        lab[st] = cur
        stack = [st]
        while stack:
            a, b = stack.pop()
            for p in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                if (0 <= p[0] < mask.shape[0] and 0 <= p[1] < mask.shape[1]
                        and mask[p] and not lab[p]):
                    lab[p] = cur
                    stack.append(p)
    return lab, cur


def perceive(api):
    f = api.capture("cam_high")
    H, C = heightmap(f)
    tz = table_z(H)
    Hc = np.where(np.isfinite(H) & (H < tz + ARM_CUT), H, np.nan)
    lab, n = components(np.isfinite(Hc) & (Hc > tz + ABOVE_TABLE))
    objs = []
    for i in range(1, n + 1):
        w = lab == i
        if w.sum() < 40:
            continue
        objs.append(dict(n=int(w.sum()), x=float(WX[w].mean()), y=float(WY[w].mean()),
                         top=float(np.nanmax(Hc[w]) - tz),
                         dx=float(np.ptp(WX[w])), dy=float(np.ptp(WY[w])),
                         rgb=[float(q) for q in C[w].mean(0)], w=w))
    return H, Hc, C, tz, objs


def rim_centre(Hc, tz, x0, y0):
    """re-centre on the rim ring: the cells within 4 mm of the rim top"""
    cx, cy = x0, y0
    for _ in range(4):
        m = (np.isfinite(Hc) & (Hc > tz + 0.045)
             & (np.hypot(WX - cx, WY - cy) < 0.085))
        if m.sum() < 20:
            return cx, cy, 0
        cx, cy = float(WX[m].mean()), float(WY[m].mean())
    return cx, cy, int(m.sum())


def held_bowl(api, tz, tips_h, tag):
    """centre + rim top of the bowl hanging under the gripper, from cam_high"""
    f = api.capture("cam_high")
    H, C = heightmap(f)
    e = np.asarray(api.eef())
    band = (np.isfinite(H) & (H - tz > tips_h - HANG_LO) & (H - tz < tips_h + HANG_HI)
            & (np.hypot(WX - e[0], WY - e[1]) < 0.10))
    n = int(band.sum())
    if n < 25:
        api.log("HELD %s NONE n=%d" % (tag, n))
        return None
    cx, cy = float(WX[band].mean()), float(WY[band].mean())
    top = float(np.nanmax(H[band]) - tz)
    api.log("HELD %s n=%d ctr=(%+.4f,%+.4f) top=%.3f dx=%.3f dy=%.3f eef=%s rgb=%s" % (
        tag, n, cx, cy, top, float(np.ptp(WX[band])), float(np.ptp(WY[band])),
        np.round(e, 4).tolist(), [round(float(q)) for q in C[band].mean(0)]))
    _dump(api, tag + "_rgb", np.asarray(f.rgb)[::2, ::2, :].astype(np.uint8))
    _dump(api, tag + "_dep", np.asarray(f.depth, dtype=np.float32)[::2, ::2])
    return cx, cy, top, n


def run(api):
    H, Hc, C, tz, objs = perceive(api)
    api.log("TABLE_Z %.4f" % tz)
    for o in objs:
        api.log("OBJ n=%d ctr=(%+.4f,%+.4f) top=%.3f dx=%.3f dy=%.3f rgb=%s" % (
            o["n"], o["x"], o["y"], o["top"], o["dx"], o["dy"],
            [round(q) for q in o["rgb"]]))

    bowls = [o for o in objs if 0.035 < o["top"] < 0.075 and 0.06 < o["dx"] < 0.17
             and 0.06 < o["dy"] < 0.17 and float(np.mean(o["rgb"])) > 70]
    # plate: height-band first, then cluster -- a bowl touching the plate merges
    # with it if the blobs are formed before the band cut (v4 seed 59 receipt)
    plates = []
    bandobjs = []
    blab, bn = components(np.isfinite(Hc) & (Hc - tz > 0.012) & (Hc - tz < 0.030))
    for i in range(1, bn + 1):
        w = blab == i
        if w.sum() < 120:
            continue
        d = dict(n=int(w.sum()), x=float(WX[w].mean()), y=float(WY[w].mean()),
                 top=float(np.nanmax(Hc[w]) - tz), dx=float(np.ptp(WX[w])),
                 dy=float(np.ptp(WY[w])), rgb=[float(q) for q in C[w].mean(0)])
        bandobjs.append(d)
        if (d["dx"] > 0.11 and d["dy"] > 0.11 and d["top"] < 0.026
                and float(np.mean(d["rgb"])) > 120):
            plates.append(d)
            api.log("PLATE_CAND n=%d ctr=(%+.4f,%+.4f) %.3fx%.3f top=%.3f rgb=%s" % (
                d["n"], d["x"], d["y"], d["dx"], d["dy"], d["top"],
                [round(q) for q in d["rgb"]]))
    if not bowls:
        api.log("PERCEPTION_FAIL bowls=0")
        return
    if not plates:
        # relax: widest light band blob that is not the cookie box
        loose = [d for d in bandobjs if d["dx"] > 0.09 and d["dy"] > 0.09
                 and float(np.mean(d["rgb"])) > 100]
        api.log("PLATE_RELAXED n=%d" % len(loose))
        if not loose:
            api.log("PERCEPTION_FAIL plates=0")
            return
        plates = [max(loose, key=lambda d: d["n"])]
    tab = np.isfinite(H) & (np.abs(H - tz) < 0.006)
    ccx = 0.5 * (np.percentile(WX[tab], 1) + np.percentile(WX[tab], 99))
    ccy = 0.5 * (np.percentile(WY[tab], 1) + np.percentile(WY[tab], 99))
    tgt = min(bowls, key=lambda b: np.hypot(b["x"] - ccx, b["y"] - ccy))
    plate = max(plates, key=lambda p: p["n"])
    bx, by, nrim = rim_centre(Hc, tz, tgt["x"], tgt["y"])
    api.log("TABLE_CTR %+.4f %+.4f" % (ccx, ccy))
    api.log("TARGET blob=(%+.4f,%+.4f) rim=(%+.4f,%+.4f) nrim=%d top=%.3f"
            % (tgt["x"], tgt["y"], bx, by, nrim, tgt["top"]))
    api.log("PLATE (%+.4f,%+.4f) top=%.3f n=%d" % (plate["x"], plate["y"],
                                                   plate["top"], plate["n"]))

    def goto(x, y, tip_h, seconds=2.0, iters=2, tag=""):
        """command in eef frame; close the loop on the measured tracking bias"""
        want = np.array([x, y, tz + tip_h + TIP_DZ])
        cmd = want.copy()
        for k in range(iters):
            api.move(cmd.tolist(), seconds=seconds)
            api.settle(GOTO_SETTLE)          # OSC is still travelling on return
            e = np.asarray(api.eef())
            err = want - e
            api.log("GOTO %s it%d want=%s eef=%s err=%s" % (
                tag, k, np.round(want, 4).tolist(), np.round(e, 4).tolist(),
                np.round(err, 4).tolist()))
            if np.linalg.norm(err) < 0.003:
                break
            cmd = cmd + err
            seconds = 1.2
        return cmd

    api.grip(0.08)
    api.settle(0.2)
    gx, gy = bx, by + PINCH_R
    goto(gx, gy, SAFE_H, seconds=2.5, tag="pre")
    goto(gx, gy, GRASP_H, seconds=2.0, tag="down")
    api.log("BEFORE_CLOSE grip=%s" % api.gripper())
    api.grip(0.0)
    api.settle(0.6)
    api.log("AFTER_CLOSE grip=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))
    goto(gx, gy, LIFT_H, seconds=2.5, tag="lift")
    api.settle(0.4)
    api.log("AFTER_LIFT grip=%s" % api.gripper())
    w = api.capture("cam_arm_wrist")
    _dump(api, "wlift_dep", np.asarray(w.depth, dtype=np.float32)[::2, ::2])
    api.log("WLIFT_T %s" % np.asarray(w.t_base_cam).reshape(-1).tolist())
    api.log("WLIFT_K %s" % np.asarray(w.intrinsics).reshape(-1).tolist())

    px, py = plate["x"], plate["y"]
    h = held_bowl(api, tz, LIFT_H, "lift")
    med = np.array(HANG_MED)
    if h is None:
        api.log("HANG_NOREAD -> median")
        off = med
    else:
        e = np.asarray(api.eef())
        off = np.array([h[0] - e[0], h[1] - e[1]])   # bowl centre minus gripper
        api.log("HANG_RAW %s" % np.round(off, 4).tolist())
        if np.linalg.norm(off - med) > HANG_GATE:
            api.log("HANG_REJECT d=%.4f -> median" % float(np.linalg.norm(off - med)))
            off = med
    api.log("HANG_OFF %s" % np.round(off, 4).tolist())

    aim_x = px - off[0] + PLACE_BIAS[0]
    aim_y = py - off[1] + PLACE_BIAS[1]
    api.log("AIM %+.4f %+.4f" % (aim_x, aim_y))
    cmd = goto(aim_x, aim_y, LIFT_H, seconds=3.0, tag="over")
    hold = (float(cmd[0]), float(cmd[1]))            # the command the arm sits at
    api.log("HOLD %+.4f %+.4f eef=%s" % (hold[0], hold[1],
                                         np.round(api.eef(), 4).tolist()))
    e = np.asarray(api.eef())

    base_now = h[2] - BOWL_H if h is not None else LIFT_H - 0.039
    tips_place = float(min(max(LIFT_H - (base_now - (plate["top"] + PLACE_GAP)),
                               0.030), LIFT_H))
    api.log("PLACE base_now=%.3f tips_place=%.3f" % (base_now, tips_place))
    goto(aim_x, aim_y, tips_place, seconds=2.0, tag="place")
    api.settle(0.3)
    api.log("BEFORE_OPEN grip=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))
    api.grip(0.08)
    api.settle(0.8)
    snap(api, "released")
    goto(aim_x, aim_y, LIFT_H, seconds=2.0, iters=1, tag="retreat")
    api.settle(0.6)
    snap(api, "final")
    api.log("DONE v8")
