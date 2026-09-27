"""c2clean goal_put_wine_on_rack_pos_k0 -- v12: BODY grasp + ramp release.

What v7's height sweep settled (debug seeds 51,55):
  * commanding eef z = table + 0.150 over the bottle centre closes the jaws on
    the NECK: width 0.0149, effort 3.000.  So `effort 3.0` really is "jaws
    blocked"; v3's 3.0 came with width 0.080, which is the post-horizon tell.
  * pressing lower while holding stalls the eef at table + 0.120, i.e. the
    fingertips sit about 0.021 m below the reported eef.
  * the jaws close along **y** (wrist view: the hand spans +-0.06 in y), and
    the grasp tolerates almost no error in **x**: v6 sat 0.0153 m off in x and
    caught nothing, v7 sat 0.006 m off and caught the neck.
  * a purely vertical descent drifts +x. So xy is corrected once at hover
    height and once again at the grasp height before the jaws close.

v8 result (0/8): the xy correction works (landed 0.6-3.4 mm off), and ep51
actually closed on the neck (width 0.0122, effort 3.0) -- but the bottle
slipped straight back out on the lift (width fell to 0.001), and ep55 caught
nothing at all at the same commanded height. The neck is a 0.015 m target
with a ~0.02 m usable height band; it is the wrong feature.

v9 grasps the BODY instead. The body is 0.042 m across and that width holds
from ~0.02 m over the table up to the shoulder at ~0.09 m -- a 0.07 m tall
band, so the residual z droop (+0.008 .. +0.019 depending on descent length)
cannot miss it.

The hang is no longer assumed. At the instant the jaws close the bottle's base
is still on the table, so hang = eef_z_at_close - table_z exactly; the release
height follows from that measurement rather than from a fingertip-offset
guess. Carry stays at z = 1.42 (v2: reachable, residual <= 0.023).

v10 (this file) changes exactly one thing about v9, whose full-15 selection was
11/15.  In the four v9 failures the descent never reached the commanded grasp
height: the eef stalled 0.023-0.049 m high (ep52 1.0191, ep54/62 0.9941) and
the jaws then closed on air or on the neck.  The v9 fix move could not rescue
it because the bias-cancelled command `cx` had drifted far from where the eef
actually was, so the whole 0.9 s went sideways and z did not change at all
(descend and descend_fix ended at the SAME z to 0.3 mm).

So v10 adds up to two "press" moves that fire when the eef is still high.
Their target is built from the ACHIEVED eef, not from the accumulated command:
`x <- e_x + 2*(bx - e_x)`, which is a small lateral demand (the doubling is the
same 2x bias cancel v6 measured, closing ~55% per move), so nearly the whole
move goes into z.  That both re-centres a fingertip that may be resting on the
bottle's shoulder and pushes down.  A seed that already arrives (11 of 15)
takes no extra move and runs byte-for-byte the v9 sequence.

v10 probe on the even debug seeds (52,54,56,58,60,62,64) scored 3/7: the press
rescued ep52 (gained 0.041 m, body grasp, success) but gained 0.0006 m on
54/62, and it over-corrected ep58 into an air grasp.  So a press aimed AT the
grasp height is too weak a command.

v11 aims the press 0.045 m BELOW the grasp height (a large command saturates
the controller the way ep58's accidentally large one did) and drops the bias
cancel from 2x to 1.5x.  Probe 4/7: ep52 and ep58 both succeed, ep58 by a
0.039 m base grasp at z = table+0.034 -- which places correctly, because the
hang is measured at the moment of closing rather than assumed.

v12 (this file) is v11 plus one stopping rule.  On 54/62 the eef is genuinely
blocked at table+0.092: neither press moves it, and the two wasted moves
starve the later `lower` (ep62 carried the bottle to the rack on a 0.024 m
grip and then released 0.05 m too high).  So a press that gains nothing WHILE
the eef is already over the bottle ends the loop; a press that gains nothing
but leaves the eef far off in xy (ep58) is a starved move, and the next press
carries a large command and frees it.
"""
import base64
import zlib
from collections import deque

import numpy as np

PROVENANCE = {
    "WORKSPACE_CROP": {
        "source": "debug seeds 51-65 (v1): cam_high cloud reaches the back wall "
                  "at x=-1.99; this crop keeps the table only",
        "allowed": True},
    "TABLE_Z": {
        "source": "debug seeds 51-65 (v1): modal depth plane, 0.9010 on all 15; "
                  "recomputed per episode at runtime",
        "allowed": True},
    "BOTTLE_SELECTORS": {
        "source": "debug seeds 51-65 (v1): bottle = darkest above-table cluster, "
                  "mean rgb ~(20,22,15) vs next darkest (66,65,63); footprint "
                  "0.041-0.043 m; top 0.157 m over table",
        "allowed": True},
    "LIE_H_RANGE": {
        "source": "debug seed 64 (v1): on that seed the bottle starts on its "
                  "side, ~0.04 m tall and ~0.16 m long",
        "allowed": True},
    "RACK_SELECTORS": {
        "source": "debug seeds 51-65 (v1): wood cluster top 0.315-0.328 m over "
                  "table with r-b = +29; cabinet deck 0.227 m with r-b = +3; "
                  "arm r-b = -18",
        "allowed": True},
    "PLACE_YFRAC": {
        "source": "debug seeds 51-65 (v1): rack ramp spans 0.137 m in y; 0.45 "
                  "from its high edge measured 0.283 m over table on all 15",
        "allowed": True},
    "GRASP_CMD_H": {
        "source": "debug seeds 51/55 (v2 wrist profile): the bottle body reads "
                  "0.037-0.042 m across from 0.02 m over the table to the "
                  "shoulder at ~0.09 m; table+0.070 aims mid-band, and the "
                  "measured z droop (+0.008..+0.019, v5/v8) stays inside it",
        "allowed": True},
    "TIP_OFF": {
        "source": "debug seeds 51,55 (v7): holding the neck, the eef stalled at "
                  "table+0.120 with the bottle's shoulder on the tips. Used "
                  "only as a descent floor, not in the placement arithmetic",
        "allowed": True},
    "BODY_W": {
        "source": "debug seeds 51-65 (v1): bottle footprint 0.041-0.043 m, so a "
                  "body grasp must close to roughly that width",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug seeds 51,55 (v2): z=1.42 reached everywhere surveyed "
                  "with residual <= 0.023",
        "allowed": True},
    "XY_TOL": {
        "source": "debug seeds 51,55 (v6 vs v7): 0.0153 m of x error grasped "
                  "air, 0.006 m caught the neck",
        "allowed": True},
    "Z_TOL": {
        "source": "debug seeds 51-65 (v9 selection run): every seed that closed "
                  "on the body arrived within 0.008 m of the commanded grasp "
                  "height; every seed that grasped air stalled 0.023-0.049 m "
                  "high. 0.015 separates the two populations",
        "allowed": True},
    "PRESS_DEPTH": {
        "source": "debug seeds 52,54,58,62 (v10 probe): a press aimed AT the "
                  "grasp height gained 0.0006 m on seeds 54/62, while the one "
                  "command that carried a large error (ep58 press1) gained "
                  "0.039 m. Aiming 0.045 m below keeps the command large; the "
                  "loop still exits as soon as the eef is at the grasp height, "
                  "and the hang is measured after the fact so a deeper close "
                  "does not corrupt the release height",
        "allowed": True},
    "PRESS_K": {
        "source": "debug seeds 52,58 (v10 probe): a 2x bias cancel overshot "
                  "ep58 by 0.0145 m while ep52 landed 0.0042 m short; 1.5x "
                  "sits between them",
        "allowed": True},
    "PRESS_MIN_GAIN": {
        "source": "debug seeds 54,58,62 (v10/v11 probes): a press that frees "
                  "the eef gains 0.039-0.084 m of z; a blocked one gains "
                  "0.0000-0.0025 m. 0.005 separates them",
        "allowed": True},
    "XY_ESC": {
        "source": "debug seeds 54,58,62 (v11 probe): after the first press the "
                  "eef sat 0.024 m off the bottle on seed 58 (where a second "
                  "press freed it) and 0.0075-0.0092 m off on 54/62 (where it "
                  "did not); 0.015 separates them",
        "allowed": True},
    "N_PRESS": {
        "source": "debug seeds 51-65 (v4 budget probe + v9 timings): the "
                  "episode takes about 14 s of api.move; v9's sequence spends "
                  "11.1 s, so at most two extra 1.0 s moves fit",
        "allowed": True},
    "HOLD_W": {
        "source": "debug seed 51 (v4/v7): free-space close reads 0.001; a neck "
                  "grasp reads 0.0149",
        "allowed": True},
    "DUMP_SIDE": {"source": "data-plumbing only", "allowed": True},
}

XR = (-0.45, 0.30)
YR = (-0.60, 0.50)
BOTTLE_H = (0.10, 0.22)
LIE_H = (0.025, 0.10)
BOTTLE_FOOT_MAX = 0.085
LIE_FOOT_MAX = 0.22
BOTTLE_DARK_MAX = 70.0
RACK_H = (0.20, 0.40)
RACK_RB_MIN = 15.0
PLACE_YFRAC = 0.45
GRASP_CMD_H = 0.070
TIP_OFF = 0.021
BODY_W_MIN = 0.025
CARRY_Z = 1.42
HOVER_H = 0.30
XY_TOL = 0.005
Z_TOL = 0.015
N_PRESS = 2
PRESS_K = 1.5
PRESS_DEPTH = 0.045
PRESS_MIN_GAIN = 0.005
XY_ESC = 0.015
HOLD_W = 0.005
DUMP_SIDE = 256
CHUNK = 1800


def _dump(api, tag, arr):
    b64 = base64.b64encode(zlib.compress(
        np.ascontiguousarray(arr).tobytes(), 6)).decode()
    api.log(f"DUMP {tag} shape={list(arr.shape)} dtype={arr.dtype} "
            f"nchunk={(len(b64) + CHUNK - 1) // CHUNK}")
    for i in range(0, len(b64), CHUNK):
        api.log(f"D {tag} {i // CHUNK} {b64[i:i + CHUNK]}")


def _shot(api, cam, tag, dump=False):
    f = api.capture(cam)
    if dump:
        step = max(1, f.rgb.shape[0] // DUMP_SIDE)
        d = np.asarray(f.depth, np.float64)
        d = np.where(np.isfinite(d) & (d > 0), d, 0.0)
        api.log(f"CAM {tag} K={np.asarray(f.intrinsics).round(3).tolist()}")
        api.log(f"CAM {tag} T={np.asarray(f.t_base_cam).round(5).tolist()}")
        _dump(api, f"{tag}_rgb", np.asarray(f.rgb, np.uint8)[::step, ::step])
        _dump(api, f"{tag}_d100um",
              np.clip(d * 10000.0, 0, 65535).astype(np.uint16)[::step, ::step])
    return f


def cloud(frame):
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    d = np.asarray(frame.depth, float)
    d = np.where(np.isfinite(d) & (d > 0), d, 0.0)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    P = np.stack([(uu - K[0, 2]) * d / K[0, 0],
                  (vv - K[1, 2]) * d / K[1, 1], d, np.ones_like(d)], -1)
    ok = d > 0
    return (P @ T.T)[..., :3][ok], np.asarray(frame.rgb, float)[ok]


def table_z(P):
    z = P[:, 2]
    z = z[(z > 0.70) & (z < 1.10)]
    h, e = np.histogram(z, bins=400)
    i = int(np.argmax(h))
    return float(np.median(z[(z > e[i] - 0.01) & (z < e[i + 1] + 0.01)]))


def components(P, C, tz, hmin=0.012, cell=0.012, nmin=60):
    a = P[:, 2] > tz + hmin
    Pa, Ca = P[a], C[a]
    if len(Pa) == 0:
        return []
    nx = int((XR[1] - XR[0]) / cell) + 2
    ny = int((YR[1] - YR[0]) / cell) + 2
    ix = np.clip(((Pa[:, 0] - XR[0]) / cell).astype(int), 0, nx - 1)
    iy = np.clip(((Pa[:, 1] - YR[0]) / cell).astype(int), 0, ny - 1)
    occ = np.zeros((nx, ny), bool)
    occ[ix, iy] = True
    lab = -np.ones((nx, ny), int)
    n = 0
    for i in range(nx):
        for j in range(ny):
            if occ[i, j] and lab[i, j] < 0:
                q = deque([(i, j)])
                lab[i, j] = n
                while q:
                    a1, b1 = q.popleft()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            u, v = a1 + da, b1 + db
                            if 0 <= u < nx and 0 <= v < ny and occ[u, v] \
                                    and lab[u, v] < 0:
                                lab[u, v] = n
                                q.append((u, v))
                n += 1
    pl = lab[ix, iy]
    out = []
    for L in range(n):
        s = pl == L
        if int(s.sum()) < nmin:
            continue
        Q, K2 = Pa[s], Ca[s]
        out.append({"n": int(s.sum()), "P": Q, "C": K2,
                    "xmin": float(Q[:, 0].min()), "xmax": float(Q[:, 0].max()),
                    "ymin": float(Q[:, 1].min()), "ymax": float(Q[:, 1].max()),
                    "ztop": float(np.percentile(Q[:, 2], 99)),
                    "h": float(np.percentile(Q[:, 2], 99) - tz),
                    "rgb": K2.mean(0)})
    return out


def _goto(api, tag, xyz, seconds):
    r = api.move([float(v) for v in xyz], seconds=seconds)
    e = np.asarray(api.eef(), float)
    api.log(f"MOVE {tag} tgt={[round(float(v), 4) for v in xyz]} "
            f"res={r:.4f} eef={e.round(4).tolist()}")
    return e


def _g(api, tag):
    g = api.gripper()
    api.log(f"GRIP {tag} width={g['width_m']:.5f} effort={g['effort']:.3f}")
    return g


def run(api):
    api.log(f"START eef={np.asarray(api.eef()).round(4).tolist()}")
    f = _shot(api, "cam_high", "h0", dump=True)
    P, C = cloud(f)
    m = ((P[:, 0] > XR[0]) & (P[:, 0] < XR[1]) &
         (P[:, 1] > YR[0]) & (P[:, 1] < YR[1]) &
         (P[:, 2] > 0.5) & (P[:, 2] < 1.35))
    P, C = P[m], C[m]
    tz = table_z(P)
    comps = components(P, C, tz)
    api.log(f"TABLE_Z {tz:.4f} ncomp={len(comps)}")
    for c in comps:
        api.log(f"COMP n={c['n']} x=({c['xmin']:.3f},{c['xmax']:.3f}) "
                f"y=({c['ymin']:.3f},{c['ymax']:.3f}) h={c['h']:.3f} "
                f"rgb={[int(v) for v in c['rgb']]}")

    # -- bottle: prefer the standing one; fall back to a lying one ----------
    bot, lying = None, False
    for c in comps:
        foot = max(c["xmax"] - c["xmin"], c["ymax"] - c["ymin"])
        if (BOTTLE_H[0] < c["h"] < BOTTLE_H[1] and foot < BOTTLE_FOOT_MAX
                and c["rgb"].mean() < BOTTLE_DARK_MAX):
            if bot is None or c["rgb"].mean() < bot["rgb"].mean():
                bot = c
    if bot is None:
        for c in comps:
            foot = max(c["xmax"] - c["xmin"], c["ymax"] - c["ymin"])
            if (LIE_H[0] < c["h"] < LIE_H[1] and 0.08 < foot < LIE_FOOT_MAX
                    and c["rgb"].mean() < BOTTLE_DARK_MAX):
                if bot is None or c["rgb"].mean() < bot["rgb"].mean():
                    bot, lying = c, True
    if bot is None:
        api.log("ABORT no bottle")
        return "no bottle"

    if lying:
        # grasp across the cylinder at its darkest, thickest end
        bx = float(np.median(bot["P"][:, 0]))
        by = float(np.median(bot["P"][:, 1]))
        gz = tz + max(0.02, bot["h"] * 0.6) + TIP_OFF
    else:
        r = 0.5 * (bot["ymax"] - bot["ymin"])
        bx, by = bot["xmax"] - r, 0.5 * (bot["ymin"] + bot["ymax"])
        gz = tz + GRASP_CMD_H
    api.log(f"BOTTLE lying={lying} xy=({bx:.4f},{by:.4f}) h={bot['h']:.3f} "
            f"gz={gz:.4f}")

    # -- rack ramp ----------------------------------------------------------
    rack = None
    for c in comps:
        if RACK_H[0] < c["h"] < RACK_H[1] and \
                (c["rgb"][0] - c["rgb"][2]) > RACK_RB_MIN:
            if rack is None or c["n"] > rack["n"]:
                rack = c
    if rack is None:
        api.log("ABORT no rack")
        return "no rack"
    Q, KC = rack["P"], rack["C"]
    R = Q[(Q[:, 2] - tz > 0.20) & ((KC[:, 0] - KC[:, 2]) > RACK_RB_MIN)]
    ylo, yhi = float(R[:, 1].min()), float(R[:, 1].max())
    py = ylo + PLACE_YFRAC * (yhi - ylo)
    px = float(np.median(R[:, 0]))
    near = R[(np.abs(R[:, 0] - px) < 0.05) & (np.abs(R[:, 1] - py) < 0.010)]
    zsurf = float(np.percentile(near[:, 2], 90)) if len(near) > 10 \
        else rack["ztop"] - 0.045
    api.log(f"RAMP y=({ylo:.3f},{yhi:.3f}) place=({px:.4f},{py:.4f}) "
            f"zsurf={zsurf:.4f} h={zsurf - tz:.4f}")

    # -- approach: hover, cancel xy bias, descend, cancel again -------------
    api.grip(0.08)
    cx, cy = bx, by
    e = _goto(api, "hover", [cx, cy, tz + HOVER_H], 2.0)
    cx += bx - e[0]
    cy += by - e[1]
    if abs(bx - e[0]) > XY_TOL or abs(by - e[1]) > XY_TOL:
        e = _goto(api, "hover_fix", [cx, cy, tz + HOVER_H], 0.9)
        cx += bx - e[0]
        cy += by - e[1]

    e = _goto(api, "descend", [cx, cy, gz], 1.4)
    api.log(f"AIM descend dxy=({bx - e[0]:+.4f},{by - e[1]:+.4f}) "
            f"dz={e[2] - gz:+.4f}")
    if abs(bx - e[0]) > XY_TOL or abs(by - e[1]) > XY_TOL:
        cx += bx - e[0]
        cy += by - e[1]
        e = _goto(api, "descend_fix", [cx, cy, gz], 0.9)
        api.log(f"AIM descend_fix dxy=({bx - e[0]:+.4f},{by - e[1]:+.4f}) "
                f"dz={e[2] - gz:+.4f}")

    # the eef may still be sitting high (v9's four failures).  Push it down
    # with a command anchored on the achieved pose, so the lateral demand is
    # small and the move spends itself on z.
    for k in range(N_PRESS):
        if e[2] - gz <= Z_TOL:
            break
        tx = float(e[0]) + PRESS_K * (bx - float(e[0]))
        ty = float(e[1]) + PRESS_K * (by - float(e[1]))
        prev_z = float(e[2])
        e = _goto(api, f"press{k}", [tx, ty, gz - PRESS_DEPTH], 1.0)
        api.log(f"AIM press{k} dxy=({bx - e[0]:+.4f},{by - e[1]:+.4f}) "
                f"dz={e[2] - gz:+.4f} gained={prev_z - e[2]:+.4f}")
        # a press that gains nothing while the eef is already over the bottle
        # is a real block, and the next press will be just as stuck (seeds
        # 54/62); spend the remaining steps on the place instead.  A press
        # that gains nothing but leaves the eef far off in xy (seed 58) is a
        # starved move, and the next one carries a large command and frees it.
        off = max(abs(bx - float(e[0])), abs(by - float(e[1])))
        if prev_z - float(e[2]) < PRESS_MIN_GAIN and off < XY_ESC:
            api.log(f"PRESS_STUCK off={off:.4f} -- closing here")
            break

    api.grip(0.0)
    api.settle(0.3)
    gc = _g(api, "closed")
    # the bottle's base is still on the table at this instant, so the distance
    # from the eef down to the base is exactly this:
    hang = float(e[2]) - tz
    api.log(f"HANG {hang:.4f} (eef_z_at_close {e[2]:.4f} - table {tz:.4f})")

    e = _goto(api, "lift", [e[0], e[1], CARRY_Z], 1.6)
    gl = _g(api, "lifted")
    api.log(f"HOLDING {gl['width_m'] > HOLD_W} "
            f"body_grasp={gl['width_m'] > BODY_W_MIN}")

    # -- place --------------------------------------------------------------
    _goto(api, "over_place", [px, py, CARRY_Z], 1.6)
    ez = zsurf + 0.012 + hang
    _goto(api, "lower", [px, py, ez], 1.5)
    _g(api, "at_place")
    api.grip(0.08)
    api.settle(0.4)
    _goto(api, "retreat", [px, py, CARRY_Z], 1.2)
    api.settle(0.5)
    _shot(api, "cam_high", "hend", dump=True)
    return "v9 body grasp + ramp release"
