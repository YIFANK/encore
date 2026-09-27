"""v7 -- edge strike with a per-episode tip calibration and a strike-height ladder.

What v3/v4 taught:
  * motion is accurate when every api.move is checked against api.eef() and
    re-issued only while off target (v3: err ~0.010 on every waypoint);
  * the episode horizon is 1000 sim steps and a frozen sim reports an
    unchanging eef, so the plan must stay lean (v3 = 183 steps for 8 moves);
  * captures cost no sim steps, so perception can be repeated freely;
  * the plate is a SAUCER, not a slab: its own depth profile rises from
    table+0.007 at the centre to table+0.019 at the outer edge (v1 dump,
    seed 51).  A blade 0.006 m above the table -- v3 -- passes underneath the
    flared rim and touches nothing, which is exactly what v3 observed;
  * the -y approach corridor at the plate's own x is shared with the bowl
    (v3 nudged the bowl 0.003 m), so the contact x must be chosen for
    clearance, not assumed.

MODE picks the contact mechanism:
  "edge"  -- blade at EDGE_H under the rim top, struck against the outer edge;
  "press" -- blade pressed down into the dish and dragged (self-calibrating,
             the descent is stopped by the plate itself).
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "CROP_X": {"source": "debug-seed 51/53/55/57 cam_high depth: table top plane spans x[-0.50,0.27]", "allowed": True},
    "CROP_Y": {"source": "debug-seed cam_high depth: cabinet/rack occupy y<-0.12; props of interest at y>-0.02", "allowed": True},
    "TABLE_Z_BAND": {"source": "debug-seed cam_high depth histogram: table plane spike, median 0.901", "allowed": True},
    "PROP_Z_MIN": {"source": "debug-seed depth: table 0.901, lowest prop (plate outer rim) top 0.920", "allowed": True},
    "PROP_Z_MAX": {"source": "debug-seed depth: stove top 0.961; excludes the arm, which reaches 1.35", "allowed": True},
    "MIN_PIX": {"source": "debug-seed component sizes: plate ~890 px, stove ~8180 px", "allowed": True},
    "TIP_OFF": {"source": "debug-seed 51/53 v11 descent ladder with error-cancelling moves: over bare table the achieved z plateaus at 0.9090 while the command falls to 0.871, and 0.9090-0.9010(table)=0.0080; the stove ladder plateaus consistently on the slab", "allowed": True},
    "LADDER_H": {"source": "debug-seed 51 plate radial depth profile: rim crest table+0.019, dish floor table+0.007; ladder brackets the outer rim wall", "allowed": True},
    "SAFE_H": {"source": "debug-seed depth: tallest prop is the wine bottle at ztop 1.059, i.e. table+0.158; transit height set above it", "allowed": True},
    "MAX_STRIKES": {"source": "debug-seed v3 step accounting: ~120 sim steps per strike against a 1000-step episode horizon", "allowed": True},
    "ALIGN_TOL": {"source": "debug-seed re-perception repeatability of the plate centre (0.000 m on a static plate)", "allowed": True},
    "OVERSHOOT": {"source": "debug-seed v7: only 0.023 m of a 0.051 m sweep transferred, so the sweep is run past the target", "allowed": True},
    "MOVED_TOL": {"source": "debug-seed re-perception repeatability: a static plate reproduces its centre to 0.000 m (v3/v6 FINAL logs)", "allowed": True},
    "APPROACH_GAP": {"source": "debug-seed geometry: clearance so the blade starts off the rim before the sweep", "allowed": True},
    "BLADE_HALF": {"source": "generic gripper mechanics: closed fingers form a blade; half-extent used for corridor clearance", "allowed": True},
    "Y_MARGIN": {"source": "debug-seed v3: a blade started 0.005 m clear of the bowl still nudged it 0.003 m", "allowed": True},
    "CX_CANDIDATES": {"source": "debug-seed geometry: plate half-width 0.068, so contact within 0.045 of centre still translates it", "allowed": True},
    "RES_TOL": {"source": "generic controller mechanics: re-issue a move whose achieved pose is off target", "allowed": True},
}

TIP_OFF = 0.0080
LADDER_H = (0.010, 0.014, 0.006)
SAFE_H = 0.220
MAX_STRIKES = 5
ALIGN_TOL = 0.012
OVERSHOOT = 0.030
MOVED_TOL = 0.004

CROP_X = (-0.52, 0.26)
CROP_Y = (-0.02, 0.55)
TABLE_Z_BAND = (0.885, 0.912)
PROP_Z_MIN = 0.915
PROP_Z_MAX = 1.20
MIN_PIX = 300
APPROACH_GAP = 0.018
BLADE_HALF = 0.018
Y_MARGIN = 0.015
CX_CANDIDATES = (0.0, 0.02, -0.02, 0.035, -0.035, 0.045)
RES_TOL = 0.012


def _dump(api, tag, fr):
    rgb = np.ascontiguousarray(fr.rgb.astype(np.uint8)[::2, ::2])
    d = np.nan_to_num(np.asarray(fr.depth, float), nan=0.0, posinf=0.0, neginf=0.0)
    dmm = np.clip(d * 1000.0, 0, 65535).astype(np.uint16)
    for name, arr in (("rgb", rgb), ("d16", dmm)):
        b = base64.b64encode(zlib.compress(arr.tobytes(), 6)).decode()
        api.log("%s %s LEN=%d" % (tag, name, len(b)))
        for i in range(0, len(b), 1900):
            api.log("%s %s C%d %s" % (tag, name, i // 1900, b[i:i + 1900]))


def _cloud(fr):
    d = np.nan_to_num(np.asarray(fr.depth, float), nan=0.0, posinf=0.0, neginf=0.0)
    rgb = np.asarray(fr.rgb).astype(float)
    K = np.asarray(fr.intrinsics, float)
    T = np.asarray(fr.t_base_cam, float)
    H, W = d.shape
    v, u = np.mgrid[0:H, 0:W]
    pcam = np.stack([(u - K[0, 2]) / K[0, 0] * d, (v - K[1, 2]) / K[1, 1] * d, d], -1)
    return pcam @ T[:3, :3].T + T[:3, 3], rgb, d


def _label(mask):
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    nxt = 0
    for i0, j0 in np.argwhere(mask):
        if lab[i0, j0]:
            continue
        nxt += 1
        stack = [(int(i0), int(j0))]
        lab[i0, j0] = nxt
        while stack:
            a, b = stack.pop()
            for p in range(max(a - 1, 0), min(a + 2, H)):
                for q in range(max(b - 1, 0), min(b + 2, W)):
                    if mask[p, q] and not lab[p, q]:
                        lab[p, q] = nxt
                        stack.append((p, q))
    return lab, nxt


def perceive(api, tag, dump=False):
    fr = api.capture("cam_high")
    if dump:
        _dump(api, tag, fr)
    P, rgb, d = _cloud(fr)
    inbox = ((d > 0) & (P[..., 0] > CROP_X[0]) & (P[..., 0] < CROP_X[1])
             & (P[..., 1] > CROP_Y[0]) & (P[..., 1] < CROP_Y[1]))
    tab = inbox & (P[..., 2] > TABLE_Z_BAND[0]) & (P[..., 2] < TABLE_Z_BAND[1])
    table_z = float(np.median(P[..., 2][tab])) if tab.sum() > 500 else 0.901
    mask = inbox & (P[..., 2] > PROP_Z_MIN) & (P[..., 2] < PROP_Z_MAX)
    lab, n = _label(mask)
    comps = []
    for k in range(1, n + 1):
        m = lab == k
        c = int(m.sum())
        if c < MIN_PIX:
            continue
        pp = P[m]
        comps.append({"px": c, "bright": float(rgb[m].mean()),
                      "xlo": float(pp[:, 0].min()), "xhi": float(pp[:, 0].max()),
                      "ylo": float(pp[:, 1].min()), "yhi": float(pp[:, 1].max()),
                      "ztop": float(np.percentile(pp[:, 2], 99.5))})
        api.log("%s COMP px=%d bright=%.1f x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f"
                % (tag, c, comps[-1]["bright"], comps[-1]["xlo"], comps[-1]["xhi"],
                   comps[-1]["ylo"], comps[-1]["yhi"], comps[-1]["ztop"]))
    api.log("%s TABLE_Z %.4f ncomp=%d" % (tag, table_z, len(comps)))
    return comps, table_z, d


def pick(comps, table_z):
    low = [c for c in comps if c["ztop"] - table_z < 0.045]
    tall = [c for c in comps if c["ztop"] - table_z >= 0.045]
    plate = max(low, key=lambda c: c["bright"]) if low else None
    stove = max(tall, key=lambda c: c["px"]) if tall else None
    return plate, stove


def goto(api, name, xyz, seconds=2.0, tries=2):
    tgt = np.asarray(xyz, float)
    e = np.asarray(api.eef(), float)
    for t in range(tries):
        api.move(tgt.tolist(), seconds=seconds)
        e = np.asarray(api.eef(), float)
        err = float(np.linalg.norm(e - tgt))
        api.log("MOVE %s try=%d tgt=%s eef=%s err=%.4f"
                % (name, t, np.round(tgt, 4).tolist(), np.round(e, 4).tolist(), err))
        if err <= RES_TOL:
            break
    return e


def centre(c):
    return 0.5 * (c["xlo"] + c["xhi"]), 0.5 * (c["ylo"] + c["yhi"])


def radius(c):
    return 0.25 * ((c["xhi"] - c["xlo"]) + (c["yhi"] - c["ylo"]))


def choose_contact(api, comps, plate, px, py, pr):
    """Contact x nearest the plate centre whose -y approach corridor is clear."""
    others = [c for c in comps if c is not plate]
    best = None
    for dx in CX_CANDIDATES:
        cx = px + dx
        inner = max(pr * pr - dx * dx, 4e-4) ** 0.5
        rim_y = py - inner
        start_y = rim_y - APPROACH_GAP
        lo, hi = cx - BLADE_HALF, cx + BLADE_HALF
        ylo = start_y - Y_MARGIN
        clear = True
        for c in others:
            if c["xhi"] < lo or c["xlo"] > hi:
                continue
            if c["yhi"] >= ylo and c["ylo"] <= py:
                clear = False
                break
        api.log("CAND dx=%+.3f cx=%.3f rim_y=%.3f start_y=%.3f clear=%s" % (dx, cx, rim_y, start_y, clear))
        if clear and best is None:
            best = (cx, start_y)
    if best is None:
        best = (px + 0.045, py - max(pr * pr - 0.045 ** 2, 4e-4) ** 0.5 - APPROACH_GAP)
        api.log("CAND none clear -> forced %s" % (np.round(best, 4).tolist(),))
    return best


def goto_precise(api, name, xyz, seconds=2.0, iters=3, tol=0.004):
    """api.move lands with a systematic per-axis bias (v7: a descent commanded
    to 0.945 settles at 0.952).  Fold the observed error back into the command."""
    tgt = np.asarray(xyz, float)
    cmd = tgt.copy()
    e = np.asarray(api.eef(), float)
    for i in range(iters):
        api.move(cmd.tolist(), seconds=seconds)
        e = np.asarray(api.eef(), float)
        err = e - tgt
        api.log("PMOVE %s i=%d cmd=%s eef=%s err=%s |err|=%.4f"
                % (name, i, np.round(cmd, 4).tolist(), np.round(e, 4).tolist(),
                   np.round(err, 4).tolist(), float(np.linalg.norm(err))))
        if float(np.linalg.norm(err)) <= tol:
            break
        cmd = cmd - err
        seconds = 1.0
    return e


def strike(api, cx, s_y, e_y, cz, safe_z, tag):
    """One edge strike.  Every approach and retreat is flown at safe_z, which
    clears the tallest prop on the table (the wine bottle, top 0.158 above the
    table): v12 lost two seeds to a waypoint at y=-0.04 that wedged the arm
    against the bottle and froze it for the rest of the episode."""
    goto_precise(api, tag + "_hi", [cx, s_y, safe_z], seconds=3.0, iters=2, tol=0.006)
    goto_precise(api, tag + "_dn", [cx, s_y, cz], seconds=2.0, iters=3, tol=0.0025)
    n = max(2, int(round(abs(e_y - s_y) / 0.022)))
    ey = s_y
    for i in range(1, n + 1):
        ey = s_y + (e_y - s_y) * i / n
        api.move([cx, ey, cz], seconds=1.0)
    e = np.asarray(api.eef(), float)
    api.log("%s swept_to=%s" % (tag, np.round(e, 4).tolist()))
    api.move([cx, ey, safe_z], seconds=2.0)
    api.move([cx, s_y - 0.13, safe_z], seconds=2.0)


def look(api, tag):
    cs, tzz, _ = perceive(api, tag)
    pl, st = pick(cs, tzz)
    return pl, tzz


def run(api):
    """v14 -- v13 with the dead constants and helpers of earlier versions
    removed, so PROVENANCE covers exactly the constants the program uses.
    Behaviour is identical to v13 (15/15 on debug seeds 51-65).

    v13 rationale:

    v12 lost seeds 53 and 55 to a single pre-positioning waypoint at
    y = plate_y - r - 0.10 ~ -0.040, which put the hand into the wine bottle
    (y -0.067..-0.024, top 0.158 above the table) at a transit height of
    table+0.14, BELOW the bottle's top.  The arm wedged there on the first move
    and every later command was a no-op.  The two seeds whose plate sat 0.014 m
    further +y cleared the bottle and both succeeded.  v13 drops that waypoint
    and flies all transits at table+0.22, above the bottle.

    v12 rationale:

    v11 settled the calibration: the fingertip sits 0.0080 m below the eef
    origin, not the 0.0312 m inferred from the earlier unconverged descents.
    Every strike from v3 to v10 therefore swept 25-35 mm above the plate, which
    is why no tip height ever transferred force.  Same ladder, real heights.

    Original v10 rationale:

    v6 showed a blade inside the dish does nothing: the saucer's interior is a
    27-degree ramp, so a horizontal push there becomes downforce and the blade
    rides over.  Only the steep OUTER edge transfers force, and v5 vs v7 put
    that band inside 1-2 mm (tip at table+0.0133 moved the plate 0.023 m, tip at
    table+0.0143 moved it not at all).  api.move alone cannot hold 1 mm, but
    folding the observed error back into the command (goto_precise, v9) holds
    ~0.002 m.  So: walk a fine ladder of tip heights, keep the first that moves
    the plate, and strike with it until the plate is aligned with the stove.
    """
    api.log("INSTR %r  [v14 edge strike, TIP_OFF=%.4f]" % (api.instruction(), TIP_OFF))
    comps, table_z, d0 = perceive(api, "HIGH", dump=True)
    plate, stove = pick(comps, table_z)
    if plate is None or stove is None:
        api.log("ABORT plate=%s stove=%s" % (plate, stove))
        return
    px, py = centre(plate)
    pr = radius(plate)
    sx, sy = centre(stove)
    api.log("GEO plate=(%.3f,%.3f) r=%.3f stove=(%.3f,%.3f) xfront=%.3f table_z=%.4f"
            % (px, py, pr, sx, sy, stove["xhi"], table_z))

    api.grip(0.0)
    safe_z = table_z + SAFE_H
    cx, _ = choose_contact(api, comps, plate, px, py, pr)

    h_best = None
    ladder = list(LADDER_H)
    for k in range(MAX_STRIKES):
        if py >= sy - ALIGN_TOL:
            api.log("ALIGNED py=%.4f sy=%.4f after %d strikes" % (py, sy, k))
            break
        dx = cx - px
        inner = max(pr * pr - dx * dx, 4e-4) ** 0.5
        s_y = py - inner - APPROACH_GAP
        e_y = s_y + (sy - py) + OVERSHOOT
        if h_best is None:
            if not ladder:
                api.log("LADDER exhausted, no contact height found")
                break
            h = ladder.pop(0)
        else:
            h = h_best
        cz = table_z + h + TIP_OFF
        api.log("STRIKE %d h=%.4f cz=%.4f cx=%.4f s_y=%.4f e_y=%.4f py=%.4f"
                % (k, h, cz, cx, s_y, e_y, py))
        strike(api, cx, s_y, e_y, cz, safe_z, "s%d" % k)
        pl, tzz = look(api, "CHK%d" % k)
        if pl is None:
            api.log("CHK%d plate not seen" % k)
            continue
        nx, ny = centre(pl)
        api.log("CHK%d plate=(%.4f,%.4f) delta=(%+.4f,%+.4f) h=%.4f" % (k, nx, ny, nx - px, ny - py, h))
        if ny - py > MOVED_TOL:
            if h_best is None:
                api.log("CONTACT HEIGHT h=%.4f (moved %+.4f)" % (h, ny - py))
            h_best = h
            px, py, pr = nx, ny, radius(pl)
            cxn, _ = choose_contact(api, [pl], pl, px, py, pr)
            cx = cxn
        else:
            api.log("no transfer at h=%.4f" % h)
            if h_best is not None:
                h_best = None          # the band moved; re-search
            px, py = nx, ny

    goto(api, "park", [px + 0.16, py - 0.28, safe_z], seconds=3.0, tries=1)
    api.settle(0.4)
    comps2, tz2, _ = perceive(api, "HIGH2", dump=True)
    p2, _ = pick(comps2, tz2)
    if p2 is not None:
        fx, fy = centre(p2)
        api.log("FINAL plate=(%.4f,%.4f) stove_y=%.4f dy_remaining=%+.4f" % (fx, fy, sy, sy - fy))
    else:
        api.log("FINAL plate=None")
