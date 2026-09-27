"""spa_bowl_between_task_k3 -- v3.

Intent: "Pick the akita black bowl NOT between the plate and the ramekin and
place it on the plate".  The pack's own language names the *between* bowl, so
the pack is an ANTI-pack: it supplies the mechanism, the instruction supplies
the target.

v3 fixes the two v2 perception bugs:
  * the cabinet fragmented because the band was capped at 1.10 m -> band the
    whole column and drop components by ztop instead;
  * bowl A fused with the plate (0.13 m apart, radii 0.055+0.068) -> vessels
    are found in a HIGH band (z>0.932, above both flat props) and the plate is
    found in a LOW band with the vessel footprints punched out.
Plus closed-loop moves (api.move undershoots ~8-16 mm) and a grasp check.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "TABLE_Z": {
        "source": "debug seeds 51-65 cam_high depth: modal base-frame z of the "
                  "table surface (~30k px in the 0.895-0.905 bin)",
        "allowed": True},
    "LOW_BAND / HIGH_BAND": {
        "source": "debug-seed cluster tops: plate 0.920, cookie box 0.920, "
                  "ramekin 0.943, both bowls 0.951; 0.932 separates them",
        "allowed": True},
    "MAX_PROP_Z": {
        "source": "debug seeds: every table prop tops out <=0.955 while the "
                  "cabinet/stove is 1.128 and the robot column 1.357",
        "allowed": True},
    "GRASP_Z": {
        "source": "pack ee_path6 lowest point of the grasp descent: demo0 t=40 "
                  "0.9301, demo1 t=40 0.9208, demo2 t=40 0.9187",
        "allowed": True},
    "CARRY_Z": {
        "source": "pack ee_path6 mid-carry apex: 1.0373 / 1.0533 / 1.0193",
        "allowed": True},
    "RELEASE_Z": {
        "source": "pack keyframes at the release (gripper_cmd -1 while the "
                  "gripper is still closed): 0.9367 / 0.9360 / 0.9269",
        "allowed": True},
    "RIM_OFFSET": {
        "source": "pack: demo grasp xy sits ~0.05 m (one rim radius) in -y from "
                  "the between-bowl centre recovered from the demo t0000 "
                  "keyframes; closed gripper gap 0.0128 m == a pinched wall",
        "allowed": True},
    "JAW_AXIS": {
        "source": "debug-seed api.tool_rotation() at home ~= diag(1,-1,-1) plus "
                  "generic parallel-jaw mechanics (fingers translate on tool y)",
        "allowed": True},
    "OPEN_HALF": {
        "source": "debug-seed api.gripper() after api.grip(0.078): width 0.0778, "
                  "so each finger stands ~0.039 m off the tool axis",
        "allowed": True},
    "BOX_CHROMA": {
        "source": "debug seeds: cookie-box cluster mean rgb (0.37,0.26,0.18), "
                  "r-b = 0.18; plate 0.055, vessels < 0.02",
        "allowed": True},
    "BOWL_BAND": {
        "source": "debug seeds 51-65: ramekin ztop 0.943-0.944 on every seed, "
                  "both bowls 0.951-0.952; 0.9465 sits mid-gap.  Needed because "
                  "on seed 65 the ramekin abuts a bowl with an ~11 mm gap and "
                  "the footprint grid fuses them.",
        "allowed": True},
    "BOWL_R / BOWL_MAX_W": {
        "source": "debug seeds 51-65: every cleanly separated bowl footprint "
                  "spans 0.109-0.112 m in both x and y (the two bowls are the "
                  "same asset), so r = 0.0553 and anything wider than 0.128 m "
                  "is a fused pair",
        "allowed": True},
    "PLATE_MIN_VALUE": {
        "source": "debug seeds: plate cluster mean value 0.55-0.58; every "
                  "cabinet/stove fragment in the same height band is 0.22-0.28",
        "allowed": True},
    "TALL_CELL": {
        "source": "generic: cell size of the XY mask that removes fixtures the "
                  "low band would otherwise clip into a fake flat prop "
                  "(v3 debug seeds produced exactly that at (-0.24,-0.12))",
        "allowed": True},
    "RAMEKIN_WIDTH": {
        "source": "debug seeds: ramekin spans 0.088 m, both bowls 0.109-0.112 m",
        "allowed": True},
    "MOVE_BIAS_TOL": {
        "source": "v2 debug seeds: api.move settles 8-18 mm short of the "
                  "commanded pose (e.g. cmd z 0.923 -> eef 0.9313/0.9395)",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi contract (effort 3.0 iff holding) confirmed on the "
                  "debug seeds: 0.05 whenever the jaws closed on air",
        "allowed": True},
}

TABLE_Z = 0.900
LOW_BAND = 0.912
HIGH_BAND = 0.932
BOWL_BAND = 0.9465
BOWL_R = 0.0553
BOWL_MAX_W = 0.128
MAX_PROP_Z = 0.99
TALL_CELL = 0.012
PLATE_MIN_VALUE = 0.42
GRASP_Z = 0.923
CARRY_Z = 1.035
RELEASE_Z = 0.938
OPEN_W = 0.078
OPEN_HALF = 0.039
CLOSE_W = 0.010
HOLD_EFFORT = 1.0

CHUNK = 1800


# ---------------------------------------------------------------- logging ---
def _blob(api, tag, arr):
    b = base64.b64encode(zlib.compress(np.ascontiguousarray(arr).tobytes(), 9)).decode()
    n = (len(b) + CHUNK - 1) // CHUNK
    api.log("%s NCHUNK=%d dtype=%s shape=(%s)"
            % (tag, n, arr.dtype, ",".join(str(s) for s in arr.shape)))
    for i in range(n):
        api.log("%s %d %s" % (tag, i, b[i * CHUNK:(i + 1) * CHUNK]))


def dump(api, tag, f=None):
    f = f if f is not None else api.capture("cam_high")
    _blob(api, "RGB_" + tag, np.ascontiguousarray(f.rgb.astype(np.uint8)[::2, ::2]))
    _blob(api, "DEP_" + tag, np.ascontiguousarray(
        (np.nan_to_num(f.depth, nan=0.0) * 1000.0).clip(0, 65535)
        .astype(np.uint16)[::2, ::2]))
    api.log("K_%s=%r" % (tag, np.asarray(f.intrinsics).tolist()))
    api.log("T_%s=%r" % (tag, np.asarray(f.t_base_cam).tolist()))
    return f


# ------------------------------------------------------------- perception ---
def cloud(f):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.asarray(f.depth, float)
    rgb = np.asarray(f.rgb, float) / 255.0
    H, W = d.shape
    v, u = np.mgrid[0:H, 0:W]
    ok = np.isfinite(d) & (d > 0.1) & (d < 5.0)
    z = d[ok]
    x = (u[ok] - K[0, 2]) / K[0, 0] * z
    y = (v[ok] - K[1, 2]) / K[1, 1] * z
    P = np.stack([x, y, z], 1) @ T[:3, :3].T + T[:3, 3]
    return P, rgb[ok]


def grid_label(P, C, cell=0.006, xr=(-0.32, 0.32), yr=(-0.42, 0.42), nmin=60):
    """Connected components on a top-down base-XY occupancy grid.  Image-space
    adjacency fuses props that occlude one another; base XY does not."""
    if len(P) == 0:
        return []
    ni = int(np.ceil((xr[1] - xr[0]) / cell)) + 2
    nj = int(np.ceil((yr[1] - yr[0]) / cell)) + 2
    gi = np.clip(np.floor((P[:, 0] - xr[0]) / cell).astype(int), 0, ni - 1)
    gj = np.clip(np.floor((P[:, 1] - yr[0]) / cell).astype(int), 0, nj - 1)
    occ = np.zeros((ni, nj), bool)
    occ[gi, gj] = True
    lab = np.zeros((ni, nj), int)
    cur = 0
    for si in range(ni):
        for sj in range(nj):
            if not occ[si, sj] or lab[si, sj]:
                continue
            cur += 1
            lab[si, sj] = cur
            stack = [(si, sj)]
            while stack:
                a, b = stack.pop()
                for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    p, q = a + da, b + db
                    if 0 <= p < ni and 0 <= q < nj and occ[p, q] and not lab[p, q]:
                        lab[p, q] = cur
                        stack.append((p, q))
    pl = lab[gi, gj]
    out = []
    for k in range(1, cur + 1):
        s = pl == k
        if s.sum() < nmin:
            continue
        p, c = P[s], C[s]
        out.append(dict(
            n=int(s.sum()),
            cx=float((p[:, 0].min() + p[:, 0].max()) / 2),
            cy=float((p[:, 1].min() + p[:, 1].max()) / 2),
            sx=float(p[:, 0].max() - p[:, 0].min()),
            sy=float(p[:, 1].max() - p[:, 1].min()),
            ztop=float(np.percentile(p[:, 2], 97)),
            bb=(float(p[:, 0].min()), float(p[:, 0].max()),
                float(p[:, 1].min()), float(p[:, 1].max())),
            rgb=[float(v) for v in c.mean(0)]))
    out.sort(key=lambda d: -d["n"])
    return out


def circle_vote(xy, r, cell=0.004, nang=72, k=2, minsep=0.085):
    """Hough vote for circles of a known radius.  Two abutting bowls merge into
    one footprint component but still contribute two distinct rim rings."""
    if len(xy) < 30:
        return []
    x0, y0 = xy[:, 0].min() - r - cell, xy[:, 1].min() - r - cell
    ni = int((xy[:, 0].max() - xy[:, 0].min() + 2 * r) / cell) + 3
    nj = int((xy[:, 1].max() - xy[:, 1].min() + 2 * r) / cell) + 3
    acc = np.zeros((ni, nj), np.int32)
    ang = np.arange(nang) * (2 * np.pi / nang)
    cs, sn = r * np.cos(ang), r * np.sin(ang)
    gi = np.clip(((xy[:, 0:1] + cs[None, :]) - x0) / cell, 0, ni - 1).astype(int)
    gj = np.clip(((xy[:, 1:2] + sn[None, :]) - y0) / cell, 0, nj - 1).astype(int)
    np.add.at(acc, (gi.ravel(), gj.ravel()), 1)
    out = []
    for _ in range(k):
        idx = int(np.argmax(acc))
        a, b = idx // nj, idx % nj
        if acc[a, b] < 20:
            break
        cx, cy = x0 + (a + 0.5) * cell, y0 + (b + 0.5) * cell
        out.append((float(acc[a, b]), cx, cy))
        ii, jj = np.mgrid[0:ni, 0:nj]
        acc[((ii - a) * cell) ** 2 + ((jj - b) * cell) ** 2 < minsep ** 2] = 0
    return out


def band(P, C, lo, hi):
    m = ((P[:, 2] > lo) & (P[:, 2] < hi)
         & (P[:, 0] > -0.32) & (P[:, 0] < 0.32)
         & (P[:, 1] > -0.42) & (P[:, 1] < 0.42))
    return P[m], C[m]


def seg2line(p, a, b):
    p, a, b = np.asarray(p, float), np.asarray(a, float), np.asarray(b, float)
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-9), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def perceive(api, tag="t0"):
    f = dump(api, tag)
    P, C = cloud(f)

    # -- bowls: the only things standing above the ramekin --------------------
    # The ramekin tops out 8 mm below the bowls (0.943-0.944 vs 0.951-0.952 on
    # every debug seed), and on some seeds it abuts a bowl with an 11 mm gap --
    # closer than the footprint grid can resolve.  So take the bowls from a TOP
    # band that the ramekin does not reach at all.
    tp, tc = band(P, C, BOWL_BAND, 3.0)
    bowls = [d for d in grid_label(tp, tc, nmin=40) if d["ztop"] < MAX_PROP_Z]
    for d in bowls:
        api.log("TOPVES n=%d c=(%.3f,%.3f) s=(%.3f,%.3f) ztop=%.3f"
                % (d["n"], d["cx"], d["cy"], d["sx"], d["sy"], d["ztop"]))

    # -- vessels (bowls + ramekin) in the wider band, for the ramekin ---------
    hp, hc = band(P, C, HIGH_BAND, 3.0)
    ves = [d for d in grid_label(hp, hc) if d["ztop"] < MAX_PROP_Z]
    for d in ves:
        api.log("VES n=%d c=(%.3f,%.3f) s=(%.3f,%.3f) ztop=%.3f rgb=%s"
                % (d["n"], d["cx"], d["cy"], d["sx"], d["sy"], d["ztop"],
                   [round(v, 3) for v in d["rgb"]]))

    # Two bowls can abut and fuse into one footprint component, so the rim
    # centres come from a Hough vote for two rings of the known bowl radius.
    # The vote is restricted to the table-prop components -- the cabinet and
    # the robot column also reach into this band and would dominate the poll.
    sel = np.zeros(len(tp), bool)
    for d in bowls:
        a, b, c2, d2 = d["bb"]
        sel |= ((tp[:, 0] >= a - 0.005) & (tp[:, 0] <= b + 0.005)
                & (tp[:, 1] >= c2 - 0.005) & (tp[:, 1] <= d2 + 0.005))
    votes = circle_vote(tp[sel][:, :2], BOWL_R)
    api.log("VOTE %s" % [(int(v), round(x, 3), round(y, 3)) for v, x, y in votes])
    if len(votes) == 2 and np.hypot(votes[0][1] - votes[1][1],
                                    votes[0][2] - votes[1][2]) > 0.09:
        bowls = [dict(n=int(v), cx=x, cy=y, sx=2 * BOWL_R, sy=2 * BOWL_R,
                      ztop=0.952, bb=(x - BOWL_R, x + BOWL_R, y - BOWL_R, y + BOWL_R),
                      rgb=[0.4, 0.4, 0.4]) for v, x, y in votes]
        api.log("BOWLS from circle vote")
    else:
        bowls = sorted(bowls or ves, key=lambda d: -max(d["sx"], d["sy"]))[:2]
        api.log("BOWLS from fallback band")
    bowls = bowls[:2]

    # ramekin: whatever is left in the vessel band once the bowls are punched
    hkeep = np.ones(len(hp), bool)
    for d in bowls:
        r = max(d["sx"], d["sy"]) / 2.0 + 0.008
        hkeep &= ((hp[:, 0] - d["cx"]) ** 2 + (hp[:, 1] - d["cy"]) ** 2) > r * r
    rem = [d for d in grid_label(hp[hkeep], hc[hkeep], nmin=80)
           if d["ztop"] < MAX_PROP_Z]
    for d in rem:
        api.log("REM n=%d c=(%.3f,%.3f) s=(%.3f,%.3f) ztop=%.3f"
                % (d["n"], d["cx"], d["cy"], d["sx"], d["sy"], d["ztop"]))

    # -- flat props ----------------------------------------------------------
    # A tall structure (cabinet, stove, robot) is CLIPPED by the low band and
    # would masquerade as a flat prop, so punch out every XY cell that carries
    # any point above the tallest table prop -- and the vessel footprints too.
    lp, lc = band(P, C, LOW_BAND, HIGH_BAND)
    keep = np.ones(len(lp), bool)
    for d in ves:
        r = max(d["sx"], d["sy"]) / 2.0 + 0.012
        keep &= ((lp[:, 0] - d["cx"]) ** 2 + (lp[:, 1] - d["cy"]) ** 2) > r * r
    tall = P[P[:, 2] > MAX_PROP_Z]
    if len(tall):
        ti = np.floor(tall[:, 0] / TALL_CELL).astype(int)
        tj = np.floor(tall[:, 1] / TALL_CELL).astype(int)
        tallset = set(zip(ti.tolist(), tj.tolist()))
        li = np.floor(lp[:, 0] / TALL_CELL).astype(int)
        lj = np.floor(lp[:, 1] / TALL_CELL).astype(int)
        near = np.array([any((a + da, b + db) in tallset
                             for da in (-1, 0, 1) for db in (-1, 0, 1))
                         for a, b in zip(li.tolist(), lj.tolist())])
        keep &= ~near
    flats = [d for d in grid_label(lp[keep], lc[keep], nmin=150)
             if d["ztop"] < MAX_PROP_Z]
    for d in flats:
        api.log("FLAT n=%d c=(%.3f,%.3f) s=(%.3f,%.3f) ztop=%.3f rgb=%s"
                % (d["n"], d["cx"], d["cy"], d["sx"], d["sy"], d["ztop"],
                   [round(v, 3) for v in d["rgb"]]))

    # plate: pale (the cookie box is warm, every fixture surface is dark grey)
    cand = [d for d in flats
            if (d["rgb"][0] - d["rgb"][2]) < 0.12
            and sum(d["rgb"]) / 3.0 > PLATE_MIN_VALUE]
    if not cand:
        cand = [d for d in flats if (d["rgb"][0] - d["rgb"][2]) < 0.12] or flats
    cand.sort(key=lambda d: -max(d["sx"], d["sy"]))
    plate = cand[0]

    if rem:
        ramekin = max(rem, key=lambda d: d["n"])
    else:
        ramekin = min(ves, key=lambda d: max(d["sx"], d["sy"]))
        api.log("RAMEKIN from fallback (narrowest vessel)")

    scored = []
    for b in bowls:
        d = seg2line((b["cx"], b["cy"]), (plate["cx"], plate["cy"]),
                     (ramekin["cx"], ramekin["cy"]))
        scored.append((d, b))
        api.log("BOWL c=(%.3f,%.3f) w=%.3f dist_to_seg=%.3f"
                % (b["cx"], b["cy"], max(b["sx"], b["sy"]), d))
    scored.sort(key=lambda t: -t[0])
    target = scored[0][1]
    other = scored[1][1] if len(scored) > 1 else ramekin
    api.log("PLATE c=(%.3f,%.3f) w=%.3f | RAMEKIN c=(%.3f,%.3f) w=%.3f"
            % (plate["cx"], plate["cy"], max(plate["sx"], plate["sy"]),
               ramekin["cx"], ramekin["cy"], max(ramekin["sx"], ramekin["sy"])))
    api.log("TARGET c=(%.3f,%.3f) d=%.3f  (excluded d=%.3f)"
            % (target["cx"], target["cy"], scored[0][0],
               scored[1][0] if len(scored) > 1 else -1.0))
    return plate, target, ramekin, other


# ------------------------------------------------------------------ motion ---
def goto(api, xyz, seconds=2.5, tol=0.004, tries=3, tag=""):
    """api.move settles short of the command; re-issue with the observed error
    folded back in (bounded, so a saturated axis cannot run the command away)."""
    tgt = np.asarray(xyz, float)
    cmd = tgt.copy()
    for i in range(tries):
        api.move(cmd.tolist(), seconds=seconds)
        got = np.asarray(api.eef(), float)
        err = tgt - got
        api.log("GOTO%s try%d cmd=%s got=%s err=%s"
                % (tag, i, np.round(cmd, 4).tolist(), np.round(got, 4).tolist(),
                   np.round(err, 4).tolist()))
        if np.max(np.abs(err)) <= tol:
            break
        cmd = np.clip(cmd + err, tgt - 0.06, tgt + 0.06)
    return np.asarray(api.eef(), float)


def run(api):
    api.log("INSTRUCTION: " + str(api.instruction()))
    plate, target, ramekin, other = perceive(api)
    rim_r = BOWL_R

    # straddle whichever rim side has room for the outer finger
    obstacles = [(d["cx"], d["cy"], max(d["sx"], d["sy"]) / 2.0)
                 for d in (plate, ramekin, other)]
    best, bestclr = -1.0, None
    for sgn in (-1.0, +1.0):
        fy = target["cy"] + sgn * (rim_r + OPEN_HALF)
        clr = min(np.hypot(target["cx"] - ox, fy - oy) - orad
                  for ox, oy, orad in obstacles)
        api.log("SIDE sgn=%+d outer_finger_y=%.3f clearance=%.3f" % (sgn, fy, clr))
        if clr > best:
            best, bestclr = clr, sgn
    sgn = bestclr
    api.log("RIM_R=%.4f SIDE=%+d" % (rim_r, sgn))

    gx = target["cx"]
    # up to three bites: nominal rim, then 6 mm further out / further in.  A
    # bite is judged by api.gripper(): effort 3.0 iff something is held, and a
    # pinched wall leaves a finite gap (the demos' was 0.0128 m).
    held = False
    for k, dr in enumerate((0.0, 0.006, -0.006)):
        gy = target["cy"] + sgn * (rim_r + dr)
        api.grip(OPEN_W)
        goto(api, [gx, gy, CARRY_Z], seconds=2.5, tries=2, tag="/hover%d" % k)
        goto(api, [gx, gy, GRASP_Z], seconds=2.5, tries=2, tag="/descend%d" % k)
        api.grip(CLOSE_W)
        api.settle(0.3)
        g = api.gripper()
        api.log("GRIP bite%d dr=%+0.3f -> %r" % (k, dr, g))
        if g["effort"] >= HOLD_EFFORT and g["width_m"] > 0.003:
            held = True
            break
    api.log("HELD=%s" % held)

    # bowl centre trails the eef by one rim radius, on the straddled side
    px = plate["cx"]
    py = plate["cy"] + sgn * rim_r
    # the jaws keep squeezing, so spend as few steps as possible while holding
    goto(api, [gx, gy, CARRY_Z], seconds=2.0, tries=1, tag="/lift")
    api.log("GRIP lifted=%r" % (api.gripper(),))
    goto(api, [px, py, CARRY_Z], seconds=2.5, tries=2, tag="/carry")
    api.log("GRIP carried=%r" % (api.gripper(),))
    goto(api, [px, py, RELEASE_Z], seconds=2.0, tries=2, tag="/place")
    api.log("GRIP placed=%r" % (api.gripper(),))
    api.grip(OPEN_W)
    api.settle(0.5)
    goto(api, [px, py, CARRY_Z], seconds=2.0, tries=1, tag="/retreat")
    api.settle(0.5)
    dump(api, "end")
