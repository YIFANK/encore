"""v3 -- perceive the scene, rim-pinch the 'between' bowl, place it centred on the plate.

Perception (all re-derived from debug seeds 51-65 + this pack):
  * deproject cam_high depth to base frame; table height = median z in the workspace.
  * "tall" band  z > tz+0.033 isolates ramekin + both bowls (they fuse lower down).
  * bowls: ztop > tz+0.047 and footprint > 0.090 m ; ramekin: ztop in (tz+0.036,tz+0.047].
  * plate: flat band z in (tz+0.012, tz+0.030), grey (R-B < 25), footprint > 0.10 m.
  * target = bowl closest to the plate->ramekin segment (the 'between' predicate).
Grasp: jaws close along world y, so pinch the near (-y) rim wall.
"""
import numpy as np

PROVENANCE = {
    "BOX": {"source": "debug-seed cam_high clouds 51-65: props all lie in x(-0.40,0.35) y(-0.02,0.42)", "allowed": True},
    "TALL_DZ": {"source": "debug seeds: ramekin/bowl footprints separate above table+0.033 m", "allowed": True},
    "BOWL_ZTOP_DZ": {"source": "debug seeds: bowl rims at tz+0.0506..0.0511, ramekin top tz+0.0426", "allowed": True},
    "BOWL_MIN_D": {"source": "debug seeds: bowl footprint 0.107-0.111 m, ramekin 0.084-0.088 m", "allowed": True},
    "RAM_D": {"source": "debug seeds: ramekin footprint 0.084-0.088 m", "allowed": True},
    "FLAT_LO/FLAT_HI": {"source": "debug seeds: plate top tz+0.0187, cookie box top tz+0.0194", "allowed": True},
    "WARM_MAX": {"source": "debug seeds: plate mean R-B = 13-14, cookie box 43-48", "allowed": True},
    "PLATE_MIN_D": {"source": "debug seeds: plate footprint 0.133-0.136 m, cookie box 0.064-0.082 m", "allowed": True},
    "GRASP_DX": {"source": "pack demos: grasp ee x minus target-bowl centre from keyframe pixel deprojection (-0.001,-0.031,-0.023)", "allowed": True},
    "GRASP_DY": {"source": "pack demos: grasp ee y minus bowl centre (-0.025,-0.039,-0.046) -> near-rim pinch", "allowed": True},
    "GRASP_DZ": {"source": "pack demos: grasp ee z minus bowl rim height (-0.007,-0.027,-0.033)", "allowed": True},
    "PLACE_DZ": {"source": "pack demos: release ee z minus grasp ee z (-0.008,+0.012,+0.008)", "allowed": True},
    "HOVER": {"source": "generic controller mechanics: clear the tallest prop before lateral motion", "allowed": True},
    "RETRY_DDY/RETRY_DDZ": {"source": "generic recovery: nudge the pinch inboard and lower after empty jaws", "allowed": True},
    "HOLD_W": {"source": "pack demos: gripper_state sum at hold 0.0125/0.0122/0.0130 vs 0.0724 open", "allowed": True},
}

BOX = (-0.40, 0.35, -0.02, 0.42)
TALL_DZ = 0.033
BOWL_ZTOP_DZ = 0.047
BOWL_MIN_D = 0.090
RAM_D = (0.055, 0.095)
FLAT_LO, FLAT_HI = 0.012, 0.030
WARM_MAX = 25.0
PLATE_MIN_D = 0.10
GRASP_DX = -0.010
GRASP_DY = -0.037
GRASP_DZ = -0.025
PLACE_DZ = 0.004
RETRY_DDY = 0.008
RETRY_DDZ = -0.006
TARGET_RANK = 0   # 0 = best "between" bowl, 1 = the runner-up twin
HOVER = 0.085
HOLD_W = 0.030


# ---------------------------------------------------------------- perception
def _label(m, minpix):
    h, w = m.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    out = []
    for i0, j0 in np.argwhere(m):
        if lab[i0, j0]:
            continue
        cur += 1
        st = [(int(i0), int(j0))]
        lab[i0, j0] = cur
        pts = []
        while st:
            a, b = st.pop()
            pts.append((a, b))
            for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                p, q = a + da, b + db
                if 0 <= p < h and 0 <= q < w and m[p, q] and not lab[p, q]:
                    lab[p, q] = cur
                    st.append((p, q))
        if len(pts) >= minpix:
            out.append(np.array(pts))
    return out


def _kasa(x, y):
    A = np.stack([x, y, np.ones_like(x)], 1)
    s, *_ = np.linalg.lstsq(A, x * x + y * y, rcond=None)
    cx, cy = s[0] / 2.0, s[1] / 2.0
    return float(cx), float(cy), float(np.sqrt(max(s[2] + cx * cx + cy * cy, 1e-9)))


def _describe(P, rgb, pts):
    p = P[pts[:, 0], pts[:, 1]]
    c = rgb[pts[:, 0], pts[:, 1]].astype(float).mean(0)
    cx, cy, r = _kasa(p[:, 0], p[:, 1])
    return dict(n=len(pts), ztop=float(np.percentile(p[:, 2], 98)),
                dx=float(p[:, 0].max() - p[:, 0].min()),
                dy=float(p[:, 1].max() - p[:, 1].min()),
                ymax=float(p[:, 1].max()), xmid=float(0.5 * (p[:, 0].max() + p[:, 0].min())),
                cx=cx, cy=cy, r=r, warm=float(c[0] - c[2]))


def cloud(frame, step=2):
    d = np.asarray(frame.depth, dtype=np.float64)[::step, ::step]
    K = np.asarray(frame.intrinsics, dtype=np.float64)
    T = np.asarray(frame.t_base_cam, dtype=np.float64)
    h, w = d.shape
    jj, ii = np.meshgrid(np.arange(w) * step, np.arange(h) * step)
    pts = np.stack([(jj - K[0, 2]) / K[0, 0] * d, (ii - K[1, 2]) / K[1, 1] * d, d], -1)
    return pts @ T[:3, :3].T + T[:3, 3]


def perceive(api):
    f = api.capture("cam_high")
    P = cloud(f)
    rgb = np.asarray(f.rgb)[::2, ::2]
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    box = (X > BOX[0]) & (X < BOX[1]) & (Y > BOX[2]) & (Y < BOX[3]) & np.isfinite(Z)
    tz = float(np.median(Z[box & (Z > 0.80) & (Z < 1.05)]))
    tall = [_describe(P, rgb, q) for q in
            _label(box & (Z > tz + TALL_DZ) & (Z < tz + 0.09), 50)]
    bowls = [c for c in tall if c['ztop'] > tz + BOWL_ZTOP_DZ and max(c['dx'], c['dy']) > BOWL_MIN_D]
    rams = [c for c in tall if tz + 0.036 < c['ztop'] <= tz + BOWL_ZTOP_DZ
            and RAM_D[0] < max(c['dx'], c['dy']) < RAM_D[1]]
    low = box & (Z > tz + FLAT_LO) & (Z < tz + FLAT_HI)
    for b in bowls:
        low &= ~((X - b['cx']) ** 2 + (Y - b['cy']) ** 2 < (b['r'] + 0.022) ** 2)
    flats = [_describe(P, rgb, q) for q in _label(low, 150)]
    plates = [c for c in flats if c['warm'] < WARM_MAX and max(c['dx'], c['dy']) > PLATE_MIN_D]
    api.log("PERCEIVE tz=%.4f nbowl=%d nram=%d nplate=%d" % (tz, len(bowls), len(rams), len(plates)))
    for c in tall + flats:
        api.log("  blob n=%3d c=(%.3f,%.3f) r=%.4f d=(%.3f,%.3f) ztop=%.4f warm=%.0f"
                % (c['n'], c['cx'], c['cy'], c['r'], c['dx'], c['dy'], c['ztop'], c['warm']))
    return tz, bowls, rams, plates, tall, flats


def pick_between(bowls, plate, ram, api, rank=0):
    p = np.array([plate['cx'], plate['cy']])
    r = np.array([ram['cx'], ram['cy']])
    u = (r - p) / np.linalg.norm(r - p)
    L = float(np.linalg.norm(r - p))
    scored = []
    for b in bowls:
        q = np.array([b['cx'], b['cy']]) - p
        t = float(q @ u)
        perp = float(np.linalg.norm(q - t * u))
        s = perp + 2.0 * (max(0.0, -t) + max(0.0, t - L))
        api.log("  cand bowl c=(%.3f,%.3f) t=%.2f perp=%.3f score=%.3f" % (b['cx'], b['cy'], t / L, perp, s))
        scored.append((s, b))
    scored.sort(key=lambda sb: sb[0])
    return scored[min(rank, len(scored) - 1)][1]


# ---------------------------------------------------------------------- run
def run(api):
    api.log("INSTRUCTION %r" % (api.instruction(),))
    tz, bowls, rams, plates, tall, flats = perceive(api)
    # fail-open fallbacks: these branches only fire where the strict classifier
    # found nothing, i.e. where the alternative is doing nothing at all.
    if not bowls:
        bowls = [c for c in tall if c['ztop'] > tz + 0.045]
        api.log("FALLBACK bowls -> %d" % len(bowls))
    if not rams:
        rams = [c for c in tall if c['ztop'] <= tz + BOWL_ZTOP_DZ] or \
               [c for c in tall if c not in bowls]
        api.log("FALLBACK ram -> %d" % len(rams))
    if not plates:
        plates = [c for c in flats if c['warm'] < WARM_MAX] or flats
        api.log("FALLBACK plate -> %d" % len(plates))
    if not bowls or not rams or not plates:
        api.log("ABORT incomplete scene")
        return
    plate = max(plates, key=lambda c: c['n'])
    ram = max(rams, key=lambda c: c['n'])
    bowl = pick_between(bowls, plate, ram, api, TARGET_RANK)
    api.log("TARGET bowl c=(%.3f,%.3f) r=%.4f ztop=%.4f | plate c=(%.3f,%.3f) r=%.4f | ram c=(%.3f,%.3f)"
            % (bowl['cx'], bowl['cy'], bowl['r'], bowl['ztop'], plate['cx'], plate['cy'], plate['r'],
               ram['cx'], ram['cy']))

    rim = bowl['ztop']
    gx, gy = bowl['cx'] + GRASP_DX, bowl['cy'] + GRASP_DY
    gz = rim + GRASP_DZ
    top = rim + HOVER

    def mv(tag, xyz, sec=2.0):
        res = api.move([float(v) for v in xyz], seconds=sec)
        e = np.asarray(api.eef(), float)
        api.log("  MOVE %-9s -> (%.3f,%.3f,%.3f) eef=(%.3f,%.3f,%.3f) res=%s"
                % (tag, xyz[0], xyz[1], xyz[2], e[0], e[1], e[2], np.round(np.asarray(res, float), 4).tolist()))
        return e

    def gstate(tag):
        g = api.gripper()
        api.log("  GRIP %-9s width=%.4f effort=%.2f" % (tag, g['width_m'], g['effort']))
        return g

    api.grip(0.08)
    gstate("start")
    mv("hover", [gx, gy, top])
    e = mv("descend", [gx, gy, gz])
    base_off = float(e[2]) - tz          # eef height above the bowl's own base
    api.log("  BASE_OFF %.4f" % base_off)
    api.grip(0.0)
    api.settle(0.4)
    gstate("closed")
    mv("lift", [gx, gy, top])
    g = gstate("lifted")
    if g['width_m'] > HOLD_W:              # nothing in the jaws -- one re-try
        api.log("  RETRY grasp (empty jaws)")
        api.grip(0.08)
        tz, bowls, rams, plates, tall, flats = perceive(api)
        cand = [b for b in bowls if np.hypot(b['cx'] - bowl['cx'], b['cy'] - bowl['cy']) < 0.06]
        if cand:
            bowl = cand[0]
            rim = bowl['ztop']
        gx, gy = bowl['cx'] + GRASP_DX, bowl['cy'] + GRASP_DY + RETRY_DDY
        gz = rim + GRASP_DZ + RETRY_DDZ
        top = rim + HOVER
        mv("hover2", [gx, gy, top])
        e = mv("descend2", [gx, gy, gz])
        base_off = float(e[2]) - tz
        api.grip(0.0)
        api.settle(0.4)
        mv("lift2", [gx, gy, top])
        gstate("lifted2")

    # the bowl sits at (+GRASP_DX,+GRASP_DY) relative to the tool, so aim the tool
    # that far the other way to drop the bowl on the plate centre.
    px, py = plate['cx'] + GRASP_DX, plate['cy'] + GRASP_DY
    pz = plate['ztop'] + base_off + PLACE_DZ
    mv("carry", [px, py, top])
    e = mv("lower", [px, py, pz])
    for _ in range(2):                    # cancel the tracking bias in z
        if abs(float(e[2]) - pz) < 0.004:
            break
        e = mv("lower2", [px, py, pz - (float(e[2]) - pz)])
    gstate("atplate")
    api.grip(0.08)
    api.settle(0.5)
    mv("retreat", [px, py, top])
    gstate("done")

    tz2, bowls2, rams2, plates2, _t2, _f2 = perceive(api)
    for b in bowls2:
        api.log("POST bowl c=(%.3f,%.3f) ztop=%.4f dplate=%.3f"
                % (b['cx'], b['cy'], b['ztop'], float(np.hypot(b['cx'] - px, b['cy'] - py))))
