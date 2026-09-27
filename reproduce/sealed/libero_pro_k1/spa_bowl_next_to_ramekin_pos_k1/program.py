"""v3 -- robust height-band perception (tall-object shadow mask + bowl
masking), demo rim-pinch grasp, release referenced to the ACHIEVED grasp
offset, plus an end-of-episode re-perception for diagnostics.
"""
import numpy as np

PROVENANCE = {
    "TABLE_MODE": {"source": "generic: z-histogram mode of the cam_high cloud (= 0.9025 on every debug seed)",
                   "allowed": True},
    "H_BOWL": {"source": "debug seeds 51/53/55/57 v1 probe: bowl cluster top z = table_z+0.0488",
               "allowed": True},
    "H_RAMEKIN": {"source": "debug seeds 51/53/55/59/61/63 v1+v2 probes: ramekin top z = table_z+0.0409",
                  "allowed": True},
    "H_PLATE": {"source": "debug seed 55 v1 probe: plate top z = table_z+0.0170",
                "allowed": True},
    "R_BOWL": {"source": "debug-seed v2 probe: bowl rim-band span 0.111 m -> outer radius 0.0555",
               "allowed": True},
    "R_PLATE_SPAN": {"source": "debug seeds 53/55 v2 probe: plate band span 0.136-0.137 m",
                     "allowed": True},
    "TALL_Z": {"source": "debug-seed v1 probe: cabinet h=0.225, arm h=0.347, stove h=0.058 all "
                         "exceed table_z+0.07, while every prop of interest is below it",
               "allowed": True},
    "BROWN_REJECT": {"source": "debug-seed v1 probe: cookie box mean rgb (92,65,46) vs plate (153,142,139)",
                     "allowed": True},
    "WS_BOX": {"source": "debug-seed v1/v2 probes: all props lie in x[-0.30,0.30] y[-0.40,0.40]",
               "allowed": True},
    "GRASP_OFFSET": {"source": "pack demo0: eef at t=60 minus the demo bowl rim centre deprojected "
                               "from keyframes/demo0_t0000.png with the measured cam_high matrices",
                     "allowed": True},
    "GRASP_Z": {"source": "pack demo0 ee_path6 t=60 z=0.9225 = table_z+0.020", "allowed": True},
    "RELEASE_Z": {"source": "pack demo0 ee_path6 t=131 z=0.945 = table_z+0.043", "allowed": True},
    "GRASP_RPY": {"source": "pack demo0 ee_path6 t=60 orientation (3.0541,-0.182,0.3433)", "allowed": True},
    "HOME_RPY": {"source": "pack demo0 ee_path6 t=0 orientation, used only to pick the euler "
                           "convention that matches api.tool_rotation() at home", "allowed": True},
    "CARRY_Z": {"source": "pack demo0 ee_path6 t=80..90 carry altitude ~1.07", "allowed": True},
    "GRIP_CLOSE": {"source": "generic controller mechanics: api.grip(<0.025) closes", "allowed": True},
}

H_BOWL, H_RAMEKIN, H_PLATE = 0.0488, 0.0409, 0.0170
R_BOWL = 0.0555
TALL_Z = 0.070
GRASP_OFFSET = np.array([0.0481, 0.0274])
GRASP_DZ, RELEASE_DZ = 0.020, 0.043
CARRY_Z = 1.07
GRASP_RPY = (3.0541, -0.182, 0.3433)
HOME_RPY = (3.1325, 0.016, -0.0772)


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _conv_a(r):
    return _rz(r[2]) @ _ry(r[1]) @ _rx(r[0])


def _conv_b(r):
    return _rx(r[0]) @ _ry(r[1]) @ _rz(r[2])


def _cloud(frame):
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 0)
    z = np.where(ok, d, 1.0)
    p = np.stack([(uu - K[0, 2]) * z / K[0, 0],
                  (vv - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], -1)
    return (p @ T.T)[..., :3], ok


def _clusters(pts, cell=0.012, min_n=25):
    if len(pts) == 0:
        return []
    key = {}
    for i, c in enumerate(map(tuple, np.floor(pts[:, :2] / cell).astype(int))):
        key.setdefault(c, []).append(i)
    seen, out = set(), []
    for c in key:
        if c in seen:
            continue
        stack, comp = [c], []
        seen.add(c)
        while stack:
            k = stack.pop()
            comp.append(k)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (k[0] + dx, k[1] + dy)
                    if n in key and n not in seen:
                        seen.add(n)
                        stack.append(n)
        mem = [i for k in comp for i in key[k]]
        if len(mem) >= min_n:
            out.append(np.array(mem))
    out.sort(key=lambda g: -len(g))
    return out


def _split2(p):
    xy = p[:, :2]
    i = int(np.argmax(((xy - xy.mean(0)) ** 2).sum(1)))
    j = int(np.argmax(((xy - xy[i]) ** 2).sum(1)))
    a, b, m = xy[i], xy[j], None
    for _ in range(15):
        m = ((xy - a) ** 2).sum(1) < ((xy - b) ** 2).sum(1)
        if m.all() or (~m).all():
            break
        a, b = xy[m].mean(0), xy[~m].mean(0)
    return m


def _span(p):
    return max(np.ptp(p[:, 0]), np.ptp(p[:, 1]))


class Scene:
    pass


def perceive(api):
    f = api.capture("cam_high")
    P, ok = _cloud(f)
    Z = np.where(ok, P[..., 2], np.nan)
    zs = Z[np.isfinite(Z)]
    hist, edges = np.histogram(zs, bins=200)
    zt = float(edges[int(np.argmax(hist))] + 0.5 * (edges[1] - edges[0]))
    X, Y = P[..., 0], P[..., 1]
    ws = ok & np.isfinite(Z) & (X > -0.30) & (X < 0.30) & (Y > -0.40) & (Y < 0.40)
    rgb = np.asarray(f.rgb).astype(float)

    # --- shadow of everything taller than any prop we care about ------------
    tall = ws & (Z > zt + TALL_Z)
    tall_xy = P[tall][:, :2]
    tcells = set(map(tuple, np.floor(tall_xy / 0.02).astype(int)))
    grown = set()
    for c in tcells:
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                grown.add((c[0] + dx, c[1] + dy))

    def not_shadowed(pts):
        cc = np.floor(pts[:, :2] / 0.02).astype(int)
        return np.array([tuple(c) not in grown for c in cc])

    def band(lo, hi):
        m = ws & (Z > zt + lo) & (Z < zt + hi)
        return P[m], rgb[m]

    # --- bowls: only their rims reach this band -----------------------------
    bp, _ = band(H_BOWL - 0.005, H_BOWL + 0.006)
    if len(bp):
        keep = not_shadowed(bp)
        bp = bp[keep]
    bowls = []
    for g in _clusters(bp, 0.012, 25):
        p = bp[g]
        parts = [p]
        if _span(p) > 0.13:
            m = _split2(p)
            parts = [p[m], p[~m]]
        for q in parts:
            if len(q) > 25 and 0.06 < _span(q) < 0.135:
                bowls.append(np.array([q[:, 0].mean(), q[:, 1].mean()]))

    def near_bowl(pts, r):
        if not bowls:
            return np.zeros(len(pts), bool)
        d = np.stack([((pts[:, :2] - b) ** 2).sum(1) for b in bowls], 0)
        return (d.min(0) < r * r)

    # --- ramekin: its own rim band, with the bowls' walls masked out --------
    rp, _ = band(H_RAMEKIN - 0.006, H_RAMEKIN + 0.004)
    ram = None
    if len(rp):
        rp = rp[not_shadowed(rp) & ~near_bowl(rp, R_BOWL + 0.012)]
        best = 0
        for g in _clusters(rp, 0.012, 30):
            p = rp[g]
            if _span(p) > 0.12:
                continue
            if len(p) > best:
                best = len(p)
                ram = np.array([p[:, 0].mean(), p[:, 1].mean()])

    # --- plate: flat, wide, light, nothing above it -------------------------
    pp, pc = band(H_PLATE - 0.006, H_PLATE + 0.006)
    plate = None
    if len(pp):
        k = not_shadowed(pp) & ~near_bowl(pp, R_BOWL + 0.008)
        if ram is not None:
            k &= (((pp[:, :2] - ram) ** 2).sum(1) > 0.055 ** 2)
        pp, pc = pp[k], pc[k]
        best = 0
        for g in _clusters(pp, 0.014, 60):
            p, c = pp[g], pc[g]
            if c[:, 0].mean() - c[:, 2].mean() > 25:        # cookie box (brown)
                continue
            if not (0.09 < _span(p) < 0.20):
                continue
            if len(p) > best:
                best = len(p)
                plate = np.array([p[:, 0].mean(), p[:, 1].mean()])

    s = Scene()
    s.zt, s.bowls, s.ram, s.plate = zt, bowls, ram, plate
    return s


def run(api):
    s = perceive(api)
    api.log("zt=%.4f bowls=%s ram=%s plate=%s" % (
        s.zt, [np.round(b, 3).tolist() for b in s.bowls],
        None if s.ram is None else np.round(s.ram, 3).tolist(),
        None if s.plate is None else np.round(s.plate, 3).tolist()))
    if not s.bowls or s.plate is None:
        return "perception failed"

    if s.ram is not None and len(s.bowls) > 1:
        tgt = min(s.bowls, key=lambda b: np.linalg.norm(b - s.ram))
    else:
        tgt = s.bowls[0]
    api.log("target=%s" % np.round(tgt, 4).tolist())

    R0 = np.asarray(api.tool_rotation(), float)
    ea = np.abs(_conv_a(HOME_RPY) - R0).max()
    eb = np.abs(_conv_b(HOME_RPY) - R0).max()
    conv = _conv_a if ea <= eb else _conv_b
    Rg = conv(GRASP_RPY)

    gxy = tgt + GRASP_OFFSET
    gz = s.zt + GRASP_DZ
    api.grip(0.08)
    api.move([gxy[0], gxy[1], CARRY_Z], rotation=Rg, seconds=2.5)
    api.move([gxy[0], gxy[1], gz + 0.05], rotation=Rg, seconds=1.5)
    api.move([gxy[0], gxy[1], gz], rotation=Rg, seconds=2.0)
    api.move([gxy[0], gxy[1], gz], rotation=Rg, seconds=1.0)
    api.settle(0.3)
    eef_g = api.eef()
    api.log("pre-grasp eef=%s" % np.round(eef_g, 4).tolist())
    api.grip(0.0)
    api.settle(0.8)
    g = api.gripper()
    api.log("closed w=%.4f eff=%.2f" % (g["width_m"], g["effort"]))

    achieved = eef_g[:2] - tgt                      # where the jaws really sit
    api.move([gxy[0], gxy[1], CARRY_Z], rotation=Rg, seconds=2.5)
    api.settle(0.4)
    g2 = api.gripper()
    api.log("lifted w=%.4f eff=%.2f off=%s" % (g2["width_m"], g2["effort"],
                                               np.round(achieved, 4).tolist()))

    pxy = s.plate + achieved
    api.move([pxy[0], pxy[1], CARRY_Z], rotation=Rg, seconds=3.0)
    api.move([pxy[0], pxy[1], s.zt + RELEASE_DZ], rotation=Rg, seconds=2.0)
    api.settle(0.3)
    api.log("over plate eef=%s" % np.round(api.eef(), 4).tolist())
    api.grip(0.08)
    api.settle(0.6)
    api.move([pxy[0], pxy[1], CARRY_Z], rotation=Rg, seconds=2.0)
    api.settle(0.6)

    s2 = perceive(api)
    api.log("post bowls=%s plate=%s" % (
        [np.round(b, 3).tolist() for b in s2.bowls],
        None if s2.plate is None else np.round(s2.plate, 3).tolist()))
    return "done"
