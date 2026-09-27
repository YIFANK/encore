"""v4 -- first full pipeline: press-drag the top drawer open, rim-pinch the
bowl, drop it in.  Heavily instrumented; every phase logs eef + residual.

Mechanism read off the K=1 pack:
  t 20-30  descend with OPEN fingers just in front of the drawer-handle bar
           until the descent is blocked (pack z stalls at 1.104)
  t 35-60  command +y and DOWN together -> the finger drags the handle out
  t 70-100 retreat, descend on the bowl rim, close
  t 100-163 carry up and back over the open drawer, release
"""
import numpy as np

PROVENANCE = {
    "R_DOWN": {"source": "generic: tool-down orientation, matches pack ee_path6 "
                         "rotvec [3.13,-0.03,-0.03] at every keyframe",
               "allowed": True},
    "PRESS_DY": {"source": "pack demo0: press point y=-0.093 vs handle bar centre "
                           "y=-0.140 measured on debug seeds -> +0.047",
                 "allowed": True},
    "DRAG_DY": {"source": "pack demo0 ee_path6: y -0.093 -> +0.065 = 0.158 m of drag",
                "allowed": True},
    "PRESS_DOWN": {"source": "pack demo0 actions t35-55: z command stays negative "
                             "throughout the drag (press while pulling)",
                   "allowed": True},
    "R_PINCH": {"source": "debug seeds 51-57 cam_high: bowl footprint 0.108x0.112 m "
                          "-> outer radius 0.055, wall midline 0.0525",
                "allowed": True},
    "EMPTY_W": {"source": "v2 debug run: finger gap 0.0010/0.0018 after closing on air vs 0.0035-0.0112 with the rim between the jaws",
                "allowed": True},
    "GRASP_DZ": {"source": "pack demo0 closes 0.0135 below the rim; v1 debug run "
                           "converged 0.004 short (POS_TOL 12 mm) and seed 57 "
                           "slipped, so command 0.030 to land 0.013-0.030 deep",
                 "allowed": True},
    "DROP_DY": {"source": "pack demo0: release y=-0.046 vs handle bar after opening "
                          "(-0.140+0.158=+0.018) -> 0.064 behind the handle",
                "allowed": True},
    "DROP_DZ": {"source": "pack demo0: release z=1.1354 vs cabinet top 1.1273 "
                          "measured on debug seeds -> +0.008",
                "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
PRESS_DY = 0.047
DRAG_DY = 0.165
PRESS_DOWN = 0.030
R_PINCH = 0.0525
EMPTY_W = 0.0028
GRASP_DZ = 0.030
DROP_DY = 0.064
DROP_DZ = 0.008

RES = 0.004
XLIM = (-0.55, 0.45)
YLIM = (-0.45, 0.45)


# --------------------------------------------------------------- perception
def _cloud(frame):
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    d = np.asarray(frame.depth, float)
    h, w = d.shape
    v, u = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 1e-4) & (d < 5.0)
    z = np.where(ok, d, 1.0)
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    P = np.stack([x, y, z], -1) @ T[:3, :3].T + T[:3, 3]
    return P, ok


def heightmap(frame):
    P, ok = _cloud(frame)
    nx = int((XLIM[1] - XLIM[0]) / RES)
    ny = int((YLIM[1] - YLIM[0]) / RES)
    hm = np.full((nx, ny), -9.0)
    src = np.full((nx, ny, 2), -1, dtype=np.int32)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    h, w = Z.shape
    vv, uu = np.mgrid[0:h, 0:w]
    ix = np.where(ok, (X - XLIM[0]) / RES, -1e9)
    iy = np.where(ok, (Y - YLIM[0]) / RES, -1e9)
    good = ok & (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & np.isfinite(Z)
    ix = ix[good].astype(int); iy = iy[good].astype(int)
    zz = Z[good]; vv = vv[good]; uu = uu[good]
    order = np.argsort(zz)
    hm[ix[order], iy[order]] = zz[order]
    src[ix[order], iy[order], 0] = vv[order]
    src[ix[order], iy[order], 1] = uu[order]
    return hm, src


def gx(i):
    return XLIM[0] + np.asarray(i) * RES


def gy(j):
    return YLIM[0] + np.asarray(j) * RES


def label(mask):
    nx, ny = mask.shape
    lab = np.zeros((nx, ny), np.int32)
    parent = [0]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    nxt = 1
    idxs = np.argwhere(mask)
    for i, j in idxs:
        nb = []
        if i > 0 and lab[i - 1, j]:
            nb.append(lab[i - 1, j])
        if j > 0 and lab[i, j - 1]:
            nb.append(lab[i, j - 1])
        if i > 0 and j > 0 and lab[i - 1, j - 1]:
            nb.append(lab[i - 1, j - 1])
        if i > 0 and j < ny - 1 and lab[i - 1, j + 1]:
            nb.append(lab[i - 1, j + 1])
        if not nb:
            lab[i, j] = nxt
            parent.append(nxt)
            nxt += 1
        else:
            m = min(nb)
            lab[i, j] = m
            for x in nb:
                ra, rb = find(m), find(x)
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)
    out = np.zeros_like(lab)
    remap = {}
    n = 0
    for i, j in idxs:
        r = find(lab[i, j])
        if r not in remap:
            n += 1
            remap[r] = n
        out[i, j] = remap[r]
    return out, n


def biggest(mask):
    lab, n = label(mask)
    best, bn = None, 0
    for k in range(1, n + 1):
        c = int((lab == k).sum())
        if c > bn:
            bn, best = c, k
    if best is None:
        return None
    return np.where(lab == best)


def table_z(hm):
    z = hm[hm > 0]
    bins = np.round(z / 0.002).astype(int)
    vals, cnt = np.unique(bins, return_counts=True)
    return float(vals[np.argmax(cnt)] * 0.002)


def find_cabinet(hm, tz):
    m = hm > tz + 0.15
    z = hm[m]
    bins = np.round(z / 0.005).astype(int)
    vals, cnt = np.unique(bins, return_counts=True)
    top = float(vals[np.argmax(cnt)] * 0.005)
    ii, jj = biggest(m & (np.abs(hm - top) < 0.009))
    return dict(top=float(hm[ii, jj].max()), x0=float(gx(ii.min())), x1=float(gx(ii.max())),
                y0=float(gy(jj.min())), y1=float(gy(jj.max())), n=int(len(ii)))


def find_handle(hm, cab, ywin=0.075):
    nx, ny = hm.shape
    ii, jj = np.mgrid[0:nx, 0:ny]
    X, Y = gx(ii), gy(jj)
    m = ((Y > cab["y1"] - 0.006) & (Y < cab["y1"] + ywin)
         & (X > cab["x0"] - 0.03) & (X < cab["x1"] + 0.03)
         & (hm > cab["top"] - 0.062) & (hm < cab["top"] - 0.012))
    if m.sum() < 15:
        return None
    zt = float(hm[m].max())
    got = biggest(m & (hm > zt - 0.014))
    if got is None:
        return None
    ii, jj = got
    return dict(cx=float(gx(ii).mean()), cy=float(gy(jj).mean()),
                x0=float(gx(ii.min())), x1=float(gx(ii.max())),
                y0=float(gy(jj.min())), y1=float(gy(jj.max())),
                top=zt, n=int(len(ii)))


def find_bowl(hm, src, rgb, tz, cab):
    nx, ny = hm.shape
    ii, jj = np.mgrid[0:nx, 0:ny]
    X, Y = gx(ii), gy(jj)
    m = (hm > tz + 0.040) & (hm < tz + 0.095)
    m &= ~((X > cab["x0"] - 0.05) & (X < cab["x1"] + 0.05)
           & (Y > cab["y0"] - 0.05) & (Y < cab["y1"] + 0.05))
    lab, n = label(m)
    out = []
    for k in range(1, n + 1):
        a, b = np.where(lab == k)
        if len(a) < 60:
            continue
        xs, ys = gx(a), gy(b)
        dx, dy = xs.max() - xs.min(), ys.max() - ys.min()
        if not (0.07 < dx < 0.16 and 0.07 < dy < 0.16):
            continue
        vs = src[a, b, 0]; us = src[a, b, 1]
        good = vs >= 0
        bright = float(rgb[vs[good], us[good]].mean()) if good.any() else 0.0
        out.append(dict(cx=float(xs.mean()), cy=float(ys.mean()),
                        x0=float(xs.min()), x1=float(xs.max()),
                        y0=float(ys.min()), y1=float(ys.max()),
                        top=float(hm[a, b].max()), n=int(len(a)), bright=bright))
    out.sort(key=lambda d: -d["bright"])
    return out




def find_cavity(hm, cab):
    """After the drag: the open drawer's floor, a plateau ~60 mm below the
    cabinet top, in front of the cabinet face."""
    nx, ny = hm.shape
    ii, jj = np.mgrid[0:nx, 0:ny]
    X, Y = gx(ii), gy(jj)
    m = ((Y > cab["y1"] - 0.01) & (Y < cab["y1"] + 0.32)
         & (X > cab["x0"] - 0.04) & (X < cab["x1"] + 0.04)
         & (hm > cab["top"] - 0.095) & (hm < cab["top"] - 0.035))
    if m.sum() < 120:
        return None
    got = biggest(m)
    if got is None:
        return None
    a, b = got
    if len(a) < 120:
        return None
    return dict(cx=float(gx(a).mean()), cy=float(gy(b).mean()),
                x0=float(gx(a.min())), x1=float(gx(a.max())),
                y0=float(gy(b.min())), y1=float(gy(b.max())),
                floor=float(np.median(hm[a, b])), n=int(len(a)))


def find_stray(hm, src, rgb, tz, cab):
    """A bowl-sized blob still standing on the TABLE after the release
    (upright ~50 mm tall, tipped ~110 mm) = the placement did not take."""
    nx, ny = hm.shape
    ii, jj = np.mgrid[0:nx, 0:ny]
    X, Y = gx(ii), gy(jj)
    m = (hm > tz + 0.035) & (hm < tz + 0.135)
    m &= ~((X > cab["x0"] - 0.06) & (X < cab["x1"] + 0.06)
           & (Y > cab["y0"] - 0.06) & (Y < cab["y1"] + 0.38))
    lab, n = label(m)
    out = []
    for k in range(1, n + 1):
        a, b = np.where(lab == k)
        if len(a) < 60:
            continue
        xs, ys = gx(a), gy(b)
        if not (0.06 < xs.max() - xs.min() < 0.17 and 0.06 < ys.max() - ys.min() < 0.17):
            continue
        vs = src[a, b, 0]; us = src[a, b, 1]
        good = vs >= 0
        bright = float(rgb[vs[good], us[good]].mean()) if good.any() else 0.0
        if bright < 90.0:
            continue
        out.append(dict(cx=float(xs.mean()), cy=float(ys.mean()),
                        top=float(hm[a, b].max()), n=int(len(a)), bright=bright))
    out.sort(key=lambda d: -d["bright"])
    return out


# --------------------------------------------------------------------- run
def _r(d):
    return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in d.items()}


def run(api):
    log = api.log
    log(f"INSTR {api.instruction()}")

    f = api.capture("cam_high")
    hm, src = heightmap(f)
    tz = table_z(hm)
    cab = find_cabinet(hm, tz)
    hnd = find_handle(hm, cab)
    if hnd is None:
        # No bar found: the drawer face is the cabinet's +y edge and the pack
        # puts the bar 0.030 m below the top, centred on the face.
        hnd = dict(cx=0.5 * (cab["x0"] + cab["x1"]), cy=cab["y1"] + 0.018,
                   top=cab["top"] - 0.030, n=0)
        log("P0 handle FALLBACK")
    bowls = find_bowl(hm, src, np.asarray(f.rgb), tz, cab)
    log(f"P0 table={tz:.4f}")
    log(f"P0 cab {_r(cab)}")
    log(f"P0 handle {_r(hnd)}")
    for b in bowls[:3]:
        log(f"P0 bowlcand {_r(b)}")
    bowl = bowls[0] if bowls else dict(cx=-0.053, cy=0.132, top=tz + 0.051)
    if not bowls:
        log("P0 bowl FALLBACK")

    def mv(name, xyz, secs, rot=R_DOWN):
        res = api.move(xyz, rotation=rot, seconds=secs)
        e = api.eef()
        log(f"MV {name} tgt={[round(float(v),4) for v in xyz]} "
            f"eef={[round(float(v),4) for v in e]} res={res:.4f}")
        return e

    # ---- phase 1: drag the top drawer open -------------------------------
    api.grip(0.08)
    px, py = hnd["cx"], hnd["cy"] + PRESS_DY
    mv("approach", [px, py, hnd["top"] + 0.09], 1.2)
    e = mv("press", [px, py, hnd["top"] - 0.035], 0.5)
    zc = float(e[2])
    log(f"CONTACT z={zc:.4f} handle_top={hnd['top']:.4f} dz={zc - hnd['top']:.4f}")
    e = mv("drag", [px, py + DRAG_DY, zc - PRESS_DOWN], 2.5)
    e = mv("lift", [float(e[0]), float(e[1]), cab["top"] + 0.15], 0.8)

    # ---- verify the opening, and find the cavity to drop into ------------
    f2 = api.capture("cam_high")
    hm2, _ = heightmap(f2)
    h2 = find_handle(hm2, cab, ywin=0.32)
    cav = find_cavity(hm2, cab)
    opened = (h2["cy"] - hnd["cy"]) if h2 else -1.0
    log(f"P1 handle {_r(h2) if h2 else None} opened={opened:.4f}")
    log(f"P1 cavity {_r(cav) if cav else None}")
    if opened < 0.10:
        log("RETRY drag")
        hb = h2 if h2 else hnd
        px2, py2 = hb["cx"], hb["cy"] + PRESS_DY
        mv("re_approach", [px2, py2, hb["top"] + 0.07], 0.8)
        e = mv("re_press", [px2, py2, hb["top"] - 0.035], 0.5)
        e = mv("re_drag", [px2, py2 + DRAG_DY, float(e[2]) - PRESS_DOWN], 2.0)
        e = mv("re_lift", [float(e[0]), float(e[1]), cab["top"] + 0.15], 0.8)
        f2 = api.capture("cam_high")
        hm2, _ = heightmap(f2)
        h2 = find_handle(hm2, cab, ywin=0.32)
        cav = find_cavity(hm2, cab)
        log(f"P2 handle {_r(h2) if h2 else None}")
        log(f"P2 cavity {_r(cav) if cav else None}")

    if cav is not None:
        dx, dy, floor = cav["cx"], cav["cy"], cav["floor"]
    else:
        hy = h2["cy"] if h2 else hnd["cy"] + DRAG_DY
        dx, dy, floor = (h2["cx"] if h2 else hnd["cx"]), hy - 0.118, cab["top"] - 0.064
    log(f"DROP tgt=({dx:.4f},{dy:.4f}) floor={floor:.4f}")

    def pick_and_place(b, tag):
        """Rim-pinch the +y arc, carry, seat on the drawer floor, let go."""
        gxp = b["cx"]
        gyp = b["cy"] + R_PINCH   # the -y arc puts a finger on the opened
                                  # drawer front (v2 stalled 5/8 at z 1.090)
        mv(tag + "hover", [gxp, gyp, b["top"] + 0.07], 1.2)
        e = mv(tag + "down", [gxp, gyp, b["top"] - GRASP_DZ], 0.6)
        api.grip(0.0)
        log(f"GRASP{tag} eef={[round(float(v),4) for v in e]} grip={api.gripper()}")
        e = mv(tag + "lift", [gxp, gyp, cab["top"] + 0.15], 1.4)
        g = api.gripper()
        log(f"LIFTED{tag} eef={[round(float(v),4) for v in e]} grip={g}")
        if g["width_m"] < EMPTY_W:
            log("REGRASP" + tag)
            api.grip(0.08)
            mv(tag + "redown", [gxp, gyp, b["top"] - GRASP_DZ], 0.8)
            api.grip(0.0)
            e = mv(tag + "relift", [gxp, gyp, cab["top"] + 0.15], 1.2)
            log(f"RELIFTED{tag} grip={api.gripper()}")
        mv(tag + "carry", [dx, dy, cab["top"] + 0.15], 1.6)
        e = mv(tag + "lower", [dx, dy, floor + 0.04], 0.9)
        log(f"ATFLOOR{tag} eef={[round(float(v),4) for v in e]} grip={api.gripper()}")
        api.grip(0.08)
        api.settle(1.0)
        mv(tag + "retreat", [dx, dy, cab["top"] + 0.17], 0.8)
        api.settle(0.5)

    pick_and_place(bowl, "")

    # ---- verify: a bowl still standing on the table means it never went in
    try:
        f3 = api.capture("cam_high")
        hm3, src3 = heightmap(f3)
        stray = find_stray(hm3, src3, np.asarray(f3.rgb), tz, cab)
        for b in stray[:2]:
            log(f"P3 stray {_r(b)}")
        cav3 = find_cavity(hm3, cab)
        log(f"P3 cavity {_r(cav3) if cav3 else None}")
        if stray:
            b = dict(stray[0])
            b["top"] = min(b["top"], tz + 0.055)
            log(f"RETRY place at {_r(b)}")
            pick_and_place(b, "b")
            f4 = api.capture("cam_high")
            hm4, src4 = heightmap(f4)
            for c in find_stray(hm4, src4, np.asarray(f4.rgb), tz, cab)[:2]:
                log(f"P4 stray {_r(c)}")
    except Exception as exc:  # noqa: BLE001
        log(f"VERIFY-SKIP {type(exc).__name__}: {exc}")
    log("END")
