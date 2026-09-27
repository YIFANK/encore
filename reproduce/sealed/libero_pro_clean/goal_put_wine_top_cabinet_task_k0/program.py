import numpy as np, zlib, base64

PROVENANCE = {
    "PARK": {"source": "debug-seed obs (v3 51/53/57/63): eef pose that clears cam_high's view of bottle+bowl", "allowed": True},
    "XLIM/YLIM/RES": {"source": "debug-seed obs (v2): extent of the deprojected table point cloud", "allowed": True},
    "TIP_OFF": {"source": "debug-seed measurement (v2 home capture): lowest deprojected point of the +y finger is 0.005 m below api.eef(); v4 closed-gripper table contact gave 0.009", "allowed": True},
    "NECK_DROP": {"source": "debug-seed obs (v3): bottle neck spans ztop-0.06..ztop-0.015, dia 0.014; body dia 0.043 below ztop-0.085", "allowed": True},
    "BOTTLE_BAND": {"source": "debug-seed obs (v3): cork top deprojects to 1.059 m = table+0.158", "allowed": True},
    "BOWL_BAND": {"source": "debug-seed obs (v3): bowl rim 0.952 = table+0.051, plate top table+0.019", "allowed": True},
    "BOWL_WIDTH_RANGE": {"source": "debug-seed obs (v3): bowl footprint 0.110 m across on 4 seeds", "allowed": True},
    "BOWL_FLOOR": {"source": "debug-seed obs (v3): bowl interior median height table+0.007", "allowed": True},
    "CARRY_CLEAR": {"source": "debug-seed obs (v3): bowl rim is the tallest obstacle between bottle and bowl", "allowed": True},
    "CAM": {"source": "generic pinhole camera mechanics (FairFrame intrinsics/extrinsics)", "allowed": True},
}

CH = 1900
XLIM = (-0.50, 0.40)
YLIM = (-0.50, 0.50)
RES = 0.005
PARK = [0.05, 0.36, 1.32]
TIP_OFF = 0.006


def blob(api, tag, arr):
    b = base64.b64encode(zlib.compress(arr.tobytes(), 9)).decode()
    n = (len(b) + CH - 1) // CH
    for i in range(n):
        api.log("%s|%d|%d|%s" % (tag, i, n, b[i * CH:(i + 1) * CH]))


def dump_raw(api, f, tag):
    api.log("%s|K|%s" % (tag, np.asarray(f.intrinsics).ravel().tolist()))
    api.log("%s|T|%s" % (tag, np.asarray(f.t_base_cam).ravel().tolist()))
    blob(api, tag + "RGB", np.asarray(f.rgb, dtype=np.uint8)[::2, ::2])
    blob(api, tag + "D", (np.nan_to_num(np.asarray(f.depth, dtype=np.float32), nan=0.0) * 10000.0).astype(np.uint16))


# ------------------------------------------------------------------ perception
def cloud(f):
    K = np.asarray(f.intrinsics, dtype=float)
    T = np.asarray(f.t_base_cam, dtype=float)
    depth = np.nan_to_num(np.asarray(f.depth, dtype=float), nan=0.0)
    rgb = np.asarray(f.rgb, dtype=np.float32)
    R = T[:3, :3]; t = T[:3, 3]; fx = K[0, 0]; cx = K[0, 2]; cy = K[1, 2]
    hh, ww = depth.shape
    rows, cols = np.mgrid[0:hh, 0:ww]
    ray = np.stack([(cols - cx) / fx, (rows - cy) / fx, np.ones_like(cols, dtype=float)], -1)
    P = (ray[..., None, :] * R[None, None]).sum(-1) * depth[..., None] + t
    ok = ((P[..., 0] > XLIM[0]) & (P[..., 0] < XLIM[1]) & (P[..., 1] > YLIM[0]) &
          (P[..., 1] < YLIM[1]) & (depth > 0.1) & (depth < 3.0))
    return P, rgb, ok


def gridmap(P, rgb, ok):
    nx = int(round((XLIM[1] - XLIM[0]) / RES)); ny = int(round((YLIM[1] - YLIM[0]) / RES))
    ix = np.clip(((P[..., 0] - XLIM[0]) / RES).astype(int), 0, nx - 1)
    iy = np.clip(((P[..., 1] - YLIM[0]) / RES).astype(int), 0, ny - 1)
    H = np.zeros((nx, ny)); C = np.zeros((nx, ny, 3))
    z = P[..., 2]; o = np.argsort(z[ok])
    H[ix[ok][o], iy[ok][o]] = z[ok][o]; C[ix[ok][o], iy[ok][o]] = rgb[ok][o]
    return H, C


def cc(mask):
    lab = np.zeros(mask.shape, np.int32); n = 0
    for i0, j0 in np.argwhere(mask):
        if lab[i0, j0]:
            continue
        n += 1; st = [(i0, j0)]; lab[i0, j0] = n
        while st:
            a, b = st.pop()
            for p, q in ((a - 1, b), (a + 1, b), (a, b - 1), (a, b + 1),
                         (a - 1, b - 1), (a - 1, b + 1), (a + 1, b - 1), (a + 1, b + 1)):
                if 0 <= p < mask.shape[0] and 0 <= q < mask.shape[1] and mask[p, q] and not lab[p, q]:
                    lab[p, q] = n; st.append((p, q))
    return lab, n


def comps(H, C, lo, hi, nmin=8, nmax=100000):
    lab, n = cc((H >= lo) & (H <= hi))
    out = []
    for k in range(1, n + 1):
        ii, jj = np.where(lab == k)
        if not (nmin <= ii.size <= nmax):
            continue
        x = XLIM[0] + (ii + 0.5) * RES; y = YLIM[0] + (jj + 0.5) * RES
        out.append(dict(n=int(ii.size), xc=float(x.mean()), yc=float(y.mean()),
                        x0=float(x.min()), x1=float(x.max()), y0=float(y.min()), y1=float(y.max()),
                        ztop=float(H[ii, jj].max()), rgb=float(C[ii, jj].mean())))
    return out


def table_z(H):
    v = H[H > 0.5]
    return float(np.median(v[(v > np.percentile(v, 5)) & (v < np.percentile(v, 60))]))


def ring_support(H, c, pad=5):
    i0 = int((c['x0'] - XLIM[0]) / RES); i1 = int((c['x1'] - XLIM[0]) / RES)
    j0 = int((c['y0'] - YLIM[0]) / RES); j1 = int((c['y1'] - YLIM[0]) / RES)
    a0 = max(0, i0 - pad); a1 = min(H.shape[0] - 1, i1 + pad)
    b0 = max(0, j0 - pad); b1 = min(H.shape[1] - 1, j1 + pad)
    sub = H[a0:a1 + 1, b0:b1 + 1]
    inner = np.zeros(sub.shape, bool); inner[i0 - a0:i1 - a0 + 1, j0 - b0:j1 - b0 + 1] = True
    v = sub[~inner]; v = v[v > 0.5]
    return float(np.median(v)) if v.size else 0.0


def find_bottle(P, rgb, ok, H, C, tz):
    """cork blob -> local neck axis from the raw cloud."""
    best = None
    for c in comps(H, C, tz + 0.099, tz + 0.259, nmin=10, nmax=400):
        if max(c['x1'] - c['x0'], c['y1'] - c['y0']) > 0.12:
            continue
        sc = 200.0 - c['rgb']
        if best is None or sc > best[0]:
            best = (sc, c)
    if best is None:
        return None
    cork = best[1]
    ztop = cork['ztop']
    near = ok & (np.abs(P[..., 0] - cork['xc']) < 0.06) & (np.abs(P[..., 1] - cork['yc']) < 0.06)
    out = {}
    for tag, (lo, hi) in (("neck", (ztop - 0.055, ztop - 0.014)),
                          ("body", (tz + 0.010, tz + 0.070))):
        m = near & (P[..., 2] > lo) & (P[..., 2] < hi)
        if m.sum() < 8:
            out[tag] = None
            continue
        y = P[..., 1][m]; x = P[..., 0][m]
        r = (y.max() - y.min()) / 2.0
        out[tag] = (float(x.max() - r), float((y.max() + y.min()) / 2.0), float(r), int(m.sum()))
    return dict(ztop=float(ztop), cork=(cork['xc'], cork['yc']), **out)


def find_bowl(H, C, tz):
    best = None
    for c in comps(H, C, tz + 0.034, tz + 0.099, nmin=40, nmax=700):
        w = max(c['x1'] - c['x0'], c['y1'] - c['y0'])
        if not (0.07 <= w <= 0.18):
            continue
        sup = ring_support(H, c)
        if abs(sup - tz) > 0.012:
            continue
        rel = c['ztop'] - sup
        if best is None or rel > best[0]:
            best = (rel, dict(x=(c['x0'] + c['x1']) / 2.0, y=(c['y0'] + c['y1']) / 2.0,
                              rim=c['ztop'], r=w / 2.0, n=c['n']))
    return best[1] if best else None


# ---------------------------------------------------------------------- motion
def goto(api, tag, xyz, secs):
    r = api.move([float(v) for v in xyz], seconds=secs)
    e = np.asarray(api.eef())
    api.log("%s|cmd%s|res%.4f|eef%s" % (tag, np.round(xyz, 4).tolist(), r, np.round(e, 4).tolist()))
    return e


def gst(api, tag):
    g = api.gripper()
    api.log("G|%s|w%.4f|e%.2f" % (tag, g['width_m'], g['effort']))
    return g


def run(api):
    api.log("INSTR|%s" % api.instruction())
    goto(api, "park", PARK, 2.2)

    f = api.capture("cam_high")
    dump_raw(api, f, "S0")
    P, rgb, ok = cloud(f)
    H, C = gridmap(P, rgb, ok)
    tz = table_z(H)
    bot = find_bottle(P, rgb, ok, H, C, tz)
    bowl = find_bowl(H, C, tz)
    api.log("PERC|tz%.4f|bot%s|bowl%s" % (tz, bot, bowl))
    if bot is None or bowl is None or bot.get("neck") is None:
        api.log("ABORT|perception")
        return

    nx, ny, nr, _ = bot["neck"]
    ztop = bot["ztop"]
    grasp_tip = ztop - 0.042                    # mid-neck, below the cork
    grasp_z = grasp_tip + TIP_OFF

    goto(api, "pre", [nx, ny, ztop + 0.09 + TIP_OFF], 2.0)
    goto(api, "down", [nx, ny, grasp_z], 1.3)
    gst(api, "pre_close")
    api.grip(0.0); api.settle(0.4)
    g = gst(api, "closed")
    e = np.asarray(api.eef())
    base_off = float(e[2] - tz)                 # eef height above the bottle's base
    api.log("BASEOFF|%.4f" % base_off)

    carry_z = tz + 0.090 + base_off             # bottle base 90 mm over the table
    goto(api, "lift", [nx, ny, carry_z], 1.3)
    gst(api, "lifted")
    goto(api, "over", [bowl['x'], bowl['y'], carry_z], 1.6)
    place_z = tz + 0.026 + base_off             # base 26 mm up: inside the rim, over the floor
    goto(api, "place", [bowl['x'], bowl['y'], place_z], 1.3)
    gst(api, "placed")
    api.grip(0.08); api.settle(0.5)
    gst(api, "released")
    goto(api, "up", [bowl['x'], bowl['y'], tz + 0.26], 1.0)

    f2 = api.capture("cam_high")
    dump_raw(api, f2, "S1")
    P2, rgb2, ok2 = cloud(f2)
    H2, C2 = gridmap(P2, rgb2, ok2)
    api.log("VERIFY|bot%s" % (find_bottle(P2, rgb2, ok2, H2, C2, tz),))
