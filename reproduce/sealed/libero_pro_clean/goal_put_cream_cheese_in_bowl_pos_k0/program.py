import zlib, base64, numpy as np

PROVENANCE = {
    "PARK_XYZ": {"source": "debug-seed observation (v1/v2 probes): pose that clears the arm from the cam_high view of the props", "allowed": True},
    "GRID": {"source": "generic camera/geometry mechanics: 4 mm top-down occupancy grid", "allowed": True},
    "RIM_BAND": {"source": "debug-seed measurement: bowl rim sits ~50 mm above the table, props of interest are under 80 mm", "allowed": True},
    "PROBE_SPOT": {"source": "debug-seed observation: bare table patch with no prop in the cam_high height map", "allowed": True},
    "GRASP_Z": {"source": "debug-seed measurement (v3): open-gripper descent onto the bare table stalls at eef z=0.9095 with table z=0.900, so eef z 0.912 puts the fingertips ~2 mm above the table, low on the 19.5 mm box", "allowed": True},
    "CARRY_Z": {"source": "debug-seed measurement (v3/v0b): the wine bottle beside the bowl tops out at z=1.06 in the cam_high height map; carry above it", "allowed": True},
    "DROP_CLEAR": {"source": "debug-seed measurement: box bottom hangs ~12 mm below the eef at GRASP_Z; 45 mm above the measured rim z clears the rim", "allowed": True},
    "HOVER_DZ": {"source": "debug-seed measurement (v4): 100 mm above the table clears the 19.5 mm box on the lateral approach", "allowed": True},
    "BLUE_Z_GATE": {"source": "debug-seed measurement (v4): the box top reads z=0.9195 on the table and z=1.020 once inside the bowl; the parked arm's blue link sits above z=1.15, so a tz+0.18 ceiling separates them", "allowed": True},
    "HOLD_W": {"source": "debug-seed measurement (v4): a successful grasp reads gripper width 0.042 m (the box y-width); an empty close reads 0.001 m", "allowed": True},
}

PARK_XYZ = [-0.10, 0.30, 1.20]
PROBE_SPOT = [0.13, 0.12]


# ---------------- perception ----------------

def pcd(K, T, depth):
    H, W = depth.shape
    v, u = np.mgrid[0:H, 0:W]
    z = np.nan_to_num(depth.astype(np.float64), nan=0.0, posinf=0.0, neginf=0.0)
    x = (u - K[0, 2]) / K[0, 0] * z
    y = (v - K[1, 2]) / K[1, 1] * z
    return np.stack([x, y, z, np.ones_like(z)], -1) @ T.T


def _corr(img, ker):
    sh = (img.shape[0] + ker.shape[0] - 1, img.shape[1] + ker.shape[1] - 1)
    out = np.fft.irfft2(np.fft.rfft2(img, sh) * np.fft.rfft2(ker, sh), sh)
    o0, o1 = ker.shape[0] // 2, ker.shape[1] // 2
    return out[o0:o0 + img.shape[0], o1:o1 + img.shape[1]]


def scene(api, fr):
    rgb = np.asarray(fr.rgb, dtype=np.float64)
    dep = np.asarray(fr.depth, dtype=np.float64)
    XYZ = pcd(np.asarray(fr.intrinsics, dtype=np.float64), np.asarray(fr.t_base_cam, dtype=np.float64), dep)[..., :3]
    X, Y, Z = XYZ[..., 0], XYZ[..., 1], XYZ[..., 2]
    ws = (X > -0.40) & (X < 0.30) & (Y > -0.40) & (Y < 0.40) & (Z > 0.5) & (Z < 1.6)
    hist, edges = np.histogram(Z[ws], bins=120, range=(0.7, 1.3))
    tz = edges[int(np.argmax(hist))] + 0.0025
    api.log("TABLE_Z %.4f" % tz)

    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    s = rgb.sum(-1) + 1e-6
    blue = (b / s > 0.40) & (b > r + 8) & (b > g + 8) & ws & (Z > tz + 0.004) & (Z < tz + 0.18)
    ys, xs = np.nonzero(blue)
    pts = XYZ[blue]
    if len(ys) > 20:
        cu, cv = np.median(xs), np.median(ys)
        k = (np.abs(xs - cu) < 60) & (np.abs(ys - cv) < 60)
        pts = pts[k]
    b_cent = pts.mean(0)
    b_top = float(np.percentile(pts[:, 2], 95))
    face = ws & (np.abs(Z - b_top) < 0.005) & (np.hypot(X - b_cent[0], Y - b_cent[1]) < 0.09)
    fp = XYZ[face]
    cent = fp.mean(0) if len(fp) > 50 else b_cent
    cc = dict(n=int(len(fp)), cent=np.asarray(cent, dtype=float), top=b_top,
              xmin=float(fp[:, 0].min()), xmax=float(fp[:, 0].max()),
              ymin=float(fp[:, 1].min()), ymax=float(fp[:, 1].max()))
    api.log("CC n=%d cent=%s top=%.4f x[%.3f,%.3f] y[%.3f,%.3f]" % (
        cc['n'], np.round(cc['cent'], 4).tolist(), cc['top'], cc['xmin'], cc['xmax'], cc['ymin'], cc['ymax']))

    gx = np.arange(-0.34, 0.16, 0.004)
    gy = np.arange(-0.16, 0.14, 0.004)
    HM = np.full((len(gx), len(gy)), tz)
    ix = np.floor((X + 0.34) / 0.004).astype(np.int64)
    iy = np.floor((Y + 0.16) / 0.004).astype(np.int64)
    ok = ws & (ix >= 0) & (ix < len(gx)) & (iy >= 0) & (iy < len(gy)) & (Z < tz + 0.30)
    np.maximum.at(HM, (ix[ok], iy[ok]), Z[ok])

    rim = ((HM > tz + 0.022) & (HM < tz + 0.080)).astype(np.float64)
    low = (HM < tz + 0.015).astype(np.float64)
    rr = np.arange(-0.09, 0.0901, 0.004)
    KX, KY = np.meshgrid(rr, rr, indexing='ij')
    KD = np.hypot(KX, KY)
    Kann = ((KD > 0.040) & (KD < 0.068)).astype(np.float64)
    Kin = (KD < 0.028).astype(np.float64)
    score = (_corr(rim, Kann) / Kann.sum()) * (_corr(low, Kin) / Kin.sum())
    i, j = np.unravel_index(int(np.argmax(score)), score.shape)
    cxy = np.array([gx[i], gy[j]])
    a, bb = np.nonzero(rim > 0)
    px, py = gx[a], gy[bb]
    d = np.hypot(px - cxy[0], py - cxy[1])
    band = (d < 0.075) & (d > 0.030)
    if band.sum() >= 12:
        A = np.stack([px[band], py[band], np.ones(int(band.sum()))], -1)
        sol = np.linalg.lstsq(A, px[band] ** 2 + py[band] ** 2, rcond=None)[0]
        fx, fy = sol[0] / 2.0, sol[1] / 2.0
        frad = float(np.sqrt(max(sol[2] + fx * fx + fy * fy, 1e-6)))
        if np.hypot(fx - cxy[0], fy - cxy[1]) < 0.03 and 0.035 < frad < 0.08:
            cxy = np.array([fx, fy])
    d = np.hypot(px - cxy[0], py - cxy[1])
    keep = d < 0.075
    bowl = dict(cent=cxy, rad=float(np.percentile(d[keep], 90)),
                rim=float(np.percentile(HM[a[keep], bb[keep]], 90)))
    api.log("BOWL cent=(%.4f,%.4f) rad=%.4f rimz=%.4f" % (cxy[0], cxy[1], bowl['rad'], bowl['rim']))
    return dict(tz=tz, cc=cc, bowl=bowl)




GRASP_Z = 0.912
CARRY_Z = 1.12
DROP_CLEAR = 0.045
HOVER_DZ = 0.10
HOLD_W = (0.028, 0.058)


def aim(api, x, y, z, s1=1.5, s2=1.0):
    """Move to (x,y,z), then cancel the residual xy tracking bias in the command."""
    api.move([x, y, z], seconds=s1)
    e = api.eef()
    api.move([x + (x - e[0]), y + (y - e[1]), z], seconds=s2)
    return api.eef()


def holding(api):
    g = api.gripper()
    return (HOLD_W[0] < g['width_m'] < HOLD_W[1]) and g['effort'] >= 1.0


def perceive(api):
    api.move(PARK_XYZ, seconds=2.0)
    api.settle(0.3)
    return scene(api, api.capture("cam_high"))


def grasp(api, sc):
    cc = sc['cc']
    tz = sc['tz']
    cx, cy = float(cc['cent'][0]), float(cc['cent'][1])
    api.grip(0.08)
    aim(api, cx, cy, tz + HOVER_DZ)
    api.move([cx, cy, GRASP_Z], seconds=1.5)
    e = api.eef()
    api.move([cx + (cx - e[0]), cy + (cy - e[1]), GRASP_Z], seconds=1.0)
    api.log("PRE_CLOSE eef=%s tgt=(%.4f,%.4f)" % (np.round(api.eef(), 4).tolist(), cx, cy))
    api.grip(0.0)
    api.settle(0.4)
    ok = holding(api)
    api.log("CLOSED grip=%s hold=%s" % (api.gripper(), ok))
    return ok


def place(api, sc):
    bw = sc['bowl']
    bx, by = float(bw['cent'][0]), float(bw['cent'][1])
    rimz = float(bw['rim'])
    e = api.eef()
    api.move([e[0], e[1], CARRY_Z], seconds=1.5)
    aim(api, bx, by, CARRY_Z)
    api.move([bx, by, rimz + DROP_CLEAR], seconds=1.5)
    api.log("AT_DROP eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(0.08)
    api.settle(0.6)
    api.move([bx, by, CARRY_Z], seconds=1.5)


def run(api):
    for cycle in range(3):
        sc = perceive(api)
        cc = np.asarray(sc['cc']['cent'], dtype=float)
        bw = sc['bowl']
        inbowl = np.hypot(cc[0] - bw['cent'][0], cc[1] - bw['cent'][1]) < bw['rad'] and cc[2] > sc['tz'] + 0.02
        api.log("CYCLE %d cc=%s inbowl=%s" % (cycle, np.round(cc, 4).tolist(), inbowl))
        if inbowl:
            api.log("DONE by re-perception")
            return
        got = False
        for k in range(2):
            got = grasp(api, sc)
            if got:
                break
            api.grip(0.08)
            api.move([float(sc['cc']['cent'][0]), float(sc['cc']['cent'][1]), sc['tz'] + 0.15], seconds=1.5)
            sc = perceive(api)
        if not got:
            api.log("NO_GRASP cycle=%d" % cycle)
            continue
        place(api, sc)
