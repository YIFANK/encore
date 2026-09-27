"""v5: straddle the target bowl's rim on the +y side, lift, carry so the BOWL
(not the eef) lands on the plate centre, release, retreat clear of the camera."""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-58 cam_high depth: modal z of the workspace plane", "allowed": True},
    "BAND_CUTS": {"source": "debug seeds 51-58 measured tops: bowl rim +0.0495, ramekin +0.0411, plate +0.0174", "allowed": True},
    "CELL": {"source": "generic 5 mm raster of the deprojected cam_high cloud", "allowed": True},
    "TIP_OFF": {"source": "debug seeds 51,53,55,57 v3 press probe: open fingertips stall at eef z = TABLE_Z+0.0058", "allowed": True},
    "BOWL_H": {"source": "debug seeds 51-58 radial height profile: bowl rim 0.0495 above its base", "allowed": True},
    "RG": {"source": "debug seeds 51,52,56 radial profile: bowl wall spans r 0.040-0.050 at 20 mm below the rim", "allowed": True},
    "GRASP_DEPTH": {"source": "debug-seed rim profile: 0.020 below the rim top, above the +0.005 interior floor", "allowed": True},
    "PLACE_CLEAR": {"source": "generic release clearance above the measured plate top", "allowed": True},
}


TABLE_Z = 0.9027          # modal workspace depth plane, debug seeds 51-58
CELL = 0.005
X0, Y0 = -0.42, -0.45
NX, NY = int(0.84 / CELL) + 1, int(0.90 / CELL) + 1


def height_map(api):
    f = api.capture("cam_high")
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    dep = np.asarray(f.depth, float)
    rgb = np.asarray(f.rgb, float) / 255.0
    H0, W0 = dep.shape
    vv, uu = np.mgrid[0:H0, 0:W0]
    x = (uu - K[0, 2]) / K[0, 0] * dep
    y = (vv - K[1, 2]) / K[1, 1] * dep
    P = np.stack([x, y, dep], -1) @ T[:3, :3].T + T[:3, 3]
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (dep > 0.05) & (X > X0) & (X < X0 + 0.84) & (Y > Y0) & (Y < Y0 + 0.90) \
        & (Z > TABLE_Z - 0.02) & (Z < 1.45)
    xi = ((X[m] - X0) / CELL).astype(int)
    yi = ((Y[m] - Y0) / CELL).astype(int)
    H = np.full((NX, NY), -1.0)
    C = np.zeros((NX, NY, 3))
    z = Z[m]
    order = np.argsort(z)
    H[xi[order], yi[order]] = z[order]
    C[xi[order], yi[order]] = rgb[m][order]
    return H, C


def _comps(mask):
    lab = np.zeros(mask.shape, int)
    cur = 0
    nx, ny = mask.shape
    idx = np.argwhere(mask)
    for i0, j0 in idx:
        if lab[i0, j0]:
            continue
        cur += 1
        st = [(int(i0), int(j0))]
        lab[i0, j0] = cur
        while st:
            a, b = st.pop()
            for p in (a - 1, a, a + 1):
                for q in (b - 1, b, b + 1):
                    if 0 <= p < nx and 0 <= q < ny and mask[p, q] and not lab[p, q]:
                        lab[p, q] = cur
                        st.append((p, q))
    return lab, cur


def blobs(H, C, lo, hi, minn):
    band = (H > TABLE_Z + lo) & (H < TABLE_Z + hi)
    lab, n = _comps(band)
    out = []
    for k in range(1, n + 1):
        sel = lab == k
        if sel.sum() < minn:
            continue
        ii, jj = np.nonzero(sel)
        x = ii * CELL + X0
        y = jj * CELL + Y0
        out.append(dict(n=int(sel.sum()), cx=float(0.5 * (x.min() + x.max())),
                        cy=float(0.5 * (y.min() + y.max())),
                        x0=float(x.min()), x1=float(x.max()),
                        y0=float(y.min()), y1=float(y.max()),
                        dx=float(x.max() - x.min()), dy=float(y.max() - y.min()),
                        top=float(H[sel].max() - TABLE_Z),
                        rgb=C[sel].mean(0)))
    return out


def _merge(bs, gap=0.05):
    """Union blobs whose bounding boxes are within `gap` of each other."""
    groups = []
    for b in bs:
        hit = None
        for g in groups:
            for o in g:
                if (max(b["x0"], o["x0"]) - min(b["x1"], o["x1"]) < gap
                        and max(b["y0"], o["y0"]) - min(b["y1"], o["y1"]) < gap):
                    hit = g
                    break
            if hit:
                break
        if hit is None:
            groups.append([b])
        else:
            hit.append(b)
    out = []
    for g in groups:
        x0 = min(o["x0"] for o in g)
        x1 = max(o["x1"] for o in g)
        y0 = min(o["y0"] for o in g)
        y1 = max(o["y1"] for o in g)
        w = float(sum(o["n"] for o in g))
        out.append(dict(n=int(w), cx=0.5 * (x0 + x1), cy=0.5 * (y0 + y1),
                        x0=x0, x1=x1, y0=y0, y1=y1, dx=x1 - x0, dy=y1 - y0,
                        top=max(o["top"] for o in g),
                        rgb=sum(o["rgb"] * o["n"] for o in g) / w))
    return out


def perceive(H, C):
    """Return (target_bowl, ramekin, other_bowls, plate) in base xy."""
    # bowls: rim tops at TABLE+0.049; the ramekin tops out at +0.041
    bw = [b for b in blobs(H, C, 0.0445, 0.10, 18)
          if b["cx"] > -0.35 and max(b["dx"], b["dy"]) < 0.16]
    bw = [b for b in _merge(bw) if 0.075 < max(b["dx"], b["dy"]) < 0.16]
    # ramekin: straight-walled cup, top +0.041, bright uniform grey
    rk = [b for b in blobs(H, C, 0.0355, 0.0425, 18)
          if b["cx"] > -0.35 and b["rgb"][0] > 0.54 and max(b["dx"], b["dy"]) < 0.13]
    rk = [b for b in _merge(rk, 0.02) if 0.05 < max(b["dx"], b["dy"]) < 0.13]
    rk.sort(key=lambda b: -b["n"])
    # plate: flat disc, bright, wide
    pl = [b for b in blobs(H, C, 0.012, 0.024, 60)
          if max(b["dx"], b["dy"]) > 0.10 and b["rgb"][0] > 0.50]
    pl.sort(key=lambda b: -b["n"])
    ram = rk[0] if rk else None
    if ram is not None and bw:
        bw.sort(key=lambda b: (b["cx"] - ram["cx"]) ** 2 + (b["cy"] - ram["cy"]) ** 2)
    tgt = bw[0] if bw else None
    return tgt, ram, bw[1:], (pl[0] if pl else None)


R_DOWN = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
TIP_OFF = 0.0058
BOWL_H = 0.0495
RG = 0.046
GRASP_DEPTH = 0.020
PLACE_CLEAR = 0.004


def goto(api, xyz, secs=2.0, tag="", corr=True):
    T = np.asarray(xyz, float)
    r = api.move(T, R_DOWN, seconds=secs)
    e = np.asarray(api.eef(), float)
    api.log("  mv%s res %.4f eef %s" % (tag, r, e.round(4).tolist()))
    if corr:
        b = T - e
        if 0.002 < np.linalg.norm(b) < 0.06:
            r = api.move(T + b, R_DOWN, seconds=1.0)
            e = np.asarray(api.eef(), float)
            api.log("  mv%s+ res %.4f eef %s" % (tag, r, e.round(4).tolist()))
    return e


def run(api):
    H, C = height_map(api)
    tgt, ram, oth, plate = perceive(H, C)
    for nm, b in (("TGT", tgt), ("RAM", ram), ("PLT", plate)):
        api.log("%s %s" % (nm, None if b is None else
                           "c(%.4f,%.4f) d(%.3f,%.3f) top+%.4f n%d"
                           % (b["cx"], b["cy"], b["dx"], b["dy"], b["top"], b["n"])))
    if tgt is None or plate is None:
        api.log("ABORT missing object")
        return

    bx, by = tgt["cx"], tgt["cy"]
    rim = TABLE_Z + tgt["top"]
    gz = rim - GRASP_DEPTH + TIP_OFF
    gx, gy = bx, by + RG

    api.grip(0.08)
    goto(api, [gx, gy, TABLE_Z + 0.14], 2.0, "hov")
    e = goto(api, [gx, gy, gz], 1.5, "dsc")
    grasp_eef = e.copy()

    api.grip(0.0)
    api.log("CLOSED %s" % api.gripper())

    goto(api, [gx, gy, TABLE_Z + 0.20], 1.5, "lift", corr=False)
    lift_eef = np.asarray(api.eef(), float)
    api.log("AFTLIFT %s eef %s" % (api.gripper(), lift_eef.round(4).tolist()))

    # the bowl centre trails the eef by the grasp radius, in the -y direction
    px, py = plate["cx"], plate["cy"] + RG
    ptop = TABLE_Z + plate["top"]
    place_z = ptop + PLACE_CLEAR + (BOWL_H - GRASP_DEPTH) + TIP_OFF
    goto(api, [px, py, TABLE_Z + 0.22], 2.5, "carry")
    e = goto(api, [px, py, place_z], 1.5, "place")
    api.log("place cmd(%.4f,%.4f,%.4f) eef %s grip %s"
            % (px, py, place_z, e.round(4).tolist(), api.gripper()))

    api.grip(0.08)
    api.settle(0.4)
    goto(api, [px, py, TABLE_Z + 0.22], 1.5, "up", corr=False)
    goto(api, [-0.21, 0.02, TABLE_Z + 0.30], 2.5, "home", corr=False)
    api.settle(0.3)

    H3, C3 = height_map(api)
    t2, r2, o2, p2 = perceive(H3, C3)
    api.log("FINAL tgt %s" % (None if t2 is None else
                              "c(%.4f,%.4f) d(%.3f,%.3f) top+%.4f"
                              % (t2["cx"], t2["cy"], t2["dx"], t2["dy"], t2["top"])))
    api.log("FINAL plate %s" % (None if p2 is None else
                                "c(%.4f,%.4f) top+%.4f" % (p2["cx"], p2["cy"], p2["top"])))
    api.log("FINAL oth %s" % [(round(b["cx"], 3), round(b["cy"], 3), round(b["top"], 3))
                              for b in o2])
