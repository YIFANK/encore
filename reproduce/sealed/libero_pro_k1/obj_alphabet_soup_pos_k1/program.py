"""c2k1clean / obj_alphabet_soup_pos_k1  --  v3

Pick the alphabet soup can (blue-labelled can) and drop it in the basket.

Perception: one cam_high RGB-D capture -> base-frame point cloud -> top-down
height map over the table -> connected components.  The arm is removed by a
hard z ceiling (every table prop measured on the debug seeds tops out below
0.15 m, the arm reaches 0.48 m).  Among can-sized components the soup is the
one whose upper band is blue; the basket is the largest component.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {
        "source": "debug seeds 51/55/59/63 cam_high depth: median base-frame z "
                  "over the table region is 0.0015 m on every seed",
        "allowed": True},
    "XLO/XHI/YLO/YHI": {
        "source": "debug seeds 51-63: every table prop deprojects inside "
                  "x in [-0.25,0.20], y in [-0.30,0.36]; bounds padded from those "
                  "observations to crop the walls/floor",
        "allowed": True},
    "Z_CEIL": {
        "source": "debug-seed measurement: tallest prop top = 0.147 m (bottle), "
                  "basket 0.142, cans 0.081; the arm column tops at 0.482 m",
        "allowed": True},
    "Z_FLOOR": {
        "source": "debug-seed measurement: table at 0.0015 m; 0.015 m above it "
                  "separates props from the table plane and its painted lines",
        "allowed": True},
    "CELL": {"source": "generic: 1 cm top-down grid, finer than the 6.4 cm can "
                       "footprint measured on the debug seeds", "allowed": True},
    "CAN_TOP_LO/CAN_TOP_HI": {
        "source": "debug-seed measurement: both cans top at 0.081 m; the other "
                  "props are at 0.019/0.020/0.140/0.142/0.147 m",
        "allowed": True},
    "BLUE_PX": {
        "source": "debug-seed measurement: the soup can top band has chroma "
                  "b-r = +0.103 while the other can is -0.185; 0.05 sits "
                  "between them", "allowed": True},
    "CAN_W_LO/CAN_W_HI": {
        "source": "debug-seed measurement: can footprint 0.064 x 0.066 m",
        "allowed": True},
    "BAND_M": {
        "source": "debug-seed measurement: the blue label occupies the upper "
                  "~3 cm of the can; a 0.030 m top band separates the blue can "
                  "(chroma b 0.393) from the red/green can (chroma r 0.443)",
        "allowed": True},
    "GRASP_BELOW_TOP": {
        "source": "pack demo0 keyframe t=42 closes at ee z=0.0447 while the same "
                  "can measures top 0.081 on the debug seeds -> 0.036 below top",
        "allowed": True},
    "CARRY_Z": {
        "source": "pack demo0 ee_path6 transport altitude 0.26-0.31 m",
        "allowed": True},
    "RELEASE_ABOVE_BASKET": {
        "source": "pack demo0 keyframe t=128 releases at ee z=0.1525 with the "
                  "basket measuring top 0.142 on the debug seeds; padded to "
                  "0.045 so the hanging can clears the rim",
        "allowed": True},
    "R_DOWN": {
        "source": "api.tool_rotation() at episode start on debug seed 51 is "
                  "straight-down to within 3 deg; generic controller mechanics",
        "allowed": True},
    "OPEN_W/CLOSE_W": {
        "source": "api.gripper() reports width 0.0778 m open on debug seeds; "
                  "api.grip doc: <0.025 closes", "allowed": True},
}

TABLE_Z = 0.0015
XLO, XHI = -0.42, 0.32
YLO, YHI = -0.48, 0.48
Z_FLOOR = TABLE_Z + 0.015
Z_CEIL = 0.20
CELL = 0.01
CAN_TOP_LO, CAN_TOP_HI = 0.050, 0.125
CAN_W_LO, CAN_W_HI = 0.035, 0.095
BAND_M = 0.030
GRASP_BELOW_TOP = 0.036
CARRY_Z = 0.28
RELEASE_ABOVE_BASKET = 0.045
OPEN_W = 0.08
CLOSE_W = 0.0

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


# --------------------------------------------------------------- perception
def _label(occ):
    lab = np.zeros(occ.shape, np.int32)
    cur = 0
    H, W = occ.shape
    for i in range(H):
        for j in range(W):
            if not occ[i, j] or lab[i, j]:
                continue
            cur += 1
            lab[i, j] = cur
            st = [(i, j)]
            while st:
                a, b = st.pop()
                for da in (-1, 0, 1):
                    for db in (-1, 0, 1):
                        p, q = a + da, b + db
                        if 0 <= p < H and 0 <= q < W and occ[p, q] and not lab[p, q]:
                            lab[p, q] = cur
                            st.append((p, q))
    return lab, cur


def _cloud(fr):
    dep = np.asarray(fr.depth, dtype=np.float64)
    K = np.asarray(fr.intrinsics, float)
    T = np.asarray(fr.t_base_cam, float)
    h, w = dep.shape
    sx = w / (2.0 * K[0, 2])
    sy = h / (2.0 * K[1, 2])
    fx, fy, cx, cy = K[0, 0] * sx, K[1, 1] * sy, K[0, 2] * sx, K[1, 2] * sy
    vv, uu = np.mgrid[0:h, 0:w]
    z = dep
    P = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z, np.ones_like(z)], -1)
    return (P @ T.T)[..., :3]


def props(api, fr):
    W3 = _cloud(fr)
    x, y, z = W3[..., 0], W3[..., 1], W3[..., 2]
    dep = np.asarray(fr.depth, float)
    m = (np.isfinite(dep) & (dep > 0) & (x > XLO) & (x < XHI) &
         (y > YLO) & (y < YHI) & (z > Z_FLOOR) & (z < Z_CEIL))
    nx = int(round((XHI - XLO) / CELL))
    ny = int(round((YHI - YLO) / CELL))
    ii = np.clip(((x - XLO) / CELL).astype(int), 0, nx - 1)
    jj = np.clip(((y - YLO) / CELL).astype(int), 0, ny - 1)
    hm = np.full((nx, ny), -1.0)
    pix = np.argwhere(m)
    for v, u in pix:
        a, b = ii[v, u], jj[v, u]
        if z[v, u] > hm[a, b]:
            hm[a, b] = z[v, u]
    lab, n = _label(hm > 0)
    cell_of = lab[ii, jj]
    rgb = np.asarray(fr.rgb, float)
    out = []
    for c in range(1, n + 1):
        ncell = int((lab == c).sum())
        if ncell < 5:
            continue
        sel = m & (cell_of == c)
        if int(sel.sum()) < 25:
            continue
        xs, ys, zs = x[sel], y[sel], z[sel]
        cc = rgb[sel]
        ch = cc / (cc.sum(1, keepdims=True) + 1e-6)
        top = float(np.percentile(zs, 99))
        band = zs > top - BAND_M
        chb = ch[band].mean(0) if int(band.sum()) > 10 else ch.mean(0)
        bluepx = band & (ch[:, 2] - ch[:, 0] > 0.05)
        if int(bluepx.sum()) >= 10:
            bx = 0.5 * float(xs[bluepx].min() + xs[bluepx].max())
            by = 0.5 * float(ys[bluepx].min() + ys[bluepx].max())
        else:
            bx = 0.5 * float(xs.min() + xs.max())
            by = 0.5 * float(ys.min() + ys.max())
        out.append(dict(
            ncell=ncell, top=top,
            xmin=float(xs.min()), xmax=float(xs.max()),
            ymin=float(ys.min()), ymax=float(ys.max()),
            cx=0.5 * float(xs.min() + xs.max()),
            cy=0.5 * float(ys.min() + ys.max()),
            w=float(xs.max() - xs.min()), h=float(ys.max() - ys.min()),
            bx=bx, by=by,
            blue=float(chb[2] - chb[0]), chb=[round(float(v), 3) for v in chb]))
        out[-1]["gx"] = out[-1]["cx"]
        out[-1]["gy"] = out[-1]["cy"]
    for p in sorted(out, key=lambda q: -q["ncell"]):
        api.log("PROP ncell=%d top=%.3f c=(%.3f,%.3f) wh=(%.3f,%.3f) blue=%+.3f chb=%s"
                % (p["ncell"], p["top"], p["cx"], p["cy"], p["w"], p["h"],
                   p["blue"], p["chb"]))
    return out


def pick_soup(ps):
    """Graded search for the blue-labelled can.  Tier 1 is the nominal
    can-sized component; the looser tiers only fire if a prop fuses with a
    neighbour or the height map is clipped."""
    t1 = [p for p in ps
          if CAN_TOP_LO < p["top"] < CAN_TOP_HI
          and CAN_W_LO < p["w"] < CAN_W_HI and CAN_W_LO < p["h"] < CAN_W_HI]
    if t1:
        return max(t1, key=lambda p: p["blue"]), "can-sized(%d)" % len(t1)
    t2 = [p for p in ps if CAN_TOP_LO < p["top"] < CAN_TOP_HI]
    if t2:
        p = max(t2, key=lambda q: q["blue"])
        p = dict(p, gx=p["bx"], gy=p["by"])
        return p, "can-height-fused(%d)" % len(t2)
    t3 = [p for p in ps if p["top"] > 0.04] or ps
    p = max(t3, key=lambda q: q["blue"])
    return dict(p, gx=p["bx"], gy=p["by"]), "loose(%d)" % len(t3)


# ------------------------------------------------------------------ motion
def goto(api, xyz, seconds=2.0):
    return api.move([float(v) for v in xyz], rotation=R_DOWN, seconds=seconds)


def run(api):
    api.log("instr=%r" % api.instruction())
    fr = api.capture("cam_high")
    ps = props(api, fr)
    if not ps:
        api.log("ABORT no props")
        return

    soup, tier = pick_soup(ps)
    api.log("tier=%s" % tier)
    basket = max(ps, key=lambda p: p["ncell"])
    api.log("SOUP c=(%.3f,%.3f) top=%.3f blue=%+.3f" %
            (soup["cx"], soup["cy"], soup["top"], soup["blue"]))
    api.log("BASKET c=(%.3f,%.3f) top=%.3f ncell=%d" %
            (basket["cx"], basket["cy"], basket["top"], basket["ncell"]))

    gx, gy = soup["gx"], soup["gy"]
    gz = soup["top"] - GRASP_BELOW_TOP

    api.grip(OPEN_W)
    goto(api, [gx, gy, max(CARRY_Z, soup["top"] + 0.12)], 2.0)
    r = goto(api, [gx, gy, soup["top"] + 0.06], 1.5)
    api.log("hover residual=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    r = goto(api, [gx, gy, gz], 2.0)
    api.log("descend residual=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(CLOSE_W)
    api.settle(0.3)
    g = api.gripper()
    api.log("closed %s" % g)

    goto(api, [gx, gy, CARRY_Z], 2.0)
    g = api.gripper()
    api.log("after lift %s eef=%s" % (g, np.round(api.eef(), 4).tolist()))

    if g["effort"] < 2.5 or g["width_m"] < 0.02:
        api.log("RETRY grasp (effort=%.2f width=%.4f)" % (g["effort"], g["width_m"]))
        api.grip(OPEN_W)
        fr2 = api.capture("cam_high")
        ps2 = props(api, fr2)
        if ps2:
            s2, t2 = pick_soup(ps2)
            api.log("retry tier=%s c=(%.3f,%.3f) top=%.3f" %
                    (t2, s2["gx"], s2["gy"], s2["top"]))
            gx, gy = s2["gx"], s2["gy"]
            goto(api, [gx, gy, s2["top"] + 0.06], 1.5)
            goto(api, [gx, gy, s2["top"] - GRASP_BELOW_TOP], 2.0)
            api.grip(CLOSE_W)
            api.settle(0.3)
            goto(api, [gx, gy, CARRY_Z], 2.0)
            api.log("after retry lift %s" % (api.gripper(),))

    bx, by = basket["cx"], basket["cy"]
    goto(api, [bx, by, CARRY_Z], 2.5)
    rz = basket["top"] + RELEASE_ABOVE_BASKET
    r = goto(api, [bx, by, rz], 2.0)
    api.log("over basket residual=%.4f eef=%s g=%s"
            % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.5)
    goto(api, [bx, by, CARRY_Z], 1.5)
    api.settle(0.5)
    api.log("done eef=%s" % np.round(api.eef(), 4).tolist())
