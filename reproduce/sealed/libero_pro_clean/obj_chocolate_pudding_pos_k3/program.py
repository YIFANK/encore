"""c2clean obj_chocolate_pudding_pos_k3 -- v1 (candidate A: the short brown box).

Mechanism is copied from the pack demos (approach high -> descend -> close ->
lift -> traverse to the basket -> descend -> open).  The TARGET, however, is
perceived: the pack demos reach y ~= -0.25 (the "BBQ"-labelled bottle), while
the instruction names the chocolate pudding.  This version targets the only
SHORT object on the table (top <= 7 cm) -- the small brown box.
"""
import numpy as np

PROVENANCE = {
    "CAM": {"source": "api.capture('cam_high').intrinsics / .t_base_cam (runtime)",
            "allowed": True},
    "Z_TABLE": {"source": "debug seeds 51-65: deprojected cam_high cloud, modal "
                          "plane at z=0.001 in base frame", "allowed": True},
    "SHORT_BAND": {"source": "debug seeds 51-65 height map: the brown box tops out "
                             "at 0.030 m, every other prop (bottles, carton, can, "
                             "basket rim) at 0.14-0.15 m", "allowed": True},
    "GRASP_Z": {"source": "pack.json demo keyframes t=54/48/53 ee z = 0.0114 / "
                          "0.0092 / 0.0094", "allowed": True},
    "LIFT_Z": {"source": "pack.json demo0 ee_path6 t=80 z = 0.2059", "allowed": True},
    "CARRY_Z": {"source": "pack.json demo0 ee_path6 t=110-120 z = 0.263", "allowed": True},
    "RELEASE_Z": {"source": "pack.json demo release keyframes z = 0.1635 / 0.1751 / "
                            "0.1656", "allowed": True},
    "OPEN_W": {"source": "api.gripper() at episode start on debug seeds = 0.0778 m",
               "allowed": True},
    "HOLD_W": {"source": "pack.json carry keyframes gripper_state sum = 0.0465 m",
               "allowed": True},
}

Z_TABLE = 0.0
SHORT_LO, SHORT_HI = 0.012, 0.070
GRASP_Z = 0.010
LIFT_Z = 0.21
CARRY_Z = 0.26
RELEASE_Z = 0.170
OPEN_W = 0.080
CELL = 0.006
XR = (-0.40, 0.30)
YR = (-0.45, 0.45)


# ---------------------------------------------------------------- perception

def _cloud(f, step=2):
    rgb = np.asarray(f.rgb, np.float32)[::step, ::step]
    d = np.asarray(f.depth, np.float32)[::step, ::step]
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H = d.shape[0]
    vv, uu = np.mgrid[0:H, 0:H].astype(np.float32)
    x = (uu * step - K[0, 2]) / K[0, 0] * d
    y = (vv * step - K[1, 2]) / K[1, 1] * d
    P = np.stack([x, y, d], -1)
    return P @ T[:3, :3].T + T[:3, 3], rgb


def _heightmap(B):
    nx = int((XR[1] - XR[0]) / CELL)
    ny = int((YR[1] - YR[0]) / CELL)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    ok = ((X > XR[0]) & (X < XR[1] - 1e-6) & (Y > YR[0]) & (Y < YR[1] - 1e-6)
          & (Z > 0.008) & (Z < 0.30))
    ix = np.clip(((X - XR[0]) / CELL).astype(int), 0, nx - 1)
    iy = np.clip(((Y - YR[0]) / CELL).astype(int), 0, ny - 1)
    hm = np.zeros((nx, ny), np.float32)
    flat = ix[ok] * ny + iy[ok]
    np.maximum.at(hm.reshape(-1), flat, Z[ok])
    return hm


def _components(mask, min_cells=8):
    """8-connected components, numpy-only label propagation."""
    nx, ny = mask.shape
    lab = np.where(mask, np.arange(nx * ny).reshape(nx, ny) + 1, 0)
    for _ in range(400):
        prev = lab
        m = lab.copy()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                s = np.roll(np.roll(lab, dx, 0), dy, 1)
                if dx == 1:
                    s[0, :] = 0
                elif dx == -1:
                    s[-1, :] = 0
                if dy == 1:
                    s[:, 0] = 0
                elif dy == -1:
                    s[:, -1] = 0
                m = np.maximum(m, s)
        lab = np.where(mask, m, 0)
        if np.array_equal(lab, prev):
            break
    out = []
    for v in np.unique(lab):
        if v == 0:
            continue
        sel = lab == v
        if sel.sum() < min_cells:
            continue
        out.append(sel)
    return out


def _bbox(sel):
    a, b = np.nonzero(sel)
    return (XR[0] + (a.min() + 0.0) * CELL, XR[0] + (a.max() + 1.0) * CELL,
            YR[0] + (b.min() + 0.0) * CELL, YR[0] + (b.max() + 1.0) * CELL)


def perceive(api):
    f = api.capture("cam_high")
    B, rgb = _cloud(f)
    hm = _heightmap(B)

    # --- target: the only SHORT free-standing object on the table ----------
    short = (hm > SHORT_LO) & (hm < SHORT_HI)
    comps = _components(short, min_cells=6)
    cands = []
    for sel in comps:
        x0, x1, y0, y1 = _bbox(sel)
        cands.append((sel.sum(), x0, x1, y0, y1, float(hm[sel].max())))
    cands.sort(key=lambda c: -c[0])
    for c in cands[:6]:
        api.log("short-cand cells=%d x[%.3f %.3f] y[%.3f %.3f] top %.3f" % c)
    # merge every short component that lies within 8 cm of the biggest one:
    # the box's top face and front face can split across a depth dropout.
    n, x0, x1, y0, y1, top = cands[0]
    for c in cands[1:]:
        if (c[1] < x1 + 0.05 and c[2] > x0 - 0.05
                and c[3] < y1 + 0.05 and c[4] > y0 - 0.05):
            x0, x1 = min(x0, c[1]), max(x1, c[2])
            y0, y1 = min(y0, c[3]), max(y1, c[4])
            top = max(top, c[5])
    tgt = (0.5 * (x0 + x1), 0.5 * (y0 + y1), top)
    api.log("TARGET x %.4f y %.4f top %.3f  xspan %.3f yspan %.3f"
            % (tgt[0], tgt[1], tgt[2], x1 - x0, y1 - y0))

    # --- basket: the big tall structure on the +y side --------------------
    tall = (hm > 0.10) & (hm < 0.20)
    tall[:, :int((0.12 - YR[0]) / CELL)] = False
    bcomps = _components(tall, min_cells=40)
    bcomps.sort(key=lambda s: -s.sum())
    bx0, bx1, by0, by1 = _bbox(bcomps[0])
    bas = (0.5 * (bx0 + bx1), 0.5 * (by0 + by1))
    api.log("BASKET x %.4f y %.4f  x[%.3f %.3f] y[%.3f %.3f]"
            % (bas[0], bas[1], bx0, bx1, by0, by1))
    return tgt, bas


# ---------------------------------------------------------------- behaviour

def run(api):
    api.log("instruction=%r" % api.instruction())
    tgt, bas = perceive(api)
    tx, ty, ttop = tgt
    bx, by = bas

    api.grip(OPEN_W)
    api.move([tx, ty, LIFT_Z], seconds=2.5)
    api.log("above: eef=%s" % np.asarray(api.eef()).round(4).tolist())
    api.move([tx, ty, 0.06], seconds=2.0)
    r = api.move([tx, ty, GRASP_Z], seconds=2.0)
    api.log("at grasp: eef=%s residual=%.4f"
            % (np.asarray(api.eef()).round(4).tolist(), r))

    api.grip(0.0)
    api.settle(0.5)
    g = api.gripper()
    api.log("closed: %s" % g)

    api.move([tx, ty, LIFT_Z], seconds=2.5)
    api.settle(0.3)
    g2 = api.gripper()
    api.log("lifted: %s eef=%s" % (g2, np.asarray(api.eef()).round(4).tolist()))

    api.move([bx, by, CARRY_Z], seconds=3.0)
    g3 = api.gripper()
    api.log("over basket: %s eef=%s" % (g3, np.asarray(api.eef()).round(4).tolist()))
    api.move([bx, by, RELEASE_Z], seconds=2.0)
    api.log("at release: eef=%s" % np.asarray(api.eef()).round(4).tolist())
    api.grip(OPEN_W)
    api.settle(0.6)
    api.move([bx, by, CARRY_Z], seconds=2.0)
    api.settle(0.5)
    return "v1 box hold=%.4f/%.4f/%.4f" % (g["width_m"], g2["width_m"], g3["width_m"])
