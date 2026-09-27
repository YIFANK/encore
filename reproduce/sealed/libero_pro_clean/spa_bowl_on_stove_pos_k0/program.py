"""c2clean spa_bowl_on_stove_pos_k0 -- v3

Intent: "pick up the black bowl on the stove and place it on the plate".

Mechanism (all re-derived on debug seeds 51/53/55/57 from cam_high RGB-D):
  * The scene holds three bowls: one on a tall cabinet top (z_top ~1.18), one
    standing on the table (rim ~table+0.043) and one standing on a raised flat
    slab with a black back panel and a burner disc -- the stove (slab
    ~table+0.026, bowl rim ~table+0.079).  Inside the height band
    [table+0.033, table+0.130] and the window x in [-0.35,0.22] the stove bowl
    is the cluster with the HIGHEST top, so "max z_top" names it.
  * The bowl's outer rim diameter is ~0.107 m, wider than the gripper's full
    opening (0.0778 m), so it cannot be straddled.  It is pinched across the
    rim wall: the jaws close along base +/-y (derived from the wrist camera
    extrinsics: wrist-image +x maps to base -y, and the two finger blobs are
    separated along wrist-image x), so the eef is placed at
    (bowl_cx, bowl_cy + r_rim) and driven down until the fingertips sit a
    little below the rim top.
  * The fingertip-to-eef z offset was measured in v2 by pressing the open
    gripper into bare table: the eef stalls 0.0071 above the table plane on
    every debug seed, so it is a constant (FINGER_OFFSET) here.
  * v2 receipt: a single api.move is time-budgeted, so the 0.135 m descent
    stopped ~8 mm high and the fingertips bit the very top of the rim, where
    the wall is thinnest; the close gap then split successes (>=0.0126) from
    failures (<=0.0120) exactly.  v3 hovers only 0.05 above the rim, bites
    0.025 below the rim top where the wall sits at r_fit-0.008, and re-issues
    every critical move until the pose is actually held.
  * Carrying: the bowl centre trails the eef by exactly the pinch offset, so
    the release pose is (plate_cx, plate_cy + r_rim).  The held bowl's base
    hangs (bowl height - grasp depth) below the fingertips.
"""
import numpy as np

PROVENANCE = {
    "R_DOWN": {"source": "generic controller mechanics: straight-down tool frame; "
                          "matches api.tool_rotation() at reset on debug seeds 51-57",
               "allowed": True},
    "WIN_X": {"source": "debug seeds 51-57 cam_high: table props lie in x[-0.32,0.15]; "
                        "robot base/mount occupies x<-0.36", "allowed": True},
    "WIN_Y": {"source": "debug seeds 51-57 cam_high: props lie in y[-0.20,0.26]",
              "allowed": True},
    "TABLE_BAND": {"source": "debug seeds 51-57 cam_high depth histogram: dominant "
                             "plane z=0.901", "allowed": True},
    "BAND_LO": {"source": "debug seed measurement: stove slab top = table+0.026, "
                          "plate/box tops = table+0.019; +0.033 clears both",
                "allowed": True},
    "BAND_HI": {"source": "debug seed measurement: tallest table-level prop "
                          "(stove bowl rim) = table+0.079; cabinet top = table+0.226",
                "allowed": True},
    "RING_DZ": {"source": "debug seed radial profile of the stove bowl: rim wall "
                          "occupies the top 0.010 m", "allowed": True},
    "GRASP_DEPTH": {"source": "v2 receipts on seeds 51-65: fingertips that stopped "
                              "0.007 below the rim top gave a 0.010-0.014 close gap "
                              "and half the bites squirted out; the interior is "
                              "0.045 deep so 0.025 down is still clear of the floor",
                    "allowed": True},
    "PINCH_SHRINK": {"source": "debug seed radial profile: the rim wall flares -- "
                               "wall mid-radius is 0.0527 at the rim top but 0.045 "
                               "at 0.025 below it, i.e. r_fit - 0.008",
                     "allowed": True},
    "FINGER_OFFSET": {"source": "v2 debug seeds 51-65: pressing the open gripper into "
                                "bare table stalled the eef 0.0071 above the table "
                                "plane on all 8 seeds", "allowed": True},
    "HOLD_W": {"source": "v2 debug seeds 51-65: post-lift gripper width is bimodal, "
                         "0.0017 when the bowl was lost and 0.0033-0.0037 when it was "
                         "held; 0.0025 separates them", "allowed": True},
    "OPEN_W": {"source": "api.gripper() at reset on debug seeds = 0.0778 m",
               "allowed": True},
    "PLATE_BAND": {"source": "debug seed measurement: plate top = table+0.019, "
                             "footprint 0.134 m across; cookie box 0.08 m across",
                   "allowed": True},
    "CLEAR_R": {"source": "debug seed measurement: gripper span 0.078 -> a 0.07 m "
                          "clear radius is enough to press on bare table",
                "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
WIN_X = (-0.35, 0.22)
WIN_Y = (-0.35, 0.35)
RES = 0.005
BAND_LO, BAND_HI = 0.033, 0.130
RING_DZ = 0.010
GRASP_DEPTH = 0.025
PINCH_SHRINK = 0.008
FINGER_OFFSET = 0.007
HOLD_W = 0.0025
OPEN_W = 0.078
CLEAR_R = 0.07


# ---------------------------------------------------------------- perception
def _cloud(f):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.asarray(f.depth, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    good = np.isfinite(d) & (d > 0.2) & (d < 5.0)
    dd = np.where(good, d, 1.0)
    x = (uu - K[0, 2]) / K[0, 0] * dd
    y = (vv - K[1, 2]) / K[1, 1] * dd
    P = np.stack([x, y, dd], -1) @ T[:3, :3].T + T[:3, 3]
    return P, good


def _grid(P, rgb, good):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = good & (X > WIN_X[0]) & (X < WIN_X[1]) & (Y > WIN_Y[0]) & (Y < WIN_Y[1]) \
        & (Z > 0.5) & (Z < 1.6)
    nx = int((WIN_X[1] - WIN_X[0]) / RES) + 1
    ny = int((WIN_Y[1] - WIN_Y[0]) / RES) + 1
    H = np.full((nx, ny), -9.0)
    C = np.zeros((nx, ny, 3))
    xi = ((X[m] - WIN_X[0]) / RES).astype(int)
    yi = ((Y[m] - WIN_Y[0]) / RES).astype(int)
    zs = Z[m]
    cs = np.asarray(rgb, float)[m]
    o = np.argsort(zs)
    H[xi[o], yi[o]] = zs[o]
    C[xi[o], yi[o]] = cs[o]
    return H, C


def _cell_xy(i, j):
    return WIN_X[0] + i * RES, WIN_Y[0] + j * RES


def _components(M):
    lab = np.zeros(M.shape, int)
    cur = 0
    nx, ny = M.shape
    for i in range(nx):
        for j in range(ny):
            if M[i, j] and lab[i, j] == 0:
                cur += 1
                st = [(i, j)]
                lab[i, j] = cur
                while st:
                    a, b = st.pop()
                    for p in (a - 1, a, a + 1):
                        for q in (b - 1, b, b + 1):
                            if 0 <= p < nx and 0 <= q < ny and M[p, q] and lab[p, q] == 0:
                                lab[p, q] = cur
                                st.append((p, q))
    return lab, cur


def _describe(H, C, lab, k):
    m = lab == k
    xs, ys = np.where(m)
    zs = H[m]
    ztop = float(zs.max())
    ring = zs > ztop - RING_DZ
    rx = WIN_X[0] + xs[ring] * RES
    ry = WIN_Y[0] + ys[ring] * RES
    ax = WIN_X[0] + xs * RES
    ay = WIN_Y[0] + ys * RES
    return dict(
        n=int(m.sum()), ztop=ztop,
        cx=float((rx.min() + rx.max()) / 2.0), cy=float((ry.min() + ry.max()) / 2.0),
        dx=float(rx.max() - rx.min()), dy=float(ry.max() - ry.min()),
        fx=float(ax.max() - ax.min()), fy=float(ay.max() - ay.min()),
        acx=float((ax.min() + ax.max()) / 2.0), acy=float((ay.min() + ay.max()) / 2.0),
        rgb=[float(v) for v in C[m].mean(0)])


def _refit(api, H, seed):
    """Re-gather the whole rim ring (a component can be split by an occluder)
    and fit a circle to it -- a partial arc still pins the centre."""
    nx, ny = H.shape
    zlo = seed["ztop"] - RING_DZ
    xs, ys = [], []
    for i in range(nx):
        for j in range(ny):
            if H[i, j] < zlo or H[i, j] > seed["ztop"] + 0.006:
                continue
            x, y = _cell_xy(i, j)
            if np.hypot(x - seed["cx"], y - seed["cy"]) < 0.10:
                xs.append(x); ys.append(y)
    xs = np.array(xs); ys = np.array(ys)
    out = dict(seed)
    out["cx"] = float((xs.min() + xs.max()) / 2.0)
    out["cy"] = float((ys.min() + ys.max()) / 2.0)
    out["dx"] = float(xs.max() - xs.min())
    out["dy"] = float(ys.max() - ys.min())
    out["fit"] = None
    if len(xs) >= 12:
        A = np.stack([xs, ys, np.ones_like(xs)], -1)
        b = xs ** 2 + ys ** 2
        try:
            sol = np.linalg.lstsq(A, b, rcond=None)[0]
            fx, fy = sol[0] / 2.0, sol[1] / 2.0
            fr = float(np.sqrt(max(sol[2] + fx * fx + fy * fy, 1e-9)))
            out["fit"] = (float(fx), float(fy), fr)
            api.log("RING n=%d bbox_c=(%.3f,%.3f) %.3fx%.3f  fit=(%.3f,%.3f) r=%.4f"
                    % (len(xs), out["cx"], out["cy"], out["dx"], out["dy"], fx, fy, fr))
            if 0.040 <= fr <= 0.070:
                out["cx"], out["cy"] = float(fx), float(fy)
                out["dx"] = out["dy"] = 2.0 * fr
        except Exception as exc:                       # pragma: no cover
            api.log("RINGFIT_FAIL %s" % exc)
    return out


def _table_z(H):
    v = H[H > 0.5]
    lo, hi = 0.80, 1.00
    v = v[(v > lo) & (v < hi)]
    hist, edges = np.histogram(v, bins=int((hi - lo) / 0.002))
    k = int(np.argmax(hist))
    return float((edges[k] + edges[k + 1]) / 2.0)


def _clear_spot(H, tz):
    """A table cell whose CLEAR_R neighbourhood is all at table level."""
    nx, ny = H.shape
    rad = int(CLEAR_R / RES)
    tall = H > tz + 0.012
    best, bestd = None, 1e9
    for i in range(rad, nx - rad):
        for j in range(rad, ny - rad):
            if H[i, j] < tz - 0.01 or H[i, j] > tz + 0.008:
                continue
            if tall[i - rad:i + rad + 1, j - rad:j + rad + 1].any():
                continue
            x, y = _cell_xy(i, j)
            d = (x + 0.08) ** 2 + (y - 0.05) ** 2
            if d < bestd:
                bestd, best = d, (x, y)
    return best


def perceive(api):
    f = api.capture("cam_high")
    P, good = _cloud(f)
    H, C = _grid(P, f.rgb, good)
    tz = _table_z(H)
    out = {"table_z": tz, "H": H, "C": C}

    M = (H > tz + BAND_LO) & (H < tz + BAND_HI)
    lab, n = _components(M)
    props = [_describe(H, C, lab, k) for k in range(1, n + 1)]
    props = [p for p in props if p["n"] >= 8]
    for p in sorted(props, key=lambda p: -p["ztop"]):
        api.log("PROP n=%d ztop=%.3f c=(%.3f,%.3f) ring=%.3fx%.3f foot=%.3fx%.3f rgb=%s"
                % (p["n"], p["ztop"], p["cx"], p["cy"], p["dx"], p["dy"],
                   p["fx"], p["fy"], [round(v) for v in p["rgb"]]))
    cands = [p for p in props if 0.06 <= max(p["dx"], p["dy"]) <= 0.16]
    seed = max(cands, key=lambda p: p["ztop"]) if cands else None
    out["bowl"] = _refit(api, H, seed) if seed is not None else None

    # plate: flat, wide, light-coloured disc just above the table
    M2 = (H > tz + 0.010) & (H < tz + 0.030)
    lab2, n2 = _components(M2)
    flats = [_describe(H, C, lab2, k) for k in range(1, n2 + 1)]
    flats = [p for p in flats if p["n"] >= 20]
    for p in sorted(flats, key=lambda p: -p["n"]):
        api.log("FLAT n=%d ztop=%.3f c=(%.3f,%.3f) foot=%.3fx%.3f rgb=%s"
                % (p["n"], p["ztop"], p["acx"], p["acy"], p["fx"], p["fy"],
                   [round(v) for v in p["rgb"]]))
    pl = [p for p in flats if max(p["fx"], p["fy"]) >= 0.095]
    out["plate"] = max(pl, key=lambda p: min(p["rgb"])) if pl else None
    return out


def support_z(H, tz, cx, cy, r):
    """Median height of the annulus just outside a prop -- what it stands on."""
    nx, ny = H.shape
    vals = []
    for i in range(nx):
        for j in range(ny):
            if H[i, j] < 0.5:
                continue
            x, y = _cell_xy(i, j)
            d = np.hypot(x - cx, y - cy)
            if r + 0.012 < d < r + 0.035 and H[i, j] < tz + 0.05:
                vals.append(H[i, j])
    return float(np.median(vals)) if vals else tz


# ------------------------------------------------------------------- motion
def goto(api, x, y, z, sec=1.6, tol=0.0, tag=""):
    """Drive to a pose; optionally re-issue until the pose is actually held
    (a single api.move is time-budgeted and can stop metres-short on a long leg)."""
    tgt = np.array([x, y, z], float)
    for k in range(3):
        api.move(tgt, R_DOWN, sec)
        e = np.asarray(api.eef(), float)
        err = float(np.linalg.norm(e - tgt))
        api.log("MOVE%s%s cmd=(%.3f,%.3f,%.3f) got=(%.3f,%.3f,%.3f) err=%.4f"
                % (tag, "" if k == 0 else "+%d" % k, x, y, z, e[0], e[1], e[2], err))
        if tol <= 0 or err <= tol:
            return e, err
        sec = 1.2
    return e, err


def run(api):
    api.log("INSTR %s" % api.instruction())
    s = perceive(api)
    tz = s["table_z"]
    api.log("TABLE_Z %.3f" % tz)
    if s["bowl"] is None or s["plate"] is None:
        api.log("ABORT bowl=%s plate=%s" % (s["bowl"], s["plate"]))
        return
    b, pl = s["bowl"], s["plate"]
    r_rim = max(b["dx"], b["dy"]) / 2.0
    sup = support_z(s["H"], tz, b["cx"], b["cy"], r_rim)
    b_h = b["ztop"] - sup
    fo = FINGER_OFFSET
    api.log("BOWL c=(%.3f,%.3f) r=%.4f ztop=%.3f support=%.3f height=%.3f"
            % (b["cx"], b["cy"], r_rim, b["ztop"], sup, b_h))
    api.log("PLATE c=(%.3f,%.3f) top=%.3f foot=%.3fx%.3f"
            % (pl["acx"], pl["acy"], pl["ztop"], pl["fx"], pl["fy"]))

    pinch_y = b["cy"] + r_rim - PINCH_SHRINK
    tip_z = b["ztop"] - GRASP_DEPTH
    lift_z = tz + 0.22

    def attempt(px, py, tzz):
        api.grip(OPEN_W)
        goto(api, px, py, b["ztop"] + 0.05 + fo, sec=1.8, tol=0.006, tag="_HOVER")
        goto(api, px, py, tzz + fo, sec=1.4, tol=0.004, tag="_DOWN")
        api.grip(0.0)
        api.settle(0.4)
        api.log("AFTER_CLOSE %s" % api.gripper())
        goto(api, px, py, lift_z + fo, sec=1.6, tol=0.010, tag="_LIFT")
        api.settle(0.3)
        g = api.gripper()
        api.log("AFTER_LIFT %s" % g)
        return float(g["width_m"])

    w = attempt(b["cx"], pinch_y, tip_z)
    if w < HOLD_W:                      # the wall squirted out of the jaws
        api.log("REGRASP w=%.4f" % w)
        s2 = perceive(api)
        if s2["bowl"] is not None:
            b = s2["bowl"]
            r_rim = max(b["dx"], b["dy"]) / 2.0
            pinch_y = b["cy"] + r_rim - PINCH_SHRINK
            tip_z = b["ztop"] - GRASP_DEPTH
            api.log("BOWL2 c=(%.3f,%.3f) r=%.4f ztop=%.3f" % (b["cx"], b["cy"], r_rim, b["ztop"]))
        w = attempt(b["cx"], pinch_y, tip_z - 0.004)
    api.log("HOLD w=%.4f" % w)

    # --- carry to the plate and release
    rel_y = pl["acy"] + r_rim - PINCH_SHRINK
    goto(api, pl["acx"], rel_y, lift_z + fo, sec=2.0, tol=0.006, tag="_CARRY")
    api.log("OVER_PLATE %s" % api.gripper())
    drop_tip = pl["ztop"] + (b_h - GRASP_DEPTH) + 0.008
    goto(api, pl["acx"], rel_y, drop_tip + fo, sec=1.4, tol=0.005, tag="_DROP")
    api.log("AT_DROP %s" % api.gripper())
    api.grip(OPEN_W)
    api.settle(0.5)
    goto(api, pl["acx"], rel_y, tz + 0.25 + fo, sec=1.4)
    api.settle(0.5)

    # --- verify by re-perception
    s3 = perceive(api)
    if s3["bowl"] is not None:
        api.log("FINAL_BOWL c=(%.3f,%.3f) ztop=%.3f"
                % (s3["bowl"]["cx"], s3["bowl"]["cy"], s3["bowl"]["ztop"]))
    api.log("DONE")
