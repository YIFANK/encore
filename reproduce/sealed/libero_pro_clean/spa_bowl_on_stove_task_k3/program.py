"""c2clean / spa_bowl_on_stove_task_k3  --  v1

Intent: "Pick the akita black bowl on the top of the cabinet and place it on
the plate".

Mechanism read off the two packs:
  * mate pack ("...black bowl on the wooden cabinet...") grasps at eef z
    1.146-1.159 over the cabinet top and releases over the plate at z
    0.954-0.980; the finger gap at the close is 0.0026-0.0027 per finger
    (gap ~0.005) -> the grasp is a RIM PINCH on a thin bowl wall, not a
    straddle (the bowl footprint measured 0.105 m wide is wider than the
    0.078 m the jaws open).
  * the release eef y sits ~+0.04 past the plate centre in both packs, i.e.
    the pinched bowl trails the eef by one rim radius along -y  ->  the
    pinch is on the +y arc of the rim, jaws closing along base y.
  * k3 pack (same mechanism on the stove bowl) closes at eef z 0.948 with
    that bowl's rim measured at 0.980 here; mate closes at 1.152 with the
    cabinet bowl's rim measured at 1.180.  Both say  eef_z = rim_top - 0.030.

Perception (own debug-seed observations, cam_high RGB-D):
  * table top 0.899-0.902, clean gap to 0.907.
  * cabinet top slab a fixture plateau at z 1.127, footprint x[-0.09,0.17]
    y[-0.345,-0.14].  Exactly ONE blob rises above it: the target bowl
    (footprint 0.105-0.11 m, rim top 1.180, centre wanders by seed).
  * the plate is the largest flat blob in [0.905,0.928] (n~610 cells at 5 mm
    = a 0.14 m disc), rim top 0.920.
"""
import numpy as np

PROVENANCE = {
    "GRID_RES": {"source": "generic mapping choice (5 mm top-down cell)", "allowed": True},
    "X_RANGE": {"source": "debug seeds 51/53/55/57 cam_high cloud extent", "allowed": True},
    "Y_RANGE": {"source": "debug seeds 51/53/55/57 cam_high cloud extent", "allowed": True},
    "TABLE_Z": {"source": "debug-seed height histogram: table surface 0.899-0.902, "
                          "first empty bin 0.903-0.906", "allowed": True},
    "FLAT_BAND": {"source": "debug-seed height histogram: plate rim tops out at 0.920, "
                            "table ends 0.902", "allowed": True},
    "CAB_BAND": {"source": "debug seeds 51-57: cabinet-top plateau median z 1.127", "allowed": True},
    "CAB_MARGIN": {"source": "debug seeds: only blob above cab_z+0.015 inside the cabinet "
                             "footprint is the target bowl (n~150-205 cells)", "allowed": True},
    "GRASP_DZ": {"source": "mate pack close eef z 1.146/1.152/1.159 vs measured cabinet-bowl "
                           "rim 1.180; k3 pack close 0.948 vs measured stove-bowl rim 0.980",
                 "allowed": True},
    "RIM_OFFSET_SIGN": {"source": "mate+k3 pack release eef y ~0.04 past the plate centre "
                                  "measured at y 0.195 -> bowl trails eef along -y",
                        "allowed": True},
    "CARRY_Z": {"source": "mate pack carry apex z 1.278-1.315", "allowed": True},
    "PLACE_DZ": {"source": "mate pack release eef z 0.954-0.980 vs measured plate rim 0.920",
                 "allowed": True},
    "APPROACH_Z": {"source": "mate pack pre-grasp descent starts from z 1.20-1.22, above the "
                             "measured 1.180 rim", "allowed": True},
}

GRID_RES = 0.005
X_RANGE = (-0.45, 0.30)
Y_RANGE = (-0.45, 0.45)
TABLE_Z = 0.902
FLAT_BAND = (0.905, 0.928)
CAB_BAND = (1.05, 1.148)
CAB_MARGIN = 0.015
CAB_SPAN = 0.085
GRASP_DZ = -0.030
CARRY_Z = 1.280
APPROACH_Z = 1.245
PLACE_DZ = 0.045


# --------------------------------------------------------------- perception
def _cloud(frame):
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    d = np.asarray(frame.depth, float)
    h, w = d.shape[:2]
    uu, vv = np.meshgrid(np.arange(w, dtype=float), np.arange(h, dtype=float))
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1)
    B = P @ T.T
    return B[..., :3], np.asarray(frame.rgb, float)


def _topdown(B, rgb):
    nx = int((X_RANGE[1] - X_RANGE[0]) / GRID_RES)
    ny = int((Y_RANGE[1] - Y_RANGE[0]) / GRID_RES)
    Z = np.full((nx, ny), np.nan)
    C = np.zeros((nx, ny, 3))
    X = B[..., 0].ravel(); Y = B[..., 1].ravel(); Zv = B[..., 2].ravel()
    Cv = rgb.reshape(-1, 3)
    ix = np.floor((X - X_RANGE[0]) / GRID_RES).astype(int)
    iy = np.floor((Y - Y_RANGE[0]) / GRID_RES).astype(int)
    m = ((ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
         & np.isfinite(Zv) & (Zv > 0.1))
    order = np.argsort(Zv[m])                    # tallest written last -> wins
    Z[ix[m][order], iy[m][order]] = Zv[m][order]
    C[ix[m][order], iy[m][order]] = Cv[m][order]
    return Z, C


def _blobs(Z, C, lo, hi, region=None, min_cells=12):
    m = np.isfinite(Z) & (Z >= lo) & (Z < hi)
    if region is not None:
        m &= region
    lab = np.zeros(m.shape, int)
    out = []
    nid = 0
    for i0 in range(m.shape[0]):
        for j0 in range(m.shape[1]):
            if not m[i0, j0] or lab[i0, j0]:
                continue
            nid += 1
            stack = [(i0, j0)]
            lab[i0, j0] = nid
            cells = []
            while stack:
                a, b = stack.pop()
                cells.append((a, b))
                for da in (-1, 0, 1):
                    for db in (-1, 0, 1):
                        p, q = a + da, b + db
                        if (0 <= p < m.shape[0] and 0 <= q < m.shape[1]
                                and m[p, q] and not lab[p, q]):
                            lab[p, q] = nid
                            stack.append((p, q))
            if len(cells) < min_cells:
                continue
            ii = np.array([c[0] for c in cells]); jj = np.array([c[1] for c in cells])
            x = X_RANGE[0] + (ii + 0.5) * GRID_RES
            y = Y_RANGE[0] + (jj + 0.5) * GRID_RES
            z = Z[ii, jj]
            out.append(dict(n=len(cells), xmin=x.min(), xmax=x.max(),
                            ymin=y.min(), ymax=y.max(),
                            cx=0.5 * (x.min() + x.max()), cy=0.5 * (y.min() + y.max()),
                            ztop=float(np.percentile(z, 97)), zmed=float(np.median(z)),
                            rgb=C[ii, jj].mean(0)))
    out.sort(key=lambda b: -b["n"])
    return out


def _grid_region(xlo, xhi, ylo, yhi, shape):
    nx, ny = shape
    xs = X_RANGE[0] + (np.arange(nx) + 0.5) * GRID_RES
    ys = Y_RANGE[0] + (np.arange(ny) + 0.5) * GRID_RES
    return (((xs >= xlo) & (xs <= xhi))[:, None]
            & ((ys >= ylo) & (ys <= yhi))[None, :])


def perceive(api):
    f = api.capture("cam_high")
    B, rgb = _cloud(f)
    Z, C = _topdown(B, rgb)

    # 1. the cabinet-top plateau (a fixture): largest blob in CAB_BAND
    cab = _blobs(Z, C, CAB_BAND[0], CAB_BAND[1], min_cells=60)
    if not cab:
        raise RuntimeError("no cabinet-top plateau found")
    cb = cab[0]
    cab_z = cb["zmed"]
    api.log("CAB n=%d x[%.3f,%.3f] y[%.3f,%.3f] z=%.3f"
            % (cb["n"], cb["xmin"], cb["xmax"], cb["ymin"], cb["ymax"], cab_z))

    # 2. the only thing standing on it is the target bowl
    reg = _grid_region(cb["xmin"] - 0.03, cb["xmax"] + 0.03,
                       cb["ymin"] - 0.03, cb["ymax"] + 0.03, Z.shape)
    tops = _blobs(Z, C, cab_z + CAB_MARGIN, cab_z + CAB_SPAN, region=reg, min_cells=30)
    for b in tops:
        api.log("TOP n=%d x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f rgb=%s"
                % (b["n"], b["xmin"], b["xmax"], b["ymin"], b["ymax"], b["ztop"],
                   np.round(b["rgb"], 0).tolist()))
    if not tops:
        raise RuntimeError("no bowl on the cabinet top")
    bowl = tops[0]
    rim_r = 0.25 * ((bowl["xmax"] - bowl["xmin"]) + (bowl["ymax"] - bowl["ymin"]))

    # 3. the plate: largest flat blob clear of the cabinet
    plates = _blobs(Z, C, FLAT_BAND[0], FLAT_BAND[1], min_cells=80)
    for b in plates[:4]:
        api.log("FLAT n=%d x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f rgb=%s"
                % (b["n"], b["xmin"], b["xmax"], b["ymin"], b["ymax"], b["ztop"],
                   np.round(b["rgb"], 0).tolist()))
    if not plates:
        raise RuntimeError("no plate found")
    plate = plates[0]
    return bowl, rim_r, plate, cab_z


# ------------------------------------------------------------------ motion
def run(api):
    api.log("INSTR %s" % api.instruction())
    bowl, rim_r, plate, cab_z = perceive(api)
    gx = bowl["cx"]
    gy = bowl["cy"] + rim_r                     # +y arc of the rim
    gz = bowl["ztop"] + GRASP_DZ
    px = plate["cx"]
    py = plate["cy"] + rim_r                    # bowl trails the eef by rim_r
    pz = plate["ztop"] + PLACE_DZ
    api.log("PLAN bowl=(%.3f,%.3f) rim_r=%.3f rim_top=%.3f grasp=(%.3f,%.3f,%.3f) "
            "plate=(%.3f,%.3f) place=(%.3f,%.3f,%.3f)"
            % (bowl["cx"], bowl["cy"], rim_r, bowl["ztop"], gx, gy, gz,
               plate["cx"], plate["cy"], px, py, pz))

    api.grip(0.08)
    r = api.move([gx, gy, APPROACH_Z], rotation=None, seconds=1.5)
    api.log("W1 res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    r = api.move([gx, gy, gz], rotation=None, seconds=1.0)
    api.log("W2 res=%.4f eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), api.gripper()))

    api.grip(0.0)
    api.settle(0.3)
    g0 = api.gripper()
    api.log("CLOSE grip=%s eef=%s" % (g0, np.round(api.eef(), 4).tolist()))

    r = api.move([gx, gy, CARRY_Z], rotation=None, seconds=1.0)
    g1 = api.gripper()
    api.log("LIFT res=%.4f eef=%s grip=%s" % (r, np.round(api.eef(), 4).tolist(), g1))

    r = api.move([px, py, CARRY_Z], rotation=None, seconds=2.0)
    api.log("TRAV res=%.4f eef=%s grip=%s" % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    r = api.move([px, py, pz], rotation=None, seconds=1.0)
    g2 = api.gripper()
    api.log("DOWN res=%.4f eef=%s grip=%s" % (r, np.round(api.eef(), 4).tolist(), g2))

    api.grip(0.08)
    api.settle(0.4)
    api.log("OPEN grip=%s" % api.gripper())
    r = api.move([px, py, pz + 0.12], rotation=None, seconds=0.6)
    api.log("RETREAT res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))

    # post-hoc self-check: is the cabinet top empty and is something on the plate?
    try:
        f = api.capture("cam_high")
        B, rgb = _cloud(f)
        Z, C = _topdown(B, rgb)
        reg = _grid_region(-0.12, 0.20, -0.38, -0.11, Z.shape)
        left = _blobs(Z, C, cab_z + CAB_MARGIN, cab_z + CAB_SPAN, region=reg, min_cells=30)
        api.log("CHECK cab_top_blobs=%d" % len(left))
        on_plate = _blobs(Z, C, plate["ztop"] + 0.01, plate["ztop"] + 0.10,
                          region=_grid_region(plate["xmin"] - 0.05, plate["xmax"] + 0.05,
                                              plate["ymin"] - 0.05, plate["ymax"] + 0.05,
                                              Z.shape), min_cells=20)
        for b in on_plate[:3]:
            api.log("CHECK plate_blob n=%d ctr(%.3f,%.3f) ztop=%.3f"
                    % (b["n"], b["cx"], b["cy"], b["ztop"]))
    except Exception as e:
        api.log("CHECK failed %s" % e)
    return "v1 rim-pinch cabinet bowl -> plate"
