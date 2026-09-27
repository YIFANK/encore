"""c2clean / spa_bowl_next_to_plate_pos_k3

Pick the black bowl nearest the plate and set it on the plate.

Mechanism (all of it re-derived from the pack + debug seeds 51-65):
  * cam_high point cloud -> table plane -> two height bands.
    low band  (tz+0.012 .. tz+0.030): flat things; the plate is the one with
    the biggest footprint (measured 0.135 x 0.135 vs 0.08 x 0.06 for the box).
    high band (tz+0.035 .. tz+0.20): the vessels; the plate/box drop out, which
    is what un-fuses the target bowl from the box on seeds 58 and 62.
  * target = the high-band cluster nearest the plate (matches the demos, whose
    plate sat in the OTHER slot -- _pos swaps the plate and the box -- and
    whose grasp still landed on the bowl nearest the plate).
  * grasp = rim pinch: Kasa circle fit on the top 6 mm of the target gives
    r = 0.0537 on every debug seed; the demos closed 0.048 m from the bowl
    centre on the -y side, i.e. just inside that circle.
"""
import numpy as np

PROVENANCE = {
    "R_DOWN": {"source": "generic controller mechanics: straight-down tool frame, matches api.tool_rotation() at reset on debug seeds", "allowed": True},
    "XLO/XHI/YLO/YHI": {"source": "debug-seed cam_high clouds: crop that holds plate+box+vessels and excludes the cabinet (y<-0.05) and the stove slab", "allowed": True},
    "TABLE_BIN": {"source": "debug-seed depth histogram, table plane at z=0.9010 on seeds 51-65", "allowed": True},
    "LOW_BAND": {"source": "debug-seed measurement: plate top tz+0.019, box top tz+0.020", "allowed": True},
    "HIGH_BAND": {"source": "debug-seed measurement: bowls tz+0.051, tin tz+0.043", "allowed": True},
    "PLATE_MIN_SPAN": {"source": "debug-seed measurement: plate footprint 0.135 x 0.135, cookie box 0.080 x 0.060", "allowed": True},
    "GRASP_OFFSET": {"source": "pack keyframes: close-EEF minus bowl centre = (-0.004,-0.056), (0.000,-0.049), (0.003,-0.040) over demos 0-2", "allowed": True},
    "GRASP_DZ": {"source": "pack keyframes: close-EEF z 0.9226/0.9395/0.9426 vs debug table z 0.9010 -> tz+0.022..tz+0.042", "allowed": True},
    "PLACE_DZ": {"source": "pack keyframes: release-EEF z 0.9355/0.9394/0.9373 -> tz+0.036", "allowed": True},
    "CARRY_DZ": {"source": "pack ee_path6: mid-transfer EEF rises to z~1.05, i.e. tz+0.15", "allowed": True},
    "CELL": {"source": "generic: 5 mm voxel, below the 0.0537 m rim radius measured on debug seeds", "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
XLO, XHI, YLO, YHI = -0.30, 0.35, -0.05, 0.45
CELL = 0.005
LOW_BAND = (0.012, 0.030)
HIGH_BAND = (0.035, 0.200)
PLATE_MIN_SPAN = 0.10
GRASP_OFFSET = 0.048
GRASP_DZ = 0.030
PLACE_DZ = 0.040
CARRY_DZ = 0.150


def _cloud(f):
    dep = np.asarray(f.depth, dtype=np.float32)
    K = np.asarray(f.intrinsics, dtype=np.float64)
    T = np.asarray(f.t_base_cam, dtype=np.float64)
    H, W = dep.shape
    v, u = np.mgrid[0:H, 0:W]
    pc = np.stack([(u - K[0, 2]) / K[0, 0] * dep, (v - K[1, 2]) / K[1, 1] * dep, dep], -1)
    return (pc.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3])


def _comps(mask):
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    H, W = mask.shape
    for sy in range(H):
        for sx in range(W):
            if mask[sy, sx] and lab[sy, sx] == 0:
                cur += 1
                lab[sy, sx] = cur
                st = [(sy, sx)]
                while st:
                    y, x = st.pop()
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            ny, nx = y + dy, x + dx
                            if 0 <= ny < H and 0 <= nx < W and mask[ny, nx] and lab[ny, nx] == 0:
                                lab[ny, nx] = cur
                                st.append((ny, nx))
    return lab, cur


def _clusters(P, zlo, zhi, minc=10):
    Q = P[(P[:, 2] > zlo) & (P[:, 2] < zhi)]
    if len(Q) == 0:
        return []
    nx = int((XHI - XLO) / CELL) + 1
    ny = int((YHI - YLO) / CELL) + 1
    hm = np.full((nx, ny), -1.0)
    ix = np.clip(((Q[:, 0] - XLO) / CELL).astype(int), 0, nx - 1)
    iy = np.clip(((Q[:, 1] - YLO) / CELL).astype(int), 0, ny - 1)
    np.maximum.at(hm, (ix, iy), Q[:, 2])
    lab, n = _comps(hm > 0)
    out = []
    for i in range(1, n + 1):
        sel = lab == i
        if sel.sum() < minc:
            continue
        aa, bb = np.where(sel)
        x0, x1 = XLO + aa.min() * CELL, XLO + aa.max() * CELL
        y0, y1 = YLO + bb.min() * CELL, YLO + bb.max() * CELL
        pm = (Q[:, 0] >= x0 - CELL) & (Q[:, 0] <= x1 + CELL) & (Q[:, 1] >= y0 - CELL) & (Q[:, 1] <= y1 + CELL)
        out.append(dict(n=int(sel.sum()), xc=(x0 + x1) / 2.0, yc=(y0 + y1) / 2.0,
                        dx=x1 - x0, dy=y1 - y0, ztop=float(hm[sel].max()), pts=Q[pm]))
    return out


def _kasa(xy):
    A = np.c_[xy[:, 0], xy[:, 1], np.ones(len(xy))]
    b = xy[:, 0] ** 2 + xy[:, 1] ** 2
    c = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = c[0] / 2.0, c[1] / 2.0
    return cx, cy, float(np.sqrt(max(c[2] + cx * cx + cy * cy, 1e-9)))


def perceive(api):
    P = _cloud(api.capture("cam_high"))
    P = P[(P[:, 0] > XLO) & (P[:, 0] < XHI) & (P[:, 1] > YLO) & (P[:, 1] < YHI)
          & (P[:, 2] > 0.70) & (P[:, 2] < 1.30)]
    h, e = np.histogram(P[:, 2], bins=200, range=(0.70, 1.10))
    tz = float(e[int(np.argmax(h))]) + 0.001
    lo = [c for c in _clusters(P, tz + LOW_BAND[0], tz + LOW_BAND[1])
          if min(c["dx"], c["dy"]) >= PLATE_MIN_SPAN]
    hi = _clusters(P, tz + HIGH_BAND[0], tz + HIGH_BAND[1])
    api.log("tz=%.4f  low=%d high=%d" % (tz, len(lo), len(hi)))
    for c in lo:
        api.log("  LOW  c=(%+.3f,%+.3f) sz=(%.3f,%.3f) h=%.3f n=%d"
                % (c["xc"], c["yc"], c["dx"], c["dy"], c["ztop"] - tz, c["n"]))
    plate = max(lo, key=lambda c: c["dx"] * c["dy"])
    best = None
    for c in hi:
        d = float(np.hypot(c["xc"] - plate["xc"], c["yc"] - plate["yc"]))
        rim = c["pts"][c["pts"][:, 2] > c["ztop"] - 0.006]
        cx, cy, r = _kasa(rim[:, :2]) if len(rim) > 30 else (c["xc"], c["yc"], 0.0)
        api.log("  HIGH d=%.3f c=(%+.3f,%+.3f) sz=(%.3f,%.3f) h=%.3f kasa=(%+.3f,%+.3f) r=%.4f"
                % (d, c["xc"], c["yc"], c["dx"], c["dy"], c["ztop"] - tz, cx, cy, r))
        c.update(d=d, kx=cx, ky=cy, r=r)
        if best is None or d < best["d"]:
            best = c
    return tz, plate, best


def run(api):
    api.log("instr: %s" % api.instruction())
    tz, plate, bowl = perceive(api)
    bx, by = bowl["kx"], bowl["ky"]
    px, py = plate["xc"], plate["yc"]
    api.log("TARGET bowl=(%.3f,%.3f) r=%.4f  plate=(%.3f,%.3f)" % (bx, by, bowl["r"], px, py))

    gx, gy = bx, by - GRASP_OFFSET
    api.grip(0.08)
    api.move([gx, gy, tz + CARRY_DZ], R_DOWN, 2.0)
    api.move([gx, gy, tz + GRASP_DZ + 0.05], R_DOWN, 1.5)
    r = api.move([gx, gy, tz + GRASP_DZ], R_DOWN, 2.0)
    api.log("at grasp eef=%s residual=%s" % (np.round(api.eef(), 4).tolist(), np.round(r, 4).tolist() if np.ndim(r) else r))
    api.grip(0.0)
    api.settle(0.3)
    g = api.gripper()
    api.log("after close gripper=%s" % g)

    api.move([gx, gy, tz + CARRY_DZ], R_DOWN, 2.0)
    api.log("after lift eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))

    dx, dy = px, py - GRASP_OFFSET
    api.move([dx, dy, tz + CARRY_DZ], R_DOWN, 2.5)
    api.move([dx, dy, tz + PLACE_DZ + 0.04], R_DOWN, 1.5)
    api.move([dx, dy, tz + PLACE_DZ], R_DOWN, 1.5)
    api.log("at place eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(0.08)
    api.settle(0.4)
    api.move([dx, dy, tz + CARRY_DZ], R_DOWN, 2.0)
    api.settle(0.5)

    try:
        tz2, plate2, bowl2 = perceive(api)
        api.log("FINAL nearest-vessel=(%.3f,%.3f) h=%.3f plate=(%.3f,%.3f) dist=%.3f"
                % (bowl2["kx"], bowl2["ky"], bowl2["ztop"] - tz2, plate2["xc"], plate2["yc"], bowl2["d"]))
    except Exception as exc:
        api.log("FINAL perceive failed: %r" % (exc,))
