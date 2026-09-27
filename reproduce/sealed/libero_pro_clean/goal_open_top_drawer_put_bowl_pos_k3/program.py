"""c2clean goal_open_top_drawer_put_bowl_pos_k3 -- v2

Mechanism (all re-derived from pack + debug seeds 51-65):
  1. perceive cam_high -> base-frame cloud; find cabinet top plane, the top
     drawer's handle slab (a protrusion in front of the cabinet face, ~30mm
     below the cabinet top), and the bowl (round cluster on the table).
  2. press the -y finger down onto the handle slab and drag +y by the demo's
     drag length -> drawer opens.
  3. straddle the bowl wall (offset = measured rim radius, on the +y side),
     close, lift.
  4. carry over the opened drawer cavity and release.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seed 51/53/55/57 cam_high deprojection of bare table (0.9012 m)", "allowed": True},
    "TIP_OFFSET": {"source": "v1 debug probe: press-down onto bare table stalls at eef z=0.9102 -> fingertips 0.009 m below the eef reference", "allowed": True},
    "DRAG_LEN": {"source": "pack demo0 ee_path6: drag phase eef y -0.093 -> +0.070 at constant z", "allowed": True},
    "DRAG_Z_ABOVE_HANDLE": {"source": "pack demos drag z=1.105 vs debug-seed measured handle top 1.098", "allowed": True},
    "HOOK_FINGER_DY": {"source": "pack demo0 drag-start eef y=-0.093 minus open half-width 0.0398 = finger at -0.133, i.e. 0.008 behind the debug-seed handle tip -0.126", "allowed": True},
    "GRASP_Z": {"source": "pack demos: gripper closes on the bowl at eef z 0.9149/0.918/0.939 (median 0.918)", "allowed": True},
    "RELEASE_Z": {"source": "pack demo0/demo1/demo2 release eef z 1.135/1.130/1.166", "allowed": True},
    "RELEASE_DY_FROM_TIP": {"source": "pack demo0 release y=-0.046 vs handle tip after a 0.163 drag (debug-seed tip -0.126 + 0.163 = 0.037)", "allowed": True},
    "RELEASE_DX_FROM_HANDLE": {"source": "pack demo0 release x=0.018 vs debug-seed handle x-centre 0.035", "allowed": True},
    "GRASP_DX": {"source": "pack demo0/demo2 grasp eef minus the demo bowl centre back-projected from keyframe t0 (-0.027,-0.034)", "allowed": True},
    "GRASP_DY": {"source": "pack demo grasp offset magnitude 0.055-0.067 vs debug-seed measured rim radius 0.055", "allowed": True},
    "CARRY_Z": {"source": "pack demo0/demo2 lift apex eef z 1.219/1.258", "allowed": True},
    "OPEN_W": {"source": "debug-seed api.gripper() at reset: width_m 0.0778-0.0802", "allowed": True},
    "CAM_MODEL": {"source": "generic pinhole camera mechanics with api-provided intrinsics/extrinsics", "allowed": True},
}

TABLE_Z = 0.9012
TIP_OFFSET = 0.009
DRAG_LEN = 0.163
DRAG_Z_ABOVE_HANDLE = 0.007
HOOK_FINGER_DY = -0.008
GRASP_Z = 0.918
RELEASE_Z = 1.145
GRASP_DX = -0.010
GRASP_DY = 0.000
RELEASE_DY_FROM_TIP = -0.083
RELEASE_DX_FROM_HANDLE = -0.017
CARRY_Z = 1.24
OPEN_W = 0.08


# ---------------------------------------------------------------- perception
def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    d = np.asarray(f.depth, dtype=np.float64)
    K = np.asarray(f.intrinsics, dtype=np.float64)
    T = np.asarray(f.t_base_cam, dtype=np.float64)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) / K[0, 0] * d
    y = (vv - K[1, 2]) / K[1, 1] * d
    P = np.stack([x, y, d], -1)
    B = P @ T[:3, :3].T + T[:3, 3]
    return B, d, np.asarray(f.rgb)


def heightmap(B, res=0.004, x0=-0.45, x1=0.35, y0=-0.45, y1=0.45):
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    nr = int(round((x1 - x0) / res))
    nc = int(round((y1 - y0) / res))
    H = np.full((nr, nc), np.nan)
    m = (X > x0) & (X < x1) & (Y > y0) & (Y < y1) & (Z > 0.80) & (Z < 1.45)
    r = ((X - x0) / res).astype(int)
    c = ((Y - y0) / res).astype(int)
    rr, cc, zz = r[m], c[m], Z[m]
    ok = (rr >= 0) & (rr < nr) & (cc >= 0) & (cc < nc)
    rr, cc, zz = rr[ok], cc[ok], zz[ok]
    o = np.argsort(zz)
    H[rr[o], cc[o]] = zz[o]
    return H, x0, y0, res


def components(mask):
    nr, nc = mask.shape
    lab = np.zeros((nr, nc), np.int32)
    n = 0
    out = []
    for r in range(nr):
        for c in range(nc):
            if mask[r, c] and lab[r, c] == 0:
                n += 1
                st = [(r, c)]
                lab[r, c] = n
                cells = []
                while st:
                    a, b = st.pop()
                    cells.append((a, b))
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if 0 <= p < nr and 0 <= q < nc and mask[p, q] and lab[p, q] == 0:
                                lab[p, q] = n
                                st.append((p, q))
                out.append(np.array(cells))
    return out


def find_cabinet_and_handle(api, H, x0, y0, res):
    nr, nc = H.shape
    rs, cs = np.mgrid[0:nr, 0:nc]
    Xg = x0 + rs * res
    Yg = y0 + cs * res
    F = np.isfinite(H)
    # cabinet top: the big flat slab on the -y side, well above the table
    cand = F & (Yg < -0.12) & (H > TABLE_Z + 0.15) & (H < TABLE_Z + 0.32)
    zc = H[cand]
    if zc.size < 200:
        return None
    hist, edges = np.histogram(zc, bins=60)
    zt = 0.5 * (edges[np.argmax(hist)] + edges[np.argmax(hist) + 1])
    top = cand & (np.abs(H - zt) < 0.006)
    face_y = float(np.percentile(Yg[top], 99.5))
    api.log("CAB top_z=%.4f face_y=%.4f cells=%d x=%.3f..%.3f"
            % (zt, face_y, int(top.sum()), float(Xg[top].min()), float(Xg[top].max())))
    # handle slab: in front of / at the face, 12..55 mm below the cabinet top
    hb = F & (H > zt - 0.055) & (H < zt - 0.012) & (Yg > face_y - 0.06) & (Yg < face_y + 0.10) \
        & (Xg > -0.25) & (Xg < 0.30)
    comps = components(hb)
    best = None
    for cells in comps:
        if len(cells) < 25:
            continue
        yy = Yg[cells[:, 0], cells[:, 1]]
        xx = Xg[cells[:, 0], cells[:, 1]]
        zz = H[cells[:, 0], cells[:, 1]]
        span_x = xx.max() - xx.min()
        if span_x < 0.08:
            continue
        score = float(np.percentile(zz, 95))
        if best is None or score > best[0]:
            best = (score, float(np.percentile(yy, 98)), 0.5 * (xx.min() + xx.max()), len(cells),
                    float(yy.min()), float(yy.max()), float(xx.min()), float(xx.max()))
    if best is None:
        return None
    api.log("HANDLE top_z=%.4f tip_y=%.4f cx=%.4f n=%d y=%.3f..%.3f x=%.3f..%.3f"
            % best)
    return {"top_z": best[0], "tip_y": best[1], "cx": best[2], "cab_top": zt, "face_y": face_y}


def find_bowl(api, H, x0, y0, res):
    """Round vessel on the table: top ~0.05 m above the table, ~0.11 m across."""
    nr, nc = H.shape
    rs, cs = np.mgrid[0:nr, 0:nc]
    Xg = x0 + rs * res
    Yg = y0 + cs * res
    F = np.isfinite(H)
    mask = F & (H > TABLE_Z + 0.012) & (H < TABLE_Z + 0.12) & (Xg > -0.22) & (Xg < 0.32) & (Yg > -0.10)
    best = None
    for cells in components(mask):
        if len(cells) < 60:
            continue
        xx = Xg[cells[:, 0], cells[:, 1]]
        yy = Yg[cells[:, 0], cells[:, 1]]
        zz = H[cells[:, 0], cells[:, 1]]
        sx = xx.max() - xx.min()
        sy = yy.max() - yy.min()
        ztop = float(np.percentile(zz, 97))
        cx = 0.5 * (xx.min() + xx.max())
        cy = 0.5 * (yy.min() + yy.max())
        api.log("CAND n=%d sx=%.3f sy=%.3f ztop=%.4f c=(%.3f,%.3f)" % (len(cells), sx, sy, ztop, cx, cy))
        h = ztop - TABLE_Z
        if not (0.030 < h < 0.090):
            continue
        if not (0.07 < sx < 0.17 and 0.07 < sy < 0.17):
            continue
        ar = max(sx, sy) / max(1e-6, min(sx, sy))
        if ar > 1.45:
            continue
        sc = len(cells)
        if best is None or sc > best[0]:
            best = (sc, cx, cy, ztop, sx, sy)
    if best is None:
        return None
    r_rim = 0.25 * (best[4] + best[5])
    api.log("BOWL c=(%.4f,%.4f) ztop=%.4f sx=%.3f sy=%.3f r=%.4f n=%d"
            % (best[1], best[2], best[3], best[4], best[5], r_rim, best[0]))
    return {"cx": best[1], "cy": best[2], "ztop": best[3], "r": r_rim}


# ---------------------------------------------------------------- motion
# The episode horizon is 1000 sim steps and every api.move costs a fixed
# minimum (measured in v2: 12 moves + settles exhausted the horizon), so the
# program runs a FIXED, short sequence of moves -- no retry loops.
def run(api):
    api.log("INSTR %s" % api.instruction())
    api.grip(OPEN_W)
    api.settle(0.2)
    halfw = 0.5 * float(api.gripper()["width_m"])
    api.log("OPEN halfw=%.4f" % halfw)

    B, d, rgb = cloud(api)
    H, x0, y0, res = heightmap(B)
    hd = find_cabinet_and_handle(api, H, x0, y0, res)
    bw = find_bowl(api, H, x0, y0, res)
    if hd is None:
        api.log("NO HANDLE -- abort")
        return

    # ---- phase 1: press the -y finger onto the handle slab and drag +y
    hook_y = hd["tip_y"] + HOOK_FINGER_DY + halfw
    z_drag = hd["top_z"] + DRAG_Z_ABOVE_HANDLE
    hx = hd["cx"]
    api.log("HOOK x=%.4f y=%.4f z=%.4f" % (hx, hook_y, z_drag))
    api.move([hx, hook_y, z_drag + 0.085], seconds=3.0)          # M1
    api.move([hx, hook_y, z_drag], seconds=2.0)                  # M2
    api.log("AT HANDLE %s" % np.round(np.array(api.eef()), 4).tolist())
    for i in (1, 2, 3):                                          # M3-M5
        api.move([hx, hook_y + DRAG_LEN * i / 3.0, z_drag - 0.025], seconds=2.0)
    e = np.array(api.eef())
    drag = float(e[1]) - hook_y
    tip_now = hd["tip_y"] + max(0.0, drag)
    api.log("AFTER DRAG %s drag=%.4f tip_now=%.4f" % (np.round(e, 4).tolist(), drag, tip_now))

    if bw is None:
        api.log("NO BOWL -- stop after opening")
        return

    # ---- phase 2: straddle the bowl wall on the +y side and lift
    gx = bw["cx"] + GRASP_DX
    gy = bw["cy"] + bw["r"] + GRASP_DY
    api.log("GRASP target (%.4f,%.4f,%.4f)" % (gx, gy, GRASP_Z))
    api.move([gx, gy, 1.05], seconds=3.0)                        # M6
    api.move([gx, gy, GRASP_Z], seconds=2.0)                     # M7
    api.log("AT BOWL %s" % np.round(np.array(api.eef()), 4).tolist())
    api.grip(0.0)
    api.settle(0.3)
    api.log("GRIP closed %s" % api.gripper())
    api.move([gx, gy, CARRY_Z], seconds=2.5)                     # M8
    api.log("AFTER LIFT %s grip %s" % (np.round(np.array(api.eef()), 4).tolist(), api.gripper()))

    # ---- phase 3: carry over the open drawer and release
    rx = hx + RELEASE_DX_FROM_HANDLE
    ry = tip_now + RELEASE_DY_FROM_TIP
    api.log("RELEASE target (%.4f,%.4f,%.4f)" % (rx, ry, RELEASE_Z))
    api.move([rx, ry, CARRY_Z], seconds=3.0)                     # M9
    api.move([rx, ry, RELEASE_Z], seconds=2.0)                   # M10
    api.log("AT DROP %s grip %s" % (np.round(np.array(api.eef()), 4).tolist(), api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.4)
    api.move([rx, ry, CARRY_Z], seconds=2.0)                     # M11
    api.log("END %s" % np.round(np.array(api.eef()), 4).tolist())
    # free post-hoc diagnosis (capture costs no sim steps)
    try:
        B3, _, _ = cloud(api)
        H3, _, _, _ = heightmap(B3)
        nr, nc = H3.shape
        rs, cs = np.mgrid[0:nr, 0:nc]
        Xg = x0 + rs * res
        Yg = y0 + cs * res
        m = np.isfinite(H3) & (H3 > 1.00) & (H3 < hd["cab_top"] - 0.005) & (Yg < tip_now + 0.02) \
            & (Yg > tip_now - 0.30) & (Xg > hx - 0.20) & (Xg < hx + 0.20)
        if m.sum() > 20:
            api.log("POST drawer-region n=%d z=%.4f..%.4f cen=(%.3f,%.3f)"
                    % (int(m.sum()), H3[m].min(), H3[m].max(), Xg[m].mean(), Yg[m].mean()))
        b3 = find_bowl(api, H3, x0, y0, res)
        api.log("POST bowl %s" % (b3,))
    except Exception as ex:
        api.log("POST err %s" % ex)
