"""v5 -- full task: rim-pinch the raised bowl, carry it over the plate on the
tall fixture, lower until its base clears the plate rim, release.

v5 adds an own-sensor hold check after the lift and re-grasps at a different
rim depth when the bite slips.

All constants re-derived on debug seeds 51-65 (see PROVENANCE)."""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51/53/55 cam_high deprojection: modal height of the bare table surface (0.9010 m)", "allowed": True},
    "Z_BIAS": {"source": "debug seed 51/53 v2 DESC log: eef z settles 0.0112 m above every commanded z while free", "allowed": True},
    "FINGER_DZ": {"source": "debug seed 51/53 v2: closed jaws stall at eef z 0.9088 on the table at 0.9010 -> fingertip 0.008 m below the eef origin", "allowed": True},
    "GRASP_R": {"source": "debug 51/53/55 radial profile of the raised bowl: rim top plateau spans r 0.050-0.058, outer wall ends at r 0.059; aim mid-lip", "allowed": True},
    "GRASP_DZ_TRIES": {"source": "debug 51-65 depth sweep: rim_z-0.008 -> gap 0.0047, 4/6; -0.015 -> 0.0072, 15/15; -0.022 -> 0.0080, 6/6; -0.028 -> 6.5/8 (one lift-slip); -0.035 -> 8/8. Centre the first try in the good band and step either way on a slip", "allowed": True},
    "HOLD_GAP": {"source": "debug 51-65: a surviving rim bite reads width 0.0068-0.0103 m with effort 3.0; the one observed lift-slip read 0.0039 m with effort 0.05", "allowed": True},
    "HELD_BASE_DZ": {"source": "debug seed 51 v2 t2 re-perception: held bowl rim top = eef_z+0.007 and bowl height = 0.051 -> base = eef_z-0.044", "allowed": True},
    "HELD_OFF": {"source": "debug seed 51 v2 t2 re-perception: held bowl centre sits GRASP_R (0.052 m) from the eef along the outward grasp radial", "allowed": True},
    "BOWL_ROUT": {"source": "debug 51/53/55 radial profile: bowl outer radius 0.059 m, rim-to-base height 0.051 m", "allowed": True},
    "PLATE_BAND": {"source": "debug 51/53/55: plate disc = cells within 0.017 m of the tall fixture's top (rim 1.1480, interior floor 1.1360, fixture top 1.1273)", "allowed": True},
    "PLATE_DEPTH_CUT": {"source": "debug 51-65: rim-minus-floor separates the three disc-like objects cleanly -- plate 0.011-0.012 m, bowl 0.044 m, bare slab ~0.001 m; cut at 0.004-0.025", "allowed": True},
    "PLATE_RAD": {"source": "debug 51/53/55: plate footprint radius 0.067-0.069 m (95th pct), bowl 0.059 m, stove slab 0.046 m", "allowed": True},
    "CLEAR_DZ": {"source": "debug 51/53/55: plate rim 1.148 sits 0.021 m above the fixture top; carry with the bowl base above it", "allowed": True},
    "ARM_ZTOP": {"source": "debug 51/53/55 cam_high: the robot arm cluster tops out at 1.371 m, every scene fixture at <=1.148 m", "allowed": True},
    "R_DOWN_X": {"source": "generic controller/camera mechanics: home tool rotation is straight down with the jaws along base y (v1 wrist frame maps image u to base -y); Rz(90 deg) turns them onto base x", "allowed": True},
}

TABLE_Z = 0.9010
Z_BIAS = 0.0112
FINGER_DZ = 0.008
GRASP_R = 0.052
GRASP_DZ_TRIES = (-0.018, -0.026, -0.012)
HOLD_GAP = 0.0050
HELD_BASE_DZ = -0.044
BOWL_ROUT = 0.059
PLATE_BAND = 0.017
CLEAR_DZ = 0.020
ARM_ZTOP = 1.25

R_DOWN_X = np.array([[0, 1.0, 0], [1.0, 0, 0], [0, 0, -1.0]])

XLO, XHI, YLO, YHI, CELL = -0.60, 0.32, -0.60, 0.60, 0.005
NX = int(round((XHI - XLO) / CELL))
NY = int(round((YHI - YLO) / CELL))


# --------------------------------------------------------------- logging aid
def _dump(api, tag, b):
    blob = base64.b64encode(zlib.compress(b, 9)).decode()
    api.log(f"BLOB {tag} len={len(blob)}")
    for i in range(0, len(blob), 1800):
        api.log(f"B {tag} {i//1800} {blob[i:i+1800]}")


def dump_frame(api, cam, tag):
    f = api.capture(cam)
    api.log(f"K {tag}_{cam} {np.asarray(f.intrinsics).ravel().tolist()}")
    api.log(f"T {tag}_{cam} {np.asarray(f.t_base_cam).ravel().tolist()}")
    _dump(api, f"{tag}_{cam}_rgb", np.asarray(f.rgb, np.uint8).tobytes())
    d = np.where(np.isfinite(f.depth), f.depth, 0.0)
    _dump(api, f"{tag}_{cam}_d", np.clip(d * 10000, 0, 65535).astype(np.uint16).tobytes())


# ------------------------------------------------------------------ percept
def heightmap(f):
    d = np.asarray(f.depth, np.float32)
    K, T = np.asarray(f.intrinsics), np.asarray(f.t_base_cam)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    pc = np.stack([(uu - K[0, 2]) * d / K[0, 0],
                   (vv - K[1, 2]) * d / K[1, 1], d, np.ones_like(d)], -1) @ T.T[:, :3]
    m = ((d > 0) & (pc[..., 0] > XLO) & (pc[..., 0] < XHI)
         & (pc[..., 1] > YLO) & (pc[..., 1] < YHI))
    pts = pc[m]
    H = np.full((NX, NY), np.nan)
    ix = np.clip(((pts[:, 0] - XLO) / CELL).astype(int), 0, NX - 1)
    iy = np.clip(((pts[:, 1] - YLO) / CELL).astype(int), 0, NY - 1)
    for i in np.argsort(pts[:, 2]):
        H[ix[i], iy[i]] = pts[i, 2]
    return H


def clusters(H, zlo):
    m = np.isfinite(H) & (H > zlo)
    ii, jj = np.nonzero(m)
    xs = XLO + (ii + .5) * CELL
    ys = YLO + (jj + .5) * CELL
    zs = H[m]
    n = len(xs)
    lab = -np.ones(n, int)
    cur = 0
    for s in range(n):
        if lab[s] >= 0:
            continue
        stack, lab[s] = [s], cur
        while stack:
            a = stack.pop()
            nb = np.nonzero((np.hypot(xs - xs[a], ys - ys[a]) < 0.011) & (lab < 0))[0]
            lab[nb] = cur
            stack.extend(nb.tolist())
        cur += 1
    out = []
    for k in range(cur):
        s = lab == k
        if s.sum() < 40:
            continue
        out.append(dict(n=int(s.sum()), x=xs[s], y=ys[s], z=zs[s],
                        cx=float(xs[s].mean()), cy=float(ys[s].mean()),
                        w=float(xs[s].max() - xs[s].min()),
                        h=float(ys[s].max() - ys[s].min()),
                        ztop=float(zs[s].max())))
    return out


def ring_depth(c):
    """how far the interior of a top-band disc falls below its own top"""
    b = c['z'] > c['ztop'] - 0.013
    if b.sum() < 20:
        return None
    cx, cy = c['x'][b].mean(), c['y'][b].mean()
    r = np.hypot(c['x'] - cx, c['y'] - cy)
    inner = c['z'][r < 0.6 * max(c['w'], c['h']) / 2.0]
    return cx, cy, float(b.sum()), float(c['ztop'] - inner.min()) if inner.size else 0.0


def run(api):
    api.log(f"INSTR {api.instruction()}")
    f0 = api.capture("cam_high")
    H = heightmap(f0)
    cs = clusters(H, TABLE_Z + 0.011)
    for c in cs:
        api.log(f"OBJ n={c['n']} c=({c['cx']:+.3f},{c['cy']:+.3f}) "
                f"w={c['w']:.3f} h={c['h']:.3f} ztop={c['ztop']:.4f}")

    # --- target: the highest bowl-sized annulus standing on the table ------
    bowls = []
    for c in cs:
        if c['ztop'] > ARM_ZTOP or c['ztop'] > TABLE_Z + 0.16:
            continue
        d = max(c['w'], c['h'])
        if not (0.095 < d < 0.15):
            continue
        rd = ring_depth(c)
        if rd is None or rd[3] < 0.030:      # a bowl is deep; a plate/slab is not
            continue
        bowls.append((c['ztop'], rd[0], rd[1], c))
    bowls.sort(key=lambda t: -t[0])
    api.log(f"BOWLS {[(round(b[0],4), round(b[1],3), round(b[2],3)) for b in bowls]}")
    if not bowls:                              # fallback: tallest table-top object
        for c in sorted(cs, key=lambda c: -c['ztop']):
            if c['ztop'] > TABLE_Z + 0.16 and c['ztop'] < ARM_ZTOP:
                continue
            if c['ztop'] > ARM_ZTOP or max(c['w'], c['h']) > 0.16:
                continue
            bowls.append((c['ztop'], c['cx'], c['cy'], c))
            break
        api.log(f"BOWL-FALLBACK {[(round(b[0],4), round(b[1],3), round(b[2],3)) for b in bowls]}")
    rim_z, cx, cy, _ = bowls[0]

    # --- goal: the plate = shallow disc on the tallest non-arm structure ---
    plates = []
    for c in cs:
        if c['ztop'] > ARM_ZTOP:
            continue
        b = c['z'] > c['ztop'] - PLATE_BAND
        if b.sum() < 150:
            continue
        px, py = float(c['x'][b].mean()), float(c['y'][b].mean())
        rb = np.hypot(c['x'][b] - px, c['y'][b] - py)
        rad = float(np.percentile(rb, 95))
        if not (0.055 < rad < 0.085):
            continue
        if abs(px - cx) < 0.03 and abs(py - cy) < 0.03:
            continue                          # that is the target bowl itself
        r = np.hypot(c['x'] - px, c['y'] - py)
        ctr = c['z'][r < 0.40 * rad]
        if ctr.size < 8:
            continue
        floor = float(np.median(ctr))
        # a plate is a shallow dish: its rim stands a little proud of its own
        # floor.  a bowl's floor is far below its rim; a bare slab has none.
        depth = float(c['ztop'] - floor)
        api.log(f"PCAND c=({px:+.3f},{py:+.3f}) rad={rad:.3f} top={c['ztop']:.4f} "
                f"floor={floor:.4f} depth={depth:.4f} n={int(b.sum())}")
        if not (0.004 < depth < 0.025):
            continue
        plates.append(dict(px=px, py=py, rad=rad, top=float(c['ztop']),
                           floor=floor, depth=depth, n=int(b.sum())))
    plates.sort(key=lambda p: -p['top'])
    if not plates:
        # fallback: the flattest shallow disc of plate size anywhere in the scene
        for c in sorted(cs, key=lambda c: -c['ztop']):
            if c['ztop'] > ARM_ZTOP:
                continue
            b = c['z'] > c['ztop'] - PLATE_BAND
            if b.sum() < 80:
                continue
            px, py = float(c['x'][b].mean()), float(c['y'][b].mean())
            rb = np.hypot(c['x'][b] - px, c['y'][b] - py)
            rad = float(np.percentile(rb, 95))
            r = np.hypot(c['x'] - px, c['y'] - py)
            ctr = c['z'][r < 0.40 * rad]
            floor = float(np.median(ctr)) if ctr.size else float(c['ztop'])
            if not (0.045 < rad < 0.095) or (c['ztop'] - floor) > 0.025:
                continue
            if abs(px - cx) < 0.03 and abs(py - cy) < 0.03:
                continue
            plates.append(dict(px=px, py=py, rad=rad, top=float(c['ztop']),
                               floor=floor, depth=float(c['ztop'] - floor),
                               n=int(b.sum())))
            break
        api.log(f"PLATE-FALLBACK {plates}")
    api.log(f"PLATES {plates}")
    pl = plates[0]
    px, py, ptop = pl['px'], pl['py'], pl['top']
    api.log(f"TARGET bowl=({cx:.4f},{cy:.4f},{rim_z:.4f}) plate=({px:.4f},{py:.4f},{ptop:.4f})")

    def goto(x, y, z, secs=2.0, tag=""):
        r = api.move([x, y, z - Z_BIAS], R_DOWN_X, secs)
        e = api.eef()
        api.log(f"M {tag} want=({x:+.4f},{y:+.4f},{z:.4f}) got=({e[0]:+.4f},{e[1]:+.4f},{e[2]:.4f}) res={r:.4f}")
        return e

    carry_z = ptop + CLEAR_DZ - HELD_BASE_DZ     # eef z that keeps the base above the plate rim

    def holding():
        """own-sensor hold test: a rim bite survives only if the jaws are still
        held apart by the wall.  a slipped bowl closes the jaws below HOLD_GAP
        and the effort flag drops."""
        g = api.gripper()
        return g['effort'] > 1.0 and g['width_m'] > HOLD_GAP, g

    # --- grasp: jaws along base x, straddling the rim wall on the -x side --
    gx, gy = cx - GRASP_R, cy
    ok = False
    for k, dz in enumerate(GRASP_DZ_TRIES):
        if k:                                    # re-perceive: a failed try nudges the bowl
            H = heightmap(api.capture("cam_high"))
            for c in clusters(H, TABLE_Z + 0.011):
                if 0.095 < max(c['w'], c['h']) < 0.15 and abs(c['ztop'] - rim_z) < 0.02 \
                        and abs(c['cx'] - cx) < 0.06 and abs(c['cy'] - cy) < 0.06:
                    rd = ring_depth(c)
                    if rd is not None and rd[3] > 0.030:
                        cx, cy, rim_z = rd[0], rd[1], c['ztop']
            gx, gy = cx - GRASP_R, cy
            api.log(f"RETRY {k} dz={dz:+.3f} bowl=({cx:.4f},{cy:.4f},{rim_z:.4f})")
        api.grip(0.08)
        goto(gx, gy, rim_z + 0.10, 2.5 if k == 0 else 2.0, f"above{k}")
        goto(gx, gy, rim_z + 0.04, 1.5, f"near{k}")
        goto(gx, gy, rim_z + dz + Z_BIAS, 1.5, f"down{k}")
        api.grip(0.0)
        api.settle(0.5)
        api.log(f"CLOSED{k} grip={api.gripper()} eef={api.eef().tolist()}")
        goto(gx, gy, rim_z + 0.10, 1.5, f"lift1_{k}")
        goto(gx, gy, carry_z, 2.0, f"lift2_{k}")
        api.settle(0.3)
        ok, g = holding()
        api.log(f"HELD{k} ok={ok} grip={g} eef={api.eef().tolist()}")
        if ok:
            break
        api.grip(0.08)                            # drop whatever it is and retry
        api.settle(0.3)
        goto(gx, gy, rim_z + 0.12, 2.0, f"reset{k}")

    # --- carry: put the HELD BOWL centre over the plate centre ------------
    if not ok:
        api.log("NOHOLD: every grasp try slipped; carrying the last one anyway")
    tx, ty = px - GRASP_R, py
    goto(tx, ty, carry_z, 3.0, "carry")
    e = api.eef()
    ex, ey = tx + (tx - e[0]), ty + (ty - e[1])   # one bounded bias-cancel pass
    ex = tx + float(np.clip(ex - tx, -0.03, 0.03))
    ey = ty + float(np.clip(ey - ty, -0.03, 0.03))
    goto(ex, ey, carry_z, 2.0, "carry2")

    # --- lower until the base is just inside the plate, then release ------
    place_z = ptop - 0.004 - HELD_BASE_DZ
    goto(ex, ey, place_z, 2.0, "place")
    api.settle(0.3)
    api.log(f"PRE-RELEASE grip={api.gripper()} eef={api.eef().tolist()}")
    api.grip(0.08)
    api.settle(0.6)
    api.log(f"RELEASED grip={api.gripper()} eef={api.eef().tolist()}")

    # --- retreat straight up, then back off ------------------------------
    # retreat STRAIGHT up only -- the open jaws still straddle the bowl rim, so
    # any lateral move here would drag the bowl back off the plate.
    goto(ex, ey, place_z + 0.05, 1.5, "up")
    api.settle(0.5)
    dump_frame(api, "cam_high", "t3")
