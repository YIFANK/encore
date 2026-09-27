"""v5 -- v4 with a slip detector that actually separates hold from slip.

v4 (13/15) regressed on v3 (14/15) because it gated the retry on the harness's
own `effort` flag, which is `gap > 0.005`: a thin but perfectly good bite on
the rim reads gap 0.0047 and was thrown away, and the retry then wrecked a
grasp that had worked. Measured over 11 debug-seed grasps, what separates the
two is not the absolute gap but whether the gap SURVIVES the lift:

    held    close -> post-lift  0.0047->0.0047 .. 0.0084->0.0069   (ratio 0.82-1.00)
    slipped                     0.0073->0.0019 .. 0.0075->0.0046   (ratio 0.26-0.61)

which is the mechanism: the jaws keep the rim wall's thickness while the bowl
is there, and collapse toward zero once it is gone. v5 gates on that ratio and
adds a free second opinion — after the carry, re-perceive and check the bowl is
no longer sitting where it started.
"""
import numpy as np

PROVENANCE = {
    "RES": {"source": "my choice: 4mm top-down raster cell", "allowed": True},
    "WORKSPACE": {"source": "debug seeds 51-65 cam_high cloud extent", "allowed": True},
    "TABLE_Z": {"source": "measured per-episode: median height-map z in 0.85-0.915 (0.9012 on debug seeds)", "allowed": True},
    "SLAB_BAND": {"source": "debug 51/53/54/55: large uniform grey slab tops at 0.931; bowl rim at 0.950", "allowed": True},
    "RIM_BAND": {"source": "debug 51/53/54/55: a 0.937 cut isolates the bowl rim from the slab", "allowed": True},
    "GRASP_DEPTH": {"source": "debug 51: rim 0.950 over table 0.901; 0.02 below the rim keeps the tips off the bowl floor", "allowed": True},
    "HOLD_RATIO": {"source": "11 debug-seed grasps (v3/v4 logs): post-lift/close gap is 0.82-1.00 when the bowl is held and 0.26-0.61 when it slipped", "allowed": True},
    "GAP_FLOOR": {"source": "same 11 grasps: every held bite read >= 0.0047", "allowed": True},
    "MOVED_MIN": {"source": "my choice: a bowl still within 30mm of its start xy at rim height has not been picked up", "allowed": True},
    "ATTEMPT_AIMS": {"source": "my choice of rim-aim variation; radii/depths bracket the v3 aim that held on 14/15", "allowed": True},
    "PLACE_CLEAR": {"source": "my choice: 8mm release clearance above the measured slab top", "allowed": True},
}

RES = 0.004
X0, Y0, X1, Y1 = -0.40, -0.50, 0.35, 0.50
HOLD_RATIO = 0.70
GAP_FLOOR = 0.003
MOVED_MIN = 0.030
PLACE_CLEAR = 0.008
# (radial inset from the measured rim radius, tangential x shift, descent below rim)
ATTEMPT_AIMS = [(0.000, 0.000, 0.020), (0.006, 0.000, 0.026), (0.003, 0.020, 0.023)]


def heightmap(api, cam="cam_high"):
    f = api.capture(cam)
    d = f.depth.astype(np.float64)
    K, T = np.asarray(f.intrinsics, float), np.asarray(f.t_base_cam, float)
    h, w = d.shape
    v, u = np.mgrid[0:h, 0:w]
    z = np.where(np.isfinite(d) & (d > 0), d, np.nan)
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    B = np.stack([x, y, z, np.ones_like(z)], -1) @ T.T
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    ok = np.isfinite(Z) & (X > X0) & (X < X1) & (Y > Y0) & (Y < Y1)
    nx, ny = int((X1 - X0) / RES), int((Y1 - Y0) / RES)
    H = np.full((nx, ny), np.nan)
    C = np.zeros((nx, ny, 3))
    ix = np.clip(((X - X0) / RES).astype(int), 0, nx - 1)
    iy = np.clip(((Y - Y0) / RES).astype(int), 0, ny - 1)
    rgb = f.rgb.astype(np.float64)
    for a, b, zz, cc in zip(ix[ok], iy[ok], Z[ok], rgb[ok]):
        if not (H[a, b] >= zz):
            H[a, b] = zz
            C[a, b] = cc
    return H, C


def blobs(H, C, zlo, zhi, minc=20):
    m = np.isfinite(H) & (H > zlo) & (H < zhi)
    lab = np.zeros(H.shape, int)
    out = []
    for a, b in np.argwhere(m):
        if lab[a, b]:
            continue
        st, cells = [(a, b)], []
        lab[a, b] = 1
        while st:
            p, q = st.pop()
            cells.append((p, q))
            for dp in (-1, 0, 1):
                for dq in (-1, 0, 1):
                    r, c = p + dp, q + dq
                    if 0 <= r < H.shape[0] and 0 <= c < H.shape[1] and m[r, c] and not lab[r, c]:
                        lab[r, c] = 1
                        st.append((r, c))
        if len(cells) < minc:
            continue
        ar = np.array(cells)
        xs, ys = X0 + ar[:, 0] * RES, Y0 + ar[:, 1] * RES
        zs = H[ar[:, 0], ar[:, 1]]
        out.append(dict(n=len(cells), xlo=float(xs.min()), xhi=float(xs.max()),
                        ylo=float(ys.min()), yhi=float(ys.max()),
                        cx=float(xs.mean()), cy=float(ys.mean()),
                        ztop=float(zs.max()), zmed=float(np.median(zs)),
                        col=[round(float(v), 1) for v in C[ar[:, 0], ar[:, 1]].mean(0)]))
    out.sort(key=lambda d: -d["n"])
    return out


def log_blobs(api, tag, H, C):
    for lo, hi, t in ((0.915, 0.937, "mid"), (0.937, 1.05, "hi")):
        for d in blobs(H, C, lo, hi):
            api.log(f"{tag}/{t} n={d['n']} x[{d['xlo']:.3f},{d['xhi']:.3f}] y[{d['ylo']:.3f},{d['yhi']:.3f}] "
                    f"ctr=({d['cx']:.3f},{d['cy']:.3f}) ztop={d['ztop']:.3f} zmed={d['zmed']:.3f} rgb={d['col']}")


def find_bowl(H, C):
    """Round, light-coloured cap standing above the slab band."""
    cands = []
    for d in blobs(H, C, 0.937, 1.02):
        w, l = d["xhi"] - d["xlo"], d["yhi"] - d["ylo"]
        if not (0.07 < w < 0.17 and 0.07 < l < 0.17):
            continue
        if min(w, l) / max(w, l) < 0.6:
            continue
        if sum(d["col"]) / 3 < 70:
            continue
        cands.append(d)
    return max(cands, key=lambda d: d["n"]) if cands else None


def find_stove(H, C):
    """The wide flat dark slab in the band between the table and the bowl rim."""
    cands = []
    for d in blobs(H, C, 0.915, 0.937, minc=300):
        if d["xhi"] - d["xlo"] < 0.14 or d["yhi"] - d["ylo"] < 0.14:
            continue
        if sum(d["col"]) / 3 > 130:
            continue
        cands.append(d)
    return max(cands, key=lambda d: d["n"]) if cands else None


def grasp_once(api, bx, by, rim_z, rim_r, aim):
    """One rim pinch. Returns (held, gap)."""
    inset, tang, depth = aim
    gx, gy = bx + tang, by + rim_r - inset
    api.grip(0.08)
    api.move([gx, gy, rim_z + 0.10], seconds=1.4)
    r = api.move([gx, gy, rim_z - depth], seconds=0.8)
    api.log(f"DOWN aim={aim} res={r:.4f} eef={api.eef().round(4).tolist()}")
    api.grip(0.0)
    close_z = float(api.eef()[2])
    g = api.gripper()
    api.log(f"CLOSE gap={g['width_m']:.4f} effort={g['effort']} close_z={close_z:.4f}")
    # staged lift: a single big command saturates the controller and the rim
    # pinch can shed the bowl (seed 54, v3)
    api.move([gx, gy, rim_z + 0.02], seconds=0.6)
    g1 = api.gripper()
    api.move([gx, gy, rim_z + 0.12], seconds=0.9)
    g2 = api.gripper()
    # the harness's own effort flag is just gap > 0.005 and rejects thin bites
    # that hold perfectly well; a slip is a gap that COLLAPSES across the lift
    held = g2["width_m"] > GAP_FLOOR and g2["width_m"] > HOLD_RATIO * max(g["width_m"], 1e-6)
    api.log(f"LIFT gap1={g1['width_m']:.4f} gap2={g2['width_m']:.4f} "
            f"effort={g2['effort']} held={held} eef={api.eef().round(4).tolist()}")
    return held, gx, gy, close_z


def run(api):
    H, C = heightmap(api)
    log_blobs(api, "PRE", H, C)
    table = float(np.nanmedian(H[np.isfinite(H) & (H > 0.85) & (H < 0.915)]))
    stove = find_stove(H, C)
    bowl = find_bowl(H, C)
    api.log(f"TABLE {table:.4f} "
            f"BOWL {bowl and (round(bowl['cx'],3), round(bowl['cy'],3), round(bowl['zmed'],3))} "
            f"STOVE {stove and (round(stove['cx'],3), round(stove['cy'],3), round(stove['ztop'],3))}")
    if bowl is None or stove is None:
        return "perception failed"
    sx, sy, sz = stove["cx"], stove["cy"], stove["ztop"]

    for k, aim in enumerate(ATTEMPT_AIMS):
        if k:                      # re-perceive: a failed pinch can shift the bowl
            H, C = heightmap(api)
            b = find_bowl(H, C)
            if b is not None:
                bowl = b
        bx, by, rim_z = bowl["cx"], bowl["cy"], bowl["zmed"]
        rim_r = ((bowl["xhi"] - bowl["xlo"]) + (bowl["yhi"] - bowl["ylo"])) / 4
        api.log(f"ATTEMPT {k} bowl=({bx:.3f},{by:.3f},{rim_z:.3f}) rim_r={rim_r:.3f}")
        held, gx, gy, close_z = grasp_once(api, bx, by, rim_z, rim_r, aim)
        if not held:
            api.grip(0.08)
            api.move([gx, gy, rim_z + 0.14], seconds=0.8)
            continue

        # carry: the bowl centre trails the eef by the rim offset it was
        # pinched at, so aim the same offset off the slab centre
        px, py = sx + (gx - bx), sy + (gy - by)
        api.move([px, py, rim_z + 0.12], seconds=1.3)
        api.log(f"OVER eef={api.eef().round(4).tolist()} gap={api.gripper()['width_m']:.4f}")

        # free second opinion: with the hand now over the slab, the bowl's old
        # seat is in plain view. Still occupied at rim height => never picked up.
        Hc, Cc = heightmap(api)
        bleft = find_bowl(Hc, Cc)
        if bleft is not None and np.hypot(bleft["cx"] - bx, bleft["cy"] - by) < MOVED_MIN:
            api.log(f"CARRY-CHECK bowl still at ({bleft['cx']:.3f},{bleft['cy']:.3f}) -- retry")
            api.grip(0.08)
            continue

        # the eef height at the moment of closing fixes how far the bowl's base
        # hangs below the hand; measure it, don't trust the commanded depth
        hang = close_z - table
        place_z = sz + PLACE_CLEAR + hang
        r = api.move([px, py, place_z], seconds=0.8)
        api.log(f"PLACE res={r:.4f} eef={api.eef().round(4).tolist()} z={place_z:.3f} hang={hang:.4f}")
        api.grip(0.08)
        api.settle(0.5)
        api.move([px, py, rim_z + 0.16], seconds=0.8)
        api.settle(0.3)
        H2, C2 = heightmap(api)
        log_blobs(api, "POST", H2, C2)
        b2 = find_bowl(H2, C2)
        api.log(f"FINAL bowl={b2 and (round(b2['cx'],3), round(b2['cy'],3), round(b2['zmed'],3))} "
                f"stove=({sx:.3f},{sy:.3f},{sz:.3f})")
        return f"v5 placed on attempt {k+1}"

    api.log("NO HOLD after all attempts")
    return "no hold"
