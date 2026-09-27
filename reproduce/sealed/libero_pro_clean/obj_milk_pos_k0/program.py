"""c2clean obj_milk_pos_k0 -- v3: perceive milk + basket, pick, place."""

import numpy as np

PROVENANCE = {
    "WS_X": {"source": "debug-seed 51/53/57/61 cam_high point cloud: props occupy x[-0.21,0.18]", "allowed": True},
    "WS_Y": {"source": "debug-seed 51/53/57/61 cam_high point cloud: props occupy y[-0.27,0.35]", "allowed": True},
    "Z_TABLE": {"source": "debug-seed 51 cam_high: modal deprojected z of the empty surface = 0.0012 m", "allowed": True},
    "Z_BAND": {"source": "debug-seed 51 cam_high: all props top out below 0.15; robot arm column occupies z>0.20", "allowed": True},
    "GRID_RES": {"source": "generic: 4 mm occupancy grid, ~1 px of cam_high at 1.1 m", "allowed": True},
    "TALL_Z": {"source": "debug-seed 51: two carton clusters top 0.140-0.142, next prop tops 0.081/0.030", "allowed": True},
    "PROBE_XY": {"source": "debug-seed 51/53/57/61: no cluster within 6 cm of (0.02,0.12)", "allowed": True},
    "TIP_DZ": {"source": "v4 debug seeds 51/57: hand held over empty table, cam_high lowest gripper point sits 0.004-0.008 below api.eef() z at eef 0.36/0.30/0.24", "allowed": True},
    "GRIP_DROP": {"source": "v1 debug seed 51: milk y-width is 0.050 at every height 0.00-0.15, ridge at top z; 0.028 below the ridge is on the gable end faces and above the 0.106 shoulder", "allowed": True},
    "W_LO/W_HI": {"source": "v1 debug seed 51: milk cross-section 0.048-0.052 in y at all heights", "allowed": True},
    "HANG": {"source": "v1 debug seed 51: carton base at table z=0.001, gripped 0.028 below a 0.141 ridge -> ~0.112 of carton hangs below the fingertips", "allowed": True},
    "Z_CARRY": {"source": "debug seeds: basket rim top 0.142; carried carton base sits ~0.05 below the fingertips", "allowed": True},
    "Z_DROP": {"source": "debug seeds: basket rim 0.142, interior open to the table", "allowed": True},
    "OPEN_W": {"source": "debug seed 51 GRIP0: gripper rests open at width 0.0778", "allowed": True},
    "PARK_XY": {"source": "debug seeds 51-65: no cluster within 8 cm of (0.10,-0.30)", "allowed": True},
}

WS_X = (-0.28, 0.25)
WS_Y = (-0.35, 0.45)
Z_BAND = (0.006, 0.19)
GRID_RES = 0.004
TALL_Z = 0.11
PROBE_XY = (0.02, 0.12)


def _cloud(api, cam="cam_high"):
    f = api.capture(cam)
    d = np.asarray(f.depth, dtype=np.float64)
    K = np.asarray(f.intrinsics, dtype=np.float64)
    T = np.asarray(f.t_base_cam, dtype=np.float64)
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]
    x = (uu - K[0, 2]) / K[0, 0] * d
    y = (vv - K[1, 2]) / K[1, 1] * d
    pc = np.stack([x, y, d], -1)
    pb = pc @ T[:3, :3].T + T[:3, 3]
    return pb.reshape(-1, 3), np.asarray(f.rgb, dtype=np.float64).reshape(-1, 3) / 255.0


def _clusters(P, C, res=GRID_RES, minpts=30):
    ix = ((P[:, 0] - WS_X[0]) / res).astype(int)
    iy = ((P[:, 1] - WS_Y[0]) / res).astype(int)
    nx = int((WS_X[1] - WS_X[0]) / res) + 2
    ny = int((WS_Y[1] - WS_Y[0]) / res) + 2
    occ = np.zeros((nx, ny), bool)
    occ[ix, iy] = True
    lab = np.zeros((nx, ny), int)
    cur = 0
    for i in range(nx):
        for j in range(ny):
            if occ[i, j] and not lab[i, j]:
                cur += 1
                lab[i, j] = cur
                st = [(i, j)]
                while st:
                    a, b = st.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            u, v = a + da, b + db
                            if 0 <= u < nx and 0 <= v < ny and occ[u, v] and not lab[u, v]:
                                lab[u, v] = cur
                                st.append((u, v))
    pl = lab[ix, iy]
    out = []
    for k in range(1, cur + 1):
        sel = pl == k
        if sel.sum() < minpts:
            continue
        Q, D = P[sel], C[sel]
        top = float(np.percentile(Q[:, 2], 99.0))
        ridge = Q[:, 2] > top - 0.006
        col = D.mean(0)
        out.append(dict(
            n=int(sel.sum()), top=top,
            x=float(Q[:, 0].mean()), y=float(Q[:, 1].mean()),
            xmin=float(Q[:, 0].min()), xmax=float(Q[:, 0].max()),
            ymin=float(Q[:, 1].min()), ymax=float(Q[:, 1].max()),
            ridge_x=float(np.median(Q[ridge, 0])), ridge_y=float(np.median(Q[ridge, 1])),
            rgb=[round(float(v), 3) for v in col],
            blue=float(col[2] / max(col.sum(), 1e-6)),
        ))
    return sorted(out, key=lambda d: -d["top"])


def perceive(api):
    P, C = _cloud(api)
    m = ((P[:, 0] > WS_X[0]) & (P[:, 0] < WS_X[1]) & (P[:, 1] > WS_Y[0]) &
         (P[:, 1] < WS_Y[1]) & (P[:, 2] > Z_BAND[0]) & (P[:, 2] < Z_BAND[1]))
    cs = _clusters(P[m], C[m])
    for c in cs:
        api.log("CLUS top=%.3f n=%4d xy=(%+.3f,%+.3f) ridge=(%+.3f,%+.3f) "
                "x[%.3f,%.3f] y[%.3f,%.3f] rgb=%s blue=%.3f"
                % (c["top"], c["n"], c["x"], c["y"], c["ridge_x"], c["ridge_y"],
                   c["xmin"], c["xmax"], c["ymin"], c["ymax"], c["rgb"], c["blue"]))
    basket = max(cs, key=lambda c: (c["xmax"] - c["xmin"]) * (c["ymax"] - c["ymin"]))
    tall = [c for c in cs if c["top"] > TALL_Z and c is not basket]
    api.log("BASKET xy=(%+.3f,%+.3f) TALL=%d" % (basket["x"], basket["y"], len(tall)))
    milk = max(tall, key=lambda c: c["blue"]) if tall else None
    if milk is not None:
        gx = milk["ridge_x"]
        gy = 0.5 * (milk["ymin"] + milk["ymax"])
        api.log("MILK ridge_x=%.3f facex=%.3f grasp=(%+.3f,%+.3f) top=%.3f blue=%.3f"
                % (milk["ridge_x"], milk["xmax"], gx, gy, milk["top"], milk["blue"]))
    return cs, basket, milk


TIP_DZ = 0.006
GRIP_DROP = 0.028
W_LO, W_HI = 0.030, 0.062
HANG = 0.115
OPEN_W = 0.078
Z_CARRY = 0.300
Z_DROP = 0.200
PARK_XY = (0.10, -0.30)


def goto(api, xyz, seconds=2.0, tol=0.006, tries=2):
    """Command, read back, cancel the standing bias once."""
    tgt = np.asarray(xyz, dtype=float)
    cmd = tgt.copy()
    r = api.move(cmd.tolist(), seconds=seconds)
    for _ in range(tries - 1):
        e = np.asarray(api.eef())
        err = tgt - e
        if np.abs(err).max() <= tol:
            break
        cmd = cmd + err
        r = api.move(cmd.tolist(), seconds=seconds)
    return r, np.asarray(api.eef())


def run(api):
    api.log("EEF0 %s" % np.round(api.eef(), 4).tolist())
    cs, basket, milk = perceive(api)
    if milk is None:
        api.log("ABORT no carton")
        return
    gx = milk["ridge_x"]
    gy = 0.5 * (milk["ymin"] + milk["ymax"])
    ridge = milk["top"]
    bx, by = basket["x"], basket["y"]
    z_tip = ridge - GRIP_DROP
    z_cmd = z_tip + TIP_DZ
    api.log("PLAN grasp=(%+.3f,%+.3f) ridge=%.3f z_tip=%.3f z_cmd=%.3f basket=(%+.3f,%+.3f)"
            % (gx, gy, ridge, z_tip, z_cmd, bx, by))

    api.grip(OPEN_W)
    api.settle(0.3)
    r, e = goto(api, [gx, gy, ridge + 0.12], seconds=2.0)
    api.log("A hover res=%.4f eef=%s" % (r, np.round(e, 4).tolist()))
    r, e = goto(api, [gx, gy, z_cmd], seconds=2.0)
    api.log("B descend res=%.4f eef=%s" % (r, np.round(e, 4).tolist()))
    api.grip(0.0)
    api.settle(0.5)
    g = api.gripper()
    api.log("C close grip=%s held=%s" % (g, W_LO < g["width_m"] < W_HI))

    if not (W_LO < g["width_m"] < W_HI):
        api.grip(OPEN_W)
        api.settle(0.3)
        z2 = z_cmd - 0.030
        r, e = goto(api, [gx, gy, z2], seconds=2.0)
        api.log("B2 retry lower res=%.4f eef=%s" % (r, np.round(e, 4).tolist()))
        api.grip(0.0)
        api.settle(0.5)
        g = api.gripper()
        api.log("C2 close grip=%s held=%s" % (g, W_LO < g["width_m"] < W_HI))

    r, e = goto(api, [gx, gy, Z_CARRY], seconds=2.0)
    api.log("D lift res=%.4f eef=%s grip=%s" % (r, np.round(e, 4).tolist(), api.gripper()))
    r, e = goto(api, [bx, by, Z_CARRY], seconds=3.0)
    api.log("E over res=%.4f eef=%s grip=%s" % (r, np.round(e, 4).tolist(), api.gripper()))
    r, e = goto(api, [bx, by, Z_DROP], seconds=2.0)
    api.log("F lower res=%.4f eef=%s grip=%s" % (r, np.round(e, 4).tolist(), api.gripper()))
    api.grip(OPEN_W)
    api.settle(0.6)
    api.log("G release grip=%s" % api.gripper())
    r, e = goto(api, [bx, by, Z_CARRY], seconds=2.0)
    api.log("H up res=%.4f" % r)
    r, e = goto(api, [PARK_XY[0], PARK_XY[1], 0.30], seconds=3.0)
    api.log("I park res=%.4f eef=%s" % (r, np.round(e, 4).tolist()))
    api.settle(0.5)
    api.log("--- post ---")
    perceive(api)
