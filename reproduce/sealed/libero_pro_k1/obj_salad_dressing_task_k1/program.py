"""c2k1clean obj_salad_dressing_task_k1 -- v4: v3 plus a grasp-retry ladder.

Intent: "Pick the tomato sauce and place it in the basket".

Perception (v1, validated on debug seeds 51/53/55/57): top-down XY occupancy
clustering over the base-frame cloud separates all six props plus the basket.
The tomato sauce is the SHORT ROUND WARM prop: h~0.078, dx~dy~0.065, body
colour r>b. Its only size twin is a second can whose body is blue (b>r), so
one colour comparison names it. The basket is the wide tall cluster.

Grasp: straight-down, fingertips on the can body, eef z = can_top - 0.035.
Release: over the basket centre at rim + 0.035.
"""
import numpy as np

PROVENANCE = {
    "WS_X": {"source": "debug-seed v0/v1 probe: crop holding every cluster",
             "allowed": True},
    "WS_Y": {"source": "debug-seed v0/v1 probe", "allowed": True},
    "TABLE_BAND": {"source": "generic camera mechanics: band above the fitted "
                             "table plane (v1 measured table_z=0.0025)",
                   "allowed": True},
    "CELL": {"source": "generic: 1 cm top-down occupancy grid", "allowed": True},
    "ARM_H": {"source": "v0 probe: arm cluster h=0.468 vs tallest prop h=0.148",
              "allowed": True},
    "CAN_H": {"source": "v1 probe debug seeds 51/53/55/57: the two short cans "
                        "measure h=0.078; every other prop is 0.017/0.135-0.148",
              "allowed": True},
    "CAN_D": {"source": "v1 probe: can footprint dx=0.064 dy=0.070; mate pack "
                        "carry gripper_state |0.0265|+|0.037| = 0.0635 width",
              "allowed": True},
    "GRASP_DROP": {"source": "mate pack keyframe t=54 grasp ee z=0.046 vs the "
                             "can top 0.081 measured in the v1 probe",
                   "allowed": True},
    "LIFT_Z": {"source": "k1 pack ee_path6 transport apex z=0.309",
               "allowed": True},
    "DROP_OVER_RIM": {"source": "k1 pack keyframe t=128 release z=0.171 vs the "
                                "basket rim 0.142 measured in the v1 probe",
                      "allowed": True},
    "HOLD_EFFORT": {"source": "FairApi contract: effort 3.0 iff holding",
                    "allowed": True},
    "GRASP_LADDER": {"source": "debug-seed v2/v3: the nominal descent lands "
                               "eef at can_top-0.026 (9 mm of controller "
                               "under-reach); the ladder spans that residual "
                               "either way", "allowed": True},
}

WS_X = (-0.32, 0.32)
WS_Y = (-0.40, 0.45)
TABLE_BAND = 0.015
CELL = 0.01
ARM_H = 0.25
CAN_H = (0.055, 0.105)
CAN_D = (0.045, 0.090)
GRASP_DROP = 0.035
LIFT_Z = 0.30
GRASP_LADDER = (0.0, -0.014, 0.014)
DROP_OVER_RIM = 0.035
HOLD_EFFORT = 1.5


# --------------------------------------------------------------- perception
def _cloud(fr):
    d = np.asarray(fr.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(fr.intrinsics, float)
    T = np.asarray(fr.t_base_cam, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    ok = np.isfinite(d) & (d > 0)
    z = np.where(ok, d, 1.0)
    pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], -1)
    return pc @ T[:3, :3].T + T[:3, 3], ok


def _grid_label(occ):
    h, w = occ.shape
    lab = -np.ones((h, w), np.int32)
    n = 0
    for sy, sx in zip(*np.nonzero(occ)):
        if lab[sy, sx] >= 0:
            continue
        stack = [(sy, sx)]
        lab[sy, sx] = n
        while stack:
            y, x = stack.pop()
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < h and 0 <= nx < w and occ[ny, nx] and lab[ny, nx] < 0:
                        lab[ny, nx] = n
                        stack.append((ny, nx))
        n += 1
    return lab, n


def scene(api):
    fr = api.capture("cam_high")
    base, ok = _cloud(fr)
    X, Y, Z = base[..., 0], base[..., 1], base[..., 2]
    inws = ok & (X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1])
    hist, edges = np.histogram(Z[inws], bins=160)
    i0 = int(np.argmax(hist))
    table_z = float(0.5 * (edges[i0] + edges[i0 + 1]))

    sel = inws & (Z > table_z + TABLE_BAND) & (Z < table_z + ARM_H)
    p = np.stack([X[sel], Y[sel], Z[sel]], -1)
    c = fr.rgb[sel].astype(float)

    nx = int((WS_X[1] - WS_X[0]) / CELL) + 1
    ny = int((WS_Y[1] - WS_Y[0]) / CELL) + 1
    gx = np.clip(((p[:, 0] - WS_X[0]) / CELL).astype(int), 0, nx - 1)
    gy = np.clip(((p[:, 1] - WS_Y[0]) / CELL).astype(int), 0, ny - 1)
    cnt = np.zeros((nx, ny), int)
    np.add.at(cnt, (gx, gy), 1)
    lab, n = _grid_label(cnt >= 4)
    cid = lab[gx, gy]

    objs = []
    for i in range(n):
        m = cid == i
        if int(m.sum()) < 60:
            continue
        pp, cc = p[m], c[m]
        ztop = float(np.percentile(pp[:, 2], 97))
        top = pp[:, 2] > ztop - 0.012
        lid = pp[top]
        body = cc[(pp[:, 2] < ztop - 0.012) & (pp[:, 2] > table_z + 0.012)]
        if len(body) < 20:
            body = cc
        objs.append(dict(
            n=int(m.sum()), ztop=ztop, h=ztop - table_z,
            dx=float(pp[:, 0].max() - pp[:, 0].min()),
            dy=float(pp[:, 1].max() - pp[:, 1].min()),
            cx=float(0.5 * (pp[:, 0].max() + pp[:, 0].min())),
            cy=float(0.5 * (pp[:, 1].max() + pp[:, 1].min())),
            lx=float(0.5 * (lid[:, 0].max() + lid[:, 0].min())),
            ly=float(0.5 * (lid[:, 1].max() + lid[:, 1].min())),
            ldx=float(lid[:, 0].max() - lid[:, 0].min()),
            ldy=float(lid[:, 1].max() - lid[:, 1].min()),
            warm=float(body[:, 0].mean() - body[:, 2].mean()),
            body=np.round(body.mean(0), 0).tolist()))
    return table_z, objs


def pick_target(objs):
    cands = [o for o in objs
             if CAN_H[0] <= o["h"] <= CAN_H[1]
             and CAN_D[0] <= o["dx"] <= CAN_D[1]
             and CAN_D[0] <= o["dy"] <= CAN_D[1]]
    if not cands:
        cands = [o for o in objs if CAN_H[0] <= o["h"] <= CAN_H[1]]
    if not cands:
        return None
    return max(cands, key=lambda o: o["warm"])


def pick_basket(objs):
    cands = [o for o in objs if o["h"] > 0.10 and o["dx"] > 0.115 and o["dy"] > 0.115]
    if not cands:
        return None
    return max(cands, key=lambda o: o["n"])


# --------------------------------------------------------------------- run
def run(api):
    api.log("v4 instr=%r" % api.instruction())
    table_z, objs = scene(api)
    api.log("table_z=%.4f nobj=%d" % (table_z, len(objs)))
    for o in objs:
        api.log("obj xy=(%.3f,%.3f) ztop=%.3f h=%.3f d=(%.3f,%.3f) "
                "lid=(%.3f,%.3f) ld=(%.3f,%.3f) warm=%+.0f body=%s"
                % (o["cx"], o["cy"], o["ztop"], o["h"], o["dx"], o["dy"],
                   o["lx"], o["ly"], o["ldx"], o["ldy"], o["warm"], o["body"]))

    tgt = pick_target(objs)
    bsk = pick_basket(objs)
    if tgt is None or bsk is None:
        api.log("ABORT: target=%s basket=%s" % (tgt is not None, bsk is not None))
        return
    tx, ty, ztop = tgt["lx"], tgt["ly"], tgt["ztop"]
    bx, by, brim = bsk["cx"], bsk["cy"], bsk["ztop"]
    api.log("TARGET xy=(%.3f,%.3f) ztop=%.3f | BASKET xy=(%.3f,%.3f) rim=%.3f"
            % (tx, ty, ztop, bx, by, brim))

    api.grip(0.08)
    api.settle(0.2)
    r = api.move([tx, ty, ztop + 0.09], seconds=2.5)
    api.log("hover res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))

    held = False
    for a, extra in enumerate(GRASP_LADDER):
        gz = ztop - GRASP_DROP + extra
        r = api.move([tx, ty, gz], seconds=2.0)
        api.log("a%d descend to %.3f res=%.4f eef=%s"
                % (a, gz, r, np.round(api.eef(), 4).tolist()))
        api.grip(0.0)
        api.settle(0.5)
        g = api.gripper()
        api.log("a%d closed width=%.4f effort=%.2f" % (a, g["width_m"], g["effort"]))
        if g["effort"] < HOLD_EFFORT:
            api.grip(0.08)
            api.settle(0.3)
            api.move([tx, ty, ztop + 0.09], seconds=2.0)
            continue
        r = api.move([tx, ty, LIFT_Z], seconds=2.5)
        g = api.gripper()
        api.log("a%d lifted res=%.4f width=%.4f effort=%.2f eef=%s"
                % (a, r, g["width_m"], g["effort"], np.round(api.eef(), 4).tolist()))
        if g["effort"] >= HOLD_EFFORT:
            held = True
            break
        api.grip(0.08)
        api.settle(0.3)
        api.move([tx, ty, ztop + 0.09], seconds=2.0)
    if not held:
        api.log("NO HOLD after ladder; proceeding to the basket anyway")

    for k, wp in enumerate([[0.5 * (tx + bx), 0.5 * (ty + by), LIFT_Z],
                            [bx, by, LIFT_Z]]):
        r = api.move(wp, seconds=2.5)
        api.log("wp%d res=%.4f eef=%s" % (k, r, np.round(api.eef(), 4).tolist()))

    dz = brim + DROP_OVER_RIM
    r = api.move([bx, by, dz], seconds=2.0)
    g = api.gripper()
    api.log("lowered to %.3f res=%.4f width=%.4f effort=%.2f eef=%s"
            % (dz, r, g["width_m"], g["effort"], np.round(api.eef(), 4).tolist()))

    api.grip(0.08)
    api.settle(0.6)
    api.log("released width=%.4f" % api.gripper()["width_m"])
    api.move([bx, by, LIFT_Z], seconds=2.0)
    api.settle(0.5)

    _, objs2 = scene(api)
    for o in objs2:
        api.log("post xy=(%.3f,%.3f) ztop=%.3f h=%.3f d=(%.3f,%.3f) warm=%+.0f"
                % (o["cx"], o["cy"], o["ztop"], o["h"], o["dx"], o["dy"], o["warm"]))
    api.log("v4 done")
