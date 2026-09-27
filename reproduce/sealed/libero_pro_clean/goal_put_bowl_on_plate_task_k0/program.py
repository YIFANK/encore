"""v2 = v1 + safety: clear-spot selection for the fingertip calibration, a fallback
tip offset, a bounded grasp retry, and a post-release re-perception receipt.

Pipeline: perceive the bottle (tall dark cluster) and the plate (broad bright flat
cluster) from cam_high; calibrate the fingertip offset with a blocked descent on
bare table; grasp the bottle neck top-down; carry it above the bowl; lower it until
its base is just over the plate; release.
"""
import zlib, base64
import numpy as np

PROVENANCE = {
    "WS_BOX": {"source": "debug-seed 51-57 cam_high deprojection: x<-0.45 / |y|>0.6 deprojects to the rear wall, so the workspace crop excludes it", "allowed": True},
    "TABLE_PCT": {"source": "debug-seed 51-57 cam_high depth: the table plane is the dominant low surface (measured z=0.9010-0.9015 on 51,53,55,57)", "allowed": True},
    "MIN_CLUSTER_PX": {"source": "debug-seed 51-57: smallest real prop cluster is ~1000 px; noise blobs are <200", "allowed": True},
    "BOTTLE_MIN_H": {"source": "debug-seed 51-57: the bottle is the only free prop taller than 0.10 m (measured 0.158)", "allowed": True},
    "BOTTLE_MAX_RGB": {"source": "debug-seed 51-57: bottle cluster mean RGB ~18; every other cluster >60", "allowed": True},
    "GRASP_H": {"source": "debug-seed 51-57 height profile of the bottle cluster: neck y-width 0.0134-0.0151 over 0.105-0.150 above table; 0.125 is mid-neck", "allowed": True},
    "PLATE_MAX_H/PLATE_MIN_RGB": {"source": "debug-seed 51-57: plate cluster top 0.0191 above table, mean RGB ~138-150; the bowl is 0.051 tall and darker", "allowed": True},
    "TIP_OFF_FALLBACK": {"source": "debug-seed 51,53,55,57 blocked-descent calibration: eef z at table contact minus table z = 0.0080 on all four", "allowed": True},
    "CAL_SPOTS": {"source": "debug-seed 51-57 cam_high segmentation: candidate spots with no above-table cluster within 0.09 m", "allowed": True},
    "NECK_LO/NECK_HI": {"source": "debug-seed 51-57 bottle height profile: 0.105-0.145 above table is pure neck (y-width 0.0134-0.0151); every other free prop tops out at 0.059", "allowed": True},
    "BODY_LO/BODY_HI": {"source": "debug-seed 51-57: bottle body is 0.043 wide up to 0.08; the bowl (the only prop that abuts it) is 0.051 tall, so 0.056-0.080 sees the bottle alone", "allowed": True},
    "FREE_Y_MIN": {"source": "debug-seed 51-65: the cabinet/rack cluster occupies y<-0.119; every free prop sits at y>-0.104", "allowed": True},
    "ARM_Z_GUARD": {"source": "debug-seed 51-57: the parked arm's lowest point is the fingertip at eef z 1.1733 - 0.008 = table+0.264, so a 0.24 cap excludes it from the bottle column", "allowed": True},
    "CARRY_CLEAR": {"source": "debug-seed 51-57: tallest obstacle between bottle and plate is the bowl at 0.051 above table; carry keeps the bottle base 0.09 above the table", "allowed": True},
    "PLACE_GAP": {"source": "debug-seed 51-57: release with the bottle base 0.012 above the plate top (0.0191) so the bottle drops onto the plate", "allowed": True},
}

TIP_OFF_FALLBACK = 0.0080
GRASP_H = 0.125
NECK_LO, NECK_HI = 0.105, 0.145
BODY_LO, BODY_HI = 0.056, 0.080
FREE_Y_MIN = -0.11
CARRY_CLEAR = 0.09
PLACE_GAP = 0.012
CAL_SPOTS = [(0.00, 0.28), (0.15, 0.26), (0.15, -0.26), (0.00, -0.32)]


def _dump(api, tag, fr):
    rgb = np.asarray(fr.rgb, dtype=np.uint8)
    d = np.asarray(fr.depth, dtype=np.float32)
    dmm = np.clip(np.nan_to_num(d, nan=0.0) * 1000.0, 0, 65535).astype(np.uint16)
    api.log("INTR %s %s" % (tag, np.asarray(fr.intrinsics).ravel().tolist()))
    api.log("EXTR %s %s" % (tag, np.asarray(fr.t_base_cam).ravel().tolist()))
    for name, arr in (("rgb", rgb), ("dep", dmm)):
        blob = base64.b64encode(zlib.compress(arr.tobytes(), 6)).decode()
        chunks = [blob[i:i + 1900] for i in range(0, len(blob), 1900)]
        api.log("IMG %s %s nchunks=%d" % (tag, name, len(chunks)))
        for i, c in enumerate(chunks):
            api.log("IMGC %s %s %d %s" % (tag, name, i, c))


def _cloud(fr):
    d = np.asarray(fr.depth, dtype=np.float32)
    K = np.asarray(fr.intrinsics, dtype=np.float64)
    T = np.asarray(fr.t_base_cam, dtype=np.float64)
    H, W = d.shape
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    x = (uu - K[0, 2]) / K[0, 0] * d
    y = (vv - K[1, 2]) / K[1, 1] * d
    P = np.stack([x, y, d, np.ones_like(d)], -1)
    B = P @ T.T
    return B[..., 0], B[..., 1], B[..., 2]


def _label(mask):
    """4-connected labeling (no scipy in the sandbox)."""
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    parent = [0]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    nxt = 1
    for v in range(H):
        for u in np.nonzero(mask[v])[0]:
            up = lab[v - 1, u] if v > 0 else 0
            lf = lab[v, u - 1] if u > 0 else 0
            if up and lf:
                lab[v, u] = min(up, lf)
                ra, rb = find(up), find(lf)
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)
            elif up or lf:
                lab[v, u] = up or lf
            else:
                lab[v, u] = nxt
                parent.append(nxt)
                nxt += 1
    for i in range(1, nxt):
        parent[i] = find(i)
    remap = np.array([0] + [parent[i] for i in range(1, nxt)], np.int32)
    return remap[lab], nxt


def perceive(api, tag=""):
    fr = api.capture("cam_high")
    rgb = np.asarray(fr.rgb, dtype=np.float32)
    X, Y, Z = _cloud(fr)
    ws = np.isfinite(Z) & (Z > 0.5) & (Z < 1.6) & (X > -0.45) & (X < 0.45) & (np.abs(Y) < 0.6)
    zt = float(np.median(Z[ws & (Z < np.percentile(Z[ws], 60))]))
    api.log("TABLE%s %.4f" % (tag, zt))
    above = ws & (Z > zt + 0.012)
    lab, n = _label(above)
    out = {"zt": zt, "occ": []}
    bottle = None
    plate = None
    for i in range(1, n):
        m = lab == i
        cnt = int(m.sum())
        if cnt < 200:
            continue
        h = float(Z[m].max() - zt)
        col = float(rgb[m].mean())
        xs = (float(X[m].min()), float(X[m].max()))
        ys = (float(Y[m].min()), float(Y[m].max()))
        api.log("CLUSTER%s n=%d h=%.3f rgb=%.0f x=[%.3f,%.3f] y=[%.3f,%.3f]" % (tag, cnt, h, col, xs[0], xs[1], ys[0], ys[1]))
        out["occ"].append((xs, ys))
        if cnt > 1200 and h < 0.035 and col > 110:
            if plate is None or cnt > int(plate.sum()):
                plate = m

    # --- bottle: height-band first (the bowl fuses with it in the table-level mask) ---
    neck = ws & (Z > zt + NECK_LO) & (Z < zt + NECK_HI) & (Y > FREE_Y_MIN)
    nlab, nn = _label(neck)
    cand = None
    for i in range(1, nn):
        m = nlab == i
        cnt = int(m.sum())
        if not (10 <= cnt <= 400):
            continue
        yw = float(Y[m].max() - Y[m].min())
        xw = float(X[m].max() - X[m].min())
        col = float(rgb[m].mean())
        api.log("NECK%s n=%d yw=%.3f xw=%.3f rgb=%.0f x=%.3f y=%.3f" % (
            tag, cnt, yw, xw, col, float(X[m].mean()), float(Y[m].mean())))
        if not (0.006 < yw < 0.030 and xw < 0.035 and col < 90):
            continue
        if cand is None or col < cand[1]:
            cand = (m, col)
    if cand is not None:
        m = cand[0]
        yc0 = float(Y[m].max() + Y[m].min()) / 2.0
        xc0 = float(X[m].max()) - float(Y[m].max() - Y[m].min()) / 2.0
        # upper body: above every other free prop, so an abutting bowl cannot leak in
        body = (ws & (Z >= zt + BODY_LO) & (Z < zt + BODY_HI)
                & (np.abs(Y - yc0) < 0.05) & (np.abs(X - xc0) < 0.05))
        if int(body.sum()) >= 30:
            r = float(Y[body].max() - Y[body].min()) / 2.0
            yc = float(Y[body].max() + Y[body].min()) / 2.0
            xb = X[body]
            xc = float(np.median(xb[xb > np.percentile(xb, 92)])) - r
        else:
            r, yc, xc = 0.0215, yc0, xc0
        col = ws & (np.abs(Y - yc) < 0.03) & (np.abs(X - xc) < 0.03) & (Z < zt + 0.24)
        h = float(Z[col].max() - zt) if int(col.sum()) > 0 else 0.158
        h = float(min(max(h, 0.10), 0.22))
        out["bottle"] = (xc, yc, h, r)
        api.log("BOTTLE%s x=%.4f y=%.4f h=%.4f r=%.4f nx=%.4f ny=%.4f" % (tag, xc, yc, h, r, xc0, yc0))
    if plate is not None:
        px = float(X[plate].min() + X[plate].max()) / 2.0
        py = float(Y[plate].min() + Y[plate].max()) / 2.0
        pz = float(Z[plate].max() - zt)
        out["plate"] = (px, py, pz)
        api.log("PLATE%s x=%.4f y=%.4f top=%.4f" % (tag, px, py, pz))
    return out


def _clear_spot(occ):
    for sx, sy in CAL_SPOTS:
        ok = True
        for (x0, x1), (y0, y1) in occ:
            if sx > x0 - 0.09 and sx < x1 + 0.09 and sy > y0 - 0.09 and sy < y1 + 0.09:
                ok = False
                break
        if ok:
            return sx, sy
    return None


def run(api):
    api.log("INSTR %s" % api.instruction())
    api.grip(0.08)
    s = perceive(api)
    zt = s["zt"]

    tip = TIP_OFF_FALLBACK
    spot = _clear_spot(s["occ"])
    if spot is not None:
        api.move([spot[0], spot[1], zt + 0.16], seconds=2.0)
        res = api.move([spot[0], spot[1], zt - 0.05], seconds=3.0)
        e = api.eef()
        meas = float(e[2]) - zt
        api.log("CAL spot=%s res=%.4f eef=%s tip=%.4f" % (str(spot), res, np.asarray(e).tolist(), meas))
        if res > 0.02 and -0.01 < meas < 0.05:   # descent was blocked by the table
            tip = meas
        api.move([spot[0], spot[1], zt + 0.20], seconds=2.0)
    else:
        api.log("CAL skipped: no clear spot")
    api.log("TIP %.4f" % tip)

    if "bottle" not in s or "plate" not in s:
        api.log("ABORT missing perception")
        return
    bx, by, bh, br = s["bottle"]
    px, py, ptop = s["plate"]

    zg = zt + GRASP_H + tip
    held = False
    for attempt in range(2):
        api.move([bx, by, zg + 0.10], seconds=2.0)
        r1 = api.move([bx, by, zg], seconds=2.0)
        api.grip(0.004)
        api.settle(0.3)
        g = api.gripper()
        api.log("GRASP try=%d res=%.4f %s" % (attempt, r1, g))
        if float(g["effort"]) >= 2.5 and float(g["width_m"]) > 0.005:
            held = True
            break
        api.grip(0.08)
        api.move([bx, by, zg + 0.12], seconds=2.0)
        s2 = perceive(api, "2")
        if "bottle" in s2:
            bx, by, bh, br = s2["bottle"]
            zg = s2["zt"] + GRASP_H + tip
    if not held:
        api.log("NOGRASP")
        return

    carry_z = zt + bh + CARRY_CLEAR + tip
    api.move([bx, by, carry_z], seconds=2.0)
    api.log("LIFT %s %s" % (api.gripper(), np.asarray(api.eef()).tolist()))
    api.move([px, py, carry_z], seconds=3.0)
    api.log("OVERPLATE %s %s" % (api.gripper(), np.asarray(api.eef()).tolist()))
    place_z = zt + ptop + PLACE_GAP + GRASP_H + tip
    r2 = api.move([px, py, place_z], seconds=2.5)
    api.log("PLACE res=%.4f %s %s" % (r2, api.gripper(), np.asarray(api.eef()).tolist()))
    api.grip(0.08)
    api.settle(0.5)
    api.move([px, py, place_z + 0.12], seconds=2.0)
    api.move([px - 0.12, py, place_z + 0.16], seconds=2.0)
    api.settle(0.5)
    perceive(api, "3")
