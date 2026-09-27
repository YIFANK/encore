"""v3 — same mechanism as v2, but the bottle and the bowl are perceived in two
disjoint height bands so that they cannot fuse when they abut.

v2 clustered one band (table+0.015 .. table+0.22) and scored 11/15: the four
losses were all "no target", the bottle and the bowl merging into a single
~3400 px component whose footprint failed both gates. Here the bottle is found
in a band the bowl cannot reach (table+0.10 up), its footprint is punched out
of the frame, and the bowl is then fitted as a circle on the surviving rim.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "GRASP_DZ": {
        "source": "pack c2k1clean_goal_put_wine_top_cabinet_task_k1 demo0: "
                  "gripper_cmd flips -1->+1 at t=32 with ee z=1.0291; minus the "
                  "table plane z=0.901 measured from my own debug-seed depth "
                  "histogram -> 0.128 m above the table",
        "allowed": True},
    "TABLE_FALLBACK": {
        "source": "debug seeds 51-65, cam_high depth histogram mode: z=0.9006 "
                  "on every seed; only used if the runtime histogram falls "
                  "outside a 0.15 m sanity window",
        "allowed": True},
    "HI_BAND": {
        "source": "debug seeds 51-65: the wine bottle stands table+0.158 tall "
                  "and no other prop exceeds table+0.059 (stove 0.031, bowl "
                  "0.051, plate/box 0.019), so a band from table+0.10 to "
                  "table+0.22 contains the bottle neck and nothing else that "
                  "passes HI_MAXDIM; the lowest visible robot-arm point is "
                  "table+0.264",
        "allowed": True},
    "HI_MAXDIM": {
        "source": "debug seeds 51-65: the bottle's neck cluster measures "
                  "0.013-0.019 across; the cabinet/rack fixture in the same "
                  "band is >0.08 wide",
        "allowed": True},
    "LO_BAND": {
        "source": "debug seeds 51-65: the bowl rim runs from table+0.036 to "
                  "table+0.051; the stove slab tops out at table+0.031 and the "
                  "plate and box at table+0.019",
        "allowed": True},
    "BOTTLE_PUNCH_R": {
        "source": "debug seeds 51-65: the bottle body is 0.043 across "
                  "(radius 0.0215), so a 0.032 m exclusion disc about its axis "
                  "clears the body with margin while removing <25% of an "
                  "abutting bowl rim",
        "allowed": True},
    "BOWL_R": {
        "source": "debug seeds 51-65: a least-squares circle fit to the bowl's "
                  "top 6 mm rim ring returns r=0.0536-0.0537 on all 15 seeds; "
                  "the only other fitted candidates are the bottle shoulder "
                  "(0.017) and the cabinet (0.078-0.091)",
        "allowed": True},
    "BOWL_FLOOR_DZ": {
        "source": "debug seeds 51/53/55/57: median depth inside the bowl rim is "
                  "z=0.908, i.e. 0.007 above the table plane",
        "allowed": True},
    "CARRY_DZ": {
        "source": "debug-seed measurement: the bowl rim tops out at table+0.051 "
                  "and the bottle hangs GRASP_DZ below the eef, so the eef must "
                  "exceed table+0.179 in transit; table+0.26 is margin",
        "allowed": True},
}

TABLE_FALLBACK = 0.9006
HI_LO, HI_HI = 0.10, 0.22
HI_MAXDIM = 0.08
HI_MINPX = 120
LO_LO, LO_HI = 0.036, 0.080
LO_MINPX = 300
BOTTLE_PUNCH_R = 0.032
BOWL_R = 0.0537
BOWL_R_TOL = 0.012
RING_DZ = 0.006
RING_MINPX = 40
BOWL_FLOOR_DZ = 0.007
GRASP_DZ = 0.128
CARRY_DZ = 0.26
XLIM = (-0.35, 0.30)
YLIM = (-0.40, 0.40)

DUMP_FRAMES = False
CHUNK = 1800


def _stream(api, tag, arr):
    raw = zlib.compress(np.ascontiguousarray(arr).tobytes(), 6)
    b64 = base64.b64encode(raw).decode()
    api.log("%s META shape=%s dtype=%s nchunks=%d" %
            (tag, list(arr.shape), str(arr.dtype), (len(b64) + CHUNK - 1) // CHUNK))
    for i in range(0, len(b64), CHUNK):
        api.log("%s D%04d %s" % (tag, i // CHUNK, b64[i:i + CHUNK]))


# --------------------------------------------------------------------------

def _label(mask):
    """4-connected labelling (no scipy dependency inside the sandbox)."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    parent = [0]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    nxt = 1
    for i in range(h):
        row = mask[i]
        if not row.any():
            continue
        for j in np.flatnonzero(row):
            up = lab[i - 1, j] if i else 0
            lf = lab[i, j - 1] if j else 0
            if up and lf:
                lab[i, j] = min(up, lf)
                ra, rb = find(up), find(lf)
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)
            elif up or lf:
                lab[i, j] = up or lf
            else:
                lab[i, j] = nxt
                parent.append(nxt)
                nxt += 1
    remap = np.zeros(nxt, np.int32)
    for k in range(1, nxt):
        remap[k] = find(k)
    return remap[lab]


def _circle(xs, ys):
    """Kasa least-squares circle fit -> (cx, cy, r)."""
    A = np.stack([xs, ys, np.ones_like(xs)], 1)
    c, *_ = np.linalg.lstsq(A, xs ** 2 + ys ** 2, rcond=None)
    cx, cy = c[0] / 2.0, c[1] / 2.0
    rr = c[2] + cx * cx + cy * cy
    return float(cx), float(cy), float(np.sqrt(rr)) if rr > 0 else -1.0


def perceive(api):
    f = api.capture("cam_high")
    dep = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = dep.shape
    vv, uu = np.mgrid[0:H, 0:W]
    good = np.isfinite(dep) & (dep > 0)
    z = np.where(good, dep, 1.0)
    P = np.stack([(uu - K[0, 2]) * z / K[0, 0],
                  (vv - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], -1) @ T.T
    X = np.where(good, P[..., 0], 1e3)
    Y = np.where(good, P[..., 1], 1e3)
    Z = np.where(good, P[..., 2], -1e3)

    box = (X > XLIM[0]) & (X < XLIM[1]) & (Y > YLIM[0]) & (Y < YLIM[1])
    table = TABLE_FALLBACK
    if int(box.sum()) > 1000:
        hist, edge = np.histogram(Z[box], bins=400)
        t = float(edge[int(np.argmax(hist))] + 0.5 * (edge[1] - edge[0]))
        if TABLE_FALLBACK - 0.15 < t < TABLE_FALLBACK + 0.15:
            table = t

    # -- band 1: the bottle, above everything else on the table ------------
    hi = box & (Z > table + HI_LO) & (Z < table + HI_HI)
    lab = _label(hi)
    bottle = None
    for i in range(1, int(lab.max()) + 1):
        s = lab == i
        if int(s.sum()) < HI_MINPX:
            continue
        dx = float(X[s].max() - X[s].min()); dy = float(Y[s].max() - Y[s].min())
        if max(dx, dy) > HI_MAXDIM:
            continue
        zt = float(Z[s].max())
        t = s & (Z > zt - 0.012)
        if int(t.sum()) < 8:
            continue
        c = {"n": int(s.sum()), "dx": dx, "dy": dy, "ztop": zt,
             "x": float((X[t].min() + X[t].max()) / 2),
             "y": float((Y[t].min() + Y[t].max()) / 2)}
        api.log("BOTCAND %s" % c)
        if bottle is None or c["ztop"] > bottle["ztop"]:
            bottle = c
    if bottle is None:
        api.log("TABLE %.4f BOTTLE None" % table)
        return table, None, None

    # -- band 2: the bowl rim, with the bottle's footprint punched out -----
    d = np.sqrt((X - bottle["x"]) ** 2 + (Y - bottle["y"]) ** 2)
    lo = (box & (Z > table + LO_LO) & (Z < table + LO_HI)
          & (d > BOTTLE_PUNCH_R))
    lab2 = _label(lo)
    bowl = None
    for i in range(1, int(lab2.max()) + 1):
        s = lab2 == i
        if int(s.sum()) < LO_MINPX:
            continue
        zt = float(Z[s].max())
        ring = s & (Z > zt - RING_DZ)
        if int(ring.sum()) < RING_MINPX:
            continue
        cx, cy, r = _circle(X[ring], Y[ring])
        c = {"n": int(s.sum()), "ring": int(ring.sum()), "ztop": zt,
             "x": cx, "y": cy, "r": r}
        api.log("BOWLCAND %s" % c)
        if abs(r - BOWL_R) > BOWL_R_TOL:
            continue
        if bowl is None or c["ring"] > bowl["ring"]:
            bowl = c
    api.log("TABLE %.4f BOTTLE %s BOWL %s" % (table, bottle, bowl))
    return table, bottle, bowl


# --------------------------------------------------------------------------

def run(api):
    api.log("INSTR %r" % api.instruction())
    table, bottle, bowl = perceive(api)
    if bottle is None or bowl is None:
        api.log("ABORT missing bottle/bowl")
        return "no target"

    bx, by = bottle["x"], bottle["y"]
    wx, wy = bowl["x"], bowl["y"]
    grasp_z = table + GRASP_DZ
    carry_z = table + CARRY_DZ
    place_z = table + BOWL_FLOOR_DZ + 0.005 + GRASP_DZ
    api.log("PLAN bottle=(%.4f,%.4f) bowl=(%.4f,%.4f) grasp_z=%.4f carry_z=%.4f "
            "place_z=%.4f" % (bx, by, wx, wy, grasp_z, carry_z, place_z))

    api.grip(0.08)
    api.log("R hover %.4f" % api.move([bx, by, carry_z], seconds=2.0))
    api.log("R descend %.4f" % api.move([bx, by, grasp_z], seconds=2.0))
    api.log("EEF@grasp %s" % np.asarray(api.eef()).round(4).tolist())
    api.grip(0.0)
    api.settle(0.4)
    api.log("GRIP_AFTER_CLOSE %s" % api.gripper())

    api.log("R lift %.4f" % api.move([bx, by, carry_z], seconds=2.0))
    api.log("GRIP_AFTER_LIFT %s EEF %s"
            % (api.gripper(), np.asarray(api.eef()).round(4).tolist()))

    api.log("R over %.4f" % api.move([wx, wy, carry_z], seconds=2.0))
    api.log("R lower %.4f" % api.move([wx, wy, place_z], seconds=2.5))
    api.log("EEF@place %s GRIP %s"
            % (np.asarray(api.eef()).round(4).tolist(), api.gripper()))

    api.grip(0.08)
    api.settle(0.6)
    api.log("R retreat %.4f" % api.move([wx, wy, carry_z], seconds=2.0))
    api.settle(1.5)

    if DUMP_FRAMES:
        fin = api.capture("cam_high")
        _stream(api, "FIN_RGB", np.asarray(fin.rgb, np.uint8))
        _stream(api, "FIN_DEP", np.asarray(fin.depth, np.float32).astype(np.float16))
    api.log("END")
    return "v3 two-band perceive, neck-grasp into bowl"
