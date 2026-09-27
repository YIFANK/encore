"""c2k1clean / spa_bowl_next_to_plate_task_k1 -- v1

Intent: "Pick the akita black bowl next to the ramekin and place it on the
plate."

Perception (cam_high RGB-D only): deproject every pixel into the base frame,
keep the workspace slab above the table plane, connected-component it, and
read each blob's top-ring (the rim).  Rim diameter separates the three
vessels: the two akita bowls measure ~0.110 m across and stand ~0.052 m tall;
the ramekin measures ~0.087 m across and stands ~0.044 m; the plate is
~0.136 m across but only ~0.020 m tall.  The intent's object is then the
bowl whose centre is nearest the ramekin; the goal is the plate.

Grasp: the bowl is a thin-walled flare (wall ~6 mm thick), far wider than the
jaws can span, so the gripper pinches the wall -- one jaw in the cavity, one
outside -- on the +y arc (the -y arc puts the outer jaw within a centimetre
of the ramekin).  Jaws close along base y with the wrist left straight down.
"""
import os
from collections import deque

import numpy as np

PROVENANCE = {
    "TABLE_Z": {
        "source": "debug-seed 51/53/57/61 cam_high depth: base-frame z "
                  "histogram of the workspace has a 101k-pixel mode at 0.900",
        "allowed": True},
    "SLAB_Z0": {
        "source": "debug-seed depth: TABLE_Z + 6 mm, the smallest offset that "
                  "separates objects from table noise",
        "allowed": True},
    "SLAB_Z1": {
        "source": "debug-seed depth: everything on the table tops out below "
                  "0.99; the cabinet/stove/arm clip at 1.05",
        "allowed": True},
    "WS_X": {"source": "debug-seed depth: base-x extent of the vessel/plate "
                       "region of cam_high", "allowed": True},
    "WS_Y": {"source": "debug-seed depth: base-y window that excludes the "
                       "cabinet, stove and robot base (all y < 0)",
             "allowed": True},
    "RIM_BAND": {"source": "debug-seed depth: 10 mm top band of a vessel blob "
                           "spans its full rim ring (x-span == y-span)",
                 "allowed": True},
    "BOWL_MIN_D": {
        "source": "debug-seed 51/53/57/61 rim spans: akita bowls 0.109-0.111, "
                  "ramekin 0.085-0.088 -> split at 0.098",
        "allowed": True},
    "VESSEL_MIN_H": {
        "source": "debug-seed rim heights: bowls z=0.952, ramekin z=0.944, "
                  "plate and cookie box z=0.920 -> split at 0.930",
        "allowed": True},
    "BOWL_MIN_ZTOP": {
        "source": "debug-seed rim heights: bowl 0.952 vs ramekin 0.944",
        "allowed": True},
    "GRASP_DROP": {
        "source": "pack keyframes: gripper closes at ee z 0.9225 (mate) / "
                  "0.929 (k1); debug-seed bowl rim top is 0.952 -> 27 mm below "
                  "the rim",
        "allowed": True},
    "GRASP_R": {
        "source": "debug-seed radial profile of a bowl blob: at z 0.922-0.927 "
                  "the wall annulus runs r 0.0397-0.0476, midline ~0.043",
        "allowed": True},
    "BOWL_BASE_DROP": {
        "source": "debug-seed: bowl base sits on TABLE_Z while the pack's "
                  "grasp ee z is 0.0225 above it",
        "allowed": True},
    "PLATE_TOP": {
        "source": "debug-seed depth: plate blob top ring at z 0.920",
        "allowed": True},
    "PLACE_CLEAR": {
        "source": "debug-seed: 5 mm release clearance above the plate top",
        "allowed": True},
    "HOVER_Z": {
        "source": "pack ee_path transport altitude (k1 t=80 z=1.05, mate t=80 "
                  "z=1.07) rounded down to clear the 0.952 rims",
        "allowed": True},
    "OPEN_W": {"source": "debug-seed api.gripper() at reset: width_m 0.0778",
               "allowed": True},
    "HELD_W_MAX": {
        "source": "pack keyframes: gripper_state while carrying is 0.0068 "
                  "(k1) / 0.0115 (mate) total width",
        "allowed": True},
    "HELD_EFFORT": {
        "source": "FairApi docs: effort 3.0 iff holding; debug-seed idle "
                  "effort 0.05",
        "allowed": True},
    "HOME_XYZ": {"source": "debug-seed api.eef() at reset: (-0.2085, 0, 1.173)",
                 "allowed": True},
    "ROUND_TOL": {
        "source": "debug seeds 51-65: an unfused bowl rim measures "
                  "sx-sy <= 0.004; a blob fused with its neighbour cannot",
        "allowed": True},
    "TOP_BAND": {
        "source": "debug-seed rim heights: bowl rim 0.952 sits 8 mm above the "
                  "ramekin rim 0.944, so a 6 mm top band keeps only the bowl",
        "allowed": True},
}

TABLE_Z = 0.900
SLAB_Z0 = 0.906
SLAB_Z1 = 0.990
WS_X = (-0.40, 0.30)
WS_Y = (0.08, 0.50)
RIM_BAND = 0.010
BOWL_MIN_D = 0.098
VESSEL_MIN_H = 0.930
BOWL_MIN_ZTOP = 0.948
GRASP_DROP = 0.027
GRASP_R = 0.043
BOWL_BASE_DROP = 0.0225
PLATE_TOP = 0.920
PLACE_CLEAR = 0.005
HOVER_Z = 1.03
OPEN_W = 0.08
HELD_W_MAX = 0.025
HELD_EFFORT = 1.0
HOME_XYZ = (-0.2085, 0.0, 1.17)
ROUND_TOL = 0.025
TOP_BAND = 0.006

DUMP = os.environ.get("FAIR_DUMP", "0") == "1"
DUMP_DIR = ("/mnt/data/YifanKang/Heron/packs/"
            "c2k1clean_spa_bowl_next_to_plate_task_k1/obs")


# ---------------------------------------------------------------- perception

def _cloud(frame):
    d = np.asarray(frame.depth, dtype=np.float64)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    ok = np.isfinite(d) & (d > 0)
    dd = np.where(ok, d, 1.0)
    pts = np.stack([(uu - cx) * dd / fx, (vv - cy) * dd / fy, dd,
                    np.ones_like(dd)], -1) @ T.T
    return pts[..., 0], pts[..., 1], pts[..., 2], ok


def _blobs(mask, min_px):
    h, w = mask.shape
    seen = np.zeros((h, w), bool)
    out = []
    for y0, x0 in np.argwhere(mask):
        if seen[y0, x0]:
            continue
        q = deque([(y0, x0)])
        seen[y0, x0] = True
        comp = []
        while q:
            y, x = q.popleft()
            comp.append((y, x))
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    q.append((ny, nx))
        if len(comp) >= min_px:
            out.append(np.array(comp))
    return out


def perceive(api, tag=""):
    """-> list of blob dicts, best-first by pixel count."""
    f = api.capture("cam_high")
    bx, by, bz, ok = _cloud(f)
    m = (ok & (bx > WS_X[0]) & (bx < WS_X[1]) & (by > WS_Y[0]) & (by < WS_Y[1])
         & (bz > SLAB_Z0) & (bz < SLAB_Z1))
    objs = []
    for comp in _blobs(m, 300):
        ys, xs = comp[:, 0], comp[:, 1]
        X, Y, Z = bx[ys, xs], by[ys, xs], bz[ys, xs]
        ztop = float(Z.max())
        rim = Z > ztop - RIM_BAND
        xr, yr = X[rim], Y[rim]
        sx, sy = float(xr.max() - xr.min()), float(yr.max() - yr.min())
        objs.append({
            "n": int(len(comp)),
            "ztop": ztop,
            "cx": float((xr.min() + xr.max()) / 2.0),
            "cy": float((yr.min() + yr.max()) / 2.0),
            "d": (sx + sy) / 2.0,
            "sx": sx, "sy": sy,
            "u": (int(xs.min()), int(xs.max())),
            "v": (int(ys.min()), int(ys.max())),
            "clip": bool(xs.max() >= bz.shape[1] - 1 or xs.min() <= 0),
            "X": X, "Y": Y, "Z": Z,
        })
    objs.sort(key=lambda o: -o["n"])
    for o in objs:
        api.log("%sblob n=%d ztop=%.3f c=(%.3f,%.3f) d=%.3f s=(%.3f,%.3f) "
                "u=%s v=%s clip=%s" % (tag, o["n"], o["ztop"], o["cx"], o["cy"],
                                       o["d"], o["sx"], o["sy"], o["u"],
                                       o["v"], o["clip"]))
    if DUMP:
        try:
            os.makedirs(DUMP_DIR, exist_ok=True)
            np.savez_compressed(
                os.path.join(DUMP_DIR, "%s%s_cam_high.npz"
                             % (os.path.basename(os.getcwd()), tag or "_t0")),
                rgb=f.rgb, depth=np.asarray(f.depth, np.float32),
                K=np.asarray(f.intrinsics, float),
                T=np.asarray(f.t_base_cam, float))
        except Exception as e:
            api.log("dump failed %r" % (e,))
    return objs


def _unfuse(api, o):
    """A blob whose rim ring is not round has swallowed its neighbour.  The
    bowl rim stands 8 mm proud of the ramekin rim, so the top 6 mm of the blob
    belongs to the bowl alone; re-centre on that."""
    if abs(o["sx"] - o["sy"]) <= ROUND_TOL:
        return o
    top = o["Z"] > o["ztop"] - TOP_BAND
    if int(top.sum()) < 100:
        return o
    xr, yr = o["X"][top], o["Y"][top]
    new = dict(o)
    new["cx"] = float((xr.min() + xr.max()) / 2.0)
    new["cy"] = float((yr.min() + yr.max()) / 2.0)
    new["sx"] = float(xr.max() - xr.min())
    new["sy"] = float(yr.max() - yr.min())
    api.log("UNFUSE s=(%.3f,%.3f)->(%.3f,%.3f) c=(%.3f,%.3f)->(%.3f,%.3f)"
            % (o["sx"], o["sy"], new["sx"], new["sy"], o["cx"], o["cy"],
               new["cx"], new["cy"]))
    return new


def identify(api, objs):
    """-> (bowl, plate) picks for this intent, or (None, None).

    Height, not width, separates the classes: bowl rims sit at 0.952, the
    ramekin at 0.944, the plate at 0.920.  Width is unreliable for a blob the
    image edge has clipped, and bowl B is clipped or off-frame on many seeds.
    """
    vessels = [o for o in objs if o["ztop"] >= VESSEL_MIN_H]
    bowls = [o for o in vessels if o["ztop"] >= BOWL_MIN_ZTOP]
    rams = [o for o in vessels
            if o["ztop"] < BOWL_MIN_ZTOP and not o["clip"]
            and o["d"] < BOWL_MIN_D]
    flats = [o for o in objs
             if o["ztop"] < VESSEL_MIN_H and o["d"] > BOWL_MIN_D]
    plate = max(flats, key=lambda o: o["d"]) if flats else None
    if not bowls or plate is None:
        return (bowls[0] if bowls else None), plate
    if rams:
        ram = max(rams, key=lambda o: o["n"])
        bowl = min(bowls, key=lambda o: np.hypot(o["cx"] - ram["cx"],
                                                 o["cy"] - ram["cy"]))
        api.log("ramekin c=(%.3f,%.3f) d=%.3f | bowl-next-to-it c=(%.3f,%.3f) "
                "dist=%.3f of %d bowls"
                % (ram["cx"], ram["cy"], ram["d"], bowl["cx"], bowl["cy"],
                   float(np.hypot(bowl["cx"] - ram["cx"],
                                  bowl["cy"] - ram["cy"])), len(bowls)))
    else:
        # No separate ramekin blob: it has fused with the bowl beside it, so
        # the intent's bowl is the one FARTHEST from the plate.
        bowl = max(bowls, key=lambda o: np.hypot(o["cx"] - plate["cx"],
                                                 o["cy"] - plate["cy"]))
        api.log("no ramekin blob; taking bowl farthest from the plate "
                "c=(%.3f,%.3f) of %d" % (bowl["cx"], bowl["cy"], len(bowls)))
    return _unfuse(api, bowl), plate


# ------------------------------------------------------------------- acting

def held(api):
    g = api.gripper()
    return (g["width_m"] <= HELD_W_MAX and g["effort"] >= HELD_EFFORT), g


def run(api):
    api.log("instruction=%r eef=%s" % (api.instruction(),
                                       np.round(api.eef(), 4).tolist()))
    objs = perceive(api)
    bowl, plate = identify(api, objs)
    if bowl is None or plate is None:
        api.log("ABORT bowl=%s plate=%s" % (bowl is not None, plate is not None))
        return "no target"

    grasp_z = bowl["ztop"] - GRASP_DROP
    place_z = PLATE_TOP + BOWL_BASE_DROP + PLACE_CLEAR

    api.grip(OPEN_W)
    api.settle(0.2)

    ok = False
    for side in (+1.0, -1.0):
        gx, gy = bowl["cx"], bowl["cy"] + side * GRASP_R
        api.log("attempt side=%+.0f grasp=(%.3f,%.3f,%.3f)"
                % (side, gx, gy, grasp_z))
        api.grip(OPEN_W)
        r = api.move([gx, gy, HOVER_Z], seconds=2.5)
        api.log("  hover res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
        r = api.move([gx, gy, grasp_z], seconds=2.0)
        api.log("  down res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
        api.grip(0.0)
        api.settle(0.4)
        ok, g = held(api)
        api.log("  closed w=%.4f eff=%.2f held=%s" % (g["width_m"], g["effort"], ok))
        r = api.move([gx, gy, HOVER_Z], seconds=2.0)
        ok, g = held(api)
        api.log("  lift res=%.4f w=%.4f eff=%.2f held=%s eef=%s"
                % (r, g["width_m"], g["effort"], ok,
                   np.round(api.eef(), 4).tolist()))
        if ok:
            break
        api.grip(OPEN_W)
        api.settle(0.2)
        api.move([gx, gy, HOVER_Z], seconds=1.5)

    if not ok:
        api.log("grasp failed on both sides")
        api.move(list(HOME_XYZ), seconds=2.5)
        return "no grasp"

    # the bowl hangs at (eef_x, eef_y - side*GRASP_R); centre it on the plate
    side_used = side
    tx, ty = plate["cx"], plate["cy"] + side_used * GRASP_R
    api.log("place over plate c=(%.3f,%.3f) -> eef (%.3f,%.3f,%.3f)"
            % (plate["cx"], plate["cy"], tx, ty, place_z))
    r = api.move([tx, ty, HOVER_Z], seconds=3.0)
    ok, g = held(api)
    api.log("  transit res=%.4f w=%.4f eff=%.2f held=%s" % (r, g["width_m"],
                                                            g["effort"], ok))
    r = api.move([tx, ty, place_z], seconds=2.0)
    api.log("  descend res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(OPEN_W)
    api.settle(0.5)
    api.move([tx, ty, HOVER_Z], seconds=2.0)
    api.move(list(HOME_XYZ), seconds=2.5)
    api.settle(0.5)
    perceive(api, tag="_end")
    return "placed side=%+.0f" % side_used
