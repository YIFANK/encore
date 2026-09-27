"""c2clean spa_bowl_next_to_plate_pos_k0 -- v3.

v2 established: perception is stable across debug seeds, and the rim pinch
(jaws along world y, centred on the rim wall, fingertips 22mm below the rim
top) grasps the bowl 4/4.  v2 failed only on the release: a bad held-object
measurement (the mask swallowed the table) put the release 15cm above the
plate.  v3 drops that measurement from the control path and derives the
release height from the grasp geometry, where the unknown fingertip offset
cancels:

    fingertip height above the bowl's base at grasp = z_grasp - off - table_z
    release fingertip z = plate_top + that + clearance
    => release eef z = plate_top + z_grasp - table_z + clearance   (off cancels)
"""
import numpy as np

PROVENANCE = {
    "WS": {"source": "debug seeds 51-57 cam_high reconstruction: table slab and "
                     "all props fall inside x[-0.35,0.35] y[-0.45,0.45]",
           "allowed": True},
    "OBJ_DZ": {"source": "debug-seed height map: table plane flat to ~2mm; 12mm "
                         "clears it and keeps the 18mm-tall plate",
               "allowed": True},
    "TIP_OFF": {"source": "debug seeds 51-57: closed jaws pressed down on bare "
                          "table stalled with EEF z = 0.9093 against a measured "
                          "table plane of 0.9016 -> fingertips sit 0.0077 below "
                          "the EEF origin (identical on all four seeds)",
                "allowed": True},
    "GRASP_DEPTH": {"source": "v3 debug seeds 51-65: fingertips landing 15.7-16.7mm "
                              "below the rim top closed to a 8.1-8.5mm gap and the "
                              "bowl did not move in the jaws (hang 0.027-0.028) on "
                              "5/5, while 24.7mm closed to 7.1-7.8mm and slipped "
                              "down to hang by the lip (hang 0.048-0.050) on 3/3. "
                              "18mm is inside the non-slip band with margin for the "
                              "+-5mm z under-convergence measured on those seeds",
                    "allowed": True},
    "HANG_CLAMP": {"source": "v3 debug seeds 51-65: measured hang of the held bowl "
                             "was 0.027-0.050; the bowl is 0.050 tall so no physical "
                             "hang can exceed it. Clamp [0.015,0.060]",
                   "allowed": True},
    "STALL_EPS": {"source": "generic controller mechanics: an OSC move that leaves "
                            "the EEF within 1mm of where it started is blocked, so "
                            "further bias correction would run the command away",
                  "allowed": True},
    "VESSEL_BAND": {"source": "debug seeds 51-57: bowls stand 0.042-0.051 above "
                              "the table, plate 0.018, cookie box 0.019",
                    "allowed": True},
    "FLAT_BAND": {"source": "same measurement: plate/box occupy 0.008-0.030",
                  "allowed": True},
    "CLASS_TOL": {"source": "debug seeds 51-57: the two matched bowls measure "
                            "0.109-0.111 across, the odd lighter vessel 0.085-0.088",
                  "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics; tool z is the approach "
                         "axis. Checked against the reported start tool_rotation",
               "allowed": True},
    "JAW_AXIS": {"source": "start tool_rotation maps tool y to world -y and Panda "
                           "jaws separate along tool y; confirmed by the v2 grasp "
                           "closing on the rim wall at a -y aim point",
                 "allowed": True},
    "PINCH_SIDE": {"source": "debug seeds 51-57 height map: the -y approach at the "
                             "plate (x[-0.02,0.19], y[-0.11,-0.02]) is bare table, "
                             "while +y is blocked by the cookie box at y>0.17",
                   "allowed": True},
    "CLEAR": {"source": "debug-seed plate profile: rim top 0.018 above table, well "
                        "floor 0.008, so a 6mm release clearance drops the bowl "
                        "less than the rim height",
              "allowed": True},
}

WS = (-0.35, 0.35, -0.45, 0.45)
OBJ_DZ = 0.012
TIP_OFF = 0.0077
GRASP_DEPTH = 0.018
GEOM_HANG = 0.050 - GRASP_DEPTH   # bowl height minus the grip depth below its rim
HANG_LO, HANG_HI = 0.015, 0.060
STALL_EPS = 0.001
CLEAR = 0.006
CLASS_TOL = 0.015
CELL = 0.004
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
SIDE = -1.0                      # pinch / release on the -y arc


# --------------------------------------------------------------- perception
def cloud(f):
    d = np.asarray(f.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    V = np.isfinite(d) & (d > 0)
    z = np.where(V, d, 1.0)
    pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], -1)
    P = pc @ T[:3, :3].T + T[:3, 3]
    return P[..., 0], P[..., 1], P[..., 2], V


def _label(occ):
    idx = {}
    pts = np.argwhere(occ)
    for k, (r, c) in enumerate(pts):
        idx[(int(r), int(c))] = k
    par = list(range(len(pts)))

    def find(a):
        while par[a] != a:
            par[a] = par[par[a]]
            a = par[a]
        return a

    for (r, c), k in idx.items():
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                j = idx.get((r + dr, c + dc))
                if j is not None:
                    ra, rb = find(k), find(j)
                    if ra != rb:
                        par[ra] = rb
    groups = {}
    for (r, c), k in idx.items():
        groups.setdefault(find(k), []).append((r, c))
    return groups


def segment(api, f, tag=""):
    X, Y, Z, V = cloud(f)
    m = V & (X > WS[0]) & (X < WS[1]) & (Y > WS[2]) & (Y < WS[3])
    h, e = np.histogram(Z[m], bins=300)
    i = int(h.argmax())
    tz = float((e[i] + e[i + 1]) / 2)
    sel = m & (Z > tz + OBJ_DZ)
    gx = np.floor((X[sel] - WS[0]) / CELL).astype(int)
    gy = np.floor((Y[sel] - WS[2]) / CELL).astype(int)
    zz, rgb = Z[sel], np.asarray(f.rgb, float)[sel]
    nx = int((WS[1] - WS[0]) / CELL) + 1
    ny = int((WS[3] - WS[2]) / CELL) + 1
    top = np.full((nx, ny), -1.0)
    np.maximum.at(top, (gx, gy), zz)
    objs = []
    for cells in _label(top > 0).values():
        if len(cells) < 8:
            continue
        cm = np.zeros((nx, ny), bool)
        for r, c in cells:
            cm[r, c] = True
        pix = cm[gx, gy]
        if pix.sum() < 40:
            continue
        o = {"n": int(pix.sum()), "top": float(zz[pix].max()),
             "rgb": rgb[pix].mean(0).round(1).tolist()}
        o["h"] = o["top"] - tz
        band = pix & (zz > o["top"] - 0.008)
        px, py = (X[sel][band], Y[sel][band]) if band.sum() >= 12 else \
                 (X[sel][pix], Y[sel][pix])
        o["x"] = (float(px.min()), float(px.max()))
        o["y"] = (float(py.min()), float(py.max()))
        o["dx"] = o["x"][1] - o["x"][0]
        o["dy"] = o["y"][1] - o["y"][0]
        o["c"] = ((o["x"][0] + o["x"][1]) / 2, (o["y"][0] + o["y"][1]) / 2)
        o["d"] = (o["dx"] + o["dy"]) / 2
        objs.append(o)
    objs.sort(key=lambda o: -o["n"])
    api.log("SEG%s table=%.4f n=%d" % (tag, tz, len(objs)))
    for o in objs:
        api.log("  OBJ n=%d c=(%.3f,%.3f) d=(%.3f,%.3f) h=%.3f top=%.3f rgb=%s"
                % (o["n"], o["c"][0], o["c"][1], o["dx"], o["dy"], o["h"],
                   o["top"], o["rgb"]))
    return tz, objs


def pick_targets(api, objs):
    flats = [o for o in objs if 0.008 < o["h"] < 0.030
             and 0.06 < o["dx"] < 0.22 and 0.06 < o["dy"] < 0.22]
    plate, best = None, -1e9
    for o in flats:
        ar = max(o["dx"], o["dy"]) / max(1e-6, min(o["dx"], o["dy"]))
        if ar < 1.25 and min(o["dx"], o["dy"]) > best:
            best, plate = min(o["dx"], o["dy"]), o
    vessels = [o for o in objs if 0.030 < o["h"] < 0.080
               and 0.05 < o["dx"] < 0.16 and 0.05 < o["dy"] < 0.16]
    if plate is None or not vessels:
        api.log("IDENT FAIL plate=%s nvessel=%d" % (plate is not None, len(vessels)))
        return None, None
    api.log("PLATE c=(%.3f,%.3f) d=(%.3f,%.3f) top=%.3f"
            % (plate["c"][0], plate["c"][1], plate["dx"], plate["dy"], plate["top"]))
    # "the black bowl" is the repeated asset: take the largest group of
    # same-diameter vessels, tie-broken toward the darker group.
    groups = []
    for v in vessels:
        for g in groups:
            if abs(g[0]["d"] - v["d"]) < CLASS_TOL:
                g.append(v)
                break
        else:
            groups.append([v])
    groups.sort(key=lambda g: (-len(g), float(np.mean([sum(o["rgb"]) for o in g]))))
    cls = groups[0]
    api.log("CLASS n=%d d=%.3f rgb=%.1f  (groups %s)"
            % (len(cls), cls[0]["d"], float(np.mean([sum(o["rgb"]) for o in cls])),
               [(len(g), round(g[0]["d"], 3)) for g in groups]))

    def dist(o):
        return float(np.hypot(o["c"][0] - plate["c"][0], o["c"][1] - plate["c"][1]))

    cls.sort(key=dist)
    for v in cls:
        api.log("CAND c=(%.3f,%.3f) d=%.3f dist=%.3f rgb=%s"
                % (v["c"][0], v["c"][1], v["d"], dist(v), v["rgb"]))
    return plate, cls[0]


# ------------------------------------------------------------------ motion
def goto(api, xyz, seconds=2.0, tag=""):
    xyz = np.asarray(xyz, float)
    r = api.move(xyz, rotation=R_DOWN, seconds=seconds)
    e = api.eef()
    api.log("MOVE%-10s cmd=(%.3f,%.3f,%.3f) got=(%.3f,%.3f,%.3f) res=%.4f"
            % (tag, xyz[0], xyz[1], xyz[2], e[0], e[1], e[2], r))
    return e


def goto_xy(api, xyz, seconds=2.0, tag="", tol=0.003, tries=2):
    """Move, then cancel the standing xy tracking bias.

    Bounded: if an iteration fails to move the EEF (blocked by contact) the
    loop stops instead of walking the command away from the target.
    """
    tgt = np.asarray(xyz, float)
    e = goto(api, tgt, seconds, tag)
    cmd = tgt.copy()
    for k in range(tries):
        err = e[:2] - tgt[:2]
        if float(np.linalg.norm(err)) <= tol:
            break
        cmd[:2] = cmd[:2] - err
        prev = e
        e = goto(api, cmd, seconds, tag + "-fix%d" % k)
        if float(np.linalg.norm(e - prev)) < STALL_EPS:
            api.log("XYFIX%s blocked, stopping" % tag)
            break
    api.log("XYERR%s %.4f" % (tag, float(np.linalg.norm(e[:2] - tgt[:2]))))
    return e


def held_probe(api, tag):
    """Points hanging below the fingertips, table excluded by a tight band."""
    e = api.eef()
    tipz = float(e[2]) - TIP_OFF
    f = api.capture("cam_high")
    X, Y, Z, V = cloud(f)
    m = (V & (np.hypot(X - e[0], Y - e[1]) < 0.085)
         & (Z < tipz - 0.002) & (Z > tipz - 0.075))
    if m.sum() < 25:
        api.log("HELD%s none n=%d tipz=%.3f" % (tag, int(m.sum()), tipz))
        return None
    o = {"n": int(m.sum()), "zmin": float(Z[m].min()),
         "cx": float((X[m].min() + X[m].max()) / 2),
         "cy": float((Y[m].min() + Y[m].max()) / 2),
         "dx": float(X[m].max() - X[m].min()),
         "dy": float(Y[m].max() - Y[m].min())}
    o["hang"] = tipz - o["zmin"]
    api.log("HELD%s n=%d c=(%.3f,%.3f) d=(%.3f,%.3f) zmin=%.3f hang=%.3f tipz=%.3f"
            % (tag, o["n"], o["cx"], o["cy"], o["dx"], o["dy"], o["zmin"],
               o["hang"], tipz))
    return o


def g(api, tag):
    s = api.gripper()
    api.log("GRIP%-10s w=%.4f e=%.2f" % (tag, s["width_m"], s["effort"]))
    return s


# -------------------------------------------------------------------- main
def attempt(api, tz, plate, bowl, tag):
    r = bowl["d"] / 2
    gx, gy = bowl["c"][0], bowl["c"][1] + SIDE * r
    z_grasp = bowl["top"] - GRASP_DEPTH + TIP_OFF
    z_up = bowl["top"] + 0.12 + TIP_OFF
    api.log("PLAN%s bowl=(%.3f,%.3f) r=%.3f top=%.3f grasp=(%.3f,%.3f,%.3f)"
            % (tag, bowl["c"][0], bowl["c"][1], r, bowl["top"], gx, gy, z_grasp))

    api.grip(0.08)
    goto(api, (gx, gy, z_up), 2.5, tag + "hover")
    # single descent: the rim pinch is insensitive to x (that error only slides
    # the aim along the rim arc) and the per-move y error is <=3mm, while extra
    # corrective moves sink the wrist another ~8mm into the slip band.
    goto(api, (gx, gy, z_grasp), 2.5, tag + "descend")
    g(api, tag + "pre")
    api.grip(0.0)
    api.settle(0.5)
    s = g(api, tag + "close")
    goto(api, (gx, gy, z_up), 2.5, tag + "lift")
    s = g(api, tag + "lifted")
    if s["width_m"] < 0.0015:
        api.log("GRASP MISS%s (jaws empty)" % tag)
        return False
    held = held_probe(api, tag + "lift")

    # release height from the MEASURED hang of the held bowl: whatever height
    # the bowl settled to in the jaws, its base must clear the plate top.
    hang = GEOM_HANG
    if held is not None and HANG_LO <= held["hang"] <= HANG_HI:
        hang = held["hang"]
    z_rel = plate["top"] + hang + TIP_OFF + CLEAR
    px, py = plate["c"][0], plate["c"][1] + SIDE * r
    api.log("PLACE%s eef=(%.3f,%.3f,%.3f) hang=%.3f plate_top=%.3f"
            % (tag, px, py, z_rel, hang, plate["top"]))
    goto(api, (px, py, z_up), 3.0, tag + "carry")
    g(api, tag + "carried")
    held_probe(api, tag + "carry")
    goto_xy(api, (px, py, z_rel), 2.5, tag + "lower")
    g(api, tag + "prerel")
    api.grip(0.08)
    api.settle(0.6)
    goto(api, (px, py, z_rel + 0.10), 2.5, tag + "up")
    goto(api, (px - 0.06, py - 0.10, z_rel + 0.16), 2.5, tag + "clear")
    api.settle(0.5)
    return True


def on_plate(api, plate, bowl_d, tag):
    """Re-perceive: is a vessel of the target class sitting on the plate?"""
    tz, objs = segment(api, api.capture("cam_high"), tag)
    best = None
    for o in objs:
        if not (0.030 < o["h"] < 0.090 and abs(o["d"] - bowl_d) < CLASS_TOL):
            continue
        dd = float(np.hypot(o["c"][0] - plate["c"][0], o["c"][1] - plate["c"][1]))
        if best is None or dd < best[0]:
            best = (dd, o)
    if best is None:
        api.log("VERIFY%s no vessel found" % tag)
        return False, None, tz, objs
    dd, o = best
    lifted = o["top"] - tz > 0.055
    api.log("VERIFY%s nearest vessel c=(%.3f,%.3f) dist=%.3f h=%.3f raised=%s"
            % (tag, o["c"][0], o["c"][1], dd, o["h"], lifted))
    return (dd < 0.045), o, tz, objs


def run(api):
    api.log("INSTR %r" % api.instruction())
    tz, objs = segment(api, api.capture("cam_high"), " init")
    plate, bowl = pick_targets(api, objs)
    if plate is None:
        api.log("ABORT: identification failed")
        return
    bowl_d = bowl["d"]
    if not attempt(api, tz, plate, bowl, " a1-"):
        api.log("attempt 1 lost the grasp")
    ok, o, tz2, objs2 = on_plate(api, plate, bowl_d, " v1")
    if ok:
        api.log("DONE ok after attempt 1")
        return

    # retry once, re-perceiving wherever the bowl actually ended up
    if o is None:
        api.log("DONE no retry target")
        return
    plate2, _ = pick_targets(api, objs2)
    plate2 = plate2 or plate
    api.log("RETRY on vessel c=(%.3f,%.3f) h=%.3f" % (o["c"][0], o["c"][1], o["h"]))
    attempt(api, tz2, plate2, o, " a2-")
    on_plate(api, plate2, bowl_d, " v2")
    api.log("DONE after retry")
