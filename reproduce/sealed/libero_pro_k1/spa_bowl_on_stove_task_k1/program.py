"""c2k1clean / spa_bowl_on_stove_task_k1 -- v6.

Intent: "Pick the akita black bowl on the top of the cabinet and place it on
the plate."

Mechanism
---------
Several look-alike bowls stand in the scene; the intent names the one on the
cabinet, which is the only prop resting on a raised shelf.  The target is
therefore found geometrically, never by appearance:

  * the modal height of the workspace cloud is the table;
  * the modal height well above it is the cabinet's top face;
  * the one sizeable cluster standing proud of that face is the bowl;
  * a Kasa circle fit on its rim ring gives centre and outer radius;
  * the plate is the widest low, flat cluster on the table.

The jaws cannot span the bowl (rim diameter ~0.108 m against a ~0.078 m jaw
opening), so the grasp is a rim pinch: the tool descends one rim radius behind
the bowl centre -- one finger inside the bowl, one outside -- and closes on
the wall.  Pinching the +x, +y or -y side of the rim never grips (debug
seeds); only the -x side does.

Two consequences of pinching the rim drive the placement:

  1. The bowl does not hang under the tool centre.  Its centre sits about one
     rim radius ahead of the eef, so releasing with the eef over the plate
     centre lands the bowl a rim radius short of the plate -- which is exactly
     how v1 failed.  Carrying the nominal grasp offset into the release point
     (v3) still left a fixed 0.021 m / 0.022 m miss, measured on debug seeds
     by Kasa-fitting the rim of the bowl where it came to rest: the wall does
     not settle at the tangent point the geometry predicts but a few tens of
     degrees around the rim, the same way every time.  PLACE_BIAS cancels that
     measured residual.  It matters because the plate is a dish: its floor
     only reaches r=0.045 before the rim starts to climb, so a bowl set down
     2 cm off centre perches on the rim instead of seating (debug seeds put
     its base at 0.923, the rim height, rather than 0.908, the floor).
  2. The carry height is measured, not assumed.  The bowl's underside rests on
     the shelf, so the drop from pinch point to bowl base is (eef z at closing
     - shelf z), read from api.eef() rather than from the commanded z, and the
     release height is the plate floor plus that same drop.

The episode's motion budget is finite -- the episode ends after 1000 sim
steps, about 26 commands -- so the trajectory is a fixed, short sequence: no
search, no re-grasp.  It is also bounded from inside.  On a seed where the
descent is obstructed the arm ends up in a strained pose and *every*
subsequent move falls short of its tolerance; a fixed 3-try ladder then
retries all of them and the episode dies mid-transport with the bowl in the
air (seeds 56 and 64 under v5).  So goto() stops retrying a move that is not
making progress, and a whole-episode command counter reserves enough of the
budget to always finish the release.  Success is never queried; the only feedback
used is perception, eef pose and gripper width.
"""
import numpy as np

PROVENANCE = {
    "WS_X": {"source": "debug seeds 51-65 cam_high clouds: base-frame x span "
                       "holding the table and its props (back wall and robot "
                       "base fall outside)", "allowed": True},
    "WS_Y": {"source": "debug seeds 51-65 cam_high clouds: base-frame y "
                       "half-span of the table", "allowed": True},
    "ARM_KEEPOUT": {"source": "debug seeds 51-65: xy radius about api.eef() "
                              "filled by the arm's own geometry in cam_high",
                    "allowed": True},
    "SHELF_BAND": {"source": "debug seeds 51-65 z histograms: the cabinet top "
                             "face lies ~0.22 m above the table modal plane",
                   "allowed": True},
    "HIGH_BAND": {"source": "debug seeds 51-65: clearance above the shelf "
                            "plane that isolates props standing on it",
                  "allowed": True},
    "MIN_PROP_PX": {"source": "debug seeds 51-65: the shelf bowl subtends "
                              "3300-4200 px above the shelf plane",
                    "allowed": True},
    "PLATE_MIN_D": {"source": "debug seeds 51-65: the plate measures ~0.136 m "
                              "across, every other table prop <0.09 m",
                    "allowed": True},
    "DZ_GRASP": {"source": "mate pack (c2k1clean_spa_bowl_on_stove_task_mate) "
                           "keyframe t=50 closes the gripper at ee z=1.159 on "
                           "a shelf-borne bowl whose rim top measures 1.180 in "
                           "debug seeds -> pinch 0.021 m below the rim",
                 "allowed": True},
    "RIM_INSET": {"source": "debug seeds 51-65 z-sliced rim radii: the wall "
                            "tapers from r=0.054 at the rim top to r=0.045 at "
                            "the pinch depth", "allowed": True},
    "GRASP_AXIS": {"source": "debug seeds 51-65 grasp trials over all four rim "
                             "sides: only the -x side grips", "allowed": True},
    "OPEN_W": {"source": "api.gripper() width_m at episode start (0.0778)",
               "allowed": True},
    "HOVER": {"source": "debug seeds 51-65: approach height above the rim that "
                        "clears the open jaws", "allowed": True},
    "CARRY_Z": {"source": "debug seeds 51-65: shelf top 1.125 plus bowl height "
                          "0.055; the transport clears both", "allowed": True},
    "PLACE_BIAS": {"source": "debug seeds 53/63/65 under v3: the released bowl "
                             "came to rest at plate+(-0.021,+0.022) (Kasa fit "
                             "on its rim ring in cam_high); this cancels it",
                   "allowed": True},
    "PLACE_GAP": {"source": "debug seeds 51-65: release clearance above the "
                            "plate floor", "allowed": True},
    "MAX_MOVES": {"source": "debug seeds 51-65: episodes end after 1000 sim "
                            "steps ~ 26 api.move commands; the fixed sequence "
                            "needs 8, so retries are capped well inside it",
                  "allowed": True},
    "MIN_GAIN": {"source": "debug seeds 51-65: a retry against an obstruction "
                           "buys under this, a retry against controller droop "
                           "buys ~0.010", "allowed": True},
    "REACH_TOL": {"source": "debug seeds 51-65: api.move settles ~0.010 m "
                            "short; this is the residual at which goto() stops "
                            "re-commanding, and the shortfall is carried into "
                            "the grasp depth by measuring api.eef()",
                  "allowed": True},
}

WS_X = (-0.45, 0.40)
WS_Y = 0.70
ARM_KEEPOUT = 0.22
SHELF_BAND = (0.10, 0.40)
HIGH_BAND = 0.015
MIN_PROP_PX = 800
PLATE_MIN_D = 0.10
DZ_GRASP = 0.021
RIM_INSET = 0.009
GRASP_AXIS = "x"
GRASP_SIGN = -1.0
OPEN_W = 0.078
HOVER = 0.09
CARRY_Z = 1.27
PLACE_GAP = 0.004
PLACE_BIAS = (0.021, -0.022)
REACH_TOL = 0.012
MAX_MOVES = 15            # whole-episode command budget (horizon is ~26)
MIN_GAIN = 0.003          # a retry that buys less than this is not working
_MOVES = [0]


# --------------------------------------------------------------- perception
def _label(mask):
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    parent = [0]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    nxt = 1
    for v in range(h):
        for u in np.nonzero(mask[v])[0]:
            up = lab[v - 1, u] if v else 0
            lf = lab[v, u - 1] if u else 0
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
    out = np.zeros_like(lab)
    remap = {}
    for i in range(1, nxt):
        r = find(i)
        if r not in remap:
            remap[r] = len(remap) + 1
    for i in range(1, nxt):
        out[lab == i] = remap[find(i)]
    return out, len(remap)


def _cloud(frame):
    dep = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = dep.shape
    v, u = np.mgrid[0:h, 0:w]
    z = np.where(np.isfinite(dep) & (dep > 0.05), dep, np.nan)
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    p = np.stack([x, y, z, np.ones_like(z)], -1)
    return (p @ T.T)[..., :3]


def _clusters(mask, X, Y, Z, min_px):
    lab, n = _label(mask)
    out = []
    for i in range(1, n + 1):
        idx = lab == i
        if int(idx.sum()) < min_px:
            continue
        x, y, z = X[idx], Y[idx], Z[idx]
        out.append(dict(n=int(idx.sum()), idx=idx,
                        xc=float((x.max() + x.min()) / 2),
                        yc=float((y.max() + y.min()) / 2),
                        dx=float(x.max() - x.min()), dy=float(y.max() - y.min()),
                        zmax=float(z.max())))
    return out


def _kasa(x, y):
    A = np.c_[x, y, np.ones(len(x))]
    b = x ** 2 + y ** 2
    c = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = c[0] / 2.0, c[1] / 2.0
    return float(cx), float(cy), float(np.sqrt(max(c[2] + cx ** 2 + cy ** 2, 1e-9)))


def look(api, want_plate=True):
    """One cam_high reading of the scene, the arm's own geometry masked out."""
    f = api.capture("cam_high")
    B = _cloud(f)
    X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
    e = np.asarray(api.eef(), float)
    rad = np.sqrt((X - e[0]) ** 2 + (Y - e[1]) ** 2)
    ws = (np.isfinite(Z) & (X > WS_X[0]) & (X < WS_X[1]) & (np.abs(Y) < WS_Y)
          & (rad > ARM_KEEPOUT))

    hist, edges = np.histogram(Z[ws], bins=np.arange(0.70, 1.50, 0.005))
    z_table = float(edges[int(np.argmax(hist))]) + 0.0025
    up = ws & (Z > z_table + SHELF_BAND[0]) & (Z < z_table + SHELF_BAND[1])
    h2, e2 = np.histogram(Z[up], bins=np.arange(z_table + SHELF_BAND[0],
                                                z_table + SHELF_BAND[1], 0.005))
    z_shelf = float(e2[int(np.argmax(h2))]) + 0.0025

    s = dict(z_table=z_table, z_shelf=z_shelf, bowl=None, plate=None)
    cl = _clusters(ws & (Z > z_shelf + HIGH_BAND), X, Y, Z, MIN_PROP_PX)
    if cl:
        b = max(cl, key=lambda c: c["n"])
        ring = b["idx"] & (Z > b["zmax"] - 0.005)
        bx, by, br = _kasa(X[ring], Y[ring])
        s["bowl"] = dict(x=bx, y=by, r=br, rim=b["zmax"], n=b["n"])

    if want_plate:
        tab = [c for c in _clusters(ws & (Z > z_table + 0.0035) &
                                    (Z < z_table + 0.10), X, Y, Z, 400)
               if c["zmax"] < z_table + 0.06
               and min(c["dx"], c["dy"]) > PLATE_MIN_D]
        if tab:
            p = max(tab, key=lambda c: min(c["dx"], c["dy"]))
            r = np.sqrt((X - p["xc"]) ** 2 + (Y - p["yc"]) ** 2)
            s["plate"] = dict(x=p["xc"], y=p["yc"], n=p["n"],
                              d=min(p["dx"], p["dy"]),
                              z=float(np.median(Z[p["idx"] & (r < 0.03)])))
    return s


# ------------------------------------------------------------------ motion
def goto(api, xyz, seconds=2.0, tries=3, tol=REACH_TOL, reserve=0):
    """Command xyz, retrying only while the retries are still buying distance
    and the episode's command budget can spare them."""
    r = 9.9
    for i in range(tries):
        if i and (_MOVES[0] >= MAX_MOVES - reserve):
            break
        prev = r
        r = api.move([float(v) for v in xyz], seconds=seconds)
        _MOVES[0] += 1
        if r <= tol or (i and prev - r < MIN_GAIN):
            break
    return r


def rim_point(x, y, off):
    """The tool point that pinches the rim of a bowl centred on (x, y)."""
    if GRASP_AXIS == "y":
        return x, y + GRASP_SIGN * off
    return x + GRASP_SIGN * off, y


def where_landed(api, p):
    """Kasa fit on the rim of whatever now stands in the plate's footprint."""
    try:
        f = api.capture("cam_high")
        B = _cloud(f)
        X, Y, Z = B[..., 0], B[..., 1], B[..., 2]
        e = np.asarray(api.eef(), float)
        rad = np.sqrt((X - e[0]) ** 2 + (Y - e[1]) ** 2)
        near = np.sqrt((X - p["x"]) ** 2 + (Y - p["y"]) ** 2)
        m = (np.isfinite(Z) & (rad > ARM_KEEPOUT) & (near < 0.16)
             & (Z > p["z"] + 0.02) & (Z < p["z"] + 0.12))
        if int(m.sum()) < 300:
            return "nothing standing on the plate"
        zt = float(Z[m].max())
        ring = m & (Z > zt - 0.005)
        if int(ring.sum()) < 60:
            return "rim too thin n=%d" % int(ring.sum())
        cx, cy, r = _kasa(X[ring], Y[ring])
        return "(%.3f,%.3f) r=%.4f top=%.4f n=%d" % (cx, cy, r, zt, int(m.sum()))
    except Exception as ex:
        return "readout failed %s" % ex


def run(api):
    api.log("intent=%s" % api.instruction())
    s = look(api)
    if s["bowl"] is None or s["plate"] is None:
        return "perception: bowl=%s plate=%s" % (s["bowl"], s["plate"])
    b, p = s["bowl"], s["plate"]
    api.log("table=%.3f shelf=%.3f bowl=(%.3f,%.3f) r=%.4f rim=%.3f n=%d"
            % (s["z_table"], s["z_shelf"], b["x"], b["y"], b["r"], b["rim"],
               b["n"]))
    api.log("plate=(%.3f,%.3f) z=%.4f d=%.3f n=%d"
            % (p["x"], p["y"], p["z"], p["d"], p["n"]))

    off = max(b["r"] - RIM_INSET, 0.030)
    gx, gy = rim_point(b["x"], b["y"], off)
    px, py = rim_point(p["x"], p["y"], off)   # centres the carried bowl
    px, py = px + PLACE_BIAS[0], py + PLACE_BIAS[1]
    z_grasp = b["rim"] - DZ_GRASP

    api.grip(OPEN_W)
    api.settle(0.2)
    r1 = goto(api, [gx, gy, b["rim"] + HOVER], 2.5, reserve=6)
    r2 = goto(api, [gx, gy, z_grasp], 2.0, reserve=5)
    held = np.asarray(api.eef(), float)
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("pinch (%.3f,%.3f,%.3f) at %s w=%.4f res=%.3f/%.3f"
            % (gx, gy, z_grasp, np.round(held, 3).tolist(), g["width_m"],
               r1, r2))

    hang = float(held[2] - s["z_shelf"])
    r3 = goto(api, [gx, gy, CARRY_Z], 2.5, reserve=4)
    z_rel = p["z"] + hang + PLACE_GAP
    r4 = goto(api, [px, py, CARRY_Z], 3.0, reserve=3)
    r5 = goto(api, [px, py, z_rel], 3.0, reserve=1)
    g2 = api.gripper()
    api.log("release (%.3f,%.3f,%.3f) hang=%.3f at %s w=%.4f res=%.3f/%.3f/%.3f"
            % (px, py, z_rel, hang, np.round(api.eef(), 3).tolist(),
               g2["width_m"], r3, r4, r5))
    api.log("moves used %d" % _MOVES[0])

    api.grip(OPEN_W)
    api.settle(0.6)
    goto(api, [px, py, z_rel + 0.16], 2.0, tries=1)
    api.settle(0.4)
    goto(api, [px - 0.10, py - 0.28, z_rel + 0.22], 2.5, tries=1)
    api.settle(0.3)
    got = where_landed(api, p)
    api.log("landed %s vs plate (%.3f,%.3f)"
            % (got, p["x"], p["y"]))
    return "v6 moves=%d" % _MOVES[0]
