"""v7: park -> perceive -> rim-straddle grasp (+y offset) -> place on the plate.

The bowl is held by a pinch on its rim wall, RIM_OFF behind the eef in y, so
the release point carries the same offset (v5 released at the plate centre and
the bowl landed on the plate's edge: 0/8).  Shape gates on the two targets and
one grasp retry on an empty close are the only additions over v6 (15/15).
"""
import numpy as np

PROVENANCE = {
    "PARK": {"source": "debug-seed v2 probe: 3 park poses tried, only "
                       "(-0.10,0.45,1.25) reached (residual 0.011) and it clears "
                       "the whole table from cam_high", "allowed": True},
    "BAND_BOWL": {"source": "debug-seed v3 height maps on seeds 51-65: the bowl "
                            "standing on the stove slab tops out at table+0.079, "
                            "table bowls at table+0.043, the cabinet bowl at "
                            "table+0.28", "allowed": True},
    "BOWL_W/PLATE_W": {"source": "debug-seed v3 footprints: bowl 0.096-0.114 wide, plate 0.132-0.138, cookie box 0.078-0.084", "allowed": True},
    "DEMO_BOWL/DEMO_PLATE": {"source": "pack keyframes: mean close eef and mean release eef", "allowed": True},
    "EMPTY_W": {"source": "debug-seed v4/v5 gripper receipts: 0.0010 closing on air, 0.0047-0.0054 holding the bowl", "allowed": True},
    "BAND_FLAT": {"source": "same height maps: plate and cookie box top out at "
                            "table+0.019, the stove slab at table+0.031",
                  "allowed": True},
    "RIM_OFF": {"source": "pack keyframes: close eef y minus the measured bowl "
                          "centre y on the debug seeds (~+0.046); bowl outer "
                          "radius 0.054 from the v3 footprints", "allowed": True},
    "Z_GRASP": {"source": "pack keyframes gripper_cmd=+1 eef z: 0.9488/0.9420/0.9695",
                "allowed": True},
    "Z_RELEASE": {"source": "pack keyframes gripper_cmd=-1 at the place: "
                            "0.9306/0.9568/0.9290", "allowed": True},
    "Z_CARRY": {"source": "pack ee_path apex 1.03-1.15 between grasp and place",
                "allowed": True},
    "XMIN..YMAX": {"source": "debug-seed v0/v1 deprojection: the table workspace, "
                             "cropped to exclude the robot mount post at x<-0.36",
                   "allowed": True},
}

PARK = (-0.10, 0.45, 1.25)
XMIN, XMAX, YMIN, YMAX, CELL = -0.36, 0.32, -0.44, 0.44, 0.006
BAND_BOWL = (0.055, 0.16)
BAND_FLAT = (0.010, 0.035)
RIM_OFF = 0.046
Z_GRASP = 0.947
Z_CARRY = 1.06
Z_RELEASE = 0.945
BOWL_W = (0.07, 0.14)
PLATE_W = 0.10
EMPTY_W = 0.0025
# last-resort anchors if perception finds nothing at all: the pack's mean
# close eef, and the mean release eef pulled back by RIM_OFF.
DEMO_BOWL = (-0.2724, -0.0961 - 0.046)
DEMO_PLATE = (0.0549, 0.2343 - 0.046)
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def cloud(f):
    H, W = f.depth.shape
    K = f.intrinsics
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:H, 0:W]
    z = f.depth.astype(float)
    pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], -1)
    T = f.t_base_cam
    return pc @ T[:3, :3].T + T[:3, 3]


def grid(P, rgb):
    nx = int(round((XMAX - XMIN) / CELL)); ny = int(round((YMAX - YMIN) / CELL))
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    ok = np.isfinite(Z) & (X > XMIN) & (X < XMAX) & (Y > YMIN) & (Y < YMAX)
    ix = np.clip(np.floor((X - XMIN) / CELL).astype(int), 0, nx - 1)
    iy = np.clip(np.floor((Y - YMIN) / CELL).astype(int), 0, ny - 1)
    Zg = np.full((nx, ny), -np.inf); Cg = np.zeros((nx, ny, 3))
    o = np.argsort(Z[ok])
    Zg[ix[ok][o], iy[ok][o]] = Z[ok][o]
    Cg[ix[ok][o], iy[ok][o]] = rgb[ok][o]
    return Zg, Cg


def label(mask):
    nx, ny = mask.shape
    lab = np.zeros(mask.shape, int); cur = 0; out = []
    for i in range(nx):
        for j in range(ny):
            if mask[i, j] and not lab[i, j]:
                cur += 1; st = [(i, j)]; lab[i, j] = cur; pts = []
                while st:
                    a, b = st.pop(); pts.append((a, b))
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            p, q = a + da, b + db
                            if 0 <= p < nx and 0 <= q < ny and mask[p, q] and not lab[p, q]:
                                lab[p, q] = cur; st.append((p, q))
                out.append(np.array(pts))
    return out


def desc(pts, Zg, Cg):
    xs = XMIN + (pts[:, 0] + 0.5) * CELL
    ys = YMIN + (pts[:, 1] + 0.5) * CELL
    zs = Zg[pts[:, 0], pts[:, 1]]
    return dict(n=len(pts), cx=float(xs.mean()), cy=float(ys.mean()),
                w=float(xs.max() - xs.min()), h=float(ys.max() - ys.min()),
                ztop=float(zs.max()), rgb=Cg[pts[:, 0], pts[:, 1]].mean(0))


def perceive(api):
    f = api.capture("cam_high")
    Zg, Cg = grid(cloud(f), f.rgb.astype(float) / 255)
    fin = np.isfinite(Zg)
    tab = float(np.median(Zg[fin]))
    bowls = [d for d in (desc(p, Zg, Cg) for p in
                         label(fin & (Zg > tab + BAND_BOWL[0]) & (Zg < tab + BAND_BOWL[1])))
             if d["n"] >= 20]
    flats = [d for d in (desc(p, Zg, Cg) for p in
                         label(fin & (Zg > tab + BAND_FLAT[0]) & (Zg < tab + BAND_FLAT[1])))
             if d["n"] >= 20]
    api.log("table %.4f nbowl %d nflat %d" % (tab, len(bowls), len(flats)))
    for d in bowls + flats:
        api.log("  cand n=%d cen(%.3f,%.3f) w%.3f h%.3f ztop %.3f rgb%s" % (
            d["n"], d["cx"], d["cy"], d["w"], d["h"], d["ztop"],
            np.round(d["rgb"], 2).tolist()))
    return tab, bowls, flats


def pick_bowl(bowls):
    """The bowl standing on the stove: the only thing in BAND_BOWL with a
    bowl-sized footprint (debug seeds 51-65: w = h = 0.096-0.114; the only
    other occupant of the band is a 7-9 cell sliver of the cabinet edge)."""
    if not bowls:
        return dict(cx=DEMO_BOWL[0], cy=DEMO_BOWL[1], w=0.0, h=0.0, n=0,
                    ztop=0.0, rgb=np.zeros(3))
    shaped = [d for d in bowls if BOWL_W[0] <= d["w"] <= BOWL_W[1]
              and BOWL_W[0] <= d["h"] <= BOWL_W[1]]
    return max(shaped or bowls, key=lambda d: d["n"])


def pick_plate(flats):
    """The plate: the pale, wide disc in BAND_FLAT. The cookie box is the same
    height but narrow (0.078-0.084) and brown (r 0.35-0.40, b 0.17-0.19); the
    stove slab is wide but grey (r 0.29-0.32)."""
    if not flats:
        return dict(cx=DEMO_PLATE[0], cy=DEMO_PLATE[1], w=0.0, h=0.0, n=0,
                    ztop=0.0, rgb=np.zeros(3))
    pale = [d for d in flats if d["rgb"][0] > 0.45 and d["w"] > PLATE_W]
    wide = [d for d in flats if d["w"] > PLATE_W]
    return max(pale or wide or flats, key=lambda d: d["n"])


def run(api):
    api.move(np.array(PARK), rotation=R_DOWN, seconds=2.5)
    api.settle(0.2)
    tab, bowls, flats = perceive(api)
    bowl = pick_bowl(bowls)
    plate = pick_plate(flats)
    api.log("TARGET bowl(%.3f,%.3f,ztop %.3f) plate(%.3f,%.3f)" % (
        bowl["cx"], bowl["cy"], bowl["ztop"], plate["cx"], plate["cy"]))

    for attempt in (0, 1):
        gx, gy = bowl["cx"], bowl["cy"] + RIM_OFF
        api.grip(0.08)
        api.move(np.array([gx, gy, 1.02]), rotation=R_DOWN, seconds=2.0)
        r = api.move(np.array([gx, gy, Z_GRASP]), rotation=R_DOWN, seconds=2.0)
        api.log("a%d seated res %.4f eef %s grip %s" % (
            attempt, r, np.round(api.eef(), 4).tolist(), api.gripper()))
        api.grip(0.0)
        api.settle(0.4)
        api.log("a%d closed %s" % (attempt, api.gripper()))
        api.move(np.array([gx, gy, Z_CARRY]), rotation=R_DOWN, seconds=2.0)
        g = api.gripper()
        api.log("a%d lifted %s eef %s" % (attempt, g, np.round(api.eef(), 4).tolist()))
        if g["width_m"] > EMPTY_W:
            break
        # jaws shut on nothing (empty-close receipt from the v4 probe: 0.0010,
        # a held bowl reads 0.0047-0.0054). Re-perceive and try once more.
        api.log("a%d EMPTY -> re-perceive" % attempt)
        api.grip(0.08)
        api.move(np.array(PARK), rotation=R_DOWN, seconds=2.0)
        api.settle(0.2)
        tab, bowls, flats = perceive(api)
        bowl = pick_bowl(bowls)

    px, py = plate["cx"], plate["cy"] + RIM_OFF
    api.move(np.array([px, py, Z_CARRY]), rotation=R_DOWN, seconds=3.0)
    api.move(np.array([px, py, Z_RELEASE]), rotation=R_DOWN, seconds=2.0)
    api.log("atplate eef %s grip %s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(0.08)
    api.settle(0.4)
    api.move(np.array([px, py, 1.05]), rotation=R_DOWN, seconds=1.5)
    api.log("done eef %s" % np.round(api.eef(), 4).tolist())
    return "v7"
