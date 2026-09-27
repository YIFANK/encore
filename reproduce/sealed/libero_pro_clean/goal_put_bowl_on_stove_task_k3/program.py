"""v4 — put the plate on the stove.

Mechanism, all measured on debug seeds 51-65 (see PROVENANCE):
  * table plane = modal height of the cam_high cloud;
  * the plate is a shallow saucer ~0.136 m across whose outer lip stands
    0.019 m above the table and whose inside slopes down to 0.006 m at the
    centre, so at fingertip height the rim is a wedge ~0.022 m thick;
  * the bowl (0.051 m tall) can touch the plate, fusing the two in a plain
    ground map -- so the plate is found in a height BAND (0.012-0.026 m) that
    the bowl, the stove slab and every tall prop fall outside of;
  * the stove is the flat slab 0.19 x 0.19 m, 0.031 m above the table;
  * fingertips sit 0.0086 m below the reported eef frame (an open-jaw press
    on bare table stalls with the eef at table + 0.0086);
  * a rim-pinched plate swings edge-down and hangs nearly its own diameter
    below the fingertips: carried at 0.16 m it drags on the table and is
    torqued out of the jaws (seed 55), so the carry runs high;
  * api.move returns anywhere inside a 12 mm ball, so every pose that matters
    is re-issued with a bias until the eef has really arrived;
  * a hold shows as gripper effort 3.0 with a finger gap above 5 mm.
"""
import numpy as np

PROVENANCE = {
    "TIP_BELOW_EEF": {"source": "debug seeds 51/53: open-jaw press on bare table stalls at eef z = table + 0.0086", "allowed": True},
    "PLATE_ANCHOR": {"source": "mate pack ee_path6: its push descends onto the plate at xy ~ (0.04,-0.02)", "allowed": True},
    "STOVE_ANCHOR": {"source": "target pack keyframes: the bowl is released over the stove at xy ~ (-0.26,0.26)", "allowed": True},
    "PLATE_BAND": {"source": "debug seed 51 wrist-depth cross section: plate lip 0.0185 m above table, inner floor 0.0055 m; bowl lip 0.051 m", "allowed": True},
    "STOVE_BAND": {"source": "debug seeds 51-65 cam_high: slab top 0.031 m above the table, 0.19 x 0.19 m", "allowed": True},
    "PLATE_SIZE": {"source": "debug seeds 51-65 cam_high: plate diameter 0.135-0.140 m", "allowed": True},
    "RIM_INSET": {"source": "target pack keyframes: the bowl grasp eef y sits within 0.011 m of the rim edge; 0.004 m inset measured to hold on debug seeds 51-65", "allowed": True},
    "GRASP_TIP_Z": {"source": "debug seeds 51-57: tips 0.004 m above the table close on the rim wedge with a 0.022 m gap and effort 3.0", "allowed": True},
    "CARRY_Z": {"source": "debug seed 55: the plate hangs ~0.13 m below the tips; at carry 0.16 m it dragged and was lost, 0.28 m clears the stove slab", "allowed": True},
    "DANGLE_FRAC": {"source": "debug seed 55 gif + geometry: a rim-pinched plate hangs edge-down, ~0.9 of its measured diameter below the fingertips", "allowed": True},
    "POS_REFINE_TOL": {"source": "generic controller mechanics: api.move returns inside a 12 mm tolerance ball", "allowed": True},
}

PLATE_ANCHOR = (0.05, -0.02)
STOVE_ANCHOR = (-0.26, 0.26)
TIP_BELOW_EEF = 0.0086
PLATE_BAND = (0.012, 0.026)
STOVE_BAND = (0.022, 0.048)
PLATE_SIZE = (0.09, 0.20)
RIM_INSET = 0.004
GRASP_TIP_Z = (0.004, 0.010, 0.016)
CARRY_Z = 0.28
DANGLE_FRAC = 0.9
POS_REFINE_TOL = 0.004
CELL = 0.005
XR = (-0.45, 0.35)
YR = (-0.45, 0.45)


# --------------------------------------------------------------------------- perception

def heightmap(api, cam="cam_high"):
    f = api.capture(cam)
    H, W = f.depth.shape
    vv, uu = np.mgrid[0:H, 0:W]
    z = f.depth.astype(np.float64)
    K, T = np.asarray(f.intrinsics), np.asarray(f.t_base_cam)
    x = (uu - K[0, 2]) * z / K[0, 0]
    y = (vv - K[1, 2]) * z / K[1, 1]
    P = (np.stack([x, y, z, np.ones_like(z)], -1) @ T.T)[..., :3].reshape(-1, 3)
    good = np.isfinite(P[:, 2])
    table = float(np.median(P[good, 2]))
    m = (good & (P[:, 0] > XR[0]) & (P[:, 0] < XR[1])
         & (P[:, 1] > YR[0]) & (P[:, 1] < YR[1]) & (P[:, 2] > table - 0.05))
    pts = P[m]
    nx = int((XR[1] - XR[0]) / CELL) + 1
    ny = int((YR[1] - YR[0]) / CELL) + 1
    Hm = np.zeros((nx, ny))
    ix = ((pts[:, 0] - XR[0]) / CELL).astype(int).clip(0, nx - 1)
    iy = ((pts[:, 1] - YR[0]) / CELL).astype(int).clip(0, ny - 1)
    o = np.argsort(pts[:, 2])
    Hm[ix[o], iy[o]] = pts[o, 2]
    return Hm, table


def components(Hm, table, band, minn=20):
    """Cells whose column top lies inside `band` above the table, grouped."""
    M = (Hm > table + band[0]) & (Hm < table + band[1])
    seen = np.zeros(M.shape, bool)
    out = []
    for i, j in np.argwhere(M):
        if seen[i, j]:
            continue
        st = [(i, j)]
        seen[i, j] = True
        comp = []
        while st:
            a, b = st.pop()
            comp.append((a, b))
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    p, q = a + da, b + db
                    if 0 <= p < M.shape[0] and 0 <= q < M.shape[1] and M[p, q] and not seen[p, q]:
                        seen[p, q] = True
                        st.append((p, q))
        comp = np.array(comp)
        if len(comp) < minn:
            continue
        cx = XR[0] + comp[:, 0] * CELL
        cy = YR[0] + comp[:, 1] * CELL
        cz = Hm[comp[:, 0], comp[:, 1]]
        out.append(dict(n=int(len(comp)), xmin=float(cx.min()), xmax=float(cx.max()),
                        ymin=float(cy.min()), ymax=float(cy.max()),
                        cx=float(cx.mean()), cy=float(cy.mean()),
                        ztop=float(cz.max()), zmed=float(np.median(cz)),
                        ex=float(cx.max() - cx.min()), ey=float(cy.max() - cy.min()),
                        h=float(cz.max() - table)))
    out.sort(key=lambda d: -d["n"])
    return out


def nearest(cs, anchor):
    return min(cs, key=lambda c: (c["cx"] - anchor[0]) ** 2 + (c["cy"] - anchor[1]) ** 2) if cs else None


def perceive(api, tag=""):
    Hm, table = heightmap(api)
    pl = [c for c in components(Hm, table, PLATE_BAND)
          if PLATE_SIZE[0] <= max(c["ex"], c["ey"]) <= PLATE_SIZE[1]
          and min(c["ex"], c["ey"]) >= 0.07]
    st = [c for c in components(Hm, table, STOVE_BAND)
          if c["ex"] >= 0.12 and c["ey"] >= 0.12]
    plate, stove = nearest(pl, PLATE_ANCHOR), nearest(st, STOVE_ANCHOR)
    others = [c for c in components(Hm, table, (0.012, 0.40))
              if plate is None or abs(c["cx"] - plate["cx"]) > 0.03
              or abs(c["cy"] - plate["cy"]) > 0.03]
    api.log("%stable=%.4f plate_cands=%d stove_cands=%d" % (tag, table, len(pl), len(st)))
    for c in pl + st:
        api.log("%scand n=%d c=(%.3f,%.3f) ex=%.3f ey=%.3f h=%.3f" %
                (tag, c["n"], c["cx"], c["cy"], c["ex"], c["ey"], c["h"]))
    if plate:
        api.log("%sPLATE c=(%.3f,%.3f) y=(%.3f,%.3f) d=%.3f h=%.3f" %
                (tag, plate["cx"], plate["cy"], plate["ymin"], plate["ymax"],
                 max(plate["ex"], plate["ey"]), plate["h"]))
    if stove:
        api.log("%sSTOVE c=(%.3f,%.3f) ztop=%.3f h=%.3f" %
                (tag, stove["cx"], stove["cy"], stove["ztop"], stove["h"]))
    return table, plate, stove, others


# --------------------------------------------------------------------------- motion

def goto(api, p, tol=POS_REFINE_TOL, tries=3, seconds=1.2, tag=""):
    p = np.asarray(p, float)
    bias = np.zeros(3)
    e = api.eef()
    for _ in range(tries):
        api.move(p + bias, seconds=seconds)
        e = api.eef()
        err = p - e
        if float(np.linalg.norm(err)) < tol:
            break
        bias = bias + err
        seconds = 0.8
    api.log("goto%s want=%s got=%s err=%.4f" %
            (tag, np.round(p, 4).tolist(), np.round(e, 4).tolist(),
             float(np.linalg.norm(p - e))))
    return e


def held(api):
    g = api.gripper()
    return g["effort"] > 1.0, g


def grasp_side(plate, others):
    """+1 grasps the far (+y) arc, -1 the near one; take the clearer side."""
    def clear(y):
        d = [min(abs(y - o["ymin"]), abs(y - o["ymax"])) for o in others
             if o["xmin"] - 0.03 < plate["cx"] < o["xmax"] + 0.03]
        return min(d) if d else 1.0
    return 1 if clear(plate["ymax"]) >= clear(plate["ymin"]) else -1


# --------------------------------------------------------------------------- policy

def pick(api, table, plate, others):
    """Rim-pinch the plate; returns (holding, rim_offset_y, diameter)."""
    side = grasp_side(plate, others)
    diam = max(plate["ex"], plate["ey"])
    for attempt, tip_z in enumerate(GRASP_TIP_Z):
        gx = plate["cx"]
        gy = (plate["ymax"] - RIM_INSET) if side > 0 else (plate["ymin"] + RIM_INSET)
        gz = table + tip_z + TIP_BELOW_EEF
        api.log("attempt %d side=%+d tip_z=%.3f grasp=(%.3f,%.3f,%.4f)" %
                (attempt, side, tip_z, gx, gy, gz))
        api.grip(0.08)
        goto(api, [gx, gy, table + 0.12], tol=0.008, tries=2, seconds=1.5, tag="/pre")
        goto(api, [gx, gy, gz], tol=0.003, tries=4, seconds=1.0, tag="/down")
        api.grip(0.0)
        ok, g = held(api)
        api.log("closed attempt=%d held=%s grip=%s" % (attempt, ok, g))
        if not ok:
            api.grip(0.08)
            goto(api, [gx, gy, table + 0.14], tol=0.01, tries=1, seconds=1.2, tag="/up")
            continue
        goto(api, [gx, gy, table + CARRY_Z], tol=0.01, tries=2, seconds=2.0, tag="/lift")
        ok, g = held(api)
        api.log("lifted held=%s grip=%s" % (ok, g))
        if ok:
            return True, gy - plate["cy"], diam
        api.grip(0.08)
    return False, 0.0, diam


def run(api):
    api.log("instruction=%r" % api.instruction())
    placed = False
    for rnd in range(3):
        table, plate, stove, others = perceive(api, tag="r%d/" % rnd)
        if plate is None:
            api.log("round %d: no plate candidate" % rnd)
            break
        st_cx, st_cy, st_top = (stove["cx"], stove["cy"], stove["ztop"]) if stove else \
            (STOVE_ANCHOR[0], STOVE_ANCHOR[1], table + 0.031)
        holding, rim_off, diam = pick(api, table, plate, others)
        if not holding:
            api.log("round %d: grasp failed" % rnd)
            continue

        # carry high: the plate swings edge-down and hangs ~0.9 diameters below
        tx, ty = st_cx, st_cy + rim_off
        api.log("carry rim_off=%+.3f diam=%.3f target=(%.3f,%.3f)" % (rim_off, diam, tx, ty))
        here = api.eef()
        n = 3
        for k in range(1, n + 1):
            w = [here[0] + (tx - here[0]) * k / n, here[1] + (ty - here[1]) * k / n,
                 table + CARRY_Z]
            goto(api, w, tol=0.02 if k < n else 0.008, tries=1 if k < n else 3,
                 seconds=1.5, tag="/carry%d" % k)
            ok, g = held(api)
            if not ok:
                api.log("DROPPED during carry leg %d grip=%s" % (k, g))
                break
        ok, g = held(api)
        api.log("over stove held=%s grip=%s" % (ok, g))
        if not ok:
            api.grip(0.08)
            goto(api, [tx, ty, table + CARRY_Z], tol=0.03, tries=1, seconds=1.2, tag="/reset")
            continue

        # lower until the dangling plate meets the slab, then let go
        floor = st_top + 0.012 + DANGLE_FRAC * diam + TIP_BELOW_EEF
        z = api.eef()[2]
        for _ in range(10):
            want = max(floor, z - 0.025)
            api.move([tx, ty, want], seconds=0.8)
            got = api.eef()[2]
            api.log("lower want=%.4f got=%.4f grip=%s" % (want, got, api.gripper()))
            if got - want > 0.012 or want <= floor + 1e-6:
                break
            z = got
        api.grip(0.08)
        api.settle(0.5)
        goto(api, [tx, ty, table + CARRY_Z], tol=0.03, tries=1, seconds=1.5, tag="/retreat")
        perceive(api, tag="final/")
        placed = True
        break
    return "placed" if placed else "failed"
