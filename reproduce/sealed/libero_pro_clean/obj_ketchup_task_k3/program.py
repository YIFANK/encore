"""v1 -- perceive the milk carton from cam_high, top-down grasp, drop in basket.

Identification rule (derived on debug seeds 51/53/55/57, v0 perception dump):
the scene holds 6 props + 1 basket.  The basket is an order of magnitude
larger in footprint than any prop.  Among the props, only the milk carton is
BOTH tall (top >= 0.10 m above the table) AND square in footprint
(min/max side >= 0.75); every bottle reads 0.53-0.57 and the flat box is
0.52.  Verified by cropping each cluster out of the RGB (the square/tall one
carries the "Milk" label and a cow).
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug seeds 51-57 cam_high depth: dominant plane of the "
                          "deprojected cloud sits at z=0.001 m", "allowed": True},
    "Z_BAND": {"source": "debug-seed height histogram: props occupy 0.01-0.17 m, the "
                         "arm re-enters above 0.25 m", "allowed": True},
    "WORKSPACE": {"source": "debug-seed cam_high cloud: all props+basket lie in "
                            "|x|<0.35, |y|<0.45", "allowed": True},
    "BASKET_MIN_PIX": {"source": "debug seeds: basket cluster ~11.8k px vs <=2.3k px "
                                 "for every prop", "allowed": True},
    "MILK_TOP_MIN": {"source": "debug seeds: milk top 0.140 m; the two flat props top "
                               "at 0.020/0.081 m", "allowed": True},
    "MILK_ASPECT_MIN": {"source": "debug seeds: milk footprint aspect 0.96, all bottles "
                                  "0.53-0.57, flat box 0.52", "allowed": True},
    "GRASP_DEPTH": {"source": "both packs: gripper closes at eef z 0.098-0.113 while "
                              "the prop top measures 0.140 m on debug seeds -> the eef "
                              "sits ~0.035 m below the top at the close",
                    "allowed": True},
    "CARRY_Z": {"source": "both packs: transfer leg runs at eef z 0.29-0.38 m",
                "allowed": True},
    "RELEASE_Z": {"source": "both packs: gripper opens over the basket at eef z "
                            "0.153-0.212 m", "allowed": True},
    "CLOSE_CMD": {"source": "FairApi doc: grip(<0.025) closes", "allowed": True},
    "OPEN_CMD": {"source": "FairApi doc: grip(>=0.025) opens; measured open width "
                           "0.078 m on debug seeds", "allowed": True},
}

TABLE_Z = 0.0
Z_BAND = (0.012, 0.24)
WORKSPACE = (0.35, 0.45)
BASKET_MIN_PIX = 5000
MILK_TOP_MIN = 0.10
MILK_ASPECT_MIN = 0.75
GRASP_DEPTH = 0.035
CARRY_Z = 0.32
RELEASE_Z = 0.18
CLOSE_CMD = 0.0
OPEN_CMD = 0.08


# --------------------------------------------------------------- perception
def _cloud(frame):
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    d = np.nan_to_num(np.asarray(frame.depth, float), nan=0.0,
                      posinf=0.0, neginf=0.0)
    h, w = d.shape
    vs, us = np.mgrid[0:h, 0:w]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    cam = np.stack([(us - cx) * d / fx, (vs - cy) * d / fy, d], -1)
    pts = cam @ T[:3, :3].T + T[:3, 3]
    pts[d <= 0] = np.nan
    return pts


def _clusters(pts, cell=0.015, min_pix=60):
    z, x, y = pts[..., 2], pts[..., 0], pts[..., 1]
    m = (np.isfinite(z) & (z > Z_BAND[0]) & (z < Z_BAND[1])
         & (np.abs(x) < WORKSPACE[0]) & (np.abs(y) < WORKSPACE[1]))
    rows, cols = np.nonzero(m)
    px, py, pz = x[m], y[m], z[m]
    key = np.floor(np.stack([px, py], -1) / cell).astype(int)
    occ = {}
    for i, k in enumerate(map(tuple, key)):
        occ.setdefault(k, []).append(i)
    seen, comps = set(), []
    for k in occ:
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            c = stack.pop()
            comp += occ[c]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in occ and n not in seen:
                        seen.add(n)
                        stack.append(n)
        comps.append(np.array(comp))
    out = []
    for comp in comps:
        if comp.size < min_pix:
            continue
        cx_, cy_, cz_ = px[comp], py[comp], pz[comp]
        dx = float(cx_.max() - cx_.min())
        dy = float(cy_.max() - cy_.min())
        out.append(dict(
            n=int(comp.size), cx=float(cx_.mean()), cy=float(cy_.mean()),
            xmid=float((cx_.max() + cx_.min()) / 2),
            ymid=float((cy_.max() + cy_.min()) / 2),
            dx=dx, dy=dy, aspect=min(dx, dy) / max(dx, dy, 1e-6),
            top=float(np.percentile(cz_, 98)),
            u=(int(cols[comp].min()), int(cols[comp].max())),
            v=(int(rows[comp].min()), int(rows[comp].max()))))
    return out


# ------------------------------------------------------------------ motion
def _goto(api, xyz, rot, seconds=2.0, tries=3, tol=0.008):
    r = None
    for _ in range(tries):
        r = api.move(xyz, rotation=rot, seconds=seconds)
        if r is not None and r < tol:
            break
    return r


def run(api):
    api.log("INSTR %r" % api.instruction())
    rot = np.asarray(api.tool_rotation(), float)
    api.log("R0 %s" % rot.round(4).tolist())

    frame = api.capture("cam_high")
    cs = _clusters(_cloud(frame))
    for c in cs:
        api.log("CL n=%d xy=(%.3f,%.3f) d=(%.3f,%.3f) ar=%.2f top=%.3f"
                % (c["n"], c["cx"], c["cy"], c["dx"], c["dy"], c["aspect"], c["top"]))

    basket = max(cs, key=lambda c: c["n"])
    props = [c for c in cs if c is not basket and c["n"] < BASKET_MIN_PIX]
    cand = [c for c in props
            if c["top"] >= MILK_TOP_MIN and c["aspect"] >= MILK_ASPECT_MIN]
    if not cand:
        api.log("MILK-FALLBACK no square/tall prop; ranking by aspect")
        cand = sorted([c for c in props if c["top"] >= MILK_TOP_MIN],
                      key=lambda c: -c["aspect"])[:1]
    milk = max(cand, key=lambda c: c["aspect"])
    api.log("MILK xy=(%.3f,%.3f) top=%.3f ar=%.2f | BASKET xy=(%.3f,%.3f) n=%d"
            % (milk["xmid"], milk["ymid"], milk["top"], milk["aspect"],
               basket["cx"], basket["cy"], basket["n"]))

    gx, gy = milk["xmid"], milk["ymid"]
    gz = milk["top"] - GRASP_DEPTH

    api.grip(OPEN_CMD)
    _goto(api, [gx, gy, CARRY_Z], rot)
    _goto(api, [gx, gy, gz + 0.06], rot)
    r = _goto(api, [gx, gy, gz], rot, seconds=2.5)
    api.log("AT-GRASP eef=%s residual=%s" % (np.round(api.eef(), 4).tolist(), r))
    api.grip(CLOSE_CMD)
    api.settle(0.6)
    g = api.gripper()
    api.log("CLOSED %s" % g)

    _goto(api, [gx, gy, CARRY_Z], rot, seconds=2.5)
    api.log("LIFTED eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))

    bx, by = basket["cx"], basket["cy"]
    _goto(api, [bx, by, CARRY_Z], rot, seconds=3.0)
    _goto(api, [bx, by, RELEASE_Z], rot, seconds=2.0)
    api.log("OVER-BASKET eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(OPEN_CMD)
    api.settle(1.0)
    _goto(api, [bx, by, CARRY_Z], rot, seconds=2.0)
    api.settle(0.5)

    f2 = api.capture("cam_high")
    for c in _clusters(_cloud(f2)):
        api.log("POST n=%d xy=(%.3f,%.3f) ar=%.2f top=%.3f"
                % (c["n"], c["cx"], c["cy"], c["aspect"], c["top"]))
    api.log("END")
