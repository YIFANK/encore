"""v6 -- ring-vote detection + wall pinch + place.

v5 held and placed on 4/8 debug seeds. Both failure modes were perception:
1 cm blob clustering fuses whatever touches, so an adjacent brown box (or the
ramekin) merged with the target bowl, moving its centre by 2-3 cm, and on one
seed the fused blob was even mistaken for the ramekin and the wrong vessel was
carried to the plate.

This version never trusts a blob centroid. Every vessel is a circle of known
radius, so each candidate is found by voting: a point on a rim ring votes for
all centres one radius away, and a true centre collects the whole ring's votes.
Fused blobs therefore still yield two separate peaks.

Geometry (debug seeds 51-65): bowl outer radius 55 mm, rim top 51 mm above the
table, interior floor 9 mm; ramekin radius 43 mm, top 43 mm; plate radius
~67 mm, top 19 mm; jaws open 77.8 mm; closed fingertips touch the table at
eef z = table + 7.8 mm.
"""
import numpy as np

PROVENANCE = {
    "GRID_RES_M": {"source": "generic clustering resolution", "allowed": True},
    "TABLE_BAND_M": {"source": "generic depth-noise band above the fitted plane",
                     "allowed": True},
    "BOWL_R": {"source": "debug seeds 51-65: black-bowl xy extent 0.110 m -> radius 0.055",
               "allowed": True},
    "BOWL_H": {"source": "debug seeds 51-65: black-bowl top 0.0512 m above the table",
               "allowed": True},
    "RAMEKIN_R": {"source": "debug seed 51: ramekin xy extent 0.086 m -> radius 0.043",
                  "allowed": True},
    "RAMEKIN_H": {"source": "debug seeds 51-65: ramekin top 0.0430 m above the table",
                  "allowed": True},
    "PLATE_R": {"source": "debug seeds 51-65: plate xy extent 0.135 m -> radius 0.067",
                "allowed": True},
    "GREY_SPREAD": {"source": "debug seed 51: vessels/plate have rgb channel spread <= 11, "
                              "the brown box 46", "allowed": True},
    "GRASP_DZ": {"source": "pack demo0 keyframe t=36 (close cmd) ee z 0.9444 minus the "
                           "table plane 0.9008 measured on debug seeds 51-65",
                 "allowed": True},
    "PLACE_DZ": {"source": "pack demo0 keyframe t=85 (open cmd) ee z 0.9367 minus the "
                           "plate top 0.9200 measured on debug seed 51", "allowed": True},
    "HOVER_Z": {"source": "pack demo0 ee_path carry apex t=60 z 1.0373, rounded to 1.03",
                "allowed": True},
    "PINCH_FRAC": {"source": "pack demo0 grasp offset 0.0258 m / bowl radius 0.055 = 0.47",
                   "allowed": True},
    "HALF_SPAN": {"source": "debug seeds 51-65: api.gripper() width at reset 0.0778, halved",
                  "allowed": True},
    "HELD_EFFORT": {"source": "FairApi contract: effort 3.0 iff holding", "allowed": True},
}

GRID_RES_M = 0.01
TABLE_BAND_M = 0.008
BOWL_R = 0.055
BOWL_H = 0.0512
RAMEKIN_R = 0.043
RAMEKIN_H = 0.0430
PLATE_R = 0.067
GREY_SPREAD = 25.0
GRASP_DZ = 0.0436
PLACE_DZ = 0.0167
HOVER_Z = 1.03
PINCH_FRAC = 0.47
HALF_SPAN = 0.0389
HELD_EFFORT = 2.5

VOTE_CELL = 0.005
X_LO, X_HI, Y_LO, Y_HI = -0.35, 0.30, -0.45, 0.45


# --------------------------------------------------------------------- vision
def cloud(f):
    d = np.asarray(f.depth, float)
    H, W = d.shape
    K = np.asarray(f.intrinsics, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vs, ws = np.mgrid[0:H, 0:W]
    ok = np.isfinite(d) & (d > 0)
    z = np.where(ok, d, 1.0)
    pts = np.stack([(ws - cx) * z / fx, (vs - cy) * z / fy, z, np.ones_like(z)], -1)
    return (pts @ np.asarray(f.t_base_cam, float).T)[..., :3], ok


def components(cells):
    todo = set(cells)
    out = []
    while todo:
        seed = todo.pop()
        comp, stack = {seed}, [seed]
        while stack:
            i, j = stack.pop()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    c = (i + di, j + dj)
                    if c in todo:
                        todo.discard(c)
                        comp.add(c)
                        stack.append(c)
        out.append(comp)
    return out


def blobs(px, py, res=GRID_RES_M, min_n=120):
    if len(px) < min_n:
        return []
    x0, y0 = px.min(), py.min()
    ii = np.floor((px - x0) / res).astype(int)
    jj = np.floor((py - y0) / res).astype(int)
    cells = {}
    for k in range(len(ii)):
        cells.setdefault((int(ii[k]), int(jj[k])), []).append(k)
    out = []
    for comp in components(set(cells)):
        idx = np.concatenate([cells[c] for c in comp])
        if len(idx) >= min_n:
            out.append((idx, len(comp)))
    return out


def ring_vote(px, py, radius, tol=0.007, min_frac=0.45, sep=0.055, max_hits=4):
    """Circle centres of radius `radius` supported by the points (Hough vote)."""
    if len(px) < 40:
        return []
    if len(px) > 2500:                       # keep the vote cheap
        sel = np.linspace(0, len(px) - 1, 2500).astype(int)
        px, py = px[sel], py[sel]
    nx = int((X_HI - X_LO) / VOTE_CELL) + 1
    ny = int((Y_HI - Y_LO) / VOTE_CELL) + 1
    acc = np.zeros((nx, ny), float)
    k = int(np.ceil((radius + tol) / VOTE_CELL))
    dd = np.arange(-k, k + 1) * VOTE_CELL
    ox, oy = np.meshgrid(dd, dd, indexing="ij")
    rr = np.hypot(ox, oy)
    keep = np.abs(rr - radius) <= tol
    offs_i = np.round(ox[keep] / VOTE_CELL).astype(int)
    offs_j = np.round(oy[keep] / VOTE_CELL).astype(int)
    pi = np.round((px - X_LO) / VOTE_CELL).astype(int)
    pj = np.round((py - Y_LO) / VOTE_CELL).astype(int)
    ii = (pi[:, None] + offs_i[None, :]).ravel()
    jj = (pj[:, None] + offs_j[None, :]).ravel()
    good = (ii >= 0) & (ii < nx) & (jj >= 0) & (jj < ny)
    np.add.at(acc, (ii[good], jj[good]), 1.0)
    hits = []
    peak = acc.max()
    if peak <= 0:
        return []
    for _ in range(max_hits):
        m = acc.argmax()
        v = acc.flat[m]
        if v < min_frac * peak:
            break
        i, j = np.unravel_index(m, acc.shape)
        cx, cy = X_LO + i * VOTE_CELL, Y_LO + j * VOTE_CELL
        hits.append((float(cx), float(cy), float(v)))
        di = int(np.ceil(sep / VOTE_CELL))
        acc[max(0, i - di): i + di + 1, max(0, j - di): j + di + 1] = 0.0
    return hits


def perceive(api):
    f = api.capture("cam_high")
    P, ok = cloud(f)
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    g = ok & np.isfinite(z)
    hz, edges = np.histogram(z[g & (z < 1.05)], bins=200)
    table_z = float(0.5 * (edges[hz.argmax()] + edges[hz.argmax() + 1]))
    m = (g & (z > table_z + TABLE_BAND_M) & (z < table_z + 0.30)
         & (x > X_LO) & (x < X_HI) & (y > Y_LO) & (y < Y_HI))
    ox, oy, oz = x[m], y[m], z[m]
    rgb = np.asarray(f.rgb, float)[m]
    spread = rgb.max(1) - rgb.min(1)

    # drop furniture: any blob standing taller than a vessel
    keep = np.zeros(len(ox), bool)
    for idx, _ in blobs(ox, oy):
        if float(oz[idx].max()) - table_z <= 0.075:
            keep[idx] = True
    grey = keep & (spread <= GREY_SPREAD)
    api.log(f"table_z={table_z:.4f} pts={len(ox)} keep={int(keep.sum())} "
            f"grey={int(grey.sum())}")

    hi = grey & (oz > table_z + BOWL_H - 0.006)
    bowls = ring_vote(ox[hi], oy[hi], BOWL_R, max_hits=3)
    api.log("bowlvote " + " ".join(f"({a:.3f},{b:.3f}){v:.0f}" for a, b, v in bowls))

    mid = grey & (oz > table_z + RAMEKIN_H - 0.011) & (oz < table_z + RAMEKIN_H + 0.002)
    rams = ring_vote(ox[mid], oy[mid], RAMEKIN_R, max_hits=4)
    rams = [r for r in rams
            if all(np.hypot(r[0] - b[0], r[1] - b[1]) > 0.030 for b in bowls)]
    api.log("ramvote " + " ".join(f"({a:.3f},{b:.3f}){v:.0f}" for a, b, v in rams))

    lo = grey & (oz > table_z + 0.011) & (oz < table_z + 0.028)
    plates = ring_vote(ox[lo], oy[lo], PLATE_R, max_hits=3)
    plates = [p for p in plates
              if all(np.hypot(p[0] - b[0], p[1] - b[1]) > 0.040 for b in bowls)
              and all(np.hypot(p[0] - r[0], p[1] - r[1]) > 0.040 for r in rams)]
    api.log("platevote " + " ".join(f"({a:.3f},{b:.3f}){v:.0f}" for a, b, v in plates))

    plate_top = table_z + 0.0192
    if plates:
        near = lo & (np.hypot(ox - plates[0][0], oy - plates[0][1]) < PLATE_R + 0.01)
        if near.sum() > 50:
            plate_top = float(np.percentile(oz[near], 97))
    return dict(table_z=table_z, bowls=bowls, rams=rams, plates=plates,
                plate_top=plate_top)


def pick_target(scene):
    bowls, rams, plates = scene["bowls"], scene["rams"], scene["plates"]
    if not bowls:
        return None
    if not plates:
        return bowls[0]
    a = np.array(plates[0][:2])
    if not rams:
        return min(bowls, key=lambda b: np.hypot(b[0] - a[0], b[1] - a[1]))
    ab = np.array(rams[0][:2]) - a
    L2 = float(ab @ ab) or 1e-9

    def dseg(b):
        p = np.array(b[:2])
        t = float(np.clip((p - a) @ ab / L2, 0.0, 1.0))
        return float(np.linalg.norm(p - (a + t * ab)))

    return min(bowls, key=dseg)


# --------------------------------------------------------------------- motion
class Mover:
    def __init__(self, api):
        self.api = api
        self.n = 0
        self.frozen = False

    def goto(self, target, seconds=1.2, iters=3, tol=0.005):
        t = np.asarray(target, float)
        cmd = t.copy()
        cur = self.api.eef()
        for _ in range(iters):
            before = cur
            self.api.move(cmd, seconds=seconds)
            self.n += 1
            cur = self.api.eef()
            err = t - cur
            res = float(np.linalg.norm(err))
            if res > 0.05 and float(np.linalg.norm(cur - before)) < 1e-4:
                self.frozen = True
                return cur, res
            if res < tol:
                break
            cmd = cmd + np.clip(err, -0.05, 0.05)
        return cur, float(np.linalg.norm(t - cur))


def side_clearance(scene, tgt, sign, s):
    """Free space where the outer finger will land."""
    p = np.array([tgt[0], tgt[1] + sign * (s + HALF_SPAN)])
    best = 9.0
    for c, r in ([(b[:2], BOWL_R) for b in scene["bowls"] if b is not tgt]
                 + [(r[:2], RAMEKIN_R) for r in scene["rams"]]
                 + [(p2[:2], PLATE_R) for p2 in scene["plates"]]):
        best = min(best, float(np.hypot(p[0] - c[0], p[1] - c[1])) - r)
    return best


def pinch(api, mv, tgt, sign, s, grasp_z, tag):
    p = np.array([tgt[0], tgt[1] + sign * s])
    api.grip(0.08)
    cur, res = mv.goto([p[0], p[1], HOVER_Z], seconds=1.2)
    api.log(f"{tag} hover res={res:.4f} at={np.round(cur, 4).tolist()}")
    if mv.frozen:
        return False, None
    cur, res = mv.goto([p[0], p[1], grasp_z], seconds=1.0, iters=2)
    api.log(f"{tag} descend res={res:.4f} at={np.round(cur, 4).tolist()}")
    if mv.frozen:
        return False, None
    api.grip(0.0)
    api.settle(0.3)
    gc = api.gripper()
    cur, _ = mv.goto([p[0], p[1], HOVER_Z], seconds=1.0, iters=1)
    gl = api.gripper()
    api.log(f"{tag} closed w={gc['width_m']:.4f} e={gc['effort']:.2f} -> lifted "
            f"w={gl['width_m']:.4f} e={gl['effort']:.2f} z={cur[2]:.4f}")
    held = gl["effort"] >= HELD_EFFORT and 0.003 < gl["width_m"] < 0.032
    return held, np.array([0.0, sign * s])


def run(api):
    mv = Mover(api)
    scene = perceive(api)
    tgt = pick_target(scene)
    if tgt is None or not scene["plates"]:
        return f"perception failed bowls={len(scene['bowls'])} plates={len(scene['plates'])}"
    api.log(f"TARGET=({tgt[0]:.3f},{tgt[1]:.3f}) plate=({scene['plates'][0][0]:.3f},"
            f"{scene['plates'][0][1]:.3f}) plate_top={scene['plate_top']:.4f}")

    s = PINCH_FRAC * BOWL_R
    grasp_z = scene["table_z"] + GRASP_DZ
    cl = {sg: side_clearance(scene, tgt, sg, s) for sg in (-1, 1)}
    api.log(f"s={s:.4f} grasp_z={grasp_z:.4f} clr={ {k: round(v, 3) for k, v in cl.items()} }")
    order = sorted((-1, 1), key=lambda sg: -cl[sg])
    if cl[-1] > 0.02 and cl[1] > 0.02:
        order = [-1, 1]                      # both clear: follow the demo's side

    held, off = False, None
    for att, sg in enumerate(order):
        held, off = pinch(api, mv, tgt, sg, s, grasp_z, f"a{att}s{sg:+d}")
        if held or mv.frozen:
            break
        scene2 = perceive(api)               # the bowl may have been nudged
        t2 = pick_target(scene2)
        if t2 is not None:
            tgt = t2
            api.log(f"re-aim TARGET=({tgt[0]:.3f},{tgt[1]:.3f})")
    api.log(f"moves={mv.n} held={held} frozen={mv.frozen}")
    if not held or mv.frozen:
        return f"no hold (moves={mv.n})"

    plate = scene["plates"][0]
    ee_place = np.array([plate[0], plate[1]]) + off
    place_z = scene["plate_top"] + PLACE_DZ
    cur, res = mv.goto([ee_place[0], ee_place[1], HOVER_Z], seconds=1.5)
    api.log(f"over plate res={res:.4f} at={np.round(cur, 4).tolist()} "
            f"g={api.gripper()['width_m']:.4f} e={api.gripper()['effort']:.2f}")
    cur, res = mv.goto([ee_place[0], ee_place[1], place_z], seconds=1.0, iters=2)
    api.log(f"lowered res={res:.4f} at={np.round(cur, 4).tolist()} place_z={place_z:.4f}")
    api.grip(0.08)
    api.settle(0.4)
    api.log(f"released g={api.gripper()['width_m']:.4f}")
    mv.goto([ee_place[0], ee_place[1], HOVER_Z], seconds=1.0, iters=1)
    api.log(f"moves_total={mv.n}")
    return "placed"
