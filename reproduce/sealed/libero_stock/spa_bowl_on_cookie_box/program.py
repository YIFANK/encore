"""c2 cell spa_bowl_on_cookie_box_stock -- v1.1.1 (v111)

Intent: "pick up the black bowl on the cookie box and place it on the plate"

v8 -> v111: fair-v1.1.1 done-excision pass, NO other change.  The nine
`api.done` attribute reads of v8 are removed: seven were log-string fields
only, and the two that appeared in conditions were dead on the whole debug
band (the ladder short-circuit -- every seed grips on rung 0 -- and the retry
gate -- every seed places).  Both conditions keep their own-sensor budget
guard (`B.left`, computed from this program's own move/grip accounting) and
simply drop the termination disjunct/conjunct.  Constants, PROVENANCE and
every motion are byte-identical to v8 otherwise.

v7 -> v8: NO behavioural change -- v8 is v7 with BUDGET_TOTAL and
PRESS_SECONDS added as explicit PROVENANCE entries so every literal in the file
is declared.  v7's receipt (sel_c2_spa_bowl_on_cookie_box_stock_v7, 15/15)
therefore carries over; v8 is re-run formally in its own right.

v5 -> v7: ONE change -- the rim is pinched on the -y side of the bowl centre
instead of the +y side.  The carry always runs from the cookie box (y ~ 0.03)
to the plate (y ~ 0.20), i.e. along +y.  Pinching at centre + rim_r*(+y) leaves
the bowl's mass TRAILING the jaws, so the +y acceleration pries it off the
pads; seeds 52 and 63 slipped exactly there.  Pinching at centre - rim_r*(+y)
puts the mass ahead of the jaws instead.

v4 -> v5: ONE change -- the bowl is carried at CARRY_Z = 1.020 instead of the
demo apex 1.065.  On debug seed 63 the rim pinch slipped mid-transfer in both
v3 and v4; the bowl fell ~11 cm and only scored because it happened to land on
the plate.  The measured geometry (grasp eef z 0.9564 with the bowl base on the
cookie-box top at 0.9204 -> base sits 0.036 m below the eef; plate top 0.9201)
says 1.020 still clears the plate by 6.4 cm while cutting the fall to 6.4 cm.

Measured on debug seeds (diagnostic runs, see NOTES): one episode holds
16.5 s of control time; a move that CONVERGES costs only ~0.225 s per 10 cm,
but a move that does not converge burns its full 2 x `seconds` cap.  v3's
caps summed to ~18.2 s, so every v3 episode hit api.done at the release
(seed 63 ran out mid-transfer and only scored because the dropped bowl
happened to land on the plate).  v4 sizes every cap from the actual travel
distance (~0.44 m/s measured), which cuts the worst case to ~9 s and buys
room for hold-verification and one full retry.
"""
import numpy as np

PROVENANCE = {
    "GRASP_ANCHOR": {
        "source": "pack.json demos[*].keyframes: mean xy of the three gripper-close "
                  "keyframes (demo0 t46, demo1 t46, demo2 t38)",
        "allowed": True},
    "PLACE_ANCHOR": {
        "source": "pack.json demos[*].keyframes: mean xy of the three gripper-open "
                  "release keyframes (demo0 t86, demo1 t96, demo2 t80)",
        "allowed": True},
    "CLOSE_Z_LADDER": {
        "source": "debug seeds 51-65: closing at 0.9530 gripped with effort 3.0 on "
                  "8/8 while 0.9680 closed on air on 8/8; rungs bracket it by 1 cm. "
                  "Consistent with pack.json ee_path z at the close command "
                  "(0.9528 / 0.9334 / 0.9582).",
        "allowed": True},
    "RELEASE_Z": {
        "source": "pack.json release-keyframe z mean (0.9476, 0.9258, 0.9387)",
        "allowed": True},
    "LIFT_Z": {
        "source": "pack.json demos[*].ee_path transfer apex (1.0703, 1.0653, 1.0410)",
        "allowed": True},
    "GRASP_SIDE_SIGN": {
        "source": "debug seeds 51-65: the carry always runs +y (cookie box y~0.03 "
                  "-> plate y~0.20); pinching the trailing rim side lost the bowl "
                  "mid-transfer on seeds 52 and 63, so the pinch is moved to the "
                  "leading side",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug seeds 51-65: grasp eef z 0.9548-0.9569 with the bowl base "
                  "resting on the cookie-box top measured at 0.9204 (post-pick "
                  "zmax) -> the held bowl's base sits 0.036 m below the eef; the "
                  "plate top measures 0.9201, so 1.020 keeps >=6 cm clearance",
        "allowed": True},
    "APPROACH_CLEAR": {
        "source": "generic controller mechanics: fixed 10 cm standoff above the "
                  "grasp point before descending",
        "allowed": True},
    "CLUSTER_GRID": {
        "source": "generic depth-cloud mechanics; 1.5 cm ground-footprint cells",
        "allowed": True},
    "TOP_SLAB": {
        "source": "generic depth-cloud mechanics: top 2 cm of a cluster is its rim band",
        "allowed": True},
    "TALL_MIN": {
        "source": "debug seeds 51-65: table_z=0.9025, target rim zmax=0.9707, "
                  "flat-disc zmax=0.9201 on 8/8 -> 3.5 cm splits the two height "
                  "classes with >1.5 cm margin either side",
        "allowed": True},
    "FOOTPRINT_EXCLUDE_PAD": {
        "source": "generic mechanics: 2 cm pad around the measured target radius "
                  "when masking the target out of the cloud",
        "allowed": True},
    "PLACE_MAX_DIST": {
        "source": "debug seeds 51-65: the flat blob nearest PLACE_ANCHOR sat "
                  "0.039-0.059 m from it on 8/8; 0.15 m is a slack sanity gate "
                  "before falling back to the demo anchor",
        "allowed": True},
    "RELEASE_HEDGE": {
        "source": "pack.json rim-pinch geometry: the bowl centre trails the jaws by "
                  "one rim radius; hedged at half (v3: 8/8 on debug seeds)",
        "allowed": True},
    "MOVE_SPEED / SEC_MIN / SEC_MAX / SEC_SAFETY": {
        "source": "debug-seed budget census (diagnostic runs on seeds 51/53/55/57): "
                  "165 x settle(0.1s) per episode; 4 converging 0.10 m moves cost "
                  "0.9 s total -> 0.44 m/s; one non-converging move at seconds=1.2 "
                  "cost exactly 2.4 s; api.grip costs 0.325 s",
        "allowed": True},
    "BUDGET_TOTAL": {
        "source": "debug-seed budget census (diagnostic run, seed 51): 165 x "
                  "api.settle(0.1 s) fit in one episode -> 16.5 s of control",
        "allowed": True},
    "PRESS_SECONDS": {
        "source": "generic controller mechanics + debug-seed census: a move that "
                  "cannot converge burns exactly 2 x seconds (measured 2.4 s at "
                  "seconds=1.2), so a deliberately-blocked 0.11 m descent is "
                  "capped at 0.6 s = 0.25 s travel + ~0.95 s press",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "FairApi contract + debug-seed readings: effort 3.0 iff holding, "
                  "0.05 when empty",
        "allowed": True},
    "OPEN_WIDTH": {
        "source": "debug-seed api.gripper() reading (open width 0.0778 m); "
                  "api.grip(>=0.025) opens",
        "allowed": True},
}

GRASP_ANCHOR = np.array([0.0644, 0.0658])
PLACE_ANCHOR = np.array([0.0675, 0.2457])
CLOSE_Z_LADDER = [0.9530, 0.9430, 0.9630]
RELEASE_Z = 0.9374
LIFT_Z = 1.0650
CARRY_Z = 1.0200
GRASP_SIDE_SIGN = 1.0
APPROACH_CLEAR = 0.10
CLUSTER_GRID = 0.015
TOP_SLAB = 0.020
TALL_MIN = 0.035
FOOTPRINT_EXCLUDE_PAD = 0.020
PLACE_MAX_DIST = 0.15
RELEASE_HEDGE = 0.5
OPEN_WIDTH = 0.08
HOLD_EFFORT = 2.5

MOVE_SPEED = 0.44        # m/s, measured
SEC_SAFETY = 2.5         # cap = 2 x seconds, so this is a ~5x margin on travel
SEC_MIN, SEC_MAX = 0.40, 1.10
BUDGET_TOTAL = 16.5      # s of control per episode, measured
PRESS_SECONDS = 0.60     # deliberately-blocked descents: 0.25 s travel + press


# ---------------------------------------------------------------- perception
def build_cloud(api, frame):
    d = np.asarray(frame.depth, dtype=np.float64)
    if d.ndim == 3:
        d = d[..., 0]
    H, W = d.shape
    K = np.asarray(frame.intrinsics, dtype=np.float64)
    T = np.asarray(frame.t_base_cam, dtype=np.float64)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:H, 0:W]
    pc = np.stack([(uu - cx) / fx * d, (vv - cy) / fy * d, d], axis=-1)
    pb = pc @ T[:3, :3].T + T[:3, 3]
    errs = []
    for (u, v) in [(200, 300), (256, 400), (320, 350)]:
        try:
            g = np.asarray(frame.deproject(u, v), dtype=np.float64)
            errs.append(round(float(np.linalg.norm(pb[v, u] - g)), 4))
        except Exception:
            pass
    api.log("cloud deproject_err=%s" % errs)
    return pb


def table_height(pts):
    z = pts[:, 2]
    m = (z > 0.60) & (z < 1.20)
    if m.sum() < 100:
        return 0.90
    hist, edges = np.histogram(z[m], bins=120, range=(0.60, 1.20))
    i = int(np.argmax(hist))
    return float(0.5 * (edges[i] + edges[i + 1]))


def footprint_clusters(pts, grid=CLUSTER_GRID, min_pts=20):
    if len(pts) == 0:
        return []
    gx = np.floor(pts[:, 0] / grid).astype(np.int64)
    gy = np.floor(pts[:, 1] / grid).astype(np.int64)
    cells = {}
    for i in range(len(pts)):
        cells.setdefault((gx[i], gy[i]), []).append(i)
    seen, out = set(), []
    for c in cells:
        if c in seen:
            continue
        stack, comp = [c], []
        seen.add(c)
        while stack:
            cur = stack.pop()
            comp.append(cur)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nb = (cur[0] + dx, cur[1] + dy)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        idxs = []
        for cc in comp:
            idxs.extend(cells[cc])
        if len(idxs) >= min_pts:
            out.append(pts[np.array(idxs)])
    return out


def describe(p):
    lo, hi = p.min(axis=0), p.max(axis=0)
    return dict(n=len(p),
                mid=np.array([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0]),
                ext=np.array([hi[0] - lo[0], hi[1] - lo[1]]),
                zmax=float(hi[2]), zmin=float(lo[2]), pts=p)


def scene(api, tag=""):
    frame = api.capture("cam_high")
    P = build_cloud(api, frame).reshape(-1, 3)
    P = P[np.isfinite(P).all(axis=1)]
    tz = table_height(P)
    A = P[(P[:, 0] > -0.15) & (P[:, 0] < 0.45) &
          (P[:, 1] > -0.55) & (P[:, 1] < 0.55) &
          (P[:, 2] > tz + 0.012) & (P[:, 2] < tz + 0.45)]
    api.log("%stable_z=%.4f above=%d" % (tag, tz, len(A)))
    return tz, A


def locate(api, tag=""):
    """-> (centre_xy, rim_r, zmax, plate_xy) or None."""
    tz, A = scene(api, tag)
    tall = A[A[:, 2] > tz + TALL_MIN]
    tcl = [describe(p) for p in footprint_clusters(tall)]
    tcl.sort(key=lambda d: np.linalg.norm(d["mid"] - GRASP_ANCHOR))
    for i, d in enumerate(tcl[:4]):
        api.log("%stall%d n=%5d mid=(%.4f,%.4f) ext=(%.3f,%.3f) zmax=%.4f dg=%.3f"
                % (tag, i, d["n"], d["mid"][0], d["mid"][1], d["ext"][0],
                   d["ext"][1], d["zmax"], np.linalg.norm(d["mid"] - GRASP_ANCHOR)))
    if not tcl:
        api.log("%sNO TALL CLUSTER" % tag)
        return None
    tgt = tcl[0]
    slab = tgt["pts"][tgt["pts"][:, 2] > tgt["zmax"] - TOP_SLAB]
    sd = describe(slab)
    centre = sd["mid"]
    rim_r = float(max(sd["ext"][0], sd["ext"][1]) / 2.0)
    api.log("%sRIM n=%d centre=(%.4f,%.4f) ext=(%.4f,%.4f) zmax=%.4f rim_r=%.4f"
            % (tag, sd["n"], centre[0], centre[1], sd["ext"][0], sd["ext"][1],
               sd["zmax"], rim_r))

    far = A[np.hypot(A[:, 0] - centre[0], A[:, 1] - centre[1]) >
            rim_r + FOOTPRINT_EXCLUDE_PAD]
    flat = far[far[:, 2] < tz + TALL_MIN]
    fcl = [describe(p) for p in footprint_clusters(flat)]
    fcl.sort(key=lambda d: np.linalg.norm(d["mid"] - PLACE_ANCHOR))
    for i, d in enumerate(fcl[:3]):
        api.log("%sflat%d n=%5d mid=(%.4f,%.4f) ext=(%.3f,%.3f) zmax=%.4f dp=%.3f"
                % (tag, i, d["n"], d["mid"][0], d["mid"][1], d["ext"][0],
                   d["ext"][1], d["zmax"], np.linalg.norm(d["mid"] - PLACE_ANCHOR)))
    if fcl and np.linalg.norm(fcl[0]["mid"] - PLACE_ANCHOR) < PLACE_MAX_DIST:
        plate = fcl[0]["mid"].copy()
    else:
        plate = PLACE_ANCHOR.copy()
        api.log("%sPLATE fallback to demo anchor" % tag)
    api.log("%sPLATE=(%.4f,%.4f)" % (tag, plate[0], plate[1]))
    return centre, rim_r, float(tgt["zmax"]), plate


# ---------------------------------------------------------------- controller
class Budget(object):
    def __init__(self, api):
        self.api = api
        self.spent = 0.0

    def move(self, xyz, seconds=None, press=False):
        e = np.asarray(self.api.eef(), dtype=np.float64)
        dist = float(np.linalg.norm(np.asarray(xyz, dtype=np.float64) - e))
        if seconds is None:
            seconds = min(SEC_MAX, max(SEC_MIN, SEC_SAFETY * dist / MOVE_SPEED))
        self.spent += 2.0 * seconds          # worst case: never converges
        self.api.move(list(xyz), seconds=seconds)
        return dist

    def grip(self, w):
        self.spent += 0.33
        self.api.grip(w)

    def settle(self, s):
        self.spent += s
        self.api.settle(s)

    @property
    def left(self):
        return BUDGET_TOTAL - self.spent


def holding(api):
    g = dict(api.gripper())
    return g, float(g.get("effort", 0.0)) >= HOLD_EFFORT


def attempt(api, B, loc, tag=""):
    centre, rim_r, zmax, plate = loc
    R = np.asarray(api.tool_rotation(), dtype=np.float64)
    jaw = R[:, 1].copy()
    jaw[2] = 0.0
    jaw = jaw / (np.linalg.norm(jaw) + 1e-9)
    # GRASP_SIDE_SIGN = +1 puts the pinch on the tool +y axis, i.e. world -y,
    # so the bowl's mass leads the jaws on the +y carry to the plate.
    s = GRASP_SIDE_SIGN
    off = s * rim_r * jaw[:2]
    api.log("%sanchor_side=%+.0f used_side=%+.0f" %
            (tag, 1.0 if float(np.dot(GRASP_ANCHOR - centre, jaw[:2])) >= 0
             else -1.0, s))
    gx, gy = centre[0] + off[0], centre[1] + off[1]
    api.log("%sjaw=%s grasp=(%.4f,%.4f)" % (tag, np.round(jaw, 3).tolist(), gx, gy))

    B.move([gx, gy, min(LIFT_Z, zmax + APPROACH_CLEAR)])
    api.log("%sapproach eef=%s spent=%.2f" %
            (tag, np.round(np.asarray(api.eef()), 4).tolist(), B.spent))

    got = False
    for k, z in enumerate(CLOSE_Z_LADDER):
        B.move([gx, gy, z], seconds=PRESS_SECONDS)
        B.grip(0.0)
        B.settle(0.15)
        g, ok = holding(api)
        api.log("%srung%d z=%.4f eef=%s grip=%s hold=%s spent=%.2f" %
                (tag, k, z, np.round(np.asarray(api.eef()), 4).tolist(), g, ok,
                 B.spent))
        if ok:
            got = True
            break
        B.grip(OPEN_WIDTH)
        if B.left < 5.0:
            break
    if not got:
        api.log("%sGRASP FAILED" % tag)
        return False, off, plate

    B.move([gx, gy, CARRY_Z])
    g, ok = holding(api)
    api.log("%slift grip=%s hold=%s eef=%s spent=%.2f" %
            (tag, g, ok, np.round(np.asarray(api.eef()), 4).tolist(),
             B.spent))
    if not ok:
        api.log("%sLOST AFTER LIFT" % tag)
        return False, off, plate

    px = plate[0] + RELEASE_HEDGE * off[0]
    py = plate[1] + RELEASE_HEDGE * off[1]
    B.move([px, py, CARRY_Z])
    g, ok = holding(api)
    api.log("%stransfer eef=%s grip=%s hold=%s spent=%.2f" %
            (tag, np.round(np.asarray(api.eef()), 4).tolist(), g, ok,
             B.spent))

    B.move([px, py, RELEASE_Z], seconds=PRESS_SECONDS)
    api.log("%srelease eef=%s grip=%s spent=%.2f" %
            (tag, np.round(np.asarray(api.eef()), 4).tolist(),
             dict(api.gripper()), B.spent))
    B.grip(OPEN_WIDTH)
    B.settle(0.30)
    B.move([px, py, CARRY_Z])
    api.log("%sPLACED spent=%.2f" % (tag, B.spent))
    return True, off, plate


def run(api):
    api.log("instruction: %s" % api.instruction())
    api.log("eef0=%s grip0=%s" %
            (np.round(np.asarray(api.eef()), 4).tolist(), dict(api.gripper())))
    B = Budget(api)

    loc = locate(api)
    if loc is None:
        return
    placed, off, plate = attempt(api, B, loc)

    if not placed and B.left > 7.0:
        api.log("RETRY: budget left %.2f s" % B.left)
        B.grip(OPEN_WIDTH)
        B.move([-0.10, 0.0, 1.15])
        loc2 = locate(api, tag="R2 ")
        if loc2 is not None:
            placed, off, plate = attempt(api, B, loc2, tag="R2 ")

    # clear the camera line of sight, then audit where the bowl landed
    B.move([-0.10, 0.0, 1.15])
    api.log("FINAL spent=%.2f placed=%s" % (B.spent, placed))
    try:
        tz2, A2 = scene(api, tag="POST ")
        near = A2[(np.hypot(A2[:, 0] - plate[0], A2[:, 1] - plate[1]) < 0.14) &
                  (A2[:, 2] > tz2 + TALL_MIN)]
        if len(near) > 30:
            nd = describe(near)
            api.log("POST bowl n=%d mid=(%.4f,%.4f) ext=(%.3f,%.3f) zmax=%.4f "
                    "d_plate=(%.4f,%.4f)" %
                    (nd["n"], nd["mid"][0], nd["mid"][1], nd["ext"][0],
                     nd["ext"][1], nd["zmax"], nd["mid"][0] - plate[0],
                     nd["mid"][1] - plate[1]))
        else:
            api.log("POST bowl: only %d pts near plate" % len(near))
    except Exception as e:
        api.log("POST audit failed %r" % e)
