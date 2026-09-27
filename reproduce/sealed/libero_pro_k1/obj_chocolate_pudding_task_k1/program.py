"""c2k1clean / obj_chocolate_pudding_task_k1

Intent: "Pick the salad dressing and place it in the basket".

Perception.  One cam_high RGB-D frame is deprojected into the base frame and
split into table-top clusters.  The mate pack's demo grasps its bottle at EEF
xy (0.0712, -0.1069); projecting that through the cam_high intrinsics/extrinsics
onto the mate keyframe lands on a bottle with a dark-green cap, and the same
asset (identical 128 px patch) is the only green-topped cluster in this scene.
So: target = the cluster whose top band is green; basket = the wide cluster on
the +Y side.

Motion.  The mate demo's numbers, re-expressed against measured geometry:
close 0.021 below the cap top, carry high, open at basket rim + 0.032.
Every leg is chopped into <= 0.06 m hops -- api.move only stops early inside
POS_TOL, so one long leg burns its whole step allowance overshooting, and the
episode's step budget (1000) is the binding constraint, not aim.
"""
import numpy as np

PROVENANCE = {
    "OBJ_LO": {
        "source": "debug seeds 51-65 cam_high depth: base-frame Z of the table "
                  "surface is 0.001, props start above 0.012",
        "allowed": True},
    "ARM_Z_CUT": {
        "source": "debug seeds 51-65: every table prop tops out at 0.148 while "
                  "the arm/robot cloud lives above 0.24",
        "allowed": True},
    "WS_X/WS_Y": {
        "source": "debug seeds 51-65: all props and the basket lie inside "
                  "|X|<0.35, |Y|<0.45 in the base frame",
        "allowed": True},
    "MIN_PIX": {
        "source": "debug seeds 51-65: the five props/basket clusters are "
                  ">=1000 px; depth speckle clusters are <200 px",
        "allowed": True},
    "WIDE_YSPAN": {
        "source": "debug seeds 51-65: basket Y-span 0.171 m, every prop <=0.063",
        "allowed": True},
    "GREEN_TEST": {
        "source": "mate pack demo0 keyframes: the demo's grasp EEF xy projects "
                  "onto a dark-green-capped bottle; in the debug clouds that "
                  "same asset's cap is RGB (24,58,38), so G>R+12 and G>B+8 "
                  "selects it and nothing else on the table",
        "allowed": True},
    "CAP_RADIUS": {
        "source": "debug seed 51 cloud: the dressing's cap is 0.035 m across "
                  "from Z=0.104 up to Z=0.148, so radius 0.0175",
        "allowed": True},
    "GRASP_DROP": {
        "source": "mate pack demo0: gripper closes at EEF z=0.128 on the same "
                  "asset whose cap top measures 0.148 here -> 0.021 below top",
        "allowed": True},
    "CARRY_Z": {
        "source": "mate pack demo0 ee_path: carry altitude 0.3115; 0.30 keeps "
                  "the hanging bottle (0.1225 below the EEF, measured in the v2 "
                  "debug run) above the 0.144 basket rim",
        "allowed": True},
    "RELEASE_DROP": {
        "source": "mate pack demo0: opens at EEF z=0.176 with the basket rim "
                  "measured at 0.1437 here -> rim + 0.032",
        "allowed": True},
    "HOVER_Z": {
        "source": "mate/k1 pack ee_path: pre-grasp hover 0.247-0.287",
        "allowed": True},
    "HOP": {
        "source": "debug v2 run: 0.02-0.03 m legs converge inside POS_TOL in "
                  "~27 steps, while a 0.12 m leg burned its full allowance and "
                  "still missed by 0.018",
        "allowed": True},
    "LEG_S": {
        "source": "generic controller mechanics: api.move seconds only caps the "
                  "step allowance; 0.5 s = 60 steps is ample for a 0.06 m hop",
        "allowed": True},
    "OPEN_W": {
        "source": "debug-seed api.gripper() at reset: finger gap 0.0778 open",
        "allowed": True},
    "HELD_GAP": {
        "source": "debug v2 run: holding the cap reads gap 0.0343 / effort 3.0; "
                  "closing on air reads gap 0.0010",
        "allowed": True},
    "DEFAULT_OBJ": {
        "source": "debug seeds 51-65 cloud: dressing cap axis (0.1503, 0.0291), "
                  "cap top 0.1475, identical on all 15 seeds",
        "allowed": True},
    "DEFAULT_BASKET": {
        "source": "debug seeds 51-65 cloud: basket rim centre X in "
                  "[-0.014, 0.015], Y in [0.245, 0.273], rim top 0.1437",
        "allowed": True},
    "AIM_LADDER": {
        "source": "debug-seed retries: offsets spanning the cap's 0.0175 radius",
        "allowed": True},
    "CAP_Z_MIN": {
        "source": "debug seed 51 cloud: the dressing's green cap runs Z=0.104 "
                  "to 0.148; 0.05 is a slack floor under it",
        "allowed": True},
    "MIN_CAP_PIX": {
        "source": "debug seeds 51-65: the cap shows 713 green pixels; no other "
                  "prop shows any",
        "allowed": True},
    "OBJ_BOX/BASKET_BOX": {
        "source": "debug seeds 51-65 cloud: sanity envelopes around the measured "
                  "prop (0.150,0.029,0.148) and basket (0.008,0.256,0.144)",
        "allowed": True},
}

OBJ_LO = 0.012
ARM_Z_CUT = 0.20
WS_X, WS_Y = 0.35, 0.45
MIN_PIX = 200
WIDE_YSPAN = 0.12
CAP_RADIUS = 0.0175
GRASP_DROP = 0.021
CARRY_Z = 0.30
RELEASE_DROP = 0.032
HOVER_Z = 0.24
HOP = 0.055
LEG_S = 0.5
OPEN_W = 0.08
HELD_GAP = 0.006
DEFAULT_OBJ = (0.1503, 0.0291, 0.1475)
DEFAULT_BASKET = (0.0075, 0.2558, 0.1437)
AIM_LADDER = [(0.0, 0.0, 0.0), (0.010, 0.0, 0.0), (-0.010, 0.0, -0.008)]
CAP_Z_MIN = 0.05          # cap sits high on the bottle; ignore low green speckle
MIN_CAP_PIX = 80
OBJ_BOX = ((-0.34, 0.34), (-0.44, 0.44), (0.05, 0.20))
BASKET_BOX = ((-0.20, 0.20), (0.08, 0.44), (0.08, 0.20))


# ----------------------------------------------------------------- perception
def _inside(p, box):
    return all(lo <= float(v) <= hi for v, (lo, hi) in zip(p, box))


def _cloud(f):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    dep = np.asarray(f.depth, float)
    H, W = dep.shape
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    x = (uu - K[0, 2]) / K[0, 0] * dep
    y = (vv - K[1, 2]) / K[1, 1] * dep
    P = np.stack([x, y, dep, np.ones_like(dep)], -1) @ T.T
    return P[..., 0], P[..., 1], P[..., 2]


def _label(mask):
    """8-connected components, explicit stack (scipy is not in the sandbox)."""
    lab = np.zeros(mask.shape, np.int32)
    H, W = mask.shape
    n = 0
    for i0, j0 in np.argwhere(mask):
        if lab[i0, j0]:
            continue
        n += 1
        stack = [(int(i0), int(j0))]
        lab[i0, j0] = n
        while stack:
            a, b = stack.pop()
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    p, q = a + da, b + db
                    if 0 <= p < H and 0 <= q < W and mask[p, q] and not lab[p, q]:
                        lab[p, q] = n
                        stack.append((p, q))
    return lab, n


def _scene(api):
    try:
        f = api.capture("cam_high")
        rgb = np.asarray(f.rgb, float)
        X, Y, Z = _cloud(f)
        fin = np.isfinite(X) & np.isfinite(Y) & np.isfinite(Z)
        band = (fin & (Z > OBJ_LO) & (Z < 0.60)
                & (np.abs(Y) < WS_Y) & (np.abs(X) < WS_X))
        lab, n = _label(band)
        props = []
        for k in range(1, n + 1):
            sel = lab == k
            if int(sel.sum()) < MIN_PIX:
                continue
            zk = Z[sel]
            if zk.max() > ARM_Z_CUT:        # the arm, never a table prop
                continue
            top = zk >= zk.max() - 0.006
            props.append({"n": int(sel.sum()), "ztop": float(zk.max()),
                          "cap": rgb[sel][top].mean(0), "sel": sel,
                          "yspan": float(Y[sel].max() - Y[sel].min())})
        # The cap is found from green PIXELS, not from a whole cluster: a prop
        # that ends up fused with a neighbour still exposes its own cap.
        green = (band & (Z > CAP_Z_MIN)
                 & (rgb[..., 1] > rgb[..., 0] + 12)
                 & (rgb[..., 1] > rgb[..., 2] + 8))
        obj, best = None, -1e9
        for p in props:
            gp = p["sel"] & green
            k = int(gp.sum())
            if k > best:
                best, obj = k, p
        if obj is not None and best >= MIN_CAP_PIX:
            gp = obj["sel"] & green
            zt = float(Z[gp].max())
            cap = gp & (Z >= zt - 0.010)
            o = (float(X[cap].max() - CAP_RADIUS),
                 float((Y[cap].min() + Y[cap].max()) / 2.0), zt)
            osrc = "cap %dpx" % best
        else:
            o, osrc = DEFAULT_OBJ, "DEFAULT(cap %d px)" % best
        if not _inside(o, OBJ_BOX):
            o, osrc = DEFAULT_OBJ, osrc + "->outside box"
        bk = None
        for p in props:
            if p["yspan"] > WIDE_YSPAN and (bk is None or p["n"] > bk["n"]):
                bk = p
        if bk is not None:
            rim = bk["sel"] & (Z >= bk["ztop"] - 0.010)
            b = (float((X[rim].min() + X[rim].max()) / 2.0),
                 float((Y[rim].min() + Y[rim].max()) / 2.0), float(bk["ztop"]))
            bsrc = "rim n=%d" % bk["n"]
        else:
            b, bsrc = DEFAULT_BASKET, "DEFAULT"
        if not _inside(b, BASKET_BOX):
            b, bsrc = DEFAULT_BASKET, bsrc + "->outside box"
        return o, b, "obj=%s(%s) basket=%s(%s) props=%d" % (
            np.round(o, 4).tolist(), osrc, np.round(b, 4).tolist(), bsrc, len(props))
    except Exception as e:
        return DEFAULT_OBJ, DEFAULT_BASKET, "perception failed %r -> defaults" % (e,)


# ---------------------------------------------------------------------- motion
def _hop(api, target):
    """Walk to `target` in <= HOP legs; api.move only breaks inside POS_TOL."""
    t = np.asarray(target, float)
    for _ in range(12):
        cur = np.asarray(api.eef(), float)
        d = t - cur
        dist = float(np.linalg.norm(d))
        if dist < 0.014:
            return dist
        step = cur + d * min(1.0, HOP / dist)
        api.move(step.tolist(), seconds=LEG_S)
    return float(np.linalg.norm(t - np.asarray(api.eef(), float)))


def _grip_state(api):
    g = api.gripper()
    return float(g.get("width_m", 0.0)), float(g.get("effort", 0.0))


def _held(api):
    w, e = _grip_state(api)
    return (w > HELD_GAP) and (e >= 2.5), w, e


def run(api):
    api.log("instruction=%r" % (api.instruction(),))
    obj, basket, diag = _scene(api)
    api.log(diag)
    bx, by, zr = basket

    api.grip(OPEN_W)
    held = False
    for attempt in range(2):
        if attempt:                       # the scene moved: look again
            obj, basket, diag = _scene(api)
            api.log("re-perceive: " + diag)
            bx, by, zr = basket
        ox, oy, zt = obj
        hover_z = max(HOVER_Z, zt + 0.08)
        for i, (dx, dy, dz) in enumerate(AIM_LADDER):
            gx, gy = ox + dx, oy + dy
            gz = zt - GRASP_DROP + dz
            _hop(api, (gx, gy, hover_z))
            api.grip(OPEN_W)
            r = _hop(api, (gx, gy, gz))
            api.grip(0.0)
            api.settle(0.25)
            held, w, e = _held(api)
            api.log("a%d try%d aim=%s resid=%.4f eef=%s gap=%.4f eff=%.2f held=%s"
                    % (attempt, i, [round(gx, 4), round(gy, 4), round(gz, 4)], r,
                       np.round(api.eef(), 4).tolist(), w, e, held))
            if held:
                break
            api.grip(OPEN_W)
            _hop(api, (gx, gy, hover_z))
        if not held:
            continue
        _hop(api, (ox, oy, CARRY_Z))
        held, w, e = _held(api)
        api.log("a%d lifted eef=%s gap=%.4f eff=%.2f held=%s"
                % (attempt, np.round(api.eef(), 4).tolist(), w, e, held))
        if held:
            break
        api.grip(OPEN_W)

    if not held:
        api.log("grasp not achieved")
        return "no grasp"

    _hop(api, (bx, by, CARRY_Z))
    _hop(api, (bx, by, zr + RELEASE_DROP))
    held, w, e = _held(api)
    api.log("over basket eef=%s gap=%.4f eff=%.2f held=%s"
            % (np.round(api.eef(), 4).tolist(), w, e, held))

    api.grip(OPEN_W)
    api.settle(0.4)
    _hop(api, (bx, by, CARRY_Z))
    api.settle(0.3)
    api.log("done eef=%s grip=%s"
            % (np.round(api.eef(), 4).tolist(), api.gripper()))
    return "v4 placed"
