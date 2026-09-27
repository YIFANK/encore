"""v7 -- v6 plus sensor verification and retry.

Pipeline: cam_high point cloud -> table plane -> connected components ->
height class picks the two cans -> blue-band colour picks the alphabet soup ->
Kasa circle fit on the rim gives the grasp centre -> top-down straddle grasp ->
carry above the basket rim -> release.  Every stage is checked with my own
sensors (gripper width/effort, re-perception); a failed check retries.

Motion law (debug seeds v4/v5): a repeated identical api.move command is a
no-op, and folding a large residual into the command makes the arm oscillate
(0.15-0.3 m per call), so the residual is folded in only once |err| < NEAR.
Budget is 1000 sim steps; one attempt costs about 200.
"""
import numpy as np

PROVENANCE = {
    "OBJ_ZMIN": {"source": "debug-seed measurement (v2 dump, 15 seeds): 0.008 m above the table plane keeps every prop; flattest prop ztop-table = 0.019", "allowed": True},
    "ARM_ZMAX": {"source": "debug-seed measurement (v2 dump): the robot's own body reaches ztop 0.486; every table prop is below 0.145", "allowed": True},
    "CAN_ZLO/CAN_ZHI": {"source": "debug-seed measurement (v2 dump, 15 seeds): both cans ztop-table = 0.080; cartons 0.141, basket rim 0.143, flat boxes 0.018-0.019", "allowed": True},
    "BLUE_BAND (0.008-0.030 below ztop)": {"source": "debug-seed measurement (v2 dump): B-R = +0.079 on the blue 'SOUP' can, -0.108 on the red/green can, |B-R|<=0.003 on the basket", "allowed": True},
    "MIN_COMP_PIX": {"source": "debug-seed measurement (v2 dump): the smallest real prop is 809 px; noise blobs are under 200", "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics: tool-to-world matrix for a straight-down wrist", "allowed": True},
    "NEAR/GAIN/TOL": {"source": "debug-seed measurement (v4, v5): identical re-issued move = no-op at |err|~0.01; residual folded in at |err|>0.03 caused 0.15-0.3 m oscillation", "allowed": True},
    "GRASP_DZ": {"source": "debug-seed measurement (v2): can height 0.080, so 0.030 below the top is mid-body and clears the table", "allowed": True},
    "HOLD_WMIN/HOLD_WMAX": {"source": "debug-seed measurement (v6): a held can reads width 0.0625 with effort 3.0; empty jaws close to 0.001", "allowed": True},
    "PLACE_MARGIN": {"source": "debug-seed measurement (v6): can bottom sits at the table at grasp, so eef_z - zg is the can-bottom height; 0.03 m clears the basket rim", "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
OBJ_ZMIN = 0.008
ARM_ZMAX = 0.25
MIN_COMP_PIX = 300
CAN_ZLO, CAN_ZHI = 0.05, 0.12
NEAR, GAIN, TOL = 0.030, 0.8, 0.004
GRASP_DZ = -0.030
HOLD_WMIN, HOLD_WMAX = 0.030, 0.076
PLACE_MARGIN = 0.03
BLUE_MIN = 0.02


def _cloud(f):
    d = np.asarray(f.depth, dtype=np.float64)
    K = np.asarray(f.intrinsics)
    T = np.asarray(f.t_base_cam)
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]
    x = (uu - K[0, 2]) / K[0, 0] * d
    y = (vv - K[1, 2]) / K[1, 1] * d
    return np.stack([x, y, d], -1) @ T[:3, :3].T + T[:3, 3]


def _label(mask):
    from scipy import ndimage
    lab, n = ndimage.label(mask, np.ones((3, 3)))
    return [np.argwhere(lab == i + 1) for i in range(n)]


def _kasa(x, y):
    A = np.c_[x, y, np.ones(len(x))]
    s = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)[0]
    cx, cy = s[0] / 2.0, s[1] / 2.0
    return float(cx), float(cy), float(np.sqrt(max(s[2] + cx * cx + cy * cy, 1e-9)))


def goto(api, p, tries=3, seconds=2.0, tag=""):
    p = np.asarray(p, float)
    e = np.asarray(api.eef(), float)
    for _k in range(tries):
        err = p - e
        if np.linalg.norm(err) < TOL:
            break
        cmd = p + (err * GAIN if np.linalg.norm(err) < NEAR else 0.0)
        api.move(cmd.tolist(), rotation=R_DOWN, seconds=seconds)
        e2 = np.asarray(api.eef(), float)
        api.log("  goto%s t%d p=%s eef=%s d=%.4f moved=%.4f"
                % (tag, _k, np.round(p, 3).tolist(), np.round(e2, 4).tolist(),
                   float(np.linalg.norm(e2 - p)), float(np.linalg.norm(e2 - e))))
        step = float(np.linalg.norm(e2 - e))
        e = e2
        if step < 0.0015:
            break
    return e


def perceive(api, tag=""):
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb).astype(np.float64) / 255.0
    P = _cloud(f)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    ws = (X > -0.40) & (X < 0.40) & (Y > -0.45) & (Y < 0.50)
    tab = ws & (Z < 0.02)
    zt = float(np.median(Z[tab])) if tab.sum() else 0.0
    comps = []
    for pix in _label(ws & (Z > zt + OBJ_ZMIN)):
        if len(pix) < MIN_COMP_PIX:
            continue
        r, c = pix[:, 0], pix[:, 1]
        p = P[r, c]
        ztop = float(p[:, 2].max())
        if ztop - zt > ARM_ZMAX:
            continue
        band = (p[:, 2] > ztop - 0.030) & (p[:, 2] < ztop - 0.008)
        col = rgb[r, c][band] if band.sum() >= 10 else rgb[r, c]
        comps.append(dict(n=len(pix), ztop=ztop, blue=float(col[:, 2].mean() - col[:, 0].mean()),
                          rim=p[p[:, 2] > ztop - 0.008]))
    for c in comps:
        api.log("comp%s n=%d ztop=%.3f blue=%+.3f" % (tag, c['n'], c['ztop'], c['blue']))
    return zt, comps


def find_target(zt, comps):
    cans = [c for c in comps if CAN_ZLO < c['ztop'] - zt < CAN_ZHI]
    if not cans:
        return None
    return max(cans, key=lambda c: c['blue'])


def run(api):
    api.log("instruction=%s" % api.instruction())
    for attempt in range(3):
        api.log("=== attempt %d" % attempt)
        zt, comps = perceive(api, tag="/a%d" % attempt)
        tgt = find_target(zt, comps)
        if tgt is None:
            api.log("no can-height component left -- stopping")
            return
        cx, cy, r = _kasa(tgt['rim'][:, 0], tgt['rim'][:, 1])
        ztop = tgt['ztop']
        api.log("TARGET blue=%+.3f c=%.4f,%.4f r=%.4f ztop=%.4f" % (tgt['blue'], cx, cy, r, ztop))
        if attempt > 0 and tgt['blue'] < BLUE_MIN:
            api.log("remaining can is not the blue one -- the target is already placed")
            return

        basket = max((c for c in comps if c is not tgt and 0.12 < c['ztop'] - zt < ARM_ZMAX),
                     key=lambda c: c['n'], default=None)
        if basket is None:
            api.log("NO BASKET")
            return
        bx, by, br = _kasa(basket['rim'][:, 0], basket['rim'][:, 1])
        bz = basket['ztop']
        api.log("BASKET n=%d ztop=%.4f c=%.4f,%.4f r=%.4f" % (basket['n'], bz, bx, by, br))

        api.grip(0.08)
        api.settle(0.2)
        goto(api, [cx, cy, ztop + 0.12], tries=2, tag="/hover")
        e = goto(api, [cx, cy, ztop + GRASP_DZ], tries=3, tag="/down")
        zg = float(e[2])
        api.log("grasp eef=%s zg=%.4f" % (np.round(e, 4).tolist(), zg))

        api.grip(0.0)
        api.settle(0.5)
        g = api.gripper()
        api.log("closed gripper=%s" % g)

        zcarry = max(zg + bz + PLACE_MARGIN, bz + 0.12)
        e = goto(api, [cx, cy, zcarry], tries=3, tag="/lift")
        g = api.gripper()
        api.log("lifted eef=%s gripper=%s" % (np.round(e, 4).tolist(), g))
        if not (HOLD_WMIN < g['width_m'] < HOLD_WMAX and g['effort'] > 1.0):
            api.log("GRASP FAILED (width %.4f effort %.2f) -- retry" % (g['width_m'], g['effort']))
            api.grip(0.08)
            api.settle(0.3)
            goto(api, [cx, cy, ztop + 0.22], tries=2, tag="/back")
            continue

        e = goto(api, [bx, by, zcarry], tries=3, tag="/over")
        g = api.gripper()
        api.log("over-basket eef=%s gripper=%s" % (np.round(e, 4).tolist(), g))
        if not (g['effort'] > 1.0 and g['width_m'] > HOLD_WMIN):
            api.log("DROPPED IN TRANSIT (width %.4f effort %.2f) -- retry" % (g['width_m'], g['effort']))
            api.grip(0.08)
            api.settle(0.3)
            continue

        api.grip(0.08)
        api.settle(0.6)
        api.log("released gripper=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))
        goto(api, [bx, by, zcarry + 0.10], tries=2, tag="/retreat")

        zt2, comps2 = perceive(api, tag="/post%d" % attempt)
        left = find_target(zt2, comps2)
        if left is None or left['blue'] < BLUE_MIN:
            api.log("VERIFIED: no blue can left on the table")
            return
        api.log("blue can still on the table (blue=%+.3f) -- retry" % left['blue'])
    api.log("attempts exhausted")
