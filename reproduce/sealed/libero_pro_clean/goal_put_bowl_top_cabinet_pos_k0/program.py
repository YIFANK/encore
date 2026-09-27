"""c2clean goal_put_bowl_top_cabinet_pos_k0 — frozen program (rim pinch).

One finger inside the bowl, one outside, straddling the rim wall at a radial
offset along the finger-closing axis (base y at the default straight-down wrist).
"""
import json
import sys

import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug-seed cam_high depth: dominant plane at z=0.901", "allowed": True},
    "TIP_OFFSET": {"source": "debug-seed 51/57 contact probes: eef z settles 0.0098-0.011 above the touched plane (bare table 0.901 -> eef 0.912; cabinet top 1.1276 -> eef 1.1374)", "allowed": True},
    "BOWL_BAND": {"source": "debug-seed cam_high depth: bowl rim plateau lies 39-69 mm above the table plane", "allowed": True},
    "CAB_BAND": {"source": "debug-seed cam_high depth: cabinet top plateau 215-240 mm above the table plane", "allowed": True},
    "RIM_OFFSET": {"source": "debug-seed radial depth profile (rim radius 0.055) plus a debug-seed aim-envelope sweep: 0.040/0.046/0.049/0.052 all succeed, 0.058 fails 1 of 4; 0.049 is the centre of the measured band", "allowed": True},
    "PINCH_DEPTH": {"source": "debug-seed radial depth profile: 15 mm below the rim top the inner wall stands at r~0.0445, so the jaws straddle solid wall", "allowed": True},
    "CARRY_Z": {"source": "debug-seed depth: tallest scene structure below 1.15; 1.25 clears it", "allowed": True},
}

TABLE_Z = 0.901
TIP_OFFSET = 0.010          # fingertip z = eef z - TIP_OFFSET
RIM_OFFSET = 0.049
PINCH_DEPTH = 0.015
CARRY_Z = 1.25
SEC = 0.5


def L(api, **kw):
    s = json.dumps(kw, default=float)
    api.log(s)
    print(s, file=sys.stderr)


def cloud(f):
    d = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]
    z = np.where(np.isfinite(d) & (d > 0.01) & (d < 5.0), d, 0.0)
    P = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z], -1)
    return P @ T[:3, :3].T + T[:3, 3], z > 0.01


def label(mask):
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    cur = 0
    seen = mask.copy()
    for y0, x0 in np.argwhere(mask):
        if not seen[y0, x0]:
            continue
        cur += 1
        stack = [(y0, x0)]
        seen[y0, x0] = False
        while stack:
            y, x = stack.pop()
            lab[y, x] = cur
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = y + dy, x + dx
                if 0 <= a < H and 0 <= b < W and seen[a, b]:
                    seen[a, b] = False
                    stack.append((a, b))
    return lab, cur


def find_bowl(X, Y, Z, ok):
    band = ok & (Z > TABLE_Z + 0.039) & (Z < TABLE_Z + 0.069) \
        & (X > -0.30) & (X < 0.32) & (Y > -0.35) & (Y < 0.35)
    lab, n = label(band)
    best = None
    for i in range(1, n + 1):
        c = lab == i
        if int(c.sum()) < 80:
            continue
        ex = float(X[c].max() - X[c].min())
        ey = float(Y[c].max() - Y[c].min())
        if best is None or min(ex, ey) > best[0]:
            best = (min(ex, ey), c, ex, ey)
    if best is None:
        return None
    _, c, ex, ey = best
    return dict(cx=float((X[c].max() + X[c].min()) / 2),
                cy=float((Y[c].max() + Y[c].min()) / 2),
                r=(ex + ey) / 4, ex=ex, ey=ey, n=int(c.sum()),
                ztop=float(np.percentile(Z[c], 95)))


def find_cab(X, Y, Z, ok):
    band = ok & (Z > TABLE_Z + 0.215) & (Z < TABLE_Z + 0.240) \
        & (X > -0.70) & (X < 0.40) & (Y > -0.80) & (Y < 0.80)
    lab, n = label(band)
    best = None
    for i in range(1, n + 1):
        c = lab == i
        k = int(c.sum())
        if k < 300:
            continue
        if best is None or k > best[0]:
            best = (k, c)
    if best is None:
        return None
    _, c = best
    return dict(n=int(c.sum()),
                xlo=float(np.percentile(X[c], 1)), xhi=float(np.percentile(X[c], 99)),
                ylo=float(np.percentile(Y[c], 1)), yhi=float(np.percentile(Y[c], 99)),
                z=float(np.percentile(Z[c], 50)))


def perceive(api):
    f = api.capture("cam_high")
    Pb, ok = cloud(f)
    X, Y, Z = Pb[..., 0], Pb[..., 1], Pb[..., 2]
    return find_bowl(X, Y, Z, ok), find_cab(X, Y, Z, ok)


def mv(api, tag, xyz, sec=SEC, iters=2, tol=0.004):
    t = np.asarray(xyz, float)
    cmd = t.copy()
    e = None
    for i in range(iters):
        r = api.move(cmd, seconds=sec)
        e = api.eef()
        err = t - e
        L(api, mv=tag, i=i, cmd=[round(float(v), 4) for v in cmd],
          eef=[round(float(v), 4) for v in e], resid=round(float(r), 4))
        if float(np.linalg.norm(err)) < tol:
            break
        cmd = cmd + err
    return e


def run(api):
    bowl, cab = perceive(api)
    L(api, phase="perceive", bowl=bowl, cab=cab)
    if bowl is None or cab is None:
        return "perception failed"

    bx, by = bowl["cx"], bowl["cy"]
    api.grip(0.08)

    # 1. hover, 2. descend beside/around the bowl, 3. close
    gy = by - RIM_OFFSET
    mv(api, "hover", [bx, gy, bowl["ztop"] + 0.06])
    gz = bowl["ztop"] - PINCH_DEPTH + TIP_OFFSET
    mv(api, "descend", [bx, gy, gz], iters=2, tol=0.003)
    api.grip(0.0)
    api.settle(0.2)
    L(api, phase="closed", grip=api.gripper(), eef=[float(v) for v in api.eef()])

    # 4. lift and verify
    mv(api, "lift", [bx, gy, CARRY_Z], iters=1)
    api.settle(0.2)
    L(api, phase="lifted", grip=api.gripper(), eef=[float(v) for v in api.eef()])
    b2, _ = perceive(api)
    L(api, phase="recheck", bowl=b2)

    # 5. carry over the cabinet and release
    ccx = (cab["xlo"] + cab["xhi"]) / 2
    ccy = (cab["ylo"] + cab["yhi"]) / 2
    # the held bowl centre sits RIM_OFFSET to +y of the eef
    ex_, ey_ = ccx, ccy - RIM_OFFSET
    mv(api, "swing", [0.0, ey_, CARRY_Z], iters=1)
    mv(api, "over", [ex_, ey_, CARRY_Z], iters=2)
    # bowl base hangs (rim height - PINCH_DEPTH) below the fingertips
    drop = cab["z"] + 0.008 + (bowl["ztop"] - TABLE_Z - PINCH_DEPTH) + TIP_OFFSET
    mv(api, "lower", [ex_, ey_, drop], iters=2, tol=0.004)
    L(api, phase="above_cab", grip=api.gripper(), eef=[float(v) for v in api.eef()])
    api.grip(0.08)
    api.settle(0.3)
    mv(api, "retreat", [ex_, ey_, CARRY_Z], iters=1)
    api.settle(0.3)
    b3, _ = perceive(api)
    L(api, phase="final", bowl=b3, grip=api.gripper())
    return "put bowl on cabinet top"
