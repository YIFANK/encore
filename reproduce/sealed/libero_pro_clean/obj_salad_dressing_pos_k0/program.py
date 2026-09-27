"""c2clean obj_salad_dressing_pos_k0 -- v4: v3 mechanism + closed-loop aim + retry.

v3 scored 4/4 on the probe subset.  Two things it left on the table, both
measured from its own logs:

  * controller residual.  The commanded grasp xy was (0.150,0.029) but the eef
    landed at (0.1406,0.029): the descent alone swung x by -0.015, and the
    residual is pose-dependent (at the hover 10cm higher the same command
    landed x +0.0056).  Harmless in v3 only because the jaws close along base
    Y (established by diffing the wrist view with the gripper open vs shut:
    the fingers move along image u, and the wrist extrinsic maps +u to base
    -y), so x is the forgiving axis.  v4 cancels the residual at the grasp
    height anyway, with a bounded 2-step servo.
  * no verification.  v4 checks the grip after the lift (effort 3.0 AND a
    plausible closed gap) and re-tries the grasp once if it came up empty.

Geometry unchanged from v3 and re-derived only from debug seeds 51-65:
the dressing is the only prop with a green cap (+0.081 vs -0.001 runner-up on
every probed seed); its neck is a constant 0.035 wide over z in [0.10,0.148];
cam_high sits at +x looking -x so radius comes from the unbiased y extent and
x_centre = x_max - radius (the wrist view at hover independently agrees:
y 0.0301 vs 0.029, x_centre 0.1515 vs 0.150).
"""

from collections import deque

import numpy as np

PROVENANCE = {
    "Z_TABLE": {
        "source": "debug seeds 51-65 cam_high pointcloud: 212k/262k points lie "
                  "below z=0.03 forming the table plane at z~=0; 0.015 is the "
                  "lift-off cut above it",
        "allowed": True},
    "Z_ARM": {
        "source": "debug-seed pointcloud z-histogram: prop mass ends by z=0.17, "
                  "the 0.24-0.49 band is the robot's own arm in frame",
        "allowed": True},
    "XY_LIM": {
        "source": "debug-seed pointcloud: room floor/walls deproject out to "
                  "x=-1.99,|y|=1.15; all 7 props+basket lie inside |x|,|y|<0.45",
        "allowed": True},
    "MIN_PX": {
        "source": "debug seeds: prop components are 851-11654 px, noise specks "
                  "are <150 px",
        "allowed": True},
    "CAP_BAND": {
        "source": "debug seeds: the dressing's green cap occupies the top ~0.035 "
                  "of the bottle, so a 0.015 top band is inside it for every prop",
        "allowed": True},
    "NECK_Z0/NECK_Z1": {
        "source": "debug-seed z-profile of the dressing: y-width is a constant "
                  "0.035 over z in [0.10,0.148] and flares to 0.063 below z=0.10",
        "allowed": True},
    "GRASP_Z": {
        "source": "midpoint of the measured parallel neck section (z 0.10-0.148)",
        "allowed": True},
    "TIP_OFFSET": {
        "source": "debug seeds 51-57: descent commanded to z=-0.06 on bare table "
                  "stalls with eef z=0.0103 on every seed; table top is z=0, so "
                  "the fingertips lead the eef frame by 0.0103 (measured live "
                  "each episode, this literal is only the fallback)",
        "allowed": True},
    "HOLD_MIN/HOLD_MAX": {
        "source": "debug seeds 51-57: a true neck grip reads closed gap 0.0368 "
                  "(neck is 0.035); a free close reads 0.0010",
        "allowed": True},
    "SERVO_TOL/SERVO_MAX/SERVO_N": {
        "source": "debug seeds: observed command-to-eef residual is <=0.015; "
                  "bounded so the bias-cancel loop cannot run away",
        "allowed": True},
    "R_DOWN": {
        "source": "generic controller mechanics: tool-to-world matrix for a "
                  "straight-down wrist; matches the observed reset "
                  "tool_rotation() diag(1,-1,-1) to within its 3.3 deg tilt",
        "allowed": True},
}

Z_TABLE = 0.015
Z_ARM = 0.25
XY_LIM = 0.45
MIN_PX = 150
CAP_BAND = 0.015
NECK_Z0, NECK_Z1 = 0.105, 0.145
GRASP_Z = 0.120
TIP_OFFSET = 0.0103
HOLD_MIN, HOLD_MAX = 0.020, 0.055
SERVO_TOL, SERVO_MAX, SERVO_N = 0.003, 0.030, 2

R_DOWN = np.array([[1.0, 0.0, 0.0],
                   [0.0, -1.0, 0.0],
                   [0.0, 0.0, -1.0]])


# ---------------------------------------------------------------- perception
def pointcloud(depth, K, T):
    H, W = depth.shape
    vs, us = np.mgrid[0:H, 0:W]
    x = (us - K[0, 2]) * depth / K[0, 0]
    y = (vs - K[1, 2]) * depth / K[1, 1]
    cam = np.stack([x, y, depth], -1).reshape(-1, 3)
    return (cam @ T[:3, :3].T + T[:3, 3]).reshape(H, W, 3)


def label(mask):
    """4-connected components; numpy-only (scipy is not guaranteed here)."""
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    cur = 0
    for sy, sx in np.argwhere(mask):
        if lab[sy, sx]:
            continue
        cur += 1
        q = deque([(int(sy), int(sx))])
        lab[sy, sx] = cur
        while q:
            y, x = q.popleft()
            for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
                if 0 <= ny < H and 0 <= nx < W and mask[ny, nx] and not lab[ny, nx]:
                    lab[ny, nx] = cur
                    q.append((ny, nx))
    return lab, cur


def components(pts, rgb):
    ok = np.isfinite(pts).all(-1)
    sel = (ok & (pts[..., 2] > Z_TABLE) & (pts[..., 2] < Z_ARM)
           & (np.abs(pts[..., 0]) < XY_LIM) & (np.abs(pts[..., 1]) < XY_LIM))
    lab, n = label(sel)
    out = []
    for i in range(1, n + 1):
        msk = lab == i
        npx = int(msk.sum())
        if npx < MIN_PX:
            continue
        P = pts[msk]
        C = rgb[msk].astype(float) / 255.0
        ztop = float(P[:, 2].max())
        cap = C[P[:, 2] > ztop - CAP_BAND].mean(0)
        out.append(dict(npx=npx, P=P, ztop=ztop, cap=cap,
                        greenness=float(cap[1] - max(cap[0], cap[2]))))
    return out


def neck_grasp(P):
    """Neck grasp xy: y from the unbiased cross-view extent, x from the visible
    near-face tangent minus that radius."""
    s = P[(P[:, 2] >= NECK_Z0) & (P[:, 2] <= NECK_Z1)]
    if len(s) < 20:
        s = P
    y0, y1 = float(s[:, 1].min()), float(s[:, 1].max())
    r = (y1 - y0) / 2.0
    return float(s[:, 0].max()) - r, (y0 + y1) / 2.0, r


def rim_centre(P, ztop):
    rim = P[P[:, 2] > ztop - 0.012]
    return (float(rim[:, 0].min() + rim[:, 0].max()) / 2.0,
            float(rim[:, 1].min() + rim[:, 1].max()) / 2.0)


# ---------------------------------------------------------------- motion
def servo(api, x, y, z, tag):
    """Move to (x,y,z) and cancel the controller's pose-dependent xy residual."""
    api.move([x, y, z], rotation=R_DOWN, seconds=2.0)
    cx, cy = x, y
    for k in range(SERVO_N):
        e = api.eef()
        ex, ey = x - float(e[0]), y - float(e[1])
        api.log("%s servo%d eef=%s err=(%+.4f,%+.4f)"
                % (tag, k, np.round(e, 4).tolist(), ex, ey))
        if abs(ex) < SERVO_TOL and abs(ey) < SERVO_TOL:
            break
        cx = x + max(-SERVO_MAX, min(SERVO_MAX, cx + ex - x))
        cy = y + max(-SERVO_MAX, min(SERVO_MAX, cy + ey - y))
        api.move([cx, cy, z], rotation=R_DOWN, seconds=1.5)
    return api.eef()


def holding(api):
    g = api.gripper()
    return (g["effort"] >= 3.0 and HOLD_MIN <= g["width_m"] <= HOLD_MAX), g


def run(api):
    api.log("INSTRUCTION %s" % api.instruction())
    f = api.capture("cam_high")
    pts = pointcloud(np.asarray(f.depth, np.float32),
                     np.asarray(f.intrinsics), np.asarray(f.t_base_cam))
    comps = components(pts, np.asarray(f.rgb, np.uint8))
    api.log("NCOMP %d" % len(comps))
    for c in comps:
        P = c["P"]
        api.log("COMP npx=%d x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f green=%+.3f" % (
            c["npx"], P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max(),
            c["ztop"], c["greenness"]))

    target = max(comps, key=lambda c: c["greenness"])
    basket = max(comps, key=lambda c: c["npx"])
    gx, gy, gr = neck_grasp(target["P"])
    bx, by = rim_centre(basket["P"], basket["ztop"])
    ztop = target["ztop"]
    api.log("TARGET green=%+.3f ztop=%.3f grasp=(%.3f,%.3f) neck_r=%.3f"
            % (target["greenness"], ztop, gx, gy, gr))
    api.log("BASKET centre=(%.3f,%.3f) ztop=%.3f" % (bx, by, basket["ztop"]))

    # ---- fingertip offset: stall a descent on the bare table (z = 0).
    api.grip(0.08)
    ymin_all = min(float(c["P"][:, 1].min()) for c in comps)
    cal = [0.05, max(-XY_LIM + 0.02, ymin_all - 0.12)]
    api.move([cal[0], cal[1], 0.20], rotation=R_DOWN, seconds=2.0)
    api.move([cal[0], cal[1], -0.06], rotation=R_DOWN, seconds=3.0)
    api.settle(0.3)
    tip = float(api.eef()[2])
    if not (0.0 <= tip <= 0.05):          # implausible stall -> use the measured literal
        api.log("CAL implausible %.4f, falling back" % tip)
        tip = TIP_OFFSET
    api.log("TIP_OFFSET=%.4f" % tip)
    api.move([cal[0], cal[1], 0.25], rotation=R_DOWN, seconds=2.0)

    # ---- grasp the neck, verify, retry once
    ok = False
    for attempt in range(2):
        api.grip(0.08)
        api.move([gx, gy, ztop + tip + 0.08], rotation=R_DOWN, seconds=2.5)
        servo(api, gx, gy, GRASP_Z + tip, "grasp%d" % attempt)
        api.grip(0.0)
        api.settle(0.5)
        api.log("a%d closed grip=%s" % (attempt, api.gripper()))
        api.move([gx, gy, GRASP_Z + tip + 0.15], rotation=R_DOWN, seconds=2.5)
        api.settle(0.3)
        ok, g = holding(api)
        api.log("a%d lifted eef=%s grip=%s holding=%s"
                % (attempt, np.round(api.eef(), 4).tolist(), g, ok))
        if ok:
            break
        api.log("a%d grasp failed, retrying" % attempt)

    # ---- carry to the basket and release above its rim
    carry_z = basket["ztop"] + tip + 0.12
    api.move([gx, gy, carry_z], rotation=R_DOWN, seconds=2.0)
    servo(api, bx, by, carry_z, "carry")
    ok2, g = holding(api)
    api.log("over basket eef=%s grip=%s holding=%s"
            % (np.round(api.eef(), 4).tolist(), g, ok2))

    api.move([bx, by, basket["ztop"] + tip + 0.03], rotation=R_DOWN, seconds=2.0)
    api.grip(0.08)
    api.settle(0.5)
    api.log("released grip=%s" % api.gripper())
    api.move([bx, by, carry_z + 0.05], rotation=R_DOWN, seconds=2.0)
    api.log("DONE")
