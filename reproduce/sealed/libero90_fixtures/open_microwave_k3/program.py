"""l90abl / open_microwave_k3 -- v8 (frozen)

Mechanism, entirely re-derived from this pack + debug-seed (51-65) perception:

  The microwave's door is a flat panel flush with the box's front plane (measured
  y_face = -0.146 on seed 51), hinged on a VERTICAL axis at the box's near-x front
  corner (x1, y_face), swinging out into +y.  Nothing on a flush face can be pushed
  open, so the door is taken by its handle: a vertical bar standing 0.042 m proud of
  the door plane at x ~ -0.06, 0.012 m thick in x, spanning z 0.95..1.07.

  A straight-down wrist cannot get near the door -- the hand body is long in its
  jaw-opening axis and fouls the box, stalling at y = -0.044 (measured, v6 probe A/D)
  while the bar is at y = -0.11.  Yawed 90 deg the hand is narrow in y and the jaws
  open along base x, so the tool can descend from directly above with the two fingers
  straddling the bar (they clear it by 0.039 in x), close on the bar, and carry it.

  Opening is then a rigid-body arc of the gripped bar about the hinge, with the wrist
  yawing by the same angle.  The pack's three demos all finish at (0.171,0.071),
  (0.155,0.029), (0.125,0.098) -- on the 90 deg arc of radius |handle-hinge| about
  that corner, which is what fixes the hinge side and the sweep magnitude.
"""

import numpy as np

PROVENANCE = {
    "TABLE_Z": {
        "source": "debug-seed cam_high deprojection: modal z over the workspace crop "
                  "(0.9012 on every debug seed probed). Computed at run time, not hard coded.",
        "allowed": True},
    "LID_SLAB_SCAN": {
        "source": "debug-seed cam_high: scan candidate slab heights 0.10..0.34 m above the "
                  "table and keep the one whose largest 4-connected top-down component has "
                  "the highest area*fill (= the microwave lid, z~1.121, x[-0.180,0.162], "
                  "y[-0.372,-0.146] on seed 51; stable to +-0.012 m across seeds 51-65).",
        "allowed": True},
    "Y_FACE": {
        "source": "debug-seed cam_high: modal y (4 mm bins) of the box's mid-height points; "
                  "-0.146 on seed 51. Vertical slices at x=-0.16,-0.10,0.00,0.08,0.15 all "
                  "return that same plane, so it is the closed door plane. Run-time computed.",
        "allowed": True},
    "HANDLE": {
        "source": "debug-seed cam_high: mid-height points protruding more than 0.025 past "
                  "y_face; bar front y=-0.1036, median x=-0.0634, x-thickness 0.0118, "
                  "z 0.951..1.071 on seed 51. Run-time computed.",
        "allowed": True},
    "BAR_HALF_THICKNESS": {
        "source": "debug-seed measurement above: the bar's measured x-thickness is 0.0118, so "
                  "its axis sits ~0.008 behind its front face; used as y_bar = h_y - 0.008.",
        "allowed": True},
    "TIP_OFFSET": {
        "source": "debug-seed measurement (v3/v4 probes): pressing the tool straight down onto "
                  "bare table stalls with api.eef()[2] = table_z + 0.008..0.012; 0.010 used.",
        "allowed": True},
    "Z_INSERT": {
        "source": "midpoint of the perceived bar's free span, clamped to (table+0.06, lid-0.05); "
                  "coincides with the pack's ee_path6 contact height band 1.02-1.05.",
        "allowed": True},
    "HINGE": {
        "source": "pack keyframes demo0_t0130 / demo1_t0149 / demo2_t0142 show the swung-open "
                  "panel attached at the box's near-x front corner, and the packs' final eef "
                  "(0.171,0.071)/(0.155,0.029)/(0.125,0.098) lie on the 90-deg arc about "
                  "(x1, y_face). Taken at run time as (perceived x1, perceived y_face).",
        "allowed": True},
    "ARC_SWEEP_100DEG": {
        "source": "pack keyframes show the panel roughly perpendicular to the box when done; "
                  "swept in 10-deg increments to 100 deg so the last commands stay loaded "
                  "against the stop.",
        "allowed": True},
    "WRIST_YAW90": {
        "source": "debug-seed v6 reach probe: with the straight-down wrist the tool stalls at "
                  "y=-0.044 (x=-0.16) and y=-0.012 (x=-0.06); yawed 90 deg it reaches y=-0.082. "
                  "Generic gripper/controller mechanics plus this measurement.",
        "allowed": True},
    "GRIP_WIDTHS": {
        "source": "api.grip semantics (<0.025 closes); 0.08 to straddle, 0.0 to clamp. The "
                  "clamped width reads 0.0227 with effort 3.0 on debug seeds = the bar.",
        "allowed": True},
    "MOVE_SECONDS": {
        "source": "debug-seed timing: episodes are capped at 1000 sim steps and each api.move "
                  "costs ~35 steps, so the plan uses ~14 moves. Generic controller mechanics.",
        "allowed": True},
}

R_DOWN = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
R_YAW90 = np.array([[0, 1.0, 0], [1.0, 0, 0], [0, 0, -1.0]])


def rotz(deg, R):
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]) @ R


def cc_largest(mask):
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    cur, best = 0, (0, 0)
    seen = mask.copy()
    for sy, sx in np.argwhere(mask):
        if not seen[sy, sx]:
            continue
        cur += 1
        stack = [(sy, sx)]
        seen[sy, sx] = False
        n = 0
        while stack:
            y, x = stack.pop()
            lab[y, x] = cur
            n += 1
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < H and 0 <= nx < W and seen[ny, nx]:
                    seen[ny, nx] = False
                    stack.append((ny, nx))
        if n > best[1]:
            best = (cur, n)
    return lab == best[0], best[1]


def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    D = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = D.shape
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    P = np.stack([(uu - K[0, 2]) / K[0, 0] * D, (vv - K[1, 2]) / K[1, 1] * D, D], -1)
    return P @ T[:3, :3].T + T[:3, 3], f


def perceive(api):
    Pb, _ = cloud(api)
    X, Y, Z = Pb[..., 0], Pb[..., 1], Pb[..., 2]
    ws = (X > -0.45) & (X < 0.45) & (Y > -0.45) & (Y < 0.45) & (Z > 0.6) & (Z < 1.6)
    z0 = float(np.median(Z[ws]))
    res = 0.006
    n = 150
    out = {"z0": z0}

    def grid(m):
        g = np.zeros((n, n), bool)
        g[((X[m] + 0.45) / res).astype(int).clip(0, n - 1),
          ((Y[m] + 0.45) / res).astype(int).clip(0, n - 1)] = True
        return g

    best = None
    for zc in np.arange(z0 + 0.10, z0 + 0.34, 0.01):
        m = ws & (Z > zc - 0.02) & (Z < zc + 0.02)
        if m.sum() < 500:
            continue
        cg, area = cc_largest(grid(m))
        if area < 200:
            continue
        ii, jj = np.where(cg)
        xs, ys = -0.45 + ii * res, -0.45 + jj * res
        fill = area / max(1.0, ((xs.max() - xs.min()) / res) * ((ys.max() - ys.min()) / res))
        if best is None or area * fill > best[0]:
            best = (area * fill, zc, xs.min(), xs.max(), ys.min(), ys.max())
    if best is None:
        return out
    _, zc, x0, x1, yb, y1 = best
    out.update(lid_z=float(zc), x0=float(x0), x1=float(x1), y_back=float(yb))
    mid = ws & (Z > z0 + 0.05) & (Z < zc - 0.05) & (X > x0 - 0.04) & (X < x1 + 0.04) & \
        (Y > yb - 0.02) & (Y < y1 + 0.14)
    if mid.sum() < 200:
        return out
    hist, edges = np.histogram(Y[mid], bins=np.arange(yb - 0.02, y1 + 0.14, 0.004))
    y_face = float(edges[int(np.argmax(hist))] + 0.002)
    out["y_face"] = y_face
    prot = mid & (Y > y_face + 0.025)
    if prot.sum() > 50:
        yh = float(Y[prot].max())
        tip = prot & (Y > yh - 0.012)
        out.update(h_y=yh, h_x=float(np.median(X[tip])),
                   h_xw=float(X[tip].max() - X[tip].min()),
                   h_zlo=float(Z[prot].min()), h_zhi=float(Z[prot].max()))
    return out


def run(api):
    P = perceive(api)
    api.log("P=%s" % {k: round(float(v), 4) for k, v in P.items()})
    if "h_x" not in P:
        api.log("no handle; abort")
        return
    z0, x1, y_face = P["z0"], P["x1"], P["y_face"]
    hx = P["h_x"]
    y_bar = P["h_y"] - 0.008                       # bar axis, 1/2 of its measured thickness in
    off = 0.010
    z_ins = 0.5 * (max(P["h_zlo"], z0 + 0.06) + min(P["h_zhi"], P["lid_z"] - 0.05))
    ez = z_ins + off
    hinge = np.array([x1, y_face])
    api.log("PLAN hx=%.3f y_bar=%.3f ez=%.3f hinge=%s" %
            (hx, y_bar, ez, np.round(hinge, 3).tolist()))

    api.grip(0.08)
    api.move(np.array([hx, y_bar, P["lid_z"] + 0.10]), rotation=R_YAW90, seconds=2.5)
    api.log("above eef=%s" % np.round(api.eef(), 4).tolist())
    api.move(np.array([hx, y_bar, ez]), rotation=R_YAW90, seconds=2.0)
    api.log("astride eef=%s" % np.round(api.eef(), 4).tolist())
    api.grip(0.0)
    api.settle(0.3)
    g = api.gripper()
    api.log("closed g=%s eef=%s" % (g, np.round(api.eef(), 4).tolist()))

    p0 = np.asarray(api.eef(), float)
    d0 = p0[:2] - hinge
    r = float(np.linalg.norm(d0))
    api.log("arc r=%.3f" % r)
    for deg in range(10, 101, 10):
        t = np.radians(deg)
        c, s = np.cos(t), np.sin(t)
        d = np.array([d0[0] * c + d0[1] * s, -d0[0] * s + d0[1] * c])
        p = np.array([hinge[0] + d[0], hinge[1] + d[1], ez])
        res = api.move(p, rotation=rotz(-deg, R_YAW90), seconds=1.2)
        api.log("arc%03d tgt=%s got=%s res=%.4f g=%s" %
                (deg, np.round(p[:2], 4).tolist(), np.round(api.eef(), 4).tolist(),
                 float(res), api.gripper()))
    Q = perceive(api)
    api.log("POST=%s" % {k: round(float(v), 4) for k, v in Q.items()})
