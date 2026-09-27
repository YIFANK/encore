"""l90abl / open_bottom_drawer_k0 -- zero-demo program.

Mechanism: the cabinet's bottom drawer is opened by capturing its handle bar
between the gripper fingers and dragging it in +y.  The handle is found from
cam_high alone (bright, low-saturation pixels on the dark cabinet, lowest of
the three z bands).  The wrist is pitched 55 deg toward the cabinet because
that orientation reaches ~70 mm further in -y than a straight-down wrist
(debug sweeps v10/v15/v18/v20); the -y reach then saturates hard at
eef y ~ -0.195, which is where this cell's difficulty lives.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {
        "source": "debug seeds 51,53,57,61,65 cam_high deprojection: dominant table plane z=0.901",
        "allowed": True},
    "BRIGHT_G": {
        "source": "debug-seed cam_high RGB: the handles are the only bright (mean>120) low-saturation (<30) pixels on the dark cabinet",
        "allowed": True},
    "CAB_Y_WINDOW": {
        "source": "debug-seed cam_high: cabinet front face y=-0.2325, handle bars protrude to y~-0.20; window [-0.32,-0.15] isolates them from tableware",
        "allowed": True},
    "CAB_X_WINDOW": {
        "source": "debug-seed cam_high: cabinet x-extent [-0.133,0.118]; window [-0.25,0.25]",
        "allowed": True},
    "Z_BAND_STEP": {
        "source": "debug-seed cam_high z histogram: three handle bands at z=0.955/1.025/1.098, separated by empty 0.005 m bins",
        "allowed": True},
    "BAR_R": {
        "source": "debug-seed cam_high yz occupancy map: handle bar section ~0.015 m, so its centre is 0.0075 behind/below the observed front/top",
        "allowed": True},
    "PITCH_DEG": {
        "source": "debug-seed orientation sweeps v10/v15/v18/v20: pitch 55 deg maximises -y reach (eef y=-0.195 vs -0.130 straight-down); larger/negative pitches reach less",
        "allowed": True},
    "X_OFFSET": {
        "source": "debug-seed reach maps v14/v15: -y reach improves with x, so aim +x of the bar centre while staying inside the bar span [-0.05,+0.046]",
        "allowed": True},
    "L_TIP": {
        "source": "debug-seed press calibration v5: eef z stalls 0.008-0.010 above the table plane, so the eef origin sits ~0.009 behind the fingertips along tool z",
        "allowed": True},
    "JAW_HALF": {
        "source": "debug-seed api.gripper(): open width 0.079 m, half-width 0.0395",
        "allowed": True},
    "Y_STAGE": {
        "source": "debug-seed cam_high: staging y=-0.145 sits in front of the cabinet face and clear of the tableware that snagged v11",
        "allowed": True},
    "PULL_STEP": {
        "source": "generic controller mechanics: incremental +y waypoints along the drawer's slide axis (drawer faces +y, from the cabinet face normal)",
        "allowed": True},
}

TABLE_Z = 0.901
BAR_R = 0.0075
L_TIP = 0.009
JAW_HALF = 0.0395
PITCH_DEG = 55.0
X_OFFSET = 0.020
Y_STAGE = -0.145
PULL_STEP = 0.035
N_PULL = 6


def frame(pitch_deg):
    p = np.deg2rad(pitch_deg)
    rot = np.array([[1.0, 0.0, 0.0],
                    [0.0, -np.cos(p), -np.sin(p)],
                    [0.0, np.sin(p), -np.cos(p)]])
    return rot, rot[:, 1], rot[:, 2]


def cloud(f):
    K = np.asarray(f.intrinsics)
    T = np.asarray(f.t_base_cam)
    dep = np.asarray(f.depth, dtype=np.float64)
    H, W = dep.shape
    vv, uu = np.mgrid[0:H, 0:W]
    X = (uu - K[0, 2]) * dep / K[0, 0]
    Y = (vv - K[1, 2]) * dep / K[1, 1]
    return np.stack([X, Y, dep], -1) @ T[:3, :3].T + T[:3, 3]


def find_bottom_handle(api):
    """Lowest of the cabinet's three bright handle bands -> (x_c, y_front, z_top)."""
    f = api.capture("cam_high")
    P = cloud(f)
    rgb = np.asarray(f.rgb).astype(np.float64)
    g = rgb.mean(-1)
    sat = rgb.max(-1) - rgb.min(-1)
    m = ((g > 120) & (sat < 30)
         & (P[..., 1] > -0.32) & (P[..., 1] < -0.15)
         & (P[..., 0] > -0.25) & (P[..., 0] < 0.25)
         & (P[..., 2] > TABLE_Z + 0.02) & (P[..., 2] < 1.12))
    pts = P[m]
    if len(pts) < 40:
        api.log("perception: only %d handle candidates" % len(pts))
        return None
    z = pts[:, 2]
    edges = np.arange(TABLE_Z + 0.02, 1.125, 0.005)
    hist, _ = np.histogram(z, bins=edges)
    occ = hist > 25
    groups = []
    i = 0
    while i < len(occ):
        if occ[i]:
            j = i
            while j + 1 < len(occ) and occ[j + 1]:
                j += 1
            groups.append((i, j))
            i = j + 1
        else:
            i += 1
    if not groups:
        api.log("perception: no handle band")
        return None
    a, b = groups[0]
    sel = pts[(z >= edges[a] - 0.004) & (z <= edges[b + 1] + 0.004)]
    return (float(np.median(sel[:, 0])),
            float(np.percentile(sel[:, 1], 97)),
            float(np.percentile(sel[:, 2], 97)))


def run(api):
    h = find_bottom_handle(api)
    if h is None:
        return
    x_c, y_front, z_top = h
    y_bar = y_front - BAR_R
    z_bar = z_top - BAR_R
    api.log("bottom handle x_c=%.4f y_front=%.4f z_top=%.4f -> bar centre (%.4f,%.4f)"
            % (x_c, y_front, z_top, y_bar, z_bar))

    rot, ty, tz = frame(PITCH_DEG)
    x = x_c + X_OFFSET
    # put the jaw centre on the bar; the -y reach saturates before this, so the
    # command is deliberately past the bar and the arm settles on its envelope.
    z_eef = z_bar - L_TIP * tz[2]
    y_cmd = y_bar - L_TIP * tz[1] - 0.05

    api.grip(0.08)
    api.settle(0.1)
    api.move([x, Y_STAGE, z_eef + 0.09], rotation=rot, seconds=0.5)
    api.move([x, Y_STAGE, z_eef], rotation=rot, seconds=0.4)
    for i in range(4):
        r = api.move([x, y_cmd, z_eef], rotation=rot, seconds=0.25)
        e = np.asarray(api.eef())
        jaw = e + L_TIP * tz
        d = np.array([0.0, y_bar - jaw[1], z_bar - jaw[2]])
        api.log("creep%d resid=%.4f eef=%s jaw=%s bar_along_tz=%+.4f bar_along_ty=%+.4f"
                % (i, r, np.round(e, 4), np.round(jaw, 4), float(d @ tz), float(d @ ty)))

    api.grip(0.0)
    api.settle(0.3)
    gs = api.gripper()
    api.log("CLOSED width=%.4f effort=%.2f" % (gs["width_m"], gs["effort"]))

    e = np.asarray(api.eef())
    for k in range(1, N_PULL + 1):
        api.move([x, e[1] + PULL_STEP * k, z_eef], rotation=rot, seconds=0.3)
    gs = api.gripper()
    api.log("after pull eef=%s width=%.4f effort=%.2f"
            % (np.round(api.eef(), 4), gs["width_m"], gs["effort"]))

    h2 = find_bottom_handle(api)
    if h2 is not None:
        api.log("VERIFY handle y_front %.4f -> %.4f  dy=%+.4f" % (y_front, h2[1], h2[1] - y_front))
