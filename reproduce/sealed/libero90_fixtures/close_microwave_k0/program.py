"""l90abl close_microwave_k0 -- v3: perceive the open door, then sweep it shut.

Scene model, derived entirely from my own debug-seed (51-65) cam_high RGB-D:
  * The microwave is the only strongly blue-dominant body on the table.
    Its top face sits at z = 1.108; the table plane is z = 0.90.
  * The body footprint is fixed across all 15 debug seeds
    (x up to ~0.17, y up to ~0.395); only the door angle changes.
  * The door is a thin vertical panel.  Its top face shows up as a ~6 mm-wide
    strip in the z = 1.108 band, sticking out of the body toward y < 0.2.
  * Fitting that strip on each of the 15 debug seeds and least-squares
    intersecting the 15 lines puts the hinge at (-0.178, 0.265); the door
    tip radius is ~0.266 m and the open angle ranges over -76 deg .. -117 deg
    (angle measured from the hinge, 0 deg = +x = door shut against the front
    face, which faces -y).
Strategy: close the gripper, drop it behind the door (on the side the door
must travel away from), then walk the tool along a circular arc about the
hinge from the measured open angle round to a small positive overshoot,
dragging the door shut.
"""
import numpy as np

PROVENANCE = {
    "BLUE_MARGIN": {
        "source": "debug seeds 51-65 cam_high RGB: the microwave shell is the only surface with b>r+20 and b>g+20 above the table",
        "allowed": True},
    "Z_TABLE": {
        "source": "debug seed 51 cam_high depth histogram: dominant plane at z=0.900",
        "allowed": True},
    "Z_ROBOT_CUT": {
        "source": "debug seeds 51-65: microwave blue tops out at z=1.108, the robot's blue links start at z~1.28; 1.15 separates them",
        "allowed": True},
    "TOP_BAND": {
        "source": "debug seeds 51-65: thickness of the z-slice that isolates the top faces of door+body",
        "allowed": True},
    "DOOR_Y_CUT": {
        "source": "debug seeds 51-65: body front edge measured at y~0.265, door strip always reaches y<0.02; 0.20 separates them",
        "allowed": True},
    "HINGE0": {
        "source": "debug seeds 51-65: least-squares intersection of the 15 fitted door-strip lines = (-0.178, 0.265), per-line residual <= 11 mm",
        "allowed": True},
    "PUSH_R": {
        "source": "debug seeds 51-65: door tip radius measured 0.254-0.273 m; contact at 0.20 m keeps the tool well inside the panel",
        "allowed": True},
    "PUSH_Z": {
        "source": "debug seeds 51-65: door panel spans z 0.93-1.108 and the tallest other prop (a mug) tops out at z=1.013; 1.05 is on the panel and clear of the mugs",
        "allowed": True},
    "HOVER_Z": {
        "source": "debug seed 51: microwave top face z=1.108; 1.20 clears it",
        "allowed": True},
    "DELTA_CLEAR": {
        "source": "debug seed 51: door-strip half width ~0.005 m; 0.05 m stand-off clears the panel for the descent",
        "allowed": True},
    "DELTA_PUSH": {
        "source": "debug seed 51: door-strip half width ~0.005 m; commanding the tool onto the panel centreline guarantees contact",
        "allowed": True},
    "END_ANGLE": {
        "source": "geometry: 0 deg is the shut pose; a small positive overshoot seats the door",
        "allowed": True},
    "ARC_STEPS": {
        "source": "generic controller mechanics: arc chord per step kept near 0.05 m",
        "allowed": True},
}

BLUE_MARGIN = 20.0
Z_TABLE = 0.90
Z_ROBOT_CUT = 1.15
TOP_BAND = 0.020
DOOR_Y_CUT = 0.20
HINGE0 = np.array([-0.178, 0.265])
PUSH_R = 0.20
PUSH_Z = 1.05
HOVER_Z = 1.20
DELTA_CLEAR = 0.05
DELTA_PUSH = 0.008
END_ANGLE = 8.0
ARC_STEPS = 8


def _cloud(frame):
    d = np.asarray(frame.depth, dtype=np.float64)
    K = np.asarray(frame.intrinsics, dtype=np.float64)
    T = np.asarray(frame.t_base_cam, dtype=np.float64)
    H, W = d.shape[:2]
    vv, uu = np.mgrid[0:H, 0:W]
    ok = np.isfinite(d) & (d > 0.05) & (d < 5.0)
    z = np.where(ok, d, 1.0)
    xc = (uu - K[0, 2]) * z / K[0, 0]
    yc = (vv - K[1, 2]) * z / K[1, 1]
    P = np.einsum("ij,jhw->ihw", T[:3, :3], np.stack([xc, yc, z])) + T[:3, 3].reshape(3, 1, 1)
    return P, ok


def perceive_door(api):
    """Return (hinge_xy, angle_deg, tip_xy) for the open microwave door."""
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb).astype(np.float64)
    P, ok = _cloud(f)
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    blue = ((b > r + BLUE_MARGIN) & (b > g + BLUE_MARGIN) & ok
            & (P[2] > Z_TABLE + 0.03) & (P[2] < Z_ROBOT_CUT))
    n = int(blue.sum())
    if n < 200:
        api.log("perceive: only %d blue px" % n)
        return None
    ztop = float(np.percentile(P[2][blue], 99.5))
    band = blue & (P[2] > ztop - TOP_BAND)
    bx, by = P[0][band], P[1][band]
    api.log("perceive: blue=%d ztop=%.4f band=%d" % (n, ztop, band.sum()))

    ycut = DOOR_Y_CUT
    for _ in range(4):
        sel = by < ycut
        if sel.sum() >= 60:
            break
        ycut += 0.03
    if sel.sum() < 60:
        api.log("perceive: door strip too small (%d, ycut=%.2f)" % (sel.sum(), ycut))
        return None
    D = np.stack([bx[sel], by[sel]], axis=1)
    c = D.mean(axis=0)
    _, s, Vt = np.linalg.svd(D - c, full_matrices=False)
    u = Vt[0]
    width = float(s[1] / np.sqrt(len(D)))
    t = (D - c) @ u
    e1, e2 = c + u * t.min(), c + u * t.max()
    tip = e1 if e1[1] < e2[1] else e2
    hinge = c + u * float((HINGE0 - c) @ u)
    rv = tip - hinge
    ang = float(np.degrees(np.arctan2(rv[1], rv[0])))
    api.log("perceive: n=%d width=%.4f tip=(%.3f,%.3f) hinge=(%.3f,%.3f) R=%.3f ang=%.1f"
            % (len(D), width, tip[0], tip[1], hinge[0], hinge[1], np.linalg.norm(rv), ang))
    if ang > -20.0 or ang < -160.0:
        api.log("perceive: angle out of expected range")
    return hinge, ang, tip


def tool_xy(hinge, ang_deg, radius, delta):
    a = np.radians(ang_deg)
    ca, sa = np.cos(a), np.sin(a)
    # radial unit vector along the panel, and the unit vector pointing to the
    # side of the panel the tool must sit on to drive it shut (angle -> 0).
    return hinge + radius * np.array([ca, sa]) + delta * np.array([sa, -ca])


def run(api):
    api.log("instruction: %r" % (api.instruction(),))
    api.grip(0.0)
    api.settle(0.2)

    seen = perceive_door(api)
    if seen is None:
        api.log("abort: no door")
        return
    hinge, ang0, tip = seen

    start = tool_xy(hinge, ang0, PUSH_R, DELTA_CLEAR)
    api.move([start[0], start[1], HOVER_Z], seconds=2.0)
    api.move([start[0], start[1], PUSH_Z], seconds=1.5)

    angles = list(np.linspace(ang0, END_ANGLE, ARC_STEPS + 1))
    lead = tool_xy(hinge, ang0, PUSH_R, DELTA_PUSH)
    api.move([lead[0], lead[1], PUSH_Z], seconds=1.0)
    for a in angles[1:]:
        p = tool_xy(hinge, a, PUSH_R, DELTA_PUSH)
        res = api.move([p[0], p[1], PUSH_Z], seconds=1.0)
        api.log("arc a=%.1f -> (%.3f,%.3f) eef=%s res=%s"
                % (a, p[0], p[1], np.asarray(api.eef()).round(3).tolist(), res))

    api.settle(0.3)
    after = perceive_door(api)
    if after is not None:
        api.log("final angle=%.1f (was %.1f)" % (after[1], ang0))

    back = tool_xy(hinge, END_ANGLE, PUSH_R, 0.10)
    api.move([back[0], back[1], PUSH_Z], seconds=1.0)
    api.move([back[0], back[1], HOVER_Z], seconds=1.0)
