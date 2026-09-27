"""c2clean spa_bowl_next_to_ramekin_task_k0 -- zero demonstrations.

Intent: "Pick the akita black bowl next to the cookie box and place it on the plate"

Mechanism: the bowl is 0.115 m across and the jaws open to 0.080 m, so it cannot
be straddled.  It is rim-pinched instead -- the eef is parked one rim radius to
+y of the bowl centre, the open jaws are lowered 20 mm past the rim top so one
finger hangs inside the bowl and one outside its wall, and closing traps the
wall between them.  The bowl is then carried at a height that clears the plate
and lowered until the bowl's own base stalls the descent on the plate.

Every constant below is measured from debug seeds 51-65 (cam_high RGB-D, eef
readback, finger gap) or is generic controller mechanics.  See PROVENANCE.
"""
import numpy as np
from scipy import ndimage

PROVENANCE = {
    "TABLE_Z": {
        "source": "debug seeds 51-65 cam_high depth: the table plane is the dominant z "
                  "in the workspace point cloud at 0.900-0.905 m",
        "allowed": True},
    "RES": {
        "source": "generic: 1 cm cell for the top-down max-z occupancy grid",
        "allowed": True},
    "XLIM/YLIM": {
        "source": "debug seeds 51-65: the back wall deprojects to x=-1.99 and the props "
                  "all lie inside x[-0.42,+0.20], y[-0.36,+0.34]; this crop keeps the "
                  "wall and the robot base (x=-0.55) out of the grid",
        "allowed": True},
    "BOWL_ZLO": {
        "source": "debug seeds 51-65: the two bowls top out at 0.952-0.955, the ramekin "
                  "at 0.944, the plate and cookie box at 0.920; 0.948 sits in the 8 mm "
                  "gap above the ramekin",
        "allowed": True},
    "BOWL_ZHI": {
        "source": "debug seeds 51-65: the cabinet and its stove slab top out at "
                  "1.098-1.127, so 1.02 drops them",
        "allowed": True},
    "BOWL_WMIN/BOWL_WMAX": {
        "source": "debug seeds 51-65: both bowls measure 0.12 x 0.12 m in the 0.948 band "
                  "on 8/8 probe seeds; the only other survivor of that band is a "
                  "0.08 x 0.03 sliver of the stove edge, excluded by this window",
        "allowed": True},
    "FLAT_LO/FLAT_HI": {
        "source": "debug seeds 51-65: plate and cookie box both top out at 0.920 over a "
                  "0.905 table",
        "allowed": True},
    "BROWN_CUT": {
        "source": "debug seeds 51-65: cookie box mean rgb (105,78,61), R-B = 44; plate "
                  "mean rgb (166,157,153), R-B = 13",
        "allowed": True},
    "PLATE_MIN_N": {
        "source": "debug seeds 51-65: the plate is 141-147 cells of 1 cm in the flat band",
        "allowed": True},
    "WALL_HALF": {
        "source": "debug seeds 51-65 5 mm height map of the bowl: the rim ring is about "
                  "two 5 mm cells wide, so the wall mid-line is ~5 mm inside the outer "
                  "edge given by the footprint radius",
        "allowed": True},
    "TIP_OFF": {
        "source": "debug seeds 51 and 53 (v8p): a CLOSED gripper driven straight down "
                  "onto bare table stalls at eef_z = 0.9090 both times, so the fingertips "
                  "sit 0.004 m below the reported eef",
        "allowed": True},
    "GRASP_DEPTH": {
        "source": "debug seeds 51-65: bowl rim 0.952 over a 0.905 table = 47 mm deep, so "
                  "20 mm past the rim leaves the inner finger well clear of the bowl floor "
                  "while putting the wall between the jaws",
        "allowed": True},
    "Z_CARRY": {
        "source": "debug seeds 51-65: the tallest obstacle on the bowl->plate route is the "
                  "cookie box / plate at 0.920, and the bowl hangs ~27 mm below the "
                  "fingertips, so 1.02 clears it by ~70 mm",
        "allowed": True},
    "Z_PRESS": {
        "source": "debug seeds 51-65: plate rim 0.920, plate floor ~0.912; commanding the "
                  "eef below that makes the seated bowl -- not the tolerance band -- stop "
                  "the descent (measured stall 0.930-0.947)",
        "allowed": True},
    "POS_TOL_BIAS": {
        "source": "generic controller mechanics: move_cartesian breaks as soon as the 3-D "
                  "error is under POS_TOL = 0.012, so a descent stops up to 12 mm high; "
                  "the command is biased 12 mm low so the fingertips land at the intended "
                  "height",
        "allowed": True},
    "JAW_AXIS": {
        "source": "debug seed 51 reset frame: the blue gripper points in the cam_high "
                  "cloud form two lobes at y=-0.045 and y=+0.047 over a single x lobe, so "
                  "the fingers open along base y",
        "allowed": True},
    "PINCH_SIDE": {
        "source": "debug seeds 51-65: the cabinet (z > 1.05) reaches y = -0.18 at the "
                  "target bowl's x, only ~45 mm outside its -y rim, so the pinch is taken "
                  "on the +y arc; the cookie box on that side is 0.920 tall, below the "
                  "0.932 fingertip height, and ends 17 mm short of the bowl's x",
        "allowed": True},
    "MOVE_BUDGET": {
        "source": "generic controller mechanics (heron/robot/libero.py, tools/fair_run.py): "
                  "action = clip(err/0.05, -1, 1), move_cartesian runs max(40, 120*seconds) "
                  "steps, api.grip costs 20 steps, the episode horizon is 1000; measured "
                  "free-air travel is ~1.2 mm/step, so seconds is sized per leg. "
                  "api.move(rotation=...) is never used: the reset wrist is 3.25 deg off "
                  "vertical and ROT_TOL/ROT_GAIN make move_pose burn its whole step cap "
                  "without converging (measured in v3/v4p)",
        "allowed": True},
}

# ---- scene constants (debug seeds 51-65) -----------------------------------
TABLE_Z = 0.905
RES = 0.01
XLIM = (-0.45, 0.35)
YLIM = (-0.45, 0.45)
BOWL_ZLO, BOWL_ZHI = 0.948, 1.02
BOWL_WMIN, BOWL_WMAX = 0.09, 0.16
FLAT_LO, FLAT_HI = 0.9085, 0.9235
BROWN_CUT = 25
PLATE_MIN_N = 60

# ---- grasp constants -------------------------------------------------------
TIP_OFF = 0.004
GRASP_DEPTH = 0.020
WALL_HALF = 0.005
Z_CARRY = 1.02
Z_PRESS = 0.910
POS_TOL_BIAS = 0.012


# ---------------------------------------------------------------- perception
def heightmap(api, cam="cam_high"):
    """Top-down max-z grid plus a mean-colour grid, both in the base frame."""
    f = api.capture(cam)
    d = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]
    cam_pts = np.stack([(uu - K[0, 2]) / K[0, 0] * d,
                        (vv - K[1, 2]) / K[1, 1] * d, d], -1)
    P = (cam_pts @ T[:3, :3].T + T[:3, 3]).reshape(-1, 3)
    C = np.asarray(f.rgb, float).reshape(-1, 3)
    m = ((P[:, 0] > XLIM[0]) & (P[:, 0] < XLIM[1]) &
         (P[:, 1] > YLIM[0]) & (P[:, 1] < YLIM[1]) &
         (P[:, 2] > 0.80) & (P[:, 2] < 1.30))
    P, C = P[m], C[m]
    nx = int(round((XLIM[1] - XLIM[0]) / RES))
    ny = int(round((YLIM[1] - YLIM[0]) / RES))
    gx = np.clip(((P[:, 0] - XLIM[0]) / RES).astype(int), 0, nx - 1)
    gy = np.clip(((P[:, 1] - YLIM[0]) / RES).astype(int), 0, ny - 1)
    idx = gx * ny + gy
    Z = np.full(nx * ny, -1.0)
    np.maximum.at(Z, idx, P[:, 2])
    acc = np.zeros((nx * ny, 3))
    cnt = np.zeros(nx * ny)
    np.add.at(acc, idx, C)
    np.add.at(cnt, idx, 1)
    col = acc / np.maximum(cnt, 1)[:, None]
    return Z.reshape(nx, ny), col.reshape(nx, ny, 3)


def clusters(Z, col, zlo, zhi, min_n=6):
    lab, n = ndimage.label((Z > zlo) & (Z < zhi), np.ones((3, 3)))
    out = []
    for i in range(1, n + 1):
        s = lab == i
        if s.sum() < min_n:
            continue
        xs, ys = np.nonzero(s)
        out.append(dict(n=int(s.sum()),
                        cx=XLIM[0] + (xs.mean() + 0.5) * RES,
                        cy=YLIM[0] + (ys.mean() + 0.5) * RES,
                        wx=(xs.max() - xs.min() + 1) * RES,
                        wy=(ys.max() - ys.min() + 1) * RES,
                        ztop=float(Z[s].max()), rgb=col[s].mean(0)))
    out.sort(key=lambda r: -r["n"])
    return out


def perceive(api, tag=""):
    Z, col = heightmap(api)
    bowls = [r for r in clusters(Z, col, BOWL_ZLO, BOWL_ZHI)
             if BOWL_WMIN <= r["wx"] <= BOWL_WMAX and BOWL_WMIN <= r["wy"] <= BOWL_WMAX]
    flats = clusters(Z, col, FLAT_LO, FLAT_HI, min_n=8)
    brown = [r for r in flats if r["rgb"][0] - r["rgb"][2] > BROWN_CUT]
    pale = [r for r in flats
            if r["rgb"][0] - r["rgb"][2] <= BROWN_CUT and r["n"] >= PLATE_MIN_N]
    api.log(f"P{tag} bowls=" + str([(round(r["cx"], 3), round(r["cy"], 3),
                                     round(r["ztop"], 3)) for r in bowls]))
    api.log(f"P{tag} box=" + str([(round(r["cx"], 3), round(r["cy"], 3)) for r in brown[:1]])
            + " plate=" + str([(round(r["cx"], 3), round(r["cy"], 3)) for r in pale[:1]]))
    return bowls, brown, pale


# ---------------------------------------------------------------- motion
def go(api, xyz, seconds=1.0, tag=""):
    res = api.move([float(v) for v in xyz], seconds=seconds)
    e = np.asarray(api.eef(), float)
    api.log(f"MOVE {tag} want={np.round(xyz, 4).tolist()} got={np.round(e, 4).tolist()} "
            f"res={res:.4f}")
    return e


# ---------------------------------------------------------------- task
def run(api):
    api.log(f"INSTR {api.instruction()}")
    bowls, brown, pale = perceive(api, "0")
    if not bowls or not brown or not pale:
        api.log("ABORT perception")
        return
    box, plate = brown[0], pale[0]

    # "the bowl next to the cookie box": of the two bowls, the nearer one to the
    # brown box.  Measured separation on the debug seeds: 0.11-0.13 m vs 0.38 m.
    tgt = min(bowls, key=lambda r: np.hypot(r["cx"] - box["cx"], r["cy"] - box["cy"]))
    bx, by, ztop = tgt["cx"], tgt["cy"], tgt["ztop"]
    rim_r = 0.25 * (tgt["wx"] + tgt["wy"])
    off = rim_r - WALL_HALF          # eef offset from the bowl centre to the wall
    api.log(f"TGT ({bx:+.3f},{by:+.3f}) ztop={ztop:.3f} rim_r={rim_r:.4f} off={off:.4f} "
            f"d_box={np.hypot(bx - box['cx'], by - box['cy']):.3f}")

    gx, gy = bx, by + off
    z_grasp = ztop - GRASP_DEPTH + TIP_OFF

    go(api, [gx, gy, Z_CARRY], seconds=3.0, tag="approach")
    go(api, [gx, gy, z_grasp - POS_TOL_BIAS], seconds=1.2, tag="down")
    api.log(f"G_pre {api.gripper()}")
    api.grip(0.0)
    api.log(f"G_closed {api.gripper()}")
    go(api, [gx, gy, Z_CARRY], seconds=1.2, tag="lift")
    api.log(f"G_lifted {api.gripper()}")

    # the bowl centre trails the eef by the full offset, so aim the eef the same
    # offset past the plate centre
    px, py = plate["cx"], plate["cy"]
    go(api, [px, py + off, Z_CARRY], seconds=3.0, tag="carry")
    api.log(f"G_carried {api.gripper()}")
    e = go(api, [px, py + off, Z_PRESS], seconds=1.0, tag="place")
    api.log(f"G_atplace {api.gripper()} stall_z={e[2]:.4f}")
    api.grip(0.08)
    go(api, [px, py + off, Z_CARRY], seconds=0.8, tag="retreat")
    perceive(api, "F")
