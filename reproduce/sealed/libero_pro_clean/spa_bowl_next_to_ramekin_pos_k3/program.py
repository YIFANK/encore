"""v2 -- perceive the scene from cam_high, rim-pinch the bowl that sits next to
the ramekin, carry it to the plate and release.

Mechanism (all of it derived from the pack + debug-seed observations, see
PROVENANCE):
  * top-down height map from the deprojected cam_high cloud, referenced to the
    table plane (median of the map).
  * height bands separate the props: bowls cap ~50 mm, the ramekin ~42 mm, the
    plate ~18 mm.  Colour splits the ramekin (bright achromatic) from a bowl
    wall and the plate from the brown box.
  * target = the bowl-class cluster nearest the ramekin (the instruction).
  * the pack grasps with the home wrist and an offset of ~0.05 m from the bowl
    centre along the direction pointing away from the ramekin; the release
    point carries the same offset relative to the plate centre.
"""

import numpy as np

PROVENANCE = {
    "GRID": {"source": "own choice of workspace raster for the cam_high point cloud (generic perception mechanics)", "allowed": True},
    "H_BOWL_MIN": {"source": "debug seeds 51/53/55/57: bowl-class clusters cap at 50 mm above the table plane, the ramekin at 42 mm", "allowed": True},
    "H_RAM": {"source": "same four debug seeds: ramekin top band 30-46 mm, footprint 0.08x0.09 m, mean RGB ~145 achromatic", "allowed": True},
    "H_PLATE": {"source": "same four debug seeds: plate top 18 mm, footprint 0.13-0.14 m, mean RGB ~(147,139,135); the brown box at the same height is (90,55,33)", "allowed": True},
    "FOOTPRINT": {"source": "debug-seed measurement of the bowl (0.11x0.11), ramekin (0.08x0.09) and plate (0.13x0.14) footprints", "allowed": True},
    "RIM_OFFSET": {"source": "pack keyframes: grasp EEF minus the bowl centre deprojected from demo0_t0000 = (0.017,0.055); release EEF minus the plate centre = (0.035,0.023)/(0.008,0.028)/(0.021,0.042)", "allowed": True},
    "GRASP_DZ": {"source": "pack keyframes: closing EEF z = 0.932/0.917/0.9215 against a table plane measured at 0.9013 on debug seeds -> ~0.020 m", "allowed": True},
    "RELEASE_DZ": {"source": "pack keyframes: opening EEF z = 0.945/0.9446/0.9322 -> ~0.044 m above the table plane", "allowed": True},
    "CARRY_Z": {"source": "pack ee_path: the carry apex rides ~0.17-0.20 m above the table plane", "allowed": True},
    "GRIP_HOLD_GAP": {"source": "pack gripper_state at the grasp keyframes: finger gap 0.008-0.013 m while carrying", "allowed": True},
}

X0, Y0, D = -0.45, -0.45, 0.01
NX, NY = 85, 90

H_BOWL_MIN = 0.046
H_RAM_LO, H_RAM_HI = 0.030, 0.046
H_PLATE_LO, H_PLATE_HI = 0.010, 0.026

RIM_OFFSET = 0.048
GRASP_DZ = 0.020
RELEASE_DZ = 0.050
CARRY_DZ = 0.18
APPROACH_DZ = 0.15


# ---------------------------------------------------------------- perception
def height_map(api):
    f = api.capture("cam_high")
    H, W = f.depth.shape[:2]
    K = f.intrinsics
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:H, 0:W]
    z = np.asarray(f.depth, float)
    ok = np.isfinite(z) & (z > 0)
    pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z, np.ones_like(z)], -1)
    P = pc @ f.t_base_cam.T
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    rgb = np.asarray(f.rgb, float)
    ix = np.floor((X - X0) / D).astype(int)
    iy = np.floor((Y - Y0) / D).astype(int)
    good = ok & (ix >= 0) & (ix < NX) & (iy >= 0) & (iy < NY)
    top = np.full((NX, NY), -9.0)
    col = np.zeros((NX, NY, 3))
    gi, gj, gz = ix[good], iy[good], Z[good]
    o = np.argsort(gz)
    gi, gj, gz = gi[o], gj[o], gz[o]
    top[gi, gj] = gz
    col[gi, gj] = rgb[good][o]
    z0 = float(np.median(top[top > -8]))
    return top - z0, col, z0, (top > -8)


def comps(M):
    lab = np.zeros(M.shape, int)
    n = 0
    for i in range(M.shape[0]):
        for j in range(M.shape[1]):
            if M[i, j] and lab[i, j] == 0:
                n += 1
                st = [(i, j)]
                lab[i, j] = n
                while st:
                    a, b = st.pop()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            x, y = a + da, b + db
                            if 0 <= x < M.shape[0] and 0 <= y < M.shape[1] \
                                    and M[x, y] and lab[x, y] == 0:
                                lab[x, y] = n
                                st.append((x, y))
    return lab, n


def clusters(hgt, col, valid, lo, hi, nmin=8):
    M = valid & (hgt > lo) & (hgt < hi)
    lab, n = comps(M)
    out = []
    for k in range(1, n + 1):
        ii, jj = np.where(lab == k)
        if len(ii) < nmin:
            continue
        x = X0 + D * (ii + 0.5)
        y = Y0 + D * (jj + 0.5)
        out.append(dict(n=len(ii), cx=float(x.mean()), cy=float(y.mean()),
                        w=float(x.max() - x.min()), l=float(y.max() - y.min()),
                        hmax=float(hgt[ii, jj].max()),
                        rgb=col[ii, jj].mean(0)))
    return out


def describe(tag, c):
    return "%s n=%d c=(%+.3f,%+.3f) w=%.3f l=%.3f hmax=%.3f rgb=%s" % (
        tag, c["n"], c["cx"], c["cy"], c["w"], c["l"], c["hmax"],
        np.round(c["rgb"]).astype(int).tolist())


def perceive(api):
    hgt, col, z0, valid = height_map(api)
    api.log("table_z=%.4f" % z0)

    bowls = [c for c in clusters(hgt, col, valid, H_BOWL_MIN, 0.09, 20)
             if 0.080 <= c["w"] <= 0.140 and 0.080 <= c["l"] <= 0.140]
    mids = clusters(hgt, col, valid, H_RAM_LO, H_RAM_HI, 20)
    flats = clusters(hgt, col, valid, H_PLATE_LO, H_PLATE_HI, 30)
    for c in bowls:
        api.log(describe("BOWL", c))
    for c in mids:
        api.log(describe("MID ", c))
    for c in flats:
        api.log(describe("FLAT", c))

    rams = [c for c in mids
            if 0.055 <= c["w"] <= 0.115 and 0.055 <= c["l"] <= 0.115
            and c["rgb"].mean() > 125
            and abs(c["rgb"][0] - c["rgb"][2]) < 25]
    plates = [c for c in flats
              if 0.100 <= c["w"] <= 0.180 and 0.100 <= c["l"] <= 0.180
              and c["rgb"].mean() > 120
              and abs(c["rgb"][0] - c["rgb"][2]) < 30]
    return z0, bowls, rams, plates


# -------------------------------------------------------------------- motion
def run(api):
    api.log("instruction=%r" % api.instruction())
    R = np.asarray(api.tool_rotation(), float)
    api.log("R_home=%s" % np.round(R, 3).tolist())

    z0, bowls, rams, plates = perceive(api)
    if not bowls or not plates:
        api.log("ABORT: bowls=%d plates=%d" % (len(bowls), len(plates)))
        return "no target"

    if rams:
        ram = max(rams, key=lambda c: c["n"])
        tgt = min(bowls, key=lambda c: (c["cx"] - ram["cx"]) ** 2
                  + (c["cy"] - ram["cy"]) ** 2)
        api.log("ramekin=(%+.3f,%+.3f)" % (ram["cx"], ram["cy"]))
    else:
        ram = None
        tgt = max(bowls, key=lambda c: c["cy"])
        api.log("ramekin=NONE -> fallback: highest-y bowl")
    plate = max(plates, key=lambda c: c["n"])
    api.log("target_bowl=(%+.3f,%+.3f) plate=(%+.3f,%+.3f)"
            % (tgt["cx"], tgt["cy"], plate["cx"], plate["cy"]))

    # rim-pinch offset: away from the ramekin, but along the world y axis --
    # the pack grasps with the home wrist, whose jaws close along y.
    if ram is not None and abs(tgt["cy"] - ram["cy"]) > 1e-6:
        sy = 1.0 if tgt["cy"] > ram["cy"] else -1.0
    else:
        sy = 1.0
    off = np.array([0.0, sy * RIM_OFFSET])
    api.log("offset=%s" % off.round(3).tolist())

    gx, gy = tgt["cx"] + off[0], tgt["cy"] + off[1]

    api.grip(0.08)
    r = api.move([gx, gy, z0 + APPROACH_DZ], rotation=R, seconds=3.0)
    api.log("pre-grasp resid=%.4f eef=%s" % (r, np.round(api.eef(), 3).tolist()))
    r = api.move([gx, gy, z0 + GRASP_DZ], rotation=R, seconds=3.0)
    api.log("descend resid=%.4f eef=%s" % (r, np.round(api.eef(), 3).tolist()))

    api.grip(0.0)
    api.settle(0.3)
    g = api.gripper()
    api.log("closed gap=%.4f effort=%.2f" % (g["width_m"], g["effort"]))

    r = api.move([gx, gy, z0 + CARRY_DZ], rotation=R, seconds=3.0)
    g = api.gripper()
    api.log("lifted resid=%.4f gap=%.4f effort=%.2f eef=%s"
            % (r, g["width_m"], g["effort"], np.round(api.eef(), 3).tolist()))

    px, py = plate["cx"] + off[0], plate["cy"] + off[1]
    r = api.move([px, py, z0 + CARRY_DZ], rotation=R, seconds=3.0)
    g = api.gripper()
    api.log("over-plate resid=%.4f gap=%.4f effort=%.2f" % (r, g["width_m"], g["effort"]))
    r = api.move([px, py, z0 + RELEASE_DZ], rotation=R, seconds=3.0)
    g = api.gripper()
    api.log("at-release resid=%.4f gap=%.4f effort=%.2f eef=%s"
            % (r, g["width_m"], g["effort"], np.round(api.eef(), 3).tolist()))

    api.grip(0.08)
    api.settle(0.5)
    api.move([px, py, z0 + CARRY_DZ], rotation=R, seconds=2.0)
    api.settle(0.5)

    # post-hoc re-perception: where did the bowl end up?
    z1, bowls2, rams2, plates2 = perceive(api)
    for c in bowls2:
        api.log("POST " + describe("BOWL", c))
    return "v2 done"
