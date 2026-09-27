"""v3 -- park, perceive, pick the wine bottle by its neck, place it on the plate.

Perception was developed offline against the v2 parked RGB-D dumps of debug
seeds 51-65; every constant below is declared in PROVENANCE.
"""
import base64
import json
import zlib

import numpy as np

PROVENANCE = {
    "PARK_XYZ": {
        "source": "debug seeds 51-58 (fs_..._v1b): at reset the arm sits at "
                  "eef (-0.208, 0.0, 1.173), on the camera's line to the tall "
                  "dark prop at y~-0.05, and the two fuse into one height-band "
                  "cluster. (-0.10, 0.30, 1.25) clears that line; verified on "
                  "all 15 debug seeds in fs_..._v2.",
        "allowed": True},
    "ROI": {
        "source": "debug seeds 51-65 (fs_..._v2) cam_high point cloud: the "
                  "tabletop props live in x(-0.32,0.30) y(-0.14,0.17); the "
                  "cabinet is at y<-0.13 and the parked gripper at y~0.30.",
        "allowed": True},
    "DARK_RGB": {
        "source": "debug seeds 51-65 (fs_..._v2): the tall prop renders at "
                  "rgb ~(8,11,7); the cabinet's right face, the only other "
                  "thing in the band, renders at ~(50,50,50).",
        "allowed": True},
    "TALL_BAND": {
        "source": "debug seeds 51-65 (fs_..._v2): the tall prop's dark glass "
                  "tops out 0.147-0.150 above the table; the next tallest "
                  "thing inside the ROI is the bowl rim at +0.051.",
        "allowed": True},
    "GREY_FLAT": {
        "source": "debug seeds 51-65 (fs_..._v2): the target disc is "
                  "unsaturated (R-B<16) and bright (R>110) and lies within "
                  "0.026 of the table; the wood table has R-B ~30.",
        "allowed": True},
    "PLATE_R": {
        "source": "debug seeds 51-65 (fs_..._v2): the disc measures "
                  "0.131-0.143 m across in both axes, so a 0.060 m voting "
                  "disc and a 0.075 m trim radius bracket it.",
        "allowed": True},
    "GRASP_DROP": {
        "source": "packs/c2k1clean_goal_put_bowl_on_plate_task_mate demo0: the "
                  "wine bottle is grasped with the eef at z 1.0291 (close) / "
                  "1.0223 (settled). Debug seeds put the same prop's dark top "
                  "at 1.047-1.050, i.e. the demo grasps 0.024-0.027 below it.",
        "allowed": True},
    "FINGER_DROP": {
        "source": "debug seeds 51-65 (fs_..._v2): with the eef parked at "
                  "z 1.2595 the lowest deprojected gripper point is 1.253, so "
                  "the eef reference sits ~0.006 above the fingertips.",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug seeds 51-65 (fs_..._v2): the tallest obstacle on the "
                  "straight line from prop to disc is the bowl rim at 0.952; "
                  "carrying at eef 1.20 puts the held base at ~1.08.",
        "allowed": True},
    "HOVER": {
        "source": "debug-seed geometry: 0.09 above the prop top clears the "
                  "0.010 cap that sits above the dark glass.",
        "allowed": True},
}

PARK_XYZ = (-0.10, 0.30, 1.25)
ROI = dict(xlo=-0.32, xhi=0.30, ylo=-0.14, yhi=0.17)
DARK_RGB = (35.0, 45.0, 35.0)
TALL_BAND = 0.09
GREY_FLAT = dict(rb=16.0, r=110.0, zhi=0.026, zlo=-0.01)
PLATE_VOTE_R = 0.060
PLATE_TRIM_R = 0.075
GRASP_DROP = 0.025
CARRY_Z = 1.20
HOVER = 0.09


# --------------------------------------------------------------------------
def _cloud(f, step=2):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    dep = np.asarray(f.depth, float)[::step, ::step]
    rgb = np.asarray(f.rgb)[::step, ::step]
    h, w = dep.shape
    vv, uu = np.mgrid[0:h, 0:w]
    uf, vf = uu * step, vv * step
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    z = dep
    P = np.stack([(uf - cx) * z / fx, (vf - cy) * z / fy, z, np.ones_like(z)], -1) @ T.T
    return P[..., :3], rgb, np.isfinite(z) & (z > 0)


def _perceive(P, rgb, ok):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    R, G, B = (rgb[..., i].astype(float) for i in range(3))
    out = {}
    win = ok & (X > -0.30) & (X < 0.30) & (Y > -0.30) & (Y < 0.30)
    hist, edges = np.histogram(Z[win], bins=200, range=(0.80, 1.05))
    table = float(edges[int(np.argmax(hist))] + 0.000625)
    out["table"] = table

    roi = ok & (X > ROI["xlo"]) & (X < ROI["xhi"]) & (Y > ROI["ylo"]) & (Y < ROI["yhi"])

    # --- the tall dark prop the instruction names
    dark = (R < DARK_RGB[0]) & (G < DARK_RGB[1]) & (B < DARK_RGB[2])
    m = roi & dark & (Z > table + TALL_BAND)
    out["n_tall"] = int(m.sum())
    out["obj"] = None
    if m.sum() >= 20:
        x, y, z = X[m], Y[m], Z[m]
        cx, cy = float(np.median(x)), float(np.median(y))
        for _ in range(4):
            keep = np.hypot(x - cx, y - cy) < 0.05
            if keep.sum() < 10:
                break
            x, y, z = x[keep], y[keep], z[keep]
            cx, cy = float(np.median(x)), float(np.median(y))
        r = float((y.max() - y.min()) / 2)
        out["obj"] = (float((x.min() + x.max() - r) / 2),
                      float((y.min() + y.max()) / 2), float(z.max()))
        out["obj_r"] = r
        out["obj_n"] = int(len(x))

    # --- the flat grey disc the instruction names as the destination
    grey = (R - B < GREY_FLAT["rb"]) & (R > GREY_FLAT["r"])
    m = roi & grey & (Z > table + GREY_FLAT["zlo"]) & (Z < table + GREY_FLAT["zhi"])
    out["n_flat"] = int(m.sum())
    out["dst"] = None
    if m.sum() >= 100:
        x, y, z = X[m], Y[m], Z[m]
        res, x0, y0, nx, ny = 0.005, -0.30, -0.16, 120, 64
        gi = np.clip(((x - x0) / res).astype(int), 0, nx - 1)
        gj = np.clip(((y - y0) / res).astype(int), 0, ny - 1)
        grid = np.zeros((nx, ny))
        np.add.at(grid, (gi, gj), 1.0)
        rad = int(round(PLATE_VOTE_R / res))
        acc = np.zeros_like(grid)
        for di in range(-rad, rad + 1):
            for dj in range(-rad, rad + 1):
                if di * di + dj * dj > rad * rad:
                    continue
                acc[max(0, di):nx + min(0, di), max(0, dj):ny + min(0, dj)] += \
                    grid[max(0, -di):nx + min(0, -di), max(0, -dj):ny + min(0, -dj)]
        bi, bj = np.unravel_index(int(np.argmax(acc)), acc.shape)
        cx, cy = x0 + (bi + .5) * res, y0 + (bj + .5) * res
        keep = np.hypot(x - cx, y - cy) < PLATE_TRIM_R
        x, y, z = x[keep], y[keep], z[keep]
        cx = float((x.min() + x.max()) / 2)
        cy = float((y.min() + y.max()) / 2)
        c = np.hypot(x - cx, y - cy) < 0.02
        floor = float(np.median(z[c])) if c.sum() > 5 else float(np.percentile(z, 10))
        out["dst"] = (cx, cy, floor)
        out["dst_d"] = (float(x.max() - x.min()), float(y.max() - y.min()))
        out["dst_n"] = int(len(x))
    return out


def _dump(api, tag, f, step=2):
    rgb = np.asarray(f.rgb)[::step, ::step]
    dep = np.asarray(f.depth, dtype=np.float32)[::step, ::step].astype(np.float16)
    api.log(f"{tag}.meta " + json.dumps({
        "rgb_shape": list(rgb.shape), "dep_shape": list(dep.shape), "step": step,
        "K": np.asarray(f.intrinsics).tolist(), "T": np.asarray(f.t_base_cam).tolist()}))
    for name, arr in (("rgb", rgb), ("dep", dep)):
        b = base64.b64encode(zlib.compress(arr.tobytes(), 6)).decode()
        for i in range(0, len(b), 1900):
            api.log(f"{tag}.{name}.chunk {i // 1900} {b[i:i + 1900]}")
        api.log(f"{tag}.{name}.end {len(b)}")


def _state(api, tag):
    e = [round(float(v), 4) for v in api.eef()]
    g = api.gripper()
    api.log(f"{tag} eef {e} grip {round(g['width_m'], 4)} eff {g['effort']}")
    return np.asarray(e), g


# --------------------------------------------------------------------------
def run(api):
    api.log("instruction " + repr(api.instruction()))
    api.grip(0.08)
    api.move(list(PARK_XYZ), seconds=3.0)
    api.settle(0.4)
    f = api.capture("cam_high")
    _dump(api, "park", f)
    s = _perceive(*_cloud(f))
    api.log("perceive " + json.dumps({k: (list(np.round(v, 4)) if isinstance(v, tuple) else v)
                                      for k, v in s.items()}))
    if s["obj"] is None or s["dst"] is None:
        api.log("ABORT: perception failed")
        return
    ox, oy, otop = s["obj"]
    dx, dy, dfloor = s["dst"]
    table = s["table"]
    grasp_z = otop - GRASP_DROP
    hang = grasp_z - table                      # eef height above the held base
    release_z = dfloor + hang
    api.log(f"plan grasp ({ox:.3f},{oy:.3f},{grasp_z:.3f}) hang {hang:.3f} "
            f"release ({dx:.3f},{dy:.3f},{release_z:.3f})")

    api.move([ox, oy, otop + HOVER], seconds=3.0)
    _state(api, "hover")
    r = api.move([ox, oy, grasp_z], seconds=2.0)
    api.log(f"descend residual {r:.4f}")
    _state(api, "at_grasp")
    api.grip(0.0)
    _, g = _state(api, "closed")

    api.move([ox, oy, CARRY_Z], seconds=2.5)
    _, g = _state(api, "lifted")
    if g["effort"] < 1.0:
        api.log("ABORT: nothing held after the lift")
        return

    api.move([dx, dy, CARRY_Z], seconds=3.0)
    _state(api, "over_dst")
    api.move([dx, dy, release_z + 0.03], seconds=2.0)
    r = api.move([dx, dy, release_z - 0.004], seconds=2.0)
    e, g = _state(api, "at_release")
    api.log(f"release residual {r:.4f}")
    api.grip(0.08)
    api.settle(0.5)
    _state(api, "opened")
    api.move([dx, dy, CARRY_Z], seconds=2.5)
    api.settle(0.5)
    _dump(api, "final", api.capture("cam_high"))
