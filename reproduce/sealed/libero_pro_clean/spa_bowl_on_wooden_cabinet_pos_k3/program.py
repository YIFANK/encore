"""c2clean / spa_bowl_on_wooden_cabinet_pos_k3 -- v8.

Pick the bowl standing on the wooden cabinet and set it down on the plate.

Everything positional is perceived from one cam_high RGB-D frame taken before
the arm moves:
  * the cabinet is the largest tall structure that is not the arm;
  * the target bowl is what stands on the cabinet's top plateau;
  * the plate is the widest low blob whose top band is a FILLED disc (a bowl's
    top band is a ring with a hole, a plate's is solid).
The grasp is a rim pinch: the jaws close along base -y (measured from
tool_rotation at the home pose), so the tool goes to the rim wall on the +y
side of the bowl centre and closes across the wall.
"""
import numpy as np
from scipy import ndimage

PROVENANCE = {
    "GRASP_DZ": {
        "source": "pack keyframes: ee z at the gripper-close keyframe "
                  "(1.1590/1.1461/1.1522, mean 1.152) minus the debug-seed "
                  "measured bowl rim top 1.180 -> 0.028 below the rim top",
        "allowed": True},
    "RIM_SIDE": {
        "source": "pack keyframe close-y (-0.2572/-0.2175/-0.2153) lies one "
                  "rim radius on the +y side of bowl centres measured on debug "
                  "seeds 51/53/55/57 (cy ~ -0.28..-0.30, r ~ 0.054)",
        "allowed": True},
    "RIM_BAND_M": {
        "source": "debug seeds 51-57: a 6 mm top band of the target blob is a "
                  "thin annulus whose Kasa fit has 1.9 mm median residual",
        "allowed": True},
    "ON_TOP_DZ": {
        "source": "debug seeds 51-57: cabinet plateau 1.129, bowl rim top "
                  "1.180 -- 18 mm separates the plateau from anything on it",
        "allowed": True},
    "ARM_ZMAX": {
        "source": "debug seeds 51-57: the arm at the home pose reaches z 1.37, "
                  "every scene fixture tops out at 1.18",
        "allowed": True},
    "LOW_ZMAX": {
        "source": "debug seeds 51-57: no arm geometry projects below z 1.02 at "
                  "the home pose, so the low band is arm-free",
        "allowed": True},
    "XCROP": {
        "source": "debug seeds 51-57: the robot mount occupies x < -0.36; "
                  "cropping at -0.38 keeps the table and drops the mount",
        "allowed": True},
    "GRID_RES": {"source": "chosen resolution of my own top-down map", "allowed": True},
    "HOVER_CLEAR": {
        "source": "debug seeds 51-65: the fingertips ride at most 23 mm below "
                  "the tool (the grasp descends to 23 mm above the plateau "
                  "without the tips touching it), so tool = rim top + 45 mm "
                  "passes the closed-enough jaws over the rim with 22 mm to "
                  "spare; v6's higher 70 mm hover flipped the arm onto its "
                  "straight-elbow branch on seeds 61/63",
        "allowed": True},
    "WALK_M": {
        "source": "controller mechanics: the position command is "
                  "clip(err/0.05, -1, 1), so hops under ~0.05 m stay inside "
                  "the linear range; v7 debug run tied the saturated 0.36 m "
                  "approach to joint 4 hitting its limit on seeds 61/63",
        "allowed": True},
    "CARRY_CLEAR": {
        "source": "debug seeds 51-65: the cabinet plateau is the tallest thing "
                  "under the carry path and the bowl's base rides `hang` below "
                  "the tool (both measured in-episode); 45 mm of air over the "
                  "plateau. v2/v3 carried at 1.28/1.21 and the arm straightened "
                  "into a pose it could not leave",
        "allowed": True},
    "PLACE_CLEAR": {
        "source": "debug-seed measurement: release margin above the plate floor",
        "allowed": True},
    "GOTO_TOL": {
        "source": "v1 debug run: api.move stops at the controller's 12 mm "
                  "position tolerance, leaving a systematic 7-14 mm offset "
                  "(measured eef minus commanded on seeds 51-65); re-issuing "
                  "the command shifted by that error closes it",
        "allowed": True},
    "CARRY_OFFSET": {
        "source": "v1 debug run: the bowl is pinched at its rim, so its centre "
                  "trails the tool by one rim radius in -y; placing the tool on "
                  "the plate centre put the bowl on the plate's edge",
        "allowed": True},
}

RES = 0.004
XMIN, XMAX = -0.55, 0.45
YMIN, YMAX = -0.65, 0.65
GRASP_DZ = 0.028
RIM_BAND_M = 0.006
ON_TOP_DZ = 0.018
ARM_ZMAX = 1.25
LOW_ZMAX = 1.02
XCROP = -0.38
CARRY_CLEAR = 0.045
HOVER_CLEAR = 0.045
WALK_M = 0.04
PLACE_CLEAR = 0.010


# ---------------------------------------------------------------- perception
def _grid(P, rgb):
    nx = int((XMAX - XMIN) / RES)
    ny = int((YMAX - YMIN) / RES)
    x, y, z = P[..., 0].ravel(), P[..., 1].ravel(), P[..., 2].ravel()
    ok = (x > XMIN) & (x < XMAX) & (y > YMIN) & (y < YMAX) & np.isfinite(z)
    ix = ((x[ok] - XMIN) / RES).astype(np.int32)
    iy = ((y[ok] - YMIN) / RES).astype(np.int32)
    zz = z[ok]
    flat = ix * ny + iy
    order = np.argsort(zz)
    H = np.full(nx * ny, np.nan, np.float32)
    H[flat[order]] = zz[order]
    return H.reshape(nx, ny)


def _cloud(f):
    K, T = np.asarray(f.intrinsics, float), np.asarray(f.t_base_cam, float)
    d = np.asarray(f.depth, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    pts = np.stack([x, y, d, np.ones_like(d)], -1)
    return pts @ T.T[:, :3]


def _blobs(mask, min_cells):
    lab, n = ndimage.label(mask, structure=np.ones((3, 3)))
    return [lab == c for c in range(1, n + 1) if int((lab == c).sum()) >= min_cells]


def _xy(m):
    idx = np.argwhere(m)
    return XMIN + idx[:, 0] * RES, YMIN + idx[:, 1] * RES


def _kasa(xs, ys):
    A = np.stack([xs, ys, np.ones_like(xs)], 1)
    b = xs ** 2 + ys ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = float(sol[0] / 2), float(sol[1] / 2)
    r = float(np.sqrt(sol[2] + cx ** 2 + cy ** 2))
    res = float(np.median(np.abs(np.hypot(xs - cx, ys - cy) - r)))
    return cx, cy, r, res


def _mode_z(z, step=0.005):
    hist, edges = np.histogram(z, bins=np.arange(np.nanmin(z) - 0.01,
                                                np.nanmax(z) + 0.02, step))
    return float(edges[int(np.argmax(hist))]) + step / 2


def _fill(H, band, cx, cy, r):
    gx = XMIN + np.arange(H.shape[0])[:, None] * RES
    gy = YMIN + np.arange(H.shape[1])[None, :] * RES
    inner = np.hypot(gx - cx, gy - cy) < 0.55 * r
    if not inner.any():
        return 0.0
    return float((band & inner).sum()) / float(inner.sum())


def perceive(api):
    f = api.capture("cam_high")
    H = _grid(_cloud(f), f.rgb)
    fin = np.isfinite(H)
    hist, edges = np.histogram(H[fin], bins=np.arange(0.70, 1.40, 0.005))
    tz = float(edges[int(np.argmax(hist))]) + 0.0025

    # -- cabinet: largest tall blob that does not reach the arm's altitude
    cab, best = None, 0
    for b in _blobs(fin & (H > 1.02), 200):
        if float(np.nanmax(H[b])) > ARM_ZMAX:
            continue
        if int(b.sum()) > best:
            cab, best = b, int(b.sum())
    if cab is None:
        raise RuntimeError("no cabinet")
    cabtop = _mode_z(H[cab])

    # -- target: the thing standing on the cabinet plateau
    on = cab & fin & (H > cabtop + ON_TOP_DZ)
    cands = sorted(_blobs(on, 60), key=lambda m: -m.sum())
    if not cands:
        raise RuntimeError("nothing on the cabinet")
    tgt = cands[0]
    ztop = float(np.percentile(H[tgt], 98))
    rim = tgt & (H > ztop - RIM_BAND_M)
    rx, ry = _xy(rim)
    cx, cy, r, res = _kasa(rx, ry)
    if not (0.02 < r < 0.11) or res > 0.006:      # fit refused -> bbox fallback
        cx, cy = 0.5 * (rx.min() + rx.max()), 0.5 * (ry.min() + ry.max())
        r = 0.5 * (rx.max() - rx.min())

    # -- plate: widest low blob whose top band is a filled disc
    gx = XMIN + np.arange(H.shape[0])[:, None] * RES
    low = fin & (H > tz + 0.015) & (H < LOW_ZMAX) & ~cab & (gx > XCROP)
    plate, pn = None, 0
    for b in _blobs(low, 60):
        zt = float(np.percentile(H[b], 98))
        band = b & (H > zt - 0.016)
        bx, by = _xy(band)
        mx, my = 0.5 * (bx.min() + bx.max()), 0.5 * (by.min() + by.max())
        rr = max(0.5 * (bx.max() - bx.min()), 0.5 * (by.max() - by.min()))
        if _fill(H, band, mx, my, rr) > 0.5 and int(band.sum()) > pn:
            kx, ky, kr, kres = _kasa(bx, by)
            if not (0.03 < kr < 0.12) or kres > 0.008:
                kx, ky, kr = mx, my, rr
            plate, pn = (kx, ky, kr, zt, float(np.nanmedian(H[band]))), int(band.sum())
    if plate is None:
        raise RuntimeError("no plate")

    return dict(tz=tz, cabtop=cabtop, cx=cx, cy=cy, r=r, ztop=ztop, res=res,
                px=plate[0], py=plate[1], pr=plate[2], pztop=plate[3],
                pfloor=plate[4], pn=pn, tn=int(tgt.sum()))


GOTO_TOL = 0.006
CARRY_OFFSET = 1.0


def goto(api, target, seconds=1.2, tol=GOTO_TOL, tries=3, gain=1.4, tag=""):
    """api.move stops inside the controller's 12 mm tolerance, so re-issue the
    command shifted by the observed error. Retries are abandoned as soon as one
    stops helping: on some seeds an axis simply will not track, and chasing it
    through four full-length moves ate the whole 1000-step episode horizon.
    """
    target = np.asarray(target, float)
    cmd = target.copy()
    err = target - api.eef()
    prev = float(np.linalg.norm(err))
    for i in range(tries):
        api.move(cmd, seconds=seconds)
        err = target - api.eef()
        n = float(np.linalg.norm(err))
        if n < tol:
            break
        if i and n > 0.8 * prev:          # no longer converging -- stop paying
            break
        prev = n
        cmd = target + np.clip(cmd + gain * err - target, -0.05, 0.05)
        seconds = 0.5                      # corrections are short by design
    if tag:
        api.log("%s err=%s eef=%s" % (tag, np.round(err, 4).tolist(),
                                      np.round(api.eef(), 4).tolist()))
    return err


def walk(api, target, step=WALK_M, seconds=0.5):
    """Approach in hops short enough that the controller's proportional command
    (err / 0.05) never saturates. A single full-length command saturates every
    axis at once, and on the far-x seeds that slammed joint 4 into its own
    limit (measured -0.0697) -- a straight elbow the controller could not undo.
    """
    target = np.asarray(target, float)
    start = api.eef()
    n = int(np.ceil(float(np.linalg.norm(target - start)) / step))
    for k in range(1, n):
        api.move(start + (target - start) * k / n, seconds=seconds)
    return n


# ---------------------------------------------------------------------- run
def run(api):
    s = perceive(api)
    api.log("SCENE %s" % {k: (round(float(v), 4) if isinstance(v, (float, np.floating))
                              else v) for k, v in s.items()})

    gx, gy = s["cx"], s["cy"] + s["r"]
    gz = s["ztop"] - GRASP_DZ
    hang = gz - s["cabtop"]                # eef height above the bowl's base
    api.log("GRASP at (%.4f, %.4f, %.4f) hang=%.4f" % (gx, gy, gz, hang))

    api.grip(0.08)
    api.log("J home=%s" % api.proprio().get("robot0_joint_pos"))
    walk(api, [gx, gy, s["ztop"] + HOVER_CLEAR])
    goto(api, [gx, gy, s["ztop"] + HOVER_CLEAR], seconds=0.8, tag="hover")
    api.log("J hover=%s" % api.proprio().get("robot0_joint_pos"))
    walk(api, [gx, gy, gz], step=0.025)
    goto(api, [gx, gy, gz], seconds=0.6, tag="descend")
    api.log("J descend=%s" % api.proprio().get("robot0_joint_pos"))
    api.grip(0.0)
    api.settle(0.3)
    api.log("after close gripper=%s" % api.gripper())

    carry_z = s["cabtop"] + hang + CARRY_CLEAR
    goto(api, [gx, gy, carry_z], seconds=1.0, tag="lift")
    api.log("after lift gripper=%s carry_z=%.4f J=%s"
            % (api.gripper(), carry_z, api.proprio().get("robot0_joint_pos")))

    # The bowl hangs one rim radius toward -y of the tool, so aim the tool that
    # far on the +y side of the plate centre.
    tx, ty = s["px"], s["py"] + CARRY_OFFSET * s["r"]
    pz = s["pfloor"] + hang + PLACE_CLEAR

    # Carry x first, then y, at the lowest height that still clears the cabinet
    # with the hanging bowl. Holding the tool far out in +x at carry height
    # straightens the elbow (measured j4 ~ -0.07 on seeds 61/63), and from that
    # pose the controller cannot pull the tool back in -- it pushed for the
    # whole remaining horizon and never moved. Retreating in x first folds the
    # elbow instead.
    walk(api, [tx, gy, carry_z])
    goto(api, [tx, gy, carry_z], seconds=0.8, tol=0.02, tag="across")
    goto(api, [tx, ty, carry_z], seconds=1.2, tol=0.02, tag="lateral")
    api.log("carry joints=%s" % api.proprio().get("robot0_joint_pos"))
    api.log("PLACE tool at (%.4f, %.4f, %.4f) gripper=%s" % (tx, ty, pz, api.gripper()))
    goto(api, [tx, ty, pz], seconds=1.2, tag="place")
    api.log("at place gripper=%s" % api.gripper())
    api.grip(0.08)
    api.settle(0.4)
    api.move([tx, ty, pz + 0.12], seconds=1.0)
    api.settle(0.4)
    _dump_scene(api)
    return "v8 done"


def _dump_scene(api):
    """Post-episode diagnostic: stream the final cam_high height map out."""
    import base64
    import zlib
    try:
        f = api.capture("cam_high")
        H = _grid(_cloud(f), f.rgb)
        raw = zlib.compress(np.ascontiguousarray(H.astype(np.float16)).tobytes(), 6)
        b64 = base64.b64encode(raw).decode()
        n = (len(b64) + 1799) // 1800
        api.log("FINALH shape=%s nchunks=%d" % (list(H.shape), n))
        for i in range(n):
            api.log("D %d %s" % (i, b64[i * 1800:(i + 1) * 1800]))
    except Exception as e:
        api.log("dump failed %s" % e)
