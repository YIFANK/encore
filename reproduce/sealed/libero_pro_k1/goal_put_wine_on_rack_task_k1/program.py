"""v4 -- full pipeline: grasp the cream cheese, carry it over the rack, release
it on the inclined slatted surface.

Mechanism, from the two packs:
  * the k1 pack ("put the wine bottle on the rack") acts on the TARGET in THIS
    scene: its release points sit at x=-0.13..-0.16, y=-0.25..-0.27, z=1.20..1.23,
    i.e. on the inclined rack surface, which is what "on the rack" means here.
  * the mate pack ("put the cream cheese in the bowl") handles THIS object: a
    straight-down wrist and a grasp with the eef at z=0.9104.
Everything metric about my own scene is re-derived from debug seeds 51/53/57/61.
"""
import base64
import json
import zlib

import numpy as np

PROVENANCE = {
    "CC_BLUE_THR": {"source": "debug seeds 51/53/57/61 cam_high RGB: the cream cheese is the only blue prop in the near-table height band; B-(R+G)/2 > 12 isolates it", "allowed": True},
    "CC_BAND": {"source": "debug seeds: table deprojects to z=0.901, cream-cheese top face to z=0.9197 -> height gate 0.906..0.940", "allowed": True},
    "Z_GRASP_CMD": {"source": "debug seed v2 probe: commanding eef z=0.900 settles at 0.9104 and closes on the slab (gap 0.0422, effort 3.0) on 4/4 seeds; matches the mate pack's own grasp z of 0.9104", "allowed": True},
    "GRIP_OK_MIN": {"source": "debug seed v2 probe: closed gap on the cream cheese is 0.0422 m on 4/4 seeds (its measured short extent is 0.039-0.041); an empty close reads ~0", "allowed": True},
    "HANG": {"source": "debug seed v2 probe: at the grasp the object bottom (table, 0.901) sits 0.0094 below the eef (0.9104); re-perceiving the held slab after the lift gives 0.008-0.011", "allowed": True},
    "CC_HALF_Y": {"source": "debug seeds: cream-cheese short extent 0.039-0.041 m -> half-width 0.021", "allowed": True},
    "JAW_OPEN_HALF": {"source": "debug seeds: gripper reports width_m=0.0778 when open -> each fingertip sits 0.039 from the tool axis", "allowed": True},
    "TIP_BELOW_EEF": {"source": "debug seed v2 probe: fingertips reach the table (0.901) with the eef at 0.9104 -> 0.010", "allowed": True},
    "RACK_GATE": {"source": "debug seeds 51/53/57/61: the rack surface occupies x[-0.41,-0.12], y[-0.36,-0.15] above z=1.12; the cabinet top (z=1.128) begins at x>-0.104 and is excluded by the x gate", "allowed": True},
    "PLACE_X": {"source": "debug seeds: rack x extent is [-0.405,-0.120] on every seed, so x=-0.23 is interior with >0.07 margin for the slab's 0.079 length; the k1 pack released at x=-0.13..-0.26", "allowed": True},
    "PLACE_Y_CLAMP": {"source": "debug seeds: rack y extent [-0.333,-0.178]; placing at the mid-y keeps the slab's 0.041 y-footprint clear of both edges", "allowed": True},
    "CARRY_Z": {"source": "debug seeds: tallest thing on the route is the rack rail at z=1.245 and the cabinet top at 1.128, so 1.30 clears everything; the k1 pack itself transits at z=1.289", "allowed": True},
    "REL_MARGIN": {"source": "debug seeds: rack plane slope dz/dy=-0.61, so the uphill fingertip at 0.039 from the tool axis sits 0.024 higher than the surface under the tool; +0.010 keeps it clear while the jaws open", "allowed": True},
}

CC_BLUE_THR = 12.0
CC_BAND = (0.906, 0.940)
Z_GRASP_CMD = 0.900
GRIP_OK_MIN = 0.030
HANG = 0.0094
JAW_OPEN_HALF = 0.039
TIP_BELOW_EEF = 0.010
RACK_GATE = dict(x=(-0.41, -0.12), y=(-0.36, -0.15), zmin=1.12)
PLACE_X = -0.23
PLACE_Y_CLAMP = (-0.285, -0.225)
CARRY_Z = 1.30
REL_MARGIN = 0.010

CH = 1900
R_DOWN = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])


# ---------------------------------------------------------------- logging ---
def _blob(api, tag, arr):
    b = base64.b64encode(zlib.compress(np.ascontiguousarray(arr).tobytes(), 6)).decode()
    n = (len(b) + CH - 1) // CH
    api.log("BLOB %s n=%d dtype=%s shape=%s" % (tag, n, arr.dtype, list(arr.shape)))
    for i in range(n):
        api.log("B %s %d %s" % (tag, i, b[i * CH:(i + 1) * CH]))


def dump(api, tag, f):
    api.log("CAM %s %s" % (tag, json.dumps({
        "K": np.asarray(f.intrinsics, float).tolist(),
        "T": np.asarray(f.t_base_cam, float).tolist()})))
    _blob(api, tag + ".rgb", f.rgb.astype(np.uint8))
    d = np.nan_to_num(np.asarray(f.depth, float), nan=0.0, posinf=0.0, neginf=0.0)
    _blob(api, tag + ".dep", np.clip(d * 10000.0, 0, 65535).astype(np.uint16))


# ------------------------------------------------------------- perception ---
def cloud(f):
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    d = np.nan_to_num(np.asarray(f.depth, float), nan=0.0, posinf=0.0, neginf=0.0)
    h, w = d.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    x = (u - K[0, 2]) * d / K[0, 0]
    y = (v - K[1, 2]) * d / K[1, 1]
    return np.stack([x, y, d, np.ones_like(d)], -1) @ T.T[:, :3], d


def find_cc(api, f):
    P, d = cloud(f)
    rgb = f.rgb.astype(np.float32)
    blue = rgb[..., 2] - 0.5 * (rgb[..., 0] + rgb[..., 1])
    m = ((d > 0) & (blue > CC_BLUE_THR)
         & (P[..., 2] > CC_BAND[0]) & (P[..., 2] < CC_BAND[1])
         & (P[..., 0] > -0.45) & (P[..., 0] < 0.30)
         & (P[..., 1] > -0.45) & (P[..., 1] < 0.45))
    vs, us = np.nonzero(m)
    if vs.size < 50:
        return None
    cu, cv = np.median(us), np.median(vs)
    keep = ((us - cu) ** 2 + (vs - cv) ** 2) < 70.0 ** 2
    pts = P[vs[keep], us[keep]]
    c = pts[:, :2].mean(0)
    A = pts[:, :2] - c
    _, _, vt = np.linalg.svd(A, full_matrices=False)
    e = A @ vt.T
    ext = (float(np.ptp(e[:, 0])), float(np.ptp(e[:, 1])))
    api.log("CC ctr=%s ztop=%.4f short=%s ext=%s n=%d"
            % (np.round(c, 4).tolist(), pts[:, 2].max(),
               np.round(vt[1], 3).tolist(), np.round(ext, 4).tolist(), len(pts)))
    return c, vt[1], ext


def fit_rack(api, f):
    P, d = cloud(f)
    g = RACK_GATE
    m = ((d > 0) & (P[..., 2] > g["zmin"])
         & (P[..., 0] > g["x"][0]) & (P[..., 0] < g["x"][1])
         & (P[..., 1] > g["y"][0]) & (P[..., 1] < g["y"][1]))
    q = P[m]
    G = np.c_[q[:, 0], q[:, 1], np.ones(len(q))]
    coef, *_ = np.linalg.lstsq(G, q[:, 2], rcond=None)
    ylo, yhi = np.percentile(q[:, 1], [3, 97])
    api.log("RACK n=%d coef=%s rms=%.4f y[%.3f,%.3f]"
            % (len(q), np.round(coef, 4).tolist(),
               (q[:, 2] - G @ coef).std(), ylo, yhi))
    return coef, float(ylo), float(yhi), q


# ------------------------------------------------------------------ motion ---
def rot_for(shortv):
    """Tool z straight down; tool y along the slab's short axis = the jaw axis."""
    ty = np.array([shortv[0], shortv[1], 0.0])
    ty = ty / np.linalg.norm(ty)
    if ty[1] < 0:
        ty = -ty
    tz = np.array([0.0, 0.0, -1.0])
    return np.stack([np.cross(ty, tz), ty, tz], axis=1)


def go(api, xyz, R, seconds=2.5, tol=0.004, tries=2):
    """Move, then cancel the standing tracking bias (bounded)."""
    tgt = np.asarray(xyz, float)
    api.move(tgt, rotation=R, seconds=seconds)
    cmd = tgt.copy()
    for _ in range(tries):
        e = np.asarray(api.eef(), float)
        err = tgt - e
        if np.linalg.norm(err) < tol:
            break
        cmd = cmd + np.clip(err, -0.03, 0.03)
        api.move(cmd, rotation=R, seconds=1.5)
    return np.asarray(api.eef(), float)


def run(api):
    api.log("INSTR %r" % api.instruction())
    f0 = api.capture("cam_high")
    dump(api, "t0", f0)
    coef, ylo, yhi, _ = fit_rack(api, f0)
    cc = find_cc(api, f0)
    if cc is None:
        api.log("NO CC -- abort")
        return
    c, shortv, ext = cc
    R = rot_for(shortv)

    # ---- grasp ----
    api.grip(0.08)
    api.move([c[0], c[1], 1.03], rotation=R, seconds=3.0)
    api.move([c[0], c[1], 0.960], rotation=R, seconds=1.5)
    api.move([c[0], c[1], Z_GRASP_CMD], rotation=R, seconds=2.0)
    api.log("GRASPPOSE eef=%s" % np.round(api.eef(), 4).tolist())
    api.grip(0.0)
    api.settle(0.4)
    g = api.gripper()
    api.log("AFTERCLOSE %s" % json.dumps(g))
    if g["width_m"] < GRIP_OK_MIN:
        api.log("GRASP FAILED -- retry once")
        api.grip(0.08)
        api.move([c[0], c[1], 1.03], rotation=R, seconds=2.0)
        f1 = api.capture("cam_high")
        cc = find_cc(api, f1)
        if cc is None:
            return
        c, shortv, ext = cc
        R = rot_for(shortv)
        api.move([c[0], c[1], 1.03], rotation=R, seconds=2.0)
        api.move([c[0], c[1], Z_GRASP_CMD], rotation=R, seconds=2.5)
        api.grip(0.0)
        api.settle(0.4)
        api.log("AFTERCLOSE2 %s" % json.dumps(api.gripper()))

    # ---- carry ----
    api.move([c[0], c[1], 1.10], rotation=R, seconds=2.5)
    api.log("AFTERLIFT eef=%s grip=%s"
            % (np.round(api.eef(), 4).tolist(), json.dumps(api.gripper())))
    api.move([c[0], c[1], CARRY_Z], rotation=R, seconds=3.0)
    api.move([0.5 * (c[0] + PLACE_X), 0.5 * (c[1] - 0.25), CARRY_Z],
             rotation=R, seconds=3.0)

    # ---- place ----
    py = float(np.clip(0.5 * (ylo + yhi), PLACE_Y_CLAMP[0], PLACE_Y_CLAMP[1]))
    px = PLACE_X
    surf = coef[0] * px + coef[1] * py + coef[2]
    # the uphill fingertip, 0.039 from the tool axis, sits over a higher part
    # of the ramp; keep it (and hence the slab) clear while the jaws open.
    uphill = surf + abs(coef[1]) * JAW_OPEN_HALF
    z_rel = uphill + TIP_BELOW_EEF + REL_MARGIN
    api.log("PLACE px=%.3f py=%.3f surf=%.4f uphill=%.4f z_rel=%.4f" % (px, py, surf, uphill, z_rel))

    go(api, [px, py, CARRY_Z], R, seconds=3.0)
    e = go(api, [px, py, z_rel], R, seconds=3.0, tol=0.004, tries=2)
    api.log("RELEASEPOSE eef=%s grip=%s"
            % (np.round(e, 4).tolist(), json.dumps(api.gripper())))
    api.grip(0.08)
    api.settle(0.6)
    api.move([px, py, CARRY_Z], rotation=R, seconds=3.0)
    api.move([px + 0.10, py + 0.12, CARRY_Z], rotation=R, seconds=2.5)
    api.settle(0.5)
    dump(api, "end", api.capture("cam_high"))
