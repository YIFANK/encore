"""Turn on the stove -- topple the knob's fin with the closed jaws as a blade.

Read off the K=1 pack: the demo drives to the knob, closes the gripper and then
holds a saturated downward push while its attitude turns 0.978 rad about the
world +Y axis (rotvec of R(t79) @ R(t41)^-1); keyframe demo0_t0079.png shows the
burner rings lit, so that turn is the "on" event.

Debug seeds show why the pack's own tool-rotation recipe does not transfer: the
knob is a disc on the table carrying an upright fin only 0.023 m thick, and a
pinch on it closes to 0.0253 m, so the fin simply rotates inside the jaws
(versions v3/v4/v5 turned the wrist up to 0.8 rad and left the fin untouched).
The fin swings about a +Y hinge at its base, so what moves it is lateral force
high on the fin: park the closed jaws behind it at fin-top height and drag
forward. The push stalls against the fin with a rising residual and then breaks
through as the fin goes over.
"""
import base64
import zlib

import numpy as np
from scipy import ndimage

PROVENANCE = {
    "DARK_MAX": {
        "source": "debug seeds 51/53/55 cam_high RGB: the knob reads mean RGB ~21, "
                  "the stove slab ~85 and the table ~150",
        "allowed": True},
    "KNOB_MAX_H": {
        "source": "debug seeds 51/53/55 cam_high depth: knob top 0.960 over a table "
                  "plane of 0.9012 (0.059 tall); the only other dark object standing "
                  "on the table, the bottle, is 0.148 tall",
        "allowed": True},
    "KNOB_MIN_W": {
        "source": "debug seeds 51/53/55: the knob disc spans 0.089-0.091 m in y, the "
                  "bottle 0.040 m",
        "allowed": True},
    "KNOB_BASE_H": {
        "source": "debug seeds 51/53/55: the knob's lowest points sit at 0.906-0.907, "
                  "i.e. on the table plane, unlike objects standing on the slab",
        "allowed": True},
    "MIN_PIX": {
        "source": "debug seeds 51/53/55: the knob is 511-531 px at stride 2; the "
                  "distractor blobs that survive the dark threshold are <350",
        "allowed": True},
    "FIN_BAND": {
        "source": "debug seeds 51/53/55: knob points above table+0.030 are the fin "
                  "alone (y-extent 0.025) -- the disc top is at table+0.022",
        "allowed": True},
    "STANDOFF": {
        "source": "debug seeds 51/53/55: the disc radius is 0.045, so starting this "
                  "far behind the fin's rear edge puts the jaws clear of the disc",
        "allowed": True},
    "PUSH_Z": {
        "source": "debug seed descents (v2-v4 logs): commanded z tracks ~0.010 high, "
                  "so table+0.035 places the jaws at ~0.947, in the fin's upper half "
                  "(fin spans table+0.022 to table+0.059)",
        "allowed": True},
    "PUSH_STEP": {
        "source": "debug seed v2 logs: api.move residuals stand at ~0.011 m, so a "
                  "0.012 m step keeps the command ahead of the tool and the contact "
                  "loaded",
        "allowed": True},
    "N_PUSH": {
        "source": "debug seeds: the fin topples between step 5 and 7 of the push "
                  "(v8 log ep60); 12 steps = 0.144 m leaves margin",
        "allowed": True},
    "R_DOWN": {
        "source": "generic controller mechanics: tool z along world -z, jaws closing "
                  "along world y, which is the fin's plane normal",
        "allowed": True},
}

DARK_MAX = 55.0
KNOB_MAX_H = 0.09
KNOB_MIN_W = 0.055
KNOB_BASE_H = 0.025
MIN_PIX = 150
FIN_BAND = 0.030
STANDOFF = 0.045
PUSH_Z = 0.035
PUSH_STEP = 0.012
N_PUSH = 12

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def _dump(api, tag, arr):
    b64 = base64.b64encode(zlib.compress(np.ascontiguousarray(arr).tobytes(), 6)).decode()
    api.log(f"DUMP {tag} shape={list(arr.shape)} dtype={arr.dtype}")
    for i in range(0, len(b64), 1800):
        api.log(f"CHUNK {tag} {i // 1800} {b64[i:i + 1800]}")


def _cloud(f):
    d = np.asarray(f.depth, np.float32)[::2, ::2]
    d = np.nan_to_num(d, nan=0.0, posinf=0.0, neginf=0.0)
    H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]
    K, T = np.asarray(f.intrinsics), np.asarray(f.t_base_cam)
    u, v = uu * 2.0, vv * 2.0
    xc = (u - K[0, 2]) * d / K[0, 0]
    yc = (v - K[1, 2]) * d / K[1, 1]
    P = np.stack([xc, yc, d], -1) @ T[:3, :3].T + T[:3, 3]
    return P, d > 0, np.asarray(f.rgb, float)[::2, ::2]


def _find_knob(api, P, ok, rgb):
    tab = float(np.median(P[..., 2][ok & (P[..., 2] > 0.80) & (P[..., 2] < 0.95)]))
    api.log(f"TABLE {tab:.4f}")
    dark = ok & (rgb.max(2) < DARK_MAX) & (P[..., 2] > tab + 0.004)
    dark &= (P[..., 0] > -0.45) & (P[..., 1] > -0.10) & (P[..., 1] < 0.55)
    lab, n = ndimage.label(dark)
    best = None
    for i in range(1, n + 1):
        c = lab == i
        if c.sum() < MIN_PIX:
            continue
        Q = P[c]
        h = Q[:, 2].max() - tab
        wy = Q[:, 1].max() - Q[:, 1].min()
        api.log(f"CAND {i} n={int(c.sum())} h={h:.3f} wy={wy:.3f} "
                f"zmin={Q[:, 2].min():.3f} x={Q[:, 0].mean():.3f} y={Q[:, 1].mean():.3f}")
        if h > KNOB_MAX_H or Q[:, 2].min() > tab + KNOB_BASE_H or wy < KNOB_MIN_W:
            continue
        if best is None or c.sum() > best[0]:
            best = (int(c.sum()), Q)
    if best is None:
        return None, tab
    Q = best[1]
    api.log(f"KNOB n={best[0]} x[{Q[:, 0].min():.3f},{Q[:, 0].max():.3f}] "
            f"y[{Q[:, 1].min():.3f},{Q[:, 1].max():.3f}] "
            f"z[{Q[:, 2].min():.3f},{Q[:, 2].max():.3f}]")
    fin = Q[Q[:, 2] > tab + FIN_BAND]
    api.log(f"LEVER n={len(fin)} xmed={np.median(fin[:, 0]):.4f} "
            f"ymed={np.median(fin[:, 1]):.4f} "
            f"x[{fin[:, 0].min():.3f},{fin[:, 0].max():.3f}] "
            f"y[{fin[:, 1].min():.3f},{fin[:, 1].max():.3f}] "
            f"ztop={fin[:, 2].max():.3f}")
    return fin, tab


def _state(api, tag):
    e = api.eef()
    g = api.gripper()
    api.log(f"ST {tag} eef={np.round(e, 4).tolist()} "
            f"w={g['width_m']:.4f} eff={g['effort']:.2f}")
    return e


def run(api):
    f = api.capture("cam_high")
    P, ok, rgb = _cloud(f)
    fin, tab = _find_knob(api, P, ok, rgb)
    if fin is None:
        api.log("NO_KNOB")
        return
    x0 = float(fin[:, 0].min())          # the fin's rear edge
    gy = float(np.median(fin[:, 1]))     # the fin's plane, in y
    api.log(f"BLADE x0={x0:.4f} gy={gy:.4f}")

    api.grip(0.0)                        # closed jaws = a rigid blade
    api.settle(0.2)
    api.move([x0 - STANDOFF, gy, tab + 0.14], rotation=R_DOWN, seconds=2.0)
    _state(api, "hover")
    for z in (0.10, 0.075, 0.055, PUSH_Z):
        api.move([x0 - STANDOFF, gy, tab + z], rotation=R_DOWN, seconds=1.2)
        _state(api, f"d{z}")
    e = api.eef()
    for k in range(1, N_PUSH + 1):
        r = api.move([x0 - STANDOFF + PUSH_STEP * k, gy, e[2]], rotation=R_DOWN,
                     seconds=0.8)
        cur = api.eef()
        api.log(f"PU {k} res={r:.4f} eef={np.round(cur, 4).tolist()}")

    f2 = api.capture("cam_high")
    _dump(api, "final_rgb", np.asarray(f2.rgb)[::2, ::2, :].astype(np.uint8))
    api.log("DONE")
