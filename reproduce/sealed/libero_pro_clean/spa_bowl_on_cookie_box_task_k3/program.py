"""v4 — v3 grasped and carried the bowl (gap 0.0097 at the close, effort 3.0 all
the way to the release) but the drop landed off-centre: the rim pinch holds the
bowl one rim radius away in -y, so releasing with the EEF over the plate centre
puts the bowl centre ~48 mm short of it.  v4 offsets the release by the measured
grasp offset and drops lower.
"""
import base64
import zlib

import numpy as np

PROVENANCE = {
    "R_DOWN": {"source": "debug seed 51: api.tool_rotation() at reset", "allowed": True},
    "TABLE_Z": {"source": "debug seeds 51/53 cam_high depth: dominant z mode 0.90 m",
                "allowed": True},
    "SLAB_SEARCH": {"source": "debug seeds 51/53: z-histogram of the deprojected cloud; "
                              "cabinet top slab 1.12 m, bowl points above it",
                    "allowed": True},
    "GRASP_DEPTH": {"source": "mate pack keyframes close at ee z 1.146-1.159 with the rim "
                              "top measured at 1.179 on debug seed 51; v3 receipt: "
                              "0.025 below the rim top closes on the wall (gap 0.0097)",
                    "allowed": True},
    "CAND": {"source": "v2 vs v3 debug receipts: pinching along x closed on air (gap "
                       "0.001), pinching at the +y rim held (gap 0.0097, effort 3.0)",
             "allowed": True},
    "WALL_IN": {"source": "debug seed 51 v3: aiming 3 mm inside the measured outer rim "
                          "radius produced a 9.7 mm bite", "allowed": True},
    "PLATE_BAND": {"source": "debug seeds 51/53: plate cluster top 0.920 m, 7-20 mm above "
                             "the table", "allowed": True},
    "RELEASE_DZ": {"source": "mate pack keyframes release at ee z 0.946-0.982 with the "
                             "plate top at 0.920 -> 30-60 mm above the plate",
                   "allowed": True},
    "CARRY_Z": {"source": "mate pack ee_path6 carry leg z 1.25-1.31", "allowed": True},
    "TARGET": {"source": "api.instruction(): 'the akita black bowl on the top of the "
                         "cabinet'", "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
TABLE_Z = 0.90
CARRY_Z = 1.29
GRASP_DEPTH = 0.025
RELEASE_DZ = 0.045
WALL_IN = 0.003
TARGET = "cabinet"      # which bowl the intent names
DUMP_CARRY = True


def _dump(api, tag, arr):
    b64 = base64.b64encode(zlib.compress(np.ascontiguousarray(arr).tobytes(), 6)).decode()
    api.log("DUMP %s shape=%s dtype=%s n=%d" % (tag, list(arr.shape), str(arr.dtype),
                                                (len(b64) + 1799) // 1800))
    for i in range(0, len(b64), 1800):
        api.log("D %s %d %s" % (tag, i // 1800, b64[i:i + 1800]))


def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    d = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = d.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d, np.ones_like(d)], -1) @ T.T
    return f, P[..., :3]


def _bowl_in(P, reg, zlo, zhi):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    up = reg & (Z > zlo) & (Z < zhi)
    if up.sum() < 50:
        return None
    hz = np.histogram(Z[up], bins=np.arange(zlo, zhi, 0.01))
    slab = hz[1][int(np.argmax(hz[0]))]
    m = reg & (Z > slab + 0.015) & (Z < slab + 0.25)
    if m.sum() < 60:
        return None
    xs, ys, zs = X[m], Y[m], Z[m]
    cx0, cy0 = np.median(xs), np.median(ys)
    keep = (np.abs(xs - cx0) < 0.10) & (np.abs(ys - cy0) < 0.10)
    xs, ys, zs = xs[keep], ys[keep], zs[keep]
    if xs.size < 60:
        return None
    top = float(np.percentile(zs, 99))
    rim = zs > top - 0.014
    x0, x1 = np.percentile(xs[rim], 2), np.percentile(xs[rim], 98)
    y0, y1 = np.percentile(ys[rim], 2), np.percentile(ys[rim], 98)
    return {"n": int(xs.size), "slab": float(slab), "top": top,
            "cx": float((x0 + x1) / 2), "cy": float((y0 + y1) / 2),
            "rx": float((x1 - x0) / 2), "ry": float((y1 - y0) / 2)}


def find_bowl(P, which):
    X, Y = P[..., 0], P[..., 1]
    if which == "cabinet":
        reg = (Y < -0.10) & (Y > -0.60) & (X > -0.55) & (X < 0.35)
        return _bowl_in(P, reg, 1.02, 1.40)
    reg = (Y > -0.10) & (Y < 0.13) & (X > -0.10) & (X < 0.35)
    return _bowl_in(P, reg, 0.895, 1.05)


def find_plate(P, rgb):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    bright = rgb.astype(float).mean(-1)
    m = ((Y > 0.05) & (Y < 0.45) & (X > -0.30) & (X < 0.35)
         & (Z > TABLE_Z + 0.006) & (Z < TABLE_Z + 0.035) & (bright > 130))
    if m.sum() < 120:
        return None
    xs, ys, zs = X[m], Y[m], Z[m]
    cx0, cy0 = np.median(xs), np.median(ys)
    keep = (np.abs(xs - cx0) < 0.12) & (np.abs(ys - cy0) < 0.12)
    xs, ys, zs = xs[keep], ys[keep], zs[keep]
    x0, x1 = np.percentile(xs, 2), np.percentile(xs, 98)
    y0, y1 = np.percentile(ys, 2), np.percentile(ys, 98)
    return {"n": int(xs.size), "cx": float((x0 + x1) / 2), "cy": float((y0 + y1) / 2),
            "rx": float((x1 - x0) / 2), "ry": float((y1 - y0) / 2),
            "top": float(np.percentile(zs, 98))}


def run(api):
    api.log("instruction: %s" % api.instruction())
    f, P = cloud(api)
    b = find_bowl(P, TARGET)
    p = find_plate(P, f.rgb)
    api.log("bowl(%s): %s" % (TARGET, b))
    api.log("plate: %s" % p)
    if b is None or p is None:
        return "perception failed"

    off_y = max(b["ry"] - WALL_IN, 0.0)      # eef sits this far +y of the bowl centre
    ax, ay, top = b["cx"], b["cy"] + off_y, b["top"]
    api.log("aim x=%.4f y=%.4f off_y=%.4f top=%.4f" % (ax, ay, off_y, top))

    api.grip(0.08)
    api.move([b["cx"] - 0.09, b["cy"] + 0.13, top + 0.10], rotation=R_DOWN, seconds=2.0)
    api.move([ax, ay, top + 0.07], rotation=R_DOWN, seconds=2.0)
    res = api.move([ax, ay, top - GRASP_DEPTH], rotation=R_DOWN, seconds=2.0)
    api.log("descend res=%.4f eef=%s" % (res, np.round(api.eef(), 4).tolist()))
    e_close = api.eef()
    api.grip(0.0)
    api.settle(0.3)
    api.log("after close: %s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))

    api.move([ax, ay, top + 0.11], rotation=R_DOWN, seconds=2.0)
    api.log("after lift grip=%s eef=%s" % (api.gripper(), np.round(api.eef(), 4).tolist()))

    # the pinch grabbed the rim at +y of the bowl centre; carry that offset to
    # the release so the BOWL, not the wrist, ends up over the plate
    hold_dy = float(e_close[1] - b["cy"])
    hold_dx = float(e_close[0] - b["cx"])
    rx = p["cx"] + hold_dx
    ry = p["cy"] + hold_dy
    api.log("hold offset dx=%.4f dy=%.4f -> release xy %.4f %.4f" % (hold_dx, hold_dy, rx, ry))

    api.move([b["cx"], b["cy"] + 0.15, CARRY_Z], rotation=R_DOWN, seconds=2.0)
    api.move([rx, ry - 0.05, CARRY_Z], rotation=R_DOWN, seconds=2.5)
    api.move([rx, ry, p["top"] + 0.16], rotation=R_DOWN, seconds=2.0)
    if DUMP_CARRY:
        fc, Pc = cloud(api)
        _dump(api, "carry", fc.rgb[::2, ::2, :])
        X, Y, Z = Pc[..., 0], Pc[..., 1], Pc[..., 2]
        e = api.eef()
        near = (np.abs(X - e[0]) < 0.12) & (np.abs(Y - e[1]) < 0.12) & (Z < e[2] - 0.01)
        if near.sum() > 50:
            api.log("held cloud n=%d x[%.3f,%.3f] y[%.3f,%.3f] zbot=%.3f eef=%s"
                    % (int(near.sum()), X[near].min(), X[near].max(), Y[near].min(),
                       Y[near].max(), float(np.percentile(Z[near], 1)),
                       np.round(e, 4).tolist()))
    res = api.move([rx, ry, p["top"] + RELEASE_DZ], rotation=R_DOWN, seconds=2.0)
    api.log("release res=%.4f grip=%s eef=%s" % (res, api.gripper(),
                                                 np.round(api.eef(), 4).tolist()))
    api.grip(0.08)
    api.settle(1.0)
    api.move([rx, ry, p["top"] + 0.22], rotation=R_DOWN, seconds=2.0)
    api.settle(0.5)
    f3, P3 = cloud(api)
    _dump(api, "final", f3.rgb[::2, ::2, :])
    X, Y, Z = P3[..., 0], P3[..., 1], P3[..., 2]
    near = (np.abs(X - p["cx"]) < 0.13) & (np.abs(Y - p["cy"]) < 0.13) & (Z < 1.05)
    if near.sum() > 50:
        api.log("final near plate n=%d ztop=%.4f xmid=%.3f ymid=%.3f"
                % (int(near.sum()), float(np.percentile(Z[near], 99)),
                   float(np.median(X[near])), float(np.median(Y[near]))))
    return "v4 done"
