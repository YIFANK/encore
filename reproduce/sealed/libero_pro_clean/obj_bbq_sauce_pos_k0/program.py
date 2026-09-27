"""v2: perceive -> calibrate fingertip offset -> pick the bbq sauce -> drop in basket.

Everything is re-derived from cam_high RGB-D in this episode; the only
literals are thresholds measured on debug seeds 51..65 (see PROVENANCE).
"""
import os
import time
from collections import defaultdict

import numpy as np

PROVENANCE = {
    "ROBOT_GCHROMA": {"source": "debug seeds 51-65 cam_high: robot links have green chromaticity "
                                "g/(r+g+b) ~0.52 while every table object is <0.42",
                      "allowed": True},
    "SCENE_ZMAX": {"source": "debug seeds 51-65: tallest table object measured 0.148 m above the "
                             "table plane; gripper hardware sits above 0.20 m at the home pose",
                   "allowed": True},
    "TABLE_Z": {"source": "debug seeds 51-65: cam_high deprojection of empty floor pixels gives "
                          "z ~0.001 m, so the support plane is z=0 in base frame",
                "allowed": True},
    "GRASP_H": {"source": "debug seeds 51-65: red-chroma cluster (the bbq bottle) has y-width "
                          "0.047 m over z in [0.02,0.06] and tapers to 0.027 m above 0.10 m; "
                          "0.045 m is mid-body",
                "allowed": True},
    "VIEW_BIAS": {"source": "debug seeds 51-65: cam_high sees only the face of an object turned "
                            "toward it, so the visible-surface points sit ~half the object's "
                            "cross-view width in front of its true centre (bottle cross-view "
                            "width 0.047 m, centroid 0.023 m camera-side of the body axis)",
                  "allowed": True},
    "HOLD_EFFORT": {"source": "debug seeds 51-57 v2: api.gripper() reads effort 3.0 with width "
                              "0.036 while carrying the bottle and effort 0.05 when open",
                    "allowed": True},
    "PROBE_XY": {"source": "debug seeds 51-65: no cluster within 0.10 m of (0.00,-0.35), so it is "
                           "bare table for the fingertip-offset probe",
                 "allowed": True},
    "OPEN_W": {"source": "debug seed 51 api.gripper() at reset: width_m 0.0778 = fully open",
               "allowed": True},
}

ROBOT_GCHROMA = 0.42
SCENE_ZMAX = 0.20
TABLE_Z = 0.0
GRASP_H = 0.045
HOLD_EFFORT = 1.0
HOLD_WMIN = 0.015
PROBE_XY = (0.00, -0.35)
OPEN_W = 0.078

DUMP_DIR = "/mnt/data/YifanKang/Heron/results/dump_c2clean_obj_bbq_sauce_pos_k0"


# ---------------------------------------------------------------- perception

def _cloud(frame):
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    dep = np.asarray(frame.depth, float)
    H, W = dep.shape
    f, cx, cy = K[0, 0], K[0, 2], K[1, 2]
    v, u = np.mgrid[0:H, 0:W]
    P = np.stack([(u - cx) / f * dep, (v - cy) / f * dep, dep, np.ones_like(dep)], -1) @ T.T
    return P[..., :3], frame.rgb.astype(float) / 255.0


def _clusters(P, rgb, cell=0.010, minpix=60):
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    s = rgb.sum(-1) + 1e-6
    robot = (rgb[..., 1] / s > ROBOT_GCHROMA) & (rgb.mean(-1) < 0.35)
    m = (np.isfinite(z) & (x > -0.42) & (x < 0.42) & (y > -0.42) & (y < 0.50)
         & (z > TABLE_Z + 0.012) & (z < SCENE_ZMAX) & (~robot))
    pts, cols = P[m], rgb[m]
    if len(pts) == 0:
        return []
    gx = np.floor(pts[:, 0] / cell).astype(int)
    gy = np.floor(pts[:, 1] / cell).astype(int)
    occ = defaultdict(list)
    for i in range(len(pts)):
        occ[(gx[i], gy[i])].append(i)
    keys = {k for k, vv in occ.items() if len(vv) >= 4}
    seen, out = set(), []
    for k in keys:
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            c = stack.pop()
            comp.append(c)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in keys and n not in seen:
                        seen.add(n)
                        stack.append(n)
        ii = np.concatenate([occ[c] for c in comp])
        if len(ii) < minpix:
            continue
        p, c = pts[ii], cols[ii]
        sc = c.sum(1) + 1e-6
        out.append(dict(n=len(ii), pts=p,
                        ctr=p.mean(0), zmax=float(p[:, 2].max()),
                        bbox=0.5 * np.array([p[:, 0].min() + p[:, 0].max(),
                                             p[:, 1].min() + p[:, 1].max()]),
                        ywid=float(p[:, 1].max() - p[:, 1].min()),
                        rch=float((c[:, 0] / sc).mean()),
                        lum=float(c.mean())))
    out.sort(key=lambda a: -a["n"])
    return out


# ------------------------------------------------------------------- program

def _axis_centre(pts, cam_xy):
    """True horizontal centre of a convex-ish object seen from one side.

    cam_high only returns the face turned toward it, so the visible points sit
    in front of the body axis.  Take the view direction d (camera -> object),
    the nearest visible point along d, and the object's width across d; assume
    a round footprint and push half that width back along d.
    """
    xy = pts[:, :2]
    c = xy.mean(0)
    d = c - cam_xy
    n = float(np.hypot(d[0], d[1]))
    if n < 1e-6:
        return c
    d = d / n
    q = np.array([-d[1], d[0]])
    s = xy @ d
    t = xy @ q
    w = float(t.max() - t.min())
    # (d, q) is an orthonormal basis of the table plane, so a point is
    # d*(p.d) + q*(p.q).  Push the nearest visible slice half a width back.
    return d * (float(s.min()) + 0.5 * w) + q * (0.5 * float(t.max() + t.min()))


def _perceive(api):
    f = api.capture("cam_high")
    P, rgb = _cloud(f)
    cam_xy = np.asarray(f.t_base_cam, float)[:2, 3]
    cs = _clusters(P, rgb)
    return cs, cam_xy


def run(api):
    api.log("instruction: %s" % api.instruction())
    cs, cam_xy = _perceive(api)
    for c in cs:
        api.log("cluster n=%d ctr=(%.3f,%.3f,%.3f) zmax=%.3f ywid=%.3f rch=%.3f lum=%.3f"
                % (c["n"], c["ctr"][0], c["ctr"][1], c["ctr"][2], c["zmax"],
                   c["ywid"], c["rch"], c["lum"]))
    if not cs:
        return "no clusters"

    # the basket is by far the largest cluster (a big open container);
    # the bbq sauce is the reddest of the remaining table objects.
    basket = cs[0]
    cand = [c for c in cs if c is not basket]
    if not cand:
        return "only one cluster"
    target = max(cand, key=lambda c: c["rch"])
    tgt = _axis_centre(target["pts"], cam_xy)
    bx, by = float(basket["bbox"][0]), float(basket["bbox"][1])
    api.log("TARGET xy=(%.4f,%.4f) rch=%.3f h=%.3f | BASKET xy=(%.4f,%.4f) rim=%.3f"
            % (tgt[0], tgt[1], target["rch"], target["zmax"], bx, by, basket["zmax"]))

    # --- calibrate the fingertip offset: press an open gripper onto bare table
    api.grip(OPEN_W)
    api.settle(0.3)
    px, py = PROBE_XY
    api.move([px, py, 0.26], seconds=2.0)
    zs = []
    for zc in (0.20, 0.14, 0.09, 0.05, 0.02, -0.01, -0.03):
        r = api.move([px, py, zc], seconds=1.5)
        e = api.eef()
        zs.append(float(e[2]))
        api.log("probe cmd %.3f -> eef_z %.4f resid %.4f" % (zc, e[2], r))
    off = min(zs[-3:])
    api.log("FINGERTIP OFFSET = %.4f" % off)
    api.move([px, py, 0.26], seconds=2.0)

    # --- grasp, with one re-perceive/retry if the jaws come up empty
    held = False
    for attempt in (0, 1):
        api.grip(OPEN_W)
        api.move([tgt[0], tgt[1], off + 0.16], seconds=2.5)
        api.move([tgt[0], tgt[1], off + GRASP_H], seconds=2.0)
        api.log("a%d at grasp eef %s" % (attempt, np.round(api.eef(), 4).tolist()))
        api.grip(0.0)
        api.settle(0.6)
        api.log("a%d closed %s" % (attempt, api.gripper()))
        api.move([tgt[0], tgt[1], off + 0.22], seconds=2.0)
        api.settle(0.3)
        g = api.gripper()
        api.log("a%d after lift eef %s grip %s"
                % (attempt, np.round(api.eef(), 4).tolist(), g))
        held = g["effort"] >= HOLD_EFFORT and g["width_m"] >= HOLD_WMIN
        if held:
            break
        api.log("a%d NOT HOLDING - re-perceiving" % attempt)
        api.grip(OPEN_W)
        api.move([px, py, 0.26], seconds=2.5)
        cs2, cam_xy2 = _perceive(api)
        cand2 = [c for c in cs2[1:]] if len(cs2) > 1 else []
        if not cand2:
            break
        t2 = max(cand2, key=lambda c: c["rch"])
        tgt = _axis_centre(t2["pts"], cam_xy2)
        api.log("retry target xy=(%.4f,%.4f) rch=%.3f" % (tgt[0], tgt[1], t2["rch"]))
    if not held:
        api.log("giving up on the grasp")
        return "no grasp"

    # --- place: the bottle hangs GRASP_H below the fingertips, and the basket
    # rim is the tallest thing on the carry, so clear rim + GRASP_H.
    drop_tip = basket["zmax"] + GRASP_H + 0.06
    api.move([bx, by, off + drop_tip + 0.05], seconds=3.0)
    api.log("over basket eef %s grip %s"
            % (np.round(api.eef(), 4).tolist(), api.gripper()))
    api.move([bx, by, off + drop_tip], seconds=2.0)
    api.grip(OPEN_W)
    api.settle(0.8)
    api.log("released grip %s eef %s"
            % (api.gripper(), np.round(api.eef(), 4).tolist()))
    api.move([bx, by, off + drop_tip + 0.06], seconds=2.0)
    api.settle(0.5)
    return "v3 done"
