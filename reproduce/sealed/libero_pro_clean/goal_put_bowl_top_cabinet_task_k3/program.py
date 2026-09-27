"""v7 — widen the chord. The dish is an INVERTED cone.

Reading the five closes I have logged as radius measurements
(r_contact = sqrt(off^2 + (gap/2)^2)):

    tip 0.9019 (floored)  off 0.0606  gap 0.0569  -> r = 0.0670
    tip 0.9051            off 0.0603  gap 0.0261  -> r = 0.0617
    tip 0.9113            off 0.0613  gap 0.0010  -> r < 0.0613
    tip 0.9164            off 0.0598  gap 0.0010  -> r < 0.0598

the dish is WIDEST AT THE TABLE (r 0.067-0.068, which is also what the
oblique cam_high footprint measures) and NARROWS going up, to r < 0.060 at
its rim crest 0.9193. So its outer wall leans inward as it rises ~28 deg from
vertical, and there is no crest overhang: a fingertip at r slightly greater
than the wall radius at its own height has its whole shank in free space.

That kills the constraint I had assumed in v3/v4 (descend outside r=0.070),
and it matters because a chord clamp on a disc ejects the disc along the
chord's perpendicular with a net force 2F*(off/r) against a friction budget
2*mu*F. v3 clamped at off/r = 0.0603/0.0617 = 0.98 -- needing mu ~ 1 -- and
crept out. Descending at off = sqrt((r_wall+0.002)^2 - jaw_half^2) instead
puts the clamp at off/r = 0.81 and should open the gap from 0.026 to ~0.072.

Also: NO re-gripping. v6 re-issued grip(0.0) at every carry hop and the gap
fell 0.0269 -> 0.0184 -> 0.0010; v3, with a single close, held 0.0167 over
the same lift. Re-applying the closing force is what ratchets the dish out.
"""
import os
import time
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug-seed cam_high depth: dominant z mode (0.901)",
                "allowed": True},
    "CAB_TOP_BAND": {"source": "debug-seed cam_high depth: second z mode, the "
                               "cabinet top plane at 1.1269", "allowed": True},
    "PLATE_BAND": {"source": "debug-seed cam_high depth: dish ztop 0.9193",
                   "allowed": True},
    "TIP_DZ": {"source": "debug-seed v2 probe: open-gripper descent onto bare "
                         "table stalls at eef z 0.9093, table 0.901",
               "allowed": True},
    "JAW_HALF": {"source": "debug-seed api.gripper() open width 0.0799",
                 "allowed": True},
    "WALL_R_FRAC": {"source": "debug-seed v3/v4 closes read as radii: the wall "
                              "radius 4.5 mm above the table (0.0617) over the "
                              "cam_high footprint radius (0.0682) = 0.905",
                    "allowed": True},
    "FINGER_CLEAR": {"source": "debug-seed v4: at 2 mm clearance the fingers "
                               "descended past the wall without fouling",
                     "allowed": True},
    "GRASP_EEF_Z": {"source": "debug-seed v3: the fingertip height that bit and "
                              "lifted the dish was eef z 0.9131 (tip 0.9051)",
                    "allowed": True},
    "GAP_HOLD_MIN": {"source": "debug-seed v3/v4: a real bite reads 0.017-0.057, "
                               "empty jaws read 0.0010-0.0015", "allowed": True},
    "PLACE_XY": {"source": "k3 pack keyframes bowl-release eef xy, and v5 "
                           "confirmed (0.006,-0.180) lands on the top",
                 "allowed": True},
    "PLACE_CLEAR": {"source": "k3 pack: release eef z 1.147-1.175 over a cabinet "
                              "top I measure at 1.1269", "allowed": True},
    "CARRY_Z": {"source": "k3 pack ee_path6: transit apex z 1.19-1.26",
                "allowed": True},
    "CARRY_STEP": {"source": "debug-seed v3: one 0.24 m goto emptied the jaws",
                   "allowed": True},
    "DOWN_ROT": {"source": "debug-seed api.tool_rotation() at episode start",
                 "allowed": True},
    "DUMP_DIR": {"source": "clean-room allowed write location (pack dir)",
                 "allowed": True},
}

DUMP_DIR = "/mnt/data/YifanKang/Heron/packs/c2clean_goal_put_bowl_top_cabinet_task_k3/dbg"
TABLE_Z = 0.901
CAB_TOP_BAND = (1.110, 1.140)
PLATE_BAND = (0.905, 0.932)
TIP_DZ = 0.0085
JAW_HALF = 0.0399
WALL_R_FRAC = 0.905
FINGER_CLEAR = 0.002
GRASP_EEF_Z = 0.9135
GAP_HOLD_MIN = 0.012
PLACE_XY = (0.010, -0.180)
PLACE_CLEAR = 0.020
CARRY_Z = 1.215
CARRY_STEP = 0.045

DOWN_ROT = np.array([[0.998, 0.0, -0.057],
                     [0.0, -1.0, 0.0],
                     [-0.057, 0.0, -0.998]])
RZ90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
YAW_ROT = RZ90 @ DOWN_ROT          # jaws separate along base x


def world(frame):
    dep = frame.depth.astype(np.float64)
    K, T = np.asarray(frame.intrinsics, float), np.asarray(frame.t_base_cam, float)
    h, w = dep.shape
    vv, uu = np.mgrid[0:h, 0:w]
    z = np.where(np.isfinite(dep) & (dep > 0), dep, np.nan)
    x = (uu - K[0, 2]) * z / K[0, 0]
    y = (vv - K[1, 2]) * z / K[1, 1]
    B = np.stack([x, y, z, np.ones_like(z)], -1) @ T.T
    return B[..., 0], B[..., 1], B[..., 2]


def components(mask, minpx=300):
    from collections import deque
    seen = np.zeros(mask.shape, bool)
    out = []
    for v0, u0 in np.argwhere(mask):
        if seen[v0, u0]:
            continue
        q = deque([(v0, u0)])
        seen[v0, u0] = True
        comp = []
        while q:
            v, u = q.popleft()
            comp.append((v, u))
            for dv, du in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = v + dv, u + du
                if (0 <= a < mask.shape[0] and 0 <= b < mask.shape[1]
                        and mask[a, b] and not seen[a, b]):
                    seen[a, b] = True
                    q.append((a, b))
        if len(comp) >= minpx:
            out.append(np.array(comp))
    return out


def dump(api, f, tag):
    try:
        os.makedirs(DUMP_DIR, exist_ok=True)
        np.savez_compressed(
            os.path.join(DUMP_DIR, "v7_%d_%s.npz" % (int(time.time()), tag)),
            rgb=f.rgb, depth=f.depth.astype(np.float32),
            K=f.intrinsics, T=f.t_base_cam, eef=api.eef())
    except Exception as e:
        api.log("[%s] save failed %r" % (tag, e))


def find_plate(api, tag):
    f = api.capture("cam_high")
    X, Y, Z = world(f)
    reach = (X > -0.40) & (X < 0.40) & (Y > -0.45) & (Y < 0.45)
    m = reach & np.isfinite(Z) & (Z > PLATE_BAND[0]) & (Z < PLATE_BAND[1])
    best = None
    for comp in components(m):
        vs, us = comp[:, 0], comp[:, 1]
        xx, yy, zz = X[vs, us], Y[vs, us], Z[vs, us]
        dx = float(xx.max() - xx.min())
        dy = float(yy.max() - yy.min())
        if not (0.10 < dx < 0.19 and 0.10 < dy < 0.19):
            continue
        if max(dx, dy) / max(1e-6, min(dx, dy)) > 1.45:
            continue
        r = dict(n=len(comp), dx=round(dx, 3), dy=round(dy, 3),
                 cx=round(float(0.5 * (xx.max() + xx.min())), 4),
                 cy=round(float(0.5 * (yy.max() + yy.min())), 4),
                 ztop=round(float(np.percentile(zz, 98)), 4))
        r["r"] = round(0.25 * (dx + dy), 4)
        if best is None or r["n"] > best["n"]:
            best = r
    api.log("[%s] PLATE %s" % (tag, best))
    dump(api, f, tag)
    return best


def cabinet_top(api):
    f = api.capture("cam_high")
    X, Y, Z = world(f)
    m = (np.isfinite(Z) & (Z > CAB_TOP_BAND[0]) & (Z < CAB_TOP_BAND[1])
         & (X > -0.45) & (X < 0.45) & (Y > -0.5) & (Y < 0.5))
    if m.sum() < 2000:
        return 1.1269, -0.158
    zt, ny = float(np.median(Z[m])), float(np.percentile(Y[m], 98))
    api.log("cabinet top z=%.4f near_y=%.4f" % (zt, ny))
    return zt, ny


def goto(api, xyz, rot, seconds=2.5, tries=3, tol=0.004):
    tgt = np.asarray(xyz, float)
    cmd = tgt.copy()
    for _ in range(tries):
        api.move(cmd, rotation=rot, seconds=seconds)
        e = api.eef()
        err = tgt - e
        if np.linalg.norm(err) <= tol:
            return e, True
        cmd = cmd + err
    return api.eef(), False


def hop(api, p0, p1, rot, tag):
    """Incremental transport. NO re-gripping (v6 receipt)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    n = max(1, int(np.ceil(float(np.linalg.norm(p1 - p0)) / CARRY_STEP)))
    for i in range(1, n + 1):
        q = p0 + (p1 - p0) * (i / float(n))
        goto(api, q, rot, seconds=3.0, tries=1, tol=0.02)
        api.settle(0.2)
        g = api.gripper()["width_m"]
        api.log("%s hop %d/%d -> %s gap=%.4f" % (tag, i, n, np.round(q, 3).tolist(), g))
        if g < GAP_HOLD_MIN:
            api.log("%s LOST at hop %d/%d" % (tag, i, n))
            return False
    return True


def chord_offset(r_foot, shrink):
    """Chord offset that clears the wall by FINGER_CLEAR on the way down."""
    r_wall = WALL_R_FRAC * r_foot - shrink
    inner = (r_wall + FINGER_CLEAR) ** 2 - JAW_HALF ** 2
    return float(np.sqrt(max(1e-6, inner))), r_wall


def run(api):
    api.log("instruction=%r" % api.instruction())
    pl = find_plate(api, "t0")
    if pl is None:
        api.log("no dish found; abort")
        return
    zt, near_y = cabinet_top(api)

    held, gx, gy, off = False, None, None, None
    for attempt, shrink in enumerate((0.0, -0.003, 0.003)):
        off, r_wall = chord_offset(pl["r"], shrink)
        gx, gy = pl["cx"], pl["cy"] + off
        api.log("attempt %d: r_foot=%.4f r_wall=%.4f off=%.4f off/r=%.3f "
                "predicted gap=%.4f"
                % (attempt, pl["r"], r_wall, off, off / max(1e-6, r_wall),
                   2.0 * np.sqrt(max(0.0, r_wall ** 2 - off ** 2))))
        api.grip(0.08)
        api.settle(0.2)
        goto(api, [gx, gy, 0.99], YAW_ROT, seconds=3.0, tries=2, tol=0.012)
        e, ok = goto(api, [gx, gy, GRASP_EEF_Z], YAW_ROT, seconds=2.5,
                     tries=3, tol=0.003)
        api.grip(0.0)
        api.settle(0.5)
        g0 = api.gripper()["width_m"]
        api.log("attempt %d eef=%s ok=%s gap=%.4f"
                % (attempt, np.round(e, 4).tolist(), ok, g0))
        if g0 >= GAP_HOLD_MIN and hop(api, [gx, gy, GRASP_EEF_Z],
                                      [gx, gy, 0.99], YAW_ROT, "lift%d" % attempt):
            held = True
            break
        api.grip(0.08)
        api.settle(0.3)
        goto(api, [gx, gy, 1.02], YAW_ROT, seconds=2.5, tries=1, tol=0.05)
        pl = find_plate(api, "retry%d" % attempt) or pl

    if not held:
        api.log("GRASP FAILED — no carry attempted")
        find_plate(api, "final")
        return
    api.log("LIFTED; dish still on the table: %s"
            % (find_plate(api, "after_lift") is not None))

    ey = min(PLACE_XY[1], near_y - 0.020)
    ex = PLACE_XY[0]
    ez = zt + (GRASP_EEF_Z - TABLE_Z) + PLACE_CLEAR
    api.log("carry to eef=(%.4f,%.4f) release z=%.4f (dish centre y=%.4f)"
            % (ex, ey, ez, ey - off))

    ok = (hop(api, [gx, gy, 0.99], [gx, gy, CARRY_Z], YAW_ROT, "rise")
          and hop(api, [gx, gy, CARRY_Z], [ex, ey, CARRY_Z], YAW_ROT, "cross")
          and hop(api, [ex, ey, CARRY_Z], [ex, ey, ez], YAW_ROT, "descend"))
    api.log("transport ok=%s eef=%s gap=%.4f"
            % (ok, np.round(api.eef(), 4).tolist(), api.gripper()["width_m"]))
    api.grip(0.08)
    api.settle(1.0)
    goto(api, [ex, ey, CARRY_Z], YAW_ROT, seconds=3.0, tries=1, tol=0.05)
    api.settle(0.6)

    f = api.capture("cam_high")
    dump(api, f, "after_place")
    X, Y, Z = world(f)
    m = (np.isfinite(Z) & (Z > zt + 0.004) & (Z < zt + 0.05)
         & (X > -0.40) & (X < 0.30) & (Y > -0.40) & (Y < -0.05))
    api.log("px resting on the cabinet top = %d" % int(m.sum()))
    if m.sum() > 200:
        api.log("resting bbox x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.4f"
                % (X[m].min(), X[m].max(), Y[m].min(), Y[m].max(),
                   np.percentile(Z[m], 98)))
    find_plate(api, "final")
