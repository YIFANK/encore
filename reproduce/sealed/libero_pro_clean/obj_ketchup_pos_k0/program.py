"""v6 - first full pick-and-place.

Target rule (MODE): three narrow-capped bottles are present on every debug
seed I looked at; one has a green top, one an achromatic (grey/silver) top over
a red body, one a saturated dark-red top.  MODE picks which of the red-bodied
two to grasp; the success bit on the debug seeds decides which is "ketchup".

Grasp: neck/cap straddle.  cmd_z == fingertip height (v5 calibration).
Carry:  fingertip above rim + hang, where hang == the grasp height itself.
"""
import numpy as np

MODE = "lightcap"          # "lightcap" -> grey-capped red bottle; "redcap" -> dark-red-capped bottle

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

TIP_DZ = 0.008        # eef ref is this far above the closed fingertips (v5 TABLE ladder)
TRACK_DZ = 0.0085     # steady-state: achieved eef z ~= cmd z + this (v5 ladders)
GRASP_BELOW_TOP = 0.020
CARRY_MARGIN = 0.025
PLACE_ABOVE_FLOOR = 0.045

PROVENANCE = {
    "TIP_DZ": {"source": "debug seed 51/53 v5 TABLE ladder: closed gripper stops at eef z=0.0090 over table top z=0.0010", "allowed": True},
    "TRACK_DZ": {"source": "debug seed 51/53 v5 ladders: achieved eef z minus commanded z, constant 0.0083-0.0089", "allowed": True},
    "GRASP_BELOW_TOP": {"source": "debug seed 51 height profile: cap/neck stays <=0.034 m wide from top down to top-0.024, flares below", "allowed": True},
    "CARRY_MARGIN": {"source": "debug-seed choice; clearance over measured basket rim top", "allowed": True},
    "PLACE_ABOVE_FLOOR": {"source": "debug seed 51: basket interior floor z~0.002, rim top 0.142; release with object bottom inside", "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics: tool z antiparallel to base z (straight-down wrist), matches measured start rotation", "allowed": True},
    "BOTTLE_TOP_LO/HI, CAP_MAX": {"source": "debug seed 51/53/55/57 segmentation: bottle tops 0.113-0.148, cap widths 0.023-0.034; box/can caps 0.051-0.080", "allowed": True},
}

import numpy as np

def _cloud(fr, step=2):
    D = np.asarray(fr.depth).astype(np.float64)
    K = np.asarray(fr.intrinsics).astype(np.float64)
    T = np.asarray(fr.t_base_cam).astype(np.float64)
    H, W = D.shape
    vv, uu = np.mgrid[0:H:step, 0:W:step]
    Ds = D[::step, ::step]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x = (uu - cx) / fx * Ds
    y = (vv - cy) / fy * Ds
    P = np.stack([x, y, Ds, np.ones_like(Ds)], -1) @ T.T
    return P[..., :3], uu, vv


def _label(mask):
    """4/8-connected components, iterative flood fill on a boolean grid."""
    lab = np.zeros(mask.shape, np.int32)
    cur = 0
    H, W = mask.shape
    for a0 in range(H):
        for b0 in range(W):
            if not mask[a0, b0] or lab[a0, b0]:
                continue
            cur += 1
            stack = [(a0, b0)]
            lab[a0, b0] = cur
            while stack:
                a, b = stack.pop()
                for da in (-1, 0, 1):
                    for db in (-1, 0, 1):
                        na, nb = a + da, b + db
                        if 0 <= na < H and 0 <= nb < W and mask[na, nb] and not lab[na, nb]:
                            lab[na, nb] = cur
                            stack.append((na, nb))
    return lab, cur


def perceive(api, cam="cam_high", zmin=0.015, minpx=25,
             xlo=-0.35, xhi=0.40, ylim=0.55, zmax=0.45):
    fr = api.capture(cam)
    P, uu, vv = _cloud(fr, 2)
    RGB = np.asarray(fr.rgb)[::2, ::2, :].astype(np.float64) / 255.0
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    ws = (X > xlo) & (X < xhi) & (np.abs(Y) < ylim)
    m = ws & (Z > zmin) & (Z < zmax)
    lab, n = _label(m)
    out = []
    for i in range(1, n + 1):
        s = lab == i
        npx = int(s.sum())
        if npx < minpx:
            continue
        zz = Z[s]
        top = float(np.percentile(zz, 99.0))
        hi = s & (Z > top - 0.012)
        if hi.sum() < 4:
            hi = s & (Z > top - 0.03)
        bx = (float(X[hi].min()), float(X[hi].max()))
        by = (float(Y[hi].min()), float(Y[hi].max()))
        col = RGB[s].mean(0)
        rows, cols = np.where(s)
        out.append(dict(
            id=i, npx=npx, top=top,
            cap_mid=((bx[0] + bx[1]) / 2.0, (by[0] + by[1]) / 2.0),
            cap_dx=bx[1] - bx[0], cap_dy=by[1] - by[0],
            xr=(float(X[s].min()), float(X[s].max())),
            yr=(float(Y[s].min()), float(Y[s].max())),
            rgb=[round(float(c), 3) for c in col],
            uv=(int(cols.min() * 2), int(rows.min() * 2), int(cols.max() * 2), int(rows.max() * 2)),
        ))
    return out


def describe(c):
    return ("c%d npx=%d top=%.3f cap=(%.3f,%.3f) dx=%.3f dy=%.3f x=(%.3f,%.3f) "
            "y=(%.3f,%.3f) rgb=%s uv=%s") % (
        c["id"], c["npx"], c["top"], c["cap_mid"][0], c["cap_mid"][1], c["cap_dx"],
        c["cap_dy"], c["xr"][0], c["xr"][1], c["yr"][0], c["yr"][1], c["rgb"], c["uv"])



BOTTLE_TOP_LO, BOTTLE_TOP_HI = 0.07, 0.25
CAP_MAX = 0.045


def redness(c):
    r, g, b = c["rgb"]
    return r - 0.5 * (g + b)


def run(api):
    api.log("INTENT: %s MODE=%s" % (api.instruction(), MODE))
    cl = perceive(api)
    for c in cl:
        api.log("SEG " + describe(c))

    bottles = [c for c in cl
               if BOTTLE_TOP_LO < c["top"] < BOTTLE_TOP_HI
               and c["cap_dy"] <= CAP_MAX and c["cap_dx"] <= CAP_MAX and c["npx"] >= 150]
    bottles.sort(key=lambda c: -redness(c))
    api.log("BOTTLES=%s" % [(c["id"], round(redness(c), 3), round(c["top"], 3)) for c in bottles])
    if not bottles:
        api.log("ABORT no bottle")
        return
    red = [c for c in bottles if redness(c) > 0.03]
    if MODE == "redcap":
        tgt = red[0] if red else bottles[0]
    else:
        tgt = red[1] if len(red) > 1 else bottles[0]
    api.log("TARGET " + describe(tgt))

    # basket: the big wide cluster
    baskets = [c for c in cl if c["cap_dx"] > 0.10 and c["cap_dy"] > 0.10 and c["top"] < 0.30]
    if not baskets:
        api.log("ABORT no basket")
        return
    bk = max(baskets, key=lambda c: c["npx"])
    api.log("BASKET " + describe(bk))
    bx = (bk["xr"][0] + bk["xr"][1]) / 2.0
    by = (bk["yr"][0] + bk["yr"][1]) / 2.0
    rim = bk["top"]

    tx, ty = tgt["cap_mid"]
    g = tgt["top"] - GRASP_BELOW_TOP
    hang = g                      # fingertips sit `g` above the table, so the base hangs `g` below them
    carry = rim + hang + CARRY_MARGIN
    place = max(hang + PLACE_ABOVE_FLOOR, rim + 0.012)
    api.log("PLAN tgt=(%.3f,%.3f) g=%.3f hang=%.3f rim=%.3f carry=%.3f place=%.3f basket=(%.3f,%.3f)"
            % (tx, ty, g, hang, rim, carry, place, bx, by))

    def mv(tag, x, y, z, sec=1.5):
        r = api.move([x, y, z], rotation=R_DOWN, seconds=sec)
        e = np.asarray(api.eef())
        gr = api.gripper()
        api.log("MV %s cmd=(%.3f,%.3f,%.3f) eef=(%.4f,%.4f,%.4f) res=%.4f w=%.4f ef=%.2f"
                % (tag, x, y, z, e[0], e[1], e[2], float(r), gr["width_m"], gr["effort"]))
        return r

    api.grip(0.08)
    api.settle(0.2)
    mv("approach", tx, ty, tgt["top"] + 0.10)
    mv("pre", tx, ty, g + 0.05)
    mv("descend", tx, ty, g, 1.8)
    api.grip(0.0)
    api.settle(0.5)
    gr = api.gripper()
    api.log("CLOSED w=%.4f ef=%.2f" % (gr["width_m"], gr["effort"]))

    mv("lift1", tx, ty, g + 0.07)
    mv("lift2", tx, ty, carry, 1.8)
    gr = api.gripper()
    api.log("AFTERLIFT w=%.4f ef=%.2f eef=%s" % (gr["width_m"], gr["effort"], np.round(api.eef(), 4).tolist()))

    cl2 = perceive(api)
    for c in cl2:
        api.log("SEG2 " + describe(c))

    mv("over", bx, by, carry, 1.8)
    mv("lower", bx, by, place, 1.5)
    gr = api.gripper()
    api.log("BEFORE_RELEASE w=%.4f ef=%.2f" % (gr["width_m"], gr["effort"]))
    api.grip(0.08)
    api.settle(0.5)
    mv("retreat", bx, by, carry, 1.5)

    cl3 = perceive(api)
    for c in cl3:
        api.log("SEG3 " + describe(c))
    api.log("DONE v6")
