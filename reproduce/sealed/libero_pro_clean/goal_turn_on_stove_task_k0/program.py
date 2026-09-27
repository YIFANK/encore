"""c2clean goal_turn_on_stove_task_k0 -- v7d (all).

The stove control is a black dial standing on the table behind the hob slab
with a fin handle on it.  The fin is located at runtime from cam_high RGB-D and
then disturbed.  MODE = all

v6 showed that grabbing the fin FIRST and twisting can shove the whole dial and
poison every later attempt, so the invasive grasp is ordered last here.
"""
import numpy as np

PROVENANCE = {
    "Z_TABLE": {"source": "debug-seed cam_high depth: bare-table pixels deproject to z=0.901", "allowed": True},
    "TIP_OFF": {"source": "v4 debug run: closed jaws driven into bare table stall at eef z=0.9177 -> 0.0167",
                "allowed": True},
    "SEARCH_BOX": {"source": "debug-seed cam_high height map: the dial+fin is the only dark structure standing "
                             "above the hob slab (top z=0.932) inside x[-0.55,-0.25], y[0.05,0.40]",
                   "allowed": True},
    "DARK_CUT": {"source": "debug-seed cam_high RGB: dial/fin pixels read 16..45 per channel, slab 69..101",
                 "allowed": True},
    "FIN_BAND": {"source": "debug-seed depth: the fin ridge is the top 12 mm of that dark cluster, 24 mm wide in y",
                 "allowed": True},
    "PRESS_X_INSET": {"source": "v5a debug run: press at x=-0.428 vs fin rear extreme x=-0.449 -> +0.020",
                      "allowed": True},
    "PRESS_DEPTH": {"source": "v5a debug run: predicate fired with the fingertip driven to z=0.947, 11 mm below "
                              "the fin top; commanded 30 mm below for margin", "allowed": True},
    "SWEEP_Y": {"source": "v5c debug run: sweeping the blade from y=0.144 to y=0.188 through the fin at y=0.211 "
                          "fired the predicate; +-0.075 about the fin covers that", "allowed": True},
    "GRASP_DZ": {"source": "v4/v5b debug runs: jaws opened to 0.08 and lowered to fin_top-0.012 close on the fin "
                           "(w 0.021..0.053, effort 3.0)", "allowed": True},
    "FALLBACK_FIN": {"source": "debug-seed cam_high RGB-D (51,52,53,55,57): fin ridge centre (-0.406, 0.211), "
                               "top z=0.960, rear x=-0.449", "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics: tool z along world -z, jaws close along world y",
               "allowed": True},
}

TIP_OFF = 0.0167
BOX = (-0.55, -0.25, 0.05, 0.40, 0.935, 1.02)
DARK_CUT = 60
FIN_BAND = 0.012
PRESS_X_INSET = 0.020
PRESS_DEPTH = 0.030
SWEEP_Y = 0.075
GRASP_DZ = 0.012
FALLBACK = (-0.406, 0.211, 0.960, -0.449)
MODE = "all"

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def st(api, tag):
    e = np.asarray(api.eef()); g = api.gripper()
    api.log("ST %-12s eef=%.4f,%.4f,%.4f w=%.4f eff=%.2f" % (tag, e[0], e[1], e[2], g["width_m"], g["effort"]))


def find_fin(api):
    f = api.capture("cam_high")
    rgb = np.asarray(f.rgb).astype(np.int32)
    dep = np.asarray(f.depth, dtype=np.float64)
    K = np.asarray(f.intrinsics); T = np.asarray(f.t_base_cam)
    H, W = dep.shape
    vv, uu = np.mgrid[0:H, 0:W]
    fl, cx, cy = K[0, 0], K[0, 2], K[1, 2]
    d = np.where(np.isfinite(dep) & (dep > 0.05), dep, np.nan)
    P = np.stack([(uu - cx) * d / fl, (vv - cy) * d / fl, d, np.ones_like(d)], -1) @ T.T
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    xlo, xhi, ylo, yhi, zlo, zhi = BOX
    m = (rgb.max(-1) < DARK_CUT) & np.isfinite(Z)
    m &= (X > xlo) & (X < xhi) & (Y > ylo) & (Y < yhi) & (Z > zlo) & (Z < zhi)
    n = int(m.sum())
    api.log("PERC px=%d" % n)
    if n < 40:
        api.log("PERC fallback (sparse)")
        return FALLBACK
    zt = float(np.percentile(Z[m], 99.0))
    band = m & (Z > zt - FIN_BAND)
    xs, ys = X[band], Y[band]
    out = (float(np.median(xs)), float(np.median(ys)), zt, float(np.percentile(xs, 5.0)))
    api.log("PERC fin x=%.4f y=%.4f top=%.4f rear=%.4f n=%d yspan=%.3f"
            % (out[0], out[1], out[2], out[3], int(band.sum()), float(ys.max() - ys.min())))
    if not (0.0 < ys.max() - ys.min() < 0.12) or not (xlo < out[0] < xhi) or not (0.93 < zt < 1.01):
        api.log("PERC implausible -> fallback")
        return FALLBACK
    return out


def do_press(api, fx, fy, ftop, frear):
    px = frear + PRESS_X_INSET
    api.grip(0.0); api.settle(0.3)
    api.move([px, fy, ftop + 0.060 + TIP_OFF], rotation=R_DOWN, seconds=3.0)
    st(api, "over_rear")
    for dz in (0.017, -0.003, -0.016, -PRESS_DEPTH):
        api.move([px, fy, ftop + dz + TIP_OFF], rotation=R_DOWN, seconds=2.0)
        st(api, "press%+.3f" % dz)


def do_sweep(api, fx, fy, ftop, frear):
    px = frear + PRESS_X_INSET
    zs = ftop - 0.014 + TIP_OFF
    api.grip(0.0); api.settle(0.3)
    api.move([px, fy - SWEEP_Y, ftop + 0.055 + TIP_OFF], rotation=R_DOWN, seconds=3.0)
    st(api, "sweep_over")
    api.move([px, fy - SWEEP_Y, zs], rotation=R_DOWN, seconds=2.2)
    st(api, "sweep_down")
    api.move([px, fy + SWEEP_Y, zs], rotation=R_DOWN, seconds=3.0)
    st(api, "sweep+y")


def do_grasp(api, fx, fy, ftop, frear):
    api.grip(0.08); api.settle(0.3)
    api.move([fx, fy, ftop + 0.050 + TIP_OFF], rotation=R_DOWN, seconds=3.0)
    st(api, "pre")
    api.move([fx, fy, ftop - GRASP_DZ + TIP_OFF], rotation=R_DOWN, seconds=2.5)
    st(api, "at")
    api.grip(0.0); api.settle(0.6)
    st(api, "closed")


def run(api):
    api.log("instruction %r MODE=%s" % (api.instruction(), MODE))
    st(api, "home")
    fx, fy, ftop, frear = find_fin(api)
    steps = {"press": (do_press,), "sweep": (do_sweep,), "grasp": (do_grasp,),
             "all": (do_press, do_sweep, do_grasp)}[MODE]
    for fn in steps:
        fn(api, fx, fy, ftop, frear)
    api.log("DONE v7d")
