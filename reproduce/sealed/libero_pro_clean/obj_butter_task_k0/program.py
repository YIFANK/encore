"""v4 -- v3 with its hold check corrected (see holding()).

v2 plus three guards, because the whole debug band (51-65) is a single
prop layout and therefore says nothing about robustness to displaced props:

  1. every height is derived from the measured cluster instead of fixed
     (grasp depth below the carton's own top, carry above the basket's own rim);
  2. the horizontal tracking bias of api.move is cancelled in the command --
     measured at the hover pose and re-applied, both over the carton and over
     the basket (api.move's reported residual is NOT the tracking error: v2
     landed 27 mm off while reporting 0.010);
  3. the grasp is verified with my own sensors (the jaws must still be apart
     and squeezing after the lift) and retried once from a re-perception if it
     is not, setting the object down first rather than dropping it.
"""
import numpy as np

PROVENANCE = {
    "Z_TABLE": {"source": "debug seeds 51-65 cam_high deprojection: the modal "
                          "height of the flat support is z=0.000 in base frame",
                "allowed": True},
    "Z_OBJ_LO": {"source": "debug 51-65: object points start ~0.012 above the "
                           "table plane; below that is table noise",
                 "allowed": True},
    "Z_OBJ_HI": {"source": "debug 51-65: all props top out at z<=0.15; the "
                           "parked arm/gripper sits at z~0.26-0.30 and would "
                           "fuse with the carton without this cap",
                 "allowed": True},
    "WORK_X": {"source": "debug 51-65: all props+basket lie in x (-0.20,+0.30); "
                         "walls/floor deproject outside",
               "allowed": True},
    "WORK_Y": {"source": "debug 51-65: all props+basket lie in |y|<0.40",
               "allowed": True},
    "CLUSTER_CELL": {"source": "grid size chosen on debug 51/53 so the 6 props "
                               "and the basket separate into 7 components "
                               "(0.02 fused the carton with the dressing "
                               "bottle)",
                     "allowed": True},
    "MIN_PX": {"source": "debug 51-65: smallest real prop had 480 px at full "
                         "resolution; noise components were <40",
               "allowed": True},
    "BASKET_MIN_EXT": {"source": "debug 51-65: basket footprint 0.16x0.17 m, "
                                 "every prop <0.08 m",
                       "allowed": True},
    "TALL_MIN": {"source": "debug 51-65: carton/bottles top >=0.11, the can "
                           "0.08 and the flat boxes 0.02-0.03",
                 "allowed": True},
    "FREE_SPOT": {"source": "debug 51-65: no cluster within 0.10 m of "
                            "(0.00,-0.38); bare table there",
                  "allowed": True},
    "GRASP_BELOW_TOP": {"source": "debug 51-65: carton top 0.140; tips 0.055 "
                                  "below it bite its upper half, clear of both "
                                  "the lid seam and the table",
                        "allowed": True},
    "CARRY_ABOVE_RIM": {"source": "debug 51-65: basket rim top 0.142 and the "
                                  "carton hangs ~0.085 below the tips, so tips "
                                  "0.12 above the rim clear it",
                        "allowed": True},
    "OPEN_W": {"source": "measured at reset on debug 51-65: api.gripper() "
                         "reports width 0.0778-0.0799 fully open",
               "allowed": True},
    "HOLD_W_MIN": {"source": "debug 51-65: an empty close collapses the jaws "
                             "toward 0; a real carton bite reads width 0.0530, "
                             "matching its measured 0.051 top-band y extent",
                   "allowed": True},
    "BIAS_MAX": {"source": "debug 51-65: observed api.move tracking bias was "
                           "7-27 mm, so corrections beyond 0.04 m would be a "
                           "perception fault, not a tracking one",
                 "allowed": True},
    "BIAS_MIN": {"source": "debug 51-65: sub-5 mm offsets are within the "
                           "controller's own convergence band",
                 "allowed": True},
}

Z_TABLE = 0.0
Z_OBJ_LO = 0.012
Z_OBJ_HI = 0.20
WORK_X = (-0.45, 0.45)
WORK_Y = 0.75
CLUSTER_CELL = 0.010
MIN_PX = 40
BASKET_MIN_EXT = 0.12
TALL_MIN = 0.10
FREE_SPOT = (0.00, -0.38)
GRASP_BELOW_TOP = 0.055
CARRY_ABOVE_RIM = 0.12
OPEN_W = 0.078
HOLD_W_MIN = 0.010
BIAS_MAX = 0.04
BIAS_MIN = 0.005


# ---------------------------------------------------------------- perception
def point_cloud(api, cam="cam_high"):
    f = api.capture(cam)
    rgb = np.asarray(f.rgb, np.float32)
    dep = np.asarray(f.depth, np.float32)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    H, W = dep.shape
    vv, uu = np.mgrid[0:H, 0:W]
    x = (uu - K[0, 2]) / K[0, 0] * dep
    y = (vv - K[1, 2]) / K[1, 1] * dep
    pc = np.stack([x, y, dep, np.ones_like(dep)], -1) @ T.T
    return rgb, pc[..., :3]


def clusters(api):
    rgb, pc = point_cloud(api)
    X, Y, Z = pc[..., 0], pc[..., 1], pc[..., 2]
    m = ((X > WORK_X[0]) & (X < WORK_X[1]) & (np.abs(Y) < WORK_Y)
         & (Z > Z_TABLE + Z_OBJ_LO) & (Z < Z_OBJ_HI))
    pts, cols = pc[m], rgb[m]
    gi = np.floor(pts[:, 0] / CLUSTER_CELL).astype(int)
    gj = np.floor(pts[:, 1] / CLUSTER_CELL).astype(int)
    keys = {}
    for n, k in enumerate(zip(gi, gj)):
        keys.setdefault(k, []).append(n)
    seen, out = set(), []
    for k in keys:
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            c = stack.pop()
            comp.append(c)
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    nb = (c[0] + da, c[1] + db)
                    if nb in keys and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        mem = np.array([n for c in comp for n in keys[c]])
        if len(mem) < MIN_PX:
            continue
        p, c = pts[mem], cols[mem]
        top = float(np.percentile(p[:, 2], 98))
        band = p[p[:, 2] > top - 0.03]
        out.append({
            "n": len(mem),
            "xy": p[:, :2].mean(0),
            "top": top,
            "top_xy": band[:, :2].mean(0),
            "ext": p[:, :2].max(0) - p[:, :2].min(0),
            "top_ext": band[:, :2].max(0) - band[:, :2].min(0),
            "rgb": c.mean(0),
        })
    return out


def pick_targets(api, cs, tag=""):
    for c in cs:
        api.log("CLUS%s n=%d xy=(%+.3f,%+.3f) top=%.3f top_xy=(%+.3f,%+.3f) "
                "ext=(%.3f,%.3f) top_ext=(%.3f,%.3f) rgb=(%.0f,%.0f,%.0f) "
                "gmb=%.1f" % (tag, c["n"], c["xy"][0], c["xy"][1], c["top"],
                              c["top_xy"][0], c["top_xy"][1],
                              c["ext"][0], c["ext"][1],
                              c["top_ext"][0], c["top_ext"][1],
                              c["rgb"][0], c["rgb"][1], c["rgb"][2],
                              c["rgb"][1] - c["rgb"][2]))
    baskets = [c for c in cs if max(c["ext"]) > BASKET_MIN_EXT]
    basket = max(baskets, key=lambda c: c["n"]) if baskets else None
    props = [c for c in cs if c is not basket and c["top"] > TALL_MIN]
    # "orange juice": among the tall props, the one whose mean colour is most
    # yellow (green channel well above blue).  Ranked, not thresholded: the
    # carton scores 35 against 19 for the runner-up on every debug seed.
    oj = max(props, key=lambda c: c["rgb"][1] - c["rgb"][2]) if props else None
    return oj, basket


# -------------------------------------------------------------------- motion
def aimed_move(api, tx, ty, tz, seconds=2.0, tag=""):
    """Move to (tx,ty,tz), then cancel whatever horizontal tracking bias the
    controller left by re-issuing the command shifted by the measured error."""
    api.move([tx, ty, tz], seconds=seconds)
    e = api.eef()
    ex, ey = tx - e[0], ty - e[1]
    err = float(np.hypot(ex, ey))
    api.log("AIM%s target=(%+.3f,%+.3f,%+.3f) got=(%+.3f,%+.3f,%+.3f) err=%.4f"
            % (tag, tx, ty, tz, e[0], e[1], e[2], err))
    if BIAS_MIN < err < BIAS_MAX:
        api.move([tx + ex, ty + ey, tz], seconds=1.5)
        e = api.eef()
        api.log("AIM%s corrected got=(%+.3f,%+.3f,%+.3f) err=%.4f"
                % (tag, e[0], e[1], e[2],
                   float(np.hypot(tx - e[0], ty - e[1]))))
    return api.eef()


def holding(api, expect_w):
    """My own grasp sensor.  `api.grip(0.0)` always commands a full close, so
    jaws that stop with a gap AND report squeeze have something between them;
    an empty close collapses to ~0.  v3 also bounded the gap from above against
    the object's SHORT extent, but the jaws close along base y and the carton
    presents its LONG extent there -- a good 0.053 bite was scored as a failure
    and dropped from carry height.  The upper bound is gone."""
    g = api.gripper()
    w, eff = g["width_m"], g["effort"]
    ok = (eff >= 3.0) and (w > HOLD_W_MIN)
    return ok, g


def try_grasp(api, oj, d, attempt):
    gx, gy = float(oj["top_xy"][0]), float(oj["top_xy"][1])
    z_grasp = max(oj["top"] - GRASP_BELOW_TOP, 0.02) + d
    api.grip(OPEN_W)
    api.settle(0.3)
    aimed_move(api, gx, gy, max(oj["top"], 0.15) + d + 0.14, seconds=2.0,
               tag="/hover%d" % attempt)
    r = api.move([gx, gy, z_grasp], seconds=2.5)
    api.log("DESCEND%d eef=%s residual=%.4f grip=%s"
            % (attempt, np.round(api.eef(), 4).tolist(), r, api.gripper()))
    api.grip(0.0)
    api.settle(0.6)
    api.log("CLOSED%d grip=%s" % (attempt, api.gripper()))
    return gx, gy


def run(api):
    api.log("INSTRUCTION %r" % api.instruction())
    cs = clusters(api)
    oj, basket = pick_targets(api, cs)
    if oj is None or basket is None:
        return "perception failed"
    api.log("TARGET oj xy=(%+.3f,%+.3f) top=%.3f top_ext=(%.3f,%.3f)"
            % (oj["top_xy"][0], oj["top_xy"][1], oj["top"],
               oj["top_ext"][0], oj["top_ext"][1]))
    api.log("TARGET basket xy=(%+.3f,%+.3f) top=%.3f"
            % (basket["top_xy"][0], basket["top_xy"][1], basket["top"]))

    api.grip(OPEN_W)
    api.settle(0.3)

    # --- calibrate: press the open fingertips onto the bare table.  The eef z
    # where the descent stalls IS the tip-to-eef offset, since the table is the
    # measured z = 0 plane.
    fx, fy = FREE_SPOT
    api.move([fx, fy, 0.22], seconds=2.0)
    r = api.move([fx, fy, -0.06], seconds=3.0)
    d = float(api.eef()[2]) - Z_TABLE
    api.log("CALIB touch eef=%s residual=%.4f tip_offset=%.4f"
            % (np.round(api.eef(), 4).tolist(), r, d))
    api.move([fx, fy, 0.28], seconds=2.0)

    expect_w = float(oj["top_ext"][1])   # jaws close along base y
    z_carry = basket["top"] + CARRY_ABOVE_RIM + d

    # --- grasp, verified by the jaw gap surviving the lift; one retry from a
    # fresh perception if it does not.
    ok = False
    for attempt in (1, 2):
        gx, gy = try_grasp(api, oj, d, attempt)
        r = api.move([gx, gy, z_carry], seconds=2.5)
        api.settle(0.3)
        ok, g = holding(api, expect_w)
        api.log("LIFTED%d eef=%s residual=%.4f grip=%s expect_w=%.3f ok=%s"
                % (attempt, np.round(api.eef(), 4).tolist(), r, g,
                   expect_w, ok))
        if ok or attempt == 2:
            break
        # set whatever is in the jaws back down before re-perceiving: opening
        # at carry height would drop it 0.25 m onto the table.
        api.move([gx, gy, oj["top"] + d + 0.01], seconds=2.0)
        api.grip(OPEN_W)
        api.settle(0.4)
        api.move([gx, gy, z_carry], seconds=2.0)
        cs2 = clusters(api)
        oj2, b2 = pick_targets(api, cs2, tag="/retry")
        if oj2 is not None:
            oj = oj2
            expect_w = float(oj["top_ext"][1])
        if b2 is not None:
            basket = b2
            z_carry = basket["top"] + CARRY_ABOVE_RIM + d
        api.log("RETRY re-aim oj xy=(%+.3f,%+.3f) top=%.3f"
                % (oj["top_xy"][0], oj["top_xy"][1], oj["top"]))

    # --- place
    bx, by = float(basket["top_xy"][0]), float(basket["top_xy"][1])
    aimed_move(api, bx, by, z_carry, seconds=3.0, tag="/basket")
    api.settle(0.3)
    _, g = holding(api, expect_w)
    api.log("OVERBASKET eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(), g))
    api.grip(OPEN_W)
    api.settle(1.0)
    api.log("RELEASED eef=%s grip=%s" % (np.round(api.eef(), 4).tolist(),
                                         api.gripper()))
    api.move([bx, by, z_carry + 0.06], seconds=2.0)
    api.settle(0.5)

    for c in clusters(api):
        api.log("POST n=%d xy=(%+.3f,%+.3f) top=%.3f rgb=(%.0f,%.0f,%.0f)"
                % (c["n"], c["xy"][0], c["xy"][1], c["top"],
                   c["rgb"][0], c["rgb"][1], c["rgb"][2]))
    return "v4 done ok=%s" % ok
