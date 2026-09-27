"""c2 spa_bowl_on_stove_stock -- v111 (= v4 with the api.done reads excised)

v111 is v4 under FAIR_PROTOCOL v1.1.1: no `.done` attribute is read anywhere,
so the program carries no termination/success signal at runtime.  Two sites
were removed and NOTHING else changed (identical constants, identical
perception, identical motion, identical retry ladder):
  (1) `if api.done: break` in the retry ladder, after a pinch that failed the
      effort test.  Inert in practice: the ladder is reached at <350 of the
      1000-step horizon on every debug seed (results.jsonl sim_steps, run
      results/sel_c2_spa_bowl_on_stove_stock_v4: max 340), and on seeds 55/62/65
      the program passed this test after a failed try0 and went on to try1.
      No replacement -- the ladder is already bounded by len(R_SWEEP)=5 and the
      hold test is api.gripper()["effort"], which is not a success oracle.
  (2) `done={api.done}` inside the final api.log f-string: observability only,
      the term is dropped.

v2 receipts (5/8, results/fs_c2_spa_bowl_on_stove_stock_v2): perception became
repeatable (rim ring 0.108x0.108 on every seed) but the reported centres are
QUANTISED to the 12 mm cell lattice (-0.240/-0.252/-0.264/-0.276 ...), i.e. up
to 6 mm of avoidable error in the one number the pinch is most sensitive to.
Every failed try closed to 12-13 mm and then lost the bowl during the lift, and
the re-look after a failed try showed the bowl DRAGGED ~12 mm toward the
gripper -- the signature of one finger landing inside the vessel and pushing,
i.e. a jaw centre that is not on the rim-wall midline.

v4 = v3 with ONE change: the retry ladder is reordered.  v3 receipts
(results/fs_c2_spa_bowl_on_stove_stock_v3, 8/8 on seeds 51..65 odd): the fitted
ring radius is identical on every seed (R_fit 0.0531-0.0533) and pinching AT it
held on 6/8 first tries; on the two that lost the bowl, r_mid+0.010 held 2/2
while r_mid-0.010 held 0/2.  So the ladder now walks OUTWARD first.

v3: (a) sub-cell grounding -- the rim TOP RING (8 mm band) is fitted with an
algebraic circle, so the centre and the wall-midline radius come from the points
themselves; (b) the pinch radius is the fitted ring radius, with a sweep around
it as retries; (c) a 5 cm probe-lift decides the hold before committing; (d) the
release hedges by the FULL pinch radius -- the demos release 40 mm to +y of the
plate centre this cell perceives, which is exactly one rim radius.
"""
import numpy as np

PROVENANCE = {
    "A_XY": {"source": "pack.json demos: mean EEF xy at the gripper-close keyframes (demo0 t43, demo1 t30, demo2 t38)", "allowed": True},
    "GRASP_Z": {"source": "pack.json demos: mean EEF z at those close keyframes (0.9488/0.9420/0.9474)", "allowed": True},
    "B_XY": {"source": "pack.json demos: mean EEF xy at the release keyframes", "allowed": True},
    "RELEASE_Z": {"source": "pack.json demos: mean EEF z at the release keyframes (0.9306/0.9387/0.9290)", "allowed": True},
    "RIM_TOP_TO_EEF": {"source": "debug seeds 51-65: perceived rim top is table+0.0764 on every seed while the demos close at 0.9461 with table at 0.9033, i.e. 33.6 mm below the rim top", "allowed": True},
    "SEARCH_R": {"source": "debug seed 51/57 height map: the nearest non-target structure sits 0.14 m from the demo anchor", "allowed": True},
    "RING_BAND": {"source": "debug seed 51 height map: rim top table+0.076 vs bowl floor table+0.035 -- an 8 mm top band is the rim wall's own top face", "allowed": True},
    "Z_CAP": {"source": "debug seed 51 height map: nothing in the scene between the bowl rim (table+0.076) and the arm (>table+0.100)", "allowed": True},
    "CELL": {"source": "generic depth-cloud gridding resolution", "allowed": True},
    "DESCENT_BIAS": {"source": "debug-seed measurement: move_cartesian stops 7-9 mm above a commanded descent (12 mm position tolerance)", "allowed": True},
    "PROBE_DZ": {"source": "debug seeds 51-65: a lost pinch reads gap<0.005 within the first few cm of lift", "allowed": True},
    "R_SWEEP": {"source": "debug seeds 51-65 (v3 run): pinching at the fitted ring radius held 6/8 first tries; of the two that failed, r+0.010 held 2/2 and r-0.010 held 0/2, so the ladder walks outward first", "allowed": True},
    "OPEN_W": {"source": "generic gripper mechanics: FairApi grip(width>=0.025) opens", "allowed": True},
    "CLOSE_W": {"source": "generic gripper mechanics: FairApi grip(width<0.025) closes", "allowed": True},
    "APPROACH_DZ": {"source": "generic controller mechanics: clearance above a grasp before descending", "allowed": True},
    "LIFT_DZ": {"source": "generic controller mechanics: carry clearance", "allowed": True},
    "JAW_AXIS_COL": {"source": "generic gripper mechanics: the jaws separate along tool-frame y, api.tool_rotation() column 1", "allowed": True},
    "HELD_EFFORT": {"source": "FairApi contract: gripper effort is 3.0 iff the jaws hold something", "allowed": True},
}

A_XY = np.array([-0.2782, -0.0996])
GRASP_Z = 0.9461
B_XY = np.array([0.0525, 0.2384])
RELEASE_Z = 0.9328
RIM_TOP_TO_EEF = 0.0336
SEARCH_R = 0.115
RING_BAND = 0.008
Z_CAP = 0.105
CELL = 0.012
DESCENT_BIAS = 0.008
PROBE_DZ = 0.05
R_SWEEP = (0.0, +0.010, +0.020, -0.010, -0.020)
OPEN_W = 0.078
CLOSE_W = 0.0
APPROACH_DZ = 0.10
LIFT_DZ = 0.13
JAW_AXIS_COL = 1
HELD_EFFORT = 1.0


# ---------------------------------------------------------------- perception
def cloud(frame):
    d = np.asarray(frame.depth, float)
    if d.ndim == 3:
        d = d[..., 0]
    h, w = d.shape
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    m = np.isfinite(d) & (d > 0.05) & (d < 5.0)
    z = d[m]
    x = (us[m] - K[0, 2]) * z / K[0, 0]
    y = (vs[m] - K[1, 2]) * z / K[1, 1]
    return np.stack([x, y, z], 1) @ T[:3, :3].T + T[:3, 3]


def table_height(P):
    m = (np.abs(P[:, 0]) < 0.45) & (np.abs(P[:, 1]) < 0.45)
    z = P[m, 2]
    lo, hi = np.percentile(z, 1), np.percentile(z, 99)
    hist, edges = np.histogram(z, bins=np.arange(lo, hi + 0.005, 0.005))
    k = int(np.argmax(hist))
    return float(0.5 * (edges[k] + edges[k + 1]))


def circle_fit(xy):
    """Algebraic (Kasa) circle fit: x^2+y^2 = 2ax + 2by + c."""
    x, y = xy[:, 0], xy[:, 1]
    Amat = np.stack([2 * x, 2 * y, np.ones_like(x)], 1)
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(Amat, b, rcond=None)
    c = np.array([sol[0], sol[1]])
    return c, float(np.sqrt(max(sol[2] + c @ c, 1e-9)))


def ground_bowl(api, P, tz, tag=""):
    """Fit the rim TOP RING: sub-cell centre + wall-midline radius."""
    eef = api.eef()
    m = ((np.linalg.norm(P[:, :2] - A_XY, axis=1) < SEARCH_R)
         & (P[:, 2] > tz + 0.020) & (P[:, 2] < tz + Z_CAP)
         & (np.linalg.norm(P - eef, axis=1) > 0.16))
    Q = P[m]
    if Q.shape[0] < 100:
        api.log(f"{tag} region too small ({Q.shape[0]})")
        return None
    zmax = float(np.percentile(Q[:, 2], 99.5))
    ring = Q[Q[:, 2] >= zmax - RING_BAND][:, :2]
    if ring.shape[0] < 40:
        api.log(f"{tag} ring too small ({ring.shape[0]})")
        return None
    c = 0.5 * (ring.min(0) + ring.max(0))          # bbox midpoint seed
    R = float('nan')
    for _ in range(3):                              # trim to the ring, refit
        rr = np.linalg.norm(ring - c, axis=1)
        keep = ring[rr < min(0.075, np.percentile(rr, 98) + 0.006)]
        if keep.shape[0] < 30:
            break
        c2, R = circle_fit(keep)
        if not np.all(np.isfinite(c2)) or R < 0.02 or R > 0.09:
            break
        c = c2
    rr = np.linalg.norm(ring - c, axis=1)
    rr = rr[rr < 0.085]
    r_mid = float(np.median(rr))
    r_out = float(np.percentile(rr, 90))
    api.log(f"{tag} ring pts={ring.shape[0]} zmax={zmax:.4f}(tz+{zmax-tz:.4f}) "
            f"centre={np.round(c,4).tolist()} R_fit={R:.4f} r_mid={r_mid:.4f} "
            f"r_out={r_out:.4f} anchor_off={np.round(c-A_XY,4).tolist()}")
    return np.asarray(c, float), r_mid, zmax


def ground_plate(api, P, tz):
    m = ((np.linalg.norm(P[:, :2] - B_XY, axis=1) < 0.13)
         & (P[:, 2] > tz + 0.008) & (P[:, 2] < tz + 0.060))
    Q = P[m]
    if Q.shape[0] < 100:
        return None
    xy = Q[:, :2]
    c = 0.5 * (xy.min(0) + xy.max(0))
    for _ in range(2):
        keep = xy[np.linalg.norm(xy - c, axis=1) < 0.085]
        if keep.shape[0] < 50:
            break
        c = 0.5 * (keep.min(0) + keep.max(0))
    api.log(f"plate pts={Q.shape[0]} centre={np.round(c,4).tolist()} "
            f"anchor_off={np.round(c-B_XY,4).tolist()}")
    return np.asarray(c, float)


# ---------------------------------------------------------------- motion
def precise(api, target, seconds=1.5, tol=0.004, iters=3):
    t = np.asarray(target, float)
    api.move(t, seconds=seconds)
    for _ in range(iters):
        err = t - api.eef()
        if float(np.linalg.norm(err)) < tol:
            break
        api.move(t + 1.5 * err, seconds=0.7)
    return float(np.linalg.norm(t - api.eef()))


def descend(api, xy, z):
    api.move([xy[0], xy[1], z - DESCENT_BIAS], seconds=1.2)
    if api.eef()[2] > z + 0.004:
        api.move([xy[0], xy[1], z - 2 * DESCENT_BIAS], seconds=0.6)
    return api.eef()


def lift(api, z_to, step=0.02):
    e = api.eef()
    z = float(e[2])
    while z < z_to - 0.004:
        z = min(z + step, z_to)
        api.move([e[0], e[1], z], seconds=0.5)
    return api.eef()


def jaw_axis(api):
    R = np.asarray(api.tool_rotation(), float)
    u = R[:, JAW_AXIS_COL][:2]
    n = float(np.linalg.norm(u))
    return (u / n) if n > 1e-6 else np.array([0.0, 1.0])


# ---------------------------------------------------------------- policy
def run(api):
    api.log(f"instruction: {api.instruction()!r}")
    api.grip(OPEN_W)
    P = cloud(api.capture("cam_high"))
    tz = table_height(P)
    api.log(f"table_z={tz:.4f}")

    plate = ground_plate(api, P, tz)
    if plate is None or float(np.linalg.norm(plate - B_XY)) > 0.10:
        api.log("plate fallback -> demo anchor")
        plate = B_XY.copy()

    g = ground_bowl(api, P, tz, "t0")
    if g is None or float(np.linalg.norm(g[0] - A_XY)) > 0.11:
        api.log("bowl fallback -> demo anchor")
        centre, r_fit, zmax = A_XY.copy(), 0.045, tz + 0.0764
    else:
        centre, r_fit, zmax = g

    u = jaw_axis(api)
    s = 1.0 if float(np.dot(A_XY - centre, u)) >= 0 else -1.0
    z_grasp = float(zmax) - RIM_TOP_TO_EEF
    api.log(f"jaw={u.round(3).tolist()} side={s:+.0f} r_fit={r_fit:.4f} z_grasp={z_grasp:.4f}")

    held, r_used = False, r_fit
    for k, dr in enumerate(R_SWEEP):
        r = float(np.clip(r_fit + dr, 0.020, 0.070))
        p = centre + s * r * u
        api.grip(OPEN_W)
        res = precise(api, [p[0], p[1], z_grasp + APPROACH_DZ], seconds=2.0)
        e = descend(api, p, z_grasp)
        api.grip(CLOSE_W)
        w0 = api.gripper()["width_m"]
        e2 = lift(api, z_grasp + PROBE_DZ)
        gs = api.gripper()
        api.log(f"try{k} r={r:.4f} p={p.round(4).tolist()} res={res:.4f} "
                f"landed={e.round(4).tolist()} dz={e[2]-z_grasp:+.4f} "
                f"w_close={w0:.4f} w_probe={gs['width_m']:.4f} eff={gs['effort']}")
        if gs["effort"] >= HELD_EFFORT:
            held, r_used = True, r
            break
        api.grip(OPEN_W)
        if k < len(R_SWEEP) - 1:
            api.move([centre[0] + 0.02, centre[1] + 0.24, z_grasp + 0.18], seconds=2.0)
            P2 = cloud(api.capture("cam_high"))
            g = ground_bowl(api, P2, tz, f"re{k}")
            if g is not None and float(np.linalg.norm(g[0] - A_XY)) <= 0.12:
                centre, r_fit, zmax = g
                z_grasp = float(zmax) - RIM_TOP_TO_EEF
    api.log(f"held={held} r_used={r_used:.4f}")

    e = api.eef()
    lift(api, z_grasp + LIFT_DZ)
    drop = plate + s * r_used * u
    precise(api, [drop[0], drop[1], z_grasp + LIFT_DZ], seconds=2.5)
    api.log(f"over plate eef={api.eef().round(4).tolist()} grip={api.gripper()}")
    api.move([drop[0], drop[1], RELEASE_Z], seconds=1.2)
    api.log(f"at release eef={api.eef().round(4).tolist()} grip={api.gripper()}")
    api.grip(OPEN_W)
    api.settle(0.4)
    e = api.eef()
    api.move([e[0], e[1], e[2] + 0.12], seconds=1.0)
    api.settle(0.5)
    api.log(f"end eef={api.eef().round(4).tolist()}")
    return f"held={held} r={r_used:.4f} drop={np.round(drop,4).tolist()}"
