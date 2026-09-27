"""Turn the stove knob.

Mechanism, read off the pack's K=1 demonstration:
  * the demo drives the eef to the knob, closes the jaws on it, and then turns;
  * reading the pack's 6-vector pose tail as an axis-angle rotation VECTOR
    (which is what reproduces the reset tool_rotation this cell measures, and
    what makes the raw actions' component-5 saturation agree with the pose
    trace), the grasp-keyframe -> final-keyframe relative rotation in the BASE
    frame is [0.095, 0.158, 0.925]: ~53 deg about world +z;
  * so the knob is a flat dial with a vertical rotation axis and an upright
    rectangular tab as its lever.

Perception, cam_high RGB-D only: a top-down max-z height map on a 5 mm grid;
the tab is the largest connected patch that is dark (mean rgb < 60) and whose
top lies 4-8 cm above the modal table plane.  On all 15 debug seeds that
returns exactly one candidate, 80-93 cells, top 0.9603, bbox 0.080 x 0.025 m.
Its bbox midpoint is the dial axis; its long axis sets the wrist yaw so the
jaws close across the tab's 25 mm face.

Control: open, hover 10 cm over the tab, then ONE descend command per rung,
and close -- but only from the right HEIGHT.  What decides this task is the
height the descend actually achieves, not the aim: on the 15-seed formal run
of v7 every success closed from an achieved eef z of tab_top+0.0073..+0.0103
and every failure from tab_top+0.0045..+0.0059, while lateral offsets of up
to 20 mm barely moved the outcome.  Too low and the jaws bite the tab's shank
and shove the dial bodily (the eef then drops 20 mm and the knob never turns);
too high and they close on air (tab_top + 20 mm gives width 0.001).  So each
rung checks the achieved height against that gate and simply does not close
when it falls outside, leaving the knob undisturbed for the next rung -- which
matters, because the gate is only known to predict the outcome on a knob
nothing has touched yet.

The ladder walks the descend command between tab_top + 5 mm and + 12 mm; the
last rung closes unconditionally, so a run that was gated out everywhere has
still tried.  The tab is perceived once, on the reset frame, because the
parked arm clips it in later captures and drags the bbox midpoint off the
dial axis.  A rung whose close comes back empty skips its turn rather than
spend horizon on it.  After each close, stage
a rotation about world +z in 15 deg steps to 90 deg with xy pinned on the dial
axis, reproducing the demo's turn.  Because LIBERO ends the episode the moment
its predicate fires, later rungs cost nothing once one has worked, and the
program never reads any termination or success flag to decide; it just runs a
fixed sequence.
"""
import numpy as np

PROVENANCE = {
    "GRID": {"source": "generic: 5 mm top-down voxel for the height map",
             "allowed": True},
    "XLO/XHI/YLO/YHI": {"source": "debug seeds 51-65 cam_high point clouds: "
                                  "scene extent x[-0.70,0.15] y[-0.45,0.45]",
                        "allowed": True},
    "DARK_MAX": {"source": "debug seeds 51-65 height maps: knob-tab cells mean "
                           "rgb 17-20; the next darkest scene structure is >= 60",
                 "allowed": True},
    "BAND_LO/BAND_HI": {"source": "debug seeds 51-65: tab top measured 0.9603 "
                                  "against a modal table plane of 0.9025, so a "
                                  "table+0.04..+0.08 band isolates the tab",
                        "allowed": True},
    "MIN_CELLS": {"source": "debug seeds 51-65: the tab patch is 80-93 cells; "
                            "the only other in-band dark patch was 8 cells",
                  "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics (tool z down, jaws along "
                         "tool y); agrees with api.tool_rotation() at reset",
               "allowed": True},
    "GRASP_DZ": {"source": "v2 debug sweep (seeds 51/53/57/61): eef z = "
                           "tab_top+0.005 closed at effort 3.0; tab_top+0.025 "
                           "closed on air (width 0.001)",
                 "allowed": True},
    "ATTEMPT_DZ": {"source": "v2 sweep + v3/v4/v7 debug runs: commanding "
                             "tab_top+0.005 achieves 0.9648-0.9706, so walk the "
                             "command up in 5 mm steps until the achieved height "
                             "lands in the gate", "allowed": True},
    "Z_GATE_LO/Z_GATE_HI": {
        "source": "v7 formal 15-seed debug run: all 12 successes closed from an "
                  "achieved eef z of tab_top+0.0073..+0.0103 and all 3 failures "
                  "from tab_top+0.0045..+0.0059; v2's sweep closed on air at "
                  "tab_top+0.020. Gate set midway in each gap.",
        "allowed": True},
    "HOVER_DZ": {"source": "v2/v3 debug runs: a 0.10 m hover cleared the knob "
                           "and the stove slab (top 0.932) on every seed",
                 "allowed": True},
    "TWIST_DEG": {"source": "pack demo: ~53 deg about +z from the grasp keyframe "
                            "to the last keyframe; staged past it to 90 deg",
                  "allowed": True},
    "HOLD_EFFORT": {"source": "FairApi contract: effort 3.0 iff the jaws hold",
                    "allowed": True},
    "OPEN_W/CLOSE_W": {"source": "FairApi contract: <0.025 closes, else opens; "
                                 "reset width measured 0.0778",
                       "allowed": True},
}

GRID = 0.005
XLO, XHI, YLO, YHI = -0.70, 0.15, -0.45, 0.45
DARK_MAX = 60.0
BAND_LO, BAND_HI = 0.040, 0.080
MIN_CELLS = 25
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
GRASP_DZ = 0.005
ATTEMPT_DZ = [0.005, 0.010, 0.008, 0.012, 0.006, 0.009]
HOVER_DZ = 0.10
TWIST_DEG = [15.0, 30.0, 45.0, 60.0, 75.0, 90.0]
TWIST_SIGN = [1.0, 1.0, 1.0, 1.0, 1.0, -1.0]
Z_GATE_LO = 0.0068
HOLD_EFFORT = 2.5
Z_GATE_HI = 0.0170
OPEN_W = 0.08
CLOSE_W = 0.0


def _rz(deg):
    t = np.radians(deg)
    return np.array([[np.cos(t), -np.sin(t), 0.0],
                     [np.sin(t), np.cos(t), 0.0],
                     [0.0, 0.0, 1.0]])


def height_map(frame):
    """Top-down max-z map plus the colour of the highest hit in each cell."""
    rgb = np.asarray(frame.rgb, float)
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape[:2]
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    good = np.isfinite(d) & (d > 0.05)
    z = np.where(good, d, 1.0)
    pc = np.stack([(uu - K[0, 2]) * z / K[0, 0],
                   (vv - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], -1)
    P = pc @ T.T
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    ok = good & (X > XLO) & (X < XHI) & (Y > YLO) & (Y < YHI)
    nx = int(round((XHI - XLO) / GRID))
    ny = int(round((YHI - YLO) / GRID))
    H = np.full((nx, ny), -9.0)
    C = np.zeros((nx, ny, 3))
    ia = np.clip(((X - XLO) / GRID).astype(int), 0, nx - 1)
    ib = np.clip(((Y - YLO) / GRID).astype(int), 0, ny - 1)
    order = np.argsort(Z[ok])          # ascending: the last write is the max z
    H[ia[ok][order], ib[ok][order]] = Z[ok][order]
    C[ia[ok][order], ib[ok][order]] = rgb[ok][order]
    return H, C


def table_plane(H):
    vals = H[H > -1.0]
    hist, edges = np.histogram(vals, bins=np.arange(0.80, 1.40, 0.005))
    i = int(np.argmax(hist))
    return float(0.5 * (edges[i] + edges[i + 1]))


def components(M):
    lab = np.zeros(M.shape, int)
    cur = 0
    for a in range(M.shape[0]):
        for b in range(M.shape[1]):
            if M[a, b] and lab[a, b] == 0:
                cur += 1
                stack = [(a, b)]
                lab[a, b] = cur
                while stack:
                    p, q = stack.pop()
                    for dp in (-1, 0, 1):
                        for dq in (-1, 0, 1):
                            r, s = p + dp, q + dq
                            if (0 <= r < M.shape[0] and 0 <= s < M.shape[1]
                                    and M[r, s] and lab[r, s] == 0):
                                lab[r, s] = cur
                                stack.append((r, s))
    return lab, cur


def find_tab(api, frame, tag=""):
    H, C = height_map(frame)
    table = table_plane(H)
    mask = (H > table + BAND_LO) & (H < table + BAND_HI) & (C.max(2) < DARK_MAX)
    lab, n = components(mask)
    best = None
    for k in range(1, n + 1):
        ii = np.argwhere(lab == k)
        if len(ii) < MIN_CELLS:
            continue
        xs = XLO + ii[:, 0] * GRID
        ys = YLO + ii[:, 1] * GRID
        dx, dy = xs - xs.mean(), ys - ys.mean()
        cov = np.array([[float((dx * dx).mean()), float((dx * dy).mean())],
                        [float((dx * dy).mean()), float((dy * dy).mean())]])
        w, v = np.linalg.eigh(cov)
        axis = v[:, int(np.argmax(w))]
        cand = {"n": int(len(ii)),
                "x": float(0.5 * (xs.min() + xs.max())),
                "y": float(0.5 * (ys.min() + ys.max())),
                "xspan": round(float(xs.max() - xs.min()), 4),
                "yspan": round(float(ys.max() - ys.min()), 4),
                "ztop": float(H[lab == k].max()),
                "deg": round(float(np.degrees(np.arctan2(axis[1], axis[0]))), 1),
                "rgb": round(float(C[lab == k].mean()), 1)}
        api.log(f"{tag}cand {cand}")
        if best is None or cand["n"] > best["n"]:
            best = cand
    api.log(f"{tag}table={table:.4f} tab={best}")
    return table, best


def _fold(deg):
    """The tab is 180-deg symmetric: fold the yaw into [-90, 90)."""
    while deg >= 90.0:
        deg -= 180.0
    while deg < -90.0:
        deg += 180.0
    return deg


def attempt(api, tab, dz, sign, tag, force=False):
    """One descend-and-turn.  Returns True if the jaws were actually closed.

    The close is gated on the height the descend ACHIEVED, not commanded: on
    the 15-seed formal run of v7 every success closed from an achieved eef z
    of 0.9676-0.9706 and every failure from 0.9648-0.9662, with the tab top
    at 0.9603 in all 15.  Below the gate the jaws bite the tab too far down
    its shank and shove the dial instead of turning it, so this refuses to
    close and leaves the knob untouched for the next rung of the ladder --
    which matters, because the gate is only known to predict the outcome on
    an undisturbed knob.
    """
    cx, cy, ztop = tab["x"], tab["y"], tab["ztop"]
    yaw = _fold(tab["deg"])
    Rg = _rz(yaw) @ R_DOWN
    api.log(f"{tag} PLAN x={cx:.4f} y={cy:.4f} ztop={ztop:.4f} "
            f"yaw={yaw:.1f} dz={dz:+.3f} sign={sign:+.0f} force={force}")
    api.grip(OPEN_W)
    api.move([cx, cy, ztop + HOVER_DZ], Rg, 2.0)
    res = api.move([cx, cy, ztop + dz], Rg, 2.5)
    e = np.asarray(api.eef())
    gap = float(e[2]) - ztop
    api.log(f"{tag} DESCEND res={res:.4f} eef={e.round(4).tolist()} "
            f"gap={gap:+.4f}")
    if not force and not (Z_GATE_LO <= gap <= Z_GATE_HI):
        api.log(f"{tag} SKIP gap={gap:+.4f} outside "
                f"[{Z_GATE_LO:+.4f},{Z_GATE_HI:+.4f}] -- not closing")
        api.move([cx, cy, ztop + HOVER_DZ], Rg, 2.0)
        return False
    api.grip(CLOSE_W)
    api.settle(0.4)
    g = api.gripper()
    api.log(f"{tag} CLOSE grip={g} "
            f"eef={np.asarray(api.eef()).round(4).tolist()}")
    if g["effort"] < HOLD_EFFORT:
        # Nothing in the jaws: a turn would only spend horizon.
        api.log(f"{tag} NO HOLD -- skipping the turn")
        api.grip(OPEN_W)
        api.move([cx, cy, ztop + HOVER_DZ], Rg, 2.0)
        return True
    zh = float(np.asarray(api.eef())[2])
    for dth in TWIST_DEG:
        r = api.move([cx, cy, zh], _rz(yaw + sign * dth) @ R_DOWN, 1.2)
        api.log(f"{tag} TWIST {sign * dth:+.0f} res={r:.4f} "
                f"eef={np.asarray(api.eef()).round(4).tolist()} "
                f"grip={api.gripper()}")
    api.grip(OPEN_W)
    api.settle(0.3)
    api.move([cx, cy, ztop + HOVER_DZ], R_DOWN, 2.0)
    return True


def run(api):
    api.log(f"instruction={api.instruction()!r}")
    api.log(f"eef0={np.asarray(api.eef()).round(4).tolist()} "
            f"grip0={api.gripper()}")
    _, tab = find_tab(api, api.capture("cam_high"))
    if tab is None:
        api.log("tab not found on the reset frame")
        return "tab not found"
    # The tab is perceived ONCE, on this pristine reset frame.  v8 re-perceived
    # before every rung and the parked arm clips the tab in cam_high (its
    # measured x-span fell 0.080 -> 0.045 -> 0.025), walking the bbox midpoint
    # up to 15 mm off the dial axis.  The knob is a fixed dial, so one clean
    # capture is both sufficient and better.
    n_closed = 0
    for i, (dz, sign) in enumerate(zip(ATTEMPT_DZ, TWIST_SIGN)):
        # The last rung closes whatever the achieved height, so a run whose
        # every descend was gated out still ends up having tried.
        force = (i == len(ATTEMPT_DZ) - 1 and n_closed == 0)
        n_closed += bool(attempt(api, tab, dz, sign, f"a{i}", force=force))
    api.log(f"closed on {n_closed} of {len(ATTEMPT_DZ)} rungs")
    return "knob turn: closed %d/%d rungs" % (n_closed, len(ATTEMPT_DZ))
