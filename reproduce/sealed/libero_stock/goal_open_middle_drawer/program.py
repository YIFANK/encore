"""c2 cell goal_open_middle_drawer_stock -- v11: repeat the blade hook until the
middle drawer stops coming out, with the blade centred by a cross-cluster median.

Mechanism (established v6-v9 on debug seeds 51-65):
  * The cabinet face plane is the modal y of the depth band between the table
    top and the cabinet lid; exactly three D-shaped pulls stand ~0.032 m proud
    of it (seed 51: z 0.940-0.956 / 1.012-1.025 / 1.082-1.098, tips y=-0.126,
    face y=-0.158). The middle cluster by height is the middle drawer's pull.
  * The pull is a thin bar (0.014 m tall, 0.095 m long) on end returns, with an
    open gap behind it. An OPEN gripper cannot use that gap -- its fingers are
    0.080 m apart in world x and land on the returns, stalling the tool against
    the bar's outer surface (v3 residual 0.022-0.032). A CLOSED gripper is one
    thin blade at the tool centre and passes between the returns.
  * Recipe: press the blade onto the drawer face above the bar (which measures
    the fingertip's tool-axis offset, since the face y is known), slide it down
    the face aiming well below the bar so contact -- not move_pose's 0.012 m
    tolerance -- stops it, then drag +y.
  * v9 ran that once on all eight odd debug seeds: the pull advanced 0.089-0.099
    m every time (film strip confirms an open drawer that stays open), but the
    benchmark bit stayed False, and the blade always un-wedged partway (finger
    gap 0.006 -> 0.001 during the drag). One bite is not the whole drawer.

v10 re-measured the pull after each drag and took another bite from wherever it
now was: 13/15 on the full debug band. Both failures (seeds 56 and 65) had the
same cause -- the middle cluster's x extent picked up a tail of foreign points
near x=-0.17, which dragged the blade's x to -0.05, off the end of a bar that
actually spans -0.004..0.082. The top and bottom clusters were clean in both
episodes and reported the SAME x extent as each other, because the three pulls
are vertically aligned. v11 therefore takes the element-wise MEDIAN of the three
clusters' x extents, which a single contaminated cluster cannot move, and it no
longer abandons the drawer after one unproductive pass -- it re-tries the bite
shifted along the bar until two passes in a row fail or the budget is gone.
"""

import numpy as np

PROVENANCE = {
    "AA_ENGAGE": {
        "source": "pack.json demos[0].ee_path6 t=90 axis-angle (1.7341,1.6522,-0.8649). "
                  "v4 measured hand-built 30-deg-below-horizontal wrists unreachable at "
                  "the handle (residual 0.042-0.053); this one reaches it below 0.010",
        "allowed": True,
    },
    "AA_LIFT": {"source": "pack.json demos[0].ee_path6 t=20 axis-angle", "allowed": True},
    "STAGE_XYZ": {"source": "pack.json demos[0].ee_path6 t=20 position -- free-space "
                            "staging on the demos' own way in", "allowed": True},
    "FACE_BAND": {
        "source": "debug-seed 51-65 cam_high depth: face y=-0.158, pull tips y=-0.126, "
                  "cabinet lid z=1.13, table top z=0.90",
        "allowed": True,
    },
    "PROTRUDE_MIN_M": {
        "source": "debug-seed v6 face profile: the pulls stand 0.032 m proud of the face, "
                  "so 0.012 m cleanly separates pull points from face points",
        "allowed": True,
    },
    "PROTRUDE_MAX_M": {
        "source": "debug-seed: upper bound that keeps table objects out of the handle "
                  "search while admitting a pull 0.032 m proud",
        "allowed": True,
    },
    "TIP_BAND_M": {
        "source": "debug-seed v6 profile: the bar's outer surface reads within 0.006 m of "
                  "its tip, so points beyond face+0.025 are bar surface and give an "
                  "uncontaminated x extent for centring the blade between the returns",
        "allowed": True,
    },
    "TIP_DELTA_M": {
        "source": "debug-seed 51 v5-v9: pressing the fingertip onto the face seats the eef "
                  "0.0164 m out from the face plane in y, i.e. 0.0209 m of tool-axis "
                  "fingertip offset",
        "allowed": True,
    },
    "CLEAR_Z_M": {
        "source": "debug-seed v6 profile: the face is clear from z=1.027 to the pull above "
                  "at z=1.082, so entering 0.030 m over the middle bar clears that bar and "
                  "passes under the one above; on later passes the entry is additionally "
                  "capped below the measured top edge of the drawer's own front panel",
        "allowed": True,
    },
    "FRONT_MARGIN_M": {
        "source": "debug-seed: once the drawer is out, its front panel is the only surface "
                  "to press on, so the entry height is kept this far below the panel's "
                  "measured top edge",
        "allowed": True,
    },
    "PRESS_PAST_M": {
        "source": "debug-seed: move_pose returns as soon as it is inside a 0.012 m position "
                  "tolerance, so the press target is set past the face plane to force the "
                  "fingertip to seat (v1 broke early at 0.0075 m and never seated)",
        "allowed": True,
    },
    "SLIDE_BELOW_M": {
        "source": "same tolerance argument for the downward slide: aiming 0.030 m below the "
                  "bar's bottom makes contact the stopping condition (v7 stopped exactly "
                  "one tolerance high, at the bar's top edge)",
        "allowed": True,
    },
    "SLIDE_PRESS_M": {
        "source": "debug-seed v5: a 0.050 m press past the face saturates the y action and "
                  "the slide stalls; 0.006 m keeps contact with the z error dominant",
        "allowed": True,
    },
    "PULL_LEN_M": {
        "source": "pack.json ee_path6 pull spans: demo0 0.170 m, demo1 0.173 m, demo2 "
                  "0.151 m of +y travel; rounded up to drag past the demos",
        "allowed": True,
    },
    "MAX_PASSES": {
        "source": "debug-seed budget: v10's three-pass episodes cost 430-456 sim steps "
                  "(~150 per pass) and v8 completed 668 steps without the episode ending, "
                  "so four passes fit",
        "allowed": True,
    },
    "XSPAN_MEDIAN": {
        "source": "debug-seed 51-65: the three pulls are vertically aligned, so their x "
                  "extents agree (seed 56: bottom [-0.019,0.067], top [-0.019,0.067], "
                  "middle contaminated to [-0.172,0.067]); the element-wise median across "
                  "the three is immune to one contaminated cluster",
        "allowed": True,
    },
    "RETRY_DX_M": {
        "source": "debug-seed v6 profile: the bar is 0.095 m long, so a bite shifted "
                  "0.018 m along it is still well inside the end returns",
        "allowed": True,
    },
    "FRONT_MIN_PTS": {
        "source": "debug-seed 65: a 247-point front-panel column gave a nonsense top edge "
                  "of 0.966 (below the bar itself), while good measurements carry 800+ "
                  "points",
        "allowed": True,
    },
    "PASS_GAIN_M": {
        "source": "debug-seed: v9's single pass advanced the pull 0.089-0.099 m, so a pass "
                  "that adds less than 0.020 m has stopped being productive",
        "allowed": True,
    },
    "OPEN_WIDTH_M": {"source": "FairApi contract: a width above 0.025 m is the open command",
                     "allowed": True},
    "CLOSE_WIDTH_M": {"source": "FairApi contract: a width below 0.025 m is the close command",
                      "allowed": True},
}

OPEN_WIDTH_M = 0.08
CLOSE_WIDTH_M = 0.0
AA_ENGAGE = (1.7341, 1.6522, -0.8649)
AA_LIFT = (2.8487, 0.5969, -0.3612)
STAGE_XYZ = (-0.0963, 0.0045, 1.1658)
FACE_BAND = dict(x=(-0.18, 0.42), y=(-0.48, -0.03), z=(0.935, 1.125))
PROTRUDE_MIN_M = 0.012
PROTRUDE_MAX_M = 0.070
TIP_BAND_M = 0.025
TIP_DELTA_M = 0.0209
CLEAR_Z_M = 0.030
FRONT_MARGIN_M = 0.012
PRESS_PAST_M = 0.030
SLIDE_BELOW_M = 0.030
SLIDE_PRESS_M = 0.006
PULL_LEN_M = 0.24
MAX_PASSES = 4
PASS_GAIN_M = 0.020
RETRY_DX_M = 0.018
FRONT_MIN_PTS = 400


def aa_to_mat(aa):
    a = np.asarray(aa, float)
    th = float(np.linalg.norm(a))
    if th < 1e-9:
        return np.eye(3)
    k = a / th
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * K @ K


R_ENGAGE = aa_to_mat(AA_ENGAGE)
APPROACH = R_ENGAGE[:, 2] / np.linalg.norm(R_ENGAGE[:, 2])
TIP_DZ = TIP_DELTA_M * abs(APPROACH[2])


def cloud(frame):
    d = np.asarray(frame.depth, float)
    h, w = d.shape[:2]
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    vv, uu = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > 0.01) & (d < 6.0)
    z = d[ok]
    x = (uu[ok] - K[0, 2]) * z / K[0, 0]
    y = (vv[ok] - K[1, 2]) * z / K[1, 1]
    P = np.stack([x, y, z, np.ones_like(z)], axis=1) @ T.T
    return P[:, :3]


def find_middle_bar(api, tag):
    P = cloud(api.capture("cam_high"))
    bx, by, bz = FACE_BAND["x"], FACE_BAND["y"], FACE_BAND["z"]
    Q = P[(P[:, 0] > bx[0]) & (P[:, 0] < bx[1]) & (P[:, 1] > by[0]) & (P[:, 1] < by[1])
          & (P[:, 2] > bz[0]) & (P[:, 2] < bz[1])]
    if Q.shape[0] < 500:
        api.log("%s perceive: only %d band points" % (tag, Q.shape[0]))
        return None
    edges = np.arange(by[0], by[1] + 1e-9, 0.005)
    hist, _ = np.histogram(Q[:, 1], bins=edges)
    j = int(np.argmax(hist))
    face_y = float(0.5 * (edges[j] + edges[j + 1]))
    H = Q[(Q[:, 1] > face_y + PROTRUDE_MIN_M) & (Q[:, 1] < face_y + PROTRUDE_MAX_M)]
    F = Q[np.abs(Q[:, 1] - face_y) < 0.010]
    if F.shape[0] > 50:
        xlo, xhi = np.percentile(F[:, 0], 2.0), np.percentile(F[:, 0], 98.0)
        H = H[(H[:, 0] > xlo - 0.02) & (H[:, 0] < xhi + 0.02)]
    if H.shape[0] < 40:
        api.log("%s perceive: face_y=%.4f, %d protruding points" % (tag, face_y, H.shape[0]))
        return None
    H = H[np.argsort(H[:, 2])]
    groups, cur = [], [H[0]]
    for p in H[1:]:
        if p[2] - cur[-1][2] > 0.012:
            groups.append(np.array(cur))
            cur = []
        cur.append(p)
    groups.append(np.array(cur))
    groups = [g for g in groups if g.shape[0] >= 25]
    for g in groups:
        api.log("%s cand n=%4d z=[%.3f,%.3f] x=[%.3f,%.3f] ytip=%.4f"
                % (tag, g.shape[0], g[:, 2].min(), g[:, 2].max(),
                   np.percentile(g[:, 0], 2.0), np.percentile(g[:, 0], 98.0), g[:, 1].max()))
    if len(groups) < 2:
        return None
    def xspan(q):
        S = q[q[:, 1] > face_y + TIP_BAND_M]
        if S.shape[0] < 20:
            S = q
        return (float(np.percentile(S[:, 0], 2.0)), float(np.percentile(S[:, 0], 98.0)))

    if len(groups) >= 3:
        big = sorted(groups, key=lambda q: -q.shape[0])[:3]
        byz = sorted(big, key=lambda q: float(np.median(q[:, 2])))
        g = byz[1]
        # the three pulls are vertically aligned, so the median x extent across
        # them survives one cluster whose tail has been contaminated
        spans = [xspan(q) for q in byz]
        xlo2 = float(np.median([a for a, _ in spans]))
        xhi2 = float(np.median([b for _, b in spans]))
        api.log("%s xspans %s -> [%.4f,%.4f]"
                % (tag, [(round(a, 3), round(b, 3)) for a, b in spans], xlo2, xhi2))
    else:
        g = sorted(groups, key=lambda q: float(np.median(q[:, 2])))[-1]
        xlo2, xhi2 = xspan(g)
    zlo = float(np.percentile(g[:, 2], 2.0))
    zhi = float(np.percentile(g[:, 2], 98.0))
    out = dict(face_y=face_y, bar_zlo=zlo, bar_zhi=zhi,
               bar_x=float(0.5 * (xlo2 + xhi2)), bar_xlo=xlo2, bar_xhi=xhi2,
               bar_ytip=float(np.percentile(g[:, 1], 95.0)), n=int(g.shape[0]))
    api.log("%s MIDDLE %s" % (tag, {k: round(v, 4) for k, v in out.items()}))
    return out


def remeasure(api, bar, tag):
    """Where is the middle pull now, and how high does its front panel reach?"""
    P = cloud(api.capture("cam_high"))
    win = ((P[:, 0] > bar["bar_xlo"] - 0.015) & (P[:, 0] < bar["bar_xhi"] + 0.015)
           & (P[:, 2] > bar["bar_zlo"] - 0.008) & (P[:, 2] < bar["bar_zhi"] + 0.008)
           & (P[:, 1] > bar["face_y"] - 0.06) & (P[:, 1] < bar["face_y"] + 0.45))
    if int(win.sum()) < 20:
        api.log("%s remeasure: only %d points in the pull window" % (tag, int(win.sum())))
        return None
    tip = float(np.percentile(P[win, 1], 99.0))
    adv = tip - bar["bar_ytip"]
    face2 = bar["face_y"] + max(0.0, adv)
    col = ((P[:, 0] > bar["bar_xlo"] - 0.015) & (P[:, 0] < bar["bar_xhi"] + 0.015)
           & (np.abs(P[:, 1] - face2) < 0.012) & (P[:, 2] > bar["bar_zlo"] - 0.10)
           & (P[:, 2] < bar["bar_zlo"] + 0.12))
    front_top = None
    if int(col.sum()) >= FRONT_MIN_PTS:
        cand = float(np.percentile(P[col, 2], 98.0))
        if cand > bar["bar_zhi"] + 0.005:
            front_top = cand
    api.log("%s remeasure: tip=%.4f adv=%.4f face2=%.4f front_top=%s (n=%d/%d)"
            % (tag, tip, adv, face2, "None" if front_top is None else round(front_top, 4),
               int(win.sum()), int(col.sum())))
    return dict(bar, bar_ytip=tip, face_y=face2, adv=adv, front_top=front_top)


def state(api, tag):
    e = api.eef()
    g = api.gripper()
    api.log("%s eef=(%.4f,%.4f,%.4f) w=%.4f eff=%.2f done=%s"
            % (tag, e[0], e[1], e[2], g["width_m"], g["effort"], api.done))
    return e


def hook_and_drag(api, bar, tag):
    tx = bar["bar_x"]
    fy = bar["face_y"]
    z_entry = bar["bar_zhi"] + CLEAR_Z_M + TIP_DZ
    top = bar.get("front_top")
    if top is not None:
        z_entry = min(z_entry, top - FRONT_MARGIN_M + TIP_DZ)
        z_entry = max(z_entry, bar["bar_zhi"] + 0.010 + TIP_DZ)
    z_deep = bar["bar_zlo"] - SLIDE_BELOW_M + TIP_DZ
    api.move((tx, bar["bar_ytip"] + 0.085, z_entry), rotation=R_ENGAGE, seconds=1.5)
    state(api, "%s.standoff" % tag)
    r = api.move((tx, fy - PRESS_PAST_M, z_entry), rotation=R_ENGAGE, seconds=1.2)
    e = state(api, "%s.press" % tag)
    api.log("%s press res=%.4f seated_y=%.4f face=%.4f entry_z=%.4f"
            % (tag, r, e[1], fy, z_entry))
    r = api.move((tx, e[1] - SLIDE_PRESS_M, z_deep), rotation=R_ENGAGE, seconds=1.5)
    e = state(api, "%s.slot" % tag)
    api.log("%s slot got_z=%.4f tip_z=%.4f (bar %.4f..%.4f) res=%.4f"
            % (tag, e[2], e[2] - TIP_DZ, bar["bar_zlo"], bar["bar_zhi"], r))
    r = api.move((tx, e[1] + PULL_LEN_M, e[2]), rotation=R_ENGAGE, seconds=1.8)
    e2 = state(api, "%s.drag" % tag)
    api.log("%s drag res=%.4f dy=%.4f" % (tag, r, e2[1] - e[1]))
    return e2


def run(api):
    api.log("instruction: %s tip_dz=%.4f" % (api.instruction(), TIP_DZ))
    api.grip(OPEN_WIDTH_M)
    bar0 = find_middle_bar(api, "P0")
    if bar0 is None:
        return "perception failed at t0"
    y0 = bar0["bar_ytip"]

    api.move(STAGE_XYZ, rotation=aa_to_mat(AA_LIFT), seconds=1.0)
    api.grip(CLOSE_WIDTH_M)
    api.log("blade closed w=%.4f" % api.gripper()["width_m"])

    bar = dict(bar0)
    total = 0.0
    stale = 0
    shifts = (0.0, RETRY_DX_M, -RETRY_DX_M, 0.0)
    for p in range(MAX_PASSES):
        if api.done:
            api.log("episode ended before pass %d" % p)
            break
        bar["bar_x"] = bar0["bar_x"] + shifts[min(stale, len(shifts) - 1)]
        e = hook_and_drag(api, bar, "p%d" % p)
        api.move((bar["bar_x"], e[1] + 0.12, e[2] + 0.10), rotation=R_ENGAGE, seconds=1.0)
        m = remeasure(api, bar0, "R%d" % p)
        if m is None:
            api.log("pass %d: lost the pull" % p)
            break
        gain = m["adv"] - total
        total = m["adv"]
        api.log("pass %d: total advance %.4f (this pass %+.4f, stale=%d)"
                % (p, total, gain, stale))
        if gain < PASS_GAIN_M:
            stale += 1
            if stale >= 3:
                api.log("three unproductive passes -- stopping")
                break
        else:
            stale = 0
            bar = dict(m)
    return "middle pull advanced %.3f m in %d passes" % (total, p + 1)
