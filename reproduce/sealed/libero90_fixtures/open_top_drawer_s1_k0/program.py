"""l90abl / open_top_drawer_s1_k0 -- "open the top drawer of the cabinet".

Zero-demo. Perception and every constant below were derived from my own
debug-seed (51-65) observations under tools/fair_run.py; see PROVENANCE.

Mechanism (derived on debug seeds 51/53):
  The cabinet stands on the -y side of the table with its drawer fronts facing
  +y, so "open" = pull +y.  Each drawer front carries a horizontal grab rail
  standing ~30 mm proud of the cabinet's top-slab edge, held by two end
  brackets.  The rail is ~11 mm tall and the slot behind it is ~12 mm wide --
  narrower than a finger, so a top-down straddle is impossible (measured: the
  gripper is blocked whenever a finger must pass behind the rail).  Pitching
  the wrist 90 deg so the fingers point along -y and the jaws open along z
  lets the open jaws swallow the rail from the front (open air both above and
  below it) and clamp it.  Then pull +y.
"""

import numpy as np

PROVENANCE = {
    "R_PITCH": {
        "source": "generic controller mechanics: tool-to-world matrix whose tool z "
                  "(finger axis) is base -y and whose jaw axis is base z",
        "allowed": True},
    "EEF_TIP_OFFSET": {
        "source": "debug seed 57 measurement: closed gripper pressed onto the cabinet top "
                  "slab stalled at eef z=1.1375 while the slab surface deprojects to "
                  "z=1.127 -> fingertips sit 0.0105 m from the eef along the tool axis",
        "allowed": True},
    "BAR_HALF_LEN": {
        "source": "debug-seed cam_high top-handle cluster: x spans about [-0.045,+0.043] "
                  "with the two mounting brackets near +-0.032, so the bracket-free "
                  "middle is 0.045 m in from the rail's +x end",
        "allowed": True},
    "BAR_MID_DROP": {
        "source": "debug-seed cam_high: the top rail's front face spans z 1.087-1.098, so "
                  "its mid-height is 0.006 m below the cluster's 98th-pct top z",
        "allowed": True},
    "TIP_INSET": {
        "source": "debug-seed cam_high: rail is ~13 mm deep in y; driving the fingertips "
                  "0.008 m past its front face puts the jaws around it",
        "allowed": True},
    "STANDOFF": {
        "source": "debug-seed observation: 0.11 m in front of the rail is clear of the "
                  "cabinet and of both table props, so the pitch can be staged there",
        "allowed": True},
    "PULL_DIST": {
        "source": "debug seeds 51/53: a 0.26 m commanded +y pull moved the rail 0.146 m, "
                  "the drawer's full travel before the arm stalls",
        "allowed": True},
    "GRAY_MIN / SAT_MAX": {
        "source": "debug-seed cam_high RGB: the metal rails and the cabinet top slab read "
                  "as bright achromatic (mean > 110, max-min < 40) against the dark "
                  "cabinet body",
        "allowed": True},
    "Z_LINK": {
        "source": "debug-seed cam_high: the three rails sit 0.071 m apart in z, so a 0.012 m "
                  "single-link gap separates them cleanly",
        "allowed": True},
    "HOLD_EFFORT": {
        "source": "api.gripper() effort reads 3.0 iff the jaws are holding (stated in the "
                  "FairApi surface); measured 3.0 with width 0.0173 on the clamped rail",
        "allowed": True},
}

R_PITCH = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
EEF_TIP_OFFSET = 0.0105
BAR_HALF_LEN = 0.045
BAR_MID_DROP = 0.006
TIP_INSET = 0.008
STANDOFF = 0.11
PULL_DIST = 0.26
GRAY_MIN, SAT_MAX = 110.0, 40.0
Z_LINK = 0.012


def cloud(api, cam="cam_high", step=2):
    f = api.capture(cam)
    H, W = f.depth.shape
    pts = np.full((H, W, 3), np.nan, dtype=np.float32)
    for v in range(0, H, step):
        for u in range(0, W, step):
            p = f.deproject(u, v)
            if p is not None:
                pts[v, u] = p
    return f, pts


def scene(api):
    """Find the top drawer rail and the cabinet top slab from cam_high."""
    f, pts = cloud(api)
    P = pts.reshape(-1, 3)
    C = f.rgb.astype(np.float32).reshape(-1, 3)
    m = np.isfinite(P[:, 0])
    P, C = P[m], C[m]
    g = C.mean(1)
    sat = C.max(1) - C.min(1)
    sel = (g > GRAY_MIN) & (sat < SAT_MAX) & (P[:, 1] < -0.05) \
        & (P[:, 2] > 0.94) & (P[:, 2] < 1.25) & (P[:, 0] > -0.6) & (P[:, 0] < 0.35)
    Q = P[sel]
    if len(Q) < 40:
        return None
    Qs = Q[np.argsort(Q[:, 2])]
    clusters, cur = [], [Qs[0]]
    for p in Qs[1:]:
        if p[2] - cur[-1][2] > Z_LINK:
            clusters.append(np.array(cur))
            cur = [p]
        else:
            cur.append(p)
    clusters.append(np.array(cur))
    bars = [c for c in clusters if len(c) >= 25
            and (c[:, 1].max() - c[:, 1].min()) < 0.07
            and (c[:, 0].max() - c[:, 0].min()) < 0.20]
    slabs = [c for c in clusters if len(c) >= 500
             and (c[:, 1].max() - c[:, 1].min()) > 0.10]
    if not bars or not slabs:
        return None
    bars.sort(key=lambda c: np.median(c[:, 2]))
    bar = bars[-1]                                   # highest rail = top drawer
    slab = max(slabs, key=lambda c: np.median(c[:, 2]))
    return {"z_top": float(np.percentile(bar[:, 2], 98)),
            "y_bf": float(np.percentile(bar[:, 1], 98)),
            "y_s": float(np.percentile(slab[:, 1], 98)),
            "x_hi": float(np.percentile(bar[:, 0], 98))}


def hold(api, tgt, rot, n=8, seconds=1.6, tol=0.005):
    """Re-issue one target until it converges or stops making progress.

    api.move is step-budgeted, so a long motion needs several identical
    commands; two successive commands that move nothing mean a real block.
    """
    tgt = np.asarray(tgt, float)
    stuck = 0
    for _ in range(n):
        prev = np.asarray(api.eef(), float)
        api.move(tgt.tolist(), rotation=rot, seconds=seconds)
        e = np.asarray(api.eef(), float)
        if np.linalg.norm(e - tgt) < tol:
            return e, False
        if np.linalg.norm(e - prev) < 0.0025:
            stuck += 1
            if stuck >= 2:
                return e, True
        else:
            stuck = 0
    return np.asarray(api.eef(), float), False


def clamp_rail(api, xc, y_eef, z_mid, n_adv=9):
    """Swallow the rail with the pitched jaws and close. Returns the gripper state."""
    api.grip(0.08)
    api.settle(0.2)
    hold(api, [xc, y_eef + STANDOFF, z_mid], R_PITCH, n=7)
    e, _ = hold(api, [xc, y_eef, z_mid], R_PITCH, n=n_adv)
    api.log("advance eef=%s tips_y=%.4f target_y=%.4f z=%.4f" % (
        np.round(e, 4).tolist(), e[1] - EEF_TIP_OFFSET, y_eef, z_mid))
    api.grip(0.0)
    api.settle(0.5)
    gs = api.gripper()
    api.log("close grip=%s" % gs)
    return e, gs


def run(api):
    s = scene(api)
    api.log("SCENE %s" % s)
    if s is None:
        api.log("no rail found")
        return
    xc = s["x_hi"] - BAR_HALF_LEN
    z_mid = s["z_top"] - BAR_MID_DROP
    y_eef = s["y_bf"] - TIP_INSET + EEF_TIP_OFFSET

    # pitch the wrist in place (fingers point -y, jaws open along z)
    e0 = api.eef()
    api.grip(0.08)
    api.settle(0.2)
    for _ in range(4):
        api.move([e0[0], e0[1], e0[2]], rotation=R_PITCH, seconds=1.2)
    api.log("pitched rot=%s" % np.round(api.tool_rotation(), 2).tolist())

    e, gs = clamp_rail(api, xc, y_eef, z_mid)
    if gs["effort"] < 3.0:
        # missed: the jaws closed on air. Re-aim 5 mm lower and try once more.
        api.log("retry: closed empty (width=%.4f)" % gs["width_m"])
        e, gs = clamp_rail(api, xc, y_eef, z_mid - 0.005, n_adv=7)
    if gs["effort"] < 3.0:
        api.log("no grip; pulling anyway")

    e, b = hold(api, [xc, e[1] + PULL_DIST, z_mid], R_PITCH, n=9, seconds=2.0)
    api.log("pull eef=%s blocked=%d grip=%s" % (np.round(e, 4).tolist(), b, api.gripper()))
    s2 = scene(api)
    if s2 is not None:
        api.log("VERIFY rail y %.4f -> %.4f (moved %.4f)" % (
            s["y_bf"], s2["y_bf"], s2["y_bf"] - s["y_bf"]))
