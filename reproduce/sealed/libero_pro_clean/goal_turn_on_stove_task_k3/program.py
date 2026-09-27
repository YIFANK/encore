"""c2clean goal_turn_on_stove_task_k3 -- v5: perceived press-and-close, re-perceived.

Pack mechanism: the demos drive the OPEN gripper down onto the small dark post at
the back-left corner of the stove slab and flip the gripper command to close; the
burner disc is already red in the next keyframe. Debug receipts (v1, v2) show the
benchmark bit fires during that descend-and-close -- successful episodes end at
58-118 sim steps, long before any wrist turn -- and that the wrist turn the demos
add afterwards never helps and twice threw the arm off the stove, so v5 drops it.

The knob is perceived each time (darkest tall cluster above the slab, selected by
the pack's own terminal-EEF prior) and pressed 7 mm +x / 6 mm +y of its top-face
centroid, 31 mm below the top. A 1000-step horizon buys about a dozen such
attempts, so a failed press is retried against a FRESHLY perceived knob over a
small ring of offsets; once the predicate fires LIBERO freezes the episode and
the remaining attempts are inert.
"""
import numpy as np

PROVENANCE = {
    "KNOB_PRIOR_XY": {
        "source": "pack.json demos[*].keyframes[-1].ee -- mean of the three terminal "
                  "EEF xy; used only to pick which perceived cluster is the knob",
        "allowed": True},
    "DARK_SUM_MAX": {
        "source": "debug-seed 51-65 cam_high RGB: the knob post reads (3,3,3), the "
                  "stove slab ~(72,72,72)", "allowed": True},
    "BAND_Z": {
        "source": "debug-seed 51-65 cam_high depth: the knob top deprojects to "
                  "z=0.9603 on every seed, the slab to z~0.932", "allowed": True},
    "CLUSTER_TOL_M": {
        "source": "debug-seed measurement: the knob's dark footprint is contiguous at "
                  "a 0.02 m XY link distance", "allowed": True},
    "PRIOR_RADIUS_M": {
        "source": "debug-seed 51-65: the perceived knob-top centroid is within 0.022 m "
                  "of the pack prior on every seed; 0.15 m is a loose gate",
        "allowed": True},
    "KNOB_TOP_Z_FALLBACK": {
        "source": "debug-seed 51-65 measurement: knob top z = 0.9603 on all 15",
        "allowed": True},
    "OFFSETS": {
        "source": "pack.json terminal EEF spread (x -0.383..-0.425, y 0.202..0.211) "
                  "plus the v1/v2 debug receipts: v1 pressed 6-9 mm +x of the "
                  "perceived knob top and scored 4/4, v2 pressed on it and lost "
                  "seeds 53 and 65", "allowed": True},
    "OFFSETS_PRESS_DZ": {
        "source": "pack.json: terminal EEF z 0.9292-0.9308 against the measured knob "
                  "top 0.9603 -> the tool frame is driven ~0.031 m below the top",
        "allowed": True},
    "REPERCEIVE_Z_TOL_M": {
        "source": "debug-seed 51-65: the knob top reads 0.9603 on every seed, so a "
                  "re-perceived cluster more than 0.012 m off in height is not it",
        "allowed": True},
    "LIFT_DZ": {"source": "debug-seed: 0.05 m clears the knob between attempts",
                "allowed": True},
    "GRIP_OPEN_M": {"source": "generic controller mechanics: api.grip(>=0.025) opens",
                    "allowed": True},
    "GRIP_CLOSE_M": {"source": "generic controller mechanics: api.grip(<0.025) closes",
                     "allowed": True},
}

KNOB_PRIOR_XY = np.array([-0.4093, 0.2059])
DARK_SUM_MAX = 60
BAND_Z = (0.938, 0.995)
CLUSTER_TOL_M = 0.02
PRIOR_RADIUS_M = 0.15
KNOB_TOP_Z_FALLBACK = 0.9603
LIFT_DZ = 0.05
REPERCEIVE_Z_TOL_M = 0.012
GRIP_OPEN_M = 0.08
GRIP_CLOSE_M = 0.0

# (dx, dy, press depth below the knob top), applied to a FRESHLY perceived knob
OFFSETS = [
    (0.007, 0.006, 0.031),
    (0.007, 0.006, 0.031),
    (0.017, 0.006, 0.031),
    (-0.003, 0.006, 0.031),
    (0.007, 0.016, 0.031),
    (0.007, -0.004, 0.031),
    (0.007, 0.006, 0.044),
    (0.027, 0.006, 0.031),
    (0.007, 0.006, 0.022),
    (0.017, 0.016, 0.031),
]


def _cloud(frame, step=2):
    d = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h:step, 0:w:step]
    z = d[::step, ::step]
    p = np.stack([(uu - K[0, 2]) * z / K[0, 0],
                  (vv - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)], -1)
    return (p @ T.T)[..., :3], np.asarray(frame.rgb)[::step, ::step].astype(int)


def _clusters(pts, tol):
    lab = -np.ones(len(pts), int)
    c = 0
    for i in range(len(pts)):
        if lab[i] >= 0:
            continue
        stack = [i]
        lab[i] = c
        while stack:
            j = stack.pop()
            nb = np.nonzero((lab < 0) &
                            (np.linalg.norm(pts[:, :2] - pts[j, :2], axis=1) < tol))[0]
            lab[nb] = c
            stack.extend(nb.tolist())
        c += 1
    return lab, c


def find_knob(api):
    f = api.capture("cam_high")
    xyz, rgb = _cloud(f)
    Z = xyz[..., 2]
    m = ((rgb.sum(-1) < DARK_SUM_MAX) & (Z > BAND_Z[0]) & (Z < BAND_Z[1]) &
         (np.abs(xyz[..., 0] - KNOB_PRIOR_XY[0]) < 0.35) &
         (np.abs(xyz[..., 1] - KNOB_PRIOR_XY[1]) < 0.35))
    pts = xyz[m]
    if len(pts) < 10:
        return None
    lab, c = _clusters(pts, CLUSTER_TOL_M)
    best, bestn = None, 0
    for k in range(c):
        q = pts[lab == k]
        if len(q) < 10:
            continue
        zt = float(q[:, 2].max())
        top = q[q[:, 2] > zt - 0.006]
        cx, cy = float(top[:, 0].mean()), float(top[:, 1].mean())
        dist = float(np.hypot(cx - KNOB_PRIOR_XY[0], cy - KNOB_PRIOR_XY[1]))
        api.log("  cluster n=%d top=(%.4f,%.4f,%.4f) dist=%.4f" % (len(q), cx, cy, zt, dist))
        if dist < PRIOR_RADIUS_M and len(q) > bestn:
            best, bestn = (cx, cy, zt), len(q)
    return best


def run(api):
    api.log("instruction=%r" % api.instruction())
    R_DOWN = np.asarray(api.tool_rotation(), float)
    knob = find_knob(api)
    if knob is None:
        knob = (float(KNOB_PRIOR_XY[0]), float(KNOB_PRIOR_XY[1]), KNOB_TOP_Z_FALLBACK)
        api.log("knob NOT found -- pack prior")
    kx, ky, ztop = knob
    api.log("knob=(%.4f, %.4f, %.4f)" % (kx, ky, ztop))

    for i, (dx, dy, dz) in enumerate(OFFSETS):
        if i:
            # A press that did not fire the predicate can still have shoved the
            # knob, so re-perceive before every retry instead of re-using a
            # stale target. The lifted gripper sits above BAND_Z and cannot be
            # mistaken for the knob.
            again = find_knob(api)
            if again is not None and abs(again[2] - ztop) < REPERCEIVE_Z_TOL_M:
                kx, ky, ztop = again
                api.log("re-perceived knob=(%.4f, %.4f, %.4f)" % (kx, ky, ztop))
        tx, ty = kx + dx, ky + dy
        api.grip(GRIP_OPEN_M)
        api.move(np.array([tx, ty, ztop + LIFT_DZ]), rotation=R_DOWN, seconds=1.0)
        r = api.move(np.array([tx, ty, ztop - dz]), rotation=R_DOWN, seconds=1.0)
        api.grip(GRIP_CLOSE_M)
        g = api.gripper()
        api.log("a%d aim=(%.3f,%.3f,%.3f) res=%.4f gap=%.4f eef=%s"
                % (i, tx, ty, ztop - dz, r, g["width_m"],
                   np.round(api.eef(), 4).tolist()))
