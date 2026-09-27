"""l90abl open_bottom_drawer_k3 — v4 (drop-height sweep, diagnostic).

Geometry measured so far (debug seeds 51/53, agentview depth + contact probes):
  cabinet front face  y = -0.230, spanning x in [-0.05, 0.11], z in [0.90, 1.13]
  three handle bars, axis along base x, front edge y = -0.200, at z = 1.10,
  1.02 and 0.95; the bottom drawer is the z = 0.95 one.
  Pressing -y with the demo wrist attitude stops at eef y = -0.2144 for eef
  z >= 0.975 (the tool reaches the face over the bar) but at y = -0.1831 for
  eef z = 0.955 (the tool hits the bar's front).

v3 pressed to the face at z = 0.9886 and dropped only to 0.9700 before pulling:
the tool was still above the bar, so the pull slid off it.  v4 keeps the -y
command pressed and drops further, sweeping three depths and re-perceiving the
fixture after each pull, so the debug seeds say which depth puts the tool inside
the handle opening.
"""

import numpy as np

PROVENANCE = {
    "DEMO_ENGAGE_ROT6": {
        "source": "pack.json demos[*] pre-pull keyframe ee[3:6]", "allowed": True},
    "ENGAGE_X": {"source": "mean of pack.json demos[*] pre-pull keyframe ee[0]", "allowed": True},
    "FACE_Z": {"source": "pack.json demo0 ee_path6 t=80 z=0.9886 (the pose it reaches "
                         "before the descent into the pull)", "allowed": True},
    "DROP_Z": {"source": "pack.json demos[*] pre-pull keyframe ee[2] = 0.9703/0.9601/0.9792, "
                         "swept downward around that band on debug seeds", "allowed": True},
    "PRESS_Y": {"source": "debug-seed contact probe (v2): -y is blocked at eef y=-0.2144 "
                          "at z=0.9886; -0.26 commands past that block", "allowed": True},
    "PULL_END_Y": {"source": "mean of pack.json demos[*] final keyframe ee[1] = -0.0271",
                   "allowed": True},
    "OPEN_BAND": {"source": "debug-seed agentview deprojection: with the drawer shut the "
                            "band y in [-0.22,-0.12], z in [0.925,1.06] over the cabinet's "
                            "x range holds only the three handle bars", "allowed": True},
    "ROT_DECODE": {"source": "v1 debug-seed measurement: rvec decoding of pack ee[3:6] "
                             "matched api.tool_rotation() to 1.15 deg vs 5.64 for rpy",
                   "allowed": True},
}

DEMO_ENGAGE_ROT6 = [[1.4781, 1.6816, -0.5879],
                    [1.5061, 1.6424, -0.6047],
                    [1.5296, 1.4241, -0.6380]]
ENGAGE_X = 0.0157
FACE_Z = 0.9886
DROP_Z = [0.9600, 0.9500, 0.9400]
PRESS_Y = -0.26
PULL_END_Y = -0.0271


def _from_rvec(v):
    v = np.asarray(v, float)
    th = float(np.linalg.norm(v))
    if th < 1e-9:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def _mean_rot(mats):
    M = sum(mats)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def _open_count(api, tag):
    """Points of the fixture that stand in the band an open drawer fills."""
    f = api.capture("cam_high")
    n = 0
    pts = []
    for v in np.linspace(220, 440, 22).astype(int):
        for u in np.linspace(60, 230, 22).astype(int):
            p = f.deproject(int(u), int(v))
            if p is None:
                continue
            if (-0.16 <= p[0] <= 0.16 and -0.225 <= p[1] <= -0.115
                    and 0.925 <= p[2] <= 1.06):
                n += 1
                pts.append((int(u), int(v), round(float(p[1]), 3), round(float(p[2]), 3)))
    api.log("OPEN %s n=%d pts=%s" % (tag, n, pts[:24]))
    return n


def run(api):
    R = _mean_rot([_from_rvec(v) for v in DEMO_ENGAGE_ROT6])
    api.log("home eef=%s" % np.round(api.eef(), 4).tolist())
    n0 = _open_count(api, "pre")

    def go(tag, xyz, seconds=1.0):
        r = api.move(xyz, rotation=R, seconds=seconds)
        p = api.eef()
        g = api.gripper()
        api.log("%s tgt=%s eef=%s res=%.4f w=%.5f" %
                (tag, [round(v, 4) for v in xyz], np.round(p, 4).tolist(), r, g["width_m"]))
        return p

    api.grip(0.08)
    go("rot", [ENGAGE_X, -0.060, FACE_Z], 2.0)

    for i, dz in enumerate(DROP_Z):
        go("a%d.press" % i, [ENGAGE_X, PRESS_Y, FACE_Z], 1.0)
        go("a%d.drop" % i, [ENGAGE_X, PRESS_Y, dz], 0.8)
        go("a%d.pull" % i, [ENGAGE_X, PULL_END_Y, dz], 1.5)
        go("a%d.lift" % i, [ENGAGE_X, PULL_END_Y, 1.08], 0.6)
        n = _open_count(api, "a%d" % i)
        api.log("a%d verdict n0=%d n=%d" % (i, n0, n))
        if n - n0 >= 8:
            return "opened at drop z=%.4f" % dz

    return "v4 sweep exhausted"
