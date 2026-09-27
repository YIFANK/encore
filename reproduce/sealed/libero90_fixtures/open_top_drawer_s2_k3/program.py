"""l90abl / open_top_drawer_s2_k3 -- v1.

Demo-faithful replication of the K=3 pack.

Pack reading (all three demos agree):
  * the gripper is commanded OPEN (gripper_cmd = -1) for every logged step of
    every demo; there is no close anywhere in the pack.  So the handle is
    engaged with open fingers, not pinched.
  * the eef starts at ~(-0.193, -0.006, 1.167), arcs to ~(-0.036, -0.123,
    1.170), then descends to z ~= 1.107 while advancing to y ~= -0.173, and
    finally travels to y ~= +0.02 at constant z.  The +y travel is ~0.19 m and
    the episodes end there: +y is the drawer's opening direction.
  * during that final travel the raw actions are a near-constant
    (dx, dy, dz) ~= (-0.30, +0.90, -0.57) while x and z do not move at all.
    A saturated command that produces no motion is a contact constraint: the
    demonstrator presses the open fingers DOWN (and slightly -x) into the
    handle for the whole pull.  Reproducing that press is the point of the
    PULL_BIAS vector below.

api.move drives action = clip((target - eef) / 0.05, -1, 1) (generic
controller mechanics, stated in the harness docstring).  So to reproduce the
demo's action ratio the pull target is placed 0.017 m in -x and 0.032 m in -z
of the engage point -- those errors alone give -0.34 and -0.64 -- and far
enough in +y that the y component stays saturated for the whole travel.  The
move can never converge (|err| >= 0.032 > POS_TOL), so it burns its whole step
budget pressing and pulling, which is exactly the demo's behaviour.
"""

import numpy as np

PROVENANCE = {
    "W_ARC": {
        "source": "pack.json ee_path6 t=020 of demos 0/1/2, componentwise mean "
                  "of (-0.0268,-0.1331,1.1684), (-0.0323,-0.1304,1.1569), "
                  "(-0.0505,-0.1061,1.1834)",
        "allowed": True},
    "W_LOW": {
        "source": "pack.json ee_path6 t=030 of demos 0/1/2, componentwise mean "
                  "of (0.0192,-0.1715,1.1088), (0.0075,-0.1546,1.1116), "
                  "(0.0122,-0.1540,1.1443)",
        "allowed": True},
    "W_ENGAGE": {
        "source": "pack.json ee_path6: x/z are the t=040 mean (0.0185, 1.1073); "
                  "y is the mean of each demo's deepest logged y "
                  "(-0.1715, -0.1793, -0.1676) = -0.1728",
        "allowed": True},
    "PULL_BIAS": {
        "source": "pack.json actions, pull phase (demo0 t45-60, demo1 t45-65, "
                  "demo2 t45-65): mean (dx,dy,dz) = (-0.30, +0.90, -0.57); "
                  "scaled by the harness gain 0.05 m/unit into a position "
                  "offset (-0.017, +0.28, -0.032); the +y term is enlarged "
                  "past the 0.19 m demo travel so it stays saturated",
        "allowed": True},
    "PULL_SECONDS": {
        "source": "pack.json: the demos' pull phase spans ~30 controller steps "
                  "(t040->t070); 0.8 s = 96 steps under the harness's "
                  "max(40, 120*seconds) cap, ~3x the demo, and the move cannot "
                  "converge so the surplus is spent holding the drag",
        "allowed": True},
    "OPEN_WIDTH_M": {
        "source": "pack.json gripper_cmd = -1.0 (open) at every keyframe of "
                  "every demo; 0.08 m is above the harness's 0.025 m "
                  "grip-intent threshold, i.e. 'open'",
        "allowed": True},
}

W_ARC = np.array([-0.0365, -0.1232, 1.1696])
W_LOW = np.array([0.0130, -0.1600, 1.1216])
W_ENGAGE = np.array([0.0185, -0.1728, 1.1073])
PULL_BIAS = np.array([-0.017, 0.280, -0.032])
PULL_SECONDS = 0.8
OPEN_WIDTH_M = 0.08


def _look(api, tag, xyz):
    """Log where a base-frame point lands in cam_high, and what depth says is
    actually there.  Pure diagnostics -- no control depends on it."""
    try:
        f = api.capture("cam_high")
        T = np.asarray(f.t_base_cam, float)
        K = np.asarray(f.intrinsics, float)
        p = np.linalg.inv(T) @ np.append(np.asarray(xyz, float), 1.0)
        if p[2] > 1e-6:
            u = int(round(K[0, 0] * p[0] / p[2] + K[0, 2]))
            v = int(round(K[1, 1] * p[1] / p[2] + K[1, 2]))
            back = f.deproject(u, v)
            api.log("%s: uv=(%d,%d) zcam=%.4f seen=%s" % (
                tag, u, v, float(p[2]),
                None if back is None else np.round(back, 4).tolist()))
    except Exception as e:  # diagnostics must never break the run
        api.log("%s: look failed %s" % (tag, e))


def run(api):
    api.log("instruction=%r" % api.instruction())
    api.log("eef0=%s grip0=%s" % (np.round(api.eef(), 4).tolist(), api.gripper()))
    _look(api, "engage_expected", W_ENGAGE)

    api.grip(OPEN_WIDTH_M)

    r = api.move(W_ARC, seconds=1.2)
    api.log("arc res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))

    r = api.move(W_LOW, seconds=0.8)
    api.log("low res=%.4f eef=%s g=%s" % (
        r, np.round(api.eef(), 4).tolist(), api.gripper()))

    r = api.move(W_ENGAGE, seconds=0.6)
    eng = api.eef()
    api.log("engage res=%.4f eef=%s g=%s" % (
        r, np.round(eng, 4).tolist(), api.gripper()))

    target = eng + PULL_BIAS
    r = api.move(target, seconds=PULL_SECONDS)
    end = api.eef()
    api.log("pull res=%.4f eef=%s g=%s travel_y=%.4f" % (
        r, np.round(end, 4).tolist(), api.gripper(), float(end[1] - eng[1])))
    _look(api, "engage_after", W_ENGAGE)
