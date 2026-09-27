"""close the bottom drawer of the cabinet -- push it shut with a closed fist.

Scene (re-derived from cam_high depth on debug seeds only): the cabinet stands
at +y with a top slab ~0.23 m above the table; the bottom drawer is already
pulled out toward -y, its interior floor a flat plane 0.025 m above the table
and its front wall topping out 0.085 m above it.  Closing it is a straight +y
push with the closed gripper on the front face.

Three measured facts decide the run:
  * COLUMN.  The arm is mounted behind -x and loses +y reach as it extends in
    +x.  Pushing on the drawer's x-centre stalls at y=0.087 and leaves the
    drawer open (v5, 0/8); pushing 0.11 m nearer the base reaches y=0.115 and
    shuts it (v6, 2/4, both failures being height, not column).
  * HEIGHT.  The fingertips sit 0.008 m below the eef, so an eef 0.043 m above
    the table puts them on the lower half of the drawer's front face.
    Calibrating that in-episode by pressing the table is what broke v6 on seeds
    55/57 -- the press landed on a prop -- so the offset is a declared constant
    and the descent column is checked clear instead.
  * -y WALL.  The arm cannot go past y ~= -0.12 (v4: it stalls at -0.124 and
    six further moves make no progress), so every approach stays at y >= -0.10.

Only the cabinet's top slab is trusted for perception: it is the one surface in
the scene above 1.09 and it carries >12000 points on every debug seed.  The
drawer's own front face is NOT measured -- a band mask for it picked up a prop
at y=-0.10 on all eight probe seeds (v7, 0/8) -- the push simply starts from a
standoff that is in front of the drawer on every debug seed and pushes through.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "cam_high depth, debug seeds 51-65: modal deprojected "
                          "tabletop height 0.901 m", "allowed": True},
    "TIP_OFF": {"source": "debug seeds 51/53 (v6): pressing the closed gripper "
                          "onto bare table stalls at eef z 0.9089, so the "
                          "fingertips are 0.008 m below the eef", "allowed": True},
    "PUSH_TIP_H": {"source": "debug-seed height scan: the drawer front face "
                             "spans z 0.901-0.985, so fingertips 0.035 m above "
                             "the table strike its lower half", "allowed": True},
    "PUSH_X_SHIFT": {"source": "debug seeds 51/53/55/57: pushing at the drawer's "
                               "x-centre stalls at y=0.087 (v5, 0/8); shifting "
                               "the column 0.11 m toward the base reaches "
                               "y=0.115 and closes it (v6)", "allowed": True},
    "CAB_TOP_LO": {"source": "debug-seed height bands: only the cabinet's top "
                             "slab reaches z 1.09-1.13", "allowed": True},
    "CAB_TOP_HI": {"source": "same scan (slab median 1.127)", "allowed": True},
    "Y_APP": {"source": "debug-seed measurement: the drawer's front wall stands "
                        "at y ~0.05-0.07 on every debug seed, so -0.05 is a "
                        "standoff in front of it and inside the -y reach wall "
                        "measured at -0.124 (v4)", "allowed": True},
    "Y_APP_ALTS": {"source": "fallback standoffs used when the first one's "
                             "descent column is occupied by a prop; all lie "
                             "between the -y reach wall and the front face",
                   "allowed": True},
    "OVERPUSH": {"source": "chosen over-command past the closed position so the "
                           "OSC keeps pressing instead of converging and "
                           "releasing (debug-seed behaviour of api.move)",
                 "allowed": True},
    "CARRY_Z": {"source": "debug-seed measurement: the drawer rim tops out at "
                          "0.985, so transiting with the eef at 1.06 clears it",
                "allowed": True},
    "CLEAR_H": {"source": "debug-seed measurement: bare tabletop deprojects to "
                          "0.901 +- 0.005, so >0.02 m in a descent column is a "
                          "prop", "allowed": True},
    "PROP_Z_MAX": {"source": "debug-seed observation: the resting arm's own "
                             "surface deprojects above 1.06, so the column "
                             "check ignores points above that", "allowed": True},
    "DEFAULT_PX": {"source": "debug-seed measurement: the cabinet slab spans "
                             "x -0.13..0.13, centre ~0.02", "allowed": True},
    "DEFAULT_YC": {"source": "debug-seed measurement: the cabinet front face is "
                             "at y 0.186-0.203", "allowed": True},
    "RETRY_MARGIN": {"source": "debug seeds: a push that shuts the drawer stalls "
                               "near y=0.115; stalling more than 0.06 m short of "
                               "that means contact was lost", "allowed": True},
}

TABLE_Z = 0.901
TIP_OFF = 0.008
PUSH_TIP_H = 0.035
PUSH_X_SHIFT = 0.11
CAB_TOP_LO, CAB_TOP_HI = 1.09, 1.16
Y_APP = -0.05
Y_APP_ALTS = (-0.05, -0.02, -0.08, 0.01)
OVERPUSH = 0.12
CARRY_Z = 1.06
CLEAR_H = 0.02
PROP_Z_MAX = 1.06
DEFAULT_PX = 0.02
DEFAULT_YC = 0.19
RETRY_MARGIN = 0.06


def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    H, W = f.depth.shape
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d = np.asarray(f.depth, float)
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    good = np.isfinite(d) & (d > 0)
    zc = np.where(good, d, 1.0)
    pc = np.stack([(uu - cx) * zc / fx, (vv - cy) * zc / fy, zc,
                   np.ones_like(zc)], axis=-1)
    P = pc @ T.T
    return good, P[..., 0], P[..., 1], P[..., 2]


def column_height(X, Y, Z, ws, x, y, half=0.035):
    """Tallest prop standing in a descent column, in metres above the table."""
    m = ws & (np.abs(X - x) < half) & (np.abs(Y - y) < half) & (Z < PROP_Z_MAX)
    if int(m.sum()) < 20:
        return 0.0
    return float(np.percentile(Z[m], 99.0) - TABLE_Z)


def run(api):
    api.grip(0.0)
    good, X, Y, Z = cloud(api)
    ws = good & (X > -0.35) & (X < 0.30) & (Y > -0.35) & (Y < 0.45)

    px, yc = DEFAULT_PX, DEFAULT_YC
    top = ws & (Z > CAB_TOP_LO) & (Z < CAB_TOP_HI) & (Y > 0.05)
    if int(top.sum()) > 300:
        px = float(np.median(X[top]))
        yc = float(np.percentile(Y[top], 2.0))
    api.log("slab n=%d px=%.3f yc=%.3f" % (int(top.sum()), px, yc))

    x_push = float(np.clip(px, -0.10, 0.10)) - PUSH_X_SHIFT
    y_app = Y_APP
    for cand in Y_APP_ALTS:
        h = column_height(X, Y, Z, ws, x_push, cand)
        api.log("column x=%.3f y=%.3f tallest=%.3f" % (x_push, cand, h))
        y_app = cand
        if h < CLEAR_H:
            break

    z_push = TABLE_Z + TIP_OFF + PUSH_TIP_H
    y_goal = yc + OVERPUSH
    api.log("plan x=%.3f y_app=%.3f z=%.3f y_goal=%.3f" % (
        x_push, y_app, z_push, y_goal))

    api.move([x_push, y_app, CARRY_Z], seconds=1.5)
    api.log("above eef=%s" % np.round(api.eef(), 4).tolist())
    r = api.move([x_push, y_app, z_push], seconds=0.8)
    api.log("down res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    r = api.move([x_push, y_goal, z_push], seconds=2.0)
    api.log("push1 res=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    r = api.move([x_push, y_goal, z_push], seconds=0.8)
    e = api.eef()
    api.log("push2 res=%.4f eef=%s" % (r, np.round(e, 4).tolist()))

    # A push that shut the drawer stalls hard against it; one that lost contact
    # (fingers rode over the rim, or the descent was blocked) stalls far short.
    if float(e[1]) < y_goal - OVERPUSH - RETRY_MARGIN or abs(e[2] - z_push) > 0.02:
        api.log("retry: stalled short at y=%.3f z=%.3f" % (e[1], e[2]))
        api.move([x_push, y_app, CARRY_Z], seconds=0.8)
        api.move([x_push - 0.02, y_app, z_push], seconds=0.8)
        r = api.move([x_push - 0.02, y_goal, z_push], seconds=1.5)
        e = api.eef()
        api.log("push3 res=%.4f eef=%s" % (r, np.round(e, 4).tolist()))

    api.log("final eef=%s" % np.round(api.eef(), 4).tolist())
    return "push stalled at y=%.3f" % e[1]
