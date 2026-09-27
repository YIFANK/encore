"""v5 -- close the top drawer by pushing it home with the open, 90 deg-yawed
gripper. Same mechanism as v4 (8/8-track on the debug probe seeds) with the
flaky in-episode fingertip probe replaced by the constant it always measured,
which buys budget for two extra verified retries.

Mechanism, derived from the pack + debug seeds 51-65:
- The pack's K=3 demos never close the gripper (gripper_cmd == -1.0 in every
  keyframe), and every demo ends with a +y sweep. So: push, do not grasp, and
  push in +y.
- The drawer's front panel spans z 1.055..1.124 with its rim top at ~1.123;
  the cabinet body front face sits at y 0.195..0.225, which is where a closed
  drawer front ends up.
- With rotation=None the jaws separate along +/-y, i.e. along the push
  direction, and the leading finger fouls the rim during the descent (v3
  stalled at z=1.131). Yawing the wrist 90 deg puts both fingers at the same y,
  so the descent in front of the panel is clean and both fingers push together.
- api.eef() sits only ~0.009 m above the fingertips (measured), so the eef must
  ride just *below* the rim, not the ~6 cm below it that the demos' raw eef z
  values suggested.
"""
import numpy as np

PROVENANCE = {
    "TIP_OFF": {
        "source": "debug seeds 51/53/55/57 (v4 run): commanded the yawed gripper down onto bare table at z=0.901 and it stalled at eef z=0.9102/0.9103 -> fingertips are 0.0092 below the eef origin",
        "allowed": True},
    "TIP_DEPTHS": {
        "source": "debug seeds 51..65 cam_high: the drawer front panel face spans ~0.068 m below its rim top (z 1.055..1.124), so tips 0.030/0.050/0.015 below the rim are all on the panel",
        "allowed": True},
    "Y_END": {
        "source": "debug seeds 51..65 cam_high: cabinet body front face at y 0.195..0.225",
        "allowed": True},
    "SLAB_LO/SLAB_HI": {
        "source": "debug seeds 51..65: the drawer rim and the cabinet top slab occupy z 1.121..1.127",
        "allowed": True},
    "TRAVEL_Z": {
        "source": "debug seed 51 height map: drawer rim 1.123 is the tallest obstacle on the approach; 1.20 clears it",
        "allowed": True},
    "DESC_STANDOFF": {
        "source": "pack ee_path: the demos stand 0.06..0.16 m in front of the face before their +y sweep",
        "allowed": True},
    "CLOSED_Y": {
        "source": "debug seeds: cabinet front face 0.195..0.225, so a closed drawer face re-perceives at y >= 0.16",
        "allowed": True},
    "R_YAW90": {
        "source": "generic controller mechanics: Rz(90 deg) applied to the straight-down tool frame api.tool_rotation() reports at reset",
        "allowed": True},
    "PERC_CROP": {
        "source": "v2 debug logs: robot-arm points intrude at x < -0.19 with z > 1.15; the cabinet lives at y in (-0.02, 0.40), x in (-0.19, 0.22)",
        "allowed": True},
    "FALLBACK": {
        "source": "debug seed 51 measurement of the drawer front rim (x centre 0.000, face y 0.068, rim z 1.124)",
        "allowed": True},
}

TIP_OFF = 0.0092
TIP_DEPTHS = (0.030, 0.050, 0.015)
SLAB_LO, SLAB_HI = 1.100, 1.145
TRAVEL_Z = 1.20
Y_END = 0.215
DESC_STANDOFF = 0.10
CLOSED_Y = 0.16
FALLBACK = (0.000, 0.068, 1.124)

R_YAW90 = np.array([[0.0, 1.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0]])


def cloud(api, step=3):
    f = api.capture("cam_high")
    H, W = np.asarray(f.depth).shape
    pts = []
    for v in range(0, H, step):
        for u in range(0, W, step):
            p = f.deproject(u, v)
            if p is not None:
                pts.append(p)
    return np.asarray(pts, dtype=float)


def perceive(api, tag=""):
    """(x_centre, y_face, z_rim) of the drawer's front rim, or None."""
    P = cloud(api)
    S = P[(P[:, 2] > SLAB_LO) & (P[:, 2] < SLAB_HI)
          & (P[:, 1] > -0.02) & (P[:, 1] < 0.40)
          & (P[:, 0] > -0.19) & (P[:, 0] < 0.22)]
    if len(S) < 60:
        api.log("%sslab pts %d -> perception fail" % (tag, len(S)))
        return None
    y_face = float(np.percentile(S[:, 1], 2.0))
    F = S[S[:, 1] < y_face + 0.06]
    z_rim = float(np.percentile(F[:, 2], 90.0))
    x_lo, x_hi = float(np.percentile(F[:, 0], 3.0)), float(np.percentile(F[:, 0], 97.0))
    x_centre = 0.5 * (x_lo + x_hi)
    api.log("%sslab n=%d y_face=%.3f z_rim=%.3f strip n=%d x[%.3f,%.3f] centre=%.3f"
            % (tag, len(S), y_face, z_rim, len(F), x_lo, x_hi, x_centre))
    return x_centre, y_face, z_rim


def run(api):
    api.log("instruction: %r" % (api.instruction(),))
    api.grip(0.08)  # the demos hold the jaws open throughout
    api.log("eef0 %s" % np.round(api.eef(), 4).tolist())

    g = perceive(api, "init ")
    x_centre, y_face, z_rim = FALLBACK if g is None else g

    for i, depth in enumerate(TIP_DEPTHS):
        z_eef = z_rim - depth + TIP_OFF
        y_from = y_face - DESC_STANDOFF
        api.log("att%d depth=%.3f -> x=%.3f y_from=%.3f z_eef=%.3f"
                % (i, depth, x_centre, y_from, z_eef))
        api.move([x_centre, y_from, TRAVEL_Z], rotation=R_YAW90, seconds=1.5)
        api.move([x_centre, y_from, z_eef], rotation=R_YAW90, seconds=1.5)
        api.log("att%d descended eef=%s" % (i, np.round(api.eef(), 4).tolist()))
        api.move([x_centre, Y_END, z_eef], rotation=R_YAW90, seconds=3.0)
        api.log("att%d pushed eef=%s grip=%s"
                % (i, np.round(api.eef(), 4).tolist(), api.gripper()))
        api.settle(0.2)
        api.move([x_centre, y_from, z_eef], rotation=R_YAW90, seconds=1.5)
        api.move([x_centre, y_from, TRAVEL_Z], rotation=R_YAW90, seconds=1.5)

        g2 = perceive(api, "after%d " % i)
        if g2 is None:
            api.log("after%d: no slab; stopping" % i)
            return
        if g2[1] >= CLOSED_Y:
            api.log("after%d: face y=%.3f -> CLOSED" % (i, g2[1]))
            return
        api.log("after%d: face y=%.3f still open" % (i, g2[1]))
        x_centre, y_face, z_rim = g2
    api.log("attempts exhausted")
