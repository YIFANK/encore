"""l90abl open_microwave_k0 -- v16: v12's hook sweep, three fixed attempts.

Scene (debug 51-65, cam_high plus one wrist look-down; zero-demo cell):
table z = 0.9012; one body, footprint x[-0.17,+0.17] y[-0.35,-0.13], top
table+0.207.  Its robot-facing (+y) face is a plane (y = -0.1456 on seed 51,
-0.1315 on seed 54, so the body shifts ~14 mm between seeds) and the only
protrusion on it is a vertical D-handle bar at x = -0.0675, front face
y = -0.1047, spanning z = table+0.043..table+0.193 and 0.014 m thick across
x.  The leaf hinges on the body's -x edge.

What opens this door, and what does not.  v7-v11 PINCHED the bar and pulled;
all four died at 40-45 deg with the finger gap first prised open (v11:
0.0140 -> 0.0405) and then empty.  Pulling a door open is retraction along
the gripper's own axes, so a pinch holds on friction alone, and the friction
runs out.  What works is a HOOK: jaws held open and straddling the bar along
the door normal, so the rear finger bears on the bar's back face and the
contact is form closure.  v12 realises that hook and scored 14/15 on the
debug seeds, with the episode frames showing the open jaws wrapped round the
bar as the leaf swings past 90 deg.

Two ways of aiming that hook were tried and both fail on the arm: entering
the slot from above is blocked by the handle's top bracket (v15's descent
stalled 28 mm high, res 0.0319, and the arc then swept air), and entering it
from +x asks the arm to reach behind itself -- home eef is x=-0.2085 -- so
v13/v14 died 0.19 m short.  v12 gets in because it turns the wrist and
crosses the workspace in ONE move and the OSC transient carries the open
jaws down across the door face and onto the bar; the episodes terminate at
sim step 40.  Seed 54, the one where the body sits 14 mm nearer, is the seed
that transient misses.

v16 keeps that motion and simply gives it three tries at three heights
(the bar is 0.150 m tall, so +/-0.025 all stay well on it), returning the
wrist to the down pose between them at y=+0.06, clear of the body.  The
sequence is fixed and open-loop -- nothing is read back to decide whether to
continue -- so on the seeds where the first attempt lands, the episode has
already ended and the rest is inert.
"""
import numpy as np

PROVENANCE = {
    "TABLE": {"source": "generic: table plane = median depth of the workspace crop, "
                        "measured 0.9012 on debug seeds 51/53/57", "allowed": True},
    "PROUD_M": {"source": "debug 51: bar front y=-0.1047 vs panel y=-0.1456, so 0.015 "
                          "separates bar cells from panel cells", "allowed": True},
    "REAR_GAP": {"source": "debug 51: the slot between the bar and the panel spans 0.041 m "
                           "of y; 0.013 in front of the measured panel puts the rear finger "
                           "inside it clear of both surfaces", "allowed": True},
    "HALF_OPEN": {"source": "debug 51 proprio: the open gripper reports width_m 0.0778, so "
                            "each finger sits 0.0389 from the commanded frame", "allowed": True},
    "STANDOFF": {"source": "debug 51: nothing on the front face is proud of the panel except "
                           "the bar, so 0.12 to +x of the bar is clear approach room",
                 "allowed": True},
    "R_DOWN": {"source": "generic controller mechanics: measured home tool_rotation is "
                         "diag(1,-1,-1) to 0.07 rad", "allowed": True},
    "R_X": {"source": "generic controller mechanics: measured home tool_rotation is "
                      "diag(1,-1,-1) to 0.07 rad, so the jaws separate along the tool y axis "
                      "and the fingers point along tool z; R_X points tool z at base -x and "
                      "tool y at base y", "allowed": True},
    "ATTEMPT_DZ": {"source": "debug 51: the bar spans z=table+0.043..table+0.193, so "
                             "attempts 0.025 above and below mid-bar both stay on it",
                   "allowed": True},
    "RESET_POSE": {"source": "debug 51 proprio: home eef is (-0.2085,0,1.1733) with the "
                             "wrist down; y=+0.06 is 0.19 clear of the body's y=-0.13 edge",
                   "allowed": True},
    "ARC_DEG": {"source": "v9/v10 receipts: waypoints exit at the controller's 0.012 m "
                          "position tolerance, so 12 deg is the smallest step that still "
                          "advances the leaf", "allowed": True},
    "ARC_STEPS": {"source": "debug-seed motion budget: horizon 500 control steps at 60 Hz; "
                            "v11 spent 409 on two approach moves and 8 waypoints",
                  "allowed": True},
    "ARC_SHRINK": {"source": "v7 receipt: eef-to-hinge distance grew 0.1207->0.1385 m during "
                             "the swing, so an unshrunk arc walks off the contact",
                   "allowed": True},
    "HINGE_INSET": {"source": "debug 51: the leaf runs from the body -x edge to the bar; "
                              "hinge taken half a cell outside the leftmost panel cell",
                    "allowed": True},
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
RESET_XYZ = [-0.21, 0.06, 1.17]
ATTEMPT_DZ = (0.0, 0.025, -0.025)
R_X = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
PROUD_M = 0.015
REAR_GAP = 0.013
HALF_OPEN = 0.0389
STANDOFF = 0.12
ARC_DEG = 12.0
ARC_STEPS = 8
ARC_SHRINK = 0.97


def yaw(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def cloud(f):
    d = f.depth
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    z = d.astype(float)
    ok = np.isfinite(z) & (z > 0)
    K = f.intrinsics
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x = (uu - cx) * z / fx
    y = (vv - cy) * z / fy
    P = np.stack([x, y, z, np.ones_like(z)], axis=-1) @ np.asarray(f.t_base_cam, float).T
    return P[..., :3], ok


def perceive(api):
    f = api.capture("cam_high")
    P, ok = cloud(f)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    crop = ok & (X > -0.50) & (X < 0.35) & (Y > -0.60) & (Y < 0.60)
    table = float(np.median(Z[crop]))

    body = ok & (X > -0.35) & (X < 0.30) & (Y > -0.45) & (Y < -0.06) & \
        (Z > table + 0.02) & (Z < table + 0.24)
    cell = 0.005
    xlo, zlo = -0.35, table + 0.02
    nx, nz = int(round(0.65 / cell)), int(round(0.22 / cell))
    G = np.full((nz, nx), np.nan)
    ia = np.clip(((Z - zlo) / cell).astype(int), 0, nz - 1)
    ib = np.clip(((X - xlo) / cell).astype(int), 0, nx - 1)
    for p, q, v in zip(ia[body], ib[body], Y[body]):
        if np.isnan(G[p, q]) or v > G[p, q]:
            G[p, q] = v
    panel_y = float(np.nanmedian(G))
    panel = ~np.isnan(G) & (np.abs(G - panel_y) < 0.010)
    hinge_x = float((xlo + (np.nonzero(panel)[1] + 0.5) * cell).min()) - 0.5 * cell

    proud = ~np.isnan(G) & (G > panel_y + PROUD_M)
    pz, px = np.nonzero(proud)
    hx = xlo + (px + 0.5) * cell
    hz = zlo + (pz + 0.5) * cell
    keep = np.abs(hx - float(np.median(hx))) < 0.03
    hx, hz = hx[keep], hz[keep]
    bar_x = float(np.median(hx))
    bar_lo, bar_hi = float(hz.min()), float(hz.max())
    api.log(f"table={table:.4f} panel_y={panel_y:.4f} hinge_x={hinge_x:+.4f} "
            f"bar_x={bar_x:+.4f} bar_z[{bar_lo:.3f},{bar_hi:.3f}] cells={hz.size}")
    return table, panel_y, hinge_x, bar_x, bar_lo, bar_hi


def run(api):
    table, panel_y, hinge_x, bar_x, bar_lo, bar_hi = perceive(api)
    gz = 0.5 * (bar_lo + bar_hi)
    rear_y = panel_y + REAR_GAP
    gy = rear_y + HALF_OPEN
    api.log(f"gz={gz:.4f} rear_finger_y={rear_y:.4f} eef_y={gy:.4f}")

    for i, dz in enumerate(ATTEMPT_DZ):
        r = api.move(RESET_XYZ, rotation=R_DOWN, seconds=1.0)
        api.log(f"reset{i} res={r:.4f} eef={np.round(api.eef(),4).tolist()}")
        r = api.move([bar_x + STANDOFF, gy, gz + dz], rotation=R_X, seconds=1.5)
        api.log(f"sweep{i} dz={dz:+.3f} res={r:.4f} eef={np.round(api.eef(),4).tolist()} "
                f"rot={np.round(api.tool_rotation(),2).tolist()}")
    api.settle(0.2)
    return "v16 hook sweep x3 done"
