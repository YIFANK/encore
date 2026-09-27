"""v6: v5 with a complete PROVENANCE block (behaviour identical to v5).

v4 (2/3 on 51,53,55) turned the tool to point along -y and pinched the top
handle bar between an upper and a lower finger, then pulled +y.  Its one
failure, seed 53, closed on air (width 0.0015, effort 0.05): that seed's bar
front sits at y=-0.1898 instead of -0.1864, and the insert move under-shot its
target by 10 mm, leaving the fingertips 3.5 mm short of the bar.

v5 therefore (a) reads the bar's own front y, top z and clear x span from
cam_high on this episode instead of using frozen numbers, (b) re-issues the
insert so it actually converges, and (c) checks gripper effort after closing
and, if it caught nothing, reopens and tries again 8 mm deeper.  v3 showed the
episode budget is 1000 sim steps and v4 used only 214, so the extra moves fit.
"""
import numpy as np

PROVENANCE = {
    "BRIGHT": {"source": "debug seeds 51/53 cam_high: handle bars are the only near-grey pixels on the dark cabinet front", "allowed": True},
    "TOP_BAND": {"source": "debug seeds 51/53 cam_high: three handle z peaks 0.951/1.021/1.096; top drawer = highest", "allowed": True},
    "SLOT_Y": {"source": "debug seeds 51/53 cam_high: cabinet front plane y=-0.2175, bar front -0.186..-0.190", "allowed": True},
    "TIP_DZ": {"source": "debug seed 51 v3: closed gripper stalled at eef_z 1.1424 on the z=1.127 cabinet top slab", "allowed": True},
    "GRASP_DZ": {"source": "debug seed 51 v4: successful pinch held at eef_z 1.0907 with bar top z 1.098 -> 8 mm below the top face", "allowed": True},
    "PALM_DY": {"source": "debug seed 51 v4: palm at eef_y -0.179 with bar front -0.1864 gripped; aim the palm 4 mm past the bar front", "allowed": True},
    "R_FRONT": {"source": "generic tool-frame mechanics: tool z = approach = -y, tool y = finger axis = +z", "allowed": True},
    "PULL_DY": {"source": "debug seed 51 v4: +0.146 m of realised travel fired the predicate; command 0.22", "allowed": True},
    "HELD_EFFORT": {"source": "FairApi contract: effort 3.0 iff holding (seen 3.00 when gripped, 0.05 when empty)", "allowed": True},
    "STAGE_Y": {"source": "chosen standoff in free air in front of the cabinet; debug seeds 51/53 cam_high show nothing between y=-0.06 and the bar", "allowed": True},
    "FB_BAR_FRONT_Y": {"source": "debug seeds 51/53 cam_high: bar front y -0.1864/-0.1898, midpoint", "allowed": True},
    "FB_BAR_TOP_Z": {"source": "debug seeds 51/53 cam_high: top-handle bright top face z max 1.0980", "allowed": True},
    "FB_BAR_X": {"source": "debug seeds 51/53 cam_high: bar x span -0.04..0.05 with posts at -0.030/+0.035 -> clear centre 0.003", "allowed": True},
}

TIP_DZ = 0.0154
GRASP_DZ = 0.008         # eef sits this far below the bar's top face
PALM_DY = 0.004          # drive the palm this far past the bar's front face
PULL_DY = 0.22
STAGE_Y = -0.060
HELD_EFFORT = 1.0

R_FRONT = np.array([[1.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0],
                    [0.0, 1.0, 0.0]])

# fallbacks, used only if perception of the bar fails outright
FB_BAR_FRONT_Y = -0.188
FB_BAR_TOP_Z = 1.098
FB_BAR_X = 0.003


def st(api, tag):
    e = np.asarray(api.eef())
    g = api.gripper()
    api.log("%-12s eef=[%.4f %.4f %.4f] w=%.4f eff=%.2f" % (
        tag, e[0], e[1], e[2], g["width_m"], g["effort"]))
    return e


def mv(api, tag, xyz, rot, sec, tries=1):
    for _ in range(tries):
        api.move([float(v) for v in xyz], rotation=rot, seconds=sec)
    e = st(api, tag)
    api.log("   tgt=[%.4f %.4f %.4f] err=%.4f" % (
        xyz[0], xyz[1], xyz[2], float(np.linalg.norm(e - np.asarray(xyz)))))
    return e


def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    dep = np.asarray(f.depth)
    K = np.asarray(f.intrinsics)
    T = np.asarray(f.t_base_cam)
    H, W = dep.shape
    vv, uu = np.mgrid[0:H, 0:W]
    P = np.stack([(uu - K[0, 2]) / K[0, 0] * dep, (vv - K[1, 2]) / K[1, 1] * dep, dep], -1)
    return f.rgb.astype(float), P @ T[:3, :3].T + T[:3, 3]


def find_top_bar(api):
    """Highest of the cabinet's grey handle bars: its front y, top z, clear x."""
    rgb, B = cloud(api)
    mx = rgb.max(-1); mn = rgb.min(-1)
    bright = (mx > 105) & (mx < 240) & ((mx - mn) < 28)
    # bars live on the cabinet front, left of the robot and in front of the box
    m = bright & (B[:, :, 1] > -0.26) & (B[:, :, 1] < -0.12) & (B[:, :, 2] > 0.92) & (B[:, :, 2] < 1.12)
    if m.sum() < 60:
        api.log("perc: no bar pixels (n=%d), using fallbacks" % m.sum())
        return FB_BAR_FRONT_Y, FB_BAR_TOP_Z, FB_BAR_X, False
    p = B[m]
    # three bars separate cleanly in z; take the highest cluster
    zs = np.sort(p[:, 2])
    top_z = zs[-1]
    sel = p[p[:, 2] > top_z - 0.030]
    if len(sel) < 40:
        api.log("perc: thin top cluster (n=%d), using fallbacks" % len(sel))
        return FB_BAR_FRONT_Y, FB_BAR_TOP_Z, FB_BAR_X, False
    front_y = float(np.percentile(sel[:, 1], 99))
    bar_top_z = float(np.percentile(sel[:, 2], 99))
    # posts break the bar near its ends; the clear span is around its x middle
    bar_x = float(0.5 * (np.percentile(sel[:, 0], 2) + np.percentile(sel[:, 0], 98)))
    api.log("perc: top bar n=%d front_y=%.4f top_z=%.4f x=%.4f (x span %.3f..%.3f)" % (
        len(sel), front_y, bar_top_z, bar_x,
        np.percentile(sel[:, 0], 2), np.percentile(sel[:, 0], 98)))
    return front_y, bar_top_z, bar_x, True


def run(api):
    st(api, "t0")
    front_y, bar_top_z, bar_x, ok = find_top_bar(api)
    grasp_z = bar_top_z - GRASP_DZ
    grasp_y = front_y - PALM_DY
    api.log("plan: x=%.4f grasp_y=%.4f grasp_z=%.4f (tips reach y=%.4f)" % (
        bar_x, grasp_y, grasp_z, grasp_y - TIP_DZ))

    api.grip(0.08)
    mv(api, "stage", [bar_x, STAGE_Y, grasp_z], R_FRONT, 2.5)

    held = False
    for att in range(3):
        y = grasp_y - 0.008 * att
        mv(api, "insert%d" % att, [bar_x, y, grasp_z], R_FRONT, 1.5, tries=2)
        api.grip(0.0)
        api.settle(0.3)
        g = api.gripper()
        api.log("close%d w=%.4f eff=%.2f" % (att, g["width_m"], g["effort"]))
        if g["effort"] >= HELD_EFFORT and g["width_m"] > 0.005:
            held = True
            break
        api.grip(0.08)
        api.settle(0.2)
    api.log("held=%s" % held)

    e = st(api, "pre_pull")
    mv(api, "pull", [e[0], e[1] + PULL_DY, e[2]], R_FRONT, 3.0)
    api.settle(0.3)
    e2 = st(api, "after")
    api.log("realised dy=%.4f" % (e2[1] - e[1]))

    fy, tz, bx, ok2 = find_top_bar(api)
    api.log("post: top bar front_y=%.4f top_z=%.4f (was %.4f)" % (fy, tz, front_y))
    api.log("v6 done")
