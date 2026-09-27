"""v11: drop the fingers into the slot behind the middle drawer's handle bar
and drag the drawer out along the face normal.

Pack evidence: the gripper action is -1 (open) at every demo timestep, so the
drawer is dragged, never gripped; the demo's whole approach and retract lie on
one horizontal axis, the drawer-face normal, with a 0.175 m retract.

Debug-seed geometry (cam_high + cam_arm_wrist depth): the cabinet's drawer face
carries three bars standing 0.030 m proud; a 55 deg down-looking wrist view
sees the middle bar's top surface end 0.021 m short of the face, and sees the
face itself below the bar top through that gap -- i.e. the bar is a rail with
an open slot behind it.  Fingers pointing straight down fit in that slot
(their thickness, not their length, spans it); fingers pointing along the
approach axis do not, which is why every horizontal insertion stalls on the
bar front.
"""
import numpy as np

PROVENANCE = {
    "PULL_AXIS": {"source": "pack ee_path/ee_path6: the whole demo approach and retract lie on one horizontal axis, the drawer face normal", "allowed": True},
    "PULL_LEN": {"source": "pack ee_path: the demo eef travels 0.175 m along that axis from the deepest insertion to the final pose; the drag here commands 8 x 0.025 = 0.20 m to cover it", "allowed": True},
    "OPEN_DRAG": {"source": "pack actions: the gripper channel is -1 (open) at all 138/138/151 timesteps of the three demos, so the handle is dragged, never gripped", "allowed": True},
    "TILT": {"source": "pack ee_path6 rotation vectors give a nose-down tool through the whole contact phase (38/21/31 deg at the deepest insertion); the exact 60 deg used here was selected on debug seeds 51/55 against 30/45/70 deg, which slip or stall", "allowed": True},
    "FINGER_AXIS": {"source": "pack demo rotations put tool y horizontal and across the approach; the debug-seed depth cloud of the open gripper at home shows the two fingers separated along tool y", "allowed": True},
    "DZ": {"source": "debug seeds 51/55 height sweep: fingertips 0.004 m above the wrist-measured bar top engage the bar; +0.016 over-shoots on one seed and -0.004 stalls on the bar front", "allowed": True},
    "PRESS": {"source": "debug seeds 51/55: commanding the tool 0.030 m below the bar top through the drag keeps the fingertip loaded against the handle instead of converging away from it", "allowed": True},
    "TALL_BAND": {"source": "debug seeds 51-65 cam_high depth: the cabinet is the only structure with top z=1.13", "allowed": True},
    "PROUD_THRESH": {"source": "debug seeds 51-65 cam_high depth: the three handle bars stand 0.030 m proud of the drawer face plane, so 0.015 separates bar rows from face rows", "allowed": True},
    "TIP_OFF": {"source": "debug seeds 51/55 contact calibration: the open gripper pressed straight down onto the measured cabinet top (z=1.1278) stalls with the eef 0.0093 above it", "allowed": True},
    "LOOK_TILT": {"source": "debug seed 51: a 55 deg down-looking wrist view resolves the middle bar top and its x span, which the grazing cam_high view does not", "allowed": True},
    "STANDOFF": {"source": "debug-seed depth: 0.10-0.11 m in front of the face clears the 0.030 m proud handle and the gripper", "allowed": True},
    "SLOT_W": {"source": "debug seed 51 cam_arm_wrist 55 deg down view: the middle bar top ends 0.021 m short of the face (fallback constant only)", "allowed": True},
    "GRIP_W": {"source": "debug-seed api.gripper(): the open gripper reports width 0.079 m, matching the demo gripper_state of +-0.04", "allowed": True},
}


PULL_LEN = 0.175
PROUD_THRESH = 0.015
TIP_OFF = 0.0093
TILT = 60.0
DZ = 0.004
PRESS = 0.030
GRIP_W = 0.050
SLOT_W = 0.021
DROP = 0.010
HOVER = 0.055


def _cloud(f):
    d = f.depth
    K = f.intrinsics
    h, w = d.shape
    jj, ii = np.meshgrid(np.arange(w), np.arange(h))
    x = (jj - K[0, 2]) / K[0, 0] * d
    y = (ii - K[1, 2]) / K[1, 1] * d
    T = f.t_base_cam
    return np.stack([x, y, d], -1) @ T[:3, :3].T + T[:3, 3]


def _biggest_component(pts, cell=0.03):
    keys = np.round(pts[:, :2] / cell).astype(int)
    occ = {}
    for i, k in enumerate(map(tuple, keys)):
        occ.setdefault(k, []).append(i)
    seen, best = set(), []
    for k in occ:
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            c = stack.pop()
            comp.append(c)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in occ and n not in seen:
                        seen.add(n)
                        stack.append(n)
        if len(comp) > len(best):
            best = comp
    idx = []
    for c in best:
        idx.extend(occ[c])
    return pts[np.array(idx)]


def _face_scan(box, bb, axis, sgn):
    other = 1 - axis
    s, o, z = box[:, axis] * sgn, box[:, other], box[:, 2]
    olo, ohi = (bb[0], bb[1]) if other == 0 else (bb[2], bb[3])
    zlo, zhi = 0.94, bb[5] - 0.02
    m = (o > olo - 0.01) & (o < ohi + 0.01) & (z > zlo) & (z < zhi)
    s, o, z = s[m], o[m], z[m]
    if len(s) < 150:
        return None
    cell = 0.01
    no = max(2, int(round((ohi - olo) / cell)))
    nz = max(2, int(round((zhi - zlo) / cell)))
    oi = ((o - olo) / cell).astype(int)
    zi = ((z - zlo) / cell).astype(int)
    ok = (oi >= 0) & (oi < no) & (zi >= 0) & (zi < nz)
    grid = np.full((nz, no), -9.0)
    np.maximum.at(grid, (zi[ok], oi[ok]), s[ok])
    filled = grid > -8.0
    prof = np.array([np.percentile(grid[k][filled[k]], 85) if filled[k].sum() > 3 else np.nan
                     for k in range(nz)])
    if not np.isfinite(prof).any():
        return None
    return dict(fill=float(filled.mean()), plane=float(np.nanmedian(prof)), prof=prof,
                grid=grid, filled=filled, zlo=zlo, olo=olo, cell=cell, axis=axis, sgn=sgn)


def perceive(api, tag=""):
    f = api.capture("cam_high")
    P = _cloud(f).reshape(-1, 3)
    ee = np.array(api.eef())
    arm = (np.abs(P[:, 0] - ee[0]) < 0.17) & (np.abs(P[:, 1] - ee[1]) < 0.17)
    tall = ((P[:, 2] > 1.05) & (P[:, 2] < 1.30) & (np.abs(P[:, 0]) < 0.8)
            & (np.abs(P[:, 1]) < 0.8) & (~arm))
    if tall.sum() < 100:
        api.log("%sNO_TALL" % tag)
        return None
    C = _biggest_component(P[tall])
    bb = [float(C[:, 0].min()), float(C[:, 0].max()), float(C[:, 1].min()),
          float(C[:, 1].max()), float(C[:, 2].min()), float(C[:, 2].max())]
    api.log("%sCAB x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f" % (tag, bb[0], bb[1], bb[2], bb[3], bb[5]))
    box = P[(P[:, 0] > bb[0] - 0.04) & (P[:, 0] < bb[1] + 0.04)
            & (P[:, 1] > bb[2] - 0.08) & (P[:, 1] < bb[3] + 0.08)
            & (P[:, 2] > 0.90) & (~arm)]
    cands = []
    for axis, sgn in ((0, -1), (0, 1), (1, -1), (1, 1)):
        r = _face_scan(box, bb, axis, sgn)
        if r is None:
            continue
        ext = bb[2 * axis] if sgn < 0 else bb[2 * axis + 1]
        onedge = abs(r["plane"] - ext * sgn) < 0.055
        r["proud"] = np.where(np.nan_to_num(r["prof"] - r["plane"], nan=-1) > PROUD_THRESH)[0]
        api.log("%sFACE ax=%d sgn=%+d plane=%.3f fill=%.2f edge=%d proud=%s"
                % (tag, axis, sgn, r["plane"] * sgn, r["fill"], onedge, r["proud"].tolist()))
        if onedge and len(r["proud"]) >= 2:
            cands.append(r)
    if not cands:
        api.log("%sNO_FACE" % tag)
        return None
    r = max(cands, key=lambda c: len(c["proud"]))
    groups = []
    for k in r["proud"]:
        if groups and k - groups[-1][-1] <= 2:
            groups[-1].append(k)
        else:
            groups.append([k])
    bars = []
    for b in groups:
        cols = []
        for k in b:
            cols.extend(np.where(r["filled"][k] & (r["grid"][k] > r["plane"] + PROUD_THRESH))[0].tolist())
        if not cols:
            continue
        zc = r["zlo"] + (b[0] + b[-1] + 1) * 0.5 * r["cell"]
        oc = r["olo"] + (float(np.median(cols)) + 0.5) * r["cell"]
        bars.append((zc, oc, len(b) * r["cell"], (max(cols) - min(cols) + 1) * r["cell"]))
        api.log("%sBAR z=%.3f o=%.3f h=%.3f w=%.3f" % ((tag,) + bars[-1]))
    return dict(bb=bb, bars=bars, axis=r["axis"], sgn=r["sgn"], plane=r["plane"] * r["sgn"])


def wrist_bar(api, axis, sgn, plane, zh, oc):
    """Down-looking wrist view: bar top height, bar back, bar x span."""
    try:
        f = api.capture("cam_arm_wrist")
    except Exception as exc:
        api.log("WRIST_ERR %r" % (exc,))
        return None
    P = _cloud(f).reshape(-1, 3)
    s = P[:, axis] * sgn
    keep = ((np.abs(P[:, 1 - axis] - oc) < 0.10) & (np.abs(P[:, 2] - zh) < 0.045)
            & (s > plane * sgn + 0.012) & (s < plane * sgn + 0.055))
    Q = P[keep]
    api.log("WB n=%d" % len(Q))
    if len(Q) < 150:
        return None
    ztop = float(np.percentile(Q[:, 2], 97))
    back = float(np.percentile(Q[:, axis] * sgn, 3)) * sgn
    olo = float(np.percentile(Q[:, 1 - axis], 2))
    ohi = float(np.percentile(Q[:, 1 - axis], 98))
    api.log("WB ztop=%.4f back=%.4f o[%.4f,%.4f]" % (ztop, back, olo, ohi))
    return dict(ztop=ztop, back=back, oc=0.5 * (olo + ohi), ow=ohi - olo)


YAW = -1.0


def down_R(axis, yaw=None):
    """Tool straight down with the fingers separated along the bar direction."""
    z = np.array([0.0, 0.0, -1.0])
    y = np.zeros(3)
    y[1 - axis] = YAW if yaw is None else yaw
    x = np.cross(y, z)
    return np.stack([x, y, z], axis=1)


def rot_err(a, b):
    c = 0.5 * (float(np.trace(a.T @ b)) - 1.0)
    return float(np.arccos(max(-1.0, min(1.0, c))))


def settle_pose(api, p, R, seconds=4.0, tries=3, ptol=0.006, rtol=0.10, tag=""):
    for t in range(tries):
        api.move(p, R, seconds=seconds)
        e = np.array(api.eef())
        pe = float(np.linalg.norm(e - np.array(p)))
        re = rot_err(np.array(api.tool_rotation()), R)
        api.log("%s t%d eef=%s pe=%.4f re=%.3f" % (tag, t, np.round(e, 4).tolist(), pe, re))
        if pe < ptol and re < rtol:
            return True
    return False


def look_R(axis, sgn, tilt_deg):
    t = np.radians(tilt_deg)
    z = np.zeros(3)
    z[axis] = -sgn * np.cos(t)
    z[2] = -np.sin(t)
    z /= np.linalg.norm(z)
    y = np.zeros(3)
    y[1 - axis] = sgn if axis == 1 else -sgn
    x = np.cross(y, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


def run(api):
    api.log("INSTR %r tilt=%.0f" % (api.instruction(), TILT))
    api.grip(0.079)
    st = perceive(api)
    if st is None or not st["bars"]:
        api.log("ABORT_PERCEPTION")
        return
    axis, sgn, plane = st["axis"], st["sgn"], st["plane"]
    bars = sorted(st["bars"])
    if len(bars) >= 3:
        pick = bars[1] if len(bars) == 3 else bars[len(bars) // 2]
    else:
        pick = min(bars, key=lambda b: abs(b[0] - 0.5 * (0.90 + st["bb"][5])))
    zh, oc = pick[0], pick[1]
    api.log("PICK zh=%.3f oc=%.3f nbars=%d axis=%d sgn=%+d plane=%.4f"
            % (zh, oc, len(bars), axis, sgn, plane))

    Rl = look_R(axis, sgn, 55.0)
    p = [0.0, 0.0, zh + 0.13]
    p[axis] = plane + sgn * 0.11
    p[1 - axis] = oc
    api.move(p, Rl, seconds=2.5)
    wb = wrist_bar(api, axis, sgn, plane, zh, oc)
    if wb is not None and 0.03 < wb["ow"] < 0.16 and abs(wb["ztop"] - zh) < 0.05:
        ztop, oc = wb["ztop"], wb["oc"]
    else:
        ztop = zh + 0.003
        api.log("WB_FALLBACK")
    api.log("BARTOP %.4f oc=%.4f" % (ztop, oc))

    R = look_R(axis, sgn, TILT)
    cos, sin = np.cos(np.radians(TILT)), np.sin(np.radians(TILT))

    def pose(tip_front, tip_z):
        p = [0.0, 0.0, tip_z + TIP_OFF * sin]
        p[axis] = plane + sgn * (tip_front + TIP_OFF * cos)
        p[1 - axis] = oc
        return p

    def tf():
        return (api.eef()[axis] - plane) * sgn - TIP_OFF * cos

    zapp = ztop + DZ
    api.move(pose(0.10, zapp), R, seconds=2.5)
    api.log("A2 eef=%s tipfront=%.4f" % (np.round(api.eef(), 4).tolist(), tf()))
    prev = api.eef()[axis]
    for f in (0.070, 0.040, 0.015, -0.008):
        api.move(pose(f, zapp), R, seconds=0.9)
        e = api.eef()
        api.log("IN cmd=%.3f eef=%s tipfront=%.4f" % (f, np.round(e, 4).tolist(), tf()))
        if abs(e[axis] - prev) < 0.003:
            break
        prev = e[axis]
    api.log("REACHED tipfront=%.4f" % tf())

    # settle the fingertips into the corner where the handle top meets the
    # drawer face, then drag out in small steps keeping the z command low so
    # the contact force is never released
    zpress = ztop - 0.030
    f0 = tf()
    api.move(pose(f0, zpress), R, seconds=1.5)
    e = api.eef()
    api.log("PRESS eef=%s tip_z=%.4f tipfront=%.4f" % (np.round(e, 4).tolist(),
            e[2] - TIP_OFF * sin, tf()))
    f0 = tf()
    for k in range(1, 9):
        api.move(pose(f0 + 0.025 * k, zpress), R, seconds=0.9)
        e = api.eef()
        api.log("DRAG%d eef=%s tipfront=%.4f tip_z=%.4f"
                % (k, np.round(e, 4).tolist(), tf(), e[2] - TIP_OFF * sin))
    api.settle(0.3)
    api.log("FINAL eef=%s" % (np.round(api.eef(), 4).tolist(),))
