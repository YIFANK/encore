"""c2 / goal_open_top_drawer_put_bowl_stock -- v7

Debug-seed receipts behind every choice here:
  * EEF ref sits 0.0066 m above the fingertips (descend-to-stall on bare table);
    jaw axis = world y with the wrist straight down, open half-gap 0.0389.
  * The top drawer's handle is a lip protruding 0.031 m from the drawer face,
    z-span 0.025 m, under a cabinet lid whose front edge is flush with the face.
    v1/v3/v4 bracketed the lip-to-face slot by stall height and it is under
    5 mm wide -> a straight-down finger CANNOT hook it (0/4, 0/4, 0/2).
  * Turned 90 deg about world x the jaw axis becomes world z, the open jaws
    straddle the lip inside the corridor between the lid and the next lip down,
    and clamp it: measured clamp gap 0.0174 m, held through a 0.16 m pull,
    drawer out on 4/4 debug seeds (v5/v6).
  * The pulled-out drawer overhangs the bowl, so the bowl is located BEFORE the
    pull, and the pull length is capped so the grasping finger still clears the
    drawer's front wall; the pinched bowl is dragged clear before it is lifted.
  * Bowl grasp = rim pinch (c2-L4): the bowl is 0.11 m wide, the jaws 0.078 m;
    measured pinch gap 0.0088 m on the rim wall (v6 ep55).
"""
import json

import numpy as np

PROVENANCE = {
    "FINGER_DZ": {"source": "debug-seed measurement: descend-to-stall on bare table "
                            "(eef z 0.9091 vs depth-histogram table plane 0.9025)",
                  "allowed": True},
    "FINGER_DY": {"source": "api.gripper() open width 0.0778 / 2; jaw axis from "
                            "api.tool_rotation()", "allowed": True},
    "R_SIDE": {"source": "generic tool-frame mechanics: R_x(90 deg) applied to the "
                         "straight-down tool frame", "allowed": True},
    "R_DOWN": {"source": "generic tool-frame mechanics: the straight-down tool frame "
                         "api.tool_rotation() reports at episode start", "allowed": True},
    "GRASP_INSET": {"source": "debug-seed relief map: lip front at face+0.031; clamp "
                              "8 mm behind it", "allowed": True},
    "PULL_MAX": {"source": "pack demos ee_path6: hook sweep start -> end y (0.158-0.177)",
                 "allowed": True},
    "LIP_CLEAR": {"source": "debug-seed v7: the -y finger stalled on the travelled lip; "
                            "14 mm keeps the finger stock clear of it", "allowed": True},
    "FINGER_OUT": {"source": "debug-seed v8: the -y finger's outer stock reaches 0.053 "
                             "from the tool axis (stall geometry against the travelled lip)",
                   "allowed": True},
    "RIM_OUT": {"source": "debug-seed geometry: the jaws (half-gap 0.0389) still straddle "
                          "the rim wall with the EEF offset up to 28 mm outward",
                "allowed": True},
    "GRASP_CLEAR": {"source": "debug-seed geometry: open half-gap 0.0389 + 0.020 margin "
                              "between the -y finger and the drawer's front wall",
                    "allowed": True},
    "GRASP_DZ": {"source": "pack demos: close-keyframe EEF z (mean 0.931) vs the debug-seed "
                           "bowl rim top (0.9521) -> 0.020 below the rim", "allowed": True},
    "ANCHOR": {"source": "pack demos: EEF xy at the gripper-close keyframes "
                         "(-0.107,0.072) (-0.103,0.025) (-0.108,0.039) (-0.114,0.010)",
               "allowed": True},
    "HELD_GAP": {"source": "debug-seed measurement: pinch gap 0.0088 on the bowl rim "
                           "(v6 ep55) vs ~0 closing on air", "allowed": True},
    "DROP_DZ": {"source": "pack demos: release-keyframe EEF z 0.03-0.05 above the drawer "
                          "rim measured on debug seeds", "allowed": True},
    "PARK": {"source": "debug-seed v2/v6: pose from which cam_high sees the bowl "
                       "unoccluded by the arm", "allowed": True},
    "CELL": {"source": "generic depth-clustering mechanics", "allowed": True},
}

FINGER_DZ = 0.0066
FINGER_DY = 0.0389
GRASP_INSET = 0.008
PULL_MAX = 0.160
GRASP_CLEAR = 0.020
LIP_CLEAR = 0.022
FINGER_OUT = 0.053
RIM_OUT = 0.028
GRASP_DZ = -0.015
ANCHOR = (-0.108, 0.036)
HELD_GAP = (0.0012, 0.013)
DROP_DZ = 0.035
CELL = 0.015
R_SIDE = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
R_DOWN = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]


# ---------------------------------------------------------------- perception
def cloud(api, cam="cam_high"):
    f = api.capture(cam)
    d = np.asarray(f.depth, dtype=float)
    K = np.asarray(f.intrinsics, dtype=float)
    T = np.asarray(f.t_base_cam, dtype=float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    x = (uu - K[0, 2]) * d / K[0, 0]
    y = (vv - K[1, 2]) * d / K[1, 1]
    P = np.stack([x, y, d], axis=-1) @ T[:3, :3].T + T[:3, 3]
    V = np.isfinite(d) & (d > 0.05) & (d < 5.0)
    return f, P, V


def table_height(P, V):
    z = P[..., 2]
    m = V & (np.abs(P[..., 0]) < 0.55) & (np.abs(P[..., 1]) < 0.55) & (z > 0.4) & (z < 1.5)
    hist, edges = np.histogram(z[m], bins=220, range=(0.4, 1.5))
    i = int(np.argmax(hist))
    return float(0.5 * (edges[i] + edges[i + 1]))


def lid(P, V, tz):
    """The cabinet lid is the one big horizontal slab on the -y side: its modal
    height, x extent and front edge define the cabinet without any clustering
    (which merges the cabinet with the shelf behind it on some seeds)."""
    m = (V & (P[..., 1] < -0.05) & (np.abs(P[..., 0]) < 0.45)
         & (P[..., 2] > tz + 0.15) & (P[..., 2] < tz + 0.30))
    p = P[m]
    if p.shape[0] < 500:
        return None
    hist, edges = np.histogram(p[:, 2], bins=150, range=(tz + 0.15, tz + 0.30))
    i = int(np.argmax(hist))
    zc = float(0.5 * (edges[i] + edges[i + 1]))
    s = p[np.abs(p[:, 2] - zc) < 0.006]
    if s.shape[0] < 300:
        return None
    return {"z": zc, "n": int(s.shape[0]),
            "xlo": float(np.percentile(s[:, 0], 1)), "xhi": float(np.percentile(s[:, 0], 99)),
            "face": float(np.percentile(s[:, 1], 99))}


def lips(P, V, lid_d, tz):
    """Bands on the cabinet's +y face that protrude past the face plane."""
    m = (V & (P[..., 0] > lid_d["xlo"] - 0.01) & (P[..., 0] < lid_d["xhi"] + 0.01)
         & (P[..., 1] > lid_d["face"] - 0.06) & (P[..., 1] < lid_d["face"] + 0.06)
         & (P[..., 2] > tz + 0.03) & (P[..., 2] < lid_d["z"] - 0.012))
    p = P[m]
    bands, zb = [], tz + 0.03
    while zb < lid_d["z"] - 0.012:
        s = p[(p[:, 2] >= zb) & (p[:, 2] < zb + 0.005)]
        if s.shape[0] >= 20:
            bands.append((round(float(zb), 4), float(s[:, 1].max())))
        zb += 0.005
    if not bands:
        return []
    face = float(np.percentile([b[1] for b in bands], 55))
    groups = []
    for zb, ym in bands:
        if ym <= face + 0.012:
            continue
        if groups and zb - groups[-1][-1][0] < 0.0075:
            groups[-1].append((zb, ym))
        else:
            groups.append([(zb, ym)])
    return groups


def clusters(f, P, V, tz, zmin=0.025, xlim=(-0.50, 0.45), ylim=(-0.45, 0.45), band=None):
    z = P[..., 2]
    m = (V & (z > tz + zmin) & (z < tz + 0.60)
         & (P[..., 0] > xlim[0]) & (P[..., 0] < xlim[1])
         & (P[..., 1] > ylim[0]) & (P[..., 1] < ylim[1]))
    if not m.any():
        return []
    pts = P[m]
    rgb = np.asarray(f.rgb, dtype=float)[m]
    gx = np.floor((pts[:, 0] - xlim[0]) / CELL).astype(int)
    gy = np.floor((pts[:, 1] - ylim[0]) / CELL).astype(int)
    nx = int(np.ceil((xlim[1] - xlim[0]) / CELL)) + 1
    ny = int(np.ceil((ylim[1] - ylim[0]) / CELL)) + 1
    cnt = np.zeros((nx, ny), dtype=int)
    np.add.at(cnt, (gx, gy), 1)
    occ = cnt >= 3
    if band is not None:
        cellmax = np.full((nx, ny), -9.0)
        np.maximum.at(cellmax, (gx, gy), pts[:, 2])
        occ = occ & (cellmax > tz + band[0]) & (cellmax < tz + band[1])
        keep = occ[gx, gy]
        pts, rgb, gx, gy = pts[keep], rgb[keep], gx[keep], gy[keep]
        if pts.shape[0] < 10:
            return []
    lab = -np.ones((nx, ny), dtype=int)
    groups = []
    for i in range(nx):
        for j in range(ny):
            if not occ[i, j] or lab[i, j] >= 0:
                continue
            stack, cells, k = [(i, j)], [], len(groups)
            lab[i, j] = k
            while stack:
                a, b = stack.pop()
                cells.append((a, b))
                for da in (-1, 0, 1):
                    for db in (-1, 0, 1):
                        p, q = a + da, b + db
                        if 0 <= p < nx and 0 <= q < ny and occ[p, q] and lab[p, q] < 0:
                            lab[p, q] = k
                            stack.append((p, q))
            groups.append(cells)
    out = []
    cell_of = lab[gx, gy]
    for k, cells in enumerate(groups):
        sel = cell_of == k
        p, c = pts[sel], rgb[sel]
        out.append({"id": k, "cells": len(cells), "npts": int(sel.sum()),
                    "xlo": round(float(p[:, 0].min()), 4), "xhi": round(float(p[:, 0].max()), 4),
                    "ylo": round(float(p[:, 1].min()), 4), "yhi": round(float(p[:, 1].max()), 4),
                    "zmax": round(float(p[:, 2].max()), 4),
                    "cx": round(float(0.5 * (p[:, 0].min() + p[:, 0].max())), 4),
                    "cy": round(float(0.5 * (p[:, 1].min() + p[:, 1].max())), 4),
                    "rgb": [int(v) for v in c.mean(axis=0).round()]})
    out.sort(key=lambda d: -d["cells"])
    return out


def rim_ring(P, V, cx0, cy0, ztop, span=0.085):
    """Points on the bowl's rim ring, near a seed centre: bbox midpoint (c2-L5)
    and half-width along the jaw axis."""
    m = (V & (np.abs(P[..., 0] - cx0) < span) & (np.abs(P[..., 1] - cy0) < span)
         & (P[..., 2] > ztop - 0.008) & (P[..., 2] < ztop + 0.005))
    p = P[m]
    if p.shape[0] < 40:
        return None
    cx = float(0.5 * (np.percentile(p[:, 0], 2) + np.percentile(p[:, 0], 98)))
    cy = float(0.5 * (np.percentile(p[:, 1], 2) + np.percentile(p[:, 1], 98)))
    s = p[np.abs(p[:, 0] - cx) < 0.018]
    if s.shape[0] < 20:
        return None
    r = 0.5 * (float(np.percentile(s[:, 1], 98)) - float(np.percentile(s[:, 1], 2)))
    return {"cx": round(cx, 4), "cy": round(cy, 4), "r": round(float(r), 4),
            "n": int(p.shape[0])}


def rim_radius(P, V, bowl):
    m = (V & (np.abs(P[..., 0] - bowl["cx"]) < 0.018)
         & (P[..., 1] > bowl["ylo"] - 0.02) & (P[..., 1] < bowl["yhi"] + 0.02)
         & (P[..., 2] > bowl["zmax"] - 0.010) & (P[..., 2] < bowl["zmax"] + 0.005))
    p = P[m]
    if p.shape[0] < 20:
        return float(min(0.058, max(0.035, 0.5 * (bowl["yhi"] - bowl["ylo"]) - 0.006)))
    r = 0.5 * (float(np.percentile(p[:, 1], 98)) - float(np.percentile(p[:, 1], 2)))
    return float(min(0.058, max(0.035, r)))


def region_top(P, V, xr, yr, zfloor):
    m = (V & (P[..., 0] > xr[0]) & (P[..., 0] < xr[1])
         & (P[..., 1] > yr[0]) & (P[..., 1] < yr[1]) & (P[..., 2] > zfloor))
    p = P[m]
    if p.shape[0] < 10:
        return {"n": int(p.shape[0]), "zmax": None}
    return {"n": int(p.shape[0]), "zmax": round(float(np.percentile(p[:, 2], 98)), 4),
            "zmed": round(float(np.median(p[:, 2])), 4)}


def ymap(P, V, xr, y0, y1, zfloor, step=0.02):
    rows, yb = [], y0
    while yb < y1:
        m = (V & (P[..., 0] > xr[0]) & (P[..., 0] < xr[1])
             & (P[..., 1] >= yb) & (P[..., 1] < yb + step) & (P[..., 2] > zfloor))
        p = P[m]
        rows.append([round(yb, 3), int(p.shape[0]),
                     round(float(np.percentile(p[:, 2], 98)), 4) if p.shape[0] > 8 else None])
        yb += step
    return rows


def say(api, **kw):
    api.log(json.dumps(kw, default=float))


# ------------------------------------------------------------------- program
def run(api):
    f, P, V = cloud(api)
    tz = table_height(P, V)
    L = lid(P, V, tz)
    say(api, table_z=round(tz, 4), lid={k: round(v, 4) for k, v in L.items()} if L else None)
    if L is None:
        return "no cabinet lid"
    gs = lips(P, V, L, tz)
    say(api, lips=[[round(g[0][0], 4), round(g[-1][0] + 0.005, 4),
                    round(max(y for _, y in g), 4)] for g in gs])
    if not gs:
        return "no handle lip"
    top = gs[-1]
    lip_zlo, lip_zhi = float(top[0][0]), float(top[-1][0] + 0.005)
    lip_y = float(max(y for _, y in top))
    face = float(L["face"])
    lid_z = float(L["z"])
    bar_x = float(0.5 * (L["xlo"] + L["xhi"]))
    mid_top = float(gs[-2][-1][0] + 0.005) if len(gs) > 1 else lip_zlo - 0.06
    insert_z = 0.5 * (lid_z + mid_top)
    say(api, face=round(face, 4), lip_z=[round(lip_zlo, 4), round(lip_zhi, 4)],
        lip_y=round(lip_y, 4), bar_x=round(bar_x, 4), insert_z=round(insert_z, 4))

    RS, RD = np.asarray(R_SIDE, float), np.asarray(R_DOWN, float)
    api.grip(0.08)

    # 1. park where cam_high sees the bowl, turning the wrist on the way
    r = api.move([bar_x, 0.29, lid_z + 0.10], rotation=RS, seconds=3.0)
    say(api, mv="park", res=round(r, 4), eef=[round(float(v), 4) for v in api.eef()])
    fp, Pp, Vp = cloud(api)
    clp = clusters(fp, Pp, Vp, tz, xlim=(-0.32, 0.22), ylim=(face + 0.03, 0.28),
                   band=(0.030, 0.058))
    for c in clp[:5]:
        say(api, cl_park=c)
    cands = [c for c in clp
             if 12 <= c["cells"] <= 80 and tz + 0.030 < c["zmax"] < tz + 0.062
             and (c["xhi"] - c["xlo"]) < 0.18 and (c["yhi"] - c["ylo"]) < 0.18]
    if not cands:
        return "no bowl candidate"
    cands.sort(key=lambda c: np.hypot(c["cx"] - ANCHOR[0], c["cy"] - ANCHOR[1]))
    bowl = cands[0]
    r_rim = rim_radius(Pp, Vp, bowl)
    bx, by = float(bowl["cx"]), float(bowl["cy"])
    ring = rim_ring(Pp, Vp, bx, by, float(bowl["zmax"]))
    if ring is not None and 0.035 <= ring["r"] <= 0.062:
        bx, by, r_rim = float(ring["cx"]), float(ring["cy"]), float(ring["r"])
    say(api, bowl=bowl, ring=ring, bx=round(bx, 4), by=round(by, 4),
        r_rim=round(r_rim, 4))

    # The lip travels with the drawer and ends up over the bowl's near rim: the
    # descending -y finger has to clear it (v7 receipt: it stalled on the lip at
    # z~1.085 on 7/7 seeds).  Cap the pull so the grasp point, offset outward by
    # at most RIM_OUT, still leaves the finger clear.
    rim_y = by + r_rim
    pull = float(min(PULL_MAX, rim_y + RIM_OUT - lip_y - FINGER_OUT - LIP_CLEAR))
    pull = float(max(0.115, pull))
    say(api, pull=round(pull, 4), rim_y=round(rim_y, 4))

    # 2. clamp the lip and pull the drawer out
    r = api.move([bar_x, face + 0.16, insert_z], rotation=RS, seconds=2.0)
    say(api, mv="pre", res=round(r, 4), eef=[round(float(v), 4) for v in api.eef()])
    grasp_y = lip_y - GRASP_INSET + FINGER_DZ
    r = api.move([bar_x, grasp_y, insert_z], rotation=RS, seconds=1.5)
    e = api.eef()
    say(api, mv="insert", res=round(r, 4), eef=[round(float(v), 4) for v in e])
    api.grip(0.0)
    g = api.gripper()
    say(api, clamp_gap=round(g["width_m"], 5))
    r = api.move([bar_x, float(e[1]) + pull, float(e[2])], rotation=RS, seconds=2.2)
    e2 = api.eef()
    say(api, mv="pull", res=round(r, 4), eef=[round(float(v), 4) for v in e2],
        gap=round(api.gripper()["width_m"], 5))
    y_front = face + (float(e2[1]) - float(e[1]))
    api.grip(0.08)
    api.move([bar_x, float(e2[1]) + 0.05, lid_z + 0.08], rotation=RS, seconds=1.2)
    say(api, y_front=round(y_front, 4))

    # 3. rim pinch on the bowl (c2-L4), wrist back to straight down
    lip_front = lip_y + (float(e2[1]) - float(e[1]))
    gx = bx
    gy = float(min(rim_y + RIM_OUT, max(rim_y, lip_front + FINGER_OUT + LIP_CLEAR)))
    say(api, lip_front=round(lip_front, 4), gy=round(gy, 4),
        hold_off=round(gy - rim_y, 4))
    gz = float(bowl["zmax"]) + GRASP_DZ
    hold_off = gy - rim_y
    r = api.move([-0.15, 0.14, lid_z + 0.09], rotation=RD, seconds=2.5)
    pr = api.proprio()
    say(api, mv="recover", res=round(r, 4), eef=[round(float(v), 4) for v in api.eef()],
        rot=[round(float(v), 3) for v in np.asarray(api.tool_rotation()).ravel()],
        joints=pr.get("robot0_joint_pos"))
    r = api.move([gx, gy, tz + 0.20], seconds=1.5)
    say(api, mv="over_bowl", res=round(r, 4), eef=[round(float(v), 4) for v in api.eef()],
        rot=[round(float(v), 3) for v in np.asarray(api.tool_rotation()).ravel()],
        joints=api.proprio().get("robot0_joint_pos"))
    r = api.move([gx, gy, gz], seconds=1.2)
    say(api, mv="descend", res=round(r, 4), tgt=[round(gx, 4), round(gy, 4), round(gz, 4)],
        eef=[round(float(v), 4) for v in api.eef()],
        joints=api.proprio().get("robot0_joint_pos"))
    if r > 0.02:
        fx, Px, Vx = cloud(api, "cam_arm_wrist")
        ee0 = api.eef()
        say(api, stall_probe=region_top(Px, Vx, (float(ee0[0]) - 0.10, float(ee0[0]) + 0.10),
                                        (float(ee0[1]) - 0.12, float(ee0[1]) + 0.12), tz + 0.02),
            rot=[round(float(v), 3) for v in np.asarray(api.tool_rotation()).ravel()])
        api.move([gx, gy, float(ee0[2]) + 0.06], seconds=0.8)
        r = api.move([gx, gy, gz], seconds=1.2)
        say(api, mv="descend2", res=round(r, 4), eef=[round(float(v), 4) for v in api.eef()])
    api.grip(0.0)
    g1 = api.gripper()
    say(api, pinch_gap=round(g1["width_m"], 5))
    if g1["width_m"] < HELD_GAP[0]:
        # closed on air: re-perceive the rim from directly overhead and retry
        api.grip(0.08)
        api.move([gx, gy, gz + 0.11], seconds=1.0)
        fw2, Pw2, Vw2 = cloud(api, "cam_arm_wrist")
        ring2 = rim_ring(Pw2, Vw2, gx, gy - r_rim, float(bowl["zmax"]), span=0.10)
        say(api, retry_ring=ring2)
        if ring2 is not None and 0.035 <= ring2["r"] <= 0.062:
            gx = float(ring2["cx"])
            gy = float(min(ring2["cy"] + ring2["r"] + RIM_OUT,
                           max(ring2["cy"] + ring2["r"],
                               lip_front + FINGER_OUT + LIP_CLEAR)))
            hold_off = gy - (ring2["cy"] + ring2["r"])
            r_rim = float(ring2["r"])
        r = api.move([gx, gy, gz], seconds=1.2)
        api.grip(0.0)
        g1 = api.gripper()
        say(api, retry_pinch=round(g1["width_m"], 5), res=round(r, 4),
            tgt=[round(gx, 4), round(gy, 4)],
            eef=[round(float(v), 4) for v in api.eef()])

    # 4. drag clear of the drawer's overhang at low height, then lift
    drag_y = float(max(gy, gy + (y_front + 0.012 + r_rim - by)))
    api.move([gx, gy, gz + 0.055], seconds=1.0)
    if drag_y > gy + 0.005:
        api.move([gx, drag_y, gz + 0.055], seconds=1.2)
    r = api.move([gx, drag_y, lid_z + 0.09], seconds=1.5)
    g2 = api.gripper()
    held = HELD_GAP[0] < g2["width_m"] < HELD_GAP[1]
    say(api, mv="lift", res=round(r, 4), drag_y=round(drag_y, 4),
        gap=round(g2["width_m"], 5), held=held,
        eef=[round(float(v), 4) for v in api.eef()])

    # 5. drop into the open drawer
    fd, Pd, Vd = cloud(api)
    dm = ymap(Pd, Vd, (L["xlo"] + 0.01, L["xhi"] - 0.01), face - 0.01, y_front + 0.04,
              tz + 0.10)
    rim_z = max([row[2] for row in dm
                 if row[2] is not None and row[2] < lid_z + 0.006] or [lid_z])
    say(api, drawer_map=dm, rim_z=round(float(rim_z), 4))
    drop_y = 0.5 * (face + y_front) + r_rim + hold_off
    drop_x = bar_x
    drop_z = float(rim_z) + DROP_DZ
    r = api.move([drop_x, drop_y, lid_z + 0.09], seconds=2.0)
    r2 = api.move([drop_x, drop_y, drop_z], seconds=1.0)
    say(api, mv="drop", res=round(r, 4), res2=round(r2, 4),
        tgt=[round(drop_x, 4), round(drop_y, 4), round(drop_z, 4)],
        eef=[round(float(v), 4) for v in api.eef()])
    api.grip(0.08)
    api.settle(0.4)
    api.move([drop_x, drop_y, lid_z + 0.10], seconds=1.0)
    fe, Pe, Ve = cloud(api)
    say(api, end_map=ymap(Pe, Ve, (L["xlo"] + 0.01, L["xhi"] - 0.01), face - 0.01,
                          y_front + 0.10, tz + 0.10))
    for c in clusters(fe, Pe, Ve, tz, xlim=(-0.32, 0.22), ylim=(face, 0.28))[:5]:
        say(api, cl_end=c)
    return "v7 pull=%.3f held=%s" % (pull, held)
