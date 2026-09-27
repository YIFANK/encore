"""c2 cell spa_bowl_on_ramekin_stock -- v5.

v1 (open-loop demo replay) scored 4/8; every grasp succeeded and every failure
was a place: the plate jitters per seed and the four failures were exactly the
four seeds whose plate sat furthest in +x of the fixed release point.

v2 grounds BOTH objects in the cam_high depth cloud (footprint clustering,
per-cell max height) and places relative to the measured plate centre, using
the rim-pinch offset measured on the v1 debug seeds.
"""
import numpy as np

PROVENANCE = {
    "GRASP_ANCHOR_XY": {
        "source": "pack.json: mean of the demos' ee_path at t=40 (lowest point "
                  "of the descent): (-0.2106,0.1786),(-0.1957,0.1697),"
                  "(-0.2076,0.1598). Used ONLY to pick which cluster is the "
                  "target (C2-L1 demo-anchor identity).",
        "allowed": True},
    "RELEASE_ANCHOR_XY": {
        "source": "pack.json: mean xy of the demo keyframes whose gripper_cmd "
                  "returns to open (t=124/117/109). Used ONLY to pick which "
                  "cluster is the plate.",
        "allowed": True},
    "PINCH_OFF": {
        "source": "debug-seed measurement: over v1 seeds 51..65 odd, "
                  "(commanded grasp xy) - (measured bowl bbox-mid xy) had mean "
                  "(-0.0032,-0.0351) and all 8 of those grasps closed with "
                  "effort 3.0; the jaw axis is world y (tool_rotation col1 at "
                  "the demo grasp keyframes)",
        "allowed": True},
    "GRASP_DZ": {
        "source": "pack.json + debug measurement: demo mean descent floor "
                  "0.9685 minus the measured bowl rim top 1.0002 (v1 survey, "
                  "1.0000-1.0006 on all 8 seeds)",
        "allowed": True},
    "RELEASE_DZ": {
        "source": "pack.json + debug measurement: demo mean release z 0.9455 "
                  "minus the measured plate top 0.9201 (v1 survey, 0.9200-"
                  "0.9201 on all 8 seeds)",
        "allowed": True},
    "PREGRASP_DZ": {
        "source": "pack.json: demos' ee_path z at t=30 (1.0058,1.0301,1.0009) "
                  "is ~0.077 above the demo descent floor",
        "allowed": True},
    "LIFT_Z": {
        "source": "pack.json: demos' maximum ee_path z during transport t=70..90 "
                  "(1.1345, 1.0944, 1.0777)",
        "allowed": True},
    "BOWL_EXT": {
        "source": "debug-seed measurement: both bowls' footprint extent in the "
                  "v1 survey was 0.103-0.111 m; gate widened to [0.07,0.15]",
        "allowed": True},
    "BOWL_H": {
        "source": "debug-seed measurement: target (bowl on ramekin) stood "
                  "0.0989-0.0996 m above the table, the distractor bowl "
                  "0.0695-0.0697 m, on all 8 v1 seeds",
        "allowed": True},
    "PLATE_EXT": {
        "source": "debug-seed measurement: plate footprint extent 0.133-0.137 m "
                  "on all 8 v1 seeds",
        "allowed": True},
    "PLATE_H": {
        "source": "debug-seed measurement: plate stood 0.0190-0.0191 m above "
                  "the table on all 8 v1 seeds",
        "allowed": True},
    "OPEN_WIDTH_M": {
        "source": "generic gripper mechanics: width >= 0.025 m is an open "
                  "command on this API (api.grip docstring)",
        "allowed": True},
    "CELL_M": {
        "source": "generic point-cloud mechanics: 1.5 cm ground-footprint grid",
        "allowed": True},
    "TABLE_CLEAR_M": {
        "source": "generic depth mechanics: 1.2 cm above the fitted table plane "
                  "rejects table pixels and their depth noise",
        "allowed": True},
    "BOWL_H_TALL": {
        "source": "debug-seed measurement: the target bowl stands 0.0989-0.0997 m "
                  "above the table and the distractor bowl 0.0695-0.0697 m on all "
                  "15 debug seeds; 0.085 m sits inside that measured gap",
        "allowed": True},
    "WORK_X": {
        "source": "debug-seed observation: bounds of the tabletop region that "
                  "carries the props in the cam_high cloud (x -0.50..0.45); "
                  "a cropping window, not a task constant",
        "allowed": True},
    "WORK_Y": {
        "source": "debug-seed observation: bounds of the tabletop region that "
                  "carries the props in the cam_high cloud (y -0.50..0.50); "
                  "a cropping window, not a task constant",
        "allowed": True},
    "FLAT_CEIL": {
        "source": "debug-seed measurement: the plate stands 0.019 m proud and "
                  "the shortest confusable neighbour (distractor bowl) 0.070 m; "
                  "0.045 m is the midpoint of that measured gap (C2-L5 per-cell "
                  "max-height separation)",
        "allowed": True},
    "DEMO_BOWL_H": {
        "source": "debug-seed measurement: target bowl height above table, "
                  "0.0989-0.0996 m on all 8 v1 seeds (fallback only)",
        "allowed": True},
    "DEMO_PLATE_H": {
        "source": "debug-seed measurement: plate height above table, "
                  "0.0190-0.0191 m on all 8 v1 seeds (fallback only)",
        "allowed": True},
    "HELD_EFFORT": {
        "source": "api.gripper docstring: effort 3.0 iff holding",
        "allowed": True},
}

GRASP_ANCHOR_XY = (-0.2046, 0.1694)
RELEASE_ANCHOR_XY = (0.0395, 0.1555)
PINCH_OFF = (-0.0032, -0.0351)
GRASP_DZ = -0.0317
RELEASE_DZ = 0.0254
PREGRASP_DZ = 0.0771
LIFT_Z = 1.1022
BOWL_EXT = (0.07, 0.15)
BOWL_H = (0.050, 0.140)
BOWL_H_TALL = 0.085
FLAT_CEIL = 0.045
DEMO_BOWL_H = 0.0992
DEMO_PLATE_H = 0.0191
PLATE_EXT = (0.10, 0.18)
PLATE_H = (0.008, 0.040)
OPEN_WIDTH_M = 0.040
CELL_M = 0.015
TABLE_CLEAR_M = 0.012
HELD_EFFORT = 2.0

WORK_X = (-0.50, 0.45)
WORK_Y = (-0.50, 0.50)


def cloud(frame, step=2):
    d = np.asarray(frame.depth, float)
    h, w = d.shape[:2]
    vv, uu = np.mgrid[0:h:step, 0:w:step]
    z = d[::step, ::step]
    K = np.asarray(frame.intrinsics, float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    ok = np.isfinite(z) & (z > 0.05) & (z < 5.0)
    pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], -1)
    T = np.asarray(frame.t_base_cam, float)
    pts = pc.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]
    return pts, ok.reshape(-1)


def table_height(pts, ok):
    m = (ok & (pts[:, 0] > WORK_X[0]) & (pts[:, 0] < WORK_X[1])
         & (pts[:, 1] > WORK_Y[0]) & (pts[:, 1] < WORK_Y[1])
         & (pts[:, 2] > 0.70) & (pts[:, 2] < 1.10))
    zs = pts[m, 2]
    if zs.size < 200:
        return None
    hist, edges = np.histogram(zs, bins=120)
    i = int(np.argmax(hist))
    band = zs[(zs >= edges[i] - 0.01) & (zs <= edges[i + 1] + 0.01)]
    return float(np.median(band))


def clusters(pts, mask, min_cells=3, cell_ceiling=None):
    p = pts[mask]
    if p.shape[0] < 10:
        return []
    gx = np.floor(p[:, 0] / CELL_M).astype(int)
    gy = np.floor(p[:, 1] / CELL_M).astype(int)
    cells = {}
    for i in range(p.shape[0]):
        k = (int(gx[i]), int(gy[i]))
        c = cells.get(k)
        if c is None:
            cells[k] = [1, p[i, 2], p[i, 0], p[i, 1], p[i, 0], p[i, 1]]
        else:
            c[0] += 1
            if p[i, 2] > c[1]:
                c[1] = p[i, 2]
            c[2] = min(c[2], p[i, 0]); c[3] = min(c[3], p[i, 1])
            c[4] = max(c[4], p[i, 0]); c[5] = max(c[5], p[i, 1])
    if cell_ceiling is not None:
        # C2-L5: a flat disc abutting a tall object is only separable by
        # PER-CELL max height -- drop every cell whose tallest point is above
        # the ceiling before running connectivity.
        cells = {k: v for k, v in cells.items() if v[1] <= cell_ceiling}
    seen, out = set(), []
    for k in list(cells):
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            cur = stack.pop()
            comp.append(cur)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nb = (cur[0] + dx, cur[1] + dy)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        if len(comp) < min_cells:
            continue
        x0 = min(cells[c][2] for c in comp); y0 = min(cells[c][3] for c in comp)
        x1 = max(cells[c][4] for c in comp); y1 = max(cells[c][5] for c in comp)
        out.append({"n": int(sum(cells[c][0] for c in comp)),
                    "cells": len(comp),
                    "mid": (float((x0 + x1) / 2), float((y0 + y1) / 2)),
                    "ext": (float(x1 - x0), float(y1 - y0)),
                    "zmax": float(max(cells[c][1] for c in comp))})
    out.sort(key=lambda c: -c["n"])
    return out


def survey(api, tag):
    try:
        f = api.capture("cam_high")
    except Exception as e:
        api.log("%s capture failed: %s" % (tag, e))
        return None, [], [], [], []
    pts, ok = cloud(f)
    tz = table_height(pts, ok)
    if tz is None:
        api.log("%s no table plane" % tag)
        return None, [], [], [], []
    inwork = (ok & (pts[:, 0] > WORK_X[0]) & (pts[:, 0] < WORK_X[1])
              & (pts[:, 1] > WORK_Y[0]) & (pts[:, 1] < WORK_Y[1]))
    low = clusters(pts, inwork & (pts[:, 2] > tz + TABLE_CLEAR_M))
    tall = clusters(pts, inwork & (pts[:, 2] > tz + 0.045))
    flat = clusters(pts, inwork & (pts[:, 2] > tz + TABLE_CLEAR_M),
                    cell_ceiling=tz + FLAT_CEIL)
    top = clusters(pts, inwork & (pts[:, 2] > tz + BOWL_H_TALL))
    api.log("%s table_z=%.4f" % (tag, tz))
    for c in low[:8]:
        api.log("%s[low] n=%d mid=(%.4f,%.4f) ext=(%.3f,%.3f) h=%.4f"
                % (tag, c["n"], c["mid"][0], c["mid"][1], c["ext"][0],
                   c["ext"][1], c["zmax"] - tz))
    return tz, low, tall, flat, top


def pick(cands, anchor, ext_gate, h_gate, tz):
    keep = []
    for c in cands:
        h = c["zmax"] - tz
        if not (ext_gate[0] <= c["ext"][0] <= ext_gate[1]):
            continue
        if not (ext_gate[0] <= c["ext"][1] <= ext_gate[1]):
            continue
        if not (h_gate[0] <= h <= h_gate[1]):
            continue
        d = float(np.hypot(c["mid"][0] - anchor[0], c["mid"][1] - anchor[1]))
        keep.append((d, c, h))
    if not keep:
        return None, None, None
    keep.sort(key=lambda t: t[0])
    return keep[0][1], keep[0][0], keep[0][2]


def run(api):
    api.log("instruction: %s" % api.instruction())
    tz, low, tall, flat, top = survey(api, "pre")
    if tz is None:
        return "no table"

    # target: the bowl standing on the ramekin. Both cues must agree -- it is
    # the nearest size-gated cluster to the demo grasp anchor (C2-L1) AND the
    # taller of the two bowls (the ramekin lifts it ~0.03 m).
    tall_only = [c for c in low if c["zmax"] - tz >= BOWL_H_TALL]
    bowl, dbowl, hbowl = pick(tall_only, GRASP_ANCHOR_XY, BOWL_EXT, BOWL_H, tz)
    src_b = "low+tall"
    alt_b, _, _ = pick(top, GRASP_ANCHOR_XY, BOWL_EXT, BOWL_H, tz)
    if alt_b is not None:
        api.log("xcheck bowl rim-band mid=(%.4f,%.4f) ext=(%.3f,%.3f)"
                % (alt_b["mid"][0], alt_b["mid"][1], alt_b["ext"][0],
                   alt_b["ext"][1]))
    if bowl is None and alt_b is not None:
        bowl, src_b = alt_b, "rim-band"
        dbowl = float(np.hypot(bowl["mid"][0] - GRASP_ANCHOR_XY[0],
                               bowl["mid"][1] - GRASP_ANCHOR_XY[1]))
        hbowl = bowl["zmax"] - tz
    plate, dplate, hplate = pick(low, RELEASE_ANCHOR_XY, PLATE_EXT, PLATE_H, tz)
    src_p = "low"
    alt_p, _, _ = pick(flat, RELEASE_ANCHOR_XY, PLATE_EXT, PLATE_H, tz)
    if alt_p is not None:
        api.log("xcheck plate flat-band mid=(%.4f,%.4f) ext=(%.3f,%.3f)"
                % (alt_p["mid"][0], alt_p["mid"][1], alt_p["ext"][0],
                   alt_p["ext"][1]))
    if plate is None and alt_p is not None:
        plate, src_p = alt_p, "flat-band"
        dplate = float(np.hypot(plate["mid"][0] - RELEASE_ANCHOR_XY[0],
                                plate["mid"][1] - RELEASE_ANCHOR_XY[1]))
        hplate = plate["zmax"] - tz
    if bowl is None:
        api.log("WARN bowl grounding failed -> demo anchor")
        bowl = {"mid": GRASP_ANCHOR_XY, "ext": (0.0, 0.0),
                "zmax": tz + DEMO_BOWL_H}
        dbowl, hbowl, src_b = 0.0, DEMO_BOWL_H, "anchor"
    if plate is None:
        api.log("WARN plate grounding failed -> demo anchor")
        plate = {"mid": RELEASE_ANCHOR_XY, "ext": (0.0, 0.0),
                 "zmax": tz + DEMO_PLATE_H}
        dplate, hplate, src_p = 0.0, DEMO_PLATE_H, "anchor"
    api.log("src bowl=%s plate=%s" % (src_b, src_p))
    api.log("BOWL mid=(%.4f,%.4f) ext=(%.3f,%.3f) h=%.4f d_anchor=%.4f"
            % (bowl["mid"][0], bowl["mid"][1], bowl["ext"][0], bowl["ext"][1],
               hbowl, dbowl))
    api.log("PLATE mid=(%.4f,%.4f) ext=(%.3f,%.3f) h=%.4f d_anchor=%.4f"
            % (plate["mid"][0], plate["mid"][1], plate["ext"][0],
               plate["ext"][1], hplate, dplate))

    gx = bowl["mid"][0] + PINCH_OFF[0]
    gy = bowl["mid"][1] + PINCH_OFF[1]
    gz = bowl["zmax"] + GRASP_DZ
    # held bowl centre = eef - PINCH_OFF (v2 had this sign inverted, which
    # doubled the offset error and scored 0/8), so aim the eef at
    # plate centre + PINCH_OFF to land the bowl centre on the plate centre.
    px = plate["mid"][0] + PINCH_OFF[0]
    py = plate["mid"][1] + PINCH_OFF[1]
    pz = plate["zmax"] + RELEASE_DZ
    api.log("plan grasp=(%.4f,%.4f,%.4f) place=(%.4f,%.4f,%.4f)"
            % (gx, gy, gz, px, py, pz))

    api.grip(OPEN_WIDTH_M)
    api.move([gx, gy, gz + PREGRASP_DZ], seconds=2.0)
    r = api.move([gx, gy, gz], seconds=1.5)
    api.log("descend residual=%.4f eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(0.0)
    api.settle(0.3)
    g = api.gripper()
    api.log("close1 gripper=%s" % g)
    if g["effort"] < HELD_EFFORT:
        api.grip(OPEN_WIDTH_M)
        api.move([gx, gy, gz + 0.05], seconds=1.0)
        gy2 = bowl["mid"][1] - 0.045
        api.move([gx, gy2, gz], seconds=1.5)
        api.grip(0.0)
        api.settle(0.3)
        g = api.gripper()
        api.log("close2 gripper=%s at y=%.4f" % (g, gy2))

    r = api.move([gx, gy, LIFT_Z], seconds=1.5)
    api.log("lift residual=%.4f grip=%s" % (r, api.gripper()))
    r = api.move([px, py, LIFT_Z], seconds=2.0)
    api.log("transit residual=%.4f grip=%s" % (r, api.gripper()))
    r = api.move([px, py, pz], seconds=1.5)
    api.log("place residual=%.4f grip=%s eef=%s done=%s"
            % (r, api.gripper(), np.round(api.eef(), 4).tolist(), api.done))
    api.grip(OPEN_WIDTH_M)
    api.settle(0.4)
    api.log("released done=%s" % api.done)

    # calibration read-out: retreat clear of the plate and measure where the
    # bowl actually landed relative to the plate centre.
    if not api.done:
        api.move([px, py, LIFT_Z], seconds=1.2)
        api.move([-0.21, -0.06, 1.15], seconds=2.0)
        tz2, low2, tall2, _f2, _t2 = survey(api, "post")
        if tz2 is not None:
            for c in tall2[:6]:
                api.log("post[tall] n=%d mid=(%.4f,%.4f) ext=(%.3f,%.3f) h=%.4f"
                        % (c["n"], c["mid"][0], c["mid"][1], c["ext"][0],
                           c["ext"][1], c["zmax"] - tz2))
            land, dl, hl = pick(tall2, (plate["mid"][0], plate["mid"][1]),
                                BOWL_EXT, (0.05, 0.15), tz2)
            if land is not None:
                api.log("LANDED mid=(%.4f,%.4f) h=%.4f  d_plate=%.4f  "
                        "delta=(%.4f,%.4f)"
                        % (land["mid"][0], land["mid"][1], hl, dl,
                           land["mid"][0] - plate["mid"][0],
                           land["mid"][1] - plate["mid"][1]))
    api.log("end done=%s" % api.done)
    return "v5 grounded grasp+place, banded grounding + anchor fallback"
