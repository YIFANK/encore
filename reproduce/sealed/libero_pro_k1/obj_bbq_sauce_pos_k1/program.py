"""v10 -- pick the BBQ bottle (not the pack demo's ketchup) and drop it in the basket.

Identity: label crops streamed out of cam_high on debug seed 51 read
"Tomato Ketchup" (silver cap, top 0.148), "Ranch Dressing" (green cap),
a soup can (top 0.081) and "BBQ" (dark red, cap and body the same red,
top 0.113). Only the BBQ bottle has a strongly red-dominant cap, so the
target is argmax(capR - capG) over bottle-height clusters.
"""
import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug-seed v1/v2 cam_high depth mode (0.0033)", "allowed": True},
    "ZCUT": {"source": "debug-seed v2: table objects top out at 0.148, arm above 0.20", "allowed": True},
    "RED_CAP_RULE": {"source": "debug-seed v9 label crops: BBQ cap rgb (68,23,6) vs ketchup (99,98,98), ranch (28,69,43), soup can (81,83,92)", "allowed": True},
    "BOTTLE_GATE": {"source": "debug-seed v9 measured tops: bbq 0.113, ketchup 0.148, ranch 0.147, can 0.081, flat boxes 0.02-0.03", "allowed": True},
    "PALM_CLEARANCE": {"source": "debug-seed v7: descent at a bottle jams at eef z = top - 0.035 while bare table reaches 0.059; v8 grasp at top-0.029 held (gap 0.033, effort 3.0)", "allowed": True},
    "WRIST_Z": {"source": "debug-seed: clear of the 0.148-tall objects", "allowed": True},
    "RELEASE_ABOVE_RIM": {"source": "pack demo0 t=117 release ee z=0.1736 vs measured basket rim 0.141", "allowed": True},
    "HELD_BAND": {"source": "pack demo0 t=117 carried gap 0.0372; debug-seed air closes give 0.001; fully open is 0.079", "allowed": True},
}

TABLE_Z = 0.0033
ZCUT = 0.20
WRIST_Z = 0.28
RELEASE_ABOVE_RIM = 0.033
HELD_LO, HELD_HI = 0.012, 0.060
HEIGHT_LADDER = [0.029, 0.017, 0.041]


def cloud(fr):
    d = np.asarray(fr.depth, float)
    K = np.asarray(fr.intrinsics, float)
    T = np.asarray(fr.t_base_cam, float)
    h, w = d.shape[:2]
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    ok = np.isfinite(d) & (d > 0)
    z = np.where(ok, d, 1.0)
    x = (uu - K[0, 2]) * z / K[0, 0]
    y = (vv - K[1, 2]) * z / K[1, 1]
    return (np.stack([x, y, z, np.ones_like(z)], -1) @ T.T)[..., :3], ok


def clusters_of(mask, X, Y, g=0.012, minpx=40):
    cells = {}
    ii, jj = np.nonzero(mask)
    gi = np.floor(X / g).astype(int)
    gj = np.floor(Y / g).astype(int)
    for a, b in zip(ii, jj):
        cells.setdefault((gi[a, b], gj[a, b]), []).append((a, b))
    seen, out = set(), []
    for c in cells:
        if c in seen:
            continue
        st, comp = [c], []
        seen.add(c)
        while st:
            cur = st.pop()
            comp.append(cur)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (cur[0] + dx, cur[1] + dy)
                    if n in cells and n not in seen:
                        seen.add(n)
                        st.append(n)
        px = [p for k in comp for p in cells[k]]
        if len(px) >= minpx:
            out.append((np.array([p[0] for p in px]), np.array([p[1] for p in px])))
    out.sort(key=lambda t: -len(t[0]))
    return out


def analyse(fr, minpx=40, zhi=ZCUT):
    rgb = np.asarray(fr.rgb)
    P, ok = cloud(fr)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    box = ok & (np.abs(X) < 0.40) & (np.abs(Y) < 0.45)
    m = box & (Z > TABLE_Z + 0.012) & (Z < zhi)
    out = []
    for a, b in clusters_of(m, X, Y, minpx=minpx):
        zz, xx, yy = Z[a, b], X[a, b], Y[a, b]
        top = float(np.percentile(zz, 98))
        col = rgb[a, b].astype(float)
        t = zz > top - 0.020
        if t.sum() < 12:
            t = zz > top - 0.035
        out.append({
            "n": len(a), "top": top,
            "r": 0.5 * float(np.percentile(yy[t], 96) - np.percentile(yy[t], 4)),
            "cap": np.array([float(np.median(xx[t])), float(np.median(yy[t]))]),
            "capbox": (float(np.percentile(xx[t], 3)), float(np.percentile(xx[t], 97)),
                       float(np.percentile(yy[t], 3)), float(np.percentile(yy[t], 97))),
            "capcol": col[t].mean(0), "col": col.mean(0)})
    return out


def split(cl):
    bas = [c for c in cl if c["r"] > 0.05 and c["n"] > 4000]
    bots = [c for c in cl if 0.07 < c["top"] < 0.19 and c["n"] > 500 and c["r"] < 0.05]
    for c in bots:
        cc, bc = c["capcol"], c["col"]
        # the BBQ bottle is the only red-dominant one, cap and body alike
        c["red"] = float((cc[0] - cc[1]) + (bc[0] - bc[1]))
    bots.sort(key=lambda c: -c["red"])
    return (bots[0] if bots else None), (bas[0] if bas else None), bots


def run(api):
    tgt, bas, bots = split(analyse(api.capture("cam_high")))
    for c in bots:
        api.log("cand top=%.3f n=%d red=%.1f cap=(%d,%d,%d) body=(%d,%d,%d) xy=(%.3f,%.3f)" % (
            c["top"], c["n"], c["red"], *c["capcol"], *c["col"], c["cap"][0], c["cap"][1]))
    if tgt is None or bas is None:
        api.log("PERCEPTION FAIL")
        return "no target"
    cx, cy, top = float(tgt["cap"][0]), float(tgt["cap"][1]), float(tgt["top"])
    bx, by = float(bas["cap"][0]), float(bas["cap"][1])
    rz = float(bas["top"]) + RELEASE_ABOVE_RIM
    api.log("TARGET (%.3f,%.3f) top=%.3f | basket (%.3f,%.3f) rim=%.3f" % (
        cx, cy, top, bx, by, bas["top"]))

    api.grip(0.08)
    api.move([cx - 0.020, cy, WRIST_Z], seconds=2.5)
    best, bd = None, 9.9
    for c in analyse(api.capture("cam_arm_wrist"), minpx=60, zhi=0.19):
        d = float(np.hypot(c["cap"][0] - cx, c["cap"][1] - cy))
        if c["top"] > 0.06 and d < bd:
            best, bd = c, d
    if best is not None and bd < 0.06:
        gx = 0.5 * (best["capbox"][0] + best["capbox"][1])
        gy = 0.5 * (best["capbox"][2] + best["capbox"][3])
        top = float(best["top"])
    else:
        gx, gy = cx, cy
    api.log("aim=(%.3f,%.3f) top=%.3f refine=%s d=%.3f" % (gx, gy, top, best is not None, bd))

    ok = False
    for i, dz in enumerate(HEIGHT_LADDER):
        gz = top - dz
        api.move([gx, gy, top + 0.075], seconds=1.5)
        res = api.move([gx, gy, gz], seconds=1.5)
        api.grip(0.0)
        api.settle(0.2)
        g = api.gripper()
        ok = g["effort"] > 2.5 and HELD_LO < g["width_m"] < HELD_HI
        api.log("rung%d gz=%.3f res=%.4f eef=%s grip=%s held=%s" % (
            i, gz, res, np.round(api.eef(), 3).tolist(), g, ok))
        if ok:
            api.move([gx, gy, 0.25], seconds=2.0)
            g = api.gripper()
            ok = g["effort"] > 2.5 and HELD_LO < g["width_m"] < HELD_HI
            api.log("lift grip=%s held=%s" % (g, ok))
            if ok:
                break
        api.grip(0.08)
    if not ok:
        api.log("GRASP FAILED")
        return "no grasp"

    api.move([bx, by, 0.29], seconds=3.0)
    api.log("over basket grip=%s eef=%s" % (api.gripper(), np.round(api.eef(), 3).tolist()))
    api.move([bx, by, rz], seconds=2.0)
    api.log("at release eef=%s grip=%s" % (np.round(api.eef(), 3).tolist(), api.gripper()))
    api.grip(0.08)
    api.settle(0.6)
    api.move([bx, by, 0.29], seconds=1.5)
    api.settle(0.4)
    api.log("end eef=%s grip=%s" % (np.round(api.eef(), 3).tolist(), api.gripper()))
    return "placed"
