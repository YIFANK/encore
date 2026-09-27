"""c2k1clean / obj_bbq_sauce_task_k1 -- v2: identify + pick + place.

Intent: "Pick the ketchup and place it in the basket".

Where every number comes from:
  * the KETCHUP appearance comes from the MATE pack keyframe (its demos handle
    the ketchup) and is cross-checked against the same-looking prop in the K1
    pack keyframe;
  * the BASKET release geometry comes from the K1 pack keyframes / ee_path
    (its demos place into the basket);
  * table height, prop heights and prop footprints are measured live from
    cam_high RGB-D on debug seeds.
"""
import numpy as np

PROVENANCE = {
    "KETCHUP_BODY_RGB": {
        "source": "packs/c2k1clean_obj_bbq_sauce_task_mate/keyframes/demo0_t0000.png -- mean RGB of "
                  "the body patch of the prop those demos pick up (px x29-37, "
                  "y55-63) = (73.6,48.0,32.6); the same-looking prop in "
                  "packs/c2k1clean_obj_bbq_sauce_task_k1/keyframes/demo0_t0000.png measures "
                  "(71.4,48.9,34.3). Averaged.",
        "allowed": True},
    "KETCHUP_CAP_RGB": {
        "source": "same two keyframes, cap patch above the body "
                  "(mate px x30-36 y49-53 = (93,89,85); k1 px x70-75 y48-52 = "
                  "(74,71,68)). Averaged; used only as a second cue.",
        "allowed": True},
    "TABLE_BAND": {"source": "generic depth mechanics: a prop pixel is one whose "
                             "deprojected z clears the fitted table plane by "
                             "more than the depth noise floor (debug seed 51: "
                             "table plane z=0.0011 with 33.6k inliers)",
                   "allowed": True},
    "ARM_H": {"source": "debug seeds 51/53/55/57: the robot-arm cluster measures "
                        "h=0.4455 above the table, every table prop h<=0.147",
              "allowed": True},
    "WS_X": {"source": "debug-seed measurement: all props fall in x[-0.20,0.18], "
                       "y[-0.21,0.27] (v0 probe cluster table)", "allowed": True},
    "WS_Y": {"source": "same", "allowed": True},
    "SAFE_Z": {"source": "debug seed 51: home eef z=0.2613 and the tallest prop "
                         "top is 0.1477, so 0.30 clears every prop",
               "allowed": True},
    "OPEN_W": {"source": "debug seed 51 api.gripper() at reset: width_m=0.0778",
               "allowed": True},
    "HOLD_EFFORT": {"source": "FairApi contract: effort 3.0 iff holding",
                    "allowed": True},
    "BASKET_AIM": {
        "source": "debug seeds 51-57: the basket cluster's top-1.5cm band is "
                  "its rim ring, so the midpoint of that band's x and y range "
                  "is the opening centre; the plain median of the band is "
                  "biased to the far rim (0.018 vs a 0.074 body centroid on "
                  "seed 51) because the camera looks down-forward",
        "allowed": True},
    "REL_DZ": {"source": "packs/c2k1clean_obj_bbq_sauce_task_k1 ee_path6: the demo releases "
                         "(gripper_cmd -1) at ee z=0.1736 while the basket top "
                         "measures 0.1415 on debug seeds -> +0.032",
               "allowed": True},
    "GRASP_FRAC": {"source": "packs/c2k1clean_obj_bbq_sauce_task_k1 keyframe t=54: the demo "
                             "closes on its prop at ee z=0.0732 with a prop of "
                             "measured height 0.112 standing on a table at "
                             "z=0.0011, i.e. 0.039 below the prop top",
                   "allowed": True},
}

KETCHUP_BODY_RGB = np.array([72.5, 48.5, 33.5])
KETCHUP_CAP_RGB = np.array([83.5, 80.0, 76.5])
TABLE_BAND = 0.015
ARM_H = 0.30
WS_X = (-0.40, 0.45)
WS_Y = (-0.45, 0.45)
SAFE_Z = 0.30
OPEN_W = 0.078
HOLD_EFFORT = 1.5
REL_DZ = 0.032
GRASP_DROP = 0.039          # metres below the prop top (K1 demo keyframe t=54)


# --------------------------------------------------------------------- vision
def _label(mask):
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    ys, xs = np.nonzero(mask)
    for y0, x0 in zip(ys, xs):
        if lab[y0, x0]:
            continue
        cur += 1
        stack = [(y0, x0)]
        lab[y0, x0] = cur
        while stack:
            y, x = stack.pop()
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not lab[ny, nx]:
                    lab[ny, nx] = cur
                    stack.append((ny, nx))
    return lab, cur


def _scene(api, step=2):
    fr = api.capture("cam_high")
    rgb = np.asarray(fr.rgb)
    d = np.asarray(fr.depth, float)
    h, w = d.shape
    vs, us = np.arange(0, h, step), np.arange(0, w, step)
    uu, vv = np.meshgrid(us, vs)
    z = d[vv, uu]
    K = np.asarray(fr.intrinsics, float)
    pc = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1],
                   z, np.ones_like(z)], -1)
    W = (pc @ np.asarray(fr.t_base_cam, float).T)[..., :3]
    X, Y, Z = W[..., 0], W[..., 1], W[..., 2]
    good = np.isfinite(Z) & (z > 0)
    inws = good & (X > WS_X[0]) & (X < WS_X[1]) & (Y > WS_Y[0]) & (Y < WS_Y[1])
    zin = Z[inws]
    hist, edges = np.histogram(zin, bins=60)
    i0 = int(np.argmax(hist))
    near = zin[np.abs(zin - 0.5 * (edges[i0] + edges[i0 + 1])) < 0.02]
    table = float(np.median(near))

    prop = inws & (Z > table + TABLE_BAND) & (Z < table + 0.60)
    lab, n = _label(prop)
    out = []
    for k in range(1, n + 1):
        m = lab == k
        if int(m.sum()) < 40:
            continue
        us_, vs_ = uu[m], vv[m]
        wx, wy, wz = X[m], Y[m], Z[m]
        col = rgb[vs_, us_].astype(float)
        ztop = float(np.percentile(wz, 98))
        hh = ztop - table
        if hh > ARM_H:                      # the robot arm, not a prop
            continue
        lo, hi = table + 0.20 * hh, table + 0.70 * hh
        body = (wz > lo) & (wz < hi)
        capm = wz > ztop - max(0.012, 0.15 * hh)
        topm = wz > ztop - 0.015
        out.append(dict(
            n=int(m.sum()), px=(float(us_.mean()), float(vs_.mean())),
            ztop=ztop, h=hh,
            body=col[body].mean(0) if body.sum() > 8 else col.mean(0),
            cap=col[capm].mean(0) if capm.sum() > 5 else col.mean(0),
            xy_all=(float(np.median(wx)), float(np.median(wy))),
            xy_top=(float(np.median(wx[topm])), float(np.median(wy[topm]))
                    ) if topm.sum() > 5 else (float(np.median(wx)),
                                              float(np.median(wy))),
            xy_rim=(float(0.5 * (wx[topm].min() + wx[topm].max())),
                    float(0.5 * (wy[topm].min() + wy[topm].max()))
                    ) if topm.sum() > 5 else (float(np.median(wx)),
                                              float(np.median(wy))),
            xy_mid=(float(0.5 * (wx.min() + wx.max())),
                    float(0.5 * (wy.min() + wy.max()))),
            yext=float(wy.max() - wy.min()), xext=float(wx.max() - wx.min()),
            wx=wx, wy=wy, wz=wz))
    return fr, table, out


def _profile(api, c, table):
    """Log the y-width of a cluster per 1 cm height band (the neck finder)."""
    z0 = table
    while z0 < c["ztop"]:
        m = (c["wz"] >= z0) & (c["wz"] < z0 + 0.01)
        if m.sum() > 4:
            api.log("  band z=%.3f n=%3d ywid=%.4f ymid=%.4f xmed=%.4f" % (
                z0, int(m.sum()), float(c["wy"][m].max() - c["wy"][m].min()),
                float(0.5 * (c["wy"][m].max() + c["wy"][m].min())),
                float(np.median(c["wx"][m]))))
        z0 += 0.01


# ------------------------------------------------------------------- behaviour
def run(api):
    api.log("v2 | %r" % api.instruction())
    fr, table, cl = _scene(api)
    api.log("table_z=%.4f props=%d" % (table, len(cl)))

    # rank every prop by appearance distance to the ketchup the mate pack picks
    for c in cl:
        c["dbody"] = float(np.linalg.norm(c["body"] - KETCHUP_BODY_RGB))
        c["dcap"] = float(np.linalg.norm(c["cap"] - KETCHUP_CAP_RGB))
        c["score"] = c["dbody"] + 0.5 * c["dcap"]
    for c in sorted(cl, key=lambda r: r["score"]):
        api.log("P n=%d px=(%.0f,%.0f) h=%.3f xy_all=(%.3f,%.3f) "
                "xy_top=(%.3f,%.3f) yext=%.3f xext=%.3f body=%s cap=%s "
                "db=%.1f dc=%.1f s=%.1f" % (
                    c["n"], c["px"][0], c["px"][1], c["h"], c["xy_all"][0],
                    c["xy_all"][1], c["xy_top"][0], c["xy_top"][1], c["yext"],
                    c["xext"], c["body"].round(1).tolist(),
                    c["cap"].round(1).tolist(), c["dbody"], c["dcap"],
                    c["score"]))

    # the basket: by far the widest, brightest, tallest-footprint prop
    basket = max(cl, key=lambda c: (c["yext"], c["n"]))
    ket = min([c for c in cl if c is not basket], key=lambda c: c["score"])
    api.log("BASKET top=(%.3f,%.3f) rim=(%.3f,%.3f) mid=(%.3f,%.3f) all=(%.3f,%.3f)"
            % (basket["xy_top"][0], basket["xy_top"][1], basket["xy_rim"][0],
               basket["xy_rim"][1], basket["xy_mid"][0], basket["xy_mid"][1],
               basket["xy_all"][0], basket["xy_all"][1]))
    api.log("BASKET ztop=%.3f | KETCHUP xy_top=(%.3f,%.3f) "
            "ztop=%.3f h=%.3f yext=%.3f" % (
                basket["ztop"],
                ket["xy_top"][0], ket["xy_top"][1], ket["ztop"], ket["h"],
                ket["yext"]))
    api.log("ketchup height profile:")
    _profile(api, ket, table)

    kx, ky = ket["xy_top"]
    ztop = ket["ztop"]
    api.grip(OPEN_W)
    api.move([kx, ky, SAFE_Z], seconds=1.5)
    api.log("hover residual eef=%s" % np.asarray(api.eef()).round(4).tolist())

    held = False
    for i, drop in enumerate((GRASP_DROP, GRASP_DROP + 0.025, 0.018)):
        gz = max(table + 0.012, ztop - drop)
        api.grip(OPEN_W)
        api.move([kx, ky, min(SAFE_Z, gz + 0.06)], seconds=1.0)
        r = api.move([kx, ky, gz], seconds=1.2)
        api.log("try%d gz=%.4f res=%.4f eef=%s" % (
            i, gz, r, np.asarray(api.eef()).round(4).tolist()))
        api.grip(0.0)
        api.settle(0.3)
        api.move([kx, ky, SAFE_Z], seconds=1.2)
        g = api.gripper()
        api.log("try%d after-lift grip=%s" % (i, g))
        if g["effort"] >= HOLD_EFFORT and g["width_m"] > 0.008:
            held = True
            hang = gz - table
            break
    if not held:
        api.log("no grasp -> stop")
        return "no grasp"

    bx, by = basket["xy_rim"]
    rz = basket["ztop"] + REL_DZ
    rz = max(rz, basket["ztop"] - 0.105 + hang)
    api.log("transport to (%.3f,%.3f) rz=%.3f hang=%.3f" % (bx, by, rz, hang))
    api.move([bx, by, SAFE_Z + 0.02], seconds=2.0)
    api.log("over-basket grip=%s eef=%s" % (
        api.gripper(), np.asarray(api.eef()).round(4).tolist()))
    api.move([bx, by, rz], seconds=1.5)
    api.grip(OPEN_W)
    api.settle(0.5)
    api.move([bx, by, SAFE_Z + 0.02], seconds=1.2)
    api.log("released; final grip=%s" % api.gripper())
    return "v2 place attempted"
