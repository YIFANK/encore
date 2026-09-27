"""Open the middle drawer of the cabinet.

Mechanism, as measured on the debug seeds of this cell:

  The cabinet is the grey box whose vertical face carries three horizontal
  handle bars standing ~31 mm proud of it, 69 mm apart.  The middle bar is the
  target.  A wrist-camera frame taken at the face (0.3 mm/px) measures the
  face at y = 0.1367, the middle bar as a 17.5 mm cylinder at y 0.1047..0.122,
  z 1.005..1.0225 -- i.e. ~9 mm below where the oblique overhead view puts the
  protrusion band, which is the BAND_Z_BIAS correction below.

  The gripper's pads separate along the TOOL Y axis (measured: the near-field
  silhouette in a wrist frame widens along tool Y when the gripper opens).  So
  the tool frame is built from the approach axis and that jaw axis directly,
  with the jaws vertical: opened to 78 mm and centred on the bar they sit at
  0.974 and 1.053, inside the clear lanes below and above it, so the bar can
  pass between them.  Then close on the bar and pull straight out.

  Route: the arm cannot hold this wrist pose and walk to the face in one move
  (the saturated OSC command fights the rotation and the arm diverges), so it
  drops down a corridor clear of both the rack and the cabinet with the wrist
  still straight down, turns there, settles as low as it can, and creeps in in
  20 mm steps.  Small steps matter: a move exits at POS_TOL, so a step the arm
  can converge on costs ~5 sim steps while an unreachable one burns its whole
  60-step cap, and the episode horizon is 500.

See NOTES.md: on every debug seed this cell's cabinet sits ~0.15 m in front of
the robot base, and the reachable set there stops about a centimetre short of
the middle bar in every wrist pose tried.
"""
import numpy as np

# --- perception (inlined from perception.py) ---
"""Cabinet / drawer-handle perception from one cam_high RGB-D frame.

Everything here is derived from the debug-seed frames of this cell only:
the scene has one grey box fixture whose vertical face carries three
horizontal handle bars that stand proud of the face.  The middle bar (median
height of the three) is the target of "open the middle drawer".
"""

GRID = 0.02          # footprint cell, m
CHROMA_MAX = 45      # grey-ish only: the arm renders saturated, fixtures do not
PROUD_MIN = 0.012    # a handle stands at least this far off its face
PROUD_MAX = 0.070


def cloud(rgb, depth, K, T):
    d = np.asarray(depth, float)
    K = np.asarray(K, float)
    h, w = d.shape
    vv, uu = np.mgrid[0:h, 0:w]
    cam = np.stack([(uu - K[0, 2]) * d / K[0, 0],
                    (vv - K[1, 2]) * d / K[1, 1], d], -1)
    T = np.asarray(T, float)
    return cam @ T[:3, :3].T + T[:3, 3]


def _components(cells):
    """4-connected components of a set of (i,j) grid cells."""
    todo = set(cells)
    out = []
    while todo:
        seed = todo.pop()
        comp = [seed]
        stack = [seed]
        while stack:
            i, j = stack.pop()
            for n in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if n in todo:
                    todo.discard(n)
                    comp.append(n)
                    stack.append(n)
        out.append(comp)
    return out


def _bands(Q, d_out, z_lo, z_hi):
    """Protruding bands on the face of Q looking along outward unit d_out."""
    s = Q @ np.asarray(d_out, float)
    zs = Q[:, 2]
    edges = np.arange(z_lo, z_hi, 0.01)
    smax, zc, n = [], [], []
    for e in edges:
        b = (zs >= e) & (zs < e + 0.01)
        if b.sum() < 25:
            smax.append(np.nan); zc.append(e + 0.005); n.append(int(b.sum())); continue
        smax.append(float(np.percentile(s[b], 97)))
        zc.append(e + 0.005); n.append(int(b.sum()))
    smax = np.array(smax); zc = np.array(zc)
    good = np.isfinite(smax)
    if good.sum() < 6:
        return None
    s_face = float(np.median(smax[good]))
    proud = smax - s_face
    hit = good & (proud > PROUD_MIN) & (proud < PROUD_MAX)
    bands, cur = [], []
    for i, hv in enumerate(hit):
        if hv:
            cur.append(i)
        elif cur:
            bands.append(cur); cur = []
    if cur:
        bands.append(cur)
    bands = [b for b in bands if len(b) >= 1]
    return {"s_face": s_face, "zc": zc, "smax": smax, "proud": proud,
            "bands": [(float(zc[b[0]] - 0.005), float(zc[b[-1]] + 0.005),
                       float(np.nanmax(proud[b]))) for b in bands]}


def perceive(rgb, depth, K, T, log=print):
    P = cloud(rgb, depth, K, T)
    rgbf = np.asarray(rgb, float)
    chroma = rgbf.max(-1) - rgbf.min(-1)
    ok = np.isfinite(depth) & (np.asarray(depth) > 0) & (chroma < CHROMA_MAX)
    ok &= (P[..., 0] > -0.62) & (P[..., 0] < 0.55) & (np.abs(P[..., 1]) < 0.75)
    pts = P[ok]
    hist, edges = np.histogram(pts[:, 2], bins=np.arange(0.80, 1.35, 0.005))
    z_table = float(edges[hist.argmax()] + 0.0025)
    log(f"PER z_table={z_table:.4f} npts={len(pts)}")

    above = pts[pts[:, 2] > z_table + 0.02]
    akey = np.round(above[:, :2] / GRID).astype(int)
    tallm = above[:, 2] > z_table + 0.07
    cell = {}
    for k, p in zip(map(tuple, akey[tallm]), above[tallm]):
        cell.setdefault(k, []).append(p)
    allcell = {}
    for k, p in zip(map(tuple, akey), above):
        allcell.setdefault(k, []).append(p)
    solid = {k for k, v in cell.items() if len(v) >= 10}
    best = None
    for comp in _components(solid):
        if len(comp) < 8:
            continue
        Q = np.array([p for k in comp for p in allcell[k]], dtype=float)
        z_top = float(np.percentile(Q[:, 2], 97))
        if not (0.15 < z_top - z_table < 0.40):
            continue
        for d_out in ((0, -1, 0), (0, 1, 0), (1, 0, 0), (-1, 0, 0)):
            r = _bands(Q, d_out, z_table + 0.02, z_top - 0.015)
            if r is None:
                continue
            nb = len(r["bands"])
            score = sum(b[2] for b in r["bands"])
            if nb == 3:
                zc3 = [0.5 * (b[0] + b[1]) for b in r["bands"]]
                d1, d2 = zc3[1] - zc3[0], zc3[2] - zc3[1]
                score += 100 if abs(d1 - d2) < 0.025 and 0.04 < d1 < 0.12 else 20
            elif nb == 2:
                score += 10
            log(f"PER cand cells={len(comp)} ztop={z_top:.3f} d={d_out} "
                f"nb={nb} bands={[(round(a,3),round(b,3),round(c,3)) for a,b,c in r['bands']]} "
                f"s_face={r['s_face']:.3f} score={score:.2f}")
            if best is None or score > best["score"]:
                best = {"score": score, "Q": Q, "d_out": np.array(d_out, float),
                        "z_top": z_top, "z_table": z_table, **r}
    if best is None:
        return None
    bands = best["bands"]
    if len(bands) == 3:
        band = bands[1]
    elif len(bands) == 2:
        band = None
    else:
        band = bands[len(bands) // 2]
    if band is None:
        return {"fail": "two-band", **{k: best[k] for k in ("z_top", "z_table")},
                "d_out": best["d_out"], "s_face": best["s_face"], "bands": bands}
    z0, z1, proud = band
    Q = best["Q"]
    s = Q @ best["d_out"]
    sel = (Q[:, 2] >= z0 - 0.004) & (Q[:, 2] <= z1 + 0.004) & (s > best["s_face"] + PROUD_MIN * 0.6)
    H = Q[sel]
    # lateral axis = the horizontal axis perpendicular to d_out
    lat = np.array([best["d_out"][1], -best["d_out"][0], 0.0])
    t = H @ lat
    res = {"z_table": z_table, "z_top": best["z_top"], "d_out": best["d_out"],
           "s_face": best["s_face"], "bands": bands,
           "n_handle": int(sel.sum()),
           "s_front": float(np.percentile(s[sel], 95)),
           "t_mid": float(0.5 * (np.percentile(t, 3) + np.percentile(t, 97))),
           "t_lo": float(np.percentile(t, 3)), "t_hi": float(np.percentile(t, 97)),
           "z_bar": float(0.5 * (z0 + z1)), "lat": lat}
    res["center"] = (res["s_front"] * best["d_out"] + res["t_mid"] * lat
                     + np.array([0, 0, res["z_bar"]]))
    log(f"PER handle z={res['z_bar']:.4f} s_front={res['s_front']:.4f} "
        f"t=[{res['t_lo']:.3f},{res['t_hi']:.3f}] mid={res['t_mid']:.3f} n={res['n_handle']}")
    return res

# --- end perception ---

BAND_Z_BIAS = 0.009      # overhead protrusion band centre - true bar centre
JAW_HALF = 0.039         # pad offset from the eef; api.gripper() opens to 0.078
CORRIDOR = 0.16          # standoff for the descent corridor, m in front of the bar
SETTLE = 0.10            # standoff while the arm settles to its lowest z
STEP_IN = 0.02           # creep increment
FACE_MARGIN = 0.002      # stop the creep this far short of the face plane
PULL_OUT = 0.26          # total outward stroke
PULL_DOWN = 0.02         # downward bias on the stroke, as the pack demo holds
CLAMP_MIN = 0.008        # a closed gap in this band means the bar is in the jaws
CLAMP_MAX = 0.030

PROVENANCE = {
    "BAND_Z_BIAS": {"source": "debug seed 51: cam_arm_wrist depth at the drawer "
                              "face puts the middle bar at z 1.005-1.0225, the "
                              "cam_high protrusion band at 1.0125-1.0325",
                    "allowed": True},
    "JAW_HALF": {"source": "api.gripper() reports width_m 0.078 when open; half "
                           "of it is each pad's offset from the eef",
                 "allowed": True},
    "CORRIDOR": {"source": "debug-seed reach probes: a descent at this standoff "
                           "clears the rack (which stops the arm at z=1.13 "
                           "further out) and the cabinet",
                 "allowed": True},
    "SETTLE": {"source": "debug-seed reach probes: standoff at which the wrist "
                         "settles to its lowest z in front of the face",
               "allowed": True},
    "STEP_IN": {"source": "debug-seed step-cost measurement: 0.02 m targets "
                          "converge inside POS_TOL in a few sim steps, larger "
                          "ones burn the 60-step move cap",
                "allowed": True},
    "FACE_MARGIN": {"source": "cam_high face plane, debug seeds", "allowed": True},
    "PULL_OUT": {"source": "pack demo ee_path6: the demo's stroke is 0.215 m",
                 "allowed": True},
    "PULL_DOWN": {"source": "pack demo raw actions: z command held at -0.5..-0.9 "
                            "throughout the pull", "allowed": True},
    "CLAMP_MIN": {"source": "wrist-camera bar diameter 0.0175 m, debug seed 51",
                  "allowed": True},
    "CLAMP_MAX": {"source": "wrist-camera bar diameter 0.0175 m, debug seed 51",
                  "allowed": True},
    "GRID": {"source": "cam_high point-cloud footprint cell, debug seeds",
             "allowed": True},
    "CHROMA_MAX": {"source": "debug-seed cam_high: fixtures render grey "
                             "(max-min < 30), the arm renders saturated",
                   "allowed": True},
    "PROUD_MIN": {"source": "debug-seed cam_high: handle bars stand 0.029-0.031 "
                            "off the face", "allowed": True},
    "PROUD_MAX": {"source": "debug-seed cam_high: handle bars stand 0.029-0.031 "
                            "off the face", "allowed": True},
}


def mk_zy(zt, yt):
    """Tool frame from the approach axis and the finger-separation axis."""
    zt = np.asarray(zt, float); zt /= np.linalg.norm(zt)
    yt = np.asarray(yt, float); yt -= zt * float(yt @ zt); yt /= np.linalg.norm(yt)
    return np.column_stack([np.cross(yt, zt), yt, zt])


def run(api):
    L = api.log
    f = api.capture("cam_high")
    per = perceive(f.rgb, np.asarray(f.depth, float), f.intrinsics, f.t_base_cam, log=L)
    if per is None or "center" not in per:
        L(f"PERCEPTION FAILED {per}")
        return "no cabinet found"
    d_out = per["d_out"]; lat = per["lat"]
    s_front = per["s_front"]; t_mid = per["t_mid"]; s_face = per["s_face"]
    z_bar = per["z_bar"] - BAND_Z_BIAS
    L(f"BANDS {per['bands']} z_bar={z_bar:.4f} s_face={s_face:.4f} s_front={s_front:.4f}")
    R0 = np.asarray(api.tool_rotation(), float)
    Rv = mk_zy(-d_out, [0, 0, -1])

    def pt(sv, z):
        return sv * d_out + t_mid * lat + np.array([0.0, 0.0, z])

    def step(tag, target, R, seconds=0.6):
        r = api.move(target, rotation=R, seconds=seconds)
        e = np.asarray(api.eef())
        L(f"W {tag} tgt={np.asarray(target).round(4).tolist()} resid={r:.4f} "
          f"eef={e.round(4).tolist()} s={float(e @ d_out):.4f} z={e[2]:.4f}")
        return e

    api.grip(0.08)
    step("transit", pt(s_front + CORRIDOR, z_bar + 0.045), R0, 1.0)
    step("turn", np.asarray(api.eef()), Rv, 1.0)
    step("settle", pt(s_front + SETTLE, z_bar), Rv, 0.8)
    step("settle2", pt(s_front + SETTLE, z_bar), Rv, 0.8)

    s_goal = s_face - FACE_MARGIN
    for i in range(8):
        prev = np.asarray(api.eef())
        s_now = float(prev @ d_out)
        if s_now - s_goal < 0.004:
            break
        e = step(f"in{i}", pt(max(s_now - STEP_IN, s_goal), z_bar), Rv, 0.5)
        adv = float((e - prev) @ (-d_out))
        L(f"ADV {i} {adv:.4f} z={e[2]:.4f} pads={e[2] - JAW_HALF:.4f}/{e[2] + JAW_HALF:.4f}")
        if adv < 0.003:
            L("IN-STALL")
            break
    e = np.asarray(api.eef())
    reached = float(e @ d_out)
    L(f"AT-BAR s={reached:.4f} front={s_front:.4f} face={s_face:.4f} z={e[2]:.4f} "
      f"pad_lo={e[2] - JAW_HALF:.4f} bar={z_bar - 0.0088:.4f}..{z_bar + 0.0088:.4f} "
      f"short_of_bar={reached - s_front:.4f}")

    api.grip(0.0)
    g = api.gripper()
    clamped = CLAMP_MIN < float(g["width_m"]) < CLAMP_MAX
    L(f"GRIP closed {g} clamped={clamped}")
    if not clamped:
        # Nothing between the pads: the pack demo pulls with its gripper open
        # the whole way, so fall back to that -- an open pad resting on the bar
        # still drags it if the stroke keeps pressing down.
        api.grip(0.08)
        L(f"GRIP reopened {api.gripper()}")

    # A clamped bar needs no press: the demo's downward command is what keeps
    # an OPEN finger hooked, and with the jaws shut it only drives the wrist
    # into its own floor and burns the stroke.  Pull straight out in stages.
    dz = 0.0 if clamped else -PULL_DOWN
    e0 = np.asarray(api.eef())
    prev = e0
    for k in range(3):
        prev = step(f"pull{k}", pt(float(prev @ d_out) + PULL_OUT / 3.0,
                                   float(prev[2]) + dz), Rv, 1.0)
        L(f"PULL{k} s={float(prev @ d_out):.4f} out={float((prev - e0) @ d_out):.4f} "
          f"grip={api.gripper()}")
    e2 = prev
    L(f"PULLED {float((e2 - e0) @ d_out):.4f} grip={api.gripper()}")

    f2 = api.capture("cam_high")
    p2 = perceive(f2.rgb, np.asarray(f2.depth, float), f2.intrinsics, f2.t_base_cam,
                  log=lambda m: L("POST " + m))
    if p2 and "s_front" in p2:
        L(f"POST front={p2['s_front']:.4f} was={s_front:.4f} "
          f"moved={p2['s_front'] - s_front:.4f}")
    return "pull complete"
