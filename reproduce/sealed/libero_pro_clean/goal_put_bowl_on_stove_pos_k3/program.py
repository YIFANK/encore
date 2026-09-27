"""c2clean goal_put_bowl_on_stove_pos_k3 -- v12: stove disc-mask + grasp retry ladder.

Chain of evidence (all from this pack + my own debug-seed 51..65 observations):
  v1  cam_high uses the plain pinhole convention; table top z = 0.901.
  v3  height-gated clustering at 0.032 m isolates a round prop ~0.11 m across
      (top 0.95-0.97) from a 0.19x0.19 m slab whose top is 0.930; the RGB dump
      in v5 shows the round prop is the speckled BOWL and the slab is the
      burner-ringed STOVE.
  v4  a bare-table descent stalls at EEF z 0.909, so the fingertips sit
      0.008 m below the EEF frame.
  v7  api.move converges in ~9 sim steps and covers ~0.07 m per second of
      command; a starved move instead burns ~60, and the episode horizon is
      ~1000 steps.
  v8  approaching the bowl from -y jams the forearm against the cabinet
      (top z 1.20 on that side) and the arm never recovers.
  v9  from +y the arm tracks cleanly down to EEF z 0.939 at the bowl and 0.939
      over the stove.
  v10 straddle offsets along +y from the bowl centre: 0.000 m stalls on the rim
      and closes on air (gap 0.001, effort 0.05); 0.030 m descends to EEF 0.939
      and closes to a 0.007 m gap at effort 3.0 that survives a lift.
The pack agrees with the shape of that grasp: its demos close with EEF z
0.920-0.927 (fingertips below the rim) and settle to a small finger gap, then
release over the stove.

v11 scored 6/8 on seeds 51..65 odd.  Both failures were diagnosed from my logs:
  * the 0.018 m slab gate FUSES the bowl into the stove component on most
    seeds (the bowl stands beside the slab and their skirts touch), which
    dragged the reported stove centre to (0.015, 0.068) on ep57 and the bowl
    was released off the slab.  v12 masks out the whole bowl DISC (xy within
    rim_r + 0.012 of the bowl centre) before looking for the slab, which is
    exact regardless of height.
  * a close that missed by ~5 mm in y left a 0.004 m gap at effort 0.05, and
    the retry could not re-find the bowl because the arm was parked above it.
    v12 retreats out of the camera's line before re-perceiving and walks a
    small ladder of straddle offsets.
"""

import json

import numpy as np

PROVENANCE = {
    "TABLE_Z": {"source": "debug-seed measurement: cam_high depth z-histogram mode (v1)", "allowed": True},
    "TIP_OFFSET": {"source": "debug-seed measurement: bare-table descent stall at EEF 0.909 (v4)", "allowed": True},
    "WS_X": {"source": "debug-seed measurement: cam_high cloud extent", "allowed": True},
    "WS_Y": {"source": "debug-seed measurement: cam_high cloud extent", "allowed": True},
    "PROP_GATE": {"source": "debug-seed gate ladder (v3): separates bowl from stove slab", "allowed": True},
    "SLAB_GATE": {"source": "debug-seed gate ladder (v3): 0.018 m keeps the 0.930 m slab", "allowed": True},
    "BOWL_EXT": {"source": "debug-seed measurement: bowl footprint 0.109-0.116 m both axes (v3, 8 seeds)", "allowed": True},
    "GRASP_OFF": {"source": "debug-seed grasp landscape (v10): +y offset 0.030 m holds at effort 3.0, 0.000 m closes on air", "allowed": True},
    "APPROACH_SIDE": {"source": "debug-seed observation: -y jams on the cabinet (v8 GIF), +y tracks (v9)", "allowed": True},
    "HOVER_Z": {"source": "debug-seed choice: above the bowl rim top 0.974 max (v3)", "allowed": True},
    "FLOOR_Z": {"source": "debug-seed measurement: deepest EEF reached beside the bowl, 0.939 (v9/v10)", "allowed": True},
    "CARRY_Z": {"source": "debug-seed choice: clears the stove top 0.931 plus the hanging bowl", "allowed": True},
    "HOLD_EFFORT": {"source": "FairApi doc: effort 3.0 iff holding; measured 3.0 on a held bowl (v10)", "allowed": True},
    "PLACE_FLOOR": {"source": "debug-seed geometry: bowl base hangs ~0.038 m below the EEF, stove top 0.931", "allowed": True},
    "RETRY_DY": {"source": "debug-seed observation (v11 ep57): a 5 mm y shift flipped a missed close into a hold", "allowed": True},
    "PARK": {"source": "debug-seed observation (v11 ep59): re-perception fails with the arm parked over the bowl; the episode start pose is clear of it", "allowed": True},
}

TABLE_Z = 0.901
TIP_OFFSET = 0.008
WS_X = (-0.45, 0.32)
WS_Y = (-0.42, 0.42)
PROP_GATE = 0.032
SLAB_GATE = 0.018
BOWL_EXT = (0.085, 0.145)
GRASP_OFF = 0.030
HOVER_Z = 0.995
FLOOR_Z = 0.930
CARRY_Z = 1.03
HOLD_EFFORT = 2.0
PLACE_FLOOR = 0.968
RETRY_DY = (0.000, -0.007, 0.007)
PARK = (-0.20, 0.02, 1.14)


def _j(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, dict):
        return {k: _j(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_j(v) for v in o]
    return o


def _log(api, **kw):
    s = json.dumps(_j(kw))
    n = (len(s) + 1699) // 1700
    for i in range(n):
        api.log("%d/%d %s" % (i, n, s[i * 1700:(i + 1) * 1700]))


def cloud(frame):
    depth = np.asarray(frame.depth, float)
    K = np.asarray(frame.intrinsics, float)
    T = np.asarray(frame.t_base_cam, float)
    H, W = depth.shape
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    z = depth
    cam = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z, np.ones_like(z)], -1)
    return cam @ T.T[:, :3]


def label(mask):
    H, W = mask.shape
    lab = np.zeros((H, W), int)
    cur = 0
    seen = mask.copy()
    for (r0, c0) in np.argwhere(mask):
        if not seen[r0, c0]:
            continue
        cur += 1
        stack = [(r0, c0)]
        seen[r0, c0] = False
        while stack:
            r, c = stack.pop()
            lab[r, c] = cur
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < W and seen[rr, cc]:
                    seen[rr, cc] = False
                    stack.append((rr, cc))
    return lab, cur


def components(xyz, inws, gate, amin):
    lab, n = label(inws & (xyz[..., 2] > TABLE_Z + gate))
    out = []
    for i in range(1, n + 1):
        m = lab == i
        a = int(m.sum())
        if a < amin:
            continue
        p = xyz[m]
        out.append(dict(a=a,
                        x=(float(p[:, 0].min()), float(p[:, 0].max())),
                        y=(float(p[:, 1].min()), float(p[:, 1].max())),
                        zt=float(p[:, 2].max())))
    return out


def find_bowl(api, tag):
    """Round prop of bowl size, top below 1.0 m: the speckled bowl."""
    f = api.capture("cam_high")
    xyz = cloud(f)
    inws = ((xyz[..., 0] > WS_X[0]) & (xyz[..., 0] < WS_X[1]) &
            (xyz[..., 1] > WS_Y[0]) & (xyz[..., 1] < WS_Y[1]) &
            (xyz[..., 2] < TABLE_Z + 0.30))
    best, bowl = 1e9, None
    for c in components(xyz, inws, PROP_GATE, 300):
        ex, ey = c["x"][1] - c["x"][0], c["y"][1] - c["y"][0]
        if c["a"] > 9000 or c["zt"] > 1.00:
            continue
        if not (BOWL_EXT[0] < ex < BOWL_EXT[1] and BOWL_EXT[0] < ey < BOWL_EXT[1]):
            continue
        if abs(ex - ey) < best:
            best, bowl = abs(ex - ey), c
    if bowl is None:
        _log(api, ev="nobowl", tag=tag)
        return None
    c = (0.5 * (bowl["x"][0] + bowl["x"][1]), 0.5 * (bowl["y"][0] + bowl["y"][1]))
    r = 0.25 * ((bowl["x"][1] - bowl["x"][0]) + (bowl["y"][1] - bowl["y"][0]))
    _log(api, ev="bowl", tag=tag, c=[round(v, 4) for v in c], r=round(r, 4),
         zt=round(bowl["zt"], 3))
    return c, r, bowl["zt"], xyz, inws


def find_stove(api, xyz, inws, bowl_c, bowl_r):
    """Wide flat slab with a ~0.93 top: the stove.

    The bowl's skirt touches the slab, so remove every point whose xy falls in
    the bowl's disc first -- that is exact at any height, unlike a gate.
    """
    d = np.hypot(xyz[..., 0] - bowl_c[0], xyz[..., 1] - bowl_c[1])
    inws = inws & (d > bowl_r + 0.012)
    stove, area = None, 0
    for c in components(xyz, inws, SLAB_GATE, 3000):
        ex, ey = c["x"][1] - c["x"][0], c["y"][1] - c["y"][0]
        if c["zt"] > 0.99 or ex < 0.12 or ey < 0.12:
            continue
        if c["a"] > area:
            stove, area = c, c["a"]
    if stove is None:
        _log(api, ev="nostove")
        return None
    c = (0.5 * (stove["x"][0] + stove["x"][1]), 0.5 * (stove["y"][0] + stove["y"][1]))
    _log(api, ev="stove", c=[round(v, 4) for v in c], zt=round(stove["zt"], 3),
         x=[round(v, 3) for v in stove["x"]], y=[round(v, 3) for v in stove["y"]])
    return c, stove["zt"]


def descend(api, x, y, z0, zmin, step=0.008, seconds=0.6):
    """Step down until the EEF stops falling; return the last EEF."""
    z, prev = z0, 9.0
    e = np.asarray(api.eef(), float)
    while z > zmin - 1e-9:
        api.move([x, y, z], seconds=seconds)
        e = np.asarray(api.eef(), float)
        if e[2] > prev - 0.002:
            break
        prev = float(e[2])
        z -= step
    return e


def run(api):
    _log(api, ev="start", eef=api.eef(), instr=api.instruction())
    found = find_bowl(api, "init")
    if found is None:
        return
    (bx, by), rim_r, bzt, xyz, inws = found
    st = find_stove(api, xyz, inws, (bx, by), rim_r)

    held = False
    for trial, dy in enumerate(RETRY_DY):
        if trial:
            api.grip(0.08)
            api.move(list(PARK), seconds=1.6)
            f2 = find_bowl(api, "retry%d" % trial)
            if f2 is None:
                break
            (bx, by), rim_r, bzt = f2[0], f2[1], f2[2]
        gy = by + GRASP_OFF + dy
        api.grip(0.08)
        api.move([bx, gy, HOVER_Z], seconds=1.4)
        e = descend(api, bx, gy, HOVER_Z - 0.008, FLOOR_Z)
        _log(api, ev="bottom", trial=trial, eef=e.tolist())
        api.grip(0.0)
        api.settle(0.3)
        api.move([bx, gy, CARRY_Z], seconds=1.0)
        g = dict(api.gripper())
        _log(api, ev="lift", trial=trial, dy=dy, grip=g, eef=api.eef())
        if g["effort"] >= HOLD_EFFORT and g["width_m"] > 0.003:
            held = True
            break
        api.grip(0.08)
    if not held or st is None:
        _log(api, ev="abort", held=held)
        return

    (sx, sy), szt = st
    api.move([sx, sy + GRASP_OFF, CARRY_Z], seconds=1.6)
    _log(api, ev="overstove", eef=api.eef(), grip=dict(api.gripper()))
    e = descend(api, sx, sy + GRASP_OFF, CARRY_Z - 0.008, PLACE_FLOOR)
    _log(api, ev="placed", eef=e.tolist(), grip=dict(api.gripper()))
    api.grip(0.08)
    api.settle(0.4)
    api.move([sx, sy + GRASP_OFF, CARRY_Z + 0.03], seconds=1.0)
    _log(api, ev="released", eef=api.eef(), grip=dict(api.gripper()))
    api.move([sx - 0.05, sy + 0.20, 1.10], seconds=1.4)
    find_bowl(api, "final")
