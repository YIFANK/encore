"""close the microwave -- v6 (= v4, dead mug code removed, finder fallback added).

Scene facts, all measured from the debug-seed cam_high dumps (v0/v1d, seeds
51/53/55/59/63):
  * table top z = 0.901 (modal deprojected z)
  * microwave: a fixed box whose TOP is a solid slab at table+0.21, footprint
    x[-0.16,0.16] y[0.25,0.37]; its open door is a thin panel whose top edge is
    at the same height, hinged at the box's -x front corner and swung out
    toward -y.  Door length 0.24-0.28 m; the open angle varies by seed
    (-80 to -110 deg).
  * both mugs top out at table+0.105, i.e. below the door/box tops.
  * the arm at home occupies z > table+0.28, so the band [table+0.14,+0.19]
    sees only the door panel and the box's near faces.

Segmentation: erode the top-slab footprint (band [table+0.195,+0.245], which
excludes the arm) -- a 1-2 cell wide door edge vanishes, the box slab survives.
The door is then whatever the [table+0.14,+0.19] band holds outside a skirt
round that slab; the hinge is where the door line re-enters the slab.

  * pressing the closed gripper into the bare table (v2, every debug seed)
    stops the eef 0.008 m above it: the eef reference sits AT the fingertips
    and the hand body rises ~0.10 m ABOVE it.  So a transit has to clear the
    tops by the fingertip height, and a push has to be low enough that the
    hand passes BESIDE the door instead of riding its top edge -- v2 transited
    at table+0.20 and clipped the door on the way past, and pushed at
    table+0.165 where the hand fouled the door's top edge (0.08 m of standing
    residual through the whole sweep).

The two mugs are left alone.  The pack's three demos all pick one up and carry
it away first, but measured, the door's swing disc (radius = the door's own
length about the measured hinge) does not reach either mug on any debug seed --
and v3, which did clear it, stalled the arm at x ~ +0.09 on half of them.

Plan: close the gripper to a blade, drop it behind the door just past the free
edge at the height the demos push from, and sweep it about the hinge until the
door reaches its stop.  Every transit runs at table+0.26 so the fingertips
clear the table+0.21 tops.
"""
import numpy as np

PROVENANCE = {
    "CROP": {"source": "debug-seed dumps: the whole scene lies in "
                       "x[-0.45,0.35] y[-0.45,0.45]", "allowed": True},
    "TOP_BAND": {"source": "debug-seed dumps: microwave top = table+0.211, arm at "
                           "home starts at table+0.28 -> [+0.195,+0.245] is the "
                           "box top alone", "allowed": True},
    "FACE_BAND": {"source": "debug-seed dumps: mug tops = table+0.105 and the arm "
                            "is above table+0.28, so [+0.14,+0.19] holds only the "
                            "door panel and the box's near faces", "allowed": True},
    "CELL": {"source": "0.015 m: half the measured door-panel thickness scale, so a "
                       "3x3 erosion deletes the door edge and keeps the box slab",
             "allowed": True},
    "TIP_OFFSET": {"source": "v2 blocked-descent probe on the bare table, all 8 "
                             "debug seeds: eef stops 0.0078 m above table_z; "
                             "TRANSIT_H and DOOR_EEF_H are derived from it",
                   "allowed": True},
    "TRANSIT_H": {"source": "0.26 m of eef above table: fingertips (eef-0.008) then "
                            "clear the table+0.21 door/box tops by 0.04",
                  "allowed": True},
    "DOOR_EEF_H": {"source": "0.085 m of eef above table: the pack's three demos all "
                             "push the door with the eef at 0.98-0.99 = table+0.08 "
                             "to +0.09, which also keeps the hand below the door top",
                   "allowed": True},
    "CONTACT_FRAC": {"source": "0.65 of the measured door length; clear of the free "
                               "edge and of the hinge", "allowed": True},
    "STANDOFF": {"source": "0.045 m behind the door plane; the v0 demo replay jammed "
                           "when a waypoint sat on the plane itself", "allowed": True},
    "FALLBACK_HINGE": {"source": "median of the hinge/free-edge/box-footprint the "
                                 "finder measured on debug seeds 51-65; used only "
                                 "if the finder returns nothing", "allowed": True},
    "FALLBACK_FREE": {"source": "see FALLBACK_HINGE", "allowed": True},
    "FALLBACK_BOX": {"source": "see FALLBACK_HINGE", "allowed": True},
    "OVERSHOOT_DEG": {"source": "18 deg past the measured door-to-box-axis angle; the "
                                "eroded slab's principal axis reads ~10 deg off the "
                                "true face direction on every debug seed",
                      "allowed": True},
}

CROP = ((-0.45, 0.35), (-0.45, 0.45))
CELL = 0.015
TIP_OFFSET = 0.008   # measured: the eef reference sits at the fingertips
TRANSIT_H = 0.26
DOOR_EEF_H = 0.085
CONTACT_FRAC = 0.65
STANDOFF = 0.045
OVERSHOOT_DEG = 18.0
N_ARC = 6
FALLBACK_HINGE = (-0.175, 0.253)
FALLBACK_FREE = (-0.205, 0.005)
FALLBACK_BOX = [(-0.16, 0.25), (0.16, 0.25), (-0.16, 0.37), (0.16, 0.37)]


# ---------------------------------------------------------------- geometry
def deproject_all(f):
    d = np.asarray(f.depth, float)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = d.shape
    v, u = np.mgrid[0:h, 0:w]
    P = np.stack([(u - K[0, 2]) * d / K[0, 0], (v - K[1, 2]) * d / K[1, 1],
                  d, np.ones_like(d)], -1) @ T.T
    return P[..., :3]


def in_crop(P):
    return ((P[..., 0] > CROP[0][0]) & (P[..., 0] < CROP[0][1]) &
            (P[..., 1] > CROP[1][0]) & (P[..., 1] < CROP[1][1]))


def table_z(P):
    z = P[in_crop(P) & (P[..., 2] > 0.5) & (P[..., 2] < 1.6)][:, 2]
    hist, edges = np.histogram(z, bins=200)
    i = int(hist.argmax())
    return float(np.median(z[(z > edges[i] - 0.01) & (z < edges[i + 1] + 0.01)]))


def cells_of(pts):
    return set(map(tuple, np.floor(pts / CELL).astype(int)))


def dilate(cells, k):
    out = set()
    for a, b in cells:
        for dx in range(-k, k + 1):
            for dy in range(-k, k + 1):
                out.add((a + dx, b + dy))
    return out


def erode(cells):
    return {c for c in cells
            if all((c[0] + dx, c[1] + dy) in cells
                   for dx in (-1, 0, 1) for dy in (-1, 0, 1))}


def biggest(cells):
    seen, best = set(), set()
    for c in cells:
        if c in seen:
            continue
        st, grp = [c], set()
        seen.add(c)
        while st:
            q = st.pop()
            grp.add(q)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (q[0] + dx, q[1] + dy)
                    if n in cells and n not in seen:
                        seen.add(n)
                        st.append(n)
        if len(grp) > len(best):
            best = grp
    return best


def unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else v


def rot2(v, a):
    c, s = np.cos(a), np.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def find_door(P, tz):
    """(hinge, free_edge, box_cell_centres) or None."""
    crop = in_crop(P)
    top = P[crop & (P[..., 2] > tz + 0.195) & (P[..., 2] < tz + 0.245)][:, :2]
    if len(top) < 50:
        return None
    box = biggest(erode(cells_of(top)))
    if len(box) < 20:
        return None
    boxc = (np.array(sorted(box), float) + 0.5) * CELL
    skirt = dilate(box, 3)
    band = P[crop & (P[..., 2] > tz + 0.14) & (P[..., 2] < tz + 0.19)][:, :2]
    if len(band) == 0:
        return None
    keep = np.array([tuple(k) not in skirt
                     for k in np.floor(band / CELL).astype(int)])
    D = band[keep]
    if len(D) < 30:
        return None
    groups = {}
    for i, k in enumerate(map(tuple, np.floor(D / CELL).astype(int))):
        groups.setdefault(k, []).append(i)
    comp = biggest(set(groups))
    D = D[np.concatenate([groups[c] for c in comp])]
    if len(D) < 30:
        return None
    c = D.mean(0)
    q = D - c
    w, V = np.linalg.eigh(q.T @ q)
    ax = V[:, int(np.argmax(w))]
    t = q @ ax
    e0, e1 = c + ax * t.min(), c + ax * t.max()
    bc = boxc.mean(0)
    if np.linalg.norm(e0 - bc) > np.linalg.norm(e1 - bc):
        e0, e1 = e1, e0
    step = unit(e0 - e1) * 0.005
    solid = dilate(box, 1)
    H = e0.copy()
    for _ in range(24):
        nxt = H + step
        if tuple(np.floor(nxt / CELL).astype(int)) in solid:
            break
        H = nxt
    return H, e1, boxc


# ---------------------------------------------------------------- policy
def run(api):
    api.log("instruction %r" % api.instruction())
    P = deproject_all(api.capture("cam_high"))
    tz = table_z(P)
    fd = find_door(P, tz)
    api.log("table_z %.4f door_found %s" % (tz, fd is not None))
    if fd is None:
        # never triggered on the 15 debug seeds; median debug-seed geometry so a
        # perception miss still attempts the push instead of standing still.
        H = np.array(FALLBACK_HINGE)
        F = np.array(FALLBACK_FREE)
        boxc = np.array(FALLBACK_BOX)
        api.log("fallback geometry")
    else:
        H, F, boxc = fd
    bc = boxc.mean(0)
    r = F - H
    L = float(np.linalg.norm(r))
    rh = unit(r)
    tan = np.array([-rh[1], rh[0]])
    s = 1.0 if float(tan @ (bc - F)) > 0 else -1.0
    tan = tan * s
    back = -tan
    qb = boxc - bc
    wv, V = np.linalg.eigh(qb.T @ qb)
    axis = V[:, int(np.argmax(wv))]
    if float(axis @ (bc - H)) < 0:
        axis = -axis
    a_open = float(np.arctan2(r[1], r[0]))
    a_shut = float(np.arctan2(axis[1], axis[0]))
    sweep = ((a_shut - a_open) * s + 2 * np.pi) % (2 * np.pi)
    sweep = float(np.clip(sweep, np.radians(45), np.radians(135))) + np.radians(OVERSHOOT_DEG)
    api.log("H %s F %s L %.3f boxC %s s %+.0f open %.1f shut %.1f sweep %.1f"
            % (H.round(4).tolist(), F.round(4).tolist(), L, bc.round(3).tolist(),
               s, np.degrees(a_open), np.degrees(a_shut), np.degrees(sweep)))

    api.grip(0.0)
    zt = tz + TRANSIT_H

    # -- set the blade behind the door and sweep the hinge arc ---------------
    zd = tz + DOOR_EEF_H
    entry = F + rh * 0.10 + back * 0.09
    contact = H + rh * (CONTACT_FRAC * L) + back * STANDOFF
    api.log("zd %.3f entry %s contact %s"
            % (zd, entry.round(3).tolist(), contact.round(3).tolist()))
    api.move([entry[0], entry[1], zt], seconds=0.5)
    api.move([entry[0], entry[1], zd], seconds=0.4)
    res = api.move([contact[0], contact[1], zd], seconds=0.5)
    api.log("contact residual %.4f eef %s" % (res, np.asarray(api.eef()).round(4).tolist()))

    v = contact - H
    for k in range(1, N_ARC + 1):
        p = H + rot2(v, sweep * k / N_ARC * s)
        res = api.move([p[0], p[1], zd], seconds=0.45)
        api.log("arc %d -> %s residual %.4f eef %s"
                % (k, p.round(3).tolist(), res, np.asarray(api.eef()).round(4).tolist()))

    P2 = deproject_all(api.capture("cam_high"))
    fd2 = find_door(P2, tz)
    if fd2 is None:
        api.log("post: no door strip left outside the box skirt")
    else:
        api.log("post H %s F %s L %.3f"
                % (fd2[0].round(4).tolist(), fd2[1].round(4).tolist(),
                   float(np.linalg.norm(fd2[1] - fd2[0]))))
    return "v6"
