"""Opening things. The capability whose absence was scoring zero.

The evaluation report named this as a representational failure rather than a
skill failure: asked to "open the middle drawer of the cabinet", the planner had
no predicate for open or closed, reduced the goal to gripper_empty(arm) — a
condition already true — verified it correctly, and stopped without touching the
cabinet. Nothing in the loop was broken. A plan can only be as honest as the
predicates available to write it in.

Adding the predicate needed a skill behind it, and finding the skill took four
measurements:

  * the drawer is a slide joint with 160 mm of travel;
  * the pre-grasp point in front of its handle is 314 mm out of reach with the
    wrist held vertical — which every motion in this system did, because the
    simulator backend sent [dx, dy, dz, 0, 0, 0, grip];
  * commanding the three zeros brings that point to within 19 mm;
  * facing the drawer with the fingers vertical and gripping level with the
    face, then pulling 240 mm, moves the drawer 140-160 mm of its 160 and
    satisfies the benchmark's own predicate.

Which direction to pull is measured, not looked up. A drawer front is a flat
panel, so the plane fitted to its own depth pixels has a normal that IS the
direction it slides — no privileged access to the simulator's joints, which is
the difference between a skill and a demo. Where a cabinet entity is also known,
the handle's offset from its body gives a second opinion, and the two are
cross-checked before the arm moves.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from ..types import SkillResult
from . import SkillContext, skill

# Measured on the winning configuration; see the module docstring.
STANDOFF_M = 0.085        # where the wrist turns, clear of the cabinet
PULL_M = 0.24             # longer than the 160 mm of travel, so the stop decides
APPROACH_LIFT_M = 0.06    # turn the wrist above the slot, not inside it
GRIP_DEPTH_M = 0.0        # aim at the face; the fingers straddle the handle
# What counts as opened. These drawers travel 160 mm and this benchmark's own
# WoodenCabinet.is_open wants 140 mm of it, so 100 mm is deliberately LOOSER
# than the grader — and that is the dangerous direction, because it lets us
# report a success the benchmark denies. It is set here rather than at 140 mm
# because the predicate should mean "substantially open" in general, not "past
# the number this particular simulator uses"; the gap is closed by the skill
# pulling to the stop, not by tuning the verifier to the grader.
#
# Defined once, here, and imported by the verifier. The last time a definition
# like this existed in two places — what counts as holding something — fixing
# one copy left the others wrong for a week.
OPEN_MIN_TRAVEL_M = 0.10
MAX_PLANE_SAMPLES = 1200  # enough for a plane; see face_normal
# Pull, look, pull again. Three rounds is the point where a drawer that has not
# opened is not going to, and each round costs one detection.
MAX_PULLS = 3
PULL_STALL_M = 0.012      # movement below this is grounding noise, not progress
# How far off its own rail a sighting may be before it is a different object.
# A drawer front slides; it does not move sideways and it does not change
# height. Generous enough to absorb grounding error (about 25 mm here), tight
# enough to reject the handle one shelf up (126 mm away in the case measured).
OFF_RAIL_MAX_M = 0.05
# Side of the fallback patch used to fit the drawer face, as a fraction of the
# image width. Big enough to be mostly panel, small enough not to swallow the
# cabinet's side wall — which is what tilts the fitted plane.
FACE_WINDOW_FRACTION = 0.16
# Half-height of the band around the handle kept when fitting the face. A
# drawer front is a vertical strip about 90 mm tall here; the band exists to
# drop the horizontal surfaces above and below it, which an overhead camera
# sees FIRST and which turn the fitted plane on its side.
FACE_BAND_M = 0.045
# How far the two direction estimates may disagree before the plane fit is
# treated as being about a different surface entirely.
CROSSCHECK_MAX_DEG = 45.0


class Face:
    """A fitted drawer front: which way it points, and where its surface is."""

    def __init__(self, normal: np.ndarray, centre: np.ndarray, flatness: float) -> None:
        self.normal = normal
        self.centre = centre
        self.flatness = flatness   # thickness/width of the fitted slab; smaller is flatter


def fit_face(frame, box, mask=None, near_z=None) -> Optional[Face]:
    """Fit the drawer front as a plane, and return both halves of the answer.

    The centre matters as much as the normal. Grounding returns a box centroid,
    which for a recessed drawer sits INSIDE the cabinet — measured, that put the
    grip 30 mm behind the panel, and the pull that followed moved the drawer
    42 mm of the 140 the benchmark requires. The mean of the near face's own
    pixels lies on the panel by construction.
    """
    return _fit(frame, box, mask, near_z)


def face_normal(frame, box, mask=None, near_z=None) -> Optional[np.ndarray]:
    """Which way the drawer front faces, fitted to its own depth pixels.

    A drawer front is a flat panel, so its normal IS the direction it slides —
    and unlike the cabinet-relative estimate this needs one entity rather than
    two. That matters more than elegance: the planner's first live attempt wrote
    `open_drawer(entity=top_drawer)` with no cabinet at all, the skill refused
    for a missing argument, and the repair loop retried the identical call until
    the budget ran out.

    Returned horizontal and pointing OUT of the cabinet (toward the camera),
    because "out" is the only end of the axis that a pull can use.
    """
    got = _fit(frame, box, mask, near_z)
    return None if got is None else got.normal


# Why the last fit was refused. A module-level note rather than a return value
# because every caller wants the Face and only the logger wants the reason —
# and a silent None here already cost four rounds of pulling in the wrong
# direction before anyone noticed the measurement was never taken.
last_fit_refusal = ""


def _fit(frame, box, mask=None, near_z=None) -> Optional[Face]:
    global last_fit_refusal
    last_fit_refusal = ""
    if frame is None or frame.depth is None or frame.t_base_cam is None or box is None:
        last_fit_refusal = "no frame, no depth, no extrinsics, or no box"
        return None
    u0, v0, u1, v1 = (int(v) for v in box[:4])
    h, w = frame.depth.shape[:2]
    u0, u1 = max(0, min(u0, u1)), min(w - 1, max(u0, u1))
    v0, v1 = max(0, min(v0, v1)), min(h - 1, max(v0, v1))
    if u1 - u0 < 6 or v1 - v0 < 6:
        last_fit_refusal = f"box is {u1 - u0}x{v1 - v0} px, too small to fit a plane"
        return None
    # Subsample. A plane needs three points and is happy with a few hundred;
    # deprojecting every pixel of a 512-wide box is a quarter of a million
    # Python-level calls for an answer that does not get better.
    stride = max(1, int(np.sqrt((u1 - u0 + 1) * (v1 - v0 + 1) / MAX_PLANE_SAMPLES)))
    us, vs = np.meshgrid(np.arange(u0, u1 + 1, stride), np.arange(v0, v1 + 1, stride))
    keep = np.ones(us.shape, dtype=bool)
    if mask is not None:
        keep &= mask[v0:v1 + 1:stride, u0:u1 + 1:stride].astype(bool)[:keep.shape[0], :keep.shape[1]]
    pts = []
    for u, v, k in zip(us.ravel(), vs.ravel(), keep.ravel()):
        if not k:
            continue
        p = frame.deproject(int(u), int(v))
        if p is not None:
            pts.append(p)
    if len(pts) < 40:
        last_fit_refusal = f"only {len(pts)} pixels in the box deprojected"
        return None
    P = np.asarray(pts, float)
    eye = frame.t_base_cam[:3, 3]

    # Cut to a height band around the handle first. "Keep the nearest half"
    # alone was wrong for this camera: agentview looks DOWN at the cabinet, so
    # the nearest surface in the window is the top of a drawer, not its face.
    # The plane came out horizontal, the horizontal-normal test then rejected
    # it, and fit_face returned None in every live episode without a word —
    # which is how the pull spent four rounds on a cabinet-relative direction
    # 53 degrees off the rail. A drawer front is a vertical strip; the band
    # keeps it and drops the horizontal surfaces above and below.
    if near_z is not None:
        band = np.abs(P[:, 2] - float(near_z)) <= FACE_BAND_M
        if band.sum() >= 30:
            P = P[band]
        else:
            last_fit_refusal = (f"only {int(band.sum())} of {len(P)} points sit within "
                                f"{FACE_BAND_M * 1000:.0f} mm of the handle's height")

    # Then keep the near surface within that band: a detector box around a
    # drawer front also catches the cabinet's side wall and whatever is behind
    # the opening, and a plane fitted through all of it is a plane through none
    # of it — measured, 18 degrees off the rail, which binds a prismatic joint
    # rather than opening it.
    dist = np.linalg.norm(P - eye, axis=1)
    P = P[dist <= np.quantile(dist, 0.5)]
    if len(P) < 30:
        last_fit_refusal = f"{len(P)} points left after keeping the near surface"
        return None

    # Then refit twice, dropping whatever still sits off the plane. Two passes
    # is enough — the survivors after the first are already the panel.
    centre = P.mean(axis=0)
    n = None
    for _ in range(2):
        _, s, vt = np.linalg.svd(P - centre, full_matrices=False)
        if s[1] < 1e-6 or s[2] / s[1] > 0.45:
            # Not planar enough to call it a face. Refusing beats returning the
            # smallest axis of a blob and calling it a normal.
            last_fit_refusal = (f"not planar: thickness/width = "
                                f"{float(s[2] / max(s[1], 1e-9)):.2f}, over 0.45")
            return None
        n = vt[2]
        resid = np.abs((P - centre) @ n)
        keep = resid <= max(0.004, 2.5 * float(np.median(resid)))
        if keep.sum() < 30:
            break
        P = P[keep]
        centre = P.mean(axis=0)
    flat = float(s[2] / max(s[0], 1e-9))
    n = np.asarray(n, float)
    n[2] = 0.0                       # a drawer slides horizontally
    if float(np.linalg.norm(n)) < 0.3:
        last_fit_refusal = (f"the fitted plane is horizontal (|n_xy| = "
                            f"{float(np.linalg.norm(n)):.2f}) — a shelf, not a drawer front")
        return None
    n = n / np.linalg.norm(n)
    to_camera = eye - centre
    to_camera[2] = 0.0
    if float(n @ to_camera) < 0:
        n = -n
    return Face(n, centre, flat)


def face_box(frame, box, mask, px) -> Optional[tuple]:
    """The image region to fit the drawer's face in, however little we were given.

    Fitting a plane needs a patch of the panel, and on this benchmark neither
    source of one is reliable: ER-2 points at the drawer but declines to box it,
    and no segmentation service runs in the simulator environment. The result was
    that `fit_face` returned None in every live episode and the pull always fell
    back to the cabinet-relative direction — 12.8 degrees off the rail, which
    binds a prismatic joint. Measured: 17 mm of travel, three attempts, against
    the 140 mm the benchmark wants.

    A window around the point the detector DID give is enough. A drawer front is
    the largest flat thing near its own handle, so a patch centred there is
    mostly panel whichever way the box would have been drawn.
    """
    if box is not None:
        return tuple(int(v) for v in box[:4])
    if mask is not None and mask.any():
        ys, xs = np.where(mask)
        return (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
    if px is None or frame is None or getattr(frame, "rgb", None) is None:
        return None
    h, w = frame.rgb.shape[:2]
    half = max(12, int(round(w * FACE_WINDOW_FRACTION / 2)))
    u, v = int(px[0]), int(px[1])
    return (max(0, u - half), max(0, v - half),
            min(w - 1, u + half), min(h - 1, v + half))


def face_direction(handle_xyz, body_xyz) -> Optional[np.ndarray]:
    """Which way the drawer opens: away from the cabinet, through its face.

    Horizontal by construction. A vertical component here would come from the
    handle being lower than the cabinet's centroid, which says nothing about
    which way it slides.
    """
    u = np.asarray(handle_xyz, float)[:2] - np.asarray(body_xyz, float)[:2]
    n = float(np.linalg.norm(u))
    if n < 0.02:
        # The handle and the cabinet grounded to the same place, so the vector
        # is noise. Refusing is right: a made-up direction sends the arm into
        # the furniture.
        return None
    return np.array([u[0] / n, u[1] / n, 0.0])


def facing_rotation(u: np.ndarray) -> np.ndarray:
    """Tool frame that faces along -u with the fingers closing vertically.

    Built by cross products rather than written out. The first version of this
    was three hand-written matrices with determinant -1 — reflections, not
    rotations — and the arm chased each of them for four seconds while reporting
    a 177-degree error, which is indistinguishable from an unreachable pose.
    """
    z = -np.asarray(u, float)
    z = z / np.linalg.norm(z)
    up = np.array([0.0, 0.0, 1.0])
    y = up - z * float(up @ z)
    if float(np.linalg.norm(y)) < 1e-6:
        y = np.cross(z, np.array([1.0, 0.0, 0.0]))
    y = y / np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def _pull_direction(ctx: SkillContext, entity: str, cabinet: Optional[str],
                    h: np.ndarray) -> tuple[Optional[np.ndarray], str, Optional[np.ndarray]]:
    """Which way to pull, where the panel's surface is, and how we know.

    Two independent estimates of the direction, in order of directness. The
    plane fitted to the drawer's own front is a measurement of the thing itself;
    the cabinet-relative vector is an inference from where the handle sits on a
    body. When both are available they are compared, because two methods
    agreeing is worth recording and two methods disagreeing is worth refusing to
    act on.
    """
    tr = ctx.belief.track(entity)
    from_plane, surface = None, None
    if tr.camera and tr.px is not None:
        try:
            frame = ctx.robot.capture(tr.camera)
            from .sensing import _mask_for, _pick_box  # noqa: PLC0415

            # Order matters, and it was wrong. The detector declines to box a
            # drawer front, so the segmenter used to be asked with a point
            # alone — and a point on a cabinet is genuinely ambiguous between
            # the handle, the drawer and the cabinet. It answered "the cabinet":
            # 22 098 px, no plane, no direction. Building the fallback window
            # FIRST and handing that to the segmenter as a box says which scale
            # was meant. Measured against the simulator's own joint axis on the
            # scene that had been failing: window alone 14.9 degrees off the
            # rail, point-prompted mask no fit at all, window-as-box plus point
            # 1.9 degrees off.
            box = _pick_box(ctx, frame, tr.description, tr.px, np.asarray(h[:2], float), entity)
            box = face_box(frame, box, None, tr.px)
            mask = _mask_for(ctx, frame, box, tr.px)
            face = fit_face(frame, box, mask, near_z=float(h[2]))
            if face is None and mask is not None:
                # The mask can be tighter than the panel; the window without it
                # is the coarser answer but still an answer.
                face = fit_face(frame, box, None, near_z=float(h[2]))
            if face is not None:
                from_plane, surface = face.normal, face.centre
                ctx.log.event("face_fitted", entity=entity,
                              normal=np.round(face.normal, 3).tolist(),
                              centre=np.round(face.centre, 3).tolist(),
                              flatness=round(face.flatness, 4))
            else:
                # Say so. A silent None here is indistinguishable from never
                # having tried, and that is exactly how four rounds of pulling
                # went by on a fallback direction without anyone noticing the
                # measurement was missing.
                ctx.log.event("face_fit_failed", entity=entity,
                              box=list(box) if box is not None else None,
                              has_mask=mask is not None,
                              reason=last_fit_refusal or "unknown")
        except Exception as e:
            ctx.log.event("face_normal_failed", entity=entity, error=f"{type(e).__name__}: {e}")

    from_body = None
    anchor = cabinet
    if anchor is None:
        # The planner does forget this argument — measured, on its first live
        # attempt. A cabinet already in the belief store is a better answer than
        # a refusal the repair loop will retry unchanged.
        anchor = next((eid for eid, t in ctx.belief.entities.items()
                       if eid != entity and t.xyz_base is not None
                       and any(w in f"{eid} {t.description}".lower()
                               for w in ("cabinet", "cupboard", "dresser", "chest"))), None)
        if anchor:
            ctx.log.event("cabinet_inferred", entity=entity, cabinet=anchor,
                          reason="no cabinet argument was given")
    if anchor is not None and ctx.belief.track(anchor).xyz_base is not None:
        from_body = face_direction(h, ctx.belief.track(anchor).xyz_base)

    if from_plane is not None and from_body is not None:
        off = float(np.degrees(np.arccos(np.clip(from_plane @ from_body, -1.0, 1.0))))
        ctx.log.event("pull_direction_crosscheck", entity=entity,
                      plane=np.round(from_plane, 3).tolist(),
                      body=np.round(from_body, 3).tolist(), disagree_deg=round(off, 1))
        if off > CROSSCHECK_MAX_DEG:
            # They disagree, so one of them is about the wrong surface — and
            # measured, it is the plane: the fallback patch around the handle
            # caught the cabinet's SIDE panel, fitted it beautifully
            # (flatness 0.012) and returned a normal along +x for a drawer that
            # opens along -y. The cabinet-relative direction is coarser but it
            # is derived from two separately grounded objects, so it cannot lock
            # onto one flat surface by accident. Prefer it, and say so.
            ctx.log.event("face_normal_overruled", entity=entity,
                          plane=np.round(from_plane, 3).tolist(),
                          body=np.round(from_body, 3).tolist(),
                          disagree_deg=round(off, 1))
            return from_body, (f"from the handle's offset from {anchor!r} — the plane "
                               f"fitted to the face pointed {off:.0f} degrees away and "
                               "was probably a different flat surface"), None
    if from_plane is not None:
        return from_plane, "fitted to the drawer front's depth", surface
    if from_body is not None:
        return from_body, f"from the handle's offset from {anchor!r}", None
    return None, ("no depth plane could be fitted to the drawer front and no "
                  "cabinet entity was available to infer it from"), None


@skill(
    name="open_drawer",
    description=(
        "Pull a drawer (or any handled panel) open. The direction to pull is measured "
        "from the drawer front's own depth; passing the cabinet entity as well gives a "
        "second, cross-checked estimate. Requires a robot that can command wrist orientation."
    ),
    params={
        "entity": {"type": "string"},
        "cabinet": {"type": "string"},
        "distance": {"type": "number"},
        "arm": {"type": "string", "enum": ["left", "right", "arm", "auto"]},
    },
    required=["entity"],
    post_hint="open(entity) — the drawer front has travelled out of the cabinet",
)
def open_drawer(ctx: SkillContext, entity: str, cabinet: Optional[str] = None,
                distance: float = PULL_M, arm: str = "auto") -> SkillResult:
    from .primitives import choose_arm  # noqa: PLC0415

    if not getattr(ctx.robot, "can_orient", lambda: False)():
        return SkillResult(ok=False, error=(
            "this robot cannot command wrist orientation, and a drawer handle "
            "cannot be reached with a vertical wrist (measured: 314 mm short)"))

    handle = ctx.belief.track(entity)
    if handle.xyz_base is None:
        return SkillResult(ok=False, error=f"open_drawer: {entity!r} has no location; perceive it first")

    from .sensing import ground_entity  # noqa: PLC0415

    side = choose_arm(ctx, np.asarray(handle.xyz_base, float), arm)
    before = np.asarray(handle.first_xyz if handle.first_xyz is not None
                        else handle.xyz_base, float)
    travelled, notes = 0.0, []

    # One pull is not enough and the reason is measurable. The direction is
    # inferred from grounding, grounding is good to a couple of centimetres, and
    # a prismatic joint pulled 18 degrees off its rail binds rather than opens —
    # measured, 42 mm of the 140 the benchmark demands. So: pull, LOOK at where
    # the panel actually ended up, recompute the direction from there, pull
    # again. Each round re-measures, which is the only thing that recovers from
    # a direction error the first round could not have known about.
    for attempt in range(MAX_PULLS):
        h = np.asarray(ctx.belief.track(entity).xyz_base, float)
        u, how, surface = _pull_direction(ctx, entity, cabinet, h)
        if u is None:
            if attempt == 0:
                return SkillResult(ok=False, error=(
                    f"open_drawer: cannot tell which way {entity!r} opens — {how}"))
            break
        # Grip the panel, not the middle of its bounding box: a box centroid for
        # a recessed drawer sits inside the cabinet, and the fingers then close
        # behind the front instead of around it.
        grip = np.asarray(surface, float) if surface is not None else h
        R = facing_rotation(u)
        ctx.log.event("open_drawer_plan", entity=entity, cabinet=cabinet, arm=side,
                      attempt=attempt + 1, grip=np.round(grip, 3).tolist(),
                      opens_along=np.round(u, 3).tolist(), direction_from=how)

        approach = grip + u * STANDOFF_M
        ctx.robot.set_gripper(side, 0.08)
        ctx.robot.move_pose(side, approach + np.array([0.0, 0.0, APPROACH_LIFT_M]), R, seconds=4.0)
        ctx.robot.move_pose(side, approach, R, seconds=2.5)
        ctx.robot.move_pose(side, grip + u * GRIP_DEPTH_M, R, seconds=2.0)
        ctx.robot.set_gripper(side, 0.0)
        ctx.robot.move_pose(side, grip + u * float(distance), R, seconds=4.0)
        ctx.robot.set_gripper(side, 0.08)
        # Back off along the same axis so the open gripper does not sweep the
        # drawer shut on the way out.
        ctx.robot.move_pose(side, grip + u * (float(distance) + 0.06), R, seconds=1.5)
        ctx.robot.settle(0.5)

        found, note = ground_entity(ctx, entity)
        now = ctx.belief.track(entity).xyz_base
        if not found or now is None:
            notes.append(f"attempt {attempt + 1}: lost sight of it ({note})")
            break
        # A drawer front travels along its rail and nowhere else. Anything that
        # has also moved sideways or changed height is a different object, and
        # believing it is how one episode reported 179 mm of travel on a drawer
        # that had moved 15 mm — the detector had latched onto the handle above.
        # The VQA verifier caught that one; this stops it being claimed at all.
        step = np.asarray(now, float) - np.asarray(
            ctx.belief.track(entity).first_xyz or before, float)
        along = float(step[:2] @ u[:2])
        sideways = float(np.linalg.norm(step[:2] - u[:2] * along))
        if sideways > OFF_RAIL_MAX_M or abs(float(step[2])) > OFF_RAIL_MAX_M:
            ctx.log.event("open_drawer_sighting_rejected", entity=entity,
                          attempt=attempt + 1, along_mm=round(along * 1000, 1),
                          sideways_mm=round(sideways * 1000, 1),
                          dz_mm=round(float(step[2]) * 1000, 1))
            notes.append(f"attempt {attempt + 1}: the thing it found had moved "
                         f"{sideways * 1000:.0f} mm sideways and "
                         f"{abs(step[2]) * 1000:.0f} mm vertically — a drawer does "
                         "neither, so that is a different object")
            break
        gained = along - travelled
        travelled += gained
        notes.append(f"attempt {attempt + 1}: +{gained * 1000:.0f} mm "
                     f"(total {travelled * 1000:.0f} mm)")
        ctx.log.event("open_drawer_progress", entity=entity, attempt=attempt + 1,
                      gained_mm=round(gained * 1000, 1),
                      total_mm=round(travelled * 1000, 1))
        if travelled >= OPEN_MIN_TRAVEL_M:
            break
        if gained < PULL_STALL_M:
            # It stopped moving. Either it is against its stop or the grip is
            # missing entirely, and another identical pull settles neither.
            notes.append("it stopped moving; further identical pulls would tell "
                         "us nothing new")
            break

    ctx.log.event("open_drawer_done", entity=entity,
                  travelled_mm=round(travelled * 1000, 1), attempts=len(notes))
    return SkillResult(
        ok=travelled > PULL_STALL_M,
        info=f"pulled {entity} {travelled * 1000:.0f} mm open; " + "; ".join(notes),
        error=None if travelled > PULL_STALL_M else
        f"{entity} did not move: " + "; ".join(notes))


@skill(
    name="close_drawer",
    description="Push a drawer shut. The inverse of open_drawer, same arguments.",
    params={
        "entity": {"type": "string"},
        "cabinet": {"type": "string"},
        "distance": {"type": "number"},
        "arm": {"type": "string", "enum": ["left", "right", "arm", "auto"]},
    },
    required=["entity"],
    post_hint="not open(entity) — the drawer front is back against the cabinet",
)
def close_drawer(ctx: SkillContext, entity: str, cabinet: Optional[str] = None,
                 distance: float = PULL_M, arm: str = "auto") -> SkillResult:
    from .primitives import choose_arm  # noqa: PLC0415

    if not getattr(ctx.robot, "can_orient", lambda: False)():
        return SkillResult(ok=False, error="this robot cannot command wrist orientation")
    handle = ctx.belief.track(entity)
    if handle.xyz_base is None:
        return SkillResult(ok=False, error=f"close_drawer: perceive {entity!r} first")
    h = np.asarray(handle.xyz_base, float)
    u, how, surface = _pull_direction(ctx, entity, cabinet, h)
    if u is None:
        return SkillResult(ok=False, error=f"close_drawer: cannot tell which way it faces — {how}")
    if surface is not None:
        h = np.asarray(surface, float)
    R = facing_rotation(u)
    side = choose_arm(ctx, h, arm)

    # Closed fingers pushing the face flat: no grasp is needed to shut a drawer,
    # and reaching for the handle only adds a way to miss.
    ctx.robot.set_gripper(side, 0.0)
    ctx.robot.move_pose(side, h + u * (STANDOFF_M + 0.04) + np.array([0.0, 0.0, APPROACH_LIFT_M]),
                        R, seconds=4.0)
    ctx.robot.move_pose(side, h + u * (STANDOFF_M + 0.04), R, seconds=2.0)
    ctx.robot.move_pose(side, h - u * float(distance), R, seconds=4.0)
    ctx.robot.move_pose(side, h - u * float(distance) + np.array([0.0, 0.0, APPROACH_LIFT_M]),
                        R, seconds=1.5)
    ctx.robot.settle(0.5)
    ctx.belief.invalidate_entity(entity, "pushed shut; its position needs re-grounding")
    ctx.log.event("close_drawer_done", entity=entity, arm=side)
    return SkillResult(ok=True, info=f"pushed {entity} shut along {np.round(-u, 2).tolist()}")
