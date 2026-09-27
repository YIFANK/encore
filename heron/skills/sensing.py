"""Sensing skills: information gathering is an action, not a side effect.

The planner inserts these when preconditions are UNKNOWN (Seeing-is-Believing
made explicit): `perceive` grounds entities in fixed cameras; `inspect` moves a
wrist camera over a region for a close look — the stationary kit's version of
an active viewpoint change.
"""
from __future__ import annotations

import json
import numpy as np

from ..types import Frame, SkillResult, Value, reprojection_error, triangulate
from . import SkillContext, skill

INSPECT_HEIGHT = 0.18
# How close to the tool a detection has to be before it IS the tool. Generous
# enough to cover the gripper and its mount, tight enough that an object the arm
# is reaching toward is not swallowed: the closest a free object gets to the
# tool centre while the arm hovers over it is HOVER_CLEARANCE, 0.10 m.
ROBOT_EXCLUSION_M = 0.07
# Where an arm goes to stop blocking the fixed cameras (matches the agent's
# goal-check park).
VIEWING_PARK_XYZ = (0.16, -0.26, 0.14)
INSPECT_ABOVE_M = 0.15  # how far above the subject the wrist camera hovers
# ...and how far above it the camera must STILL be for the look to be worth
# making. The D405 focuses from 70 mm; closer than this the wrist is about to
# touch what it came to look at, and lowering further to find a reachable pose
# would trade a refused motion for a collision.
INSPECT_MIN_ABOVE_M = 0.07
# How near the camera-to-object line the tool has to be before it is in the way.
# The gripper and its mount are roughly this wide, and the overhead camera is
# far enough away that the shadow it casts on the table is about the same.
VIEW_BLOCK_RADIUS_M = 0.08
# How far to step aside. Comfortably clear of the block radius, small enough to
# stay inside the arm's envelope from wherever it happened to be standing.
VIEW_CLEAR_OFFSET_M = 0.16
# How far above the table a pixel has to sit before the depth-carved mask calls
# it object rather than table. Above the sidecar's depth noise, below the
# thinnest thing we place (a plate's dish is 8 mm).
DEPTH_MASK_MIN_HEIGHT_M = 0.004
# Half-width of the height band kept around the modal height. Wide enough to
# hold one object's own relief (a tilted block, a dish's rim), narrow enough to
# exclude a 22 mm step to whatever sits on or under it.
DEPTH_MASK_BAND_M = 0.008
# How far the tool may have drifted before a look stops being a picture of now.
# Below the arm's own tracking error, so a settle or a servo twitch does not
# throw away a good frame, and far below any deliberate motion.
LOOK_REUSE_TOLERANCE_M = 0.002


TRIANGULATION_MAX_REPROJ_PX = 25.0
# How far apart two views have to look, measured as the angle the object sees
# between them. Below this the depth is not being solved so much as guessed
# along the shared line of sight. The twin's overhead and low-front pair subtend
# tens of degrees; two nearly-coincident downward cameras subtend about four.
TRIANGULATION_MIN_PARALLAX_DEG = 12.0
# Re-identification guard: how far an object may appear to jump, and for how
# long after a belief was formed, before the sighting is treated as a mix-up.
REID_MAX_JUMP_M = 0.25
# A position OUR OWN pick/place wrote obeys different physics: nothing in this
# cell moves by itself, so a detection far from where we put it is a lookalike.
# 60 mm covers settle/bounce and calibration error; 135 mm to the twin red
# block sailed under the 0.25 sighting limit and stole a placed identity.
FROM_ACTION_MAX_JUMP_M = 0.06
# The same guard, for a look we already know was obstructed. Set above the
# grounding error this rig measures (11 mm from a mask, 23 mm from a bare pixel)
# and well below the separation the scene guarantees between two objects
# (100 mm), so it separates "the same object, measured noisily" from "a
# different object that answers the same description".
BLOCKED_MAX_JUMP_M = 0.05
REID_TRUST_WINDOW_S = 30.0
# A sighting this fresh is the SAME observation, not an old belief being
# trusted: perceive grounds an object and pick verifies it a tenth of a second
# later, which cost four model calls and a segmentation — 47 s — to re-derive a
# position that had not had time to change. Deliberately short; anything longer
# is a question about staleness, which belongs to budgets.verify_max_age_s.
REGROUND_CACHE_S = 3.0


# Bottom-up identification: how many proposals we are willing to ask about, and
# what counts as an object rather than the table, a wall, or the arm.
IDENTIFY_MAX_CHECKS = 6
IDENTIFY_MIN_CONF = 0.6
IDENTIFY_MIN_FOOTPRINT_M = 0.015
IDENTIFY_MAX_FOOTPRINT_M = 0.40
# Widest thing this arm could pick up, used to choose among SAM2's part/object/
# group readings. Deliberately generous: the point is to reject "the whole
# table", not to second-guess the grasp planner.
GRASPABLE_MAX_SPAN_M = 0.30
IDENTIFY_CROP_MARGIN_PX = 12
# A tabletop object is tens of pixels across in the overhead frame, and handing
# ER a 36x52 crop got back "the low resolution makes the label unreadable" on
# every candidate — the identification never failed, it never got to happen.
# Upscale so the model has something to read.
IDENTIFY_CROP_MIN_PX = 256


def _usable_cameras(ctx: SkillContext) -> list[str]:
    """Fixed cameras that can actually yield a 3D position.

    Every camera in this list costs a detection call, and on this rig cam_low
    has neither extrinsics nor a homography — so it was queried on every
    grounding, doubling the model calls, and could never contribute a position.
    A camera earns its API call by being calibrated.

    Two things this used to get wrong, both of which mattered once a second
    viewpoint existed:

    The candidate list was the literal pair ("cam_high", "cam_low"), so a rig
    that grew a third fixed camera would simply not be asked. It is now every
    camera the backend has that is not on an arm. Wrist cameras stay out
    deliberately — they are reachable only by flying the arm there, which is a
    motion and a decision, not a look.

    And "calibrated" was inferred from whether the CONFIG named a file. That is
    a proxy for the real question, and it is wrong in the twin, where the pose
    comes from the model and no file exists or should. Ask the backend; fall
    back to the filename when it cannot answer, so the real rig is unchanged.
    """
    posed = getattr(ctx.robot, "camera_calibrated", None)
    out, fixed = [], []
    for name in ctx.robot.cameras:
        cam = ctx.cfg.cameras.get(name)
        if name.endswith("_wrist") or (cam is not None and cam.kind == "wrist"):
            continue
        fixed.append(name)
        ok = None
        if callable(posed):
            try:
                ok = bool(posed(name))
            except Exception:
                ok = None      # the backend cannot answer; that is not a "no"
        if ok is None:
            ok = True if cam is None else bool(
                cam.extrinsics_file or cam.homography_file
                or getattr(cam, "proj_file", None))
        if ok:
            out.append(name)
        else:
            ctx.log.event("camera_skipped", camera=name,
                          reason="no extrinsics, homography or projection matrix")
    return out or fixed


def _upscale(crop: np.ndarray) -> np.ndarray:
    """Enlarge a small crop so a vision model has pixels to work with.

    Nearest-neighbour on purpose. There is no information to recover here, and
    an interpolating resize would invent smooth edges and plausible-looking
    texture — exactly the kind of detail the model is being asked to judge.
    """
    h, w = crop.shape[:2]
    factor = int(np.ceil(IDENTIFY_CROP_MIN_PX / max(1, min(h, w))))
    if factor <= 1:
        return np.ascontiguousarray(crop)
    return np.ascontiguousarray(np.kron(crop, np.ones((factor, factor, 1), np.uint8)))


def _plausible_object(ctx: SkillContext, frame, box) -> tuple[bool, str]:
    """Is this proposed region a graspable object rather than scenery?

    Without depth we can only judge in pixels, which is why this stays lenient
    there: refusing a real object costs a failed task, admitting scenery costs
    one VQA call.
    """
    u0, v0, u1, v1 = box
    if u1 - u0 < 6 or v1 - v0 < 6:
        return False, "smaller than 6 px"
    h, w = frame.rgb.shape[:2]
    if (u1 - u0) * (v1 - v0) > 0.5 * h * w:
        return False, "covers half the image — scenery, not an object"
    if frame.depth is None or frame.t_base_cam is None:
        return True, ""
    lo = frame.deproject(u0, v1)   # bottom-left and bottom-right of the region,
    hi = frame.deproject(u1, v1)   # i.e. where it meets the surface
    if lo is None or hi is None:
        return True, ""
    span = float(np.linalg.norm(np.asarray(hi[:2]) - np.asarray(lo[:2])))
    if span < IDENTIFY_MIN_FOOTPRINT_M:
        return False, f"footprint {span * 1000:.0f} mm — too small to grasp"
    if span > IDENTIFY_MAX_FOOTPRINT_M:
        return False, f"footprint {span * 1000:.0f} mm — furniture, not an object"
    return True, ""


def identify_by_parts(ctx: SkillContext, frame, description: str, prior_xy=None):
    """Find an object by segmenting everything, then asking about each region.

    The top-down path asks a detector to find a named thing, and when the
    detector does not recognise the thing it returns nothing at all — measured:
    ER-2 gives zero boxes for a 22x27 px stick of butter that is plainly
    visible. Bottom-up inverts the dependency. SAM2 needs no vocabulary to find
    a region, so recognition is reduced to "is THIS a stick of butter?" over a
    tight crop, which is both easier and checkable.

    Candidates are ranked before being asked about, so the usual case costs one
    or two VQA calls rather than one per region. Returns (mask, box, conf) or
    (None, None, 0.0).
    """
    seg = getattr(ctx, "segmenter", None)
    if seg is None or not getattr(seg, "enabled", False):
        return None, None, 0.0
    proposals = seg.propose(frame.rgb)
    if not proposals:
        return None, None, 0.0

    kept, dropped = [], []
    for p in proposals:
        ok, why = _plausible_object(ctx, frame, p["box"])
        (kept if ok else dropped).append((p, why))
    if not kept:
        ctx.log.event("identify_no_candidates", description=description,
                      proposals=len(proposals),
                      example_reason=dropped[0][1] if dropped else "")
        return None, None, 0.0

    def rank(item):
        p = item[0]
        if prior_xy is not None:
            u0, v0, u1, v1 = p["box"]
            world = frame.deproject((u0 + u1) // 2, (v0 + v1) // 2)
            if world is not None:
                return float(np.linalg.norm(np.asarray(world[:2]) - prior_xy))
        return -float(p["score"])  # nothing better to go on: trust SAM's own score

    kept.sort(key=rank)
    ctx.log.event("identify_candidates", description=description,
                  proposals=len(proposals), plausible=len(kept),
                  asking=min(len(kept), IDENTIFY_MAX_CHECKS))

    # ONE question about all the crops, not one question per crop.
    #
    # This loop used to ask "is this it?" once per candidate, up to six times,
    # stopping at the first yes. Measured over a 60-episode LIBERO-PRO sweep:
    # 129 invocations, 766 VQA calls, 12 successes. Seventy-nine per cent of the
    # run's entire VQA budget — 2088 of 2636 seconds — went into a path that
    # found the object 9% of the time, because a miss costs the full six calls
    # and a miss is the usual outcome.
    #
    # The model reads every image it is given in one request either way, so
    # showing it all the candidates at once asks exactly the same question for
    # a sixth of the calls. It also asks a BETTER question: "which of these" lets
    # the model compare the candidates against each other, where "is this it?"
    # six times over cannot.
    h, w = frame.rgb.shape[:2]
    crops, boxes = [], []
    for p, _ in kept[:IDENTIFY_MAX_CHECKS]:
        u0, v0, u1, v1 = p["box"]
        m = IDENTIFY_CROP_MARGIN_PX
        crop = frame.rgb[max(0, v0 - m):min(h, v1 + m + 1),
                         max(0, u0 - m):min(w, u1 + m + 1)]
        if crop.size == 0:
            continue
        crops.append(Frame(camera=f"{frame.camera}:crop{len(crops) + 1}",
                           rgb=_upscale(crop)))
        boxes.append(p)
    if not crops:
        return None, None, 0.0

    picked, conf, reason = _which_crop(ctx, crops, description)
    ctx.log.event("identify_check", description=description, candidates=len(crops),
                  picked=picked, confidence=round(float(conf), 2), reason=reason[:120])
    if picked is None or conf < IDENTIFY_MIN_CONF:
        return None, None, 0.0
    p = boxes[picked]
    return p["mask"], p["box"], float(conf)


def _which_crop(ctx: SkillContext, crops: list, description: str):
    """Which of these images shows the thing, or None. (index, confidence, why).

    A single question over every candidate. The answer is an INDEX rather than a
    yes/no, so "none of them" stays expressible — a bottom-up search that cannot
    come back empty would bind the entity to whichever region looked least
    unlike it, which is worse than not finding it.
    """
    pick = getattr(ctx.grounder, "choose", None)
    if callable(pick):
        try:
            return pick(crops, description)
        except Exception as e:
            return None, 0.0, f"{type(e).__name__}: {e}"

    # A grounder without `choose` keeps the old behaviour: ask about each crop
    # until one says yes. Correct, just six times the calls.
    for i, sub in enumerate(crops):
        try:
            value, conf, reason = ctx.grounder.vqa(
                [sub], f"Is the object in the centre of this image {description}?")
        except Exception as e:
            return None, 0.0, f"{type(e).__name__}: {e}"
        if value is Value.TRUE and conf >= IDENTIFY_MIN_CONF:
            return i, float(conf), str(reason)
    return None, 0.0, "none of the candidates matched"


def _by_physical_size(ctx: SkillContext, frame, cands):
    """Pick among SAM2's part / object / group readings using metres.

    The segmenter cannot know which reading is meant and its own score does not
    settle it — pointed at a laptop it ranked the KEYBOARD first. But we have the
    camera calibration, so we can ask how wide each candidate would actually be
    and keep the one that is the size of a thing this arm could pick up. Falls
    back to SAM's order when the frame is uncalibrated, which is no worse than
    before.
    """
    best, best_penalty = None, None
    sizes = []
    for c in cands:
        vs, us = np.where(c["mask"])
        if us.size == 0:
            continue
        left = frame.deproject(int(us.min()), int(vs.max()))
        right = frame.deproject(int(us.max()), int(vs.max()))
        if left is None or right is None:
            return cands[0]["mask"]
        span = float(np.linalg.norm(np.asarray(right[:2]) - np.asarray(left[:2])))
        sizes.append(round(span, 3))
        # Distance outside the graspable band, in metres; zero inside it, so
        # anything plausible wins on SAM's own ordering and only implausible
        # candidates get ranked against each other.
        penalty = max(0.0, IDENTIFY_MIN_FOOTPRINT_M - span,
                      span - GRASPABLE_MAX_SPAN_M)
        if best_penalty is None or penalty < best_penalty:
            best, best_penalty = c, penalty
    if best is None:
        return cands[0]["mask"]
    ctx.log.event("sam_candidate_by_size", spans_m=sizes,
                  chose_px=int(best["pixels"]), chose_score=round(best["score"], 3),
                  penalty_m=round(best_penalty, 3))
    return best["mask"]


def _mask_from_depth(frame, bb, table_z: float):
    """A mask carved out of the depth image: the tallest thing in the box.

    THE CAMERA ALREADY KNOWS WHERE THE OBJECT ENDS. We carry dense depth on
    every calibrated frame and were using one pixel of it — the box centroid —
    which is why supports grounded 5-16 mm off while blocks managed 2 mm: a
    plate's box centroid and its body centre are different points, and the box
    catches table on every side. TiPToP gets its precision by intersecting a
    SAM2 mask with the point cloud; we do not keep a SAM server warm, but for
    the question "which pixels in this box are the object" the depth image IS a
    mask. Anything meaningfully above the table is object; of that, keep the
    upper half of the height range, which drops a neighbouring support seen
    through the box's corners (a plate at 8 mm under a block at 30 mm) without
    knowing anything about either.

    Returns an HxW boolean or None when depth is missing or the box holds
    nothing tall enough — in which case the caller's box path stands unchanged.
    """
    if frame is None or frame.depth is None or frame.intrinsics is None \
            or frame.t_base_cam is None or bb is None:
        return None
    u0, v0, u1, v1 = (int(v) for v in bb[:4])
    h, w = frame.depth.shape[:2]
    u0, v0 = max(0, u0), max(0, v0)
    u1, v1 = min(w - 1, u1), min(h - 1, v1)
    if u1 - u0 < 3 or v1 - v0 < 3:
        return None
    z = frame.depth[v0:v1 + 1, u0:u1 + 1].astype(float)
    ok = np.isfinite(z) & (z > 0.03)
    if ok.sum() < 24:
        return None
    fx, fy = frame.intrinsics[0, 0], frame.intrinsics[1, 1]
    cx, cy = frame.intrinsics[0, 2], frame.intrinsics[1, 2]
    vs, us = np.mgrid[v0:v1 + 1, u0:u1 + 1]
    zz = np.where(ok, z, np.nan)
    pts = np.stack([(us - cx) * zz / fx, (vs - cy) * zz / fy, zz,
                    np.ones_like(zz)], axis=0).reshape(4, -1)
    height = (np.asarray(frame.t_base_cam, dtype=float) @ pts)[2].reshape(z.shape) - table_z
    above = ok & (height > DEPTH_MASK_MIN_HEIGHT_M)
    if above.sum() < 24:
        return None
    # THE OBJECT IS THE DOMINANT HEIGHT, NOT THE TALLEST. The first version kept
    # the upper half of the height range, which is right for a block and exactly
    # backwards for a support with a block sitting on it: it kept the block and
    # located the plate at the block. Measured — supports' median fell 5.7 mm to
    # 1.9 mm but the tail grew a 208 mm blunder. The pixels that ARE the object
    # are the ones there are most of, so keep the band around the modal height:
    # a block's box is mostly block top, a plate's box is mostly plate, and the
    # minority (table sliver, cargo, a neighbour's edge) falls outside the band
    # either way.
    h_mid = float(np.nanmedian(height[above]))
    band = max(DEPTH_MASK_BAND_M, 0.25 * h_mid)
    keep = above & (np.abs(height - h_mid) < band)
    if keep.sum() < 24:
        return None
    mask = np.zeros((h, w), dtype=bool)
    mask[v0:v1 + 1, u0:u1 + 1] = keep
    return mask


def _mask_for(ctx: SkillContext, frame, bb, point_px):
    """SAM2 mask for one object, or None if masks are unavailable.

    Both prompts are passed when both exist: the box gives the extent, the point
    says which object is meant when the box holds two touching ones. Either
    alone is enough, which is what makes this work on the real rig — there the
    detector points reliably and boxes almost nothing.
    """
    seg = getattr(ctx, "segmenter", None)
    if seg is None or not getattr(seg, "enabled", False):
        return None
    if bb is None and point_px is None:
        return None
    try:
        cands = seg.candidates(frame.rgb, bb[:4] if bb is not None else None,
                               point=point_px)
        if not cands:
            return None
        if len(cands) == 1 or bb is not None:
            return cands[0]["mask"]
        return _by_physical_size(ctx, frame, cands)
    except Exception as e:
        # Segmentation is an optimisation. Nothing it does should be able to
        # stop the robot from grounding an object.
        ctx.log.event("sam_call_failed", error=f"{type(e).__name__}: {e}")
        return None


def _box_contains(b, u: int, v: int, slack: float = 0.25) -> bool:
    """Does this box belong to the object at (u, v)?

    Generous by design. Point and box come from the same detector but not from
    the same head, and on a 40-pixel object they disagree by a few pixels — a
    strict test would then reject the correct box. The question being asked is
    "same object or a different one", where the alternatives are tens of pixels
    apart, so a quarter of the box's own size is the right order of tolerance.
    """
    x0, y0, x1, y1 = float(b[0]), float(b[1]), float(b[2]), float(b[3])
    mx, my = slack * abs(x1 - x0), slack * abs(y1 - y0)
    return (min(x0, x1) - mx <= u <= max(x0, x1) + mx
            and min(y0, y1) - my <= v <= max(y0, y1) + my)


def _pick_box(ctx: SkillContext, frame, description: str, point_px, prior_xy, entity_id: str):
    """The box that belongs to THIS object, among any lookalikes.

    Position is derived from the box, so associating only the point would leave
    the mistake in place. Prefer the box containing the point we already
    associated; failing that, the one nearest what we believed.
    """
    try:
        get_all = getattr(ctx.grounder, "boxes", None)
        candidates = list(get_all(frame, description)) if callable(get_all) else []
        if not candidates:
            one = ctx.grounder.box(frame, description)
            candidates = [one] if one else []
    except Exception:
        return None
    if not candidates:
        return None
    u, v = point_px
    if len(candidates) == 1:
        # A lone box is not automatically THIS object's box. The detector often
        # points at every instance and boxes only one, and taking that box on
        # trust silently relocates the entity onto whichever lookalike got
        # boxed — undoing the association the point had already made correctly.
        if _box_contains(candidates[0], u, v):
            return candidates[0]
        ctx.log.event("box_rejected", entity=entity_id, box=[int(x) for x in candidates[0][:4]],
                      point=[int(u), int(v)],
                      reason="the only box offered does not contain the associated point")
        return None
    containing = [b for b in candidates if _box_contains(b, u, v)]
    if len(containing) == 1:
        return containing[0]
    pool = containing or candidates
    if prior_xy is not None:
        scored = []
        for b in pool:
            centre = frame.deproject((b[0] + b[2]) // 2, (b[1] + b[3]) // 2)
            d = float(np.linalg.norm(np.asarray(centre[:2]) - prior_xy)) if centre is not None else 1e3
            scored.append((d, b))
        scored.sort(key=lambda x: x[0])
        if scored[0][0] < 1e3:
            if scored[0][1] is not candidates[0]:
                ctx.log.event("reid_box_reassociated", entity=entity_id,
                              candidates=len(candidates), distance_m=round(scored[0][0], 3))
            return scored[0][1]
    return pool[0]


def _generic(description: str) -> str | None:
    """The same object described in the way its lookalikes share.

    "the metal bowl with a gold rim" -> "the bowl". Reuses the head-noun rule the
    cross-episode memory already needed, for the same underlying reason: a model
    re-invents distinguishing detail every time it is asked for a visual phrase,
    and the detail is often not in the picture.
    """
    from ..memory import _head  # noqa: PLC0415

    head = _head(description)
    return f"the {head}" if head else None


def _choose_by_relation(ctx: SkillContext, entity_id: str, rel, frame, candidates):
    """Pick the candidate a spatial relation names, or None if it cannot decide.

    Deliberately scored from the raw detector point rather than the box-and-mask
    centroid used for the final position. A single pixel is worth about 22 mm
    against the mask's 11 mm, and the gaps this has to resolve are the distances
    between different objects — on these scenes, 150 mm and up. Paying two extra
    deprojections per candidate to sharpen a decision that is not close would be
    a waste; the winner is re-deprojected properly below either way.
    """
    from . import relations  # noqa: PLC0415

    bad = relations.validate(rel)
    if bad:
        ctx.log.event("relation_unusable", entity=entity_id, relation=str(rel), reason=bad)
        return None

    anchors = []
    for anchor_id in rel.of:
        a = ctx.belief.track(anchor_id)
        if a.xyz_base is None:
            ctx.log.event("relation_unusable", entity=entity_id, relation=str(rel),
                          reason=f"anchor {anchor_id!r} has no position yet")
            return None
        anchors.append(np.asarray(a.xyz_base, dtype=float))

    pts, keep = [], []
    for cand in candidates:
        u, v, _conf = cand
        p = frame.deproject(int(u), int(v))
        if p is None:
            continue
        pts.append(np.asarray(p, dtype=float))
        keep.append(cand)
    if len(keep) < 2:
        ctx.log.event("relation_unusable", entity=entity_id, relation=str(rel),
                      reason=f"only {len(keep)} candidate(s) could be deprojected")
        return None

    choice = relations.choose(rel, pts, anchors)
    ctx.log.event("relation_resolved", entity=entity_id, relation=str(rel),
                  candidates=len(keep), chose_px=[int(keep[choice.index][0]),
                                                  int(keep[choice.index][1])],
                  top_px=[int(candidates[0][0]), int(candidates[0][1])],
                  chose_xyz=np.round(pts[choice.index], 3).tolist(),
                  margin_m=None if choice.margin is None else round(choice.margin, 3),
                  decisive=choice.decisive, note=choice.note)
    if not choice.decisive:
        # Still the best answer available, and still acted on — but a 1 cm margin
        # between two bowls means the geometry did not really separate them, and
        # a later failure should be able to read that here rather than infer it.
        ctx.log.event("relation_close_call", entity=entity_id, relation=str(rel),
                      margin_m=round(choice.margin, 3))
    return keep[choice.index]


def relational_order(ctx: SkillContext, entity_ids: list[str]) -> list[str]:
    """Ground anchors before the entities whose identity depends on them.

    Without this the order is whatever the planner listed, and "the bowl between
    the plate and the ramekin" gets resolved against two anchors that have no
    positions yet — which reads, in the journal, exactly like having no relation
    at all.
    """
    # An anchor the planner forgot to perceive is the same defect as no relation
    # at all, and it is one extra detection to prevent. Pull them in rather than
    # discovering at grounding time that the relation cannot be evaluated.
    entity_ids = list(entity_ids)
    for eid in list(entity_ids):
        rel = ctx.belief.track(eid).relation
        for anchor in (rel.of if rel else []):
            if anchor not in entity_ids:
                entity_ids.append(anchor)
                ctx.log.event("anchor_added", entity=eid, anchor=anchor,
                              reason="named by a spatial relation but not in the perceive list")

    remaining = list(entity_ids)
    placed: list[str] = []
    seen: set[str] = set()
    while remaining:
        progressed = False
        for eid in list(remaining):
            rel = ctx.belief.track(eid).relation
            deps = [a for a in (rel.of if rel else []) if a in entity_ids and a != eid]
            if all(d in seen for d in deps):
                placed.append(eid)
                seen.add(eid)
                remaining.remove(eid)
                progressed = True
        if not progressed:
            # A cycle ("A near B", "B near A"). Neither can be resolved by the
            # other, so ground them in the order given and let the relation fall
            # back to the detector's own ranking, with the reason on record.
            ctx.log.event("relation_cycle", entities=list(remaining))
            placed.extend(remaining)
            break
    return placed


def _span_from_box(frame, bb) -> float | None:
    """How wide the boxed object is on the table, in metres.

    Measured across the box's BOTTOM edge, which is where the object meets the
    support; the top edge of a tall object deprojects onto whatever is behind it.
    """
    if bb is None:
        return None
    try:
        p0 = frame.deproject(bb[0], bb[3])
        p1 = frame.deproject(bb[2], bb[3])
    except Exception:
        return None
    if p0 is None or p1 is None:
        return None
    return float(np.linalg.norm(np.asarray(p1[:2]) - np.asarray(p0[:2])))


def _is_the_robot(ctx: SkillContext, p, entity_id: str) -> str | None:
    """The arm that this detection actually landed on, if it landed on one.

    The robot is the largest moving thing in its own cameras' view and it is
    coloured like the furniture — this rig's arm is white and so is the plate,
    its tray is grey and so are the links. Detectors put objects on it, and
    nothing downstream questions the result:

      * 21 of 330 groundings in a lifelong run placed a table object between
        0.17 and 0.21 m above a table at -0.015 — thirteen of them the grey tray;
      * one of those became a FALSE SUCCESS. `in(red_block, grey_tray)` verified
        true "by an active close-range look" while the block was 98 mm from the
        tray, because the tray had been grounded onto the arm at z=0.143, the
        wrist was then aimed THERE, and the block was measured relative to the
        same wrong point. Two errors in the same direction cancel, and the test
        that was supposed to catch the placement confirmed it instead.

    A height band cannot separate these — the arm spends most of its time at
    object height. What can is that we know exactly where the arm is: it is the
    one object in the scene whose position needs no perception at all.

    Held objects are exempt: an object in the gripper is supposed to be there.

    Tests against the WHOLE arm skeleton where the backend can produce one
    (arm_polyline: joint origins + tool), not just the tool point — the
    collect-loop's phantom "red block" at (0.452, 0.162) was the FOREARM in
    frame while the TCP was parked 20 cm away, and a tool-point sphere is
    blind to every link behind the wrist.
    """
    tr = ctx.belief.track(entity_id) if ctx.belief is not None else None
    held_by = getattr(tr, "held_by", None)
    point = np.asarray(p, dtype=float)[:3]
    inner = getattr(ctx.robot, "_robot", ctx.robot)
    poly_fn = getattr(inner, "arm_polyline", None)
    for arm in getattr(ctx.robot, "arms", []):
        if arm == held_by:
            continue
        try:
            if callable(poly_fn):
                if _dist_to_polyline(point, np.asarray(poly_fn(arm), dtype=float)) \
                        < ROBOT_EXCLUSION_M:
                    return arm
                continue
            tool = np.asarray(ctx.robot.get_cartesian(arm), dtype=float)[:3]
        except Exception:
            continue
        if float(np.linalg.norm(point - tool)) < ROBOT_EXCLUSION_M:
            return arm
    return None


def _dist_to_polyline(point: np.ndarray, pts: np.ndarray) -> float:
    """Shortest distance from `point` to the polyline through `pts`."""
    best = float("inf")
    for a, b in zip(pts[:-1], pts[1:]):
        ab = b - a
        denom = float(ab @ ab)
        t = 0.0 if denom < 1e-12 else float(np.clip((point - a) @ ab / denom, 0.0, 1.0))
        best = min(best, float(np.linalg.norm(point - (a + t * ab))))
    return best


def _parallax_deg(hits) -> float:
    """The widest angle any two of these views subtend at what they are looking at.

    Approximated at the cameras' own optical centres rather than at the object,
    which needs no depth to compute and is what makes it usable BEFORE the
    triangulation it is deciding about.
    """
    centres = []
    for h in hits:
        t = getattr(h[3], "t_base_cam", None)
        if t is not None:
            centres.append(np.asarray(t, dtype=float)[:3, 3])
    best = 0.0
    for i, a in enumerate(centres):
        for b in centres[i + 1:]:
            u, v = a, b
            na, nb = np.linalg.norm(u), np.linalg.norm(v)
            if na < 1e-9 or nb < 1e-9:
                continue
            gap = float(np.linalg.norm(u - v))
            best = max(best, np.degrees(2 * np.arcsin(min(1.0, gap / (na + nb)))))
    return best


def expected_xy(tr) -> np.ndarray | None:
    """Where this entity should be, when that is a claim worth defending.

    Two kinds of belief live in the same field and they do not decay alike. A
    SIGHTING goes stale: the world may have moved on while nobody was looking,
    so after REID_TRUST_WINDOW_S it stops being a reason to prefer one lookalike
    over another. A position written by our own `pick` or `place` does NOT go
    stale. Nothing in this cell moves on its own, and every skill that touches an
    entity either rewrites its track or invalidates it — so continuity of
    identity through a grasp is a fact about what we did, not an observation that
    ages out.

    Treating the two alike is what made a scene with duplicates unverifiable.
    Measured in the twin, on `put the red block on the white plate` with two red
    blocks: the block was placed 18 mm from the plate, the arm that had just put
    it there stood over the overhead view, and thirty seconds later the guard
    expired and `red_block` silently rebound to the OTHER red block 0.34 m away.
    The episode could then never be verified — and since only verified episodes
    teach, the skill library stayed permanently empty.

    Returns None when there is nothing to defend: no position at all, or a belief
    something has explicitly invalidated (a refuted grasp, a push, an
    open_gripper). That is the escape hatch — an object that really has been
    displaced is found wherever it now is, because the skill that displaced it
    said so.
    """
    if tr.xyz_base is None or tr.confidence <= 0.0:
        return None
    if tr.from_sighting and tr.age() >= REID_TRUST_WINDOW_S:
        return None
    return np.asarray(tr.xyz_base[:2], dtype=float)


def refute_expectation(ctx: SkillContext, entity_id: str, note: str = "") -> None:
    """A deliberate look at where we put something, that saw nothing, refutes it.

    Dead reckoning from our own `pick` or `place` does not decay with time — see
    `expected_xy` — but it is not immune to evidence. Without this the identity
    guard, which is right to refuse rebinding a placed block onto its twin,
    would refuse FOREVER: an object that really did end up somewhere else could
    never be found again, and the entity would be stranded for the rest of the
    episode.

    Pointing a camera at the expectation and not finding it is exactly the
    evidence that settles that, and it costs nothing extra — the caller has
    already paid for the look. Invalidating here releases the identity through
    the escape hatch `expected_xy` already honours, so the next grounding is
    free to bind wherever the object now is.
    """
    tr = ctx.belief.track(entity_id)
    if tr.xyz_base is None or tr.from_sighting or tr.confidence <= 0.0:
        return  # nothing dead-reckoned to refute
    ctx.log.event("expectation_refuted", entity=entity_id,
                  at=[round(float(v), 3) for v in tr.xyz_base[:2]], note=note[:160])
    ctx.belief.invalidate_entity(
        entity_id, reason="looked where we put it and it was not there")



# HOW MANY TIMES MAY WE ASK ABOUT ONE OBJECT IN ONE LOOK?
# Measured, rig 2026-08-06, one episode: 54 model calls, of which the happy
# path needs two — locate_many for every entity at once, then proposal_match
# to put names on the segments. The other fifty were the same question asked
# in a different voice after an empty answer: locate, point, point, locate,
# box, box, crop_recheck, box, box, crop_recheck... twenty of the responses
# were literally "[]". A detector that says nothing three times is not going
# to say something on the seventh; what changes an empty answer is a
# different VIEW, not a different phrasing, and that is what the step-aside
# is for. So: three calls per entity per look, then report it unseen.
GROUNDING_CALL_BUDGET = 3


def _calls_spent(ctx: SkillContext) -> int:
    return int(getattr(ctx.grounder, "calls", 0) or 0)


def _planar_parallax_correct(frame, xy_z, h: float = 0.022):
    """Planar deprojection maps an object's TOP pixel onto the table plane —
    displaced away from the camera by h * horizontal_offset / camera_height
    (rig LAWS L7: 5-15 mm across this table for a cube top). Pull the point
    back toward the camera's ground point, using the pose implied by the
    frame's own homography, so the BASE grounding is right and no runtime
    closed loop has to re-measure the same constant error."""
    try:
        hom = getattr(frame, "h_pixel_world", None)
        intr = getattr(frame, "intrinsics", None)
        if hom is None or intr is None:
            return xy_z
        from ..robot.workspace import pose_from_plane_homography  # noqa: PLC0415
        pz = float(getattr(frame, "plane_z", 0.0) or 0.0)
        C = pose_from_plane_homography(np.asarray(hom, float),
                                       np.asarray(intr, float), pz)[:3, 3]
        cz = float(C[2] - pz)
        if cz < 0.3:
            return xy_z
        x, y = float(xy_z[0]), float(xy_z[1])
        f = float(h) / cz
        return (x + (float(C[0]) - x) * f, y + (float(C[1]) - y) * f,
                float(xy_z[2]))
    except Exception:
        return xy_z


def _support_top_at(ctx: SkillContext, x: float, y: float,
                    exclude: str | None = None) -> float | None:
    """Top surface of a support this point sits over, if any.

    `exclude` drops one entity from the search — the thing being grasped cannot
    be its own support, and the plate sits squarely inside its own footprint.

    THE TABLE IS NOT THE ONLY FLOOR. Depth on a small object is often junk and
    falls back to a plane — and that plane was always the table, so a block
    resting ON A PLATE was grounded 20 mm too low. The grasp then descended to
    the plate's rim height, closed on the rim, and carried the plate away a few
    centimetres at a time: over ten episodes the plate walked right across the
    workspace, and every "the robot grabbed the plate" complaint traces here.
    """
    best = None
    for eid in getattr(ctx.belief, "entities", []):
        if exclude is not None and eid == exclude:
            continue
        tr = ctx.belief.track(eid)
        if tr.xyz_base is None:
            continue
        text = f"{tr.description} {eid}".lower()
        if not any(w in text for w in ("plate", "tray", "bowl", "dish", "basket", "bin")):
            continue
        half = (float(tr.span_m) / 2 if getattr(tr, "span_m", None) else 0.07)
        if abs(x - float(tr.xyz_base[0])) <= half and abs(y - float(tr.xyz_base[1])) <= half:
            top = float(tr.xyz_base[2])
            best = top if best is None else max(best, top)
    return best


def ground_by_proposal(ctx: SkillContext, entity_ids: list[str],
                       frame) -> dict[str, tuple[float, float, float, tuple[int, int], float]]:
    """SAM-first grounding: segment everything, then ask ER a multiple-choice
    question. Precision comes from SAM's geometric boundary (centroid of the
    chosen mask), semantics from ER looking at numbered candidates — an ER
    pixel mis-point cannot corrupt the position because ER never points.

    Returns {entity_id: (x, y, z, (u, v), conf)} for the entities it matched;
    callers fall back to point-grounding for the rest.
    """
    seg = getattr(ctx, "segmenter", None)
    if seg is None or not getattr(seg, "enabled", False):
        ctx.log.event("proposal_skipped", why="no segmenter")
        return {}
    masks = getattr(frame, "_sam_proposals", None)
    if masks is None:
        try:
            masks = seg.propose(frame.rgb, max_masks=40)
        except Exception as e:
            ctx.log.event("proposal_skipped", why=f"propose: {type(e).__name__}: {e}"[:150])
            return {}
        # Cache on the frame OBJECT: a capture is immutable, so every entity
        # grounded from it shares one segmentation round-trip (measured:
        # 25-30 s of an episode was repeat SAM calls on identical frames).
        try:
            object.__setattr__(frame, "_sam_proposals", masks)
        except Exception:
            pass
    h, w = frame.rgb.shape[:2]
    wx, wy = ctx.cfg.safety.workspace_x, ctx.cfg.safety.workspace_y
    keep = []
    for m in masks:
        area = float(m.get("pixels") or 0)
        if not (150 <= area <= 0.08 * h * w):
            continue
        box = m.get("box")
        if not box:
            continue
        # Pixel centroid, not box centre: the box of a plate mask stretches
        # over its shadow and any occluding arm pixels; the mask does not.
        cx, cy = (box[0] + box[2]) // 2, (box[1] + box[3]) // 2
        raw = m.get("mask")
        try:
            if isinstance(raw, str) and raw:
                import base64
                import cv2 as _cv2
                arr = _cv2.imdecode(
                    np.frombuffer(base64.b64decode(raw), np.uint8), 0)
            elif isinstance(raw, np.ndarray):
                arr = raw
            else:
                arr = None
            if arr is not None:
                ys, xs = np.nonzero(arr)
                if len(xs) > 50:
                    cx, cy = int(xs.mean()), int(ys.mean())
        except Exception:
            pass
        pt = frame.deproject(int(cx), int(cy))
        if pt is None:
            continue
        if not (wx[0] - 0.05 <= float(pt[0]) <= wx[1] + 0.05
                and wy[0] - 0.05 <= float(pt[1]) <= wy[1] + 0.05):
            continue
        # Tabletop objects live near the table: a mask centroid deprojecting
        # 22 cm up is the arm; far below is a shadow. But glossy surfaces
        # (the styrofoam plate: 25 mm scatter, dropouts 84 mm deep) give junk
        # DEPTH while their pixel position is perfectly good — for those,
        # fall back to the table-plane assumption instead of dropping the
        # mask, which is how the white plate became undetectable.
        if not (ctx.cfg.table_z - 0.03 <= float(pt[2]) <= ctx.cfg.table_z + 0.12):
            planar = None
            deproject_planar = getattr(frame, "deproject_planar", None)
            if callable(deproject_planar):
                planar = deproject_planar(int(cx), int(cy))
            if planar is None:
                planar = (float(pt[0]), float(pt[1]), ctx.cfg.table_z)
            pt = _planar_parallax_correct(
                frame, (float(planar[0]), float(planar[1]), ctx.cfg.table_z))
            ctx.log.event("proposal_planar_fallback", px=[int(cx), int(cy)],
                          xyz=[round(v, 3) for v in pt])
        keep.append((int(cx), int(cy), pt, m))
        if len(keep) >= 16:
            break
    if not keep:
        ctx.log.event("proposal_skipped", why=f"0 of {len(masks)} masks survived filters")
        return {}
    # One multiple-choice question for ALL entities. Thin outlines with the
    # number OUTSIDE the box: a filled dot covered a 30 mm block completely
    # and the model truthfully answered "absent".
    import cv2
    canvas = frame.rgb.copy()
    for i, (cx, cy, _, m) in enumerate(keep):
        x0, y0, x1, y1 = m["box"]
        cv2.rectangle(canvas, (x0, y0), (x1, y1), (255, 255, 0), 2)
        cv2.putText(canvas, str(i), (max(x0 - 2, 0), max(y0 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
    from dataclasses import replace
    marked = replace(frame, rgb=canvas)
    names = {e: ctx.belief.track(e).description for e in entity_ids}
    q = ("Yellow numbered boxes outline segmented objects. For each item below, "
         "answer the box numbers of EVERY object matching that description "
         "(several boxes can match when lookalikes exist), or [] if absent. "
         "Items: " + json.dumps(list(names.values())))
    schema = {"type": "object",
              "properties": {d: {"type": "array", "items": {"type": "integer"}}
                             for d in names.values()},
              "required": list(names.values())}
    try:
        answer = ctx.grounder._call("proposal_match", q, [marked], "minimal", schema)
        if isinstance(answer, str):
            answer = json.loads(answer[answer.index("{"):answer.rindex("}") + 1])
    except Exception as e:
        ctx.log.event("proposal_parse_failed", note=f"{type(e).__name__}: {e}"[:200])
        return {}
    out = {}
    # ONE MASK, ONE ENTITY — across CALLS, not just within one. The batch
    # grounds entities in separate calls over the SAME cached frame, and each
    # call started with a fresh claim table: both per-instance red blocks of a
    # plural task bound to the same marker ten seconds apart, the second
    # entity tracked a block that was then picked away as the first, and every
    # later pick closed on air where it used to be. Claims live on the frame
    # (like the mask cache); an entity re-grounding on the same frame releases
    # its own old claim first.
    prev = getattr(frame, "_proposal_claims", None)
    if prev is None:
        prev = {}
        try:
            object.__setattr__(frame, "_proposal_claims", prev)
        except Exception:
            pass
    claimed: dict[int, str] = {i: e for i, e in prev.items()
                               if e not in entity_ids}
    for eid, desc in names.items():
        got = answer.get(desc)
        idxs = [i for i in (got if isinstance(got, list) else [got])
                if isinstance(i, int) and 0 <= i < len(keep)]
        # ONE MASK, ONE ENTITY. The model once assigned the same box to
        # "red_block" AND "plate", and both tracks collapsed onto one point.
        idxs = [i for i in idxs if i not in claimed]
        # Same reach-preference the point path has: "the blue block" truthfully
        # describes the blue table CLAMP at (0.47, 0.06); when any candidate
        # deprojects inside the workspace, out-of-reach lookalikes lose.
        gb = getattr(ctx, "grounding_bounds", None)
        wx2, wy2 = gb if gb else (ctx.cfg.safety.workspace_x, ctx.cfg.safety.workspace_y)
        inside2 = [i for i in idxs
                   if wx2[0] - 0.05 <= float(keep[i][2][0]) <= wx2[1] + 0.05
                   and wy2[0] - 0.05 <= float(keep[i][2][1]) <= wy2[1] + 0.05]
        if inside2:
            idxs = inside2
        if not idxs:
            if got:
                ctx.log.event("proposal_collision", entity=eid,
                              taken_by=[claimed.get(i) for i in
                                        (got if isinstance(got, list) else [got])
                                        if isinstance(i, int)])
            continue
        idx = idxs[0]
        if len(idxs) > 1:
            # Lookalikes: the same association rule as point grounding — a
            # prior picks the candidate, never the model's listing order.
            prior = expected_xy(ctx.belief.track(eid))
            if prior is None and ctx.belief.track(eid).xyz_base is not None:
                prior = np.asarray(ctx.belief.track(eid).xyz_base[:2], dtype=float)
            if prior is not None:
                idx = min(idxs, key=lambda i: float(
                    np.linalg.norm(np.asarray(keep[i][2][:2]) - prior)))
                ctx.log.event("proposal_reassociated", entity=eid,
                              candidates=idxs, chose=idx)
            else:
                # FIRST binding, no prior: bind to the instance the task still
                # needs. "Put a red block on the plate" with one red already
                # sitting there bound to the satisfied one (the planner's
                # near-the-plate disambiguator selects it precisely) and the
                # arm re-placed a placed block. The anchor's own chosen mask
                # says which candidates already satisfy the goal — geometry
                # from segmentation, no distance thresholds.
                picked = _prefer_goal_pending(ctx, eid, idxs, keep, claimed)
                if picked is not None:
                    idx = picked
        cx, cy, pt, m = keep[idx]
        # The point path has rejected robot self-detections for weeks; the
        # proposal choice needed the same reflex — a raised gripper's red
        # parts were chosen as "the red block" at (0.455, 0.159).
        where = _is_the_robot(ctx, pt, eid)
        if where:
            ctx.log.event("proposal_is_the_robot", entity=eid,
                          xyz=[round(float(v), 3) for v in pt], arm=where)
            continue
        claimed[idx] = eid
        # A graspable's mask deprojecting well above the table is board-depth
        # garbage (z=0.055 slipped the generic keep-band and dragged xy 2 cm);
        # supports may stand tall, small things may not.
        z = float(pt[2])
        support_like = any(w in desc.lower() for w in
                           ("plate", "tray", "bowl", "basket", "dish", "bin"))
        if support_like:
            # A support NEVER levitates, and the glossy plate's depth is known
            # garbage (+48 mm tonight; on() then read the block as 43 mm BELOW
            # the plate). Anchor to the plane — this feeds PLACE height too.
            planar = getattr(frame, "deproject_planar", None)
            p2 = planar(int(cx), int(cy)) if callable(planar) else None
            base = p2 if p2 is not None else pt
            pt = (float(base[0]), float(base[1]), ctx.cfg.table_z + 0.008)
            z = float(pt[2])
        small = not support_like and "table" not in desc.lower()
        # Depth is TRUSTED inside the physically possible band and replaced by
        # the plane outside it. "Always planar" was tried and knocked over an
        # upright 7 cm box: the plane anchors a TALL object at table height
        # and the grasp went in at ankle level. Junk depth (board surfaces,
        # z below the table) still falls back; honest depth stands.
        if small and not (ctx.cfg.table_z + 0.005 <= z <= ctx.cfg.table_z + 0.15):
            planar = getattr(frame, "deproject_planar", None)
            p2 = planar(int(cx), int(cy)) if callable(planar) else None
            base = _support_top_at(ctx, float(pt[0]), float(pt[1]))
            floor = base if base is not None else ctx.cfg.table_z
            if p2 is not None:
                pt = (float(p2[0]), float(p2[1]), floor + 0.015)
            else:
                pt = (float(pt[0]), float(pt[1]), floor + 0.015)
            pt = _planar_parallax_correct(frame, pt)
            ctx.log.event("proposal_planar_z", entity=eid, bad_z=round(z, 3),
                          xyz=[round(float(v), 3) for v in pt])
        out[eid] = (float(pt[0]), float(pt[1]), float(pt[2]), (cx, cy), 0.85)
        ctx.log.event("grounding_by_proposal", entity=eid, marker=idx,
                      px=[cx, cy], xyz=[round(float(v), 4) for v in pt])
    if out:
        _recheck_chosen(ctx, frame, keep, claimed, out)
    prev.clear()
    prev.update(claimed)
    return out


def _prefer_goal_pending(ctx: SkillContext, eid: str, idxs: list[int],
                         keep, claimed: dict[int, str]) -> int | None:
    """Among lookalike candidates, the one whose goal relation is still unmet.

    Only consults the plan's own goal predicates (stashed on the context by
    the agent) and the anchor's already-chosen mask box in this same call —
    membership in a segmented extent, not a hand-tuned radius. Returns None
    whenever the goal names no anchor here, the anchor is not yet claimed,
    or the split gives no usable preference.
    """
    for name, args, negated in getattr(ctx, "goal_predicates", None) or []:
        if name not in ("on", "in") or len(args) < 2 or args[0] != eid:
            continue
        a_idx = next((i for i, e in claimed.items() if e == args[1]), None)
        if a_idx is None:
            continue
        box = keep[a_idx][3]["box"]
        inside = [i for i in idxs
                  if box[0] <= keep[i][0] <= box[2] and box[1] <= keep[i][1] <= box[3]]
        want = inside if negated else [i for i in idxs if i not in inside]
        if want:
            ctx.log.event("proposal_goal_pending_choice", entity=eid,
                          anchor=args[1], chose=want[0],
                          already_satisfying=inside, negated=negated)
            return want[0]
    return None


def _crop_confirms(ctx: SkillContext, frame, bb, pxy, desc: str,
                   entity_id: str) -> bool:
    """One close look at the winning patch before it becomes the position.

    The detector binds descriptions while reading a whole cluttered scene,
    and that is where identity swaps live: a sorting run asked for "the red
    block", ER pointed at the BLUE block sitting on the plate (conf 0.95),
    and the episode spent three attempts re-placing the wrong block on its
    own destination. A close-up of the chosen patch alone is a far easier
    question. Errs permissive — any failure to ask keeps the sighting.
    """
    rgb = getattr(frame, "rgb", None)
    if rgb is None or not hasattr(rgb, "shape") \
            or not callable(getattr(ctx.grounder, "_call", None)):
        return True
    # A REGION'S CLOSE-UP IS BARE TABLE BY DEFINITION. "The left side of the
    # table" crops to a patch of wood, the recheck truthfully answers "this is
    # not a left side", and the region never earns a position — measured on
    # the rig relay, twice: the handover succeeded and the final place died
    # with 'left_side has no grounded location'. The recheck exists to catch
    # identity swaps between OBJECTS; a table region has no lookalike to swap
    # with, and the whole-scene detector is the right judge of where it is.
    if any(word in str(desc).lower() for word in
           ("side of", "corner", "edge of", "half of", "area", "region",
            "front of", "back of", "middle of", "center of", "centre of")):
        ctx.log.event("crop_recheck_skipped_region", entity=entity_id, desc=desc)
        return True
    h, w = rgb.shape[:2]
    if bb is not None:
        x0, y0, x1, y1 = [int(v) for v in bb[:4]]
    else:
        x0, y0, x1, y1 = int(pxy[0]) - 60, int(pxy[1]) - 60, \
            int(pxy[0]) + 60, int(pxy[1]) + 60
    m = 25
    crop = rgb[max(y0 - m, 0):min(y1 + m, h), max(x0 - m, 0):min(x1 + m, w)]
    if crop.size == 0:
        return True
    from dataclasses import replace
    question = (f"Close-up crop of one object just identified as {desc!r}. "
                "Answer true only if the crop really shows that item — judge "
                "object type and colour only; spatial qualifiers cannot be "
                "judged from a crop.")
    schema = {"type": "object", "properties": {"match": {"type": "boolean"}},
              "required": ["match"]}
    try:
        ans = ctx.grounder._call("crop_recheck", question,
                                 [replace(frame, rgb=np.ascontiguousarray(crop))],
                                 "minimal", schema)
        if isinstance(ans, str):
            ans = json.loads(ans[ans.index("{"):ans.rindex("}") + 1])
        ok = bool(ans.get("match", True))
    except Exception as e:
        ctx.log.event("crop_recheck_failed", entity=entity_id,
                      note=f"{type(e).__name__}: {e}"[:150])
        return True
    if not ok:
        ctx.log.event("crop_recheck_rejected", entity=entity_id, desc=desc,
                      box=[x0, y0, x1, y1])
    return ok


def _recheck_chosen(ctx: SkillContext, frame, keep, claimed: dict[int, str],
                    out: dict) -> None:
    """Look every chosen crop in the eye before anyone acts on it.

    The multiple-choice answer binds descriptions to boxes while reading a
    whole cluttered scene, and it is where identity swaps happen: a sorting
    run placed the BLUE block on the plate convinced it was the red one, and
    the episode then spiralled on "red_block is no longer visible". A close-up
    of the chosen mask alone is a far easier question, so ask it — one batched
    call for all winners, and any "no" drops that entity back to the point
    path rather than letting a wrong binding drive the arm. The lineup canvas
    is saved with the episode as evidence.
    """
    import cv2
    from dataclasses import replace
    order = [(idx, eid) for idx, eid in claimed.items() if eid in out]
    if not order:
        return
    h, w = frame.rgb.shape[:2]
    tiles = []
    for idx, eid in order:
        x0, y0, x1, y1 = keep[idx][3]["box"]
        m = 25
        crop = frame.rgb[max(y0 - m, 0):min(y1 + m, h),
                         max(x0 - m, 0):min(x1 + m, w)].copy()
        if crop.size:
            tiles.append((eid, crop))
    if not tiles:
        return
    th = max(c.shape[0] for _, c in tiles) + 28
    canvas = np.full((th, sum(c.shape[1] + 8 for _, c in tiles), 3), 255, np.uint8)
    x, names = 0, []
    for k, (eid, c) in enumerate(tiles):
        canvas[28:28 + c.shape[0], x:x + c.shape[1]] = c
        cv2.putText(canvas, str(k), (x + 2, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        names.append((eid, ctx.belief.track(eid).description))
        x += c.shape[1] + 8
    try:
        d = getattr(ctx.log, "dir", None)
        if d is not None:
            cv2.imwrite(str(d / "frames" / f"lineup-{int(ctx.log.t0) % 100000}"
                            f"-{len(list((d / 'frames').glob('lineup-*')))}.png"),
                        cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))
    except Exception:
        pass
    q = ("Numbered close-up crops of objects just identified in a scene. For "
         "each number, answer true only if the crop really shows that item. "
         "Judge object type and colour only — spatial qualifiers cannot be "
         "judged from a crop. Items: "
         + json.dumps({str(k): desc for k, (_, desc) in enumerate(names)}))
    schema = {"type": "object",
              "properties": {str(k): {"type": "boolean"} for k in range(len(names))},
              "required": [str(k) for k in range(len(names))]}
    try:
        ans = ctx.grounder._call("proposal_recheck", q,
                                 [replace(frame, rgb=canvas)], "minimal", schema)
        if isinstance(ans, str):
            ans = json.loads(ans[ans.index("{"):ans.rindex("}") + 1])
    except Exception as e:
        ctx.log.event("proposal_recheck_failed",
                      note=f"{type(e).__name__}: {e}"[:150])
        return
    for k, (eid, desc) in enumerate(names):
        if ans.get(str(k)) is False:
            out.pop(eid, None)
            ctx.log.event("proposal_recheck_rejected", entity=eid, desc=desc,
                          crop=k)


def ground_entity(ctx: SkillContext, entity_id: str, cameras: list[str] | None = None,
                  frames: dict | None = None, need_look: bool = False) -> tuple[bool, str]:
    """Locate one entity, spending at most GROUNDING_CALL_BUDGET model calls."""
    # A human verdict must not wait out a perception recovery: the phase-mark
    # hook covers motion phases, this covers every grounding round (the 37 s
    # of active-look churn AFTER the operator had already ruled, watched).
    _cb = ctx.scratch.get("interrupt_check")
    if _cb:
        _cb()
    orch = ctx.grounder
    had = getattr(orch, "_budget", None)
    try:
        if hasattr(orch, "_budget") or orch is not None:
            orch._budget = [f"ground({entity_id})", GROUNDING_CALL_BUDGET]
    except Exception:
        pass
    try:
        return _ground_entity(ctx, entity_id, cameras, frames, need_look)
    except Exception as e:
        if type(e).__name__ != "BudgetExhausted":
            raise
        ctx.log.event("grounding_gave_up", entity=entity_id,
                      calls=GROUNDING_CALL_BUDGET)
        return False, (f"{entity_id} was asked about {GROUNDING_CALL_BUDGET} times "
                       "from this view and not found; a different view is what "
                       "changes an empty answer, not a different phrasing")
    finally:
        try:
            orch._budget = had
        except Exception:
            pass


def _ground_entity(ctx: SkillContext, entity_id: str, cameras: list[str] | None = None,
                   frames: dict | None = None, need_look: bool = False) -> tuple[bool, str]:
    """Locate one entity and update its track. Returns (found, note).

    Point in every candidate camera, then pick the strongest 3D estimate:
      1. two-view triangulation (needs `proj` on >= 2 frames) — full 3D, no depth;
      2. single-view deprojection (aligned depth, or the table-plane homography).
    """
    tr = ctx.belief.track(entity_id)
    if tr.held_by:
        from ..verify import holding_something  # noqa: PLC0415

        g = ctx.robot.get_gripper(tr.held_by)
        if holding_something(g):
            return True, f"{entity_id} is held by the {tr.held_by} arm (not grounded visually)"
        tr.held_by = None  # believed held, but the gripper is empty — find it visually
    # ONE grounding per claim. A sighting still inside half the trust window
    # is the answer — verification chains were re-grounding entities seen
    # seconds earlier, and the goal check alone spent ~30 s repeating looks.
    # Positions written by our own actions still get their one visual
    # confirmation (trying is not doing); after that the confirmed sighting
    # is reused like any other.
    if (not need_look
            and tr.xyz_base is not None and tr.from_sighting and not tr.held_by
            and tr.confidence >= 0.8
            and tr.age() < min(ctx.cfg.budgets.verify_max_age_s, REID_TRUST_WINDOW_S) / 2):
        ctx.log.event("grounding_reused_sighting", entity=entity_id,
                      age_s=round(tr.age(), 2))
        xyz = tr.xyz_base
        return True, (f"{entity_id} at [{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}] "
                      f"(reused {tr.age():.0f}s-old sighting)")
    cams = cameras or _usable_cameras(ctx)
    # THE SEED GOES FIRST, WHICHEVER MODE FOLLOWS IT. This rig grounds in
    # proposal mode, and the survey check lived only in the point path below —
    # so the seed reached an entity only when the proposal path had already
    # failed for it. The blue cube got its seed that way; the plate never did,
    # and place aimed at the big plate 25 cm outside the workspace twice more
    # (rig 2026-08-06, place_using_given_position 0.747, 0.171).
    _seed = (ctx.scratch.get("survey") or {}).get(entity_id)
    _use_seed = bool(_seed) and not ctx.scratch.get(f"survey_used_{entity_id}")
    # Proposal mode routes through the same chokepoint every caller uses —
    # wiring it only into perceive() missed the pre-plan brief entirely (and
    # the sighting-reuse gate then starved perceive of stale targets).
    if not _use_seed and getattr(ctx.cfg, "grounding_mode", "point") == "proposal":
        frame0 = (frames or {}).get(cams[0]) or ctx.robot.capture(cams[0])
        got = ground_by_proposal(ctx, [entity_id], frame0)
        if entity_id in got:
            xg, yg, zg = got[entity_id][:3]
            prior = expected_xy(tr)
            if prior is not None and tr.confidence >= 0.5:
                jump = float(np.hypot(xg - prior[0], yg - prior[1]))
                limit = (BLOCKED_MAX_JUMP_M if blocked_from_view(ctx, entity_id)
                         else REID_MAX_JUMP_M)
                if not tr.from_sighting:
                    limit = min(limit, FROM_ACTION_MAX_JUMP_M)
                if jump > limit:
                    # Same identity defense as the point path; the active-look
                    # release machinery is the falsifier for a wrong claim.
                    ctx.log.event("grounding_rejected", entity=entity_id,
                                  camera=cams[0], xyz=[round(v, 3) for v in (xg, yg, zg)],
                                  jump_m=round(jump, 3), limit_m=limit,
                                  reason="proposal candidate jumped from a defended "
                                         "position; probably another object")
                    got = {}
        if entity_id in got:
            x, y, z, px0, conf0 = got[entity_id]
            # A sighting that CONFIRMS an action-written position refines the
            # coordinates but keeps the action pedigree: "nothing moved it" is
            # a fact about the action history, and losing it after one look
            # let a lookalike walk through the relaxed sighting limit.
            confirming = (not tr.from_sighting and tr.xyz_base is not None
                          and float(np.hypot(x - tr.xyz_base[0], y - tr.xyz_base[1]))
                          <= FROM_ACTION_MAX_JUMP_M)
            ctx.belief.update_track(entity_id, xyz_base=(x, y, z), camera=cams[0],
                                    px=px0, confidence=conf0,
                                    from_sighting=not confirming)
            ctx.belief.add_evidence("visual_point",
                                    f"proposal-grounded {entity_id} at [{x:.3f}, {y:.3f}]",
                                    camera=cams[0])
            return True, f"{entity_id} at [{x:.3f}, {y:.3f}, {z:.3f}] (proposal, {cams[0]})"
    hits: list[tuple[float, str, tuple[int, int], object]] = []  # conf, cam, px, frame
    # Association, not just detection. When several similar objects are in the
    # scene the model's top pick is a coin flip, and the caller finds out only
    # when the arm goes to the wrong one. We already know roughly where this
    # entity should be, so use that to choose among the candidates.
    prior_xy = expected_xy(tr)
    spent0 = _calls_spent(ctx)
    for cam in cams:
        if _calls_spent(ctx) - spent0 >= GROUNDING_CALL_BUDGET:
            ctx.log.event("grounding_budget_spent", entity=entity_id, camera=cam,
                          calls=GROUNDING_CALL_BUDGET, stage="cameras")
            break
        # A frame handed in by a batched look is the SAME image the batch asked
        # about, which is what lets the detector's per-frame cache answer here
        # for free. Capturing our own would produce a different frame and throw
        # that away.
        frame = (frames or {}).get(cam) or ctx.robot.capture(cam)
        # THE SURVEY ALREADY ASKED. One call at the top of the episode named
        # everything on the table and pointed at each of it; asking "where is
        # the small white plate" again is paying twice for one answer, and it
        # is the question that cost 54 model calls in a single rig episode.
        # The seed is only the PIXEL — the deprojection, the mask, the
        # plausibility band and the jump guard all still run below, so a stale
        # seed is caught by the same machinery that catches a stale sighting.
        # Bound before the seed short-circuit, not inside it: the relation
        # widening below calls through this name, and hoisting it into the
        # branch made the very first perceive of the first survey episode die
        # with UnboundLocalError.
        get_all = getattr(ctx.grounder, "points", None)
        seed = (ctx.scratch.get("survey") or {}).get(entity_id)
        candidates = []
        from_survey = False
        if seed and seed.get("camera") == cam and not ctx.scratch.get(f"survey_used_{entity_id}"):
            ctx.scratch[f"survey_used_{entity_id}"] = True
            u, v = seed["point"]
            candidates = [(int(u), int(v), float(seed.get("conf", 0.8)))]
            from_survey = True
            ctx.log.event("grounding_from_survey", entity=entity_id, camera=cam,
                          px=[int(u), int(v)])
        if not candidates:
            candidates = get_all(frame, tr.description) if callable(get_all) else []
        if not candidates:
            hit = ctx.grounder.point(frame, tr.description)
            candidates = [hit] if hit else []
        # A relation is a statement that several things look alike. If only one
        # came back, the description is too specific to have found the others —
        # measured, the planner answered "the metal bowl with a gold rim" for an
        # object the instruction called "the akita black bowl", the detector
        # obligingly returned exactly that one, and the relation had nothing to
        # choose between. Widening to the head noun is what makes the mechanism
        # live rather than merely present.
        # THE RELATION IS FOR WHEN NOTHING BETTER IS KNOWN, and the survey is
        # something better. It looked at the whole table and said which green
        # thing is the green block; the relation heuristic then saw a single
        # candidate, widened "the green block" to "the block", got three, and
        # re-picked by "nearest the plate" — a different object, which the crop
        # re-check then correctly refused, twice, spending the whole budget
        # (rig 2026-08-06, px 841,296 replaced by 826,429).
        if from_survey and tr.relation is not None:
            ctx.log.event("relation_deferred_to_survey", entity=entity_id,
                          relation=str(tr.relation))
        if tr.relation is not None and not from_survey and len(candidates) < 2 and callable(get_all):
            wide = _generic(tr.description)
            if wide and wide != tr.description.lower().strip():
                try:
                    more = list(get_all(frame, wide))
                except Exception:
                    more = []
                if len(more) > len(candidates):
                    ctx.log.event("widened_for_relation", entity=entity_id,
                                  was=tr.description, asked=wide,
                                  candidates=f"{len(candidates)} -> {len(more)}")
                    candidates = more
        if not candidates:
            continue
        # "The blue block" can truthfully describe blue junk parked at the
        # table's edge — the detector is not wrong, the CHOICE is. An object
        # the arm cannot reach is never the manipulation target, so candidates
        # deprojecting outside the workspace box (+5 cm slack) are excluded
        # before any scoring. If everything is outside, say so instead of
        # handing back a ghost for downstream IK to choke on.
        gb = getattr(ctx, "grounding_bounds", None)
        wx, wy = gb if gb else (ctx.cfg.safety.workspace_x, ctx.cfg.safety.workspace_y)
        inside = []
        for u, v, conf in candidates:
            pt = frame.deproject(u, v)
            if pt is None or (wx[0] - 0.05 <= float(pt[0]) <= wx[1] + 0.05
                              and wy[0] - 0.05 <= float(pt[1]) <= wy[1] + 0.05):
                inside.append((u, v, conf))
        if not inside:
            # Everything is out of reach — that can be TRUE (a block pushed
            # off the arena is still a real block, and verification needs its
            # position). Keep the candidates; reach is the mover's problem.
            inside = candidates
        if len(inside) < len(candidates):
            ctx.log.event("grounding_excluded_outside", entity=entity_id, camera=cam,
                          dropped=len(candidates) - len(inside))
        candidates = inside
        chosen = candidates[0]
        # Precedence matters. A fresh prior means this entity is already BOUND to
        # one of the lookalikes and we are re-finding it; the relation that named
        # it originally may well have stopped holding — a bowl picked from
        # between the plate and the ramekin is no longer between them. So the
        # relation decides the first binding and the prior defends it afterwards.
        if prior_xy is None and len(candidates) > 1 and tr.relation is not None:
            picked = _choose_by_relation(ctx, entity_id, tr.relation, frame, candidates)
            if picked is not None:
                chosen = picked
        if prior_xy is not None and len(candidates) > 1:
            scored = []
            for u, v, conf in candidates:
                p = frame.deproject(u, v)
                d = float(np.linalg.norm(np.asarray(p[:2]) - prior_xy)) if p is not None else 1e3
                scored.append((d, (u, v, conf)))
            scored.sort(key=lambda x: x[0])
            if scored[0][0] < 1e3:
                chosen = scored[0][1]
                if chosen != candidates[0]:
                    ctx.log.event("reid_reassociated", entity=entity_id, camera=cam,
                                  candidates=len(candidates),
                                  chose_px=[chosen[0], chosen[1]],
                                  top_px=[candidates[0][0], candidates[0][1]],
                                  distance_m=round(scored[0][0], 3))
        u, v, conf = chosen
        hits.append((conf, cam, (u, v), frame))
        # The RUNNERS-UP ride along: when the crop re-check refuses the
        # winner (ER pointed at the plate as "the yellow block" and the
        # close-up truthfully said no), the grounding used to die right
        # there with candidates still on the table. Rejection now falls
        # through to the next candidate, ordered after the winner.
        for (u2, v2, c2) in candidates[:4]:
            if (u2, v2) != (u, v):
                hits.append((min(c2, conf) * 0.9, cam, (u2, v2), frame))
        if not blocked_from_view(ctx, entity_id):
            # A SECOND CAMERA IS A SECOND CHANCE, NOT A SECOND HABIT.
            #
            # Asking every camera about every entity on every look doubled the
            # detector bill and bought nothing in the ordinary case: measured
            # across the twin runs, 5.9 `locate` calls per episode on one camera
            # and 17.3 on two, for the same work — and the accuracy comparison
            # had already shown that the extra view does not improve the
            # position (triangulation is now the fallback for exactly that
            # reason).
            #
            # What the second camera is for is the case where the first cannot
            # see: the tool hovering on the sightline with nowhere to step
            # aside. So it is consulted when that has happened, or when this
            # camera came back with nothing, and otherwise left alone. The
            # capture still happened — it costs nothing and keeps the geometry
            # available — but the model is not asked about it.
            break
    plausible_z = (ctx.cfg.table_z - 0.12, ctx.cfg.table_z + 0.60)
    # A held block may honestly be 40 cm up; a PLATE may not. Supports sit on
    # the table, and a support grounded in the air redirects every subsequent
    # place to a phantom (the white plate once grounded at z=0.107 and the aim
    # followed it out of the workspace).
    if any(w in tr.description.lower() for w in
           ("plate", "tray", "bowl", "dish", "basket", "bin")):
        plausible_z = (ctx.cfg.table_z - 0.12, ctx.cfg.table_z + 0.08)
    # A SUPPORT rests on the table by definition. The generic band exists for
    # held or stacked objects; a plate "measured" 22 cm in the air (specular
    # highlight + homography, rig 2026-08-03) sailed through it and the place
    # aimed at thin air. Supports get a band as thick as their walls could be.
    tr_kind = f"{tr.description} {entity_id}".lower()
    if any(w in tr_kind for w in ("plate", "tray", "bowl", "basket", "dish")):
        plausible_z = (ctx.cfg.table_z - 0.05, ctx.cfg.table_z + 0.15)
    if not hits:
        # The detector recognised nothing. That is not the same as nothing being
        # there, so try the other direction: segment every region and ask about
        # each. Only worth the calls once the cheap path has failed outright.
        for cam in cams:
            if _calls_spent(ctx) - spent0 >= GROUNDING_CALL_BUDGET:
                ctx.log.event("grounding_budget_spent", entity=entity_id,
                              calls=_calls_spent(ctx) - spent0, stage="identify_by_parts")
                break
            frame = ctx.robot.capture(cam)
            mask, bb, conf = identify_by_parts(ctx, frame, tr.description, prior_xy)
            if mask is None:
                continue
            p = frame.deproject_region(bb[0], bb[1], bb[2], bb[3],
                                       support_z=ctx.cfg.table_z, mask=mask)
            if p is None or not (plausible_z[0] <= float(p[2]) <= plausible_z[1]):
                continue
            centre = (int((bb[0] + bb[2]) // 2), int((bb[1] + bb[3]) // 2))
            ctx.log.event("identified_by_parts", entity=entity_id, camera=cam,
                          box=[int(x) for x in bb], confidence=round(conf, 2),
                          xyz=np.round(p, 3).tolist())
            # Confidence is carried through unmodified: this route confirmed the
            # object by VQA on a crop, but it skipped the re-identification jump
            # guard below, so downstream verification should not treat it as
            # more certain than a normal sighting.
            ctx.belief.update_track(entity_id, xyz_base=tuple(float(x) for x in p),
                                    camera=cam, px=centre, confidence=conf,
                                    from_sighting=True,
                                    note="identified by segmenting the scene")
            return True, (f"{entity_id} was not recognised by the detector; found it "
                          f"by segmenting the scene and identifying regions "
                          f"(confidence {conf:.2f})")
        # ACTIVE PERCEPTION: "not found" from a camera the arm may be
        # standing in front of is not an answer. Step aside once (the same
        # park that clears both fixed views), drop the stale look, and ask
        # again with an honestly clear sightline. One retreat per entity per
        # episode — if it is still missing from a clear view, it is missing.
        if not ctx.scratch.get(f"stepped_aside_{entity_id}"):
            ctx.scratch[f"stepped_aside_{entity_id}"] = True
            moved = False
            for arm_ in getattr(ctx.robot, "arms", []):
                try:
                    ctx.robot.move_cartesian(arm_, np.asarray(VIEWING_PARK_XYZ),
                                             seconds=1.5)
                    moved = True
                except Exception:
                    pass
            if moved:
                ctx.log.event("active_look_step_aside", entity=entity_id)
                forget_the_look(ctx, reason=f"stepped aside to re-find {entity_id}")
                # need_look: the whole point of stepping aside is a FRESH
                # observation — the reuse gate must not answer from the very
                # sighting that just failed to find anything.
                return ground_entity(ctx, entity_id, cameras, need_look=True)
        return False, f"{entity_id} ({tr.description!r}) not found in {cams}"

    method, xyz, cam, px, conf = None, None, None, None, 0.0
    rejected: list[tuple[str, float]] = []
    span: float | None = None
    if xyz is None:  # deprojection from the strongest single view
        for rank, (c, cname, q, frame) in enumerate(sorted(hits, key=lambda h: -h[0])):
            if rank and _calls_spent(ctx) - spent0 >= GROUNDING_CALL_BUDGET:
                ctx.log.event("grounding_budget_spent", entity=entity_id,
                              calls=_calls_spent(ctx) - spent0, stage="runners_up")
                break
            # Prefer the centroid of the object's whole visible patch: a single
            # pixel returns the surface facing the camera, biasing the estimate
            # toward it by half the object's depth — measured at 79 mm on a
            # basket, which is the difference between dropping the object IN and
            # perching it on the rim.
            p, how = None, "single-view"
            bb = _pick_box(ctx, frame, tr.description, q, prior_xy, entity_id)
            # A mask narrows the region from "the box" to "the object". It is
            # optional: no service, a slow one, or a mask that fails its sanity
            # checks all give None, and the box is used instead.
            mask = _mask_for(ctx, frame, bb, q)
            if mask is None:
                mask = _mask_from_depth(frame, bb, ctx.cfg.table_z)
                if mask is not None:
                    ctx.log.event("mask_from_depth", entity=entity_id, camera=cname,
                                  pixels=int(mask.sum()))
            if bb is None and mask is not None:
                # The detector pointed but would not draw a box — the normal
                # case on this rig's wide overhead frame, where ER-2 points at
                # everything and boxes almost nothing. The mask supplies the
                # extent the detector withheld.
                vs, us = np.where(mask)
                bb = (int(us.min()), int(vs.min()), int(us.max()), int(vs.max()))
                ctx.log.event("box_from_mask", entity=entity_id, camera=cname,
                              box=list(bb), pixels=int(mask.sum()))
            if bb is not None:
                p = frame.deproject_region(bb[0], bb[1], bb[2], bb[3],
                                           support_z=ctx.cfg.table_z, mask=mask)
                if p is not None:
                    how = "box-mask" if mask is not None else "box-centroid"
            if p is None:
                p = frame.deproject(*q)
            if p is None:
                continue
            # DEPTH ON A CHARUCO BOARD IS GARBAGE: the black squares eat the
            # projector pattern, and one pixel deprojected to -31 mm, then
            # +51 mm, dragging xy with it (depth-based xy scales with z) —
            # the grasp closed on air 5 cm up. A small tabletop object whose
            # depth-z is implausible keeps its PIXEL and falls back to the
            # table-plane assumption, same doctrine as the proposal path.
            small = not any(w in tr_kind for w in
                            ("plate", "tray", "bowl", "basket", "dish", "bin"))
            if (small and not tr.held_by
                    and not (ctx.cfg.table_z + 0.005 <= float(p[2])
                             <= ctx.cfg.table_z + 0.05)):
                planar = getattr(frame, "deproject_planar", None)
                p2 = planar(int(q[0]), int(q[1])) if callable(planar) else None
                base = _support_top_at(ctx, float(p[0]), float(p[1]))
                floor = base if base is not None else ctx.cfg.table_z
                if p2 is None:
                    p2 = (float(p[0]), float(p[1]), floor + 0.015)
                p2 = _planar_parallax_correct(
                    frame, (float(p2[0]), float(p2[1]), floor + 0.015))
                ctx.log.event("depth_z_implausible_planar_fallback", entity=entity_id,
                              camera=cname, bad_z=round(float(p[2]), 3),
                              xyz=[round(float(v), 3) for v in p2])
                p = np.asarray([p2[0], p2[1], floor + 0.015])
                how += "+planar-z"
            where = _is_the_robot(ctx, p, entity_id)
            if where:
                ctx.log.event("grounding_is_the_robot", entity=entity_id, camera=cname,
                              xyz=np.round(p, 3).tolist(), arm=where)
                rejected.append((cname, float(p[2])))
                continue
            if plausible_z[0] <= float(p[2]) <= plausible_z[1]:
                if not _crop_confirms(ctx, frame, bb, q, tr.description, entity_id):
                    rejected.append((cname, float(p[2])))
                    continue
                method, xyz, cam, px, conf = how, p, cname, q, c
                # The box is in hand; measure the object's width from it now.
                # `pick` needs this to decide whether it must take the rim, and
                # re-asking the detector for it later costs a full round trip.
                span = _span_from_box(frame, bb)
                break
            rejected.append((cname, float(p[2])))
            ctx.log.event("grounding_rejected", entity=entity_id, camera=cname,
                          xyz=np.round(p, 3).tolist(), z=round(float(p[2]), 3),
                          band=[round(plausible_z[0], 3), round(plausible_z[1], 3)],
                          table_z=ctx.cfg.table_z, reason="implausible height")
    if xyz is None:
        # TRIANGULATION IS THE FALLBACK, NOT THE PREFERENCE — and it took a
        # measurement to find that out, because the reverse is what the theory
        # suggests. Two views constrain three unknowns; one view plus a plane
        # assumes the object is on the plane. Ordered that way for months.
        #
        # tools/probe_grounding.py, 48 readings over identical layouts, arm
        # parked so nothing is occluded:
        #
        #     single view    median 2.4 mm   p90  8.4 mm   worst  13.5 mm   0/48 over 30 mm
        #     triangulated   median 2.2 mm   p90 15.9 mm   worst  51.6 mm   2/48 over 30 mm
        #
        # The medians tie and every tail statistic is worse. The reason is
        # CORRESPONDENCE: triangulation assumes the two views point at the same
        # physical point, and what they actually report is each view's own box
        # centroid. From 77 degrees apart, "the middle of the top face" and "the
        # middle of the visible side" are different places on the object, so the
        # rays meet where it is not. The z bias shows it — +3.2 mm triangulated
        # against +0.3 mm from one view.
        #
        # The wide baseline that makes depth well-conditioned is the same thing
        # that breaks correspondence. So it stays for the rigs it was written
        # for — depth-free, homography-only, where one view cannot place an
        # object off the table plane at all — and gets out of the way of a
        # deprojection that measured better.
        tri_views = [(h[3].proj, (float(h[2][0]), float(h[2][1])))
                     for h in hits if h[3].proj is not None]
        parallax = _parallax_deg(hits)
        if len(tri_views) >= 2 and parallax < TRIANGULATION_MIN_PARALLAX_DEG:
            # TWO VIEWS FROM ALMOST THE SAME PLACE DO NOT MAKE A DEPTH. The
            # mock's cameras sit 0.05 m apart, both looking straight down from
            # ~0.8 m — 8 degrees of parallax — and triangulating from that put a
            # bowl 59 mm off in z, enough to turn `on` from true to false.
            #
            # Reprojection error does NOT catch this, and that is worth stating:
            # the solution fits both images well. It is only badly CONSTRAINED
            # along the shared line of sight, and a residual measured in pixels
            # cannot see that. The geometry has to be checked in the world.
            ctx.log.event("triangulation_skipped", entity=entity_id,
                          parallax_deg=round(parallax, 1),
                          need_deg=TRIANGULATION_MIN_PARALLAX_DEG,
                          note="the cameras are too close together to constrain depth")
            tri_views = []
        if len(tri_views) >= 2:
            p = triangulate(tri_views)
            if p is not None:
                errs = [reprojection_error(proj, p, q) for proj, q in tri_views]
                if max(errs) <= TRIANGULATION_MAX_REPROJ_PX:
                    best = max(hits, key=lambda h: h[0])
                    method, xyz, cam, px = (f"triangulation({len(tri_views)} views)",
                                            p, best[1], best[2])
                    conf = min(0.95, best[0]) * 0.95
                else:
                    ctx.log.event("triangulation_rejected", entity=entity_id,
                                  max_reproj_px=round(max(errs), 1))
    if xyz is None:
        if rejected:
            # Naming the real cause matters: "not calibrated" sends the repair
            # loop off rewriting the object's DESCRIPTION, which cannot help when
            # the sighting was correct and the height band was wrong.
            cname, z = rejected[0]
            return False, (
                f"{entity_id} was located in {cname} but at z={z:.3f} m, outside the "
                f"plausible band {np.round(plausible_z, 3).tolist()} implied by "
                f"table_z={ctx.cfg.table_z}. The sighting is probably fine and the "
                "configured table height is wrong for this scene — rewording the "
                "object description will not help."
            )
        return False, (f"{entity_id} was seen in {[h[1] for h in hits]} but no camera is calibrated "
                       "for 3D (run `heron calibrate`)")

    # Scenes contain several similar objects, and a detector that picks the wrong
    # one reports it with full confidence. A jump that large, moments after we
    # ourselves put the object somewhere, is far more likely to be a mistaken
    # identity than real motion — so it is refused rather than believed.
    #
    # AND HOW FAR IS TOO FAR DEPENDS ON WHETHER WE COULD SEE. The general
    # tolerance is loose because a prior is noisy and objects do sometimes get
    # knocked. Neither applies when `_clear_the_view` has just reported that the
    # tool is on the sightline with nowhere to step: nothing acted on the object,
    # and the one thing we know is that the place it should be is hidden. Any
    # displacement at all is then better explained by a lookalike than by motion,
    # so the bar drops to the measurement error. Measured, at the loose bar: a
    # red block reported 115 mm from where `place` had just put it, accepted at
    # confidence 0.90, `on` scored FALSE, and the repair moved a SECOND block
    # onto a plate that already had one.
    #
    # A first, blunter version refused every reading from an obstructed look.
    # That went too far in the other direction — the tool sat over the middle of
    # a 140 mm tray, whose sightline is blocked at the centroid and perfectly
    # clear everywhere else, and `in` had no container to compare against and
    # stayed UNKNOWN to the end of the episode. Distance from the prior is the
    # discriminator; obstruction only sets how much of it to allow.
    prior = expected_xy(tr)
    # A weak prior (confidence < 0.5) is a tiebreaker for choosing among
    # lookalikes, not a claim about the world — it must never REJECT a
    # detection. The workspace-centre seed the rearrange step plants is
    # exactly such a hint, and defending it refused the real block 0.27 m
    # away (rig, 2026-08-03).
    if prior is not None and tr.confidence < 0.5:
        prior = None
    if prior is not None:
        jump = float(np.linalg.norm(np.asarray(xyz[:2]) - prior))
        obstructed = blocked_from_view(ctx, entity_id)
        limit = BLOCKED_MAX_JUMP_M if obstructed else REID_MAX_JUMP_M
        if not tr.from_sighting:
            limit = min(limit, FROM_ACTION_MAX_JUMP_M)
        if jump > limit:
            from_action = not tr.from_sighting
            ctx.log.event("grounding_rejected", entity=entity_id, camera=cam,
                          xyz=np.round(xyz, 3).tolist(), jump_m=round(jump, 3),
                          from_action=from_action, obstructed=obstructed,
                          limit_m=limit,
                          reason=("the arm was blocking the view of where it should be"
                                  if obstructed else
                                  "implausible jump from where we put it"
                                  if from_action else
                                  "implausible jump from a recent sighting")
                                 + "; probably another object")
            since = ("we put it there" if from_action
                     else f"it was seen {tr.age():.0f}s ago")
            blocked_note = (" — and the arm was standing between the camera and that "
                            "spot, so a detection elsewhere is a lookalike, not a move"
                            if obstructed else "")
            return False, (f"{entity_id} was detected {jump:.2f} m from where {since}, "
                           "with nothing acting on it since — that is more likely a "
                           f"different, similar-looking object than real motion{blocked_note}")
    confirming = (not tr.from_sighting and tr.xyz_base is not None
                  and float(np.linalg.norm(np.asarray(xyz[:2])
                                           - np.asarray(tr.xyz_base[:2])))
                  <= FROM_ACTION_MAX_JUMP_M)
    ctx.belief.update_track(entity_id, xyz_base=tuple(xyz), camera=cam, px=px,
                            span_m=span,
                            confidence=conf, from_sighting=not confirming)
    ev = ctx.belief.add_evidence("visual_point",
                                 f"grounded {entity_id} at {np.round(xyz, 3).tolist()} via {method}", camera=cam)
    ctx.log.event("grounding", entity=entity_id, camera=cam, px=list(px), xyz=np.round(xyz, 4).tolist(),
                  conf=conf, method=method, evidence=ev.id)
    return True, f"{entity_id} at {np.round(xyz, 3).tolist()} (conf {conf:.2f}, {method}, {cam})"


@skill(
    name="perceive",
    description=(
        "Ground entities in the fixed cameras and update the belief store. "
        "entities=['all'] grounds every program entity not currently held."
    ),
    params={
        "entities": {"type": "array", "items": {"type": "string"}},
        "cameras": {"type": "array", "items": {"type": "string"}},
    },
    required=[],
    post_hint="visible(entity) true/false and fresh locations for each requested entity",
)
def perceive(ctx: SkillContext, entities: list[str] | None = None, cameras: list[str] | None = None) -> SkillResult:
    targets = list(ctx.belief.entities) if not entities or entities == ["all"] else entities
    targets = relational_order(ctx, targets)
    # A THING CHANGES WHEN SOMETHING MOVES IT, NOT WHEN THE CLOCK RUNS. The
    # reuse gate used to expire a sighting after 12.5 s, and a pick and a place
    # take sixty — so every entity was "stale" by the time the arm finished,
    # and the plan's closing perceive re-asked the detector about a block it had
    # just watched being put down. Measured on the rig 2026-08-06: after place
    # reported success, one perceive step spent 14 model calls over 52 seconds
    # with the arm hovering above the plate, and then another one started.
    #
    # Age was never the right question. We know exactly what we disturbed: pick,
    # place and push all invalidate what they touched, a failed grasp clears the
    # entity it missed, and our own dead reckoning writes from_sighting=False.
    # So the gate is provenance — a real sighting, still believed, not in a
    # gripper — and the clock is out of it. What ages a belief here is an
    # action, and actions already say so.
    fresh, stale = [], []
    for entity_id in targets:
        tr = ctx.belief.track(entity_id)
        if (tr.xyz_base is not None and tr.from_sighting and not tr.held_by
                and tr.confidence >= 0.8):
            fresh.append(entity_id)
        else:
            stale.append(entity_id)
    for entity_id in fresh:
        ctx.log.event("perceive_reused_sighting", entity=entity_id,
                      age_s=round(ctx.belief.track(entity_id).age(), 2))
    targets = stale
    frames = _batched_look(ctx, targets, cameras) if targets else {}
    notes = [f"{e}: reused a {ctx.belief.track(e).age():.0f}s-old sighting" for e in fresh]
    missing = []
    # Proposal mode lives in ground_entity (the chokepoint), where the
    # identity-jump defense applies; a batch shortcut here bypassed it.
    for entity_id in targets:
        found, note = ground_entity(ctx, entity_id, cameras, frames=frames)
        notes.append(note)
        if not found:
            missing.append(entity_id)
    info = "; ".join(notes) or "nothing to ground"
    return SkillResult(ok=True, info=info + (f" | MISSING: {missing}" if missing else ""))


def occludes_view(cam_xyz, target_xyz, tool_xyz, radius: float) -> bool:
    """Is the tool standing between this camera and this point?

    Pure geometry over three known positions — the camera's pose is calibrated,
    the target's is believed, and the tool's needs no perception at all. The tool
    blocks the view when it comes within `radius` of the segment from the camera
    to the target.
    """
    c = np.asarray(cam_xyz, dtype=float)[:3]
    p = np.asarray(target_xyz, dtype=float)[:3]
    t = np.asarray(tool_xyz, dtype=float)[:3]
    ray = p - c
    n = float(np.linalg.norm(ray))
    if n < 1e-6:
        return False
    s = float(np.dot(t - c, ray) / (n * n))
    if not (0.0 <= s <= 1.0):
        return False            # beside the camera, or beyond the target
    nearest = c + ray * s
    return float(np.linalg.norm(t - nearest)) < radius


def clear_view_offset(cam_xyz, target_xyz, distance: float) -> np.ndarray:
    """Where to send the tool so it stops blocking this view.

    Sideways, in the horizontal direction that leaves the camera-to-target line
    fastest — perpendicular to it, seen from above. Lifting would not help: the
    tool is already above the target, which is exactly where the line is.
    """
    c = np.asarray(cam_xyz, dtype=float)[:2]
    p = np.asarray(target_xyz, dtype=float)[:2]
    d = p - c
    n = float(np.linalg.norm(d))
    side = np.array([-d[1], d[0]]) / n if n > 1e-6 else np.array([1.0, 0.0])
    return np.asarray(target_xyz, dtype=float)[:2] + side * distance


def _mark_blocked(ctx: SkillContext, entities: list[str]) -> None:
    """Remember that this look could not actually see these entities.

    A look the system knew was obstructed used to be handed on with no mark on
    it at all, and a caller cannot tell the difference by looking at the result.
    See `blocked_from_view` for what that cost.
    """
    ctx.scratch.setdefault("view_blocked", set()).update(entities)


def blocked_from_view(ctx: SkillContext, entity: str) -> bool:
    """Whether the last look was known to be obstructed for this entity.

    THE SYSTEM SAW THIS COMING AND WENT AHEAD ANYWAY. Measured in the twin, on a
    scene with two identical red blocks: `place` put one on the plate, the arm
    finished hovering over it, `_clear_the_view` computed that the tool was on
    the camera-to-object line, tried to step aside, found nowhere reachable to
    step, logged `view_blocked_but_stuck` — and the look went out regardless.

    The detector could not see the block under the gripper, so it returned the
    OTHER red block, 115 mm away. Nothing about that reply says "obstructed": it
    is a confident sighting of a real object that answers the description. `on`
    scored FALSE at confidence 0.90, the repair moved the second block onto the
    plate as well, and the episode was recorded as a success with both blocks on
    the plate and neither one the block the plan had been following.

    A view we knew was blocked must not read as evidence about what is behind
    it. With duplicates in the scene it is not even a missing answer — it is a
    wrong one, delivered confidently.
    """
    return entity in (ctx.scratch.get("view_blocked") or ())


def _clear_the_view(ctx: SkillContext, cam: str, frame, targets: list[str]) -> bool:
    """Move the arm out of this camera's way, if it is in it. True if it moved.

    Measured over the twin's lifelong runs: 40 groundings failed as "not found
    in ['cam_high']" while the tool was parked at exactly the xy being looked
    for. `place` finishes by lifting to a hover DIRECTLY ABOVE what it just put
    down, and verification then asks the overhead camera to confirm the
    placement through the gripper that made it. The object was there every time.
    """
    tool_of = getattr(ctx.robot, "get_cartesian", None)
    if frame is None or frame.t_base_cam is None or not callable(tool_of):
        return False
    arms = list(getattr(ctx.robot, "arms", []))
    if not arms:
        return False
    cam_xyz = np.asarray(frame.t_base_cam, dtype=float)[:3, 3]
    for arm in arms:
        try:
            tool = np.asarray(tool_of(arm), dtype=float)
        except Exception:
            continue
        blocked = [t for t in targets
                   if (xyz := ctx.belief.track(t).xyz_base) is not None
                   and occludes_view(cam_xyz, xyz, tool, VIEW_BLOCK_RADIUS_M)]
        if not blocked:
            continue
        xyz = ctx.belief.track(blocked[0]).xyz_base
        xy = clear_view_offset(cam_xyz, xyz, VIEW_CLEAR_OFFSET_M)
        target = np.array([xy[0], xy[1], float(tool[2])])
        ask = getattr(ctx.robot, "reachable", None)
        if callable(ask) and not ask(arm, target):
            ctx.log.event("view_blocked_but_stuck", camera=cam, arm=arm,
                          entities=blocked,
                          note="the arm is in the way and cannot step aside there")
            _mark_blocked(ctx, blocked)
            continue
        try:
            ctx.robot.move_cartesian(arm, target, seconds=1.0)
        except Exception as e:
            ctx.log.event("view_clear_failed", camera=cam, arm=arm,
                          error=f"{type(e).__name__}: {e}")
            _mark_blocked(ctx, blocked)
            continue
        ctx.log.event("view_cleared", camera=cam, arm=arm, entities=blocked,
                      moved_to=[round(float(v), 3) for v in target])
        return True
    return False


def _batched_look(ctx: SkillContext, targets: list[str],
                  cameras: list[str] | None) -> dict[str, object]:
    """One capture and one request per camera, for everything about to be ground.

    Grounding is written per entity — it has to be, because association,
    relations and the re-identification guard are all about ONE object's
    history. But it captured its own frame each time, so six entities meant six
    captures of a world that had not moved, six distinct frames, and therefore
    six readings of essentially the same image. The detector's own per-frame
    cache could never hit across entities because no two entities ever shared a
    frame.

    Measured over a 60-episode sweep: 929 one-at-a-time `locate` calls and zero
    `locate_many`, plus 486 repeat groundings of the same entity in the same
    camera within a single episode.

    So capture once, ask once, and hand the frames on. Everything downstream is
    unchanged — it just stops paying for what it already has. Returns the frames
    by camera; an empty dict means the caller should carry on as before.
    """
    if not targets or ctx.grounder is None:
        return {}
    prime = getattr(ctx.grounder, "prime_locate", None)
    many = getattr(ctx.grounder, "locate_many", None)
    if not callable(prime) or not callable(many):
        return {}          # a grounder without the batched path: nothing to gain

    queries, frames = [], {}
    for entity_id in targets:
        d = ctx.belief.track(entity_id).description
        if d and d not in queries:
            queries.append(d)
    if len(queries) < 2:
        return {}          # one object: the batch IS the single call

    # ONE LOOK PER MOMENT, not one per predicate.
    #
    # A goal check evaluates several predicates back to back and each one came
    # here for its own capture and its own request. Measured in a 58-second
    # pick-and-place: five separate `locate_many` calls, 11 seconds, on a scene
    # that had not changed between them.
    #
    # "Verification looks for itself" means it must not be handed the plan's
    # value — not that two predicates judged at the same instant need two
    # photographs. What makes this safe is the same premise the identity work
    # rests on: nothing in this cell moves on its own, so a look is current
    # until WE move. `heron/agent.py` also drops it after every skill.
    cached = ctx.scratch.get("look_frames")
    if cached and _still_current(ctx, cached.get("tool")):
        ctx.log.event("look_reused", cameras=sorted(cached["frames"]),
                      queries=len(queries))
        return cached["frames"]

    # A fresh look starts with a clean slate; the obstruction marks belong to
    # the look that recorded them and are reused exactly as long as it is.
    ctx.scratch["view_blocked"] = set()
    # EVERY SHUTTER FIRST, THEN EVERY QUESTION. Capturing and asking in the same
    # loop interleaves a three-second model call between one camera's frame and
    # the next, so "one look per moment" quietly became two moments three
    # seconds apart — which is wrong for triangulation, wrong for the identity
    # guard that assumes the frames agree, and wrong for the cache below, whose
    # eviction window has to be able to tell one look from the next.
    for cam in (cameras or _usable_cameras(ctx)):
        try:
            frame = ctx.robot.capture(cam)
        except Exception:
            continue
        # Look once; if the arm is standing in the shot, step it aside and look
        # again. One retake beats a whole verification that fails because the
        # gripper that did the work is now hiding it.
        if _clear_the_view(ctx, cam, frame, targets):
            try:
                frame = ctx.robot.capture(cam)
            except Exception:
                pass
        frames[cam] = frame
    # ...but only the FIRST is asked about. Every camera captured is free;
    # every camera questioned is a model call per look, and the second view has
    # been measured not to improve the position it returns. It earns its call
    # when the first camera cannot answer, which `ground_entity` decides per
    # entity. Priming the reserve here would pay for it on every look instead.
    for cam, frame in list(frames.items())[:1]:
        try:
            found = many(frame, queries)
        except Exception:
            continue
        seeded = prime(frame, found)
        ctx.log.event("batched_look", camera=cam, asked=len(queries), seeded=seeded,
                      saved_calls=max(0, seeded - 1),
                      reserve=[c for c in frames if c != cam])
    ctx.scratch["look_frames"] = {"frames": frames, "tool": _tool_now(ctx)}
    return frames


def _tool_now(ctx: SkillContext):
    """Where every arm is, or None if the backend cannot say."""
    ask = getattr(ctx.robot, "get_cartesian", None)
    if not callable(ask):
        return None
    out = {}
    for arm in getattr(ctx.robot, "arms", []):
        try:
            out[arm] = np.asarray(ask(arm), dtype=float).copy()
        except Exception:
            return None
    return out or None


def _still_current(ctx: SkillContext, tool_then) -> bool:
    """Is a look taken at `tool_then` still a picture of now?

    Only if we have not moved since. A backend that cannot report its tool pose
    gets no reuse at all: better to pay for a second look than to judge a
    placement from a photograph taken before it happened.
    """
    if not tool_then:
        return False
    now = _tool_now(ctx)
    if not now or set(now) != set(tool_then):
        return False
    return all(float(np.linalg.norm(now[a] - tool_then[a])) < LOOK_REUSE_TOLERANCE_M
               for a in now)


def forget_the_look(ctx: SkillContext, reason: str = "") -> None:
    """Drop the shared look. Anything that changes the world calls this."""
    ctx.scratch.pop("view_blocked", None)
    if ctx.scratch.pop("look_frames", None) is not None and reason:
        ctx.log.event("look_forgotten", reason=reason)


def _reachable_look_height(ctx: SkillContext, side: str, cx: float, cy: float,
                           want: float, subject_z: float) -> float | None:
    """The highest height at or below `want` where the wrist can look down.

    Returns `want` unchanged when the backend cannot answer — a backend without
    a reachability model has nothing better to offer, and refusing to look at
    all would be worse than trying. Returns None when no height in the band is
    both reachable and far enough above the subject to focus on it.

    The band is capped at the workspace CEILING, which it was not. Measured: a
    look at a container rim asked for 0.28 m against a ceiling of 0.18, so every
    probe in the band was refused, the function fell through and handed back the
    0.28 anyway, and `travel` raised SafetyViolation. That is not a bad look, it
    is a look that could never have happened — and it took out the wrist fallback
    that `on` and `in` fall back TO, leaving a correct placement permanently
    UNKNOWN.
    """
    ask = getattr(ctx.robot, "reachable", None)
    if not callable(ask):
        return want
    ceiling = float(ctx.cfg.safety.workspace_z[1])
    floor = max(float(subject_z) + INSPECT_MIN_ABOVE_M, ctx.cfg.safety.workspace_z[0])
    if floor > ceiling + 1e-9:
        # The subject is so high that focusing on it would need the wrist above
        # the workspace. Nothing to search.
        ctx.log.event("inspect_above_the_ceiling", arm=side,
                      xy=[round(cx, 3), round(cy, 3)],
                      needs=round(floor, 3), ceiling=round(ceiling, 3))
        return None
    want = min(float(want), ceiling)
    z = float(want)
    while z >= floor - 1e-9:
        if ask(side, np.array([cx, cy, z])):
            if z < want - 1e-6:
                ctx.log.event("inspect_height_lowered", arm=side,
                              wanted=round(want, 3), used=round(z, 3),
                              xy=[round(cx, 3), round(cy, 3)])
            return z
        z -= 0.01
    # Nothing in the band is reachable. Refuse the look rather than command a
    # pose the safety layer will reject: the caller sees a failed look either
    # way, and this way the failure names the reason instead of arriving as a
    # SafetyViolation from three frames down.
    ctx.log.event("inspect_no_reachable_height", arm=side,
                  xy=[round(cx, 3), round(cy, 3)],
                  band=[round(floor, 3), round(want, 3)])
    return None


@skill(
    name="inspect",
    description=(
        "Move a wrist camera directly above an entity (or x/y point) for a close-up view, "
        "then re-ground the entity from that view. Use when overhead grounding is uncertain "
        "or an object may be occluded/inside a container."
    ),
    params={
        "entity": {"type": "string"},
        "x": {"type": "number"},
        "y": {"type": "number"},
        "arm": {"type": "string", "enum": ["left", "right", "auto"]},
    },
    required=[],
    post_hint="fresh close-range grounding of the entity; visible(entity) resolved",
)
def inspect(ctx: SkillContext, entity: str | None = None, x: float | None = None,
            y: float | None = None, arm: str = "auto",
            above_z: float | None = None) -> SkillResult:
    from .primitives import choose_arm  # local import avoids cycle at module load

    if entity is not None:
        tr = ctx.belief.track(entity)
        if tr.xyz_base is None:
            return SkillResult(ok=False, error=f"inspect: {entity!r} has no location estimate; perceive first or pass x/y")
        cx, cy = float(tr.xyz_base[0]), float(tr.xyz_base[1])
    elif x is not None and y is not None:
        cx, cy = float(x), float(y)
    else:
        return SkillResult(ok=False, error="inspect needs entity= or x=/y=")

    side = choose_arm(ctx, np.array([cx, cy, 0.0]), arm)
    wrist_cam = f"cam_{side}_wrist"
    if wrist_cam not in ctx.robot.cameras:
        return SkillResult(ok=False, error=f"no wrist camera {wrist_cam!r}")
    from .primitives import travel

    # Relative to the thing being inspected, not an absolute height: a fixed
    # 0.18 m put the wrist camera 30 mm above a basket rim, looking at nothing.
    look_z = INSPECT_HEIGHT
    subject_z = ctx.cfg.table_z
    # `above_z` names the surface the look has to CLEAR — a container's rim,
    # not the table. Looking for something inside a basket, the old floor was
    # the table, and `_reachable_look_height` dutifully lowered the camera to
    # rim height: the wrist ended up beside the basket, staring at its wall
    # point-blank, and "not found in cam_arm_wrist" was the whole harvest.
    # (Callers pass it directly; it is not in the model-facing schema.)
    if above_z is not None:
        subject_z = max(subject_z, float(above_z))
    if entity is not None and ctx.belief.track(entity).xyz_base is not None:
        subject_z = max(subject_z, float(ctx.belief.track(entity).xyz_base[2]))
        look_z = subject_z + INSPECT_ABOVE_M
    elif x is not None and y is not None:
        look_z = subject_z + INSPECT_ABOVE_M
    # ...and only as high as the arm can actually hold a downward wrist there,
    # which is the same lesson approach_pose already learned. A fixed look
    # height is a guess about reach: at (0.381, 0.280) nothing above 0.05 m has
    # a solution, so the look was refused by the safety envelope and the
    # predicate it was sent to settle stayed UNKNOWN. Measured in the twin, that
    # aborted an episode whose block was sitting on the plate the whole time.
    look_z = _reachable_look_height(ctx, side, cx, cy, look_z, subject_z)
    if look_z is None:
        return SkillResult(ok=False, error=(
            f"no height above ({cx:.3f}, {cy:.3f}) is both inside the workspace and far "
            f"enough above the subject for the {side} wrist camera to focus on it"))
    travel(ctx, side, np.array([cx, cy, look_z]))
    if entity is not None:
        found, note = ground_entity(ctx, entity, cameras=[wrist_cam])
        if not found:
            # The wrist was aimed at this entity's own believed position, so a
            # look that sees nothing is a look that disproves it.
            refute_expectation(ctx, entity, note)
        return SkillResult(ok=True, info=f"inspected with {wrist_cam}: {note}")
    frame = ctx.robot.capture(wrist_cam)
    ctx.log.frame(frame, tag="inspect")
    return SkillResult(ok=True, info=f"inspected ({cx:.2f},{cy:.2f}) with {wrist_cam}", frames=[frame])
