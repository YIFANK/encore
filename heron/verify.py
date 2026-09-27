"""Predicate verification: cheap sensors first, vision second, VQA last.

Each predicate has a primary procedure and an independent alternate; the
diagnosis engine cross-checks the two when they matter. Every verdict writes
Evidence into the belief store, so "why did the agent think the block was in
the bowl" is always answerable.

Vocabulary (also documented to the planner in prompts.py):
    holding(entity, arm)      gripper_empty(arm)      visible(entity)
    in(entity, container)     on(entity, support)     open(entity)
    near(entity, target)
"""
from __future__ import annotations

import numpy as np

from .skills import SkillContext
from .skills.sensing import expected_xy, ground_entity, occludes_view, refute_expectation
from .types import Evidence, PredicateSpec, Value

HOLD_MIN_WIDTH = 0.006
HOLD_MIN_EFFORT = 0.5
# An OPEN gripper holds nothing, however hard it is pushing. Without this bound
# the real rig read "holding" the moment it opened — 0.080 m of aperture and
# 10.5 N of resting reaction satisfy both tests above — and every pick was
# refused for a precondition that could never clear. Holding the block, for
# comparison, reads 0.021 m and 84 N.
HOLD_MAX_WIDTH = 0.075
CONTAINER_HALF_EXTENT_FALLBACK = 0.07
CONTAINER_HALF_EXTENT_MAX = 0.16  # a bad deprojection must not widen the test without bound
IN_Z_SLACK = 0.06  # how far above the container's centre still counts as below its rim
# Containers are hollow: the usable cavity is narrower than the outside box.
INNER_CAVITY_FRACTION = 0.6
# Confirming something you cannot see needs more than a shrug.
OCCLUDED_VQA_MIN_CONF = 0.85
# How far a geometric test has to clear its own threshold before the answer
# means anything.
#
# `in` and `on` compare two of OUR OWN position estimates. Grounding on this rig
# measures 11 mm from a mask and 22.7 mm from a single pixel, and the plate's
# centre came back 13.6 mm off in the twin. A verdict that clears its threshold
# by less than that is reporting the noise in its own inputs.
#
# Measured: `in(red_block, grey_tray)` returned TRUE at |dxy| = 35 mm against a
# 42 mm cavity — seven millimetres of margin — with confidence 0.95, because the
# confidence was the DETECTOR's (0.95 on both groundings) and had nothing to do
# with how close the call was. Ground truth: 100 mm. The episode was recorded as
# a success.
GEOMETRIC_MARGIN_M = 0.015


def margin_confidence(base: float, margin_m: float) -> float:
    """Discount a geometric verdict by how narrowly it was decided.

    A comfortable margin keeps the caller's confidence; a margin inside the
    measurement error is scaled toward "cannot tell". Never returns zero: the
    test did produce a reading, and downstream is entitled to know which side of
    the line it fell on, only not to trust it.
    """
    if margin_m >= GEOMETRIC_MARGIN_M:
        return base
    return max(0.4, base * max(0.0, float(margin_m)) / GEOMETRIC_MARGIN_M)
ON_XY_FACTOR = 1.1
# Supports do not get more forgiving as they get wider: a benchmark that asks for
# "on the plate" means centred on it, within about a hand's width.
# Matched to the benchmark's own On(): centres within 30 mm. A verifier that is
# looser than the thing grading it can only produce successes that do not count.
ON_XY_MAX = 0.030


def _param(ctx, predicate: str, name: str, default, subject: str = ""):
    """A verifier threshold, patched or not (heron/patches.py).

    The constants above stay the source of truth for an empty store. What a
    patch buys is SCOPE: "on() tolerates 45 mm for the dish rack" is a claim
    about one container class, and editing the constant would apply it to
    every placement on the table — the exact overgeneralisation that retired
    place_offset.
    """
    store = getattr(ctx, "patches", None)
    if store is None:
        return default
    from .patches import PatchLevel  # noqa: PLC0415
    tr = ctx.belief.track(subject) if subject and subject in getattr(
        ctx.belief, "entities", {}) else None
    return store.param(PatchLevel.PREDICATE, predicate, name, default,
                       {"predicate": predicate, "entity_id": subject,
                        "entity_description": tr.description if tr else ""})
ON_Z_MAX = 0.10
ON_SUPPORT_EDGE_MARGIN_M = 0.010  # an object centred within (half - this) is resting, not perched  # higher than this and the object is being carried, not resting
# One definition, in the module whose skill has to achieve it.
from .skills.articulate import OPEN_MIN_TRAVEL_M  # noqa: E402



def _bare(desc: str) -> str:
    """Strip a leading article so a description can follow "any"."""
    d = str(desc).strip()
    low = d.lower()
    for art in ("the ", "a ", "an "):
        if low.startswith(art):
            return d[len(art):]
    return d


class Verdict:
    def __init__(self, value: Value, confidence: float, evidence: Evidence, note: str = "") -> None:
        self.value = value
        self.confidence = confidence
        self.evidence = evidence
        self.note = note


def verify(ctx: SkillContext, spec: PredicateSpec, alternate: bool = False) -> Verdict:
    """Evaluate the *positive* predicate; negation is applied by callers via
    spec.satisfied_by. `alternate=True` runs the independent second method."""
    # AN OPERATOR'S ASSERTION OUTRANKS THE INSTRUMENTS, for this episode and
    # this predicate only. The correction console lets a person say "that IS
    # in the tray" while looking at the same frames the verifier used; the
    # assertion lands here as an episode-scoped patch keyed by the predicate's
    # full text, so it can never leak onto another pair of objects or another
    # run. It is checked before the verifiers because its whole purpose is to
    # overrule them — and it is logged as evidence, not laundered into a
    # sensor reading.
    store = getattr(ctx, "patches", None)
    if store is not None and not alternate:
        from .patches import PatchLevel  # noqa: PLC0415
        episode = getattr(getattr(ctx, "log", None), "dir", None)
        for patch in store.applicable(
                PatchLevel.PREDICATE, spec.key,
                {"predicate": spec.key,
                 "episode": str(episode.name) if episode is not None else ""}):
            if "assert" in patch.delta:
                val = Value(patch.delta["assert"])
                ev = ctx.belief.add_evidence(
                    "operator", f"{spec.key} asserted {val.value} by {patch.provenance}")
                ctx.log.event("operator_assertion", predicate=spec.key,
                              value=val.value, patch=patch.id)
                return Verdict(val, 1.0, ev, note=f"operator assertion ({patch.provenance})")
    fn = {
        "holding": _holding_alt if alternate else _holding,
        "gripper_empty": _gripper_empty,
        "visible": _visible,
        "in": _in_alt if alternate else _in,
        "on": _on_alt if alternate else _on,
        "open": _open_alt if alternate else _open,
        "near": _near_alt if alternate else _near,
        "all_in": _all_in,
    }.get(spec.name)
    if fn is None:
        ev = ctx.belief.add_evidence("assumption", f"no verifier for predicate {spec.name!r}")
        return Verdict(Value.UNKNOWN, 0.0, ev, note=f"unverifiable predicate {spec.name}")
    try:
        verdict = fn(ctx, spec)
    except Exception as e:
        # ACTIVE VERIFICATION MOVES THE ARM, and a motion can be refused.
        #
        # `visible` falls back to a close-range wrist look when the overhead
        # cameras cannot find something, and `in`/`on` fly the wrist over a
        # container whose rim occludes the overhead view. Those are skill calls
        # made from inside a verifier, and they were made OUTSIDE the registry,
        # which is the only place that catches anything. So a placement the arm
        # could not reach came back as SafetyViolation, escaped check(), escaped
        # the step loop, and ended the episode:
        #
        #   _visible -> _close_look -> inspect -> travel -> SafetyViolation
        #   "target [0.432, 0.166, 0.135] is inside the workspace box but the
        #    arm cannot hold its approach orientation there"
        #
        # with 10 steps done, 2 edits spent, and status "crashed" — no
        # diagnosis, no repair, nothing to resume from. A verifier's whole job
        # is to answer TRUE, FALSE or UNKNOWN. Failing to answer is UNKNOWN,
        # which _verify_specs already routes into diagnosis and repair; it is
        # never grounds for ending the run. Nothing has moved when this fires:
        # the safety check refuses BEFORE commanding the arm.
        ev = ctx.belief.add_evidence(
            "assumption", f"{spec.key}: verification raised {type(e).__name__}: {e}")
        verdict = Verdict(Value.UNKNOWN, 0.0, ev,
                          note=f"could not be verified ({type(e).__name__}: {e})")
        ctx.log.event("verify_raised", predicate=spec.key, alternate=alternate,
                      error=f"{type(e).__name__}: {e}")
    ctx.belief.set_predicate(spec, verdict.value, verdict.confidence, verdict.evidence)
    ctx.log.event(
        "verify", predicate=spec.key, alternate=alternate, value=verdict.value.value,
        confidence=round(verdict.confidence, 2), note=verdict.note, evidence=verdict.evidence.id,
    )
    return verdict


def check(ctx: SkillContext, spec: PredicateSpec, alternate: bool = False) -> tuple[Value, Verdict]:
    """Verdict for the spec *including* negation."""
    verdict = verify(ctx, spec, alternate=alternate)
    return spec.satisfied_by(verdict.value), verdict


# -- proprioceptive ----------------------------------------------------------

def holding_something(g: dict) -> bool:
    """Whether a gripper reading means an object is held.

    One definition, imported by everything that needs it. There used to be three
    near-copies — here, in pick, and in ground_entity's held-object shortcut —
    and when the open-gripper case was fixed in this one the others kept the
    bug, so pick never recorded that it had grasped anything and grounding went
    hunting on the table for a block that was in the gripper.
    """
    # Squeeze test: holding means the fingers are commanded NARROWER than
    # where they stopped — an object is in the way. After a release the
    # command equals the width (nothing squeezed), yet friction torque kept
    # "effort" above threshold and an open gripper read as loaded (holding
    # went UNKNOWN on a block that was plainly on the table).
    cmd = g.get("commanded_m")
    if cmd is not None and cmd >= 0 and cmd > g["width_m"] - 0.005:
        return False
    return (HOLD_MIN_WIDTH < g["width_m"] < HOLD_MAX_WIDTH
            and g["effort"] > HOLD_MIN_EFFORT)


def _gripper_state(ctx: SkillContext, arm: str) -> tuple[bool, dict[str, float]]:
    g = ctx.robot.get_gripper(arm)
    something = holding_something(g)
    return something, g


def _norm_arm(ctx: SkillContext, arm: str, entity: str | None = None) -> str | None:
    """Planner-written arm strings can be sloppy ('auto'); resolve or bail."""
    if arm in ctx.robot.arms:
        return arm
    if entity is not None:
        held_by = ctx.belief.track(entity).held_by
        if held_by in ctx.robot.arms:
            return held_by
    return None


def _holding(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    entity = spec.args[0]
    arm = _norm_arm(ctx, spec.args[1] if len(spec.args) > 1 else "", entity)
    if arm is None:
        # "auto" (or any planner-invented name) with no held_by belief: the
        # grippers themselves can still answer. If NO arm holds anything, the
        # entity is definitively not held — that is a proprioceptive fact, not
        # an unknown. Only a loaded gripper of uncertain contents stays open.
        states = {a: _gripper_state(ctx, a)[0] for a in ctx.robot.arms}
        detail = ", ".join(f"{a}={'loaded' if s else 'empty'}" for a, s in states.items())
        ev = ctx.belief.add_evidence("proprioceptive",
                                     f"holding({spec.args}): arm 'auto' → scanned all arms: {detail}")
        if not any(states.values()):
            return Verdict(Value.FALSE, 0.9, ev, note=f"no gripper holds anything ({detail})")
        return Verdict(Value.UNKNOWN, 0.4, ev,
                       note=f"a gripper is loaded but contents unconfirmed ({detail})")
    something, g = _gripper_state(ctx, arm)
    tr = ctx.belief.track(entity)
    detail = f"width={g['width_m']:.3f} effort={g['effort']:.2f} track.held_by={tr.held_by}"
    ev = ctx.belief.add_evidence("proprioceptive", f"holding({entity},{arm}): {detail}")
    if not something:
        if tr.held_by == arm:
            # Belief maintenance: the grasp we believed in is refuted by the
            # sensor — drop the flag so grounding looks for it on the table.
            tr.held_by = None
            ctx.belief.invalidate_entity(entity, reason="gripper empty; held-by belief refuted")
        return Verdict(Value.FALSE, 0.9, ev, note="gripper reports nothing held")
    if tr.held_by == arm:
        return Verdict(Value.TRUE, 0.85, ev, note="gripper loaded and this entity was the grasp target")
    return Verdict(Value.UNKNOWN, 0.4, ev, note="gripper loaded but identity of held object unconfirmed")


def _holding_alt(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    """Independent visual check through the wrist camera."""
    entity = spec.args[0]
    arm = _norm_arm(ctx, spec.args[1] if len(spec.args) > 1 else "", entity)
    if arm is None:
        return _holding(ctx, spec)
    cam = f"cam_{arm}_wrist"
    if cam not in ctx.robot.cameras:
        return _holding(ctx, spec)
    frame = ctx.robot.capture(cam)
    path = ctx.log.frame(frame, tag="verify_holding")
    tr = ctx.belief.track(entity)
    value, conf, reason = ctx.grounder.vqa(
        [frame], f"Is {tr.description} currently held between the robot gripper fingers?"
    )
    ev = ctx.belief.add_evidence("visual_vqa", f"holding_alt({entity},{arm}): {reason}", camera=cam, image_path=path)
    return Verdict(value, conf, ev, note=reason)


def _gripper_empty(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    arm = _norm_arm(ctx, spec.args[0] if spec.args else "")
    if arm is None:
        # gripper_empty(auto) before a pick whose arm is chosen at execution:
        # the condition the plan actually cares about is "SOME arm is free to
        # do the pick". Scan them all — proprio is free — instead of stalling
        # the episode on an unresolvable name (rig, 2026-08-17: gripper_empty
        # (auto) sat UNKNOWN through a repair loop and aborted a trivial stack).
        states = {a: _gripper_state(ctx, a)[0] for a in ctx.robot.arms}
        detail = ", ".join(f"{a}={'loaded' if s else 'empty'}" for a, s in states.items())
        ev = ctx.belief.add_evidence("proprioceptive",
                                     f"gripper_empty({spec.args}): arm 'auto' → scanned all arms: {detail}")
        if not all(states.values()):
            return Verdict(Value.TRUE, 0.9, ev, note=f"an arm is free ({detail})")
        return Verdict(Value.FALSE, 0.9, ev, note=f"every gripper is loaded ({detail})")
    something, g = _gripper_state(ctx, arm)
    ev = ctx.belief.add_evidence("proprioceptive", f"gripper_empty({arm}): width={g['width_m']:.3f} effort={g['effort']:.2f}")
    return Verdict(Value.FALSE if something else Value.TRUE, 0.9, ev)


# -- visual ------------------------------------------------------------------

NEAR_SLACK_M = 0.06


def _vqa_primary(ctx: SkillContext, spec: PredicateSpec,
                question: str) -> Verdict | None:
    """Witness-view VQA as the PRIMARY verdict for world-only predicates.

    The geometric tests were meant to be the reliable layer, with VQA as
    corroboration — the Pigey-era ordering. The rig's failure ledger says
    otherwise: every recent on()/in() false verdict traced to MEASUREMENT
    noise (a glossy plate's depth putting it 5 cm in the air, a grounding
    drifting to a lookalike), while multi-view VQA answered the same scenes
    correctly. A predicate about the world alone — no robot state in it —
    is exactly a visual question, so ask it as one: every fixed camera
    testifies, and geometry is consulted only when the views cannot settle
    it. Proprioceptive predicates (holding, gripper_empty) never come here.

    Returns None when VQA is unavailable or unsure — the caller falls
    through to its geometric path.
    """
    try:
        frames = _witness_frames(ctx)
        if not frames:
            return None
        value, conf, reason = ctx.grounder.vqa(frames, question)
        ctx.log.event("vqa_primary", predicate=spec.key, vqa=str(value),
                      conf=round(float(conf), 2), note=str(reason)[:90])
        if value is not Value.UNKNOWN and float(conf) >= 0.6:
            want_false = spec.negated
            ev = ctx.belief.add_evidence(
                "visual_vqa", f"{spec.key} by witness views: {reason}"[:200])
            return Verdict(value, min(0.85, float(conf)), ev,
                           note=f"witness-view VQA; {reason}"[:150])
    except Exception:
        pass
    return None


def _all_in(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    """Every instance of a CLASS is in the container — the universal `in`.

    The vocabulary could not say "all the red blocks are in the tray": `in` is
    existential (deliberately — with lookalikes "the red block" is ill-posed),
    so a repeat_until over it would stop at the first block. This is the loop
    predicate for "move ALL the X", and it is a class claim, so it is asked as
    one: the witness views answer whether any instance REMAINS outside, which
    is also the phrasing that keeps working when the operator drops a sixth
    block on the table mid-task — a class question needs no enumeration to
    notice a new member.
    """
    a, b = spec.args[0], spec.args[1]
    da_ = _bare(ctx.belief.track(a).description or a)
    db_ = ctx.belief.track(b).description or b
    v = _vqa_primary(ctx, spec,
                     f"Are ALL of the {da_}s in or on the {db_}? "
                     f"Answer false if any {da_} remains outside it, anywhere "
                     f"on the table. The images show the same table from "
                     f"different angles.")
    if v is not None:
        return v
    ev = ctx.belief.add_evidence(
        "assumption", f"all_in({a},{b}): no witness view could answer the class question")
    return Verdict(Value.UNKNOWN, 0.3, ev, note="class membership needs a visual answer")


def _near(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    """Is `a` beside `b` — within the target's own footprint plus a hand's width?

    THE VOCABULARY WAS THE SCORE. "Push the plate to the front of the stove"
    cannot be said in {holding, gripper_empty, visible, in, on, open}, so the
    planner reached for `gripper_empty(arm)` — true at t=0 — and the vacuous-
    goal guard (rightly) aborted in 25-35 s. Every push-family LIBERO-PRO task
    scored zero BY CONSTRUCTION, on tasks the push primitive could attempt.
    The guard was correct; the vocabulary was too small to let it matter.

    Deliberately coarse: adjacency, not direction. "To the front of" needs a
    frame this predicate does not carry, so a push that lands beside the stove
    on the wrong side still reads TRUE here — the benchmark will disagree and
    the discrepancy will be visible in the journal, which beats refusing to try.
    """
    a, b = spec.args[0], spec.args[1]
    ok_a, ok_b, ta, tb, notes = _locate_pair(ctx, a, b)
    if not (ok_a and ok_b) or ta.xyz_base is None or tb.xyz_base is None:
        ev = ctx.belief.add_evidence("visual_point", f"near({a},{b}): grounding incomplete; {notes}")
        return Verdict(Value.UNKNOWN, 0.35, ev, note=notes)
    limit = _half_extent(ctx, b) + NEAR_SLACK_M
    d = float(np.linalg.norm(np.asarray(ta.xyz_base[:2]) - np.asarray(tb.xyz_base[:2])))
    value = Value.TRUE if d <= limit else Value.FALSE
    ev = ctx.belief.add_evidence(
        "visual_point", f"near({a},{b}): |dxy|={d:.3f} vs {limit:.3f}", camera=ta.camera)
    conf = margin_confidence(max(0.5, min(ta.confidence, tb.confidence)), abs(limit - d))
    return Verdict(value, conf, ev, note=ev.detail)


def _near_alt(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    a, b = spec.args[0], spec.args[1]
    da, db = ctx.belief.track(a).description, ctx.belief.track(b).description
    frames = [ctx.robot.capture(c) for c in ctx.robot.cameras[:1]]
    value, conf, reason = ctx.grounder.vqa(frames, f"Is {da or a} right next to {db or b}?")
    ev = ctx.belief.add_evidence("vqa", f"near({a},{b}): {reason}")
    return Verdict(value, conf, ev, note=reason)


def _visible(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    entity = spec.args[0]
    # Seeing it seconds ago counts as seeing it. Without this, perceive grounds
    # an object and pick's precondition re-grounds it a tenth of a second later
    # — four model calls and a segmentation, 47 s, to re-derive a position that
    # had no time to change.
    #
    # Only a real SIGHTING may be reused. A pose written by pick or place is
    # dead reckoning; an object placed into a bowl has a fresh position and is
    # precisely the case that needs a close look, not a shortcut.
    tr = ctx.belief.track(entity)
    if (tr.from_sighting and tr.xyz_base is not None
            and tr.age() <= ctx.cfg.budgets.verify_max_age_s):
        ev = ctx.belief.add_evidence(
            "visual", f"visible({entity}): seen {tr.age():.1f}s ago at "
            f"{[round(float(v), 3) for v in tr.xyz_base]}")
        ctx.log.event("visible_reused_sighting", entity=entity, age_s=round(tr.age(), 2))
        return Verdict(Value.TRUE, min(0.8, float(tr.confidence)), ev,
                       note=f"{entity} was seen {tr.age():.1f}s ago")
    found, note = ground_entity(ctx, entity)
    if not found:
        # "Not visible from the fixed cameras" is not "not there" — an object
        # just placed into a container is hidden by its rim, and reporting it
        # missing sends the repair loop off trying to pick it up again. Look
        # once from close range at the last place we believed it was.
        tr = ctx.belief.track(entity)
        if tr.xyz_base is not None and not ctx.scratch.get("visible_looking"):
            ctx.scratch["visible_looking"] = True
            try:
                found, note = _close_look(ctx, entity, tr.xyz_base)
            finally:
                ctx.scratch.pop("visible_looking", None)
    ev = ctx.belief.add_evidence("visual_point", f"visible({entity}): {note}")
    return Verdict(Value.TRUE if found else Value.FALSE, 0.8 if found else 0.6, ev, note=note)


def _close_look(ctx: SkillContext, entity: str, at) -> tuple[bool, str]:
    """One wrist-camera look at a believed location. Returns (found, note)."""
    from .skills.primitives import choose_arm  # noqa: PLC0415  (cycle at import)
    from .skills.sensing import inspect  # noqa: PLC0415

    side = choose_arm(ctx, np.array([float(at[0]), float(at[1]), 0.0]), "auto")
    wrist_cam = f"cam_{side}_wrist"
    if wrist_cam not in ctx.robot.cameras:
        return False, f"{entity} not found and no wrist camera to look closer with"
    result = inspect(ctx, x=float(at[0]), y=float(at[1]), arm=side)
    ctx.log.event("visible_close_look", entity=entity, camera=wrist_cam, ok=result.ok)
    if not result.ok:
        return False, f"{entity} not found; close look failed: {result.error}"
    found, note = ground_entity(ctx, entity, cameras=[wrist_cam], need_look=True)
    if not found:
        # We pointed a camera at where we believed it was and it was not there.
        # If that belief was our own dead reckoning, this is the evidence that
        # retires it — otherwise the identity guard would defend a position we
        # have just disproved.
        refute_expectation(ctx, entity, note)
    return found, f"{note} (after a close-range look)"


def _locate_pair(ctx: SkillContext, a: str, b: str, need_look: bool = False):
    """Fresh overhead grounding of two entities within the same capture burst.

    "The same capture burst" is what the docstring always claimed and what the
    code never did: two calls to `ground_entity`, each capturing its own frame,
    so the two objects whose RELATIVE position is the whole question were
    measured in different images — and the detector's per-frame cache could not
    help because no two frames matched.

    Every `on` and `in` check goes through here, and those run on every goal
    check and every postcondition. One shared look for both.
    """
    from .skills.sensing import _batched_look  # noqa: PLC0415  (cycle at import)

    frames = _batched_look(ctx, [a, b], None)
    ok_a, note_a = ground_entity(ctx, a, need_look=need_look, frames=frames)
    ok_b, note_b = ground_entity(ctx, b, need_look=need_look, frames=frames)
    ta, tb = ctx.belief.track(a), ctx.belief.track(b)
    return ok_a, ok_b, ta, tb, f"{note_a}; {note_b}"


def _half_extent(ctx: SkillContext, entity_id: str) -> float:
    """Container half-extent from a bounding box in its grounding camera.

    Grounding already drew that box and now records the width it measured, so
    the usual path costs nothing. Every `in` and `on` check calls this, and
    every one of them was re-capturing the frame and re-asking the detector for
    a box it had just been given: three of the eleven model calls in a
    58-second pick-and-place, 6.5 seconds, on a number that had not changed.

    The detector call below remains for a container whose position came from
    somewhere that had no box.
    """
    tr = ctx.belief.track(entity_id)
    if tr.span_m:
        return min(float(tr.span_m) / 2.0, CONTAINER_HALF_EXTENT_MAX)
    if tr.camera is None or ctx.grounder is None:
        return CONTAINER_HALF_EXTENT_FALLBACK
    frame = ctx.robot.capture(tr.camera)
    hit = ctx.grounder.box(frame, tr.description)
    if hit is None or frame.depth is None:
        return CONTAINER_HALF_EXTENT_FALLBACK
    u0, v0, u1, v1, _ = hit
    # Deproject the box's BOTTOM edge, where the container meets the surface. The
    # mid-height edges of a tall container seen from an angled camera project
    # PAST it onto the floor behind — measured on a basket, that read a half
    # extent of 116 mm against a true 61 mm, making the containment test almost
    # twice as permissive as intended.
    p0 = frame.deproject(u0, v1)
    p1 = frame.deproject(u1, v1)
    if p0 is None or p1 is None:
        p0 = frame.deproject(u0, (v0 + v1) // 2)
        p1 = frame.deproject(u1, (v0 + v1) // 2)
    if p0 is None or p1 is None:
        return CONTAINER_HALF_EXTENT_FALLBACK
    width = float(np.linalg.norm(np.asarray(p1[:2]) - np.asarray(p0[:2])))
    return float(np.clip(width / 2, 0.03, CONTAINER_HALF_EXTENT_MAX))


def _look_into(ctx: SkillContext, a: str, b: str):
    """Move a wrist camera over the container and try to see the object inside.

    Verification that moves the robot is a real cost, so it happens once, only
    when the fixed cameras have already failed, and only to answer a question
    they structurally cannot: whether something is inside a container whose rim
    is in the way.
    """
    tb = ctx.belief.track(b)
    if tb.xyz_base is None or not any(c.endswith("_wrist") for c in ctx.robot.cameras):
        return None
    from .skills.primitives import choose_arm  # noqa: PLC0415  (cycle at module load)
    from .skills.sensing import inspect  # noqa: PLC0415

    # Aim the look at the CONTAINER: the object's own position is exactly what we
    # do not have, so asking to inspect it would be circular.
    cx, cy = float(tb.xyz_base[0]), float(tb.xyz_base[1])
    side = choose_arm(ctx, np.array([cx, cy, 0.0]), "auto")
    wrist_cam = f"cam_{side}_wrist"
    if wrist_cam not in ctx.robot.cameras:
        return None
    # The look has to clear the RIM, and the rim is at the container's own
    # height, not the table's. Aimed with the table as its floor, the reachable-
    # height search lowered the camera to rim level and the wrist photographed
    # the basket's wall from point-blank — "not found", every time, for an
    # object that was inside.
    result = inspect(ctx, x=cx, y=cy, arm=side, above_z=float(tb.xyz_base[2]))
    ctx.log.event("in_active_look", entity=a, container=b, camera=wrist_cam, ok=result.ok)
    if not result.ok:
        return None
    # BOTH of them, in the SAME frame. This used to ground only the object from
    # the wrist and compare it against the container's OVERHEAD belief — which
    # is the point the wrist had just been aimed at. Any error in that point
    # moved the camera and the reference together, so the two cancelled and the
    # test could not fail:
    #
    #   grey_tray grounded at z=0.143 (it was the arm), wrist aimed there,
    #   block measured 27 mm from it -> in(block, tray) = TRUE
    #   ground truth: the block was 98 mm away, the tray's inner half is 43 mm
    #
    # and the skill library then learned from that episode. A relative
    # measurement is only a measurement when both terms come from one look.
    from .skills.sensing import _batched_look  # noqa: PLC0415  (cycle at import)

    frames = _batched_look(ctx, [a, b], [wrist_cam])
    # need_look: this function IS the look — a cached sighting here would make
    # "both terms from one frame" silently mean "both terms from the cache".
    ok_a, note = ground_entity(ctx, a, cameras=[wrist_cam], frames=frames, need_look=True)
    ok_b, note_b = ground_entity(ctx, b, cameras=[wrist_cam], frames=frames, need_look=True)
    if not ok_a:
        return None
    if not ok_b:
        # The container is not in the very view we pointed at it. Something is
        # wrong with where we think it is, and whatever the object's position
        # came out as, it is not evidence about being inside THIS container.
        ctx.log.event("in_active_look_no_container", entity=a, container=b,
                      camera=wrist_cam, note=note_b[:120])
        return None
    ta, tb = ctx.belief.track(a), ctx.belief.track(b)
    if ta.xyz_base is None or tb.xyz_base is None:
        return None
    half = _half_extent(ctx, b)
    da = np.asarray(ta.xyz_base[:2]) - np.asarray(tb.xyz_base[:2])
    inner = max(0.02, half * INNER_CAVITY_FRACTION)
    value = Value.TRUE if float(np.linalg.norm(da)) <= inner else Value.FALSE
    ev = ctx.belief.add_evidence(
        "visual_point",
        f"in({a},{b}) after looking in from the wrist: |dxy|={np.linalg.norm(da):.3f} "
        f"vs inner={inner:.3f}; {note}", camera=ta.camera)
    return Verdict(value, 0.8, ev, note="resolved by an active close-range look")


def _in(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    # NOT VQA-primary, deliberately: containment is the predicate where
    # cameras lie — the contents hide behind the container's walls, and a
    # confident answer from a blind viewpoint is exactly what the active
    # close-range look exists to replace. on/near judge visible surfaces;
    # in keeps look-first.
    a, b = spec.args[0], spec.args[1]
    # Containment is a VISIBILITY question as much as a position one: "expected
    # here and not seen" is the covered-object signal. A cached sighting can
    # answer where, but never whether the camera can see it — demand the look.
    ok_a, ok_b, ta, tb, notes = _locate_pair(ctx, a, b, need_look=True)
    if not ok_b:
        ev = ctx.belief.add_evidence("visual_point", f"in({a},{b}): container not found; {notes}")
        return Verdict(Value.UNKNOWN, 0.3, ev, note="container not visible")
    if not ok_a:
        # Disappearing INTO the container is the expected outcome of a successful
        # place, so "not visible" is evidence for containment as often as against
        # it — and asking a model to judge from the SAME blind viewpoint just
        # launders the ambiguity into a confident answer. Change the viewpoint
        # instead: a wrist camera held over the container can see into it, and
        # then the geometric test can actually run.
        if ok_b and not ctx.scratch.get("in_looking"):
            ctx.scratch["in_looking"] = True
            try:
                looked = _look_into(ctx, a, b)
            finally:
                ctx.scratch.pop("in_looking", None)
            if looked is not None:
                return looked
        # Before guessing, use what we DID. If the last placement put this object
        # into this container, the gripper is empty, and nothing has acted since,
        # then "invisible" is the expected consequence of success rather than
        # evidence against it. That inference is exactly what a belief store with
        # action history is for — and it is logged as action-derived, so a reader
        # can tell it apart from something we actually saw.
        # The durable record of "we put it there" is the BELIEF, not
        # `ctx.last_place`. `place` writes the object's track at the aim with
        # from_sighting=False, and that survives; last_place is cleared by the
        # agent after every place step's own postcondition check, so by the time
        # the GOAL is verified it was always empty and this inference could
        # never fire. Measured on LIBERO: two episodes whose benchmark said
        # done ended failed/aborted with `in()` at UNKNOWN 0.40 to the last
        # line — the object invisible precisely BECAUSE it was in the basket,
        # while the dead-reckoned track had been sitting inside the container's
        # cavity the whole time.
        placed = ctx.last_place or {}
        aim_xy = None
        if placed.get("entity") == a and placed.get("target") == b and placed.get("aim"):
            aim_xy = np.asarray(placed["aim"][:2], dtype=float)
        elif (ta.xyz_base is not None and not ta.from_sighting
              and not ta.held_by):
            aim_xy = np.asarray(ta.xyz_base[:2], dtype=float)
        if ok_b and aim_xy is not None and tb.xyz_base is not None:
            offset = float(np.linalg.norm(aim_xy - np.asarray(tb.xyz_base[:2])))
            inner = max(0.02, _half_extent(ctx, b) * INNER_CAVITY_FRACTION)
            if offset <= inner:
                ev = ctx.belief.add_evidence(
                    "proprioceptive",
                    f"in({a},{b}): released {offset:.3f} m from the container centre "
                    f"(inner {inner:.3f}) and the gripper is empty; not visible since, "
                    "which is what containment looks like")
                ctx.log.event("in_from_action", entity=a, container=b,
                              offset_m=round(offset, 4), inner_m=round(inner, 4))
                return Verdict(Value.TRUE, 0.7, ev, note="inferred from the placement action")
        alt = _in_alt(ctx, spec)
        if alt.value is Value.TRUE and alt.confidence < OCCLUDED_VQA_MIN_CONF:
            alt = Verdict(Value.UNKNOWN, alt.confidence, alt.evidence,
                          note=f"{alt.note} (too uncertain to confirm a hidden object)")
        if alt.value is not Value.UNKNOWN:
            ctx.log.event("in_occluded_deferred_to_vqa", entity=a, container=b,
                          value=alt.value.value, confidence=round(alt.confidence, 2))
            return alt
        ev = ctx.belief.add_evidence("visual_point", f"in({a},{b}): target not visible from fixed cameras; {notes}")
        return Verdict(Value.UNKNOWN, 0.4, ev, note="target not visible; may be occluded by container rim")
    # THE LOOK FOUND IT, SO THE CAMERAS ARE NOT BLIND HERE. The refusal at the
    # top of this function is about containment hiding its own contents — a
    # confident answer from a viewpoint that cannot see inside — and that case
    # is the branch above, where ok_a is false. This branch is the opposite: we
    # just saw the object. What the geometric test below has and VQA does not
    # is the need to decide WHICH instance it is measuring, and `on` already
    # learned what that costs.
    #
    # Measured on round 0 of the training loop. A scene with two identical red
    # cubes, one of them in the tray: the detector grounded the one still on
    # the table at 0.95 confidence, and the geometry then correctly reported
    # that cube 103 mm from a tray the OTHER one was sitting 13 mm inside. 35
    # of 76 perfect demonstrations were discarded that way, with no false
    # positives anywhere — the verifier was never wrong about what it believed,
    # only about which object it was looking at. The same frame, asked as a
    # question, came back TRUE at 1.00 on every phrasing, and volunteered the
    # reason unprompted: "there are two red blocks, and one is inside the grey
    # tray". Existential for the same reason `on` is: with one object the two
    # phrasings are the same question, and with two the existential one is what
    # the instruction meant.
    v = _vqa_primary(ctx, spec,
                     f"Is any {_bare(ta.description or a)} inside or resting in "
                     f"the {tb.description or b}? "
                     f"The images show the same table from different angles.")
    if v is not None:
        return v
    half = _half_extent(ctx, b)
    da = np.asarray(ta.xyz_base[:2]) - np.asarray(tb.xyz_base[:2])
    # The box measures the container's OUTER width; the cavity an object has to
    # land in is smaller by the wall, so accepting the full outer half-extent
    # counts objects perched on the rim as contained.
    inner = max(0.02, half * _param(ctx, "in", "inner_cavity_fraction",
                                    INNER_CAVITY_FRACTION, b))
    inside_xy = float(np.linalg.norm(da)) <= inner
    # "Below the rim" has to mean below the rim. A fixed 25 cm slack is larger
    # than most containers, so an object still in the gripper above the basket
    # satisfied it.
    z_slack = min(_param(ctx, "in", "z_slack", IN_Z_SLACK, b), half)
    below_rim = ta.xyz_base[2] <= tb.xyz_base[2] + z_slack
    value = Value.TRUE if (inside_xy and below_rim) else Value.FALSE
    conf = max(0.5, min(0.95, min(ta.confidence, tb.confidence)))
    conf = margin_confidence(conf, abs(inner - float(np.linalg.norm(da))))
    ev = ctx.belief.add_evidence(
        "visual_point",
        f"in({a},{b}): |dxy|={np.linalg.norm(da):.3f} vs inner={inner:.3f} "
        f"(outer half {half:.3f}), z_a={ta.xyz_base[2]:.3f} z_b={tb.xyz_base[2]:.3f} "
        f"slack={z_slack:.3f}",
        camera=ta.camera,
    )
    return Verdict(value, conf, ev, note=ev.detail)


def _in_alt(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    a, b = spec.args[0], spec.args[1]
    ta, tb = ctx.belief.track(a), ctx.belief.track(b)
    cam = "cam_high" if "cam_high" in ctx.robot.cameras else ctx.robot.cameras[0]
    frame = ctx.robot.capture(cam)
    path = ctx.log.frame(frame, tag="verify_in")
    value, conf, reason = ctx.grounder.vqa(
        [frame], f"Is {ta.description} inside {tb.description}? Judge only from what is visible."
    )
    ev = ctx.belief.add_evidence("visual_vqa", f"in_alt({a},{b}): {reason}", camera=cam, image_path=path)
    return Verdict(value, conf, ev, note=reason)


def _witness_frames(ctx: SkillContext, primary: str | None = None) -> list:
    """Frames from every FIXED camera for a yes/no visual question.

    Calibration gates grounding (an uncalibrated camera cannot yield a
    position), but a VQA needs no metric map — a second viewpoint is a second
    witness for free. The table-level cam_low cannot measure anything, yet
    "is the block ON the plate" is unambiguous exactly from its angle.
    """
    frames = []
    for name in getattr(ctx.robot, "cameras", []) or []:
        # Wrist cameras ride along too: capturing one is free, and after a
        # stack the placing arm now PARKS at a side-witness pose whose wrist
        # camera stares at the tower — the exact view the top-down occlusion
        # hides (a stacked block honestly covers the one beneath it, and the
        # front camera's grazing sightline is one marker board away from
        # blind). An uninformative wrist frame just gets ignored by the VLM.
        try:
            frames.append(ctx.robot.capture(name))
        except Exception:
            continue
    if not frames and primary:
        try:
            frames.append(ctx.robot.capture(primary))
        except Exception:
            pass
    return frames


def _on(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    a, b = spec.args[0], spec.args[1]
    da_ = ctx.belief.track(a).description or a
    db_ = ctx.belief.track(b).description or b
    # EXISTENTIAL on purpose: "the red block" is ill-posed with two red
    # blocks in the scene, and the task's meaning is existential anyway —
    # "remove the red block from the plate" is satisfied when NO red block
    # rests there, "put a red block on" when ANY does. With a unique object
    # the two phrasings are the same question. (A removal episode moved the
    # off-plate twin to a random spot and the witness truthfully kept
    # answering about the one still sitting on the plate.)
    v = _vqa_primary(ctx, spec, f"Is any {_bare(da_)} resting on top of the {db_}? "
                                f"The images show the same table from different angles.")
    if v is not None:
        return v
    ok_a, ok_b, ta, tb, notes = _locate_pair(ctx, a, b)
    if ok_b and not ok_a and not ctx.scratch.get("on_looking"):
        # THE ARM THAT PUT IT THERE IS STANDING OVER IT. `place` finishes with
        # the tool hovering above the target, so the overhead camera's view of a
        # placement is blocked by the gripper that made it. Measured in the twin:
        # "red_block not found in ['cam_high']" on both verification attempts of
        # an episode whose block was on the plate the entire time.
        #
        # That is a viewpoint problem, and the answer to a viewpoint problem is a
        # different viewpoint rather than a guess. Where to point it is not a
        # guess either: `place` recorded where it sent the object, and that is
        # what `expected_xy` hands back. `in` has had this since containers
        # started hiding their contents; `on` needed it for the same reason and
        # did not have it, so a correct placement scored UNKNOWN and the episode
        # ended unverified.
        #
        # This is the SECOND line of defence, and it should rarely fire. The
        # first is `_clear_the_view`, which `_locate_pair` has already run: it
        # computes whether the tool sits on the camera-to-object line, steps it
        # aside, and retakes the overhead shot — one small motion, no wrist
        # flight, and it protects every predicate rather than just this one.
        # What reaches here is what that could not fix: an arm with nowhere
        # clear to step (logged as `view_blocked_but_stuck`), or an object
        # genuinely hidden by something that is not the robot. Keep both; they
        # answer different halves of the same question.
        at = expected_xy(ta)
        if at is not None:
            from_action = not ta.from_sighting
            ctx.scratch["on_looking"] = True
            try:
                ok_a, look_note = _close_look(ctx, a, at)
            finally:
                ctx.scratch.pop("on_looking", None)
            ta = ctx.belief.track(a)
            notes = f"{notes}; {look_note}"
            ctx.log.event("on_active_look", entity=a, support=b, ok=ok_a,
                          at=[round(float(at[0]), 3), round(float(at[1]), 3)],
                          from_action=from_action)
    if not (ok_a and ok_b):
        # The overhead camera losing sight of a stacked object is the EXPECTED
        # look of success — red on blue reads as one blob from above, and the
        # parked arm can sit square on the sightline. Before conceding UNKNOWN,
        # put the question to every fixed camera as a witness: the table-level
        # view sees a stack side-on, where the answer is unambiguous.
        try:
            frames_w = _witness_frames(ctx, primary=None)
            if frames_w:
                da = ctx.belief.track(a).description
                db = ctx.belief.track(b).description
                vw, cw, nw = ctx.grounder.vqa(
                    frames_w, f"Is any {_bare(da)} resting on top of the {db}? The views "
                              f"show the same table from different angles.")
                ctx.log.event("on_witness_vqa", entity=a, support=b,
                              vqa=str(vw), conf=round(float(cw), 2))
                if vw is not Value.UNKNOWN and float(cw) >= 0.7:
                    ev = ctx.belief.add_evidence(
                        "vqa", f"on({a},{b}) by witness view: {nw}"[:200])
                    return Verdict(vw, min(0.8, float(cw)), ev,
                                   note=f"witness-view VQA; {nw}"[:160])
        except Exception:
            pass
        ev = ctx.belief.add_evidence("visual_point", f"on({a},{b}): grounding incomplete; {notes}")
        return Verdict(Value.UNKNOWN, 0.35, ev, note=notes)
    # "On" is a centring claim, not an overlap claim. Scaling the tolerance with
    # the support's width makes a wide plate accept an object resting on its very
    # edge — measured, this verifier allowed 74 mm on a plate whose own test
    # requires the two centres within 30 mm, and it returned TRUE on three
    # placements that missed by 42-70 mm. An object is either centred on its
    # support or it is balanced on the lip, and those deserve different answers.
    holder = ctx.belief.track(a).held_by
    if holder:
        # Grounding reports a held object as "found" without a position, because
        # it is wherever the gripper is. Whatever else is true, an object in the
        # hand is not resting on anything.
        ev = ctx.belief.add_evidence(
            "proprioceptive", f"on({a},{b}): {a} is still held by the {holder} arm")
        return Verdict(Value.FALSE, 0.85, ev, note=f"{a} is still in the {holder} gripper")
    if ta.xyz_base is None or tb.xyz_base is None:
        ev = ctx.belief.add_evidence("visual_point", f"on({a},{b}): no position for one of them; {notes}")
        return Verdict(Value.UNKNOWN, 0.3, ev, note="missing position estimate")
    if getattr(ctx.cfg, "on_criterion", "centred") == "support":
        # Physically on it: within the support's own footprint, less an edge
        # margin. margin_confidence still discounts near-boundary verdicts, so
        # narrow calls get VQA corroboration at the goal check as before.
        half = max(0.02, _half_extent(ctx, b) - ON_SUPPORT_EDGE_MARGIN_M)
    else:
        half = min(_half_extent(ctx, b) * ON_XY_FACTOR,
                   _param(ctx, "on", "xy_max", ON_XY_MAX, b))
    da = np.asarray(ta.xyz_base[:2]) - np.asarray(tb.xyz_base[:2])
    dz = float(ta.xyz_base[2]) - float(tb.xyz_base[2])
    centred = float(np.linalg.norm(da)) <= half
    resting = -0.01 <= dz <= ON_Z_MAX
    value = Value.TRUE if (centred and resting) else Value.FALSE
    ev = ctx.belief.add_evidence(
        "visual_point",
        f"on({a},{b}): |dxy|={np.linalg.norm(da):.3f} vs {half:.3f}, dz={dz:+.3f} "
        f"(allowed -0.01..{ON_Z_MAX:.2f})",
        camera=ta.camera,
    )
    conf = margin_confidence(max(0.5, min(ta.confidence, tb.confidence)),
                             abs(half - float(np.linalg.norm(da))))
    if value is Value.FALSE:
        lp = getattr(ctx, "last_place", None) or {}
        we_put_it_there = (lp.get("entity") == a and lp.get("target") == b) or (
            # The durable record: place wrote this track (from_sighting=False)
            # and the identity defense kept it. last_place is cleared after
            # the step's own postcondition, so the goal check must read the
            # belief, not the scratchpad.
            ta.xyz_base is not None and not ta.from_sighting and not ta.held_by)
        if we_put_it_there:
            # Our own place says the object went onto this support; a geometric
            # FALSE right after usually means the re-grounding drifted to a
            # lookalike (measured: GT 3 mm on the tray scored |dxy|=136 mm off
            # the OTHER red block). Two stronger layers disagree — the tiebreak
            # is a DISCRIMINATIVE visual question, phrased with the object
            # descriptions so a white board cannot stand in for a white plate.
            try:
                cam = ta.camera or tb.camera or next(iter(ctx.cfg.cameras or {}), None)
                frames2 = _witness_frames(ctx, primary=cam)
                q = (f"Is the {ctx.belief.track(a).description} resting on top of "
                     f"the {ctx.belief.track(b).description}? Judge only that "
                     f"specific object and support.")
                v2, c2, note2 = ctx.grounder.vqa(frames2, q)
                ctx.log.event("on_action_vqa_arbitration", entity=a, support=b,
                              vqa=str(v2), conf=round(float(c2), 2))
                if v2 is Value.TRUE and float(c2) >= 0.7:
                    ev2 = ctx.belief.add_evidence(
                        "vqa", f"on({a},{b}): geometry said no but we placed it "
                               f"there and the view agrees: {note2}"[:200])
                    return Verdict(Value.TRUE, min(0.8, float(c2)), ev2,
                                   note=f"placed-there + VQA corroboration; {note2}"[:160])
            except Exception:
                pass
    return Verdict(value, conf, ev, note=ev.detail)


def _open(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    """Has this drawer come out of its cabinet?

    Answered by comparing where the front is NOW against where it was the first
    time we saw it, which is the only baseline that is evidence rather than
    intention. A pose written by the pulling skill would make this predicate a
    restatement of what the skill tried to do — and the whole reason this system
    verifies is that trying is not doing.

    UNKNOWN when there is no baseline, deliberately: the alternate verifier asks
    the question visually, and an honest "I never saw it closed" is worth more
    than a confident guess from one observation.
    """
    entity = spec.args[0]
    from .skills.sensing import ground_entity  # noqa: PLC0415

    tr = ctx.belief.track(entity)
    if tr.first_xyz is None:
        ev = ctx.belief.add_evidence(
            "assumption", f"open({entity}): never seen before anything acted on it")
        return Verdict(Value.UNKNOWN, 0.0, ev,
                       note="no baseline position; nothing to compare against")
    found, note = ground_entity(ctx, entity)
    tr = ctx.belief.track(entity)
    if not found or tr.xyz_base is None:
        ev = ctx.belief.add_evidence("visual_point", f"open({entity}): not located; {note}")
        return Verdict(Value.UNKNOWN, 0.3, ev, note=note)
    travel = float(np.linalg.norm(np.asarray(tr.xyz_base[:2], float)
                                  - np.asarray(tr.first_xyz[:2], float)))
    value = Value.TRUE if travel >= OPEN_MIN_TRAVEL_M else Value.FALSE
    ev = ctx.belief.add_evidence(
        "visual_point",
        f"open({entity}): the front has moved {travel * 1000:.0f} mm from where it "
        f"started (open at {OPEN_MIN_TRAVEL_M * 1000:.0f} mm)", camera=tr.camera)
    return Verdict(value, max(0.5, float(tr.confidence)), ev, note=ev.detail)


def _open_alt(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    entity = spec.args[0]
    tr = ctx.belief.track(entity)
    cam = ctx.robot.cameras[0]
    frame = ctx.robot.capture(cam)
    path = ctx.log.frame(frame, tag="verify_open")
    value, conf, reason = ctx.grounder.vqa(
        [frame], f"Is {tr.description} pulled open, sticking out of the cabinet?")
    ev = ctx.belief.add_evidence("visual_vqa", f"open_alt({entity}): {reason}",
                                 camera=cam, image_path=path)
    return Verdict(value, conf, ev, note=reason)


def _on_alt(ctx: SkillContext, spec: PredicateSpec) -> Verdict:
    a, b = spec.args[0], spec.args[1]
    ta, tb = ctx.belief.track(a), ctx.belief.track(b)
    # The camera the subject was just GROUNDED in comes first: its stored
    # pixel needs no cross-camera reprojection, whose 1-2 cm error drifted
    # the deictic circle onto a neighbouring block (twin plural run, red_2:
    # 'the object circled consists of blue blocks', conf 1.0, goal failed
    # with both reds verifiably on the plate).
    ta_cam = getattr(ta, "camera", None)
    prefer = ([ta_cam] if ta_cam in ctx.robot.cameras and "wrist" not in str(ta_cam) else []) + \
        (["cam_low"] if "cam_low" in ctx.robot.cameras else []) + \
        [c for c in ctx.robot.cameras if c != "cam_low" and "wrist" not in c]
    seen = set()
    prefer = [c for c in prefer if not (c in seen or seen.add(c))]
    # A camera whose sightline to the subject the arm is standing on cannot
    # answer, and it does not say so — it confidently describes the occluder
    # (twin, this run: the circle landed on a grey link and "the circled
    # object is white/grey" failed a goal the witness views had confirmed
    # three times). The tool's position needs no perception; check the
    # geometry first and skip blind cameras. All blind -> UNKNOWN, honestly.
    frame, cam = None, None
    for c in prefer:
        f = ctx.robot.capture(c)
        if ta.xyz_base is not None and getattr(f, "t_base_cam", None) is not None:
            cam_xyz = np.asarray(f.t_base_cam, dtype=float)[:3, 3]
            blocked = False
            inner = getattr(ctx.robot, "_robot", ctx.robot)
            poly_fn = getattr(inner, "arm_polyline", None)
            for arm in getattr(ctx.robot, "arms", []):
                try:
                    # The whole skeleton, where the backend can produce it —
                    # the failure frame shows a FOREARM on the sightline with
                    # the tool nowhere near it.
                    pts = (np.asarray(poly_fn(arm), dtype=float)
                           if callable(poly_fn) else
                           [np.asarray(ctx.robot.get_cartesian(arm),
                                       dtype=float)[:3]])
                except Exception:
                    continue
                if any(occludes_view(cam_xyz, ta.xyz_base, pt, radius=0.08)
                       for pt in pts):
                    blocked = True
                    break
            if blocked:
                ctx.log.event("verify_view_blocked", predicate=spec.key, camera=c)
                continue
        frame, cam = f, c
        break
    if frame is None:
        ev = ctx.belief.add_evidence(
            "visual_vqa", f"on_alt({a},{b}): every fixed camera's sightline "
            "to the subject is blocked by the arm")
        return Verdict(Value.UNKNOWN, 0.3, ev,
                       note="the arm blocks every fixed view of the subject; "
                            "no independent answer available")
    # "Is THE red block on the plate?" is ill-posed in a scene with two red
    # blocks: the model truthfully answers about whichever one it attends to,
    # and a sorting run failed its goal check at conf 1.0 while the placed
    # block sat centred on the plate — the verifier was describing the OTHER
    # red block. We hold a defended position for the subject (the jump guard
    # keeps lookalikes out of it), so point at it: mark that spot in the
    # image and ask about the marked object. Deixis needs no article.
    # A STACK HIDES ITS OWN EVIDENCE. Asked "is it on top of the red cube",
    # a camera looking down at a successful stack answers "no red cube is
    # present underneath it" — truthfully, because the cargo is covering it.
    # Measured on the rig: the orange cube sat at the red cube's xy, 17 mm up,
    # the witness side view said yes three times, and this view said no three
    # times with that exact wording, and the goal failed.
    #
    # So when the belief says the subject is ABOVE the target's top rather
    # than beside it, ask the question this view can actually answer: is the
    # thing raised, or is it on the table? That is also the discriminator for
    # the real failure — a cargo that toppled is lying on the table next to
    # the support, in plain sight.
    stacked = False
    if ta.xyz_base is not None and tb.xyz_base is not None:
        rise = float(ta.xyz_base[2]) - float(tb.xyz_base[2])
        dxy = float(np.hypot(ta.xyz_base[0] - tb.xyz_base[0],
                             ta.xyz_base[1] - tb.xyz_base[1]))
        stacked = rise > 0.005 and dxy < max(0.5 * _half_extent(ctx, b), 0.02)
    q = (f"Is {ta.description} resting on top of {tb.description}?" if not stacked
         else f"Is {ta.description} raised up on something, rather than lying "
              f"flat on the table surface?")
    marked = frame
    u = v = None
    if cam == getattr(ta, "camera", None) and getattr(ta, "px", None) is not None \
            and ta.age() < 20.0:
        u, v = int(ta.px[0]), int(ta.px[1])
    elif ta.xyz_base is not None and getattr(frame, "proj", None) is not None:
        p = frame.proj @ np.array([*ta.xyz_base[:3], 1.0])
        if abs(p[2]) > 1e-12:
            u, v = int(p[0] / p[2]), int(p[1] / p[2])
    if u is not None and getattr(frame, "rgb", None) is not None:
        if True:
            h, w = frame.rgb.shape[:2]
            if 0 <= u < w and 0 <= v < h:
                import cv2
                from dataclasses import replace
                canvas = frame.rgb.copy()
                cv2.circle(canvas, (u, v), 22, (255, 0, 255), 3)
                marked = replace(frame, rgb=canvas)
                q = (f"A magenta circle marks a spot. Is the object at the "
                     f"magenta circle {ta.description}, and is it resting on "
                     f"top of {tb.description}?") if not stacked else (
                    f"A magenta circle marks a spot. Is the object at the "
                    f"magenta circle {ta.description}, and is it raised up on "
                    f"something rather than lying flat on the table?")
    path = ctx.log.frame(marked, tag="verify_on")
    value, conf, reason = ctx.grounder.vqa([marked], q)
    ev = ctx.belief.add_evidence("visual_vqa", f"on_alt({a},{b}): {reason}", camera=cam, image_path=path)
    return Verdict(value, conf, ev, note=reason)
