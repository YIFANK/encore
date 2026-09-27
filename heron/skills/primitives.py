"""Motion primitives for the stationary kit: top-down pick/place/push in the
world frame, with proprioceptive evidence attached to every result.

Constants that encode rig geometry live here in one block (a Pigey lesson:
scattered inline magic numbers made its behaviors impossible to retune).
"""
from __future__ import annotations

from typing import Optional

import time

import numpy as np

from ..types import SkillResult
from . import SkillContext, skill

HOVER_CLEARANCE = 0.10  # m above an object's top surface for approach/retreat
GRASP_DEPTH = 0.02  # descend this far below the perceived top point to grasp
DROP_CLEARANCE = 0.04  # release height above the support surface
SETTLE_AFTER_RELEASE_S = 1.0  # let the object land before anything judges it
# A PLACEMENT IS CORRECTED BY MEASUREMENT, NOT COMPUTED MORE PRECISELY — the
# same sentence as the push loop, for the same reason. Measured over 85 rig
# placements: a median 6.9 mm bias in +y and 20 mm of scatter. A 138 mm plate
# swallows that; a 28 mm cube stacked on another does not, and the yellow cube
# landed 8-15 mm off its target three times running and rolled off each time.
# The bias lives in the world map, which is why aiming harder cannot remove it
# and looking afterwards can.
# How far from where the pick's wrist look measured cam_high's bias that
# measurement is still trusted for a place aim (same regional bias field).
WRIST_BIAS_REACH_M = 0.20
# Step back toward the base (orientation stays vertical) so the support
# leaves the cargo's shadow and enters the wrist camera's frame.
WRIST_LOOK_STEPBACK_M = 0.10
PLACE_PRECISE_TOL_M = 0.006
# A post-place support re-ground further than this from the support's prior
# position is a mis-match, not a moved support (see place_precise_retarget).
RETARGET_MAX_JUMP_M = 0.06
# How far above a stack's top the cargo is let go. FLAT_DROP_CLEARANCE (15 mm)
# is a plate's number: a walled dish catches whatever it is given. A cube has
# no walls, and the drop is the whole difference between a stack and a cube
# bouncing off one. Measured on the rig with a real touch as the ruler: the
# tool contacts the support's top at z = 16.5 mm, and geometry was commanding
# the release at 42.7 mm — a 26 mm drop.
STACK_RELEASE_GAP_M = 0.003
# A TOUCH LIGHT ENOUGH NOT TO BE A PUSH. The ordinary probe descends at 20 mm/s
# and calls it contact at 1.5 N over baseline; effort is polled at ~12 Hz, so the
# tool travels 1.7 mm between samples and presses several more before it halts.
# Against a table that is nothing. Against a 30 g cube standing free it is a
# shove: measured on the rig, the descent felt the support's top at 16.5 mm and
# moved it out from under the cargo in the same motion. Slower and lighter costs
# a couple of seconds and keeps the support where it was.
STACK_PROBE_SPEED_MPS = 0.005
STACK_CONTACT_MARGIN_N = 0.6
PLACE_PRECISE_TRIES = 2
FLAT_DROP_CLEARANCE = 0.015   # release height over a support with no walls
FLAT_SUPPORT_WORDS = ("plate", "tray", "table", "mat", "board", "stove", "counter")
OPEN_WIDTH = 0.08
GRIPPER_MAX_SPAN_M = 0.075  # anything wider has to be taken by its edge
# How far out from the centre to take an over-wide vessel. Measured on the
# 96 mm bowl: 20-48 mm all lift it, 0 mm closes on the hole unless the descent
# is deep, and the old span/2 with an over-measured span aimed 58 mm out.
RIM_OFFSET_MAX_M = 0.045
STACK_TARGET_HALF_EXTENT_M = 0.045  # targets this small are stacked onto by feel
CLOSED_ON_AIR_WIDTH = 0.006  # narrower than this after closing = nothing grasped
PUSH_DEPTH_Z = 0.02  # push at this height above the table (capped by the object's own height)
PUSH_MAX_SWEEPS = 3  # a push is corrected by measurement, not computed more carefully
PUSH_GOAL_TOL_M = 0.025

# Where one arm can put something down and the other can pick it up. The search
# below finds the place; these say what "reachable enough" means there. The
# overlap on this rig is 40 mm wide and 50 mm tall, so the hover is small on
# purpose — asking for the usual 0.18 m approach height would report no overlap
# at all and make the two arms look like two separate robots.
HANDOFF_RELEASE_Z_M = 0.01
# How much air over the table a spot needs before something may be put down
# there: enough to descend onto it and to come back for it later.
PLACE_APPROACH_M = 0.05
HANDOFF_HOVER_M = 0.04
HANDOFF_SEARCH_STEP_M = 0.02
HANDOFF_SEARCH_MAX_M = 0.30

# Contact-descend (the depth-free grasp): step down until external effort jumps.
# On a rig without depth an object's HEIGHT is unknown, so lateral transit happens
# at APPROACH_HEIGHT above the table — clear of anything on it — and every
# approach is a vertical descent that feels its way down.
APPROACH_HEIGHT_M = 0.22
CONTACT_STEP_M = 0.005  # 6.5 mm punched THROUGH sim contact in one step
CONTACT_MARGIN_N = 1.5  # rise over the free-space baseline that counts as touch
CONTACT_MAX_DESCENT_M = 0.26  # enough to reach the table from APPROACH_HEIGHT
# How far above the height perception expects to feel for the surface. Every
# step is a blocking round-trip to the controller, so feeling all the way down
# from the hover took 25 of them and looked exactly like what it was: the arm
# stuttering downward. Depth tells us where the object is; the steps only need
# to cover the error in that estimate, not the whole descent.
CONTACT_PROBE_M = 0.012  # measured on the rig: each felt step costs ~3.5 s
# (blocking move + external-effort round-trip), so the probe segment is kept
# to two steps — z_expected has been good to ~5 mm whenever we had it
CONTACT_BACKOFF_M = 0.008  # lift slightly so fingers close beside, not through
# How far the object may honestly ride from the tool centre once it is clamped
# between the jaws: half its own width, plus a little for the measurement. An
# offset larger than that is not the object sitting off-centre in the gripper —
# it is the arm not having gone where it was sent, and compensating for THAT at
# the release doubles the error instead of cancelling it (see `pick`).
CARRY_MEASUREMENT_SLACK_M = 0.005
# How far the tool may settle from the grasp point before closing is pointless,
# and how many times to send it again first. The default span is a small
# object's: unknown width should fail a marginal grasp rather than report one
# that closed on the table.
GRASP_ARRIVAL_DEFAULT_SPAN_M = 0.04
GRASP_ARRIVAL_SLACK_M = 0.005
GRASP_ARRIVAL_TRIES = 3

# Grasp-proposal service (cfg.grasp_service_url): the v1 safety envelope.
# M2T2 proposes full 6-DOF grasps; this rig executes the top-down family it has
# already proven — take xy + depth + yaw from a proposal only when its approach
# fits the cone the approach_rvec machinery and tilt fallbacks already handle,
# and keep Heron's own hover/descend motion. Full 6-DOF execution is v2.
GRASP_SERVICE_MAX_TILT_DEG = 30.0
# A proposal is only a proposal for THIS object if it lands near where the
# object is believed to be; farther than this and the model has ranked a
# neighbour (or the table edge), not the target. Measured in the twin with two
# blocks 62 mm apart: plausible-scoring grasps sit in the GAP between them at
# ~30-40 mm from either centre, so the radius must stay under that — half a
# small object's span (~16 mm) plus the belief's own error (~10 mm), not more.
GRASP_SERVICE_MATCH_RADIUS_M = 0.03
GRASP_SERVICE_MIN_SCORE = 0.5  # weaker than this loses to the vertical heuristic
# World-frame crop sent to the service. Tight on purpose: M2T2 is
# object-agnostic and ranks by graspability, so a bigger neighbour inside the
# crop soaks up the proposal budget — measured with an orange cube 6 cm from
# the target, the top-200 contained nothing on the block at all. The believed
# position this crop centres on comes from the VLM grounding, which has been
# right within a centimetre all session.
GRASP_SERVICE_CROP_RADIUS_M = 0.08



def approach_pose(ctx: SkillContext, xy, perceived_z: float,
                  arm: str = "right") -> np.ndarray:
    """Safe transit point above (x, y): as high as the arm can actually hold.

    APPROACH_HEIGHT_M was chosen for a rig with no depth, where an object's
    height is unknown and the only safe answer is to clear the whole table. It
    is also, at some reaches, higher than this arm can point a gripper downward:
    table_z + 0.22 = 0.205 m, and the first real pick failed with no IK solution
    there while the block beneath it was reachable. So take the highest height
    the arm can hold, not the highest we would like.
    """
    want = max(float(perceived_z) + HOVER_CLEARANCE, ctx.cfg.table_z + APPROACH_HEIGHT_M)
    want = min(want, ctx.cfg.safety.workspace_z[1])
    ask = getattr(ctx.robot, "reachable", None)
    if not callable(ask):
        return np.array([float(xy[0]), float(xy[1]), want])

    # Search all the way down to the object, not to perceived_z + HOVER_CLEARANCE.
    # That floor is a comfort margin, and comfort is not always available: at
    # (0.458, 0.118) nothing above 0.05 m can be reached at all, while the block
    # itself can. Stopping at the margin and returning it anyway produced a
    # height that was never checked — which is how a hover at 0.086 m got
    # logged as "lowered to a reachable height" and then failed IK.
    lowest = float(perceived_z)
    z = want
    while z >= lowest - 1e-9:
        if ask(arm, np.array([float(xy[0]), float(xy[1]), z])):
            if z < want - 1e-6:
                ctx.log.event("approach_height_lowered", arm=arm,
                              wanted=round(want, 3), used=round(z, 3),
                              clearance_mm=round((z - float(perceived_z)) * 1000),
                              xy=[round(float(xy[0]), 3), round(float(xy[1]), 3)])
            return np.array([float(xy[0]), float(xy[1]), z])
        z -= 0.01
    raise ValueError(
        f"nothing above ({xy[0]:.3f}, {xy[1]:.3f}) is reachable between "
        f"z={lowest:.3f} and z={want:.3f} — the object is outside this arm's "
        f"workspace, not merely awkward to approach")


# Radius around the destination xy over which the wrist camera's depth is read.
# Wide enough to survive a few millimetres of aim error, narrow enough not to
# catch the object next to the target.
WRIST_PROBE_RADIUS_M = 0.025
# How far past the jaws' half-width the held object may stick out sideways.
# The jaw separation measures one axis of the cargo; the other is unknown, and
# a block's diagonal adds ~40% to its half-width. Generous, because a cargo
# pixel that survives into the annulus poisons the percentile from above.
HELD_CARGO_SLACK_M = 0.025
WRIST_MIN_POINTS = 40
# How far below the tool frame's origin a point still counts as the gripper
# rather than as the world. Small: on this arm the whole mount sits above the
# tool origin, and cutting deeper would start discarding the surface itself
# whenever the hover is low.
WRIST_TOOL_MARGIN_M = 0.005
# What a surface height is allowed to be, relative to the table. Anything else
# is a measurement failure wearing a plausible-looking number.
MAX_OBJECT_HEIGHT_M = 0.15
MAX_SURFACE_BELOW_TABLE_M = 0.02
# Release this far above the measured surface: enough not to press the object
# into it, small enough that it does not fall.
PLACE_RELEASE_GAP_M = 0.008

# Letting go at the full 80 mm aperture throws each jaw 40 mm out from the tool
# centre, through whatever is already on the support. Open only as far as the
# object needs to fall out of.
RELEASE_JAW_CLEARANCE_M = 0.012
# Objects sent to the same container were sent to the same POINT, so the second
# landed on the first. Measured in the MuJoCo twin, with ground-truth positions
# so perception could not be blamed: each block landed 2-4 mm from the target
# centre, and then the next placement knocked it to 35-50 mm. Spread them.
PLACE_SLOT_MARGIN_M = 0.006
# How much of a support's own half-width a learned aiming correction may spend.
# A systematic bias is a real thing and worth carrying between episodes; a
# correction that can push the aim off the edge of the thing it is aiming at is
# not a correction, it is a measurement problem wearing one.
PLACE_HINT_MAX_FRACTION = 0.5


def _usable_place_hint(ctx, target: str, hint: tuple[float, float]) -> tuple[float, float]:
    """A learned aiming correction, or nothing if it is too big to be one.

    A CORRECTION LARGER THAN THE TARGET AIMS OFF THE TARGET. Measured, and the
    cause of every remaining failure in the eight-episode run: the stored offset
    for the grey tray was (31, 66) mm against a tray whose half-width is 55 mm,
    so every placement on it was aimed clean off the edge — 72 mm out, against
    an `on` tolerance of 30 mm — and the residual could never come back.

    How it got there is the interesting part, because it is the other bugs. The
    residual is measured by grounding the object after a failed place, which is
    exactly when the arm is hovering over it; the detector returned the OTHER
    red block, and the distance to a lookalike was folded into the aim as though
    it were a systematic bias. The obstruction guard now refuses that reading, so
    the poison cannot be brewed again — but a bound that is a fraction of the
    support rather than a flat 80 mm is what makes it undrinkable.
    """
    dx, dy = float(hint[0]), float(hint[1])
    if not dx and not dy:
        return 0.0, 0.0
    span = _visual_span(ctx, target)
    if span is None:
        return dx, dy      # nothing to judge it against; the old bound stands
    limit = PLACE_HINT_MAX_FRACTION * 0.5 * float(span)
    size = float(np.hypot(dx, dy))
    if size <= limit:
        return dx, dy
    ctx.log.event("place_hint_refused", target=target,
                  hint_mm=[round(dx * 1000, 1), round(dy * 1000, 1)],
                  limit_mm=round(limit * 1000, 1), span_mm=round(float(span) * 1000, 1),
                  note="a correction this large aims off the support it is correcting for")
    return 0.0, 0.0


def release_width(held_width_m: float | None) -> float:
    """How wide to open to let go of something that measures `held_width_m`."""
    if not held_width_m or held_width_m <= 0.0:
        return OPEN_WIDTH     # nothing measured: the old behaviour, unchanged
    return float(min(OPEN_WIDTH, float(held_width_m) + 2 * RELEASE_JAW_CLEARANCE_M))


# Ring directions in the order they are tried, starting with the one that points
# back toward the base. On this arm reach runs out first in +x — a 47 mm offset
# straight outward was refused by the kinematics at both containers — so trying
# the outward direction first spends the whole search on the one place it cannot
# go.
SLOT_ANGLES_RAD = tuple(np.radians([180, 120, 240, 60, 300, 0]))


def slot_offsets(step: float, rings: int = 2) -> list[np.ndarray]:
    """Candidate positions on a support, best first, relative to its centre.

    The centre, then rings of six. Neighbours on a ring are `step` apart and the
    ring is `step` from the centre, so `step` is the only quantity a caller has
    to get right: no two candidates are ever closer together than that.
    """
    out = [np.zeros(2)]
    for ring in range(1, int(rings) + 1):
        radius = step * ring
        for a in SLOT_ANGLES_RAD:
            out.append(np.array([radius * np.cos(a), radius * np.sin(a)]))
    return out


def tool_floor(ctx: SkillContext) -> float:
    """Lowest height the tool may be commanded to.

    The raw workspace box is a metre below a tabletop scene, so a descent that
    trusted it would drive the tool through the table.
    """
    return max(ctx.cfg.safety.workspace_z[0], ctx.cfg.table_z - 0.03)


# A corner both fixed cameras ignore, in the ARM'S OWN frame. Sent as a world
# point it means "go to the right arm's corner", which for the left arm is deep
# inside its dead zone. Agent._clear_view_for_goal uses the same pose for the
# same reason: an arm parked over its own work hides the thing it just did.
VIEWING_PARK_XYZ = (0.16, -0.26, 0.14)


class _Phases:
    """Stopwatch for a skill's own steps, so 'the arm was idle' stops being a
    guess. The journal showed a 16.5-second gap inside one pick with nothing
    logged in it; whether that was travel, a gripper ramp or a service call
    could not be read off, and the two have opposite fixes.
    """

    def __init__(self, ctx: SkillContext, skill: str, entity: str) -> None:
        self.ctx, self.skill, self.entity = ctx, skill, entity
        self.t0 = time.perf_counter()
        self.marks: list[tuple[str, float]] = []
        self._publish()

    def _publish(self) -> None:
        """Hang the live stopwatch where a watcher can read it.

        The journal already carries this AFTER the skill finishes, which is
        precisely too late for the question a person asks while looking at a
        motionless arm: is it stuck, or is it descending 5 mm at a time? Those
        look identical on a camera and differ by one word here.
        """
        self.ctx.live_phase = {
            "skill": self.skill, "entity": self.entity,
            "done": self.marks[-1][0] if self.marks else None,
            "since": self.marks[-1][1] if self.marks else 0.0,
            "t0": self.t0,
        }

    def mark(self, what: str) -> None:
        cb = self.ctx.scratch.get("interrupt_check")
        if cb:
            cb()      # a human verdict stops the episode at the next phase
        now = time.perf_counter()
        prev = self.marks[-1][1] if self.marks else 0.0
        self.marks.append((what, now - self.t0))
        self.ctx.log.event("phase", skill=self.skill, entity=self.entity,
                           phase=what, seconds=round(now - self.t0 - prev, 2))
        self._publish()

    def done(self) -> None:
        self.ctx.log.event("phases", skill=self.skill, entity=self.entity,
                           total_s=round(time.perf_counter() - self.t0, 2),
                           marks=[(w, round(t, 2)) for w, t in self.marks])
        self.ctx.live_phase = None


def park_for_look(ctx: SkillContext, side: str) -> bool:
    """Get an arm out of the cameras' way before measuring what it just did.

    A skill that acts and then looks is looking past its own forearm. The push
    loop learned this the expensive way: after each sweep the tool sits over the
    object it just moved, and re-grounding the green plate returned z = 79 mm
    and then 121 mm — the arm's own green trim, twice, once close enough to the
    plausible band to be believed and used to re-aim.
    """
    try:
        t = np.asarray(ctx.cfg.arms[side].t_world_base, dtype=float)
        park = (t @ np.array([*VIEWING_PARK_XYZ, 1.0]))[:3]
        ctx.robot.move_cartesian(side, park, seconds=1.2)
        return True
    except Exception as e:
        ctx.log.event("park_for_look_failed", arm=side,
                      error=f"{type(e).__name__}: {e}"[:120])
        return False


def grasp_height(top: float, support: Optional[float], floor: float,
                 trim: float = 0.0) -> float:
    """How low the fingertips are commanded, given what the object stands on.

    The bite is GRASP_DEPTH below the perceived top — an approximation of "grip
    it in the middle" that holds while the middle of an object on the table is
    20 mm down. It stops holding the moment the object is standing on something.
    Measured, rig batch 2026-08-06 ep8: block in a paper plate grounded at
    z = 12.7 mm, plate surface at -7 mm, 20 mm below the block's top is -7.3 mm,
    clipped by the workspace floor to -5 mm — TWO MILLIMETRES above the plate.
    The fingers closed at the block's base, where the plate is, and took both.

    So on a support the bite is half of what stands proud of it, which is what
    "the middle" means when the floor is not the table, and the floor itself
    rises to the support. On the table nothing changes: support is None and the
    arithmetic is the one that has always run.
    """
    bite = GRASP_DEPTH
    if support is not None:
        bite = min(bite, max(0.0, 0.5 * (top - support)))
        floor = max(floor, support)
    return max(top - bite + trim, floor)


def _support_key(target: Optional[str], dest: np.ndarray) -> str:
    """One support, one key, whichever way the destination was named.

    The agent path names the container; the dataflow path hands a bare position
    in. Both have to land in the same bucket or the spread starts over from the
    centre halfway through the task.
    """
    if target:
        return f"entity:{target}"
    return "xy:%.2f,%.2f" % (float(dest[0]), float(dest[1]))


def forget_placement(ctx: SkillContext, entity: str) -> None:
    """The entity left whatever support it was on; its slot is free again.

    Called by `pick`. Without this the ledger below turned every repair into a
    spiral: place, fail to verify, re-pick the SAME block, re-place — and the
    spread treated the block's own first placement as an occupant to avoid,
    aiming 47 mm off a centre that was actually empty.
    """
    for key, occupants in ctx.scratch.get("placed_on", {}).items():
        if entity in occupants:
            del occupants[entity]
            ctx.log.event("place_slot_freed", support=key, entity=entity)



def _mask_yaw(ctx: SkillContext, entity: str, grasp) -> float | None:
    """Wrist yaw that closes the fingers across the object's NARROW side.

    The geometric path grasped everything with a fixed wrist: a standing tin
    got the jaws along its wide face and the grasp relied on luck. The depth
    mask around the believed position gives the object's principal axis, and
    with the finger line at yaw 0 lying along world y, the wrist rotation
    that closes across the minor axis is numerically the major-axis angle,
    folded to (-pi/2, pi/2]. Round objects (no dominant axis) return None
    and keep the default wrist.
    """
    try:
        from .sensing import _mask_from_depth  # noqa: PLC0415
        cam = next((n for n, c in ctx.cfg.cameras.items()
                    if c.kind == "overhead"
                    and n in getattr(ctx.robot, "cameras", [])), None)
        if cam is None:
            return None
        frame = ctx.robot.capture(cam)
        proj = getattr(frame, "proj", None)
        if proj is None or frame.depth is None:
            return None
        p = proj @ np.array([grasp[0], grasp[1], grasp[2], 1.0])
        if abs(p[2]) < 1e-9:
            return None
        u, v = int(p[0] / p[2]), int(p[1] / p[2])
        h, w = frame.depth.shape[:2]
        r = 70
        bb = (max(u - r, 0), max(v - r, 0), min(u + r, w - 1), min(v + r, h - 1))
        mask = _mask_from_depth(frame, bb, ctx.cfg.table_z)
        if mask is None or int(mask.sum()) < 60:
            return None
        vs, us = np.nonzero(mask)
        pts = np.stack([us - us.mean(), vs - vs.mean()]).astype(float)
        cov = pts @ pts.T / len(us)
        evals, evecs = np.linalg.eigh(cov)
        if evals[1] < 2.5 * evals[0]:
            return None      # roundish — no axis worth aligning to
        du, dv = evecs[:, 1]
        planar = getattr(frame, "deproject_planar", None)
        if not callable(planar):
            return None
        c0 = planar(int(us.mean()), int(vs.mean()))
        c1 = planar(int(us.mean() + du * 40), int(vs.mean() + dv * 40))
        if c0 is None or c1 is None:
            return None
        yaw = float(np.arctan2(c1[1] - c0[1], c1[0] - c0[0]))
        while yaw <= -np.pi / 2:
            yaw += np.pi
        while yaw > np.pi / 2:
            yaw -= np.pi
        return yaw
    except Exception:
        return None


def _seed_perceived_occupants(ctx: SkillContext, dest: np.ndarray,
                              entity: str, target: str) -> None:
    """The spread ledger records what this RUN put on the support; anything an
    EARLIER episode left there is invisible to it — an orange was set down dead
    centre on a red block a previous test had parked on the plate, twice. Look
    once at place time: proposal masks whose centroid deprojects inside the
    support's extent, near table height, are occupants, whoever put them there.
    """
    seg = getattr(ctx, "segmenter", None)
    if seg is None or not getattr(seg, "enabled", False):
        return
    tr = ctx.belief.track(target)
    span = float(getattr(tr, "span_m", None) or 0.14)
    try:
        cam = "cam_high" if "cam_high" in ctx.robot.cameras else ctx.robot.cameras[0]
        frame = ctx.robot.capture(cam)
        masks = seg.propose(frame.rgb, max_masks=40)
    except Exception:
        return
    h, w = frame.rgb.shape[:2]
    occ = ctx.scratch.setdefault("placed_on", {}).setdefault(
        _support_key(target, dest), {})
    n = 0
    for m in masks:
        box = m.get("box")
        area = float(m.get("pixels") or 0)
        if not box or not (150 <= area <= 0.05 * h * w):
            continue
        cx, cy = (box[0] + box[2]) // 2, (box[1] + box[3]) // 2
        pt = frame.deproject(int(cx), int(cy))
        if pt is None:
            continue
        d = float(np.hypot(pt[0] - dest[0], pt[1] - dest[1]))
        # Inside the support, not the support's own mask centre, and not
        # something in the air (the held object riding the gripper).
        if 0.02 < d < span / 2 * 0.9 and float(pt[2]) < ctx.cfg.table_z + 0.06:
            occ[f"seen_{n}"] = np.asarray([float(pt[0]), float(pt[1])])
            n += 1
    if n:
        ctx.log.event("support_occupants_seen", target=target, count=n)


def _spread_within_support(ctx: SkillContext, side: str, dest: np.ndarray,
                           entity: str, target: Optional[str], at) -> tuple[np.ndarray, int]:
    """Move the aim off the support's centre when something ELSE is already there.

    Returns (aim, slot). Slot 0 is the centre and means the support was empty as
    far as this episode knows — which is the only thing it can know: nothing here
    looks at the support, it remembers what this run put there.

    The ledger records WHO sits where, not merely that somewhere is taken.
    Measured before it did: a block was placed, failed to verify, was re-picked
    and re-placed — and the spread dodged the block's own previous position,
    aiming at slot 1, 47 mm out. `slot_step` (47 mm for a block) is outside both
    `verify.ON_XY_MAX` (30 mm) and a tray cavity's `in` bound (33 mm), so that
    placement could never verify, the repair loop re-placed it again, and each
    pass drifted further. Avoiding yourself is how the avoidance became the
    failure it was avoiding.
    """
    key = _support_key(target, dest)
    ledger = ctx.scratch.setdefault("placed_on", {})
    occupants: dict[str, np.ndarray] = ledger.setdefault(key, {})
    occupants.pop(entity, None)      # what is in the gripper is not on the support
    if not occupants:
        occupants[entity] = np.zeros(2)
        return dest, 0
    taken = list(occupants.values())

    held = ctx.robot.get_gripper(side).get("width_m")
    step = slot_step(held, release_width(held))
    # A support we have measured cannot be spread over further than it goes.
    span = getattr(at, "span_m", None)
    if span:
        room = 0.5 * float(span) - 0.5 * float(held or 0.0) - PLACE_SLOT_MARGIN_M
        if room <= 0.0:
            ctx.log.event("place_slot_no_room", support=key, span_m=round(float(span), 4),
                          note="the support is not wide enough for a second object; "
                               "aiming at the centre anyway")
            return dest, len(taken)
        step = min(step, room)

    ask = getattr(ctx.robot, "reachable", None)
    # Ask about a height the tool will actually be sent to. `dest[2]` is the
    # SUPPORT's own surface — a plate reads -0.006, below the workspace floor —
    # so asking there answers "no" everywhere and every slot looks unreachable,
    # which is exactly what the first version of this did at both containers.
    probe_z = max(float(dest[2]) + PLACE_RELEASE_GAP_M, tool_floor(ctx))
    for off in slot_offsets(step):
        if any(float(np.linalg.norm(off - t)) < step - 1e-6 for t in taken):
            continue
        aim = dest + np.array([off[0], off[1], 0.0])
        # An offset outside the arm's envelope is worse than a crowded support:
        # it turns a mediocre placement into a failed step.
        if callable(ask) and not ask(side, np.array([aim[0], aim[1], probe_z])):
            continue
        occupants[entity] = off
        ctx.log.event("place_slot", support=key, slot=len(occupants) - 1,
                      step_mm=round(step * 1000, 1),
                      offset_mm=[round(float(v) * 1000) for v in off],
                      beside=sorted(k for k in occupants if k != entity))
        return aim, len(occupants) - 1

    ctx.log.event("place_slot_none_reachable", support=key, step_mm=round(step * 1000, 1),
                  occupied=len(occupants),
                  note="every free slot on this support is outside the arm's envelope; "
                       "aiming at the centre, where something already is")
    return dest, len(occupants)


def slot_step(object_width_m: float | None, aperture_m: float) -> float:
    """Centre-to-centre spacing that keeps a descending jaw off its neighbour.

    The jaw swings `aperture/2` out from the tool centre and the neighbour
    occupies its own half-width, so anything less than the sum of those is a
    collision dressed up as a placement.
    """
    half_object = 0.5 * float(object_width_m or 0.0)
    return 0.5 * float(aperture_m) + half_object + PLACE_SLOT_MARGIN_M


def surface_height_below(ctx: SkillContext, arm: str, xy) -> tuple[float | None, str]:
    """Height of whatever is under (x, y), read from that arm's wrist camera.

    This is the measurement contact-descent was substituting for. The overhead
    camera sits 1.04 m up — outside the D405's 7-50 cm specification — and on a
    glossy plate its depth scatters 25.5 mm with points 84 mm BELOW the table.
    The wrist camera at the hover pose is 10-20 cm away, inside spec, looking
    down the approach axis, and its specular geometry is different from the
    overhead's view of the ceiling light.

    Returns (z, note). z is None when the camera has no depth, no extrinsics, or
    too few returns near the target — in which case the caller must fall back
    rather than descend onto a number nobody measured.
    """
    cam = next((n for n, c in ctx.cfg.cameras.items()
                if c.kind == "wrist" and c.arm == arm), None)
    if cam is None or cam not in getattr(ctx.robot, "cameras", []):
        return None, f"no wrist camera configured for the {arm} arm"
    try:
        frame = ctx.robot.capture(cam)
    except Exception as e:
        return None, f"{cam} capture failed: {type(e).__name__}: {e}"
    if frame.depth is None or frame.intrinsics is None or frame.t_base_cam is None:
        return None, f"{cam} has no depth or no hand-eye transform"

    h, w = frame.depth.shape[:2]
    vs, us = np.mgrid[0:h, 0:w]
    z = frame.depth
    ok = np.isfinite(z) & (z > 0.03) & (z < 1.0)
    if ok.sum() < WRIST_MIN_POINTS:
        return None, f"{cam} returned almost no depth ({int(ok.sum())} px)"
    fx, fy = frame.intrinsics[0, 0], frame.intrinsics[1, 1]
    cx, cy = frame.intrinsics[0, 2], frame.intrinsics[1, 2]
    zz = z[ok].astype(float)
    pts = np.stack([(us[ok] - cx) * zz / fx, (vs[ok] - cy) * zz / fy, zz, np.ones_like(zz)])
    world = (frame.t_base_cam @ pts)[:3]

    tool = np.asarray(ctx.robot.get_cartesian(arm), dtype=float)
    # THE HELD OBJECT HANGS EXACTLY WHERE THE PROBE LOOKS. During a place the
    # gripper holds the thing being placed, directly under the camera and over
    # the aim point — so the probe measured the CARGO's top, not the support.
    # Measured in the twin, hovering at 0.17 with a block in the jaws: surface_z
    # came back 0.1605, the block's own top, was refused as implausible (rightly),
    # and every place then fell back to feeling its way down. The wrist-depth
    # path never once ran on the twin for exactly the motion it was built for.
    #
    # The gripper cut above the tool origin cannot see this: the cargo is BELOW
    # the origin. Cut its footprint around the TOOL xy instead — it rides with
    # the tool, not with the aim — and widen the window so an annulus of real
    # surface remains. On a plate the annulus is still plate; on a walled vessel
    # it is the rim, which is the safer height to release above anyway.
    grip_of = getattr(ctx.robot, "get_gripper", None)
    grip = grip_of(arm) if callable(grip_of) else {}
    held_w = float(grip.get("width_m") or 0.0)
    excl = 0.0
    if held_w > CLOSED_ON_AIR_WIDTH:
        excl = 0.5 * held_w + HELD_CARGO_SLACK_M
    outer = WRIST_PROBE_RADIUS_M + excl
    near = (np.abs(world[0] - float(xy[0])) < outer) & \
           (np.abs(world[1] - float(xy[1])) < outer)
    if excl > 0.0:
        near &= np.hypot(world[0] - tool[0], world[1] - tool[1]) > excl
    if near.sum() < WRIST_MIN_POINTS:
        # Say where the tool actually WAS. This camera is mounted 55 mm off the
        # approach axis with a 40-degree field, so at a 14 cm hover it covers
        # barely 10 cm of table: a tool that ended up a few centimetres from
        # where it was sent is looking somewhere else entirely, and "saw only 10
        # points" on its own gives no way to tell that from a depth failure.
        off = float(np.linalg.norm(tool[:2] - np.asarray(xy, dtype=float)))
        return None, (f"{cam} saw only {int(near.sum())} points within "
                      f"{WRIST_PROBE_RADIUS_M * 1000:.0f} mm of the target; the "
                      f"tool was {off * 1000:.0f} mm from the point asked about "
                      f"(at {np.round(tool[:2], 3).tolist()})")

    # THE GRIPPER IS IN ITS OWN CAMERA'S VIEW, and it is the highest thing
    # there. Measured in the twin, hovering 0.13 m over a block: 66% of the
    # probe window was gripper, between 0.158 and 0.185, while the block's top
    # was 0.015 and the table -0.015. Taking the 90th percentile of that returns
    # the mount, and `place` would then release from above where it started.
    # Nothing real can be up there: we are hovering ABOVE the surface we are
    # about to release onto, by construction.
    tool_z = float(tool[2])
    near &= world[2] < tool_z - WRIST_TOOL_MARGIN_M
    if near.sum() < WRIST_MIN_POINTS:
        return None, (f"{cam} saw only {int(near.sum())} points below the tool "
                      f"at z={tool_z:.3f} — the view is filled by the gripper")
    # The TOP of what is there: we are placing onto its surface, not into it.
    top = float(np.percentile(world[2][near], 90))

    # A surface has to be somewhere a surface can be. Measured on the rig: this
    # returned 0.204 m for a plate 2 cm tall — about the height of the camera
    # itself, which is what a bad extrinsic or an invalid depth looks like — and
    # place tried to release there. The safety box caught it, but only by
    # accident of the number being large; a wrong answer of 0.05 would have
    # passed and dropped the block from 5 cm.
    lo = ctx.cfg.table_z - MAX_SURFACE_BELOW_TABLE_M
    hi = ctx.cfg.table_z + MAX_OBJECT_HEIGHT_M
    if not (lo <= top <= hi):
        ctx.log.event("wrist_surface_implausible", camera=cam, surface_z=round(top, 4),
                      band=[round(lo, 3), round(hi, 3)], table_z=ctx.cfg.table_z)
        return None, (f"{cam} measured the surface at z={top:.3f}, outside the "
                      f"{lo:.3f}..{hi:.3f} band a real surface can occupy above a "
                      f"table at {ctx.cfg.table_z:.3f} — bad depth or a bad "
                      f"extrinsic, not a surface")
    ctx.log.event("wrist_surface", camera=cam, arm=arm,
                  xy=[round(float(xy[0]), 3), round(float(xy[1]), 3)],
                  points=int(near.sum()), surface_z=round(top, 4),
                  spread_mm=round(float(np.std(world[2][near])) * 1000, 1))
    return top, f"{cam} measured the surface at z={top:.3f} from {int(near.sum())} points"


def _move(ctx: SkillContext, arm: str, xyz: np.ndarray, seconds: float,
          orientation=None) -> None:
    """move_cartesian that only mentions orientation when there is one, so
    every duck-typed backend without a wrist keeps its narrow signature."""
    if orientation is None:
        ctx.robot.move_cartesian(arm, xyz, seconds=seconds)
    else:
        ctx.robot.move_cartesian(arm, xyz, seconds=seconds, orientation=orientation)


def _service_grasp(ctx: SkillContext, entity: str, target: np.ndarray):
    """Best in-envelope proposal from the grasp service, or None with the
    reason logged. Returns (xyz, yaw_rad, score).

    Every exit that is not a usable proposal logs why and returns None: the
    caller's heuristic path is the contract, and a dead tunnel, a slow GPU, or
    a service with opinions about the wrong object must all cost one log line
    and nothing else.
    """
    url = ctx.cfg.grasp_service_url
    if not url:
        return None
    from ..perception.graspclient import GraspServiceClient, GraspServiceError  # noqa: PLC0415

    cam = next((n for n, c in ctx.cfg.cameras.items()
                if c.kind == "overhead" and n in getattr(ctx.robot, "cameras", [])), None)
    if cam is None:
        ctx.log.event("grasp_service_unavailable", entity=entity,
                      reason="no overhead camera on this rig")
        return None
    try:
        frame = ctx.robot.capture(cam)
    except Exception as e:
        ctx.log.event("grasp_service_unavailable", entity=entity,
                      reason=f"{cam} capture failed: {type(e).__name__}: {e}")
        return None
    # Every other calibrated fixed camera joins in: one overhead view shows a
    # sphere as a cap and a flat tin as a sliver, and M2T2 ranked neighbours
    # over the object in every rig episode of 2026-08-05. Fusion is free —
    # the second camera is already capturing.
    frames = [frame]
    for extra in getattr(ctx.robot, "cameras", []):
        if extra == cam or "wrist" in extra:
            continue
        try:
            f2 = ctx.robot.capture(extra)
            if f2.depth is not None and f2.t_base_cam is not None:
                frames.append(f2)
        except Exception:
            pass
    if len(frames) > 1:
        ctx.log.event("grasp_service_views", entity=entity,
                      cameras=[f.camera for f in frames])
    frame = frames if len(frames) > 1 else frame
    client = ctx.scratch.get("grasp_client")
    if client is None or client.url != url.rstrip("/"):
        client = GraspServiceClient(url, timeout_s=ctx.cfg.grasp_service_timeout_s)
        ctx.scratch["grasp_client"] = client
    try:
        # max_grasps well above what the envelope keeps: the model's TOP-scored
        # grasps are sometimes all on a neighbour, and the on-target ones must
        # still be in the list when that happens (seen in the twin with two
        # blocks 62 mm apart).
        # The cloud's ceiling hugs the believed top: everything taller in the
        # crop is machinery (this rig's own forearm crosses the view at
        # 6-12 cm), and M2T2 will happily rank grasps on it above the block.
        # A deep candidate list in ONE pass: a 2 cm block owns a few hundred
        # points of the cloud and its grasps rank poorly against larger
        # structure — present 0-5 times in a top-50, reliably present deeper.
        # (Two passes were tried and blew the service-timeout budget.)
        proposals = client.propose(
            frame, center_xy=target[:2], radius_m=GRASP_SERVICE_CROP_RADIUS_M,
            z_range=(ctx.cfg.table_z - 0.02,
                     min(ctx.cfg.table_z + MAX_OBJECT_HEIGHT_M * 2,
                         float(target[2]) + 0.05)),
            num_runs=1, max_grasps=200)
        for cam, dz in getattr(client, "dropped_views", []) or []:
            ctx.log.event("grasp_view_dropped", entity=entity, camera=cam,
                          disagreement_mm=round(dz * 1000, 1),
                          reason="this view puts the surface somewhere else; "
                                 "fusing it would double the scene")
    except GraspServiceError as e:
        ctx.log.event("grasp_service_unavailable", entity=entity, reason=str(e)[:200])
        return None

    floor = ctx.cfg.table_z - MAX_SURFACE_BELOW_TABLE_M
    # A proposal well above the believed top is a grasp on something else
    # standing over the object — measured on the rig: with the wrist parked in
    # the camera's view corridor, M2T2's best "block" grasp was on the wrist
    # itself, 12 cm up, and xy alone could not tell them apart.
    ceiling = min(ctx.cfg.table_z + MAX_OBJECT_HEIGHT_M, float(target[2]) + 0.06)
    # The radius scales with the OBJECT: the model's TCP may sit anywhere on
    # a large object and still be a grasp of it, but on a 30 mm block a TCP
    # 25 mm out is a grasp of the table — the fused two-view cloud carries
    # 1-2 cm of registration residual, and two such proposals closed on air
    # in the first dual-arm episode while the geometric centre (2.4 mm
    # median) was available all along. Small things demand agreement.
    # UNKNOWN size demands a TIGHTER gate, not a looser one: an entity whose
    # position came from our own place (no measured span) got the permissive
    # default and a proposal 2.5 cm off sailed through — two air-closes on a
    # 3 cm cube the geometric centre then picked first try (bisected live).
    span_gate = (getattr(ctx.belief.track(entity), "span_m", None)
                 if getattr(ctx, "belief", None) is not None else None) or 0.03
    match_r = float(np.clip(span_gate * 0.5, 0.015, GRASP_SERVICE_MATCH_RADIUS_M))

    def _near_target(p) -> bool:
        # The TCP is what gets executed, so the TCP is what the radius gates.
        # (A looser "contact OR tcp" rule was tried and admitted a 20-degree
        # grasp whose contact grazed the block while its tcp — where the
        # fingers actually close on a vertical descent — sat 44 mm out.)
        return float(np.hypot(p.tcp_xyz[0] - target[0],
                              p.tcp_xyz[1] - target[1])) \
            <= match_r

    survivors = [
        p for p in proposals
        if p.tilt_deg <= GRASP_SERVICE_MAX_TILT_DEG
        and _near_target(p)
        and floor <= float(p.tcp_xyz[2]) <= ceiling
    ]
    if not survivors:
        # Say what was CLOSEST to acceptable, not just that nothing was: the
        # tuning of this envelope lives or dies on knowing whether rejections
        # are misses by millimetres or proposals about a different object.
        nearest = sorted(proposals, key=lambda p: float(
            np.hypot(p.tcp_xyz[0] - target[0], p.tcp_xyz[1] - target[1])))[:3]
        ctx.log.event("grasp_service_rejected", entity=entity,
                      proposals=len(proposals),
                      target=[round(float(v), 4) for v in target],
                      nearest=[{"tcp": [round(float(v), 4) for v in p.tcp_xyz],
                                "tilt_deg": round(p.tilt_deg, 1),
                                "score": round(p.score, 3)} for p in nearest],
                      reason="no proposal is near-top-down, near the target, "
                             "and at a plausible height")
        return None
    best = max(survivors, key=lambda p: p.score)
    if best.score < GRASP_SERVICE_MIN_SCORE:
        # A weak proposal is worse than the heuristic: score 0.32 at 70 deg
        # yaw spun the wrist and closed on air. Below the bar, decline.
        ctx.log.event("grasp_service_low_score", entity=entity,
                      score=round(best.score, 3))
        return None
    # Where to CLOSE: the proposal's tcp is a shallow pinch at the surface
    # (measured on a 2 cm block: close point at the top edge, and any xy error
    # squeezes the object out). The heuristic's proven recipe is "surface
    # minus GRASP_DEPTH"; apply the same bite anchored on the model's own
    # surface evidence — the contact point when given, the tcp otherwise.
    anchor = float(best.tcp_xyz[2])
    if best.contact is not None:
        anchor = min(anchor, float(best.contact[2]))
    # ...and never above the believed top. Measured on the rig: a depth-noise
    # tower put the winning contact 2 cm over the block, the close happened at
    # top-edge height, and the pads took nothing. The grounding's own surface
    # estimate caps the anchor, so the service close is never SHALLOWER than
    # the heuristic close that already works — the proposal contributes its
    # xy and yaw, and its depth only when it says to go deeper.
    anchor = min(anchor, float(target[2]))
    ctx.log.event("grasp_service_proposal", entity=entity, camera=cam,
                  xyz=[round(float(v), 4) for v in best.tcp_xyz],
                  anchor_z=round(anchor, 4),
                  yaw_deg=round(float(np.degrees(best.yaw_rad)), 1),
                  tilt_deg=round(best.tilt_deg, 1), score=round(best.score, 3),
                  proposals=len(proposals), in_envelope=len(survivors),
                  latency_s=round(getattr(client, "last_latency_s", 0.0), 2))
    xyz = np.array([float(best.tcp_xyz[0]), float(best.tcp_xyz[1]), anchor])
    return xyz, best.yaw_rad, best.score


def descend_to_contact(ctx: SkillContext, arm: str, x: float, y: float, z_start: float,
                       z_floor: float, z_expected: float | None = None,
                       orientation=None, speed_mps: float = 0.02,
                       margin_n: float | None = None) -> tuple[float, bool]:
    """Lower the tool at (x, y) until contact. Returns (z_stop, touched).

    The arm feels for the surface with the driver's gravity/friction-compensated
    external effort, baselined in free space so the threshold is drift-proof.

    `z_expected` is where perception thinks the surface is. Given it, the tool
    drops there in ONE motion and only feels the last CONTACT_PROBE_M — the
    steps then cover the error in the estimate rather than the entire descent.
    Without it (a depth-free rig) the whole descent is felt, which is correct
    and slow.
    """
    z = z_start
    if z_expected is not None:
        z_fast = max(float(z_expected) + CONTACT_PROBE_M, z_floor)
        if z_fast < z_start - 1e-6:
            _move(ctx, arm, np.array([x, y, z_fast]), 1.2, orientation)
            z = z_fast
            ctx.log.event("fast_descent", arm=arm, from_z=round(z_start, 4),
                          to_z=round(z_fast, 4),
                          probing_m=round(CONTACT_PROBE_M, 3))
    # Baseline AFTER the fast move: it must be taken where the stepping starts,
    # in free space, or the threshold is compared against a different pose.
    if getattr(ctx.robot, "felt_descend", None) is not None and \
            getattr(getattr(ctx.robot, "_robot", None), "felt_descend", None) is not None:
        # Hardware path: one constant-velocity command, effort polled at 12 Hz,
        # halt on contact. The stepped loop below remains for sim and mock.
        floor_pt = np.array([x, y, max(z_floor, z - CONTACT_MAX_DESCENT_M)])
        z_stop, touched = ctx.robot.felt_descend(
            arm, floor_pt, speed_mps,
            CONTACT_MARGIN_N if margin_n is None else margin_n)
        ctx.log.event("felt_descend", arm=arm, z=round(z_stop, 4), touched=touched,
                      speed_mps=speed_mps,
                      margin_n=CONTACT_MARGIN_N if margin_n is None else margin_n)
        return z_stop, touched
    # Not IMMEDIATELY after the move: a reading taken while the arm still
    # decelerates measured 253 N against a resting 10 N, and no contact ridge
    # ever cleared a threshold parked 24x above reality (every twin stack
    # descent tonight said contact_not_found straight through the block).
    settle = getattr(ctx.robot, "settle", None)
    if callable(settle):
        settle(0.4)
    else:
        time.sleep(0.4)
    reads = [ctx.robot.get_external_effort(arm) for _ in range(3)]
    baseline = sorted(reads)[1]
    descended = 0.0
    while z - CONTACT_STEP_M >= z_floor and descended < CONTACT_MAX_DESCENT_M:
        z -= CONTACT_STEP_M
        descended += CONTACT_STEP_M
        _move(ctx, arm, np.array([x, y, z]), 0.4, orientation)
        effort = ctx.robot.get_external_effort(arm)
        if effort - baseline >= CONTACT_MARGIN_N:
            ctx.log.event("contact", arm=arm, z=round(z, 4), effort=round(effort, 2),
                          baseline=round(baseline, 2), descended_m=round(descended, 3))
            return z, True
    ctx.log.event("contact_not_found", arm=arm, z=round(z, 4), descended_m=round(descended, 3))
    return z, False


def travel(ctx: SkillContext, arm: str, target: np.ndarray, seconds: float = 2.0) -> None:
    """Move in segments that each respect the safety displacement cap.

    A WAYPOINT IS A TARGET AND HAS TO BE HOLDABLE. Interpolating between two
    reachable poses does not give reachable poses: from the ready pose
    (0.518, 0, 0.428) to a cup hover at 0.165 the midpoint lands high and far
    at (0.39, -0.045, 0.29), where this arm cannot point the gripper down at
    all — the safety layer refused it and the skill died between two points it
    could perfectly well have visited. Each waypoint is lowered to a height the
    arm can hold, the same rule approach_pose already applies to hovers.
    """
    target = np.asarray(target, dtype=float)
    start = ctx.robot.get_cartesian(arm)
    dist = float(np.linalg.norm(target - start))
    max_seg = ctx.cfg.safety.max_step_m * 0.8
    n = max(1, int(np.ceil(dist / max_seg)))
    ask = getattr(ctx.robot, "reachable", None)
    # A TRANSIT FLOOR, not the target's height. The first version refused to
    # lower a waypoint below the destination — and the destination was a hover
    # at 0.165 where the arm cannot hold a downward wrist mid-reach, while the
    # SAME xy is reachable at 0.12 and below. Transit clears the tallest thing
    # on this table (a 55 mm cup) with room to spare and stays inside the band
    # the arm can actually hold.
    floor = ctx.cfg.table_z + 0.08

    def _lowered(p):
        """The waypoint, dropped to a height the arm can hold, or None."""
        if not callable(ask) or ask(arm, p):
            return p
        z = float(p[2])
        while z > floor + 1e-6:
            z = max(floor, z - 0.02)
            q = np.array([p[0], p[1], z])
            if ask(arm, q):
                ctx.log.event("waypoint_lowered", arm=arm,
                              xy=[round(float(p[0]), 3), round(float(p[1]), 3)],
                              wanted=round(float(p[2]), 3), used=round(z, 3))
                return q
        return None

    def _waypoints(a, b, k):
        return [a + (b - a) * (j / k) for j in range(1, k + 1)]

    pts = _waypoints(start, target, n)
    pts = [(_lowered(p) if i < len(pts) - 1 else p) for i, p in enumerate(pts)]
    # IF YOU CANNOT GET THERE FROM HERE, GO HOME FIRST. The ready pose holds
    # the gripper HORIZONTAL, and every point near it is unholdable with the
    # downward approach these skills use — so a split journey from the ready
    # pose has no legal first waypoint at any height, and the skill died
    # between two poses it could each have visited. home() is a joint move
    # with no orientation constraint; from the home posture the same line is
    # ordinary.
    if n > 1 and any(p is None for p in pts[:-1]):
        try:
            ctx.log.event("travel_via_home", arm=arm,
                          from_xyz=[round(float(v), 3) for v in start],
                          to_xyz=[round(float(v), 3) for v in target])
            ctx.robot.home(arm)
            start = ctx.robot.get_cartesian(arm)
            dist = float(np.linalg.norm(target - start))
            n = max(1, int(np.ceil(dist / max_seg)))
            pts = _waypoints(start, target, n)
            pts = [(_lowered(p) if i < len(pts) - 1 else p)
                   for i, p in enumerate(pts)]
        except Exception as e:
            ctx.log.event("travel_home_failed", arm=arm,
                          error=f"{type(e).__name__}: {e}"[:120])
    pts = [p for p in pts if p is not None]
    for p in pts:
        ctx.robot.move_cartesian(arm, p, seconds=seconds / len(pts) if len(pts) > 1 else seconds)


def _resolve_point(ctx: SkillContext, entity: str, max_age_s: float | None = None) -> np.ndarray:
    tr = _belief(ctx).track(entity)
    if tr.xyz_base is None:
        raise ValueError(f"entity {entity!r} has no grounded location — run perceive first")
    if max_age_s is not None and tr.age() > max_age_s:
        raise ValueError(f"entity {entity!r} location is stale ({tr.age():.0f}s) — run perceive first")
    return np.array(tr.xyz_base, dtype=float)


class _NoBelief:
    """Stands in when a caller runs the primitives on values alone.

    The dataflow path hands positions in directly, so there is nothing to
    remember and nothing to look up. Rather than scatter `if ctx.belief` through
    every primitive, absorb the calls here: reads answer "nothing known", writes
    go nowhere. Anything that genuinely needs a belief still gets one from the
    agent, which always supplies it.
    """

    class _Track:
        # Whatever a real EntityTrack carries, this answers "not known". Listing
        # the fields explicitly beats __getattr__ returning None for typos.
        id = ""
        description = ""
        xyz_base = None
        camera = None
        px = None
        span_m = None
        carry_offset = None
        held_by = None
        confidence = 0.0
        notes = ""
        from_sighting = False
        t = 0.0

        def age(self):
            return float("inf")

    def track(self, _entity):
        return self._Track()

    def update_track(self, *a, **k):
        return self._Track()

    def invalidate_entity(self, *a, **k):
        pass

    @property
    def entities(self):
        return {}


def _belief(ctx: SkillContext):
    return ctx.belief if ctx.belief is not None else _NoBelief()


def _pinch_refine(ctx: SkillContext, side: str, grasp: np.ndarray,
                  orientation) -> np.ndarray:
    """Close the grasp aim through the overhead camera — no models involved.

    Pinch in the air over the aim: the fingertips are the only thing that
    moves between the open/closed frames (heron.robot.fingertip finds them),
    the pinch pixel ray-projects to the pinch height, and the aim shifts by
    the measured error, up to twice. This catches the POSITION-DEPENDENT
    fingertip offset a constant aim_trim cannot: measured on the left arm,
    -13 mm at r~0.25-0.35 growing past -20 mm near the base — first grasps
    kept closing on air by exactly the trim residual. Costs ~8 s per pick;
    enabled per arm via cfg.pinch_refine_arms."""
    if side not in list(getattr(ctx.cfg, "pinch_refine_arms", None) or []):
        return grasp
    try:
        from ..robot.fingertip import pinch_point_from_pair  # noqa: PLC0415
        from ..robot.workspace import pose_from_plane_homography  # noqa: PLC0415
    except ImportError:
        return grasp
    cam = next((n for n, c in ctx.cfg.cameras.items()
                if c.kind == "overhead"), None)
    if cam is None:
        return grasp
    zpin = 0.048
    target = np.asarray(grasp[:2], float)
    aim = target.copy()
    corrected = False
    for it in range(2):
        _move(ctx, side, np.array([aim[0], aim[1], zpin]), 1.5, orientation)
        ctx.robot.set_gripper(side, 0.06)
        time.sleep(0.6)
        fa = ctx.robot.capture(cam)
        ctx.robot.set_gripper(side, 0.0)
        time.sleep(0.7)
        fb = ctx.robot.capture(cam)
        hit = pinch_point_from_pair(fa.rgb, fb.rgb)
        if hit is None:
            ctx.log.event("pinch_refine_missed", arm=side, iteration=it)
            break
        u, v, conf = hit
        hom = getattr(fb, "h_pixel_world", None)
        intr = getattr(fb, "intrinsics", None)
        if hom is None or intr is None:
            ctx.log.event("pinch_refine_missed", arm=side, iteration=it,
                          note="frame carries no homography")
            break
        pz = float(getattr(fb, "plane_z", 0.0) or 0.0)
        pose = pose_from_plane_homography(np.asarray(hom, float),
                                          np.asarray(intr, float), pz)
        cc = pose[:3, 3]
        p0 = np.asarray(hom, float) @ np.array([float(u), float(v), 1.0])
        p0 = np.array([p0[0] / p0[2], p0[1] / p0[2], pz])
        s = (zpin - cc[2]) / (p0[2] - cc[2])
        q = (cc + s * (p0 - cc))[:2]
        err = q - target
        ctx.log.event("pinch_refine", arm=side, iteration=it,
                      conf=round(float(conf), 2),
                      err_mm=[round(float(err[0]) * 1000, 1),
                              round(float(err[1]) * 1000, 1)])
        if abs(err[0]) > 0.08 or abs(err[1]) > 0.08:
            ctx.log.event("pinch_refine_declined", arm=side,
                          note="error exceeds the 80 mm trust bound")
            break
        aim = aim - err
        corrected = True
        if abs(err[0]) < 0.004 and abs(err[1]) < 0.004:
            break
    ctx.robot.set_gripper(side, OPEN_WIDTH)
    time.sleep(0.4)
    if not corrected:
        return grasp
    out = np.asarray(grasp, float).copy()
    out[0], out[1] = float(aim[0]), float(aim[1])
    return out


def _radial_sag_comp(ctx: SkillContext, side: str, p):
    """Aim past the reach-dependent fingertip sag (rig LAWS L14).

    Under the gripper's weight the fingertips land SHORT radially: ~16 mm at
    r=0.42-0.47 from the base, unmeasurably small inside the comfort band,
    measured on BOTH arms by ring search. Deterministic physics, zero
    runtime cost — the simple chain's answer to grasps that closed on air
    twice at r=0.39-0.42 while r=0.30 grasps carried 0.3 mm off centre."""
    k = float(getattr(ctx.cfg, "sag_comp_m_per_m", 0.0) or 0.0)
    if k <= 0.0:
        return p
    r0 = float(getattr(ctx.cfg, "sag_comp_r0_m", 0.30) or 0.30)
    base = np.asarray(ctx.cfg.arms[side].t_world_base, float)[:2, 3]
    v = np.asarray(p[:2], float) - base
    r = float(np.linalg.norm(v))
    if r <= r0 or r < 1e-9:
        return p
    out = np.asarray(p, float).copy()
    comp = k * (r - r0)
    out[0] += v[0] / r * comp
    out[1] += v[1] / r * comp
    ctx.log.event("radial_sag_comp", arm=side, r=round(r, 3),
                  comp_mm=round(comp * 1000, 1))
    return out


def choose_arm(ctx: SkillContext, xyz: np.ndarray, requested: str = "auto") -> str:
    arms = list(getattr(ctx.cfg, "active_arms", None) or ctx.robot.arms)
    if requested in arms:
        return requested
    if len(arms) == 1:
        return arms[0]
    # AN ARM NAMED IN THE INSTRUCTION IS AN ORDER. The planner does not
    # reliably copy "with the left arm" into the plan's arm fields (two
    # episodes watched, 2026-08-18: the fields simply weren't emitted), so
    # the operator's words are honoured here, above every heuristic.
    text = str(getattr(ctx, "instruction", "") or "").lower()
    wants_left = any(w in text for w in ("left arm", "左臂", "左手"))
    wants_right = any(w in text for w in ("right arm", "右臂", "右手"))
    if wants_left != wants_right:
        forced = "left" if wants_left else "right"
        if forced in arms:
            ctx.log.event("arm_forced_by_instruction", arm=forced)
            return forced
    # REACH decides, not a halves convention. The y-halves rule assumed the
    # stationary kit's mounting (arms across y, origin mid-table); this rig's
    # arms face each other across X — right base at the origin, left at
    # x=0.93 — and the convention sent a centre-table pick at (0.36, 0.02)
    # to the left arm, which cannot get there at any height. Each arm's own
    # calibrated IK answers; among the willing, the nearer base wins.
    def _base_xy(a):
        t = np.asarray(ctx.cfg.arms[a].t_world_base, dtype=float)
        return t[:2, 3]

    hover = (float(xyz[0]), float(xyz[1]),
             max(float(xyz[2]), ctx.cfg.table_z) + 0.05)
    willing = []
    for a in arms:
        try:
            if ctx.robot.reachable(a, hover):
                willing.append(a)
        except Exception:
            pass
    pool = willing or arms
    return min(pool, key=lambda a: float(
        np.linalg.norm(_base_xy(a) - np.asarray(xyz[:2], dtype=float))))


def _is_flat_support(ctx: SkillContext, entity: str) -> bool:
    """Whether the destination has walls to catch a falling object."""
    tr = _belief(ctx).track(entity)
    text = f"{tr.description} {entity}".lower()
    return any(word in text for word in FLAT_SUPPORT_WORDS)


def _visual_span(ctx: SkillContext, entity: str) -> float | None:
    """Width of the entity on the table, from its bounding box. None if unknown.

    Grounding already drew that box and now records what it measured, so the
    usual path costs nothing. The detector call below is the fallback for an
    entity whose position came from somewhere that had no box — dead reckoning
    after a place, or a caller passing a bare position in.

    It used to be the only path: a fresh capture and a fresh detector call on
    every pick, for a width the grounding a tenth of a second earlier had
    already computed and discarded. Measured over a 60-episode sweep, 511
    detector calls and 1166 seconds, all of it on the critical path of a grasp.
    """
    tr = _belief(ctx).track(entity)
    if tr.span_m is not None:
        return float(tr.span_m)
    if tr.camera is None or ctx.grounder is None:
        return None
    try:
        frame = ctx.robot.capture(tr.camera)
        bb = ctx.grounder.box(frame, tr.description)
    except Exception:
        return None
    if bb is None:
        return None
    p0 = frame.deproject(bb[0], bb[3])
    p1 = frame.deproject(bb[2], bb[3])
    if p0 is None or p1 is None:
        return None
    return float(np.linalg.norm(np.asarray(p1[:2]) - np.asarray(p0[:2])))


WRIST_REFINE_MAX_M = 0.06  # a larger "correction" is a different object


def _wrist_refine(ctx: SkillContext, side: str, entity: str,
                  grasp: np.ndarray) -> np.ndarray:
    """Close the loop on the grasp xy from the wrist camera at the hover.

    The overhead camera aims the grasp from a metre away through a fitted
    homography; its open-loop error budget is centimetres and it drifts with
    every remount (measured 2026-08-17: fingertips on the block's corner, not
    its middle). The wrist camera at the hover is 0.1-0.2 m from the object
    with a hand-eye transform — re-ground the SAME entity through it and move
    the aim by the measured difference. A general mechanism: whatever put the
    error there (homography extrapolation, table flex, a nudged camera), the
    last look before closing is the one that did not travel through it.
    """
    ctx.scratch.pop("wrist_refined_pick", None)  # only a fresh success counts
    if not getattr(ctx.cfg, "wrist_refine", False):
        return grasp
    cam = next((n for n, c in ctx.cfg.cameras.items()
                if c.kind == "wrist" and c.arm == side), None)
    if cam is None or cam not in getattr(ctx.robot, "cameras", []):
        return grasp
    tr = _belief(ctx).track(entity)
    before = None if tr.xyz_base is None else np.asarray(tr.xyz_base, dtype=float)
    if before is None:
        return grasp
    try:
        from .sensing import forget_the_look, ground_entity  # noqa: PLC0415
        forget_the_look(ctx, reason="wrist refine wants a fresh look from the hover")
        found, note = ground_entity(ctx, entity, cameras=[cam], need_look=True)
    except Exception as e:
        ctx.log.event("wrist_refine_error", entity=entity,
                      error=f"{type(e).__name__}: {e}"[:150])
        return grasp
    after = _belief(ctx).track(entity).xyz_base
    if not found or after is None:
        ctx.log.event("wrist_refine_missed", entity=entity, note=str(note)[:150])
        return grasp
    delta = np.asarray(after, dtype=float)[:2] - before[:2]
    d = float(np.linalg.norm(delta))
    if d > WRIST_REFINE_MAX_M:
        # The wrist view found something too far away to be the aimed-at
        # object standing still — likelier a mis-identification at close
        # range. Keep the overhead aim; the arrival/closed-on-air guards
        # below still protect the grasp itself.
        ctx.log.event("wrist_refine_declined", entity=entity,
                      correction_mm=round(d * 1000, 1),
                      note="correction exceeds the same-object bound")
        return grasp
    ctx.log.event("wrist_refine", entity=entity, camera=cam,
                  correction_mm=round(d * 1000, 1),
                  xy=[round(float(v), 4) for v in (before[:2] + delta)])
    # The delta just measured is cam_high's LOCAL bias, not this object's
    # quirk — whatever produced it (planar parallax on a dark cube, regional
    # homography error) offsets every grounding in this neighbourhood the
    # same way. Remember it so `place` can aim at a support through the same
    # correction: picks were centred while stacks landed a constant cube-edge
    # to one side, because only picks got the wrist's verdict.
    ctx.scratch["wrist_bias"] = {"arm": side, "xy": before[:2] + delta,
                                 "delta": delta}
    ctx.scratch["wrist_refined_pick"] = entity
    return grasp + np.array([delta[0], delta[1], 0.0])


@skill(
    name="pick",
    description=(
        "Grasp an entity top-down: hover above its believed location, open, descend, close, lift. "
        "dx/dy nudge the grasp point (meters, world frame) — use for regrasp attempts."
    ),
    params={
        "entity": {"type": "string", "description": "entity id from the program"},
        "arm": {"type": "string", "enum": ["left", "right", "auto"]},
        "strategy": {"type": "string", "enum": ["auto", "no_service", "plain"],
                     "description": "grasp approach; retries downgrade auto->no_service->plain"},
        "dx": {"type": "number"},
        "dy": {"type": "number"},
        "contact": {"type": "boolean",
                    "description": "feel for the surface instead of trusting the perceived height "
                                   "(default true; the reliable mode on depth-free rigs)"},
    },
    required=["entity"],
    post_hint="holding(entity, arm)",
)
def pick(ctx: SkillContext, entity: str, arm: str = "auto", dx: float = 0.0, dy: float = 0.0,
         contact: bool | None = None, at=None, strategy: str = "auto") -> SkillResult:
    """Grasp `entity`. Pass `at` to skip looking for it.

    `at` is a Located (or a bare xyz) from heron.skills.values — a position
    somebody already measured. Without it this falls back to the belief store,
    which is how it has always worked and which is why a long task kept
    re-perceiving: a belief has an age, and a measurement handed over as a value
    does not go stale between one step and the next.
    """
    if contact is None:
        contact = ctx.cfg.contact_descend
    # A look taken BEFORE we ourselves moved this entity is stale by
    # definition: looks write from_sighting=True, our pick/place write False,
    # so a from-action track means every earlier look predates the motion. A
    # relay's second pick closed on the block's ORIGINAL spot because the
    # dataflow handed it the first perceive's coordinates. Moved it -> re-see.
    _tr = ctx.belief.track(entity) if getattr(ctx, "belief", None) is not None else None
    if at is not None and _tr is not None and _tr.xyz_base is not None \
            and not _tr.from_sighting:
        ctx.log.event("supplied_position_stale", entity=entity,
                      note="we moved this entity after that look; re-grounding")
        at = None
    if at is not None:
        target = np.asarray(getattr(at, "xyz", at), dtype=float)
        ctx.log.event("pick_using_given_position", entity=entity,
                      xyz=[round(float(v), 4) for v in target],
                      source=getattr(at, "source", "caller"))
    else:
        target = _resolve_point(ctx, entity,
                                max_age_s=ctx.cfg.budgets.verify_max_age_s * 4)
    # NO LEARNED CORRECTION IS APPLIED HERE. It used to be, keyed by visual
    # description, and it crossed between objects — the blue block inherited
    # the orange block's +20 mm and the gripper closed on air beside a
    # correctly grounded cube. Outcomes are still recorded for analysis; the
    # aim is the aim perception produced.
    side = choose_arm(ctx, target, arm)
    grasp = target + np.array([dx, dy, 0.0])
    floor = tool_floor(ctx)
    # THE FLOOR UNDER A GRASP IS WHATEVER THE OBJECT STANDS ON. Grounding
    # learned this already (_support_top_at, "the table is not the only floor");
    # the descent had not, and that is the whole of the plate-carrying bug. A
    # block sitting in a paper plate grounds at z = 12.7 mm and GRASP_DEPTH puts
    # the fingertips at -5 mm, which is 2 mm above the plate's own surface: the
    # jaws close at the block's BASE, in the plate, instead of around its middle.
    # With contact_descend off on this rig nothing stopped them — they closed on
    # block and plate together, and `place` set the plate down 180 mm away at the
    # spot it had chosen for the block (rig batch 2026-08-06 ep8; the plate
    # leaves the table at t=34 s of clips/b_8.gif).
    from .sensing import _support_top_at  # noqa: PLC0415
    support = _support_top_at(ctx, float(grasp[0]), float(grasp[1]), exclude=entity)
    # A support has to be UNDER the thing. Aliases ("plate" and "white_plate"
    # for one dish) and the vessel's own grasp would otherwise make an object
    # its own floor and leave nothing to bite.
    if support is not None and support > float(grasp[2]) - 0.008:
        support = None
    base_z = support if support is not None else ctx.cfg.table_z
    # Perceived height caps at a small object's plausible top. Depth-based
    # grounding hands back the mask centroid's z, and a mask polluted by arm
    # pixels once read 0.061 — the grasp then closed on air 4 cm up. A block's
    # top is never above its own floor + 5 cm; anything taller is measurement junk.
    top = min(float(grasp[2]), base_z + 0.05)
    if support is not None:
        floor = max(floor, support)
    grasp_z = grasp_height(top, support, floor, ctx.cfg.grasp_z_trim_m)
    if support is not None:
        ctx.log.event("grasp_floor_is_support", entity=entity,
                      support_top=round(float(support), 4), top=round(top, 4),
                      grasp_z=round(grasp_z, 4))

    # Consult the grasp-proposal service first (cfg.grasp_service_url; a no-op
    # when unset). An accepted proposal replaces the believed xy, the
    # perceived-height grasp depth, and adds a wrist yaw — and supersedes the
    # rim heuristic, since the model already chose where on the object to
    # close. The hover/descend motion and the contact probe stay Heron's own.
    orientation = None
    rim = ""
    span = None  # measured only on the heuristic path; the carry bound below
    # Strategy ladder (ASPIRE's interleave-the-approach pattern, ours by
    # explicit arg): a retry that repeats the same strategy re-buys the same
    # failure. "auto" = service + mask-yaw; "no_service" = geometric centre
    # with yaw; "plain" = the old faithful fixed-wrist vertical.
    svc = _service_grasp(ctx, entity, grasp) if strategy == "auto" else None
    if svc is not None:
        sxyz, yaw, score = svc
        grasp = np.array([sxyz[0], sxyz[1], max(float(grasp[2]), float(sxyz[2]))])
        # The service picks where on the object to close, not how far to reach
        # past it — the support bound applies to its depth exactly as to ours.
        grasp_z = grasp_height(float(sxyz[2]), support, floor)
        if abs(yaw) > np.radians(3.0):
            from ..perception.graspclient import yawed_orientation  # noqa: PLC0415
            orientation = yawed_orientation(ctx.cfg.arms[side].approach_rvec, yaw)
        rim = f"; grasp from service (score {score:.2f}, yaw {np.degrees(yaw):.0f} deg)"
    elif (span := _visual_span(ctx, entity)) and span > GRIPPER_MAX_SPAN_M \
            and support is not None:
        # A WIDTH MEASURED THROUGH THE DISH YOU ARE STANDING IN IS THE DISH'S.
        # blue_block came back 138 mm — five times its own size — because the
        # detector boxed the plate it was sitting in, and the rim rule then
        # aimed 45 mm sideways onto the plate's edge. An object cannot be wider
        # than the thing it rests on and still be resting on it.
        ctx.log.event("rim_grasp_declined", entity=entity, span_m=round(span, 4),
                      support_top=round(float(support), 4),
                      reason="the wide box belongs to the support, not the object")
        span = None
    elif span and span > GRIPPER_MAX_SPAN_M:
        # Sideways, but NOT upwards. This used to raise the grasp to
        # target_z + span/4, on the reasoning that a rim is above a centroid.
        # Measured against ground truth on the bowl every spatial task starts
        # with: that put the fingers 26 mm ABOVE the bowl's top edge, where they
        # closed on air every time. The perceived point is already at the top of
        # an open vessel seen from above, so the ordinary descent is right and
        # the correction was the whole failure.
        #
        # The offset is capped because it was span/2 and the span itself comes
        # from a mask that over-measures — 116 mm read on a 96 mm bowl, which
        # aimed 58 mm out when 20-48 mm all lift it and further out does not.
        offset = min(span / 2.0, RIM_OFFSET_MAX_M)
        grasp = grasp + np.array([0.0, offset, 0.0])
        rim = (f"; too wide to grasp centrally ({span * 1000:.0f} mm > "
               f"{GRIPPER_MAX_SPAN_M * 1000:.0f} mm), taking the rim "
               f"{offset * 1000:.0f} mm out")
        ctx.log.event("rim_grasp", entity=entity, span_m=round(span, 4),
                      offset_m=round(offset, 4),
                      grasp=[round(float(v), 4) for v in grasp[:2]], grasp_z=round(grasp_z, 4))
    if svc is None and orientation is None and strategy != "plain":
        myaw = _mask_yaw(ctx, entity, grasp)
        if myaw is not None and abs(myaw) > np.radians(8.0):
            from ..perception.graspclient import yawed_orientation  # noqa: PLC0415
            orientation = yawed_orientation(ctx.cfg.arms[side].approach_rvec, myaw)
            rim += f"; wrist yawed {np.degrees(myaw):.0f} deg across the narrow side"
            ctx.log.event("mask_yaw_grasp", entity=entity,
                          yaw_deg=round(float(np.degrees(myaw)), 1))
    # Per-arm aim trim: the left arm's fingertips land a measured
    # [-13.3, -3.0] mm from the commanded point (fingertip-camera closed
    # loop, 2026-08-18) — a kinematic offset the base calibration cannot
    # absorb. The config carries the counter-offset per arm. An arm using
    # pinch_refine SKIPS the trim: the loop drives the fingertips onto the
    # raw aim, and pre-trimming made it chase a point 13 mm past the cube
    # (watched: the two compensations double-counted into a clean miss).
    _trim = (None if side in list(getattr(ctx.cfg, "pinch_refine_arms", None)
                                  or [])
             else (getattr(ctx.cfg, "aim_trim_xy_m", None) or {}).get(side))
    if _trim:
        grasp = grasp + np.array([float(_trim[0]), float(_trim[1]), 0.0])
        ctx.log.event("aim_trim_applied", arm=side, skill="pick",
                      trim_mm=[round(float(_trim[0]) * 1000, 1),
                               round(float(_trim[1]) * 1000, 1)])
    grasp = _radial_sag_comp(ctx, side, grasp)
    hover = approach_pose(ctx, grasp[:2], grasp[2], side)

    # ALREADY HOLDING IT IS SUCCESS, NOT A REASON TO OPEN THE JAWS. A repair
    # loop re-picked an entity the gripper was already carrying; the first act
    # of a pick is to open, and the cargo dropped where the arm stood.
    _held = _belief(ctx).track(entity).held_by
    if _held:
        return SkillResult(ok=True, info=(
            f"pick({entity}): the {_held} arm already holds it — not opening "
            f"the jaws on cargo"))

    ph = _Phases(ctx, "pick", entity)
    ctx.robot.set_gripper(side, OPEN_WIDTH)
    ph.mark("open_gripper")
    travel(ctx, side, hover)
    ph.mark("travel_to_hover")
    refined = _wrist_refine(ctx, side, entity, grasp)
    if refined is not grasp:
        grasp = refined
        ph.mark("wrist_refine")
    refined = _pinch_refine(ctx, side, grasp, orientation)
    if refined is not grasp:
        grasp = refined
        ph.mark("pinch_refine")
    touched = False
    if contact:
        z_stop, touched = descend_to_contact(ctx, side, float(grasp[0]), float(grasp[1]),
                                             float(hover[2]), floor,
                                             z_expected=grasp_z,
                                             orientation=orientation)
        if touched:
            grasp_z = max(z_stop + CONTACT_BACKOFF_M, floor)
            _move(ctx, side, np.array([grasp[0], grasp[1], grasp_z]), 0.6, orientation)
        else:  # felt nothing: fall back to the perceived height
            _move(ctx, side, np.array([grasp[0], grasp[1], grasp_z]), 1.0, orientation)
    else:
        _move(ctx, side, np.array([grasp[0], grasp[1], grasp_z]), 1.5, orientation)
    ph.mark("descend")
    # DID THE ARM ARRIVE? At the far edge of its reach the controller settles
    # short — 30 mm at the column the two arms share — and the jaws then close
    # beside the object. Afterwards this is nearly undiagnosable: the gripper
    # reports a width, the belief records a grasp, and the miss surfaces two
    # skills later as an object that "moved by itself". The tool's own position
    # says it now, before anything is closed on, and re-commanding the same
    # target from a standstill usually gets there.
    arrived = _grasp_arrival(ctx, side, grasp[:2], grasp_z, span, orientation)
    if arrived is not None:
        _belief(ctx).invalidate_entity(entity, reason="the arm could not reach the grasp point")
        return SkillResult(ok=False, error=(
            f"pick({entity}): the {side} arm stopped {arrived * 1000:.0f} mm from "
            f"xy={np.round(grasp[:2], 3).tolist()} and did not close the gap when sent "
            f"again — that point is at the edge of its reach, not merely awkward. "
            f"Closing here would grasp the table beside the object."))
    ctx.robot.set_gripper(side, 0.0)
    ph.mark("close")
    _move(ctx, side, hover, 1.5, orientation)
    ph.mark("lift")
    ph.done()

    g = ctx.robot.get_gripper(side)
    if g["width_m"] < CLOSED_ON_AIR_WIDTH:
        # The jaws met. CLOSED_ON_AIR_WIDTH has documented this rule since the
        # constant was written, and nothing enforced it: a grasp that closed to
        # 0.4 mm reported ok=True three times at the same wrong spot while the
        # repair loop hunted phantoms downstream. A miss the gripper itself can
        # feel must fail HERE, where the diagnosis is one sentence.
        _belief(ctx).invalidate_entity(entity, reason="grasp closed on air")
        return SkillResult(ok=False, error=(
            f"pick({entity}): closed on air at xy={np.round(grasp[:2], 3).tolist()} "
            f"(width {g['width_m'] * 1000:.1f} mm) — the object is not where it was "
            f"believed to be; re-perceive before retrying"))
    from ..verify import holding_something  # noqa: PLC0415  (avoids a cycle at import)

    grasped = holding_something(g)
    if grasped:
        tool = ctx.robot.get_cartesian(side)
        # Where the object actually ended up relative to the tool. The gripper
        # closes on whatever part of the object it reached, so the object rides
        # off-centre — repeatably. `place` commands the tool, so without this the
        # object is released systematically to one side of the target.
        #
        # But this subtraction only measures the object's offset if the tool went
        # where it was sent. When it did not, the same difference reads as a
        # carry, and it is then subtracted at the release — where the arm makes
        # the SAME error again, so the two add instead of cancelling. Measured in
        # the twin against MuJoCo ground truth, over ten placements: the object
        # sat 0-7 mm from the tool centre (it is dragged there by the closing
        # jaws), while this read 28-42 mm, and placements landed a median 63 mm
        # from the aim against 31 mm with no compensation at all.
        #
        # An object clamped between the jaws cannot have its centre further out
        # than half its own width. That bound separates the two cases with a fact
        # about the gripper rather than a guess: a 32 mm block cannot ride 34 mm
        # off-centre, a wide vessel taken by the rim genuinely can.
        prior = _belief(ctx).track(entity).xyz_base
        carry = None
        if prior is not None:
            carry = tuple(float(prior[i]) - float(tool[i]) for i in range(3))
        # A WRIST-REFINED GRASP CARRIES CENTRED, AND THE FK CANNOT VOTE. This
        # subtraction reads (perceived - controller FK at the hover), and on
        # this rig the FK at long reach reports 15-20 mm short of where the
        # tool physically stands (the link-flex lie). With the wrist camera
        # having centred the aim and the closing jaws dragging the cube to
        # centre, the honest carry is ~0 — the 13-17 mm this read anyway was
        # the FK's lie, and place then subtracted it from every release: felt
        # contact came 12 mm early and the cargo tumbled 108 mm (2026-08-19
        # 14:08 episode). Keep the reading in the journal; do not apply it.
        if carry is not None and ctx.scratch.get("wrist_refined_pick") == entity:
            ctx.log.event("carry_offset_fk_distrusted", entity=entity,
                          fk_residual_mm=[round(c * 1000, 1) for c in carry[:2]])
            carry = None
        _belief(ctx).update_track(entity, xyz_base=tuple(tool), confidence=0.9, held_by=side)
        forget_placement(ctx, entity)   # it left its slot when it left the support
        if carry is not None:
            width = max(float(span or 0.0), float(g["width_m"] or 0.0))
            limit = 0.5 * width + CARRY_MEASUREMENT_SLACK_M
            reach = max(abs(c) for c in carry[:2])
            if reach <= limit:
                _belief(ctx).track(entity).carry_offset = carry
                ctx.log.event("carry_offset", entity=entity,
                              offset_mm=[round(c * 1000, 1) for c in carry])
            else:
                # Not discarded silently: this is the arm's tracking error, and a
                # journal that shows it is the only way anyone finds that out.
                ctx.log.event("carry_offset_rejected", entity=entity,
                              offset_mm=[round(c * 1000, 1) for c in carry],
                              limit_mm=round(limit * 1000, 1),
                              object_width_mm=round(width * 1000, 1),
                              reason="further from the tool centre than the object "
                                     "is wide — the arm did not reach the grasp "
                                     "point, which is not something place can undo")
    return SkillResult(
        ok=True,
        info=f"pick({entity}){rim} with {side} arm at xy={np.round(grasp[:2], 3).tolist()} z={grasp_z:.3f} "
        f"({'felt the surface' if touched else 'used perceived height'}); "
        f"gripper width={g['width_m']:.3f} effort={g['effort']:.2f}",
        proprio={"width_m": g["width_m"], "effort": g["effort"], "arm_is_left": float(side == "left"),
                 "contact_found": float(touched)},
    )


def _grasp_arrival(ctx: SkillContext, side: str, xy, z: float,
                   span: Optional[float], orientation) -> Optional[float]:
    """None if the tool is over the grasp point; else how far short it stopped.

    The tolerance is the object's own half-width: jaws that still overlap the
    object will drag it to centre as they close, and jaws that do not will
    close on the table. An unknown span falls back to a small object's, which
    is the conservative direction — it fails a grasp that might have worked
    rather than reporting one that did not.
    """
    reader = getattr(ctx.robot, "get_cartesian", None)
    if not callable(reader):
        return None
    target = np.asarray(xy, dtype=float)
    allow = 0.5 * float(span or GRASP_ARRIVAL_DEFAULT_SPAN_M) + GRASP_ARRIVAL_SLACK_M
    off = 0.0
    for attempt in range(GRASP_ARRIVAL_TRIES):
        off = float(np.linalg.norm(np.asarray(reader(side), dtype=float)[:2] - target))
        if off <= allow:
            if attempt:
                ctx.log.event("grasp_arrival_recovered", arm=side, tries=attempt + 1,
                              off_mm=round(off * 1000, 1))
            return None
        ctx.log.event("grasp_arrival_short", arm=side, attempt=attempt + 1,
                      off_mm=round(off * 1000, 1), allowed_mm=round(allow * 1000, 1),
                      xy=[round(float(v), 3) for v in target])
        if attempt + 1 < GRASP_ARRIVAL_TRIES:
            _move(ctx, side, np.array([target[0], target[1], float(z)]), 1.0, orientation)
    return off


def _random_clear_spot(ctx: SkillContext, side: str, held: str,
                       near=None) -> np.ndarray:
    """A random reachable table point outside every known object's footprint.

    Keep-out per tracked object is its own measured half-extent plus the held
    object's plus 3 cm of air — physical clearance from measured spans, not a
    tuned magic radius. Margins shrink once if the table is too crowded to
    satisfy them; a spot is still returned (the alternative is refusing to
    put the object anywhere, which strands the self-reset loop).
    """
    gb = getattr(ctx, "grounding_bounds", None)
    wx, wy = gb if gb else (ctx.cfg.safety.workspace_x, ctx.cfg.safety.workspace_y)
    tr_held = ctx.belief.track(held)
    held_half = (getattr(tr_held, "span_m", None) or 0.04) / 2
    keep_out = []
    for eid in ctx.belief.entities:
        if eid == held:
            continue
        tr = ctx.belief.track(eid)
        if tr.xyz_base is None:
            continue
        half = (getattr(tr, "span_m", None) or 0.08) / 2
        # The FOOTPRINT is physical and the MARGIN is comfort; only comfort
        # may relax. Relaxing both put a "beside the plate" release 57 mm from
        # the centre of a 62 mm plate — on its rim — because a crowded table
        # scaled the plate itself down by 0.6.
        keep_out.append((float(tr.xyz_base[0]), float(tr.xyz_base[1]),
                         half, held_half + 0.03))
    rng = np.random.default_rng()
    edge = 0.03

    def _ok(x, y, relax):
        # Reachable for the PLACE and for the RE-PICK: a relay dropped a
        # block on a spot the arm could hover over but whose staged pick
        # approach had no solution, and the block was stranded at the edge.
        #
        # The re-pick needs A descent path, not a particular one. This asked
        # for table_z + 0.13, and at the far edge of this arm's reach nothing
        # above table_z + 0.05 has a solution at all — so every point in the
        # strip the two arms share was called unusable, and a destination
        # placed there was snapped 88 mm inward, out of the other arm's reach.
        # 50 mm over the table is 35 mm over a cube's top: room to descend,
        # which is the thing the check was ever about.
        return (not any(np.hypot(x - kx, y - ky) < hard + soft * relax
                        for kx, ky, hard, soft in keep_out)
                and ctx.robot.reachable(side, (x, y, ctx.cfg.table_z + 0.01))
                and ctx.robot.reachable(side, (x, y, ctx.cfg.table_z + PLACE_APPROACH_M)))

    for relax in (1.0, 0.6):
        # A requested point is an INTENT, not a licence to collide: use it
        # exactly when it is clear, else the nearest clear spot to it.
        if near is not None and _ok(float(near[0]), float(near[1]), relax):
            return np.array([float(near[0]), float(near[1]), ctx.cfg.table_z])
        best = None
        for _ in range(60):
            x = float(rng.uniform(wx[0] + edge, wx[1] - edge))
            y = float(rng.uniform(wy[0] + edge, wy[1] - edge))
            if not _ok(x, y, relax):
                continue
            if near is None:
                return np.array([x, y, ctx.cfg.table_z])
            d = float(np.hypot(x - near[0], y - near[1]))
            if best is None or d < best[0]:
                best = (d, x, y)
        if best is not None:
            return np.array([best[1], best[2], ctx.cfg.table_z])
    if near is not None:
        return np.array([float(near[0]), float(near[1]), ctx.cfg.table_z])
    return np.array([(wx[0] + wx[1]) / 2, (wy[0] + wy[1]) / 2, ctx.cfg.table_z])



@skill(
    name="place",
    description=(
        "Release the held entity above a target: either target entity id (place onto/into it) "
        "or absolute world x/y. With NEITHER, the robot samples a random clear spot on the "
        "reachable table that avoids every known object's footprint — ALWAYS prefer that for "
        "'remove/park/put aside' intents; hand-written x/y coordinates skip the collision "
        "check and may land on another object. Only pass x/y when the instruction itself "
        "names a location. dx/dy offset from the target (meters)."
    ),
    params={
        "entity": {"type": "string", "description": "the entity currently held"},
        "target": {"type": "string", "description": "destination entity id; omit when using x/y"},
        "x": {"type": "number"},
        "y": {"type": "number"},
        "dx": {"type": "number"},
        "dy": {"type": "number"},
        "arm": {"type": "string", "enum": ["left", "right", "auto"]},
        "contact": {"type": "boolean",
                    "description": "lower until the object touches down before releasing "
                                   "(default true; the reliable mode on depth-free rigs)"},
    },
    required=["entity"],
    post_hint="in(entity, target) or on(entity, target); not holding(entity, arm)",
)
def place(
    ctx: SkillContext,
    entity: str,
    target: Optional[str] = None,
    x: Optional[float] = None,
    y: Optional[float] = None,
    dx: float = 0.0,
    dy: float = 0.0,
    arm: str = "auto",
    contact: bool | None = None,
    precise: bool = False,
    at=None,
) -> SkillResult:
    """Release the held object at `target`, at (x, y), or at `at`.

    `at` is a Located from heron.skills.values: a destination somebody already
    measured. Handed one, this makes no detector call at all — which is the
    difference between a four-placement task costing four looks and costing one.
    """
    if contact is None:
        # Depth quality is a property of the SURFACE. The block reads cleanly;
        # the glossy plate scatters 25.5 mm inside its own mask, with points
        # 84 mm below the table. Releasing onto that by depth alone drops the
        # object from the wrong height, so place may feel for the surface even
        # where pick does not.
        contact = ctx.cfg.contact_descend if ctx.cfg.contact_descend_place is None \
            else ctx.cfg.contact_descend_place
    tr = _belief(ctx).track(entity)
    side = tr.held_by or (arm if arm in ctx.robot.arms else None)
    if side is None and len(ctx.robot.arms) == 1:
        side = ctx.robot.arms[0]     # single-arm rig: there is nothing to choose
    if side is None:
        # Naming the real problem matters. This used to say "pick it first or
        # pass arm=", and the orchestrator dutifully spent ten edits inventing
        # arm names that do not exist on this robot, never once retrying the
        # pick that had silently failed.
        raise ValueError(
            f"nothing is held: the grasp of {entity!r} did not take hold, so there is "
            f"nothing to place. Re-run pick for {entity!r}. (Arms on this robot: "
            f"{list(ctx.robot.arms)}.)")
    # THE GOAL NAMES WHAT TO CLEAR. A bare place — and a planner-invented
    # x/y, which is a wish exactly like "anywhere clear" — is graded by the
    # program's own negated predicates: "not on(X, plate)". Twice on the rig
    # the release landed on the plate's rim at the same invented (0.465, 0.0)
    # because the plate had never been individually grounded, its track had no
    # position, and the keep-out was therefore empty. Being judged by a
    # predicate earns the support a position BEFORE any spot is chosen.
    if at is None and target is None:
        for gname, gargs, gneg in getattr(ctx, "goal_predicates", None) or []:
            if gneg and gname in ("on", "in") and gargs and gargs[0] == entity \
                    and len(gargs) > 1 \
                    and _belief(ctx).track(gargs[1]).xyz_base is None:
                from .sensing import ground_entity as _ge  # noqa: PLC0415
                _ge(ctx, gargs[1], need_look=True)
    if at is not None:
        dest = np.asarray(getattr(at, "xyz", at), dtype=float)
        ctx.log.event("place_using_given_position", entity=entity,
                      xyz=[round(float(v), 4) for v in dest],
                      source=getattr(at, "source", "caller"))
    elif target is not None:
        dest = _resolve_point(ctx, target, max_age_s=ctx.cfg.budgets.verify_max_age_s * 4)
        # A NEAR goal tolerates the target's footprint plus ~6 cm — and needs
        # to, because regions ground to points the arm cannot always occupy:
        # "the lower-left corner" landed at (0.145, 0.170), the literal table
        # corner against the rail, and the place died in IK three times. When
        # the plan's own goal for this pair is near(), the destination is an
        # intent: snap it to the nearest reachable clear spot toward it. An
        # on/in goal keeps the exact point — snapping onto the table beside a
        # plate would fake the goal.
        if any(n == "near" and len(a) >= 2 and a[0] == entity and a[1] == target
               and not neg
               for n, a, neg in getattr(ctx, "goal_predicates", None) or []):
            snapped = _random_clear_spot(ctx, side, entity, near=dest[:2])
            if abs(snapped[0] - dest[0]) > 1e-9 or abs(snapped[1] - dest[1]) > 1e-9:
                ctx.log.event("place_snapped_clear", entity=entity,
                              asked=[round(float(dest[0]), 3), round(float(dest[1]), 3)],
                              xyz=[round(float(v), 4) for v in snapped])
            dest = snapped
    elif x is not None and y is not None:
        # Bare-table destination: height comes from the TABLE, not from the
        # workspace box. On the rig the box floor is +0.01 while the table sits
        # at -0.015; floor+0.02 put the release 6 cm in the air and the block
        # bounced wherever it liked (watched live, 2026-08-03).
        dest = _random_clear_spot(ctx, side, entity, near=(x, y))
        if abs(dest[0] - x) > 1e-9 or abs(dest[1] - y) > 1e-9:
            ctx.log.event("place_snapped_clear", entity=entity,
                          asked=[round(float(x), 3), round(float(y), 3)],
                          xyz=[round(float(v), 4) for v in dest])
    else:
        # No destination at all: "remove X from the plate" means ANY clear
        # spot. Sample the reachable table, keeping out of every known
        # entity's footprint (not just the source support — general): a spot
        # is clear when it is farther from each tracked object than that
        # object's own measured extent plus the held object's, with margin.
        dest = _random_clear_spot(ctx, side, entity)
        ctx.log.event("place_random_clear_spot", entity=entity,
                      xyz=[round(float(v), 4) for v in dest])
    if target is not None:
        _seed_perceived_occupants(ctx, dest, entity, target)
    dest, slot = _spread_within_support(ctx, side, dest, entity, target, at)
    # Carry forward what earlier episodes learned about aiming at THIS container.
    # A placement miss is usually systematic, so the correction belongs in the
    # aim rather than in a retry that starts from the same wrong point.
    #
    # Caller dx/dy rides through the SAME support-bounds guard as the memory
    # hint: the planner once invented dx=-0.012 toward a 40 mm block, the guard
    # dutifully refused the memory hint — and the raw arg was applied anyway,
    # aiming 12 mm off a target that tolerates 8. One channel, one guard.
    hint = (float(dx), float(dy))
    if target is not None:
        stored = (0.0, 0.0)
        if ctx.memory is not None:
            stored = ctx.memory.place_offset(_belief(ctx).track(target).description)
        hint = _usable_place_hint(ctx, target, (hint[0] + stored[0], hint[1] + stored[1]))
    dest = dest + np.array([hint[0], hint[1], 0.0])
    # Remember what we aimed at: comparing it with where the object actually
    # settles is the only way to measure the residual.
    aim = dest
    ctx.last_place = {"entity": entity, "target": target, "aim": tuple(float(v) for v in aim),
                      "hint": hint}
    # The object does not sit at the tool centre. `pick` measured where it
    # actually rode in the gripper; placing the TOOL at the target therefore
    # puts the OBJECT somewhere else, repeatably. From here on `dest` is where
    # the TOOL goes and `aim` is where the OBJECT is meant to end up — they are
    # not the same point, and the belief store has to be told the second one.
    carry = _belief(ctx).track(entity).carry_offset
    if carry:
        dest = aim - np.array([float(carry[0]), float(carry[1]), 0.0])
    # The aim and the tool's destination diverge by exactly the carry, and the
    # journal is where that split has to be visible: a landing can only be
    # charged to the right party (aim, carry, or motion) if both points are on
    # record, and the info string below prints only the tool's.
    ctx.log.event("place_aim", entity=entity, target=target,
                  aim=[round(float(v), 4) for v in aim],
                  tool_dest=[round(float(v), 4) for v in dest],
                  carry_mm=[round(float(c) * 1000, 1) for c in carry[:2]] if carry else None)
    _trim = (getattr(ctx.cfg, "aim_trim_xy_m", None) or {}).get(side)
    if _trim:
        dest = np.array([float(dest[0]) + float(_trim[0]),
                         float(dest[1]) + float(_trim[1]), float(dest[2])])
        ctx.log.event("aim_trim_applied", arm=side, skill="place",
                      trim_mm=[round(float(_trim[0]) * 1000, 1),
                               round(float(_trim[1]) * 1000, 1)])
    dest = _radial_sag_comp(ctx, side, dest)
    # AIM THE SUPPORT THROUGH THE SAME CORRECTION THE PICK GOT. The pick's
    # wrist look measures cam_high's LOCAL bias (planar parallax on a dark
    # cube, regional error — whatever put it there); the support sits a few
    # centimetres away inside the same bias field, but its grounding never
    # met the wrist camera — which is why picks centred while stacks landed
    # a constant cube-edge to one side (2026-08-19). Apply the measured
    # delta, no new observation, no extra motion. Bookkeeping stays
    # consistent: `aim` remains the believed (biased-frame) position, and
    # the post-place measurement reads the landing through the same biased
    # camera, so the comparison cancels the bias the same way.
    if target is not None and getattr(ctx.cfg, "wrist_refine", False):
        # THE SUPPORT GETS THE SAME VERDICT THE PICK GOT: a straight-down
        # wrist look, no tilt (Yifan: the wrist decides, same as pick). With
        # cargo in the fingers a look from directly above photographs the
        # cargo, so step BACK toward the base first — orientation stays
        # vertical, the support just enters the frame beside the cargo's
        # shadow. If the look misses (occlusion, same-colour cargo), fall
        # back to the bias vector the pick measured minutes ago.
        looked = False
        try:
            t_wb_ = np.asarray(ctx.cfg.arms[side].t_world_base, float)
            back_ = t_wb_[:2, 3] - np.asarray(dest[:2], float)
            back_ = back_ / max(float(np.linalg.norm(back_)), 1e-9)
            view_xy = (float(dest[0]) + back_[0] * WRIST_LOOK_STEPBACK_M,
                       float(dest[1]) + back_[1] * WRIST_LOOK_STEPBACK_M)
            _move(ctx, side, approach_pose(ctx, view_xy, 0.02, side), 1.6)
            refined = _wrist_refine(ctx, side, target, np.asarray(dest, float))
            if refined is not None and float(np.linalg.norm(
                    np.asarray(refined[:2]) - np.asarray(dest[:2], float))) > 1e-6:
                dest = np.array([float(refined[0]), float(refined[1]),
                                 float(dest[2])])
                looked = True
                ctx.log.event("place_support_wrist_look", target=target,
                              xy=[round(float(v), 4) for v in dest[:2]])
        except Exception as e:
            ctx.log.event("place_support_wrist_look_failed", target=target,
                          error=f"{type(e).__name__}: {e}"[:100])
        _wb = ctx.scratch.get("wrist_bias")
        if not looked and _wb is not None:
            _ttr = _belief(ctx).track(target)
            near = float(np.linalg.norm(
                np.asarray(dest[:2], float) - np.asarray(_wb["xy"], float)))
            if (_ttr.camera and "wrist" not in str(_ttr.camera)
                    and near < WRIST_BIAS_REACH_M):
                dest = np.array([float(dest[0]) + float(_wb["delta"][0]),
                                 float(dest[1]) + float(_wb["delta"][1]),
                                 float(dest[2])])
                ctx.log.event("place_aim_wrist_bias", target=target,
                              delta_mm=[round(float(v) * 1000, 1)
                                        for v in _wb["delta"]],
                              measured_at_mm=round(near * 1000, 1),
                              xy=[round(float(v), 4) for v in dest[:2]])
    hover = approach_pose(ctx, dest[:2], dest[2], side)
    # Mirror pick: a floor taken from the raw workspace box is a metre below a
    # tabletop scene, and a contact descent would drive the tool through it.
    floor = tool_floor(ctx)
    # How far above the target to let go depends on what is underneath. A
    # container has walls that funnel a falling object inward; a flat support has
    # none, and a 4 cm drop both lets the object roll and shoves the support
    # itself — measured at 35 mm of plate movement, larger than the entire
    # tolerance the placement is judged against.
    clearance = DROP_CLEARANCE
    if target is None or _is_flat_support(ctx, target):
        # A bare table is the flattest support there is. The constant is the
        # default; a SKILL-level patch can re-tune it for one support class
        # (level="skill", target="place", delta={"flat_drop_clearance": ...}),
        # which is the second rung of the repair-routing ladder — the first
        # being predicate thresholds in verify.py.
        clearance = FLAT_DROP_CLEARANCE
        store = getattr(ctx, "patches", None)
        if store is not None and target is not None:
            from ..patches import PatchLevel  # noqa: PLC0415
            clearance = store.param(
                PatchLevel.SKILL, "place", "flat_drop_clearance", clearance,
                {"skill": "place", "entity_id": target,
                 "entity_description": _belief(ctx).track(target).description})
    drop = np.array([dest[0], dest[1], max(float(dest[2]) + clearance, floor)])

    ph = _Phases(ctx, "place", entity)
    travel(ctx, side, hover)
    ph.mark("travel_to_hover")
    # One measurement, then one motion. Feeling the way down cost 25 blocking
    # round-trips from the hover — the arm visibly stuttering — and it existed
    # only because nothing could tell us how high the surface was. The wrist
    # camera can: at the hover it is 10-20 cm from the target, inside the D405's
    # 7-50 cm specification, where the overhead camera at 1.04 m is not.
    # Stacking blind spot: placing onto a small solid object, the cargo hangs
    # exactly between the wrist camera and the support's top face, so the only
    # surface the camera can see near the destination is the TABLE — measured
    # live: surface_z=-0.015 from 35k points while a 30 mm block sat at +0.030
    # directly under the tool (twin stacking, 2026-08-03). The release then
    # happens at table height and ploughs the support aside. A small solid
    # target therefore skips the camera and feels for its top by contact.
    stack_half = 0.0
    if target is not None and target in getattr(_belief(ctx), "entities", ()):
        from ..verify import _half_extent  # noqa: PLC0415  (verify imports this package)
        stack_half = _half_extent(ctx, target)
    stacking = 0.0 < stack_half <= STACK_TARGET_HALF_EXTENT_M and not _is_flat_support(ctx, target)
    # Stacking is the case that cannot survive the scatter, so it asks for the
    # correction without being told: a 28 mm cube tolerates about 7 mm before
    # its centre of mass leaves the one below it, and the measured placement is
    # 11 mm off.
    precise = precise or stacking
    # (Removed 2026-08-19: the tilted step-back wrist look at the support.
    # Measured corrections were 4-8 mm — inside the measurement stack's own
    # noise — for ~8 s of choreography per stack, and the oblique viewpoint
    # magnifies pixel error. Yifan called it twice: the felt-descent finds
    # the support top anyway, and the cargo's own post-place measurement
    # judges the landing.)
    if stacking:
        surface, why = None, "support top occluded by the cargo — feeling for it"
        ctx.log.event("place_on_solid_by_contact", target=target,
                      half_extent_m=round(stack_half, 4))
    else:
        surface, why = surface_height_below(ctx, side, dest[:2])
    ph.mark("surface_probe")
    if surface is not None:
        drop = np.array([dest[0], dest[1], max(surface + PLACE_RELEASE_GAP_M, floor)])
        ctx.log.event("place_from_wrist_depth", surface_z=round(surface, 4),
                      release_z=round(float(drop[2]), 4), note=why)
        ctx.robot.move_cartesian(side, drop, seconds=1.5)
    elif stacking and getattr(getattr(ctx.robot, "_robot", None), "felt_descend", None) is not None:
        # FEEL, BUT GENTLY. Geometry alone cannot place the release within three
        # millimetres — the support's grounded height carries about 9 mm of
        # error, which is the whole tolerance — and the ordinary probe presses
        # hard enough to move a free-standing cube. So the rig still feels for
        # the top, at a quarter of the speed and 40% of the force, and releases
        # just above wherever it found it.
        z_stop, touched = descend_to_contact(
            ctx, side, float(dest[0]), float(dest[1]), float(hover[2]), floor,
            z_expected=float(dest[2]) + 2.0 * stack_half,
            speed_mps=STACK_PROBE_SPEED_MPS, margin_n=STACK_CONTACT_MARGIN_N)
        if not touched:
            ctx.log.event("stack_support_gone", target=target,
                          descended_to=round(float(z_stop), 4))
            # Without this, "re-perceive" is a lie: the perceive skill happily
            # reuses a seconds-old sighting, which is exactly the belief that
            # just sent the probe into empty air — watched loop three times on
            # a bowl mis-matched as the (occluded) support cube, 2026-08-19.
            _belief(ctx).invalidate_entity(
                target, reason="felt nothing where it was believed to stand")
            return SkillResult(ok=False, error=(
                f"place({entity} on {target}): descended to the table without "
                f"touching anything — {target} is not under the aim any more. "
                "Re-perceive it before trying again."))
        drop = np.array([dest[0], dest[1],
                         max(z_stop + STACK_RELEASE_GAP_M, floor)])
        ctx.log.event("stack_felt_release", target=target,
                      felt_z=round(float(z_stop), 4),
                      release_z=round(float(drop[2]), 4),
                      geometric_would_be=round(float(dest[2]) + 2.0 * stack_half, 4))
        ctx.robot.move_cartesian(side, drop, seconds=1.0)
    elif stacking:
        # STACKING IS GEOMETRY IN THE TWIN, where soft contacts never form an
        # effort ridge.
        # The reason was written here for MuJoCo's soft contacts and it turns
        # out to be a fact about the world: a support you can push is a support
        # you must not press. Measured on the rig, green cube onto orange:
        # descent one felt the top at 16.5 mm, placed 1.8 mm from its aim, and
        # shoved the support out from under it; descents two and three felt
        # nothing at all because there was nothing left to feel. The force that
        # trips a contact detector is more than a 30 g cube needs to slide.
        #
        # So the height comes from what we already know — the support's own
        # position plus its own height — and the cargo is released a few
        # millimetres above it. The correction loop after the release is what
        # absorbs the error in that estimate; it does not need a touch.
        top = float(dest[2]) + 2.0 * stack_half
        # +12 mm of slack in sim, where the P-controller stops ~9.5 mm short of
        # a commanded height (measured: cmd 0.0607, actual 0.0512) and the cargo
        # ended up IN CONTACT at "release" — jaws opening under load shot the
        # block 30 mm sideways. The rig tracks its commands, so it needs only
        # the clearance itself.
        sim = getattr(getattr(ctx.robot, "_robot", None), "felt_descend", None) is None
        slack = 0.010 if sim else 0.0
        drop = np.array([dest[0], dest[1],
                         max(top + STACK_RELEASE_GAP_M + slack, floor)])
        ctx.robot.move_cartesian(side, drop, seconds=1.2)
        actual = None
        try:
            tf = getattr(ctx.robot, "get_cartesian", None)
            if callable(tf):
                actual = [round(float(v), 4) for v in list(tf(side))[:3]]
        except Exception:
            pass
        ctx.log.event("stack_geometric_release", top_z=round(top, 4),
                      release_z=round(float(drop[2]), 4),
                      commanded=[round(float(v), 4) for v in drop],
                      actual_tcp=actual)
    elif contact or stacking:
        # No usable wrist depth: feel for the surface rather than release from a
        # height nobody measured.
        ctx.log.event("wrist_depth_unavailable", reason=why)
        z_stop, touched = descend_to_contact(ctx, side, float(dest[0]), float(dest[1]),
                                             float(hover[2]), floor,
                                             z_expected=float(dest[2]) + 2.0 * stack_half)
        drop = np.array([dest[0], dest[1], max(z_stop + CONTACT_BACKOFF_M, floor)])
        # FEELING NOTHING WHERE A SUPPORT SHOULD BE IS AN ANSWER, NOT A FLOOR.
        # Stacking the green cube on the orange one: the first descent felt the
        # top at 16.5 mm and the placement landed 1.8 mm from its aim — and
        # nudged the support out from under it. The next two descents felt
        # nothing at all, went to the table, and set the cargo down there, which
        # verified as "the orange cube has nothing on it" and read as a
        # placement error when it was a missing support.
        if stacking and not touched:
            ctx.log.event("stack_support_gone", target=target,
                          expected_top=round(float(dest[2]) + 2.0 * stack_half, 4),
                          descended_to=round(float(z_stop), 4))
            _belief(ctx).invalidate_entity(
                target, reason="felt nothing where it was believed to stand")
            return SkillResult(ok=False, error=(
                f"place({entity} on {target}): descended to the table without "
                f"touching anything — {target} is not under the aim any more. "
                "Re-perceive it before trying again; releasing here would put "
                f"{entity} on the table and call it a stack."))
        if not touched:
            ctx.robot.move_cartesian(side, drop, seconds=1.0)
    else:
        ctx.robot.move_cartesian(side, drop, seconds=1.5)
    # Measure the object before letting it go, and open only far enough for it
    # to fall out. At the full aperture each jaw sweeps 40 mm sideways at table
    # height, which is how a placement 9 mm from the tray's centre ended up
    # shoving its neighbour 38 mm.
    ctx.robot.set_gripper(side, release_width(ctx.robot.get_gripper(side).get("width_m")))
    ctx.robot.move_cartesian(side, hover, seconds=1.5)
    # A released object is still falling. Judging containment now reads the
    # world mid-flight and scores a placement that is about to succeed as a
    # failure — measured on LIBERO, twenty more physics steps were the entire
    # difference between 0/2 and 2/2 on this task.
    settle = getattr(ctx.robot, "settle", None)
    if callable(settle):
        settle(SETTLE_AFTER_RELEASE_S)

    # Where the OBJECT was sent, not where the tool was. This is the position
    # everything downstream defends the entity's identity with — the verifier
    # looks for it here, and re-grounding prefers the instance nearest it — so
    # recording the tool's destination handed the block's identity to whichever
    # lookalike happened to be closer to the gripper.
    _belief(ctx).update_track(
        entity, xyz_base=(float(aim[0]), float(aim[1]), float(drop[2])), confidence=0.6,
        held_by=None, note="placed; height approximate until re-perceived",
    )
    # AND THEN GET OUT OF THE WAY. The arm used to stop at the hover directly
    # over what it had just put down, which is where the next look has to
    # happen from — 52 seconds of it on the rig 2026-08-06, hanging over the
    # plate while the verification asked the detector fourteen times about a
    # block it had watched being placed. The retreat is free: it runs while
    # nothing else is waiting on the arm, and it leaves the cameras a clear
    # view of exactly the thing about to be judged.
    ph.mark("release_and_settle")
    if stacking:
        # Park as a WITNESS, not just out of the way: from a step back toward
        # the base at low height, this arm's wrist camera stares at the tower
        # it just built — the side view that the top-down occlusion and the
        # front camera's grazing sightline both miss. verify's witness frames
        # include wrist cameras, so the goal check reads this view for free.
        try:
            base_xy = np.asarray(ctx.cfg.arms[side].t_world_base, float)[:2, 3]
            back = base_xy - np.asarray(dest[:2], float)
            back = back / max(float(np.linalg.norm(back)), 1e-9)
            witness = np.array([dest[0] + back[0] * 0.15,
                                dest[1] + back[1] * 0.15, 0.18])
            ctx.robot.move_cartesian(side, witness, seconds=1.5)
            ctx.log.event("stack_witness_park", target=target,
                          xyz=[round(float(v), 4) for v in witness])
        except Exception:
            park_for_look(ctx, side)
    else:
        park_for_look(ctx, side)
    ph.mark("park")

    # LOOK AT WHERE IT LANDED, AND IF THAT IS NOT WHERE IT WAS SENT, DO IT
    # AGAIN. The correction is a difference between two measurements taken the
    # same way, so whatever the world map gets wrong about this patch of table
    # cancels: we are not asking "where is it really" but "how far is it from
    # where I meant", and both readings carry the same bias.
    corrected_mm = None
    if precise:
        from .sensing import forget_the_look, ground_entity  # noqa: PLC0415

        for attempt in range(PLACE_PRECISE_TRIES):
            _belief(ctx).invalidate_entity(entity, reason="placed; measuring where")
            forget_the_look(ctx, reason="placed; looking at where it landed")
            seen, note = ground_entity(ctx, entity, need_look=True)
            if not seen:
                ctx.log.event("place_precise_blind", entity=entity, note=note[:90])
                break
            now = np.asarray(_belief(ctx).track(entity).xyz_base, float)
            off = np.asarray(aim[:2], float) - now[:2]
            gap = float(np.linalg.norm(off))
            # Good enough is half the object's own width: the centre of mass is
            # still over the support and the object stands. Tighter than that
            # is chasing the measurement's own noise — an 8 mm reading on a
            # cleanly landed 21 mm-cube stack triggered a re-grasp whose
            # support re-ground matched a bowl (2026-08-19), turning a finished
            # stack into a failure.
            span_now = _belief(ctx).track(entity).span_m
            tol = max(PLACE_PRECISE_TOL_M,
                      0.5 * float(span_now)) if span_now else PLACE_PRECISE_TOL_M
            ctx.log.event("place_precise_measured", entity=entity, attempt=attempt + 1,
                          landed=[round(float(v), 4) for v in now[:2]],
                          aimed=[round(float(v), 4) for v in aim[:2]],
                          off_mm=round(gap * 1000, 1),
                          tol_mm=round(tol * 1000, 1))
            if gap <= tol:
                corrected_mm = round(gap * 1000, 1)
                break
            if attempt + 1 >= PLACE_PRECISE_TRIES:
                corrected_mm = round(gap * 1000, 1)
                break
            # Pick it up from where it IS and put it down again. NO mirrored
            # extrapolation: aim + (aim - landed) presumes the miss was a
            # systematic aiming bias, but a stack topple is not a bias — on
            # the rig (2026-08-18) a 94 mm tumble off the support's edge got
            # mirrored into an aim 94 mm past the support, onto bare table,
            # while the support itself had been shoved and its stale position
            # was never re-read. A big miss is evidence the WORLD moved, not
            # the aim: re-ground the support and aim at where it is now; with
            # no support to re-ground, retry the original aim unchanged.
            try:
                pick(ctx, entity, arm=side)
            except Exception as e:
                ctx.log.event("place_precise_regrasp_failed", entity=entity,
                              error=f"{type(e).__name__}: {e}"[:110])
                break
            retry_at = np.asarray(aim, float).copy()
            if target is not None:
                prior_t = _belief(ctx).track(target).xyz_base
                _belief(ctx).invalidate_entity(
                    target, reason="place missed; the support may have moved")
                t_seen, t_note = ground_entity(ctx, target, need_look=True)
                fresh = _belief(ctx).track(target).xyz_base
                if t_seen and fresh is not None:
                    jump = (float(np.linalg.norm(
                        np.asarray(fresh[:2], float) - np.asarray(prior_t[:2], float)))
                        if prior_t is not None else 0.0)
                    if jump > RETARGET_MAX_JUMP_M:
                        # A support that was under a felt touch seconds ago did
                        # not teleport across the table; a re-ground that far
                        # away matched a LOOKALIKE (the white bowl, 135 mm off,
                        # was matched as an occluded support cube 2026-08-19).
                        # Keep the aim; drop the impostor belief too.
                        ctx.log.event("place_precise_retarget_rejected",
                                      target=target, jump_mm=round(jump * 1000, 1),
                                      xyz=[round(float(v), 4) for v in fresh[:2]])
                        _belief(ctx).invalidate_entity(
                            target, reason="re-ground jumped implausibly far")
                    else:
                        retry_at = np.asarray(fresh, float)
                        ctx.log.event("place_precise_retarget", target=target,
                                      xyz=[round(float(v), 4) for v in retry_at])
                else:
                    ctx.log.event("place_precise_target_lost", target=target,
                                  note=str(t_note)[:90])
            # A point derived from a measurement is a destination, not a hint:
            # handed back through x/y it would be snapped to a clear spot.
            return place(ctx, entity, target=target, at=retry_at,
                         arm=side, contact=contact, precise=False)
    ph.done()

    g = ctx.robot.get_gripper(side)
    return SkillResult(
        ok=True,
        info=f"place({entity}) -> {target or (x, y)} at {np.round(dest, 3).tolist()} with {side} arm"
        + (f" ({corrected_mm:.1f} mm from the aim after correction)" if corrected_mm is not None else "")
        + (f" (slot {slot}: something was already there)" if slot else ""),
        proprio={"width_m": g["width_m"], "effort": g["effort"]},
    )


@skill(
    name="push",
    description=("Push an entity by sliding the closed gripper through it — either toward a named "
                 "entity (preferred: direction and distance are measured at execution) or along an "
                 "explicit world-frame (dx, dy)."),
    params={
        "entity": {"type": "string"},
        "toward": {"type": "string",
                   "description": "entity to push toward; ends beside it (satisfies near(entity, toward))"},
        "dx": {"type": "number", "description": "push displacement x (m); ignored when toward is given"},
        "dy": {"type": "number", "description": "push displacement y (m); ignored when toward is given"},
        "arm": {"type": "string", "enum": ["left", "right", "auto"]},
    },
    required=["entity"],
    post_hint="entity displaced; location must be re-perceived (near(entity, toward) if toward given)",
)
def push(ctx: SkillContext, entity: str, dx: float = 0.0, dy: float = 0.0,
         toward: Optional[str] = None, arm: str = "auto", at=None) -> SkillResult:
    # `at` is the dataflow executor's value channel, same as pick's: a position
    # a look already produced. push consumes the entity value, so it must be
    # able to catch it — the signature missing this parameter ended episodes
    # with a TypeError three repairs deep.
    if at is not None:
        target = np.asarray(getattr(at, "xyz", at), dtype=float)
    else:
        target = _resolve_point(ctx, entity)
    if toward:
        # "Push the plate to the front of the stove" names a DESTINATION, not a
        # vector, and the planner has no numbers to compute one from — asked for
        # dx/dy anyway, it emitted zeros and the guard below ended the episode.
        # So measure at execution, the way every other primitive grounds its
        # arguments: aim at the target's centre and stop with the cargo's own
        # centre at the target's edge — adjacent, inside near()'s limit, without
        # ramming the fixture.
        from ..verify import _half_extent  # noqa: PLC0415  (verify imports this package)
        dest = _resolve_point(ctx, toward)
        gap = np.asarray(dest[:2], float) - np.asarray(target[:2], float)
        span = float(np.linalg.norm(gap))
        if span < 1e-6:
            raise ValueError(f"push toward {toward}: it is at the same spot as {entity}")
        # Aim the CENTRE a little onto the rim, not at it: a push under-delivers
        # (the block slides less than the fingertip travels — measured 69 mm of
        # loss on the twin's first sweep), and stopping the centre exactly at
        # the edge left it 8 mm outside near()'s own limit. The loop below
        # re-measures and sweeps again, so overshooting the aim is safe.
        run = span - _half_extent(ctx, toward) * 0.5
        if run <= 0:
            return SkillResult(ok=True, info=f"push({entity} -> {toward}): already adjacent "
                                             f"({span * 1000:.0f} mm apart); nothing to do")
        dx, dy = (gap / span * run).tolist()
    side = choose_arm(ctx, target, arm)
    direction = np.array([dx, dy, 0.0])
    n = np.linalg.norm(direction[:2])
    if n < 1e-6:
        raise ValueError("push needs a nonzero dx/dy (or a `toward` entity)")
    start = target - direction / n * 0.06
    # PUSH_DEPTH_Z is a height ABOVE THE TABLE, and it was applied as an
    # absolute z. With the table at -0.015 the sweep ran 35 mm up — grazing the
    # top edge of a 30 mm block. The twin showed the symptom exactly: five
    # pushes, sensible vectors, and the block skittering SIDEWAYS off the
    # fingertip instead of travelling; the rig showed clean air.
    # AND HOW LOW DEPENDS ON WHAT IS BEING PUSHED. PUSH_DEPTH_Z is a block's
    # waist; on a plate 19 mm tall the fingertip swept at 20 mm passed clean
    # over the top and the plate did not move at all (rig, green plate, 100 mm
    # commanded, 20 mm measured — inside the grounding noise). Sweep at a
    # fraction of the object's OWN height, which is a block's waist for a block
    # and a plate's wall for a plate.
    height = max(0.0, float(target[2]) - ctx.cfg.table_z)
    sweep = min(PUSH_DEPTH_Z, 0.4 * height) if height > 0 else PUSH_DEPTH_Z
    start[2] = max(ctx.cfg.safety.workspace_z[0], ctx.cfg.table_z + sweep)
    ctx.log.event("push_sweep_height", entity=entity,
                  object_height_mm=round(height * 1000, 1),
                  sweep_z=round(float(start[2]), 4),
                  floored=bool(ctx.cfg.table_z + sweep < ctx.cfg.safety.workspace_z[0]))
    end = target + direction
    end[2] = start[2]

    # A PUSH IS THE ONE PRIMITIVE THAT CANNOT BE AIMED. The object leaves the
    # fingertip the moment they part company, so where it stops depends on
    # friction, on where the contact landed relative to the object's centre, and
    # on how round it is. Measured on the rig's green plate: 100 mm commanded,
    # 37 mm travelled, and two thirds of that sideways — a disc contacted off
    # centre rotates away rather than sliding along. Every one of those errors
    # is visible AFTERWARDS, so the sweep is repeated against a measurement
    # rather than computed more carefully. Open loop was the whole problem.
    goal_xy = np.asarray(target[:2], float) + np.asarray([dx, dy], float)
    swept = 0
    for attempt in range(PUSH_MAX_SWEEPS):
        ctx.robot.set_gripper(side, 0.0)
        travel(ctx, side, approach_pose(ctx, start[:2], start[2], side))
        ctx.robot.move_cartesian(side, start, seconds=1.5)
        ctx.robot.move_cartesian(side, end, seconds=2.0)
        ctx.robot.move_cartesian(side, end + np.array([0, 0, HOVER_CLEARANCE]), seconds=1.5)
        swept += 1
        _belief(ctx).invalidate_entity(entity, reason="pushed; must re-perceive")

        from .sensing import forget_the_look, ground_entity  # noqa: PLC0415
        park_for_look(ctx, side)
        forget_the_look(ctx, reason="pushed; the arm has moved out of the way")
        found, note = ground_entity(ctx, entity, need_look=True)
        now = _belief(ctx).track(entity).xyz_base if found else None
        if now is None:
            return SkillResult(ok=True, info=(
                f"push({entity}) swept {swept}x toward ({dx:.2f},{dy:.2f}) but the "
                f"object could not be re-found afterwards — {note[:80]}"))
        now = np.asarray(now, float)
        left = goal_xy - now[:2]
        gap = float(np.linalg.norm(left))
        ctx.log.event("push_progress", entity=entity, sweep=swept,
                      at=[round(float(v), 3) for v in now[:2]],
                      remaining_mm=round(gap * 1000, 1))
        if gap <= PUSH_GOAL_TOL_M:
            break
        if attempt + 1 >= PUSH_MAX_SWEEPS:
            break
        # Re-aim from where it actually is, at what is actually left.
        n2 = float(np.linalg.norm(left))
        target = np.array([now[0], now[1], float(target[2])])
        direction = np.array([left[0], left[1], 0.0])
        start = target - direction / n2 * 0.06
        start[2] = max(ctx.cfg.safety.workspace_z[0], ctx.cfg.table_z + sweep)
        end = target + direction
        end[2] = start[2]

    return SkillResult(ok=True, info=(
        f"push({entity}) by ({dx:.2f},{dy:.2f}) with {side} arm in {swept} sweep"
        f"{'s' if swept > 1 else ''}; {gap * 1000:.0f} mm short of the aim"))


def handoff_site(ctx: SkillContext, giver: str, receiver: str) -> np.ndarray:
    """Where both arms can reach — searched for, not written down.

    The two bases are 0.93 m apart and each arm reaches about 0.6 m, so the
    overlap on this rig is one column about 40 mm wide. A constant would be
    right for this table and wrong for the next one, and wrong silently. The
    overlap of two reaches centred on two bases straddles the midpoint of
    those bases, so the search starts there and spirals out; on this geometry
    it succeeds on the first candidate.

    Both the set-down height and a hover above it must be reachable by both
    arms: the giver has to get out from over the object, and the receiver has
    to come down onto it.
    """
    def base_xy(a: str) -> np.ndarray:
        return np.asarray(ctx.cfg.arms[a].t_world_base, dtype=float)[:2, 3]

    mid = (base_xy(giver) + base_xy(receiver)) / 2.0
    z = max(ctx.cfg.table_z + HANDOFF_RELEASE_Z_M,
            float(ctx.cfg.safety.workspace_z[0]))
    tried = 0
    for r in np.arange(0.0, HANDOFF_SEARCH_MAX_M, HANDOFF_SEARCH_STEP_M):
        n = 1 if r < 1e-6 else max(8, int(2 * np.pi * r / HANDOFF_SEARCH_STEP_M))
        for k in range(n):
            th = 2 * np.pi * k / n
            xy = mid + r * np.array([np.cos(th), np.sin(th)])
            p = np.array([xy[0], xy[1], z])
            tried += 1
            if all(ctx.robot.reachable(a, p)
                   and ctx.robot.reachable(a, p + [0, 0, HANDOFF_HOVER_M])
                   for a in (giver, receiver)):
                ctx.log.event("handoff_site", xyz=[round(float(v), 3) for v in p],
                              searched=tried,
                              from_midpoint_mm=round(float(np.linalg.norm(xy - mid)) * 1000, 1))
                return p
    raise ValueError(
        f"the {giver} and {receiver} arms have no common ground within "
        f"{HANDOFF_SEARCH_MAX_M:.2f} m of the midpoint between their bases "
        f"({mid.round(3).tolist()}) — they cannot hand anything over, and a task "
        f"that needs both of them cannot be done on this rig")


@skill(
    name="handover",
    description=(
        "Pass a held object to the other arm by setting it down where both arms reach "
        "and picking it back up. Use when the arm holding something cannot reach where "
        "it has to go — check with the destination, not by guessing."
    ),
    params={
        "entity": {"type": "string", "description": "the entity currently held"},
        "to_arm": {"type": "string", "enum": ["left", "right", "auto"]},
    },
    required=["entity"],
    post_hint="holding(entity, to_arm)",
)
def handover(ctx: SkillContext, entity: str, to_arm: str = "auto") -> SkillResult:
    """Move `entity` from the arm that holds it into the other arm.

    Deliberately a table handoff, not a gripper-to-gripper one. Two grippers
    meeting in the air have to agree on a point to millimetres while neither
    can see it; a table takes the object at whatever height it is let go and
    holds it still while the second arm looks. The cost is one extra descent.
    """
    arms = list(getattr(ctx.cfg, "active_arms", None) or ctx.robot.arms)
    tr = _belief(ctx).track(entity)
    giver = tr.held_by
    if giver is None:
        raise ValueError(
            f"nothing to hand over: {entity!r} is not held by either arm. Pick it "
            f"up first. (Arms on this robot: {arms}.)")
    if len(arms) < 2:
        raise ValueError(
            f"this robot is running with one arm ({arms}); there is no second arm "
            f"to hand {entity!r} to")
    receiver = to_arm if (to_arm in arms and to_arm != giver) \
        else next(a for a in arms if a != giver)

    site = handoff_site(ctx, giver, receiver)
    # AS A VALUE, NOT AS x/y. The x/y path treats a destination as a wish and
    # snaps it to the nearest clear reachable spot — right for "put it down
    # somewhere over there", fatal here: it moved the release 88 mm, out of the
    # receiving arm's reach and onto a plate, and the handoff had nothing to
    # pick up. The site is the one point on this table both arms can occupy;
    # there is nothing nearby to snap to.
    place(ctx, entity, at=site, arm=giver, precise=True)
    # THE GIVER LEAVES BEFORE THE RECEIVER ARRIVES. The site is at the far edge
    # of both reaches, so the giver's forearm lies across exactly the space the
    # receiver has to descend through — and across the cameras that would find
    # the object there.
    park_for_look(ctx, giver)
    from .sensing import forget_the_look  # noqa: PLC0415  (sensing imports this package)
    forget_the_look(ctx, reason="handed over; the giving arm has moved away")
    pick(ctx, entity, arm=receiver)
    now = _belief(ctx).track(entity)
    if now.held_by != receiver:
        raise ValueError(
            f"handover({entity}) put it down at {site.round(3).tolist()} but the "
            f"{receiver} arm did not take it — it is on the table in the shared "
            f"strip, within reach of both arms")
    return SkillResult(ok=True, info=(
        f"handover({entity}) from {giver} to {receiver} via "
        f"({site[0]:.3f}, {site[1]:.3f})"))


@skill(
    name="home",
    description="Send an arm (or both) to its home pose, clearing the camera view.",
    params={"arm": {"type": "string", "enum": ["left", "right", "both"]}},
    required=[],
    post_hint="at_home(arm); overhead cameras unobstructed",
)
def home(ctx: SkillContext, arm: str = "both") -> SkillResult:
    sides = ctx.robot.arms if arm == "both" else [arm]
    for side in sides:
        ctx.robot.home(side)
    return SkillResult(ok=True, info=f"homed {sides}")


@skill(
    name="open_gripper",
    description="Open an arm's gripper (drops anything held at the current pose).",
    params={"arm": {"type": "string", "enum": ["left", "right"]}},
    required=["arm"],
    post_hint="gripper_empty(arm)",
)
def open_gripper(ctx: SkillContext, arm: str) -> SkillResult:
    ctx.robot.set_gripper(arm, OPEN_WIDTH)
    for tr in _belief(ctx).entities.values():
        if tr.held_by == arm:
            tr.held_by = None
            _belief(ctx).invalidate_entity(tr.id, reason="released by open_gripper; location unknown")
    g = ctx.robot.get_gripper(arm)
    return SkillResult(ok=True, info=f"opened {arm} gripper", proprio=g)
