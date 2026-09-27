"""Acting in order to see: the physical half of information gathering.

Every other skill in this package changes the world because the task wants it
changed. These change it because the agent cannot otherwise KNOW something —
the object is under a cup, behind a box, inside a drawer, and no amount of
looking from where the cameras are will settle it. The move is: displace the
occluder, look, put it back, and report what the look produced.

The put-back matters. A search that leaves the scene rearranged has changed
the task it was searching for, and the next episode inherits the mess; the
cup goes back where it stood so that ten reveals cost ten looks and nothing
else.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from ..types import SkillResult
from . import SkillContext, skill

# How far aside the occluder is carried before the look. Far enough that the
# object under it is clear of the occluder's own shadow, near enough that the
# arm stays inside the dexterous band it picked the occluder up from.
ASIDE_M = 0.10
LIFT_M = 0.10


@skill(
    name="lift_and_peek",
    description=(
        "Lift an occluder (a cup, a box, a lid), look underneath for an entity, "
        "and put the occluder back where it stood. Use when a target cannot be "
        "seen and something in the scene could be hiding it. Reports whether the "
        "entity was found, and leaves the scene as it was."
    ),
    params={
        "occluder": {"type": "string", "description": "entity to lift"},
        "look_for": {"type": "string",
                     "description": "entity to search for underneath; defaults to the entity the goal is about"},
        "arm": {"type": "string", "enum": ["left", "right", "auto"]},
    },
    required=["occluder"],
    post_hint="visible(look_for) true when the entity was under this occluder; the occluder is back where it stood",
)
def lift_and_peek(ctx: SkillContext, occluder: str, look_for: Optional[str] = None,
                  arm: str = "auto") -> SkillResult:
    from .primitives import choose_arm, pick, place  # noqa: PLC0415
    from .sensing import forget_the_look, ground_entity  # noqa: PLC0415

    # WHAT WE ARE LOOKING FOR IS NOT A MYSTERY. The planner repeatedly emitted
    # lift_and_peek(occluder=cup) and left the target implicit — reasonably, in
    # a plan whose whole goal is one entity — and the step died on a missing
    # argument three times in a row. The goal predicates say what the task is
    # about; asking the model to restate it is a formality, not information.
    if not look_for:
        for _name, args, _neg in getattr(ctx, "goal_predicates", None) or []:
            cand = next((a for a in args if a != occluder
                         and a in ctx.belief.entities), None)
            if cand:
                look_for = cand
                break
    if not look_for:
        raise ValueError(
            "lift_and_peek: no look_for given and the goal names no other "
            "entity to search for")

    tr = ctx.belief.track(occluder)
    if tr.xyz_base is None:
        found, note = ground_entity(ctx, occluder, need_look=True)
        if not found:
            raise ValueError(f"lift_and_peek: {occluder} not found — {note}")
        tr = ctx.belief.track(occluder)
    home_xy = (float(tr.xyz_base[0]), float(tr.xyz_base[1]))
    side = choose_arm(ctx, np.asarray([*home_xy, ctx.cfg.table_z]), arm)

    pick(ctx, occluder, arm=side)
    # Carry it aside along +y (toward the arm's own working room) rather than
    # straight up: an occluder held overhead is exactly where the cameras were
    # about to look.
    aside = (home_xy[0], home_xy[1] + ASIDE_M)
    ctx.robot.move_cartesian(
        side, np.asarray([aside[0], aside[1], ctx.cfg.table_z + LIFT_M]), seconds=1.5)
    ctx.log.event("peek_lifted", occluder=occluder, look_for=look_for,
                  from_xy=[round(v, 3) for v in home_xy])

    # The look. A stale sighting must not answer a question the whole point of
    # which is that the world just changed.
    forget_the_look(ctx, reason=f"lifted {occluder} to look for {look_for}")
    found, note = ground_entity(ctx, look_for, need_look=True)
    ctx.log.event("peek_result", occluder=occluder, look_for=look_for,
                  found=bool(found), note=note[:160])

    # Put it back, whatever the answer was.
    try:
        place(ctx, occluder, x=home_xy[0], y=home_xy[1], arm=side)
    except Exception as e:
        ctx.log.event("peek_restore_failed", occluder=occluder,
                      error=f"{type(e).__name__}: {e}"[:150])

    where = ""
    if found:
        p = ctx.belief.track(look_for).xyz_base
        if p is not None:
            where = f" at [{p[0]:.3f}, {p[1]:.3f}]"
    return SkillResult(
        ok=True,
        info=(f"looked under {occluder}: {look_for} "
              + (f"FOUND{where}" if found else "not there")
              + f"; {occluder} restored to {[round(v, 3) for v in home_xy]}"),
    )
