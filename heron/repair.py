"""Repair policy: rules fix the cheap, common cases; the orchestrator is
consulted only for PROGRAM/SAFETY-class failures or when rules run dry.

This is the escalation ladder Pigey enforced with scattered guard patches
(forced re-Perceive, forced VLA escalation), promoted to one place with the
spiral-regrasp trick from its sim harness built in.
"""
from __future__ import annotations

from .program import EditType, ProgramEdit, Step, StepStatus, TaskProgram
from .skills import SkillContext
from .types import FailureClass, FailureReport

# Regrasp jitter schedule, meters (attempt 2 uses index 0, etc.).
# NO BLIND JITTER anywhere in this file. A 20 mm spiral was the old answer to a
# missed grasp; on a 30 mm block it moves the aim from the centre to the edge,
# converting a near miss into a certain one. It was the third self-applied
# offset this rig was bitten by (learned place hints, cross-object grasp hints,
# this one). A retry changes what it can justify changing: a FRESH look and a
# different grasp STRATEGY. Aim where perception says.


def rule_based_repair(
    ctx: SkillContext, program: TaskProgram, step: Step, report: FailureReport
) -> list[ProgramEdit] | None:
    """Return edits to apply, or None to escalate to the orchestrator."""
    if report.failure_class is FailureClass.PERCEPTION:
        # Independent check said the postcondition actually holds — no edit;
        # the agent marks the step done and records the contradiction.
        return []

    if report.failure_class is FailureClass.STATE_DRIFT:
        if any(s.name not in ("visible", "in", "on") for s in report.failed_predicates):
            # e.g. holding() drifted — re-perceiving can't fix that; the program
            # needs new steps, which is the orchestrator's call.
            for spec in report.failed_predicates:
                if spec.args:
                    ctx.belief.invalidate_entity(spec.args[0], reason="state drift detected")
            return None
        for spec in report.failed_predicates:
            if spec.args:
                ctx.belief.invalidate_entity(spec.args[0], reason="state drift detected")
        perceive = Step(
            id=program.fresh_step_id(), skill="perceive",
            args={"entities": sorted({a for s in report.failed_predicates for a in s.args if a in ctx.belief.entities})
                  or ["all"]},
            rationale="re-ground after external change",
        )
        return [ProgramEdit(type=EditType.INSERT_STEPS, step_id=step.id, steps=[perceive],
                            rationale=f"drift: {report.summary}", source="rules")]

    if (report.failure_class is FailureClass.SKILL_EXEC and step.skill == "pick"
            and "closed on air" in (report.summary or "") and step.attempts < step.max_attempts):
        # The gripper itself felt the miss — same recovery as a verified grasp
        # miss: fresh grounding, then retry with the spiral jitter schedule.
        edits = []
        entity = step.args.get("entity")
        if entity:
            perceive = Step(id=program.fresh_step_id(), skill="perceive", args={"entities": [entity]},
                            rationale="fresh grounding before regrasp")
            edits.append(ProgramEdit(type=EditType.INSERT_STEPS, step_id=step.id, steps=[perceive],
                                     rationale="re-ground before retry", source="rules"))
        # Change the STRATEGY, never the aim: the same approach retried on a
        # fresh grounding re-buys the same miss, but nudging the aim off a
        # correctly grounded 30 mm block guarantees one.
        ladder = ["no_service", "plain", "auto"]
        strat = ladder[(step.attempts - 1) % len(ladder)]
        edits.append(ProgramEdit(type=EditType.RETRY_STEP, step_id=step.id,
                                 new_args={"strategy": strat},
                                 rationale=f"retry closed-on-air with strategy {strat}",
                                 source="rules"))
        return edits

    if (report.failure_class is FailureClass.PROGRAM
            and "never established" in (report.summary or "")
            and any(sp.name == "holding" and not sp.negated
                    for sp in report.failed_predicates)):
        # The plan reached a step that needs the object IN HAND and nothing
        # ever picked it up. Completing the missing action is mechanical —
        # measured, the orchestrator spent 37 s of thinking to answer
        # "insert pick" for a place whose plan simply lacked one.
        spec = next(sp for sp in report.failed_predicates
                    if sp.name == "holding" and not sp.negated)
        entity = spec.args[0] if spec.args else None
        if entity:
            pick_step = Step(id=program.fresh_step_id(), skill="pick",
                             args={"entity": entity},
                             rationale="the plan assumed this was already held")
            return [ProgramEdit(type=EditType.INSERT_STEPS, step_id=step.id,
                                steps=[pick_step],
                                rationale="missing pick before a hold-requiring step",
                                source="rules")]

    if report.failure_class is FailureClass.SKILL_EXEC and "stale" in (report.summary or "").lower():
        perceive = Step(id=program.fresh_step_id(), skill="perceive", args={"entities": ["all"]},
                        rationale="refresh stale grounding")
        return [ProgramEdit(type=EditType.INSERT_STEPS, step_id=step.id, steps=[perceive],
                            rationale="stale grounding", source="rules")]

    if report.failure_class is FailureClass.SKILL_EFFECT and step.attempts < step.max_attempts:
        edits: list[ProgramEdit] = []
        entity = report.failed_predicates[0].args[0] if report.failed_predicates and report.failed_predicates[0].args else None
        if entity:
            perceive = Step(id=program.fresh_step_id(), skill="perceive", args={"entities": [entity]},
                            rationale="fresh grounding before retry")
            edits.append(ProgramEdit(type=EditType.INSERT_STEPS, step_id=step.id, steps=[perceive],
                                     rationale="re-ground before retry", source="rules"))
        # A fresh look is the retry. Moving the aim is not.
        edits.append(ProgramEdit(type=EditType.RETRY_STEP, step_id=step.id, new_args=None,
                                 rationale=f"retry after {report.details.get('failure_mode', 'effect failure')}",
                                 source="rules"))
        return edits

    return None  # PROGRAM, SAFETY, exhausted attempts → orchestrator


def validate_edit(program: TaskProgram, edit: ProgramEdit, known_skills: list[str]) -> str | None:
    """Dry-run an orchestrator-proposed edit; returns an error string or None.

    A malformed proposal must bounce back as feedback, never corrupt the plan.
    """
    for s in edit.steps:
        if s.skill not in known_skills:
            return f"edit uses unknown skill {s.skill!r}; available: {known_skills}"
        if s.status is not StepStatus.PENDING:
            return f"inserted step {s.id} must be pending"
    trial = program.model_copy(deep=True)
    try:
        trial.apply_edit(edit.model_copy(deep=True))
    except Exception as e:
        return f"edit does not apply: {e}"
    return None
