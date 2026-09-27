"""Failure diagnosis: turn "postcondition false" into a *classified* failure.

Pigey's recovery quality depended on the VLM re-reading its own scrollback;
here a deterministic procedure narrows the cause first, so repair (rule-based
or orchestrator-proposed) starts from a typed FailureReport instead of a vibe.

Classification questions, in order:
  1. Did the skill even run? (SAFETY / SKILL_EXEC)
  2. Does an independent verifier disagree? (PERCEPTION)
  3. Where is the object actually? Same place = effect never happened;
     elsewhere = dropped mid-way (both SKILL_EFFECT); missing = lost track.
  4. Did the world change without us acting on it? (STATE_DRIFT — detected at
     precondition time)
  5. Out of attempts / plan can't work as written? (PROGRAM)
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .program import Step
from .skills import SkillContext
from .skills.sensing import ground_entity
from .types import FailureClass, FailureReport, PredicateSpec, SkillResult, Value
from .verify import check

DISPLACED_THRESHOLD_M = 0.05


def _primary_entity(spec: PredicateSpec) -> Optional[str]:
    return spec.args[0] if spec.args else None


def diagnose_exec_error(step: Step, result: SkillResult) -> FailureReport:
    err = result.error or "unknown execution error"
    if "SafetyViolation" in err or "EStop" in err:
        cls = FailureClass.SAFETY
    elif "ValueError" in err or "unknown args" in err or "disabled" in err:
        # Deterministic contract violations: retrying identically cannot help.
        cls = FailureClass.PROGRAM
    else:
        cls = FailureClass.SKILL_EXEC
    return FailureReport(step_id=step.id, skill=step.skill, failure_class=cls,
                         summary=err + " (deterministic error — retrying the same step unchanged will fail again)"
                         if cls is FailureClass.PROGRAM else err,
                         attempts=step.attempts)


def diagnose_postcondition(
    ctx: SkillContext, step: Step, failed: list[PredicateSpec], result: SkillResult
) -> FailureReport:
    evidence_ids: list[str] = []
    details: dict = {"skill_info": result.info}

    # (2) Independent re-verification: does a second method disagree?
    # Overturning needs CONFIDENT contradiction — and proprioception is close to
    # ground truth, so a visual VQA may only overrule it at high confidence
    # (Pigey lesson kept: scoop-grips exist; VLM hallucinations are commoner).
    OVERRIDE_MIN_CONF = 0.85
    contradicted: list[str] = []
    for spec in failed:
        sat, verdict = check(ctx, spec, alternate=True)
        evidence_ids.append(verdict.evidence.id)
        if sat is Value.TRUE and verdict.confidence >= OVERRIDE_MIN_CONF:
            contradicted.append(spec.key)
    if contradicted and len(contradicted) == len(failed):
        return FailureReport(
            step_id=step.id, skill=step.skill, failure_class=FailureClass.PERCEPTION,
            summary=f"primary verifier said {[s.key for s in failed]} failed but independent checks say satisfied",
            failed_predicates=failed, evidence_ids=evidence_ids, attempts=step.attempts,
            details={"contradicted": contradicted, **details},
        )

    # (3) Locate the primary entity to distinguish miss / drop / lost.
    entity = next((e for e in (_primary_entity(s) for s in failed) if e), None)
    if entity is not None:
        before = ctx.belief.track(entity).xyz_base
        found, note = ground_entity(ctx, entity)
        details["relocate_note"] = note
        if found:
            after = np.asarray(ctx.belief.track(entity).xyz_base)
            moved = before is not None and float(np.linalg.norm(after[:2] - np.asarray(before[:2]))) > DISPLACED_THRESHOLD_M
            mode = "displaced_during_skill" if moved else "effect_never_happened"
            details["failure_mode"] = mode
            summary = (
                f"{step.skill} ran but {[s.key for s in failed]} still false; "
                f"{entity} {'moved to ' + str(np.round(after, 3).tolist()) if moved else 'is still at its prior location'}"
            )
        else:
            details["failure_mode"] = "entity_lost"
            summary = f"{step.skill} ran but {entity} is no longer visible from fixed cameras"
        return FailureReport(
            step_id=step.id, skill=step.skill, failure_class=FailureClass.SKILL_EFFECT,
            summary=summary, failed_predicates=failed, evidence_ids=evidence_ids,
            attempts=step.attempts, details=details,
        )

    return FailureReport(
        step_id=step.id, skill=step.skill, failure_class=FailureClass.SKILL_EFFECT,
        summary=f"postconditions {[s.key for s in failed]} not satisfied after {step.skill}",
        failed_predicates=failed, evidence_ids=evidence_ids, attempts=step.attempts, details=details,
    )


def diagnose_precondition(
    ctx: SkillContext, step: Step, failed: list[PredicateSpec]
) -> FailureReport:
    """A precondition believed satisfied earlier is now false → drift vs stale plan."""
    was_true_before = any(ctx.belief.was_ever(s, Value.TRUE if not s.negated else Value.FALSE) for s in failed)
    cls = FailureClass.STATE_DRIFT if was_true_before else FailureClass.PROGRAM
    summary = (
        f"preconditions {[s.key for s in failed]} do not hold before {step.skill}"
        + (" (they held earlier — world changed outside our actions)" if cls is FailureClass.STATE_DRIFT else
           " (never established — the plan assumed them)")
    )
    return FailureReport(step_id=step.id, skill=step.skill, failure_class=cls,
                         summary=summary, failed_predicates=failed, attempts=step.attempts)


def escalate(report: FailureReport) -> FailureReport:
    """Attempts exhausted: same evidence, but now it's the program's problem."""
    return FailureReport(
        step_id=report.step_id, skill=report.skill, failure_class=FailureClass.PROGRAM,
        summary=f"exhausted local repairs ({report.attempts} attempts): {report.summary}",
        failed_predicates=report.failed_predicates, evidence_ids=report.evidence_ids,
        attempts=report.attempts, details=report.details,
    )
