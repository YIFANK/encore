"""The task program: an explicit, editable plan instead of implicit chat history.

Pigey kept task state, plan, and recovery in the VLM's interaction history; the
only structure was a scene-memory JSON re-rendered into the prompt. Heron makes
the program the primary object: the planner emits it, the executor walks it,
verification gates each step, and repair is a typed *edit* to it. Every edit is
recorded, so an episode ends with the full history of what the agent believed
the plan was and why it changed.
"""
from __future__ import annotations

import time
from enum import Enum
from typing import Any, ClassVar, Optional

from pydantic import BaseModel, Field

from .types import EntityDecl, PredicateSpec


class StepStatus(str, Enum):
    PENDING = "pending"
    RUNNING = "running"
    DONE = "done"
    FAILED = "failed"
    SKIPPED = "skipped"


class Step(BaseModel):
    id: str
    skill: str
    args: dict[str, Any] = Field(default_factory=dict)  # "@entity_id" strings resolve via belief
    pre: list[PredicateSpec] = Field(default_factory=list)
    post: list[PredicateSpec] = Field(default_factory=list)
    # A GUARD, not a precondition. A failed precondition is a fault to repair;
    # a false guard means the step is simply not needed on this pass — "orient
    # the connector IF it is misaligned" skips silently when it is straight.
    # Three-valued honesty: a guard that verifies UNKNOWN runs the step, because
    # guarded steps are protective actions that are safe when unnecessary, and
    # skipping on ignorance turns "if unsure, do nothing" into a policy nobody
    # chose. The verify call gathers evidence first, so UNKNOWN means "looked
    # and still cannot tell", not "did not look".
    when: Optional[PredicateSpec] = None
    rationale: str = ""
    status: StepStatus = StepStatus.PENDING
    attempts: int = 0
    max_attempts: int = 2
    # Set when this step came from expanding a learned skill, so the outcome —
    # including any repair the executor had to make — can be credited back to it.
    origin_skill: Optional[str] = None
    origin_args: dict[str, Any] = Field(default_factory=dict)

    def compact(self) -> str:
        pre = ", ".join(str(p) for p in self.pre) or "-"
        post = ", ".join(str(p) for p in self.post) or "-"
        return f"[{self.id}:{self.status.value}] {self.skill}({self.args}) pre: {pre} post: {post}"


class EditType(str, Enum):
    RETRY_STEP = "retry_step"  # re-run the step, optionally with new args
    INSERT_STEPS = "insert_steps"  # add recovery steps before a step
    REPLACE_STEP = "replace_step"  # swap a step for one or more alternatives
    SKIP_STEP = "skip_step"  # postcondition already holds / step unnecessary
    REGROUND_ENTITY = "reground_entity"  # description was wrong; update it and re-ground
    REPLACE_TAIL = "replace_tail"  # replan from a step onward (keeps completed prefix)
    ABORT = "abort"  # goal judged unreachable


class ProgramEdit(BaseModel):
    """One local repair. Applied by TaskProgram.apply_edit, never by hand."""

    type: EditType
    step_id: Optional[str] = None
    new_args: Optional[dict[str, Any]] = None  # RETRY_STEP
    steps: list[Step] = Field(default_factory=list)  # INSERT/REPLACE/REPLACE_TAIL
    entity_id: Optional[str] = None  # REGROUND_ENTITY
    new_description: Optional[str] = None  # REGROUND_ENTITY
    rationale: str = ""
    source: str = "rules"  # rules | orchestrator
    t: float = Field(default_factory=time.time)


class TaskProgram(BaseModel):

    # Cycle tasks (terminal state == start state) are graded by their steps.
    process_goal: bool = False
    goal: str
    goal_predicates: list[PredicateSpec] = Field(default_factory=list)
    entities: list[EntityDecl] = Field(default_factory=list)
    steps: list[Step] = Field(default_factory=list)
    edits: list[ProgramEdit] = Field(default_factory=list)
    # The program-level WHILE: after the body completes, this predicate is
    # verified; not-yet-satisfied re-arms the body for another pass. "Move all
    # the blocks to the tray" is one body and this loop, not five copies of the
    # body — and the plan survives discovering a sixth block. Bounded by
    # max_repeats and by every existing budget (wall clock, steps, edits), so a
    # loop whose predicate can never satisfy ends by budget, loudly, instead of
    # spinning.
    repeat_until: Optional[PredicateSpec] = None
    repeats_done: int = 0
    max_repeats: int = 8
    aborted: bool = False
    abort_reason: str = ""

    # -- navigation ---------------------------------------------------------
    # A failure raised after the whole program ran carries this instead of a
    # step id: there is no step to blame, the plan simply did not achieve the
    # goal. Edits aimed here append to the end.
    GOAL_STEP_ID: ClassVar[str] = "goal"

    def step(self, step_id: str) -> Step:
        for s in self.steps:
            if s.id == step_id:
                return s
        raise KeyError(f"no step {step_id!r}")

    def next_pending(self) -> Optional[Step]:
        for s in self.steps:
            if s.status in (StepStatus.PENDING, StepStatus.RUNNING):
                return s
        return None

    def done(self) -> bool:
        return self.aborted or self.next_pending() is None

    def rearm(self) -> None:
        """Reset the body for another pass of `repeat_until`.

        Statuses reset; `edits` and the attempt COUNTERS do not — history is a
        record, and the budget guards read totals. perceive steps re-run: each
        pass must re-ground, because the previous pass moved the world.
        """
        self.repeats_done += 1
        for st in self.steps:
            st.status = StepStatus.PENDING

    def entity(self, entity_id: str) -> EntityDecl:
        for e in self.entities:
            if e.id == entity_id:
                return e
        raise KeyError(f"no entity {entity_id!r}")

    def fresh_step_id(self) -> str:
        existing = {s.id for s in self.steps}
        i = len(self.steps) + 1
        while f"s{i}" in existing:
            i += 1
        return f"s{i}"

    # -- edits --------------------------------------------------------------
    def apply_edit(self, edit: ProgramEdit) -> None:
        """Mutate the program according to a typed edit. Raises on malformed edits
        so a bad orchestrator proposal fails loudly instead of corrupting the plan."""
        if edit.type is EditType.ABORT:
            self.aborted = True
            self.abort_reason = edit.rationale
        elif edit.type is EditType.RETRY_STEP:
            if edit.step_id == self.GOAL_STEP_ID:
                raise ValueError("cannot retry the goal — propose steps that achieve it")
            s = self.step(edit.step_id)
            s.status = StepStatus.PENDING
            if edit.new_args:
                s.args = {**s.args, **edit.new_args}
        elif edit.type is EditType.SKIP_STEP:
            self.step(edit.step_id).status = StepStatus.SKIPPED
        elif edit.type is EditType.INSERT_STEPS:
            if not edit.steps:
                raise ValueError("INSERT_STEPS with no steps")
            idx = self._index(edit.step_id)
            self._require_new_ids(edit.steps)
            self.steps[idx:idx] = edit.steps
            # The failed anchor step runs again after the inserted recovery.
            if edit.step_id != self.GOAL_STEP_ID:
                anchor = self.step(edit.step_id)
                if anchor.status is StepStatus.FAILED:
                    anchor.status = StepStatus.PENDING
        elif edit.type is EditType.REPLACE_STEP:
            if not edit.steps:
                raise ValueError("REPLACE_STEP with no steps")
            idx = self._index(edit.step_id)
            self._require_new_ids(edit.steps)
            self.steps[idx : idx + 1] = edit.steps
        elif edit.type is EditType.REPLACE_TAIL:
            idx = self._index(edit.step_id)
            self._require_new_ids(edit.steps, allow_existing_from=idx)
            self.steps[idx:] = edit.steps
        elif edit.type is EditType.REGROUND_ENTITY:
            e = self.entity(edit.entity_id)
            if edit.new_description:
                e.description = edit.new_description
        else:  # pragma: no cover
            raise ValueError(f"unknown edit type {edit.type}")
        self.edits.append(edit)

    def _index(self, step_id: Optional[str]) -> int:
        if step_id is None:
            raise ValueError("edit requires step_id")
        # A goal-level failure names no real step: the program ran to the end
        # and simply did not achieve what it was for. Edits aimed there belong
        # at the end. Without this, a plan that OMITS a step can never be
        # repaired — measured: the planner produced perceive + pick for "put the
        # block on the plate", the goal check failed, and every proposed repair
        # bounced off "no step 'goal'".
        if step_id == self.GOAL_STEP_ID and not any(s.id == step_id for s in self.steps):
            return len(self.steps)
        for i, s in enumerate(self.steps):
            if s.id == step_id:
                return i
        raise KeyError(f"no step {step_id!r}")

    def _require_new_ids(self, new_steps: list[Step], allow_existing_from: int | None = None) -> None:
        keep = self.steps if allow_existing_from is None else self.steps[:allow_existing_from]
        existing = {s.id for s in keep}
        for s in new_steps:
            if s.id in existing:
                raise ValueError(f"edit reuses existing step id {s.id!r}")
            existing.add(s.id)

    # -- rendering ----------------------------------------------------------
    def compact(self) -> str:
        lines = [f"GOAL: {self.goal}"]
        if self.goal_predicates:
            lines.append("SUCCESS IFF: " + "; ".join(str(p) for p in self.goal_predicates))
        lines.append("ENTITIES: " + "; ".join(f"{e.id}={e.description!r} ({e.role})" for e in self.entities))
        lines += ["STEPS:"] + [f"  {s.compact()}" for s in self.steps]
        if self.edits:
            lines.append(f"EDITS SO FAR: {len(self.edits)} (latest: {self.edits[-1].type.value} — {self.edits[-1].rationale})")
        return "\n".join(lines)
