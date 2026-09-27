"""Corrections as typed, scoped, withdrawable objects — not source-code commits.

Every fix this project has actually needed lived at one of five levels, and the
history is unambiguous about it: the existential `in()` question was a change to
a PREDICATE, the 3 mm stack-release gap was a SKILL parameter for one class of
support, the rim grasp was a SKILL rule for one class of object, program recall
reuses a whole PROGRAM, and the retired place_offset was a skill parameter whose
scope was wrong — keyed by visual description, it crossed between objects and a
gripper closed on air beside a correctly grounded cube.

All of those became code, because the system had nothing to hold them as data.
This module is that container:

    Patch = (level, delta, scope, provenance, validation)

`level` says WHICH model was wrong (grounding / predicate / program / skill /
policy). `delta` says what changes — for parameter patches a plain mapping, for
program patches a rule the compiler consults. `scope` says WHEN it applies, and
is deliberately explicit: the place_offset failure was not a bad correction, it
was a correct correction with an unstated scope, silently generalised by a
string match. A patch whose scope cannot be stated does not go in the store.

Application is a LOOKUP, not a mutation: callers ask `param(...)` at the moment
they need a value, so withdrawing a patch is deleting a row, and what ran with
which patch is reconstructable from the journal.
"""
from __future__ import annotations

import json
import time
import uuid
from enum import Enum
from pathlib import Path
from typing import Any, Optional

from pydantic import BaseModel, Field


class PatchLevel(str, Enum):
    GROUNDING = "grounding"   # which object a symbol picks out; episode-scoped
    PREDICATE = "predicate"   # what a verifier checks, or its thresholds
    PROGRAM = "program"       # task logic: steps, ordering, guards
    SKILL = "skill"           # how a primitive executes: parameters, phases
    POLICY = "policy"         # learned motor behaviour; points at data/checkpoints


class Scope(BaseModel):
    """When a patch applies. Every field is a conjunct; absent means 'any'.

    Explicit on purpose — the one scope mechanism this project has tried before
    was an implicit string match on visual descriptions, and its failure is the
    reason this class exists. `entity_class` matches against an entity's
    description by whole word ("bowl" matches "the blue bowl", not "bowlder");
    `entity_id` pins a single instance and therefore a single episode's
    binding; `task_contains` scopes to instructions mentioning a phrase.
    """

    entity_id: Optional[str] = None
    entity_class: Optional[str] = None
    skill: Optional[str] = None
    predicate: Optional[str] = None
    task_contains: Optional[str] = None
    # Episode-scoped patches die with the episode that made them. This is the
    # narrowest scope and the default for anything applied live from a
    # correction that has not been validated yet.
    episode: Optional[str] = None

    def matches(self, ctx: dict[str, Any]) -> bool:
        if self.entity_id and ctx.get("entity_id") != self.entity_id:
            return False
        if self.entity_class:
            desc = str(ctx.get("entity_description", "") or "").lower()
            if self.entity_class.lower() not in desc.split() and \
               self.entity_class.lower() not in desc:
                return False
        if self.skill and ctx.get("skill") != self.skill:
            return False
        if self.predicate and ctx.get("predicate") != self.predicate:
            return False
        if self.task_contains:
            if self.task_contains.lower() not in str(ctx.get("task", "")).lower():
                return False
        if self.episode and ctx.get("episode") != self.episode:
            return False
        return True

    def breadth(self) -> int:
        """How many conjuncts pin this down — more is narrower. Used to prefer
        the most specific applicable patch, never to guess a missing scope."""
        return sum(v is not None for v in
                   (self.entity_id, self.entity_class, self.skill,
                    self.predicate, self.task_contains, self.episode))


class Validation(BaseModel):
    """What happened when the patch was tried on states it was not written on.

    A patch with no validation record may still be applied — a live correction
    has to take effect now — but it must not OUTLIVE the episode until it has
    one. That is the promotion rule, and it is enforced by the store, not by
    good intentions.
    """

    trials: int = 0
    fixed: int = 0          # previously-failing cases that now pass
    broken: int = 0         # previously-passing cases that now fail
    note: str = ""
    t: Optional[float] = None

    @property
    def net(self) -> int:
        return self.fixed - self.broken


class Patch(BaseModel):
    id: str = Field(default_factory=lambda: uuid.uuid4().hex[:10])
    level: PatchLevel
    # For parameter patches: {"param_name": value, ...}. For program patches:
    # {"rule": <text the compiler injects>, ...} or {"insert_before": skill,
    # "step": {...}}. The delta's meaning is fixed by (level, target).
    target: str            # predicate name, skill name, or "compile" for program
    delta: dict[str, Any] = Field(default_factory=dict)
    scope: Scope = Field(default_factory=Scope)
    # Where this came from: "operator:binding_panel", "repair:in_alt",
    # "distilled:episode-...". Provenance is what lets a later reader decide
    # how much to trust a patch the validator has not seen many trials on.
    provenance: str = ""
    rationale: str = ""
    validation: Optional[Validation] = None
    withdrawn: bool = False
    withdrawn_reason: str = ""
    t: float = Field(default_factory=time.time)

    def live(self) -> bool:
        return not self.withdrawn


class PatchStore:
    """All patches, one JSONL file, loaded whole. Small on purpose: a store
    with ten thousand rows is a store nobody audited."""

    def __init__(self, path: str | Path | None) -> None:
        self.path = Path(path) if path else None
        self.patches: list[Patch] = []
        if self.path and self.path.exists():
            for line in self.path.read_text().splitlines():
                if line.strip():
                    try:
                        self.patches.append(Patch.model_validate_json(line))
                    except Exception:
                        pass

    # -- write ---------------------------------------------------------------
    def add(self, patch: Patch) -> Patch:
        self.patches.append(patch)
        self._flush()
        return patch

    def withdraw(self, patch_id: str, reason: str) -> None:
        for p in self.patches:
            if p.id == patch_id:
                p.withdrawn = True
                p.withdrawn_reason = reason
        self._flush()

    def end_episode(self, episode: str) -> int:
        """Retire unvalidated episode-scoped patches. The promotion rule:
        a correction outlives its episode only once validation says it should."""
        n = 0
        for p in self.patches:
            if (p.live() and p.scope.episode == episode
                    and (p.validation is None or p.validation.trials == 0)):
                p.withdrawn = True
                p.withdrawn_reason = "episode ended without validation"
                n += 1
        if n:
            self._flush()
        return n

    def _flush(self) -> None:
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(
            "\n".join(p.model_dump_json() for p in self.patches) + "\n"
            if self.patches else "")

    # -- read ----------------------------------------------------------------
    def applicable(self, level: PatchLevel, target: str,
                   ctx: dict[str, Any]) -> list[Patch]:
        hits = [p for p in self.patches
                if p.live() and p.level == level and p.target == target
                and p.scope.matches(ctx)]
        # Most specific last, so a caller folding deltas in order lets the
        # narrow patch override the broad one — never the other way round.
        return sorted(hits, key=lambda p: p.scope.breadth())

    def param(self, level: PatchLevel, target: str, name: str,
              default: Any, ctx: dict[str, Any] | None = None) -> Any:
        """The one call sites use: a value, patched or not.

        This is what makes application a lookup. The call site keeps its
        default in the code — the behaviour with an empty store is exactly the
        behaviour before this module existed.
        """
        value = default
        for p in self.applicable(level, target, ctx or {}):
            if name in p.delta:
                value = p.delta[name]
        return value

    def rules(self, ctx: dict[str, Any]) -> list[Patch]:
        """Program-level rules the compiler should honour for this task."""
        return self.applicable(PatchLevel.PROGRAM, "compile", ctx)
