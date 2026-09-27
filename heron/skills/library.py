"""Learned skills: parameterized macros the agent writes, keeps, and reuses.

The registry below this file holds PRIMITIVES — the fixed vocabulary the
platform guarantees (pick, place, ground, descend_to_contact, ...). They are
hand-written because they touch the hardware and must be safety-checked.

What they cannot express is experience. "Grasp this kind of block by descending
to contact first, then closing with reduced effort, then lifting 6 cm before
translating" is knowledge earned in one episode that should be free in the next.
A learned skill is exactly that: a NAMED, PARAMETERIZED sequence of primitive
steps, with the pre/postconditions that made it work.

Two decisions worth stating, because they are what keep this safe:

  * A skill is DATA, not code. It expands into ordinary typed Steps over
    registered primitives, so nothing new can reach the hardware — a learned
    skill can only re-order and re-parameterize what the platform already
    allows. No generated Python is executed.

  * A skill expands at PLAN time rather than executing as one opaque call. The
    executor therefore still verifies every primitive step, and a failure is
    still diagnosed and repaired at primitive granularity. A black-box skill
    would collapse exactly the structure that makes recovery possible; here,
    reuse and introspection coexist.

Skills accumulate statistics (uses, successes) and provenance (which episodes
produced and repaired them), so a skill that stops working can be demoted on
evidence instead of intuition.
"""
from __future__ import annotations

import json
import re
import time
from pathlib import Path
from typing import Any, Optional

from pydantic import BaseModel, Field

from ..program import Step
from ..types import PredicateSpec

PARAM_RE = re.compile(r"^\$([A-Za-z_][A-Za-z0-9_]*)$")
NAME_RE = re.compile(r"^[a-z][a-z0-9_]{2,39}$")


class SkillStep(BaseModel):
    """One primitive call inside a skill body. `$name` strings take parameters."""

    skill: str
    args: dict[str, Any] = Field(default_factory=dict)
    pre: list[PredicateSpec] = Field(default_factory=list)
    post: list[PredicateSpec] = Field(default_factory=list)
    rationale: str = ""


class SkillVersion(BaseModel):
    """An archived body, kept with the score it earned so a revision can be undone."""

    version: int
    body: list[SkillStep]
    uses: int
    successes: int
    reason: str = ""
    retired_at: float = Field(default_factory=time.time)

    @property
    def score(self) -> float:
        # Laplace smoothing: an unproven version must not outrank a proven one
        # on the strength of a single lucky run.
        return (self.successes + 1) / (self.uses + 2)


class LearnedSkill(BaseModel):
    name: str
    description: str  # what it does
    when_to_use: str = ""  # the retrieval cue the planner reads
    params: list[str] = Field(default_factory=list)
    body: list[SkillStep]
    uses: int = 0
    successes: int = 0
    version: int = 1
    history: list[SkillVersion] = Field(default_factory=list)
    deprecated: bool = False
    created_at: float = Field(default_factory=time.time)
    updated_at: float = Field(default_factory=time.time)
    provenance: list[str] = Field(default_factory=list)  # episode ids

    @property
    def success_rate(self) -> Optional[float]:
        return None if self.uses == 0 else self.successes / self.uses

    @property
    def score(self) -> float:
        return (self.successes + 1) / (self.uses + 2)


def _substitute(value: Any, args: dict[str, Any]) -> Any:
    """Replace `$param` leaves with argument values, at any nesting depth."""
    if isinstance(value, str):
        m = PARAM_RE.match(value)
        if m:
            if m.group(1) not in args:
                raise KeyError(m.group(1))
            return args[m.group(1)]
        return value
    if isinstance(value, list):
        return [_substitute(v, args) for v in value]
    if isinstance(value, dict):
        return {k: _substitute(v, args) for k, v in value.items()}
    return value


def _substitute_predicate(spec: PredicateSpec, args: dict[str, Any]) -> PredicateSpec:
    return spec.model_copy(update={"args": [_substitute(a, args) for a in spec.args]})


class SkillLibrary:
    """Learned skills on disk, one JSON file each."""

    def __init__(self, directory: str | Path) -> None:
        self.dir = Path(directory)
        self._skills: dict[str, LearnedSkill] = {}
        if self.dir.exists():
            for f in sorted(self.dir.glob("*.json")):
                try:
                    self._skills[f.stem] = LearnedSkill(**json.loads(f.read_text()))
                except Exception:
                    continue  # a corrupt entry must not take the library down

    def __len__(self) -> int:
        return len(self._skills)

    def names(self) -> list[str]:
        return sorted(self._skills)

    def get(self, name: str) -> Optional[LearnedSkill]:
        return self._skills.get(name)

    def add(self, skill: LearnedSkill, primitives: list[str]) -> LearnedSkill:
        """Validate against the primitive vocabulary, then persist."""
        self.validate(skill, primitives)
        skill.updated_at = time.time()
        self._skills[skill.name] = skill
        self._write(skill)
        return skill

    def remove(self, name: str) -> bool:
        if name not in self._skills:
            return False
        del self._skills[name]
        path = self.dir / f"{name}.json"
        if path.exists():
            path.unlink()
        return True

    @staticmethod
    def validate(skill: LearnedSkill, primitives: list[str]) -> None:
        """Reject anything that could not expand into a runnable program."""
        if not NAME_RE.match(skill.name):
            raise ValueError(f"invalid skill name {skill.name!r}: use lower_snake_case, 3-40 chars")
        if skill.name in primitives:
            raise ValueError(f"{skill.name!r} would shadow a platform primitive")
        if not skill.body:
            raise ValueError("a skill needs at least one step")
        declared = set(skill.params)
        used: set[str] = set()

        def collect(value: Any) -> None:
            if isinstance(value, str):
                m = PARAM_RE.match(value)
                if m:
                    used.add(m.group(1))
            elif isinstance(value, list):
                for v in value:
                    collect(v)
            elif isinstance(value, dict):
                for v in value.values():
                    collect(v)

        for i, step in enumerate(skill.body):
            if step.skill not in primitives:
                raise ValueError(
                    f"step {i} calls {step.skill!r}, which is not a platform primitive "
                    f"(available: {sorted(primitives)}). Learned skills may only compose "
                    "primitives — that is what keeps them safe to run."
                )
            collect(step.args)
            for spec in list(step.pre) + list(step.post):
                collect(spec.args)
        unknown = used - declared
        if unknown:
            raise ValueError(f"body uses undeclared parameters {sorted(unknown)}")
        unused = declared - used
        if unused:
            raise ValueError(f"declared parameters {sorted(unused)} are never used")

    def expand(self, name: str, args: dict[str, Any], step_id: str) -> list[Step]:
        """Turn one skill invocation into the typed steps the executor runs."""
        skill = self._skills.get(name)
        if skill is None:
            raise KeyError(f"unknown learned skill {name!r}")
        missing = [p for p in skill.params if p not in args]
        if missing:
            raise ValueError(f"{name} missing arguments {missing}")
        steps: list[Step] = []
        for i, body_step in enumerate(skill.body):
            try:
                resolved_args = _substitute(body_step.args, args)
            except KeyError as e:
                raise ValueError(f"{name} step {i} references unknown parameter ${e.args[0]}") from e
            steps.append(Step(
                # Derived from the invocation id so ids stay unique even when the
                # same skill appears twice in one program.
                id=f"{step_id}_{name[:8]}{i}",
                origin_skill=name,
                origin_args=dict(args),
                skill=body_step.skill,
                args=resolved_args,
                pre=[_substitute_predicate(p, args) for p in body_step.pre],
                post=[_substitute_predicate(p, args) for p in body_step.post],
                rationale=body_step.rationale or f"{name}: step {i + 1}/{len(skill.body)}",
            ))
        return steps

    def record_outcome(self, name: str, ok: bool, episode: str = "") -> None:
        skill = self._skills.get(name)
        if skill is None:
            return
        skill.uses += 1
        skill.successes += int(bool(ok))
        skill.updated_at = time.time()
        if episode and episode not in skill.provenance:
            skill.provenance.append(episode)
        self._write(skill)

    # -- self-iteration ------------------------------------------------------
    @staticmethod
    def generalize(steps: list[Step], args: dict[str, Any]) -> list[SkillStep]:
        """Turn concrete executed steps back into a parameterized body.

        This is what separates a reusable skill from a recording. The steps that
        actually ran mention THIS episode's entities ("red_block", "right"); any
        value that came from an invocation argument is put back as `$param`, so
        the lesson transfers to the next object instead of being welded to this
        one. Values that were not arguments stay literal — they are part of the
        technique (a 6 cm lift, a reduced grip effort), not of the situation.
        """
        by_value = {}
        for param, value in args.items():
            if isinstance(value, (str, int, float, bool)):
                by_value.setdefault(value, param)

        def degeneralize(value: Any) -> Any:
            if isinstance(value, (str, int, float, bool)) and value in by_value:
                return f"${by_value[value]}"
            if isinstance(value, list):
                return [degeneralize(v) for v in value]
            if isinstance(value, dict):
                return {k: degeneralize(v) for k, v in value.items()}
            return value

        body = []
        for s in steps:
            body.append(SkillStep(
                skill=s.skill,
                args=degeneralize(s.args),
                pre=[p.model_copy(update={"args": [degeneralize(a) for a in p.args]}) for p in s.pre],
                post=[p.model_copy(update={"args": [degeneralize(a) for a in p.args]}) for p in s.post],
                rationale=s.rationale,
            ))
        return body

    def revise(self, name: str, new_body: list[SkillStep], reason: str,
               primitives: list[str], episode: str = "") -> LearnedSkill:
        """Adopt a repaired step sequence as the skill's next version.

        When an expansion only succeeded after the agent repaired it, those
        repairs ARE the skill's missing knowledge. The previous body is archived
        with the score it earned, so a revision that turns out worse can be
        rolled back on evidence rather than argued about.
        """
        skill = self._skills.get(name)
        if skill is None:
            raise KeyError(f"unknown learned skill {name!r}")
        if [s.model_dump() for s in new_body] == [s.model_dump() for s in skill.body]:
            return skill  # nothing changed; do not churn the version counter
        candidate = skill.model_copy(deep=True)
        candidate.body = new_body
        self.validate(candidate, primitives)

        skill.history.append(SkillVersion(version=skill.version, body=skill.body,
                                          uses=skill.uses, successes=skill.successes,
                                          reason=reason))
        skill.body = new_body
        skill.version += 1
        skill.uses = 0          # the new version has its own record to earn
        skill.successes = 0
        skill.updated_at = time.time()
        if episode and episode not in skill.provenance:
            skill.provenance.append(episode)
        self._write(skill)
        return skill

    def rollback_if_worse(self, name: str, min_uses: int = 3) -> Optional[int]:
        """Return to the best-scoring archived version if this one is worse.

        Only judges once the current version has had `min_uses` attempts —
        otherwise a revision is condemned before it has had a chance to work.
        """
        skill = self._skills.get(name)
        if skill is None or not skill.history or skill.uses < min_uses:
            return None
        best = max(skill.history, key=lambda v: v.score)
        if best.score <= skill.score:
            return None
        skill.history.append(SkillVersion(version=skill.version, body=skill.body,
                                          uses=skill.uses, successes=skill.successes,
                                          reason=f"rolled back in favour of v{best.version}"))
        skill.history.remove(best)
        skill.body = best.body
        skill.version += 1
        skill.uses, skill.successes = best.uses, best.successes
        skill.updated_at = time.time()
        self._write(skill)
        return best.version

    def retire_if_failing(self, name: str, min_uses: int = 4,
                          min_rate: float = 0.34) -> bool:
        """Hide a skill that keeps failing — with no better version to fall back on."""
        skill = self._skills.get(name)
        if skill is None or skill.deprecated or skill.uses < min_uses:
            return False
        rate = skill.success_rate
        if rate is None or rate >= min_rate:
            return False
        skill.deprecated = True
        skill.updated_at = time.time()
        self._write(skill)
        return True

    def consolidate(self, name: str) -> str:
        """Post-episode upkeep: prefer the best version, retire the hopeless."""
        restored = self.rollback_if_worse(name)
        if restored is not None:
            return f"rolled back to v{restored}"
        if self.retire_if_failing(name):
            return "deprecated (kept failing)"
        return "kept"

    def _write(self, skill: LearnedSkill) -> None:
        self.dir.mkdir(parents=True, exist_ok=True)
        (self.dir / f"{skill.name}.json").write_text(skill.model_dump_json(indent=1))

    def catalog(self, min_success_rate: float = 0.0) -> str:
        """Planner-facing documentation, mirroring the primitive catalog's shape."""
        lines = []
        for name in self.names():
            s = self._skills[name]
            if s.deprecated:
                continue
            rate = s.success_rate
            if rate is not None and rate < min_success_rate:
                continue
            params = json.dumps({p: {"type": "string"} for p in s.params}, separators=(",", ":"))
            record = ("unused so far" if s.uses == 0
                      else f"{s.successes}/{s.uses} successful")
            lines.append(
                f"- {s.name}: {s.description}\n"
                f"    params={params} required={s.params}\n"
                f"    use when: {s.when_to_use or 'see description'}\n"
                f"    expands to {len(s.body)} primitive steps; {record}"
            )
        return "\n".join(lines)
