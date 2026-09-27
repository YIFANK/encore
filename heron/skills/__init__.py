"""Skill registry: parameterized primitives + sensing + VLA rollouts.

Each skill is a plain function taking (SkillContext, **params) -> SkillResult.
The registry renders the catalog into the planner prompt, so adding a skill
here is all it takes for the orchestrator to start using it.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Optional

from ..types import SkillResult

if TYPE_CHECKING:  # avoid import cycles; runtime access is duck-typed
    from ..belief import BeliefStore
    from ..config import HeronConfig
    from ..episode import EpisodeLogger
    from ..interfaces import Grounder
    from ..memory import MemoryStore
    from ..perception import SamSegmenter
    from ..robot.safety import SafeRobot


@dataclass
class SkillContext:
    robot: "SafeRobot"
    belief: "BeliefStore"
    grounder: "Grounder"
    cfg: "HeronConfig"
    log: "EpisodeLogger"
    memory: Optional["MemoryStore"] = None
    # Mask service. None (or a segmenter with no URL) means grounding falls back
    # to detection boxes.
    segmenter: Optional["SamSegmenter"] = None
    # Where the last place aimed, so a miss can be measured rather than guessed.
    last_place: dict[str, Any] = field(default_factory=dict)
    # Re-entrancy guards and other per-episode bookkeeping.
    scratch: dict[str, Any] = field(default_factory=dict)
    # The running skill's stopwatch, for anything watching live (heron/console).
    # None between skills. Written by _Phases; read by the console.
    live_phase: Optional[dict[str, Any]] = None
    # Typed, scoped corrections (heron/patches.py). None degrades every lookup
    # to its in-code default, so nothing below this line requires a store.
    patches: Optional["PatchStore"] = None


@dataclass
class SkillSpec:
    name: str
    description: str
    params: dict[str, Any]  # JSON-schema properties
    required: list[str]
    post_hint: str  # tells the planner which postconditions this skill normally earns
    fn: Callable[..., SkillResult] = field(repr=False, default=None)  # type: ignore[assignment]


class SkillRegistry:
    def __init__(self) -> None:
        self._skills: dict[str, SkillSpec] = {}

    def register(self, spec: SkillSpec) -> None:
        self._skills[spec.name] = spec

    def get(self, name: str) -> SkillSpec:
        if name not in self._skills:
            raise KeyError(f"unknown skill {name!r}; available: {sorted(self._skills)}")
        return self._skills[name]

    def names(self) -> list[str]:
        return sorted(self._skills)

    def execute(self, ctx: SkillContext, name: str, args: dict[str, Any],
                supplied: dict[str, Any] | None = None) -> SkillResult:
        """Run a skill. `args` came from the planner; `supplied` did not.

        The two are kept apart on purpose. `args` is validated against the
        schema the model was shown, so a hallucinated argument is refused rather
        than silently accepted. `supplied` is what the EXECUTOR hands down — a
        position a look already produced — and it is deliberately absent from
        that schema: the planner names objects, it does not get to write
        coordinates. Merging the two would either expose `at` to the model or
        reject the executor's own value as unknown.
        """
        if name in getattr(ctx.cfg, "disable_skills", []):
            return SkillResult(ok=False, error=f"skill {name!r} is disabled in this deployment")
        spec = self.get(name)
        missing = [r for r in spec.required if r not in args]
        if missing:
            return SkillResult(ok=False, error=f"{name} missing required args {missing}")
        unknown = [k for k in args if k not in spec.params]
        if unknown:
            return SkillResult(ok=False, error=f"{name} got unknown args {unknown}; allowed: {sorted(spec.params)}")
        args = {**args, **(supplied or {})}
        try:
            return spec.fn(ctx, **args)
        except Exception as e:  # SafetyViolation etc. surface as failed results
            return SkillResult(ok=False, error=f"{type(e).__name__}: {e}")

    def catalog(self, exclude: list[str] | None = None) -> str:
        """Human/model-readable skill documentation for the planner prompt."""
        lines = []
        for spec in sorted(self._skills.values(), key=lambda s: s.name):
            if exclude and spec.name in exclude:
                continue
            params = json.dumps(spec.params, separators=(",", ":"))
            lines.append(
                f"- {spec.name}: {spec.description}\n"
                f"    params={params} required={spec.required}\n"
                f"    typical postconditions: {spec.post_hint}"
            )
        return "\n".join(lines)


registry = SkillRegistry()


def skill(name: str, description: str, params: dict[str, Any], required: list[str], post_hint: str):
    def wrap(fn: Callable[..., SkillResult]) -> Callable[..., SkillResult]:
        registry.register(SkillSpec(name=name, description=description, params=params,
                                    required=required, post_hint=post_hint, fn=fn))
        return fn

    return wrap


def load_all() -> SkillRegistry:
    """Import all skill modules so their decorators run, and return the registry."""
    from . import articulate, primitives, search, sensing, tamp, vla  # noqa: F401

    return registry
