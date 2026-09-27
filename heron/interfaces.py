"""Model-facing interfaces. GeminiOrchestrator implements all three against
ER 2; ScriptedOrchestrator implements them offline for tests and demos.

Keeping these narrow is what makes the orchestrator swappable (Pigey lesson:
its sim harness supported five providers by translating chat formats; Heron
instead fixes the *decision types* and lets any model implement them).
"""
from __future__ import annotations

from typing import Optional, Protocol, runtime_checkable

from .program import ProgramEdit, TaskProgram
from .types import FailureReport, Frame, Value


@runtime_checkable
class Grounder(Protocol):
    """Turns language into pixels (and judgments) on specific frames."""

    def point(self, frame: Frame, query: str) -> Optional[tuple[int, int, float]]:
        """(u, v, confidence) for the best match of `query` in `frame`, else None."""
        ...

    def box(self, frame: Frame, query: str) -> Optional[tuple[int, int, int, int, float]]:
        """(u0, v0, u1, v1, confidence) bounding box, else None."""
        ...

    def vqa(self, frames: list[Frame], question: str) -> tuple[Value, float, str]:
        """Three-valued visual question answering with a short justification."""
        ...


@runtime_checkable
class Planner(Protocol):
    def plan(self, instruction: str, frames: list[Frame], skills_doc: str, hints: str) -> TaskProgram: ...


@runtime_checkable
class Repairer(Protocol):
    def propose_repair(
        self,
        program: TaskProgram,
        belief_summary: str,
        failure: FailureReport,
        skills_doc: str,
        frames: list[Frame],
    ) -> ProgramEdit: ...


class Orchestrator(Grounder, Planner, Repairer, Protocol):
    """The full brain interface: ground + plan + repair."""
