"""Offline orchestrator: grounds against MockRobot ground truth and compiles
template plans for pick-and-place instructions.

Purpose: exercise the ENTIRE agent loop (grounding -> execution -> verification
-> diagnosis -> repair) deterministically in CI, with optional injected
perception lies. It is not meant to be clever — cleverness is Gemini's job.
"""
from __future__ import annotations

import re
from typing import Optional

import numpy as np

from ..program import EditType, ProgramEdit, Step, TaskProgram
from ..robot.mock import MockRobot
from ..types import EntityDecl, FailureReport, Frame, PredicateSpec, Value

KIND_SYNONYMS = {
    "block": {"block", "cube", "brick"},
    "bowl": {"bowl", "dish", "basin"},
    "cup": {"cup", "mug", "glass"},
    "plush": {"plush", "toy", "stuffed"},
}


def project(frame: Frame, p_world: np.ndarray) -> Optional[tuple[int, int]]:
    if frame.intrinsics is None or frame.t_base_cam is None:
        return None
    inv = np.linalg.inv(frame.t_base_cam)
    p_cam = inv @ np.array([p_world[0], p_world[1], p_world[2], 1.0])
    if p_cam[2] <= 1e-6:
        return None
    fx, fy = frame.intrinsics[0, 0], frame.intrinsics[1, 1]
    cx, cy = frame.intrinsics[0, 2], frame.intrinsics[1, 2]
    u = int(round(cx + fx * p_cam[0] / p_cam[2]))
    v = int(round(cy + fy * p_cam[1] / p_cam[2]))
    h, w = frame.rgb.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return None
    return u, v


class ScriptedOrchestrator:
    def __init__(self, mock: MockRobot, lies: dict[str, tuple[float, float, float]] | None = None) -> None:
        self.mock = mock
        self.lies = dict(lies or {})  # description-substring -> fake world xyz
        self.repair_calls: list[FailureReport] = []

    # -- description -> mock object -----------------------------------------
    def _resolve(self, query: str) -> Optional[str]:
        tokens = set(re.findall(r"[a-z]+", query.lower()))
        best_name, best_score = None, 0
        for name, obj in self.mock.objects.items():
            name_tokens = set(name.lower().split("_"))
            score = len(tokens & name_tokens)
            for canon, syns in KIND_SYNONYMS.items():
                if obj.kind == canon and tokens & syns:
                    score += 1
            if score > best_score:
                best_name, best_score = name, score
        return best_name if best_score >= 1 else None

    # -- Grounder ------------------------------------------------------------
    def points(self, frame: Frame, query: str) -> list[tuple[int, int, float]]:
        hit = self.point(frame, query)
        return [hit] if hit else []

    def point(self, frame: Frame, query: str) -> Optional[tuple[int, int, float]]:
        for needle, fake_xyz in self.lies.items():
            if needle in query.lower():
                px = project(frame, np.array(fake_xyz))
                return (*px, 0.85) if px else None
        name = self._resolve(query)
        if name is None:
            return None
        obj = self.mock.objects[name]
        if any(st.holding == name for st in self.mock._arms.values()):
            return None  # a held object is not groundable on the table
        px = project(frame, obj.xyz + np.array([0, 0, obj.size[2] / 2]))
        return (*px, 0.9) if px else None

    def boxes(self, frame: Frame, query: str):
        hit = self.box(frame, query)
        return [hit] if hit else []

    def box(self, frame: Frame, query: str) -> Optional[tuple[int, int, int, int, float]]:
        name = self._resolve(query)
        if name is None:
            return None
        obj = self.mock.objects[name]
        top = obj.xyz + np.array([0, 0, obj.size[2] / 2])
        c0 = project(frame, top + np.array([-obj.size[0] / 2, -obj.size[1] / 2, 0]))
        c1 = project(frame, top + np.array([obj.size[0] / 2, obj.size[1] / 2, 0]))
        if c0 is None or c1 is None:
            return None
        u0, v0 = c0
        u1, v1 = c1
        return min(u0, u1), min(v0, v1), max(u0, u1), max(v0, v1), 0.9

    def vqa(self, frames: list[Frame], question: str) -> tuple[Value, float, str]:
        q = question.lower()
        if "held between" in q or "gripper fingers" in q:
            name = self._resolve(q)
            held = name is not None and any(st.holding == name for st in self.mock._arms.values())
            return (Value.TRUE if held else Value.FALSE), 0.85, "mock proprio-visual check"
        if "inside" in q:
            m = re.search(r"is (.+?) inside (.+?)\?", q)
            if m:
                a, b = self._resolve(m.group(1)), self._resolve(m.group(2))
                if a and b:
                    return (Value.TRUE if self.mock.supporting_container(a) == b else Value.FALSE), 0.85, "mock geometry"
        return Value.UNKNOWN, 0.4, "scripted orchestrator cannot answer this"

    # -- Planner -------------------------------------------------------------
    def plan(self, instruction: str, frames: list[Frame], skills_doc: str, hints: str) -> TaskProgram:
        text = instruction.lower()
        m = re.search(r"(?:put|place|drop) the (.+?) (?:in|into|inside) the (.+)", text)
        rel = "in"
        if m is None:
            m = re.search(r"(?:put|place|stack) the (.+?) (?:on|onto) the (.+)", text)
            rel = "on"
        if m:
            a_desc, b_desc = m.group(1).strip(" ."), m.group(2).strip(" .")
            a_name, b_name = self._resolve(a_desc), self._resolve(b_desc)
            if a_name is None or b_name is None:
                raise ValueError(f"scripted planner cannot ground {a_desc!r}/{b_desc!r} in the mock scene")
            arm = "left" if self.mock.gt_xyz(a_name)[1] >= 0 else "right"
            ent = [EntityDecl(id=a_name, description=f"the {a_desc}", role="object"),
                   EntityDecl(id=b_name, description=f"the {b_desc}", role="container" if rel == "in" else "surface")]
            steps = [
                Step(id="s1", skill="perceive", args={"entities": ["all"]},
                     post=[PredicateSpec(name="visible", args=[a_name]), PredicateSpec(name="visible", args=[b_name])],
                     rationale="ground everything before acting"),
                Step(id="s2", skill="pick", args={"entity": a_name, "arm": arm},
                     pre=[PredicateSpec(name="visible", args=[a_name]), PredicateSpec(name="gripper_empty", args=[arm])],
                     post=[PredicateSpec(name="holding", args=[a_name, arm])],
                     rationale=f"grasp with the {arm} arm ({'y>=0' if arm == 'left' else 'y<0'} half)"),
                Step(id="s3", skill="place", args={"entity": a_name, "target": b_name, "arm": arm},
                     pre=[PredicateSpec(name="holding", args=[a_name, arm])],
                     post=[PredicateSpec(name=rel, args=[a_name, b_name]),
                           PredicateSpec(name="holding", args=[a_name, arm], negated=True)],
                     rationale="release above the destination"),
            ]
            return TaskProgram(goal=instruction, goal_predicates=[PredicateSpec(name=rel, args=[a_name, b_name])],
                               entities=ent, steps=steps)
        m = re.search(r"pick up the (.+)", text)
        if m:
            a_desc = m.group(1).strip(" .")
            a_name = self._resolve(a_desc)
            if a_name is None:
                raise ValueError(f"scripted planner cannot ground {a_desc!r}")
            arm = "left" if self.mock.gt_xyz(a_name)[1] >= 0 else "right"
            ent = [EntityDecl(id=a_name, description=f"the {a_desc}")]
            steps = [
                Step(id="s1", skill="perceive", args={"entities": ["all"]},
                     post=[PredicateSpec(name="visible", args=[a_name])]),
                Step(id="s2", skill="pick", args={"entity": a_name, "arm": arm},
                     pre=[PredicateSpec(name="visible", args=[a_name])],
                     post=[PredicateSpec(name="holding", args=[a_name, arm])]),
            ]
            return TaskProgram(goal=instruction, goal_predicates=[PredicateSpec(name="holding", args=[a_name, arm])],
                               entities=ent, steps=steps)
        raise ValueError(f"scripted planner does not understand {instruction!r}")

    # -- Repairer ------------------------------------------------------------
    def propose_repair(self, program: TaskProgram, belief_summary: str, failure: FailureReport,
                       skills_doc: str, frames: list[Frame]) -> ProgramEdit:
        self.repair_calls.append(failure)
        step = program.step(failure.step_id) if failure.step_id != "goal" else None
        # Structural repair: a place step lost its held object -> re-acquire it
        # (perceive + pick inserted before the place, which then runs again).
        lost_hold = next((p for p in failure.failed_predicates if p.name == "holding" and not p.negated), None)
        if step is not None and step.skill == "place" and lost_hold is not None:
            entity = lost_hold.args[0]
            arm = lost_hold.args[1] if len(lost_hold.args) > 1 else "auto"
            n = len(self.repair_calls)
            steps = [
                Step(id=f"rp{n}a", skill="perceive", args={"entities": [entity]},
                     post=[PredicateSpec(name="visible", args=[entity])],
                     rationale="find where the lost object went"),
                Step(id=f"rp{n}b", skill="pick", args={"entity": entity, "arm": arm},
                     pre=[PredicateSpec(name="visible", args=[entity])],
                     post=[PredicateSpec(name="holding", args=[entity, arm])],
                     rationale="re-acquire before placing"),
            ]
            return ProgramEdit(type=EditType.INSERT_STEPS, step_id=step.id, steps=steps,
                               rationale="scripted: object no longer held — re-perceive and re-pick it",
                               source="orchestrator")
        if step is not None and step.attempts <= step.max_attempts + 1:
            return ProgramEdit(type=EditType.RETRY_STEP, step_id=step.id,
                               rationale="scripted: one extra retry", source="orchestrator")
        return ProgramEdit(type=EditType.ABORT,
                           rationale=f"scripted: giving up after {failure.summary}", source="orchestrator")
