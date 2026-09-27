"""Perception as part of the plan, and deliberately NOT part of verification.

The planner still writes a list of typed steps. It does not need to change: the
dependencies are already in the arguments — `perceive` says what it grounds,
`pick` and `place` name what they act on. So a program written as a list can be
read as a graph, and the three things the graph form buys can be had without
touching the planner:

  * what a plan will cost in detector round trips, before anything moves;
  * whether it acts on something nobody looked for;
  * when a position stops being true — because something moved it, not because
    a clock ran out.

The last one is the behaviour change. `_resolve_point` refused a position older
than `verify_max_age_s`, and the clock cannot know whether anything moved:
measured twice in the twin, an episode failed on "entity 'grey_tray' location is
stale (106s)" for a tray that had not moved all episode.

VERIFICATION IS EXCLUDED ON PURPOSE. A check handed the number that decided the
action would be confirming the action against its own premise — the exact shape
of every false success measured today.
"""
from __future__ import annotations

import numpy as np
import pytest

from heron.program import Step, TaskProgram
from heron.skills import dataflow
from heron.types import EntityDecl, PredicateSpec


def _prog(steps, entities=("block", "bowl")):
    return TaskProgram(
        goal="put the block in the bowl",
        entities=[EntityDecl(id=e, description=f"the {e}") for e in entities],
        steps=steps,
        goal_predicates=[PredicateSpec(name="in", args=["block", "bowl"])])


def test_the_cost_of_a_plan_is_known_before_it_runs():
    p = _prog([Step(id="s1", skill="perceive", args={"entities": ["block", "bowl"]}),
               Step(id="s2", skill="pick", args={"entity": "block"}),
               Step(id="s3", skill="place", args={"entity": "block", "target": "bowl"})])
    assert dataflow.program_cost(p) == 1, "one look, however many things it asks about"


def test_a_second_look_is_a_second_call():
    p = _prog([Step(id="s1", skill="perceive", args={"entities": ["block"]}),
               Step(id="s2", skill="pick", args={"entity": "block"}),
               Step(id="s3", skill="perceive", args={"entities": ["bowl"]}),
               Step(id="s4", skill="place", args={"entity": "block", "target": "bowl"})])
    assert dataflow.program_cost(p) == 2


def test_a_plan_that_acts_on_what_it_never_looked_for_is_named():
    p = _prog([Step(id="s1", skill="perceive", args={"entities": ["block"]}),
               Step(id="s2", skill="place", args={"entity": "block", "target": "bowl"})])
    assert dataflow.unresolved(p) == [("s2", "bowl")]


def test_perceiving_everything_resolves_everything():
    """`perceive` with no list grounds the whole scene, so nothing after it is
    unresolved — otherwise the commonest plan in the corpus would be rejected."""
    p = _prog([Step(id="s1", skill="perceive", args={"entities": ["all"]}),
               Step(id="s2", skill="pick", args={"entity": "block"}),
               Step(id="s3", skill="place", args={"entity": "block", "target": "bowl"})])
    assert dataflow.unresolved(p) == []


def test_a_step_that_moves_something_retires_its_position():
    pick = Step(id="s2", skill="pick", args={"entity": "block"})
    place = Step(id="s3", skill="place", args={"entity": "block", "target": "bowl"})
    assert dataflow.moves(pick) == ["block"]
    assert dataflow.moves(place) == ["block"], "the object moved; the container did not"
    assert "bowl" not in dataflow.moves(place)


def test_a_look_produces_and_an_action_consumes():
    assert dataflow.produces(Step(id="s", skill="perceive",
                                  args={"entities": ["block", "bowl"]})) == ["block", "bowl"]
    assert dataflow.consumes(Step(id="s", skill="place",
                                  args={"entity": "block", "target": "bowl"})) == ["block", "bowl"]
    assert dataflow.produces(Step(id="s", skill="pick", args={"entity": "block"})) == []


# -- the split ---------------------------------------------------------------

def test_no_verifier_is_fed_a_planned_value():
    """The whole point of a verifier is to disagree with the plan. Feeding it the
    number that decided the action removes the only independence in the stack.

    Pinned structurally: the skills that receive values are exactly the ones that
    MOVE the arm, and no predicate name appears among them.
    """
    predicates = {"holding", "gripper_empty", "visible", "in", "on", "open"}
    assert not (set(dataflow.CONSUMES) & predicates)
    assert set(dataflow.CONSUMES) <= {"pick", "place", "push",
                                      "open_drawer", "close_drawer"}


def test_the_agent_hands_values_to_actions_only(tmp_path):
    from heron.agent import Agent
    from heron.config import HeronConfig
    from heron.robot.mock import MockRobot, standard_scene
    from heron.robot.safety import SafeRobot
    from heron.skills.values import Located
    import sys
    sys.path.insert(0, "tests")
    from test_agent_loop import ScriptedOrchestrator

    cfg = HeronConfig()
    cfg.episodes_dir = str(tmp_path / "ep")
    cfg.memory_dir = str(tmp_path / "mem")
    cfg.skills_dir = str(tmp_path / "sk")
    mock = MockRobot()
    standard_scene(mock)
    agent = Agent(SafeRobot(mock, cfg.safety), ScriptedOrchestrator(mock), cfg, name="df")
    agent._values["red_block"] = Located(xyz=np.array([0.05, 0.10, 0.02]),
                                         source="look", conf=0.9)

    picked = agent._value_for(Step(id="s", skill="pick", args={"entity": "red_block"}))
    assert "at" in picked, "an action should be handed the position"

    looked = agent._value_for(Step(id="s", skill="perceive",
                                     args={"entities": ["red_block"]}))
    assert "at" not in looked, "a look must go and look"


def test_a_moved_object_is_not_handed_its_old_position(tmp_path):
    from heron.agent import Agent
    from heron.config import HeronConfig
    from heron.robot.mock import MockRobot, standard_scene
    from heron.robot.safety import SafeRobot
    from heron.skills.values import Located
    import sys
    sys.path.insert(0, "tests")
    from test_agent_loop import ScriptedOrchestrator

    cfg = HeronConfig()
    cfg.episodes_dir = str(tmp_path / "ep")
    cfg.memory_dir = str(tmp_path / "mem")
    cfg.skills_dir = str(tmp_path / "sk")
    mock = MockRobot()
    standard_scene(mock)
    agent = Agent(SafeRobot(mock, cfg.safety), ScriptedOrchestrator(mock), cfg, name="df")
    agent._values["red_block"] = Located(xyz=np.array([0.05, 0.10, 0.02]),
                                         source="look", conf=0.9)
    agent._retire_values(Step(id="s2", skill="pick", args={"entity": "red_block"}))
    assert "red_block" not in agent._values
    after = agent._value_for(Step(id="s3", skill="place",
                                    args={"entity": "red_block", "target": "blue_bowl"}))
    assert "at" not in after, "a stale value must not survive the move that invalidated it"


# -- a plan is structure, so it can be reused -------------------------------

def test_a_remembered_program_is_reset_not_resumed(tmp_path):
    """A program holds entities and an order, and no coordinates — positions are
    ground afresh every episode, which is exactly why the same instruction in a
    rearranged scene wants the same program.

    But the one that earned its place finished: its steps are DONE and its
    attempts are spent. Replaying that state into a scene that has been
    rearranged since would be a far worse bug than the 14-second planning call
    it saves.
    """
    from heron.agent import Agent
    from heron.config import HeronConfig
    from heron.program import StepStatus
    from heron.robot.mock import MockRobot, standard_scene
    from heron.robot.safety import SafeRobot
    import sys
    sys.path.insert(0, "tests")
    from test_agent_loop import ScriptedOrchestrator

    cfg = HeronConfig()
    cfg.episodes_dir = str(tmp_path / "ep")
    cfg.memory_dir = str(tmp_path / "mem")
    cfg.skills_dir = str(tmp_path / "sk")
    mock = MockRobot()
    standard_scene(mock)
    agent = Agent(SafeRobot(mock, cfg.safety), ScriptedOrchestrator(mock), cfg, name="pc")

    finished = _prog([Step(id="s1", skill="perceive", args={"entities": ["all"]},
                           status=StepStatus.DONE, attempts=3),
                      Step(id="s2", skill="pick", args={"entity": "block"},
                           status=StepStatus.DONE, attempts=2)])
    finished.aborted = True
    finished.abort_reason = "from the episode that earned it"
    agent.memory.remember_program("put the block in the bowl",
                                  finished.model_dump(mode="json"))

    got = agent._recall_or_plan("put the block in the bowl", [])
    assert all(s.status is StepStatus.PENDING for s in got.steps)
    assert all(s.attempts == 0 for s in got.steps)
    assert not got.aborted and not got.abort_reason


def test_only_a_plan_that_worked_is_remembered(tmp_path):
    from heron.memory import MemoryStore

    store = MemoryStore(str(tmp_path))
    assert store.recall_program("put the block in the bowl") is None


def test_a_plan_that_stops_working_is_dropped(tmp_path):
    """A cached plan that quietly stopped working would be much worse than the
    call it saved, so every use is scored and a failing one retires itself."""
    from heron.memory import MemoryStore

    store = MemoryStore(str(tmp_path))
    store.remember_program("do the thing", {"goal": "do the thing"})
    assert store.recall_program("do the thing") is not None
    for _ in range(MemoryStore.PROGRAM_MIN_USES_BEFORE_RETIRING):
        store.record_program_outcome("do the thing", False)
    assert store.recall_program("do the thing") is None


def test_the_same_instruction_in_any_wording_of_case_is_one_program(tmp_path):
    from heron.memory import MemoryStore

    store = MemoryStore(str(tmp_path))
    store.remember_program("Put the RED block  in the tray.", {"goal": "x"})
    assert store.recall_program("put the red block in the tray") is not None
