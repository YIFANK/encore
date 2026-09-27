"""The agent must expand learned skills, then credit them for what happened."""
from __future__ import annotations

from heron.agent import Agent
from heron.config import HeronConfig
from heron.program import Step, StepStatus, TaskProgram
from heron.skills.library import LearnedSkill, SkillStep
from heron.types import PredicateSpec


class _StubOrchestrator:
    def plan(self, *a, **k):
        raise AssertionError("not used")

    def point(self, *a, **k):
        return None


def _agent(tmp_path) -> Agent:
    cfg = HeronConfig()
    cfg.skills_dir = str(tmp_path / "skills")
    cfg.memory_dir = str(tmp_path / "memory")
    cfg.episodes_dir = str(tmp_path / "episodes")
    return Agent(robot=None, orchestrator=_StubOrchestrator(), cfg=cfg, name="test")


def _learn(agent, name="two_stage_pick"):
    agent.skills.add(LearnedSkill(
        name=name,
        description="look, then pick",
        params=["target", "arm", "camera"],
        body=[
            SkillStep(skill="perceive", args={"camera": "$camera"}),
            SkillStep(skill="pick", args={"arm": "$arm", "target": "$target"},
                      post=[PredicateSpec(name="holding", args=["$arm", "$target"])]),
        ],
    ), agent.registry.names())


def test_a_learned_skill_expands_into_primitive_steps(tmp_path):
    agent = _agent(tmp_path)
    _learn(agent)
    program = TaskProgram(goal="test", steps=[
        Step(id="s1", skill="two_stage_pick", args={"target": "block", "arm": "right", "camera": "cam_high"}),
        Step(id="s2", skill="place", args={"arm": "right"}),
    ])
    agent._expand_learned(program)

    assert [s.skill for s in program.steps] == ["perceive", "pick", "place"]
    assert all(s.origin_skill == "two_stage_pick" for s in program.steps[:2])
    assert program.steps[2].origin_skill is None
    assert program.steps[1].args == {"arm": "right", "target": "block"}


def test_invocation_conditions_wrap_the_expansion(tmp_path):
    agent = _agent(tmp_path)
    _learn(agent)
    program = TaskProgram(goal="test", steps=[Step(
        id="s1", skill="two_stage_pick", args={"target": "block", "arm": "right", "camera": "cam_high"},
        pre=[PredicateSpec(name="gripper_empty", args=["right"])],
        post=[PredicateSpec(name="holding", args=["right", "block"])],
    )])
    agent._expand_learned(program)
    assert program.steps[0].pre[0].name == "gripper_empty", "precondition must gate the first step"
    assert program.steps[-1].post[-1].name == "holding", "postcondition must land on the last step"


def test_an_unknown_skill_is_left_for_the_executor_to_report(tmp_path):
    agent = _agent(tmp_path)
    program = TaskProgram(goal="test", steps=[Step(id="s1", skill="nonexistent", args={})])
    agent._expand_learned(program)
    assert [s.skill for s in program.steps] == ["nonexistent"]


def test_a_clean_run_credits_the_skill(tmp_path):
    agent = _agent(tmp_path)
    _learn(agent)
    program = TaskProgram(goal="test", steps=[
        Step(id="a", skill="perceive", args={}, origin_skill="two_stage_pick",
             status=StepStatus.DONE, attempts=1),
        Step(id="b", skill="pick", args={}, origin_skill="two_stage_pick",
             status=StepStatus.DONE, attempts=1),
    ])
    agent._consolidate_skills(program, "succeeded")
    s = agent.skills.get("two_stage_pick")
    assert (s.uses, s.successes) == (1, 1)


def test_a_repaired_step_charges_the_skill_even_if_the_episode_succeeded(tmp_path):
    """Needing repair is the skill's own defect — that is the signal to revise it."""
    agent = _agent(tmp_path)
    _learn(agent)
    program = TaskProgram(goal="test", steps=[
        Step(id="a", skill="perceive", args={}, origin_skill="two_stage_pick",
             status=StepStatus.DONE, attempts=1),
        Step(id="b", skill="pick", args={}, origin_skill="two_stage_pick",
             status=StepStatus.DONE, attempts=3),
    ])
    agent._consolidate_skills(program, "succeeded")
    s = agent.skills.get("two_stage_pick")
    assert (s.uses, s.successes) == (1, 0)


def test_a_failure_elsewhere_does_not_blame_the_skill(tmp_path):
    agent = _agent(tmp_path)
    _learn(agent)
    program = TaskProgram(goal="test", steps=[
        Step(id="a", skill="perceive", args={}, origin_skill="two_stage_pick",
             status=StepStatus.DONE, attempts=1),
        Step(id="b", skill="pick", args={}, origin_skill="two_stage_pick",
             status=StepStatus.DONE, attempts=1),
        Step(id="c", skill="place", args={}, status=StepStatus.FAILED, attempts=3),
    ])
    agent._consolidate_skills(program, "failed")
    s = agent.skills.get("two_stage_pick")
    assert (s.uses, s.successes) == (1, 1)


def test_learning_and_revising_from_executed_steps(tmp_path):
    agent = _agent(tmp_path)
    executed = [
        Step(id="s1", skill="inspect", args={"entity": "red_block"}, status=StepStatus.DONE),
        Step(id="s2", skill="pick", args={"arm": "right", "target": "red_block"},
             status=StepStatus.DONE),
    ]
    agent.learn_skill("block_lift", "ground then pick a block", ["target", "arm"],
                      executed, {"target": "red_block", "arm": "right"},
                      when_to_use="small blocks on the table")
    assert agent.skills.get("block_lift").body[1].args == {"arm": "$arm", "target": "$target"}
    assert "block_lift" in agent._catalog()

    repaired = executed[:1] + [
        Step(id="s1b", skill="open_gripper", args={"arm": "right"}, status=StepStatus.DONE),
    ] + executed[1:]
    agent.revise_skill("block_lift", repaired, {"target": "red_block", "arm": "right"},
                       "grasp missed with the gripper still closed")
    s = agent.skills.get("block_lift")
    assert s.version == 2
    assert [b.skill for b in s.body] == ["inspect", "open_gripper", "pick"]


def test_the_catalog_offers_learned_skills_to_the_planner(tmp_path):
    agent = _agent(tmp_path)
    assert "Learned skills" not in agent._catalog()
    _learn(agent)
    doc = agent._catalog()
    assert "Learned skills" in doc and "two_stage_pick" in doc


def test_the_survey_binds_names_through_its_own_goal():
    """Rig 2026-08-06: two white plates, and the words could not choose.

    'white_plate' shares every word with large_white_plate and with
    small_white_plate, so similarity picked the first in the list and the arm
    spent the episode reaching for a plate 20 cm outside its workspace. The
    survey had already chosen — its goal said on(green_cube, small_white_plate)
    and its note said why — and both goals are predicates over the same task,
    so their arguments correspond.
    """
    from heron.agent import _predicate

    assert _predicate("on(green_cube, small_white_plate)") == (
        "on", ["green_cube", "small_white_plate"])
    assert _predicate("not on(a, b)") == ("on", ["a", "b"])
    assert _predicate("gripper_empty(right)") == ("gripper_empty", ["right"])
    assert _predicate("nonsense") is None
    assert _predicate("") is None

    # The binding itself: plan predicate on(green_block, white_plate) against
    # survey predicate on(green_cube, small_white_plate).
    plan = ("on", ["green_block", "white_plate"])
    survey = _predicate("on(green_cube, small_white_plate)")
    assert survey is not None and survey[0] == plan[0]
    bound = dict(zip(plan[1], survey[1]))
    assert bound["white_plate"] == "small_white_plate", "the model's own choice wins"
    assert bound["green_block"] == "green_cube"
