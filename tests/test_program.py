import pytest

from heron.program import EditType, ProgramEdit, Step, StepStatus, TaskProgram
from heron.types import EntityDecl, PredicateSpec


def prog() -> TaskProgram:
    return TaskProgram(
        goal="test",
        entities=[EntityDecl(id="a", description="the a")],
        steps=[
            Step(id="s1", skill="perceive", args={}),
            Step(id="s2", skill="pick", args={"entity": "a"}),
            Step(id="s3", skill="place", args={"entity": "a", "target": "b"}),
        ],
    )


def test_next_pending_walks_in_order():
    p = prog()
    assert p.next_pending().id == "s1"
    p.steps[0].status = StepStatus.DONE
    assert p.next_pending().id == "s2"


def test_retry_resets_and_merges_args():
    p = prog()
    p.steps[1].status = StepStatus.FAILED
    p.apply_edit(ProgramEdit(type=EditType.RETRY_STEP, step_id="s2", new_args={"dx": 0.02}))
    assert p.steps[1].status is StepStatus.PENDING
    assert p.steps[1].args == {"entity": "a", "dx": 0.02}


def test_insert_before_failed_step_resets_anchor():
    p = prog()
    p.steps[1].status = StepStatus.FAILED
    new = Step(id="s4", skill="perceive", args={})
    p.apply_edit(ProgramEdit(type=EditType.INSERT_STEPS, step_id="s2", steps=[new]))
    assert [s.id for s in p.steps] == ["s1", "s4", "s2", "s3"]
    assert p.step("s2").status is StepStatus.PENDING


def test_insert_rejects_duplicate_ids():
    p = prog()
    with pytest.raises(ValueError):
        p.apply_edit(ProgramEdit(type=EditType.INSERT_STEPS, step_id="s2",
                                 steps=[Step(id="s1", skill="perceive", args={})]))


def test_replace_tail_keeps_prefix():
    p = prog()
    p.steps[0].status = StepStatus.DONE
    p.apply_edit(ProgramEdit(type=EditType.REPLACE_TAIL, step_id="s2",
                             steps=[Step(id="t1", skill="pick", args={"entity": "a"})]))
    assert [s.id for s in p.steps] == ["s1", "t1"]


def test_abort_and_goal_predicates():
    p = prog()
    p.apply_edit(ProgramEdit(type=EditType.ABORT, rationale="nope"))
    assert p.done() and p.aborted


def test_predicate_negation_semantics():
    from heron.types import Value

    spec = PredicateSpec(name="holding", args=["a", "left"], negated=True)
    assert spec.satisfied_by(Value.FALSE) is Value.TRUE
    assert spec.satisfied_by(Value.TRUE) is Value.FALSE
    assert spec.satisfied_by(Value.UNKNOWN) is Value.UNKNOWN


def test_a_goal_level_failure_can_be_repaired_by_appending():
    """The planner produced perceive + pick for "put the block on the plate".
    The goal check failed, and every repair bounced off "no step 'goal'" — so a
    plan that OMITS a step could never be fixed."""
    from heron.program import EditType, ProgramEdit, Step, StepStatus, TaskProgram

    prog = TaskProgram(goal="put the block on the plate", steps=[
        Step(id="s1", skill="perceive", args={}, status=StepStatus.DONE),
        Step(id="s2", skill="pick", args={"entity": "block"}, status=StepStatus.DONE),
    ])
    prog.apply_edit(ProgramEdit(
        type=EditType.INSERT_STEPS, step_id=TaskProgram.GOAL_STEP_ID,
        steps=[Step(id="r1", skill="place",
                    args={"entity": "block", "target": "plate"})],
        rationale="the plan never placed anything"))

    assert [s.id for s in prog.steps] == ["s1", "s2", "r1"]
    assert prog.next_pending().skill == "place"


def test_retrying_the_goal_itself_is_refused():
    from heron.program import EditType, ProgramEdit, Step, StepStatus, TaskProgram

    prog = TaskProgram(goal="g", steps=[Step(id="s1", skill="pick", args={},
                                             status=StepStatus.DONE)])
    with pytest.raises(ValueError, match="cannot retry the goal"):
        prog.apply_edit(ProgramEdit(type=EditType.RETRY_STEP,
                                    step_id=TaskProgram.GOAL_STEP_ID,
                                    rationale="pointless"))
