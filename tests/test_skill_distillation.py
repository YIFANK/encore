"""Which part of a successful episode becomes a skill.

The library could keep, expand, score, revise and retire a learned skill long
before anything ever created one: `learn_skill` was reachable from the tests and
from nowhere in the running system, so `skill_library/` never existed on disk and
every episode started from the same blank vocabulary as the first.

These pin the two judgements that make the entry point worth having — WHICH
fragment is worth naming, and WHAT inside it is situation rather than technique.
Both are pure functions of the program, so they are tested without a robot.
"""
from __future__ import annotations

from heron.program import Step, StepStatus, TaskProgram
from heron.skills import distill
from heron.types import EntityDecl, PredicateSpec


def _step(sid, skill, args, attempts=1, status=StepStatus.DONE, origin=None):
    return Step(id=sid, skill=skill, args=args, status=status, attempts=attempts,
                origin_skill=origin)


def _program(steps, entities=None):
    return TaskProgram(
        goal="put the red block in the blue bowl",
        entities=entities or [
            EntityDecl(id="red_block", description="the small red block"),
            EntityDecl(id="blue_bowl", description="the blue bowl"),
        ],
        steps=steps,
        goal_predicates=[PredicateSpec(name="in", args=["red_block", "blue_bowl"])],
    )


# -- which fragment ----------------------------------------------------------

def test_a_plan_that_worked_first_time_teaches_nothing():
    """A fresh planner would produce it again. Recording it costs prompt space
    forever and buys nothing."""
    program = _program([
        _step("s1", "perceive", {"entities": ["all"]}),
        _step("s2", "pick", {"entity": "red_block", "arm": "left"}),
        _step("s3", "place", {"entity": "red_block", "target": "blue_bowl"}),
    ])
    assert distill.candidates(program) == []


def test_the_fragment_is_the_part_that_was_fought_for_not_the_whole_plan():
    """Measured on the offline demo, whose injected failures are a grasp that
    closes on air and a block snatched from the gripper: taking the whole run of
    completed steps produced a six-step skill described as "perceive then
    perceive then pick then perceive then pick then place" — a recording of one
    episode wearing a skill's clothes."""
    program = _program([
        _step("s1", "perceive", {"entities": ["all"]}),
        _step("s4", "perceive", {"entities": ["red_block"]}),
        _step("s2", "pick", {"entity": "red_block", "arm": "left", "dx": 0.02}, attempts=2),
        _step("rp1a", "perceive", {"entities": ["red_block"]}),
        _step("rp1b", "pick", {"entity": "red_block", "arm": "left"}),
        _step("s3", "place", {"entity": "red_block", "target": "blue_bowl"}),
    ])
    got = distill.candidates(program)
    assert len(got) == 1
    assert [s.skill for s in got[0].steps] == ["perceive", "pick"], \
        [s.skill for s in got[0].steps]
    assert got[0].earned == ["s2"]


def test_the_look_before_a_retry_is_kept_but_the_one_after_is_not():
    """Re-perceiving before trying again is very often the whole technique. A
    trailing look is bookkeeping for whatever came next, and a skill that ends
    with one makes its caller pay for a look it did not ask for."""
    program = _program([
        _step("s1", "perceive", {"entities": ["red_block"]}),
        _step("s2", "pick", {"entity": "red_block", "arm": "left"}, attempts=3),
        _step("s3", "perceive", {"entities": ["red_block"]}),
    ])
    got = distill.candidates(program)
    assert [s.skill for s in got[0].steps] == ["perceive", "pick"]


def test_a_lone_retried_step_is_left_to_the_grasp_hints():
    """One step is already a named thing — the primitive — and "pick with a 20 mm
    nudge" is what the cross-episode grasp hints already store. Learning it here
    too would put the same lesson in two places that can disagree."""
    program = _program([
        _step("s1", "place", {"entity": "red_block", "target": "blue_bowl"}),
        _step("s2", "pick", {"entity": "red_block", "arm": "left"}, attempts=2),
    ])
    # s1 finished clean, so the window is s2 alone: below the two-step minimum.
    assert distill.candidates(program) == []


def test_steps_that_came_from_a_learned_skill_are_not_relearned():
    program = _program([
        _step("a0", "perceive", {"entities": ["red_block"]}, origin="pick_block"),
        _step("a1", "pick", {"entity": "red_block", "arm": "left"}, attempts=2,
              origin="pick_block"),
    ])
    assert distill.candidates(program) == []


def test_an_unfinished_step_breaks_the_run():
    """A fragment is only evidence if it ran to completion."""
    program = _program([
        _step("s1", "perceive", {"entities": ["red_block"]}),
        _step("s2", "pick", {"entity": "red_block", "arm": "left"}, attempts=2,
              status=StepStatus.FAILED),
    ])
    assert distill.candidates(program) == []


# -- situation vs technique --------------------------------------------------

def test_entities_and_the_arm_become_parameters_and_the_offsets_do_not():
    """This is the whole difference between a skill and a recording. The 20 mm
    nudge is why the second attempt worked — it is part of the technique and has
    to stay literal. "red_block" is this episode and has to come back as a
    parameter, or the lesson is welded to one object."""
    program = _program([
        _step("s1", "perceive", {"entities": ["red_block"]}),
        _step("s2", "pick", {"entity": "red_block", "arm": "left", "dx": 0.02, "dy": 0.0},
              attempts=2),
    ])
    cand = distill.candidates(program)[0]
    assert set(cand.params) == {"block", "arm"}
    assert cand.params["block"] == "red_block"
    assert cand.params["arm"] == "left"

    from heron.skills.library import SkillLibrary

    body = SkillLibrary.generalize(cand.steps, distill.args_for(cand))
    pick = body[-1]
    assert pick.args["entity"] == "$block"
    assert pick.args["arm"] == "$arm"
    assert pick.args["dx"] == 0.02, "the nudge is technique, not situation"


def test_a_parameter_is_named_for_the_head_noun_not_the_episode_local_id():
    """Entity ids are episode-local; the head noun of the visual description is
    the part that transfers, which is why cross-episode grasp hints are keyed on
    it too."""
    program = _program(
        [_step("s1", "perceive", {"entities": ["e7"]}),
         _step("s2", "pick", {"entity": "e7", "arm": "right"}, attempts=2)],
        entities=[EntityDecl(id="e7", description="the white basket with a grey liner")],
    )
    cand = distill.candidates(program)[0]
    assert "basket" in cand.params, cand.params


def test_two_entities_with_the_same_head_noun_get_distinct_parameters():
    program = _program(
        [_step("s1", "perceive", {"entities": ["b1", "b2"]}),
         _step("s2", "place", {"entity": "b1", "target": "b2", "arm": "left"}, attempts=2)],
        entities=[EntityDecl(id="b1", description="the small red bowl"),
                  EntityDecl(id="b2", description="the large blue bowl")],
    )
    cand = distill.candidates(program)[0]
    assert len(set(cand.params.values())) == len(cand.params), cand.params
    assert len(cand.params) >= 2


# -- naming and duplicates ---------------------------------------------------

def test_the_name_says_what_it_does():
    program = _program([
        _step("s1", "perceive", {"entities": ["red_block"]}),
        _step("s2", "pick", {"entity": "red_block", "arm": "left"}, attempts=2),
    ])
    cand = distill.candidates(program)[0]
    assert distill.propose_name(cand, set()) == "pick_block"


def test_the_same_technique_twice_is_one_skill_credited_twice():
    """Without this the library grows by a near-duplicate per episode, each
    accumulating its own thin statistics, so none of them ever earns enough
    evidence to be trusted or retired."""
    program = _program([
        _step("s1", "perceive", {"entities": ["red_block"]}),
        _step("s2", "pick", {"entity": "red_block", "arm": "left"}, attempts=2),
    ])
    cand = distill.candidates(program)[0]
    assert distill.already_known(cand, {"pick_block": ("perceive", "pick")}) == "pick_block"
    assert distill.already_known(cand, {"other": ("pick", "place")}) is None


def test_a_clashing_name_does_not_overwrite_the_skill_that_has_it():
    program = _program([
        _step("s1", "perceive", {"entities": ["red_block"]}),
        _step("s2", "pick", {"entity": "red_block", "arm": "left"}, attempts=2),
    ])
    cand = distill.candidates(program)[0]
    got = distill.propose_name(cand, {"pick_block"})
    assert got and got != "pick_block"
