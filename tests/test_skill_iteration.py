"""Self-iteration: a skill must improve on repairs and back out of bad revisions."""
from __future__ import annotations

import pytest

from heron.skills.library import LearnedSkill, SkillLibrary, SkillStep
from heron.types import PredicateSpec

PRIMITIVES = ["pick", "place", "ground", "descend_to_contact", "move_to"]


def _skill() -> LearnedSkill:
    return LearnedSkill(
        name="block_grasp",
        description="Grasp a block",
        params=["target", "arm"],
        body=[
            SkillStep(skill="ground", args={"entity": "$target"}),
            SkillStep(skill="pick", args={"arm": "$arm", "target": "$target"},
                      post=[PredicateSpec(name="holding", args=["$arm", "$target"])]),
        ],
    )


def _repaired_body() -> list[SkillStep]:
    """What the agent learned the hard way: descend to contact before closing."""
    return [
        SkillStep(skill="ground", args={"entity": "$target"}),
        SkillStep(skill="descend_to_contact", args={"arm": "$arm", "over": "$target"}),
        SkillStep(skill="pick", args={"arm": "$arm", "target": "$target"},
                  post=[PredicateSpec(name="holding", args=["$arm", "$target"])]),
    ]


def test_a_repair_becomes_the_next_version(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_skill(), PRIMITIVES)
    lib.record_outcome("block_grasp", False)
    lib.revise("block_grasp", _repaired_body(), "grasp missed without contact descent",
               PRIMITIVES, episode="ep-7")

    s = lib.get("block_grasp")
    assert s.version == 2
    assert [b.skill for b in s.body] == ["ground", "descend_to_contact", "pick"]
    assert s.uses == 0 and s.successes == 0, "a new version must earn its own record"
    assert len(s.history) == 1 and s.history[0].version == 1
    # And it is immediately usable.
    steps = lib.expand("block_grasp", {"target": "b1", "arm": "right"}, "s1")
    assert [x.skill for x in steps] == ["ground", "descend_to_contact", "pick"]


def test_an_identical_body_does_not_churn_the_version(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_skill(), PRIMITIVES)
    lib.revise("block_grasp", _skill().body, "no change", PRIMITIVES)
    assert lib.get("block_grasp").version == 1


def test_a_revision_that_breaks_the_contract_is_refused(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_skill(), PRIMITIVES)
    bad = _repaired_body() + [SkillStep(skill="teleport", args={})]
    with pytest.raises(ValueError, match="not a platform primitive"):
        lib.revise("block_grasp", bad, "nonsense", PRIMITIVES)
    assert lib.get("block_grasp").version == 1, "a rejected revision must not mutate the skill"


def test_a_worse_revision_is_rolled_back(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_skill(), PRIMITIVES)
    for _ in range(4):                       # v1 works well
        lib.record_outcome("block_grasp", True)
    lib.revise("block_grasp", _repaired_body(), "trying a contact descent", PRIMITIVES)
    for _ in range(3):                       # v2 does not
        lib.record_outcome("block_grasp", False)

    assert lib.consolidate("block_grasp") == "rolled back to v1"
    s = lib.get("block_grasp")
    assert [b.skill for b in s.body] == ["ground", "pick"]
    assert s.successes == 4, "rolling back restores the version's earned record"


def test_a_revision_is_not_judged_before_it_has_run(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_skill(), PRIMITIVES)
    for _ in range(4):
        lib.record_outcome("block_grasp", True)
    lib.revise("block_grasp", _repaired_body(), "trying", PRIMITIVES)
    lib.record_outcome("block_grasp", False)
    assert lib.rollback_if_worse("block_grasp") is None
    assert lib.get("block_grasp").version == 2


def test_a_hopeless_skill_is_retired_and_hidden(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_skill(), PRIMITIVES)
    for _ in range(5):
        lib.record_outcome("block_grasp", False)
    assert lib.consolidate("block_grasp") == "deprecated (kept failing)"
    assert "block_grasp" not in lib.catalog()
    assert lib.get("block_grasp") is not None, "deprecated, not deleted — evidence is kept"


def test_a_working_skill_is_left_alone(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_skill(), PRIMITIVES)
    for _ in range(5):
        lib.record_outcome("block_grasp", True)
    assert lib.consolidate("block_grasp") == "kept"
    assert lib.get("block_grasp").version == 1


def test_versions_survive_a_restart(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_skill(), PRIMITIVES)
    lib.revise("block_grasp", _repaired_body(), "contact descent", PRIMITIVES)

    reloaded = SkillLibrary(tmp_path)
    s = reloaded.get("block_grasp")
    assert s.version == 2 and len(s.history) == 1
    assert s.history[0].reason == "contact descent"


def test_generalizing_executed_steps_puts_the_parameters_back(tmp_path):
    """A recording of one episode is not a skill until its entities become slots."""
    from heron.program import Step

    executed = [
        Step(id="s1_a", skill="ground", args={"entity": "red_block"}),
        Step(id="s1_b", skill="descend_to_contact", args={"arm": "right", "lift_m": 0.06}),
        Step(id="s1_c", skill="pick", args={"arm": "right", "target": "red_block"},
             post=[PredicateSpec(name="holding", args=["right", "red_block"])]),
    ]
    body = SkillLibrary.generalize(executed, {"target": "red_block", "arm": "right"})
    assert body[0].args == {"entity": "$target"}
    assert body[2].args == {"arm": "$arm", "target": "$target"}
    assert body[2].post[0].args == ["$arm", "$target"]
    # Not an argument, so it is technique and stays literal.
    assert body[1].args["lift_m"] == 0.06


def test_a_generalized_body_is_a_valid_revision(tmp_path):
    from heron.program import Step

    lib = SkillLibrary(tmp_path)
    lib.add(_skill(), PRIMITIVES)
    executed = [
        Step(id="s1_a", skill="ground", args={"entity": "blue_block"}),
        Step(id="s1_b", skill="descend_to_contact", args={"arm": "left", "over": "blue_block"}),
        Step(id="s1_c", skill="pick", args={"arm": "left", "target": "blue_block"}),
    ]
    body = SkillLibrary.generalize(executed, {"target": "blue_block", "arm": "left"})
    lib.revise("block_grasp", body, "learned from repair", PRIMITIVES)
    steps = lib.expand("block_grasp", {"target": "green_block", "arm": "right"}, "s9")
    assert steps[1].args == {"arm": "right", "over": "green_block"}, \
        "the lesson must transfer to a different object and arm"
