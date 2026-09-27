"""Learned skills must expand into runnable typed steps — and refuse otherwise."""
from __future__ import annotations

import pytest

from heron.skills.library import LearnedSkill, SkillLibrary, SkillStep
from heron.types import PredicateSpec

PRIMITIVES = ["pick", "place", "ground", "descend_to_contact", "move_to"]


def _grasp_skill() -> LearnedSkill:
    return LearnedSkill(
        name="careful_block_grasp",
        description="Locate a block, descend until contact, then grasp",
        when_to_use="small rigid objects whose height is uncertain",
        params=["target", "arm"],
        body=[
            SkillStep(skill="ground", args={"entity": "$target"},
                      post=[PredicateSpec(name="visible", args=["$target"])]),
            SkillStep(skill="descend_to_contact", args={"arm": "$arm", "over": "$target"}),
            SkillStep(skill="pick", args={"arm": "$arm", "target": "$target"},
                      pre=[PredicateSpec(name="gripper_empty", args=["$arm"])],
                      post=[PredicateSpec(name="holding", args=["$arm", "$target"])]),
        ],
    )


def test_expands_into_typed_steps_with_substituted_arguments(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_grasp_skill(), PRIMITIVES)
    steps = lib.expand("careful_block_grasp", {"target": "red_block", "arm": "right"}, "s3")
    assert [s.skill for s in steps] == ["ground", "descend_to_contact", "pick"]
    assert steps[2].args == {"arm": "right", "target": "red_block"}
    assert steps[0].post[0].args == ["red_block"]
    assert steps[2].post[0].args == ["right", "red_block"]


def test_step_ids_stay_unique_across_two_invocations(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_grasp_skill(), PRIMITIVES)
    a = lib.expand("careful_block_grasp", {"target": "b1", "arm": "right"}, "s1")
    b = lib.expand("careful_block_grasp", {"target": "b2", "arm": "left"}, "s2")
    ids = [s.id for s in a + b]
    assert len(ids) == len(set(ids)), f"colliding step ids: {ids}"


def test_rejects_a_body_calling_something_that_is_not_a_primitive(tmp_path):
    """A learned skill may only compose what the platform already allows."""
    lib = SkillLibrary(tmp_path)
    bad = _grasp_skill()
    bad.body.append(SkillStep(skill="run_shell", args={"cmd": "rm -rf /"}))
    with pytest.raises(ValueError, match="not a platform primitive"):
        lib.add(bad, PRIMITIVES)


def test_rejects_undeclared_and_unused_parameters(tmp_path):
    lib = SkillLibrary(tmp_path)
    undeclared = _grasp_skill()
    undeclared.params = ["target"]
    with pytest.raises(ValueError, match="undeclared parameters"):
        lib.add(undeclared, PRIMITIVES)

    unused = _grasp_skill()
    unused.params = ["target", "arm", "speed"]
    with pytest.raises(ValueError, match="never used"):
        lib.add(unused, PRIMITIVES)


def test_rejects_shadowing_a_primitive(tmp_path):
    lib = SkillLibrary(tmp_path)
    clash = _grasp_skill()
    clash.name = "pick"
    with pytest.raises(ValueError, match="shadow"):
        lib.add(clash, PRIMITIVES)


def test_expansion_requires_every_declared_argument(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_grasp_skill(), PRIMITIVES)
    with pytest.raises(ValueError, match="missing arguments"):
        lib.expand("careful_block_grasp", {"target": "b1"}, "s1")


def test_survives_a_restart_and_tracks_outcomes(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_grasp_skill(), PRIMITIVES)
    lib.record_outcome("careful_block_grasp", True, episode="ep-1")
    lib.record_outcome("careful_block_grasp", False, episode="ep-2")

    reloaded = SkillLibrary(tmp_path)
    s = reloaded.get("careful_block_grasp")
    assert s is not None and s.uses == 2 and s.successes == 1
    assert s.provenance == ["ep-1", "ep-2"]
    assert s.success_rate == pytest.approx(0.5)


def test_catalog_can_hide_skills_that_stopped_working(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_grasp_skill(), PRIMITIVES)
    for _ in range(4):
        lib.record_outcome("careful_block_grasp", False)
    assert "careful_block_grasp" in lib.catalog()
    assert "careful_block_grasp" not in lib.catalog(min_success_rate=0.5)


def test_a_corrupt_entry_does_not_take_the_library_down(tmp_path):
    lib = SkillLibrary(tmp_path)
    lib.add(_grasp_skill(), PRIMITIVES)
    (tmp_path / "broken.json").write_text("{not json")
    reloaded = SkillLibrary(tmp_path)
    assert reloaded.names() == ["careful_block_grasp"]
