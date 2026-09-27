"""Corrections as data: scoped, layered, withdrawable, and episode-bounded.

The scope machinery is the part with a scar behind it: place_offset generalised
by an implicit string match, a correction crossed between objects, and a gripper
closed on air beside a correctly grounded cube. These tests pin the properties
that failure demands: nothing applies out of scope, narrow beats broad, and an
unvalidated correction dies with its episode.
"""
import time

import numpy as np
import pytest

from heron.agent import Agent
from heron.config import HeronConfig
from heron.orchestrator.scripted import ScriptedOrchestrator
from heron.patches import Patch, PatchLevel, PatchStore, Scope, Validation
from heron.program import Step, StepStatus, TaskProgram
from heron.robot.mock import MockRobot, standard_scene
from heron.robot.safety import SafeRobot
from heron.types import PredicateSpec


def test_an_empty_store_changes_nothing():
    store = PatchStore(None)
    assert store.param(PatchLevel.PREDICATE, "in", "z_slack", 0.06, {}) == 0.06


def test_a_patch_applies_only_inside_its_scope(tmp_path):
    store = PatchStore(tmp_path / "p.jsonl")
    store.add(Patch(level=PatchLevel.PREDICATE, target="on",
                    delta={"xy_max": 0.045},
                    scope=Scope(entity_class="rack"),
                    provenance="test"))
    rack = {"entity_description": "the dish rack"}
    plate = {"entity_description": "the white plate"}
    assert store.param(PatchLevel.PREDICATE, "on", "xy_max", 0.030, rack) == 0.045
    assert store.param(PatchLevel.PREDICATE, "on", "xy_max", 0.030, plate) == 0.030


def test_the_narrow_scope_overrides_the_broad_one(tmp_path):
    store = PatchStore(tmp_path / "p.jsonl")
    store.add(Patch(level=PatchLevel.SKILL, target="pick",
                    delta={"trim": 0.002}, scope=Scope(), provenance="broad"))
    store.add(Patch(level=PatchLevel.SKILL, target="pick",
                    delta={"trim": 0.008},
                    scope=Scope(entity_class="bowl"), provenance="narrow"))
    bowl = {"entity_description": "the blue bowl"}
    block = {"entity_description": "the red block"}
    assert store.param(PatchLevel.SKILL, "pick", "trim", 0.0, bowl) == 0.008
    assert store.param(PatchLevel.SKILL, "pick", "trim", 0.0, block) == 0.002


def test_an_unvalidated_correction_dies_with_its_episode(tmp_path):
    """The promotion rule. A live correction has to act now, but it may not
    outlive the episode until validation has seen it — that is the difference
    between a correction and a superstition."""
    store = PatchStore(tmp_path / "p.jsonl")
    ephemeral = store.add(Patch(level=PatchLevel.GROUNDING, target="rebind",
                                delta={"entity": "red_block", "to": "det_3"},
                                scope=Scope(episode="ep-1"), provenance="operator"))
    keeper = store.add(Patch(level=PatchLevel.PREDICATE, target="in",
                             delta={"z_slack": 0.04},
                             scope=Scope(episode="ep-1"),
                             validation=Validation(trials=5, fixed=3, broken=0),
                             provenance="operator"))
    assert store.end_episode("ep-1") == 1
    assert not next(p for p in store.patches if p.id == ephemeral.id).live()
    assert next(p for p in store.patches if p.id == keeper.id).live()


def test_withdrawal_is_immediate_and_survives_reload(tmp_path):
    path = tmp_path / "p.jsonl"
    store = PatchStore(path)
    p = store.add(Patch(level=PatchLevel.PREDICATE, target="in",
                        delta={"z_slack": 0.2}, provenance="test"))
    store.withdraw(p.id, "regressed on held-out episodes")
    again = PatchStore(path)
    assert not any(q.live() for q in again.patches)
    assert again.param(PatchLevel.PREDICATE, "in", "z_slack", 0.06, {}) == 0.06


# -- the patched verifier -----------------------------------------------------

@pytest.fixture()
def rig(tmp_path):
    cfg = HeronConfig()
    cfg.episodes_dir = str(tmp_path / "ep")
    cfg.memory_dir = str(tmp_path / "mem")
    cfg.skills_dir = str(tmp_path / "sk")
    cfg.patches_path = str(tmp_path / "patches.jsonl")
    mock = MockRobot()
    standard_scene(mock)
    agent = Agent(SafeRobot(mock, cfg.safety), ScriptedOrchestrator(mock), cfg,
                  name="patch-rig")
    # Calling the executor directly skips run()'s setup; without a start time
    # the wall-clock budget reads "1970" and aborts everything instantly.
    agent._t0 = time.time()
    agent.orchestrator.deadline = time.time() + 300
    return cfg, mock, agent


def test_a_predicate_patch_changes_a_verdict_without_a_commit(rig):
    """The in() cavity fraction, loosened for one container class only.

    This is the shape of the whole idea: the constant in the code is untouched,
    the store holds one row, and only the scoped container reads differently.
    """
    from heron.types import Value
    from heron.verify import _in

    cfg, mock, agent = rig
    bowl = mock.gt_xyz("blue_bowl")
    # A block sitting just OUTSIDE the default inner cavity (0.6 of the half
    # extent) but inside a loosened one.
    half = 0.07
    off = half * 0.75
    mock.objects["red_block"].xyz = np.array([bowl[0] + off, bowl[1], bowl[2] + 0.01])
    agent.belief.update_track("blue_bowl", xyz_base=tuple(bowl), confidence=0.95,
                              from_sighting=True)
    agent.belief.update_track("red_block",
                              xyz_base=tuple(mock.objects["red_block"].xyz),
                              confidence=0.95, from_sighting=True)
    agent.belief.track("blue_bowl").description = "the blue bowl"
    agent.belief.track("blue_bowl").span_m = 2 * half
    class _NoOpinion(ScriptedOrchestrator):
        def vqa(self, frames, question):
            from heron.types import Value as V
            return V.UNKNOWN, 0.0, "cannot tell from these views"

    agent.ctx.grounder = _NoOpinion(mock)

    spec = PredicateSpec(name="in", args=["red_block", "blue_bowl"])
    before = _in(agent.ctx, spec)
    assert before.value is Value.FALSE, before.note

    agent.patch_store.add(Patch(
        level=PatchLevel.PREDICATE, target="in",
        delta={"inner_cavity_fraction": 0.9},
        scope=Scope(entity_class="bowl"),
        validation=Validation(trials=5, fixed=4, broken=0),
        provenance="test", rationale="wide shallow bowls hold things off-centre"))
    after = _in(agent.ctx, spec)
    assert after.value is Value.TRUE, after.note

    # And a container whose description is out of scope keeps the strict test.
    # "basin" still resolves to the same mock object (a bowl synonym for the
    # scripted grounder) but does not contain the scope word "bowl".
    agent.belief.track("blue_bowl").description = "the blue basin"
    strict = _in(agent.ctx, spec)
    assert strict.value is Value.FALSE, strict.note


# -- guards and the program-level while ---------------------------------------

def _bare_program(steps, **kw) -> TaskProgram:
    return TaskProgram(goal="test", steps=steps, **kw)


def test_a_false_guard_skips_without_a_fault(rig):
    cfg, mock, agent = rig
    program = _bare_program([
        Step(id="s1", skill="home", args={"arm": "right"},
             when=PredicateSpec(name="holding", args=["red_block", "right"])),
    ])
    agent._execute_until_blocked(program)
    assert program.step("s1").status is StepStatus.SKIPPED
    assert not program.edits, "a skipped guard is not a failure and repairs nothing"


def test_a_true_guard_runs_the_step(rig):
    cfg, mock, agent = rig
    agent.belief.update_track("red_block",
                              xyz_base=tuple(mock.gt_xyz("red_block")),
                              confidence=0.95, from_sighting=True)
    agent.ctx.grounder = ScriptedOrchestrator(mock)
    program = _bare_program([
        Step(id="s1", skill="home", args={"arm": "right"},
             when=PredicateSpec(name="visible", args=["red_block"])),
    ])
    agent._execute_until_blocked(program)
    assert program.step("s1").status is StepStatus.DONE


def test_repeat_until_rearms_the_body_and_respects_its_bound(rig):
    """The program-level while: an unsatisfiable loop predicate re-runs the
    body exactly max_repeats times and then stops loudly, not forever."""
    cfg, mock, agent = rig
    program = _bare_program(
        [Step(id="s1", skill="home", args={"arm": "right"})],
        repeat_until=PredicateSpec(name="holding", args=["red_block", "right"]),
        max_repeats=3)
    agent._execute_until_blocked(program)
    assert program.repeats_done == 2, "3 passes total = 2 re-arms"
    assert program.step("s1").status is StepStatus.DONE


def test_repeat_until_stops_the_moment_the_predicate_holds(rig):
    cfg, mock, agent = rig
    agent.belief.update_track("red_block",
                              xyz_base=tuple(mock.gt_xyz("red_block")),
                              confidence=0.95, from_sighting=True)
    agent.ctx.grounder = ScriptedOrchestrator(mock)
    program = _bare_program(
        [Step(id="s1", skill="home", args={"arm": "right"})],
        repeat_until=PredicateSpec(name="visible", args=["red_block"]),
        max_repeats=5)
    agent._execute_until_blocked(program)
    assert program.repeats_done == 0
    assert program.done()


# -- the console's controls ----------------------------------------------------

def test_an_operator_assertion_overrules_the_verifier_for_one_episode(rig):
    """Assert in(red_block, blue_bowl)=true from the console; verify() must
    return TRUE with operator evidence — and a different predicate, or another
    episode, must be untouched."""
    from heron.console import Console
    from heron.types import Value
    from heron.verify import verify

    cfg, mock, agent = rig
    console = Console.__new__(Console)      # no server; just the correction path
    console.ctx = agent.ctx
    console.snapshot = lambda: {"stub": True}

    out = console.correct({"kind": "predicate_assert",
                           "predicate": "in(red_block,blue_bowl)", "value": "true"})
    assert out["ok"], out
    spec = PredicateSpec(name="in", args=["red_block", "blue_bowl"])
    v = verify(agent.ctx, spec)
    assert v.value is Value.TRUE
    assert "operator assertion" in v.note

    other = PredicateSpec(name="in", args=["green_block", "blue_bowl"])
    assert "operator" not in verify(agent.ctx, other).note

    # The episode ends; the unvalidated assertion dies with it.
    import os
    agent.patch_store.end_episode(os.path.basename(str(agent.log.dir)))
    v2 = verify(agent.ctx, spec)
    assert "operator" not in v2.note


def test_not_there_invalidates_the_belief_and_records_why(rig):
    from heron.console import Console

    cfg, mock, agent = rig
    agent.belief.update_track("red_block", xyz_base=(0.1, 0.2, 0.0),
                              confidence=0.95, from_sighting=True)
    console = Console.__new__(Console)
    console.ctx = agent.ctx
    console.snapshot = lambda: {"stub": True}
    out = console.correct({"kind": "not_there", "entity": "red_block"})
    assert out["ok"], out
    assert agent.belief.track("red_block").confidence == 0.0
    assert any(p.level.value == "grounding" for p in agent.patch_store.patches)


def test_the_loop_body_is_bound_to_an_unsatisfied_instance(rig):
    """repeat_until all_in(X, C) implies each pass serves an X still outside C.

    Pass two of the first live run bound "the red block" to the one already in
    the tray and lifted it back out. The exclusion is the loop's own semantics,
    so the executor attaches it; a planner-declared relation is respected."""
    from heron.types import EntityDecl, SpatialRelation

    cfg, mock, agent = rig
    program = TaskProgram(
        goal="move all the red blocks into the tray",
        entities=[EntityDecl(id="red_block", description="the red block"),
                  EntityDecl(id="tray", description="the grey tray")],
        steps=[Step(id="s1", skill="pick", args={"entity": "red_block"})],
        repeat_until=PredicateSpec(name="all_in", args=["red_block", "tray"]))
    agent._loop_targets_the_unsatisfied(program)
    rel = program.entity("red_block").relation
    assert rel is not None and rel.kind == "inside" and rel.negated
    assert rel.of == ["tray"]
    assert program.entity("tray").relation is None

    # A relation the planner DID declare is not overwritten.
    explicit = TaskProgram(
        goal="g", entities=[EntityDecl(id="red_block", description="d",
                                       relation=SpatialRelation(kind="near", of=["tray"]))],
        steps=[], repeat_until=PredicateSpec(name="all_in", args=["red_block", "tray"]))
    agent._loop_targets_the_unsatisfied(explicit)
    assert explicit.entity("red_block").relation.kind == "near"


def test_guard_arms_are_normalized_like_every_other_predicate(rig):
    """holding(x, auto) as a guard: "auto" is an arg convention, not an arm.
    Unnormalized it verified UNKNOWN, ran the step, and place() raised on an
    empty gripper — a repair the cascade skip should have made unnecessary."""
    from heron.types import EntityDecl

    cfg, mock, agent = rig
    cfg.active_arms = ["right"]        # the mock has two arms; the mode is one
    program = TaskProgram(
        goal="g", entities=[EntityDecl(id="x", description="x")],
        steps=[Step(id="s1", skill="place", args={"entity": "x"},
                    when=PredicateSpec(name="holding", args=["x", "auto"]))])
    agent._normalize_arms(program)
    assert program.step("s1").when.args[1] == "right"


def test_the_loop_entity_changes_identity_at_the_turn(rig):
    """Pass one placed the block, so the freshest prior points INTO the
    container — and grounding's prior-defends-the-binding rule then re-bound
    the same block on pass two and lifted it back out. At the turn of the loop
    the binding is released so the NOT-inside relation chooses afresh."""
    from heron.types import EntityDecl

    cfg, mock, agent = rig
    agent.belief.update_track("red_block", xyz_base=(0.36, 0.28, 0.03),
                              confidence=0.9, from_sighting=False)  # placed there
    program = TaskProgram(
        goal="move all", entities=[EntityDecl(id="red_block", description="the red block")],
        steps=[Step(id="s1", skill="home", args={"arm": "right"})],
        repeat_until=PredicateSpec(name="all_in", args=["red_block", "tray"]),
        max_repeats=3)
    agent._t0 = __import__("time").time()
    agent._execute_until_blocked(program)   # loop predicate never satisfies -> rearms
    assert program.repeats_done == 2
    tr = agent.belief.track("red_block")
    assert tr.confidence == 0.0, "the binding must be released at each turn"
