"""When the fixed cameras cannot see into a container, move and look.

Asking a model to judge containment from the same blind viewpoint converts an
ambiguity into a confident answer — which is how an agent ends up reporting a
success it never achieved. Changing viewpoint answers the question instead.
"""
from __future__ import annotations

import numpy as np
import pytest

from heron.agent import Agent
from heron.config import HeronConfig
from heron.robot.mock import MockRobot
from heron.robot.safety import SafeRobot
from heron.types import PredicateSpec, Value
from heron.verify import check

from heron.robot.mock import standard_scene
from test_agent_loop import ScriptedOrchestrator


@pytest.fixture
def rig(tmp_path):
    cfg = HeronConfig()
    cfg.episodes_dir = str(tmp_path / "ep")
    cfg.memory_dir = str(tmp_path / "mem")
    cfg.skills_dir = str(tmp_path / "sk")
    mock = MockRobot()
    standard_scene(mock)
    robot = SafeRobot(mock, cfg.safety)
    agent = Agent(robot, ScriptedOrchestrator(mock), cfg, name="active")
    return cfg, mock, agent


class _BlindToContents(ScriptedOrchestrator):
    """Fixed cameras cannot see the block; the wrist camera can."""

    def __init__(self, mock):
        super().__init__(mock)
        self.wrist_calls = 0

    def point(self, frame, query):
        if "red" in query.lower():
            if frame.camera.endswith("_wrist"):
                self.wrist_calls += 1
                return super().point(frame, query)
            return None
        return super().point(frame, query)


def _put_block_in_bowl(mock, agent) -> None:
    """Seat the block inside the bowl and tell the belief store what to look for."""
    bowl = mock.objects["blue_bowl"]
    mock.objects["red_block"].xyz = np.array([bowl.xyz[0], bowl.xyz[1], bowl.top_z - 0.01])
    assert mock.supporting_container("red_block") == "blue_bowl"
    for eid, desc in (("red_block", "the red block"), ("blue_bowl", "the blue bowl")):
        agent.belief.track(eid).description = desc


def test_an_occluded_object_is_resolved_by_looking_from_the_wrist(rig):
    cfg, mock, agent = rig
    orch = _BlindToContents(mock)
    agent.orchestrator = orch
    agent.ctx.grounder = orch

    _put_block_in_bowl(mock, agent)
    spec = PredicateSpec(name="in", args=["red_block", "blue_bowl"])
    value, verdict = check(agent.ctx, spec)
    assert orch.wrist_calls > 0, "the verifier never changed viewpoint"
    assert value is Value.TRUE, verdict.note
    assert "active" in verdict.note or "look" in verdict.note


def test_the_active_look_happens_at_most_once(rig):
    """Verification that moves the robot must not recurse."""
    cfg, mock, agent = rig
    orch = _BlindToContents(mock)
    agent.orchestrator = orch
    agent.ctx.grounder = orch
    _put_block_in_bowl(mock, agent)
    before = len(mock.motion_log)
    check(agent.ctx, PredicateSpec(name="in", args=["red_block", "blue_bowl"]))
    moves = len(mock.motion_log) - before
    assert moves > 0
    assert "in_looking" not in agent.ctx.scratch, "the re-entrancy guard leaked"


def test_a_released_object_is_given_time_to_land(rig):
    """The benchmark samples success while stepping; judging mid-fall loses it."""
    cfg, mock, agent = rig
    calls = []
    mock.settle = lambda seconds=1.0: calls.append(seconds)

    from heron.skills.primitives import place, pick

    for eid, desc in (("red_block", "the red block"), ("blue_bowl", "the blue bowl")):
        agent.belief.track(eid).description = desc
    agent.belief.update_track("red_block", xyz_base=tuple(mock.gt_xyz("red_block")), confidence=0.9)
    agent.belief.update_track("blue_bowl", xyz_base=tuple(mock.gt_xyz("blue_bowl")), confidence=0.9)
    pick(agent.ctx, entity="red_block", arm="left")
    place(agent.ctx, entity="red_block", target="blue_bowl")
    assert calls, "place returned without letting the object settle"


def test_the_object_is_aimed_not_the_tool(rig):
    """The object rides off-centre in the gripper; the target is for the object."""
    cfg, mock, agent = rig
    from heron.skills.primitives import place

    agent.belief.track("red_block").description = "the red block"
    agent.belief.track("blue_bowl").description = "the blue bowl"
    bowl = tuple(mock.gt_xyz("blue_bowl"))
    agent.belief.update_track("blue_bowl", xyz_base=bowl, confidence=0.9)
    agent.belief.update_track("red_block", xyz_base=bowl, confidence=0.9, held_by="left")
    agent.belief.track("red_block").carry_offset = (0.025, -0.005, 0.0)

    before = len(mock.motion_log)
    place(agent.ctx, entity="red_block", target="blue_bowl")
    xs = [_x(m) for m in mock.motion_log[before:] if "left ->" in m]
    assert xs, mock.motion_log
    # Commanded tool x must sit 25 mm short of the bowl so the OBJECT lands on it.
    assert min(abs(x - (bowl[0] - 0.025)) for x in xs) < 1e-6, (xs, bowl[0])


def _x(motion_line: str) -> float:
    return float(motion_line.split("[", 1)[1].split(",", 1)[0])


def test_hidden_is_not_missing(rig):
    """An object inside a container must not be reported gone — that sends the
    repair loop off trying to pick up something it already placed."""
    cfg, mock, agent = rig
    orch = _BlindToContents(mock)
    agent.orchestrator = orch
    agent.ctx.grounder = orch
    _put_block_in_bowl(mock, agent)
    agent.belief.update_track("red_block", xyz_base=tuple(mock.gt_xyz("red_block")), confidence=0.6)

    value, verdict = check(agent.ctx, PredicateSpec(name="visible", args=["red_block"]))
    assert value is Value.TRUE, verdict.note
    assert "close-range" in verdict.note
    assert orch.wrist_calls > 0


def test_a_genuinely_absent_object_is_still_reported_missing(rig):
    cfg, mock, agent = rig

    class _BlindEverywhere(ScriptedOrchestrator):
        def point(self, frame, query):
            return None if "red" in query.lower() else super().point(frame, query)

    orch = _BlindEverywhere(mock)
    agent.orchestrator = orch
    agent.ctx.grounder = orch
    agent.belief.track("red_block").description = "the red block"
    agent.belief.update_track("red_block", xyz_base=(0.05, 0.10, 0.02), confidence=0.6)

    value, _ = check(agent.ctx, PredicateSpec(name="visible", args=["red_block"]))
    assert value is Value.FALSE


def test_a_similar_looking_object_does_not_steal_the_identity(rig):
    """Scenes hold several near-identical cans; a confident detector still errs."""
    cfg, mock, agent = rig
    from heron.skills.sensing import ground_entity

    agent.belief.track("red_block").description = "the red block"
    # We believe, moments ago, that the block is at the bowl.
    agent.belief.update_track("red_block", xyz_base=(0.05, -0.15, 0.03), confidence=0.9)

    class _WrongTwin(ScriptedOrchestrator):
        def point(self, frame, query):
            return super().point(frame, "green block") if "red" in query.lower() \
                else super().point(frame, query)

        def box(self, frame, query):
            return super().box(frame, "green block") if "red" in query.lower() \
                else super().box(frame, query)

    agent.ctx.grounder = _WrongTwin(mock)
    found, note = ground_entity(agent.ctx, "red_block")
    assert not found, note
    assert "similar-looking" in note
    # The belief is left alone rather than corrupted.
    assert agent.belief.track("red_block").xyz_base == (0.05, -0.15, 0.03)


def test_a_real_move_is_still_believed_once_a_SIGHTING_is_stale(rig):
    """The guard must not turn into a refusal to accept change.

    For a sighting the release is time: we may simply have looked away while
    the world moved. (A position we ourselves put the object at is released by
    evidence instead — see the next test — because a clock is not a reason to
    stop believing something nothing has acted on.)
    """
    cfg, mock, agent = rig
    from heron.skills.sensing import ground_entity

    agent.belief.track("green_block").description = "the green block"
    agent.belief.update_track("green_block", xyz_base=(0.05, -0.15, 0.03), confidence=0.9,
                              from_sighting=True)
    agent.belief.track("green_block").t = 0.0  # long ago
    found, note = ground_entity(agent.ctx, "green_block")
    assert found, note


def test_looking_where_we_put_it_and_seeing_nothing_releases_the_belief(rig):
    """...and that is what stops the identity guard stranding an object forever.

    An object we placed keeps its identity against every lookalike in the scene
    for as long as we have no evidence against it. The evidence that ends it is
    a camera pointed at the expectation, which the verifier makes anyway.
    """
    cfg, mock, agent = rig
    from heron.skills.sensing import ground_entity, inspect

    class _Blind(ScriptedOrchestrator):
        def point(self, frame, query):
            return None if "green" in query.lower() else super().point(frame, query)

        def points(self, frame, query):
            return [] if "green" in query.lower() else super().points(frame, query)

    agent.belief.track("green_block").description = "the green block"
    # Written the way `place` writes it: dead reckoning, and long ago.
    agent.belief.update_track("green_block", xyz_base=(0.05, -0.15, 0.03), confidence=0.6)
    agent.belief.track("green_block").t = 0.0
    assert not ground_entity(agent.ctx, "green_block")[0], "the twin must not be taken"

    agent.ctx.grounder = _Blind(mock)
    inspect(agent.ctx, entity="green_block")          # look at the expectation
    agent.ctx.grounder = ScriptedOrchestrator(mock)   # it is visible again, elsewhere

    found, note = ground_entity(agent.ctx, "green_block")
    assert found, note
    got = np.asarray(agent.belief.track("green_block").xyz_base)
    assert float(np.linalg.norm(got[:2] - mock.gt_xyz("green_block")[:2])) < 0.05, got
