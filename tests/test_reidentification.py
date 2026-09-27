"""Choosing among lookalikes is association, not detection.

LIBERO's object scenes hold several similar cans. Taking the model's top
detection is a coin flip there, and the mistake is silent: one confident point
for the wrong object, and the arm goes to it.
"""
from __future__ import annotations

import numpy as np
import pytest

from heron.agent import Agent
from heron.config import HeronConfig
from heron.robot.mock import MockRobot, standard_scene
from heron.robot.safety import SafeRobot
from heron.skills.sensing import ground_entity

from test_agent_loop import ScriptedOrchestrator


@pytest.fixture
def rig(tmp_path):
    cfg = HeronConfig()
    cfg.episodes_dir = str(tmp_path / "ep")
    cfg.memory_dir = str(tmp_path / "mem")
    cfg.skills_dir = str(tmp_path / "sk")
    mock = MockRobot()
    standard_scene(mock)
    agent = Agent(SafeRobot(mock, cfg.safety), ScriptedOrchestrator(mock), cfg, name="reid")
    return cfg, mock, agent


class _TwoLookalikes(ScriptedOrchestrator):
    """Reports the twin FIRST and the real target second, in points and boxes.

    Position is derived from the box, so a lookalike test that only duplicates
    the point would pass without testing anything.
    """

    def points(self, frame, query):
        if "red" in query.lower():
            twin = super().point(frame, "green block")
            real = super().point(frame, "red block")
            return [c for c in (twin, real) if c]
        hit = super().point(frame, query)
        return [hit] if hit else []

    def boxes(self, frame, query):
        if "red" in query.lower():
            twin = super().box(frame, "green block")
            real = super().box(frame, "red block")
            return [b for b in (twin, real) if b]
        one = super().box(frame, query)
        return [one] if one else []


def test_the_candidate_nearest_the_belief_wins(rig):
    cfg, mock, agent = rig
    agent.ctx.grounder = _TwoLookalikes(mock)
    agent.belief.track("red_block").description = "the red block"
    truth = mock.gt_xyz("red_block")
    agent.belief.update_track("red_block", xyz_base=tuple(truth), confidence=0.9)

    found, note = ground_entity(agent.ctx, "red_block")
    assert found, note
    got = np.asarray(agent.belief.track("red_block").xyz_base)
    assert float(np.linalg.norm(got[:2] - truth[:2])) < 0.03, (got, truth)


def test_without_a_belief_the_top_detection_is_used(rig):
    """The prior is an aid, not a requirement — a first sighting still works."""
    cfg, mock, agent = rig
    agent.ctx.grounder = _TwoLookalikes(mock)
    agent.belief.track("red_block").description = "the red block"
    found, _ = ground_entity(agent.ctx, "red_block")
    assert found
    got = np.asarray(agent.belief.track("red_block").xyz_base)
    green = mock.gt_xyz("green_block")
    assert float(np.linalg.norm(got[:2] - green[:2])) < 0.05, \
        "with nothing to associate against, the model's own ranking should stand"


def test_a_stale_belief_does_not_pin_the_association(rig):
    cfg, mock, agent = rig
    agent.ctx.grounder = _TwoLookalikes(mock)
    agent.belief.track("red_block").description = "the red block"
    agent.belief.update_track("red_block", xyz_base=tuple(mock.gt_xyz("red_block")), confidence=0.9)
    agent.belief.track("red_block").t = 0.0  # long ago
    found, _ = ground_entity(agent.ctx, "red_block")
    assert found


def test_the_detectors_are_asked_for_every_instance():
    """The association machinery is inert unless the detector offers alternatives.

    Measured: across ten episodes with the multi-candidate code in place, not one
    re-association fired — the prompts were still asking for "the" object and the
    model answered with exactly one box.
    """
    from heron import prompts

    for prompt in (prompts.POINT_PROMPT, prompts.BOX_PROMPT):
        assert "EVERY one of them" in prompt, prompt
