"""The relation has to survive all the way to which object the arm goes to.

test_spatial_relations.py checks the arithmetic. This checks the wiring: that
the detector's candidates all reach the chooser, that anchors are grounded
before the thing that refers to them, and — the part that actually failed on the
benchmark — that the entity ends up bound to the bowl the sentence names rather
than to whichever one the detector happened to rank first.
"""
from __future__ import annotations

import numpy as np
import pytest

from heron.agent import Agent
from heron.config import HeronConfig
from heron.robot.mock import MockRobot
from heron.robot.safety import SafeRobot
from heron.skills.sensing import ground_entity, perceive, relational_order
from heron.types import EntityDecl, SpatialRelation

from test_agent_loop import ScriptedOrchestrator

# Three identical bowls, a plate and a ramekin — LIBERO's spatial scene in
# miniature. Only bowl_mid lies between the two anchors.
BOWLS = {
    "bowl_near_plate": (0.14, 0.10, 0.03),
    "bowl_mid": (0.02, 0.10, 0.03),
    "bowl_far": (-0.06, 0.26, 0.03),
}
PLATE = (0.16, 0.10, 0.01)
RAMEKIN = (-0.10, 0.10, 0.02)


class _AllThreeBowls(ScriptedOrchestrator):
    """Answers "the black bowl" with every bowl, in a deliberately unhelpful order.

    bowl_far is returned first, so any code that takes the detector's top pick
    binds to the wrong object and the test fails — which is exactly what the
    benchmark was reporting.
    """

    ORDER = ["bowl_far", "bowl_near_plate", "bowl_mid"]

    def points(self, frame, query):
        if "bowl" in query.lower():
            out = []
            for name in self.ORDER:
                obj = self.mock.objects[name]
                hit = super().point(frame, name.replace("_", " "))
                if hit:
                    out.append(hit)
            return out
        hit = super().point(frame, query)
        return [hit] if hit else []

    def boxes(self, frame, query):
        # No boxes: the position then comes from the detector point, which is the
        # same estimate the chooser scored. Boxes are tested elsewhere.
        return []


@pytest.fixture
def rig(tmp_path):
    cfg = HeronConfig()
    cfg.episodes_dir = str(tmp_path / "ep")
    cfg.memory_dir = str(tmp_path / "mem")
    cfg.skills_dir = str(tmp_path / "sk")
    mock = MockRobot()
    for name, xyz in BOWLS.items():
        mock.add_object(name, "bowl", (30, 30, 30), xyz, (0.06, 0.06, 0.04))
    mock.add_object("plate", "plate", (240, 240, 235), PLATE, (0.14, 0.14, 0.01))
    mock.add_object("ramekin", "cup", (240, 240, 235), RAMEKIN, (0.05, 0.05, 0.04))
    agent = Agent(SafeRobot(mock, cfg.safety), _AllThreeBowls(mock), cfg, name="rel")
    agent.ctx.grounder = _AllThreeBowls(mock)
    return cfg, mock, agent


def _declare(agent, negated=False):
    agent.belief.declare_entities([
        EntityDecl(id="plate", description="the white plate"),
        EntityDecl(id="ramekin", description="the small white ramekin"),
        EntityDecl(id="bowl_target", description="the black bowl",
                   relation=SpatialRelation(kind="between", of=["plate", "ramekin"],
                                            negated=negated)),
    ])


def _bound_to(agent) -> str:
    """Which mock object the entity actually ended up on."""
    got = np.asarray(agent.belief.track("bowl_target").xyz_base, dtype=float)
    named = {**BOWLS}
    return min(named, key=lambda n: np.linalg.norm(np.asarray(named[n])[:2] - got[:2]))


def test_between_binds_to_the_middle_bowl_not_the_top_detection(rig):
    cfg, mock, agent = rig
    _declare(agent)
    res = perceive(agent.ctx, entities=["plate", "ramekin", "bowl_target"])
    assert res.ok, res.error
    assert _bound_to(agent) == "bowl_mid", res.info


def test_negated_between_binds_to_a_different_bowl(rig):
    cfg, mock, agent = rig
    _declare(agent, negated=True)
    perceive(agent.ctx, entities=["plate", "ramekin", "bowl_target"])
    assert _bound_to(agent) != "bowl_mid"


def test_anchors_are_grounded_before_the_entity_that_names_them(rig):
    cfg, mock, agent = rig
    _declare(agent)
    order = relational_order(agent.ctx, ["bowl_target", "plate", "ramekin"])
    assert order.index("plate") < order.index("bowl_target")
    assert order.index("ramekin") < order.index("bowl_target")


def test_an_anchor_the_planner_forgot_is_pulled_in(rig):
    """A relation whose anchors were never perceived is the same defect as no
    relation, and it is one extra detection to prevent."""
    cfg, mock, agent = rig
    _declare(agent)
    order = relational_order(agent.ctx, ["bowl_target"])
    assert set(order) == {"bowl_target", "plate", "ramekin"}
    assert order[-1] == "bowl_target"


def test_a_mutual_relation_does_not_hang(rig):
    cfg, mock, agent = rig
    agent.belief.declare_entities([
        EntityDecl(id="a", description="the black bowl",
                   relation=SpatialRelation(kind="near", of=["b"])),
        EntityDecl(id="b", description="the black bowl",
                   relation=SpatialRelation(kind="near", of=["a"])),
    ])
    assert set(relational_order(agent.ctx, ["a", "b"])) == {"a", "b"}


def test_a_fresh_belief_outranks_the_relation(rig):
    """Once bound, re-grounding must defend the binding rather than re-derive it:
    a bowl picked up from between the plate and the ramekin is no longer between
    them, and re-applying the relation would silently jump to another bowl."""
    cfg, mock, agent = rig
    _declare(agent)
    truth = np.asarray(BOWLS["bowl_far"], dtype=float)
    agent.belief.update_track("bowl_target", xyz_base=tuple(truth), confidence=0.9,
                              from_sighting=True)
    for anchor, xyz in (("plate", PLATE), ("ramekin", RAMEKIN)):
        agent.belief.update_track(anchor, xyz_base=xyz, confidence=0.9, from_sighting=True)

    found, note = ground_entity(agent.ctx, "bowl_target")
    assert found, note
    assert _bound_to(agent) == "bowl_far", note


def test_without_a_relation_the_top_detection_is_still_used(rig):
    cfg, mock, agent = rig
    agent.belief.declare_entities([
        EntityDecl(id="bowl_target", description="the black bowl"),
    ])
    found, note = ground_entity(agent.ctx, "bowl_target")
    assert found, note
    assert _bound_to(agent) == _AllThreeBowls.ORDER[0]
