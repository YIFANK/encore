"""End-to-end mock episodes through the full loop: plan -> execute -> verify ->
diagnose -> repair. Each test targets one failure class."""
import numpy as np
import pytest
from pathlib import Path

from heron.agent import Agent
from heron.config import HeronConfig
from heron.episode import read_journal
from heron.orchestrator.scripted import ScriptedOrchestrator
from heron.program import StepStatus
from heron.robot.mock import MockRobot, standard_scene
from heron.robot.safety import SafeRobot
from heron.types import Frame


@pytest.fixture()
def rig(tmp_path):
    cfg = HeronConfig()
    cfg.episodes_dir = str(tmp_path / "episodes")
    cfg.memory_dir = str(tmp_path / "memory")
    mock = MockRobot()
    standard_scene(mock)
    return cfg, mock, SafeRobot(mock, cfg.safety)


def kinds(episode_dir):
    return [r["kind"] for r in read_journal(episode_dir)]


def failures(episode_dir):
    return [(r["step_id"], r["failure_class"]) for r in read_journal(episode_dir) if r["kind"] == "failure"]


def test_clean_episode_succeeds(rig):
    cfg, mock, robot = rig
    agent = Agent(robot, ScriptedOrchestrator(mock), cfg, name="clean")
    summary = agent.run("put the red block in the blue bowl")
    assert summary["status"] == "succeeded"
    assert mock.supporting_container("red_block") == "blue_bowl"
    assert failures(agent.log.dir) == []


def test_grasp_miss_recovers_via_rule_retry(rig):
    cfg, mock, robot = rig
    mock.inject.grasp_misses = 1
    agent = Agent(robot, ScriptedOrchestrator(mock), cfg, name="miss")
    summary = agent.run("put the red block in the blue bowl")
    assert summary["status"] == "succeeded"
    fs = failures(agent.log.dir)
    # The gripper feels a closed-on-air miss during pick itself now, so the
    # failure is skill_exec (was skill_effect back when only the verification
    # layer could catch it) — recovery path is identical.
    assert ("s2", "skill_exec") in fs
    edits = [r["edit"]["type"] for r in read_journal(agent.log.dir) if r["kind"] == "program_edit"]
    assert "retry_step" in edits and "insert_steps" in edits
    assert mock.supporting_container("red_block") == "blue_bowl"


def test_snatched_object_is_state_drift_and_repaired_structurally(rig):
    cfg, mock, robot = rig
    orch = ScriptedOrchestrator(mock)
    stolen = {"armed": True}

    def steal(step, program):
        if stolen["armed"] and step.skill == "pick" and step.status is StepStatus.DONE:
            mock.teleport("red_block", (0.12, 0.08, 0.02))
            stolen["armed"] = False

    agent = Agent(robot, orch, cfg, name="steal", on_step_end=steal)
    summary = agent.run("put the red block in the blue bowl")
    assert summary["status"] == "succeeded"
    fs = failures(agent.log.dir)
    assert any(cls == "state_drift" for _, cls in fs)
    inserted = [r["edit"] for r in read_journal(agent.log.dir)
                if r["kind"] == "program_edit" and r["edit"]["source"] == "orchestrator"]
    assert any(e["type"] == "insert_steps" for e in inserted)
    assert mock.supporting_container("red_block") == "blue_bowl"


class LyingOrchestrator(ScriptedOrchestrator):
    """Lies about the red block's position for N point() calls once armed —
    models a transient grounding error after a correct placement."""

    def __init__(self, mock):
        super().__init__(mock)
        self.lie_countdown = 0

    # One grounding asks for a point in every camera and then one box, so the
    # counter ticks on the box — the call that happens exactly once per
    # grounding — and the points simply follow whatever it is doing.
    def point(self, frame: Frame, query: str):
        if self.lie_countdown > 0 and "red" in query.lower():
            return super().point(frame, "yellow cup")  # confidently wrong location
        return super().point(frame, query)

    def box(self, frame: Frame, query: str):
        # Position is resolved from the BOX (a single pixel reports the object's
        # near surface), so a perception error has to move with it.
        if self.lie_countdown > 0 and "red" in query.lower():
            self.lie_countdown -= 1
            return super().box(frame, "yellow cup")
        return super().box(frame, query)


def test_verifier_contradiction_is_perception_class(rig, monkeypatch):
    cfg, mock, robot = rig
    orch = LyingOrchestrator(mock)
    agent = Agent(robot, orch, cfg, name="lie")
    # Arm the lie right after the place skill executes, so the very next
    # grounding of the red block (the postcondition check) is wrong.
    original_execute = type(agent.registry).execute

    def patched(self, ctx, name, args, supplied=None):
        # `supplied` is the executor's own channel for a position a look already
        # produced. It is kept out of the planner's schema on purpose — the model
        # names objects, it does not write coordinates — so it travels beside
        # `args` rather than inside them.
        result = original_execute(self, ctx, name, args, supplied)
        if name == "place":
            orch.lie_countdown = 1
        return result

    monkeypatch.setattr(type(agent.registry), "execute", patched)
    summary = agent.run("put the red block in the blue bowl")
    assert summary["status"] == "succeeded"
    fs = failures(agent.log.dir)
    rejected = [r for r in read_journal(agent.log.dir)
                if r["kind"] == "grounding_rejected"]
    # Two acceptable fates for a transient lying detection, in order of
    # preference: the identity defense refuses it at grounding (the from-action
    # jump limit — the lie never even perturbs the episode), or it slips in and
    # the independent re-check surfaces the contradiction as perception-class.
    assert rejected or any(cls == "perception" for _, cls in fs)
    if not rejected:
        # Only the slipped-through path produces an override record.
        assert "perception_override" in kinds(agent.log.dir)
    assert mock.supporting_container("red_block") == "blue_bowl"


def _xy(motion_line: str) -> tuple[float, float]:
    inner = motion_line.split("[", 1)[1].split("]", 1)[0]
    parts = [float(x) for x in inner.split(",")]
    return parts[0], parts[1]


def test_a_missed_grasp_is_retried_with_a_fresh_look_not_a_nudged_aim(rig, tmp_path):
    """What a retry may change, and what it may not.

    The old answer to a missed grasp was a 20 mm spiral and a learned offset.
    On a 30 mm block a 20 mm nudge moves the aim from the centre to the edge —
    it converts a near miss into a certain one — and the learned version
    crossed between objects (the blue block inherited the orange block's
    +20 mm). So a retry now buys a FRESH look and a different grasp strategy,
    and the aim stays where perception put it.
    """
    cfg, mock, robot = rig
    mock.inject.grasp_misses = 1
    agent = Agent(robot, ScriptedOrchestrator(mock), cfg, name="retry")
    assert agent.run("put the red block in the blue bowl")["status"] == "succeeded"

    journal = read_journal(agent.log.dir)
    retries = [r for r in journal if r["kind"] == "program_edit"
               and r["edit"]["type"] == "retry_step"]
    assert retries, "a missed grasp must be retried"
    for r in retries:
        args = r["edit"].get("new_args") or {}
        assert "dx" not in args and "dy" not in args, args
    # A fresh look precedes the retry.
    assert any(r["kind"] == "program_edit"
               and r["edit"]["type"] == "insert_steps"
               and any(s["skill"] == "perceive" for s in r["edit"]["steps"])
               for r in journal)
    # And nothing was applied to the aim from memory either.
    from heron.memory import MemoryStore  # noqa: PLC0415

    assert MemoryStore(cfg.memory_dir).grasp_offset("the red block") == (0.0, 0.0)


def test_budget_abort_is_graceful(rig):
    cfg, mock, robot = rig
    cfg.budgets.max_steps_executed = 2  # not enough to finish
    agent = Agent(robot, ScriptedOrchestrator(mock), cfg, name="budget")
    summary = agent.run("put the red block in the blue bowl")
    assert summary["status"] in ("aborted", "failed")
    assert "budget_abort" in kinds(agent.log.dir) or "goal_check" in kinds(agent.log.dir)


def test_vla_primary_rewrites_the_catalog_and_default_stays_off(rig):
    """Pigey is an orchestrator over TiPToP + pi0.5 — the strong low level does
    the manipulation. Measured with vla as a repair-time fallback instead: 24
    invocations on a LIBERO-PRO sweep, and on every spatial/goal episode it
    arrived after the scripted primitives had burned the budget, as
    'ran 0.0s, stopped by environment ended'. vla_primary inverts the order;
    it must stay off by default because only rigs with a policy backend can
    honour it."""
    cfg, mock, robot = rig
    agent = Agent(robot, ScriptedOrchestrator(mock), cfg, name="cat")
    assert "MANIPULATION POLICY" not in agent._catalog()
    cfg.vla_primary = True
    assert "MANIPULATION POLICY" in agent._catalog()
    assert "FALLBACK" in agent._catalog()


def test_vla_verbs_scopes_the_routing_and_defers_to_vla_primary(rig):
    """The blanket inversion measured worse than grounding-first (3/18 vs 6/18)
    — the policy follows perturbed instruction text into swap traps — but the
    articulation verbs are where the primitives have never once succeeded on
    LIBERO-PRO. vla_verbs routes only those verbs to the policy; pick/place
    stay scripted; vla_primary, when set, wins outright."""
    cfg, mock, robot = rig
    agent = Agent(robot, ScriptedOrchestrator(mock), cfg, name="verbs")
    assert "ROUTING RULE" not in agent._catalog()
    cfg.vla_verbs = ["open", "close", "push"]
    doc = agent._catalog()
    assert "ROUTING RULE" in doc and "open, close, push" in doc
    assert "MANIPULATION POLICY" not in doc          # not the blanket inversion
    cfg.vla_primary = True                           # blanket wins when both set
    doc = agent._catalog()
    assert "MANIPULATION POLICY" in doc and "ROUTING RULE" not in doc
    cfg.vla_primary = False
    cfg.disable_skills = ["vla"]                     # no policy backend, no rule
    assert "ROUTING RULE" not in agent._catalog()
