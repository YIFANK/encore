"""Identity survives a grasp. A sighting's authority does not.

The defect these pin was measured in the MuJoCo twin on `put the red block on
the white plate`, in a scene holding two red blocks. The agent moved one of
them to 18 mm from the plate; the arm that had just placed it then stood in the
overhead camera's way; thirty seconds later the re-identification guard expired
and `red_block` silently rebound to the OTHER red block, 0.34 m away. Every
verification after that was UNKNOWN, the episode ended unverified, and since
only verified episodes teach, the skill library stayed empty on any scene with
duplicate objects.
"""
from __future__ import annotations

import time

import numpy as np
import pytest

from heron.agent import Agent
from heron.config import HeronConfig
from heron.episode import read_journal
from heron.orchestrator.scripted import ScriptedOrchestrator, project
from heron.robot.mock import MockRobot, standard_scene
from heron.robot.safety import SafeRobot
from heron.skills.primitives import pick, place
from heron.skills.sensing import (BLOCKED_MAX_JUMP_M, REID_MAX_JUMP_M, REID_TRUST_WINDOW_S,
                                  _mark_blocked, blocked_from_view,
                                  expected_xy, forget_the_look, ground_entity)
from heron.types import PredicateSpec, Value
from heron.verify import _on


@pytest.fixture
def rig(tmp_path):
    cfg = HeronConfig()
    cfg.episodes_dir = str(tmp_path / "ep")
    cfg.memory_dir = str(tmp_path / "mem")
    cfg.skills_dir = str(tmp_path / "sk")
    mock = MockRobot()
    standard_scene(mock)
    agent = Agent(SafeRobot(mock, cfg.safety), ScriptedOrchestrator(mock), cfg, name="ident")
    agent.belief.track("red_block").description = "the red block"
    agent.belief.track("green_block").description = "the green block"
    agent.belief.track("blue_bowl").description = "the blue bowl"
    return cfg, mock, agent


class _TwoLookalikes(ScriptedOrchestrator):
    """Offers the twin FIRST and the real target second, in points and boxes.

    Position is derived from the box, so a lookalike test that only duplicated
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


def _placed(agent, entity: str, xyz, ago: float = 0.0) -> None:
    """Write the belief the way `place` writes it: dead reckoning, not a sighting."""
    agent.belief.update_track(entity, xyz_base=tuple(xyz), confidence=0.6,
                              held_by=None, from_sighting=False)
    agent.belief.track(entity).t = time.time() - ago


def _sighted(agent, entity: str, xyz, ago: float = 0.0) -> None:
    agent.belief.update_track(entity, xyz_base=tuple(xyz), confidence=0.9,
                              from_sighting=True)
    agent.belief.track(entity).t = time.time() - ago


# -- what the expectation is worth ------------------------------------------

def test_a_position_we_put_the_object_at_does_not_expire(rig):
    cfg, mock, agent = rig
    _placed(agent, "red_block", mock.gt_xyz("red_block"), ago=REID_TRUST_WINDOW_S * 3)
    assert expected_xy(agent.belief.track("red_block")) is not None


def test_a_sighting_does_expire(rig):
    cfg, mock, agent = rig
    _sighted(agent, "red_block", mock.gt_xyz("red_block"), ago=REID_TRUST_WINDOW_S * 3)
    assert expected_xy(agent.belief.track("red_block")) is None


def test_invalidating_a_belief_releases_the_identity(rig):
    """The escape hatch: a skill that displaced the object says so, and then the
    object is found wherever it now is."""
    cfg, mock, agent = rig
    _placed(agent, "red_block", mock.gt_xyz("red_block"))
    agent.belief.invalidate_entity("red_block", reason="dropped")
    assert expected_xy(agent.belief.track("red_block")) is None


# -- what that buys, in grounding -------------------------------------------

def test_a_placed_block_keeps_its_identity_long_after_the_window(rig):
    cfg, mock, agent = rig
    agent.ctx.grounder = _TwoLookalikes(mock)
    truth = mock.gt_xyz("red_block")
    _placed(agent, "red_block", truth, ago=REID_TRUST_WINDOW_S * 3)

    found, note = ground_entity(agent.ctx, "red_block")
    assert found, note
    got = np.asarray(agent.belief.track("red_block").xyz_base)
    twin = mock.gt_xyz("green_block")
    assert float(np.linalg.norm(got[:2] - truth[:2])) < 0.03, (got, truth)
    assert float(np.linalg.norm(got[:2] - twin[:2])) > 0.10, "rebound onto the twin"


def test_an_old_sighting_still_stops_pinning_the_association(rig):
    """The other half of the rule, so this is a distinction and not a blanket."""
    cfg, mock, agent = rig
    agent.ctx.grounder = _TwoLookalikes(mock)
    _sighted(agent, "red_block", mock.gt_xyz("red_block"), ago=REID_TRUST_WINDOW_S * 3)

    found, _ = ground_entity(agent.ctx, "red_block")
    assert found
    got = np.asarray(agent.belief.track("red_block").xyz_base)
    twin = mock.gt_xyz("green_block")
    assert float(np.linalg.norm(got[:2] - twin[:2])) < 0.05, \
        "with the sighting expired, the detector's own ranking should stand"


def test_a_distant_lookalike_is_refused_however_long_ago_we_placed_it(rig):
    """The case with no alternatives to choose between: one detection, far away.

    This is what the twin actually hit — the block we placed was under the
    gripper and invisible, so the detector offered only its twin. Rebinding
    there is not a re-association, it is losing the object.
    """
    cfg, mock, agent = rig
    far = np.array([-0.25, 0.10, 0.02])
    mock.teleport("green_block", far)
    agent.ctx.grounder = ScriptedOrchestrator(mock, lies={"red block": tuple(far)})
    _placed(agent, "red_block", mock.gt_xyz("red_block"), ago=REID_TRUST_WINDOW_S * 3)

    found, note = ground_entity(agent.ctx, "red_block")
    assert not found, note
    assert "we put it there" in note, note
    assert float(np.linalg.norm(
        np.asarray(agent.belief.track("red_block").xyz_base[:2]) - far[:2])) > 0.2


# -- what `place` leaves behind ---------------------------------------------

def _hold(mock, agent, entity: str, arm: str = "left") -> None:
    mock._arms[arm].holding = entity
    mock._arms[arm].gripper_width = 0.03
    agent.belief.update_track(entity, xyz_base=tuple(mock.get_cartesian(arm)),
                              confidence=0.9, held_by=arm)


def test_place_records_where_the_object_went_not_where_the_tool_went(rig):
    cfg, mock, agent = rig
    _sighted(agent, "blue_bowl", mock.gt_xyz("blue_bowl"))
    _hold(mock, agent, "red_block")
    agent.belief.track("red_block").carry_offset = (0.02, 0.0, 0.0)

    place(agent.ctx, entity="red_block", target="blue_bowl", arm="left", contact=False)
    aim = np.asarray(agent.ctx.last_place["aim"], dtype=float)
    got = np.asarray(agent.belief.track("red_block").xyz_base, dtype=float)
    assert float(np.linalg.norm(got[:2] - aim[:2])) < 1e-6, (got, aim)
    # ...and specifically not the point the TOOL was sent to, which is the aim
    # displaced by the carry offset.
    assert float(np.linalg.norm(got[:2] - (aim[:2] - np.array([0.02, 0.0])))) > 0.01


# -- what a carry offset is allowed to be -----------------------------------

def test_a_carry_offset_wider_than_the_object_is_not_a_carry_offset(rig):
    """Measured in the twin against MuJoCo ground truth: the object sat 0-7 mm
    from the tool centre while this read 28-42 mm. The difference was the arm
    not reaching the point it was sent to — an error `place` repeats rather than
    cancels, so subtracting it doubled the miss (63 mm against 31 mm)."""
    cfg, mock, agent = rig
    _sighted(agent, "red_block", mock.gt_xyz("red_block"))
    result = pick(agent.ctx, entity="red_block", arm="left", dx=0.04, contact=False)
    assert result.ok and mock._arms["left"].holding == "red_block"
    assert agent.belief.track("red_block").carry_offset is None
    rejected = [r for r in read_journal(agent.log.dir) if r["kind"] == "carry_offset_rejected"]
    assert rejected, "silently dropping it would hide the arm's tracking error"


def test_an_offset_the_object_can_actually_ride_at_is_kept(rig):
    cfg, mock, agent = rig
    _sighted(agent, "red_block", mock.gt_xyz("red_block"))
    result = pick(agent.ctx, entity="red_block", arm="left", dx=0.01, contact=False)
    assert result.ok and mock._arms["left"].holding == "red_block"
    carry = agent.belief.track("red_block").carry_offset
    assert carry is not None and abs(carry[0] + 0.01) < 5e-3, carry


def test_the_bound_is_the_objects_own_width_not_a_constant(rig):
    """The same 30 mm offset is honest on a 70 mm cup and impossible on a 40 mm
    block. That is what keeps the rim grasp of a wide vessel — the case this
    mechanism was built for — working."""
    cfg, mock, agent = rig
    agent.belief.track("yellow_cup").description = "the yellow cup"
    _sighted(agent, "yellow_cup", mock.gt_xyz("yellow_cup"))
    assert pick(agent.ctx, entity="yellow_cup", arm="right", dx=0.03, contact=False).ok
    assert agent.belief.track("yellow_cup").carry_offset is not None

    _sighted(agent, "red_block", mock.gt_xyz("red_block"))
    assert pick(agent.ctx, entity="red_block", arm="left", dx=0.03, contact=False).ok
    assert agent.belief.track("red_block").carry_offset is None


# -- what the verifier does when the arm is in the way ----------------------

class _BlockedOverhead(ScriptedOrchestrator):
    """The fixed cameras cannot see the block; a wrist camera can.

    Not a contrivance: `place` finishes with the tool hovering over the target,
    so the gripper that made the placement is between the overhead camera and
    the thing it put down.
    """

    @staticmethod
    def _blind(frame, query) -> bool:
        return not frame.camera.endswith("_wrist") and "red" in query.lower()

    def points(self, frame, query):
        return [] if self._blind(frame, query) else super().points(frame, query)

    def point(self, frame, query):
        return None if self._blind(frame, query) else super().point(frame, query)

    def boxes(self, frame, query):
        return [] if self._blind(frame, query) else super().boxes(frame, query)

    def box(self, frame, query):
        return None if self._blind(frame, query) else super().box(frame, query)


def test_on_looks_from_the_wrist_when_the_arm_blocks_the_overhead_view(rig):
    cfg, mock, agent = rig
    agent.ctx.grounder = _BlockedOverhead(mock)
    bowl = mock.gt_xyz("blue_bowl")
    landed = np.array([bowl[0], bowl[1], bowl[2] + 0.02])
    mock.objects["red_block"].xyz = landed
    _sighted(agent, "blue_bowl", bowl)
    _placed(agent, "red_block", landed)      # where `place` sent it

    verdict = _on(agent.ctx, PredicateSpec(name="on", args=["red_block", "blue_bowl"]))
    looks = [r for r in read_journal(agent.log.dir) if r["kind"] == "on_active_look"]
    assert looks and looks[0]["ok"], looks
    assert looks[0]["from_action"] is True, "it should aim at where the place put it"
    assert verdict.value is not Value.UNKNOWN, verdict.note


# -- a blocked look does not merely fail; with a lookalike it lies ----------

class _DecoyOverhead(ScriptedOrchestrator):
    """The overhead camera, gripper in the way, answers with the OTHER red block.

    This is the shape of the real reply. A blocked view does not come back
    empty when a duplicate is on the table — it comes back confident, correct
    as a detection, and about the wrong object.
    """

    def __init__(self, mock, decoy) -> None:
        super().__init__(mock)
        self.decoy = np.asarray(decoy, dtype=float)

    def _decoying(self, frame, query) -> bool:
        return not frame.camera.endswith("_wrist") and "red" in query.lower()

    def point(self, frame, query):
        if self._decoying(frame, query):
            px = project(frame, self.decoy)
            return (*px, 0.9) if px else None
        return super().point(frame, query)

    def points(self, frame, query):
        hit = self.point(frame, query)
        return [hit] if hit else []

    def box(self, frame, query):
        if self._decoying(frame, query):
            return None
        return super().box(frame, query)

    def boxes(self, frame, query):
        hit = self.box(frame, query)
        return [hit] if hit else []


def test_a_look_we_knew_was_blocked_does_not_get_the_loose_tolerance(rig):
    """The system saw this coming and went ahead anyway.

    `_clear_the_view` computed that the tool sat on the camera-to-object line,
    tried to step aside, found nowhere reachable, logged `view_blocked_but_stuck`
    — and the look went out regardless. Measured in the twin: the detector
    returned the other red block 115 mm away, inside the 250 mm general
    tolerance, so it was accepted at confidence 0.90. `on` scored FALSE, the
    repair moved that second block onto the plate as well, and the episode was
    recorded as a success with both blocks on the plate and neither of them the
    one the plan had been following.
    """
    cfg, mock, agent = rig
    bowl = mock.gt_xyz("blue_bowl")
    landed = np.array([bowl[0], bowl[1], bowl[2] + 0.02])
    # The measured separation: the other red block, 115 mm away — comfortably
    # inside the general tolerance, which is why it was believed.
    decoy = landed + np.array([0.115, 0.0, 0.0])
    assert BLOCKED_MAX_JUMP_M < 0.115 < REID_MAX_JUMP_M, \
        "the decoy has to sit in the gap between the two tolerances or this proves nothing"
    agent.ctx.grounder = _DecoyOverhead(mock, decoy)
    mock.objects["red_block"].xyz = landed
    _sighted(agent, "blue_bowl", bowl)
    _placed(agent, "red_block", landed)

    _mark_blocked(agent.ctx, ["red_block"])
    found, note = ground_entity(agent.ctx, "red_block")
    assert not found, f"a lookalike behind a blocked view must not answer: {note}"
    assert tuple(agent.belief.track("red_block").xyz_base) == tuple(landed), \
        "and it must not overwrite the position `place` wrote — that is the one we trust"


def test_a_wide_support_under_the_tool_is_still_seen(rig):
    """The blunt version of this refused every reading from an obstructed look,
    and that was worse. The tool sat over the middle of a 140 mm tray, whose
    sightline is blocked at the centroid and clear everywhere else; `in` then had
    no container to compare against and stayed UNKNOWN to the end of the episode.
    Distance from the prior is the discriminator — obstruction only sets how much
    of it to allow."""
    cfg, mock, agent = rig
    bowl = mock.gt_xyz("blue_bowl")
    _placed(agent, "blue_bowl", bowl)
    _mark_blocked(agent.ctx, ["blue_bowl"])
    found, note = ground_entity(agent.ctx, "blue_bowl")
    assert found, f"it is right where we believe it is; the tool hides its middle: {note}"


def test_the_verifier_goes_to_the_wrist_instead_of_believing_the_decoy(rig):
    cfg, mock, agent = rig
    bowl = mock.gt_xyz("blue_bowl")
    landed = np.array([bowl[0], bowl[1], bowl[2] + 0.02])
    agent.ctx.grounder = _DecoyOverhead(mock, landed + np.array([0.115, 0.0, 0.0]))
    mock.objects["red_block"].xyz = landed
    _sighted(agent, "blue_bowl", bowl)
    _placed(agent, "red_block", landed)
    _mark_blocked(agent.ctx, ["red_block"])

    verdict = _on(agent.ctx, PredicateSpec(name="on", args=["red_block", "blue_bowl"]))
    assert verdict.value is not Value.FALSE, \
        f"a confident FALSE off an obstructed view is what moved a second block: {verdict.note}"
    looks = [r for r in read_journal(agent.log.dir) if r["kind"] == "on_active_look"]
    assert looks, "it should have gone and looked from somewhere it could see"


def test_nowhere_to_step_aside_is_recorded_rather_than_shrugged_off(rig):
    """The mark is what makes the refusal possible; without it the caller cannot
    tell an obstructed look from a clean one."""
    cfg, mock, agent = rig
    agent.ctx.scratch.pop("view_blocked", None)
    assert not blocked_from_view(agent.ctx, "red_block")
    _mark_blocked(agent.ctx, ["red_block", "blue_bowl"])
    assert blocked_from_view(agent.ctx, "red_block")
    assert blocked_from_view(agent.ctx, "blue_bowl")
    forget_the_look(agent.ctx, "the arm moved")
    assert not blocked_from_view(agent.ctx, "red_block"), \
        "a mark outlives its look only until the world changes"


# -- a success that is invisible by design ------------------------------------

def test_containment_survives_the_clearing_of_last_place(rig):
    """Two LIBERO episodes whose benchmark said done ended failed/aborted with
    in() at UNKNOWN 0.40 to the last line. The object was invisible precisely
    BECAUSE it was in the basket: the rim blocks the overhead camera, and the
    action-derived inference that should have covered this read `ctx.last_place`
    — which the agent clears after every place step, so at goal time it was
    always empty. The durable record is the belief: `place` writes the track at
    the aim with from_sighting=False, and that is what the inference reads now.
    """
    from heron.verify import _in

    cfg, mock, agent = rig
    bowl = mock.gt_xyz("blue_bowl")
    _sighted(agent, "blue_bowl", bowl)
    # The block was placed into the bowl and is now invisible: hide it from the
    # detector entirely (inside a container, under the rim).
    agent.ctx.grounder = ScriptedOrchestrator(mock)
    mock.objects["red_block"].xyz = np.array([bowl[0], bowl[1], bowl[2] + 0.01])
    _placed(agent, "red_block", [bowl[0], bowl[1], bowl[2] + 0.01])   # dead reckoning
    agent.ctx.grounder.lies = {"red block": (9.9, 9.9, 9.9)}          # never seen again
    agent.ctx.last_place = {}                                          # as the agent leaves it

    verdict = _in(agent.ctx, PredicateSpec(name="in", args=["red_block", "blue_bowl"]))
    assert verdict.value is Value.TRUE, (verdict.value, verdict.note)
    assert "placement action" in verdict.note


def test_a_sighted_position_gets_no_containment_credit(rig):
    """The inference is about what WE did, not what we saw. A stale sighting
    inside the container's footprint is not a placement record."""
    from heron.verify import _in

    cfg, mock, agent = rig
    bowl = mock.gt_xyz("blue_bowl")
    _sighted(agent, "blue_bowl", bowl)
    agent.ctx.grounder = ScriptedOrchestrator(mock)
    _sighted(agent, "red_block", [bowl[0], bowl[1], bowl[2] + 0.01])  # sighting, not action
    agent.ctx.grounder.lies = {"red block": (9.9, 9.9, 9.9)}
    agent.ctx.last_place = {}

    verdict = _in(agent.ctx, PredicateSpec(name="in", args=["red_block", "blue_bowl"]))
    assert verdict.value is not Value.TRUE, verdict.note


def test_looking_into_a_container_clears_its_rim(rig):
    """The wrist photographed the basket's wall from point-blank: aimed with the
    table as its floor, the reachable-height search lowered the camera to rim
    level. The look has to clear the rim, which is at the container's height."""
    from heron.skills.sensing import INSPECT_MIN_ABOVE_M, inspect

    cfg, mock, agent = rig
    moves = []
    original = mock.move_cartesian

    def spy(arm, xyz, seconds=1.0):
        moves.append(np.asarray(xyz, float).copy())
        return original(arm, xyz, seconds=seconds)

    mock.move_cartesian = spy
    rim_z = 0.07
    result = inspect(agent.ctx, x=0.05, y=-0.15, arm="right", above_z=rim_z)
    assert result.ok, result.error
    assert moves and moves[-1][2] >= rim_z + INSPECT_MIN_ABOVE_M - 1e-9, \
        f"looked from {moves[-1][2]:.3f}, below the rim it had to clear"


# -- which object, not where ---------------------------------------------------

class _WrongTwinButHonestEyes(ScriptedOrchestrator):
    """Grounds "the red block" onto the twin that is NOT in the bowl, and
    answers the visual question correctly. Exactly what round 0 of the training
    loop measured: 0.95-confidence point on the wrong cube, and a VQA that
    volunteered "there are two red blocks, and one is inside the grey tray"."""

    def __init__(self, mock, decoy_xyz) -> None:
        super().__init__(mock)
        self.decoy = np.asarray(decoy_xyz, dtype=float)
        self.asked: list[str] = []

    def point(self, frame, query: str):
        if "red" in query.lower():
            px = project(frame, self.decoy)
            return (*px, 0.95) if px else None
        return super().point(frame, query)

    def box(self, frame, query: str):
        if "red" in query.lower():
            return None          # force the point path, as the twin scene does
        return super().box(frame, query)

    def vqa(self, frames, question: str):
        self.asked.append(question)
        return Value.TRUE, 1.0, "a red block is sitting in the blue bowl"


def test_containment_asks_the_image_before_measuring_the_wrong_twin(rig):
    """35 of 76 perfect demonstrations were thrown away by this.

    The scene held two identical red cubes with one of them in the container.
    The detector grounded the other at 0.95 confidence and the geometry then
    correctly reported IT 103 mm away — a right measurement of the wrong
    object, and there were no false positives anywhere, so nothing downstream
    could tell that half the training set had been discarded for a reason that
    had nothing to do with the task.
    """
    from heron.verify import _in

    cfg, mock, agent = rig
    bowl = mock.gt_xyz("blue_bowl")
    decoy = np.array([bowl[0] + 0.30, bowl[1] - 0.10, bowl[2]])
    _sighted(agent, "blue_bowl", bowl)
    mock.objects["red_block"].xyz = np.array([bowl[0], bowl[1], bowl[2] + 0.01])
    agent.ctx.grounder = _WrongTwinButHonestEyes(mock, decoy)

    verdict = _in(agent.ctx, PredicateSpec(name="in", args=["red_block", "blue_bowl"]))
    assert verdict.value is Value.TRUE, (verdict.value, verdict.note)
    assert "witness-view VQA" in verdict.note
    assert any("any red block" in q for q in agent.ctx.grounder.asked), \
        f"the question has to be existential, not about one instance: {agent.ctx.grounder.asked}"


class _NoOpinion(ScriptedOrchestrator):
    """Sees, but will not say. `_vqa_primary` documents this case — unavailable
    or unsure — and it is the one where the measurement has to survive."""

    def vqa(self, frames, question: str):
        return Value.UNKNOWN, 0.0, "cannot tell from these views"


def test_containment_still_measures_when_the_eyes_have_no_opinion(rig):
    """The geometry is the fallback, not the discard pile. A grounder that
    cannot answer the question must leave the measurement in place."""
    from heron.verify import _in

    cfg, mock, agent = rig
    bowl = mock.gt_xyz("blue_bowl")
    _sighted(agent, "blue_bowl", bowl)
    mock.objects["red_block"].xyz = np.array([bowl[0], bowl[1], bowl[2] + 0.01])
    agent.ctx.grounder = _NoOpinion(mock)

    verdict = _in(agent.ctx, PredicateSpec(name="in", args=["red_block", "blue_bowl"]))
    assert verdict.value is Value.TRUE, (verdict.value, verdict.note)
    assert "|dxy|" in verdict.note, "the geometric path should have produced this"
