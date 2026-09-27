"""Fixes for the tabletop suites, where every episode scored zero.

Each test pins a defect that was measured on a real episode, not imagined.
"""
from __future__ import annotations

import numpy as np
import pytest

from heron.agent import Agent
from heron.config import HeronConfig
from heron.program import EditType, Step, TaskProgram
from heron.robot.mock import MockRobot, standard_scene
from heron.robot.safety import SafeRobot
from heron.types import EntityDecl, PredicateSpec, Value
from heron.verify import ON_XY_MAX, _on

from test_agent_loop import ScriptedOrchestrator


@pytest.fixture
def rig(tmp_path):
    cfg = HeronConfig()
    cfg.episodes_dir = str(tmp_path / "ep")
    cfg.memory_dir = str(tmp_path / "mem")
    cfg.skills_dir = str(tmp_path / "sk")
    mock = MockRobot()
    standard_scene(mock)
    agent = Agent(SafeRobot(mock, cfg.safety), ScriptedOrchestrator(mock), cfg, name="tt")
    return cfg, mock, agent


def test_on_does_not_get_more_forgiving_as_the_support_gets_wider():
    """A plate 14 cm across still means 'centred', not 'anywhere on it'."""
    assert ON_XY_MAX <= 0.04, "measured: the benchmark's own On() allows 30 mm"


def test_an_object_on_the_lip_is_not_on_the_support(rig):
    cfg, mock, agent = rig
    for eid, desc in (("red_block", "the red block"), ("blue_bowl", "the blue bowl")):
        agent.belief.track(eid).description = desc
    bowl = np.asarray(mock.gt_xyz("blue_bowl"))
    # Actually put the block on the bowl's lip, 55 mm off centre — inside the old
    # box-scaled threshold, outside the tolerance the benchmark really applies.
    mock.objects["red_block"].xyz = np.array([bowl[0] + 0.055, bowl[1], bowl[2] + 0.02])
    verdict = _on(agent.ctx, PredicateSpec(name="on", args=["red_block", "blue_bowl"]))
    assert verdict.value is Value.FALSE, verdict.note
    assert "dxy" in verdict.note


def test_an_object_still_in_the_gripper_is_not_resting_on_anything(rig):
    cfg, mock, agent = rig
    for eid, desc in (("red_block", "the red block"), ("blue_bowl", "the blue bowl")):
        agent.belief.track(eid).description = desc
    bowl = np.asarray(mock.gt_xyz("blue_bowl"))
    # Centred over the bowl, but still in the gripper: geometry alone would say
    # yes, and it is exactly the moment before a placement either works or does not.
    mock.objects["red_block"].xyz = np.array([bowl[0], bowl[1], bowl[2] + 0.01])
    mock._arms["left"].holding = "red_block"      # actually in the gripper
    mock._arms["left"].gripper_width = 0.03       # closed ON something, not on air
    agent.belief.track("red_block").held_by = "left"
    verdict = _on(agent.ctx, PredicateSpec(name="on", args=["red_block", "blue_bowl"]))
    assert verdict.value is Value.FALSE, verdict.note


def test_place_without_a_grasp_says_to_re_pick(rig):
    """The old message sent the repair loop off inventing arm names."""
    cfg, mock, agent = rig
    from heron.skills.primitives import place

    agent.belief.track("red_block").description = "the red block"
    agent.belief.update_track("red_block", xyz_base=(0.05, 0.10, 0.02), confidence=0.9)
    agent.belief.track("blue_bowl").description = "the blue bowl"
    agent.belief.update_track("blue_bowl", xyz_base=tuple(mock.gt_xyz("blue_bowl")), confidence=0.9)

    result = agent.registry.execute(agent.ctx, "place",
                                    {"entity": "red_block", "target": "blue_bowl", "arm": "nonexistent"})
    assert not result.ok
    assert "Re-run pick" in result.error, result.error
    assert "left" in result.error and "right" in result.error, result.error


def test_a_goal_already_true_is_refused(rig):
    """Asked for something it cannot express, the planner reached for a
    condition that was already satisfied and declared victory."""
    cfg, mock, agent = rig
    program = TaskProgram(
        goal="open the drawer",
        steps=[Step(id="s1", skill="perceive", args={})],
        goal_predicates=[PredicateSpec(name="gripper_empty", args=["left"])],
    )
    agent._reject_vacuous_goal(program)
    assert program.aborted, "a goal true before acting must not be accepted"
    assert "cannot be expressed" in (program.abort_reason or "")


def test_a_real_goal_is_left_alone(rig):
    cfg, mock, agent = rig
    agent.belief.track("red_block").description = "the red block"
    agent.belief.track("blue_bowl").description = "the blue bowl"
    program = TaskProgram(
        goal="put the block in the bowl",
        steps=[Step(id="s1", skill="perceive", args={})],
        goal_predicates=[PredicateSpec(name="in", args=["red_block", "blue_bowl"])],
    )
    agent._reject_vacuous_goal(program)
    assert not program.aborted


def test_the_verifier_is_no_looser_than_the_benchmark():
    """A test looser than the one grading it can only manufacture false successes."""
    assert ON_XY_MAX <= 0.030, "LIBERO's On() requires the centres within 30 mm"


def test_a_flat_support_gets_a_lower_release(rig):
    """A plate has no walls: a 4 cm drop rolls the object and shoves the plate."""
    cfg, mock, agent = rig
    from heron.skills.primitives import DROP_CLEARANCE, FLAT_DROP_CLEARANCE, _is_flat_support

    assert FLAT_DROP_CLEARANCE < DROP_CLEARANCE
    agent.belief.track("blue_bowl").description = "the blue bowl"
    agent.belief.track("plate").description = "the white dinner plate"
    assert _is_flat_support(agent.ctx, "plate")
    assert not _is_flat_support(agent.ctx, "blue_bowl"), "a bowl has walls to catch a drop"


def _ctx(robot):
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext

    return SkillContext(robot=robot, belief=None, grounder=None, cfg=HeronConfig(),
                        log=EpisodeLogger("/tmp/heron-test-episodes", "descend"))


def test_descent_drops_fast_to_the_expected_height_then_feels():
    """25 blocking round-trips from the hover is what made the arm stutter down.

    With depth we know where the surface is, so the steps only have to cover the
    error in that estimate.
    """
    import numpy as np

    from heron.skills.primitives import CONTACT_PROBE_M, CONTACT_STEP_M, descend_to_contact

    moves = []

    class Robot:
        arms = ["right"]

        def move_cartesian(self, arm, xyz, seconds=1.0):
            moves.append(float(np.asarray(xyz)[2]))

        def get_external_effort(self, arm):
            return 0.0                      # never "touches": count the whole descent

    ctx = _ctx(Robot())
    z_start, z_expected, floor = 0.125, 0.0, -0.02
    descend_to_contact(ctx, "right", 0.35, 0.18, z_start, floor, z_expected=z_expected)

    assert moves, "nothing was commanded"
    first_drop = z_start - moves[0]
    assert first_drop > 0.05, f"first move only descended {first_drop * 1000:.0f} mm"
    assert moves[0] == pytest.approx(z_expected + CONTACT_PROBE_M, abs=1e-9)
    stepped = len(moves) - 1
    assert stepped <= CONTACT_PROBE_M / CONTACT_STEP_M + 4, \
        f"{stepped} probing steps — the point was to stop feeling the whole way"


def test_descent_without_depth_still_feels_all_the_way():
    """A rig with no depth has no expected height, and must keep the old
    behaviour rather than dropping blind onto an object of unknown size."""
    import numpy as np

    from heron.skills.primitives import descend_to_contact

    moves = []

    class Robot:
        arms = ["right"]

        def move_cartesian(self, arm, xyz, seconds=1.0):
            moves.append(float(np.asarray(xyz)[2]))

        def get_external_effort(self, arm):
            return 0.0

    ctx = _ctx(Robot())
    descend_to_contact(ctx, "right", 0.35, 0.18, 0.125, -0.02, z_expected=None)
    drops = [0.125 - moves[0]] + [moves[i] - moves[i + 1] for i in range(len(moves) - 1)]
    assert max(drops) < 0.01, "no step may be a blind drop when the height is unknown"


def test_an_open_gripper_is_empty_however_hard_it_pushes():
    """The real rig reads 0.080 m and 10.5 N with the jaws wide open, which the
    original width-and-effort test scored as holding something — so every pick
    was refused for a precondition that could never clear."""
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.types import PredicateSpec, Value
    from heron.verify import check

    class Robot:
        arms = ["right"]

        def __init__(self, width, effort):
            self._g = {"width_m": width, "effort": effort}

        def get_gripper(self, arm):
            return dict(self._g)

    from heron.belief import BeliefStore

    def verdict(width, effort):
        ctx = SkillContext(robot=Robot(width, effort), belief=BeliefStore(),
                           grounder=None, cfg=HeronConfig(),
                           log=EpisodeLogger("/tmp/heron-test-episodes", "grip"))
        return check(ctx, PredicateSpec(name="gripper_empty", args=["right"]))[0]

    assert verdict(0.080, 10.5) is Value.TRUE, "wide open, measured on the rig"
    assert verdict(0.021, 84.0) is Value.FALSE, "holding the block, measured on the rig"
    assert verdict(0.0, 0.05) is Value.TRUE, "closed on nothing"
    assert verdict(0.020, 3.0) is Value.FALSE, "the mock backend's holding numbers"





def test_an_uncalibrated_camera_costs_no_api_calls():
    """Grounding queried cam_high and cam_low on every call, doubling the model
    round-trips, while cam_low had neither extrinsics nor a homography and so
    could never contribute a position."""
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.sensing import _usable_cameras

    cfg = HeronConfig()
    cfg.cameras["cam_high"].homography_file = "calibration/cam_high_homography.npz"
    cfg.cameras["cam_low"].homography_file = None
    cfg.cameras["cam_low"].extrinsics_file = None

    class Robot:
        cameras = ["cam_high", "cam_low"]

    ctx = SkillContext(robot=Robot(), belief=None, grounder=None, cfg=cfg,
                       log=EpisodeLogger("/tmp/heron-test-episodes", "cams"))
    assert _usable_cameras(ctx) == ["cam_high"]

    # Calibrate it and it earns its call back.
    cfg.cameras["cam_low"].extrinsics_file = "calibration/cam_low.npz"
    assert _usable_cameras(ctx) == ["cam_high", "cam_low"]


def test_nothing_calibrated_still_tries_rather_than_going_blind():
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.sensing import _usable_cameras

    cfg = HeronConfig()
    for c in cfg.cameras.values():
        c.homography_file = c.extrinsics_file = None

    class Robot:
        cameras = ["cam_high", "cam_low"]

    ctx = SkillContext(robot=Robot(), belief=None, grounder=None, cfg=cfg,
                       log=EpisodeLogger("/tmp/heron-test-episodes", "cams"))
    assert _usable_cameras(ctx) == ["cam_high", "cam_low"]


def test_a_fresh_sighting_is_reused_but_dead_reckoning_is_not():
    """perceive sees the block, pick's precondition asks again a tenth of a
    second later — that cost four model calls and a segmentation. A pose written
    by place is different: the object may be inside a container, which is
    exactly when a close look is needed."""
    from heron.belief import BeliefStore
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.types import PredicateSpec, Value
    from heron.verify import check

    looked = []

    class Grounder:
        def points(self, *a, **k):
            looked.append("points")
            return []

        def point(self, *a, **k):
            looked.append("point")
            return None

    class Robot:
        arms = ["right"]
        cameras = ["cam_high"]

        def capture(self, cam):
            looked.append("capture")
            from heron.types import Frame
            return Frame(camera=cam, rgb=np.zeros((8, 8, 3), np.uint8))

        def get_gripper(self, arm):
            return {"width_m": 0.08, "effort": 10.5}

    cfg = HeronConfig()
    belief = BeliefStore()
    belief.track("cube").description = "a red cube"
    ctx = SkillContext(robot=Robot(), belief=belief, grounder=Grounder(), cfg=cfg,
                       log=EpisodeLogger("/tmp/heron-test-episodes", "sight"))

    belief.update_track("cube", xyz_base=(0.35, 0.18, 0.0), confidence=0.95,
                        from_sighting=True)
    value, _ = check(ctx, PredicateSpec(name="visible", args=["cube"]))
    assert value is Value.TRUE and looked == [], f"re-looked at a fresh sighting: {looked}"

    # Dead reckoning after a place: must actually go and look.
    belief.update_track("cube", xyz_base=(0.5, 0.1, 0.0), confidence=0.9)
    check(ctx, PredicateSpec(name="visible", args=["cube"]))
    assert looked, "a position nobody saw must not stand in for seeing it"


def test_sam_recovers_after_its_cooldown():
    """Muting used to be permanent: enabled went false, which stopped the
    requests, which meant nothing could ever succeed and reset the counter. One
    slow minute disabled masks for the whole episode and grounding quietly fell
    back to the least accurate mode."""
    import time as _time

    from heron.perception import SamSegmenter
    from heron.perception.sam import FAILURES_BEFORE_MUTE

    seg = SamSegmenter("http://127.0.0.1:1", timeout_s=0.5)   # refuses instantly
    for _ in range(FAILURES_BEFORE_MUTE):
        seg.mask(np.zeros((8, 8, 3), np.uint8), (0, 0, 4, 4))
    assert not seg.enabled, "should go quiet after repeated failures"

    seg._muted_until = _time.time() - 1        # pretend the cooldown elapsed
    assert seg.enabled, "must try again rather than stay off forever"


def test_place_can_feel_for_the_surface_when_pick_does_not():
    """Depth quality belongs to the surface: the block reads cleanly, the glossy
    plate scatters 25 mm."""
    cfg = HeronConfig()
    cfg.contact_descend = False
    assert cfg.contact_descend_place is None

    cfg.contact_descend_place = True
    # pick follows contact_descend, place follows its own switch
    assert cfg.contact_descend is False and cfg.contact_descend_place is True


def test_an_unreachable_target_is_refused_by_safety_not_by_the_driver():
    """workspace_z claims 0.40 while a straight-down wrist cannot hold much
    above 0.15 over most of the table. The box is a coarse guard; the envelope
    is curved."""
    from heron.robot.safety import SafeRobot, SafetyViolation

    class Robot:
        arms = ["right"]
        cameras = []

        def get_cartesian(self, arm):
            return np.array([0.3, 0.0, 0.1])

        def reachable(self, arm, xyz):
            return float(np.asarray(xyz)[2]) <= 0.15   # the measured shape

        def move_cartesian(self, arm, xyz, seconds=1.0):
            raise AssertionError("should never have been commanded")

    cfg = HeronConfig().safety
    cfg.workspace_z = (0.01, 0.40)
    safe = SafeRobot(Robot(), cfg)
    with pytest.raises(SafetyViolation, match="envelope, not a box"):
        safe.move_cartesian("right", np.array([0.3, 0.0, 0.30]))


def _wrist_ctx(depth_z=None, cam_ok=True, cam_z=0.20):
    """A rig whose right wrist camera looks straight down from `cam_z`."""
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.types import Frame

    cfg = HeronConfig()
    cfg.cameras["cam_right_wrist"].kind = "wrist"
    cfg.cameras["cam_right_wrist"].arm = "right"

    class Robot:
        arms = ["right"]
        cameras = ["cam_high", "cam_right_wrist"] if cam_ok else ["cam_high"]
        moves = []

        def capture(self, cam):
            h = w = 60
            if depth_z is None:
                return Frame(camera=cam, rgb=np.zeros((h, w, 3), np.uint8))
            # camera 0.20 m above the world origin looking down
            d = np.full((h, w), cam_z - depth_z, np.float32)
            k = np.array([[60.0, 0, 30.0], [0, 60.0, 30.0], [0, 0, 1.0]])
            t = np.array([[1.0, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, cam_z], [0, 0, 0, 1.0]])
            return Frame(camera=cam, rgb=np.zeros((h, w, 3), np.uint8), depth=d,
                         intrinsics=k, t_base_cam=t)

        def move_cartesian(self, arm, xyz, seconds=1.0):
            self.moves.append(np.asarray(xyz, float).copy())

        def get_cartesian(self, arm):
            return np.array([0.0, 0.0, cam_z])

    return SkillContext(robot=Robot(), belief=None, grounder=None, cfg=cfg,
                        log=EpisodeLogger("/tmp/heron-test-episodes", "wrist"))


def test_wrist_camera_measures_the_surface_under_the_tool():
    from heron.skills.primitives import surface_height_below

    ctx = _wrist_ctx(depth_z=0.03)          # something 30 mm tall under the tool
    z, note = surface_height_below(ctx, "right", (0.0, 0.0))
    assert z is not None, note
    assert abs(z - 0.03) < 0.002, f"read {z:.4f}, expected 0.030"


def test_no_wrist_depth_reports_why_instead_of_guessing():
    from heron.skills.primitives import surface_height_below

    z, note = surface_height_below(_wrist_ctx(depth_z=None), "right", (0.0, 0.0))
    assert z is None and "no depth" in note

    z, note = surface_height_below(_wrist_ctx(depth_z=0.03, cam_ok=False), "right", (0.0, 0.0))
    assert z is None and "no wrist camera" in note


def test_a_target_outside_the_probe_radius_is_not_answered():
    """Reading the whole frame would return the height of whatever else is in
    view, which is worse than admitting the target was not seen."""
    from heron.skills.primitives import surface_height_below

    ctx = _wrist_ctx(depth_z=0.03)
    z, note = surface_height_below(ctx, "right", (0.5, 0.5))
    assert z is None and "within" in note


def test_an_impossible_surface_height_is_rejected():
    """Measured on the rig: the wrist camera returned 0.204 m for a plate 2 cm
    tall — about the height of the camera itself. The safety box caught it only
    because the number was large; 0.05 would have passed and dropped the block
    from five centimetres."""
    from heron.skills.primitives import surface_height_below

    # camera high enough that 19 cm still returns valid depth
    ctx = _wrist_ctx(depth_z=0.19, cam_z=0.50)
    ctx.cfg.table_z = 0.0
    z, note = surface_height_below(ctx, "right", (0.0, 0.0))
    assert z is None, f"accepted an impossible surface at {z}"
    assert "outside the" in note and "not a surface" in note


def test_a_plausible_surface_is_still_accepted():
    from heron.skills.primitives import surface_height_below

    ctx = _wrist_ctx(depth_z=0.02)          # a plate, 2 cm
    ctx.cfg.table_z = 0.0
    z, _ = surface_height_below(ctx, "right", (0.0, 0.0))
    assert z is not None and abs(z - 0.02) < 0.002


# -- perception as values -----------------------------------------------------

def _value_ctx(points=None, boxes=None):
    """A context whose detector counts how many times it was asked."""
    import types as _t

    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.types import Frame

    calls = []

    class Grounder:
        def points(self, frame, query):
            calls.append(("points", query))
            return points.get(query, []) if points else []

        def boxes(self, frame, query):
            calls.append(("boxes", query))
            return boxes.get(query, []) if boxes else []

    k = np.array([[60.0, 0, 30.0], [0, 60.0, 30.0], [0, 0, 1.0]])
    t = np.array([[1.0, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 1.0], [0, 0, 0, 1.0]])
    depth = np.full((60, 60), 0.94, np.float32)

    class Robot:
        cameras = ["cam_high"]
        captures = 0

        def capture(self, cam):
            Robot.captures += 1
            return Frame(camera=cam, rgb=np.zeros((60, 60, 3), np.uint8),
                         depth=depth, intrinsics=k, t_base_cam=t)

    cfg = HeronConfig()
    cfg.cameras["cam_high"].homography_file = "x"      # counts as calibrated
    cfg.table_z = 0.0
    ctx = SkillContext(robot=Robot(), belief=None, grounder=Grounder(), cfg=cfg,
                       log=EpisodeLogger("/tmp/heron-test-episodes", "values"))
    return ctx, calls, Robot


def test_locate_returns_a_value_and_writes_no_belief():
    from heron.skills.values import locate

    ctx, calls, _ = _value_ctx(points={"the block": [(30, 30, 0.9)]})
    det = locate(ctx, "the block")
    assert det is not None and det.point == (30, 30) and det.query == "the block"
    assert det.frame is not None, "the frame travels with the detection"
    assert ctx.belief is None, "locate must not need a belief store at all"


def test_to_base_makes_no_detector_call():
    """Turning pixels into metres is arithmetic. It cost a round trip because it
    was buried inside a skill that re-detected first."""
    from heron.skills.values import locate, to_base

    ctx, calls, _ = _value_ctx(points={"the block": [(30, 30, 0.9)]})
    det = locate(ctx, "the block")
    before = len(calls)
    where = to_base(ctx, det)
    assert where is not None
    assert len(calls) == before, f"to_base called the detector: {calls[before:]}"
    assert where.source in ("mask", "box", "single-pixel")


def test_look_locates_several_things_from_one_capture():
    """Three objects from three captures are three slightly different worlds,
    and the arm may have moved between them."""
    from heron.skills.values import look

    ctx, calls, Robot = _value_ctx(points={
        "a": [(20, 30, 0.9)], "b": [(40, 30, 0.9)], "c": [(30, 40, 0.9)]})
    Robot.captures = 0
    seen = look(ctx, ["a", "b", "c"])
    assert set(seen.seen) == {"a", "b", "c"}
    assert Robot.captures == 1, f"captured {Robot.captures} times for one look"
    frames = {id(v.detection.frame) for v in seen.seen.values()}
    assert len(frames) == 1, "every object must be located in the SAME frame"


def test_the_cost_of_a_plan_is_known_before_it_runs():
    """23 detector calls for a two-object task was not a decision anyone made —
    each skill decided in private. A graph can be counted."""
    from heron.skills.dataflow import cost, sort_by_colour

    plan = sort_by_colour({
        "the first red block": "the white plate",
        "the second red block": "the white plate",
        "the first blue block": "the grey tray",
        "the second blue block": "the grey tray",
    })
    assert cost(plan) == 1, "six things, one look, one request"
    assert len([n for n in plan if n.kind == "pick"]) == 4
    assert len([n for n in plan if n.kind == "place"]) == 4


def test_a_plan_that_uses_what_it_never_looked_for_is_rejected():
    from heron.skills.dataflow import Look, Pick, Place, missing_values

    ok = [Look(["a", "b"]), Pick("a"), Place("a", onto="b")]
    assert missing_values(ok) == []

    bad = [Look(["a"]), Pick("a"), Place("a", onto="b")]
    assert missing_values(bad) == ["b"], "should name what was never located"


def test_a_moved_object_is_forgotten_rather_than_reused():
    """Its position was true when measured and is history once it is picked up."""
    import types as _t

    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.dataflow import Look, Pick, run
    from heron.skills.values import Located

    picked = []

    class Robot:
        arms = ["right"]
        cameras = ["cam_high"]

    ctx = SkillContext(robot=Robot(), belief=None, grounder=None, cfg=HeronConfig(),
                       log=EpisodeLogger("/tmp/heron-test-episodes", "flow"))

    import heron.skills.dataflow as df
    import heron.skills.values as values
    from heron.types import SkillResult

    def fake_look(c, queries, camera=None):
        p = values.Perception()
        for q in queries:
            p.seen[q] = Located(np.array([0.3, 0.1, 0.0]), "mask", 0.9)
            p.calls += 1
        return p

    def fake_pick(c, entity, arm="auto", at=None, **kw):
        picked.append((entity, at is not None))
        return SkillResult(ok=True, info="picked")

    df.look = fake_look
    import heron.skills.primitives as prim
    real_pick = prim.pick
    prim.pick = fake_pick
    try:
        out = run(ctx, [Look(["a"]), Pick("a")])
    finally:
        prim.pick = real_pick
        df.look = values.look

    assert out.ok and picked == [("a", True)], "pick must be handed the value"
    assert "a" not in out.seen, "a picked object's old position must not linger"


def test_look_asks_for_everything_in_one_request():
    """Six objects cost six calls because the code asked one query at a time,
    while the model reads the whole frame to answer about any part of it."""
    from heron.skills.values import look

    ctx, calls, Robot = _value_ctx()
    asked = []

    class Batch:
        def locate_many(self, frame, queries):
            asked.append(list(queries))
            return {q: {"point": (30, 30), "box": None, "conf": 0.9} for q in queries}

        def points(self, frame, query):
            calls.append(("points", query))
            return []

        def boxes(self, frame, query):
            return []

    ctx.grounder = Batch()
    Robot.captures = 0
    seen = look(ctx, ["a", "b", "c", "d", "e", "f"])

    assert len(seen.seen) == 6
    assert seen.calls == 1, f"six objects should cost one call, spent {seen.calls}"
    assert asked == [["a", "b", "c", "d", "e", "f"]]
    assert calls == [], f"fell back to one-at-a-time: {calls}"


def test_whatever_the_batch_missed_is_asked_for_singly():
    """A partial answer plus a few singles still beats one call each."""
    from heron.skills.values import look

    ctx, calls, _ = _value_ctx(points={"c": [(30, 30, 0.9)]})

    class PartialBatch:
        def locate_many(self, frame, queries):
            return {"a": {"point": (20, 20), "box": None, "conf": 0.9}}

        def points(self, frame, query):
            calls.append(("points", query))
            return [(30, 30, 0.9)] if query == "c" else []

        def boxes(self, frame, query):
            return []

    ctx.grounder = PartialBatch()
    seen = look(ctx, ["a", "b", "c"])
    assert set(seen.seen) == {"a", "c"}
    assert [q for _, q in calls] == ["b", "c"], f"asked singly for: {calls}"


def test_an_over_wide_vessel_is_gripped_beside_its_centre_but_never_above_it():
    """The bowl every libero_spatial task starts with, measured against the
    simulator's own body position.

    The bowl is 96 mm across and the gripper opens to 78 mm, so pick takes the
    rim. It used to also RAISE the grasp by span/4 on the reasoning that a rim
    sits above a centroid — which put the fingers at z = 0.9542 against a bowl
    whose top edge is at 0.9284, 26 mm of clear air, and they closed on nothing
    in every attempt. Sweeping the real thing: 20-48 mm out and 10-30 mm BELOW
    the top all lift it; 26 mm above lifts nothing.
    """
    from heron.skills.primitives import GRASP_DEPTH, GRIPPER_MAX_SPAN_M, RIM_OFFSET_MAX_M

    perceived_top = 0.9252          # what grounding reported for this bowl
    grasp_z = perceived_top - GRASP_DEPTH
    assert grasp_z < perceived_top, "a rim grasp must descend, not rise"
    # And it lands inside the window the sweep found (10-30 mm below the top).
    true_top = 0.9284
    assert 0.008 <= (true_top - grasp_z) <= 0.035

    # The offset is capped: the measured span over-reads (116 mm on a 96 mm
    # bowl), and half of that aims 58 mm out, past everything that worked.
    span = 0.1159
    assert span > GRIPPER_MAX_SPAN_M
    assert min(span / 2.0, RIM_OFFSET_MAX_M) == RIM_OFFSET_MAX_M
    assert RIM_OFFSET_MAX_M <= 0.048


# -- two objects, one container ----------------------------------------------
#
# Every placement onto a container aimed at its centre, so the second object was
# dropped onto the first. Measured in the MuJoCo twin with GROUND-TRUTH
# positions, so perception could not be blamed: each block landed 2-4 mm from
# the plate's centre and the next placement then knocked it to 35-50 mm. Even
# with the two aims staggered 40 mm apart the first block still moved 38 mm,
# because the gripper opens to 80 mm and each jaw swings 40 mm out at table
# height.

def test_the_release_aperture_is_the_object_plus_clearance():
    from heron.skills.primitives import OPEN_WIDTH, RELEASE_JAW_CLEARANCE_M, release_width

    held = 0.029                     # the 32 mm block, as the gripper reads it
    got = release_width(held)
    assert got == pytest.approx(held + 2 * RELEASE_JAW_CLEARANCE_M)
    assert got > 0.032, "must open wider than the object or it cannot fall out"
    assert got < OPEN_WIDTH, "opening fully is what swept the neighbour aside"


def test_an_unmeasured_grip_falls_back_to_opening_fully():
    """A backend with no gripper feedback must not be given a narrow aperture
    computed from a zero it never meant as a width."""
    from heron.skills.primitives import OPEN_WIDTH, release_width

    assert release_width(None) == OPEN_WIDTH
    assert release_width(0.0) == OPEN_WIDTH


def test_slot_spacing_clears_a_jaw_and_a_neighbour():
    from heron.skills.primitives import PLACE_SLOT_MARGIN_M, release_width, slot_step

    held = 0.029
    step = slot_step(held, release_width(held))
    # The jaw reaches aperture/2 from the tool centre; the neighbour occupies
    # its own half width. Anything closer is a collision, not a placement.
    assert step >= 0.5 * release_width(held) + 0.5 * held + PLACE_SLOT_MARGIN_M - 1e-9
    assert step == pytest.approx(0.5 * release_width(held) + 0.5 * held
                                 + PLACE_SLOT_MARGIN_M)


def test_no_two_slots_are_closer_together_than_the_step():
    """`step` is the only quantity the caller has to get right, so the geometry
    has to honour it between every pair — not just between a slot and the
    centre."""
    from heron.skills.primitives import slot_offsets

    step = 0.047
    offs = slot_offsets(step, rings=2)
    assert len(offs) == 13
    assert np.allclose(offs[0], 0.0), "the first object goes in the middle"
    for i, a in enumerate(offs):
        for b in offs[i + 1:]:
            assert float(np.linalg.norm(a - b)) >= step - 1e-9, \
                f"{np.round(a, 4)} and {np.round(b, 4)} are closer than {step}"


def test_the_first_slot_tried_leans_back_toward_the_base():
    """This arm runs out of reach in +x first: a 47 mm offset straight outward
    was refused by the kinematics at BOTH containers, and a search that starts
    there spends itself on the one direction that cannot work."""
    from heron.skills.primitives import slot_offsets

    first_ring = slot_offsets(0.05, rings=1)[1]
    assert first_ring[0] < 0, f"first candidate offset is {first_ring}, not toward the base"


def _place_ctx(reachable=lambda arm, xyz: True):
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext

    class Robot:
        arms = ["right"]
        cameras = []

        def get_gripper(self, arm):
            return {"width_m": 0.029, "effort": 5.0}

        def reachable(self, arm, xyz):
            return reachable(arm, xyz)

    return SkillContext(robot=Robot(), belief=None, grounder=None, cfg=HeronConfig(),
                        log=EpisodeLogger("/tmp/heron-test-episodes", "slots"))


def test_the_second_object_onto_one_support_is_not_aimed_at_the_first():
    from heron.skills.primitives import _spread_within_support, release_width, slot_step

    ctx = _place_ctx()
    dest = np.array([0.375, 0.135, -0.006])
    first, slot0 = _spread_within_support(ctx, "right", dest.copy(), "block_a", "white_plate", None)
    second, slot1 = _spread_within_support(ctx, "right", dest.copy(), "block_b", "white_plate", None)

    assert slot0 == 0 and np.allclose(first, dest), "the first one goes in the middle"
    assert slot1 == 1
    step = slot_step(0.029, release_width(0.029))
    assert float(np.linalg.norm(second[:2] - first[:2])) >= step - 1e-9
    assert second[2] == pytest.approx(dest[2]), "the spread is horizontal only"


def test_a_named_container_and_a_bare_position_are_the_same_support():
    """The agent path names the container and the dataflow path hands a
    position in. If those land in different buckets the spread restarts from
    the centre halfway through a task and stacks after all."""
    from heron.skills.primitives import _support_key

    dest = np.array([0.375, 0.135, -0.006])
    assert _support_key("white_plate", dest) == _support_key("white_plate", dest + 0.02)
    assert _support_key(None, dest) == _support_key(None, dest + 0.001)
    assert _support_key(None, dest) != _support_key(None, dest + np.array([0.1, 0, 0]))


def test_an_offset_the_arm_cannot_reach_is_not_chosen():
    """An aim outside the envelope turns a crowded support into a failed step.
    Refuse the offset, not the placement."""
    from heron.skills.primitives import _spread_within_support

    # Only the half-plane toward the base is reachable, as on the real arm.
    ctx = _place_ctx(reachable=lambda arm, xyz: float(np.asarray(xyz)[0]) < 0.376)
    dest = np.array([0.375, 0.135, -0.006])
    _spread_within_support(ctx, "right", dest.copy(), "block_a", "plate", None)
    second, slot = _spread_within_support(ctx, "right", dest.copy(), "block_b", "plate", None)
    assert slot == 1
    assert second[0] < 0.376, f"chose an unreachable slot at x={second[0]:.3f}"


def test_a_support_too_small_to_share_is_still_aimed_at_its_centre():
    """Better a stack on the only spot that exists than a placement beside the
    container."""
    from heron.skills.primitives import _spread_within_support

    class _Tiny:
        span_m = 0.03      # narrower than the object being placed

    ctx = _place_ctx()
    dest = np.array([0.375, 0.135, -0.006])
    _spread_within_support(ctx, "right", dest.copy(), "block_a", "eggcup", _Tiny())
    second, _ = _spread_within_support(ctx, "right", dest.copy(), "block_b", "eggcup", _Tiny())
    assert np.allclose(second, dest)


def test_the_spread_is_bounded_by_a_measured_support():
    """A mask that says how wide the plate is bounds how far out the next object
    may go; without it the spread would walk objects off the edge."""
    from heron.skills.primitives import _spread_within_support

    class _Plate:
        span_m = 0.124                       # the twin's plate, 62 mm radius

    ctx = _place_ctx()
    dest = np.array([0.375, 0.135, -0.006])
    _spread_within_support(ctx, "right", dest.copy(), "block_a", "plate", _Plate())
    second, _ = _spread_within_support(ctx, "right", dest.copy(), "block_b", "plate", _Plate())
    out = float(np.linalg.norm(second[:2] - dest[:2]))
    assert out + 0.5 * 0.029 <= 0.5 * _Plate.span_m, \
        f"the object's far edge overhangs the plate ({out * 1000:.0f} mm out)"


# -- a verifier must never end the episode ------------------------------------

def test_a_verifier_that_cannot_move_the_arm_answers_unknown():
    """Active verification moves the arm: `visible` falls back to a close-range
    wrist look, `in`/`on` fly the wrist over a container. Those skill calls are
    made from inside the verifier and therefore OUTSIDE the registry, the only
    place that catches anything.

    Measured: a placement the arm could not reach raised SafetyViolation inside
    _visible -> _close_look -> inspect -> travel, escaped check(), escaped the
    step loop, and ended a run at status "crashed" with 10 steps done and 2
    edits spent. Nothing had moved — the safety check refuses before commanding
    the arm — so there was nothing to recover from except the exception itself.
    """
    from heron.belief import BeliefStore
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.robot.safety import SafetyViolation
    from heron.skills import SkillContext
    from heron.types import EntityDecl, PredicateSpec, Value
    from heron.verify import check

    class _Robot:
        arms = ["right"]
        cameras = ["cam_high", "cam_right_wrist"]

        def get_gripper(self, arm):
            raise SafetyViolation(
                "target [0.432, 0.166, 0.135] is inside the right workspace box "
                "but the arm cannot hold its approach orientation there")

    ctx = SkillContext(robot=_Robot(), belief=BeliefStore(), grounder=None,
                       cfg=HeronConfig(),
                       log=EpisodeLogger("/tmp/heron-test-episodes", "verify-raises"))
    ctx.belief.declare_entities([EntityDecl(id="cube", description="the small red cube")])
    spec = PredicateSpec(name="holding", args=["cube", "right"])

    sat, verdict = check(ctx, spec)      # must not raise
    assert sat is Value.UNKNOWN
    assert verdict.confidence == 0.0
    assert "SafetyViolation" in verdict.note


def test_an_unverifiable_precondition_is_a_failure_not_a_success():
    """UNKNOWN has to fall on the side of "not satisfied", or swallowing the
    exception would turn a crash into a step that runs on an assumption nobody
    checked."""
    from heron.types import PredicateSpec, Value

    spec = PredicateSpec(name="holding", args=["cube", "right"])
    assert spec.satisfied_by(Value.UNKNOWN) is Value.UNKNOWN
    assert PredicateSpec(name="holding", args=["cube", "right"],
                         negated=True).satisfied_by(Value.UNKNOWN) is Value.UNKNOWN


def test_the_close_look_drops_to_a_height_the_arm_can_hold():
    """A fixed look height is a guess about reach. Measured in the twin: with a
    block sitting on the plate at (0.381, 0.280), nothing above 0.05 m has a
    downward solution, so the wrist look was refused by the safety envelope, the
    `on(...)` it was sent to settle stayed UNKNOWN, and the episode aborted on a
    task it had physically completed."""
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.sensing import INSPECT_ABOVE_M, _reachable_look_height

    ceiling = 0.10

    class Robot:
        arms = ["right"]
        cameras = ["cam_right_wrist"]

        def reachable(self, arm, xyz):
            return float(np.asarray(xyz)[2]) <= ceiling    # the measured shape

    cfg = HeronConfig()
    cfg.safety.workspace_z = (0.01, 0.40)
    ctx = SkillContext(robot=Robot(), belief=None, grounder=None, cfg=cfg,
                       log=EpisodeLogger("/tmp/heron-test-episodes", "look"))
    subject_z = -0.007
    want = subject_z + INSPECT_ABOVE_M          # 0.143, above what the arm can hold
    got = _reachable_look_height(ctx, "right", 0.381, 0.280, want, subject_z)
    assert got <= ceiling + 1e-9, f"chose {got:.3f}, which the arm cannot hold"
    assert got <= want


def test_a_look_the_arm_cannot_make_at_all_is_refused_by_name():
    """WHAT THIS FIX DOES NOT DO, stated so it is not mistaken for more.

    When the entire band between "close enough to focus" and "as high as we
    wanted" is out of reach, there is no look to be had at that spot. Lowering
    further would put the wrist inside the thing it came to look at. So there is
    no height, `inspect` fails saying why, and the predicate stays UNKNOWN —
    which is honest, and which diagnosis can act on.

    That is exactly the measured case: a block resting on the plate at
    (0.381, 0.280) where nothing above 0.05 m has a downward solution, while the
    look wanted 0.166. Verifying that placement needs a look from an offset
    viewpoint, which this does not attempt.

    An earlier version handed the height back unchanged and let `travel` refuse
    it. That reads the same from a distance and is not: the refusal arrived as a
    SafetyViolation raised three frames down, from a pose nothing had any reason
    to command.
    """
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.sensing import INSPECT_ABOVE_M, INSPECT_MIN_ABOVE_M, _reachable_look_height

    class Robot:
        arms = ["right"]
        cameras = ["cam_right_wrist"]

        def reachable(self, arm, xyz):
            return float(np.asarray(xyz)[2]) <= 0.05

    ctx = SkillContext(robot=Robot(), belief=None, grounder=None, cfg=HeronConfig(),
                       log=EpisodeLogger("/tmp/heron-test-episodes", "look"))
    subject_z = 0.016                            # a block resting on the plate
    want = subject_z + INSPECT_ABOVE_M
    got = _reachable_look_height(ctx, "right", 0.381, 0.280, want, subject_z)
    assert got is None, "must not invent a height inside the subject, nor one nobody can hold"


def test_the_close_look_never_descends_onto_what_it_is_looking_at():
    """Lowering to find a reachable pose must stop well above the subject:
    trading a refused motion for a collision is not an improvement, and the
    D405 cannot focus closer than 70 mm anyway."""
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.sensing import INSPECT_ABOVE_M, INSPECT_MIN_ABOVE_M, _reachable_look_height

    class Robot:
        arms = ["right"]
        cameras = ["cam_right_wrist"]

        def reachable(self, arm, xyz):
            return False        # nothing at all is reachable here

    cfg = HeronConfig()
    ctx = SkillContext(robot=Robot(), belief=None, grounder=None, cfg=cfg,
                       log=EpisodeLogger("/tmp/heron-test-episodes", "look"))
    subject_z = 0.02
    want = subject_z + INSPECT_ABOVE_M
    got = _reachable_look_height(ctx, "right", 0.3, 0.0, want, subject_z)
    # Nothing was reachable, so there is no look — and in particular no height
    # inside the object was ever proposed on the way to saying so.
    assert got is None
    from heron.episode import read_journal

    for record in read_journal(ctx.log.dir):
        if record["kind"] == "inspect_height_lowered":
            assert record["used"] >= subject_z + INSPECT_MIN_ABOVE_M


def test_a_backend_with_no_reachability_model_still_looks():
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.sensing import _reachable_look_height

    class Robot:
        arms = ["right"]
        cameras = ["cam_right_wrist"]

    ctx = SkillContext(robot=Robot(), belief=None, grounder=None, cfg=HeronConfig(),
                       log=EpisodeLogger("/tmp/heron-test-episodes", "look"))
    assert _reachable_look_height(ctx, "right", 0.3, 0.0, 0.18, 0.0) == 0.18


def test_every_registered_skill_points_at_the_function_that_bears_its_name():
    """A decorator applies to whatever function follows it.

    Inserting a helper between `@skill(name="inspect", ...)` and `def inspect`
    silently rebound the registry's "inspect" to the helper. Nothing failed at
    import, nothing failed in the suite, and the first symptom was a live run
    reporting `_reachable_look_height() got an unexpected keyword argument
    'entity'` after four minutes of episodes.
    """
    from heron.skills import load_all, registry

    load_all()
    wrong = [(name, registry.get(name).fn.__name__)
             for name in registry.names()
             if registry.get(name).fn.__name__.lstrip("_") != name]
    assert not wrong, f"registered under the wrong function: {wrong}"


# -- perception that is not paid for twice ------------------------------------

def test_the_span_grounding_measured_is_not_re_asked_for():
    """`pick` needs the object's width to decide whether it must take the rim.
    Grounding draws the box that width comes from, and used to throw it away —
    so every grasp paid a fresh capture and a fresh detector call for a number
    computed a tenth of a second earlier. Measured over a 60-episode sweep: 511
    detector calls, 1166 seconds, all on the critical path of a grasp."""
    from heron.belief import BeliefStore
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.primitives import _visual_span
    from heron.types import EntityDecl

    class _Grounder:
        def box(self, *a, **k):
            raise AssertionError("asked the detector for a span already measured")

    class _Robot:
        arms = ["right"]
        cameras = ["cam_high"]

        def capture(self, cam):
            raise AssertionError("captured a frame for a span already measured")

    belief = BeliefStore()
    belief.declare_entities([EntityDecl(id="bowl", description="the black bowl")])
    belief.update_track("bowl", xyz_base=(0.3, 0.1, 0.02), camera="cam_high",
                        span_m=0.096, confidence=0.9)
    ctx = SkillContext(robot=_Robot(), belief=belief, grounder=_Grounder(),
                       cfg=HeronConfig(),
                       log=EpisodeLogger("/tmp/heron-test-episodes", "span"))
    assert _visual_span(ctx, "bowl") == pytest.approx(0.096)


def test_an_entity_with_no_measured_span_still_falls_back_to_the_detector():
    """Dead reckoning after a place, or a caller handing a bare position in,
    leaves no box behind. Refusing to look would turn a cheap call into a failed
    rim decision on a 96 mm bowl."""
    from heron.belief import BeliefStore
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.skills import SkillContext
    from heron.skills.primitives import _visual_span
    from heron.types import EntityDecl, Frame

    asked = []

    class _Grounder:
        def box(self, frame, description):
            asked.append(description)
            return (10, 10, 30, 30, 0.9)

    class _Robot:
        arms = ["right"]
        cameras = ["cam_high"]

        def capture(self, cam):
            return Frame(camera=cam, rgb=np.zeros((60, 60, 3), np.uint8))

    belief = BeliefStore()
    belief.declare_entities([EntityDecl(id="bowl", description="the black bowl")])
    belief.update_track("bowl", xyz_base=(0.3, 0.1, 0.02), camera="cam_high",
                        confidence=0.9)
    ctx = SkillContext(robot=_Robot(), belief=belief, grounder=_Grounder(),
                       cfg=HeronConfig(),
                       log=EpisodeLogger("/tmp/heron-test-episodes", "span"))
    _visual_span(ctx, "bowl")
    assert asked == ["the black bowl"], "should have fallen back to the detector"


def test_a_span_is_measured_across_the_box_bottom_not_its_middle():
    """The bottom edge is where the object meets the support. The top edge of a
    tall object deprojects onto whatever is standing behind it."""
    from heron.skills.sensing import _span_from_box
    from heron.types import Frame

    seen = []

    class _F(Frame):
        def deproject(self, u, v):
            seen.append((u, v))
            return np.array([u * 0.001, 0.0, 0.0])

    frame = _F(camera="cam_high", rgb=np.zeros((60, 60, 3), np.uint8))
    got = _span_from_box(frame, (10, 20, 50, 40))
    assert got == pytest.approx(0.040)
    assert {v for _, v in seen} == {40}, f"deprojected rows {seen}, not the bottom edge"
    assert _span_from_box(frame, None) is None


def test_a_plan_that_promises_nothing_cannot_succeed(rig):
    """`_goal_check` walks the goal predicates and reports what is unmet. Over an
    empty list it reports nothing unmet, which the loop reads as "all goal
    predicates verified".

    Measured in the twin: a plan for "move the red block into the grey tray"
    came back as two picks, no place, no postcondition on any step and no goal
    predicates at all. The episode was recorded as SUCCEEDED with the block
    206 mm from the tray, and the skill library then learned from it. One
    episode in 62 — and total when it happens.
    """
    cfg, mock, agent = rig
    program = TaskProgram(
        goal="move the red block into the grey tray",
        steps=[Step(id="s1", skill="perceive", args={}),
               Step(id="s2", skill="pick", args={"entity": "red_block"})],
        goal_predicates=[],
    )
    agent._reject_vacuous_goal(program)
    assert program.aborted, "an ungradeable plan must not be allowed to run"
    assert "nothing to verify" in (program.abort_reason or "")


def test_the_empty_goal_check_is_what_made_it_look_like_success(rig):
    """Pins the mechanism, so a future change to _goal_check cannot quietly
    restore it."""
    cfg, mock, agent = rig
    program = TaskProgram(goal="do something", steps=[], goal_predicates=[])
    assert agent._goal_check(program) == [], \
        "an empty goal list reports nothing unmet — which is why it must be refused earlier"


def test_a_plan_graded_on_the_wrong_object_is_refused(rig):
    """The plan verified itself, not the task.

    Asked to "move the blue block onto the white plate", the planner declared
    the target `grey_plate`, "the grey square plate" — the tray, which is a grey
    square, sitting next to the white plate it was asked for. It put the block
    on the tray, verified `on(blue_block, grey_plate)`, and was recorded as
    SUCCEEDED. Nothing downstream could catch it: the predicate was true and the
    block really was on the thing the plan pointed at. The goal predicates are
    written by the model that names the entities, so verifying them confirms the
    plan's self-consistency, never its fidelity to the sentence.
    """
    cfg, mock, agent = rig
    program = TaskProgram(
        goal="move the blue block onto the white plate",
        entities=[EntityDecl(id="blue_block", description="the small blue block"),
                  EntityDecl(id="grey_plate", description="the grey square plate")],
        steps=[Step(id="s1", skill="perceive", args={})],
        goal_predicates=[PredicateSpec(name="on", args=["blue_block", "grey_plate"])],
    )
    agent._reject_misgrounded_goal(program, "move the blue block onto the white plate")
    # REPAIRED, not refused: the description is the detector's query, and the
    # instruction's own phrase is the one query the task guarantees to identify
    # the object. Grounding then finds the WHITE PLATE instead of confirming
    # the tray — the guard now fixes the plan instead of merely vetoing it.
    assert not program.aborted, program.abort_reason
    target = next(e for e in program.entities if e.id == "grey_plate")
    assert "white" in target.description.lower(), target.description


def test_a_faithful_plan_is_left_alone(rig):
    cfg, mock, agent = rig
    program = TaskProgram(
        goal="move the blue block onto the white plate",
        entities=[EntityDecl(id="blue_block", description="the small blue block"),
                  EntityDecl(id="white_plate", description="the white circular plate")],
        steps=[Step(id="s1", skill="perceive", args={})],
        goal_predicates=[PredicateSpec(name="on", args=["blue_block", "white_plate"])],
    )
    agent._reject_misgrounded_goal(program, "move the blue block onto the white plate")
    assert not program.aborted


def test_an_extra_colour_the_task_never_named_is_not_a_defect(rig):
    """A plan may well mention a green block that is in the way. Only the
    direction that matters is checked: a colour the INSTRUCTION used that the
    graded entities do not mention."""
    cfg, mock, agent = rig
    program = TaskProgram(
        goal="move the blue block onto the white plate",
        entities=[EntityDecl(id="blue_block", description="the blue block"),
                  EntityDecl(id="white_plate", description="the white plate"),
                  EntityDecl(id="green_block", description="the green block in the way")],
        steps=[Step(id="s1", skill="perceive", args={})],
        goal_predicates=[PredicateSpec(name="on", args=["blue_block", "white_plate"])],
    )
    agent._reject_misgrounded_goal(program, "move the blue block onto the white plate")
    assert not program.aborted


def test_a_colourless_instruction_is_not_second_guessed(rig):
    """With nothing to compare, the check has nothing to say and says nothing."""
    cfg, mock, agent = rig
    program = TaskProgram(
        goal="put the block on the plate",
        entities=[EntityDecl(id="block", description="the block"),
                  EntityDecl(id="plate", description="the plate")],
        steps=[Step(id="s1", skill="perceive", args={})],
        goal_predicates=[PredicateSpec(name="on", args=["block", "plate"])],
    )
    agent._reject_misgrounded_goal(program, "put the block on the plate")
    assert not program.aborted


def test_the_misgrounded_plan_is_dropped_from_the_cache(rig):
    """It was cached BECAUSE it reported success, and a sister recall then
    spread it to the next instruction, which cached its own copy with a fresh
    success. A mis-grounded plan does not merely fail once; it gains confidence.
    Retiring it at the abort stops the next episode paying a recall to abort on
    it again — and leaves the correct plan sitting next to it alone."""
    cfg, mock, agent = rig
    bad = {
        "goal": "move the blue block onto the white plate",
        "entities": [{"id": "blue_block", "description": "the small blue block"},
                     {"id": "grey_plate", "description": "the grey square plate"}],
        "steps": [{"id": "s1", "skill": "perceive", "args": {}}],
        "goal_predicates": [{"name": "on", "args": ["blue_block", "grey_plate"],
                             "negated": False}],
    }
    good = {
        "goal": "put the blue block on the white plate",
        "entities": [{"id": "blue_block", "description": "the blue block"},
                     {"id": "white_plate", "description": "the white plate"}],
        "steps": [{"id": "s1", "skill": "perceive", "args": {}}],
        "goal_predicates": [{"name": "on", "args": ["blue_block", "white_plate"],
                             "negated": False}],
    }
    agent.memory.remember_program("move the blue block onto the white plate", bad)
    agent.memory.remember_program("put the blue block on the white plate", good)

    agent._reject_misgrounded_goal(TaskProgram(**bad),
                                   "move the blue block onto the white plate")
    assert agent.memory.recall_program("move the blue block onto the white plate") is None
    assert agent.memory.recall_program("put the blue block on the white plate") is not None


def test_the_same_failure_eleven_times_is_not_eleven_problems(rig):
    """The repair cap counted step ids, and `replace_tail` mints new ones.

    So every repair reset the counter it was meant to be spending, and the cap
    never bound. Measured in the twin: an object landed at (0.436, 0.238),
    outside the arm's envelope, and "nothing above (0.436, 0.238) is reachable
    between z=0.000 and z=0.180" came back ELEVEN times about thirty seconds
    apart until the episode's whole edit budget was gone — 442 seconds, 18 steps,
    12 edits, for a fact that was equally true at the first attempt.
    """
    from heron.types import FailureClass, FailureReport

    cfg, mock, agent = rig
    program = TaskProgram(goal="put the block down",
                          steps=[Step(id="s1", skill="perceive", args={})],
                          goal_predicates=[PredicateSpec(name="on", args=["a", "b"])])
    # The jitter is the point: the same fact, reported in the third decimal.
    # New contract: the wall triggers ONE full replan first (a fresh approach
    # is a legitimate bet), and the SAME wall after the replan is the abort.
    for i, xy in enumerate(("0.435, 0.238", "0.436, 0.238", "0.436, 0.237",
                            "0.435, 0.237", "0.436, 0.238", "0.435, 0.238",
                            "0.436, 0.237", "0.436, 0.238")):
        report = FailureReport(
            step_id=f"r{i}_1", skill="place", failure_class=FailureClass.PROGRAM,
            summary=f"ValueError: nothing above ({xy}) is reachable between "
                    f"z=0.000 and z=0.180 — the object is outside this")
        if agent._giving_up_on(program, report):
            break
    assert program.aborted, "the same wall, hit five times, is still one wall"
    assert "nothing has" in (program.abort_reason or "")
    assert i + 1 <= cfg.budgets.max_repairs_per_step, \
        f"gave up after {i + 1} identical failures; the budget is {cfg.budgets.max_repairs_per_step}"


def test_different_failures_are_still_worth_repairing(rig):
    """The guard must not fire on a task that is making progress through a
    series of genuinely different problems."""
    from heron.types import FailureClass, FailureReport

    cfg, mock, agent = rig
    program = TaskProgram(goal="put the block down",
                          steps=[Step(id="s1", skill="perceive", args={})],
                          goal_predicates=[PredicateSpec(name="on", args=["a", "b"])])
    for summary in ("the gripper closed on nothing",
                    "red_block was not found in ['cam_high']",
                    "the arm cannot hold its approach orientation there",
                    "blue_bowl has no location estimate"):
        report = FailureReport(step_id="s1", skill="place",
                               failure_class=FailureClass.SKILL_EFFECT, summary=summary)
        assert not agent._giving_up_on(program, report), summary
    assert not program.aborted


def test_a_correction_bigger_than_the_target_is_not_a_correction(rig):
    """The cause of every remaining failure in the eight-episode run.

    The stored aiming offset for the grey tray was (31, 66) mm against a tray
    whose half-width is 55 mm, so every placement on it was aimed clean off the
    edge — 72 mm out, against an `on` tolerance of 30 mm. Three episodes, three
    identical misses, and the residual could never come back because the aim was
    already outside the thing it was aiming at.
    """
    from heron.skills.primitives import PLACE_HINT_MAX_FRACTION, _usable_place_hint

    cfg, mock, agent = rig
    agent.belief.track("grey_tray").description = "the grey square tray"
    agent.belief.update_track("grey_tray", xyz_base=(0.30, 0.0, 0.0), confidence=0.9,
                              span_m=0.110)
    assert _usable_place_hint(agent.ctx, "grey_tray", (0.0307, 0.0655)) == (0.0, 0.0)


def test_a_correction_that_fits_is_still_carried(rig):
    """A systematic bias is real and worth inheriting; only the ones that aim off
    the support are refused."""
    from heron.skills.primitives import _usable_place_hint

    cfg, mock, agent = rig
    agent.belief.track("grey_tray").description = "the grey square tray"
    agent.belief.update_track("grey_tray", xyz_base=(0.30, 0.0, 0.0), confidence=0.9,
                              span_m=0.110)
    assert _usable_place_hint(agent.ctx, "grey_tray", (0.008, -0.006)) == (0.008, -0.006)


def test_with_no_measured_size_the_hint_is_left_alone(rig):
    """Nothing to judge it against. Refusing every hint on an unmeasured support
    would throw away the mechanism to guard against one misuse of it."""
    from heron.skills.primitives import _usable_place_hint

    cfg, mock, agent = rig
    agent.belief.track("mystery").description = "a thing of unknown size"
    assert _usable_place_hint(agent.ctx, "mystery", (0.03, 0.03)) == (0.03, 0.03)


def test_replacing_the_same_object_aims_at_the_centre_again():
    """The spiral this ledger change exists to kill.

    A block was placed, failed to verify, was re-picked and re-placed — and the
    spread treated the block's OWN first placement as an occupant to avoid,
    aiming at slot 1, 47 mm out. That offset is outside both `verify.ON_XY_MAX`
    (30 mm) and a tray cavity's `in` bound (33 mm), so the re-placement could
    never verify, the repair loop went around again, and each pass drifted
    further. Avoiding yourself turned the avoidance into the failure.
    """
    from heron.skills.primitives import _spread_within_support

    ctx = _place_ctx()
    dest = np.array([0.375, 0.135, -0.006])
    first, slot0 = _spread_within_support(ctx, "right", dest.copy(), "block_a", "plate", None)
    again, slot1 = _spread_within_support(ctx, "right", dest.copy(), "block_a", "plate", None)
    assert slot0 == 0 and slot1 == 0
    assert np.allclose(again, dest), "what is in the gripper is not on the support"


def test_picking_an_object_frees_its_slot_for_the_next_one():
    from heron.skills.primitives import _spread_within_support, forget_placement

    ctx = _place_ctx()
    dest = np.array([0.375, 0.135, -0.006])
    _spread_within_support(ctx, "right", dest.copy(), "block_a", "plate", None)
    forget_placement(ctx, "block_a")     # what pick does on success
    fresh, slot = _spread_within_support(ctx, "right", dest.copy(), "block_b", "plate", None)
    assert slot == 0 and np.allclose(fresh, dest), \
        "block_a left with the gripper; its slot is not haunted"


def test_near_is_in_the_vocabulary_and_grades_geometrically(rig):
    """"Push the plate to the front of the stove" could not be said in the old
    vocabulary, so the planner reached for gripper_empty(arm) — true at t=0 —
    and the vacuous-goal guard rightly aborted in 25-35 s. Every push-family
    LIBERO-PRO task scored zero BY CONSTRUCTION. The guard was correct; the
    vocabulary was too small to let it matter."""
    from heron.verify import NEAR_SLACK_M, _near

    cfg, mock, agent = rig
    agent.belief.track("red_block").description = "the red block"
    agent.belief.track("blue_bowl").description = "the blue bowl"
    bowl = mock.gt_xyz("blue_bowl")
    agent.belief.update_track("blue_bowl", xyz_base=tuple(bowl), confidence=0.9,
                              from_sighting=True, span_m=0.10)
    mock.teleport("red_block", np.array([bowl[0] + 0.08, bowl[1], 0.02]))
    agent.belief.update_track("red_block", xyz_base=(bowl[0] + 0.08, bowl[1], 0.02),
                              confidence=0.9, from_sighting=True)
    v = _near(agent.ctx, PredicateSpec(name="near", args=["red_block", "blue_bowl"]))
    assert v.value is Value.TRUE, v.note        # 80 mm vs 50+60 mm limit

    mock.teleport("red_block", np.array([bowl[0] + 0.30, bowl[1], 0.02]))
    agent.belief.update_track("red_block", xyz_base=(bowl[0] + 0.30, bowl[1], 0.02),
                              confidence=0.9, from_sighting=True)
    v = _near(agent.ctx, PredicateSpec(name="near", args=["red_block", "blue_bowl"]))
    assert v.value is Value.FALSE, v.note


def test_a_push_goal_is_now_expressible_not_vacuous(rig):
    cfg, mock, agent = rig
    agent.belief.track("red_block").description = "the red block"
    agent.belief.track("blue_bowl").description = "the blue bowl"
    bowl = mock.gt_xyz("blue_bowl")
    agent.belief.update_track("blue_bowl", xyz_base=tuple(bowl), confidence=0.9,
                              from_sighting=True, span_m=0.10)
    agent.belief.update_track("red_block", xyz_base=(bowl[0] + 0.30, bowl[1], 0.02),
                              confidence=0.9, from_sighting=True)
    program = TaskProgram(
        goal="push the red block to the front of the blue bowl",
        entities=[EntityDecl(id="red_block", description="the red block"),
                  EntityDecl(id="blue_bowl", description="the blue bowl")],
        steps=[Step(id="s1", skill="push", args={"entity": "red_block"})],
        goal_predicates=[PredicateSpec(name="near", args=["red_block", "blue_bowl"])],
    )
    agent._reject_vacuous_goal(program)
    assert not program.aborted, program.abort_reason


def test_push_toward_measures_its_own_direction(rig):
    """The vocabulary fix exposed the next gap in the same family: near() made
    the goal expressible, but push still demanded a world-frame dx/dy the
    planner has no numbers to compute — it emitted zeros, and 'push needs a
    nonzero dx/dy' ended the episode at t=118 s. A destination is a name, not
    a vector: push(entity, toward=Y) measures both at execution."""
    from heron.skills.primitives import push

    cfg, mock, agent = rig
    agent.belief.track("red_block").description = "the red block"
    agent.belief.track("blue_bowl").description = "the blue bowl"
    bowl = np.asarray(mock.gt_xyz("blue_bowl"))
    block = np.array([bowl[0] + 0.15, bowl[1], 0.02])
    mock.teleport("red_block", block)
    agent.belief.update_track("red_block", xyz_base=tuple(block), confidence=0.9,
                              from_sighting=True)
    agent.belief.update_track("blue_bowl", xyz_base=tuple(bowl), confidence=0.9,
                              from_sighting=True, span_m=0.10)

    result = push(agent.ctx, "red_block", toward="blue_bowl")
    assert result.ok, result.info
    # The computed run is the measured gap minus the target's half extent:
    # 150 mm - 50 mm = 100 mm, straight along -x. The skill's own info string
    # records the vector it derived — the journal is how this gets audited.
    # Aim is EDGE + half*0.5 deeper: a push under-delivers (69 mm of slide loss
    # measured on the twin), and stopping the centre exactly at the edge left it
    # 8 mm outside near()'s own limit. Overshoot is safe — push re-measures.
    assert "(-0.13,0.00)" in result.info, result.info

    # Zeros without a destination still refuse loudly rather than shove air.
    with pytest.raises(ValueError, match="toward"):
        push(agent.ctx, "red_block")


def test_the_support_criterion_scores_resting_not_centred(rig):
    """First hardware episode: block 47 mm from the plate's centre — ON the
    plate to the person standing there — failed by the benchmark's 30 mm exam
    criterion. The rig grades physics, the benchmark grades itself."""
    from heron.verify import _on

    cfg, mock, agent = rig
    bowl = mock.gt_xyz("blue_bowl")
    agent.belief.track("blue_bowl").description = "the blue bowl"
    agent.belief.update_track("blue_bowl", xyz_base=tuple(bowl), confidence=0.9,
                              from_sighting=True, span_m=0.124)
    agent.belief.track("red_block").description = "the red block"
    landed = (bowl[0] + 0.047, bowl[1], bowl[2] + 0.02)
    mock.objects["red_block"].xyz = np.array(landed)   # grounding must agree
    agent.belief.update_track("red_block", xyz_base=landed,
                              confidence=0.9, from_sighting=True)
    spec = PredicateSpec(name="on", args=["red_block", "blue_bowl"])
    assert _on(agent.ctx, spec).value is Value.FALSE          # centred: 47 > 30
    cfg.on_criterion = "support"
    assert _on(agent.ctx, spec).value is Value.TRUE, "47 mm on a 62 mm-half support is resting"


def test_a_block_on_a_plate_is_not_grounded_at_table_height():
    """The table is not the only floor.

    Depth on a small object is often junk and falls back to a plane — and that
    plane was always the table, so a block resting ON A PLATE came back 20 mm
    too low. The grasp then descended to the plate's rim, closed on it, and
    carried the plate a few centimetres; over ten rig episodes the plate walked
    across the workspace.
    """
    from heron.skills.sensing import _support_top_at

    class _Tr:
        def __init__(self, xyz, desc, span=None):
            self.xyz_base, self.description, self.span_m = xyz, desc, span

    class _Belief:
        entities = ["white_plate", "blue_block"]

        def track(self, eid):
            return {"white_plate": _Tr((0.30, -0.05, -0.007), "the white plate", 0.13),
                    "blue_block": _Tr((0.30, -0.05, 0.008), "the blue block", 0.03)}[eid]

    class _Ctx:
        belief = _Belief()

    # Over the plate -> the plate's top is the floor.
    assert _support_top_at(_Ctx(), 0.30, -0.05) == -0.007
    # Off the plate -> no support, the caller falls back to the table.
    assert _support_top_at(_Ctx(), 0.30, 0.25) is None


def test_a_block_in_a_plate_is_not_gripped_through_the_plate():
    """Rig batch 2026-08-06, ep8: the arm carried the plate away.

    The measured numbers: the block grounded at z = 12.7 mm, the plate's own
    surface at -7 mm, tool_floor 30 mm below the tabletop, contact_descend off.
    The old rule commanded -7.3 mm — under the plate — and the jaws closed on
    block and plate together, which `place` then set down 180 mm away.
    """
    from heron.skills.primitives import GRASP_DEPTH, grasp_height

    table_z, block_top, plate_top = 0.0, 0.0127, -0.007
    floor = table_z - 0.03

    on_the_table = grasp_height(block_top, None, floor)
    assert on_the_table == pytest.approx(block_top - GRASP_DEPTH), \
        "an object on the table must grasp exactly as it always did"

    in_the_plate = grasp_height(block_top, plate_top, floor)
    assert in_the_plate > plate_top, "the fingertips went below the plate"
    assert in_the_plate < block_top, "and must still bite into the block"
    # Half of the 19.7 mm standing proud of the plate.
    assert in_the_plate == pytest.approx(0.0029, abs=1e-4)


def test_the_floor_never_drops_below_the_thing_underneath():
    """Even a flat object with nothing to bite stops at its support."""
    from heron.skills.primitives import grasp_height

    # A sheet lying on a plate: top and support almost coincide, so the bite
    # rounds to nothing and the answer is the support, not the workspace floor.
    assert grasp_height(0.001, 0.0, -0.03) == pytest.approx(0.0005, abs=1e-4)
    assert grasp_height(-0.005, 0.0, -0.03) == pytest.approx(0.0)
