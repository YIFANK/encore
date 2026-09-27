"""The grasp-proposal service is an upgrade, never a dependency.

Three claims, each pinned: the geometry (yaw from a proposed closing line, an
orientation that keeps the approach vertical), the v1 safety envelope (only
near-top-down proposals near the believed object at plausible heights), and
the fallback doctrine (a dead service costs one log line, not the pick).
"""
from __future__ import annotations

import json

import numpy as np
import pytest

from heron.config import HeronConfig
from heron.episode import EpisodeLogger
from heron.perception.graspclient import (
    GraspProposal,
    GraspServiceError,
    _rodrigues,
    yawed_orientation,
)
from heron.robot.mock import MockRobot, standard_scene
from heron.robot.safety import SafeRobot
from heron.skills import SkillContext
from heron.skills.primitives import _service_grasp, pick

TOPDOWN_RVEC = [0.0, np.pi / 2, 0.0]  # wxai: R_y(90 deg) points tool +x down


def _proposal(xyz, approach=(0, 0, -1.0), binormal=(0, 1.0, 0), score=0.9):
    a = np.asarray(approach, float)
    b = np.asarray(binormal, float)
    return GraspProposal(tcp_xyz=np.asarray(xyz, float),
                         approach=a / np.linalg.norm(a),
                         binormal=b / np.linalg.norm(b), score=score)


# -- geometry ----------------------------------------------------------------

def test_yaw_is_the_smallest_wrist_turn_onto_the_closing_line():
    assert _proposal([0, 0, 0], binormal=(0, 1, 0)).yaw_rad == pytest.approx(0.0)
    assert np.degrees(_proposal([0, 0, 0], binormal=(1, 1, 0)).yaw_rad) == pytest.approx(-45.0)
    # A line, not a direction: 90 deg either way is the same line, and the
    # answer stays inside (-90, 90] rather than commanding a 3/4 wrist turn.
    assert abs(np.degrees(_proposal([0, 0, 0], binormal=(1, 0, 0)).yaw_rad)) == pytest.approx(90.0)
    assert np.degrees(_proposal([0, 0, 0], binormal=(0, -1, 0)).yaw_rad) == pytest.approx(0.0)


def test_tilt_measures_the_angle_from_straight_down():
    assert _proposal([0, 0, 0], approach=(0, 0, -1)).tilt_deg == pytest.approx(0.0)
    assert _proposal([0, 0, 0], approach=(1, 0, -1)).tilt_deg == pytest.approx(45.0)
    assert _proposal([0, 0, 0], approach=(0, 0, 1)).tilt_deg == pytest.approx(180.0)


def test_yawed_orientation_turns_the_fingers_but_not_the_approach():
    for binormal in [(1, 1, 0), (1, 0.2, 0), (0.3, -1, 0)]:
        yaw = _proposal([0, 0, 0], binormal=binormal).yaw_rad
        r = _rodrigues(yawed_orientation(TOPDOWN_RVEC, yaw))
        # Tool +x (the approach axis) still points straight down...
        assert r @ np.array([1.0, 0, 0]) == pytest.approx([0, 0, -1.0], abs=1e-9)
        # ...and tool +y (the finger line) lies along the proposed closing line.
        fingers = r @ np.array([0, 1.0, 0])
        b = np.asarray(binormal, float) / np.linalg.norm(binormal)
        assert abs(float(np.dot(fingers[:2], b[:2]))) == pytest.approx(1.0, abs=1e-9)


# -- envelope ----------------------------------------------------------------

class _FakeClient:
    last_latency_s = 0.05

    def __init__(self, url, proposals):
        self.url = url.rstrip("/")
        self.proposals = proposals

    def propose(self, frame, **kw):
        if isinstance(self.proposals, Exception):
            raise self.proposals
        return self.proposals


def _ctx(tmp_path, proposals):
    cfg = HeronConfig()
    cfg.grasp_service_url = "http://fake:1"
    robot = MockRobot()
    standard_scene(robot)
    ctx = SkillContext(robot=SafeRobot(robot, cfg.safety), belief=None, grounder=None,
                       cfg=cfg, log=EpisodeLogger(tmp_path / "ep", "grasp"))
    ctx.scratch["grasp_client"] = _FakeClient(cfg.grasp_service_url, proposals)
    return ctx


def _events(ctx, kind):
    lines = (ctx.log.dir / "journal.jsonl").read_text().splitlines()
    return [r for r in map(json.loads, lines) if r["kind"] == kind]


def test_envelope_takes_the_best_topdown_proposal_near_the_target(tmp_path):
    target = np.array([0.10, 0.05, 0.03])
    ctx = _ctx(tmp_path, [
        _proposal([0.11, 0.05, 0.025], score=0.6, binormal=(1, 1, 0)),
        _proposal([0.10, 0.06, 0.030], score=0.9),                       # winner
        _proposal([0.10, 0.05, 0.030], score=0.95, approach=(1, 0, -0.5)),  # 63 deg tilt
        _proposal([0.40, 0.30, 0.030], score=0.99),                      # someone else's object
        _proposal([0.10, 0.05, 0.40], score=0.99),                       # floating in the air
    ])
    got = _service_grasp(ctx, "red_block", target)
    assert got is not None
    xyz, yaw, score = got
    assert score == pytest.approx(0.9)
    assert xyz == pytest.approx([0.10, 0.06, 0.030])
    assert yaw == pytest.approx(0.0)
    (ev,) = _events(ctx, "grasp_service_proposal")
    assert ev["in_envelope"] == 2 and ev["proposals"] == 5


def test_envelope_rejects_everything_rather_than_stretch(tmp_path):
    target = np.array([0.10, 0.05, 0.03])
    ctx = _ctx(tmp_path, [
        _proposal([0.10, 0.05, 0.03], score=0.99, approach=(1, 0, -1)),  # 45 deg
    ])
    assert _service_grasp(ctx, "red_block", target) is None
    (ev,) = _events(ctx, "grasp_service_rejected")
    assert ev["proposals"] == 1


# -- fallback doctrine -------------------------------------------------------

def test_a_dead_service_costs_one_log_line_not_the_pick(tmp_path):
    ctx = _ctx(tmp_path, GraspServiceError("connection refused (fake)"))
    result = pick(ctx, "red_block", arm="left", contact=False,
                  at=np.array([0.05, 0.10, 0.02]))  # the mock red_block — since
    # the closed-on-air guard, a pick that grips nothing FAILS, so the fallback
    # path must be aimed at a real object to prove the service outage is benign
    assert result.ok
    (ev,) = _events(ctx, "grasp_service_unavailable")
    assert "refused" in ev["reason"]


def test_unset_url_means_the_service_is_never_consulted(tmp_path):
    ctx = _ctx(tmp_path, GraspServiceError("must not be called"))
    ctx.cfg.grasp_service_url = None
    assert _service_grasp(ctx, "red_block", np.array([0.1, 0.05, 0.03])) is None
    assert not _events(ctx, "grasp_service_unavailable")
    assert not _events(ctx, "grasp_service_rejected")


def test_felt_descend_is_used_when_the_backend_offers_it(tmp_path):
    """Hardware path: one continuous descent, not stepped probing."""
    from heron.skills.primitives import descend_to_contact

    calls = []

    class _Backend:
        def felt_descend(self, arm, xyz_floor, speed, threshold):
            calls.append((arm, tuple(np.round(xyz_floor, 3)), speed, threshold))
            return 0.021, True

    class _Ctx:
        class robot:  # duck-typed: SafeRobot shape
            _robot = _Backend()

            @staticmethod
            def felt_descend(arm, xyz_floor, speed, threshold):
                return _Ctx.robot._robot.felt_descend(arm, xyz_floor, speed, threshold)

            @staticmethod
            def get_external_effort(arm):
                raise AssertionError("stepped probe must not run")

        class log:
            @staticmethod
            def event(*a, **k):
                pass

    z, touched = descend_to_contact(_Ctx, "right", 0.30, 0.10, 0.14, -0.02,
                                    z_expected=None)
    assert touched and z == 0.021 and len(calls) == 1


def test_a_view_that_disagrees_about_the_scene_is_not_fused():
    """Rig 2026-08-06: cam_high put the tabletop at -21 mm and cam_low at +5 mm.

    Concatenated, every surface appeared twice 26 mm apart, and grasp success
    over 400 picks fell from 65-73% single-view to 37% the day multi-view landed.
    """
    import numpy as np

    from heron.perception.graspclient import views_that_agree
    from heron.types import Frame

    def view(name, plane_z):
        # A flat surface at plane_z, straight under a camera 1 m above it.
        k = np.array([[600.0, 0, 320.0], [0, 600.0, 240.0], [0, 0, 1.0]])
        t = np.eye(4)
        t[:3, :3] = np.diag([1.0, -1.0, -1.0])     # looking down
        t[2, 3] = plane_z + 1.0
        return Frame(camera=name, rgb=np.zeros((480, 640, 3), np.uint8),
                     depth=np.full((480, 640), 1.0, np.float32),
                     intrinsics=k, t_base_cam=t)

    anchor = view("cam_high", -0.021)
    agrees = view("cam_side", -0.017)          # 4 mm out: still one scene
    disagrees = view("cam_low", +0.005)        # 26 mm out: a second scene

    keep, dropped = views_that_agree([anchor, agrees], (0.0, 0.0), 0.14)
    assert [f.camera for f in keep] == ["cam_high", "cam_side"], "4 mm is agreement"
    assert dropped == []

    keep, dropped = views_that_agree([anchor, disagrees], (0.0, 0.0), 0.14)
    assert [f.camera for f in keep] == ["cam_high"], "26 mm is not one scene"
    assert dropped and dropped[0][0] == "cam_low"
    assert abs(dropped[0][1] - 0.026) < 0.002

    # The anchor is never dropped, however lonely it is.
    keep, dropped = views_that_agree([disagrees], (0.0, 0.0), 0.14)
    assert [f.camera for f in keep] == ["cam_low"] and dropped == []
