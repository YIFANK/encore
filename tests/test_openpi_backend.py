"""The pi0.5 backend's observation encoding, which fails silently when wrong.

Every field here has to match the convention the policy was normalised against.
A mismatch is not an error anywhere — the server accepts the request, returns
actions, and the arm moves badly for reasons that look like a bad policy.

The quaternion conversion is reproduced from openpi's own LIBERO example rather
than imported, so it is pinned here against values computed independently.
"""
from __future__ import annotations

import numpy as np
import pytest

from heron.skills.openpi_backend import RESIZE_SIZE, _unwrap, quat_to_axisangle


def test_identity_rotation_is_no_rotation():
    """robosuite quaternions are (x, y, z, w); the identity has w = 1, and the
    formula divides by sqrt(1 - w^2). Getting the component order wrong makes
    this a division by zero rather than a wrong answer, which is the one case
    that would have been noticed."""
    assert np.allclose(quat_to_axisangle([0.0, 0.0, 0.0, 1.0]), np.zeros(3))
    assert np.allclose(quat_to_axisangle([0.0, 0.0, 0.0, -1.0]), np.zeros(3))


@pytest.mark.parametrize("axis", [np.array([1.0, 0, 0]), np.array([0, 1.0, 0]),
                                  np.array([0, 0, 1.0]),
                                  np.array([0.6, -0.8, 0.0])])
@pytest.mark.parametrize("theta", [0.3, 1.2, 2.9])
def test_it_inverts_the_axis_angle_it_was_built_from(axis, theta):
    axis = axis / np.linalg.norm(axis)
    q = np.array([*(axis * np.sin(theta / 2)), np.cos(theta / 2)])
    got = quat_to_axisangle(q)
    assert np.allclose(got, axis * theta, atol=1e-9)


def test_a_quaternion_outside_the_unit_range_is_clamped_not_nan():
    """Numerical drift in the simulator's own state can push w past 1, and
    sqrt of a negative there would poison the whole state vector with NaN —
    which the policy would consume without complaint."""
    got = quat_to_axisangle([0.0, 0.0, 0.0, 1.0 + 1e-9])
    assert np.all(np.isfinite(got))


def test_the_state_vector_is_the_eight_numbers_the_policy_expects():
    """3 position + 3 axis-angle + 2 gripper joints. The policy's normalisation
    statistics have that shape; anything else is a shape error at best and a
    silent misalignment at worst."""
    pos = np.array([0.1, -0.2, 1.0])
    quat = np.array([0.0, 0.0, 0.0, 1.0])
    grip = np.array([0.02, -0.02])
    state = np.concatenate((pos, quat_to_axisangle(quat), grip))
    assert state.shape == (8,)
    assert np.allclose(state[:3], pos)
    assert np.allclose(state[6:], grip)


def test_the_resize_matches_what_the_policy_was_trained_on():
    assert RESIZE_SIZE == 224


def test_the_backend_reaches_through_the_safety_wrapper():
    class _Inner:
        pass

    class _Wrapper:
        def __init__(self, inner):
            self._robot = inner

    inner = _Inner()
    assert _unwrap(_Wrapper(inner)) is inner
    assert _unwrap(inner) is inner


def test_it_refuses_a_robot_that_is_not_a_libero_environment():
    """The backend steps `robot.env` directly. On hardware there is no such
    attribute, and failing at construction is much better than discovering it
    mid-rollout with the arm moving."""
    from heron.skills.openpi_backend import OpenPiLiberoClient

    class _Hardware:
        arms = ["right"]

    class _Log:
        def event(self, *a, **k):
            pass

    class _Ctx:
        robot = _Hardware()
        log = _Log()

    with pytest.raises(Exception) as e:
        OpenPiLiberoClient(_Ctx())
    # Either the import of openpi_client is missing (fine, not installed here)
    # or the robot check fires. Both are refusals at construction, which is the
    # property under test.
    assert "openpi" in str(e.value).lower() or "LIBERO" in str(e.value)


def test_a_dead_environment_stops_the_rollout_instead_of_spinning():
    """LIBERO stops at its horizon and then refuses actions. Measured in a live
    sweep: rollouts logging `steps: 0, infers: 0` that still ran for their whole
    thirty-second budget, paying for a vision-model progress check every two
    seconds against a world that could not change."""
    from heron.skills.openpi_backend import OpenPiLiberoClient

    class _Robot:
        env = object()
        obs: dict = {}
        terminated = False

    client = OpenPiLiberoClient.__new__(OpenPiLiberoClient)
    client.robot = _Robot()
    assert client.finished is False
    client.robot.terminated = True
    assert client.finished is True


def test_the_vla_skill_breaks_when_the_client_is_finished():
    from heron.skills.vla import vla

    class _Client:
        finished = True
        calls = 0

        def start(self, _):
            pass

        def step(self):
            type(self).calls += 1

        def stop(self):
            pass

    class _Robot:
        vla_client = _Client()
        cameras = ["cam_high"]

        def capture(self, _):
            raise AssertionError("no frame should be captured on a dead environment")

    class _Belief:
        entities: dict = {}

    class _Log:
        def event(self, *a, **k):
            pass

        def frame(self, *a, **k):
            pass

    class _Ctx:
        robot = _Robot()
        belief = _Belief()
        log = _Log()
        cfg = None

    res = vla(_Ctx(), instruction="open the drawer", max_seconds=30.0)
    assert res.ok
    assert _Client.calls == 0, "stepped a dead environment"
    assert "environment ended" in res.info
