"""Gripper-differencing must find the pinch point and refuse ambiguous frames."""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("cv2", reason="detector needs OpenCV (installed on the robot host)")

from heron.robot.fingertip import pinch_point_from_pair


def _scene(finger_offset: int, pinch=(400, 300)) -> np.ndarray:
    """Table with a static decoy gripper plus two fingers around `pinch`."""
    img = np.full((480, 640, 3), 200, np.uint8)
    img[280:320, 100:140] = 20            # the idle arm's gripper — never moves
    u, v = pinch
    img[v - 15:v + 15, u - finger_offset - 8:u - finger_offset] = 20
    img[v - 15:v + 15, u + finger_offset:u + finger_offset + 8] = 20
    return img


def test_finds_the_pinch_point_between_the_fingers():
    hit = pinch_point_from_pair(_scene(24), _scene(6))
    assert hit is not None
    u, v, conf = hit
    assert abs(u - 400) <= 6 and abs(v - 300) <= 6, f"got {(u, v)}"
    assert conf > 0.5


def test_ignores_the_static_decoy_gripper():
    """The idle arm is identical in both frames, so it cannot pull the centroid."""
    a, b = _scene(24), _scene(6)
    hit = pinch_point_from_pair(a, b)
    assert hit is not None and hit[0] > 300, "centroid drifted toward the idle arm"


def test_rejects_identical_frames():
    assert pinch_point_from_pair(_scene(20), _scene(20)) is None


def test_rejects_a_globally_changed_scene():
    """A brightness jump or a bumped camera must not be read as a fingertip."""
    a = _scene(24)
    b = np.clip(_scene(6).astype(np.int16) + 60, 0, 255).astype(np.uint8)
    assert pinch_point_from_pair(a, b) is None
