"""The safety box that got it wrong in both directions.

First it was too tight: the configured box stopped at x = -0.45 while the
perturbed goal scenes put a cabinet at x = -0.51, so approaching its drawer was
refused as a safety violation and the repair loop retried the refused motion
until the budget ran out — six episodes, all aborted, none of them touched.

Then it was too loose: widening to the extent of every geom caught the room's
floor plane and produced a box of +/- 3.12 m. That is not a permissive safety
check, it is the absence of one, which is the worse of the two mistakes.
"""
from __future__ import annotations

import numpy as np

from heron.robot.libero import ARM_REACH_M, widened_box

# The perturbed goal scene, measured: cabinet body at x = -0.40 with geoms out to
# -0.51, tabletop objects across y, robot base at (-0.66, 0).
SCENE_MIN = np.array([-0.513, -0.402])
SCENE_MAX = np.array([0.286, 0.432])
BASE = np.array([-0.66, 0.0])
CONFIGURED_X = [-0.45, 0.45]
CONFIGURED_Y = [-0.45, 0.45]


def test_the_box_grows_to_include_the_cabinet_it_used_to_exclude():
    x, _ = widened_box(SCENE_MIN, SCENE_MAX, BASE, CONFIGURED_X, CONFIGURED_Y)
    assert x[0] <= -0.513, "the cabinet's own geometry has to be inside the box"


def test_it_never_narrows_what_was_configured():
    """A config deliberately tight for a real rig must not be loosened, and a
    scene smaller than the box must not shrink it."""
    tiny_min, tiny_max = np.array([-0.05, -0.05]), np.array([0.05, 0.05])
    x, y = widened_box(tiny_min, tiny_max, BASE, CONFIGURED_X, CONFIGURED_Y)
    assert x == (-0.45, 0.45)
    assert y == (-0.45, 0.45)


def test_it_never_extends_past_the_arm_s_reach():
    """The floor plane case: a scene extent of half the room must not become the
    safety box."""
    room_min, room_max = np.array([-3.0, -3.0]), np.array([3.0, 3.0])
    x, y = widened_box(room_min, room_max, BASE, CONFIGURED_X, CONFIGURED_Y)
    assert x[0] >= BASE[0] - ARM_REACH_M
    assert x[1] <= max(CONFIGURED_X[1], BASE[0] + ARM_REACH_M)
    assert y[0] >= BASE[1] - ARM_REACH_M and y[1] <= BASE[1] + ARM_REACH_M
    assert (x[1] - x[0]) < 2.6, f"still a box, not the room: {x}"


def test_the_margin_leaves_room_to_stand_off():
    """An approach starts a standoff away from the object's face, so a box that
    ends exactly at the object refuses the approach to it."""
    x, _ = widened_box(SCENE_MIN, SCENE_MAX, BASE, CONFIGURED_X, CONFIGURED_Y,
                       margin=0.12)
    assert x[0] <= float(SCENE_MIN[0]) - 0.11


def test_a_scene_and_a_config_that_agree_change_nothing():
    x, y = widened_box(np.array([-0.3, -0.3]), np.array([0.3, 0.3]), BASE,
                       CONFIGURED_X, CONFIGURED_Y)
    assert x == (-0.45, 0.45) and y == (-0.45, 0.45)
