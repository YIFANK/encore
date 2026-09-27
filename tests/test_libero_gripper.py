"""LIBERO's gripper is binary, so a commanded WIDTH has to be read as an intent.

robosuite takes -1 (open) or +1 (close); Heron's primitives speak metres. The
translation is a threshold, and a threshold is only safe if it separates every
width the codebase actually commands. It did not: it sat at 0.05 m, wider than
some objects the arm is asked to hold, and `place` now opens only as far as the
held object needs instead of always to the full 80 mm. Releasing a 20 mm object
asks for 44 mm — which the old threshold read as CLOSE, so the place would have
squeezed instead of letting go, on a backend where nothing reports an error.

This enumerates the call sites rather than testing the number, so adding a new
one with a width in the wrong band fails here instead of in a sweep.
"""
from __future__ import annotations

import pytest

from heron.robot.libero import GRIP_INTENT_WIDTH_M
from heron.skills.primitives import OPEN_WIDTH, release_width


def intent(width_m: float) -> str:
    """What the backend will do with this width."""
    return "close" if width_m < GRIP_INTENT_WIDTH_M else "open"


# Every width heron/ commands, with what the caller meant by it.
COMMANDED = [
    (0.0, "close", "pick: grasp"),
    (0.015, "close", "open_drawer: take the handle"),
    (0.06, "open", "open_drawer: let the handle go"),
    (OPEN_WIDTH, "open", "pick: open before descending / place: legacy full release"),
    (0.08, "open", "explicit full open"),
]


@pytest.mark.parametrize("width,want,why", COMMANDED)
def test_every_commanded_width_means_what_the_caller_meant(width, want, why):
    assert intent(width) == want, f"{why}: {width} m read as {intent(width)}"


@pytest.mark.parametrize("held", [0.006, 0.010, 0.020, 0.029, 0.045, 0.070])
def test_releasing_anything_the_gripper_can_hold_opens_it(held):
    """`holding_something` accepts 6 to 75 mm, so those are the widths `place`
    can be asked to let go of. Every one of them must open."""
    assert intent(release_width(held)) == "open", (
        f"releasing a {held * 1000:.0f} mm object asks for "
        f"{release_width(held) * 1000:.0f} mm, which this backend would grip")


def test_the_threshold_sits_between_gripping_and_releasing():
    """A deliberate squeeze on a drawer handle (15 mm) is a grip; the narrowest
    release (a 6 mm object, 30 mm of aperture) is not. The threshold has to fall
    strictly between, and this says how much room is left on each side."""
    assert 0.015 < GRIP_INTENT_WIDTH_M < release_width(0.006)
