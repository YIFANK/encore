"""Placement corrections are learned from VERIFIED landings only.

The first design learned from failures — and a failure's residual is measured
by the same perception that just failed. One bad night compounded
the_white_plate to a (+63, -77) mm "correction" and every subsequent placement
missed by a plate-width, generating more failures to learn from. Success-only
learning is the standing doctrine everywhere else; it applies here too.
"""
from __future__ import annotations

import numpy as np
import pytest

from heron.memory import MemoryStore


def test_a_miss_never_moves_the_aim(tmp_path):
    """Failure residuals are measured by the perception that just failed."""
    m = MemoryStore(tmp_path)
    assert m.learned_place_offset("the basket") == (0.0, 0.0)
    m.record_place("the basket", 0.079, -0.010, success=False)
    assert m.learned_place_offset("the basket") == (0.0, 0.0)


def test_repeated_verified_agreement_converges(tmp_path):
    m = MemoryStore(tmp_path)
    for _ in range(6):
        m.record_place("the basket", 0.05, 0.0, success=True)
    dx, _ = m.learned_place_offset("the basket")
    assert -0.08 <= dx <= -0.02, dx


def test_a_clean_success_leaves_the_aim_alone(tmp_path):
    m = MemoryStore(tmp_path)
    m.record_place("the basket", 0.05, 0.0, success=True)
    before = m.learned_place_offset("the basket")
    m.record_place("the basket", 0.0, 0.0, success=True)
    after = m.learned_place_offset("the basket")
    assert abs(after[0]) <= abs(before[0]), "zero residual only decays the prior"


def test_the_correction_is_bounded(tmp_path):
    """A wild measurement must not fling the aim off the container."""
    m = MemoryStore(tmp_path)
    for _ in range(20):
        m.record_place("the basket", 0.9, -0.9, success=True)
    dx, dy = m.learned_place_offset("the basket")
    assert abs(dx) <= 0.08 and abs(dy) <= 0.08, (dx, dy)


def test_hints_are_per_container_and_survive_a_restart(tmp_path):
    m = MemoryStore(tmp_path)
    m.record_place("the basket", 0.05, 0.0, success=True)
    m.record_place("the white plate", -0.02, 0.03, success=True)
    reloaded = MemoryStore(tmp_path)
    assert reloaded.learned_place_offset("the basket")[0] < 0
    assert reloaded.learned_place_offset("the white plate")[0] > 0
    assert reloaded.learned_place_offset("an unseen bowl") == (0.0, 0.0)


def test_the_planner_is_told_about_a_learned_correction(tmp_path):
    m = MemoryStore(tmp_path)
    m.record_place("the basket", 0.06, 0.0, success=True)
    assert "place hint" in m.planner_hints()


def test_a_paraphrased_description_finds_the_same_hint(tmp_path):
    """Descriptions are re-invented by the model every episode; the object is not."""
    m = MemoryStore(tmp_path)
    m.record_place("the white woven basket", 0.06, 0.0, success=True)
    dx, _ = m.learned_place_offset("the woven basket with white liner")
    assert dx < 0, "the same basket under new wording must inherit the correction"
    # And a genuinely different object must not.
    assert m.learned_place_offset("the red dinner plate") == (0.0, 0.0)


def test_paraphrases_accumulate_into_one_entry(tmp_path):
    m = MemoryStore(tmp_path)
    m.record_place("the white woven basket", 0.05, 0.0, success=True)
    m.record_place("the woven basket with a white liner", 0.05, 0.0, success=True)
    m.record_place("white basket", 0.05, 0.0, success=False)
    hints = MemoryStore(tmp_path)._places
    assert len(hints) == 1, f"memory fragmented across wordings: {list(hints)}"
    counts = next(iter(hints.values()))
    assert counts["successes"] == 2 and counts["failures"] == 1


def test_the_models_three_wordings_of_one_basket_merge(tmp_path):
    """Measured on a real run: ER named the same basket three ways in eight episodes."""
    m = MemoryStore(tmp_path)
    for wording in ("the white basket with grey liner",
                    "the woven basket with white lining",
                    "the square woven basket"):
        m.record_place(wording, 0.05, 0.0, success=False)
    assert len(MemoryStore(tmp_path)._places) == 1, list(MemoryStore(tmp_path)._places)


def test_a_part_does_not_become_the_object(tmp_path):
    """'basket with a grey liner' is a basket; a liner on its own is not."""
    m = MemoryStore(tmp_path)
    m.record_place("the white basket with grey liner", 0.05, 0.0, success=False)
    assert m.learned_place_offset("the grey liner") == (0.0, 0.0)
    assert m.learned_place_offset("the red dinner plate") == (0.0, 0.0)


def test_a_colour_never_inherits_another_colours_offset(tmp_path):
    """The blue block closed on air 20 mm out because the head-noun rule let
    it inherit the orange block's learned offset. A shared kind is not a
    shared identity."""
    from heron.memory import MemoryStore

    m = MemoryStore(str(tmp_path))
    m.record_grasp("the small orange block", 0.02, 0.0, True)
    learned = m.learned_grasp_offset("the small orange block")
    assert learned[0] > 0.0                      # it learned something
    assert m.learned_grasp_offset("the blue block") == (0.0, 0.0)   # and kept it to itself
    assert m.grasp_offset("the small orange block") == (0.0, 0.0)  # never APPLIED
    # Same object, different wording, still matches.
    assert m.learned_grasp_offset("the orange block") == learned
