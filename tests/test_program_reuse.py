"""A plan written for one object, reused for its sister.

"Put the red block in the tray" and "put the blue block in the tray" are the
same plan; only the fillers change. Reusing it saves the planner call — one call
and 14 seconds, the largest single item on a clean pick-and-place's model budget.

The danger is the opposite of the saving. A plan retargeted onto a sentence it
does not actually fit would act confidently on the WRONG object, so the
alignment is the most conservative one there is: same length, same order, every
difference a single word for a single word, and every substituted word has to
land in an entity description. Anything else is planned fresh.
"""
from __future__ import annotations

from heron.memory import MemoryStore, _retarget, _substitutions

PROGRAM = {
    "goal": "put the red block in the grey tray",
    "entities": [{"id": "block", "description": "the red block"},
                 {"id": "tray", "description": "the grey tray"}],
    "steps": [{"id": "s1", "skill": "perceive", "args": {"entities": ["block", "tray"]}},
              {"id": "s2", "skill": "pick", "args": {"entity": "block"}},
              {"id": "s3", "skill": "place", "args": {"entity": "block", "target": "tray"}}],
    "goal_predicates": [{"name": "in", "args": ["block", "tray"], "negated": False}],
}


def test_a_sister_sentence_reuses_the_plan(tmp_path):
    store = MemoryStore(str(tmp_path))
    store.remember_program("put the red block in the grey tray", PROGRAM)
    got = store.recall_program("put the blue block in the grey tray")
    assert got is not None
    assert [e["description"] for e in got["entities"]] == ["the blue block", "the grey tray"]
    assert got["goal"] == "put the blue block in the grey tray"


def test_the_structure_is_untouched(tmp_path):
    """Order, arguments and predicate structure are what the plan IS, and they
    do not move. Ids here carry no object word, so there is nothing in them to
    retarget — see `test_an_id_that_names_a_colour_is_retargeted_too` for the
    case where there is."""
    store = MemoryStore(str(tmp_path))
    store.remember_program("put the red block in the grey tray", PROGRAM)
    got = store.recall_program("put the blue block in the grey tray")
    assert [s["skill"] for s in got["steps"]] == ["perceive", "pick", "place"]
    assert got["steps"][2]["args"] == {"entity": "block", "target": "tray"}
    assert got["goal_predicates"] == PROGRAM["goal_predicates"]
    assert [e["id"] for e in got["entities"]] == ["block", "tray"]


def test_a_different_container_is_not_a_sister(tmp_path):
    """"grey tray" -> "white plate" swaps a word the entities DO describe, but
    also changes which container the plan aims at — and it is two substitutions
    in a short sentence, so the guard refuses it rather than half-rewriting."""
    store = MemoryStore(str(tmp_path))
    store.remember_program("put the red block in the grey tray", PROGRAM)
    assert store.recall_program("put the red block on the white plate") is None


def test_a_restructured_sentence_is_not_a_sister(tmp_path):
    store = MemoryStore(str(tmp_path))
    store.remember_program("put the red block in the grey tray", PROGRAM)
    for other in ("pick up the red block and put it in the grey tray",
                  "put the red block in the grey tray and then go home",
                  "put the grey tray in the red block"):
        assert store.recall_program(other) is None, other


def test_a_word_that_lands_nowhere_refuses_the_reuse():
    """If a swapped word is not in any entity description, the sentences differ
    by something the plan does not describe — a verb, a preposition — and the
    plan does not transfer."""
    assert _retarget(PROGRAM, {"put": "throw"}) is None


def test_the_alignment_refuses_a_word_that_would_become_two_things():
    assert _substitutions(["a", "a"], ["b", "c"]) is None
    assert _substitutions(["a", "b"], ["x", "y", "z"]) is None
    assert _substitutions([], []) is None


def test_more_than_half_a_sentence_changing_is_not_a_renaming():
    assert _substitutions(["put", "red", "here"], ["throw", "blue", "there"]) is None


def test_a_retired_program_is_not_offered_to_its_sisters(tmp_path):
    store = MemoryStore(str(tmp_path))
    store.remember_program("put the red block in the grey tray", PROGRAM)
    for _ in range(MemoryStore.PROGRAM_MIN_USES_BEFORE_RETIRING):
        store.record_program_outcome("put the red block in the grey tray", False)
    assert store.recall_program("put the blue block in the grey tray") is None


def test_a_swap_is_refused_even_though_the_motion_would_be_right():
    """"Put the grey tray in the red block" aligns against the stored sentence
    word for word. A swap exchanges the ROLES, and the roles are what the plan's
    structure encodes — reach for the small thing, release it above the large
    one, at a height chosen for a support with walls. Retargeted onto its own
    mirror image, that structure aims a well-formed plan at an absurd act."""
    assert _substitutions("put the red block in the grey tray".split(),
                          "put the grey tray in the red block".split()) is None


def test_a_plain_rename_is_still_allowed():
    assert _substitutions("put the red block in the grey tray".split(),
                          "put the blue block in the grey tray".split()) == {"red": "blue"}


NAMED = {
    "goal": "put the red block in the grey tray",
    "entities": [{"id": "red_block", "description": "the red block"},
                 {"id": "grey_tray", "description": "the grey tray"}],
    "steps": [{"id": "s1", "skill": "pick", "args": {"entity": "red_block"}},
              {"id": "s2", "skill": "place",
               "args": {"entity": "red_block", "target": "grey_tray"},
               "pre": [{"name": "visible", "args": ["red_block"], "negated": False}]}],
    "goal_predicates": [{"name": "in", "args": ["red_block", "grey_tray"], "negated": False}],
}


def test_an_id_that_names_a_colour_is_retargeted_too(tmp_path):
    """A red block's plan reused for a blue one must not still call it
    `red_block`. The first version left ids alone, reasoning that an id is only
    a handle the plan uses to talk to itself. Measured against that: an episode
    for "move the red block onto the white plate" ran under the goal predicate
    `on(blue_block, grey_plate)` and reported it verified. Nothing was wrong with
    the motion; the record of it was unreadable, and an operator cannot tell a
    stale name from a grounding failure by looking."""
    store = MemoryStore(str(tmp_path))
    store.remember_program("put the red block in the grey tray", NAMED)
    got = store.recall_program("put the blue block in the grey tray")
    assert [e["id"] for e in got["entities"]] == ["blue_block", "grey_tray"]
    assert got["goal_predicates"][0]["args"] == ["blue_block", "grey_tray"]
    assert got["steps"][1]["args"] == {"entity": "blue_block", "target": "grey_tray"}
    assert got["steps"][1]["pre"][0]["args"] == ["blue_block"]


def test_a_rename_that_would_collide_is_refused():
    """Two entities may not end up sharing one handle. Better to pay the planner
    than to run a plan where two arguments point at the same thing."""
    two = {
        "goal": "swap the red block and the blue block",
        "entities": [{"id": "red_block", "description": "the red block"},
                     {"id": "blue_block", "description": "the blue block"}],
        "steps": [], "goal_predicates": [],
    }
    assert _retarget(two, {"red": "blue"}) is None
