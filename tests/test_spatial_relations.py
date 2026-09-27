"""Reference by geometry, for the objects appearance cannot separate.

The measured failure these pin down: on "pick the akita black bowl NOT between
the plate and the ramekin", the planner was required to emit a purely visual
description, could not find one (the bowls are identical), and invented "the
metal bowl on the far right". It grasped that, placed it neatly on the plate,
verified all of its own postconditions, and the benchmark scored zero — three
episodes running. Nothing downstream can catch a wrong binding, so the binding
has to be right.

Coordinates below are the ones actually recorded in that episode's journal.
"""
from __future__ import annotations

import numpy as np

from heron.skills import relations as R
from heron.types import EntityDecl, SpatialRelation

# From episodes/20260801-002018-…-ep2/journal.jsonl.
PLATE = np.array([0.088, 0.1855, 0.9092])
RAMEKIN = np.array([-0.1956, 0.1837, 0.9286])
# The bowl the run actually grasped — well off the plate–ramekin line.
CHOSEN = np.array([-0.2078, 0.3415, 0.9234])
# A bowl sitting between the two anchors, which is what "between" names.
BETWEEN = np.array([-0.055, 0.185, 0.923])


def rel(kind, *of, negated=False):
    return SpatialRelation(kind=kind, of=list(of), negated=negated)


def test_between_picks_the_bowl_on_the_segment():
    c = R.choose(rel("between", "plate", "ramekin"), [CHOSEN, BETWEEN], [PLATE, RAMEKIN])
    assert c.index == 1
    assert c.decisive


def test_negation_picks_the_other_one_from_the_same_numbers():
    """'not between' must not need its own scoring path — only a sign."""
    yes = R.choose(rel("between", "plate", "ramekin"), [CHOSEN, BETWEEN], [PLATE, RAMEKIN])
    no = R.choose(rel("between", "plate", "ramekin", negated=True),
                  [CHOSEN, BETWEEN], [PLATE, RAMEKIN])
    assert yes.index == 1 and no.index == 0
    assert no.margin == yes.margin      # same separation, read the other way


def test_between_is_symmetric_in_its_anchors():
    a = R.choose(rel("between", "plate", "ramekin"), [CHOSEN, BETWEEN], [PLATE, RAMEKIN])
    b = R.choose(rel("between", "ramekin", "plate"), [CHOSEN, BETWEEN], [RAMEKIN, PLATE])
    assert a.index == b.index


def test_between_rejects_a_candidate_beyond_the_far_anchor():
    """Collinear but outside the span is not between. A perpendicular-distance
    test would score this a perfect zero, which is why the score is the ellipse
    measure and not the distance to the line."""
    beyond = RAMEKIN + (RAMEKIN - PLATE) * 0.5      # on the line, past the ramekin
    c = R.choose(rel("between", "plate", "ramekin"), [beyond, BETWEEN], [PLATE, RAMEKIN])
    assert c.index == 1
    assert c.decisive


def test_near_and_far_from_are_opposites():
    near = R.choose(rel("near", "ramekin"), [CHOSEN, BETWEEN], [RAMEKIN])
    far = R.choose(rel("far_from", "ramekin"), [CHOSEN, BETWEEN], [RAMEKIN])
    assert near.index != far.index


def test_on_prefers_shared_footprint_over_mere_proximity():
    """'the bowl on the plate' is about being above it, not beside it."""
    on_it = PLATE + np.array([0.005, 0.005, 0.03])
    beside = PLATE + np.array([0.12, 0.0, 0.0])
    c = R.choose(rel("on", "plate"), [beside, on_it], [PLATE])
    assert c.index == 1


def test_a_tie_is_reported_rather_than_hidden():
    """Two bowls equidistant from the anchor: still answered, but not decisive —
    a later failure should be able to read that the geometry did not separate
    them, instead of inferring it."""
    a = RAMEKIN + np.array([0.10, 0.0, 0.0])
    b = RAMEKIN + np.array([-0.10, 0.0, 0.0])
    c = R.choose(rel("near", "ramekin"), [a, b], [RAMEKIN])
    assert not c.decisive
    assert c.margin < 1e-6


def test_one_candidate_is_decisive_because_there_is_nothing_to_confuse_it_with():
    c = R.choose(rel("between", "plate", "ramekin"), [BETWEEN], [PLATE, RAMEKIN])
    assert c.index == 0 and c.margin is None and c.decisive


def test_wrong_arity_is_refused_not_guessed():
    assert R.validate(rel("between", "plate")) is not None
    assert R.validate(rel("near", "plate", "ramekin")) is not None
    assert R.validate(rel("between", "plate", "ramekin")) is None


def test_unknown_kind_is_refused():
    assert R.validate(SpatialRelation(kind="left_of", of=["plate"])) is not None


def test_entity_declaration_round_trips_the_relation():
    d = EntityDecl(id="bowl_target", description="the black bowl",
                   relation=rel("between", "plate", "ramekin", negated=True))
    again = EntityDecl.model_validate_json(d.model_dump_json())
    assert again.relation.negated and again.relation.of == ["plate", "ramekin"]
    assert str(again.relation) == "not between(plate, ramekin)"


def test_entities_without_a_relation_stay_unchanged():
    d = EntityDecl(id="plate", description="the white plate")
    assert d.relation is None


# -- an anchor is never the answer to its own relation -------------------------

def test_the_thing_nearest_the_plate_is_not_the_plate():
    """Measured on libero_spatial: `near(plate)` scored a candidate 7 mm from
    the plate's own grounded position and beat every real bowl by 0.29 m. The
    gripper closed on the plate, held nothing, and the episode burned eleven
    repairs re-picking it."""
    from heron.skills.relations import choose
    from heron.types import SpatialRelation

    plate = np.array([0.074, 0.037, 0.908])
    cands = [
        plate + np.array([0.001, 0.007, 0.000]),   # the plate itself, as detected
        np.array([0.030, 0.366, 0.924]),           # a real bowl, further away
        np.array([-0.111, 0.268, 0.925]),          # another real bowl
    ]
    got = choose(SpatialRelation(kind="near", of=["plate"]), cands, [plate])
    assert got.index != 0, "chose the anchor as its own referent"
    nearest = 1 + int(np.argmin([np.linalg.norm(c[:2] - plate[:2]) for c in cands[1:]]))
    assert got.index == nearest, "should be the nearest thing that is not the plate"
    assert "dropped for being an anchor" in got.note


def test_a_bowl_on_a_plate_is_still_allowed_to_be_on_it():
    """`on` is the case where sharing the anchor's footprint is the whole point.
    The exclusion must key on height, not on distance alone, or every `on(...)`
    task loses its answer."""
    from heron.skills.relations import choose
    from heron.types import SpatialRelation

    stove = np.array([0.20, -0.10, 0.900])
    on_it = stove + np.array([0.005, 0.004, 0.030])     # a bowl resting on it
    elsewhere = np.array([0.20, 0.25, 0.905])
    got = choose(SpatialRelation(kind="on", of=["stove"]), [on_it, elsewhere], [stove])
    assert np.allclose([on_it, elsewhere][got.index], on_it)


def test_a_candidate_at_the_anchor_and_no_higher_is_the_anchor():
    from heron.skills.relations import ANCHOR_SAME_XY_M, is_anchor

    a = np.array([0.20, -0.10, 0.900])
    assert is_anchor(a + np.array([0.005, 0.0, 0.0]), a)
    assert is_anchor(a + np.array([0.005, 0.0, 0.005]), a), "inside the depth noise"
    assert not is_anchor(a + np.array([0.005, 0.0, 0.030]), a), "sitting on it"
    assert not is_anchor(a + np.array([ANCHOR_SAME_XY_M + 0.001, 0.0, 0.0]), a)


def test_everything_looking_like_an_anchor_still_returns_an_answer():
    """Refusing outright would lose the step. Decide among what there is and say
    in the note that the filter could not be applied."""
    from heron.skills.relations import choose
    from heron.types import SpatialRelation

    a = np.array([0.20, -0.10, 0.900])
    got = choose(SpatialRelation(kind="near", of=["x"]),
                 [a + np.array([0.001, 0, 0]), a + np.array([0, 0.002, 0])], [a])
    assert got.index in (0, 1)
    assert "dropped" not in got.note


def test_the_anchor_filter_survives_negation():
    """"the bowl NOT next to the plate" must not answer with the plate either —
    negation flips which candidate wins, not which candidates exist."""
    from heron.skills.relations import choose
    from heron.types import SpatialRelation

    plate = np.array([0.074, 0.037, 0.908])
    cands = [plate.copy(), np.array([0.08, 0.10, 0.92]), np.array([0.5, 0.5, 0.92])]
    got = choose(SpatialRelation(kind="near", of=["plate"], negated=True), cands, [plate])
    assert got.index == 2, "the farthest non-anchor candidate"
