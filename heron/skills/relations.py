"""Which of the lookalikes is meant — decided by arithmetic, not by adjective.

LIBERO's spatial suite is ten variations on one sentence: "pick up the black
bowl BETWEEN the plate and the ramekin", "NEXT TO the cookie box", "ON the
stove". The bowls are the same bowl. Nothing about their appearance separates
them, so a system that resolves reference by writing a better visual phrase has
already lost — and ours did, in a measurable way: asked for a purely visual
description of "the akita black bowl not between the plate and the ramekin",
the planner answered "the metal bowl on the far right", grasped that, placed it
neatly on the plate, and verified every one of its own postconditions while the
benchmark scored zero. The failure is invisible downstream because everything
downstream is about whichever object got bound.

The fix is to keep the relation as a relation until the anchors have positions,
and then to compute. Scores are all "higher is better", so a negated relation
("NOT between") is the same computation with the sign flipped, which is the
whole reason the task-redefinition suite comes for free.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from ..types import SpatialRelation

# Relations the planner may write. Anything else is refused at plan time rather
# than silently ignored, because a relation that is quietly dropped looks
# exactly like a relation that was honoured.
KINDS = ("between", "near", "far_from", "on", "inside")
ARITY = {"between": 2, "near": 1, "far_from": 1, "on": 1, "inside": 1}

# A candidate this much better than the runner-up is a decision; anything closer
# is a coin flip worth recording. Chosen from the geometry these scenes actually
# have: LIBERO's bowls sit ~0.15 m apart, so a margin under 3 cm means the
# relation did not really separate them.
CLEAR_MARGIN_M = 0.03

# A candidate this close to an anchor, and no higher than it, IS that anchor.
#
# Measured on libero_spatial: "the akita black bowl next to the plate" scored a
# candidate at -0.007 — seven millimetres from the plate's own grounded position
# — and won by 0.29 m over every real bowl. The gripper then closed on the plate
# with 134 mm of measured span, held nothing, and the episode spent eleven
# repairs re-picking the same non-object. Nothing in the arithmetic forbade it,
# because the thing nearest the plate is the plate.
#
# The height clause is what keeps `on` working: a bowl ON a plate does share the
# plate's footprint, and the only thing distinguishing it from the plate is that
# it sits above it.
ANCHOR_SAME_XY_M = 0.04
ANCHOR_SAME_Z_M = 0.01


def _xy(p) -> np.ndarray:
    return np.asarray(p, dtype=float)[:2]


def is_anchor(cand, anchor) -> bool:
    """Is this candidate the anchor itself rather than something near it?"""
    c = np.asarray(cand, dtype=float)
    a = np.asarray(anchor, dtype=float)
    if float(np.linalg.norm(c[:2] - a[:2])) >= ANCHOR_SAME_XY_M:
        return False
    if len(c) > 2 and len(a) > 2 and float(c[2]) - float(a[2]) > ANCHOR_SAME_Z_M:
        return False        # standing above the anchor: a different thing, on it
    return True


def score(kind: str, cand, anchors: list) -> float:
    """How well `cand` fits `kind` with respect to `anchors`. Higher is better.

    Every score is in metres and monotone, so scores are comparable between
    candidates for the same relation (they are never compared ACROSS relations).
    """
    c = _xy(cand)
    a = _xy(anchors[0])

    if kind == "between":
        b = _xy(anchors[1])
        # The ellipse measure: |c-a| + |c-b| - |a-b| is zero exactly on the
        # segment and grows both when the candidate steps off the line and when
        # it slides past either end. One number covers both ways of not being
        # between two things, which a perpendicular distance does not.
        return -float(np.linalg.norm(c - a) + np.linalg.norm(c - b) - np.linalg.norm(a - b))

    if kind == "near":
        return -float(np.linalg.norm(c - a))

    if kind == "far_from":
        return float(np.linalg.norm(c - a))

    if kind in ("on", "inside"):
        # Supported by, or contained in: the same test at this resolution —
        # share the anchor's footprint and sit no lower than it. Height is a
        # tie-break, not the test, because a bowl ON a plate is 2 cm above it
        # and our depth is good to about 1 cm.
        d_xy = float(np.linalg.norm(c - a))
        below = max(0.0, float(anchors[0][2]) - float(cand[2])) if (
            len(np.asarray(cand)) > 2 and len(np.asarray(anchors[0])) > 2) else 0.0
        return -(d_xy + 0.5 * below)

    raise ValueError(f"unknown spatial relation {kind!r}; known: {KINDS}")


def validate(rel: SpatialRelation) -> Optional[str]:
    """Why this relation cannot be used, or None if it can."""
    if rel.kind not in KINDS:
        return f"unknown relation {rel.kind!r}; known: {', '.join(KINDS)}"
    want = ARITY[rel.kind]
    if len(rel.of) != want:
        return f"{rel.kind} needs {want} anchor(s), got {len(rel.of)}: {rel.of}"
    return None


class Choice:
    """The selected candidate and enough about the decision to argue with it."""

    def __init__(self, index: int, best: float, runner_up: Optional[float],
                 note: str) -> None:
        self.index = index
        self.best = best
        self.runner_up = runner_up
        self.note = note

    @property
    def margin(self) -> Optional[float]:
        return None if self.runner_up is None else abs(self.best - self.runner_up)

    @property
    def decisive(self) -> bool:
        return self.margin is None or self.margin >= CLEAR_MARGIN_M


def choose(rel: SpatialRelation, candidates: list, anchors: list) -> Choice:
    """Pick the candidate the relation names.

    `candidates` and `anchors` are world points (3-vectors; the third component
    is used only by `on`/`inside`). Anchors must be in the order `rel.of` names
    them — "between the plate and the ramekin" is symmetric, but "on" is not.
    """
    if not candidates:
        raise ValueError("no candidates to choose between")

    # A relation describes something's position RELATIVE TO the anchors, so an
    # anchor is never the answer to its own relation. Detectors do return them:
    # asked for "the black bowl" on a table holding a plate, one of the four
    # candidates was the plate.
    live = [i for i, c in enumerate(candidates)
            if not any(is_anchor(c, a) for a in anchors)]
    dropped = len(candidates) - len(live)
    if not live:
        # Everything looks like an anchor. Refusing outright would lose the step;
        # say so in the note and decide among what there is.
        live, dropped = list(range(len(candidates))), 0

    scores = [score(rel.kind, candidates[i], anchors) for i in live]
    if rel.negated:
        # "The bowl NOT between the plate and the ramekin". Least-fitting rather
        # than best-fitting; the same numbers read the other way round.
        scores = [-s for s in scores]
    order = sorted(range(len(scores)), key=lambda i: -scores[i])
    best = order[0]
    second = scores[order[1]] if len(order) > 1 else None
    note = (f"{rel} over {len(live)} candidate(s): "
            f"best score {scores[best]:+.3f}"
            + (f", runner-up {second:+.3f}" if second is not None else "")
            + (f"; {dropped} candidate(s) dropped for being an anchor"
               if dropped else ""))
    return Choice(live[best], scores[best], second, note)
