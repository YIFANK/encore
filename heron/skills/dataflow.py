"""Run a task as a graph of values instead of a list of independent skills.

The difference is not stylistic. In the list form each skill arranges its own
perception, so nobody knows how many detector calls a task will make until it
has made them — measured: 23 for two objects, 127 s of a 207-second episode,
and a long-horizon run where a tray's position "went stale" and had to be
re-found although it had never moved.

Here a look produces VALUES, and the steps that need them are handed them:

    plan = [
        Look(["the red block", "the white plate"]),
        Pick("the red block"),
        Place("the red block", onto="the white plate"),
    ]

`cost()` answers "how many detector calls is this?" before anything moves,
because the answer is a property of the graph rather than of what each skill
decides in private. Nothing here replaces the agent loop — verification,
diagnosis and repair are unchanged. What changes is where perception happens.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

from ..types import SkillResult
from . import SkillContext
from .values import Located, Perception, look


@dataclass
class Node:
    """One stage. `needs` names the values it consumes."""

    kind: str
    args: dict[str, Any] = field(default_factory=dict)
    needs: list[str] = field(default_factory=list)

    def detector_calls(self) -> int:
        """What this stage costs. A look is ONE request however many things it
        asks about — the model reads the whole frame either way."""
        return 1 if self.kind == "look" and self.args.get("queries") else 0


def Look(queries: list[str], camera: Optional[str] = None) -> Node:
    return Node("look", {"queries": list(queries), "camera": camera})


def Pick(query: str, arm: str = "auto") -> Node:
    return Node("pick", {"query": query, "arm": arm}, needs=[query])


def Place(query: str, onto: str, arm: str = "auto") -> Node:
    return Node("place", {"query": query, "onto": onto, "arm": arm}, needs=[onto])


@dataclass
class Outcome:
    ok: bool
    detector_calls: int
    steps: list[tuple[str, bool, str]] = field(default_factory=list)
    seen: dict[str, Located] = field(default_factory=dict)


def cost(nodes: list[Node]) -> int:
    """Detector calls this plan will make. Known before it runs."""
    return sum(n.detector_calls() for n in nodes)


def missing_values(nodes: list[Node]) -> list[str]:
    """Names consumed before anything produced them.

    A plan that picks something it never looked for is malformed, and saying so
    up front is cheaper than discovering it three motions in.
    """
    have: set[str] = set()
    gaps: list[str] = []
    for n in nodes:
        for q in n.needs:
            if q not in have:
                gaps.append(q)
        if n.kind == "look":
            have.update(n.args["queries"])
    return gaps


def run(ctx: SkillContext, nodes: list[Node]) -> Outcome:
    """Execute the graph, threading values from look() into the motions."""
    gaps = missing_values(nodes)
    if gaps:
        raise ValueError(f"plan uses {gaps} before any look produced them")

    from .primitives import pick, place  # noqa: PLC0415

    seen: dict[str, Located] = {}
    out = Outcome(ok=True, detector_calls=0)
    ctx.log.event("dataflow_start", steps=[n.kind for n in nodes],
                  detector_calls_planned=cost(nodes))

    for n in nodes:
        if n.kind == "look":
            got: Perception = look(ctx, n.args["queries"], camera=n.args.get("camera"))
            seen.update(got.seen)
            out.detector_calls += got.calls
            found = sorted(got.seen)
            ok = len(found) == len(n.args["queries"])
            out.steps.append(("look", ok, f"found {found}"))
            if not ok:
                out.ok = False
                missing = [q for q in n.args["queries"] if q not in got.seen]
                out.steps.append(("look", False, f"never found {missing}"))
                break
            continue

        query = n.args["query"]
        try:
            if n.kind == "pick":
                res: SkillResult = pick(ctx, entity=query, arm=n.args["arm"],
                                        at=seen.get(query))
            else:
                res = place(ctx, entity=query, arm=n.args["arm"],
                            at=seen.get(n.args["onto"]))
        except Exception as e:
            out.steps.append((n.kind, False, f"{type(e).__name__}: {e}"))
            out.ok = False
            break
        out.steps.append((n.kind, bool(res.ok), res.info or res.error or ""))
        if not res.ok:
            out.ok = False
            break
        # The object moved, so what we knew about it is now historical. Say so
        # rather than letting a stale value be handed to a later step.
        seen.pop(query, None)

    out.seen = seen
    ctx.log.event("dataflow_end", ok=out.ok, detector_calls=out.detector_calls,
                  steps=[(k, ok) for k, ok, _ in out.steps])
    return out


# ---------------------------------------------------------------------------
# Reading an ordinary TaskProgram as a graph.
#
# The planner writes a LIST of typed steps, and it does not need to change: the
# dependencies are already in the arguments. `perceive` says which entities it
# grounds; `pick` and `place` name the entity and the target they act on. So the
# same three questions the graph form can answer — what will this cost, does it
# use anything nobody looked for, and when does a value stop being valid — can be
# answered about a program that was never written as a graph.
#
# What this deliberately does NOT do is hand values to the VERIFIERS. A verifier
# exists to disagree with the plan, and a check fed the number that decided the
# action cannot: it would confirm the action against its own premise. Actions
# consume values; verification looks for itself.

# Which argument of each skill names a thing it needs a position for.
CONSUMES = {"pick": ("entity",), "place": ("entity", "target"), "push": ("entity",),
            "open_drawer": ("entity", "cabinet"), "close_drawer": ("entity", "cabinet")}
# ...and which it leaves somewhere new, so the value stops being true.
MOVES = {"pick": ("entity",), "place": ("entity",), "push": ("entity",),
         "open_drawer": ("entity",), "close_drawer": ("entity",)}
# Steps that produce positions, and what each costs in detector round trips.
# A look is ONE request however many things it asks about.
PRODUCES = {"perceive": "entities", "inspect": "entity"}


def _named(step, key: str) -> list[str]:
    v = step.args.get(key)
    if isinstance(v, str):
        return [v]
    if isinstance(v, list):
        return [x for x in v if isinstance(x, str)]
    return []


def produces(step) -> list[str]:
    key = PRODUCES.get(step.skill)
    if key is None:
        return []
    got = _named(step, key)
    # `perceive` treats an empty list and the literal ["all"] alike: both ground
    # the whole scene. Reading only the first as "everything" made the commonest
    # plan in the corpus — perceive(all), pick, place — look like it acted on
    # things nobody had looked for.
    if not got or got == ["all"]:
        return ["*"]
    return got


def consumes(step) -> list[str]:
    return [e for k in CONSUMES.get(step.skill, ()) for e in _named(step, k)]


def moves(step) -> list[str]:
    return [e for k in MOVES.get(step.skill, ()) for e in _named(step, k)]


def program_cost(program) -> int:
    """Detector round trips this program will make, known before it runs.

    Only the looks are counted, because only looks cost a request — and that is
    the point: with perception in the plan the number is a property of the plan
    rather than of what each skill decides in private.
    """
    return sum(1 for s in program.steps if s.skill in PRODUCES)


def unresolved(program) -> list[tuple[str, str]]:
    """(step id, entity) for every action that acts on something never looked for.

    A plan that picks what it never located is malformed, and saying so before
    anything moves is cheaper than discovering it three motions in.
    """
    have: set[str] = set()
    gaps: list[tuple[str, str]] = []
    for step in program.steps:
        if "*" not in have:
            for e in consumes(step):
                if e not in have:
                    gaps.append((step.id, e))
        have.update(produces(step))
    return gaps


def sort_by_colour(targets: dict[str, str]) -> list[Node]:
    """A long-horizon plan: every object located ONCE, then moved.

    `targets` maps each object's description to its destination's description.
    Four placements cost one look here and four in the list form — and the gap
    widens with every extra object.
    """
    queries = list(dict.fromkeys([*targets.keys(), *targets.values()]))
    nodes: list[Node] = [Look(queries)]
    for obj, dest in targets.items():
        nodes.append(Pick(obj))
        nodes.append(Place(obj, onto=dest))
    return nodes
