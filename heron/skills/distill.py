"""Turn an episode that worked into a skill the next episode gets for free.

The library in `library.py` has everything needed to keep, expand, score, revise
and retire a learned skill — and until now nothing ever created one. `learn_skill`
was called from the tests and from nowhere else, so `skill_library/` never
existed on disk and every episode started from the same blank vocabulary it had
on the first run. The machinery was a loop with no entry.

This is the entry, and the whole design is in the answer to one question: WHICH
part of a successful episode is worth naming?

Not all of it. A plan that ran clean first time teaches nothing a fresh planner
would not produce again — recording it would fill the library with restatements
of the obvious, and every one of them costs planner-prompt space and has to be
scored and retired later. What is worth keeping is the fragment the agent had to
FIGHT for: a step that failed and was retried with a different offset, a
recovery step the repair loop had to insert first. That is knowledge earned in
this episode which is not in the primitive catalog and not in the planner's
priors, and it is exactly what `SkillLibrary.generalize` was written to preserve.

So the rule is: a contiguous run of steps that finished, that no learned skill
already owns, and that contains at least one step which did not work the first
time.

Everything here is a pure function of the program. No robot, no model, no I/O —
the decision of what is worth learning should be inspectable and testable on its
own, separately from the decision of what to call it.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any, Optional

from ..memory import _head
from ..program import Step, StepStatus, TaskProgram

# A single step is already a named thing — the primitive. Below two steps there
# is no composition to remember.
MIN_STEPS = 2
# Above this the fragment is the plan, not a technique inside it, and a skill
# that large will never match a future task closely enough to be reused.
MAX_STEPS = 6
# Steps whose only job is to look. A fragment that is all perception has no
# technique in it; one that STARTS with perception usually does, and the look is
# part of the lesson ("re-perceive before the second attempt").
SENSING = {"perceive", "inspect"}


@dataclass
class Candidate:
    """A fragment worth naming, and everything needed to name it."""

    steps: list[Step]
    params: dict[str, str] = field(default_factory=dict)   # param name -> value
    earned: list[str] = field(default_factory=list)        # ids of the hard-won steps

    @property
    def signature(self) -> tuple[str, ...]:
        """What this fragment IS, for telling two candidates apart."""
        return tuple(s.skill for s in self.steps)


def _worked_for_it(step: Step) -> bool:
    """Did this step cost the agent something before it worked?

    `attempts > 1` covers the inserted-recovery case too: every repair that
    inserts steps re-runs the step it was inserted before, so a fragment that
    needed a recovery always contains a retried step as well.
    """
    return step.status is StepStatus.DONE and step.attempts > 1


def _param_name(description: str, entity_id: str, taken: set[str]) -> str:
    """A generic name for a specific thing.

    The entity id is episode-local ("red_block_1") and welding it into a skill
    name would produce a skill about that episode. The head noun of the visual
    description is the part that transfers — the same reason cross-episode grasp
    hints are keyed on it rather than on the model's free-form wording.
    """
    base = _head(description) or re.sub(r"[^a-z0-9]+", "_", entity_id.lower()).strip("_")
    base = base or "thing"
    if base not in taken:
        return base
    for i in range(2, 20):
        if f"{base}{i}" not in taken:
            return f"{base}{i}"
    return f"{base}_{len(taken)}"


def _parameters(steps: list[Step], program: TaskProgram) -> dict[str, str]:
    """Which concrete values in this fragment are situation rather than technique.

    Entity ids and the arm are the situation: they change every episode and must
    come back as `$params`. Distances, efforts and heights are the technique —
    a 6 mm backoff is part of what makes the grasp work, not part of which block
    it was — so they stay literal, which is what `generalize` does with anything
    it is not told to abstract.
    """
    described = {e.id: e.description for e in program.entities}
    params: dict[str, str] = {}
    taken: set[str] = set()
    for step in steps:
        for key, value in step.args.items():
            if not isinstance(value, str):
                continue
            if value in described and value not in params.values():
                name = _param_name(described[value], value, taken)
                taken.add(name)
                params[name] = value
            elif key == "arm" and value not in params.values():
                taken.add("arm")
                params["arm"] = value
    return params


def _runs(program: TaskProgram) -> list[list[Step]]:
    """Maximal contiguous stretches of finished, not-already-learned steps."""
    out: list[list[Step]] = []
    current: list[Step] = []
    for step in program.steps:
        keep = step.status is StepStatus.DONE and not step.origin_skill
        if keep:
            current.append(step)
            continue
        if current:
            out.append(current)
        current = []
    if current:
        out.append(current)
    return out


def _window(run: list[Step]) -> list[Step]:
    """Narrow a run to the part that was actually fought for.

    Taking the whole run produces a skill that IS the plan. Measured on the
    offline demo, whose two injected failures are a grasp that closes on air and
    a block snatched out of the gripper: the first version of this kept all six
    completed steps and wrote a skill called `pick_block_place` whose
    description read "perceive then perceive then pick then perceive then pick
    then place". Nothing about that transfers — it is a recording of one episode
    wearing a skill's clothes, and it would sit in the planner prompt forever.

    The lesson lives between the first and last step that had to be retried. One
    preceding sensing step is pulled in with it, because "re-perceive before you
    try again" is very often the whole technique.

    Trailing perception is dropped either way: it is bookkeeping for whatever
    came next, and a skill that ends with a look makes its caller pay for one it
    did not ask for.
    """
    hard = [i for i, s in enumerate(run) if _worked_for_it(s)]
    if not hard:
        return []
    lo, hi = hard[0], hard[-1]
    if lo > 0 and run[lo - 1].skill in SENSING:
        lo -= 1
    window = run[lo:hi + 1]
    while window and window[-1].skill in SENSING:
        window = window[:-1]
    return window[:MAX_STEPS]


def candidates(program: TaskProgram) -> list[Candidate]:
    """Fragments of this program worth promoting into skills, best first."""
    found: list[Candidate] = []
    for whole in _runs(program):
        run = _window(list(whole))
        if len(run) < MIN_STEPS:
            continue
        earned = [s.id for s in run if _worked_for_it(s)]
        if not earned:
            continue          # nothing here a fresh plan would have got wrong
        if all(s.skill in SENSING for s in run):
            continue          # a fragment that only looks has no technique in it
        found.append(Candidate(steps=run, params=_parameters(run, program), earned=earned))
    # Most-fought-for first: that is the fragment whose knowledge is least likely
    # to be reproduced by planning alone.
    found.sort(key=lambda c: (-len(c.earned), -len(c.steps)))
    return found


def already_known(candidate: Candidate, bodies: dict[str, tuple[str, ...]]) -> Optional[str]:
    """The name of a skill that already says this, if there is one.

    Without this the library grows by one near-duplicate per episode —
    `pick_block`, `pick_block_v2`, `pick_block_v3` — each one costing planner
    prompt space and each one accumulating its own thin statistics, so none of
    them ever earns enough evidence to be trusted or retired. Recognising the
    repeat instead lets the existing skill take the credit for it, which is what
    makes its success rate mean something.
    """
    mine = candidate.signature
    for name, seq in bodies.items():
        if seq == mine:
            return name
    return None


def propose_name(candidate: Candidate, existing: set[str]) -> Optional[str]:
    """A readable, stable name for a fragment, or None if one cannot be made.

    Deliberately not a model call. The library is scored and retired on evidence,
    so a name only has to be distinct and legible; spending an API call at the
    end of every successful episode to phrase it better would buy very little and
    would make learning depend on being online.
    """
    verbs = [s.skill for s in candidate.steps if s.skill not in SENSING]
    nouns = [n for n in candidate.params if n != "arm"]
    parts = [verbs[0] if verbs else candidate.steps[0].skill]
    if nouns:
        parts.append(nouns[0])
    if len(verbs) > 1 and verbs[-1] != verbs[0]:
        parts.append(verbs[-1])
    base = re.sub(r"[^a-z0-9_]+", "_", "_".join(parts).lower()).strip("_")
    if len(base) < 3:
        base = f"skill_{base}" if base else None
    if not base:
        return None
    if base not in existing:
        return base
    for i in range(2, 50):
        if f"{base}_v{i}" not in existing:
            return f"{base}_v{i}"
    return None


def describe(candidate: Candidate) -> tuple[str, str]:
    """(description, when_to_use) — the latter is what the planner actually reads."""
    seq = " then ".join(s.skill for s in candidate.steps)
    what = f"{seq}, as it finally worked"
    retried = len(candidate.earned)
    when = (f"the same sequence on a similar object; earned in one episode where "
            f"{retried} of its {len(candidate.steps)} steps only worked after a retry")
    return what, when


def args_for(candidate: Candidate) -> dict[str, Any]:
    """The invocation arguments `generalize` needs to put `$params` back."""
    return dict(candidate.params)
