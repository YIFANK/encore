"""Cross-episode memory: grasp hints and distilled lessons (Waddle-flavored).

Pigey forgot everything between trials. Heron keeps two small stores:
- hints.json: per-object-description grasp offsets learned from which retries
  finally worked, applied automatically on the next pick of a similar object.
- lessons.json: short natural-language notes distilled from episodes, surfaced
  to the planner prompt so repeated failure modes stop being rediscovered.
"""
from __future__ import annotations

import json
import re
import time

import numpy as np
from pathlib import Path
from typing import Any, Callable, Optional


STOPWORDS = {"the", "a", "an", "of", "with", "and", "on", "in", "to", "that", "this",
             "some", "small", "large", "big", "little"}


def _norm(description: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", description.lower()).strip("_")


def _tokens(description: str) -> set[str]:
    return {t for t in re.split(r"[^a-z0-9]+", description.lower()) if t and t not in STOPWORDS}


POSTMODIFIERS = (" with ", " that ", " containing ", " holding ", " next to ", " on the ")


def _head(description: str) -> str | None:
    """The noun a description is really about.

    English noun phrases are head-final, but a trailing "with ..." describes a
    PART rather than the object — "the white basket with a grey liner" is a
    basket, not a liner. Cutting the postmodifier and taking the last content
    word makes three model-invented wordings of one basket agree.
    """
    text = " " + re.sub(r"[^a-z0-9]+", " ", description.lower()).strip() + " "
    for marker in POSTMODIFIERS:
        idx = text.find(marker)
        if idx > 0:
            text = text[:idx]
    words = [w for w in text.split() if w and w not in STOPWORDS]
    return words[-1] if words else None


# Words that tell two objects of the same KIND apart. One definition, shared
# with the plan audit in agent.py: "plate" and "dish" may name one object;
# "white" and "grey" never do.
COLOUR_WORDS = frozenset({
    "red", "orange", "yellow", "green", "blue", "purple", "pink", "brown",
    "black", "white", "grey", "gray", "silver", "gold", "beige", "tan",
    "teal", "navy", "violet",
})


def _colours(text: str) -> set[str]:
    words = {w.strip(".,;:'\"()").lower()
             for w in str(text).replace("_", " ").split()}
    return {"grey" if w == "gray" else w for w in words & COLOUR_WORDS}


def _best_match(key: str, known: dict, min_overlap: float = 0.25) -> str | None:
    """Find a stored hint for the same object under a different wording.

    Descriptions come from a model and are re-invented every episode — the same
    basket arrives as "the white woven basket" one run and "the woven basket with
    white liner" the next. Keying memory on the exact string means a lesson is
    filed under a name that never recurs, so nothing ever accumulates. Matching
    on shared content words is crude, but it is the difference between a memory
    that works across episodes and one that only looks like it does.
    """
    if key in known:
        return key
    want = _tokens(key.replace("_", " "))
    if not want:
        return None
    head = _head(key.replace("_", " "))
    best, best_score = None, 0.0
    for candidate in known:
        have = _tokens(candidate.replace("_", " "))
        if not have:
            continue
        # The head noun decides identity, and shared modifiers cannot override
        # it: "the white basket with a grey liner" and "the grey liner" have half
        # their words in common and are different objects. Overlap only ranks
        # candidates that already agree on what the thing IS.
        candidate_head = _head(candidate.replace("_", " "))
        if head and candidate_head and head != candidate_head:
            continue
        # DIFFERENT COLOURS, DIFFERENT OBJECTS. The head-noun rule exists so
        # three wordings of one basket agree — but it made "the blue block"
        # inherit "the small orange block"'s learned +20 mm offset, and the
        # rig closed on air 20 mm from a block it had grounded correctly
        # (2026-08-06). A shared kind is not a shared identity.
        kc, cc = _colours(key), _colours(candidate)
        if kc and cc and not (kc & cc):
            continue
        score = len(want & have) / len(want | have)
        if head and head == candidate_head:
            score = max(score, 0.5)
        if score > best_score:
            best, best_score = candidate, score
    return best if best_score >= min_overlap else None


def _substitutions(have: list[str], want: list[str]) -> Optional[dict[str, str]]:
    """Word-for-word replacements turning one sentence into another, or None.

    Deliberately the most conservative alignment there is: same length, same
    order, and every difference a single word swapped for a single word. Two
    sentences that pass this differ only in WHICH things they talk about, which
    is exactly when a plan for one is a plan for the other.

    Anything looser — a reordering, an extra clause, a different verb — is a
    different task. Retargeting a plan onto a sentence it does not fit would
    act confidently on the wrong object, and the planner call it saves is not
    worth that.
    """
    if len(have) != len(want) or not have:
        return None
    subs: dict[str, str] = {}
    for a, b in zip(have, want):
        if a == b:
            continue
        if a in subs and subs[a] != b:
            return None      # the same word would have to become two things
        subs[a] = b
    # Every difference must be a consistent renaming, and the verb frame must
    # survive: if more than half the sentence changed it is not a sister.
    if len(subs) * 2 > len(have):
        return None
    # AND NOTHING MAY BE SWAPPED WITH ANYTHING ELSE. "Put the grey tray in the
    # red block" aligns against "put the red block in the grey tray" word for
    # word. This was first refused because the entity ids stayed put, leaving
    # `block` to describe a tray; `_rename_ids` has since fixed that, and the
    # refusal stands for the reason underneath it. A swap exchanges the ROLES,
    # and the roles are what the plan's structure encodes — reach for the small
    # thing, release it above the large one, at a height chosen for a support
    # with walls. Retargeted onto its own mirror image that structure aims a
    # confident, well-formed plan at a physically absurd act.
    if set(subs) & set(subs.values()):
        return None
    return subs


def _rename_ids(out: dict, subs: dict[str, str]) -> Optional[dict]:
    """Carry the substitution into the entity ids as well. None on a collision.

    An id is a name the plan uses to talk to itself, and the first version of
    this left ids alone on that reasoning. It is wrong twice over.

    Ids are not inert — `_is_flat_support` reads the id text alongside the
    description — and even where a stale word is harmless to the code it is not
    harmless to the reader. A retargeted plan for "move the red block onto the
    white plate" ran under the goal predicate `on(blue_block, grey_plate)`,
    reported that predicate verified, and the rollout player faithfully printed
    it: the operator saw a blue block named in a red block's episode and had no
    way to tell a naming artefact from a grounding failure. Names that lie about
    what they name cost more than they save.

    A rename that would put two entities on one id, or land on an id already in
    use, is refused outright — the plan is then planned fresh rather than
    executed with two things sharing a handle.
    """
    entities = out.get("entities") or []
    rename: dict[str, str] = {}
    for decl in entities:
        old = str(decl.get("id", ""))
        new = "_".join(subs.get(w.lower(), w) for w in old.split("_"))
        if new != old:
            rename[old] = new
    if not rename:
        return out
    ids = {str(d.get("id", "")) for d in entities}
    after = [rename.get(i, i) for i in ids]
    if len(set(after)) != len(ids):
        return None
    # Ids appear as step arguments and as predicate arguments, always as a whole
    # string and never as part of a sentence, so an exact-match walk reaches
    # every reference without touching prose.
    def walk(node):
        if isinstance(node, dict):
            return {k: walk(v) for k, v in node.items()}
        if isinstance(node, list):
            return [walk(v) for v in node]
        if isinstance(node, str):
            return rename.get(node, node)
        return node

    return walk(out)


def _retarget(program: dict, subs: dict[str, str]) -> Optional[dict]:
    """Rewrite a program's object words. None if a substitution cannot be traced.

    Step order, arguments and predicate structure stay exactly as they were;
    what moves is every place the old object words are written down — the entity
    descriptions, the goal sentence, and the entity ids that name them.
    """
    out = json.loads(json.dumps(program))
    entities = out.get("entities") or []
    if not entities:
        return None
    touched = set()
    for decl in entities:
        words = str(decl.get("description", "")).split()
        decl["description"] = " ".join(subs.get(w.lower(), w) for w in words)
        touched.update(w.lower() for w in words if w.lower() in subs)
    # Every word the instruction swapped has to land somewhere. One that does
    # not means the sentences differ by something the entities do not describe
    # — a different container, a different verb — and the plan does not transfer.
    if touched != set(subs):
        return None
    out["goal"] = " ".join(subs.get(w.lower(), w) for w in str(out.get("goal", "")).split())
    return _rename_ids(out, subs)


class MemoryStore:
    def __init__(self, root: str | Path) -> None:
        self.dir = Path(root)
        self.dir.mkdir(parents=True, exist_ok=True)
        self._hints = self._load("hints.json")
        self._places = self._load("place_hints.json") or {}
        self._lessons: list[dict[str, Any]] = self._load("lessons.json") or []
        if not isinstance(self._lessons, list):
            self._lessons = []

    def _load(self, name: str):
        path = self.dir / name
        if path.exists():
            try:
                return json.loads(path.read_text())
            except json.JSONDecodeError:
                return {}
        return {} if name == "hints.json" else []

    # -- programs ------------------------------------------------------------
    #
    # A PROGRAM IS STRUCTURE; POSITIONS ARE VALUES. That is the whole reason a
    # plan can be reused: it names entities and orders steps, and it contains no
    # coordinates at all — those are ground afresh every episode. So the same
    # instruction in a rearranged scene wants the same program.
    #
    # Planning is one call and 14 seconds, on every episode, for a task the
    # planner has already solved. Measured on the twin's clean pick-and-place:
    # 14 s of a 39 s model budget, the single largest item on the happy path.
    #
    # Kept under the same discipline as a learned skill: only a SUCCEEDED
    # episode contributes one, outcomes are recorded, and a program that starts
    # failing is dropped rather than argued with. A cached plan that quietly
    # stopped working would be much worse than the call it saved.
    PROGRAM_MIN_USES_BEFORE_RETIRING = 3
    PROGRAM_MIN_SUCCESS_RATE = 0.5

    def _programs(self) -> dict:
        if not hasattr(self, "_program_cache"):
            self._program_cache = self._load("programs.json") or {}
        return self._program_cache

    @staticmethod
    def _instruction_key(instruction: str) -> str:
        return re.sub(r"[^a-z0-9]+", " ", instruction.lower()).strip()

    def recall_program(self, instruction: str) -> Optional[dict]:
        """The program that worked for this instruction before, or None.

        Falls back to a program written for a SISTER instruction — one that
        differs only in which objects it names — with the object words
        substituted through. "Put the red block in the tray" and "put the blue
        block in the tray" are the same plan; only the fillers change.

        Strictly bounded to single-token substitutions in otherwise identical
        sentences. Anything that reorders, lengthens or restructures is a
        different task and is planned fresh: a plan retargeted onto a sentence
        it does not actually fit would act confidently on the wrong object,
        which is far worse than paying for the planner.
        """
        exact = self._programs().get(self._instruction_key(instruction))
        if exact and not exact.get("retired"):
            return exact.get("program")
        return self._recall_sister(instruction)

    def _recall_sister(self, instruction: str) -> Optional[dict]:
        want = self._instruction_key(instruction).split()
        for entry in self._programs().values():
            if entry.get("retired") or not entry.get("program"):
                continue
            have = self._instruction_key(entry.get("instruction", "")).split()
            subs = _substitutions(have, want)
            if subs is None or not subs:
                continue
            program = _retarget(entry["program"], subs)
            if program is not None:
                return program
        return None

    def remember_program(self, instruction: str, program_json: dict) -> None:
        key = self._instruction_key(instruction)
        entry = self._programs().get(key) or {"uses": 0, "successes": 0}
        entry.update({"program": program_json, "instruction": instruction})
        self._programs()[key] = entry
        self._save_programs()

    def retire_program(self, instruction: str, reason: str = "",
                       audit: Optional[Callable[[dict], bool]] = None) -> list[str]:
        """Drop the cached plans this instruction could be served from.

        The exact entry is not enough. A plan reaches an instruction two ways,
        and the one that did the damage was the SISTER path: a mis-grounded plan
        stored under "move the blue block onto the white plate" was handed to
        "move the red block onto the white plate", which then cached its own
        copy. Retiring only the copy leaves the parent to hand out another.

        `audit` returns True for a plan that should go, and is applied to the
        program AS IT WOULD BE SERVED — retargeted, for a sister. Without it the
        first version of this retired every entry that merely aligned with the
        instruction, including a correct plan that sat one verb away from a
        broken one. Being adjacent to a bad plan is not a defect.
        """
        key = self._instruction_key(instruction)
        want = key.split()
        hit = []
        for k, entry in self._programs().items():
            stored = entry.get("program")
            if not stored:
                continue
            if k == key:
                served = stored
            else:
                subs = _substitutions(self._instruction_key(entry.get("instruction", "")).split(),
                                      want)
                served = _retarget(stored, subs) if subs else None
            if served is None or (audit is not None and not audit(served)):
                continue
            entry["retired"] = True
            if reason:
                entry["retired_because"] = reason
            hit.append(k)
        if hit:
            self._save_programs()
        return hit

    def record_program_outcome(self, instruction: str, ok: bool) -> None:
        key = self._instruction_key(instruction)
        entry = self._programs().get(key)
        if entry is None:
            return
        entry["uses"] = entry.get("uses", 0) + 1
        entry["successes"] = entry.get("successes", 0) + int(bool(ok))
        if (entry["uses"] >= self.PROGRAM_MIN_USES_BEFORE_RETIRING
                and entry["successes"] / entry["uses"] < self.PROGRAM_MIN_SUCCESS_RATE):
            entry["retired"] = True
        self._save_programs()

    def _save_programs(self) -> None:
        (self.dir / "programs.json").write_text(json.dumps(self._programs(), indent=2))

    def _save(self) -> None:
        (self.dir / "hints.json").write_text(json.dumps(self._hints, indent=2))
        (self.dir / "place_hints.json").write_text(json.dumps(self._places, indent=2))
        (self.dir / "lessons.json").write_text(json.dumps(self._lessons, indent=2))

    # -- grasp hints ---------------------------------------------------------
    def grasp_offset(self, description: str) -> tuple[float, float]:
        """Always zero: AIM WHERE PERCEPTION SAYS.

        Learned aim corrections cost this rig two days and never demonstrably
        earned one grasp. Place hints learned from failures walked a plate aim
        63 mm off; grasp hints, keyed by head noun, handed one block another
        block's +20 mm and the gripper closed on air beside a correctly
        grounded 30 mm cube. A systematic aim error belongs in calibration,
        where it can be measured once and checked, not in a per-object memory
        that drifts silently and crosses between objects.

        Outcomes are still RECORDED (record_grasp/record_place) — the numbers
        are worth having; acting on them is what stops here.
        """
        return 0.0, 0.0

    def learned_grasp_offset(self, description: str) -> tuple[float, float]:
        """What a learning scheme WOULD have applied. Telemetry only."""
        h = self._hints.get(_best_match(_norm(description), self._hints) or "")
        return (float(h.get("dx", 0.0)), float(h.get("dy", 0.0))) if h else (0.0, 0.0)

    def record_grasp(self, description: str, dx: float, dy: float, success: bool) -> None:
        key = _best_match(_norm(description), self._hints) or _norm(description)
        h = self._hints.setdefault(key, {"dx": 0.0, "dy": 0.0, "successes": 0, "failures": 0})
        if success:
            h["successes"] += 1
            if abs(dx - h["dx"]) > 1e-4 or abs(dy - h["dy"]) > 1e-4:
                # dx/dy is the TOTAL offset the grasp actually used (stored
                # prior + any retry jitter). Blend toward what worked — which
                # includes drifting back to zero when zero is what worked.
                h["dx"] = round(0.5 * h["dx"] + 0.5 * dx, 4)
                h["dy"] = round(0.5 * h["dy"] + 0.5 * dy, 4)
        else:
            h["failures"] += 1
        self._save()

    # -- place hints ---------------------------------------------------------
    def place_offset(self, description: str) -> tuple[float, float]:
        """Always zero — same doctrine as grasp_offset: aim where perception says."""
        return 0.0, 0.0

    def learned_place_offset(self, description: str) -> tuple[float, float]:
        """What a learning scheme WOULD have applied. Telemetry only."""
        h = self._places.get(_best_match(_norm(description), self._places) or "")
        return (float(h.get("dx", 0.0)), float(h.get("dy", 0.0))) if h else (0.0, 0.0)

    def record_place(self, description: str, residual_dx: float, residual_dy: float,
                     success: bool) -> None:
        """Fold where the object actually landed back into where we aim next time.

        A placement that misses is not just this episode's problem: the miss is
        usually systematic (a container localised slightly off, an object held
        off-centre), so the residual is worth more as a prior for the next
        episode than as a one-off retry. Successes shrink the correction toward
        whatever is currently working; misses push it against the observed error.
        """
        key = _best_match(_norm(description), self._places) or _norm(description)
        h = self._places.setdefault(key, {"dx": 0.0, "dy": 0.0, "successes": 0, "failures": 0})
        if success:
            h["successes"] += 1
            # Success-only learning (the standing doctrine, now applied here
            # too): fold the landing residual in ONLY when the landing was
            # verified. Learning from failures created a death spiral twice —
            # a failure's residual is measured by the same perception that
            # just failed, and each bad lesson caused more failures to learn
            # from: the_white_plate accumulated (+63, -77) mm and every
            # placement missed by a plate-width.
            h["dx"] = round(float(np.clip(h["dx"] - 0.3 * residual_dx, -0.08, 0.08)), 4)
            h["dy"] = round(float(np.clip(h["dy"] - 0.3 * residual_dy, -0.08, 0.08)), 4)
        else:
            h["failures"] += 1
        self._save()

    # -- lessons -------------------------------------------------------------
    def add_lesson(self, text: str, source_episode: str = "") -> None:
        self._lessons.append({"t": time.time(), "text": text.strip(), "episode": source_episode})
        self._lessons = self._lessons[-50:]
        self._save()

    def planner_hints(self, max_items: int = 8) -> str:
        parts = []
        # Grasp hints are deliberately NOT advertised: pick() applies them
        # itself, and a planner told "grasps best offset +0.020" dutifully
        # passes dx=0.02 on top — the offset then lands twice (40 mm miss on a
        # 30 mm block, twin stacking, 2026-08-03). Place hints stay: place has
        # no offset argument, so the planner cannot double-apply them.
        for key, h in self._places.items():
            if abs(h.get("dx", 0)) > 1e-4 or abs(h.get("dy", 0)) > 1e-4:
                parts.append(f"place hint: aim into {key} offset by "
                             f"({h['dx']:+.3f}, {h['dy']:+.3f}) m — earlier drops landed short")
        parts += [l["text"] for l in self._lessons[-max_items:]]
        return "\n".join(f"- {p}" for p in parts[-max_items:]) or "(none yet)"

    # -- distillation --------------------------------------------------------
    def distill_episode(self, journal: list[dict[str, Any]], episode_name: str = "") -> list[str]:
        """Rule-based distillation from a journal. (Grasp offsets are recorded live
        by the agent; this pass extracts episode-level lessons.)"""
        added: list[str] = []
        for rec in journal:
            if rec.get("kind") == "failure" and rec.get("failure_class") == "state_drift":
                lesson = "scene changed mid-episode before; re-perceive destinations right before placing"
                if all(l["text"] != lesson for l in self._lessons):
                    self.add_lesson(lesson, episode_name)
                    added.append(lesson)
            if rec.get("kind") == "perception_override":
                lesson = "overhead grounding has produced false negatives; cross-check with wrist views before trusting a FALSE"
                if all(l["text"] != lesson for l in self._lessons):
                    self.add_lesson(lesson, episode_name)
                    added.append(lesson)
        return added
