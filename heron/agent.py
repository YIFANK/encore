"""The Heron agent loop.

    compile program -> for each step:
        verify preconditions (gather information if unknown)
        execute skill
        verify postconditions
        on failure: diagnose -> classify -> repair locally (rules first,
        orchestrator for the hard cases) under explicit budgets
    finally: verify goal predicates, distill memory, close the journal.

Everything the model decides passes through typed objects; everything the
robot does leaves evidence. No decision hides in chat history.
"""
from __future__ import annotations

import re
import time
from typing import Any, Callable, Optional

import numpy as np

from .belief import BeliefStore
from .config import HeronConfig
from .diagnose import diagnose_exec_error, diagnose_postcondition, diagnose_precondition, escalate
from .episode import EpisodeLogger, read_journal
from .memory import MemoryStore
from .perception import SamSegmenter
from .program import EditType, ProgramEdit, Step, StepStatus, TaskProgram
from .repair import rule_based_repair, validate_edit
from .skills import SkillContext, load_all
from .skills import dataflow, distill
from .skills.sensing import forget_the_look, ground_entity
from .skills.library import SkillLibrary
from .types import (FailureClass, FailureReport, Frame, PredicateSpec,
                    SpatialRelation, Value)
from .verify import check

# A confident independent contradiction overturns a claimed success.
GOAL_VETO_MIN_CONF = 0.7
# Below this, a geometric goal verdict has to be corroborated before the task is
# called finished. `verify.margin_confidence` puts a verdict here exactly when it
# cleared its threshold by less than the error in the estimates it was computed
# from — which is when a second opinion stops being a formality.
GOAL_NEEDS_BACKUP_CONF = 0.6

# Words that tell two objects of the same kind apart, and the only class of them
# safe to compare literally. "Plate" and "dish" may name one object; "white" and
# "grey" never do. So a colour the instruction used and the plan's graded
# entities do not mention is a fact about the plan, not a guess about wording.
from .memory import COLOUR_WORDS  # one definition, shared with hint matching


def _colours(text: str) -> set[str]:
    words = {w.strip(".,;:'\"()").lower() for w in str(text).replace("_", " ").split()}
    return {"grey" if w == "gray" else w for w in words & COLOUR_WORDS}


def misgrounded_colours(instruction: str, program: TaskProgram) -> list[str]:
    """Colours the task named that the plan's GRADED entities do not mention.

    Module-level because it is also how a cached plan is audited without running
    it — see `_reject_misgrounded_goal` for what it caught and why it is this
    narrow.
    """
    want = _colours(instruction) | _colours(program.goal)
    if not want:
        return []
    described = {e.id: e.description for e in program.entities}
    seen: set[str] = set()
    for spec in program.goal_predicates:
        for name in spec.args:
            seen |= _colours(name) | _colours(described.get(name, ""))
    return sorted(want - seen)



def _self_defeating(program: TaskProgram) -> bool:
    """Does any step re-achieve a state the goal explicitly negates?"""
    undo = {(p.args[0], p.args[1]) for p in program.goal_predicates
            if p.negated and p.name in ("on", "in") and len(p.args) >= 2}
    if not undo:
        return False
    for st in program.steps:
        if st.skill == "place" and (st.args.get("entity"),
                                    st.args.get("target")) in undo:
            return True
    return False


class _HumanVerdict(BaseException):
    # BaseException, deliberately: the skill registry converts Exception into
    # a failed SkillResult, which would swallow a verdict raised mid-skill.
    # As BaseException it sails through every generic handler up to run()'s
    # explicit catch — the operator's word stops the arm within one motion.
    def __init__(self, verdict: str):
        self.verdict = verdict


def _reachable_twin(want: str, objects, reach) -> str | None:
    """Another object of the same kind that the arm can actually get to.

    Same kind means it shares the content words that are not about which one:
    large_white_plate and small_white_plate are both white plates, and if the
    model picked the one out of the envelope the other is what it would have
    picked had it known.
    """
    keep = _words(want) - {"large", "small", "left", "right", "front", "back",
                           "near", "far", "upper", "lower"}
    best = None
    for o in objects:
        if o["id"] == want or not reach.get(o["id"], False):
            continue
        if keep and keep <= _words(o["id"]):
            best = best or o["id"]
    return best


def _predicate(text: str):
    """'on(a, b)' -> ('on', ['a', 'b']); None if it is not one."""
    t = str(text).strip()
    if t.lower().startswith("not "):
        t = t[4:].strip()
    if "(" not in t or not t.endswith(")"):
        return None
    name, _, rest = t.partition("(")
    args = [a.strip() for a in rest[:-1].split(",") if a.strip()]
    return (name.strip().lower(), args) if name.strip() and args else None


def _words(text: str) -> set[str]:
    """Content words, for matching a plan's name against a survey's name."""
    drop = {"the", "a", "an", "of", "on", "in", "with", "and"}
    return {w for w in "".join(c if c.isalnum() else " " for c in text.lower()).split()
            if w and w not in drop}


class Agent:
    def __init__(
        self,
        robot,  # SafeRobot
        orchestrator,  # Orchestrator (Gemini or Scripted)
        cfg: HeronConfig,
        name: Optional[str] = None,
        memory: Optional[MemoryStore] = None,
        on_step_end: Optional[Callable[[Step, TaskProgram], None]] = None,
        log: Optional[EpisodeLogger] = None,
    ) -> None:
        self.cfg = cfg
        self.robot = robot
        self.orchestrator = orchestrator
        self.registry = load_all()
        self.log = log if log is not None else EpisodeLogger(cfg.episodes_dir, name)
        self.belief = BeliefStore()
        self.memory = memory if memory is not None else MemoryStore(cfg.memory_dir)
        self.skills = SkillLibrary(cfg.skills_dir)
        from .patches import PatchStore  # noqa: PLC0415
        self.patch_store = PatchStore(getattr(cfg, "patches_path", None))
        # Positions a look produced, waiting to be handed to the actions that
        # need them. Cleared per episode; retired per object as things move.
        self._values: dict[str, Any] = {}
        self.segmenter = SamSegmenter(cfg.sam_url, log=self.log)
        self.ctx = SkillContext(robot=robot, belief=self.belief, grounder=orchestrator,
                                cfg=cfg, log=self.log, memory=self.memory,
                                segmenter=self.segmenter, patches=self.patch_store)
        self.on_step_end = on_step_end
        self._steps_executed = 0
        self._edits_applied = 0
        self._repairs_per_step: dict[str, int] = {}
        self._seen_failures: dict[str, int] = {}
        self._replanned_sigs: set[str] = set()
        self._t0 = 0.0
        # Live-console handles; see run().
        self.program = None
        self.instruction = ""

    # -- public --------------------------------------------------------------
    def brief(self, program, frames) -> str:
        """What I see and what I intend, in the operator's own language.

        One minimal call over the frames the plan was made from; falls back
        to a template when the orchestrator cannot narrate (mock, offline).
        """
        steps = " -> ".join(
            f"{s.skill}({', '.join(str(v) for v in (s.args or {}).values())})"
            for s in program.steps)
        goal = ", ".join(str(p) for p in program.goal_predicates)
        # The planner writes a rationale per step; the briefing narrates THAT
        # chain of thought instead of improvising commentary over a finished
        # plan. A recalled program says so — its reasoning is inherited from
        # the episode that earned it, not fresh.
        why = "; ".join(f"{s.skill}: {s.rationale}" for s in program.steps
                        if getattr(s, "rationale", ""))
        provenance = ("这个计划是从记忆中复用的(这条指令之前成功过),没有重新推理。"
                      if getattr(self, "_plan_recalled", False) else
                      "这个计划是你刚针对当前画面推理出来的。")
        call = getattr(self.orchestrator, "_call", None)
        if callable(call):
            try:
                text = call(
                    "briefing",
                    "你是桌面机械臂 agent,刚接到任务并做完计划,现在向站在旁边的操作员口头汇报。"
                    "用口语化的中文说 2-4 句:(1) 你在桌面上看到了什么(只说与任务相关的);"
                    f"(2) 你打算怎么做以及为什么。{provenance}"
                    f"不要列表,不要客套。任务: {program.goal}。"
                    f"计划步骤: {steps}。目标条件: {goal}。"
                    + (f"你规划每一步时写下的理由: {why}。汇报要忠实于这些理由,不要另编。"
                       if why else "")
                    + ("如果计划是复用的,开头提一句。" if getattr(self, "_plan_recalled", False) else ""),
                    list(frames), "minimal", None)
                if isinstance(text, str) and text.strip():
                    return text.strip()
            except Exception:
                pass
        return f"任务: {program.goal}。计划: {steps}。目标: {goal}。"

    def _survey_call(self, instruction: str, frames) -> None:
        """The one look, taken FIRST — before the goal check, before planning.

        tiptop's pipeline reads the image and the instruction together and comes
        back with every object, a name for each that collides with no other, and
        the goal in predicates over those names. Everything after it is
        geometry. Ours asked the same model three times for three parts of that
        answer, in an order where none of them could use the others: a VQA that
        judged the goal before anything had been located, a planner that named
        entities the detector then had to find one at a time, and only then a
        survey.

        Taken first, the survey is what the goal check and the planner read.
        """
        survey = getattr(self.orchestrator, "survey", None)
        if not callable(survey) or not frames:
            return
        try:
            self._survey_raw = survey(frames[0], instruction)
        except Exception as e:
            self.log.event("survey_failed", error=f"{type(e).__name__}: {e}"[:150])
            self._survey_raw = None

    def _survey(self, instruction: str, frames, entities=None, goal=None) -> None:
        """One look, before anything else, that names everything it can see.

        THERE IS NO REASON TO OBSERVE MORE THAN ONCE. Measured on the rig
        2026-08-06, a single episode made 54 model calls; two of them did the
        work and the rest were the same question asked again after an empty
        answer, once per perceive step, once per camera, once per phrasing.
        A survey answers all of it from one image: every object, an id that
        collides with no other id, and the goal in predicates over those ids.

        The ids matter as much as the count. Two plates on this table both
        answer to "the white plate", and grounding spent calls all day landing
        on the big one out of reach, refusing the jump, and looking again. The
        model looking at the picture says large_white_plate and
        small_white_plate, and the ambiguity never reaches the belief store.

        What is seeded here is the PIXEL, not the position: the deprojection,
        the mask, the plausibility band and the jump guard all still run. This
        removes the question "where is the white plate", which the detector was
        being asked six times an episode and which had no right answer.
        """
        got = getattr(self, "_survey_raw", None)
        if not got or not frames:
            return
        objects = got.get("objects") or []
        if not objects:
            return
        # THE SURVEY'S NAMES ARE NOT THE PLAN'S NAMES, AND THAT IS THE POINT.
        # The planner asks for "white_plate"; the survey, looking at a table
        # with two of them, answers large_white_plate and small_white_plate.
        # Matching by id alone silently dropped exactly the seed that mattered
        # — the plate — and grounding went back to landing on the big one out
        # of reach. So each plan entity takes the survey object whose words it
        # shares most, one object to one entity, and inherits the survey's own
        # description with it.
        # THE SURVEY ALREADY CHOSE, AND SAID SO IN ITS GOAL. Asked to put a
        # block on "the white plate" with two on the table, it answered
        # on(green_cube, small_white_plate) and wrote in its note which one it
        # picked and why. Matching names by their words cannot see that:
        # "white_plate" shares every word with BOTH large_white_plate and
        # small_white_plate, so the first in the list won and the arm spent the
        # episode reaching for a plate 20 cm outside its workspace.
        #
        # Both goals are predicates over the same task in the same vocabulary,
        # so their arguments correspond position by position. That binding is
        # the model's own decision about which plate, and it outranks any
        # similarity score we could invent.
        # WHICH PLATE THE ARM CAN REACH IS NOT SOMETHING THE MODEL CAN SEE.
        # It chose by what it could: empty, central, near the block. On this
        # table the other white plate sits 25 cm outside the right arm's
        # workspace, and place() got as far as "nothing above (0.734, 0.176) is
        # reachable" before a repair dropped the target altogether and set the
        # cube down on a bare patch of table — reported as success.
        #
        # So we apply the one constraint we hold and it does not: geometry.
        # Nothing here overrules what the model MEANT; it only declines to aim
        # at a place the arm demonstrably cannot go.
        reach = {}
        for o in objects:
            pos = frames[0].deproject(*o["point"])
            ok = False
            if pos is not None:
                # ANY active arm counts. active_arms[0] was fine while there
                # was one arm and wrong the moment there were two: with
                # [left, right] this asked only the LEFT arm, every block on
                # the right half was declared out of reach, and the survey's
                # whole seeding was discarded on the first bimanual episode.
                try:
                    arms = list(self.cfg.active_arms or ["right"])
                    ok = any(bool(self.robot.reachable(a, pos)) for a in arms)
                except Exception:
                    sx, sy = self.cfg.safety.workspace_x, self.cfg.safety.workspace_y
                    ok = sx[0] <= pos[0] <= sx[1] and sy[0] <= pos[1] <= sy[1]
            reach[o["id"]] = ok
        unreachable = sorted(k for k, v in reach.items() if not v)
        if unreachable:
            self.log.event("survey_out_of_reach", ids=unreachable)

        bound = {}
        survey_goal = [_predicate(g) for g in (got.get("goal") or [])]
        survey_goal = [g for g in survey_goal if g]
        for name, args in ((p.name, list(p.args)) for p in (goal or [])):
            for s_name, s_args in survey_goal:
                if s_name == name and len(s_args) == len(args):
                    for mine, theirs in zip(args, s_args):
                        if not reach.get(theirs, True):
                            alt = _reachable_twin(theirs, objects, reach)
                            if alt:
                                self.log.event("survey_target_out_of_reach",
                                               chose=theirs, using=alt, entity=mine)
                                theirs = alt
                        bound.setdefault(mine, theirs)
                    break
        seeded = {}
        taken = set()
        by_id = {o["id"]: o for o in objects}
        for decl in (entities or []):
            claim = by_id.get(bound.get(decl.id, ""))
            if claim is not None and claim["id"] not in taken:
                taken.add(claim["id"])
                seeded[decl.id] = {"camera": frames[0].camera, "point": claim["point"],
                                   "box": claim.get("box"), "conf": claim.get("conf", 0.8),
                                   "description": claim.get("description") or claim["id"],
                                   "survey_id": claim["id"]}
                continue
            want = _words(f"{decl.id} {decl.description}")
            best, score = None, 0.0
            for o in objects:
                if o["id"] in taken:
                    continue
                have = _words(f"{o['id']} {o.get('description') or ''}")
                overlap = len(want & have) / max(1, len(want))
                if overlap > score:
                    best, score = o, overlap
            if best is None or score < 0.5:
                continue
            taken.add(best["id"])
            seeded[decl.id] = {"camera": frames[0].camera, "point": best["point"],
                               "box": best.get("box"), "conf": best.get("conf", 0.8),
                               "description": best.get("description") or best["id"],
                               "survey_id": best["id"]}
        for o in objects:                     # anything the plan never named
            if o["id"] not in taken and o["id"] not in seeded:
                seeded[o["id"]] = {"camera": frames[0].camera, "point": o["point"],
                                   "box": o.get("box"), "conf": o.get("conf", 0.8),
                                   "description": o.get("description") or o["id"],
                                   "survey_id": o["id"]}
        self.ctx.scratch["survey"] = seeded
        self.log.event("survey_seeded",
                       matched={k: v["survey_id"] for k, v in seeded.items()
                                if v.get("survey_id") != k},
                       ids=sorted(seeded),
                       camera=frames[0].camera, note=str(got.get("note") or "")[:160])

    def run(self, instruction: str, plan_hook=None) -> dict:
        self._t0 = time.time()
        # The wall-clock budget, handed to the layer that spends it. Between-step
        # checks stay (they produce the tidy "budget exhausted" abort); this is
        # what stops a verification chain or a retry backoff from spending
        # minutes PAST the deadline one model call at a time. A refused call
        # surfaces as UNKNOWN in verification and as a failed repair elsewhere,
        # both of which the loop already knows how to end an episode on.
        self.orchestrator.deadline = self._t0 + self.cfg.budgets.wall_clock_s
        self.log.event("episode_start", instruction=instruction, mode=self.cfg.mode)
        frames = self._fixed_frames()
        for f in frames:
            self.log.frame(f, tag="initial")
        asked = instruction
        # WHAT A WATCHER NEEDS TO SEE. Two attributes, kept current, so a live
        # console (heron/console.py) can read the running plan and the task
        # straight off the agent instead of reconstructing them from the
        # journal — a reconstruction can disagree with the thing it is meant to
        # be a window into, and disagree exactly when something is going wrong.
        self.instruction = instruction
        self._survey_call(instruction, frames)
        while True:
            program = self._recall_or_plan(asked, frames)
            self._strip_pointless_staging(program)
            self.program = program
            self._expand_learned(program)
            self._normalize_arms(program)
            self._loop_targets_the_unsatisfied(program)
            self._fill_default_posts(program)
            self._reject_unknown_skills(program)
            self._reject_misgrounded_goal(program, asked)
            self._reject_vacuous_goal(program)
            if plan_hook is None:
                break
            # Say what I see and what I intend; the operator's reply either
            # releases the plan or becomes a correction the next plan is made
            # under. Dialogue before motion — a wrong plan is cheapest here.
            correction = plan_hook(self, program, frames)
            if not correction:
                break
            self.log.event("operator_correction", text=correction)
            asked = f"{instruction} (operator correction: {correction})"
        self._survey(instruction, frames, program.entities, program.goal_predicates)
        self.belief.declare_entities(program.entities)
        # Grounding gets to know what the plan is FOR. With lookalikes, "a red
        # block" must bind to an instance the task still needs to act on — the
        # rig bound it to the red block already sitting on the plate (its
        # `near(white_plate)` disambiguator selects exactly the wrong one once
        # the goal is half-done) and the arm re-placed the placed block while
        # the one on the table was never touched.
        self.ctx.goal_predicates = [(p.name, list(p.args), bool(p.negated))
                                    for p in program.goal_predicates]
        # The raw instruction rides along for the skills: an operator naming
        # an arm outranks every arm-selection heuristic, and the planner does
        # not reliably copy "with the left arm" into the plan's arm fields.
        self.ctx.instruction = self.instruction
        # Phase-level verdict checks: skills call this hook at every phase
        # mark, so 判成功/判失败 lands within ~one motion instead of waiting
        # out a whole repair loop (measured: 51 s from click to stop).
        self.ctx.scratch["interrupt_check"] = self._check_human_verdict
        # What this plan will cost to see, before anything moves. The number is
        # a property of the plan rather than of what each skill decides in
        # private — which is the whole point of putting perception in the plan.
        # `unresolved` is the same reading used the other way: an action on
        # something no look produced is malformed, and saying so here is cheaper
        # than discovering it three motions in.
        self.log.event("plan_perception", looks=dataflow.program_cost(program),
                       unresolved=[f"{s}:{e}" for s, e in dataflow.unresolved(program)])
        self.log.event("plan", program=program)
        self.log.snapshot("program_initial", program)

        goal_repairs = 0
        crash = None
        try:
            while True:
                self._check_human_verdict()
                self._execute_until_blocked(program)
                if program.aborted:
                    status, reason = "aborted", program.abort_reason
                    break
                unmet = self._goal_check(program)
                if not unmet:
                    status, reason = "succeeded", "all goal predicates verified"
                    break
                if self._budget_exceeded(program) is not None:
                    status, reason = "failed", f"goal predicates unmet: {[str(p) for p in unmet]}"
                    break
                goal_repairs += 1
                report = FailureReport(
                    step_id="goal", skill="goal", failure_class=FailureClass.PROGRAM,
                    summary=f"program completed but goal predicates unmet: {[str(p) for p in unmet]}",
                    failed_predicates=unmet,
                )
                self.log.event("failure", step_id="goal", failure_class="program", summary=report.summary)
                if self._giving_up_on(program, report):
                    status, reason = "aborted", program.abort_reason
                    break
                if not self._orchestrator_repair(program, None, report):
                    status, reason = "aborted", program.abort_reason or "no viable goal repair"
                    break
        except _HumanVerdict as hv:
            if hv.verdict == "__budget__":
                self.log.event("budget_abort", wall_s=round(time.time() - self._t0, 1))
                status, reason = "aborted", "wall-clock budget exhausted"
            else:
                self.log.event("human_verdict", verdict=hv.verdict)
                status, reason = hv.verdict, "operator verdict from the terminal"
        except Exception as e:
            # An escaped exception used to be recorded as reason="exception",
            # which threw away the one thing needed to fix it. Whatever gets
            # here is a bug in Heron, not a robot failure, so say what it was.
            import traceback
            crash = f"{type(e).__name__}: {e}"
            self.log.event("agent_crashed", error=crash,
                           traceback=traceback.format_exc()[-1500:])
            status, reason = "crashed", crash
        finally:
            retired = 0
            if getattr(self, "patch_store", None) is not None:
                import os
                retired = self.patch_store.end_episode(os.path.basename(str(self.log.dir)))
                if retired:
                    self.log.event("patches_retired", count=retired,
                                   note="unvalidated episode-scoped corrections do not outlive the episode")
            summary = {
                "instruction": instruction,
                "status": locals().get("status", "crashed"),
                "reason": locals().get("reason", crash or "exception"),
                "steps_executed": self._steps_executed,
                "edits_applied": self._edits_applied,
                "wall_s": round(time.time() - self._t0, 1),
            }
            done = locals().get("status", "crashed")
            # Only a plan that WORKED is worth keeping, and every use is scored
            # so one that stops working is dropped rather than argued with.
            try:
                if done == "succeeded":
                    self.memory.remember_program(instruction, program.model_dump(mode="json"))
                self.memory.record_program_outcome(instruction, done == "succeeded")
            except Exception as e:
                self.log.event("program_memory_failed", error=f"{type(e).__name__}: {e}")
            self._consolidate_skills(program, locals().get("status", "crashed"))
            try:
                self._learn_from_episode(program, locals().get("status", "crashed"))
            except Exception as e:
                # Learning is upkeep. An episode that reached the end has already
                # produced its result, and losing that result because the library
                # could not write a file would be a poor trade.
                self.log.event("skill_learning_failed", error=f"{type(e).__name__}: {e}")
            self.log.snapshot("program_final", program)
            self.log.close(**summary)
            try:
                self.memory.distill_episode(read_journal(self.log.dir), str(self.log.dir))
            except Exception:
                pass
        return summary

    def _hints_with_rules(self) -> str:
        """Planner hints, plus every validated program-level rule in scope.

        This is where a PROGRAM patch acts: not by editing an existing plan
        but by standing in front of the compiler every time one is made. A
        rule like "wide vessels are grasped by the rim, so plan a regrasp"
        earned on one task applies to the next because its scope says so —
        not because a string happened to match.
        """
        hints = self.memory.planner_hints()
        store = getattr(self, "patch_store", None)
        if store is None:
            return hints
        rules = store.rules({"task": getattr(self, "instruction", "") or ""})
        if not rules:
            return hints
        lines = "\n".join(f"- {p.delta.get('rule', '')} [{p.provenance}]"
                           for p in rules if p.delta.get("rule"))
        return hints + ("\n\nVALIDATED RULES from prior corrections — honour them:\n"
                        + lines if lines else "")

    def _loop_targets_the_unsatisfied(self, program: TaskProgram) -> None:
        """A repeat_until body must operate on an instance that still needs the
        work — enforced structurally, not requested politely.

        Measured on the first live run of "move all the red blocks into the
        grey tray": pass one moved a block into the tray, pass two grounded
        "the red block" existentially, bound the one ALREADY IN THE TRAY,
        lifted it back out and undid its own work. The planner had been told
        (in prose) to declare the exclusion; it did not. But the exclusion is
        not a style preference — it is what repeat_until all_in(X, C) MEANS:
        each pass serves an X for which in(X, C) does not yet hold. Semantics
        implied by the program's own structure are the executor's to enforce.
        """
        ru = program.repeat_until
        if ru is None or ru.name != "all_in" or len(ru.args) < 2:
            return
        x, container = ru.args[0], ru.args[1]
        for e in program.entities:
            if e.id == x and e.relation is None:
                e.relation = SpatialRelation(kind="inside", of=[container],
                                             negated=True)
                self.log.event("loop_relation_bound", entity=x,
                               container=container,
                               note="the body grounds an instance still outside")

    def _strip_pointless_staging(self, program) -> None:
        """Drop a place-onto-an-unrelated-OBJECT + re-pick pair from the plan.

        The planner sometimes stages the cargo on whatever else the scene
        offers: "stack red on blue" planned place-on-PLATE first — the
        program memory is full of tonight's plate tasks and the model
        imitated the shape. A staging hop is only legitimate as a coordinate
        drop for a cross-arm relay (target=None); parking the cargo ON an
        unrelated object and re-picking it buys nothing and doubles the
        failure surface.
        """
        try:
            steps = list(getattr(program, "steps", []) or [])
            preds = list(getattr(program, "goal_predicates", []) or [])
            for i, st in enumerate(steps):
                if st.skill != "place":
                    continue
                e = (st.args or {}).get("entity")
                tgt = (st.args or {}).get("target")
                if not e or not tgt:
                    continue
                if any(e in p.args and tgt in p.args for p in preds):
                    continue
                for j in range(i + 1, len(steps)):
                    s2 = steps[j]
                    if s2.skill in ("pick", "place") and                             (s2.args or {}).get("entity") == e:
                        if s2.skill == "pick":
                            drop = {st.id, s2.id}
                            program.steps = [x for x in steps
                                             if x.id not in drop]
                            self.log.event("plan_staging_stripped", entity=e,
                                           via=tgt, dropped=sorted(drop))
                            return
                        break
        except Exception as e:
            self.log.event("plan_staging_strip_failed",
                           error=f"{type(e).__name__}: {e}"[:100])


    def _recall_or_plan(self, instruction: str, frames: list[Frame]) -> TaskProgram:
        """The program that worked for this instruction before, or a fresh one.

        A program names entities and orders steps; it holds no coordinates,
        because positions are ground afresh every episode. So the same
        instruction in a rearranged scene wants the same program, and planning
        it again buys nothing — one call and 14 seconds, the largest single item
        on a clean pick-and-place's 39-second model budget.

        The recalled program is RESET, not resumed: every step back to pending,
        attempts to zero, and the edits of the episode that earned it dropped.
        Resuming would carry a finished run's state into a scene that has been
        rearranged since, which is a different and much worse bug than the call
        it saves.
        """
        remembered = self.memory.recall_program(instruction)
        if remembered:
            program = None
            try:
                program = TaskProgram(**remembered)
            except Exception as e:
                self.log.event("program_recall_failed", error=f"{type(e).__name__}: {e}")
            if program is not None and _self_defeating(program):
                # A remembered plan that undoes its own goal is poison, not a
                # saving: a "remove from the plate" program carrying
                # place(target=plate) — inherited from a repair inside an
                # episode that then counted as a success — put the block BACK
                # twice in tonight's loop. Audit at recall; plan fresh.
                self.log.event("program_recall_rejected",
                               instruction=instruction,
                               why="a step re-achieves a negated goal predicate")
            elif program is not None:
                # Process-ness is a fact about the CURRENT scene (is the
                # goal already true right now?), not the remembered one —
                # same doctrine as coordinates. The pre-flight below re-sets
                # it when today's scene warrants.
                program.process_goal = False
                for step in program.steps:
                    step.status = StepStatus.PENDING
                    step.attempts = 0
                    # "A program holds no coordinates" is enforced, not
                    # assumed: a remembered removal kept replaying the
                    # planner's place(0.3, -0.25) from the episode that
                    # earned it, in every scene since. Positions are the
                    # current scene's business — dropping them here hands
                    # the step to the clear-spot sampler.
                    stale = [k for k in ("x", "y", "dx", "dy")
                             if k in (step.args or {})]
                    for k in stale:
                        del step.args[k]
                    if stale:
                        self.log.event("recalled_coords_stripped",
                                       step_id=step.id, skill=step.skill,
                                       dropped=stale)
                program.edits = []
                program.aborted = False
                program.abort_reason = ""
                self._plan_recalled = True
                self.log.event("program_recalled", instruction=instruction,
                               steps=[s.skill for s in program.steps],
                               saved_call="plan")
                return program
        self._plan_recalled = False
        return self.orchestrator.plan(instruction, frames, self._catalog(),
                                      self._hints_with_rules())

    # -- learned skills ------------------------------------------------------
    def _catalog(self) -> str:
        """Primitives plus whatever the agent has learned and kept."""
        doc = self.registry.catalog(exclude=self.cfg.disable_skills)
        learned = self.skills.catalog()
        if learned:
            doc += ("\n\nLearned skills (composed from the primitives above, refined over "
                    "past episodes — prefer them when they fit):\n" + learned)
        if self.cfg.vla_primary and "vla" not in (self.cfg.disable_skills or []):
            doc += (
                "\n\nMANIPULATION POLICY FOR THIS RIG: a learned policy does the physical "
                "work. For every step that MOVES anything — picking, placing, opening, "
                "closing, pushing, inserting — emit a `vla` step with a short imperative "
                "instruction naming the objects (one verb, concrete nouns, e.g. 'pick up "
                "the black bowl and place it on the plate'), max_seconds 40, and explicit "
                "postconditions so the outcome is verified rather than assumed. Use "
                "`perceive`/`inspect` for looking, exactly as before. The scripted "
                "pick/place primitives remain available as the FALLBACK when a vla step "
                "has failed its postconditions twice — not as the first choice.")
        elif self.cfg.vla_verbs and "vla" not in (self.cfg.disable_skills or []):
            verbs = ", ".join(self.cfg.vla_verbs)
            doc += (
                f"\n\nROUTING RULE FOR THIS RIG: for any step whose action is one of "
                f"[{verbs}] — and ONLY those — emit a `vla` step with a short imperative "
                "instruction naming the objects (one verb, concrete nouns, e.g. 'open "
                "the middle drawer of the cabinet'), max_seconds 40, and explicit "
                "postconditions so the outcome is verified rather than assumed. Every "
                "other step — picking, placing, perceiving, inspecting — uses the "
                "scripted skills exactly as documented above; do not route them to "
                "`vla`. When naming objects in a vla instruction, describe what you "
                "actually observed in the scene, not the instruction's own wording.")
        return doc

    def _expand_learned(self, program: TaskProgram) -> None:
        """Replace learned-skill steps with the primitive steps they stand for.

        Expanding here, rather than executing a skill as one opaque call, keeps
        every downstream mechanism intact: each primitive step is still verified,
        and a failure inside a skill is still diagnosed and repaired in place.
        """
        expanded: list[Step] = []
        used: list[str] = []
        for step in program.steps:
            if self.skills.get(step.skill) is None:
                expanded.append(step)
                continue
            try:
                sub = self.skills.expand(step.skill, step.args, step.id)
            except Exception as e:
                self.log.event("skill_expansion_failed", skill=step.skill, error=str(e))
                expanded.append(step)  # let the executor report it as an unknown skill
                continue
            # The invocation's own conditions belong to the fragment as a whole.
            if step.pre:
                sub[0].pre = list(step.pre) + list(sub[0].pre)
            if step.post:
                sub[-1].post = list(sub[-1].post) + list(step.post)
            expanded.extend(sub)
            used.append(step.skill)
            self.log.event("skill_expanded", skill=step.skill, step_id=step.id,
                           into=[s.id for s in sub])
        program.steps = expanded

    def _reject_misgrounded_goal(self, program: TaskProgram, instruction: str) -> None:
        """Refuse a plan whose graded entities are not the ones the task named.

        THE GOAL PREDICATES ARE WRITTEN BY THE SAME MODEL THAT NAMES THE
        ENTITIES, so "all goal predicates verified" is not by itself evidence
        that the task was done — only that the plan was self-consistent. Every
        other guard here checks the plan against the world. This one checks it
        against the sentence, which is the one thing the world cannot settle.

        Measured, and caught by eye rather than by any check I had: asked to
        "move the blue block onto the white plate" the planner declared the
        target `grey_plate`, "the grey square plate" — the tray, which is a grey
        square, sitting next to the white plate it was asked for. It then put
        the block on the tray, verified `on(blue_block, grey_plate)`, and was
        recorded as SUCCEEDED. Nothing downstream could catch it: the predicate
        was true, the block really was on the thing the plan pointed at.

        Then it got worse, because that episode was cached. The next task,
        "move the RED block onto the white plate", recalled it as a sister,
        substituted red for blue, inherited the tray as its target, succeeded on
        its own terms again, and was cached again with a fresh success. A
        mis-grounded plan does not merely fail once; it gains confidence.

        The test is deliberately narrow. Only colours, only the entities the
        goal predicates actually name, and only the direction that matters: a
        colour the INSTRUCTION used that appears nowhere among them. The
        converse — a colour in a description the instruction never used — is
        ordinary and legal, since a plan may well name a green block that is in
        the way. Narrow enough that when it fires it is right.
        """
        missing = misgrounded_colours(instruction, program)
        if not missing:
            return
        # FIRST, TRY TO REPAIR RATHER THAN REFUSE. Measured on LIBERO spatial:
        # the planner honestly described a glazed black bowl as "the metal bowl
        # with a gold rim" — same object, no colour word — and this guard killed
        # 4/4 episodes at zero steps. The description is the DETECTOR'S QUERY,
        # and the instruction's own phrase is the one query the task guarantees
        # to identify the object. So when the instruction's colour is missing,
        # put the instruction's own words back: grounding then searches the
        # scene for "the akita black bowl" and its existing candidate scoring
        # picks the best match. The grey-tray case is fixed by the same move —
        # the description becomes "the white plate" and grounding finds the
        # white plate instead of confirming the tray.
        repaired = []
        for colour in list(missing):
            m = re.search(r"((?:\bthe\s+)?(?:[\w-]+\s+){0,2}" + colour +
                          r"(?:\s+[\w-]+){0,2})", instruction, re.IGNORECASE)
            if not m:
                continue
            phrase = m.group(1).strip()
            if not phrase.lower().startswith("the "):
                phrase = "the " + phrase
            graded_ids = {a for spec in program.goal_predicates for a in spec.args}
            # The phrase's head noun says WHICH entity it names: "the white
            # plate" repairs the plate-shaped entity, not whichever graded
            # entity happens to lack the colour word (the first version chose
            # blue_block and painted it white).
            head = phrase.split()[-1].lower().rstrip("s")
            target = next((e for e in program.entities
                           if e.id in graded_ids
                           and colour not in _colours(e.description)
                           and colour not in _colours(e.id)
                           and (head in e.id.lower() or head in e.description.lower())),
                          None)
            if target is None:
                continue
            self.log.event("misgrounded_goal_repaired", entity=target.id,
                           was=target.description, now=phrase, colour=colour)
            target.description = phrase
            repaired.append(colour)
        missing = misgrounded_colours(instruction, program)
        if not missing:
            try:
                self.memory.retire_program(instruction, "misgrounded (repaired)")
            except Exception:
                pass
            return
        graded = {a for spec in program.goal_predicates for a in spec.args}
        described = {e.id: e.description for e in program.entities}
        named = sorted(f"{n}={described.get(n, '?')!r}" for n in graded if n in described)
        reason = (
            f"the plan is graded on {[str(p) for p in program.goal_predicates]}, whose "
            f"entities are {named} — nothing there is {' or '.join(missing)}, which the "
            f"instruction asked for. Verifying this goal would confirm the plan, not the "
            f"task: a block placed on a grey tray satisfies on(block, grey_tray) however "
            f"plainly the instruction said white plate")
        self.log.event("misgrounded_goal", instruction=instruction, missing=missing,
                       predicates=[str(p) for p in program.goal_predicates], entities=named)
        # A cached plan that fails this fails it every time it is recalled, and
        # it was cached BECAUSE it reported success. Drop it here rather than
        # paying a recall to abort on it again tomorrow.
        def fails(served: dict) -> bool:
            try:
                return bool(misgrounded_colours(instruction, TaskProgram(**served)))
            except Exception:
                return False

        try:
            dropped = self.memory.retire_program(instruction, "misgrounded", audit=fails)
            if dropped:
                self.log.event("program_retired", keys=dropped, why="misgrounded")
        except Exception as e:
            self.log.event("program_memory_failed", error=f"{type(e).__name__}: {e}")
        # Same one-shot medicine as the vacuous-goal case: name the disease to
        # the planner and ask again. Degenerate arm-state goals fail the colour
        # audit too (gripper_empty mentions no object), and that path aborted
        # BEFORE the vacuous replan could fire.
        if not getattr(self, "_vacuous_replanned", False):
            self._vacuous_replanned = True
            feedback = (f"{instruction} (Your previous plan was graded on "
                        f"{[str(p) for p in program.goal_predicates]}, which never "
                        f"mentions {' or '.join(missing)}. State the goal as a "
                        "location predicate about the named object: declare the "
                        "object and a region/support entity, and grade on "
                        "near(<object>, <region>) or on(<object>, <support>). "
                        "gripper_empty/holding are not task goals.)")
            try:
                fresh = self.orchestrator.plan(feedback, self._fixed_frames(),
                                               self._catalog(),
                                               self._hints_with_rules())
                if fresh.goal_predicates and not misgrounded_colours(instruction, fresh) \
                        and any(p.name in ("on", "in", "near")
                                for p in fresh.goal_predicates):
                    program.goal_predicates = fresh.goal_predicates
                    program.steps = fresh.steps
                    program.edits = []
                    self._repairs_per_step.clear()
                    self._values.clear()
                    for e in fresh.entities:
                        if e.id not in {x.id for x in program.entities}:
                            program.entities.append(e)
                    self.belief.declare_entities(program.entities)
                    self.log.event("misgrounded_goal_replanned",
                                   goal=[str(p) for p in fresh.goal_predicates])
                    return
            except Exception as e:
                self.log.event("vacuous_replan_failed",
                               error=f"{type(e).__name__}: {e}"[:150])
        program.apply_edit(ProgramEdit(type=EditType.ABORT, rationale=reason, source="rules"))

    def _reject_vacuous_goal(self, program: TaskProgram) -> None:
        """Refuse a goal that already holds before anything has been done.

        When a task cannot be expressed in the predicate vocabulary, the planner
        does not say so — it reaches for whatever it CAN express. Asked to open a
        drawer, it emitted `gripper_empty(arm)`, which was true at t=0, verified
        it correctly, and declared victory in sixteen seconds without moving.
        A truthful abort is worth far more than a success that means nothing.

        The same applies, more sharply, to a goal that says NOTHING. `_goal_check`
        walks the goal predicates and reports what is unmet; over an empty list
        it reports nothing unmet, which the loop reads as "all goal predicates
        verified". Measured: a plan for "move the red block into the grey tray"
        came back as two picks, no place, no postcondition on any step and no
        goal predicates at all — and the episode was recorded as SUCCEEDED with
        the block 206 mm from the tray. Then the skill library learned from it.

        Rare (1 episode in 62) and total when it happens: a program that promises
        nothing is graded as having delivered everything.
        """
        if not program.goal_predicates:
            reason = ("the plan declares no goal predicates, so there is nothing to "
                      "verify and no way to tell success from having done nothing — "
                      "every task must state what it means to have finished")
            self.log.event("ungradeable_goal", goal=program.goal,
                           steps=[s.skill for s in program.steps])
            program.apply_edit(ProgramEdit(type=EditType.ABORT, rationale=reason,
                                           source="rules"))
            return
        already = []
        for spec in program.goal_predicates:
            sat, _ = check(self.ctx, spec)
            if sat is Value.TRUE:
                already.append(spec)
        if len(already) == len(program.goal_predicates):
            # A cycle task ("put it on the plate, then put it back") has a
            # terminal state identical to the start — the goal snapshot cannot
            # grade the journey. When the plan carries REAL work with per-step
            # postconditions, that is not planner laziness: run it in PROCESS
            # mode, where success = every step executed and verified. The
            # abort below still guards the lazy case (no steps, or steps that
            # promise nothing).
            moving = [st for st in program.steps
                      if st.skill not in ("perceive", "inspect") and st.post]
            if len(moving) >= 2:
                program.process_goal = True
                self.log.event("process_goal", predicates=[str(p) for p in already],
                               steps=[st.skill for st in moving])
                return
            # ONE targeted replan before giving up: procedural hints in the
            # instruction ("push it across, then pick it up") reliably seduce
            # the planner into arm-state goals — twice in a row tonight. Tell
            # it exactly what was wrong and ask again.
            if not getattr(self, "_vacuous_replanned", False):
                self._vacuous_replanned = True
                feedback = (f"{program.goal} (Your previous plan's goal predicates "
                            f"{[str(p) for p in already]} already held before acting — "
                            "they do not express this task. State the DESTINATION as a "
                            "location predicate: declare a region entity (e.g. "
                            "left_half_of_table) and set the goal to "
                            "near(<object>, <region>) or on(<object>, <support>). "
                            "gripper_empty/holding are not task goals.)")
                try:
                    fresh = self.orchestrator.plan(feedback, self._fixed_frames(),
                                                   self._catalog(),
                                                   self._hints_with_rules())
                    if fresh.goal_predicates and any(
                            p.name in ("on", "in", "near") for p in fresh.goal_predicates):
                        program.goal_predicates = fresh.goal_predicates
                        program.steps = fresh.steps
                        program.edits = []
                        self._repairs_per_step.clear()
                        self._values.clear()
                        for e in fresh.entities:
                            if e.id not in {x.id for x in program.entities}:
                                program.entities.append(e)
                        self.belief.declare_entities(program.entities)
                        self.log.event("vacuous_goal_replanned",
                                       goal=[str(p) for p in fresh.goal_predicates])
                        return self._reject_vacuous_goal(program)
                except Exception as e:
                    self.log.event("vacuous_replan_failed",
                                   error=f"{type(e).__name__}: {e}"[:150])
            reason = (f"the goal {[str(p) for p in already]} already holds before acting — "
                      "the task as stated cannot be expressed in this predicate vocabulary "
                      f"({sorted({'holding', 'gripper_empty', 'visible', 'in', 'on', 'near', 'open'})})")
            self.log.event("vacuous_goal", predicates=[str(p) for p in already])
            program.apply_edit(ProgramEdit(type=EditType.ABORT, rationale=reason, source="rules"))

    def _reject_unknown_skills(self, program: TaskProgram) -> None:
        """Drop steps calling skills that do not exist.

        A hallucinated skill used to survive planning and surface much later as a
        precondition failure, after several repairs had chased the wrong problem.
        Catching it here turns a confusing three-repair spiral into one plain
        message the repair loop can act on.
        """
        known = set(self.registry.names()) - set(self.cfg.disable_skills)
        bad = [s for s in program.steps if s.skill not in known]
        if not bad:
            return
        for step in bad:
            self.log.event("unknown_skill_in_plan", step_id=step.id, skill=step.skill,
                           available=sorted(known))
            step.status = StepStatus.SKIPPED
            step.rationale = (f"{step.skill!r} is not an available skill "
                              f"(available: {sorted(known)})")

    def _record_place_outcome(self, step: Step, success: bool) -> None:
        """Turn a placement miss into a correction the next episode inherits.

        The residual is measured, not assumed: where the object actually settled
        minus where the skill aimed. That is the quantity a retry needs and the
        quantity the next episode needs, and they are the same number.
        """
        if step.skill != "place" or self.memory is None:
            return
        aim = self.ctx.last_place.get("aim")
        target = self.ctx.last_place.get("target")
        entity = self.ctx.last_place.get("entity")
        if not aim or not target or not entity:
            return
        description = self.belief.track(target).description
        residual = (0.0, 0.0)
        if not success:
            found, _ = ground_entity(self.ctx, entity)
            landed = self.belief.track(entity).xyz_base
            if not found or landed is None:
                return   # cannot measure it, so do not pretend to
            residual = (float(landed[0]) - float(aim[0]), float(landed[1]) - float(aim[1]))
            self.log.event("place_residual", entity=entity, target=target,
                           aim=[round(v, 4) for v in aim[:2]],
                           landed=[round(float(landed[0]), 4), round(float(landed[1]), 4)],
                           residual_mm=[round(residual[0] * 1000, 1), round(residual[1] * 1000, 1)])
        self.memory.record_place(description, residual[0], residual[1], success)
        self.ctx.last_place = {}

    def _consolidate_skills(self, program: TaskProgram, status: str) -> None:
        """Credit each learned skill for how its steps actually went, then upkeep.

        A skill is charged with a failure when its own steps needed repair, not
        merely when the episode ended badly — the rest of the plan is not its
        fault, and blaming it for that would retire useful skills.
        """
        outcomes: dict[str, bool] = {}
        for step in program.steps:
            if not step.origin_skill:
                continue
            clean = step.status is StepStatus.DONE and step.attempts <= 1
            outcomes[step.origin_skill] = outcomes.get(step.origin_skill, True) and clean
        for name, ok in outcomes.items():
            self.skills.record_outcome(name, ok, episode=str(self.log.dir))
            verdict = self.skills.consolidate(name)
            self.log.event("skill_outcome", skill=name, ok=ok, verdict=verdict,
                           episode_status=status)

    def _learn_from_episode(self, program: TaskProgram, status: str) -> None:
        """Keep what this episode had to work for, so the next one starts with it.

        Two halves, and only the second one existed before: a skill whose
        expansion needed repair gets those repairs folded back in (`revise`), and
        a fragment of ordinary steps that needed repair becomes a NEW skill.
        Without the second half the library could only ever refine skills that
        something else had already created, and nothing ever did — `learn_skill`
        was reachable from the tests and from nowhere in the running system.

        Only successful episodes teach. A fragment from a failed episode has no
        evidence that its technique was the right one; it has evidence that
        something after it was wrong, which is not the same claim.
        """
        if status != "succeeded":
            return

        # Half one: an existing skill that only worked after being repaired. The
        # repairs ARE its missing knowledge; the previous body is archived with
        # the score it earned, so a bad revision can be rolled back on evidence.
        for name in self.skills.names():
            steps = [s for s in program.steps if s.origin_skill == name]
            if not steps or not any(s.attempts > 1 for s in steps):
                continue
            if any(s.status is not StepStatus.DONE for s in steps):
                continue
            args = dict(steps[0].origin_args)
            try:
                self.revise_skill(name, steps, args,
                                  reason=f"repaired in {self.log.dir.name}")
            except Exception as e:
                self.log.event("skill_revision_failed", skill=name,
                               error=f"{type(e).__name__}: {e}")

        # Half two: promote a fragment that was fought for into a new skill.
        bodies = {n: tuple(b.skill for b in s.body)
                  for n in self.skills.names() if (s := self.skills.get(n))}
        for cand in distill.candidates(program):
            seen = distill.already_known(cand, bodies)
            if seen is not None:
                # The same technique, rediscovered. Credit the skill that already
                # says it rather than filing a near-duplicate beside it.
                self.skills.record_outcome(seen, True, episode=str(self.log.dir))
                self.log.event("skill_rediscovered", skill=seen,
                               steps=[s.skill for s in cand.steps])
                break
            name = distill.propose_name(cand, set(self.skills.names()))
            if name is None:
                continue
            description, when = distill.describe(cand)
            try:
                self.learn_skill(name, description, sorted(cand.params), cand.steps,
                                 distill.args_for(cand), when_to_use=when)
            except Exception as e:
                # A fragment that will not validate against the primitive
                # vocabulary is not a crisis; it is one candidate not kept.
                self.log.event("skill_not_learned", skill=name,
                               error=f"{type(e).__name__}: {e}")
                continue
            self.log.event("skill_learned_from_episode", skill=name,
                           steps=[s.skill for s in cand.steps],
                           params=sorted(cand.params), earned=cand.earned)
            break   # one skill per episode: the library grows on evidence, not volume

    def learn_skill(self, name: str, description: str, params: list[str],
                    steps: list[Step], args: dict, when_to_use: str = "") -> str:
        """Promote an executed fragment into a reusable, parameterized skill."""
        from .skills.library import LearnedSkill  # noqa: PLC0415

        skill = LearnedSkill(
            name=name, description=description, when_to_use=when_to_use, params=params,
            body=SkillLibrary.generalize(steps, args), provenance=[str(self.log.dir)],
        )
        self.skills.add(skill, self.registry.names())
        self.log.event("skill_learned", skill=name, steps=len(skill.body))
        return name

    def revise_skill(self, name: str, steps: list[Step], args: dict, reason: str) -> str:
        """Fold the repairs that made a skill work back into its definition."""
        body = SkillLibrary.generalize(steps, args)
        before = self.skills.get(name)
        self.skills.revise(name, body, reason, self.registry.names(), episode=str(self.log.dir))
        after = self.skills.get(name)
        self.log.event("skill_revised", skill=name, reason=reason,
                       version=after.version, was=before.version if before else None)
        return name

    # -- main execution ------------------------------------------------------
    def _execute_until_blocked(self, program: TaskProgram) -> None:
        while not self._program_finished(program):
            over = self._budget_exceeded(program)
            if over is not None:
                program.apply_edit(ProgramEdit(type=EditType.ABORT, rationale=over, source="rules"))
                self.log.event("budget_abort", reason=over)
                return
            step = program.next_pending()
            # origin_skill travels with the event: without it the journal cannot
            # say whether a step came from a learned skill or from the planner,
            # and "did the library help?" is not answerable after the fact.
            self.log.event("step_start", step_id=step.id, skill=step.skill, args=step.args,
                           attempt=step.attempts + 1, origin_skill=step.origin_skill)

            # 0. Guard: is this step needed on this pass at all?
            if step.when is not None:
                gval, gverdict = check(self.ctx, step.when)
                sat = step.when.satisfied_by(gval)
                self.log.event("guard", step_id=step.id, predicate=str(step.when),
                               value=gval.value, satisfied=sat.value,
                               note=str(gverdict.note)[:120])
                held = (self.belief.track(step.args["entity"]).held_by
                        if step.args.get("entity") in self.belief.entities else None)
                if sat is Value.FALSE and held and step.skill in ("place", "handover"):
                    # A GUARD MAY NOT STRAND CARGO. The planner guarded the
                    # put-down with on(yellow_block, plate) — true at plan
                    # time, false the moment the pick before it lifted the
                    # block. Skipping left the block in the gripper, and the
                    # next pick's first act is to OPEN THE JAWS at its hover:
                    # the block fell from there, back onto the plate it was
                    # being cleared from. Whatever a guard says, an entity in
                    # the gripper still has to go somewhere.
                    self.log.event("guard_overridden_cargo", step_id=step.id,
                                   entity=step.args.get("entity"), held_by=held)
                elif sat is Value.FALSE:
                    step.status = StepStatus.SKIPPED
                    self.log.event("step_skipped_by_guard", step_id=step.id,
                                   skill=step.skill)
                    continue

            # 1. Preconditions. THE GRIPPER OUTRANKS THE PLAN'S BOOKKEEPING:
            # the planner wrote holding(block, left), choose_arm rightly picked
            # the right arm, and the mismatch read as a world failure — repair
            # re-picked, the re-pick opened the jaws, and the cargo dropped
            # where it stood. If the belief says an arm holds the entity, the
            # predicate's arm argument is corrected to the arm that HAS it.
            for spec in step.pre:
                if spec.name == "holding" and len(spec.args) > 1 and not spec.negated:
                    held = self.belief.track(spec.args[0]).held_by \
                        if spec.args[0] in self.belief.entities else None
                    if held and spec.args[1] != held:
                        self.log.event("holding_arm_corrected", step_id=step.id,
                                       entity=spec.args[0], plan_said=spec.args[1],
                                       gripper_says=held)
                        spec.args[1] = held
            pre_specs = step.pre
            if self.cfg.verify_mode == "terminal":
                # Terminal mode: cameras wait until the goal. Proprio is free
                # and stays; the closed-on-air guard inside pick still fires.
                pre_specs = [sp for sp in step.pre if sp.name in self.PROPRIO]
            failed_pre = self._verify_specs(pre_specs, gather=True,
                                            unknown_blocks=False)
            if failed_pre:
                report = diagnose_precondition(self.ctx, step, failed_pre)
                self.log.event("failure", step_id=step.id, failure_class=report.failure_class.value,
                               summary=report.summary)
                self._handle_failure(program, step, report)
                continue

            self._check_human_verdict()
            # 2. Execute.
            step.status = StepStatus.RUNNING
            step.attempts += 1
            self._steps_executed += 1
            result = self.registry.execute(self.ctx, step.skill, step.args,
                                           supplied=self._value_for(step))
            # Whatever just ran may have changed the world — a grasp closes on
            # something without moving the tool, so the pose check alone is not
            # enough. Any shared look taken before it is now history.
            forget_the_look(self.ctx, reason=f"{step.skill} ran")
            self._retire_values(step)
            self.log.event("skill_exec", step_id=step.id, skill=step.skill, ok=result.ok,
                           info=result.info, error=result.error, proprio=result.proprio)
            if not result.ok:
                report = diagnose_exec_error(step, result)
                self.log.event("failure", step_id=step.id, failure_class=report.failure_class.value,
                               summary=report.summary)
                self._handle_failure(program, step, report)
                continue

            # 3. Postconditions.
            if step.skill in dataflow.PRODUCES:
                self._collect_values(step)
            post_specs = step.post
            if self.cfg.verify_mode == "terminal":
                post_specs = [sp for sp in step.post if sp.name in self.PROPRIO]
            if step.skill == "place" and any(sp.name in ("on", "in") for sp in post_specs):
                # The arm that just placed is parked square over its own work;
                # both cameras honestly reported a fresh stack invisible and
                # the repair loop dismantled a correct placement. Step aside
                # before judging.
                self._clear_view_for_goal()
            failed_post = self._verify_specs(post_specs, gather=True)
            if not failed_post:
                self._record_place_outcome(step, success=True)
                self._mark_done(program, step)
            else:
                self._record_place_outcome(step, success=False)
                report = diagnose_postcondition(self.ctx, step, failed_post, result)
                self.log.event("failure", step_id=step.id, failure_class=report.failure_class.value,
                               summary=report.summary, details=report.details)
                step.status = StepStatus.FAILED
                self._record_grasp_outcome(step, success=False)
                self._handle_failure(program, step, report)

            if self.on_step_end is not None:
                self.on_step_end(step, program)

    def _program_finished(self, program: TaskProgram) -> bool:
        """done(), plus the program-level while.

        When the body has completed and `repeat_until` is set, verify it: not
        satisfied and passes remain -> re-arm the body and keep going. The
        guard evaluation is a real verify — it looks — because the loop
        predicate is exactly the kind of claim ("all blocks are in the tray")
        that dead reckoning gets confidently wrong.
        """
        if not program.done():
            return False
        if program.aborted or program.repeat_until is None:
            return True
        lval, lverdict = check(self.ctx, program.repeat_until)
        sat = program.repeat_until.satisfied_by(lval)
        self.log.event("repeat_until", predicate=str(program.repeat_until),
                       value=lval.value, satisfied=sat.value,
                       repeats_done=program.repeats_done)
        # UNKNOWN does NOT satisfy the loop, and does not spin it either: an
        # unanswerable loop predicate re-arms once and then meets max_repeats.
        if sat is Value.TRUE:
            return True
        if program.repeats_done + 1 >= program.max_repeats:
            self.log.event("repeat_budget_exhausted",
                           repeats=program.repeats_done + 1)
            return True
        # THE LOOP ENTITY CHANGES IDENTITY AT THE TURN. Grounding's precedence
        # rule — "the relation decides the first binding and the prior defends
        # it afterwards" — is right within a pass and exactly wrong across
        # passes: pass one placed the block, so the freshest prior points INTO
        # the container, and pass two re-bound the same block and lifted it out
        # again. Measured on the first fixed run of "move all the red blocks":
        # the arm undid its own work twice. Releasing the binding here lets the
        # NOT-inside relation choose a fresh instance, which is what "each pass
        # serves an unsatisfied X" means.
        if program.repeat_until.name == "all_in" and program.repeat_until.args:
            x = program.repeat_until.args[0]
            self.belief.invalidate_entity(
                x, reason="loop pass complete; the body serves a different instance next")
            self.log.event("loop_binding_released", entity=x,
                           repeats_done=program.repeats_done + 1)
        program.rearm()
        return False

    def _mark_done(self, program: TaskProgram, step: Step) -> None:
        step.status = StepStatus.DONE
        entity = step.args.get("entity")
        desc = self.belief.track(entity).description if entity in self.belief.entities else ""
        self.log.event("step_done", step_id=step.id, skill=step.skill, entity_description=desc)
        self._record_grasp_outcome(step, success=True)
        if self.on_step_end is not None:
            self.on_step_end(step, program)

    def _record_grasp_outcome(self, step: Step, success: bool) -> None:
        if step.skill != "pick" or self.memory is None:
            return
        entity = step.args.get("entity")
        if entity and entity in self.belief.entities:
            desc = self.belief.track(entity).description
            hx, hy = self.memory.grasp_offset(desc)
            # pick() applied stored-prior + arg jitter; teach memory the total,
            # or the blend converges on the jitter alone and walks the prior
            # away from the offset that actually closed on the object.
            self.memory.record_grasp(desc, float(step.args.get("dx", 0.0)) + hx,
                                     float(step.args.get("dy", 0.0)) + hy, success)

    # -- verification with active information gathering ----------------------
    PROPRIO = ("holding", "gripper_empty")  # free to re-check — never trust cache

    def _value_for(self, step: Step) -> dict:
        """Hand an ACTION the position a look already produced.

        This is the dataflow rule applied to an ordinary typed program: a look
        produces positions, and the steps that need them are given them, rather
        than each skill arranging its own perception in private.

        The gain is not only the saved call. `_resolve_point` refuses a position
        older than a clock, and the clock does not know whether anything moved —
        measured twice in the twin's runs, an episode failed on "entity
        'grey_tray' location is stale (106s) — run perceive first" for a tray
        that had sat still the whole time. A value handed down the graph is valid
        because nothing has invalidated it, which is the question that was
        actually being asked.

        Only actions. Verification is deliberately left to look for itself: a
        check fed the number that decided the action would be confirming the
        action against its own premise, which is the shape of every false
        success measured today.
        """
        if step.skill not in dataflow.CONSUMES:
            return {}
        for key in dataflow.CONSUMES[step.skill]:
            entity = step.args.get(key)
            if not isinstance(entity, str):
                continue
            value = self._values.get(entity)
            if value is None:
                continue
            self.log.event("value_handed_on", step_id=step.id, skill=step.skill,
                           entity=entity, source=getattr(value, "source", "look"))
            return {"at": value}   # `at` names one position: the thing acted on
        return {}

    def _retire_values(self, step: Step) -> None:
        """A value stops being true when the step moves what it described."""
        for entity in dataflow.moves(step):
            if self._values.pop(entity, None) is not None:
                self.log.event("value_retired", step_id=step.id, entity=entity,
                               reason=f"{step.skill} moved it")

    def _collect_values(self, step: Step) -> None:
        """Record what a look produced, so later steps can be handed it."""
        from .skills.values import Located  # noqa: PLC0415

        for entity in dataflow.produces(step):
            ids = list(self.belief.entities) if entity == "*" else [entity]
            for eid in ids:
                tr = self.belief.track(eid)
                if tr.xyz_base is None or not tr.from_sighting:
                    continue
                self._values[eid] = Located(
                    xyz=np.asarray(tr.xyz_base, dtype=float), source="look",
                    conf=float(tr.confidence), span_m=tr.span_m)

    def _check_deadline(self) -> None:
        """The wall budget ends the EPISODE, not just the model calls.

        Refusing calls after the deadline left the loop churning through
        retries and motions for another 300-430 s (measured: 616 s and 733 s
        episodes on a 300 s budget). The same checkpoints that honor the
        operator's verdict honor the clock.
        """
        if time.time() > self._t0 + self.cfg.budgets.wall_clock_s:
            raise _HumanVerdict("__budget__")

    def _check_human_verdict(self) -> None:
        """The operator watched the arm do the thing (or drop it); their word
        ends the episode wherever it is — between steps, between goal
        predicates, mid-verification — instead of paying for a chain that
        answers a question already answered from two feet away."""
        hv = getattr(self, "human_verdict", None)
        if hv:
            raise _HumanVerdict(hv)
        self._check_deadline()

    def _verify_specs(self, specs: list[PredicateSpec], gather: bool,
                      unknown_blocks: bool = True) -> list[PredicateSpec]:
        self._check_human_verdict()
        # One batched look for every entity the specs mention, up front: each
        # visual predicate then reads a fresh track instead of commissioning
        # its own grounding round-trip. Measured on the rig, serial per-
        # predicate grounding put the FIRST verdict at 65.7 s and the second
        # at 100 s for the same two entities. The reuse gate inside perceive
        # keeps this free when the tracks are already fresh.
        visual_entities = sorted({a for spec in specs if spec.name not in self.PROPRIO
                                  for a in spec.args if a in self.belief.entities})
        if len(visual_entities) > 1:
            from .skills.sensing import perceive
            try:
                perceive(self.ctx, entities=visual_entities)
            except Exception:
                pass  # verification falls back to its own per-predicate looks
        failed: list[PredicateSpec] = []
        for spec in specs:
            if spec.name not in self.PROPRIO:
                cached = self.belief.holds(spec, max_age_s=self.cfg.budgets.verify_max_age_s)
                if cached is Value.TRUE:
                    continue
            sat, _ = check(self.ctx, spec)
            if sat is Value.UNKNOWN and gather:
                sat = self._gather_information(spec)
            if sat is Value.UNKNOWN and not unknown_blocks:
                # Precondition mode: an unanswerable check is not a failure.
                # The primitives carry their own physical guards (pick's
                # closed-on-air check, place's cargo check) — executing is how
                # an UNKNOWN gets answered. Blocking on it sent a trivial
                # stack into a repair loop that ended in a fabricated abort
                # (rig, 2026-08-17: gripper_empty(auto) unknown → abort
                # "red cube is missing" with the cube in plain view).
                self.log.event("precondition_unknown_proceeding", predicate=spec.key)
                continue
            if sat is not Value.TRUE:
                failed.append(spec)
        return failed

    def _gather_information(self, spec: PredicateSpec) -> Value:
        """UNKNOWN is an action item: take a closer look, then decide.

        For in/on the wrist camera flies over the container (rims occlude the
        overhead view); the check then re-runs. Still UNKNOWN counts as FALSE
        downstream — diagnosis's independent VQA gets the final word.
        """
        if spec.name not in ("in", "on", "visible") or not spec.args:
            return Value.UNKNOWN
        target = spec.args[1] if len(spec.args) > 1 else spec.args[0]
        self.log.event("info_gathering", predicate=spec.key, inspect=target)
        self._steps_executed += 1
        result = self.registry.execute(self.ctx, "inspect", {"entity": spec.args[0]}
                                       if self.belief.track(spec.args[0]).xyz_base else {"entity": target})
        self.log.event("skill_exec", step_id="(info)", skill="inspect", ok=result.ok,
                       info=result.info, error=result.error)
        sat, _ = check(self.ctx, spec)
        return sat

    # -- failure handling ----------------------------------------------------
    @staticmethod
    def _signature(report: FailureReport) -> str:
        """What this failure IS, with the measurements taken out.

        Numbers are stripped because the eleven identical failures below were
        reported at 0.435 and 0.436 in alternation — the same fact, jittering in
        the third decimal, which is enough to look like eleven different
        problems to anything comparing strings.
        """
        return f"{report.failure_class.value}|{re.sub(r'[-+]?[0-9]*[.]?[0-9]+', '#', report.summary)}"

    def _giving_up_on(self, program: TaskProgram, report: FailureReport) -> bool:
        """Stop when the same failure has come back unchanged too many times.

        THE REPAIR CAP COUNTED STEP IDS, AND `replace_tail` MINTS NEW ONES. So
        every repair reset the counter it was supposed to be spending, and the
        cap never bound. Measured in the twin: an object landed at (0.436, 0.238),
        outside the arm's envelope, and

            ValueError: nothing above (0.436, 0.238) is reachable between
            z=0.000 and z=0.180 — the object is outside this

        came back ELEVEN times, about thirty seconds apart, until the episode's
        whole edit budget was gone. 442 seconds, 18 steps, 12 edits, for a fact
        that was equally true at the first attempt: the arm cannot go there.

        Retrying is a bet that something has changed. Nothing had. Counting the
        failure rather than the step is what makes that bet checkable, and this
        is the only cap that survives a repair renaming everything.
        """
        sig = self._signature(report)
        self._seen_failures[sig] = self._seen_failures.get(sig, 0) + 1
        if self._seen_failures[sig] < getattr(self.cfg.budgets, "max_same_failure", 3):
            return False
        reason = (f"the same failure has come back {self._seen_failures[sig]} times unchanged — "
                  f"{report.summary}. Repairing it again would be a bet that something has "
                  f"changed, and nothing has")
        self.log.event("same_failure_repeated", times=self._seen_failures[sig],
                       failure_class=report.failure_class.value, summary=report.summary)
        # One full replan per failure signature before giving up: local
        # edits patch steps, but an unchanged failure means the APPROACH is
        # wrong, and the planner — shown the failure — can choose another.
        if sig not in self._replanned_sigs:
            self._replanned_sigs.add(sig)
            if self._replan(program, report):
                self._seen_failures[sig] = 0
                return False
        program.apply_edit(ProgramEdit(type=EditType.ABORT, rationale=reason, source="rules"))
        return True

    def _handle_failure(self, program: TaskProgram, step: Step, report: FailureReport) -> None:
        if self._giving_up_on(program, report):
            return
        if step.attempts > step.max_attempts and report.failure_class in (
            FailureClass.SKILL_EFFECT, FailureClass.SKILL_EXEC
        ):
            report = escalate(report)
            self.log.event("failure_escalated", step_id=step.id, summary=report.summary)

        edits = rule_based_repair(self.ctx, program, step, report)
        if edits == []:  # PERCEPTION contradiction: postcondition actually holds
            self.log.event("perception_override", step_id=step.id,
                           note="independent verifier confirms postcondition; accepting step")
            self._mark_done(program, step)
            return
        if edits is not None and self._repairs_per_step.get(step.id, 0) < self.cfg.budgets.max_repairs_per_step:
            for edit in edits:
                program.apply_edit(edit)
                self._edits_applied += 1
                self.log.event("program_edit", step_id=step.id, edit=edit)
            self._repairs_per_step[step.id] = self._repairs_per_step.get(step.id, 0) + 1
            return
        self._orchestrator_repair(program, step, report)


    def _replan(self, program: TaskProgram, report: FailureReport) -> bool:
        """Throw away the remaining plan and ask the planner for a fresh one,
        with the failure in its face. Local edits patch steps; this changes
        the approach. The GOAL PREDICATES ARE KEPT — the task is the same,
        only the road is new — and the repair counters reset with the step
        ids they were counting.
        """
        frames = self._fixed_frames()
        for f in frames:
            self.log.frame(f, tag="replan_context")
        note = (f"A previous attempt failed: {report.summary}. "
                "Plan a different approach for the same goal.")
        try:
            fresh = self.orchestrator.plan(f"{program.goal} ({note})", frames,
                                           self._catalog(),
                                           self._hints_with_rules())
        except Exception as e:
            self.log.event("replan_failed", error=f"{type(e).__name__}: {e}"[:200])
            return False
        try:
            self._expand_learned(fresh)
            self._normalize_arms(fresh)
            self._fill_default_posts(fresh)
            self._reject_unknown_skills(fresh)
        except Exception as e:
            self.log.event("replan_failed", error=f"prepare: {type(e).__name__}: {e}"[:200])
            return False
        # A REPLAN RETIRES EVERY STEP ID IT REPLACES. Repair bookkeeping and
        # any pending edit still name the old ids, and the first lookup after
        # the swap raised KeyError("no step 'r3_1'") — two episodes crashed
        # outright today. Swap the steps and the ledgers together.
        program.steps = fresh.steps
        for e in fresh.entities:
            if e.id not in {x.id for x in program.entities}:
                program.entities.append(e)
        self.belief.declare_entities(program.entities)
        self._repairs_per_step.clear()
        self._values.clear()
        program.edits = []
        self._edits_applied += 1
        self.log.event("replan", steps=[s.skill for s in fresh.steps],
                       why=report.summary[:160])
        self.log.snapshot(f"program_replan_{self._edits_applied}", program)
        return True

    def _orchestrator_repair(self, program: TaskProgram, step: Optional[Step], report: FailureReport) -> bool:
        """Ask the orchestrator for a typed edit; validate before applying."""
        if step is not None and self._repairs_per_step.get(step.id, 0) >= self.cfg.budgets.max_repairs_per_step:
            self.log.event("repair_cap", step_id=step.id)
            if self._replan(program, report):
                return True
            program.apply_edit(ProgramEdit(
                type=EditType.ABORT,
                rationale=f"step {step.id} consumed its repair budget "
                          f"({self.cfg.budgets.max_repairs_per_step}) without progress", source="rules"))
            return False
        frames = self._fixed_frames()
        for f in frames:
            self.log.frame(f, tag="repair_context")
        edit = None
        request = report
        for attempt in range(2):  # a malformed proposal gets one retry with feedback
            try:
                edit = self.orchestrator.propose_repair(program, self.belief.summary(), request,
                                                        self._catalog(), frames)
            except Exception as e:
                self.log.event("repair_error", error=str(e))
                program.apply_edit(ProgramEdit(type=EditType.ABORT, rationale=f"repair failed: {e}",
                                               source="rules"))
                return False
            err = validate_edit(program, edit, self.registry.names())
            if err is None and program.edits and edit.type is EditType.RETRY_STEP:
                last = program.edits[-1]
                if last.type is EditType.RETRY_STEP and last.step_id == edit.step_id \
                        and last.new_args == edit.new_args:
                    err = ("this exact retry already ran and failed — choose a different edit "
                           "(replace_step / replace_tail with corrected args)")
            if err is None:
                break
            self.log.event("repair_rejected", error=err, edit=edit, attempt=attempt + 1)
            request = report.model_copy(deep=True)
            request.summary += f" | YOUR PREVIOUS EDIT WAS INVALID: {err}. Fix exactly that."
            edit = None
        if edit is None:
            program.apply_edit(ProgramEdit(type=EditType.ABORT,
                                           rationale="orchestrator repair invalid twice", source="rules"))
            return False
        program.apply_edit(edit)
        self._normalize_arms(program)
        self._edits_applied += 1
        if step is not None:
            self._repairs_per_step[step.id] = self._repairs_per_step.get(step.id, 0) + 1
        self.log.event("program_edit", step_id=report.step_id, edit=edit)
        if edit.type is EditType.REGROUND_ENTITY and edit.entity_id:
            self.belief.declare_entities([e for e in program.entities if e.id == edit.entity_id])
            self.belief.invalidate_entity(edit.entity_id, reason="re-described by repair")
        return edit.type is not EditType.ABORT

    def _fill_default_posts(self, program: TaskProgram) -> None:
        """Give bare steps their canonical postconditions.

        A low-thinking planner emits steps with no post specs at all; every
        layer built on postconditions (step verification, repair targeting,
        process-mode grading) then starves. The semantics of the primitives
        are not the planner's to invent — pick means holding, place onto a
        target means on — so fill them here, typed-program completion in the
        same spirit as inserting the missing pick.
        """
        arms = list(self.cfg.active_arms or self.robot.arms)
        arm = arms[0] if len(arms) == 1 else None
        for st in program.steps:
            if st.post:
                continue
            e = st.args.get("entity")
            a2 = st.args.get("arm") if st.args.get("arm") in (arms or []) else arm
            specs = []
            if st.skill == "pick" and e and a2:
                specs = [PredicateSpec(name="holding", args=[e, a2])]
            elif st.skill == "place" and e:
                tgt = st.args.get("target")
                if tgt:
                    specs = [PredicateSpec(name="on", args=[e, tgt])]
                if a2:
                    specs.append(PredicateSpec(name="holding", args=[e, a2],
                                               negated=True))
            elif st.skill == "open_gripper" and a2:
                specs = [PredicateSpec(name="gripper_empty", args=[a2])]
            if specs:
                st.post = specs
                self.log.event("default_posts_filled", step_id=st.id,
                               skill=st.skill, posts=[str(p) for p in specs])

    def _normalize_arms(self, program: TaskProgram) -> None:
        """Coerce arm references to the ACTIVE arm set.

        Single-arm is an operating MODE, not only a hardware fact: a dual-arm
        rig running right-arm-only sets cfg.active_arms=["right"], and a plan
        (or its goals) may not reference the parked arm — a goal of
        gripper_empty(left) on an unsensed arm sat UNKNOWN forever and failed
        an episode whose task had physically succeeded at 76 s.
        """
        arms = list(self.cfg.active_arms or self.robot.arms)
        if len(arms) != 1:
            return
        arm = arms[0]
        for s in program.steps:
            if "arm" in s.args and s.args["arm"] != arm:
                s.args["arm"] = arm
            # Guards too. `when` arrived after this sweep was written, and a
            # guard of holding(x, auto) slipped through: "auto" is an ARG
            # convention, never an arm name, so the verifier answered UNKNOWN,
            # the guard ran the step, and place() raised on an empty gripper —
            # repaired downstream, but the cascade skip should have been free.
            for spec in list(s.pre) + list(s.post) + ([s.when] if s.when else []):
                if spec.name == "holding" and len(spec.args) > 1 and spec.args[1] != arm:
                    spec.args[1] = arm
                if spec.name == "gripper_empty" and spec.args and spec.args[0] != arm:
                    spec.args[0] = arm
        for spec in program.goal_predicates:
            if spec.name == "holding" and len(spec.args) > 1 and spec.args[1] != arm:
                spec.args[1] = arm
            if spec.name == "gripper_empty" and spec.args and spec.args[0] != arm:
                spec.args[0] = arm

    # -- goal & budgets ------------------------------------------------------
    VIEWING_PARK_XYZ = (0.16, -0.26, 0.14)   # clear of both fixed cameras

    def _clear_view_for_goal(self) -> None:
        """The arm that finished the task is usually parked square over its
        own work — after a stack release both cameras honestly reported the
        blocks invisible and the goal check failed on visibility. Before the
        final verdict, tuck the arm to a corner the cameras ignore."""
        import numpy as np
        for arm in (self.cfg.active_arms or self.robot.arms):
            try:
                # THE PARK IS A POSE IN THE ARM'S OWN FRAME, not a world point.
                # Sent as a world point it means "go to the right arm's corner"
                # — which for the left arm is 70 cm inside its dead zone; the
                # IK tilt fallbacks found some contorted solution and the idle
                # arm swung up for no reason (watched live, 2026-08-06).
                t = np.asarray(self.cfg.arms[arm].t_world_base, dtype=float)
                park = (t @ np.array([*self.VIEWING_PARK_XYZ, 1.0]))[:3]
                self.robot.move_cartesian(arm, park, seconds=1.2)
            except Exception:
                pass

    def _goal_check(self, program: TaskProgram) -> list[PredicateSpec]:
        self._clear_view_for_goal()
        if getattr(program, "process_goal", False):
            # Process mode: the journey was the goal. Every state-changing step
            # ran and its postconditions were verified at the time; a terminal
            # snapshot that equals the start is exactly what success looks like.
            # A step a REPAIR skipped is not an unmet obligation: the edit
            # that skipped it either replaced it (the replacements ran and
            # verified) or judged it unnecessary — a skipped second pick's
            # holding() post failed an episode whose goal was verified on
            # the table (rig, 2026-08-05 11:37).
            undone = [s for s in program.steps
                      if s.post and s.status not in (StepStatus.DONE,
                                                     StepStatus.SKIPPED)]
            self.log.event("process_goal_check", undone=[s.id for s in undone])
            return [spec for s in undone for spec in s.post]
        unmet = []
        for spec in program.goal_predicates:
            sat, primary = check(self.ctx, spec)
            if sat is Value.UNKNOWN:
                sat = self._gather_information(spec)
                primary = None
            if sat is Value.TRUE:
                # Declaring the task done is the one verdict nobody downstream
                # re-checks, so it gets a second, INDEPENDENT opinion. The
                # geometric test compares two of our own estimates, which share
                # the pointing bias and cancel it out; asking the model what it
                # sees is the only estimator in the stack that does not.
                alt_sat, alt = check(self.ctx, spec, alternate=True)
                if alt_sat is Value.FALSE and alt.confidence >= GOAL_VETO_MIN_CONF:
                    self.log.event("goal_veto", predicate=spec.key,
                                   confidence=round(alt.confidence, 2), note=alt.note)
                    sat = Value.FALSE
                elif (primary is not None and primary.confidence < GOAL_NEEDS_BACKUP_CONF
                        and alt_sat is not Value.TRUE):
                    # A NARROW geometric yes that the independent look cannot
                    # corroborate is not a finished task. Measured: `in(block,
                    # tray)` at 35 mm against a 42 mm cavity — inside the error
                    # of the estimates it was computed from — while the second
                    # opinion said "no red block is clearly visible". Ground
                    # truth: 100 mm. The episode was recorded as a success and
                    # the skill library learned from it.
                    #
                    # Only NARROW calls need the backup. A comfortable margin
                    # keeps its confidence and stands on its own, which is what
                    # lets a container legitimately hide its contents without
                    # every `in` becoming unverifiable.
                    self.log.event("goal_unconfirmed", predicate=spec.key,
                                   primary_confidence=round(primary.confidence, 2),
                                   second_opinion=alt_sat.value, note=alt.note[:100])
                    sat = Value.UNKNOWN
            if sat is not Value.TRUE:
                unmet.append(spec)
        self.log.event("goal_check", unmet=[str(p) for p in unmet])
        return unmet

    def _budget_exceeded(self, program: TaskProgram) -> Optional[str]:
        b = self.cfg.budgets
        if self._steps_executed >= b.max_steps_executed:
            return f"step budget exhausted ({b.max_steps_executed})"
        if self._edits_applied >= b.max_edits:
            return f"edit budget exhausted ({b.max_edits})"
        if time.time() - self._t0 > b.wall_clock_s:
            return f"wall clock exceeded ({b.wall_clock_s}s)"
        return None

    def _fixed_frames(self) -> list[Frame]:
        # Context for the planner. A camera that will not open must not end the
        # episode: cam_low is uncalibrated and contributes nothing but a second
        # view, and its UVC index moves every time another camera is handed to
        # librealsense. Losing the view is a degradation; losing the run is not
        # a proportionate response to it.
        frames = []
        for c in ("cam_high", "cam_low"):
            if c not in self.robot.cameras:
                continue
            try:
                frames.append(self.robot.capture(c))
            except Exception as e:
                self.log.event("camera_unavailable", camera=c,
                               error=f"{type(e).__name__}: {str(e)[:150]}")
        if not frames:
            raise RuntimeError("no camera could be opened — nothing can be grounded")
        return frames
