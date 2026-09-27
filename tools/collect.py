"""Autonomous LeRobot-format data collection, in the twin or at the rig.

tools/lifelong.py answers "does the agent get better?"; this answers "can the
agent be the data collector?" — the loop is the same shape, but every episode
is recorded at control rate into a LeRobotDataset for pi0.5 fine-tuning via
Trossen's openpi fork, and the between-episode reset works on hardware:

  * the twin teleports objects (sim.place_body), exactly as lifelong.py does;
  * the rig cannot teleport, so the AGENT rearranges — ground each block,
    pick it, place it at a random reachable point, using the same primitives
    and the same sample_layout/reachability checks as the teleport path.

Only VERIFIED successes teach (the standing doctrine): every episode is
recorded, each carries the verifier's verdict — and the twin's ground-truth
landing — in the dataset's sidecar, and tools/filter_success.py exports the
success-only subset.

    python tools/collect.py --config configs/trossen_sorting.yaml --episodes 2
    python tools/collect.py --config configs/abaka.yaml --episodes 100 --say

HARD RULE: the real rig is never run unattended. 100 episodes is 2.5-4 hours
of a human within reach of the estop; the pause gate below assumes exactly
that person exists.
"""
from __future__ import annotations

import argparse
import json
import random
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import lifelong  # noqa: E402  (sample_layout, make_task, landed, BLOCKS, ...)

# Pause (not abort) after this many consecutive non-successes: something
# systematic is wrong — lighting, a dropped block out of reach, a wedged
# gripper — and burning the remaining budget on it helps nobody.
MAX_CONSECUTIVE_FAILURES = 3
DEFAULT_IMAGE_HW = (480, 640)

# What the real-rig rearrangement moves. One block per colour, deliberately:
# ground_entity tells duplicates apart by a position prior, and a rearrange
# step has no prior — it is the step that scrambles them. Randomising one of
# each colour still gives every episode a fresh layout.
REARRANGE_ENTITIES = {"red_block": "the red block", "blue_block": "the blue block"}


def rearrange_entities(cfg):
    """Scene inventory is a property of today's table, not of the codebase:
    configs may override with collect_rearrange_entities / _supports
    ({name: description} maps). Defaults preserve the original scene."""
    return dict(getattr(cfg, "collect_rearrange_entities", None) or REARRANGE_ENTITIES)


def rearrange_supports(cfg):
    return dict(getattr(cfg, "collect_rearrange_supports", None) or REARRANGE_SUPPORTS)
# Supports are grounded before sampling so the randomised layout keeps out of
# them — a block reset onto the plate hands the next episode a pre-solved task.
REARRANGE_SUPPORTS = {"white_plate": "the white plate", "grey_tray": "the grey tray"}


def speak(text: str, enabled: bool = True) -> None:
    """macOS `say`, fire-and-forget. Silence is never an error."""
    if not enabled:
        return
    try:
        subprocess.Popen(["say", text], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    except Exception:
        pass


def eta_str(done: int, total: int, wall_times: list[float]) -> str:
    if not wall_times or done >= total:
        return "eta unknown"
    remaining = (total - done) * (sum(wall_times[-10:]) / len(wall_times[-10:]))
    h, m = int(remaining // 3600), int(remaining % 3600 // 60)
    return f"eta {h}h{m:02d}m" if h else f"eta {m}m"


class FailureGate:
    """Auto-PAUSE after N consecutive failures; a human decides what happens next."""

    def __init__(self, limit: int = MAX_CONSECUTIVE_FAILURES, say_enabled: bool = True,
                 on_pause: str = "ask") -> None:
        self.limit = limit
        self.say_enabled = say_enabled
        self.on_pause = on_pause
        self.streak = 0
        self.reasons: list[str] = []

    def record(self, success: bool, reason: str = "") -> None:
        if success:
            self.streak, self.reasons = 0, []
        else:
            self.streak += 1
            self.reasons.append(reason or "unknown")

    def should_pause(self) -> bool:
        return self.limit > 0 and self.streak >= self.limit

    def pause(self) -> bool:
        """True = resume, False = stop the run."""
        why = f"{self.streak} consecutive failures; last: {self.reasons[-1][:120]}"
        print(f"\n[collect] PAUSED — {why}", flush=True)
        speak(f"Collection paused after {self.streak} consecutive failures.",
              self.say_enabled)
        if self.on_pause == "continue":
            self.streak, self.reasons = 0, []
            return True
        if self.on_pause == "abort" or not sys.stdin.isatty():
            # Nobody can answer a prompt here; stopping is the safe reading of
            # "three failures in a row" when the operator cannot weigh in.
            print("[collect] no interactive operator — stopping the run", flush=True)
            return False
        answer = input("[collect] fix the scene, then Enter to resume "
                       "(q + Enter stops the run): ").strip().lower()
        if answer.startswith("q"):
            return False
        self.streak, self.reasons = 0, []
        return True


# -- between-episode rearrangement -------------------------------------------
VIEWING_PARK = (0.16, -0.26, 0.14)   # right-front corner, clear of the overhead view


def _park_for_viewing(robot) -> None:
    """The staged home hovers over the table centre and hides exactly what the
    scramble needs to see (a red block sat invisible under the parked gripper).
    Before grounding, tuck the arm to a corner the overhead camera ignores."""
    try:
        robot.move_cartesian("right", np.array(VIEWING_PARK), seconds=1.5)
    except Exception:
        pass


def _pick_with_regrasp(ctx, ground_entity, pick, name: str, tries: int = 3) -> None:
    """The agent's repair loop regrasps on a felt miss; the scripted reset
    paths just died. Same reflex here: closed-on-air -> fresh grounding ->
    try again, up to `tries` grasps."""
    for attempt in range(tries):
        try:
            pick(ctx, name)
            return
        except Exception as e:
            if "closed on air" not in str(e) or attempt == tries - 1:
                raise
            print(f"[collect] {name}: grasp closed on air — re-grounding for "
                  f"regrasp {attempt + 2}/{tries}", flush=True)
            found, note = ground_entity(ctx, name)
            if not found:
                raise RuntimeError(f"lost {name} for regrasp: {note}") from e


def rearrange_real(robot, cfg, log, orchestrator, segmenter, rng: random.Random) -> bool:
    """Hardware cannot teleport: pick each block and put it somewhere new.

    Same sampler and the same reachability standard as the twin's teleport
    path (lifelong.rearrange), so the two backends see statistically the same
    layouts. Returns False when a layout or a move could not be made — the
    caller reseeds and tries again rather than running on a half-shuffled
    scene.
    """
    from heron.belief import BeliefStore
    from heron.skills import SkillContext
    from heron.skills.primitives import OPEN_WIDTH, pick, place
    from heron.skills.sensing import ground_entity
    from heron.types import EntityDecl

    ctx = SkillContext(robot=robot, belief=BeliefStore(), grounder=orchestrator,
                       cfg=cfg, log=log, memory=None, segmenter=segmenter)
    ctx.belief.declare_entities([EntityDecl(id=k, description=v)
                                 for k, v in rearrange_entities(cfg).items()])
    # The blue table CLAMP at x=0.47 sits between the arena edge (0.43) and
    # the workspace box (0.53): reach-preference admitted it, the arena guard
    # then rejected it, three strikes, run over. For the scramble, candidate
    # preference uses the ARENA itself.
    ctx.grounding_bounds = (lifelong.X_RANGE, lifelong.Y_RANGE)
    # The scramble step has no history to lean on, and ground_entity with no
    # prior takes the detector's first candidate — which, on the rig, was blue
    # junk at the table's edge 0.35 m outside the workspace while the real
    # block sat centre-table (2026-08-03, three aborted runs). A weak
    # workspace-centre prior makes the existing association machinery prefer
    # the candidate inside the arena; it steers only the CHOICE among
    # lookalikes, never invents a position (grounding stays visual).
    cx = sum(lifelong.X_RANGE) / 2.0
    cy = sum(lifelong.Y_RANGE) / 2.0
    for k in REARRANGE_ENTITIES:
        ctx.belief.update_track(k, xyz_base=(cx, cy, cfg.table_z), confidence=0.1)
    robot.set_gripper("right", OPEN_WIDTH)
    _park_for_viewing(robot)
    # Ground the supports first: their positions define keep-out discs for the
    # sampler. A support that cannot be seen simply contributes no disc — the
    # reset must not die because a tray is missing from this particular table.
    avoid: list[tuple[float, float, float]] = []
    for sup, desc in rearrange_supports(cfg).items():
        if sup not in ctx.belief.entities:
            ctx.belief.declare_entities([EntityDecl(id=sup, description=desc)])
        try:
            ok, _ = ground_entity(ctx, sup)
        except Exception:
            ok = False
        if ok:
            tr = ctx.belief.track(sup)
            half = float(tr.span_m) / 2.0 if tr.span_m else 0.065
            avoid.append((float(tr.xyz_base[0]), float(tr.xyz_base[1]), half))
    spots = lifelong.sample_layout(rng, len(REARRANGE_ENTITIES), avoid=avoid)
    if len(spots) < len(REARRANGE_ENTITIES):
        return False
    for (x, y) in spots:
        if not all(robot.reachable("right", np.array([x, y, z]))
                   for z in (0.03, 0.06, 0.10)):
            return False
    names = list(rearrange_entities(cfg))
    rng.shuffle(names)
    for name, (x, y) in zip(names, spots):
        try:
            found, note = ground_entity(ctx, name)
            if not found:
                # Not on today's table (scene inventories drift) — rearrange
                # what IS here rather than abandoning the whole layout.
                print(f"[collect] rearrange: cannot see {name} — skipping it "
                      f"({note})", flush=True)
                continue
            gx, gy = (float(v) for v in ctx.belief.track(name).xyz_base[:2])
            # A grounding outside the arena is a mis-association (junk at the
            # table's edge wearing the right colour), not a block to fetch.
            # Fail fast and clean — before the arm swings at it — so the retry
            # loop reseeds and the detector gets another look.
            if not (lifelong.X_RANGE[0] - 0.03 <= gx <= lifelong.X_RANGE[1] + 0.03
                    and lifelong.Y_RANGE[0] - 0.03 <= gy <= lifelong.Y_RANGE[1] + 0.03):
                print(f"[collect] rearrange: {name} grounded outside the arena "
                      f"at [{gx:.3f}, {gy:.3f}] — rejecting the association", flush=True)
                ctx.belief.invalidate_entity(name, reason="grounded outside arena")
                return False
            _pick_with_regrasp(ctx, ground_entity, pick, name)
            place(ctx, name, x=float(x), y=float(y))
        except Exception as e:
            print(f"[collect] rearrange failed on {name}: "
                  f"{type(e).__name__}: {e}", flush=True)
            try:
                robot.set_gripper("right", OPEN_WIDTH)
                robot.home("right")
            except Exception:
                pass
            return False
    robot.home("right")
    # Verified reset, same doctrine as the tasks: look again and CHECK the
    # layout — off every support's disc, inside the arena. One corrective move
    # per block, then report honestly.
    for name, (x, y) in zip(names, spots):
        try:
            found, _ = ground_entity(ctx, name)
        except Exception:
            found = False
        if not found:
            continue
        bx, by = (float(v) for v in ctx.belief.track(name).xyz_base[:2])
        on_support = any((bx - ax) ** 2 + (by - ay) ** 2 < (ar + 0.02) ** 2
                         for ax, ay, ar in avoid)
        in_arena = (lifelong.X_RANGE[0] - 0.02 <= bx <= lifelong.X_RANGE[1] + 0.02
                    and lifelong.Y_RANGE[0] - 0.02 <= by <= lifelong.Y_RANGE[1] + 0.02)
        if not in_arena:
            # An out-of-arena re-grounding is a lookalike mis-association, not
            # a block to chase with the arm. Drop the claim and let the caller
            # reseed — the block we actually placed is where we placed it.
            print(f"[collect] reset verify: {name} re-grounded outside the arena "
                  f"at [{bx:.3f}, {by:.3f}] — rejecting the association", flush=True)
            ctx.belief.invalidate_entity(name, reason="re-grounded outside arena")
            continue
        if on_support:
            retry = lifelong.sample_layout(rng, 1, avoid=avoid)
            if not retry:
                return False
            try:
                _pick_with_regrasp(ctx, ground_entity, pick, name)
                place(ctx, name, x=float(retry[0][0]), y=float(retry[0][1]))
                robot.home("right")
            except Exception as e:
                print(f"[collect] reset re-do failed on {name}: "
                      f"{type(e).__name__}: {e}", flush=True)
                return False
            log.event("reset_redo", entity=name, was=[round(bx, 3), round(by, 3)],
                      reason="on support" if on_support else "outside arena")
    return True


def rearrange(backend, robot, cfg, log, orchestrator, segmenter,
              rng: random.Random, reset_mode: str = "auto",
              fixed_supports: bool = False) -> bool:
    if reset_mode != "agent" and hasattr(backend, "place_body"):
        return lifelong.rearrange(backend, rng, fixed_supports=fixed_supports)
    return rearrange_real(robot, cfg, log, orchestrator, segmenter, rng)


def run_reset_episode(robot, cfg, log, orchestrator, segmenter,
                      rng: random.Random, colour: str, support: str) -> tuple[bool, str]:
    """Take the block OFF the support and put it at a random clear spot.

    This is a RECORDED episode, not backstage shuffling: a fleet that learns
    only "put it on the plate" can never reset itself, so the retrieval is a
    first-class task in the dataset. Script-driven (ground -> pick -> place at
    a keep-out-sampled point -> verify off-support) because the plan is always
    the same two primitives — no reason to pay a planner for it.
    """
    from heron.belief import BeliefStore
    from heron.skills import SkillContext
    from heron.skills.primitives import OPEN_WIDTH, pick, place
    from heron.skills.sensing import ground_entity
    from heron.types import EntityDecl

    block = f"{colour}_block"
    ctx = SkillContext(robot=robot, belief=BeliefStore(), grounder=orchestrator,
                       cfg=cfg, log=log, memory=None, segmenter=segmenter)
    ctx.belief.declare_entities([
        EntityDecl(id=block, description=f"the {colour} block"),
        EntityDecl(id=support, description=REARRANGE_SUPPORTS.get(
            support, f"the {support.replace('_', ' ')}")),
    ])
    robot.set_gripper("right", OPEN_WIDTH)
    try:
        ok, note = ground_entity(ctx, support)
        if not ok:
            return False, f"cannot see {support}: {note}"
        tr = ctx.belief.track(support)
        sx, sy = float(tr.xyz_base[0]), float(tr.xyz_base[1])
        half = float(tr.span_m) / 2.0 if tr.span_m else 0.065
        # The block is ON the support — seed the prior there so lookalikes
        # elsewhere on (or off) the table lose the association.
        ctx.belief.update_track(block, xyz_base=(sx, sy, cfg.table_z + 0.02),
                                confidence=0.3)
        ok, note = ground_entity(ctx, block)
        if not ok:
            return False, f"cannot see {block}: {note}"
        avoid = [(sx, sy, half)]
        # A FIXED HOME FOR THE BLOCK, WHEN ONE IS ASKED FOR. Randomising the
        # reset spread the training distribution, which is what a policy wants
        # from a big dataset — and what a small one cannot afford. Twenty
        # episodes that each start somewhere else teach twenty starts; twenty
        # from the same place teach one task. It is also the difference between
        # a demo that works and one that works sometimes.
        fixed = getattr(cfg, "collect_reset_xy", None)
        if fixed:
            spot = [(float(fixed[0]), float(fixed[1]))]
            log.event("reset_to_fixed_spot", xy=[round(v, 3) for v in spot[0]])
        else:
            spot = lifelong.sample_layout(rng, 1, avoid=avoid)
        if not spot:
            return False, "no clear spot to reset to"
        _pick_with_regrasp(ctx, ground_entity, pick, block)
        place(ctx, block, x=float(spot[0][0]), y=float(spot[0][1]))
        robot.home("right")
        # Verify the point of the exercise: the block is OFF the support.
        ok, note = ground_entity(ctx, block)
        if not ok:
            return False, f"lost {block} after reset: {note}"
        bx, by = (float(v) for v in ctx.belief.track(block).xyz_base[:2])
        if (bx - sx) ** 2 + (by - sy) ** 2 < (half + 0.02) ** 2:
            return False, f"{block} still on {support} at [{bx:.3f}, {by:.3f}]"
        return True, f"{block} reset to [{bx:.3f}, {by:.3f}]"
    except Exception as e:
        try:
            robot.set_gripper("right", OPEN_WIDTH)
            robot.home("right")
        except Exception:
            pass
        return False, f"{type(e).__name__}: {e}"


# -- the loop -----------------------------------------------------------------
def run_truth_teacher(backend, robot, cfg, log, colour: str, support: str) -> dict:
    """Do the task with the primitives on the twin's own poses, then let the
    verifier judge it exactly as it judges the agent.

    A DEMONSTRATION IS NOT A PLAN. What a policy learns from is the trajectory,
    and routing every episode through the planner buys nothing for that while
    costing ~14 model calls and a day's worth of VLM variance: a success-rate
    curve collected that way measures the policy and the orchestrator's mood at
    once. The teacher here has fixed, known competence — it looks the answer up
    in MuJoCo — so the only thing that moves between rounds is the policy.

    The LABEL still comes from the verifier, through the same orchestrator and
    the same predicates the agent is graded by. That is the whole point: the
    audit in tools/audit_verifier.py compares that label against ground truth,
    and it would mean nothing if the teacher graded its own homework.
    """
    import numpy as np  # noqa: PLC0415

    from heron.belief import BeliefStore  # noqa: PLC0415
    from heron.orchestrator.simtruth import SimTruthGrounder  # noqa: PLC0415
    from heron.skills import SkillContext  # noqa: PLC0415
    from heron.skills.primitives import park_for_look, pick, place  # noqa: PLC0415
    from heron.types import PredicateSpec  # noqa: PLC0415
    from heron.verify import verify  # noqa: PLC0415

    # Any block of the named colour, nearest the arm among the ones it can
    # reach — the instruction did not single one out, and neither should this.
    arm = (cfg.active_arms or robot.arms)[0]
    named = [n for n, c in lifelong.BLOCKS.items() if c == colour]
    reachable = []
    for n in named:
        try:
            p = backend.body_xyz(n)
        except KeyError:
            continue
        if robot.reachable(arm, np.array([p[0], p[1], max(p[2], cfg.table_z) + 0.03])):
            reachable.append((float(np.linalg.norm(p[:2])), n))
    if not reachable:
        return {"status": "crashed", "reason": f"no reachable {colour} block on the table",
                "steps_executed": 0, "edits_applied": 0}
    block = min(reachable)[1]

    belief = BeliefStore()
    for name in (block, support):
        belief.update_track(name, xyz_base=tuple(backend.body_xyz(name)),
                            confidence=1.0, from_sighting=True)
    # THE VERIFIER READS THE DESCRIPTION, NOT THE BODY NAME. "red_block_2" is a
    # name only MuJoCo knows, and handed to a detector as "red block 2" it asks
    # for a distinction no camera can make: two identical cubes, and the first
    # smoke run grounded the wrong one and reported the block 234 mm from a
    # tray it was sitting 5 mm inside. The instruction says "the red block" and
    # ground truth accepts any block of that colour; the verifier should be
    # asked the same question, in the same words.
    belief.track(block).description = f"the {colour} block"
    belief.track(support).description = lifelong.SUPPORTS.get(
        support, support.replace("_", " "))
    acting = SkillContext(robot=robot, belief=belief, grounder=SimTruthGrounder(backend),
                          cfg=cfg, log=log, segmenter=None)
    steps = 0
    try:
        pick(acting, entity=block)
        steps += 1
        place(acting, entity=block, target=support)
        steps += 1
        park_for_look(acting, arm)
    except Exception as e:                                        # noqa: BLE001
        return {"status": "crashed", "reason": f"{type(e).__name__}: {e}"[:300],
                "steps_executed": steps, "edits_applied": 0}

    # NOW ASK THE INSTRUMENT THAT GRADES THE AGENT. The belief is cleared first
    # so the verifier has to look: judging from positions the teacher wrote
    # would be the teacher marking its own paper in a slower way.
    from heron.orchestrator.gemini import GeminiOrchestrator  # noqa: PLC0415

    belief.invalidate_entity(block, reason="placed; the verifier must look")
    belief.invalidate_entity(support, reason="placed; the verifier must look")
    judging = SkillContext(robot=robot, belief=belief,
                           grounder=GeminiOrchestrator(cfg, log),
                           cfg=cfg, log=log, segmenter=None)
    spec = PredicateSpec(name="in" if support == "grey_tray" else "on",
                         args=[block, support])
    try:
        verdict = verify(judging, spec)
    except Exception as e:                                        # noqa: BLE001
        return {"status": "unknown", "reason": f"verifier failed: {type(e).__name__}: {e}"[:300],
                "steps_executed": steps, "edits_applied": 0}
    from heron.types import Value  # noqa: PLC0415

    # UNKNOWN IS NOT A SUCCESS. Three-valued on purpose: an episode the
    # verifier could not see is not a demonstration it endorsed, and folding
    # it into "failed" would also lose the distinction the audit needs.
    status = {Value.TRUE: "succeeded", Value.FALSE: "failed"}.get(verdict.value, "unknown")
    note = verdict.note or str(verdict.value)
    return {"status": status,
            "reason": ("goal predicate verified" if status == "succeeded"
                       else f"{spec}: {note}")[:300],
            "steps_executed": steps, "edits_applied": 0}


def run_collection(config: str, episodes: int, dataset_root: str,
                   repo_id: str = "heron/trossen-pick-place",
                   fps: int = 30, cameras: list[str] | None = None,
                   image_hw: tuple[int, int] = DEFAULT_IMAGE_HW,
                   use_videos: bool = True, seed: int = 0,
                   say_enabled: bool = False, on_pause: str = "ask",
                   task_note: str = "", reset_mode: str = "auto",
                   supports: list[str] | None = None,
                   rearrange_entities: list[str] | None = None,
                   colour: str | None = None, same_words: bool = False,
                   teacher: str = "agent", fixed_supports: bool = False) -> dict:
    """Collect `episodes` recorded episodes. Returns a run summary dict."""
    from heron.agent import Agent
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator
    from heron.perception import SamSegmenter
    from heron.record import EpisodeRecorder, LeRobotWriter
    from heron.robot.safety import SafeRobot

    global REARRANGE_ENTITIES
    if rearrange_entities:
        REARRANGE_ENTITIES = {e: f"the {e.replace('_', ' ')}" for e in rearrange_entities}
    cfg = HeronConfig.load(config)
    if cfg.mode == "sim":
        from heron.robot.trossen_sim import TrossenSim

        # record_camera=None: the twin's episode film buffers frames in memory
        # and nothing here flushes it per episode, so over a hundred episodes
        # it is a slow leak — and the LeRobotDataset IS the recording now.
        backend = TrossenSim(cfg, scene=cfg.sim_scene, record_camera=None)
    elif cfg.mode == "real":
        from heron.robot.trossen import TrossenStationary

        backend = TrossenStationary(cfg)
    else:
        raise SystemExit(f"collection needs mode sim or real, not {cfg.mode!r}")
    robot = SafeRobot(backend, cfg.safety)

    cams = list(cameras) if cameras else list(backend.cameras)
    writer = LeRobotWriter(dataset_root, repo_id,
                           cameras={c: tuple(image_hw) for c in cams},
                           fps=fps, use_videos=use_videos)
    recorder = EpisodeRecorder(backend, writer, arm="right", cameras=cams)
    already = writer.num_episodes
    if already:
        print(f"[collect] resuming: dataset already holds {already} episodes")

    gate = FailureGate(say_enabled=say_enabled, on_pause=on_pause)
    rng = random.Random(seed + already)
    wall_times: list[float] = []
    collected = successes = 0
    is_sim = hasattr(backend, "place_body")
    per_episode_budget = cfg.budgets.wall_clock_s
    # THE HOME IS WHERE THE HUMAN PUT IT. A hand-typed reset coordinate is a
    # guess about a table you are not standing at; the block's own starting
    # position is the arrangement the operator actually set up, and pinning the
    # reset to it means every episode after the first begins where the first
    # one did. Only measured once, before anything moves.
    if colour and not getattr(cfg, "collect_reset_xy", None):
        from heron.belief import BeliefStore
        from heron.skills import SkillContext
        from heron.skills.sensing import ground_entity
        from heron.types import EntityDecl

        probe_log = EpisodeLogger(cfg.episodes_dir, "collect-home")
        probe = SkillContext(robot=robot, belief=BeliefStore(),
                             grounder=GeminiOrchestrator(cfg, probe_log),
                             cfg=cfg, log=probe_log, memory=None,
                             segmenter=SamSegmenter(cfg.sam_url, log=probe_log))
        probe.belief.declare_entities(
            [EntityDecl(id=f"{colour}_block", description=f"the {colour} block")])
        found, note = ground_entity(probe, f"{colour}_block", need_look=True)
        if found:
            xyz = probe.belief.track(f"{colour}_block").xyz_base
            cfg.collect_reset_xy = [round(float(xyz[0]), 4), round(float(xyz[1]), 4)]
            print(f"[collect] reset home taken from the first sighting: "
                  f"{cfg.collect_reset_xy}")
        else:
            print(f"[collect] could not see the {colour} block to set a reset home "
                  f"({note[:60]}) — resets will sample a spot")

    print(f"[collect] target {episodes} episodes -> {dataset_root} "
          f"(cameras {cams}, {fps} fps, per-episode budget {per_episode_budget:.0f}s)")
    speak(f"Starting collection of {episodes} episodes.", say_enabled)

    pending_reset: tuple[str, str] | None = None  # (colour, support) after a verified forward success
    scene_randomised = False  # True right after a successful reset episode
    try:
        while collected < episodes:
            episode_log = EpisodeLogger(cfg.episodes_dir, "collect")
            orch = GeminiOrchestrator(cfg, episode_log)
            segmenter = SamSegmenter(cfg.sam_url, log=episode_log)

            if pending_reset is not None and collected < episodes:
                r_colour, r_support = pending_reset
                pending_reset = None
                r_instruction = (f"pick up the {r_colour} block from the "
                                 f"{r_support.replace('_', ' ')} and place it on the table")
                speak(f"Episode {collected + 1} of {episodes}: {r_instruction}", say_enabled)
                t0 = time.time()
                recorder.start(r_instruction)
                r_ok, r_note = run_reset_episode(robot, cfg, episode_log, orch,
                                                 segmenter, rng, r_colour, r_support)
                index = recorder.stop(
                    instruction=r_instruction, colour=r_colour, support=r_support,
                    verifier_status="succeeded" if r_ok else "failed",
                    verifier_reason=r_note[:300], verified_success=r_ok,
                    steps=2, edits=0, episode_dir=str(episode_log.dir),
                    backend="sim" if is_sim else "real",
                    wall_s=round(time.time() - t0, 1), reset_wall_s=0.0,
                    reset_episode=True)
                wall = time.time() - t0
                wall_times.append(wall)
                collected += 1
                successes += int(r_ok)
                gate.record(r_ok, r_note)
                print(f"[collect] {collected}/{episodes} "
                      f"{'ep%s' % index if index is not None else 'DROPPED(too short)'} "
                      f"{r_instruction[:40]!r} -> {'succeeded' if r_ok else 'failed'} "
                      f"({successes} verified, {wall:.0f}s, RESET-EPISODE)", flush=True)
                if gate.should_pause() and not gate.pause():
                    break
                if collected >= episodes:
                    break
                scene_randomised = r_ok
                episode_log = EpisodeLogger(cfg.episodes_dir, "collect")
                orch = GeminiOrchestrator(cfg, episode_log)
                segmenter = SamSegmenter(cfg.sam_url, log=episode_log)

            reset_t0 = time.time()
            if scene_randomised:
                scene_randomised = False   # the reset episode already scrambled it
            elif not rearrange(backend, robot, cfg, episode_log, orch, segmenter, rng,
                               reset_mode=reset_mode, fixed_supports=fixed_supports):
                print("[collect] could not rearrange to a fresh reachable layout; "
                      "reseeding", flush=True)
                rng = random.Random(rng.random())
                gate.record(False, "rearrangement failed")
                if gate.should_pause() and not gate.pause():
                    break
                continue

            reset_wall = time.time() - reset_t0
            # ONE TASK, OR TWENTY FIFTHS OF FOUR TASKS. make_task samples the
            # colour and the phrasing, which is what a large dataset wants and
            # what a small one cannot survive: twenty episodes over four colours
            # is five demonstrations each, and the first two of this run came out
            # red when the scene had been set up around the blue cube.
            instruction, task_colour, support = lifelong.make_task(rng, supports=supports)
            if colour or same_words:
                task_colour = colour or task_colour
                if colour and supports:
                    support = supports[0]
                into = support == "grey_tray"
                phrasing = (lifelong.PHRASINGS[0] if same_words
                            else rng.choice(lifelong.PHRASINGS))
                instruction = phrasing.format(
                    colour=task_colour, prep="in" if into else "on",
                    motion="into" if into else "onto",
                    target=lifelong.SUPPORTS[support])
            speak(f"Episode {collected + 1} of {episodes}: {instruction}", say_enabled)
            t0 = time.time()
            recorder.start(instruction)
            try:
                if teacher == "truth":
                    summary = run_truth_teacher(backend, robot, cfg, episode_log,
                                                task_colour, support)
                else:
                    summary = Agent(robot, orch, cfg, log=episode_log).run(instruction)
            except Exception as e:   # one bad episode must not end the run
                summary = {"status": "crashed",
                           "reason": f"{type(e).__name__}: {e}",
                           "steps_executed": None, "edits_applied": None}
            status = summary.get("status")
            verified = status == "succeeded"
            meta = {
                "instruction": instruction, "colour": task_colour, "support": support,
                "verifier_status": status,
                "verifier_reason": str(summary.get("reason", ""))[:300],
                "verified_success": verified,
                "steps": summary.get("steps_executed"),
                "edits": summary.get("edits_applied"),
                "episode_dir": str(episode_log.dir),
                "backend": "sim" if is_sim else "real",
                "wall_s": round(time.time() - t0, 1),
                "reset_wall_s": round(reset_wall, 1),
            }
            if is_sim:   # benchmark-standard ground truth, where it exists
                on_support, centred, best = lifelong.landed(backend, task_colour, support)
                meta.update(gt_on_support=on_support, gt_centred=centred,
                            gt_dxy_mm=None if best == float("inf") else round(best * 1000))
            index = recorder.stop(**meta)

            wall = time.time() - t0
            wall_times.append(wall)
            collected += 1
            successes += int(verified)
            gate.record(verified, str(summary.get("reason", status)))
            if verified:
                # The block now sits on its goal; retrieving it is the next
                # recorded episode, not backstage shuffling.
                pending_reset = (task_colour, support)
            line = (f"[collect] {collected}/{episodes} "
                    f"{'ep%s' % index if index is not None else 'DROPPED(too short)'} "
                    f"{instruction[:40]!r} -> {status} "
                    f"({successes} verified, {wall:.0f}s, "
                    f"{eta_str(collected, episodes, wall_times)})")
            print(line, flush=True)
            if collected % 10 == 0 or collected == episodes:
                speak(f"{collected} of {episodes} episodes done, "
                      f"{successes} verified successes.", say_enabled)
            if gate.should_pause() and not gate.pause():
                break
    except KeyboardInterrupt:
        print("\n[collect] interrupted; the dataset is complete up to the last "
              "saved episode", flush=True)
        if recorder.recording:
            recorder.abort()
    finally:
        if recorder.recording:
            recorder.abort()
        writer.finalize()      # flush metadata: without this the dataset
        robot.shutdown()       # cannot be re-opened offline
    result = {"collected": collected, "verified_successes": successes,
              "dataset_root": str(dataset_root), "episodes_in_dataset": writer.num_episodes,
              "note": task_note}
    print(f"[collect] done: {json.dumps(result)}")
    speak(f"Collection finished. {successes} verified successes "
          f"out of {collected} episodes.", say_enabled)
    return result


def run_single(config: str, instruction: str, dataset_root: str | None = None,
               repo_id: str = "heron/trossen-pick-place", fps: int = 30,
               image_hw: tuple[int, int] = DEFAULT_IMAGE_HW,
               use_videos: bool = True, say_enabled: bool = False) -> dict:
    """One spoken instruction -> one recorded episode. The voice operator's
    direct-command path; no rearrangement, the scene is whatever the operator
    set up (or left behind)."""
    from heron.agent import Agent
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator
    from heron.record import EpisodeRecorder, LeRobotWriter
    from heron.robot.safety import SafeRobot

    cfg = HeronConfig.load(config)
    if cfg.mode == "sim":
        from heron.robot.trossen_sim import TrossenSim

        backend = TrossenSim(cfg, scene=cfg.sim_scene, record_camera=None)
    else:
        from heron.robot.trossen import TrossenStationary

        backend = TrossenStationary(cfg)
    robot = SafeRobot(backend, cfg.safety)
    episode_log = EpisodeLogger(cfg.episodes_dir, "voice")
    recorder = writer = None
    if dataset_root:
        writer = LeRobotWriter(dataset_root, repo_id,
                               cameras={c: tuple(image_hw) for c in backend.cameras},
                               fps=fps, use_videos=use_videos)
        recorder = EpisodeRecorder(backend, writer, arm="right")
    agent = Agent(robot, GeminiOrchestrator(cfg, episode_log), cfg, log=episode_log)
    # Human verifier channel: the console's 成功/失败 buttons drop a one-word
    # file; a watcher thread hands it to the agent, which ends the episode
    # with that verdict at the next check point (every step boundary and
    # every verification). The human outranks the goal checker — a stacked
    # block honestly occludes the one under it, and the operator two feet
    # away is the better sensor (Yifan, 2026-08-17).
    _verdict_file = ROOT / ".human_verdict"
    _verdict_file.unlink(missing_ok=True)
    _watch_stop = threading.Event()

    def _watch_verdict() -> None:
        while not _watch_stop.is_set():
            try:
                v = _verdict_file.read_text().strip().lower()
            except OSError:
                _watch_stop.wait(0.5)
                continue
            _verdict_file.unlink(missing_ok=True)
            if v in ("succeeded", "failed"):
                agent.human_verdict = v
                episode_log.event("human_verdict_received", verdict=v)
                return

    threading.Thread(target=_watch_verdict, daemon=True).start()
    try:
        if recorder:
            recorder.start(instruction)
        summary = agent.run(instruction)
    finally:
        _watch_stop.set()
        _verdict_file.unlink(missing_ok=True)
        # 判定后自动静息 (Yifan, 2026-08-18): an operator verdict ends the
        # episode wherever the arms stand. Open the grippers first (folded
        # cargo is a surprise for the next grasp), then fold both arms to
        # their sleep pose so the next test starts from a clean rig.
        if "operator verdict" in str(locals().get("summary", {}).get("reason", "")):
            for _a in list(getattr(robot, "arms", []) or []):
                try:
                    robot.set_gripper(_a, 0.06)
                    time.sleep(0.6)
                    robot.rest(_a)
                except Exception as _e:
                    # The journal may already be closed here (the episode ended
                    # with the verdict); a tidy-up failure must not crash the run.
                    try:
                        episode_log.event("post_verdict_rest_failed", arm=_a,
                                          error=f"{type(_e).__name__}: {_e}"[:100])
                    except Exception:
                        print(f"[collect] post-verdict rest failed for {_a}: "
                              f"{type(_e).__name__}: {_e}", flush=True)
        if recorder and recorder.recording:
            status = locals().get("summary", {}).get("status", "crashed")
            recorder.stop(instruction=instruction, verifier_status=status,
                          verified_success=status == "succeeded",
                          verifier_reason=str(locals().get("summary", {}).get("reason", ""))[:300],
                          backend=cfg.mode, episode_dir=str(episode_log.dir))
        if writer:
            writer.finalize()
        robot.shutdown()
    speak(f"Task {summary.get('status', 'crashed')}.", say_enabled)
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(ROOT / "configs/trossen_sorting.yaml"))
    ap.add_argument("--episodes", type=int, default=2)
    ap.add_argument("--dataset-root", default=str(ROOT / "datasets/pick_place"))
    ap.add_argument("--repo-id", default="heron/trossen-pick-place")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--cameras", nargs="*", default=None,
                    help="default: every camera the backend has")
    ap.add_argument("--image-size", type=int, nargs=2, default=list(DEFAULT_IMAGE_HW),
                    metavar=("H", "W"))
    ap.add_argument("--png", action="store_true",
                    help="store frames as PNG instead of encoding video")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--say", action="store_true", help="speak progress via macOS say")
    ap.add_argument("--colour", default=None,
                    help="pin every episode to this block colour (e.g. blue). "
                         "Without it the colour is sampled, which spreads a large "
                         "dataset and shatters a small one.")
    ap.add_argument("--same-words", action="store_true",
                    help="use one phrasing for every episode instead of sampling one")
    ap.add_argument("--rearrange-entities", nargs="*", default=None,
                    help="which blocks the scramble moves (default: red and blue). "
                         "A cycle task that only needs the red block must not die "
                         "because something blue-ish is hiding.")
    ap.add_argument("--supports", nargs="*", default=None,
                    help="restrict tasks to these supports (e.g. white_plate) — "
                         "the rig has no grey tray; the sampler must not offer it")
    ap.add_argument("--fixed-supports", action="store_true",
                    help="scramble only the blocks; leave the plate and tray where "
                         "they are. On the rig the fixtures do not wander, and a "
                         "policy asked to learn where the tray went as well as "
                         "where the block is learned neither from 76 episodes.")
    ap.add_argument("--teacher", choices=["agent", "truth"], default="agent",
                    help="who performs the episode. 'agent' plans it; 'truth' runs "
                         "the primitives on the twin's own poses — no planner, no "
                         "model calls for the acting, fixed known competence. The "
                         "verifier still supplies the label either way.")
    ap.add_argument("--reset", choices=["auto", "agent"], default="auto",
                    help="agent: reset with the robot's own arms even in the twin "
                         "(the rig's only option; in sim it measures the full "
                         "self-resetting cycle instead of teleporting)")
    ap.add_argument("--on-pause", choices=["ask", "abort", "continue"], default="ask",
                    help="what the 3-consecutive-failure gate does; 'ask' needs a tty")
    args = ap.parse_args()
    run_collection(args.config, args.episodes, args.dataset_root, args.repo_id,
                   fps=args.fps, cameras=args.cameras,
                   image_hw=tuple(args.image_size), use_videos=not args.png,
                   seed=args.seed, say_enabled=args.say, on_pause=args.on_pause,
                   reset_mode=args.reset, supports=args.supports,
                   rearrange_entities=args.rearrange_entities,
                   colour=args.colour, same_words=args.same_words,
                   teacher=args.teacher, fixed_supports=args.fixed_supports)
    return 0


if __name__ == "__main__":
    sys.exit(main())
