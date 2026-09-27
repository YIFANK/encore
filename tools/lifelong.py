"""Leave the agent in the cell and see whether it gets better at being there.

The skill library can now grow on its own (heron/skills/distill.py), which
raises the only question that matters about it: does a library that has seen a
hundred episodes make the hundred-and-first go better, or is it just a folder
that fills up? A single episode cannot answer that. This runs episodes back to
back in the MuJoCo twin, with the library and the cross-episode memory persisting
between them, and reports the comparison the claim actually rests on — the first
third of the run against the last.

The scene is REARRANGED between episodes and the instruction is drawn from a
generator. Running the same layout repeatedly would measure memorisation of one
arrangement, which is not the claim; the objects have to move or "learned" means
"looked it up".

    python tools/lifelong.py --episodes 40
    python tools/lifelong.py --episodes 40 --resume        # append to the log

Every episode is journalled as usual under episodes/, and one row per episode is
appended to the run log so a long run can be read while it is still going, or
picked up after it is interrupted.
"""
from __future__ import annotations

import argparse
import json
import random
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# Where objects may be put. Inside the arm's measured envelope with a margin —
# a task whose object cannot be reached measures the sampler, not the agent.
X_RANGE = (0.22, 0.40)
Y_RANGE = (-0.18, 0.30)
MIN_SEPARATION_M = 0.10        # so a grasp cannot straddle two objects
BLOCK_Z = 0.001
SUPPORT_Z = -0.006

# Two blocks per colour, so "the red block" names either of two identical
# objects and the agent has to keep hold of WHICH one it moved.
#
# This ran for a while on a one-block-per-colour scene instead
# (assets/trossen_distinct.xml, still there and still the easier control), after
# the first run on this layout could not verify a single episode: the agent
# moved one red block, the verifier re-grounded "the red block" onto the other,
# measured 153 mm from the plate against a true 18 mm, and the
# re-identification guard turned that into UNKNOWN. Only verified episodes
# teach, so the library stayed permanently empty.
#
# That was a real defect and it is fixed rather than avoided: identity
# established by a grasp no longer expires (heron/skills/sensing.py
# `expected_xy`), `place` records where it sent the OBJECT rather than where it
# sent the tool, and `on` looks from the wrist when the arm that made the
# placement is standing in the overhead camera's way. Duplicates are the case
# those changes exist for, so this is the scene that exercises them.
BLOCKS = {"red_block": "red", "red_block_2": "red",
          "blue_block": "blue", "blue_block_2": "blue"}
SUPPORTS = {"white_plate": "the white plate", "grey_tray": "the grey tray"}

# Wordings for the same intent. The planner should not be able to key on one
# phrasing, and a library that only helps for one phrasing has not generalised.
PHRASINGS = (
    "put the {colour} block {prep} {target}",
    "pick up the {colour} block and place it {prep} {target}",
    "move the {colour} block {motion} {target}",
)


# A reset that drops a block back onto the plate hands the next episode a
# pre-solved task — and the sampler COULD do that: the plate at (0.375, 0.135)
# r=0.062 sits inside X_RANGE x Y_RANGE. Keep-out = support half-extent +
# block half-width + margin.
SUPPORT_KEEPOUT_MARGIN_M = 0.025
BLOCK_HALF_M = 0.015


def sample_layout(rng: random.Random, n: int,
                  avoid: list[tuple[float, float, float]] = ()) -> list[tuple[float, float]]:
    """`n` well-separated points on the table, or as many as fit.

    `avoid` is a list of (x, y, radius) keep-out discs — the supports, so a
    randomised block never starts an episode already sitting on its goal.
    """
    out: list[tuple[float, float]] = []
    for _ in range(4000):
        if len(out) == n:
            break
        p = (rng.uniform(*X_RANGE), rng.uniform(*Y_RANGE))
        if any((p[0] - ax) ** 2 + (p[1] - ay) ** 2 <
               (ar + BLOCK_HALF_M + SUPPORT_KEEPOUT_MARGIN_M) ** 2
               for ax, ay, ar in avoid):
            continue
        if all((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 >= MIN_SEPARATION_M ** 2
               for q in out):
            out.append(p)
    return out


def reset_arm(sim) -> None:
    """Send the arm home with an open gripper before a fresh episode.

    The robot is reused across episodes, so without this each one inherits the
    last one's pose AND its grip. Measured: the top failure over the first seven
    episodes was `preconditions ['gripper_empty(right)'] do not hold before pick
    (never established)` — the previous episode had ended mid-grasp, the objects
    were then teleported out from between the fingers, and the new plan's very
    first assumption was false through no fault of its own.
    """
    from heron.skills.primitives import OPEN_WIDTH

    sim.set_gripper("right", OPEN_WIDTH)
    sim.home("right")


def rearrange(sim, rng: random.Random, fixed_supports: bool = False) -> bool:
    """Put every object somewhere new and reachable. False if it could not.

    `fixed_supports` leaves the plate and the tray exactly where they are and
    only moves the blocks. Which task is being posed depends on it: with the
    supports moving too, a demonstration has to teach both "find the block" and
    "find the tray", and 76 of them taught neither — round 0's policies came no
    closer than 96 mm over forty trials. The skill under study is the pick and
    the placement, not the search for a fixture that does not move on the rig.
    """
    reset_arm(sim)
    sup_names = list(SUPPORTS)
    move_supports = True
    if fixed_supports:
        # A NAMED PLACE, NOT WHEREVER THE XML LEFT THEM AND NOT WHEREVER THE RNG
        # LANDS. Two reasons. The scene file puts the tray at (0.36, 0.28),
        # which has no IK solution at z=0.10, so honouring its position failed
        # this function's own reachability check on every episode forever — 120
        # reseeds and not one demonstration. And sampling a fixed spot per run
        # would put it somewhere else during collection than during evaluation,
        # which is the train/test mismatch this flag exists to remove.
        # Measured to have a solution at 0.03 through 0.16 m, 0.30 m apart.
        sup_spots = [FIXED_SUPPORT_XY[n] for n in sup_names]
    else:
        sup_spots = sample_layout(rng, len(sup_names))
    if len(sup_spots) < len(sup_names):
        return False
    avoid = [(x, y, SUPPORT_HALF_M.get(n2, 0.062)) for n2, (x, y) in zip(sup_names, sup_spots)]
    blk_names = list(BLOCKS)
    blk_spots = sample_layout(rng, len(blk_names), avoid=avoid)
    if len(blk_spots) < len(blk_names):
        return False
    names = sup_names + blk_names
    spots = sup_spots + blk_spots
    # Check reachability at BOTH the grasp height and a hover, and check it
    # before moving anything. `reachable` is multi-seeded and stochastic, so a
    # single query at one height can pass and the same point fail later — which
    # is how objects ended up where `approach_pose` then reported "nothing above
    # (x, y) is reachable between z=-0.007 and z=0.180". Placing objects only
    # after every one of them has cleared also stops a rejected layout leaving
    # the scene half rearranged.
    for _, (x, y) in zip(names, spots):
        if not all(sim.reachable("right", np.array([x, y, z]))
                   for z in (0.03, 0.06, 0.10)):
            return False
    for name, (x, y) in zip(names, spots):
        if name in SUPPORTS and not move_supports:
            continue          # already where it belongs; moving it re-drops it
        sim.place_body(name, (x, y, SUPPORT_Z if name in SUPPORTS else BLOCK_Z))
    return True


def make_task(rng: random.Random,
              supports: list[str] | None = None) -> tuple[str, str, str]:
    """(instruction, colour, support) for one episode.

    The task names a COLOUR, not a body. There are two blocks of each colour and
    nothing distinguishes them, so "the blue block" means any of them — which is
    what the planner is told ("an X / any X = exactly one, choose the easiest").
    Scoring against one particular body would fail the agent for obeying the
    instruction, so the colour is what travels to `landed`.
    """
    colour = rng.choice(sorted(set(BLOCKS.values())))
    support = rng.choice(list(supports) if supports else list(SUPPORTS))
    into = support == "grey_tray"
    text = rng.choice(PHRASINGS).format(
        colour=colour, prep="in" if into else "on",
        motion="into" if into else "onto", target=SUPPORTS[support])
    return text, colour, support


# Half-extents of the supports, from assets/trossen_sorting.xml. Needed to say
# whether an object is physically on one, which is a different question from
# whether it is centred on one.
SUPPORT_HALF_M = {"white_plate": 0.062, "grey_tray": 0.055}
# Where the supports stand when they are not being scrambled. Chosen by sweeping
# our own IK: both have a solution at every height from 0.03 to 0.16 m, which is
# more than the grasp and the hover need, and they are 0.30 m apart.
FIXED_SUPPORT_XY = {"white_plate": (0.30, 0.18), "grey_tray": (0.30, -0.12)}
BLOCK_HALF_M = 0.016


def landed(sim, colour: str, support: str) -> tuple[bool, bool, float]:
    """Ground truth: (on the support, centred on it, best distance in metres).

    TWO standards, reported separately, because they disagree and conflating
    them makes the whole run unreadable. The first version of this used a single
    75 mm tolerance and reported "landed: yes" for episodes the agent called
    failures — the agent was right: it applies `verify.ON_XY_MAX`, 30 mm, which
    is LIBERO's own On() tolerance and deliberately no looser than the benchmark
    that grades it. A block 52 mm from a plate's centre is resting on the plate
    and is not on it in the sense anything downstream means.

    Any block of the named colour counts, for the same reason `make_task` names
    a colour: the instruction did not single one out.
    """
    from heron.verify import ON_XY_MAX

    b = sim.body_xyz(support)
    room = SUPPORT_HALF_M.get(support, 0.06) - BLOCK_HALF_M
    best = float("inf")
    on_support = centred = False
    for name, c in BLOCKS.items():
        if c != colour:
            continue
        a = sim.body_xyz(name)
        d = float(np.linalg.norm(a[:2] - b[:2]))
        if a[2] <= b[2]:
            continue                       # under it, or knocked off the table
        best = min(best, d)
        on_support = on_support or d <= room
        centred = centred or d <= ON_XY_MAX
    return on_support, centred, best


def summarise(rows: list[dict]) -> str:
    """First third against last third — the only comparison that tests the claim."""
    if len(rows) < 6:
        return "too few episodes to compare"
    k = max(2, len(rows) // 3)
    early, late = rows[:k], rows[-k:]

    def stat(group, key):
        vals = [r[key] for r in group if r.get(key) is not None]
        return sum(vals) / len(vals) if vals else float("nan")

    lines = [f"\nfirst {k} episodes vs last {k}:",
             "%-26s %8s %8s" % ("", "first", "last")]
    for label, key, scale in (("centred on target (%)", "centred", 100),
                              ("physically on it (%)", "on_support", 100),
                              ("agent said success (%)", "succeeded", 100),
                              ("used a learned skill (%)", "used_skill", 100),
                              ("repairs per episode", "edits", 1),
                              ("steps per episode", "steps", 1),
                              ("seconds per episode", "wall_s", 1)):
        lines.append("%-26s %8.1f %8.1f" % (label, stat(early, key) * scale,
                                            stat(late, key) * scale))
    lines.append(f"\nlibrary: {rows[0]['library_size']} skills at the start, "
                 f"{rows[-1]['library_size']} at the end")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(ROOT / "configs/trossen_sorting.yaml"),
                    help="configs/trossen_lifelong.yaml is the same cell with one "
                         "block per colour — the control, where nothing has to be "
                         "told apart from its twin")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--orchestrator", default="gemini", choices=["gemini", "scripted"])
    ap.add_argument("--log", default=str(ROOT / "reports" / "lifelong.jsonl"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--resume", action="store_true",
                    help="append to an existing log and keep its episode numbering")
    args = ap.parse_args()

    from heron.agent import Agent
    from heron.config import HeronConfig
    from heron.episode import EpisodeLogger
    from heron.orchestrator.gemini import GeminiOrchestrator
    from heron.robot.safety import SafeRobot
    from heron.robot.trossen_sim import TrossenSim

    cfg = HeronConfig.load(args.config)
    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []
    if args.resume and log_path.exists():
        rows = [json.loads(line) for line in log_path.read_text().splitlines() if line]
        print(f"resuming after {len(rows)} episodes")
    elif log_path.exists():
        log_path.unlink()

    rng = random.Random(args.seed + len(rows))
    # Record a film. The rollout player puts it on one timeline with the
    # journal's tool calls, and an episode you cannot watch is one whose failure
    # has to be inferred from JSON — which is how most of today went.
    sim = TrossenSim(cfg, scene=cfg.sim_scene,
                     record_camera=cfg.sim_record_camera or "cam_high",
                     record_every=cfg.sim_record_every)
    robot = SafeRobot(sim, cfg.safety)

    print(f"{'ep':>4} {'instruction':<44} {'status':<10} {'landed':>6} "
          f"{'steps':>5} {'edits':>5} {'skills':>6} {'s':>6}")
    try:
        for i in range(len(rows), len(rows) + args.episodes):
            if not rearrange(sim, rng):
                print(f"{i:>4} could not lay out a reachable scene; reseeding")
                rng = random.Random(rng.random())
                continue
            instruction, colour, support = make_task(rng)

            episode_log = EpisodeLogger(cfg.episodes_dir, "lifelong")
            orch = (GeminiOrchestrator(cfg, episode_log) if args.orchestrator == "gemini"
                    else None)
            if orch is None:
                raise SystemExit("the twin has no scripted orchestrator; use --orchestrator gemini")
            # One film per episode, on the journal's own clock. The sim is
            # reused across episodes, so its frames would otherwise accumulate
            # into one long strip; and a recorder started at a different moment
            # than the logger would put every tick in the wrong place.
            sim.frames.clear()
            sim.frame_times.clear()
            sim._recording_t0 = episode_log.t0
            agent = Agent(robot, orch, cfg, log=episode_log)
            before = len(agent.skills)
            t0 = time.time()
            try:
                summary = agent.run(instruction)
            except Exception as e:      # one bad episode must not end the run
                summary = {"status": "crashed", "reason": f"{type(e).__name__}: {e}",
                           "steps_executed": None, "edits_applied": None}

            on_support, centred, best = landed(sim, colour, support)
            try:
                sim.save_video(episode_log.dir / "run.gif")
            except Exception as e:
                print("   (film not written: %s)" % e)
            journal = episode_log.dir / "journal.jsonl"
            events = [json.loads(x) for x in journal.read_text().splitlines()] \
                if journal.exists() else []
            row = {
                "episode": i,
                "instruction": instruction,
                "colour": colour, "support": support,
                "status": summary.get("status"),
                "succeeded": summary.get("status") == "succeeded",
                "on_support": on_support,
                "centred": centred,
                "dxy_mm": None if best == float("inf") else round(best * 1000),
                "steps": summary.get("steps_executed"),
                "edits": summary.get("edits_applied"),
                "wall_s": round(time.time() - t0, 1),
                "used_skill": any(e.get("kind") == "step_start" and e.get("origin_skill")
                                  for e in events),
                "learned": [e["skill"] for e in events
                            if e.get("kind") == "skill_learned_from_episode"],
                "rediscovered": [e["skill"] for e in events
                                 if e.get("kind") == "skill_rediscovered"],
                "library_size": len(agent.skills),
                "library_grew": len(agent.skills) - before,
                "episode_dir": str(episode_log.dir),
            }
            rows.append(row)
            with log_path.open("a") as fh:
                fh.write(json.dumps(row) + "\n")
            print(f"{i:>4} {instruction[:44]:<44} {str(row['status']):<10} "
                  f"{('centred' if row['centred'] else 'on' if row['on_support'] else 'no'):>7} "
                  f"{str(row['steps']):>5} {str(row['edits']):>5} "
                  f"{row['library_size']:>6} {row['wall_s']:>6.0f}", flush=True)
    except KeyboardInterrupt:
        print("\ninterrupted; the log is complete up to the last row")
    finally:
        print(summarise(rows))
        sim.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
