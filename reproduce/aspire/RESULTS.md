# RESULTS — aspire_demos_ab (ASPIRE vs ASPIRE+demos)

Suite: `libero_goal_swap`. Development seeds 51–65, held-out seeds 1–50,
strictly separated. Held-out numbers come from ASPIRE's own
`run_fix_loop_validation.py` manifests (`evidence_scope: heldout_full`), run
once per task per arm by a coordinator daemon that is the only process in the
setup permitted to touch seeds 1–50.

**Status: COMPLETE — all six cells have held-out numbers.**
Run date 2026-08-21, AbakaAI, git commit `680bad4`.

## Arms

| | Arm A | Arm B |
|---|---|---|
| Name | ASPIRE | ASPIRE+demos |
| Delta | — | K=5 LIBERO human teleop demos of the base task, plus one addendum file naming them |
| Repo checkout | `/mnt/data/YifanKang/ASPIRE` | `/mnt/data/YifanKang/ASPIRE_armB` |
| Prompt difference | \- | exactly two diff hunks: scratch-dir path, demo addendum |
| Treatment actually consumed | n/a | **3/3 agents opened the hdf5 and cited it** |

## Model / provider pin (fairness)

| Role | Arm A | Arm B |
|---|---|---|
| Inner code-gen model (ASPIRE exploration) | `aws_anthropic_bedrock-claude-sonnet-4-6` | `aws_anthropic_bedrock-claude-sonnet-4-6` |
| Outer Stage 1 debugging agent | `claude-opus-5[1m]`, first-party | `claude-opus-5[1m]`, first-party |

Same model both arms, as required for a ±demos comparison. Opus-4.6 paper parity
was not available through this box's Bedrock routing; this is the footnoted
"same-sonnet both arms" option.

## Held-out results (seeds 1–50)

| Task | Arm A (ASPIRE) | Arm B (ASPIRE+demos) |
|---|---|---|
| turn_on_the_stove | **50/50** (1.00) | **50/50** (1.00) |
| put_the_bowl_on_the_plate | **50/50** (1.00) | **50/50** (1.00) |
| open_the_middle_drawer_of_the_cabinet | **31/50** (0.62) | **48/50** (0.96) |

Totals over the three tasks: **Arm A 131/150 (0.873)**, **Arm B 148/150 (0.987)**.

Run identities (hash of fix code + config + seed set):

| Cell | run_id | code_sha256 (first 12) |
|---|---|---|
| A/stove | `9bffbdcbf4c3` | `89b7dda81c4d` |
| A/bowl | `3ec626e7f3be` | `5c111b25e143` |
| A/drawer (superseded) | `3f4613b19b4c` | `12a572b0258d` |
| A/drawer (delivered) | `b2a1ebcfa295` | `2714dba3e6bf` |
| B/stove | `409527d0f05e` | `ca11feb343ec` |
| B/bowl | `fc498e948b15` | `bc859134861b` |
| B/drawer | `8032fab97ef8` | `27971a08248d` |

### One corrected cell

The coordinator daemon's stage1-done test was "`fix_code.py` and `findings.md`
both present". For A/drawer that fired early: Stage 2 launched 05:35 local while
the agent kept working until 05:52, rewriting `fix_code.py` at 05:45. The
resulting 31/50 therefore measured a **superseded** program, while all five other
cells (verified by comparing each manifest's `identity.code_sha256` against the
delivered file) measured the program their agent actually delivered.

That asymmetry lands in the single cell where the two arms diverge, so the cell
was re-run against the delivered `fix_code.py` — same script, same flags, same 50
seeds, separate immutable run directory (the original manifest is untouched).

**The correction changed nothing.** Both runs score **31/50**:

| run | code | passes | seeds passed only here |
|---|---|---|---|
| `3f4613b19b4c` (superseded code) | `12a572b0258d` | 31/50 | 2, 5, 30 |
| `b2a1ebcfa295` (delivered code) | `2714dba3e6bf` | 31/50 | 48, 49, 50 |

28 seeds pass in both. The two programs are equivalent in aggregate on the
held-out set, so the headline 31/50 is safe either way — but the number now
comes from the artifact Stage 1 actually delivered. Full account in `NOTES.md`,
session 8.

## Development traces (seeds 51–65)

Stage 1 never touched seeds 1–50; these are the dev-sweep tallies its agents
actually measured.

| Task | Arm A initial program | Arm A delivered | Arm B initial program | Arm B delivered |
|---|---|---|---|---|
| turn_on_the_stove | 15/15 | 15/15 | 15/15 | 15/15 |
| put_the_bowl_on_the_plate | 14/15 (fail 64) | 15/15 | 11/15 (fail 58, 62, 64, 65) | 15/15 |
| open_the_middle_drawer_of_the_cabinet | 12/15 (fail 51, 57, 64) | partial re-validation: 51 ✓, 52 ✓, 53 ✗ | 15/15 | 15/15 |

Two caveats, both from the agents' own write-ups rather than inferred:

- **A/drawer** left seeds 57 and 64 as `BLOCKED` (episode-horizon exhaustion) at
  the 3-replay limit, and its delivered program was only partially re-validated
  on the dev seeds because the early Stage 2 launch was contending with it for
  the GPU and the shared perception servers.
- **A/bowl** returned after 21 minutes with a `findings.md` still labelled
  "Stage 1 in progress" and a "Blocked seeds: *(pending sweep completion)*"
  section. Its `fix_code.py` was final before Stage 2 started (sha verified) and
  scored 50/50, so the number is sound; the write-up is not complete.

## Stage 1 cost (the secondary signal)

Wallclock and agent-event counts from the supervisor logs. Both arms ran
concurrently on the same GPUs under the same load, so these are comparable.

| Task | Arm A | Arm B |
|---|---|---|
| turn_on_the_stove | 79 min / 1152 events | 43 min / 584 events |
| put_the_bowl_on_the_plate | 21 min / 349 events | 36 min / 663 events |
| open_the_middle_drawer_of_the_cabinet | 104 min / 1650 events | 21 min / 529 events |
| **total** | **204 min / 3151 events** | **100 min / 1776 events** |

## Reading of the result

On two of three tasks both arms saturate at 50/50, so the ±demos row measures
nothing there — the tasks are too easy for this pipeline to separate the arms.
The drawer task is the only one that discriminates, and there Arm B is far ahead:
**48/50 vs 31/50**. But that is a single task, one run per cell, so the accuracy
claim rests on one cell and must be quoted as such — the 0.873 vs 0.987 totals
are that one cell diluted by two ceilings, not three independent measurements.

The more legible difference is **cost, not accuracy**: Arm B reached a delivered
program in about half the wallclock and about half the agent turns of Arm A
(above), with the largest gap on the hardest task. The demos appear to have been
used as a measurement shortcut — B/drawer read the pull axis and stroke length
straight off the teleop end-effector traces ("y ≈ −0.146 at constant z, then
~0.17–0.22 m in +y") rather than deriving them from probes, and finished Stage 1
in 21 minutes against Arm A's 104.

Caveats worth stating with the numbers: n=3 tasks, one run per cell, no repeats
for the agents themselves, ceiling effects on two of three tasks, and the
inner-model pin is sonnet-4-6 rather than the paper's opus-4.6.

## Reproduction

- `python3 collect_results.py` — reads the manifests off the box, writes
  `/mnt/data/YifanKang/aspire_ab_results/ab_heldout.json`, prints the table.
- Per-arm Stage 1 artifacts:
  `<checkout>/aspire/sim/outputs/libero_fix_loop/libero_goal_swap/<task>/`
  (`fix_code.py`, `findings.md`, `task_analysis.md`, `initial_seed_*.log`).
- Held-out manifests: `.../outputs/libero_fix_loop_eval/libero_goal_swap/<task>/runs/<run_id>/`.

## Deviations

All logged in `NOTES.md` (sessions 4–8). The load-bearing ones: 3 task GPUs
instead of 5; two agents per task GPU (same task, opposite arms); Stage 1 agents
run on the laptop over ssh rather than on the compute host; supervised
`--resume` after transient API death (not needed this session); services started
by hand to keep logs off a full root disk; and the corrected A/drawer eval above.

---

# v2 — Arm B (+demos) x ALL TEN `libero_goal_swap` tasks, opus inner

Run date **2026-08-21** (09:34 → 14:21 local, 4 h 47 m wallclock), AbakaAI,
commit `680bad4`, Arm-B checkout `/mnt/data/YifanKang/ASPIRE_armB`.
Inner Stage 1 agent pinned to **`claude-opus-5`** (`--model`, explicit, not
inherited). Scope per TASK.md's lean revision: **Arm B only** — the
plain-ASPIRE column cites their published Table 8, and v1's three Arm-A cells
below stay as the reproduction sanity row.

**Status: COMPLETE — all ten cells have held-out numbers at
`evidence_scope: heldout_full`, 50 trials each.**

## Held-out results (seeds 1–50, one run per cell)

| Task | Held-out | Rate | Dev sweep (51–65) |
|---|---|---|---|
| put_the_bowl_on_top_of_the_cabinet | **50/50** | 1.00 | 15/15 |
| turn_on_the_stove | **50/50** | 1.00 | 15/15 (initial 0/15) |
| put_the_cream_cheese_in_the_bowl | **49/50** | 0.98 | incomplete at delivery |
| put_the_wine_bottle_on_the_rack | **49/50** | 0.98 | 15/15 |
| put_the_bowl_on_the_plate | **48/50** | 0.96 | 15/15 |
| put_the_wine_bottle_on_top_of_the_cabinet | **45/50** | 0.90 | 15/15 (initial 12/15) |
| open_the_middle_drawer_of_the_cabinet | **43/50** | 0.86 | 15/15 (initial 1/15) |
| push_the_plate_to_the_front_of_the_stove | **20/50** | 0.40 | 7/15 (initial 0/15) |
| open_the_top_drawer_and_put_the_bowl_inside | **17/50** | 0.34 | incomplete at delivery |
| put_the_bowl_on_the_stove | **16/50** | 0.32 | 8/15 |
| **total** | **387/500** | **0.774** | |

Every cell: exactly one 50-trial run on disk, and its manifest
`identity.code_sha256` matches the `fix_code.py` Stage 1 delivered — verified
twice, by the coordinator at completion and again by `collect10.py` afterwards.
Machine-readable: `armB_heldout_v2.json`.

## Cost per cell (Stage 1, the dominant cost)

| Task | Stage 1 wallclock | Agent events | Attempts | API deaths |
|---|---|---|---|---|
| put_the_bowl_on_the_plate | 32 min | 838 | 1 | 0 |
| put_the_wine_bottle_on_top_of_the_cabinet | 44 min | 988 | 1 | 0 |
| put_the_bowl_on_top_of_the_cabinet | 44 min | 1280 | 4 | 2 |
| put_the_cream_cheese_in_the_bowl | 44 min | 1920 | 3 | 0 |
| open_the_top_drawer_and_put_the_bowl_inside | 61 min | 2182 | 4 | 2 |
| put_the_wine_bottle_on_the_rack | 65 min | 2454 | 1 | 0 |
| open_the_middle_drawer_of_the_cabinet | 68 min | 2180 | 1 | 0 |
| turn_on_the_stove | 80 min | 2524 | 1 | 0 |
| put_the_bowl_on_the_stove | 97 min | 3870 | 1 | 0 |
| push_the_plate_to_the_front_of_the_stove | 147 min | 3512 | 3 | 2 |
| **total** | **682 agent-min** (4 h 47 m wallclock over 3 GPUs) | 21748 | | 6 |

## Treatment actually consumed — 10/10

Every Arm-B agent opened the K=5 hdf5 and cited measurements from it; verified
both in the local agent logs (`aspire_demos` / `h5py` / `K5_demos` references,
6–22 per cell) and in the box-side `findings.md` / `task_analysis.md`. A nominal
treatment would make the whole column meaningless, so this is checked, not
assumed.

## What these numbers do and do not support

- This is a **one-arm column**, not an A/B. Nothing here measures the ±demos
  delta; that comparison exists only in v1's three cells below, and there it
  rests on a single task (drawer, 48/50 vs 31/50). Quoting 0.774 against
  ASPIRE's published Table 8 compares across *different inner models* (opus-5
  here vs their reported setup) and different hardware — it is a reference
  point, not a controlled contrast.
- The spread is the story: five cells at 0.96–1.00 and three at 0.32–0.40. The
  three low cells are the ones whose dev sweeps also came in low or incomplete
  (7/15, 8/15, unfinished), so dev score predicted held-out score — the fix loop
  knew it had not converged, and TASK.md's "freeze and evaluate anyway" rule is
  what put those numbers on the board.
- **Two cells delivered on an incomplete dev sweep**
  (`put_the_cream_cheese_in_the_bowl`, `open_the_top_drawer_and_put_the_bowl_inside`).
  Both had returned cleanly without artifacts and were nudged by the supervisor
  to write Steps 4–5. The nudge does not tell an agent to stop debugging, but it
  does push toward delivering, and `open_the_top_drawer` describes its own sweep
  as "still in flight when Stage 1 was cut short" — it then scored 17/50. Treat
  that cell as under-converged rather than as a clean measurement of the method.
- Single run per cell, no repeat seeds, n=1 agent per task. The three GPUs
  carried concurrent Stage 1 and Stage 2 work, so wallclock figures include
  contention.

---

## Independent audit (session 12, 2026-08-21)

Every number above was originally read by one collector (`collect10.py`, which
reads `manifest.json`'s `passes`/`trials`). A single reader that is wrong looks
exactly like a correct one, so all ten v2 cells and all seven archived v1 runs
were re-read by a deliberately different probe (`verify12.py`, `verify_v1.py`):
it counts the **per-trial result directories**, whose names encode each outcome
(`trial_NN_..._taskcompleted_{0,1}`), and cross-checks the manifest's per-seed
`results` map.

| Check | Result |
|---|---|
| trial-dir recount == manifest, per cell | 10/10 exact |
| per-seed recount == manifest, per cell | 10/10 exact |
| v2 total | **387/500** by all three readers |
| **executed** seed set per cell (per-trial `seed`) | exactly 1..50, 10/10 |
| dev seeds (>50) inside any held-out run | none |
| manifest `code_sha256` == delivered `fix_code.py` | 10/10 |
| run dirs per task (no superseded run hiding) | exactly 1, 10/10 |
| v1 archive (3 tasks x 2 arms + A/drawer re-run) | all 7 reproduce the v1 table |

The executed-seed check had never been run before: every earlier check trusted
`identity.seeds`, i.e. what each run *declared* it would sweep. Reading the seed
off each of the 500 recorded trials proves what was actually swept. No number
changed. This establishes that the numbers are what the runs produced — a
separate question from whether the three low cells were under-converged, which
they were, as stated above.

---

# v3 — REPAIR PASS (2026-08-21)

Scope, from TASK.md's repair block: rerun two suspect cells properly, write up
`push_the_plate_to_the_front_of_the_stove` as a documented mechanism gap and
compare it against ASPIRE's own approach. **Nothing else in the v2 table is
rerun.** Protocol unchanged: opus inner, dev 51–65, one held-out 1–50 per cell.

## A. `push_the_plate_to_the_front_of_the_stove` — 20/50, and what ASPIRE did

### A.0 The comparison in TASK.md is against the wrong table — and it inverts

TASK.md motivates this section with "ASPIRE reports 0.76 on this task in their
Table 8". That is true, and it is **not our suite**. In the paper
(arXiv:2607.00272):

- **Table 7** is *Position perturbation (Swap)* — suite `libero_goal_swap`,
  which is what this whole campaign runs.
- **Table 8** is *Task perturbation (Goal)* — suite `libero_goal_task`, a
  different perturbation axis (instruction paraphrase, not object-pose swap).

`Push plate to stove` appears in both. Their numbers:

| | w/o Engine & Evo | w/o Evo (fix loop) | Evo. search | Aspire (final) |
|---|---|---|---|---|
| **Table 7 — goal_swap (our suite)** | 0.00 | **0.00** | **0.00** | **0.00** |
| Table 8 — goal_task (not our suite) | 0.26 | 0.58 | 0.76 | 0.76 |

So on the suite we actually ran, **ASPIRE reports 0.00 on this task in every one
of their four configurations, and our Arm-B cell scores 0.40 (20/50)**. The
premise of "their agent found something ours never did" is exactly backwards
here: this is the one goal_swap cell where their published pipeline scores zero
and ours does not. Their 0.76 belongs to a suite whose plate starts in a fixed,
non-swapped pose.

That reading also matches the banked knowledge in their repo, which is
suite-specific (§A.2): every plate-push entry is labelled `goal_task`.

### A.1 Our mechanism gap, as our own agent measured it

The diagnosis in the cell's `findings.md` is solid and is quoted here as the
documented gap rather than rerun. Three measurements do the work:

1. **`solve_ik` targets a frame 0.111 m below what `robot_cartesian_pos`
   reports.** Uncorrected, the fingertips sweep ~10 cm above the plate and move
   it 0.0000 m. `solve_ik` returning non-`None` does not mean the pose was
   achieved — it returns joints whose achieved z is 0.1 m off.
2. **Contact height has almost no margin.** Table top is world z = −0.011, the
   plate rim tops out at ~0.006. Commanding z = 0.012 rides *over* the rim
   (0.24 m of stroke → 0.027 m of plate travel, pushed sideways); commanding
   z = 0.004 moves it 0.118 m cleanly.
3. **The unreachable thing is the approach point, not the goal.** The pusher
   must stand *behind* the plate. Measured failures to fold down to table height
   at (0.399, −0.165), (0.448, −0.176), (0.418, −0.124), (0.569, −0.140),
   (0.621, −0.128) — at every yaw tried — against successes at (0.62, 0.30),
   (0.70, 0.12), (0.346, −0.064). The plate starts near y = −0.06 and the goal
   is at y = +0.21, so the straight push line always needs the arm inside that
   band. The agent's workaround (sweep all 72 push directions, keep only those
   whose approach point clears y ≥ −0.095, bootstrap with a +x push first) walks
   the plate there in six segments on the dev seeds it solved, and stalls ~0.22 m
   short on the ones it did not: 7/15 dev, 20/50 held-out, after 12 versions and
   147 minutes — the most expensive cell in the campaign.

Grasping was considered and rejected on evidence: `plan_grasp` on the plate mask
returned 3 candidates, best score 0.208, selected pose 14 mm *below* the table.

### A.2 What ASPIRE's own repo has for this task that our agent never had

Their released tree contains three artifacts that bear directly on the wall our
agent hit. None of them were available to our agent, and that is protocol-correct
rather than a mistake: the fix loop starts its skill library from zero, and the
seed library our agents read (`.claude/libero/skills/`, 443 lines, hash verified
identical in both checkouts at dispatch) has an *empty placeholder* where push
knowledge would go — `manipulation.md` literally says "Add entries for pushing
objects without grasping". Our agent had to discover all of this from scratch.

**(a) `.claude/libero/evosearch/skills/wrist-rotation-blocking.md` — directly
contradicts our agent's terminal diagnosis.** It describes our exact symptom:

> An approach position passes IK (`solve_ik` returns joints, `move_to_joints`
> reports completed), but the arm physically stops well above the target Z —
> typically 0.15–0.30 m above.

and attributes it not to a kinematic reach limit but to **arm-body collision**
(elbow/forearm/wrist base), which IK cannot see. The prescribed fix is one line
— `j[6] += np.pi/2` after `solve_ik`, which changes the arm's spatial footprint
without moving the EEF — and the file's own rule is "Before declaring an approach
'impossible' due to blocking, test it once with `j[6] += π/2`." Its validated
context is *"Franka Panda … reduces arm profile in the lateral (Y) axis during
descent. Confirmed effective for approaches near shelves/cabinets at Y offsets of
0.10–0.13 m"* — the exact band (y = −0.124 … −0.176) our agent measured as a
wall and wrote up as a reach limit. Our agent did search wrist *yaw* (two values,
−116° and −150°, passed to `topdown_quat`), which is a different thing: yaw
changes the commanded EEF orientation, `j[6] +=` changes the arm configuration
behind a fixed EEF pose. This is the single most likely difference between "12
versions, bootstrap diagonals stall" and a clean straight push.

**(b) `.claude/libero/evosearch/skills/push-contact-tasks.md` — a different
approach geometry.** Two of its rules cut against what our agent built:

- *Descend vertically onto the back rim, do not sweep in laterally.* Their
  standoff is `plate_center − push_dir * plate_radius` (the rim itself),
  approached from directly above. Ours is `plate_center − u * (radius + 0.035)`
  — 35 mm further into the dead zone, and the extra 35 mm is exactly the kind of
  margin that decides whether an approach point clears y = −0.095.
- *Arm target ≠ object final position*: `arm_target = desired_final − push_dir *
  plate_radius`, or the plate overshoots by a radius.

**(c) `aspire/real/.agents/skills/yam-simulation-transfer/references/plate-libero-transfer.md`
— their banked solution is a different primitive entirely.** For
`libero_goal_task/push_the_plate_to_the_front_of_the_stove` it records **31/50
(62%)** with the key pattern *"arm-body nudge, lift with grasp quat"*: they
**grasp and carry** the plate, using an *arm-body* nudge (descend at mid-height
so the forearm, not the fingertips, shoves the plate) only for seeds where the
grasp width is below 0.04. It also names the trap that would have bitten a naive
grasp attempt — reorienting to a top-down quat immediately after `close_gripper`
drops thin flat objects; lift 5 cm in the original grasp quat first.

Our agent never got there: it read `plan_grasp`'s best score of 0.208 with a pose
below the table as "a grasp is not available" and committed to pure pushing. On
`goal_swap` that judgement may well be right — their own Table 7 says their
pipeline scores 0.00 there — but it was made from one probe, not from a tried
alternative.

### A.3 Honest summary

Ours found the reach wall and worked around it to 0.40; theirs, on this suite,
reports 0.00 in all four configurations. The techniques in their repo that our
agent lacked are real and specific (`j[6] += π/2` for arm-body blocking; rim-not-
standoff vertical descent; grasp-and-carry with an arm-body nudge fallback), but
they are banked against the `goal_task` variant, and the 0.76 they are attached
to is a different table for a different perturbation axis. The paper-grade
observation is therefore the reverse of the one TASK.md anticipated: **on
`goal_swap`'s plate push, the published ASPIRE pipeline finds nothing and our
+demos fix-loop cell finds a partial solution — and the technique that would most
plausibly close the remaining 0.60 is sitting unused in their own evolutionary-
search skill directory, which the fix-loop agent does not read.**

### A.4 The "does not read it" claim, verified from the prompt (session 15)

§A.2/A.3 rest on the assertion that a fix-loop agent never sees
`.claude/libero/evosearch/skills/`. That was re-derived independently this
session and it holds — by prompt text rather than by inference:

- ASPIRE's own `.claude/libero/fix-loop/subagent-prompt.md` line 80 — the file
  our prompts are generated from verbatim — says: *"Follow
  `.claude/libero/fix-loop/skills/task-exploration.md`, then read the relevant
  shared skill library in `.claude/libero/skills/`."* It names exactly two
  directories, and `evosearch/skills/` is neither.
- `.claude/libero/skills/` is 443 lines across five files, and
  `manipulation.md` is 32 of them — of which the entire "Pushing / Sliding"
  section is the stub *"Add entries for pushing objects without grasping:
  contact approach, force direction, step size, distance control."* No content.
  That is the push knowledge a fix-loop agent starts with.
- By contrast `evosearch/skills/push-contact-tasks.md` tells *its* agent (§4
  step 2) to run `find .claude/libero -name "*.md" | sort` and read every match
  — an instruction that would have surfaced `wrist-rotation-blocking.md`. The
  fix-loop prompt has no equivalent sweep.
- The third artifact, the plate reference carrying the 31/50 arm-body-nudge
  entry, is under `aspire/real/.agents/skills/` — a different top-level tree
  (`aspire/real`, the YAM hardware stack), not under `aspire/sim` at all.

So the gap is structural rather than an oversight by our agent: three ASPIRE
directories hold plate-push knowledge, and the fix-loop agent is pointed at the
one whose push section is an empty placeholder. Nothing was withheld on our
side — the skill-library hash `75b397c6…c449` with zero promotions is the
protocol-correct cold start, and it is the same start Arm A gets in v3.

### A.5 CORRECTION (2026-08-21, from the Arm A run) — the technique was found, and it did not win

§A.2/A.3 argued from ASPIRE's repo that Arm B's `push_the_plate` cell had
misdiagnosed its wall — calling "cannot reach table height below y ≈ −0.09" a
*kinematic reach limit* when their own `evosearch/skills/wrist-rotation-
blocking.md` describes the identical symptom as **arm-body collision**, fixable
with `j[6] += π/2`. §A.3 called that "the single most likely difference between
'12 versions, bootstrap diagonals stall' and a clean straight push."

The Arm A run tests that prediction, and splits it.

**The mechanism half is confirmed, by experiment rather than by reading.** Arm A's
agent on this task — no demos, no access to that skill file — found it from
scratch and proved it with a controlled probe:

> Probing an 8-point grid around the plate (x 0.36–0.58, y −0.06 → −0.16) at
> z = 0.01: **8/8 BLOCKED** with the raw IK solution, **5/8 OK** after
> `j[6] += π/2`. The blocker is the arm *body* against a 23–30 cm wooden knife
> block that stands immediately to the plate's left (y ∈ [−0.20, −0.15] across
> x ∈ [0.25, 0.50]); the fingertip itself has clearance.

Same eight points, same height, one variable changed, 0/8 → 5/8. The wall is an
obstacle, and the obstacle has a name. Arm B's findings never mention it.

**The causal half is refuted.** The agent that *had* the technique scored
**13/50 (0.26)**; the agent that got the physics wrong scored **20/50 (0.40)** —
and Arm A held the `CONVERGE=1` advantage, so the comparison runs against it.

| | dev 51–65 | held-out | mechanism committed to |
|---|---|---|---|
| Arm A (plain) | 4/15 | **13/50 · 0.26** | knife block; `j[6] += π/2` |
| Arm B (+demos) | 7/15 | **20/50 · 0.40** | reach wall; 72-direction search filtered by a measured band |
| ASPIRE published, `goal_swap` | – | **0.00** (all four configs) | – |
| ENCORE (c2) | – | **50/50 · 1.00** | different harness |

On this task a correct causal model was worth less than a robust search. Arm B's
band filter is *right for the wrong reason* — "cannot fold down below y ≈ −0.09"
is a description of where the knife block sits — and filtering candidate approach
points against an empirically measured band works whether or not you know why the
band exists. Arm A spent its budget establishing the mechanism (185 min, 2062
events, the most expensive cell of the campaign) and had less left for the search
that actually converts contacts into completed pushes.

**So §A.3 should not be read as "hand the fix-loop agent that skill file and the
cell is solved."** The skill-library gap it identifies is real and structural
(§A.4), and the mechanism it names is correct — but it is not sufficient, and on
the evidence here it is not even the binding constraint. The binding constraint
is that both arms are searching for a push strategy at all, in a harness where
ENCORE gets 50/50.

## C. The two repaired cells — results (2026-08-21)

Both cells were rerun from a genuine clean slate (box-side outputs archived and
removed, fresh session ids — not `--resume` of the contexts that contained the
cut-short delivery), under `CONVERGE=1`, model `claude-opus-5`, dev 51–65 to
convergence, then ONE held-out 1–50. Nothing else in the v2 table was touched.

| Cell (Arm B, +demos) | Stage 1 | dev 51–65 | v2 | **repaired** |
|---|---|---|---|---|
| `open_the_top_drawer_and_put_the_bowl_inside` | 135 min, 2 attempts | 8/15 → **15/15** (swept twice) | ~~17/50 (0.34)~~ | **48/50 (0.96)** |
| `put_the_cream_cheese_in_the_bowl` | 37 min, 3 attempts | confirmed complete | ~~49/50 (0.98)~~ | **49/50 (0.98)** |

Struck-out values are the v2 numbers and the reason they are not quotable: both
cells delivered while their own 51–65 sweep was still in flight, `open_the_top_
drawer`'s findings saying so in as many words ("still in flight when Stage 1 was
cut short"). Their manifests survive at
`/mnt/data/YifanKang/aspire_ab_results/archive_repair_pre_20260821/`.

Integrity, per cell: coordinator-launched Stage 2 only; exactly one 50-trial run;
`identity.code_sha256` verified against the delivered `fix_code.py`
(`7dbc76a06b3c9763`, `747fe8c2be4781d9`).

### What the repair actually shows

**`open_the_top_drawer` moves 17/50 → 48/50 — a 31-trial swing on the same task,
same model, same protocol.** The only change is that the agent was allowed to
finish. Session 11 had already flagged this cell as under-converged rather than
a clean measurement of the method; the repair confirms that by a wide margin, and
retroactively justifies reading the v2 low cells as harness artifacts wherever
their dev sweep was incomplete.

The mechanism is visible in the cell's own trace. Its delivered program scores
15/15 on dev, and the agent swept those 15 seeds **twice** on the byte-identical
delivered binary (md5 `1d561dd5…`) unprompted, because it had established that
the environment is non-deterministic — seed 54 returned `grip on handle` readings
of 0.209 and −0.001 from identical code. One pass cannot separate a converged
program from a lucky one. That double-check is exactly the work the v2 supervisor
truncated, and it is worth 31 held-out trials here.

`put_the_cream_cheese_in_the_bowl` reproduces 49/50 exactly. That is the useful
kind of null — the v2 value was right, and it now rests on a sweep the agent
confirmed complete, so it becomes quotable rather than changing.

### Corrected v2 Arm-B total

**418/500 = 0.836** (was 387/500 = 0.774). One cell moves; nothing else is rerun.

Applied to the comparison in §B, our +demos column now leads the like-for-like
"w/o Evo" configuration on **9 of 10 tasks** (`put_the_bowl_on_the_stove`, 0.32
vs 0.56, is the sole remaining loss) and the mean gap widens from 0.774–0.47 to
0.836–0.47. The caveats in §B are unchanged and still apply: different inner
model, different hardware, one run per cell, and our arm carries the K=5 demo
treatment their pipeline does not.

## B. Our ten-task column against the correct ASPIRE table

Since §A.0 establishes that the comparable published column is **Table 7**
(Position perturbation / Swap), not Table 8, the whole v2 table should be cited
against it. Their protocol matches ours exactly — "Aspire learns and collects
skills on seeds 51–65 and evaluates one generated program on seeds 1–50".

| Task (goal_swap) | **Ours: Arm B (+demos), opus-5** | ASPIRE w/o Evo (fix loop only) | ASPIRE Evo. search | ASPIRE (final) |
|---|---|---|---|---|
| put_the_bowl_on_top_of_the_cabinet | **1.00** | 0.74 | 1.00 | 1.00 |
| turn_on_the_stove | **1.00** | 0.68 | 0.90 | 0.90 |
| put_the_cream_cheese_in_the_bowl | **0.98** | 0.74 | 0.92 | 0.74 |
| put_the_wine_bottle_on_the_rack | **0.98** | 0.46 | 0.98 | 0.98 |
| put_the_bowl_on_the_plate | **0.96** | 0.36 | 0.92 | 0.92 |
| put_the_wine_bottle_on_top_of_the_cabinet | 0.90 | 0.24 | 1.00 | 1.00 |
| open_the_middle_drawer_of_the_cabinet | **0.86** | 0.02 | 0.72 | 0.72 |
| push_the_plate_to_the_front_of_the_stove | **0.40** | 0.00 | 0.00 | 0.00 |
| open_the_top_drawer_and_put_the_bowl_inside | **0.96** ‡ | 0.86 | – | 0.86 |
| put_the_bowl_on_the_stove | 0.32 | 0.56 | 0.96 | 0.96 |
| **mean** | **0.836** ‡ | **0.47** | 0.82 (n=9) | 0.81 |

‡ repaired in this pass (§C): `open_the_top_drawer` 0.34 → **0.96**,
`cream_cheese` unchanged at 0.98, mean 0.774 → **0.836**. Every other row is the
v2 number and was not rerun.

The like-for-like column is **"w/o Evo"** — Robot Execution Engine + skill
library, i.e. a fix loop with no evolutionary search, which is exactly what we
ran. Against it our +demos column is ahead on **9 of 10 tasks** and on the mean
(0.836 vs 0.47), and it now edges their full pipeline (0.81), which adds an
evolutionary search we did not run. One task remains behind:
`put_the_bowl_on_the_stove` (0.32 vs 0.56). `open_the_top_drawer` was the other
loss (0.34 vs 0.86) and the repair reverses it to 0.96 — that cell was measuring
our supervisor, not the method.

**Do not over-read this.** It is not a controlled comparison: different inner
model (opus-5 vs their Claude Opus 4.6), different hardware and GPU contention,
one run per cell on our side, and our arm has the K=5 demo treatment that their
pipeline does not. It is a reference point that says our reproduction is in the
right regime and, on the fix-loop-only configuration, above it.

---

# v3 — THE THREE-COLUMN TABLE (2026-08-21)

TASK.md SCOPE v3's deliverable: ten `libero_goal_swap` tasks, held-out seeds
1–50, three columns. This supersedes the "cite their published Table 8" decision
— our own 3-cell Arm A repro scored 0.873 against their published macro of 0.45,
so citing their number would have compared ENCORE against a strawman.

## The table

Held-out seeds 1–50, one 50-trial run per cell, `evidence_scope: heldout_full`.

| Task (`libero_goal_swap`) | ENCORE (c2, our harness) | ASPIRE (opus, their framework) | ASPIRE+demos (opus, their framework) |
|---|---|---|---|
| open_the_middle_drawer_of_the_cabinet | 0/50 · **0.00** | 50/50 · **1.00** | 43/50 · 0.86 |
| put_the_bowl_on_the_stove | 48/50 · 0.96 | 38/50 · 0.76 | 16/50 · **0.32** |
| put_the_wine_bottle_on_top_of_the_cabinet | 50/50 · 1.00 | 50/50 · 1.00 | 45/50 · 0.90 |
| open_the_top_drawer_and_put_the_bowl_inside | 50/50 · 1.00 | 42/50 · 0.84 | 48/50 · 0.96 |
| put_the_bowl_on_top_of_the_cabinet | 50/50 · 1.00 | 50/50 · 1.00 | 50/50 · 1.00 |
| push_the_plate_to_the_front_of_the_stove | 50/50 · **1.00** | 13/50 · **0.26** | 20/50 · 0.40 |
| put_the_cream_cheese_in_the_bowl | 45/50 · 0.90 | 39/50 · 0.78 | 49/50 · 0.98 |
| turn_on_the_stove | 50/50 · 1.00 | 50/50 · 1.00 | 50/50 · 1.00 |
| put_the_bowl_on_the_plate | 50/50 · 1.00 | 50/50 · 1.00 | 48/50 · 0.96 |
| put_the_wine_bottle_on_the_rack | 46/50 · 0.92 | 49/50 · 0.98 | 49/50 · 0.98 |
| **macro mean** | **439/500 = 0.878** | **431/500 = 0.862** | **418/500 = 0.836** |

Sources: ENCORE from `autoresearch/campaigns/c2/evals/eval_v111_c2_goal_*_pos/
results.jsonl` (50 unique episodes per cell, `benchmark_success`); both ASPIRE
columns from ASPIRE's own run manifests, `armA_heldout_v3.json` /
`armB_heldout_v3.json`.

## The headline is not the mean — it is that the means hide everything

The three aggregates sit inside 4 points of each other (0.878 / 0.862 / 0.836)
and they are the least informative numbers in the table. Per cell the columns
disagree violently, and each column's weakness is somewhere else:

- **ENCORE's mean is carried by nine near-perfect cells and holed by one total
  failure.** `open_the_middle_drawer_of_the_cabinet` is **0/50** — a known
  reachability mechanism gap in that harness, not noise. Both ASPIRE arms solve
  that same cell (1.00 and 0.86). Drop that one cell and ENCORE is 439/450 =
  0.976; no other column has anything like that shape.
- **Both ASPIRE arms collapse on `push_the_plate` (0.26, 0.40) where ENCORE
  scores 50/50.** That is the largest single gap in the table and it runs the
  other way. It is also the cell where ASPIRE's *own published* pipeline scores
  0.00 on this suite (§A.0), so the fix loop — ours or theirs — has a structural
  problem with non-prehensile pushing that ENCORE's harness does not.
- **Six of ten cells are at or near ceiling in all three columns.** They separate
  nothing. The table is really decided by four cells: middle_drawer, push_plate,
  bowl_on_stove, cream_cheese.

So the honest one-line reading is: **on this suite the three systems are within
noise of each other in aggregate, and the interesting content is entirely in
which cell each one fails.** Anyone quoting 0.878 vs 0.862 vs 0.836 as a ranking
is quoting three different failure modes averaged into indistinguishability.

## The ±demos comparison is CONFOUNDED in this table — do not read it as the ablation

Arm A (0.862) scores **above** Arm B (0.836), which reads as "+demos hurts". That
reading is not supported, because the two columns did not run under the same
harness:

- Arm A ran with **`CONVERGE=1`** — the supervisor fix from the repair pass,
  which refuses to accept a Stage 1 delivery until the agent confirms it swept
  all 15 dev seeds with the delivered program.
- **Eight of ten Arm B cells ran without it** (v2 supervisor). Only the two
  repaired cells had it.

`CONVERGE=1` can only ever cause *more* debugging, never less, so it advantages
Arm A. And the effect size is measured, not hypothetical: the repair pass moved
`open_the_top_drawer` from **17/50 to 48/50** — 31 trials — on the same task,
model and protocol, with the supervisor as the only change. That single measured
effect is roughly seven times the 13-trial gap between the two columns.

The per-cell evidence points the same way. On `put_the_bowl_on_the_stove` Arm A
delivered a program at **13/15 dev** and scored 38/50; Arm B delivered at **8/15**
and scored 16/50 — a 22-trial gap on the cell where the dev-convergence gap is
widest. That is the supervisor asymmetry showing up directly, not a demo effect.

**This was a deliberate, logged choice at dispatch, taken in the conservative
direction** (TASK.md v3 asks for Arm A "to convergence", and reproducing a known
harness defect in the new column to match would have been worse). But it means
the ±demos row in this table is not the ablation. The v1 three-task A/B, where
both arms ran the same supervisor, remains the only clean ±demos measurement in
this campaign.

**What would fix it:** rerun the eight non-`CONVERGE` Arm-B cells under
`CONVERGE=1`. That is ~8 fix loops, and it is the single highest-value follow-up
in the campaign — without it the +demos claim rests on v1's one non-ceiling cell.

## Where +demos does still look real, per cell

Holding the confound in mind, Arm B beats Arm A on exactly three cells, and two
of them are the repaired ones (i.e. the cells where the supervisors *did* match):

| Cell | A | B | note |
|---|---|---|---|
| put_the_cream_cheese_in_the_bowl | 0.78 | **0.98** | both CONVERGE — a like-for-like Arm B win |
| open_the_top_drawer_and_put_the_bowl_inside | 0.84 | **0.96** | both CONVERGE — a like-for-like Arm B win |
| push_the_plate_to_the_front_of_the_stove | 0.26 | **0.40** | A had CONVERGE, B did not — B wins *against* the confound |

**The two cells where the harnesses genuinely match both go to +demos**, and the
third goes to +demos despite the confound running the other way. That is a much
better-founded (if small, n=3) version of the ±demos claim than the macro mean,
and it is the version worth quoting until the eight-cell rerun exists.

## `push_the_plate`: a correct mechanism model lost to a robust search

The most instructive single cell in the campaign, and it refutes part of §A.2/A.3
— see §A.5 below.

## Cost

| | Stage 1 wallclock | agent events | API deaths | converge checks used |
|---|---|---|---|---|
| Arm A (10 cells) | **991 agent-min** | 14 056 | **0** | 10 (one per cell) |
| Arm B v2 (10 cells) | 682 agent-min | 21 748 | 6 | n/a (feature did not exist) |

Arm A spent ~45 % more wallclock for ~35 % fewer events — consistent with
`CONVERGE=1` buying longer, sweep-dominated Stage 1s (each cell paid for at least
one extra full 15-seed sweep) rather than more agent turns. Campaign wallclock
16:17 → 23:37 local (7 h 20 m) across three task GPUs, concurrent with a second
Heron campaign on GPU 3.

## Integrity — verified on executed data, not declarations

Per cell, all ten, by an independent probe reading the per-trial result
directories rather than the manifest summary:

| Check | Arm A |
|---|---|
| independent trial-dir recount == manifest `passes`/`trials` | 10/10 exact |
| **executed** seed set (per-trial `seed` fields) == exactly 1..50 | 10/10 |
| any dev seed (>50) inside a held-out run | **none** |
| duplicate trial indices | none |
| manifest `identity.code_sha256` == sha256 of delivered `fix_code.py` | 10/10 |
| run directories per task | exactly 1 — no superseded run hiding |
| `evidence_scope` | `heldout_full` 10/10 |
| Stage 1 agents that invoked `run_fix_loop_validation.py` | **0** (grepped every attempt log for Bash invocations) |

Every Stage 2 was launched by `coordinator_daemon_armA.sh`, the only component
permitted to touch seeds 1–50. Arm A started from the same cold state Arm B did:
empty `outputs/`, skill library `75b397c6…c449` byte-identical across both
checkouts, zero skill promotions.

## Caveats that apply to the whole table

1. **The ENCORE column runs on a different harness.** Different perception stack,
   different API surface, different action interface — only the suite, the bddl
   task definitions and the sealed seed set 1–50 are shared. This is exactly the
   confound the planned **ENCORE-on-CaP-X port** is meant to remove, and it should
   be named as such wherever this table is used: column 1 is not a controlled
   comparison against columns 2–3, it is a reference point on the same benchmark.
2. **The ±demos row is confounded by the supervisor asymmetry** (above). Not the
   ablation.
3. **One run per cell.** Two agents independently established that this pipeline
   is **not bitwise reproducible** — SAM3 runs on shared GPU servers and small
   mask differences propagate into every downstream pose. `put_the_bowl_on_the_
   stove`'s agent swept its delivered program twice and got 13/15 both times but
   with *different* seeds failing ({51,64} and {54,64}); `open_the_top_drawer`'s
   found `grip on handle` readings of 0.209 and −0.001 from byte-identical code
   on seed 54. So single-cell differences of a few trials are inside run-to-run
   noise, and only the large gaps in this table should be read as signal.
4. **Inner model** `claude-opus-5` (first-party) for both ASPIRE columns, pinned
   and verified per cell; ASPIRE's paper used Opus 4.6.
5. Three task GPUs instead of their five (TASK.md-sanctioned), and the box was
   shared with a second campaign throughout. Wallclock only — every held-out eval
   is a deterministic replay of a fixed program over a fixed seed set.


---

# v4 — THE FULL 58-CELL PLAIN-ASPIRE COLUMN (2026-08-24, session 17)

Scope v4's deliverable: ASPIRE's own framework, their runbook verbatim, no demo
addendum, inner code-generation model pinned to **claude-opus-5** — run over the
58-cell comparable matrix and set beside ENCORE's c2 column on the same cells and
the same sealed seeds 1-50.

**All 58 cells were executed and banked; nothing is projected or partial.** The
Stage 1 sweeps finished 2026-08-22 15:24 and the last held-out eval 2026-08-22
16:54 (`scheduler_v4.log`, `coordinator_v4.log`); session 17 audited and collected
them.

## Headline

| column | 58 cells, seeds 1-50 | rate |
|---|---|---|
| **ASPIRE (their framework, opus-5 inner)** | **2596 / 2900** | **0.895** |
| **ENCORE (c2, our harness)** | **2761 / 2900** | **0.952** |

## Per suite

| suite | cells | ASPIRE (opus-5) | ENCORE (c2) | gap |
|---|---:|---|---|---:|
| `libero_goal_swap` | 10 | 431/500 = 0.862 | 439/500 = 0.878 | +0.016 |
| `libero_goal_task` | 10 | 432/500 = 0.864 | 445/500 = 0.890 | +0.026 |
| `libero_object_swap` | 10 | 498/500 = 0.996 | 500/500 = 1.000 | +0.004 |
| `libero_object_task` | 10 | 498/500 = 0.996 | 500/500 = 1.000 | +0.004 |
| `libero_spatial_swap` | 10 | 418/500 = 0.836 | 496/500 = 0.992 | +0.156 |
| `libero_spatial_task` | 8 | 319/400 = 0.797 | 381/400 = 0.953 | +0.155 |
| **total** | **58** | **2596/2900 = 0.895** | **2761/2900 = 0.952** | **+0.057** |

The two columns are close on the object suites (both essentially saturated) and on
the goal suites, and separate almost entirely on **spatial**: `spatial_swap`
0.836 vs 0.992 and `spatial_task` 0.797 vs 0.953. Three quarters of the 165-trial
total gap lives in those 18 cells. That localisation is the result — not the mean.

**The harness confound still applies and is still the reason for the planned
ENCORE-on-CaP-X port**: the ENCORE column ran on our harness, the ASPIRE column on
theirs. Same suite, same bddl, same sealed seeds, same inner model family — but not
the same execution stack, so a per-suite gap of this shape is a hypothesis about
where the methods differ, not yet a measurement of it.

## Per cell

| suite / task | ASPIRE | ENCORE (c2 cell) |
|---|---:|---:|
| `libero_goal_swap/open_the_middle_drawer_of_the_cabinet` | 50/50 | 0/50 (`goal_open_middle_drawer_pos`) |
| `libero_goal_swap/open_the_top_drawer_and_put_the_bowl_inside` | 42/50 | 50/50 (`goal_open_top_drawer_put_bowl_pos`) |
| `libero_goal_swap/push_the_plate_to_the_front_of_the_stove` | 13/50 | 50/50 (`goal_push_plate_front_stove_pos`) |
| `libero_goal_swap/put_the_bowl_on_the_plate` | 50/50 | 50/50 (`goal_put_bowl_on_plate_pos`) |
| `libero_goal_swap/put_the_bowl_on_the_stove` | 38/50 | 48/50 (`goal_put_bowl_on_stove_pos`) |
| `libero_goal_swap/put_the_bowl_on_top_of_the_cabinet` | 50/50 | 50/50 (`goal_put_bowl_top_cabinet_pos`) |
| `libero_goal_swap/put_the_cream_cheese_in_the_bowl` | 39/50 | 45/50 (`goal_put_cream_cheese_in_bowl_pos`) |
| `libero_goal_swap/put_the_wine_bottle_on_the_rack` | 49/50 | 46/50 (`goal_put_wine_on_rack_pos`) |
| `libero_goal_swap/put_the_wine_bottle_on_top_of_the_cabinet` | 50/50 | 50/50 (`goal_put_wine_top_cabinet_pos`) |
| `libero_goal_swap/turn_on_the_stove` | 50/50 | 50/50 (`goal_turn_on_stove_pos`) |
| `libero_goal_task/open_the_middle_drawer_of_the_cabinet` | 49/50 | 48/50 (`goal_open_middle_drawer_task`) |
| `libero_goal_task/open_the_top_drawer_and_put_the_bowl_inside` | 42/50 | 49/50 (`goal_open_top_drawer_put_bowl_task`) |
| `libero_goal_task/push_the_plate_to_the_front_of_the_stove` | 50/50 | 50/50 (`goal_push_plate_front_stove_task`) |
| `libero_goal_task/put_the_bowl_on_the_plate` | 50/50 | 50/50 (`goal_put_bowl_on_plate_task`) |
| `libero_goal_task/put_the_bowl_on_the_stove` | 0/50 | 50/50 (`goal_put_bowl_on_stove_task`) |
| `libero_goal_task/put_the_bowl_on_top_of_the_cabinet` | 50/50 | 0/50 (`goal_put_bowl_top_cabinet_task`) |
| `libero_goal_task/put_the_cream_cheese_in_the_bowl` | 50/50 | 48/50 (`goal_put_cream_cheese_in_bowl_task`) |
| `libero_goal_task/put_the_wine_bottle_on_the_rack` | 44/50 | 50/50 (`goal_put_wine_on_rack_task`) |
| `libero_goal_task/put_the_wine_bottle_on_top_of_the_cabinet` | 50/50 | 50/50 (`goal_put_wine_top_cabinet_task`) |
| `libero_goal_task/turn_on_the_stove` | 47/50 | 50/50 (`goal_turn_on_stove_task`) |
| `libero_object_swap/pick_up_the_alphabet_soup_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_alphabet_soup_pos`) |
| `libero_object_swap/pick_up_the_bbq_sauce_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_bbq_sauce_pos`) |
| `libero_object_swap/pick_up_the_butter_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_butter_pos`) |
| `libero_object_swap/pick_up_the_chocolate_pudding_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_chocolate_pudding_pos`) |
| `libero_object_swap/pick_up_the_cream_cheese_and_place_it_in_the_basket` | 49/50 | 50/50 (`obj_cream_cheese_pos`) |
| `libero_object_swap/pick_up_the_ketchup_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_ketchup_pos`) |
| `libero_object_swap/pick_up_the_milk_and_place_it_in_the_basket` | 49/50 | 50/50 (`obj_milk_pos`) |
| `libero_object_swap/pick_up_the_orange_juice_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_orange_juice_pos`) |
| `libero_object_swap/pick_up_the_salad_dressing_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_salad_dressing_pos`) |
| `libero_object_swap/pick_up_the_tomato_sauce_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_tomato_sauce_pos`) |
| `libero_object_task/pick_up_the_alphabet_soup_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_alphabet_soup_task`) |
| `libero_object_task/pick_up_the_bbq_sauce_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_bbq_sauce_task`) |
| `libero_object_task/pick_up_the_butter_and_place_it_in_the_basket` | 48/50 | 50/50 (`obj_butter_task`) |
| `libero_object_task/pick_up_the_chocolate_pudding_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_chocolate_pudding_task`) |
| `libero_object_task/pick_up_the_cream_cheese_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_cream_cheese_task`) |
| `libero_object_task/pick_up_the_ketchup_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_ketchup_task`) |
| `libero_object_task/pick_up_the_milk_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_milk_task`) |
| `libero_object_task/pick_up_the_orange_juice_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_orange_juice_task`) |
| `libero_object_task/pick_up_the_salad_dressing_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_salad_dressing_task`) |
| `libero_object_task/pick_up_the_tomato_sauce_and_place_it_in_the_basket` | 50/50 | 50/50 (`obj_tomato_sauce_task`) |
| `libero_spatial_swap/pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate` | 49/50 | 50/50 (`spa_bowl_between_pos`) |
| `libero_spatial_swap/pick_up_the_black_bowl_from_table_center_and_place_it_on_the_plate` | 48/50 | 50/50 (`spa_bowl_table_center_pos`) |
| `libero_spatial_swap/pick_up_the_black_bowl_in_the_top_drawer_of_the_wooden_cabinet_and_place_it_on_the_plate` | 31/50 | 50/50 (`spa_bowl_top_drawer_cabinet_pos`) |
| `libero_spatial_swap/pick_up_the_black_bowl_next_to_the_cookie_box_and_place_it_on_the_plate` | 47/50 | 50/50 (`spa_bowl_cookie_box_pos`) |
| `libero_spatial_swap/pick_up_the_black_bowl_next_to_the_plate_and_place_it_on_the_plate` | 50/50 | 50/50 (`spa_bowl_next_to_plate_pos`) |
| `libero_spatial_swap/pick_up_the_black_bowl_next_to_the_ramekin_and_place_it_on_the_plate` | 40/50 | 49/50 (`spa_bowl_next_to_ramekin_pos`) |
| `libero_spatial_swap/pick_up_the_black_bowl_on_the_cookie_box_and_place_it_on_the_plate` | 35/50 | 50/50 (`spa_bowl_on_cookie_box_pos`) |
| `libero_spatial_swap/pick_up_the_black_bowl_on_the_ramekin_and_place_it_on_the_plate` | 50/50 | 50/50 (`spa_bowl_on_ramekin_pos`) |
| `libero_spatial_swap/pick_up_the_black_bowl_on_the_stove_and_place_it_on_the_plate` | 38/50 | 50/50 (`spa_bowl_on_stove_pos`) |
| `libero_spatial_swap/pick_up_the_black_bowl_on_the_wooden_cabinet_and_place_it_on_the_plate` | 30/50 | 47/50 (`spa_bowl_on_wooden_cabinet_pos`) |
| `libero_spatial_task/pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate` | 19/50 | 49/50 (`spa_bowl_between_task`) |
| `libero_spatial_task/pick_up_the_black_bowl_from_table_center_and_place_it_on_the_plate` | 46/50 | 43/50 (`spa_bowl_table_center_task`) |
| `libero_spatial_task/pick_up_the_black_bowl_in_the_top_drawer_of_the_wooden_cabinet_and_place_it_on_the_plate` | 18/50 | 50/50 (`spa_bowl_top_drawer_cabinet_task`) |
| `libero_spatial_task/pick_up_the_black_bowl_next_to_the_cookie_box_and_place_it_on_the_plate` | 44/50 | 47/50 (`spa_bowl_cookie_box_task`) |
| `libero_spatial_task/pick_up_the_black_bowl_next_to_the_plate_and_place_it_on_the_plate` | 50/50 | 43/50 (`spa_bowl_next_to_plate_task`) |
| `libero_spatial_task/pick_up_the_black_bowl_next_to_the_ramekin_and_place_it_on_the_plate` | 50/50 | 49/50 (`spa_bowl_next_to_ramekin_task`) |
| `libero_spatial_task/pick_up_the_black_bowl_on_the_ramekin_and_place_it_on_the_plate` | 50/50 | 50/50 (`spa_bowl_on_ramekin_task`) |
| `libero_spatial_task/pick_up_the_black_bowl_on_the_wooden_cabinet_and_place_it_on_the_plate` | 42/50 | 50/50 (`spa_bowl_on_wooden_cabinet_task`) |

## Two extra cells, outside the 58

`libero_spatial_task/..._on_the_cookie_box_...` (33/50) and
`..._on_the_stove_...` (50/50) were executed by the scheduler but have **no c2
counterpart**: c2's siblings load re-authored bddl from `Heron/bddl_fixed/`, not
this suite. They are excluded from every total above and reported here so the
executed set (60 cells) reconciles with the reported set (58) with nothing hidden.

## Integrity — re-derived from executed data, session 17

Auditor `audit_v4.py` plus an independent recount (`recount.py`) over all 60
executed cells:

- **one run per cell, 60/60** — no ambiguous second evaluation anywhere.
- **executed seed set == exactly 1-50, 60/60**; **dev-seed leak (51-65): none**.
- **delivered `fix_code.py` sha256 == the manifest's evaluated `code_sha256`,
  60/60** — the evaluated program is the delivered one.
- **`evidence_scope == heldout_full`, `status == complete`, 60/60**.
- **pass counts confirmed twice independently**: the per-seed `task_completed`
  field and, separately, the `taskcompleted_*` token parsed out of each trial
  directory's own name. Manifest, field recount and name recount agree for all 60
  cells; **zero discrepancies**.
- **`libero_goal_swap` re-derives as 431/500**, identical to sessions 15 and 16 —
  the new collection path reproduces an independently-produced number.
- **Inner model verified per cell, not assumed**: 18 242 model records across the
  60 agent event streams, **all `claude-opus-5`, zero others**.

### One flag raised and resolved: the `sonnet-4-6` in the output path

Every eval trial lives under a directory segment `aws_anthropic_bedrock-claude-sonnet-4-6`.
It does **not** mean a sonnet model produced or evaluated anything. It is a
**hardcoded path literal in ASPIRE's own repo** (`scripts/libero/make_snapshot_table.py:89`),
and the held-out eval is a deterministic replay of `fix_code.py` that calls no model
at all. The code generator is our outer `claude -p --model claude-opus-5`, verified
above from the agents' own event streams. Recorded because a reader of these paths
would otherwise reach the wrong conclusion.

The auditor also flags `results.jsonl passes 0 vs manifest N` for all 60 cells.
Checked at the source: ASPIRE's fix-loop eval **writes no `results.jsonl`** — the
per-seed record lives in `manifest.json` — so that check is vacuous rather than
failing. It fires identically on the ten goal_swap cells already validated at
431/500. The recount above replaces it.

---

# v5 — ARM B AT K=3 UNDER CONVERGE=1, ALL TEN `libero_goal_swap` CELLS (2026-09-02)

Why: the banked Arm-B column (418/500) mixed two supervisor regimes (eight
cells under the v2 clean-return supervisor, two repaired cells under
`CONVERGE=1`) and gave the baseline K=5 demonstrations while ENCORE receives
K=3. Both confounds removed in one pass (Yifan's call, 15:50): all ten cells
re-acquired from a clean slate with fresh session ids, K=3 demo files
(`aspire_demos_k3/`, `demo_0..demo_2` of the same LIBERO hdf5 as ENCORE's packs),
`CONVERGE=1`, `MAX_CLEAN_NUDGES=6`, `CONVERGE_CHECKS=2`, model `claude-opus-5`,
prompts byte-identical to v2 except the addendum's K/path lines and one GPU
pin, dev 51–65 to convergence, ONE held-out 1–50 per cell by the coordinator.
Pre-specified reading (NOTES.md, before dispatch): suite level only; compare to
A′ 431/500 with draw variance in mind; the 418 column is retired.

## Held-out (seeds 1–50, one run per cell)

| Task | K=5 / mixed supervisor (retired) | **K=3 / CONVERGE=1** | Δ |
|---|---|---|---|
| open_the_middle_drawer_of_the_cabinet | 43 | **43** | 0 |
| put_the_bowl_on_the_stove | 16 | **48** | +32 |
| put_the_wine_bottle_on_top_of_the_cabinet | 45 | **45** | 0 |
| open_the_top_drawer_and_put_the_bowl_inside | 48 | **41** | −7 |
| put_the_bowl_on_top_of_the_cabinet | 50 | **50** | 0 |
| push_the_plate_to_the_front_of_the_stove | 20 | **11** | −9 |
| put_the_cream_cheese_in_the_bowl | 49 | **49** | 0 |
| turn_on_the_stove | 50 | **50** | 0 |
| put_the_bowl_on_the_plate | 48 | **50** | +2 |
| put_the_wine_bottle_on_the_rack | 49 | **50** | +1 |
| **total** | 418/500 = 0.836 | **437/500 = 0.874** | +19 |

Integrity: `collect10.py` (manifest `passes/trials`, one 50-trial run per cell,
`identity.code_sha256` = delivered `fix_code.py`) and `verify12.py` (per-cell
manifests, exactly one run directory, `evidence_scope: heldout_full`) agree on
every cell. Machine-readable: `armB_heldout_k3_converge.json`. Stage 1 wallclock
15:48 → 21:28; `push_the_plate` alone took 4 h 27 m (v12 of its program) and
delivered at 11/50, the only cell below 41.

## Reading

Same-framework goal_swap columns are now A′ (no demos) 431 / Arm B (K=3, raw
trajectory files) 437 / ENCORE-on-CaP-X (K=3 packs) 437, all under
`CONVERGE=1`. Handing ASPIRE's loop the same three demonstrations as raw files
changes the suite score by +6/500 — inside the 12–26-point single-draw
variance measured in `abl_c2`, so the paper's sentence ("brings no measurable
gain") stands, now with the supervisor confound removed and the demo count
matched. Per cell the two draws of Arm B differ by up to 32 trials on the same
task with the same demos (bowl_on_stove 16 → 48; push_plate 20 → 11), which is
the same draw variance again: do not read per-cell deltas.
