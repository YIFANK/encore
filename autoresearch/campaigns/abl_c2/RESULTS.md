# abl_c2 — ENCORE component ablation results

**Status: COMPLETE** (2026-08-21). All nine cells of the 3x3 have sealed
numbers; the drawer row's variant-B cell has two, for the reason given below;
variant C additionally carries a six-task extension.

Sealed held-out evaluation: seeds 1-50, coordinator-run, blind, once per cell,
under `tools/fair_run.py --split eval` (fair-v1.1.1 gate). Every scored program
was verified at declaration time and again in two full audit sweeps: the eval's
own `program_archived.py` md5 == the frozen `program.py` md5 == the md5
recorded in `NOTES.md` when the cell was declared; every eval covers seeds 1-50
exactly once; every scored program carries a PROVENANCE dict.

Learn-phase episode counts are debug-split (seeds 51-65) episodes summed over
every probe and selection run of the cell.

**Read `NOTES.md` "CONFOUNDS" before interpreting any row.** The single most
important caveat is in §4 Variance below and it applies to every number here.

---

## 1. The 3x3 — sealed success (N/50)

| task | full ENCORE (c2) | A naive-demos | B no-LAWS | C no-verify |
|---|---|---|---|---|
| turn on the stove | 50/50 | **50/50** | **50/50** | **50/50** |
| put the bowl on the plate | 50/50 | **47/50** | **50/50** | **16/50** |
| open the middle drawer | 48/50 | **50/50** | **0/50** ᵃ / **50/50** ᵇ | **19/50** |

ᵃ draw 1, 5-version cap (the campaign budget) — budget exhausted at v5.
ᵇ draw 2, 12-version cap (reference parity; declared protocol deviation, R15) —
converged at v4, i.e. *inside* the original cap. Both are reported; see §4.

### Learn-phase episodes consumed (sample efficiency)

| task | full ENCORE (c2) | A naive-demos | B no-LAWS | C no-verify |
|---|---|---|---|---|
| turn on the stove | 60 | **23** | **23** | **0** |
| put the bowl on the plate | 64 | **50** | **31** | **0** |
| open the middle drawer | 86 | **21** | **43** ᵃ / **48** ᵇ | **0** |

### Program versions written

| task | full ENCORE (c2) | A naive-demos | B no-LAWS | C no-verify |
|---|---|---|---|---|
| turn on the stove | 3 | **1** | **1** | **1** |
| put the bowl on the plate | 4 | **4** | **2** | **1** |
| open the middle drawer | 11 | **3** | **5** ᵃ (exhausted) / **4** ᵇ | **1** |

---

## 2. Variant C extended to nine tasks

Variant C is the one arm with no learn phase — one program, zero episodes, one
sealed eval — so it can be swept cheaply across a task suite. Six further
c2 goal-suite `_stock` tasks were run as fresh variant-C cells (same pack
construction, same brief, same gate), each against its own full-ENCORE
reference.

| task | full ENCORE (c2) | C no-verify |
|---|---|---|
| turn on the stove | 50/50 | **50/50** |
| put the bowl on the plate | 50/50 | **16/50** |
| open the middle drawer of the cabinet | 48/50 | **19/50** |
| put the bowl on the stove | 50/50 | **27/50** |
| put the bowl on top of the cabinet | 50/50 | **0/50** |
| put the cream cheese in the bowl | 50/50 | **0/50** |
| open the top drawer and put the bowl inside | 49/50 | **4/50** |
| put the wine bottle on the rack | 47/50 | **1/50** |
| push the plate to the front of the stove | 46/50 | **0/50** |
| **total** | **440/450 (97.8%)** | **117/450 (26.0%)** |

Six of nine tasks fall below 20%; four are at or near zero. The one tie is
`turn_on_the_stove`, which every condition solves and which therefore has no
discriminative power (NOTES R7).

This extension widens variant C only — there is no A or B arm on the six new
tasks — so it is a C-vs-reference comparison across nine tasks, not a wider
3x3.

---

## 3. What the table supports

**The stove row is a ceiling row.** All four conditions score 50/50. It cannot
support any claim that a component does or does not matter in general; it shows
only that none was needed there.

**Variant C (no verification) is the one component with a large, consistent
effect.** Across nine tasks, 117/450 against the reference's 440/450. The
failures are not sloppiness: the C programs are closed-loop, guarded, and
designed to degrade gracefully. They fail because a guard threshold or a
transferred constant that cannot be calibrated without an episode is wrong, and
nothing in the program can discover that. The worked example
(`put_the_cream_cheese_in_the_bowl`, 0/50, NOTES R25): the object-detection
branch returned a bit-identical position on all 50 episodes while the *bowl*
branch of the same pipeline tracked the layout correctly, and the wrist-camera
correction that would have fixed it — ~0.15 m — was rejected 50 times out of 50
by a "< 5 cm" sanity clamp the worker had no way to calibrate. Every defensive
mechanism worked exactly as written and the composition scored zero.

Each C worker was required to list, *before* any eval, what it would have
probed first. In the zero-scoring cells the thing that actually broke is on
that list. The agents identified the right risks and had no way to retire
them — which is what the ablation is measuring.

**Variant A (raw demos instead of a mined pack) is not clearly harmful, and on
one task beat the reference.** 50/50, 47/50, 50/50 against 50/50, 50/50, 48/50,
at 23/50/21 learn-phase episodes against 60/64/86. On the drawer it beat the
reference on both axes. The proposed mechanism (NOTES R10/R14) is *salience*,
not information: the decisive fact lives in the dense action stream, and the
raw dump gave the worker no keyframe table to anchor on, so it read the
actions. `pack.json` carries the same array. This is n=1 per arm and is stated
as a hypothesis worth a dedicated experiment, not a finding.

**Variant B (no law-writing) has no measurable effect on the success bit here,
but see §4.** It ties the reference on stove and bowl-on-plate at lower cost,
and on the drawer produced both the worst and the best cell in the campaign.

---

## 4. Variance — the caveat that governs everything above

The drawer row's variant-B cell was run twice: same variant, same task, same
pack, same brief. Draw 1 scored 0/15 at selection and 0/50 sealed. Draw 2
scored 15/15 at selection and 50/50 sealed. The version cap differed (5 vs 12)
but was **not** the operative difference: draw 2 converged at v4, one version
inside draw 1's cap.

The two draws differ in which diagnostic path a fresh worker happened to take,
and in nothing else that was controlled. **Every other cell in this table is
n=1, and this is a direct measurement that a single cell of this campaign can
span the entire outcome range.** No individual cell's number should be read as
that variant's effect on that task. The aggregate that survives this is the
nine-task variant-C total, because it averages nine independent draws.

---

## 5. Protocol deviations and residual channels (all declared)

1. **Raised version cap on one cell.** `ablB2_goal_open_middle_drawer` ran at
   12 versions instead of 5, to clear the reference's 11 (NOTES R15). Applies
   to that rerun only. Both B-drawer numbers are retained; neither replaces the
   other.
2. **Variant C permits offline reasoning over the pack, including executing its
   own code on pack-derived inputs.** Three workers disclosed doing so (stub-api
   control-flow lint; perception functions run on pack keyframes). No cell
   touched an environment, a seed, or any benchmark asset, so none could learn a
   fact absent from its pack. This is a slightly weaker ablation than "one shot,
   no checking of any kind", and weaker in the direction that **favours**
   variant C. Two workers did not report doing this, so there is a small
   unmeasured inconsistency within the C arm. (NOTES CONFOUND 4, R22.)
3. **Variant B removes the law-writing discipline, not a seeded law library.**
   The reference cells also had an empty `LAWS.md`, so no arm here can measure
   the shared-library channel. That needs a separate arm. (NOTES CONFOUND 1.)
4. **Variant A is a presentation ablation, not information removal.** The raw
   dump is a strict superset of the pack's numeric content, and A receives more
   image frames, not fewer. (NOTES CONFOUND 2.)
5. **Cells are not hermetically isolated at the harness level.** They are
   isolated by brief, by cluster-side namespacing, and by post-hoc audit. In
   session 1 all workers shared one local scratchpad and two disclosed
   collisions (NOTES R13, adjudicated there). From session 2 onward every worker
   got a private scratchpad. Between-cell independence must be claimed at that
   strength and no higher.
6. **A route not taken, and why.** Extending the C row by evaluating c2's own
   archived `program_v1` was attempted first and abandoned: those programs read
   `api.done`, an in-episode success oracle that the current gate refuses and
   that fresh C cells were denied outright, so they are a strictly easier
   condition wearing variant C's label. No seeds were consumed. (NOTES R18.)

---

## Provenance

- Reference column: `campaigns/c2/scoreboard_frozen.json`, not rerun.
- Per-cell receipts, mechanisms, adjudications and the full round log:
  `NOTES.md` (R0-R26).
- Cell definitions: `cell_manifest.txt`; briefs generated by `mkworker_abl.sh`;
  evals launched by `run_eval.sh`.
