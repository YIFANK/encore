# c2clean / spa_bowl_on_stove_task_k0 — working notes

Intent: **"Pick the akita black bowl on the top of the cabinet and place it on the plate"**
Zero demos. Debug seeds 51–65 only; probe subset = 51,53,55,57,59,61,63,65.
Runner: `tools/fair_run.py` exclusively.

## Scene (measured; cam_high RGB-D dumped through `api.log`, reconstructed offline)

`api.log` costs no sim steps, so the whole depth image can be shipped out as
zlib+base64 chunks and the perception developed offline. Base-frame facts, all
re-derived in this cell:

| thing | where | z |
|---|---|---|
| table top | workspace plane | 0.903 |
| cabinet top plane | x[-0.14,0.17] y[-0.34,-0.14] | 1.125 |
| **target bowl** (on the cabinet top) | ep51 (0.036,-0.269) · ep55 (0.048,-0.287), Ø 0.105 | rim 1.180 |
| stove slab | x≈-0.25 y≈-0.11 | 0.929 |
| bowl on the stove (distractor) | (-0.246,-0.109) | rim 0.978 |
| bowl on the table (distractor) | (-0.201,0.210) | rim 0.944 |
| **plate** (goal) | (0.054,0.198), 0.136 × 0.140 | top 0.920 |
| cookie box (distractor) | (0.078,0.024) | top 0.921 |

Three bowls in the scene. The instruction names the one **on the cabinet top**, so
the support plane (1.125, i.e. table+0.22) is the identity cue — no colour needed.
The goal `plate` is the largest *bright* low-profile blob standing on the table:
mean RGB ≈145 against ≈69 for the cookie box and 63–86 for the stove slab, with the
table bowl excluded by height (top = table+0.041 vs plate top = table+0.017).

**Bowl geometry** (radial cross-section of the debug-seed cloud). The bowl is a
thin cone, not a cylinder: outer radius 0.0546 at the rim (z 1.180) falling to
0.0436 at z 1.150, with the inner face tracking it ~0.009 inside. Both faces rise
at ~66°, so the shell is a 7–10 mm plate tilted 24° off vertical, and there is no
undercut anywhere. At 0.110 across it is far too wide for the 0.078 jaw span, so
the grasp has to be a **rim pinch**. Wrist-camera geometry puts the jaw separation
along the wrist image `u` axis, which maps to base **y** at the home tool rotation,
so the pinch goes on the ±y arc.

**Controller mechanics that dominated the whole cell.** `move_cartesian` returns as
soon as the residual drops under POS_TOL (0.012) and otherwise runs
`2 · 60 · seconds` steps. So `seconds` is a *step budget*, not a duration, and the
episode horizon is 1000 steps. A seed whose residual settles just above 0.012 pays
the full budget on every leg.

## Versions

| v | change | probe result |
|---|---|---|
| v0 | perception probe, no motion | — (72 sim steps = fixed per-episode overhead) |
| v1 | + jaw axis, contact probe, rim pinch | 0/4 — probe loop ate the horizon |
| v2 | descend-to-stall pinch | 0/4 — bit the shell, bowl escaped on the lift |
| v3 | shallow pinch, bias-cancelled aim, gentle lift, place | **7/8** |
| v4 | centre from unoccluded bbox edges + width-verified re-grasp | 7/8 |
| v5 | circle fit to the rim ring + staged place descent | 7/8 |
| v6 | fewer moves (pre-compensated descent, merged legs) | 7/8 |
| v7 | every leg given a short step budget | **8/8** |
| v8 | bite threshold 0.005 → 0.003 (margin) | 8/8 · formal **14/15** |
| v9 | closed-loop pinch depth (one-sided) | 9/10 |
| **v10** | **two-sided depth regulation — FROZEN** | 9/10 · formal **14/15** |
| v11 | gentler two-step lift, shorter settle | 8/10 — rejected |

(v9–v11 probe on 51,53,55,57,58,59,61,63,64,65, which swaps in the two hardest
seeds the v8 selection run exposed.)

### v0 — perception probe (no motion)
Hypothesis: the scene can be read from cam_high alone.
Evidence: seeds 51,53 — depth dumped and reconstructed offline; table, cabinet top,
all three bowls, plate and box recovered. 72 sim steps for a motionless episode.
Verdict: perception route confirmed.

### v1 — jaw axis + contact probe + rim pinch
Hypothesis: a stepped contact descent measures the eef→fingertip offset.
Evidence: seeds 51,53 hit sim_steps 1000. The "contact" the probe reported was
horizon exhaustion, not contact — past the horizon the eef freezes and the gripper
reports effort 3.0 regardless. One reading survived: on seed 51 the eef stalled at
1.1355 over a cabinet top at 1.125, giving **tip offset ≈ 0.010**.
Verdict: budget is ~12 moves per episode; drop the separate calibration.

### v2 — descend-to-stall rim pinch
Hypothesis: descending until the eef stalls finds the deepest straddle of the wall.
Evidence: seeds 51,53,55,57. The stall was the inner fingertip hitting the bowl's
**cavity floor** (eef 1.139–1.145, floor 1.135 — confirming tip offset 0.010), and
the eef drifted up to 0.025 in +x on the way, so the jaws met the wall ~33° off its
normal. Jaws bit the shell (w 0.0073–0.0101, effort 3.0) but the bowl escaped on
every lift (w decayed to 0.0048, effort back to 0.05); the GIF shows it gone from
the cabinet top. Note `effort` is only a gap flag (3.0 iff gap > 5 mm), so the
width, not the effort, is the receipt.
Verdict: stop descending at a *chosen* depth, and fix the lateral aim.

### v3 — shallow pinch, bias-cancelled aim, gentle lift, full place
Hypothesis: pinch where the shell is thin (fingertips 0.020 below the rim), meet it
square, and the bite will carry.
Evidence: **7/8** (51,53,57,59,61,63,65 ✓; 55 ✗). Closed width 0.0073–0.0081 with
effort 3.0 on all seven; 0.0010 on seed 55.
Verdict: the grasp is right. Seed 55 closed on air.

### v4 — unbiased centre from the near bbox edges + width-verified re-grasp
Hypothesis: seed 55's bowl blob is clipped on its far (−y) side (ey 0.092 against
ex 0.108), so the bbox centre sits 0.008 toward +y of the true centre.
Evidence: 7/8. The re-grasp fired on 55 and *worked* — second bite 0.0056, effort
3.0, carried to the plate — but the place descent jammed 10 mm off-centre.
Verdict: diagnosis right, remedy incomplete.

### v5 — circle fit to the rim ring + staged place descent
Hypothesis: fitting a circle to the rim ring beats any bbox-edge heuristic.
Evidence: 7/8. The fit is excellent (seed 55: centre (0.0483,−0.2871), R 0.0525,
rms 0.0028 — and it recovers exactly the centre the clipped bbox had missed).
Seed 55 bit the shell at 0.0051 and carried the bowl, then froze above the plate.
Verdict: perception solved; the remaining failure is not perception.

### v6 — fewer moves
Hypothesis: seed 55 is running out of horizon before it releases.
Evidence: 7/8, and the timing proved the mechanism. On seed 55 the descent and
staging legs took 9–10 s of wall clock each against 0.4–1.4 s on seed 51, because
55's residual settles at 0.016–0.026, just *above* POS_TOL = 0.012, so each 2.5 s
leg ran its full 300 steps. Seed 51 finished the entire task in 205 sim steps; seed
55 died at 1000 with the bowl still in the gripper above the plate.
Verdict: the failure is step budget, not motion.

### v7 — a short step budget on every leg
Hypothesis: capping `seconds` per leg (1.0–1.2 s, corrections 0.6 s) and widening
the correction-loop exit to 0.006 keeps a non-converging seed inside the horizon.
Evidence: **8/8** on the probe seeds (51,53,55,57,59,61,63,65). Seed 55 finished at
828 sim steps; every other seed in 178–207.
Verdict: task solved; margin on seed 55 is thin.

### v8 — bite threshold 0.003
Hypothesis: seed 55's first bite (0.0047) triggered a re-grasp it did not need —
the same 0.0050 bite carried the bowl in v6. Bites that hold read 0.0047–0.0081;
air reads 0.0010. A threshold of 0.003 sits between the populations rather than
inside the holding one, and skipping the needless re-grasp buys back ~250 steps.
Evidence: 8/8 on the probe, seed 55 down from 828 to 533 sim steps. First formal
run on all 15 debug seeds: **14/15**, failing only seed 58 (horizon, 1000 steps),
with seed 64 uncomfortably close at 797.
Verdict: good, and the formal run exposed two hard seeds (58, 64) the odd-seed
probe had never covered — folded into the probe subset from here on.

### v9 — closed-loop pinch depth (one-sided)
Hypothesis: from the v8 selection run, **depth is the discriminator, not lateral
aim**. Every bite that held had the fingertips 0.0199–0.0207 below the rim; every
air close was at 0.0135–0.0183. Lateral offsets from R_fit−0.006 to R_fit−0.015
both bit, so that window is wide. The descent undershoots its command by a
seed-dependent 0.004–0.010.
Evidence: 9/10 on 51,53,55,57,58,59,61,63,64,65. Seeds 58 and 64 — the two that
had been closing on air — both fixed. Seed 57 regressed: commanding a deeper
descent *and* keeping the z term of DESC_BIAS double-counted the undershoot, the
fingertips went 0.0264 deep, and the over-deep bite let the bowl rotate in the
jaws mid-carry (width grew 0.0044 → 0.0170).
Verdict: right variable, one-sided correction is not enough.

### v10 — two-sided depth regulation  ← **FROZEN**
Hypothesis: measure the depth actually achieved and correct it into 0.019–0.025
from either side; halve the z pre-compensation so the double-count goes away.
Evidence: 9/10 on the probe; **14/15 on the full 15 debug seeds**. Seed 57 fixed,
seeds 58/64 keep v9's fix. Step margin improves everywhere (worst non-failing seed
625 steps against v8's 797).
Verdict: selected.

### v11 — gentler two-step lift, shorter settle
Hypothesis: seed 58 bites (0.0053, effort 3.0) and arrives empty at the top of the
lift, so a shorter first lift step and less post-grip squeeze time should keep it.
Evidence: 8/10 — worse. Seed 58 unchanged, and seed 57 regressed: the 0.4 s settle
it had been given is load-bearing, the grip needs that time to establish.
Verdict: rejected, reverted to v10.

## Mechanism gap (seed 58, the one seed v10 does not solve)

Falsifiable statement: **the rim pinch has no purchase margin at the far edge of
the arm's reach.** Seed 58 puts the bowl at y = −0.3099, 0.010 further from the
base than any other debug seed. There v10 perceives it correctly (circle fit
centre (0.032,−0.3099), R 0.0520, rms 0.0029), lands the fingertips 0.0197 below
the rim — squarely inside the proven 0.019–0.025 window — and closes on the shell
at width 0.0053 with effort 3.0, a real bite. The bowl is gone by the top of the
lift (width 0.0030, effort 0.05).

The receipt for *why* is in the eef during the close: on seed 58 the hand is driven
**+0.022 in x and +0.021 in z** while the jaws shut, against −0.020 x and −0.007 z
on seed 51. The hand is riding up and out along the 66° outer cone instead of
settling onto the shell — at that extension the arm cannot hold station against
the closing force, so the bite that the width reports is a glancing one that the
lift then strips. Closing that gap needs a mechanism this API does not offer: a
force- or impedance-held station during the close (the wrist can be posed, but
`move` is a position servo that has already returned by the time the jaws move),
or a tilted jaw axis that meets the cone normally — which trades the slip for a
worse one, since gravity then acts almost entirely along the clamp plane.

Rejected explanations, each with its own receipt: it is not perception (the circle
fit is as good as on any seed), not depth (0.0197, in the good window), not lateral
aim (offset R_fit−0.012, and bites hold anywhere from R_fit−0.006 to R_fit−0.015),
not the step budget (884 of 1000, and the release executes), and not lift speed
(v11's gentler two-step lift left it unchanged).

## DECLARATION

- **Frozen version: v10.** `packs/c2clean_spa_bowl_on_stove_task_k0/program.py`
  md5 `ec8c737bdd5dc9d79a437cdee79d72a7` == `program_v10.py` (verified on cluster).
- **Selection receipt: 14/15** on the full debug split (seeds 51–65), directory
  `results/sel_c2clean_spa_bowl_on_stove_task_k0_v10`. Seed 58 is the only failure;
  every other seed succeeds, 11 of them in under 260 sim steps.
- **Per-version receipt chain** (probe subset 51,53,55,57,59,61,63,65 unless noted):
  v0 perception only · v1 0/4 · v2 0/4 · v3 **7/8** · v4 7/8 · v5 7/8 · v6 7/8 ·
  v7 **8/8** · v8 8/8, formal **14/15** (`sel_..._v8`) ·
  v9 9/10 (probe 51,53,55,57,58,59,61,63,64,65) · v10 9/10 same probe, formal
  **14/15** (`sel_..._v10`) · v11 8/10, rejected.
  v8 and v10 tie at 14/15 on the same seed; v10 is frozen because it regulates the
  grasp depth on both sides (v9 showed an unregulated descent can overshoot into a
  bite that rotates mid-carry) and leaves far more horizon margin — worst
  non-failing seed 625 sim steps against v8's 797.
- **PROVENANCE**: present as a top-level literal dict; every calibrated constant in
  the module is covered (checked by AST). No `.done` read anywhere; `tools/
  fewshot_run.py` never invoked; all runs under `tools/fair_run.py --split debug`
  on seeds 51–65 only.
