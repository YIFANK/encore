# l90abl — LIBERO-90 articulated-fixture modality ablation. Pre-registered 2026-09-08, before any cell ran.

## Why these tasks
The robosuite ablation (rsabl) refuted "the action channel matters wherever the
manipulation type is not visible in the image": on Door the K=0 agent found the
latch turn by search because the wrong strategy's failure was diagnosable from
its own sensors (a verified hold that would not pull). The refined condition is:
**the action channel is load-bearing when the manipulation type is not visible
AND the wrong strategy fails silently in the proximal signal.** LIBERO's drawer
family is the case we have (K=0 0/50 across 17 versions: the jaws reach the bar
and drag air, and nothing says why). LIBERO-90 has more of the same fixture
family plus matched controls on the same fixtures.

## Cells (8 tasks × 3 arms = 24)
Arms as in rsabl: `k3` (full K=3 pack from the LIBERO-90 human demos, first 3),
`vis` (keyframe RGB + language only), `k0` (no pack). fair_run, `--bddl`, debug
seeds 51-65, sealed eval 1-50 coordinator-run, fresh agent per cell, laws from
zero, PROVENANCE required.

**Open group (prediction: k3 > vis ≈ k0):**
open_microwave (S7), open_top_drawer_s1 (S1), open_top_drawer_s2 (S2),
open_bottom_drawer (S1). Engaging a bar handle with a hook/press attitude that
the image does not show; a mis-engaged pinch reaches the bar and pulls air with
no sensor telling the agent why.

**Control group (prediction: k3 ≈ vis ≈ k0):**
close_microwave (S6), close_bottom_drawer (S4), close_top_drawer (S5) — closing
is a push on a visible door/front, diagnosable by the fixture's motion;
turn_on_stove (S3) — K=0 already solved the goal-suite stove at 50/50 (c2k0), so
a knob turn is diagnosable here.

## Predictions (falsifiable)
1. On the open group, k3 beats k0 by ≥ 15 episodes per cell in at least 3 of 4
   cells, and vis is within 10 of k0 in at least 3 of 4.
2. On the control group, all three arms are within 10 episodes of each other in
   at least 3 of 4 cells.
3. If instead k0 solves the open group too, the LIBERO drawer result was
   specific to that cabinet/handle, and the claim should be dropped from the
   paper's general statement and kept as a case study.

## Analysis, fixed in advance
Per-cell successes/50 on the sealed band; the two group tables; per-cell
differences k3−vis and k3−k0. No pooling across groups, no test at n=4; the
qualitative half is what each arm did at the handle, from the agents' own NOTES.

## Known limits
One draw per arm; human LIBERO demos (not synthetic) — the K=3 pack here is the
same kind as the paper's. Packs are built with the task's own stock language.
