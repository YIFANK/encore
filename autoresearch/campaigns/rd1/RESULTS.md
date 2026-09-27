# rd1 — Encore on RoboDojo (10 tasks x K=3/K=0), sealed results

COMPLETE 2026-09-17: all 19 cells declared and evaluated (seeds 1-50 of the official eval pools, one blind draw, frozen md5, on-box scheduler). Development: one Claude Code (opus-5) session per cell, debug band 51-65, no note file, no cross-cell sharing; sessions cut by network drops were resumed in place (resume_worker.sh).

| task | K=3 success | K=3 mean score | K=0 success | K=0 mean score |
|---|---|---|---|---|
| build_tower | 36/50 | 0.79 | 0/50 | 0.08 |
| put_bottles_into_dustbin | 15/50 | 0.54 | 0/50 | 0.03 |
| make_kong | 36/50 | 0.72 | 0/50 | 0.00 |
| classify_objects | 8/50 (6 unstable) | 0.25 | 0/50 (6 unstable) | 0.02 |
| organize_table | 0/50 | 0.25 | 0/50 | 0.20 |
| arrange_largest_number | 0/50 | 0.25 | 0/50 | 0.03 |
| pack_objects_into_box | 0/50 (1 unstable) | 0.02 | 0/50 (1 unstable) | 0.00 |
| imitate_sorting_sequence | 0/50 (3 unstable) | 0.04 | 0/50 (3 unstable) | 0.01 |
| fold_clothes | 0/50 | 0.00 | 0/50 | 0.00 |
| classify_objects_by_language | no K=3 arm (no demonstration data) | — | 0/50 | 0.08 |

**Totals:** K=3 95/450 over 9 tasks (21%); K=0 0/450 over the same 9 tasks (0%), 0/500 over 10. Mean partial score K=3 0.32 vs K=0 0.04. Development receipts (debug band) predicted every sealed outcome in direction: build_tower 9/15 -> 36/50, make_kong 13/15 -> 36/50, classify 4/15 -> 8/50, bottles 3/15 -> 15/50; every 0/15 cell -> 0/50.

## Reading

- Every success is K=3: build_tower 36/50, make_kong 36/50, put_bottles 15/50, classify_objects 8/50; the same four tasks are 0/50 at K=0. In each the K=0 agent solved the manipulation but not the goal (build_tower K=0 worker: "the manipulation is solved; the target structure is not known"; make_kong K=0 never found which tile to expose). The K=3 workers read the goal off the pack's keyframe images. This is the RoboDojo counterpart of LIBERO's drawer_task: demonstrations are necessary where the sentence underspecifies the goal.
- Where success is 0 on both arms, the partial score still separates them (arrange 0.25 vs 0.03, organize 0.25 vs 0.21, bottles 0.54 vs 0.03).
- The Astra report's own numbers on this benchmark: Direct 26% SR (RGB-only, 1.13 B tokens per 50 episodes), hybrid pi0.5+Astra 48%. Encore's frozen programs cost zero run-time tokens; the comparison is cheap-frozen-program vs. expensive-online-VLM, on different task subsets, so quote task-by-task, not as one number.
- Every sealed number matches its cell's development receipt in direction (bottles K=3 3/15 -> 15/50; build_tower K=3 9/15 -> 36/50; all 0/15 cells -> 0/50).

## Harness incidents (all fixed; voided runs kept under results/eval_rd_*.void_*)

1. imitate_sorting_sequence loads a support-arm trajectory (Assets/Traj/<task>/<seed>/<layout>.pkl) by band name; the band copy did not mirror it, so both workers and the first sealed eval had zero executed episodes. Fixed (mirror + cleanup); both workers rebuilt and rerun.
2. RoboDojo _task.yml caps eval_nums at 25 for arrange, fold_clothes, pack_objects (and many others); eval_client takes min(EVAL_NUM, cap), so 50-layout bands were truncated to 25 and published as 0/50. Fixed in pipeline_utils (EVAL_NUM overrides); runner now refuses to publish a sealed result with holes beyond the unstable-layout count.
3. /mnt/data ENOSPC (co-tenant) killed the first batch; evals now run from an on-box nohup scheduler with a free-space guard (fair_run_robodojo writes results once, never appends).
4. The eval gate matched only `## DECLARATION`; four cells declared under `# DECLARATION` and were skipped for a day (build_tower_k3 among them). Gate regex widened.
5. Mac network drops (API ENOTFOUND) ended six worker sessions mid-loop; each resumed in place with a coordinator nudge (resume_worker.sh), same session id.
6. Unstable physics layouts (classify_objects 6, pack 1 each, imitate_k0 3) get no verdict from the simulator; counted as failures here, reported per cell.

Files: sealed_results.json, sealed_table.md, run_eval_onbox.sh, resume_worker.sh, PREREGISTERED.md, workers/*/NOTES.md.
