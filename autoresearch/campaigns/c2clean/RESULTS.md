# c2clean — results (2026-09-13). K=3 and K=0 re-acquired on all 60 LIBERO-PRO perturbation cells with the shared note file removed

Launched 2026-09-12 20:29, all 120 agents finished 2026-09-13 07:03 (8 concurrent, claude-opus-5, Claude Code 2.1.257); sealed evals seeds 1-50, coordinator-run, one per cell, 120/120 rc=0, md5 frozen before each eval (`eval_freeze.txt`). Two K=0 cells (wine_on_rack pos/task) ended their first session without a DECLARATION (agent waiting on a background notification a headless session never gets) and were resumed in place as fresh sessions, disclosed in their NOTES.md. Pre-registration: PREREGISTERED.md.

## Headline

| arm | clean draw (this campaign) | original draw (c2fix / c2k0, Aug 25-26) |
|---|---|---|
| K=3 | 2889/3000 = 96.3 % | 2874/3000 = 95.8 % |
| K=0 | 2846/3000 = 94.9 % | 2622/3000 = 87.4 % |
| K=3 − K=0, paired over 60 cells | **+43 episodes** (16 wins / 9 losses / 35 ties) | +252 episodes (20 / 9 / 31) |

## Pre-registered predictions

1. K=3 within 2 points of 95.8 %: **holds** (96.3). K=0 within 3 points of 87.4 %: **fails** (94.9, +7.5).
2. Eight-cell gap ≥ 30 on ≥ 6/8: **fails** (1/8).
3. Closing-width cells clean K=3 ≥ 40/50 on ≥ 3/4: **holds** (3/4; bowl_top_cabinet_task K=3 is 0/50).

## The eight mechanism-gap cells (clean K=3 / K=0 | original K=3 / K=0 | original K=1)

| cell | clean | original | K=1 |
|---|---|---|---|
| goal_open_middle_drawer_task | 50 / 0 | 50 / 0 | 49 |
| goal_put_wine_on_rack_task | 50 / 49 | 50 / 0 | 50 |
| goal_put_bowl_top_cabinet_task | 0 / 50 | 43 / 1 | 50 |
| goal_open_middle_drawer_pos | 35 / 22 | 0 / 7 | 50 |
| spa_bowl_cookie_box_pos | 46 / 50 | 45 / 11 | 50 |
| spa_bowl_on_cookie_box_task | 50 / 44 | 45 / 30 | 49 |
| spa_bowl_top_drawer_cabinet_pos | 50 / 48 | 47 / 40 | 43 |
| goal_put_bowl_on_stove_task | 50 / 42 | 50 / 3 | 5 |

Only `goal_open_middle_drawer_task` replicates (50 / 0 in both draws). Every other original gap either closes because clean K=0 solved it (wine_on_rack 49, bowl_top_cabinet 50, bowl_cookie_box 50, bowl_on_stove 42) or was itself a low K=3 draw (drawer_pos 0 → 35). `bowl_top_cabinet_task` reverses completely: clean K=3 froze at 0/15 after seven versions, clean K=0 reached 15/15 at v23.

## Effort

| arm | median h/cell | max | total agent-h |
|---|---|---|---|
| K=0 original | 1.38 | 5.6 | 116 |
| K=0 clean | 0.57 | 8.5 | 58 |
| K=3 original | 0.45 | 1.9 | 33 |
| K=3 clean | 0.51 | 2.2 | 37 |

Clean K=0 used 63 % less agent time than the original K=0 and scored 7.5 points higher.

## Two explanations, not separable with this data

1. **The shared file hurt K=0.** Five of the eight original K=0 gap-cell NOTES cite the file's entries repeatedly (bowl_on_stove 7×, bowl_top_cabinet 6×), and three of its seven entries describe how to read a demonstration the K=0 agent does not have. The time/score pattern (longer and worse with the file) fits a misleading prior.
2. **The harness changed.** Same model id (claude-opus-5) but Claude Code 2.1.232 → 2.1.257 between the draws; system prompt and tool behaviour may differ. A rerun of K=0 with the file under 2.1.257 would separate the two (60 cells, ~45 agent-h).

## What survives

* The contract and the system: both arms above 94 % on the sixty perturbation cells, against 89.3 % for the ASPIRE rerun.
* `open_middle_drawer_task` (0 vs 50 in both draws) and the LIBERO-90 `open_bottom_drawer` rule: demonstrations are necessary where the default pinch is geometrically impossible AND fails silently.
* K=1 ≈ K=3 (original draw; K=1 was not re-acquired).

## What does not

* "Demonstrations raise perturbation success from 87.4 % to 95.8 %" and "the effect lives in eight cells". Under the clean protocol the paired difference is +43 episodes of 3000, inside draw variance, and the eight-cell block does not replicate.

## Per-cell table (clean K=3 / K=0 | original K=3 / K=0 / K=1)

| cell | clean | original |
|---|---|---|
| goal_open_middle_drawer_pos | 35 / 22 | 0 / 7 / 50 |
| goal_open_middle_drawer_task | 50 / 0 | 50 / 0 / 49 |
| goal_open_top_drawer_put_bowl_pos | 50 / 50 | 50 / 48 / 50 |
| goal_open_top_drawer_put_bowl_task | 50 / 49 | 49 / 50 / 45 |
| goal_push_plate_front_stove_pos | 50 / 50 | 50 / 49 / 50 |
| goal_push_plate_front_stove_task | 50 / 49 | 50 / 50 / 50 |
| goal_put_bowl_on_plate_pos | 47 / 48 | 49 / 50 / 50 |
| goal_put_bowl_on_plate_task | 50 / 50 | 50 / 50 / 50 |
| goal_put_bowl_on_stove_pos | 49 / 50 | 49 / 42 / 49 |
| goal_put_bowl_on_stove_task | 50 / 42 | 50 / 3 / 5 |
| goal_put_bowl_top_cabinet_pos | 50 / 50 | 50 / 50 / 50 |
| goal_put_bowl_top_cabinet_task | 0 / 50 | 43 / 1 / 50 |
| goal_put_cream_cheese_in_bowl_pos | 46 / 50 | 46 / 48 / 49 |
| goal_put_cream_cheese_in_bowl_task | 50 / 44 | 44 / 44 / 50 |
| goal_put_wine_on_rack_pos | 38 / 40 | 46 / 46 / 44 |
| goal_put_wine_on_rack_task | 50 / 49 | 50 / 0 / 50 |
| goal_put_wine_top_cabinet_pos | 50 / 50 | 48 / 48 / 50 |
| goal_put_wine_top_cabinet_task | 50 / 50 | 50 / 50 / 50 |
| goal_turn_on_stove_pos | 50 / 50 | 45 / 50 / 50 |
| goal_turn_on_stove_task | 50 / 50 | 50 / 48 / 50 |
| obj_alphabet_soup_pos | 50 / 50 | 50 / 50 / 50 |
| obj_alphabet_soup_task | 50 / 50 | 50 / 50 / 50 |
| obj_bbq_sauce_pos | 50 / 50 | 50 / 50 / 50 |
| obj_bbq_sauce_task | 50 / 50 | 50 / 50 / 50 |
| obj_butter_pos | 50 / 50 | 50 / 50 / 50 |
| obj_butter_task | 50 / 50 | 50 / 50 / 50 |
| obj_chocolate_pudding_pos | 50 / 50 | 50 / 50 / 50 |
| obj_chocolate_pudding_task | 50 / 50 | 50 / 50 / 50 |
| obj_cream_cheese_pos | 50 / 50 | 50 / 50 / 50 |
| obj_cream_cheese_task | 50 / 50 | 50 / 50 / 50 |
| obj_ketchup_pos | 50 / 50 | 50 / 50 / 50 |
| obj_ketchup_task | 50 / 50 | 50 / 50 / 50 |
| obj_milk_pos | 50 / 48 | 50 / 50 / 50 |
| obj_milk_task | 50 / 50 | 50 / 50 / 50 |
| obj_orange_juice_pos | 50 / 50 | 50 / 50 / 50 |
| obj_orange_juice_task | 50 / 50 | 50 / 50 / 50 |
| obj_salad_dressing_pos | 50 / 50 | 50 / 50 / 50 |
| obj_salad_dressing_task | 50 / 50 | 50 / 50 / 50 |
| obj_tomato_sauce_pos | 50 / 50 | 50 / 50 / 50 |
| obj_tomato_sauce_task | 50 / 50 | 50 / 47 / 50 |
| spa_bowl_between_pos | 49 / 50 | 50 / 50 / 50 |
| spa_bowl_between_task | 46 / 46 | 45 / 48 / 46 |
| spa_bowl_cookie_box_pos | 46 / 50 | 45 / 11 / 50 |
| spa_bowl_cookie_box_task | 50 / 50 | 50 / 49 / 50 |
| spa_bowl_next_to_plate_pos | 50 / 50 | 48 / 50 / 50 |
| spa_bowl_next_to_plate_task | 50 / 50 | 50 / 50 / 50 |
| spa_bowl_next_to_ramekin_pos | 49 / 50 | 47 / 45 / 48 |
| spa_bowl_next_to_ramekin_task | 49 / 46 | 49 / 39 / 46 |
| spa_bowl_on_cookie_box_pos | 50 / 47 | 50 / 50 / 50 |
| spa_bowl_on_cookie_box_task | 50 / 44 | 45 / 30 / 49 |
| spa_bowl_on_ramekin_pos | 50 / 45 | 50 / 47 / 50 |
| spa_bowl_on_ramekin_task | 50 / 50 | 50 / 49 / 50 |
| spa_bowl_on_stove_pos | 50 / 50 | 50 / 50 / 50 |
| spa_bowl_on_stove_task | 50 / 49 | 50 / 46 / 43 |
| spa_bowl_on_wooden_cabinet_pos | 45 / 39 | 33 / 45 / 50 |
| spa_bowl_on_wooden_cabinet_task | 50 / 50 | 50 / 50 / 50 |
| spa_bowl_table_center_pos | 50 / 50 | 50 / 49 / 49 |
| spa_bowl_table_center_task | 42 / 47 | 46 / 47 / 47 |
| spa_bowl_top_drawer_cabinet_pos | 50 / 48 | 47 / 40 / 43 |
| spa_bowl_top_drawer_cabinet_task | 48 / 44 | 50 / 46 / 50 |

## Efficiency metrics beyond success rate (2026-09-14, `metrics.py`, per-cell JSON in `metrics_per_cell.json`)

Development-band runs from the cluster (fs_/sel_ dirs: episodes, successes, ok.gif mtimes), agent sessions from WALLCLOCK.md (per-session durations, resume gaps excluded), tokens and cost from each session's result event. 'v1' = the agent's first formally probed program.

| metric | clean k3 | clean k0 | orig k3 | orig k0 |
|---|---|---|---|---|
| v1 dev success fraction, median | 0.00 | 0.00 | 0.00 | 0.00 |
| time to first dev success (h), median | 0.24 | 0.27 | 0.21 | 0.57 |
| dev episodes to first success, median | 12 | 16 | 12 | 20 |
| version of first success, median | 2 | 4 | 2 | 4 |
| development episodes, median | 46 | 53 | 42 | 53 |
| program versions, median | 4 | 6 | 3 | 6 |
| agent wall-clock (h), median | 0.51 | 0.55 | 0.43 | 0.52 |
| assistant turns, median | 114 | 116 | 89 | 104 |
| output tokens (k), median | 66 | 69 | 60 | 70 |
| cache-read tokens (M), median | 6.5 | 6.5 | 4.2 | 5.0 |
| cost (USD), median | 6.15 | 6.41 | 4.78 | 5.59 |
| agent wall-clock (h), total | 36.55 | 43.77 | 32.46 | 45.66 |
| development episodes, total | 3009 | 3504 | 2855 | 3731 |
| output tokens (k), total | 4756 | 5387 | 4253 | 5469 |
| cost (USD), total | 466.42 | 508.08 | 353.60 | 452.15 |
| cells whose v1 already succeeded on some seed | 29 | 1 | 19 | 6 |
| cells with no dev success | 1 | 1 | 1 | 3 |

paired clean K=3 vs K=0 (per cell, K=0 minus K=3): wins = K=3 cheaper
  v1 dev success fraction              median diff   0.00   K=3 cheaper 0 / K=0 cheaper 29 / tie 30  (n=59)
  time to first dev success (h)        median diff   0.06   K=3 cheaper 39 / K=0 cheaper 19 / tie 0  (n=58)
  dev episodes to first success        median diff      4   K=3 cheaper 35 / K=0 cheaper 21 / tie 2  (n=58)
  version of first success             median diff      2   K=3 cheaper 45 / K=0 cheaper 7 / tie 6  (n=58)
  development episodes                 median diff      4   K=3 cheaper 32 / K=0 cheaper 26 / tie 2  (n=60)
  program versions                     median diff      2   K=3 cheaper 44 / K=0 cheaper 10 / tie 6  (n=60)
  agent wall-clock (h)                 median diff   0.10   K=3 cheaper 40 / K=0 cheaper 20 / tie 0  (n=60)
  assistant turns                      median diff      0   K=3 cheaper 30 / K=0 cheaper 26 / tie 4  (n=60)
  output tokens (k)                    median diff      5   K=3 cheaper 37 / K=0 cheaper 23 / tie 0  (n=60)
  cache-read tokens (M)                median diff    0.0   K=3 cheaper 31 / K=0 cheaper 29 / tie 0  (n=60)
  cost (USD)                           median diff   0.26   K=3 cheaper 33 / K=0 cheaper 27 / tie 0  (n=60)
