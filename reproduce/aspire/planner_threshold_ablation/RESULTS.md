# aspire_abl results (stage 1 + heldout complete 2026-09-16)

Two single-factor ablations inside ASPIRE's own fix loop, 20 spatial cells (10 swap + 10 task), sealed seeds 1-50, one draw. P = ASPIRE without its grasp planner (plan_grasp / select_top_down_grasp removed from the allowed API); T = ASPIRE without the gripper-width thresholds line in the brief. Control A = plain ASPIRE v4 (2026-08-24, opus-5, CONVERGE=1), which covers 18 of the 20 cells.

| cell | A (control) | P (no planner) | T (no thresholds) | Encore K=0 | Encore K=3 |
|---|---|---|---|---|---|
| swap/between_the_plate_and_the_ramekin_and_place_it_on_the_p | 49 | 49 | 49 | 50 | 49 |
| swap/from_table_center_and_place_it_on_the_plate | 48 | 49 | 50 | 50 | 50 |
| swap/in_the_top_drawer_of_the_wooden_cabinet_and_place_it_on | 31 | 47 | 36 | 48 | 50 |
| swap/next_to_the_cookie_box_and_place_it_on_the_plate | 47 | 47 | 50 | 50 | 46 |
| swap/next_to_the_plate_and_place_it_on_the_plate | 50 | 50 | 50 | 50 | 50 |
| swap/next_to_the_ramekin_and_place_it_on_the_plate | 40 | 34 | 47 | 50 | 49 |
| swap/on_the_cookie_box_and_place_it_on_the_plate | 35 | 9 | 48 | 47 | 50 |
| swap/on_the_ramekin_and_place_it_on_the_plate | 50 | 44 | 29 | 45 | 50 |
| swap/on_the_stove_and_place_it_on_the_plate | 38 | 50 | 44 | 50 | 50 |
| swap/on_the_wooden_cabinet_and_place_it_on_the_plate | 30 | 50 | 36 | 39 | 45 |
| task/between_the_plate_and_the_ramekin_and_place_it_on_the_p | 19 | 43 | 48 | 46 | 46 |
| task/from_table_center_and_place_it_on_the_plate | 46 | 46 | 23 | 47 | 42 |
| task/in_the_top_drawer_of_the_wooden_cabinet_and_place_it_on | 18 | 45 | 43 | 44 | 48 |
| task/next_to_the_cookie_box_and_place_it_on_the_plate | 44 | 50 | 50 | 50 | 50 |
| task/next_to_the_plate_and_place_it_on_the_plate | 50 | 47 | 45 | 50 | 50 |
| task/next_to_the_ramekin_and_place_it_on_the_plate | 50 | 44 | 46 | 46 | 49 |
| task/on_the_cookie_box_and_place_it_on_the_plate | n/a | 21 | 50 | n/a | n/a |
| task/on_the_ramekin_and_place_it_on_the_plate | 50 | 50 | 35 | 50 | 50 |
| task/on_the_stove_and_place_it_on_the_plate | n/a | 50 | 40 | n/a | n/a |
| task/on_the_wooden_cabinet_and_place_it_on_the_plate | 42 | 46 | 50 | 50 | 50 |

**18 common cells (of 900):** A 737 | P 800 (+63) | T 779 (+42) | Encore K=0 862 | Encore K=3 874.
All 20 cells: P 871/1000, T 869/1000.

Per cell vs A: P better 8 / worse 5 / tie 5; T better 11 / worse 5 / tie 2.

## Against the preregistration

1. "P beats A on spatial by >= 40 trials (737 -> >= 777)": **confirmed**, 800 (+63).
2. "T within +-25 of A": **falsified upward**, 779 (+42). Removing the thresholds line also helped.

## Reading

Both ablations move ASPIRE toward Encore K=0 (862) without reaching it; the remaining gap on the same 18 cells is 62 (P) and 83 (T) trials. The biggest recoveries are the control's four worst cells, and they recover under BOTH ablations (between_task 19 -> 43/48; top_drawer_task 18 -> 45/43; top_drawer_swap 31 -> 47/36; on_wooden_cabinet_swap 30 -> 50/36). That pattern fits the hypothesis (a fixed strategy family the agent will not leave) but it is also what a date/harness drift would look like: the control ran on 2026-08-24, these on 2026-09-15, same model id claude-opus-5 but not the same Claude Code version. The per-cell scatter is large in both directions (P on_the_cookie_box_swap 35 -> 9; T on_the_ramekin_swap 50 -> 29).

**Before quoting +63/+42 as the ablation effect, rerun plain ASPIRE (A) on the four worst control cells at today's date** (4 cells, ~4 h). If A also recovers them, the effect is drift; if not, the ablation stands.

Files: recount_abl.json (per-cell), agent_{P,T}_*/STAGE2_RESULT, PREREGISTERED.md, armA_full58.json (control).