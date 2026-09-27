# rd2 — RoboDojo, contact and goal tasks, three demonstration arms

Written 2026-09-18, before any worker was launched.

**Question.** rd1 showed demonstrations are necessary where the sentence underspecifies the
GOAL (build_tower, make_kong, classify_objects) and once where it underspecifies the
ACTION (put_bottles: handover). rd2 asks which part of a demonstration carries the help,
on tasks picked to separate the two: what the images show (goal, object identity, order)
versus what the numbers show (end-effector poses, gripper commands, 25 Hz actions).
Concurrent work (GPT-Policy, arXiv 2609.19138) reports on a real robot, 3 trials per
condition, that action records add over video on contact-sensitive tasks (cap unscrewing,
plug reinsertion); rd2 is the sealed, 12-task, frozen-program version of that comparison.

**Arms (per task).** k3 = K=3 full pack (keyframe images + poses + gripper + actions);
vis = the same three demonstrations reduced to keyframe images only (no number of any
kind); k0 = no pack. Same brief otherwise, same harness, same model (claude-opus-5 via
Claude Code), no note file, nothing crosses cells. Debug band = episodes 51-65 (pool 2),
sealed band = episodes 1..20 of pool 0 (first pass; may be extended to 1..50 with the same
frozen programs, which is a superset, never a redraw).

**Tasks (12).** Contact / mechanism: fasten_screws, plug_in_charger, insert_key,
insert_tubes, hang_mugs, make_toast, store_laptop_and_headphones, press_by_number.
Goal-image analogues: push_T, swap_T, stack_blocks (align_blocks was the first choice; the public dataset has no demonstrations for it, found before launch, so it was swapped for stack_blocks). Interaction: play_tic_tac_toe.

**Predictions.**
1. Contact group: k3 > vis on summed successes, by at least 15% of episodes; vis >= k0.
2. Goal group: vis within 10% of k3; both above k0.
3. k0 scores 0 on at least 6 of the 12 tasks.
4. At least 3 tasks are 0 on all three arms (program-infeasible here: candidates
   hang_mugs, fasten_screws, play_tic_tac_toe).
Any of these failing is reported as such.

**Harness.** tools/fair_run_robodojo.py (--eval-n 20), on-box eval per cell
(rd2_eval_one.sh), scheduler with automatic outage resumes (run.sh). Incidents go in
RESULTS.md.
