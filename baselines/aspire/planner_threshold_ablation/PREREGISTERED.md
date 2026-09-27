# aspire_abl — what in ASPIRE's own loop loses the spatial suite? Pre-registered 2026-09-15, before any cell ran.

## Question

On the 20 LIBERO-PRO spatial cells, ASPIRE's released loop on opus-5 (our v4 rerun,
converge supervisor) scores 737/900; Encore K=0 under the clean protocol scores
862/900 with the same inner model. Both loops see the same seeds and predicate.
Which of ASPIRE's design choices carries the gap?

## Arms (single-factor, everything else = the v4 recipe verbatim)

* **P — no grasp planner.** `plan_grasp`, `plan_grasp_from_point_clouds` and
  `select_top_down_grasp` removed from the API registry and from the brief's
  ALLOWED list (the brief says so and points at masks / point clouds / OBB +
  solve_ik / goto_pose). Perception (SAM3, Molmo, OBB), the shared skill library,
  the thresholds, the converge supervisor: unchanged.
* **T — no thresholds in the brief.** The one line telling the agent what
  gripper width counts as grasped / marginal / air is replaced by "establish it from
  your own traces"; the (empty) per-object threshold table is removed from
  skills/grasp.md. Planner and everything else: unchanged.

Reference columns: ASPIRE v4 (planner + thresholds) 737/900; Encore K=0 clean
(neither) 862/900. Same 20 cells, seeds 51-65 dev / 1-50 sealed, opus-5,
supervise_stage1 with CONVERGE=1, coordinator-only Stage 2, code-hash audit.

## Predictions

1. **P beats A on spatial by ≥ 40 trials** (737 → ≥ 777 of 900): the top-down
   Contact-GraspNet grasp is the wrong family for bowls next to fixtures and in
   drawers, and the planner's presence keeps the agent from leaving it.
2. **T is within ± 25 trials of A**: the width thresholds are convenient, not
   decisive; an agent that must measure them does so in one probe.
3. Neither arm reaches Encore K=0's 862: the remaining gap is the verification
   contract (no runtime success bit, program-side checks) and the 15-seed formal
   selection, which this campaign does not vary.

Failure directions: if P ≤ A, the planner is not the mechanism and the paper's
"tools lock in a strategy" sentence is withdrawn; if T ≥ A + 40, the brief's
thresholds were misleading the agent, which is a different (and cheaper) claim.
