# rd1 — Encore on RoboDojo (Isaac Sim, ARX X5 bimanual). Pre-registered 2026-09-15, before any cell ran.

## What this is

The first Encore campaign outside LIBERO/robosuite: RoboDojo's ten-task subset used
by the "GPT-6 Astra as an Embodied Policy" report (anonymous-report-421.github.io),
run under our fair protocol with the same inner model (claude-opus-5), the same
brief structure, and one frozen program per cell. Two arms per task:

* **K=3**: three teleoperated RoboDojo episodes distilled by `tools/fair_pack_robodojo.py`
  (three views per keyframe, both arms' poses, gripper openness, 25 Hz actions).
* **K=0**: intent sentence only.

`classify_objects_by_language` has no released demonstrations, so it runs K=0 only
(19 cells). The `_random` generalization variants are not run in this campaign.

## Protocol

* Development band = official layout files 0-14 of seed dir 2 (episodes 51-65);
  sealed band = layout files 0-49 of seed dirs 0 then 1 (episodes 1-50), copied and
  renumbered per run so RoboDojo's own SeedManager executes exactly that list.
* Success = RoboDojo's reward manager at episode end (`_result.json`), never visible
  to the program; the program ends the episode when it returns (scored as if the step
  budget had run out). Score (partial credit) is recorded alongside.
* Depth, intrinsics, extrinsics and gripper joint states are enabled in a config
  variant (`arx_x5_encore`); the only upstream code change is a five-line observation
  patch adding gripper joint states. Perception at run time: RGB-D + a VLM for
  ground/vqa (60 calls per episode). No object poses, no simulator state.
* Fresh agent per cell, 250-turn cap, mechanism-gap stop allowed, argmax selection.

## Predictions

1. **Programs beat the token-in-the-loop policy on the pick-and-place subset.** On
   the five tasks that are pick-and-place with a semantic choice (put_bottles,
   classify_objects, classify_objects_by_language, pack_objects_into_box,
   arrange_largest_number) the K=3 frozen programs average ≥ 40 % success on the
   sealed band. The report's numbers on the same tasks: Astra Direct 26 % mean SR
   over ten tasks, hybrid 48 %.
2. **The deformable and fine-motor tasks fail.** fold_clothes and make_kong score
   ≤ 10 % for both arms; build_tower ≤ 20 %. These are the tasks where a program's
   contact model is weakest, and the report's Direct arm scored 0-12 on them too.
3. **Demonstrations matter more here than on LIBERO-PRO.** Paired over the nine
   K=3/K=0 pairs, K=3 wins by ≥ 15 points on at least four tasks; the mechanism
   we expect: bimanual hand-over ordering and grasp sites for objects whose
   graspable part is not the visual centroid.
4. **Cost.** Development per cell stays under $20 and 3 hours of agent time; run-time
   tokens are zero by construction.

Failure directions: if (1) fails the honest headline is "programs are competitive
with, not better than, a VLM policy on this benchmark". If (3) fails, the
demonstration claim narrows further to the LIBERO-90 rule. All 19 cells are
reported regardless.

## Not decided in advance

The comparison to the report is task-level only: their five trials per task on a
different layout mix are not a paired comparison with our fifty sealed layouts.

## Harness incidents (logged as they happened)

* 2026-09-14 15:09-16:09 EDT: first launch died in a coordinator network outage;
  13 cells relaunched fresh, 6 resumed in place (noted in their NOTES.md).
* A runner race (shared layout band dir per episode list) voided some early probes
  ("missing" judgements); fixed before the relaunch.
* Until 2026-09-15 10:40 CST, `api.vqa`/`api.ground` failed on every call (wrong
  logger object, then no network route to the VLM from the cluster). Cells that ran
  before then developed without a VLM; an addendum was appended to every workspace.
* `frame.t_base_cam` is delivered in the simulator's OpenGL convention while
  `deproject` assumes OpenCV; five of the first seven agents discovered and
  corrected this themselves. Left as is for consistency within the campaign,
  documented in the addendum; `api.ground` corrects it coordinator-side.
