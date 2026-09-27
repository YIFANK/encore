# c2k1clean / spa_bowl_between_pos_k1 — working notes

Task: "pick up the black bowl between the plate and the ramekin and place it on
the plate". K=1 pack, `_pos` perturbation (object placements move across seeds).
Runner: `tools/fair_run.py` only. Debug seeds 51-65.

## Pack reading (inputs)

`pack.json`: one demo, 98 steps, stride 10, keyframes at t=0/36/85/97.

| t | ee (x,y,z) | gripper_cmd | gripper_state |
|---|---|---|---|
| 0 | -0.2129, -0.0021, 1.1760 | -1 (open) | 0.0362/-0.0362 |
| 36 | -0.0402, 0.1762, 0.9444 | +1 (close) | 0.0393/-0.0393 |
| 85 | 0.0579, 0.1548, 0.9367 | -1 (open) | 0.0064/-0.0061 |
| 97 | 0.0657, 0.1504, 1.0216 | -1 | 0.0385/-0.0384 |

Readings:
- The close command is issued at ee z = **0.9444**; the open command at
  ee z = **0.9367**. Carry apex (ee_path t=60/70) z ~ 1.01-1.04.
- The held gripper state is 0.0064/-0.0061 → **~12.5 mm of jaw span occupied**
  while carrying. An empty close would read ~0. So the demo does not engulf the
  bowl: it **pinches a ~12 mm wall** (the rim lip).
- Demo keyframe t=0 (128 px agentview) shows: dark cabinet (left), stove,
  a brown box, a light-rimmed plate, and three round vessels — one lighter and
  smaller (ramekin) and two dark (black bowls). The bowl the demo grasps is the
  middle one, which sits between the plate and the lighter vessel. That is the
  literal reading of "between the plate and the ramekin".

## v1 — perception probe (no motion), seeds 51,53,55,57,59,61,63,65

Receipt: `results/fs_c2k1clean_spa_bowl_between_pos_k1_v1`, 0/8 (no motion, as
designed — the run existed only to measure the scene).

Measured, identical across the seeds inspected:
- `cam_high` K = fx=fy=618.04, c=(256,256); camera at base (0.659, 0, 1.610).
- Table plane z = **0.9014**; gripper open width at reset = **0.0778 m**.
- Cluster table (ep51): plate h=0.019 r=0.062 grey(152,143,141); brown box
  h=0.019 r=0.045 (93,66,47); two vessels h=**0.0507** r=0.054 rgb~(108,109,106)
  and (101,102,100); one vessel h=**0.0424** r=0.038 rgb~(115,116,117),
  brightest — the ramekin. Stove h=0.059 very dark at x=-0.40.
- So the discriminants are clean: plate = flat + grey (channel spread < 25);
  brown box = flat but saturated (spread ~46); ramekin = the vessel with the
  lowest top (42 mm vs 51 mm) and the brightest top.
- `_pos` really does permute: in seeds 51-55 the plate and the brown box sit at
  the pixel positions that the demo keyframe has swapped.
- Controller: `POS_TOL = 0.012` — a bare `move()` stops anywhere within 12 mm of
  the commanded point, so fine placement needs bias-cancelled re-commands.

## v2 — classification + rim-pinch direction ladder (calibration)
Hypothesis: the target is the black bowl nearest the plate↔ramekin segment; a
radial rim pinch at the fitted rim circle holds it; the correct radial direction
is unknown, so try +x, (+x,-y), (+x,+y) and let the gripper's own effort/width
report which one holds.

Receipt: `results/fs_c2k1clean_spa_bowl_between_pos_k1_v2`, **0/8**.
Evidence: every close returned width 0.0010 / effort 0.05 — air. Positioning was
not the problem (bias-cancelled residuals 2-5 mm). Two things were unknown: how
far below the reported eef the fingertips are, and where the bowl's wall really
is. Verdict: rejected; the ladder was aimed at a rim radius taken from a circle
fit over the top 12 mm of the cloud, which is not the outer wall.

## v3 — contact profiling (closed jaws driven down until blocked)
Hypothesis: the fingertips are not at the eef z, so the ladder descended above
(or below) the wall.
Receipt: `results/fs_c2k1clean_spa_bowl_between_pos_k1_v3`, seeds 51,53.
Evidence: **closed fingertips stop at eef z = table + 0.0078** — so the demo's
close at table + 0.0436 puts the tips ~36 mm up, mid-wall, as intended. A probe
at r=0.060 from the fitted centre reached the table (outside the bowl); one at
r=0.050 slid 26 mm sideways — i.e. **a fingertip landing on the outer wall
shoves the bowl**, which is what spoiled attempts 2 and 3 of the v2 ladder.
Second, unplanned result: after ~800 sim steps the arm stopped responding while
residuals stayed large — **the episode horizon is finite and a blocked move
burns its whole `seconds` budget**. Verdict: calibration succeeded; budget
discipline added (never command an unreachable pose; `frozen` detector).

## v4 — zero-motion shape probe
Receipt: `results/fs_c2k1clean_spa_bowl_between_pos_k1_v4`, seeds 51-54.
Cross-sections through the target bowl (height in mm vs offset from centre):
`±55:50 ±50:44 ±45:36 ±40:23 ±35:14 ±30:11 ±25:9`. So the bowl is a disc of
**outer radius 55 mm**, rim top 51 mm, **interior floor 9 mm**, wall rising
between r=35 and r=55 mm. Its bbox centre and the rim-circle fit agree
(-0.053, 0.200); the point-cloud centroid (-0.067, 0.204) is biased toward the
camera and must not be used.
Decisive consequence: the jaws open 77.8 mm, **less than the bowl's 110 mm**, so
the bowl cannot be engulfed. Projecting the demo's grasp (pack t=36 ee) through
the measured camera matrix lands at 128 px (92.3, 76.7) — on the middle vessel,
25.8 mm from its centre. That is a **wall pinch with one finger inside the bowl
and one outside**, at s ≈ 0.47 R, and the tool frame's finger axis is world y
(R0 col1 = (0,-1,0)), so the offset must be along ±y.

## v5 — first full attempt
Receipt: `results/fs_c2k1clean_spa_bowl_between_pos_k1_v5`, **4/8**
(51,53,59,63 true; 55,57,61,65 false), 5-11 moves per episode.
Evidence: where perception was clean the pinch is decisive — closed width
0.0069 at effort 3.00, held through the lift. All four failures were
perception: 1 cm blob clustering **fuses whatever touches**. On 55/57/61 the
brown box merged with the target bowl (blob radius 0.079 instead of 0.055,
rgb spread 20 instead of 2), moving the centre 2-3 cm; on 65 the ramekin merged
with a bowl, the fused blob was mistaken for the ramekin, and the **wrong
vessel** was carried to the plate ("placed", success False).
Verdict: mechanism right, centres wrong.

## v6 — ring-vote detection (FROZEN)
Hypothesis: a blob centroid is unusable when objects abut, but every vessel is a
circle of known radius, so voting (each rim point votes for centres one radius
away) recovers both centres from a fused blob.
Receipt (probe): `results/fs_c2k1clean_spa_bowl_between_pos_k1_v6`, **8/8**.
Receipt (selection, all 15 debug seeds):
`results/sel_c2k1clean_spa_bowl_between_pos_k1_v6`, **15/15**.
Evidence: on the four seeds v5 lost, the vote separates the target cleanly
(e.g. ep65 bowlvote (-0.040,0.200)=402 and (-0.180,0.320)=370, ramvote
(-0.205,0.210)=648 against a 349 runner-up), and every episode grasps on the
first attempt in 5 moves, effort 3.00, width ~0.007. The retry side and the
re-perceive path were never needed on the debug band.
Residual roughness (did not cost a seed): the final lowering is consistently
stopped 15-23 mm high with ~10 mm of lateral slide as the carried bowl meets the
plate; the release happens there and the bowl drops the last centimetre.

# DECLARATION

- Frozen version: **program_v6.py**, md5 `a144340ed151e75ceb6d338d696fab7e`,
  identical to `packs/c2k1clean_spa_bowl_between_pos_k1/program.py`
  (verified on the cluster).
- Selection receipt: **15/15** on the full 15 debug seeds (51-65),
  `results/sel_c2k1clean_spa_bowl_between_pos_k1_v6`.
- Receipt chain: v1 0/8 (perception only, by design) → v2 0/8 → v3 (2-seed
  calibration) → v4 (4-seed zero-motion shape probe) → v5 4/8 → v6 8/8 probe,
  15/15 selection.
- Version archive: `program_v1.py` … `program_v6.py` in the pack directory.
- PROVENANCE: present, 14 calibrated constants, every one sourced to a pack
  field or a debug-seed measurement; `allowed: True` throughout. Program
  contains no forbidden token and no `.done` attribute read (checked with the
  same AST rule the eval gate uses).
- Eval seeds 1-50 were never touched.
