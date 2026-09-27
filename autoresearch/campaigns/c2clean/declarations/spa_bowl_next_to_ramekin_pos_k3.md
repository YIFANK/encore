# c2clean / spa_bowl_next_to_ramekin_pos_k3

Intent: "pick up the black bowl next to the ramekin and place it on the plate".
Runner: tools/fair_run.py only. Debug seeds 51-65. Pack = K=3 demos.

## What the pack says (read before any run)

* All three demos close the gripper with the **home wrist** (tool x ~ world +x,
  tool z straight down; the 12-15 deg tilt at the grasp keyframe is the same
  tilt the wrist starts with). No reorientation is used for the pick.
* Closing EEF z = 0.932 / 0.917 / 0.9215. Release EEF z = 0.945 / 0.9446 /
  0.9322. Carry apex ~1.07-1.10.
* Carried finger gap at the grasp keyframes = 0.0115 / 0.0125 / 0.0122 — a
  wall-thickness bite, i.e. a rim straddle, not a whole-bowl straddle (the
  jaws only open to 0.078 and the bowl is 0.11 across).
* Projecting the demo keyframes with the cam_high intrinsics/extrinsics
  measured on seed 51 (fx = 618.04 * 128/512, cx = cy = 64; verified by
  projecting the t=0 EEF onto the gripper in demo0_t0000.png):
  - demo0 target bowl centre deprojects to (-0.174, +0.296) at rim height;
    grasp EEF (-0.1575, +0.3512) -> **offset (+0.017, +0.055)**.
  - demo0 plate centre deprojects to (+0.054, +0.205); release EEFs give
    offsets (+0.035,+0.023) / (+0.008,+0.028) / (+0.021,+0.042).
  The release offset ~= the grasp offset: the pinched bowl hangs off the EEF
  by the grasp vector, so the release point is plate centre + that vector.
* The offset points away from the ramekin (bowl - ramekin = (0.022,0.129),
  unit (0.17,0.99); the demo offset unit is (0.29,0.96)).

## v1 — perception probe (seeds 51,53,55,57)

Hypothesis: the scene can be read from a top-down height map of the cam_high
cloud alone. Program: no motion, logs a 1 cm raster of max-z and the colour at
that cell over x[-0.45,0.40] x y[-0.45,0.45].

Evidence (all four seeds, heights above the table plane z0 = 0.9013):

| class | footprint | hmax | mean RGB |
|---|---|---|---|
| bowl (x2) | 0.11 x 0.11 | 50 mm | (127,123,102) |
| ramekin | 0.08 x 0.09 | 42 mm | (145,144,144) |
| plate | 0.13 x 0.14 | 18 mm | (147,139,135) |
| brown box | 0.08 x 0.06 | 18 mm | (90,55,33) |

Verdict: three height bands (>46 / 30-46 / 10-26 mm) plus one colour test each
separate every prop. Band-masking also un-fuses the ramekin from the target
bowl (they merge into one component on seed 57 at a single threshold). In these
seeds the target bowl, the ramekin and the plate barely move; the _pos
perturbation mainly relocates the second (distractor) bowl, so the *nearest to
the ramekin* rule is what actually picks the target.

## v2 — perceive + rim pinch + carry (FROZEN)

Hypothesis: target = the bowl-class cluster nearest the ramekin; pinch its rim
at 0.048 m from the centre along world y, sign away from the ramekin (the jaws
close along y with the home wrist); descend to z0+0.020 (the pack's closing
height over a table measured at 0.9013); carry at z0+0.18; release at plate
centre + the same offset, at z0+0.050 (the pack's release height).

Evidence, probe run `results/fs_c2clean_spa_bowl_next_to_ramekin_pos_k3_v2`
(seeds 51,53,55,57,59,61,63,65): **8/8 benchmark_success**. Per-episode
receipts: descend residual 0.007-0.012, closed gap 0.0087-0.0096 with effort
3.0 in every episode, carry residuals < 0.012, release EEF z = 0.958-0.961.
Post-episode re-perception finds only the distractor bowl still on the table.
The gif (ep59) shows a clean pick, carry and place.

Note on the receipts: the gap keeps decaying after the close (0.0088 -> 0.0045
in 5 of 8 episodes) and the harness's `effort` flag, which is just
`gap > HELD_MIN_GAP`, then reads 0.05 mid-carry even though the bowl is still
held and the place succeeds. Treat gap survival, not the effort flag, as the
hold receipt here.

Selection: see DECLARATION.

## Margin: how far the aim can be wrong

15/15 says nothing about how much slack the grasp has, so the frozen program was
re-run on seeds 51,55,59,63 with the rim offset displaced +-10 mm
(`program_m038.py`, `program_m058.py`):

| RIM_OFFSET | seeds 51,55,59,63 | closed gap |
|---|---|---|
| 0.038 | 4/4 | 0.0081-0.0091 |
| 0.048 (frozen) | 4/4 (8/8 on the probe set) | 0.0087-0.0096 |
| 0.058 | 4/4 | 0.0087-0.0110 |

Receipts: `results/fs_c2clean_spa_bowl_next_to_ramekin_pos_k3_m038`,
`..._m058`. The straddle self-centres over at least a +-10 mm band, so a rim
radius estimate that is a centimetre off still grasps.

## DECLARATION

* Frozen version: **v2**. `packs/c2clean_spa_bowl_next_to_ramekin_pos_k3/program.py`
  md5 `891ad3f118b44566afd4efc5b828fca9` == `program_v2.py` (identical files).
* Selection receipt (formal, full 15 debug seeds 51-65):
  **15/15 benchmark_success**, dir
  `results/sel_c2clean_spa_bowl_next_to_ramekin_pos_k3_v2`.
* Receipt chain:
  - v1 perception probe (no motion) — seeds 51,53,55,57, dir
    `results/fs_c2clean_spa_bowl_next_to_ramekin_pos_k3_v1`; produced the
    height/colour class table above.
  - v2 probe — seeds 51,53,55,57,59,61,63,65, **8/8**, dir
    `results/fs_c2clean_spa_bowl_next_to_ramekin_pos_k3_v2`.
  - v2 selection — seeds 51-65, **15/15**, dir
    `results/sel_c2clean_spa_bowl_next_to_ramekin_pos_k3_v2`.
  - margin probes m038 / m058 — 4/4 each (above).
* PROVENANCE: present in program.py, one entry per calibrated constant, every
  source either a pack field or a debug-seed measurement recorded above.
