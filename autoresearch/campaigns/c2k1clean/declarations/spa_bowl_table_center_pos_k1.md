# c2k1clean / spa_bowl_table_center_pos_k1

Intent: "pick up the black bowl from table center and place it on the plate".
Runner: tools/fair_run.py only. Pack: K=1 demo (103 steps, 4 keyframes).

## Pack reading (the only task-specific input)

- `action_scale = [0.2958, 0.2041, 0.4735, 0.02, 0.0803, 0.0287, 1.0]`
- keyframes / ee_path6 (x,y,z,r,p,y,grip):
  - t0   ee (-0.2010, -0.0016, 1.1734) grip open (-1), width 2*0.0362
  - t35  ee (-0.0758,  0.0402, 0.9412) grip flips to CLOSE (+1)
  - lowest point of the descent: ee_path6[4] = (-0.0908, 0.0409, 0.9194)
  - t99  ee ( 0.0634,  0.2268, 0.9342) grip flips to OPEN (-1),
          gripper_state 0.0081/-0.0076 -> held width ~0.0157
  - t102 ee ( 0.0630,  0.2277, 0.9337)
- Keyframe images are 128x128 renders of the cam_high view.

## v1 -- perception probe (pixel clustering), seeds 51..65 odd

Receipt: `results/fs_c2k1clean_spa_bowl_table_center_pos_k1_v1` (0/8, no motion).

Measured, identical in every seed:
- `cam_high`: 512x512, K = fx=fy=618, c=(256,256);
  `t_base_cam` = [[0,0.628,-0.778,0.659],[1,0,0,0],[0,-0.778,-0.628,1.61]]
- **table plane z = 0.9020** in all 8 seeds.
- eef at reset = (-0.2085, 0.000, 1.1733), gripper width 0.0778.

Prop inventory (height above the 0.902 plane, top-down extent):
| prop | h | top z | extent | mean rgb |
|---|---|---|---|---|
| bowl (x2) | 0.049 | 0.951 | 0.111 x 0.111 | ~(107,108,105) |
| ramekin   | 0.041 | 0.943 | 0.087 x 0.088 | ~(115,116,117) |
| plate     | 0.018 | 0.920 | 0.136 x 0.136 | ~(153,143,141) |
| cookie box| 0.018 | 0.920 | 0.082 x 0.061 | ~(92,65,46) |
| cabinet+stove | 0.226 | 1.128 | huge | ~(66,64,63) |
| parked arm | 0.459 | 1.361 | 0.121 x 0.203 | (21,41,50) |

So **a top-z band isolates the bowls**: bowl 0.951 vs ramekin 0.943 vs
plate/box 0.920. Pixel clustering fused bowl+plate (ep51/57/59/61/63/65) and
bowl+cabinet (ep51/55), hence v2's top-down grid.

Demo bowl identified: the demo keyframe shows the grasped bowl at px ~(256,288);
ep53 has a bowl at px (259,289) = xy (-0.101, 0.005). The demo grasp xy
(-0.091, 0.041) is therefore ~0.037 out from that bowl's centre -- a rim/wall
grasp, not a centre grasp. Demo grasp z = 0.9194 = table + 0.0174.

`_pos` also PERMUTES identities across sites: the demo has the plate at
px~(390,375) and a bowl at px~(475,333); ep53 has a bowl at (396,376) and the
plate at (475,334).

## v2 -- perception probe #2 (top-down 5 mm max-height grid), seeds 51..65

Receipt: `results/fs_c2k1clean_spa_bowl_table_center_pos_k1_v2` (0/15, no motion).
Findings (all 15 seeds), with a fixed crop x,y in [-0.45,0.45]:
- table plane z = 0.9016, table-cell bbox centre `tcen` = (-0.087, 0.043+-0.002)
  in every seed -- a stable reference point.
- **exactly two bowls** (ext 0.110x0.110, ztop 0.952). One is effectively
  pinned at **x = -0.078, y in [-0.015, +0.012]** in all 15 seeds; the other
  roams: (0.068,0.195) (0.062,0.188) (0.108,0.188) (0.057,-0.083)
  (0.078,0.167) (0.073,0.195) (0.057,0.208) ... The pinned one is the site the
  demo grasps -> **"table center" = the bowl nearest `tcen`**
  (d_tcen 0.046-0.055 vs 0.12-0.24 for the other; 2.5-4x margin).
- the plate = the flat-band blob with 460-560 cells, ext 0.135, ztop 0.920,
  mean rgb ~(158,151,149); it sits near (0.005, 0.32) with ~+-0.03 jitter.
  The brown cookie box shares its height but has min channel ~45.
- a wide-crop re-run (x[-0.8,0.6], y[-0.8,0.8], dir `..._wide`, seeds 53/57/63)
  confirmed nothing relevant falls outside the narrow crop; the table really
  spans x[-0.633,0.272] y[-0.558,0.557].

## The geometric model (hypothesis -> evidence)

Hypothesis: the demo releases with the **bowl centre over the plate centre**.
Evidence: the bowl is held rigidly at `grasp_eef - bowl_centre = (-0.0128,
+0.0409)`, so the demo's plate centre must be `release_eef - that offset` =
(0.0764, 0.1858). Projecting (0.076, 0.186, 0.920) through the (seed-invariant)
camera lands at px (386,375) spanning u 338-433, v 340-414 -- exactly the plate
in `demo0_t0000.png`. Verdict: confirmed; the release point is therefore
`plate_centre + GRASP_OFF`.

The grasp is a **+y wall straddle**: 0.043 out from the bowl centre (bowl radius
0.055), descending to z 0.9194 = table + 0.018, closing to width 0.0157 in the
demo. Empty-air closes read 0.0046, loaded closes 0.0094 -- a clean held/not
discriminator.

## v3 -- first motion version

perceive -> pick the bowl nearest `tcen` -> grip open -> hover 1.00 ->
descend 0.9194 at `bowl + (-0.0128,+0.0409)` -> close -> carry at 1.02 ->
descend 0.9342 at `plate + (-0.0128,+0.0409)` -> open -> retreat -> re-perceive.

- probe (8 seeds 51..65 odd): **8/8**,
  `results/fs_c2k1clean_spa_bowl_table_center_pos_k1_v3`
- selection (all 15 debug seeds): **15/15**,
  `results/sel_c2k1clean_spa_bowl_table_center_pos_k1_v3`
- every seed: 2 bowls + 1 plate found, correct target, close read
  width 0.0094-0.0095 / effort 3.00, and post-place the target bowl vanishes
  from the 0.9455-0.9650 bowl band (a bowl standing on the 0.920 plate tops out
  at ~0.970) -- the sensor-side confirmation that it landed on the plate.
- note: the controller undershoots in z (commanded 0.9194 -> reached 0.9269;
  commanded 0.9342 -> reached 0.943-0.950). It grasps and places anyway.

## v4 -- v3 + dormant fallbacks (FROZEN)

Same nominal path; adds, firing only when the strict path finds nothing:
1. a splitter for a single over-wide bowl-band blob (two bowls touching),
2. a relaxed plate filter (brightest sizeable non-brown flat blob),
3. a three-rung grasp ladder gated on the measured width (>= 0.007) and
   effort (>= 1.0): demo offset -> the mirrored offset (same straddle, other
   wall) -> demo offset 8 mm deeper. The release offset tracks the rung used.

- probe (8 seeds): **8/8**, `results/fs_c2k1clean_spa_bowl_table_center_pos_k1_v4`
- selection (all 15 debug seeds): **15/15**,
  `results/sel_c2k1clean_spa_bowl_table_center_pos_k1_v4`
- rung 0 succeeded in 15/15 -- the fallbacks never fired on debug, so v4 is
  v3's behaviour plus headroom for eval layouts debug does not contain.

## DECLARATION

- **Frozen version: v4.** `packs/c2k1clean_spa_bowl_table_center_pos_k1/program.py`
  md5 `2f8677e27461b55b01e460dde9cf3e0b` == `program_v4.py` (same md5).
- **Selection receipt: 15/15 on the full 15-seed debug split**, dir
  `results/sel_c2k1clean_spa_bowl_table_center_pos_k1_v4`.
- Receipt chain:
  | version | probe (8 seeds) | selection (15 seeds) |
  |---|---|---|
  | v1 perception probe | 0/8 (no motion) `fs_..._v1` | -- |
  | v2 perception probe | 0/15 (no motion) `fs_..._v2` | -- |
  | v3 first motion | 8/8 `fs_..._v3` | **15/15** `sel_..._v3` |
  | v4 + fallbacks | 8/8 `fs_..._v4` | **15/15** `sel_..._v4` |
- `PROVENANCE` is present in program.py as a top-level literal dict covering
  every calibrated constant; sources are this pack's demo and debug-seed
  (51-65) measurements only.
- Protocol: every run went through `tools/fair_run.py --split debug`;
  `tools/fewshot_run.py` was never invoked; seeds 1-50 were never touched;
  `api.done` is never read; no forbidden file was opened.
