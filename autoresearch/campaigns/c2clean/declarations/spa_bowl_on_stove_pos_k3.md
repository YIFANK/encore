# c2clean / spa_bowl_on_stove_pos_k3

Intent: "pick up the black bowl on the stove and place it on the plate".
_pos cell: object placements are permuted per seed, so the demo anchors are
decoys; every target must be perceived.

## Pack facts (packs/c2clean_spa_bowl_on_stove_pos_k3/pack.json)

K=3 demos, keyframes carry eef 6-dof + gripper state/cmd.
Convention read off the states: gripper_cmd +1 = close, -1 = open.

| demo | close eef (x,y,z)          | release eef (x,y,z)      |
|------|----------------------------|--------------------------|
| 0    | (-0.2754, -0.1161, 0.9488) | ( 0.0621, 0.2475, 0.9306)|
| 1    | (-0.2752, -0.0861, 0.9420) | ( 0.0687, 0.2164, 0.9568)|
| 2    | (-0.2667, -0.0863, 0.9695) | ( 0.0340, 0.2390, 0.9290)|

close z mean 0.9534 (spread 0.027); release z mean 0.9388.
Carry apex from ee_path ~1.03-1.15.
Demo xy differ across demos (=> _pos), so only the z numbers transfer.

## Scene (from my own debug-seed captures)

Camera cam_high: K f=618.04, c=(256,256); T_base_cam constant
  x_base = 0.6283*yc - 0.778*zc + 0.6586
  y_base = xc
  z_base = -0.778*yc - 0.6283*zc + 1.6104
=> image u grows with base +y, image v grows with base +x.

Table top z = 0.901 (median of the deprojected workspace).

Props: cabinet (top ~1.15, with a bowl on its top ~1.18), a stove slab
(top ~ table+0.044), a black bowl ON the stove, a black bowl on the table
(top ~ table+0.043), a plate (top ~ table+0.019, wide pale disc) and a
cookie box (top ~ table+0.019, small brown).

Impostor: the robot's mounting post at x ~ -0.41, y ~ -0.14 reaches
z 0.96 and is black + compact. Excluded by x >= -0.36.

## Versions

- v0 (probe): logged camera model, depth stats, z histogram. table = 0.901.
- v1 (probe): top-down height map + connected components at the HOME pose.
  Verdict: the arm at home fuses with the stove/cabinet; home pose is unusable
  for perception.
- v2 (probe): tried 3 park poses. (0.20,0.40,1.10) and (0.25,-0.40,1.10) are
  blocked (residual 0.21 / 0.27). **(-0.10, 0.45, 1.25) reaches with residual
  0.011 and clears the whole table out of the camera frame.** Adopted as PARK.
  Verdict: park-then-perceive is the perception protocol for this cell.
- v3 (probe): park + dump full-scene RGB-D (256x256) on all 15 debug seeds,
  so the detector can be designed offline. -> see below.

- v4 (probe, seeds 51/53): perceive + descend on the bowl CENTRE with the jaws
  open. Receipts: `tool_rotation` at the straight-down wrist is diag(1,-1,-1),
  so the finger-opening axis is world y; the descent stalled at eef z 0.9645
  (commanded 0.940) with the jaw width squeezed from 0.0798 to 0.0719 — the
  open jaws (span 0.080) jam on the rim of a 0.108 bowl; the close then read
  width 0.0010 / effort 0.05. Verdict: centre descent is an empty close, and
  0.0010 is the empty-jaw receipt. Also burned the whole 1000-step horizon on
  12 moves, so the move budget has to be spent deliberately.
- v5 (8 probe seeds 51..65 odd): rim straddle — aim the eef RIM_OFF=0.046 in
  +y from the bowl centre, seat at z 0.947, close, carry at 1.06, release over
  the PLATE CENTRE at 0.945. Close read width 0.0134 / effort 3.0 (holding),
  lift read 0.0048 / effort 0.05. **0/8.** The gif shows the bowl picked,
  carried and set down on the plate's EDGE. Diagnosis: the pinch holds the
  bowl's rim, so the bowl's centre trails the eef by RIM_OFF; releasing at the
  plate centre puts the bowl 46 mm off-centre on a plate of radius 0.069.
  (The effort flag is a gap threshold — 0.0048 reads "not holding" while the
  bowl is in fact still gripped, so width, not effort, is the receipt here.)
- v6 = v5 with the release point pushed by the same RIM_OFF:
  `py = plate.cy + RIM_OFF`. Probe 8/8, **formal selection 15/15**
  (results/sel_c2clean_spa_bowl_on_stove_pos_k3_v6).
- envelope (6 seeds each, results/fs_..._v6{a,b,c,d}):
  RIM_OFF 0.038 -> 6/6, 0.054 -> 6/6, Z_GRASP 0.939 -> 6/6, 0.955 -> 6/6.
  Both calibrated constants tolerate +-8 mm, against a per-seed target spread
  of about +-15 mm in xy, so the aim is not sitting on a cliff.
- v7 = v6 + three guards that never fire on the debug seeds: a footprint-shape
  gate on the bowl (w,h in [0.07,0.14]) and a width gate on the plate
  (w > 0.10, pale) instead of "largest blob"; one grasp retry if the jaws lift
  with width <= 0.0025 (the v4 empty-close receipt); pack-mean fallbacks if a
  band comes back empty. **Formal selection 15/15**
  (results/sel_c2clean_spa_bowl_on_stove_pos_k3_v7); the retry branch fired on
  0 of the 15 seeds, every first lift read width 0.0047-0.0054.

## DECLARATION

- Frozen version: **v7**.
  `packs/c2clean_spa_bowl_on_stove_pos_k3/program.py`
  md5 `e2920a4cb85692eb074c189c5f2f0a8c` ==
  `packs/c2clean_spa_bowl_on_stove_pos_k3/program_v7.py` (same md5).
- Selection receipt: **15/15** on the full 15 debug seeds 51-65,
  `results/sel_c2clean_spa_bowl_on_stove_pos_k3_v7`.
- Receipt chain: v0/v1/v2/v3 perception probes -> v4 centre-descent probe
  (empty close, 0/2) -> v5 rim straddle, release at plate centre, 0/8 ->
  v6 release offset by RIM_OFF, 8/8 probe and 15/15 formal
  (`results/sel_c2clean_spa_bowl_on_stove_pos_k3_v6`) -> envelope sweep
  v6a-v6d, 6/6 each -> v7 guarded, 15/15 formal.
- PROVENANCE: present as a top-level literal dict in program.py, covering
  PARK, BAND_BOWL, BAND_FLAT, BOWL_W/PLATE_W, RIM_OFF, Z_GRASP, Z_RELEASE,
  Z_CARRY, EMPTY_W, DEMO_BOWL/DEMO_PLATE and the workspace crop. Every
  constant comes from this pack's keyframes or from my own debug-seed
  captures; no LIBERO prior knowledge was used.
- Eval seeds 1-50 were never touched.
