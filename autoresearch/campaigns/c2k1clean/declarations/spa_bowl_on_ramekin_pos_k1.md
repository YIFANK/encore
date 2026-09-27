# c2k1clean / spa_bowl_on_ramekin_pos_k1

Task: "pick up the black bowl on the ramekin and place it on the plate".
Runner: tools/fair_run.py only (FAIR_PROTOCOL v1.1.1; no shared note file).
Pack: packs/c2k1clean_spa_bowl_on_ramekin_pos_k1 (K=1, 4 keyframes, ee_path6
of 14 poses, 137 raw actions).

## Pack reading (the only task-specific prior)

- keyframes: t=0 open, ee z 1.1686; t=32 gripper_cmd flips to CLOSE at
  ee (-0.1883, 0.1403, 0.9899); t=124 flips to OPEN at (0.0434, 0.1649,
  0.9554) -> release; t=136 open at z 1.0312.
- gripper_state while holding = [0.0049, -0.0023] -> ~7 mm jaw span: the demo
  pinches something thin (a bowl wall); it does not straddle the whole bowl.
- transport altitude ee_path6[8][2] = 1.1345.
- t=0 keyframe image: a dark cabinet, a stove, TWO dark bowls (one standing on
  a small ramekin), one light plate.

## Version log (hypothesis -> evidence -> verdict)

### v1 -- perception dump + rim-pinch attempt. seeds 51,53,55,57 -> 0/4
H: cam_high RGB-D is enough to find the table plane, both dark bowls and the
plate; the bowl on the ramekin is the elevated one.
E (results/fs_c2k1clean_spa_bowl_on_ramekin_pos_k1_v1, 0/4, note "no
candidate"): table plane z = 0.9020 (depth mode, 113k px). Seed 51: cabinet
(0.015,-0.249) h 0.226; robot column h 0.464; light disc (0.080,0.017) h 0.036
lum 153; dark bowl (0.057,0.200) top 0.9515 h 0.050; dark bowl (-0.198,0.213)
top 0.9988 h 0.097. The two bowl tops differ by 0.046 m = the ramekin's lift.
tool_rotation at rest = [[.998,0,-.057],[0,-1,0],[-.057,0,-.998]] -> the
rotation=None wrist points straight down with its jaw axis on world x or y.
V: PARTIAL. Bowl separation works; the plate was never segmented (it is
flatter than the +12 mm object cut), so the program aborted before moving.

### v2 -- perception only, full debug band. seeds 51-65 -> 0/15 (by design)
H: a +5 mm object cut plus richer descriptors segments the plate too, stably
across seeds.
E: 15/15 seeds give the same 5 table props. The light disc h 0.036 lum 151-153
is the PLATE (confirmed against ep51's rendered frame: plate front-centre,
free bowl front-right, bowl-on-ramekin back-right); the stove's burner reads
lum 26-27 and the bowls lum 104-120, so luminance separates all three classes.
Bowl extents 0.106-0.111 m, plate 0.136 m.
V: PERCEPTION SETTLED. Classifier: bowl = ext 0.085-0.140, h 0.030-0.130,
lum 80-140, hollow > 0.020; plate = ext 0.100-0.180, h 0.010-0.060, lum > 140;
target = the bowl with the highest top.

### v3 -- wall straddle at the demo's grasp height. seeds 51..65 odd -> 0/8
H: the jaws (0.072 m) are far wider than the bowl wall, so a straddle with one
finger inside the bowl works anywhere from 16 mm off-centre outwards; the demo
says the close happens 0.0089 m below a bowl-on-ramekin rim.
E: all four rim points (tool-y +/-, tool-x +/-) closed on air, finger gap
0.001-0.005 m; the descents stopped 8-12 mm high (heron's move_cartesian
breaks inside a 12 mm ball). Also: my clearance veto was tripped by a cluster
belonging to the robot arm, not a table prop.
V: REFUTED as executed. Either the fingertips are not at api.eef()'s height,
or the straddle needs to reach deeper than 9 mm below the rim.

### v4 -- fingertip calibration + grasp ladder. seeds 51,53,55,57 -> 4/4
H: pressing the OPEN jaws onto bare table can only be stopped by the
fingertips, so api.eef() there minus the table plane IS the fingertip offset;
with that known, a ladder over (radial offset, depth below the rim) finds a
straddle that survives the lift.
E: TIP_OFF = 0.0069 m on all four seeds (eef z 0.9088-0.9089 against table
0.9020) -- the tips are essentially AT the reported eef, so v3's failure was
depth, not offset. Ladder results, offset 0.045 m: depth 0.015 m grips
(gap 0.0067-0.0074) but loses the bowl on the lift on 3 of 4 seeds; depth
0.032 m holds (gap 0.0050-0.0094 after the lift). Release at plate_top +
0.017 + TIP_OFF lands the bowl with residual 0.010-0.026 (contact-stopped).
V: MECHANISM FOUND. 4/4 benchmark successes. Costly, 264-528 sim steps,
because the winning rung was reached 1st-4th.

### v5 -- evidence-ordered ladder, hardcoded TIP_OFF, re-tracking. FROZEN
H: putting the deep straddle (offset 0.045, depth 0.032, jaw-axis sign +1)
first, dropping the in-episode table-touch calibration (TIP_OFF is constant at
0.0069), and re-perceiving the bowl after any dropped rung gives the same
grasp at a fraction of the step budget.
E: probe results/fs_c2k1clean_spa_bowl_on_ramekin_pos_k1_v5 = 8/8 on
51,53,...,65, 143-147 sim steps each (rung 0 wins on every seed).
Selection: results/sel_c2k1clean_spa_bowl_on_ramekin_pos_k1_v5 = **15/15** on
all 15 debug seeds (140-147 sim steps).
V: SELECTED and frozen.

## Receipt chain

| version | seeds | result dir | score |
|---|---|---|---|
| v1 | 51,53,55,57 | fs_c2k1clean_spa_bowl_on_ramekin_pos_k1_v1 | 0/4 (no plate found) |
| v2 | 51-65 | fs_c2k1clean_spa_bowl_on_ramekin_pos_k1_v2 | 0/15 (perception only, no motion) |
| v3 | 51,53,...,65 | fs_c2k1clean_spa_bowl_on_ramekin_pos_k1_v3 | 0/8 (closed on air) |
| v4 | 51,53,55,57 | fs_c2k1clean_spa_bowl_on_ramekin_pos_k1_v4 | 4/4 |
| v5 | 51,53,...,65 | fs_c2k1clean_spa_bowl_on_ramekin_pos_k1_v5 | 8/8 |
| v5 | 51-65 (formal) | sel_c2k1clean_spa_bowl_on_ramekin_pos_k1_v5 | **15/15** |

## DECLARATION

- Frozen version: **v5**.
  `packs/c2k1clean_spa_bowl_on_ramekin_pos_k1/program.py` md5
  `70b532dbe3ff1b3dddacbe209d2ac1a1` == `program_v5.py` (verified on AbakaAI).
- Selection receipt: **15/15** on the full 15-seed debug band, dir
  `results/sel_c2k1clean_spa_bowl_on_ramekin_pos_k1_v5`
  (`benchmark_success: true` on seeds 51-65, 140-147 sim steps each).
- Per-version receipt chain: table above; every formally probed version is
  archived as `program_vN.py` in the pack dir (v1-v5).
- PROVENANCE: present in program.py, 13 entries, every one `allowed: True`
  with a source that is either a pack field or a debug-seed measurement.
  Re-checked against fair_run.py's gate: no forbidden tokens, no `.done`
  attribute read, PROVENANCE parses and validates.
- Clean room: writes confined to
  `packs/c2k1clean_spa_bowl_on_ramekin_pos_k1/*` and
  `results/*c2k1clean_spa_bowl_on_ramekin_pos_k1*`; no benchmark asset, no
  other cell's artifacts, no `fewshot_run.py`. Every constant is re-derived
  from this pack plus my own debug-seed observations.
- Eval seeds 1-50 were never touched.

STOP.
