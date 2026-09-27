# c2clean goal_turn_on_stove_task_k3

Intent: "Turn off the stove". bddl: turn_on_the_stove. Pack language: "turn on the stove".
Pack K=3 demos, mechanism read off the keyframes:
  home (-0.21, 0.00, 1.16) -> approach (-0.43/-0.38, 0.21, 0.93) with gripper OPEN,
  gripper_cmd flips to close at the approach keyframe, then a ~1.3-1.8 rad wrist
  rotation held in place at z=0.929. Terminal finger gap 0.029-0.035 m (fully open
  is 0.072), i.e. the fingers are closed on a ~3 cm object -> the stove knob.
  Keyframe RGB: the burner disc turns RED between the close keyframe and the last
  keyframe, so the graded event is the burner lighting.
Rotation convention: interpreting ee_path6 rpy as ZYX (Rz Ry Rx) makes the
close->terminal delta a near-pure world +y rotation (rotvec [0.34,1.36,0.14],
[0.14,1.31,0.09], [0.15,1.82,0.58]); the XYZ reading gives a messy
[-0.48,-1.33,0.47]. ZYX adopted, to be confirmed against api.tool_rotation() at t0.

## v1 (probe 51,53,55,57)
Hypothesis: replay the pack in absolute coords -- descend to the mean terminal EEF
(-0.4093, 0.2059, 0.9298), close, then rotate 1.45 rad about world (0.21,1.50,0.27)
in 5 steps. Also dumps cam_high RGB-D (zlib+base64 through api.log) for perception.
Receipt: results/fs_c2clean_goal_turn_on_stove_task_k3_v1 -- 4/4 (51,53,55,57).
Verdict: the rotation NEVER executed. Successful episodes stop at 65-77 sim steps
with the EEF and tool axis frozen: LIBERO terminates the episode when the
predicate fires, so the benchmark bit had already fired during the descend+close.
The wrist turn the demos append is not the load-bearing part; the press is.

## probe_percep (all 15 debug seeds, no motion)
0/15 -- confirms nothing is already satisfied at reset. Chunked base64 RGB-D dump
(api.log truncates each line at 2000 chars, so v1's single-line dump was lost).
Measured, every seed: the knob is a near-black post (RGB (3,3,3)) at the back-left
corner of the stove slab (slab reads ~(72,72,72)); its flat top deprojects to
z = 0.9603 on all 15 seeds; top-face centroid x in [-0.4302,-0.4097],
y in [0.1977,0.2168]. Two fixed dark distractors exist at y ~ -0.05 (top z ~ 0.99);
they are rejected by a 0.15 m gate around the pack's terminal-EEF prior.

## v2 (probe 51,53,55,57,59,61,63,65)
Hypothesis: perceive the knob, press on its top-face centroid (dz 0.032), close,
then stage the pack's wrist turn in 5 steps.
Receipt: results/fs_c2clean_goal_turn_on_stove_task_k3_v2 -- 6/8 (lost 53, 65).
Verdict: two independent findings. (a) The aim envelope is tight: pressing exactly
on the perceived centroid loses seeds that a 7-9 mm +x offset wins. (b) The staged
turn is actively harmful -- on 53 and 65 rotation step 5 flipped the tool axis to
toolz=[-0.67,0.19,-0.71] and the arm ran away to x=-0.745, burning the horizon.

## v3 (formal 15) / v1b (11 seeds)
v3: perceive the knob, drop the turn, press at +7 mm x / +6 mm y / 31 mm below the
top, and back it with a 10-point raster of further presses (the horizon buys ~12).
Receipt: results/fs_c2clean_goal_turn_on_stove_task_k3_v3 -- 15/15, every episode
ending at 68-74 sim steps, i.e. attempt 0 fired everywhere and the raster was
never exercised.
v1b (fixed absolute target, seeds 52,54,56,58-65): 11/11, so v1 is also 15/15
across the debug split. v3 is preferred because its aim tracks the perceived knob
rather than one absolute point.

## v4 (envelope test, formal 15)
Hypothesis: does the raster actually rescue a bad first press? Same as v3 with the
known-good offset moved LAST and v2's losing (0,0) aim first.
Receipt: results/fs_c2clean_goal_turn_on_stove_task_k3_v4 -- 13/15 (lost 55, 57).
Verdict: the raster rescues (59 at 153 steps, 65 at 424, 64 at 625) but not always
-- and note that on 55 and 57 even the known-good offset, replayed as attempt 9,
failed. A press that misses SHOVES the knob, so a stale target goes stale.

## v5 (frozen) + v5b (envelope test, formal 15)
Hypothesis: re-perceive the knob before every retry instead of re-using the
attempt-0 target, and make the retry ring tight around the known-good offset.
Receipts:
  results/sel_c2clean_goal_turn_on_stove_task_k3_v5  -- 15/15 (all 68-74 steps)
  results/fs_c2clean_goal_turn_on_stove_task_k3_v5b  -- 15/15 with the deliberately
    bad (0,0) aim first: 55 rescued at 556 steps, 57 at 545, 64 at 492, 59/65 at 149
  results/sel_c2clean_goal_turn_on_stove_task_k3_v5f -- 15/15 on the frozen file
Verdict: re-perception closes the gap v4 exposed. The first press wins on every
debug seed on its own, and the retry chain independently wins on every debug seed
even when the first press is aimed at the known-bad point.

# DECLARATION

Frozen version: program.py == program_v5.py, md5 54fc06500f04ce15636997ea874648c0
(equal to results/sel_c2clean_goal_turn_on_stove_task_k3_v5f/program_archived.py).

Selection receipt (full 15 debug seeds 51-65, one formal run of the frozen file):
  15/15  results/sel_c2clean_goal_turn_on_stove_task_k3_v5f
Corroborating full-15 run of the same program before the comment-only edit:
  15/15  results/sel_c2clean_goal_turn_on_stove_task_k3_v5

Per-version receipt chain:
  v1   4/4   fs_..._v1   (51,53,55,57)            pack replay, fixed target
  v1b 11/11  fs_..._v1b  (52,54,56,58-65)         same program, rest of the split
  percep 0/15 fs_..._percep                        perception only, no motion
  v2   6/8   fs_..._v2   (odd seeds)              perceived centroid + staged turn
  v3  15/15  fs_..._v3   (all 15)                 perceived + offset press + raster
  v4  13/15  fs_..._v4   (all 15)                 envelope test, stale-target raster
  v5b 15/15  fs_..._v5b  (all 15)                 envelope test, re-perceived retries
  v5  15/15  sel_..._v5  (all 15)                 selection
  v5f 15/15  sel_..._v5f (all 15)                 selection, frozen file

PROVENANCE present in program.py: 13 entries, every calibrated constant covered
(KNOB_PRIOR_XY, DARK_SUM_MAX, BAND_Z, CLUSTER_TOL_M, PRIOR_RADIUS_M,
KNOB_TOP_Z_FALLBACK, OFFSETS, OFFSETS_PRESS_DZ, REPERCEIVE_Z_TOL_M, LIFT_DZ,
GRIP_OPEN_M, GRIP_CLOSE_M). Sources are pack.json fields and debug-seed 51-65
RGB-D / EEF / gripper measurements only. api.done is never read.
