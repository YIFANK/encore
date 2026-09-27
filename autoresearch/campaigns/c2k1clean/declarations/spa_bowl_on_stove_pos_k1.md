# c2k1clean / spa_bowl_on_stove_pos_k1 — NOTES

Intent: "pick up the black bowl on the stove and place it on the plate"
Pack: K=1 demo, 155 steps, 4 keyframes, stride 10.

## Pack facts (read directly from pack.json)
- kf t=0   ee=(-0.2008, 0.0016, 1.1730) grip_cmd=-1 (open), gap 0.0725
- kf t=43  ee=(-0.2754,-0.1161, 0.9488) grip_cmd=+1 (CLOSE) -> grasp point
- kf t=141 ee=( 0.0673, 0.2453, 0.9415) grip_cmd=-1, gap 0.0074 (holding)
- kf t=154 ee=( 0.0621, 0.2475, 0.9306) grip_cmd=-1, gap 0.0774 (released)
- ee_path apex during transport z ~= 1.147
- action_scale = [0.2626 0.3058 0.3383 0.0167 0.0387 0.0286 1.0]
Scene (kf images, 128px agentview): dark cabinet upper-left with a bowl on top;
a light stove platform with a dark bowl on it; a second bowl on the table
(right of centre); a cookie box; a plate (white disc, pink rim) lower-right.
So there are >=3 bowls -> the SUPPORT (stove) is the identity cue.

## Debug-seed measurements (all from cam_high RGB-D, seeds 51-65)
table plane (depth-histogram mode) z = 0.9010
stove top        = table + 0.0253     free bowl on table: top = table + 0.0427
bowl ON STOVE    = table + 0.0782     cabinet top        = table + 0.278
plate            = table + 0.0187, footprint 0.140 x 0.140, colour ~(151,143,140)
cookie box       = table + 0.0194, 0.088 x 0.064, colour ~(97,68,47)
bowl footprint   ~ 0.090 m across; jaws open to 0.0805 -> cannot straddle
fingertip offset F: eef z at a table-blocked descent = table + 0.0088..0.0111
episode budget   ~1000 control steps; an UNCONVERGED move burns
                 max(40, 60*seconds)*2 steps, so `seconds` is the real budget
move_path/act    absent on the LIBERO backend (rig-only) -- use move() only
LIBERO ends the episode the moment the predicate fires, so a frozen arm after
the place descent is the success, not the horizon.

## Version log
v1  perception dump (no motion).  Camera fixed across seeds; scene layout
    jitters only ~2 cm. -> receipt results/fs_..._v1 (0/8, no motion)
v2  calibration probe. F = 0.009-0.011 m; residual at (0.15,0.30) is 0.05 =
    reach limit, not controller error; horizon ~1000 steps. 0/2, no attempt.
v3  full-res (512) cluster table for all 8 probe seeds -> the numbers above.
    0/8, no motion.
v4  first attempt. Wall-pinch at (cx, cy+R), grasp z = rim-0.0305 (the demo's
    own offset).  Grasp succeeded on 8/8 (effort 3.0, bowl gone from the
    stove), transport + release executed -- but 0/8.
    DIAGNOSIS from the gif: released with the TOOL POINT over the plate
    centre; since the tool holds the bowl by its WALL the bowl centre lands
    one radius (~55 mm) off the plate.
v5  release aimed at plate_centre MINUS the measured carry offset
    (bowl_centre - achieved grasp eef).  8/8 on the probe subset.
    receipt: results/fs_c2k1clean_spa_bowl_on_stove_pos_k1_v5  8/8
v6  v5 hardened: grasp-retry ladder on a jaws-closed-on-air reading, colour
    gate on the plate demoted to a preference behind a roundness gate,
    cabinet-skirt waypoint if the plate is detected behind the cabinet,
    PROVENANCE completed.  Probe receipt: fs_..._v6 8/8.

# DECLARATION

Frozen version: **program_v6.py**
  md5(program.py) == md5(program_v6.py) == aa514dffcb0274bb1e8c905b82dae342
  (verified on the cluster in packs/c2k1clean_spa_bowl_on_stove_pos_k1/)

Selection receipt (one formal run, all 15 debug seeds 51-65):
  **15/15** — results/sel_c2k1clean_spa_bowl_on_stove_pos_k1_v6
  (51 T, 52 T, 53 T, 54 T, 55 T, 56 T, 57 T, 58 T, 59 T, 60 T, 61 T, 62 T,
   63 T, 64 T, 65 T)

Per-version receipt chain (probe subset 51,53,55,57,59,61,63,65 unless noted):
  v1  results/fs_c2k1clean_spa_bowl_on_stove_pos_k1_v1   0/8  (perception dump, no motion)
  v2  results/fs_c2k1clean_spa_bowl_on_stove_pos_k1_v2   0/2  (calibration, seeds 51,53, no attempt)
  v3  results/fs_c2k1clean_spa_bowl_on_stove_pos_k1_v3   0/8  (512-res cluster table, no motion)
  v4  results/fs_c2k1clean_spa_bowl_on_stove_pos_k1_v4   0/8  (pick OK 8/8, release off by one bowl radius)
  v5  results/fs_c2k1clean_spa_bowl_on_stove_pos_k1_v5   8/8  (carry offset applied)
  v6  results/fs_c2k1clean_spa_bowl_on_stove_pos_k1_v6   8/8  (hardened)
  v6  results/sel_c2k1clean_spa_bowl_on_stove_pos_k1_v6 15/15 (FORMAL SELECTION)

PROVENANCE: present in program.py as a top-level literal dict, covering
GRASP_BELOW_RIM, RELEASE_ABOVE_PLATE, BOWL_TOP_BAND, RIM_BAND,
PLATE_SIZE_BAND, HOVER_H/LIFT_H/CARRY_H, HELD_MIN_WIDTH and CELL. Every
constant is sourced either from this pack (the two demo keyframe heights) or
from this cell's own debug-seed RGB-D measurements. No foreign constants.

Clean room: only packs/c2k1clean_spa_bowl_on_stove_pos_k1/* and
results/*c2k1clean_spa_bowl_on_stove_pos_k1* were written on the cluster; no
benchmark asset, no other cell's artifacts, and no program/NOTES file under
any pack directory were read. tools/fewshot_run.py was never invoked.

STOP.
