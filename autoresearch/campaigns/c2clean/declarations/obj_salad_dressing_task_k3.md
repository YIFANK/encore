# c2clean / obj_salad_dressing_task_k3

Intent: **"Pick the tomato sauce and place it in the basket"** (the bddl string
names a different object; the intent sentence is authoritative, and the
benchmark bit turned out to grade the intent object -- see v4).

Packs used: `c2clean_obj_salad_dressing_task_k3` (language "pick up the salad
dressing and place it in the basket", demos in MY scene) and
`c2clean_obj_salad_dressing_task_mate` (language "pick up the tomato sauce and
place it in the basket", demos in a DIFFERENT scene). pack.json + keyframes
only.

## What the packs say

| | task pack (salad dressing) | mate pack (tomato sauce) |
|---|---|---|
| closing EEF z | 0.117 / 0.123 / 0.126 | 0.045 / 0.049 / 0.058 |
| held gripper qpos sum | 0.018 | 0.061-0.064 |
| release EEF | (0.0,0.24,~0.20) | (-0.02,0.25,~0.19) |

So the named object is a SHORT, WIDE prop (~0.08 m tall, ~0.062 m across),
unlike the tall thin-necked bottle the task pack picks. Rotations in
`ee_path6[3:6]` are rotation VECTORS, not euler rpy: rebuilding the home value
[3.127, 0.013, -0.117] as a rotvec reproduces the tool matrix I read back at
t=0 (off-diagonals 0.009 vs 0.000) while the rpy reading does not (-0.117).
Both packs' yaw drift is teleop noise on a round can, so the program keeps the
wrist straight down.

## Version chain

| ver | hypothesis | run | result |
|---|---|---|---|
| v1 | map the scene from cam_high | fs_..._v1, seeds 51,53 | 6 clusters; two props hidden inside the robot-column cluster |
| v2 | drop any XY cell containing a point above 0.20 m -> the arm disappears | fs_..._v2, 15 seeds | 7 props on every seed; layout static except basket / red box / flat box. Two short cans (top 0.081) at (-0.201,-0.087) and (-0.160,0.056) |
| v3 | grasp the warmer can, drop it in the basket | fs_..._v3, seeds 51,53 | 0/2. Mechanics fine (closed 0.062, effort 3.0, released over the basket) but it took the BLUE can: scored over arm-masked cluster pixels, rb -1.0 vs -0.4 |
| v4 | score colour over a height window (table+10mm .. top) instead | fs_..._v4, seeds 51,53,55,57 -> **4/4**; sel_..._v4, 15 seeds -> **15/15** | rb 18.5 vs -6.6, decisive |
| v5 | v4 + lift-time sensor check (width > 0.035 and effort > 1) with one re-perceive/retry | sel_..._v5, 15 seeds | see DECLARATION |

### Identity chain (why the warm can is the tomato sauce)

1. Project the mate pack's three closing EEF points through the cam_high
   intrinsics/extrinsics I measured at runtime -> pixel (~192,295) of the mate
   pack's own t=0 keyframe. That is a squat can, grey lid, red/green body.
2. Its crop measures mean(R-B) = +13.4.
3. In my scene exactly two props share that height class (top 0.081). Measured
   over the height window they score +18.5 and -6.6; the warm one is picked.
   Ranking, not thresholding -- the absolute level moves with the shadow the
   robot casts over both cans.

### Why v3 failed (the load-bearing lesson)

The arm-cell mask that makes the props visible at all also deletes the parts of
a prop that the robot happens to hang over -- roughly half of each can here.
What survives is the grey lid, so a colour cue computed on cluster membership
reads ~0 for everything. Receipt: after the arm left the area, the same can
re-measured n=1350 (vs 615) and rb +17.6 (v3 ep51, Q5). Segment with the mask;
sample colour with a height window.

### v6 -- aim-envelope measurement (not a candidate)

v5 with a deliberate +12 mm offset on the jaw axis (y), seeds 51,53,55,57
(`fs_..._v6`). Every seed's FIRST grasp failed exactly as designed -- jaws
closed to 0.0252 with effort 3.0, then read width 0.0017 after the lift, i.e.
empty -- the retry fired on all four, re-perceived from the lifted pose, and
all four episodes still scored. So:

* the can grasp tolerates less than 12 mm of aim error (15/15 at 0 mm, 0/4
  first-try at 12 mm), and
* the v5 retry converts that failure into a success, which is why v5 is frozen
  over v4 even though both scored 15/15 clean.

## DECLARATION

* **Frozen version: v5.** `packs/c2clean_obj_salad_dressing_task_k3/program.py`
  md5 `80ee65aad2cecd9b1628e1393a44fae1` == `program_v5.py`.
* **Selection receipt: 15/15** on the full debug band (seeds 51-65),
  `results/sel_c2clean_obj_salad_dressing_task_k3_v5`.
* Receipt chain:
  * v1 `fs_..._v1` seeds 51,53 -- perception only, no motion.
  * v2 `fs_..._v2` seeds 51-65 -- perception only, 7 props on every seed.
  * v3 `fs_..._v3` seeds 51,53 -- **0/2**, wrong can (arm-masked colour cue).
  * v4 `fs_..._v4` seeds 51,53,55,57 -- **4/4**; `sel_..._v4` seeds 51-65 -- **15/15**.
  * v5 `sel_..._v5` seeds 51-65 -- **15/15**, retry never fired.
  * v6 `fs_..._v6` seeds 51,53,55,57 -- **4/4** under a 12 mm aim insult, all
    four via the retry (envelope measurement, not a candidate).
* PROVENANCE dict present in program.py; every constant traces to one of the
  two named packs, a debug-seed measurement, or generic camera/gripper
  mechanics.
* The benchmark bit grades the INTENT object: v3 dropped the blue can in the
  basket and scored false; v4 dropped the warm can and scored true.
