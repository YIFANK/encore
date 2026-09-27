# c2k1clean / goal_put_cream_cheese_in_bowl_pos_k1

Intent: **put the cream cheese in the bowl**. K=1 pack, `_pos` perturbation cell.
Runner: `tools/fair_run.py` only. Debug band = seeds 51-65.

## Pack reading (the only task-specific input)

`pack.json`: one demo, 92 steps, 4 keyframes, `ee_path6` of 10 poses.

| index | t | eef xyz | grip cmd | reading |
|---|---|---|---|---|
| kf0 | 0 | (-0.2054, 0.0076, 1.1850) | -1 (open) | home |
| kf1 | 40 | (-0.0213, 0.1119, 0.9104) | +1 (close) | **grasp**: z = 0.9104 |
| p6[6] | ~60 | (-0.0304, 0.0508, 1.0415) | +1 | carry apex |
| p6[8] | ~80 | (-0.0916, -0.0207, 0.9669) | -1 (open) | **release**: z = 0.9669 |
| kf3 | 91 | (-0.0731, -0.0071, 1.0118) | -1 | retreat |

Keyframe PNGs (128x128) show the demo scene: bottle centre, bowl left of it,
plate front-left, stove right, cream-cheese carton right of the bottle. At
t=0091 the blue carton is sitting inside the bowl.

**The pack xy is a decoy.** The demo grasp (-0.021, +0.112) and release
(-0.092, -0.021) do not match any debug seed (carton lives near (+0.05, -0.02),
bowl near (-0.20, -0.05)). Only the *heights* transfer, so the program
perceives both xy's every episode and takes GRASP_Z / RELEASE_Z from the pack.

## Probes

### p1 (probe_v1.py) - raw sensor dump, seeds 51,52
Streamed cam_high + cam_arm_wrist RGB + depth out through `api.log`
(zlib+base64, 1800-char chunks) so perception could be built offline.
Receipt: `results/fs_..._p1`.
- table z = 0.900 (117k/262k depth pixels in one 10 mm bin)
- cam_high: K f=618.04, c=(256,256); t_base_cam puts the camera at
  (0.659, 0, 1.610) looking down the -x axis. +y in base = +u in the image.
- home eef (-0.2085, 0, 1.1733), tool_rotation ~ diag(1,-1,-1) tilted 3.3 deg,
  gripper width 0.0778 open.

### p2 (probe_v2.py) - cam_high dump, all 15 debug seeds
Receipt: `results/fs_..._p2`. Prop inventory (5 mm top-down max-z raster):

| prop | z top | footprint | mean RGB | note |
|---|---|---|---|---|
| cream cheese | 0.9196 | 0.080 x 0.042 | (71,78,94) | **target**, B-R = +22..+25 |
| plate | 0.920 | 0.135 x 0.135 | (142,133,130) | B-R = -12 |
| bowl rim | 0.9522 | 0.110 x 0.110 | (105,105,97) | **goal** |
| stove slab | 0.932 | 0.190 x 0.190 | (72,72,72) | |
| pot on stove | 0.959 | 0.030 x 0.025 | (27,27,27) | same band as the bowl, dark |
| bottle | 1.059 | 0.028 x 0.043 | (48,43,30) | tallest obstacle on the carry |
| wood rack | 1.090 | | (110,98,87) | |
| cabinet | 1.100 | | (72,72,71) | |

Perturbation over the 15 seeds is small and translation-only: carton
x in [0.033, 0.070], y in [-0.038, -0.003], **no yaw change** (footprint stays
0.080-0.085 x 0.040-0.045 axis-aligned in every seed); bowl x in [-0.213,
-0.188], y in [-0.075, -0.038].

Two separation facts that drove the detector:
1. 8-connected clustering on a plain `z > table+12 mm` mask **fuses the bowl
   with the wood rack** (seeds 52, 62). Height-*band* masking first
   (38-100 mm) removes the rack/cabinet/bottle and leaves only the rim.
2. The arm at the home pose **splits the rim into two bright pieces**
   (seeds 53, 58, 59); taking the largest piece alone biases the bowl centre
   by ~25 mm in y. Merging bright band pieces within 0.14 m restores a
   0.110 x 0.110 bbox on all 15 seeds.

## Versions

### v1 - perceive both props, pack heights, carry over the bottle
Hypothesis: the cell is a plain pick-and-place; the only thing the `_pos`
perturbation breaks is the demo xy, so perceiving carton + bowl and replaying
the pack's grasp/release *heights* should solve it. Jaws close along base y
(R_DOWN = diag(1,-1,-1)), i.e. across the carton's 40 mm side, not its 80 mm
side. Carry at z 1.12 to clear the bottle (1.059), which sits between the
carton and the bowl.

Evidence: offline, the detector returns the carton and the bowl on 15/15 debug
seeds (blue margin >= 21.7 vs a 6.0 cut; bowl bbox 0.110 x 0.110 every seed).

Receipt: `results/fs_..._v1` = **6/8** (51,53,57,59,61,65 ok; 55,63 fail).

Verdict: perception is right on both failures (target and goal xy are correct
to the millimetre). What breaks is motion: on 55 and 63 the very first move
toward the carton ends 46-58 mm off, the close grabs air, and the episode then
burns all 1000 sim steps (successes use 162-309).

### v2 - closed-loop goto with bias cancellation
Hypothesis: the v1 failures are tracking error, so re-issuing the move with the
command shifted by the observed error (bounded, with a straight-up retreat when
the error stops shrinking) should land them.

Receipt: `results/fs_..._v2` = **6/8**, the same two failures.

Verdict: refuted, and the trace says why. The staged approach reaches
(tx, ty, 1.12) with 6 mm of error, then the descent to 0.96 stops dead at
z = 1.0647 and *never moves again* - every later command, including a lateral
one, returns the identical eef to 4 decimals. It is not tracking error, it is
a hard stop, and the bias-cancel retries just burn the horizon.

## Probing the stall (p3, p4, p5, p6, p7, p8)

- **p3** (seeds 55/63/57, 2 cm descent steps + joint logging): the stall is at
  z 1.0647 with joint 2 frozen at 0.717 while the control seed sails through
  the same value to 1.007. `rotation=None` stalls identically, so it is not the
  orientation term. A 90 deg yawed wrist *did* reach 0.920 - but see p4.
- **p4** (all 15 seeds, six yaws per episode): **a stall is sticky.** Once an
  episode stalls, the next yaw usually stalls too, so per-episode ladders read
  as nonsense (theta 90 rescued 55 in p3 and failed it in p4). Any wrist
  comparison needs a full retreat between rungs.
- **p5** (all 15, descend in clear table space then walk in at z 0.95): 5/15
  held. Worse: the lateral move overshoots and bulldozes the carton (three
  seeds ended 170 mm past the command), and on six more the descent stopped on
  top of the carton at 0.9212. Rejected.
- **p6** (raster of xy offsets, no reset): showed a boundary but contaminated
  by stickiness.
- **p7** (absolute xy raster, **full retreat to the home pose between cells**)
  - the clean measurement:

  | seed 57, lowest z | x .02 | x .05 | x .08 | x .11 |
  |---|---|---|---|---|
  | y -0.060 | 1.065 | 1.065 | 1.065 | 0.939 |
  | y -0.050 | 1.065 | 1.065 | 1.064 | 0.938 |
  | y -0.040 | 0.937 | 0.932 | 0.926 | 0.923 |
  | y -0.030 | 0.937 | 0.931 | 0.926 | 0.923 |

  Seed 55 gives the same picture with the boundary at y = -0.033 instead of
  -0.045. **There is a descent envelope: with the wrist straight down the arm
  cannot reach grasp height below a y threshold near -0.04, and the threshold
  moves ~12 mm between seeds.** The stall heights are quantised (1.065, 1.095,
  1.106) and the envelope widens with x. The carton lands inside the strip on
  ~2 seeds in 15 - exactly seeds 55 and 63.
- **p8** (escape ladder, one rung per clean retreat, seeds 55/56/57/63):

  | rung | 55 | 56 | 57 | 63 |
  |---|---|---|---|---|
  | A yaw 0 | . | Y | Y | . |
  | B yaw 0, x+30 | . | Y | Y | . |
  | **C yaw +20** | **Y** | **Y** | **Y** | **Y** |
  | D yaw -20 | . | Y | Y | . |
  | E yaw 180 | . | Y | Y | . |
  | F yaw +20, x+30 | Y | . | Y | Y |
  | G stage at +x | Y | . | Y | . |
  | H closed jaws | . | . | . | . |

  (The later rungs on 56 fail because an earlier rung had moved the carton.)
  Yaw **+20 deg** reaches grasp height and lifts on 4/4, and the jaw span it
  needs is 80*sin20 + 44*cos20 = 69 mm, inside the 78 mm opening - the measured
  closed width is 0.042 either way, the same as at yaw 0.

### v3 - wrist-yaw rung ladder for the pick
Hypothesis: make +20 deg the primary descent wrist and fall back through
(+20, x+30), (0), (-20), using the eef z reading to tell a stalled descent
(z >= grasp_z + 0.020) from a landed one, with a retreat to the home pose
between rungs.

Receipt: `results/fs_..._v3` = **8/8**. Every seed took rung 0 on the first try.

Verdict: the pick is solved. But three seeds (51, 63, 65) used 773-792 sim
steps against 172-184 for the rest, and their `lowered` log line reads
z = 1.0821 against a 0.967 command: **the same envelope bites on the release**,
and `goto`'s bias-cancel turned a 27 mm stall into a 115 mm one by chasing a
command the arm cannot follow. Those three succeeded by dropping the carton
11 cm into the bowl, which is luck, not mechanism.

- **p9** (release-side envelope, empty gripper, clean retreat between rungs,
  seeds 51/57/62/63/65): descending to release height over the bowl stalls at
  0.994 for yaw 0, +-20 and 180 whenever the bowl centre sits below
  y = -0.055. **Yaw +90 landed on 5/5** (z 0.976, err 0.009); yaw +45 on 4/5.

### v4 - release-wrist ladder too
Hypothesis: after crossing, reorient at carry height and descend, trying
+90, +45, +20, 0 until the eef lands within 20 mm of release height. Plain
`api.move`, not `goto`, so a stall cannot be amplified. Rotating about the
vertical tool z only spins the pinched carton, so the grip survives.

Receipt: `results/fs_..._v4` = **8/8**, and now *every* seed takes rung 0 and
place rung 0 on the first try, holds (effort 3.0, width 0.042) from the close
all the way to the release, and lands the release at z 0.9738-0.9740 against a
0.967 command (6.8 mm). Sim steps 191-203 of the 1000 horizon on 7 of 8.

Verdict: selected for the formal run.

---

# DECLARATION

**Cell:** c2k1clean / `goal_put_cream_cheese_in_bowl_pos_k1`
**Frozen version:** v4.
`packs/c2k1clean_goal_put_cream_cheese_in_bowl_pos_k1/program.py`
md5 `4da022060762925c543a1b8ab51883de` == `program_v4.py` (verified on the cluster).

**Selection receipt (full 15 debug seeds, one formal run):**
`results/sel_c2k1clean_goal_put_cream_cheese_in_bowl_pos_k1_v4` = **15/15**
(seeds 51-65, every one `"benchmark_success": true`).
On all 15 the program took pick rung 0 (yaw +20, no offset) and place rung 0
(yaw +90) on the first try, held the carton at effort 3.0 / width 0.042 from
the close through the release, and landed the release 6.6-6.8 mm above the
0.967 command. Sim steps 191-204 of the 1000 horizon on 14 of 15 (seed 55: 794).

**Per-version receipt chain (probe subset 51,53,55,57,59,61,63,65 unless noted):**

| version | receipt dir | score | note |
|---|---|---|---|
| v1 | `fs_..._v1` | 6/8 | perception right everywhere; 55/63 stall |
| v2 | `fs_..._v2` | 6/8 | closed-loop goto: refuted, stall is a hard stop |
| v3 | `fs_..._v3` | 8/8 | pick rung ladder; release still stalls on 3 seeds |
| v4 | `fs_..._v4` | 8/8 | + release-wrist ladder; all first-try |
| **v4** | `sel_..._v4` | **15/15** | **formal selection run, all 15 debug seeds** |

Perception-only probes: `fs_..._p1` (sensor dump, 51-52), `fs_..._p2`
(15 seeds), `fs_..._p3` .. `fs_..._p9` (stall diagnosis).

**PROVENANCE:** present as a top-level literal dict in `program.py`, 20 entries,
every one sourced to `pack.json` (GRASP_Z, RELEASE_Z_OVER_RIM), to a debug-seed
measurement (TABLE_Z, CARRY_Z, HOVER_Z, TARGET_BAND, BLUE_MARGIN, GOAL_BAND,
GOAL_BRIGHT, GOAL_MERGE_R, OPEN_W, HELD_W, HOME, RUNGS, RELEASE_YAWS,
STALL_MARGIN, GOTO_TOL, GOTO_TRIES/STEP_CAP, FREE_LIFT), or to generic
camera/controller mechanics (GRID, R_DOWN). No LIBERO prior knowledge was used:
the table height, every prop height, the grasp width, the wrist yaws and the
reach envelope were all re-measured in this cell.

**Transferable finding.** The load-bearing discovery is not perceptual, it is
kinematic: *this arm has a descent envelope, and a stall inside it is sticky.*
With the wrist straight down the arm simply cannot reach grasp height below
eef y ~ -0.04, it stops dead at one of a few quantised heights (1.065, 1.095,
1.106), and once it has stalled the next attempt usually stalls too - so any
per-episode comparison of escape strategies is worthless unless each rung is
preceded by a full retreat to the reset pose. Measured that way the fix is one
number: yaw the wrist +20 deg for the pick and +90 deg for the place. A stall
is also directly readable (commanded z vs `api.eef()[2]`, a 140 mm gap), which
is what lets the program pick its own rung without any success feedback.

**STOP.**
