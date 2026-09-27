# c2clean / goal_open_middle_drawer_pos_k3

Intent: **open the middle drawer of the cabinet**. Runner: `tools/fair_run.py` only.
Pack: `packs/c2clean_goal_open_middle_drawer_pos_k3/` (K=3).

## What the pack says

* `actions[:, -1] == -1.0` at **every** timestep of all three demos (138/138/151).
  The gripper is open for the whole episode: the handle is **dragged, never
  gripped**.
* `ee_path`/`ee_path6`: the tool starts straight down, turns nose-down-and-
  forward, drives into the drawer face along one horizontal axis, and then
  translates **0.175 m** straight back out along that same axis.
* Reading `ee_path6[:, 3:6]` as **rotation vectors** (not rpy) is what makes the
  demo consistent: the tool approach axis then points along the travel
  direction. Under rpy it points 90 deg away from the motion.
* Tilt profile of the approach axis below horizontal: 87 deg (home) ->
  ~20-40 deg through the whole contact phase (38/21/31 deg at the deepest
  insertion of demos 0/1/2). The demo is **nose-down** on the handle, not
  level.

## Scene (all from cam_high / cam_arm_wrist depth on debug seeds 51-65)

* The cabinet is the only structure with top z = 1.128; it sits at
  x in [-0.53, -0.27], y in [0.09, 0.33] on every debug seed (it translates by
  about +-0.01 m seed to seed, so it is perceived, not hardcoded).
* Its drawer face is the **-y** face (plane y ~ 0.12-0.14 by seed), i.e. this
  scene is **mirrored** relative to the pack's demo scene, where the cabinet
  sat at -y and the drawer opened toward +y. The face finder tests all four
  vertical faces and keeps the one that (a) sits on the bbox extreme in that
  direction and (b) carries proud rows.
* Three handle bars stand **0.030 m proud** of that face at z ~ 0.95 / 1.02 /
  1.09; the middle one is 0.085 m wide in x, its top is at z ~ 1.023-1.033.
* Fingertip offset from the eef along tool z: **0.0093 m**, measured by pressing
  the open gripper straight down onto the measured cabinet top (z = 1.1278).

## Version chain

| ver | change | receipt |
|---|---|---|
| v0 | sensor dump only (no motion) | RGB-D of 15 debug seeds; found the cabinet, the -y face, three handle bands |
| v1 | face finder + open-finger insert + pull, 15 deg tilt | 0/4 (51,53,55,57). Face finder picked the wrong face (+y, plane 0.233): proud-count scoring was contaminated by the +x face |
| v2 | proper 4-face scan (bbox-extreme + proud rows) | 0/4. Correct face; fingertips passed **above** the bar (tip z 1.037) and touched the drawer face; the pull caught nothing |
| v3 | wrist metrology at two standoffs | not scored. Head-on wrist depth: face plane 0.1354, bar front 0.105, bar 0.088 wide, z 1.00-1.03 |
| v4 | vertical-finger pinch on the bar | 0/4. The straight-down + 90 deg yaw pose is unreachable at the drawer; the arm froze at z 1.19 |
| v5a/b/c | fingertip calibration in-episode + 3 grasp heights | 0/3 each. The in-episode calibration was contaminated: the moves starved (each `api.move` gets max(60, 20*seconds) steps, so 0.8 s and 2.5 s moves cost the same 60) |
| v6cal | table press + face press calibration | tip offset 0.0924 from the table press -- later shown to be a workspace z-floor at z ~ 0.994, not contact |
| v7top/face | press onto the **cabinet top** (well above any floor) | **tip offset = 0.0093**. The face press stalled early: big single moves lock the arm up; gradual creeps track |
| v8 top/upper/centre/lower | 4 engagement heights, 15 deg tilt | 0/2 each. All four stall at tipfront 0.028-0.030 = the bar front. The bar is solid and 0.085 wide vs a 0.080 finger gap: the fingers can never straddle it |
| v9diag | close the gripper at contact, effort receipt | effort 0.05, width 0.001 -- nothing between the fingers, so it is not a pinch |
| v10look | wrist views at 55 / 15 / -35 deg | The 55 deg down view sees the bar top end 0.021 m short of the face and sees the face **below** the bar top through that gap: the bar is a rail with a slot behind it |
| v11 | drop straight-down fingers into that slot | 0/3. The +90 deg yaw deadlocks mid-turn (45 deg short) |
| v12a/b | staged wrist turn, both yaw signs | 0/2 each. **YAW = -1 converges** (rot err 0.009); but with a straight-down tool the eef cannot get closer than 0.036 m to the face, so the fingers never reach the slot |
| v14a/b | nose-down 30 / 45 deg, reach the face, press down, drag | 0/2 each. Both reach tipfront ~0.002 (fingertips at the face) and press to tip z 1.024 -- the demo's exact contact state -- but the drag at 0.035 m steps slips |
| v15/v16 probe | contact cross-section of the handle, open and closed gripper | inconclusive: the OSC has configuration-dependent z floors that masquerade as contact |
| v17 | 45 deg, finer 0.025 m drag steps | 0/3. Retracts smoothly for 0.118 m with no load: at 45 deg the fingertip rides over the bar |
| v18look | cam_high dump at the contact pose | confirmed the pose; the arm is where the log says |
| v19 a-f | sweep tilt x engagement height: (45,+.016) (45,+.006) (45,-.004) (60,+.016) (60,+.004) (70,+.010) | **(60, +0.004) = 2/2**, (60,+0.016) = 1/2, everything else 0/2 |
| v19e | same, widened to seeds 53,57,59,61,63,65 | **6/6** (8/8 over all probe seeds) |
| **v20** | v19e with the PROVENANCE dict completed (logic identical) | **14/15 formal selection** |
| v21 | 11 x 0.019 m drag steps instead of 8 x 0.025 | 4/5 and burns the step budget (up to 1000); no better on the failing seed |
| v22 | press command 0.050 m below the bar top instead of 0.030 | **0/4** -- the press depth sets the fingertip's resting height through the controller's force balance; 0.030 is load-bearing |

## Mechanism

Three things have to be right together, and each was falsified separately above:

1. **Nose-down 60 deg.** At 15-45 deg the fingertip either stalls on the bar
   front (too low) or rides over it with no load (too high). At 60 deg the
   finger's lower-front corner sits against the bar.
2. **Fingertips 0.004 m above the wrist-measured bar top.** +0.016 over-shoots,
   -0.004 stalls on the bar front.
3. **A z command 0.030 m below the bar top held through the whole drag.** A
   converged z command releases the contact and the finger slides off;
   0.050 m below over-drives and loses it too (v22, 0/4).

The insertion then stops at tipfront ~0.020 (not at the face), the fingertip
loads the handle, and 8 x 0.025 m outward steps drag the drawer to its stop --
visible in the log as the drag freezing at tipfront ~0.16 while tip z holds.

## DECLARATION

* **Frozen version: `program.py` == `program_v20.py`, md5
  `f78418b049f6185f6d9a8a0f6b01b122`** (verified on the cluster copy too).
* **Selection receipt: 14/15 on the full debug band (seeds 51-65),
  `results/sel_c2clean_goal_open_middle_drawer_pos_k3_v20`.**
  Per seed: 51 52 53 54 55 57 58 59 60 61 62 63 64 65 succeed; **56 fails** --
  it loses the hook at 0.140 m of drag where the successful seeds stall at
  0.160 m, i.e. it slips ~0.02 m short of the predicate.
* Per-version receipt chain: the table above; every formally probed version is
  archived as `program_vN.py` in the pack directory.
* `PROVENANCE` is present as a top-level literal dict with 14 entries covering
  every calibrated constant (PULL_AXIS, PULL_LEN, OPEN_DRAG, TILT, FINGER_AXIS,
  DZ, PRESS, TALL_BAND, PROUD_THRESH, TIP_OFF, LOOK_TILT, STANDOFF, SLOT_W,
  GRIP_W). Sources are the pack's own fields plus debug-seed (51-65)
  observations only.
* Splits respected: only `--split debug` was ever run, only seeds 51-65; eval
  seeds 1-50 were never touched. `tools/fewshot_run.py` was never invoked.
