# c2k1clean / spa_bowl_on_stove_task_k1

Intent: **"Pick the akita black bowl on the top of the cabinet and place it on
the plate."**  Runner: `tools/fair_run.py` only. Split: debug = seeds 51-65;
probe subset = 51,53,55,57,59,61,63,65; selection = all 15.

## Scene, as read from the debug seeds (no benchmark files opened)

`cam_high` gives 512x512 RGB + dense metric depth + K + T_base_cam, so the
whole scene is available as a base-frame point cloud. Reading it:

| thing | measurement (seeds 51-65) |
|---|---|
| table plane | z = 0.903 (modal height of the workspace cloud) |
| cabinet top face ("the shelf") | z = 1.125 (modal height 0.10-0.40 above the table) |
| bowl on the cabinet | rim top z = 1.180, Kasa rim radius 0.0540 +- 0.0001, ~3400-4200 px above the shelf |
| plate | floor z = 0.908, rim crest 0.920 at r=0.065, outer d = 0.135-0.137 |
| other props | a bowl on the stove, a third bowl on the table (rim 0.944), a cookie box (d<0.09) |
| gripper | opens to width_m 0.0778; jaws translate along base **x** |

Three look-alike bowls stand in the scene, so the target is identified by the
only thing the intent actually says about it -- it is the one on the cabinet,
i.e. the sole prop standing on the raised shelf. No appearance cue is used.

## Version history (hypothesis -> evidence -> verdict)

**v0 (probe).** Capture only; dumped cam_high/cam_arm_wrist RGB-D + eef for
all 8 probe seeds so perception could be developed offline. Verdict: gave the
table/shelf/bowl/plate numbers above; perception agreed across all 8 seeds
(bowl rim radius 0.0540-0.0542, rim top 1.180 every time).

**v1 (x4: rim pinch, four sides).** Hypothesis: the jaws (0.078 m) cannot span
the bowl (0.108 m), so the grasp must be a rim pinch -- descend one rim radius
off centre, one finger in, one out, and close on the wall. Which side is
unknown, so all four were run: `+y`, `-y`, `+x`, `-x`, 8 seeds each.
Evidence: `fs_..._v1yp` 0/8, `v1ym` 0/8, `v1xp` 0/8, **`v1xm` 2/8**.
Verdict: the jaw axis is base **x** and the tool must sit *behind* the bowl
centre. Gifs of the v1xm failures showed the bowl carried to the plate and set
down *beside* it -- the tool was over the plate centre, but the bowl hangs one
rim radius ahead of the tool, so the bowl missed by that radius.

**v2 (closed-loop waypoints + verify + re-grasp).** Hypothesis: drive every
waypoint closed-loop off `api.eef()` (one `api.move` settles ~0.010 m short),
verify the lift by re-reading the shelf, retry deeper on a miss.
Evidence: 0/8, and `sim_steps` = 1000 in *every* episode. Verdict: **the
episode horizon is 1000 sim steps** and the retry ladder exhausts it -- the
arm simply stops moving mid-transport (logged residuals jump to ~0.5 while
the eef stays put). Also: pinching 0.021 m below the rim *after* the residual
was cancelled put the hand body on the rim and shoved the bowl ~0.03 m
forward instead of gripping. A cheap, fixed, short trajectory is required.

**v3 (v1xm motion + release offset).** Carry the grasp offset into the release
point; take the carry drop from the *measured* eef z at closing minus the
shelf z. Evidence: **3/8** {51,55,57}, 252-444 steps. Grip width at closing
0.0071-0.0102 and still 0.0049-0.0062 at the plate in *all eight* episodes ->
**the grasp and the carry are not the problem**.
  - **v3a** (pinch 0.014 below rim): 0/8, two episodes hit the 1000-step wall.
  - **v3b** (pinch 0.028 below rim): 4/8 {53,57,63,65}.
  v3 and v3b succeed on near-complementary seeds with near-identical logs,
  which said the remaining error was not in the grasp but in *where the bowl
  ends up*.

**v4 (measure the carried bowl from the wrist camera).** Hypothesis: measure
the tool-to-bowl offset in flight and aim the release by it. Evidence: 3/8;
the wrist fit locked onto the hand body, not the bowl (`rim` came back 0.035
*above* the eef, `dy` exactly 0.000, radius 0.034-0.059), and the guard
rejected it every time, so v4 degenerated to v3. Verdict: the wrist camera
sits too close -- at carry height the bowl half fills the frame and its near
arc is behind the hand. Abandoned.

The v4 run did pay for itself: its end-of-episode readout showed that in the
failures the bowl *is* on the plate, centred within ~0.01 of it, but with its
base at 0.923 -- the plate's **rim** height -- not 0.908, its floor. The plate
is a dish whose floor only reaches r=0.045, so a bowl set down a couple of
centimetres off centre perches on the rim instead of seating.

**v5probe.** Dumped carry-time and end-of-episode RGB-D to disk and
Kasa-fitted the rim of the bowl where it came to rest:

| seed | resting bowl centre | plate centre | miss |
|---|---|---|---|
| 53 | (0.036, 0.226) | (0.060, 0.204) | (-0.024, +0.022) |
| 63 | (0.041, 0.229) | (0.058, 0.207) | (-0.017, +0.022) |
| 65 | (0.044, 0.222) | (0.067, 0.199) | (-0.023, +0.023) |

A **fixed** miss, same direction and size every seed: the wall does not settle
at the tangent point the nominal geometry predicts but a few tens of degrees
around the rim (measured tool-to-bowl offset ~(+0.032, +0.025), magnitude
0.041 = the rim radius at pinch depth, rather than the assumed (+0.045, 0)).

**v5 = v3 + `PLACE_BIAS = (0.021, -0.022)`.** Cancel the measured residual.
Evidence: probe subset **8/8**. Every episode terminated early (the predicate
fires as the bowl reaches the plate), which is why the end-of-run readout
reports "nothing standing on the plate" -- the sim is frozen with the arm
over the plate and the arm keepout masks it.

**v6 = v5 + an internal command budget.** The two v5 losses (56, 64) both
burned the full 1000-step horizon. Diagnosis: the descent to the rim jams on
those seeds (descent residual 0.022-0.023 against 0.010-0.012; the tool ends
0.018 forward of the rim point instead of 0.003, and the bite is thin --
width 0.0038-0.0049 against 0.0071-0.0102). The arm is then strained for the
rest of the episode, every later move misses its tolerance, the fixed 3-try
ladder retries all of them, and the episode dies mid-transport. v6 stops
retrying a move that gains less than 0.003, and reserves budget so the
release always runs. Evidence: probe on 51,52,54,56,58,60,62,64 = 6/8, with
`moves used 7` on the two hard seeds instead of ~24. Verdict: the guard works
(commands bounded) but the two seeds still fail -- their loss is the jam
itself, not the budget. On the full 15 v6 reproduces v5 seed-for-seed with
identical `sim_steps` on every episode, so the guard is inert on debug and
purely protective off it.

**v7 = v6 + a lateral re-aim before closing.** Hypothesis: cancel the 0.018 m
forward slip at the pinch before the gripper closes. Evidence: 6/8 on the same
probe, and the logs show why it is worse in mechanism -- the corrective move,
commanded at the achieved height, actually rose to z=1.185 (above the 1.180
rim), so the jaws closed on air (width 0.0010) and `hang` came back 0.060.
Verdict: **rejected**; a lateral command in a jammed pose does not hold height.

## Mechanism gap (the residual 2/15)

Falsifiable statement: on ~13% of seeds (56 and 64 of 51-65) the vertical
descent to the rim-pinch point jams ~0.013 m above its target and the tool is
displaced ~0.018 m in +x, which both spends the step horizon and thins the
bite so that PLACE_BIAS -- calibrated on the clean bite -- no longer aims the
bowl at the plate. What is missing is a way to reach the pinch depth in those
poses. The three levers reachable through the fair API were tried and each is
receipted above: a shallower pinch (v3a, 0/8), a deeper one (v3b, 4/8), and a
lateral re-aim at depth (v7, jaws close on air). A fix would need either a
wrist rotation to re-approach the rim on a different bearing, or a measurement
of the carried bowl good enough to re-aim the release per-episode -- the
wrist camera cannot supply the latter (v4: at carry height the bowl half
fills the frame and its near arc is behind the hand).

## DECLARATION

- **Frozen version: v6.** `packs/c2k1clean_spa_bowl_on_stove_task_k1/program.py`
  md5 `2ea8666ed3113dc3de6c178b0af0c7bd` == `program_v6.py` (verified on the
  cluster).
- **Selection receipt (full 15 debug seeds, 51-65): 13/15** in
  `results/sel_c2k1clean_spa_bowl_on_stove_task_k1_v6`.
  Failures: seeds 56 and 64 only (the jam above). v5 has an equal full-15
  receipt, `results/sel_c2k1clean_spa_bowl_on_stove_task_k1_v5`, **13/15**,
  failing the same two seeds with the same per-seed `sim_steps`; v6 is frozen
  over v5 because its command budget is inert on debug yet bounds the
  starvation cascade that cost v2 every episode.
- **PROVENANCE:** present, 18 entries, all `allowed: True` with a pack field or
  debug-seed measurement as source; the program passes the eval gate's token
  scan and reads no `.done`.

### Receipt chain

| version | run dir (`results/`) | seeds | score |
|---|---|---|---|
| v0 probe | `fs_..._v0` | 8 probe | scene capture only |
| v1 `+y` | `fs_..._v1yp` | 8 probe | 0/8 |
| v1 `-y` | `fs_..._v1ym` | 8 probe | 0/8 |
| v1 `+x` | `fs_..._v1xp` | 8 probe | 0/8 |
| v1 `-x` (kept as `program_v1.py`) | `fs_..._v1xm` | 8 probe | 2/8 |
| v2 | `fs_..._v2` | 8 probe | 0/8 |
| v3 | `fs_..._v3` | 8 probe | 3/8 |
| v3a | `fs_..._v3a` | 8 probe | 0/8 |
| v3b | `fs_..._v3b` | 8 probe | 4/8 |
| v4 | `fs_..._v4` | 8 probe | 3/8 |
| v5probe | `fs_..._v5probe` | 51,53,63,65 | 1/4 (instrumentation run) |
| v5 | `fs_..._v5` | 8 probe | 8/8 |
| **v5** | **`sel_..._v5`** | **all 15** | **13/15** |
| v6 | `fs_..._v6` | 51,52,54,56,58,60,62,64 | 6/8 |
| **v6 (FROZEN)** | **`sel_..._v6`** | **all 15** | **13/15** |
| v7 | `fs_..._v7` | 51,52,54,56,58,60,62,64 | 6/8, rejected |

Probe subset = 51,53,55,57,59,61,63,65. No eval seed (1-50) was ever run or
inspected; no benchmark asset was opened.
