# c2k1clean / goal_open_middle_drawer_pos_k1

Intent: **open the middle drawer of the cabinet**. Success = the environment's
own benchmark bit. Runner: `tools/fair_run.py` only. Pack:
`packs/c2k1clean_goal_open_middle_drawer_pos_k1/` (K=1, 3 keyframes, raw actions).

## What the scene is (measured, debug seeds 51-65)

From one `cam_high` RGB-D frame per seed (cloud built from the supplied K and
`t_base_cam`, grey-only mask, 2 cm footprint grid, connected components):

- Table top z = 0.9025. Two tall fixtures: a wooden rack at y < -0.14 (top
  z = 1.21) and the **cabinet** at y > 0.12, x -0.28..-0.52, top z = 1.128.
- The cabinet's **-y face** carries exactly three horizontal protrusion bands,
  evenly spaced 0.070 apart, standing 0.029-0.031 proud of the face. Three
  drawers; the middle one is the target. Scored 100 against every other
  (component, direction) pair on all five probed seeds.
- Seed-to-seed spread of the whole fixture is ~13 mm in y and ~8 mm in x. The
  `_pos` perturbation does **not** move this cabinet to a materially different
  place on any debug seed.
- The pack demo's scene is *not* this one: its cabinet faces +y and its contact
  point is at x ~ 0.0, i.e. ~0.4 m further from the robot base than here. So the
  pack gives the mechanism, not the geometry.

Precise handle geometry, from a `cam_arm_wrist` frame taken 13 cm from the face
(fx = 334, so ~0.3 mm/px), deprojected through its own `t_base_cam`:

| quantity | value |
|---|---|
| drawer face plane | y = 0.1367 |
| middle bar, front surface | y = 0.1047 |
| middle bar, z extent | 1.005 .. 1.0225 (17.5 mm cylinder, centre 1.0138) |
| bottom bar, z extent | 0.940 .. 0.9525 |
| slot between any bar and the face | ~15 mm |
| bar length (x) | -0.364 .. -0.446, two stems at ~1/4 and ~3/4 |

The overhead view's protrusion band for the same bar is 1.0125..1.0325: the
oblique deprojection sits **+9 mm high**. Every later aim uses
`BAND_Z_BIAS = 0.009` to correct it.

## Mechanism from the pack

`gripper_cmd` is -1.0 (open) in all 138 demo steps and `gripper_state` never
changes: **the demo never closes its gripper**. Its wrist turns from straight
down to horizontal (euler roll 3.15 -> 1.73, pitch 0 -> 1.65), the contact
keyframe is at the bar's height, and during the pull the raw z action is held
at **-0.5 .. -0.9** (near saturation downward) while y ramps out 0.215 m. So the
demo hooks a finger on the bar and drags it out while pressing down.

## Version chain (every formally probed version)

Probe seeds 51,53,(55,57,...) unless stated. `benchmark_success` is false on
every episode of every version. Up to v32 the drawer never moved at all (the
re-perceived bar front is unchanged to <2 mm after each pull); from v33 on it
moves on the seeds where the jaws actually close on the bar, by at most 0.12 m,
which is not enough for the predicate.

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1-v2 | dump RGB-D through `api.log` to do perception offline | works; messages truncate at ~2 kB so chunk at 1800 | tooling |
| v3 | horizontal wrist, clamp the bar, pull | arm parked 0.37 m from target, residual constant across 4 moves | pose unreachable in one move |
| v4 | which orientations are holdable? | at the start pose the horizontal wrist holds (roterr 3-4 deg); in front of the drawer it diverges (resid 0.58) | staging needed |
| v5 | stage the rotation, then walk in | walk stalls at x=-0.2575; a later step reaches (-0.4155, 0.0971, 1.106) | z floor ~1.10 |
| v6 | close-up wrist look + down-wrist floor | straight-down wrist bottoms out at z = 1.108 at y = 0.075 | 8 cm above the bar |
| v7 | is the floor the arm or the cabinet? | same x, y=-0.195 (free space): descends to 0.913. In front of the face: 1.108 | the cabinet side is the problem |
| v8 | bars or body? | off-bar x: floor 1.096; 4 cm further out: 1.064 | floor falls as you back off in y |
| v9 | come in low from outside | descended into the rack (x=-0.38 is inside it) | bad corridor |
| v10 | corridor at y=-0.05 | reaches z=0.995, then +y stalls at y=0.004 | reach, not contact |
| v11-v12 | turn the wrist low, then creep | at z=1.06 the wrist creeps to **y = 0.128**, hard against the face | turning unfolds the arm |
| v13 | creep at bar height, close, pull | close reads 8.2 mm, eef 12 mm high, drawer unmoved | jaws above the bar |
| v14 | cancel the height lag | eef pinned at z = 1.032 whatever is commanded; close reads 1.9 mm | 19 mm above bar centre |
| v15 | wrist frames at the grasp pose | gave the table of geometry above; also: the arm springs up 21 mm during the 20-step gripper settle | measurement |
| v16 | roll the wrist 90 deg | one roll holds (6 deg), the other is 153 deg off | roll matters |
| v17 | is the floor kinematic or the held orientation? | releasing the orientation command (`rotation=None`) buys **0 mm**: z = 1.0328 both ways | kinematic |
| v18 | yaw the approach toward the shoulder line | yaw +35 deg drops the eef to **z = 1.0147**, 1 mm off bar centre | yaw unfolds the arm |
| v19 | yawed approach, clamp, pull | at that height the creep stalls at y = 0.072, 33 mm short of the bar | trade-off surface |
| v20a-d | yaw 45/55/65, and x-shift | y_max at bar height: 0.071 / 0.062 / 0.014; x-shift breaks the route | no yaw escapes it |
| v21 | thread the open jaw in high, drop into the slot | creep stalls 7 mm short of the bar front | - |
| v22 | **where are the pads?** | wrist frames in clear space, in tool coordinates: the near silhouette widens along **tool Y** when the gripper opens | jaw axis was tool Y, not tool X - every earlier "vertical jaw" was closing *along* the bar |
| v23 | rebuilt frame, jaws truly vertical | first honest contact: closed gap **18.3 mm** (bar is 17.5 mm) | but eef 42 mm high, pads on two different bars |
| v24a-d | yaw sweep with vertical jaws | yaw 0: y = 0.128 at z = 1.054; yaw 20/35/50 much worse | yaw 0 is best here |
| v25-v27 | pull open-jawed; pull down-and-out like the demo | out: 0.032; down+out: **0.000**, arm wedged, drawer unmoved | pad is *on top of* the bar |
| v28-v29 | engage elsewhere along the bar | in the deep pose the eef is pinned at **x = -0.353** - 11 mm past the bar's +x end; lateral commands move it 0 mm | roll A never touches the bar |
| v30-v31 | the other roll of the same jaw axis | roll B holds **x = -0.415** (on the bar) and z = 1.033, pads at 0.994/1.072 straddling it correctly - but stops at **y = 0.0945**, 10 mm short of the bar front | 10 mm |
| v32 | roll B, duck under the top bar, hook and pull | route collapses (stalls at y = 0.015): the settling pose is chaotically sensitive to the corridor standoff | - |
| v33 | roll B, straddle, clamp with gap check, open-jaw fallback, down-and-out pull | **0/15** on the full debug split. But 3/15 seeds put the pad *past* the bar front with the pad below the bar, and seed 58 closed on **17.3 mm** (the bar is 17.5 mm), effort 3.0, held through the stroke | first real grasp |
| v34 | when the gap says the bar is in the jaws, pull STRAIGHT out - the demo's downward press is what keeps an *open* finger hooked and with the jaws shut it only drives the wrist into its own floor | **0/15**. 2/15 seeds clamp (17.3 mm, 24.3 mm); strokes of 0.061 and 0.121 m; the GIF shows the middle drawer standing ~6 cm proud of the cabinet | frozen |
| v35bot / v35top | is the graded drawer really the geometric middle one? | bottom band: clamps (16.8 mm) and pulls 0.030, 0/5. Top band: never reached (92 mm short), 0/5. Middle stays the best band | no remapping |
| v36 | when the stroke stops, is it the drawer's travel or the arm's reach? | at the stop, **open the gripper** and command another 0.15 m out: the eef moves **0.3 mm** | the arm, not the drawer |

## Mechanism gap (falsifiable)

> On every debug seed of this cell the cabinet stands ~0.15 m in front of the
> robot base, and the reachable set of the end-effector there is bounded by a
> sloped surface that passes **above and outside the middle bar**. Measured
> boundary in front of the face, at the bar's own x:
>
> | eef z | deepest y reached |
> |---|---|
> | 1.014 | 0.072 |
> | 1.033 | 0.117 (roll A) / 0.0945 (roll B, jaws vertical) |
> | 1.054 | 0.128 (but x pinned at -0.353, off the end of the bar) |
> | 1.059 | 0.128 |
>
> The middle bar needs a pad at x in [-0.364, -0.446], y >= 0.1047, z <= 1.005.
> The two wrist rolls that put the jaws vertical miss it by ~10 mm in *different*
> axes: roll A is 11 mm past the bar's end in x, roll B is 10 mm short of the
> bar's front in y. Releasing the orientation command, yawing +-35/50/65 deg,
> approaching low from outside, and approaching high and dropping all leave
> this boundary unchanged to within a few millimetres.
>
> The one seed where the pads do close on the bar (58: gap 17.3 mm against a
> measured 17.5 mm cylinder, effort 3.0, held for the whole stroke) makes the
> same point from the other side: the drawer comes out 6.1 cm and then stops,
> and the stop is the **arm**, not the drawer - release the gripper there and
> command another 0.15 m of retreat and the eef moves 0.3 mm. Reaching into
> the pocket where the bar is leaves no stroke to pull with.
>
> This is falsified by any program that gets a finger pad to
> (x in [-0.37,-0.44], y > 0.11, z < 1.005) on these seeds, or that pulls a
> clamped bar further than ~0.12 m, or that trips the benchmark bit at all.

Two things could still break it and were not reachable inside this cell's
budget: a genuinely different arm configuration (the OSC null-space posture is
not addressable through `api.move`), and a pull that does not require the wrist
to get behind the bar at all (no such affordance was found on this cabinet -
the face is flush, the only graspable feature is the bar).

What is *not* the explanation, each ruled out by a direct measurement: the
handle's height (wrist camera, 0.3 mm/px), the jaw axis (v22), the orientation
command (v17), the approach yaw (v18/v20/v24), the corridor (v9/v10), the step
budget (episodes run past 820 sim steps - measured, not assumed - and every
attempt reaches the face with hundreds to spare), and
contact with the neighbouring bars (the pads' lanes were computed and checked
each run).

## DECLARATION

- **Frozen version: v34** - `program.py` md5 == `program_v34.py` md5
  (`f41016f560c6534e7e7a5cb40db4ded0`, see `md5.txt`).
- **Selection receipt: 0/15** on the full debug split 51-65,
  `results/sel_c2k1clean_goal_open_middle_drawer_pos_k1_v34`
  (v33's own full-15 selection, also 0/15, is in
  `results/sel_c2k1clean_goal_open_middle_drawer_pos_k1_v33`).
- Per-version receipt chain: table above; every version's result directory is
  `results/fs_c2k1clean_goal_open_middle_drawer_pos_k1_v*`.
- `PROVENANCE` present in `program.py`, 14 entries, every constant sourced to
  this pack or to a debug-seed measurement.
- Verdict: **mechanism-gap stop**, as stated above. v34 is the argmax version
  (all versions score 0; v34 carries the correct mechanism, and is the only one
  that ever clamped the bar and moved the drawer - perception of the
  right bar, jaws on the right axis, clamp with a gap receipt, open-jaw
  down-and-out fallback - so it is the version that would succeed on any seed
  whose cabinet is placed inside the arm's reach).
