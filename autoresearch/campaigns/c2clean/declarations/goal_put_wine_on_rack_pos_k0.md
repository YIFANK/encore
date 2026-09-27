# c2clean / goal_put_wine_on_rack_pos_k0

Intent: **put the wine bottle on the rack**. No demonstration pack (k0).
Runner: `tools/fair_run.py` only. Debug seeds 51-65. Eval seeds 1-50, blind.

## Scene, as measured from debug seeds (no prior knowledge used)

All numbers below are re-derived in this cell from cam_high / cam_arm_wrist
RGB-D on debug seeds only.

- Table plane `z = 0.9010` on all 15 debug seeds (modal depth plane). The raw
  cam_high cloud reaches `x = -1.99` (back wall), so the workspace must be
  cropped before the table is fitted.
- Above-table props (seed 51, representative):
  | prop | x | y | height over table | mean rgb |
  |---|---|---|---|---|
  | rack (tan wood) | -0.10 .. 0.17 | -0.367 .. -0.102 | 0.327 | (126,109,96) |
  | cabinet (grey deck) | -0.387 .. -0.131 | -0.367 .. -0.146 | 0.227 | (66,65,63) |
  | wine bottle (near-black) | 0.041 .. 0.068 | -0.042 .. 0.001 | 0.157 | (20,22,15) |
  | robot arm | -0.45 .. -0.03 | -0.105 .. 0.306 | 0.445 | (48,57,63) |
  | blue box | -0.081 .. -0.001 | 0.111 .. 0.152 | 0.019 | (66,72,89) |
- **Rack surface geometry** (unoccluded wrist top-down, v2). The top is a
  smooth monotone ramp, invariant in x and descending in +y:
  `h = 0.305` at `y = -0.276` down to `h = 0.236` at `y = -0.160`
  (slope 0.578, ~30 deg). A raised rail reaches `h = 0.343` at
  `y = -0.296..-0.276`; a narrow flat ledge (`h ~= 0.12`, 3.4 cm wide) sits at
  `y = -0.158..-0.124`; then table. **No grooves** at 2 mm resolution, so the
  ramp is effectively a solid inclined plane.
- Bottle vertical profile (wrist top-down): neck 0.006-0.015 m wide at
  h 0.12-0.15; shoulder widens to 0.037 m at h 0.06-0.10; footprint 0.042 m.
- Cross-seed spread: the bottle centre moves over
  `x 0.041..0.062, y -0.035..-0.009`; the rack shifts ~0.015 m in y. A
  perception-driven program is required, not fixed coordinates.
- **Seed 64 starts with the bottle already lying on its side** (h ~ 0.04, not
  0.157) and the rest of the scene rearranged. 1 of 15 debug seeds. Any
  height-gated bottle detector must handle this case or it finds no bottle.

## Harness mechanics measured in this cell

- **Episode budget** (v4): fourteen 1.0 s `api.move` calls fit; after that the
  eef stops responding entirely and every later move returns a residual equal
  to the full remaining distance. A move whose target is unreachable burns its
  whole allowance.
- **`api.gripper()` post-horizon tell** (v3 vs v4): once the episode is over,
  `gripper()` reads `width 0.080 / effort 3.000` regardless of the real state.
  A free-space `api.grip(0.0)` on a live episode reads `width 0.001 /
  effort 0.050`. So **closed width, not effort, is the hold sensor here.**
- Reach (v2): every surveyed hover pose over the table and the rack was
  reached at `z = 1.42` with residual <= 0.023.
- Wrist camera sits at `eef + [0.049, 0.000, 0.098]` with the wrist straight
  down; `u -> -y_base`, `v -> -x_base`.

## Version log

### v1 - perception probe (no motion). seeds 51,53,55,57 then 52-65.
Hypothesis: the scene can be measured well enough from cam_high alone.
Evidence: zlib+base64 RGB-D through `api.log` reconstructs the scene offline.
Table plane, all props and their colours recovered; but the agentview is
oblique, so the rack's own shadow hides its far half.
Verdict: perception plumbing good, rack geometry not yet trustworthy.
Detector dry-run over all 15 seeds: bottle found on 14/15 (miss = seed 64,
bottle lying down); rack ramp isolated and `hsurf = 0.283` on **all 15**.

### v2 - wrist survey (no grasp). seeds 51,55.
Hypothesis: a top-down wrist view removes the occlusion and settles the rack.
Evidence: eight hover poses all reached (residual <= 0.023). Rack resolved as
the 30-deg ramp described above.
Verdict: geometry settled; reach over the whole rack confirmed.

### v3 - first end-to-end pick & place. seeds 51,53,55,57. **0/4.**
Hypothesis: measure the fingertip offset by pressing the open gripper on bare
table, then grasp the neck and place mid-ramp.
Evidence: the press stalled at `eef z = table + 0.0082` (so TIP_OFF ~ 0.008),
but **every move after it did nothing** - eef frozen at [0.133,0.118,1.182],
residuals equal to full distance, gripper stuck reading 0.080/3.000.
Verdict: refuted, and the reason is budget, not aim. The blocked press ate the
episode. Also learned: `effort 3.0` here is the post-horizon artefact.

### v4 - budget + gripper probe. seeds 51,55.
Hypothesis: v3's freeze is either an exhausted step budget or a wedged
controller.
Evidence: 14 alternating 1.0 s moves executed, then `FROZEN_AT it=14` with
identical eef readings; the gripper closed cleanly to 0.001 in free space
before the loop.
Verdict: budget, confirmed. Design rule adopted: <= ~10 moves per episode.

### v5 - lean pick & place (7 moves, no calibration press). seeds 51,53,55,57. **0/4.**
Hypothesis: with the budget respected, a body grasp and a release mid-ramp
puts the bottle on the rack.
Evidence: the whole sequence ran, but the single long descend from the hover
landed 0.032 m off in x (target x 0.0545, achieved 0.0864) and the jaws closed
on air (width 0.001). The bottle was never touched.
Verdict: refuted. `api.move` does not land where it is told on a long, mostly
vertical descent - it drifts +x and stops ~0.019 m high.

### v6 - closed-loop re-aim. seeds 51,53,55,57. **0/4.**
Hypothesis: re-commanding target + (target - achieved) cancels the bias.
Evidence: it does converge (pregrasp reached 0.005 m), but each fix only
closes ~55% of the error, so it took 2-3 moves per waypoint; the episode ran
out of budget at the 9th move and the release never happened. Grasp still
caught air at 0.0153 m of x error.
Verdict: right idea, too expensive. Budget must be spent on at most one fix
per waypoint.

### v7 - grasp-height sweep (probe). seeds 51,55.
Hypothesis: either the fingertip offset is not 0.008 m, or the xy never
arrives; a ladder of commanded heights with a closed-width readout separates
them.
Evidence, decisive:
- commanding eef z = table + 0.150 closed the jaws on the **neck**:
  `width 0.0149, effort 3.000`.
- pressing lower while holding stalled the eef at table + 0.120, putting the
  **fingertips ~0.021 m below the reported eef**.
- so `effort 3.0` genuinely means "jaws blocked"; v3's 3.0 came paired with
  width 0.080, which is the post-horizon artefact. Width plus effort together
  are unambiguous.
- xy: 0.006 m of error caught the neck, v6's 0.0153 m caught nothing. The
  grasp needs |dx| <~ 0.010.
Verdict: fingertip offset and the xy tolerance both settled.

### v8 - neck grasp + one re-aim per waypoint. 8 probe seeds. **0/8.**
Hypothesis: v7's neck height plus a single xy fix per waypoint fits the budget
and grasps.
Evidence: the xy correction works well (landed 0.6-3.4 mm off target) and the
whole sequence now fits. ep51 **did** close on the neck (width 0.0122,
effort 3.0) - and then the bottle **slipped straight back out on the lift**
(width fell to 0.001). ep55 caught nothing at the same commanded height.
Verdict: the neck is the wrong feature - a 0.015 m target with only a ~0.02 m
usable height band, and it does not survive the lift.

### v9 - body grasp, measured hang.
Hypothesis: the body (0.042 m across, holding that width over a 0.07 m tall
band from 0.02 to 0.09 m over the table) is wide enough that the z droop
cannot miss it and firm enough to survive the lift. Also stop guessing the
hang: at the instant the jaws close the bottle's base is still on the table,
so `hang = eef_z_at_close - table_z` exactly.
Evidence: probe 51,53,...,65 **8/8**; full-15 selection
`results/sel_c2clean_goal_put_wine_on_rack_pos_k0_v9` **11/15** (fails 52, 54,
62, 64). The body grasp works: every seed that actually reached the commanded
grasp height closed to 0.0302-0.0307 and held through the lift and the place,
and the measured hang put the release right. All four failures share one
cause: the eef never got down to the commanded height. It stalled 0.023-0.049
high (ep52 1.0191, ep54/62 0.9941) and the jaws closed on air.
Verdict: the grasp and the place are settled; the DESCENT is the open problem.
**Do not conclude on the probe subset** -- the 8 odd seeds were 8/8 while the
even seeds were 3/7.

### v10 - press moves anchored on the achieved eef. probe 52,54,56,58,60,62,64. **3/7.**
Hypothesis: v9's fix move cannot rescue a stalled descent because the
bias-cancelled command `cx` has drifted far from where the eef actually is, so
the whole move goes sideways (descend and descend_fix end at the SAME z to
0.3 mm). A command built from the achieved eef is nearly pure-vertical.
Evidence: ep52 freed (press0 gained 0.0406, closed 0.0306, success -- so that
stall was a starved move, not a block). But ep54/62 gained 0.0006 over two
presses, and ep58 (a v9 success) was over-corrected into an air grasp by the
2x bias cancel.
Verdict: half right. A press aimed AT the grasp height is too weak a command.

### v11 - deep press (aim 0.045 below), 1.5x cancel. same 7 seeds. **4/7.**
Hypothesis: what freed ep58's descent in v10 was an accidentally LARGE command
(dx -0.036, dz -0.093). Aiming the press below the grasp height keeps the
command large on every axis; the loop still exits the moment the eef arrives,
and a deeper close cannot corrupt the release because the hang is measured, not
assumed.
Evidence: ep52 and ep58 both succeed. ep58 freed by 0.0838 and closed 0.0392
at z = table+0.034 -- a base grasp -- and the measured hang placed it correctly
anyway. ep54/62 still blocked at table+0.092, and there the two wasted presses
starved the later `lower`: ep62 carried the bottle to the rack on a 0.0239 grip
and then released it 0.05 m too high.
Verdict: the deep press is the right command; the wasted presses cost the place.

### v12 - stop pressing when the block is real. same 7 seeds. **5/7.** FROZEN.
Hypothesis: the two stalls are different things and can be told apart at
runtime. A press that gains nothing while the eef is already over the bottle
(off 0.0075-0.0092 on 54/62) is a real block and the next press will be just as
stuck; a press that gains nothing but leaves the eef far off in xy (off 0.024
on ep58) is a starved move, and the next press carries a large command and
frees it. So: give up on the first kind and spend the steps on the place.
Evidence: probe 5/7 (52, 56, 58, 60, 62 ok). ep62 converted exactly as
predicted -- same 0.0239 grip, but `lower` now had the budget and landed the
bottle. Full-15 selection
`results/sel_c2clean_goal_put_wine_on_rack_pos_k0_v12` **13/15**.
The 11 seeds that reach the grasp height on their own take no press at all and
run the v9 sequence unchanged (ep51 control: zero press lines).
Verdict: accepted, frozen.

## Mechanism gap (the two seeds that remain)

Both residual failures are the SAME unexplained mechanism, and it is
falsifiable: **the eef can be blocked above the commanded grasp height with the
fingers already centred on the target, and no command frees it.**
- ep54: blocked at z = table+0.0907 with the eef 0.0075 from the bottle centre;
  press gained 0.0024, a second press in v11 gained -0.0001. Jaws closed 0.0037.
- ep64 (the seed whose bottle starts lying on its side): blocked at
  table+0.0897 with the eef 0.0006 from the target; press gained 0.0003.
It is not a reach limit -- ep57 reached z = 0.9746 at almost the same xy
(0.0647, -0.0324) that ep54 is blocked at (0.0595, -0.0279). It is not the
step budget -- the block appears on the 3rd move of the episode and survives
re-commanding. What is missing is a way to SENSE what the fingers are resting
on: the only hold sensor available here is the closed width after the jaws
shut, which is read too late to re-plan the approach. A wrist capture at the
blocked pose would answer it; that costs the move budget the place needs.

## DECLARATION

- **Frozen version: v12.** `packs/c2clean_goal_put_wine_on_rack_pos_k0/program.py`
  md5 `97714f0afcb4846207192c39e53a8da9` == `program_v12.py` (same md5, verified
  on the cluster).
- **Selection receipt: 13/15** on the full debug set 51-65, one formal run,
  `results/sel_c2clean_goal_put_wine_on_rack_pos_k0_v12` (13 x
  `"benchmark_success": true`). Fails 54 and 64.
- **Receipt chain** (every formally probed version):
  | version | run dir | seeds | score |
  |---|---|---|---|
  | v1 | fs_..._v1, fs_..._v1b | 51,53,55,57 then 51-65 | perception only |
  | v2 | fs_..._v2 | 51,55 | survey only |
  | v3 | fs_..._v3 | 51,53,55,57 | 0/4 |
  | v4 | fs_..._v4 | 51,55 | budget probe |
  | v5 | fs_..._v5 | 51,53,55,57 | 0/4 |
  | v6 | fs_..._v6 | 51,53,55,57 | 0/4 |
  | v7 | fs_..._v7 | 51,55 | height sweep |
  | v8 | fs_..._v8 | 51,53,...,65 | 0/8 |
  | v9 | fs_..._v9 / sel_..._v9 | 8 odd / all 15 | 8/8 / **11/15** |
  | v10 | fs_..._v10 | 7 even | 3/7 |
  | v11 | fs_..._v11 | 7 even | 4/7 |
  | v12 | fs_..._v12 / sel_..._v12 | 7 even / all 15 | 5/7 / **13/15** |
  (all dirs prefixed `results/`, suffixed `_c2clean_goal_put_wine_on_rack_pos_k0_`)
- **PROVENANCE**: present in program.py as a top-level literal dict, 17 entries,
  every one sourced to a debug-seed measurement made in this cell. No pack (k0),
  no prior-knowledge constants, no `.done` read, no use of `fewshot_run.py`.

## RESUME 2026-09-13T11:04:57Z (coordinator note)
The previous session ended without a DECLARATION: its last message says it was waiting for a background notification that a headless session never receives. This is an outage, not a result. Resume under the unchanged c2clean rules from your own workspace and cluster artifacts only (packs/c2clean_goal_put_wine_on_rack_pos_k0/ program_v*.py, results/fs_* and results/sel_* dirs). Finish the selection if missing, freeze, and write the DECLARATION.
