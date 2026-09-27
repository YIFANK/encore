# abl_c2 / ablA_goal_open_middle_drawer — worker ledger

Variant A ("naive-demos"): no mined pack. Task-specific input was
`packs/ablA_goal_open_middle_drawer/raw_demos.json` (K=3 dense
proprioception + dense raw actions, no segmentation, no keyframes, no
ee_path, no action_scale) plus 44 uniform-stride 128x128 RGB frames.
LAWS.md was empty (by design) and stays empty.

## 2026-08-20 — structure recovery from the raw dump (pre-v1)

The dump gave three demos of length 138/138/151 with obs keys
`ee_ori, ee_pos, ee_states, gripper_states, joint_states` and a (T,7)
action array. Everything below I had to derive myself.

1. **This is a hook, not a grasp.** `actions[:,6] == -1.0` at *every*
   timestep of *all three* demos, and `gripper_states[:,0]` never leaves
   [0.029, 0.041]. There is no grip event, so there is no gripper-based
   segmentation cue at all.
2. **Phase boundary.** `ee_pos[:,1]` (world y) descends monotonically then
   ascends monotonically. `argmin(y)` is therefore the contact keyframe:
   t = 93/138, 104/138, 116/151.
3. **Two phases.** 0..hook = reorient + reach; hook..end = a straight +y
   drag with z flat (std < 6 mm within each demo).
4. **Hook pose.** eef at the hook keyframe:
   x = 0.0006 / 0.0419 / 0.0260 (mean 0.023),
   y = -0.1455 / -0.1441 / -0.1502 (mean -0.147),
   z = 1.0320 / 1.0437 / 1.0373 (mean 1.0377).
5. **Orientation.** `ee_ori` is an axis-angle rotation vector. Converted to
   3x3 and averaged over the three hook keyframes (SVD-orthonormalised):
   tool z = [0.037, -0.819, -0.572] — a ~55 deg forward tilt off
   straight-down, pointing into -y and down; tool y (finger-opening axis)
   = +x.
6. **Approach waypoints.** The three demos resampled onto a common phase
   coordinate (0 = start, 1 = hook, 2 = end) and averaged.
7. **Pull travel.** Final `ee_pos[:,1]` = 0.070 / 0.061 / 0.016, i.e. a
   drag of 0.166 .. 0.216 m in +y.
8. **The scene is static across seeds.** Pixel diff of the cabinet crop of
   `frames/demo{0,1,2}_t0000.png` leaves only a 1-pixel outline residual
   (<= ~1 cm at the agentview scale) while the wooden rack in the same crop
   moves many pixels. So the 41 mm hook-x spread between demos is teleop
   noise (a measured x-tolerance), not scene randomisation, and open-loop
   base-frame waypoints are admissible.

## v1 — hypothesis: replay the demo eef poses open-loop

Waypoints = the phase-aligned demo mean; orientation = R_HOOK; fingers
open; pull to y = +0.120 at z = 1.031. Added a coarse cam_high point-cloud
log as failure insurance.

**Evidence** — `results/fs_ablA_goal_open_middle_drawer_v1`, seeds
51,53,55,57: **0/4**. Every single move converged, residual ~0.010,
*including the whole pull* (ep51 ended at eef y = +0.1107, res 0.0096).
The gripper swept the full pull path carrying nothing.

**Verdict: REFUTED, but informatively.** Position was reproduced exactly
and the task still failed, so the missing ingredient is not geometry.
Re-reading the demo *action* stream over the pull phase: `actions[:,2]` is
pinned near -0.94 (max down) and `actions[:,1]` near +0.9 through the whole
drag, while measured z stays flat — the operator was dragging under a
standing downward load. A position move that converges applies no load.

## v2 — hypothesis: the load, not the pose, is the task

Same waypoints, but the press/pull targets are commanded 30 mm *below* and
20 mm *behind* the contact surface so the OSC keeps pushing into the
handle. Also instrumented: dense cam_high cloud, a straight-down contact
probe for the eef->fingertip offset, `api.tool_rotation()` after every
commanded rotation, plus a second, mechanically different attempt
(straight-down finger drop) as a hedge.

**Evidence** — `results/fs_ablA_goal_open_middle_drawer_v2`, seeds 51,53:
**2/2**. Diagnostics from ep51:
- commanded R_HOOK tool z = [0.037,-0.819,-0.572];
  achieved `api.tool_rotation()` z = [0.046,-0.815,-0.577] → the rotation
  convention is correct and was never the problem.
- move residuals jump from 0.010 (v1, unloaded) to 0.0446 at `A.press` and
  0.036-0.076 through the pull → the gripper is now loaded.
- the eef never got past y = -0.002 during the pull (it was dragging), vs
  +0.111 unloaded in v1.
- measured cabinet geometry: top plate z = 1.125 (1844 cloud points in that
  band), x-span [-0.094, +0.159], frame front face y = -0.158.
- contact probe: `api.eef()` sits 0.053 m proximal of the fingertips. With
  the 55 deg tilt this puts the fingertips at ~(0.025, -0.188, 1.008),
  i.e. inside the cabinet's front recess and below the frame lip —
  consistent with a hook behind the middle drawer's handle.
- the hedge branch never ran on either seed.

**Verdict: CONFIRMED.** The task is a load-controlled drag, not a
pose-controlled one.

## v3 — freeze: v2's winning attempt, diagnostics stripped

Removed the scan, the contact probe and the straight-down hedge; kept the
tilted hook + press + loaded pull; lengthened the pull to y = +0.150
(0.298 m of commanded travel, above the largest demo travel of 0.216 m);
added one *mechanically gated* re-hook — if the pull tracks its target all
the way out to eef y >= +0.085 then nothing was under load and the drawer
never moved, so a second hook at the same depth cannot shut anything. The
gate threshold sits between the measured unloaded endpoint (+0.111, v1)
and the measured loaded endpoint (~0.00, v2).

**Evidence** — `results/sel_ablA_goal_open_middle_drawer_v3`, formal run on
all 15 debug seeds 51..65: **15/15**. sim_steps 340-406 (horizon 1000).
The slip gate fired on 0 of 15 episodes.

**Verdict: SELECTED.**

## Receipt chain

| ver | run dir | seeds | result |
|-----|---------|-------|--------|
| v1 | `results/fs_ablA_goal_open_middle_drawer_v1` | 51,53,55,57 | 0/4 |
| v2 | `results/fs_ablA_goal_open_middle_drawer_v2` | 51,53 | 2/2 |
| v3 | `results/sel_ablA_goal_open_middle_drawer_v3` | 51..65 (all 15) | **15/15** |

Total debug episodes consumed across all probes + selection: **21**
(4 + 2 + 15). Versions used: 3 of the 5 allowed.

## Candidate law (for the coordinator to bank or reject)

**"Pose replay is insufficient for articulated-fixture manipulation; the
demo's standing contact load must be replayed too."**
Falsifiable form: for a fixture task whose demo action stream shows a
control channel pinned near saturation while the corresponding measured
state stays flat, an open-loop replay of the demo *poses* alone scores ~0,
and re-issuing the same waypoints displaced through the contact surface by
~0.03 m (so the position controller keeps loading it) recovers the task
with no other change.
Receipt: identical waypoints, identical orientation, identical gripper
state — v1 0/4 (seeds 51,53,55,57) vs v2 2/2 (seeds 51,53) vs v3 15/15
(seeds 51-65). The only edit between v1 and v2's winning branch was the
press overshoot.

Secondary, weaker: **a demo whose gripper action channel never changes is a
hook/push task, and the finger-opening axis is then irrelevant** — the
whole episode here runs with `api.grip(0.08)` and never closes.

# DECLARATION

- **Frozen program**: `packs/ablA_goal_open_middle_drawer/program.py`,
  md5 `116ea9c98384c39a3c55e619ae6aaf3c`, identical to
  `packs/ablA_goal_open_middle_drawer/program_v3.py` (same md5, verified on
  the cluster).
- **Selection receipt**: **15/15** on the full debug split (seeds 51-65),
  run dir `results/sel_ablA_goal_open_middle_drawer_v3`.
- **Receipt chain**: v1 0/4 → v2 2/2 → v3 15/15 (table above).
- **PROVENANCE**: present as a top-level literal dict in program.py, with
  11 entries covering R_HOOK, HOOK_X/Y/Z, APPROACH_WAYPOINTS, PULL_Z,
  PRESS_DZ, PRESS_DY, PULL_Y_END, GRIPPER_OPEN and SLIP_Y_GATE. Every
  source is either a named field/derivation of this cell's own
  `raw_demos.json` / `frames/*.png`, or a measurement from my own debug-seed
  observations (logged move residuals, `api.tool_rotation()`, cam_high
  depth cloud, contact probe).
- **Clean room**: programs were executed only via `tools/fair_run.py`
  (`--split debug`, episode lists inside 51-65). Seeds 1-50 never touched.
  No .bddl/.xml/.hdf5/init-state file opened; the bddl path was only ever
  an opaque `--bddl` argument. Nothing read under campaigns/c1, c2 or
  c2fix, no packs/c1_*, packs/c2_*, results/*c1_*, results/*c2_*, no other
  abl_c2 cell's pack/results/workspace, no tools/probe_*.py. `api.done` is
  never referenced. All LIBERO-specific priors that surfaced in the session
  context were quarantined; every constant above is re-derived from this
  cell's own dump or its own debug observations.

STOP.
