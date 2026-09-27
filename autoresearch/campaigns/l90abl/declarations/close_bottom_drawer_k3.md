# l90abl / close_bottom_drawer_k3 — working notes

Intent: "close the bottom drawer of the cabinet". FAIR_PROTOCOL v1.1.1, runner
`tools/fair_run.py` only, debug seeds 51-65, eval seeds 1-50 blind.

## What the pack says

`packs/l90abl_close_bottom_drawer_k3/pack.json`, K=3, 8 keyframe images.

* Every demo starts at the reset pose (x≈-0.20, z≈1.16, wrist straight down)
  and ends 0.14-0.20 m further in **+y** (d0 `[0.012,0.174,0.933]`,
  d1 `[0.040,0.204,0.952]`, d2 `[-0.037,0.140,0.944]`).
* `gripper_cmd = -1.0` at every keyframe of every demo — the gripper never
  closes. So the task is a **push along +y**, not a grasp-and-pull.
* `ee_path6` rotations are robosuite **axis-angle**, not euler: their norms run
  to 3.6-4.4 rad (> π, which euler would not need but robosuite's
  `quat2axisangle` produces because it does not fold the angle into [0,π]), and
  read as axis-angle the three final tool-z axes agree physically
  (`(-0.40,0.61,-0.68)`, `(-0.29,0.93,-0.24)`, `(-0.66,0.58,-0.48)` — all
  forward-and-down, 45-75° of forward tilt), while read as euler xyz they point
  in three unrelated directions. Hence: the demos **tilt the wrist forward**
  before pushing.

## Scene, measured from my own debug-seed depth (v1 dump, seeds 51/53/55/57)

Table top z = 0.900. Cabinet on the +y side; its bottom drawer is pulled out
toward -y. Per-seed measurements:

| structure | measurement |
| --- | --- |
| drawer side walls | vertical planes at x ≈ -0.105 and +0.115, z 0.918-0.983 |
| drawer front panel | wide face at y ≈ 0.05-0.08, x span 0.2245 m, top edge z ≈ 0.985 |
| drawer handle | bar at x ∈ [-0.05,+0.05], z ≈ 0.95, protruding ~0.03 m in front of the panel |
| cabinet front face (= closed position) | y ≈ 0.184-0.199 |
| clutter | a metal bowl right in front of the drawer (y -0.10..0.00, rim z ≈ 0.955) and a wine bottle at x ≈ -0.15, both re-placed per seed |

The cabinet itself is nearly fixed across seeds (front face varies ~0.015 m);
the bowl and bottle move, which is why the descent lane is chosen per episode.

## Version chain

| ver | change | probe receipt |
| --- | --- | --- |
| v1 | perception dump only (RGB-D → my results dir), no task action | 0/4 (51,53,55,57) — by construction; gave the table height, the drawer geometry and the clutter |
| v2 | straight-down wrist, jaws closed, push +y at the handle height, fingertip offset estimated from cam_high | **1/4** (`fs_..._v2`) — 55 ok. 51/57 pushed nothing (the fingertip estimate was garbage: the gripper body occludes its own fingers from a top-down view, leaving the hand 0.17 m high). 53/55 did push, and both **stalled with the hand at y ≈ 0.115-0.118** while the drawer needs its front at ≈ 0.185 |
| v3 | tilt the tool 70° forward (`R = [[1,0,0],[0,-c,s],[0,-s,-c]]`), hand at panel_top+0.012 so it clears the drawer rim and the bowl, push in 0.03 m steps | **3/8** (`fs_..._v3`, 51,53,55,57,59,61,63,65). Two mechanisms: (A) seeds 53/55/63 staged the tilt at y0 ≤ -0.100, could not reach that tilted pose, drifted UP to z ≈ 1.178 and **froze** (prog = 0.0000 forever); (B) 51/57/59/61/65 all engaged and stalled at y = 0.196-0.218, drawer 0.00-0.05 m short |
| v4 | commanded the deep target straight from the staging pose so the action saturates | **0/8** (`fs_..._v4`) — regression, and it explains B's other half: with the whole error saturated the hand travels **diagonally**, is still at z ≈ 1.06 when it crosses the panel plane, and only reaches push height 0.035 m *past* the panel. It flew over the drawer front and only brushed the handle (front moved 0.11-0.14 m, then the hand sat ahead of the panel with no contact) |
| v5 | v3's incremental walk-in (which puts the hand down *in front of* the panel) **then** 3 saturated chunks at front+0.30; staging y clamped to ≥ -0.085 | **7/8** (`fs_..._v5`). The clamp fixed all three frozen seeds. The one failure, 61, stalled at y = 0.1962 where every success stalled at 0.2000-0.2182 |
| v6 | lane candidates tried **outside-first** (dx = -0.095, +0.095, -0.075, +0.075, -0.045, +0.045, 0); if the stroke ends short of the pre-push cabinet face, slide 0.045 m sideways while pressed and lean again | **8/8** (`fs_..._v6`); selection run below |

## Mechanism, as finally understood

1. **Direction and contact height.** Push +y with the hand at
   `panel_top + 0.012` (≈ 0.997): high enough to clear the bowl rim (0.955) and
   the drawer rim (0.985), while the 70°-tilted fingers hang into the panel's
   z band and do the pushing.
2. **The wrist must be tilted.** With the wrist straight down the hand jams at
   y ≈ 0.117 (v2, two independent seeds) — far short of the 0.185 the drawer
   front has to reach. Tilted, the same push runs to y ≈ 0.20-0.22.
3. **The tilted staging pose has a back limit.** Staging at y ≤ -0.100 is not
   reachable tilted: the arm rises to z ≈ 1.178 and deadlocks (v3, 3 seeds).
   Clamp the staging y to ≥ -0.085.
4. **Engagement must be incremental, the finish must be saturated.** The
   controller is `action = clip(err/0.05, -1, 1)`. A 0.03 m step commands 60 %
   effort — enough to move the drawer but not to seat it (v3: five seeds left
   0.00-0.05 m on the table). A saturated command from far away makes the hand
   descend diagonally and vault the panel (v4: 0/8). Walking in at 0.03 m and
   *then* holding a target 0.30 m past the front gives both contact geometry
   and full effort (v5: 7/8).
5. **How far the hand can push depends on the lane.** Furthest-+y at push
   height was 0.2000-0.2182 at x = -0.092..-0.075 and +0.114, but only 0.1962
   at x = -0.043. Outside lanes reach further, so try them first (v6: 8/8).

## Verification (no runtime success signal used)

Every episode re-perceives the drawer after the stroke and logs
`front_y → front_y'` against the cabinet face measured *before* touching
anything; the retry trigger is that comparison, never a success bit.
`api.done` is never read.

## DECLARATION

**Frozen version: v6.** `packs/l90abl_close_bottom_drawer_k3/program.py`
md5 `aac349dcedf3f744de6de844f5308b7b` == `program_v6.py` (verified on the
cluster).

**Selection receipt (formal, all 15 debug seeds 51-65):**
`results/sel_l90abl_close_bottom_drawer_k3_v6` — **15/15**
(`benchmark_success: true` on 51,52,53,54,55,56,57,58,59,60,61,62,63,64,65).

**Per-version receipt chain (probe runs, seeds 51,53,55,57[,59,61,63,65]):**

| ver | probe dir | result |
| --- | --- | --- |
| v1 | `results/fs_l90abl_close_bottom_drawer_k3_v1` | 0/4 (perception dump, no action) |
| v2 | `results/fs_l90abl_close_bottom_drawer_k3_v2` | 1/4 |
| v3 | `results/fs_l90abl_close_bottom_drawer_k3_v3` | 3/8 |
| v4 | `results/fs_l90abl_close_bottom_drawer_k3_v4` | 0/8 |
| v5 | `results/fs_l90abl_close_bottom_drawer_k3_v5` | 7/8 |
| v6 | `results/fs_l90abl_close_bottom_drawer_k3_v6` | 8/8 |

**PROVENANCE:** present, 12 entries, every one `allowed: True` with a source in
{this pack's fields, my own debug-seed measurements, generic controller
mechanics}. Checked against the eval gate offline: no forbidden tokens, no
`.done` attribute read anywhere in the program, `PROVENANCE` parses as a
top-level literal dict.

**Clean room:** writes were confined to
`packs/l90abl_close_bottom_drawer_k3/*` and `results/*l90abl_close_bottom_drawer_k3*`
(including the v1 RGB-D dump directory `results/l90abl_close_bottom_drawer_k3_dump_v1`).
No benchmark asset, no other campaign's pack/results, and no other program file
was read. Eval seeds 1-50 were never touched. Campaign `LAWS.md` was empty at
the start of this session.

STOP.
