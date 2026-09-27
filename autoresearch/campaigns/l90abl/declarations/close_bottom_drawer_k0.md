# l90abl / close_bottom_drawer_k0 — worker notes

Intent: "close the bottom drawer of the cabinet". Zero demos. Runner: tools/fair_run.py only.
Probe subset: seeds 51,53,55,57,59,61,63,65. Selection: full 15 (51-65).

## Scene, re-derived from debug-seed observations only

cam_high extrinsics put the camera at base (0.659, 0.0, 1.610) looking along (-0.778, 0, -0.628),
so in the rendered image **up = -x, right = +y**. Everything below comes from deprojected
cam_high depth on debug seeds 51-65.

- Table plane z = 0.902 (modal height of the deprojected cloud).
- Cabinet: a block whose top face is a large horizontal plane at z = 1.127 (= table + 0.225),
  x in [-0.13, 0.14], y in [0.21, 0.38]. Its front face (min y of the top plane) is y ~ 0.215.
- The bottom drawer is **already open at reset** and slides along **y**: it is an open-topped box
  with interior floor at z = 0.92 (table + 0.018) and side-wall tops at z = 0.983 (table + 0.081),
  x in [-0.11, 0.13]. Its front panel's top edge is at z ~ 1.00 and its y varies by seed
  (0.056 - 0.080 over the probe subset), i.e. the pull-out distance is seed-dependent.
- A vertical face whose normal is +/-y is seen exactly edge-on by cam_high, so the drawer front
  panel is invisible in depth except for its top edge. Perceive the drawer by its **side-wall
  top band** (table+0.06 .. table+0.11), not by its front face.
- Obstacles in front of the drawer: a bowl (rim z = 0.952, x in [-0.045, 0.085],
  y in [-0.105, 0.010]) and a bottle (x in [-0.16, -0.11], y ~ 0.02-0.08, top z = 1.058).
  Their placement varies by seed, so the push column must be chosen per episode.
- The robot's own arm at reset is a tall structure at x ~ -0.21, y ~ 0 and must be masked out of
  perception (it otherwise impersonates the cabinet: see v4).

## Version log

### v1 / v2 — perception dumps (no motion)
Hypothesis: nothing; establish the scene. Evidence: logs above; v1 wrote npys to a relative path
that landed outside the pack, v2 used an absolute path under the pack dir and recovered
rgb/depth. Verdict: scene model above.

### v3 — fingertip calibration + hardcoded push — 0/4 (fs_..._v3)
Hypothesis: measure the eef-to-fingertip offset by pressing a closed gripper into bare table,
then push.
Evidence: descent stalled at eef z = 0.9095 against a table plane of 0.901, twice
(cmd 0.92 -> 0.9098, cmd 0.84 -> 0.9095), so **api.eef() sits only ~0.008 above the closed
gripper's contact point**. But results.jsonl reports `sim_steps: 1000` and every move after the
fourth returned instantly with a frozen eef: **the episode budget is 1000 steps and
`move(seconds=s)` costs ~100*s steps, i.e. ~10 s of commanded motion for the whole episode.**
Verdict: calibration is unaffordable in-episode; the plan must fit in <= ~9 s of moves.

### v4 — perception + single push — 0/8 (fs_..._v4)
Hypothesis: perceive the drawer, thread between bowl and bottle, push +y.
Evidence: perception was contaminated by the robot's own arm — the "cabinet" band reported
x_min = -0.215 and y_front = 0.038 in **all eight** seeds (the arm, identical every reset), so
the push target y_end = 0.108 was far short of the true closed pose. All eight pushes converged
(res ~ 0.011) at y ~ 0.10, i.e. they stopped before doing the work. GIF of ep61 nevertheless
shows the drawer visibly much further in at the end.
Verdict: mask the arm; command far past the closed pose so the move stays unconverged.

### v5 — arm-masked perception + over-travel push — **6/8** (fs_..._v5)
Hypothesis: as above, y_end = drawer_front + 0.32 (capped 0.34).
Evidence: perception now clean and stable (cab top 1.127, cab front 0.211-0.229, drawer front
0.056-0.080 by seed). Every push **stalls at eef y ~ 0.110-0.129** (res ~ 0.22) — about 0.10 m
short of the closed pose — so the gripper delivers an impulse and **the drawer coasts the rest of
the way** on its prismatic joint; LIBERO ends the episode the moment the predicate fires
(successful GIFs are 14 frames vs 120 for failures).
Successes: 51, 55, 57, 59, 61, 65 — all chose push_x ~ -0.09.
Failures: 53, 63 — both chose push_x ~ +0.10, within 0.02 m of the drawer's right wall.
The clearance-only argmax always lands on an *edge* of the drawer, which is the worst contact.
Verdict: keep the push, keep the contact point off the drawer's edges, and add a second impulse.

### v6 — edge keep-out + centre bias + two impulses — 0/8 (fs_..._v6)
Hypothesis: the v5 failures came from contacting the drawer's edge, so bias the column to the
mid-line and deliver two impulses.
Evidence: the centre weight (0.30) swamped the clearance term — seeds 51 and 63 chose
push_x ~ 0.00 with **clearance 0.000**, drove straight into the bowl and stalled at y ~ 0.085
(vs 0.115 when the lane is clear). Seed 53 at push_x 0.090 (clr 0.023) still failed.
Verdict: clearance must be a hard constraint, not a soft penalty. The double impulse was neutral
(seed 53 ended at the same drawer pose as v5) and is not worth its ~2 s of budget.

### v7 — push above the bowl rim (z = table+0.072) — 0/8 (fs_..._v7)
Hypothesis: fly the fingertips over the bowl rim (table+0.050) so the column choice stops
mattering.
Evidence: obstacle detection collapsed (obs = 0; the threshold table+0.055 is above the bowl rim
and the bottle was inside the arm mask), and more importantly the push became *weaker*: the
stall point was unchanged (eef y 0.115-0.117, so the extra height buys no reach) but the drawer
ended at y_front = 0.145-0.147 instead of flush — 0.07 m short.
Verdict: **the push height is load-bearing.** table+0.045 stays.

### v8 — v5 height, corrected arm mask, left-preferred column — 3/8 (fs_..._v8)
Hypothesis: prefer the -x column (which bought reach in v5) subject to a hard clearance floor.
Evidence: the clearance profile came back **0.000 across the entire drawer** on seeds 53/63, so
the fallback again chose the right edge. Cause: the obstacle corridor extended 0.24 m in front of
the drawer and swallowed the wooden rack at y < -0.16, which spans the whole drawer width.
Verdict: measure obstacles only in the lane the gripper actually occupies.

### v9 — corridor limited to the approach lane — **8/8 probe, 15/15 selection**  [FROZEN]
Changes from v5: arm mask is `x > eef_x + 0.05` plus a 0.05 m cylinder (the 0.15 m cylinder of
v5-v8 was swallowing the bottle); obstacles are points above table+0.03 in the lane
y in [drawer_front-0.15, drawer_front-0.012]; the column is the **smallest x** (most -x) whose
clearance >= 0.032, falling back to the best left-half column and then to the global argmax.
Evidence: probe seeds 51..65 odd = 8/8 (fs_..._v9); full 15 debug seeds = **15/15**
(sel_l90abl_close_bottom_drawer_k0_v9). Every episode's POST re-perception put the drawer front
within 0.001 m of the cabinet front. Chosen columns were -0.093..-0.080 on every seed.

## Mechanism (what actually closes the drawer)

The arm cannot reach the closed pose: at the push height the eef stalls at y ~ 0.110-0.129 while
the drawer front must travel to y ~ 0.211-0.229. The gripper therefore delivers a ~0.05 m
**impulse** and the drawer coasts the remaining ~0.10 m on its prismatic joint. That is why
(a) the push must be commanded far past the closed pose so the move never converges and keeps
pressing, (b) the push height must land squarely on the front panel rather than skim its top
edge, and (c) the contact column matters: the -x column both buys ~0.013 m more +y reach and
transmits the impulse cleanly. LIBERO ends the episode the moment the predicate fires, so a
successful GIF is ~14 frames against 120 for a failure — a useful (post-hoc) diagnostic.

## DECLARATION

- Frozen version: **program_v9.py**, md5 `417c89138a7035cdb10a67bbf188b935`
  == `packs/l90abl_close_bottom_drawer_k0/program.py` (verified on AbakaAI).
- Selection receipt: **15/15** on the full 15 debug seeds 51-65,
  `results/sel_l90abl_close_bottom_drawer_k0_v9` (results.jsonl, `"benchmark_success": true` x15).
- Probe receipt chain (seeds 51,53,55,57,59,61,63,65 unless noted):
  v1/v2 perception dumps (no motion) — fs_..._v1 (2 eps), fs_..._v2 (4 eps);
  v3 0/4 (fs_..._v3); v4 0/8 (fs_..._v4); v5 6/8 (fs_..._v5); v6 0/8 (fs_..._v6);
  v7 0/8 (fs_..._v7); v8 3/8 (fs_..._v8); v9 8/8 (fs_..._v9).
- PROVENANCE present in program.py: 13 calibrated constants, every source a debug-seed
  measurement, a prior debug-run receipt, or generic controller/step-budget mechanics.
  No demonstration pack was issued or used; no `.done` attribute is read anywhere in the program.
