# l90abl / close_top_drawer_k0 — zero-demo, FAIR_PROTOCOL v1.1.1

Intent: "close the top drawer of the cabinet" (KITCHEN_SCENE5).
No demonstration pack; every constant re-derived from debug seeds 51-65.

## Scene, as measured (cam_high point cloud, seeds 51/52/53/55)

Base-frame facts, identical to ~1 cm on all four probed seeds (the cabinet is a
fixture; only its x wanders by about 0.01):

| feature | value |
|---|---|
| table top | z = 0.900 |
| cabinet front plane (dead flat over z 0.96..1.04) | y = 0.2163 |
| cabinet top slab | z = 1.127, y 0.216..0.376, x -0.13..0.14 |
| open top drawer: interior floor | z = 1.064 |
| open top drawer: front panel face | y = 0.0671, z 1.035..1.124 |
| open top drawer: handle bar | y = 0.0353, z 1.083..1.098 |
| closing travel | 0.2163 - 0.0671 = **0.149 m in +y** |

Camera frame: `t_base_cam` puts cam_high at (0.659, 0, 1.610) looking toward -x
and down; image right = base +y, image down = base +x.

Objects on the table (plate, bottle, bowl) all sit at y < 0.0 with z_max 1.048,
so the whole drawer corridor at y > 0.02 is clear.

## Version chain

### v1 — perception probe (2 eps, no motion)
Hypothesis: nothing known; dump intrinsics/extrinsics and a coarse height map.
Evidence: table plane 0.900; one tall structure at x ~0, y 0.20..0.38, z 1.13
(the cabinet) and a lower shelf at z 1.06 in front of it.
Verdict: useful but too coarse — replaced by a raw dump.

### v2 — base64 RGB-D dump (4 eps, no motion)
Hypothesis: analysing the cloud offline beats printing text maps.
Evidence: first attempt lost data — `api.log` truncates a message at ~2000
chars, so 4000-char base64 chunks were silently clipped. Re-ran with 1500-char
chunks; decoded cleanly. Produced the table above.
Verdict: the drawer is pulled out toward -y; closing is a straight +y push on
the front panel / handle. **Useful law: api.log truncates at ~2000 chars.**

### v3 — first push, with in-episode tip calibration (51/53/55) → 0/3
Plan: close the gripper, press it onto the cabinet top to learn the
fingertip-to-eef offset, then descend in front of the handle and push +y.
Evidence: the calibration press stalled with eef z = 1.1382 on a surface
measured at 1.1272, so **fingertip offset = 0.011** (independently confirmed by
the fingertips reading z ~1.16 in the cloud with the eef at 1.1733). The push
itself worked — 0.035 m steps converged with residual ~0.011 — but the eef only
reached y = 0.1197 before push4/push5 returned an *identical* eef with a growing
residual: the 1000-step episode budget was exhausted. `sim_steps: 1000` on all
three.
Verdict: mechanism sound, budget spent on calibration. Also learned the hand
casing spans 0.20 m along the gripper-width axis and only 0.03 m across it
(cloud blob at eef+0.03..eef+0.11), so with the default wrist the casing fouls
the cabinet top slab long before the fingertips reach the front plane.

### v4 — yaw the wrist 90°, drop the calibration (51/53/55/57) → **3/4**
Changes: hard-code TIP_OFFSET = 0.011; single approach move; wrist yawed 90°
about world z (`[[0,1,0],[1,0,0],[0,0,-1]]`) so the casing presents its 0.03 m
face to the cabinet; seven moves total.
Evidence: seeds 53/55/57 succeeded (episode terminated at ~100 sim steps). Seed
51 ran to the end and failed: the eef stopped at y = 0.1719 and the drawer front
was re-perceived at y = 0.1845 — **0.032 short of the 0.2163 front plane**.
Verdict: the cap `y_closed - 0.033` was simply too shy. The failure also
calibrated the contact geometry: drawer front = eef_y + 0.0126, i.e. the closed
gripper block pushes 13 mm ahead of the eef point.

### v5 — push to the front plane (FROZEN)
Change: `y_cap = y_closed - 0.008` (was -0.033), plus two repeat "seat" commands
at the cap to absorb the move tolerance.
Evidence: probe 51,53,...,65 → **8/8**. Formal full-15 selection → **15/15**.
Episodes terminate at 95-99 sim steps (LIBERO ends the episode the moment the
predicate fires, during push3). The ep51 GIF's last frame shows the drawer
flush with the cabinet body, so this is real closure, not a predicate artifact.

## Candidate laws for LAWS.md

1. **`api.log` truncates a message at ~2000 characters.** Chunk any base64 dump
   at <=1500 chars or you lose bytes silently (base64 padding error on decode).
2. **The hand casing is 0.20 m wide along the gripper-width axis and 0.03 m
   across it.** When a push must run the fingertips into a recess under an
   overhanging slab, yaw the wrist 90° about world z to present the narrow face;
   the eef then reaches ~0.17 m further before the casing fouls anything.
3. **A closed gripper pushes ~13 mm ahead of the eef point.** Size the final
   push command as `y_target = y_goal_plane - 0.013`, not to the perceived face.
4. **Spend the episode budget on the task, not on calibration.** An in-episode
   press-to-calibrate cost v3 the entire 1000-step horizon; the same offset was
   recoverable from a single earlier run's log and hard-coded for free.
5. **Fixture geometry can be read once and reused.** The cabinet front plane sat
   at y = 0.2163 +/- 0.003 on every debug seed; the seed perturbation moves the
   loose props, not the cabinet.

---

# DECLARATION

- **Frozen version:** `program.py` == `program_v5.py`,
  md5 `efba19f2d734d9167f4fb467b825f625` (verified on the cluster and locally).
- **Selection receipt (full 15 debug seeds 51-65):** **15/15**
  `results/sel_l90abl_close_top_drawer_k0_v5` on AbakaAI
  (`grep -c '"benchmark_success": true'` = 15).
- **Receipt chain:**
  - v1 `results/fs_l90abl_close_top_drawer_k0_v1` — perception only (2 eps).
  - v2 `results/fs_l90abl_close_top_drawer_k0_v2` — RGB-D dump only (4 eps).
  - v3 `results/fs_l90abl_close_top_drawer_k0_v3` — 0/3 (51,53,55), budget-starved.
  - v4 `results/fs_l90abl_close_top_drawer_k0_v4` — 3/4 (51,53,55,57).
  - v5 `results/fs_l90abl_close_top_drawer_k0_v5` — 8/8 probe (51,53,...,65).
  - v5 `results/sel_l90abl_close_top_drawer_k0_v5` — 15/15 formal selection.
- **PROVENANCE:** present in `program.py` as a top-level literal dict covering
  TIP_OFFSET, PUSH_TIP_Z, YAW90, Y_CAP, APPROACH_GAP, FRONT_BAND, CLOSED_BAND,
  FLOOR_BAND, ROI — every source is either a debug-seed (51-65) cam_high
  measurement, a debug-run stall reading, or generic wrist/camera mechanics.
  No demonstration pack exists for this cell and none was used; no benchmark
  asset file was opened; `api.done` is never read.
- **Archived versions:** `program_v1.py` .. `program_v5.py` in the pack dir.

STOP.
