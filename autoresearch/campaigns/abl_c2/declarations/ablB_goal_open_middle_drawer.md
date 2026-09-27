# abl_c2 / ablB_goal_open_middle_drawer — worker ledger (variant B, no-LAWS)

Intent: "open the middle drawer of the cabinet".
Runner: tools/fair_run.py on AbakaAI, GPU 6, split=debug (seeds 51-65 only).
Pack: packs/ablB_goal_open_middle_drawer (K=3 demos, keyframes + ee_path6 +
raw actions + action_scale). No law file was read, written or sought.

## What the pack says (v1 hypothesis)
All three demos: gripper_cmd = -1.0 (OPEN) at every keyframe — the drawer is
never pinched. The tool rolls from straight-down to a tilted pose whose
approach axis is (0,-sin57,-cos57) with the finger-separation axis along
world +x, descends to z ~ 1.036, reaches y ~ -0.142 (demos agree to +-3 mm),
then translates +y by ~0.17 m at constant x,z. Engage x differs per demo
(-0.005 / +0.041 / +0.015).

## Version chain (hypothesis -> evidence -> verdict)

**v1** (md5 ec6840a5…) — replay the demo mean pose path, plus a coarse
base-frame height map from my own cam_high depth.
Probe: results/fs_ablB_goal_open_middle_drawer_v1, seeds 51,53,…,65 → **0/8**.
Evidence: every move landed inside tolerance (residuals 0.008-0.011), so the
arm went exactly where the demos went; 214 sim steps; nothing moved. Reading
the logged cam_high extrinsics/intrinsics and re-projecting the commanded
poses onto the episode gif showed the eef stopping ~1 cm in FRONT of the
cabinet's drawer-front hardware. Verdict: the demo engage pose is free air in
these seeds — the cabinet does not sit where the demo constants put it.

**v2** (md5 81846696…) — measure the scene instead: table height, cabinet
footprint, y-extent per height band, plus a self-calibration of the fingertip
offset along the tool axis.
Probe: …_v2, seeds 51,53,55,57 → **0/4**.
Evidence: the y<-0.05 cutoff let table clutter (bowl/plate/bottle) into the
"cabinet" set, so the profile's protruding bands were those objects; the
program drove to x=-0.19, y=-0.05 and pulled empty air. The fingertip
self-calibration also over-read (0.285 m) and fell back. Verdict: measurement
window wrong, not the idea.

**v3** (md5 ad8158b7…) — cutoff moved out to y<-0.10 and x>-0.08 (excludes the
slatted rack), press PAST the measured front plane so contact stalls the move,
close the gripper, drag +y, re-measure, retry +-0.02 m in height.
Probe: …_v3, seeds 51,53,55,57 → **0/4**.
Evidence — this run produced the geometry the cell was missing (seed 51):
drawer-front plane y = -0.158; three rows standing 0.030 m proud of it, tips
at y ~ -0.128, centred at z ~ 0.951 / 1.016 / 1.085 (pitch ~0.065-0.07 m);
cabinet top z = 1.128; table z = 0.901. Contact stalls calibrated the
gripper's lead over the reported eef: pressing at a hardware-free height
stalled at y=-0.148 against the -0.158 plane (lead 0.010), pressing at the
middle row stalled at y=-0.118 against the -0.128 tips (lead 0.010). Every
close returned width 0.0018 m / effort 0.05 — shut on air. Verdict: at the
demo's 57 deg tilt the finger blades meet the hardware broadside; an
open-fingered drag past a 3 cm protrusion carries nothing.

**v4** (md5 e1f28658…) — near-horizontal approach (80 deg) so the finger slot
faces the protrusion; grasp it; verify with the gripper's width/effort
readback; retry +-0.015 m in height.
First launch aborted on a format-string bug in a log line (4 episodes, zero
usable steps, dir discarded); relaunched after a one-line fix as the same
version.
Probe: …_v4, seeds 51,53,55,57 → **0/4**.
Evidence: detection was clean and stable (3 rows at z 0.954/1.023/1.095,
x medians 0.042-0.081, tips y=-0.126). But the press slid +0.034 m in x under
contact (commanded x=0.053, achieved 0.087), putting the protrusion outside
the finger slot (half-width ~0.035 m); all three closes shut on air. The end
scan showed the front plane unchanged.

**v5** (md5 f8e17118…, FROZEN) — x estimated from the 10-90 percentile spread
of the proud points rather than a mixed-row median; commanded x corrected by
the error actually observed at the standoff pose; tilt reduced to 72 deg;
press only 0.02 m past the tip; gripper sensor decides capture, searching
+-0.03 m in x; if no capture is sensed, a closed-fingertip hook (set the shut
blade on the front 0.025 m below the row, rise 0.030 m into its underside,
drag +y).
Probe: …_v5, seeds 51,53,55,57 → **0/4** (721-733 sim steps/episode).
Evidence: the x correction worked (standoff errors 0.002-0.008 m), but all
three x offsets — achieved eef x = 0.010, 0.034, 0.089 — stalled at the SAME
depth, eef y ~ -0.119, i.e. against something at y ~ -0.129 spanning at least
8 cm of x. Combined with the proud-point spread (xlo=0.005, xhi=0.081) that
identifies the hardware as a ~7.6 cm wide horizontal BAR, not a compact knob:
a slot that separates its fingers along world x cannot swallow a bar that runs
along world x, which is why every close in v3/v4/v5 shut on air. The
closed-blade fallback was the only thing that ever loaded the drawer: the end
scan shows the middle row's y moving -0.1271 → -0.1168 and the front plane
-0.158 → -0.148, i.e. the middle drawer came out ~0.010 m — short of the
benchmark bit.

## DECLARATION
- Frozen program: `packs/ablB_goal_open_middle_drawer/program.py`,
  md5 `f8e171189efc9cbf742cacfda8c8e7ff` == `program_v5.py` (same md5).
- Selection (ONE formal run, all 15 debug seeds 51-65):
  `results/sel_ablB_goal_open_middle_drawer_v5` → **0/15**.
- Receipt chain (all on split=debug, GPU 6):

  | version | md5 | result dir | seeds | score |
  |---|---|---|---|---|
  | v1 | ec6840a58f57ac48f451eb73c4ef795d | fs_ablB_goal_open_middle_drawer_v1 | 51,53,…,65 | 0/8 |
  | v2 | 8184669624a068d4db8635a9e85bef91 | fs_ablB_goal_open_middle_drawer_v2 | 51,53,55,57 | 0/4 |
  | v3 | ad8158b7b8968f2bfcd0e88b056a3fa1 | fs_ablB_goal_open_middle_drawer_v3 | 51,53,55,57 | 0/4 |
  | v4 | e1f2865822d403e840d003583ea088a4 | fs_ablB_goal_open_middle_drawer_v4 | 51,53,55,57 | 0/4 (+4 aborted, discarded) |
  | v5 | f8e171189efc9cbf742cacfda8c8e7ff | fs_ablB_goal_open_middle_drawer_v5 | 51,53,55,57 | 0/4 |
  | v5 (selection) | f8e171189efc9cbf742cacfda8c8e7ff | sel_ablB_goal_open_middle_drawer_v5 | 51..65 | **0/15** |

- Debug episodes consumed in total: **43** (8+4+4+4 aborted+4+4 probes + 15
  selection). No eval seed (1-50) was ever touched.
- PROVENANCE: present in the frozen program.py as a top-level literal dict,
  15 entries, each with a pack field or an own-debug-observation source.
- Version budget: 5 of 5 used; argmax selected and frozen at v5 (the only
  version whose end-scan showed the middle drawer displaced). No v6 was
  written.
- Clean room: no .bddl/.xml/.hdf5/init-state file opened; nothing under
  campaigns/c1, c2 or c2fix, no packs/c1_*, packs/c2_*, results/*c1_*,
  results/*c2_*, no other abl_c2 cell's material; no law file read or written;
  api.done never read; every execution went through tools/fair_run.py.
