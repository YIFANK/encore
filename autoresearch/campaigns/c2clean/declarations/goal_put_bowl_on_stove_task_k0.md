# c2clean / goal_put_bowl_on_stove_task_k0

Intent: **"Put the plate on the stove"** (re-authored `_task` cell; the bddl is
the stock bowl-on-stove file but the graded object is the plate).
No demonstration pack. Everything below was derived from debug seeds 51-65.

## The scene, as measured (v1, cam_high RGB-D)

Table plane z = 0.901. Deprojected clusters, stable across seeds 51/53/57/61:

| cluster | x range | y range | ztop | what |
|---|---|---|---|---|
| ~1300 px, rgb (0.61,0.59,0.58) | [-0.016, 0.119] | [-0.074, 0.061] | 0.920 | **the plate** (Ø 0.135, scalloped brim) |
| ~600 px, rgb (0.41,0.42,0.40) | [-0.137,-0.027] | [-0.058, 0.049] | 0.952 | bowl |
| ~2100 px, rgb (0.25,0.25,0.25) | [-0.452,-0.162] | [ 0.116, 0.306] | 0.960 | **the stove**; slab top 0.929, the 0.96 is the knob |
| ~1080 px | [-0.209,-0.027] | [-0.071, 0.049] | 1.059 | wine bottle |
| ~270 px, bluish | [-0.079,-0.001] | [ 0.111, 0.152] | 0.920 | small box |
| ~12800 px | wide | y < -0.12 | 1.244 | cabinet |

Plate radial profile: interior floor 0.908, rim rising to 0.920 at r = 0.0676.
So the dish is 19mm tall and its brim overhangs a much smaller base.

Identification rule used by the program (no hard-coded positions): among
clusters above the table, the plate is the one with `ztop < 0.930` (bowl 0.952,
stove 0.960 are above it) and the largest footprint in 0.08-0.22 m; the stove is
the largest remaining cluster with `0.93 < ztop < 1.00` and footprint > 0.15 x
0.10.

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | perception probe, stream RGB-D out through api.log | scene table above | scene mapped |
| v2 | chord-grasp the plate, probe fingertip offset at (0.22,-0.22) | probe target out of reach; the whole 1000-step horizon burnt on one move | dead; taught me the step budget |
| v3 | same, probe on reachable bare table | open gripper stalls at eef z 0.9377 over table (seeds 51,53); close bit nothing (gap 0.0018) | grasp failed |
| v4 | land fingertips at table height, slide in, close; both jaw-axis hypotheses | both closed on air, plate never moved | grasp failed |
| v5 | go under the brim (off 0.028 and 0.000) | both closed on air; plate not nudged at all | grasp failed |
| v6 | touch-calibrate against four surfaces of known height | ambiguous: "contact" indistinguishable from the 12mm tolerance | inconclusive |
| v7 | **one clean question**: press bare table open, then shut | open stalls at 0.9377 twice, shut at 0.9085 twice | **the jaws rise ~29mm as they close** |
| v8 | photograph the gripper beside the plate, open and shut | at tool offset R+0.001 the close bit: gap 0.0187, effort 3.0; the lift squeezed it out | first real bite |
| v9 | sweep the approach offset | every 7mm step was a no-op | **a move shorter than the 12mm POS_TOL never steps the arm** |
| v10 | drive to the rim with overshooting (bias-cancelled) moves | gap 0.063/0.068, effort 3.0, held through four raises, plate gone from the table (seeds 51,53) | **grasp solved** |
| v11 | + carry to the slab and release | 4/4 on seeds 51,53,55,57 | works, but the release height came from a bad measurement |
| v12 | release height from the grasp geometry; 2 cycles | 6/8; seeds 61,65 bit 29mm / nothing | depth of the plunge is the discriminator |
| v13 | deterministic deep plunge (45mm below the soft floor) | 7/8; plunge now lands at 0.9092 every seed | seed 59 missed, and the retry never ran |
| v14 | retract before re-perceiving so the retry has a target | **8/8** probe, **14/15** selection (`sel_..._v14`); seed 52 missed 3x from the same side | near-side approach is not universal |
| v15 | retry from the +y side | seed 52 fixed, but the +y bite is only 0.025 and bled out mid-carry on 59, and my verdict called that a success | verdict bug + ordering bug |
| v16 | +y demoted to third choice; release requires the jaws still loaded; carry shortened to 2 lifts + 1 diagonal | 6/6 probe (51,52,55,59,61,65); **15/15** selection | **frozen** |

## Mechanism, in one paragraph

The plate is 19mm tall with an overhanging scalloped brim, and the gripper
cannot be positioned around it the obvious way: with the jaws **open** the
fingers hang 36.8mm below `api.eef()` and meet the table softly at eef z 0.9377,
while **closed** they hang only 7.5mm below it — so the act of closing lifts the
fingers 29mm, out of the dish entirely. Every v3-v5 grasp closed on air for that
reason. The grasp that works ignores the soft floor: command the open jaws
45mm *below* it at the plate centre offset by (R + 1mm) on the near side, which
saturates the controller and ploughs the fingers down to eef z 0.9092, low
enough that the pads are alongside the dish when they close. That bites 60-68mm
of plate at effort 3.0. The dish's pose in the jaws is then read straight off
the close — at that instant it is still resting on the table, so its centre is
(cx - eef_x, cy - eef_y) from the tool and its base is (eef_z - 0.901) below it —
and the release height over the slab follows from those two numbers. Because a
wedge pinch bleeds as the arm moves, the carry is kept to two lifts and one
diagonal, and the jaws are checked for load at the moment of release.

Two further constants of the harness, both of which cost versions to learn:
a commanded move shorter than the 12mm position tolerance returns without
stepping the simulator at all (v9), and the episode horizon is ~1000 controller
steps, of which one pick-and-place costs ~305 (v16).

## DECLARATION

* **Frozen version: v16.** `packs/c2clean_goal_put_bowl_on_stove_task_k0/program.py`
  md5 `3d65e20b400d2d0c363c38cc6fadfc41` == `program_v16.py` md5
  `3d65e20b400d2d0c363c38cc6fadfc41`.
* **Selection receipt: 15/15** on the full debug band (seeds 51-65, one formal
  run), dir `results/sel_c2clean_goal_put_bowl_on_stove_task_k0_v16`.
  Every one of the 15 episodes reports `"benchmark_success": true`.
  Prior formal selection run: v14, 14/15,
  `results/sel_c2clean_goal_put_bowl_on_stove_task_k0_v14`.
* Per-version receipt chain: the table above; every formally-probed version is
  archived as `program_vN.py` in the pack directory.
* `PROVENANCE` is present in the frozen program and covers every calibrated
  constant, each sourced to a named debug-seed measurement.
