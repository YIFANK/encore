# c2k1clean — goal_open_top_drawer_put_bowl_task_k1

Intent: **"Open the top layer of the drawer and put the cream cheese inside"**
Success = the environment's own benchmark bit, visible only post-episode.

Packs (evidence, not solutions):
- `..._task_k1` — language *"open the top drawer and put the bowl inside"*:
  same TARGET (top drawer), different object. K=1, 170 steps.
- `..._task_mate` — language *"put the cream cheese in the bowl"*: same OBJECT
  (cream cheese), different target. K=1, 92 steps.

Scene read (all from my own debug-seed RGB-D, seeds 51-65):
table z = 0.9014; cabinet on the -y side with three handle bars; cream cheese
is a flat blue slab on the table at +y.

---

## v0 — perception probe (no motion)

Hypothesis: the scene can be measured from cam_high depth alone.
Evidence (8 probe seeds, `fs_..._v0`):
- cam_high K/T: camera at (0.659, 0, 1.610), image +u -> base +y, +v -> base
  +x/-z. Table plane z = 0.9014 on every seed.
- Cabinet front face is the dominant y-plane over z in (0.95, 1.105): face_y
  ranges -0.152 .. -0.172 across seeds (the fixture *does* translate, ~2 cm).
- Three handle bars protrude 0.034 past the face; the top bar's top surface is
  at z = 1.0985 on **every** seed (cabinet z is fixed, x/y perturb).
- Bar cross-section at mid-x (ep51): y -0.1405..-0.1264, z 1.0928..1.0985 ->
  a cylinder of radius ~0.007, centre y = tip - 0.007, centre z = top - 0.007.
- Cabinet top surface: z = 1.1274, overhanging forward to y = face + 0.003.
- cam_arm_wrist depth: the robot's own geometry bottoms out at base z 1.164
  while api.eef() reads 1.1733 -> **fingertips sit 0.009 beyond the eef**.
- Cream cheese: exactly one cluster with B/(R+G+B) > 0.40 on the table in all
  8 seeds; footprint 0.077 x 0.041, top z 0.9203 (a 19 mm-tall slab).

Verdict: perception is reliable; constants recorded in PROVENANCE.

## v1 — top-down handle probe -> REFUTED

Hypothesis: put the open jaw's inner finger into the slot between the bar and
the drawer front, close, pull +y (the k1 pack pulls the handle from y -0.093
to +0.065 with the wrist straight down).
Evidence (`fs_..._v1`, seeds 51/53/57/59):
- `PROBE A_gap`: descent commanded to eef z 1.0620 stalled at **1.1366**, i.e.
  the fingertip at 1.1276 — exactly the measured cabinet top, 1.1274.
- `PROBE C_free` and `B_barmid` stalled with the tip at 1.0967 ~ the bar top.
- The subsequent close read width 0.001 (nothing held) and face_y was
  unchanged after the pull: the drawer never moved.

Verdict: **the slot is roofed.** Face at -0.160, bar back at -0.1467, but the
cabinet top overhangs to -0.157, so only a 10 mm strip of the 17 mm slot is
open from above — narrower than the finger. A top-down handle grasp is
mechanically impossible here, whatever the k1 pack's wrist looked like.

## v2 — side-on bar grasp -> drawer opens 4/4

Hypothesis: turn the wrist so the approach axis is -y and the jaws separate
along z; they then straddle the bar top/bottom in free air, clear of the roof.
Evidence (`fs_..._v2`, seeds 51/53/57/59):
- `CLOSED grip width 0.01736, effort 3.0` on all four seeds — the closed gap
  equals the measured bar diameter, which is the receipt that the bar (and not
  air, and not the drawer face) is in the jaws.
- The pull tracked cleanly: eef y -0.126 -> +0.038, residual 0.011 throughout,
  width and effort unchanged. Drawer travel 0.164 m, and it bottoms out at the
  same absolute stop on every seed.
- post_open depth: the drawer cavity floor is z 1.0565, wall rim 1.1234,
  cavity spans y -0.145..-0.010 and x -0.07..0.14, front panel at y 0.004.
- Cream cheese pick FAILED on all four: descent commanded to eef z 0.913
  landed at 0.9239 and the close read width 0.0014 (empty). Same +0.011 z
  offset at AT_DROP (1.1555 for a 1.145 command) with no contact anywhere
  near, so this is the controller's standing offset, not contact.

Verdict: opening solved; the grasp needs the offset cancelled.

## v3 — bias-cancelled grasp -> 6/8

Hypothesis: the +0.011 offset is the mover's own POS_TOL (0.012), so a bounded
loop that re-commands target + residual will land on the object.
Evidence (`fs_..._v3`, 8 probe seeds): 6/8. The grasp now reads width 0.0422 =
the measured 0.038 box width, effort 3.0. Losses: ep55 grasped well but the
lift starved at 0.9538 for a 1.05 command; ep63's descent stalled 0.079 high
and the retry re-perceived the robot's own blue body as the cream cheese.
Verdict: mechanism right, transport fragile.

## v4 — re-home between phases -> 7/8

Hypothesis: the jams are arm-configuration, not perception; returning to the
start pose between the drawer and the table re-poses the elbow (both packs
also start their demos from there). Plus a footprint sanity check so the
robot's own blue body can never be mistaken for the cream cheese.
Evidence: 7/8. ep55 fixed. ep63: the descent swung the eef 0.109 m in +x.

## v5 — staged descent + aim guard -> 7/8 (same seed)

## diag (joint logging, seeds 57/63)
At the hover the two seeds' joint vectors agree to 0.007 rad. Under ONE
saturated descent command, 63's joint 4 ran from -1.87 to its -0.0695 limit
and the arm wedged; 57's stayed at -1.83. A wedged arm could not be commanded
home afterwards (residual 0.28) but did free on a straight-up retreat.

## v6 — unsaturated 0.035 m descent rungs -> 7/8 (same seed)
The rungs did not help: the very first rung already flipped 63.

## diag2 (seeds 57/63, pick with the drawer still SHUT)
Both seeds pick cleanly (width 0.0422, effort 3.0). So the obstacle is the
open drawer, not the table scene or the target.

## v7 — cross the drawer above its rim (z 1.22) -> 6/8
The x-flip is gone: 63 now descends straight and simply STOPS at z 0.9884.
New loss on 55: a good grasp thrown away because the held-check demanded
z > 1.04 and the saturated 0.18 m lift starved at 0.956.

## diag3 (seeds 57/63, wrist + overhead dumps at the stall)
Depth under the jaws at the stall: bare table, box top 0.9203, nothing else
within x -0.10..0.06, y 0.06..0.18 up to z 1.10. No joint is at a limit. So
the stall is contact somewhere up the arm, not under the hand.

## v8 — laddered lift + approach dither -> 7/8
55 fixed by the laddered lift. 63 still stalls at 0.9884 and the dithering
burned the step budget.

## v9 — pull the drawer to an ABSOLUTE stop -> 8/8 on the probe subset

Hypothesis (the one that was actually load-bearing): PULL_DIST was a fixed
0.175 m *distance*, but the cabinet's own y varies ~0.02 m across seeds, so
the drawer ended up in a different place on every seed. Seed 63's drawer front
finished at y +0.013 and seed 57's at -0.003; the forearm grazes that front
wall on the way out to the cream cheese, and 16 mm decides it. Pulling to a
fixed y instead makes the opened drawer identical on every seed.
Evidence (`fs_..._v9`, seeds 51,53,55,57,59,61,63,65): **8/8**, every grasp
first-try with no dither, closed gap 0.0422 and effort 3.0 on all eight,
drawer travel 0.132-0.148 m (the k1 pack's own drag is 0.158 m).

---

# DECLARATION

**Frozen version: v9.** `packs/c2k1clean_goal_open_top_drawer_put_bowl_task_k1/program.py`
md5 `92b5d388ba848c19a7c3d86828925582` == `program_v9.py` (verified on AbakaAI).

**Selection receipt (formal, full 15 debug seeds 51-65):**
`results/sel_c2k1clean_goal_open_top_drawer_put_bowl_task_k1_v9` — **15/15**
`"benchmark_success": true`, note `v9 held=True` on every seed.

**PROVENANCE:** present as a top-level literal dict, 20 calibrated constants,
every one sourced to a named pack field or to my own debug-seed measurement.
`scan_program(..., "eval")` passes and the program contains no `.done` read.

**Receipt chain**

| ver | change | probe run | result |
|-----|--------|-----------|--------|
| v0 | perception only, RGB-D dumped through the log | `fs_..._v0` (8) | scene measured: table 0.9014, three handle bars, cabinet top 1.1274, fingertips 0.009 beyond the eef, cream cheese = the one blue table cluster |
| v1 | top-down handle grasp | `fs_..._v1` (4) | 0/4 — descent stalls with the tip at 1.1276 on the cabinet top; the handle slot is roofed |
| v2 | side-on bar grasp, jaws separating along z | `fs_..._v2` (4) | drawer opens 4/4 (closed gap 0.0174 = the measured bar diameter, effort 3.0); pick misses |
| v3 | bounded bias-cancelling moves | `fs_..._v3` (8) | 6/8 |
| v4 | re-home between phases, footprint check on the blue mask | `fs_..._v4` (8) | 7/8 |
| v5 | staged descent + lateral-aim guard | `fs_..._v5` (8) | 7/8 |
| v6 | unsaturated 0.035 m descent rungs | `fs_..._v6` (8) | 7/8 |
| v7 | cross the open drawer above its rim (z 1.22) | `fs_..._v7` (8) | 6/8 |
| v8 | laddered lift + approach dither | `fs_..._v8` (8) | 7/8 |
| **v9** | **pull the drawer to an absolute y stop** | `fs_..._v9` (8) | **8/8** |
| v9 | — | `sel_..._v9` (**15**) | **15/15** |

Diagnostics (not scored versions): `diag` joint logging, `diag2` pick with the
drawer shut, `diag3` wrist/overhead dumps at the stall.

**What the cell turned on.** Not perception — the cabinet, the three bars and
the cream cheese read cleanly from cam_high depth on every seed from v0
onward. It turned on two facts about the *fixture*, both of which the packs
state only implicitly:

1. The handle cannot be taken from above. The cabinet top overhangs to within
   3 mm of the drawer face, leaving 10 mm of the 17 mm bar slot open — thinner
   than a finger. Turning the wrist so the jaws separate along z and straddle
   the bar top-to-bottom is the only grasp that fits, and the closed gap
   (0.0165-0.0174, the bar's measured diameter) is the receipt that it worked.
2. The drawer must be pulled to an absolute stop, not by a fixed distance. The
   cabinet translates ~0.02 m across seeds, so a fixed drag leaves the opened
   drawer in a different place each time, and the arm's forearm grazes its
   front wall on the way out to the cream cheese. That 16 mm was the whole
   difference between 7/8 and 8/8, and it is invisible in any single episode.

The k1 pack contributed the mechanism (handle first, then place inside) and
the mate pack the grasp height for the cream cheese (eef 0.9104). Neither
pack's xy is usable: the packs' scenes are not these scenes.
