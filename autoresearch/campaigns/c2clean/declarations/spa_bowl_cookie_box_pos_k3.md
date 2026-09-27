# c2clean / spa_bowl_cookie_box_pos_k3 — worker notes

Intent: "pick up the black bowl next to the cookie box and place it on the plate"
Runner: tools/fair_run.py only. Debug seeds 51-65. FAIR_PROTOCOL v1.1.1.

## Pack reading (K=3, demos 0/1/2)

| quantity | demo0 | demo1 | demo2 | used as |
|---|---|---|---|---|
| close (gripper_cmd -1 -> +1) eef z | 0.9263 | 0.9228 | 0.9309 | GRASP_DEPTH = z − table = 0.0255 |
| release keyframe finger gap \|q0−q1\| | 0.0086 | 0.0104 | 0.0076 | WALL_GAP = 0.0086 (pinched rim wall) |
| release eef z | 0.9332 | 0.9300 | 0.9372 | PLACE_LIFT = 0.0255 above the landing surface |
| carry apex z | 1.062 | 1.077 | 1.064 | CARRY_H = table + 0.17 |

Demo keyframe images show the grasp is a **rim straddle** on the larger of the
table bowls, the one adjacent to the red cookie box, released over the plate.

**The demo xy is a decoy.** In the demo scenes the plate sits at the FRONT-right
site (image u≈395,v≈365); on every debug seed 51-65 the plate is at the
BACK-right site and a bowl occupies the front-right one. Nothing about the
demos' absolute x,y transfers — only the heights, the pinch gap and the
mechanism.

## probe0 — pure perception dump (all 15 debug seeds), 0 sim steps

RGB-D streamed out through api.log (zlib+base64) and parsed offline.
cam_high is fixed across seeds; table z = 0.9012 on every seed.

Two measurements classify every prop, with no colour needed:
* **top** height above the table
* **interior** = mean height inside a disc of half the fitted rim radius

| prop | top | interior | rim r | verdict |
|---|---|---|---|---|
| big bowl | 0.0507-0.0508 | 0.0074 | 0.0534-0.0538 | hollow, tall → BOWL |
| small bowl | 0.0424 | 0.0072 | 0.0419-0.0423 | hollow, tall → BOWL |
| plate | 0.0191 | 0.0072 | 0.0645-0.0649 | hollow, flat, wide → PLATE |
| cookie box | 0.0192 | 0.0191 | 0.0304 | **solid** (interior == top) → BOX |

Layout is stable over all 15 seeds (±2 cm jitter): plate (−0.20,+0.20),
big bowl (+0.13,−0.075), small bowl (+0.06,+0.20), cookie box (+0.068,+0.031).
A third bowl sits on a pedestal behind, fused with the arm blob (top 0.47), and
is rejected by the top>0.15 filter.

Target selector = bowl nearest the cookie box. Margin over the runner-up is
0.109-0.144 m vs 0.146-0.195 m; **worst seed is 57 at 0.144 vs 0.146 — a 2 mm
margin**. On all 15 seeds the nearest bowl is also the larger one, which is the
one the demos grasped (demo grasp depth 0.0255 below a 0.0508 rim top). Tie
rule added in v1: if the two distances are within 0.020, take the wider bowl.

## v1 — perception → rim-straddle grasp → carry → place
receipt: `results/fs_c2clean_spa_bowl_cookie_box_pos_k3_v1` **4/8** (51,53,55,57,59,61,63,65)
ok: 53,55,59,63   fail: 51,57,61,65

Perception was correct on every seed (right bowl, right plate, right side).
The grasp worked on every seed (effort 3.0, gap 0.008-0.015 = the rim wall).

**Single failure mode:** the carry traverse from the bowl to the plate freezes.
The arm reaches the commanded y and z but x sticks at +0.014…+0.019 and never
crosses to the plate at x≈−0.20. All four failures burn the full 1000-step
horizon there; the four successes complete the same move and finish in ~185
steps. The horizon is 1000 steps (from results.jsonl sim_steps).

Second defect exposed: `precise_move` cancels the standing error by adding it
to the command, unbounded. Against a blocked axis the command runs away
(residual 0.84 m), which turns a stall into a thrash. Must be bounded.

Next: probe1 walks the same traverse in 8 short legs while logging
robot0_joint_pos, to see which joint saturates.

## probe1 — legs + joint logging (seeds 51,53,57,61,65)

Its crude stand-in perception fitted one circle over BOTH bowls, so it grasped
air at (0.05,+0.18) and did **not** reproduce the failing traverse — a negative
control I mis-set. Still useful: from (0.05,+0.18,1.06) the arm walked in legs
all the way to x=−0.32 with every leg inside 12 mm, so x=−0.20 is not out of
reach, and the pedestal that carries the third bowl stands only ~0.08-0.12 m.

## v2 — bounded precise_move + carry walked in 0.055 m legs
receipt: `results/fs_c2clean_spa_bowl_cookie_box_pos_k3_v2` **2/8** (ok 55,65)

Worse than v1. Legs did not help: the carry still froze, now around
(+0.01,+0.26) with z drifting UP 27 mm. Grasps were still fine everywhere.

## probe2 — the real carry, in legs, with joints (seeds 51,59,63) — DIAGNOSIS
receipt: `results/fs_c2clean_spa_bowl_cookie_box_pos_k3_probe2` 3/3

Right after the lift to the pack's carry height (table+0.17) the arm reads

    q = [-0.029, 1.482, -0.008, **-0.097**, 0.007, 1.527, 0.757]

The Panda's joint 4 stop is ≈ −0.070. **The arm is at full elbow extension**,
so the carry is a reach saturation, not a collision — the eef holds y and z,
gives up x, and the episode burns the horizon. probe2 wedged at leg 2, retreated,
climbed to z=1.19 (joint 4 → −0.27) and the identical walk then tracked every
leg to the plate: 3/3 placed.

## v3 — climb to table+0.30, then walk the diagonal
receipt: `results/fs_c2clean_spa_bowl_cookie_box_pos_k3_v3` **3/8** (ok 51,55,65)

Height alone is not enough. The diagonal still wedges, now at (−0.04,+0.28).
Asking the elbow to fold and the base to rotate at the same time is what jams.

## probe3 — L route: climb, pull IN along −x, then swing +y
receipt: `results/fs_c2clean_spa_bowl_cookie_box_pos_k3_probe3` **5/5** on
exactly the seeds that wedged (53,57,59,61,63)

The pull-in is a pure radial move and folds the elbow monotonically
(joint 4 −0.296 → −2.329); the swing afterwards is a pure base rotation at the
short radius. Every leg of both stayed inside 12 mm.

## v4 — L route, but approached the bowl over the top
receipt: `results/fs_c2clean_spa_bowl_cookie_box_pos_k3_v4` **3/8** (ok 51,55,65)

Same carry code as probe3, opposite result: the PULL-IN itself wedged at
x≈+0.03. The difference is only how the arm got to the grasp. v4 climbed to
table+0.30 over the bowl and descended; probe3 went straight from home to the
hover. **On a 7-DOF arm the elbow posture the OSC settles into during the
approach survives the grasp and decides whether the carry can pull in.**

## v5 — probe3's approach + probe3's L carry  ← candidate
receipt: `results/fs_c2clean_spa_bowl_cookie_box_pos_k3_v5` **8/8**
(51,53,55,57,59,61,63,65), 237-473 sim steps of the 1000-step horizon.

Approach is two plain moves (hover at rim_top+0.09, then down to
rim_top−0.0255); carry is climb → pull in → swing → descend, all in legs.
ep57 is the tie case (0.144 vs 0.146 m to the cookie box): the TIE_M rule fired
and picked the wider bowl, which is the one the demos grasped. Correct.

**Formal 15-seed selection: `results/sel_c2clean_spa_bowl_cookie_box_pos_k3_v5`
→ 12/15.** Failures 52, 58, 62, all the same wedge. (Do not conclude on the
probe subset: the 8 odd seeds read 8/8.)

## What the wedge actually is (probe4/probe5/probe6)

Perception was never wrong on any of the 15 seeds. The whole gap is one
kinematic fact, now measured leg by leg:

* At the grasp the elbow is folded: joint 4 ≈ −0.38 (its stop is −0.070).
* **The lift is what straightens it.** On seed 52 the first lift leg — 46 mm of
  pure +z — already puts joint 4 at −0.073, and it never leaves the stop again
  for the rest of the episode, through every later move.
* On a seed that works (53) the same lift dips to −0.083 at leg 2 and then
  *recovers* to −0.278 by the top. Same code, same geometry to 5 mm.
  It is the OSC's null-space branch, not reach: the eef even travels the wrong
  way (asked for x−0.035, moved x+0.018) while z overshoots.
* Once pinned, nothing tried frees it: legs instead of one move (probe4),
  lift and pull-in combined (probe5), escaping at 25 mm altitude before
  climbing (probe6), five altitudes × both orders (v6), a 90° wrist yaw about
  the tool's own approach axis, and a descent back over the empty pick spot
  (v7/v8). The descent does refold it, to −0.164, and the next climb re-pins it.
* Discriminator at the moment of the grasp, over the seeds with logs:
  joint 4 ≤ −0.37 → wedges (52, 58); joint 4 ≥ −0.28 → carries (53, 57).
  The *more* folded grasp is the one that fails.

## v6 / v7 / v8 — wedge detection + a ladder of carry routes

A leg that ends >30 mm from its own waypoint is the pin, not a slow servo, so
each route is abandoned at the first such leg and the next is tried.

* v6 (5 altitudes × order): 58 ✓ 62 ✓ 53 ✓ 57 ✓ 52 ✗
  (`results/fs_c2clean_spa_bowl_cookie_box_pos_k3_v6`)
* v7 (adds posture reset + wrist yaw rungs): same 4/5, 52 ✗
  (`results/fs_c2clean_spa_bowl_cookie_box_pos_k3_v7`)
* v8 (reset descends the full grasp depth and yaws while the arm is free):
  58 ✓ 53 ✓ 52 ✗ (`results/fs_c2clean_spa_bowl_cookie_box_pos_k3_v8`)

Rung 1 is v5's route verbatim, so nothing v5 carried is put at risk; rung 2
(swing before pull-in) is what rescues 58 and 62.

**Formal 15-seed selection runs**

| version | receipt dir | score | failures |
|---|---|---|---|
| v5 | `results/sel_c2clean_spa_bowl_cookie_box_pos_k3_v5` | 12/15 | 52, 58, 62 |
| v7 | `results/sel_c2clean_spa_bowl_cookie_box_pos_k3_v7` | **14/15** | 52 |
| v8 | `results/sel_c2clean_spa_bowl_cookie_box_pos_k3_v8` | **14/15** | 52 |

v7 and v8 tie at 14/15 and fail on the same seed. v7 is frozen: it is the one
with the wider debug-seed receipt (5 diagnostic seeds vs v8's 3), and v8's last
rung drives joint 7 to its 2.897 stop, which is a hazard v7's rungs do not
carry.

---

# Mechanism gap (seed 52 only)

**Falsifiable statement.** The missing mechanism is *null-space control*. The
fair API commands Cartesian pose only; the OSC resolves the Panda's redundancy
itself, and on seed 52 it resolves the lift into a branch with joint 4 hard on
its −0.070 stop. From that branch the eef cannot reach the plate, and no
Cartesian command can leave it, because leaving it requires moving the elbow
without moving the eef — exactly what a Cartesian-only interface cannot ask
for. A single extra degree of command (a null-space posture target, a joint
command, or `api.act`, which LiberoRobot does not implement) would resolve it.

**Receipt.** `results/sel_c2clean_spa_bowl_cookie_box_pos_k3_v7`, ep52:
joint 4 reads −0.070 from the second leg of the first carry route onward and
still reads −0.070 after all six rungs — five altitudes, both orders, a 90°
yaw about the tool's own approach axis, and a descent back to the grasp depth
(which does lift it to −0.164, and the next climb re-pins it). The eef ends
0.23 m from the plate. Perception on that seed is correct: the right bowl
(+0.120,−0.070, r 0.0537), the right plate (−0.200,+0.183, r 0.0646), and the
bowl is gripped (effort 3.0, gap 0.0078 = the rim wall).

**Prediction that would falsify it.** If a route exists that frees joint 4 with
pose commands alone, then some Cartesian waypoint sequence from
(0.115,−0.021,1.193) reaches (−0.200,+0.233) without the eef reversing
direction. Six rungs found none.

---

# DECLARATION

* **Frozen version: v7.** `program.py` md5 `2abba773c99f8deec1ccb264be2e7372`
  == `program_v7.py` (identical on the Mac and on AbakaAI).
* **Selection receipt: 14/15** on the full debug band 51-65,
  `results/sel_c2clean_spa_bowl_cookie_box_pos_k3_v7`. Only seed 52 fails, on
  the documented mechanism gap above.
* **Receipt chain** (all under `results/`, probes `fs_*`, selections `sel_*`):
  probe0 perception dump 15/15 seeds → v1 4/8 → probe1 → v2 2/8 → probe2
  (diagnosis, 3/3) → v3 3/8 → probe3 (L route, 5/5) → v4 3/8 → v5 8/8 probe,
  **12/15 formal** → probe4 → probe5 → probe6 → v6 4/5 → v7 4/5 probe,
  **14/15 formal** → v8 **14/15 formal**.
* **PROVENANCE**: present, 19 entries, every one `allowed: True` with a source
  that is either a pack field or a debug-seed measurement. No `.done` read.
* Archived versions: `program_v1.py` … `program_v8.py` in the pack directory.
