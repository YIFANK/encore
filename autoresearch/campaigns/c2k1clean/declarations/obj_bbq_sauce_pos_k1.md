# c2k1clean / obj_bbq_sauce_pos_k1 — working notes

Intent: "pick up the bbq sauce and place it in the basket". K=1 pack, LIBERO
object-swap `_pos` cell. Runner: `tools/fair_run.py` only.

## Pack reading (before any run)

`pack.json`: one demo, 134 steps, 4 keyframes.
- t=0   ee (-0.1421, 0.0147, 0.2543), gripper open (cmd -1)
- t=54  ee ( 0.0274,-0.1298, 0.0732), gripper cmd flips to CLOSE  -> grasp point
- t=117 ee (-0.0067, 0.2406, 0.1736), gripper cmd flips to OPEN   -> release point
- t=133 ee ( 0.0066, 0.2628, 0.2285), retreat
Carried finger gap at t=117 = |0.0187 - (-0.0185)| = 0.0372; open = 0.0724.

Keyframe diff (t0000 vs t0133, my own local image arithmetic on the pack's own
PNGs) isolates the object that leaves the table: silhouette rows 64-80, cols
44-52 in the 128-px keyframe -> u 176..212, v 256..324 in 512-px capture space.
That is the pixel identity cue for "bbq sauce".

## Version log

### v1 — perception probe (seeds 51,53,55,57) — receipt fs_..._v1, 0/4 (no motion)
Hypothesis: cam_high RGB-D is enough to map the scene.
Evidence: table plane z = 0.0033 (depth mode, 120k px). Six clusters. The arm
(top z 0.398) fuses with a table object in the xy grid.
**All four seeds are byte-identical except the basket** — (0.077,0.252),
(0.060,0.254), (0.080,0.237), (0.085,0.251). Verdict: in this cell `_pos`
moves the goal fixture, not the objects. Program still perceives both per
episode so it does not depend on that.

### v2 — geometry probe (seeds 51,52) — receipt fs_..._v2, 0/2 (no motion)
Hypothesis: cutting at z<0.20 unfuses the arm.
Evidence: it does; a 6th object appears (top 0.113, rgb 58,27,8 — a second,
*orange* sauce bottle at (-0.148,0.059)). Camera pose logged: cam at
(0.8966, 0, 0.65), forward (-0.849, 0, -0.529) — a 32-degree grazing view from
+x. Live cluster u=(177,215) v=(238,316) matches the pack's keyframe-diff
silhouette -> **target identified**: top 0.148, body rgb (82,64,56), cap rgb
(99,98,98) — the only tall object with a bright *neutral* cap (green bottle
cap (22,54,35), orange bottle cap (68,23,6)). Verdict: identity cue = tall +
neutral bright cap. Holds on every debug seed seen since.

### v3 — first full pick-and-place (seeds 51,53) — receipt fs_..._v3, 0/2
Hypothesis: aim = cap centroid corrected one radius toward the camera.
Evidence: descent to z=0.073 **blocked at z=0.1098, residual 0.0431**, eef
deflected +0.031 in x; close grabbed air (width 0.001). GIF confirms the
gripper is over the right bottle. Verdict: identity right, aim wrong.

### v4 — descent sweep in x at y=-0.099 — receipt fs_..._v4, 0/1
Evidence: x=0.031 blocked at z=0.1116; x=0.041 blocked at z=0.1115;
x=0.021 **clean** (res 0.0119, reached z=0.0834) but closed on air (0.0011).
Verdict: a clean descent and a successful grasp are not the same event.

### v5 — 2-D aim ladder with re-perception — receipt fs_..._v5, 0/2
Two defects found, both mine:
1. Re-perceiving with the arm parked over the target makes the arm occlude it,
   and the ladder silently re-targeted the *green* bottle (rungs 2-4).
2. The episode ran out of LIBERO's 500-step horizon; after termination the env
   refuses steps, so `grip(0.0)` never moved the fingers and
   `gripper()` reported effort 3.0 with width 0.0798 (fully open) — a **false
   "held"**. Held test must bound width on both sides (0.015 < w < 0.060) and
   the step budget must be spent on at most ~2 grasp attempts.

### v6 — wrist-camera refine — receipt fs_..._v6, 0/1
Hypothesis: the grazing cam_high biases the centroid; a top-down wrist view
will not.
Evidence: from eef (0.0302,-0.0998,0.2801) the wrist cam sits at
(0.0864,-0.0999,0.3736) looking down (fwd -0.065,0.003,-0.998). Cap box
x 0.036..0.068, y -0.115..-0.084 -> **cap centre (0.052,-0.0995), cap 32x31 mm**.
This agrees with the cam_high *corrected* centre and disagrees with the raw
cap median (0.061). Verdict: the bottle's xy is settled at (0.052,-0.0995);
aiming the eef there still blocks at z=0.12. So the remaining unknown is
mechanical, not perceptual.

### v7 — mechanics probe (seed 51) — receipt fs_..._v7, 0/1 (probe)
Hypothesis: the block is either contact or reach saturation; a fine-grained
descent ladder tells them apart.
Evidence: on **bare table** at (0.052, 0.000) the eef descends freely to
z=0.059 with both jaws open and jaws closed — so no reach limit, no table
guard. At the bottle (0.052,-0.0995) it descends freely to eef z=0.121 and
then **stalls at eef z≈0.112** while drifting +0.017 in x.
Verdict: contact, at a fixed height. Bottle top 0.147, so the gripper's palm
meets the cap when the eef is 0.035 below the object top. The demo's
ee z=0.0732 is therefore NOT reachable on this object — first hint that the
demo is not about this object.

### v8 — grasp above the palm collision (seeds 51,53) — receipt fs_..._v8, 0/2
Hypothesis: grasp at top-0.029 instead of the demo's depth.
Evidence: **grasp works** — res 0.0078, gap 0.0334, effort 3.0, lifted,
carried, released over the basket; 294 sim steps of 500. The GIF shows the
bottle sitting inside the basket. And the benchmark still says False.
Verdict: the motion is right and the *object* is wrong.

### v9 — identity probe (seed 51) — receipt fs_..._v9, 0/1 (no motion)
Hypothesis: read the labels rather than infer them. Streamed base64 RGB crops
of every cluster out through api.log and reassembled them locally.
Evidence (labels legible at 512-px capture resolution):
| cluster | top | body rgb | cap rgb | label |
|---|---|---|---|---|
| o2 | 0.148 | (82,64,56) | (99,98,98) | **Tomato Ketchup** |
| o1 | 0.147 | (59,68,58) | (22,54,35) | **Ranch Dressing** |
| o3 | 0.081 | (66,68,75) | (81,83,92) | open soup can |
| o4 | 0.113 | (58,27,8) | (68,23,6) | **BBQ** |
Verdict: **the pack's K=1 demo picks up the ketchup, not the bbq sauce.**
The keyframe-diff silhouette (u 176..212) is cluster o2 = ketchup. This is a
swap cell: the demo names the wrong target and every demo-derived xy/z is a
decoy. The only sound target cue is the object's own appearance.

### v10 — BBQ target — receipt fs_..._v10 4/4, sel_..._v10 **15/15** — FROZEN
Identity rule: argmax over bottle-height clusters (0.07 < top < 0.19, n > 500,
rim radius < 0.05) of (capR-capG) + (bodyR-bodyG). Measured on seed 51:
BBQ 75.0, ketchup 19.3, soup can -3.5, ranch -41.1 — a 4x margin, and the same
ordering on every debug seed run.
Geometry: coarse centre from cam_high, then a top-down cam_arm_wrist capture
from (cx-0.02, cy, 0.28) refines the cap box (the grazing 32-degree cam_high
view biases the visible centroid toward the camera by ~2 cm; the wrist view
does not). Grasp at top-0.029, clear of the palm/cap collision measured in v7.
Held test bounds the gap on BOTH sides (0.012 < w < 0.060) so a terminated
episode's frozen-open gripper cannot read as "held" (the v5 false positive).
Release at basket rim + 0.033 (the one demo-derived constant that survives:
it is a clearance over the goal fixture, not a statement about the object).
Every debug seed grasps on rung 0.

## DECLARATION

- **Frozen version: v10.** `packs/c2k1clean_obj_bbq_sauce_pos_k1/program.py`
  md5 `3d29e1c708208eaaa97ca2ca8c23c4f5` == `program_v10.py` (same md5, verified
  on the cluster and locally).
- **Selection receipt: 15/15** on the full 15-seed debug split (51-65),
  `results/sel_c2k1clean_obj_bbq_sauce_pos_k1_v10`.
- **PROVENANCE** present in program.py: 8 constants, every source either a
  pack field or a debug-seed measurement recorded above.
- Never ran `--split eval`; never touched seeds 1-50; only wrote under
  `packs/c2k1clean_obj_bbq_sauce_pos_k1/*` and
  `results/*c2k1clean_obj_bbq_sauce_pos_k1*`.

### Receipt chain
| v | what | seeds | dir | result |
|---|---|---|---|---|
| v1 | perception probe | 51,53,55,57 | fs_..._v1 | 0/4 (no motion) |
| v2 | geometry probe | 51,52 | fs_..._v2 | 0/2 (no motion) |
| v3 | pick-place, cap-centroid aim | 51,53 | fs_..._v3 | 0/2 |
| v4 | descent sweep in x | 51 | fs_..._v4 | 0/1 |
| v5 | 2-D aim ladder | 51,53 | fs_..._v5 | 0/2 |
| v6 | wrist-camera refine | 51 | fs_..._v6 | 0/1 |
| v7 | mechanics probe | 51 | fs_..._v7 | 0/1 (probe) |
| v8 | grasp above palm collision | 51,53 | fs_..._v8 | 0/2 (ketchup placed) |
| v9 | label identity probe | 51 | fs_..._v9 | 0/1 (no motion) |
| **v10** | **BBQ target** | 51,53,55,57 / **51-65** | fs_..._v10 / **sel_..._v10** | 4/4 / **15/15** |

### The one thing that mattered
Eight versions failed for two different reasons and only one of them was the
obvious one. The aim/mechanics work (v3-v8) ended in a clean grasp, a clean
carry and a clean release into the basket that the benchmark scored False,
because the pack's single demonstration picks up the **Tomato Ketchup** bottle
while the intent sentence says **bbq sauce**. Every constant I had derived
from the demo's keyframes — grasp xy, grasp z, the keyframe-diff silhouette —
described the wrong object, and each one looked like a perception bug.
The demo was only falsified by reading the labels off the pixels
(v9), which is the cheapest experiment in the whole chain and should have come
first. Under a swap cell, treat the demo as evidence about the *motion* and
never about the *target*.
