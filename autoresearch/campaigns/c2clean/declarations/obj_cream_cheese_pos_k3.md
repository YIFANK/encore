# c2clean / obj_cream_cheese_pos_k3

Intent: "pick up the cream cheese and place it in the basket".
Runner: `tools/fair_run.py` only. Pack: `packs/c2clean_obj_cream_cheese_pos_k3/` (K=3).

## What the pack says

3 demos, all with the same shape:

| demo | close (eef xyz)            | release (eef xyz)          |
|------|----------------------------|----------------------------|
| 0    | (0.0558, -0.1171, 0.0108)  | (0.0298, +0.2661, 0.1789)  |
| 1    | (0.0231, -0.1091, 0.0101)  | (0.0654, +0.2396, 0.2193)  |
| 2    | (0.0243, -0.1160, 0.0101)  | (-0.0354, +0.2555, 0.1517) |

So: a top-down grasp with the eef ~10 mm above the table, then a carry to
y ~ +0.25 and an open. Keyframe images show six props (two cans, two tall
boxes, two small boxes lying flat) plus a wicker basket on the +y side; the
gripper closes on the small **blue** box. Demo xy is a decoy for a `_pos`
cell, so only the *mechanism* (top-down pinch at z~0.010, release over the
basket at z~0.15-0.22) was carried across; every position is re-perceived.

## v0 -- perception probe (seeds 51,53)

Hypothesis: need the scene geometry before anything else.
Evidence: cam_high is a 512x512 pinhole, K=[[618.04,0,256],[0,618.04,256]],
t_base_cam puts the camera at (0.897, 0, 0.65) looking back along -x. Dense
deprojection matches `api.deproject` to <0.5 mm. Tabletop plane reads
**z = 0.000**; prop tops fall in [0.01, 0.145]; the arm occupies z > 0.25.
Verdict: usable. (A log-chunking bug truncated the RGB dump -- `api.log`
clips a message at ~2000 chars, so dump chunks must be <=1500.)

## v1 -- top-down height map (8 seeds)

Hypothesis: max-z per 5 mm cell + connected components segments the props.
Evidence: only 6 clusters, and two props were missing -- they had been
absorbed into the arm's cluster, which spans x[-0.227,-0.097] at z up to 0.40
and sits directly over the -x half of the table.
Verdict: segmentation works, but the arm has to be dealt with.

## v2 -- raw depth dump (all 15 debug seeds)

Confirmed the merge: the two "missing" props really do sit at x ~ -0.14..-0.19,
underneath the reset arm. Adding a **ceiling at z < 0.17** (below the arm, above
the tallest prop at 0.144) splits everything cleanly. All 15 debug seeds then
give exactly 7 clusters:

- basket: n~490, top 0.143-0.144, grey top (182,182,182)
- 2 cans: top 0.081
- 2 tall boxes: top 0.141-0.142
- 2 flat boxes: top 0.019-0.020 -- the target class

Identity inside the flat class is colour, and the margin is large:
blue-minus-red of the top surface is **+24..+30** for the cream cheese and
**-55..-65** for the other flat box, on every one of the 15 debug seeds.
The `_pos` perturbation on this cell is small (target x in [-0.147,-0.135],
y in [+0.054,+0.062]; basket centre varies ~1 cm) but nothing is hard-coded.

## v3 -- first end-to-end attempt

Pipeline: perceive -> park the arm at (0.00, 0.30, 0.40) (clears the frame,
sits above the basket which is measured before the move) -> re-perceive
without occlusion -> top-down grasp at the target's bbox centre with jaws
along base y (the boxes lie long-axis-along-x, 0.080 x 0.045) -> lift to
0.26 -> release at 0.20 over the basket centre.

Probe receipt: **8/8** on seeds 51,53,...,65
(`results/fs_c2clean_obj_cream_cheese_pos_k3_v3`).
Parking fixed the footprint: the target reads dx=0.080 on every seed instead
of the 0.050-0.080 the occluded view gave. The descent stalls at eef
z=0.0195 rather than the commanded 0.010 (contact with the box top), and the
close still bites: width 0.0422, effort 3.0, held all the way to release.

## v4 -- hardened

Three changes, no change of mechanism:
1. Target selection is a **rank** (argmax of blue-minus-red over the flat
   class), not a threshold, so a colour shift on an unseen seed degrades
   gracefully.
2. Grasp is **verified** (effort 3.0 and 0.010 < width < 0.065 after both the
   close and the lift) with up to 3 attempts, re-perceiving between attempts.
3. Basket re-measured from the post-park frame.

## Formal selection

Both v3 and v4 run on the full 15 debug seeds (51-65).


Receipt: **15/15** for v3 (`results/sel_c2clean_obj_cream_cheese_pos_k3_v3`)
and **15/15** for v4 (`results/sel_c2clean_obj_cream_cheese_pos_k3_v4`).
In v4 every one of the 15 seeds grasped on attempt 0 -- the retry path never
fired -- with width 0.04222 and effort 3.0 held from close to release, and the
target's blue-minus-red read +23..+24 against the other flat box's -55..-65.

## Aim envelope (v4 derivatives, seeds 51,53,55,57)

A score alone does not say how much margin there is, so the grasp aim was
deliberately displaced in y (the axis the jaws close along) and re-run:

| dy      | result | dir                                                    |
|---------|--------|--------------------------------------------------------|
| +0.012  | 4/4    | `results/fs_c2clean_obj_cream_cheese_pos_k3_aim_p012`  |
| -0.012  | 4/4    | `results/fs_c2clean_obj_cream_cheese_pos_k3_aim_m012`  |
| +0.020  | 0/4    | `results/fs_c2clean_obj_cream_cheese_pos_k3_aim_p020`  |

So the grasp tolerates roughly +-15 mm of aim error on a target whose measured
centre is repeatable to ~2 mm across seeds. The failure mode at +20 mm is the
jaws closing off the edge of a 45 mm-wide box, which is what the geometry
predicts. (Archived as `program_v4probe_dy*.py`; these are probes of v4, not
candidate versions.)

## DECLARATION

- Frozen version: **v4**. `program.py` md5 `01b132ab841866608fd8c51a7f1887aa`
  == `program_v4.py` md5 `01b132ab841866608fd8c51a7f1887aa`.
- Selection receipt: **15/15** on the full 15 debug seeds 51-65,
  `results/sel_c2clean_obj_cream_cheese_pos_k3_v4`.
- Receipt chain:
  - v0 perception probe, seeds 51,53 -- `results/fs_..._v0` (no manipulation)
  - v1 height-map probe, 8 seeds -- `results/fs_..._v1` (no manipulation)
  - v2 depth dump, 15 seeds -- `results/fs_..._v2` (no manipulation)
  - v3 first end-to-end -- probe 8/8 `results/fs_..._v3`, selection 15/15
    `results/sel_..._v3`
  - v4 hardened (ranked selection, verified grasp + retry) -- selection 15/15
    `results/sel_..._v4`
  - v4 aim-envelope probes -- 4/4 at dy=+-0.012, 0/4 at dy=+0.020
- PROVENANCE: present in `program.py` as a top-level literal dict covering
  every calibrated constant; all sources are this pack's keyframes/ee paths,
  debug-seed (51-65) measurements, or generic camera/controller mechanics.
- Clean room: no benchmark asset, init state, gt trace, other-campaign
  artifact, or any pack's program/NOTES was read. No prior-context LIBERO
  constants were used -- table height, prop height classes, the box footprint,
  the grasp height and the blue-vs-brown colour cue were all re-measured here.
