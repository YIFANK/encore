# c2clean spa_bowl_on_cookie_box_pos_k3 — worker notes

Intent: "pick up the black bowl on the cookie box and place it on the plate"
Runner: `tools/fair_run.py` only. Pack: `packs/c2clean_spa_bowl_on_cookie_box_pos_k3/`.
No shared note file; every constant is re-derived from the pack + debug seeds 51–65.

## What the pack says (K=3 demos)

Keyframe EEF, base frame:

| demo | close command | EEF at close | release |
|---|---|---|---|
| 0 | t=46 (0.078, 0.064, 0.970) | t=55 (0.069, 0.064, 0.953), gap 0.0085 | (0.069, 0.224, 0.948) |
| 1 | t=46 (0.046, 0.067, 0.930) | z≈0.953 (ee_path) | (0.079, 0.256, 0.926) |
| 2 | t=38 (0.069, 0.067, 0.980) | z≈0.958 (ee_path) | (0.055, 0.258, 0.939) |

Mechanism: straight-down wrist, descend onto the bowl's rim, close, lift to
≈1.06, translate to the plate, descend, open. The gripper closes to a 0.0085 m
gap — a rim pinch across the bowl wall, not a body grasp (the bowl is 0.11 m
across and the jaws open to 0.078).

**The demo xy is a decoy.** This is a `_pos` cell and the demo scene is not the
debug scene: in the demos the plate stands on the table at y≈+0.23 and a bowl
sits on the cabinet top; on every debug seed the plate is *on the cabinet top*
at y≈−0.28 and a second bowl occupies the table spot the demo plate had. Both
the grasp and the place have to be perceived.

Recovering the demo's grasp offset: cam_high's intrinsics/extrinsics (read from
a debug-seed capture) project the demo EEF into the 128 px keyframes and it
lands on the target bowl's +y rim edge. Ray-casting the bowl's rim-centre pixel
of `demo0_t0000.png` onto the rim-top plane gives the demo bowl centre
(0.081, 0.024); the EEF at close was (0.069, 0.064), i.e. **+0.040 m along +y**,
which is the gripper's closing axis. Rim radius from the same cast: 0.058.

## Scene, measured on debug seeds 51–65 (identical layout, positions jittered)

| thing | where | z |
|---|---|---|
| table | — | 0.9013 |
| cookie box top | under the target bowl | 0.9201 |
| **target bowl** rim top | (0.055…0.080, 0.012…0.040), 0.110 across | **0.9706** |
| distractor bowl rim top | (≈0.06, ≈0.19), 0.110 across | 0.9518 |
| cabinet top | y ∈ [−0.36, −0.145] | 1.1268 |
| **plate** top | on the cabinet, 0.135 across | **1.1465** |

Two independent cues name the target and agree on all 15 seeds: its rim is
0.019 m higher than the other bowl's (the cookie box lifts it), and the cookie
box's checker red is the only r>115/g<95/b<95/r−g>45 material in the table band
(9–23 px/seed, centroid 0.045 m from the target vs 0.20 m from the distractor).
The program selects on the red cue and falls back to the highest rim.

The plate is clipped by the left edge of the frame, so its y midpoint is
biased; its x extent (0.134–0.136 every seed) is the true diameter and the
unclipped +y edge minus that radius gives the centre.

## Version log

### v1 — sensor dump, no motion
H: develop perception offline from debug RGB-D, zero sim steps.
E: `api.log` truncates at 2000 chars, so the 60 kB chunks were cut.
V: rerun with 1900-char chunks.

### v2 — sensor dump, 1900-char chunks
E: full cam_high RGB + depth recovered for all 15 debug seeds; all the scene
measurements above come from it. V: perception developed and validated offline
(target bowl and plate found on 15/15) before any motion was run.

### v3 — first full attempt · probe 51…65 odd → **0/8**
H: perceive bowl + plate, grasp at rim_top−0.018 with the +0.040 y offset,
carry at 1.215, release at plate_top+0.037.
E: the grasp held on 8/8 (gap 0.0076–0.0085, effort 3.0) and the bowl reached
the plate every time, but the seed-51 gif shows it sitting on the plate's −y
edge. The EEF was aimed *at* the plate centre while the rim-pinched bowl hangs
0.040 m to −y of the EEF.
V: the place aim must carry the same offset the grasp used.

### v4 — place aim = plate centre + grasp offset · probe → **7/8** (fail 61)
E: seven seeds succeed at the release step (the episode terminates there). Seed
61's grip gap collapsed 0.0080 → 0.0041 during the lift and it missed.
V: the trail offset can change when the bowl slips; measure it instead.

### v5 — re-measure the held bowl's centre in flight · probe → **7/8** (fail 61)
E: the measurement is clean (trail −0.050 ± 0.001 on 7 seeds, logged
`HELD n≈1550 ext≈(0.085,0.080)`), and seed 61 reads differently — n=3638,
ext (0.108,0.103) — i.e. the bowl had slid *down* in the jaws. Correcting the
aim alone did not save it: its base ended 0.017 m below the plate surface at
release, so it was driven into the plate and shoved it off the cabinet.
V: the release height, not just the aim, has to follow the real grip.

### v6 — also measure the hang (EEF→bowl base) in flight · probe → **7/8** (fail 61)
E: the hang is measurable (0.031 ± 0.001 on the seven, 0.059 on seed 61,
confirming a 0.026 m slip) and the release height was corrected, but seed 61
still lost the bowl off the plate.
V: correcting downstream of a bad grip is not enough — fix the grip.

### v7 — deepen the grasp command to rim_top−0.028 · probe → **8/8**
H: `api.move` stops within POS_TOL (0.012) and was landing ~7 mm high of the
0.018 command, so the jaws bit the rim's thin top edge; commanding 0.028 makes
the realised bite the demo's own.
E: descent now lands at 0.949–0.952 (vs 0.959), the closed gap is 0.0078–0.0092
on every seed (vs 0.0041 on the old seed 61), no slip anywhere.
V: selected. **Formal 15-seed run: 15/15**
(`results/sel_c2clean_spa_bowl_on_cookie_box_pos_k3_v7`).

### v8 — margin probe (NOT a selection candidate)
Place aim displaced +0.018 m in y to measure how much slack the place has.
E: **4/6** (fail 53, 59). An 18 mm displacement of the place aim breaks a third
of the seeds, so the place envelope is narrower than 18 mm on the +y side —
which is what the geometry says it should be (plate radius 0.067, bowl radius
0.055, so ~0.012 m of overhang is all the plate can take). v7 aims within about
2 mm of the plate centre on every debug seed, i.e. it sits in the middle of a
narrow but real basin rather than on its edge.
V: no change to the frozen program; recorded as the measured aim envelope.

## DECLARATION

- **Frozen version: v7.** `packs/c2clean_spa_bowl_on_cookie_box_pos_k3/program.py`
  md5 `a08b80971b816f6c39d3f976d7d4dbd0` == `program_v7.py` (same md5, verified
  on the cluster).
- **Selection receipt: 15/15** on the full 15 debug seeds (51–65),
  `results/sel_c2clean_spa_bowl_on_cookie_box_pos_k3_v7`.
- Receipt chain (probe subset 51,53,…,65 unless noted):
  v1 sensor dump (no motion) · v2 sensor dump (no motion) ·
  v3 **0/8** · v4 **7/8** · v5 **7/8** · v6 **7/8** · v7 **8/8** →
  formal **15/15** · v8 margin probe **4/6** (aim displaced 18 mm, diagnostic
  only, not a selection candidate).
- PROVENANCE: present, 14 entries, every one sourced to this pack, a debug-seed
  measurement, or generic camera/controller mechanics; all `allowed: True`.
  Checked against the eval gate's own rules: no forbidden tokens, no `.done`
  read.
- Splits respected: only seeds 51–65 were ever run, always `--split debug`.
  Seeds 1–50 were never touched. Every program ran under `tools/fair_run.py`;
  `tools/fewshot_run.py` was never invoked. Cluster writes stayed inside
  `packs/c2clean_spa_bowl_on_cookie_box_pos_k3/` and
  `results/*c2clean_spa_bowl_on_cookie_box_pos_k3*`.

### What the cell turned on
Three things, in the order they bit:
1. A `_pos` cell can move the goal to a different *support*: the pack demos
   place on a table-level plate, every debug seed's plate is on the cabinet top
   0.25 m higher. Only the mechanism transfers; both endpoints are perceived.
2. A rim-pinched bowl's centre trails the EEF by the whole grasp offset, at the
   place as well as the grasp. Aiming the EEF at the plate centre puts the bowl
   on the plate's edge (v3: 0/8 → v4: 7/8).
3. `api.move` stops within POS_TOL, so a grasp-depth *command* is not a grasp
   depth. Commanding the demo's 0.018 realised ~0.011 and bit the rim's thin
   top edge; the one seed that slid through the jaws could not be rescued by
   any downstream correction, and deepening the command to 0.028 fixed it
   outright (v6: 7/8 → v7: 8/8, 15/15).
