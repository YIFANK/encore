# c2clean / spa_bowl_next_to_plate_pos_k3

Intent: "Pick up the black bowl next to the plate and place it on the plate."
Runner: tools/fair_run.py only. Debug seeds 51-65. No shared note file.

## Pack reading (K=3)

Every demo has the same 4-keyframe shape: reset -> gripper_cmd flips to +1
(the close = grasp) -> gripper_cmd flips to -1 with the fingers shut (the
release = place) -> retreat.

| demo | grasp EEF | release EEF | closed finger gap |
|------|-----------|-------------|-------------------|
| 0 | (0.0084, 0.2540, 0.9226) | (0.0584, 0.1525, 0.9355) | 0.0068 |
| 1 | (0.0118, 0.2615, 0.9395) | (0.0428, 0.1531, 0.9394) | 0.0083 |
| 2 | (0.0145, 0.2700, 0.9426) | (0.0647, 0.1680, 0.9373) | 0.0172 |

A ~7 mm closed gap says the demos pinch the bowl WALL, not the whole bowl.

## Scene, re-derived from the debug seeds (cam_high RGB-D, logged out with
## zlib+base64 through api.log and reconstructed offline)

Table z = 0.9010 on all 15 seeds. Objects (height above table, footprint):

* plate         (0.06, 0.03)  h 0.019, 0.135 x 0.135
* cookie box    (0.06, 0.20)  h 0.020, 0.080 x 0.060
* bowl A        (0.01, 0.31)  h 0.051, rim radius 0.0537
* bowl B        (-0.18, 0.32) h 0.051, rim radius 0.0537
* small tin     (-0.20, 0.19) h 0.043, rim radius 0.0421
* cabinet y < -0.05, stove slab x -0.23 / y -0.12: cropped out.

**_pos swaps the plate and the cookie box.** Projecting the pack keyframe
images through the (verified) cam_high model puts the demo plate at
(0.065, 0.20) and the demo box at (0.098, 0.03) -- exactly each other's debug
slots. So the demo release xy is NOT transferable as an absolute; the target
and the goal must both be perceived.

Demo grasp minus the bowl-A centre = (-0.004,-0.056), (0.000,-0.049),
(0.003,-0.040): a pure -y offset of ~0.048, just inside the measured rim
radius 0.0537. Demo release minus the demo plate centre = the same ~-0.045 in
y, i.e. the bowl centre lands on the plate centre. Grasp z = tz+0.022..0.042,
release z = tz+0.036.

## Versions

### v1 -- perceive both, rim-pinch at the demo offset

Hypothesis: target = nearest high-band cluster to the plate; grasp = rim pinch
0.048 m on the -y side at tz+0.030; place = plate centre carried with the same
offset, release at tz+0.040.

Two height bands do the segmentation work: the low band (tz+0.012..0.030)
holds only flat things and the plate is the one with min-span >= 0.10 (the box
is 0.06 wide); the high band (tz+0.035..0.20) drops the plate and the box,
which is what un-fuses the target bowl from the box on seeds 58 and 62 (they
merge into one 0.165 x 0.170 blob in a single-band clustering).

Offline replay of the perception on all 15 debug clouds: exactly one plate
candidate and the correct target bowl every time, rim radius 0.0537 +/- 0.0002.

Evidence: probe seeds 51,53,55,57,59,61,63,65 -> **8/8**
(results/fs_c2clean_spa_bowl_next_to_plate_pos_k3_v1).

Receipt notes: `effort` is a gap flag, not a force -- it reads 3.0 whenever the
gap exceeds ~5 mm and 0.05 below that, so the post-lift drop to 0.0046 m is the
jaws finishing their squeeze on the thin rim, NOT a dropped bowl (ep53 frame 26
shows the bowl still in the jaws while effort reads 0.05). The place descent
stalls ~0.027 m above the command, so the bowl is released from just above the
plate.

Verdict: **selected**. Full-15 formal run seeds 51-65 -> **15/15**
(results/sel_c2clean_spa_bowl_next_to_plate_pos_k3_v1).

### Aim envelope (margin check, post-selection, not a selection run)

The score alone does not say how much aim error the rim pinch tolerates, so the
grasp offset was displaced +/- 8 mm and re-run on seeds 51,55,59,63:

| GRASP_OFFSET | seeds 51,55,59,63 |
|--------------|-------------------|
| 0.040 | 4/4 (results/fs_..._env040) |
| 0.048 (frozen) | 4/4 within the 15/15 selection run |
| 0.056 | 4/4 (results/fs_..._env056) |

So the pinch survives the whole 0.040-0.056 band the three demos themselves
span; the frozen value sits in the middle of a plateau rather than on an edge.

## DECLARATION

* Frozen version: **v1**. `packs/c2clean_spa_bowl_next_to_plate_pos_k3/program.py`
  md5 `170c65b2a138e1848b55720b2cea4c31` == `program_v1.py` (same md5).
* Selection receipt: **15/15** on the full debug split (seeds 51-65),
  `results/sel_c2clean_spa_bowl_next_to_plate_pos_k3_v1`.
* Receipt chain:
  * dump / dump2 -- perception probes only (no manipulation), 0/15 by design.
  * v1 probe (51,53,55,57,59,61,63,65): 8/8,
    `results/fs_c2clean_spa_bowl_next_to_plate_pos_k3_v1`.
  * v1 formal (51-65): 15/15,
    `results/sel_c2clean_spa_bowl_next_to_plate_pos_k3_v1`.
  * envelope 0.040 / 0.056 (51,55,59,63): 4/4 and 4/4.
* PROVENANCE: present as a top-level literal dict in program.py, covering
  R_DOWN, the workspace crop, the table bin, both height bands, the plate span
  test, the grasp offset, and the grasp / place / carry heights. Every one is
  sourced to a pack keyframe field or a debug-seed measurement.
* No shared note file was read or written. Only the pack dir and
  `results/*c2clean_spa_bowl_next_to_plate_pos_k3*` were written on the cluster.

STOP.
