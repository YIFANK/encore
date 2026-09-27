# c2clean / goal_put_bowl_on_stove_task_k3 — "Put the plate on the stove"

Re-authored cell. The two packs both name half the intent and neither solves it:

- target pack (`..._task_k3`, "put the bowl on the stove"): three K=3 demos that
  grasp the **bowl** at its rim (eef y within 0.011 m of the rim edge, tips low
  on the wall) and release over the **stove** at xy ≈ (-0.26, 0.26).
- mate pack (`..._task_mate`, "push the plate to the front of the stove"): the
  gripper never closes; it descends at xy ≈ (0.04, -0.02) — **on the plate** —
  and sweeps +y.

So the packs give me two anchors (which cluster is the plate, which is the
stove) and one mechanism (rim pinch, straight-down wrist, grasp at the rim
edge). Everything metric was re-measured on debug seeds.

## Scene, measured (cam_high RGB-D, debug seeds 51–65)

| quantity | value | how |
|---|---|---|
| table plane | 0.9011 | modal height of the cloud |
| plate | ⌀0.135–0.140, lip 0.019 above table, inner floor 0.006 | wrist-depth cross section, v1 |
| bowl | ⌀0.11, 0.051 above table | cam_high components |
| stove slab | 0.19 × 0.19, top 0.031 above table | cam_high components |
| fingertips | 0.0086 **below** the reported eef | open-jaw press on bare table stalls at eef = table + 0.0086 (v1) |
| api.move | returns anywhere inside a 12 mm ball (POS_TOL) | generic controller mechanics; every pose is re-issued with a bias |

The plate is a shallow saucer, not a flat disc: at fingertip height its rim is a
wedge ≈0.022 m thick, which is what the jaws bite.

## Version chain

| ver | hypothesis | evidence | verdict |
|---|---|---|---|
| v0 | perception probe — stream RGB-D out through `api.log` (zlib+base64 chunks) | scene identified: plate, bowl, bottle, stove, cabinet, rack, small box | scene mapped |
| v1 | measure the mechanism: fingertip offset, plate cross-section, one rim pinch | press stalls at eef = table+0.0086; plate profile lip 0.0185 → centre 0.0055; the trial closed **in air** (width 0.001, effort 0.05) because every `api.move` stops 12 mm short | tip offset + 12 mm tolerance are the two facts that matter |
| v2 | rim pinch at tips 0.004 above the table with **bias-corrected** moves | 4/4 on 51,53,55,57. Closes at gap 0.0224 effort 3.0; after the lift the gap settles to 0.0059 — the plate has swung edge-down | works, but the hold is 0.9 mm above the `HELD_MIN_GAP` threshold |
| v3 | harden perception (mask tall columns so the bottle stops bridging stove+arm), contact-seeking release | **6/8**. ep63: plate **fused with the bowl** (one component, h=0.051) → shape test rejected it. ep55: plate **dropped mid-carry** | two distinct, diagnosable failures |
| v4 | (a) find the plate in a height **band** (0.012–0.026) the bowl/stove/tall props fall outside of; (b) carry at 0.28 not 0.16 — a rim-pinched plate hangs ~0.9 diameters edge-down and at 0.16 it drags on the table and is torqued out of the jaws; (c) drop recovery: re-perceive and re-pick, up to 3 rounds | **8/8** on 51,53,…,65. ep63 plate found and placed; ep55 carried clean | frozen |

## Two things worth stating plainly

1. **The predicate fires mid-carry.** LIBERO ends the episode the moment its
   goal test passes, and on every seed that happened while the plate was still
   hanging over the stove (v2: 131 sim steps, before the release). I did not
   build anything on that — the program still lowers the payload to the slab and
   opens, and where the episode survives long enough to re-perceive (ep63) the
   final height map shows the plate resting on the stove (slab component
   h: 0.031 → 0.047). Success is read only post-episode from results.jsonl.
2. **The hold is thin.** The bite starts at 22 mm across the rim wedge and
   settles to 5.9 mm once the plate swings edge-down — 0.9 mm above the gap
   that still counts as holding. The carry height is what makes it survive;
   the drop recovery is there for when it does not.

## DECLARATION

- Frozen version: **program_v4.py**, copied to `program.py` —
  md5 `5004b8684058cf4db69adf0a2d6f3a84` for both.
- Selection receipt: **15/15** on the full debug band (seeds 51–65), one formal
  run — `results/sel_c2clean_goal_put_bowl_on_stove_task_k3_v4`.
- Receipt chain: v0 probe · v1 probe · v2 4/4 (51,53,55,57) ·
  v3 6/8 (51,53,…,65) · v4 8/8 (51,53,…,65).
- PROVENANCE present in program.py: 11 constants, every one sourced to a named
  pack field or a debug-seed measurement.
