# c2k1clean / obj_alphabet_soup_pos_k1

Intent: "pick up the alphabet soup and place it in the basket".
Runner: `tools/fair_run.py` only. Pack: `packs/c2k1clean_obj_alphabet_soup_pos_k1/`
(K=1, demo0, 148 steps, 4 keyframes). No note file; no cross-cell inputs.

## What the pack says

`demos[0].keyframes`: t=0 approach, **t=42 close at ee (-0.1112, -0.2471, 0.0447)**
with `gripper_cmd +1`, t=128 **release at ee (-0.0352, 0.2252, 0.1525)**, t=147 retreat.
`ee_path6` shows a transport leg at z 0.26-0.31.

Keyframe images (128 px) identify the target by *appearance*, not position: the
object held over the basket at t=128 is a **can with a blue upper label band and an
orange/yellow lower band under a grey lid**. In the demo's t=0 frame that can sits
at the back-left, partly overlapping the milk carton in image space; it is gone in
t=128. So: target = the blue-labelled can. The demo's grasp xy is a `_pos` decoy and
was not used.

## Scene, measured on the debug seeds (v1 dump)

v1 was a no-motion program that pushed the `cam_high` RGB-D frame out through
`api.log` (zlib+base64, 1800-char chunks) so perception could be designed offline.
Deprojected to base frame, table plane z = **0.0015 m** on every seed.

Components of the top-down height map (1 cm cells), consistent across seeds 51-65:

| prop | top z | footprint | top-band chroma (r,g,b) | b-r |
|---|---|---|---|---|
| basket | 0.142 | 0.159 x 0.170 | 0.333 0.334 0.334 | +0.001 |
| **soup can (blue)** | **0.081** | **0.064 x 0.069** | 0.291 0.316 0.394 | **+0.103** |
| other can (red/green) | 0.081 | 0.064 x 0.068 | 0.442 0.300 0.258 | -0.185 |
| salad-dressing bottle | 0.148 | 0.036 x 0.062 | 0.130 0.557 0.312 | +0.182 |
| milk carton | 0.141 | 0.053 x 0.054 | 0.450 0.295 0.255 | -0.196 |
| flat orange box | 0.019 | 0.075 x 0.039 | 0.513 0.294 0.192 | -0.321 |
| flat blue box | 0.020 | 0.062 x 0.042 | 0.282 0.310 0.408 | +0.126 |

Two traps found and handled:

1. **The arm fuses with the target.** The arm's start column sits at
   x in [-0.24,-0.09], y in [-0.10,0.13] — the soup can at (-0.149, 0.060) falls
   inside it, so max-z footprint clustering swallows the can into the arm
   component. Fixed by a hard **pixel** z ceiling of 0.20 m before gridding
   (tallest prop = 0.148; arm column = 0.482).
2. **Blue alone names the wrong prop.** The flat blue box scores b-r = +0.126,
   *higher* than the soup can's +0.103. Colour is only ranked inside a height +
   footprint gate (top in [0.050,0.125], both footprint axes in [0.035,0.095]),
   which admits exactly the two cans; within that gate the margin is 0.103 vs
   -0.185.

Grasp height comes from the pack: the demo closed at z = 0.0447 and the same can
measures top = 0.081, so **grasp = top - 0.036**. Release height likewise: the demo
released at 0.1525 with the basket top at 0.142; padded to top + 0.045.

## Version chain

| ver | change | probe | receipt |
|---|---|---|---|
| v1 | no-motion RGB-D dump through `api.log` | 51,55,59,63 | 0/4 (by design); `results/fs_..._v1` |
| v2 | full pick-and-place; z-ceiling declustering; height+footprint gate then max blue; basket = largest component | 51,53,...,65 | **8/8**; `results/fs_..._v2` |
| v3 | + graded candidate tiers (fused / clipped fallbacks), fallback floor raised to top > 0.04 so the flat blue box can never win, and a sensor-verified **regrasp** (effort < 2.5 or width < 0.02 after the lift -> reopen, re-perceive, retry once) | 51,53,...,65 | **8/8**; `results/fs_..._v3` |
| v4 | **envelope probe, not a candidate**: v3 with the grasp aim deliberately displaced +12 mm in x and +12 mm in y | 51,53,...,65 | 8/8; `results/fs_..._v4` |

**v4 is the margin receipt.** A 17 mm diagonal aim error makes the *first* close
air-close (`width 0.0010, effort 0.05`); the run still scores 8/8 because the
regrasp fires, re-perceives to (-0.152, 0.060) and lifts holding
(`width 0.0626, effort 3.0`). So the direct-grasp envelope is under 17 mm diagonal,
and the retry path is exercised and works end to end. In the nominal v3 the aim is
the component bbox mid, which reproduced to within 3 mm across all 15 debug seeds
and never triggered the retry.

Sensor receipts on every nominal episode: closed width 0.0625 m on a 0.064 m can,
effort 3.0 held through the lift and still 3.0 over the basket.

## DECLARATION

- **Frozen version: v3.** `packs/c2k1clean_obj_alphabet_soup_pos_k1/program.py`
  md5 `2b1cde1bf2a677c8fa0d07e2303a7e55` == `program_v3.py` md5
  `2b1cde1bf2a677c8fa0d07e2303a7e55`.
- **Selection receipt: 15/15** on the full debug split (seeds 51-65),
  `results/sel_c2k1clean_obj_alphabet_soup_pos_k1_v3` — every episode
  `"benchmark_success": true`.
- Per-version receipt chain: v1 dump, v2 8/8, v3 8/8, v4 (detuned envelope) 8/8,
  v3 formal 15/15.
- `PROVENANCE` is a top-level literal dict in `program.py` covering every
  calibrated constant; each entry cites either a pack field (`demo0` keyframe
  t=42 / t=128 ee) or a debug-seed measurement from the v1 RGB-D dump.
- Eval seeds 1-50 were never run or read. No forbidden path was opened.
