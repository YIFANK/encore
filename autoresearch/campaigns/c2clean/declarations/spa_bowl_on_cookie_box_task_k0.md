# c2clean / spa_bowl_on_cookie_box_task_k0 — worker notes

Intent: *"Pick the akita black bowl on the top of the cabinet and place it on the plate"*
No demonstration pack. Everything below is measured on debug seeds 51–65 (cam_high
RGB-D, EEF, gripper sensor, my own logs) or is generic controller/camera mechanics.

## Scene, as re-derived from the debug seeds

Perception is a top-down max-z height map (4 mm cells) built by deprojecting cam_high
with the plain OpenCV pinhole convention (verified: the alternative sign convention puts
the whole scene at z 1.85–3.2 m, which is off the table).

| structure | where (seed 51) | height |
|---|---|---|
| table plane | dominant plateau | 0.897–0.900 |
| cabinet top | y < −0.14 slab | 1.124 |
| **cabinet-top bowl (target)** | ctr ≈ (0.03, −0.28), bbox 0.108–0.112 | ztop 1.180 |
| bowl on the cookie box | ctr ≈ (0.07, 0.03) | ztop 0.971 |
| **plate (goal)** | ctr ≈ (0.06, 0.20), bbox 0.136 | ztop 0.920 |
| small table bowl | ctr ≈ (−0.20, 0.21) | ztop 0.944 |

Layout is near-fixed across seeds; the target bowl jitters ±0.03 m in x, so the grasp
must be perceived per seed. Bowl height = 1.180 − 1.124 = 0.056; rim radius 0.056.

**Target ambiguity.** The bddl filename names the *cookie-box* bowl while the intent
sentence names the *cabinet-top* bowl. Settled empirically, not by assumption: v6 ep51
picked the cabinet-top bowl and `benchmark_success` came back **true**. The graded
object is the one the instruction names.

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | dump RGB-D through `api.log` so perception can be done offline | works once chunks are ≤1900 chars (`api.log` truncates a message at 2000) | perception pipeline established |
| v2 | rim-pinch with the grasp offset in **x** | closed width 0.0010 m, effort 0.05 = empty close | refuted; jaws do not separate along x |
| v3 | same, offset in **y**, fingertips assumed 0.091 m below the EEF | still empty (0.0010 / 0.05); "stall" at eef 1.2376 was my detector firing on a no-op command, not contact | refuted; the offset was wrong |
| v4 | dump the wrist camera at the pre-close pose | jaws sit just under the wrist camera — the 0.091 m offset is impossible | diagnostic |
| v5 | measure the offset against surfaces of known height | open jaws descending on the 0.900 m table stop at eef 0.9090 → **fingertip offset 0.009 m**; at the bowl centre the open 0.078 m pair wedges on the inner wall at rim level | calibration; v3 had been closing 49 mm above the rim, in air |
| v6 | straddle the rim wall on the +y arc at the corrected height | **grips**: width 0.0067–0.0081, effort 3.00; ep51 scored. ep53 lost it on a single 2 s lift; a width-only hold test was fooled by a 0.0041 m empty close | 1/2 |
| v7 | effort (not width) as the hold test; bias-corrected aim; stepped lift with a per-step hold check; retry ladder over four rim arcs | probe 8/8, **formal 15/15**, every seed grasping on the first arc. Residue: seeds 63/64 lost the bite during the single 0.20 m hop to the plate and scored on a lucky drop | **15/15** |
| v8 | replace that hop with a 0.03 m stepped descent | **7/8** — repeated re-commands swing the eef laterally and shake the thin rim bite out (lost on 53/61/63) | refuted; carry left alone |
| v9 | keep v7's carry, press the hand 12 mm further onto the rim before closing | seats at eef 1.174 instead of 1.183–1.189; bite thickens to 0.0087–0.0121 m (v7: 0.0065–0.0098). Probe 8/8, **formal 15/15** | **selected** |

## Mechanism, in one paragraph

The hand bottoms out on the bowl lip at eef ≈ rim + 0.009, so no deep grasp of this bowl
is available: the only purchase is a shallow straddle of the rim wall, jaws separating
along world y, gap centre placed one rim radius from the bowl centre on the +y arc (the
outer jaw then falls over the free 1.124 m cabinet top, the inner jaw inside the mouth).
The bite is thin — 9–12 mm of wall — so the hold is re-checked with `effort` at every
stage, the lift is stepped, and a lost bite retries the next arc. Carry height is capped
at 1.285 (v2 receipt: eef 1.3198 over the cabinet only reaches 1.2964). Release height
comes from the *measured* grasp: `hang = eef_at_grasp − (rim − 0.056)`, so the bowl base
lands 5 mm above the plate top.

## DECLARATION

- **Frozen version: v9.** `packs/c2clean_spa_bowl_on_cookie_box_task_k0/program.py`
  md5 `a5b4494fe7bb858e8a074f32fbdd9073` == `program_v9.py` (verified on AbakaAI).
- **Selection receipt: 15/15 on the full 15 debug seeds (51–65)**, directory
  `results/sel_c2clean_spa_bowl_on_cookie_box_task_k0_v9`. All 15 grasped on the first
  arc; max 245 of 1000 sim steps used, no episode errors.
- Per-version receipt chain: table above. Formally probed versions archived as
  `program_v1.py` … `program_v9.py` in the pack directory.
- Runner-up v7: also 15/15 (`results/sel_c2clean_spa_bowl_on_cookie_box_task_k0_v7`);
  v9 preferred on bite margin (min closed gap 0.0087 vs 0.0065 m).
- `PROVENANCE` present in program.py, covering every calibrated constant; all sources
  are debug-seed measurements from this cell or generic controller/camera mechanics.
- No demonstration pack was available or used; no shared note file exists (FAIR_PROTOCOL
  v1.1.1); `api.done` is never read.
