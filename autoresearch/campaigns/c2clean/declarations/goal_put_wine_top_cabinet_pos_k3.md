# c2clean — goal_put_wine_top_cabinet_pos_k3

Intent: "put the wine bottle on top of the cabinet". Runner: `tools/fair_run.py` only.
Debug/learn seeds 51-65. Eval seeds 1-50 never touched. No shared note file.

---

## Pack reading (K=3 demos)

All three demos have the same four-keyframe shape:

| demo | t0 eef | close eef (grip_cmd 1.0) | release eef (grip_cmd -1.0) | len |
|---|---|---|---|---|
| demo0 | (-0.211, -0.010, 1.174) | (-0.182, -0.065, 1.029) | (-0.007, -0.226, 1.252) | 93 |
| demo1 | (-0.201,  0.003, 1.181) | (-0.246, -0.047, 0.992) | (-0.015, -0.200, 1.209) | 130 |
| demo2 | (-0.214,  0.003, 1.167) | (-0.233, -0.061, 1.024) | (-0.025, -0.212, 1.255) | 87 |

Mechanism the pack teaches: top-down close on the bottle at eef z ~0.99-1.03,
lift to ~1.29, carry, release at eef z ~1.21-1.26. The wrist stays essentially
straight down (rx ~3.1) with an inconsistent yaw (rz -0.41 / -0.94 / -0.59), so
yaw is not load-bearing and `rotation=None` is used throughout.

**The pack's absolute xy is a decoy twice over.** This is a `_pos` cell, and the
debug layout is also *mirrored* relative to the pack keyframes: in the pack the
cabinet sits at y~-0.22 (image left), on debug seeds it sits at y~+0.23 (image
right). Only the pack's *relative* quantities transfer, and that is what the
program uses:

- grasp depth below the bottle's top,
- carry altitude (`ee_path6` peak z: 1.2945 / 1.2828 / 1.2966),
- release height above the support.

The last one is the key cross-check. Subtracting each demo's grasp height above
the table from its release z puts the bottle's base at **1.124 / 1.118 / 1.132** —
and the support this cell perceives has its top at **1.1276**. The pack and the
scene agree to within 4-10 mm, which is what licensed the whole placement rule.

---

## v0 — perception probe (no motion)

Ships cam_high + cam_arm_wrist RGB-D home through `api.log` (zlib + base64).
Receipt: frames returned on 15/15 debug seeds; 0/15 success, as expected — it
moves nothing.

- **`api.log` truncates every line at 2000 chars.** The first shipment was
  silently corrupt (chunks cut from 4000 to ~1982 and zlib failed downstream).
  Chunk payloads dropped to 1900.
- Deprojection re-derived and validated against `f.deproject` spot checks to
  <1.5 mm.
- cam_high: f=618.04, c=(256,256); `t_base_cam` constant across seeds; camera at
  (0.659, 0, 1.610) looking down the -x/-z diagonal. Its u axis is *exactly*
  base +y, which makes silhouette-derived y unbiased and silhouette-derived x
  biased by the object's radius (corrected in the program).
- Table plane z = **0.901** (>128k px of 512x512).
- Cabinet top = a ~6.8k-px horizontal plane at **1.1276** (= table + 0.227). The
  band 1.100-1.119 that first looked like the top is the cabinet's *front face*,
  a vertical surface; the fine histogram separates them cleanly.
- Bottle top = **table + 0.153..0.157** in every seed (same asset, `_pos` moves
  it but cannot resize it). Bottle xy spread over the 15 debug seeds:
  x ∈ [-0.100, -0.071], y ∈ [-0.015, +0.012] — a ~±1.5 cm perturbation.

### Dry-testing the detector offline paid for itself

The v0 blobs let me run candidate detectors on all 15 seeds locally, at zero
cluster cost. The first detector I wrote would have failed: it picked the
**wooden rack** (which reaches z 1.09, taller than the bottle's 1.056) in 9 of
15 seeds, and its cabinet centroid was dragged to y≈0.07 by robot-arm pixels
crossing the same height band. Both were fixed before any episode ran.

---

## v1 — perceive, pick, place

*Hypothesis:* the bottle is the tallest compact dark column on the table; the
cabinet is the largest xy-connected blob of the dominant plane above table+0.12;
the pack's relative heights then determine grasp and release z.

- Cabinet: largest connected blob of the plane, so arm pixels cannot move the
  centroid. Target = the blob's 2/98-percentile midpoint, ≈ (-0.40, 0.225).
- Bottle: dark (lum<90) points above table+0.09, xy-clustered, filtered by a
  **two-sided** footprint span (0.015-0.060 m in both axes) — rack edges are
  slivers ≤0.010 m wide, the arm is a mass ≥0.21 m wide.
- Neck band a slice below the cork: y from the silhouette midpoint, x from the
  near surface backed off by the measured radius.

*Evidence:* probe seeds 51,53,...,65 → **8/8**
(`results/fs_c2clean_goal_put_wine_top_cabinet_pos_k3_v1`).
Formal 15 → **15/15** (`results/sel_c2clean_goal_put_wine_top_cabinet_pos_k3_v1`).

Execution is very stable: closed gripper width 0.0149-0.0150 with effort 3.0 on
every seed, held through lift, carry and place. `api.move` converges to a ~10 mm
residual and stops, so the real grasp lands ~10 mm above the commanded z — still
inside the demo band — and the real release ~7 mm high, i.e. the bottle is
dropped ~6 mm onto the cabinet.

*Verdict:* works, but the **margin is one grid cell**. Auditing the rejected
candidates showed rack fragments with x-spans of exactly 0.010 against a
`SPAN_MIN` of 0.015 — and on ep64 one such fragment was *taller* than the bottle
(1.0761 vs 1.0561), so a single 5 mm change in its silhouette would have made
v1 grasp the rack. Not safe for 50 unseen seeds.

---

## v2 — second, independent cue (FROZEN)

*Hypothesis:* luminance separates the bottle from the rack with far more room
than footprint does, so gating on both makes a single-cue failure harmless.

*Evidence:* median pixel luminance per candidate blob, all 15 debug seeds:

| candidate | median luminance |
|---|---|
| wine bottle | **2.0 - 2.3** (all 15 seeds) |
| robot arm mass | 12.0 - 19.7 (rejected 4x over by footprint: span 0.21-0.27) |
| wooden-rack edges | 67.7 - 87.7 |

The gate at 8.0 sits in a gap with ~3x margin below and ~8x above, and the
nearest competitor that could pass the footprint test is 30x away. Added with a
loose height bracket (top must be table+0.10..0.22) and a max-cells selector
instead of max-ztop, plus a fallback to v1's rule if every gate rejects, so a
surprise can never abandon the episode.

Verified offline first: **v2's perception output is bit-identical to v1's on all
15 debug seeds** (max |Δ| = 0 on every field), so this is pure margin, not a
behaviour change.

*Receipts:*
- Formal 15 → **15/15** (`results/sel_c2clean_goal_put_wine_top_cabinet_pos_k3_v2`,
  `grep -c '"benchmark_success": true' results.jsonl` = 15 of 15 lines).
- **Aim-envelope probe:** the same program with a deliberate **+12 mm** offset
  injected into the grasp y (the gripper's closing axis, the most sensitive one)
  still scores **6/6** on seeds 51,53,...,61
  (`results/fs_c2clean_goal_put_wine_top_cabinet_pos_k3_yoff12`). 15/15 alone
  says nothing about margin; this says the grasp absorbs at least 12 mm of aim
  error, against a perception spread of ~2 mm.

*Verdict:* frozen.

---

## DECLARATION

- **Frozen version: v2.** `packs/c2clean_goal_put_wine_top_cabinet_pos_k3/program.py`
  md5 `9cafead37470af2557d44b41b1e822f7` == `program_v2.py` (verified on the
  cluster and locally).
- **Selection receipt (full 15 debug seeds): 15/15** —
  `results/sel_c2clean_goal_put_wine_top_cabinet_pos_k3_v2`
  (results.jsonl: 15 lines, 15 with `"benchmark_success": true`).
- **Per-version receipt chain:**
  | version | run | seeds | score |
  |---|---|---|---|
  | v0 (perception probe, no motion) | `fs_..._v0` | 51-65 | 0/15 (expected) |
  | v1 | `fs_..._v1` | 51,53,…,65 | 8/8 |
  | v1 | `sel_..._v1` | 51-65 | **15/15** |
  | v2 | `sel_..._v2` | 51-65 | **15/15** |
  | v2 + 12 mm grasp-y error (envelope probe) | `fs_..._yoff12` | 51,53,…,61 | 6/6 |
- **PROVENANCE present** in `program.py` as a top-level literal dict covering
  every calibrated constant (`GRASP_BELOW_TOP`, `CARRY_Z`, `RELEASE_CLEAR`,
  `HOVER`, `TABLE_BAND`, `BOTTLE_MIN_HEIGHT`, `BOTTLE_SPAN`, `DARK_LUM`,
  `MED_LUM_MAX`, `H_TOP_BAND`, `CAB_BAND`, `GRID_RES`, `GRIP_OPEN`,
  `GRIP_CLOSE`). Sources are this pack's demo keyframes/`ee_path6` and
  debug-seed (51-65) RGB-D measurements only. No LIBERO prior knowledge, no
  foreign-cell constants.
- **Clean room:** only `packs/c2clean_goal_put_wine_top_cabinet_pos_k3/*` and
  `results/*c2clean_goal_put_wine_top_cabinet_pos_k3*` were written. No .bddl /
  .xml / .hdf5 / init_states / gt_trace read; no other cell's or campaign's
  artifacts read; `tools/fewshot_run.py` never invoked; `api.done` never read.

STOP.
