# c2clean spa_bowl_next_to_ramekin_task_k0 — worker notes

Intent: "Pick the akita black bowl next to the cookie box and place it on the plate"
Zero demonstrations. Every constant re-derived from debug seeds 51-65.

## Scene (measured, seeds 51,53,55,57,59,61,63,65 via cam_high RGB-D)

`cam_high`: K f=618.04, c=(256,256); t_base_cam origin (0.659, 0, 1.610) looking
down toward -x. Image +u -> base +y, image +v -> base +x. The camera sits at
y=0, so y measurements are symmetric; x is seen obliquely.

| thing | centre (x,y) | footprint | ztop |
|---|---|---|---|
| table | — | — | 0.900-0.905 |
| bowl A (by the ramekin) | (-0.18, +0.32) | 0.12 x 0.12 | 0.952 |
| ramekin | (-0.20, +0.19) | 0.10 x 0.10 | 0.944 |
| **bowl B (by the cookie box)** | (+0.13, -0.07) | 0.12 x 0.12 | 0.952-0.955 |
| cookie box | (+0.07, +0.03) | 0.09 x 0.07 | 0.920, rgb (105,78,61) |
| plate | (+0.06, +0.21) | 0.15 x 0.15 | 0.920, rgb (166,157,153) |
| cabinet + stove slab | (+0.10, -0.22) | 0.22 x 0.21 | 1.098-1.127 |
| robot base | (-0.55, 0.00) | — | — |

Layout is near-fixed (a `_task` cell); per-seed jitter is under ~2.5 cm and every
number the program uses is perceived per-episode anyway.

**Name/intent mismatch.** The bddl is `..._next_to_the_ramekin_...` but the
intent sentence says *next to the cookie box*, and those are two different bowls
here. The brief makes the intent sentence the task, so the program targets bowl
B. The benchmark bit credits it: 15/15.

## Detectors

- **bowls**: 1 cm top-down max-z grid, band `0.948 < z < 1.02`, footprint
  0.09-0.16 m on both axes. The 0.948 cut sits in the 8 mm gap between the
  ramekin top (0.944) and the bowl tops (0.952); 1.02 drops the cabinet. Yields
  exactly the two bowls on 8/8 probe seeds (the only other survivor of the band
  is a 0.08 x 0.03 sliver of the stove edge, killed by the footprint window).
- **cookie box**: band 0.9085-0.9235, R-B > 25 (measured 44 vs the plate's 13).
- **plate**: same band, R-B <= 25, >= 60 cells (measured 141-147).
- **target** = the bowl nearer the cookie box: 0.11-0.13 m vs 0.38 m.

## Mechanism

The bowl is 0.115 m across and the jaws open to 0.080 m, so it cannot be
straddled. It is **rim-pinched**: park the eef one rim radius (minus half the
wall) to +y of the bowl centre, lower the open jaws 20 mm past the rim top so
one finger hangs inside the bowl and one outside the wall, close on the wall,
carry above the plate, and lower until the bowl's own base stalls the descent.

The +y arc is forced: the cabinet (z > 1.05) reaches y = -0.18 at the target
bowl's x, only ~45 mm outside its -y rim. On the +y side the only neighbour is
the cookie box, which is 0.920 tall (below the 0.932 fingertip height) and ends
17 mm short of the bowl's x.

## Version log

### v1, v2 — perception probes (no motion)
Dump cam_high RGB-D out through `api.log` (zlib+base64) and do the perception
offline. v1 lost data: **`api.log` truncates each message at 2000 chars**; v2
chunks at 1900 and is complete. Receipt: 8/8 seeds decoded, table above.
Captures cost no sim steps.

### v3 — first manipulation attempt. 0/4, 1000 steps (horizon exhausted)
Hypothesis: rim pinch with an explicit straight-down `rotation` matrix, and an
in-episode fingertip calibration. Evidence: every move burned its entire step
cap without converging. Verdict: **never pass `rotation` to `api.move` here.**
`move_pose` requires `‖axisangle(R·Rᵀ)/0.30‖ < 0.06`, i.e. the wrist within 1.0°,
but the reset wrist is already 3.25° off vertical, so the rotation term never
converges and each call spends `max(60, 60·seconds)` steps. `rotation=None`
takes the `move_cartesian` path — position only, tolerance 0.012 — and converges.

### v4p — calibration probe. Misread
Position-only moves. Got `TIP_OFF = 0.1138` from a descent that stalled at
eef_z = 1.019 over what I took for bare table. The descent had in fact been
crawling (0.15 mm/step against the free-air 1.2 mm/step) for the whole probe
because the hover height I had chosen was already dragging the hardware through
the scene. The number was wrong by 110 mm and poisoned v5-v7p.

### v5 — full task on the wrong vertical frame. 0/4, 191 steps
All moves converged cleanly (res ~0.010 = POS_TOL) and the xy aim was right —
checked by projecting the commanded eef into cam_high and matching it against
the film strip. But `G_closed` reported width 0.0018, i.e. **empty jaws**.

### v6p — depth ladder on the +y arc. Gap 0.0018 at all six heights
Closed the jaws at eef_z from 1.072 down to 1.022 and the gap never moved off
"fully shut" — even at the level where, on the v4p frame, the fingertips should
have been 62 mm *below* the table. Falsified the v4p offset.

### v7p — is the jaw axis base-x? No. 0/2, 1000 steps
Ladder on the -x arc. Gap still 0.0018, and x never converged (stuck 21 mm off).
Settled the axis question **for free** instead: in the seed-51 reset frame the
blue gripper points in the cam_high cloud form **two lobes at y=-0.045 and
y=+0.047 over a single x lobe** — the fingers open along base **y**, as v5
assumed. Zero sim steps, no episode needed.

### v8p — touch probe. The measurement that unlocked the cell
Drove a **closed** gripper straight down onto bare table. It stalls at
eef_z = **0.9090** on both seeds 51 and 53, over a 0.905 table:

> **TIP_OFF = 0.004** — the reported eef *is* the fingertip.

So v5/v6p/v7p had been closing the jaws ~100 mm above the rim the whole time.

### v9 — same plan, correct vertical frame. 4/4, then 15/15
`G_closed` width 0.0168 at effort 3.0, carried, placed, bit true.
Receipts: `fs_..._v9` 4/4 (51,53,55,57); `sel_..._v9` **15/15**.

### v9d012 / v9d028 — aim-envelope probes. 4/4 each
`GRASP_DEPTH` moved to 0.012 and 0.028 (±8 mm around the frozen 0.020), seeds
51,55,59,63. Both 4/4, every close at effort 3.0, gaps 0.010-0.018. The grasp is
not sitting on a cliff — 15/15 is a band, not a coincidence.

### v10 — FROZEN. 15/15
Behaviourally identical to v9; v9 had accumulated a stale duplicate constants
block (including the wrong `TIP_OFF = 0.114`, shadowed at runtime but still
described in its PROVENANCE). v10 is a clean rewrite with one constants block
and a PROVENANCE entry that matches what the program actually does.
Receipt: `sel_c2clean_spa_bowl_next_to_ramekin_task_k0_v10` **15/15**.

## Things that cost time, for whoever reads this next

1. `api.log` truncates at 2000 chars — chunk below it.
2. `api.move(rotation=...)` cannot converge from the reset wrist; use
   `rotation=None`.
3. `move_cartesian` breaks as soon as the **3-D** error is under POS_TOL=0.012,
   so a pure descent stops up to 12 mm high and a sub-12 mm step is a silent
   no-op. Bias the command 12 mm past the target.
4. Saturated free-air travel is ~1.2 mm/step, so the 1000-step horizon is really
   a ~1.2 m path budget. The whole task costs ~110 steps once it works.
5. A stalled descent is only evidence of contact if you know where the fingertips
   are. Calibrate with a **closed** gripper onto a surface of known height before
   trusting any stall.

---

# DECLARATION

- **Frozen version**: `program_v10.py`, copied to
  `packs/c2clean_spa_bowl_next_to_ramekin_task_k0/program.py`.
  `md5 == 05a3b58714aaaf0a5ab96ce8dff7b4de` for both files (verified on cluster).
- **Selection receipt**: one formal run on the full 15 debug seeds,
  `results/sel_c2clean_spa_bowl_next_to_ramekin_task_k0_v10` — **15/15**
  `benchmark_success: true` (51,52,53,54,55,56,57,58,59,60,61,62,63,64,65).
- **Per-version receipt chain**:

  | version | run dir | seeds | result |
  |---|---|---|---|
  | v1 | `fs_..._v1` | 51,53,55,57 | perception probe, log truncated |
  | v2 | `fs_..._v2` | 51..65 odd (8) | perception probe, 8/8 decoded |
  | v3 | `fs_..._v3` | 51,53,55,57 | 0/4, horizon exhausted |
  | v4p | `fs_..._v4p` | 51,53 | calibration probe (offset misread) |
  | v5 | `fs_..._v5` | 51,53,55,57 | 0/4, empty jaws |
  | v6p | `fs_..._v6p` | 51,53 | depth ladder, gap 0.0018 at all 6 levels |
  | v7p | `fs_..._v7p` | 51,53 | 0/2, jaw axis settled from the reset frame |
  | v8p | `fs_..._v8p` | 51,53 | TIP_OFF = 0.004 (both seeds) |
  | v9 | `fs_..._v9` | 51,53,55,57 | **4/4** |
  | v9 | `sel_..._v9` | 51-65 (15) | **15/15** |
  | v9d012 | `fs_..._v9d012` | 51,55,59,63 | 4/4 (envelope, depth 0.012) |
  | v9d028 | `fs_..._v9d028` | 51,55,59,63 | 4/4 (envelope, depth 0.028) |
  | **v10** | `sel_..._v10` | 51-65 (15) | **15/15 — FROZEN** |

- **PROVENANCE**: present in `program.py` as a top-level literal dict covering
  every calibrated constant (TABLE_Z, RES, XLIM/YLIM, BOWL_ZLO, BOWL_ZHI,
  BOWL_WMIN/BOWL_WMAX, FLAT_LO/FLAT_HI, BROWN_CUT, PLATE_MIN_N, WALL_HALF,
  TIP_OFF, GRASP_DEPTH, Z_CARRY, Z_PRESS, POS_TOL_BIAS, JAW_AXIS, PINCH_SIDE,
  MOVE_BUDGET). Sources are debug-seed measurements and generic
  controller/camera mechanics only. No demonstration pack was issued to this
  cell and none was read.
- **Clean room**: writes confined to
  `packs/c2clean_spa_bowl_next_to_ramekin_task_k0/*` and
  `results/*c2clean_spa_bowl_next_to_ramekin_task_k0*`. No .bddl/.xml/.urdf/
  .hdf5/init_states/gt_trace was opened, no other pack's program or NOTES, no
  other campaign's artifacts, no `tools/probe_*.py`. `api.done` is never read.
  Eval seeds 1-50 were never touched.

STOP.
