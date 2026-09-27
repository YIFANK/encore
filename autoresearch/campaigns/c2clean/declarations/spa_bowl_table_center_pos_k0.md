# c2clean / spa_bowl_table_center_pos_k0 — worker notes

Intent: *pick up the black bowl from table center and place it on the plate*.
Zero demonstrations (the pack directory is empty). Every constant below is
re-derived from debug seeds 51–65 and declared in `PROVENANCE`.

## Scene, as measured (v1b/v2, all 15 debug seeds)

Table plane z = 0.9013 (modal bin of the cam_high height map; the runtime
`table_z()` returns 0.9013 on every debug seed). Camera at (0.659, 0, 1.610)
looking down the −x direction; image right = +y, image down = +x.

Above-table objects, stable across all 15 seeds to ±0.02 m:

| object | centre (x, y) | top above table | footprint | mean rgb |
|---|---|---|---|---|
| robot base / arm | −0.60, 0.00 | 0.107 / 0.47 | — | olive |
| stove knob (black) | −0.40, −0.13 | 0.059 | 0.09 × 0.09 | 20 |
| stove slab | −0.34, −0.12 | 0.030 | 0.19 × 0.16 | 65 |
| bowl C | −0.21, +0.19 | 0.042 | 0.085 × 0.09 | 140 |
| **bowl D (target)** | **−0.08, 0.00** | **0.051** | **0.11 × 0.11** | 108 |
| bowl E | +0.06, +0.20 | 0.051 | 0.11 × 0.11 | 118 |
| **plate** | **0.00, +0.33** | **0.019** | **0.135 × 0.135** | 150 |
| cookie box | +0.08, +0.03 | 0.019 | 0.08 × 0.06 | 95,66,46 |
| cabinet | +0.02, −0.25 | 0.227 | 0.31 × 0.21 | 84 |

Target bowl cross-section (5 mm grid slices through the rim centre): outer rim
radius 0.055–0.060, rim top 0.051, interior floor 0.007, interior surface
reaching 0.025 at radius 0.036. **The bowl is 0.114 across and the jaws open
0.079, so it cannot be straddled** — the grasp has to be a rim pinch.

"table center": the midpoint of the 1st–99th percentile box of the observed
table plane is (−0.1175, ≈0.00). Bowl D sits 0.039 from it, bowls C and E sit
0.21 and 0.27 away. Unanimous and by a 5× margin on all 15 debug seeds.

## Version chain

| ver | change | probe | receipt |
|---|---|---|---|
| v1 | perception dump | — | api.log truncates a message at 2000 chars; chunks were silently clipped |
| v1b | 1800-char chunks | 15 seeds | full RGB-D recovered; deprojection convention pinned to `deproject(u,v)` / array `[v,u]` |
| v2 | measurement episode | 4 seeds | tips sit 0.0074 below the eef (closed gripper stalls at eef_z 0.9087 on a 0.9013 table); free-space downward tracking lag a flat 0.0112; wrist-depth finger blobs separate along world **y** |
| v3 | first rim pinch, PINCH_R 0.043, tips 0.025 | 4 seeds | 0/4. Closes to 0.0078 at effort 3.0 and *does* carry the bowl, but it hangs ~0.05 m off the gripper and landed 0.049 short of the plate, tipped |
| v4 | measure the held bowl from cam_high after the lift and over the plate | 8 seeds | 0/8 but the bowl reached the plate every time, off-centre by a very repeatable (−0.033, +0.023), std (0.004, 0.009). Seed 59 lost to plate detection: the plate merged with the bowl beside it into one blob |
| v5 / v5b | band-mask the plate before clustering; add the measured landing bias back; ± a contact-seated descent | 8 seeds | **5/8 both**. Same three seeds (53/55/63) lost either way, so seating is neutral. Logs showed `goto` reading the eef mid-travel, exiting with an over-corrected command standing, and the following settle carrying the arm up to 0.012 past the aim |
| v6 / v6b | settle 0.25 s before reading the eef; pin the descent to the command `goto` converged on | 8 seeds | v6 (seated) 7/8, **v6b (no seating) 8/8** — seating hurts, dropped |
| v6b | — | **full 15** | **14/15** (`sel_..._v6b`). Only seed 62 lost |
| v7 / v7b | gate the hang measurement against its debug median; perception fallbacks; v7b releases 0.010 lower | 8 seeds incl. 62 | **8/8 both** |
| v7, v7b | — | **full 15** | **15/15 each** (`sel_..._v7`, `sel_..._v7b`) |
| v8 | v7 with the dead seating branch deleted and PROVENANCE completed | **full 15** | **15/15** (`sel_..._v8`) — **FROZEN** |

## Why each fix was load-bearing

**The pinch.** Jaws open 0.079, bowl is 0.114 wide. The jaws separate along
world y (v2 wrist-depth receipt: the near-field blobs sit at columns 0–79 and
176–255 of the wrist image, and the wrist camera's x axis maps to −y in base
frame). So the gripper parks at (bowl_x, bowl_y + 0.043) and closes across the
rim wall. It bites 0.0078 and holds.

**The hang, not the pinch radius, sets the placement.** A rim-pinched bowl does
not hang under the gripper; it hangs ~0.06 m along −y with its rim above the
fingertips. Predicting that from PINCH_R put the bowl 0.049 short. Measuring it
from cam_high after the lift is repeatable to ±0.003 (median (0.0101, −0.0606)
over 15 seeds), but the reading is occluded on the gripper side, so it is
~0.02 too long — hence the fixed `PLACE_BIAS` added back on top.

**The overshoot was the difference between 5/8 and 8/8.** `goto` returned while
the OSC was still travelling, so its convergence test passed on a transient and
left `want + err` standing as the command; the next `settle` then carried the
arm up to 0.012 m past the aim. Since the placement basin is only about ±0.012
wide (deliberate ±0.015 aim displacements on seeds 51–58 score 3–5 of 8 against
8/8 at the nominal aim), that drift alone accounted for the losses.

**Seating by contact does not help.** Descending until the eef stops tracking
scored 7/8 against 8/8 for a computed-height release: on seed 53 the descent
never detected contact, drove the bowl into the plate and the grip width grew
from 0.0079 to 0.0102 as the bowl was pushed up into the jaws. The predicate
fires while the bowl is still gripped and resting on the plate, so pressing
adds only risk.

**Band-mask before clustering.** Grouping above-table cells first fuses the
plate with a bowl that touches it (seed 59, and seed 64 in the offline data).
Cutting the 0.012–0.030 height band *first* and clustering that leaves exactly
one blob wider than 0.11 with mean rgb > 120 on all 15 seeds — the plate.

**Gate the measurement against its own median.** Seed 62's lift blob came back
n=83 / top=0.176 against n=53–67 / top=0.186–0.189 everywhere else, throwing the
hang offset 0.019 off and the aim with it. Rejecting a reading further than
0.012 from the debug median recovers seed 62 and costs nothing elsewhere.

## Aim envelope (seeds 51–58, v6b)

| aim shift | score |
|---|---|
| nominal | 8/8 |
| x − 0.015 | 5/8 |
| x + 0.015 | 3/8 |
| y − 0.015 | 3/8 |
| y + 0.015 | 3/8 |

The basin is roughly ±0.012 and the nominal aim sits inside it; the gated hang
measurement reproduces to ±0.003.

v7 and v7b both also scored 15/15 formally (`sel_..._v7`, `sel_..._v7b`). v7 was
preferred over v7b (which releases 0.010 lower) because catching the plate rim
with an off-centre base is the worse failure mode on unseen layouts, and v7's
release height is the one the aim-envelope sweep was measured at. v8 is v7 with
dead code removed; its executed path is identical.

## DECLARATION

- **Frozen version:** `program_v8.py`, copied to
  `packs/c2clean_spa_bowl_table_center_pos_k0/program.py`.
  md5 `f184ff551b3fd7e182a782d8ff1fbbb1` — identical for both files, verified on
  the cluster and locally.
- **Selection receipt:** **15/15** on the full 15 debug seeds (51–65), dir
  `results/sel_c2clean_spa_bowl_table_center_pos_k0_v8`. Every episode
  `"benchmark_success": true`.
- **Per-version receipt chain:** v1 (dump, log truncation found) → v1b (15-seed
  RGB-D) → v2 (4 seeds, tip offset / tracking lag / jaw axis) → v3 0/4 → v4 0/8
  → v5 5/8, v5b 5/8 → v6 7/8, v6b 8/8 → v6b **14/15** formal → v7 8/8, v7b 8/8
  (incl. seed 62) → v7 **15/15** formal, v7b **15/15** formal → v8 **15/15**
  formal, frozen.
- **PROVENANCE:** present as a top-level literal dict in the frozen program,
  covering all 19 module-level calibrated constants (`WS_BOX`, `CELL`,
  `ABOVE_TABLE`, `ARM_CUT`, `TIP_DZ`, `BOWL_R`, `PINCH_R`, `GRASP_H`, `SAFE_H`,
  `LIFT_H`, `CHUNK`, `HANG_RADIUS`, `BOWL_CLASS`, `PLATE_CLASS`, `BOWL_H`,
  `HANG_BAND`, `PLACE_GAP`, `PLACE_BIAS`, `HANG_MED`, `HANG_GATE`,
  `PLATE_BAND`, `GOTO_SETTLE`). Every source is a debug-seed 51–65 measurement
  or generic controller/camera mechanics. No pack was supplied and none was
  read; no benchmark asset was opened; `api.done` is never referenced
  (AST-checked).
- **Clean room:** all cluster writes confined to
  `packs/c2clean_spa_bowl_table_center_pos_k0/*` and
  `results/*c2clean_spa_bowl_table_center_pos_k0*`. Only `--split debug`
  (seeds 51–65) was ever run; seeds 1–50 were never touched.
