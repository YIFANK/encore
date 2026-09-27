# c2clean / spa_bowl_cookie_box_task_k0 — NOTES

Intent: "Pick the akita black bowl on the stove and place it on the plate".
Zero demos. All constants re-derived from debug seeds 51-65 only.

## Scene (measured, debug seeds 51,53,55,57)

Top-down height map from `cam_high` (512x512 RGB-D, deprojected with the given
intrinsics + `t_base_cam`; standard pinhole, depth is z-depth along the camera
axis — verified against the API's own `deproject()` samples to 0.05 mm).

| thing | where | top z | size |
|---|---|---|---|
| table | — | 0.9025 | — |
| **stove bowl (target)** | x -0.25..-0.28, y -0.13..-0.17 | **0.9795-0.9800** (table+0.077) | 0.105-0.110 across |
| stove slab | x -0.45..-0.16, y -0.25..-0.04 | 0.926-0.930 | — |
| table bowl (akita, twin) | x ~0.13, y ~-0.07 | 0.952 (table+0.050) | 0.108 x 0.110 |
| small bowl | x ~-0.19, y ~0.20 | 0.944 | 0.086 |
| **plate (goal)** | x ~0.06, y ~0.20 | 0.9201 (table+0.018) | 0.135 x 0.135, mean rgb 148 |
| cookie box | x ~0.07, y ~0.04 | 0.9205 | 0.081 x 0.060 |
| cabinet | x -0.13..0.18, y -0.35..-0.14 | 1.128 | huge |

Three bowls are present. The intent names the one **on the stove**; the bddl
filename names a different one ("next to the cookie box"). The stove raises the
target 26 mm above its twin, and that is the cue used: the target is the only
object whose top surface lands in the **table+0.055 .. table+0.115** band.
The cabinet (top 1.128) and the robot (fingers at 1.20) top out far above the
band, so they never enter it; the stove knobs do (0.9605) but are <=45 cells.

Bowl outer diameter 0.109 m > the 0.078 m jaw opening, so the bowl cannot be
straddled; it is taken by a **rim pinch on the +y arc** (the straight-down
wrist's closing axis is world y, so at the ring's +y extreme the jaws cross the
wall with one finger inside and one outside).

## Version chain

### v1 — perception dump (2 seeds) — VOID, mechanism bug
Hypothesis: stream RGB-D out through `api.log` (zlib+base64) and do all
perception offline. Evidence: `api.log` **silently truncates a message at
~2000 chars**, so 3000-char chunks arrived cut and zlib refused them.
Verdict: method sound, chunk size wrong.

### v2 — perception dump, CHUNK=1500 (seeds 51,53,55,57) — 0/4 (no motion)
Evidence: full RGB-D recovered for 4 seeds; the table is at z=0.9025, the
deprojection convention is standard pinhole, and the scene table above was
measured. Verdict: perception baseline established.

### v3 — calibration + first grasp (seeds 51,57) — 0/2
Hypothesis: label above-table pixels in IMAGE space, take the highest small
component as the target. Evidence: it selected **the robot's own fingers**
(blue, z=1.1997) and the arm *bridged* the cabinet to the stove bowl into one
482x276 mm component — image-space connectivity fuses whatever occludes.
Both seeds produced byte-identical logs (a scene-independent answer is itself
the receipt). Verdict: perception must run on a top-down XY grid, not in the
image. Also: the descent never stalled (10 mm steps, free-space tracking bias
+0.0103 m) so the tip offset was not measured.

### v4 — XY-grid perception + rim pinch + place (seeds 51,53,55,57) — **4/4**
Changes: top-down 5 mm height map; target = max-ztop component inside the
elevated band with >=60 cells and a 0.06-0.16 m footprint, arcs merged by a
3-cell dilation (the oblique camera splits the rim ring into two arcs);
plate = the pale 0.10-0.18 m flat disc; z commands pre-compensated by Z_BIAS.
Evidence: 4/4 benchmark_success. Fingertip offset measured by blocked descent
= 0.0062 m on every seed. Grip closed to w=0.0115 at effort 3.00.
Verdict: mechanism works, but two receipts show thin margins —
 * after the lift the gripper reports `w=0.0047 ef=0.05` ("not holding")
   while re-perception finds the bowl 45 mm below the eef and moving with it:
   **the effort flag is a gap threshold, and a rim wall is too thin to trip
   it.** Re-perception, not effort, is the holding receipt here.
 * the bowl trails the eef by **0.041 m, not the 0.055 m rim radius** — the
   closing jaws drag it ~14 mm inward. Aiming the eef at `plate + r` therefore
   lands the bowl 14 mm off-centre and it overhangs the plate rim (visible in
   the final frame). Succeeded anyway, but with ~0 mm of margin.

### v5 — measured carry offset + pinch retry (seeds 51..65 odd)
Changes: (a) `measure_held()` re-perceives the payload in a band that stops
0.015 m below the eef (excluding the fingers) and returns both the hang and
the true eef->bowl xy offset; the carry aims so the *bowl*, not the eef,
lands on the plate centre; (b) if nothing hangs under the eef after the lift,
re-perceive and retry the pinch once; (c) dropped the wasteful `predesc`
correction move.
Evidence: **7/8** on seeds 51..65 odd (`fs_..._v5`). Seed 55 failed, and the
log shows it was a false negative in *my own* check, not a grasp failure:
`CLOSED1 w=0.0149 ef=3.00` / `LIFTED1 w=0.0139 ef=3.00` — a *firmer* bite than
any success — yet `measure_held` returned `HELD_NONE n=15`, so the program
retried, dropped the bowl, retried again and gave up. The window stopped at
`eef_z - 0.015`; a firm bite holds the bowl HIGH, so the payload sat almost
entirely above that cut, while a thin bite lets the bowl slide 22 mm down into
easy view. The check rejected exactly the better grasp.
Also: the "clean" offset of -0.051 is itself biased. It is the bbox midpoint of
a mask clipped at the +y side, so it reads 0.090 m of a 0.109 m bowl. v4's
landing error independently pins the true offset at -0.041.
Verdict: hold-detection and offset both wrong; the placement was no better
than v4's.

### v6 — two hold receipts + nominal carry offset (seeds 51..65 odd) — **8/8**
Changes: `measure_held` window widened to `eef_z + 0.02` and gated on x extent
(the bowl is 0.109 m across, the fingers only ~0.03, so x extent separates
them); retry now fires only when **both** receipts fail (`effort < 3.0` AND no
payload seen), because each receipt is blind in a different regime; carry uses
the nominal OFF_Y = -0.041 with the lift estimate logged as a cross-check.
Evidence: 8/8. Seed 55 fixed. Grasping is cleanly **bimodal**:
 * thin bite (51,53,61,63,65): `w->0.0046 ef=0.05`, bowl slides down, hang 0.045
 * firm bite (55,57,59): `w~0.0135 ef=3.00`, bowl held high, hang reads 0.023
Verdict: works, but the firm-bite seeds expose the next defect — with
`rel_z = plate + 0.015 + hang` and an under-read hang, the release drove the
bowl into the plate until the arm stalled (MV13 produced zero motion), and the
saturating command dragged placement up to **19 mm** off the plate centre
(ep59: bowl x 0.0863 vs plate 0.0675). Bowl r 0.0545 + plate r 0.0675 leaves
only 13 mm of budget, so that is a near miss that happened to score.

### v7 — hang floor + more release clearance (seeds 51..65 odd) — **8/8**
The firm-bite hang of 0.023 is an artefact: the gripper body occludes the
bowl's base from cam_high. The v6 **stall heights minus the plate top** give
the true hang on those same seeds — 0.045, 0.046, 0.049 — i.e. the hang is
~0.046 in *both* grip regimes. So `hang = max(measured, 0.045)` and the drop
clearance goes 0.015 -> 0.020.
Evidence: 8/8, and placement error against the plate centre collapses from
<=19 mm to:

| seed | 51 | 53 | 55 | 57 | 59 | 61 | 63 | 65 |
|---|---|---|---|---|---|---|---|---|
| err x (mm) | +1.7 | +0.9 | -12.7 | +1.8 | -0.3 | +2.4 | +1.5 | +0.3 |
| err y (mm) | +0.5 | +0.2 | -5.1 | +0.6 | -0.2 | +1.0 | +0.5 | +0.1 |

7 of 8 seeds land within 2.4 mm of the plate centre against a 13 mm budget.
Seed 55's -12.7 mm is a *frozen* reading: MV13 and MV14 report an eef identical
to MV12's, the signature of the episode ending mid-move because the predicate
had already fired.
Verdict: **selected.**

## DECLARATION

* **Frozen version: v7.** `packs/c2clean_spa_bowl_cookie_box_task_k0/program.py`
  md5 `14d8a867bab63f99245369e1d1ce134d` == `program_v7.py` (same md5).
* **Selection receipt: 15/15** on the full 15 debug seeds 51-65 —
  `results/sel_c2clean_spa_bowl_cookie_box_task_k0_v7` (every episode
  `benchmark_success: true`).
* **Receipt chain:** v1 void (log truncation) -> v2 0/4 no motion (perception
  baseline) -> v3 0/2 (image-space perception selected the robot's own
  fingers) -> v4 **4/4** -> v5 **7/8** -> v6 **8/8** -> v7 **8/8** probe,
  **15/15** formal.
* **PROVENANCE:** present as a top-level literal dict in program.py, covering
  R_DOWN, the grid, BOWL_BAND, BOWL_MIN_CELLS/BOWL_FOOT, the plate band /
  footprint / paleness, Z_BIAS, FREE_SPOT, GRASP_DEPTH, BOWL_R, OFF_Y,
  HANG_NOM/HANG_MIN and DROP. Every constant is sourced to a debug-seed
  (51-65) measurement or to generic controller/camera mechanics. No pack was
  supplied and none was used.
* No `api.done` read anywhere in the program (verified by grep).

## What the sensors actually say (for the record)

1. **`effort 3.0` is a jaw-gap threshold, not a force.** A thin rim bite reads
   `w=0.0046 ef=0.05` while the bowl is demonstrably held (re-perception finds
   it 45 mm below the eef, travelling with the arm). Treating `effort` as *the*
   hold signal would abort every good thin-bite grasp; treating re-perception
   as *the* hold signal aborts every good firm-bite grasp. The program needs
   both, OR-ed.
2. **The closing jaws drag the bowl ~13 mm inward.** The carry must aim so the
   *bowl* lands on the plate, not the eef, and the offset is 0.041 — not the
   0.0545 rim radius.
3. **Occlusion under-reads the hang.** Whenever the payload is held high the
   gripper body hides its base; the release height must be floored.
4. **A frozen eef across consecutive moves means the episode ended.** LIBERO
   terminates on the predicate, so repeated identical eef readings after a
   place are a (post-hoc, log-only) sign the place already scored.
