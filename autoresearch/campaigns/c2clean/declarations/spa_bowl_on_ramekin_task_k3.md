# c2clean / spa_bowl_on_ramekin_task_k3 — worker notes

Intent: **"Pick the akita black bowl on the cookie box and place it on the plate"**
Runner: `tools/fair_run.py` only. Debug seeds 51-65.

## Scene (own cam_high RGB-D, seeds 51-65, v1 probe)

Camera `cam_high`: K f=618, principal (256,256); t_base_cam puts the camera at
(0.659, 0, 1.610) with image-right = base +y and image-down = base +x. Table
plane z = 0.901 (modal depth over the workspace).

Props, all 15 debug seeds (per-seed ring fits):

| prop | centre x | centre y | rim/top z | ring r |
|---|---|---|---|---|
| bowl on **cookie box** (red/white checkered support) | 0.058…0.085 | 0.014…0.043 | 0.970 | 0.053 |
| bowl on **ramekin** (grey cylinder support) | −0.207…−0.182 | 0.183…0.212 | 1.000 | 0.056 |
| plate | 0.050…0.072 | 0.183…0.210 | 0.920 | 0.059 |
| cabinet (large dark box) | −0.145…0.18 | −0.36…−0.145 | 1.128 | — |
| stove slab | −0.45…−0.365 | −0.185…−0.095 | 0.960 | — |

Both bowls are the same grey "akita black bowl" asset; **the support is the only
identity cue**. Support-band mean redness R/(R+G+B), 0.045 m below the rim top:
checkered cookie box **0.45**, ramekin **0.33** — clean separation on every seed
checked. Rim-top height (0.970 vs 1.000) is the tie-break.

Target bowl geometry: rim top 0.970, interior floor 0.927, rim ring radius
0.053 (outer edge ~0.058). Outer diameter 0.116 m **exceeds the max jaw opening
0.078 m** → the rim wall must be pinched, not straddled.

## Packs

- `..._task_k3` (K=3, language "…black bowl **on the ramekin**…on the plate"):
  **same bddl scene as mine**. Demos grasp at (−0.19…−0.20, 0.14…0.16, 0.952…0.990)
  = the *ramekin* bowl, i.e. the wrong object for my instruction, offset ≈ −0.05 in
  y from the bowl centre, dx ≈ 0, z ≈ rim_top − 0.01…−0.05. Release over the plate
  at (0.03…0.06, 0.14…0.17, 0.934…0.955).
- `..._task_mate` (K=3, language "…black bowl **on the cookie box**…on the plate"):
  a *different* scene layout, but the same object/support pair. demo0 closes the
  gripper at z = 0.953 = rim_top − 0.017.

Both packs give the same mechanism: jaws separate along base y under the
straight-down wrist, so the eef sits one rim radius along −y from the bowl centre
and descends to just under the rim top.

## Version log

### v1 / v1b — perception-only probe (no motion)
Hypothesis: need to see my own scene; the instruction names a support ("cookie
box") that the pack images show in a *different* scene.
Evidence: 8 + 15 episodes, zero motion, all `benchmark_success: false` (expected).
Gotcha: `api.log` truncates a message at ~2000 chars — the zlib+base64 RGB-D
datapipe needs ≤1800-char chunks (v1 chunks of 3000 were silently clipped and
would not decompress).
Verdict: scene resolved (table above). **My scene contains BOTH bowls**; the one
named by the instruction (on the cookie box) exists and is distinct from the one
the k3 pack demonstrates.

### v2 — perceive → rim-pinch → place (first motion version)
Hypothesis: pick the checkered-support bowl by support redness, pinch the rim at
one radius along −y, carry at z=1.10, release with the bowl base 0.008 above the
plate rim.
Constants: PINCH_R 0.053, GRASP_DZ −0.017, HANG 0.035, CARRY_Z 1.10.
Evidence: probe 51,53,…,65 → **6/8** (`results/fs_…_v2`). The two failures (59, 63)
never touched the plate: `PERC plate c=(-0.214,0.025) ztop=1.353` — the ring fit
took a *rectangular 3D box* around a component, so the robot column leaked into
the fit and a band component won the "largest" contest. Grasp itself was perfect
on all 8 (effort 3.0, closed gap 0.0085 = the rim wall).
Verdict: mechanism right, segmentation wrong.

### v3 — exact component membership in the ring fit
Hypothesis: fit each component from *its own cells* (grid-cell membership test)
instead of a bounding box, and require the fitted top to fall inside the band.
Evidence: probe 8 seeds → **0/8** (`results/fs_…_v3`). The leak was gone, but the
plate band now cleanly resolves the **rectangular white stove base** at
(−0.26,−0.12,0.932) with ~950 cells, which beats the real plate (~360 cells) on
the "largest component" rule every single seed.
Verdict: "largest in band" is not a plate detector.

### v4 — shape test for the plate  ← FROZEN
Hypothesis: a disc is identified by *shape*, not size. Two scale-free tests on
each band component: (i) fill = footprint_area / (π r²) — a filled disc cannot
exceed 1, a rectangle does; (ii) radial residual std of the top-ring points.
Measured on debug seeds: plate r=0.062, fill **0.74**, residual **0.0042**;
stove base fill **1.64**, residual **0.025**. Cuts: r∈[0.045,0.085], fill≤1.10,
residual≤0.008, ztop≤0.930.
Evidence:
- offline replay of `perceive()` on the v1b frames of **all 15 debug seeds**:
  target bowl and plate correct 15/15 (local harness, zero sim steps).
- probe 51,53,…,65 → **8/8** (`results/fs_…_v4`).
- **formal selection, all 15 debug seeds → 15/15** (`results/sel_c2clean_spa_bowl_on_ramekin_task_k3_v4`).
Verdict: accepted, frozen.

### Aim envelope (information only — not used for selection)
Per "a 15/15 says nothing about margin", two displaced copies on seeds 52,54,56,58:
- `program_margin_r061.py`: PINCH_R 0.053 → **0.061** (+8 mm lateral) → **4/4**
- `program_margin_dz030.py`: GRASP_DZ −0.017 → **−0.030** (13 mm deeper) → **4/4**
The rim pinch is not living on a knife edge in either axis; the closing jaws drag
the bowl to centre (closed gap 0.0085 on every episode, effort 3.0 through lift,
carry and place).

## Notes for whoever reads this next
- The instruction's object ("the bowl **on the cookie box**") **does exist** in this
  bddl scene, and it is *not* the bowl the k3 pack demonstrates (that one is on the
  ramekin). Both bowls are the same grey asset; only the support distinguishes them.
  The benchmark bit fires for the instruction's bowl — the re-authored predicate
  follows the instruction, not the bddl filename.
- The final retreat move reports a residual ~0.11 (it is blocked by the released
  bowl). Harmless: the predicate has already fired and the bowl is not dragged.

## DECLARATION

- **Frozen version: v4.** `packs/c2clean_spa_bowl_on_ramekin_task_k3/program.py`
  md5 `8eefbec2880fd9d24a631418b7f72604` == `program_v4.py` (verified on cluster).
- **Selection receipt: 15/15 on the full debug split (seeds 51-65)**,
  dir `results/sel_c2clean_spa_bowl_on_ramekin_task_k3_v4`.
- **Receipt chain:**
  | version | run dir | seeds | result |
  |---|---|---|---|
  | v1 / v1b | `fs_…_v1`, `fs_…_v1b` | 8 / 15 | perception probe, no motion (0 by construction) |
  | v2 | `fs_…_v2` | 51,53,…,65 | 6/8 |
  | v3 | `fs_…_v3` | 51,53,…,65 | 0/8 |
  | v4 | `fs_…_v4` | 51,53,…,65 | 8/8 |
  | **v4** | **`sel_…_v4`** | **51-65 (all 15)** | **15/15** |
  | v4 ±aim | `fs_…_marginR`, `fs_…_marginZ` | 52,54,56,58 | 4/4, 4/4 (envelope only) |
- **PROVENANCE**: present as a top-level literal dict in `program.py`, covering
  every calibrated constant (TABLE_Z, GRID, BOWL_BAND, PLATE_BAND, PLATE_SHAPE,
  FURNITURE_Z, RED_BAND_DZ, PINCH_R, GRASP_DZ, HANG, CARRY_Z, PLACE_CLEAR). Sources
  are the two named packs and own debug-seed (51-65) RGB-D / gripper observations.
- Clean room respected: writes confined to `packs/c2clean_spa_bowl_on_ramekin_task_k3/*`
  and `results/*c2clean_spa_bowl_on_ramekin_task_k3*`; `fewshot_run.py` never invoked;
  no `api.done` read; eval split never touched.
