# c2k1clean / obj_tomato_sauce_task_k1

Intent: **"Pick the bbq sauce and place it in the basket"** (authoritative).
Runner: `tools/fair_run.py` only. Splits sealed: debug = 51-65.

## Evidence from the two packs (pack.json + keyframes only)

| pack | language | what the demo moves |
|---|---|---|
| `..._task_k1` (my scene) | "pick up the tomato sauce and place it in the basket" | a squat can, grasped at ee (0.054,-0.107,0.049), released at ee (-0.040,0.259,0.181) |
| `..._task_mate` (other scene) | "pick up the bbq sauce and place it in the basket" | a short **dark-amber bottle**, grasped at ee (0.033,-0.121,0.080), released at ee (-0.007,0.241,0.174) |

So the object I want is the mate pack's dark-amber bottle, and the k1 pack's can is
explicitly *not* it. The k1 grasp xy (0.054,-0.107) coincides with the debug-seed can
cluster at (0.070,-0.101) — which confirms the k1 pack is shot in my scene and that the
can is the decoy.

## v0 — perception probe (no motion). `fs_..._v0`, seeds 51,53,55,57

Deprojected `cam_high` depth to base frame, table plane from the depth histogram mode
(**table_z = 0.0069**), 4-connected components on the band above it.

Stable cluster table (all four seeds):

| cluster | centre | h | ext | top colour |
|---|---|---|---|---|
| robot arm | (-0.120,0.006) | 0.479 | — | — |
| **basket** | (~0.07,~0.26) | 0.137 | 0.159 x 0.170 | (167,167,167) |
| boxes (1 or 2 clusters) | y ~ -0.21 | 0.136 | 0.053 each | (104,90,44)/(125,96,88) |
| can (= tomato sauce, decoy) | (0.070,-0.101) | 0.074 | 0.064 x 0.070 | (86,76,75) |
| flat prop | (0.157,0.030) | 0.022 | 0.082 x 0.047 | (84,66,60) |
| **bbq sauce** | (~-0.19,~-0.08) | 0.106 | 0.026 x 0.049 | **(66,21,4)** |

Verdict: the bbq sauce is separable with a wide margin on *any* of three cues — top
colour (summed 91 vs >= 210 for everything else), footprint, and height. Rule adopted:
drop the basket (ext > 0.12), keep props with ext <= 0.09 and 0.07 <= h <= 0.13, take
the darkest top. A ceiling of z < 0.20 m removes the arm from the mask entirely.

## v1 — pick and place, release inside the basket. `fs_..._v1`, 8 probe seeds -> **0/8**

Hypothesis: grasp at `top - 0.033` (the mate pack's close height relative to that same
bottle's measured top) and release at `rim + 0.037` (the k1 pack's release height
relative to the measured rim).

Evidence: **the grasp is perfect 8/8** — held=True, width 0.0363, effort 3.0, every
seed. The *release* fails, identically on all 8: the descent over the basket is blocked
**27 mm short** (residual 0.0269 vs ~0.009 for a converged move) and the eef slides
+0.010 in x while pushing. The gif shows why — the payload hangs 0.079 m below the tool
and swings/tilts behind the transport, so on the way down its base catches the basket
rim, stands against it, and topples out. Post-episode re-perception puts the dark prop
at (0.170,0.266), top 0.032: lying on the table just outside the basket's far wall.

## v2 — damp the swing, release clear of the rim. `fs_..._v2`, 8 probe seeds -> **8/8**

Three changes, all from the v1 receipt:
1. measure the payload hang at runtime (`hang = eef_z_at_close - table_z`, 0.074-0.075 m)
   rather than reaching to a fixed depth;
2. `settle(0.4)` after the lift and `settle(0.6)` after a slower (3.0 s) transport, so
   the payload stops swinging before anything descends;
3. release at `rim + hang + 0.012` — the base just clear of the rim, so nothing ever
   touches it — then `settle(1.0)` to let it drop in before retreating.

Receipt: over-basket residual is now 0.009-0.012 on every seed (converged, not blocked),
and `benchmark_success` is true on 51,53,55,57,59,61,63,65.

This also answers the identity question empirically: the environment's own predicate
fires when the **dark-amber bottle** reaches the basket, i.e. the re-authored cell
tracks the intent sentence, not the bddl's filename.

## DECLARATION

- **Frozen version: v2.** `packs/c2k1clean_obj_tomato_sauce_task_k1/program.py`
  md5 `3458b1325931431fca05361e9f6314b1` == `program_v2.py` (same md5, verified on the
  cluster). `program_v0.py` / `program_v1.py` archived alongside it.
- **Selection receipt: 15/15** on the full debug band 51-65 —
  `results/sel_c2k1clean_obj_tomato_sauce_task_k1_v2` (every episode
  `"benchmark_success": true`, 231-246 sim steps).
- **Receipt chain:**
  - v0 (perception probe, no motion) — `fs_..._v0`, seeds 51,53,55,57: cluster table
    above; the bbq sauce is separable by top colour, footprint and height.
  - v1 — `fs_..._v1`, 8 probe seeds: **0/8**. Grasp 8/8, release blocked on the rim 8/8.
  - v2 — `fs_..._v2`, 8 probe seeds: **8/8**; then the formal 15/15 above.
- **PROVENANCE** present in `program.py` as a top-level literal dict; every calibrated
  constant is sourced from a named pack field or a debug-seed measurement.
- No `tools/fewshot_run.py` use; no forbidden reads; writes confined to
  `packs/c2k1clean_obj_tomato_sauce_task_k1/*` and `results/*c2k1clean_obj_tomato_sauce_task_k1*`.

STOP.
