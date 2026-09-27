# c2k1clean / obj_ketchup_task_k1 — NOTES

Intent: **"Pick the milk and place it in the basket"**
Runner: `tools/fair_run.py` only. Splits sealed: debug = 51-65, eval = 1-50 (never touched).

## Evidence read

- `packs/c2k1clean_obj_ketchup_task_k1/` — pack.json + 7 keyframes. Language
  `pick up the ketchup and place it in the basket`. Grasps at ee ≈ (-0.14, -0.22, 0.013)
  after one failed close at z≈0.066; transports at z≈0.29; releases at (0.092, 0.223, 0.177).
- `packs/c2k1clean_obj_ketchup_task_mate/` — pack.json + 4 keyframes. Language
  `pick up the milk and place it in the basket`, in a **different scene** (different
  prop set entirely). Closes on the carton at ee z = **0.1001**; releases at (-0.006, 0.245, 0.142).
  The label naming the same target as my intent was treated as untrusted; the pack was
  used only as motion/appearance evidence. Its grasp height is the one constant taken
  from it (`Z_GRASP`).
- Keyframe crop comparison: the mate pack's grasped prop and the upright carton in the
  k1 scene are the same asset (red upper band, dark/green lower band).

## Scene, as measured from my own debug-seed observations (v1 dump, seeds 51-65)

cam_high is 512×512 with dense valid depth. Table plane sits at base-frame **z ≈ 0.001**.
Deprojecting the full cloud and clustering everything above z = 0.012 gives, every seed:

| cluster | px | x | y | ztop | top-band span | upper RGB |
|---|---|---|---|---|---|---|
| basket | ~11.5k | +0.07 | +0.25 | 0.142 | 0.151×0.171 | [153,153,152] |
| arm | 9903 | -0.121 | +0.003 | 0.447 | — | [16,64,17] |
| can | 2225 | +0.121 | -0.199 | 0.081 | 0.061×0.063 | [63,68,80] |
| ketchup bottle | 1968 | -0.103 | -0.236 | 0.148 | 0.020×0.035 | [92,80,75] |
| **milk carton** | ~1775 | **-0.174** | **-0.079** | **0.140** | **0.050×0.052** | **[122,78,67]** |
| ranch bottle | 1709 | -0.134 | +0.058 | 0.148 | 0.018×0.036 | [33,56,43] |
| bbq bottle | ~1370 | +0.06 | -0.10 | 0.113 | 0.016×0.030 | [60,22,4] |
| flat blue box | ~980 | +0.156 | +0.03 | 0.020 | — | [78,86,106] |

Seed-to-seed jitter on debug is small (≈1 cm) and only the carton, the bbq bottle,
the flat box and the basket move at all — so **position was deliberately not used as
the identity cue**; the program re-derives the carton from shape + colour each episode.

## Version log

### v1 — perception dump (hypothesis: is depth usable at all?)
No motion; dumped RGB/depth/K/T per episode to `results/dbg_c2k1clean_obj_ketchup_task_k1/`.
**Evidence:** depth 100% valid, 0.74-2.79 m; table plane recovered at z=0.001; all eight
clusters separate cleanly. **Verdict:** perception is tractable; 0/8 as expected (no motion).
Receipt: `results/fs_c2k1clean_obj_ketchup_task_k1_v1` 0/8.

### v2 — perceive → grasp ladder → place (hypothesis: the carton is the only tall non-tapering prop)
Score = 100·min(top-band spans) + (R−G), gated on ztop ≥ 0.09 and 600 ≤ px ≤ 4000.
Grasp ladder (0.100, 0.080, 0.120) around the mate pack's grasp height; carry at 0.29;
release at basket ztop + 0.13.
**Evidence:** `results/fs_c2k1clean_obj_ketchup_task_k1_v2` **8/8**. First ladder rung on
every seed, no retries. Post-close width **0.0533 m** on all eight — matches the carton's
measured 0.052 m width, so the gripper is demonstrably holding the carton and not a bottle.
**Verdict:** mechanism works, but the identification margin is unsafe: the blue can
*out-scores* the carton on the linear score (56.1 vs 49.1, because its circular top is
wider) and is excluded only by a 9 mm height gate.

### v3 — two independent identity gates (hypothesis: box-ness and redness each exclude a different impostor)
Replaced the single score with hard gates applied independently — `fullness ≥ 0.035`
(excludes the three tapering bottles: 0.016-0.036) **and** `redness ≥ 20` (excludes the
blue-grey can: R−G = −5) **and** `ztop ≥ 0.09` — with a documented relaxation path if no
candidate passes. Added a second full pass (re-perceive and re-pick) if the carton is
dropped in transit, and an explicit held-check over the basket before release.
**Evidence:** probe `results/fs_c2k1clean_obj_ketchup_task_k1_v3` **8/8**, strict gate
fired on every seed (full 0.049-0.051, red 43-45, score 48.3-49.8), first pass, first rung.
**Verdict:** selected.

## Mechanism summary

1. Capture `cam_high`, deproject the whole depth map into the base frame, crop to the
   workspace, threshold at table + 11 mm, 4-connected-label.
2. Milk = prop cluster (600-4000 px, ztop ≤ 0.30) that keeps a ≥ 35 mm cross-section in
   the band 35-5 mm below its own top **and** whose upper band is red-dominant (R−G ≥ 20).
   Basket = the largest cluster below the arm's height.
3. Open, hover at z = 0.26 over the carton centroid, descend to z = 0.100, close, settle,
   lift to 0.29. Verify with gripper effort and width; retry at 0.080 / 0.120 otherwise.
4. Transport at z = 0.29 via a midpoint (clears the ranch bottle at ztop 0.148 — the
   carton's base rides at 0.29 − 0.10 = 0.19), re-check the hold over the basket,
   descend to basket ztop + 0.13 = 0.272 so the carton's base clears the 0.142 rim,
   open, settle, retreat.

Success is never read at runtime: `api.done` is never touched, and every branch is driven
by my own gripper/residual sensors.

---

# DECLARATION

- **Frozen version:** `packs/c2k1clean_obj_ketchup_task_k1/program.py`
  md5 `0a2e412d7ffa7dd918e9ad8fffcbb5f8` == `program_v3.py` (same md5, verified on cluster).
- **Selection receipt (full 15 debug seeds 51-65):**
  `results/sel_c2k1clean_obj_ketchup_task_k1_v3` — **15/15**
  (`grep -c '"benchmark_success": true' results.jsonl` = 15).
- **Per-version receipt chain:**
  - v1 `results/fs_c2k1clean_obj_ketchup_task_k1_v1` — 0/8 (perception dump, no motion)
  - v2 `results/fs_c2k1clean_obj_ketchup_task_k1_v2` — 8/8 (probe subset)
  - v3 `results/fs_c2k1clean_obj_ketchup_task_k1_v3` — 8/8 (probe subset)
  - v3 `results/sel_c2k1clean_obj_ketchup_task_k1_v3` — **15/15 (formal selection)**
- **PROVENANCE:** present as a top-level literal dict in `program.py`, covering every
  calibrated constant. All sources are the two named packs, debug-seed measurements, or
  generic controller/camera mechanics. No prior-knowledge constants were used: object
  sizes, heights, grasp offsets and the table plane were all re-measured from the v1 dump.
- **Clean room:** writes confined to `packs/c2k1clean_obj_ketchup_task_k1/*` and
  `results/*c2k1clean_obj_ketchup_task_k1*`. No forbidden file was opened; the bddl path
  was passed to `--bddl` as an opaque string. `tools/fewshot_run.py` never invoked.

STOP.
