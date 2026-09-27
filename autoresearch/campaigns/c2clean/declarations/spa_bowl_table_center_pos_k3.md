# c2clean / spa_bowl_table_center_pos_k3

Intent: *pick up the black bowl from table center and place it on the plate*
(success = the environment's own benchmark bit, judged post-episode).

Everything below was derived from `packs/c2clean_spa_bowl_table_center_pos_k3/`
(pack.json + keyframes) plus my own debug-seed (51-65) RGB-D captures. No shared
note file, no other cell's artifacts.

## Scene, as re-derived on debug seeds 51-65

`cam_high` RGB-D deprojected into the base frame (workspace crop
x∈[-0.32,0.32], y∈[-0.30,0.42]):

| prop | top z | footprint | brightness | position |
|---|---|---|---|---|
| table plane | 0.9008 | — | — | z-histogram mode, identical on all 15 seeds |
| **two black bowls** | 0.952 (table+0.051) | 0.111 x 0.111 | 120-132 | bowl A x = **-0.076 on every seed**, y ∈ [-0.015, 0.012]; bowl B roams (x 0.057-0.105, y -0.082..0.210) |
| **plate** | 0.920 (table+0.019) | 0.136 x 0.13 | 142-152 | x ≈ 0.005, y ≈ 0.32 |
| cookie box | 0.920 | 0.082 x 0.060 | 68 | x ≈ 0.08, y ≈ 0.03 |
| ramekin | 0.944 | 0.086 x 0.088 | 123 | x ≈ -0.21, y ≈ 0.20 |
| stove slab / cabinet | 0.932 / 1.128 | large | 70 | y < -0.11 |

Two facts make this segmentable with no colour model at all:

* **z > table+0.044 is exactly the two bowls.** Every other prop tops out at
  table+0.044 or below, so a single height gate yields precisely two
  components (200 < n < 5000, 0.08 < dx,dy < 0.15) on all 15 seeds. Without
  the gate, bowl A fuses with the stove slab on seeds 51 and 55 (grouping
  before height-gating loses the target).
* **the plate is the unique bright wide low component.** Band
  table+0.012 .. +0.028, dx,dy > 0.10, mean brightness > 125: one component on
  every seed. The cookie box shares the height band but reads brightness 68.

## What the `_pos` perturbation actually moves

The pack's demo release EEF is (0.060, 0.228) — in the *demo* scene that is the
plate (keyframe demo0_t0099 shows the hand over the plate). On the debug seeds
the plate is at (0.005, 0.32) and *bowl B* sits near (0.06, 0.20). The
perturbation roughly swapped the plate and bowl B, so **the demo's place xy is a
decoy**; only the demo's *relative* geometry transfers. Bowl A, by contrast,
barely moves — which is what makes the demo grasp anchor usable as an identity
cue (below).

## Target identity — "table center"

`target = the bowl nearest the pack's demo grasp anchor (-0.084, 0.047)`.
Bowl A is 0.05-0.06 m from the anchor on every seed; the runner-up is
**3.4x-6.0x further** (min margin 3.4 on seeds 54/55/58/60/62). The anchor is the
mean of the three demos' `ee_path` descent minima. Independent corroboration:
bowl A is also the min-|y| and min-x bowl on all 15 seeds.

## Grasp mechanism

The jaws open 0.0786 m (pack `gripper_state` 0.0393 per finger) and the bowl is
0.111 m across, so the bowl cannot be straddled — the demos pinch the rim wall
radially. Tool rotation at home is ≈ diag(1,-1,-1), i.e. the jaw axis is base
**y**, and the demo EEF closes at bowl_centre + (-0.008, +0.047): one finger
inside the bowl, one outside, 0.048 m out along the jaw axis. So the bowl hangs
at `eef - GRASP_OFF`, and the release pose must be `plate_centre + GRASP_OFF`,
not the plate centre itself.

**Depth is the whole task.** The descent is contact-limited, and the window is
one-sided-narrow on both ends:

| GRASP_DZ (below rim top) | commanded z | achieved z | descent residual | close gap | outcome |
|---|---|---|---|---|---|
| -0.034 (pack demo value) | 0.918 | 0.918 | — | 0.0122 | slips on the lift, 0/8 held (v1 attempt 0) |
| -0.040 | 0.912 | 0.9233 | 0.0113 | 0.0102 | **0/8** — holds through two lift stages, gap collapses to 0.0048 at z=0.957 |
| **-0.046** | 0.906 | **0.9153** | 0.0093 | 0.0095 | **8/8 probe, 15/15 formal** |
| -0.055 | 0.897 | 0.9111 | 0.0194 | 0.0082 | **0/8** — the saturated command drifts the aim 12 mm in xy, and on the third try it wedges (gap frozen at 0.0163, effort 3.0) and the next move starves at residual 0.32 |

The receipt that separates a hold from a slip is the **gap surviving the lift**:
a real hold decays 0.0095 → 0.0069 over the three staged lifts and stays at
effort 3.0; a slip drops to 0.0046 with effort 0.1. Effort 3.0 *at the moment of
closing* means nothing — every failing attempt also read 3.0 there.

Note that commanding deeper does **not** just saturate harmlessly: past the
contact point the controller buys force on every axis at once and the tool
walks sideways (v4's achieved xy was (-0.072, 0.053) for a commanded
(-0.084, 0.046)). -0.046 is the last depth whose residual stays at the clean
0.0093 contact value.

## Version chain

| ver | change | receipt |
|---|---|---|
| v0 | perception probe + blind demo-mean replay (no perception in the loop) | 0/4 (51,53,55,57); dumped RGB-D that produced the table above. Confirms the `_pos` decoy. |
| cap | capture-only on all 15 debug seeds | perception validated offline: 2 bowls + 1 plate on 15/15, target margin ≥ 3.4x |
| v1 | full perception + rim pinch at the pack's -0.034, retry at -0.040 | **3/8** (51,53,61). `benchmark_success == held` exactly on all 8 — the place leg was already correct, the grasp was the whole gap |
| v2 | single clean attempt at -0.040 + re-squeeze + 3-stage lift + re-perceive-and-retry (3 tries) | **0/8** — isolates the depth as the cause; the -0.040 successes in v1 came from grabbing an already-nudged bowl |
| **v3** | v2 with GRASP_DZ = **-0.046** | **8/8 probe; 15/15 formal** |
| v4 | v3 with GRASP_DZ = -0.055 (upper-envelope probe) | **0/8** — wedge + saturated-command drift, as above |

## DECLARATION

* **Frozen version: v3.**
  `packs/c2clean_spa_bowl_table_center_pos_k3/program.py`
  md5 `7a770dcf6c2b9e5dfe9ef0fc68698ae6` == `program_v3.py` (verified on cluster).
* **Selection receipt: 15/15** on the full debug split (seeds 51-65), one formal
  run, `results/sel_c2clean_spa_bowl_table_center_pos_k3_v3` (0 failures).
* Per-version receipt chain: v0 0/4 · v1 3/8 · v2 0/8 · **v3 8/8 probe, 15/15
  formal** · v4 0/8 (all under `results/fs_c2clean_spa_bowl_table_center_pos_k3_v*`).
* `PROVENANCE` present in program.py covering WS, DEMO_ANCHOR, BOWL_BAND,
  BOWL_SIZE, PLATE_BAND, GRASP_OFF, GRASP_DZ, PLACE_DZ, CARRY_Z, OPEN_W,
  HOLD_GAP_MIN — every source is this pack or a debug-seed (51-65) measurement.
* Eval seeds 1-50 were never touched; no `--split eval` run was issued; the
  program contains no read of `api.done`.
