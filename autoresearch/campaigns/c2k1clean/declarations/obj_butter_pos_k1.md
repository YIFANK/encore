# c2k1clean / obj_butter_pos_k1 — worker notes

Cell: `pick up the butter and place it in the basket`, K=1 pack, `_pos`
perturbation. Runner: `tools/fair_run.py` only. Debug seeds 51-65.

## Pack reading (inputs, before any motion)

`pack.json`: one demo, 169 steps, 4 keyframes, `ee_path6` at stride 10.

| t | ee xyz | grip_cmd | meaning |
|---|--------|----------|---------|
| 0 | (-0.1468, 0.0068, 0.2591) | -1 (open) | home |
| 63 | (-0.1214, -0.2542, 0.0095) | +1 (close) | **grasp** |
| 150 | (0.0234, 0.2500, 0.1909) | -1 (open) | **release** |
| 168 | (0.0184, 0.2543, 0.2382) | -1 | retreat |

Transport segment of `ee_path6` rides at z = 0.226–0.278.

## Which object is the butter (identity, derived from the pack alone)

`demo0_t0000.png` vs `demo0_t0168.png` differ (outside the arm) in exactly one
patch: u 29–36, v 61–68 (128-px frame). That is the object the demo removed
from the table, i.e. the butter.

Projecting the demo grasp point (-0.1214, -0.2542, 0.02) through the debug-seed
camera (K = f 618.04, c 256; `t_base_cam` from `api.capture`) lands at
px512 = (124.8, 254.2) = px128 = (31.2, 63.5) — inside that same patch.
Identity and grasp site agree.

Colour signature over the diff mask: mean RGB (87.6, 62.6, 50.5),
R−B = **37**, sat 0.45. The scene's *other* small box: (82.7, 69.7, 63.3),
R−B = **19**. So the butter is the **redder of the two short boxes**.

## Frame check (debug seed 51)

- `api.eef()` at reset = (-0.1485, 0.000, 0.2613) ≈ pack t=0 ee. **Same frame.**
- Table plane z = 0.0015 (depth histogram mode). Pack grasp z = 0.0095, i.e.
  ~8 mm above the table, and ~10 mm below the butter's measured top.
- `api.gripper()` at rest: width 0.0778, effort 0.05.

## v1 — world-grid clustering probe

Hypothesis: a 1.2 cm world-grid union-find would separate the scene objects.
Evidence: `results/fs_c2k1clean_obj_butter_pos_k1_v1`, seeds 51/53/55/57 — only
6 components for a scene with 7 objects + basket + arm; neighbours fused.
Verdict: **rejected**, segmentation too coarse. Frame facts above were the
useful output.

## v2 — pixel-space connected components probe

Hypothesis: 4-connected components on the above-table mask separate the cast.
Evidence: `results/fs_c2k1clean_obj_butter_pos_k1_v2`, seeds 51–58 — 8 blobs
every seed, one per object plus the arm and the basket. Also established that
the program process **can write files**, so full RGB-D captures were dumped and
inspected offline.

Per-seed blob table (seeds 51–58) — the cast, with the arm as B01:

| blob | xy | h | ext | mean RGB | R−B | what |
|------|----|---|-----|----------|-----|------|
| B00 | (0.07, 0.24–0.27) | 0.140 | 0.14×0.15 | (140,139,135) | 6 | **basket** (moves) |
| B01 | (-0.121, 0.003) | 0.445 | — | (21,42,52) | -31 | robot arm |
| B02 | (0.071, -0.101) | 0.079 | 0.03×0.06 | (68,60,51) | 17 | can |
| B03 | (-0.124, 0.058) | 0.138 | — | (91,66,30) | 61 | carton |
| B04 | (-0.183, -0.079) | 0.146 | 0.02×0.06 | (81,62,53) | 28 | bottle |
| B05 | (0.113, -0.198) | 0.028 | 0.080×0.048 | (72,57,51) | 21 | brown box (decoy) |
| B06 | (-0.11, -0.24) | 0.111 | 0.02×0.05 | (57,26,8) | 49 | BBQ bottle (moves) |
| B07 | (0.157, 0.030) | 0.017 | 0.075×0.039 | (103,63,44) | 59 | **butter** |

Verdict: **accepted** as the perception front-end.

### Two traps this table exposes

1. **The demo anchor is a decoy.** The demo grasped (-0.121, -0.254). In every
   debug seed that site holds the *BBQ sauce bottle* (B06, h = 0.111) — visually
   confirmed from the dumped RGB. Driving to the pack's grasp xy would grasp the
   wrong object. The butter sits at (0.157, 0.030) instead.
2. **Colour alone is not enough.** B03 (carton, R−B = 61) and B06 (BBQ,
   R−B = 49) are as red as the butter (59). The separator is **height**: the
   butter and the brown decoy are the only blobs under 0.055 m
   (0.017 / 0.028 vs 0.079–0.148 for everything else). Among those two, redness
   picks the butter (59 vs 21) — the same ordering the demo diff gave (37 vs 19).

So the rule is: *short blob, then reddest*. Both cues are re-derived here; both
are declared in PROVENANCE.

## v3 — first full pick-and-place

Hypothesis: perceive → top-down grasp at `ztop − 0.010` with the default
straight-down wrist → carry at z = 0.27 → release 0.050 m above the basket rim.
Evidence: `results/fs_c2k1clean_obj_butter_pos_k1_v3`, seeds 51/53/55/57/59/61/63/65
— **1/8**. The grasp itself was perfect in every episode (`after close
grip={'width_m': 0.0389, 'effort': 3.0}`, i.e. the jaws had closed on the
butter's 0.039 m minor extent), but the *release* went to
`BASKET xy=(-0.121,0.003) rim=0.447` — the **robot arm**. Its bbox
(0.121 × 0.225 = 0.0272 m²) narrowly outranked the basket's
(0.159 × 0.170 = 0.0270 m²) in the "largest footprint" test.
Verdict: identity + grasp confirmed; **basket selector rejected**.

## v4 — exclude the arm from the basket vote

Hypothesis: gate the basket vote on `h < 0.30` (arm 0.445 vs every table
object ≤ 0.148), then take the largest footprint.
Evidence: `results/fs_c2k1clean_obj_butter_pos_k1_v4` — **8/8**
(51/53/55/57/59/61/63/65), ~220 of 500 sim steps per episode.
Verdict: **accepted**.

Basket-centre check (offline, from the dumped RGB-D): the basket blob's x
distribution is bimodal — a far-wall lip near x = −0.06 and a near-wall lip near
x = +0.08 — so the *median* x (0.074) sits on the near rim while the **bbox
midpoint (0.005, 0.264) is the true centre**. v3/v4 already used the bbox
midpoint; this confirms it is the right statistic rather than a lucky one.

## v5 — hardening (frozen version)

Hypothesis: v4 is correct on the debug layouts but two of its assumptions are
untested, because seeds 51–65 barely move anything (only the basket, ±0.03 m,
and the BBQ bottle, ±0.015 m; the butter's `ext` is 0.075 × 0.039 in all 15).
Add two guards that are provably inert on these layouts but cover an unseen one:

1. **Jaw yaw from the measured footprint.** PCA on the butter's top-face world
   xy gives the minor axis; the wrist is spun about world z so the jaw axis
   (column 1 of `api.tool_rotation()`, measured as world −y at rest) lies along
   it. Applied only when the required yaw exceeds a 12° deadband.
2. **Sensed grasp check + bounded retry.** `holding()` = effort ≥ 2.0 and
   0.004 < width < 0.070. On a miss: re-open, re-perceive, re-aim, up to 3
   attempts. Step budget allows it (220 used of 500).

Evidence: `results/fs_c2k1clean_obj_butter_pos_k1_v5` — **8/8** on the probe
subset, and both guards measurably inert: `minor=(0.01,-1.00)`,
`w=0.039/0.075`, `yaw=0.8deg use_rot=False`, zero retries. The PCA recovers the
butter's true short axis, so the mechanism is exercised even though it does not
fire.
Verdict: **accepted, frozen**.

## Formal selection receipts (full 15 debug seeds, 51–65)

| version | dir | result |
|---------|-----|--------|
| v4 | `results/sel_c2k1clean_obj_butter_pos_k1_v4` | **15/15** |
| v5 | `results/sel_c2k1clean_obj_butter_pos_k1_v5` | **15/15** |

v5 selected on the tie: it is v4 plus two guards that the logs show are exactly
inert on this band (`use_rot=False` ×15, no `try1` line in any episode), so it
cannot do worse here and degrades more gracefully if an eval seed rotates the
butter or the first close misses.

## Receipt chain

| v | change | run | result |
|---|--------|-----|--------|
| v1 | world-grid clustering probe | `fs_..._v1` (4 seeds) | rejected — 6 blobs for 9 objects |
| v2 | pixel-space CC probe | `fs_..._v2` (8 seeds) | accepted as front-end; cast table + both traps |
| v3 | first full pick-and-place | `fs_..._v3` (8 seeds) | **1/8** — basket vote picked the arm |
| v4 | arm excluded from basket vote | `fs_..._v4` (8 seeds) | **8/8** |
| v4 | (formal) | `sel_..._v4` (15 seeds) | **15/15** |
| v5 | + yaw alignment, + sensed-grasp retry | `fs_..._v5` (8 seeds) | **8/8**, guards inert |
| v5 | (formal) | `sel_..._v5` (15 seeds) | **15/15** |

## DECLARATION

- **Frozen version: v5.**
  `packs/c2k1clean_obj_butter_pos_k1/program.py` md5
  `a76cc4c3fd11ef724f83c2e6c9d9e011`
  == `program_v5.py` md5 `a76cc4c3fd11ef724f83c2e6c9d9e011`.
- **Selection receipt: 15/15** on the full 15 debug seeds (51–65),
  `results/sel_c2k1clean_obj_butter_pos_k1_v5`.
- **PROVENANCE present** in `program.py`: 13 entries covering every calibrated
  constant. Sources are this pack's `pack.json`/`keyframes/` (butter identity
  from the t0000-vs-t0168 diff, grasp depth, carry altitude, release height
  above the rim), debug-seed 51–58 measurements (table plane, blob heights, arm
  height, gripper widths, minor-axis orientation), and generic camera/controller
  mechanics (pinhole deprojection, `api.grip(<0.025)` closes, effort 3.0 iff
  holding). No LIBERO fact was imported from memory or prior context; the
  object cast, its heights and its colours were all re-measured here.
- Every formally probed version archived as `program_vN.py` (v1–v5), locally and
  in the pack dir.
- Splits respected: only seeds 51–65 were ever run, always `--split debug`;
  seeds 1–50 untouched. `api.done` never read. No `fewshot_run.py`.

### Note for the coordinator: what this cell actually tests

The `_pos` band is nearly static on the debug seeds — only the basket and one
distractor bottle move, by ≤3 cm. The load-bearing difficulty is not motion
robustness but **identity**: the demo's grasp site (−0.121, −0.254) holds the
*BBQ sauce bottle* in every debug seed, so any policy that transfers the pack's
grasp xy grasps the wrong object. The butter has to be found by its own
signature (shortest-but-one blob, reddest of the two short ones). If the eval
band 1–50 perturbs positions more widely than 51–65 does, the perception
front-end is unchanged by that; the untested edges are butter rotation (covered
by the v5 yaw guard, never exercised here) and butter/decoy blob fusion
(**not** covered — see below).

### Known uncovered edge (falsifiable)

If an eval seed places the butter touching another object so the two merge into
one above-table blob, the merged blob's height becomes that of the taller
object and the `h < 0.055` short-blob gate drops the butter; the program would
then fall back to the reddest remaining short blob (the brown decoy) or, if
none, the reddest table blob. No debug seed produces an adjacency closer than
~0.09 m, so this could not be tested on the allowed band.
