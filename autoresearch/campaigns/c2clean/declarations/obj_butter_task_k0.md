# c2clean / obj_butter_task_k0 — worker notes

Intent: **"Pick the orange juice and place it in the basket"** (no demonstration
pack; everything below is derived from debug seeds 51-65 only).

## v1 — perception probe (no motion)

Hypothesis: with no pack, the only way to learn the scene is to look at it.
Dump cam_high + cam_arm_wrist RGB-D out through `api.log` (zlib+base64, 1800-char
chunks) and do the perception offline.

Evidence (seeds 51, 53; `results/fs_c2clean_obj_butter_task_k0_v1`):

- Base frame: the flat support is **z = 0.000**; props occupy z 0.012-0.15; the
  parked arm/gripper sits at z 0.26-0.30.
- `cam_high` is a fixed agentview (fx=618, 512x512, t_base_cam translation
  (0.897, 0, 0.65)); image +u -> base +y, image +v -> base +x.
- Scene = 6 props in a loose double row plus a wicker basket to the +y side.
  Height-gated (0.012 < z < 0.20) XY clustering on a 0.01 m grid separates them
  into exactly 7 components (a 0.02 m grid fuses the carton with the dressing
  bottle; a 0.30 m z-cap fuses the carton with the parked gripper fingers).

| component | xy (m) | top (m) | mean rgb | identity |
|---|---|---|---|---|
| n≈2900 | (+0.03, +0.26) | 0.142 | (140,139,135) | **basket** |
| n≈465  | (−0.13, +0.06) | 0.139 | ( 93, 67, 31) | **orange-juice carton** |
| n≈441  | (−0.19, −0.08) | 0.148 | ( 82, 64, 55) | salad-dressing bottle |
| n≈398  | (+0.17, +0.03) | 0.113 | ( 60, 27,  8) | BBQ-sauce bottle |
| n≈503  | (+0.06, −0.10) | 0.081 | ( 68, 61, 52) | can |
| n≈297  | (+0.11, −0.20) | 0.029 | ( 72, 57, 51) | flat box |
| n≈124  | (−0.11, −0.24) | 0.019 | ( 85, 54, 39) | flat box |

Verdict: the carton is visually unambiguous (orange fruit + "JUICE" on a yellow
carton). Two separating rules, both cheap:
- **basket** = the only component with a footprint > 0.12 m (0.16x0.17 vs <0.08).
- **orange juice** = among components taller than 0.10, the one that maximises
  yellowness `mean(G) − mean(B)`: 35 for the carton vs 19 (BBQ), 9 (dressing).
  Nearly 2x margin, so no threshold is needed — rank, don't cut.

Seeds 51 and 53 differ only in the basket (~15 mm in x); the props were
pixel-identical. Per-seed perception is kept anyway (nothing is hard-coded).

## v2 — calibrate, top-down pick, drop in the basket

Hypothesis: a straight top-down pick works if the one unknown — the fingertip
z offset below the eef reference — is *measured in-episode* rather than guessed.
Press the open fingertips onto the bare table at a free spot (0.00, −0.38); the
eef z where the descent stalls IS that offset, because the table is the
measured z = 0 plane. Then grasp with tips at 0.085 (upper half of the 0.139 m
carton), lift to tips 0.26 (above the 0.142 m basket rim), and open.

Evidence (probe seeds 51,53,55,57; `results/fs_c2clean_obj_butter_task_k0_v2`):
**4/4 benchmark_success**. ep51 receipt:

- `CALIB touch eef z=0.0091, residual 0.0715` -> tip offset **9.1 mm**, stalled
  against the table while still commanding 60 mm lower (i.e. real contact).
- `DESCEND eef=[-0.144, 0.059, 0.101]`, jaws open 0.0799.
- `CLOSED width 0.0530, effort 3.0` — the closed gap equals the carton's
  measured top-band y extent (0.051), so the bite is on the carton, not air.
- `LIFTED z=0.265, effort 3.0` and `OVERBASKET effort 3.0` — the grip survived
  the whole carry.
- `POST`: the carton component is gone from (−0.13,+0.06) and the basket
  component's top rises 0.142 -> 0.190. The carton is in the basket by my own
  sensors, independently of the benchmark bit.

Note the place move lands ~27 mm +x of the command (`OVERBASKET`
[0.041,0.278] for a [0.013,0.266] command) while reporting residual 0.010 —
the reported residual is not the tracking error. Harmless here: the basket is
0.16x0.17 m, so the drop is still well inside.

Verdict: v2 is the selection candidate.

### Formal selection run — v2

`results/sel_c2clean_obj_butter_task_k0_v2`: **15/15**.

But the run also exposed how little that number means here: across all 15 debug
seeds the six props are identical to the millimetre (the carton is at
(-0.139,+0.059), top 0.140, on every single seed) and only the basket jitters
(x +-0.02, y +-0.015). The debug band is **one prop layout**, so 15/15 is one
draw, not fifteen. Anything the eval band perturbs differently is untested.
Hence v3/v4.

## v3 — derive every height, cancel the tracking bias, verify the grasp

Three guards, all aimed at layouts the debug band cannot show me:
1. grasp depth and carry height derived from the *measured* carton top and
   basket rim, not from fixed numbers;
2. `aimed_move()`: move, measure the leftover horizontal error, re-issue the
   command shifted by it. (`api.move`'s reported residual is not the tracking
   error — v2 landed 27 mm off while reporting 0.010.)
3. a hold check on my own sensors, with one retry from a fresh perception.

Evidence (`fs_..._v3`, seeds 51,53,55,57): 4/4 success, but every seed took the
retry. **The hold check was wrong**: `expect_w` was `min(top_ext)` = 0.027, the
carton's SHORT axis, while the jaws close along base y where the carton
presents its LONG axis (0.051). A perfectly good 0.053 bite failed the
`w < expect_w + 0.025` test, so the program opened the jaws at carry height,
dropped the carton 0.25 m, and re-picked it. It got away with it four times.

Verdict: right idea, inverted test. Not run formally.

## v4 — v3 with the hold check corrected  **[FROZEN]**

`api.grip(0.0)` always commands a full close, so jaws that stop with a gap AND
report squeeze have something between them; an empty close collapses toward 0.
The upper bound is therefore unnecessary and was the whole bug — removed. Also:
the retry now sets the object down (`top + d + 0.01`) before re-perceiving,
instead of opening at carry height.

Evidence (`results/sel_c2clean_obj_butter_task_k0_v4`, all 15 debug seeds):
**15/15, zero retries.** Every seed: `LIFTED1 ... width 0.0530, effort 3.0`,
and effort still 3.0 at `OVERBASKET`, i.e. the grip is proven by my own sensors
from close to release, before the benchmark bit is consulted.

### Aim envelope (margin, not score)

15/15 on one layout says nothing about how much displacement the grasp takes,
so I measured it directly: copies of v4 with a fixed offset injected into the
grasp aim and the retry disabled (`envprobe_dy*.py` — probes, not candidates).

| injected lateral aim error | seeds 51,53,55,57 |
|---|---|
| 10 mm | 4/4 |
| 15 mm | 4/4 |
| 20 mm | 4/4 |
| 28 mm | **0/4** |

The break is exactly where the geometry predicts: jaws open 0.0799 on a carton
measured 0.051 wide leaves 14.5 mm of slack per side. At 28 mm the log reads
`CLOSED1 width 0.0010, effort 0.05` and `POST` still shows the carton standing
at (-0.133,+0.062) — the jaws closed on air. Two things follow: the frozen
program's own aim error after correction is ~3 mm, a ~7x margin; and the hold
check is a *true* detector — it fired `ok=False` on exactly the episodes where
the grasp missed, which is what arms the retry on a displaced eval seed.

---

# DECLARATION

- **Frozen version: v4.** `packs/c2clean_obj_butter_task_k0/program.py`
  md5 `88b64ca312b2a1c43016dfcf5cba37a8` == `program_v4.py` (same md5, verified
  on the cluster).
- **Selection receipt: 15/15** on the full debug band (seeds 51-65),
  `results/sel_c2clean_obj_butter_task_k0_v4`, note `v4 done ok=True` on every
  episode, zero retries.
- **PROVENANCE**: present, 16 entries, every calibrated constant sourced to a
  debug-seed measurement or to generic controller/camera mechanics. The eval
  gate was dry-checked by calling `fair_run.scan_program(program.py, "eval")`
  directly (NOT by running `--split eval`): **accepted**.
- **Receipt chain**
  | version | run | seeds | result |
  |---|---|---|---|
  | v1 perception probe (no motion) | `fs_..._v1` | 51,53 | scene mapped; 7 components |
  | v2 calibrate+pick+place | `fs_..._v2` | 51,53,55,57 | 4/4 |
  | v2 formal | `sel_..._v2` | 51-65 | **15/15** |
  | v3 (+guards, bad hold test) | `fs_..._v3` | 51,53,55,57 | 4/4, but a needless drop+repick on every seed |
  | **v4 formal (FROZEN)** | `sel_..._v4` | 51-65 | **15/15, 0 retries** |
  | envprobe dy=10/15/20 mm | `fs_..._env{10,15,20}` | 51,53,55,57 | 4/4 each |
  | envprobe dy=28 mm | `fs_..._env28` | 51,53,55,57 | 0/4, jaws close empty |

- **Mechanism, in one line**: the orange juice is the tallest-class component
  whose mean colour is most yellow (G−B = 35 vs 19 for the runner-up); the
  basket is the only component with a footprint over 0.12 m; the single unknown
  — the fingertip offset below the eef reference — is measured in-episode by
  pressing the open jaws onto the table, whose plane the same depth image puts
  at z = 0. No demonstrations, no hard-coded positions, no success feedback.
