# c2clean / obj_chocolate_pudding_task_k0 — working notes

Intent: **"Pick the salad dressing and place it in the basket"**
No demonstration pack (k0). Runner: `tools/fair_run.py` only. Debug seeds 51-65.

---

## v1 / v1b — RGB-D dump (no motion)

**Hypothesis.** With no pack, everything must come from my own observations, so
the cheapest first move is to get the raw cam_high RGB-D off the box and do
perception offline (zero sim steps spent on it).

**Method.** zlib+base64 of the raw `rgb`/`depth` buffers, chunked through
`api.log`, plus `intrinsics` / `t_base_cam`.

**Evidence.**
- v1 (`results/fs_..._v1`, seeds 51,53,55,57): chunks came back TRUNCATED —
  `api.log` caps a message at 2000 characters, so 3000-char chunks lost their
  tails and zlib refused the stream.
- v1b (`results/fs_..._v1b`, seeds 51,53,55,57,59,61,63,65) with 1900-char
  chunks: all 8 seeds decoded cleanly.

**Verdict.** Log-as-datapipe works; the cap is 2000 chars per `api.log` line.

### What the scene is
cam_high: K = fx=fy=618.04, c=(256,256); camera at base (0.897, 0, 0.650)
looking back along -x and 32 degrees down. Table plane z = 0.0012.

Deprojecting and footprint-clustering (15 mm xy cells, max-z per cell,
8-connected) gives, identically on all 8 debug seeds:

| cluster | centre (x,y) | top z | dx x dy | look |
|---|---|---|---|---|
| basket | (0.006..0.014, 0.245..0.261) | 0.144 | 0.15 x 0.17 | silver weave |
| ketchup | (0.101, -0.200) | 0.148 | 0.037 x 0.063 | orange label |
| **salad dressing** | **(0.151, 0.029)** | **0.148** | 0.036 x 0.063 | **green cap** |
| OJ carton | (0.050, -0.101) | 0.143 | 0.053 x 0.053 | yellow/orange |

Two further props (an alphabet-soup can and a BBQ-sauce bottle) sit behind
x = -0.06 and are cropped out with the robot column; they are never candidates.

The RGB crop identifies the target by eye: the bottom-right bottle is a
"Creamy Ranch Dressing" bottle with a **green cap**; the other dark bottle is
BBQ sauce. Target = the green-capped bottle at (0.151, 0.029).

**Colour separation is wide.** Fraction of points in the top-30 mm band with
`G - max(R,B) > 0.04`: target **0.76**, every other cluster **0.00**.

**Shape of the target** (per-1 cm z band, y-extent = true diameter):
```
z 0.14  dy 0.035  green      <- cap top disc, x 0.134..0.168 -> centre x 0.151
z 0.13  dy 0.035  green
z 0.11  dy 0.034  green
z 0.10  dy 0.034  grey       <- neck
z 0.08  dy 0.041
z 0.05  dy 0.060  }           body flares to 63 mm
z 0.03  dy 0.063  }
```
So there is a clean 35 mm-wide column from z=0.09 to the 0.1475 top. That is
the grasp feature (jaws open to 0.079, body at 0.063 would be marginal).

**Object layout is bit-identical across all 8 debug seeds**; only the basket
jitters, by about +-0.02 m. Perception is still done at runtime so the program
does not depend on that.

---

## v2 / v2b — fingertip offset calibration

**Hypothesis.** `api.eef()` is not the fingertip; I need the offset to pick a
grasp height, and it must be measured, not assumed.

**v2** descended an open gripper at (0.05, 0.24) and stalled at eef z = 0.146.
That "clear table patch" was wrong — it is inside the basket footprint
(x -0.07..0.08, y 0.17..0.34, rim 0.144). Discarded.

**v2b** (`results/fs_..._v2b`, seed 51) descended a CLOSED gripper at
(0.22, 0.20), genuinely clear table. eef z froze at **0.0095** for three
successive lower commands while the command went to -0.005.

**Verdict.** `fingertip z = eef z - 0.0083` (table at 0.0012). Adopted as
`TIP_OFFSET`.

**Side finding (load-bearing).** `api.move` does NOT converge in one call on a
long move: the 2.0 s hover command left a 0.045 residual, and each 1.0 s
descent step tracked ~30 mm below its commanded z. Moves must be re-issued
until the returned residual drops. Every later version uses a `goto()` helper
that repeats the same command up to N times until `residual < tol`.

---

## v3 — first full pick and place

Perceive -> select target (greenest small cluster, top < 0.25) and basket
(largest cluster with a footprint >= 0.10 m) -> hover 0.26 -> descend to
`top - 0.025 + TIP_OFFSET` = 0.1308 -> close -> lift 0.34 -> over basket ->
release at 0.30 -> retreat + re-perceive.

**Evidence** (`results/fs_..._v3`, seeds 51,53,55,57): **4/4 benchmark_success**.
Grasp width 0.0368 with effort 3.0 (matches the 35 mm neck), effort held 3.0
through the whole carry. Post-episode re-perception: the green cluster is gone
from the table.

**Concern.** The trailing retreat move returned residual 0.0354 with a frozen
eef — the episode horizon had run out. The release happened in time, but the
margin was thin.

---

## v4 — trimmed for horizon

Same as v3 with the post-release retreat and re-perception dropped and the
coarse `goto` tolerances loosened (hover/carry 0.012-0.015, descend 0.006).

**Evidence.**
- probe `results/fs_..._v4` (8 seeds 51..65 odd): **8/8**
- selection `results/sel_..._v4` (all 15 debug seeds): **15/15**

## v4aim — aim-envelope probe (diagnostic, not a candidate)

v4 with the grasp aim displaced by (+12, +12) mm.
`results/fs_..._v4aim`, seeds 51,53,55,57: **0/4**, gripper closed on air
(width 0.001, effort 0.05).

**Why it matters.** The v4 descent has a repeatable steady-state bias: it aims
(0.1511, 0.0291) and lands at (0.1587, 0.0295), i.e. **+7.6 mm in x**. The neck
is 35 mm and the jaws 79 mm, so the capture half-window is 22 mm; v4 spends a
third of it on a bias it could simply cancel. 15/15 was true but on a thin
margin.

---

## v5 — cancel the descent tracking bias (FROZEN)

After the descent, read `api.eef()`, subtract the lateral error from the
command and re-issue (up to 2 corrections, 4 mm deadband). At grasp height the
open jaws are 22 mm clear of the neck, so the correction cannot disturb the
prop. The lift then uses the corrected xy.

**Evidence.**
- probe `results/fs_..._v5` (8 seeds): **8/8**. Bias cancelled in one step:
  `AIM_ERR ex=+0.0076` -> `ex=+0.0006`; `GRASP_POSE eef=[0.1518, 0.0292]` vs
  aim (0.1511, 0.0291) — 0.7 mm instead of 7.6 mm. Grasp width 0.0369,
  effort 3.0.
- selection `results/sel_c2clean_obj_chocolate_pudding_task_k0_v5`
  (all 15 debug seeds 51-65): **15/15 benchmark_success**.

### Aim-envelope probes (diagnostics, not candidates)

Both displace the aim away from the perceived cap centre; the closed loop then
drives faithfully to the *wrong* point, so these measure how much PERCEPTION
error the grasp tolerates.

| probe | injected offset | eef error at close | result |
|---|---|---|---|
| v5 (frozen) | none | (+0.7, +0.1) mm | **15/15** |
| v4 (biased aim) | none | (+7.6, +0.4) mm | 15/15 |
| `fs_..._v5aim2` | (+13, 0) mm | (+16.5, +0.2) mm | **0/4**, closed on air |
| `fs_..._v5aim` | (+12, +12) mm | (+15.6, +12.2) mm | **0/4**, closed on air |
| `fs_..._v4aim` | (+12, +12) mm | (+22, +12) mm | **0/4**, closed on air |

**Envelope.** The 35 mm neck is captured at 7.6 mm of aim error and lost by
16.5 mm, so the usable window is roughly +-12 mm about the cap centre. v5's
bias cancellation is what keeps the frozen program at 0.7 mm instead of
spending 7.6 mm of that 12 mm budget before perception error is even counted.
On the debug seeds the perceived cap centre is identical to 0.1 mm across all
15 seeds, so the whole window is available to eval-seed layout differences.

---

## DECLARATION

- **Frozen version: v5.**
  `packs/c2clean_obj_chocolate_pudding_task_k0/program.py`
  md5 `0aeff41dbc1c9a442c4cb0c11cb197b0` ==
  `packs/c2clean_obj_chocolate_pudding_task_k0/program_v5.py` (verified on the
  cluster).
- **Selection receipt (full 15 debug seeds): 15/15 benchmark_success**, dir
  `results/sel_c2clean_obj_chocolate_pudding_task_k0_v5`
  (seeds 51,52,53,54,55,56,57,58,59,60,61,62,63,64,65).
- **Per-version receipt chain**

  | version | run dir | seeds | result |
  |---|---|---|---|
  | v1 | `fs_..._v1` | 51,53,55,57 | dump truncated (2000-char `api.log` cap) |
  | v1b | `fs_..._v1b` | 8 odd | RGB-D decoded on all 8; scene mapped |
  | v2 | `fs_..._v2` | 51,53 | probe point was inside the basket; discarded |
  | v2b | `fs_..._v2b` | 51 | `TIP_OFFSET = 0.0083 m` measured |
  | v3 | `fs_..._v3` | 51,53,55,57 | 4/4, but ran the horizon out on retreat |
  | v4 | `fs_..._v4` | 8 odd | 8/8 |
  | v4 | `sel_..._v4` | all 15 | 15/15 (superseded: 7.6 mm aim bias) |
  | v4aim | `fs_..._v4aim` | 51,53,55,57 | 0/4 (envelope diagnostic) |
  | **v5** | `fs_..._v5` | 8 odd | 8/8, aim error 7.6 mm -> 0.7 mm |
  | **v5** | `sel_..._v5` | **all 15** | **15/15 — SELECTED** |
  | v5aim | `fs_..._v5aim` | 51,53,55,57 | 0/4 (envelope diagnostic) |
  | v5aim2 | `fs_..._v5aim2` | 51,53,55,57 | 0/4 (envelope diagnostic) |

- **PROVENANCE present** in `program.py` as a top-level literal dict covering
  XLO/XHI/YLO/YHI/ZLO/ZHI, CELL, GREEN_T, TIP_OFFSET, GRASP_DEPTH, AIM_TOL,
  CARRY_Z, RELEASE_Z. Every constant traces to a v1b cam_high point-cloud
  measurement, the v2b fingertip-offset measurement, or generic controller
  mechanics. No pack was supplied and none was read; no benchmark asset file
  was opened; `api.done` is never referenced.
- Not mechanism-blocked. STOP.
