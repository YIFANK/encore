# c2clean / obj_milk_task_k0 — working notes

Intent: **"Pick the butter and place it in the basket"**. No demo pack. All
constants below are re-derived from debug seeds 51-65 only.

## Scene (measured, cam_high deprojection, debug seeds 51,53,55,57,59,61,63,65)

Base-frame, table surface at z ≈ 0.0014. Seven components after a
`z < 0.18` gate that removes the parked arm (see v2 note).

| label (read with the wrist cam, v4) | ctr (x,y) | top z | dx × dy | mean rgb | r−b |
|---|---|---|---|---|---|
| basket | (+0.005,+0.264) | 0.1436 | 0.159 × 0.170 | [140,139,134] | +6 |
| orange juice | (+0.150,+0.030) | 0.1428 | 0.047 × 0.053 | [94,67,31] | +63 |
| milk | (−0.121,−0.240) | 0.1416 | 0.053 × 0.054 | [85,60,54] | +31 |
| can | (−0.153,+0.057) | 0.0812 | 0.063 × 0.069 | [63,55,46] | +17 |
| **FARM FRESH BUTTER** | **(+0.105,−0.204)** | **0.0189** | **0.075 × 0.039** | **[92,56,39]** | **+53** |
| cream cheese | (+0.055,−0.098) | 0.0199 | 0.080 × 0.042 | [72,78,97] | −25 |
| chocolate pudding | (−0.193,−0.080) | 0.0296 | 0.080 × 0.048 | [69,54,48] | +21 |

Prop positions are identical to 4 decimals across all 8 probed debug seeds;
only the basket drifts (rim ctr x ∈ [−0.007,+0.021], y ∈ [+0.250,+0.271]).
Every box is axis-aligned, long axis along x, so a straight-down wrist closes
its jaws across the SHORT (y) axis.

## Version log

### v1 / v1b — perception probe (no motion)
Hypothesis: RGB-D can be shipped out through `api.log` (zlib+base64) and all
perception done offline, at zero sim cost.
Evidence: v1 chunked at 4000 chars and every line came back truncated to
~1975 — `api.log` caps a line near 2000 bytes. v1b at 1500 chars round-tripped
512×512×3 RGB + 512×512 float depth from both cameras on 8 seeds.
Verdict: works; this is the perception pipeline for every later version.

### v2 — pick the THICKEST flat box, place in basket → **0/4** (51,53,55,57)
Hypothesis: "butter" is the thickest of the three flat boxes (the one whose
cam_high crop looked like a golden slab on a brown wrapper).
Evidence: mechanics were clean — grasp closed to width 0.0458 on a 0.048-wide
box at effort 3.0, held through the lift and the whole carry, target site
re-perceived empty, released inside the basket. Yet `benchmark_success` false
on all four seeds. Also knocked the milk carton over: the transit from the
pressed-down calibration pose straight to the grasp hover is a low diagonal
that sweeps through anything in between.
Verdict: mechanics fine, target wrong.

### v3 — v2 + RGB dumps at lift / over-basket / end → 0/1 (51)
Hypothesis: find out where the object actually went.
Evidence: the `cam_arm_wrist` dump taken over the basket shows the held box
head-on, and it reads **"INSTANT CHOCOLATE PUDDING"**. The thickest-box rule
selected the wrong object.
Verdict: the wrist camera at ~0.17 m above a prop is a legible label reader.

### v4 — wrist label read over all six props (no grasp) → n/a (51)
Hypothesis: read every label and identify the butter directly.
Evidence: labels, in the order the probe visited them: milk / **FARM FRESH
BUTTER** / Cream Cheese / CHOCOLATE PUDDING / orange juice / a can (top view).
Butter is the *thinnest* flat box (top 0.0189), at (+0.105,−0.204), and by far
the warmest: r−b = +53 vs +21 (pudding) and −25 (cream cheese).
Verdict: selection rule = rank the flat boxes by r−b and take the argmax
(32-count margin over the runner-up); the thickness rule is refuted.

### v5 — warmest flat box + safe vertical transits → probe 4/4 (51,53,55,57), **formal 15/15**
Hypothesis: with the butter correctly identified, the v2 mechanics carry the task.
Changes: (a) target = argmax(r−b) among components with top < 0.06;
(b) every lateral move is a hop at z = 0.26 with vertical approach/retreat, so
no transit sweeps a standing carton; (c) wrist-camera dumps over the basket
before release and after retreat, to verify the object actually lands inside
(cam_high cannot see the near half of the basket floor).
Evidence: closed width 0.0389 against a measured short axis of 0.039 on every
seed, effort 3.0 held through the whole carry, target site re-perceived empty,
success on all 15 debug seeds (`results/sel_c2clean_obj_milk_task_k0_v5`,
~448 sim steps of the horizon used). Two roughnesses: the descent into the
basket is contact-limited and stalled ~25 mm high on seed 53, and the single
big retreat command after it jammed (residual 0.0851, eef did not move).
Verdict: correct; hardened below.

### v6 — grasp retry + release re-issue + staged retreat → 7/7 (52,54,...,64)
Hypothesis: a grasp retry and a verified release make the program safe on seeds
that deviate from the debug layout.
Evidence: 7/7, grasp held on the first attempt every time, staged retreat no
longer jams. But two defects showed in the logs: (a) the release loop always
ran twice because it tested `width_m`, which still read the closed 0.0389 after
the jaws had let go — the honest release receipt is `effort` falling 3.0 → 0.05;
(b) the retry condition included `len(still) == 0`, so a stray component at the
target site would have opened the jaws in mid-air and dropped a held box.
Verdict: right idea, wrong sensors.

### v7 — retry only on an unambiguously empty hand; release keyed to effort
→ probe 8/8 (51,53,...,65), **formal 15/15**
Evidence: `GRASP empty=False still=0 attempt=0` and a single-pass release on
every seed (`results/sel_c2clean_obj_milk_task_k0_v7`).
Verdict: accepted.

### v8 — v7 with one docstring word changed → **formal 15/15**  ← FROZEN
v7's docstring contained the literal token `api.` + `done` while asserting that
nothing reads it. That is a comment, not an attribute access, but the eval gate
refuses that attribute and there was no reason to hand it an ambiguous string.
v8 is v7 with that sentence reworded and is otherwise byte-identical
(`diff` over the two files is comment-only). It was re-run formally so that the
frozen bytes carry their own receipt rather than inheriting v7's.
Evidence: 15/15 on seeds 51-65, `results/sel_c2clean_obj_milk_task_k0_v8`.

## What the cell turned on

The task is mechanically easy — a flat box, an axis-aligned short side well
inside the jaw span, and a wide-open basket. The whole difficulty was
**identification**. Three flat boxes sit on the table and the intent names one
of them. Appearance at cam_high resolution (~25×28 px per box) is not legible:
the chocolate-pudding box reads convincingly as a golden slab on a brown
wrapper, and picking it scored 0/4 with flawless mechanics — a clean grasp, a
clean carry and a clean release into the basket, and no success bit. The
diagnostic that broke it open was parking the wrist camera ~0.17 m above each
prop and shipping the frame out through `api.log`: at that range every package
label is readable, and the box labelled FARM FRESH BUTTER turned out to be the
*thinnest* of the three, not the thickest.

Once the label was known, the cam_high signature that separates it is colour,
not size: r−b = +53 for butter against +21 (pudding) and −25 (cream cheese).
The frozen program ranks on that margin rather than thresholding it.

## DECLARATION

- **Frozen version: v8.**
  `packs/c2clean_obj_milk_task_k0/program.py`
  md5 `8d2dcdae7ebb0cf2871baaf0ddcf4e2c`
  == `program_v8.py`
  == `results/sel_c2clean_obj_milk_task_k0_v8/program_archived.py` (all three verified equal).
- **Selection receipt: 15/15** on the full 15 debug seeds 51-65,
  `results/sel_c2clean_obj_milk_task_k0_v8` (`benchmark_success: true` ×15).
- **Receipt chain (all formally-probed versions archived as `program_vN.py`):**

  | ver | seeds | result | note |
  |---|---|---|---|
  | v1 / v1b | 51-65 (8) | n/a | perception probe, no motion |
  | v2 | 51,53,55,57 | 0/4 | thickest flat box = chocolate pudding |
  | v3 | 51 | 0/1 | diagnostic dumps; wrist cam read the wrong label |
  | v4 | 51 | n/a | wrist label read over all six props |
  | v5 | 51,53,55,57 | 4/4 | correct target |
  | v5 | 51-65 (15) | **15/15** | `results/sel_..._v5` |
  | v6 | 52,54,...,64 (7) | 7/7 | retry/release hardening, wrong sensors |
  | v7 | 51,53,...,65 (8) | 8/8 | sensors fixed |
  | v7 | 51-65 (15) | **15/15** | `results/sel_..._v7` |
  | **v8** | **51-65 (15)** | **15/15** | `results/sel_..._v8` — **FROZEN** |

- **PROVENANCE present:** 13 entries, top-level literal dict, every entry
  `allowed: True`, each sourced to a debug-seed measurement or to generic
  controller/camera mechanics. No pack was issued for this cell and none was read.
- **Clean room:** writes confined to `packs/c2clean_obj_milk_task_k0/*` and
  `results/*c2clean_obj_milk_task_k0*`; no benchmark asset, init state, trace or
  other campaign's artifact was opened; seeds 1-50 never touched; `--split eval`
  never invoked; nothing reads the episode termination flag.

STOP.
