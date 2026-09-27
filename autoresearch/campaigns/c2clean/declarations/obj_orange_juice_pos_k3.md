# c2clean — obj_orange_juice_pos_k3

Intent: "pick up the orange juice and place it in the basket".
Runner: `tools/fair_run.py` only. No shared note file (FAIR_PROTOCOL v1.1.1).

## Scene reading (from the pack + debug seeds 51/53/57/61)

**Pack (K=3).** Every demo is a three-phase trace: approach, close at
`gripper_cmd=+1`, carry, open at `gripper_cmd=-1`.

| demo | close EEF | release EEF | ee_path z-max | held gripper qpos-sum |
|---|---|---|---|---|
| 0 | (0.0779, -0.1064, 0.1203) | (0.0018, 0.2243, 0.1683) | 0.296 | 0.0546 |
| 1 | (0.0459, -0.1086, 0.0931) | (-0.0215, 0.2575, 0.1467) | 0.317 | 0.0533 |
| 2 | (0.0479, -0.1086, 0.0981) | (0.0546, 0.3026, 0.1464) | 0.300 | 0.0533 |

The demo t=0 EEF (-0.1405, 0.0054, 0.2729) matches `api.eef()` on a debug seed
(-0.1485, 0.0, 0.2613) to within a centimetre, so **the pack EEF frame is the
FairApi base frame** — pack z values are directly comparable to my own.

The pack's keyframe PNGs identify the target: comparing `demo*_t0000.png` with
the post-release frame, the squat **orange-labelled** prop is the one that
vanishes from the table and reappears inside the basket. Everything else
(green dressing bottle, two brown sauce bottles, a small box) stays put.

**Decoy check.** The demo grasp xy (x≈0.05, y≈-0.108) is nowhere near the
carton on any debug seed (x≈-0.138, y≈0.059). This is a `_pos` cell: the pack
xy is a decoy and the only transferable quantities are *heights* and the
*mechanism*. What does transfer:

- grasp z sits ~23–50 mm below the carton top (carton top measured at
  Z=0.143 on every debug seed) → `GRASP_DROP = 0.045`;
- carry apex ≈ 0.30 → `CARRY_Z`;
- release z ≈ basket rim + 0.005…0.025 → `RELEASE_DZ = 0.027`.

**Geometry from my own depth (v0 probe, cam_high deprojection).** The dominant
plane sits at Z = 0.000–0.006, so the table top is Z = 0 and every height
below is an above-table height. Connected components of the
height-gated (Z>0.018) workspace mask give 7 props plus the robot body
(the only component reaching Z=0.420, hence `Z_ARM = 0.35`):

| comp | X | Y | Ztop | orange px | what |
|---|---|---|---|---|---|
| c3 | [-0.164, -0.124] | [0.032, 0.085] | 0.143 | **308** | orange juice |
| c4 | [-0.071, 0.086] | [0.170, 0.342] | 0.144 | 0 | basket |
| others | — | — | ≤0.148 | ≤50 | distractors |

Colour rule `r/(r+g+b)>0.44 ∧ b/(r+g+b)<0.22 ∧ sum>200` puts 308 px on the
carton and ≤50 on any other prop — a 6× margin, so the target is picked by
argmax of orange-pixel count, not by a threshold.

Basket = the largest remaining footprint (0.157 × 0.171 m; the next biggest
prop is 0.036 × 0.062 m).

## Version log

### v0 — perception probe (4 seeds, `results/fs_…_v0`)
Hypothesis: I need the base-frame scene before I can plan anything.
Method: capture cam_high, log K / T_base_cam / EEF / gripper, and stream the
RGB-D back through `api.log` as chunked zlib+base64 (zero sim steps beyond the
capture). Evidence: the table plane, the component table above, and
`api.gripper()` reporting width 0.0778 at rest.
Verdict: scene solved; the carton is a 27 × 51 mm footprint box 143 mm tall,
and its 51 mm width fits inside the 78 mm jaw opening with the default
straight-down wrist (which closes along base **y**, as the demo grasp frames
show). No wrist rotation needed.

### v1 — perceive → top-down grasp → lower into the basket
Hypothesis: colour-argmax for the carton, largest-footprint for the basket,
grasp at `Ztop − 0.045`, carry at 0.30, release at `rim + 0.027`.
Evidence: probe 8/8 (`results/fs_…_v1`), formal **15/15**
(`results/sel_…_v1`). Gripper reads effort 3.0 / width 0.0531 at the close and
the same 0.0531 after the lift and after the traverse — the bite never decays.
Commanded grasp z 0.098 lands at 0.1076 (a ~10 mm controller undershoot),
which the 45 mm drop absorbs.
Verdict: works.

### v1e — aim envelope (`results/fs_…_v1ep12`, `…_v1em12`)
Hypothesis: 15/15 says nothing about margin; displace the aim and find where
it breaks. Method: v1 with the grasp y shifted +12 mm and −12 mm, 4 seeds each.
Evidence: **4/4 and 4/4**. Verdict: the grasp tolerates ≥12 mm of lateral
aim error in either direction, so perception noise on an unseen layout is not
the failure mode to guard against.

### v2 — anti-fusion localisation + grasp retry  ← FROZEN
Hypothesis: the debug seeds only ever perturb the **basket** (x spread
−0.014…+0.015, y spread 0.245…0.273) and one distractor; the carton is
byte-identical on all 15. A blind eval seed that moves the carton against a
neighbour would weld the two into one component and drag the bbox mid off the
target — a failure v1's debug receipt cannot see.
Method: (a) once a component's orange pixels are found, keep only the part of
it within 45 mm of the orange-label centroid before taking the bbox
(`LOCALISE_R`); (b) after the lift, verify the hold with
`effort ≥ 3.0 ∧ width > 0.010` and, if it failed, re-perceive and bite 15 mm
lower once.
Evidence: offline, the localisation gate is a **no-op on the debug seeds** —
same aim to 0.1 mm as v1. On the cluster: probe 8/8 (`results/fs_…_v2`),
formal **15/15** (`results/sel_…_v2`), every episode reporting `tries=1`, i.e.
the retry never fired and the guard cost nothing.
Verdict: identical behaviour on debug, strictly more robust off it. Frozen.

## DECLARATION

- **Frozen version:** `program_v2.py` → `packs/c2clean_obj_orange_juice_pos_k3/program.py`,
  md5 `f8e7e6c0183441f4df56a45cbbd45b58` (identical on the Mac and the cluster).
- **Selection receipt:** **15/15** on the full debug split (seeds 51–65),
  `results/sel_c2clean_obj_orange_juice_pos_k3_v2/results.jsonl`.
- **Receipt chain:**
  - v0 perception probe — 4 seeds, `results/fs_c2clean_obj_orange_juice_pos_k3_v0`
  - v1 probe 8/8 — `results/fs_c2clean_obj_orange_juice_pos_k3_v1`
  - v1 formal 15/15 — `results/sel_c2clean_obj_orange_juice_pos_k3_v1`
  - v1e +12 mm 4/4 — `results/fs_c2clean_obj_orange_juice_pos_k3_v1ep12`
  - v1e −12 mm 4/4 — `results/fs_c2clean_obj_orange_juice_pos_k3_v1em12`
  - v2 probe 8/8 — `results/fs_c2clean_obj_orange_juice_pos_k3_v2`
  - v2 formal 15/15 — `results/sel_c2clean_obj_orange_juice_pos_k3_v2`
- **Tie-break:** v1 and v2 both score 15/15. v2 is declared because its two
  additions are provably inert on the debug split (same aim, `tries=1`
  everywhere) and only engage in layouts the debug split never produced.
- **PROVENANCE:** present as a top-level literal dict in `program.py`, covering
  WS_X, WS_Y, Z_TABLE, Z_PROP, Z_ARM, ORANGE, GRASP_DROP, CARRY_Z, RELEASE_DZ,
  GRIP_OPEN, LOCALISE_R, RETRY_DZ, HOLD_EFFORT. Every constant is sourced from
  either this pack's contents or a debug-seed measurement; no foreign or
  prior-knowledge constants were used.
- Seeds 1–50 were never run or inspected. `tools/fewshot_run.py` was never
  invoked. No forbidden file was read.
