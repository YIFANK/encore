# c2clean / obj_ketchup_task_k3 — worker notes

Intent: **"Pick the milk and place it in the basket"**. Runner: `tools/fair_run.py`
only. Packs: `c2clean_obj_ketchup_task_k3` (language "pick up the ketchup and place
it in the basket") and `c2clean_obj_ketchup_task_mate` (language "pick up the milk
and place it in the basket"), pack.json + keyframes only.

## What the packs say (evidence, not a solution)

All 6 demos (3 per pack) share one shape, read off `ee_path6`:

| leg | evidence |
|---|---|
| approach | start ~(-0.15, 0.00, 0.25), descend over the prop |
| close (`g` flips −1→+1) | eef z **0.098 – 0.113**, prop at y ≈ −0.25 in the demo scenes |
| transfer | eef z **0.29 – 0.38** |
| release (`g` flips +1→−1) | eef z **0.153 – 0.212**, over xy ≈ (0.0±0.06, +0.25) |

Both packs release at the same place, so the basket is the shared goal; the two
packs differ only in which prop they close on. Keyframe PNGs confirm: the k3 pack
closes on a bottle, the mate pack closes on a red/white carton.

The demo scenes are NOT my scene (different prop sets), so only the *mechanism*
(grasp depth below the prop top, carry altitude, release altitude) transfers —
every xy must come from my own perception.

## v0 — perception probe (seeds 51,53,55,57)

No motion. Dumped cam_high / cam_arm_wrist RGB + depth through `api.log`
(zlib + base64, 1800-char chunks; `log` truncates at 2000) and did the
perception offline.

Findings (`results/fs_c2clean_obj_ketchup_task_k3_v0`):

- Table plane at **z = 0.001**; props occupy 0.01–0.17; the arm re-enters above 0.25.
- Six props + one basket, all inside |x|<0.35, |y|<0.45.
- The basket is ~11.8k px vs ≤2.3k for any prop — an order of magnitude, so
  "largest cluster = basket" is unambiguous.
- Per-cluster crops of the RGB identify the props. The one with the **Milk**
  label and a cow is the cluster at xy ≈ (−0.18, −0.08), top 0.140.

| cluster | xy | footprint | aspect | top | what it is |
|---|---|---|---|---|---|
| 0 | (−0.105, −0.237) | 0.033×0.062 | 0.53 | 0.148 | orange sauce bottle |
| 1 | ( 0.114, −0.200) | 0.064×0.070 | 0.91 | 0.081 | short can |
| 2 | ( 0.064, −0.104) | 0.028×0.049 | 0.57 | 0.113 | brown bottle |
| **3** | **(−0.181, −0.083)** | 0.051×0.053 | **0.96** | **0.140** | **milk carton** |
| 4 | ( 0.155,  0.027) | 0.080×0.042 | 0.52 | 0.020 | flat blue box |
| 5 | (−0.136,  0.058) | 0.034×0.063 | 0.54 | 0.148 | green bottle |
| 6 | ( 0.030,  0.255) | 0.157×0.171 | 0.92 | 0.142 | basket |

**Identification rule, with margin:** among non-basket clusters, milk is the only
one that is both tall (top ≥ 0.10) and square (aspect ≥ 0.75). The tall props
score 0.53 / 0.57 / 0.54 against milk's 0.96; the only other square cluster is the
can, at top 0.081, well under the 0.10 cut. Both tests are needed — neither alone
separates.

Layout across 51/53/55/57 is near-deterministic (clusters 0,1,5 pixel-identical;
2,3,4,6 wiggle ≤1 cm). I did not hard-code any xy regardless — the program
re-perceives every episode, so a different eval layout is handled by the rule, not
by the numbers.

## v1 — perceive-and-place

Hypothesis: the pack mechanism + the aspect/height rule is enough; the graded
predicate follows the **instruction** (milk), not the bddl filename (ketchup).

Program: capture cam_high → deproject → 1.5 cm grid clustering → basket = largest
cluster, milk = tall ∧ square → open, hover at 0.32, descend to
`top − 0.035` (from the packs' close height vs the measured 0.140 top), close,
verify `effort`/`width`, lift to 0.32, traverse to the basket centroid, descend to
0.18, open. Wrist held at the episode's own `tool_rotation()` throughout (never
`rotation=None`).

Evidence (`results/fs_c2clean_obj_ketchup_task_k3_v1`, seeds 51,53,55,57):

**4/4 benchmark_success = true.**

Per-episode sensors (ep51): grasp at eef (−0.2025, −0.0836, 0.1142), residual
0.0094; closed width **0.0534** with effort 3.0 — i.e. the jaws stopped on a
53 mm body, matching the milk carton's measured 0.051×0.053 footprint, so the
width itself is the receipt that the *correct* prop is in the hand. Width held
0.0533 through the lift and over the basket (no slip). Post-episode re-capture:
the milk cluster is gone from the table and the basket cluster grows from 11.8k
to 14.2k px with its top rising 0.142 → 0.236 — independent confirmation of
placement, without reading any success flag.

**Verdict: the graded predicate is the instruction's object.** Placing the milk
scores; the bddl's ketchup was never touched.

## Selection

Formal full-15 run of v1: see DECLARATION below.

## v2 — aim-envelope probe (NOT a candidate)

Per "a 15/15 says nothing about margin": v1 re-run with the grasp xy displaced
+12 mm in both x and y (17 mm diagonal) and the grasp 10 mm shallower
(`GRASP_DEPTH` 0.035 → 0.025). Seeds 51,53,55,57:

**4/4 benchmark_success = true**, closed width 0.0537 (vs 0.0534 nominal),
effort 3.0, held through the lift. So the milk grasp tolerates at least 17 mm of
lateral aim error and 10 mm of height error — the 15/15 is not sitting on a cliff.
v2 exists only to measure that; the frozen program is v1.

## Receipt chain

| version | run dir | seeds | result |
|---|---|---|---|
| v0 | `results/fs_c2clean_obj_ketchup_task_k3_v0` | 51,53,55,57 | perception dump, no motion, 0/4 (by construction) |
| v1 | `results/fs_c2clean_obj_ketchup_task_k3_v1` | 51,53,55,57 | **4/4** |
| v1 | `results/sel_c2clean_obj_ketchup_task_k3_v1` | 51–65 (all 15) | **15/15** |
| v2 | `results/fs_c2clean_obj_ketchup_task_k3_v2` | 51,53,55,57 | 4/4 (envelope probe, displaced aim) |

Rule margins over the full 15-seed selection run: milk aspect **0.90–0.98**
against a 0.75 cut, with the next-best prop at 0.57; closed gripper width
**0.05336–0.05337 m on every one of the 15 seeds** — the identical hold width
every episode means the same prop was grasped every time, the correct one.

# DECLARATION

- **Frozen version: v1.** `packs/c2clean_obj_ketchup_task_k3/program.py`
  md5 `49edeba4b39c56eb6d7cb4efc09b0267` == `program_v1.py` (same md5).
- **Selection receipt: 15/15** on the full debug band (seeds 51–65),
  `results/sel_c2clean_obj_ketchup_task_k3_v1`.
- Archived versions: `program_v0.py` (perception probe),
  `program_v1.py` (frozen), `program_v2.py` (envelope probe).
- `PROVENANCE` present in program.py: 11 constants, each sourced to a named pack
  field or a debug-seed measurement. No constant came from outside this cell.
- No `api.done` read anywhere; success was never consulted at runtime — the
  program verifies with gripper width/effort and a post-episode re-capture.
