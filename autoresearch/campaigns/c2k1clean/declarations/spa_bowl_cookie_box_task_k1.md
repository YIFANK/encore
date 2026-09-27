# spa_bowl_cookie_box_task_k1 — working notes

Intent: "Pick the akita black bowl on the stove and place it on the plate".
Runner: tools/fair_run.py only. Debug seeds 51-65.

## Scene (derived from cam_high RGB-D on debug seeds, v1-v3 probes)
table_z = 0.9005 (modal workspace surface height, identical on every seed probed).
Base frame: +x toward the table front (viewer), +y to the robot's left-to-right
(the plate side is +y). Props, heights quoted as h = z - table_z:

| prop | h_top | size | note |
|---|---|---|---|
| wooden cabinet | 0.227 | large | x -0.14..0.18, y -0.35..-0.10 |
| stove slab | 0.030 | ~0.20 x 0.18 | x -0.37..-0.17, y -0.23..-0.05 |
| bowl ON the stove | 0.079-0.082 | dia 0.107 | the intent's object |
| bowl next to cookie box | 0.051 | dia 0.109 | on the table, x~0.10-0.14 |
| third (metallic) bowl | 0.043 | dia ~0.10 | x~-0.20, y~+0.20 |
| cookie box | 0.020 | 0.082 x 0.059 | lies flat, rgb (95,68,47) |
| plate | 0.019-0.020 | dia ~0.13 | y +0.14..+0.28, the goal |
| robot arm / fingers | >=0.29 | - | fingers at y = +-0.049, jaw axis is base-y |

## Mechanism (the grasp)
The bowl is ~0.11 m across at every height; the jaws open to 0.0778 m, so the
gripper cannot straddle it. Both demo packs close to a finger gap of ~0.0086 m,
which is a wall thickness -> the grasp is a WALL PINCH: one jaw inside the
cavity, one outside, tool centred on the bowl wall about a rim-radius off the
bowl centre, along the jaw axis (base-y).
Grasp height: k1 demo closes on a table bowl at h=0.019; mate demo closes on the
stove bowl at h=0.048-0.051. The 0.030 difference is exactly the stove slab
height. Rule: grasp_h = bowl_base_h + 0.019, bowl_base_h = h_top - 0.051.
Carry at h=0.245 (mate ee_path6), release at h=0.032 (mate opens at h=0.030).

## Version log
- v1 (ep51,53): perception only. ASCII height/luma/redness maps. Gave table_z
  and the gross layout. 0 motion.
- v2 (ep51,53,57): connected-component descriptors at 4 height thresholds.
  Found plate, cookie box and the two table-standing bowls; the stove bowl stayed
  merged into the cabinet/stove blob at every flat threshold. 0 motion.
- v3 (ep51,57): 1 cm height field of the back-left region, printed in cm. This is
  what separated the stove slab (h=0.030) from the bowl on it (rim h=0.079-0.082,
  cavity floor h=0.040) and showed the bowl moves ~3.5 cm between seeds.
- v4a/b/c (ep51,53,57,61): first end-to-end attempts, identical except the pinch
  offset GRASP_DY.
  * v4a GRASP_DY=+0.050 -> **4/4 benchmark_success**
  * v4b GRASP_DY=-0.050 -> **4/4 benchmark_success**
  * v4c GRASP_DY= 0.000 -> 0/4. Descend residual 0.026-0.032 (blocked by the
    bowl), close gives gap 0.002 and effort 0.05: nothing held.
  Verdict: the wall-pinch hypothesis is confirmed, and both pinch sides work
  while the centred grasp cannot work at all. Also confirms the benchmark
  predicate is satisfied by the bowl the INTENT names (the stove bowl), even
  though the bddl filename names the cookie-box bowl.
  Two defects found in v4a's receipts, to fix in v5:
  * the HELD check read gap<0.005/effort 0.05 after the lift on runs that were
    in fact holding -- the gap fluctuates while the arm accelerates, so hold has
    to be sampled right after the close, not after the lift;
  * the plate centre was taken as the blob bbox midpoint in x and the median in
    y, which disagree by 3 cm; the bowl landed at y=0.199 against a plate whose
    near edge is at y=0.140, i.e. on the plate but with little margin.
- v5 (probe ep51,53,55,57,59,61,63,65 -> **8/8**; selection ep51-65 -> **15/15**):
  v4a hardened. Changes: (1) the hold test is sampled immediately after the
  close instead of after the lift, because the finger gap dips below the 0.005
  threshold while the arm accelerates and made a real hold read as a miss;
  (2) a descend residual above 0.018 aborts the rung before spending a close,
  since v4c showed a blocked column stalls at 0.026-0.032 while a clear one
  lands at 0.008-0.012; (3) a retry ladder over pinch offsets
  (+0.050, -0.050, +0.044), each rung verified by residual then by gap+effort;
  (4) the plate centre is the blob bbox midpoint in BOTH axes and the blob must
  be plate-sized (0.09..0.19) and roughly circular, with a Kasa circle fit
  logged alongside as a cross-check.
  On all 15 selection seeds the FIRST rung held: descend residual 0.0089-0.0112,
  close gap 0.0113-0.0145 at effort 3.00. No episode used a retry, no stderr.

## Note on the predicate (observation, not an assumption the program relies on)
The bddl passed to --bddl is the "black bowl next to the cookie box" task, while
the intent sentence names the bowl on the stove. These are two different props
in this scene (measured: the cookie-box bowl tops out at h=0.051 at x~0.10-0.14,
the stove bowl at h=0.079-0.082 at x~-0.25 to -0.31). The program follows the
intent sentence and moves the STOVE bowl, and benchmark_success fired on 15/15
debug seeds. That establishes the predicate is satisfied by the bowl the intent
names; it does not distinguish whether the predicate is specific to that
instance or agnostic between the two akita bowls, and nothing in the program
depends on which it is.

## DECLARATION
- Frozen version: **v5**.
  `packs/c2k1clean_spa_bowl_cookie_box_task_k1/program.py`
  md5 5fb101a7b89d4e227ae2d63029057248 ==
  `packs/c2k1clean_spa_bowl_cookie_box_task_k1/program_v5.py` (same md5).
- Selection receipt: **15/15** on the full 15 debug seeds 51-65,
  `results/sel_c2k1clean_spa_bowl_cookie_box_task_k1_v5`
  (per-episode: 51,52,53,54,55,56,57,58,59,60,61,62,63,64,65 all
  `"benchmark_success": true`; sim_steps 139-161).
- Per-version receipt chain:
  | version | dir | seeds | result |
  |---|---|---|---|
  | v1 | fs_..._v1 | 51,53 | perception only, 0 motion |
  | v2 | fs_..._v2 | 51,53,57 | perception only, 0 motion |
  | v3 | fs_..._v3 | 51,57 | perception only, 0 motion |
  | v4a | fs_..._v4a | 51,53,57,61 | 4/4 (pinch +0.050) |
  | v4b | fs_..._v4b | 51,53,57,61 | 4/4 (pinch -0.050) |
  | v4c | fs_..._v4c | 51,53,57,61 | 0/4 (centred grasp) |
  | v5 | fs_..._v5 | 51,53,55,57,59,61,63,65 | 8/8 |
  | v5 | sel_..._v5 | 51-65 (all 15) | **15/15** |
- Archived versions in the pack dir: program_v1.py, program_v2.py, program_v3.py,
  program_v4a.py, program_v4b.py, program_v4c.py, program_v5.py.
- PROVENANCE: present in program.py as a top-level literal dict, 17 entries,
  every calibrated constant sourced to either the named packs' pack.json /
  keyframes or to a numbered debug-seed probe recorded above.
- Clean room: the only cluster writes were
  `packs/c2k1clean_spa_bowl_cookie_box_task_k1/*` and
  `results/*c2k1clean_spa_bowl_cookie_box_task_k1*`. No forbidden file was read;
  the bddl path was only ever passed to --bddl. Every program ran under
  tools/fair_run.py; tools/fewshot_run.py was never invoked. api.done is never
  read.
