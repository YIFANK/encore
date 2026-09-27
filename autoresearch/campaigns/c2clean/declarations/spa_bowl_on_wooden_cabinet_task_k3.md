# c2clean / spa_bowl_on_wooden_cabinet_task_k3 — worker notes

Intent: **"Pick the akita black bowl on the stove and place it on the plate"**
(re-authored cell: the bddl file name still says *wooden cabinet*; the
instruction sentence is authoritative and the benchmark bit confirmed it —
lifting the **stove** bowl onto the plate scores).

Runner: `tools/fair_run.py` only. Split debug = seeds 51–65.

---

## Evidence read from the two packs (before any run)

Both packs are the **same LIBERO-spatial scene** (their `t0000` keyframes are
pixel-wise the same layout): a wooden cabinet with a bowl on top, a stove slab
with a bowl on it, a third bowl on the bare table, a cookie box, a plate.

| pack | language | grasp keyframe eef (3 demos) | release eef |
|---|---|---|---|
| `..._task_k3` | *bowl on the wooden **cabinet** → plate* | (-0.010,-0.257,**1.159**), (-0.016,-0.217,1.146), (-0.021,-0.214,1.152) | (0.049,0.225,0.988) (0.074,0.242,0.954) (0.060,0.214,0.980) |
| `..._task_mate` | *bowl on the **stove** → plate* | (-0.276,-0.119,**0.948**), (-0.275,-0.086,0.946), (-0.284,-0.097,0.948) | (0.060,0.248,0.931) (0.063,0.229,0.939) (0.034,0.239,0.929) |

So the two packs differ only in the *object half*: the k3 pack grasps ~0.21 m
higher (cabinet top) at a different xy; both release on the same plate. The
mate pack therefore carries the grasp height and the release height for my
intent, and the k3 pack corroborates the release site.

---

## v0 — perception probe (seeds 51,53,55). No motion.

Dumped cam_high RGB + depth through `api.log` (zlib+base64) and did the
perception offline. Measurements (base frame, all three seeds agree to ~1 cm):

- table top **0.901**; stove slab top **0.927**; cabinet top **1.128**
- **stove bowl** centre (-0.252,-0.136), rim top **0.980**, outer rim radius
  **0.054**, interior floor 0.935; wall at grasp depth spans radius
  0.036…0.054 → jaw midline at radius **0.045**
- table bowl centre (-0.20,0.21), rim top 0.944, ⌀0.085 (a *different,
  smaller* bowl) — support 0.901
- cabinet bowl centre (0.02,-0.28), rim top 1.156 — support 1.128
- plate centre (0.054,0.197), rim 0.921, sunken interior 0.908, ⌀≈0.14
- cookie box (0.075,0.020), top 0.921, 0.088×0.056 (rectangular)

**Identity rule derived here:** the three bowls are told apart by their
*support height*, not by size or colour. Table = 0.901, stove = 0.927,
cabinet = 1.128. "on the stove" = the bowl whose support is the middle one.

Independent agreement: the geometric jaw-midline radius (0.045) matches the
mate pack's mean grasp offset (~0.042) from my measured bowl centre. Grasp
height 0.946 = my measured rim top 0.980 − 0.034.

## v1 — first motion program. **0/4** (51,53,55,57)

Hypothesis: perceive → rim-pinch at centre + 0.045·ŷ → carry at z 1.06 →
release over the plate at z 0.936.

Evidence: `TARGET=None`. At reset the arm hangs at (-0.209, 0.000, 1.173),
directly over the stove; in the top-down height map the arm's body **fuses
with the stove bowl** into one component, c=(-0.231,-0.049) spanning
0.150×0.294 with top 1.371. The size/height filters then rejected everything.
The plate was found correctly.

Verdict: perception bug, not a mechanism gap. Two fixes: park the arm before
capturing, and cap the bowl band at 1.05 so the arm/cabinet cannot bridge.

## v2 — park the arm, cap the band. **4/4** (51,53,55,57)

Changes: `api.move((0.06,0.40,1.20))` + settle before `capture`, bowl cells
restricted to 0.930 < z < 1.050, plus a fallback (highest support under the
cap) if the support rule selects nothing.

Receipts, ep51: `TARGET c=(-0.252,-0.136) top=0.980 sup=0.926 n=265` (the
stove bowl, correctly separated), `PLATE c=(0.054,0.197)`, grasp at
(-0.252,-0.085,0.946), closed gap **0.0128** (effort 3.0) — i.e. the jaws
took the bowl wall, not air and not the whole bowl. The gap then keeps
decaying to 0.0049 during the lift (the jaws go on closing on a thin bite);
`effort` reads 0.05 there because the harness's effort flag is really a
gap>5 mm test, so the **gap survival across the lift**, not effort, is the
hold receipt. Post-episode re-perception: the stove component drops from
top 0.980 to 0.932 — the bowl is off the stove. Final GIF frame confirms the
bowl sitting in the plate with the other two bowls untouched.

Full-15 selection: see DECLARATION below.

## v2 — full-15 selection. **15/15**

`results/sel_c2clean_spa_bowl_on_wooden_cabinet_task_k3_v2` — seeds 51–65 all
`benchmark_success: true`, no program errors, **first-try grasp on every
seed** (the -y fallback branch never fired).

Per-seed receipts (perceived rim radius → jaw offset, closed gap at contact,
gap after the lift):

| r_out | 0.054 | 0.057 | 0.060 | 0.063 | 0.066 |
|---|---|---|---|---|---|
| seeds | 56 | 55,64 | 51,52,53,60 | 57,58,59,61,62,63,65 | 54 |

offset 0.045–0.057, closed gap 0.0093–0.0128, post-lift gap 0.0034–0.0049.
The aim therefore held across a 12 mm spread of commanded offset that the
seeds sampled on their own, so the grasp is not tuned to one radius.

**Confirmation the re-authored predicate follows the instruction, not the
bddl file name:** the program never touches the cabinet bowl (the bddl's
namesake) and scores 15/15 by moving the *stove* bowl.

---

# DECLARATION

- **Frozen version:** `packs/c2clean_spa_bowl_on_wooden_cabinet_task_k3/program.py`
  — md5 `adbf0564a79625117a2c20c9245e37cc` == `program_v2.py` (same md5).
- **Selection receipt:** **15/15** on the full debug band (seeds 51–65),
  `results/sel_c2clean_spa_bowl_on_wooden_cabinet_task_k3_v2`.
- **Receipt chain:**
  - v0 — perception probe, no motion, seeds 51/53/55 → scene geometry above.
  - v1 — 0/4 (51,53,55,57), `results/fs_..._v1`; cause: the arm, left at
    its reset pose over the stove, fused with the bowl in the height map.
  - v2 — 4/4 (51,53,55,57), `results/fs_..._v2`; then 15/15 on all 15.
- **PROVENANCE:** present in `program.py`, 12 entries (TABLE_Z, BOWL_BAND,
  BOWL_CAP, SUPPORT_CUTS, PLATE_BAND, WALL_HALF, GRASP_DROP, PLACE_Z,
  CARRY_Z, PARK, R_DOWN, GRIP_OPEN), each sourced to the named packs'
  `pack.json`/`keyframes/`, my own debug-seed observations, or generic
  controller/camera mechanics.
- Archived versions: `program_v0.py`, `program_v1.py`, `program_v2.py`.
