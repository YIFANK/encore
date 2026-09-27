# c2clean / goal_put_wine_top_cabinet_task_k0

Intent: **"put the wine bottle in the bowl"** (no demonstration pack; k0).
Runner: `tools/fair_run.py` only. Debug split 51-65. Eval 1-50 never touched.

Note on the bddl filename: the cell's bddl path is the stock
`put_the_wine_bottle_on_top_of_the_cabinet.bddl`, but the instruction is
re-authored. The two goals were treated as an open question and settled
empirically: v5 puts the bottle **in the bowl** and `benchmark_success` fires
15/15, so the graded predicate follows the instruction, not the filename. No
program logic ever reads a success signal at runtime.

---

## Scene, as measured on debug seeds (all constants re-derived here)

Deprojected `cam_high` RGB-D, top-down height map at 5 mm, x∈[-0.50,0.40],
y∈[-0.50,0.50]. My own deprojection reproduces `api.deproject` to 0.1 mm on 4
anchors (v2), so the map is trustworthy.

| thing | where | top z |
|---|---|---|
| table plane | — | **0.901** |
| wine bottle | x≈-0.20, y≈-0.05 | 1.059 (cork) |
| bowl | x≈-0.09, y≈0.00, Ø 0.110 | 0.952 |
| plate | x≈0.05, y≈0.00, Ø 0.140 | 0.920 |
| cabinet / rack / stove / pot / small box | fixtures | 1.128 / 1.245 / 0.932 / 0.960 / 0.920 |

Bottle profile vs height (v3, seeds 51 & 63) — this is the load-bearing
measurement:

```
z 0.901-0.975   Ø 0.043 (body)
z 0.975-0.995   shoulder
z 1.000-1.045   Ø 0.014 (neck)
z 1.045-1.059   Ø 0.015 (cork, tan)
```

Layout varies ±0.02 m across seeds, so every number above is perceived per
episode, not hard-coded.

---

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | dump state + RGB-D through `api.log` | `api.log` truncates at ~2048 chars, so 60k chunks were silently cut | datapipe needs ~1900-char chunks |
| v2 | dump full RGB-D at the home pose | table z = 0.901; my deprojection == `api.deproject`; **bottle fuses with the parked arm** (one component, ztop 1.371) | home pose cannot see the bottle |
| v3 | park the arm aside, then capture | bottle and bowl cleanly separated on 4/4 seeds; detectors validated offline | perception solved; 475 sim steps for 2 moves |
| v4 | full pick-and-place + in-episode fingertip probe | **0/4.** Probe spot at x=+0.24 is past the reach envelope: the arm stretched out and collapsed onto the table; the remaining moves burned the 1000-step horizon (`sim_steps: 1000`, eef frozen) | two failures: over-reach, and no step budget for a probe |
| v5 | frugal single park + neck grasp, tip offset measured offline | **4/4** probe, **15/15** formal; ~400 sim steps | **FROZEN** |

### Why the grasp is on the neck
A top-down grasp on the 43 mm body is impossible: the bottle's own neck and
cork stand 85 mm above the body, so the gripper palm fouls them long before the
fingertips reach the body. The neck (Ø 14 mm, z 1.00-1.045) is the only
top-down purchase, and it is captive — the shoulder flares below it, so a
slipping bottle jams rather than falling through. Receipt: the jaws close to
**w = 0.0150 at effort 3.00** and hold that width unchanged through lift,
carry and place on every seed.

### Fingertip offset without a motion probe
v4 showed an in-episode probe is unaffordable. Instead the offset was read off
the v2 home-pose depth map: the lowest deprojected point of the +y finger is at
z = 1.1682 with `api.eef()` z = 1.1733 → **fingertips sit 5 mm below the eef
reference**. v4's (contaminated) table-contact probe gave 9 mm; two independent
measurements agreeing to 4 mm is enough for a 45 mm-tall neck. `TIP_OFF = 0.006`.

### The step budget is the real constraint
The episode horizon is 1000 sim steps and a `move` costs roughly 75 steps per
commanded second, terminating early once converged. v4 asked for ~35 s of
motion and died. v5 asks for ~11 s and lands at ~400 steps, leaving ~2.5x
headroom.

---

## DECLARATION

- **Frozen version: v5.** `packs/c2clean_goal_put_wine_top_cabinet_task_k0/program.py`
  md5 `21695350a369d3183c3c598beeecd4c1` == `program_v5.py` (verified on the cluster).
- **Selection receipt: 15/15** on the full debug split 51-65 —
  `results/sel_c2clean_goal_put_wine_top_cabinet_task_k0_v5` (every episode
  `"benchmark_success": true`).
- **Per-version receipt chain**
  - v1 `results/fs_..._v1` — 0/4 (perception dump, no motion)
  - v2 `results/fs_..._v2` — 0/2 (perception dump, no motion)
  - v3 `results/fs_..._v3` — 0/4 (park + perception only, no manipulation)
  - v4 `results/fs_..._v4` — 0/4 (over-reach + horizon exhaustion)
  - v5 `results/fs_..._v5` — **4/4** (51,53,57,63)
  - v5 `results/sel_..._v5` — **15/15** (51-65)
- **PROVENANCE**: present in `program.py` as a top-level literal dict, covering
  PARK, XLIM/YLIM/RES, TIP_OFF, NECK_DROP, BOTTLE_BAND, BOWL_BAND,
  BOWL_WIDTH_RANGE, BOWL_FLOOR, CARRY_CLEAR, CAM. Every constant traces to a
  debug-seed measurement recorded above or to generic pinhole-camera mechanics.
  No demonstration pack exists for this cell and none was used; no LIBERO-internal
  file was opened; no prior-context LIBERO constant was carried in.
