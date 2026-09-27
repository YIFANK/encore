# c2clean / obj_butter_pos_k3 — working notes

Intent: "pick up the butter and place it in the basket".
Runner: `tools/fair_run.py` only. Debug band = seeds 51-65.

## Scene, as measured on the debug seeds (v1 probe)

`cam_high` is 512x512, K = fx=fy=618.04, c=(256,256); `t_base_cam` puts the
camera at base (0.897, 0.000, 0.650) looking down the -x direction. The modal
z of the deprojected cloud — the table top — sits at **z = 0.001**. fair_run's
own stdout prints the reachable workspace as x(-0.45, 0.45), y(-0.45, 0.52).

Seven things stand above the table (robot arm excluded):

| prop (by look)   | centre (x, y)      | top z | XY span      | warmth (R-B, p85) |
|------------------|--------------------|-------|--------------|-------------------|
| bbq bottle       | (-0.185, -0.079)   | 0.148 | 0.03 x 0.06  | 65                |
| orange-juice box | (-0.129, +0.059)   | 0.143 | 0.03 x 0.05  | 116               |
| ketchup bottle   | (-0.11,  -0.24)    | 0.113 | 0.02 x 0.05  | 70                |
| **basket**       | ( 0.00,  +0.26)    | 0.144 | 0.16 x 0.17  | 15                |
| soup can         | ( 0.057, -0.105)   | 0.081 | 0.06 x 0.07  | 64                |
| brown slab       | ( 0.115, -0.196)   | 0.029 | 0.08 x 0.05  | **41**            |
| **orange slab**  | ( 0.152, +0.029)   | 0.019 | 0.074 x 0.038| **124**           |

Across all eight probed debug seeds only the ketchup bottle, the orange slab
and the basket move at all, and only by about +-5 mm. The layout is otherwise
frozen.

## Which prop is the butter

Three independent lines, none of them a position prior (this is a `_pos` cell,
and the demo layout is *not* the debug layout — the demo's grasp slot is
occupied by the ketchup bottle on every debug seed, so demo xy is a decoy):

1. **Grasp height.** All three pack demos close the gripper at eef
   z = 0.0095 / 0.0100 / 0.0111, with the table at 0.001. Only a slab a
   couple of centimetres tall can be grasped there; every bottle/carton/can in
   the scene is 0.08-0.15 tall. So the target is one of the two slabs.
2. **Appearance.** The pack keyframes show two small boxes; the grasped one is
   ORANGE with a dark top band, the untouched one is larger and BROWN. The
   frame where the prop hangs over the basket (demo0 t=150) shows the same
   orange body.
3. **Warmth margin.** Of the two slabs on the debug seeds, one scores 41 on
   R-B (p85) and the other 124. No overlap, on any seed.

=> butter = the flat, strongly warm slab. Rule: among components with
top < 0.045, take the largest R-B percentile.

## Version log

### v1 — perception probe (no motion)
Hypothesis: the scene must be read before anything is aimed at.
Evidence: dumped cam_high RGB+depth for seeds 51,53,...,65 through chunked
`api.log`; produced the table above. `results/fs_c2clean_obj_butter_pos_k3_v1`,
0/8 by construction (the program commands no motion).
Verdict: scene mapped; identity rule derived.

Gotcha found offline: a pure colour robot-mask (G or B exceeding R) also eats
the butter's blue top band and halves its cluster, moving the top-face
centroid 18 mm toward the camera. Gating the mask by z > 0.16 (the tallest
prop is 0.148, the arm hangs from 0.26) fixes it; the cluster then measures
0.074 x 0.038 x 0.019 on every seed and the top-face centroid is stable to
+-3 mm.

### v2 — perceive, top-down grasp, carry, drop
Hypothesis: the demos' absolute grasp height transfers (same prop, same
table), so perceived xy + GRASP_Z = 0.010 with a straight-down wrist is
enough; release over the perceived basket rim centre at z = 0.20.
Evidence: `results/fs_..._v2`, 0/8 — but only because this fair server's
FairApi has no `move_path` (`AttributeError`) and the program died at the
transport. Everything before that worked on all 8 probed seeds: the perceived
butter was the orange slab every time, and `after close` / `after lift` both
reported effort 3.0 with width_m 0.0389 — exactly the slab's measured 38 mm y
span, i.e. the jaws are on the object and holding through the lift.
Verdict: perception + grasp confirmed; drop `move_path`.

### v3 — same, with the transport as three `move()` calls
Hypothesis: the only defect in v2 was the unavailable primitive.
Evidence: probe `results/fs_..._v3` **8/8** (seeds 51,53,...,65); formal
`results/sel_..._v3` **15/15**, 189-195 sim steps, held=True at every
checkpoint including over the basket.
Verdict: works. Kept as a candidate.

### v5aim014 / v5aim022 — aim-envelope probes (measurement, not candidates)
Hypothesis: 15/15 says nothing about margin; displace the grasp aim along the
tight axis (y, across the jaws) until it breaks.
Evidence: v3 + 14 mm of deliberate y error -> **8/8**; v3 + 22 mm -> **0/8**.
The landed y tracks the command to under 1 mm in both, so those are real
object-relative errors of +14.7 mm and +21.6 mm.
Verdict: the y envelope ends at about 20 mm, which is exactly the geometric
half-slack (jaws 77.8 mm, slab 38.9 mm -> 19.5 mm). Seed-to-seed scatter of
the perceived butter centre is +-3 mm, so the frozen program runs at roughly
6x margin on its tightest axis.

### v4 — staged approach (via the carry height) + one re-grasp retry
Hypothesis: v3 arrives at the hover pose straight from home and still carries
velocity, so the descent overshoots; staging the approach over the target
first should make the descent track, and a sensor-gated retry costs nothing
when the first grasp holds.
Evidence: formal `results/sel_..._v4` **15/15**, 206-215 sim steps, retry
never triggered on any seed. Tracking did improve: v3 lands +8.3 mm off in x
and stalls at z = 0.016-0.018 (stopped on the slab), v4 lands -11.0 mm off in
x and reaches the commanded z = 0.0094-0.0098. y tracking is +-0.7 mm in both.
At the +22 mm aim offset that broke v3 0/8, v4 scores **8/8**
(`results/fs_..._v5aim022r`).
Verdict: **frozen**. Note the honest mechanism: at +22 mm the retry did NOT
fire — v4 wins there because its staged approach lands 2 mm short of the
command (+19.7 mm true error instead of +21.6 mm), which falls just inside the
19.5 mm half-slack. The retry remains an untested safety net; it is kept
because it cannot cost anything (it is gated on effort < 2.0 after the lift)
and there are ~300 sim steps of unused horizon.

## Residual risk on the blind eval band

The debug band's layout is nearly frozen — across seeds 51-65 only the ketchup
bottle, the butter and the basket move, and only by about +-5 mm. If seeds
1-50 perturb positions much harder, the exposure is:
  * two props coming within 12 mm of each other would fuse into one grid
    component (CELL = 0.012). The butter would then read as a large warm
    component and the grasp would aim between the two.
  * a prop landing under the arm's masked silhouette would be lost entirely.
Neither is defended against, because neither occurs on any debug seed and a
defence tuned on scenes I cannot see would be guesswork. The identity rule
itself (flattest-and-warmest) has a 3x colour margin (124 vs 41) and does not
depend on position at all, which is the part that matters for a `_pos` cell.

## DECLARATION

* Frozen version: **v4**. `packs/c2clean_obj_butter_pos_k3/program.py`
  md5 `4a0c147b34c9a2dba27cf7756bf96c0d` == `program_v4.py` (verified on the
  cluster).
* Selection receipt (full 15 debug seeds, one formal run):
  **15/15** — `results/sel_c2clean_obj_butter_pos_k3_v4`.
* Receipt chain:
  | version | run | seeds | score |
  |---------|-----|-------|-------|
  | v1 probe | `fs_..._v1` | 8 | 0/8 (no motion by design) |
  | v2 | `fs_..._v2` | 8 | 0/8 (`move_path` unavailable) |
  | v3 | `fs_..._v3` | 8 | 8/8 |
  | v3 | `sel_..._v3` | 15 | 15/15 |
  | v4 | `sel_..._v4` | 15 | **15/15** (selected) |
  | v3 +14 mm aim | `fs_..._v5aim014` | 8 | 8/8 (envelope) |
  | v3 +22 mm aim | `fs_..._v5aim022` | 8 | 0/8 (envelope) |
  | v4 +22 mm aim | `fs_..._v5aim022r` | 8 | 8/8 (envelope) |
* PROVENANCE: present, 19 entries, all `allowed: True`; `scan_program(...,
  "eval")` passes and `_reads_done` is False.
* No `tools/fewshot_run.py` was invoked; no eval seed (1-50) was touched; no
  forbidden asset was opened.
