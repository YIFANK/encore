# c2k1clean / goal_push_plate_front_stove_task_k1

Intent: **"Push the cream cheese to the front of the stove"**
bddl (opaque string): `push_the_plate_to_the_front_of_the_stove.bddl`
Runner: `tools/fair_run.py` only. Debug seeds 51-65. No note file.

## Evidence read from the two packs

| pack | language | what it gives |
|---|---|---|
| `..._task_k1` | "push the plate to the front of the stove" | the **target**: two keyframes (t=0, t=154) that bracket the plate's motion, plus a 155-step ee_path6 |
| `..._task_mate` | "put the cream cheese in the bowl" | the **object**: grasp height and closed gap for the cream cheese |

Mate pack keyframe decode (`gripper_cmd` -1 = open, +1 = close, read off the
`gripper_state` flips): it closes on the cream cheese at t=40 with
`ee = (-0.0213, 0.1119, 0.9104)`, and at t=75 the carried state is
`gripper_state [0.0216, -0.0213]` — a **0.043 m closed gap**. So the object my
intent names is graspable top-down at **eef z = 0.9104**, closing across its
0.042 m width. (The mate pack's *xy* is a decoy from another scene; only the
height and the gap transfer.)

Task pack keyframes are 128x128 renders. Deprojecting them through my own
debug-seed camera model (cam_high K + `t_base_cam`, table plane z = 0.900):

* `demo0_t0000.png` plate face centre → **(0.030, -0.012)**; my own measured
  plate centre on seed 51 is **(0.030, 0.000)**. Pack scene and my scene share
  a frame to ~3 mm, so absolute deprojections from the pack images are usable.
* `demo0_t0154.png` plate face centre → **(-0.052, 0.196)**. That is a site the
  task's own predicate accepted.

## Scene measured on debug seeds 51/53/55/57 (cam_high RGB-D, 2x subsampled)

| cluster | npx | x range | y range | z top | blue-minus-warm |
|---|---|---|---|---|---|
| cabinet | 12.8k | [-0.39, 0.16] | [-0.35, -0.12] | 1.206 | -0.029 |
| arm/base | 3.2k | [-0.27, -0.15] | [-0.10, 0.10] | 1.364 | **+0.076** |
| **stove** | 2.1k | [-0.45, **-0.161**] | [0.117, 0.307] | 0.959 | 0.000 |
| bowl | 591 | [-0.14, -0.03] | [-0.06, 0.05] | 0.951 | -0.008 |
| plate | 423 | [-0.02, 0.12] | [-0.07, 0.06] | 0.919 | -0.030 |
| bottle | 334 | [-0.21, -0.18] | [-0.07, -0.03] | 1.057 | -0.020 |
| **cream cheese** | 254 | 0.078 long | 0.042 wide | 0.919 | **+0.078** |

Identity rule: **colour needs a height gate here** — the arm's own livery is as
blue as the cream cheese (+0.076 vs +0.078); only its 1.364 m top separates
them. So: gate to `ztop < 0.935` (which admits only the plate at 0.919 and the
cheese at 0.919; bowl 0.951 / stove 0.959 / bottle 1.057 all fail), then take
the largest blue-minus-warm. Margin over the plate is 0.078 vs -0.030, a
ranking not a threshold call. A size gate (`npx < 900`) is the second line and
independently excludes arm/cabinet/stove.

Goal site, expressed relative to the perceived stove so it tracks the fixture:
the demo's own stove front edge deprojects to x = -0.184 and its y-mid to
0.193, giving offsets +0.132 / +0.003; against my measured stove (front edge
-0.161, y-mid 0.212) the same endpoint gives +0.109 / -0.016. I took the
midpoints: **+0.115 m in front of the stove's front edge, -0.010 m from its y
midpoint** (→ (-0.046, 0.202) on seed 51).

## Version log

### v1 — perception dump, no motion
*Hypothesis*: none; a zero-motion probe that zlib+base64s cam_high RGB-D
through `api.log` so all measurement happens offline at zero sim cost.
*Evidence*: `results/fs_..._v1`, seeds 51/53/55/57 → 0/4 (expected; it never
moves). Produced the table above; confirmed table top z = 0.900.
*Verdict*: scene fully measured. Cheese and stove are both identifiable
without importing any constant from outside this cell.

### v2 — grasp the cheese, carry it to the demo's endpoint
*Hypothesis*: the predicate is a region check on the cream cheese, so setting
it down where the task pack's plate ended satisfies it; a top-down grasp at the
mate pack's own grasp height is more precise than replaying the demo's
press-and-drag with a much smaller object.
*Evidence*: `results/fs_..._v2` 8/8, `results/sel_..._v2` **15/15**. Grasp
receipt on every seed: closed width **0.0422** against the pack's 0.043 gap,
effort 3.00, `held=True`.
*But*: `sim_steps` was **129-133** on all 15, and the logged `PLACE` residual
was 0.0773 with the eef frozen at z = 0.987. **The carry never ran** — LIBERO
terminated the episode a few steps into the lift. v2 scores 15/15 on a
mechanism I had not intended and had not isolated.
*Verdict*: 15/15, but not yet understood. Do not freeze on this.

### v2ctl — idle control (not a candidate)
*Hypothesis*: the initial scene already satisfies the graded predicate and any
stepping ends the episode.
*Evidence*: 24 × `settle(0.5)` = **792 sim steps**, no motion at all, seeds
51/53/55/57 → **0/4**.
*Verdict*: **refuted.** The initial state is not a success, and the horizon is
at least 792 steps (so v2's 131 really was an early termination).

### v3diagA — same approach-and-close over bare table (not a candidate)
*Hypothesis*: the bit is fired by the arm's motion, not by the cream cheese.
*Evidence*: the identical approach + close + lift + wait, aimed 0.10 m in +x of
the cheese where seed 51 shows bare table, ran its **full 362 steps** on seeds
51/53/55/57 → **0/4**. Gripper closed to width 0.0010, effort 0.05 (air).
*Verdict*: **refuted.** The cream cheese is what fires the bit.

**Mechanism, as far as debug seeds can settle it**: with `move(2.0s)` ≈ 40 sim
steps and `settle(0.4)` ≈ 26 (both calibrated against v3diagA's 362 and
v2ctl's 792), v2's 131 steps land a dozen steps into the lift — about 2-3 cm of
rise. So **the bit fires as soon as the cream cheese leaves the table at its
own start pose**, which on every debug seed is already in front of the stove
(x ≈ -0.04, y ≈ 0.13, against a stove front edge of -0.161 spanning
y ∈ [0.117, 0.307]). This is a *weaker* condition than the one the task pack
demonstrates. I cannot tell from seeds 51-65 — where the cheese always starts
in front of the stove — whether an unseen layout starts it somewhere the lift
alone is not enough.

### v4 — rehearse the reach, grasp, then still deliver to the pack's endpoint
*Hypothesis*: keeping v2's proven grasp-and-lift and *then* carrying the object
to the site the pack's own demo ended at strictly dominates v2 — the extra leg
is free whenever the lift already sufficed (the episode has ended), and is the
only thing that helps if it did not.
*Design consequence*: because the carry can never execute on a debug seed, its
reachability is verified **before** the grasp, while the episode is still live:
the empty gripper is walked from the cheese hover to the goal hover and back,
and the residual is a per-episode receipt. If the site does not answer
(`residual ≥ 0.02`) or nothing is in hand, the program holds rather than
dragging the jaws across the scene on a guess. It then presents the object
*airborne* over the goal (settle) before seating it, covering both a region
with a z-floor and a table-level one.
*Evidence*:
  * `results/fs_..._v4` seeds 51,53,…,65 → **8/8**. Rehearsal receipt on
    seed 51: `REHEARSE residual=0.0103 eef=[-0.039, 0.196, 0.995]
    goal_reachable=True` — the goal site is reachable under the controller.
  * `results/sel_..._v4` full 15 debug seeds → **15/15**, sim_steps 140-147.
  * Envelope probe `results/fs_..._v4env` (grasp aim deliberately displaced by
    +0.015 m in x and +0.012 m in y, seeds 51,53,…,65) → **8/8**, closed width
    still 0.0422 on every seed. The grasp is not living on its aim tolerance.
*Verdict*: **frozen.** Same 15/15 as v2 with the mechanism isolated, the
transport verified live, and a measured aim margin.

## Honest limitation

The graded bit fires on the lift on all 15 debug seeds, so the delivery leg of
v4 is *never exercised* there — its evidence is the pre-grasp rehearsal
residual (0.0103 m, reachable) and the fact that the same `api.move` primitive
places the eef to ~0.01 m everywhere else in the program, not an end-to-end
success. If eval seeds 1-50 start the cream cheese in front of the stove as
every debug seed does, v4 scores exactly what v2 scores; the delivery leg only
matters if some eval layout starts it elsewhere, and there it is the reason to
prefer v4.

---

# DECLARATION

* **Frozen version**: `program_v4.py`
  `packs/c2k1clean_goal_push_plate_front_stove_task_k1/program.py`
  md5 **`924ec7cad8754548a6ed8a4ccd106a12`** == `program_v4.py` md5
  == `results/sel_c2k1clean_goal_push_plate_front_stove_task_k1_v4/program_archived.py` md5.
* **Selection receipt (full 15 debug seeds, one formal run)**: **15/15**
  `results/sel_c2k1clean_goal_push_plate_front_stove_task_k1_v4`
  (seeds 51-65, every `"benchmark_success": true`, sim_steps 140-147).
* **Receipt chain**:

  | version | run dir | seeds | result |
  |---|---|---|---|
  | v1 perception dump | `fs_..._v1` | 51,53,55,57 | 0/4 (no motion, by design) |
  | v2 grasp + carry | `fs_..._v2` | 8 probe seeds | 8/8 |
  | v2 grasp + carry | `sel_..._v2` | 51-65 | 15/15 |
  | v2ctl idle control | `fs_..._v2ctl` | 51,53,55,57 | 0/4 over 792 steps |
  | v3diagA bare-table control | `fs_..._v3diagA` | 51,53,55,57 | 0/4 over 362 steps |
  | **v4 (frozen)** | `fs_..._v4` | 8 probe seeds | **8/8** |
  | **v4 (frozen)** | **`sel_..._v4`** | **51-65** | **15/15** |
  | v4env displaced aim | `fs_..._v4env` | 8 probe seeds | 8/8 |

* **PROVENANCE**: present as a top-level literal dict in `program.py`, covering
  `TABLE_Z`, `GRASP_Z`, `HOLD_GAP`, `GOAL_DX_FROM_STOVE_FRONT`,
  `GOAL_DY_FROM_STOVE_MID`, `CHEESE_TOP_MAX`, `OBJ_BAND_Z`, `WS_BOUNDS`,
  `CARRY_Z`, `REHEARSE_OK_RESIDUAL`, `PLACE_CLEARANCE`, `R_DOWN` — every one
  sourced to a named pack field or to a debug-seed measurement of my own.
* **Clean room**: no `.bddl`/`.xml`/`.hdf5`/init_states/gt_trace read; no other
  campaign's artifacts read; no other pack's `program*.py` or `NOTES.md` read;
  `tools/fewshot_run.py` never invoked; `api.done` never read (AST-checked
  before upload: 0 `.done` attribute reads). Cluster writes confined to
  `packs/c2k1clean_goal_push_plate_front_stove_task_k1/*` and
  `results/*c2k1clean_goal_push_plate_front_stove_task_k1*`.

STOP.
