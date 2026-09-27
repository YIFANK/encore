# c2k1clean / goal_put_cream_cheese_in_bowl_task_k1 — "put the wine bottle in the bowl"

Runner: `tools/fair_run.py` only. Splits sealed: every run below is `--split debug`
on seeds 51-65. Seeds 1-50 never touched.

## What the two packs each gave

- **mate** (`..._task_mate`, "put the wine bottle on top of the cabinet") — the OBJECT
  half. Its `ee_path6` closes the gripper (`gripper_cmd` -1 → +1) at eef
  z = **1.0223** with a held finger gap of 0.0165 in `gripper_state`. That is the
  only quantitative thing I took from it, and it is the whole grasp.
- **k1** (`..._task_k1`, "put the cream cheese in the bowl") — the TARGET half. It
  releases over the bowl from eef z ≈ 0.959, i.e. ~60 mm above the table, which
  told me the target is a shallow vessel you drop into rather than a surface you
  set down on.

Neither pack's xy was used: both demo scenes differ from my seeds, and everything
positional is re-perceived per episode.

## What I measured myself (debug seeds 51-65, cam_high RGB-D)

Probe v0 logged zlib+base64 RGB-D through `api.log` (note: `api.log` truncates a
message at 2000 chars, so chunks are 1900) and I deprojected offline.

| quantity | measurement | spread over 15 seeds |
|---|---|---|
| table z | 0.9010 | identical |
| bottle top z | 1.0587-1.0588 | identical |
| bottle neck diameter (z ≥ 1.005) | 0.014-0.017 | — |
| bottle body diameter (z ≤ 0.955) | 0.043 | — |
| bottle xy | (-0.184..-0.206, -0.038..-0.065) | ±11 mm |
| bowl outer diameter | 0.109-0.112 | — |
| bowl rim z / interior floor z | 0.9512 / 0.9074 | identical |
| bowl xy | (-0.089..-0.116, -0.016..+0.014) | ±14 mm |

Two facts make the perception trivial once measured: the **bottle is the only narrow
(span < 0.06) component standing 90-210 mm above the table** — the band's upper edge
cuts off the parked arm, whose lowest geometry is at 1.111 — and the **bowl is the only
round component (aspect 0.97-1.01, diameter 0.11) in the 25-90 mm band**; the stove
slab (0.17), the cabinet strip (0.158 × 0.04) and the plate (below the band) all fail.

`api.ground`, `api.sam3` and `api.vqa` are all absent on the LIBERO backend
(`AttributeError` / `unknown op`), so this is pure depth geometry.

## Version chain

| version | hypothesis | run | verdict |
|---|---|---|---|
| v0 | perception probe, no motion | `fs_..._v0`, 15 seeds | dumped RGB-D; established every number in the table above |
| v1 | pinch the neck at the mate pack's z, carry, lower until the base is 10 mm above the bowl floor, release | `fs_..._v1`, seeds 51,53,…,65 | **8/8** |
| v2 | v1 + park at the arm's start pose and re-perceive | `fs_..._v2`, same 8 | **8/8**; re-perception found no bottle |
| v3 | v2 but park at (-0.2085, +0.15, 1.20) to clear the camera's line of sight | `fs_..._v3`, same 8 | **8/8**; re-perception *still* found no bottle |

**Why the re-perception never fires, and what I used instead.** v3's park pose is a
no-op: LIBERO terminates the episode the moment the predicate fires, and
`_step_env` returns immediately once terminated, so on a successful episode the arm
freezes where it released and every later move is discarded. Post-release
re-perception is therefore structurally unavailable on exactly the episodes I would
want it for. (I did not read `api.done`; this is the physical consequence, visible
in the frozen eef.) The verification I do have:

- **Grasp is on the neck, not on air:** closed gap = **0.0150 m on all 15 seats**,
  which is the measured neck diameter (0.014-0.017), with effort 3.0.
- **Nothing slips in flight:** the gap is still 0.0150 at effort 3.0 after the lift,
  after the traverse and at the seat, on every seed.
- **The seat converges rather than jamming:** seat residual 0.0074-0.0120 m over 15
  seeds (POS_TOL is 0.012), so the bottle is lowered to the commanded height instead
  of being driven into the dish.
- **The placement is visible:** v1/v2/v3 dump the final cam_high RGB through the log.
  Decoded offline, it shows the bottle standing upright inside the bowl.

## Mechanism

1. Perceive: table z from the modal height; bottle = largest narrow component in
   (table+0.09, table+0.21), its xy from the centroid of the top 12 mm cap disc
   (fully visible from above, so no grazing-view bbox bias); bowl = largest round
   component in (table+0.025, table+0.09), with its interior floor from the 5th
   percentile inside half the radius.
2. Grasp z = the mate pack's 1.0223, clamped into the measured neck band
   [ztop-0.050, ztop-0.022]. On every seed the clamp is inactive — 1.0223 is
   ztop-0.036, mid-neck.
3. Open, hover at 1.15, descend, close. The base offset (how far the bottle's base
   hangs below the grip) is `grasp_z - table` = 0.1213, exact because the bottle was
   standing on the table when it was grasped.
4. Carry at 1.20 (base at 1.079, above every measured obstacle), traverse to the bowl
   xy, lower to `bowl_floor + 0.010 + base_offset` so the base enters the dish, open.

The margins are wide in both directions: the jaws close from 78 mm onto a 15 mm neck,
and the bowl's 45 mm interior radius against the bottle's 21 mm base radius leaves
~24 mm of xy slack, against an observed placement error of ~5 mm.

---

# DECLARATION

- **Frozen version:** `packs/c2k1clean_goal_put_cream_cheese_in_bowl_task_k1/program.py`,
  md5 `a60715d29bf1d048e929f25e01c3ce92` == `program_v3.py` (same md5, verified on the
  cluster).
- **Selection receipt (full 15 debug seeds, one formal run):** **15/15**
  `benchmark_success: true` in
  `results/sel_c2k1clean_goal_put_cream_cheese_in_bowl_task_k1_v3`
  (seeds 51-65, `--split debug`).
- **Per-version receipt chain:** v0 perception probe (`fs_..._v0`, 15 seeds, no motion);
  v1 8/8 (`fs_..._v1`); v2 8/8 (`fs_..._v2`); v3 8/8 (`fs_..._v3`) on seeds
  51,53,55,57,59,61,63,65; v3 15/15 formal (`sel_..._v3`).
- **PROVENANCE:** present as a top-level literal dict in `program.py`, 18 entries, each
  sourced either to the named packs' `pack.json` or to a debug-seed measurement above.
- **Archived versions:** `program_v0.py`, `program_v1.py`, `program_v2.py`,
  `program_v3.py` in the pack directory.
