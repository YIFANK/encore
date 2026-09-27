# c2clean / goal_put_bowl_top_cabinet_pos_k0 — worker notes

Intent: "Put the bowl on top of the cabinet."  No demonstration pack (k0).
Everything below is derived from debug seeds 51-65 only, via `tools/fair_run.py`.

## v0 — perception dump (seeds 51,53,57,61)
Hypothesis: nothing known about the scene; harvest it.
Method: zero-motion program that zlib+base64-dumps cam_high / cam_arm_wrist
RGB-D + intrinsics/extrinsics to **stderr** (api.log truncates at 2000 chars;
`program_ep<seed>.stderr` does not), decoded offline.
Evidence (0/4 success, as designed):

* table plane `z = 0.901`; workspace crop needed (walls deproject to x ≈ -1.99).
* cam_high at base (0.659, 0, 1.610); image u -> base +y, image v -> base +x/-z.
* scene: wooden rack, wine bottle, stove slab + knob, a plate, a **bowl**,
  a small blue box, and a dark **cabinet** on the +y side.
* cabinet top = a flat plateau at `z = 1.1276` (0.227 above the table),
  x in [-0.526, -0.274], y in [0.136, 0.323] — same to ~10 mm on all 4 seeds.
* bowl: rim top `z = 0.9514` (50 mm tall), footprint 0.110 x 0.111 -> rim
  radius **0.055**, centre varies per seed (0.047..0.059 in x, -0.025..-0.012
  in y on the probed seeds).
* radial depth profile of the bowl: interior floor 0.907 at r=0, rising
  smoothly to the rim (r 0.036 -> z 0.918, r 0.042 -> 0.930, r 0.048 -> 0.943,
  r 0.054 -> 0.9514) then dropping to the table within one 6 mm bin.
  It is a shallow, smoothly flaring dish with a very thin rim wall.

Verdict: bowl and cabinet are both recoverable from a single cam_high frame by
height-banding + connected components; pick the bowl as the band component with
the largest `min(extent_x, extent_y)` (the bottle and rack slivers lose).

## v1 — first calibration probe (seeds 51,57)
Hypothesis: measure the fingertip offset and the reach to the cabinet.
Evidence: **sim_steps hit 1000 after 4 moves** and every later move silently
no-oped (the harness stops stepping at the LIBERO horizon and `move` returns
instantly with a stale eef). `move_cartesian` caps at
`max(40, 60*seconds*2)` steps, so `seconds=2.0/3.0` burns 240-360 steps each.
Verdict: budget is ~1000 control steps; use `seconds=0.5` (60-step cap)
everywhere. v1's own numbers discarded except the table touch.

## v2 — budget-aware calibration (seeds 51,57)
Evidence (540 sim steps, both seeds agree to ~1 mm):

* a `seconds=0.5` move converges in **one** call; repeating it is a free no-op.
* the controller stops inside `POS_TOL = 0.012`, biased **against** the
  direction of travel: commanded x 0.0585 -> 0.0496, commanded -0.4003 ->
  -0.3898. So a raw command lands ~9-11 mm short. Fix: re-issue with
  `cmd += (target - eef)` (one correction closes it to ~3 mm).
* **fingertip offset**: descent onto bare table (plane 0.901) parks the eef at
  z = 0.9119; descent onto the cabinet top (plane 1.1276) parks it at 1.1374.
  Two independent planes give `tip_z = eef_z - 0.0098..0.011` -> **0.010**.
* the cabinet top centre is reachable: (-0.400, 0.230, 1.24) converged with
  residual 0.011.
* (x,y) ~ (0.20, 0.20) is at the reach boundary — a saturated lateral command
  there drags z down; not used afterwards.

## v3a — strategy A: symmetric "cup" grasp  -> 0/4  (seeds 51,53,57,61)
Hypothesis: descend with jaws fully open (±0.039) around the whole bowl to
11 mm above the table, where the outer radius should be < 0.03, and close on
the outside wall.
Evidence: the descent **stalled at eef z = 0.952** (residual 0.035 -> 0.067):
the open jaws land on the rim, not beside it — the bowl's outer radius at the
rim (0.055) exceeds the open half-gap (0.039), so there is no way down.
Closing then gave `width 0.001, effort 0.05` (empty jaws) and the post-lift
re-perception found the bowl still on the table at its original centre.
Verdict: refuted. A 110 mm dish cannot be straddled by a 78 mm gripper.

## v3b — strategy B: rim pinch  -> 4/4 probe, **15/15 selection**
Hypothesis: put one finger inside the bowl and one outside, straddling the rim
wall. At the default straight-down wrist the tool y axis is base -y, so the
jaws separate along base y: aim the eef at `(cx, cy - 0.052)` and descend to
`tip_z = rim_top - 0.015`.
Evidence (ep51): close -> `width 0.0087, effort 3.0` (holding); lift to 1.25
keeps effort 3.0; carry to the cabinet centre (compensating the 0.052 y offset
between eef and the held bowl centre), lower to
`cab_z + 0.008 + (rim_top - table - 0.015) + tip_offset`, open.
The post-release "retreat" move no-ops (residual 0.076) — the predicate fired
and LIBERO terminated the episode.
Receipts: `fs_..._v3b` 4/4 (51,53,57,61);
`sel_..._v3b` **15/15**, ~215 sim steps per episode (of ~1000).

## Aim-envelope probes (4 seeds each, 51,53,57,61)
Displacing only `RIM_OFFSET` from its 0.052 nominal:

| offset | result |
|--------|--------|
| 0.040  | 4/4 |
| 0.046  | 4/4 |
| 0.049  | 4/4 (and 15/15 formal) |
| 0.052  | 4/4 (and 15/15 formal) |
| 0.058  | **3/4** |

So the working band is roughly `[0.040, 0.055]` and the failure is one-sided
(too far out, the outer finger misses the wall and the pinch grabs air).
`0.049` sits nearest the centre of that band, with ≥ 9 mm of tested margin
inward and ~ 6 mm to the first observed failure outward.

## v3b_o049 — 15/15 selection
Same program with `RIM_OFFSET = 0.049`. Receipt:
`sel_c2clean_goal_put_bowl_top_cabinet_pos_k0_v3b_o049` = **15/15**.

## v4 — FROZEN
Byte-identical to v3b_o049 apart from the module docstring, two PROVENANCE
source strings, and the returned note. Re-run formally on the frozen bytes.

## Mechanism (what the program does)

1. One `cam_high` RGB-D frame -> base-frame cloud -> crop the workspace
   (x ∈ [-0.70, 0.40], y ∈ [-0.80, 0.80]) to drop the walls.
2. **Bowl** = the connected component of the height band
   `table + 0.039 .. table + 0.069` with the largest `min(extent_x, extent_y)`;
   its centre is the bbox centre (the rim ring is symmetric from this view),
   its rim top the 95th percentile height.
3. **Cabinet top** = the largest component of the band
   `table + 0.215 .. table + 0.240`; its plane height and 1/99-percentile
   x/y extent give the drop site.
4. Grasp: hover 60 mm above the rim at `(cx, cy - 0.049)`, descend so the
   fingertips sit 15 mm below the rim top (`eef_z = rim_top - 0.015 + 0.010`),
   close. Receipt of a real grip: `width ≈ 0.008, effort 3.0`.
5. Carry at z = 1.25 (clears every scene structure; the tallest is the rack
   at ~1.14), swinging through `(0.0, y_goal)` first so the payload never
   crosses the bottle or the rack.
6. Place: the held bowl's centre trails the eef by exactly `RIM_OFFSET` in +y,
   so the eef goes to `(cab_x_mid, cab_y_mid - 0.049)` and lowers to
   `cab_z + 0.008 + (rim_top - table - 0.015) + 0.010`, i.e. the bowl's base
   8 mm above the cabinet top. Open, retreat.
7. Every commanded pose goes through `mv()`, which re-issues the command with
   `cmd += (target - eef)` once, cancelling the ~9-11 mm `POS_TOL` tracking
   bias.

## DECLARATION

* **Frozen version**: `packs/c2clean_goal_put_bowl_top_cabinet_pos_k0/program.py`,
  md5 `0cb3d8362ec5fdbfe4159f2a862c68d7`, identical to
  `program_v4.py` (same md5, verified on the cluster).
* **Selection receipt (full 15 debug seeds, 51-65)**:
  `results/sel_c2clean_goal_put_bowl_top_cabinet_pos_k0_v4` = **15/15**
  (`benchmark_success: true` on every episode).
* **Per-version receipt chain**:

  | version | what | seeds | result | dir |
  |---------|------|-------|--------|-----|
  | v0  | perception dump, no motion | 51,53,57,61 | 0/4 (by design) | `fs_..._v0` |
  | v1  | calibration probe (over-budget) | 51,57 | 0/2 (by design) | `fs_..._v1` |
  | v2  | budget-aware calibration | 51,57 | 0/2 (by design) | `fs_..._v2` |
  | v3a | cup grasp | 51,53,57,61 | **0/4** | `fs_..._v3a` |
  | v3b | rim pinch, offset 0.052 | 51,53,57,61 | 4/4 | `fs_..._v3b` |
  | v3b | rim pinch, offset 0.052 | 51-65 | **15/15** | `sel_..._v3b` |
  | v3b_o040 | envelope probe | 51,53,57,61 | 4/4 | `fs_..._v3b_o040` |
  | v3b_o046 | envelope probe | 51,53,57,61 | 4/4 | `fs_..._v3b_o046` |
  | v3b_o058 | envelope probe | 51,53,57,61 | 3/4 | `fs_..._v3b_o058` |
  | v3b_o049 | rim pinch, offset 0.049 | 51-65 | **15/15** | `sel_..._v3b_o049` |
  | **v4** | **frozen (= v3b_o049 bytes)** | **51-65** | **15/15** | `sel_..._v4` |

* **PROVENANCE**: present in `program.py`; every calibrated constant
  (`TABLE_Z`, `TIP_OFFSET`, `RIM_OFFSET`, `PINCH_DEPTH`, band edges, `CARRY_Z`)
  is sourced to a debug-seed measurement listed above. No demonstration pack
  was supplied or used; no foreign or prior-knowledge constant is present.
* No `api.done` read; no `fewshot_run.py`; writes confined to
  `packs/c2clean_goal_put_bowl_top_cabinet_pos_k0/*` and
  `results/*c2clean_goal_put_bowl_top_cabinet_pos_k0*`.

STOP.
