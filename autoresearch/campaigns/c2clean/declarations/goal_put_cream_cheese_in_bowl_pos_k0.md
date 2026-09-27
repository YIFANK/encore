# c2clean — goal_put_cream_cheese_in_bowl_pos_k0

Intent: "put the cream cheese in the bowl". No demonstration pack (k0); everything
below was derived from debug seeds 51-65 under `tools/fair_run.py --split debug`.

## Scene, as measured (debug seeds only)

- Base frame from `t_base_cam` of `cam_high`: +x runs toward the camera (bottom of the
  rendered image), +y runs to the right of the image. Table top modal z = **0.900**.
- Cream cheese: a flat blue-labelled box lying on the table, top face at **z = 0.9195**
  (19.5 mm tall), top-face footprint ~**81 x 41 mm** (x by y). Its y-width (41 mm) is
  the axis the default straight-down gripper closes along.
- Bowl: rim z = **0.9514**, rim radius **0.056 m**, centre near (-0.20, -0.05). It is the
  only prop in the scene that reads as a *ring* in the top-down height map — rim cells at
  table+0.03..0.05 enclosing a hole at table height. The plate and the stove slab are
  solid discs/slabs and are rejected by the hole term.
- Wine bottle stands beside the bowl, top at **z = 1.06** — it sits almost exactly on the
  straight line from the box to the bowl, so the carry must clear it.
- `_pos` jitter across the 15 debug seeds is small: box centre x 0.036..0.062,
  y -0.037..-0.009; bowl centre x -0.204..-0.193, y -0.062..-0.038.

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v0 | dump RGB-D through `api.log` (zlib+base64) so perception can be designed offline | `api.log` truncates a message at ~2000 chars, so 4000-char chunks lost data (base64 padding error) | mechanism found, chunking wrong |
| v0b | same with 1900-char chunks | 8/8 seeds decoded cleanly; scene identified (bowl, bottle, plate, stove, cabinet, blue box) | perception pipeline established |
| v1 | park the arm at (0.20, 0.25, 1.25) to unocclude cam_high | unreachable: residual 0.341, eef drooped to (0.097, 0.217, 0.910) — a saturated far target travels diagonally and ends on the plate | park pose rejected |
| v2 | two candidate park poses A=(-0.10,0.30,1.20), B=(0.05,0.32,1.15) | both converge (residual 0.007 / 0.009); A puts the whole arm outside the prop area | **PARK_XYZ = A** |
| v3 | calibrate the fingertip offset by descending with an open gripper | open-gripper descent onto bare table stalls at eef z = **0.9095** (table 0.900); descending to a target 60 mm *below* reachable burned the whole 1000-step horizon and dragged x by 27-39 mm | fingertip ≈ eef − 0.010; never command an unreachable z |
| v4 | straight-line pick and place: grasp at eef z 0.912, carry at 1.12, release 45 mm above the rim | **3/4** (51,53,57 ok; 55 missed — closed to width 0.0014 on air after the descent landed 5.3 mm off the commanded y) | mechanism correct, aim not repeatable |
| v5 | cancel the tracking bias in the command (re-issue each move offset by its own residual) + verify the grasp by gripper width and retry | probe 8/8; formal **15/15** (`results/sel_c2clean_goal_put_cream_cheese_in_bowl_pos_k0_v5`) | selected candidate |
| v6 | v5 with the blue-pixel height ceiling raised tz+0.12 → tz+0.18, because a box resting *in* the bowl reads z = 1.020 and tz+0.12 = 1.0225 was a 2 mm margin; the parked arm's blue link stays above 1.15 so it is still excluded | formal **15/15** (`results/sel_c2clean_goal_put_cream_cheese_in_bowl_pos_k0_v6`) | **FROZEN** |

## What actually made it work

1. **Park before perceiving.** At the home pose the arm lies across the bowl in the
   cam_high view. PARK_XYZ swings it clear without touching anything.
2. **Ring-plus-hole detection for the bowl.** A top-down height map, correlated with an
   annulus kernel (rim cells) and a disc kernel (low interior), then a Kasa circle fit on
   the ring band. This needs no colour and no prior about where the bowl is.
3. **Cancel the tracking bias.** The single biggest failure mode in v4 was lateral: the
   commanded xy is not the achieved xy. Issuing the move, reading the eef, then re-issuing
   the target shifted by the observed error brought the grasp error under ~1 mm.
4. **Verify with the gripper, not with success.** A held box reads width 0.042 with
   effort 3.0; an empty close reads 0.001. That single check drives the retry loop
   (seeds 55 and 63 needed a second grasp attempt: 455/464 sim steps vs 210 for a
   first-try success).
5. **Carry above the bottle.** CARRY_Z = 1.12 clears the 1.06 bottle top that sits on the
   straight path between the box and the bowl.

## Budget

Horizon is 1000 sim steps. A first-try success costs ~210; a one-retry success ~460. The
program allows 3 outer cycles x 2 grasp attempts, which fits.

---

# DECLARATION

- **Frozen version: `program_v6.py`**
  `md5(program.py) == md5(program_v6.py) == 29539f2938186013cba9e06178ba9196`
- **Selection receipt (full 15 debug seeds 51-65): 15/15**
  dir `results/sel_c2clean_goal_put_cream_cheese_in_bowl_pos_k0_v6`
- **Receipt chain**
  - v0  — `results/fs_c2clean_goal_put_cream_cheese_in_bowl_pos_k0_v0` (4 seeds, perception dump, chunking bug)
  - v0b — `results/fs_c2clean_goal_put_cream_cheese_in_bowl_pos_k0_v0b` (8 seeds, perception dump OK)
  - v1  — `results/fs_c2clean_goal_put_cream_cheese_in_bowl_pos_k0_v1` (8 seeds, park unreachable)
  - v2  — `results/fs_c2clean_goal_put_cream_cheese_in_bowl_pos_k0_v2` (2 seeds, park A/B both converge)
  - v3  — `results/fs_c2clean_goal_put_cream_cheese_in_bowl_pos_k0_v3` (3 seeds, fingertip calibration)
  - v4  — `results/fs_c2clean_goal_put_cream_cheese_in_bowl_pos_k0_v4` (4 seeds, **3/4**)
  - v5  — `results/fs_c2clean_goal_put_cream_cheese_in_bowl_pos_k0_v5` (8 seeds, 8/8);
          `results/sel_c2clean_goal_put_cream_cheese_in_bowl_pos_k0_v5` (15 seeds, **15/15**)
  - v6  — `results/sel_c2clean_goal_put_cream_cheese_in_bowl_pos_k0_v6` (15 seeds, **15/15**)
- **PROVENANCE** present as a top-level literal dict in `program.py`, covering
  PARK_XYZ, GRID, RIM_BAND, PROBE_SPOT, GRASP_Z, CARRY_Z, DROP_CLEAR, HOVER_DZ,
  BLUE_Z_GATE, HOLD_W. Every constant traces to a debug-seed measurement or to generic
  controller/camera mechanics. No pack was supplied and none was read; no benchmark asset
  file was opened; `api.done` is never referenced.
