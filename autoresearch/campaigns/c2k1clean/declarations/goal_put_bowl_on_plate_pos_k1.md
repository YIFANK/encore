# c2k1clean / goal_put_bowl_on_plate_pos_k1

Intent: "put the bowl on the plate".  K=1 pack, _pos perturbation, debug seeds
51-65, eval seeds 1-50 (blind, coordinator-run).

## What the pack says

`pack.json` has one 90-step demo, stride 10, four keyframes:

| t | ee xyz | grip cmd | finger state | gap |
|---|--------|----------|--------------|-----|
| 0 | -0.2144, 0.0079, 1.1560 | -1 (open) | 0.0362/-0.0362 | 0.0724 |
| 32 | -0.0943, 0.0335, 0.9237 | +1 (close) | 0.0392/-0.0393 | 0.0785 |
| 76 | 0.0448, 0.0121, 0.9359 | -1 (open) | 0.0058/-0.0053 | **0.0111** |
| 89 | 0.0512, 0.0156, 0.9893 | -1 | 0.0386/-0.0388 | 0.0774 |

The load-bearing fact is the closed finger gap of **0.0111 m**.  That is far
thinner than any bowl, so the demo's grasp is a **rim pinch**: one jaw inside
the bowl, one outside, straddling the wall.  The demo xy is a decoy under the
_pos perturbation; only the mechanism transfers.

## Scene, re-derived from debug-seed RGB-D (cam_high, all 15 seeds)

Perception dump probe (v0) logged zlib+base64 RGB-D through `api.log`
(chunked at 1900 chars; the client truncates a log line at 2000).

* camera is fixed across seeds; table z = **0.9005** in 15/15.
* **bowl**: the only prop whose top lands 0.0516-0.0517 m above the table.
  Kasa circle fit on its rim ring: r = 0.0541-0.0545 in 15/15.  Height band
  [table+0.040, table+0.070] isolates it every time.
* **plate**: top ring 0.0199-0.0201 above the table, cell bbox 0.140 x 0.140.
  A plain height band fuses it with the blue matchbox (seeds 56/61/62/64/65)
  or with the cabinet base (52/53/54/63).  Cell mean RGB separates them
  cleanly: plate 138/131/128, matchbox 71/76/91, stove 80/80/80, red board
  92/70/61.  Band + brightness>110 + a keep-out ring around the bowl gives
  ex=ey=0.140 in 15/15.
* bowl radial profile (seed 58): inner surface 0.0453 at 20 mm below the rim,
  outer ~0.0575 at the rim -> wall midline ~0.050 at the grasp depth.

## Controller mechanics (measured, program_vh / vf)

* episode step budget ~**1120 env steps** (vh: saturated 60-step moves froze
  during the 20th).
* saturated travel ~**0.0095 m/step** in free space high above the table, but
  an order of magnitude slower for -y moves at carry height near the table.
* `api.move` returns inside its own 12 mm tolerance; cancelling the measured
  error in the command ("goto") converges to <3 mm in 2-3 tries.
* open-gripper fingertips stall **0.0086 m** above the table (vf front column),
  i.e. the tips are essentially at the eef z.

## Version log

### v0 / v0b / v0c -- perception dump, no motion
Receipt: 15/15 debug seeds dumped; all geometry above.

### v1 -- rim pinch, lift to rim+0.12, translate, descend, release
Receipt: **1/8** (51,53,55,57,59,61,63,65; only 57).
Grasp worked everywhere (gap 0.0078-0.0083, effort 3.0).  Every episode froze
at the same state mid-PLACE: the step budget ran out.  Path was ~0.86 m of
max-axis travel plus 3-try gotos at seconds=1.5.

### v2 -- same, but carry at rim+0.03 and release without a descent
Receipt: **2/8** (57, 61).  Reaches END, so the budget is fixed.  New failure:
the OVER move stalls ~50 mm short **in -y** (target y -0.09, reached -0.038)
even with the cap unspent elsewhere.  Confirmed by vr/vf: the arm reaches -y
badly at carry height with a downward wrist.

### v3 -- pinch the **+y** arc of the rim instead of the -y arc
Hypothesis: the release point is plate_y + GRASP_R, so pinching the far arc
moves it from y=-0.09 (unreachable) to y=+0.005 (easy).
Receipt: **7/8** -- 51,53,55,57,59,61,63 true; 65 false.
dir `results/fs_c2k1clean_goal_put_bowl_on_plate_pos_k1_v3`.
Only ep65 failed; its eef drifted +8.6 mm in x between OVER and RELEASED
because `goto` leaves the controller setpoint on the bias-cancelled command.

### v4 -- v3 + setpoint re-anchor before the release + post-place self-check
(running)
Receipt: **7/8** probe (fail 65).  The setpoint re-anchor was a no-op: `api.move`
returns without stepping when it is already inside POS_TOL, so it never
refreshes the controller's desired pose.

### v5 -- seat the bowl instead of dropping it
Command a target 8 mm below the plate rim so the descent is contact-limited.
Receipt: **13/15** over the whole debug band
(`..._v5` seeds 51,53,..,65 = 7/8; `..._v5b` seeds 52,54,..,64 = 6/7;
fails 64 and 65).
Landing analysis (bowl centre = release eef + (0,-GRASP_R), compared with the
plate centre): x offsets were +0.6, +5.1, +5.7, +5.5, +6.2, -12.2, +5.5, +6.4,
+5.7, +6.6, +5.4, -11.8, -0.8, **+9.3**, **+8.5** mm -- the two failures are the
two largest +x landings, and the median landing is +5.5 mm in +x.
y offsets averaged -0.7 mm, so GRASP_R = 0.050 is the right carry offset.

### v6 -- v5 with PLACE_DX = -0.0055 (cancel the measured +x landing bias)
Receipt: **14/15**, dir `results/fs_c2k1clean_goal_put_bowl_on_plate_pos_k1_v6`,
only seed 64 fails.  Landing x offsets collapse to |dx| <= 1.7 mm on 10 of 15
seeds; the rest land -10 to -18 mm and still succeed, so the -x side has plenty
of margin.

### v7 -- v5 with a settled fine-correction before the seat (no bias cancel)
Receipt: **13/15** (fails 64, 65).  Does not help on its own.

### seed 64 forensics
Its plate fit is visually exact (overlay `ov64`), and under v6 it lands at
dx +2.8 mm / dy -1.1 mm -- squarely on the plate by my own estimate -- and
still scores false.  In seed 64 the blue matchbox abuts the plate (their
height-band cells are contiguous, n=460 vs ~300 for a clean plate), so the
bowl's rim lands over the box.  Held-bowl re-perception is not available as a
cross-check: at the carry pose the bowl is almost entirely occluded by the
gripper in cam_high (dump `held65`).

### v8 -- v6 + settled fine-correction before the seat
Receipt: **13/15** (fails 64, 65).  The extra settle/correct costs more than it
buys; dropped.

### seed 64, measured
Post-place dump with the arm retreated (`h64`, band [table+0.050, +0.095]
isolates the settled bowl as a single component): bowl at (-0.1925,-0.0375),
top **0.0791** above the table.  The plate centre was (-0.1875,-0.0475), so the
bowl landed 11 mm off, mostly in +y, and its top is 7 mm higher than a bowl
seated flat on the plate rim would be (0.0517 + 0.020 = 0.0717) -- it is
perched on the plate's rim, not sitting in the dish.  So the residual failure
mode is the carried bowl slipping in the thin pinch, not the plate estimate.

### v13 -- v6 + post-place verify and re-centre
LIBERO latches `task_success`, so a corrective re-place can only add episodes;
it cannot undo one already scored.  After the release the arm retreats to
table+0.20 (which crops out of the cloud), `find_bowl` locates the settled bowl
by its rim ring, and if it is >8 mm from the aim point or its top is above
table+0.074 the program re-grasps and re-places it.
Receipt: **14/15**, fails 64.  The retry did fire on 64 but used the wrong
hang (measured from the perched bowl's top, 23 mm too much), so the re-place
dropped it from too high.

### v14 -- v13 with the hang taken from the bowl's own height, up to 2 retries
Receipt: **14/15** (fails 64).  On 64 the retry re-centred the bowl from
11.6 mm off to 1.9 mm off -- and it still scored false, with the settled bowl's
top at 0.0779 (base 0.0262, i.e. 6 mm above the plate rim).  In seed 64 a dark
prop 0.0265 high sits 0.050 m from the plate centre, inside the bowl's own
0.0575 radius, so a centred bowl rests on it rather than in the dish.

### v9 (SEAT_PRESS 0.016) / v10 (PLACE_DX -0.0110)
Aim-envelope probes.  Receipt: **14/15** each (fails 64).  The -x side tolerates
at least 18 mm of landing error, so the placement has real margin.

### v15 -- v14 + an aim nudge away from the nearest low dark prop
Receipt: **14/15** (fails 64).  Neither helps nor hurts; dropped as a
one-seed heuristic with no measured payoff.

### v16 -- v14 + a sanity guard on the settled-bowl reading
Skips the retry when the reading is not bowl-sized (40-320 cells) or is more
than 0.12 from the aim, which suppresses the spurious retries that fire on
episodes already terminated by their own success.  Receipt: **14/15** (fails
64), dir `results/sel_c2k1clean_goal_put_bowl_on_plate_pos_k1_v16`.

### v17 -- v16 with the full PROVENANCE dict and docstring (FROZEN)
Byte-identical to v16 outside the docstring and PROVENANCE (verified by diff
of the stripped sources), so its behaviour is v16's.

## Mechanism-gap note on seed 64

Falsifiable statement: seed 64 fails because a low dark prop whose top is
0.0265 m above the table sits 0.050 m from the plate centre -- inside the
bowl's 0.0575 m rim radius -- so any bowl centred on that plate rests on the
prop instead of in the dish, and the benchmark predicate does not fire.
Receipt on debug seeds: under v14 the corrective re-place put the bowl 1.9 mm
from the plate centre and its measured top was 0.0779 (base 0.0262), matching
the prop's 0.0265 top rather than the 0.020 plate rim; six independent
variants (v6, v9, v10, v13, v14, v15, v16, v17), including ones that shifted
the aim by -11 mm, pressed 16 mm harder, and nudged 7 mm away from the prop,
all scored 14/15 with 64 the only failure.  The missing mechanism is moving
the obstructing prop off the plate first, which no version attempts.

## DECLARATION

* Frozen version: **v17**.
  `packs/c2k1clean_goal_put_bowl_on_plate_pos_k1/program.py`
  md5 `a6c32bc279d8385e33d70008a9a430fb`, identical to
  `program_v17.py` and to the run's `program_archived.py`.
* Selection receipt: **14/15** on the full 15 debug seeds (51-65),
  dir `results/sel_c2k1clean_goal_put_bowl_on_plate_pos_k1_v17`;
  only seed 64 fails.
* PROVENANCE: present, 26 entries, every entry sourced to this pack, a
  debug-seed measurement, or generic controller/camera mechanics.
* Per-version receipt chain (all on debug seeds only; eval seeds 1-50 never
  touched, `--split debug` throughout):

  | ver | change | seeds | score |
  |-----|--------|-------|-------|
  | v0/v0b/v0c | RGB-D dump, no motion | 51-65 | perception only |
  | vh | step-budget probe | 51 | ~1120 steps, 0.0095 m/step |
  | vr / vf | reach + descent-column probes | 51 | fingertips 0.0086 above the table |
  | v1 | rim pinch, high carry, descend, release | 8 probe | 1/8 |
  | v2 | low carry, release without descending | 8 probe | 2/8 |
  | v3 | pinch the +y rim arc | 8 probe | 7/8 |
  | v4 | + setpoint re-anchor (no-op) | 8 probe | 7/8 |
  | v5 | seat instead of drop | 15 | 13/15 |
  | v6 | + PLACE_DX -0.0055 | 15 | **14/15** |
  | v7 | settled fine-correction only | 15 | 13/15 |
  | v8 | v6 + fine-correction | 15 | 13/15 |
  | v9 | v6, SEAT_PRESS 0.016 | 15 | 14/15 |
  | v10 | v6, PLACE_DX -0.0110 | 15 | 14/15 |
  | v13 | v6 + verify/re-place | 15 | 14/15 |
  | v14 | v13 with the correct hang, 2 retries | 15 | 14/15 |
  | v15 | v14 + aim nudge off the intruder | 15 | 14/15 |
  | v16 | v14 + retry sanity guard | 15 | 14/15 |
  | **v17** | **v16 + full PROVENANCE (frozen)** | **15** | **14/15** |

STOP.
