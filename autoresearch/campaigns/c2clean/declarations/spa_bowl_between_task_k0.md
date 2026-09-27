# c2clean / spa_bowl_between_task_k0 — NOTES

Intent: "Pick the akita black bowl **not** between the plate and the ramekin and
place it on the plate". Zero demos. Everything below is derived from debug
seeds 51-65 only.

## Runtime facts re-derived from the runner (tools/fair_run.py, heron/robot/libero.py)

- Episode horizon = **500 sim steps** (fair_run default). `move(rotation=R)`
  costs up to `max(60, 60*seconds)` steps and needs BOTH pos and rot converged;
  `move(rotation=None)` costs up to `max(40, 120*seconds)` and breaks on
  position alone. `grip()` = 20 steps, `settle(s)` = 60*s.
  Once the horizon is spent, every move returns instantly as a silent no-op.
- POS_TOL = 0.012 m, so a move legitimately stops up to 12 mm short.

## v0 — perception probe (no motion). 4 seeds (51,53,55,57)

Shipped cam_high RGB-D off the sandbox as zlib+base64 chunks through `api.log`
and reconstructed locally.

Scene (base frame, identical layout across seeds up to a few-cm jitter — this is
a `_task` cell, so no positional perturbation):

| prop | centre (x,y) ep51 | top above table | note |
|---|---|---|---|
| cabinet + stove | x[-0.45,0.18] y[-0.35,-0.04] | 0.227 | entirely at y<0 |
| bowl A | (-0.167, 0.310) | 0.051 | rim radius 0.054 |
| bowl B | (-0.052, 0.199) | 0.051 | rim radius 0.054 |
| ramekin | (-0.207, 0.192) | 0.043 | rim radius 0.042 |
| plate | ( 0.067, 0.196) | 0.019 | radius 0.064, mean rgb (153,142,139) |
| cookie box | ( 0.076, 0.031) | 0.020 | mean rgb (93,66,46) |

Table plane z = **0.9008** (modal depth of the clear tabletop, all 4 seeds).

Separability results:
- bowls vs ramekin: an 8 mm top-height gap (0.051 vs 0.043) — a height band at
  +0.046 holds the two bowls and nothing else. Needed, because on seed 57 the
  ramekin and bowl B abut and fuse into one XY component.
- plate vs cookie box: same height (0.019 / 0.020) but mean brightness 145 vs
  68 — a mean-channel cut at 110 separates them.

Betweenness test (|P-B| + |B-R| - |P-R|, ep51): bowl B = 0.0001 (exactly on the
plate-ramekin segment), bowl A = 0.1118. Two orders of magnitude apart, so
**target = bowl A**, the +y one.

## v1 — motion calibration. seeds 51,53

Descended a CLOSED gripper onto verified-clear table at (0.15,0.10):
achieved eef z tracked the command (undershoot 8-11 mm) down to cz=0.92
(ez=0.9095), then froze at ez=0.9092 for cz=0.91/0.90/0.89.

- **Fingertips sit 0.0084 m below the eef reference** (0.9092 - 0.9008).
- Descent undershoot is a consistent ~0.010 m; command that much low.
- The rest of the probe ran past the 500-step horizon and returned frozen
  no-ops — the receipt for the budget note above.

## v2 — first pick-and-place  (running)

Hypothesis: in-program band segmentation finds both bowls + ramekin + plate,
the betweenness excess names bowl A, and a +y rim pinch 25 mm below the rim
lifts it onto the plate.

Result: **4/4** on probe seeds 51,53,55,57. This is also the receipt that the
graded predicate accepts the bowl the *re-authored* instruction names (the one
NOT between): every success was scored after placing bowl A.

Selection run (all 15): **14/15**, `sel_c2clean_spa_bowl_between_task_k0_v2`.
Two defects showed up on the seeds the probe subset had never touched:

- **seed 58 (fail).** Grasped and carried correctly, but the transport stopped
  POS_TOL short and the following descent re-aimed and overshot: the bowl centre
  landed 9 mm off the plate centre, which is most of the 10 mm of plate-over-bowl
  clearance, and it was released from 8.7 mm up. Bowl left the plate.
- **seed 56 (success, but by luck).** Only ONE bowl was detected, with an
  inflated r=0.0744 and n=915 — the two bowls had fused into one connected
  component, so the circle fit straddled both. The grasp closed on nothing
  liftable and the grip was lost on the lift (`lifted` width 0.0047, effort
  0.05); the episode then wandered for the full 500 steps and the predicate
  fired anyway. No betweenness test ran at all (one bowl -> no comparison).

## v0b — RGB-D probe of the even debug seeds (52,54,56,58,60,62,64)

Fusion is not rare. At the height-band + connected-component level:
seeds 54, 56, 62 fuse **all three** vessels into one component (n≈5400-5700)
and lose the ramekin entirely; seeds 58, 60 fuse the ramekin with the near bowl.
The `_task` jitter is enough to make the props touch. Any detector that gets
objects from XY connectivity is unreliable here.

## v3 — constrained-radius Hough + closed-loop place

Hypothesis: the rim radius is the stable invariant (0.0537-0.0539 m on every
clean fit), so voting for centres at a FIXED radius separates touching rims —
a point on rim A votes only for centres one bowl-radius from it, and the two
peaks stay distinct however close the bowls sit.

Evidence: validated offline against all 11 probed seeds before spending any
sim time. Two bowls + ramekin + plate recovered on every one, including the
three all-fused seeds. Betweenness excess separates the bowls by 5x in the
worst case (seed 56: 0.0297 vs 0.1497) and by ~300x in the best.

Also: bias-cancel the transport's lateral residual at carry height (before the
descent, so the descent's lateral command is near zero), and release with the
bowl base 3 mm above the plate instead of 8.7 mm.

Verdict: **5/5** on the hard-seed probe 51,54,56,58,62. Both former defects fixed.
But seed 56 still spent 456 of 500 sim steps.

## v4 — guard the bias-cancel, cap the stalls

Hypothesis: on seed 56 the plate sits at +x 0.129, further out than on any
other seed, and the arm saturates against its reach envelope there. A
repeatable stop at the envelope is a reach limit, not tracking slop, so
re-commanding past it cannot help — and v3's bias move indeed made the error
WORSE (residual 0.0169 -> 0.0260) while each stalled move burned its full
`max(40, 120*seconds)` step cap.

Evidence: cancel only residuals inside the tracking-slop band [0.005, 0.015]
(ordinary slop is bounded by POS_TOL=0.012), and shorten every `seconds` so a
stalled move burns a smaller cap. Seed 56 dropped from 456 to 313 sim steps.

Verdict: **15/15**, `sel_c2clean_spa_bowl_between_task_k0_v4`.

---

# DECLARATION

- **Frozen version: v4.** `packs/c2clean_spa_bowl_between_task_k0/program.py`
  md5 `d57c62c193cef59e7c7c16fdb465fcdf` == `program_v4.py` (verified on the
  cluster).
- **Selection receipt (full 15 debug seeds): 15/15**, dir
  `results/sel_c2clean_spa_bowl_between_task_k0_v4`
  (51,52,53,54,55,56,57,58,59,60,61,62,63,64,65 all `benchmark_success: true`;
  149-313 sim steps of the 500-step horizon, so no episode ran near the budget).
- **PROVENANCE present** — 15 entries, every calibrated constant sourced to a
  debug-seed measurement or to generic controller/camera mechanics. The program
  passes `fair_run.scan_program(..., "eval")`, which checks the PROVENANCE dict,
  the forbidden-token scan, and the AST refusal of any `.done` read.
- **No demonstrations were available or used**; no note file. Every constant was
  re-derived from seeds 51-65 in this cell.

## Receipt chain

| ver | what changed | probe | formal 15 |
|---|---|---|---|
| v0 | perception probe, no motion (RGB-D shipped out through `api.log`) | 51,53,55,57 | — |
| v0b | same probe, even seeds — found the fusion problem | 52,54,56,58,60,62,64 | — |
| v1 | motion calibration: fingertip offset 0.0084 m, descent undershoot 0.010 m, 500-step horizon | 51,53 | — |
| v2 | component-fit perception + rim pinch + place | 4/4 (51,53,55,57) | **14/15** |
| v3 | fixed-radius Hough (fusion-robust) + closed-loop place | 5/5 (51,54,56,58,62) | — |
| v4 | bias-cancel guarded to the slop band; shorter step caps | — | **15/15** |

## Method, in one paragraph

Height bands do the identification and a fixed radius does the separation.
Bowl tops sit 0.051 m above the table and the ramekin 0.043 m, so a 0.046 m cut
isolates the two bowls even when all three vessels touch; the plate and the
cookie box share a height (0.019 / 0.020 m) and are split by brightness
(mean rgb 145 vs 68). The bowl the instruction names is the one with the larger
|P-B| + |B-R| - |P-R| — zero on the plate-ramekin segment, and 0.10-0.16 off it.
The grasp is a rim pinch one rim-radius along world y (the axis the fingers
separate along), fingertips 0.025 m below the measured rim top, on the arc
facing away from the other vessels; the place puts the bowl base 3 mm over the
plate, with the carry's lateral bias cancelled first.
