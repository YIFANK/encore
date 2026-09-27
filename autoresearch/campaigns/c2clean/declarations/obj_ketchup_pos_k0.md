# c2clean / obj_ketchup_pos_k0 — working notes

Intent: "Pick up the ketchup and place it in the basket."
K=0: no demonstration pack. Every constant below is re-derived from my own
debug-seed (51-65) observations. No shared note file.

## v1 — perception dump (seeds 51,53) — FAILED AS A PROBE

Hypothesis: I can ship RGB-D off the box through `api.log` and do perception
offline (zero sim steps).
Evidence: `api.log` truncates every message at exactly 2000 characters, so the
zlib+base64 blobs arrived corrupt (`zlib.error: incomplete or truncated
stream`). 0 sim steps used; both episodes `benchmark_success: false` (no motion).
Verdict: technique sound, transport broken. Chunk the payload.

## v2 — chunked perception dump (seeds 51,53,55,57)

Hypothesis: chunking at 1800 chars/line recovers the full frame.
Evidence: 256x256 RGB + float16 depth decoded cleanly on all four seeds.
Deprojection convention confirmed OpenCV (x right, y down, z forward) against
`t_base_cam`: the table deprojects to a flat plane at **z = 0.001**, whereas the
OpenGL sign convention gives a 1.3 m-thick "table". Back wall lands at x ~= -1.99,
so the workspace must be cropped before anything is fitted.

Scene (identical structure on all four seeds; 8 clusters):

| id | what it is | top z | cap dx/dy | mean rgb | redness |
|----|-----------|-------|-----------|----------|---------|
| c1 | robot arm | 0.449 | — | — | — |
| c2 | flat box | 0.140 | 0.005/0.051 | .37 .25 .23 | — |
| c3 | **bottle, grey/silver cap, red body** | 0.148 | 0.026/0.030 | .32 .25 .21 | 0.088 |
| c4 | bottle, green cap | 0.148 | 0.030/0.034 | .24 .27 .23 | -0.011 |
| c5 | **basket** | 0.142 | 0.153/0.170 | .55 .55 .53 | — |
| c6 | bottle, dark-red cap and body | 0.113 | 0.026/0.023 | .23 .11 .04 | 0.161 |
| c7 | can/tin | 0.081 | 0.064/0.069 | .25 .26 .28 | — |
| c8 | small flat box | 0.020 | 0.080/0.041 | .32 .35 .43 | — |

Per-seed jitter is small (<= 13 mm) and only c2, c5, c6, c8 move at all across
51/53/55/57; c3, c4, c7 were bit-identical. The program is nevertheless fully
perception-driven, since the eval band 1-50 is unseen.

Verdict: three narrow-capped bottles are the ketchup candidates. Colour alone
does not name the target — **c6 is redder than c3** — so identity had to be
settled by experiment (below).

## v3 — first tip-offset probe (seeds 51,53) — MISLEADING

Hypothesis: descend a closed gripper onto empty table; the z plateau gives the
fingertip-to-eef offset.
Evidence: plateau at eef z = 0.1009 over a table at 0.001 => "offset 0.100 m".
Both episodes burned the **full 1000 sim steps** on 19 moves.
Verdict: wrong on both counts, and the reason matters. The descent was at
(0.00, 0.10) with `seconds=1.6-2.0`; the arm stalled at 0.101 and **slid +32 mm
in x** while z froze. v5 later reached z = 0.009 freely at a different xy, so
0.101 was not contact. Two lessons recorded:
1. **A move that cannot converge burns its entire time budget.** A converged
   move costs ~14 sim steps; a starved one costs ~70. The horizon is 1000 steps,
   so the budget is set by starved moves, not by move count.
2. A repeatable z plateau is not proof of contact — check whether the eef is
   sliding laterally at the same time.

## v4 — deeper ladders, short `seconds` (seeds 51,53)

Evidence: 15 moves cost only **212 sim steps** (confirming lesson 1). Descent
free to eef z = 0.095 over empty table and to top+0.097 over c3 — i.e. neither
ladder reached contact. No block, no plateau.
Verdict: offset is small, not 0.100. Push the ladders further down.

## v5 — tip-offset calibration, SETTLED (seeds 51,53)

Evidence, empty table at (-0.20, 0.40):

    cmd_z  0.100 -> eef 0.1078      cmd_z  0.010 -> eef 0.0189
    cmd_z  0.085 -> eef 0.0934      cmd_z -0.010 -> eef 0.0092   <- plateau
    cmd_z  0.055 -> eef 0.0631      cmd_z -0.030 -> eef 0.0090   <- plateau

and over c3 (top 0.148): free to eef 0.1669, then cmd 0.138 blocked at eef
0.1525 while x slid +12 mm (the closed gripper hit the cap and shoved it).

Two constants, both direct measurements:
* **TIP_DZ = 0.008** — closed fingertips stop at eef z = 0.0090 over a table top
  at z = 0.0010. Confirmed independently by the c3 ladder.
* **TRACK_DZ = 0.0085** — achieved eef z is *above* the command by a constant
  0.0083-0.0089 across every rung (this is the whole of the ~0.009 residual).

These two nearly cancel: **commanded z ~= fingertip height**, which is what the
program uses.

Height profile of c3 (cross-view axis dy, the reliable one):

    z 0.148 dy 0.030 (grey cap)    z 0.100 dy 0.050 (red)
    z 0.136 dy 0.033               z 0.076 dy 0.058
    z 0.124 dy 0.034               z 0.028 dy 0.063
    z 0.112 dy 0.042

Cap/neck stays <= 0.034 wide from the top down to top-0.024, then flares. Jaws
open to 0.078, so a neck straddle at **top - 0.020** has >2x margin. Grasping
the body instead would need 0.058-0.063 inside a 0.078 jaw: rejected.

## v6 — full pick-and-place — 4/4 probe, **15/15 formal**

Design:
* target = narrow-capped bottle (top 0.07-0.25, cap <= 0.045 both axes,
  >= 150 px), ranked by redness `r - (g+b)/2`; MODE `lightcap` takes the
  *second* reddest of those with redness > 0.03, i.e. the grey-capped red bottle
  c3 rather than the all-red c6.
* grasp fingertip z = top - 0.020; `hang` (fingertip-to-object-base) **equals
  the grasp height itself**, because the object was standing on the table when
  it was grasped. No separate hang measurement is needed.
* carry z = basket rim + hang + 0.025; release z = max(hang + 0.045, rim + 0.012)
  so the bottle's base is inside the basket while the fingers stay above the rim.
* explicit `R_DOWN` on every move.

Identity verdict: **c3 is the ketchup.** MODE=lightcap scored 4/4 on
51,53,55,57 straight away, so the `redcap` arm (c6) was never needed — the
success bit is the only evidence that names the object, and it is unambiguous.

Own-sensor receipts, seed 51 (not the success bit):
* `CLOSED w=0.0336 ef=3.00` — closed gap 0.0336 matches the measured cap width
  0.030-0.034, so the jaws are on the neck, not air and not the shoulder.
* effort 3.00 held through `lift1`, `lift2`, `over` and `lower`.
* `SEG2` after the lift: c3 is **gone** from the table cluster list (7 -> 6
  props), i.e. the object left the table with the gripper.
* `SEG3` after release: 6 props, ketchup no longer among them.

Note: the final `retreat` move is a no-op (`eef` unchanged, effort drops to
0.05) because LIBERO ends the episode the moment the predicate fires. Expected;
`api.done` was never read.

Receipts:
* probe 51,53,55,57 — **4/4** — `results/fs_c2clean_obj_ketchup_pos_k0_v6`
* formal 51-65 — **15/15** — `results/sel_c2clean_obj_ketchup_pos_k0_v6`

## v7 — aim-margin diagnostic (NOT a selection candidate)

15/15 reports the score, not the margin. v7 is v6 with the grasp xy displaced
by a deliberate (+12 mm, +12 mm) to find where the neck straddle breaks.

Evidence (4/4 on 51,53,55,57 with the aim displaced +12 mm in x AND y, a 17 mm
diagonal error): `CLOSED w=0.0326 ef=3.00` on seed 51 versus `0.0336` when aimed
correctly. The closed gap barely changes, so **the neck straddle self-centres** —
the closing jaws drag the bottle into the gap rather than pushing it away.
Verdict: the grasp is not living on a knife edge; a >17 mm perception error on
the eval band would still be absorbed. v7 is diagnostic only and is NOT frozen.

## Residual risk on the sealed eval band (1-50)

The program is fully perception-driven — no scene coordinate is hardcoded — so
plain translation of the props is handled by construction. The two ways it could
still fail:

1. **Cluster fusion.** The target is picked from connected components of the
   height map. If a perturbation slides the ketchup until its pixels touch
   another prop (or the arm, which starts directly above c3's neighbourhood),
   the fused cluster's `top` would be the taller member's and the bottle filter
   (top 0.07-0.25, cap <= 0.045) would drop it. Not observed on any of the 15
   debug seeds.
2. **Colour rank inversion.** Identity rests on c3 being the *second* reddest
   narrow-capped bottle. Measured separation on the debug band is large
   (c6 0.161, c3 0.088, c4 -0.011: gaps of 0.073 and 0.099), so this would take
   a lighting change, not a position change. `_pos` perturbs position only.

Neither was reachable from the debug band, so both are stated as risks rather
than repaired speculatively.

# DECLARATION

* **Frozen version: v6.** `packs/c2clean_obj_ketchup_pos_k0/program.py`
  md5 `6e3e54dc92e7dcc1a6f0715b0a6024d1` == `program_v6.py` (verified on the
  cluster).
* **Selection receipt: 15/15** on the full debug band 51-65, one formal run,
  `results/sel_c2clean_obj_ketchup_pos_k0_v6`.
* **Per-version receipt chain:**

  | ver | role | seeds | result | dir |
  |-----|------|-------|--------|-----|
  | v1 | perception dump | 51,53 | probe failed (2000-char log cap) | `fs_..._v1` |
  | v2 | chunked perception dump | 51,53,55,57 | scene decoded, 8 clusters | `fs_..._v2` |
  | v3 | tip-offset probe | 51,53 | misleading plateau; horizon lesson | `fs_..._v3` |
  | v4 | deeper ladders | 51,53 | no contact reached; 212 sim steps | `fs_..._v4` |
  | v5 | tip-offset calibration | 51,53 | TIP_DZ=0.008, TRACK_DZ=0.0085 | `fs_..._v5` |
  | v6 | full pick-and-place | 51,53,55,57 | **4/4** | `fs_..._v6` |
  | v6 | **formal selection** | 51-65 | **15/15** | `sel_..._v6` |
  | v7 | aim-margin diagnostic | 51,53,55,57 | 4/4 at +17 mm aim error | `fs_..._v7` |

* **PROVENANCE present** in `program.py`, covering TIP_DZ, TRACK_DZ,
  GRASP_BELOW_TOP, CARRY_MARGIN, PLACE_ABOVE_FLOOR, R_DOWN and the bottle-filter
  thresholds. Every source is a debug-seed (51-65) measurement or generic
  controller/camera mechanics. No pack (K=0), no shared note file, no foreign
  constants.
* `api.done` never read (AST-checked on the frozen file).
* Not mechanism-blocked. Frozen at v6.
