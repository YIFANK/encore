# c2clean — goal_put_bowl_on_plate_pos_k0

Intent: **"put the bowl on the plate"**. No demonstration pack (k0). Runner:
`tools/fair_run.py` only. Debug seeds 51-65; eval seeds 1-50 never touched.

## Scene, as measured from my own debug-seed RGB-D (v0 dump, seeds 51-65)

`cam_high` is at base (0.659, 0, 1.610) looking down the -x axis at ~38°;
`K = 618.04` focal, 512×512. Table plane **z = 0.9010** (modal z of the flat
surface, identical on all 8 probed seeds). Heights below are *above* that plane.

| object | centre (x, y) | top h | notes |
|---|---|---|---|
| **bowl** (target) | (−0.045±0.008, +0.133±0.010) | 0.0511 | body of revolution, rim radius 0.0537 (Kasa scatter 1.2 mm), interior open down to h=0.008 |
| **plate** (goal) | (−0.21±0.02, −0.044±0.012) | 0.020 rim | flat disc, floor h≈0.0087, radius ≈0.078 |
| cabinet | y < −0.12 | 0.23 / 0.35 | left of scene |
| stove slab | (−0.26, +0.21) | 0.032 | + knob at (−0.384,+0.21) h 0.060 |
| wine bottle | (+0.055, −0.018) | 0.159 | |
| small box | (−0.080, +0.006) | 0.020 | abuts the plate rim in some seeds |

Bowl radial profile (seed 51, heights above table vs. radius from its axis):

```
outer wall radius by height band:  h .020→.0417  .030→.0458  .040→.0498  .050→.0546
inner wall (max h at radius):      ρ .0385→.0282 .042→.0368  .0455→.0434 .0525→.0521
interior floor: h 0.0074 out to ρ=0.024   →  the bowl is ~44 mm deep and open
```

**The decisive constraint**: the gripper's open gap is 0.0778 (measured at
episode start) and the bowl is 0.107 across. It cannot be grasped across.
The interior is open from h=0.008 up, and the wall is only ~6 mm thick
horizontally, so the available hold is a **wall straddle**: one finger inside
the bowl, one outside, closing on the wall. At h=0.030 the wall midline sits
at radius 0.0422 = rim radius − 0.0115.

## Version log

### v0 — perception dump (no actions)
Hypothesis: nothing; instrumentation only. RGB-D from both cameras piped out
through `api.log` as zlib+base64 chunks (api.log truncates at 2000 chars, so
1800-char chunks), decoded on the Mac.
Evidence: `results/fs_..._v0`, 0/8 as expected (no actions taken). All of the
table above comes from this dump.
Verdict: perception validated offline on all 8 probed seeds — bowl found with
r = 0.0537 and scatter ≤1.3 mm every time; plate found every time.

### v1 — straddle grasp, in-episode self-calibration
Hypothesis: the bowl can be picked by straddling its wall on the −y side at
h=0.030 and set down centred on the plate; the two unknowns that cannot be
read from a static dump (the eef→fingertip offset, and where the bowl ends up
relative to the eef after the close) are measured *inside* the episode — the
first by pressing the closed gripper onto empty table and reading the stall
height, the second by re-fitting the bowl's circle below the fingertips while
it is in the air.
Evidence: `results/fs_..._v1`, **0/8**. The receipts showed every move after the
press was a silent no-op and `sim_steps: 1000` — the press burned the whole
episode horizon (each blocked `move(seconds=2.0)` costs its full 240-step cap,
and the press issued several). It did answer its question: the closed gripper
stalled with the eef at 0.9089 over a 0.9009 table, so the **eef reference sits
0.0080 above the fingertip plane** — which independently matches the v0 dump,
where the lowest gripper point is z=1.1652 with the eef reported at 1.17328.
Verdict: hypothesis untested; calibration answered and promoted to a constant.

### v2 — same mechanism, press removed, episode budgeted
Hypothesis: with the press gone and every waypoint re-commanded at most 3× at
0.6 s, the straddle grasp fits inside 1000 steps.
Evidence: `results/fs_..._v2`, **5/8** (51,55,59,61,63), 378–1000 steps.
Verdict: the mechanism works — `CLOSED` read width 0.0069–0.0095 (the wall) with
effort 3.0 on all 8. Three *distinct* failures: ep53's grip slipped on the lift;
ep57 bounced (the in-flight `hang` was reading the mask floor, not the bowl, so
the release happened 0.11 m up); ep65 hit the horizon mid-descent.

### v3 — set-down, budget, grasp check
Hypothesis: fix those three — anchor the `hang` band to the fingertips, drop the
parking trip (the start pose occludes the plate but never the bowl), cut the
travel height 0.20 → 0.10, and re-grasp once if the hand comes up empty.
Evidence: `results/fs_..._v3`, **8/8**, 310–441 steps. v3b (straddle 8 mm
deeper, H_CLOSE 0.022) also 8/8 — the grasp is not sensitive to that.
Verdict: passes, but by the wrong mechanism. `LOWER` stalled at z = 1.003 with a
0.05 residual on *every* seed: the bowl was still being dropped 55 mm. A −y
straddle puts the eef 47 mm to −y of the plate at release, so the hand body
(±0.06 in y) reaches y = −0.155 — inside the cabinet, which stands 0.26–0.35
above the table for all y < −0.122.

### v4 — straddle the +y side
Hypothesis: grasping the far side from the cabinet moves the release pose to
y ≈ 0.00 and lets the bowl actually be set down.
Evidence: `results/fs_..._v4` **8/8** with every move converged (residuals
0.002–0.013) and **93–154 steps**, a tenth of the horizon. Formal 15:
`results/sel_..._v4` **14/15**, ep54 the loss.
Verdict: mechanism right; one seed unexplained.

### Envelope probe — how much aim error does the task actually tolerate?
15/15 says nothing about margin, so I displaced the release aim directly
(`program_v5ex*`, 8 probe seeds each):

| displacement | result |
|---|---|
| +0.020 in y | **8/8** |
| +0.030 in y | **0/8** |
| +0.030 in x | 3/8 |

A cliff at ~0.025, exactly where geometry puts it: the plate's flat floor disc
is 0.052 across the radius and the bowl's foot ring 0.025. **Everything below is
sized against that 25 mm budget.**

### v5 — `hang` measured over the bowl only
Hypothesis: v4's `hang` disc (radius 0.13 about the eef) reaches the stove slab,
whose near edge is 0.115 from the grasp pose; two seeds read 0.087 (the mask
floor) and re-grasped for nothing.
Evidence: `results/fs_..._v5` **8/8**. Verdict: passes; spurious retries gone.

### ep54 diagnosed — the plate is not always flat
Dumped all 15 debug seeds from the *parked* pose (`program_v0c`) and fitted each
plate's floor plane. **ep54's plate tilts 7.9° with its centre 0.0133 above the
table** (flat seeds: 1.6°, 0.0075); ep64 tilts 4.9°, ep56 3.8°, ep62 3.3°. It
rests against the cabinet's foot. Two consequences, both fatal at 25 mm:
1. absolute height bands read the tilted floor as the rim ring and fitted a
   circle of radius 0.0545 instead of 0.0655;
2. a floor-cell centroid only sees the *downhill* half of a tilted floor, so it
   is pulled downhill — up to 22 mm (ep64) from the true centre;
3. and the bowl, set down on a slope, slid 25 mm downhill — ep54's v4 receipt
   has the bowl resting 25 mm to +y of the plate centre, i.e. at the cliff.

### v6 — parked read + rim-circle plate fit
Hypothesis: read the scene from a parked pose (0.05, 0.32) — bare table on all
15 seeds, off every cam_high sight line — and take the plate centre from a
**circle fit to its rim ring** instead of a centroid. A clipped disc's centroid
moves with the clipping; a circle fit needs only an arc.
Evidence: offline on all 15 parked dumps the fit is far steadier than the
centroid, but still broke on ep54 (r = 0.0545, tilt-poisoned band).
Verdict: necessary, not sufficient. Not run on the cluster.

### v7 — tilt-aware plate, and a placement that checks itself  ← FROZEN
Hypothesis: take every plate band **relative to the plate's own fitted floor
plane** (so tilt and standoff drop out of both the segmentation and the release
height), express the grasp height relative to the bowl's own rim rather than the
table (same thing on a table, right thing on a plate), and then **verify and
correct**: park, re-fit the bowl, and if it settled more than 0.012 from the
plate centre, pick it up and set it down displaced by minus the observed miss —
the slide belongs to the slope, so repeating it from a corrected start lands on
the centre. LIBERO latches task success, so a correction can only ever add.
Evidence:
* offline on all 15 parked dumps the fit is now consistent — **r = 0.0637–0.0647
  with 1.4 mm residual scatter and 83–96% ring coverage on every seed**,
  ep54 included. Projected back into the image the circle traces the plate rim
  and the centroid marker is visibly off-centre on the tilted seeds.
* probe `results/fs_..._v7` **8/8**, 135–146 steps, every seed "placed and
  verified" on the first attempt, settled miss **0.5–9.9 mm** against the 25 mm
  cliff.
* formal `results/sel_..._v7` **15/15**.
* ep54 by the intended mechanism: tilt 8.31° detected, floor read as 0.0140
  instead of 0.0087, and the slide fell from 25 mm to **5.5 mm**.
Verdict: **selected.**

---

# DECLARATION

**Frozen version: v7.** `packs/c2clean_goal_put_bowl_on_plate_pos_k0/program.py`
md5 `8f21d2d15f1089575a08b92ef0711feb` == `program_v7.py` (identical on the Mac
and on the cluster).

**Selection receipt (full 15 debug seeds, one formal run):**
**15/15** — `results/sel_c2clean_goal_put_bowl_on_plate_pos_k0_v7`
(seeds 51-65, all `benchmark_success: true`, 135-146 of 1000 sim steps each).

**Receipt chain**

| version | mechanism change | probe (8 seeds) | formal (15 seeds) |
|---|---|---|---|
| v0 | perception dump, no actions | 0/8 (expected) | — |
| v1 | straddle grasp + in-episode press calibration | 0/8 — press ate the horizon | — |
| v2 | press removed, budgeted | 5/8 | — |
| v3 | set-down, no park, lower travel, grasp retry | 8/8 | — |
| v3b | as v3, straddle 8 mm deeper | 8/8 | — |
| v4 | straddle the +y side (clears the cabinet) | 8/8 | 14/15 |
| v5ex2/ex3/ex4/exx3 | aim-envelope probes (+0.02y / +0.03y / +0.04y / +0.03x) | 8/8, 0/8, 0/8, 3/8 | — |
| v5 | `hang` measured over the bowl only | 8/8 | — |
| v6 | parked read + rim-circle plate fit | not run | — |
| **v7** | **tilt-aware plate plane + verify-and-correct** | **8/8** | **15/15** |

**PROVENANCE**: present in `program.py` as a top-level literal dict, 15 entries,
every calibrated constant covered. Sources are debug-seed (51-65) observations —
my own RGB-D dumps, gripper state, eef readings and run receipts — plus generic
controller and camera mechanics. No demonstration pack exists for this cell and
none was used; no prior-context LIBERO facts were used (the reach/height/offset
numbers here were all re-derived from the v0/v0b/v0c dumps).

**Eval seeds 1-50 were never run or inspected.** No `tools/fewshot_run.py`.
No `api.done` read (AST-checked on the frozen file).
