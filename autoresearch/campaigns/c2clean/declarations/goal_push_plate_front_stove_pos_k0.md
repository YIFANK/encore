# c2clean / goal_push_plate_front_stove_pos_k0 — worker notes

Intent: **push the plate to the front of the stove**. K=0 (no demonstration pack).
Everything below is derived from debug seeds 51–65 only.

## Scene geometry (measured, seeds 51/53/55/57, cam_high RGB-D)

Base frame recovered from `t_base_cam` of cam_high:
`t = (0.659, 0.0, 1.610)`, camera x-axis = base **+y**, camera y-axis (image down)
= base **+x** (and −z). So in the agentview image: **right = +y, down = +x**, and the
camera sits in FRONT of the robot looking back at it. The robot mount is beyond the
far table edge, facing **+x**.

| item | how identified | x | y | z top |
|---|---|---|---|---|
| table plane | depth histogram spike (116 k px) | −0.50 … +0.27 | −0.55 … +0.54 | **0.903** |
| stove | biggest dark slab, burner spiral + knob | −0.46 … −0.16 | +0.11 … +0.31 | 0.961 |
| plate | brightest low disc (mean RGB ≈ 142) | −0.12 … +0.02 | +0.06 … +0.21 | **0.921** |
| bowl | grey (105) | −0.15 … −0.03 | −0.06 … +0.07 | 0.952 |
| wine bottle | dark (21), tall | −0.21 … −0.17 | −0.08 … −0.02 | 1.059 |
| cookie box | blue (72,78,95) | +0.01 … +0.10 | −0.06 … +0.00 | 0.920 |
| cabinet + rack | huge, y < −0.12 | — | −0.36 … −0.12 | 1.245 |

Plate diameter ≈ 0.136 m (r ≈ 0.068), standing 0.018 m proud of the table.
_pos perturbation across 51/53/55/57 is small: plate centre moves ≈ ±0.015 m,
stove ≈ ±0.010 m. **Not concluded on the probe subset** — the full debug band is
re-measured before selection.

## Which way is "the front of the stove"?

Two readings. Settled geometrically before spending a run on the wrong one:

* **−x (the robot's side, where the stove's control knob sits).** The stove's −x
  edge is at x ≈ −0.452 and the table's far edge is at x ≈ −0.50. That leaves a
  0.05 m strip, and the plate is 0.136 m across: **the plate cannot be placed
  there at all.** Reading refuted by geometry.
* **+x (the open side of the table, toward the agentview camera).** The plate
  already lies on that side (x ≈ −0.05 vs the stove's front edge −0.16) but is
  offset ≈ 0.07 m in y from the stove's centre line (y 0.14 vs 0.21).

So H1: the missing ingredient is the **lateral (y) alignment** with the stove,
and the push is a short +y translation. This is what v2 tests.

## Version log

### v1 — pure observation probe (no motion)
Hypothesis: the scene can be read from cam_high alone; no pack is needed.
Evidence: 4/4 episodes returned full RGB-D through `api.log`; segmentation above
the table plane yields the eight components tabulated above.
Gotcha found: **`api.log` truncates each message at 2000 characters**, so the
zlib+base64 image datapipe must chunk at ≤1900 chars (the first launch silently
lost 97 % of every payload). `benchmark_success` false 4/4, as expected for a
no-motion probe — this also confirms the initial state does not already satisfy
the predicate.
Verdict: perception pipeline good; kept as the basis of every later version.

### v2 — perceive, blocked-descent calibration, push +y  (H1)
Hypothesis: closing the gripper and sweeping the closed tip through the plate's
−y rim, from the plate's current y to the stove's centre y, satisfies the
predicate.
Mechanism details: fingertip offset is measured live each episode by a blocked
descent onto bare table at (plate_x, plate_yhi + 0.12); push height is
table_z + 0.006 + tip_offset, so the tip strikes the rim wall (rim is 0.018 tall);
the sweep is broken into ≤0.02 m sub-moves.
Evidence: 0/4. The blocked descent stalled at eef_z 0.9322 on all four seeds
(table 0.9010), read as a 0.0312 fingertip offset — **this number was wrong and
cost eight versions** (see v11). The arm also wedged after the hard press and
every later move was a no-op.
Verdict: rejected; the calibration it produced poisoned v3–v10.

### v3 — checked moves, no contact calibration
Hypothesis: v2's wandering arm was starved moves, not a broken plan.
Evidence: motion became accurate (err ~0.010 on every waypoint, 183 sim steps
for 8 moves) and the sweep ran exactly as planned — and the plate did not move
by a millimetre. Also established that **captures cost no sim steps**, so
perception can be repeated freely, and that the episode horizon is 1000 steps.
Verdict: motion layer good, contact layer broken.

### v4 — fingertip from depth; 0/4
Hypothesis: measure the tip instead of inferring it, by taking the lowest point
near the eef in cam_high depth.
Evidence: the "gripper" it measured was the bowl (radius 0.055 around the eef
caught it), giving off=0.1056. Push height 0.0836 m too high. Also exhausted
the 1000-step horizon.
Verdict: rejected. Lesson: the hand body occludes its own fingertips from a
downward camera, so cam_high can never see them.

### v5 / v6 — edge strike vs press-and-drag, run in parallel; 0/4 each
Hypothesis: either strike the rim wall below its crest (v5) or press into the
dish and drag (v6).
Evidence: v6's descent stalled 0.007 m above its command *on the dish* and the
blade then slid the full sweep without moving the plate at all. The saucer's
interior is a 27-degree ramp, so a horizontal push there becomes downforce.
v5 moved nothing either. (First launch of v5 was lost to a GPU index of 8.)
Verdict: press-and-drag refuted as a mechanism for this object.

### v7 — strike-height ladder with per-episode probe; 0/4
Evidence: rung 1 moved the plate +0.023 m — the only transfer seen before v12 —
and rungs 2-4 nothing. The probe descent stalled at 0.9089 over the same bare
table that v2/v8 stalled at 0.9322, **which a real contact cannot do**.
Verdict: the single transfer was incidental (a hard press that slid the arm),
and the stall-based calibration was now suspect.

### v8 — three-surface descent probe; 0/2
Evidence: bare table reproduced 0.9322; the stove and plate probes never ran
because the arm froze after the first hard press.
Verdict: inconclusive, but it confirmed the freeze is caused by pressing.

### v9 — rim pinch; 0/3
Evidence: `goto_precise` (fold the observed error back into the command) landed
waypoints to 0.0003–0.002 m, a real gain kept in every later version. The grasp
closed to width 0.0010 — on air.
Verdict: grasp missed, for the same reason every strike missed.

### v10 — ladder of five tip heights, all precise; 0/3
Evidence: **zero transfer at every assumed tip height from table+0.010 to
table+0.0165**, with sweeps running 0.039 m past the rim. This falsified the
whole contact model rather than any one height.
Verdict: rejected, and it forced the calibration question.

### v11 — CALIBRATION ONLY (the version that unlocked the cell)
Hypothesis: error-cancelling moves track a free target to ~0.002 m, so a
genuine contact is now distinguishable from a move that merely ran out of
travel: the achieved z plateaus while the command keeps falling.
Evidence (seed 51, bare table at 0.9010):

| commanded z | achieved z | gap |
|---|---|---|
| 0.951 | 0.9514 | +0.000 |
| 0.921 | 0.9203 | −0.001 |
| 0.906 | 0.9092 | +0.003 |
| 0.891 | 0.9090 | **+0.018** |
| 0.871 | 0.9090 | **+0.038** |

The plateau is 0.9090 and 0.9090 − 0.9010 = **0.0080**: the fingertip sits
0.008 m below the eef origin, and the tip rests exactly on the table. The stove
ladder plateaus consistently on the slab.
Verdict: **TIP_OFF = 0.0080, not 0.0312.** Every strike from v3 to v10 swept
25–35 mm above a plate that is 19 mm tall.

### v12 — v10's ladder at the corrected height; **2/4**
Evidence: first contact on the first rung (h = table+0.010, eef_z = 0.919):
seed 51 moved the plate +0.0176 m in y and succeeded; seed 57 succeeded 75 sim
steps in, mid-sweep. Seeds 53 and 55 froze on the very first waypoint at
(x≈0.00, y≈−0.040, z=1.041) — into the **wine bottle**, whose top is 0.158 above
the table, i.e. above that transit height. The two seeds that succeeded had
their plate 0.014 m further +y, which cleared the bottle.
Verdict: mechanism confirmed; path planning is the remaining defect. Also
confirms H1 — "the front of the stove" is the +x side, reached by aligning the
plate laterally with the stove.

### v13 — v12 with all transits above the bottle; **8/8 probe, 15/15 selection**
Hypothesis: dropping the pre-positioning waypoint and flying every approach and
retreat at table+0.22 recovers the seeds v12 lost to the collision.
Evidence: 8/8 on the probe subset (51,53,…,65), then 15/15 on the full debug
band (`results/sel_c2clean_goal_push_plate_front_stove_pos_k0_v13`). Seed 53 is
the only one that needs more than one strike (774 sim steps against a median of
almost exactly 160); every other seed terminates between 82 and 211 steps.
Verdict: accepted as the mechanism.

### v14 — FROZEN. v13 with dead code removed
Hypothesis: the constants and helpers left over from v2–v11 (`MODE`, `EDGE_H`,
`PRESS_H`, `GRASP_H`, `LIFT_H`, `FRONT_GAP`, `MEAS_*`, `TIP_OFF_FALLBACK`,
`measure_tip`, `grip_state`) are all unreferenced, so removing them cannot
change behaviour, and PROVENANCE then covers exactly the live constants.
Evidence: 15/15 on the full debug band, with **sim-step counts identical to
v13 on all fifteen seeds** — the strongest available check that the cleanup is
behaviour-preserving. `results/sel_c2clean_goal_push_plate_front_stove_pos_k0_v14`.
Verdict: **frozen as program.py**.

---

## What the cell actually turned on

One number. The fingertip sits **0.0080 m** below the eef origin, not the
0.0312 m that two "blocked descents" implied. Those descents had not stalled on
anything — `api.move` had simply run out of travel, and because a starved move
and a contact look identical in a single reading, the wrong constant survived
nine versions. Every strike from v3 to v10 swept 25–35 mm above a 19 mm plate,
which is why the plate never moved by so much as a millimetre and why the rim
grasp closed on air.

What separated them was **error-cancelling moves** (v9): once a free move tracks
its target to ~0.002 m, a contact is visible as a plateau — the achieved z stops
following while the command keeps falling. That test (v11) is cheap, decisive,
and is the thing to reach for first next time, before any contact constant is
trusted.

Two secondary lessons, both paid for:
* the hand body occludes its own fingertips from a downward camera, so cam_high
  can never measure them (v4, v5);
* the saucer's interior is a 27-degree ramp, so a horizontal push from inside
  becomes downforce and the blade rides over (v6). Only the steep outer rim
  wall transfers force.

---

## DECLARATION

* **Frozen version:** `program.py`, md5 `6efb01187d24380bfa42bb72c2e5e726`,
  identical to `program_v14.py` (same md5, verified on the cluster).
* **Selection receipt:** **15/15** on the full debug band, seeds 51–65, one
  formal run, `results/sel_c2clean_goal_push_plate_front_stove_pos_k0_v14`.
* **Per-version receipt chain:**

| version | what changed | seeds | result |
|---|---|---|---|
| v1 | observation probe, no motion | 51,53,55,57 | 0/4 (expected) |
| v2 | blocked-descent calibration + push | 51,53,55,57 | 0/4 |
| v3 | checked moves | 51,53,55,57 | 0/4 |
| v4 | fingertip from cam_high depth | 51,53,55,57 | 0/4 |
| v5 | edge strike at table+0.013 | 51,53,55,57 | 0/4 |
| v6 | press-and-drag | 51,53,55,57 | 0/4 |
| v7 | strike-height ladder | 51,53,55,57 | 0/4 |
| v8 | three-surface descent probe | 51,53 | 0/2 |
| v9 | rim pinch; adds `goto_precise` | 51,53,55 | 0/3 |
| v10 | five precise tip heights | 51,53,55 | 0/3 |
| v11 | calibration only → **TIP_OFF = 0.0080** | 51,53 | 0/2 (no push) |
| v12 | ladder at the corrected height | 51,53,55,57 | **2/4** |
| v13 | transits above the wine bottle | 51,53,…,65 | **8/8**; selection **15/15** |
| **v14** | dead code removed, PROVENANCE pruned | **51–65** | **15/15 (frozen)** |

* **PROVENANCE:** present, and every one of the eighteen live constants is
  covered — checked by AST against the module-level assignments, with no
  missing and no stale entries. Sources are debug-seed (51–65) RGB-D
  measurements, debug-seed motion logs, and generic controller/camera
  mechanics. No demonstration pack was supplied or used, and no benchmark asset
  file was read.
* **Splits:** seeds 1–50 were never touched; every run used `--split debug`.

