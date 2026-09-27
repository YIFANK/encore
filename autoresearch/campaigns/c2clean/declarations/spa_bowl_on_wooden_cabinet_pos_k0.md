# c2clean / spa_bowl_on_wooden_cabinet_pos_k0

Intent: "pick up the black bowl on the wooden cabinet and place it on the plate"
No demo pack (k0). Debug seeds 51-65; probe subset 51,53,55,57.

## Scene geometry (all numbers re-derived from debug-seed cam_high RGB-D)

Measured on seeds 51/53/55/57, agreeing to <1 mm unless noted:

| quantity | value | how |
|---|---|---|
| table top z | 0.8994 | mode of deprojected z over the workspace |
| cabinet top z | 1.1273 | median z of the tall (>table+0.15) component |
| bowl rim top z | 1.1799 | max z of the component above the cabinet top |
| bowl rim radius | 0.0527 +- 0.0004 | Kasa circle fit on the rim ring |
| bowl outer radius | ~0.056 | radius where max-z falls back to the cabinet top |
| bowl interior floor z | 1.136 | max z for r < 0.02 |
| bowl height | 0.0526 | rim top - cabinet top |
| plate top z | 0.9508 | cells above the slab plane (slab 0.926) |
| plate radius | ~0.066 | half bbox span |
| gripper open width | 0.0792 | api.grip(0.08) settled |
| TIP_OFFSET | 0.0091 | blocked descent, closed jaws, on the bare table |

Layout: one tall fixture (the cabinet, top 1.1273) carrying exactly one bowl.
Two further bowls and a cookie box sit on the table; the plate sits on a low
grey slab. The elevated bowl is therefore unambiguous without any colour cue —
it is the only bowl not on the table.

`_pos` perturbation: the cabinet bowl's centre moves ~5 cm in x across seeds
(bcx -0.019 .. +0.032) and ~2 cm in y; the plate moves ~1.3 cm. Rim top z and
cabinet top z are invariant.

## Key mechanical facts

- **The bowl cannot be straddled.** Outer radius 0.056 (diameter 0.112) against
  a 0.0792 max jaw opening, and the rim is the widest point, so jaws lowered
  around the body collide with the rim. A rim pinch is the only parallel-jaw
  grasp available.
- **The rim wall is thin** (<3 mm: max-z falls from 1.180 to the cabinet top
  within one 2.5 mm radial bin). Expect the closed gap to sit below the
  effort-flag threshold, so `effort` will not certify the hold — verify with
  gap survival across the lift instead.
- **Jaws open along world y** under R_DOWN = diag(1,-1,-1), so the pinch offset
  from the bowl centre must be along +-y.
- **The camera clips the cabinet bowl** at the left FOV edge, biasing its
  bbox centroid toward +y. The Kasa ring fit recovers the true centre; its
  radius is stable to 0.4 mm across seeds, which is the receipt that the fit is
  not being dragged by the clipping.
- **Move tracking is biased** by up to 12 mm (eef lands past/short of the
  command). All grasp-critical poses go through a bounded bias-cancel loop.

## Version log

### v1 — perception probe (no motion)
Hypothesis: the scene can be read from cam_high alone.
Evidence: RGB-D streamed back through api.log (zlib+base64) and analysed
offline; full geometry table above recovered. 0/4 success (no motion attempted).
Verdict: perception sufficient; proceed.

### v2 — calibration probe (no grasp)
Hypothesis: the fingertip-to-eef offset and the reach above the bowl can be
measured directly.
Evidence: closed jaws driven into the bare table stall at eef z = 0.9085 on
4/4 seeds against a table at 0.8994 -> TIP_OFFSET = 0.0091. Reach to
(bcx, bcy, 1.21..1.31) succeeds with residual 0.008-0.018. 0/4 (no grasp).
Verdict: calibration good; the bowl is reachable; controller bias needs
cancelling.

### v3 — first rim pinch
Hypothesis: a rim pinch at radius 0.055 on the +y side, biting 0.020 below the
rim top, holds the bowl through a carry to the plate.
Evidence: **3/4** (51,53,55 ok; 57 fail) — `fs_..._v3`. The failure is legible
in the gripper trace: the closed gap decayed 0.0072 -> 0.0048 and the hold flag
dropped during the lift, and the gif shows the bowl landing off-centre on the
plate. The two survivors held their gap (0.0056, 0.0069).
Verdict: the grasp works; the grip is marginal and an un-noticed slip shifts
the bowl in the jaws, which then lands it off the plate.

### v4 — verified grip + measured carry offset
Hypothesis: checking the grip after the lift (and retrying) plus measuring the
held bowl's offset will fix the slip-then-miss failure.
Evidence: **5/8** — `fs_..._v4`. Three new failure modes, all harness mechanics:
61/63 burned the whole 1000-step horizon before the gripper ever closed; the
"is it held" test passed on a frozen wide-open gripper; 65 placed the bowl
below the table because the held-bowl fit banded on the wrist (1.43 m) instead
of the rim.
Verdict: the retry idea is right, the instruments were wrong. Root causes found
by reading the controller: `move_pose` requires BOTH position and rotation to
converge and an exactly-straight-down R_DOWN sits on the 0.06 rad tolerance, so
moves never returned; and `effort` is only `(close commanded AND gap > 0.005)`.

### v5 — position-only moves, honest hold test
Hypothesis: commanding position only (rotation=None) removes the starvation,
and a gap-band + gap-survival + eef-rose test is an honest hold check.
Evidence: **5/8** — `fs_..._v5`. Horizon no longer exhausted (max 839). New
failure: 61/63's carry never moved in x.
Verdict: starvation fixed. The bias-cancel loop was extrapolating a STARVED
move as if it were a tracking bias, aiming the second command 0.26 m past the
target, outside the reach envelope.

### v6 — starvation-safe bias cancel, set-down place
Hypothesis: only fold small residuals back into the command; ground the bowl on
the plate instead of dropping it.
Evidence: **5/8** — `fs_..._v6`. The set-down works where it runs (55 stalls
0.0065 above the commanded depth, i.e. it grounds). 61/63 still jam; the gif
shows the wrist wedged against the cabinet. Where the held-bowl fit did fire
(51,55,59) it measured the carry offset as **0.038-0.044**, not the assumed
0.055 — and all three succeeded.
Verdict: the jam is a collision, not a budget or a reach limit.

### v7 — routed carry (regression)
Hypothesis: clearing the cabinet in y and dropping to mid-height before the
traverse avoids the jam.
Evidence: **4/8** — `fs_..._v7`. Worse. 51/59 now fail purely on BUDGET (three
grasp attempts plus a four-waypoint carry exceed 1000 steps), and the
held-bowl fit never fired at all (the hand body outranks the rim in a top-down
max-z map, so the component's ztop is the hand).
Verdict: the route is right but was both too expensive and not actually clear:
at y=-0.045 the held bowl (r 0.056, centre trailing by ~0.042) still overlaps
the cabinet's +y face, and its widest part hangs below the cabinet top.

### v8 — cabinet-aware clearance + affordable moves
Hypothesis: compute the clearance from the perceived cabinet
(`cab_ymax + OFF_Y + BOWL_R + 0.020`), hold it until x is past the cabinet, and
stop converging waypoints to grasp accuracy.
Evidence: **6/8** — `fs_..._v8`. Budget healthy: successes run 210-507 steps.
51 (2 attempts) and 53 (3 attempts) now both fit and pass. 61/63 still fail.
Verdict: best so far. The remaining two are the seeds with the largest bcx
(+0.041), where the arm ends up extended and the single 0.33 m retract command
leaves x frozen while the arm is over-folded.

### v9 / v10 / v11 / v12 — four routes around the stall (all 6/8)
Hypothesis (each): the frozen retract on 61/63 is command saturation, cabinet
collision, wrong waypoint order, or too high a carry.
Evidence: **6/8 each**. The walked traces make the stall precise and identical:

| version | retract taken at | stalls at x | eef behaviour |
|---|---|---|---|
| v9  | z 1.086 (mid) | 0.0615 | climbs |
| v10 | z 1.27 (carry) | 0.0024 | climbs to 1.2889 |
| v11 | retracing the inbound diagonal | 0.0027 | climbs to 1.2889 |
| v12 | z 1.03 (low) | 0.0647 | climbs |

Seed 55 runs the identical walk and tracks every hop to within 9 mm.
Verdict: not budget, not saturation, not waypoint order, not height.

### probe_reach — is it a reach limit? (diagnostic, not a candidate)
With an **empty** gripper, seeds 61/63 retract -x from the pregrasp pose by
0.234 m — the same as seed 55 (0.233 m), tracking every hop.
Verdict: no kinematic barrier and no reach limit. The arm is free; something
about holding the bowl is not.

### probe_bowl — the bowl, or the swing? (diagnostic)
Holding the bowl, retract -x with no y swing at all:
seed 61 travels **0.100 m**, seed 55 travels **0.290 m**. After a +y swing,
seed 61 manages 0.019 m.
It also exposed a clean correlation in the gripper trace:

| seed | \|eef_x - bcx\| at close | closed gap |
|---|---|---|
| 55 / 57 / 51 | 0.003 | 0.0085-0.0091 |
| 61 / 63 | 0.013 | 0.0052-0.0055 |

The jaws close along y, so they cut a diameter of the rim only when eef_x
equals the bowl's centre x; 13 mm off makes the bite a chord through a thin
sliver. `descend_to` had been converging z alone and let x drift up to 14 mm
during the descent.

### v13 — centre the jaws in x and y before closing
Hypothesis: converging the descent in all three axes, not just z, restores the
full-thickness bite and with it the carry.
Evidence: the aim fix works exactly as intended — seed 61's centring error
falls 0.0125 -> 0.0027 and its closed gap rises 0.0055 -> 0.0061. But the
carry stalls at x=0.0647, **byte-identical to v12**. 6/8 probe.
**Formal selection: 12/15** (`sel_..._v13`), failures 60, 61, 63.
Verdict: grip quality was a real defect and is fixed, but it is not what
blocks the carry — the stall is geometric.

### v14 — retract before the swing
Hypothesis: probe_bowl says retracting first is worth 0.10 m, so reorder.
Evidence: **6/8**. The pull does reach x=-0.0553 as predicted, and then the
+y swing springs x straight back to +0.003.
Verdict: the ordering gain is real but not additive; the swing undoes it.

### v15 — relocate the bowl and re-grasp
Hypothesis: stop routing around the limit and move the bowl — set it down at
the furthest x the arm reached, re-perceive, re-grasp from the easier pose.
Evidence: **6/8**. The set-down descent was itself blocked (28 mm short), so
the bowl was dropped from height and bounced back to bcx=0.0355, almost where
it started; the re-grasp then ran out of horizon.
Verdict: the relocate is sound in principle but needs a descent the loaded arm
cannot make at that pose.

### probe_side — which SIDE of the rim is pinched (diagnostic)
Pinching the **-y** (far) side instead of the +y side, then the same retract:
seed 63, a clean hold (gap 0.0079 throughout), travels **0.2895 m** — against
0.099 m for the same seed, same bowl, same carry, pinched on the +y side.
Verdict: **this is the mechanism.** A +y pinch leaves the bowl hanging on the
cabinet side of the gripper, between the arm and the fixture; a -y pinch puts
it on the far side and the carry is free.

### v16 — far-side pinch
Hypothesis: pinch the far side, carry by pulling x at the grasp y, then turn
onto the plate once past the cabinet.
Evidence: **5/8**. Seeds **61 and 63 now pass** — the two that had failed under
every one of v8-v15. It loses 53/59/65 instead: their pull hops were
single-shot, so nothing corrected the y and z the arm sheds while dragging x,
and y bled to -0.39 (against -0.340 on the seed that completes).
Verdict: the side change fixes the systematic failure and introduces a
different, narrower one.

### v17 — hold y through the pull
Hypothesis: re-converge each hop that has shed more than 0.02 in y or z.
Evidence: **5/8**. The correction does not help — the re-issued hop drifts
further -y still. The y bleed is the arm at its envelope, not a tracking error.
Verdict: the far-side pull has a narrow y band that cannot be servoed into.

---

## Formal selection runs (full 15 debug seeds)

| version | pinch side | result | failures |
|---|---|---|---|
| **v13** | +y (near) | **12/15** | 60, 61, 63 |
| v16 | -y (far) | 8/15 | 52, 53, 56, 59, 62, 64, 65 |
| v17 | -y (far), drift-corrected | 10/15 | 51, 58, 59, 64, 65 |

Dirs: `results/sel_c2clean_spa_bowl_on_wooden_cabinet_pos_k0_v13` / `_v16` / `_v17`.

### Why the two families were not combined
They are complementary on the debug band — v17 succeeds on exactly the three
seeds v13 fails — so a side-selecting hybrid would score 15/15 here. It was not
taken, because the only pre-grasp signal available to select on is the bowl's
centre x, and that separation is a knife edge:

    near-side PASSES at bcx = ... 0.0321, 0.0332, 0.0354, 0.0377
    near-side FAILS  at bcx =     0.0402, 0.0413, 0.0472

A 2.5 mm gap, with the observed bcx distribution dense right through it. A cut
placed anywhere in that gap is fitted to three seeds and would be a coin flip
for any eval seed landing in it. The runtime alternative — grasp near-side,
measure the pull, and re-grasp far-side if it stalls — was costed and does not
fit: the recovery needs ~1400 steps against a 1000-step horizon, because it
pays for two full grasps and two carries.

## MECHANISM GAP (documented stop)

**Statement (falsifiable).** With the bowl pinched on its +y rim, the loaded
arm cannot retract the eef past x ~= -0.05 when the bowl starts at
bcx >~ 0.040. This is not reach, not step budget, not command saturation and
not waypoint ordering — all four were tested and excluded. It is the side of
the rim that is pinched: a +y pinch leaves the bowl hanging between the
gripper and the cabinet, and that configuration blocks the retract.

**Receipts on debug seeds.**
- Empty gripper, seed 61, retract -x from the pregrasp pose: **0.234 m**, every
  hop tracked (`probe_reach`). Seed 55: 0.233 m. No kinematic barrier.
- Same seed 61 holding the bowl, +y pinch, same retract: **0.100 m**, then the
  eef climbs and is dragged -y (`probe_bowl`). Seed 55: 0.290 m.
- Seed 63 holding the bowl, **-y pinch**, same retract: **0.2895 m** against
  0.099 m for the identical seed, bowl and carry pinched +y (`probe_side`).
- Four carry routes (mid-height, carry-height, inbound-diagonal retrace,
  low-height) stall seed 61 at x = 0.0615 / 0.0024 / 0.0027 / 0.0647 — the eef
  rising each time, which is the controller converting blocked -x into +z.

**What is missing.** A grasp that both holds reliably and leaves the bowl clear
of the cabinet. The far-side pinch supplies the clearance but not the hold
(its closed gaps run 0.0046-0.0083 against 0.0085-0.0091 near-side, and its
retries make matters worse by shoving the bowl away from the robot rather than
toward it). Closing this would need either a wrist yaw — so the jaws close
along x and the bowl hangs neither toward nor away from the cabinet — or a
regrasp budget the 1000-step horizon does not allow. `move_pose` is the only
route to a yaw and it starves on this task (see v4/v5), so the yaw was not
reachable through the fair API as it stands.

---

# DECLARATION

- **Frozen version:** `program_v13.py`, md5 `a1c123f23ff69ed2803c3524850c405a`,
  copied to `packs/c2clean_spa_bowl_on_wooden_cabinet_pos_k0/program.py`
  (md5 verified identical).
- **Selection receipt:** **12/15** on the full debug band (seeds 51-65),
  `results/sel_c2clean_spa_bowl_on_wooden_cabinet_pos_k0_v13`.
  Failures: seeds 60, 61, 63, all three exhausting the 1000-step horizon on the
  carry, all three the mechanism gap above.
- **Argmax:** v13 is the argmax over the three versions taken to a formal
  15-seed run (12 vs 10 vs 8).
- **PROVENANCE:** present, 13 entries, every one sourced to a debug-seed
  measurement or to generic controller/camera mechanics. Verified to contain no
  forbidden tokens and no `api.done` read.
- **Receipt chain:** v1 perception probe -> v2 calibration (TIP_OFFSET 0.0091,
  4/4) -> v3 3/4 -> v4 5/8 -> v5 5/8 -> v6 5/8 -> v7 4/8 -> v8 6/8 -> v9 6/8 ->
  v10 6/8 -> v11 6/8 -> v12 6/8 -> **v13 6/8 probe, 12/15 formal** -> v14 6/8 ->
  v15 6/8 -> v16 5/8 (8/15) -> v17 5/8 (10/15); plus three diagnostic probes
  (`probe_reach`, `probe_bowl`, `probe_side`) that located the mechanism gap.
- **Clean room:** no demonstrations used (k0). Every constant re-derived from
  this cell's own debug-seed RGB-D, gripper and eef observations, or from the
  controller/camera mechanics read out of the harness. Seeds 1-50 never touched.
