# c2k1clean / spa_bowl_between_task_k1 — working notes

Intent: *Pick the akita black bowl **not** between the plate and the ramekin and
place it on the plate.*
Pack language (the demo's own task): *pick up the black bowl **between** the plate
and the ramekin and place it on the plate.* → the K=1 demo grasps the
**anti-target**; only its grasp *mechanics* transfer, not its target identity.

## Scene model (derived from cam_high RGB-D on debug seeds 51/53, v0/v2)

| prop | x | y | footprint ext | top − table | mean rgb |
|---|---|---|---|---|---|
| table plane | — | — | — | z = 0.9012 | — |
| cabinet/stove mass | 0.05 | −0.22 | 0.58 | 0.226 | (66,65,64) |
| akita bowl A | −0.079 | 0.206 | 0.110 | 0.050 | (104,105,102) |
| akita bowl B | −0.197 | 0.322 | 0.110 | 0.050 | (98,99,96) |
| ramekin | −0.216 | 0.193 | 0.086 | 0.042 | (116,117,118) |
| plate | 0.053 | 0.236 | 0.137 | 0.019 | (152,141,138) |
| cookie box | 0.081 | 0.031 | 0.082×0.060 | 0.019 | (93,66,46) |

Frame: −x is toward the robot base / top of the agentview image; +y is to the
right of the image; camera at (0.659, 0, 1.610) looking back and down.

Identity rule (no demo needed): the two mutually most-similar raised round
clusters are the **twin bowls**; the flattest widest light disc is the **plate**;
the remaining raised vessel is the **ramekin**. Target = the bowl with the larger
perpendicular distance to the plate–ramekin **segment**. On 51/53 the scores are
0.009 (between) vs 0.1235 (not between) — a 13× margin, not a close call.

## Version log

### v0 — perception probe (seeds 51,53,55,57), 0/4 (no motion)
Hypothesis: a table-plane height band over the cam_high point cloud separates
the props. Evidence: 8 clusters, table z = 0.9012, the table above. Verdict:
scene is fully legible from depth alone; identity needs no pack input.

### v1 — calibration probe (51,53), 0/2 (no grasp)
K = f 618.04, c (256,256). `R_home ≈ diag(1,−1,−1)` (straight-down tool, yaw ≈ 0).
A closed gripper in free air reports `width_m = 0.0010`. **One `api.move(…, 2.0 s)`
does not converge** — residual 0.107 on a 0.12 m step; must iterate.
Fingertip PCA from cam_high was contaminated by the forearm → jaw axis left
undetermined here; settled by the v2 sweep instead.

### v2a–v2d — grasp-geometry sweep (51,53,55,57), 0/4 each
Four configs, same target selection. `after_close` width / effort on ep51:

| ver | tool yaw | TCP offset from bowl centre | close w | effort |
|---|---|---|---|---|
| v2a | 0° | +x 0.050 | 0.0121 | **3.00** |
| v2b | 90° | +y 0.050 | 0.0010 | 0.05 |
| v2c | 0° | +x 0.040 | 0.0079 | **3.00** |
| v2d | 0° | −x 0.050 | 0.0010 | 0.05 |

Verdict: **the jaw axis of the straight-down tool is world x**, and the grasp is a
**rim pinch on the +x arc** (the arc facing the camera / away from the base).
0.0121 m closed width matches the pack demo's held width 0.0125 m
(`gripper_state` at t=85) — the demo does the same pinch.
−x fails: that arc is shadowed by the approach.

**Key negative result turned positive:** v2a/v2c reported `effort = 0.05` and
`width → 0.0047` *after the lift*, so the program aborted the transport — but the
`ep51_fail.gif` shows the bowl **is** riding on the gripper, tilted. The pinch
collapses into a **cradle**: the jaws close past the rim and the bowl hangs on the
fingers. `api.gripper().effort` therefore does **not** report this payload.
Verification must be by **re-perception** (target's old cell empty + a blob
hanging under the eef), never by effort.

### v3 — full pipeline with re-perception verification
Adds: Kasa circle fit on the top 8 mm of each cluster (a whole-cluster centroid
is biased ~20–30 % outward by the obliquely-seen outer wall), an
error-compensated second aim after each hover, a stepped lift, a measured carry
offset, and release over the plate.

### v4p — centre-estimator probe (51,53), no motion
The cluster **median is biased 0.026 m in −x**; the cluster bounding-box centre
and a Kasa fit of the topmost 6 mm agree to 0.001 m with a radial sd of 0.0013.
Verdict: the rim centre is (−0.171, 0.314) with r = 0.053, not the median's
(−0.197, 0.322). Every earlier offset must be re-read against the Kasa centre:
the grasps that pinched sat at radial **0.019 / 0.032 / 0.037**, the ones that
closed on air at **0.044 / 0.045 / 0.052**.

### v5 — ladder + payload check (8 seeds), 0/8
Two carries, six misses. Three defects, all instructive:
1. `payload()` counted **props resting on the table** (the mask floor was
   table+0.03), so a failed grasp still reported a payload.
2. The 4-step retry ladder **exhausted the episode**: later tries logged an
   identical frozen eef and a gripper that would not close. LIBERO's horizon is
   finite and every `goto(tries=6)` multiplies the cost.
3. `POS_TOL = 0.012` (read from `heron/robot/libero.py`): **move_cartesian stops
   12 mm out**, so repeating the same command never converges. Re-commanding
   `target + (target − eef)` turns a residual d into |d − 0.012|.

### v6/v6b — Hough perception (8 seeds), 0/8
Seeds 57/61/63/65 fuse the ramekin (and sometimes both bowls) into one
connected component, which is why v5 aborted on half the split. Replaced
component labelling with a **fixed-radius Hough over the rim band**
(z > table+0.0445 keeps bowl rims only, since the ramekin top is table+0.042):
votes 1147/970 for the two real bowls vs 335/317 for spurious peaks.
**Perception is now correct on all 8 seeds** (target margin 0.005 vs 0.12).
Motion still missed: the descent landed at radial 0.046 instead of 0.030.

### v7 — mid-height correction (8 seeds), 0/8
A lateral correction at rim depth is **blocked by the outer finger**, so the
slide never moved. Every seed landed at radial +0.049 with the descent stalled
at table+0.0445.

### v8 — pre-compensated descent (8 seeds), 0/8
Aim fixed: d = (+0.030, +0.005) on all eight seeds. Two new defects: the
"mover dead" detector false-fired on a blocked repeat, and the descent stalled
at table+0.0488.

### v9p / v9d — horizon and descent probes
`settle(s)` costs exactly 60·s steps; the arm stalls after ≈2.5 k steps, and
gifs cap at 120 frames (record_every = 5). v8 had only used ≈345 steps, so the
stall was **not** the horizon.
v9d is the decisive one: `api.move(..., rotation=R_DOWN)` routes to `move_pose`,
and from a hover at table+0.103 **one** command to table+0.020 converges to
table+0.031 with the commanded xy hit to 0.002 m. The same descent with
`rotation=None` routes to `move_cartesian`, stalls at table+0.047, and needs
six pushes to reach table+0.030. The "drift" of v6/v7 was never drift — it was
an unconverged descent.

### v10a/b/c — offset sweep (8 seeds each): **1/8, 4/8, 1/8**
radial 0.022 → 1/8, **0.030 → 4/8**, 0.038 → 1/8. First successes.
The split is a **bistable hover**:

| | approach z | descent | closed width | outcome |
|---|---|---|---|---|
| 51,57,59,63 | table+0.106 | table+0.033 in one push | 0.0150–0.0166 | carried, placed 0.002–0.005 from the plate centre → **success** |
| 53,55,61,65 | table+0.1035 | stalls at table+0.042 | 0.0010–0.0101 | dropped or placed 0.028 off |

A correctly cradled bowl profiles at **(+0.002, −0.051)** from the eef in all
four successes, and the placement that uses it is accurate to 5 mm.

### v11a/b/c — deeper descent + canonical carry offset: 0/8, 2/8, 1/8 (regression)
Two lessons. A deeper descent (table+0.031, closed width 0.0188) cradles the
bowl **askew** — it is a different grasp mode, not a better one. And
substituting the canonical carry offset for the measured profile is wrong: in
v11a the bowl really was held at (−0.024, −0.041) and the canon placed it
0.025 m off; in v11b it hung at **+y** and the canon placed it 0.108 m off.
**Always use the measured profile**; the cure is to reach the right grasp mode,
not to paper over the wrong one.

### v12a/b/c — hover lock (8 seeds): **6/8**, 3/8, 3/8
Nudging the hover to table+0.106 when it lands elsewhere converted 61 and 65
without disturbing 51/57/59/63. Its own defect: the "cradle window" gate
re-flew on a close of 0.0129 and the re-fly **shoved the bowl 0.037 m in +y**,
after which nothing worked.

### v13a/b/c — tighter lock (8 seeds): 4/8, 4/8, 4/8 (regression)
Locking xy as well as z moves the hover to (+0.030, −0.003) and the descent
then reaches table+0.032 with a *weaker* close (0.0134-0.0139). The good
descents all start from the hover offset the approach produces on its own.

### v14a/b — lift-and-check gate (8 seeds): 6/8, **7/8**
Re-flying only on an **empty** close (width ≤ 0.006, nothing disturbed) keeps
v12a's gains without its damage. v14b (3 tries) → 7/8 probe, **11/15 formal**.

### v15 — payload verification: 7/8 probe, **11/15 formal**
Seed 53 exposed a phantom grasp: the bowl was shoved 0.048 m aside, which
empties its cell, so the cell test alone reported a grasp of nothing. A cradled
bowl profiles **1900–2400 points** in the 10–50 mm slices under the eef; an
empty gripper profiles 66–69. Gate on that.

### v16 — full hover-pose lock: 9/15 (regression)
Forcing the canonical pose with extra corrective moves changes the arm's
internal configuration; the pose is a *symptom* of the good approach, not a
sufficient cause.

### v17a/b — firm-close gate: 10/15, 10/15 (regression)
The closed width does not cleanly separate outcomes on the full split (0.0113
succeeded, 0.0140 failed), and re-grasping a nudged bowl rarely recovers.

### v18a/b/c — release height: **12/15**, **12/15**, 11/15
Measured across the v15 selection run: a firm carry lands **exactly** where the
profile says (ratio 0.92–1.01 in y, with a systematic −0.004 m in x), while a
loose carry slips a further 0.030 m in −x during the drop. Releasing at
table+0.072 instead of table+0.082 removes that slip and fixes seed 54.
Remaining: 53, 56, 62.

### v19a/b — forced deep push: 12/15, 12/15 (no change)
Re-commanding a stalled descent to table+0.000 does press deeper (table+0.025
instead of creeping 1 mm per push) but does not change the outcome: those
descents were already outside the window in **radial offset**, not depth.
The real discriminator, read off the v19 logs:

| | hover offset | push0 | closed width | outcome |
|---|---|---|---|---|
| good | (+0.034, +0.011) | d = +0.044…+0.045, dz = 0.033 | 0.0150–0.0166 | placed 0.002–0.009 from the plate |
| stalled | (+0.026…+0.032, ≈0.000) | d = +0.048…+0.054, dz = 0.038–0.043 | 0.0010–0.0131 | empty or phantom |
| deep | (+0.029, −0.001) | d = +0.040, dz = 0.031 | 0.0140 | cradled askew, 0.030 off |

### v20a/b — z-only lock / no lock: 8/15, 8/15 (regression, hypothesis refuted)
Both variants score identically, so the ingredient that matters in the v12
hover lock is exactly the part I suspected of harm: its **xy re-mirroring**.
Removing it costs four seeds. The lock stays as it is.

### v21 — converged release descent
v19's logs show the release descent also stops POS_TOL short (seed 56 released
from table+0.0836 when table+0.072 was commanded), and every extra millimetre
of drop is a millimetre the bowl can slide. Adds one mirrored correction.
**Result: 10/15** (fails 53, 54, 56, 60, 62) — a regression, and an instructive
one. Driving the release all the way down to table+0.072 sets the bowl on the
plate while it is still in the jaws; opening then drags it. The *unconverged*
release of v18a, which stops ~0.012 m high, is the better one. v18a's 12/15
stands as the argmax.

### v22 — v18a, cleaned for the freeze
No behavioural change: removes three constants left dead by earlier versions
(COMP_X, COMP_Y, Z_MID), renames two PROVENANCE keys to match the constants
they document (OFF, Z_APPROACH) and adds entries for N_TRY and the Hough grid,
so PROVENANCE now covers every calibrated constant exactly. Re-run formally on
all 15 debug seeds to carry its own receipt.

### v23 — command-corrected descent
v19 established that a stalled descent cannot be rescued by pressing harder:
the controller stops POS_TOL from its command, so a repeat creeps ~1 mm. v23
instead re-flies to the hover and corrects the *command* by the measured miss
(`cmd -= landing − 0.045`), accepting only a landing in the window the cradling
grasps occupy (radial 0.041–0.048, depth ≤ table+0.037).
**Result: 11/15** (fails 53, 55, 56, 62) — a regression. The correction *works*
(ep53's reflies land the descent at radial 0.037–0.039, inside the window) but
those reflown descents arrive at table+0.029–0.031, the **too-deep** regime,
and the jaws close on air (0.0010) every time. Steering the landing in xy
drags the landing in z with it; the two cannot be set independently.

---

## DECLARATION

**Frozen version: v22** — `packs/c2k1clean_spa_bowl_between_task_k1/program.py`,
md5 `1d8021bc59ae05c7e09d0e40b142c6f0`, identical to the archived
`program_v22.py` (same md5). v22 is v18a with dead constants removed and the
PROVENANCE keys made to match the constants they document; it reproduces v18a's
receipt seed-for-seed.

**Selection receipt (full 15 debug seeds): 12/15**
`results/sel_c2k1clean_spa_bowl_between_task_k1_v22r` — 15 rows, one per
episode, no duplicates, single program `program_v22.py`.
Passes 51, 52, 54, 55, 57, 58, 59, 60, 61, 63, 64, 65. **Fails 53, 56, 62.**

> An earlier directory, `sel_..._v22`, holds 25 rows: the first v22 launch was
> still alive when the session was interrupted and kept appending after the
> directory was recreated, so episodes 56–65 are doubled there. It is **not**
> the receipt. `sel_..._v22r` is the clean re-run and is the one that counts.

**PROVENANCE**: present as a top-level literal dict, 16 entries, every entry
`"allowed": True`, covering every calibrated constant in the file (verified by
AST walk). The program reads no `api.done` and touches only
`capture / eef / grip / gripper / instruction / log / move / settle`.

### Receipt chain (every formally-probed version)

| ver | what changed | probe (8 seeds) | formal (15 seeds) |
|---|---|---|---|
| v0 | perception probe | 0/4 | — |
| v1 | camera + jaw-axis probe | 0/2 | — |
| v2a–d | grasp-geometry sweep | 0/4 each | — |
| v3 | re-perception verification | 0/8 | — |
| v4p | centre-estimator probe | 0/2 | — |
| v5 | retry ladder + payload check | 0/8 | — |
| v6/v6b | fixed-radius Hough perception | 0/8 | — |
| v7 | mid-height lateral correction | 0/8 | — |
| v8 | pre-compensated descent | 0/8 | — |
| v9p/v9d | horizon + descent probes | 0/1, 0/2 | — |
| v10a/b/c | radial-offset sweep 0.022/0.030/0.038 | 1/8, **4/8**, 1/8 | — |
| v11a/b/c | deeper descent + canonical carry | 0/8, 2/8, 1/8 | — |
| v12a/b/c | hover z-lock + cradle-window gate | **6/8**, 3/8, 3/8 | — |
| v13a/b/c | lock xy as well as z | 4/8 each | — |
| v14a/b | re-fly only on an empty close | 6/8, **7/8** | 11/15 (v14b) |
| v15 | payload-profile verification | 7/8 | 11/15 |
| v16 | full hover-pose lock | — | 9/15 |
| v17a/b | firm-close gate 0.0145 / 0.0130 | — | 10/15, 10/15 |
| v18a/b/c | release at table+0.072 / 0.065 / 0.082 | — | **12/15**, **12/15**, 11/15 |
| v19a/b | forced deep push to table+0.000 / 0.010 | — | 12/15, 12/15 |
| v20a/b | z-only lock / no lock | — | 8/15, 8/15 |
| v21/v21b | converged release descent | — | 10/15, 10/15 |
| **v22** | **v18a cleaned (frozen)** | — | **12/15** |
| v23 | command-corrected descent re-fly | — | 11/15 |

### Mechanism-gap stop

The three residual failures are one mechanism, and it is **not** a perception
or a belief failure. Perception is exact on all 15 seeds: the fixed-radius
Hough finds both bowl rims to ~0.002 m even where they fuse with the ramekin
into a single depth cluster, and the "not between the plate and the ramekin"
test separates the two bowls by 0.005 m vs 0.12 m — a 20× margin.

**The missing mechanism is a contact-referenced grasp.** The bowl's outer rim
radius is 0.0545 m and the gripper's full jaw span is 0.0799 m (±0.040 m), so
the bowl cannot be grasped across: the only grasp available is a rim pinch,
and on this object the pinch *collapses into a cradle* — the jaws close past
the thin rim wall (closed width 0.015 m against 0.0010 m in free air) and the
bowl rides on the fingers. `api.gripper().effort` never reports that payload
(`get_external_effort` returns 0.0 on LIBERO, and `effort` is just
`grip_cmd > 0 and gap > HELD_MIN_GAP`), which is why the carry has to be
verified by re-perception instead.

Whether the cradle forms at all is decided by where the descent *ends*, and the
window is about **±0.003 m in radial offset (0.041–0.048 from the rim centre)
and ±0.002 m in depth (table+0.032…0.035)** — measured across ~40 logged
descents. LIBERO's position controller stops `POS_TOL = 0.012 m` from its
command, so no single command can be placed inside that window on purpose. The
good landings are produced by the arm's own settling, not by aiming.

Falsifiable form: *if `api.move` could be replaced by a contact-stopped descent
(stop on first contact) or by a width-triggered close (close when the jaw gap
reports the rim between the fingers), seeds 53, 56 and 62 would convert.*
Everything reachable through the present API was tried and each is recorded
above with its receipt:

- steer the landing with a mirrored correction — v8: aim fixed to ±0.001 m, the
  descent then stalls 0.012 m high;
- correct laterally at rim depth — v7: physically blocked by the outer finger;
- lock the whole hover pose — v16: 9/15, the pose is a symptom, not a cause;
- lock z only, or not at all — v20: 8/15 both, the lock's xy re-mirroring is
  load-bearing;
- press a stalled descent deeper — v19: presses, but those landings were wrong
  in radial offset, not depth; no change;
- correct the *command* from the measured landing and re-fly — v23: lands in
  the xy window and immediately overshoots in z into the too-deep regime; 11/15;
- gate on the closed width and re-grasp — v12a/v17: the width does not separate
  outcomes cleanly on the full split (0.0113 succeeded, 0.0140 failed) and a
  re-fly shoves the bowl up to 0.037 m, making the retry worse than the miss.

Evidence on the three seeds, from the frozen version's own receipt:

- **53** — never enters the window in three attempts: landings
  (+0.048, table+0.038), (+0.050, table+0.040), (+0.048, table+0.040); closes
  0.0129 / 0.0010 / 0.0010; `payload_n = 0`. No carry.
- **56** — overshoots into the deep regime on the first push
  (+0.040, table+0.0305), cradles the bowl **askew** (it hangs at +0.045 m in
  y instead of −0.051 m) and places it 0.030 m from the plate centre.
- **62** — first attempt empty; second lands (+0.048, table+0.0354) and cradles
  the bowl diagonally (−0.043, −0.038), placing it 0.042 m off.

Argmax declared: **v22, 12/15 on the full debug split.**
