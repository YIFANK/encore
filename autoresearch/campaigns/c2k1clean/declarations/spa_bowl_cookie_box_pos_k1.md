# c2k1clean / spa_bowl_cookie_box_pos_k1

Intent: *pick up the black bowl next to the cookie box and place it on the plate*
Runner: `tools/fair_run.py` only. Debug seeds 51-65. Pack = K=1 demo + 3 keyframes.

## Pack read-out (the only task-specific input)

| field | value |
|---|---|
| keyframe t=0 | ee (-0.2149, -0.0032, 1.1593), gripper open (0.0362/-0.0362) |
| keyframe t=48 | ee (0.1486, -0.0325, 0.9263), gripper **close** commanded |
| keyframe t=122 | ee (0.0845, 0.2333, 0.9332), gripper **open** commanded, state (0.0059, -0.0027) |

Held gap at release = 0.0059 + 0.0027 = **8.6 mm**; open gap = 78.8 mm.

## What the surveys established

**s4** (seeds 51/53/55) — table plane `table_z = 0.9010` on every seed. A contact
ladder on a clear patch stalls at eef z = 0.9241, so **fingertip_z = eef_z - 0.023**.
The demo's close (0.9263 = table+0.0253) therefore puts its *fingertips on the table*,
4 cm below a 42-51 mm bowl rim.

**Mechanism.** A 0.078 m jaw opening cannot straddle a 0.106 m bowl, so the demo
grasp must be a **rim pinch** — one finger down inside the bowl, one outside,
squeezing the wall. The default wrist (`rotation=None`) has tool_y = world −y, so
**the jaws close along world Y** and the pinch point is the bowl centre offset along
±y by the wall radius.

**s5** — offsetting along **+x** (a tangential pinch) closes to 0.060 with effort 3.0
and is ejected on the lift, shoving the bowl 48 mm. Confirms the offset must be
along the jaw axis. (Trials must re-perceive: a shoved bowl invalidates the next one.)

**s9/s10** — the scene holds two round vessels that a **wall-radius profile**
separates cleanly and identically on every seed: the ramekin's visible radius is
flat with height (taper **0.003** over 30 mm) while the black bowl's grows **0.020**
— a rounded interior. v1/v2 grasped the *ramekin* whenever the cookie box merged
into the bowl's cluster. On the black bowl a rim-top pinch closes on air and a
bottom pinch grips but is pulled out by the lift; the mid-wall pinch holds.

**s11** — `api.grip` is **binary**: `grip(0.035)` opens the jaws fully (0.0796), so a
partial opening is unavailable. More important, `api.move` leaves a **steady ~0.010 m
proportional droop in z** and re-commanding the same target does not close it. Since
the wall radius grows 0.6 mm per mm of height, a pinch radius computed for the
*intended* depth then falls inside the cavity and the jaws shut on air.

**s6/s7/s8** — reach. The plate region is reachable (down to table+0.04) *when
approached from the central park* (0.02, 0.00, table+0.30); approached directly it
stalls. Repeatedly commanding into a blocked pose **wedges the controller
permanently** (a single push then retreat is recoverable). The −y side of the target
bowl drives the outer finger into the fixture at y < −0.166.

## Version receipts (probe = seeds 51,53,...,65)

| ver | change | probe |
|---|---|---|
| v1 | rim pinch ±y, clearance by centre distance, eef aimed at plate | 0/8 |
| v2 | signed distance to inflated footprints; aim the *bowl* at the plate | 0/8 |
| v3 | taper picks the bowl; staged descent; carry waypoints | **2/8** (first successes) |
| v4 | fix side-ranking identity bug; converging descent; carry at +0.20 | **4/8** |
| v5 | deeper pinch (fingertip 0.017), carry at +0.30 | 2/8 |
| v6 | v4 grasp + 180° yaw so the bowl hangs to −y of the tool | 3/8 |
| v7 | **droop-cancelled landing** + radial correction at the achieved height | 1/8 (7/8 grasps held) |
| v8 | v7 grasp + transit routed via the central park | **4/8** |
| v9 | −90° yaw (half the wrist excursion), two-stage lift | **5/8** |
| v10 | yaw ladder (−90/180/0) + early slip abort | **5/8** |
| v11 | retry with identical geometry, 3 attempts, slower lift | 4/8 |
| v12 | v10 + x-droop cancellation + longer squeeze settle | 3/8 |

Failure causes, in the order they were removed: wrong vessel (ramekin) → outer
finger into the fixture on the −y side → eef aimed at the plate instead of the bowl
→ controller wedge from repeated pushes → descent droop putting the pinch inside the
cavity → transit stalling on a straight-line path → the wrist's ±180° yaw ambiguity.

## Remaining mechanism gap

The pinch holds a **~6 mm sloping wall by friction alone**. On the three seeds that
still fail, the close reports effort 3.0 and a plausible width (0.0053-0.0072) and
the bowl then slides out of the jaws during the lift. Nothing in the FairApi
increases the squeeze (`api.grip` is binary and already commanded shut, and
re-commanding it mid-lift does not restore a slipping grip), and the only wall
geometry steep enough to resist the slip is the top 10 mm, where the wall is too
thin for the jaws to catch it at all. v5, v11 and v12 each attacked the retention
directly — deeper pinch, slower lift, longer squeeze — and all three scored *below*
v9/v10, so the slip is not a tuning artefact of the lift profile.

Falsifiable statement: **with a binary parallel gripper, a straight-down wrist and
this bowl's ~29° wall, the pinch's friction margin is marginal, and no choice of
pinch height or radius available from cam_high alone lifts the success rate above
~2/3 on the debug seeds.** It would be falsified by any pinch geometry that holds on
seeds 55, 61 and 65, or by a grip that survives the lift after a first-attempt slip.

## DECLARATION

See the receipt block appended below.

---

# DECLARATION

**Frozen version:** `packs/c2k1clean_spa_bowl_cookie_box_pos_k1/program.py`
md5 `2f522ee78a6443e704eacea4bede5c37` == `program_v10.py` (verified on the cluster).

**Selection receipt (full 15 debug seeds, 51-65):**
`results/sel_c2k1clean_spa_bowl_cookie_box_pos_k1_v10` — **9/15**
ok `[51,52,53,54,57,58,59,63,64]`, fail `[55,56,60,61,62,65]`.

Tie-break: `results/sel_c2k1clean_spa_bowl_cookie_box_pos_k1_v9` also scored **9/15**
(ok `[51,52,53,57,58,59,60,63,64]`). v10 was chosen because it is v9 plus two strict
supersets of behaviour — a yaw ladder that recovers the wrist configurations in which
v9's single −90° yaw silently does nothing, and an early slip abort that sets a
slipping bowl down instead of dropping it from 0.11 m — so it should degrade more
gracefully on unseen layouts.

**Per-version receipt chain (probe = seeds 51,53,55,57,59,61,63,65):**
v1 0/8 · v2 0/8 · v3 2/8 · v4 4/8 · v5 2/8 · v6 3/8 · v7 1/8 · v8 4/8 ·
**v9 5/8** · **v10 5/8** · v11 4/8 · v12 3/8.
Result dirs: `results/fs_c2k1clean_spa_bowl_cookie_box_pos_k1_v{1..12}`;
surveys `…_s4, _s5, _s6, _s7, _sv8, _sv8x, _s9, _s10, _s11`.

**PROVENANCE:** present as a top-level literal dict in `program.py`, covering the
workspace crop, the table-plane mode, the fingertip offset (debug-seed contact
ladder), the grasp depth and hold-width band (pack keyframes t=48/t=122 plus
debug-seed measurement), the jaw half-opening, the park height, the bowl/plate shape
gates and the cookie-box colour gate, and the stall tolerance. Every constant is
sourced from this pack or from my own debug-seed observations.

**Mechanism-gap note:** the residual 6/15 are all the same failure — the pinch grips
(effort 3.0, width 0.0053-0.0072) and the bowl then slides out of the jaws during the
lift, because the grasp holds a ~6 mm wall sloping at ~29° by friction alone and
`api.grip` is binary, so the squeeze cannot be increased. Three separate attacks on
retention (v5 deeper pinch, v11 slower lift, v12 longer squeeze) all scored below
v9/v10, so this is a mechanism limit rather than a tuning one. The falsifiable form
is stated in the section above.
