# c2clean / goal_open_middle_drawer_task_k0 — working notes

Intent: "open the bottom drawer of the cabinet". No demonstration pack.
Runner: tools/fair_run.py only. Debug seeds 51-65; probes on 51 (plus 53/57 for
scene variance).

## Scene, as measured (debug seeds 51/53/57, cam_high RGB-D, own deprojection)

Deprojection validated against the table plane (z = 0.901 everywhere on bare
table). Cabinet front face = the modal y of elevated points: **-0.1575 (s51),
-0.1620 (s53), -0.1720 (s57)** — so the face moves up to 1.5 cm between seeds
and must be perceived per episode.

Three handle bars protrude from that face, identical across the three seeds:

| band | z (mean) | x span | tip y | thickness |
|---|---|---|---|---|
| bottom | 0.953 | -0.006 .. 0.081 | face + 0.032 | 0.0165 in z (closed-grip width) |
| middle | 1.021 | -0.006 .. 0.081 | face + 0.032 | 0.0174 |
| top | 1.095 | -0.006 .. 0.081 | face + 0.032 | 0.0174 |

Each bar is a ⊓ bracket: crossbar along base x at y ≈ face+0.022, two stems at
the ends (x ≈ 0.00 and 0.072) joining the face. The slot between crossbar and
face is ~11 mm deep in y.

Gripper geometry, measured (v12/v17, pressing on the bare table at z=0.901):
- straight-down wrist: eef stops at **z=0.9095** open and **0.9097** shut →
  the fingertips sit at the eef (8 mm below it).
- horizontal (side) wrist: eef stops at **z=0.9967** open, **~0.999** shut →
  the hand hangs **~0.096** below the eef, independent of jaw state.
- rolled side wrist (jaws along x): floor **z≈0.941** → hangs ~0.041.
- open jaw width 0.079 → pads 0.0395 either side of the eef; the shut finger
  pair is ~15 mm thick in y (v30: the face stops the eef at y = face + 0.0075).

Controller mechanics (re-derived): every api.move costs at least 60 sim steps
(horizon 1000 → ~16 moves per episode), move_pose starves badly — the same
command issued once reaches y=-0.127 and issued with a deliberate overshoot
reaches y=-0.150 (v26 vs v27). All approaches below use overshoot or an
error-cancelling re-issue.

## Version log (hypothesis → evidence → verdict)

- **v1** ship cam_high/cam_arm_wrist RGB-D through api.log (chunked; api.log
  truncates at ~2000 chars, so chunks are 1900). → scene reconstructed offline.
  Verdict: perception pipeline works.
- **v2/v3** side pinch (jaws vertical, approach -y) on the middle bar, stepped
  approach. → x centroid contaminated by a stray cluster, approach starved and
  burnt the step budget. Verdict: need robust percentiles + closed loop.
- **v4** closed-loop approach + depth ladder with the closed width as receipt.
  → grip at eef y = ytip+0.010, width 0.0165, effort 3.0. Pull yanked 0.08 m in
  one move and the bar escaped. Verdict: mechanism found, pull too coarse.
- **v5** same, seated deeper (dy=-0.005), pull in 0.025 m steps. → held width
  0.0173/effort 3.0 through **0.16 m** of travel; GIF shows the middle drawer
  fully out. **benchmark_success = false (0/2).**
- **v6/v8** same on the bottom bar. → the open horizontal gripper never gets
  near it (eef floors at z≈1.00) and burns all 1000 steps. Verdict: bottom bar
  is a different problem.
- **v7** same on the top bar. → opens it (0.11 m of travel). **false (0/2).**
- **v9/v10/v12/v13/v17/v20/v25** reach probes: horizontal wrist floor ≈ 1.000
  everywhere tested (three stations within 4 mm); tilting the approach down
  (Rx(θ)·R_side) lowers the floor (θ=55 → 0.941, θ=70 → 0.9215) but costs -y
  reach (θ=70 stalls at y≈-0.078, θ=30 at -0.109, versus -0.143 for θ=0).
- **v11** top-down pinch on the bottom bar (rear pad into the slot). → descent
  blocked at z=1.108: the rear pad hits the drawer face. Verdict: the pad+finger
  (0.0395+0.008) cannot be parked in an 11 mm slot.
- **v14/v15/v16/v18/v19/v23/v24** tilted-wrist pinches on the bottom bar, four
  roll/tilt families, approach from the front and along the bar's own axis from
  the free +x table. → every one stalls 2.5-6 cm short in -y (eef pinned around
  x≈0.09, y≈-0.10, z≈0.97). Verdict: no tilt satisfies "low enough" and
  "deep enough" at once.
- **v21** middle drawer + release + retreat (in case the predicate dislikes a
  held handle). → **false (0/2).** Verdict: not a "hand still on it" problem.
- **v22** approach the bottom bar with the jaws SHUT then open around the bar.
  → shut changes nothing: the floor is the hand, not the fingers (z≈0.999).
- **v26/v27** rolled side wrist (jaws close along x → thin in y): reaches
  **y=-0.150 at z=0.967** with an overshoot command. Best depth achieved.
- **v30** rolled+shut, drive to y=-0.150 then descend into the slot. → descent
  blocked at z=0.969: the 15 mm pair is resting on the bar's top, i.e. it does
  not fit the 11 mm slot. Pull moves nothing (face_y unchanged).
- **v31** press-drag: press the shut pair onto the bar's top and drag +y 0.10 m.
  → drawer does not move (face_y -0.1575 → -0.1575; GIF shows it shut).
- **v32** wedge-hook: go under the bar (z commanded 0.920) then lift and drag.
  → the approach stalls at y=-0.1185 because the rolled hand (±0.041 in z about
  the eef) fouls the bar's tip; the eef never gets under the bar.
- **v33 (frozen)** opens every bar the arm can actually engage — middle, then
  top — each verified by closed width 0.0174 at effort 3.0 and 0.16 m / 0.11 m
  of pull, then releases and retreats.

## Mechanism gap (falsifiable)

**Claim.** With this FairApi surface the bottom drawer's handle cannot be
engaged, for two independent geometric reasons measured on debug seed 51:

1. *From the side (the grasp that works on the other two bars).* The jaws must
   close vertically, which requires a horizontal wrist, and a horizontal hand
   extends 0.096 m below the eef (v12: eef floors at 0.9967 over a 0.901 table).
   The bottom bar sits 0.048 m above the table, so the eef would have to be at
   0.949 and the hand 0.047 m inside the table top. Rolling the wrist so the
   jaws close along x reduces the hang to 0.041 m, which is still larger than
   the 0.048 m gap once the bar's own 0.016 m thickness is taken out — v32
   stalls with the hand against the bar's tip at y=-0.1185.
2. *From above/behind (hook or top-down pinch).* Anything that catches the bar
   on a +y pull must sit in the slot between the bar and the drawer face. The
   slot is 11 mm deep (bar back at face+0.011); the shut finger pair is 15 mm
   thick (v30: the face stops the eef at face+0.0075 and the descent then stops
   on top of the bar at z=0.969), and a single open pad is 0.0395+0.008 from the
   eef, which puts it inside the face (v11, blocked at z=1.108).

**Prediction that would falsify it:** any program that puts the eef below
z = 0.96 at y ≤ -0.13 in front of this cabinet, or that reports a closed
gripper width in 0.008..0.035 with effort 3.0 while the eef is within 2 cm of
(0.038, -0.136, 0.949). Across 20 probe versions and four wrist families, the
deepest low pose reached was (0.046, -0.150, 0.967) and the lowest deep pose
(0.047, -0.1185, 0.942) — both a bar-thickness away from what a grip needs.

**Receipt that the target really is the bottom drawer:** the middle bar (v5,
0.16 m of travel, drawer visibly out in the GIF) and the top bar (v7, 0.11 m)
both open cleanly, with and without releasing and retreating (v21), and the
benchmark bit is false in every one of those episodes. The instruction also
says "bottom".

## DECLARATION

- Frozen version: **program.py == program_v33.py**, md5
  `011b5af6330b8ef339b47f920ed62220` (both files).
- Selection receipt (full 15 debug seeds 51-65):
  `results/sel_c2clean_goal_open_middle_drawer_task_k0_v33` — **0/15**
  (13/15 seeds opened both reachable drawers with verified grips; seeds 56 and
  64 failed the grip receipt and opened neither — the benchmark bit is false in
  all 15 either way).
- Per-version receipts: fs_c2clean_goal_open_middle_drawer_task_k0_v1 … _v33
  under results/ on AbakaAI (v5 0/2, v7 0/2, v21 0/2, v33 0/2 on 51,53; the
  rest are probes).
- PROVENANCE: present in program.py, every constant sourced to a debug-seed
  measurement or to generic controller/camera mechanics.
- Status: **mechanism-blocked**, documented above; argmax version declared
  (all versions tie at 0, v33 is the one that performs the full mechanically
  possible behaviour and self-verifies each grip).
