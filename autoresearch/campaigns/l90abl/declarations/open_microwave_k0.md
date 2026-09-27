# l90abl / open_microwave_k0 — zero-demo cell, FAIR_PROTOCOL v1.1.1

Intent: "open the microwave" (KITCHEN_SCENE7). No demonstration pack; every
constant below was measured from debug-seed observations (seeds 51–65) through
`tools/fair_run.py` only. Campaign LAWS.md was empty at session start.

## The scene, as measured (v1–v6, perception only)

| fact | value | how |
| --- | --- | --- |
| table plane | z = 0.9012 | median depth of the workspace crop, identical on 51/53/57 |
| one body | footprint x[-0.17,+0.17] y[-0.35,-0.13], top table+0.207 | cam_high point cloud, 4-connected components on a 1 cm occupancy grid |
| robot-facing (+y) face | plane at y = -0.1456 (seed 51), -0.1315 (seed 54) | max-y per (z,x) cell at 5 mm |
| window | black patch x[-0.16,-0.10] on that face | RGB (0,0,~40) against the panel's (16,16,48) |
| **handle** | vertical D-bar, x = -0.0675, front face y = -0.1047, z = table+0.043 … table+0.193 | the only cells >15 mm proud of the panel |
| bar thickness | 0.014 m across x | the finger gap when v11 closed on it at mid-height |
| hinge | the body's -x edge, x ≈ -0.18 | leftmost panel cell; confirmed by the eef tracking a constant-radius circle about it while the leaf swung |
| eef frame | the fingertip frame | wrist camera at z=1.3445 with the eef commanded to 1.2511 saw the finger tops at 1.269, i.e. the fingers rise from the commanded point |
| palm clearance | fingertips cannot go >~45 mm below the top of what the fingers straddle | v8 commanded 88 mm under the bar top and jammed (res 0.14, 794 steps); v9 commanded 45 mm and converged (res 0.0116) |

The microwave moves ≤14 mm between seeds, but every constant is re-derived per
episode from that episode's own cam_high frame.

## Version chain

| ver | hypothesis | evidence | verdict |
| --- | --- | --- | --- |
| v1–v6 | perception only: find the door and its handle | see table above; v6 also looked straight down on the handle from 0.14 m with the wrist camera | scene solved |
| v7 | pinch the bar top-down (wrist yawed 90°, jaws across base x), walk an 18° arc about the hinge | grip real — closed 0.0227 on the 0.020 bar, effort 3.0; leaf reached 37°, pads lost it at the 54° waypoint | 0/2 |
| v8 | reach the grasp in one combined move to save steps | palm fouled the bar: stalled at z=1.1276, res 0.14 → 0.18, 794 steps, no grasp | 0/2, but it measured the palm clearance |
| v9 | grasp height from the measured bar top (−0.045), 12° steps | waypoints tracked to <13 mm; leaf to 44°, then slip | 0/3 |
| v10 | yaw the wrist to the MEASURED leaf angle, 8° steps | removed the 4–6° wrist lead; leaf to 43°, gap prised 0.0227→0.0390 then empty | 0/3 |
| v11 | side grasp at mid-bar (tool along −y), clear of both brackets | closed 0.0140 (the bar's true thickness); same ceiling, 40°, gap 0.0140→0.0405 | 0/3 |
| **v12** | stop pinching: **hook** — jaws held open, straddling the bar along the door normal, rear finger bearing on the bar's back face | **14/15** (`results/sel_l90abl_open_microwave_k0_v12`); episodes terminate at sim step 40 | works, but by accident |
| v13 | stage v12's approach: turn the wrist clear of the body, then translate | proved the approach side wrong — tool along −x asks the arm to reach behind itself; move died 0.194 m short | 0/4 |
| v14 | same hook from the −x side, jaws left open | the +x-pointing wrist cannot hold y=-0.09; standoff missed by 0.066, slot-in by 0.094 | 0/4 |
| v15 | top-down hook (default wrist, jaws already along the door normal) | descent stalled 28 mm high on the handle's **top bracket** (res 0.0319), arc then swept air | 0/4 |
| **v16** | keep v12's motion, give it three fixed attempts at three heights (mid-bar ±0.025), wrist reset to down between them | seed 54 — the one v12 missed — converts on attempt 2 | **frozen** |

## Why a pinch cannot open this door

Opening a hinged door is, by construction, retraction along the gripper's own
approach axis, and the jaw axis swings into the pull direction as the leaf
turns. Four independent pinch geometries (v7, v9, v10, v11) therefore all show
the same signature and the same 40–45° ceiling: the finger gap is first prised
open by 15–25 mm, then goes empty at 0.001 with effort 0.05. A pinch is holding
on friction alone. The hook replaces friction with form closure — the open
rear finger bears normal to the bar's back face — and carries the leaf past 90°.

Aiming that hook is the hard part on this arm. The slot behind the bar is
closed at the top by the handle's own bracket (v15) and reaching it from +x is
outside the workspace (v13/v14). What lands it is v12's single combined
move: the wrist turn and the workspace crossing happen together and the OSC
transient carries the open jaws down across the door face and onto the bar.
v16 keeps that and only makes it repeatable, at three heights on the 0.150 m
bar, wrist returned to the down pose at y=+0.06 between attempts. The sequence
is fixed and open-loop — nothing is read back to decide whether to continue, so
on seeds where the first attempt lands, the episode has already ended and the
remaining attempts are inert.

## Candidate law (for LAWS.md)

**A hinged door defeats a pinch; hook it.** Pulling a door open is retraction
along the gripper's own axes and the jaw axis rotates into the pull as the leaf
swings, so a parallel-jaw pinch on a handle holds by friction only: measured
here, four different pinch geometries all stalled at 40–45° with the finger gap
prised open 15–25 mm before it went empty. Hold the jaws OPEN instead and
straddle the handle bar along the door normal — the rear finger then bears
normal to the bar's back face, form closure rather than friction, and the same
arc carries the leaf past 90°.

**Corollary — the palm sets the grasp height.** Between the fingers sits the
palm, so the fingertip frame cannot descend more than ~45 mm below the top of
whatever the fingers straddle. Commanding 88 mm below jammed the descent and
burned 794 control steps; the resulting silent 57 mm residual reads exactly
like a successful approach.

## DECLARATION

- **Frozen version: v16** — `packs/l90abl_open_microwave_k0/program.py`,
  md5 5af42c0a3493b378347962fcade0b810, identical to `program_v16.py`.
- **Selection receipt: 15/15 on the full 15 debug seeds 51–65**,
  `results/sel_l90abl_open_microwave_k0_v16` (every seed true; 172–191 sim
  steps, i.e. every episode terminated inside attempt 1 or 2 of the sweep).
- Runner-up receipt: v12, **14/15**, `results/sel_l90abl_open_microwave_k0_v12`
  (only seed 54 failed — the seed where the body sits 14 mm nearer in y).
- PROVENANCE present in program.py, every constant sourced to a debug-seed
  measurement or to generic controller/camera mechanics.
- No demonstration pack was read; no forbidden asset was opened; only
  `tools/fair_run.py --split debug` was ever invoked.
