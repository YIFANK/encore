# l90abl_turn_on_stove_k0 — NOTES

Zero-demo cell. Intent: "turn on the stove". Runner: `tools/fair_run.py` only.

## Scene (derived from debug seeds 51/53/55/57, cam_high RGB-D)
- Table plane z = 0.901 (modal deprojected z).
- Left: a shallow pan/plate. Centre: a moka pot (top ~1.03).
- Right: a **stove slab**, flat top ~table+0.025, footprint x[-0.145,+0.045],
  y[+0.105,+0.295], with a raised burner ring at ~table+0.031.
- Just behind the slab (more -x): a **rotary knob** = a disc, top ~table+0.022,
  half-width ~0.045, carrying an upright **lever bar** whose flat top face is an
  80 x 25 mm rectangle at ~table+0.059, centred on the knob's pivot.
- Knob pivot varies only slightly with seed: c = (-0.201..-0.212, +0.194..+0.205).
- Bar detection (band table+0.048..+0.070 inside the knob window, largest
  grid-connected component, PCA long axis) is exact and stable: n≈450 pts,
  th0 = +178.5..+179.0 deg on every seed probed.

## Fingertip offset
The closed gripper pressed onto the table stalls with eef at table+0.0075, and
an eef at table+0.042 engages the bar (top table+0.059) while clearing the disc
(top table+0.022). Both agree: **fingertip ≈ 8 mm below the eef frame.**

## Version chain
| ver | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | perception probe | wide height map; found slab + knob | scene mapped |
| v2 | fine map of the stove region | 10 mm map; bar/disc/slab heights split cleanly | geometry fixed |
| v3 | one 150 mm blade push in +y at the bar's far end | stalls 48 mm in, bar rotates 0.0 deg | 0/4 |
| v4a | stepped +y push, 15 mm hops, 133 mm overshoot | eef frozen at y=0.1698 for 8 hops, dth=-0.3 deg | **+y is the joint limit** — 0/3 |
| v4b | same, -y | bar rotates **+12.9 deg** then stalls | **free sense = increasing atan2 angle** — 0/3 |
| v6 | arc-sweep the blade about the pivot | 1.5 s descent starved: eef settled at table+0.063, fingertip resting ON the bar top; dth=+1.6 deg | 0/4 — descent bug |
| v7 | v6 + descend clear of the disc at r=0.085, confirm height, move in radially | dth +5..+7 deg; GIFs show the bar springing back upright on the failures | **2/4** |
| v7b | v7 on the other probe seeds 59/61/63/65 | dth +5..+7 deg, same spring-back | 0/4 (v7 = **2/8**) |
| v8 | straddle the bar with the open jaws, close, twist the wrist about world +z | grip effort 3.0 held throughout; bar turned -24.8 deg and stayed | **4/4 probe, 15/15 formal** |

## Law (candidate, falsifiable)
**A sprung detent needs a non-slipping drive.** A blade push and an arc sweep
both rotate this knob a few degrees and then lose it — the contact slides off
the lever and the knob springs back upright, so the rotation never latches.
Straddling the lever with the open jaws and closing on it converts the same arm
motion into a positive angular drive: one wrist twist about the pivot carries
the knob past its detent every time. Receipt on debug seeds: identical
perception and identical pivot, blade push 2/8 vs grasp+twist 15/15.
Corollary: the jaws separate along base y and the lever lies along base x, so
the straddle needs no pre-rotation — check the jaw axis against the feature
axis before adding a wrist alignment.

## DECLARATION
- **Frozen version: v8.** `packs/l90abl_turn_on_stove_k0/program.py` md5
  `b0b2564142fcf22efc13c15dcd89840c` == `program_v8.py` (same md5).
- **Selection receipt: 15/15** on the full 15 debug seeds 51-65,
  `results/sel_l90abl_turn_on_stove_k0_v8` (`benchmark_success: true` x15,
  no `program_error`, no `error`).
- Per-version receipt chain: v3 0/4 (`fs_..._v3`), v4a 0/3 (`fs_..._v4a`),
  v4b 0/3 (`fs_..._v4b`), v6 0/4 (`fs_..._v6`), v7 2/4 (`fs_..._v7`) + 0/4
  (`fs_..._v7b`), v8 4/4 (`fs_..._v8`) then 15/15 (`sel_..._v8`).
- PROVENANCE present in program.py, covering KNOB_WIN, BAR_ZLO/BAR_ZHI,
  TIP_OFF, GRASP_Z, TURN_DIR/TURN_STEP/NTURN, APPROACH_R, ZTOL, TABLE_Z —
  every constant sourced to a debug-seed measurement or generic controller
  mechanics. No pack (zero-demo cell). No `.done` read (grep: 0).
- Archived: program_v1..v8 in the pack dir. STOP.
