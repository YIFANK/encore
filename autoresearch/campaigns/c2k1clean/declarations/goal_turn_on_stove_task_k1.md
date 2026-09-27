# c2k1clean / goal_turn_on_stove_task_k1

Intent served to `api.instruction()`: **"Turn off the stove"**.
Pack `language`: **"turn on the stove"**. Graded bit: the environment's own
benchmark predicate for `turn_on_the_stove.bddl`. The instruction and the
demonstration disagree in direction; the predicate is the on-predicate, so the
program turns the knob the way the demo turns it and treats the wording
difference as a goal perturbation rather than a change of mechanism. Flagged
here because it is the one thing about this cell a reader should know.

## What the pack says

K=1, 80 steps, 3 keyframes, `ee_path`/`ee_path6`/raw `actions`/`action_scale`.

* keyframes: t0 at the home pose, t41 at `[-0.4328, 0.2209, 0.9279]` with the
  gripper command flipping to close, t79 at `[-0.4197, 0.2020, 0.9308]` — the
  eef barely moves between them, so the last phase is a **turn in place**.
* the 6-vector pose tail is an axis-angle **rotation vector**, not euler rpy.
  Test: this cell measures `api.tool_rotation()` at reset as
  `[[.9984,.0005,-.0568],[.0005,-1,0],[-.0568,0,-.9984]]`, whose rotvec is
  `[-3.140,-0.001,0.089]`; the demo's t0 tail is `[3.116,-0.013,-0.211]`
  (same axis up to the pi-rotation sign ambiguity, 7 deg apart). Under
  euler-xyz the third component would have to be ~0, and it is -0.211.
* under the rotvec reading, t41 -> t79 in the BASE frame is
  `[0.095, 0.158, 0.925]`: **53 deg about world +z**. That agrees with the raw
  actions, whose component 5 saturates at 0.375 through the turn phase while
  components 3 and 4 stay near zero. Under euler-xyz the same pair reads as
  81 deg about world +y, which no flat dial can do.
* conclusion: the knob is a **flat dial with a vertical axis** and an upright
  rectangular tab as its lever.

## What the scene says (debug seeds only)

Perception is cam_high RGB-D -> a top-down max-z height map on a 5 mm grid,
keeping the colour of the highest hit per cell.

* modal table plane 0.9025 on every one of seeds 51-65.
* the knob is a dark disc (~0.095 m across, top 0.920) with an upright tab:
  bbox 0.080 x 0.025 m, **top 0.9603**, cells mean rgb 17-20.
* detector: largest connected patch that is dark (`rgb.max < 60`) and whose
  top lies in `table+0.04 .. table+0.08`. Validated offline on **all 15**
  debug-seed captures: exactly one candidate every time, 80-93 cells, top
  0.9603, span 0.075-0.085 x 0.020-0.025, long axis within 3 deg of world x.
  The only other in-band dark patch anywhere was 8 cells (the robot base),
  killed by `MIN_CELLS = 25`.
* the knob moves across seeds by about +-9 mm in x and +-8 mm in y, so the
  position must be perceived, not memorised.

## Version chain

| ver | what changed | probe | receipt dir |
|-----|--------------|-------|-------------|
| v1 | perception dump only, no motion | 0/4 (by design) | `fs_..._v1`, `fs_..._v1b` (11 more seeds) |
| v2 | knob perception + 4-height close sweep | **4/4** | `fs_..._v2` |
| v3 | deliberate grasp + staged +z twist | **8/8** | `fs_..._v3` |
| v4 | v3 + 3 attempts, re-perception, both turn signs | **8/8** | `fs_..._v4` |
| v5 | v4 + closed-loop aim correction at a 35 mm pre-grasp | **8/8** | `fs_..._v5` |
| v6 | correction moved to the grasp height | **7/8** | `fs_..._v6` |
| v7 | v4 control, attempt heights `[+5, +5, -8] mm` | **12/15 formal** | `sel_..._v7` |
| v8 | close gated on the height the descend ACHIEVED, 5-rung ladder | **14/15 formal** | `fs_..._v8` (5/6), `sel_..._v8` |
| **v9** | v8 + perceive once, denser ladder, skip the turn on an empty close | **14/15 formal** | `sel_..._v9` |
| v10 | v9 + +-6 mm lateral dither on the last two rungs | **14/15 formal** | `sel_..._v10` |

Probe subset for v3-v6: 51,53,55,57,59,61,63,65.

### v2 — the calibration sweep that also solved the task

Closing at eef `z = tab_top + 0.025` gives width 0.001 at effort 0.05 (air);
at `tab_top + 0.005` it gives width 0.014-0.025 at **effort 3.0**. On every
seed the eef and the gripper froze immediately after that close and the
episode came back `benchmark_success: true` — LIBERO had terminated. So the
predicate is fired by the **descend-and-close on the tab**, not by the later
wrist rotation. 4/4.

### v3/v4 — the deliberate version

Same close, then a staged rotation about world +z (15 deg steps to 90) with xy
pinned on the dial axis. 8/8 and 8/8. In every episode the logs show the eef
frozen from the close onward, i.e. the turn stages were no-ops because the
episode was already over. The turn is kept because it is the demo's mechanism
and it is free, but **on debug seeds it never fired the predicate by itself.**

### The aim-window probes (and the correction they forced)

A probe copy of v4 with the aim displaced +12 mm in y scored **1/4** (seeds
52/54/56/58, `fs_..._offset`), which looked like a tight window. The failing
episodes closed at effort 3.0 with the tab in the jaws and still did not fire,
and one re-perceived the tab at 72 deg afterwards — so a grip and even a large
turn are not sufficient; the close has to happen in the right configuration.

That motivated closed-loop aim correction, and the correction is where the
work went wrong and then right:

* **v5** corrected at a 35 mm pre-grasp and then descended on the corrected
  command. The correction converged there (residual error ~2 mm) but the
  descend then landed **14 mm off in x and 8 mm off in y** — the controller's
  standing error is height-dependent, so a bias measured at one height is the
  wrong bias at another. 8/8 anyway. Its offset probe
  (`fs_..._offset5`, +12 mm) scored 4/4, but that is an artifact: v5's own
  -8 mm landing bias cancelled most of the injected +12 mm.
* **v6** corrected at the grasp height, which does land the jaw centre within
  ~3 mm. It scored **7/8** — worse. The failure (ep51,
  `v6/program_ep51.log`) is the point of this whole cell: the corrective
  moves let the controller settle, the achieved eef z dropped from ~0.969
  (every unconverged version) to 0.961, and the close came back width 0.001
  at effort 0.05 — **air**, 5 mm from the tab centre. Its attempt 2 did get a
  real 0.029 m hold at effort 3.0 and turned the knob through 75 deg with the
  grip intact, and still did not fire.
* window probes on v6 at y offsets -20 / -12 / +12 mm scored 4/4, 3/4, 3/4
  (`fs_..._wm20`, `fs_..._wm012`, `fs_..._wp012`). So there is no sharp
  geometric aperture; the v4 1/4 reading was small-sample noise, and lateral
  aim within +-20 mm is not what decides the outcome.

**Verdict.** What decides it is the *descend*, not the aim: a single
unconverged `api.move` to `tab_top + 0.005` leaves the controller pressing
down at an achieved z of ~0.969, and closing under that press turns the dial.
Anything that lets the arm converge first (extra lateral moves) trades the
press away and the jaws shut on air. Recorded as the cell's one real finding.

### v7 — the frozen candidate

v4's control exactly (one unconverged descend per attempt, no lateral
refinement), with the attempt heights changed from `[+5, -8, +13] mm` to
`[+5, +5, -8] mm`: `tab_top + 5 mm` held at effort 3.0 on 16/16 first
attempts across v3 and v4, so the second shot should repeat the proven height
(with a fresh capture, since the knob may have moved) rather than try a worse
one. Three attempts, turn sign `+, +, -`.

v7 and v4 both scored **12/15** formally, failing the *same* three seeds —
52, 60, 62 — every one of them even, i.e. none of them in the odd probe
subset that had read 8/8. A probe subset is not a verdict.

### The finding: the achieved descend height decides the task

Pulling the achieved eef z of the first descend out of all 15 formal
episodes of v7 against their success bits gives a clean separation, with the
tab top at 0.9603 on all 15:

| | achieved eef z | as tab_top + |
|---|---|---|
| 12 successes | 0.9676 .. 0.9706 | +0.0073 .. +0.0103 |
| 3 failures (52/60/62) | 0.9648 .. 0.9662 | +0.0045 .. +0.0059 |

The v6 air-close (0.9609) and the v2 sweep's air-close at a commanded 0.985
(achieved 0.9806) sit outside on either side, so the working band is bounded
both ways: too low and the jaws take the tab by its shank and shove the whole
dial (the eef then drops ~20 mm and the knob never turns); too high and they
shut on nothing.

It is **not** an aim problem. Lateral offsets of -20/-12/+12 mm scored
4/4, 3/4, 3/4; the landed y error is +-9 mm on successes and failures alike;
and the base disc's centre — a far better conditioned estimate than the
25 mm tab — agrees with the tab's bbox midpoint to within 2.5 mm on every
seed, so there was no better centre to aim at. The commanded height is not
the lever either: the same command produced achieved heights from +0.0045 to
+0.0103 depending on how the fingers happened to meet the tab.

So v8/v9 stopped trying to aim and started **measuring**: descend, read the
achieved height, and close only if it fell inside the gate; otherwise lift
away without closing, leaving the knob untouched (the gate is only known to
predict the outcome on an undisturbed knob — v6 ep51 and v8 ep60 both got a
real effort-3.0 hold and a 75-85 deg turn on an *already disturbed* knob and
still did not fire). Then walk the descend command up and down in a ladder
until some rung lands in the gate. That repaired 52 and 60 and cost nothing
on the 12 seeds that already worked: **14/15**.

Two further things v9 fixes over v8: the parked arm clips the tab in
cam_high, so v8's per-rung re-perception watched the measured x-span fall
0.080 -> 0.045 -> 0.025 and walked the bbox midpoint up to 15 mm off the dial
axis; v9 perceives once, on the pristine reset frame. And a rung whose close
comes back empty now skips its 6 turn commands instead of spending horizon.

## Mechanism gap (ep62)

ep62 fails under v8, v9 and v10 alike. Its tab measures the thinnest of the
15 seeds (y-span 0.020 against 0.025) and in v9 four rungs closed from inside
the gate: three came back empty (width 0.001) and the fourth took a real
0.0106 m hold at effort 3.0 and turned 90 deg about +z with the grip intact,
and the predicate still did not fire.

Falsifiable statement of what is missing: **the program can put the jaws on
the tab and rotate the dial, but it has no sensor for how far the dial has
actually turned, and on this seed the turn it achieves is not the turn the
predicate wants.** Re-perceiving the tab after a turn does report a rotated
long axis (72 deg on one probe, 85 deg on another), but that reading is taken
with the arm parked over the knob and clipping it, so it is not trustworthy
enough to close a loop on; and the 180-deg symmetry of the tab makes +72 and
-108 deg indistinguishable from its long axis alone, so the reading cannot
even resolve the sign of the turn. Closing that loop needs either an
unoccluded view of the knob between turn stages (which would need a retreat
the horizon can barely afford) or an asymmetric feature on the dial that
breaks the 180-deg ambiguity. Neither was available within this cell's
budget. Three versions were spent on ep62 (v8's ladder, v9's clean single
capture, v10's +-6 mm dither) and none moved it.

## DECLARATION

* **Frozen version: v9.** `program.py` md5 `a2e53ce0f059f765b8aa688e24011820`
  == `program_v9.py` (verified on the Mac and on the cluster).
* **Selection receipt: 14/15** on the full 15 debug seeds (51-65),
  `results/sel_c2k1clean_goal_turn_on_stove_task_k1_v9`. Only ep62 fails.
* v9 is the argmax of a three-way tie at 14/15 (v8, v9, v10 all 14/15, all
  failing only ep62). v9 is chosen as the simplest of the three: v8 carries
  the re-perception occlusion bug, and v10's lateral dither was shown to do
  nothing.
* Per-version receipt chain: v1 `fs_..._v1` + `fs_..._v1b` (perception dump,
  no motion) -> v2 `fs_..._v2` 4/4 -> v3 `fs_..._v3` 8/8 -> v4 `fs_..._v4`
  8/8 -> v5 `fs_..._v5` 8/8 -> v6 `fs_..._v6` 7/8 -> v7 `sel_..._v7` **12/15**
  and v4 `sel_..._v4` **12/15** -> v8 `fs_..._v8` 5/6, `sel_..._v8` **14/15**
  -> v9 `sel_..._v9` **14/15** -> v10 `sel_..._v10` **14/15**.
  Envelope probes (not selection candidates): `fs_..._offset` (v4, +12 mm)
  1/4, `fs_..._offset5` (v5, +12 mm) 4/4, `fs_..._wm20` 4/4,
  `fs_..._wm012` 3/4, `fs_..._wp012` 3/4.
* PROVENANCE present in `program.py`: 13 entries, every one sourced to this
  pack, a debug-seed measurement, or generic controller/camera mechanics; all
  `allowed: True`. Checked against fair_run's own gate logic offline —
  no forbidden tokens, zero `.done` attribute reads, no unsourced entries.
* Splits respected: every run above used `--split debug` with episodes drawn
  only from 51-65. Seeds 1-50 were never touched.
