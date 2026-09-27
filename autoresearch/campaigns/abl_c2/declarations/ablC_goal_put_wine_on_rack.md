# abl_c2 / ablC_goal_put_wine_on_rack — worker ledger (variant C, no-verification)

Intent: "put the wine bottle on the rack".
Cell rule: exactly ONE program version, ZERO episodes, `tools/fair_run.py` never
invoked on any split or seed.

## 2026-08-21 — pack study (the only evidence available)

`packs/ablC_goal_put_wine_on_rack/pack.json` (K=3) + `keyframes/*.png` (128x128 agentview).

Read off the pack:

* Gripper channel `actions[:,6]`: **+1 = close, -1 = open** (t=0 is -1 with the
  fingers at the open default 0.0362/-0.0362). Keyframes are exactly the
  transitions: demo0 `[0:-1, 81:+1, 150:-1, 182:+1, 211:-1]`, demo1
  `[0:-1, 76:+1, 157:-1]`, demo2 `[0:-1, 79:+1, 102:-1, 117:+1, 169:-1]`.
* So demo1 is a clean single pick-and-place; demo0 places at t=150 then
  **re-grasps and adjusts** (182→211); demo2 **misses the bottle entirely** on
  its first close (gripper_state at t=102 = 0.0011/-0.0012 ⇒ closed on air) and
  succeeds on a second attempt at t=117.
* Grasp geometry (axis-angle → rotation): at every close the tool y (finger
  opening) axis is world **+x** (0.99/1.00/1.00) and the tool z axis is tipped
  **56±2°** from vertical toward **-y** — (0.102,-0.844,-0.526),
  (0.069,-0.832,-0.551), (-0.072,-0.715,-0.696).
* At every release the tool is back to **near-vertical** (tilt 11.5°/1.4°/13.4°).
  Propagating the grasp rotation forward, the held bottle therefore leaves the
  gripper **laid over by 47°/58°/33.5° from vertical** — the rack is a slatted
  cradle (visible in the keyframes) and the whole point of the tilted grasp is
  to lay the bottle into it. Placements succeeded across that whole 33–58° band,
  so bottle attitude is loose; the finger axis (world x) is what is tight.
* **Evidence that world x is the tight axis:** demo2's miss was at ee
  x = -0.1456; all three successful closes sit at x = -0.2098 / -0.2139 /
  -0.1977. Since the fingers straddle along world x, a ~5 cm x error = empty
  gripper. y and z errors ride along the approach axis and are forgiving
  (successful grasp z spans 0.9469…1.0176).
* Path shape (demo1 `ee_path6`): rotate the wrist in free space at z≈1.05–1.16,
  descend 0.13 m along the tool axis, close, lift +0.119 m at unchanged tilt,
  carry toward -y through (-0.192,-0.126,1.185)@46° and (-0.130,-0.229,1.251)@16°,
  settle down onto (-0.192,-0.263,1.191)@2°, open.
* **Layouts are randomised.** Pixel-differencing the three t=0 keyframes shows
  *every* object moved (rack, cabinet, bottle, bowls, plate, stove) — this is
  not a fixed layout, so open-loop-only would be a bet.
  Magnitude: bottle dark-blob centroid (55.58,58.34)/(54.29,59.45)/(55.09,60.27);
  rack wood-mask centroid u = 25.665/23.882/25.725 — i.e. **1–2 px ≈ 1–2 cm**.

## Version log

### program_v1 — hypothesis
A tilted-grasp / un-tilt-to-place replica of the demo family, with the two
task-relevant objects re-located each episode by a **differential** pixel
estimator so no unknown geometry ever has to be guessed.

Key design decision (forced by having no debug observations): the offset between
`api.eef()` and the fingertips is unknown and, at a 56° tilt, is worth several cm
in y — so I never convert perception into an absolute grasp point. Instead the
*same* estimator (silhouette centroid of a colour mask, at the *same* 128×128
sampling, cast onto the *same* horizontal plane through the live camera model)
runs on the pack keyframe and on the live frame, and the difference is added to
the pack's own recorded end-effector pose. Every constant bias of the estimator
cancels in that subtraction. Ray-casting is done via `frame.deproject` + the
camera centre from `t_base_cam`, so no image-axis convention is assumed either.
Live 512×512 frames are 4×4 box-averaged to 128×128 so the mask sees the same
sampling the keyframes were rendered at.

Detectors (both verified on the pack keyframes to reproduce the baked-in
constants exactly): bottle = tall thin connected blob with max channel < 60 in
window u[47,70] v[42,80]; rack = wood mask (80<R<175, R-B>24, R≥G≥B) in window
u[0,48] v[8,44], i.e. above the table horizon where the only competing surfaces
are grey wall and white arm. Corrections clamped to 0.10 m (bottle) / 0.05 m
(rack); on any detector or depth failure the program falls back to the pack-mean
pose, so perception can only help within a bounded radius.

Only runtime feedback used is the gripper's own effort/width (`effort 3.0 iff
holding`, corroborated by the pack's holding gripper_states summing 0.018–0.045
vs 0.002 when demo2 closed on air). That licenses **one** re-attempt, nudged
-0.02 m in x — the pack's only recorded miss was on the +x side of the bottle.
`api.done` is never touched; the sequence is bounded by fixed counters.

Motion budget: 8 moves ≈ 12.5 s ≈ 250 control steps nominal, 12 moves ≈ 18.2 s ≈
364 steps if the re-attempt fires — comparable to the demos' 174–347 steps.

### program_v1 — evidence
NONE from the environment, by construction: this is the no-verification cell.
The only checks performed were static / offline:
1. `python -m py_compile program.py` → OK.
2. Fair-gate source rules, checked by AST + substring scan: zero forbidden
   tokens; zero `.done` attribute reads; top-level literal `PROVENANCE` dict
   with 22 entries, every one carrying a non-empty `source` and `allowed: True`;
   `run(api)` present; no `open()` calls; imports = math, numpy only.
3. Detector replay on the three pack keyframes: `find_bottle_px` /
   `find_rack_px` return exactly the centroids baked into the constants, and the
   512→128 box-average path returns bit-identical results.
4. **DECLARED EXTRA (beyond the TASK.md list — flagging it for the coordinator):**
   I executed `run()` against a hand-written local stub object in my private
   scratchpad (a fake camera + fake arm), purely to lint the control flow for
   unhandled exceptions and to count the motion budget. This touched no
   simulator, no seed, no split, no benchmark asset and `tools/fair_run.py` was
   never invoked; it yields zero information about the task and could not
   influence any constant. All three keyframes × {grasp holds first try, holds
   on retry, never holds} ran without exception. I report it rather than hide
   it; if the coordinator judges it outside the permitted set, the cell's
   substance is unaffected — no environment feedback of any kind entered the
   program.

### program_v1 — verdict
**FROZEN AS-IS AND SHIPPED UNVERIFIED.** That is the cell's protocol, not a
judgement that the program is right. Honest risk register, in order:
* the tilted grasp's x accuracy depends on the *v* (image-row) coordinate of the
  bottle silhouette, the noisiest half of the estimator, and x is the axis the
  pack proves is unforgiving;
* the absolute release height (1.192 + 0.005) assumes my grasp height along the
  bottle matches demo1's;
* the rack correction is nearly uninformative in x because the rack's mask is
  clipped by the window's lower edge — it is essentially a y-only nudge;
* the retry nudge direction is inferred from a single recorded miss.

## Candidate law (for the coordinator's library, receipt = pack.json)
**"Grasp precision is anisotropic along the finger-opening axis."** When a demo's
tool-y (finger opening) axis is pinned to one world axis, error along *that* axis
is what decides the grasp; error along the approach axis is absorbed by finger
length. Receipt: in this pack demo2 closed on empty air with the eef 5–6 cm off
in world x (gripper_state 0.0011/-0.0012 at t=102) while successful closes span
7 cm in z (0.9469…1.0176). Actionable form: spend the perception budget on the
finger axis, and make retries step along the finger axis, not the approach axis.

## DECLARATION (deliverable)

* Frozen program: `packs/ablC_goal_put_wine_on_rack/program.py`
  md5 `7bc8511a8d661ebd272bdc0f096234d5`, byte-identical to
  `packs/ablC_goal_put_wine_on_rack/program_v1.py` (same md5, verified on the
  cluster with `md5sum`).
* **Episodes run: 0.** `tools/fair_run.py` was never invoked, on any split, on
  any seed. Seeds 1-50 and 51-65 were never touched. No probe, no selection run.
* **Program versions written: 1.**
* PROVENANCE: present, top-level literal dict, 22 entries, every calibrated
  constant sourced to a pack field / pack keyframe pixel measurement / generic
  camera-controller mechanics, all `allowed: True`.
* Clean-room: no .bddl/.xml/.hdf5/init-state file opened (the bddl path was
  never used at all); nothing read under campaigns/c1, c2 or c2fix, no packs/c1_*
  or packs/c2_*, no results/*c1_* or *c2_*, no other abl_c2 cell's pack, results
  or workspace; no LIBERO prior from memory or system context was used — every
  constant is re-derived from this cell's own pack.
* Cluster writes: only `packs/ablC_goal_put_wine_on_rack/program.py` and
  `program_v1.py`.

STOP.
