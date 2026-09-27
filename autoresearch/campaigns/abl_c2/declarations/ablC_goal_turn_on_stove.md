# abl_c2 / ablC_goal_turn_on_stove — worker ledger (variant C, "no-verify")

Intent: **turn on the stove**. Variant C contract: exactly ONE program version,
ZERO episodes, `tools/fair_run.py` never invoked on any split or seed.

## 2026-08-20 (UTC) — session

### Inputs consulted
- `packs/ablC_goal_turn_on_stove/pack.json` (K=3 demos: keyframes with EEF 6-DoF +
  gripper_cmd + gripper_state, `ee_path`, `ee_path6`, per-step `actions`,
  `action_scale`, language).
- `packs/ablC_goal_turn_on_stove/keyframes/*.png` (9 agentview frames, 128×128),
  inspected visually and numerically (pixel-value maps of the stove region).
- Nothing else. No .bddl/.xml/.hdf5/init-state file, no c1/c2/c2fix material, no
  other abl_c2 cell's pack/results/workspace, no `tools/probe_*`.

### Reading of the pack (the evidence the single version is built on)
1. All three demos are the same two-phase motion: translate from home
   (eef ≈ (−0.208, 0.00, 1.16)) to a single point, close the gripper there
   (`gripper_cmd` flips −1→+1 at t = 41 / 48 / 40), then rotate in place.
2. Post-grasp eef z is 0.9279–0.9316 in every demo (±2 mm) → **z is a fixed
   scene constant**. Post-grasp eef xy spans x ∈ [−0.4342, −0.3857] (0.048 m) and
   y ∈ [0.2064, 0.2200] (0.013 m) → **xy is the quantity that may vary**.
3. Fitting `R_final = Rz(θ) · R_at_close` (axis-angle → matrix, world frame) gives
   θ = **+53° / +50° / +72°** about world **+z**, residuals 0.25 / 0.13 / 0.08.
   Independently, raw `actions[:,5]` is pinned at **+0.375** (saturated positive
   yaw) throughout the tail of every demo → a counter-clockwise (from above) yaw
   of at least ~50°, visibly resisted by the knob.
4. `actions[:,2]` stays at −0.25…−0.55 for the whole twist while achieved z does
   not move → the demos **press down against a stop** while twisting.
5. Final closed finger spans: 0.0306 / 0.0285 / 0.0346 m → the fingers pinch a
   ~3 cm post, giving a sensor-only grasp self-check (`api.gripper()` width/effort)
   that touches no success signal.
6. Keyframe pixel maps: the knob renders at max(R,G,B) ≤ 29; the burner disk is
   60–129 and the stove plate 140–190 → a darkness threshold of 45 separates the
   knob from both.

### Version 1 — the only version
- **Hypothesis.** Grasping the knob is a top-down pinch on a vertical post whose z
  is a fixed constant and whose xy is the only layout-varying quantity; the
  success bit follows from a ≥ ~50° world-+z yaw of that post while pressed down.
  Therefore: take z, the yaw axis/magnitude, the press, and the fallback xy from
  the pack; recover the xy per episode from a top-down wrist-camera look, gated
  and clamped so that any perception failure degrades to the pack nominal rather
  than to a wild move.
- **Mechanism.** open gripper → move to (NOM_XY, z = 1.075) with the wrist straight
  down → up to two `cam_arm_wrist` looks: inside the central 56 % of the frame,
  take the nearest dark (max RGB < 45) surface slab (5th-percentile depth +
  0.035 m), robust-median its pixels, `deproject` the centroid, gate it (≥ 8
  pixels, 0.02–0.40 m below the eef, within 0.10 m of NOM_XY in xy), clamp the
  result to ±0.06 m of NOM_XY; for a straight-down tool this deprojected point is
  the knob axis and hence exactly where the eef must sit → descend to z = 1.005
  then z = 0.930 and close → self-check the finger span (hold iff effort ≥ 2.5 or
  0.012 ≤ width ≤ 0.055); on "empty", retry at the two demo-extreme x offsets
  (∓0.024 m), and if all three fail, return to the primary estimate, close, and
  twist anyway → twist: `Rz(θ)·R_now` for θ = 25°, 50°, 75°, 95°, 110°, 110°, 110°
  at a target z of 0.920 (1 cm of press, mirroring the demos' saturated −z
  command), re-asserting the close mid-ramp.
- **Evidence.** None from execution — this cell forbids it. The only evidence is
  the pack analysis above plus the permitted static checks:
  - `python -m py_compile program.py` → OK.
  - Forbidden-token scan over the full source (all 18 listed tokens) → 0 hits.
  - AST scan for any `.done` attribute read → 0.
  - Top-level literal `PROVENANCE` dict → present, 19 entries, every entry has
    `source` and `allowed: True`; every module-level calibrated constant
    (NOM_XY, Z_GRASP, Z_PRESS, Z_PRE, Z_LOOK, TWIST_DEG, GRIP_OPEN, GRIP_CLOSE,
    HOLD_W_MIN, HOLD_W_MAX, HOLD_EFFORT, RETRY_OFFSETS, CLAMP_XY, DARK_MAX,
    NEAR_BAND, BORDER_FRAC, GATE_DZ, GATE_XY, R_DOWN) is covered, all sourced to a
    pack field or to generic controller/camera mechanics.
  - Local mock-object walk of the control flow (a plain Python stub object; no
    simulator, no environment, no seed, no `fair_run`) exercised both the
    verified-hold and the never-holds branches without error.
- **Verdict.** FROZEN and shipped unverified, by contract. Confidence is
  argued, not measured: the yaw axis/sign/magnitude and the grasp z are strongly
  determined by the pack (3/3 demos agree); the grasp xy is the open risk, since
  the pack cannot tell me whether the 0.048 m cross-demo x spread is stove
  placement variation or grasp slop on a fixed fixture, and I was not permitted to
  find out.

### Open question I could not settle without episodes
The three `t0000` keyframes put the stove within ~1.5 px of the same place while
the post-grasp eef x differs by up to 0.048 m across demos. Either the stove is a
fixed fixture and the demos grasp the knob sloppily (in which case the task has
several cm of xy tolerance and the pack nominal alone would do), or the fixture
moves and perception is mandatory. The program is written to be correct under
either reading, but the reading itself is untested.

## DECLARATION
- Frozen program: **`program.py` == `program_v1.py`**, md5
  **`71368f8d93e2a489e151befc7b96025c`** (identical local and on the cluster at
  `/mnt/data/YifanKang/Heron/packs/ablC_goal_turn_on_stove/`).
- **Episodes run: 0.** `tools/fair_run.py` was never invoked — no probe, no
  selection run, no debug seed, on any split. Seeds 1–50 and 51–65 untouched.
  `tools/fewshot_run.py` never invoked.
- **Program versions written: 1.**
- **PROVENANCE present:** yes — top-level literal dict, 19 entries, all
  `allowed: True`, covering every calibrated constant.
- Cluster writes made: only `packs/ablC_goal_turn_on_stove/program.py` and
  `packs/ablC_goal_turn_on_stove/program_v1.py`.
- Forbidden reads: none.
