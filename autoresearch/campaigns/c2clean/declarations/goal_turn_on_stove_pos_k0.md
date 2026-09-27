# c2clean / goal_turn_on_stove_pos_k0

Zero-demo cell. Intent: "turn on the stove". No pack, no shared note file.
Everything below is derived from debug seeds 51-65 via `tools/fair_run.py` only.

---

## v1 — pure perception probe (no motion)

**Hypothesis.** Nothing is known about the scene; the intent names a "stove"
but neither its location nor the mechanism that turns it on. Dump RGB-D from
both cameras through `api.log` (zlib+base64) and do the perception offline, so
the probe costs zero sim steps and zero motion risk.

**Evidence.** `results/fs_c2clean_goal_turn_on_stove_pos_k0_v1` (seeds
51,53,55,57), 0/4 as expected — the program never moves.

Decoded cam_high cloud (base frame):

- Table plane **z = 0.902** (clear patch x[0.00,0.12], y[0.28,0.42] holds no
  points above it on any of the four seeds).
- A dark slab, x[-0.10,0.20] y[-0.09,0.22], top **z ≈ 0.932**, with a concentric
  spiral on it — the stove hob. Flat: nothing to actuate.
- A separate very dark object just behind the slab in x: a **flat circular disc**
  (diameter ≈ 0.095, top z ≈ 0.920) carrying a **raised bar** across its whole
  diameter — bar top **z = 0.961**, length ≈ 0.085 along x, width ≈ 0.024
  along y, standing ≈ 0.041 above the disc. That is the knob, and the bar is a
  handle.
- The arm at home sits at x[-0.27,-0.15]; it does not occlude the knob
  (y ≈ 0.13).
- `_pos` perturbation moves the knob by ~1 cm between seeds, so the knob must be
  perceived per episode, not hard-coded.

**Verdict.** Mechanism hypothesis: the knob is a dial with a bar handle; turning
the stove on means **rotating the bar about the disc's vertical axis**. The bar
is 24 mm wide, well inside the gripper's 78 mm opening, so it is graspable
across its width with the default straight-down wrist (whose tool y-axis is
base −y, i.e. the jaws close across the bar, not along it).

---

## v2 — calibrate, grasp the bar, twist about the disc axis  **[FROZEN]**

**Hypothesis.** Per episode: (a) find the knob as the darkest tall blob in
x[-0.14,0.30], y[-0.12,0.45]; (b) calibrate the fingertip-to-eef offset by
descending a closed gripper onto the clear table patch; (c) close the jaws
across the bar at the disc centre; (d) yaw the wrist about world z in 30° steps.
Verification is by my own sensors only: the closed finger gap must equal the
measured bar width.

**Evidence.**

- Probe `results/fs_c2clean_goal_turn_on_stove_pos_k0_v2` (51,53): **2/2**.
- Probe `results/fs_c2clean_goal_turn_on_stove_pos_k0_v2p`
  (51,53,55,57,59,61,63,65): **8/8**.
- **Selection (formal, all 15 debug seeds):
  `results/sel_c2clean_goal_turn_on_stove_pos_k0_v2` → 15/15.**
  (51,52,53,54,55,56,57,58,59,60,61,62,63,64,65 all
  `"benchmark_success": true`; sim_steps 486-599.)

Receipts from the ep51 log:

    KNOB disc_c=[-0.0413,0.1295] ztop=0.96 bar_len=0.0849 bar_wid=0.0241
    TIP_OFFSET 0.0074   (closed gripper blocked at eef z 0.9094, table 0.902)
    CLOSED  eef=[-0.0354,0.1326,0.9630] gap=0.0244 eff=3.00
    TWIST30 gap=0.0314 eff=3.00
    TWIST60 gap=0.0346 eff=3.00

The closed gap **0.0244 == the perceived bar width 0.0241**: the jaws are on the
bar, not on air and not on the disc. The gap then *grows* through the twist as
the bar rotates inside the jaws — the bar is being turned, which is the
mechanism receipt. The eef freezes after TWIST60 and every later move returns
the identical pose: LIBERO terminates the episode the moment the predicate
fires, so the knob reaches "on" between 30° and 60° of yaw. The commanded 90°
step and the retreat are free no-ops.

**Aim margin.** `program_v2m.py` = v2 with a deliberate **+12 mm** offset on the
across-bar (y) aim. `results/fs_c2clean_goal_turn_on_stove_pos_k0_v2m`
(51,53,55,57): **4/4**, and `CLOSED gap` is still 0.0244-0.0245 on all four — the
close drags the bar to the jaw midline, so the grasp self-centres and the aim
envelope is wider than the 12 mm controller position tolerance. The score is not
sitting on a knife edge.

**Selection margin on perception.** In every logged episode the knob candidate
beats the runner-up by ~8x on the tall-pixel count (730 vs 92 on ep51), so the
dark-blob pick is not a near tie.

**Verdict.** Accept. Frozen.

---

## DECLARATION

- **Frozen version:** v2.
  `program.py` md5 `d17e26e84b8b3a622d2f4edfc53f52cd` == `program_v2.py` md5
  `d17e26e84b8b3a622d2f4edfc53f52cd` (verified locally and on AbakaAI).
- **Selection receipt (full 15 debug seeds):** **15/15** —
  `results/sel_c2clean_goal_turn_on_stove_pos_k0_v2`.
- **Per-version receipt chain:**
  - v1 perception probe — `fs_..._v1`, 0/4 (no motion by design).
  - v2 probe — `fs_..._v2`, 2/2; `fs_..._v2p`, 8/8.
  - v2 selection — `sel_..._v2`, **15/15**.
  - v2m aim-margin diagnostic (+12 mm y) — `fs_..._v2m`, 4/4.
- **PROVENANCE:** present as a top-level literal dict in `program.py`, covering
  TABLE_Z, CLEAR_XY, DARK_LUM, SEARCH_BOX, BAR_BAND, CHUNK. Every constant is
  sourced from a debug-seed measurement or from generic controller/camera
  mechanics. No pack existed for this cell; nothing was read from any other
  cell, campaign, or benchmark asset.
- **Splits:** only seeds 51-65 were ever run, always with `--split debug`.
  `tools/fewshot_run.py` was never invoked. `api.done` is never read.
