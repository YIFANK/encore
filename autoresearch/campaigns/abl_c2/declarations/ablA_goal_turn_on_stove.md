# abl_c2 / ablA_goal_turn_on_stove — worker ledger

Variant A ("naive-demos"): no mined pack. Task-specific input was
`packs/ablA_goal_turn_on_stove/raw_demos.json` (K=3 dense proprio + dense raw
actions, schema `raw-demo-dump-v1`) plus `frames/*.png` (128x128 RGB, uniform
stride 10). LAWS.md empty; no law consumed.

All dates UTC, 2026-08-20/21.

---

## Structure re-derivation (the work the pack would otherwise have done)

The dump gives three arrays per demo and nothing else. Everything below was
computed locally from those arrays; no keyframes, no ee_path, no scales were
supplied.

1. **Phase segmentation from the action array.** `actions[:,6]` is the gripper
   channel (-1 open / +1 close): its first sign flip locates the close instant
   at t = 41 / 48 / 40 for demo0/1/2. `|actions[:,5]| > 0.30` isolates a
   26 / 26 / 31-step terminal phase in which `actions[:,5]` is pinned at
   +0.375 while `actions[:,2]` stays negative and `obs.ee_pos` is frozen. So
   the demo is exactly: reach -> close -> sustained twist under downward
   preload. Two keyframes and one rotation, nothing else.

2. **Rotation semantics from `obs.ee_ori`.** `ee_ori` is axis-angle
   (|v| ~ pi throughout, i.e. a tool pointing down with a rotating axis).
   Rebuilding R(t) and reading the world yaw `atan2(R[1,0], R[0,0])` shows a
   monotone POSITIVE rotation about the WORLD z axis, reaching +57.7 deg
   (demo0), +45.3 (demo1), +70.8 (demo2) at the last logged step. Demos are cut
   at success, so the predicate has fired by +45 deg at the latest. Direction
   is unambiguous: all three go the same way.

3. **The grasp pose, as a cross-demo statistic.** median(`obs.ee_pos`) over the
   twist phase = (-0.4334, 0.2141, 0.9290) / (-0.4295, 0.2122, 0.9293) /
   (-0.3946, 0.2107, 0.9297). y and z agree to 2-3 mm; x spreads 39 mm, which
   is itself evidence that the graspable feature is wide in x. Per-axis median
   of the three -> KNOB_XYZ = (-0.4295, 0.2122, 0.9293).

4. **Grip scale.** finger gap `gripper_states[:,0]-[:,1]` = 0.072-0.080 m all
   the way down (open) and settles at 0.027-0.033 m through the twist: the
   gripper is closed on a ~3 cm feature, not on air.

5. **Scene, from the frames.** Upscaled `demo*_t0000.png` show a flat white
   stove plate with a dark burner and, at its back edge, a black conical knob
   the gripper straddles from above. Measured white-plate bbox and dark-knob
   bbox agree to <= 1.5 px across the three demos -> the layout is effectively
   fixed, which licensed an open-loop pose rather than a perception-gated one.

Effort split: ~all of the analysis budget went into (1)-(5); building the
program from the recovered structure was mechanical.

---

## v1 — hypothesis -> evidence -> verdict

**Hypothesis.** The task is a fixed 4-step open-loop sequence: open, drop
straight onto KNOB_XYZ, close, then walk the tool orientation through a ladder
of +yaw waypoints about world z (target = Rz(yaw) @ R_at_grasp) while holding
the grasp point 10 mm low as a downward preload — the exact mechanism the raw
actions describe. Vision is carried as a LOGGED DIAGNOSTIC ONLY (dark-pixel
cloud deprojected from cam_high near the prior); it never moves the arm, so
version 1 tests the pure demo-derived geometry.

**Evidence.**
- probe `results/fs_ablA_goal_turn_on_stove_v1` (seeds 51,53,...,65):
  **8/8** `benchmark_success`.
- selection `results/sel_ablA_goal_turn_on_stove_v1` (seeds 51..65):
  **15/15** `benchmark_success`.
- from `program_ep*.log`: grip closes to width 0.025-0.030 with effort 3.00 on
  every seed (matches the demo's 0.027-0.033 held gap); descent residual
  <= 0.012 m on 7/8 probe seeds (ep63: 0.033 m, off-target grasp, still
  succeeded).
- the yaw ladder SATURATES: achieved yaw is 37-47 deg on the probe seeds and
  24.0 deg on ep63, and waypoints 75/100 deg add nothing (identical achieved
  yaw). The knob/wrist reaches a hard stop well before the ladder ends; success
  fires at or below that stop on every seed.
- the diagnostic vision channel gave a dark-blob median of
  x in [-0.379, -0.373], y in [0.177, 0.186], z = 0.9263 with ztop = 0.9603 on
  all 8 probe seeds — spread <= 6 mm in x, <= 9 mm in y, 0 mm in z. Independent
  confirmation that seeds 51-65 present an essentially fixed layout.

**Verdict.** ACCEPTED and frozen. No v2-v5 were written or run: 15/15 on the
full debug band with a saturating (i.e. margin-carrying) twist leaves nothing
the remaining budget could buy that would not be overfitting to 15 seeds.

---

## Hygiene incident (disclosed, no effect on the result)

Mid-session an external process overwrote my LOCAL scratchpad copy
`.../scratchpad/program_v1.py` with a file belonging to a DIFFERENT abl_c2 cell
(`ablC_goal_open_middle_drawer`), and a harness diff of that overwrite surfaced
in my context. I did not open, request or use that file; it concerns a
different task (drawer opening) and contributed nothing to this cell. It could
not have contaminated the result either way: v1 was authored, uploaded and had
already scored 8/8 before the overwrite, and the cluster copies are provably
unchanged — `packs/ablA_goal_turn_on_stove/program.py`, `program_v1.py`,
`results/fs_..._v1/program_archived.py` and
`results/sel_..._v1/program_archived.py` all carry md5
`b36ac5d46f82a868c98dd564f52e1ed8`. No sibling-cell content was read from the
cluster at any point.

---

## DECLARATION

- **Frozen program:** `packs/ablA_goal_turn_on_stove/program.py`
  md5 `b36ac5d46f82a868c98dd564f52e1ed8`, byte-identical to
  `packs/ablA_goal_turn_on_stove/program_v1.py` (same md5).
- **Selected version:** v1 (of a 5-version budget; versions 2-5 unused).
- **Selection receipt:** **15/15** on the full debug band 51..65,
  `results/sel_ablA_goal_turn_on_stove_v1`
  (`program_archived.py` md5 `b36ac5d46f82a868c98dd564f52e1ed8`).
- **Receipt chain:**
  - v1 probe — `results/fs_ablA_goal_turn_on_stove_v1` — seeds
    51,53,55,57,59,61,63,65 — **8/8**.
  - v1 selection — `results/sel_ablA_goal_turn_on_stove_v1` — seeds 51..65 —
    **15/15**.
- **Debug episodes consumed:** 8 (probe) + 15 (selection) = **23**.
- **PROVENANCE:** present as a top-level literal dict covering all 7 calibrated
  constants (KNOB_XYZ, PRESS_DZ, APPROACH_DZ, GRASP_OPEN_W, GRASP_CLOSE_W,
  YAW_LADDER_DEG, SEARCH_WINDOW), each sourced to a named field/segmentation of
  this cell's own `raw_demos.json`.
- **Runner:** every episode via `tools/fair_run.py ... --split debug` on
  AbakaAI GPU 4. `tools/fewshot_run.py` never invoked. Seeds 1-50 never
  touched; `--split eval` never run. No `.done` read anywhere in the program.

## Candidate law (for the coordinator to bank)

**"A demonstrated in-place wrist twist is a saturating actuator: the achieved
yaw is set by the mechanism's stop, not by the commanded target, so an
orientation ladder that overshoots the demo band costs nothing and buys the
whole tolerance."** Falsifiable receipt: in
`results/fs_ablA_goal_turn_on_stove_v1` the commanded ladder ran to +100 deg
but the achieved yaw stalled at 37-47 deg (24.0 deg on ep63) and was identical
across the 75 and 100 deg waypoints on all 8 seeds — yet every seed succeeded,
including one that reached only half the minimum yaw any demo displayed
(+45.3 deg, demo1). Prediction: clipping the ladder at the demo maximum
(+71 deg) leaves the debug score unchanged; clipping it below ~24 deg breaks it.
