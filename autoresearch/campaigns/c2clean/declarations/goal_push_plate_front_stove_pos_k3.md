# c2clean — goal_push_plate_front_stove_pos_k3

Intent: "push the plate to the front of the stove". K=3 pack, `_pos` perturbation,
debug seeds 51-65 only.

## Pack read (mechanism)

- `gripper_cmd == -1.0` at every keyframe of all three demos, and
  `gripper_state` stays near the open width. **No grasp** — this is a
  press-and-drag.
- `ee_path6` in each demo: home (x≈-0.20, z≈1.17) → descend at the plate to
  z = 0.917-0.920 → translate +y ≈ 0.28 m at constant z → small -x curl → end.
- demo0 eef Δ = (-0.088, +0.284); the plate itself (back-projected from the
  keyframe images) moves Δ = (-0.10, +0.22). So ≈ 0.06 m of the eef's travel is
  free play before the fingers reach the dish wall.

## Perception (v0 / v0b probes)

`api.log` truncates a message at ~2000 chars — v0's 3000-char base64 chunks were
silently clipped and undecodable. v0b re-dumped RGB-D at 2× downsample in
1700-char chunks; all 15 debug seeds decoded.

Measured on all 15 debug seeds (cam_high, back-projected point cloud):

| thing | signature | value |
|---|---|---|
| table | modal z of workspace cloud | 0.9010 on 15/15 |
| plate | disc, ztop = table+0.018, extent 0.131-0.136 × 0.134-0.136 (aspect ≈1) | centre x ∈ [-0.067,-0.042], y ∈ [0.122,0.151] |
| stove | slab, ztop = table+0.038..0.044, extent 0.244-0.262 × 0.189-0.190 | centre ≈ (-0.260, 0.206) |
| bowl | ztop = table+0.050, extent 0.108×0.108 | — |
| blue box | ztop = table+0.019, extent 0.081×0.041 | — |

Plate detector = 15/15. Stove detector initially missed 4 seeds (ztop 0.038 <
0.040 threshold); band widened to (0.030, 0.115) → 15/15.

**`_pos` swapped the plate and the blue box.** In the demo keyframes the plate
sits at base ≈ (0.033,-0.016) and the blue box at (-0.064,0.133); on the debug
seeds the plate is at ≈(-0.056,0.140) and the box at ≈(0.06,-0.02). So the demo
push (+0.22 m of y) is NOT the push these seeds need — they need ≈ +0.07 m.

## Goal

Demo final plate position, from the bright plate interior in
demo0_t0154 / demo1_t0127 / demo2_t0124 back-projected onto the plate-top plane:
(-0.048,0.190), (-0.036,0.209), (-0.053,0.210), mean (-0.046,0.203) — biased a
little -y because the gripper occludes the +y side. Against the measured stove
centre (-0.260,0.206) that is **goal = stove + (0.210, 0.008)**.

The ray-cast was validated against seed 51: the plate pixel centroid
back-projects to (-0.048,0.144) vs (-0.053,0.151) from the depth cloud (7 mm).

## Versions

- **v0 / v0b** — perception dumps, no motion. 0/8 (v0) confirms the start state
  is not already a success, i.e. the ≈0.07 m of y really is the gap.
- **v1** — perceive → press at the plate centre (z = table+0.016, gripper left
  open) → drag by err + 0.05 m of slack → lift → retreat → re-perceive →
  one correction pass. Receipt below.

## Version receipt chain

| ver | change | run | receipt |
|---|---|---|---|
| v0 | perception dump, no motion | `fs_..._v0`, seeds 51,53,…,65 | 0/8 — start state is not already a success; also exposed the ~2000-char `api.log` truncation |
| v0b | same, 1700-char chunks, 2× downsample | `fs_..._v0b`, all 15 | 0/15 (no motion); RGB-D decoded for all 15 seeds → detector built and validated offline: plate 15/15, stove 15/15 after widening the ztop band to (0.030,0.115) |
| v1 | perceive → press at plate centre (z = table+0.016, gripper left open, as the demos do) → drag by err + 0.05 m slack → lift → retreat → re-perceive → 1 correction pass | `fs_..._v1`, seeds 51,53,…,65 | **8/8**, 46-49 sim steps (LIBERO terminates the episode as the predicate fires, mid-drag) |
| v1 | — | `sel_..._v1`, all 15 | **15/15** |
| v2 | drag budget 1.0 s → 2.0 s, correction passes 2 → 3 (headroom for an eval seed whose plate starts further from the goal, e.g. the demos' own 0.22 m push) | `sel_..._v2`, all 15 | **15/15**, identical per-seed step counts to v1 (the extra budget is never consumed on the debug band) |

Hypothesis → evidence → verdict, in short:

- *H1: the task is a grasp-and-place.* Refuted by the pack — `gripper_cmd` is
  -1.0 at every keyframe of all three demos. It is a press-and-drag.
- *H2: replay the demo push (+0.22 m in y).* Refuted — `_pos` swapped the plate
  with the blue box, so the plate already starts ≈0.07 m from the goal on the
  debug seeds; replaying the demo displacement would overshoot by 3×.
- *H3: the goal is the demo's own final plate pose, taken relative to the
  perceived stove.* Confirmed: 15/15 on the full debug split, against 0/8 for
  the no-motion control.

## DECLARATION

- **Frozen version: v2.** `program.py` md5 `b05add9592e2c3305b469a82863c6795`
  == `program_v2.py` md5 `b05add9592e2c3305b469a82863c6795`.
- **Selection receipt: 15/15** on the full 15 debug seeds (51-65), directory
  `results/sel_c2clean_goal_push_plate_front_stove_pos_k3_v2`.
- Per-version receipt chain: table above (archived as `program_v0.py`,
  `program_v0b.py`, `program_v1.py`, `program_v2.py` in the pack dir).
- `PROVENANCE` present in `program.py`: 10 entries (TABLE_BAND, PLATE_ZTOP,
  PLATE_EXTENT, STOVE_ZTOP, STOVE_EXTENT, GOAL_OFFSET, PRESS_Z, DRAG_SLACK,
  HOVER_Z, RETREAT_XYZ), each sourced to pack fields or debug-seed (51-65)
  measurements only.
- No `api.done` read; no `fewshot_run.py`; no eval-split or benchmark-asset
  access.
