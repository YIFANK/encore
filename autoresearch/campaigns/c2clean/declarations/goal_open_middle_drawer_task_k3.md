# c2clean — goal_open_middle_drawer_task_k3

Intent served to the program: **"open the bottom drawer of the cabinet"**.
Pack language: "open the middle drawer of the cabinet" (K=3 demos).
Runner: `tools/fair_run.py` only. Debug band 51-65; probes on 51,53,55,57(,59-65).

## Scene, re-derived from the pack + debug seeds (no foreign constants)

Pack (`pack.json`):
- Every keyframe of all three demos has `gripper_cmd = -1`: the demos open the
  drawer with the gripper OPEN the whole episode.
- `ee_path6` rotation triples are **world-frame rotation vectors (axis-angle)**,
  not euler. Integrating the raw actions' `a[3:6]*action_scale` as world-frame
  axis-angle deltas tracks the rvec reading (mean 23-51 deg drift over a whole
  demo) far better than the euler reading (56-103 deg), and only the rvec
  reading puts the tool approach axis anti-parallel to the pull.
- Pull-phase tool frame: approach axis ≈ (0, -cos t, -sin t) with t ≈ 15/27/36
  deg for demo0/1/2; finger-opening axis ≈ world +x.
- Pre-pull eef: (-0.004,-0.145,1.032), (0.041,-0.143,1.045), (0.015,-0.139,1.030);
  terminal eef y = +0.025/+0.030/+0.016 at z ≈ 1.037.
- Through the whole pull the raw action is saturated **down** (dz ≈ -0.94) and
  partly -x while +y grows, yet the measured height never changes: the fingers
  are loaded on the handle and the drawer is dragged out under that load.

Debug-seed perception (cam_high RGB-D shipped out through `api.log`, 0 sim steps):
- Cabinet: x ∈ [-0.10, 0.15], y ∈ [-0.33, -0.16], top z = 1.13. The face that
  opens is the **+y face at y = -0.157**.
- Three handle bars stand 0.027 m off that face, each spanning x ∈ [-0.03, 0.09],
  tops at **z = 0.956 / 1.025 / 1.098**.
- Feel scan (v2): pressing the flat face stalls the eef at y = -0.1487, pressing
  a bar stalls it at -0.1205 → the fingertip leads the reported eef by ~9 mm.
- A metal bowl (top z ≈ 0.95, x -0.14..-0.04) and a plate (z ≈ 0.92, x -0.02..0.10)
  stand on the table in front of the cabinet, both starting at y ≥ -0.07.
- Geometry repeats across seeds within ~13 mm, but the program perceives the face
  plane and the bands per episode anyway.

## Version chain

| ver | hypothesis | receipt | verdict |
|-----|-----------|---------|---------|
| v1 | replay the demo poses; pure +y pull at constant z | 0/4 `fs_..._v1` | refuted: an unloaded pull slips off the bar |
| v2 | feel scan: where does a press stall vs height? | diagnostic `fs_..._v2`: face -0.1487, bar -0.1205 at z=1.03 | gave the fingertip offset and the bar heights |
| v3 | demo load: pull target 9 cm low / 25 cm out so -z and +y stay saturated | 0/4 `fs_..._v3`, middle drawer out ~0.067 m | mechanism moves a drawer |
| v4 | v3 + the demos' -x load | 0/4 `fs_..._v4`, same | -x is not the missing piece |
| v5 | press to the face first, then v3's drag | 0/4 `fs_..._v5`, same | — |
| v6 | pinch the MIDDLE bar with the wrist turned 90° about its approach axis | 0/4 `fs_..._v6`; film shows the middle drawer **fully open** | the middle drawer is NOT graded |
| v7 | same pinch on the BOTTOM bar | 0/4 `fs_..._v7` | later shown to have shoved the bowl, not the drawer |
| v8 | same pinch on the TOP bar (control) | 0/4 `fs_..._v8`; film shows the top drawer **fully open** | the top drawer is NOT graded |
| v9 | iterated re-grip with perceived bar tracking | 0/2 `fs_..._v9` | the "bar" it re-perceived was the bowl |
| v10 | depth movie of one pull | diagnostic `fs_..._v10b` | film: cabinet shut, gripper off shoving the bowl |
| v11 | settle before reading the gap (`api.grip` returns mid-close, so 0.017 was a still-closing gripper, not a grip) | 0/2 `fs_..._v11`: settled 0.0214 then decays to 0.0012 under load | the side-on pinch squirts off the bar |
| v12 | shove, then grip the exposed rim from above | 0/2 `fs_..._v12` | rim perception still picking up the bowl |
| v13/v14 | pick an obstacle-free x on the bottom bar, stand off inside the empty y-slot | 0/2 each | **the bottom bar cannot be pinched at all**: the fingers-vertical wrist stalls at z ≈ 1.00 against the face at every x |
| v15 | give up the pinch on the bottom bar: run the DEMOS' loaded drag there instead (fingertips seated on the bar top at z = bar_top+0.008, press in, then a saturated down+out target) | **2/2** `fs_..._v15`, travel 0.188 m, 182 sim steps | mechanism found |
| **v16** | v15 cleaned up: per-episode perception, plus a re-perceive and a second seated drag if travel < 0.12 m | **8/8** `fs_..._v16`, **15/15** `sel_..._v16` | **frozen** |

Two things made this cell hard, and both are receipts rather than guesses:
1. **The pack is a decoy.** Its demos work the middle bar (pull height 1.032 vs
   middle bar top 1.025); opening that drawer fully scores false, as does the top
   one. Only the drawer the intent sentence names is graded.
2. **`api.grip` reads the gap mid-close.** A close onto air reports ~0.017-0.030 m
   with effort 3.0 if you read it immediately, which is indistinguishable from a
   catch; only after `api.settle` does the number mean anything. Four versions of
   "the bar is between the fingers" were that artefact.

## DECLARATION

- **Frozen version: v16.** `packs/c2clean_goal_open_middle_drawer_task_k3/program.py`
  md5 `c2fcb7dec3216d4aed7636236c881ac6` == `program_v16.py` (same md5, verified on
  the cluster and locally).
- **Selection receipt: 15/15** on the full debug band 51-65 —
  `results/sel_c2clean_goal_open_middle_drawer_task_k3_v16` (every episode
  `"benchmark_success": true`).
- Probe receipt: 8/8 on 51,53,55,57,59,61,63,65 —
  `results/fs_c2clean_goal_open_middle_drawer_task_k3_v16`.
- Per-version receipt chain: table above; every formally probed version archived
  as `program_vN.py` in the pack dir (v1-v16).
- **PROVENANCE present** in program.py: R_TILT_DEG, R_DOWN, BAR_X, TARGET_BAND,
  SEAT_ABOVE_BAR, FACE_MARGIN, PRESS_INTO, DRAG_OUT/DRAG_DOWN, CRACK_MIN,
  STANDOFF — every constant sourced to a pack field or a debug-seed measurement.
- Clean room respected: writes only under the pack dir and `results/*c2clean_*`;
  no benchmark asset, no other campaign's artifacts, no `api.done` read
  (program.py contains no `.done` attribute access).
