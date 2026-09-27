# c2clean / goal_put_cream_cheese_in_bowl_task_k0

Intent: **"put the wine bottle in the bowl"** (zero demos; everything below is
derived from debug seeds 51-65 only).

## Version log

### v1 / v1b — perception dump (no motion)
Hypothesis: with no pack, the only way in is to see the scene myself; the log
channel is the only way to get pixels off the box.
Method: zlib+base64 the cam_high / cam_arm_wrist RGB-D through `api.log`, decode
locally. First attempt (4000-char chunks) came back corrupt — `api.log`
truncates a message at ~2000 chars; 1500-char chunks are intact.
Evidence (seeds 51,53,55,57, `fs_..._v1b`):
- `f.t_base_cam` + `f.intrinsics` with the plain pinhole convention
  (x right, y down, z forward) reproduces `f.deproject` to 1e-4 m (checked
  in-episode in v2: `api=[-0.1908,-0.0635,0.9751] ours=[-0.1908,-0.0635,0.9751]`).
- Table plane z = **0.901**. Scene: cabinet+rack (y < -0.12), stove and a flat
  blue box (y > 0.09), a plate (top 0.920), the **bowl** (rim 0.952, rim
  diameter 0.110, inner surface bottoming out at 0.918-0.924) and the **wine
  bottle** (top 1.059, i.e. 0.158 tall; body diameter 0.043 below z 0.96,
  neck diameter 0.015 above z 1.00).
- Layout is nominally fixed but jitters ~1.5 cm per seed, so every number has to
  be perceived per episode.
Verdict: geometry sufficient; the neck is the grasp feature (0.015 into a
0.078 jaw), and grasping the neck leaves the bottle base 0.13 below the eef,
which clears the 0.952 rim.

### v2 / v2b / v2c — in-episode perception + close-height sweep
Hypothesis: the eef→fingertip offset is unknown, so calibrate it by closing the
jaws at descending heights and reading the width the jaws report.
Two crashes first, both instructive: the **robot arm itself** is a component in
the cam_high cloud (top 1.371, 12.8k px) and outranks the bottle unless the
bottle test is bracketed (`0.10 < ztop-table < 0.20`, px < 4000); and
`deproject` lives on the **frame**, not on `api`.
Evidence (`fs_..._v2c`, seeds 51,53,55,57): closes at eef z = 1.180 … 1.060 all
read width 0.0010 / effort 0.05 (air); eef z = 1.046 and 1.026 read width
0.0148-0.0150 / effort 3.00 — exactly the measured neck diameter. Achieved z
runs ~0.006 above the commanded z.
Verdict: fingertips bite the neck from ~0.013 below the bottle top downward;
grasp at `neck_top - 0.030` commanded.

### v3 — grasp neck, carry, stand the bottle in the bowl  **[FROZEN]**
Hypothesis: hold the neck, carry at eef 1.15 (base at ~1.017, above the 0.952
rim), then lower until the base sits on the bowl's inner floor
(`place_z = 0.928 + (eef_z_at_grasp - 0.901)`) and release. The base offset is
read per episode from the eef height at the moment of the close, so no constant
hang length is assumed.
Evidence — probe `fs_..._v3` (51,53,55,57,59,61,63,65): **8/8**. Receipts per
episode: width 0.0149 / effort 3.00 at GRASP and still 0.0149 / 3.00 after the
lift and at PLACE (no slip); the post-release re-perception finds a single
0.158-tall dark component centred on the bowl (seed 51: bottle at x -0.078,
y -0.006, top 1.064 vs bowl centre x -0.082, y -0.005), i.e. the bottle is
standing in the bowl. Episodes end at ~163 sim steps, well inside budget.
Verdict: keep. No further versions needed.

## DECLARATION

- **Frozen version: v3.** `packs/c2clean_goal_put_cream_cheese_in_bowl_task_k0/program.py`
  md5 `02b21c8ef2539e1e550b01ed162cac80` == `program_v3.py` (same md5).
- **Selection receipt: 15/15** on the full debug split (seeds 51-65),
  `results/sel_c2clean_goal_put_cream_cheese_in_bowl_task_k0_v3`
  (`benchmark_success: true` on 51,52,53,54,55,56,57,58,59,60,61,62,63,64,65).
- Receipt chain: v1b perception dump (4 seeds, no motion) → v2c close-height
  calibration (4 seeds, no motion beyond the probe) → v3 probe 8/8
  (`fs_..._v3`) → v3 selection 15/15 (`sel_..._v3`).
- `PROVENANCE` is present in program.py and covers TABLE_Z, GRASP_DEPTH,
  CARRY_Z, BOWL_FLOOR_Z and the segmentation thresholds; every constant traces
  to a debug-seed 51-65 measurement listed above. No pack, no foreign source.
- Runner: `tools/fair_run.py` only; `tools/fewshot_run.py` never invoked;
  `api.done` never read; no benchmark asset file ever opened.
