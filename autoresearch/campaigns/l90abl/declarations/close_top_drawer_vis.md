# l90abl / close_top_drawer_vis — "close the top drawer of the cabinet"

Vision-only pack (`fair-pack-v1-vision`, K=3, two 128x128 keyframes per demo:
t0 and tN, no EEF / gripper / action channels). Everything below is measured
from those images plus debug seeds 51-65.

## Scene, as measured (debug seed 51, `cam_high` deprojected point cloud)

Base frame from `t_base_cam`: camera at (0.659, 0, 1.610), so image-right =
base **+y**, image-down = base **+x**. Table top z = 0.901.

| structure | y | z | x |
|---|---|---|---|
| drawer handle | 0.03 – 0.06 | 1.082 – 1.098 | −0.051 … 0.038 |
| open drawer box (front panel face → interior) | 0.06 – 0.21 | 1.054 – 1.124 | −0.112 … 0.110 |
| cabinet body / top slab | 0.21 – 0.39 | 1.001 – 1.127 | −0.129 … 0.127 |

So the top drawer is open by ~0.145 m and **closes along +y**. The three pack
keyframes agree: the protruding slab is gone at tN and the arm ends over the
cabinet.

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | perception only — recover the base frame and a top-down height map | ep51: got the table plane (0.901), the drawer/cabinet split above, and the +y close axis | scene understood |
| v2 | push the panel at the cloud's "nearest tall thing" | `xc = −0.157`: the **robot arm** (z 1.28–1.37) leaked into the band and became the push target; the gripper swept empty air at x=−0.16 | bug — z band must exclude the arm |
| v3 | fix the band, calibrate the fingertip by pressing on the table first | `sim_steps = 1000` exactly, eef frozen from the first calibration press onward | **horizon is the scarce resource**: `move_cartesian` burns `120 × seconds` steps when blocked; the calibration ate the whole episode |
| v4 | lean run; read the tip offset from a free capture at the staging pose | `tip_off = 0.106` → pushed at z=1.192 and sailed to y=0.32 unobstructed; drawer untouched. The 0.09 m cloud window straddled the **handle** (z 1.082) | tip offset contaminated |
| v5 | calibration probe: press a closed gripper onto bare table, twice, at two spots | seeds 51/53, both spots: eef rests at table+0.008…0.015 | **the eef frame is the fingertip** (offset ≈ 0.012 m), not 0.10 m above it |
| v6 | v4's push at the corrected height (tip at table+0.19) | contact on all 8 probe seeds; eef advanced −0.033 → **0.120** and then froze with residual identical to 4 decimals across every seed and every step | real contact, hard geometric stop, 0/8 |
| v7 | diagnose the stop: y/z corridor profile at the stall + retract before the post survey | drawer front moved 0.067 → **0.165** (needs ~0.21); at the stall, gripper points sit at y 0.18–0.22 / z **1.125–1.216** against the cabinet top slab at z=**1.127** | **the hand body, not the fingertip, leads with a straight-down wrist**, and it grounds out on the cabinet top ~0.045 m short |
| **v8** | turn the tool to point along +y so the fingertips lead and the hand trails behind them | 4/4 probe, **15/15 selection**; eef reaches y=0.211 (the closed-panel position) in a single push, ~210 sim steps | **FROZEN** |

## Candidate law (offered to LAWS.md)

*A downward wrist cannot finish a horizontal push into a recessed opening.* The
eef frame sits at the fingertips, so with the tool pointing down the **hand
body is the leading surface in the push direction** and it collides with
whatever overhangs the opening (here a cabinet top slab 3 mm above the drawer
rim) while the fingertip still has travel left. The receipt is diagnostic, not
inferential: the stall residual was identical to 4 decimals on 8/8 seeds
(no friction jitter), the drawer stopped 0.045 m short, and the same push with
`rotation` set so the tool approach axis is the push direction went 0 → 15/15.
Corollary: `PUSH_H` and the panel/handle z-split are the only scene constants
that matter; the push itself needs no force tuning.

# DECLARATION

- **Frozen version: v8.** `packs/l90abl_close_top_drawer_vis/program.py`
  md5 `9bf8d572dbafb3dc02d043c2df3bed7e` == `program_v8.py` (verified on the
  cluster and locally).
- **Selection receipt: 15/15** on the full debug band 51–65,
  `results/sel_l90abl_close_top_drawer_vis_v8` (`benchmark_success: true` on
  every one of seeds 51,52,…,65).
- Probe receipts: v6 0/8 (51,53,55,57,59,61,63,65), v8 4/4 (51,53,55,57).
- Archived versions: `program_v1.py` … `program_v8.py` in the pack dir.
- `PROVENANCE` present in `program.py`, covering all eight calibrated
  constants; every source is this pack's keyframes, a debug-seed (51–65)
  measurement, or generic controller mechanics.
- Clean room respected: reads limited to `packs/l90abl_close_top_drawer_vis/`
  (pack.json + keyframes), `results/*l90abl_close_top_drawer_vis*`, and the
  harness runner/client surface. No `.bddl`/`.hdf5`/`init_states`, no other
  campaign's artifacts, no other pack's `program*.py` / `NOTES.md`,
  no `tools/probe_*.py`. `api.done` never read. Eval band (seeds 1–50) never
  touched.
