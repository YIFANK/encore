# c2clean — goal_put_bowl_on_plate_task_k3

Intent: **"Put the wine bottle on the plate"** (graded by the environment's own
benchmark bit). Runner: `tools/fair_run.py` only. No shared note file.

---

## Evidence read (packs)

Both packs are shot in **this same scene** (cabinet at y≈−0.22, wine bottle,
bowl, plate, stove, blue cube), which is what makes them useful.

| pack | language | what it gives |
|---|---|---|
| `..._k3` | *put the bowl on the plate* | acts on my **target**. Release keyframes at x = 0.0448 / 0.0499 / 0.0500 → the plate is the front disc at x ≈ +0.05. |
| `..._mate` | *put the wine bottle on top of the cabinet* | handles my **object**. Closing keyframes at EEF z = 1.0291 / 0.9917 / 1.0238 with held finger gaps 0.0165 / 0.0246 / 0.0157 m. |

Neither pack is the task: the k3 pack moves the wrong object, the mate pack
moves it to the wrong place. The intent sentence picks one half of each.

**The key cross-check.** My own debug-seed depth profile of the bottle gives a
body diameter of 0.0394 m (z ≤ 0.975), a taper, and a **neck of 0.0134 m**
(z ≥ 1.005), top at 1.0553–1.0586 m. The mate pack's three held gaps land
exactly on that profile — 0.0157/0.0165 on the neck, 0.0246 at z = 0.992 on
the shoulder. Pack and perception agree independently, so the grasp offset
transfers: **close 0.032 m below the measured bottle top.**

## Perception (cam_high RGB-D, arm at home)

- **table** = modal z of the cropped workspace → 0.9010 m on every seed.
- **bottle** = the only dark (L<80) tall (0.97–1.10 m) thing in the mid-table
  crop. The cabinet block is dark and tall too but sits at y < −0.12; the other
  black prop sits at x = −0.389; the arm at home clears 1.14. Axis taken as
  `max_x − neck_radius` (the camera is at x = +0.659, so only the +x half of
  the neck is visible and a plain centroid is biased ~4 mm forward).
- **plate** = the low flat disc (z 0.9035–0.9215, bright) **after cutting out
  the bowl's own rim disc** (found separately in the 0.932–0.958 band, since
  the bowl rim tops out at 0.952 and the plate rim at 0.920). Clustering alone
  fuses bowl and plate on 4 of 5 seeds — the two-band cut separates them on 5/5.
- **release height** = `plate_floor + (grasp_z − table)`. Both terms are
  depth measurements, so a common depth bias cancels. Measured
  plate_floor − table = 0.0070–0.0074 m on 5 seeds.

Per-seed scatter that made perception (not a fixed anchor) necessary: bottle
axis moves over x ∈ [−0.203, −0.187], y ∈ [−0.062, −0.046]; plate centre over
x ∈ [0.040, 0.054], y ∈ [−0.023, −0.007].

## Version log

| ver | hypothesis | evidence | verdict |
|---|---|---|---|
| probe_v1 | dump RGB-D through `api.log` | `api.log` truncates at 2000 chars — 6 chunks of 60 000 arrived as 6×2000 | rejected, re-chunked |
| probe_v2 | same at 1800-char chunks, cam_high only | full 512×512 RGB + depth recovered on seeds 51/53/57/61/65 | all constants below derived from this |
| **v1** | perceive bottle + plate; neck grasp at `top − 0.032`; carry at 1.13; closed-loop descent to `plate_floor + (grasp_z − table)`; release | probe 8/8 (51,53,55,57,59,61,63,65), `results/fs_..._v1` | **selected** |

## Margins (receipts from `results/sel_..._v1/program_ep*.log`)

- Grasp closed on the **first** attempt on every seed; held gap 0.0148 m
  against a 0.0134 m neck and a 0.030 m accept bound. The retry branch never
  fired.
- The seat loop stalls at ~4 mm above the computed release height and reports a
  growing residual — that is the bottle base **in contact with the plate**, so
  the descent self-corrects the small height underestimate rather than dropping
  the bottle. `SEAT stalled — treating as contact` on the logged seeds.
- Lateral: the tool lands ~7 mm +x of the commanded plate centre (tracking
  bias) against a **67 mm** plate radius.

---

# DECLARATION

- **Frozen version:** `program_v1.py` → `packs/c2clean_goal_put_bowl_on_plate_task_k3/program.py`
  (md5 `4893576028d98bfd358c7a7278256cbd`, identical on both files).
- **Selection receipt (full 15 debug seeds):** **15 / 15**
  `results/sel_c2clean_goal_put_bowl_on_plate_task_k3_v1` — seeds 51…65 all
  `"benchmark_success": true`.
- **Receipt chain:** probe_v1 (log-cap discovery, no motion) → probe_v2
  (perception dump, no motion) → v1 probe 8/8
  `results/fs_c2clean_goal_put_bowl_on_plate_task_k3_v1` → v1 selection 15/15.
- **PROVENANCE:** present as a top-level literal dict in `program.py`, covering
  every calibrated constant; sources are the two named packs and debug-seed
  (51–65) RGB-D/gripper measurements only.
- Clean room: no `.bddl`/`.xml`/`.hdf5`/init_states read; no other campaign's
  artifacts; no `program*.py`/`NOTES.md` from any pack directory;
  `fewshot_run.py` never invoked; `api.done` never read.

STOP.
