# c2k1clean / obj_cream_cheese_task_k1

Intent: **"Pick the alphabet soup and place it in the basket"**
Runner: `tools/fair_run.py` only. Splits sealed (debug 51-65; eval blind).

## Evidence read (packs only)

| pack | language | what it shows |
|---|---|---|
| `..._task_k1` | "pick up the cream cheese and place it in the basket" | TARGET half: a low prop (grasp at ee z=0.009) carried to the basket, released at ee z≈0.179 over a rim |
| `..._task_mate` | "pick up the alphabet soup and place it in the basket" | OBJECT half: grasp at ee z=0.0447, transport at z 0.25-0.31, release at ee z=0.1525 |

Appearance cue for the named object, from the mate pack's own keyframes: diffing
`demo0_t0000.png` against `demo0_t0128.png` isolates the body that left the table
(128-px bbox x 27-37, y 54-67). Its column reads dark lid → **blue band**
(≈26,40,74) → orange band. The same object is visible inside the basket at t=128
with the same blue (34,50,93) + orange (90,55,5) stack. So: *the named object is
the can whose label band reads blue.*

## Scene, measured on debug seeds (v0/v0b capture dumps, cam_high RGB-D)

- table plane z ≈ 0.0012; K = 618.04 f, 512², T_base_cam fixed.
- basket: rim apex z = 0.1436, rim bbox 0.16 × 0.17 m, centre ≈ (0.005, 0.264)
  (the *cluster median* is biased to the near wall — the rim bbox midpoint is
  the honest centre).
- two can-shaped props, apex z = 0.081, rim bbox 0.065 × 0.070:
  - (−0.119, −0.240): band colour (38,50,64) → **B−R = +26** (blue)
  - (+0.101, −0.201): band colour (37,42,19) → B−R = −13 (green/brown)
- two flat props (apex 0.019/0.020) and two cartons (apex ≈ 0.125) behind the
  parked arm. Carry altitude 0.26 clears all of them.
- Across seeds 51-55 the two cans do not move at all; the basket and the flat
  props jitter by ≤ 1.5 cm. Perception is still done per-episode (nothing is
  hard-coded to a seed).

## Version log

| ver | hypothesis | evidence | verdict |
|---|---|---|---|
| v0 | can the cloud be segmented at all from cam_high? | `fs_..._v0` (51,53,55,57): table z, cluster table; showed the parked arm swallows the two cartons but not the cans | perception viable |
| v0b | dump raw captures for offline work | `fs_..._v0b` (51-55): 5 npz dumps → offline rim-centre and colour-band measurements above | gave every constant in PROVENANCE |
| v1 | target = can-sized cluster with the bluest label band; grasp 0.035 below its apex (mate pack t=42); release 0.015 over the basket rim (mate pack t=128) | `fs_..._v1` (51,53,55,57,59,61,63,65) = **8/8**; ep59 log shows blue=+26.1 chosen over −13.0, grasp width 0.0625 / effort 3.00, release at (−0.008,0.264,0.168); `ep59_ok.gif` last frame shows the blue can in the basket | selected |

Grasp is verified with my own sensors only (gripper width + effort 3.0 after the
close *and* after the lift, with a re-perceive/re-aim retry); no runtime success
signal is read, and `api.done` is never touched.

## DECLARATION

- **Frozen version: v1.** `packs/c2k1clean_obj_cream_cheese_task_k1/program.py`
  md5 `49e5d1019bcebb5573b0df23d57bb7eb` == `program_v1.py` (same md5).
- **Selection receipt (full 15 debug seeds 51-65): 15/15**, dir
  `results/sel_c2k1clean_obj_cream_cheese_task_k1_v1` (every episode
  `"benchmark_success": true`).
- Receipt chain:
  - v0 `results/fs_c2k1clean_obj_cream_cheese_task_k1_v0` — perception dump, no motion (0/4, by design).
  - v0b `results/fs_c2k1clean_obj_cream_cheese_task_k1_v0b` — capture dump, no motion (0/5, by design).
  - v1 `results/fs_c2k1clean_obj_cream_cheese_task_k1_v1` — probe 8/8 (51,53,55,57,59,61,63,65).
  - v1 `results/sel_c2k1clean_obj_cream_cheese_task_k1_v1` — formal 15/15.
- `PROVENANCE` present in program.py: 18 entries, every calibrated constant
  sourced to the mate/k1 pack contents or to my own debug-seed captures.
- Clean room: writes confined to `packs/c2k1clean_obj_cream_cheese_task_k1/*`
  and `results/*c2k1clean_obj_cream_cheese_task_k1*`; the v0b `dump_*.npz`
  scratch captures have been deleted from the pack dir. No forbidden file was
  read; no `fewshot_run.py`; no `api.done`; only the brief's FairApi surface
  (capture/eef/gripper/proprio/instruction/move/grip/settle/log) — the
  `ground`/`vqa`/`sam3`/`pick_at` surfaces present in the client were not used.

STOP.
