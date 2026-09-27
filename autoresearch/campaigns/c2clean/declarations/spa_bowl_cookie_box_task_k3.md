# c2clean / spa_bowl_cookie_box_task_k3 — notes

Intent: "Pick the akita black bowl on the stove and place it on the plate".
Runner: tools/fair_run.py only. Packs: `_task_k3` (target-half demos: the bowl
next to the cookie box -> plate) and `_task_mate` (object-half demos: the bowl
on the stove -> plate).

## Pack reading (evidence)

- mate pack language: "pick up the black bowl on the stove and place it on the
  plate". Its three demos close the gripper at EEF
  (-0.275,-0.116,0.948), (-0.275,-0.086,0.942), (-0.275,-0.090,0.959) and
  release over (0.060,0.248,0.931), (0.063,0.229,0.939), (0.034,0.239,0.929).
  Carry apex z 1.08-1.14.
- k3 pack demos close at (0.146,-0.034,0.919), (0.126,-0.056,0.924),
  (0.141,-0.029,0.919) — a bowl standing on the table, ~27 mm lower than the
  stove bowl. Same release region over the plate.
- So: object = the bowl standing on the stove slab; target = the plate; the
  stove raises the grasp by ~0.027 m.

## Version log

### v1 — blind replay of the mate pack's mean waypoints (probe, 4 seeds)
Hypothesis: the mate pack's mean grasp/release points are close enough to work
without perception, and the graded predicate accepts the instruction's bowl
(the bddl filename names a different one).
Run: results/fs_c2clean_spa_bowl_cookie_box_task_k3_v1, seeds 51,53,55,57.
Evidence: **3/4** (51,53,57 ok; 55 fail). sim_steps 149-262 of a 500 horizon.
ep55 log: finger gap after the lift 0.0018 m vs 0.0046-0.0047 on the three
successes — the jaws closed on air. Its bowl sits ~4 cm further in -y.
Verdict: the predicate DOES accept the stove bowl (instruction target is the
graded one). Fixed waypoints are not enough; the bowl moves per seed.

### perception probes (no scored behaviour)
- `probe_dump.py` (15 seeds): api.log truncates each message at 2000 chars, so
  RGB-D is chunked at 1900. cam_high height maps give table z = 0.900, plate
  top 0.920 at (0.05,0.25), cookie box top 0.920, table bowls top 0.944-0.952,
  stove-bowl cluster top 0.979-0.980. A first pass gated at z<1.06 split the
  rim ring into two arcs, which looked like gripper occlusion.
- `probe_dump2.py`: parks the arm aside for a clean cam_high, then hovers for
  wrist views. It refuted the occlusion reading: gated at z in [0.955,1.05] the
  home frame and the parked frame give bit-identical ring bboxes on all six
  seeds — the gripper sits above 1.05 and is cut by the gate, and the "two
  arcs" were cells whose max-z the gripper had raised past the old 1.06 cap.
  So v2 needs no parking move; one cam_high at home serves both detections.

### v2 — perception-driven straddle of the bowl wall on the +y arc
Hypothesis: the stove bowl is the only structure above 0.955 m in the stove
neighbourhood, so its ring bbox centre is measurable every seed; the mate
pack's grasps sit a fixed +0.042 m in y from that centre (the wall midline),
and its releases sit +0.040 m in y from the plate's measured centre — the same
offset the rim grasp carries. A gripper-gap check after the lift separates a
held rim (0.0046-0.0048) from air (0.0018-0.0025) and buys one re-perceive.
Offline check on all 15 debug-seed cam_high dumps: bowl found on 15/15
(size 0.09-0.11 m, rim top 0.979-0.980), plate found on 15/15
(size 0.13-0.14 m, centre x 0.045-0.070, y 0.185-0.215).
Probe: results/fs_c2clean_spa_bowl_cookie_box_task_k3_v2, seeds
51,53,55,57,59,61,63,65 -> **8/8**.
Selection (formal, all 15 debug seeds):
results/sel_c2clean_spa_bowl_cookie_box_task_k3_v2 -> **15/15**,
sim_steps 122-282 (ep65 spent 282 because its first grasp came up empty at
gap 0.0025; the re-perceive found the bowl nudged to (-0.310,-0.110) and the
retry held at 0.0044).
Verdict: frozen.

### aim envelope (margin check, not a candidate version)
GRASP_DY re-run at 0.030 and at 0.054 on seeds 51-58:
results/..._dy030 -> 8/8, results/..._dy054 -> 8/8. The grasp tolerates at
least +-12 mm about the chosen 0.042, so the offset is not perched on a cliff.

## DECLARATION

- Frozen version: **v2**. `packs/c2clean_spa_bowl_cookie_box_task_k3/program.py`
  md5 `b58e0cca9ee28c31c6a4b64b9e33cb86` == `program_v2.py` (same md5, verified
  on the cluster).
- Selection receipt: **15/15** on the full 15 debug seeds (51-65),
  `results/sel_c2clean_spa_bowl_cookie_box_task_k3_v2`.
- Receipt chain:
  - v1 (mate-pack waypoint replay, no perception): 3/4 on 51,53,55,57
    (`results/fs_..._v1`).
  - perception probes `probe_dump.py` (15 seeds), `probe_dump2.py` (6 seeds):
    no scored behaviour, used to calibrate the height-map windows.
  - v2 probe: 8/8 on 51,53,55,57,59,61,63,65 (`results/fs_..._v2`).
  - v2 formal: 15/15 on 51-65 (`results/sel_..._v2`).
  - margin: GRASP_DY 0.030 -> 8/8, 0.054 -> 8/8 on 51-58.
- PROVENANCE: present in program.py as a top-level literal dict covering
  BOWL_WIN, BOWL_ZLO/ZHI, GRASP_DY, GRASP_DZ, PLATE_WIN/ZLO/ZHI, PLACE_DY,
  PLACE_DZ, HELD_GAP, FALLBACK_GRASP/FALLBACK_PLATE, LIFT_Z. Every constant is
  sourced to the two named packs or to debug-seed (51-65) measurements.
- Clean room: reads limited to the two named packs' pack.json + keyframes,
  tools/fair_run.py + tools/fair_client.py + heron/robot/libero.py controller
  mechanics, and my own results dirs. No bddl/xml/hdf5/init_states, no other
  campaign's artifacts, no program.py from any pack. api.done never read.
- Seeds 1-50 never touched.
