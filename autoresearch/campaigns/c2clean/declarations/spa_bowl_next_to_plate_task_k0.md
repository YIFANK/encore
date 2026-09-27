# c2clean spa_bowl_next_to_plate_task_k0 — NOTES

Intent: "Pick the akita black bowl next to the ramekin and place it on the plate"
No demo pack (k0). Runner: tools/fair_run.py only.

## v1 — perception probe (no motion)
Hypothesis: need camera conventions + scene layout from scratch.
Method: dump cam_high / cam_arm_wrist RGB-D (2x downsampled) via api.log,
compare api.deproject against OpenCV/OpenGL pixel models. Seeds 51,53.
Evidence: deproject lives on FairFrame, not FairApi (v1 crashed); api.log truncates
at 2000 chars. Verdict: superseded by v2.

## v2 / v2b — perception dump, all 15 debug seeds (no motion)
Evidence (offline decode of the zlib+base64 RGB-D in the logs):
- table plane z = 0.900 (modal z of the workspace cloud, all seeds).
- cam_high: K = 618.04 f, 256 centre; t_base_cam optical axis lies in the x-z
  plane, so the base y axis is purely cross-view -> y extents are unbiased.
- scene = cabinet+stove (y<0), ramekin (span 0.086-0.088, top 0.944),
  1 or 2 akita bowls (span 0.110-0.111, top 0.952; seeds 57,62 have one),
  plate (span 0.135-0.137, top 0.920), cookie box (span 0.082, top 0.920).
- target bowl (nearest the ramekin) sits at x~-0.18, y~0.32 on every seed.
Verdict: vessel/flat split at 0.030 above table separates bowls+ramekin from
plate+box on all 15 seeds; smallest-span vessel is always the ramekin.

## v3 — blocked-descent calibration (seeds 51,53)
Evidence: open fingers on the bare table stall the eef at z = 0.9086 (cmd 0.88
and 0.86 both land there) -> tips sit 0.0086 below the eef origin. api.move
lands ~+0.009 high in z and up to 8 mm off in xy, so moves are closed-loop
(re-issue cmd + measured error, 2 iterations, converges to <4 mm).
All four bowl/plate/ramekin waypoints are reachable at z=1.02.
Verdict: TIP_DZ = 0.0086; use closed-loop goto().

## v4 — first full attempt: +y rim pinch, carry, release over the plate
Hypothesis: the bowl (110 mm outer) is wider than the jaws (79 mm max), so
straddle the rim on its +y arc (away from the ramekin) at ymax, tips 18 mm
below the rim top; the bowl centre then trails the eef by the same rim offset,
so release at plate centre + that offset, bowl base 6 mm above the plate top.
Evidence: 4/4 on seeds 51,53,55,57 (results/fs_..._v4). Closed gap 0.0072 with
effort 3.0 held through lift, carry and release.
Verdict: mechanism works. Bug spotted in the logs: the z<1.0 cut sat inside the
segmentation mask, slicing the cabinet into prop-sized blobs (nbowls=4 logged);
the nearest-to-ramekin rule survived it, but it is wrong.

## v5 — v4 with the labelling fix (tall cut applied to component top)
Hypothesis: the labelling fix changes nothing on the score but removes a real
failure mode (a cabinet slice could out-rank the ramekin as smallest vessel).
Evidence: FORMAL SELECTION RUN, all 15 debug seeds ->
  results/sel_c2clean_spa_bowl_next_to_plate_task_k0_v5 = **15/15**.
Verdict: SELECTED.

## Envelope probes (v5e1-e4, seeds 51,55,59,63) — margin, not score
Rationale: 15/15 says nothing about how much aim error the grasp tolerates, and
the eval band is 50 unseen layouts. Perturb the frozen aim and see where it
breaks: e1 = grasp y +8 mm (outward), e2 = y -8 mm (inward),
e3 = grasp depth 10 mm (shallow), e4 = depth 26 mm (deep).
Evidence: e1 4/4, e2 4/4, e3 4/4, e4 4/4
  (results/fs_c2clean_spa_bowl_next_to_plate_task_k0_v5e1..e4).
Verdict: the rim pinch tolerates at least +/-8 mm of lateral aim error and any
grasp depth in 10-26 mm below the rim top. The frozen aim is not sitting on a
cliff, so no hardening (grasp-retry, alternative arc) is warranted; every
perturbation I could afford stayed at ceiling.

---

# DECLARATION

Frozen version: **v5**
  packs/c2clean_spa_bowl_next_to_plate_task_k0/program.py
  md5 4218e53747108edf86209c8f6a5f14a5 == program_v5.py (same md5, verified on
  the cluster).

Selection receipt (formal, full 15 debug seeds 51-65, one run):
  **15/15** — results/sel_c2clean_spa_bowl_next_to_plate_task_k0_v5
  (`grep -c '"benchmark_success": true' .../results.jsonl` = 15).

Per-version receipt chain:
  v1  crashed  — api.deproject does not exist (it is FairFrame.deproject).
  v2  0/4 (no motion) + v2b 0/11 (no motion) — perception dumps, all 15 seeds.
  v3  0/2 (calibration only) — TIP_DZ = 0.0086 from blocked descent.
  v4  4/4 seeds 51,53,55,57 — results/fs_..._v4. First full attempt.
  v5  15/15 seeds 51-65 — results/sel_..._v5. SELECTED.
  v5e1 4/4, v5e2 4/4, v5e3 4/4, v5e4 4/4 — aim-envelope probes on v5.

Mechanism (all of it re-derived on the debug seeds; no pack, no priors):
  1. cam_high RGB-D -> base-frame point cloud; table plane z = 0.900.
  2. Height-gate at 12 mm, 4-connected label, drop components whose TOP exceeds
     1.00 (arm, cabinet, stove). Props left: ramekin, 1-2 akita bowls, plate,
     cookie box.
  3. Split on top height: > 0.030 above table = vessel (bowls 0.052, ramekin
     0.044), <= 0.030 = flat (plate 0.020, box 0.021). Smallest-span vessel is
     the ramekin (0.086-0.088 vs the bowls' 0.110-0.111); largest-span flat is
     the plate (0.135-0.137 vs the box's 0.082). Rank, never threshold.
  4. Target = the bowl nearest the ramekin. The instruction names it, and on
     13/15 seeds there is a second, farther bowl to reject.
  5. The bowl's 110 mm outer diameter exceeds the jaws' 79 mm, so straddle the
     rim on its +y arc (away from the ramekin) at the measured ymax, tips 18 mm
     below the rim top. The camera's optical axis lies in the x-z plane, so the
     y extents are cross-view and unbiased -- that is why the pinch aims along
     y. Closed gap 0.007 at effort 3.0 is the receipt that the rim is held.
  6. Because the eef sits at the rim, the bowl centre trails it by that same
     rim offset: release at plate centre + the offset, bowl base 6 mm above the
     plate top.
  7. Every waypoint is closed-loop (re-issue command + measured eef error, 2
     iterations): api.move lands ~9 mm high in z and up to 8 mm off in xy.

PROVENANCE: present in program.py, 10 entries, every calibrated constant
sourced to a debug-seed measurement or generic controller/camera mechanics.
