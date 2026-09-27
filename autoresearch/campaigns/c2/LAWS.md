# Campaign c2 — Law Library (fair protocol; grown from zero)

Promoted laws only: falsifiable, banked >=20-episode receipt, clean per
FAIR_PROTOCOL (PROVENANCE-sourced constants, debug-seed derivation only).

---

## C2-L1 — Demo-anchor identity: when the K demos' closing-keyframe EEF xy agree to ~3 cm, their mean identifies the target — pick the nearest size-gated depth blob, no colour/semantic model needed
*(from `obj_butter_stock`; selection 15/15, banked **50/50** blind eval.)*
Receipt: target blob 0.022-0.026 m from the anchor vs ~0.24 m for the nearest
competitor (10x margin) on every debug seed; banked eval_c2_obj_butter_stock.

## C2-L2 — Group the depth cloud by GROUND FOOTPRINT, not image connectivity, whenever a short target can stand behind a tall one
*(from `obj_chocolate_pudding_stock`; banked **50/50**.)*
A short prop overlapping a taller one in the image is absorbed into the tall
prop's component (measured fused x-extent 0.281 m), so every selection rule
mis-targets. Footprint clustering on a 1.5 cm XY grid separates them (per-prop
extents <=0.081 m; target lands 0.023 m from the demo anchor).
Receipt: v1 image-space components 0/8 -> v2 footprint clustering (only
change) 4/4 + 15/15 selection, banked 50/50.

## C2-L3 — A knob-driven "turn on" predicate fires between 30° and 60° of world-+z wrist rotation with the lever held; cap the twist near 90°, never at 45°
*(from `goal_turn_on_stove_stock`; banked **50/50**.)*
Receipt: per-increment api.done across 15 selection episodes — False at 30° on
15/15, True at 45° on 14/15, one seed required 60°. Fixture jitter in this
scene family is ~2 cm/seed: a fixed demo-median pose works but with little
margin; perception clamped to within 6 cm of the demo prior buys margin with
no downside (measured detector hit 15/15).

## C2-L4 — Wide-vessel rim pinch: near-zero final demo gripper width on an object wider than the jaws means the demo pinches the RIM — aim at centre ± rim radius along the jaw closing axis, for grasp AND release
*(from `spa_bowl_table_center_stock`, banked **50/50**; independently confirmed
by `spa_bowl_next_to_plate_stock`'s v3 13/15-first-try receipt, eval pending.)*
Receipt: demo width collapses to ~10 mm on a 111 mm bowl; jaw axis from
tool_rotation; pinch at centre + rim_radius along that axis closed with effort
3.0 on 15/15; release hedged at plate centre + half rim offset kept the bowl
within 26 mm of centre. Banked 50/50.

## C2-L5 — Concave objects under one oblique depth camera: use the above-table cluster's BBOX MIDPOINT, not the point centroid (the far inner wall biases the centroid); separate an abutting flat disc by PER-CELL MAX HEIGHT
*(from `spa_bowl_table_center_stock`; banked **50/50**.)*
Receipt: centroid biased 13.5 mm in -x, identical on all 15 seeds; plain
clustering fused plate+bowl on 5/8 seeds — per-cell max-height filtering took
plate grounding 3/8 -> 8/8 (single change).

## C2-L6 — "Place it in the basket" is OBJECT-SPECIFIC: a clean completed placement of the wrong object scores 0
*(from `obj_bbq_sauce_stock`; selection 15/15, eval pending — receipt is the
wrong-object control arm.)*
Receipt: v5 grasped the amber-label bottle (effort 3.0 throughout), released
it centred over the basket -> 0/8; the same program body pointed at the
demo-anchored bottle -> 8/8 + 15/15. Corollary (same cell): a component's
deprojected horizontal SPAN is not a shape statistic (border pixels deproject
to the background plane: 0.1893 m span for a 0.042 m bottle) — use pixel count
and cap-face height instead.

---
## Ops notes (harness, not laws)
- fair_run resolves --program to absolute (client cwd = sandbox). Launch with
  `setsid nohup ... </dev/null &`; kill ONLY exact PIDs — `pkill -f fair_run`
  hits sibling cells (three collisions in wave 1).
- Cluster numpy >= 2.0: `ndarray.ptp` is gone.
- A blocked move_cartesian burns its full 2*60*seconds step cap; horizon is
  1000 sim steps.

## C2-L7 — Fixed-fixture insertion: fit the fixture pose from the K demos' pre-release EEF points; the insertion pose is QUANTISED (k·90°+φ), so park at the fitted radius on the nearest REACHABLE axis instead of trusting per-episode hole perception
*(from `rs_square_insert` (robosuite NutAssemblySquare); selection 15/15, banked **45/50** blind
eval — vs a privileged-era bare-agent baseline of ~11/100 on the same task.)*
Receipt: peg fitted from pack demos at rms 1.7 mm (r=0.0668, φ=-1.1°, k∈{1,2,3} — never the
unreachable +x axis); replacing the noisy per-episode perceived hole offset with the pack-fitted
radius was the single decisive change (7/8 → 8/8 → sel 15/15). Refuted control: treating the
13 mm demo-fit-vs-perceived peg-centre gap as a removable perception bias made things worse
(v4 6/8, v7 5/8). Companion mechanics: overshoot re-command to beat the harness's 12 mm move
tolerance (narrower than the ±6 mm cross-handle grasp clearance); a threaded part must still be
pushed, but a deep push squeezes it out of the fingers.
