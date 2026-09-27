# NOTES — abl_c2 / ablC_goal_bowl_on_plate  (variant C: NO VERIFICATION)

Intent: "put the bowl on the plate".
Worker session: 2026-08-20 (UTC). Coordinator runs the blind sealed eval.

LAWS.md was empty for this cell (by design) and was not written to.

---

## v1 — the only version (hypothesis -> evidence -> verdict)

**Hypothesis.** The task is a rim-pinch pick of a small bowl and a centred
place onto a plate. Three claims, all read off the pack:

1. *The bowl cannot be grasped across its body.* In `pack.json` the gripper
   opens to a 0.072 m gap (`gripper_state` +/-0.0362) and closes on the object
   to a 0.011 m gap (demo0 t=76: [0.0058, -0.0053]; demo1 t=81: [0.0056,
   -0.0078]; demo2 t=87: [0.0041, -0.0067]). An 11 mm bite is a wall, not a
   body. The keyframe silhouette makes the bowl ~17 px across at 128x128,
   i.e. ~0.072 m — too wide for the jaws anyway. So: rim pinch.

2. *The pinch sits ~0.035 m from the bowl centre along base +y.* Bowl
   silhouette centroids in `demo{0,1,2}_t0000.png` are u = 63.0 / 65.0 / 66.0
   while the grasp-keyframe ee y is 0.0335 / 0.0431 / 0.0462. The three ee-y
   values rise monotonically with u, and the robot column (base y = 0) images
   at u ~ 63.5, which is also the principal point of a 128 px image. Removing
   the u-dependence leaves a grasp offset of 0.0335 / 0.0368 / 0.0357 in +y —
   a 3.3 mm spread across three layouts. That is the rim radius, and it is
   spent along +y. Two independent supports for the +y direction: (a) an
   agentview camera in front of the robot has image-right = base +y and
   image-down = base +x (right-handed camera axes), and the gripper body — a
   parallel jaw is widest along its travel axis — images as a bar running
   left-right in demo0_t0032 / demo1_t0036 / demo2_t0045; (b) the offset
   magnitude equals the measured silhouette radius, which only a radial pinch
   would produce.

3. *Heights are layout-invariant.* The grasp ee z is 0.9237 / 0.9202 / 0.9197
   across three different layouts (spread 4 mm) because the bowl always stands
   on the same table. The release ee z exceeds it by 0.0122 / 0.0078 / 0.0112,
   i.e. the plate is a ~10 mm thick flat disc and the bowl is carried at a
   fixed height offset. So z can be a constant and only xy needs perception.

**Program (v1).** One `run(api)`:

- Segment `cam_high` once. The keyframe pixel statistics separate every
  relevant surface without any prior about the benchmark: table (R-B ~ 33,
  G-B ~ 17), plate rim (R-G ~ 35, G-B ~ 6 — unique in the scene, one blob per
  keyframe), plate interior (neutral, value 168, per-pixel std 5), bowl
  (neutral, value ~110, per-pixel std 33-38 — mottled). Candidate blobs are
  8-connected components of the neutral mask (raw and once-eroded, deduped),
  filtered on value mean / value std / aspect, then lifted to the base frame
  with `api.deproject` (the API's own deprojection, so no camera-convention
  guess) and filtered again on metric radius, height above the measured table
  top, workspace box, and the fraction of table-coloured pixels in a
  surrounding annulus (an object standing free on the table, which rejects the
  appliance's burner disc). Bowl centre and radius come from the topmost 12 mm
  band of the blob's points — the rim ring — so the concave interior does not
  bias the centroid.
- The pinch direction is re-derived at runtime: the demo grasp orientation
  (rotation vector 3.0965, -0.0677, -0.0459) maps base +y into the tool frame,
  and `api.tool_rotation()` maps it back out, snapped to the nearest principal
  axis. If the fair wrist's canonical straight-down yaw differs from the demo's
  by 90 deg, the offset follows it instead of pointing the wrong way.
- Pick at `bowl_centre + r*d` (r = measured rim radius, clamped to
  [0.028, 0.045]), z = 0.9212. Close, lift 55 mm, and test the hold with the
  program's own sensors only (`api.gripper()` effort >= 2.5, width <= 0.032,
  sampled twice so one dropped read cannot cost a good grasp). On failure:
  open, back off to the demo start pose, re-perceive, and retry along -d then
  the perpendicular. Budget 3 attempts.
- Carry at z = 1.030 (the demos' carry waypoints: 1.0179 / 1.0322 / 1.0277)
  and place at `plate_centre + r*d` — the *same* offset re-applied — so the
  bowl centre, which hangs at eef - r*d, lands on the plate centre. That is
  better centred than the demos themselves, which released with the bowl up to
  20 mm off the plate centre. Descend to 0.9212 + 0.012, re-check the hold,
  open, retreat.
- Fallbacks if segmentation returns nothing: bowl (-0.102, 0.006), plate
  (0.048, -0.010) — the demo grasp/release ee xy minus the rim offset.
- No runtime success signal is consulted; `api.done` is never touched.

**Evidence available to v1.** Pack only, as the variant requires. The colour /
blob stage was exercised on the three pack keyframes as pack study (no api, no
environment, no policy execution): it returns exactly one plate candidate per
keyframe (the true one, value mean 168, std 5) and puts the true bowl among the
bowl candidates in all three, with the competing appliance blob 30 px wide
versus the bowl's 16-17 px — outside the 0.050 m metric radius gate.

**Verdict.** UNVERIFIED BY CONSTRUCTION. No episode was run, so this program
has no measured success rate at freeze time. Its claims are argued from the
pack; the blind eval is their first and only test.

---

## Candidate law (falsifiable, with a pack receipt)

*A parallel-jaw top-down grasp whose commanded closure gap is far smaller than
the target's silhouette is a rim pinch, and the constant that matters is the
vector from the object centroid to the pinch, not the pinch position. Re-apply
that same vector about the destination and the carried object lands centred.*
Receipt: pack.json gripper_state 0.011 m closure versus a 0.072 m silhouette,
and a grasp offset of 0.0335 / 0.0368 / 0.0357 m that is constant across three
layouts whose grasp positions differ by 14 mm.

---

## DECLARATION

- Frozen program: `packs/ablC_goal_bowl_on_plate/program.py`
  md5 `3af3200b305fb41e98683751101a7e03`, identical to `program_v1.py`
  (same md5, verified on the cluster with md5sum).
- **Episodes run: 0.** `tools/fair_run.py` was never invoked, on any split, on
  any seed. Seeds 1-50 and 51-65 were never touched. No probe, no selection
  run, no debug seed, no gif, no result file.
- **Program versions written: 1.** There is no program_v2.
- PROVENANCE: present as a top-level literal dict, 45 entries, one per
  module-level calibrated constant, every entry `{"source": ..., "allowed":
  True}`. Sources are pack fields / pack-keyframe pixel statistics / the
  documented FairApi surface / generic camera and controller mechanics. No
  constant comes from outside this cell's pack.
- Static self-checks run (the permitted ones only):
  `python -m py_compile program.py` -> OK; forbidden-token scan over the source
  for all 17 gate tokens -> none present; AST scan for any `.done` attribute
  read -> 0; AST check that PROVENANCE is a top-level literal dict and that
  every upper-case module-level constant appears in it -> pass, none missing,
  none malformed.
- Clean room: no .bddl/.xml/.hdf5/init-state file was opened (the bddl path was
  never used at all, since nothing was run); nothing under campaigns/c1, c2 or
  c2fix, no packs/c1_*, packs/c2_*, results/*c1_*, results/*c2_*, no
  tools/probe_*.py, and no other abl_c2 cell's pack, results or workspace was
  read. (The session scratchpad is shared with a sibling abl_c2 cell; its pack
  directory was noticed and deliberately not opened — this cell's pack was
  re-fetched into a private subdirectory instead.) No LIBERO prior from memory
  or system context was used; the bowl geometry, grasp offset, heights and
  colours were all re-derived from this pack.

STOP.
