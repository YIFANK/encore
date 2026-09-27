# abl_c2 / ablA_goal_bowl_on_plate — worker ledger (variant A, naive raw demos)

Intent: "put the bowl on the plate". Runner: tools/fair_run.py ONLY, GPU 5,
split debug (seeds 51-65). Task-specific input: packs/ablA_goal_bowl_on_plate/
raw_demos.json (K=3, dense proprio + dense actions, no segmentation, no
keyframes, no summary) + frames/*.png at a uniform stride. LAWS.md empty.

## Structure I had to re-derive from the raw dump (there was no pack)
1. **Phase boundaries.** The dump has no keyframes. `actions[:,6]` turned out to
   be the gripper COMMAND channel (-1 open / +1 close) with exactly TWO
   transitions per demo: close at t=32/36/36, open at t=76/81/87. Those two
   events are the whole task grammar (approach / pinch / carry / lower /
   release) and every constant below is cut from them.
2. **Heights.** z_grasp = min ee_pos[:,2] in the closing window =
   .9182/.9174/.9188; z_place = the ee_pos z plateau just before the open
   command = .9332/.9262/.9274; z_carry = max ee_pos z between the events =
   1.0388/1.0453/1.0484. Spread across demos < 1.5 mm -> these are scene
   constants, not demo noise.
3. **Grasp type.** gripper_states[:,0]-[:,1] collapses 0.079 -> ~0.005 m at the
   close and stays there through the carry: a thin wall between the fingers, so
   the demos pinch the bowl RIM, not the bowl body.
4. **Grasp axis.** api.tool_rotation() at reset is
   [[.998,0,-.057],[0,-1,0],[-.057,0,-.998]] -> the tool y axis (finger closing
   axis) is world -y, so the rim pinch has to sit at a +/-y extreme of the rim
   ring.
5. **XY.** The demo grasp/place xy are only priors (spread 1.6 / 2.8 cm); debug
   seeds put the plate up to 5.9 cm away from the demo prior, so both objects
   must be perceived per episode. Perception is cam_high RGB-D: neutral-colour
   rule (table pixels are warm, R-B ~ +33; plate and bowl are the neutral
   blobs), plate = compact neutral bright blob (mean 166, std 5) inside a 0.13 m
   gate on the demo place prior, bowl = neutral raised blob (table+0.010..0.055)
   inside a 0.13 m gate on the demo grasp prior with the plate neighbourhood
   excluded; rim = top 8 mm of the bowl blob; grasp = rim centre + (0, +r);
   place eef = plate centre + the SAME offset so the bowl lands centred.

## Version ledger (hypothesis -> evidence -> verdict)
- **v1** (2026-08-20). H: absolute demo z's transfer; demo xy priors + a colour
  segmentation locate the objects; a rim pinch on the -x side.
  E: results/fs_ablA_goal_bowl_on_plate_v1 = **0/4** (51,53,55,57). Log gave the
  real measurements: table top deprojects to z=0.9009 (4/4), plate top
  table+0.016, plate detection clean; BUT the bowl detector locked onto the wine
  bottle (ztop 0.993, r 0.010) in 4/4, so no bowl centre and no offset
  compensation -> the bowl was released ~r off the plate centre.
  V: REFUTED on bowl detection and on the -x rim direction; the z constants and
  the plate detector are CONFIRMED.
- **v2** (2026-08-20). H: gate the bowl mask by height band + plate exclusion;
  grasp at rim centre + (0,+r) (finger axis is world y); place eef = plate
  centre + the grasp offset.
  E: fs_..._v2 = **8/8** (51..65 odd); formal sel_ablA_goal_bowl_on_plate_v2 =
  **14/15**, 143-149 sim steps on wins (predicate fires at the release), single
  loss ep62 (rim lost on the lift, then two more attempts failed).
  V: CONFIRMED. This is the working mechanism.
- **v3** (2026-08-20). H: the demos' literal grasp height 0.9181 is better than
  the 0.928 we actually achieve (api.move leaves a systematic +0.010 m z lag);
  close the loop on measured eef z, and fit a circle to the rim ring.
  E: fs_..._v3 = **6/8** (converted ep62, broke ep51 and ep59; ep65 hit the
  1000-step horizon). Logs: at the deeper height the close still reads effort
  3.0 but the rim slips on the lift, and the re-perceived bowl centre walks
  +0.016 m per attempt -> descending to 0.918 SHOVES the bowl instead of
  straddling its rim.
  V: REFUTED. The 0.928 OSC steady state is the correct rim-straddle height;
  the demos' literal fingertip z is NOT reproducible with this controller.
- **v4** (2026-08-20). H: keep v2 mechanics exactly, add a strictly additive
  retry ladder (+y, +y' on the re-perceived rim, -y, +x) and a closed-on-air
  early-out, so no attempt-0 success can change.
  E: formal sel_ablA_goal_bowl_on_plate_v4 = **14/15**, same single loss (ep62).
  V: TIE with v2 on score, strict superset in failure coverage -> declared.

## DECLARATION
- Frozen program: `packs/ablA_goal_bowl_on_plate/program.py`
  md5 `06381a9ee12ba1d91e955e25257f3c4f` == `program_v4.py` (verified on cluster).
- Selection receipt: **14/15** on the full debug split 51..65,
  dir `results/sel_ablA_goal_bowl_on_plate_v4` (only ep62 false).
- Receipt chain: v1 0/4 (fs_..._v1) -> v2 8/8 probe (fs_..._v2), 14/15 formal
  (sel_..._v2) -> v3 6/8 probe (fs_..._v3) -> v4 14/15 formal (sel_..._v4).
- Versions used: 4 of 5. Debug episodes consumed: **50** (4 + 8 + 15 + 8 + 15).
- PROVENANCE: present in program.py as a top-level literal dict, 13 entries, all
  sourced to raw_demos.json / own dump frames / own debug-seed measurements /
  generic camera+controller mechanics.
- Clean room: no .bddl/.xml/.hdf5/init-state file opened; nothing under
  campaigns/c1, c2 or c2fix, no packs/c1_*, packs/c2_*, results/*c1_*,
  results/*c2_*, no other abl_c2 cell touched; only fair_run.py used; seeds 1-50
  never run; api.done never read.

## Candidate law (for the coordinator to bank)
**"Replay the demo's contact POSE, not its contact HEIGHT."** When a demo's
end-effector height is replayed as a position command through an OSC controller
with a steady-state lag, the achieved height (command + lag) can be the
functional pose and the lag-corrected height a failure. Receipt: commanding
Z_GRASP=0.9181 lands the tool at 0.928 and yields 14/15 (sel_..._v2 / _v4);
closing the loop so the achieved height equals the demo's 0.9181 yields 6/8
(fs_..._v3) with the re-perceived object centre walking +0.016 m per attempt.
Falsifiable: on any other cell, lag-correcting a replayed contact height should
not improve success and may degrade it.
