# abl_c2 / ablC_goal_put_bowl_top_cabinet — worker ledger (variant C, no-verify)

Intent: "put the bowl on top of the cabinet".
Date: 2026-08-21 (UTC). All times absolute.

## Ablation contract as executed
Variant C = NO VERIFICATION. Exactly one program version, written from the pack
alone, sent straight to the coordinator's blind sealed eval. `tools/fair_run.py`
was never invoked, on any split, on any seed. Zero episodes were executed. There
was no probe, no selection run, no iteration, no program_v2.

## Inputs actually read
- `/mnt/data/YifanKang/Heron/packs/ablC_goal_put_bowl_top_cabinet/pack.json`
  (K=3, demo lengths 91/110/99, stride 10, keyframes with ee/ee_path6/gripper,
  raw actions, action_scale, language).
- `packs/ablC_goal_put_bowl_top_cabinet/keyframes/*.png` (12 renders, 128x128).
Nothing else. No .bddl/.xml/.hdf5/init-state file, no c1/c2/c2fix material, no
other abl_c2 cell's pack/results/workspace. LAWS.md is empty for this cell, as
specified.

## v1 — hypothesis -> evidence -> verdict

**Hypothesis (the whole program, stated once because there is only one version).**
The task is a rim-pinch pick of a shallow speckled bowl off the table and a
set-down onto the large dark cabinet top at negative y. Concretely:
1. the bowl is grasped by pinching its *rim wall*, not by clamping its body;
2. the grasp height is a layout-independent absolute z from the pack;
3. the bowl's xy must be perceived, because the pack cannot tell me where it
   will be in an unseen layout;
4. the cabinet is close to static across layouts, so a perceived drop point
   clamped to the demo release prior beats either source alone;
5. release height = grasp height + (perceived cabinet top − perceived table),
   which sets the bowl down instead of dropping it.

**Evidence, all of it from the pack.**
- *Rim pinch.* `gripper_state` sums while the bowl is being carried are 0.0125
  (demo0 t=81), 0.0059 (demo1 t=98), 0.0125 (demo2 t=92), against 0.0787 open.
  A 6–12 mm closure cannot be the width of a bowl body; it is the thickness of a
  single wall between the jaws. So one finger goes inside the rim, one outside.
- *Absolute grasp z.* The ee z at the gripper-close keyframe is 0.9147 / 0.9139
  / 0.9162 — a 2 mm spread across three layouts. The bowl always starts on the
  table, and the table does not move, so this height transfers verbatim; it needs
  no camera calibration. Adopted as `Z_GRASP = 0.9149`.
- *Bowl xy must be perceived.* The three demo grasp xy are (-0.0990, 0.0504),
  (-0.1047, 0.0338), (-0.0848, 0.0179) — a 21 x 32 mm scatter. Their circumcircle
  has radius 18.8 mm, i.e. they are consistent with three different azimuths on
  one bowl rim rather than with a fixed offset, so the pack gives no usable
  point estimate of the bowl centre. Perception it is.
- *Cabinet near-static.* Thresholding the three `demo*_t0000.png` at max-channel
  < 70 puts the cabinet silhouette at u in [0,44], with its top edge at
  v = 46 / 48 / 47 and dark-pixel counts 2320 / 2310 / 2311. Across the three
  demo layouts the cabinet moves about 2 px of 128. Meanwhile the demo *release*
  points span 0.087 m in x and 0.053 m in y on that same top face — that spread
  is demonstrator slop on a big flat surface, and it doubles as a measured
  tolerance: anywhere in a ~9 x 5 cm window on the top works.
- *Release height identity.* `Z_RELEASE_PRIOR − Z_GRASP = 1.1570 − 0.9149 =
  0.2421 m` is how far the demos raised the held bowl. Since the bowl's underside
  sat on the table at grasp time, 0.2421 m is (to within the release clearance)
  the cabinet's rise above the table. That converts an un-calibratable absolute
  into a run-time-measurable one:
  `z_release = Z_GRASP + (z_cabinet_top − z_table) + 0.008`.
- *Carry height.* max z along `ee_path6` while holding is 1.2219 / 1.2638 /
  1.1983; `CARRY_Z = 1.255` sits inside that envelope.
- *Scene colours*, measured off `demo0_t0000.png` patches: bowl grey 117 /
  sat 0.071 / grey-std 35 (speckled), stove burner grey 91 / sat 0.006, plate
  grey 160 / sat 0.070, table grey 172 / sat 0.178, cabinet rgb (33,28,25),
  wine bottle sat 0.483, blue box sat 0.229. Colour alone does not separate the
  bowl from the burner, so the bowl detector leans on depth geometry
  (height above table, rim radius, concavity, and whether the blob's immediate
  surround is the table or something raised) and uses colour only to reject the
  chromatic objects.

**Design choices forced by having no verification.**
- *Camera convention is established, not assumed.* The analytic pinhole cloud is
  scored against a dozen `frame.deproject()` calls and the better of the two
  axis conventions is kept; if neither reproduces the api, the program falls back
  to a coarse cloud built entirely from `deproject`.
- *Jaw-closing axis is not assumed.* The rim offset must be radial for the pinch
  to work, which requires knowing which tool column carries the closing axis.
  Rather than guess a convention I could not test, attempt 1 offsets along the
  tool-Y horizontal and attempt 2 along tool-X, with the two separated by
  `api.gripper()` effort/width only. No success or termination signal is
  consulted; `api.done` is never touched.
- *Every stage degrades to a pack prior.* Perception failure -> grasp at
  `GRASP_PRIOR_XY` with r = 0.040 and release at `PLACE_PRIOR_XY` /
  `Z_RELEASE_PRIOR`. Perceived drop point is clamped to within
  `CAB_TRUST_RADIUS = 0.15 m` of the release prior, so a bad cabinet detection
  cannot throw the bowl off the table.
- *Time budget.* Demos are 91–110 control steps. Moves are sized
  `clip(dist/0.15, 0.7, 2.2)` s with one residual-triggered correction, and the
  second grasp attempt is skipped once 15 s of commanded motion is spent.

**Verdict.** Cannot be stated. That is the point of this cell: with zero
episodes there is no evidence for or against the hypothesis, only its
derivation. The coordinator's blind eval on seeds 1-50 is the first and only
measurement this program will ever receive.

## Candidate law (falsifiable, receipt = this pack)
*A carry-phase gripper width far narrower than the object's body identifies a
rim/wall pinch rather than a body grasp.* Receipt: `gripper_state` sums of
0.0125 / 0.0059 / 0.0125 against an 0.0787 open jaw while a bowl is demonstrably
being transported. Falsifiable: a body grasp of the same bowl would have to
report a carry width equal to its outer diameter.

## Static self-checks run (the only checks this variant permits)
- `python -m py_compile program.py` -> OK.
- forbidden-token scan (gt_pose, gt_trace, gt_joints, _sim, env.sim, sim.data,
  body_xpos, site_xpos, geom_xpos, .bddl, bddl_files, init_states, pruned_init,
  h5py, .hdf5, api.robot, self.robot, check_success, OffScreenRenderEnv) -> NONE.
- AST scan for any `.done` attribute read -> 0 occurrences.
- `PROVENANCE` is a top-level literal dict, 31 entries, every value
  `{"source": <str>, "allowed": True}`; an AST pass confirms all 31 module-level
  UPPER_CASE constants are declared and none is undeclared.
- AST name-resolution audit of every function (no unresolved loads) and an audit
  of the FairApi surface touched: `api.capture`, `api.eef`, `api.gripper`,
  `api.grip`, `api.instruction`, `api.log`, `api.move`, `api.settle`,
  `api.tool_rotation`, `frame.deproject`. Nothing outside the documented surface.

No program was executed against the environment at any point.

## DECLARATION
- Frozen program: `packs/ablC_goal_put_bowl_top_cabinet/program.py`
  md5 `2702c379f778d1f981e3009ee1f0fdc6`, identical to `program_v1.py`
  (same md5, verified on the cluster).
- Episodes run: **0** (no `tools/fair_run.py` invocation of any kind, on any
  split, on any seed; seeds 51-65 untouched, seeds 1-50 untouched).
- Program versions written: **1**.
- PROVENANCE: present, top-level literal dict, 31 entries, all
  `allowed: True`, all sourced to a pack field or to generic
  controller/camera mechanics (no debug-seed observations exist for this cell).
- Cluster writes: only `packs/ablC_goal_put_bowl_top_cabinet/program.py` and
  `program_v1.py`.
