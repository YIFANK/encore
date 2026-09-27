# abl_c2 / ablC_goal_put_cream_cheese_in_bowl — worker ledger (variant C, "no-verify")

Intent: **put the cream cheese in the bowl**
Variant C contract: exactly ONE program version, ZERO episodes, straight to the
blind sealed eval. `tools/fair_run.py` never invoked, on any split, any seed.

---

## 2026-08-21 (UTC) — inputs studied

Only input read: `packs/ablC_goal_put_cream_cheese_in_bowl/` (pack.json, K=3;
keyframes/*.png, 12 frames at 128x128). No .bddl/.xml/.hdf5/init-state file was
opened; no c1/c2/c2fix material and no other abl_c2 cell's pack, results or
workspace was read.

Measurements taken from the pack (these are the whole evidence base):

| quantity | demo0 | demo1 | demo2 |
|---|---|---|---|
| t=0 ee | (-0.2054, 0.0076, 1.1850) | (-0.2052, 0.0184, 1.1605) | (-0.2045, 0.0028, 1.1736) |
| gripper_cmd -1 -> +1 (close) ee | (-0.0213, 0.1119, 0.9104) | (-0.0134, 0.1367, 0.9205) | (-0.0479, 0.1521, 0.9106) |
| carry apex z (ee_path6) | 1.0415 | 1.0790 | 1.0510 |
| gripper_cmd +1 -> -1 (open) ee | (-0.0961,-0.0246, 0.9592) | (-0.0453,-0.0099, 0.9731) | (-0.0835,-0.0016, 0.9785) |
| final keyframe ee z | 1.0118 | 1.0441 | 1.0404 |
| held gripper_state (per finger) | 0.0216 | 0.0211 | 0.0237 |
| episode length | 92 | 113 | 103 |

Image measurements (128x128 keyframes, cam_high/agentview):

- **Cream cheese = the only blue-ish object in the scene.** A mask
  `(B-R>=12) & (B>=45) & (max<=185)` fires on exactly ONE ~50 px blob in each
  of the 12 keyframes and on nothing else anywhere in the image.
- **Bowl = the mottled achromatic grey bowl.** blob area ~165 px, value mean
  104-107, value std 33-36. The confusable neighbours: the plate (~10 cm away
  in y) is achromatic but uniform and bright — value mean 164-166, value std
  ~5; the stove (~0.28 m away in +y) is a much larger blob; the wine bottle
  next to the bowl is near-black (value ~5); the wood table is warm (R-B ~33).
- **Which grey object is "the bowl" is settled by direct pack evidence:** in
  the FINAL keyframe of all three demos the blue cream-cheese blob sits within
  ~1-3 px of the bowl blob centroid ((60.6,70.2)/(62.2,75.1)/(63.1,71.3) vs
  bowl (61.6,71.1)/(61.8,72.6)/(64.1,72.4)). The cheese ends up in the mottled
  grey bowl, not in the plate.
- **Layout variation across the three demos is small**: bowl centroid moves
  <=2.5 px, cheese centroid <=3.3 px, plate <=3.6 px — roughly 2-3 cm. This
  sizes the prior windows used as sanity gates (~5x that spread).

---

## Version v1 (the only version) — hypothesis -> evidence -> verdict

**Hypothesis.** With no episodes available, the highest-expected-value policy
is: (a) reproduce the demos' *heights* exactly, since z is the axis the pack
pins down unambiguously and the table/bowl/box are fixed assets; (b) make
*xy* fully closed-loop from cam_high RGB-D plus a wrist-camera refinement,
since xy is the axis the 50 unseen layouts will move; (c) put every runtime
decision on my own sensors (gripper effort/width, deprojected re-perception,
move residuals) and never on any success or termination signal.

**Mechanism.**
1. `perceive()` captures `cam_high`, builds a base-frame point cloud by
   vectorised pinhole deprojection from `.intrinsics`/`.t_base_cam`, and
   *self-calibrates the camera-axis convention* by scoring the OpenCV and
   OpenGL/MuJoCo variants against `frame.deproject()` at 9 probe pixels
   (this removes the only unverifiable piece of camera math).
2. Table height = median z of warm (R-B>=22) pixels. Objects = points
   >12 mm above it.
3. Cream cheese = largest blue-mask component inside a +/-0.20 m sanity window
   around the demo grasp-xy mean; bowl = achromatic component with value mean
   in [35,158] AND value std >= 10 (the std test is what rejects the plate),
   inside a +/-0.13/0.115 m window around the demo release-xy mean, nearest to
   that prior.
4. Pick: hover at z=1.030 over the cheese, re-detect it from `cam_arm_wrist`
   and accept the correction only if <5 cm, descend to z=0.911 (demo close
   height, floored at table+5 mm), close, then check `api.gripper()` — if
   effort < 2.5 and width is outside [0.008,0.062], do ONE sensor-driven
   regrasp (reopen, lift to 1.00, re-look from the wrist, re-descend, close).
5. Place: carry at z=1.070, wrist-refine the bowl centre (accept if <6 cm),
   descend to z=0.970 (demo release height), open, retreat to 1.060.
6. One conservative retry: park at the demo home pose so the arm leaves the
   agentview frame, re-perceive the cheese, and only if it is >0.090 m in xy
   from the bowl centre (vs the ~0.03 m the demos show for an in-bowl box) run
   pick+place again. This uses no success feedback — it is re-perception of my
   own scene.
7. Every perception path is wrapped so a failure degrades to open-loop replay
   of the pack's mean grasp/release xy rather than aborting.

Deliberate non-choices, all argued from the pack: no wrist-yaw control (the
demos' grasp yaw is only ~0.12 rad from the t=0 yaw, and the grasped width of
0.043 m sits well inside the 0.079 m finger span either way, so the default
straight-down orientation is what the demos effectively used); no adaptive
grasp z from the perceived box top (the eef-to-fingertip offset is not
derivable from the pack, so an adaptive z would need an undeclarable constant);
no `api.done`, no success flag, no runtime reward of any kind.

**Evidence.** By construction of this cell there is NO behavioural evidence:
zero episodes were run. The only evidence is (i) the pack measurements
tabulated above, and (ii) the static self-checks below.

**Verdict.** FROZEN as submitted. Untested against the simulator by design.

---

## Static self-checks (the only checks the variant permits)

- `python -m py_compile program.py` -> OK.
- Forbidden-token scan over the full source for `gt_pose, gt_trace, gt_joints,
  _sim, env.sim, sim.data, body_xpos, site_xpos, geom_xpos, .bddl, bddl_files,
  init_states, pruned_init, h5py, .hdf5, api.robot, self.robot, check_success,
  OffScreenRenderEnv` -> clean (0 hits).
- AST scan for any `Attribute` node with `attr == "done"` -> 0 occurrences.
- `PROVENANCE` parses as a top-level **literal** dict (ast.literal_eval): 30
  entries, every value has `{"source": ..., "allowed": True}`.
- Every module-level uppercase constant (31 assignments, 30 distinct names) has
  a PROVENANCE entry -> 0 missing. Every source is either a pack.json field, a
  pack keyframe-image pixel statistic, the FairApi spec, or generic
  controller/camera mechanics. No debug-seed measurements exist, because no
  debug seed was run.
- `run(api)` is defined at module level.

---

## DECLARATION

- **Frozen version:** `program.py` == `program_v1.py`
- **md5:** `302179bc1bae8b950d718cb055d28a4e` (identical local and on cluster at
  `/mnt/data/YifanKang/Heron/packs/ablC_goal_put_cream_cheese_in_bowl/`)
- **Episodes run: 0** — `tools/fair_run.py` was never invoked, on any split, on
  any seed. `tools/fewshot_run.py` was never invoked. Seeds 1-50 and 51-65 were
  never touched.
- **Program versions written: 1**
- **PROVENANCE present:** yes — top-level literal dict, 30 entries, all
  `allowed: True`, covering every calibrated constant.
- **Clean-room:** no .bddl/.xml/.urdf/.hdf5/init-state file opened; nothing
  under `campaigns/c1|c2|c2fix`, no `packs/c1_*`/`packs/c2_*`, no
  `results/*c1_*`/`results/*c2_*`, no `tools/probe_*.py`, and no other abl_c2
  cell's pack/results/workspace was read. Cluster writes were confined to
  `packs/ablC_goal_put_cream_cheese_in_bowl/{program.py,program_v1.py}`.
- **Prior-knowledge quarantine:** no LIBERO-specific prior from memory or system
  context was used; all constants re-derived from this cell's pack.

STOP — ready for the coordinator's blind sealed eval on seeds 1-50.
