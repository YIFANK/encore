# abl_c2 / ablC_goal_push_plate_front_stove — worker ledger (variant C, no-verify)

Intent: "push the plate to the front of the stove".
Variant C contract: exactly ONE program version, straight to the blind sealed
eval, ZERO episodes run, `tools/fair_run.py` never invoked on any split or seed.

All absolute dates UTC, 2026-08-21.

---

## Inputs consulted

Only `packs/ablC_goal_push_plate_front_stove/` (scp'd down 2026-08-21):
`pack.json` (K=3 demos: keyframe EEF + gripper state, `ee_path`, `ee_path6`,
raw `actions`, `action_scale`, language) and the seven 128x128 `keyframes/*.png`
agentview renders. Nothing else. `LAWS.md` is empty for this cell.

### What the pack says
- Gripper is **open and never actuated**: `gripper_cmd = -1.0` and
  `gripper_state = [0.0362, -0.0362]` at every keyframe of every demo. This is a
  **push with the open gripper**, not a pick-and-place.
- All three demos have the same shape: descend from the start pose
  (~[-0.20, 0.00, 1.17]) to ~[0.04, -0.02, **0.918**], then sweep +y ~0.28 m at a
  z that never leaves 0.9157-0.9236, then curl back in -x and stop.
- Keyframe images: the plate (neutral disc, mean-RGB ~168, reddish rim
  ~168/122/113) starts left-front of the stove; in the final keyframe it sits
  just in front of (below, in image v) the white stove slab.
- Layout does vary a little even inside K=3: plate centroid at 128x128 is
  (61.9,90.0)/(59.7,89.8)/(57.7,91.0) and the burner is at
  (91.0,57.0)/(93.0,58.0)/(91.5,57.3) — so both plate and stove move between
  episodes and neither may be hard-coded.

---

## v1 — hypothesis -> evidence -> verdict

**Hypothesis.** Success is a region predicate on the plate relative to the
stove, so the policy must (a) locate the plate and the stove per episode, (b)
reproduce the demos' *relative* end configuration, and (c) reproduce their
push mechanics (open gripper, fixed z = 0.918, straight sweep). With no
episodes permitted, robustness has to come from perception that is anchored in
measurements taken off the pack keyframes themselves, plus tiered fallbacks.

**Evidence assembled from the pack (all static, images + json only).**

1. *Burner as the stove reference.* Segmenting the keyframes at 128x128, the
   burner is the dark neutral disc whose 3-px halo is 0.62-0.64 bright-neutral
   (the white stove top); every other dark blob in the three t=0 frames scores
   <= 0.32. Its centroid is essentially threshold-invariant (moves < 0.3 px as
   the dark band is swept from V<=85 to V<=115), unlike the bright stove-slab
   centroid, which shifts 3.4 px in v as the bright threshold moves 170 -> 195.
2. *Goal offset.* plate centroid in the FINAL keyframe minus burner centroid in
   the t=0 keyframe: (3.0,21.7) / (4.1,22.2) / (5.3,20.7) px at 128x128 —
   mean (4.13, 21.53), spread +-1.0 px in u and +-0.6 px in v. Measured against
   the bright slab instead it is (3.99, 19.60) (kept as a fallback).
3. *Plate identity.* The plate's halo is 0.33-0.36 "rim-red"
   (R-G >= 22, R-B >= 30, V <= 185) in all three t=0 frames and 0.10-0.37 in the
   three final frames, while **every other round neutral blob in all six frames
   scores 0.00** — the wood table is R-G = 16 and is excluded. This is the
   primary plate test; the brightness band [150,182] is the fallback.
4. *Scale.* Over the push the plate travels with the end-effector (~0.28 m) and
   moves ~37 px at 128x128, giving ~7.6 px per 5 cm, hence a 19-px-wide plate is
   ~0.15 m across. Used only as the radius fallback; at run time the radius is
   measured from the fitted table-plane homography.

**Program mechanism.** Capture `cam_high` with the arm parked at the pack's
start pose; block-average the frame 4x to 128x128 so every threshold above is
applied to exactly the quantity it was measured on. Deproject a grid of pixels,
take the modal z inside a window around the pack's push height as the table
plane, and fit a pixel->table-plane homography by DLT from the bare-table
points. Locate the plate (red-halo test, then brightness band, then nominal) and
the burner (bright-halo test, then bright slab, then nominal). Goal =
homography(burner_px + (4.13,21.53)*4) — applying the offset in the same
projective frame in which it was measured, so the raised stove top's parallax
cancels between measurement and use. Then place the open gripper one plate
radius + 15 mm behind the plate along the plate->goal line, drop to z = 0.918,
and translate by exactly (goal - plate) in 5 cm / 0.7 s increments; park,
re-perceive, and repeat while the plate is short of the goal, inside a 20 s
commanded-motion budget (<= 3 sweeps). If perception fails or returns an
implausible displacement, replay demo1's own push waypoints open loop. No
termination flag, success bit, or any other runtime outcome signal is read.

**Verdict.** UNVERIFIED BY CONSTRUCTION. This is the ablation: no probe, no
selection run, no evidence from any episode. The only checks run were the
permitted static ones (below) plus an offline exercise of the program's own
perception functions against the pack keyframes and a synthetic pinhole camera,
which confirmed the code paths execute and that the plate and burner blobs
selected are the correct ones in all three t=0 frames (and, for the plate, in
all three final frames). That harness executes no policy in any simulator.

---

## Static self-checks (the only checks permitted, all run 2026-08-21)

| check | result |
|---|---|
| `python -m py_compile program.py` | OK |
| forbidden tokens (`gt_pose`, `gt_trace`, `gt_joints`, `_sim`, `env.sim`, `sim.data`, `body_xpos`, `site_xpos`, `geom_xpos`, `.bddl`, `bddl_files`, `init_states`, `pruned_init`, `h5py`, `.hdf5`, `api.robot`, `self.robot`, `check_success`, `OffScreenRenderEnv`) | none present |
| AST scan for any `.done` attribute read | 0 occurrences |
| top-level literal `PROVENANCE` dict | present, 44 entries |
| every entry has `{"source": <non-empty str>, "allowed": True}` | yes |
| every module-level constant covered | 44/44, no orphan keys |
| `run(api)` defined | yes |

---

## DECLARATION

- **Frozen program**: `packs/ablC_goal_push_plate_front_stove/program.py`
  == `program_v1.py`, md5 **`2bcaf7e2096cf5eab20d07d72be7964f`** (verified
  identical on AbakaAI after scp).
- **Episodes run: 0.** `tools/fair_run.py` was never invoked — not on
  `--split eval`, not on seeds 1-50, not on seeds 51-65, not on any seed. No
  policy was executed in any simulator at any point.
- **Program versions written: 1.**
- **PROVENANCE present**: yes — a top-level literal dict covering all 44
  calibrated constants; every source is a pack field, a measurement taken off
  the pack keyframes, or generic controller/camera mechanics.
- **Clean room**: the only cluster paths touched are
  `packs/ablC_goal_push_plate_front_stove/` (read pack.json + keyframes, wrote
  program.py and program_v1.py). No .bddl/.xml/.hdf5/init-state file was opened;
  the bddl path was never used. Nothing under `autoresearch/campaigns/c1|c2|c2fix`,
  no `packs/c1_*` or `packs/c2_*`, no `results/*c1_*` or `results/*c2_*`, no
  `tools/probe_*.py`, and no other abl_c2 cell's pack, results or workspace was
  read. No LIBERO prior from memory or system context was used; every constant
  is re-derived from this cell's pack.

STOP — cell is frozen and handed to the coordinator for the blind sealed eval.

---

## Candidate law (falsifiable, receipt = pack keyframes)

**"Anchor a goal region on the highest-contrast sub-feature of the landmark,
not on the landmark's own silhouette."** Receipt: on this pack, the goal offset
measured against the stove's *bright slab* centroid drifts 3.4 px in v (~0.06 m
of table x) as the brightness threshold moves 170 -> 195, whereas the same offset
measured against the *burner* disc moves < 0.3 px across a 30-level sweep of the
dark threshold. Prediction: any cell whose target is expressed relative to a
large shaded prop will be threshold-fragile unless it is re-anchored on an
embedded high-contrast feature. Untested in-episode by this cell, by design.
