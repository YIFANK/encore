# l90abl / open_top_drawer_s1_vis — notes

Intent: "open the top drawer of the cabinet". Pack = K=3 vision-only demos
(keyframe images + language, no EEF/gripper/action data). Runner: `tools/fair_run.py`
only. Debug seeds 51–65.

## Scene, as read off the pack images and my own debug-seed observations

The three pack demos all show the same thing: a dark cabinet at the left edge of
agentview, and by the final keyframe a drawer has slid out toward the table. The
pack carries no numbers, so every constant below comes from my own captures on
debug seed 51.

Frame convention recovered from `cam_high.t_base_cam` (camera at base
(0.659, 0, 1.610), viewing direction (-0.778, 0, -0.628)): image right = base +y,
image down = base +x. So the cabinet sits at -y and its drawers open toward +y.

Geometry (seed 51, cam_high deprojection + a top-down `cam_arm_wrist` height map
taken while hovering at z=1.25):

| feature | value |
|---|---|
| table top | z = 0.9012 |
| cabinet top slab | z = 1.1272, front edge y ≈ -0.238 |
| drawer front panel | y ≈ -0.236 |
| three handle rails | z = 1.097 / 1.021 / 0.954 (0.071 apart) |
| top rail extent | x ∈ [-0.045, 0.045], y ∈ [-0.222, -0.198] |
| slot behind top rail | y ∈ (-0.235, -0.222), ~13 mm wide, open to the floor |
| fingertip below eef | 0.011 (from a descent stalling on the slab) |
| open gripper half-width | 0.039 |

Finger axis: with the default straight-down wrist the fingers separate along
**base y**, read off the probe-1 wrist camera (its +x column is base -y, and the
two finger blobs sit at opposite image-u extremes).

## Version chain

### probe_v0 — perception only (seeds 51, 53)
Hypothesis: locate the handles. Evidence: the table above. Verdict: the top rail
is a cylinder along x, ~24 mm in y, standing ~35 mm proud of the panel and 30 mm
below the cabinet's top slab.

### probe_v1 — top-down straddle of the rail (seed 51)
Hypothesis: descend with the default wrist at the rail centre, close, pull.
Evidence: the descent stalled at eef z=1.137 (fingertips 1.128 ≈ the slab top
1.127); the close returned width 0.0010 / effort 0.05 (empty air) and the +y pull
swept freely to y=+0.082. Verdict: refuted — the far finger lands on the cabinet
top slab before the near finger is anywhere near the rail.

### probe_v2 — four top-down variants (seeds 51, 53, 55, 57)
S0 open fingers with the rear finger aimed down the slot; S1 the same without
closing; S2 a closed gripper down the slot; S3 enter low in front, back up to the
panel, rise into the slot. Evidence: **all four stalled**, at eef z = 1.139,
1.139, 1.148 and 1.095 respectively, and the stall height *rose* as the target
moved further into -y. Every close read effort 0.05. Verdict: top-down is
mechanism-blocked — this is not one obstacle but the hand body (elongated along
base y, the finger-separation axis) fouling the cabinet's top front corner. The
rail lives in a 30 mm-tall, 35 mm-deep pocket a top-down hand cannot enter.
0/4.

### probe_v3 — rotate the wrist (seeds 51, 53, 55, 57)
S0 yawed 90° (hand body along base x), closed, into the slot. S1 pure
reachability test of a horizontal wrist. S2 horizontal wrist
(`R = [[1,0,0],[0,0,-1],[0,1,0]]`: approach axis -y, fingers separating along
base z), open, creep in -y at rail height, close, pull. S3 the same but closed and
slipped under the rail then lifted.

Evidence: S1 held the horizontal wrist at every commanded pose with residual
≤0.012 — the orientation is freely reachable. **S2 succeeded**: the creep stalled
at eef y = -0.2116 (residual jumped 0.009 → 0.0185), the close returned
**width 0.0174, effort 3.0**, and the +y pull ran the drawer from -0.212 to
-0.064. S0 stalled at z=1.185; S3 hooked nothing (lift residual 0.034) and swept
free. Verdict: **1/4, and the one that worked is the horizontal wrist.** The
mechanism is that a -y approach trails the hand body away from the cabinet
instead of into it.

### program_v1 — S2 with per-seed perception, grasp verification and retry
Grey-metal mask (value > 110, channel spread < 14) restricted to y < -0.12 and
z ∈ (0.93, 1.15); highest dense 5 mm z band; grasp at that band's top minus
0.0087 with the horizontal wrist; creep in -y until the arm stalls; close; accept
only on effort ≥ 2.5 and width ∈ (0.006, 0.032), else retry at dz ∈ {0, ∓0.008,
∓0.016}; then up to three +y drags, stopping when the drawer stops advancing.

Evidence: probe **8/8** (`results/fs_l90abl_open_top_drawer_s1_vis_v1`), formal
**15/15** (`results/sel_l90abl_open_top_drawer_s1_vis_v1`). Every seed grasped
first try at dz=0 with width 0.0174 / effort 3.0. The ep51 GIF shows the top
drawer fully extended.

**Correction found while auditing the receipt.** The grey mask is dominated by
the cabinet's top slab, so the band it reports is that slab (z≈1.1276), *not* the
rail (1.097) — v1's constant `RAIL_R` and its PROVENANCE entry described the
wrong surface. The program nevertheless works, and for a checkable reason: the
slab is rigidly above the top drawer, so grasping at 1.1276 − 0.0087 = 1.1189
puts the **lower** finger at 1.1189 − 0.039 = 1.0799, inside the measured rail
band 1.073–1.097, and the closure clamps the rail against it — hence the width
receipt equal to the rail's own diameter. Nothing horizontal is trusted: the
reported y is only a standoff and the creep-to-stall parks the tool on the rail.

### program_v2 — v1 with the datum named and provenanced correctly (FROZEN)
Behaviour-identical to v1: every numeric constant is unchanged except `FALLBACK`,
which never fires when perception succeeds and which v1 had set to the rail
(-0.198, 1.097) rather than to the datum this perception actually reports. Docs,
identifiers (`RAIL_R` → `TOP_REF_DROP`, `find_top_rail` → `find_top_ref`) and the
PROVENANCE entry now state what is measured. Re-run formally so the receipt
matches the frozen file's md5.

Evidence: formal **15/15** (`results/sel_l90abl_open_top_drawer_s1_vis_v2`).

## Candidate law (offered to LAWS.md)

*A handle in a shallow pocket forbids the top-down wrist.* When a graspable
feature sits within roughly one hand-height below an overhanging surface and
within one hand-depth in front of it, no top-down descent reaches it — the hand
body, which is elongated along the finger-separation axis, fouls the overhang
while the fingertips are still tens of millimetres clear. The receipt is a stall
height that *rises* as the target moves further under the overhang (probe 2:
1.095 at y=-0.155, 1.139 at y=-0.190, 1.148 at y=-0.229), which distinguishes it
from a fingertip contact (constant stall height) and from a reach limit (the same
poses were held freely with a rotated wrist). The fix is to rotate the approach
axis into the free direction — here -y, the direction the drawer opens — so the
body trails away from the obstruction; then creep until the arm stalls rather
than aiming, which self-centres the tool on the feature.

## DECLARATION

- **Frozen version:** `program_v2.py`, copied to
  `packs/l90abl_open_top_drawer_s1_vis/program.py`.
  md5 `5335cf1ea3e84848c1771f50bd42ec63` — **program.py == program_v2.py**
  (program_v1.py is `346938e811d545dbf3edb57323aabc20`, archived).
- **Selection receipt (full 15 debug seeds 51–65):** **15/15**, dir
  `results/sel_l90abl_open_top_drawer_s1_vis_v2`.
- **Receipt chain:**
  - probe_v1 top-down straddle — 0/1 (`fs_..._probe1`)
  - probe_v2 four top-down variants — 0/4 (`fs_..._probe2`)
  - probe_v3 four wrist rotations — 1/4, horizontal wrist only (`fs_..._probe3`)
  - program_v1 probe subset (51,53,…,65) — 8/8 (`fs_..._v1`)
  - program_v1 formal 15 — 15/15 (`sel_..._v1`)
  - program_v2 formal 15 — **15/15** (`sel_..._v2`)
- **PROVENANCE:** present as a top-level literal dict in `program.py`, covering
  R_HORZ, GRAY_V_MIN, GRAY_SAT_MAX, CAB_Y_MAX, TOP_REF_DROP, Z_LO/Z_HI,
  BAND_HALF, STANDOFF, ADV_STEP/ADV_STALL, GRIP_LO/GRIP_HI, Z_RETRIES and
  PULL_DIST/PULL_SEC. Every source is this pack, a debug-seed (51–65)
  measurement, or generic controller/camera mechanics.
- **Clean room:** writes confined to `packs/l90abl_open_top_drawer_s1_vis/*` and
  `results/*l90abl_open_top_drawer_s1_vis*`; scratch probe programs and their
  `.npz` captures removed from the pack dir. No benchmark asset, init state,
  demonstration hdf5, other campaign's artifact, or any `program*.py`/`NOTES.md`
  under any pack directory was read. `tools/fewshot_run.py` was never invoked and
  `api.done` was never read.
