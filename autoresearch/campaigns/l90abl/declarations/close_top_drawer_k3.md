# l90abl / close_top_drawer_k3 — NOTES

Intent: "close the top drawer of the cabinet". Runner: `tools/fair_run.py` only.

## Pack reading (packs/l90abl_close_top_drawer_k3/pack.json, K=3)
- Every keyframe of every demo has `gripper_cmd = -1.0` and `gripper_state`
  ~ (+0.036,-0.036) -> (+0.040,-0.040). **The gripper never closes.** The skill
  is a push, not a grasp.
- ee_path shape per demo: start ~(-0.20, 0.00, 1.17); swing to y ~ -0.09..-0.11
  while advancing +x; descend to z ~ 1.05; then a terminal **+y sweep** at
  roughly constant x and z.
- Demo final eef: d0 (0.0587, 0.1144, 1.0642), d1 (-0.0084, 0.0904, 1.0443),
  d2 (-0.0113, 0.0705, 1.0133). So the closing direction is **+y**.
- ee_path6 columns 4-6 are euler-like: roll ~ pi (wrist straight down),
  yaw drifting 0 -> ~0.3 rad. rotation=None (straight down) is close enough.

## v1 — perception probe (no motion), seeds 51,55,59. 0/3 by construction.
Established the base frame from cam_high: **+x = toward the viewer (down-image),
+y = to the right in image**. Camera at (0.659, 0, 1.610). Table z = 0.901.

## v2 — perceive + push. seeds 51,53,55,57,59,61,63,65 -> **0/8**
(results/fs_l90abl_close_top_drawer_k3_v2)
Hypothesis: locate the drawer as "tall stuff on the +y side", push +y.
Evidence: perception was **poisoned by the robot arm** (arm points reach
z=1.371 at x -0.19..-0.22), so z_rim came out 1.362 and the push ran at
z=1.279 — 20 cm above the drawer, touching nothing. Null run.
Verdict: rejected; but its logged (y,z) cross-sections gave the real geometry.

### Scene geometry (debug seeds 51/53/57/61/65, cam_high cross-sections)
| feature | value |
|---|---|
| table | z = 0.901 |
| drawer rim top / cabinet top slab | z = 1.121 .. 1.127 |
| drawer inner floor | z = 1.055 .. 1.060 |
| drawer front panel face | y = 0.050 .. 0.080 (seed-dependent), z 1.055..1.124 |
| drawer handle | y 0.025..0.050, z 1.085..1.098, x -0.05..+0.05 |
| cabinet body front face | y = 0.195 .. 0.225, visible from z 1.002 up |
| drawer x span | ~ [-0.11, +0.12] |
| robot arm contamination | x < -0.19 and z > 1.15 |

So a closed drawer front sits near y ~ 0.21; the drawer starts ~0.13-0.16 m out.
Note the demos' terminal +y sweeps end at y = 0.07..0.11, i.e. **short of 0.21** —
either their drawers were less open or a partial close fires the predicate.

## v3 — corrected perception + push at the demos' raw eef z. seeds 51..65 odd -> **0/8**
Design: slab band z in [1.100, 1.145], y > -0.02, x in [-0.19, 0.22] (excludes
the arm); y_face = 2nd pct of slab y; x_centre from the front strip; push to
y=0.215 at z = 1.0443 (demo median), retrying at 1.0642 then 1.0133 and
re-perceiving between attempts.
(results/fs_l90abl_close_top_drawer_k3_v3)
Perception was now right (y_face 0.068, z_rim 1.127, x_centre 0.000 on ep51).
Evidence: the descent to z=1.0443 **stalled at z=1.1309** and the gripper width
collapsed 0.079 -> 0.040. With `rotation=None` the jaws separate along **+/-y**,
which is the push direction, so the leading finger lands on the drawer rim /
handle on the way down. All 1000 sim steps burned on blocked moves; attempts 1
and 2 never moved at all.
Verdict: **the failure is the wrist yaw, not the perception.** Taking the demos'
final eef z (1.01-1.06) literally is also wrong -- see v4.

## v4 — yaw the wrist 90 deg + put the FINGERTIPS in the panel band. **8/8**
(results/fs_l90abl_close_top_drawer_k3_v4, seeds 51,53,55,57,59,61,63,65)
Two changes:
1. `rotation = Rz(90 deg) @ straight-down`, so both fingers sit at the same y.
   The descent in front of the panel is then clean and both jaws push together.
2. Height is set from the **fingertips**, not the eef origin. An in-episode
   probe pressed the yawed gripper onto bare table (z=0.901) and it stalled at
   eef z=0.9102 -> **tip_off = 0.0092 m**, identical on every seed. Push height
   = z_rim - 0.030 + tip_off = 1.103.
Every seed closed on attempt 0: pushed to eef y ~ 0.177..0.182, re-perceived
face y 0.179..0.214 (>= CLOSED_Y 0.16). 372 sim steps of 1000.

**Law candidate.** *The demo's eef z is not the contact height when the tool is
a push face.* The three demos end at eef z 1.0133/1.0443/1.0642, all at or below
the front panel's bottom edge (1.055); the contact is made by the hand body
above the eef origin. Re-deriving the height from perceived geometry plus a
measured fingertip offset is what worked, and it also explains the 5 cm spread
across the demos: a 7 cm-tall panel tolerates it.

**Law candidate.** *A jaw axis parallel to the push direction is a snag.* Yaw
the wrist so the jaws straddle the push axis, not lie along it.

## v5 — v4 with the in-episode tip probe replaced by TIP_OFF = 0.0092
The probe is a blocked move at a reach-limited pose; its `pre` move already
undershot to (0.104,-0.301,0.918) instead of (0.20,-0.32,1.10). It returned the
same number on every seed, so it is a constant, and its failure mode (a bad
reading -> fallback 0.033 -> a push 24 mm too high, riding over the rim) is a
real risk on unseen seeds. Removing it also frees ~150 steps, spent instead on
a third verified retry (tip depths 0.030, 0.050, 0.015 below the rim).

## Selection
Formal full-15 (seeds 51..65) runs for **both** v4 and v5:
results/sel_l90abl_close_top_drawer_k3_v4 and ..._v5. Freeze the argmax.

### Selection receipts
- **v5: 15/15** — `results/sel_l90abl_close_top_drawer_k3_v5`, seeds 51..65.
  Every seed closed on **attempt 0**; re-perceived drawer face y = 0.168..0.185
  (threshold 0.16). 91-93 sim steps of 1000 — the two spare retry heights were
  never needed.
- **v4: 15/15** — `results/sel_l90abl_close_top_drawer_k3_v4`, seeds 51..65
  (same mechanism, with the in-episode tip probe).
The two tie at 15/15. **v5 is frozen**, on robustness: it has no blocked-move
probe at a reach-limited pose (whose bad-reading fallback would push 24 mm too
high), it spends 92 sim steps instead of 372, and it keeps two unused verified
retry heights in reserve.

### Receipt chain
| version | seeds | result | dir |
|---|---|---|---|
| v1 | 51,55,59 | 0/3 (perception probe, no motion by design) | fs_..._v1 |
| v2 | 51..65 odd | 0/8 — arm poisoned z_rim, pushed 20 cm too high | fs_..._v2 |
| v3 | 51..65 odd | 0/8 — jaws along the push axis snagged the rim | fs_..._v3 |
| v4 | 51..65 odd | **8/8** | fs_..._v4 |
| v4 | 51..65 all | **15/15** (formal) | sel_..._v4 |
| v5 | 51..65 all | **15/15** | sel_..._v5 |

## DECLARATION
- **Frozen version: v5.** `packs/l90abl_close_top_drawer_k3/program.py`
  md5 `bc7baf6c6d49980c0b3842d7bc8657d6` == `program_v5.py` (verified on the
  cluster).
- **Selection receipt (full 15 debug seeds 51-65): 15/15**, dir
  `results/sel_l90abl_close_top_drawer_k3_v5`. Every seed closed on attempt 0;
  re-perceived drawer face y 0.168-0.185 against a 0.16 threshold; 91-93 sim
  steps of the 1000-step budget.
- **Receipt chain:** v1 0/3 (perception probe, no motion) -> v2 0/8 -> v3 0/8 ->
  v4 8/8 probe subset, then 15/15 formal -> v5 15/15 formal. Every formally
  probed version is archived as `program_vN.py` in the pack directory.
- **PROVENANCE:** present as a top-level literal dict in `program.py`, covering
  TIP_OFF, TIP_DEPTHS, Y_END, SLAB_LO/SLAB_HI, TRAVEL_Z, DESC_STANDOFF,
  CLOSED_Y, R_YAW90, PERC_CROP, FALLBACK. Every constant is sourced to either
  the pack (demo gripper commands, ee_path standoffs) or a debug-seed
  measurement (cam_high cross-sections on seeds 51-65, the bare-table fingertip
  stall on seeds 51-57). No LIBERO prior knowledge was used; the quarantined
  facts in context were not consulted.
- **Mechanism, in one line:** the demos never close the gripper and always end
  with a +y sweep, so the skill is a push; perceive the drawer's front rim from
  cam_high, yaw the wrist 90 deg so the jaws straddle rather than lie along the
  push axis, put the *fingertips* 30 mm below the perceived rim, and push to
  y=0.215 (the cabinet's front face), verifying by re-perception.
