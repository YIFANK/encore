# c2clean / goal_turn_on_stove_pos_k3 — worker notes

Intent: **turn on the stove**. Runner: `tools/fair_run.py` only. Pack:
`packs/c2clean_goal_turn_on_stove_pos_k3/` (K=3 demos, 3 keyframes each).

## Reading the pack

Three demos, 80/96/89 steps. Each one: fly out from home to a point on the
table, close the gripper there, then hold that point and rotate.

| demo | grasp keyframe ee (xyz) | final ee | closed `gripper_state` sum |
|---|---|---|---|
| demo0 | (-0.4328, 0.2209, 0.9279) | (-0.4197, 0.2020, 0.9308) | 0.0306 |
| demo1 | (-0.4258, 0.2194, 0.9360) | (-0.4249, 0.2051, 0.9294) | 0.0285 |
| demo2 | (-0.3777, 0.2093, 0.9633) | (-0.3834, 0.2105, 0.9292) | 0.0346 |

Position is essentially frozen after the grasp (±15 mm) — the whole task is
the rotation. The grasp x differs by 55 mm across the three demos, so the
target's table position is **not** fixed and must be perceived.

**Rotation convention.** `ee_path6` carries an euler triple. Matching the raw
`actions[:, 3:6] * action_scale[3:6]` against the per-stride pose deltas:

| convention | delta applied in world | delta applied in tool |
|---|---|---|
| `xyz`  | 0.299 / 0.207 / 0.274 | 0.364 / 0.230 / 0.332 |
| `xzy`  | 0.253 / 0.147 / 0.172 | 0.369 / 0.275 / 0.408 |
| `XZY`  | 0.365 / 0.277 / 0.411 | **0.209 / 0.114 / 0.115** |

(mean per-stride residual, one number per demo). The two best fits agree that
the post-grasp motion is a rotation about the **vertical** axis — which also
matches the raw actions, where dim 5 (the z rotation) saturates at +0.375 for
the whole post-grasp phase while dims 3 and 4 stay near zero. Magnitude from
the grasp keyframe to the final keyframe: **0.93 / 1.08 / 1.50 rad**. The two
fits disagree on the *sense*, so the sense was settled empirically (v2).

Demos also hold dz at -0.37..-0.52 through the whole twist: they keep pressing
down while turning.

## Reading the scene (debug seeds, cam_high RGB-D)

Perception dump probe (`probe_dump.py`, seeds 51/52/53/58/65 — 72 sim steps,
no motion) streamed a 128×128 RGB + depth crop out through `api.log`.

- table top z = **0.9009**; `t_base_cam` puts cam_high at (0.659, 0, 1.610)
  looking down the -x axis at ~39° below horizontal.
- The target is a **stove control knob**: a dark disc whose top face is a
  horizontal plane at z = 0.920 (table+0.019) spanning ~84 × 84 mm, with a
  raised **bar-shaped handle** on top at z = 0.960 (table+0.059), ~84 mm long
  and ~25 mm wide. Luminance 12–25, against a stove slab at 64 and a table at
  160. The same silhouette (wide dark base, narrow upright tab) appears in the
  pack keyframes, with the bar's long axis along world x in both.
- Demo grasp z 0.929 = table + 0.028, i.e. the jaws bite the bar 31 mm below
  its top. Demo closed width 0.0285–0.0346 ≈ the bar's narrow dimension, and
  the open width is 0.0778, so the jaws must close **across** the bar — which
  fixes the wrist yaw to the bar's long axis (fingers open along tool y).
- The knob sat at (-0.037, 0.130) on all five probed debug seeds, i.e. the
  `_pos` layout is fixed within the split but ~390 mm in x away from where the
  demos found it. Perception is therefore load-bearing for eval seeds 1–50.
- Distractors in the same darkness band: the wine bottle and the cabinet, both
  of which carry dark pixels above table+0.10 (the knob does not), and the
  cabinet also spans 0.356 m (the knob 0.084). Those two tests isolate the knob.

## Version log

### v1 — perceive knob, grasp bar, twist about tool z (positive sense)
*Hypothesis*: replicate the demos — straight-down approach yawed to the bar,
close, twist 1.55 rad about the tool approach axis pressing down.
*Evidence* (`results/fs_..._v1`, seeds 51,53,55,57,59,61,63,65): **0/8**.
Detection was clean on every seed (knob = (-0.037, 0.130), yaw -0.034, and the
bottle/cabinet/slab components all rejected) and the grasp took hold — closed
width 0.0257 at effort 3.00, matching the demos' 0.029–0.035. But through the
twist the jaws were steadily **forced open**, 0.0257 → 0.0350 → 0.0501 →
0.0574. The handle was not following the wrist.
*Verdict*: right target, right grasp, wrong twist sense (or wrong axis).

### v2 — A/B both twist senses inside one episode (diagnostic)
*Hypothesis*: the two surviving euler conventions disagree only on the sense;
run both back to back, retreating and re-perceiving between them, and let the
handle bar's re-perceived yaw say which one turned it.
*Evidence* (`results/fs_..._v2`, seeds 51,53,55,57): **4/4**. On ep51:

```
P0  knob=(-0.0370,0.1298) yaw=-0.034
A+  closed w=0.0257 e=3.00  ->  after twist w=0.0541 e=3.00
P1  knob=(-0.0389,0.1296) yaw= 0.006          <- bar did NOT move
B-  closed w=0.0258 e=3.00  ->  after twist w=0.0297 e=3.00
P2  knob=(-0.0354,0.0260) yaw= 0.893          <- bar turned 51 deg
```

*Verdict*: **TWIST_SIGN = -1** (`R_app @ Rz(-theta)`, i.e. counter-clockwise
seen from above). The jaw width is a clean receipt for the two cases: a turn
that takes hold ends near 0.030, a turn that fails ends near 0.054. (The
burner-redness sensor I added was useless — it locks onto a warm-toned prop at
(-0.055, -0.063) and reads the same before and after; dropped.)

### v3 — single negative twist, with a re-seat retry on the pry signature
Same as v1 with `TWIST_SIGN = -1`, plus: if the post-twist width exceeds
PRY_W = 0.045 the program re-opens, lifts, re-perceives and turns again.
*Evidence*: probe (`results/fs_..._v3`, seeds 51,53,55,57,59,61,63,65) **8/8**,
retry never fired (ep51: closed 0.0257 → twisted 0.0301).
Formal selection: see the declaration below.

## DECLARATION

- **Frozen version: v3.** `packs/c2clean_goal_turn_on_stove_pos_k3/program.py`
  md5 `1c820d76207b4784c3d8079d78ab759c` == `program_v3.py` (same md5).
- **Selection receipt: 15/15** on the full debug split (seeds 51–65),
  `results/sel_c2clean_goal_turn_on_stove_pos_k3_v3`. Every episode terminated
  in 134–203 sim steps with a post-twist jaw width of 0.027–0.030 m — the
  "turn took hold" receipt from v2 — and the re-seat retry never fired.
- **Receipt chain**
  | version | run dir | seeds | result |
  |---|---|---|---|
  | probe_dump (perception only) | `fs_..._dump` | 51,52,53,58,65 | 0/5, by design — no motion |
  | v1 (positive twist) | `fs_..._v1` | 51,53,55,57,59,61,63,65 | **0/8**, jaws pried open 0.026→0.057 |
  | v2 (A/B diagnostic, both senses) | `fs_..._v2` | 51,53,55,57 | **4/4**, negative sense identified |
  | v3 (negative twist + re-seat retry) | `fs_..._v3` | 51,53,55,57,59,61,63,65 | **8/8** |
  | v3 formal selection | `sel_..._v3` | 51–65 (all 15) | **15/15** |
- **PROVENANCE** present in `program.py` as a top-level literal dict covering
  all eleven calibrated constants: GRASP_DZ, HOVER_DZ, PRESS_DZ, TWIST_RAD,
  TWIST_SIGN, DARK_LUM, KNOB_BAND, KNOB_TALL, KNOB_MAX_SPAN, BAR_DZ, PRY_W.
  Sources are pack fields (demo keyframe ee / gripper_state, ee_path6 relative
  rotation, raw actions) and debug-seed cam_high RGB-D measurements only.
- No LIBERO-specific prior knowledge was used: the table height, the knob's
  geometry, the darkness threshold, the grasp depth and the twist sense were
  all re-derived from this pack plus debug seeds 51–65. Seeds 1–50 were never
  touched; `tools/fewshot_run.py` was never invoked.
