# c2k1clean / goal_turn_on_stove_pos_k1

Intent: "turn on the stove". K=1 pack, `_pos` perturbation. FAIR_PROTOCOL v1.1.1,
runner `tools/fair_run.py` only. No shared note file.

## What the pack says

`pack.json` (demo0, length 80, 3 keyframes):

- t=0   eef (-0.2091, 0.0014, 1.1558), gripper open
- t=41  eef (-0.4328, 0.2209, 0.9279), gripper command flips to CLOSE
- t=79  eef (-0.4197, 0.2020, 0.9308), rpy pitch 0.054 -> 1.447

Relative attitude t41 -> t79 is a rotvec of **0.978 rad about world
[0.183, 0.957, 0.084] ~ +Y**, with the eef translating only 0.008 m. Raw actions
t=54..77 hold `dz = -0.49` (saturated push, action_scale[2] = 0.3869) with a
constant rotation command while the gripper stays closed. Keyframe
`demo0_t0079.png` shows the burner rings lit red, so the twist is the "on" event.

## What the debug seeds show (v1, pure perception probe, 0 motion)

Dumped 512x512 RGB-D through `api.log` (zlib+base64, 1800-char chunks) and did
the perception offline. Seeds 51/53/55:

- table plane z = 0.9012
- the stove is a grey slab, top ~0.926-0.931, spanning ~0.29 x 0.30 m
- the knob is a **separate near-black object on the table** (mean RGB ~21):
  a disc z 0.907-0.923, diameter ~0.088 m, with an upright **fin** above it,
  z 0.923-0.960, 0.023 m thick in y, extended in x. Fin plane normal = world y,
  i.e. a lever that swings in the x-z plane about a +y hinge — exactly the axis
  the pack rotates about.
- the only other dark table object is the wine bottle: 0.148 m tall, y-extent
  0.040 m. Height <= 0.09 AND y-extent >= 0.055 separates knob from bottle.
- knob centre moves ~15-30 mm across seeds (the `_pos` perturbation), so it must
  be perceived, not hard-coded.

## Version chain

| v | change | probe | receipt |
|---|--------|-------|---------|
| v1 | perception probe, no motion | 51,53,55 | 0/3 (expected) — gave the scene model above |
| v2 | grasp fin at its median x, straight-down jaws, 8 coarse twist steps to 1.4 rad about +Y, press 0.04 | 51,53,55,57 4/4; **formal 15: 12/15** (`sel_..._v2`) | works, but by accident: on the wins the fin pops out of the jaws at TW1 and the arm lurches +0.08 m in x |
| v3 | pack's own grasp attitude (yaw -0.372), 14 fine twist steps, press 0.015 | 51,52,54,56,58,60,62,64 | **0/8** — rotation tracked cleanly to 0.77 rad then the wrist flipped; GIF shows the fin **unmoved** |
| v4 | v3 but straight-down jaws (grip 0.0253 = fin thickness) | same 8 | **0/8** — flip at ~0.55 rad, fin unmoved |
| v5 | v4 with the wrist 180 deg about tool z | same 8 | **0/8** |
| v6 | grip the fin, pure +x translation push, no wrist rotation | same 8 | **0/8** |
| v7 | same, -x | same 8 | **0/8** |
| v8 | **blade push**: close the empty jaws, park them 0.045 m behind the fin at fin-top height, drag +x in 12 steps | same 8 | **8/8** |
| v9 | v2 but grasp the fin's back end (x_min + 0.015) | same 8 | **8/8** |

### Verdict on the mechanism

A pinch on the fin is not a rigid grip: the jaws close to 0.0253 m on a 0.023 m
plate, so the fin rotates *inside* the jaws and tool rotation does nothing
(v3/v4/v5 final frames show the fin exactly as it started). Pushing at the fin's
base has no moment arm (v6/v7). What turns it is **lateral force high on the
fin**: either a blade drag from behind (v8) or the same coarse twist applied at
the fin's rear end where there is a lever arm (v9). The v2 wins were this by
accident — a saturated press plus an overshooting rotation command that shoved
the arm forward.

v8 leaves a legible receipt in its own log: the push stalls with a rising
residual at the fin (PU4-5, residual 0.019 -> 0.031), then breaks through as the
fin topples.

## Selection

See DECLARATION below.

| v10 | v8 with named constants + a PROVENANCE entry for every calibrated constant (motion numerically identical to v8) | **formal 15: 15/15** (`sel_..._v10`) | frozen |

## DECLARATION

- **Frozen version: v10.** `packs/c2k1clean_goal_turn_on_stove_pos_k1/program.py`
  md5 `6d484c4660a5d5e370f239f316a0f5cc` == `program_v10.py` (verified on the
  cluster).
- **Selection receipt: 15/15 on the full 15 debug seeds (51-65)**, dir
  `results/sel_c2k1clean_goal_turn_on_stove_pos_k1_v10`.
- **PROVENANCE present**: 11 entries covering DARK_MAX, KNOB_MAX_H, KNOB_MIN_W,
  KNOB_BASE_H, MIN_PIX, FIN_BAND, STANDOFF, PUSH_Z, PUSH_STEP, N_PUSH, R_DOWN.
  Every constant is sourced from this pack or from my own debug-seed
  observations; nothing is carried in from outside the cell.

### Receipt chain

| version | seeds | result | dir |
|---|---|---|---|
| v1 (perception probe) | 51,53,55 | 0/3 (no motion) | `fs_..._v1` |
| v2 | 51,53,55,57 | 4/4 | `fs_..._v2` |
| v2 | 51-65 (formal) | **12/15** (52,60,62 fail) | `sel_..._v2` |
| v3 | 51,52,54,56,58,60,62,64 | 0/8 | `fs_..._v3` |
| v4 | same 8 | 0/8 | `fs_..._v4` |
| v5 | same 8 | 0/8 | `fs_..._v5` |
| v6 | same 8 | 0/8 | `fs_..._v6` |
| v7 | same 8 | 0/8 | `fs_..._v7` |
| v8 | same 8 | 8/8 | `fs_..._v8` |
| v8 | 51-65 (formal) | 15/15 | `sel_..._v8` |
| v9 | same 8 | 8/8 | `fs_..._v9` |
| v10 (= v8 + PROVENANCE) | 51-65 (formal) | **15/15** | `sel_..._v10` |

No eval seed (1-50) was ever run or read; `--split debug` was used throughout.
