# c2clean / obj_alphabet_soup_pos_k3

Intent: "pick up the alphabet soup and place it in the basket".
Runner: `tools/fair_run.py` only. Splits sealed (debug 51-65; eval 1-50 never touched).

## Pack reading (before any run)

- K=3 demos, all one shape: reach to ~(-0.11, -0.24, 0.045), close, carry to
  ~(-0.03, +0.25, 0.14), open. Roll ~3.11 rad (straight-down wrist) throughout.
- **The closing height is in the pack.** demo0 t42 and demo2 t46 close at
  ee z=0.045. demo1 closes first at z=0.076 and that grasp came back EMPTY
  (`gripper_state` [0.0012, -0.0013] at t65), re-opened, and re-closed at
  z=0.045 (t80), which held. So 0.045 is a demonstrated hold and 0.076 is a
  demonstrated miss -- the pack contains its own negative control.
- Release: ee z = 0.152 / 0.145 / 0.124 over the basket.
- **Identity cue.** Projecting each demo's closing EEF into that demo's own
  keyframe (cam_high K/T taken from a debug seed) lands at (u,v) = (31.5,61.5),
  (31.7,61.0), (32.4,61.1) on a patch of RGB ~ (29,43,79) in all three demos:
  a blue-bodied can. The pack-difference agrees -- that prop is present in
  demo0_t0000 and gone from demo0_t0128 after the place.
- The demo layout is NOT the debug layout: the demos grasp at y ~ -0.24, the
  debug seeds put the can at y = +0.058. So layouts really do permute, and in
  the demo layout the target can stands hard against the milk carton. That
  drove the v2 hardening below.

## v0 / v0b -- sensor dumps, no motion

`api.log` truncates a message at 2000 chars (v0 receipt), so v0b chunks a
zlib+base64 RGB-D payload across many lines. v0b ran all 15 debug seeds;
perception was then developed offline against those dumps.

- Table plane z = 0.000-0.002. cam_high at (0.897, 0, 0.65), fx = 618.04.
  The offline reprojection reproduces `api.deproject` to 1e-6 at 6 test pixels,
  so the offline detector and the on-cluster one are the same function.
- **The reset arm poisons a max-z height map**: it occupies z 0.25-0.49 over
  x[-0.235,-0.095] and swallows the two far props, including the target. A
  height BAND (z < 0.200) removes it and the scene segments cleanly. This was
  the single biggest perception trap in the cell.
- Scene: 2 geometrically identical cans (rim 0.064-0.070 across, top z=0.081),
  a bottle (top 0.148), a milk carton (0.140), a small box, and the basket
  (0.16 x 0.17 footprint, rim 0.143, hollow interior).
- **Prop positions are IDENTICAL across all 15 debug seeds**; only the basket
  moves, over x[-0.017,+0.012], y[+0.240,+0.270]. The program therefore
  perceives everything rather than hard-coding a layout, since eval seeds are
  unseen and the demos prove other layouts exist.
- **The colour cue needs a body gate.** Averaged over the whole top-down
  component the grey lid dominates and the target reads B-R = -0.2, which is
  not separable. Restricted to the body band (0.015 < z < top-0.015) the target
  reads B-R = **+9.2**, the only positive value in the scene (bottle -4.4,
  basket -7.1, other can -21.2, milk -35.4). Inside the can-shape gate the
  contest is +9.2 vs -21.2.
- Gripper opens to 0.0778 against a 0.065 can: ~6 mm clearance per side.

## v1 -- perceive, rank, grasp at the demo height

Hypothesis: rank can-shaped components by body blueness, grasp the winner at
the pack's own closing height 0.045, release at the perceived basket centre at
0.145; verify each attempt by gripper width after the lift.

Evidence: **8/8** on the probe subset 51,53,...,65
(`results/fs_c2clean_obj_alphabet_soup_pos_k3_v1`). Every seed picked the
target on attempt 0 with the retry ladder unused, closed to w=0.0625 at
effort 3.00 (an empty close reads 0.0025), and released holding.

Verdict: the mechanism is right. But 8/8 on a layout that never varies says
nothing about eval, so before selecting I measured the envelope and hardened
the two things the debug layout cannot exercise.

## env -- aim-envelope probe (not a task attempt)

One episode per seed sweeping several aim offsets, re-perceiving before each so
drift does not accumulate into the measured offset. Seeds 52, 58
(`results/fs_c2clean_obj_alphabet_soup_pos_k3_env`), identical results on both:

| offset | +0.008 | +0.014 | +0.020 |
|---|---|---|---|
| along y | HOLD | miss | miss |
| along x | HOLD | HOLD | HOLD |

So **y is the closing axis and the only tight one** (tolerance ~±0.010); x is
forgiving past 0.020 (it holds a 0.050 chord instead of the 0.0625 centred
bite). A retry ladder must therefore walk y, and its first step must exceed
0.010 or it re-tries inside the same miss.

## v2 -- FROZEN

Two hardenings over v1, both aimed at layouts the debug split cannot show:

1. **A second component pass on a low height slab** (0.020 < z < 0.110). In the
   pack's own layout the target can stands against the milk carton; under plain
   connectivity they fuse into one component whose "rim" would be the carton's
   top and whose centre would be the carton's. On the slab the carton's cells
   are simply absent, so the can stands alone. The basket is still taken from
   the full pass, because slicing fragments it.
2. **A retry ladder tuned to the measured envelope**: (0,0,0), (0,0,-0.010),
   (0,-0.014,0), (0,+0.014,0), (0,-0.024,0) -- steps along y, sized above the
   0.010 tolerance, re-perceiving between attempts.

Before spending a run, v2's perception was replayed offline against all 15 v0b
dumps: target (-0.1486,+0.0598) on every seed, candidate set exactly the two
cans, basket within noise of v1 -- i.e. identical behaviour on the debug layout,
so the hardening is free insurance rather than a change of method.

Selection receipt: **15/15** on the full debug split 51-65,
`results/sel_c2clean_obj_alphabet_soup_pos_k3_v2` (results.jsonl contains 15
`"benchmark_success": true`).

## Receipt chain

| version | what changed | run | result |
|---|---|---|---|
| v0  | sensor dump | fs_..._v0 (51,53,57,61) | log cap 2000 chars found |
| v0b | chunked sensor dump | fs_..._v0b (all 15) | scene + colour cue derived |
| v1  | perceive/rank/grasp | fs_..._v1 (51,53,...,65) | 8/8 |
| env | aim-offset sweep | fs_..._env (52,58) | y tol ~0.010, x tol >0.020 |
| v2  | slab pass + y ladder | sel_..._v2 (all 15) | **15/15** |

## DECLARATION

- Frozen version: **v2**. `packs/c2clean_obj_alphabet_soup_pos_k3/program.py`
  md5 `9cc35b72bc023352fa7d756f16b21016` == `program_v2.py` (same md5, verified
  on the cluster).
- Selection receipt: **15/15** on the full 15 debug seeds 51-65, dir
  `results/sel_c2clean_obj_alphabet_soup_pos_k3_v2`.
- Per-version receipt chain: above.
- PROVENANCE: present in program.py as a top-level literal dict covering all 15
  calibrated constants; every source is either a pack field or a debug-seed
  (51-65) measurement. No prior-context or LIBERO-specific constant was used --
  the table height, prop sizes, grasp height, colour cue, basket geometry and
  aim tolerance were each re-derived here and are cited to the run that
  produced them.
- `api.done` is never read anywhere in the program.
- STOP.
