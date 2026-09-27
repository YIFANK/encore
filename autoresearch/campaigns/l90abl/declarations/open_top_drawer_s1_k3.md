# l90abl / open_top_drawer_s1_k3 — working notes

Intent: "open the top drawer of the cabinet" (KITCHEN_SCENE1, LIBERO-90).
Runner: `tools/fair_run.py` only. Pack: `packs/l90abl_open_top_drawer_s1_k3/`
(K=3 demos, keyframes + ee_path6 + raw actions).

## What the pack says

Every keyframe of every demo carries `gripper_cmd = -1.0` — the gripper is
**never closed**. The wrist stays straight down (rpy ≈ (π, 0, ~0)). Each demo
is the same three-phase move:

| demo | contact eef (x, y, z) | final y | +y travel |
|---|---|---|---|
| 0 | (+0.0154, −0.1679, 1.1043) | +0.0575 | 0.225 |
| 1 | (−0.0157, −0.1818, 1.1067) | −0.0004 | 0.177 |
| 2 | (+0.0016, −0.1611, 1.1046) | +0.0140 | 0.177 |

So: descend to z ≈ 1.105 at y ≈ −0.17, then translate +y ≈ 0.18–0.23 with x
and z held. The drawer opens toward +y.

## Scene (perceived; identical on all 15 debug seeds)

Cabinet near face y = −0.2307, top z = 1.131, x ∈ (−0.15, +0.12), table
z = 0.901. Three handle bars, each protruding 32 mm to y ≈ −0.1995, on two end
posts:

| bar | z band | x span |
|---|---|---|
| bottom | 0.940–0.955 | |
| middle | 1.005–1.020 | |
| **top** | **1.083–1.098** | **−0.050 … +0.045** |

Fingertips sit **0.0064 m** below the eef (v6: open jaws pressed on the cabinet
top, a known plane at 1.131, stalled the eef at 1.1374). Open jaws are ±0.040 m
from the eef in y (tool rotation column 1 = (0,−1,0)).

Episode horizon is 1000 sim steps (v5 seeds 55/59 capped there exactly).

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | replay the pack pose open-loop | **0/8** (probe). Gripper width pinned at 0.0799 on 5 seeds — swept through free space; 3 seeds blocked high (eef 1.1368, res 0.033) | rejected; and the scene is not where I assumed |
| v2 | perception only, 0 sim steps, all 15 seeds | cabinet identical on every seed (top 1.131, face −0.23) — the failure is not seed variation | scene is deterministic |
| v3 | fine (5 mm × 1 cm) max-y map + tabletop tip probe | found the three bars and their x spans; tip probe **failed** — target (0.32, 0.28) was out of reach and never converged laterally | geometry good, drop unknown |
| v4 | measure drop, then hook the slot behind the bar | **0/8**. `face_y` taken as a 60th percentile over a point set running 17 cm back along the cabinet top read −0.2462, every band counted as protruding, the bars merged into one group, and the far jaw was driven 7 mm *into* the face and wedged | perception bug |
| v5 | face = median of per-band max-y | **3/8** — but all three were **accidents**: those episodes terminated at 150–192 steps during the tabletop tip probe and every later `api.eef()` returned the frozen pose. Where the hook actually ran it jammed at eef 1.141 | not a real 3/8 |
| v6 | measure drop on the cabinet top; sweep descent depth | drop = **0.0064**. Jaw at −0.211/−0.197 stalls at tip 1.099 (bar top); jaw at −0.227/−0.236 stalls at tip 1.130 (cabinet top). Rungs drifted up to 8 mm off their commanded y | the 15 mm slot was straddled, never entered |
| v7 | close the loop on placement before the descent | **0/8**. Placement now lands within 2 mm, but the **descent** drifts 15–20 mm in −y (pure-z command; the OSC null space carries the wrist back). Every rung stalls at tip 1.099 and the pull then runs free (res 0.007–0.011, no load) | descent, not placement, is the leak |
| v8 | correct x/y at every 8 mm of descent | **0/4**. 8 mm slices are *under* `api.move`'s 12 mm stop band, so the first slice of every rung arrived already "converged", moved nothing, and tripped the stall test | slice must exceed POS_TOL |
| v9 | same, 20 mm slices | **0/4**, but clean data: y now held to 3 mm all the way down and it *still* stalls at tip 1.0997 at jaw −0.2229 and −0.2200, while jaw −0.2251 stalls at 1.1305 | **the slot is not enterable** — bar body runs back to ≈ −0.215, counter lip reaches ≈ −0.225, and what is between is thinner than the finger |
| **v10** | **the pack is not hooking — it presses the open fingertip on the bar top and drags** | **4/4 probe, 15/15 selection** | **frozen** |

## Why v10 works

v9's stall pose is *the pack's contact pose*: v9 stalls at eef
(0.003, −0.180, 1.1059), fingertip 1.0995; the pack's contact keyframes sit at
eef y −0.163/−0.170/−0.177, z 1.1045/1.1046/1.1068, fingertip 1.098–1.100.
The pack rests the open fingertip on top of the bar and drags the drawer out by
friction.

v7 reproduced that pose and still failed for one reason: it commanded the pull
at the height the descent had *reached*, so the wrist floated 3 mm up and let
go. v10 keeps commanding a z **below** the contact (22 mm, comfortably past the
12 mm stop band) for the whole pull, so the controller holds the finger loaded
on the bar while translating +y. The very first rung — press 0.022 at the
pack's own jaw depth dy = 0.014 — succeeds on every seed; the remaining rungs
never run, because LIBERO ends the episode when the predicate fires.

Cost: 409–412 of the 1000 available sim steps.

## Candidate law (for LAWS.md)

A protruding drawer handle is not always hookable. Check the slot behind it
against the finger, not against the perceived protrusion: a 32 mm standoff can
leave under 15 mm of clear depth once the bar body and the counter lip are
subtracted, and a guarded descent that holds y to 3 mm will still stall on the
bar top. When the slot is too thin, press the open fingertip on the bar's top
surface and drag — and command the pull at a z *below* the contact, because a
pull commanded at the height the descent reached floats off within the
controller's own stop band.

## DECLARATION

- **Frozen version: v10.** `packs/l90abl_open_top_drawer_s1_k3/program.py`
  md5 `e04be043876043636f68fa9ff828596f` ==
  `packs/l90abl_open_top_drawer_s1_k3/program_v10.py` md5
  `e04be043876043636f68fa9ff828596f`.
- **Selection receipt: 15/15** on the full 15 debug seeds (51–65),
  `results/sel_l90abl_open_top_drawer_s1_k3_v10` — every episode
  `"benchmark_success": true`, 409–412 sim steps each.
- **Per-version receipt chain** (all `results/fs_l90abl_open_top_drawer_s1_k3_v*`):
  v1 0/8 · v2 0/15 (perception only) · v3 0/4 (probe) · v4 0/8 · v5 3/8
  (all three spurious, see above) · v6 0/4 (probe) · v7 0/8 · v8 0/4 · v9 0/4 ·
  **v10 4/4 probe → 15/15 selection**.
- **PROVENANCE**: present in `program.py` as a top-level literal dict covering
  every calibrated constant (CAB_TOP, CAB_Y/MID_X, PROTRUDE_MIN, JAW_HALF,
  TIP_DROP, JAW_DEPTHS, PRESS, PULL_LEN/PULL_STEP, PLACE_TOL/STEP), sourced
  only from pack.json fields and debug-seed measurements.
- Every formally probed version archived as `program_vN.py` in the pack dir.
- Eval seeds 1–50 never touched; `--split eval` never invoked.
