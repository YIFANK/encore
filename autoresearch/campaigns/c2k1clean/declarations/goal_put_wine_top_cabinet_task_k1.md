# c2k1clean / goal_put_wine_top_cabinet_task_k1

Intent: **"put the wine bottle in the bowl"** (re-authored; success = the
environment's own benchmark bit).

## Pack inventory (deviation from the brief)

The brief names two pack directories. Only one exists on the cluster:

- `packs/c2k1clean_goal_put_wine_top_cabinet_task_k1/` — present, K=1, language
  `"put the wine bottle on top of the cabinet"`.
- `packs/c2k1clean_goal_put_wine_top_cabinet_task_mate/` — **does not exist**
  (`ls` on AbakaAI: "No such file or directory"). No mate evidence was used.

The one pack is also mate-shaped rather than k1-shaped relative to the brief's
description: its demos handle the OBJECT my intent names (the wine bottle)
toward a DIFFERENT target (the cabinet top). It is therefore evidence for the
grasp half of the intent only; the bowl had to come entirely from my own
debug-seed perception.

Pack facts used: demo0, length 93, stride 10. `gripper_cmd` flips -1 -> +1 at
t=32 with `ee = [-0.1823, -0.0652, 1.0291, ...]`; `gripper_state` goes to
~0.0165 total width by t=85, i.e. the demo grasps the bottle **neck**, not the
body. That single number (close height 1.0291) is the only quantity carried out
of the pack.

## v1 — perception probe (no motion)

Hypothesis: nothing; establish the scene from my own sensors.
Program: capture `cam_high` + `cam_arm_wrist`, zlib+base64 the RGB and depth
through `api.log`, do no motion at all. Perception then done offline on the Mac.

Receipt: `results/fs_..._v1`, seeds 51,53,55,57, 0/4 (expected — no motion),
72 sim steps each.

Evidence (all four seeds agree):

- Table plane z = **0.901** (mode of the cam_high depth histogram over the
  workspace).
- `cam_high` K = 618.04 / (256,256); t_base_cam has base-y == camera-x, so the
  view is oblique in x only and **y spans are unbiased while x spans are not**.
- Five props resolve as clusters above the table (height above table, footprint):
  | cluster | h | dx x dy | id |
  |---|---|---|---|
  | dark, 1.27k px | 0.158 | 0.028 x 0.043 | **wine bottle** |
  | grey, 6.9k px | 0.031 | 0.187 x 0.190 | stove slab |
  | grey, 2.15k px | 0.051 | 0.110 x 0.110 | **bowl** |
  | blue, 0.97k px | 0.019 | 0.079 x 0.042 | small blue box |
  | pale, 0.7k px | 0.019 | 0.136 x 0.137 | plate |
- Bottle profile by height: body diameter 0.043 up to z~0.99, **neck diameter
  0.015** from z 1.015 up to the top at 1.059. The pack's close height 1.029 is
  30 mm below the bottle top => a neck grasp, confirmed.
- Bowl: outer diameter 0.110, rim top 0.952 (table+0.051), interior floor
  z=0.908 (table+0.007) => a 44 mm deep well.
- The robot arm's lowest visible point is z=1.165 (table+0.264), so capping the
  cluster mask at table+0.22 removes the arm without touching a prop. The
  cabinet/rack fixture is one >9000 px cluster and is removed by a size cap.
- **x-bias correction:** the bottle's bbox mid is 7 mm nearer the camera than
  its true axis (only the near side of a cylinder is visible). Taking the bbox
  mid of the **top 12 mm** of the cluster instead — a full disc seen from above
  — is unbiased and agrees with the radius-corrected axis to <1 mm. The neck is
  only 15 mm wide, so this correction is load-bearing.

Verdict: bowl and bottle are each uniquely selected by a height+footprint gate
on all four probe seeds. Proceed to a real attempt.

## v2 — neck-grasp, lower into the bowl, release

Hypothesis: the demo's grasp height transfers verbatim (table+0.128 = the
neck), and a 43 mm bottle base lowered into a 44 mm deep, 110 mm wide bowl will
seat and stay upright.

Program: re-perceive every episode (table plane from the depth histogram,
clusters gated by height + footprint + roundness); open; hover at table+0.26;
descend to table+0.128 at the bottle's top-band axis; close; lift; translate
over the bowl axis; lower to bowl-floor + 0.005 + 0.128; open; retreat; settle;
stream the final frame back.

Probe receipt: `results/fs_..._v2`, seeds 51,53,55,57 — **4/4**.

Evidence from ep51: closed gripper width **0.0149** (== the measured 15 mm neck)
with effort 3.0, and effort 3.0 still held after the lift and at the place
pose — the grasp is a real neck bite, not a jam. The streamed final frames show
the bottle **standing upright, seated in the bowl** on every probe seed.

Verdict: mechanism confirmed. Selected for the formal 15-seed run without
change.

Selection receipt: `results/sel_..._v2`, seeds 51-65 — **11/15**.
Losses: 54, 61, 63, 65 — all four returned `"no target"`, i.e. the program
aborted in perception and never moved. Their logs show one cluster of
n~3400 px (== 1270 bottle + 2150 bowl) with h=0.157 and a 0.15-0.17 footprint:
when the bottle and the bowl **abut**, the single-band clustering fuses them and
the fused blob is too wide for the bottle gate and too tall for the bowl gate.
Bottle-bowl centre separation is 0.108-0.112 on exactly those four seeds and
0.118-0.144 on the eleven that passed.

Verdict: perception failure, not mechanism failure. The mechanism is 11/11
wherever it was allowed to run.

## v3 — two disjoint height bands (FROZEN)

Hypothesis: the bottle and the bowl can never fuse if they are found in bands
that do not overlap. The bottle reaches table+0.158 and every other prop stops
at table+0.059, so a band from table+0.10 up contains the bottle and nothing
else that is narrow; the bowl can then be found below it once the bottle's
footprint is punched out of the frame.

Program: band 1 = z in (table+0.10, table+0.22), components narrower than 0.08
-> the bottle neck; axis = bbox mid of its top 12 mm. Band 2 = z in
(table+0.036, table+0.080) with a 0.032 m exclusion disc about the bottle axis;
each component's top-6 mm rim ring gets a Kasa circle fit and the bowl is the
candidate whose radius matches 0.0537 +/- 0.012. Motion is unchanged from v2.

Offline validation against the streamed RGB-D of **all 15 debug seeds**:
perception 15/15, fitted bowl radius 0.0536-0.0537 on every seed, the only
other fitted candidates being the bottle shoulder (0.017) and the cabinet
(0.078-0.091) — a wide margin on both sides.

Probe receipt: `results/fs_..._v3`, seeds 51,53,54,57,61,63,64,65 (the four
v2 losses included) — **8/8**.

Selection receipt: `results/sel_..._v3`, seeds 51-65 — **15/15**.

## Receipt chain

| version | mechanism | run dir | seeds | score |
|---|---|---|---|---|
| v1 | perception probe, no motion | `results/fs_..._v1` | 51,53,55,57 | 0/4 (expected) |
| v1 (re-run as dump) | perception probe, no motion | `results/fs_..._dump` | 52,54,56,58-65 | 0/11 (expected) |
| v2 | one-band perceive + neck-grasp into bowl | `results/fs_..._v2` | 51,53,55,57 | **4/4** |
| v2 | (formal selection) | `results/sel_..._v2` | 51-65 | **11/15** |
| v3 | two-band perceive + neck-grasp into bowl | `results/fs_..._v3` | 51,53,54,57,61,63,64,65 | **8/8** |
| v3 | (formal selection) | `results/sel_..._v3` | 51-65 | **15/15** |

---

# DECLARATION

- **Frozen version: v3.**
  `packs/c2k1clean_goal_put_wine_top_cabinet_task_k1/program.py`
  md5 `864ca5de263bb86f77670388542dc113` ==
  `packs/c2k1clean_goal_put_wine_top_cabinet_task_k1/program_v3.py`
  md5 `864ca5de263bb86f77670388542dc113`.
- **Full-15-seed selection receipt: 15/15**, seeds 51-65, run dir
  `results/sel_c2k1clean_goal_put_wine_top_cabinet_task_k1_v3`.
- **Per-version receipt chain:** see the table above (v1 0/4 + 0/11 probes,
  v2 4/4 probe -> 11/15 selection, v3 8/8 probe -> 15/15 selection).
- **PROVENANCE present:** yes — nine entries (GRASP_DZ, TABLE_FALLBACK,
  HI_BAND, HI_MAXDIM, LO_BAND, BOTTLE_PUNCH_R, BOWL_R, BOWL_FLOOR_DZ,
  CARRY_DZ), each sourced to either the named pack's `pack.json` or a debug-seed
  (51-65) measurement, all `allowed: True`. Verified with
  `fair_run.scan_program(program.py, "eval")` -> PASS (no `api.done` read,
  no forbidden tokens, PROVENANCE accepted).
- **Protocol:** every run used `tools/fair_run.py --split debug` on seeds 51-65
  only. `tools/fewshot_run.py` was never invoked. Seeds 1-50 were never
  touched. Cluster writes were confined to
  `packs/c2k1clean_goal_put_wine_top_cabinet_task_k1/*` and
  `results/*c2k1clean_goal_put_wine_top_cabinet_task_k1*`. No forbidden asset
  was read; the `_mate` pack named in the brief does not exist on the cluster
  and no other campaign's artifacts were consulted.

STOP.
