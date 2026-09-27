# c2k1clean / obj_orange_juice_task_k1 — working notes

Intent: **"Pick the chocolate pudding and place it in the basket"**
Success = the environment's own benchmark bit (`benchmark_success` in results.jsonl).

## Pack reading (evidence, before any run)

Both packs are `pack.json` + four 128x128 keyframes, K=1.

| | `_k1` pack | `_mate` pack |
|---|---|---|
| language | pick up the **orange juice** and place it in the basket | pick up the **chocolate pudding** and place it in the basket |
| length | 126 | 168 |
| close (gripper_cmd +1) at | t=42..50, ee=(0.078,-0.106,**0.114**) | t=54..60, ee=(-0.110,-0.250,**0.011**) |
| open (release) at | t=116..125, ee=(0.002,0.224,0.168) | t=150, ee=(-0.003,0.246,0.164) |
| carried gripper_state sum | 0.0546 | 0.0465 |

Two things the packs settle straight away:

1. **Both packs release into a basket at y≈+0.23..0.25, x≈0.0.** So the "different
   target" half of the brief does not bite here — the mate pack's demo is the same
   shape of task as the intent.
2. **The named object is a flat prop.** The mate demo closes the gripper at ee
   z=0.011; the k1 demo closes on the orange juice at z=0.114. A ~10 cm difference
   in grasp altitude — the chocolate pudding sits nearly on the table, the orange
   juice is a tall carton.

`ee_path6` rotation triples have norm ≈ π about ~+x, i.e. the rotvec of a
straight-down wrist whose jaws lie along world y — the same pose `api.move`
gives for `rotation=None`. So no wrist rotation is needed anywhere.

## v0 — perception probe (seeds 51,53,55,57), no manipulation

`results/fs_c2k1clean_obj_orange_juice_task_k1_v0`

- `cam_high`: 512x512, depth 100% finite, K f=618.04 c=256, camera at base x=0.897
  looking down the −x direction; **+y is image-right, larger x is nearer the camera**.
- **table z = 0.0069**, stable to 4 decimals on every seed.
- Clusters (above-table, seed 51; identical to 3 d.p. on 53/55/57 for the three
  middle rows):

  | | xy | height | ext | mean rgb |
  |---|---|---|---|---|
  | flat box | (−0.103,−0.251) | **0.012** | (0.047,0.029) | (103,43,10) |
  | bottle | (0.109,−0.206) | 0.106 | (0.027,0.048) | (58,26,7) |
  | carton | (0.069,−0.099) | 0.132 | (0.053,0.053) | (90,64,29) |
  | **arm** | (−0.132,−0.009) | 0.394 | (0.115,0.214) | (32,42,62) |
  | bottle | (0.164,0.029) | 0.141 | (0.036,0.062) | (83,66,58) |
  | **basket** | (0.029,0.255) | 0.135 | (0.157,0.171) | (141,140,136) |

**Verdicts from v0.**

- Both packs' grasp xy land on a cluster I can see, which cross-validates the frame
  convention: the k1 demo closes at (0.078,−0.106) over the carton at (0.069,−0.099),
  the mate demo at (−0.110,−0.251) over the flat box at (−0.103,−0.251).
  ~~So the flat box at (−0.103,−0.251) is the chocolate pudding.~~
  **FALSIFIED by v3a–v3e** — that box is a decoy and the pudding is the *other* flat
  box, at (−0.138,+0.059). Reading identity off the mate pack's anchor was the single
  wrong turn in this cell; see the v3 section.
- The layout barely moves across seeds: the carton and one bottle are bit-identical
  on all four seeds, the flat box and the basket jitter by ≤11 mm. *(Not yet a
  conclusion — only 4 of 15 seeds; v1 surveys all of them.)*
- **The arm fuses with a fifth prop.** Re-projecting the cluster centroids into the
  episode gif shows five props in the scene but only four standalone clusters; a
  second small flat box at ≈(−0.137,+0.063) is swallowed by the arm cluster, which
  spans 0.214 in y. So an open-ended "above the table" mask is not a usable
  segmenter here.
- The pudding is only 88 px — it stands *behind* a tall bottle and is ~75% occluded
  from `cam_high`, so its cam_high centroid is biased toward its far edge.

## v1 — low band + mate-shaped pick and place

Two fixes carried into v1:

- **Band-mask before grouping.** The parked arm's lowest point is >0.15 above the
  table, so restricting the footprint mask to z ∈ [zt+0.010, zt+0.055] drops the arm
  entirely and unfuses the fifth prop. Column height is then recovered separately by
  taking the tallest above-table point over each footprint cell — that keeps the
  flat/tall discrimination while grouping only on the low band.
- **Wrist refine.** Because cam_high sees only a quarter of the pudding, v1 stages
  at the coarse xy and re-centroids from straight overhead with `cam_arm_wrist`
  before descending.

Motion is the mate demo's shape: descend vertically, close at zt+0.0045 (the demo's
own 0.0114 minus the measured table 0.0069), lift to zt+0.25, traverse, descend to
basket_top+0.030 (demo release 0.1635 over a basket top of 0.142), open.

Status: running on probe seeds 51,53,55,57,59,61,63,65.

## v1 — first pick and place (seeds 51..65 odd, 8 eps) — 0/8

`results/fs_..._v1`. Mechanically clean: `try0` closed with effort 3.00, the object
was carried and released over the basket. But **0/8**.

Two defects the logs exposed, neither fatal to the bit on its own:

- `_table_z` as "median of the lowest 60%" pulled in the far floor and put the table
  at 0.0012 instead of v0's 0.0069 — a 5.7 mm systematic.
- Column height (tallest point standing over a footprint) made the *second* flat box
  read 0.400 tall, because the arm happened to hover over it. Height has to be read
  from the low band's own z-spread, which the arm cannot reach.

## v2 — fixed table z, band spread, rim-band basket centre — 0/4

`results/fs_..._v2`. The important line is the post-release survey: **`slot_emptied=True`,
no flat prop loose on the table, basket footprint unchanged (2403 px before and after)**.
The object goes into the basket and stays there, and the bit is still false. So the
place is fine and the *object* is wrong.

## v2oj — same machinery aimed at the k1 pack's carton — 0/4, VOID

`results/fs_..._v2oj`. Grasped the carton (w=0.0531, vs the k1 pack's carried 0.0546 —
a match), but at release `w=0.0012 eff=0.05`: it had slipped out during the single
2.5 s traverse, and reappeared loose at (0.038,−0.029). Not a test of the hypothesis.

## v3a–v3e — one build per prop, 4 seeds each: the identity map

Transport rebuilt as four short legs with `api.grip(0.0)` re-squeezed between each.

| build | prop | held width | benchmark |
|---|---|---|---|
| v3a | flat box @(−0.138,+0.059) | **0.0458** | **4/4** |
| v3b | carton @(0.074,−0.099) | 0.0532 | 0/4 (held to release) |
| v3c | bottle @(0.111,−0.206) | 0.0266 | 0/4 |
| v3d | bottle @(0.167,+0.029) | 0.0335 | 0/4 |
| v3e | bottle @(−0.183,−0.080) | 0.0012 | 0/4 (never held) |

**The chocolate pudding is the flat box at (−0.138,+0.059), not the one the mate
pack's grasp xy points at.** The mate pack was recorded in a scene with a different
prop set, so its anchor is a decoy — but its *receipt* is not: the carried
`gripper_state` sum 0.0201+0.0264 = **0.0465** sits 0.7 mm from the right box's
measured hold width and 7.6 mm from the decoy's (0.0389). The k1 pack corroborates
the same rule on the carton (pack 0.0546 vs measured 0.0531).

**Lesson carried into v4: in a pack recorded on another layout, the transferable
identity cue is how wide the gripper ends up, not where it went.**

## v3a — formal selection on all 15 debug seeds — **14/15**

`results/sel_c2k1clean_obj_orange_juice_task_k1_v3a`. Only seed 62 fails.

Seed 62's log is clean end to end — right box (w=0.0458), held through release — but
it released at **(0.043,0.220)** where every other seed released at (0.060,0.241).
The final frame shows the box wedged against the basket's far-left inner corner
rather than sitting on its floor. The culprit is `basket_pose`: it took the rim ring
only within the *low-band* footprint cells, and the low band sees just the wall facing
the camera, so the estimate is both biased inward and noisy seed to seed.

## v4 — identity from the pack receipt, basket centre from the whole rim

- selection: rank the flat props by height, grasp the best, then **check the width
  against the mate pack** and swap to the other flat box if the receipt disagrees.
  No fixed xy anywhere, so a permuted eval layout cannot mislead it.
- `basket_pose` now gathers every point within 0.16 m of the basket that stands above
  the prop band, and averages the top 20 mm of it — a complete, symmetric ring.

Status: running on all 15 debug seeds.

## v4 — formal selection on all 15 debug seeds — **15/15**

`results/sel_c2k1clean_obj_orange_juice_task_k1_v4`. Seed 62 fixed.

The mechanism behaved exactly as designed on every seed: the height ranking put the
pudding first (`cand0`), and the pack receipt confirmed it at
**miss = 0.0007 m** — the swap path never had to fire, which is the right outcome
for a ranking that is already correct. The rebuilt `basket_pose` returns
(0.015..0.024, 0.257..0.259) across the band, against the mate pack's own release
at (−0.003,0.246); the old estimate's seed-62 outlier (0.043,0.220) is gone.
Post-release on every seed: `slot_emptied=True`, and the decoy flat box still sitting
untouched on the table.

## v5 — two robustness edits — **15/15** (FROZEN)

Neither edit changes anything on the debug band; both remove an abort that could only
hurt on a permuted eval layout.

- basket = widest footprint outright, instead of widest-above-a-threshold. The basket
  is 0.144–0.157 wide against ≤0.080 for every prop, so the two rules agree here, but
  the threshold version aborts outright if an eval frame comes in slightly small.
- if no prop falls under the flat cut, fall back to the shortest prop instead of
  aborting.

`results/sel_c2k1clean_obj_orange_juice_task_k1_v5`: **15/15**, and
`grep WARN|ABORT|no hold|receipt rejects` over all 15 logs is empty — no fallback
path fired, so the 15/15 is the same mechanism v4 ran.

---

# DECLARATION

**Frozen version: v5.** `packs/c2k1clean_obj_orange_juice_task_k1/program.py`
md5 `558abc9cbcac87f80d02a729c3c7605c` == `program_v5.py` (identical local and on
the cluster).

**Selection receipt: 15/15 on the full debug band (seeds 51–65),**
`results/sel_c2k1clean_obj_orange_juice_task_k1_v5`.

**PROVENANCE**: present as a top-level literal dict in `program.py`, 15 entries, every
one sourced to a named pack field, a debug-seed measurement, or documented controller
/ camera mechanics. No constant comes from outside this cell.

## Receipt chain

| version | seeds | result | what it established |
|---|---|---|---|
| v0 | 51,53,55,57 | perception only | frame convention, table z=0.0069, the prop table, the arm fusing with a prop |
| v1 | 8 probe seeds | 0/8 | pick and place execute cleanly; table-z and column-height defects |
| v2 | 51,53,55,57 | 0/4 | post-release survey proves the object reaches the basket — so it is the wrong object |
| v2oj | 51,53,55,57 | 0/4 VOID | carton slipped mid-traverse (w=0.0012 at release); not a test |
| v3a | 51,53,55,57 | **4/4** | the pudding is the flat box at (−0.138,+0.059) |
| v3b–v3e | 51,53,55,57 | 0/4 each | every other prop rejected; identity map complete |
| v3a | **51–65** | **14/15** | seed 62 lost to a biased, noisy basket centre |
| v4 | **51–65** | **15/15** | receipt-based identity + whole-rim basket centre |
| **v5** | **51–65** | **15/15** | **frozen** |

## The one transferable lesson

The mate pack pointed at the wrong box. Its grasp **xy** was a decoy — it was recorded
in a scene with a different prop set — while its carried **gripper width** named the
object exactly: pack 0.0465 against a measured 0.0458 for the right box and 0.0389 for
the decoy 25 cm away. The k1 pack agrees on the carton (0.0546 against 0.0531). So
when a pack comes from another layout, the identity cue that survives the move is what
the gripper *ends up doing*, not where it went — and it is cheap to check at runtime,
because the width is readable the moment the object leaves the table.
