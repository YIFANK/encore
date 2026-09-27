# c2clean / spa_bowl_between_task_k3 — worker notes

Intent: **"Pick the akita black bowl NOT between the plate and the ramekin and
place it on the plate"**.

## Pack reading (the pack is an ANTI-pack)

`pack.json` language is *"pick up the black bowl **between** the plate and the
ramekin and place it on the plate"* — i.e. K=3 demos of the **excluded** bowl.
So the pack supplies the *mechanism* (grasp height, carry altitude, release
height, the rim-offset carry model) and the *instruction* supplies the target.

Pack keyframes per demo: `t=0` home, `t≈36` close command at the grasp,
`t≈73-85` open command at the place, `t≈83-97` retreat.

Mechanism read off `ee_path6`:

| quantity | demo0 | demo1 | demo2 |
|---|---|---|---|
| grasp z (lowest of descent, t=40) | 0.9301 | 0.9208 | 0.9187 |
| carry apex z | 1.0373 (t60) | 1.0533 (t60) | 1.0193 (t70) |
| release z (kf, gripper_cmd→-1) | 0.9367 | 0.9360 | 0.9269 |
| closed gripper gap | 0.0125 | 0.0122 | 0.0130 |
| grasp xy | (-0.051, 0.178) | (-0.088, 0.141) | (-0.071, 0.145) |
| place xy | (0.058, 0.155) | (0.063, 0.135) | (0.045, 0.174) |

Closed gap ≈ 12.8 mm ⇒ a **pinched bowl wall**, not a body grasp. Locating the
"between" bowl in the demo `t0000` keyframes (ray→table-plane through the
camera pose returned by `capture`) puts it at ≈(-0.05, 0.19), so the demo grasp
sits ≈0.05 m in **-y** from the bowl centre ≈ one rim radius. `tool_rotation()`
at home is ≈diag(1,-1,-1), so with `rotation=None` the jaws close along base
**y** — consistent with a -y rim straddle. Place xy + one rim radius in +y
lands on the plate centre, so **the carried bowl centre trails the eef by the
full rim radius**.

## v1 — perception only (5 seeds)

`api.log` truncates a message at 2000 chars, so RGB-D is shipped as 1800-char
base64 chunks of zlib'd arrays (RGB at 256², depth as uint16 mm at 256²).

Scene (essentially fixed across debug seeds, ±2 cm jitter):

| prop | centre (x,y) seed 51 | ztop |
|---|---|---|
| cabinet + stove | (0.03, -0.22) | 1.128 |
| robot column | (-0.18, 0.00) | 1.357 |
| bowl A (**between**) | (-0.067, 0.204) | 0.951 |
| bowl B (**target**) | (-0.183, 0.320) | 0.951 |
| ramekin | (-0.208, 0.193) | 0.944 |
| cookie box | (0.078, 0.031) | 0.921 |
| plate | (0.060, 0.227) | 0.920 |

Table top z = 0.900 (modal, 30k px). Bowl A sits essentially **on** the
plate–ramekin segment (d ≈ 0.005); bowl B is 0.14 off it. Huge margin, so
"between" is decided by distance to that segment.

Image-space connected components **fuse** the ramekin with bowl A on seeds
57/61/65 (occlusion bridges them); they are 0.14 m apart in base XY, so v2
labels props on a **top-down base-XY occupancy grid** instead.

Prop classification: cookie box by chroma (r-b = 0.18 vs <0.06 for everything
else), plate = lowest ztop, ramekin = narrowest vessel (sy 0.088 vs 0.110).

## v2 — first full attempt

Hypothesis: perceive → target = bowl farther from the plate–ramekin segment →
rim-straddle at (cx, cy - rim_r, 0.923) → lift to 1.030 → carry to
(plate_x, plate_y - rim_r) → release at 0.938.

Evidence: **0/8** (`fs_c2clean_spa_bowl_between_task_k3_v2`). Two perception
bugs. (a) The band was capped at 1.10 m, which *clipped* the cabinet instead of
excluding it, splitting off a fragment at (-0.24,-0.12) with ztop 0.932 that
then out-voted the real plate. (b) Bowl A fused with the plate in the 0.915 m
band (centres 0.13 m apart, radii 0.055+0.068), giving a bogus sy=0.137.
Consequence: the target test used a fake plate and picked the wrong bowl; the
gripper closed to width 0.001 / effort 0.05 (air) on every seed.

## v3 — height-banded perception + closed-loop moves

* vessels from a HIGH band (z > 0.932, above both flat props) — clean on all 8;
* plate from a LOW band (0.912–0.932) with vessel footprints punched out;
* `goto()` re-issues `api.move` with the observed error folded back in, because
  `api.move` settles 8–18 mm short (cmd z 0.923 → eef 0.9313).

Evidence: **0/8** (`..._v3`). But the *mechanism* was confirmed for the first
time: `GRIP closed={'width_m': 0.0103, 'effort': 3.0}` — a held wall pinch,
matching the demos' 0.0128 m gap. Residual bug: the cabinet fragment still
looked like a flat prop (the low band clips tall structures too), still won the
plate vote, so the carry aimed at (-0.24,-0.17) — *over the cabinet*, whose top
is 1.128 m vs CARRY_Z 1.035 — and the bowl was knocked out of the jaws
(effort 3.0 → 0.05, width 0.0092 → 0.0038 mid-carry).

## v4 — kill the fixture fragments; bite retry; shortest hold

1. **Tall mask**: any XY cell carrying a point above MAX_PROP_Z (0.99) is
   punched out of the low band, so a clipped fixture cannot masquerade as a
   flat prop.
2. **Plate by value**: plate mean value 0.55–0.58, every cabinet fragment
   0.22–0.28 → `PLATE_MIN_VALUE = 0.42` (the cookie box is already excluded by
   r−b = 0.18).
3. **Side selection**: the outer finger stands 0.039 m off the tool axis; on
   the −y side of the target bowl it clears the ramekin by **0.001 m**, on +y
   by 0.175 m. The program now straddles whichever rim side has room, and
   mirrors the carry offset to match.
4. **Bite retry** (rim, +6 mm, −6 mm) judged by `effort >= 1.0 and width_m >
   0.003`; move retries trimmed while holding (the jaws keep squeezing).

Evidence: **7/8** (`fs_c2clean_spa_bowl_between_task_k3_v4`, seeds
51,53,55,57,59,61,63,65 — all ok but ep65). Receipt from ep51:
`TARGET c=(-0.172,0.315) d=0.119 (excluded bowl d=0.004)`,
`SIDE sgn=-1 clearance=0.001 / sgn=+1 clearance=0.175` → `SIDE=+1`,
`GRIP bite0 -> {'width_m': 0.0106, 'effort': 3.0}`, held through lift
(0.0101), carry (0.0095) and place (0.0091).

ep65 **crashed**: the ramekin and bowl A abut with an ~11 mm gap on that seed,
closer than the 6 mm footprint grid can resolve, so the 0.932 band returned
only **two** vessel components — the fused pair was taken as a bowl and the
real target as the ramekin, then `scored[1]` raised IndexError and the episode
ended at the perception stage.

## v5 — split the fused pair by height

The ramekin tops out at 0.943–0.944 on **every** debug seed and both bowls at
0.951–0.952, so a 0.9465 band contains the bowls and nothing else regardless of
how closely they abut in XY. The ramekin is then whatever survives in the
0.932–0.9465 band once the bowl footprints are punched out, with the v4
narrowest-vessel rule kept as a fallback. Every indexing path is now guarded.

Evidence: **14/15** — formal selection run
`results/sel_c2clean_spa_bowl_between_task_k3_v5` (seeds 51-65). Only **ep56**
fails.

ep56 is a genuinely different layout (plate at (0.128,0.171) rather than
~(0.07,0.21)) in which the **two bowls abut each other**: the 0.9465 band
returned a single component `c=(-0.173,0.308) s=(0.184,0.190)`, so the fused
pair was taken as one bowl, there was no second bowl to compare against
(`excluded d=-1.000`), and the grasp closed on the pair at width 0.025 —
a body grab, not a wall pinch. The debug layout therefore is **not** fixed;
seeds 51-55 merely look alike.

## v6 — circle vote for the two rim rings

Both bowls are the same asset: on every cleanly separated debug footprint they
span 0.109-0.112 m in x and in y, so the rim radius is a measured constant
(BOWL_R = 0.0553) rather than something to read off a possibly-fused bbox. A
Hough vote over the 0.9465-band points for two rings of that radius recovers
both centres whether or not the footprints touch. The vote is restricted to the
table-prop components first — the cabinet and the robot column also reach into
that band and otherwise win the poll outright (offline check on seeds
51/53/57/61/65: unrestricted peaks land on the stove at (0.10,-0.28) and the
robot at (-0.21,-0.005)).

Offline agreement on the five clean seeds, vote vs. the v4/v5 wide-band
centres that grasped successfully: seed 51 (-0.051,0.200) vs (-0.053,0.200)
and (-0.171,0.312) vs (-0.172,0.315) — within 3 mm on every seed.

Evidence: **15/15** — formal selection run
`results/sel_c2clean_spa_bowl_between_task_k3_v6` (seeds 51-65, all
`ep*_ok.gif`). ep56 receipt:
`VOTE [(257,-0.135,0.268),(241,-0.207,0.344)]` → `BOWLS from circle vote`,
`TARGET c=(-0.207,0.344) d=0.130 (excluded d=0.063)`,
`SIDE sgn=-1 clearance=-0.004 / sgn=+1 clearance=0.130` → `SIDE=+1`,
`GRIP bite0 -> {'width_m': 0.0111, 'effort': 3.0}` — a wall pinch where v5 had
closed on the pair at 0.025.

---

# DECLARATION

**Frozen version: v6.**
`packs/c2clean_spa_bowl_between_task_k3/program.py` md5
`707c076ee2f2422f87024effcebf0f7f` == `program_v6.py` == the
`program_archived.py` of the selection run below.

**Selection receipt: 15/15** on the full 15 debug seeds (51-65),
`results/sel_c2clean_spa_bowl_between_task_k3_v6` — counted as
`"benchmark_success": true` in `results.jsonl`.

**Receipt chain** (every formally probed version archived as `program_vN.py`
in the pack dir):

| version | seeds | result | dir |
|---|---|---|---|
| v1 | 51,53,57,61,65 | perception dump only | `fs_..._v1` |
| v2 | 8 probe | 0/8 | `fs_..._v2` |
| v3 | 8 probe | 0/8 (grasp mechanism confirmed) | `fs_..._v3` |
| v4 | 8 probe | 7/8 | `fs_..._v4` |
| v5 | all 15 | 14/15 | `sel_..._v5` |
| **v6** | **all 15** | **15/15** | **`sel_..._v6`** |

**PROVENANCE**: present as a top-level literal dict in `program.py`, one entry
per calibrated constant (TABLE_Z, LOW/HIGH/BOWL band cuts, MAX_PROP_Z,
TALL_CELL, BOWL_R/BOWL_MAX_W, PLATE_MIN_VALUE, BOX_CHROMA, RAMEKIN_WIDTH,
GRASP_Z, CARRY_Z, RELEASE_Z, RIM_OFFSET, JAW_AXIS, OPEN_HALF, MOVE_BIAS_TOL,
HOLD_EFFORT), each sourced to either a `pack.json` field or a debug-seed
(51-65) measurement. No `api.done` read anywhere in the program.

**Eval never touched**: seeds 1-50 were never run; every run above used
`--split debug`.
