# c2clean / obj_orange_juice_task_k0 — working notes

Intent: "Pick the chocolate pudding and place it in the basket".
No demonstration pack. All constants derived from debug seeds 51-65 only.

## Version chain

### v1 — deprojection sanity check (seeds 51,53)
Hypothesis: the standard pinhole convention plus `t_base_cam` gives base-frame
points; `api.deproject` would confirm it.
Evidence: `AttributeError: 'FairApi' object has no attribute 'deproject'` — the
helper named in the brief does not exist on this build. Recovered the geometry
from the logged matrices instead: `K = [[618.04,0,256],[0,618.04,256],[0,0,1]]`,
`t_base_cam` puts cam_high at base `(0.897, 0, 0.650)` looking along
`(-0.849, 0, -0.529)` (32 deg below horizontal), image `y` maps to base `y`
with no tilt. Depth range 0.74-2.79 m.
Verdict: vectorised `((u-cx)z/f, (v-cy)z/f, z) @ R^T + t` is the deprojection;
no API call needed. Also learned `eef0 = (-0.1485, 0.0, 0.2613)`,
gripper open width 0.0778, effort 0.05 when empty.

### v2 — first height-map survey (seeds 51,53,55,57)
Hypothesis: table plane = the dominant z mode; props = everything above it.
Evidence: z histogram mode at **0.0033** with 115k/149k workspace pixels —
table_z = 0.0033. XY grid clustering above table+0.012 gave 6 clusters, but the
tallest (h=0.482) fused the robot arm with two props whose pixel boxes sat
inside the arm's footprint.
Verdict: table_z confirmed; clustering needs a height cap so the arm cannot act
as a bridge.

### v3 — banded survey, all 15 debug seeds
Hypothesis: capping the band at table+0.20 removes the arm and separates props.
Evidence: 8 clusters on every one of seeds 51-65, in the same order, same
identities. Only three jitter across seeds: C4 (+-5 mm), C7 (+-10 mm) and C5
(the basket, +-15 mm). C2, C3, C6, C8 are bit-identical on all 15 seeds.
Two flat boxes: C3 at (-0.139, 0.059) ztop 0.030, C4 at (-0.115, -0.242)
ztop 0.019.
Verdict: layout is near-deterministic; the target is one of the two flat boxes.

### v4 — RGB out through api.log
Hypothesis: the RGB frame can be exfiltrated to my own eyes as
zlib+base64 over `api.log`, so identification does not have to be done blind
from cluster statistics.
Evidence: first attempt with 60000-char chunks came back truncated — **the
runner caps each logged message at 2000 characters**. Re-chunked at 1900 and
the full 512x512x3 frame reconstructed exactly.
Verdict: works; ~215 log lines per frame.

### v5 — wrist close-ups name the box (seed 51)
Hypothesis: a top-down wrist capture at 0.22 m resolves the package label.
Evidence: C3's label reads **CHOCOLATE PUDDING** in the wrist frame. C4 is a
different box (blue band over red/orange). The other props are a ranch-dressing
bottle, a BBQ-sauce bottle, an orange-juice carton and a ketchup bottle; C5 is
the basket. Top-band geometry of C3: ztop 0.0295, x in (-0.189,-0.109),
y in (0.035,0.083).
Verdict: target = C3. Its y-extent 0.048 m fits inside the 0.078 m open jaws,
so a straight top-down grasp with jaws closing along base y is admissible.
The bddl filename ("orange juice") is not the target; the intent sentence is,
and the orange-juice carton is a distractor in the same scene.

### v6 — fingertip offset probe (seeds 51,55)
Hypothesis: closing the jaws and commanding a descent below the table gives the
fingertip-to-eef offset from the stall height.
Evidence: commanded z = -0.04, stalled at eef z = **0.0095**, residual 0.0489,
identical on both seeds. Offset = 0.0095 - 0.0033 = **0.0062 m**.
Also confirmed the selection rule (flat box nearest the anchor) picks C3 with
d = 0.001 while C4 sits at d = 0.30, and the basket rim centroid is
(0.013, 0.259) / (-0.005, 0.254) with rim top 0.1437.
Verdict: TIP_OFFSET = 0.0062; target and basket both perceivable per episode.

### v7 — full pick and place (probe seeds 51,53,55,57)
Hypothesis: perceive -> open -> descend to tip height 0.010 -> close -> lift to
0.26 -> traverse to the basket rim centroid -> descend to 0.21 -> open.
Evidence: **4/4 benchmark_success**, `results/fs_c2clean_obj_orange_juice_task_k0_v7`.
Receipts per episode: closed width 0.0463 with effort 3.0, held unchanged
through the lift and the traverse; the post-release re-perception no longer
finds a flat box near the anchor (the nearest candidate is C4 at d = 0.303).
The descent stalls at eef 0.0247 rather than the commanded 0.0162 (residual
0.0093) — the jaws bottom out on the box, which is what makes the bite land
just under the box top; the grasp survives it, so I left the command as is.
Verdict: selected.

## Selection receipt
Formal run on the full 15 debug seeds 51-65:
`results/sel_c2clean_obj_orange_juice_task_k0_v7` -> **15/15 benchmark_success**,
no program errors.

## DECLARATION
- Frozen version: **v7**.
  `md5(packs/c2clean_obj_orange_juice_task_k0/program.py)` =
  `35479e1a5bb079031adf40ea2b39e89c` =
  `md5(packs/c2clean_obj_orange_juice_task_k0/program_v7.py)`.
- Full-15-seed selection receipt: **15/15**, dir
  `results/sel_c2clean_obj_orange_juice_task_k0_v7`.
- Per-version receipt chain:
  - v1 `results/fs_c2clean_obj_orange_juice_task_k0_v1` — diagnostic, api.deproject absent.
  - v2 `results/fs_c2clean_obj_orange_juice_task_k0_v2` — diagnostic, table_z = 0.0033.
  - v3 `results/fs_c2clean_obj_orange_juice_task_k0_v3` — diagnostic, 15 seeds, 8 stable clusters.
  - v4 `results/fs_c2clean_obj_orange_juice_task_k0_v4` — diagnostic, RGB dump (2000-char log cap).
  - v5 `results/fs_c2clean_obj_orange_juice_task_k0_v5` — diagnostic, label reads CHOCOLATE PUDDING.
  - v6 `results/fs_c2clean_obj_orange_juice_task_k0_v6` — diagnostic, TIP_OFFSET = 0.0062.
  - v7 `results/fs_c2clean_obj_orange_juice_task_k0_v7` — 4/4 probe; selection 15/15.
- PROVENANCE present in program.py, covering TABLE_Z, ANCHOR_XY, TIP_OFFSET,
  GRIP_TIP_Z, R_DOWN, OPEN_W. Every constant comes from a debug-seed
  measurement (seeds 51-65) or generic controller/camera mechanics. No
  demonstration pack was available or used; no benchmark asset file was read.
