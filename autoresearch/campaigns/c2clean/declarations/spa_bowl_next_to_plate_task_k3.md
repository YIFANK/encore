# c2clean / spa_bowl_next_to_plate_task_k3

Intent: **"Pick the akita black bowl next to the ramekin and place it on the plate"**
Runner: `tools/fair_run.py` only. Splits sealed (debug = 51-65).

## Scene reading (from the two packs + my own debug-seed captures)

The bddl behind this cell is the *"bowl next to the **plate**"* task, but the
intent names the bowl next to the **ramekin** — the two halves of the cell come
from the two packs:

| pack | its own language | what it gives me |
|---|---|---|
| `..._task_k3` | "pick up the black bowl next to the **plate** and place it on the plate" | the right **target** (plate), the wrong object; and it is shot in **my** scene |
| `..._task_mate` | "pick up the black bowl next to the **ramekin** and place it on the plate" | the right **object**, in a sibling scene |

Mechanism common to all 6 demos across both packs: approach from home, descend,
close the jaws at eef **z 0.919-0.930 m**, lift to **z 1.05-1.10 m**, translate,
descend to **z 0.933-0.956 m**, release. The wrist is essentially straight down
(roll ~pi, |pitch| < 0.3 rad at the close); yaw varies freely between demos, so
it is not load-bearing.

### Perception datapipe (probe1)
A motion-free program streamed cam_high/cam_arm_wrist RGB-D out through
`api.log` (zlib+base64 chunks) so the scene could be analysed offline at zero
sim steps. Receipt: `results/fs_..._probe1` (5 seeds, 0/5 by construction — it
never moves).

Measured on debug seeds 51/53/57/61/65 — table top z = **0.9006 m** on all five.
The three vessels separate cleanly by **top height above the table**:

| object | top | footprint |
|---|---|---|
| plate | 0.0193-0.0194 | 0.135-0.137 |
| ramekin | 0.0429-0.0431 | 0.085-0.088 |
| akita bowl | 0.0509-0.0512 | 0.109-0.111 |

So a 0.046 m floor isolates the bowls from everything else, and the target is
simply **the bowl nearest the ramekin** (0.128 m away vs 0.241 m for the other
bowl on ep51 — the separation is never ambiguous on any debug seed).

### Grasp mechanism
The jaws open to 0.0778 m and the bowl is 0.110 m across, so the bowl cannot be
spanned. The grasp is a **wall straddle**: with the wrist straight down the jaws
close along base **y**, so the gripper is offset **+0.045 m in y** from the bowl
centre, putting one finger inside the bowl and one outside with the wall
between them. That offset is what the mate pack shows (closing eef y
0.347/0.351/0.360 against bowl centres near y 0.31). Descent to table+0.024 is
contact-limited and actually stalls at table+0.034, which is the grasp.

While carried, the bowl centre therefore trails the eef by the same 0.045 m in
-y, so the release is commanded at `(plate_x, plate_y + 0.045)`.

## Version log

### v1 — height-band masking + wall straddle
*Hypothesis:* band-mask each vessel class by height, take the bowl nearest the
ramekin, straddle its wall at +y, release over the plate.
*Evidence:* `results/fs_..._v1` = **4/5** (51,57,61,65 ok; 53 lost).
*Verdict:* mechanism correct — all 5 grasps closed on the wall (width
0.0073-0.0078, effort 3.0). **This also settled the cell's central question:**
the benchmark bit rewards the **intent's** bowl (next to the ramekin), not the
bowl the k3 pack demonstrates. The instruction is the target; the pack is only
the mechanism.
*ep53 fault (perception, not manipulation):* the plate was read from a thin
height band, and a neighbouring bowl's **sloping wall passes through that same
band**, so the bowl fused into the plate component and dragged the plate centre
**37 mm** in +y (yc 0.2298 vs a true 0.1927). The bowl was released off the
plate.

### v2 — top-down max-z height map
*Hypothesis:* label each XY cell by the **highest** thing standing in it, so a
bowl occupies its cells at bowl height and can never also appear at plate
height; subtract bowl and ramekin neighbourhoods before reading the plate
(the bowl's *inner* wall also crosses the plate band). Also close the loop on
the two XY hover poses — api.move lands with a direction-specific residual up
to 11 mm in +x.
*Evidence:* offline replay of the 5 probe captures recovered the plate on all
five, ep53 included (0.063, 0.198 vs the ground truth 0.0633, 0.1927 read after
the bowl had been removed). Then `results/fs_..._v2` = **8/8** (seeds 51-58),
~175 sim steps per episode.
*Verdict:* adopted.

### v3 — regrasp guard threshold (= v2 apart from one constant)
*Hypothesis:* v2 carried a latent landmine. `api.gripper()`'s `effort` field is
a **gap threshold, not a force reading** — four of the eight v2 successes
carried the bowl to the plate while reporting effort 0.1 at a jaw width of
0.0044-0.0048 m. v2's regrasp guard fired below 0.004 m, i.e. **0.4 mm** under a
real hold, so a slightly thinner bite would have made the program open its jaws
in mid-air and drop a bowl it was actually carrying.
*Evidence:* across 13 successful grasps (v1 + v2) the closed width never fell
below 0.0044 m; an empty close reaches ~0. Guard moved to 0.0025 m.
*Evidence (selection):* `results/sel_..._v3` = **15/15**. *Verdict:* frozen.

## DECLARATION

**Frozen version: v3.**

- `packs/c2clean_spa_bowl_next_to_plate_task_k3/program.py`
  md5 `daa3a06cf14a72e38ce25546665aa985`
  == `program_v3.py` md5 `daa3a06cf14a72e38ce25546665aa985` ✔
- **Selection receipt (full 15 debug seeds, one formal run): 15/15**
  dir `results/sel_c2clean_spa_bowl_next_to_plate_task_k3_v3`
  (`grep -c '"benchmark_success": true' results.jsonl` = 15; seeds 51-65, each
  `success=True`; 167-202 sim steps per episode).
- **Receipt chain**
  | version | seeds | result | dir |
  |---|---|---|---|
  | probe_v1 (perception only, no motion) | 51,53,57,61,65 | 0/5 by construction | `results/fs_..._probe1` |
  | v1 | 51,53,57,61,65 | **4/5** | `results/fs_..._v1` |
  | v2 | 51-58 | **8/8** | `results/fs_..._v2` |
  | v3 | 51,53,57,61,65 + 51-58 inherited; formal 51-65 | **15/15** | `results/sel_..._v3` |
- **PROVENANCE**: present as a top-level literal dict in `program.py`, 15
  entries, every one sourced to a named pack's keyframes or to a debug-seed
  measurement. Verified by running `fair_run.scan_program(program.py, "eval")`
  directly: **EVAL GATE: PASS** (no forbidden tokens, no `api.done` read,
  PROVENANCE accepted).
- Archived: `program_v1.py`, `program_v2.py`, `program_v3.py`, `probe_v1.py`
  alongside `program.py` in the pack dir.

**No mechanism gap.** Every debug seed is solved by the same fixed sequence,
and every step is verified by the program's own sensors (jaw width after the
close and after the lift, eef read-back after each aimed move) rather than by
any runtime success signal.

STOP.
