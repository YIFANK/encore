# c2clean spa_bowl_next_to_plate_pos_k0 — worker notes

Intent: "Pick up the black bowl next to the plate and place it on the plate."
K=0: no demonstration pack. Every constant re-derived from debug seeds 51-65.

## v1 — observation only (seeds 51,53), 0/2 (expected; no motion)

Receipt: `results/fs_c2clean_spa_bowl_next_to_plate_pos_k0_v1` 0/2.

Runtime surface (from the log, not from any prior):
- start EEF `[-0.2085, 0.0, 1.1733]`, gripper open `width_m=0.0778`, `effort=0.05`.
- start tool rotation ~3.3 deg off straight-down:
  `[[0.9984,0.0005,-0.0568],[0.0005,-1,0],[-0.0568,0,-0.9984]]`.
  Tool x -> world +x, tool y -> world -y, tool z -> world -z (approach).
  So with a straight-down wrist the **jaws separate along world y**.
- `cam_high`: K fx=fy=618.04, c=(256,256); t_base_cam puts it at
  (0.659, 0.0, 1.610) looking down/back along -x. Full 512x512, depth valid 100%.
- `cam_arm_wrist`: fx=fy=333.63, mounted ~(-0.153, 0, 1.267) in base frame.

Scene (offline reconstruction of the dumped RGB-D, seeds 51 and 53 agree):
- **table top z = 0.9018** (modal z over the workspace crop).
- 7 components above table+12mm inside x[-0.35,0.35] y[-0.45,0.45]:

| comp | what | centre (x,y) | footprint | height above table |
|---|---|---|---|---|
| c1 | robot arm | (-0.21, 0.00) | — | 0.469 |
| c2 | cabinet + stove slab | (-0.08, -0.20) | 0.52 x 0.32 | 0.226 |
| c3 | small light steel bowl | (-0.207, 0.193) | 0.085 x 0.087 | 0.042 |
| c4 | dark speckled bowl | (-0.172, 0.315) | 0.109 x 0.109 | 0.050 |
| c5 | dark speckled bowl | (0.006, 0.310) | 0.110 x 0.111 | 0.050 |
| c6 | plate | (0.072, 0.044) | 0.134 x 0.133 | 0.018 |
| c7 | cookie box | (0.069, 0.201) | 0.081 x 0.061 | 0.019 |

Target selection: c4 and c5 are a matched pair (same size/height/texture) =
the "black bowl" class; c3 is a different, smaller, lighter asset. Distances
to the plate centre: c5 0.274, c3 0.316, c4 0.365. **c5 is the nearest vessel
to the plate under either reading**, so "the black bowl next to the plate" = c5.

Mechanism consequence: bowl rim outer diameter 0.110 > max jaw opening 0.0778,
so the bowl cannot be straddled. It must be **rim-pinched** (one jaw inside the
bowl, one outside), with the jaw axis along world y.

Unknowns to calibrate before a grasp can be planned:
1. fingertip z offset below the EEF origin,
2. whether `rotation=None` really holds the wrist straight down,
3. move accuracy / reach at the target.

## v2 — first end-to-end attempt (seeds 51,53,55,57), 0/4

Receipt: `results/fs_c2clean_spa_bowl_next_to_plate_pos_k0_v2` 0/4.

Hypothesis: rim-pinch the nearest bowl (jaws along world y, aim at the rim
wall), lift, carry to the plate centre, open.

Evidence:
- **Perception is stable.** All four seeds identified the same plate
  (c=(0.071-0.073, 0.028-0.043), d=0.136) and the same target bowl
  (c≈(0.01-0.02, 0.30), rim radius 0.055, top 0.952); the runner-up candidate
  was 0.32-0.33 away against the target's 0.26-0.29.
- **The rim pinch works 4/4**: closed gap 0.0051-0.0053, effort 3.0.
- **The in-episode tip probe was useless**: the commanded probe point
  (0.22, 0.10) is out of reach; the arm ran to (0.156, 0.095) on every seed and
  the retreat move failed outright (commanded z 1.062, stayed at 0.910). It did
  however stall on the table at EEF z 0.9093 against a table plane of 0.9016,
  which is a usable fingertip offset of **0.0077**.
- **The failure is the release.** `held_geometry` masked a 0.25m-deep slab under
  the EEF, so it swallowed the table and reported hang=0.167 on every seed. The
  release went to EEF z 1.109, i.e. ~15cm above the plate; the bowl was dropped.
- Side finding: `effort` is only a threshold on the gap at 0.005 (0.0050 -> 3.0,
  0.0049 -> 0.05), so it is not an independent holding signal.

Verdict: perception + grasp correct, release arithmetic wrong.

## v3 — release from grasp geometry (seeds 51..65 odd), 8/8

Receipt: `results/fs_c2clean_spa_bowl_next_to_plate_pos_k0_v3` **8/8**.

Changes: dropped the unreachable tip probe (TIP_OFF fixed at the measured
0.0077); release height derived from the grasp, where the fingertip offset
cancels: `z_rel = plate_top + (z_grasp - table_z) + CLEAR`; "black bowl" = the
largest same-diameter vessel group (the repeated asset, d=0.110-0.111) so the
odd lighter vessel (d=0.087) can never be selected; bias-cancelled xy; a
verify+retry tail.

Evidence and the residual defect:
- The episode **freezes on predicate fire** (every move after the release
  returns the same EEF), so the retry tail never actually executed. Success
  happened at the attempt-1 release on all 8.
- **Two grasp modes, cleanly separated by depth**:

  | fingertip below rim | close gap | gap after lift | hang | seeds |
  |---|---|---|---|---|
  | 0.0157-0.0167 | 0.0081-0.0085 | 0.0079-0.0083 | 0.027-0.028 | 53,57,59,63,65 |
  | 0.0247 | 0.0071-0.0078 | 0.0045-0.0047 | 0.048-0.050 | 51,55,61 |

  The deep grasp loses ~3mm of gap during the lift and the bowl slips down to
  hang by its lip (hang 0.049 ≈ the bowl's full 0.050 height). It still scored,
  but only because the 21mm of unmodelled hang made the release press the bowl
  into the plate instead of dropping it — and the blocked descent then made the
  bias-cancel loop walk the x command 13mm away from the target.
- The depth scatter came from the corrective moves themselves: each extra move
  at the grasp pose sinks the wrist another ~8mm.

Verdict: 8/8 but on a mechanism with two regimes, one of them accidental.

## v4 — one-shot descent, measured hang, guarded bias loop (seeds 51..65 odd), 8/8

Receipt: `results/fs_c2clean_spa_bowl_next_to_plate_pos_k0_v4` **8/8**.

Changes:
1. `GRASP_DEPTH` 0.022 -> 0.018 and the grasp descent is now a **single move**
   (no bias-cancel at the grasp pose). The pinch is insensitive to x error — at
   the -y extremum of a 0.055 circle a 9mm x error moves the rim by 0.6mm — and
   the per-move y error is <=3mm, so the corrections bought nothing and cost
   8mm of depth.
2. Release height from the **measured** hang of the held bowl
   (`z_rel = plate_top + hang + TIP_OFF + CLEAR`), clamped to [0.015,0.060],
   falling back to the geometric 0.032.
3. The bias-cancel loop stops when a move fails to shift the EEF by 1mm
   (blocked by contact) instead of walking the command away.
4. Retry class filter tightened to CLASS_TOL so a failed attempt cannot pick up
   the smaller vessel.

Evidence — the slip regime is gone:

| | v3 | v4 |
|---|---|---|
| fingertip below rim | 0.0157-0.0247 | 0.0087-0.0187 |
| gap lost during the lift | up to 0.0031 (3/8 slipped) | <= 0.0004 (0/8) |
| measured hang | 0.027 or 0.049 (bimodal) | 0.033-0.039 (unimodal) |
| placement xy error | up to 0.0067 | <= 0.0022 |
| bias loop blocked | yes, on the 3 slip seeds | never |

Verdict: same probe score, strictly better margins on every mechanism receipt.
v4 selected for the formal 15-seed run.

## v5probe — aim-envelope measurement (NOT a candidate), seeds 51,55,59,63: 4/4

Receipt: `results/fs_c2clean_spa_bowl_next_to_plate_pos_k0_v5probe` 4/4.

v4 with the release xy deliberately displaced by (+0.015, +0.015) — 21mm total.
Still 4/4, so the "bowl on plate" predicate tolerates at least 21mm of
placement error. v4's worst measured placement error over the full debug band
is 4.4mm, i.e. roughly a 5x margin. The 15/15 is therefore not a knife-edge
result on placement aim.

---

# DECLARATION

**Frozen version: v4.** `packs/c2clean_spa_bowl_next_to_plate_pos_k0/program.py`
md5 `f05298c52effc12d8fbcb37f8073a09b` == `program_v4.py` (verified on the
cluster).

**Selection receipt (formal, full 15 debug seeds 51-65):**
`results/sel_c2clean_spa_bowl_next_to_plate_pos_k0_v4` — **15/15**
(`"benchmark_success": true` x15).

**Per-version receipt chain**

| version | seeds | score | receipt dir |
|---|---|---|---|
| v1 observation only | 51,53 | 0/2 | `fs_..._v1` |
| v2 first end-to-end | 51,53,55,57 | 0/4 | `fs_..._v2` |
| v3 release from grasp geometry | 51..65 odd | 8/8 | `fs_..._v3` |
| v4 one-shot descent + measured hang | 51..65 odd | 8/8 | `fs_..._v4` |
| **v4 formal selection** | **51-65 (all 15)** | **15/15** | `sel_..._v4` |
| v5probe aim envelope (not a candidate) | 51,55,59,63 | 4/4 | `fs_..._v5probe` |

**PROVENANCE**: present as a top-level literal dict in program.py, covering all
13 calibrated constants. Every source is either a debug-seed (51-65) measurement
or generic controller/camera mechanics. No pack was supplied to this cell (K=0)
and none was read.

**Mechanism, in one paragraph.** The bowl's rim is 0.110 across and the jaws open
to 0.0778, so it cannot be straddled; it is rim-pinched with the jaw axis along
world y (the start tool rotation maps tool y to world -y), the jaw pair centred
on the rim wall at `bowl_centre + (0, -rim_radius)`. Grasp depth is the whole
game: fingertips 24.7mm below the rim top let the bowl slip down and hang by its
lip, while 8.7-18.7mm holds it rigidly (gap loss <=0.5mm over the lift on 15/15).
Because the fingertip-to-EEF offset cancels between the grasp and the release,
the release height needs no tip calibration; v4 instead re-perceives the held
bowl's hang after the lift and releases at `plate_top + hang + TIP_OFF + 6mm`.
Target identity comes from the instruction alone: "the black bowl" is the
repeated asset (the two 0.110-diameter vessels, against a lone 0.087 one), and
"next to the plate" is the nearer of that pair — a margin of 71-122mm on all 15
seeds, so the `_pos` perturbation never threatens the choice.

**Margins over the full debug band (15/15):** target-selection margin
0.071-0.122 m; fingertip depth below the rim 8.7-18.7mm against a slip threshold
at >=24.7mm; grip gap lost during the lift 0.0-0.5mm; measured hang 0.033-0.040m
(unimodal); placement xy error 0.5-4.4mm against a >=21mm tolerance measured by
v5probe.

STOP.
