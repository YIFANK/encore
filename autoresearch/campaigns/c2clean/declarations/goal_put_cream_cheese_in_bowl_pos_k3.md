# c2clean — goal_put_cream_cheese_in_bowl_pos_k3

Intent: "put the cream cheese in the bowl".  Runner: `tools/fair_run.py` only.
Splits: debug 51–65 (mine), eval 1–50 (never touched).

## Scene, as derived from the pack + my own debug-seed captures

Pack (K=3): four keyframes per demo.  t≈40 is the close (ee z 0.9104 / 0.9205 / 0.9106),
t≈75–95 is the release (ee z 0.959 / 0.973 / 0.979), wrist yaw near zero at both, so the
demos are a straight-down pick with jaws on the object's short axis and a release ~2 cm
above the receptacle.  The keyframe images show one small blue carton that is absent from
the table and present inside the bowl in the last frame — that difference names the target.

My own cam_high captures on all 15 debug seeds (v0 was a pure perception dump; RGB+depth
shipped out through `api.log` as zlib+base64 and decoded off-box):

| thing | measurement (all 15 debug seeds) |
|---|---|
| table top | z ≈ 0.9012 |
| carton | 75 × 39 × 18 mm, top z 0.9196 ± 0.0001, long axis within 6° of +x, the only workspace surface with B>R+20 and B>G+10 |
| bowl | rim top z 0.9522 ± 0.0003, Kasa radius 0.0526 ± 0.0001 |
| plate (distractor) | top z 0.920 |
| stove (distractor) | top z 0.932 |
| bottle (corridor obstacle) | top z 1.059, footprint ~28 × 42 mm |
| cabinet | stepped: over the carton's x range a ledge reaches y ≈ −0.125 and tops at z = 1.098; set back to y ≈ −0.15 elsewhere (tops 1.019 / 1.127) |
| eef ↔ fingertips | fingertips ≈ 0.030 m below the eef (free-table descents stop at eef z ≈ 0.931) |

Perception that ships: carton = blue mask → top 4 mm slab → oriented-extent midpoint;
bowl = rim height band 0.942–0.958 → fixed-radius vote on a 2 mm grid → Kasa refit on the
inliers (occlusion-robust: the bottle fuses with the bowl in 7 of 15 seeds and a bbox
centre is then up to 2 cm wrong, the circle fit is not).

## Version chain

| ver | hypothesis | evidence | verdict |
|---|---|---|---|
| v0 | perception dump, no motion | 15/15 seeds decoded; carton and bowl located on every seed | perception is solid |
| v1 | straight-down pick, diagonal transits, release 2 cm over the rim | **5/8** probe (51,53,57,59,65 ok; 55,61,63 fail). 55/63 burned the 1000-step cap with the descent frozen at eef z 1.0635; 61 closed on a 16 mm corner bite after a diagonal descent dragged the carton | moves must be closed-loop; descent must be vertical |
| v2 | close every move in a loop, converge xy at altitude, verify jaw width, retry the grasp | **6/8** (61 fixed). 55/63 still frozen at *exactly* z 1.0635 | the stall is a fixed obstacle, not tracking error |
| v3probe | is the column reachable at all? descend at dy = 0.08/0.05/0.02/0.00 from the carton on seeds 55/57/63 | every descent reached z ≈ 0.93 — but the controller had left the eef ~1 cm further +y than commanded on the dy=0 runs | no reach limit; the blocker is y-dependent |
| v4 | walk in laterally at altitude instead of one long diagonal | **6/8**, 55/63 frozen at 1.0635 again with eef y −0.038 / −0.037 | walking is not the fix; confirms a y threshold |
| v5 | the hand lands on the cabinet ledge (top 1.098): blocked at eef_y − y_wall = 0.0868, clear at 0.0956. Push the grasp column out to y_wall + 0.100, capped at +0.015 so the jaws still straddle the carton (half-jaw 0.039 vs half-carton 0.0195) | **8/8** probe, **15/15** selection | FROZEN |

Only seeds 55 (+0.0106), 60 (+0.0020) and 63 (+0.0114) need the offset; the closing jaw
slides the carton the same distance back to the jaw centre, so the bite is still central.

## DECLARATION

- Frozen version: **program_v5.py**, `md5 = e4f4d6d1952e1ea15a99e75fb37efd37`,
  identical to `packs/c2clean_goal_put_cream_cheese_in_bowl_pos_k3/program.py` (same md5 on the cluster).
- Selection receipt (full 15 debug seeds, one formal run):
  **15/15** — `results/sel_c2clean_goal_put_cream_cheese_in_bowl_pos_k3_v5`
  (51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 all `benchmark_success: true`;
  198–672 sim steps of the 1000 budget).
- Per-version receipts: v1 5/8 `fs_..._v1`, v2 6/8 `fs_..._v2`, v3probe (envelope map)
  `fs_..._v3probe`, v4 6/8 `fs_..._v4`, v5 8/8 `fs_..._v5` — all on probe seeds
  51,53,55,57,59,61,63,65; v0 15/15 perception dump `fs_..._v0`.
- PROVENANCE: present in program.py as a top-level literal dict covering every calibrated
  constant (WS bounds, blue thresholds, top slab, rim band/radius/tolerance, GRASP_DZ,
  STAGE_Z, APPROACH_DY, RELEASE_DZ, MOVE_TOL, HOLD_W_MIN, OPEN_W, Y_CLEAR, MAX_OFF,
  LEDGE_Z), each sourced to this pack's keyframes or to my own debug-seed measurements.
- Clean room: no `--split eval`, seeds 1–50 never touched; writes confined to
  `packs/c2clean_goal_put_cream_cheese_in_bowl_pos_k3/*` and
  `results/*c2clean_goal_put_cream_cheese_in_bowl_pos_k3*`; `api.done` never read; no
  forbidden file opened; no shared note file.
