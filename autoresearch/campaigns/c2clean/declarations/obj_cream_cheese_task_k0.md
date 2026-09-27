# c2clean / obj_cream_cheese_task_k0 — working notes

Intent: **"Pick the alphabet soup and place it in the basket"** (zero demos).
Runner: `tools/fair_run.py` only. Debug seeds 51–65.

The bddl filename names cream cheese; the *instruction* names the alphabet
soup. Resolved empirically, not by assumption: v6 grasped the blue "SOUP" can
and the benchmark bit came back `true` on 4/4 seeds. The graded target is the
object the instruction names.

## Scene, as measured (v1/v2 dumps, all 15 debug seeds)

`api.log` truncates each message at ~2000 chars, so RGB-D is shipped out as
zlib+base64 in ≤1900-char chunks and perception is done offline from
`program_ep<seed>.log`. Table plane (median z of the cropped cloud below 0.02)
= **0.0011 m** on every debug seed.

Seven props plus the robot. Connected components of the cloud above
table+0.008, workspace-cropped to x∈(−0.40,0.40), y∈(−0.45,0.50):

| comp | ztop−table | B−R in the 8–30 mm band | what it is |
|---|---|---|---|
| n≈1603 | 0.080 | **+0.079** | blue "SOUP" can — the target |
| n≈2268 | 0.080 | −0.108 | red/green can |
| n≈1833 | 0.142 | −0.342 | orange-juice carton |
| n≈1931 | 0.141 | −0.242 | milk carton |
| n≈11900 | 0.143 | +0.003 | basket (destination) |
| n≈826 | 0.019 | +0.072 | flat blue box |
| n≈885 | 0.018 | −0.168 | flat red box |
| n=12001 | 0.485 | −0.000 | the robot's own body |

Both cans and the OJ carton sit at *pixel-identical* coordinates on all 15
debug seeds; only the basket (±0.015 m), the milk carton and the two flat boxes
jitter. This is a `_task` cell, so layout is near-deterministic — but the
program re-perceives every episode rather than leaning on that.

Target identification is therefore a two-stage cue, both re-derived here:
**height class** (0.05 < ztop−table < 0.12) isolates the two cans, then
**blue-band colour** (mean B−R over the band 8–30 mm below the component's top)
separates them by 0.19, with every non-can component at |B−R| ≤ 0.003 or
strongly negative. Kasa circle fit on the rim band (top 8 mm) gives the grasp
centre; r = 0.0304, i.e. a 61 mm can against 78 mm of jaw travel — 8 mm of
clearance per side.

## Motion law (the expensive lesson)

- **v4:** a `goto` that re-issues the *same* `api.move` target is a no-op.
  `api.move` considers itself converged at |err| ≈ 0.008–0.012 and returns
  without stepping the sim; three retries cost 0 steps and changed nothing.
  A correction must move the **command**, not repeat it.
- **v5:** folding the full residual into the command (cmd = p + err) is
  violently unstable when the residual is large — single calls travelled
  0.15–0.32 m and the arm oscillated across the table, burning the whole
  1000-step horizon and knocking the target can away.
- **v6 law, used since:** command `p` exactly while |err| ≥ 0.030; fold in
  `0.8·err` only once |err| < 0.030. First call lands within ~1 cm, the nudge
  closes it to ~2–5 mm. One full pick-and-place costs ~200 of 1000 steps.

Descent needs no fingertip calibration: at the grasp the can's bottom rests on
the table, so `eef_z − z_grasp` *is* the height of the can's bottom for the rest
of the episode. Release height = `z_grasp + basket_ztop + 0.03`.

## Version chain

| ver | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | ship RGB-D out through `api.log` | 3000-char chunks truncated at ~2000 | fixed → CHUNK 1900 |
| v1b | half-res dump decodes | cloud reproduces `api.deproject` to 1e-4 m | perception pipeline validated |
| v2 | full-res dump, 15 seeds | scene table above; layout near-deterministic | target and cues identified |
| v3 | descend-and-grasp with 1 s moves | "blocked" fired spuriously; move is time-budgeted, not convergent | rejected |
| v4 | converging goto by repeating the target | 213 sim steps, retries were no-ops | rejected; found the no-op law |
| v5 | bias-cancelled goto, gain 1.0 | 0.15–0.32 m oscillations, horizon exhausted, can knocked away | rejected; found the instability |
| v6 | gain applied only inside 0.030 m | **4/4** on 51,55,60,65; held width 0.0625 @ effort 3.0 | accepted |
| v7 | v6 + sensor verification and retry | **8/8** on 51,53,…,65, every episode verified on the first attempt, zero retries | selected |

v7 adds, over v6: a post-lift hold check (width ∈ (0.030, 0.076) and effort >
1.0), a mid-carry hold check, and a post-release re-perception that requires no
blue can to remain in the can height class — each failure retries the whole
pick, up to 3 attempts inside the 1000-step budget.

## Aim envelope

15/15 says nothing about margin, so the grasp aim was deliberately displaced
and re-run on 4 seeds (`program_env.py` / `program_env12.py`, v7 with a bias
added to the grasp xy only):

| injected bias | diagonal | result | closed width |
|---|---|---|---|
| none | 0 | 15/15 | 0.0625 |
| +6, +6 mm | 8.5 mm | **4/4** | 0.0629 |
| +12, +12 mm | 17 mm | **4/4** | 0.0610–0.0625 |

The closed width barely moves across a 17 mm displacement: a top-down straddle
of a free-standing cylinder self-centres as the jaws close, so the grasp is not
aim-limited at the accuracy this perception delivers (Kasa centre repeatable to
~2 mm, `goto` lands within 2–5 mm). The real bound is jaw travel, 78 mm of
opening against a 62.5 mm can.

## DECLARATION

- **Frozen version: v7.** `packs/c2clean_obj_cream_cheese_task_k0/program.py`
  md5 `342da3220b5d7dc9bf0c5219097b853f` == `program_v7.py` (8371 bytes) ==
  `results/sel_c2clean_obj_cream_cheese_task_k0_v7/program_archived.py`.
- **Selection receipt: 15/15** on the full debug split 51–65, one formal run,
  `results/sel_c2clean_obj_cream_cheese_task_k0_v7`. Every episode succeeded on
  attempt 0; no retry path was exercised.
- **Per-version receipt chain**
  - v1 `fs_…_v1` (3 seeds) — log truncation found, no motion
  - v1b `fs_…_v1b` (4 seeds) — RGB-D round-trip validated against `api.deproject`
  - v2 `fs_…_v2` (15 seeds) — full-res scene dump; target + cues identified
  - v3 `fs_…_v3` (4 seeds) — 0/4, time-budgeted move misread as contact
  - v4 `fs_…_v4` (4 seeds) — 0/4, 213 sim steps; repeated move is a no-op
  - v5 `fs_…_v5` (2 seeds) — 0/2, 1000 steps burned by command oscillation
  - v6 `fs_…_v6` (51,55,60,65) — **4/4**, ~200 sim steps per episode
  - v7 `fs_…_v7` (51,53,…,65) — **8/8**, all verified on attempt 0
  - v7 `sel_…_v7` (51–65) — **15/15** ← selection
  - envelope `fs_…_env6mm` **4/4**, `fs_…_env12mm` **4/4**
- **PROVENANCE** present as a top-level literal dict in `program.py`, covering
  every calibrated constant; all sources are debug-seed measurements from the
  v1/v2/v4/v5/v6 runs or generic controller/camera mechanics. No pack was
  issued for this cell and none was read.
- No `api.done` read; success was judged only post-episode from
  `results.jsonl`. Eval seeds 1–50 were never touched.
