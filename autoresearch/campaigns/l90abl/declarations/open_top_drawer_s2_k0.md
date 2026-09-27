# l90abl / open_top_drawer_s2_k0 — notes

Intent: "open the top drawer of the cabinet" (KITCHEN_SCENE2). Zero demos.
Runner: tools/fair_run.py only. Debug seeds 51-65.

## Scene, as measured (cam_high depth, seeds 51 and 53, agreeing to ~3 mm)

| quantity | value |
|---|---|
| table top | z = 0.901 |
| cabinet front panel plane | y = -0.2175 |
| cabinet top face | z = 1.127, front edge also at y = -0.218 (flush with the panel) |
| three handle bars (z peaks) | 0.951 / 1.021 / 1.096 — top drawer = the 1.096 one |
| top bar visible top face | z = 1.091..1.098, y = -0.206..-0.186 |
| top bar posts | x ~ -0.030 and +0.035; clear span between them, centre x ~ 0.003 |
| fingertip offset below eef | 0.0154 m |
| finger thickness above the tip | ~0.017 m at tip+0.012, ~0.022 m at tip+0.032 |
| gripper open inner faces | ±0.0389 m about the eef, along tool y |

The drawers face +y and open toward +y.

## Versions

### v1 — perception dump (seeds 51,53). 0/2, no motion.
Hypothesis: none, instrumentation. Established the table/cabinet/handle numbers
above and that cam_high's image-right is base +y.

### v2 — first motion probe. 0/1.
Hypothesis: a move that fails to reach its target has hit something.
Evidence: `api.move(seconds=1.5..2.0)` leaves 8-45 mm of tracking error even in
free air. Verdict: WRONG — non-arrival is not evidence of contact. Need a
convergent goto with a real stall test.

### v3 — calibration with a convergent goto. 0/1.
Hypothesis: press the closed gripper onto the cabinet top slab to get the
fingertip offset.
Evidence: pressing at (0, -0.30) stalled hard at eef_z = 1.1424 with the slab at
z = 1.127, and three further attempts slid laterally 3.6 mm each while z did not
move at all — a real contact. Verdict: **TIP_DZ = 0.0154**.
Also: the episode budget is **1000 sim steps**; a *blocked* move burns its whole
`seconds`x100 allowance, which is what exhausted this episode.

### v4 — horizontal-wrist pinch of the top bar. **2/3** (51 ok, 53 fail, 55 ok).
Hypothesis: the 12 mm slot between the bar back (-0.206) and the panel (-0.218)
is too narrow to hook — the finger is 17-22 mm thick a centimetre above its tip
and would foul the panel. So turn the tool to point along -y (tool z = world -y,
tool y = world +z), let the fingers separate vertically, and pinch the bar from
the front; then pull +y.
Evidence: seed 51 closed to width 0.0165 at effort 3.00 (the bar is ~16 mm
thick) and the pull carried the eef 146 mm in +y; success fired at step 131.
Verdict: **RIGHT — this is the mechanism.**
Failure mode on 53: close gave width 0.0015, effort 0.05 — air. That seed's bar
front is at y = -0.1898 vs -0.1864 on 51, and the insert move under-shot its
target by 10 mm, leaving the fingertips 3.5 mm short of the bar.
Also learned: a *converged* move is cheap. v4 spent only 214 of 1000 steps for
2.5+2.0+3.0 s of commanded motion, so there is room for perception and retries.

### v5 — per-seed perception + verified close. (running)
Hypothesis: reading the bar's own front y / top z / clear x per episode,
re-issuing the insert so it converges, and retrying the close 8 mm deeper
whenever effort stays at 0.05, removes the seed-53 failure mode.
Evidence: 8/8 on the probe subset (51,53,...,65),
`results/fs_l90abl_open_top_drawer_s2_k0_v5`, then 15/15 on the full debug
split, `results/sel_l90abl_open_top_drawer_s2_k0_v5`. Every seed gripped on the
**first** close — width 0.0174, effort 3.00, no retry used — and realised
dy = 0.148 m. Verdict: **RIGHT**; the per-seed read of the bar front absorbs the
3-4 mm of seed-to-seed variation that broke v4 on seed 53.

### v6 — v5 with a complete PROVENANCE block. **15/15** (frozen).
Behaviour identical to v5; `STAGE_Y` and the three perception fallbacks
`FB_BAR_FRONT_Y / FB_BAR_TOP_Z / FB_BAR_X` were constants v5 had left
undeclared. Re-ran the formal selection so the receipt matches the frozen file.
Evidence: 15/15, `results/sel_l90abl_open_top_drawer_s2_k0_v6`, seeds
51-65 all true at 132-134 sim steps.

## Method, in one paragraph

The three grey handle bars separate cleanly by height in a cam_high point
cloud (z peaks 0.951 / 1.021 / 1.096); the highest is the top drawer's. The
12 mm slot between that bar's back face and the cabinet front panel is too
narrow to hook, because the finger is 17-22 mm thick a centimetre above its
tip and would foul the panel — and the cabinet top edge is flush with the
panel, so there is no room above either. The move that works is to turn the
tool to point along -y (tool z = world -y, tool y = world +z) so the fingers
separate *vertically*, drive the palm ~4 mm past the bar's own measured front
face at 8 mm below its measured top face, pinch the bar between the upper and
lower finger, and pull +y. Gripper effort is the receipt: 3.0 means the bar is
held, 0.05 means the close caught air, and the program retries 8 mm deeper.

## DECLARATION

- **Frozen version:** `packs/l90abl_open_top_drawer_s2_k0/program.py`,
  md5 `0ebba61f298a4c413ca937b18617ba7d` == `program_v6.py` (same md5).
- **Selection receipt:** **15/15** on the full 15 debug seeds (51-65),
  `results/sel_l90abl_open_top_drawer_s2_k0_v6`.
- **Receipt chain:** v1 0/2 (perception dump, no motion) · v2 0/1 (motion probe;
  refuted "non-arrival == contact") · v3 0/1 (calibration; TIP_DZ = 0.0154,
  1000-step budget) · v4 2/3 on 51,53,55 (`fs_..._v4`, mechanism found) ·
  v5 8/8 probe (`fs_..._v5`) then 15/15 formal (`sel_..._v5`) ·
  v6 15/15 formal (`sel_..._v6`, frozen).
- **PROVENANCE:** present in program.py, covering all 13 calibrated constants;
  every source is a debug-seed (51/53) cam_high or wrist-cam measurement, a
  debug-seed motion receipt, or generic tool-frame/controller mechanics.
- Evaluation seeds 1-50 were never run or touched.
