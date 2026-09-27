# c2clean / obj_chocolate_pudding_task_k3

Intent: **"Pick the salad dressing and place it in the basket"**
Runner: `tools/fair_run.py` only. Splits sealed: debug = 51-65.

## Scene (re-derived from debug seeds 51/53/55/57, cam_high RGB-D via api.log datapipe)

Base frame: table top at z ~= 0.006 m. cam_high at t=(0.897, 0, 0.65), optical axis
(-0.849, 0, -0.529) -- a 32-degree grazing view, so side-facing surfaces are biased
toward +x while lateral (y) extents and *top* discs are unbiased.

Top-down 5 mm height map, seed 51 (identical on 53/55/57 except the basket):

| prop | mid (x,y) | z_top | note |
|---|---|---|---|
| basket | (+0.008, +0.258) | 0.144 | 492 cells; every other prop <= 145 |
| robot gripper (home) | (-0.147, -0.003) | 0.299 | fuses with the BBQ-sauce bottle |
| chocolate pudding (flat box) | (-0.120, -0.240) | 0.029 | lying flat, far left |
| can | (-0.197, -0.082) | 0.081 | |
| orange juice carton | (+0.050, -0.100) | 0.143 | |
| **salad dressing** | (+0.152, +0.030) | 0.148 | green cap, top-band rgb (0.12,0.26,0.17) |
| tomato ketchup | (+0.103, -0.200) | 0.148 | |

Height-band profile of the dressing bottle (seed 51): body y-width 0.060-0.063 below
z=0.08; **green cap is a 0.035-wide cylinder from z=0.110 to z=0.148**. The cap's top
disc is fully visible, so its bbox mid (+0.150, +0.0295) is an unbiased centre.

Greenness (g - (r+b)/2) of the top 25 mm band separates the dressing from everything
else by a factor of ~10: dressing 0.115, every other prop <= 0.01. The robot arm
renders green/blue in this build, so the arm must be masked (z<0.19 plus a 0.085
box around the start eef xy).

## Packs

- `..._task_k3` language = "pick up the chocolate pudding and place it in the basket"
  -- same scene as mine. Demos close at (-0.11,-0.25,z=0.011) = the flat pudding box,
  hold gap 0.0465 (= the box's measured y-width 0.046, so **gap == sum of the two
  finger coordinates, no offset**), release over the basket at z 0.164-0.176.
- `..._task_mate` language = "pick up the salad dressing and place it in the basket"
  -- a *different* scene (different prop set) whose dressing bottle is a slim
  narrow-necked model: grasp z 0.117-0.128, hold gap 0.018. That gap does NOT
  transfer: my scene's dressing has a 0.035 cap. Only the *phase structure*
  (descend / close / lift to 0.29-0.31 / traverse to the basket / descend to
  ~0.18-0.20 / open) transfers.

## Version log

### v0 / v0b -- perception probe (no manipulation)
Hypothesis: the cam_high RGB-D can be shipped out through `api.log` (zlib+base64) and
the whole scene reconstructed offline.
Evidence: v0 chunks at 3000 chars were truncated by the logger's ~2000-char line cap
(320760 of 485828 base64 chars survived); at 1800 chars all four arrays round-trip.
seeds 51/53/55/57, 72 sim steps each, benchmark_success false (expected: no motion).
Verdict: datapipe works; scene table above.

### v1 -- green-cap grasp, perceived basket
Hypothesis: the intent's target is the green-capped dressing bottle; grasping its cap
(z_top - 0.022) and releasing over the perceived basket centre at rim + 0.055 wins the
benchmark bit.
Evidence: **8/8** on probe seeds 51,53,...,65
(`results/fs_c2clean_obj_chocolate_pudding_task_k3_v1`). Identification margin is
decisive -- greenness 0.121 for the dressing vs 0.033 for the runner-up (the orange
juice carton). Close reads width 0.0368 / effort 3.0 and the effort never drops through
the lift, the traverse or the descent into the basket.
Verdict: **the benchmark bit grades the object the INSTRUCTION names, not the object the
bddl filename names.** The bddl is `pick_up_the_chocolate_pudding_...` and the pudding
box was never touched, yet all 8 episodes scored true. Kept as the mechanism.

### v1-envelope -- aim-envelope sweep (the important negative result)
Hypothesis: 8/8 says nothing about margin. Displace the commanded grasp aim off the
perceived cap centre and find where it breaks. 4 seeds (51,55,59,63) per offset.

| axis | offset | score |
|---|---|---|
| y | -0.020 / +0.020 | 4/4 / 4/4 |
| z | -0.015 / +0.015 | 4/4 / 4/4 |
| x | -0.014 | 0/4 |
| x | -0.010 | 0/4 |
| x | -0.007 | 0/4 |
| x | -0.004 | **4/4** |
| x | 0.000 | **8/8** |
| x | +0.006 | **4/4** |
| x | +0.010 | **4/4** |
| x | +0.012 | 0/4 |
| x | +0.014 | 0/4 |
| x | +0.020 | 0/4 |

Verdict: y and z are slack; **x is the load-bearing axis** and its pass band is only
~16 mm wide, centred at **+0.003** relative to the perceived cap centre, half-width
~0.008. Every x failure is an *air close* (width 0.0010, effort 0.05) -- the pad misses
the cap entirely rather than grabbing and slipping, so a miss is cheaply detectable.
v1's dx=0 sat 3 mm off-centre with only 5.5 mm of margin on the tight side.

### v2 -- re-centred aim + sensor-verified retry ladder  [FROZEN]
Hypothesis: since a miss is an air close and therefore self-evident to the gripper
sensor, the narrow x band can be made irrelevant: re-centre the aim on the measured
band centre (+0.003) and, on a failed close, re-perceive and retry along a ladder of x
aims that walks the band outward (+0.003, +0.009, -0.003, +0.012, -0.006). Add a
post-release re-perception that redoes the whole cycle if the dressing is not inside
the basket footprint, and a move budget so retries cannot run the horizon away.
Evidence:
- probe 8/8 on 51,53,...,65 (`results/fs_c2clean_obj_chocolate_pudding_task_k3_v2`)
- **retry mechanism receipt**: a variant whose ladder begins with a deliberate
  30 mm miss (`..._envretry`, seeds 51,55,59,63) scores **4/4**. ep51 trace:
  `dx=-0.030 -> width 0.0010 effort 0.05 (miss)` -> re-perceive -> `dx=+0.003 ->
  width 0.0368 effort 3.00` -> release -> `VERIFY dressing at (0.020,0.258) is inside
  basket (0.008,0.257)`. The recovery and the independent placement check both fire.
- **selection, full 15 debug seeds: 15/15**
  (`results/sel_c2clean_obj_chocolate_pudding_task_k3_v2`, 221-222 sim steps/episode).
Verdict: frozen as `program.py`.

## Caveat carried forward
Debug seeds 51-65 randomise **only the basket** (mid x -0.002..+0.008, y 0.251..0.263);
the six table props deproject to identical cells on every debug seed. So the 15/15 does
not itself exercise prop-position generalisation. What covers that risk is that nothing
in the program is a hard-coded prop position -- target and basket are both re-perceived
every cycle -- plus the measured aim envelope above and the retry ladder that spans it.

## DECLARATION

- **Frozen version**: `program.py`, md5 `397f91586e78b3640375dc36b29c4430`,
  identical to `program_v2.py` (same md5).
- **Selection receipt**: **15/15** on the full debug split 51-65,
  `results/sel_c2clean_obj_chocolate_pudding_task_k3_v2`.
- **Receipt chain**:
  - v0/v0b -- perception probe, seeds 51/53/55/57, no motion (0/4 expected)
  - v1 -- `fs_..._v1`, 8/8 on 51,53,...,65
  - v1-envelope -- 12 sweep runs x 4 seeds, `fs_..._env{xp,xm,yp,ym,zp,zm,xm14,xm10,xm07,xm04,xp06,xp10,xp12,xp14}`
  - v2 -- `fs_..._v2`, 8/8 on 51,53,...,65; `fs_..._envretry`, 4/4 with a forced 30 mm miss
  - v2 selection -- `sel_..._v2`, **15/15** on 51-65
- **PROVENANCE**: present in `program.py` as a top-level literal dict covering all 15
  calibrated constants; every source is either a named pack field, a debug-seed
  measurement, or generic controller/camera mechanics.
- No forbidden reads, no `fewshot_run.py`, no `api.done`, writes confined to
  `packs/c2clean_obj_chocolate_pudding_task_k3/*` and `results/*c2clean_obj_chocolate_pudding_task_k3*`.
