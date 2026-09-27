# c2clean goal_put_wine_top_cabinet_pos_k0 — worker notes

Intent: "put the wine bottle on top of the cabinet". Zero demonstrations; every
constant re-derived from debug seeds 51-65 (probe subset 51,53,...,65).

## v1 — perception probe (results/fs_..._v1, v1b; seeds 51,53,55,57)

Hypothesis: with no pack, the scene must be read from cam_high RGB-D alone.
Method: dumped RGB + depth through `api.log` as zlib+base64 (chunked) and did
the geometry offline on the Mac. First attempt used 6000-char chunks; `api.log`
truncates a message at ~2000 chars, so the payload was corrupt — 1700-char
chunks fixed it (v1b).

Evidence (all four seeds, cam_high deprojected to the base frame):
- table plane z = 0.905 (dominant z bin; 130k points).
- cabinet = flat grey slab, top z = 1.128 (table+0.223), x[-0.53,-0.27],
  y[0.10,0.33], ~0.25 x 0.22 m, mean RGB [63,61,59]; essentially fixed across
  seeds (<=0.01 m drift).
- wine bottle = dark ([8,12,7]) column, top z = 1.059 (table+0.154), body
  diameter 0.036-0.042 over z in [ztop-0.13, ztop-0.07], neck 0.016 above that,
  gold cap at the top 0.01. `_pos` moves it ~0.02 m between seeds
  (51: (-0.094,0.000), 53: (-0.090,0.005), 55: (-0.086,-0.011), 57: (-0.077,-0.013)).
- other props (bowl rim table+0.045, plate/blue box table+0.015, stove slab,
  wooden rack) are all far shorter than the bottle -> a height gate of
  table+0.10 isolates it; the only taller dark blob is the robot arm, which is
  rejected because it floats (its lowest top-cell is table+0.43).
- cam_high sees only the near (+x) face, so the centre is recovered as
  x = x_max(band) - r with r from the fully-visible y extent.

Verdict: perception sufficient; no demos needed.

## v2 — first grasp/carry/place probe (results/fs_..._v2, v2b, v2c)

v2: crashed — the tallest dark blob was the robot arm. v2b: added the
"grounded" test, then the cabinet detector's flat-fraction cut (0.35) rejected
the real cabinet at 0.34. v2c (cut 0.15 + flat-point/span tests): **4/4
benchmark_success on seeds 51,53,55,57.**

Evidence from v2c: commanding xy = bottle centre, z = ztop-0.09 leaves a 0.06 m
residual (the descent is starved) but closes on the **neck** (width 0.0150-0.0162,
effort 3.0) in 4/4 seeds; the bottle then hangs 0.121 m below the eef. The
episode terminated mid-carry over the cabinet, i.e. the predicate fired as the
hanging bottle arrived at the cabinet top — the carry height happened to put the
bottle base 0.007 m above the slab.

Verdict: mechanism works, but success came from a skim, and every api.move was
starved (a 0.36 m move left a 0.11 m residual). v3 makes it deliberate.

## v3 — converged moves, hold check, real place-down

Changes: `goto()` repeats api.move until the residual converges (bounded tries);
a grip check (effort > 1 and width in [0.008,0.030]) with one re-perceive/retry;
the hang length is measured at runtime (eef z at closure - table); the place
point is the cabinet-top centroid pulled 0.07 m inside the measured rectangle;
the bottle is lowered to top+hang+0.006, released, and the arm retreats; then
the scene is re-perceived to log where the bottle ended up.

(receipts below)

Receipt: probe seeds 51,53,...,65 -> **8/8**
(results/fs_c2clean_goal_put_wine_top_cabinet_pos_k0_v3); formal full-15
selection -> **14/15** (results/sel_c2clean_goal_put_wine_top_cabinet_pos_k0_v3),
failing seed 52.

Seed 52 post-mortem (its log): the descent stalls at ztop-0.042 in every seed
because the gripper lands on the bottle's cap and the jaws straddle the neck —
that stall IS the grasp registration, not a starved move. v3 issued a second
blind push at the stall (tries=2); on seed 52 (bottle 0.005-0.02 m further in
-x than any other seed) that push toppled the bottle: the close returned width
0.0010 / effort 0.05 (empty jaws), the re-perceive found no standing bottle
(`BOTTLE cands 0`), and the run carried nothing to the cabinet.

## v4 — stepped, contact-sensing descent (FROZEN)

Hypothesis: the topple comes from pressing after contact, so replace the blind
descent with 0.018 m steps (0.8 s each) from ztop+0.075 and stop at the first
step whose achieved Δz < 0.006 — the contact that registers the cap.
Everything else is v3.

Evidence: seed 52 now steps 1.116 -> 1.022 with Δz 0.019,0.018,...,0.013 and
then 0.0012, logs `contact at z=1.0217 (ztop-0.037)`, closes at width 0.0150 /
effort 3.00, held=True, and places. Probe seeds 52,54,56,58,60,51,53 -> **7/7**
(results/fs_c2clean_goal_put_wine_top_cabinet_pos_k0_v4). Episodes also got
~45% shorter (357 vs 631 steps).

Verdict: frozen.

## DECLARATION

- Frozen version: **program_v4.py**; `packs/c2clean_goal_put_wine_top_cabinet_pos_k0/program.py`
  md5 `3532b5cafb9a9276233d0c87c344cda8` == `program_v4.py` md5 (verified on AbakaAI).
- Selection receipt (one formal run, all 15 debug seeds 51-65):
  **15/15 benchmark_success**, dir
  `results/sel_c2clean_goal_put_wine_top_cabinet_pos_k0_v4` (no program errors).
- Per-version receipt chain:
  - v1  perception probe (no motion) — fs_..._v1 (chunking bug), fs_..._v1b: RGB-D
    recovered for seeds 51,53,55,57; scene geometry measured.
  - v2  grasp/carry/place probe — fs_..._v2 (crash: arm mistaken for the bottle),
    fs_..._v2b (cabinet rejected by a too-tight flatness cut), fs_..._v2c: **4/4**
    on 51,53,55,57. (The cluster's program_v2.py is the v2c state of that file.)
  - v3  converged moves + hold check + explicit place-down — fs_..._v3: **8/8** on
    51,53,...,65; sel_..._v3: **14/15** (seed 52 topple).
  - v4  stepped contact-sensing descent — fs_..._v4: **7/7** on 52,54,56,58,60,51,53;
    sel_..._v4: **15/15**.
- PROVENANCE: present as a top-level literal dict in program.py; every one of the
  24 module-level constants is covered by a key, all sourced from debug-seed
  (51-65) RGB-D/EEF/gripper measurements or generic camera/controller mechanics.
  No pack was issued for this cell and none was read; no `.done` read anywhere.

Mechanism summary (what the cell taught): a "put X on top of Y" cell with no
demos is solvable from geometry alone here — a height gate plus a grounded test
isolates the bottle, the flat-slab test names the cabinet top, and the grasp
needs no tip calibration because the gripper self-registers on the bottle's cap:
descend until the eef stops moving, then close on the neck it is straddling. The
one thing that breaks it is pushing after that contact.
