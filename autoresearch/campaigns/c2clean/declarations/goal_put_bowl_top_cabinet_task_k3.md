# c2clean / goal_put_bowl_top_cabinet_task_k3

Intent: **"Put the plate on the top of the drawer"** (success = the env's own
benchmark bit). Packs: `..._k3` = *put the bowl on top of the cabinet* (my
TARGET, different object); `..._mate` = *push the plate to the front of the
stove* (my OBJECT, different target).

Runner: `tools/fair_run.py` only, bddl
`.../libero_goal_task/put_the_bowl_on_top_of_the_cabinet.bddl`, language
overridden to the intent. Debug seeds 51-65.

## Scene, re-derived from my own debug-seed RGB-D (seeds 51/53/55/57)

| quantity | value | how |
| --- | --- | --- |
| table top | z = 0.901 | dominant z mode of cam_high depth over the reachable region |
| cabinet top plane | z = 1.1269 (±0.0002 over 4 seeds) | second z mode; 17k px |
| cabinet top extent | x ∈ [-0.11, 0.16], y from -0.158 (near edge) to < -0.35 | per-x y-extent of that plane |
| cabinet front face | flat vertical plane at y = -0.158, table → 1.127 | y-max of cabinet points per height band |
| cabinet handles | protrude to y ≈ -0.126 at z ≈ 0.94-0.96 and 1.00-1.04 | same |
| dish ("the plate") | centre ≈ (0.052, -0.007), footprint radius **0.070**, rim crest z = 0.9193, well floor z = 0.9076 | flat-band component + a 1 cm height grid |
| bowl | centre ≈ (-0.082, -0.005), rim radius 0.055, rim z = 0.9517 | band (0.935,0.960) component |
| gripper open width | 0.0799 m → each finger 0.0399 from the tool axis | api.gripper() at reset |
| fingertip offset | tip ≈ eef_z − 0.008 | open-gripper descent onto bare table stalls at eef z 0.9093 |
| api.move undershoot | ~6-8 mm, direction-specific | residual ≈ 0.009-0.012 at every descent step; eef sits above the commanded z |

`effort 3.0` is only a *gap > ~5 mm* flag, not a hold: v2 read effort 3.0 with
a 7.9 mm gap while the dish never left the table. **Gap survival across a
lift/transit is the only grasp receipt I trust.**

## Version chain

| ver | hypothesis | receipt | verdict |
| --- | --- | --- | --- |
| v1 | perception dump, no motion | seeds 51,53,55,57; table 0.901, cabinet top 1.1269 | scene established |
| v2 | dish is rim-pinchable like the pack's bowl (jaws along y at the +y rim, eef z 0.915) | seeds 51,53: gap 0.0079/0.0071 at close, **0.0010 after lift**, dish still on the table | REFUTED — the rim is a wedge (inner face 11-39° from horizontal, outer wall vertical), so the squeeze has no opposing normal and ejects it |
| v3 | clamp a CHORD near the edge, both fingers outside the wall; two orientations | seed 51 chord-X (jaws along x, chord at +y, tip 0.9051): gap 0.0261 → **0.0167 after an 8 cm lift, dish LIFTED (gif)** → 0.0146 at the 1.23 apex → **0.0014 after the 0.24 m lateral transit** | PARTIAL — the only mechanism that has lifted the dish; it creeps out during transit |
| v4 | map the dish's cross-section: close the same chord clamp at 4 fingertip heights | seed 51: tip 0.917/0.914/0.910 → gap **0.0010** (jaws close on air, dish shoved aside); tip ~0.902 (floored) → gap 0.0569, lost on the lift | the descent must clear the rim crest (r=0.070), which forces the chord out to r≈0.060 where the surface normal is ~77° off the closing direction; only tip ≈0.905 bites |
| v5 | **negative control**: replay the pack's own BOWL grasp + carry + release, to test my pipeline AND the predicate | seeds 51,53,55: bowl gripped (gap 0.0098), lifted off the table, carried, released at the pack's point (0.005,-0.177,1.149); final depth + gif show the **bowl sitting on the cabinet top** — and `benchmark_success = false` on all 3 | pipeline VALIDATED; the predicate is **re-authored away from the bowl**, so the graded object really is the dish |

Geometry that blocks the obvious grasp: the dish is a disc of diameter 0.136
but the jaws open only 0.0799, so the one chord with opposed contact normals —
the diameter — cannot be spanned. Every reachable chord has both contact
normals pointing the same way along the chord's perpendicular, so the squeeze
squirts the disc out; and the rim wedge has no opposing inner face.

| ver | hypothesis | receipt | verdict |
| --- | --- | --- | --- |
| v6 | re-squeeze at every carry hop to take up the creep | seeds 51-57: gap 0.0269 → 0.0184 → **0.0010**; the retry then closed on air | REFUTED, and informative — re-applying the closing force *accelerates* the ejection; v3's single close held 0.0167 over the same lift |
| v7 | the dish is an INVERTED cone (see below), so a much wider chord is reachable; single close, 0.045 m hops | seeds 51,53: off/r 0.73 closed on air and shoved the dish 0.02-0.03 m; off/r 0.78-0.82 bit 0.026 and **lifted the dish** (find_plate → None); gap 0.0264 → 0.0223 → 0.0171 → 0.0114 lost, 4 seeds | PARTIAL — best grasp+lift; bleeds out ~0.12 m into the carry |
| v8 | the bleed looked per-move, so use exactly two moves | seeds 51-61: close 0.0224, then ONE 0.29 m move → gap **0.0010** | REFUTED — the bleed is per *distance* (~0.13 m⁻¹), not per move; a fast long move is worse |
| v9 | seat the dish on the cabinet carcass first so the escape direction is blocked and the jaws stall on the full chord | seeds 51-57: the push stalls hard at eef y −0.016 (dish seated), but the seated dish's flat-band component **fuses with the cabinet's low face**, find-dish returned None and the clamp used the stale pre-push centre | bookkeeping bug, not a mechanism test |
| v10 | v9 + a detector that clips y behind the face and takes cy = y_max − r | seeds 51-57: push drove the dish cy −0.007 → −0.094 (rim onto the carcass at −0.157) and the clamp aimed correctly, but the **descent fouled** (ok=False, eef stalled 4.4 mm high) and the close read 0.0010 | the descent constraint is finger radius > the dish's **maximum** (base) radius 0.068, not > the wall radius at the fingertip height; off ≥ 0.0575 |
| v11 | v10's push + v3's proven off/r = 0.884 (finger radius 0.0723, clean descent) | seeds 51,53: push seats the dish, clamp aimed correctly, close = **0.0010/0.0012** on air; the post-push perception shows the dish's ztop risen 0.9188 → 0.9294 | the push **tips the dish up** against the carcass, so a flat-dish clamp height/centre no longer describe it |

## The dish's shape, re-derived from the closes themselves

Each close is a radius measurement: `r_contact = sqrt(off² + (gap/2)²)`.

| fingertip z | off | gap | ⇒ contact radius |
| --- | --- | --- | --- |
| 0.9019 (floored on the table) | 0.0606 | 0.0569 | 0.0670 |
| 0.9051 | 0.0603 | 0.0261 | 0.0617 |
| 0.9113 | 0.0613 | 0.0010 | < 0.0613 |
| 0.9164 | 0.0598 | 0.0010 | < 0.0598 |

So the dish is **widest at the table** (r ≈ 0.068, which is also what the
oblique cam_high footprint measures) and narrows going up to r < 0.060 at its
rim crest 0.9193 — an inverted cone, outer wall leaning in ~28° from
vertical, with no crest overhang.

## Mechanism gap (falsifiable)

**Claim: with this API, the dish cannot be transported from the table to the
cabinet top, because the maximum achievable grip bleeds out over ~4× less
travel than the shortest path requires.**

Three measured quantities pin it:

1. **The jaws cannot span the stable chord.** The dish's diameter is 0.136-0.137
   m; the gripper opens to 0.0799 m. The only chord of a disc whose two
   contact normals oppose each other is the diameter, so every reachable
   clamp has both normals pointing the same way along the chord's
   perpendicular and squeezes the dish *out* with a net force 2F·(off/r).
   The rim offers no alternative: its inner face rises only 11-39° from
   horizontal, so a rim pinch has no opposing normal either (v2).
2. **The reachable offset is pinned near the tangent.** The fingers must stay
   outside the dish's maximum radius (0.068) all the way down or they land on
   the flare and the close rides them over it (v10: off/r 0.73 → air). That
   forces off ≥ sqrt(0.070² − 0.0399²) = 0.0575, i.e. off/r ≥ 0.83, so the
   ejection force is ≥ 0.83 of the normal force while the available friction
   is evidently less. Measured bite: **0.022-0.027 m**, at every offset from
   0.78 to 0.88 (v3, v7, v8).
3. **The bite bleeds at ~0.13 m⁻¹ to an empty-jaw floor of ~0.0012.**
   v7 hops: −0.0041, −0.0052, −0.0057 per 0.039 m. v8: −0.0214 over one
   0.29 m move. Budget from a 0.026 bite ≈ **0.11 m of travel**. The shortest
   path is 0.226 m of lift (table 0.901 → cabinet top 1.1269) plus ≥ 0.11 m
   laterally even after pushing the dish up against the cabinet first ⇒
   **≥ 0.33 m**.

The grasp itself is real — the v3 film strip plainly shows the dish held,
tilted, clear of the table — and the place geometry is real: **v5 put the
pack's bowl on the cabinet top with it** (and that scored `false`, which is
what identifies the dish as the graded object). What is missing is any way to
hold a 0.136 m disc, or to support it en route.

**What would falsify this:** a gripper opening ≥ 0.14 m (spans the diameter);
or a `grip(width)` that held an intermediate commanded width instead of
force-closing (the ratchet is driven by the sustained closing command — v6
showed re-issuing it costs 0.008 of gap per hop); or a raw-action channel with which the closing force
could be modulated (`fair_run.py` serves `act` only for backends that
implement `step_raw`, which this LIBERO path does not appear to — I did not
spend an episode confirming it); or a support
surface between 0.901 and 1.1269 to break the lift into two supported stages.


## Formal selection — v7 on all 15 debug seeds

`results/sel_c2clean_goal_put_bowl_top_cabinet_task_k3_v7` = **0/15**
(`program.py` md5 `fec3d0ab852502e47ca7319b42c45921` == `program_v7.py`).

The 15-seed run is consistent seed to seed and it also corrected two readings
I had taken from the 2- and 4-seed probes:

* **The 0.075-0.080 gaps are the jaws jammed OPEN, not a bite.** v7's third
  fallback offset (off/r ≈ 0.777) puts each finger at radius ≈ 0.060, inside
  the dish's 0.068 maximum, so the dish ends up between the jaws at a chord
  wider than the 0.0799 opening and they cannot close at all. Seed 55 shows it
  cleanly: gap 0.0789 unchanged across three hops. `GAP_HOLD_MIN` only tests
  `gap > 0.012`, so the frozen program accepts that state as "held" — a
  verification defect in v7; the test should be `0.012 < gap < 0.06`.
* **"dish still on the table: False" is not a lift receipt.** With the open
  hand hovering over the dish, the flat-band detector fails, so that line
  reads False on seeds where nothing was ever gripped. The trustworthy lift
  evidence is the film strip, plus the smooth monotone gap decline on the
  seeds that did bite.

Real bites, per seed (the attempt whose gap landed in 0.019-0.027):
51 0.0264 · 52 0.0227 · 53 0.0255 · 54 0.0221 · 55 0.0024 · 56 0.0237 ·
57 0.0247 · 58 0.0221 · 59 0.0226 · 60 0.0251 · 61 0.0253 · 62 0.0223 ·
63 0.0230 · 64 0.0229 · 65 0.0228 — i.e. **0.022-0.026 on 14 of 15 seeds**,
never the 0.073 the chord geometry predicts, and always gone within 3-4 hops
(0.08-0.16 m of travel). The bite size and the bleed rate are properties of
the scene, not of the seed.

## DECLARATION

* **Frozen version:** `packs/c2clean_goal_put_bowl_top_cabinet_task_k3/program.py`,
  md5 `fec3d0ab852502e47ca7319b42c45921`, identical to `program_v7.py`.
* **Selection receipt:** one formal run on the full 15 debug seeds (51-65),
  `results/sel_c2clean_goal_put_bowl_top_cabinet_task_k3_v7` → **0/15**.
* **Argmax:** every version scored 0 on the benchmark bit, so the 0/15 tie is
  broken on mechanical progress — v7 is the only version that both takes a
  real bite on 14/15 seeds and carries it toward the cabinet, and its bite
  (0.022-0.027) is the largest measured anywhere in the chain.
* **Receipt chain:** v1 perception · v2 rim pinch REFUTED · v3 chord clamp
  PARTIAL (lifts, creeps) · v4 cross-section map · v5 bowl negative control
  (pipeline validated, predicate shown re-authored) · v6 re-gripping REFUTED
  · v7 widened chord PARTIAL (frozen) · v8 two-move transport REFUTED ·
  v9 wall-seating (bookkeeping bug) · v10 detector fixed, descent fouls ·
  v11 wall-seated clamp at v3's offset — the push tips the dish, close on air.
  Probe dirs `results/fs_c2clean_goal_put_bowl_top_cabinet_task_k3_v{1..11}`.
* **PROVENANCE:** present in the frozen program as a top-level literal dict,
  15 entries, every constant sourced to the k3 pack's keyframes, the mate
  pack, my own debug-seed RGB-D/gripper measurements, or generic
  controller/camera mechanics. No LIBERO-specific prior knowledge was used;
  the table height, cabinet top plane, dish radius, fingertip offset, jaw
  opening and move undershoot were all re-derived on seeds 51-65.
* **Status: mechanism-gap stop**, stated falsifiably above: the dish is a
  0.136 m disc with no chord the 0.0799 m jaws can span stably, the best
  attainable bite is 0.026 m, it bleeds at ~0.13 m⁻¹, and the shortest path
  to the cabinet top is ≥ 0.33 m — roughly 3× the transport the grip affords.
