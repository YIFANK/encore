# c2clean / obj_alphabet_soup_pos_k0 — zero-demo cell

Intent: "pick up the alphabet soup and place it in the basket".
No demo pack. Everything below comes from debug seeds 51-65 and generic
camera/controller mechanics. Runner: `tools/fair_run.py` only.

## v0 — perception dump (seeds 51,53,55 then 51..65 odd)
Hypothesis: with no pack, the only way in is to look at the scene.
Program logs cam_high / cam_arm_wrist RGB-D as zlib+base64 chunks through
`api.log`, decoded offline.
Evidence: first attempt used 3000-char chunks; the logger truncates a message
at ~2000 chars, so every chunk lost ~1000 chars and the base64 would not
decode. Re-ran with 1800-char chunks (`fs_..._v0b`, 8 seeds) — clean decode.
Verdict: usable sensor pipe, zero sim steps.

Scene (cam_high, base frame; table plane z = 0.001):
- robot gripper, ztop 0.35
- basket: n~11500, ztop 0.144, rim bbox centre ~(0.01, 0.26), spans 0.159/0.170
- can A: ztop 0.081, top spans 0.063/0.070, mean B-R = **+7.4** (blue label,
  yellow letters on an orange band — letters => alphabet soup)
- can B: ztop 0.081, top spans 0.064/0.068, mean B-R = **-13.3** (red/green
  bands with tomatoes => not the soup)
- bottle ztop 0.148, milk carton ztop 0.142, small box ztop 0.019
Across all 8 probed seeds every prop cluster was pixel-identical; only the
basket moved (x 0.010-0.036, y 0.246-0.267). In this cell the `_pos`
perturbation visibly moves the destination, not the props — but the program
still perceives everything every episode, so it does not depend on that.

## v1 — reach + fingertip calibration (seeds 51,53)
Hypothesis: the unknown is the eef-to-fingertip offset and whether the far
can is reachable.
Evidence:
- `reach_above` (-0.149, 0.060, 0.15): residual 0.009, eef landed on target.
  Far can is reachable.
- open-finger press straight down onto bare table, commanded z = -0.06:
  stalled at **eef z = 0.00953** at (0.25,0.10) and again at (-0.13,0.23).
  => fingertips sit ~0.0095 below the eef reference.
- a single `api.move` with seconds=2.0 does not converge (0.035 m short in z);
  moves must be re-issued until the residual is small.
Verdict: TIP = 0.0095; use a `goto()` that re-issues until |eef-target|<tol.

## v2 — first full pick and place (seeds 51,53,55,57) → **4/4**
Grasp: top-down, R_DOWN = diag(1,-1,-1) passed explicitly on every move;
eef z = can_ztop - 0.020 + TIP (jaws bite 20 mm below the can top);
close; lift to 0.25 (can bottom then ~0.19, clears the 0.144 basket rim);
translate to the basket rim-bbox centre; open.
Evidence: closed width 0.0626 with effort 3.0 (= the can diameter), held all
the way to the release. 4/4 benchmark_success.

## v3 — hardening (seeds 51..65 odd) → **8/8**
Changes, all fallbacks that only fire when the primary path fails:
- band-mask fallback (`z in [0.055,0.095]`) if the whole-prop shape gate finds
  no can — survives a footprint fusion with a taller neighbour on eval seeds.
- grasp verification (`effort>1.0` and `0.03<width<0.078`) + one re-perceive
  and retry with a 12 mm deeper bite.
Verdict: no regression, 8/8.

## v3d — aim-envelope diagnostic (seeds 51,53,55,57) → 4/4
Hypothesis: how much perception error can the grasp absorb?
Injected a deliberate **+20 mm x aim error** on the first attempt.
Evidence: closed width 0.0592, effort 3.0, held, 4/4 success — the parallel
close drags the cylinder to centre, so a 20 mm aim error is still a grasp and
the retry path never had to fire. Verdict: the aim envelope is >= 20 mm, far
wider than any plausible cluster-centroid error; perception noise on the blind
eval seeds is not a live risk.

---

# DECLARATION

- **Frozen version:** `packs/c2clean_obj_alphabet_soup_pos_k0/program.py`,
  md5 `354bde8bd576edbe049b3beeaccadef9` == `program_v3.py` (same md5).
- **Selection receipt (formal, full 15 debug seeds 51-65):**
  `results/sel_c2clean_obj_alphabet_soup_pos_k0_v3` — **15/15**
  `"benchmark_success": true` out of 15 episodes.
- **Receipt chain:**
  - v0 / v0b `results/fs_..._v0`, `fs_..._v0b` — perception only, 0 successes by design
  - v1 `results/fs_..._v1` (2 seeds) — calibration only, 0 by design
  - v2 `results/fs_..._v2` — 4/4
  - v3 `results/fs_..._v3` — 8/8
  - v3d `results/fs_..._v3d` — 4/4 with a forced 20 mm aim error (diagnostic)
  - v3 formal `results/sel_..._v3` — **15/15**
- **PROVENANCE:** present as a top-level literal dict in program.py, covering
  Z_TABLE, WS_BOUNDS, CAN_SHAPE_GATE, BLUE_CAN_CUE, TIP_OFFSET, GRASP_DEPTH,
  BASKET_GATE, CARRY_Z. Every constant traces to a debug-seed measurement or
  generic camera/controller mechanics; no pack, no foreign source.
- No `api.done` read anywhere; no `fewshot_run.py`; writes confined to
  `packs/c2clean_obj_alphabet_soup_pos_k0/*` and `results/*c2clean_obj_alphabet_soup_pos_k0*`.

STOP.
