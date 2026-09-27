# c2clean / obj_butter_pos_k0 — notes

Intent: "pick up the butter and place it in the basket". No demo pack (k0).
Everything below is derived from my own debug-seed (51-65) observations.

## v1 — RGB-D dump probe (4 seeds)
Hypothesis: I can do perception offline by piping RGB-D through `api.log`.
Evidence: worked, but `api.log` truncates each message at exactly 2000 chars,
so 3000-char base64 chunks came back mutilated (b64 decode error).
Verdict: mechanism OK, chunk size must be <= ~1900.

## v2 — same dump, 1900-char chunks (8 seeds: 51,53,...,65)
Evidence: clean decode. cam_high is a 512x512 view from base (0.897, 0, 0.650)
looking back along -x; +x = toward the camera (bottom of image), +y = image
right. Depth histogram over the workspace crop: 141k points in z=[0.000,0.010)
= table plane (Z_TABLE ~ 0.005); prop band 0.012-0.15; a second band at
0.25-0.40 = the robot arm.

Height-map clustering (6mm cells, 8-connected) finds 7 props on every seed:

| id | ctr (x,y)        | top   | footprint    | mean rgb            | what |
|----|------------------|-------|--------------|---------------------|------|
| C0 | (0.01..0.04, 0.25..0.27) | 0.141 | 0.157 x 0.171 | 0.55/0.55/0.53 | basket |
| C1 | (0.064, -0.101)  | 0.081 | 0.064 x 0.070 | 0.27/0.24/0.20 | can |
| C2 | (-0.130, 0.059)  | 0.139 | 0.040 x 0.053 | 0.36/0.26/0.12 | carton |
| C3 | (-0.185, -0.079) | 0.148 | 0.033 x 0.063 | 0.32/0.24/0.21 | pale bottle |
| C4 | (0.109, -0.198)  | 0.029 | 0.080 x 0.048 | 0.28/0.22/0.20 | flat box A |
| C5 | (-0.10..-0.12, -0.23..-0.24) | 0.113 | 0.027 x 0.048 | 0.23/0.10/0.03 | dark bottle |
| C6 | (0.150..0.155, 0.026..0.033) | 0.019 | 0.075 x 0.039 | 0.41/0.25/0.18 | flat box B |

Verdict: across all 8 debug seeds ONLY the basket (C0, ~2.5cm of travel), the
dark bottle (C5) and the butter (C6, ~5mm) move; C1-C4 are identical to 3
decimals. The layout is near-fixed on the debug split, so I must NOT key on
position — the eval split may perturb more.

## v3 — wrist-camera survey of the two flat boxes (2 seeds)
Hypothesis: a close-up from `cam_arm_wrist` resolves which flat box is butter.
Evidence: hovered at z=0.16 over each flat cluster and dumped the wrist RGB.
  - C4 reads "CHOCOLATE PUDDING"
  - C6 reads "FARM FRESH BUTTER"  <-- the target
Verdict: **the butter is C6**: the thinner flat box (top 0.019 vs 0.029) and by
far the redder one (R-B = 0.233 vs 0.082). The identification rule I will ship
is appearance-based, not positional: among clusters with top < 0.060 and
footprint < 0.12, take argmax of (R - B).

Side evidence: `api.move` under-shoots. Commanding (0.155, 0.029, 0.160)
landed the eef at (0.145, 0.028, 0.153) — 10mm short in x, 7mm low — while the
returned residual read 0.009. So the returned residual is not the standing
error; I have to close the loop on `api.eef()` myself.

## v4 — calibration + full pipeline (running)
Hypothesis: the eef-to-fingertip offset can be measured in-episode by
descending closed fingers onto a bare patch of table and reading the stall
height; then grasp the butter at mid-box-height and drop it over the basket.

Evidence (4 seeds, 51/53/55/57): 3/4 succeeded. The whole pipeline works.
The probe measured the fingertip offset at 0.0042-0.0044 on every seed
(closed fingers stall at eef z=0.0092-0.0094 against a table plane at 0.005),
and the grasp closed to width 0.0389 == the box's measured short side 0.038.
ep55 failed by horizon exhaustion: after the probe the arm FROZE at
(0.167, 0.033, 0.169) and never moved again, while my unbounded
bias-cancelling loop kept amplifying the command out to (-0.238, 0.672).
Verdict: the in-episode probe is both unnecessary (the offset is constant) and
dangerous; the servo loop needs a bound and a freeze detector.

## v5 -- probe removed, servo bounded, grasp retry added
Hypothesis: with the probe replaced by the measured constant
FINGERTIP_OFFSET=0.0043, a clipped feed-forward correction and a freeze
detector, the pipeline is both shorter and safer.
Evidence: probe subset (8 seeds) 8/8 at ~207 of 1000 sim steps.
FORMAL SELECTION RUN, all 15 debug seeds: **15/15**
  results/sel_c2clean_obj_butter_pos_k0_v5
Verdict: mechanism solved on the debug split.

## v6probe -- reach envelope at grasp height (not a candidate)
Hypothesis: the debug split pins the butter at (0.15, 0.03) on every seed, so
I have only tested one aim point; the blind eval split may move it.
Evidence: swept 30 targets at z=0.012 with the wrist straight down.
x <= 0.10 reaches the whole y span [-0.30, 0.30]; x=0.20 is fine for
|y| <= 0.15 but clips to |y|~0.23 at |y|=0.30; x=0.28 never arrives -- the eef
saturates at x=0.212-0.216 and then stayed frozen there for every later target
in the sweep.
Verdict: there is a hard envelope at x~0.216, and saturating against it looks
like a permanent wedge.

## v7probe -- is a saturated arm actually wedged? (not a candidate)
Hypothesis: the v6probe freeze is recoverable, not terminal.
Evidence: deliberately drove into (0.30, -0.30, 0.012) three times (eef parked
at (0.174, -0.302, 0.011)), then a straight-up move to z=0.25 freed it
(eef z 0.229), a central pose and two normal grasp poses all followed.
Verdict: saturation is not a wedge -- it is repeated saturation against the
same unreachable target. A straight-up retreat is a valid recovery.

## v8 -- v5 + envelope clamp + freeze recovery  [FROZEN]
Hypothesis: clamping targets to x <= 0.21 and retreating-then-retrying on a
genuine freeze protects the eval split without changing debug behaviour.
Guarded so it cannot hurt: the retreat fires only when the eef truly stopped
moving (moved < 1mm) AND the standing error is > 15mm, so a descent that is
merely short of tolerance -- or one stalled on contact -- is never abandoned.
Evidence: FORMAL SELECTION RUN, all 15 debug seeds: **15/15**
  results/sel_c2clean_obj_butter_pos_k0_v8
sim_steps per episode are identical to v5's (208/209/208/207/207/206/207/210/
207/206/206/207/208/209/206), i.e. neither guard fires on the debug split.
Verdict: strict superset of v5; freeze this one.

## v9probe -- grasp aim margin (not a candidate)
Hypothesis: 15/15 says nothing about how much perception error the grasp
tolerates; displace the aim to find the edge.
Evidence: v8 with the grasp aim displaced +15mm along y (the jaw-closing axis)
scored 4/4 on seeds 51/55/59/63, and the closed width was still 0.03894 --
identical to the undisplaced grasp, so the close self-centres on the box.
Verdict: the aim margin is at least 15mm against a per-seed perception spread
of <= 3mm (a 5x margin). Geometrically consistent: the jaws open to 0.080 and
the box short side is 0.039, giving 0.0205 of half-clearance.

---

# DECLARATION

**Frozen version: v8.**
- `packs/c2clean_obj_butter_pos_k0/program.py`
  md5 `f4294090709f8c737a07662fd159563e`
  == `packs/c2clean_obj_butter_pos_k0/program_v8.py` (same md5, verified on the
  cluster).
- **Selection receipt: 15/15 on the full 15 debug seeds (51-65)**, directory
  `results/sel_c2clean_obj_butter_pos_k0_v8`.
- PROVENANCE: present as a top-level literal dict in program.py, covering every
  calibrated constant. Sources are debug-seed observations (cam_high RGB-D,
  eef readings, gripper width, the v3 wrist survey, the v4 table probe, the
  v6probe reach sweep, the v7probe recovery test) plus generic
  controller/camera/occupancy-grid mechanics. No pack was given and none was
  read; no foreign constants.

## Receipt chain
| version | what changed | run | result |
|---------|--------------|-----|--------|
| v1 | RGB-D dump probe | fs_..._v1 (4 seeds) | chunking bug found (api.log truncates at 2000 chars) |
| v2 | 1900-char chunks | fs_..._v2 (8 seeds) | scene mapped: 7 props, layout near-fixed on debug |
| v3 | wrist close-up survey | fs_..._v3 (2 seeds) | butter identified by label ("FARM FRESH BUTTER") |
| v4 | calibration + full pipeline | fs_..._v4 (4 seeds) | 3/4; fingertip offset 0.0043; ep55 arm freeze |
| v5 | probe removed, servo bounded | fs_..._v5 (8 seeds) | 8/8 |
| v5 | (selection) | **sel_..._v5 (15 seeds)** | **15/15** |
| v6probe | reach sweep | fs_..._v6probe | envelope saturates at x~0.216 |
| v7probe | wedge recovery test | fs_..._v7probe | straight-up retreat frees the arm |
| v8 | + envelope clamp, freeze recovery | **sel_..._v8 (15 seeds)** | **15/15**, guards never fire |
| v9probe | aim displaced +15mm in y | fs_..._v9probe (4 seeds) | 4/4 -- margin >= 15mm |

## How it works (mechanism)
1. `cam_high` RGB-D -> base-frame cloud -> crop the workspace (the uncropped
   cloud runs out to x=-1.99 on the walls) -> height-gate to 0.012-0.22 ->
   6mm occupancy grid -> 8-connected components. 7 props, every seed.
2. **Butter** = among clusters that are flat (top < 0.060; the two flat boxes
   top at 0.019 and 0.029, the next tallest prop at 0.081) and small
   (< 0.12 across), the one with the largest redness R-B. The butter reads
   0.234, the chocolate pudding 0.082 -- the rule is appearance-based, so it
   survives the positional perturbation. The wrist-camera survey in v3 is what
   licensed it: I read the labels off both boxes rather than guessing.
3. **Basket** = the only cluster wider than 0.12 in both axes (0.157 x 0.171;
   every other prop is under 0.081 wide).
4. Grasp: hover at 0.16, descend to `top/2 + 0.0043` (mid-box height plus the
   measured fingertip offset), close, lift to 0.22. Holding is verified from
   the gripper's own width (0.025 < w < 0.055; a good grasp reads 0.0389 == the
   box short side, an air-close reads 0.001, a failed close 0.080) -- never
   from any success signal. One retry on a failed hold.
5. Place: over the basket at 0.28, down to `rim + 0.0043 + 0.02`, open.
6. Every motion goes through a closed-loop `goto`: `api.move` under-shoots by
   ~10mm and the residual it returns is NOT the standing error, so the error is
   re-measured from `api.eef()` and fed forward with a clipped correction.

## Caveats for the eval split
- The debug split is nearly a fixed layout (only the basket, the dark bottle
  and the butter move at all, and the butter by only ~5mm), so the identity
  rule was validated against appearance, not against 15 genuinely different
  positions. The aim margin (>= 15mm) and the reach envelope (x <= ~0.216) are
  measured, and both guards in v8 exist for positions I could not produce.
- The known residual risk is cluster fusion: if an eval seed abuts the butter
  against another prop, the footprint clustering would merge them and both the
  centre and the redness would be contaminated. Nothing on the debug split
  produced that case, so I could not test a de-fusing rule against it.
