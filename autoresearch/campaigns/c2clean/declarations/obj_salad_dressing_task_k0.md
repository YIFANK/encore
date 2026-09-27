# c2clean / obj_salad_dressing_task_k0 — worker notes

Intent: "Pick the tomato sauce and place it in the basket". No demonstration
pack; everything below is derived from debug seeds 51-65 only.

## v1 — perception probe (results/fs_..._v1, seeds 51,53,57,61)

Hypothesis: nothing yet; find out what is on the table.
Method: no motion. Dumped cam_high + cam_arm_wrist RGB-D through `api.log`
(zlib + base64, 1800-char chunks) and decoded them locally.

Evidence:
- cam_high K = [[618.04,0,256],[0,618.04,256],...], T_base_cam known; the
  support plane deprojects to z = 0.001 m, so **table z = 0**.
- Height-gated (z > 0.055 m) connected components, all four seeds:

  | prop (from RGB)      | xy (m)           | top (m) | cross-view width dy |
  |----------------------|------------------|---------|---------------------|
  | "Tomato Ketchup" bottle | (-0.107,-0.237) | 0.148 | 0.060 → 0.034 at cap |
  | red/green can (tomato sauce) | (-0.195,-0.082) | 0.081 | 0.061 |
  | blue/yellow can      | (-0.145,+0.057)  | 0.081   | 0.059 |
  | green-capped dressing bottle | (+0.062,-0.099) | 0.148 | 0.060 → 0.033 neck |
  | milk carton          | (+0.168,+0.028)  | 0.140   | 0.050 |
  | flat blue box        | (+0.110,-0.202)  | 0.020   | — |
  | basket               | (+0.02,+0.256)   | 0.143 rim | 0.155 x 0.15 |

Verdict: **the prop layout is fixed across seeds; only the basket shifts**
(y 0.246-0.263 over the seeds seen). Identity is legible from cam_high alone.

## v2 — calibration probe (results/fs_..._v2, seeds 51,53)

Hypothesis: the eef frame sits some unknown distance above the fingertips;
measure it by driving into an empty table spot.
Evidence: full-res cam_high dump reads the orange bottle's label as **"Tomato
Ketchup"**, so the *tomato sauce* is the open red/green **can** (top 0.081 m),
not the bottle. Blocked-descent numbers were ambiguous (the arm kept creeping
to eef z 0.009 while xy drifted 6 cm), so no offset was fixed here.
Also: open width 0.0796 m, closing on air reads 0.001 m at effort 0.05.

Verdict: identity settled; offset not settled.

## v3 — first pick-and-place (results/fs_..._v3, seeds 51,53,55,57) — 0/4

Hypothesis: fingertips ≈ 0.076 m below the eef (from v2's table contact).
Evidence: perception ran correctly (target selected at (-0.200,-0.081)), but
the close at eef z = 0.130 read **width 0.001 / effort 0.05** — air. The
post-episode height map shows every prop still in place.
Verdict: refuted — the offset is much smaller than 0.076 m.

## v4 — closing staircase (results/fs_..._v4, seeds 51,53)

Hypothesis: bracket the offset by closing the gripper at descending eef
heights over the can and watching the closed width.
Evidence (ep51): closes at eef z 0.127/0.118/0.111/0.099/0.091 all read
0.0010 m (air); eef z **0.0792 read 0.0621 m at effort 3.00**, the lift held
it (POST shows the can gone from the table and a 0.250 m tall blob in the
gripper).
Verdict: with the can top at 0.081 m, **the fingertips sit ~0.006 m below the
eef**. Grasp mid-body at fingertip z = 0.044 → eef z = 0.050.

## v5 — tomato-sauce can → basket (results/fs_..._v5, seeds 51,53,55,57) — 4/4

Mechanism: perceive → select the short (0.055-0.105 m top) component with the
reddest body band (z 0.02-0.07: red can mean(r-b) = +0.180, blue can = -0.123)
→ open, hover, descend to eef z 0.050, force-close, verify width/effort → lift
to z 0.30 (the can bottom then rides 0.25 m up, clear of the 0.143 m basket
rim) → translate over the basket's top-slab bbox centre → release.

## v5d — control: same mechanism, salad dressing instead — 0/4

Hypothesis under test: the bddl filename says *salad dressing* while the
instruction says *tomato sauce*; which one does the environment's own bit
grade?
Evidence: v5d grasped the dressing bottle by the neck (CLOSED w 0.0371, effort
3.00), carried it and released it over the basket; the post-episode height map
shows the bottle gone from the table and the basket's top risen from 0.144 to
0.162 m — the place genuinely succeeded — and the benchmark bit was **false on
all 4 seeds**, while v5 (the can) was **true on all 4**.
Verdict: **the instruction names the graded object.** This is a re-authored
cell: the bddl filename is stale, the sentence is authoritative.

## v6 — wrist re-centring + guarded retries

Hypothesis (from the v5 aim-envelope probe below): the grasp envelope is
geometric and tight, so a biased cam_high estimate is the main risk on unseen
layouts; a straight-down wrist view from 0.30 m should remove that bias.

Aim-envelope probe on v5 (results/fs_..._m010, m015, seeds 51,53,55,57):
displacing the grasp aim by **+10 mm in y gives 0/4** (m015 likewise) — the can
is 0.061 m across in a 0.0796 m jaw, so the whole tolerance is ±5 mm.

v6 mechanism = v5 plus: hover at 0.30 m over the coarse estimate, re-perceive
with cam_arm_wrist, take the nearest component within 0.07 m and re-aim at its
top-slab bbox centre; then a guarded grasp that checks width > 0.020 m **and**
effort > 1.0 after the lift, with up to 3 attempts and a cam_high re-perception
between attempts.

Evidence:
- results/fs_..._v6 (51,53,55,57): **4/4**.
- results/fs_..._v6b010, same seeds, with **+10 mm injected into the cam_high
  estimate**: **4/4** — where v5 scored 0/4. ep51 log: `REFINE (-0.200,-0.071)
  -> (-0.199,-0.080) d=0.0106`, then a first-attempt grasp (w 0.0616, eff 3.00).
Verdict: the wrist refine recovers a 10 mm perception error; keep it.

## v7 — frozen version

Identical to v6 except that one leftover unused constant (`GRASP_FRAC`) was
deleted so that every module constant is declared in PROVENANCE. Re-ran the
formal selection rather than inherit v6's receipt.

Selection: **results/sel_c2clean_obj_salad_dressing_task_k0_v7 — 15/15** on the
full debug band 51-65, no failures, and all 15 grasped on the first attempt
(`AT_GRASP[t0]` x15, no retry ever fired).

# DECLARATION

- **Frozen version: v7.** `packs/c2clean_obj_salad_dressing_task_k0/program.py`
  md5 `05d22b9b8b5ff92d15c0d33cefbc5933` == `program_v7.py` (same md5).
- **Selection receipt: 15/15** on seeds 51-65,
  `results/sel_c2clean_obj_salad_dressing_task_k0_v7`.
- **PROVENANCE present**, 13 entries, covering every module constant
  (R_DOWN, TABLE_Z, WORKSPACE, Z_OBJ_FLOOR, SHORT_BAND, RED_CUE, FINGER_OFFSET,
  HOVER_Z, REFINE_R, MAX_TRIES, GRASP_FINGERTIP_Z, HOLD_W, CARRY_Z). Every one
  is a debug-seed measurement or generic controller/camera mechanics; no pack
  was issued for this cell and none was read.
- **Receipt chain** (all probes on debug seeds only; eval seeds 1-50 never
  touched):

  | version | seeds | result | what it established |
  |---------|-------|--------|---------------------|
  | v1 | 51,53,57,61 | perception only | scene inventory; table z = 0; layout fixed across seeds |
  | v2 | 51,53 | calibration | "Tomato Ketchup" label ⇒ the *tomato sauce* is the red/green can |
  | v3 | 51,53,55,57 | 0/4 | fingertip offset of 0.076 m refuted (closed on air) |
  | v4 | 51,53 | staircase | fingertips ≈ 0.006 m below the eef; grasp at eef z 0.050 |
  | v5 | 51,53,55,57 | **4/4** | can → basket works |
  | v5d | 51,53,55,57 | **0/4** | control: the *dressing* delivered into the basket still scores 0 |
  | v5 (formal) | 51-65 | 15/15 | — |
  | m010 / m015 | 51,53,55,57 | 0/4 | aim envelope is ±5 mm |
  | v6 | 51,53,55,57 | 4/4 | wrist refine + retries |
  | v6b010 | 51,53,55,57 | 4/4 | recovers a 10 mm injected aim bias |
  | **v7 (formal)** | **51-65** | **15/15** | frozen |

- **Finding worth carrying out of the cell:** this is a re-authored cell — the
  bddl filename says *salad dressing*, the instruction says *tomato sauce*, and
  the v5/v5d pair shows the **instruction names the graded object**: delivering
  the dressing into the basket, verified by re-perception, scores 0/4, while
  delivering the tomato-sauce can scores 4/4 and 15/15.
