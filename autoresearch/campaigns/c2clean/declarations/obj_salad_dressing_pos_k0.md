# c2clean / obj_salad_dressing_pos_k0

Intent: *pick up the salad dressing and place it in the basket*
No demonstration pack. Everything below is derived from debug seeds 51-65 only.

---

## v1 — RGB-D datapipe probe (no task attempt)

**Hypothesis.** With no pack and no simulator surface, the only way to do real
perception is to get the camera frames off the box: zlib+base64 the RGB-D
arrays through `api.log` and deproject them offline, costing zero sim steps.

**Evidence.** `fs_..._v1`, seeds 51,53,57,61. Streams arrived but would not
decode: base64 length was not a multiple of 4. Chunk lengths came back 1984/1985
against a requested 3000 — **`api.log` truncates a message at ~2000 chars**, so
every chunk silently lost its tail.

**Verdict.** Datapipe sound, framing wrong. Refuted the chunk size, not the idea.

## v2 — datapipe fixed (no task attempt)

**Hypothesis.** Chunk at 1400 and carry a per-array `adler32` + length so the
decoder can *prove* the stream is intact rather than assuming it.

**Evidence.** `fs_..._v2`, seeds 51,53,55,57,59,61,63,65. All 16 arrays on all
8 seeds decoded with matching checksums. Depth sent as uint16 mm (half the
bytes, 1 mm resolution).

Scene, from the deprojected cloud (base frame):

- **Table top is z ≈ 0**: 212k of 262k points lie below z = 0.03.
- 7 components in `0.015 < z < 0.25`, `|x|,|y| < 0.45`.
- Zooming the cam_high crops reads the labels directly: the top-left bottle is
  **"Tomato Ketchup"**, the front-centre bottle is **"Creamy Ranch Dressing"**.
  Two bottle-shaped props; the instruction names one of them.
- **Cap colour separates them.** Greenness `G − max(R,B)` over each component's
  top-15 mm band: dressing **+0.081**, runner-up **−0.001**, rest ≤ −0.006.
  Identical on all 8 seeds. Height does not separate them (both ztop 0.148).
- **The target does not move; the destination does.** The dressing's component
  is byte-identical across all 8 probed seeds (npx 2571, same bbox); the basket
  centre ranges over ~2 cm. This `_pos` cell perturbs the basket, the milk
  carton and the butter box. *Not relied upon* — the program re-perceives both
  every episode.
- **Dressing shape profile** (y-width per z-slice): 0.063 body at z 0.03-0.05,
  tapering through a shoulder at z 0.07-0.10, then a **constant 0.035 neck/cap
  from z 0.10 to its top at 0.148**. Jaws open to 0.078, so the neck is a clean
  pinch.
- **View bias.** cam_high sits at (0.897, 0, 0.65) looking −x, so within a
  z-slice only a prop's +x face is seen: **y is unbiased, x is not**. Hence
  radius from the y extent and `x_centre = x_max − radius`. The basket rim is a
  closed ring seen from above, so its bbox midpoint *is* its centre.

**Verdict.** Perception settled. Target cue = argmax cap greenness (margin 0.082).

## v3 — first full attempt + two calibrations

**Hypothesis.** Pinch the measured neck at z = 0.120, carry, release over the
perceived rim centre. Two unknowns the brief does not give are measured in-run:
the jaw closing axis (diff the wrist view open vs shut) and the fingertip-to-eef
offset (stall a descent on the bare table, whose height we know is 0).

**Evidence.** `fs_..._v3`, seeds 51,53,55,57 → **4/4**.

- `TIP_OFFSET` = 0.0103 on every seed (commanded z = −0.06, stalled at 0.0103).
- Closed gap **0.0368** on a 0.035 neck at effort 3.0, and the width held to
  five decimals through lift *and* the whole carry — a real grip, not a graze.
- **Jaws close along base Y.** The open-vs-shut wrist diff moves only along
  image *u*; the wrist extrinsic maps +u to base −y. So the *unbiased* axis is
  the critical one and the *biased* axis is the forgiving one — lucky, and the
  reason v3's 9 mm x residual was harmless.
- Controller residual is real and pose-dependent: commanded grasp (0.150,0.029)
  landed (0.1406,0.029) — the descent alone swung x by −0.015, where the same
  command 10 cm higher landed x **+**0.0056.

**Selection run** `sel_..._v3`, all 15 debug seeds → **15/15**.

**Verdict.** Mechanism works. Two gaps left: uncancelled residual, no verification.

## v4 — closed-loop aim + verified retry

**Hypothesis.** Cancel the pose-dependent residual with a bounded 2-step servo
at the grasp height (and over the basket), and verify the grip after the lift
(effort 3.0 **and** a plausible closed gap) rather than assuming it, retrying
the grasp once if empty. Geometry unchanged from v3.

**Evidence.** `fs_..._v4`, 8 probe seeds → **8/8**. Servo converges as intended:
grasp x error 0.0087 → 0.0013, basket 0.0068 → 0.0032, landing (0.0083,0.2561)
against a perceived (0.008,0.256). No retry was ever needed on debug seeds.

**Verdict.** Strictly better aimed than v3 at equal score. See selection below.

## Aim-envelope probes (diagnostic only — never selection candidates)

15/15 says nothing about *margin*, and the eval seeds may place the bottle
where the debug seeds never do. So v4 was re-run with a deliberate offset
injected into the perceived grasp xy, 4 seeds each:

| x offset | result | | y offset | result |
|---|---|---|---|
| +0.015 | 4/4 | | +0.010 | 4/4 |
| −0.015 | 4/4 | | −0.010 | 4/4 |
| **+0.030** | **0/4** | | +0.016 | 4/4 |
| **−0.030** | **0/4** | | −0.016 | 4/4 |
| | | | **+0.025** | **4/4** |
| | | | **−0.025** | **4/4** |

Dirs `fs_..._env_{xp,xm,xp3,xm3,yp,ym,yp2,ym2,yp3,ym3}`.

**The envelope is ±25 mm or wider in y and breaks between 15 and 30 mm in x** —
the opposite of the naive expectation. y is wide *past the neck radius* (0.0175)
because the jaws close along y and sweep the neck to centre as they shut; x is
bounded instead by the finger plates' half-width, since an x error just slides
the neck off the end of the plates with no restoring motion.

This is what settles v4 over v3. Both score 15/15, but v3 leaves an uncancelled
−9.4 mm x residual, which spends most of a ~15-30 mm x budget on the *tighter*
axis before perception error is even counted. v4's servo cuts that to ~1.3 mm.
Against an x estimate whose two independent views (cam_high 0.1500, wrist at
hover 0.1515) agree to 1.5 mm, v4 keeps roughly a 10× margin on the binding axis.

---

## DECLARATION

**Frozen version: v4.**
`packs/c2clean_obj_salad_dressing_pos_k0/program.py`
md5 `b050972bfccd5948b0cc7d1ec94b8e45` == `program_v4.py` (verified on the
cluster and locally).

**Selection receipt (full 15 debug seeds, 51-65):**
**15/15** — `results/sel_c2clean_obj_salad_dressing_pos_k0_v4`

**Per-version receipt chain**

| ver | run dir | seeds | result |
|---|---|---|---|
| v1 | `fs_..._v1` | 51,53,57,61 | probe only — datapipe truncated, no attempt |
| v2 | `fs_..._v2` | 8 probe seeds | probe only — perception settled, no attempt |
| v3 | `fs_..._v3` | 51,53,55,57 | **4/4** |
| v3 | `sel_..._v3` | all 15 | **15/15** |
| v4 | `fs_..._v4` | 8 probe seeds | **8/8** |
| v4 | `sel_..._v4` | all 15 | **15/15** (selected) |

Plus 10 aim-envelope diagnostic runs (40 episodes), tabled above. They are
offset-injected copies of v4 (`probe_env_*.py`, each marked "not for selection")
and were never selection candidates.

**PROVENANCE**: present as a top-level literal dict, 11 entries, every
calibrated constant sourced to a debug-seed measurement or to generic
controller/camera mechanics. No pack was consulted (this is a k0 cell). No
`.done` attribute is read anywhere in the program.

**Mechanism summary.** Deproject cam_high; take connected components above the
table; pick the target as argmax cap greenness (+0.081 vs −0.001, the label
crops confirm "Creamy Ranch Dressing" vs "Tomato Ketchup") and the basket as
the largest component. Grasp the 0.035-wide neck at z = 0.120, taking the
radius from the unbiased cross-view y extent and x from the near-face tangent
minus that radius. Measure the fingertip offset each episode by stalling a
descent on the bare table. Servo out the controller's pose-dependent residual
at the grasp and over the basket; verify the grip by effort *and* closed gap
after the lift, retrying once; release over the perceived rim centre.

No mechanism gap. STOP.
