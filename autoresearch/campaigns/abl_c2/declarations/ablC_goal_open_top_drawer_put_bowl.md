# abl_c2 / ablC_goal_open_top_drawer_put_bowl — worker ledger

Variant C (**no verification**). One program version, written from the pack
alone, sent straight to the blind sealed eval. **Zero episodes run.**
`tools/fair_run.py` was never invoked, on any split, on any seed.

Intent: *"open the top drawer and put the bowl inside"*.

LAWS.md is empty for this cell and was not written to.

---

## Version 1 (the only version) — hypothesis → evidence → verdict

**Hypothesis.** The task decomposes into a fixed-pose drawer drag followed by a
perceived rim-pinch pick-and-place, and the whole thing can be written from the
pack because (a) the cabinet fixture is not re-randomised, so its hook pose can
be replayed in absolute base coordinates, and (b) the only object that moves
between layouts — the bowl — is recoverable from cam_high depth with a grasp
whose tolerant axis is the one perception is weakest on.

**Evidence assembled from the pack (no episode, no runner):**

1. *Phase structure.* All three demos share the same shape in `ee_path6`: a
   transit to (x≈0.03, y≈−0.09, z≈1.106) with jaws **open**, then a monotone
   **+y translation of 0.158–0.163 m at constant x and z**, then a descent to
   z≈0.917 where the gripper closes, then a carry to ≈(0.004, −0.048, 1.13)
   where it opens. Keyframe `demo0_t0100.png` shows the cabinet drawer already
   extended right after the +y segment ⇒ **the drawer slide axis is base +y and
   the +y segment IS the opening stroke.**

2. *The cabinet is a fixed fixture.* Row-by-row dark-mask extents of the
   cabinet silhouette in `demo{0,1,2}_t0000.png` agree to within ~2 px of 128
   (≲1 cm). Meanwhile the wooden rack, the white stove pad and the bowl all
   move visibly between the same three frames. ⇒ replaying the hook pose in
   absolute coordinates is sound; the 0.020/0.045/0.024 spread in demo hook x
   is teleoperator slack, and it also tells me the handle tolerates ≥±1.2 cm in
   x (consistent with a horizontal handle bar).

3. *The demonstrated grasp is a rim pinch, not an enclosure.* `gripper_state`
   while holding is (0.0022, −0.0025) ⇒ **total closed width 4.7 mm**. A bowl
   of ~7 cm across cannot be held at 4.7 mm by enclosure; the jaws are pinching
   a ~5 mm wall.

4. *The pinch is on the −y rim extreme.* Demo grasp y = 0.0715 / 0.025 / 0.0395
   with grasp x pinned at −0.107/−0.103/−0.107. Using the image scale fixed by
   the known 0.16 m drawer travel, the bowl blob in each `t0000` frame sits at
   larger y than the grasp point, by ~0.026–0.048 m — i.e. `grasp_y =
   centre_y − r`, r ≈ 0.037 (rim ≈ 0.074 m across). The opposite hypothesis
   (`centre_y + r`) predicts the bowl 15–20 px left of where it actually
   renders, and is refuted.

5. *That pinch is self-centring in y and fragile in x.* Parallel jaws closing
   symmetrically about a wall, with the inner jaw already inside the bowl and
   the outer jaw outside it, converge on the same wall from both sides; a y
   error merely slides the bowl until capture. The x error, by contrast, walks
   the contact point around the circle toward the tangent. ⇒ spend the
   perception budget on the bowl's centroid x and take y from the blob's 3rd
   percentile.

**What version 1 does.** Segment cam_high into a base-frame cloud (pinhole
convention auto-selected by agreeing with `api.deproject` on probe pixels,
with a per-pixel `deproject` fallback); replay the demo hook and drag; retract
to the demo t=0 arm pose (unoccluded in every keyframe) and re-segment the
cabinet to check that its +y face advanced by >0.055 m — a drawer check built
purely from my own depth, with one perception-guided re-hook if it did not;
then pick the tallest bowl-sized bright blob (height ranking, not area, so the
flat plate beside the bowl cannot win), pinch its −y rim wall at z=0.917,
confirm with `api.gripper()` effort before the carry, and release at a point
expressed **relative to where my own drag actually ended** so an over- or
under-pull does not put the bowl outside the drawer. Every perception step
degrades to a demonstration-mean constant on failure, and every move/grip call
is guarded, so no perception fault can abort the episode.

**Verdict.** UNVERIFIED BY CONSTRUCTION. This cell forbids running it, so the
hypothesis above carries no episode evidence — that absence is the measurement.
The program is frozen as written.

### Candidate law (falsifiable, receipt = this pack)
> *A demonstrated closed-gripper width far below the target object's diameter
> means the demonstration is pinching a wall, and the pinch point is offset
> from the object centroid by one radius along the jaw-closing axis. Recover
> the sign of that offset by checking the demonstration's grasp coordinate
> against the object's silhouette in the keyframe images, not by assuming the
> grasp is at the centroid.*
> Receipt: pack `gripper_state` (0.0022, −0.0025) vs a ~0.074 m rim; the
> centroid reading is refuted by 15–20 px in all three `t0000` keyframes.

---

## DECLARATION

- **Frozen program:** `packs/ablC_goal_open_top_drawer_put_bowl/program.py`
- **md5:** `eb8032b3ddb4a1430d2d2285a852450c`
- **`program.py` md5 == `program_v1.py` md5:** YES (verified on AbakaAI, both
  files `eb8032b3ddb4a1430d2d2285a852450c`)
- **Episodes run: 0** — `tools/fair_run.py` was never invoked, on any split,
  on any seed; seeds 1–50 and 51–65 were never touched. `tools/fewshot_run.py`
  was never invoked.
- **Program versions written: 1**
- **PROVENANCE present:** YES — top-level literal dict, **48 entries covering
  all 48 module-level calibrated constants** (checked by AST: every entry has a
  non-empty `source` and `allowed: True`; no constant is missing and no entry
  is orphaned).

### Static self-checks run (the permitted set only — source-text checks)
| check | result |
|---|---|
| `python -m py_compile program.py` | OK |
| forbidden-token scan (all 19 tokens) | none present |
| AST scan for any `.done` attribute read | 0 occurrences |
| `PROVENANCE` is a top-level literal dict | yes, 48 entries, 0 malformed |
| every module constant covered by PROVENANCE | yes (0 missing, 0 orphaned) |
| AST undefined-name scan | none |

No policy was executed anywhere — not under `fair_run`, not under any local
mock. Cluster interaction was limited to `scp` of
`packs/ablC_goal_open_top_drawer_put_bowl/` down and `program.py` /
`program_v1.py` up.

### What I would have probed first, had probing been allowed
(ordered by how much I expect the answer to move the eval number)

1. **Does the hook actually catch the handle?** The single highest-variance
   guess in the program. I replayed (0.030, −0.091, 1.106) but I never saw a
   handle — I do not know whether the jaws drop into a gap behind a bar, or
   straddle the bar, or land on top of it. One probe episode with a log of the
   drag residual answers it outright.
2. **Which world axis do the jaws close along at the default straight-down
   wrist?** My whole rim-pinch geometry assumes the closing axis is base y at
   the demonstrated orientation. If it is x, the correct pinch point rotates
   90° about the bowl and my grasp point is on the tangent — the worst case for
   this program. Trivial to settle with one grasp and a gripper-effort read.
3. **Bowl-vs-plate segmentation.** I rank candidate blobs by height because the
   plate is flatter, but I have never seen the depth image; if the plate's rim
   clears my threshold and out-ranks the bowl, the program picks up the wrong
   object with full confidence. One capture would have settled it.
4. **Whether the opening drawer sweeps the bowl.** I argued from the demo x
   offsets that the drawer's x-span misses the bowl by ~5 cm, but the demo0
   keyframes put the drawer's outer corner close to the bowl's footprint. If it
   does shove the bowl, my post-drag re-perception should cope — untested.
5. **Release height and drawer floor.** I chose z=1.132 from the two demos that
   released once, and read demo2's higher release (1.166) as the reason it had
   to re-pick. That is an inference from a single failure; 1 cm either way is
   the difference between a gentle set-down and a bounce-out.
6. **The step/time horizon.** I budgeted ~17 s of commanded motion and gate my
   retries on a 21 s budget, entirely by analogy with the 170/187/284-step
   demos. If the horizon is tighter than I guessed, the retries are what get
   truncated; if it is much looser, I under-used it.
7. **The pinhole-convention auto-select.** It checks itself against
   `api.deproject`, but I have never seen it choose.

### Clean-room self-audit
No `.bddl`/`.xml`/`.urdf`/`.hdf5`/init-state file was opened (the bddl path was
never touched at all). Nothing under `autoresearch/campaigns/c1|c2|c2fix/`, no
`packs/c1_*` or `packs/c2_*`, no `results/*c1_*` or `results/*c2_*`, no
`tools/probe_*.py`, and no other abl_c2 cell's pack, results or workspace was
read or listed. No LIBERO prior from memory or system context was used: every
constant traces to this cell's pack or to generic controller/camera/parallel-jaw
mechanics, as declared in PROVENANCE.

**STOP** — handing off to the coordinator for the blind sealed eval on seeds 1–50.
