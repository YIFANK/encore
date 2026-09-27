# c2clean / goal_open_top_drawer_put_bowl_task_k3

Intent: **"Open the top layer of the drawer and put the cream cheese inside"**

## What the two packs say

- `..._task_k3` — language `open the top drawer and put the bowl inside`.
  Same TARGET (the top drawer), different object (a bowl). Gives the drawer
  mechanism.
- `..._task_mate` — language `put the cream cheese in the bowl`.
  Same OBJECT (the cream cheese), different target. Gives the grasp.

Both packs are shot in the SAME scene (identical keyframe layouts): dark
cabinet with two drawers at image-left, chopping board, wine bottle, grey
bowl, pink-rimmed plate, stove, and a small navy-blue box = the cream cheese.

### Mechanism read off the k3 pack (ee_path6 + keyframes)
1. **Drawer**: gripper stays OPEN (`gripper_cmd = -1`) through the whole
   opening phase. The eef descends onto the handle at
   `(x ~ 0.02-0.05, y ~ -0.09, z ~ 1.105)` and then translates **+y at
   constant z** to `y ~ +0.07`. So handle travel ~= 0.16 m in +y.
2. **Grasp**: closes at eef `z ~ 0.913` (bowl in k3, cream cheese in mate:
   0.9104 / 0.9205 / 0.9106).
3. **Release into the drawer**: `(0.018,-0.046,1.135)`, `(-0.013,-0.043,1.130)`,
   `(0.006,-0.055,1.166)`. NOTE: this point rides with the drawer, so it is
   only valid when the drawer has travelled the full ~0.16 m.

## Version log

### v1 — perception probe (no motion). Receipt: `fs_..._v1`, seeds 51,53,55
Hypothesis: I need the deprojection convention and the scene geometry before
I can command anything.
Evidence:
- `f.deproject` (NOT `api.deproject`) matches the plain OpenCV pinhole
  `p_cam = ((u-cx)z/fx, (v-cy)z/fy, z)` followed by `t_base_cam`, exactly.
- cam_high K = 618.039 / 256, `t_base_cam` translation (0.6586, 0, 1.6104);
  base **+y == camera +u**, so image-left is -y.
- Height map (seed 51): **table top z = 0.900**, **cabinet top z = 1.13** over
  `x in [-0.10,0.175]`, `y in [-0.35,-0.15]`; a **handle protrusion at
  z ~ 1.10, y ~ -0.1375, x in [-0.01,+0.09]**.
- The blue-box mask `b>r+10 & b>g+5 & b>25` isolates the cream cheese alone on
  the 128 px pack keyframes (51 px) but picks up ~7.4k px at 512; it needs a
  table-height gate.
Verdict: convention fixed, geometry fixed. Proceed to a full attempt.

### v2 — full pipeline, pack-absolute drop. Receipt: `fs_..._v3` → **0/4**
(seeds 51,53,55,57; dir `fs_c2clean_goal_open_top_drawer_put_bowl_task_k3_v2`)
Hypothesis: replicate the k3 pull verbatim, grasp the blue box at the mate
pack's z, release at the k3 pack's absolute drop point.
Evidence (seed 51):
- The **pick is solved**: closing at eef z 0.912 on the perceived blue cluster
  gives width 0.042 / effort 3.0 — the mate pack closed at 0.043.
- The **pull under-travels**: hooking at eef y = handle_y + 0.047 (the demo
  offset) moved the handle only from y=-0.1375 to y~-0.05, i.e. **0.09 m**,
  against the demos' 0.16 m. The far jaw sits at eef_y - 0.039, so at the
  demo offset it lands at the handle bar's *front* edge and slips off.
- Because the drawer under-travelled, the pack's absolute drop point
  (0.004,-0.048) was no longer over the interior; the gif shows the box landing
  on the table beside the grey bowl (and knocking it over).
Verdict: two named defects — (a) shallow hook slips, (b) the drop point must be
perceived, not copied. Both are fixed in v3.

### v3 — perceived handle, deeper hook, closed-loop pull. Receipt: `fs_..._v3` → **0/4**
Evidence: hooking at eef y = handle_y + 0.030 gave **0.000 m** of travel on all
four seeds. The `find_interior` detector also fired on the closed cabinet (it
was reading the cabinet's vertical front face), so the box was dropped on the
cabinet top.
Verdict: a deeper hook is not the answer, and the interior detector was wrong.

### v4 — geometric under-hook + press-drag fallback. Receipt: `fs_..._v4` → **0/2**
Evidence: descending to z=1.052 in front of the ledge **wedged the arm** at
(0.077,-0.072,1.095); every later move returned the identical eef and the
episode burned all 1000 sim steps. A blocked `api.move` burns its whole budget.
Verdict: the arm cannot work below the 1.13 cabinet top. Added a jam guard.

### v5 — no-motion FRONT-PROFILE probe (max y per (z,x) cell). Receipt: `fs_..._v5`
This is the measurement that unlocked the cell. A top-down max-z map hides
anything that is not the tallest thing in its cell; projecting the other way
shows the drawer front in elevation:
```
cabinet front face   y = -0.160 at every height
cabinet top plateau  z = 1.13, front edge y = -0.157
TOP drawer handle    protrudes to y = -0.130, z in [1.083,1.098], x in [-0.02,+0.10]
2nd / 3rd handles    same shape at z ~ 1.020 and z ~ 0.950
```

### v6 — v2's drag + perceived drop. Receipt: `fs_..._v6` → **0/4**, travel 0.000
### v7 — controlled A/B of v2's hook x and hop speed. Receipt: `fs_..._v7` → travel 0.000, 0.000
Hypothesis: v2's 0.090 m came from its hook x (0.081) or its 1.5 s hops.
Evidence: both trials, on two seeds, reproduced v2's eef trace to within a few
mm (hook at eef y -0.087, z 1.112) and moved the drawer **0.000 m**.
Verdict: v2's 0.090 m was a knife-edge contact, not a mechanism. Position-hold
dragging does not work.

### v8 — under-hook from below the U-bar. Receipt: `fs_..._v8` → **0/4**, travel -0.002
The film-camera frames show the handles are U-bars on end posts, so a hook
should exist. But the slide toward the face at z=1.045 moved **1 mm** (err
0.077) — the same refusal as v4. Confirms: no approach below the cabinet top.
The (z,y) occupancy probe did confirm the bar fills z in [1.08,1.095],
y in [-0.152,-0.125], with z in [1.03,1.075] empty underneath.

### v9 — **the mechanism**: drag under a sustained downward press
Hypothesis from the k3 pack's RAW ACTIONS: through the whole pull phase
(demo0 t=34..55) the command is `dy ~ +0.70` (near saturation) with
`dz ~ -0.60..-0.25` — hard DOWN — while the eef z holds constant at 1.105.
That -dz is not motion, it is **force**. Every earlier version commanded a
position at the contact height, which converges with ~zero contact force.
Fix: seat the far jaw on the bar, then command each +y waypoint 0.060 BELOW the
seated height so the controller keeps pressing.
Receipt (`fs_..._v9`, seeds 51,53,55,57): **travel 0.167 / 0.241 / 0.160 / 0.250**
and the eef tracked its own +y ladder to within 5 mm — the drawer now follows
the hand 1:1. Verdict: mechanism found.
Defect: seven pressing hops ate all 1000 sim steps before the pick ran (a move
that cannot converge costs `seconds x ~100` steps), so 0/4 on the benchmark bit.
(A first launch of v9 also had a non-terminating pull loop that polled achieved
y; killed by exact PID and replaced with a fixed waypoint ladder.)

### v10 — same travel in four hops, trimmed tail. Receipt: `fs_..._v10` → **3/4**
seeds 51 ✓, 53 ✓, 55 ✗, 57 ✓; travel 0.159 on all four; sim_steps 866/865/1000/865.
Seed 55's loss is diagnosed, not mysterious: its cream cheese sits at y=0.113,
only ~0.08 in front of the opened drawer's front wall, and the straight-down
descent stalled at z=0.989 instead of 0.912, so the jaws closed on air
(width 0.001, **effort 0.05** where a real hold reads 3.0) and the program
carried nothing to the drawer.

### v11 — verified grasp with a +y re-approach
Hypothesis: the grasp failure is detectable with my own sensor (effort) and
recoverable by staging the approach 0.075 further out in +y, away from the
opened drawer. Also removes v10's final retreat move, which stalled on every
seed and cost ~120 steps for nothing.
Receipt: pending.

Receipt (`fs_..._v11`, seeds 51,53,55,57,59,61,63,65): **6/8**. The retry itself
worked on both losses (55, 63: width 0.042 / effort 3.0 after the first descent
stalled ~0.077 high and closed on air), but both then ran out of episode —
sim_steps 1000 against 862-866 on the wins — so the lift out of the grasp never
ran. Verdict: mechanism right, budget wrong.

### v12 — v11 with the step cost trimmed. **FROZEN**
Hypothesis: nothing about the mechanism needs to change; a move that cannot
converge costs `seconds x ~100` sim steps, so shortening exactly those buys the
retry its room. Seat 1.1 → 0.7 s, the four pressing hops 1.0 → 0.8 s, the
post-release settle 0.6 → 0.3 s, the retry's three moves 1.3/1.2/1.0 →
1.1/1.0/0.9.
Evidence: probe (seeds 51,53,55,57,59,61,63,65) **8/8**, sim_steps 718-722 on
the clean seeds and 920 on the two that need the retry — ~80 steps of margin.
Selection: **15/15**.

---

# DECLARATION

**Frozen version: v12.**
`packs/c2clean_goal_open_top_drawer_put_bowl_task_k3/program.py`
md5 `4fd2bf38b0180ec8fbca785631ee510b` == `program_v12.py` (same md5, verified on
the cluster).

**Selection receipt (full 15 debug seeds, one formal run): 15/15**
dir `results/sel_c2clean_goal_open_top_drawer_put_bowl_task_k3_v12`
```
51 T 719   52 T 718   53 T 718   54 T 719   55 T 920
56 T 730   57 T 718   58 T 721   59 T 721   60 T 874
61 T 718   62 T 720   63 T 920   64 T 723   65 T 722
```
(no seed reached the 1000-step cap)

**Per-version receipt chain**

| ver | what changed | receipt | result |
|-----|--------------|---------|--------|
| v1  | no-motion probe: deprojection convention, scene geometry | `fs_..._v1` (51,53,55) | cv pinhole confirmed; table 0.900, cabinet top 1.13 |
| v2  | pack drag + pack-absolute drop | `fs_..._v2` (51,53,55,57) | 0/4; travel 0.090 once, drop missed |
| v3  | deeper hook + closed-loop pull | `fs_..._v3` (51,53,55,57) | 0/4; travel 0.000 |
| v4  | geometric under-hook | `fs_..._v4` (51,53) | 0/2; arm wedged, 1000 steps burned |
| v5  | no-motion FRONT-PROFILE probe | `fs_..._v5` (51) | handle located: ledge to y=-0.130, z 1.083-1.098 |
| v6  | v2 drag + perceived drop | `fs_..._v6` (51,53,55,57) | 0/4; travel 0.000 |
| v7  | controlled A/B of v2's hook x and hop speed | `fs_..._v7` (51,53) | travel 0.000, 0.000 — v2's 0.090 was luck |
| v8  | under-hook from below the U-bar | `fs_..._v8` (51,53,55,57) | 0/4; travel -0.002, slide refused |
| v9  | **drag under a standing downward press** | `fs_..._v9` (51,53,55,57) | travel **0.167/0.241/0.160/0.250**; 0/4 on steps |
| v10 | same travel in 4 hops, trimmed tail | `fs_..._v10` (51,53,55,57) | **3/4**; 55 lost to a blocked descent |
| v11 | verified grasp + `+y` re-approach | `fs_..._v11` (8 seeds) | **6/8**; retry works, budget overruns |
| v12 | step cost trimmed | `fs_..._v12` (8 seeds) → `sel_..._v12` (15) | **8/8** → **15/15** |

**PROVENANCE**: present in `program.py` as a top-level literal dict with 14
entries. Every constant is sourced to one of the two named packs
(`c2clean_goal_open_top_drawer_put_bowl_task_k3`,
`c2clean_goal_open_top_drawer_put_bowl_task_mate`) or to a debug-seed (51-65)
measurement logged in this cell's own runs. No LIBERO-specific prior knowledge
was used: table height, cabinet height, handle position, jaw span, tip offset,
grasp height and drop point were all re-derived here.

**The one load-bearing finding**: the drawer does not open by position control.
The k3 pack's *raw actions* command `dz ~ -0.6..-0.25` — hard down — throughout
the pull while the eef z holds at 1.105. That is contact force, not motion.
Seating the jaw on the handle and then commanding each +y waypoint 0.060 BELOW
the seated height turned 0.000 m of travel into 0.159 m on every seed. The
keyframes and ee_path alone were not enough to see this; it only shows up in
the action stream.
