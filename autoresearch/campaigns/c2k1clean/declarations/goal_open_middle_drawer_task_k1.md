# c2k1clean / goal_open_middle_drawer_task_k1

Intent: **"open the bottom drawer of the cabinet"**.
Pack language: **"open the middle drawer of the cabinet"** (pack.json `language`).
So the pack supplies the MECHANISM; the instruction supplies the TARGET.

## What the pack says (pack.json, keyframes/)

* K=1, one demo, 138 steps, 3 keyframes, 14-point `ee_path` / `ee_path6`.
* `gripper_cmd` is -1.0 at every keyframe and the 7th slot of all 138 raw
  actions is -1.0: **the demo never closes the jaws**. It is an open-jaw hook,
  not a grasp.
* The `ee_path6` rotation triples are **rotation VECTORS, not euler rpy**.
  Receipt: `rv2R([3.1477,-0.0273,-0.0727])` (the t=0 triple) is symmetric and
  matches the symmetric matrix `api.tool_rotation()` returns at the start of
  debug seed 51,
  `[[0.9984,0.0005,-0.0568],[0.0005,-1,0],[-0.0568,0,-0.9984]]`; both euler
  orders give a non-symmetric matrix with the off-diagonals in the wrong slots.
* Shape of the demo: the wrist tilts from straight-down to an approach axis of
  `(-0.059,-0.785,-0.617)` (38 deg below horizontal, pointing into the
  cabinet -- this swings the hand BODY up and away from the drawer face),
  descends to `(-0.0036,-0.1453,1.0322)`, then drags **+y by 0.170 m** at
  constant height.

## What I measured myself (cam_high RGB-D, debug seeds 51/53/57/61)

Deprojected with each frame's own intrinsics + `t_base_cam`.

* table plane z = 0.9025; cabinet top slab z = 1.1275, footprint
  x in [-0.10,0.16], front edge `yface` = -0.157 / -0.161 / -0.170 / -0.165
  (the cabinet translates a few mm per seed; nothing else about it moves).
* **Three pull-bars** protrude in +y from that face. z bands
  `0.9375-0.9575`, `1.0125-1.0275`, `1.0875-1.1025` on every seed, i.e. tops
  0.9557 / 1.0245 / 1.0979, spacing 0.069 / 0.073. Each is ~0.087 long in x
  and ~0.025 deep in y, carried on two short posts.
* The lowest band's own x/y readout is polluted by a prop standing on the table
  beside the cabinet; the two upper bars are shielded by the drawer above them,
  so `bottom_bar()` takes x and y from their consensus and only z from the low
  band.
* Wrist-camera cross-section at the bar's x centre (v5, seed 51):
  **drawer face y = -0.163, bar y in [-0.139,-0.131], z in [0.936,0.952]**,
  drawer-front bottom edge 0.908, table 0.900. A ~0.024 m pocket sits behind
  the bar, open above and below.

## Version chain (hypothesis -> evidence -> verdict)

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v0 | perception probe | streamed cam_high RGB-D through `api.log`; chunks must be <=1900 chars (the log truncates at ~2000) | tooling ok |
| v1 | re-aim the demo hook onto the lowest bar by feature offsets | **0/8**; every residual small -> the hand swept free air, bars unmoved | refuted |
| v2 | geometry probe | with the demo wrist the hand stops at `yface+0.0163` when it passes ABOVE a bar (z = bar+0.030) and at `yface+0.036` at bar height -- the bar itself blocks it. Both numbers identical on seeds 51 and 57 | measurement |
| v3 | same hook, jaws CLOSED (thin prong) | **0/8**; the drop stalls at z=0.9637 and the pull sweeps free | refuted |
| v4 | same hook, jaws OPEN | **0/8**, but the agentview first-vs-last crop shows the bottom drawer visibly displaced | partial |
| v5 | wrist camera at the hook pose | **the 0.080 m jaw span straddles the WHOLE handle** -- both fingers sit outside the bar's two posts; also yielded the cross-section above | diagnosis |
| v6 | turn the wrist fully horizontal so the jaws split vertically, then clamp the bar | unreachable at this height: the arm never descends (eef stuck at z=1.002 for a z=0.944 command) | refuted |
| v7 | keep the demo tilt, roll the jaw axis 90 deg about the approach axis | reachable (rotation achieved), but the upper jaw hits the drawer face at eef y=-0.117 while the lower jaw is still in front of the bar; clamp width 0.0018 = air. Geometry: a 38 deg tilt puts 0.08*sin38 = 0.05 m of y-spread between the jaw tips, twice the 0.024 pocket. **A tilt above ~18 deg cannot clamp an x-aligned bar** | refuted |
| v8 | descent-depth probe, closed prong | at eef y=-0.115 the prong falls free to z=0.912 (below the bar); at y=-0.127 it stalls at 0.965 (on top of the bar). Brackets the prong foot at ~0.010 in -y and ~0.0095 below the eef | measurement |
| v9 | hook from UNDERNEATH: drop in front, drive in under the bar, lift into the pocket, drag | **0/8**; the lift jams at z=0.9214 and slides +y -- the prong rides up the bar's front rather than entering the pocket | refuted |
| v10 | **control**: replay the same hook on the MIDDLE bar, the one the pack actually demos | on seeds 51 and 57 the 1.0245 band disappears from the re-perception and the agentview shows the middle drawer pulled out -- **the mechanism reading is correct** -- yet `benchmark_success` is still **false** | key control |
| v11 | retract the arm before the final capture so the re-perception measures the DRAWER, not the gripper | **bottom drawer travels 0.1715 m** (bar yfront -0.1261 -> +0.0454), i.e. full travel, matching the demo's 0.170. **0/4** | key measurement |
| v12 | hook the bottom bar a second time | the drawer is already at its stop (second cycle adds 0.0000). **0/4** | refuted |
| v13 | hook the TOP bar | travels only 0.0243. **0/4** | partial |
| v14 | hook the MIDDLE bar, with the retract-and-measure harness | the middle band vanishes (drawer out). **0/4** | refuted |
| v15 | TOP bar, two cycles | 0.024-0.028 on all 8 seeds, second cycle adds 0.0000. **0/8** | refuted |
| v16 | **final candidate**: the v11 mechanism, cleaned up, with full PROVENANCE and one conditional retry | bottom drawer open on **8/8** probe seeds (confirmed frame-by-frame on 51/53/57/59/63/65). **0/8** | argmax |
| v17 | conjunction check: open the bottom drawer AND the middle drawer in one episode | both visibly open in the final frame. **0/8** | refuted |

## Mechanism-gap stop

**Falsifiable statement of the missing mechanism.** There is no reachable
action on this cell that sets the graded bit. Specifically: the graded
predicate does not respond to the articulated state of any of the cabinet's
three drawers.

Receipts, all on debug seeds, ~90 episodes:

* **bottom drawer driven to its full travel** (0.1715 / 0.1766 / 0.1581 m
  measured by re-perception with the arm retracted; visually confirmed open in
  the last agentview frame of **all 15** selection episodes) -> **0/15**.
* **middle drawer driven fully open** (v10 on 2 seeds, v14 on 4) -> **0/6**.
  This is the drawer the pack itself demonstrates and the drawer named by the
  `--bddl` filename, so it is the one reading of the cell under which my
  replay is an exact reproduction of a known-good demonstration.
* **top drawer** engaged but only 0.024-0.028 m (the hook slips there)
  -> **0/12**.
* **bottom and middle open together** in one episode -> **0/8**.

What would falsify this: any program that sets `benchmark_success` on a debug
seed. Since the three drawers are the only articulated degrees of freedom on
the only cabinet in the scene (the top-down height map over the whole
workspace shows one drawer cabinet at x in [-0.10,0.16] plus a leaning wooden
rack and the arm mount, nothing else above the table at y<0), and the
instruction asks for nothing but an opening, I can see no further state to
reach. The most likely explanation is a re-authoring gap: the language was
changed to "bottom" while the goal expression was left pointing at something
the scene cannot satisfy, so `env.check_success()` never returns true (the
runner treats a raising predicate as "not a success", which is
indistinguishable from a false one from inside the program).

## DECLARATION

* **Frozen version: v16.** `program.py` md5 `97733f64affc8c6f2a14ea7783676a22`
  == `program_v16.py` md5 `97733f64affc8c6f2a14ea7783676a22`.
* **Full-15-seed selection receipt: 0/15**, dir
  `results/sel_c2k1clean_goal_open_middle_drawer_task_k1_v16`
  (seeds 51-65, `--split debug`). The bottom drawer is open in the final frame
  of all 15 episodes.
* **Per-version receipt chain** (all `--split debug`, dirs under
  `results/`):
  `fs_..._v0`, `fs_..._v0b` (probes) ·
  `fs_..._v1` 0/8 · `fs_..._v2` (probe, 2 seeds) · `fs_..._v3` 0/8 ·
  `fs_..._v4` 0/8 · `fs_..._v5` (probe, 1 seed) · `fs_..._v6` 0/4 ·
  `fs_..._v7` 0/4 · `fs_..._v8` (probe, 2 seeds) · `fs_..._v9` 0/8 ·
  `fs_..._v10` 0/2 · `fs_..._v11` 0/4 · `fs_..._v12` 0/4 · `fs_..._v13` 0/4 ·
  `fs_..._v14` 0/4 · `fs_..._v15` 0/8 · `fs_..._v16` 0/8 · `fs_..._v17` 0/8 ·
  `sel_..._v16` **0/15**.
* **PROVENANCE present** in `program.py`: 6 entries covering the demo
  waypoints / engage pose / pull rotation, the 0.20 m pull length, the two
  engage offsets `DY_ENGAGE` / `DZ_ENGAGE`, the whole perception stack
  (`find_bars` / `bottom_bar` constants), and the retract / verification
  thresholds. Every source is either a named pack.json field or a named
  debug-seed measurement.
* **Argmax version: v16 (0/15).** All versions tie at zero; v16 is the one
  that does what the instruction asks and does it reliably (bottom drawer open
  on 15/15 debug seeds).

STOP.
