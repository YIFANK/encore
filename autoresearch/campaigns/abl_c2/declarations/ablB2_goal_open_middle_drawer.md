# abl_c2 / ablB2_goal_open_middle_drawer (variant B, no-LAWS) — worker ledger

Task intent: "open the middle drawer of the cabinet". Runner: tools/fair_run.py
on AbakaAI, GPU 6. Debug seeds 51-65 only. Version cap for this cell: 12.

This session is a CONTINUATION: version 1 was written and probed by an earlier,
interrupted session for this same cell. Its artifacts were inherited and
diagnosed from scratch (no access to the earlier session's reasoning).

## Version log

### v1 (inherited) — 0/4
Hypothesis (reconstructed from the artifact): the pack's ee_path6 can be
replayed open-loop — a fixed tilted "hook" at HOOK_XYZ=(0.017,-0.142,1.036)
followed by a pull to y=+0.050.
Evidence: results/fs_ablB2_goal_open_middle_drawer_v1, seeds 51,53,55,57 →
0/4. All four move residuals were ~0.01, i.e. the arm reached every waypoint
with no resistance; the before/after cabinet crops of ep51_fail.gif show the
cabinet completely untouched. Projecting v1's own logged engage EEF into the
cam_high frame it also logged (K, t_base_cam) puts it ~10 px (256-res) on the
+y side of the handle column visible in the gif — a clean miss of ~0.03 m.
Verdict: refuted. A fixed replay of the demo waypoints does not transfer,
because the cabinet is not at the pack-demo pose in the debug seeds.

### v2 — 0/4 (seeds 51,53,55,57)
Hypothesis: localise the cabinet and its middle handle from cam_high depth
each episode and hook relative to the perceived handle *tip*; sweep four
insertion depths/heights inside one episode (a successful pull terminates the
episode, so later trials only run after earlier ones fail).
Evidence: perception worked and was consistent across seeds — table
z=0.9025, cabinet top plateau z=1.1275, drawer face plane y=-0.1573 (ep51),
three evenly spaced handle bands at z=0.948 / 1.018 / 1.092 with tips 0.031 m
proud of the face. But every pull met no resistance (res ~0.009) except
trial 1, which was *blocked head-on* at y=-0.1175 (res 0.034) — the fingertip
hit the bar rather than passing it.
Verdict: perception correct, engagement geometry wrong. Aiming at the handle
tip either stops outside the bar or collides with it.

### v3 — 4/4 probe, 14/15 selection
Hypothesis: read the pack's engage waypoint against v2's measured numbers.
demo0 engages at y=-0.145 = face_y+0.012 (0.019 *behind* the tip, deep in the
slot between bar and drawer face) and z=1.032 = band centre +0.014, and it
gets there by translating in -y at z=1.05 — clear between two handle bands —
and only then descending. v1's replay stalled at face+0.024, still outside the
bar. So keep the demo's waypoint *shape* but anchor it on the per-episode
perceived face plane and handle band, and sweep the two slot offsets.
Evidence: probe 51,53,55,57 → 4/4 (fs_..._v3). Formal 15-seed selection
sel_ablB2_goal_open_middle_drawer_v3 → 14/15; only seed 56 failed.
Verdict: mechanism confirmed.

### v4 — 6/6 probe, 15/15 selection  ← FROZEN
Hypothesis: seed 56's failure is a perception outlier, not a mechanism gap.
Its log shows the middle handle band reporting xmed=-0.167 (tip points spread
over the whole 0.47 m drawer face, dragged to the cabinet's far front corner)
while the other two bands of the same cabinet agreed at 0.042 / 0.067; all
four trials then executed cleanly 0.22 m away from the actual handle.
Change: tighten the tip window (ymax-0.012 → ymax-0.008) and take the handle
x as the median across the three detected handle bands, overriding the chosen
band whenever it disagrees by more than 0.03 m.
Evidence: probe 51,53,55,56,57,59 → 6/6 (fs_..._v4, seed 56 now succeeds).
Formal 15-seed selection sel_ablB2_goal_open_middle_drawer_v4 → 15/15.
Verdict: accepted; frozen.

## Mechanism (what was observed)

The drawer handles are bars standing 0.031 m proud of the drawer face, with a
slot between bar and face. Pulling the drawer requires a gripper surface
inside that slot: contact must be on a face-side (-y-facing) surface, so
anything at or outside the bar tip either slips off (small residual, no
motion) or collides head-on (large residual). The working engagement is
face_y+0.012 in depth and band-centre+0.014 in height, reached by translating
in -y at a height midway between two handle bands and then descending into the
slot, followed by a 0.24 m pull in +y at constant height. Everything is
anchored per-episode on the cam_high depth cloud: table height by z-histogram
mode, cabinet by top-view connected component, top plateau by z-histogram of
the high points, face plane by the median of the per-height max-y profile, and
the three handle bands by protrusion above that plane, the middle one being
the middle drawer's.

## DECLARATION

- Frozen program: `packs/ablB2_goal_open_middle_drawer/program.py`
  md5 `966a6855afb86de770b738ae59e6d163`
  == `packs/ablB2_goal_open_middle_drawer/program_v4.py` (same md5)
  == `results/sel_ablB2_goal_open_middle_drawer_v4/program_archived.py` (same md5)
- Selection receipt: **15/15** on the full 15-seed debug split (51..65),
  result dir `results/sel_ablB2_goal_open_middle_drawer_v4`
- Per-version receipt chain:
  - v1 (inherited): `results/fs_ablB2_goal_open_middle_drawer_v1` — 0/4 (51,53,55,57)
  - v2: `results/fs_ablB2_goal_open_middle_drawer_v2` — 0/4 (51,53,55,57)
  - v3: `results/fs_ablB2_goal_open_middle_drawer_v3` — 4/4 (51,53,55,57);
        `results/sel_ablB2_goal_open_middle_drawer_v3` — 14/15 (51..65)
  - v4: `results/fs_ablB2_goal_open_middle_drawer_v4` — 6/6 (51,53,55,56,57,59);
        `results/sel_ablB2_goal_open_middle_drawer_v4` — 15/15 (51..65)
- Versions used: 4 of the 12-version cap.
- Debug episodes run in this cell (all probes + selections, including the
  inherited v1's 4): 4 + 4 + 4 + 15 + 6 + 15 = **48**
- PROVENANCE: present as a top-level literal dict in program.py, covering
  TILT_DEG, SLOT_FY, SLOT_FZ, CLEAR_DZ, STANDOFF_DY, PULL_M, TRIALS,
  X_ROBUST, TABLE_BIN, GRID_M.
- Splits: seeds 1-50 never touched; `--split debug` only; every run via
  tools/fair_run.py.
