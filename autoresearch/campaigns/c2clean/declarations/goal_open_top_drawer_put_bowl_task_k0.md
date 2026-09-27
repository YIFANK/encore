# c2clean / goal_open_top_drawer_put_bowl_task_k0 — NOTES

Intent: "Open the top layer of the drawer and put the cream cheese inside".
No demo pack (k0). Everything below is derived from debug seeds 51-65 only.

## Scene (cam_high RGB-D, seeds 51/53/55 — layout near-identical, ~2 cm jitter)
- Table plane z = 0.902.
- Cabinet at image-left: top plane z = 1.126, front face plane y = -0.161,
  x span -0.095..0.159 (visible top), y span -0.347..-0.157.
- THREE drawer handles on the +y face, bars at z 0.930-0.956 (bottom),
  1.009-1.025 (middle), 1.082-1.098 (top). Bar front surface y = -0.125,
  bar x span -0.007..0.081 (centre x ~ 0.037..0.047). Identical on 51/53/55.
- Cream cheese = the only blue-dominant prop, a flat box: top face z = 0.919
  (1.7 cm tall), footprint 0.072 (x) x 0.035 (y). Centre moves ~2 cm between
  seeds, so it must be perceived per episode.
- Glass bowl ~(-0.075, 0.00) top z 0.952; plate, stove, wine bottle elsewhere.

## Harness mechanics (measured, not assumed)
- Episode horizon = 1000 sim steps (v3h probe: burnt settles, sim_steps capped
  at exactly 1000). Every move/grip/settle must be budgeted.
- move(rotation=None) -> move_cartesian, <= max(40, 120*seconds) steps, breaks
  at 12 mm. move(rotation=R) -> move_pose, <= max(60, 60*seconds) steps.
- eef() is the GRIP SITE: v3 descent over bare table stalled at eef z = 0.9096
  on a 0.902 plane => fingertips sit 8 mm beyond the grip site along tool z.
- Jaws separate along the TOOL Y axis: at the v3 pre-grasp the open fingers
  resolved at base y = -0.175 and -0.085 about eef y = -0.128.
- Open gap 0.078 m (outer half-width ~0.045); grip() is binary
  (<0.025 = close, else open). effort 3.0 iff gap > 5 mm after a close.

## Version chain
- v1 (seeds 51,53,55) perception probe. RGB-D shipped out through api.log as
  zlib+base64 chunks (<=1800 chars/line) and decoded offline. No motion.
- v2 (seed 51) calibration attempt at x=0.15,y=0.30 — UNREACHABLE: a 2.0 s move
  burnt its whole 240-step budget and stalled 49 mm short. Verdict: probe point
  is outside the workspace; discard.
- v3 (seed 51) calibration at x=-0.25,y=0.38 (reachable). Gave TIP=0.008 and the
  jaw axis (above). 338 steps. v3h gave the 1000-step horizon.
- v4 (seeds 51,53) drawer only. TOP-DOWN grasp of the bar is geometrically
  impossible: the bar-to-face gap is 2.4 cm but the open half-width is 4.5 cm,
  and the cabinet top (z 1.126, front edge y -0.157) roofs the gap. Instead a
  SIDE-ON grasp with the wrist turned (R_SIDE: approach along -y, jaws along z)
  straddles the 1.7 cm bar vertically. Result: closed gap 0.0174 with effort
  3.0, held through +0.15 m of pull; at +0.20 m the gap collapsed to 0.0044
  (drawer travel limit yanks the bar out). Post-open depth: drawer floor plane
  z = 1.064, front-panel rim z = 1.124, aperture x -0.05..0.10, y -0.13..-0.04.
- v5 (seeds 51,53,55,57 probe then formal 15) FULL TASK: perceive handle + box
  from one cam_high frame; side-on bar grasp; pull +0.15 m; release; top-down
  pick of the box (jaws across its 3.5 cm width, eef z 0.912 => fingertips
  2 mm above the table, closed gap 0.0422 with effort 3.0); re-capture; find
  the drawer floor plane; descend and release. Probe 4/4, formal 15/15.
  Weakness found in the receipt: the cavity search window was too loose, so the
  aperture centre came out at y = -0.128, the REAR lip under the cabinet-top
  overhang. The descent jammed at eef z 1.137 (box bottom 1.127, i.e. level
  with the 1.124 rim) and the box was effectively dropped over the lip. It
  worked on every debug seed but the margin is one centimetre.
- v6 (formal 15) = v5 with the cavity search confined to the cabinet's own
  x-span (handle bar x +- 0.14) and to y in [handle_end - 0.22, handle_end -
  0.01], centre taken from the 5/95 percentiles of the floor plane. Aperture
  centre now y = -0.100 (ep51) / -0.110 (ep58), the descent reaches eef z
  1.121 / 1.131 (box bottom 1.111 / 1.121, BELOW the 1.124 rim, i.e. inside
  the drawer at release rather than over its lip). Formal 15/15.

## Why the side-on grasp is the load-bearing choice
A top-down grasp of the handle is not merely hard, it is excluded by measured
geometry: the bar-to-face gap is 2.4 cm (bar front y -0.125, face y -0.161),
the open jaw half-width is 4.5 cm, and the cabinet top (z 1.126, front edge
y -0.157) roofs the only slot a finger could occupy. Turning the wrist so the
approach axis is -y and the jaws separate along z straddles the 1.7 cm bar in
free air; the closed gap of 0.0174 m with effort 3.0 is the receipt that the
bar, and not air, is between the fingers.

## DECLARATION
- Frozen version: **v6**. packs/c2clean_goal_open_top_drawer_put_bowl_task_k0/
  program.py md5 = 77627f7106098111cfcb4f4cdf4973fc == program_v6.py.
- Selection receipt (full 15 debug seeds, one formal run):
  **15/15** in results/sel_c2clean_goal_open_top_drawer_put_bowl_task_k0_v6
  (seeds 51-65, every episode benchmark_success true).
- Per-version receipt chain:
  - v1 results/fs_..._v1 (51,53,55) — perception only, 0/3 by construction.
  - v2 results/fs_..._v2 (51) — unreachable probe point, discarded.
  - v3 results/fs_..._v3 (51) + v3h (51) — calibration, horizon = 1000 steps.
  - v4 results/fs_..._v4 (51,53) — drawer opens, 0/2 (no placement attempted).
  - v5 results/fs_..._v5 (51,53,55,57) 4/4; formal results/sel_..._v5 **15/15**.
  - v6 formal results/sel_..._v6 **15/15** (frozen; better placement margin).
- PROVENANCE: present in program.py, 12 entries, every one sourced to a
  debug-seed measurement or to generic controller/camera mechanics; verified
  against the eval gate's own checks (no forbidden tokens, no api.done read,
  all entries allowed+sourced).
- No demonstration pack was used (k0) and no shared note file exists.
