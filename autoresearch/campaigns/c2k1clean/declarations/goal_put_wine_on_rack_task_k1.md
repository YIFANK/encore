# c2k1clean / goal_put_wine_on_rack_task_k1

Intent: **"Put the cream cheese on the rack"** (bddl = put_the_wine_bottle_on_the_rack).

## Pack reading

| pack | language | what it gives |
|---|---|---|
| `..._task_k1` | "put the wine bottle on the rack" | the TARGET, **in this scene**: grasp side-on at (-0.210,-0.080,1.021), release on the rack at (-0.156,-0.269,**1.203**) and again at (-0.131,-0.250,1.235) |
| `..._task_mate` | "put the cream cheese in the bowl" | the OBJECT: straight-down wrist (rpy 3.09,0.04,0.14), grasp with the eef at **z=0.9104** |

So the rack is the goal region and the cream cheese is the payload; the k1 pack's
release z pins where "on the rack" lives, the mate pack pins the grasp height.

## Tooling

`api.log` truncates at 2000 chars, so `program_v*.py` chunk zlib+base64 RGB-D
into `B <tag> <i> <payload>` lines and `decode.py` / `hmap.py` reassemble them
locally. All perception is therefore done offline at zero sim cost.

## Scene (re-derived from debug seeds 51/53/57/61, cam_high RGB-D)

- table plane **z = 0.901**
- **cream cheese**: blue slab, 0.078 x 0.040 x 0.019 m lying flat, top at
  **z = 0.9197** on every seed, long axis within 4 deg of +x.
  Centre wanders over x[-0.099,0.004], y[0.094,0.154].
  It is the only blue prop in the 0.906-0.940 band -> `B-(R+G)/2 > 12`.
- **rack**: slatted wooden plank, a *plane inclined ~31 deg about x*.
  Fit `z = a*x + b*y + c` with a in [-0.034,-0.002], **b ~ -0.61**, c in
  [1.012,1.024]; rms 5-11 mm. Extent x[-0.405,-0.120], y[-0.333,-0.178].
  Side rails reach z=1.245. Slats give a +-3 mm ripple along x only.
- other props: cabinet top (flat, z=1.128, x>-0.104), bowl (ztop 0.952),
  plate (ztop 0.920), stove slab (0.926/0.932), wine bottle + robot arm.

## Version log

### v1 — perception probe (seeds 51,53,57,61)
Hypothesis: the scene can be read entirely from cam_high.
Evidence: yes; all of the above. First attempt truncated at 2000 chars, fixed by
chunking.
Verdict: scene understood. No motion, 0/4 (expected).
Receipt: `results/fs_..._v1`.

### v2 — grasp probe (seeds 51,53,57,61)
Hypothesis: a top-down pinch across the slab's 40 mm short axis holds it, and a
blocked descent onto the table calibrates the fingertip offset.
Evidence:
- the descent never blocks — the eef simply trails the command by a constant
  ~10 mm, so commanding z=0.900 settles the eef at **0.9104**, which is
  *exactly* the mate pack's own grasp z. Two independent sources agree.
- close -> gap **0.0422**, effort 3.0, on 4/4 seeds (slab short extent
  0.039-0.041, so this is the slab, not an empty close).
- lift to 1.09 -> gap unchanged, effort 3.0, and the slab is visibly out of the
  scene and in the wrist camera. **4/4 grasps.**
- hang: object bottom sits **9.4 mm** below the eef (from the grasp geometry;
  re-perceiving the held slab gives 8-11 mm).
Verdict: grasp solved. 0/4 benchmark (no place attempted).
Receipt: `results/fs_..._v2`.

### v3 — full pipeline (seeds 51,53,55,57,59,61,63,65)
Hypothesis: releasing the slab just above the inclined rack surface leaves it
there; mu(MuJoCo default ~1) > tan(31 deg)=0.6 so it should not slide.
Release height is set by the *uphill fingertip*: the jaws open to +-0.039 along
y, and at 0.61 slope that point is 24 mm above the surface under the tool, so
`z_rel = surf + 0.61*0.039 + 0.010 + 0.010`.
Evidence: **crashed 0/8** — `api.move_path` does not exist on the LIBERO
backend (`AttributeError: 'LiberoRobot' object has no attribute 'move_path'`),
same class of gap as `api.act`. The grasp half ran perfectly first.
Verdict: harness bug, not a mechanism failure.
Receipt: `results/fs_..._v3` 0/8.

### v4 — v3 with the carry expressed as discrete `api.move` calls
Only change from v3: the two-waypoint `move_path` becomes two `api.move` calls.
Evidence: **8/8** on the probe subset (51,53,55,57,59,61,63,65). Every seed
closes at gap 0.0422 / effort 3.0 and releases at (-0.230, y, ~1.219); the
episode terminates ~1 step after the release, i.e. the predicate fires on the
release itself.
Verdict: selected.
Receipt: `results/fs_..._v4` 8/8.

Note on why the release height self-normalises: `surf` came out at
1.1750-1.1753 on all 15 seeds even though the plane's intercept ranges over
1.012-1.024. Placing at the rack's *own* mid-y makes the seed-to-seed shift of
the rack cancel, because the rack translates along its own slope.

## DECLARATION

- **Frozen version: v4.**
  `packs/c2k1clean_goal_put_wine_on_rack_task_k1/program.py`
  md5 `914215a14c6fffbbf0e826bf21b0f068` == `program_v4.py` (verified on the
  cluster and locally).
- **Selection receipt (full 15 debug seeds, 51-65): 15/15.**
  `results/sel_c2k1clean_goal_put_wine_on_rack_task_k1_v4`
  All 15 seeds: closed gap 0.0422 / effort 3.0, release at
  x=-0.230+-0.002, z=1.219+-0.002.
- **Receipt chain**
  | version | seeds | result | note |
  |---|---|---|---|
  | v1 | 51,53,57,61 | 0/4 | perception probe, no motion |
  | v2 | 51,53,57,61 | 0/4 | grasp probe, no place; 4/4 grasps |
  | v3 | 8 probe seeds | 0/8 | crash: `move_path` unsupported on LIBERO |
  | v4 | 8 probe seeds | **8/8** | full pipeline |
  | v4 | **15 debug seeds** | **15/15** | **selection** |
- **PROVENANCE**: present in `program.py` as a top-level literal dict, 14
  entries, every one sourced to a named pack field or a debug-seed measurement.
- Archived: `program_v1.py` .. `program_v4.py` in the pack directory.

Mechanism in one line: the k1 pack names the rack as the goal region and pins
its release altitude, the mate pack pins the cream cheese's grasp height, and
the only thing that had to be derived was that the rack is a 31-degree ramp —
so the release height must clear the *uphill fingertip*, 0.61*0.039 = 24 mm
above the surface under the tool, not the surface itself.
