# Running a fair campaign on the physical rig (protocol v1.3)

The simulated campaigns (c1, c2) and a hardware campaign run the *same*
protocol. This document is only about the three places where hardware needs a
different mechanism to keep the same guarantee, plus the commands to run.

| guarantee | in simulation | on the rig |
|---|---|---|
| reproducible initial state | seed | **pose card** placed by the reset executor and verified to tolerance |
| sealed evaluation band | server refuses seeds outside 51–65 | eval cards live in a 0700 directory the coordinator alone dispatches |
| success bit not visible at runtime | benchmark predicate, post-episode only | **no predicate exists during the episode**; the judge runs after the program's process is gone |
| information contract | process isolation + RPC + AST gate | identical, unchanged |

Everything else — clean-room worker briefs, one formal 15-episode selection,
md5 freeze, one blind evaluation, PROVENANCE, the ban on `.done` reads — is
the machinery already in `tools/fair_run.py`.

## The pieces

| file | role |
|---|---|
| `heron/robot/rig_fair.py` | `RigRobot`: the fair API surface over `TrossenStationary`. Workspace + effort guards, command budget, coordinator-private episode trace. `task_success` stays False for the whole episode by construction. |
| `tools/rig_protocol.py` | Pose cards: generates the debug band (`d51`–`d65`, open) and the evaluation band (`e1`–`e50`, sealed 0700) from one layout file and a seed. |
| `tools/rig_reset.py` | Reset executor and verifier: perceive → place → verify against the card; escalates with `NEEDS-HUMAN` rather than proceeding from an unverified scene. |
| `tools/rig_judge.py` | Coordinator-side success bit from geometric predicates, with a recorded VLM second opinion that never overrides geometry. |
| `tools/rig_pack.py` | LeRobot teleop episodes → fair demonstration pack, same schema and validator as the simulated packs. |

## Depth

This rig cannot serve depth (macOS will not hand librealsense the D405s).
Frames therefore carry a table-plane homography, and `FairFrame.deproject`
falls back to it automatically: exact for anything resting on the table,
`None` above it. `frame.proj` (DLT) is exposed for two-camera triangulation of
off-plane points. Worker briefs must state this — a program written for
dense depth will silently get `None`.

## Running one task, end to end

**1. Author the layout** (once per task) — placement regions and tolerance:

```bash
cat > layouts/bowl_on_plate.json <<'JSON'
{"objects": {"bowl":  {"region": [0.30,-0.16,0.46,-0.04], "yaw_range": [-3.14,3.14]},
             "plate": {"region": [0.30, 0.04,0.46, 0.16], "yaw_range": [0,0]}},
 "min_separation_m": 0.12, "tolerance_m": 0.02, "tolerance_rad": 0.20}
JSON
```

**2. Generate both bands.** The evaluation band is drawn first so that
regenerating the debug band later cannot shift it:

```bash
python3 tools/rig_protocol.py make --task bowl_on_plate \
  --layout layouts/bowl_on_plate.json --seed 20260817 \
  --out cards/bowl_on_plate --sealed-out ~/rig_sealed/bowl_on_plate
```

**3. Record K=3 demonstrations** with the existing teleop recorder, then
distil them:

```bash
python3 tools/rig_pack.py --dataset ~/rig_data/bowl_on_plate --k 3 \
  --language "put the bowl on the plate" --out packs/rig_bowl_on_plate
```

**4. Per trial** — reset, verify, run, judge:

```bash
python3 tools/rig_reset.py apply  --config configs/abaka.yaml \
  --cards cards/bowl_on_plate --card d51
python3 tools/rig_reset.py verify --config configs/abaka.yaml \
  --cards cards/bowl_on_plate --card d51 --json    # exit 1 = out of tolerance, do not run

python3 tools/fair_run.py program --rig --config configs/abaka.yaml \
  --cards cards/bowl_on_plate --card d51 \
  --language "put the bowl on the plate" \
  --program packs/rig_bowl_on_plate/program.py \
  --split debug --episode-list 51 --out results/rig_dbg_bowl_on_plate_v1

python3 tools/rig_judge.py --config configs/abaka.yaml \
  --goal goals/bowl_on_plate.json --cards cards/bowl_on_plate \
  --card d51 --episode 51 --out results/rig_dbg_bowl_on_plate_v1
```

The goal file the judge reads:

```json
{"language": "put the bowl on the plate",
 "predicate": {"type": "on", "object": "bowl", "target": "plate", "xy_tol_m": 0.05},
 "vlm_question": "Is the bowl resting on the plate? Answer yes or no."}
```

**5. Blind evaluation** is the same loop over the sealed cards, run once by
the coordinator with `--cards ~/rig_sealed/bowl_on_plate --card e1 …e25`,
after the program is md5-frozen. Workers never see that path.

## Budget reality

A trial is roughly: reset 40–60 s, episode up to the command budget, judge
10 s. Twenty-five sealed trials is therefore about an hour of wall clock per
arm, and a debug band of 15 is about 25 minutes per pass. Plan the evaluation
band at 20–25 cards rather than 50 unless the task is fast.

## What still needs a human

Escalations are counted, not hidden. `rig_reset.py` exits 2 with a
`NEEDS-HUMAN` line when an object is off the table, out of reach, or not
visible. Each escalation is one row in the campaign's cost table — the
honest hardware analogue of the "free" resets simulation gives away.

## Open items before the first campaign

- Fill `workspace.min_xyz` / `max_xyz` / `max_effort_n` in the rig config;
  `RigRobot` refuses out-of-envelope targets and needs real numbers.
- Confirm `plane_z` for the table in world frame (passed to `rig_reset.py`).
- Decide the per-episode command budget (`--max-commands`, default 400).
- One dry run of the whole loop on a single card before any campaign, to
  measure real reset time and escalation rate.
