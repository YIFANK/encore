# Encore: experiment code

Code for *Encore: Few-Shot Agentic Discovery of Manipulation Strategies*. A coding agent reads a few
demonstrations, writes a policy program against a fixed perception-and-action
API, develops it on a disjoint band of initial states, and freezes it; the
frozen program is then evaluated once on sealed states, judged only by the
benchmark's own success predicate.

This branch contains only what the paper's experiments use. Sealed results,
frozen programs, and a script that recomputes every number in the paper are in
`reproduce/` (start with `python3 reproduce/verify.py`, standard library only;
see `reproduce/README.md`).

## Layout

| Path | What it is |
|---|---|
| `tools/fair_run.py`, `tools/fair_client.py` | The evaluation harness. `fair_run.py` owns the simulator (or the robot); the agent's program runs in a separate process and reaches the world only through the RPC API in `fair_client.py`. No simulator state, object poses, or success signal crosses that boundary. |
| `tools/fair_run_robodojo.py`, `heron/robot/robodojo_env.py`, `tools/robodojo/` | The same harness for RoboDojo (Isaac Sim, bimanual), and the adapter installed into a RoboDojo checkout. |
| `tools/fair_pack.py`, `tools/fair_pack_robodojo.py`, `tools/fair_pack_strip.py`, `tools/strip_pack.py`, `tools/l90abl_build_packs.sh` | Demonstration distillers: keyframes at gripper events, frame strips, end-effector paths, actions, and the reduced (images-only) variants. |
| `heron/` | Perception and robot library behind the API: RGB-D grounding, deprojection, the LIBERO / robosuite / RoboDojo / Trossen adapters, kinematics, and the VLM client used for grounding and VQA. |
| `tools/rig_*.py`, `heron/robot/rig_fair.py`, `tools/collect.py`, `tools/depth_server.py`, `tools/spawn_rig_task*.sh` | Real-robot experiments on a bimanual Trossen WidowX AI: the fair API over hardware, pose-card protocol, reset, judge, teleoperation recording, pack building, camera server, and agent launch. |
| `tools/calibrate_*.py`, `tools/handeye_calibrate.py`, `tools/cross_calibrate.py`, `tools/recalibrate.py`, `tools/measure_table.py`, ... | Rig camera and arm calibration. |
| `autoresearch/campaigns/` | One folder per experiment: cell lists, the agent-brief generator, run, freeze-and-evaluate scripts, pre-registration. |
| `autoresearch/tasks/rig_*` | Briefs of the two real-robot tasks. |
| `autoresearch/FAIR_PROTOCOL.md`, `README.md`, `RIG_PROTOCOL.md` | The worker contract, the coordinator procedure, and the rig protocol. |
| `baselines/aspire/` | Our drivers for the ASPIRE baseline (NVlabs/ASPIRE at commit `680bad4`): the opus-5 rerun and the planner / threshold ablation. |

## Experiments

| Paper | Campaign | Arms |
|---|---|---|
| LIBERO-PRO, both perturbation axes, 60 cells (Fig. 2, Table 3) | `c2clean` | K=3, K=0 |
| LIBERO-PRO unperturbed column | `c2` | K=3 |
| Development efficiency (Fig. 3) | `c2clean/metrics.py` over the agent transcripts | K=3, K=0 |
| Verification ablation (Table 2) | `abl_c2` | verification loop removed |
| LIBERO-90 articulated fixtures | `l90abl` | K=3, images only, K=0 |
| RoboDojo, ten tasks (Table 1 left) | `rd1` | K=3, K=0 |
| RoboDojo, twelve tasks (Table 1 right) | `rd2` (`cluster/` = evaluation-box scripts) | K=3, images only, K=0 |
| ASPIRE rerun and ablations | `baselines/aspire/` | ASPIRE as released, + demonstrations, no grasp planner, no thresholds |
| Real robot: cube handover, cup inversion | `autoresearch/tasks/rig_handover`, `rig_flip_cup` | K=5 |

## Running a campaign

Each campaign runs four steps, one fresh agent per cell:

1. **Packs.** Build demonstration packs on the evaluation box, for example
   `tools/fair_pack.py` for LIBERO or `tools/fair_pack_robodojo.py` for RoboDojo.
2. **Workspaces.** `autoresearch/campaigns/<c>/build_all.sh` (it calls
   `mkworker.sh`) writes one workspace per cell with the brief, the pack, the
   API documentation, and the development seeds.
3. **Development.** `run.sh` launches a headless Claude Code session per cell
   (`tools/ar_launch_worker.sh`; authentication through `tools/claude_auth.sh`
   from `ANTHROPIC_API_KEY` or `CLAUDE_CODE_OAUTH_TOKEN`, in the environment
   or a git-ignored `.env`). The paper uses model `claude-opus-5`. The agent
   ends with a declaration naming its frozen program.
4. **Freeze and evaluate once.** `run_eval.sh` records the program's md5 and
   runs the sealed evaluation:

```bash
# LIBERO-PRO / LIBERO-90
env -u PYTHONPATH .venv/bin/python tools/fair_run.py program --seed-episodes \
  --bddl <bddl> --language "<sentence>" --program packs/<cell>/program.py \
  --split eval --out results/eval_<cell> --tmp-root <scratch>

# RoboDojo
env -u PYTHONPATH .venv/bin/python tools/fair_run_robodojo.py --task <task> \
  --program packs/<cell>/program.py --split eval --eval-n 50 --gpu <g> \
  --out results/eval_<cell>
```

Development seeds are 51-65; sealed seeds are 1-50 (RoboDojo: the first 50
official evaluation layouts). A static check refuses any program that reads
the episode-termination flag, so every program must verify its own progress.

## Setup

- Python 3.10, `pip install -e .[dev]`; real robot adds `.[real,record]`.
- LIBERO-PRO at commit `eafdb80` with its perturbation `bddl_files` and
  `init_files`; paths are set in `configs/libero.yaml`.
- RoboDojo at commit `ee67a14`, then `tools/robodojo/install_encore.sh
  <robodojo-root>`. The installer copies the Encore robot, camera, and sim
  configs, registers the policy, and applies two patches: gripper joint
  states in observations, and an `EVAL_NUM` override without which a
  50-episode evaluation is cut to a task's default of 25.
- Grounding and VQA: RoboDojo uses `configs/robodojo_ppapi.yaml`
  (`gemini-3.5-flash` through an OpenAI-compatible relay, `PPAPI_BASE_URL`
  and `PPAPI_API_KEY` in `.env`); LIBERO uses `configs/libero.yaml`
  (Gemini Robotics-ER, `GEMINI_API_KEY`).
- Real robot: two Trossen WidowX AI arms and four RealSense D405 cameras;
  `configs/rig5090.yaml`, `tools/depth_server.py` for the cameras, and the
  calibration tools above. Two rig-host files are not yet in this
  repository: `start_sam3.sh`, which starts the SAM3 segmentation service
  the rig API calls on port 8772, and `tools/rig_cupstack_ep.sh`, the
  episode template the rig task briefs point agents to.

## Tests

```bash
python -m pytest -q
```
