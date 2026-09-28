# ENCORE

**Few-Shot Agentic Discovery of Manipulation Strategies**

Yifan Kang, Zihan Wang, Zhiwen Fan, Bangya Liu

A coding agent reads a few robot demonstrations, writes a policy program
against a fixed perception and action API, refines it over a small number of
development episodes, and freezes it. The frozen program is then evaluated
once on held-out episodes, judged only by the benchmark's own success check.

This repository holds the code behind the paper's experiments and every
frozen program and per-episode result the paper reports.

---

## Reproduce the results

Every program the agents wrote and froze is in this repository, so you can run
them directly instead of asking an agent to rediscover them.

| Level | What it does | Needs | Time |
|---|---|---|---|
| **1. Run a frozen program** | Evaluates a saved program on its 50 held-out episodes | LIBERO-PRO or RoboDojo, a GPU | minutes to hours per program |
| **2. Rerun the pipeline** | Builds packs, lets a fresh agent develop a program, evaluates it | Level 1 plus Claude Code | about an hour per cell |

### 1. Run a frozen program

Every evaluated program is in `reproduce/sealed/<experiment>/<cell>/program.py`,
next to its per-episode results (see [`reproduce/README.md`](reproduce/README.md)). After [setup](#setup):

```bash
# LIBERO-PRO
env -u PYTHONPATH .venv/bin/python tools/fair_run.py program --seed-episodes \
  --bddl <bddl file> --language "<task sentence>" \
  --program reproduce/sealed/libero_pro_clean/<cell>/program.py \
  --split eval --out results/eval_<cell>

# RoboDojo
env -u PYTHONPATH .venv/bin/python tools/fair_run_robodojo.py --task <task> \
  --program reproduce/sealed/robodojo_rd2/<cell>/program.py \
  --split eval --eval-n 50 --gpu 0 --out results/eval_<cell>
```

The BDDL file and sentence for each LIBERO-PRO cell are listed in
`autoresearch/campaigns/c2clean/eval_manifest.txt`.

### 2. Rerun the whole pipeline

Each experiment lives in `autoresearch/campaigns/<name>/` and runs in four steps:

1. **Build packs**: `tools/fair_pack.py` (LIBERO) or `tools/fair_pack_robodojo.py` (RoboDojo).
2. **Write workspaces**: `build_all.sh` creates one folder per cell with the task, pack, and API docs.
3. **Develop**: `run.sh` starts one headless Claude Code session per cell (model `claude-opus-5`).
   Set `ANTHROPIC_API_KEY` or `CLAUDE_CODE_OAUTH_TOKEN` in the environment or in a git-ignored `.env`.
4. **Evaluate once**: `run_eval.sh` records the program's hash, then runs the held-out evaluation.

Agents are not deterministic, so a rerun produces different programs and
slightly different numbers. Running the saved programs (level 1) reproduces the paper's figures.

---

## Where each result comes from

| Result in the paper | Saved results | Experiment folder |
|---|---|---|
| LIBERO-PRO, K=3 and K=0 (Fig. 4, Tables 3–4) | `reproduce/sealed/libero_pro_clean` | `c2clean` |
| LIBERO-PRO, K=1 (Tables 3–4) | `reproduce/sealed/libero_pro_k1` | `c2k1clean` |
| Unperturbed tasks (98.6%) | `reproduce/sealed/libero_stock` | `c2` |
| Development efficiency (Fig. 2) | `c2clean/metrics_per_cell.json` | `c2clean` |
| RoboDojo (Table 2) | `reproduce/sealed/robodojo_rd1`, `robodojo_rd2`, `robodojo_k1` | `rd1`, `rd2`, `k1rd` |
| Verification ablation (Table 5) | `reproduce/sealed/verification_ablation` | `abl_c2` |
| LIBERO-90 fixture study (App. C) | `reproduce/sealed/libero90_fixtures` | `l90abl` |
| ASPIRE rerun and ablations | `reproduce/aspire/` | `baselines/aspire/` |
| Real robot: cube handover, cup inversion | not simulated | `autoresearch/tasks/rig_handover`, `rig_flip_cup` |

---

## What is in the repository

| Folder | Contents |
|---|---|
| `reproduce/` | Every frozen program and its per-episode results |
| `tools/fair_run.py`, `tools/fair_client.py` | The evaluation harness. The program runs in its own process and reaches the simulator only through the API; no object poses or success signal cross that boundary. |
| `tools/fair_run_robodojo.py`, `tools/robodojo/` | The same harness for RoboDojo, and its installer |
| `tools/fair_pack*.py`, `tools/strip_pack.py` | Turn demonstrations into packs: keyframes, gripper events, frame strips, trajectories |
| `heron/` | Perception and robot library behind the API (grounding, depth, simulator and robot adapters) |
| `autoresearch/campaigns/` | One folder per experiment: task lists, agent briefs, run and evaluation scripts |
| `tools/rig_*.py`, `tools/calibrate_*.py` | Real-robot harness and camera calibration |
| `baselines/aspire/` | Our scripts for running the ASPIRE baseline |

---

## Setup

<details>
<summary>Python and simulators</summary>

- Python 3.10: `pip install -e .[dev]`
- **LIBERO-PRO** at commit `eafdb80`, with its perturbation `bddl_files` and `init_files`.
  Set the paths in `configs/libero.yaml`.
- **RoboDojo** at commit `ee67a14`, then run `tools/robodojo/install_encore.sh <robodojo-root>`.
  The installer adds our robot and camera configs and two small patches
  (gripper state in observations, and 50-episode evaluations instead of the default 25).

</details>

<details>
<summary>Vision-language model keys</summary>

- LIBERO uses Gemini Robotics-ER (`GEMINI_API_KEY`), set in `configs/libero.yaml`.
- RoboDojo uses `gemini-3.5-flash` through an OpenAI-compatible relay
  (`PPAPI_BASE_URL`, `PPAPI_API_KEY`), set in `configs/robodojo_ppapi.yaml`.

Put keys in a git-ignored `.env` file.

</details>

<details>
<summary>Real robot</summary>

Two Trossen WidowX AI arms and four RealSense D405 cameras: `pip install -e .[real,record]`,
`configs/rig5090.yaml`, and `tools/depth_server.py` for the cameras.
Two helper files that live on the robot host are not included yet:
`start_sam3.sh` (the segmentation service) and `tools/rig_cupstack_ep.sh`.

</details>

<details>
<summary>Tests</summary>

```bash
python -m pytest -q
```

</details>
