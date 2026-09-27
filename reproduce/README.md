# Reproducing the Encore results

This directory contains everything needed to check and re-run the
simulation results of *Encore: Few-Shot Agentic Discovery of Manipulation
Strategies*.

- The repository root is the paper's experiment code (harness, distillers,
  rig tools, campaigns). The harness exactly as it ran on the evaluation box is
  commit `798e8f7` of our development repository, available on request.
- `reproduce/sealed/` holds the **frozen program and per-episode sealed results
  of every evaluation the paper reports** (319 evaluations).
- `autoresearch/campaigns/<campaign>/` holds each campaign's definition: cell
  lists, the agent brief generator, pre-registration, run and evaluation
  scripts, freeze records, aggregates, and each agent's own development notes
  (`declarations/`).

## 1. Check every number (no simulator needed)

```bash
python3 reproduce/verify.py
```

Standard library only. It recomputes each quantitative claim of the paper from
the bundled per-episode results and prints `51/51 claims reproduced`; with the paper
source at `../paper_ws_corl/main.tex` (or `PAPER_TEX=`) it also checks that the key
numbers appear verbatim in the text. It also
checks that every bundled program has the md5 recorded in `MANIFEST.tsv`, and
for 279 of the 319 evaluations that this md5 equals the one written to the
campaign's freeze record *before* the sealed evaluation ran. The remaining 40
(the unperturbed LIBERO-PRO column and the no-verification ablation) predate
freeze records; for those the evidence is the `program_archived.py` that the
evaluator copied into the result folder at evaluation time.

## 2. Where each result comes from

| Paper | `reproduce/sealed/<experiment>` | Campaign | Notes |
|---|---|---|---|
| LIBERO-PRO K=3 / K=0, Fig. 4, Tables 3-4 | `libero_pro_clean` (120) | `c2clean` | clean brief, nothing shared across cells |
| Unperturbed column (98.6%) | `libero_stock` (30) | `c2` | `c2/scoreboard_frozen.json` |
| Development efficiency, Fig. 2, App. B | from `c2clean/metrics_per_cell.json` | `c2clean` | `metrics.py` over agent transcripts |
| ASPIRE opus-5 rerun (89.3%), Fig. 4 | `reproduce/aspire/v4_60cells`, `goalswap_demos/armA_no_demos.json` | ASPIRE | 50 cells + goal-swap control arm = 2679/3000 |
| ASPIRE + three demonstrations | `reproduce/aspire/goalswap_demos` | ASPIRE | 431 vs 437 of 500 |
| ASPIRE planner / threshold ablation | `reproduce/aspire/planner_threshold_ablation` | ASPIRE | 18 cells shared with the control |
| Verification ablation, Table 5 (App. C) | `verification_ablation` (9) + 9 of `libero_stock` | `abl_c2` | earlier brief with a shared note file |
| LIBERO-90 fixture study | `libero90_fixtures` (24) | `l90abl` | earlier brief with a shared note file |
| RoboDojo, ten tasks, Table 2 left | `robodojo_rd1` (19) | `rd1` | 50 sealed episodes per cell |
| RoboDojo, three arms, Table 2 right | `robodojo_rd2` (36) | `rd2` | 50 episodes; six zero tasks at 20 |
| $K{=}1$ arm, LIBERO-PRO (Tables 3-4, Fig. 4) | `libero_pro_k1` (60) | `c2k1clean` | clean brief, one demonstration |
| $K{=}1$ arm, RoboDojo (Table 2) | `robodojo_k1` (21) | `k1rd` (workspaces in rd1/, rd2/) | 50 episodes |

`MANIFEST.tsv` maps every cell to the result folder it came from on the
evaluation box (`/mnt/data/YifanKang/Heron/results/<dir>`).

## 3. Re-run a sealed evaluation of a frozen program

Environment (Section 5) on a Linux box with an NVIDIA GPU. From the repository
root, with the benchmark checkouts at the paths in `configs/` and the tool
headers:

LIBERO-PRO and LIBERO-90 (bddl path and sentence per cell are in the campaign's
`eval_manifest.txt` or `cells.txt`):

```bash
env -u PYTHONPATH .venv/bin/python tools/fair_run.py program --seed-episodes \
  --bddl <bddl> --language "<sentence>" \
  --program reproduce/sealed/libero_pro_clean/<cell>/program.py \
  --split eval --out results/eval_<cell> --tmp-root <scratch-dir>
```

RoboDojo (Isaac Sim through the RoboDojo conda env; the bridge launches it):

```bash
env -u PYTHONPATH .venv/bin/python tools/fair_run_robodojo.py --task <task> \
  --program reproduce/sealed/robodojo_rd2/<cell>/program.py \
  --split eval --eval-n 50 --gpu 0 --out results/eval_<cell>
```

Evaluation seeds 1-50 (LIBERO) and the first 50 official evaluation layouts
(RoboDojo) are fixed, and a frozen program makes no language-model call except
the harness's grounding and VQA calls, so a re-run should land within a few
episodes of the bundled result; programs that call `api.ground` / `api.vqa`
depend on the vision-language model behind them (Section 6).

## 4. Re-run development (acquisition)

Each campaign's `mkworker.sh` / `build_all.sh` writes one agent workspace per
cell, `run.sh` launches fresh Claude Code sessions (model `claude-opus-5`),
and `run_eval.sh` (or `cluster/*.sh` for RoboDojo) freezes and evaluates the
declared program once. Demonstration packs are built on the evaluation box by
`tools/fair_pack.py` (LIBERO), `tools/fair_pack_robodojo.py` (RoboDojo, with
`--images-only` for the images arm), and `tools/l90abl_build_packs.sh`.
Development is a fresh stochastic agent session per cell, so a re-acquisition
reproduces the distribution, not the exact program: the paper's Limitations
and the `c2clean` notes document one gap that did not survive re-acquisition.

## 5. Environment

`reproduce/env/`:

- `heron_venv_freeze.txt` - the harness venv (Python 3.10.12) that ran `tools/fair_run*.py`.
- `robodojo_conda_freeze.txt` - the RoboDojo conda env (Python 3.11.16, Isaac Sim client).
- `dpofficial_venv_freeze.txt` - the diffusion-policy venv used by earlier LIBERO tooling.
- `robodojo_version.txt` - RoboDojo commit `ee67a14` and submodule commits
  (XPolicyLab, IsaacLab, curobo); `robodojo_local.patch` - our two local edits
  (the `EVAL_NUM` override in `utils/pipeline_utils.py` so 50-episode evaluations
  are not truncated to a task's default count, and an observation-manager
  change); `robodojo_env_cfg_encore.tgz` - the Encore robot/camera/sim configs;
  `install_all.sh`, `dl_tasks.sh`, `assets_dl.sh` - the install recipe used.
- `libero_version.txt` - LIBERO-PRO commit `eafdb80` plus the sha256 of its
  perturbation `bddl_files/` and `init_files/` (160 files), and the GPU/driver.
- ASPIRE: github.com/NVlabs/ASPIRE at commit `680bad4`; our coordinator and
  wrapper scripts are in `reproduce/aspire/`.

Hardware: 8x A100-SXM4-80GB, driver 580.159.03.

## 6. Known deviations

- **Grounding model.** RoboDojo programs call the harness's grounding/VQA,
  served by Gemini Robotics-ER 2 (`configs/libero.yaml`). Its prepaid credit ran
  out on 2026-09-24 02:05 (box time); six evaluations that call the VLM were
  then voided and re-run with `gemini-3.5-flash` through an OpenAI-compatible
  relay (`configs/robodojo_ppapi.yaml`, now the bridge default): press-by-number
  x3 and push-T K=0 / images (50 episodes), and store-laptop K=0 (20 episodes).
  On a pointing probe the relay model is within 1-2 px; GPT models were 17-127 px
  off and were not used.
- **Earlier brief.** The verification ablation and the LIBERO-90 study ran
  before the shared note file was removed from the agent brief (the paper says
  so); LIBERO-PRO main and RoboDojo use the clean brief.
- `autoresearch/campaigns/astra_direct/` is a GPT-6-Astra direct-policy
  baseline that is not in the paper; its `compare.py` prints ASPIRE as
  2678/3000, one below the recount (2679).

## 7. Not in this repository

- Agent session transcripts (large; they also carry session tokens). They
  are archived with the campaign workspaces and are what
  `c2clean/metrics.py` reads.
- Demonstration packs and keyframe images (regenerated by the pack builders
  from the benchmarks' own demonstration datasets).
- Per-episode videos.
- The real-robot experiment (bimanual Trossen rig): its code lives on the main
  development line (`claude/verification-and-call-budget`), and its bands are
  human-judged.
