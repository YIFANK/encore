#!/bin/bash
# abl_c2 coordinator-only blind sealed eval of an ARBITRARY archived program
# (the variant-C "archived v1" extension — see NOTES.md R17).
#   run_eval_prog.sh <eval_label> <abs_program_path_on_cluster> <bddl> <intent> <gpu>
# Same gate as run_eval.sh: fair_run --split eval, seeds 1-50, published only
# after episode 50, PROVENANCE/token/AST gate enforced by the runner.
set -euo pipefail
LABEL=$1; PROG=$2; BDDL=$3; INTENT=$4; GPU=$5
R=/mnt/data/YifanKang/Heron

ssh AbakaAI "cd $R && env -u PYTHONPATH CUDA_VISIBLE_DEVICES=$GPU setsid nohup \
  .venv/bin/python tools/fair_run.py program --seed-episodes \
  --bddl $BDDL --language \"$INTENT\" \
  --program $PROG \
  --split eval --out results/eval_$LABEL \
  --gt-dir /mnt/data/YifanKang/gt_private/eval_$LABEL \
  > $R/results/eval_$LABEL.out 2>&1 </dev/null &"
echo "[abl-eval] launched $LABEL on GPU $GPU -> results/eval_$LABEL  (program $PROG)"
