#!/bin/bash
# abl_c2 coordinator-only blind sealed eval (seeds 1-50, run ONCE per cell).
#   run_eval.sh <cell> <gpu>
# Workers never run this. fair_run --split eval stages every artifact privately
# and publishes to --out only after episode 50, so a half-finished eval can
# never be iterated on.
set -euo pipefail
CELL=$1; GPU=$2
AC=$(cd "$(dirname "$0")" && pwd)
LINE=$(grep "^${CELL}|" "$AC/cell_manifest.txt")
BDDL=$(echo "$LINE" | cut -d'|' -f2)
INTENT=$(echo "$LINE" | cut -d'|' -f3)
R=/mnt/data/YifanKang/Heron

ssh AbakaAI "cd $R && env -u PYTHONPATH CUDA_VISIBLE_DEVICES=$GPU setsid nohup \
  .venv/bin/python tools/fair_run.py program --seed-episodes \
  --bddl $BDDL --language \"$INTENT\" \
  --program $R/packs/$CELL/program.py \
  --split eval --out results/eval_$CELL \
  --gt-dir /mnt/data/YifanKang/gt_private/eval_$CELL \
  > $R/results/eval_$CELL.out 2>&1 </dev/null &"
echo "[abl-eval] launched $CELL on GPU $GPU -> results/eval_$CELL"
