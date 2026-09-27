#!/bin/bash
R=/mnt/data/YifanKang/robodojo
export HOME=$R/home PATH=$R/bin:$PATH GIT_TERMINAL_PROMPT=0 GIT_LFS_SKIP_SMUDGE=1
export HF_REVISION=master HF_REPO_URL=https://www.modelscope.cn/datasets/RoboDojo-Benchmark/RoboDojo.git
cd $R/RoboDojo
echo "=== $(date) assets via modelscope"
bash scripts/init_assets.sh < /dev/null 2>&1 | tail -30
echo "=== $(date) rc=${PIPESTATUS[0]}"
du -sh .cache/robodojo_assets_repo 2>/dev/null
