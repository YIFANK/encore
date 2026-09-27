#!/bin/bash
# Create an autoresearch task workspace.
#   tools/ar_new_task.sh <name> <bddl_rel> <demo_hdf5_cluster_path> "<language>"
# <bddl_rel> is relative to the LIBERO-PRO bddl_files dir, e.g.
#   libero_10/LIVING_ROOM_SCENE2_put_both_the_alphabet_soup_and_the_tomato_sauce_in_the_basket
set -euo pipefail
NAME=$1; BDDL=$2; HDF5=$3; LANG=$4
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WS=$ROOT/autoresearch/tasks/$NAME
L=/mnt/data/YifanKang/LIBERO-PRO/libero/libero
H=/mnt/data/YifanKang/Heron

mkdir -p "$WS/reference" "$WS/.claude"

# Pack extraction on the cluster (idempotent), then fetch pack.json locally.
ssh AbakaAI "cd $H && [ -f packs/$NAME/pack.json ] || .venv/bin/python tools/fewshot_pack.py --hdf5 $HDF5 --k 3 --out packs/$NAME" >/dev/null
mkdir -p "$WS/packs/$NAME"
scp -q AbakaAI:$H/packs/$NAME/pack.json "$WS/packs/$NAME/"

cp "$ROOT/autoresearch/LAWS.md" "$WS/LAWS.md"
cp "$ROOT/autoresearch/PROTOCOL.md" "$WS/CLAUDE.md"
cp "$ROOT/packs/l2_put_both_the_cream_ch/program.py" "$WS/reference/program_l2c.py"

cat > "$WS/TASK.md" <<EOF
# Task: $NAME

Language intent: "$LANG"

- bddl:  $L/bddl_files/$BDDL.bddl
- inits: $L/init_files/$BDDL.pruned_init
- demos: $HDF5  (pack already extracted to cluster packs/$NAME/)
- probe:
  ssh AbakaAI 'H=$H; L=$L; cd \$H && MUJOCO_EGL_DEVICE_ID=<gpu> \$H/.venv/bin/python \$H/tools/fewshot_run.py program --bddl \$L/bddl_files/$BDDL.bddl --init-states \$L/init_files/$BDDL.pruned_init --program \$H/packs/$NAME/program.py --reward \$H/packs/$NAME/reward.py --episode-list 0,7,14,3,11 --out \$H/results/fs_${NAME}_vN'
- formal: same command with --episodes 20, out final20_${NAME}_stock
- replay fidelity check: fewshot_run replay --hdf5 $HDF5 --k 3
- pick a GPU with low load from: nvidia-smi (avoid heavily used ones)

Follow CLAUDE.md (protocol) and LAWS.md. Your deliverable is the banked
20-episode number plus NOTES.md and WALLCLOCK.md.
EOF

cat > "$WS/.claude/settings.json" <<'EOF'
{
  "permissions": {
    "allow": [
      "Bash", "Read", "Edit", "Write", "Grep", "Glob", "TodoWrite", "WebFetch"
    ]
  }
}
EOF

touch "$WS/NOTES.md" "$WS/WALLCLOCK.md"
echo "workspace ready: $WS"
