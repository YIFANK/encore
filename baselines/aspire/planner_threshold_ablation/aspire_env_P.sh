# ASPIRE A/B session env — source this in EVERY shell on AbakaAI.
export ASPIRE_ROOT=/mnt/data/YifanKang/ASPIRE_ablP/aspire/sim
export PYTHON_ROOT=/mnt/data/YifanKang/ASPIRE_ablP
export PYTHONPATH="$PYTHON_ROOT"
export MUJOCO_GL=egl
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1
# root disk is 100% full -- every cache off /
export UV_CACHE_DIR=/mnt/data/YifanKang/cache/uv
export XDG_CACHE_HOME=/mnt/data/YifanKang/.cache
export TMPDIR=/mnt/data/YifanKang/tmp
export HF_HOME=/mnt/data/YifanKang/cache/hf
export ROBOT_DESCRIPTIONS_CACHE=/mnt/data/YifanKang/.cache/robot_descriptions
# this box exports HF_HUB_OFFLINE=1 / TRANSFORMERS_OFFLINE=1 into every ssh
# session (inherited, not from ~/.bashrc); they silently break every hf_hub_download.
unset HF_HUB_OFFLINE TRANSFORMERS_OFFLINE
export HF_HUB_ETAG_TIMEOUT=60
export HF_ENDPOINT=https://hf-mirror.com
export HF_TOKEN=$(cat /mnt/data/YifanKang/cache/hf/token 2>/dev/null)
export HUGGING_FACE_HUB_TOKEN="$HF_TOKEN"
mkdir -p "$TMPDIR" "$XDG_CACHE_HOME" "$ROBOT_DESCRIPTIONS_CACHE" 2>/dev/null
cd "$ASPIRE_ROOT"
