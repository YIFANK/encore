#!/bin/bash
# Install the Encore adapter files into a RoboDojo checkout (run on the cluster).
set -euo pipefail
RD=${1:-/mnt/data/YifanKang/robodojo/RoboDojo}
HERE="$(cd "$(dirname "$0")" && pwd)"
cp "$HERE/arx_x5_encore.yml" "$RD/env_cfg/arx_x5_encore.yml"
cp "$HERE/sim_config_encore.yml" "$RD/env_cfg/sim/sim_config_encore.yml"
python3 - "$RD" <<'PY'
import sys, pathlib, re
rd = pathlib.Path(sys.argv[1])
src = (rd / "env_cfg/camera/camera_config.yml").read_text()
# enable the distance annotator on every camera block (it is commented out upstream)
dst = re.sub(r"    # distance_to_image_plane_capture:\n    #   type: distance_to_image_plane\n    #   device: cpu",
             "    distance_to_image_plane_capture:\n      type: distance_to_image_plane\n      device: cpu", src)
assert dst != src, "camera_config.yml: commented distance annotator block not found"
(rd / "env_cfg/camera/camera_config_encore.yml").write_text(dst)
print("camera_config_encore.yml written")
PY
mkdir -p "$RD/XPolicyLab/policy/Encore"
touch "$RD/XPolicyLab/policy/Encore/__init__.py"
cp "$HERE/deploy.yml" "$HERE/deploy.py" "$HERE/model.py" "$RD/XPolicyLab/policy/Encore/"
python3 "$HERE/patch_obs_manager.py" "$RD"
python3 "$HERE/patch_pipeline_evalnum.py" "$RD"
# Eval_Layout for the new config name is created per band by fair_run_robodojo.py
mkdir -p "$RD/Assets/Eval_Layout/RoboDojo/arx_x5_encore" 2>/dev/null || true
echo "[install_encore] done"
