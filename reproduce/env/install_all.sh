#!/bin/bash
# RoboDojo native install without sudo, everything under /mnt/data (home fs is full).
set -uo pipefail
R=/mnt/data/YifanKang/robodojo
export HOME=$R/home XDG_CACHE_HOME=$R/home/.cache PIP_CACHE_DIR=$R/home/.cache/pip TMPDIR=$R/home/tmp
export PATH=$R/bin:$PATH TERM=xterm-256color OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y PRIVACY_CONSENT=Y
mkdir -p $TMPDIR
cd $R
echo "=== $(date) start"
if ! git lfs version >/dev/null 2>&1; then
  echo "=== git-lfs"; V=3.6.1; curl -sL -o /tmp/gl.tgz https://github.com/git-lfs/git-lfs/releases/download/v$V/git-lfs-linux-amd64-v$V.tar.gz && tar -xzf /tmp/gl.tgz -C $TMPDIR && cp $TMPDIR/git-lfs-$V/git-lfs $R/bin/ && git lfs version
fi
cd $R/RoboDojo
echo "=== install.sh --from conda ($(date))"
bash scripts/install.sh --from conda 2>&1 | tail -n 400 > $R/install_core.log; rc=${PIPESTATUS[0]}; echo "install.sh rc=$rc"
source $HOME/miniconda3/bin/activate RoboDojo
echo "=== conda-forge system libs ($(date))"
conda install -y -c conda-forge libvulkan-loader vulkan-tools ffmpeg libglu 2>&1 | tail -5
python -c "import isaacsim, isaaclab; print(isaacsim+isaaclab import ok)" 2>&1 | tail -3
echo "=== assets ($(date))"
bash scripts/init_assets.sh 2>&1 | tail -20
python utils/update_embodiment_config_path.py 2>&1 | tail -3
du -sh $R/RoboDojo/.cache/robodojo_assets_repo 2>/dev/null
echo "=== $(date) done"
