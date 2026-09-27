#!/usr/bin/env bash
# Start ASPIRE perception servers. Deviations from their
# scripts/common/start_perception_servers.sh, both forced by this host:
#   - logs to /mnt/data/YifanKang/logs (their script hardcodes /tmp; / is 100% full)
#   - SAM3=GPU1, GraspNet=GPU2 (their 0/1; GPUs 0 and 7 are other tenants')
# No pkill: ports 8114-8116 were verified free, and their script's blanket
# pkill would kill another project's servers on this shared box.
source /mnt/data/YifanKang/aspire_env.sh
LOGDIR=/mnt/data/YifanKang/logs; mkdir -p "$LOGDIR"
PY=.venv-libero/bin/python3

bash scripts/common/apply_contact_graspnet_patch.sh

CUDA_VISIBLE_DEVICES=1 nohup "$PY" -u cap/serving/launch_sam3_server.py \
  --device cuda --port 8114 --host 127.0.0.1 > "$LOGDIR/sam3.log" 2>&1 &
echo "SAM3 pid $!"
CUDA_VISIBLE_DEVICES=2 nohup "$PY" -u cap/serving/launch_contact_graspnet_server.py \
  --port 8115 --host 127.0.0.1 > "$LOGDIR/graspnet.log" 2>&1 &
echo "GraspNet pid $!"
nohup "$PY" -u cap/serving/launch_pyroki_server.py \
  --port 8116 --host 127.0.0.1 --robot panda_description --target-link panda_hand \
  > "$LOGDIR/pyroki.log" 2>&1 &
echo "PyRoKi pid $!"
