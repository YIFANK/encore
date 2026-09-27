#!/bin/bash
# l90abl packs — ON THE CLUSTER. K=3 pack from the LIBERO-90 demo hdf5 (first 3
# demos, images from the dataset), then a vision-only copy (keyframe RGB + language).
set -uo pipefail
cd /mnt/data/YifanKang/Heron
while IFS='|' read -r task bddl h lang grp; do
  [ -z "$task" ] && continue; [ -s "$h" ] || { echo "MISSING $h"; continue; }
  k3=packs/l90abl_${task}_k3; vis=packs/l90abl_${task}_vis
  [ -d "$k3" ] || { env -u PYTHONPATH .venv/bin/python tools/fair_pack.py --hdf5 "$h" --k 3 --language "$lang" --out "$k3" > /mnt/data/YifanKang/l90abl_pack_${task}_k3.log 2>&1 && echo "built $k3" || echo "FAIL $k3"; }
  if [ -d "$k3" ] && [ ! -d "$vis" ]; then cp -r "$k3" "$vis" && env -u PYTHONPATH .venv/bin/python tools/fair_pack_strip.py --pack "$vis" --mode vision > /mnt/data/YifanKang/l90abl_pack_${task}_vis.log 2>&1 && env -u PYTHONPATH .venv/bin/python tools/fair_pack_strip.py --pack "$vis" --mode vision --audit >> /mnt/data/YifanKang/l90abl_pack_${task}_vis.log 2>&1 && echo "built $vis (audited)" || echo "FAIL $vis"; fi
done < /mnt/data/YifanKang/l90abl_cells.txt
echo "programs inside l90abl packs (must be 0): $(find packs -maxdepth 2 -path 'packs/l90abl_*' -name 'program*.py' | wc -l)"
