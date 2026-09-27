#!/bin/bash
# K=1 packs for the clean K=1 arm (2026-09-25). Program-free, fresh directories.
#  LIBERO-PRO: packs/c2k1clean_<cell>_k1 (+ _mate for Task cells), first demo of the stock task's hdf5,
#              exactly as c2clean's K=3 packs but --k 1.
#  RoboDojo:   packs/rd_<task>_k1 (rd1 tasks), packs/rd2_<task>_k1 (rd2 tasks), episode_0000000 only.
set -uo pipefail
cd /mnt/data/YifanKang/Heron
STOCK=/mnt/data/YifanKang/c2fix_stock_manifest.txt; MATES=/mnt/data/YifanKang/c2fix_skillmate.txt; CELLS=/mnt/data/YifanKang/c2fix_cells.txt
L=/mnt/data/YifanKang/tmp/k1_packs; mkdir -p $L
hdf5_for() { awk -F'|' -v b="$1" '$1==b{print $3}' "$STOCK"; }
ok=0; bad=0
while read -r cell; do
  [ -z "$cell" ] && continue; base=${cell%_pos}; base=${base%_task}
  out=packs/c2k1clean_${cell}_k1
  [ -d "$out" ] || { env -u PYTHONPATH .venv/bin/python tools/fair_pack.py --hdf5 "$(hdf5_for "$base")" --k 1 --out "$out" > $L/$cell.log 2>&1 && ok=$((ok+1)) || { echo "FAIL $cell"; bad=$((bad+1)); }; }
  case "$cell" in *_task) mate=$(awk -F'|' -v c="$cell" '$1==c{print $2}' "$MATES")
    if [ -n "$mate" ] && [ "$mate" != NONE ] && [ ! -d packs/c2k1clean_${cell}_mate ]; then
      env -u PYTHONPATH .venv/bin/python tools/fair_pack.py --hdf5 "$(hdf5_for "$mate")" --k 1 --out packs/c2k1clean_${cell}_mate > $L/${cell}_mate.log 2>&1 && ok=$((ok+1)) || { echo "FAIL ${cell}_mate"; bad=$((bad+1)); }
    fi;; esac
done < "$CELLS"
R=/mnt/data/YifanKang/robodojo/data
for t in put_bottles_into_dustbin organize_table imitate_sorting_sequence arrange_largest_number pack_objects_into_box classify_objects build_tower make_kong fold_clothes; do
  [ -d packs/rd_${t}_k1 ] || { env -u PYTHONPATH .venv/bin/python tools/fair_pack_robodojo.py --episodes $R/$t/episode_0000000.hdf5 --out packs/rd_${t}_k1 > $L/rd_$t.log 2>&1 && ok=$((ok+1)) || { echo "FAIL rd_$t"; bad=$((bad+1)); }; }
done
for t in plug_in_charger insert_tubes push_T swap_T stack_blocks press_by_number make_toast store_laptop_and_headphones play_tic_tac_toe insert_key fasten_screws hang_mugs; do
  [ -d packs/rd2_${t}_k1 ] || { env -u PYTHONPATH .venv/bin/python tools/fair_pack_robodojo.py --episodes $R/$t/episode_0000000.hdf5 --out packs/rd2_${t}_k1 > $L/rd2_$t.log 2>&1 && ok=$((ok+1)) || { echo "FAIL rd2_$t"; bad=$((bad+1)); }; }
done
echo "built=$ok failed=$bad"
echo "programs inside k1 packs (must be 0): $(find packs -maxdepth 2 \( -path 'packs/c2k1clean_*' -o -path 'packs/rd_*_k1' -o -path 'packs/rd2_*_k1' \) -name 'program*.py' | wc -l)"
python3 - <<'PY'
import json,glob
bad=[p for p in glob.glob('packs/c2k1clean_*/pack.json')+glob.glob('packs/rd*_k1/pack.json') if len(json.load(open(p)).get('demos',[]))!=1]
print("packs with demos != 1:", bad)
PY
