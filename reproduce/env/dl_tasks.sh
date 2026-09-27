#!/bin/bash
R=/mnt/data/YifanKang/robodojo
for t in organize_table classify_objects_by_language imitate_sorting_sequence arrange_largest_number pack_objects_into_box classify_objects build_tower make_kong fold_clothes; do
  D=$R/data/$t; mkdir -p $D
  for i in 0 1 2; do f=episode_000000${i}.hdf5; [ -s $D/$f ] && continue
    curl -fsSL --retry 3 -o $D/$f "https://hf-mirror.com/datasets/RoboDojo-Benchmark/RoboDojo/resolve/main/data/RoboDojo/$t/arx_x5/data/$f"; echo "$t $f rc=$? $(stat -c %s $D/$f 2>/dev/null)" >> $R/dl_tasks.log
  done
done
echo "ALL DONE $(date)" >> $R/dl_tasks.log
