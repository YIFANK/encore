#!/bin/bash
for c in press_by_number_k3 press_by_number_vis press_by_number_k0 stack_blocks_k3 stack_blocks_vis stack_blocks_k0 swap_T_k3 swap_T_vis swap_T_k0 push_T_k3 push_T_vis push_T_k0 insert_tubes_k3 insert_tubes_vis insert_tubes_k0 play_tic_tac_toe_vis; do
  t=${c%_*}; nohup bash /mnt/data/YifanKang/tmp/rd2_eval50.sh $c $t > /dev/null 2>&1 < /dev/null &
  sleep 2
done
sleep 20; cat /mnt/data/YifanKang/tmp/rd2_eval50.log
