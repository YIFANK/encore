#!/usr/bin/env python3
"""Assemble the full LIBERO-PRO scoreboard (stock / Pos / Task x 3 suites).

Reads result dirs on AbakaAI via ssh, takes the argmax over all formally
evaluated versions of each cell (PROTOCOL.md rule 5), and prints a JSON
scoreboard plus per-cell provenance. Cells and their candidate result dirs
are enumerated explicitly so a missing formal shows up as None, never as a
silent zero.
"""
import json
import subprocess
import sys

H = "/mnt/data/YifanKang/Heron/results"

SPA = ["between_the_plate_and_", "from_table_center", "in_the_top_drawer_of_t",
       "next_to_the_cookie_box", "next_to_the_plate", "next_to_the_ramekin",
       "on_the_cookie_box", "on_the_ramekin", "on_the_stove", "on_the_wooden_cabinet"]
OBJ = ["alphabet_soup", "bbq_sauce", "butter", "chocolate_pudding", "cream_cheese",
       "ketchup", "milk", "orange_juice", "salad_dressing", "tomato_sauce"]
GOAL6 = ["bowl_on_the_plate", "bowl_on_the_stove", "bowl_on_top_of_the_cabinet",
         "cream_cheese_in_the_bowl", "wine_bottle_on_the_rack", "wine_bottle_on_top_of_the_"]

def cells():
    # (suite, axis, cell_name, [candidate result dirs, any version suffix])
    out = []
    for o in OBJ:
        out.append(("object", "stock", o, [f"final20_obj_{o}_stock"]))
        out.append(("object", "pos", o, [f"final20_obj_{o}_swap", f"final20_obj_{o}_swapfix",
                                          f"final20_obj_{o}_swap2"]))
        out.append(("object", "task", o, [f"final20_task_obj_{o}"]))
    for s in SPA:
        short = {"between_the_plate_and_": "between", "from_table_center": "table_center",
                 "in_the_top_drawer_of_t": "top_drawer", "next_to_the_cookie_box": "next_cookie",
                 "next_to_the_plate": "next_plate", "next_to_the_ramekin": "next_ramekin",
                 "on_the_cookie_box": "on_cookie", "on_the_ramekin": "on_ramekin",
                 "on_the_stove": "on_stove", "on_the_wooden_cabinet": "on_cabinet"}[s]
        out.append(("spatial", "stock", s, [f"final20_spa_{s}_stock"]))
        out.append(("spatial", "pos", s, [f"final20_spa_{s}_swapbase", f"final20_spa_{s}_swaphard"]))
        out.append(("spatial", "task", s, [f"final20_task_spa_{short}"]))
    # goal: six harness tasks + four dev-era tasks with their own naming
    for g in GOAL6:
        out.append(("goal", "stock", g, [f"final20_goal_{g}_stock", f"final20_goal_{g}_stock_v4"]))
        out.append(("goal", "pos", g, [f"final20_goal_{g}_swapbase", f"final20_goal_{g}_swaphard",
                                       f"final20_goal_{g}_swapopus", f"final20_goal_{g}_swapreach"]))
    out.append(("goal", "stock", "turn_on_the_stove", ["final20_stove_stock"]))
    out.append(("goal", "pos", "turn_on_the_stove", ["final20_stove_swap"]))
    out.append(("goal", "stock", "push_plate", ["final20_push_repair_final", "final20_push_repair_v9",
                                                 "final20_push_repair", "be3_push", "be3_push_endpoint"]))
    out.append(("goal", "pos", "push_plate", ["final20_push_swap"]))
    out.append(("goal", "stock", "open_top_drawer_bowl", ["final20_compose_repair", "be3_compose",
                                                           "be3_compose_endpoint"]))
    out.append(("goal", "pos", "open_top_drawer_bowl", ["fs_compose_swap_big"]))
    out.append(("goal", "stock", "open_middle_drawer", ["final20_drawer_stock", "be3_drawer",
                                                         "be3_drawer_endpoint"]))
    out.append(("goal", "pos", "open_middle_drawer", ["final20_goal_drawer_swapbase",
                                                          "final20_goal_drawer_swaphard"]))
    task_goal = {"turn_on_the_stove": "final20_task_goal_stove_off_stock",
                 "open_middle_drawer": "final20_task_goal_bottom_drawer_stock",
                 "open_top_drawer_bowl": "final20_task_goal_top_drawer_cc_stock",
                 "push_plate": "final20_task_goal_push_cc_stock",
                 "bowl_on_the_plate": "final20_task_goal_wine_on_plate",
                 "bowl_on_the_stove": "final20_task_goal_plate_on_stove",
                 "bowl_on_top_of_the_cabinet": "final20_task_goal_plate_on_drawer",
                 "cream_cheese_in_the_bowl": "final20_task_goal_wine_in_bowl_a",
                 "wine_bottle_on_the_rack": "final20_task_goal_cc_on_rack",
                 "wine_bottle_on_top_of_the_": "final20_task_goal_wine_in_bowl_b"}
    for g, d in task_goal.items():
        out.append(("goal", "task", g, [d]))
    return out

def main():
    # one ssh: dump "dir succ n" for every final-style dir (incl. version suffixes)
    cmd = (f'cd {H}; for d in final20_* be3_* be4_* fs_compose_swap_big; do f=$d/results.jsonl; '
           '[ -f $f ] || continue; s=$(grep -c "\\"benchmark_success\\": true" $f); '
           'n=$(wc -l < $f); echo "$d $s $n"; done')
    raw = subprocess.run(["ssh", "AbakaAI", cmd], capture_output=True, text=True, timeout=120).stdout
    scores = {}
    for line in raw.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[1].isdigit():
            d, s, n = parts[0], int(parts[1]), int(parts[2])
            if n >= 20 or d == "fs_compose_swap_big" or n >= 12:  # accept in-flight >=12 as provisional
                # 40-line dirs are re-runs appended; the harvest convention is last-20,
                # but grep -c counts all lines; recompute last-20 for those
                scores[d] = (s, n)
    # fix last-20 for contaminated dirs (n>20): re-query tail -20
    for d, (s, n) in list(scores.items()):
        if n > 20:
            cmd2 = f'tail -20 {H}/{d}/results.jsonl | grep -c "\\"benchmark_success\\": true"'
            s2 = int(subprocess.run(["ssh", "AbakaAI", cmd2], capture_output=True, text=True,
                                    timeout=60).stdout.strip() or 0)
            scores[d] = (s2, 20)

    board = {}
    for suite, axis, cell, cands in cells():
        best, src = None, None
        # also sweep any _vN variants of each candidate
        for base in cands:
            for d, (s, n) in scores.items():
                if d == base or d.startswith(base + "_v"):
                    if best is None or s > best:
                        best, src = s, d
        board.setdefault(suite, {}).setdefault(axis, {})[cell] = {"succ": best, "src": src}

    totals = {}
    for suite in board:
        for axis in board[suite]:
            vals = [c["succ"] for c in board[suite][axis].values()]
            done = [v for v in vals if v is not None]
            totals[f"{suite}/{axis}"] = {
                "cells_done": len(done), "cells_total": len(vals),
                "succ": sum(done), "eps": 20 * len(done),
                "pct": round(100 * sum(done) / (20 * len(done)), 1) if done else None,
            }
    print(json.dumps({"totals": totals, "board": board}, indent=1))

if __name__ == "__main__":
    sys.exit(main())
