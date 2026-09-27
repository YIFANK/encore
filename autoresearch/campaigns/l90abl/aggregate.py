#!/usr/bin/env python3
"""l90abl: collect sealed-eval counts per cell and print the open/control group tables."""
import subprocess, json, os
HERE=os.path.dirname(os.path.abspath(__file__))
rows=[l.rstrip("\n").split("|") for l in open(f"{HERE}/cells.txt") if l.strip()]
tasks=[(r[0], r[4]) for r in rows]
cells=[f"{t}_{a}" for t,_ in tasks for a in ("k3","vis","k0")]
cmd="cd /mnt/data/YifanKang/Heron/results; for c in %s; do f=eval_l90abl_$c/results.jsonl; [ -s $f ] && echo \"$c $(grep -c '\"benchmark_success\": true' $f) $(wc -l < $f)\"; done" % " ".join(cells)
out=subprocess.run(["ssh","-n","AbakaAI",cmd],capture_output=True,text=True).stdout
r={}
for l in out.splitlines():
    p=l.split()
    if len(p)==3 and p[0] in cells: r[p[0]]=(int(p[1]),int(p[2]))
json.dump(r,open(f"{HERE}/final_counts.json","w"),indent=1)
print(f"sealed cells collected: {len(r)}/{len(cells)}")
fmt=lambda x: f"{x[0]:3d}/{x[1]:<3d}" if x else "   -   "
for grp in ("open","control"):
    print(f"\n[{grp}]  {'task':20s} {'k3':>7s} {'vis':>7s} {'k0':>7s}   {'k3-vis':>7s} {'k3-k0':>6s}")
    for t,g in tasks:
        if g!=grp: continue
        c={a:r.get(f"{t}_{a}") for a in ("k3","vis","k0")}
        d=lambda a,b: f"{c[a][0]-c[b][0]:+d}" if c[a] and c[b] else "-"
        print(f"         {t:20s} {fmt(c['k3']):>7s} {fmt(c['vis']):>7s} {fmt(c['k0']):>7s}   {d('k3','vis'):>7s} {d('k3','k0'):>6s}")
