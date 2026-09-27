#!/usr/bin/env python3
import json, glob, os
out = {"evals": {}, "staging_incomplete": []}
for d in sorted(glob.glob("results/eval_*c2_*")):
    if not os.path.isdir(d):
        continue
    rj = os.path.join(d, "results.jsonl")
    if not os.path.exists(rj):
        out["evals"][os.path.basename(d)] = {"success": None, "total": 0, "note": "no results.jsonl"}
        continue
    s = t = 0
    for line in open(rj):
        t += 1
        if '"benchmark_success": true' in line:
            s += 1
    out["evals"][os.path.basename(d)] = {"success": s, "total": t}
for s in sorted(glob.glob("results/.eval_*.staging")):
    out["staging_incomplete"].append(os.path.basename(s))
print(json.dumps(out, indent=1))
