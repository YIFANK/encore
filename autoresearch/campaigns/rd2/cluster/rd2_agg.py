import json,glob,os
for f in sorted(glob.glob('/mnt/data/YifanKang/Heron/results/eval_rd2_*/results.jsonl')):
    d=os.path.basename(os.path.dirname(f))
    if 'incomplete' in d: continue
    rs=[json.loads(l) for l in open(f)]; n=len(rs)
    print(f"{d[9:]:34s} {sum(bool(r.get('benchmark_success')) for r in rs):2d}/{n}  score {sum(float(r.get('score',0)) for r in rs)/n:.2f}  unstable {sum(1 for r in rs if 'missing' in str(r.get('judge','')))}  err {sum(1 for r in rs if r.get('error'))}  steps_med {sorted(int(r.get('sim_steps',0)) for r in rs)[n//2]}")
