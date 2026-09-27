import glob, json, os, re, sys
EVAL="outputs/libero_fix_loop_eval"
cells=[l.strip().split("/") for f in sys.argv[1:] for l in open(f) if l.strip()]
out={}
for s,t in cells:
    ms=sorted(glob.glob(os.path.join(EVAL,s,t,"runs","*","manifest.json")))
    rec={"n_runs":len(ms)}
    for m in ms:
        d=json.load(open(m)); R=d.get("results",{})
        # independent recount: from per-seed records AND from the trial_dir name string
        tc=sum(1 for v in R.values() if v.get("task_completed")==1)
        name=sum(1 for v in R.values() if re.search(r"taskcompleted_1$", os.path.basename(v.get("trial_dir",""))))
        seeds=sorted(int(k) for k in R)
        rec.update(manifest_passes=d.get("passes"), manifest_trials=d.get("trials"),
                   recount_task_completed=tc, recount_from_dirname=name,
                   n_seeds=len(seeds), seeds_exactly_1_50=(seeds==list(range(1,51))),
                   status=d.get("status"), scope=d.get("evidence_scope"),
                   model_dir=sorted({p.split("/run/")[0].split("/")[-1] for p in (v.get("trial_dir","") for v in R.values()) if "/run/" in p}))
    out[f"{s}/{t}"]=rec
print("JS"+json.dumps(out)+"JE")
