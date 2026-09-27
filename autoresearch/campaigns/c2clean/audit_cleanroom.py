#!/usr/bin/env python3
"""c2fix cleanroom audit.

Each worker may touch: its own workspace, packs/c2clean_<cell>*/ (own pack, own
mate pack, and its own delivered program there), its own fs_/sel_ result dirs.
Flagged: any reference to old-namespace packs (c1_/c2_/c2k0_), another cell's
c2fix pack, another cell's workspace, or eval result dirs. READ = a verb that
opens content; mention = the string appears (e.g. in a listing) — context call.
"""
import glob, json, os, re, sys

VERB = r"(cat|head|tail|less|more|sed|awk|grep|open\(|read_text|json\.load|load\(|scp|rsync|cp|diff)\b"
flags, scanned = [], 0
for ws in sorted(glob.glob("workers/*/")):
    cell = os.path.basename(ws.rstrip("/"))
    own = re.compile(rf"packs/c2clean_{re.escape(cell)}(_mate)?/")
    foreign_pack = re.compile(r"packs/(c1_|c2_|c2k0_|c2clean_)[A-Za-z0-9_]+")
    eval_dir = re.compile(r"results/eval_")
    other_ws = re.compile(r"workers/(?!" + re.escape(cell) + r"/)[a-z_]+/")
    for f in sorted(glob.glob(ws + "transcript_*.jsonl")):
        scanned += 1
        for line in open(f, errors="replace"):
            if "packs/" not in line and "results/eval_" not in line and "workers/" not in line:
                continue
            try: d = json.loads(line)
            except Exception: continue
            m = d.get("message"); c = m.get("content") if isinstance(m, dict) else None
            if not isinstance(c, list): continue
            for b in c:
                if b.get("type") != "tool_use": continue
                blob = json.dumps(b.get("input", {}))
                hits = set()
                for mt in foreign_pack.finditer(blob):
                    seg = mt.group(0)
                    if own.match(seg + "/") or seg.startswith(f"packs/c2clean_{cell}"): continue
                    hits.add(seg)
                if eval_dir.search(blob): hits.add("results/eval_*")
                if other_ws.search(blob): hits.add("other-worker-ws")
                if hits:
                    kind = "READ" if re.search(VERB + r"[^\n]*(" + "|".join(re.escape(h) for h in hits if h != "other-worker-ws") + ")", blob) and any(h != "other-worker-ws" for h in hits) else "mention"
                    flags.append((cell, kind, sorted(hits), blob[:160]))
print(f"scanned {scanned} transcripts across {len(glob.glob('workers/*/'))} cells")
reads = [f for f in flags if f[1] == "READ"]
print(f"READs: {len(reads)}  mentions: {len(flags)-len(reads)}")
for cell, kind, hits, blob in flags[:30]:
    print(f"[{kind}] {cell} {hits}\n    {blob}")
sys.exit(1 if reads else 0)
