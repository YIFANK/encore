#!/usr/bin/env python3
"""Appendix result tables, generated from campaign records:
- macro table mirroring the bars figure (baselines transcribed from the same
  sources as the figure; Encore columns recomputed from final_counts.json)
- per-task breakdown over the 30 LIBERO-PRO tasks x {Pos,Task} x {K=3,K=0},
  plus the unperturbed (stock) column from campaign c2 (never granted).
"""
import json, os, sys
REPO=os.path.abspath(os.path.join(os.path.dirname(__file__),"..","..",".."))
d=json.load(open(os.path.join(REPO,"autoresearch","campaigns","c2fix","final_counts.json")))
sb=json.load(open(os.path.join(REPO,"autoresearch","campaigns","c2","scoreboard_frozen.json")))["evals"]
V11={"obj_butter","obj_chocolate_pudding","goal_turn_on_stove","goal_bowl_on_plate",
     "spa_bowl_between","spa_bowl_next_to_plate","obj_alphabet_soup","spa_bowl_on_stove"}
def stock(base):
    for key in (f"eval_v11_c2_{base}_stock", f"eval_v111_c2_{base}_stock"):
        if key in sb: return sb[key]["success"], sb[key]["total"]
    alias={"goal_put_bowl_on_plate":"goal_bowl_on_plate"}
    b=alias.get(base)
    if b:
        for key in (f"eval_v11_c2_{b}_stock", f"eval_v111_c2_{b}_stock"):
            if key in sb: return sb[key]["success"], sb[key]["total"]
    return None
cells=sorted(d["c2fix"])
bases=sorted({c[:-4] if c.endswith("_pos") else c[:-5] for c in cells})
BASE={  # baselines as in the bars figure: [pi05, cap0, aspire46, aspire5]
 "obj":{"pos":[17,22,98,100],"task":[1,18,95,100]},
 "goal":{"pos":[38,26,81,86],"task":[0,17,45,86]},
 "spa":{"pos":[20,12,51,84],"task":[1,14,60,80]},
}
def enc(camp,suite,ax):
    cs=[c for c in cells if c.startswith(suite) and c.endswith(ax)]
    return 100*sum(d[camp][c][0] for c in cs)/(50*len(cs))
# ---- macro table
zero=lambda n: n+" & "+" & ".join(["0.00"]*8)+" \\\\"
rows=[("$\\pi_{0.5}$",0),("CaP-Agent0",1),("ASPIRE (opus-4.6)",2),("ASPIRE (opus-5, our rerun)",3)]
L=[]
L.append("\\begin{tabular}{lcccccccc}")
L.append("\\toprule")
L.append(" & \\multicolumn{2}{c}{libero-object} & \\multicolumn{2}{c}{libero-goal} & \\multicolumn{2}{c}{libero-spatial} & \\multicolumn{2}{c}{Overall} \\\\")
L.append("\\cmidrule(lr){2-3}\\cmidrule(lr){4-5}\\cmidrule(lr){6-7}\\cmidrule(lr){8-9}")
L.append("Method & Pos & Task & Pos & Task & Pos & Task & Pos & Task \\\\")
L.append("\\midrule")
L.append(zero("OpenVLA")); L.append(zero("$\\pi_0$"))
for name,i in rows:
    vals=[]
    for s in ("obj","goal","spa"):
        vals+= [BASE[s]["pos"][i], BASE[s]["task"][i]]
    op=sum(BASE[s]["pos"][i] for s in ("obj","goal","spa"))/3
    ot=sum(BASE[s]["task"][i] for s in ("obj","goal","spa"))/3
    L.append(name+" & "+" & ".join(f"{v/100:.2f}" for v in vals+[op,ot])+" \\\\")
for camp,label in (("c2k0","\\method{} ($K{=}0$)"),("c2fix","\\method{} ($K{=}3$)")):
    vals=[]
    for s in ("obj","goal","spa"):
        vals+= [enc(camp,s,"_pos"), enc(camp,s,"_task")]
    op=sum(enc(camp,s,"_pos") for s in ("obj","goal","spa"))/3
    ot=sum(enc(camp,s,"_task") for s in ("obj","goal","spa"))/3
    fmt=lambda v: f"\\textbf{{{v/100:.2f}}}" if camp=="c2fix" else f"{v/100:.2f}"
    L.append(label+" & "+" & ".join(fmt(v) for v in vals+[op,ot])+" \\\\")
L.append("\\bottomrule"); L.append("\\end{tabular}")
open("tables_macro.tex","w").write("\n".join(L))
# ---- per-task table
def nice(b):
    return b.replace("_"," ").replace("goal ","").replace("obj ","").replace("spa ","")
L=["\\begin{tabular}{lccccc}","\\toprule",
   " & Stock & \\multicolumn{2}{c}{Pos} & \\multicolumn{2}{c}{Task} \\\\",
   "\\cmidrule(lr){3-4}\\cmidrule(lr){5-6}",
   "Task & $K{=}3$ & $K{=}3$ & $K{=}0$ & $K{=}3$ & $K{=}0$ \\\\"]
for suite,sname in (("obj","libero-object"),("goal","libero-goal"),("spa","libero-spatial")):
    L.append("\\midrule")
    L.append(f"\\multicolumn{{6}}{{l}}{{\\emph{{{sname}}}}} \\\\")
    for b in [x for x in bases if x.startswith(suite)]:
        st=stock(b)
        stv=f"{st[0]/st[1]:.2f}" if st else "--"
        v=[d["c2fix"].get(b+"_pos",[0])[0]/50, (d["c2k0"].get(b+"_pos") or [0])[0]/50,
           d["c2fix"].get(b+"_task",[0])[0]/50, (d["c2k0"].get(b+"_task") or [0])[0]/50]
        L.append(nice(b)+" & "+stv+" & "+" & ".join(f"{x:.2f}" for x in v)+" \\\\")
L+= ["\\bottomrule","\\end{tabular}"]
open("tables_pertask.tex","w").write("\n".join(L))
print("tables written")
