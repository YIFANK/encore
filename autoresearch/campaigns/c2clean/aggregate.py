#!/usr/bin/env python3
"""c2clean vs original draws: per-cell table, arm sums, eight-cell block, prediction checks."""
import json,re,os,sys
HERE=os.path.dirname(os.path.abspath(__file__)); REPO=os.path.abspath(os.path.join(HERE,'../../..'))
fc=json.load(open(f'{REPO}/autoresearch/campaigns/c2fix/final_counts.json'))
old={'k3':{c:v[0] for c,v in fc['c2fix'].items()},'k0':{c:v[0] for c,v in fc['c2k0'].items()}}
new={'k3':{},'k0':{}}
for line in open(f'{HERE}/eval.log'):
    m=re.search(r'done (\S+)_(k3|k0) rc=(\d+) success=(\d+)/50',line)
    if m and m.group(3)=='0': new[m.group(2)][m.group(1)]=int(m.group(4))
gap=["goal_open_middle_drawer_task","goal_put_wine_on_rack_task","goal_put_bowl_top_cabinet_task","goal_open_middle_drawer_pos","spa_bowl_cookie_box_pos","spa_bowl_on_cookie_box_task","spa_bowl_top_drawer_cabinet_pos","goal_put_bowl_on_stove_task"]
cells=sorted(old['k3'])
done={a:[c for c in cells if c in new[a]] for a in new}
print(f"evaluated: k3 {len(done['k3'])}/60  k0 {len(done['k0'])}/60")
for a in ('k3','k0'):
    cs=done[a]; s_new=sum(new[a][c] for c in cs); s_old=sum(old[a][c] for c in cs)
    print(f"{a}: clean {s_new}/{50*len(cs)} = {100*s_new/(50*len(cs)):.1f}%   original same cells {s_old}/{50*len(cs)} = {100*s_old/(50*len(cs)):.1f}%   |Δ|>=10 cells: {sum(abs(new[a][c]-old[a][c])>=10 for c in cs)}")
both=[c for c in cells if c in new['k3'] and c in new['k0']]
if both:
    print(f"paired cells {len(both)}: clean k3-k0 = {sum(new['k3'][c]-new['k0'][c] for c in both)}  original = {sum(old['k3'][c]-old['k0'][c] for c in both)}")
print("\neight mechanism-gap cells (clean k3 / k0 | original k3 / k0):")
ok=0;n=0
for c in gap:
    a=new['k3'].get(c,'·'); b=new['k0'].get(c,'·')
    if isinstance(a,int) and isinstance(b,int): n+=1; ok+= (a-b)>=30
    print(f"  {c:36} {str(a):>3} / {str(b):<3} | {old['k3'][c]:>3} / {old['k0'][c]:<3}")
print(f"prediction 2 (gap>=30): {ok}/{n} evaluated pairs")
print("\ncells with |clean-original|>=10:")
for a in ('k3','k0'):
    for c in done[a]:
        if abs(new[a][c]-old[a][c])>=10: print(f"  {a} {c:36} clean {new[a][c]:>3}  original {old[a][c]:>3}")
json.dump(new,open(f'{HERE}/final_counts_clean.json','w'),indent=1)
