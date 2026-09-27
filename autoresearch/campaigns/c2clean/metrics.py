#!/usr/bin/env python3
"""Efficiency metrics per cell beyond success rate: time/episodes to first success, development episodes,
versions, wall-clock, tokens, cost. Clean K=3 vs K=0 (same harness), original c2fix vs c2k0 for reference."""
import json,glob,os,re,statistics,datetime as dt
HERE=os.path.dirname(os.path.abspath(__file__)); C=os.path.abspath(os.path.join(HERE,'..'))
runs=json.load(open(f'{HERE}/_claude_tmp_devruns_c2.json'))
def epoch(s): return dt.datetime.fromisoformat(s.replace('Z','+00:00')).timestamp()
def worker(ws):
    sess={}
    for sid,kind,ts in re.findall(r'session (\S+) (start|end)\s+(\d{4}-\d\d-\d\dT\S+?Z)',open(f'{ws}/WALLCLOCK.md').read()):
        sess.setdefault(sid,{})[kind]=epoch(ts)
    st=[v['start'] for v in sess.values() if 'start' in v]; en=[v.get('end',v['start']) for v in sess.values() if 'start' in v]
    m=dict(start=min(st),end=max(en),hours=sum(e-a for a,e in zip(st,en))/3600,turns=0,out_tok=0,think=0,cache_read=0,in_tok=0,cost=0.0)
    for f in glob.glob(f'{ws}/transcript_*.jsonl'):
        for line in open(f,errors='ignore'):
            if '"type":"assistant"' in line: m['turns']+=1
            elif '"type":"result"' in line:
                try: j=json.loads(line)
                except Exception: j={}
                u=j.get('usage') or {}; m['cost']+=float(j.get('total_cost_usd') or 0)
                m['in_tok']+=int(u.get('input_tokens',0))+int(u.get('cache_creation_input_tokens',0)); m['cache_read']+=int(u.get('cache_read_input_tokens',0)); m['out_tok']+=int(u.get('output_tokens',0))
                m['think']+=int((u.get('output_tokens_details') or {}).get('thinking_tokens',0))
    return m
def cellmetrics(camp,key,ws):
    m=worker(ws); rs=sorted([r for r in runs if r['camp']==camp and r['cell']==key],key=lambda r:r['end'])
    m['versions']=max([r['ver'] for r in rs],default=0); m['dev_eps']=sum(r['n'] for r in rs); m['dev_ok']=sum(r['s'] for r in rs)
    m['sim_steps']=sum(r['steps'] for r in rs); m['sim_h']=sum(r['dur'] for r in rs)/3600
    v1=[r for r in rs if r['ver']==1]; m['v1_ok']=(sum(r['s'] for r in v1)/max(1,sum(r['n'] for r in v1))) if v1 else None; m['v1_any']=int(any(r['s']>0 for r in v1)) if v1 else None
    cum=0; m['eps_to_first']=None; m['t_first_h']=None; m['ver_first']=None
    for r in rs:
        cum+=r['n']
        if r['s']>0:
            m['eps_to_first']=cum; m['ver_first']=r['ver']
            m['t_first_h']=((r['first_ok'] or r['end'])-m['start'])/3600; break
    return m
arms={}
for arm in ('k3','k0'):
    arms[f'clean {arm}']={os.path.basename(ws)[:-3]:cellmetrics('c2clean',os.path.basename(ws),ws) for ws in sorted(glob.glob(f'{HERE}/workers/*_{arm}'))}
arms['orig k3']={os.path.basename(ws):cellmetrics('c2fix',os.path.basename(ws),ws) for ws in sorted(glob.glob(f'{C}/c2fix/workers/*'))}
arms['orig k0']={os.path.basename(ws):cellmetrics('c2k0',os.path.basename(ws),ws) for ws in sorted(glob.glob(f'{C}/c2k0/workers/*'))}
json.dump(arms,open(f'{HERE}/metrics_per_cell.json','w'),indent=1)
M=[('v1_ok','v1 dev success fraction','med'),('t_first_h','time to first dev success (h)','med'),('eps_to_first','dev episodes to first success','med'),('ver_first','version of first success','med'),
   ('dev_eps','development episodes','med'),('versions','program versions','med'),('hours','agent wall-clock (h)','med'),('turns','assistant turns','med'),
   ('out_tok','output tokens (k)','med'),('cache_read','cache-read tokens (M)','med'),('cost','cost (USD)','med')]
def fmt(k,v):
    if v is None: return '–'
    if k=='out_tok': return f'{v/1e3:.0f}'
    if k=='cache_read': return f'{v/1e6:.1f}'
    if k in ('t_first_h','hours','sim_h'): return f'{v:.2f}'
    if k=='cost': return f'{v:.2f}'
    if k=='v1_ok': return f'{v:.2f}'
    return f'{v:.0f}'
names=list(arms); print('| metric | '+' | '.join(names)+' |'); print('|---|'+'---|'*len(names))
for k,lab,_ in M:
    row=[]
    for a in names:
        vals=[c[k] for c in arms[a].values() if c[k] is not None]
        row.append(fmt(k,statistics.median(vals)) if vals else '–')
    print(f'| {lab}, median | '+' | '.join(row)+' |')
for k,lab in (('hours','agent wall-clock (h)'),('dev_eps','development episodes'),('out_tok','output tokens (k)'),('cost','cost (USD)')):
    print(f'| {lab}, total | '+' | '.join(fmt(k,sum(c[k] for c in arms[a].values())) for a in names)+' |')
print('| cells whose v1 already succeeded on some seed | '+' | '.join(str(sum((c["v1_any"] or 0) for c in arms[a].values())) for a in names)+' |')
print('| cells with no dev success | '+' | '.join(str(sum(c["t_first_h"] is None for c in arms[a].values())) for a in names)+' |')
print('\npaired clean K=3 vs K=0 (per cell, K=0 minus K=3): wins = K=3 cheaper')
for k,lab,_ in M:
    d=[arms['clean k0'][c][k]-arms['clean k3'][c][k] for c in arms['clean k3'] if arms['clean k0'][c][k] is not None and arms['clean k3'][c][k] is not None]
    print(f'  {lab:36} median diff {fmt(k,statistics.median(d)):>6}   K=3 cheaper {sum(x>0 for x in d)} / K=0 cheaper {sum(x<0 for x in d)} / tie {sum(x==0 for x in d)}  (n={len(d)})')
