#!/usr/bin/env python3
"""Per-frame change of each output against the base run of the same ride: angle (deg) by which it moved.
usage: sens.py SWEEP_DIR RIDE MODEL"""
import sys,json,gzip,math,statistics as st,glob,os
D,ride,model=sys.argv[1:4]
def load(p):
    o={}
    for ln in gzip.open(p,'rt'):
        if '"road"' not in ln[:20]: continue
        m=json.loads(ln); o[m['seq']]=m
    return o
def at(xs,ys,x):
    for i in range(len(xs)-1):
        if xs[i]<=x<=xs[i+1]:
            f=(x-xs[i])/(xs[i+1]-xs[i]) if xs[i+1]>xs[i] else 0
            return ys[i]+f*(ys[i+1]-ys[i])
def feats(m):
    f={}
    p=m.get('pose') or {}
    if p.get('vx',0)>5:
        f['pose_dir']=math.degrees(math.atan2(p['vy'],p['vx'])); f['pose_pitch']=math.degrees(math.atan2(p['vz'],p['vx']))
    L=m['leads'][0]
    if L['prob']>0.7 and L['x'][0]>20:
        f['lead']=math.degrees(math.atan2(L['y'][0],L['x'][0])); f['lead_x']=L['x'][0]
    ll=m['lane_lines']
    if ll[1]['prob']>0.5 and ll[2]['prob']>0.5:
        for x in (10,30,60):
            c=0.5*(at(ll[1]['x'],ll[1]['y'],x)+at(ll[2]['x'],ll[2]['y'],x))
            f['lane%d'%x]=math.degrees(math.atan2(c,x))
        f['lane0_m']=0.5*(ll[1]['y'][0]+ll[2]['y'][0])
    pl=m.get('plan')
    if pl:
        for x in (30,60):
            y=at(pl['x'],pl['y'],x)
            if y is not None: f['plan%d'%x]=math.degrees(math.atan2(y,x))
    rt=m.get('road_transform')
    if 'lead' in f and 'lane30' in f:
        c=0.5*(at(ll[1]['x'],ll[1]['y'],min(f['lead_x'],190))+at(ll[2]['x'],ll[2]['y'],min(f['lead_x'],190)))
        f['lead-lane']=math.degrees(math.atan2(L['y'][0]-c,f['lead_x']))
    if 'lead' in f and pl:
        y=at(pl['x'],pl['y'],f['lead_x'])
        if y is not None: f['lead-plan']=math.degrees(math.atan2(L['y'][0]-y,f['lead_x']))
    return f
base=load(f'{D}/{ride}.{model}.base.jsonl.gz'); bf={s:feats(m) for s,m in base.items()}
keys=['pose_dir','pose_pitch','lead','lead_x','lane10','lane30','lane60','lane0_m','plan30','plan60','lead-lane','lead-plan']
print(f'{ride} {model}: median change against base (deg; lead_x, lane0_m in m); last rows: absolute medians')
print('variant   '+' '.join(f'{k:>10s}' for k in keys))
for p in sorted(glob.glob(f'{D}/{ride}.{model}.*.jsonl.gz')):
    tag=os.path.basename(p).split('.')[-3]
    v=load(p); row=[]
    for k in keys:
        if tag=='base': d=[bf[s][k] for s in bf if k in bf[s]]
        else:
            d=[]
            for s,m in v.items():
                if s in bf and k in bf[s]:
                    f=feats(m)
                    if k in f: d.append(f[k]-bf[s][k])
        row.append(f'{st.median(d):+10.2f}' if len(d)>20 else '         -')
    print(f'{tag:9s} '+' '.join(row))
