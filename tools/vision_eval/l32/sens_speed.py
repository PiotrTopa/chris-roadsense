#!/usr/bin/env python3
"""Change of each raw output's direction between two replays of the same frames (B minus A), by own speed.
usage: sens_speed.py A.jsonl.gz B.jsonl.gz   (e.g. RIDE.off / RIDE.poff: calibration yaw +3.4 deg, pitch -1.4 deg)"""
import sys,json,gzip,math,statistics as st
BINS=((0,0.3),(0.3,3),(3,7),(7,11),(11,15),(15,99))
def at(xs,ys,x):
    if x<xs[0] or x>xs[-1]: return None
    for i in range(len(xs)-1):
        if xs[i]<=x<=xs[i+1]:
            f=(x-xs[i])/(xs[i+1]-xs[i]) if xs[i+1]>xs[i] else 0
            return ys[i]+f*(ys[i+1]-ys[i])
def load(p):
    o={}
    for ln in gzip.open(p,'rt'):
        if ln.startswith('{"type":"road"'):
            m=json.loads(ln); o[m['seq']]=m
    return o
def feats(m, need_lines=0.5):
    f={}
    L=m['leads'][0]
    if L['prob']>0.5 and L['x'][0]>6: f['lead']=math.degrees(math.atan2(L['y'][0],L['x'][0])); f['lead_x']=L['x'][0]
    ll=m['lane_lines']
    for i,n in ((1,'left'),(2,'right')):
        if ll[i]['prob']>=need_lines:
            f[n]=math.degrees(math.atan2(at(ll[i]['x'],ll[i]['y'],20)-ll[i]['y'][0],20))
    pl=m.get('plan')
    if pl and pl['x'][-1]>20:
        y=at(pl['x'],pl['y'],20)
        if y is not None: f['plan']=math.degrees(math.atan2(y,20))
    return f
A=load(sys.argv[1]); B=load(sys.argv[2])
K=('lead','left','right','plan')
print('B - A, median change of direction (deg) by own speed of A; n in brackets; lines: prob >= 0.5 in both, direction 0-20 m')
print('   v m/s    runs  '+'  '.join(f'{k:>14s}' for k in K)+'   lead prob A/B   line prob A/B (min inner)')
for b in BINS:
    d={k:[] for k in K}; n=0; pa=[];pb=[];la=[];lb=[]
    for s,m in A.items():
        v=abs(m.get('v_ego') or 0)
        if not (b[0]<=v<b[1]) or s not in B: continue
        n+=1; fa=feats(m); fb=feats(B[s])
        pa.append(m['leads'][0]['prob']); pb.append(B[s]['leads'][0]['prob'])
        la.append(min(m['lane_lines'][1]['prob'],m['lane_lines'][2]['prob'])); lb.append(min(B[s]['lane_lines'][1]['prob'],B[s]['lane_lines'][2]['prob']))
        for k in K:
            if k in fa and k in fb: d[k].append(fb[k]-fa[k])
    if not n: continue
    print(f"   {b[0]:4.1f}-{b[1]:4.1f} {n:6d}  "+'  '.join((f"{st.median(d[k]):+6.2f} ({len(d[k]):5d})" if len(d[k])>=8 else f"     - ({len(d[k]):5d})") for k in K)+f"   {st.mean(pa):.2f}/{st.mean(pb):.2f}       {st.median(la):.2f}/{st.median(lb):.2f}")
