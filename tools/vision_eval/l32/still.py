import sys,json,gzip,math,statistics as st
def at(xs,ys,x):
    for i in range(len(xs)-1):
        if xs[i]<=x<=xs[i+1]:
            f=(x-xs[i])/(xs[i+1]-xs[i]) if xs[i+1]>xs[i] else 0
            return ys[i]+f*(ys[i+1]-ys[i])
def feats(m):
    f={}
    L=m['leads'][0]; ll=m['lane_lines']
    if L['prob']>0.7 and L['x'][0]>15:
        f['lead_x']=L['x'][0]; f['lead']=math.degrees(math.atan2(L['y'][0],L['x'][0]))
    if ll[1]['prob']>0.5 and ll[2]['prob']>0.5:
        f['lane30']=math.degrees(math.atan2(0.5*(at(ll[1]['x'],ll[1]['y'],30)+at(ll[2]['x'],ll[2]['y'],30)),30))
        if 'lead' in f:
            x=min(f['lead_x'],190); c=0.5*(at(ll[1]['x'],ll[1]['y'],x)+at(ll[2]['x'],ll[2]['y'],x))
            f['lead-lane']=math.degrees(math.atan2(L['y'][0]-c,f['lead_x'])); f['lead-lane_m']=L['y'][0]-c
    return f
def load(p,still):
    o={}
    for ln in gzip.open(p,'rt'):
        if not ln.startswith('{"type":"road"'): continue
        m=json.loads(ln)
        if still and m['seq']%5: continue
        o[m['seq']//5-1 if still else m['seq']]=feats(m)
    return o
def summ(name,o):
    r=[]
    for k in ('lead','lane30','lead-lane','lead_x'):
        v=[f[k] for f in o.values() if k in f]
        r.append(f"{k} {st.median(v):+.2f} [{st.quantiles(v,n=4)[0]:+.2f},{st.quantiles(v,n=4)[2]:+.2f}] n {len(v)}" if len(v)>10 else f"{k} n {len(v)}")
    print(f"{name:28s} "+' | '.join(r))
    # by distance
    for lo,hi in ((15,30),(30,45),(45,60),(60,90)):
        v=[f['lead-lane_m'] for f in o.values() if 'lead-lane_m' in f and lo<=f['lead_x']<hi]
        if len(v)>5: print(f"      lead {lo}-{hi} m: lead - lane centre {st.median(v):+.2f} m (n {len(v)})")
    return o
mode=sys.argv[1]
if mode=='control':
    s=load(sys.argv[2],True); p=load(sys.argv[3],False)
    summ('still pairs',s)
    pp={}
    for i,f in s.items():
        for d in range(0,4):
            if 20*i+1+d in p: pp[i]=p[20*i+1+d]; break
    summ('real pairs, same frames',pp)
    for k in ('lead','lane30','lead_x','lead-lane'):
        d=[s[i][k]-pp[i][k] for i in s if i in pp and k in s[i] and k in pp[i]]
        print(f"   still - real, {k}: median {st.median(d):+.2f}, median |d| {st.median([abs(x) for x in d]):.2f}, n {len(d)}")
else:
    for p in sys.argv[2:]: summ(p.split('/')[-1][:-9],load(p,True))
