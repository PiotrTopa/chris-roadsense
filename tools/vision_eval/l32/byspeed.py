#!/usr/bin/env python3
"""Raw `road`: lead minus lane centre / minus plan at the lead's distance, by own speed. usage: byspeed.py ROAD.jsonl.gz ..."""
import sys,json,gzip,math,statistics as st
BINS=((0,0.3),(0.3,3),(3,7),(7,11),(11,15),(15,99))
def at(xs,ys,x):
    if x<xs[0] or x>xs[-1]: return None
    for i in range(len(xs)-1):
        if xs[i]<=x<=xs[i+1]:
            f=(x-xs[i])/(xs[i+1]-xs[i]) if xs[i+1]>xs[i] else 0
            return ys[i]+f*(ys[i+1]-ys[i])
for p in sys.argv[1:]:
    rows={b:{'n':0,'lead':0,'lines':0,'dl':[],'dp':[],'ang':[],'ly':[],'x':[],'lane_dir':[],'lp':[]} for b in BINS}
    for ln in gzip.open(p,'rt'):
        if not ln.startswith('{"type":"road"'): continue
        m=json.loads(ln); v=abs(m.get('v_ego') or 0)
        b=next(b for b in BINS if b[0]<=v<b[1]); r=rows[b]; r['n']+=1
        ll=m['lane_lines']; both=ll[1]['prob']>=0.5 and ll[2]['prob']>=0.5
        r['lp'].append(min(ll[1]['prob'],ll[2]['prob']))
        if both:
            r['lines']+=1
            c30=0.5*(at(ll[1]['x'],ll[1]['y'],30)+at(ll[2]['x'],ll[2]['y'],30)); c5=0.5*(at(ll[1]['x'],ll[1]['y'],5)+at(ll[2]['x'],ll[2]['y'],5))
            r['lane_dir'].append(math.degrees(math.atan2(c30-c5,25)))
        L=m['leads'][0]
        if L['prob']<0.7 or L['x'][0]<8: continue
        r['lead']+=1; x=L['x'][0]; r['ly'].append(L['y'][0]); r['x'].append(x)
        if both:
            a=at(ll[1]['x'],ll[1]['y'],x); c=at(ll[2]['x'],ll[2]['y'],x)
            if a is not None: d=L['y'][0]-0.5*(a+c); r['dl'].append(d); r['ang'].append(math.degrees(math.atan2(d,x)))
        pl=m.get('plan')
        if pl:
            y=at(pl['x'],pl['y'],x)
            if y is not None: r['dp'].append(L['y'][0]-y)
    print('==',p.split('/')[-1][:-9])
    print('   v m/s     runs  lines%  min-line-prob  lead  lead_x  lead_y  lead-lane m (n)   angle deg  lead-plan m (n)  lane dir 5-30 m deg')
    f=lambda v,fmt='+.2f': format(st.median(v),fmt) if len(v)>=8 else '   -'
    for b in BINS:
        r=rows[b]
        if not r['n']: continue
        print(f"   {b[0]:4.1f}-{b[1]:4.1f} {r['n']:6d} {100*r['lines']/r['n']:6.0f} {f(r['lp'],'.2f'):>10s} {r['lead']:9d} {f(r['x'],'.0f'):>6s} {f(r['ly']):>7s} {f(r['dl']):>8s} ({len(r['dl']):4d}) {f(r['ang']):>9s} {f(r['dp']):>9s} ({len(r['dp']):4d}) {f(r['lane_dir']):>9s}")
