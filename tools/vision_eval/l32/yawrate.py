import sys,json,gzip,math,statistics as st
for p in sys.argv[1:]:
    b={}
    try:
        for ln in gzip.open(p,'rt'):
            if not ln.startswith('{"type":"road"'): continue
            q=json.loads(ln)['pose']
            if q['vx']<6.7 or math.atan2(q['vy_std'],q['vx'])>0.00873: continue
            w=math.degrees(q['yaw_rate'])
            if abs(w)>3: continue
            k=math.floor(w/0.5)*0.5
            b.setdefault(k,[]).append((math.degrees(math.atan2(q['vy'],q['vx'])),math.degrees(math.atan2(q['vz'],q['vx']))))
    except Exception: pass
    print(p.split('/')[-1][:40] if 'messages' not in p else p.split('/')[-2])
    for k in sorted(b):
        if len(b[k])>30: print(f"   yaw rate {k:+.1f}..{k+0.5:+.1f} deg/s: dir {st.median([x[0] for x in b[k]]):+.2f} deg, vz/vx {st.median([x[1] for x in b[k]]):+.2f}  n {len(b[k])}")
