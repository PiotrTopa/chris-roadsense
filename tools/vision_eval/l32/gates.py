import sys,json,gzip,math,statistics as st
for p in sys.argv[1:]:
    vx=[];n=0;fast=0;straight=0;sure=0;ang=[]
    try:
        for ln in gzip.open(p,'rt'):
            if not ln.startswith('{"type":"road"'): continue
            q=json.loads(ln)['pose']; n+=1
            if q['vx']<6.7: continue
            fast+=1
            if abs(q['yaw_rate'])>0.0349: continue
            straight+=1
            a=math.degrees(math.atan2(q['vy_std'],q['vx'])); ang.append(a)
            if a<=0.25: sure+=1
    except Exception as e: pass
    qs=st.quantiles(ang,n=10) if len(ang)>20 else [float('nan')]*9
    print(f"{p.split('/')[-2][-15:]+'/'+p.split('/')[-1][:28]:46s} runs {n:6d} fast {fast:6d} straight {straight:6d} sure {sure:6d} | angle std deg p10 {qs[0]:.2f} p50 {qs[4]:.2f} p90 {qs[8]:.2f}")
