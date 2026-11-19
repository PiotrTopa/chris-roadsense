#!/usr/bin/env python3
"""Tables of docs/lanes/L32-lead-lateral.md section 4 from ~/datasets/l32/res (score.sh)."""
import json,os,statistics as st
R=os.path.expanduser('~/datasets/l32/res')
T=('off','warm','cold','pcold','poff')
lat={t:json.load(open(f'{R}/lat_{t}.json')) for t in T}
rides=['a1-night','krakow-2024','s7-elblag-lowsun','tunnels-s3','warsaw-city','krakow-night-tunnel','gdansk-night-snow','warsaw-storm']
def ang(d,k):
    x=(d or {}).get('angle',{}).get(k)
    return f"{x['deg']:+.2f}" if x else "-"
print("**Angle lead - lane centre, deg (leads from 20 m; median, + = lead to the right)**\n")
print("| ride | off | warm | cold | poff (config wrong by the car's error) | pcold |\n|---|---|---|---|---|---|")
for r in rides:
    row=[ang(lat[t].get(f'{r}.{t}.13.8'),'lane') for t in T]
    if all(x=='-' for x in row): continue
    print(f"| {r} | {row[0]} | {row[1]} | {row[2]} | {row[4]} | {row[3]} |")
print("\n**Angle lead - plan, deg**\n")
print("| ride | off | warm | cold | poff | pcold |\n|---|---|---|---|---|---|")
for r in rides:
    row=[ang(lat[t].get(f'{r}.{t}.13.8'),'plan') for t in T]
    if all(x=='-' for x in row): continue
    print(f"| {r} | {row[0]} | {row[1]} | {row[2]} | {row[4]} | {row[3]} |")
print("\n**Lead - lane centre by distance, m (median per bin, bins with >= 20 frames)**\n")
print("| ride, run | 15-30 | 30-45 | 45-60 | 60-75 | 75-90 | 90-105 m |\n|---|---|---|---|---|---|---|")
for r in rides:
    for t in ('off','warm','poff','pcold'):
        d=lat[t].get(f'{r}.{t}.13.8')
        if not d or sum(b['n_lane']>=20 for b in d['bins'])<2: continue
        print(f"| {r}, {t} | "+" | ".join(f"{b['lane']:+.2f}" if b['n_lane']>=20 else "-" for b in d['bins'])+" |")
print("\n**Direction of `pose` on straight fast runs, deg (the calibration's remaining yaw error)**\n")
print("| ride | off | warm | cold | poff | pcold |\n|---|---|---|---|---|---|")
for r in rides:
    row=[(f"{lat[t][f'{r}.{t}.13.8']['pose_dir']:+.2f}" if 'pose_dir' in lat[t].get(f'{r}.{t}.13.8',{}) else '-') for t in T]
    print(f"| {r} | {row[0]} | {row[1]} | {row[2]} | {row[4]} | {row[3]} |")
