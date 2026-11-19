#!/usr/bin/env python3
"""Compact per-ride table from evaluate.py's RESULTS.json for the L32 streams. usage: eval_table.py RESULTS.json"""
import json, sys
R = json.load(open(sys.argv[1]))
S = ('off.13.8', 'warm.13.8', 'cold.13.8', 'poff.13.8', 'pcold.13.8')
keys = ['inner_line_changes_pm', 'both_inner_present_pct', 'jitter_tr_10m', 'jitter_tr_50m', 'lead_onoff_pm', 'lead_id_changes_pm',
        'lead_d_noise_15-30m', 'lead_d_noise_30-60m', 'lead_d_noise_60-200m', 'lead_v_noise']
for ride, r in R.items():
    print('==', ride)
    print('| measure | ' + ' | '.join(s[:-5] for s in S) + ' |')
    for k in keys:
        print(f'| {k} | ' + ' | '.join(str(r[s].get(k)) for s in S) + ' |')
    for b in ('20-40', '40-70', '70-100'):
        print(f'| far range {b} m: median / p90 | ' + ' | '.join(f"{r[s]['far_tracked_' + b][0]:.2f} / {r[s]['far_tracked_' + b][1]:.2f}" if r[s].get('far_tracked_' + b) else '-' for s in S) + ' |')
    if 'egolane' in r:
        print('| ego lane right / wrong-confident % | ' + ' | '.join(f"{r['egolane'][s]['label']['right_pct_of_all']} / {r['egolane'][s]['label']['wrong_confident_pct']}" for s in S) + ' |')
