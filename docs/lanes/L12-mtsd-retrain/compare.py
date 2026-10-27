#!/usr/bin/env python3
"""Pre-registered comparison of the L12 variants (eval_variant.sh outputs in ~/l12/eval/<variant>/).
usage: compare.py VARIANT [VARIANT ...]   (the first one is the reference, base)"""
import json, os, sys
E = os.path.expanduser('~/l12/eval')
vs = sys.argv[1:]


def load(v):
    d = f'{E}/{v}'
    r = {'score': [json.load(open(f'{d}/score_p{p}.json'))['pooled'] for p in (0, 2)]}
    r['l8'] = json.load(open(f'{d}/krakow-2024_p0.jsonl.l8.json'))
    # E4 B-33 70
    b70 = 0
    for p in range(4):
        tl = json.load(open(f'{d}/tl_krakow-2024_p{p}.json'))[0]
        b70 += any(x['limit_kmh'] == 70 and x['kind'] == 'sign' and abs(x['start_t'] - 87.9) <= 10 for x in tl['speed_limit'])
    r['b70'] = b70
    d7 = d7t = 0
    for p in (0, 2):
        tl = json.load(open(f'{d}/tl_s1-roadworks_p{p}.json'))[0]
        d7 += any(x['id'] == 'expressway' and abs(x['start_t'] - 67) <= 10 for x in tl['intervals'])
        cam, hit = {}, False
        for l in open(f'{d}/s1-roadworks_p{p}.jsonl'):
            m = json.loads(l)['m']
            if m['type'] == 'camera': cam[m['seq']] = m['file_frame']
            elif m['type'] == 'signs' and m['seq'] in cam and 55 <= cam[m['seq']] / 20 <= 80:
                hit |= any(('limited-access-road' in x['cls'] or 'information--motorway' in x['cls']) for x in m['detections'])
        d7t += hit
    r['d7'], r['d7t'] = d7, d7t
    return r


R = {v: load(v) for v in vs}
rows = []
def tot(r, key):
    return sum(sum(x[key] for x in s['rules'].values()) for s in r['score'])
def mw(r):
    return sum(sum(x['tp'] for x in s[k].values()) for s in r['score'] for k in ('momentary', 'warnings'))
def l8(r, k):
    return sum(x[k] if not isinstance(x[k], list) else len(x[k]) for x in r['l8'].values())
print('| metric | ' + ' | '.join(vs) + ' |'); print('|---' * (len(vs) + 1) + '|')
lines = [
    ('E3 false rules (6 rides x 2 phases)', lambda r: tot(r, 'false')),
    ('E3 missed intervals', lambda r: tot(r, 'misses')),
    ('E3 matched intervals', lambda r: tot(r, 'matched')),
    ('E3 momentary + warning true positives', mw),
    ('E3 momentary + warning engine events', lambda r: sum(sum(x['engine'] for x in s[k].values()) for s in r['score'] for k in ('momentary', 'warnings'))),
    ('E2 lamp/light/other accepted as no-entry (4 phases)', lambda r: l8(r, 'false_no_entry')),
    ('E2 lamp/light/other accepted as any sign', lambda r: l8(r, 'false_any')),
    ('E2 lamp/light/other tracks (all)', lambda r: l8(r, 'false_total')),
    ('E2 real round red accepted / all', lambda r: f"{l8(r, 'real_rr_acc')}/{l8(r, 'real_rr_all')}"),
    ('E2 B-2 physical signs caught (sum over phases)', lambda r: l8(r, 'b2_signs')),
    ('E2 B-2 per phase', lambda r: [len(x['b2_signs']) for x in r['l8'].values()]),
    ('E2 B-33 per phase', lambda r: [len(x['b33_signs']) for x in r['l8'].values()]),
    ('E4 B-33 70 served (phases of 4)', lambda r: r['b70']),
    ('E4 D-7 expressway interval (phases of 2)', lambda r: r['d7']),
    ('E4 D-7 track classed limited-access/motorway (phases of 2)', lambda r: r['d7t']),
]
for name, f in lines:
    print(f'| {name} | ' + ' | '.join(str(f(R[v])) for v in vs) + ' |')
rules = sorted(R[vs[0]]['score'][0]['rules'])
for k in rules:
    vals = []
    for v in vs:
        s = R[v]['score']
        a = (s[0]['rules'][k]['agreement_mean'] + s[1]['rules'][k]['agreement_mean']) / 2
        f = s[0]['rules'][k]['false'] + s[1]['rules'][k]['false']; m = s[0]['rules'][k]['misses'] + s[1]['rules'][k]['misses']
        vals.append(f'{a:.3f} (F{f} M{m})')
    print(f'| agreement {k} (false, missed) | ' + ' | '.join(vals) + ' |')
