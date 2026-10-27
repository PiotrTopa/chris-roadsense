#!/usr/bin/env python3
"""L8's Krakow check on offline sign streams (ride_signs.py output for krakow-2024, one file per phase).

usage: krakow_l8.py HARVEST_DIR OUT0.jsonl:0 [OUT1.jsonl:1 ...]

Labels come from L8's every-frame harvest of the same ride file (HARVEST_DIR = ~/l8/harvest/krakow: dets.jsonl,
tracks.json, labels_false.json): each offline track's reported boxes are matched (same file frame, IoU >= 0.3) to
harvest detections; the track takes the majority label of the matched harvest tracks: `lamp` / `light` / `other`
(L8's hand labels of every harvest track with >= 2 round red votes that was not a sign), `real` (the other round
red harvest tracks), `sign?` (harvest tracks that were never round red: not labelled by L8), `none` (no match).
Decisions are the ended tracks after the api's context check (ride_signs.py applies signctx), i.e. what the api
uses. Per phase:
  false_no_entry   lamp/light/other tracks accepted as regulatory--no-entry      (L8 result: 0 per loop)
  false_any        lamp/light/other tracks accepted as any sign class (not background / other-sign / traffic-sign)
  real_rr          real round red tracks accepted with a round red class / all real round red tracks
  b2, b33          physical real B-2 / B-33 signs (harvest track ids) with an accepted no-entry / speed-limit track
"""
import collections, json, re, sys

RR = re.compile(r'^regulatory--(no-|maximum-speed-limit|road-closed|height-limit|weight-limit|width-limit|stop)')
NONSIGN = ('background', 'other-sign', 'traffic-sign')


def iou(a, b):
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    i = max(0, x1 - x0) * max(0, y1 - y0)
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i + 1e-9)


def main():
    hd = sys.argv[1]
    dets = collections.defaultdict(list)
    for l in open(f'{hd}/dets.jsonl'):
        r = json.loads(l); dets[r['f']].append(r)
    tracks = json.load(open(f'{hd}/tracks.json'))
    lf = json.load(open(f'{hd}/labels_false.json'))
    fk2t, tlab, tcls = {}, {}, {}
    for t in tracks:
        for f, k in t['dets']: fk2t[(f, k)] = t['tid']
        rr = sum(v for c, v in t['votes'].items() if RR.match(c))
        tlab[t['tid']] = lf.get(str(t['tid'])) or ('real' if rr >= 2 else 'sign?')
        tcls[t['tid']] = t['cls']
    allres = {}
    for spec in sys.argv[2:]:
        path, phase = spec.rsplit(':', 1); phase = int(phase)
        cam, boxes, ended = {}, collections.defaultdict(list), {}
        for l in open(path):
            m = json.loads(l)['m']
            if m['type'] == 'camera': cam[m['seq']] = m['file_frame']
            elif m['type'] == 'signs':
                ff = cam.get(m['seq'])
                if ff is None: continue
                ff = min(11999, ff + phase)
                for d in m.get('detections') or []: boxes[d['track_id']].append((ff, d['box']))
                for e in m.get('tracks_ended') or []: ended[e['track_id']] = e
        rows = []
        for tid, e in ended.items():
            votes = collections.Counter()
            for ff, b in boxes.get(tid, []):
                best = max(((iou(b, h['box']), h) for h in dets.get(ff, [])), key=lambda x: x[0], default=(0, None))
                if best[0] >= 0.3 and (ff, best[1]['k']) in fk2t: votes[fk2t[(ff, best[1]['k'])]] += 1
            if votes:
                htid = votes.most_common(1)[0][0]; lab = tlab[htid]
            else:
                htid, lab = None, 'none'
            t0 = boxes[tid][0][0] / 20 if boxes.get(tid) else None
            rows.append(dict(tid=tid, htid=htid, label=lab, cls=e['cls'], acc=not e.get('suppressed'), reason=e.get('reason'),
                             t=t0, best=e.get('best_score'), hits=e.get('hits')))
        FALSE = ('lamp', 'light', 'other')
        r = {}
        r['tracks'] = len(rows)
        r['false_no_entry'] = sum(x['label'] in FALSE and x['acc'] and x['cls'] == 'regulatory--no-entry' for x in rows)
        r['false_any'] = sum(x['label'] in FALSE and x['acc'] and x['cls'] not in NONSIGN for x in rows)
        r['false_total'] = sum(x['label'] in FALSE for x in rows)
        r['real_rr_acc'] = sum(x['label'] == 'real' and x['acc'] and bool(RR.match(x['cls'])) for x in rows)
        r['real_rr_all'] = sum(x['label'] == 'real' for x in rows)
        r['b2_signs'] = sorted({x['htid'] for x in rows if x['label'] == 'real' and x['acc'] and x['cls'] == 'regulatory--no-entry'})
        r['b33_signs'] = sorted({(x['htid'], x['cls'].split('--')[1]) for x in rows if x['label'] == 'real' and x['acc'] and 'maximum-speed-limit' in x['cls']})
        r['false_accepted_rows'] = [x for x in rows if x['label'] in FALSE and x['acc'] and x['cls'] not in NONSIGN]
        r['real_rr_lost_rows'] = [x for x in rows if x['label'] == 'real' and not (x['acc'] and RR.match(x['cls']))]
        r['reasons'] = dict(collections.Counter(f"{x['reason']}:{x['label']}" for x in rows if not x['acc']))
        allres[f'{path}:{phase}'] = r
        print(f"phase {phase}: tracks {r['tracks']}; false (lamp/light/other) {r['false_total']}: accepted as no-entry "
              f"{r['false_no_entry']}, as any sign {r['false_any']}; real round red accepted {r['real_rr_acc']}/{r['real_rr_all']}; "
              f"B-2 signs {len(r['b2_signs'])}, B-33 {r['b33_signs']}")
        for x in r['false_accepted_rows']: print('   false accepted:', x)
        for x in r['real_rr_lost_rows']: print('   real RR not accepted as RR:', x)
        print('   suppressed reason:label', r['reasons'])
    json.dump(allres, open(sys.argv[2].rsplit(':', 1)[0] + '.l8.json', 'w'), indent=1, default=str)


if __name__ == '__main__':
    main()
