#!/bin/bash
# Offline evaluation of one sign-model variant on the six L11 rides (phases 0 and 2; krakow-2024 also 1 and 3):
# signd_replay (laptop build) on the phone's recorded frames -> rules_replay (deployed engine) -> L11 score.py,
# plus L8's Krakow lamp check. usage: eval_variant.sh NAME THREADS -- SIGND_REPLAY_ARGS...
# Output: ~/l12/eval/NAME/ (never committed: ride frames and derived streams).
set -u
NAME=$1; THR=$2; shift 3
PY=$HOME/projects/roadsense/.venv-models/bin/python
L=$HOME/projects/roadsense/docs/lanes/L12-mtsd-retrain
O=$HOME/l12/eval/$NAME; mkdir -p $O; cd $O || exit 1
RIDES="krakow-2024 rural-zloczew forest-vistula-spit s1-roadworks warsaw-city katowice-rain"
for P in 0 2 1 3; do
  for s in $RIDES; do
    if [ $P = 1 -o $P = 3 ] && [ $s != krakow-2024 ]; then continue; fi
    [ -s ${s}_p$P.jsonl ] || $PY -I $L/ride_signs.py $s $HOME/l11/rec/$s.jsonl ${s}_p$P.jsonl --phase $P -- \
        $HOME/l12/build-x86/signd_replay --threads $THR --no-timing "$@" 2>> ride_signs.log
    $PY $HOME/projects/roadsense/tools/rules_replay.py ${s}_p$P.jsonl --quiet --out tl_${s}_p$P.json > /dev/null 2>> rules.log
  done
  if [ $P = 0 -o $P = 2 ]; then
    $PY - $P $RIDES <<'PYEOF'
import json, sys
p, slugs = sys.argv[1], sys.argv[2:]
runs = []
for s in slugs: runs += json.load(open(f"tl_{s}_p{p}.json"))
json.dump(runs, open(f"tl_p{p}.json", "w"))
PYEOF
    $PY $HOME/l11/scripts/score.py tl_p$P.json $HOME/l11/labels --out score_p$P.json > score_p$P.txt 2>&1
  fi
done
$PY -I $L/krakow_l8.py $HOME/l8/harvest/krakow krakow-2024_p0.jsonl:0 krakow-2024_p1.jsonl:1 krakow-2024_p2.jsonl:2 krakow-2024_p3.jsonl:3 > l8.txt 2>&1
echo "$NAME done $(date -Is)"
