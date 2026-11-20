#!/bin/sh
# Timelines + scores for every labelled ride in three variants: pre (pre-registered engine, lights on),
# pre_nolights (pre-registered, lights off), final (priority inference on, lights on).
PY=$HOME/projects/roadsense/.venv-models/bin/python
R=$HOME/projects/roadsense/tools/rules_replay.py
cd $HOME/l11/rec/eval || exit 1
for v in pre pre_nolights final; do
  case $v in pre) A="--no-priority-inference";; pre_nolights) A="--no-priority-inference --no-lights";; final) A="";; esac
  for s in "$@"; do
    $PY $R $HOME/l11/rec/$s.jsonl --quiet $A --out tl_${v}_$s.json > /dev/null 2>&1
  done
  $PY - $v "$@" <<'PYEOF'
import json, sys
v, slugs = sys.argv[1], sys.argv[2:]
runs = []
for s in slugs:
    runs += json.load(open(f"tl_{v}_{s}.json"))
json.dump(runs, open(f"tl_{v}.json", "w"))
PYEOF
  $PY $HOME/l11/scripts/score.py tl_$v.json $HOME/l11/labels --out score_$v.json > score_$v.txt 2>&1
  echo "$v done: $(grep -c agree score_$v.txt) rule rows"
done
