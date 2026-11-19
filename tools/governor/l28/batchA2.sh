#!/bin/sh
# Lane L28, job A part 2 (root): the fan duty at which the ungoverned stack sits at the limit (L24 runbook step 4):
# F = 60, 40, 25, 300 s each from a cool start; stops at the first (highest) F whose last minute has the limiter
# active more than 50 % of the time. Writes the chosen F to out/F (empty if none qualified).
L=/home/user/l28; export L24_HOME=$L; O=$L/out; : > $O/F
echo "A2 start $(date '+%F %T')" >> $O/runs.log
for F in ${FANS:-60 40 25}; do
  sh $L/cool.sh 45 480
  sh $L/cond.sh pilot$F off $F 300 $L/profile.sh
  python3 $L/compare.py $O/pilot$F.jsonl --from 240 --json > $O/pilot$F.last60.json
  q=$(python3 -c "import json,sys;d=json.load(open(sys.argv[1]));p=list(d.values())[0]['delivered'];print(p['platform']['limiter_active_pct'], p['platform']['hottest_max'], p['aborted'])" $O/pilot$F.last60.json)
  echo "pilot$F last minute: limiter_active_pct hottest_max aborted = $q" >> $O/runs.log
  set -- $q
  if [ "$3" = True ]; then echo "pilot$F aborted at the safety stop" >> $O/runs.log; break; fi
  if awk -v a=$1 'BEGIN{exit !(a>50)}'; then echo $F > $O/F; break; fi
done
sh $L/cool.sh 50 480
echo A2-DONE $(date +%H:%M:%S) F=$(cat $O/F) >> $O/runs.log
