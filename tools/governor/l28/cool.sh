#!/bin/sh
# cool.sh TARGET_C [MAX_S=600]: stack down, fan 100 %, host mode on, wait until the hottest zone is under TARGET_C
# (lane L28; the fan is set once, so the next fan change is at least 10 s later by construction: min wait 20 s)
L=/home/user/l28
L21_HOME=$L sh $L/stack.sh down
python3 $L/host.py set_fan '{"pct":100}' >/dev/null
sleep 20
i=0; while [ $i -lt ${2:-600} ]; do
  h=$(sh $L/hot.sh); awk -v h=$h -v t=$1 'BEGIN{exit !(h<t)}' && break
  sleep 5; i=$((i+5))
done
echo "cool $(date +%H:%M:%S) hottest $(sh $L/hot.sh) after $((i+20)) s" >> $L/out/runs.log
