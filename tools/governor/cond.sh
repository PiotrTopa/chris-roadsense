#!/bin/sh
# cond.sh NAME on|off|LEVEL FAN_PCT SECONDS [PROFILE]     (root, on the device; bench)
# One condition of the governor proof: the stack of PROFILE (tools/budget/stack.sh) on the replayed ride from
# its first frame, the host's fan held at FAN_PCT, the governor automatic (on), disabled (off) or held at a
# shed level (a number); trace.py records what a consumer gets and what the platform does. Results in
# $L24_HOME/out/NAME.*; the fan and the host's mode are left as set (the caller restores them).
L=${L24_HOME:-/home/user/l24}; O=$L/out; mkdir -p $O $L/etc
N=$1; GOV=$2; FAN=$3; T=${4:-480}; P=${5:-$L/profile.sh}
python3 $L/host.py roadsense_mode '{"mode":"on"}'
python3 $L/host.py set_fan "{\"pct\":$FAN}"
case "$GOV" in
  off) echo '{"enabled": false}' > $L/etc/governor.json ;;
  *)   echo '{}' > $L/etc/governor.json ;;
esac
S=$(date '+%Y-%m-%d %H:%M:%S')
L21_HOME=$L sh $L/stack.sh up $P || exit 1
sleep 6
case "$GOV" in on|off) ;; *) python3 -c "
import urllib.request as u
print(u.urlopen(u.Request('http://127.0.0.1:8090/v2/governor', data=b'{\"force\": $GOV}', method='POST'), timeout=5).read().decode()[:120])" ;; esac
python3 $L/trace.py $O/$N.jsonl $T --stop-cmd "L21_HOME=$L sh $L/stack.sh down" > $O/$N.trace.log 2>&1
for u in camerad modeld signd envd api; do journalctl -u l21-$u -o cat --since "$S" > $O/$N.$u.log 2>&1; done
L21_HOME=$L sh $L/stack.sh down
echo "DONE $N gov=$GOV fan=$FAN $(date +%H:%M:%S)" >> $O/runs.log
