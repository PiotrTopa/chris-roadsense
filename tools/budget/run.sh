#!/bin/sh
# run.sh NAME PROFILE [SECONDS=240] [SETTLE=40] : bring a profile up, let it settle, measure the budget
# with only the host's consumer on the api, then 45 s with a /ws probe on every topic (rates,
# latency), collect the daemons' own stats lines, bring it down. Results in ~/l21/out/NAME.*
L=${L21_HOME:-/home/user/l21}; N=$1; P=$2; T=${3:-240}; ST=${4:-40}; O=$L/out; mkdir -p $O
sh $L/stack.sh up $P || exit 1
sleep $ST
S=$(date '+%Y-%m-%d %H:%M:%S')
python3 $L/measure.py $N $T --out $O > $O/$N.budget 2>&1
python3 $L/wsprobe.py 45 > $O/$N.ws 2>&1
for u in camerad modeld signd envd mapd api; do journalctl -u l21-$u -o cat --since "$S" > $O/$N.$u.log 2>&1; done
python3 -c 'import urllib.request as u;print(u.urlopen("http://127.0.0.1:8090/v1/health",timeout=5).read().decode())' > $O/$N.health 2>&1
sh $L/stack.sh down
echo DONE $N $(date +%H:%M:%S) >> $O/runs.log
