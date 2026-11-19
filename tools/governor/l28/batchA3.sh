#!/bin/sh
# Lane L28, job A part 3 (root): the proof (L24 runbook step 5), 480 s each from a cool start, same ride from its
# first frame: ref (governor on, fan 100), hotN (off, F), hotG (on, F), ref2 (on, 100), then hotOld (L21's builds,
# supercombo, installed api: the private guards) at F.  usage: batchA3.sh F
L=/home/user/l28; export L24_HOME=$L; O=$L/out; F=$1; T=${T:-480}
# L28: the safety stop on the first sample over 95 C (not three)
export TRACE_ARGS="${TRACE_ARGS:---stop-n 1}"
echo "A3 start $(date '+%F %T') F=$F" >> $O/runs.log
for spec in "ref on 100" "hotN off $F" "hotG on $F" "ref2 on 100"; do
  set -- $spec; sh $L/cool.sh 45 600; sh $L/cond.sh $1 $2 $3 $T $L/profile.sh
done
[ -n "$NO_OLD" ] || { sh $L/cool.sh 45 600; sh $L/cond.sh hotOld off $F $T /home/user/l21/profiles/F-final.sh; }
sh $L/cool.sh 50 600
python3 $L/host.py fan_auto; python3 $L/host.py roadsense_mode '{"mode":"auto"}'
echo A3-DONE $(date +%H:%M:%S) >> $O/runs.log
