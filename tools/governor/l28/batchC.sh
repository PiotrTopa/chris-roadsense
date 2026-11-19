#!/bin/sh
# Lane L28, job C (root): sign proposals from the light detector, A/B on the phone (roadsense-signs L24-S section 7).
# The L28 stack (new default road model, governor automatic) with signd as is, with --lights-signs-keep
# --lights-signs-nohits, and with --lights-signs; tools/budget/run.sh (240 s budget after 60 s, 45 s /ws probe),
# fan 100 %, cool start, in the order base, keep, all, base2 (base2 = the run-to-run spread).
L=/home/user/l28; P=$L/cprof; mkdir -p $P
echo '. /home/user/l28/profile.sh' > $P/c-base.sh
printf 'SIGND_EXTRA="--lights-signs-keep --lights-signs-nohits"\n. /home/user/l28/profile.sh\n' > $P/c-keep.sh
printf 'SIGND_EXTRA="--lights-signs"\n. /home/user/l28/profile.sh\n' > $P/c-all.sh
cp $P/c-base.sh $P/c-base2.sh
echo '{}' > $L/etc/governor.json
python3 $L/host.py roadsense_mode '{"mode":"on"}'
echo "C start $(date '+%F %T')" >> $L/out/runs.log
for p in c-base c-keep c-all c-base2; do
  sh $L/cool.sh 45 600
  L21_HOME=$L sh $L/run.sh $p $P/$p.sh 240 60
done
sh $L/cool.sh 50 600
python3 $L/host.py fan_auto; python3 $L/host.py roadsense_mode '{"mode":"auto"}'
echo C-DONE $(date +%H:%M:%S) >> $L/out/runs.log
