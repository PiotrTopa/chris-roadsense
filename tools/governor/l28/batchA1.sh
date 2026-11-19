#!/bin/sh
# Lane L28, job A part 1 (root): smoke, then watts per level (L24 runbook steps 2-3; renumbered: 8 = plan, 9 = road)
L=/home/user/l28; export L24_HOME=$L
echo "A1 start $(date '+%F %T')" >> $L/out/runs.log
sh $L/cool.sh 50 300
sh $L/cond.sh smoke 6 100 120 $L/profile.sh
for l in 0 3 4 5 6 7 8 9; do sh $L/cool.sh 55 240; sh $L/cond.sh lvl$l $l 100 150 $L/profile.sh; done
sh $L/cool.sh 50 300
echo A1-DONE $(date +%H:%M:%S) >> $L/out/runs.log
