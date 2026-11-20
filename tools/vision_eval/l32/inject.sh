#!/bin/sh
# host motion injected into the 2.0 chain (vision model, the ride's calibration): exact, a yaw-rate bias, a speed-proportional
# yaw-rate error (a steering-angle zero offset: yaw rate = v tan(delta) / wheelbase)
R=${1:-warsaw-city}; AC=$HOME/datasets/l32/ac; C=$HOME/datasets/l32/scripts/chain.sh
A=$HOME/projects/roadsense/api; CAR=$HOME/projects/cyberpank-prius-gen2-computer
{
echo "sh $C $R.H_ego_exact $AC/$R.off.13.8.jsonl.gz $A $CAR $CAR --ego"
echo "sh $C $R.H_ego_bias  $AC/$R.off.13.8.jsonl.gz $A $CAR $CAR --ego --ego-yaw-bias 0.02"
echo "sh $C $R.H_ego_steer $AC/$R.off.13.8.jsonl.gz $A $CAR $CAR --ego --ego-yaw-per-speed 0.002"
} | xargs -P 3 -I{} sh -c '{}'
