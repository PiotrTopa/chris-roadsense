#!/bin/sh
S=$HOME/datasets/l32/scripts/sweep.sh
{
echo "$S a1-night 60 60 supercombo base 0 0"
echo "$S a1-night 60 60 supercombo yaw+2 2 0"
echo "$S a1-night 60 60 supercombo yaw-2 -2 0"
echo "$S a1-night 60 60 vision pitch+1 0 0 1"
echo "$S a1-night 60 60 vision pitch-1 0 0 -1"
echo "$S a1-night 60 60 vision yaw+2 2 0"
echo "$S a1-night 60 60 vision yaw-2 -2 0"
echo "$S warsaw-city 210 60 vision base 0 0"
echo "$S warsaw-city 210 60 vision yaw+2 2 0"
echo "$S warsaw-city 210 60 vision yaw-2 -2 0"
echo "$S tunnels-s3 30 60 vision base 0 0"
echo "$S tunnels-s3 30 60 vision yaw+2 2 0"
echo "$S tunnels-s3 30 60 vision yaw-2 -2 0"
echo "$S warsaw-city 210 60 supercombo base 0 0"
echo "$S warsaw-city 210 60 supercombo yaw+2 2 0"
echo "$S warsaw-city 210 60 supercombo yaw-2 -2 0"
echo "$S a1-night 60 60 supercombo pitch+1 0 0 1"
} | xargs -P 5 -I{} sh -c '{}'
