#!/bin/sh
# hottest thermal zone in C (zones under 150 C), lane L28
for z in /sys/class/thermal/thermal_zone*/temp; do cat $z; done | awk '$1<150000{if($1>m)m=$1}END{printf "%.1f\n", m/1000}'
