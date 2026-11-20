#!/bin/sh
A=$HOME/datasets/l32/scripts/ac.sh; O=$HOME/datasets/l32/ac; ST=$HOME/datasets/l32/ac/state; mkdir -p $ST
while read slug s d; do
  rm -f $ST/$slug.json $ST/$slug.p.json
  sh $A $slug $s $d off $O 0 0                                             # before: the ride's calib.json as it is
  sh $A $slug $s $d cold $O 0 0 --auto-calib on --calib-state $ST/$slug.json   # estimate from nothing during the ride
  sh $A $slug $s $d warm $O 0 0 --auto-calib on --calib-state $ST/$slug.json   # second pass: estimate known from the start
  sh $A $slug $s $d pcold $O 3.4 -1.4 --auto-calib on --calib-state $ST/$slug.p.json  # config off by the car's error
  sh $A $slug $s $d poff $O 3.4 -1.4                                   # the same wrong config, no estimate
  echo "$(date +%H:%M:%S) $slug done"
done <<LIST
a1-night 0 600
warsaw-city 0 900
krakow-2024 60 240
tunnels-s3 0 300
s7-elblag-lowsun 60 240
krakow-night-tunnel 0 900
gdansk-night-snow 60 240
warsaw-storm 60 240
LIST
