#!/bin/sh
A=$HOME/datasets/l32/scripts/ac.sh; O=$HOME/datasets/l32/ac; ST=$O/state
slug=krakow-2024; s=60; d=240
rm -f $ST/$slug.json $ST/$slug.p.json
sh $A $slug $s $d cold $O 0 0 --auto-calib on --calib-state $ST/$slug.json
sh $A $slug $s $d warm $O 0 0 --auto-calib on --calib-state $ST/$slug.json
sh $A $slug $s $d pcold $O 3.4 -1.4 --auto-calib on --calib-state $ST/$slug.p.json
