#!/bin/sh
# The regression bisect by configuration on one ride (default warsaw-city, 900 s with traffic-light stops), through the chain
# to the MFD director. Calibration: ride = the ride's calib.json; car = that made wrong by the car's error (yaw +3.4, pitch -1.4 deg).
R=${1:-warsaw-city}
AC=/home/piotr/datasets/l32/ac; C=/home/piotr/datasets/l32/scripts/chain.sh
NEWAPI=/home/piotr/projects/roadsense/api; OLDAPI=/home/piotr/projects/wt/roadsense-l32-pre2/api
NEWCAR=/home/piotr/projects/cyberpank-prius-gen2-computer; OLDCAR=/home/piotr/projects/wt/prius-l32-pre2
{
for cal in car ride; do
  if [ $cal = car ]; then sc=scpoff; vis=poff; else sc=scoff; vis=off; fi
  echo "sh $C $R.A_pre2_sc.$cal   $AC/$R.$sc.13.8.jsonl.gz  $OLDAPI $OLDCAR $OLDCAR"   # A: the chain before 2.0, supercombo
  echo "sh $C $R.B_v2_sc.$cal     $AC/$R.$sc.13.8.jsonl.gz  $NEWAPI $NEWCAR $NEWCAR"   # B: 2.0 code, supercombo
  echo "sh $C $R.C_v2_vis.$cal    $AC/$R.$vis.13.8.jsonl.gz $NEWAPI $NEWCAR $NEWCAR"   # C: 2.0 code, vision (installed default)
  echo "sh $C $R.M_v2_vis_oldfe.$cal $AC/$R.$vis.13.8.jsonl.gz $NEWAPI $NEWCAR $OLDCAR" # M: 2.0 api + backend, the Pi frontend from before 2.0
  echo "sh $C $R.N_v2_sc_oldfe.$cal  $AC/$R.$sc.13.8.jsonl.gz  $NEWAPI $NEWCAR $OLDCAR"
  echo "sh $C $R.V_pre2_vis.$cal  $AC/$R.$vis.13.8.jsonl.gz $OLDAPI $OLDCAR $OLDCAR"   # V: the chain before 2.0 fed with the vision model (model alone)
done
} | xargs -P 4 -I{} sh -c '{}'
echo bisect done
