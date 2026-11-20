#!/bin/sh
# after ac_all.sh: lead place by distance, lead tables (L27/L29), lanes / far range (L24-M) before and after
W=$HOME/projects/wt/roadsense-l32; AC=$HOME/datasets/l32/ac; R=$HOME/datasets/l32/res; mkdir -p $R
PY=$HOME/projects/roadsense/.venv-models/bin/python
cd $AC
for t in off warm cold pcold poff; do
  python3 $W/tools/vision_eval/lead_lateral.py $(ls *.$t.13.8.jsonl.gz) --json $R/lat_$t.json > $R/lat_$t.txt
done
# the lead tables: supercombo's stream of L24-M as the reference stream, then the vision streams
mkdir -p $R/road; for f in $AC/*.13.8.jsonl.gz; do ln -sf $f $R/road/; done
for f in $HOME/datasets/openpilot-models/l24/road/*.sc.13.8.jsonl.gz $HOME/datasets/openpilot-models/l24/road/*.vis.20.jsonl.gz; do ln -sf $f $R/road/; done
$PY $W/tools/vision_eval/lead_eval.py extract $R/road $R/lead > $R/extract.log 2>&1
for set in "tune krakow-2024,warsaw-storm,gdansk-night-snow,tunnels-s3" "held s7-elblag-lowsun,krakow-night-tunnel,a1-night,warsaw-city"; do
  set -- $set
  $PY $W/tools/vision_eval/lead_eval.py score $R/lead --rides $2 --streams sc.13.8,off.13.8,warm.13.8,cold.13.8,pcold.13.8,poff.13.8 --json $R/lead_$1.json > $R/lead_$1.txt 2>&1
done
$PY $W/tools/vision_eval/evaluate.py $R/road --rides a1-night,warsaw-city,krakow-2024,tunnels-s3,warsaw-storm --streams sc.13.8,vis.20,off.13.8,warm.13.8,cold.13.8,pcold.13.8,poff.13.8 --out $R/eval.json > $R/eval.log 2>&1
$PY $W/tools/vision_eval/summarize.py $R/eval.json --ref off.13.8 --tests warm.13.8,cold.13.8,pcold.13.8,poff.13.8 > $R/eval_summary.md 2>&1
echo scored
