#!/bin/sh
# usage: chain.sh NAME ROAD.jsonl.gz API_DIR BACKEND_REPO FRONTEND_REPO [chain_api.py args]
# road stream -> api State -> car backend SceneBuilder -> MFD frontend director; table by own speed at every stage
set -eu
N=$1; ROAD=$2; API=$3; BE=$4; FE=$5; shift 5
D=$HOME/datasets/l32/chain; S=$HOME/datasets/l32/scripts
PA=$HOME/projects/roadsense/.venv-models/bin/python; PF=$HOME/projects/cyberpank-prius-gen2-computer/.venv-fe/bin/python
$PA $S/chain_api.py "$ROAD" $D/$N.ws.jsonl.gz --api "$API" "$@" 2>/dev/null
$PF $S/chain_backend.py $D/$N.ws.jsonl.gz $D/$N.scenes.jsonl.gz --repo "$BE" 2>/dev/null >/dev/null
$PF $S/chain_frontend.py $D/$N.scenes.jsonl.gz $D/$N.ws.jsonl.gz $D/$N.rows.jsonl.gz --repo "$FE" 2>/dev/null >/dev/null
python3 $S/chain_table.py $N $D/$N.ws.jsonl.gz $D/$N.scenes.jsonl.gz $D/$N.rows.jsonl.gz --json $D/$N.json > $D/$N.txt
