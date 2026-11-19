# Stack profile of lane L28 for tools/budget/stack.sh: builds of the merged v2 branches in ~/l28 (roadsense 27b9fcc,
# roadsense-signs 3749b60), the new default road model (vision + policy at 5 Hz, models in ~/l28/models), the
# proposed 2.0 arguments of L21/L27 (envd on the little cores, lights at 5 Hz, the car light-state model), no thermal
# flags (the governor decides). SIGND_EXTRA appends signd options (job C).
L28=/home/user/l28
M=/var/lib/roadsense/models
CAMERAD_BIN=$L28/build-rs/camerad-replay/camerad-replay
MODELD_BIN=$L28/build-rs/modeld/modeld
MODELD_ARGS="--models-dir $L28/models --backend ncnn --cpus 4-7 --prep-cpus 0-3 --max-hz 20 --gpu-yield 80 --gpu-lead 2 --cpu-latency-us 0 --stats 10"
SIGND_BIN=$L28/build-signs/signd
SIGND_ARGS="--det-backend ncnn --cls-backend ort --threads 2 --cpus 4-7 --max-hz 5 --gpu-sync turn --stats 10 --lights --lights-backend ncnn-cpu --lights-threads 2 --lights-cpus 4-7 --lights-hz 5 --lights-roi2 0.125,0.125,0.875,0.569 --lights-thr-keep 0.12 --lights-coast 3 --lights-state /home/user/l21/models/lights_state_car_48x24.onnx,$M/lights_state_bdd_48x24.onnx --verifier $M/sign_verifier_32.onnx --class-min-score regulatory--no-entry=0.71 --cls $M/mtsd_signcls_l12_64.onnx --cls-names $M/mtsd_signcls_l12_classes.txt --dist-match-new 2.0 ${SIGND_EXTRA:-}"
ENVD_BIN=$L28/build-rs/envd/envd
ENVD_ARGS="--backend ort --threads 3 --cpus 0-2 --max-hz 1 --stats 60"
API_PY=$L28/roadsense/api
MAPD_PY=$L28/roadsense/mapd
