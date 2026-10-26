# Order L2-envd-phone: envd on the Poco F1 (build, cost, 30-min run next to the stack)

Placed by lane L2 (envd), 2026-10-26. Execution lane: Opus. Phone: Poco F1 192.168.0.86 (user ssh;
root via `dotinfra vault exec poco_user_password -- ssh user@192.168.0.86 'sudo -S -p "" CMD'`,
password on stdin only, never printed). Background: docs/lanes/L2-envd.md (laptop numbers only;
the user's thermal stress test `rs-stress-signd` occupied the phone during the lane).

Preconditions: `systemctl is-active rs-stress-signd` is `inactive` (never stop it); nothing else
running that the user did not start; never reboot, never touch USB/VBUS; stop everything of this
order the moment any thermal zone passes 95 C (`cat /sys/class/thermal/thermal_zone*/temp | sort -n | tail -1`).

## 1. Build and install (user tree, then root install)
```
# laptop: sync the committed tree and push the envd model files
scripts/sync-phone.sh                                   # ~/roadsense on the phone
scp models/envd-image.onnx models/envd-classes.json user@192.168.0.86:~/roadsense/models/
# the probe-anchored class file (not committed, trained on BDD/DAWN): ~/datasets/envd-cache/envd-classes-probe.json on the P52
# TinyCLIP fallback (row F): ~/datasets/envd-cache/onnx/tinyclip40m.onnx + classes_tinyclip40m.json, renamed as in row F
# optional GPU backend: also models/envd-image-ncnn.param/.bin (tools/envd/make_ncnn.sh; see the lane report
# for whether the ncnn conversion of the chosen tower was verified)
# phone, as user: build only envd (keeps the installed modeld/camerad untouched)
cmake -S ~/roadsense/envd -B ~/l2/build-envd -DCMAKE_BUILD_TYPE=Release \
      -DROADSENSE_NCNN=ON -DNCNN_DIR=/home/user/vulkan-work/ncnn && make -C ~/l2/build-envd -j4
# phone, as root: install the binary, model files and unit; leave the unit stopped
install -m 755 ~user/l2/build-envd/envd ~user/l2/build-envd/envd_replay /usr/local/bin/
install -m 644 ~user/roadsense/models/envd-image.onnx ~user/roadsense/models/envd-classes.json /var/lib/roadsense/models/
install -m 644 ~user/roadsense/systemd/roadsense-envd.service /etc/systemd/system/ && systemctl daemon-reload
```
(If `scripts/install-phone.sh` is run instead, it does all of the above but also rebuilds and
reinstalls modeld/camerad-dvr: fine when the stack is stopped, otherwise prefer the steps above.)

## 2. Cost of one frame, stack stopped and stack running (10 min total)
With the ring live (camerad-dvr running) and modeld + api running as deployed, signd stopped:
```
ROADSENSE_RUNDIR=/tmp/l2 /usr/local/bin/envd --frames 60 --stats 10 --verbose ARGS
```
one run per row, record `env_ms` / `encode_ms` median from the verbose lines, envd CPU (`top -b -n 3 -d 5 | grep envd`),
hottest zone before / after:

| config | ARGS |
|---|---|
| A (unit default) | `--backend ort --threads 2 --cpus 0-3 --max-hz 1` |
| B | `--backend ort --threads 4 --cpus 0-3 --max-hz 1` |
| C (big cores) | `--backend ort --threads 2 --cpus 4-7 --max-hz 1` |
| D (GPU, only if the ncnn model is installed) | `--backend ncnn --cpus 0-3 --max-hz 1` |
| E (uncapped, 30 frames, cost ceiling) | A with `--max-hz 0` and `--frames 30` |
| F (TinyCLIP-40M fallback, 3x cheaper, -4 macro points) | A with `--model ~user/roadsense/models/envd-tinyclip40m.onnx --classes ~user/roadsense/models/envd-classes-tinyclip40m.json` |

Expectation from the laptop (docs/lanes/L2-envd.md section 1b: 89 ms at 2 threads on an i7):
A should land in the 500-1000 ms range on the A55s, i.e. at the 1 s period; B/C ~half of that;
F ~1/3. If A exceeds ~800 ms, the unit file should move to whichever of B/C/F stays under ~500 ms
with the least big-core use (F costs 4 points of macro accuracy).

## 3. 30 minutes live next to the stack
`systemctl start roadsense-envd` with camerad-dvr + modeld + api running (signd as the user left it).
Every 60 s: `curl -s localhost:8080/v1/env | python3 -c 'import json,sys; d=json.load(sys.stdin); print({g: (d[g]["label"], round(d[g]["confidence"],2), d[g]["stable_s"]) for g in ("place","weather","light")}, d["env_ms"])'`,
the modeld rate from its journal, hottest zone, cpu0/cpu4 freq. Record the number of label changes per
group over the 30 min (the bench scene is static: the expectation is 0 changes after the first
10 s, and `stable_s` growing to ~1800). Then `systemctl stop roadsense-envd` unless the user wants it on.

## Report
`docs/lanes/L2-envd-phone.md`: the cost table (config, env_ms median/p90, envd CPU %, modeld rate
with and without envd, temps), the 30-min stability line, and whether the unit's default (A) is
right or should change; phone state at hand-back (envd installed, unit stopped unless told otherwise).
