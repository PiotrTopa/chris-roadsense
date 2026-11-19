# The fault table

Every fault class of the taxonomy (DESIGN.md section 5a) injected into the simulated town drive (seed 1, 240 s,
the fault from 95 s to 150 s unless it is a permanent change), through the integrator as built. Reproduce with
`tools/fault_campaign.py sim` of the chris-roadsense repository; `tests/test_faults.py` asserts a subset.

Columns: **detected after**: seconds from the onset to the first alarm that the unfaulted run does not have (a
source `fault` / `suspect` / `stale`, a finding in `ego_state.diagnostics`). **isolated**: `ok` the injected
source (or one that shares its sensor) was taken out; `named (warning)` a finding names it and nothing was taken
out; `none`; `WRONG` another source was taken out. **wrong-but-valid**: seconds during which an output was
published valid and was wrong by more than its tolerance (speed 0.5 m/s, yaw rate 0.02 rad/s, position 15 m,
heading 6 degrees, lead distance 4 m) *and* by more than 3 of its own stated sigma, beyond what the unfaulted
run shows: the cost that matters. **recovered after**: seconds from the end of the fault until alarms and errors
are gone (`-`: not within the run).

"Before" is the integrator before this work (the same faults, the same scoring).

## The reference car's sensor set (vehicle, camera, receiver, map)

| fault | before: detected after (s) | isolated | wrong-but-valid | after: detected after (s) | isolated | wrong-but-valid | largest such error | recovered after (s) |
|---|---|---|---|---|---|---|---|---|
| vehicle speed: bias +2 m/s | 12.4 | none | position 32 s, speed 55 s | 2.9 | ok + gnss_track | position 20 s, speed 6 s | position 40.3, speed 2.09 | 32 |
| vehicle speed: scale 0.7 | 1.9 | WRONG: vision | position 43 s, speed 55 s | 0.6 | ok + gnss_track | position 10 s, speed 1 s | position 27.6, speed 2.83 | 36 |
| vehicle speed: scale 0.9 | 26.4 | none | position 20 s, speed 55 s | 1.9 | ok + gnss_track | position 40 s, speed 5 s | position 26.3, speed 1.45 | 35 |
| vehicle speed: slow scale drift to +17 % | 57.5 | none | position 14 s, speed 44 s | 31.0 | ok + gnss_track | position 5 s, speed 30 s | position 16.1, speed 1.81 | 32 |
| vehicle speed: stuck | 10.0 | WRONG: vision | position 45 s, speed 49 s | 1.6 | ok + gnss_track | position 6 s | position 33.5 | - |
| vehicle speed: each sample repeated 5x | - | none | 0 | 2.5 | named (warning) | 0 | - | 29 |
| vehicle speed: noise burst 1 m/s | 4.5 | named (warning) | speed 18 s | 0.4 | ok | speed 3 s | speed 1.47 | - |
| vehicle speed: outliers 10 % of 5 m/s | 15.6 | named (warning) | speed 16 s | 0.6 | ok | speed 13 s | speed 3.62 | - |
| vehicle speed: unit (km/h read as m/s) | 1.8 | WRONG: vision | heading 2 s, position 53 s, speed 57 s, yaw_rate 1 s | 0.5 | ok + gnss_track | position 12 s, speed 0 s | position 251, speed 19.7 | - |
| vehicle speed: sign | 1.8 | WRONG: vision | heading 3 s, position 48 s, speed 57 s | 0.5 | ok + gnss_track | position 12 s, speed 0 s | position 205, speed 18.2 | - |
| vehicle speed signal only: scale 0.7 (wheels right) | 1.7 | ok + vision | heading 0 s, position 2 s, speed 3 s | 0.6 | ok + gnss_track | position 6 s, speed 1 s | position 27.6, speed 2.83 | 36 |
| vehicle wheels only: scale 0.8 | 1.7 | ok | 0 | 1.0 | ok + gnss_track | 0 | - | 35 |
| gyro: bias 0.02 rad/s | 14.2 | none | heading 21 s, position 10 s, yaw_rate 2 s | 1.4 | ok | heading 14 s, yaw_rate 2 s | heading 9.36, yaw_rate 0.02 | 1 |
| gyro: scale 1.3 | 43.8 | none | heading 19 s, position 12 s, yaw_rate 14 s | 36.9 | ok | heading 19 s, position 16 s, yaw_rate 14 s | heading 17.3, position 22.9, yaw_rate 0.04 | 12 |
| gyro: slow bias drift to 0.03 rad/s | 33.8 | none | heading 20 s, position 13 s, yaw_rate 5 s | 21.0 | ok | heading 18 s, position 15 s | heading 13.6, position 24.4 | 39 |
| gyro: stuck | 37.9 | none | heading 18 s, position 14 s, yaw_rate 15 s | 2.1 | ok | heading 14 s, position 14 s, yaw_rate 1 s | heading 46.5, position 157, yaw_rate 0.11 | - |
| gyro: noise burst 0.03 rad/s | 6.2 | none | heading 17 s, position 13 s, yaw_rate 34 s | 0.4 | named (warning) | yaw_rate 1 s | yaw_rate 0.06 | 7 |
| gyro: sign | 36.4 | named (warning) | heading 17 s, position 14 s, yaw_rate 15 s | 34.8 | ok | yaw_rate 1 s | yaw_rate 0.13 | - |
| gyro: unit (deg/s read as rad/s) | 1.8 | named (warning) | heading 57 s, lead 2 s, position 50 s, yaw_rate 54 s | 0.3 | ok | 0 | - | - |
| accelerometer: bias 1 m/s2 | 27.4 | none | 0 | 31.5 | none | 0 | - | - |
| accelerometer: sign | 10.8 | named (warning) | 0 | 10.9 | named (warning) | 0 | - | 45 |
| accelerometer: stuck | - | none | 0 | 2.1 | named (warning) | 0 | - | 6 |
| vehicle feed: drop-out | 0.6 | ok | 0 | 0.6 | ok | 0 | - | 0 |
| vehicle feed: silent 7 s, 3 stale samples on return | 0.6 | ok | 0 | 0.6 | ok | 0 | - | 18 |
| vehicle feed: content 0.5 s old (delay jump) | - | none | speed 3 s, yaw_rate 3 s | 49.9 | named (warning) | speed 3 s, yaw_rate 3 s | speed 0.7, yaw_rate 0.07 | 0 |
| vehicle feed: stamps 0.5 s late | 0.5 | WRONG: vision | speed 13 s, yaw_rate 12 s | 0.5 | WRONG: vision | speed 13 s, yaw_rate 12 s | speed 0.62, yaw_rate 0.06 | 10 |
| vehicle feed: stamps 0.5 s early | - | none | 0 | 0.7 | named (warning) | 0 | - | 4 |
| camera speed: scale 0.3 | 1.8 | ok | 0 | 0.5 | ok + gnss_track | 0 | - | 35 |
| camera speed: scale 0.85 | - | none | 0 | 7.8 | ok | 0 | - | 1 |
| camera speed: bias +1.5 m/s | - | none | 0 | 1.1 | ok | 0 | - | 5 |
| camera speed: slow scale drift to -17 % | - | none | 0 | 30.6 | named (warning) | 0 | - | 1 |
| camera speed: stuck | 10.5 | ok | 0 | 1.5 | ok + gnss_track | 0 | - | 35 |
| camera speed: noise burst 2 m/s | 5.2 | named (warning) | 0 | 3.4 | named (warning) | 0 | - | 0 |
| camera speed: noise 1.5 m/s and stated std x 0.2 (lies) | 5.3 | named (warning) | 0 | 6.6 | named (warning) | 0 | - | 0 |
| camera yaw rate: sign | 36.4 | named (warning) | 0 | 34.8 | ok | 0 | - | - |
| camera yaw rate: scale 1.5 | 38.6 | named (warning) | heading 5 s | 40.1 | named (warning) | 0 | - | 0 |
| camera feed: drop-out | 0.5 | ok | 0 | 0.7 | ok | 0 | - | 0 |
| camera feed: content 0.3 s old (delay jump) | - | none | 0 | - | none | 0 | - | 0 |
| camera feed: stamps 0.3 s late | - | none | 0 | 0.9 | named (warning) | 0 | - | 5 |
| camera mount: pitch step 1.7 deg | 3.1 | named (warning) | 0 | 0.6 | named (warning) | 0 | - | 0 |
| camera mount: pitch step 8 deg (model out of range) | 1.8 | ok | lead 2 s | 0.4 | ok | lead 2 s | lead 52.1 | 0 |
| camera mount: slow pitch creep 0.03 deg/s | 49.0 | named (warning) | 0 | 29.3 | named (warning) | 0 | - | 9 |
| camera lead: range scale 1.3 | - | none | lead 44 s | - | none | lead 44 s | lead 100 | 0 |
| camera lead: phantom lead at 12 m | - | none | lead 55 s | - | none | lead 55 s | lead 64.7 | 1 |
| receiver: position jump 30 m | 4.4 | named (warning) | position 48 s | 0.4 | ok | position 47 s | position 28.3 | 32 |
| receiver: slow position drift 1 m/s | 39.8 | none | position 26 s | 36.4 | ok | position 35 s | position 40.6 | 32 |
| receiver: noise burst 15 m | 7.3 | named (warning) | 0 | 0.4 | ok | 0 | - | - |
| receiver: outliers 20 % of 60 m | - | none | 0 | 0.4 | ok | 0 | - | - |
| receiver: frozen fix while moving | 5.4 | ok | 0 | 5.4 | ok | 0 | - | 35 |
| receiver: lat / lon swapped (frame) | 5.4 | named (warning) | position 42 s | 3.4 | ok | position 39 s | position 508 | - |
| receiver: noise 15 m and stated accuracy x 0.3 (lies) | 7.3 | named (warning) | 0 | 0.4 | ok | 0 | - | - |
| receiver: drop-out | 4.4 | ok | 0 | 4.3 | ok | 0 | - | 0 |
| receiver: fixes 2 s older (delay jump) | 5.4 | named (warning) | position 38 s | 6.4 | ok | position 36 s | position 30.5 | 35 |
| map: matcher on a parallel road 25 m away | 4.3 | named (warning) | 0 | 4.3 | named (warning) | 0 | - | 24 |
| map: bearing turned 90 deg (frame) | - | none | 0 | - | none | 0 | - | 0 |
| clock: epoch change (all stamps step back 600 s) | -0.1 | WRONG: vehicle,wheels | 0 | -0.1 | ok + vehicle,wheels | 0 | - | 0 |
| two together: vehicle and camera speed x 0.7 | 2.2 | named (warning) | position 43 s, speed 55 s | 1.9 | WRONG: gnss_track | position 43 s, speed 55 s | position 66.7, speed 4.3 | 32 |
| two together: vehicle speed stuck, receiver out | 4.4 | ok + vision | position 54 s, speed 49 s | 1.6 | ok | position 6 s | position 32.9 | - |
| two together: camera out of range, receiver jump 30 m | 1.8 | ok | lead 2 s, position 58 s | 0.4 | ok | position 57 s | position 28.3 | 0 |

before: 59 faults: detected 46, isolated correctly 12, wrong source blamed 6, no output confidently wrong 29; confidently-wrong output time in all 1657 s

after: 59 faults: detected 55, isolated correctly 38, wrong source blamed 2, no output confidently wrong 31; confidently-wrong output time in all 748 s

## With a front radar (radar faults)

| fault | before: detected after (s) | isolated | wrong-but-valid | after: detected after (s) | isolated | wrong-but-valid | largest such error | recovered after (s) |
|---|---|---|---|---|---|---|---|---|
| radar: range bias 3 m | - | none | 0 | - | none | 0 | - | 0 |
| radar: range scale 1.1 | - | none | lead 1 s | - | none | lead 1 s | lead 84.8 | 0 |
| radar: range rate bias 1 m/s | 3.7 | ok | lead 4 s | 2.1 | ok | 0 | - | 33 |
| radar: mount knocked 3 deg | 8.8 | named (warning) | 0 | 8.8 | named (warning) | 0 | - | 0 |
| radar: azimuth sign (frame) | 37.9 | named (warning) | lead 10 s | 37.8 | named (warning) | lead 8 s | lead 79.1 | 2 |
| radar: drop-out | 0.4 | ok | 0 | 0.6 | ok | 0 | - | 15 |
| radar: content 0.2 s old | 13.7 | named (warning) | lead 1 s | 13.4 | named (warning) | lead 2 s | lead 79.2 | 0 |
| radar: ghosts | 3.6 | ok | lead 29 s | 1.6 | ok | lead 18 s | lead 77.2 | 53 |

before: 8 faults: detected 6, isolated correctly 3, wrong source blamed 0, no output confidently wrong 3; confidently-wrong output time in all 45 s

after: 8 faults: detected 6, isolated correctly 3, wrong source blamed 0, no output confidently wrong 4; confidently-wrong output time in all 29 s
