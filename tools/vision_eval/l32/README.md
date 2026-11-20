# Lane L32 tools (docs/lanes/L32-lead-lateral.md)

The scripts as they were run on the laptop; paths of the laptop are in the `.sh` drivers (rides in
`<datasets>/rides`, outputs in `<datasets>/l32`, the car repo and the pre-2.0 worktrees). The Python tools take
their paths as arguments.

| what | files |
|---|---|
| the chain trace: `road` stream -> the api's `State` (any version) -> the car backend's `SceneBuilder` -> the MFD frontend's director (any car repo version); the lead against the lane centre at every stage by own speed | `chain_api.py`, `chain_backend.py`, `chain_frontend.py`, `chain_table.py`, `chain_episode.py`, driver `chain.sh` |
| the regression bisect by configuration (pre-2.0 chain / 2.0 code with supercombo / with vision / new backend with the old frontend) | `bisect.sh`, `bisect_table.py` |
| host motion injected (exact, yaw-rate bias, speed-proportional yaw-rate error) | `inject.sh` (`chain_api.py --ego ...`) |
| calibration perturbation replays and their comparison | `sweep.sh`, `sweep2.sh`, `sens.py`, `sens_speed.py` (by own speed), `byspeed.py` |
| before / after replays with the online calibration, scoring | `ac.sh`, `ac_all.sh`, `ac_krakow.sh`, `ac_sc.sh` (supercombo), `score.sh`, `tables.py`, `eval_table.py` |
| modeld as a daemon with camerad-replay (wrong calibration, restart) | `daemon_test.sh` |
| the car's recordings | `gates.py`, `yawrate.py`, `still.py`; `../calib_from_pose.py` |
