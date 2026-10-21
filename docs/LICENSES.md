# Third-party components

| Component | Where | License | Notes |
|---|---|---|---|
| openpilot `supercombo.onnx` v0.9.7 | `scripts/fetch-models.sh` | MIT (comma.ai, part of the openpilot repository) | 51,452,435 bytes, sha256 `b31b504bc0b440d3bc72967507a00eb4f112285626fbfb3135011500325ee6d6` |
| openpilot output parsing constants | `modeld/` (re-implemented in C++) | MIT | index tables from `selfdrive/modeld/constants.py` |
| ONNX Runtime | Alpine package `onnxruntime` | MIT | CPU execution provider |
| libcamera | Alpine package `libcamera` | LGPL-2.1+ (library), used unmodified | |
| traffic-sign detector `mtsd_signdet_yolov8n_384x640.onnx` + classifier `mtsd_signcls_yolov8n_64.onnx` | exported locally from `jkot/pova-traffic-sign-recognition-models` (Hugging Face, commit `0c135006544630d813603f48866de2f11287c518`) | **no licence on the weights** (all rights reserved by default); trained with Ultralytics (AGPL-3.0 claim on weights) on Mapillary Traffic Sign Dataset (CC BY-NC-SA 4.0, non-commercial) | private, non-commercial use only; never re-host; see docs/SIGN_MODELS.md for the export recipe |
