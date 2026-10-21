# Third-party components

| Component | Where | License | Notes |
|---|---|---|---|
| openpilot `supercombo.onnx` v0.9.7 | `scripts/fetch-models.sh` | MIT (comma.ai, part of the openpilot repository) | 51,452,435 bytes, sha256 `b31b504bc0b440d3bc72967507a00eb4f112285626fbfb3135011500325ee6d6` |
| openpilot output parsing constants | `modeld/` (re-implemented in C++) | MIT | index tables from `selfdrive/modeld/constants.py` |
| ONNX Runtime | Alpine package `onnxruntime` | MIT | CPU execution provider |
| libcamera | Alpine package `libcamera` | LGPL-2.1+ (library), used unmodified | |

Sign detection is not part of roadsense. The reference sign provider,
[roadsense-signs](https://github.com/PiotrTopa/roadsense-signs), is a separate
AGPL-3.0 project because its models are Ultralytics-trained (AGPL-3.0 claim on
the weights) and carry no licence of their own; it talks to roadsense only over
the frame ring and a JSON socket (docs/ARCHITECTURE.md), so no sign code or model
is linked into or shipped with roadsense.
