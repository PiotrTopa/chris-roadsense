# roadsense notebook

## Fold 0 (2026-10-23): baseline
Facts (measured on the phone, DVR camera behind the hub, see docs/PHONE_DVR_DEPLOY.md):
1. camerad-dvr 19.5 fps into the ring, ~3 % of a core.
2. modeld on the Adreno 630 (ncnn/Vulkan, fp16-packed, `TU_DEBUG=flushall`): 71-79 ms per run,
   9.3-11.5 runs/s (12/s at its best on 2026-10-21), die 63-70 C without signd.
3. signd on 4 big cores (ONNX Runtime, MTSD YOLOv8n 384x640 + 64x64 classifier): 170-200 ms,
   4.8-5.6 fps; drives the die to a ~95 C plateau without a fan, ~86 C with the user's
   improved cooling (23:13 CEST).
Open question that opens L1: can signs move to the GPU, or is the GPU already saturated by modeld?

## Lanes
| Lane | Model | State | Report |
|---|---|---|---|
| L1 signs-vulkan | Fable | running (2026-10-23) | docs/lanes/L1-signs-vulkan.md |
