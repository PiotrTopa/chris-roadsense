// ncnn + Vulkan backend for supercombo v0.9.7 (see backend.h). The model is the converted
// graph from tools/ncnn/make_ncnn.sh: <stem>.param + <stem>.bin, inputs in0..in6 in spec order
// except in6, which is the 8 features_buffer rows the graph actually reads ([8,512], rows
// 94, 89, ..., 59 of the [99,512] buffer), gathered here on the host; output out0 (6504).
//
// Precision: fp16-packed storage with fp32 arithmetic on the GPU (the Adreno 630 turnip driver has
// no VK_KHR_16bit_storage, so ncnn's fp16 storage/arithmetic modes are unavailable there and GPU
// fp32 is ~5x slower). The five small layers ncnn has no Vulkan kernel for run on the CPU.
//
// Turnip workaround: on the Adreno 630 with Mesa turnip 26.2, ncnn 20261020 returns randomly
// corrupted outputs for this graph (about half the runs; errors from 0.1 m to hundreds of metres,
// no error code). Turnip's TU_DEBUG=flushall (flush + invalidate all GPU caches around every
// dispatch) makes every run bit-identical and correct for ~+5 ms per run; TU_DEBUG=syncdraw,
// ncnn light mode off, packing/subgroup/local-memory options and a conservative barrier patch
// in ncnn's blob allocator do not. So the backend adds "flushall" to TU_DEBUG before Vulkan
// starts (other drivers ignore the variable); opt.ncnn_flushall = false (--ncnn-no-flushall)
// skips that.
#pragma once

#include "backend.h"

namespace roadsense {
// Declared in backend.h when ROADSENSE_HAVE_NCNN is defined:
//   std::unique_ptr<InferenceBackend> make_ncnn_backend(const SupercomboOptions &opt);
// opt.model_path is the .param file; the .bin next to it (same stem) holds the weights.
// opt.threads is ncnn's CPU thread count (CPU-fallback layers, default 1 in modeld).
// opt.ncnn_fp16 = false disables fp16 packing (GPU fp32 everywhere, ~320 ms/run on the SD845).
}  // namespace roadsense
