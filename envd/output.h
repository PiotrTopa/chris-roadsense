// The `env` JSON line (docs/ARCHITECTURE.md). Shared by envd and tools/envd_replay.
#pragma once

#include <cstdint>
#include <string>

#include "pipeline.h"

namespace roadsense {
namespace envd {

struct FrameMeta {
    uint64_t seq = 0;
    int64_t ts_mono_ns = 0, ts_real_ns = 0;
};

// model: a short name for the "model" field (e.g. "ViT-B-32/openai ort").
std::string env_json(const FrameMeta &m, const EnvResult &r, const std::string &model, bool with_frame_scores = true);

}  // namespace envd
}  // namespace roadsense
