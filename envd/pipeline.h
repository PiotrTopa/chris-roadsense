// envd pipeline: NV12 frame -> preprocessing -> image encoder -> cosine scores against the class
// text embeddings -> per-group softmax -> temporal smoothing. Shared by envd and tools/envd_replay.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "classes.h"
#include "encoder.h"
#include "prep.h"
#include "smoother.h"

namespace roadsense {
namespace envd {

struct PipelineConfig {
    EncoderOptions enc;
    std::string classes_path = "/var/lib/roadsense/models/envd-classes.json";
    double window_s = 10;      // smoothing window
    double hysteresis = 0.10;  // smoothed-score margin a challenger needs to take the label
    double min_hold_s = 3;     // no label change within this time of the previous one
    int ignore_bottom = 0;     // frame rows ignored at the bottom (burned-in overlays)
};

struct GroupResult {
    const ClassGroup *group = nullptr;
    std::vector<float> frame;      // this frame's softmax
    std::vector<double> scores;    // smoothed
    int frame_label = -1, label = -1;
    double confidence = 0, stable_s = 0;
};

struct EnvResult {
    std::vector<GroupResult> groups;
    FrameStats stats;
    double prep_ms = 0, encode_ms = 0, total_ms = 0;
};

class EnvPipeline {
public:
    explicit EnvPipeline(const PipelineConfig &cfg);  // throws std::runtime_error
    // t_s: frame time in seconds (monotonic), drives the smoothing window.
    void process(const Nv12View &v, double t_s, EnvResult *out);
    void reset();  // forget the smoothing history (camera restart)
    const ClassFile &classes() const { return cf_; }
    const char *backend() const { return enc_->name(); }
    const std::vector<float> &embedding() const { return emb_; }  // last frame's, L2-normalised
    const PipelineConfig &config() const { return cfg_; }

private:
    PipelineConfig cfg_;
    ClassFile cf_;
    std::unique_ptr<Preprocessor> prep_;
    std::unique_ptr<ImageEncoder> enc_;
    std::vector<GroupSmoother> smooth_;
    std::vector<float> input_, emb_;
};

}  // namespace envd
}  // namespace roadsense
