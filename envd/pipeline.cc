#include "pipeline.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace roadsense {
namespace envd {

using Clock = std::chrono::steady_clock;

EnvPipeline::EnvPipeline(const PipelineConfig &cfg) : cfg_(cfg), cf_(load_class_file(cfg.classes_path)) {
    cfg_.enc.size = cf_.image_size;
    cfg_.enc.embed_dim = cf_.embed_dim;
    prep_ = std::make_unique<Preprocessor>(cf_.image_size, cf_.mean, cf_.std, cfg.ignore_bottom);
    enc_ = make_encoder(cfg_.enc);
    for (const ClassGroup &g : cf_.groups)
        smooth_.emplace_back(int(g.classes.size()), cfg.window_s, cfg.hysteresis, cfg.min_hold_s);
    input_.resize(size_t(3) * cf_.image_size * cf_.image_size);
    emb_.resize(size_t(cf_.embed_dim));
}

void EnvPipeline::reset() {
    for (GroupSmoother &s : smooth_) s.reset();
}

void EnvPipeline::process(const Nv12View &v, double t_s, EnvResult *out) {
    const auto t0 = Clock::now();
    prep_->run(v, input_.data(), &out->stats);
    const auto t1 = Clock::now();
    enc_->encode(input_.data(), emb_.data());
    const auto t2 = Clock::now();
    double n = 0;
    for (float x : emb_) n += double(x) * x;
    n = std::sqrt(std::max(n, 1e-12));
    for (float &x : emb_) x = float(x / n);

    out->groups.resize(cf_.groups.size());
    for (size_t gi = 0; gi < cf_.groups.size(); ++gi) {
        const ClassGroup &g = cf_.groups[gi];
        GroupResult &r = out->groups[gi];
        r.group = &g;
        const size_t k = g.classes.size(), d = size_t(cf_.embed_dim);
        r.frame.assign(k, 0.f);
        float mx = -1e30f;
        for (size_t c = 0; c < k; ++c) {
            double dot = 0;
            const float *row = g.emb.data() + c * d;
            for (size_t j = 0; j < d; ++j) dot += double(row[j]) * emb_[j];
            r.frame[c] = float(cf_.logit_scale * dot);
            mx = std::max(mx, r.frame[c]);
        }
        double sum = 0;
        for (float &x : r.frame) {
            x = std::exp(x - mx);
            sum += x;
        }
        for (float &x : r.frame) x = float(x / sum);
        r.frame_label = int(std::max_element(r.frame.begin(), r.frame.end()) - r.frame.begin());
        smooth_[gi].update(r.frame.data(), t_s);
        r.scores = smooth_[gi].scores();
        r.label = smooth_[gi].label();
        r.confidence = smooth_[gi].confidence();
        r.stable_s = smooth_[gi].stable_s(t_s);
    }
    const auto t3 = Clock::now();
    out->prep_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    out->encode_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    out->total_ms = std::chrono::duration<double, std::milli>(t3 - t0).count();
}

}  // namespace envd
}  // namespace roadsense
