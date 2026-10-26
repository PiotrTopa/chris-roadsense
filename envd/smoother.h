// Temporal smoothing with hysteresis for one class group.
//
// Keeps the per-frame probability vectors of the last `window_s` seconds and reports their mean
// ("scores"). The reported label starts as the arg-max of the first sample and then only moves to
// another class when that class's smoothed score exceeds the current label's by `hysteresis`
// (so a 50/50 flicker between two classes never toggles the label), and never within `min_hold_s`
// of the last change. Confidence = smoothed score of the reported label.
#pragma once

#include <algorithm>
#include <deque>
#include <vector>

namespace roadsense {
namespace envd {

class GroupSmoother {
public:
    GroupSmoother(int n, double window_s, double hysteresis, double min_hold_s)
        : n_(n), window_s_(window_s), hyst_(hysteresis), hold_s_(min_hold_s), smoothed_(size_t(n), 0.0) {}

    void reset() {
        hist_.clear();
        label_ = -1;
        changed_t_ = 0;
    }

    // p: n probabilities of this frame; t_s: its time in seconds (monotonic).
    void update(const float *p, double t_s) {
        hist_.push_back({t_s, std::vector<float>(p, p + n_)});
        while (!hist_.empty() && t_s - hist_.front().t > window_s_) hist_.pop_front();
        std::fill(smoothed_.begin(), smoothed_.end(), 0.0);
        for (const Sample &s : hist_)
            for (int i = 0; i < n_; ++i) smoothed_[size_t(i)] += s.p[size_t(i)];
        for (double &v : smoothed_) v /= double(hist_.size());
        const int best = int(std::max_element(smoothed_.begin(), smoothed_.end()) - smoothed_.begin());
        if (label_ < 0) {
            label_ = best;
            changed_t_ = t_s;
            changes_ = 0;
        } else if (best != label_ && smoothed_[size_t(best)] > smoothed_[size_t(label_)] + hyst_ &&
                   t_s - changed_t_ >= hold_s_) {
            label_ = best;
            changed_t_ = t_s;
            ++changes_;
        }
    }

    int label() const { return label_; }
    double confidence() const { return label_ < 0 ? 0.0 : smoothed_[size_t(label_)]; }
    double stable_s(double now_s) const { return label_ < 0 ? 0.0 : now_s - changed_t_; }
    const std::vector<double> &scores() const { return smoothed_; }
    size_t samples() const { return hist_.size(); }
    unsigned changes() const { return changes_; }

private:
    struct Sample {
        double t;
        std::vector<float> p;
    };
    int n_;
    double window_s_, hyst_, hold_s_;
    std::deque<Sample> hist_;
    std::vector<double> smoothed_;
    int label_ = -1;
    double changed_t_ = 0;
    unsigned changes_ = 0;
};

}  // namespace envd
}  // namespace roadsense
