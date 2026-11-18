// openpilot v0.11.1 driving_policy on ONNX Runtime (CPU), fed by the driving_vision features.
//
// openpilot (compile_modeld.py) pushes every 20 Hz vision run's hidden state into a queue and gives the
// policy every 4th entry: 25 rows 200 ms apart, newest = this run. roadsense runs the vision model at
// whatever rate the GPU allows (13-20 runs/s), so the history is kept by camera time: row k is the stored
// run closest to t - 0.2 k s, if one lies within 50 ms of it and the history reaches back that far, else
// zeros (openpilot's queues also start at zero; at 20 runs/s the rows are exactly openpilot's).
// desire_pulse is zero (no lane-change desire in roadsense), traffic_convention as for supercombo.
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "vision_layout.h"

namespace roadsense {

class VisionPolicy {
public:
    VisionPolicy(const std::string &path, int threads, bool rhd, bool allow_spinning);
    ~VisionPolicy();
    // hidden: the run's 512 features; ts_ns: camera time of the run's current frame.
    // out: vis::kPolicyOutputLen values. Returns false (err) on an ORT error.
    bool run(const float *hidden, int64_t ts_ns, float *out, std::string *err);
    // Adds a run's features to the history without running the policy.
    void push(const float *hidden, int64_t ts_ns);
    int rows_filled() const { return rows_filled_; }  // history rows found in the last run (of 25)

private:
    bool infer(int64_t ts_ns, float *out, std::string *err);
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::deque<std::pair<int64_t, std::array<float, vis::kHiddenLen>>> hist_;
    std::vector<float> feat_, desire_, traffic_;
    int rows_filled_ = 0;
};

}  // namespace roadsense
