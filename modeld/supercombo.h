// supercombo v0.9.7 runner (ONNX Runtime C++ API, CPU EP) + output parser.
//
// Input filling and feedback FIFOs follow docs/SUPERCOMBO_SPEC.md sections 1.1 and 4; parsing
// follows section 3 (parse_model_outputs.py). All spatial outputs stay in openpilot's
// calibrated/device frame (x forward, y right, z down), as docs/ARCHITECTURE.md specifies.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "supercombo_layout.h"

namespace pocopilot {

struct SupercomboOptions {
    std::string model_path = "/var/lib/pocopilot/models/supercombo-fp32.onnx";
    int threads = 4;           // intra-op threads (4 big cores on the SD845)
    bool rhd = false;          // false: right-hand traffic (LHD car, e.g. Poland) -> traffic_convention [1,0]
    float steer_delay = sc::kDefaultSteerDelay;
    bool allow_spinning = true;  // ORT intra-op thread spinning
};

using Arr33 = std::array<float, sc::kIdxN>;
using Arr6 = std::array<float, sc::kLeadTrajLen>;

struct RoadOutputs {
    // plan (best hypothesis by weight), 33 x 15 means + position stds
    std::array<std::array<float, sc::kPlanWidth>, sc::kIdxN> plan{};
    std::array<std::array<float, sc::kPlanWidth>, sc::kIdxN> plan_std{};
    int plan_hyp = 0;
    float plan_weight = 0;  // softmax weight of the selected hypothesis
    struct Line {
        Arr33 y{}, z{}, y_std{}, z_std{};
        float prob = 0;
    };
    std::array<Line, sc::kNumLaneLines> lane_lines{};
    std::array<Line, sc::kNumRoadEdges> road_edges{};  // prob unused
    struct Lead {
        float prob = 0;
        Arr6 x{}, y{}, v{}, a{}, x_std{}, y_std{}, v_std{}, a_std{};
        int hyp = 0;
    };
    std::array<Lead, sc::kLeadSelection> leads{};
    std::array<float, 6> pose{}, pose_std{};
    std::array<float, 6> sim_pose{}, sim_pose_std{};
    std::array<float, 6> road_transform{}, road_transform_std{};
    std::array<float, 3> wide_from_device_euler{};
    float desired_curvature = 0, desired_curvature_std = 0;
    std::array<float, 8> desire_state{};
    std::array<std::array<float, 8>, sc::kDesirePredLen> desire_pred{};
    std::array<float, sc::kMetaLen> meta{};  // sigmoid of all 48
    bool fcw = false;                        // openpilot hardBrakePredicted
    bool finite = true;                      // false if any raw output was NaN/inf
};

class Supercombo {
public:
    explicit Supercombo(const SupercomboOptions &opt);
    ~Supercombo();

    // Image inputs: each pointer is one packed frame (sc::kFramePacked floats).
    // Runs the model once and advances the feedback FIFOs. desire: one-hot of the current
    // desire (nullptr = none). Returns false (and sets err) on an ORT error.
    bool run(const float *med_prev, const float *med_cur, const float *big_prev, const float *big_cur,
             const float *desire, std::string *err);

    const std::vector<float> &raw() const { return out_; }
    const RoadOutputs &outputs() const { return parsed_; }
    float v_ego() const { return v_ego_; }       // vEgo fed into the last run
    float steer_delay() const { return opt_.steer_delay; }
    uint64_t runs() const { return runs_; }

    // Direct access to the input tensors (for dumps / tests).
    const std::vector<float> &input(int i) const { return in_[i]; }

    static RoadOutputs parse(const float *raw);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    SupercomboOptions opt_;
    std::array<std::vector<float>, 7> in_;
    std::vector<float> out_;
    std::array<float, 8> prev_desire_{};
    RoadOutputs parsed_;
    float v_ego_ = 0;
    uint64_t runs_ = 0;
    std::array<float, 5> fcw5_{};  // last 5 values of hard-brake-5 at t=2s
    std::array<float, 2> fcw3_{};  // last 2 values of hard-brake-3 at t=2s
};

// One `road` JSON line (docs/ARCHITECTURE.md), without the trailing newline.
struct RoadMeta {
    uint64_t seq = 0, seq_prev = 0;
    int64_t ts_mono_ns = 0, ts_real_ns = 0;
    double model_ms = 0, prep_ms = 0;
    double pair_dt_ms = 0;  // time between the two frames of the pair
    float v_ego = 0, steer_delay = 0;
    uint64_t frames_skipped = 0;  // selected frames the model did not run on (cumulative)
    double thermal_c = -1;        // hottest thermal zone (C), < 0 = unknown (null)
    double effective_hz = 0;      // current run-rate cap (max_hz, halved while thermally throttled)
    bool rate_limited = false;    // true while the thermal guard has halved the rate
};
std::string road_json(const RoadOutputs &o, const RoadMeta &m);

}  // namespace pocopilot
