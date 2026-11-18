// Road model runner (inference backend from backend.h: ONNX Runtime CPU or ncnn Vulkan) + output parser.
//
//   road_model "supercombo": openpilot v0.9.7 supercombo (recurrent, image pair 50 ms apart). Default.
//   road_model "vision":     openpilot v0.11.1 driving_vision (stateless, image pair 200 ms apart, uint8
//                            inputs, nearest warp; vision_layout.h), optionally followed by driving_policy
//                            on ONNX Runtime (CPU) for the plan (policy_path; empty = no plan: the `road`
//                            message then has no `plan` key, docs/lanes/L24-M-vision-model.md).
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

#include "backend.h"
#include "supercombo_layout.h"

namespace roadsense {

struct SupercomboOptions {
    std::string road_model = "supercombo";  // "supercombo" (v0.9.7) or "vision" (v0.11.1 driving_vision)
    std::string policy_path;                // vision: driving_policy-fp32.onnx (ORT CPU); empty = no plan
    int policy_threads = 1;                 // vision: ORT intra-op threads of the policy
    double policy_hz = 0;                   // vision: run the policy at most this often (0 = after every vision
                                            // run); between policy runs the last plan is published again
    std::string backend = "ort";  // "ort" (ONNX Runtime CPU) or "ncnn" (Vulkan), see backend.h
    std::string model_path = "/var/lib/roadsense/models/supercombo-fp32.onnx";  // ncnn: the .param file
    int threads = 4;           // ort: intra-op threads (4 big cores on the SD845); ncnn: CPU threads (1)
    bool ncnn_fp16 = true;     // ncnn: fp16-packed storage (false: fp32 everywhere, ~5x slower on turnip)
    bool ncnn_flushall = true; // ncnn: add TU_DEBUG=flushall (turnip workaround, see ncnn_backend.h)
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
    // What the model produced (vision: no desired_curvature; plan and desire_state only with the policy).
    bool has_plan = true, has_curvature = true, has_desire_state = true;
    bool plan_mhp = true;                    // false: single-hypothesis plan (policy), `plan.prob` is null
    const char *model = "supercombo";        // `road_model` of the JSON line
};

bool is_vision(const SupercomboOptions &o);
class VisionPolicy;  // vision.cc

class Supercombo {
public:
    explicit Supercombo(const SupercomboOptions &opt);
    ~Supercombo();

    // Image inputs: each pointer is one packed frame (sc::kFramePacked floats).
    // Runs the model once and advances the feedback FIFOs. desire: one-hot of the current
    // desire (nullptr = none). Returns false (and sets err) on a backend error.
    // ts_ns: camera time of the current frame (vision + policy: places the run's features in the 5 Hz
    // history; < 0 = previous + 50 ms).
    bool run(const float *med_prev, const float *med_cur, const float *big_prev, const float *big_cur,
             const float *desire, std::string *err, int64_t ts_ns = -1);
    // vision only: take the 1576 raw outputs of a run made elsewhere (tools/modeld_replay --rates shares one
    // vision run between several simulated run rates), then policy + parse as run() does.
    bool run_raw(const float *raw, int64_t ts_ns, std::string *err);

    // Two-stage run (ncnn, InferenceBackend::can_split): prefetch() packs the image pair and records the
    // vision stage; submit() runs it on the GPU; run(Prefetch) does the rest exactly like run() above.
    // prefetch() and submit() touch no Supercombo state, so they may run on another thread while
    // run() of the previous pair is going.
    struct Prefetch {
        std::vector<float> img0, img1;
        std::unique_ptr<Stage1Job> job;
    };
    bool can_prefetch() const;
    std::unique_ptr<Prefetch> prefetch(const float *med_prev, const float *med_cur, const float *big_prev,
                                       const float *big_cur, std::string *err);
    bool submit(Prefetch &p, std::string *err);
    bool run(Prefetch &p, const float *desire, std::string *err, int64_t ts_ns = -1);

    const std::vector<float> &raw() const { return out_; }
    const RoadOutputs &outputs() const { return parsed_; }
    float v_ego() const { return v_ego_; }       // vEgo fed into the last run
    float steer_delay() const { return opt_.steer_delay; }
    uint64_t runs() const { return runs_; }

    // Direct access to the input tensors (for dumps / tests).
    const std::vector<float> &input(int i) const { return in_[i]; }

    static RoadOutputs parse(const float *raw);
    static RoadOutputs parse_vision(const float *raw);
    const char *backend_name() const { return backend_->name(); }
    bool vision() const { return vision_; }
    double pair_dt_s() const;  // time between the two frames of the pair the model expects (0.05 / 0.2)
    double policy_ms() const { return policy_ms_; }  // last policy run (vision), 0 otherwise

private:
    void prepare_aux(const float *desire);  // desire FIFO, lateral params
    void finish(int64_t ts_ns);             // parse, FCW, feedback FIFOs / policy
    void fcw();
    void feedback();  // supercombo FIFOs

    std::unique_ptr<InferenceBackend> backend_;
    bool vision_ = false;
    std::unique_ptr<VisionPolicy> policy_;
    int64_t last_ts_ = 0, policy_ts_ = -1;
    std::array<float, 1000> policy_out_{};  // last policy output (vis::kPolicyOutputLen)
    bool policy_valid_ = false;
    double policy_ms_ = 0;
    SupercomboOptions opt_;
    std::array<std::vector<float>, 7> in_;  // vision: in_[0], in_[1] only
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

}  // namespace roadsense
