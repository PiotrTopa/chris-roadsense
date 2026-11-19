// Road model runner (inference backend from backend.h: ONNX Runtime CPU or ncnn Vulkan) + output parser.
//
//   road_model "vision":     openpilot v0.11.1 driving_vision (stateless, image pair 200 ms apart, uint8
//                            inputs, nearest warp; vision_layout.h) followed by driving_policy on ONNX
//                            Runtime (CPU) for the plan (policy_path; empty = no plan: the `road` message
//                            then has no `plan` key). The default since lane L27
//                            (docs/lanes/L27-new-model-default.md).
//   road_model "supercombo": openpilot v0.9.7 supercombo (recurrent, image pair 50 ms apart). Selectable
//                            (--road-model supercombo), and what resolve_road_model() falls back to when
//                            no model was named and the vision model's files are not installed.
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
    std::string road_model = "vision";      // "vision" (v0.11.1 driving_vision + policy) or "supercombo" (v0.9.7)
    std::string policy_path;                // vision: driving_policy-fp32.onnx (ORT CPU); empty = no plan
    int policy_threads = 1;                 // vision: ORT intra-op threads of the policy
    double policy_hz = 5;                   // vision: run the policy at most this often (0 = after every vision
                                            // run); between policy runs the last plan is published again, marked
                                            // (`plan.held`, `plan.age_ms`). 5: the policy's own history step is
                                            // 200 ms; measured effect on the plan in docs/lanes/L27
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
    double plan_age_ms = 0;                  // camera time of this run minus that of the run the plan was made on
    bool plan_held = false;                  // the plan is the previous policy run's (policy_hz), not this run's
    const char *model = "supercombo";        // `road_model` of the JSON line
};

bool is_vision(const SupercomboOptions &o);

// Which road model to run and with which files, for modeld and modeld_replay (one rule for both).
//   road_model_set: the model was named (--road-model / config `road_model`): strict, a missing file is an error.
//   Not named, --model given: the file decides (a name containing "driving_vision" is the vision model, anything
//     else supercombo, which is what every command written before lane L27 meant).
//   Neither: the default, vision with the policy, when driving_vision and driving_policy are both installed;
//     otherwise supercombo if its file is, with `fallback` set (reported in every `road` line and in health), so
//     a new binary on a host that only has the old model's files keeps working as before and says so.
//   policy_set: --policy given ("none" = run without a plan). Not given: the default policy file, which must exist.
// Fills o->road_model, o->model_path, o->policy_path. Returns false with *err when nothing can run.
struct RoadModelChoice {
    bool fallback = false;   // the default model's files were missing; the other model runs
    std::string note;        // one line for the log
};
bool resolve_road_model(SupercomboOptions *o, bool road_model_set, bool model_set, bool policy_set,
                        RoadModelChoice *choice, std::string *err,
                        const std::string &models_dir = "/var/lib/roadsense/models");
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
    bool has_policy() const { return policy_ != nullptr; }
    // The policy's rate cap at run time (the load governor's `policy_hz`): <= 0 = after every vision run.
    void set_policy_hz(double hz) { policy_hz_ = hz; }
    double policy_hz() const { return policy_hz_; }

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
    double policy_hz_ = 0;
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
    double effective_hz = 0;      // current run-rate cap (max_hz, lowered by the load governor's road_hz)
    bool rate_limited = false;    // true while the load governor holds the cap below max_hz
    double rate_hz = 0;           // measured runs a second over the last 2 s
    int governor_level = -1;      // the load governor's shed level (0 = nothing shed), -1 = no governor
    bool limited = false;         // the platform limiter is cutting clocks (as the governor's probe sees it)
    bool road_model_fallback = false;  // resolve_road_model(): not the default model, its files were missing
    double policy_hz = -1;        // vision + policy: the policy's rate cap now (0 = every run); < 0 = no policy
    double policy_ms = 0;         // CPU time of the policy step of this run (history push only when the plan is held)
};
std::string road_json(const RoadOutputs &o, const RoadMeta &m);

}  // namespace roadsense
