#include "vision.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>

#include "supercombo.h"

namespace roadsense {

// ------------------------------------------------------------------ policy ---

struct VisionPolicy::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "modeld-policy"};
    Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::unique_ptr<Ort::Session> session;
};

VisionPolicy::VisionPolicy(const std::string &path, int threads, bool rhd, bool allow_spinning)
    : impl_(std::make_unique<Impl>()) {
    Ort::SessionOptions so;
    so.SetIntraOpNumThreads(threads > 0 ? threads : 1);
    so.SetInterOpNumThreads(1);
    so.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    if (!allow_spinning) so.AddConfigEntry("session.intra_op.allow_spinning", "0");
    impl_->session = std::make_unique<Ort::Session>(impl_->env, path.c_str(), so);
    Ort::AllocatorWithDefaultOptions alloc;
    if (impl_->session->GetInputCount() != 3 || impl_->session->GetOutputCount() != 1)
        throw std::runtime_error("policy: expected 3 inputs and 1 output (v0.11.1 driving_policy)");
    for (size_t i = 0; i < 3; ++i) {
        auto name = impl_->session->GetInputNameAllocated(i, alloc);
        bool found = false;
        for (auto *n : vis::kPolicyInputNames) found |= std::strcmp(n, name.get()) == 0;
        if (!found) throw std::runtime_error(std::string("policy: unexpected input ") + name.get());
        Ort::TypeInfo info = impl_->session->GetInputTypeInfo(i);
        if (info.GetTensorTypeAndShapeInfo().GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
            throw std::runtime_error("policy: inputs must be float32 (use driving_policy-fp32.onnx)");
    }
    feat_.assign(size_t(vis::kPolicyHist) * vis::kHiddenLen, 0.0f);
    desire_.assign(size_t(vis::kPolicyHist) * 8, 0.0f);
    traffic_.assign(2, 0.0f);
    traffic_[rhd ? 1 : 0] = 1.0f;
}

VisionPolicy::~VisionPolicy() = default;

bool VisionPolicy::run(const float *hidden, int64_t ts, float *out, std::string *err) {
    push(hidden, ts);
    return infer(ts, out, err);
}

void VisionPolicy::push(const float *hidden, int64_t ts) {
    if (!hist_.empty() && (ts <= hist_.back().first || ts - hist_.back().first > 1'000'000'000))
        hist_.clear();  // stream restart or gap: never mix histories
    std::array<float, vis::kHiddenLen> h;
    std::memcpy(h.data(), hidden, sizeof(float) * vis::kHiddenLen);
    hist_.emplace_back(ts, h);
    const int64_t span = int64_t((vis::kPolicyHist - 1) * vis::kPolicyHistDtS * 1e9) + 100'000'000;
    while (!hist_.empty() && ts - hist_.front().first > span) hist_.pop_front();
}

bool VisionPolicy::infer(int64_t ts, float *out, std::string *err) {
    // row 24 = now, row 24-k = t - 0.2 k s; nearest stored run within 60 ms
    rows_filled_ = 0;
    size_t j = hist_.size();  // search backwards, targets move back monotonically
    for (int k = 0; k < vis::kPolicyHist; ++k) {
        const int64_t target = ts - int64_t(k * vis::kPolicyHistDtS * 1e9 + 0.5);
        float *row = feat_.data() + size_t(vis::kPolicyHist - 1 - k) * vis::kHiddenLen;
        int64_t best = INT64_MAX;
        size_t bi = 0;
        for (size_t i = j; i-- > 0;) {
            const int64_t d = std::llabs(hist_[i].first - target);
            if (d < best) best = d, bi = i;
            if (hist_[i].first < target) break;
        }
        // nearest within 50 ms (runs 50-100 ms apart at 13-20 runs/s); never before the oldest kept run (start-up:
        // zeros like openpilot's queue, not the first run repeated)
        if (best <= 50'000'000 && target >= hist_.front().first - 25'000'000) {
            std::memcpy(row, hist_[bi].second.data(), sizeof(float) * vis::kHiddenLen);
            ++rows_filled_;
            j = bi + 1;
        } else {
            std::memset(row, 0, sizeof(float) * vis::kHiddenLen);
        }
    }
    try {
        const int64_t s_des[3] = {1, vis::kPolicyHist, 8}, s_tc[2] = {1, 2}, s_feat[3] = {1, vis::kPolicyHist, vis::kHiddenLen};
        std::array<Ort::Value, 3> in = {
            Ort::Value::CreateTensor<float>(impl_->mem, desire_.data(), desire_.size(), s_des, 3),
            Ort::Value::CreateTensor<float>(impl_->mem, traffic_.data(), traffic_.size(), s_tc, 2),
            Ort::Value::CreateTensor<float>(impl_->mem, feat_.data(), feat_.size(), s_feat, 3)};
        const int64_t os[2] = {1, vis::kPolicyOutputLen};
        Ort::Value o = Ort::Value::CreateTensor<float>(impl_->mem, out, vis::kPolicyOutputLen, os, 2);
        const char *on = vis::kOutputName;
        impl_->session->Run(Ort::RunOptions{nullptr}, vis::kPolicyInputNames, in.data(), 3, &on, &o, 1);
    } catch (const Ort::Exception &e) {
        if (err) *err = std::string("policy: ") + e.what();
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ parser ---

namespace {
inline float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }
template <size_t N>
void softmax(const float *in, std::array<float, N> &out) {
    float mx = in[0];
    for (size_t i = 1; i < N; ++i) mx = std::max(mx, in[i]);
    float s = 0;
    for (size_t i = 0; i < N; ++i) s += (out[i] = std::exp(in[i] - mx));
    for (size_t i = 0; i < N; ++i) out[i] /= s;
}
}  // namespace

// The v0.11.1 vision outputs in the RoadOutputs of supercombo (same frames, grids and units). Not produced:
// plan, desire_state, desired_curvature, sim_pose (left zero, flagged in has_*). meta is rearranged into the
// v0.9.7 layout (supercombo_layout.h) so the JSON and the FCW rule stay the same: gas press for t = 2..10 s
// (the model also gives t = 0 and brake press, not published), blinkers for t = 0..10 s.
RoadOutputs Supercombo::parse_vision(const float *r) {
    RoadOutputs o;
    o.model = "vision";
    o.has_plan = o.has_curvature = o.has_desire_state = false;
    o.plan_mhp = false;
    for (int i = 0; i < vis::kOutputLen; ++i)
        if (!std::isfinite(r[i])) {
            o.finite = false;
            break;
        }
    auto lines = [&](int mean, int stdv, int n, RoadOutputs::Line *out) {
        for (int l = 0; l < n; ++l)
            for (int x = 0; x < sc::kIdxN; ++x) {
                const int k = l * sc::kLaneLineStride + x * 2;
                out[l].y[x] = r[mean + k];
                out[l].z[x] = r[mean + k + 1];
                out[l].y_std[x] = std::exp(r[stdv + k]);
                out[l].z_std[x] = std::exp(r[stdv + k + 1]);
            }
    };
    lines(vis::kLaneLinesMean, vis::kLaneLinesStd, sc::kNumLaneLines, o.lane_lines.data());
    lines(vis::kRoadEdgesMean, vis::kRoadEdgesStd, sc::kNumRoadEdges, o.road_edges.data());
    for (int l = 0; l < sc::kNumLaneLines; ++l) o.lane_lines[l].prob = sigmoid(r[vis::kLaneLinesProb + 2 * l + 1]);
    for (int i = 0; i < sc::kLeadSelection; ++i) {
        auto &L = o.leads[i];
        L.hyp = 0;
        L.prob = sigmoid(r[vis::kLeadProb + i]);
        const float *lm = r + vis::kLeadMean + i * sc::kLeadTrajLen * sc::kLeadWidth;
        const float *ls = r + vis::kLeadStd + i * sc::kLeadTrajLen * sc::kLeadWidth;
        for (int t = 0; t < sc::kLeadTrajLen; ++t) {
            L.x[t] = lm[t * 4 + 0];
            L.y[t] = lm[t * 4 + 1];
            L.v[t] = lm[t * 4 + 2];
            L.a[t] = lm[t * 4 + 3];
            L.x_std[t] = std::exp(ls[t * 4 + 0]);
            L.y_std[t] = std::exp(ls[t * 4 + 1]);
            L.v_std[t] = std::exp(ls[t * 4 + 2]);
            L.a_std[t] = std::exp(ls[t * 4 + 3]);
        }
    }
    for (int i = 0; i < 6; ++i) {
        o.pose[i] = r[vis::kPoseMean + i];
        o.pose_std[i] = std::exp(r[vis::kPoseStd + i]);
        o.road_transform[i] = r[vis::kRoadTransformMean + i];
        o.road_transform_std[i] = std::exp(r[vis::kRoadTransformStd + i]);
    }
    for (int i = 0; i < 3; ++i) o.wide_from_device_euler[i] = r[vis::kWideFromDeviceEulerMean + i];
    for (int k = 0; k < sc::kDesirePredLen; ++k) softmax(r + vis::kDesirePred + 8 * k, o.desire_pred[k]);
    const float *m = r + vis::kMeta;
    o.meta[sc::kMetaEngaged] = sigmoid(m[0]);
    for (int k = 0; k < sc::kMetaDisengageN; ++k) {
        for (int f = 0; f < 6; ++f)
            o.meta[sc::kMetaDisengageBase + sc::kMetaDisengageStride * k + f] =
                sigmoid(m[vis::kMetaDisengageBase + vis::kMetaDisengageStride * k + f]);
        o.meta[sc::kMetaDisengageBase + sc::kMetaDisengageStride * k + 6] =
            sigmoid(m[vis::kMetaPressBase + vis::kMetaPressStride * (k + 1)]);  // gas press, t = 2 + 2k s
    }
    for (int j = 0; j < sc::kMetaBlinkerN; ++j) {
        o.meta[sc::kMetaBlinkerBase + 2 * j] = sigmoid(m[vis::kMetaPressBase + vis::kMetaPressStride * j + 2]);
        o.meta[sc::kMetaBlinkerBase + 2 * j + 1] = sigmoid(m[vis::kMetaPressBase + vis::kMetaPressStride * j + 3]);
    }
    return o;
}

}  // namespace roadsense
