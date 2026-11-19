#include "supercombo.h"

#include <chrono>
#include <stdexcept>

#include "vision.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <unistd.h>

namespace roadsense {

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

static_assert(vis::kPolicyOutputLen == 1000, "policy_out_ size");

bool is_vision(const SupercomboOptions &o) { return o.road_model == "vision"; }

bool resolve_road_model(SupercomboOptions *o, bool road_model_set, bool model_set, bool policy_set,
                        RoadModelChoice *choice, std::string *err, const std::string &dir) {
    auto readable = [](const std::string &p) { return access(p.c_str(), R_OK) == 0; };
    const bool ncnn = o->backend == "ncnn";
    const std::string vis_model = dir + (ncnn ? "/driving_vision-ncnn.param" : "/driving_vision-fp32.onnx");
    const std::string sc_model = dir + (ncnn ? "/supercombo-ncnn.param" : "/supercombo-fp32.onnx");
    const std::string policy = dir + "/driving_policy-fp32.onnx";
    RoadModelChoice ch;
    if (road_model_set && o->road_model != "vision" && o->road_model != "supercombo") {
        *err = "unknown road model '" + o->road_model + "' (vision, supercombo)";
        return false;
    }
    if (!road_model_set) {
        if (model_set) {
            const size_t sl = o->model_path.find_last_of('/');
            const std::string base = sl == std::string::npos ? o->model_path : o->model_path.substr(sl + 1);
            o->road_model = base.find("driving_vision") != std::string::npos ? "vision" : "supercombo";
            ch.note = "road model " + o->road_model + " (from the model file's name; no --road-model given)";
        } else if (readable(vis_model) && (policy_set || readable(policy))) {
            o->road_model = "vision";
        } else if (readable(sc_model)) {
            o->road_model = "supercombo";
            ch.fallback = true;
            ch.note = "FALLBACK to supercombo: the default road model's files are not installed (" +
                      std::string(readable(vis_model) ? policy : vis_model) + " missing); `road_model_fallback` is set";
        } else {
            *err = "no road model installed: neither " + vis_model + " + " + policy + " nor " + sc_model;
            return false;
        }
    }
    if (o->road_model == "vision") {
        if (!model_set) o->model_path = vis_model;
        if (!policy_set) o->policy_path = policy;
        if (o->policy_path == "none") o->policy_path.clear();
        if (!o->policy_path.empty() && !readable(o->policy_path)) {
            *err = "policy " + o->policy_path + " not readable (the plan needs it; --policy none runs without a plan)";
            return false;
        }
    } else {
        if (!model_set) o->model_path = sc_model;
        o->policy_path.clear();
    }
    if (ch.note.empty())
        ch.note = "road model " + o->road_model +
                  (o->road_model == "vision" ? (o->policy_path.empty() ? ", no policy (no plan in `road`)" : ", policy " + o->policy_path) : "");
    if (choice) *choice = ch;
    return true;
}

double Supercombo::pair_dt_s() const { return vision_ ? vis::kPairDtS : sc::kModelPeriodS; }

Supercombo::Supercombo(const SupercomboOptions &opt) : opt_(opt) {
    if (opt.road_model != "supercombo" && opt.road_model != "vision")
        throw std::runtime_error("unknown road model '" + opt.road_model + "' (supercombo, vision)");
    vision_ = is_vision(opt);
    policy_hz_ = opt.policy_hz;
    if (vision_) {
        in_[0].assign(sc::kImgInput, 0.0f);
        in_[1].assign(sc::kImgInput, 0.0f);
        out_.assign(vis::kOutputLen, 0.0f);
        if (!opt.policy_path.empty())
            policy_ = std::make_unique<VisionPolicy>(opt.policy_path, opt.policy_threads, opt.rhd, opt.allow_spinning);
        backend_ = make_backend(opt);
        return;
    }
    const std::array<std::vector<int64_t>, 7> shapes = {{{1, sc::kImgC, sc::kImgH, sc::kImgW},
                                                          {1, sc::kImgC, sc::kImgH, sc::kImgW},
                                                          {1, sc::kDesireLen, sc::kDesireW},
                                                          {1, sc::kTrafficConventionLen},
                                                          {1, sc::kLatParamsLen},
                                                          {1, sc::kPrevCurvLen, 1},
                                                          {1, sc::kFeatLen, sc::kFeatW}}};
    for (int i = 0; i < 7; ++i) {
        size_t n = 1;
        for (auto d : shapes[i]) n *= size_t(d);
        in_[i].assign(n, 0.0f);  // spec 1.1: all buffers start at zero
    }
    out_.assign(sc::kOutputLen, 0.0f);

    // traffic_convention[int(is_rhd)] = 1 (spec 1.1)
    in_[3][opt.rhd ? 1 : 0] = 1.0f;

    backend_ = make_backend(opt);
}

Supercombo::~Supercombo() = default;

bool Supercombo::run(const float *med_prev, const float *med_cur, const float *big_prev, const float *big_cur,
                     const float *desire, std::string *err, int64_t ts_ns) {
    // images: [prev | cur] (spec 2.3)
    std::memcpy(in_[0].data(), med_prev, sizeof(float) * sc::kFramePacked);
    std::memcpy(in_[0].data() + sc::kFramePacked, med_cur, sizeof(float) * sc::kFramePacked);
    std::memcpy(in_[1].data(), big_prev, sizeof(float) * sc::kFramePacked);
    std::memcpy(in_[1].data() + sc::kFramePacked, big_cur, sizeof(float) * sc::kFramePacked);
    prepare_aux(desire);
    std::array<const float *, 7> in;
    for (int i = 0; i < 7; ++i) in[i] = in_[i].data();
    if (!backend_->run(in, out_.data(), err)) return false;
    finish(ts_ns);
    return true;
}

bool Supercombo::run_raw(const float *raw, int64_t ts_ns, std::string *err) {
    if (!vision_) {
        if (err) *err = "run_raw: vision model only";
        return false;
    }
    std::memcpy(out_.data(), raw, sizeof(float) * vis::kOutputLen);
    finish(ts_ns);
    return true;
}

bool Supercombo::can_prefetch() const { return backend_->can_split(); }

std::unique_ptr<Supercombo::Prefetch> Supercombo::prefetch(const float *med_prev, const float *med_cur,
                                                           const float *big_prev, const float *big_cur,
                                                           std::string *err) {
    auto p = std::make_unique<Prefetch>();
    p->img0.resize(in_[0].size());
    p->img1.resize(in_[1].size());
    std::memcpy(p->img0.data(), med_prev, sizeof(float) * sc::kFramePacked);
    std::memcpy(p->img0.data() + sc::kFramePacked, med_cur, sizeof(float) * sc::kFramePacked);
    std::memcpy(p->img1.data(), big_prev, sizeof(float) * sc::kFramePacked);
    std::memcpy(p->img1.data() + sc::kFramePacked, big_cur, sizeof(float) * sc::kFramePacked);
    p->job = backend_->record_stage1(p->img0.data(), p->img1.data(), err);
    if (!p->job) return nullptr;
    return p;
}

bool Supercombo::submit(Prefetch &p, std::string *err) { return backend_->submit_stage1(p.job.get(), err); }

bool Supercombo::run(Prefetch &p, const float *desire, std::string *err, int64_t ts_ns) {
    prepare_aux(desire);
    std::array<const float *, 7> in;
    in[0] = p.img0.data();
    in[1] = p.img1.data();
    for (int i = 2; i < 7; ++i) in[i] = in_[i].data();
    if (!backend_->run_stage2(p.job.get(), in, out_.data(), err)) return false;
    finish(ts_ns);
    return true;
}

void Supercombo::prepare_aux(const float *desire) {
    if (vision_) return;  // no auxiliary inputs (v_ego: finish())
    // desire: rising-edge pulse, index 0 forced 0, FIFO shift, newest at row 99 (spec 1.1)
    std::array<float, 8> d{};
    if (desire) std::copy(desire, desire + 8, d.begin());
    d[0] = 0;
    float *dz = in_[2].data();
    std::memmove(dz, dz + sc::kDesireW, sizeof(float) * (sc::kDesireLen - 1) * sc::kDesireW);
    for (int i = 0; i < 8; ++i)
        dz[(sc::kDesireLen - 1) * sc::kDesireW + i] = (d[i] - prev_desire_[i] > 0.99f) ? d[i] : 0.0f;
    prev_desire_ = d;

    // lateral_control_params = [vEgo, steer delay]; vEgo = the model's own pose vx of the previous run
    // (no car speed available), clamped to a sane range.
    v_ego_ = runs_ ? std::clamp(parsed_.pose[0], 0.0f, 70.0f) : 0.0f;
    if (!std::isfinite(v_ego_)) v_ego_ = 0;
    in_[4][0] = v_ego_;
    in_[4][1] = opt_.steer_delay;
}

void Supercombo::finish(int64_t ts_ns) {
    ++runs_;
    if (ts_ns < 0) ts_ns = last_ts_ + int64_t(sc::kModelPeriodS * 1e9);
    last_ts_ = ts_ns;
    if (vision_) {
        parsed_ = parse_vision(out_.data());
        // v_ego (published only, nothing is fed back): the pose of this very run. The vision model's speed comes
        // from the same image pair as its lead speed, and it is 2-4 times steadier than supercombo's (L27).
        v_ego_ = std::clamp(parsed_.pose[0], 0.0f, 70.0f);
        if (!std::isfinite(v_ego_)) v_ego_ = 0;
        policy_ms_ = 0;
        if (policy_) {
            // the history gets every run's features; the policy itself may run less often (policy_hz_)
            const bool due = policy_hz_ <= 0 || policy_ts_ < 0 || ts_ns < policy_ts_ ||
                             double(ts_ns - policy_ts_) >= 1e9 / policy_hz_ - 25e6;
            std::string perr;
            const auto t0 = std::chrono::steady_clock::now();
            if (due) {
                if (policy_->run(out_.data() + vis::kHiddenState, ts_ns, policy_out_.data(), &perr)) {
                    policy_ts_ = ts_ns;
                    policy_valid_ = true;
                } else {
                    policy_valid_ = false;
                    fprintf(stderr, "modeld: %s\n", perr.c_str());
                }
            } else {
                policy_->push(out_.data() + vis::kHiddenState, ts_ns);
            }
            policy_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            // A held plan is never older than 1 s: after that there is no plan rather than a stale one.
            if (policy_valid_ && ts_ns - policy_ts_ > 1'000'000'000) policy_valid_ = false;
            if (policy_valid_) {
                const auto &po = policy_out_;
                bool fin = true;
                for (float v : po) fin = fin && std::isfinite(v);
                for (int t = 0; t < sc::kIdxN; ++t)
                    for (int c = 0; c < sc::kPlanWidth; ++c) {
                        parsed_.plan[t][c] = po[vis::kPlanMean + t * vis::kPlanWidth + c];
                        parsed_.plan_std[t][c] = std::exp(po[vis::kPlanStd + t * vis::kPlanWidth + c]);
                    }
                softmax(po.data() + vis::kDesireState, parsed_.desire_state);
                parsed_.has_plan = parsed_.has_desire_state = true;
                parsed_.plan_hyp = 0;
                parsed_.plan_weight = NAN;
                parsed_.plan_held = !due;
                parsed_.plan_age_ms = double(ts_ns - policy_ts_) * 1e-6;
                parsed_.finite = parsed_.finite && fin;
            }
        }
        fcw();
        return;
    }
    parsed_ = parse(out_.data());
    fcw();
    feedback();
}

void Supercombo::fcw() {
    // FCW (fill_model_msg.py): hard-brake-5 > [.05,.05,.15,.15,.15] for the last 5 runs and
    // hard-brake-3 > [.7,.7] for the last 2, using the t = 2 s entries.
    std::rotate(fcw5_.begin(), fcw5_.begin() + 1, fcw5_.end());
    fcw5_.back() = parsed_.meta[sc::kMetaDisengageBase + 5];
    std::rotate(fcw3_.begin(), fcw3_.begin() + 1, fcw3_.end());
    fcw3_.back() = parsed_.meta[sc::kMetaDisengageBase + 3];
    static const float t5[5] = {.05f, .05f, .15f, .15f, .15f};
    bool f = fcw3_[0] > .7f && fcw3_[1] > .7f;
    for (int i = 0; i < 5; ++i) f = f && fcw5_[i] > t5[i];
    parsed_.fcw = f;
}

void Supercombo::feedback() {
    // feedback FIFOs after the run (spec 4)
    float *fb = in_[6].data();
    std::memmove(fb, fb + sc::kFeatW, sizeof(float) * (sc::kFeatLen - 1) * sc::kFeatW);
    std::memcpy(fb + (sc::kFeatLen - 1) * sc::kFeatW, out_.data() + sc::kHiddenState, sizeof(float) * sc::kHiddenLen);
    float *pc = in_[5].data();
    std::memmove(pc, pc + 1, sizeof(float) * (sc::kPrevCurvLen - 1));
    pc[sc::kPrevCurvLen - 1] = out_[sc::kDesiredCurvMean];
}

RoadOutputs Supercombo::parse(const float *r) {
    RoadOutputs o;
    for (int i = 0; i < sc::kOutputLen; ++i)
        if (!std::isfinite(r[i])) {
            o.finite = false;
            break;
        }

    // plan: hypothesis with the largest weight logit (softmax is monotonic), spec 3.2
    int best = 0;
    for (int h = 1; h < sc::kPlanMhpN; ++h)
        if (r[sc::kPlanHypLen * h + sc::kPlanWeightOff] > r[sc::kPlanHypLen * best + sc::kPlanWeightOff]) best = h;
    {
        float mx = r[sc::kPlanHypLen * best + sc::kPlanWeightOff], s = 0;
        for (int h = 0; h < sc::kPlanMhpN; ++h) s += std::exp(r[sc::kPlanHypLen * h + sc::kPlanWeightOff] - mx);
        o.plan_weight = 1.0f / s;
    }
    o.plan_hyp = best;
    const float *pm = r + sc::kPlanStart + sc::kPlanHypLen * best;
    for (int t = 0; t < sc::kIdxN; ++t)
        for (int c = 0; c < sc::kPlanWidth; ++c) {
            o.plan[t][c] = pm[t * sc::kPlanWidth + c];
            o.plan_std[t][c] = std::exp(pm[sc::kPlanStdOff + t * sc::kPlanWidth + c]);
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
    lines(sc::kLaneLinesMean, sc::kLaneLinesStd, sc::kNumLaneLines, o.lane_lines.data());
    lines(sc::kRoadEdgesMean, sc::kRoadEdgesStd, sc::kNumRoadEdges, o.road_edges.data());
    for (int l = 0; l < sc::kNumLaneLines; ++l) o.lane_lines[l].prob = sigmoid(r[sc::kLaneLinesProb + 2 * l + 1]);

    // leads: for output i, the hypothesis with the larger logit in weight column i
    for (int i = 0; i < sc::kLeadSelection; ++i) {
        int bh = 0;
        for (int h = 1; h < sc::kLeadMhpN; ++h)
            if (r[sc::kLeadStart + sc::kLeadHypLen * h + sc::kLeadWeightOff + i] >
                r[sc::kLeadStart + sc::kLeadHypLen * bh + sc::kLeadWeightOff + i])
                bh = h;
        const float *lm = r + sc::kLeadStart + sc::kLeadHypLen * bh;
        auto &L = o.leads[i];
        L.hyp = bh;
        L.prob = sigmoid(r[sc::kLeadProb + i]);
        for (int t = 0; t < sc::kLeadTrajLen; ++t) {
            L.x[t] = lm[t * 4 + 0];
            L.y[t] = lm[t * 4 + 1];
            L.v[t] = lm[t * 4 + 2];
            L.a[t] = lm[t * 4 + 3];
            L.x_std[t] = std::exp(lm[sc::kLeadStdOff + t * 4 + 0]);
            L.y_std[t] = std::exp(lm[sc::kLeadStdOff + t * 4 + 1]);
            L.v_std[t] = std::exp(lm[sc::kLeadStdOff + t * 4 + 2]);
            L.a_std[t] = std::exp(lm[sc::kLeadStdOff + t * 4 + 3]);
        }
    }

    for (int i = 0; i < 6; ++i) {
        o.pose[i] = r[sc::kPoseMean + i];
        o.pose_std[i] = std::exp(r[sc::kPoseStd + i]);
        o.sim_pose[i] = r[sc::kSimPoseMean + i];
        o.sim_pose_std[i] = std::exp(r[sc::kSimPoseStd + i]);
        o.road_transform[i] = r[sc::kRoadTransformMean + i];
        o.road_transform_std[i] = std::exp(r[sc::kRoadTransformStd + i]);
    }
    for (int i = 0; i < 3; ++i) o.wide_from_device_euler[i] = r[sc::kWideFromDeviceEulerMean + i];
    o.desired_curvature = r[sc::kDesiredCurvMean];
    o.desired_curvature_std = std::exp(r[sc::kDesiredCurvStd]);
    softmax(r + sc::kDesireState, o.desire_state);
    for (int k = 0; k < sc::kDesirePredLen; ++k) softmax(r + sc::kDesirePred + 8 * k, o.desire_pred[k]);
    for (int i = 0; i < sc::kMetaLen; ++i) o.meta[i] = sigmoid(r[sc::kMeta + i]);
    return o;
}

// ------------------------------------------------------------------ JSON ---

namespace {
struct J {
    std::string s;
    void num(double v) {
        if (!std::isfinite(v)) {
            s += "null";
            return;
        }
        char b[32];
        int n = std::snprintf(b, sizeof b, "%.5g", v);
        s.append(b, size_t(n));
    }
    void key(const char *k) {
        s += '"';
        s += k;
        s += "\":";
    }
    template <class It>
    void arr(It b, It e) {
        s += '[';
        for (It i = b; i != e; ++i) {
            if (i != b) s += ',';
            num(*i);
        }
        s += ']';
    }
    template <class C>
    void arr(const C &c) {
        arr(std::begin(c), std::end(c));
    }
    template <class F>
    void arrf(int n, F f) {
        s += '[';
        for (int i = 0; i < n; ++i) {
            if (i) s += ',';
            num(f(i));
        }
        s += ']';
    }
};
}  // namespace

std::string road_json(const RoadOutputs &o, const RoadMeta &m) {
    J j;
    j.s.reserve(24000);
    char hdr[256];
    std::snprintf(hdr, sizeof hdr, "{\"type\":\"road\",\"seq\":%llu,\"ts_mono_ns\":%lld,\"ts_real_ns\":%lld,",
                  (unsigned long long)m.seq, (long long)m.ts_mono_ns, (long long)m.ts_real_ns);
    j.s += hdr;
    j.key("model_ms"), j.num(m.model_ms), j.s += ',';
    j.key("prep_ms"), j.num(m.prep_ms), j.s += ',';
    j.key("frame_seq_prev"), j.s += std::to_string(m.seq_prev), j.s += ',';
    j.key("pair_dt_ms"), j.num(m.pair_dt_ms), j.s += ',';
    j.key("frames_skipped"), j.s += std::to_string(m.frames_skipped), j.s += ',';
    j.key("v_ego"), j.num(m.v_ego), j.s += ',';
    j.key("effective_hz"), j.num(m.effective_hz), j.s += ',';
    j.key("rate_limited"), j.s += m.rate_limited ? "true" : "false", j.s += ',';
    j.key("rate_hz"), j.num(m.rate_hz), j.s += ',';
    j.key("governor_level"), j.num(m.governor_level >= 0 ? double(m.governor_level) : NAN), j.s += ',';
    j.key("limited"), j.s += m.limited ? "true" : "false", j.s += ',';
    j.key("steer_delay"), j.num(m.steer_delay), j.s += ',';
    j.key("valid"), j.s += o.finite ? "true" : "false", j.s += ',';
    j.key("road_model"), j.s += '"', j.s += o.model, j.s += "\",";
    j.key("road_model_fallback"), j.s += m.road_model_fallback ? "true" : "false", j.s += ',';
    j.key("policy_hz"), j.num(m.policy_hz >= 0 ? m.policy_hz : NAN), j.s += ',';
    j.key("policy_ms"), j.num(m.policy_hz >= 0 ? m.policy_ms : NAN), j.s += ',';

    // plan (absent, not null, when the model has none: consumers test road.get("plan"))
    if (o.has_plan) {
    static const char *pk[15] = {"x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az",
                                 "roll", "pitch", "yaw", "roll_rate", "pitch_rate", "yaw_rate"};
    j.key("plan"), j.s += '{';
    j.key("t"), j.arr(sc::T_IDXS);
    for (int c = 0; c < 15; ++c) {
        j.s += ',';
        j.key(pk[c]);
        j.arrf(sc::kIdxN, [&](int t) { return o.plan[t][c]; });
    }
    j.s += ',', j.key("x_std"), j.arrf(sc::kIdxN, [&](int t) { return o.plan_std[t][0]; });
    j.s += ',', j.key("y_std"), j.arrf(sc::kIdxN, [&](int t) { return o.plan_std[t][1]; });
    j.s += ',', j.key("z_std"), j.arrf(sc::kIdxN, [&](int t) { return o.plan_std[t][2]; });
    j.s += ',', j.key("prob"), j.num(o.plan_mhp ? o.plan_weight : NAN);
    // held: the plan of an earlier run, published again (policy_hz); age_ms: how much earlier (camera time)
    j.s += ',', j.key("held"), j.s += o.plan_held ? "true" : "false";
    j.s += ',', j.key("age_ms"), j.num(o.plan_age_ms);
    j.s += "},";
    }

    auto line = [&](const RoadOutputs::Line &l, bool prob) {
        j.s += '{';
        j.key("x"), j.arr(sc::X_IDXS), j.s += ',';
        j.key("y"), j.arr(l.y), j.s += ',';
        j.key("z"), j.arr(l.z), j.s += ',';
        j.key("y_std"), j.arr(l.y_std);
        if (prob) j.s += ',', j.key("prob"), j.num(l.prob);
        j.s += '}';
    };
    j.key("lane_lines"), j.s += '[';
    for (int i = 0; i < sc::kNumLaneLines; ++i) {
        if (i) j.s += ',';
        line(o.lane_lines[i], true);
    }
    j.s += "],";
    j.key("road_edges"), j.s += '[';
    for (int i = 0; i < sc::kNumRoadEdges; ++i) {
        if (i) j.s += ',';
        line(o.road_edges[i], false);
    }
    j.s += "],";

    j.key("leads"), j.s += '[';
    for (int i = 0; i < sc::kLeadSelection; ++i) {
        const auto &L = o.leads[i];
        if (i) j.s += ',';
        j.s += '{';
        j.key("prob"), j.num(L.prob), j.s += ',';
        j.key("t_offset"), j.num(sc::LEAD_T_OFFSETS[i]), j.s += ',';
        j.key("t"), j.arr(sc::LEAD_T_IDXS), j.s += ',';
        j.key("x"), j.arr(L.x), j.s += ',';
        j.key("y"), j.arr(L.y), j.s += ',';
        j.key("v"), j.arr(L.v), j.s += ',';
        j.key("a"), j.arr(L.a), j.s += ',';
        j.key("x_std"), j.arr(L.x_std), j.s += ',';
        j.key("y_std"), j.arr(L.y_std), j.s += ',';
        j.key("v_std"), j.arr(L.v_std), j.s += ',';
        j.key("a_std"), j.arr(L.a_std);
        j.s += '}';
    }
    j.s += "],";

    static const char *posek[6] = {"vx", "vy", "vz", "roll_rate", "pitch_rate", "yaw_rate"};
    j.key("pose"), j.s += '{';
    for (int i = 0; i < 6; ++i) j.key(posek[i]), j.num(o.pose[i]), j.s += ',';
    for (int i = 0; i < 6; ++i) {
        std::string k = std::string(posek[i]) + "_std";
        j.key(k.c_str()), j.num(o.pose_std[i]);
        if (i < 5) j.s += ',';
    }
    j.s += "},";
    j.key("road_transform"), j.arr(o.road_transform), j.s += ',';
    j.key("road_transform_std"), j.arr(o.road_transform_std), j.s += ',';
    j.key("desired_curvature"), j.num(o.has_curvature ? o.desired_curvature : NAN), j.s += ',';
    j.key("desired_curvature_std"), j.num(o.has_curvature ? o.desired_curvature_std : NAN), j.s += ',';

    if (o.has_desire_state) j.key("desire_state"), j.arr(o.desire_state), j.s += ',';
    else j.key("desire_state"), j.s += "null,";
    j.key("desire_pred"), j.s += '[';
    for (int k = 0; k < sc::kDesirePredLen; ++k) {
        if (k) j.s += ',';
        j.arr(o.desire_pred[k]);
    }
    j.s += "],";

    auto meta_slice = [&](int off) {
        j.arrf(sc::kMetaDisengageN, [&](int k) { return o.meta[sc::kMetaDisengageBase + sc::kMetaDisengageStride * k + off]; });
    };
    j.key("meta"), j.s += '{';
    j.key("engaged_prob"), j.num(o.meta[sc::kMetaEngaged]), j.s += ',';
    j.key("disengage_probs"), j.s += '{';
    static const char *dk[7] = {"gas", "brake", "steer_override", "brake_3ms2", "brake_4ms2", "brake_5ms2", "gas_press"};
    for (int i = 0; i < 7; ++i) {
        if (i) j.s += ',';
        j.key(dk[i]), meta_slice(i);
    }
    j.s += "},";
    j.key("blinkers"), j.s += '{';
    j.key("left"), j.arrf(sc::kMetaBlinkerN, [&](int k) { return o.meta[sc::kMetaBlinkerBase + 2 * k]; }), j.s += ',';
    j.key("right"), j.arrf(sc::kMetaBlinkerN, [&](int k) { return o.meta[sc::kMetaBlinkerBase + 2 * k + 1]; });
    j.s += "},";
    j.key("hard_brake_pred"), meta_slice(5), j.s += ',';
    j.key("fcw"), j.s += o.fcw ? "true" : "false";
    j.s += "}}";
    return j.s;
}

}  // namespace roadsense
