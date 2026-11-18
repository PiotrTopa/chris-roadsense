// ONNX Runtime backend (CPU EP, fp32 graph) + backend factory.
#include <cstring>
#include <stdexcept>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "backend.h"
#include "supercombo.h"
#include "vision_layout.h"

namespace roadsense {

namespace {

class OrtBackend : public InferenceBackend {
public:
    explicit OrtBackend(const SupercomboOptions &opt) {
        Ort::SessionOptions so;
        so.SetIntraOpNumThreads(opt.threads);
        so.SetInterOpNumThreads(1);
        so.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        if (!opt.allow_spinning) so.AddConfigEntry("session.intra_op.allow_spinning", "0");
        session_ = std::make_unique<Ort::Session>(env_, opt.model_path.c_str(), so);
        vision_ = is_vision(opt);
        if (vision_) {
            check_vision();
            return;
        }

        // Check the graph's I/O against the spec (section 1).
        Ort::AllocatorWithDefaultOptions alloc;
        if (session_->GetInputCount() != 7 || session_->GetOutputCount() != 1)
            throw std::runtime_error("model: expected 7 inputs and 1 output (supercombo v0.9.7)");
        for (size_t i = 0; i < 7; ++i) {
            auto name = session_->GetInputNameAllocated(i, alloc);
            bool found = false;
            for (auto *n : sc::kInputNames) found |= std::strcmp(n, name.get()) == 0;
            if (!found) throw std::runtime_error(std::string("model: unexpected input ") + name.get());
            Ort::TypeInfo info = session_->GetInputTypeInfo(i);  // keep alive: the shape info refers into it
            auto ti = info.GetTensorTypeAndShapeInfo();
            if (ti.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
                throw std::runtime_error(std::string("model: input ") + name.get() +
                                         " is not float32; use the fp32-converted model");
        }
    }

    const char *name() const override { return "ort"; }

    bool run(const std::array<const float *, 7> &in, float *out, std::string *err) override {
        if (vision_) return run_vision(in, out, err);
        static const std::array<std::vector<int64_t>, 7> shapes = {{{1, sc::kImgC, sc::kImgH, sc::kImgW},
                                                                     {1, sc::kImgC, sc::kImgH, sc::kImgW},
                                                                     {1, sc::kDesireLen, sc::kDesireW},
                                                                     {1, sc::kTrafficConventionLen},
                                                                     {1, sc::kLatParamsLen},
                                                                     {1, sc::kPrevCurvLen, 1},
                                                                     {1, sc::kFeatLen, sc::kFeatW}}};
        try {
            std::vector<Ort::Value> inputs;
            inputs.reserve(7);
            for (int i = 0; i < 7; ++i) {
                size_t n = 1;
                for (auto d : shapes[i]) n *= size_t(d);
                // ORT never writes an input tensor; the const_cast only satisfies the C API.
                inputs.push_back(Ort::Value::CreateTensor<float>(mem_, const_cast<float *>(in[i]), n,
                                                                 shapes[i].data(), shapes[i].size()));
            }
            const int64_t oshape[2] = {1, sc::kOutputLen};
            Ort::Value output = Ort::Value::CreateTensor<float>(mem_, out, sc::kOutputLen, oshape, 2);
            const char *out_name = sc::kOutputName;
            session_->Run(Ort::RunOptions{nullptr}, sc::kInputNames, inputs.data(), 7, &out_name, &output, 1);
        } catch (const Ort::Exception &e) {
            if (err) *err = e.what();
            return false;
        }
        return true;
    }

private:
    // v0.11.1 driving_vision: inputs img, big_img uint8 [1,12,128,256], output [1,1576] float32 (the
    // fp32-converted graph; the release's fp16-output graph is rejected).
    void check_vision() {
        Ort::AllocatorWithDefaultOptions alloc;
        if (session_->GetInputCount() != 2 || session_->GetOutputCount() != 1)
            throw std::runtime_error("model: expected 2 inputs and 1 output (v0.11.1 driving_vision)");
        for (size_t i = 0; i < 2; ++i) {
            auto name = session_->GetInputNameAllocated(i, alloc);
            if (std::strcmp(name.get(), vis::kInputNames[0]) && std::strcmp(name.get(), vis::kInputNames[1]))
                throw std::runtime_error(std::string("model: unexpected input ") + name.get());
            Ort::TypeInfo info = session_->GetInputTypeInfo(i);
            if (info.GetTensorTypeAndShapeInfo().GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8)
                throw std::runtime_error("model: vision inputs must be uint8");
        }
        Ort::TypeInfo oi = session_->GetOutputTypeInfo(0);
        auto ot = oi.GetTensorTypeAndShapeInfo();
        if (ot.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || ot.GetElementCount() != size_t(vis::kOutputLen))
            throw std::runtime_error("model: vision output must be float32 [1,1576] (use driving_vision-fp32.onnx)");
        u8_[0].resize(sc::kImgInput);
        u8_[1].resize(sc::kImgInput);
    }

    bool run_vision(const std::array<const float *, 7> &in, float *out, std::string *err) {
        try {
            const int64_t shape[4] = {1, sc::kImgC, sc::kImgH, sc::kImgW};
            std::vector<Ort::Value> inputs;
            for (int i = 0; i < 2; ++i) {
                // the packed frames hold whole numbers 0..255
                const float *f = in[i];
                uint8_t *u = u8_[i].data();
                for (size_t k = 0; k < sc::kImgInput; ++k) u[k] = uint8_t(f[k]);
                inputs.push_back(Ort::Value::CreateTensor<uint8_t>(mem_, u, sc::kImgInput, shape, 4));
            }
            const int64_t oshape[2] = {1, vis::kOutputLen};
            Ort::Value output = Ort::Value::CreateTensor<float>(mem_, out, vis::kOutputLen, oshape, 2);
            const char *out_name = vis::kOutputName;
            session_->Run(Ort::RunOptions{nullptr}, vis::kInputNames, inputs.data(), 2, &out_name, &output, 1);
        } catch (const Ort::Exception &e) {
            if (err) *err = e.what();
            return false;
        }
        return true;
    }

    bool vision_ = false;
    std::array<std::vector<uint8_t>, 2> u8_;
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "modeld"};
    Ort::MemoryInfo mem_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::unique_ptr<Ort::Session> session_;
};

}  // namespace

std::unique_ptr<InferenceBackend> make_ort_backend(const SupercomboOptions &opt) {
    return std::make_unique<OrtBackend>(opt);
}

bool backend_available(const std::string &kind) {
#ifdef ROADSENSE_HAVE_NCNN
    if (kind == "ncnn") return true;
#endif
    return kind == "ort";
}

namespace {
// "null": no network; a Supercombo with it only takes outputs computed elsewhere (run_raw, tools/modeld_replay).
class NullBackend : public InferenceBackend {
public:
    const char *name() const override { return "null"; }
    bool run(const std::array<const float *, 7> &, float *, std::string *err) override {
        if (err) *err = "null backend";
        return false;
    }
};
}  // namespace

std::unique_ptr<InferenceBackend> make_backend(const SupercomboOptions &opt) {
    if (opt.backend == "ort") return make_ort_backend(opt);
    if (opt.backend == "null") return std::make_unique<NullBackend>();
#ifdef ROADSENSE_HAVE_NCNN
    if (opt.backend == "ncnn") return make_ncnn_backend(opt);
#else
    if (opt.backend == "ncnn") throw std::runtime_error("ncnn backend not built in (configure with -DROADSENSE_NCNN=ON)");
#endif
    throw std::runtime_error("unknown backend '" + opt.backend + "' (ort, ncnn)");
}

}  // namespace roadsense
