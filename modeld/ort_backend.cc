// ONNX Runtime backend (CPU EP, fp32 graph) + backend factory.
#include <cstring>
#include <stdexcept>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "backend.h"
#include "supercombo.h"

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

std::unique_ptr<InferenceBackend> make_backend(const SupercomboOptions &opt) {
    if (opt.backend == "ort") return make_ort_backend(opt);
#ifdef ROADSENSE_HAVE_NCNN
    if (opt.backend == "ncnn") return make_ncnn_backend(opt);
#else
    if (opt.backend == "ncnn") throw std::runtime_error("ncnn backend not built in (configure with -DROADSENSE_NCNN=ON)");
#endif
    throw std::runtime_error("unknown backend '" + opt.backend + "' (ort, ncnn)");
}

}  // namespace roadsense
