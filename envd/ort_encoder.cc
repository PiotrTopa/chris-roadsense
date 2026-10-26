#include <onnxruntime_cxx_api.h>

#include <stdexcept>

#include "encoder.h"

namespace roadsense {
namespace envd {

namespace {

std::string shape_str(const std::vector<int64_t> &s) {
    std::string r = "[";
    for (size_t i = 0; i < s.size(); ++i) r += (i ? "," : "") + std::to_string(s[i]);
    return r + "]";
}

class OrtEncoder : public ImageEncoder {
public:
    explicit OrtEncoder(const EncoderOptions &opt)
        : in_shape_{1, 3, opt.size, opt.size}, out_shape_{1, opt.embed_dim}, out_n_(size_t(opt.embed_dim)) {
        Ort::SessionOptions so;
        so.SetIntraOpNumThreads(opt.threads);
        so.SetInterOpNumThreads(1);
        so.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        if (!opt.allow_spinning) so.AddConfigEntry("session.intra_op.allow_spinning", "0");
        session_ = std::make_unique<Ort::Session>(env_, opt.model.c_str(), so);
        Ort::AllocatorWithDefaultOptions alloc;
        in_name_ = session_->GetInputNameAllocated(0, alloc).get();
        out_name_ = session_->GetOutputNameAllocated(0, alloc).get();
        const auto gi = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        const auto go = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        if (gi != in_shape_)
            throw std::runtime_error(opt.model + ": input is " + shape_str(gi) + ", expected " + shape_str(in_shape_));
        size_t n = 1;
        for (int64_t d : go) n *= size_t(d > 0 ? d : 1);
        if (n != out_n_)
            throw std::runtime_error(opt.model + ": output is " + shape_str(go) + ", expected " + shape_str(out_shape_));
        in_n_ = size_t(3) * opt.size * opt.size;
    }
    const char *name() const override { return "ort"; }
    void encode(const float *in, float *out) override {
        Ort::Value iv = Ort::Value::CreateTensor<float>(mem_, const_cast<float *>(in), in_n_, in_shape_.data(), in_shape_.size());
        Ort::Value ov = Ort::Value::CreateTensor<float>(mem_, out, out_n_, out_shape_.data(), out_shape_.size());
        const char *i = in_name_.c_str(), *o = out_name_.c_str();
        session_->Run(Ort::RunOptions{nullptr}, &i, &iv, 1, &o, &ov, 1);
    }

private:
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "envd"};
    Ort::MemoryInfo mem_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::unique_ptr<Ort::Session> session_;
    std::string in_name_, out_name_;
    std::vector<int64_t> in_shape_, out_shape_;
    size_t in_n_ = 0, out_n_ = 0;
};

}  // namespace

std::unique_ptr<ImageEncoder> make_ort_encoder(const EncoderOptions &opt) { return std::make_unique<OrtEncoder>(opt); }

bool encoder_available(const std::string &backend) {
    if (backend == "ort") return true;
#ifdef ROADSENSE_HAVE_NCNN
    if (backend == "ncnn") return true;
#endif
    return false;
}

std::unique_ptr<ImageEncoder> make_encoder(const EncoderOptions &opt) {
    if (opt.backend == "ort") return make_ort_encoder(opt);
#ifdef ROADSENSE_HAVE_NCNN
    if (opt.backend == "ncnn") return make_ncnn_encoder(opt);
#endif
    throw std::runtime_error("backend '" + opt.backend + "' not available in this build");
}

}  // namespace envd
}  // namespace roadsense
