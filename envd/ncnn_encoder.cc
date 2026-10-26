// ncnn + Vulkan encoder (see encoder.h). Models from tools/envd/make_ncnn.sh (pnnx): input blob
// in0, output out0 [D]. Precision and turnip workaround as in modeld/ncnn_backend.cc.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include <gpu.h>
#include <net.h>

#include "encoder.h"

namespace roadsense {
namespace envd {

namespace {

class NcnnEncoder : public ImageEncoder {
public:
    explicit NcnnEncoder(const EncoderOptions &opt) : size_(opt.size), out_n_(size_t(opt.embed_dim)) {
        if (opt.ncnn_flushall) {
            const char *t = getenv("TU_DEBUG");
            if (!t || !*t) setenv("TU_DEBUG", "flushall", 1);
            else if (!strstr(t, "flushall")) setenv("TU_DEBUG", (std::string(t) + ",flushall").c_str(), 1);
        }
        if (ncnn::get_gpu_count() <= 0) throw std::runtime_error("ncnn: no Vulkan device");
        const bool fp16 = opt.ncnn_fp16;
        net_.opt.use_vulkan_compute = true;
        net_.opt.use_fp16_packed = fp16;
        net_.opt.use_fp16_storage = fp16;
        net_.opt.use_fp16_arithmetic = fp16;
        net_.opt.use_bf16_storage = false;
        net_.opt.use_packing_layout = true;
        net_.opt.num_threads = opt.threads > 0 ? opt.threads : 1;
        const std::string ext = ".param";
        if (opt.model.size() <= ext.size() || opt.model.compare(opt.model.size() - ext.size(), ext.size(), ext) != 0)
            throw std::runtime_error("ncnn: model must be the .param file: " + opt.model);
        const std::string bin = opt.model.substr(0, opt.model.size() - ext.size()) + ".bin";
        if (net_.load_param(opt.model.c_str()) != 0) throw std::runtime_error("ncnn: cannot load " + opt.model);
        if (net_.load_model(bin.c_str()) != 0) throw std::runtime_error("ncnn: cannot load " + bin);
        in_.create(size_, size_, 3);
        in_.fill(0.0f);
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<float> tmp(out_n_);
        encode(nullptr, tmp.data());  // warm-up on zeros, checks the output size
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        const ncnn::GpuInfo &gi = ncnn::get_gpu_info(ncnn::get_default_gpu_index());
        fprintf(stderr, "envd: ncnn %s on %s (fp16 packed %d), TU_DEBUG=%s, warm-up %.0f ms\n", opt.model.c_str(),
                gi.device_name(), fp16 && gi.support_fp16_packed(), getenv("TU_DEBUG") ? getenv("TU_DEBUG") : "", ms);
    }
    ~NcnnEncoder() override {
        net_.clear();
        ncnn::destroy_gpu_instance();
    }
    const char *name() const override { return "ncnn"; }
    void encode(const float *in, float *out) override {
        const size_t plane = size_t(size_) * size_;
        if (in)
            for (int c = 0; c < 3; ++c) std::memcpy(in_.channel(c), in + c * plane, plane * sizeof(float));
        ncnn::Extractor ex = net_.create_extractor();
        if (ex.input("in0", in_) != 0) throw std::runtime_error("ncnn: bad input in0");
        ncnn::Mat o;
        const int r = ex.extract("out0", o);
        if (r != 0 || o.empty() || o.elemsize != 4) throw std::runtime_error("ncnn: extract out0 failed");
        if (o.total() != out_n_)
            throw std::runtime_error("ncnn: out0 has " + std::to_string(o.total()) + " values, expected " + std::to_string(out_n_));
        std::memcpy(out, o.data, out_n_ * sizeof(float));
    }

private:
    ncnn::Net net_;
    ncnn::Mat in_;
    int size_;
    size_t out_n_;
};

}  // namespace

std::unique_ptr<ImageEncoder> make_ncnn_encoder(const EncoderOptions &opt) { return std::make_unique<NcnnEncoder>(opt); }

}  // namespace envd
}  // namespace roadsense
