#include "ncnn_backend.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include <gpu.h>
#include <net.h>

#include "supercombo.h"

namespace roadsense {

namespace {

// features_buffer rows the graph reads: Gather(axis 1, indices -5, -10, ..., -40) of [1,99,512].
constexpr int kFeatRows = 8;
inline int feat_row(int k) { return sc::kFeatLen - 5 * (k + 1); }  // 94, 89, ..., 59

class NcnnBackend : public InferenceBackend {
public:
    explicit NcnnBackend(const SupercomboOptions &opt) {
        if (opt.ncnn_flushall) {  // must be set before ncnn creates the Vulkan instance (ncnn_backend.h)
            const char *t = getenv("TU_DEBUG");
            if (!t || !*t) setenv("TU_DEBUG", "flushall", 1);
            else if (!strstr(t, "flushall")) setenv("TU_DEBUG", (std::string(t) + ",flushall").c_str(), 1);
        }
        if (ncnn::get_gpu_count() <= 0) throw std::runtime_error("ncnn: no Vulkan device");
        const bool fp16 = opt.ncnn_fp16;
        net_.opt.use_vulkan_compute = true;
        net_.opt.use_fp16_packed = fp16;
        net_.opt.use_fp16_storage = fp16;      // not supported by turnip on the Adreno 630: ncnn drops it
        net_.opt.use_fp16_arithmetic = fp16;   // ditto
        net_.opt.use_bf16_storage = false;
        net_.opt.use_packing_layout = true;
        net_.opt.num_threads = opt.threads > 0 ? opt.threads : 1;

        std::string param = opt.model_path, bin;
        const std::string ext = ".param";
        if (param.size() > ext.size() && param.compare(param.size() - ext.size(), ext.size(), ext) == 0)
            bin = param.substr(0, param.size() - ext.size()) + ".bin";
        else
            throw std::runtime_error("ncnn: --model must be the .param file (weights: same stem .bin)");
        if (net_.load_param(param.c_str()) != 0) throw std::runtime_error("ncnn: cannot load " + param);
        if (net_.load_model(bin.c_str()) != 0) throw std::runtime_error("ncnn: cannot load " + bin);
        const auto &in = net_.input_names(), &out = net_.output_names();
        if (in.size() != 7 || out.size() != 1)
            throw std::runtime_error("ncnn: expected 7 inputs and 1 output (supercombo-ncnn.param)");

        in0_.create(sc::kImgW, sc::kImgH, sc::kImgC);
        in1_.create(sc::kImgW, sc::kImgH, sc::kImgC);
        in2_.create(sc::kDesireW, sc::kDesireLen);
        in3_.create(sc::kTrafficConventionLen);
        in4_.create(sc::kLatParamsLen);
        in5_.create(1, sc::kPrevCurvLen);
        in6_.create(sc::kFeatW, kFeatRows);
        for (ncnn::Mat *m : {&in0_, &in1_, &in2_, &in3_, &in4_, &in5_, &in6_}) m->fill(0.0f);

        // Warm-up: the first run builds the Vulkan pipelines and uploads the weights (~0.1-1.5 s).
        const auto t0 = std::chrono::steady_clock::now();
        std::string err;
        if (!extract(&err)) throw std::runtime_error("ncnn: warm-up run failed: " + err);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        const ncnn::GpuInfo &gi = ncnn::get_gpu_info(ncnn::get_default_gpu_index());
        const char *td = getenv("TU_DEBUG");
        fprintf(stderr, "modeld: ncnn on %s (fp16 packed %d, storage %d, arithmetic %d), TU_DEBUG=%s, warm-up %.0f ms\n",
                gi.device_name(), fp16 && gi.support_fp16_packed(), fp16 && gi.support_fp16_storage(),
                fp16 && gi.support_fp16_arithmetic(), td ? td : "", ms);
    }

    ~NcnnBackend() override {
        net_.clear();
        // ncnn's global Vulkan instance outlives the Net; release it so the driver shuts down cleanly.
        ncnn::destroy_gpu_instance();
    }

    const char *name() const override { return "ncnn"; }

    bool run(const std::array<const float *, 7> &in, float *out, std::string *err) override {
        const size_t plane = size_t(sc::kImgW) * sc::kImgH;
        for (int c = 0; c < sc::kImgC; ++c) {  // channel stride may be padded: copy per channel
            std::memcpy(in0_.channel(c), in[0] + c * plane, plane * sizeof(float));
            std::memcpy(in1_.channel(c), in[1] + c * plane, plane * sizeof(float));
        }
        std::memcpy(in2_.data, in[2], sizeof(float) * sc::kDesireLen * sc::kDesireW);
        std::memcpy(in3_.data, in[3], sizeof(float) * sc::kTrafficConventionLen);
        std::memcpy(in4_.data, in[4], sizeof(float) * sc::kLatParamsLen);
        std::memcpy(in5_.data, in[5], sizeof(float) * sc::kPrevCurvLen);
        for (int k = 0; k < kFeatRows; ++k)
            std::memcpy(in6_.row(k), in[6] + size_t(feat_row(k)) * sc::kFeatW, sizeof(float) * sc::kFeatW);
        if (!extract(err)) return false;
        std::memcpy(out, out_.data, sizeof(float) * sc::kOutputLen);
        return true;
    }

private:
    bool extract(std::string *err) {
        ncnn::Extractor ex = net_.create_extractor();
        const ncnn::Mat *ins[7] = {&in0_, &in1_, &in2_, &in3_, &in4_, &in5_, &in6_};
        static const char *names[7] = {"in0", "in1", "in2", "in3", "in4", "in5", "in6"};
        for (int i = 0; i < 7; ++i)
            if (ex.input(names[i], *ins[i]) != 0) {
                if (err) *err = std::string("ncnn: bad input ") + names[i];
                return false;
            }
        int r = ex.extract("out0", out_);
        if (r != 0 || out_.empty() || out_.total() != size_t(sc::kOutputLen) || out_.elemsize != 4) {
            if (err) *err = "ncnn: extract out0 failed (" + std::to_string(r) + ", " + std::to_string(out_.total()) + " values)";
            return false;
        }
        return true;
    }

    ncnn::Net net_;
    ncnn::Mat in0_, in1_, in2_, in3_, in4_, in5_, in6_, out_;
};

}  // namespace

std::unique_ptr<InferenceBackend> make_ncnn_backend(const SupercomboOptions &opt) {
    return std::make_unique<NcnnBackend>(opt);
}

}  // namespace roadsense
